/**
 * @file main.cpp
 * @brief 三轮全向底盘固件 v2 - ROS2 集成版
 * @description
 *   融合了 Gemini 蓝牙版本验证过的运动学符号约定 + 软启动逻辑,
 *   通过 UART2 (GPIO16/17) 与上位机 ROS2 桥接节点通信。
 *
 * @对外接口 (符合 ROS2 REP-103 右手系)
 *   - /cmd_vel 里:  vx > 0 表示前进,  vy > 0 表示左移,  wz > 0 表示左转(逆时针)
 *   - /odom   里:  同上
 *
 * @对内运动学
 *   在逆解前对 vx 和 wz 取反, 以对齐 Gemini 已验证的"前进/左转"符号约定。
 *   同样地, 正解出来的 vx 和 wz 也要取反再送出, 这样 ROS2 侧看到的就是标准 REP-103 方向。
 *
 * @协议
 *   RX (上位机 -> ESP32): v,<vx>,<vy>,<w>\n   单位 m/s, m/s, rad/s  (REP-103)
 *   TX (ESP32 -> 上位机): o,<vx>,<vy>,<w>\n   单位 m/s, m/s, rad/s  (REP-103)
 *
 * @调试
 *   Serial  (USB, GPIO1/3)     -> debug 打印
 *   Serial2 (UART2, GPIO16/17) -> 与 Xavier 通信, 经电平转换器
 */

#include <Arduino.h>
#include "config.h"
#include "driver/pcnt.h"

// ================================================================
// 通信对象
// ================================================================
HardwareSerial& RosSerial = Serial2;

// ================================================================
// 目标速度 (机体系, ROS2 REP-103 约定)
// ================================================================
volatile float target_vx = 0.0f;   // m/s,  +X 前进
volatile float target_vy = 0.0f;   // m/s,  +Y 左移
volatile float target_wz = 0.0f;   // rad/s, +Z 逆时针(左转)

// 软启动 (低通滤波) 后的速度
float smooth_vx = 0.0f;
float smooth_vy = 0.0f;
float smooth_wz = 0.0f;

// 低通滤波系数 (Gemini 验证过的值, 每 10ms 迭代 α=0.05 -> 时间常数 ~200ms)
#define SMOOTH_ALPHA   0.05f

// ================================================================
// 轮速 & 机体速度
// ================================================================
float target_wheel_rad_s[3] = {0.0f, 0.0f, 0.0f};
float actual_wheel_rad_s[3] = {0.0f, 0.0f, 0.0f};

float actual_vx = 0.0f;   // ROS2 REP-103
float actual_vy = 0.0f;
float actual_wz = 0.0f;

// PID 状态
float pid_integral[3] = {0.0f, 0.0f, 0.0f};
int   pwm_output[3]   = {0, 0, 0};

// 时间戳
unsigned long last_pid_time  = 0;
unsigned long last_odom_time = 0;
unsigned long last_cmd_time  = 0;

// 串口接收缓冲
char rx_buffer[SERIAL_RX_BUFFER_SIZE];
int  rx_index = 0;

// ================================================================
// 硬件初始化
// ================================================================
void initPCNT(pcnt_unit_t unit, int pin_a, int pin_b) {
    pcnt_config_t cfg = {
        .pulse_gpio_num = pin_a,
        .ctrl_gpio_num  = pin_b,
        .lctrl_mode     = PCNT_MODE_KEEP,
        .hctrl_mode     = PCNT_MODE_REVERSE,
        .pos_mode       = PCNT_COUNT_INC,
        .neg_mode       = PCNT_COUNT_DIS,
        .counter_h_lim  = PCNT_HIGH_LIMIT,
        .counter_l_lim  = PCNT_LOW_LIMIT,
        .unit           = unit,
        .channel        = PCNT_CHANNEL_0,
    };
    pcnt_unit_config(&cfg);
    pcnt_set_filter_value(unit, PCNT_FILTER_VALUE);
    pcnt_filter_enable(unit);
    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);
    pcnt_counter_resume(unit);
}

void initAllMotors() {
    pinMode(MOTOR_STBY_PIN, OUTPUT);
    digitalWrite(MOTOR_STBY_PIN, HIGH);

    pinMode(MOTOR_A_IN1, OUTPUT); pinMode(MOTOR_A_IN2, OUTPUT);
    pinMode(MOTOR_B_IN1, OUTPUT); pinMode(MOTOR_B_IN2, OUTPUT);
    pinMode(MOTOR_C_IN1, OUTPUT); pinMode(MOTOR_C_IN2, OUTPUT);

    ledcSetup(0, 20000, 10); ledcAttachPin(MOTOR_A_PWM, 0);
    ledcSetup(1, 20000, 10); ledcAttachPin(MOTOR_B_PWM, 1);
    ledcSetup(2, 20000, 10); ledcAttachPin(MOTOR_C_PWM, 2);
}

// 与 Gemini 蓝牙版完全一致的 setMotorPWM (已被实测验证)
void setMotorPWM(int idx, int pwm) {
    int p1, p2, ch;
    float scale = 1.0f;

    if (idx == 0)      { p1 = MOTOR_A_IN1; p2 = MOTOR_A_IN2; ch = 0; scale = MOTOR_A_SCALE; }
    else if (idx == 1) { p1 = MOTOR_B_IN1; p2 = MOTOR_B_IN2; ch = 1; scale = MOTOR_B_SCALE; }
    else               { p1 = MOTOR_C_IN1; p2 = MOTOR_C_IN2; ch = 2; scale = MOTOR_C_SCALE; }

    pwm = (int)(pwm * scale);

    // PWM=0 时直接拉低, 避免残留电流噪音
    if (pwm == 0) {
        digitalWrite(p1, LOW); digitalWrite(p2, LOW);
        ledcWrite(ch, 0);
        return;
    }

    // 死区判断 (在配平之后)
    if (abs(pwm) < MOTOR_DEADZONE_START) pwm = 0;

    // 限幅
    if (pwm >  PID_OUTPUT_LIMIT) pwm =  PID_OUTPUT_LIMIT;
    if (pwm < -PID_OUTPUT_LIMIT) pwm = -PID_OUTPUT_LIMIT;

    if (pwm > 0) {
        digitalWrite(p1, HIGH); digitalWrite(p2, LOW);
        ledcWrite(ch, pwm);
    } else if (pwm < 0) {
        digitalWrite(p1, LOW); digitalWrite(p2, HIGH);
        ledcWrite(ch, -pwm);
    } else {
        digitalWrite(p1, LOW); digitalWrite(p2, LOW);
        ledcWrite(ch, 0);
    }
}

void stopAllMotors() {
    for (int i = 0; i < 3; i++) {
        setMotorPWM(i, 0);
        pid_integral[i] = 0.0f;
        pwm_output[i]   = 0;
    }
    pcnt_counter_clear(PCNT_UNIT_A);
    pcnt_counter_clear(PCNT_UNIT_B);
    pcnt_counter_clear(PCNT_UNIT_C);
}

// ================================================================
// 运动学 (含坐标系翻转)
// ================================================================
//
// 【关键说明】
// Gemini 蓝牙测试证明: 当 target_vx = -0.3 时小车物理上前进。
// 说明 config.h 的内部坐标系和 ROS2 标准 REP-103 正好 X、Yaw 方向反了。
// 这里在逆/正解边界处统一翻转, 把"脏"隔离在运动学内部, 对外全部 REP-103。
//
// ROS2 (REP-103)                     内部运动学
//   vx > 0 前进     ─── 取反 ──►     vx_internal = -vx_ros
//   vy > 0 左移     ─── 直通 ──►     vy_internal =  vy_ros
//   wz > 0 左转     ─── 取反 ──►     wz_internal = -wz_ros
//

void inverseKinematics(float ros_vx, float ros_vy, float ros_wz, float out_wheel[3]) {
    float vx = -ros_vx;
    float vy =  ros_vy;
    float wz = -ros_wz;

    // vi = (1/r) * (-sin(αi)*vx + cos(αi)*vy + R*ω)
    // config.h 里 WHEEL_X_SIN_COEF 已经是 -sin(α), 直接乘加即可
    float vA_ms = (WHEEL_A_SIN_COEF * vx + WHEEL_A_COS_COEF * vy + ROBOT_RADIUS_M * wz);
    float vB_ms = (WHEEL_B_SIN_COEF * vx + WHEEL_B_COS_COEF * vy + ROBOT_RADIUS_M * wz);
    float vC_ms = (WHEEL_C_SIN_COEF * vx + WHEEL_C_COS_COEF * vy + ROBOT_RADIUS_M * wz);

    out_wheel[0] = vA_ms * INV_WHEEL_RADIUS;
    out_wheel[1] = vB_ms * INV_WHEEL_RADIUS;
    out_wheel[2] = vC_ms * INV_WHEEL_RADIUS;
}

// 正解 (轮速 -> 机体速度), 输出直接就是 ROS2 REP-103 方向
void forwardKinematics(const float wheel[3], float& ros_vx, float& ros_vy, float& ros_wz) {
    float vA = wheel[0] * WHEEL_RADIUS_M;
    float vB = wheel[1] * WHEEL_RADIUS_M;
    float vC = wheel[2] * WHEEL_RADIUS_M;

    // 内部坐标系下的正解 (120° 均布三轮解析解)
    float int_vx = (2.0f / 3.0f) * (WHEEL_A_SIN_COEF * vA + WHEEL_B_SIN_COEF * vB + WHEEL_C_SIN_COEF * vC);
    float int_vy = (2.0f / 3.0f) * (WHEEL_A_COS_COEF * vA + WHEEL_B_COS_COEF * vB + WHEEL_C_COS_COEF * vC);
    float int_wz = (vA + vB + vC) / (3.0f * ROBOT_RADIUS_M);

    // 翻回 ROS2 REP-103
    ros_vx = -int_vx;
    ros_vy =  int_vy;
    ros_wz = -int_wz;
}

// ================================================================
// 编码器 -> 轮子角速度 (rad/s)
// ================================================================
float readWheelSpeed(pcnt_unit_t unit, float dir_coef, float dt) {
    int16_t count = 0;
    pcnt_get_counter_value(unit, &count);
    pcnt_counter_clear(unit);

    const float pulses_per_rad = (float)ENCODER_PULSE_PER_WHEEL_REV / (2.0f * PI);
    return (count / pulses_per_rad) / dt * dir_coef * ENCODER_CALIB_FACTOR;
}

// ================================================================
// PID
// ================================================================
int computePID(int idx, float target, float current) {
    float error = target - current;
    pid_integral[idx] += error;

    if (pid_integral[idx] >  PID_INTEGRAL_LIMIT) pid_integral[idx] =  PID_INTEGRAL_LIMIT;
    if (pid_integral[idx] < -PID_INTEGRAL_LIMIT) pid_integral[idx] = -PID_INTEGRAL_LIMIT;

    // 目标接近零时清积分, 防止静止爬行
    if (fabs(target) < 0.1f && fabs(error) < 0.2f) pid_integral[idx] = 0.0f;

    float output = PID_VEL_KP * error + PID_VEL_KI * pid_integral[idx];
    return (int)output;
}

// ================================================================
// 串口协议解析 (非阻塞逐字节)
// ================================================================
void processRxByte(char c) {
    if (c == CMD_VEL_END_CHAR) {
        rx_buffer[rx_index] = '\0';

        if (rx_index > 2 && rx_buffer[0] == CMD_VEL_START_CHAR && rx_buffer[1] == CMD_VEL_SEPARATOR) {
            float vx, vy, wz;
            int parsed = sscanf(rx_buffer + 2, "%f,%f,%f", &vx, &vy, &wz);
            if (parsed == 3) {
                if (vx >  MAX_LINEAR_VEL_X) vx =  MAX_LINEAR_VEL_X;
                if (vx < -MAX_LINEAR_VEL_X) vx = -MAX_LINEAR_VEL_X;
                if (vy >  MAX_LINEAR_VEL_Y) vy =  MAX_LINEAR_VEL_Y;
                if (vy < -MAX_LINEAR_VEL_Y) vy = -MAX_LINEAR_VEL_Y;
                if (wz >  MAX_ANGULAR_VEL)  wz =  MAX_ANGULAR_VEL;
                if (wz < -MAX_ANGULAR_VEL)  wz = -MAX_ANGULAR_VEL;

                target_vx = vx;
                target_vy = vy;
                target_wz = wz;
                last_cmd_time = millis();
            }
        }
        rx_index = 0;
    } else if (c != '\r' && rx_index < SERIAL_RX_BUFFER_SIZE - 1) {
        rx_buffer[rx_index++] = c;
    } else if (rx_index >= SERIAL_RX_BUFFER_SIZE - 1) {
        rx_index = 0;
    }
}

void readSerial() {
    while (RosSerial.available()) {
        processRxByte((char)RosSerial.read());
    }
}

// ================================================================
// 里程计发送
// ================================================================
void sendOdometry() {
    RosSerial.printf("o,%.4f,%.4f,%.4f\n", actual_vx, actual_vy, actual_wz);
}

// ================================================================
// setup
// ================================================================
void setup() {
    Serial.begin(DEBUG_BAUDRATE);
    delay(200);
    Serial.println(">> ESP32 Chassis Firmware v2 Starting...");

    RosSerial.begin(UART_ROS_BAUDRATE, SERIAL_8N1, UART_ROS_RX_PIN, UART_ROS_TX_PIN);
    Serial.printf(">> UART2 @ %d bps, RX=GPIO%d, TX=GPIO%d\n",
                  UART_ROS_BAUDRATE, UART_ROS_RX_PIN, UART_ROS_TX_PIN);

    initAllMotors();
    initPCNT(PCNT_UNIT_A, ENCODER_A_A, ENCODER_A_B);
    initPCNT(PCNT_UNIT_B, ENCODER_B_A, ENCODER_B_B);
    initPCNT(PCNT_UNIT_C, ENCODER_C_A, ENCODER_C_B);

    stopAllMotors();

    unsigned long now = millis();
    last_pid_time  = now;
    last_odom_time = now;
    last_cmd_time  = now;

    Serial.println(">> Ready. Waiting for cmd_vel from ROS2...");
}

// ================================================================
// loop
// ================================================================
void loop() {
    readSerial();

    unsigned long now = millis();

    // cmd_vel 超时保护
    if (now - last_cmd_time > CMD_VEL_TIMEOUT_MS) {
        target_vx = 0.0f;
        target_vy = 0.0f;
        target_wz = 0.0f;
    }

    // PID 循环 @ 100Hz
    if (now - last_pid_time >= PID_SAMPLE_TIME_MS) {
        float dt = (now - last_pid_time) / 1000.0f;
        last_pid_time = now;

        // ---- 1) 停车快速路径 (Gemini 验证过, 立即归零) ----
        if (target_vx == 0.0f && target_vy == 0.0f && target_wz == 0.0f) {
            smooth_vx = 0.0f;
            smooth_vy = 0.0f;
            smooth_wz = 0.0f;

            setMotorPWM(0, 0);
            setMotorPWM(1, 0);
            setMotorPWM(2, 0);
            pid_integral[0] = pid_integral[1] = pid_integral[2] = 0.0f;

            pcnt_counter_clear(PCNT_UNIT_A);
            pcnt_counter_clear(PCNT_UNIT_B);
            pcnt_counter_clear(PCNT_UNIT_C);

            actual_vx = 0.0f;
            actual_vy = 0.0f;
            actual_wz = 0.0f;

            // 停车时也要发 odom 维持心跳
            if (now - last_odom_time >= ODOM_CALC_PERIOD_MS) {
                last_odom_time = now;
                sendOdometry();
            }
            return;
        }

        // ---- 2) 软启动 (一阶低通) ----
        smooth_vx = smooth_vx * (1.0f - SMOOTH_ALPHA) + target_vx * SMOOTH_ALPHA;
        smooth_vy = smooth_vy * (1.0f - SMOOTH_ALPHA) + target_vy * SMOOTH_ALPHA;
        smooth_wz = smooth_wz * (1.0f - SMOOTH_ALPHA) + target_wz * SMOOTH_ALPHA;

        // ---- 3) 逆解 (内部自动翻转 ROS2 -> Gemini 约定) ----
        inverseKinematics(smooth_vx, smooth_vy, smooth_wz, target_wheel_rad_s);

        // ---- 4) 读编码器 ----
        actual_wheel_rad_s[0] = readWheelSpeed(PCNT_UNIT_A, ENCODER_DIR_A, dt);
        actual_wheel_rad_s[1] = readWheelSpeed(PCNT_UNIT_B, ENCODER_DIR_B, dt);
        actual_wheel_rad_s[2] = readWheelSpeed(PCNT_UNIT_C, ENCODER_DIR_C, dt);

        // ---- 5) PID ----
        for (int i = 0; i < 3; i++) {
            pwm_output[i] = computePID(i, target_wheel_rad_s[i], actual_wheel_rad_s[i]);
            setMotorPWM(i, pwm_output[i]);
        }

        // ---- 6) 正解 (内部自动翻转回 ROS2) ----
        forwardKinematics(actual_wheel_rad_s, actual_vx, actual_vy, actual_wz);
    }

    // 里程计发送 @ 50Hz
    if (now - last_odom_time >= ODOM_CALC_PERIOD_MS) {
        last_odom_time = now;
        sendOdometry();
    }
}
