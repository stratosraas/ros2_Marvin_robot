#ifndef CONFIG_H
#define CONFIG_H

#include <math.h>





// 右前轮电机 A
#define MOTOR_A_PWM     23  // A轮PWM速度控制
#define MOTOR_A_IN1     22  // A轮方向控制线1
#define MOTOR_A_IN2     21  // A轮方向控制线2

// 左前轮电机 B
#define MOTOR_B_PWM     19  // B轮PWM速度控制
#define MOTOR_B_IN1     5   // B轮方向控制线1
#define MOTOR_B_IN2     18  // B轮方向控制线2

// 后轮电机 C
#define MOTOR_C_PWM     33  // C轮PWM速度控制
#define MOTOR_C_IN1     14  // C轮方向控制线1
#define MOTOR_C_IN2     27  // C轮方向控制线2

// 驱动器公共使能引脚 (STBY)
#define MOTOR_STBY_PIN  4   // 高电平使能所有电机驱动



// 电机 A（右前轮）
#define ENCODER_A_A      34  // PCNT单元0
#define ENCODER_A_B      35

// 电机 B（左前轮）
#define ENCODER_B_A      36  // PCNT单元1
#define ENCODER_B_B      39

// 电机 C（后轮）
#define ENCODER_C_A      25  // PCNT单元2
#define ENCODER_C_B      26


// PCNT硬件编码器配置
#define USE_HARDWARE_ENCODER    1
#define PCNT_UNIT_A  PCNT_UNIT_0
#define PCNT_UNIT_B  PCNT_UNIT_1
#define PCNT_UNIT_C  PCNT_UNIT_2

#define PCNT_HIGH_LIMIT  32767   
#define PCNT_LOW_LIMIT   -32768   // PCNT最大脉冲数为32767
#define PCNT_FILTER_VALUE 50      // 滤波脉冲数，防止抖动



#define ENCODER_PPR             500     // 编码器每转物理线数
#define GEAR_RATIO              30.0    // 减速比（电机规格）

// ESP32 PCNT是2倍频
#define ENCODER_PULSE_PER_MOTOR_REV   (ENCODER_PPR * 2)  // 1000脉冲/电机轴每转
#define ENCODER_PULSE_PER_WHEEL_REV   (ENCODER_PULSE_PER_MOTOR_REV * GEAR_RATIO) // 30000脉冲/轮子每转

// 校准系数（如果后续测试有微小偏差可调整）
#define ENCODER_CALIB_FACTOR    1.0     // 校准系数（实测30000，理论30000，所以是1.0）

// 方向校准系数（基于测试结果）
#define ENCODER_DIR_A           1.0    // A电机：正转时计数为负，所以乘以
#define ENCODER_DIR_B           1.0    // B电机：正转时计数为负，所以乘以
#define ENCODER_DIR_C           1.0     // C电机：正转时计数为正，所以乘以1


// PID控制器初始参数
#define PID_SAMPLE_TIME_MS      10      // 100Hz PID计算
#define PID_VEL_KP              550    // 比例系数
#define PID_VEL_KI              15    // 积分系数
#define PID_INTEGRAL_LIMIT      1000.0  // 积分限幅
#define PID_OUTPUT_LIMIT        1023    // PWM输出限幅




// 轮子半径
#define WHEEL_RADIUS_MM     30.0    // mm
#define WHEEL_RADIUS_M      0.03   // m

#define ROBOT_RADIUS_MM     100.0    // mm  
#define ROBOT_RADIUS_M      0.10     // m


// 采用标准120°三轮布局（弧度制），+X 朝前（camera facing）
#define WHEEL_ANGLE_A      (300.0 * PI / 180.0)    // 右前轮 A = 300°（-60°）
#define WHEEL_ANGLE_B      (60.0  * PI / 180.0)    // 左前轮 B = 60°
#define WHEEL_ANGLE_C      (180.0 * PI / 180.0)    // 后轮 C = 180°


// 运动学逆解矩阵系数预计算
// 对于三轮全向：轮速 = (1/r) * [ -sin(θ)  cos(θ)  R ] * [vx, vy, ω]^T00000000

#define WHEEL_A_SIN_COEF    (-sin(WHEEL_ANGLE_A))  // sin(300°) = -√3/2
#define WHEEL_A_COS_COEF    (cos(WHEEL_ANGLE_A))   // cos(300°) = 1/2

#define WHEEL_B_SIN_COEF    (-sin(WHEEL_ANGLE_B))  // sin(60°) = √3/2
#define WHEEL_B_COS_COEF    (cos(WHEEL_ANGLE_B))   // cos(60°) = 1/2

#define WHEEL_C_SIN_COEF    (-sin(WHEEL_ANGLE_C))  // sin(180°) = 0
#define WHEEL_C_COS_COEF    (cos(WHEEL_ANGLE_C))   // cos(180°) = -1

// 运动学正解矩阵系数预计算（里程计用）
// 用于从轮速计算机体速度
#define INV_WHEEL_RADIUS    (1.0 / WHEEL_RADIUS_M)
#define INV_ROBOT_RADIUS    (1.0 / ROBOT_RADIUS_M)


// 主控制循环
#define CONTROL_LOOP_FREQ       100     // Hz
#define CONTROL_LOOP_PERIOD_MS 10 // ms


// 里程计计算频率
#define ODOM_CALC_FREQ          50      // Hz (比控制循环低，节省CPU)
#define ODOM_CALC_PERIOD_MS     20      // ms

// 状态发布频率
#define STATUS_PUBLISH_FREQ     10      // Hz
#define STATUS_PUBLISH_PERIOD_MS 100    // ms



// 机器人最大速度（与Nav2一致）
#define MAX_LINEAR_VEL_X        0.5     // m/s
#define MAX_LINEAR_VEL_Y        0.5     // m/s
#define MAX_ANGULAR_VEL         1.0     // rad/s

// 电机转速限制
/*#define MOTOR_SAFE_RPM           163.0       // 安全运行转速*/

/*#define MAX_WHEEL_RAD_S         (MOTOR_SAFE_RPM * 2 * PI / 60.0)  // 约17.06 rad/s*/

#define MOTOR_DEADZONE_START    60      // PWM值（采用反转的开始值，确保两个方向都可靠）
#define MOTOR_DEADZONE_END      62      // PWM值（两个方向完全克服死区相同）
#define UART_ROS_RX_PIN         16
#define UART_ROS_TX_PIN         17
#define UART_ROS_BAUDRATE       115200

// 命令协议
#define CMD_VEL_START_CHAR      'v'
#define CMD_VEL_SEPARATOR       ','
#define CMD_VEL_END_CHAR        '\n'

// 里程计协议
#define ODOM_START_CHAR         'o'
#define ODOM_SEPARATOR          ','
#define ODOM_END_CHAR           '\n'

// 缓冲区
#define SERIAL_RX_BUFFER_SIZE   256
#define SERIAL_TX_BUFFER_SIZE   256

#define CMD_VEL_TIMEOUT_MS      500     // 0.5秒无指令自动停止
#define WATCHDOG_TIMEOUT_MS     1000    // 看门狗超时

#define DEBUG_BAUDRATE          115200
#define DEBUG_LEVEL_INFO        3
#define CURRENT_DEBUG_LEVEL     DEBUG_LEVEL_INFO

#define MOTOR_A_SCALE   1.0   // 给弱的 A 轮加 5% 的力
#define MOTOR_B_SCALE   1.0   // 给强的 B 轮减 5% 的力
#define MOTOR_C_SCALE   1.0    // C 轮保持不动

#endif /* CONFIG_H */
