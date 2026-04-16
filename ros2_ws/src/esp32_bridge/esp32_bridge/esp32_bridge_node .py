#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
esp32_bridge_node.py
====================
ROS2 <-> ESP32 串口桥接节点

功能:
  1. 订阅 /cmd_vel (geometry_msgs/Twist), 编码成 "v,vx,vy,w\\n" 发给 ESP32
  2. 从串口读 "o,vx,vy,w\\n", 在节点内积分得到 x/y/theta
  3. 发布 /odom (nav_msgs/Odometry)
  4. 广播 TF:  odom -> base_link

协议 (与 config.h 一致):
  RX (bridge -> ESP32): v,<vx>,<vy>,<w>\\n     单位 m/s, m/s, rad/s
  TX (ESP32 -> bridge): o,<vx>,<vy>,<w>\\n     单位 m/s, m/s, rad/s
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import serial

from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """2D 偏航角 -> 四元数"""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class Esp32BridgeNode(Node):
    def __init__(self):
        super().__init__('esp32_bridge')

        # ---------------- 参数声明 ----------------
        self.declare_parameter('port', '/dev/ttyTHS0')        # Xavier 载板 UART 设备名, 按实际改
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('publish_tf', True)            # 是否广播 odom->base_link
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('cmd_timeout_sec', 0.5)        # 上位机侧超时
        self.declare_parameter('max_linear', 0.5)             # 速度限幅 (与 config.h 对齐)
        self.declare_parameter('max_angular', 1.0)
        # ---- 里程计标度校准 (只影响 /odom 积分, 不影响 cmd_vel 下发) ----
        # 用法: 让小车跑实际 2.0m, 看 /odom 报告距离. 系数 = 实际 / odom报告
        self.declare_parameter('linear_scale', 1.0)           # 线速度积分校准
        self.declare_parameter('angular_scale', 1.0)          # 角速度积分校准

        self.port         = self.get_parameter('port').value
        self.baudrate     = self.get_parameter('baudrate').value
        self.publish_tf   = self.get_parameter('publish_tf').value
        self.odom_frame   = self.get_parameter('odom_frame').value
        self.base_frame   = self.get_parameter('base_frame').value
        self.max_linear   = float(self.get_parameter('max_linear').value)
        self.max_angular  = float(self.get_parameter('max_angular').value)
        self.linear_scale  = float(self.get_parameter('linear_scale').value)
        self.angular_scale = float(self.get_parameter('angular_scale').value)
        self.get_logger().info(
            f'odom calibration: linear_scale={self.linear_scale}, angular_scale={self.angular_scale}'
        )

        # ---------------- 里程计状态 ----------------
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0
        self.vx    = 0.0
        self.vy    = 0.0
        self.wz    = 0.0
        self.last_odom_time = None   # 首帧时初始化
        self.state_lock = threading.Lock()
        self.cmd_count  = 0
        self.odom_count = 0

        # ---------------- 串口打开 ----------------
        self.ser = None
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.05,        # 50ms read timeout, 避免读线程空转阻塞退出
                write_timeout=0.1,
            )
            self.get_logger().info(f'Opened serial {self.port} @ {self.baudrate}')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial {self.port}: {e}')
            raise

        # 清空可能残留的缓冲
        try:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
        except Exception:
            pass

        # ---------------- ROS 接口 ----------------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, qos
        )
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---------------- 串口读线程 ----------------
        self.rx_buffer = ''
        self.running = True
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()

        # 状态打印定时器 (每 2s)
        self.create_timer(2.0, self._status_print)

        self.get_logger().info('esp32_bridge node ready.')

    # ================================================================
    # cmd_vel 回调 -> 串口发送
    # ================================================================
    def cmd_vel_callback(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        # 限幅
        vx = max(-self.max_linear,  min(self.max_linear,  vx))
        vy = max(-self.max_linear,  min(self.max_linear,  vy))
        wz = max(-self.max_angular, min(self.max_angular, wz))

        line = f'v,{vx:.4f},{vy:.4f},{wz:.4f}\n'
        try:
            self.ser.write(line.encode('ascii'))
            self.cmd_count += 1
            if self.cmd_count <= 3:
                self.get_logger().info(f'sent -> {line.strip()}')
        except Exception as e:
            self.get_logger().warn(f'Serial write failed: {e}')

    # ================================================================
    # 串口读线程 - 逐行解析 o,vx,vy,w
    # ================================================================
    def _rx_loop(self):
        while self.running:
            try:
                data = self.ser.read(128)
            except Exception as e:
                self.get_logger().warn(f'Serial read failed: {e}')
                time.sleep(0.1)
                continue

            if not data:
                continue

            try:
                self.rx_buffer += data.decode('ascii', errors='ignore')
            except Exception:
                self.rx_buffer = ''
                continue

            # 按 \n 切分
            while '\n' in self.rx_buffer:
                line, self.rx_buffer = self.rx_buffer.split('\n', 1)
                line = line.strip()
                if line.startswith('o,'):
                    self._handle_odom_line(line)

    def _handle_odom_line(self, line: str):
        # line = "o,vx,vy,w"
        try:
            parts = line.split(',')
            if len(parts) != 4:
                return
            vx = float(parts[1])
            vy = float(parts[2])
            wz = float(parts[3])
        except ValueError:
            return

        now = self.get_clock().now()
        with self.state_lock:
            self.odom_count += 1
            if self.odom_count <= 3:
                self.get_logger().info(f'recv <- o,{vx:.4f},{vy:.4f},{wz:.4f}')

            # 首帧: 初始化时间戳, 不积分
            if self.last_odom_time is None:
                self.last_odom_time = now
                self.vx, self.vy, self.wz = vx, vy, wz
                self._publish_odom(now)
                return

            dt = (now - self.last_odom_time).nanoseconds * 1e-9
            self.last_odom_time = now

            # 异常 dt (跳变/第一次), 只更新速度不积分位姿
            if dt <= 0.0 or dt > 0.5:
                self.vx, self.vy, self.wz = vx, vy, wz
                self._publish_odom(now)
                return

            # 应用里程计标度校准 (只影响位置积分, 不影响 PID 闭环)
            vx_cal = vx * self.linear_scale
            vy_cal = vy * self.linear_scale
            wz_cal = wz * self.angular_scale

            # 机体系速度 -> 世界系 (三轮全向, 支持 vy 非零)
            cos_th = math.cos(self.theta)
            sin_th = math.sin(self.theta)
            dx_world = (vx_cal * cos_th - vy_cal * sin_th) * dt
            dy_world = (vx_cal * sin_th + vy_cal * cos_th) * dt
            dth      = wz_cal * dt

            self.x     += dx_world
            self.y     += dy_world
            self.theta += dth
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

            # 发布的 twist 用校准后的值, 这样 RTAB-Map / Nav2 看到的速度也是校准过的
            self.vx, self.vy, self.wz = vx_cal, vy_cal, wz_cal
            self._publish_odom(now)

    # ================================================================
    # 发布 /odom + TF
    # ================================================================
    def _publish_odom(self, stamp):
        q = yaw_to_quaternion(self.theta)
        stamp_msg = stamp.to_msg()

        odom = Odometry()
        odom.header.stamp = stamp_msg
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id  = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x  = self.vx
        odom.twist.twist.linear.y  = self.vy
        odom.twist.twist.angular.z = self.wz

        # 简单的协方差 (纯里程计可信度一般, 让 RTAB-Map / Nav2 有个参考)
        for i in range(36):
            odom.pose.covariance[i]  = 0.0
            odom.twist.covariance[i] = 0.0
        odom.pose.covariance[0]  = 0.01   # x
        odom.pose.covariance[7]  = 0.01   # y
        odom.pose.covariance[35] = 0.02   # yaw
        odom.twist.covariance[0]  = 0.01
        odom.twist.covariance[7]  = 0.01
        odom.twist.covariance[35] = 0.02

        self.odom_pub.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp_msg
            t.header.frame_id = self.odom_frame
            t.child_frame_id  = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = q
            self.tf_broadcaster.sendTransform(t)

    # ================================================================
    # 调试输出
    # ================================================================
    def _status_print(self):
        with self.state_lock:
            self.get_logger().info(
                f'pose=({self.x:+.3f},{self.y:+.3f},{math.degrees(self.theta):+.1f}°)  '
                f'vel=({self.vx:+.3f},{self.vy:+.3f},{self.wz:+.3f})  '
                f'tx={self.cmd_count} rx={self.odom_count}'
            )

    # ================================================================
    # 清理
    # ================================================================
    def destroy_node(self):
        self.running = False
        try:
            if self.ser and self.ser.is_open:
                # 安全停止
                self.ser.write(b'v,0.0,0.0,0.0\n')
                time.sleep(0.05)
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Esp32BridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
