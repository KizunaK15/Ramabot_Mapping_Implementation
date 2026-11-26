#!/usr/bin/env python3
import math
import time
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw: float):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class MappingOdomBridge(Node):
    def __init__(self):
        super().__init__('mapping_odom_bridge')
        self.declare_parameter('serial_port', '/dev/ttyUSB1')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('invert_yaw', False)
        self.declare_parameter('invert_linear', False)
        self.declare_parameter('yaw_scale', 1.0)
        self.declare_parameter('linear_scale', 1.0)
        self.declare_parameter('wheel_radius', 0.0675)
        self.declare_parameter('ticks_per_rev', 1189.64)

        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.invert_yaw = self.get_parameter('invert_yaw').get_parameter_value().bool_value
        self.invert_linear = self.get_parameter('invert_linear').get_parameter_value().bool_value
        self.yaw_scale = self.get_parameter('yaw_scale').get_parameter_value().double_value
        self.linear_scale = self.get_parameter('linear_scale').get_parameter_value().double_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').get_parameter_value().double_value

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.rx_buffer = ""

        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = baud
        self.ser.timeout = 0.02
        self.ser.dsrdtr = False
        self.ser.rtscts = False
        self.ser.dtr = False
        self.ser.rts = False
        self.ser.open()
        self.ser.reset_input_buffer()

        self.last_log_time = time.time()
        self.get_logger().info(
            f'Terhubung ke M5Stack Core Grey di {port} @ {baud} bps (Baseline Clean, invert_yaw={self.invert_yaw})'
        )

        self.odom_pub = self.create_publisher(Odometry, '/odom', 20)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 20)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.reset_srv = self.create_service(Trigger, '/reset_odom', self.reset_odom_cb)

        self.timer = self.create_timer(0.01, self.read_serial_loop)

    def cmd_vel_cb(self, msg: Twist):
        cmd_str = f"CMD:{msg.linear.x:.3f},{msg.angular.z:.3f}\n"
        try:
            self.ser.write(cmd_str.encode('utf-8'))
        except serial.SerialException as e:
            self.get_logger().error(f'Gagal kirim CMD serial: {e}')

    def reset_odom_cb(self, request, response):
        try:
            self.ser.write(b"RESET\n")
            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0
            self.rx_buffer = ""
            response.success = True
            response.message = "Odometri & gyro heading M5Stack di-reset ke 0"
        except serial.SerialException as e:
            response.success = False
            response.message = str(e)
        return response

    def read_serial_loop(self):
        try:
            if self.ser.in_waiting > 0:
                chunk = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                self.rx_buffer += chunk

            if '\n' not in self.rx_buffer:
                return

            lines = self.rx_buffer.split('\n')
            self.rx_buffer = lines[-1]

            latest_parts = None
            for raw_line in lines[:-1]:
                line = raw_line.strip()
                if line.startswith('ODO:'):
                    parts = line[4:].split(',')
                    if len(parts) >= 7:
                        latest_parts = parts

            if latest_parts is None:
                return

            now_sec = time.time()
            raw_x = float(latest_parts[0]) * self.linear_scale
            raw_y = float(latest_parts[1]) * self.linear_scale
            theta = float(latest_parts[2]) * self.yaw_scale
            vx = float(latest_parts[3]) * self.linear_scale
            vth = float(latest_parts[4]) * self.yaw_scale
            ticks_l = float(latest_parts[5])
            ticks_r = float(latest_parts[6])

            if self.invert_linear:
                raw_x = -raw_x
                raw_y = -raw_y
                vx = -vx

            if self.invert_yaw:
                theta = -theta
                vth = -vth

            self.x = raw_x
            self.y = raw_y
            self.theta = normalize_angle(theta)

            if now_sec - self.last_log_time >= 2.0:
                self.last_log_time = now_sec
                self.get_logger().info(
                    f'ODOM -> x={self.x:.3f}m, y={self.y:.3f}m, yaw={math.degrees(self.theta):.1f} deg, ticks=({int(ticks_l)},{int(ticks_r)})'
                )

            now = self.get_clock().now().to_msg()
            qx, qy, qz, qw = yaw_to_quaternion(self.theta)

            tf_msg = TransformStamped()
            tf_msg.header.stamp = now
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = self.x
            tf_msg.transform.translation.y = self.y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation.x = qx
            tf_msg.transform.rotation.y = qy
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf_msg)

            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = self.odom_frame
            odom.child_frame_id = self.base_frame
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.orientation.x = qx
            odom.pose.pose.orientation.y = qy
            odom.pose.pose.orientation.z = qz
            odom.pose.pose.orientation.w = qw
            odom.pose.covariance = [
                0.01, 0.0,  0.0,  0.0,  0.0,  0.0,
                0.0,  0.01, 0.0,  0.0,  0.0,  0.0,
                0.0,  0.0,  1e6,  0.0,  0.0,  0.0,
                0.0,  0.0,  0.0,  1e6,  0.0,  0.0,
                0.0,  0.0,  0.0,  0.0,  1e6,  0.0,
                0.0,  0.0,  0.0,  0.0,  0.0,  0.001
            ]
            odom.twist.twist.linear.x = vx
            odom.twist.twist.angular.z = vth
            odom.twist.covariance = odom.pose.covariance
            self.odom_pub.publish(odom)

            wheel_l = (ticks_l / self.ticks_per_rev) * 2.0 * math.pi
            wheel_r = (ticks_r / self.ticks_per_rev) * 2.0 * math.pi
            if self.invert_linear:
                wheel_l = -wheel_l
                wheel_r = -wheel_r
            js = JointState()
            js.header.stamp = now
            js.name = ["Wheel1", "Wheel2"]
            js.position = [wheel_l, wheel_r]
            self.joint_pub.publish(js)
        except Exception as e:
            self.get_logger().warn(f'Serial parse warning: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = MappingOdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser.is_open:
            try:
                node.ser.write(b"CMD:0.000,0.000\n")
                node.ser.close()
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
