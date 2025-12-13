#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import math

from sensor_msgs.msg import Imu, MagneticField


class VectorNavDriver(Node):
    def __init__(self):
        super().__init__('imu_driver')
        
        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('frame_id', 'imu_link')
        
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        
        # Serial connection
        self.serial_port = serial.Serial(port, baudrate=baud, timeout=1)
        self.get_logger().info(f'Connected to VectorNav IMU on {port} at {baud} baud')
        
        # Configure VN-100 to output VNYMR (Yaw, Pitch, Roll, Mag, Accel, Gyro)
        # You may need to adjust this based on your specific needs
        self.configure_imu()
        
        # Publishers
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        self.mag_pub = self.create_publisher(MagneticField, '/mag', 10)
        
        # Timer for reading data
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100 Hz
    
    def configure_imu(self):
        """Configure VectorNav to output desired data."""
        # Request VNYMR output at 40 Hz (adjust as needed)
        # $VNWRG,07,40*XX - sets async data output rate
        # The VN-100 may need specific configuration commands
        try:
            # Example: Set output to VNYMR format
            self.serial_port.write(b'$VNWRG,06,14*XX\r\n')  # Output VNYMR
            self.serial_port.write(b'$VNWRG,07,40*XX\r\n')  # 40 Hz rate
        except Exception as e:
            self.get_logger().warn(f'Could not configure IMU: {e}')
    
    def parse_vnymr(self, line):
        """
        Parse VNYMR sentence from VectorNav.
        Format: $VNYMR,yaw,pitch,roll,mag_x,mag_y,mag_z,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z*checksum
        """
        try:
            # Remove checksum
            if '*' in line:
                line = line.split('*')[0]
            
            parts = line.split(',')
            
            if parts[0] != '$VNYMR' or len(parts) < 13:
                return None
            
            return {
                'yaw': float(parts[1]),       # degrees
                'pitch': float(parts[2]),     # degrees
                'roll': float(parts[3]),      # degrees
                'mag_x': float(parts[4]),     # Gauss
                'mag_y': float(parts[5]),     # Gauss
                'mag_z': float(parts[6]),     # Gauss
                'accel_x': float(parts[7]),   # m/s^2
                'accel_y': float(parts[8]),   # m/s^2
                'accel_z': float(parts[9]),   # m/s^2
                'gyro_x': float(parts[10]),   # rad/s
                'gyro_y': float(parts[11]),   # rad/s
                'gyro_z': float(parts[12]),   # rad/s
            }
        except (ValueError, IndexError) as e:
            self.get_logger().debug(f'Parse error: {e}')
            return None
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles (in radians) to quaternion."""
        cr = math.cos(roll / 2)
        sr = math.sin(roll / 2)
        cp = math.cos(pitch / 2)
        sp = math.sin(pitch / 2)
        cy = math.cos(yaw / 2)
        sy = math.sin(yaw / 2)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return (x, y, z, w)
    
    def timer_callback(self):
        try:
            line = self.serial_port.readline().decode('ascii', errors='ignore').strip()
            
            if line.startswith('$VNYMR'):
                data = self.parse_vnymr(line)
                
                if data:
                    now = self.get_clock().now().to_msg()
                    
                    # Create IMU message
                    imu_msg = Imu()
                    imu_msg.header.stamp = now
                    imu_msg.header.frame_id = self.frame_id
                    
                    # Convert Euler to quaternion (degrees to radians)
                    roll_rad = math.radians(data['roll'])
                    pitch_rad = math.radians(data['pitch'])
                    yaw_rad = math.radians(data['yaw'])
                    
                    qx, qy, qz, qw = self.euler_to_quaternion(roll_rad, pitch_rad, yaw_rad)
                    imu_msg.orientation.x = qx
                    imu_msg.orientation.y = qy
                    imu_msg.orientation.z = qz
                    imu_msg.orientation.w = qw
                    
                    # Angular velocity (already in rad/s)
                    imu_msg.angular_velocity.x = data['gyro_x']
                    imu_msg.angular_velocity.y = data['gyro_y']
                    imu_msg.angular_velocity.z = data['gyro_z']
                    
                    # Linear acceleration (m/s^2)
                    imu_msg.linear_acceleration.x = data['accel_x']
                    imu_msg.linear_acceleration.y = data['accel_y']
                    imu_msg.linear_acceleration.z = data['accel_z']
                    
                    # Covariance (set to -1 if unknown, or populate if known)
                    imu_msg.orientation_covariance[0] = -1.0
                    imu_msg.angular_velocity_covariance[0] = -1.0
                    imu_msg.linear_acceleration_covariance[0] = -1.0
                    
                    self.imu_pub.publish(imu_msg)
                    
                    # Create MagneticField message
                    mag_msg = MagneticField()
                    mag_msg.header.stamp = now
                    mag_msg.header.frame_id = self.frame_id
                    
                    # Convert Gauss to Tesla (1 Gauss = 1e-4 Tesla)
                    mag_msg.magnetic_field.x = data['mag_x'] * 1e-4
                    mag_msg.magnetic_field.y = data['mag_y'] * 1e-4
                    mag_msg.magnetic_field.z = data['mag_z'] * 1e-4
                    
                    self.mag_pub.publish(mag_msg)
                    
                    self.get_logger().debug(f'Published IMU data: yaw={data["yaw"]:.2f}')
                    
        except serial.SerialException as e:
            self.get_logger().error(f'Serial error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = VectorNavDriver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
