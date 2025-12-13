#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import utm
from datetime import datetime, timezone

from gps_interfaces.msg import Customgps


class GPSDriver(Node):
    def __init__(self):
        super().__init__('gps_driver')
        
        # Declare and get the port parameter
        self.declare_parameter('port', '/dev/ttyUSB0')
        port = self.get_parameter('port').get_parameter_value().string_value
        
        # Set up serial connection
        self.serial_port = serial.Serial(port, baudrate=4800, timeout=1)
        self.get_logger().info(f'Connected to GPS on {port}')
        
        # Publisher
        self.publisher = self.create_publisher(Customgps, '/gps', 10)
        
        # Timer to read serial data
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
    
    def parse_gpgga(self, sentence):
        """Parse a GPGGA sentence and return a dictionary of values."""
        parts = sentence.split(',')
        
        if len(parts) < 15 or parts[0] != '$GPGGA':
            return None
        
        try:
            # Parse UTC time (HHMMSS.sss)
            utc_raw = parts[1]
            if utc_raw:
                hours = int(utc_raw[0:2])
                minutes = int(utc_raw[2:4])
                seconds = float(utc_raw[4:])
            else:
                hours, minutes, seconds = 0, 0, 0.0
            
            # Parse latitude (DDMM.MMMMM)
            lat_raw = parts[2]
            lat_dir = parts[3]
            if lat_raw:
                lat_deg = int(lat_raw[0:2])
                lat_min = float(lat_raw[2:])
                latitude = lat_deg + lat_min / 60.0
                if lat_dir == 'S':
                    latitude = -latitude
            else:
                latitude = 0.0
            
            # Parse longitude (DDDMM.MMMMM)
            lon_raw = parts[4]
            lon_dir = parts[5]
            if lon_raw:
                lon_deg = int(lon_raw[0:3])
                lon_min = float(lon_raw[3:])
                longitude = lon_deg + lon_min / 60.0
                if lon_dir == 'W':
                    longitude = -longitude
            else:
                longitude = 0.0
            
            # Parse altitude
            altitude = float(parts[9]) if parts[9] else 0.0
            
            # Parse HDOP
            hdop = float(parts[8]) if parts[8] else 0.0
            
            return {
                'utc_hours': hours,
                'utc_minutes': minutes,
                'utc_seconds': seconds,
                'latitude': latitude,
                'longitude': longitude,
                'altitude': altitude,
                'hdop': hdop
            }
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f'Error parsing GPGGA: {e}')
            return None
    
    def timer_callback(self):
        try:
            line = self.serial_port.readline().decode('ascii', errors='ignore').strip()
            
            if line.startswith('$GPGGA'):
                data = self.parse_gpgga(line)
                
                if data and data['latitude'] != 0.0:
                    # Convert to UTM
                    utm_coords = utm.from_latlon(data['latitude'], data['longitude'])
                    
                    # Create message
                    msg = Customgps()
                    
                    # Header
                    msg.header.frame_id = 'GPS1_Frame'
                    
                    # Convert UTC to epoch time
                    now = datetime.now(timezone.utc)
                    gps_time = now.replace(
                        hour=data['utc_hours'],
                        minute=data['utc_minutes'],
                        second=int(data['utc_seconds']),
                        microsecond=int((data['utc_seconds'] % 1) * 1e6)
                    )
                    epoch_time = gps_time.timestamp()
                    msg.header.stamp.sec = int(epoch_time)
                    msg.header.stamp.nanosec = int((epoch_time % 1) * 1e9)
                    
                    # GPS data
                    msg.latitude = data['latitude']
                    msg.longitude = data['longitude']
                    msg.altitude = data['altitude']
                    msg.utm_easting = utm_coords[0]
                    msg.utm_northing = utm_coords[1]
                    msg.zone = utm_coords[2]
                    msg.letter = utm_coords[3]
                    msg.hdop = data['hdop']
                    msg.gpgga_read = line
                    
                    self.publisher.publish(msg)
                    self.get_logger().debug(f'Published GPS: {data["latitude"]}, {data["longitude"]}')
                    
        except serial.SerialException as e:
            self.get_logger().error(f'Serial error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = GPSDriver()
    
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
