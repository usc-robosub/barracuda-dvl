#!/usr/bin/env python

"""
Water Linked DVL A50 ROS Driver

This ROS node interfaces with the Water Linked DVL A50 using the TCP JSON protocol.
It publishes odometry and pose information and provides a service to control acoustics.

Author: Generated for Water Linked DVL A50 integration
License: MIT
"""

import rospy
import socket
import json
import threading
import time
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, Point, Quaternion, Vector3
from sensor_msgs.msg import Range
from std_srvs.srv import SetBool, SetBoolResponse
import tf2_ros
import tf.transformations
import numpy as np


class WaterLinkedDVLDriver:
    """ROS driver for Water Linked DVL A50"""
    
    def __init__(self):
        rospy.init_node('waterlinked_dvl_driver', anonymous=False)
        
        # Parameters
        self.dvl_host = rospy.get_param('~dvl_host', '192.168.2.95')  # Default DVL IP
        self.dvl_port = rospy.get_param('~dvl_port', 16171)  # TCP port for JSON protocol
        self.frame_id = rospy.get_param('~frame_id', 'dvl_link')
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom')
        self.publish_tf = rospy.get_param('~publish_tf', True)
        self.connection_timeout = rospy.get_param('~connection_timeout', 5.0)
        self.reconnect_interval = rospy.get_param('~reconnect_interval', 2.0)
        
        # Publishers
        self.odom_pub = rospy.Publisher('dvl/odometry', Odometry, queue_size=10)
        self.pose_pub = rospy.Publisher('dvl/pose', PoseWithCovariance, queue_size=10)
        self.altitude_pub = rospy.Publisher('dvl/altitude', Range, queue_size=10)
        
        # Services
        self.acoustic_service = rospy.Service('dvl/set_acoustic_enabled', SetBool, self.set_acoustic_enabled_callback)
        
        # TF broadcaster
        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Socket and connection management
        self.socket = None
        self.connected = False
        self.socket_lock = threading.Lock()
        self.running = True
        
        # Data storage
        self.last_velocity_msg = None
        self.last_position_msg = None
        
        rospy.loginfo("Water Linked DVL Driver initialized")
        rospy.loginfo(f"Connecting to DVL at {self.dvl_host}:{self.dvl_port}")
        
    def connect_to_dvl(self):
        """Establish TCP connection to DVL"""
        try:
            with self.socket_lock:
                if self.socket:
                    self.socket.close()
                
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.settimeout(self.connection_timeout)
                self.socket.connect((self.dvl_host, self.dvl_port))
                self.connected = True
                rospy.loginfo("Successfully connected to DVL")
                return True
                
        except socket.error as e:
            rospy.logwarn(f"Failed to connect to DVL: {e}")
            self.connected = False
            return False
    
    def disconnect_from_dvl(self):
        """Close TCP connection to DVL"""
        with self.socket_lock:
            if self.socket:
                self.socket.close()
                self.socket = None
            self.connected = False
    
    def send_command(self, command_dict):
        """Send JSON command to DVL and return response"""
        try:
            with self.socket_lock:
                if not self.connected or not self.socket:
                    return None
                
                command_str = json.dumps(command_dict) + '\n'
                self.socket.send(command_str.encode('utf-8'))
                
                # Read response
                response_data = ""
                while True:
                    chunk = self.socket.recv(1024).decode('utf-8')
                    if not chunk:
                        break
                    response_data += chunk
                    if '\n' in response_data:
                        break
                
                if response_data.strip():
                    return json.loads(response_data.strip())
                return None
                
        except (socket.error, json.JSONDecodeError) as e:
            rospy.logwarn(f"Error sending command: {e}")
            self.connected = False
            return None
    
    def set_acoustic_enabled_callback(self, req):
        """Service callback to enable/disable acoustics"""
        response = SetBoolResponse()
        
        command = {
            "command": "set_config",
            "parameters": {
                "acoustic_enabled": req.data
            }
        }
        
        result = self.send_command(command)
        
        if result and result.get('success', False):
            response.success = True
            response.message = f"Acoustics {'enabled' if req.data else 'disabled'} successfully"
            rospy.loginfo(response.message)
        else:
            response.success = False
            error_msg = result.get('error_message', 'Unknown error') if result else 'Communication error'
            response.message = f"Failed to set acoustics: {error_msg}"
            rospy.logwarn(response.message)
        
        return response
    
    def parse_velocity_report(self, data):
        """Parse velocity-and-transducer report and publish odometry"""
        try:
            if data.get('type') != 'velocity':
                return
            
            current_time = rospy.Time.now()
            
            # Create odometry message
            odom_msg = Odometry()
            odom_msg.header.stamp = current_time
            odom_msg.header.frame_id = self.odom_frame_id
            odom_msg.child_frame_id = self.frame_id
            
            # Velocity data (body frame)
            odom_msg.twist.twist.linear.x = data.get('vx', 0.0)
            odom_msg.twist.twist.linear.y = data.get('vy', 0.0)
            odom_msg.twist.twist.linear.z = data.get('vz', 0.0)
            
            # Covariance matrix for velocity
            covariance = data.get('covariance', [[0]*3 for _ in range(3)])
            if len(covariance) == 3 and len(covariance[0]) == 3:
                # Fill 6x6 covariance matrix (only linear velocities)
                twist_cov = [0.0] * 36
                for i in range(3):
                    for j in range(3):
                        twist_cov[i*6 + j] = covariance[i][j]
                odom_msg.twist.covariance = twist_cov
            
            # Publish odometry
            self.odom_pub.publish(odom_msg)
            
            # Publish altitude as Range message
            if data.get('velocity_valid', False) and 'altitude' in data:
                range_msg = Range()
                range_msg.header.stamp = current_time
                range_msg.header.frame_id = self.frame_id
                range_msg.radiation_type = Range.ULTRASOUND
                range_msg.field_of_view = 0.1  # Approximate beam width
                range_msg.min_range = 0.05
                range_msg.max_range = 50.0
                range_msg.range = data['altitude']
                self.altitude_pub.publish(range_msg)
            
            self.last_velocity_msg = odom_msg
            
            rospy.logdebug(f"Published velocity: vx={data.get('vx', 0):.3f}, "
                          f"vy={data.get('vy', 0):.3f}, vz={data.get('vz', 0):.3f}")
            
        except Exception as e:
            rospy.logwarn(f"Error parsing velocity report: {e}")
    
    def parse_position_report(self, data):
        """Parse dead-reckoning report and publish pose"""
        try:
            if data.get('type') != 'position_local':
                return
            
            current_time = rospy.Time.now()
            
            # Create pose message
            pose_msg = PoseWithCovariance()
            
            # Position
            pose_msg.pose.position.x = data.get('x', 0.0)
            pose_msg.pose.position.y = data.get('y', 0.0)
            pose_msg.pose.position.z = data.get('z', 0.0)
            
            # Orientation from roll, pitch, yaw (in degrees)
            roll = np.radians(data.get('roll', 0.0))
            pitch = np.radians(data.get('pitch', 0.0))
            yaw = np.radians(data.get('yaw', 0.0))
            
            # Convert to quaternion
            quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
            pose_msg.pose.orientation.x = quat[0]
            pose_msg.pose.orientation.y = quat[1]
            pose_msg.pose.orientation.z = quat[2]
            pose_msg.pose.orientation.w = quat[3]
            
            # Covariance (simplified - using std as diagonal elements)
            std = data.get('std', 0.01)
            pose_cov = [0.0] * 36
            # Position covariance
            pose_cov[0] = std * std   # x
            pose_cov[7] = std * std   # y
            pose_cov[14] = std * std  # z
            # Orientation covariance (rough estimate)
            pose_cov[21] = 0.01  # roll
            pose_cov[28] = 0.01  # pitch
            pose_cov[35] = 0.01  # yaw
            pose_msg.covariance = pose_cov
            
            # Publish pose
            self.pose_pub.publish(pose_msg)
            
            # Publish TF if enabled
            if self.publish_tf:
                from geometry_msgs.msg import TransformStamped
                
                transform = TransformStamped()
                transform.header.stamp = current_time
                transform.header.frame_id = self.odom_frame_id
                transform.child_frame_id = self.frame_id
                
                transform.transform.translation.x = pose_msg.pose.position.x
                transform.transform.translation.y = pose_msg.pose.position.y
                transform.transform.translation.z = pose_msg.pose.position.z
                
                transform.transform.rotation = pose_msg.pose.orientation
                
                self.tf_broadcaster.sendTransform(transform)
            
            self.last_position_msg = pose_msg
            
            rospy.logdebug(f"Published pose: x={data.get('x', 0):.3f}, "
                          f"y={data.get('y', 0):.3f}, z={data.get('z', 0):.3f}")
            
        except Exception as e:
            rospy.logwarn(f"Error parsing position report: {e}")
    
    def listen_for_data(self):
        """Main loop to listen for DVL data"""
        while self.running and not rospy.is_shutdown():
            if not self.connected:
                if self.connect_to_dvl():
                    continue
                else:
                    rospy.logwarn(f"Retrying connection in {self.reconnect_interval} seconds...")
                    time.sleep(self.reconnect_interval)
                    continue
            
            try:
                with self.socket_lock:
                    if not self.socket:
                        continue
                    
                    # Read data from socket
                    data_buffer = ""
                    while True:
                        chunk = self.socket.recv(1024).decode('utf-8')
                        if not chunk:
                            raise socket.error("Connection closed by DVL")
                        
                        data_buffer += chunk
                        
                        # Process complete JSON messages (separated by newlines)
                        while '\n' in data_buffer:
                            line, data_buffer = data_buffer.split('\n', 1)
                            line = line.strip()
                            
                            if not line:
                                continue
                            
                            try:
                                json_data = json.loads(line)
                                
                                # Route data based on type
                                msg_type = json_data.get('type')
                                if msg_type == 'velocity':
                                    self.parse_velocity_report(json_data)
                                elif msg_type == 'position_local':
                                    self.parse_position_report(json_data)
                                elif msg_type == 'response':
                                    # Handle command responses if needed
                                    rospy.logdebug(f"Received response: {json_data}")
                                
                            except json.JSONDecodeError as e:
                                rospy.logwarn(f"Invalid JSON received: {e}")
                                continue
                            
            except socket.error as e:
                rospy.logwarn(f"Socket error: {e}")
                self.connected = False
                self.disconnect_from_dvl()
                time.sleep(self.reconnect_interval)
                
            except Exception as e:
                rospy.logerr(f"Unexpected error in data listener: {e}")
                time.sleep(1.0)
    
    def run(self):
        """Main execution function"""
        # Start data listening thread
        listener_thread = threading.Thread(target=self.listen_for_data)
        listener_thread.daemon = True
        listener_thread.start()
        
        rospy.loginfo("DVL driver started. Publishing on topics:")
        rospy.loginfo("  - /dvl/odometry (nav_msgs/Odometry)")
        rospy.loginfo("  - /dvl/pose (geometry_msgs/PoseWithCovariance)")
        rospy.loginfo("  - /dvl/altitude (sensor_msgs/Range)")
        rospy.loginfo("Services available:")
        rospy.loginfo("  - /dvl/set_acoustic_enabled (std_srvs/SetBool)")
        
        # Keep the node running
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down DVL driver...")
        finally:
            self.running = False
            self.disconnect_from_dvl()


def main():
    """Main entry point"""
    try:
        driver = WaterLinkedDVLDriver()
        driver.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Failed to start DVL driver: {e}")


if __name__ == '__main__':
    main()