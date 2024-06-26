# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import time
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            'imu',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.euler= [0,0,0]

    def listener_callback(self, msg):

        #self.get_logger().info('I heard: "%d"' % msg.angular_velocity_covariance[0])
        self.euler[0] = msg.linear_acceleration_covariance[0]   # roll
        self.euler[1] = msg.linear_acceleration_covariance[4]  #pitch
        self.euler[2] = msg.linear_acceleration_covariance[8]  #yaw
        print(self.euler)
        if self.euler[1] > 0.3:
            print(2)
        

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()
    
    rclpy.spin(minimal_subscriber)
    msg.linear_acceleration_covariance[0]
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()

    print(minimal_subscriber.euler)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
