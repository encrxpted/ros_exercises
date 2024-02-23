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

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import math


class OpenSpacePublisher(Node):

    def __init__(self):
        super().__init__('open_space_publisher')
        self.subscription = self.create_subscription(
            LaserScan,
            'fake_scan',
            self.listener_callback,
            10)
        self.publisher_distance = self.create_publisher(Float32, 'open_space/distance', 10)
        self.publisher_angle = self.create_publisher(Float32, 'open_space/angle', 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        d = -2.0/3 * math.pi
        i =0
        i_of_d = 0
        for x in msg.ranges:
            if x > d:
                d = x
                i_of_d = i
            i=i+1

        angle = -2.0/3*math.pi + i/400.0 * 4.0/3 * math.pi
        msg_angle = Float32()
        msg_angle.data = angle
        msg_dist = Float32()
        msg_dist.data = d
        self.publisher_angle.publish(msg_angle)
        self.publisher_distance.publish(msg_dist)


def main(args=None):
    rclpy.init(args=args)

    open_space_publisher = OpenSpacePublisher()

    rclpy.spin(open_space_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    open_space_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
