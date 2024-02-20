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
import math
import random
from sensor_msgs.msg import LaserScan


class FakeScanPublisher(Node):

    def __init__(self):
        super().__init__('fake_scan_publisher')
        self.publisher_ = self.create_publisher(LaserScan, 'fake_scan', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        scan = LaserScan()

        current_time = rclpy.time.Time().to_msg()
        scan.header.stamp = current_time
        scan.header.frame_id = 'base_link'
        scan.angle_min = -2.0/3 * math.pi
        scan.angle_max = 2.0/3 *math.pi
        scan.angle_increment = 1.0/300 * math.pi
        scan.time_increment = .5
        scan.range_min = 1.0
        scan.range_max = 10.0
        scan.ranges = [random.uniform(-2.0/3*math.pi, 2.0/3*math.pi) for i in range(401) ]

        self.publisher_.publish(scan)
        #self.get_logger().info("Publishing")
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    fake_scan_publisher = FakeScanPublisher()

    rclpy.spin(fake_scan_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    fake_scan_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
