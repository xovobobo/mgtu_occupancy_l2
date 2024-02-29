#!/usr/bin/env python

import array
import rospy
import numpy as np
import math

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan


class OccupancyPublisher:
    def __init__(self) -> None:
        self.__publisher = rospy.Publisher('oneshot_occupancy', OccupancyGrid, queue_size=10)
        self.__resolution = rospy.get_param('~resolution', 0.1)
        map_side = rospy.get_param('~map_side', 60)
        self.__height = self.__width =  round(map_side / self.__resolution)
        self.__obstacle_color = rospy.get_param('~obstacle_color', 100)
        self.__subscriber = rospy.Subscriber('/scan', LaserScan, self.laserscan_cb, queue_size=10)

    def laserscan_cb(self, laser_msg:LaserScan):
        msg = OccupancyGrid()
        msg.header = laser_msg.header

        msg.info.origin.position.x = -self.__width /2 * self.__resolution
        msg.info.origin.position.y = -self.__height /2 * self.__resolution
        msg.info.origin.position.z = 0
        msg.info.origin.orientation.w = 1

        msg.info.resolution = self.__resolution
        msg.info.height = self.__height
        msg.info.width = self.__width

        occ_data = np.full((self.__height, self.__width), fill_value=-1, dtype=np.int8)

        for i, scan_range in enumerate(laser_msg.ranges):
            if math.isinf(scan_range):
                continue

            angle = laser_msg.angle_min + i * laser_msg.angle_increment

            x = math.floor((msg.info.origin.position.x + (scan_range * math.cos(angle))) / self.__resolution)
            y = math.floor((msg.info.origin.position.y + (scan_range * math.sin(angle))) / self.__resolution)

            if abs(x) >= self.__width or abs(x) < 0:
                continue
            elif abs(y) >= self.__height or abs(y) < 0:
                continue

            occ_data[y,x] = self.__obstacle_color

        msg.data = array.array('b', np.ndarray.tobytes(occ_data))

        self.__publisher.publish(msg)
        rospy.loginfo("Карта опубликована")


if __name__ == '__main__':
    rospy.init_node('oneshot_node')
    occ_publisher = OccupancyPublisher()
    rospy.spin()