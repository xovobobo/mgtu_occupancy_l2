#!/usr/bin/env python

import array
import rospy
import numpy as np

from threading import Lock
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid


class OccupancyPublisher:
    def __init__(self) -> None:
        self.__lock = Lock()
        self.__obstacles = []
        self.__publisher = rospy.Publisher('oneshot_occupancy', OccupancyGrid, queue_size=10)
        self.__resolution = rospy.get_param('~resolution', 0.2)
        map_side = rospy.get_param('~map_side', 60)
        self.__height = self.__width =  round(map_side / self.__resolution)

        self.__timer = rospy.Timer(rospy.Duration(secs=1), self.timer_cb)
        self.__clicked_point_sub = rospy.Subscriber('/clicked_point', PointStamped, self.clicked_point_cb, queue_size=1)

    def clicked_point_cb(self, msg:PointStamped):
        x,y = round(msg.point.x / self.__resolution), round(msg.point.y / self.__resolution)
        with self.__lock:
            self.__obstacles.append([x,y])

    def timer_cb(self, _):
        msg = OccupancyGrid()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'occ_test'

        msg.info.origin.position.x = 0
        msg.info.origin.position.y = 0
        msg.info.origin.position.z = 0
        msg.info.origin.orientation.w = 1

        msg.info.resolution = self.__resolution
        msg.info.height = self.__height
        msg.info.width = self.__width

        occ_data = np.full((self.__height, self.__width), fill_value=-1, dtype=np.int8)
        with self.__lock:
            obstacles = np.array(self.__obstacles.copy())

        if len(obstacles) >0:
            # for obst in obstacles:
            #     cv2.circle(occ_data, (obst[0], obst[1]), 5, color=100, thickness=-1)
            occ_data[obstacles[:, 1], obstacles[:, 0]] = 100

        msg.data = array.array('b', np.ndarray.tobytes(occ_data))

        self.__publisher.publish(msg)
        rospy.loginfo("Карта опубликована")


if __name__ == '__main__':
    rospy.init_node('oneshot_node')
    occ_publisher = OccupancyPublisher()
    rospy.spin()