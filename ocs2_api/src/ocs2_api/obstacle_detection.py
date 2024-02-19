#!/usr/bin/env python3
import numpy as np
import rospy
from grid_map_msgs.msg import GridMap
import std_msgs.msg
from sensor_msgs import point_cloud2
import matplotlib.pyplot as plt
import sensor_msgs.point_cloud2 as pc2
from mpl_toolkits import mplot3d
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Bool


class PCLFromGrid():
    def __init__(self):
        self.grid_map = None
        self.body_height = 0
        self.diff_body_floor_height = 0.49

    def listener(self):
        rospy.Subscriber('/convex_plane_decomposition_ros/filtered_map', GridMap, self.grid_map_callback)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.curr_state_callback)
        #rospy.Subscriber('/convex_plane_decomposition_ros/planar_terrain', GridMap, self.grid_map_callback)
    def grid_map_callback(self, data):
        self.grid_map = data
    def curr_state_callback(self, data):
        pose = data.pose[-1]
        self.body_height = pose.position.z
    def get_pointcloud_map(self):
        detect_obstacle = rospy.Publisher('slow_down', Bool, queue_size=10)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.listener()
            slow_down = False
            if self.grid_map != None:
                if 'elevation' == self.grid_map.layers[0]:
                    elevation_data = self.grid_map.data[0].data
                    resolution = self.grid_map.info.resolution
                    size_x = self.grid_map.info.length_x
                    size_y = self.grid_map.info.length_y
                    position_x = self.grid_map.info.pose.position.x
                    position_y = self.grid_map.info.pose.position.y
                    pointcloud_data = []
                    for i in range(len(elevation_data)):
                        x = (i % size_x) * resolution + position_x
                        y = (i // size_x) * resolution + position_y
                        z = elevation_data[i]
                        pointcloud_data.append([x,y,z])
                    pointcloud_data_numpy = np.array(pointcloud_data, dtype=np.float32)
                    header = std_msgs.msg.Header()
                    header.stamp = rospy.Time.now()
                    header.frame_id = 'pointcloud_from_gridmap'
                    poincloud_msg = pc2.create_cloud_xyz32(header,pointcloud_data_numpy)
                    slow_down = self.process_pointcloud(poincloud_msg)
            detect_obstacle.publish(slow_down)
            rate.sleep()

    def process_pointcloud(self, pointcloud_msg):
        cloud_points = list(point_cloud2.read_points(pointcloud_msg, skip_nans=True, field_names=("x", "y", "z")))
        cloud_points = np.array(cloud_points)
        height_max = np.max(cloud_points[:,2])
        height_min = np.min(cloud_points[:,2])
        diff_max = abs(self.body_height-height_max)
        diff_min = abs(self.body_height-height_min)
        slow_down = False
        if abs(diff_max - self.diff_body_floor_height) > 0.15 or abs(diff_min-self.diff_body_floor_height) > 0.15:
            slow_down = True
        return slow_down

if __name__ == '__main__':
    try:
        rospy.init_node('Processing_gridmap')
        pcl = PCLFromGrid().get_pointcloud_map()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


