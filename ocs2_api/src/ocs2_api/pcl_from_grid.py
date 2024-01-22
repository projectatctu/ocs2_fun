#!/usr/bin/env python
import numpy as np
import rospy
from grid_map_msgs.msg import GridMap
from sensor_msgs.msg import PointCloud2
import pcl
import std_msgs.msg
from sensor_msgs import point_cloud2
import matplotlib.pyplot as plt
import sensor_msgs.point_cloud2 as pc2
from mpl_toolkits import mplot3d

class PCLFromGrid():
    def __init__(self):
        self.grid_map = None

    def listener(self):
        rospy.Subscriber('/convex_plane_decomposition_ros/filtered_map', GridMap, self.grid_map_callback)
        #rospy.Subscriber('/convex_plane_decomposition_ros/planar_terrain', GridMap, self.grid_map_callback)
    def grid_map_callback(self, data):
        self.grid_map = data
    def get_pointcloud_map(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.listener()
            if self.grid_map != None:
                #print(self.grid_map.basic_layers)
                if 'elevation' == self.grid_map.layers[0]:
                    #elevation = self.grid_map.layers['elevation']
                    #print(len(self.grid_map.data))
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
                    self.process_pointcloud(poincloud_msg)
            rate.sleep()

    def process_pointcloud(self, pointcloud_msg):
        fig = plt.figure()
        #print(pointcloud_msg.shape)
        cloud_points = list(point_cloud2.read_points(pointcloud_msg, skip_nans=True, field_names=("x", "y", "z")))
        #print(cloud_points)
        #print(cloud_points.shape)
        cloud_points = np.array(cloud_points)
        #print(cloud_points.shape)
        print(np.linalg.norm(cloud_points[:,2]))
        ax = fig.add_subplot(projection='3d')
        ax.scatter(cloud_points[:, 0], cloud_points[:, 1], cloud_points[:, 2])
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('Grid map')
        plt.show()


if __name__ == '__main__':
    try:
        rospy.init_node('Processing_gridmap')
        pcl = PCLFromGrid().get_pointcloud_map()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


