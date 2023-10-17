#!/usr/bin/env python3

#Created by Tony Jacob
#tony.jacob@uri.edu
#Filters the pcl2 published by ping360_sonar
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from math import nan

class Processing:
    def __init__(self) -> None:
        # Subscribe to the PointCloud2 topic
        rospy.Subscriber("/msis", PointCloud2, self.pointcloud_callback)
        self.pcl_pub = rospy.Publisher("/filtered_msis", PointCloud2, queue_size=1)
        
        self.publish = rospy.get_param("/ping360_sonar_node/Driver/filtered_pointcloud/publish", True)
        self.std_dev_multiplier = rospy.get_param("/ping360_sonar_node/Driver/filtered_pointcloud/std_dev_multiplier", 2.0)
        self.pub_topic = rospy.get_param("/ping360_sonar_node/Driver/filtered_pointcloud/pub_topic", "/alpha_rise/msis/pointcloud/filtered")
        self.radius = rospy.get_param("/ping360_sonar_node/Driver/filtered_pointcloud/radius", 2)
    
    def pointcloud_callback(self, pointcloud_msg):
        if self.publish:
            pcl_msg = PointCloud2()
            pcl_msg.header.frame_id = pointcloud_msg.header.frame_id
            pcl_msg.header.stamp = pointcloud_msg.header.stamp
            
            pcl_msg.fields = pointcloud_msg.fields
            pcl_msg.height = pointcloud_msg.height
            pcl_msg.width = pointcloud_msg.width
            pcl_msg.point_step = pointcloud_msg.point_step
            pcl_msg.row_step = pointcloud_msg.row_step
            pcl_msg.is_dense = pointcloud_msg.is_dense


            mean, std_dev, intensities = self.get_intensities(pointcloud_msg=pointcloud_msg)
        
            # Populate filtered pointclouds.
            points = np.zeros((pcl_msg.width,len(pcl_msg.fields)),dtype=np.float32)
            for index,point in enumerate(pc2.read_points(pointcloud_msg, skip_nans=True)):
                if index >= self.radius * 60:
                    ##Total bins = 1200. Set range = 20m. 1m = 60bins.
                    x, y, z, i = point[:4]
                    #Filter
                    if i > mean+self.std_dev_multiplier *std_dev:              
                        points[index][0] = x
                        points[index][1] = y
                        points[index][3] = i
                    else:
                        points[index][0] = nan
                        points[index][1] = nan
                        points[index][3] = nan
                else:
                    points[index][0] = nan
                    points[index][1] = nan
                    points[index][3] = nan
            
            pcl_msg.data = points.tobytes()
            self.pcl_pub.publish(pcl_msg)

    def get_intensities(self,pointcloud_msg):
        """
        Returns the mean, std_dev and the echo intensity arrays.
        """
        intensities = []
        for index,point in enumerate(pc2.read_points(pointcloud_msg, skip_nans=True)):
            x, y, z, i = point[:4]
            intensities.append(i)
        intensities = np.array(intensities)
        mean = np.mean(intensities)
        std_dev = np.std(intensities)
        return mean, std_dev, intensities.tolist()
    
if __name__ == '__main__':
    rospy.init_node('pointcloud_filter', anonymous=True)
    process=Processing()
    rospy.spin()