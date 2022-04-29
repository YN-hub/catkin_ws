#include<ros/ros.h>
#include <std_msgs/String.h>
#include<sensor_msgs/PointCloud2.h>
#include<sensor_msgs/point_cloud_conversion.h>
#include<sensor_msgs/PointCloud.h>

ros::Publisher converted_pointcloud;
void poincloud2_to_pointcloud(const sensor_msgs::PointCloud2& msg)
{
    sensor_msgs::PointCloud output;
    sensor_msgs::convertPointCloud2ToPointCloud(msg,output);
    converted_pointcloud.publish(output);
    
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud2_to_pointcloud");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("velodyne_points", 10, poincloud2_to_pointcloud);
  converted_pointcloud=nh.advertise<sensor_msgs::PointCloud>("poincloud2_to_pointcloud",100);

  ros::spin();
  return 0;
}