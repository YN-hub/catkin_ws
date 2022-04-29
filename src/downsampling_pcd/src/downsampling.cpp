#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>



ros::Publisher downsampled_pcd;

void downsampling_pointcloud2(const sensor_msgs::PointCloud2& msg)
{
    sensor_msgs::PointCloud2 output;
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::PointCloud<pcl::PointXYZRGB> downsampled_cloud;
    pcl::fromROSMsg(msg, cloud);
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
	  vg.setInputCloud(cloud.makeShared());
	  vg.setLeafSize(0.2, 0.2, 0.2);
	  vg.filter(downsampled_cloud);
    pcl::toROSMsg(downsampled_cloud, output);
    downsampled_pcd.publish(output);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "downsampling_pcd");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("cloud_pcd", 100, downsampling_pointcloud2);//subscribeするtopicの名前　サブスクライバのキューサイズ　spin()実行時に呼ばれる関数名
  downsampled_pcd=nh.advertise<sensor_msgs::PointCloud2>("downsampled_pointcloud2",100);

  ros::spin();
  return 0;
}
