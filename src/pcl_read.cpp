//PCL读取点云文件.pcd
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
int main(int argc, char **argv)
{
    ros::init(argc,argv,"pcl_read");
    ros::NodeHandle nh;
    ros::Publisher pcl_pub=nh.advertise<sensor_msgs::PointCloud2>("pcl_output",1);
    sensor_msgs::PointCloud2 output;
    pcl::PointCloud <pcl::PointXYZ> cloud;
    pcl::io::loadPCDFile("/home/nvidia/dataset/test_pcd.pcd",cloud);
    pcl::toROSMsg(cloud,output);
    output.header.frame_id="pclread";
    ros::Rate loop_rate(1);
    while(ros::ok())
    {
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }
            
    return 0;
}
