//迭代最近点（Iterative Closest Point）算法执行配准和匹配
//运行pcl_read,pcl_filter,pcl_downsampling,然后运行该程序
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <iostream>
using namespace std;
class cloudHandler
{
public:
    cloudHandler()
    {
        pcl_sub = nh.subscribe("pcl_downsampled", 10, &cloudHandler::cloudCB, this);
        pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_matched", 1);
    }

    void cloudCB(const sensor_msgs::PointCloud2 &input)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud_in;
        pcl::PointCloud<pcl::PointXYZ> cloud_out;
        pcl::PointCloud<pcl::PointXYZ> cloud_aligned;
        sensor_msgs::PointCloud2 output;

        pcl::fromROSMsg(input, cloud_in);

        cloud_out = cloud_in;

        for (size_t i = 0; i < cloud_in.points.size (); ++i)
        {
            cloud_out.points[i].x = cloud_in.points[i].x + 0.7f;
        }
        cout<<"cloud_in size:"<<cloud_in.points.size()<<endl;
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(cloud_in.makeShared());
        icp.setInputTarget(cloud_out.makeShared());

        icp.setMaxCorrespondenceDistance(5);//设置对应距离：该参数定义了对准过程中两个对应点之间应具有的最小距离？？
       
        //三个基本的停止条件
        icp.setMaximumIterations(300);//设置最大迭代次数
        icp.setTransformationEpsilon (1e-12);//设定阈值：上一个转换和当前转换之间的差别小于特性的阈值
        icp.setEuclideanFitnessEpsilon(0.1);//在循环中两次连续步骤之间的欧几里得平方误差之和低于特定的阈值

        icp.align(cloud_aligned);// 通过得到的变换矩阵，将source点云拼接到target点云上
        cout<<"has converged:"<<icp.hasConverged()<<" socre: "<<icp.getFitnessScore()<<endl;
        cout<<"Transformation:"<<icp.getFinalTransformation()<<endl;
        cout<<cloud_aligned.points.size()<<endl;
        pcl::toROSMsg(cloud_aligned, output);
        pcl_pub.publish(output);
    }

protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    ros::Publisher pcl_pub;
};

main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_matching");

    cloudHandler handler;

    ros::spin();

    return 0;
}
