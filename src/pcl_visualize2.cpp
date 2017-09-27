
//PCL更加复杂以及完整的可视化组件PCLVisualizer的示例
//运行pcl_read, pcl_filter,然后运行该程序
//该程序同时显示原始的点云和通过filtered的点云

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>

class cloudHandler
{
public:
    cloudHandler(): viewer("Cloud Viewer")
    {
        output_sub = nh.subscribe("pcl_output", 1, &cloudHandler::outputCB, this);
        filtered_sub = nh.subscribe("pcl_filtered", 1, &cloudHandler::filteredCB, this);
        viewer_timer = nh.createTimer(ros::Duration(0.1), &cloudHandler::timerCB, this);

        viewer.createViewPort(0.0, 0.0, 0.5, 1.0, output_view);
        viewer.setBackgroundColor(0, 0, 0, output_view);

        viewer.createViewPort(0.5, 0.0, 1.0, 1.0, filtered_view);
        viewer.setBackgroundColor(0, 0, 0, filtered_view);

        viewer.addCoordinateSystem(1.0);
        viewer.initCameraParameters();
    }
    void outputCB(const sensor_msgs::PointCloud2ConstPtr& input)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*input, cloud);

        viewer.removeAllPointClouds(output_view);
        viewer.addPointCloud<pcl::PointXYZ>(cloud.makeShared(), "output", output_view);
    }

    void filteredCB(const sensor_msgs::PointCloud2ConstPtr& input)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*input, cloud);

        viewer.removeAllPointClouds(filtered_view);
        viewer.addPointCloud<pcl::PointXYZ>(cloud.makeShared(), "filtered", filtered_view);
    }

    void timerCB(const ros::TimerEvent&)
    {
        viewer.spinOnce();

        if (viewer.wasStopped())
        {
            ros::shutdown();
        }
    }
protected:
    ros::NodeHandle nh;
    ros::Subscriber output_sub, filtered_sub;
    pcl::visualization::PCLVisualizer viewer;
    int output_view, filtered_view;
    ros::Timer viewer_timer;
};

int main (int argc, char **argv)
{
    ros::init (argc, argv, "pcl_visualize2");

    cloudHandler handler;

    ros::spin();

    return 0;
}
