#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class PointCloudFilter {
public:
    PointCloudFilter() {
        // Initialize ROS node handle, subscriber, and publisher
        sub_ = nh_.subscribe("/cloud_in", 1, &PointCloudFilter::cloudCallback, this);
        pub_ = nh_.advertise<PointCloud>("/filtered_cloud", 1);

        // Get parameters from launch file or parameter server
        nh_.param("filter_limit_min", filter_limit_min_, 0.2);
        nh_.param("filter_limit_max", filter_limit_max_, 1.5);
    }

    void cloudCallback(const PointCloud::ConstPtr& cloud) {
        PointCloud::Ptr filtered_cloud(new PointCloud);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(filter_limit_min_, filter_limit_max_);
        pass.filter(*filtered_cloud);
        pub_.publish(filtered_cloud);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    double filter_limit_min_;
    double filter_limit_max_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "filter_pointcloud");
    PointCloudFilter filter;
    ros::spin();
    return 0;
}
