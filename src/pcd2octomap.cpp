/**
 * @file pcd2octomap.cpp
 * @brief Convert pcd file to binary octomap
 */

#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h> // 추가된 부분
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;

class PCDToOctomapConverter {
public:
    PCDToOctomapConverter() {
        pcl_sub = nh.subscribe("pcd_topic", 1, &PCDToOctomapConverter::pcdCallback, this);
    }

    void pcdCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
        pcl::PointCloud<pcl::PointXYZRGBA> cloud;
        pcl::fromROSMsg(*cloud_msg, cloud); // fromROSMsg 함수 사용

        cout << "point cloud loaded, point size = " << cloud.points.size() << endl;

        octomap::OcTree tree(0.05);

        for (auto& p : cloud.points) {
            tree.updateNode(octomap::point3d(p.x, p.y, p.z), true);
        }

        tree.updateInnerOccupancy();

        string output_file = "/path/to/output.ot";
        tree.writeBinary(output_file);

        cout << "Octomap saved to " << output_file << endl;
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcd2octomap_node");

    PCDToOctomapConverter converter;

    ros::spin();

    return 0;
}
