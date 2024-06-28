/**
 * @file pcd2colorOctomap.cpp
 * @brief Convert pcd file to color octomap
 */

#include <iostream>
#include <string>

// ROS headers
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

// PCL headers
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// OctoMap headers
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

using namespace std;

/** 
 * Main function
 *
 *  Convert pcd file to color octomap. This function uses
 * pcl::PointXYZRGBA and octomap::ColorOcTree
 *
 * @param[in] argc How many argument
 * @param[in] argv Concrete argument
 * @return 0 for success, otherwise failure
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcd2colorOctomap");
    ros::NodeHandle nh;

    if (argc != 3)
    {
        ROS_ERROR("Usage: rosrun your_package_name pcd2colorOctomap <input_file> <output_file>");
        return -1;
    }

    string input_file = argv[1], output_file = argv[2];
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(input_file, *cloud) == -1)
    {
        ROS_ERROR("Couldn't read file %s\n", input_file.c_str());
        return -1;
    }

    ROS_INFO("Point cloud loaded, point size = %lu", cloud->points.size());

    // OctoMap 객체 생성
    octomap::ColorOcTree tree(0.05);

    // 포인트 클라우드 데이터를 OctoMap에 복사
    for (auto p : cloud->points)
    {
        tree.updateNode(octomap::point3d(p.x, p.y, p.z), true);
        tree.integrateNodeColor(p.x, p.y, p.z, p.r, p.g, p.b);
    }

    // OctoMap 업데이트
    tree.updateInnerOccupancy();

    // OctoMap을 .ot 파일로 저장
    if (tree.write(output_file))
    {
        ROS_INFO("Octomap saved as %s", output_file.c_str());
    }
    else
    {
        ROS_ERROR("Failed to write octomap to file %s", output_file.c_str());
        return -1;
    }

    ROS_INFO("Conversion from PCD to color Octomap complete");
    return 0;
}
