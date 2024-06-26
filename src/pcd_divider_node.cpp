#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcd_divider_node");
    ros::NodeHandle nh("~");

    std::string input_pcd_file;
    nh.param("pcd_file", input_pcd_file, std::string("input.pcd"));
    
    float z_threshold;
    nh.param("z_threshold", z_threshold, 0.0f);

    // PointCloud 객체 생성
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // PCD 파일 읽기
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_pcd_file, *cloud) == -1) {
        PCL_ERROR("Couldn't read the file %s\n", input_pcd_file.c_str());
        return -1;
    }

    std::cout << "Loaded " << cloud->width * cloud->height << " data points from " << input_pcd_file << std::endl;

    // z축 기준 반으로 자르기
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        if (cloud->points[i].z < z_threshold) {
            inliers->indices.push_back(i);
        }
    }

    // 첫번째 반의 점들을 추출
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_half1(new pcl::PointCloud<pcl::PointXYZ>);
    extract.filter(*cloud_half1);

    // 두번째 반의 점들을 추출
    extract.setNegative(true);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_half2(new pcl::PointCloud<pcl::PointXYZ>);
    extract.filter(*cloud_half2);

    // 두개의 PCD 파일로 저장
    pcl::io::savePCDFileASCII("/home/sh/catkin_ws/pcd/map_down.pcd", *cloud_half1);
    pcl::io::savePCDFileASCII("/home/sh/catkin_ws/pcd/map_top.pcd", *cloud_half2);

    std::cout << "Saved map_down.pcd with " << cloud_half1->width * cloud_half1->height << " data points." << std::endl;
    std::cout << "Saved map_top.pcd with " << cloud_half2->width * cloud_half2->height << " data points." << std::endl;

    return 0;
}
