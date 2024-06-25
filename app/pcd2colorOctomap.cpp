/**
 * @file pcd2colorOctomap.cpp
 * @brief Convert pcd file to color octomap
 */

#include <iostream>
#include <assert.h>

//pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//octomap 
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

using namespace std;

/** 
 * Main function
 *
 *  Convert pcd file to color octomap. This function uses
 * pcl::PointXYZRGBA and octomap::OcTree
 *
 * @param[in] argc How many argument
 * @param[in] argv Concrete argument
 * @return 0 for success, otherwise failure
 */
int main( int argc, char** argv )
{
    if (argc != 3)
    {
        cout<<"Usage: pcd2colorOctomap <input_file> <output_file>"<<endl;
        return -1;
    }

    string input_file = argv[1], output_file = argv[2];
    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    pcl::io::loadPCDFile<pcl::PointXYZRGBA> ( input_file, cloud );

    cout<<"point cloud loaded, piont size = "<<cloud.points.size()<<endl;

    // octomap 변수 선언
    cout<<"copy data into octomap..."<<endl;
    // 해상도를 매개변수로 사용하여 컬러 옥토맵 객체를 만듭니다. 여기서는 0.05로 설정했습니다.
    octomap::ColorOcTree tree( 0.05 );

    for (auto p:cloud.points)
    {
        // 포인트 클라우드의 점을 옥토맵에 삽입합니다.
        tree.updateNode( octomap::point3d(p.x, p.y, p.z), true );
    }

    // 색상 설정
    for (auto p:cloud.points)
    {
        tree.integrateNodeColor( p.x, p.y, p.z, p.r, p.g, p.b );
    }

    // 옥토맵 업데이트
    tree.updateInnerOccupancy();
    // 옥토맵 저장, .bt 파일이 아닌 .ot 파일로 저장해야 합니다.
    tree.write( output_file );
    cout<<"done."<<endl;

    return 0;
}

