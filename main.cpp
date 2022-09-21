#include <iostream>
#include "patchworkpp.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>


void viewTwoPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud1,const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud2 ,int pointssize=1)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("test"));
    viewer->setBackgroundColor(0,0,0); // rgb
    viewer->addCoordinateSystem();

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(cloud1, 255, 0, 0); // rgb
    viewer->addPointCloud<pcl::PointXYZ>(cloud1, color1, "cloud1"); 
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointssize, "cloud1"); 

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color2(cloud2, 0, 255, 0); // rgb
    viewer->addPointCloud<pcl::PointXYZ>(cloud2, color2, "cloud2"); 
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointssize, "cloud2"); 

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }
    viewer->close();
}

void pcl2eigen(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,Eigen::MatrixX3f &cloud)
{
    int num=cloud_in->points.size();
    cloud.resize(num,3);
    for (int i = 0; i < num; i++)
    {
        cloud.row(i)<<cloud_in->points[i].x,cloud_in->points[i].y,cloud_in->points[i].z;
    }
}

void eigen2pcl(const Eigen::MatrixX3f &cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
        int num=cloud_in.rows();
        for (int i = 0; i < num; i++)
        {
            pcl::PointXYZ p;
            p.x=cloud_in(i,0);        p.y=cloud_in(i,1);        p.z=cloud_in(i,2);
            cloud->push_back(p);
        }
}

int main(int argc, char const *argv[])
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>("../data/testdata.pcd",*cloud);

    // Patchwork++ initialization
    patchwork::Params patchwork_parameters;
    patchwork::PatchWorkpp Patchworkpp(patchwork_parameters);

    // Load point cloud
    Eigen::MatrixX3f cloudEigen;
        pcl2eigen(cloud,cloudEigen);
    // Estimate Ground
    Patchworkpp.estimateGround(cloudEigen);
        Eigen::MatrixX3f ground     = Patchworkpp.getGround();
        Eigen::MatrixX3f nonground  = Patchworkpp.getNonground();


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_NoGround(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Ground(new pcl::PointCloud<pcl::PointXYZ>);
        eigen2pcl(ground,cloud_Ground);
        eigen2pcl(nonground,cloud_NoGround);

    std::cout << "Origianl Points  #: " << cloud->points.size() << std::endl;
    std::cout << "Ground Points    #: " << ground.rows() << std::endl;
    std::cout << "Nonground Points #: " << nonground.rows() << std::endl;


    viewTwoPointCloud(cloud_NoGround,cloud_Ground);


    return 0;
}
