#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include <time.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "myICP.h"

void visualize_pcd(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_ptr,
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_ptr,
	pcl::PointCloud<pcl::PointXYZ>::Ptr translated_cloud_ptr)
{
	//int vp_1, vp_2;
	// Create a PCLVisualizer object
	pcl::visualization::PCLVisualizer viewer("registration Viewer");
	//viewer.createViewPort (0.0, 0, 0.5, 1.0, vp_1);
	// viewer.createViewPort (0.5, 0, 1.0, 1.0, vp_2);
	viewer.setBackgroundColor(255, 255, 255);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(source_cloud_ptr, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(target_cloud_ptr, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_h(translated_cloud_ptr, 0, 0, 255);
	viewer.addPointCloud(source_cloud_ptr, src_h, "source cloud");
	viewer.addPointCloud(target_cloud_ptr, tgt_h, "tgt cloud");
	viewer.addPointCloud(translated_cloud_ptr, final_h, "final cloud");
	//viewer.addCoordinateSystem(1.0);
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

int main(int argc, char** argv)
{
	//std::string source_filename = "C:/Users/wangz/Desktop/models/dragonRotated.pcd";
	//std::string target_filename = "C:/Users/wangz/Desktop/models/dragon.pcd";
	std::string source_filename = "C:/Users/wangz/Desktop/models/bunRotated.pcd";
	std::string target_filename = "C:/Users/wangz/Desktop/models/bun.pcd";
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);// load the source point cloud
	pcl::io::loadPCDFile(source_filename, *source_cloud_ptr);
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);// load the target point cloud
	pcl::io::loadPCDFile(target_filename, *target_cloud_ptr);

	clock_t start = clock();
	my_ICP transformation_calculator;
	transformation_calculator.set_source_point_cloud(source_cloud_ptr);
	transformation_calculator.set_target_point_cloud(target_cloud_ptr);
	transformation_calculator.set_mean_square_error_difference_boundary(1e-10);

	Eigen::Matrix4f final_transformation;	// the final transformation from source point cloud to the target point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_source_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);	// the transformed souce cloud
	Eigen::Matrix4f initial_guess;
	initial_guess = Eigen::Matrix4f::Identity();

	transformation_calculator.align(transformed_source_cloud_ptr, initial_guess);
	clock_t icp_time = clock();
	std::cout << "Iterative closest point algorithm takes: " << (double)(icp_time - start) / (double)CLOCKS_PER_SEC << " s" << std::endl;

	final_transformation = transformation_calculator.get_final_transformation();
	std::cout << final_transformation << std::endl;

	visualize_pcd(source_cloud_ptr, target_cloud_ptr, transformed_source_cloud_ptr);
	return 0;
}