#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <vector>
#include <ctime>
#include "my_sacia.h"
#include "my_fpfh.h"
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

//点云可视化
void visualize_pcd(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_ptr,
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_ptr,
	pcl::PointCloud<pcl::PointXYZ>::Ptr translated_cloud_ptr)
{
	// Create a PCLVisualizer object
	pcl::visualization::PCLVisualizer viewer("registration Viewer");
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

pcl::PointCloud<std::vector<double> > computeFPFH(std::string filename, double normal_radius, double FPFH_radius, int subdivision)
{
	std::cout << "Reading " << filename << std::endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile <pcl::PointXYZ>(filename.c_str(), *cloud) == -1)
		// load the file
	{
		PCL_ERROR("Couldn't read file");
	}
	std::cout << "Loaded " << cloud->points.size() << " points." << std::endl;
	std::cout << "Point Cloud width: " << cloud->width << " Point Cloud height: " << cloud->height << std::endl;
	// Compute the normals
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
	normal_estimation.setInputCloud(cloud);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	normal_estimation.setSearchMethod(kdtree);

	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud< pcl::Normal>);
	normal_estimation.setRadiusSearch(normal_radius);
	normal_estimation.compute(*normals);

	pcl::PointCloud<std::vector<double> >::Ptr fpfh_result_ptr(new pcl::PointCloud<std::vector<double> >);
	my_fpfh my_fpfh_calculator(subdivision);
	my_fpfh_calculator.set_cloud(cloud);
	my_fpfh_calculator.set_normal(normals);
	my_fpfh_calculator.set_radius(FPFH_radius);
	my_fpfh_calculator.compute_fpfh(fpfh_result_ptr);

	return *fpfh_result_ptr;
}

int main(int argc, char**argv)
{
	std::string bun_target = "C:/Users/wangz/Desktop/models/bun.pcd";	// mesh resolution 0.001
	std::string bun_source = "C:/Users/wangz/Desktop/models/bunRotated.pcd"; // mesh resolution 0.00045
	std::string dragon_source = "C:/Users/wangz/Desktop/models/dragonRotated.pcd";
	std::string dragon_target = "C:/Users/wangz/Desktop/models/dragon.pcd";

	pcl::PointCloud<std::vector<double> > source_FPFH;
	pcl::PointCloud<std::vector<double> > target_FPFH;

	clock_t start = clock();

	//source_FPFH = computeFPFH(bun_source, 0.01, 0.003, 11);
	//target_FPFH = computeFPFH(bun_target, 0.01, 0.003, 11);
	source_FPFH = computeFPFH(dragon_source, 0.01, 0.003, 11);
	target_FPFH = computeFPFH(dragon_target, 0.01, 0.003, 11);


	clock_t feature_time = clock();
	std::cout << "Calculate FPFH feature time: " << (double)(feature_time - start) / (double)CLOCKS_PER_SEC << " s" << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZ>() );
	pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_cloud(new pcl::PointCloud<pcl::PointXYZ>() );

	//pcl::io::loadPCDFile <pcl::PointXYZ>(bun_source.c_str(), *src_cloud);
	//pcl::io::loadPCDFile <pcl::PointXYZ>(bun_target.c_str(), *tgt_cloud);
	pcl::io::loadPCDFile <pcl::PointXYZ>(dragon_source.c_str(), *src_cloud);
	pcl::io::loadPCDFile <pcl::PointXYZ>(dragon_target.c_str(), *tgt_cloud);

	my_sacia transformation_calculator;
	transformation_calculator.set_src_cloud(src_cloud);
	transformation_calculator.set_tgt_cloud(tgt_cloud);
	transformation_calculator.set_src_cloud_feature(source_FPFH.makeShared());
	transformation_calculator.set_tgt_cloud_feature(target_FPFH.makeShared());

	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::Matrix4f transformation;
	transformation_calculator.set_sample_number(10);
	//transformation_calculator.set_max_iteration(100);
	transformation_calculator.set_max_iteration(200);
	transformation_calculator.align(transformed_cloud);
	transformation = transformation_calculator.get_transformation_matrix();
	clock_t coarse_registration_time = clock();
	std::cout << "Compute the initial alignment time: " << (double)(coarse_registration_time - feature_time) / (double)CLOCKS_PER_SEC << " s" << std::endl;
	std::cout << "transformation matrix found by sac-ia" << transformation << std::endl;
	visualize_pcd(src_cloud, tgt_cloud, transformed_cloud);
	return 0;
}