#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include "my_fpfh.h"
#include "match_model.h"
#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <time.h>

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

//点云可视化
void visualize_pcd(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr matched_cloud_ptr)
{
	// Create a PCLVisualizer object
	pcl::visualization::PCLVisualizer viewer("registration Viewer");
	viewer.setBackgroundColor(255, 255, 255);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(source_cloud_ptr, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> matched_h(matched_cloud_ptr, 255, 0, 0);
	viewer.addPointCloud(source_cloud_ptr, src_h, "source cloud");
	viewer.addPointCloud(matched_cloud_ptr, matched_h, "matched cloud");
	//viewer.addCoordinateSystem(1.0);
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

int main()
{
	std::string bunModel = "C:/Users/wangz/Desktop/models/bun.pcd";	// mesh resolution 0.001
	std::string happyModel = "C:/Users/wangz/Desktop/models/happy.pcd"; // mesh resolution 0.00045
	std::string dragonModel = "C:/Users/wangz/Desktop/models/dragon.pcd"; // mesh resolution 0.0006
	std::string scene = "C:/Users/wangz/Desktop/models/happyScene.pcd"; // mesh resolution 0.0013

	match_model match;
	pcl::PointCloud<std::vector<double> > model_FPFH;
	pcl::PointCloud<std::vector<double> > scene_FPFH;

	clock_t start = clock();

	scene_FPFH = computeFPFH(scene, 0.01, 0.003, 11);

	model_FPFH = computeFPFH(bunModel, 0.01, 0.003, 11);
	match.add_model(model_FPFH.makeShared());

	model_FPFH = computeFPFH(happyModel, 0.01, 0.003, 11);
	match.add_model(model_FPFH.makeShared());

	model_FPFH = computeFPFH(dragonModel, 0.01, 0.003, 11);
	match.add_model(model_FPFH.makeShared());

	clock_t featureTime = clock();
	std::cout << "Calculate FPFH feature time: " << (double)(featureTime - start) / (double)CLOCKS_PER_SEC << " s" << std::endl;

	double result = 0;
	std::vector<int> index_to_match;
	for (int i = 0; i < 200; i++) index_to_match.push_back(i);

	std::vector<int> matchedIndex;
	result = match.match(scene_FPFH.makeShared(), 2, index_to_match, matchedIndex);

	clock_t matchTime = clock();
	std::cout << "Calculate 200 points' match time: " << (double)(matchTime - featureTime) / (double)CLOCKS_PER_SEC << " s" << std::endl;
	std::cout << "Model Match success rate: " << result << std::endl;
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile <pcl::PointXYZ>(happyModel.c_str(), *source_cloud_ptr);

	pcl::PointCloud<pcl::PointXYZ>::Ptr matched_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	matched_cloud_ptr->height = 1;
	matched_cloud_ptr->width = matchedIndex.size();
	matched_cloud_ptr->is_dense = false;
	matched_cloud_ptr->resize(matched_cloud_ptr->height * matched_cloud_ptr->width);
	for (int i = 0; i < matchedIndex.size(); i++)
	{
		matched_cloud_ptr->points[i] = source_cloud_ptr->points[matchedIndex[i]];
	}

	visualize_pcd(source_cloud_ptr, matched_cloud_ptr);
	return 0;
}