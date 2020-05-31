#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <time.h>
#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include "match_model.h"

pcl::PointCloud<pcl::PFHSignature125> compute_pfh(std::string filename, double normal_radius, double pfh_radius)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(filename.c_str(), *cloud);

	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(normal_radius);
	ne.compute(*normals);

	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
	pfh.setInputCloud(cloud);
	pfh.setInputNormals(normals);
	pfh.setSearchMethod(tree);

	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfh_ptr(new pcl::PointCloud<pcl::PFHSignature125>());
	pfh.setRadiusSearch(pfh_radius);
	pfh.compute(*pfh_ptr);

	return *pfh_ptr;
}

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
	pcl::PointCloud<pcl::PFHSignature125 > model_PFH;
	pcl::PointCloud<pcl::PFHSignature125 > scene_PFH;

	clock_t start = clock();

	scene_PFH = compute_pfh(scene, 0.01, 0.005);

	model_PFH = compute_pfh(bunModel, 0.01, 0.005);
	match.add_model(model_PFH.makeShared());

	model_PFH = compute_pfh(happyModel, 0.01, 0.005);
	match.add_model(model_PFH.makeShared());

	model_PFH = compute_pfh(dragonModel, 0.01, 0.005);
	match.add_model(model_PFH.makeShared());

	clock_t featureTime = clock();
	std::cout << "Calculate PFH feature time: " << (double)(featureTime - start) / (double)CLOCKS_PER_SEC << " s" << std::endl;

	double result = 0;
	std::vector<int> index_to_match;
	for (int i = 0; i < 200; i++) index_to_match.push_back(i);

	std::vector<int> matchedIndex;
	result = match.match(scene_PFH.makeShared(), 2, index_to_match, matchedIndex);

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