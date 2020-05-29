#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <iostream>
#include <time.h>
#include "my_fpfh.h"

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

int main()
{
	std::string bunModel = "C:/Users/wangz/Desktop/models/bun.pcd";	// mesh resolution 0.001
	std::string happyModel = "C:/Users/wangz/Desktop/models/happy.pcd"; // mesh resolution 0.00045
	std::string dragonModel = "C:/Users/wangz/Desktop/models/dragon.pcd"; // mesh resolution 0.0006
	std::string scene = "C:/Users/wangz/Desktop/models/happyScene.pcd"; // mesh resolution 0.0013

	pcl::PointCloud<std::vector<double> > fpfh_result;

	clock_t start = clock();
	fpfh_result = computeFPFH(happyModel, 0.01, 0.003, 11);
	clock_t feature_time = clock();
	
	std::cout << "feature time is: " << (feature_time - start) / CLOCKS_PER_SEC << " s" << std::endl;

	double temp_sum;
	for (int i = 0; i < 3; i++)
	{
		temp_sum = 0;
		for (int j = 0; j < 11; j++)
		{
			std::cout << fpfh_result.points[0][i * 11 + j] << " ";
		}
		std::cout << std::endl;
		std::cout << temp_sum << std::endl;
	}
	return 0;
}