#include <iostream>
#include <pcl/io/pcd_io.h>
#include "my_spin_image.h"
#include <pcl/features/normal_3d.h>

int main()
{
	std::string filename = "C:/Users/wangz/Desktop/models/bun.pcd";	// mesh resolution 0.001
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
	normal_estimation.setRadiusSearch(0.01);
	normal_estimation.compute(*normals);

	// Setup spin image computation
	my_spin_image spin_image(0.001, 8, 8, 60);
	spin_image.set_cloud(cloud);
	spin_image.set_normal(normals);

	pcl::PointCloud<std::vector<double> >::Ptr spin_image_descriptor(new pcl::PointCloud<std::vector<double> >);
	spin_image.compute_spin_image(spin_image_descriptor);

	for (int i = 0; i < 17; i++)
	{
		for (int j = 0; j < 9; j++)
			std::cout << spin_image_descriptor->points[0][i * 8 + j] << "  ";
		std::cout << std::endl;
	}
	return 0;
}