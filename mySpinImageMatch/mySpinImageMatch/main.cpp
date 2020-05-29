#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include "matchModel.h"
#include "my_spin_image.h"
#include <pcl/visualization/cloud_viewer.h>
#include <time.h>

pcl::PointCloud<std::vector<double> > computeSpinImage(std::string filename, double normal_radius, double bin_size, int image_width,
													   int image_height, double support_angle)
{
	std::cout << "Reading " << filename << std::endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile <pcl::PointXYZ>(filename.c_str(), *cloud) == -1)
		// load the file
	{
		PCL_ERROR("Couldn't read file");
	}
	std::cout << "Loaded " << cloud->points.size() << " points." << std::endl;
	// Compute the normals
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
	normal_estimation.setInputCloud(cloud);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	normal_estimation.setSearchMethod(kdtree);

	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud< pcl::Normal>);
	normal_estimation.setRadiusSearch(normal_radius);
	normal_estimation.compute(*normals);

	// Setup spin image computation
	my_spin_image spin_image_calculator(bin_size, image_width, image_height, support_angle);
	spin_image_calculator.set_cloud(cloud);
	spin_image_calculator.set_normal(normals);

	pcl::PointCloud<std::vector<double> >::Ptr spin_images(new pcl::PointCloud<std::vector<double> >);

	// Actually compute the spin images
	spin_image_calculator.compute_spin_image(spin_images);
	return *spin_images;
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

int main(int argc, char** argv)
{
	match_model matchPoints;
	pcl::PointCloud<std::vector<double> > modelSI;
	pcl::PointCloud<std::vector<double> > sceneSI;

	std::string bunModel = "C:/Users/wangz/Desktop/models/bun.pcd";	// mesh resolution 0.001
	std::string happyModel = "C:/Users/wangz/Desktop/models/happy.pcd"; // mesh resolution 0.0045
	std::string dragonModel = "C:/Users/wangz/Desktop/models/dragon.pcd"; // mesh resolution 0.006
	std::string scene = "C:/Users/wangz/Desktop/models/happyScene.pcd"; // mesh resolution 0.0013

	clock_t start = clock();

	sceneSI = computeSpinImage(scene, 0.002, 0.001, 8, 8, 60);

	modelSI = computeSpinImage(bunModel, 0.002, 0.001, 8, 8, 60);
	matchPoints.add_model(modelSI.makeShared());

	modelSI = computeSpinImage(happyModel, 0.002, 0.001, 8, 8, 60);
	matchPoints.add_model(modelSI.makeShared());

	modelSI = computeSpinImage(dragonModel, 0.002, 0.001, 8, 8, 60);
	matchPoints.add_model(modelSI.makeShared());

	clock_t featureTime = clock();
	std::cout << "Calculate SpinImage feature time: " << (double)(featureTime - start) / (double)CLOCKS_PER_SEC << " s" << std::endl;

	std::vector<int> point_index_to_match;
	for (int i = 0; i < 200; i++)
	{
		point_index_to_match.push_back(i);
	}

	double res = 0;
	std::vector<int> matched_index;
	res = matchPoints.match(sceneSI.makeShared(), 2, point_index_to_match, matched_index);

	clock_t matchTime = clock();
	std::cout << "Calculate 200 points' match time: " << (double)(matchTime - start) / (double)CLOCKS_PER_SEC << " s" << std::endl;
	std::cout << res << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile <pcl::PointXYZ>(happyModel.c_str(), *source_cloud_ptr);

	pcl::PointCloud<pcl::PointXYZ>::Ptr matched_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	matched_cloud_ptr->height = 1;
	matched_cloud_ptr->width = matched_index.size();
	matched_cloud_ptr->is_dense = false;
	matched_cloud_ptr->resize(matched_cloud_ptr->height * matched_cloud_ptr->width);
	for (int i = 0; i < matched_index.size(); i++)
	{
		matched_cloud_ptr->points[i] = source_cloud_ptr->points[matched_index[i]];
	}

	visualize_pcd(source_cloud_ptr, matched_cloud_ptr);
	return 0;
}