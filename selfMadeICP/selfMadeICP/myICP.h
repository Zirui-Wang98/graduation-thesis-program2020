#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

class my_ICP
{
public:
	my_ICP(double input_mean_square_error_difference_boundary = 0.01);
	~my_ICP();
	void set_source_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_source_cloud_ptr);
	void set_target_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_target_cloud_ptr);
	void set_mean_square_error_difference_boundary(double input_mean_square_error_difference_boundary = 1e-10);
	Eigen::Matrix4f get_final_transformation();
	void compute_correspondence(pcl::PointCloud<pcl::PointXYZ>::Ptr match_target_cloud_ptr, pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree,
								pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_match_ptr);	// find the closest match point for each transformed source cloud point
	void align(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_after_transformation, Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity() );	// find the final transformation matrix

private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_ptr;
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_ptr;	// target cloud can have less points or more points than source cloud 
	pcl::PointCloud<pcl::PointXYZ>::Ptr match_target_cloud_ptr;	// match target cloud should have same points as source cloud points
	Eigen::Matrix4f final_transformation;
	double mean_square_error_difference_boundary;	
};