#pragma once
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#ifndef PI 
#define PI 3.14159265
#endif

class my_spin_image
{
public:
	my_spin_image(double input_bin_size, int input_image_width = 8, int input_image_height = 8, double input_support_angle = 60);
	~my_spin_image(void);
	void set_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr);
	void set_normal(pcl::PointCloud<pcl::Normal>::Ptr input_normal_ptr);
	void compute_spin_image(pcl::PointCloud<std::vector<double> >::Ptr result_spin_image);

private:
	double bin_size;
	double support_angle;
	int image_width;	// stands for alpha and j
	int image_height;	// stands for beta and i
	pcl::PointCloud<pcl::Normal>::Ptr normal;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};