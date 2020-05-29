#pragma once
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

class my_fpfh
{
public:
	my_fpfh(int input_subdivision = 11);
	~my_fpfh();
	void set_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
	void set_normal(pcl::PointCloud<pcl::Normal>::Ptr input_normals);
	void set_radius(double input_radius);
	void compute_fpfh(pcl::PointCloud<std::vector<double> >::Ptr result_fpfh_ptr);
private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	pcl::PointCloud<pcl::Normal>::Ptr normals;
	double radius;
	int subdivision;
};