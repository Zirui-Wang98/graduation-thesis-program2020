#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <pcl/kdtree/kdtree_flann.h>

class my_sacia
{
public:
	my_sacia(int input_min_sample_point_distance = 0, int input_max_iteration = 100, int input_sample_number = 10,
		int input_k_randomness = 1, double input_huber_loss_function_threshold = 0.01);
	~my_sacia();
	void align(pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud);
	void compute_the_matched_pointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_match_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_cloud_match_ptr);
	void compute_one_matched_point(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_match_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_cloud_match_ptr, int current_index, pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdTree);
	Eigen::Matrix4f get_transformation_matrix();	// after apply the compute_the_transformation_matrix()
	void set_min_sample_point_distance(double input_min_sample_point_distance);
	void set_max_iteration(int input_max_iteration);
	void set_sample_number(int input_sample_number);
	void set_k_randomness(int input_k_randomness);
	void set_huber_loss_function_threshold(double input_huber_loss_function_threshold);
	void set_src_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_src_cloud_ptr);
	void set_src_cloud_feature(pcl::PointCloud<std::vector<double> >::Ptr input_src_cloud_feature_ptr);
	void set_tgt_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_tgt_cloud_ptr);
	void set_tgt_cloud_feature(pcl::PointCloud<std::vector<double> >::Ptr input_tgt_cloud_feature_ptr);

private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_ptr;
	pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_cloud_ptr;
	pcl::PointCloud<std::vector<double> >::Ptr src_cloud_feature_ptr;
	pcl::PointCloud<std::vector<double> >::Ptr tgt_cloud_feature_ptr;
	int max_iteration;	// maximum number of iterations the internal optimization should run for
	double min_sample_point_distance;	// minimum distance between samples
	int sample_number;	// the number of samples to use during each iteration, must greater than 2
	int k_randomness;	// for each selected 
	double huber_loss_function_threshold;
	double huber_loss;
	Eigen::Matrix4f transformation_matrix;	// the computed transformation matrix
	std::vector<int> point_not_for_choose;
};