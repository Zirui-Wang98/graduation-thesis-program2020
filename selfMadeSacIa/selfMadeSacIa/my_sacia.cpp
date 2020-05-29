#include "my_sacia.h"
#include <pcl/registration/transformation_estimation_svd.h>

my_sacia::my_sacia(int input_min_sample_point_distance, int input_max_iteration, int input_sample_number,
				   int input_k_randomness, double input_huber_loss_function_threshold)
{
	min_sample_point_distance = input_min_sample_point_distance;
	max_iteration = input_max_iteration;
	sample_number = input_sample_number;
	k_randomness = input_k_randomness;
	huber_loss_function_threshold = input_huber_loss_function_threshold;
}

my_sacia::~my_sacia()
{
	;
}

Eigen::Matrix4f my_sacia::get_transformation_matrix()
{
	return transformation_matrix;
}

void my_sacia::set_min_sample_point_distance(double input_min_sample_point_distance)
{
	min_sample_point_distance = input_min_sample_point_distance;
}

void my_sacia::set_max_iteration(int input_max_iteration)
{
	max_iteration = input_max_iteration;
}

void my_sacia::set_sample_number(int input_sample_number)
{
	sample_number = input_sample_number;
}

void my_sacia::set_k_randomness(int input_k_randomness)
{
	k_randomness = input_k_randomness;
}

void my_sacia::set_huber_loss_function_threshold(double input_huber_loss_function_threshold)
{
	huber_loss_function_threshold = input_huber_loss_function_threshold;
}

void my_sacia::set_src_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_src_cloud_ptr)
{
	src_cloud_ptr = input_src_cloud_ptr;
}

void my_sacia::set_src_cloud_feature(pcl::PointCloud<std::vector<double> >::Ptr input_src_cloud_feature_ptr)
{
	src_cloud_feature_ptr = input_src_cloud_feature_ptr;
}

void my_sacia::set_tgt_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_tgt_cloud_ptr)
{
	tgt_cloud_ptr = input_tgt_cloud_ptr;
}

void my_sacia::set_tgt_cloud_feature(pcl::PointCloud<std::vector<double> >::Ptr input_tgt_cloud_feature_ptr)
{
	tgt_cloud_feature_ptr = input_tgt_cloud_feature_ptr;
}

double distance(pcl::PointXYZ src_point, pcl::PointXYZ tgt_point, Eigen::Matrix4f transformation_matrix)
{
	double result = 0;
	Eigen::Vector4f src_vec(src_point.x, src_point.y, src_point.z, 1);
	Eigen::Vector4f transformed_src_vec;
	transformed_src_vec = transformation_matrix * src_vec;

	pcl::PointXYZ transformed_src_point;
	transformed_src_point.x = transformed_src_vec[0] / transformed_src_vec[3];
	transformed_src_point.y = transformed_src_vec[1] / transformed_src_vec[3];
	transformed_src_point.z = transformed_src_vec[2] / transformed_src_vec[3];

	result += (transformed_src_point.x - tgt_point.x) * (transformed_src_point.x - tgt_point.x);
	result += (transformed_src_point.y - tgt_point.y) * (transformed_src_point.y - tgt_point.y);
	result += (transformed_src_point.z - tgt_point.z) * (transformed_src_point.z - tgt_point.z);

	result = sqrt(result);
	return result;
}

double huber_punishment_function(pcl::PointCloud<pcl::PointXYZ>::Ptr src_match_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_match_ptr,
	Eigen::Matrix4f transformation_matrix, double threshold)
{
	double result = 0;
	double temp_distance = 0;

	for (int i = 0; i < src_match_ptr->size(); i++)
	{
		temp_distance = distance(src_match_ptr->points[i], tgt_match_ptr->points[i], transformation_matrix);
		if (temp_distance <= threshold) result += 1.f / 2 * temp_distance * temp_distance;
		else {
			result += threshold * 1.f / 2 * (2 * temp_distance - threshold);
		}
	}

	return result;
}

double distance(std::vector<double> src_feature, std::vector<double> tgt_feature)
{
	double result = 0;
	for (size_t i = 0; i < src_feature.size(); i++)
	{
		result += (src_feature[i] - tgt_feature[i]) * (src_feature[i] - tgt_feature[i]);
	}
	result = sqrt(result);
	return result;
}

int whether_tgt_can_be_find(int& target_match_index, pcl::PointCloud<std::vector<double> >::Ptr tgt_cloud_feature_ptr,
							std::vector<double> src_feature, int k_randomness)
{
	int result = 0;
	pcl::PointCloud<pcl::PointXYZ> distance_of_feature_cloud;
	distance_of_feature_cloud.width = tgt_cloud_feature_ptr->size();
	distance_of_feature_cloud.height = 1;
	distance_of_feature_cloud.is_dense = false;
	distance_of_feature_cloud.resize(distance_of_feature_cloud.width * distance_of_feature_cloud.height);

	for (int i = 0; i < tgt_cloud_feature_ptr->size(); i++)
	{
		distance_of_feature_cloud.points[i].x = distance(src_feature, tgt_cloud_feature_ptr->points[i]);
		distance_of_feature_cloud.points[i].y = 0;
		distance_of_feature_cloud.points[i].z = 0;
	}

	std::vector<int> point_index_k_nearest_search;
	std::vector<float> square_distance_k_nearest_search;

	pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;
	kdTree.setInputCloud(distance_of_feature_cloud.makeShared());

	pcl::PointXYZ origin_point;
	origin_point.x = 0;
	origin_point.y = 0;
	origin_point.z = 0;

	kdTree.nearestKSearch(origin_point, k_randomness, point_index_k_nearest_search, square_distance_k_nearest_search);
	if (point_index_k_nearest_search.size() > 0)
	{
		int temp_num = point_index_k_nearest_search.size();
		temp_num = temp_num * ((1.f * rand()) / (RAND_MAX + 1));
		target_match_index = point_index_k_nearest_search[temp_num];
		return 1;
	}
	return 0;
}

int is_the_source_point_index_usable(int source_point_index, std::vector<int>& point_not_for_choose)
{
	for (int i = 0; i < point_not_for_choose.size(); i++)
	{
		if (source_point_index == point_not_for_choose[i]) return 0;
	}
	return 1;
}

void my_sacia::compute_one_matched_point(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_match_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_cloud_match_ptr,
										 int current_index, pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdTree)
{
	int chosen_src_point_index = 0;
	int match_tgt_point_index = 0;
	int is_src_point_usable = 0;
	int is_tgt_can_be_find = 0;

	while (is_tgt_can_be_find == 0)	// repeat the process until the matched target cloud point is found
	{
		while (is_src_point_usable == 0)	
		{
			chosen_src_point_index = src_cloud_ptr->size() * ((1.0f * rand()) / (RAND_MAX + 1));	// randomly choose a source point's index
			is_src_point_usable = is_the_source_point_index_usable(chosen_src_point_index, point_not_for_choose);
		}

		is_tgt_can_be_find = whether_tgt_can_be_find(match_tgt_point_index, tgt_cloud_feature_ptr, src_cloud_feature_ptr->points[chosen_src_point_index], k_randomness);
		if (is_tgt_can_be_find == 0) point_not_for_choose.push_back(chosen_src_point_index);	// if no target match, then the chosen source point is abolished
	}
	src_cloud_match_ptr->points[current_index] = src_cloud_ptr->points[chosen_src_point_index];	// get one matched pair of points
	tgt_cloud_match_ptr->points[current_index] = tgt_cloud_ptr->points[match_tgt_point_index];

	if (min_sample_point_distance == 0)
	{
		point_not_for_choose.push_back(chosen_src_point_index);
	}
	else
	{	// if the min_sample_point_distance is greater than 0
		std::vector<int> point_index_radius_search;
		std::vector<float> square_distance_radius_search;
		kdTree->radiusSearch(src_cloud_ptr->points[chosen_src_point_index], min_sample_point_distance, point_index_radius_search, square_distance_radius_search);
		int temp_indicator = 0;
		for (int i = 0; i < point_index_radius_search.size(); i++)
		{
			temp_indicator = 0;
			temp_indicator = is_the_source_point_index_usable(point_index_radius_search[i], point_not_for_choose);	// check if the index is already in the point_not_for_choose vector
			if (temp_indicator) point_not_for_choose.push_back(point_index_radius_search[i]);
		}
	}
}

void my_sacia::compute_the_matched_pointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_match_ptr,
	pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_cloud_match_ptr)
{
	src_cloud_match_ptr->width = sample_number;	// sample number counts for the number of points to calculate the 
	src_cloud_match_ptr->height = 1;
	src_cloud_match_ptr->is_dense = false;
	src_cloud_match_ptr->resize(src_cloud_match_ptr->width * src_cloud_match_ptr->height);

	tgt_cloud_match_ptr->width = sample_number;
	tgt_cloud_match_ptr->height = 1;
	tgt_cloud_match_ptr->is_dense = false;
	tgt_cloud_match_ptr->resize(tgt_cloud_match_ptr->width * tgt_cloud_match_ptr->height);
	
	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdTree(new pcl::KdTreeFLANN<pcl::PointXYZ>() );
	kdTree->setInputCloud(src_cloud_ptr);	//kdTree with input cloud of source point cloud

	for (int i = 0; i < sample_number; i++)
	{
		compute_one_matched_point(src_cloud_match_ptr, tgt_cloud_match_ptr, i, kdTree);		// get each pair of point match
	}
}

void transform_cloud(Eigen::Matrix4f transform_matrix, pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_ptr,
					 pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud)
{
	Eigen::Vector4f temp_vector;
	for (int i = 0; i < source_cloud_ptr->size(); i++)
	{
		temp_vector[0] = source_cloud_ptr->points[i].x;
		temp_vector[1] = source_cloud_ptr->points[i].y;
		temp_vector[2] = source_cloud_ptr->points[i].z;
		temp_vector[3] = 1;
		temp_vector = transform_matrix * temp_vector;
		final_cloud->points[i].x = temp_vector[0] / temp_vector[3];
		final_cloud->points[i].y = temp_vector[1] / temp_vector[3];
		final_cloud->points[i].z = temp_vector[2] / temp_vector[3];
	}
}

void my_sacia::align(pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_match_ptr(new pcl::PointCloud<pcl::PointXYZ>() );
	pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_cloud_match_ptr(new pcl::PointCloud<pcl::PointXYZ>());

	double min_huber_loss = INFINITE;
	double temp_huber_loss = 0;
	Eigen::Matrix4f temp_transformation_matrix;
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> SVD_tranfomation_calculator;

	for (int i = 0; i < max_iteration; i++)
	{
		compute_the_matched_pointCloud(src_cloud_match_ptr, tgt_cloud_match_ptr);	// to get the matched point pairs
		SVD_tranfomation_calculator.estimateRigidTransformation(*src_cloud_match_ptr, *tgt_cloud_match_ptr, temp_transformation_matrix);	// get the transformation matrix from src to target
		temp_huber_loss = huber_punishment_function(src_cloud_match_ptr, tgt_cloud_match_ptr, temp_transformation_matrix, huber_loss_function_threshold);
		if (temp_huber_loss < min_huber_loss)
		{
			min_huber_loss = temp_huber_loss;
			transformation_matrix = temp_transformation_matrix;	// when huber loss is down the transformation matrix is changed
			point_not_for_choose.clear();
		}
	}
	huber_loss = min_huber_loss;
	transformed_cloud->width = src_cloud_ptr->width;
	transformed_cloud->height = src_cloud_ptr->height;
	transformed_cloud->is_dense = false;
	transformed_cloud->resize(transformed_cloud->width * transformed_cloud->height);
	transform_cloud(transformation_matrix, src_cloud_ptr, transformed_cloud);
}
