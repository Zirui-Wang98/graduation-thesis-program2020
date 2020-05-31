#include <pcl/registration/transformation_estimation_svd.h>
#include "myICP.h"

my_ICP::my_ICP(double input_mean_square_error_difference_boundary)
{
	mean_square_error_difference_boundary = input_mean_square_error_difference_boundary;
}

my_ICP::~my_ICP()
{
	;
}

void my_ICP::set_source_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_source_cloud_ptr)
{
	source_cloud_ptr = input_source_cloud_ptr;
}

void my_ICP::set_target_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_target_cloud_ptr)
{
	target_cloud_ptr = input_target_cloud_ptr;
}

void my_ICP::set_mean_square_error_difference_boundary(double input_mean_square_error_difference_boundary)
{
	mean_square_error_difference_boundary = input_mean_square_error_difference_boundary;
}

Eigen::Matrix4f my_ICP::get_final_transformation()
{
	return final_transformation;
}

double calculate_distance(pcl::PointXYZ point_one, pcl::PointXYZ point_two)
{
	double distance = 0;
	distance = (point_one.x - point_two.x) * (point_one.x - point_two.x) + (point_one.y - point_two.y) * (point_one.y - point_two.y)
		+ (point_one.z - point_two.z) * (point_one.z - point_two.z);
	distance = sqrt(distance);
	return distance;
}

int find_closest_match_point_index(pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree, pcl::PointXYZ source_point)
{
	std::vector<int> pointIdxKSearch;
	std::vector<float> pointKSquareDistance;
	kdtree->nearestKSearch(source_point, 1, pointIdxKSearch, pointKSquareDistance);
	return pointIdxKSearch[0];
}

void my_ICP::compute_correspondence(pcl::PointCloud<pcl::PointXYZ>::Ptr match_target_cloud_ptr,
									pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_match_ptr)
{
	for (int i = 0; i < match_target_cloud_ptr->size(); i++)
	{
		int index_of_match = 0;
		index_of_match = find_closest_match_point_index(kdtree, cloud_to_match_ptr->points[i]);
		match_target_cloud_ptr->points[i] = target_cloud_ptr->points[index_of_match];
	}
}

void transform_source_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_ptr, Eigen::Matrix4f transformation_matrix)
{
	Eigen::Vector4f result_vector;
	for (int i = 0; i < source_cloud_ptr->size(); i++)
	{
		result_vector[0] = source_cloud_ptr->points[i].x;
		result_vector[1] = source_cloud_ptr->points[i].y;
		result_vector[2] = source_cloud_ptr->points[i].z;
		result_vector[3] = 1;
		result_vector = transformation_matrix * result_vector;
		source_cloud_ptr->points[i].x = result_vector[0] / result_vector[3];
		source_cloud_ptr->points[i].y = result_vector[1] / result_vector[3];
		source_cloud_ptr->points[i].z = result_vector[2] / result_vector[3];
	}
}

double mean_square_error(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr match_cloud_ptr)
{
	double result = 0, distance = 0;
	for (int i = 0; i < source_cloud_ptr->size(); i++)
	{
		distance = calculate_distance(source_cloud_ptr->points[i], match_cloud_ptr->points[i]);
		result = result + distance * distance;
	}
	result = result / source_cloud_ptr->size();
	return result;
}

void my_ICP::align(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_after_transformation, Eigen::Matrix4f initial_guess)
{
	point_cloud_after_transformation->width = source_cloud_ptr->width;
	point_cloud_after_transformation->height = source_cloud_ptr->height;
	point_cloud_after_transformation->is_dense = false;
	point_cloud_after_transformation->resize(point_cloud_after_transformation->width * point_cloud_after_transformation->height);

	for (int i = 0; i < source_cloud_ptr->size(); i++)
	{
		point_cloud_after_transformation->points[i] = source_cloud_ptr->points[i];
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr match_target_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	match_target_cloud_ptr->width = source_cloud_ptr->width;
	match_target_cloud_ptr->height = source_cloud_ptr->height;
	match_target_cloud_ptr->is_dense = false;
	match_target_cloud_ptr->resize(match_target_cloud_ptr->width * match_target_cloud_ptr->height);

	transform_source_cloud(point_cloud_after_transformation, initial_guess);	// guess transformation
	final_transformation = initial_guess;
	double previous_mean_square_error = INFINITY, current_mean_square_error = INFINITY;

	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
	kdtree->setInputCloud(target_cloud_ptr);

	compute_correspondence(match_target_cloud_ptr, kdtree, point_cloud_after_transformation);	// compute the match point cloud for source point cloud
	current_mean_square_error = mean_square_error(point_cloud_after_transformation, match_target_cloud_ptr);	
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> SVD_tranfomation_calculator;
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 transformation_matrix;

	Eigen::Matrix4f temp_transformation;
	SVD_tranfomation_calculator.estimateRigidTransformation(*point_cloud_after_transformation, *match_target_cloud_ptr, temp_transformation);
	final_transformation = temp_transformation * final_transformation;
	transform_source_cloud(point_cloud_after_transformation, temp_transformation);	// first calculated transformation
	compute_correspondence(match_target_cloud_ptr, kdtree, point_cloud_after_transformation);
	previous_mean_square_error = current_mean_square_error;
	current_mean_square_error = mean_square_error(point_cloud_after_transformation, match_target_cloud_ptr);

	while ((previous_mean_square_error - current_mean_square_error) > mean_square_error_difference_boundary)
	{
		SVD_tranfomation_calculator.estimateRigidTransformation(*point_cloud_after_transformation, *match_target_cloud_ptr, temp_transformation);
		final_transformation = temp_transformation * final_transformation;
		transform_source_cloud(point_cloud_after_transformation, temp_transformation);
		compute_correspondence(match_target_cloud_ptr, kdtree, point_cloud_after_transformation);
		previous_mean_square_error = current_mean_square_error;
		current_mean_square_error = mean_square_error(point_cloud_after_transformation, match_target_cloud_ptr);
	}
}
