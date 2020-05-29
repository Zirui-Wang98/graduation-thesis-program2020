#include "my_spin_image.h"
#include <pcl/kdtree/kdtree_flann.h>

my_spin_image::my_spin_image(double input_bin_size, int input_image_width,
							 int input_image_height, double input_support_angle)
{
	bin_size = input_bin_size;
	image_width = input_image_width;
	image_height = input_image_height;
	support_angle = input_support_angle;
}

my_spin_image::	~my_spin_image(void)
{
	
}

void my_spin_image::set_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr)
{
	cloud = input_cloud_ptr;
}

void my_spin_image::set_normal(pcl::PointCloud<pcl::Normal>::Ptr input_normal_ptr)
{
	normal = input_normal_ptr;
}

std::vector<double> compute_one_point_spin_image(pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
												 pcl::PointCloud<pcl::Normal>::Ptr normal, double radius, int image_width, int image_height,
												 double bin_size, int support_angle, int index)
{
	std::vector<double> result_spin_image;
	result_spin_image.resize((image_width + 1 ) * (2 * image_height + 1 ) );
	std::vector<int> point_index_radius_search;
	std::vector<float> point_radius_squared_distance;
	double alpha_max = bin_size * image_width;
	double beta_max = bin_size * image_height;

	pcl::PointXYZ target_point = cloud->points[index];
	kdtree->radiusSearch(target_point, radius, point_index_radius_search, point_radius_squared_distance);
	if (point_index_radius_search.size() > 1)
	{
		double temp_alpha, temp_beta;
		int bin_i, bin_j;
		double real_bin_i, real_bin_j;
		double temp_cos_of_normals;
		pcl::PointXYZ temp_vector;
		double temp_squared_vector;
		double a, b;
		for (int i = 1; i < point_index_radius_search.size(); i++)
		{
			temp_cos_of_normals = normal->points[index].normal_x * normal->points[point_index_radius_search[i] ].normal_x +
				normal->points[index].normal_y * normal->points[point_index_radius_search[i] ].normal_y +
				normal->points[index].normal_z * normal->points[point_index_radius_search[i] ].normal_z;

			if (temp_cos_of_normals >= cos(support_angle / 180.f * PI))
			{
				temp_vector.x = cloud->points[point_index_radius_search[i] ].x - cloud->points[index].x;
				temp_vector.y = cloud->points[point_index_radius_search[i] ].y - cloud->points[index].y;
				temp_vector.z = cloud->points[point_index_radius_search[i] ].z - cloud->points[index].z;

				temp_squared_vector = pow(temp_vector.x, 2) + pow(temp_vector.y, 2) + pow(temp_vector.z, 2);

				temp_beta = normal->points[index].normal_x * temp_vector.x;
				temp_beta += normal->points[index].normal_y * temp_vector.y;
				temp_beta += normal->points[index].normal_z * temp_vector.z;

				temp_alpha = sqrt(temp_squared_vector - pow(temp_beta, 2));

				if (temp_alpha < alpha_max && abs(temp_beta) < beta_max)
				{
					real_bin_i = (beta_max - temp_beta) / bin_size;
					real_bin_j = temp_alpha / bin_size;
					bin_i = real_bin_i;
					bin_j = real_bin_j;
					
					a = real_bin_i - bin_i;
					b = real_bin_j - bin_j;

					result_spin_image[bin_i * (image_width + 1) + bin_j] += (1 - a) * (1 - b);		// (i,j)
					result_spin_image[bin_i * (image_width + 1) + bin_j + 1] += (1 - a) * b;		// (i, j+1)
					result_spin_image[(bin_i + 1) * (image_width + 1) + bin_j] += a * (1-b);		// (i+1, j)
					result_spin_image[(bin_i + 1) * (image_width + 1) + bin_j + 1] += a * b;	// (i+1, j+1)
				}
			}
		}
	}
	else
	{
		std::cerr << "Input bin_size is too small!" << std::endl;
		exit(1);
	}
	
	double temp_sum = 0;
	for (int i = 0; i < result_spin_image.size(); i++)
		temp_sum += result_spin_image[i];
	for (int i = 0; i < result_spin_image.size(); i++)
		result_spin_image[i] = result_spin_image[i] / temp_sum;

	return result_spin_image;
}

void my_spin_image::compute_spin_image(pcl::PointCloud<std::vector<double> >::Ptr result_spin_image)
{
	result_spin_image->width = cloud->width;
	result_spin_image->height = cloud->height;
	result_spin_image->is_dense = false;
	result_spin_image->resize(result_spin_image->width * result_spin_image->height);

	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
	kdtree->setInputCloud(cloud);


	double width = image_width * bin_size; // stands for alpha and j
	double height = image_height * bin_size; // stands for beta and i
	double radius = sqrt(pow(width, 2) + pow(height, 2));

	for (int i = 0; i < result_spin_image->size(); i++)
		result_spin_image->points[i] = compute_one_point_spin_image(kdtree, cloud, normal, radius, image_width, image_height, bin_size, support_angle, i);
}

