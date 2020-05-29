#include "my_fpfh.h"
#include <pcl/kdtree/kdtree_flann.h>

#ifndef PI
#define PI 3.14159265
#endif

my_fpfh::my_fpfh(int input_subdivision)
{
	subdivision = input_subdivision;
}

my_fpfh::~my_fpfh()
{
}

void my_fpfh::set_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
{
	cloud = input_cloud;
}

void my_fpfh::set_normal(pcl::PointCloud<pcl::Normal>::Ptr input_normals)
{
	normals = input_normals;
}

void my_fpfh::set_radius(double input_radius)
{
	radius = input_radius;
}

void compute_one_spfh(pcl::PointCloud<std::vector<double> >::Ptr spfh, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
									 pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree,
									 int subdivision, double radius, int index)
{
	std::vector<double> result_spfh;
	result_spfh.resize(3 * subdivision);
	
	for (int i = 0; i < 3 * subdivision; i++)
		result_spfh[i] = 0;

	std::vector<int> point_index_radius_search;
	std::vector<float> point_radius_squared_distance;

	pcl::PointXYZ target_point = cloud->points[index];
	kdtree->radiusSearch(target_point, radius, point_index_radius_search, point_radius_squared_distance);

	Eigen::Vector3f u, v, w;
	Eigen::Vector3f pi, pj;
	Eigen::Vector3f ni, nj;
	Eigen::Vector3f pi_to_pj, temp_vec3f;

	double alpha, phi, theta;
	double alpha_segment, phi_segment, theta_segment;
	alpha_segment = 2.f / subdivision;
	phi_segment = 2.f / subdivision;
	theta_segment = 2.f * PI / subdivision;

	int region_id;

	if (point_index_radius_search.size() == 1)
	{
		std::cerr << "Please raise the radius value!" << std::endl;
		exit(1);
	}
	else
	{
		for (int i = 1; i < point_index_radius_search.size(); i++)
		{
			pi.x() = cloud->points[index].x;
			pi.y() = cloud->points[index].y;
			pi.z() = cloud->points[index].z;
			ni.x() = normals->points[index].normal_x;
			ni.y() = normals->points[index].normal_y;
			ni.z() = normals->points[index].normal_z;

			pj.x() = cloud->points[point_index_radius_search[i] ].x;
			pj.y() = cloud->points[point_index_radius_search[i] ].y;
			pj.z() = cloud->points[point_index_radius_search[i] ].z;
			nj.x() = normals->points[point_index_radius_search[i] ].normal_x;
			nj.y() = normals->points[point_index_radius_search[i] ].normal_y;
			nj.z() = normals->points[point_index_radius_search[i] ].normal_z;

			pi_to_pj.x() = pj.x() - pi.x();
			pi_to_pj.y() = pj.y() - pi.y();
			pi_to_pj.z() = pj.z() - pi.z();

			pi_to_pj = pi_to_pj.normalized();

			if (ni.dot(pi_to_pj) < (-1 * nj.dot(pi_to_pj) ) )	// if pj has smaller angle, change pi and pj 
			{
				temp_vec3f = pi;
				pi = pj;
				pj = temp_vec3f;

				temp_vec3f = ni;
				ni = nj;
				nj = temp_vec3f;

				pi_to_pj = -pi_to_pj;
			}

			u = ni;
			v = pi_to_pj.cross(u);
			v = v.normalized();
			w = u.cross(v);
			w = w.normalized();

			alpha = v.dot(nj);		// between -1 and 1
			phi = u.dot(pi_to_pj);	// between -1 and 1
			theta = atan2(w.dot(nj), u.dot(nj));	// between -pi and pi

			region_id = (alpha + 1) / alpha_segment;
			result_spfh[region_id] += 1;
			region_id = (phi + 1) / phi_segment;
			result_spfh[subdivision + region_id] += 1;
			region_id = (theta + PI) / theta_segment;
			result_spfh[subdivision * 2 + region_id] += 1;
		}
	}

	for (int i = 0; i < subdivision; i++)		// transform to percentage values
	{
		result_spfh[i] = result_spfh[i] / (point_index_radius_search.size() - 1) * 100;
		result_spfh[i + subdivision] = result_spfh[i + subdivision] / (point_index_radius_search.size() - 1) * 100;
		result_spfh[i + 2 * subdivision] = result_spfh[i + 2 * subdivision] / (point_index_radius_search.size() - 1) * 100;
	}

	spfh->points[index] = result_spfh;
}

void comput_one_fpfh(pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree, pcl::PointCloud<std::vector<double> >::Ptr fpfh_ptr,
					 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double radius, int subdivision,
					 pcl::PointCloud<std::vector<double> >::Ptr spfh_ptr, int index)
{
	std::vector<double> result_fpfh;
	result_fpfh = spfh_ptr->points[index];

	pcl::PointXYZ target = cloud->points[index];
	std::vector<int> point_index_radius_search;
	std::vector<float> point_radius_squared_distance;
	kdtree->radiusSearch(target, radius, point_index_radius_search, point_radius_squared_distance);

	std::vector<double> weight;
	weight.resize(point_index_radius_search.size() - 1);	// skip the first index, as it should be exactly the target point's index

	for (int i = 0; i < weight.size(); i++)
	{
		weight[i] = 1 / sqrt(point_radius_squared_distance[i + 1]) / weight.size();
	}

	for (int i = 0; i < weight.size(); i++)
	{
		for (int j = 0; j < 3 * subdivision; j++)
		{ 
			result_fpfh[j] = result_fpfh[j] + spfh_ptr->points[point_index_radius_search[i + 1]][j] * weight[i];
		}
	}

	double temp_sum;

	for (int i = 0; i < 3; i++)
	{
		temp_sum = 0;
		for (int j = 0; j < subdivision; j++)
			temp_sum += result_fpfh[j + i * subdivision];

		for (int j = 0; j < subdivision; j++)
			result_fpfh[j + i * subdivision] = result_fpfh[j + i * subdivision] / temp_sum * 100;
	}
	fpfh_ptr->points[index] = result_fpfh;
}

void my_fpfh::compute_fpfh(pcl::PointCloud<std::vector<double> >::Ptr result_fpfh_ptr)
{
	result_fpfh_ptr->width = cloud->width;
	result_fpfh_ptr->height = cloud->height;
	result_fpfh_ptr->is_dense = false;
	result_fpfh_ptr->resize(result_fpfh_ptr->width * result_fpfh_ptr->height);

	pcl::PointCloud<std::vector<double> >::Ptr spfh_ptr(new pcl::PointCloud<std::vector<double> >);
	spfh_ptr->width = cloud->width;
	spfh_ptr->height = cloud->height;
	spfh_ptr->is_dense = false;
	spfh_ptr->resize(spfh_ptr->width * spfh_ptr->height);

	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
	kdtree->setInputCloud(cloud);

	for (int i = 0; i < spfh_ptr->size(); i++)
		compute_one_spfh(spfh_ptr, cloud, normals, kdtree, subdivision, radius, i);

	for (int i = 0; i < result_fpfh_ptr->size(); i++)
	{
		comput_one_fpfh(kdtree, result_fpfh_ptr, cloud, radius, subdivision, spfh_ptr, i);
	}
}
