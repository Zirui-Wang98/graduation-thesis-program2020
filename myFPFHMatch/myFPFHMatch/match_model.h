#pragma once

#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class match_model {
public:

	match_model(void);
	~match_model(void);
	void add_model(pcl::PointCloud<std::vector<double> >::Ptr modelFPFH);
	double match(pcl::PointCloud<std::vector<double> >::Ptr scene_FPFH, int real_match, std::vector<int> point_index_to_match, std::vector<int>& matched_index);

protected:
	std::vector<pcl::PointCloud<std::vector<double> >::Ptr> model_FPFH_database;
	std::vector<long int> model_FPFH_counts;
};