#pragma once

#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class match_model {
public:

	match_model(void);
	~match_model(void);
	void add_model(pcl::PointCloud<pcl::PFHSignature125 >::Ptr model_PFH_ptr);
	double match(pcl::PointCloud<pcl::PFHSignature125 >::Ptr scene_PFH_ptr, int real_match, std::vector<int> point_index_to_match, std::vector<int>& matched_index);

protected:
	std::vector<pcl::PointCloud<pcl::PFHSignature125 >::Ptr> model_PFH_database;
	std::vector<long int> model_PFH_counts;
};