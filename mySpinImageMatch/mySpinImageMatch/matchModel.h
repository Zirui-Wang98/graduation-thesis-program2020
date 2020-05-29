#pragma once

#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class match_model {
public:

	match_model(void);
	~match_model(void);
	void add_model(pcl::PointCloud<std::vector<double> >::Ptr modleSI);
	double match(pcl::PointCloud<std::vector<double> >::Ptr sceneSI, int realMatch, std::vector<int> point_index_vec, std::vector<int>& matched_index);	// number states for the usage of sceneSI

protected:
	std::vector<pcl::PointCloud<std::vector<double> >::Ptr> model_SI_database;
	std::vector<long int> model_SI_counts;
};
