#include "match_model.h"

match_model::match_model()
{
	;
}

match_model::~match_model()
{
	;
}

void match_model::add_model(pcl::PointCloud<std::vector<double> >::Ptr model_FPFH)
{
	model_FPFH_database.push_back(model_FPFH);
	model_FPFH_counts.push_back(model_FPFH->size());
}

double compute_distance(std::vector<double> descriptor1, std::vector<double> descriptor2)
{
	double res = 0.0;
	for (int i = 0; i < descriptor1.size(); i++)
	{
		res = res + (descriptor1[i] - descriptor2[i]) * (descriptor1[i] - descriptor2[i]);
	}
	return sqrt(res);
}

int match_one_point(const std::vector<pcl::PointCloud<std::vector<double> >::Ptr>& model_FPFH_database, const std::vector<long int>& model_FPFH_counts, 
					std::vector<double> descriptor, int realMatch, int& index)
{	
	std::vector<double> result_distance;
	
	double temp_distance = 0.0;
	for (int i = 0; i < model_FPFH_database.size(); i++)
	{
		for (int j = 0; j < model_FPFH_database[i]->size(); j++)
		{
			temp_distance = compute_distance(model_FPFH_database[i]->points[j], descriptor);
			result_distance.push_back(temp_distance);
		}
	}

	long int nearest_point_index = 0;
	double distance = result_distance[0];

	for (long int i = 0; i < result_distance.size(); i++)
	{
		if (distance > result_distance[i])
		{
			nearest_point_index = i;
			distance = result_distance[i];
		}
	}

	long int lower_bound(0), upper_bound(0);
	for (int i = 0; i < realMatch - 1; i++) lower_bound += model_FPFH_counts[i];
	upper_bound = lower_bound + model_FPFH_counts[realMatch - 1];

	if (nearest_point_index < upper_bound && nearest_point_index >= lower_bound)
	{
		index = nearest_point_index - lower_bound;
		return 1;
	}
	return 0;
}

double match_model::match(pcl::PointCloud<std::vector<double> >::Ptr scene_FPFH, int real_match, std::vector<int> point_index_to_match, std::vector<int>& matched_index)
{
	double result = 0;
	int index = -1;
	for (int i = 0; i < point_index_to_match.size(); i++)
	{
		index = -1;
		result += match_one_point(model_FPFH_database, model_FPFH_counts, scene_FPFH->points[point_index_to_match[i]], real_match, index);
		if (index >= 0)
		{
			matched_index.push_back(index);
		}
	}

	result = result / (double)point_index_to_match.size();
	return result;
}
