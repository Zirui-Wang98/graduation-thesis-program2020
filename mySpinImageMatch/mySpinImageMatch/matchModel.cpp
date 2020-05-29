#include "matchModel.h"
#include <pcl/kdtree/kdtree_flann.h>

match_model::match_model()
{
	;
}

match_model::~match_model()
{
	;
}

void match_model::add_model(pcl::PointCloud<std::vector<double> >::Ptr modleSI)
{
	model_SI_database.push_back(modleSI);
	model_SI_counts.push_back(modleSI->points.size());
}

double correlation(std::vector<double> scenePointSI, std::vector<double> modlePointSI)
{
	double res = 0;
	double tempNumerator(0.0), tempDenominator1(0.0), tempDenominator2(0.0);
	for (int i = 0; i < scenePointSI.size(); i++)
	{
		tempNumerator += (scenePointSI[i] - 1.f / scenePointSI.size()) * (modlePointSI[i] - 1.f / modlePointSI.size());
		tempDenominator1 += (scenePointSI[i] - 1.f / scenePointSI.size()) * (scenePointSI[i] - 1.f / scenePointSI.size());
		tempDenominator2 += (modlePointSI[i] - 1.f / modlePointSI.size()) * (modlePointSI[i] - 1.f / modlePointSI.size());
	}
	res = tempNumerator / (sqrt(tempDenominator1) * sqrt(tempDenominator2) );
	return res;
}

double similarity(std::vector<double> scene_point_SI, std::vector<double> modle_point_SI)
{
	double res = 0, tempCorrelation(0.0);
	tempCorrelation = correlation(scene_point_SI, modle_point_SI);
	res = atanh(tempCorrelation) - 3 / (scene_point_SI.size() - 3);
	return res;
}

int calculate_one_point_match(std::vector<pcl::PointCloud<std::vector<double> >::Ptr>& model_SI_database,
							  std::vector<long int>& model_SI_counts, std::vector<double> sceneSI, int& realMatch, int& matched_index)
{
	std::vector<double> similarities;
	for (int i = 0; i < model_SI_database.size(); i++)
	{
		for (int j = 0; j < model_SI_database[i]->size(); j++)
		{
			similarities.push_back(similarity(sceneSI, model_SI_database[i]->points[j]));
		}
	}

	double maxSimilarity = similarities[0];
	long int maxSimilarityIndex = 0;

	for (int i = 0; i < similarities.size(); i++)
	{
		if (similarities[i] > maxSimilarity)
		{
			maxSimilarity = similarities[i];
			maxSimilarityIndex = i;
		}
	}

	long int lowerBound(0), upperBound(0);
	for (int i = 0; i < realMatch - 1; i++)
	{
		lowerBound += model_SI_counts[i];
	}
	upperBound = lowerBound + model_SI_counts[realMatch - 1];

	if (maxSimilarity < 1.5) return -1;
	else if (maxSimilarityIndex < upperBound && maxSimilarityIndex >= lowerBound)
	{
		matched_index = maxSimilarityIndex - lowerBound;
		return 1;
	}
	return 0;
}


double match_model::match(pcl::PointCloud<std::vector<double> >::Ptr sceneSI, int realMatch, std::vector<int> point_index_vec, std::vector<int>& matched_index)
{
	int total = 0;
	int temp = 0;
	int denominator = point_index_vec.size();
	int temp_index = 0;
	for (int i = 0; i < point_index_vec.size(); i++)
	{
		temp = calculate_one_point_match(model_SI_database, model_SI_counts, sceneSI->points[point_index_vec[i]], realMatch, temp_index);
		if (temp == 1)
		{
			total++;
			matched_index.push_back(temp_index);
		}
	}
	return (double)total / denominator;
}
