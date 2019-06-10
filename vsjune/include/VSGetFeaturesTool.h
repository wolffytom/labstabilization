#pragma once
class VSGetFeaturesTool
{
public:
	VSVideo *video;
	VSFeatureModel *featureModel;
	VSFeatureModel::VSFeatureModelType featureModelType;

	bool useGPU = 0;
	enum GetAdjacentPointsFeatureMethods{SURF = 1};
	GetAdjacentPointsFeatureMethods getAdjacentPointsFeatureMethod;

	VSGetFeaturesTool(VSVideo *pvideo, VSFeatureModel *pfeatureModel, 
		VSFeatureModel::VSFeatureModelType pfeatureModelType);

	void GetFeaturesForAdjacentPointsFeatureModel();
	void GetFeaturesForAdjacentPointsFeatureModelWithSURF();

	~VSGetFeaturesTool();
};

