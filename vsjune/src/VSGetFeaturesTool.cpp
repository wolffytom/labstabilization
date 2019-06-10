#include "stdafx.h"
#include "VSGetFeaturesTool.h"


VSGetFeaturesTool::VSGetFeaturesTool(VSVideo *pvideo, VSFeatureModel *pfeatureModel,
	VSFeatureModel::VSFeatureModelType pfeatureModelType)
{
	this->video = pvideo;
	this->featureModel = pfeatureModel;
	this->featureModelType = pfeatureModelType;

	switch (featureModelType)
	{
	case VSFeatureModel::AdjacentPointsFeatureModel:
	{
		this->getAdjacentPointsFeatureMethod = SURF;
		this->useGPU = 1;
		this->GetFeaturesForAdjacentPointsFeatureModel();
	}
		break;
	default:
		break;
	}
}


VSGetFeaturesTool::~VSGetFeaturesTool()
{
}

void VSGetFeaturesTool::GetFeaturesForAdjacentPointsFeatureModel()
{
	switch (this->GetFeaturesForAdjacentPointsFeatureModel)
	{
	case SURF:
	{
		GetFeaturesForAdjacentPointsFeatureModelWithSURF();
	}
		break;
	default:
		break;
	}
}

void VSGetFeaturesTool::GetFeaturesForAdjacentPointsFeatureModelWithSURF()
{

}

