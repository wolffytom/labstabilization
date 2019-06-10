#include "stdafx.h"
#include "VSAdjacentPointsFeatureModel.h"


VSAdjacentPointsFeatureModel::VSAdjacentPointsFeatureModel(int pVideoFramesCount)
{
	this->videoFramesCount = pVideoFramesCount;
	this->thisModelType = AdjacentPointsFeatureModel;
	
}


VSAdjacentPointsFeatureModel::~VSAdjacentPointsFeatureModel()
{
}
