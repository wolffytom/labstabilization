#pragma once
#include "VSFeatureModel.h"
class VSAdjacentPointsFeatureModel :
	public VSFeatureModel
{
public:
	VSAdjacentPointsFeatureModel(int pVideoFramesCount);
	~VSAdjacentPointsFeatureModel();
};

