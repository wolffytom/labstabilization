#pragma once
#include "VSData.h"
class VSFeatureModel :
	public VSData
{
public:
	enum VSFeatureModelType{AdjacentPointsFeatureModel = 1};
	VSFeatureModelType thisModelType;
	int videoFramesCount;

	VSFeatureModel();
	~VSFeatureModel();
};

