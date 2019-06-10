#include "stdafx.h"
#include "VProTransModel.h"


VProTransModel::VProTransModel()
{
}


VProTransModel::~VProTransModel()
{
	cout << "delete";
}

cv::Point2f* VProTransModel::Trans(cv::Point2f* fromPoint)
{
	switch (ModelType)
	{
	case VProTransModel::Homography:
		return ((VHomographyModel*)this)->HomographyTrans(fromPoint);
		break;
	case VProTransModel::Translation:
		return new cv::Point2f();
		break;
	default:
		return new cv::Point2f();
		break;
	}
}

double VProTransModel::GetEucDis(VProTransModel *trans1, VProTransModel *trans2)
{
	if (trans1->ModelType != trans2->ModelType)
	{
		cout << "Error: Different ModelType Compare!!!!";
		return -1;
	}
	double dis = 0;
	for (int i = 0; i < trans1->ParCount; i++)
	{
		dis += (trans1->Par[i] - trans2->Par[i]) * (trans1->Par[i] - trans2->Par[i]);
	}
	return dis;
}