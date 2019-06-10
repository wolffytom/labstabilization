#pragma once
#include "stdafx.h"

class VHomographyModel : public VProTransModel
{
public:
	VHomographyModel();
	VHomographyModel(double *parPar);
	VHomographyModel(cv::Point2f **fromPoint, cv::Point2f **toPoint);
	~VHomographyModel();
	cv::Point2f* HomographyTrans(cv::Point2f *fromPoint);
	void InitializePars();
	void BuildHomoTrans(VMatchedPoint **matchedPoints);
	void BuildHomoTrans(cv::Point2f **fromPoint, cv::Point2f **toPoint);
	VHomographyModel* GetInverse();

	double *H[2];
};

