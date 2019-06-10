#pragma once
class VProTransModel
{
public:

	enum ModelTypes { Homography, Translation };
	ModelTypes ModelType;
	double *Par;
	int ParCount;

	VProTransModel();
	~VProTransModel();
	cv::Point2f* Trans(cv::Point2f *fromPoint);

	static double GetEucDis(VProTransModel *trans1, VProTransModel *trans2);
};

