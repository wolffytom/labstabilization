#pragma once
#include "VRecCellModel.h"
class VResample
{
public:

	static cv::Mat GetResampleMat(VMotionModel *reviseModel, cv::Mat &oriMat);
	static cv::Mat GetResampleMatByInvTrans(VMotionModel *reviseModel, cv::Mat &oriMat, double reservedRate);
	static void BilinearInter(uchar *data, cv::Mat &From, double nx, double ny, int width, int height);
	static int VResample::GetPixel(int b, cv::Mat &From, int x, int y);
	static cv::Mat GetResampleMatByRecCellModel(VRecCellModel *recCellModel, cv::Mat &oriMat, int frameIndex);
	static void ThreeOrderInter(uchar *data, cv::Mat &From, double nx, double ny, int width, int height);

	VResample();
	~VResample();
};

