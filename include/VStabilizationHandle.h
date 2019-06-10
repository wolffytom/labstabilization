#pragma once

#include "stdafx.h"
#include "VVideo.h"
#include "VTriCell.h"

using namespace std;

class VStabilizationHandle
{
public:
	//parameters
	const double _surfD = 400.0;
	const double _globalGoodPointRANSACFilter = 0.1;
	const double _globalGoodPointMinDISFilter = 0.7;
	const int _cellRows = 4;
	const int _cellCols = 4;
	const int _expectGoodPointsPerCells = 3;
	const double _reservedRate = 0.7;

	cv::Point2f **StandardControlPoints;
	cv::Point2f ***NewControlPoints;
	cv::Point2f ***InvControlPoints;
	cv::Point2f ***MonControlPoints;

	VVideo *Video;

	VStabilizationHandle(char *inputFileName);
	~VStabilizationHandle();
private:
	void _TESTShow();
	void _TESTShow2();
	void _TESTShow3();
	void TestConsole();

	void GetAllMatchedPoints();
	void GetAllMatchedPointsNormal();
	void GetAllMatchedPointsGPU();

	void GetGlobalGoodPoints();
	void GetGlobalGoodPointsWithRANSAC();
	void GetGlobalGoodPointsWithMinDis();

	void BuildCells();
	void BuildStandardControlPointsModel();
	void Calculate2DInterpolationWPerCell(VTriCell &refCell, int rowIndex, int colIndex);
	void Calculate2DInterpolationW();

	void EstimateMotion();

	void SelectMatchedPoints();
	void SelectACellPoints(VTriCell *pCell);

	void Stabilize();
	void VStabilizationHandle::StabilizeWithRecModel();

	//bool CmpDisForSort(const VMatchedPoint &mp1, const VMatchedPoint &mp2);
};

