#pragma once
#include "stdafx.h"
#include "V2DVector.h"
#include "VVideo.h"

class VVideo;

class VRecCellModel
{
public:
	int IteratorTimes = 10;

	int CellRows;
	int CellCols;
	VVideo *pVideo;
	V2DVector ****F;
	V2DVector ****C;
	V2DVector ****P;
	V2DVector ****Plast;
	V2DVector ****B;
	V2DVector ****BM;
	cv::Point2f ***recCenter;

	VRecCellModel(VVideo *parPVideo);
	void InitializeRecCellModel();
	void Iterate();
	void CalculateB();
	double GaussFunc(double stdv, double x);
	cv::Point2f* GetSrcPosition(int iFrame, int x, int y);
	~VRecCellModel();
	void OutputPToFile(int pRow, int pCol, string filename, bool isX);
};
