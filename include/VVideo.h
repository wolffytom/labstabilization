#pragma once

#include "stdafx.h"
#include "VTriCell.h"
#include "VRecCellModel.h"

class VMotionModel;

class VVideo
{
public:
	char FileName[255];
	int CellRows;
	int CellCols;
	float CellHeight;
	float CellWidth;
	VTriCell ***Cells;

	cv::VideoCapture Capture;
	double Fps;
	int Width;
	int Height;
	int FramesCount;

	vector<VMatchedPoint> *AllMatchedPoints;//the former goodMatchedPointsCount Points Are Good Points
	int *AllMatchedPointsCount;
	int *GoodMatchedPointsCount;
	//cv::Mat *ColorPic;

	VMotionModel **MotionModel;
	VMotionModel **ReviseModel;
	VRecCellModel *recCellModel;

	VVideo(char *filename, int cellRows, int cellCols);
	void RefreshCapture();
	~VVideo();
	void PrintVideoInfo();
};

