#pragma once
#include "stdafx.h"

class VProTransModel;

class VMotionModel
{
public:
	double _alpha;
	double _beta = 0.01;
	double ErrorE;
	cv::Point2f **ControlPoints;
	int CellRows;
	int CellCols;
	int FrameIndex;
	VVideo *pVideo;
	VProTransModel ***TransModel;
	VProTransModel ***invTransModel;
	cv::Point2f **StandardControlPoints;

	VMotionModel(cv::Point2f **srcControlPoints, cv::Point2f **newControlPoints, VVideo *video);
	VMotionModel(int frameIndex, double parAlpha, VVideo *video, cv::Point2f **standardControlPoints);
	~VMotionModel();

	void InitializeTransModel();
	void BuildModel();
	void BuildModel_InitializeArray(int dimen, double *lA, double **A, double *B, double *X);
	int BuildModel_GetxIndex(int rowIndex, int colIndex);
	void BuildTransModel();
	void BuildModel_Ed(double **A, double *B, int iCellRow, int iCellCol, int *lVertexxIndex);
	void BuildModel_Es(double **A, double *B, int iCellRow, int iCellCol, int *lVertexxIndex);
	void BuildModel_Es_ATriangle(double **A, int VdxIndex, int V1xIndex, int V0xIndex, double s, double isClockWise);
	void CalculateError();
	cv::Point2f* GetInversePoint(int x, int y, int cellRow, int cellCol);
	void BuildInverseTransModel();
	double CalculateError_eh();
	double CalculateError_es();
};

