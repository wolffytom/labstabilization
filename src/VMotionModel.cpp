#include "stdafx.h"
#include "VMotionModel.h"
#include "VLinearEquationsSolver.h"


VMotionModel::VMotionModel(cv::Point2f **srcControlPoints, cv::Point2f **newControlPoints, VVideo *video)
{
	pVideo = video;
	CellRows = pVideo->CellRows;
	CellCols = pVideo->CellCols;
	StandardControlPoints = srcControlPoints;
	ControlPoints = newControlPoints;

	TransModel = new VProTransModel**[CellRows];
	for (int iRow = 0; iRow < CellRows; iRow++)
	{
#ifdef TRIANGULAR_DEVIDE_POINTS
		TransModel[iRow] = new VProTransModel*[CellCols * 2];
#else
		TransModel[iRow] = new VProTransModel*[CellCols];
#endif
	}

	BuildTransModel();

	//BuildInverseTransModel();
}

VMotionModel::VMotionModel(int frameIndex, double parAlpha, VVideo *video, cv::Point2f **standardControlPoints)
{
	StandardControlPoints = standardControlPoints;
	FrameIndex = frameIndex;
	_alpha = parAlpha;
	pVideo = video;
	CellRows = pVideo->CellRows;
	CellCols = pVideo->CellCols;

	ControlPoints = new cv::Point2f*[CellRows + 1];
	for (int iRow = 0; iRow < CellRows + 1; iRow++)
	{
		ControlPoints[iRow] = new cv::Point2f[CellRows + 1];
	}
	
	TransModel = new VProTransModel**[CellRows];
	for (int iRow = 0; iRow < CellRows; iRow++)
	{
#ifdef TRIANGULAR_DEVIDE_POINTS
		TransModel[iRow] = new VProTransModel*[CellCols * 2];
#else
		TransModel[iRow] = new VProTransModel*[CellCols];
#endif
	}

	BuildModel();

	BuildTransModel();

	CalculateError();
}

VMotionModel::~VMotionModel()
{
	for (int iRow = 0; iRow < CellRows + 1; iRow++)
	{
		delete[]ControlPoints[iRow];
	}
	delete[]ControlPoints;

	for (int iRow = 0; iRow < CellRows; iRow++)
	{
		delete[]TransModel[iRow];
	}
	delete[]TransModel;
}


void VMotionModel::BuildModel_InitializeArray(int dimen, double *lA, double **A, double *B, double *X)
{
	for (int rowA = 0; rowA < dimen; rowA++)
	{
		B[rowA] = X[rowA] = 0.0;
		A[rowA] = &lA[rowA * dimen];
		for (int colA = 0; colA < dimen; colA++)
			A[rowA][colA] = 0.0;
	}
}

/**
* @brief Get x's index of one cell in the Functions
**/
int VMotionModel::BuildModel_GetxIndex(int rowIndex, int colIndex)
{
	return (rowIndex * (CellCols + 1) + colIndex) * 2;
}

/**
* @brief Build one cell's Es
**/
void VMotionModel::BuildModel_Ed(double **A, double *B, int iCellRow, int iCellCol, int *lVertexxIndex)
{
	//-------------------initialize the iterator-------------------------
	std::vector<VMatchedPoint>::iterator itPoint;
	std::vector<VMatchedPoint>::iterator itEnd;
#ifdef TRIANGULAR_DEVIDE_POINTS
	itPoint = pVideo->Cells[FrameIndex][iCellRow][iCellCol * 2].MatchedPoints.begin();
	itEnd = itPoint + pVideo->Cells[FrameIndex][iCellRow][iCellCol * 2].GoodMatchedCount;
	bool nextHalf = false;
#else
	itPoint = pVideo->Cells[FrameIndex][iCellRow][iCellCol].MatchedPoints.begin();
	itEnd = itPoint + pVideo->Cells[FrameIndex][iCellRow][iCellCol].GoodMatchedCount;
#endif

	//------------------for Each matched point in this cell-------------------
	while (true)
	{
		// judge if break
		if (itPoint == itEnd)
		{
#ifdef TRIANGULAR_DEVIDE_POINTS
			if (nextHalf == false)//if done the upper half, do the lower half
			{
				nextHalf = true;
				itPoint = pVideo->Cells[FrameIndex][iCellRow][iCellCol * 2 + 1].MatchedPoints.begin();
				itEnd = itPoint + pVideo->Cells[FrameIndex][iCellRow][iCellCol * 2 + 1].GoodMatchedCount;
				continue;
			}
			else break;//if done the lower half, break
#else
			break;
#endif
		}

		double *lw = &(itPoint->w[0][0]);//linear array of 4 w
		for (int iEquaVertex = 0; iEquaVertex < 4; iEquaVertex++)
		{
			for (int iVarVertex = 0; iVarVertex < 4; iVarVertex++)
			{
				//x
				A[lVertexxIndex[iEquaVertex]][lVertexxIndex[iVarVertex]]
					+= lw[iEquaVertex] * lw[iVarVertex] * 2;
				//y
				A[1 + lVertexxIndex[iEquaVertex]][1 + lVertexxIndex[iVarVertex]]
					+= lw[iEquaVertex] * lw[iVarVertex] * 2;
			}

			//x
			B[lVertexxIndex[iEquaVertex]] += lw[iEquaVertex] * itPoint->DstPoint.x * 2;
			//y
			B[1 + lVertexxIndex[iEquaVertex]] += lw[iEquaVertex] * itPoint->DstPoint.y * 2;
		}

		itPoint++;//Next point
	}
}

//Vd: Compare point
//V1: Right Angle point
//v0: another point
//s: ||Vd-V0||/||V1-V0||
//isClockWise: value of 1.0 or -1.0
//             is Wise of [vector(V0-V1) to vector(Vd-V1)] is clockwise,
//             same as the wise of Vd-V0-V1
void VMotionModel::BuildModel_Es_ATriangle(double **A, int VdxIndex, int V1xIndex, int V0xIndex, double s, double isClockWise)
{
	double signX = isClockWise;
	double signY = -isClockWise;

	//EquaVdx
	A[VdxIndex][VdxIndex] += (_alpha * 2);
	A[VdxIndex][V1xIndex] += (_alpha * -2);
	A[VdxIndex][V0xIndex + 1] += (_alpha * -2 * s * signX);
	A[VdxIndex][V1xIndex + 1] += (_alpha * 2 * s * signX);
	//EquaV1x
	A[V1xIndex][VdxIndex] += (_alpha * 2) * -1;
	A[V1xIndex][V1xIndex] += (_alpha * -2) * -1;
	A[V1xIndex][V0xIndex + 1] += (_alpha * -2 * s * signX) * -1;
	A[V1xIndex][V1xIndex + 1] += (_alpha * 2 * s * signX) * -1;
	//EquaV0y
	A[V0xIndex + 1][VdxIndex] += (_alpha * 2) * -1 * s * signX;
	A[V0xIndex + 1][V1xIndex] += (_alpha * -2) * -1 * s * signX;
	A[V0xIndex + 1][V0xIndex + 1] += (_alpha * -2 * s * signX) * -1 * s * signX;
	A[V0xIndex + 1][V1xIndex + 1] += (_alpha * 2 * s * signX) * -1 * s * signX;
	//EquaV1y
	A[V1xIndex + 1][VdxIndex] += (_alpha * 2) * s * signX;
	A[V1xIndex + 1][V1xIndex] += (_alpha * -2) * s * signX;
	A[V1xIndex + 1][V0xIndex + 1] += (_alpha * -2 * s * signX) * s * signX;
	A[V1xIndex + 1][V1xIndex + 1] += (_alpha * 2 * s * signX) * s * signX;

	//EquaVdy
	A[VdxIndex + 1][VdxIndex + 1] += (_alpha * 2);
	A[VdxIndex + 1][V1xIndex + 1] += (_alpha * -2);
	A[VdxIndex + 1][V0xIndex] += (_alpha * -2 * s * signY);
	A[VdxIndex + 1][V1xIndex] += (_alpha * 2 * s * signY);
	//EquaV1y
	A[V1xIndex + 1][VdxIndex + 1] += (_alpha * 2) * -1;
	A[V1xIndex + 1][V1xIndex + 1] += (_alpha * -2) * -1;
	A[V1xIndex + 1][V0xIndex] += (_alpha * -2 * s * signY) * -1;
	A[V1xIndex + 1][V1xIndex] += (_alpha * 2 * s * signY) * -1;
	//EquaV0x
	A[V0xIndex][VdxIndex + 1] += (_alpha * 2) * s * signY * -1;
	A[V0xIndex][V1xIndex + 1] += (_alpha * -2) * s * signY * -1;
	A[V0xIndex][V0xIndex] += (_alpha * -2 * s * signY) * s * signY * -1;
	A[V0xIndex][V1xIndex] += (_alpha * 2 * s * signY) * s * signY * -1;
	//EquaV1x
	A[V1xIndex][VdxIndex + 1] += (_alpha * 2) * s * signY;
	A[V1xIndex][V1xIndex + 1] += (_alpha * -2) * s * signY;
	A[V1xIndex][V0xIndex] += (_alpha * -2 * s * signY) * s * signY;
	A[V1xIndex][V1xIndex] += (_alpha * 2 * s * signY) * s * signY;
}

void VMotionModel::BuildModel_Es(double **A, double *B, int iCellRow, int iCellCol, int *lVertexxIndex)
{
	double devideHeightByWidth = pVideo->CellHeight / pVideo->CellWidth;
	double clockWise = 1.0;
	int **vertexxIndex = new int*[2];
	vertexxIndex[0] = &lVertexxIndex[0];
	vertexxIndex[1] = &lVertexxIndex[2];

	BuildModel_Es_ATriangle(A, vertexxIndex[0][1], vertexxIndex[0][0], vertexxIndex[1][0], 1 / devideHeightByWidth, clockWise);
	BuildModel_Es_ATriangle(A, vertexxIndex[0][0], vertexxIndex[1][0], vertexxIndex[1][1], devideHeightByWidth, clockWise);
	BuildModel_Es_ATriangle(A, vertexxIndex[1][0], vertexxIndex[1][1], vertexxIndex[0][1], 1 / devideHeightByWidth, clockWise);
	BuildModel_Es_ATriangle(A, vertexxIndex[1][1], vertexxIndex[0][1], vertexxIndex[0][0], devideHeightByWidth, clockWise);

	BuildModel_Es_ATriangle(A, vertexxIndex[1][0], vertexxIndex[0][0], vertexxIndex[0][1], devideHeightByWidth, -clockWise);
	BuildModel_Es_ATriangle(A, vertexxIndex[1][1], vertexxIndex[1][0], vertexxIndex[0][0], 1 / devideHeightByWidth, -clockWise);
	BuildModel_Es_ATriangle(A, vertexxIndex[0][1], vertexxIndex[1][1], vertexxIndex[1][0], devideHeightByWidth, -clockWise);
	BuildModel_Es_ATriangle(A, vertexxIndex[0][0], vertexxIndex[0][1], vertexxIndex[1][1], 1 / devideHeightByWidth, -clockWise);

	delete[]vertexxIndex;
}

void VMotionModel::BuildModel()
{
	int dimen = (CellRows + 1) * (CellCols + 1) * 2;//The dimention of the Equantions

	//Linear Equations parameters AX=B
	//means partial(E)/partial(xi) = partial(E)/partial(yi) = 0
	//(R)th row mean (R/2)th Point's x or y variable's partial(e)/partial(x or y) = 0
	double *lA = new double[dimen*dimen];
	double **A = new double*[dimen];
	double *B = new double[dimen];
	double *X = new double[dimen];
	BuildModel_InitializeArray(dimen, lA, A, B, X);//Open space and initialize A¡¢B to zero
	
	//-------------------------------Buil the Equations----------------------------------------
	//for Each Cell
	for (int iCellRow = 0; iCellRow < CellRows; iCellRow++)
	{
		for (int iCellCol = 0; iCellCol < CellCols; iCellCol++)
		{
			int vertexxIndex[2][2];//The Row/Col index of 4 vertex's x, the y's index is (x'index + 1)
			for (int iVR = 0; iVR < 2; iVR++)
			{
				for (int iVC = 0; iVC < 2; iVC++)
				{
					vertexxIndex[iVR][iVC] = BuildModel_GetxIndex(iCellRow + iVR, iCellCol + iVC);
				}
			}

			BuildModel_Ed(A, B, iCellRow, iCellCol, &vertexxIndex[0][0]);
			BuildModel_Es(A, B, iCellRow, iCellCol, &vertexxIndex[0][0]);
		}
	}

	//--------------------------------Solve the Equations---------------------------------------
	VLinearEquationsSolver *equtionSolver = new VLinearEquationsSolver(dimen, A, B, X);

	//-------------------------------get value of vertex----------------------------------------
	for (int iCellRow = 0; iCellRow < CellRows + 1; iCellRow++)
	{
		for (int iCellCol = 0; iCellCol < CellCols + 1; iCellCol++)
		{
			int indexx = BuildModel_GetxIndex(iCellRow, iCellCol);
			ControlPoints[iCellRow][iCellCol].x = X[indexx];
			ControlPoints[iCellRow][iCellCol].y = X[indexx + 1];
		}
	}

	delete[]lA;
	delete[]A;
	delete[]B;
	delete[]X;
}

void VMotionModel::CalculateError()
{
	ErrorE = CalculateError_eh() + _beta * CalculateError_es();
}

void VMotionModel::BuildTransModel()
{
	cv::Point2f **fromPoint = new cv::Point2f*[3];
	cv::Point2f **toPoint = new cv::Point2f*[3];
	for (int iCellRow = 0; iCellRow < CellRows; iCellRow++)
	{
		for (int iCellCol = 0; iCellCol < CellCols; iCellCol++)
		{
#ifdef TRIANGULAR_DEVIDE_POINTS
			fromPoint[0] = &StandardControlPoints[iCellRow][iCellCol];
			toPoint[0] = &ControlPoints[iCellRow][iCellCol];

			fromPoint[1] = &StandardControlPoints[iCellRow + 1][iCellCol + 1];
			toPoint[1] = &ControlPoints[iCellRow + 1][iCellCol + 1];

			fromPoint[2] = &StandardControlPoints[iCellRow + 1][iCellCol];
			toPoint[2] = &ControlPoints[iCellRow + 1][iCellCol];

			TransModel[iCellRow][iCellCol * 2] = new VHomographyModel(fromPoint, toPoint);

			fromPoint[2] = &StandardControlPoints[iCellRow][iCellCol + 1];
			toPoint[2] = &ControlPoints[iCellRow][iCellCol + 1];

			TransModel[iCellRow][iCellCol * 2 + 1] = new VHomographyModel(fromPoint, toPoint);
#else
#endif
		}
	}
	delete[]fromPoint;
	delete[]toPoint;
}

void VMotionModel::BuildInverseTransModel()
{
	invTransModel = new VProTransModel**[CellRows];
	for (int iCellRow = 0; iCellRow < CellRows; iCellRow++)
	{
		invTransModel[iCellRow] = new VProTransModel*[CellCols];
		for (int iCellCol = 0; iCellCol < CellCols; iCellCol++)
		{
			invTransModel[iCellRow][iCellCol * 2] = ((VHomographyModel*)TransModel[iCellRow][iCellCol * 2])->GetInverse();
			invTransModel[iCellRow][iCellCol * 2 + 1] = ((VHomographyModel*)TransModel[iCellRow][iCellCol * 2 + 1])->GetInverse();
		}
	}
}

double VMotionModel::CalculateError_eh()
{
#ifdef TRIANGULAR_DEVIDE_POINTS

	double eh = 0;

	for (int iCellRow = 0; iCellRow < CellRows; iCellRow++)
	{
		for (int iHalfCol = 0; iHalfCol < CellCols * 2; iHalfCol++)
		{
			std::vector<VMatchedPoint>::iterator itPoint = pVideo->Cells[FrameIndex][iCellRow][iHalfCol].MatchedPoints.begin();
			std::vector<VMatchedPoint>::iterator itEnd = itPoint + pVideo->Cells[FrameIndex][iCellRow][iHalfCol].GoodMatchedCount;

			for (; itPoint != itEnd; itPoint++)
			{
				cv::Point2f *toPoint = TransModel[iCellRow][iHalfCol]->Trans(&(itPoint->SrcPoint));
				double dis = (toPoint->x - itPoint->DstPoint.x) * (toPoint->x - itPoint->DstPoint.x)
					+ (toPoint->y - itPoint->DstPoint.y) * (toPoint->y - itPoint->DstPoint.y);
				eh += dis;
			}
		}
	}

	return eh;
#else
	cout << "TODO Not_TRIANGULAR_DEVIDE_POINTS..."
	return -1;
#endif
}

double VMotionModel::CalculateError_es()
{
#ifdef TRIANGULAR_DEVIDE_POINTS
	double es = 0;

	for (int iCellRow = 0; iCellRow < CellRows; iCellRow++)
	{
		for (int iHalfCol = 0; iHalfCol < CellCols * 2 - 1; iHalfCol++)
		{
			es += VProTransModel::GetEucDis(
				TransModel[iCellRow][iHalfCol], TransModel[iCellRow][iHalfCol + 1]);
		}
	}

	for (int iCellRow = 0; iCellRow < CellRows - 1; iCellRow++)
	{
		for (int iCellCol = 0; iCellCol < CellCols; iCellCol++)
		{
			es += VProTransModel::GetEucDis(
				TransModel[iCellRow][iCellCol * 2], TransModel[iCellRow + 1][iCellCol * 2 + 1]);
		}
	}

	return es;
#else
	cout << "TODO Not_TRIANGULAR_DEVIDE_POINTS..."
		return -1;
#endif
}

cv::Point2f* VMotionModel::GetInversePoint(int x, int y, int cellRow, int cellCol)
{
	return invTransModel[cellRow][cellCol]->Trans(new cv::Point2f(x, y));
}