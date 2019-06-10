#include "stdafx.h"
#include "VHomographyModel.h"

VHomographyModel::VHomographyModel()
{
	ModelType = ModelTypes::Homography;
	InitializePars();
}

VHomographyModel::VHomographyModel(double *parPar)
{
	ModelType = ModelTypes::Homography;
	InitializePars();
	memcpy(Par, parPar, sizeof(double) * 6);
}

VHomographyModel::VHomographyModel(cv::Point2f **fromPoint, cv::Point2f **toPoint)
{
	ModelType = ModelTypes::Homography;
	InitializePars();
	BuildHomoTrans(fromPoint, toPoint);
}

void VHomographyModel::BuildHomoTrans(cv::Point2f **fromPoint, cv::Point2f **toPoint)
{
	VLinearEquationsSolver *psolver;
	double *lA = new double[9];
	double **A = new double*[3];
	double *B = new double[3];
	double *X = new double[3];

	//BuildA
	for (int iPoint = 0; iPoint < 3; iPoint++)
	{
		A[iPoint] = &lA[3 * iPoint];
		A[iPoint][0] = fromPoint[iPoint]->x;
		A[iPoint][1] = fromPoint[iPoint]->y;
		A[iPoint][2] = 1;
	}

	//X
	for (int iPoint = 0; iPoint < 3; iPoint++)
	{
		B[iPoint] = toPoint[iPoint]->x;
	}
	psolver = new VLinearEquationsSolver(3, A, B, X);
	memcpy(&H[0][0], X, sizeof(double) * 3);
	delete psolver;

	//Y
	for (int iPoint = 0; iPoint < 3; iPoint++)
	{
		B[iPoint] = toPoint[iPoint]->y;
	}
	psolver = new VLinearEquationsSolver(3, A, B, X);
	memcpy(&H[1][0], X, sizeof(double) * 3);
	delete psolver;

	delete[]lA;
	delete[]A;
	delete[]B;
	delete[]X;
}

void VHomographyModel::BuildHomoTrans(VMatchedPoint **matchedPoints)
{
	double *lA = new double[6 * 6];
	double **A = new double*[6];
	double *X = Par;
	double *B = new double[6];
	for (int iPoint = 0; iPoint < 3; iPoint++)
	{
		int row = iPoint * 2;
		A[row] = &lA[row * 6];
		B[row] = matchedPoints[iPoint]->DstPoint.x;
		A[row][0] = matchedPoints[iPoint]->SrcPoint.x;
		A[row][1] = matchedPoints[iPoint]->SrcPoint.y;
		A[row][2] = 1;
		A[row][3] = A[row][4] = A[row][5] = 0;

		row++;
		A[row] = &lA[row * 6];
		B[row] = matchedPoints[iPoint]->DstPoint.y;
		A[row][0] = A[row][1] = A[row][2] = 0;
		A[row][3] = matchedPoints[iPoint]->SrcPoint.x;
		A[row][4] = matchedPoints[iPoint]->SrcPoint.y;
		A[row][5] = 1;
	}

	VLinearEquationsSolver solver(6, A, B, X);

	delete[]lA;
	delete[]A;
	delete[]B;
}

void VHomographyModel::InitializePars()
{
	ParCount = 6;
	Par = new double[6];
	H[0] = &Par[0];
	H[1] = &Par[3];
}

VHomographyModel::~VHomographyModel()
{
	delete[]Par;
}

cv::Point2f* VHomographyModel::HomographyTrans(cv::Point2f *fromPoint)
{
	cv::Point2f *pToPoint = new cv::Point2f(
		H[0][0] * fromPoint->x + H[0][1] * fromPoint->y + H[0][2],
		H[1][0] * fromPoint->x + H[1][1] * fromPoint->y + H[1][2]
		);
	return pToPoint;
}

VHomographyModel* VHomographyModel::GetInverse()
{
	double *invPar = new double[6];
	double **invH = new double*[2];
	invH[0] = &invPar[0];
	invH[1] = &invPar[3];
	double div = H[1][1] * H[0][0] - H[0][1] * H[1][0];
	invH[0][0] = H[1][1] / div;
	invH[0][1] = -H[0][1] / div;
	invH[0][2] = (H[0][1] * H[1][2] - H[0][2] * H[1][1]) / div;
	invH[1][0] = -H[1][0] / div;
	invH[1][1] = H[0][0] / div;
	invH[1][2] = (H[0][2] * H[1][0] - H[1][2] * H[0][0]) / div;
	return new VHomographyModel(invPar);
}