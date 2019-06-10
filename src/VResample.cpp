#include "stdafx.h"
#include "VResample.h"

int VResample::GetPixel(int b, cv::Mat &From, int x, int y)
{
	uchar *data = From.ptr<uchar>(y);
	return *(data + 3 * x + b);
}

void VResample::ThreeOrderInter(uchar *data, cv::Mat &From, double nx, double ny, int width, int height)
{
	int x, y;
	x = nx;
	y = ny;
	double dx = nx - x;
	double dy = ny - y;
	if (x >= 0 && x + 1 < width && y >= 0 && y + 1 < height)
		for (int i = 0; i < 3; i++)
		{
			*(data + i) =
				(1 - dx) * (1 - dy) * GetPixel(i, From, x, y) +
				(dx)* (1 - dy) * GetPixel(i, From, x + 1, y) +
				(1 - dx) * (dy)* GetPixel(i, From, x, y + 1) +
				(dx)* (dy)* GetPixel(i, From, x + 1, y + 1)
				;
		}
}

void VResample::BilinearInter(uchar *data, cv::Mat &From, double nx, double ny, int width, int height)
{
	int x, y;
	x = nx;
	y = ny;
	double dx = nx - x;
	double dy = ny - y;
	if (x >= 0 && x + 1 < width && y >= 0 && y + 1 < height)
		for (int i = 0; i < 3; i++)
		{
			*(data + i) =
				(1 - dx) * (1 - dy) * GetPixel(i, From, x, y) +
				(dx)* (1 - dy) * GetPixel(i, From, x + 1, y) +
				(1 - dx) * (dy)* GetPixel(i, From, x, y + 1) +
				(dx)* (dy)* GetPixel(i, From, x + 1, y + 1)
				;
		}
}


cv::Mat VResample::GetResampleMatByRecCellModel(VRecCellModel *recCellModel, cv::Mat &oriMat, int frameIndex)
{
	int height = recCellModel->pVideo->Height;
	int width = recCellModel->pVideo->Width;

	cv::Mat *re = new cv::Mat(height, width, CV_8UC3, cv::Scalar(0, 0, 0));

	for (int iR = 0; iR < height; iR++)
	{
		uchar *data = re->ptr<uchar>(iR);
		for (int iC = 0; iC < width; iC++)
		{
			cv::Point2f *from = recCellModel->GetSrcPosition(frameIndex, iC, iR);

			BilinearInter(data + iC * 3, oriMat, from->x, from->y, width, height);
			delete(from);
		}
	}
	return *re;
}

cv::Mat VResample::GetResampleMatByInvTrans(VMotionModel *reviseModel, cv::Mat &oriMat, double reservedRate)
{
	int height = reviseModel->pVideo->Height;
	int width = reviseModel->pVideo->Width;

	int CellRows = reviseModel->pVideo->CellRows;
	int CellCols = reviseModel->pVideo->CellCols;

	double CellHeight = reviseModel->pVideo->CellHeight;
	double CellWidth = reviseModel->pVideo->CellWidth;

	cv::Mat *re = new cv::Mat(height, width, CV_8UC3, cv::Scalar(0, 0, 0));

	double startrate = 0;
	double endrate = 1;
	if (false)
	{
		startrate = (1 - reservedRate) / 2;
		endrate = 1 - startrate;
	}
	for (int iR = height * startrate; iR < height * endrate; iR++)
	{
		uchar *data = re->ptr<uchar>(iR);
		for (int iC = width * startrate; iC < width * endrate; iC++)
		{
			int cellR = iR / CellHeight;
			int cellC = iC / CellWidth;
			int cellX = iC - cellC * CellWidth;
			int cellY = iR - cellR * CellHeight;
			int cellHC;
			if (cellY * CellWidth >= cellX * CellHeight)
				cellHC = cellC * 2;
			else
				cellHC = cellC * 2 + 1;

			double **H = ((VHomographyModel*)(reviseModel->TransModel[cellR][cellHC]))->H;
			//cv::Point2f *fromPoint = reviseModel->TransModel[cellR][cellHC]->Trans(inPoint);
			double fromX = H[0][0] * iC + H[0][1] * iR + H[0][2];
			double fromY = H[1][0] * iC + H[1][1] * iR + H[1][2];

			BilinearInter(data + iC * 3, oriMat, fromX, fromY, width, height);

			//delete fromPoint;
		}
	}
	return *re;
}

cv::Mat VResample::GetResampleMat(VMotionModel *reviseModel, cv::Mat &oriMat)
{
	int height = reviseModel->pVideo->Height;
	int width = reviseModel->pVideo->Width;

	int CellRows = reviseModel->pVideo->CellRows;
	int CellCols = reviseModel->pVideo->CellCols;

	double CellHeight = reviseModel->pVideo->CellHeight;
	double CellWidth = reviseModel->pVideo->CellWidth;

	cv::Mat *re = new cv::Mat(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
	for (int iR = height/2; iR < height; iR++)
	{
		uchar *data = re->ptr<uchar>(iR);
		for (int iC = width/2; iC < width; iC++)
		{
			bool found = false;
			cv::Point2f *pSourse = NULL;
			for (int iCellR = 0; iCellR < CellRows; iCellR++)
				for (int iHCellC = 0; iHCellC < CellCols * 2; iHCellC++)
				{
					int iCellC = iHCellC / 2;
					cv::Point2f *tp = reviseModel->GetInversePoint(iC, iR, iCellR, iHCellC);
					double cellx = tp->x - iCellC * CellWidth;
					double celly = tp->y - iCellR * CellHeight;
					if (cellx >= 0 && cellx <= CellWidth
						&& celly >= 0 && celly <= CellHeight)
					{
						if (iHCellC % 2 == 0)
						{
							if (celly * CellWidth >= cellx * CellHeight)
							{
								found = true;
								pSourse = tp;
							}
						}
						else
						{
							if (celly * CellWidth <= cellx * CellHeight)
							{
								found = true;
								pSourse = tp;
							}
						}
					}
					delete tp;
				}

			if (found == true)
			{
				BilinearInter(data + iC * 3, oriMat, pSourse->x, pSourse->y ,width, height);
				delete pSourse;
			}
			cout << iC << ".";
		}
	}
	return *re;
}

VResample::VResample()
{
}


VResample::~VResample()
{
}
