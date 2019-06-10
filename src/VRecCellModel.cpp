#include "stdafx.h"
#include "VRecCellModel.h"
#include <math.h>
#include <fstream>


VRecCellModel::VRecCellModel(VVideo *parPVideo)
{
	pVideo = parPVideo;
	pVideo->recCellModel = this;
	CellCols = pVideo->CellCols;
	CellRows = pVideo->CellRows;
	F = new V2DVector***[pVideo->FramesCount];
	C = new V2DVector***[pVideo->FramesCount];
	P = new V2DVector***[pVideo->FramesCount];
	Plast = new V2DVector***[pVideo->FramesCount];
	B = new V2DVector***[pVideo->FramesCount];
	BM = new V2DVector***[pVideo->FramesCount];
	for (int iFrame = 0; iFrame < pVideo->FramesCount; iFrame++)
	{
		F[iFrame] = new V2DVector**[CellRows];
		C[iFrame] = new V2DVector**[CellRows];
		P[iFrame] = new V2DVector**[CellRows];
		Plast[iFrame] = new V2DVector**[CellRows];
		B[iFrame] = new V2DVector**[CellRows];
		BM[iFrame] = new V2DVector**[CellRows];
		for (int iRow = 0; iRow < CellRows; iRow++)
		{
			F[iFrame][iRow] = new V2DVector*[CellCols];
			C[iFrame][iRow] = new V2DVector*[CellCols];
			P[iFrame][iRow] = new V2DVector*[CellCols];
			Plast[iFrame][iRow] = new V2DVector*[CellCols];
			B[iFrame][iRow] = new V2DVector*[CellCols];
			BM[iFrame][iRow] = new V2DVector*[CellCols];
			for (int iCol = 0; iCol < CellCols; iCol++)
			{
				F[iFrame][iRow][iCol] = new V2DVector(0, 0);
				C[iFrame][iRow][iCol] = new V2DVector(0, 0);
				P[iFrame][iRow][iCol] = new V2DVector(0, 0);
				Plast[iFrame][iRow][iCol] = new V2DVector(0, 0);
				B[iFrame][iRow][iCol] = new V2DVector(0, 0);
				BM[iFrame][iRow][iCol] = new V2DVector(0, 0);
			}
		}
	}
	recCenter = new cv::Point2f**[CellRows];
	for (int iRow = 0; iRow < CellRows; iRow++)
	{
		recCenter[iRow] = new cv::Point2f*[CellCols];
		for (int iCol = 0; iCol < CellCols; iCol++)
		{
			recCenter[iRow][iCol] = new cv::Point2f();
		}
	}

	InitializeRecCellModel();

	Iterate();

	CalculateB();

	OutputPToFile(3, 3, "", 1);
}

void VRecCellModel::InitializeRecCellModel()
{
	for (int iRow = 0; iRow < CellRows; iRow++)
	{
		for (int iCol = 0; iCol < CellCols; iCol++)
		{
			recCenter[iRow][iCol]->x = (0.5 + iCol) * pVideo->CellWidth;
			recCenter[iRow][iCol]->y = (0.5 + iRow) * pVideo->CellHeight;
		}
	}
	for (int iFrame = 0; iFrame < pVideo->FramesCount - 1; iFrame++)
	{
		for (int iRow = 0; iRow < CellRows; iRow++)
		{
			for (int iCol = 0; iCol < CellCols; iCol++)
			{
				cv::Point2f *transPoint1, *transPoint2;
				transPoint1 = pVideo->MotionModel[iFrame]->TransModel[iRow][iCol * 2]->Trans(recCenter[iRow][iCol]);
				transPoint2 = pVideo->MotionModel[iFrame]->TransModel[iRow][iCol * 2 + 1]->Trans(recCenter[iRow][iCol]);
				F[iFrame][iRow][iCol]->x = (transPoint1->x + transPoint2->x) / 2 - recCenter[iRow][iCol]->x;
				F[iFrame][iRow][iCol]->y = (transPoint1->y + transPoint2->y) / 2 - recCenter[iRow][iCol]->y;

				P[iFrame + 1][iRow][iCol]->x = C[iFrame + 1][iRow][iCol]->x
					= C[iFrame][iRow][iCol]->x + F[iFrame][iRow][iCol]->x;
				P[iFrame + 1][iRow][iCol]->y = C[iFrame + 1][iRow][iCol]->y
					= C[iFrame][iRow][iCol]->y + F[iFrame][iRow][iCol]->y;
			}
		}
	}
}

double VRecCellModel::GaussFunc(double stdv, double x)
{
	return 1 / sqrt(2 * 3.1416) / stdv * exp(-x * x / 2 / stdv / stdv);
}

void VRecCellModel::Iterate()
{
	for (int iIt = 0; iIt < IteratorTimes; iIt++)
	{
		cout << iIt << " th iterate\n";
		//Initialize
		for (int iFrame = 0; iFrame < pVideo->FramesCount; iFrame++)
		{
			for (int iRow = 0; iRow < CellRows; iRow++)
			{
				for (int iCol = 0; iCol < CellCols; iCol++)
				{
					Plast[iFrame][iRow][iCol]->x = P[iFrame][iRow][iCol]->x;
					Plast[iFrame][iRow][iCol]->y = P[iFrame][iRow][iCol]->y;
				}
			}
		}

		//Calculate
		for (int iFrame = 0; iFrame < pVideo->FramesCount; iFrame++)
		{
			for (int iRow = 0; iRow < CellRows; iRow++)
			{
				for (int iCol = 0; iCol < CellCols; iCol++)
				{
					double numeratorX = 1 * C[iFrame][iRow][iCol]->x;
					double numeratorY = 1 * C[iFrame][iRow][iCol]->y;
					double denominator = 1;

					double lamda = 5;
					int o = 60;
					int rStart = iFrame - o / 2;
					int rEnd = iFrame + o / 2;
					if (rStart < 0) rStart = 0;
					if (rEnd > pVideo->FramesCount) rEnd = pVideo->FramesCount;
					for (int r = rStart; r < rEnd; r++)
					{
						double normt = abs(r - iFrame);
						double normm = abs(C[iFrame][iRow][iCol]->x - C[r][iRow][iCol]->x) + abs(C[iFrame][iRow][iCol]->y - C[r][iRow][iCol]->y);
						double wtr = GaussFunc(15, normt) * 2000 * GaussFunc(150, normm);
						if (0 && iRow == 0 && iCol == 0)
						{
							cout << GaussFunc(20, normt);
							system("pause");
						}

						denominator += 2 * wtr * lamda;
						numeratorX += 2 * wtr * lamda * Plast[r][iRow][iCol]->x;
						numeratorY += 2 * wtr * lamda * Plast[r][iRow][iCol]->y;
					}

					double domainWeight = 0;
					for (int nRow = iRow - 1; nRow <= iRow + 1; nRow++)
						for (int nCol = iCol - 1; nCol <= iCol + 1; nCol++)
						{
							if (nRow < 0 || nRow >= pVideo->CellRows
								|| nCol < 0 || nCol >= pVideo->CellCols)
								continue;
							if (nRow == iRow && nCol == iCol)
								continue;

							denominator += domainWeight;
							numeratorX += domainWeight * Plast[iFrame][nRow][nCol]->x;// -0.5 * C[iFrame][nRow][nCol]->x + 0.5 * C[iFrame][iRow][iCol]->x;
							numeratorY += domainWeight * Plast[iFrame][nRow][nCol]->y;// -0.5 * C[iFrame][nRow][nCol]->y + 0.5 * C[iFrame][iRow][iCol]->y;
						}

					P[iFrame][iRow][iCol]->x = numeratorX / denominator;
					P[iFrame][iRow][iCol]->y = numeratorY / denominator;
				}
			}
		}
	}
}

void VRecCellModel::CalculateB()
{
	for (int iFrame = 0; iFrame < pVideo->FramesCount; iFrame++)
	{
		for (int iRow = 0; iRow < CellRows; iRow++)
		{
			for (int iCol = 0; iCol < CellCols; iCol++)
			{
				B[iFrame][iRow][iCol]->x = P[iFrame][iRow][iCol]->x - C[iFrame][iRow][iCol]->x;
				B[iFrame][iRow][iCol]->y = P[iFrame][iRow][iCol]->y - C[iFrame][iRow][iCol]->y;
				BM[iFrame][iRow][iCol]->x = -B[iFrame][iRow][iCol]->x;
				BM[iFrame][iRow][iCol]->y = -B[iFrame][iRow][iCol]->y;
			}
		}
	}
}

//#define NO_EDGE
cv::Point2f* VRecCellModel::GetSrcPosition(int iFrame, int x, int y)
{
	if (iFrame == 3)
	{
		int a = 0;
	}

	float srcX, srcY, dx, dy;

	if (x <= recCenter[0][0]->x)
	{
#ifndef NO_EDGE
		if (y <= recCenter[0][0]->y)
		{
			dx = BM[iFrame][0][0]->x;
			dy = BM[iFrame][0][0]->y;
		}
		else if (y >= recCenter[CellRows - 1][0]->y)
		{
			dx = BM[iFrame][CellRows - 1][0]->x;
			dy = BM[iFrame][CellRows - 1][0]->y;
		}
		else//Left
		{
			int UpRow = (y - recCenter[0][0]->y) / pVideo->CellHeight;
			int DownRow = UpRow + 1;
			int DownDis = recCenter[DownRow][0]->y - y;
			float UpWeight = ((float)DownDis) / pVideo->CellHeight;
			float DownWeight = 1 - UpWeight;
			dx = BM[iFrame][UpRow][0]->x * UpWeight + BM[iFrame][DownRow][0]->x * DownWeight;
			dy = BM[iFrame][UpRow][0]->y * UpWeight + BM[iFrame][DownRow][0]->y * DownWeight;
		}
#else
		return new cv::Point2f(-1, -1);

#endif
	}
	else if (x >= recCenter[0][CellCols - 1]->x)
	{
#ifndef NO_EDGE
		if (y <= recCenter[0][0]->y)
		{
			dx = BM[iFrame][0][CellCols - 1]->x;
			dy = BM[iFrame][0][CellCols - 1]->y;
		}
		else if (y >= recCenter[CellRows - 1][0]->y)
		{
			dx = BM[iFrame][CellRows - 1][CellCols - 1]->x;
			dy = BM[iFrame][CellRows - 1][CellCols - 1]->y;
		}
		else//Right
		{
			int UpRow = (y - recCenter[0][0]->y) / pVideo->CellHeight;
			int DownRow = UpRow + 1;
			int DownDis = recCenter[DownRow][0]->y - y;
			float UpWeight = ((float)DownDis) / pVideo->CellHeight;
			float DownWeight = 1 - UpWeight;
			dx = BM[iFrame][UpRow][CellCols - 1]->x * UpWeight + BM[iFrame][DownRow][CellCols - 1]->x * DownWeight;
			dy = BM[iFrame][UpRow][CellCols - 1]->y * UpWeight + BM[iFrame][DownRow][CellCols - 1]->y * DownWeight;
		}
#else
		return new cv::Point2f(-1, -1);
#endif
	}
	else
	{
		if (y <= recCenter[0][0]->y)//Up
		{
#ifndef NO_EDGE
			int LeftCol = (x - recCenter[0][0]->x) / pVideo->CellWidth;
			int RightCol = LeftCol + 1;
			int RightDis = recCenter[0][RightCol]->x - x;
			float LeftWeight = ((float)RightDis) / pVideo->CellWidth;
			float RightWeight = 1 - LeftWeight;
			dx = BM[iFrame][0][LeftCol]->x * LeftWeight + BM[iFrame][0][RightCol]->x * RightWeight;
			dy = BM[iFrame][0][LeftCol]->y * LeftWeight + BM[iFrame][0][RightCol]->y * RightWeight;
#else
			return new cv::Point2f(-1, -1);
#endif
		}
		else if (y >= recCenter[CellRows - 1][0]->y)//Down
		{
#ifndef NO_EDGE
			int LeftCol = (x - recCenter[0][0]->x) / pVideo->CellWidth;
			int RightCol = LeftCol + 1;
			int RightDis = recCenter[0][RightCol]->x - x;
			float LeftWeight = ((float)RightDis) / pVideo->CellWidth;
			float RightWeight = 1 - LeftWeight;
			dx = BM[iFrame][CellRows - 1][LeftCol]->x * LeftWeight + BM[iFrame][CellRows - 1][RightCol]->x * RightWeight;
			dy = BM[iFrame][CellRows - 1][LeftCol]->y * LeftWeight + BM[iFrame][CellRows - 1][RightCol]->y * RightWeight;
#else
			return new cv::Point2f(-1, -1);
#endif
		}
		else
		{
			int LeftCol = (x - recCenter[0][0]->x) / pVideo->CellWidth;
			int RightCol = LeftCol + 1;
			int RightDis = recCenter[0][RightCol]->x - x;
			float LeftWeight = ((float)RightDis) / pVideo->CellWidth;
			float RightWeight = 1 - LeftWeight;
			int UpRow = (y - recCenter[0][0]->y) / pVideo->CellHeight;
			int DownRow = UpRow + 1;
			int DownDis = recCenter[DownRow][0]->y - y;
			float UpWeight = ((float)DownDis) / pVideo->CellHeight;
			float DownWeight = 1 - UpWeight;

			dx = BM[iFrame][UpRow][LeftCol]->x * LeftWeight * UpWeight
				+ BM[iFrame][DownRow][LeftCol]->x * LeftWeight * DownWeight
				+ BM[iFrame][DownRow][RightCol]->x * RightWeight * DownWeight
				+ BM[iFrame][UpRow][RightCol]->x * RightWeight * UpWeight;
			dy = BM[iFrame][UpRow][LeftCol]->y * LeftWeight * UpWeight
				+ BM[iFrame][DownRow][LeftCol]->y * LeftWeight * DownWeight
				+ BM[iFrame][DownRow][RightCol]->y * RightWeight * DownWeight
				+ BM[iFrame][UpRow][RightCol]->y * RightWeight * UpWeight;
		}
	}

	srcX = x + dx;
	srcY = y + dy;
	return new cv::Point2f(srcX, srcY);
}

VRecCellModel::~VRecCellModel()
{
}

void VRecCellModel::OutputPToFile(int pRow, int pCol, string filename, bool isX)
{
	stringstream filenamess;
	if (isX)
		filenamess << pRow << "," << pCol << "," << "x" << ".csv";
	else
		filenamess << pRow << "," << pCol << "," << "y" << ".csv";

	std::ofstream fOut(filenamess.str());
	if (isX)
		fOut << "原始x" << "," << "稳定后x" << endl;
	else
		fOut << "原始y" << "," << "稳定后y" << endl;
	for (int iFrame = 0; iFrame < pVideo->FramesCount; iFrame++)
	{
		if (isX)
			fOut << C[iFrame][pRow][pCol]->x << "," << P[iFrame][pRow][pCol]->x << endl;
		else
			fOut << C[iFrame][pRow][pCol]->y << "," << P[iFrame][pRow][pCol]->y << endl;

	}
	fOut.close();
}