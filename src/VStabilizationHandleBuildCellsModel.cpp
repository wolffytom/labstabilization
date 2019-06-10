#include "stdafx.h"
#include "VStabilizationHandle.h"

void VStabilizationHandle::BuildCells()
{
	try
	{
		for (int iFrame = 0; iFrame < Video->FramesCount - 1; iFrame++)
		{
			//first iterator
			std::vector<VMatchedPoint>::iterator itBegin = Video->AllMatchedPoints[iFrame].begin();
			//loop variable
			std::vector<VMatchedPoint>::iterator itAt;

			itAt = itBegin;
			for (int iPoint = 0; iPoint < Video->GoodMatchedPointsCount[iFrame]; iPoint++, itAt++)
			{
				VMatchedPoint &p = *itAt;
				int colIndex = p.SrcPoint.x / Video->CellWidth;
				int rowIndex = p.SrcPoint.y / Video->CellHeight;
#ifdef TRIANGULAR_DEVIDE_POINTS
				
				float cellx = p.SrcPoint.x - colIndex * Video->CellWidth;
				float celly = p.SrcPoint.y - rowIndex * Video->CellHeight;
				
				int inRightPart = 0;
				//y/x < height/width
				if (celly * Video->CellWidth < cellx * Video->CellHeight)
				{
					inRightPart = 1;
				}
				VTriCell &inCell = Video->Cells[iFrame][rowIndex][colIndex * 2 + inRightPart];
#else
				VTriCell &inCell = Video->Cells[iFrame][rowIndex][colIndex];
#endif

				inCell.MatchedPoints.push_back(p);
				inCell.GoodMatchedCount++;
			}

			cout << ".";
		}
		cout << "done" << endl;
	}
	catch (const std::exception &e)
	{
		std::cout << "BuildCells Failed---error:  " << e.what() << std::endl;
		system("pause");
	}
}

void VStabilizationHandle::BuildStandardControlPointsModel()
{
	StandardControlPoints = new cv::Point2f*[Video->CellRows + 1];
	for (int iRow = 0; iRow < Video->CellRows + 1; iRow++)
	{
		StandardControlPoints[iRow] = new cv::Point2f[Video->CellCols + 1];

		for (int iCol = 0; iCol < Video->CellCols + 1; iCol++)
		{
			StandardControlPoints[iRow][iCol].x = iCol * Video->CellWidth;
			StandardControlPoints[iRow][iCol].y = iRow * Video->CellHeight;
		}
	}

}

void VStabilizationHandle::Calculate2DInterpolationWPerCell(VTriCell &refCell, int rowIndex, int colIndex)
{
	for (vector<VMatchedPoint>::iterator it = refCell.MatchedPoints.begin();
		it != refCell.MatchedPoints.end(); it++)
	{
		float wRight = (it->SrcPoint.x - StandardControlPoints[rowIndex][colIndex].x) / Video->CellWidth;
		float wLeft = 1 - wRight;
		float wBottom = (it->SrcPoint.y - StandardControlPoints[rowIndex][colIndex].y) / Video->CellHeight;
		float wTop = 1 - wBottom;

		it->w[0][0] = wLeft * wTop;
		it->w[0][1] = wRight * wTop;
		it->w[1][0] = wLeft * wBottom;
		it->w[1][1] = wRight * wBottom;
	}
}

void VStabilizationHandle::Calculate2DInterpolationW()
{
	try
	{
		for (int iFrame = 0; iFrame < Video->FramesCount - 1; iFrame++)
		{
			for (int iRow = 0; iRow < Video->CellRows; iRow++)
			{
#ifdef TRIANGULAR_DEVIDE_POINTS
				for (int iHCol = 0; iHCol < Video->CellCols * 2; iHCol++)//iHCol:Half Column Index
				{
					int iCol = iHCol / 2;
					VTriCell &refCell = Video->Cells[iFrame][iRow][iHCol];
					Calculate2DInterpolationWPerCell(refCell, iRow, iCol);
				}
#else
				for (int iCol = 0; iCol < Video->CellCols; iCol++)
				{
					VTriCell &refCell = Video->Cells[iFrame][iRow][iCol];
					Calculate2DInterpolationWPerCell(refCell, iRow, iCol);
				}
#endif
			}
		}
		

		cout << "Calculate2DInterpolationW done" << endl;
	}
	catch (const std::exception &e)
	{
		std::cout << "Calculate2DInterpolationW Failed---error:  " << e.what() << std::endl;
		system("pause");
	}
}

void VStabilizationHandle::EstimateMotion()
{
	cout << "EstimateMotion begin:";
	//for each frame
	for (int iFrame = 0; iFrame < Video->FramesCount - 1; iFrame++)
	{
		VMotionModel **pMotionModelArray = new VMotionModel*[11];
		VMotionModel *pBestMotionModel;

		int besti = 0;
		double _alpha = 0.05;
		pBestMotionModel = new VMotionModel(iFrame, _alpha, Video, StandardControlPoints);
		/*
		for (int ia = 0; ia < 11; _alpha += 0.01, ia++)
		{
			pMotionModelArray[ia] = new VMotionModel(iFrame, _alpha, Video, StandardControlPoints);
			if (pMotionModelArray[ia]->ErrorE < pMotionModelArray[besti]->ErrorE)
				besti = ia;
		}
		pBestMotionModel = pMotionModelArray[besti];
		for (int ia = 0; ia < 11; ia++)
		{
			if (ia != besti)
				delete(pMotionModelArray[ia]);
		}
		delete[]pMotionModelArray;*/

		Video->MotionModel[iFrame] = pBestMotionModel;
		cout << pBestMotionModel->_alpha;

		cout << ".";
	}
	cout << endl << "EstimateMotion done!" << endl;
}