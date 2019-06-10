#include "stdafx.h"
#include "VStabilizationHandle.h"

VStabilizationHandle::VStabilizationHandle(char *inputFileName)
{
	try
	{
		Video = new VVideo(inputFileName, _cellRows, _cellCols);

		//TestConsole();

		GetAllMatchedPointsGPU();
		GetGlobalGoodPointsWithMinDis();
		//GetGlobalGoodPointsWithRANSAC();
		//SelectMatchedPoints();

		BuildCells();
		BuildStandardControlPointsModel();
		Calculate2DInterpolationW();
		
		EstimateMotion();
		StabilizeWithRecModel();
		_TESTShow3();

		system("pause");
	}
	catch (const std::exception &e)
	{
		std::cout << "error:  " << e.what() << std::endl;
		system("pause");
	}
}

void VStabilizationHandle::TestConsole()
{
	cv::Point2f fp1(0, 0);
	cv::Point2f fp2(0, 1);
	cv::Point2f fp3(1, 0);
	cv::Point2f tp1(2, 0);
	cv::Point2f tp2(0, 0);
	cv::Point2f tp3(2, 2);

	VMatchedPoint mp1(fp1, tp1);
	VMatchedPoint mp2(fp2, tp2);
	VMatchedPoint mp3(fp3, tp3);

	VMatchedPoint **mp = new VMatchedPoint*[3];
	mp[0] = &mp1;
	mp[1] = &mp2;
	mp[2] = &mp3;
}


VStabilizationHandle::~VStabilizationHandle()
{
}

void VStabilizationHandle::_TESTShow3()
{
	cv::Size size(Video->Width, Video->Height);
	cv::VideoWriter outputWriter(
		"trans.avi",
		CV_FOURCC('D', 'I', 'V', 'X'),
		Video->Fps,//fps
		size,
		true
		);

	cv::Mat oldMat;
	Video->RefreshCapture();
	for (int iFrame = 0; iFrame < Video->FramesCount - 1; iFrame++)
	{
		Video->Capture >> oldMat;
		//cv::Mat newMat = Video->ColorPic[iFrame];
		//cv::Mat newMat = VResample::GetResampleMatByInvTrans(Video->ReviseModel[iFrame], (Video->ColorPic[iFrame]), _reservedRate);
		cv::Mat newMat = VResample::GetResampleMatByRecCellModel(Video->recCellModel, (oldMat), iFrame);
		if (0)
		{
			for (int iRow = 0; iRow < Video->CellRows + 1; iRow++)
			{
				for (int iCol = 0; iCol < Video->CellCols + 1; iCol++)
				{
					cv::Point2f fromPoint = StandardControlPoints[iRow][iCol];
					//cv::Point2f toPoint = NewControlPoints[iFrame][iRow][iCol];
					cv::Point2f toPoint2;
					toPoint2.x = (NewControlPoints[iFrame + 1][iRow][iCol].x - NewControlPoints[iFrame][iRow][iCol].x) * 10 + fromPoint.x;
					toPoint2.y = (NewControlPoints[iFrame + 1][iRow][iCol].y - NewControlPoints[iFrame][iRow][iCol].y) * 10 + fromPoint.y;
					//toPoint2.x = NewControlPoints[iFrame][iRow][iCol].x * 10 + fromPoint.x;
					//toPoint2.y = NewControlPoints[iFrame][iRow][iCol].y * 10 + fromPoint.y;
					cv::Point2f toPoint;
					toPoint.x = InvControlPoints[iFrame][iRow][iCol].x;
					toPoint.y = InvControlPoints[iFrame][iRow][iCol].y;
					arrowedLine(
						newMat,
						fromPoint,
						toPoint,
						cv::Scalar(0, 255, 0),
						2, 8);
					arrowedLine(
						newMat,
						fromPoint,
						toPoint2,
						cv::Scalar(0, 0, 255),
						2, 8);
				}
			}
		}
		if (0)
		{
			for (int iRow = 0; iRow < Video->CellRows; iRow++)
			{
				for (int iCol = 0; iCol < Video->CellCols; iCol++)
				{
					cv::Point2f fromPoint = *Video->recCellModel->recCenter[iRow][iCol];
					//cv::Point2f toPoint = NewControlPoints[iFrame][iRow][iCol];
					cv::Point2f toPoint;
					toPoint.x = (Video->recCellModel->BM[iFrame][iRow][iCol]->x) * 100 + fromPoint.x;
					toPoint.y = (Video->recCellModel->BM[iFrame][iRow][iCol]->y) * 100 + fromPoint.y;
					//toPoint2.x = NewControlPoints[iFrame][iRow][iCol].x * 10 + fromPoint.x;
					//toPoint2.y = NewControlPoints[iFrame][iRow][iCol].y * 10 + fromPoint.y;
					//v::Point2f toPoint;
					//toPoint.x = InvControlPoints[iFrame][iRow][iCol].x;
					//toPoint.y = InvControlPoints[iFrame][iRow][iCol].y;
					arrowedLine(
						newMat,
						fromPoint,
						toPoint,
						cv::Scalar(0, 255, 0),
						2, 8);
				}
			}
		}
		if (1)
		{
			cv::imshow("_TESTShow", newMat);
			cv::waitKey(1);
		}
		outputWriter.write(newMat);
		std::cout << ".";
		newMat.release();
	}
}

void VStabilizationHandle::_TESTShow()
{
	cv::Size size(Video->Width, Video->Height);
	cv::VideoWriter outputWriter(
		"trans.avi",
		CV_FOURCC('D', 'I', 'V', 'X'),
		10,//fps
		size,
		true
		);

	cv::Mat oldMat;
	Video->RefreshCapture();
	for (int iFrame = 0; iFrame < Video->FramesCount - 1; iFrame++)
	{
		Video->Capture >> oldMat;
		for (int iRow = 0; iRow < Video->CellRows + 1; iRow++)
		{
			for (int iCol = 0; iCol < Video->CellCols + 1; iCol++)
			{
				cv::Point2f fromPoint = StandardControlPoints[iRow][iCol];
				cv::Point2f toPoint = InvControlPoints[iFrame][iRow][iCol];
				arrowedLine(
					oldMat,
					fromPoint,
					toPoint,
					cv::Scalar(0, 255, 0),
					2, 8);
			}
			/*
			for (int iHCol = 0; iHCol < Video->CellCols * 2; iHCol++)//iHCol:Half Column Index
			{
				VTriCell &drawCell = Video->Cells[iFrame][iRow][iHCol];
				int iCol = iHCol / 2;
				for (vector<VMatchedPoint>::iterator it = drawCell.MatchedPoints.begin();
					it != drawCell.MatchedPoints.end(); it++)
				{
					double time = 10;
					cv::Point2f tOriDst;
					cv::Point2f tCalDst;
					tOriDst.x = (it->DstPoint.x - it->SrcPoint.x) * time + it->SrcPoint.x;
					tOriDst.y = (it->DstPoint.y - it->SrcPoint.y) * time + it->SrcPoint.y;

					cv::Point2f* calDst = Video->MotionModel[iFrame]->TransModel[iRow][iCol]->Trans(&(it->SrcPoint));
					tCalDst.x = (calDst->x - it->SrcPoint.x) * time + it->SrcPoint.x;
					tCalDst.y = (calDst->y - it->SrcPoint.y) * time + it->SrcPoint.y;

					arrowedLine(
						Video->ColorPic[iFrame],
						it->SrcPoint,
						tCalDst,
						cv::Scalar(0, 255, 0),
						2, 8);

					arrowedLine(
						Video->ColorPic[iFrame],
						it->SrcPoint,
						tOriDst,
						cv::Scalar(0, 0, 255),
						2, 8);
				}
			}*/
		}
		cv::imshow("_TESTShow", oldMat);
		cv::waitKey(500);
		outputWriter.write(oldMat);
	}
}

void VStabilizationHandle::_TESTShow2()
{
	cv::Size size(Video->Width, Video->Height);
	cv::VideoWriter outputWriter(
		"trans.avi",
		CV_FOURCC('D', 'I', 'V', 'X'),
		10,//fps
		size,
		true
		);
	cv::Mat oldMat;
	Video->RefreshCapture();
	for (int iFrame = 0; iFrame < Video->FramesCount - 1; iFrame++)
	{
		Video->Capture >> oldMat;
		for (int iRow = 0; iRow < Video->CellRows + 1; iRow++)
		{
			for (int iCol = 0; iCol < Video->CellCols + 1; iCol++)
			{
				cv::Point2f tCP;
				tCP.x = StandardControlPoints[iRow][iCol].x;
				tCP.y = StandardControlPoints[iRow][iCol].y;
				double times = 10;
				tCP.x += times * (Video->MotionModel[iFrame]->ControlPoints[iRow][iCol].x - StandardControlPoints[iRow][iCol].x);
				tCP.y += times * (Video->MotionModel[iFrame]->ControlPoints[iRow][iCol].y - StandardControlPoints[iRow][iCol].y);

				arrowedLine(
					oldMat,
					StandardControlPoints[iRow][iCol],
					tCP,
					cv::Scalar(0, 255, 0),
					2, 8);
			}
		}
		cv::imshow("_TESTShow", oldMat);
		cv::waitKey(500);
		outputWriter.write(oldMat);
	}
}

