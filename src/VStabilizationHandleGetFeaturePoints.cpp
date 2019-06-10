#include "stdafx.h"
#include "VStabilizationHandle.h"

#include <opencv2/core/cuda.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/cuda.hpp>
#include <opencv2/cudafeatures2d.hpp>
//#include <opencv2/core/cuda.hpp>

bool CmpMatchedPointsWithError(const VMatchedPoint &mp1, const VMatchedPoint &mp2)
{
	if (mp1.Error < mp2.Error)
		return true;
	return false;
}

bool CmpMatchedPointsWithDis(const VMatchedPoint &mp1, const VMatchedPoint &mp2)
{
	if (mp1.Dis < mp2.Dis)
		return true;
	return false;
}

void VStabilizationHandle::GetAllMatchedPointsNormal()
{
	try
	{
		std::cout << "GetAllMatchedPointsBegin:" << endl;
		cv::Ptr<cv::Feature2D> surf = cv::xfeatures2d::SURF::create(_surfD);

		//----------------------the former frame variables------------------
		cv::Mat thisFrameColor;
		cv::Mat thisFrameDescriptor;
		std::vector<cv::KeyPoint> thisFrameKeypoints;

		//------------------------get first frame----------------------
		Video->Capture >> thisFrameColor;//read first frame

		surf->detectAndCompute(thisFrameColor, cv::Mat(), thisFrameKeypoints, thisFrameDescriptor);
		cv::FlannBasedMatcher matcher;

		//------------------------for every frame--------------------------
		for (int iFrame = 0; iFrame < Video->FramesCount - 1; iFrame++)
		{

			//---------------------the later frame variables------------------
			cv::Mat nextFrameColor;
			cv::Mat nextFrameDescriptor;
			std::vector<cv::KeyPoint> nextFrameKeypoints;

			//---------------------get next frame-----------------------------
			Video->Capture >> nextFrameColor;//read next frame
			surf->detectAndCompute(nextFrameColor, cv::Mat(), nextFrameKeypoints, nextFrameDescriptor);

			//------------------------Match-----------------------------------
			std::vector< cv::DMatch > matches;
			matcher.match(thisFrameDescriptor, nextFrameDescriptor, matches);

			for (std::vector< cv::DMatch >::iterator it = matches.begin(); it != matches.end(); it++)
			{
				cv::Point2f &thisFramePoint = thisFrameKeypoints[it->queryIdx].pt;
				cv::Point2f &nextFramePoint = nextFrameKeypoints[it->trainIdx].pt;
				Video->AllMatchedPoints[iFrame].push_back(*(new VMatchedPoint(thisFramePoint, nextFramePoint)));
			}

			Video->GoodMatchedPointsCount[iFrame] =
				Video->AllMatchedPointsCount[iFrame] =
				Video->AllMatchedPoints[iFrame].end() - Video->AllMatchedPoints[iFrame].begin();
			

			thisFrameColor = nextFrameColor;
			thisFrameDescriptor = nextFrameDescriptor;
			thisFrameKeypoints.clear();
			thisFrameKeypoints = nextFrameKeypoints;

			std::cout << ".";
		}
		std::cout << endl << "done" << endl;
	}
	catch (const std::exception &e)
	{
		std::cout << "GPU Motion Estimate Failed---error:  " << e.what() << std::endl;
		system("pause");
	}
}

void VStabilizationHandle::GetAllMatchedPointsGPU()
{
	try
	{
		std::cout << "GetAllMatchedPointsBegin:" << endl;
		cv::cuda::SURF_CUDA cudaSurf(_surfD,
			4,
			3,
			false,
			0.01f,
			false);

		//----------------------the former frame variables------------------
		cv::Mat thisFrameColor, thisFrameGray;
		cv::cuda::GpuMat thisFrameGPU;
		cv::cuda::GpuMat thisFrameDescriptor;
		std::vector<cv::KeyPoint> thisFrameKeypoints;

		//------------------------get first frame----------------------
		Video->Capture >> thisFrameColor;//read first frame
		cv::cvtColor(thisFrameColor, thisFrameGray, cv::COLOR_RGB2GRAY);//turn the picture to gray for surf match
		cout << "out1";
		thisFrameGPU.upload(thisFrameGray);
		cout << "out2";
		cudaSurf(thisFrameGPU, cv::cuda::GpuMat(), thisFrameKeypoints, thisFrameDescriptor);

		//------------------------for every frame--------------------------
		for (int iFrame = 0; iFrame < Video->FramesCount - 1; iFrame++)
		{
			//---------------------the later frame variables------------------
			cv::Mat nextFrameColor, nextFrameGray;
			cv::cuda::GpuMat nextFrameGPU;
			cv::cuda::GpuMat nextFrameDescriptor;
			std::vector<cv::KeyPoint> nextFrameKeypoints;

			//---------------------get next frame-----------------------------
			Video->Capture >> nextFrameColor;//read next frame
			cv::cvtColor(nextFrameColor, nextFrameGray, cv::COLOR_RGB2GRAY);//turn to gray
			nextFrameGPU.upload(nextFrameGray);
			cudaSurf(nextFrameGPU, cv::cuda::GpuMat(), nextFrameKeypoints, nextFrameDescriptor);

			//------------------------Match-----------------------------------
			std::vector< cv::DMatch > matches;

			cv::Ptr<cv::cuda::DescriptorMatcher> matcher =
				cv::cuda::DescriptorMatcher::createBFMatcher(cudaSurf.defaultNorm());
			matcher->match(thisFrameDescriptor, nextFrameDescriptor, matches);

			for (std::vector< cv::DMatch >::iterator it = matches.begin(); it != matches.end(); it++)
			{
				cv::Point2f &thisFramePoint = thisFrameKeypoints[it->queryIdx].pt;
				cv::Point2f &nextFramePoint = nextFrameKeypoints[it->trainIdx].pt;
				Video->AllMatchedPoints[iFrame].push_back(*(new VMatchedPoint(thisFramePoint, nextFramePoint)));
			}

			Video->GoodMatchedPointsCount[iFrame] = 
				Video->AllMatchedPointsCount[iFrame] 
				= Video->AllMatchedPoints[iFrame].end() - Video->AllMatchedPoints[iFrame].begin();

			thisFrameColor = nextFrameColor;
			thisFrameGray = thisFrameGray;//In fact, this row is not necessary
			thisFrameGPU = nextFrameGPU;
			thisFrameDescriptor = nextFrameDescriptor;
			thisFrameKeypoints.clear();
			thisFrameKeypoints = nextFrameKeypoints;

			std::cout << ".";
		}
		std::cout << endl << "done" << endl;
	}
	catch (const std::exception &e)
	{
		std::cout << "GPU Motion Estimate Failed---error:  " << e.what() << std::endl;
		system("pause");
	}
}

void VStabilizationHandle::GetGlobalGoodPointsWithRANSAC()
{
	if (this->_globalGoodPointRANSACFilter < 1.0)
	{
		std::cout << "RANSAC-GetGoodPoints---Begin:" << endl;
		try
		{
			//------------------------for every frame--------------------------
			for (int iFrame = 0; iFrame < Video->FramesCount - 1; iFrame++)
			{
				//first iterator
				std::vector<VMatchedPoint>::iterator itBegin = Video->AllMatchedPoints[iFrame].begin();
				//loop variable
				std::vector<VMatchedPoint>::iterator itAt;

				//---------------------build vector for getting homography---------------
				std::vector<cv::Point2f> srcPoints, dstPoints;
				srcPoints.clear();
				dstPoints.clear();

				itAt = itBegin;
				for (int iPoint = 0; iPoint < Video->GoodMatchedPointsCount[iFrame]; iPoint++, itAt++)
				{
					srcPoints.push_back(itAt->SrcPoint);
					dstPoints.push_back(itAt->DstPoint);
				}

				//------------------------------build Global Homography-------------------
				cv::Mat globalH = cv::findHomography(
					srcPoints,
					dstPoints,
					cv::RANSAC);
				cv::Mat globalHfloat;
				globalH.convertTo(globalHfloat, CV_32FC1);
				std::cout << "Global Homography:  Columns: " << globalHfloat.cols << " ---Rows: " << globalHfloat.rows;
				float H00 = globalHfloat.at<float>(0, 0);
				float H01 = globalHfloat.at<float>(0, 1);
				float H02 = globalHfloat.at<float>(0, 2);
				float H10 = globalHfloat.at<float>(1, 0);
				float H11 = globalHfloat.at<float>(1, 1);
				float H12 = globalHfloat.at<float>(1, 2);
				std::cout << "----Get H OK" << endl;

				//-------------------------caucalate Global Error--------------------------
				itAt = itBegin;
				for (int iPoint = 0; iPoint < Video->GoodMatchedPointsCount[iFrame]; iPoint++, itAt++)
				{
					float estmDstPointx = H00 * itAt->SrcPoint.x + H01 * itAt->SrcPoint.y + H02;
					float estmDstPointy = H10 * itAt->SrcPoint.x + H11 * itAt->SrcPoint.y + H12;
					itAt->Error = (itAt->DstPoint.x - estmDstPointx) * (itAt->DstPoint.x - estmDstPointx)
						+ (itAt->DstPoint.y - estmDstPointx) * (itAt->DstPoint.y - estmDstPointx);

					/*float dx = estmDstPointx - itAt->DstPoint.x;
					float dy = estmDstPointy - itAt->DstPoint.y;
					float newx = itAt->SrcPoint.x + dx * 10;
					float newy = itAt->SrcPoint.y + dy * 10;
					cv::Point2f newpoint(newx, newy);

					
					arrowedLine(
						Video->ColorPic[iFrame],
						itAt->SrcPoint,
						newpoint,
						cv::Scalar(255, 0, 0),
						2, 8);

					float oldx = itAt->SrcPoint.x + (itAt->DstPoint.x - itAt->SrcPoint.x) * 10;
					float oldy = itAt->SrcPoint.y + (itAt->DstPoint.y - itAt->SrcPoint.y) * 10;
					cv::Point2f oldpoint(oldx, oldy);*/
					
					/*arrowedLine(
						Video->ColorPic[iFrame],
						itAt->SrcPoint,
						oldpoint,
						cv::Scalar(0, 255, 0),
						2, 8);*/
				}
				std::sort(itBegin, itBegin + Video->GoodMatchedPointsCount[iFrame],
					CmpMatchedPointsWithError);

				//---------------------------select Good Points------------------------------
				Video->GoodMatchedPointsCount[iFrame] = Video->GoodMatchedPointsCount[iFrame] * this->_globalGoodPointRANSACFilter;
				/*
				itAt = itBegin;
				for (int iPoint = 0; iPoint < Video->GoodMatchedPointsCount[iFrame]; iPoint++, itAt++)
				{
					arrowedLine(
						Video->ColorPic[iFrame],
						itAt->SrcPoint,
						itAt->DstPoint,
						cv::Scalar(0, 0, 255),
						2, 8);
				}
				cv::imshow("Global Good Points", Video->ColorPic[iFrame]);
				cv::waitKey(1000);*/

				std::cout << ".";

			}
			std::cout << endl << "done" << endl;
		}
		catch (const std::exception &e)
		{
			std::cout << "RANSAC-GetGoodPoints Failed---error:  " << e.what() << std::endl;
			system("pause");
		}
	}
}

void VStabilizationHandle::GetGlobalGoodPointsWithMinDis()
{
	if (this->_globalGoodPointMinDISFilter < 1.0)
	{
		std::cout << "MINDIS-GetGoodPoints---Begin:" << endl;
		try
		{
			//------------------------for every frame--------------------------
			for (int iFrame = 0; iFrame < Video->FramesCount - 1; iFrame++)
			{
				//first iterator
				std::vector<VMatchedPoint>::iterator itBegin = Video->AllMatchedPoints[iFrame].begin();
				//loop variable
				std::vector<VMatchedPoint>::iterator itAt;

				//sort
				std::sort(itBegin, itBegin + Video->GoodMatchedPointsCount[iFrame],
					CmpMatchedPointsWithDis);

				//---------------------------select Good Points------------------------------
				Video->GoodMatchedPointsCount[iFrame] = Video->AllMatchedPointsCount[iFrame] * this->_globalGoodPointMinDISFilter;

				itAt = itBegin;
				for (int iPoint = 0; iPoint < Video->GoodMatchedPointsCount[iFrame]; iPoint++, itAt++)
				{
					/*arrowedLine(
						Video->ColorPic[iFrame],
						itAt->SrcPoint,
						itAt->DstPoint,
						cv::Scalar(0, 255, 0),
						2, 8);*/
				}
				//stringstream ssPC;
				//ssPC << goodMatchedPointsCount;
				//cv::imshow("Global Good Points", Video->ColorPic[iFrame]);
				//cv::waitKey(1000);

				std::cout << ".";

			}
			std::cout << endl << "done" << endl;
		}
		catch (const std::exception &e)
		{
			std::cout << "GetGoodPoints Failed---error:  " << e.what() << std::endl;
			system("pause");
		}
	}
}

void VStabilizationHandle::SelectMatchedPoints()
{
	try
	{
		std::cout << "Select Matched Points Begin:" << endl;

		//------------------------for every frame--------------------------
		for (int iFrame = 0; iFrame < Video->FramesCount - 1; iFrame++)
		{
			VTriCell **frameCells = Video->Cells[iFrame];
			
			for (int iRow = 0; iRow < Video->CellRows; iRow++)
				for (int iCol = 0; iCol < Video->CellCols * 2; iCol++)
				{
					VTriCell *iCell = &frameCells[iRow][iCol];
					SelectACellPoints(iCell);
					//cout << iCell->GoodMatchedCount;
					/*
			
					for (std::vector<VMatchedPoint>::iterator it = iCell->MatchedPoints.begin(); it != iCell->MatchedPoints.end(); it++)
					{
						cv::arrowedLine(
							Video->ColorPic[iFrame],
							it->SrcPoint,
							it->DstPoint,
							cv::Scalar(0, 0, 255),
							2, 8);
					}*/
				}

			stringstream ss;
			ss << iFrame;
			cv::String title(ss.str());
			//cv::imshow(title, Video->ColorPic[iFrame]);
			//cv::waitKey(5000);
		}

		std::cout << endl << "Select Matched Points Done" << endl;
	}
	catch (const std::exception &e)
	{
		std::cout << "Select Matched Points Failed---error:  " << e.what() << std::endl;
		system("pause");
	}
}

void VStabilizationHandle::SelectACellPoints(VTriCell *pCell)
{
	pCell->GoodMatchedCount = pCell->MatchedPoints.end() - pCell->MatchedPoints.begin();
	cout << pCell->GoodMatchedCount << endl;
	if (4 > pCell->GoodMatchedCount)
		return;
	/*
	if (5 == pCell->GoodMatchedCount)
	{
		for (std::vector<VMatchedPoint>::iterator it = pCell->AllMatchedPoints.begin(); it != pCell->AllMatchedPoints.end(); it++)
		{
			cv::arrowedLine(
				thisFrameDebug,
				it->SrcPoint,
				it->DstPoint,
				cv::Scalar(0, 0, 255),
				2, 8);
		}
		cv::imshow("DEBUG5", thisFrameDebug);
		cv::waitKey(1000);
	}*/


	std::vector<cv::Point2f> srcPoints, dstPoints;
	for (std::vector<VMatchedPoint>::iterator it = pCell->MatchedPoints.begin(); it != pCell->MatchedPoints.end(); it++)
	{
		srcPoints.push_back(it->SrcPoint);//ThisFrame
		dstPoints.push_back(it->DstPoint);//NextFrame
	}

	cv::Mat H = cv::findHomography(
		srcPoints,
		dstPoints,
		cv::RANSAC);

	cv::Mat Hfloat;
	H.convertTo(Hfloat, CV_32FC1);
	cout << "Columns: " << Hfloat.cols << " ---Rows: " << Hfloat.rows;
	float H00 = Hfloat.at<float>(0, 0);
	float H01 = Hfloat.at<float>(0, 1);
	float H02 = Hfloat.at<float>(0, 2);
	float H10 = Hfloat.at<float>(1, 0);
	float H11 = Hfloat.at<float>(1, 1);
	float H12 = Hfloat.at<float>(1, 2);
	cout << "Get H OK" << endl;

	for (std::vector<VMatchedPoint>::iterator it = pCell->MatchedPoints.begin(); it != pCell->MatchedPoints.end(); it++)
	{
		cv::Point2f &srcPoint = it->SrcPoint;
		cv::Point2f &dstPoint = it->DstPoint;
		cv::Point2f idealPoint;
		idealPoint.x = H00 * srcPoint.x + H01 * srcPoint.y + H02;
		idealPoint.y = H10 * srcPoint.x + H11 * srcPoint.y + H12;

		float errorDis = (idealPoint.x - dstPoint.x) * (idealPoint.x - dstPoint.x)
			+ (idealPoint.y - dstPoint.y) * (idealPoint.y - dstPoint.y);

		it->Error = errorDis;
	}

	std::sort(pCell->MatchedPoints.begin(), pCell->MatchedPoints.end(), CmpMatchedPointsWithError);

	if (pCell->GoodMatchedCount > _expectGoodPointsPerCells)
		pCell->GoodMatchedCount = _expectGoodPointsPerCells;

	std::vector<VMatchedPoint>::iterator it = pCell->MatchedPoints.begin();
	for (int iPoint = 0; iPoint < pCell->GoodMatchedCount; iPoint++, it++)
	{
		pCell->MatchedPoints.push_back(*it);
	}
}
