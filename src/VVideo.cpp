#include "stdafx.h"
#include "VVideo.h"
#include <string.h>


VVideo::VVideo(char *filename, int cellRows, int cellCols)
{
	CellRows = cellRows;
	CellCols = cellCols;

	Capture.open(filename);
	std::strcpy(FileName, filename);
	Fps = Capture.get(CV_CAP_PROP_FPS);
	Width = Capture.get(CV_CAP_PROP_FRAME_WIDTH);
	Height = Capture.get(CV_CAP_PROP_FRAME_HEIGHT);
	FramesCount = Capture.get(CV_CAP_PROP_FRAME_COUNT);

	CellHeight = ((float)Height) / CellRows;
	CellWidth = ((float)Width) / CellCols;

	//ColorPic = new cv::Mat[FramesCount - 1];
	AllMatchedPoints = new vector<VMatchedPoint>[FramesCount - 1];
	AllMatchedPointsCount = new int[FramesCount - 1];
	GoodMatchedPointsCount = new int[FramesCount - 1];
	MotionModel = new VMotionModel*[FramesCount - 1];

	Cells = new VTriCell**[FramesCount - 1];
	for (int iFrame = 0; iFrame < FramesCount - 1; iFrame++)
	{
		Cells[iFrame] = new VTriCell*[CellRows];
		for (int iRow = 0; iRow < CellRows; iRow++)
		{
#ifdef TRIANGULAR_DEVIDE_POINTS
			Cells[iFrame][iRow] = new VTriCell[CellCols * 2];
#else
			Cells[iFrame][iRow] = new VTriCell[CellCols];
#endif
		}
	}

	PrintVideoInfo();
}

void VVideo::RefreshCapture()
{
	Capture.open(FileName);
}

void VVideo::PrintVideoInfo()
{
	cout << "InputVideo:  " << FileName << endl;
	cout << "InputVideoFramesCount:  " << FramesCount << endl;
	cout << "InputVideoWidth:  " << Width << endl;
	cout << "InputVideoHeight:  " << Height << endl;
	cout << "InputFPS:  " << Fps << endl << "--------------------" << endl;
}

VVideo::~VVideo()
{
}
