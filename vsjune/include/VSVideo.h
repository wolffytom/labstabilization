#pragma once
#include "VSData.h"
#include "stdafx.h"

class VSVideo :
	public VSData
{
public:
	char FileName[255];
	cv::VideoCapture Capture;
	cv::Mat *ColorPic;
	double Fps;
	int Width;
	int Height;
	int FramesCount;

	VSVideo();
	~VSVideo();
};

