#pragma once

#include "stdafx.h"

class VMatchedPoint
{
public:
	cv::Point2f SrcPoint;
	cv::Point2f DstPoint;
	float Dis;
	float Error;

	double w[2][2];//[0][0]:lefttop;[0][1]:righttop;[1][0]:leftbottom;[1][1]:rightborrom

	VMatchedPoint(cv::Point2f &src, cv::Point2f &dst);
	~VMatchedPoint();
};

