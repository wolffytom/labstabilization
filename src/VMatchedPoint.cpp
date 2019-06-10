#include "stdafx.h"
#include "VMatchedPoint.h"


VMatchedPoint::VMatchedPoint(cv::Point2f &src, cv::Point2f &dst)
{
	memcpy(&SrcPoint, &src, sizeof(SrcPoint));
	memcpy(&DstPoint, &dst, sizeof(DstPoint));
	Dis = (SrcPoint.x - DstPoint.x) * (SrcPoint.x - DstPoint.x) + (SrcPoint.y - DstPoint.y) * (SrcPoint.y - DstPoint.y);
}


VMatchedPoint::~VMatchedPoint()
{
}
