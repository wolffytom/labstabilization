#pragma once
#include "stdafx.h"

#ifdef HAVE_CLP
#include <ClpSimplex.hpp>
#include <ClpPresolve.hpp>
#include <ClpPrimalColumnSteepest.hpp>
#include <ClpDualRowSteepest.hpp>
#endif

class VLPOptimalPath
{
public:
	double xEdgeDis;
	double yEdgeDis;
	double reservedRate;

	VVideo *pVideo;
	double expandRate = 1;
	double maxen = 20;
	double wn = 0.5;
	double w1 = 10;
	double w2 = 0.5;
	double w3 = 2;
	VMotionModel **OptimalMotionModel;

	cv::Point2f ***MotionDCP;
	cv::Point2f ***OpticalCP;
	cv::Point2f ***InvOpticalCP;

	VLPOptimalPath(VVideo *parPVideo, double parReservedRate);
	~VLPOptimalPath();
	void BuildLinearProgrammingModel();
	void BuildMotionDCP();
};

