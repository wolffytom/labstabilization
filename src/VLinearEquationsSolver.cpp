#include "stdafx.h"
#include "VLinearEquationsSolver.h"

#include <opencv2/core.hpp>


VLinearEquationsSolver::VLinearEquationsSolver(int parDimen, double **parA, double *parB, double *parX)
{
	dimen = parDimen;
	A = parA;
	B = parB;
	X = parX;

	Solve();
}


VLinearEquationsSolver::~VLinearEquationsSolver()
{
}

void VLinearEquationsSolver::Solve()
{
	cv::Mat mA(dimen, dimen, CV_64FC1, &A[0][0]);
	cv::Mat mB(dimen, 1, CV_64FC1, &B[0]);
	cv::Mat mX(dimen, 1, CV_64FC1, &X[0]);

	try
	{
		cv::solve(mA, mB, mX, cv::DECOMP_LU);
	}
	catch (const std::exception &e)
	{
		std::cout << "Solve Equations error:  " << e.what() << std::endl;
		system("pause");
	}
}
