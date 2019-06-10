#include "stdafx.h"
#include "VLPOptimalPath.h"

#ifdef HAVE_CLP
/*if have clp, we can use these code to get lp optimal path*/
#include <ClpSimplex.hpp>
#include <ClpPresolve.hpp>
#include <ClpPrimalColumnSteepest.hpp>
#include <ClpDualRowSteepest.hpp>
#endif
#include <vector>


VLPOptimalPath::VLPOptimalPath(VVideo *parPVideo, double parReservedRate)
{
	pVideo = parPVideo;
	reservedRate = parReservedRate;
	xEdgeDis = pVideo->Width * (1 - reservedRate) / 2;
	yEdgeDis = pVideo->Height * (1 - reservedRate) / 2;

	BuildMotionDCP();
	BuildLinearProgrammingModel();
}

VLPOptimalPath::~VLPOptimalPath()
{
}

void VLPOptimalPath::BuildMotionDCP()
{
	MotionDCP = new cv::Point2f**[pVideo->FramesCount - 1];
	OpticalCP = new cv::Point2f**[pVideo->FramesCount];
	InvOpticalCP = new cv::Point2f**[pVideo->FramesCount];
	int CPCols = pVideo->CellCols + 1;//Control Points Columns count
	int CPRows = pVideo->CellRows + 1;//Control Points Rows count
	for (int iFrame = 0; iFrame < pVideo->FramesCount - 1; iFrame++)
	{
		MotionDCP[iFrame] = new cv::Point2f*[CPRows];
		for (int cRow = 0; cRow < CPRows; cRow++)
		{
			MotionDCP[iFrame][cRow] = new cv::Point2f[CPCols];
			for (int cCol = 0; cCol < CPCols; cCol++)
			{
				MotionDCP[iFrame][cRow][cCol].x = pVideo->MotionModel[iFrame]->ControlPoints[cRow][cCol].x 
					- pVideo->MotionModel[iFrame]->StandardControlPoints[cRow][cCol].x;
				MotionDCP[iFrame][cRow][cCol].y = pVideo->MotionModel[iFrame]->ControlPoints[cRow][cCol].y
					- pVideo->MotionModel[iFrame]->StandardControlPoints[cRow][cCol].y;
				MotionDCP[iFrame][cRow][cCol].x *= expandRate;
				MotionDCP[iFrame][cRow][cCol].y *= expandRate;
			}
		}
	}

	for (int iFrame = 0; iFrame < pVideo->FramesCount; iFrame++)
	{
		OpticalCP[iFrame] = new cv::Point2f*[CPRows];
		InvOpticalCP[iFrame] = new cv::Point2f*[CPRows];
		for (int cRow = 0; cRow < CPRows; cRow++)
		{
			OpticalCP[iFrame][cRow] = new cv::Point2f[CPCols];
			InvOpticalCP[iFrame][cRow] = new cv::Point2f[CPCols];
			for (int cCol = 0; cCol < CPCols; cCol++)
			{
				OpticalCP[iFrame][cRow][cCol].x = pVideo->MotionModel[0]->StandardControlPoints[cRow][cCol].x;
				OpticalCP[iFrame][cRow][cCol].y = pVideo->MotionModel[0]->StandardControlPoints[cRow][cCol].y;
				InvOpticalCP[iFrame][cRow][cCol].x = pVideo->MotionModel[0]->StandardControlPoints[cRow][cCol].x;
				InvOpticalCP[iFrame][cRow][cCol].y = pVideo->MotionModel[0]->StandardControlPoints[cRow][cCol].y;
			}
		}
	}
}

void VLPOptimalPath::BuildLinearProgrammingModel()
{
	const int INF = 100;

	/**********************problem abstract**********************\
	Using Linear-Programming way to get the optimal path.
	Linear-Programming is the way to solve problem as min CtX
	while
	rowlb <= Ax <=rowub
	and
	collb <= x <=colub
	
	A is a matric
	x is unknown Variable
	\*************************************************************/

	/*-----------------------LP Parameters------------------------*/
	//vector C
	std::vector<double> C_;

	//MatricA
	std::vector<int> ARows_, ACols_;
	std::vector<double> AElems_;
	//define a Macro to add an element to matric A on (row,col)
#define A_ADD(row,col,elem) ARows_.push_back(row); ACols_.push_back(col); AElems_.push_back(elem);

	//limiting
	std::vector<double> Rowub_, Rowlb_, Colub_, Collb_;

	//other Parameters
	int CPCols = pVideo->CellCols + 1;//Control Points Columns count
	int CPRows = pVideo->CellRows + 1;//Control Points Rows count
	int dimen = CPCols * CPRows * 2;//Dimension Count
	int innerDimen = (CPCols - 1) * (CPRows - 1) * 2;
	//Get Control Points Cell's X'x index
#define GCP_XI(cpRowIndex,cpColIndex) ((cpRowIndex) * (CPCols) + (cpColIndex)) * 2
	int framesCount = pVideo->FramesCount;
	
	/****************structure Column's explanation**************\
	dimen = CPCols * CPRows * 2
	the first (FramesCount*dimen) columns:
	means the Optimal Revise to Optimal Picture from old frames
	the Nth dimen Column means Variables
	dx[0][0],dy[0][0].....dx[0][CPCols-1],dy[0][CPCols-1],dx[1][0],dy[1][0]
	.......dx[CPRows-1][CPCols-1],dy[CPRows-1][CPCols-1] on Nth frame

	the next (FramesCount-1)*(dimen) columns means Each (FramesCount-1) e1 vector at (dimen) dimension
	the next (FramesCount-2)*(dimen) columns means Each (FramesCount-2) e2 vector at (dimen) dimension
	the next (FramesCount-3)*(dimen) columns means Each (FramesCount-3) e3 vector at (dimen) dimension
	\*************************************************************/

	//Get Column Index macro
#define GCI_R(frameIndex) (frameIndex) * dimen //Get Column Index of frameIndex'th frane's Revise path
	int esRowStartColumn = framesCount * dimen;
	int esRowDimen = (CPCols - 1) * CPRows * 2;
#define GCI_ESROW(frameIndex) esRowStartColumn + (frameIndex) * esRowDimen;
	int esColStartColumn = esRowStartColumn + framesCount * esRowDimen;
	int esColDimen = CPCols * (CPRows - 1) * 2;
#define GCI_ESCOL(frameIndex) esColStartColumn + (frameIndex) * esColDimen;
	int e1StartColumn = esColStartColumn + framesCount * esColDimen;
#define GCI_E1(frameIndex) e1StartColumn + (frameIndex) * dimen //Get Column Index of frameIndex'th e1's [0][0].x index
	int e2StartColumn = e1StartColumn + (framesCount - 1) * dimen;
#define GCI_E2(frameIndex) e2StartColumn + (frameIndex) * dimen //Get Column Index of frameIndex'th e2's [0][0].x index
	int e3StartColumn = e2StartColumn + (framesCount - 2) * dimen;
#define GCI_E3(frameIndex) e3StartColumn + (frameIndex) * dimen //Get Column Index of frameIndex'th e2's [0][0].x index
	int AColumnsCount = e3StartColumn + (framesCount - 3) * dimen;

	/*------------------Build Columns limiting and C-----------------*/
	for (int aCol = 0; aCol < esRowStartColumn; aCol++)
	{
		Colub_.push_back(INF);
		Collb_.push_back(-INF);
		C_.push_back(0);
	}
	for (int aCol = esRowStartColumn; aCol < e1StartColumn; aCol++)
	{
		Colub_.push_back(maxen);
		Collb_.push_back(0);
		C_.push_back(wn);
	}
	for (int aCol = e1StartColumn; aCol < e2StartColumn; aCol++)
	{
		Colub_.push_back(INF);
		Collb_.push_back(0);
		C_.push_back(w1);
	}
	for (int aCol = e2StartColumn; aCol < e3StartColumn; aCol++)
	{
		Colub_.push_back(INF);
		Collb_.push_back(0);
		C_.push_back(w2);
	}
	for (int aCol = e3StartColumn; aCol < AColumnsCount; aCol++)
	{
		Colub_.push_back(INF);
		Collb_.push_back(0);
		C_.push_back(w3);
	}

	/*-------------------for Inclusion constraint-------------------*/
	// Top: DY < yEdgeDis   Bottom: -DY < yEdgeDis
	// Left: DX < xEdgeDis   Right: -DX < xEdgeDis
	
	for (int iFrame = 0; iFrame < framesCount; iFrame++)
	{
		for (int cCol = 0; cCol < CPCols; cCol++)
		{
			int topYIndex = GCP_XI(0, cCol) + 1;
			Colub_[topYIndex] = yEdgeDis;

			int bottomYIndex = GCP_XI(CPRows - 1, cCol) + 1;
			Collb_[bottomYIndex] = -yEdgeDis;
		}
		for (int cRow = 0; cRow < CPCols; cRow++)
		{
			int leftXIndex = GCP_XI(cRow, 0);
			Colub_[leftXIndex] = xEdgeDis;

			int rightXIndex = GCP_XI(cRow, CPCols - 1);
			Collb_[rightXIndex] = -xEdgeDis;
		}
	}

	/*---------------------------Build Rows---------------------------*/
	int aRow = 0;

	// for esRows -e <= left - right <= e
	for (int iFrame = 0; iFrame < framesCount; iFrame++)
	{
		for (int cRow = 0; cRow < CPRows; cRow++)
			for (int cCol = 0; cCol < CPCols - 1; cCol++)
			{
				int leftxIndex = GCI_R(iFrame) + GCP_XI(cRow, cCol);
				int leftyIndex = leftxIndex + 1;
				int rightxIndex = GCI_R(iFrame) + GCP_XI(cRow, cCol + 1);
				int rightyIndex = rightxIndex + 1;
				int exIndex = GCI_ESROW(iFrame) + (cRow * (CPCols - 1) + cCol) * 2;
				int eyIndex = exIndex + 1;

				//left - right + e >= 0
				//x
				A_ADD(aRow, leftxIndex, 1);
				A_ADD(aRow, rightxIndex, -1);
				A_ADD(aRow, exIndex, 1);
				Rowlb_.push_back(0);
				Rowub_.push_back(INF);
				aRow++;

				//y
				A_ADD(aRow, leftyIndex, 1);
				A_ADD(aRow, rightyIndex, -1);
				A_ADD(aRow, eyIndex, 1);
				Rowlb_.push_back(0);
				Rowub_.push_back(INF);
				aRow++;

				//left - right - e <= 0
				//x
				A_ADD(aRow, leftxIndex, 1);
				A_ADD(aRow, rightxIndex, -1);
				A_ADD(aRow, exIndex, -1);
				Rowlb_.push_back(-INF);
				Rowub_.push_back(0);
				aRow++;

				//y
				A_ADD(aRow, leftyIndex, 1);
				A_ADD(aRow, rightyIndex, -1);
				A_ADD(aRow, eyIndex, -1);
				Rowlb_.push_back(-INF);
				Rowub_.push_back(0);
				aRow++;
			}
	}

	// for esCols -e <= Up - Down <= e
	for (int iFrame = 0; iFrame < framesCount; iFrame++)
	{
		for (int cRow = 0; cRow < CPRows - 1; cRow++)
			for (int cCol = 0; cCol < CPCols; cCol++)
			{
				int UpxIndex = GCI_R(iFrame) + GCP_XI(cRow, cCol);
				int UpyIndex = UpxIndex + 1;
				int DownxIndex = GCI_R(iFrame) + GCP_XI(cRow + 1, cCol);
				int DownyIndex = DownxIndex + 1;
				int exIndex = GCI_ESCOL(iFrame) + (cRow * CPCols + cCol) * 2;
				int eyIndex = exIndex + 1;

				//Up - Down + e >= 0
				//x
				A_ADD(aRow, UpxIndex, 1);
				A_ADD(aRow, DownxIndex, -1);
				A_ADD(aRow, exIndex, 1);
				Rowlb_.push_back(0);
				Rowub_.push_back(INF);
				aRow++;

				//y
				A_ADD(aRow, UpyIndex, 1);
				A_ADD(aRow, DownyIndex, -1);
				A_ADD(aRow, eyIndex, 1);
				Rowlb_.push_back(0);
				Rowub_.push_back(INF);
				aRow++;

				//Up - Down - e <= 0
				//x
				A_ADD(aRow, UpxIndex, 1);
				A_ADD(aRow, DownxIndex, -1);
				A_ADD(aRow, exIndex, -1);
				Rowlb_.push_back(-INF);
				Rowub_.push_back(0);
				aRow++;

				//y
				A_ADD(aRow, UpyIndex, 1);
				A_ADD(aRow, DownyIndex, -1);
				A_ADD(aRow, eyIndex, -1);
				Rowlb_.push_back(-INF);
				Rowub_.push_back(0);
				aRow++;
			}
	}

	// for -e1 <= F(t) + B(t+1) - B(t) <= e1
	for (int iFrame = 0; iFrame < framesCount - 1; iFrame++)
	{
		for (int cRow = 0; cRow < CPRows; cRow++)
			for (int cCol = 0; cCol < CPCols; cCol++)
			{
				int xIndex = GCP_XI(cRow, cCol);

				int BtxIndex = GCI_R(iFrame) + xIndex;//B(t)
				int BtyIndex = BtxIndex + 1;

				int Bt1xIndex = GCI_R(iFrame + 1) + xIndex;//B(t+1)
				int Bt1yIndex = Bt1xIndex + 1;

				int e1xIndex = GCI_E1(iFrame) + xIndex;//e1
				int e1yIndex = e1xIndex + 1;

				double Ftx = MotionDCP[iFrame][cRow][cCol].x;
				double Fty = MotionDCP[iFrame][cRow][cCol].y;

				//B(t) - B(t+1) - e1 <= F(t)
				//x
				A_ADD(aRow, BtxIndex, 1);
				A_ADD(aRow, Bt1xIndex, -1);
				A_ADD(aRow, e1xIndex, -1);
				Rowlb_.push_back(-INF);
				Rowub_.push_back(Ftx);
				aRow++;
				//y
				A_ADD(aRow, BtyIndex, 1);
				A_ADD(aRow, Bt1yIndex, -1);
				A_ADD(aRow, e1yIndex, -1);
				Rowlb_.push_back(-INF);
				Rowub_.push_back(Fty);
				aRow++;

				//B(t) - B(t+1) + e1 >= F(t)
				//x
				A_ADD(aRow, BtxIndex, 1);
				A_ADD(aRow, Bt1xIndex, -1);
				A_ADD(aRow, e1xIndex, 1);
				Rowlb_.push_back(Ftx);
				Rowub_.push_back(INF);
				aRow++;
				//y
				A_ADD(aRow, BtyIndex, 1);
				A_ADD(aRow, Bt1yIndex, -1);
				A_ADD(aRow, e1yIndex, 1);
				Rowlb_.push_back(Fty);
				Rowub_.push_back(INF);
				aRow++;
			}
	}

	
	// for -e2 <= F(t+1) - F(t) + B(t+2) - 2*B(t+1) + B(t) <= e2
	for (int iFrame = 0; iFrame < framesCount - 2; iFrame++)
	{
		for (int cRow = 0; cRow < CPRows; cRow++)
			for (int cCol = 0; cCol < CPCols; cCol++)
			{
				int xIndex = GCP_XI(cRow, cCol);

				int BtxIndex = GCI_R(iFrame) + xIndex;//B(t)
				int BtyIndex = BtxIndex + 1;

				int Bt1xIndex = GCI_R(iFrame + 1) + xIndex;//B(t+1)
				int Bt1yIndex = Bt1xIndex + 1;

				int Bt2xIndex = GCI_R(iFrame + 2) + xIndex;//B(t+2)
				int Bt2yIndex = Bt1xIndex + 2;

				int e2xIndex = GCI_E2(iFrame) + xIndex;//e2
				int e2yIndex = e2xIndex + 1;

				double Ftx = MotionDCP[iFrame][cRow][cCol].x;
				double Fty = MotionDCP[iFrame][cRow][cCol].y;

				double Ft1x = MotionDCP[iFrame + 1][cRow][cCol].x;
				double Ft1y = MotionDCP[iFrame + 1][cRow][cCol].y;

				//-B(t+2) + 2*B(t+1) -B(t) - e2 <= F(t+1) - F(t)
				//x
				A_ADD(aRow, BtxIndex, -1);
				A_ADD(aRow, Bt1xIndex, 2);
				A_ADD(aRow, Bt2xIndex, -1);
				A_ADD(aRow, e2xIndex, -1);
				Rowlb_.push_back(-INF);
				Rowub_.push_back(Ft1x - Ftx);
				aRow++;
				//y
				A_ADD(aRow, BtyIndex, -1);
				A_ADD(aRow, Bt1yIndex, 2);
				A_ADD(aRow, Bt2yIndex, -1);
				A_ADD(aRow, e2yIndex, -1);
				Rowlb_.push_back(-INF);
				Rowub_.push_back(Ft1y - Fty);
				aRow++;

				//-B(t+2) + 2*B(t+1) -B(t) + e2 >= F(t+1) - F(t)
				//x
				A_ADD(aRow, BtxIndex, -1);
				A_ADD(aRow, Bt1xIndex, 2);
				A_ADD(aRow, Bt2xIndex, -1);
				A_ADD(aRow, e2xIndex, 1);
				Rowlb_.push_back(Ft1x - Ftx);
				Rowub_.push_back(INF);
				aRow++;
				//y
				A_ADD(aRow, BtyIndex, -1);
				A_ADD(aRow, Bt1yIndex, 2);
				A_ADD(aRow, Bt2yIndex, -1);
				A_ADD(aRow, e2yIndex, 1);
				Rowlb_.push_back(Ft1y - Fty);
				Rowub_.push_back(INF);
				aRow++;
			}
	}
	/*
	// for -e3 <= F(t+2) - 2*F(t+1) + F(t) + B(t+3) - 3*B(t+2) + 3*B(t+1) - B(t) <= e3
	for (int iFrame = 0; iFrame < framesCount - 3; iFrame++)
	{
		for (int cRow = 0; cRow < CPRows; cRow++)
			for (int cCol = 0; cCol < CPCols; cCol++)
			{
				int xIndex = GCP_XI(cRow, cCol);

				int BtxIndex = GCI_R(iFrame) + xIndex;//B(t)
				int BtyIndex = BtxIndex + 1;

				int Bt1xIndex = GCI_R(iFrame + 1) + xIndex;//B(t+1)
				int Bt1yIndex = Bt1xIndex + 1;

				int Bt2xIndex = GCI_R(iFrame + 2) + xIndex;//B(t+2)
				int Bt2yIndex = Bt1xIndex + 2;

				int Bt3xIndex = GCI_R(iFrame + 3) + xIndex;//B(t+3)
				int Bt3yIndex = Bt1xIndex + 3;

				int e3xIndex = GCI_E3(iFrame) + xIndex;//e3
				int e3yIndex = e3xIndex + 1;

				double Ftx = MotionDCP[iFrame][cRow][cCol].x;
				double Fty = MotionDCP[iFrame][cRow][cCol].y;

				double Ft1x = MotionDCP[iFrame + 1][cRow][cCol].x;
				double Ft1y = MotionDCP[iFrame + 1][cRow][cCol].y;

				double Ft2x = MotionDCP[iFrame + 2][cRow][cCol].x;
				double Ft2y = MotionDCP[iFrame + 2][cRow][cCol].y;

				//-B(t+3) + 3*B(t+2) - 3*B(t+1) + B(t) - e3 <= F(t+2) - 2*F(t+1) + F(t)
				//x
				A_ADD(aRow, BtxIndex, 1);
				A_ADD(aRow, Bt1xIndex, -3);
				A_ADD(aRow, Bt2xIndex, 3);
				A_ADD(aRow, Bt3xIndex, -1);
				A_ADD(aRow, e3xIndex, -1);
				Rowlb_.push_back(-INF);
				Rowub_.push_back(Ft2x - 2*Ft1x + Ftx);
				aRow++;
				//y
				A_ADD(aRow, BtyIndex, 1);
				A_ADD(aRow, Bt1yIndex, -3);
				A_ADD(aRow, Bt2yIndex, 3);
				A_ADD(aRow, Bt3yIndex, -1);
				A_ADD(aRow, e3yIndex, -1);
				Rowlb_.push_back(-INF);
				Rowub_.push_back(Ft2y - 2*Ft1y + Fty);
				aRow++;

				//-B(t+3) + 3*B(t+2) - 3*B(t+1) + B(t) + e3 >= F(t+2) - 2*F(t+1) + F(t)
				//x
				A_ADD(aRow, BtxIndex, 1);
				A_ADD(aRow, Bt1xIndex, -3);
				A_ADD(aRow, Bt2xIndex, 3);
				A_ADD(aRow, Bt3xIndex, -1);
				A_ADD(aRow, e3xIndex, 1);
				Rowlb_.push_back(Ft2x - 2 * Ft1x + Ftx);
				Rowub_.push_back(INF);
				aRow++;
				//y
				A_ADD(aRow, BtyIndex, 1);
				A_ADD(aRow, Bt1yIndex, -3);
				A_ADD(aRow, Bt2yIndex, 3);
				A_ADD(aRow, Bt3yIndex, -1);
				A_ADD(aRow, e3yIndex, 1);
				Rowlb_.push_back(Ft2y - 2 * Ft1y + Fty);
				Rowub_.push_back(INF);
				aRow++;
			}
	}*/

	//BuildMatricA

#ifdef HAVE_CLP
	CoinPackedMatrix A(true, &ARows_[0], &ACols_[0], &AElems_[0], AElems_.size());
	A.setDimensions(aRow, e3StartColumn);

	ClpSimplex model(false);
	model.loadProblem(A, &Collb_[0], &Colub_[0], &C_[0], &Rowlb_[0], &Rowub_[0]);

	
	ClpDualRowSteepest dualSteep(1);
	model.setDualRowPivotAlgorithm(dualSteep);

	ClpPrimalColumnSteepest primalSteep(1);
	model.setPrimalColumnPivotAlgorithm(primalSteep);

	model.scaling(1);

	ClpPresolve presolveInfo;
	ClpSimplex *presolvedModel(presolveInfo.presolvedModel(model));

	if (presolvedModel)
	{
		presolvedModel->dual();
		presolveInfo.postsolve(true);
		model.checkSolution();
		model.primal(1);
	}
	else
	{
		model.dual();
		model.checkSolution();
		model.primal(1);
	}

	const double *sol = model.getColSolution();
	/*
	cout << "framescount:" << framesCount << endl;
	for (int iFrame = 0; iFrame < framesCount - 1; iFrame++)
	{
		for (int cRow = 0; cRow < CPRows; cRow++)
		{
			for (int cCol = 0; cCol < CPCols; cCol++)
			{
				int xIndex = GCP_XI(cRow, cCol);

				int BtxIndex = GCI_R(iFrame) + xIndex;//B(t)
				int BtyIndex = BtxIndex + 1;

				int Bt1xIndex = GCI_R(iFrame + 1) + xIndex;//B(t+1)
				int Bt1yIndex = Bt1xIndex + 1;

				int e1xIndex = GCI_E1(iFrame) + xIndex;//e1
				int e1yIndex = e1xIndex + 1;

				double Ftx = MotionDCP[iFrame][cRow][cCol].x;
				double Fty = MotionDCP[iFrame][cRow][cCol].y;

				double e = Ftx - sol[BtxIndex] + sol[Bt1xIndex];
				cout << Bt1xIndex << "!" << Ftx << " -" << sol[BtxIndex] - sol[Bt1xIndex] << "       ";
					/*
					//B(t) - B(t+1) - e1 <= F(t)
					//x
					A_ADD(aRow, BtxIndex, 1);
					A_ADD(aRow, Bt1xIndex, -1);
					A_ADD(aRow, e1xIndex, -1);
					Rowlb_.push_back(-INF);
					Rowub_.push_back(Ftx);
					aRow++;
					//y
					A_ADD(aRow, BtyIndex, 1);
					A_ADD(aRow, Bt1yIndex, -1);
					A_ADD(aRow, e1yIndex, -1);
					Rowlb_.push_back(-INF);
					Rowub_.push_back(Fty);
					aRow++;

					//B(t) - B(t+1) + e1 >= F(t)
					//x
					A_ADD(aRow, BtxIndex, 1);
					A_ADD(aRow, Bt1xIndex, -1);
					A_ADD(aRow, e1xIndex, 1);
					Rowlb_.push_back(Ftx);
					Rowub_.push_back(INF);
					aRow++;
					//y
					A_ADD(aRow, BtyIndex, 1);
					A_ADD(aRow, Bt1yIndex, -1);
					A_ADD(aRow, e1yIndex, 1);
					Rowlb_.push_back(Fty);
					Rowub_.push_back(INF);
					aRow++;
			}
			cout << endl;
		}
	}

	
	for (int iC = 0; iC < e1StartColumn; iC++)
		cout << sol[iC] << " ";
	cout << endl << "------------------" << endl;
	for (int iC = 0; iC < e1StartColumn; iC++)
		cout << C_[iC] << " ";
	cout << endl << "------------------" << endl;
	for (int iC = e1StartColumn; iC < e2StartColumn; iC++)
		cout << sol[iC] << " ";
	cout << endl << "------------------" << endl;
	for (int iC = e1StartColumn; iC < e2StartColumn; iC++)
		cout << C_[iC] << " ";*/

	double ObjSum = 0;
	for (int iC = 0; iC < e2StartColumn; iC++)
	{
		ObjSum += C_[iC] * sol[iC];
	}
	std::cout << "objsum = " << ObjSum << endl;
	for (int iFrame = 0; iFrame < pVideo->FramesCount; iFrame++)
	{
		for (int cRow = 0; cRow < CPRows; cRow++)
		{
			for (int cCol = 0; cCol < CPCols; cCol++)
			{
				int xIndex = GCI_R(iFrame) + GCP_XI(cRow, cCol);
				int yIndex = xIndex + 1;
				OpticalCP[iFrame][cRow][cCol].x += sol[xIndex] / expandRate;
				OpticalCP[iFrame][cRow][cCol].y += sol[yIndex] / expandRate;
				InvOpticalCP[iFrame][cRow][cCol].x -= sol[xIndex] / expandRate;
				InvOpticalCP[iFrame][cRow][cCol].y -= sol[yIndex] / expandRate;
			}
		}
	}
#endif

	
#undef A_ADD
#undef GCI_R
#undef GCI_E1
#undef GCI_E2
#undef GCI_E3
}