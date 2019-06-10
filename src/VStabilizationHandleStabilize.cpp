#include "stdafx.h"
#include "VStabilizationHandle.h"

void VStabilizationHandle::Stabilize()
{
	cout << "Stabilize begin:";

	VLPOptimalPath *optimal = new VLPOptimalPath(Video, _reservedRate);
	MonControlPoints = optimal->MotionDCP;
	InvControlPoints = optimal->InvOpticalCP;
	NewControlPoints = optimal->OpticalCP;

	Video->ReviseModel = new VMotionModel*[Video->FramesCount - 1];
	for (int iFrame = 0; iFrame < Video->FramesCount - 1; iFrame++)
	{
		Video->ReviseModel[iFrame] = new VMotionModel(StandardControlPoints, InvControlPoints[iFrame], Video);
		cout << ".";
	}

	cout << "Stabilize done!";
}

void VStabilizationHandle::StabilizeWithRecModel()
{
	cout << "Stabilize begin:";
	
	VRecCellModel *recCellModel = new VRecCellModel(Video);

	cout << "Stabilize done!";
}