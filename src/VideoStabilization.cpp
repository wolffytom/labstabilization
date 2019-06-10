// VideoStabilization.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "VStabilizationHandle.h"
#include <time.h>

int _tmain(int argc, _TCHAR* argv[])
{
	clock_t start, finish;
	start = clock();
	new VStabilizationHandle("1000-2000.avi");
	finish = clock();
	double duration = ((double)(finish - start) / CLOCKS_PER_SEC) / 3600.0;
	cout << "Done in " << duration << " hours";
	return 0;
}