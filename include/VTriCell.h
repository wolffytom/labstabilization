#pragma once

#include "stdafx.h"

using namespace std;

class VMatchedPoint;

class VTriCell
{
public:
	vector<VMatchedPoint> MatchedPoints;
	int GoodMatchedCount = 0;

	VTriCell();
	~VTriCell();
};

