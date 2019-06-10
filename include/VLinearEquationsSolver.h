#pragma once
class VLinearEquationsSolver
{
public:
	int dimen;
	double **A;
	double *B;
	double *X;

	VLinearEquationsSolver(int parDimen, double **parA, double *parB, double *parX);
	~VLinearEquationsSolver();

	void Solve();
};

