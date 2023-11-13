//#include "stdafx.h"
#include "Kalman.h"
#include <math.h>

#define PI 3.141592654
#define PAY 180

Kalman::Kalman()
{

}

Kalman::~Kalman()
{
}

void Kalman::init(void)
{
	r = 0.23;
	q = 5;

	Xhat_[0] = 0; Xhat_[1] = 0; Xhat_[2] = 0;

	eye3[0][0] = 1; eye3[0][1] = 0; eye3[0][2] = 0;
	eye3[1][0] = 0; eye3[1][1] = 1; eye3[1][2] = 0;
	eye3[2][0] = 0; eye3[2][1] = 0; eye3[2][2] = 1;

	zeros33[0][0] = 0; zeros33[0][1] = 0; zeros33[0][2] = 0;
	zeros33[1][0] = 0; zeros33[1][1] = 0; zeros33[1][2] = 0;
	zeros33[2][0] = 0; zeros33[2][1] = 0; zeros33[2][2] = 0;

	Jh[0][0] = 1; Jh[0][1] = 0; Jh[0][2] = 0;
	Jh[1][0] = 0; Jh[1][1] = 1; Jh[1][2] = 0;
	Jh[2][0] = 0; Jh[2][1] = 0; Jh[2][2] = 1;

	V[0][0] = 1; V[0][1] = 0; V[0][2] = 0;
	V[1][0] = 0; V[1][1] = 1; V[1][2] = 0;
	V[2][0] = 0; V[2][1] = 0; V[2][2] = 1;

	W[0][0] = 0; W[0][1] = 0; W[0][2] = 0;
	W[1][0] = 0; W[1][1] = 0; W[1][2] = 0;
	W[2][0] = 0; W[2][1] = 0; W[2][2] = 0;

	P_[0][0] = 0.5; P_[0][1] = 0; P_[0][2] = 0;
	P_[1][0] = 0; P_[1][1] = 0.5; P_[1][2] = 0;
	P_[2][0] = 0; P_[2][1] = 0; P_[2][2] = 0.5;

	R[0][0] = r; R[0][1] = 0; R[0][2] = 0;
	R[1][0] = 0; R[1][1] = r; R[1][2] = 0;
	R[2][0] = 0; R[2][1] = 0; R[2][2] = r;

	Q[0][0] = q; Q[0][1] = 0; Q[0][2] = 0;
	Q[1][0] = 0; Q[1][1] = q; Q[1][2] = 0;
	Q[2][0] = 0; Q[2][1] = 0; Q[2][2] = q;

	arrCpy33(eye3, F);
	arrCpy1(Xhat_, X1);
	arrCpy1(Xhat_, X2);
	arrCpy33(P_, P);
	arrCpy33(zeros33, p1);
	arrCpy33(zeros33, p2);
	arrCpy33(zeros33, p3);
	arrCpy33(zeros33, JhT);
	arrCpy33(zeros33, FT);
	/*Fu[0][0] = 0; Fu[0][1] = 0;
	Fu[1][0] = 0; Fu[1][1] = 0;
	Fu[2][0] = 0; Fu[2][1] = 0;
	u1[0][0] = 0; u1[0][1] = 0;
	u1[1][0] = 0; u1[1][1] = 0;
	u1[2][0] = 0; u1[2][1] = 0;
	FuT[0][0] = 0; FuT[0][1] = 0; FuT[0][2] = 0;
	FuT[1][0] = 0; FuT[1][1] = 0; FuT[1][2] = 0;
	*/
	dt = 0.100;
}

/*
X=Fx*X+Fu*W
P(k)=Fx*P(k-1)*Fx(T)+Fu*Q*Fu(T);
k=p*H*(H*p*H(T)+R)^(-1)
X=x+k*(y-H*x)
P=(I-k*H)*p
*/
void Kalman::kalman(double ww[2], double zz[3], double hh[3], double x_new[3])
{
	arrCpy1(zz, Z);
	arrCpy1(hh, H);
	w[0] = ww[0];
	w[1] = ww[1];
	//initialize F
	F[0][2] = -dt*w[0] * sin(Z[2] * PI / 180);
	F[1][2] = dt*w[0] * cos(Z[2] * PI / 180);

	//(1-1) project the state ahead : Xhat(k)_=f(Xhat(k-1),u(k-1),0)
	//initialize Xhat
	Xhat_[0] += dt*w[0] * cos(Z[2] * PI / 180);
	Xhat_[1] += dt*w[0] * sin(Z[2] * PI / 180);
	Xhat_[2] = Z[2];

	//(1-2) project the error covariance ahead : P(k)_=F(k)*P(k-1)*FT(k)+W(k)*Q(k-1)*WT(k)
	transform33(F, FT);
	mul_arr33(F, P, p1);
	mul_arr33(p1, FT, p2);
	//mul_arr22(Fu, Q, u1);
	//mul_arr23(u1, FuT, p1);
	mul_arr33(W, Q, w1);
	sum_arr33(p2, w1, P_);

	//(2-1) compute the Kalman gain : K(k)=P(k)_*JhT(k)*(Jh(k)*P(k)_*JhT(k)_+V(k)*R(k)*V(k)_)^-1
	transform33(Jh, JhT);
	transform33(V, VT);
	mul_arr33(Jh, P_, p1);
	mul_arr33(p1, JhT, p2);
	mul_arr33(V, R, p3);
	mul_arr33(p3, VT, p1);
	sum_arr33(p2, p1, p3);
	inverse(p3, p2);
	mul_arr33(P_, JhT, p1);
	mul_arr33(p1, p2, K);

	//(2-2) update estimate with measurment z(k) : Xhat(k)=Xhat(k)_+K(k)*[Z(k)-H]
	sub_arr3(Z, H, X1);
	mul_arr31(K, X1, X2);
	sum_arr3(Xhat_, X2, Xhat);

	//(2-3) update error covariance
	mul_arr33(K, Jh, p1);
	sub_arr33(eye3, p1, p2);
	mul_arr33(p2, P_, P);

	arrCpy1(Xhat, x_new);
}

void Kalman::transform33(double Arr1[3][3], double Arr2[3][3])
{
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			Arr2[j][i] = Arr1[i][j];
		}
	}
}

void Kalman::transform32(double Arr1[3][2], double Arr2[2][3])
{
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 2; j++)
		{
			Arr2[j][i] = Arr1[i][j];
		}
	}
}

void Kalman::transform22(double Arr1[2][2], double Arr2[2][2])
{
	for (int i = 0; i < 2; i++)
	{
		for (int j = 0; j < 2; j++)
		{
			Arr2[j][i] = Arr1[i][j];
		}
	}
}

void Kalman::mul_arr22(double m1[][2], double m2[2][2], double m3[][2])
{
	int i, j, k;
	for (i = 0; i <3; i++)
	{
		for (j = 0; j <2; j++)
		{
			m3[i][j] = 0;
			for (k = 0; k <2; k++)
			{
				m3[i][j] = m3[i][j] + (m1[i][k] * m2[k][j]);
			}
		}
	}
}

void Kalman::mul_arr33(double m1[][3], double m2[3][3], double m3[3][3])
{
	int i, j, k;
	for (i = 0; i <3; i++)
	{
		for (j = 0; j <3; j++)
		{
			m3[i][j] = 0;
			for (k = 0; k <3; k++)
			{
				m3[i][j] = m3[i][j] + (m1[i][k] * m2[k][j]);
			}
		}
	}
}

void Kalman::mul_arr23(double m1[][2], double m2[2][3], double m3[3][3])
{
	int i, j, k;
	for (i = 0; i <3; i++)
	{
		for (j = 0; j <3; j++)
		{
			m3[i][j] = 0;
			for (k = 0; k <2; k++)
			{
				m3[i][j] = m3[i][j] + (m1[i][k] * m2[k][j]);
			}
		}
	}
}

void Kalman::mul_arr32(double m1[][2], double m2[2], double m3[3])
{
	int i, k;
	for (i = 0; i < 3; i++)
	{
		m3[i] = 0;
		for (k = 0; k < 2; k++)
		{
			m3[i] = m3[i] + (m1[i][k] * m2[k]);
		}
	}
}

void Kalman::mul_arr31(double m1[][3], double m2[3], double m3[3])
{
	int i, k;
	for (i = 0; i < 3; i++)
	{
		m3[i] = 0;
		for (k = 0; k < 3; k++)
		{
			m3[i] = m3[i] + (m1[i][k] * m2[k]);
		}
	}
}

void Kalman::sub_arr3(double A1[3], double A2[3], double A3[3])
{
	for (int i = 0; i < 3; i++)
	{
		A3[i] = A1[i] - A2[i];
	}
}

void Kalman::sum_arr3(double A1[3], double A2[3], double A3[3])
{
	for (int i = 0; i < 3; i++)
	{
		A3[i] = A1[i] + A2[i];
	}
}

void Kalman::sum_arr33(double A1[3][3], double A2[3][3], double A3[3][3])
{
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			A3[i][j] = A1[i][j] + A2[i][j];
		}
	}
}

void Kalman::sub_arr33(double A1[3][3], double A2[3][3], double A3[3][3])
{
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			A3[i][j] = A1[i][j] - A2[i][j];
		}
	}
}

void Kalman::arrCpy1(double A1[3], double A2[3])
{
	for (int i = 0; i < 3; i++)
	{
		A2[i] = A1[i];
	}
}

void Kalman::arrCpy33(double A1[3][3], double A2[3][3])
{
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			A2[i][j] = A1[i][j];
		}
	}
}

/*void Kalman::inverse(double x[3][3], double xout[3][3])
{
double determinant = 0;
for (int m = 0; m < 3; m++)
determinant = determinant + (x[0][m] * (x[1][(m + 1) % 3] * x[2][(m + 2) % 3] - x[1][(m + 2) % 3] * x[2][(m + 1) % 3]));
for (int m = 0; m < 3; m++){
for (int n = 0; n < 3; n++)
xout[m][n] = x[(m + 1) % 3][(n + 1) % 3] * (x[(m + 2) % 3][(n + 2) % 3] - (x[(m + 1) % 3][(n + 2) % 3] * x[(m + 2) % 3][(n + 1) % 3])) / determinant;
}
}*/

double Kalman::determinantOfMinor(int theRowHeightY, int theColumnWidthX, const double theMatrix[/*Y=*/3][/*X=*/3])
{
	int x1 = theColumnWidthX == 0 ? 1 : 0;  /* always either 0 or 1 */
	int x2 = theColumnWidthX == 2 ? 1 : 2;  /* always either 1 or 2 */
	int y1 = theRowHeightY == 0 ? 1 : 0;  /* always either 0 or 1 */
	int y2 = theRowHeightY == 2 ? 1 : 2;  /* always either 1 or 2 */

	return (theMatrix[y1][x1] * theMatrix[y2][x2])
		- (theMatrix[y1][x2] * theMatrix[y2][x1]);
}

//(B)Determinant is now : (Note the minus sign!)

double Kalman::determinant(const double theMatrix[/*Y=*/3][/*X=*/3])
{
	return (theMatrix[0][0] * determinantOfMinor(0, 0, theMatrix))
		- (theMatrix[0][1] * determinantOfMinor(0, 1, theMatrix))
		+ (theMatrix[0][2] * determinantOfMinor(0, 2, theMatrix));
}

//(C)And the inverse is now :

bool Kalman::inverse(double theMatrix[/*Y=*/3][/*X=*/3], double theOutput[/*Y=*/3][/*X=*/3])
{
	double det = determinant(theMatrix);

	/* Arbitrary for now.  This should be something nicer... */
	if (abs(det) < 1e-2)
	{
		//memset(theOutput, 0, sizeof theOutput);
		return false;
	}

	double oneOverDeterminant = 1.0 / det;

	for (int y = 0; y < 3; y++)
		for (int x = 0; x < 3; x++)
		{
		/* Rule is inverse = 1/det * minor of the TRANSPOSE matrix.  *
		* Note (y,x) becomes (x,y) INTENTIONALLY here!              */
		theOutput[y][x]
			= determinantOfMinor(x, y, theMatrix) * oneOverDeterminant;

		/* (y0,x1)  (y1,x0)  (y1,x2)  and (y2,x1)  all need to be negated. */
		if (1 == ((x + y) % 2))
			theOutput[y][x] = -theOutput[y][x];
		}

	return true;
}