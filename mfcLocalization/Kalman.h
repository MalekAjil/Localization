#pragma once
class Kalman
{
public:
	Kalman();
	~Kalman();
	void init();
	void kalman(double w[2], double Z[3], double x_new[3]);
	void transform33(double Arr1[3][3], double Arr2[3][3]);
	void transform32(double Arr1[3][2], double Arr2[2][3]);
	void transform22(double Arr1[2][2], double Arr2[2][2]);
	void mul_arr22(double m1[2][2], double m2[2][2], double m3[2][2]);
	void mul_arr23(double m1[][2], double m2[2][3], double m3[3][3]);
	void mul_arr33(double m1[3][3], double m2[3][3], double m3[3][3]);
	void mul_arr32(double m1[3][2], double m2[2], double m3[3]);
	void mul_arr31(double m1[3][3], double m2[3], double m3[3]);
	void sub_arr3(double A1[3], double A2[3], double A3[3]);
	void sum_arr3(double A1[3], double A2[3], double A3[3]);
	void sum_arr33(double A1[3][3], double A2[3][3], double A3[3][3]);
	void sub_arr33(double A1[3][3], double A2[3][3], double A3[3][3]);
	void arrCpy1(double A1[3], double A2[3]);
	void arrCpy33(double A1[3][3], double A2[3][3]);
	//void inverse(double x[3][3], double xout[3][3]);
	double determinantOfMinor(int theRowHeightY, int theColumnWidthX, const double theMatrix[/*Y=*/3][/*X=*/3]);
	double determinant(const double theMatrix[/*Y=*/3][/*X=*/3]);
	bool inverse(double theMatrix[/*Y=*/3][/*X=*/3], double theOutput[/*Y=*/3][/*X=*/3]);

private:
	double Z[3], w[2], Xhat[3], Xhat_[3], Jh[3][3], H[3][3], P[3][3], P_[3][3], F[3][3], W[3][3], V[3][3], K[3][3], R[3][3]/*, Q[3][3]*/;
	double  Fu[3][2], FuT[2][3], u1[3][2], Q[2][2];
	double X1[3], X2[3];
	double p1[3][3], p2[3][3], p3[3][3], w1[3][3], w2[3][3];
	double FT[3][3], JhT[3][3], WT[3][3], VT[3][3];
	double eye3[3][3], zeros33[3][3];
	//int k = 0;
	double dt = 0.100;
	double rx,ry,rt, qx,qy;

};

