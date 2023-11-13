#include "Kalman.h"
#include <stdio.h>
#include <iostream>
using namespace std;

int main(int *argc, char* argv)
{
	Kalman kl;
	double X[3], Y[3], XK[3], V[3];
	X[0] = 0; X[1] = -10; X[2] = -90;
	Y[0] = 0; Y[1] = -11; Y[2] = -90.4;
	V[0] = 100; V[1] = 0;
	kl.init();
	kl.kalman(V, X, Y, XK);
	printf("%.2f   %.2f   %.2f ", XK[0], XK[1], XK[2]);
	X[0] = 0; X[1] = -20; X[2] = -90;
	Y[0] = 0; Y[1] = -22; Y[2] = -90.4;
	kl.kalman(V, X, Y, XK);
	printf("%.2f   %.2f   %.2f ", XK[0], XK[1], XK[2]);
	std::system("pause");
	return 0;
}