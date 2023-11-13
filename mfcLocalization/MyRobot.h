#pragma once
#include "Aria.h"
#include <iostream>
#include <fstream>
#include <list>
#include "Kalman.h"

class MyRobot
{
public:
	MyRobot();
	MyRobot(ArRobot *amigo); //constructor
	virtual ~MyRobot(); //deconstructor
	
	double Gauss(int x); //returns a value between 0 and 1 depending on
	//the values x, mean, and stdev
	int connect(int ip); //connects to the robot	
	int disconnect(); //disconnects from the robot
	void stop(); //Stops the robot from moving and turning
	bool is_Connected(); //returns true if the robot is currently connected

	void avoidObstacle();
	bool checkFront();
	
	int GoTo(double x, double y, int num);
	int Go(double Speed, double distance);// go forward at the specified distance with the specified speed
	int Turn(double Speedr, double Angle);// the robot turns at the specified angle with the specified speed
	int Correct(double target[3]);

	void GoToWarp(double xg, double yg,int num);
	
	void CompletePrint();

	CWinThread* pThread;
	int moveThread;
private:
	Kalman *kl;
};

