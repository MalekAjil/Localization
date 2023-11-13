#include "stdafx.h"
#include "MyRobot.h"
#include <process.h>
#include <windows.h>
#include "afxwin.h"

#define PI 3.14159265
#define PAY 180

FILE *OutFile, *MFile,*TMFile;
std::string str = "";
int ret, i = 0, n = 0;
std::list<double> X0, Y0, X, Y, T, XX, YY, TT, XU, YU, TU, XK, YK, TK, S2, S3, GS2, GS3;
double  theta = 0, d = 0, t = 0;
double mean=800, stdev=100, var=stdev*stdev, filter=0;
/*
filter = 0.0; //minimum probability in order for coordinate to be valid (drawn)
mean = 800; //mean for the gaussian function
stdev = 200; //standard deviation for the gaussian function
var = stdev*stdev;
*/
double oldPos[3], myPos[3], robPos[3], KalmenPos[3],SonarPos[3];
double pos0[3], uPos[3], pos2[3], pos3[3], th31 = 0;
double Vel[2];

double theta1 = 0;

double Rvel, Lvel = Rvel = 100, minDist = 250;//25 cm
double r = 1;//10cm
double l;
double sonars[8][4];// 0=sX ; 1=sY ; 2=sTh ; 3=sDistance
double sonarD0[8], sonarsD1[8],sonarG[8];

// connection to the robot
ArTcpConnection con;
// the robot
ArRobot *robot;
ArKeyHandler keyHandler;
ArSonarDevice sonar;
//	ArCommands cmd;
double MinDist = 150; // Minimum distance to turn
double front, left, right; // Distance at Left Right and Front of the robot ( from sonar ).

int Wide = 150; // Number of cells of the map in the console
int Height = 150; // Idem
int res = 500; // Resolution 1000 => 1m
int ** Grille = NULL; // Grid for mapping

int ConsoleWide = 36; // Number of cells to show in console
int ConsoleHeight = 25;

int PositionMapX = Wide / 2 - ConsoleWide / 2; // Starting position in console
int PositionMapY = Height / 2 - ConsoleHeight / 2;

MyRobot::MyRobot(){ kl = new Kalman(); robot->init(); }
MyRobot::~MyRobot(){}
MyRobot::MyRobot(ArRobot *amigo)
{
	moveThread = 0; //initialize at 0 to indicate no thread is running
	robot = amigo;
	robot->init();
	kl = new Kalman();
}

double MyRobot::Gauss(int x)
{
	double A = 1 / (stdev*sqrt(2 * PI));
	double B = ((x - mean)*(x - mean)) / (2 * var);
	return 500 * A*exp(-B);
}

void MyRobot::stop()
{
	//robot = amigo;
	robot->lock();
	robot->setVel(0);
	robot->setRotVel(0);
	robot->unlock();
}

int MyRobot::connect(int ip)
{
	//robot = amigo;
	if (is_Connected())	{		return 0;	}
	else
	{
		//robot = amigo;
		OutFile = fopen("mm1.dat", "w");
		MFile = fopen("printPath.m", "w");
		TMFile = fopen("printTeta.m", "w");
		std::fprintf(OutFile, "Robot Positioning Correction System\n\n");
		std::fprintf(MFile, "function printPath()\n");
		std::fprintf(TMFile, "function printTeta()\n");
		sonars[0][2] = 90; sonars[1][2] = 44; sonars[2][2] = 12; sonars[3][2] = -12;
		sonars[4][2] = -44; sonars[5][2] = -90; sonars[6][2] = -144; sonars[7][2] = 144;
		
		Aria::init();
		Aria::setKeyHandler(&keyHandler);
		robot->attachKeyHandler(&keyHandler);
		con.setPort();
		//5- open connection between robot and program
		if (ip == 1)//if robot
		{
			if ((ret = con.open("169.254.2.115", 8101)) != 0)
			{
				Aria::shutdown();
				return 1;
			}
		}
		else	//if simulator
		{
			if ((ret = con.open("localhost", 8101)) != 0)
			{
				Aria::shutdown();
				return 1;
			}
		}
	
		robot->setDeviceConnection(&con);
		// try to connect, if we fail, the connection handler should bail
		if (!robot->blockingConnect())
		{
			// this should have been taken care of by the connection handler but just in case
			//m->Format(_T("asyncConnect failed because robot is not running in its own thread.\n"));
			Aria::shutdown();
			return 2;
		}
		// turn on the motors, turn off amigobot sounds
		robot->comInt(ArCommands::ENABLE, 1);
		robot->comInt(ArCommands::SOUNDTOG, 0);
		robot->comInt(ArCommands::SONAR, 1);
		robot->enableAutonomousDrivingSonar();
		robot->enableSonar();
		// start the robot running, true so that if we lose connection the run stops
		robot->runAsync(true);
		l = robot->getRobotRadius();
		std::fprintf(OutFile, "TcpConnect:l=%.2f\n", l);
		//std::fprintf(MFile, "TcpConnect:l=%.2f\n", l);
		kl->init();
		Sleep(5000);
	}
	return 0;
}

int MyRobot::disconnect()
{
	CompletePrint();
	//robot = amigo;
	if (is_Connected())
	{
		robot->setVel(0); // Stop the robot
		robot->setRotVel(0);
		robot->disconnect(); // Disconnect from the robot
		fclose(OutFile);
		fclose(MFile);
		fclose(TMFile);
	}
	Aria::shutdown(); // Shutdown Aria
	return 0;
}

bool MyRobot::is_Connected()
{
	return robot->isConnected(); //returns true if the robot is connected
}

void MyRobot::avoidObstacle()
{
	for (int i = 0; i < 8; i++)
	{
		sonars[i][3] = robot->getSonarRange(i);
	}
	if ((sonars[1][3] + sonars[2][3]) / 2 < minDist && (sonars[1][3] + sonars[2][3]) < (sonars[3][3] + sonars[4][3]))
	{
		robot->setVel2(Lvel, 0);
	}
	else if ((sonars[3][3] + sonars[4][3]) / 2 < minDist && (sonars[3][3] + sonars[4][3]) < (sonars[1][3] + sonars[2][3]))
	{
		robot->setVel2(0, Rvel);
	}
	else if (sonars[0][3] < minDist)
	{
		robot->setVel2(Lvel, Rvel / 2);
	}
	else if (sonars[5][3] < minDist)
	{
		robot->setVel2(Lvel / 2, Rvel);
	}
	else
	{
		robot->setVel2(Lvel, Rvel);
	}
}

bool MyRobot::checkFront()
{
	sonars[2][3] = robot->getSonarRange(2);
	sonars[3][3] = robot->getSonarRange(3);
	if (sonars[2][3] < minDist&&sonars[3][3] < MinDist)
	{
		return true;
	}
	return false;
}

int MyRobot::GoTo(double x, double y, int num)// goto specified point
{
	int n = num;	
	oldPos[0] = robot->getX();
	oldPos[1] = robot->getY();
	oldPos[2] = robot->getTh();

	std::fprintf(OutFile, "TcpConnect: Pos_%d=(%.2f,%.2f,%.2f), Trans. Vel=%.2f, Battery=%.2fV\n", n - 1, oldPos[0], oldPos[1], oldPos[2], robot->getVel(), robot->getBatteryVoltage());
	X.push_back(oldPos[0]);
	Y.push_back(oldPos[1]);
	T.push_back(oldPos[2]);
	X0.push_back(robPos[0]);
	X0.push_back(x);
	Y0.push_back(robPos[1]);
	Y0.push_back(y);
	double target = sqrt((x - robPos[0])*(x - robPos[0]) + (y - robPos[1])*(y - robPos[1]));
	// calcul of the distance to reach the goal	point
	double Angle = 0, rotVel = 15;
	// determinig the angle between robot initial direction and goal
	Angle = atan2((y - robPos[1]), (x - robPos[0]));

	if ((robPos[2] >= 0) && (Angle < robPos[2] && Angle >= robPos[2] - PI))
	{
		rotVel *= -1;
	}
	else if ((robPos[2] < 0) && (Angle < 0))
	{
		if (Angle < robPos[2] && Angle >= -PI)
		{
			rotVel *= -1;
		}
	}
	else if ((robPos[2] < 0) && (Angle > 0))
	{
		if (Angle > robPos[2] + PI &&  Angle <= PI)
		{
			rotVel *= -1;
		}
	}
	else
	{
		rotVel = abs(rotVel);
	}
	robPos[2] = Angle;
	Turn(rotVel, Angle / PI * 180);
	robot->lock();
	robot->setRotVel(0);
	robot->setVel(0);
	robot->unlock();
	Go(200, target);// go forward to the goal with a speed of 200 mm/s	

	robPos[0] = x;// save the new robot 's position
	robPos[1] = y;
	return 1;
}

int MyRobot::Go(double Speed, double Dist)// go forward at the specified distance with the specified speed
{
	fprintf(OutFile, "Go forward : %.2f  at Speed : %.2f mm/s\n", Dist, Speed);
	double Dist0 = 0;
	double dt = 100;
	bool isJammed = false;
	sonarD0[2] = robot->getSonarRange(2);
	sonarD0[3] = robot->getSonarRange(3);
	while (Dist0 < Dist)
	{
		myPos[0] = robot->getX();
		myPos[1] = robot->getY();
		myPos[2] = robot->getTh();
		Rvel = robot->getRightVel();
		Lvel = robot->getLeftVel();
		Vel[0] = robot->getVel();
		Vel[1] = robot->getRotVel();
		X.push_back(myPos[0]);
		Y.push_back(myPos[1]);
		T.push_back(myPos[2]);
		robot->lock();
		robot->enableMotors();
		if (abs(Dist - Dist0) < 300)
		{
			robot->setVel(abs(Dist - Dist0) / 300 * Speed + 5);// adjusting the speed
		}		
		else
		{
			robot->setVel(Speed);
		}
		robot->unlock();
		// current travelled distance
		Dist0 = sqrt((myPos[0] - oldPos[0]) *(myPos[0] - oldPos[0]) + (myPos[1] - oldPos[1])*(myPos[1] - oldPos[1]));
		std::fprintf(OutFile, "myX = %.2f , myY = %.2f , myTheta = %.2f \n RightVel= %.2f  LeftVel= %.2f , Vel= %.2f RotVel= %.2f \n ", myPos[0], myPos[1], myPos[2], Rvel, Lvel, Vel[0], Vel[1]);
		std::fprintf(OutFile, "dist0 = %.2f , target = %.2f \n", Dist0, Dist);
		//sonars[2][3] = robot->getSonarRange(2);
		sonars[3][3] = robot->getSonarRange(3);
		//sonarsD1[2] = (sonarD0[2] - sonars[2][3]) *cos((sonars[2][2])*PI / 180);
		sonarsD1[3] = (sonarD0[3] - sonars[3][3]) *cos((sonars[3][2])*PI / 180);
		//sonarD0[2] = sonars[2][3];
		sonarD0[3] = sonars[3][3];
		std::fprintf(OutFile, "s2=%.2f , s3=%.2f  ,,  Ds2=%.2f   Ds3=%.2f  \n", sonars[2][3], sonars[3][3], sonarsD1[2], sonarsD1[3]);
		SonarPos[0] += (sonarsD1[3]) * cos(myPos[2] * PI / 180);
		SonarPos[1] += (sonarsD1[3]) * sin(myPos[2] * PI / 180);
		SonarPos[2] = robot->getTh();
		std::fprintf(OutFile, "X_sonar=%.2f ,Y_sonar=%.2f ,theta=%.2f\n", SonarPos[0], SonarPos[1], SonarPos[2]);
		XX.push_back(SonarPos[0]);
		YY.push_back(SonarPos[1]);
		TT.push_back(SonarPos[2]);
		//S2.push_back(sonars[2][3]);
		S3.push_back(sonars[3][3]);

		kl->kalman(Vel, SonarPos, KalmenPos);
		XK.push_back(KalmenPos[0]);
		YK.push_back(KalmenPos[1]);
		TK.push_back(KalmenPos[2]);
		std::fprintf(OutFile, "Kalman : Xk =%.2f ,Yk =%.2f\n", KalmenPos[0], KalmenPos[1]);
		isJammed = checkFront();
		if (isJammed)
		{
			std::fprintf(OutFile, "****** there is an obstakle in front of robot ******\n");
			std::fprintf(MFile, "\n msg= '****** there is an obstakle in front of robot ******'\n");
			disconnect();
			break;
		}
		ArUtil::sleep(dt);
	}
	if (!isJammed)
		Correct(KalmenPos);
	robot->lock();
	robot->setVel(0);
	robot->unlock();
	return 1;
}

int MyRobot::Turn( double Speedr, double Angle)// the robot turns at the specified angle with the specified speed
{
	fprintf(OutFile, " Turn : Target: %.2f deg at Speed : %.2f deg /s\n", Angle, Speedr);
	//double Angle0 = 0;
	double Angle0 = robot->getTh();
	double dt = 100;
	t = dt / 1000;
	l = robot->getRobotRadius();
	r = 1;
	robot->enableMotors();
	while (true)
	{		
		Vel[0] = robot->getVel();
		Vel[1] = robot->getRotVel();
		robot->lock();
		//robot->setRotVel(Speedr);
		if (abs(Angle - Angle0) < 10)
		{
			robot->setRotVel(Speedr / 10);
		}
		else{ robot->setRotVel(Speedr); }
		robot->unlock();
		pos0[2] += (r*(robot->getRightVel() - robot->getLeftVel()) / (2 * l)) * t * PI / 180;
		Angle0 = robot->getTh();
		SonarPos[2] = Angle0;		
		fprintf(OutFile, " myTeta : %.2f deg at Speed : %.2f deg /s\n Theta_0 =%.2f   uTheata=%.2f\n", Angle0, robot->getRotVel(), pos0[2], uPos[2]);
		kl->kalman(Vel, SonarPos, KalmenPos);
		T.push_back(Angle0);
		TT.push_back(SonarPos[2]);
		TK.push_back(KalmenPos[2]);		
		if (abs(Angle - Angle0) < 1)
		{
			break;
		}		
		ArUtil::sleep(dt);
	}
	robot->lock();
	robot->setRotVel(0);
	robot->unlock();
	//wait until robot stop moving and sonar is stable
	ArUtil::sleep(2000);
	return 1;
}

int MyRobot::Correct(double target[3])
{
	double Angle = 0, rotVel = 15, myAngle = myPos[2];
	double dist;
	// calcul of the distance to reach the goal	point
	if ((target[0] - myPos[0]) < 2 && (target[1] - myPos[1]) > 2)
		dist = (target[1] - myPos[1]);
	else if ((target[0] - myPos[0]) > 2 && (target[1] - myPos[1]) < 2)
		dist = (target[0] - myPos[0]);
	else
	{
		// calcul of the distance to reach the goal	point
		dist = sqrt((target[0] - myPos[0])*(target[0] - myPos[0]) + (target[1] - myPos[1])*(target[1] - myPos[1]));
		// determinig the angle between robot initial direction and goal
		Angle = atan2((target[1] - myPos[1]), (target[0] - myPos[0]));
		robot->setDeltaHeading(Angle);
	}
	
	robot->lock();
	robot->move(dist);
	robot->setVel(0);
	robot->unlock();
	return 1;
}

void MyRobot::GoToWarp(double xg, double yg, int num)
{
	int n = num;
	fprintf(OutFile, "Go warp to : %.2f on X and %.2f on Y \n", xg, yg);
	oldPos[0] = robot->getX();
	oldPos[1] = robot->getY();
	oldPos[2] = robot->getTh();

	std::fprintf(OutFile, "TcpConnect: Pos_%d=(%.2f,%.2f,%.2f), Trans. Vel=%.2f, Battery=%.2fV\n", n - 1, oldPos[0], oldPos[1], oldPos[2], robot->getVel(), robot->getBatteryVoltage());
	X.push_back(oldPos[0]);
	Y.push_back(oldPos[1]);
	myPos[0] = robot->getX();
	X0.push_back(robPos[0]);
	X0.push_back(xg);
	Y0.push_back(robPos[1]);
	Y0.push_back(yg);

	double dt = 100;
	sonarD0[2] = robot->getSonarRange(2);
	sonarD0[3] = robot->getSonarRange(3);

	double k1, k2, k3, tg;
	double x0, y0, t0, ro, alpha, beta, v, w;
	k1 = 0.6; // Define coefficients k1 >0 k2 -k1 >0 k3 <0
	k2 = 50;
	k3 = -50;

	//xg = 1000; // Destination point
	//yg = -1000;
	tg = -PI / 2;
	/*During the calculus it get the new coordinates of the robot in polar and calculate the new speed and speed
		rotation according to k1, k2 and k3.
		Then it sends the command to the robot.*/
	for (;;)
	{
		ArUtil::sleep(100);
		x0 = robot->getX();
		y0 = robot->getY();
		t0 = robot->getTh()*PI / 180 + tg;

		ro = sqrt((xg - x0)*(xg - x0) + (yg - y0)*(yg - y0)); // Calculus according to robot position
		alpha = -t0 + atan2((yg - y0), (xg - x0));
		beta = -t0 - alpha;

		v = k1*ro;
		if (v > 150) // Limit speed and rotation speed
			v = 150;
		w = k2* alpha + k3* beta;
		if (w > 45)
			w = 45;
		if (w < -45)
			w = -45;
		//
		myPos[0] =x0;
		myPos[1] = y0;
		myPos[2] = t0;
		Rvel = robot->getRightVel();
		Lvel = robot->getLeftVel();
		Vel[0] = robot->getVel();
		Vel[1] = robot->getRotVel();
		X.push_back(myPos[0]);
		Y.push_back(myPos[1]);
		T.push_back(myPos[2]);

		robot->lock();
		robot->setVel(v);
		robot->setRotVel(w);
		robot->unlock();
		
		std::fprintf(OutFile, "myX = %.2f , myY = %.2f , myTheta = %.2f \n RightVel= %.2f  LeftVel= %.2f , Vel= %.2f RotVel= %.2f \n ", myPos[0], myPos[1], myPos[2], Rvel, Lvel, Vel[0], Vel[1]);		
		kl->kalman(Vel, myPos, KalmenPos);
		XK.push_back(KalmenPos[0]);
		YK.push_back(KalmenPos[1]);
		TK.push_back(KalmenPos[2]);
		std::fprintf(OutFile, "Kalman : Xk =%.2f ,Yk =%.2f\n", KalmenPos[0], KalmenPos[1]);
		//
		if (abs(robot->getX() - xg) < 2 && abs(robot->getY() - yg) < 2 && abs(robot->getTh() - tg * 180 / PI) < 2)
			// Stop the robot if it reach his destination point
		{
			break;
		}
	}
	CompletePrint();
	robPos[0] = xg;// save the new robot 's position
	robPos[1] = yg;
}

void MyRobot::CompletePrint()
{
	int k = 0, NumCol = 50;
	std::fprintf(MFile, "X0= [ ");
	for (std::list<double>::iterator itx = X0.begin(); itx != X0.end(); ++itx)
	{
		std::fprintf(MFile, "%.2f ", *itx);
		if (++k == NumCol){ std::fprintf(MFile, "...\n"); k = 0; }
	}
	k = 0;
	std::fprintf(MFile, " ];\n\nY0 = [ ");
	for (std::list<double>::iterator ity = Y0.begin(); ity != Y0.end(); ++ity)
	{
		std::fprintf(MFile, "%.2f ", *ity);
		if (++k == NumCol){ std::fprintf(MFile, "...\n"); k = 0; }
	}
	k = 0;
	std::fprintf(MFile, " ];\n\nmyX = [ ");
	for (std::list<double>::iterator itx = X.begin(); itx != X.end(); ++itx)
	{
		std::fprintf(MFile, "%.2f ", *itx);
		if (++k == NumCol){ std::fprintf(MFile, "...\n"); k = 0; }
	}
	k = 0;
	std::fprintf(MFile, " ];\n\nmyY = [ ");
	for (std::list<double>::iterator ity = Y.begin(); ity != Y.end(); ++ity)
	{
		std::fprintf(MFile, "%.2f ", *ity);
		if (++k == NumCol){ std::fprintf(MFile, "...\n"); k = 0; }
	}
	k = 0;
	std::fprintf(MFile, " ];\n\nXX = [ ");
	for (std::list<double>::iterator itx = XX.begin(); itx != XX.end(); ++itx)
	{
		std::fprintf(MFile, "%.2f ", *itx);
		if (++k == NumCol){ std::fprintf(MFile, "...\n"); k = 0; }
	}
	k = 0;
	std::fprintf(MFile, " ];\n\nYY = [ ");
	for (std::list<double>::iterator ity = YY.begin(); ity != YY.end(); ++ity)
	{
		std::fprintf(MFile, "%.2f ", *ity);
		if (++k == NumCol){ std::fprintf(MFile, "...\n"); k = 0; }
	}	
	k = 0;
	std::fprintf(MFile, " ];\n\nXK = [ ");
	for (std::list<double>::iterator itx = XK.begin(); itx != XK.end(); ++itx)
	{
		std::fprintf(MFile, "%.2f ", *itx);
		if (++k == NumCol){ std::fprintf(MFile, "...\n"); k = 0; }
	}
	k = 0;
	std::fprintf(MFile, " ];\n\nYK = [ ");
	for (std::list<double>::iterator ity = YK.begin(); ity != YK.end(); ++ity)
	{
		std::fprintf(MFile, "%.2f ", *ity);
		if (++k == NumCol){ std::fprintf(MFile, "...\n"); k = 0; }
	}
	std::fprintf(MFile, " ];\nplot(X0,Y0,'k-','LineWidth', 3);\ngrid on\n hold on\n");
	std::fprintf(MFile, "plot(myX, myY, 'b-', 'LineWidth', 2); \nplot(XK, YK, 'r-', 'LineWidth', 1.5); \nplot(XX, YY, 'g-', 'LineWidth', 2.5); \n");
	fprintf(MFile, "\ntitle('Robot Localization Correct System');\nlegend('System Model','measured','kalman Filter','real');\nhold off;\nend\n ");
	// print theta info
	
	k = 0;
	std::fprintf(TMFile, "\nMyTheta = [");
	for (std::list<double>::iterator itx = T.begin(); itx != T.end(); ++itx)
	{
		std::fprintf(TMFile, "%.2f ", *itx);
		if (++k == NumCol){ std::fprintf(TMFile, "...\n"); k = 0; }
	}
	k = 0;
	std::fprintf(TMFile, " ];\n\nSonar_Theta = [ ");
	for (std::list<double>::iterator itx = TT.begin(); itx != TT.end(); ++itx)
	{
		std::fprintf(TMFile, "%.2f ", *itx);
		if (++k == NumCol){ std::fprintf(TMFile, "...\n"); k = 0; }
	}
	k = 0;
	std::fprintf(TMFile, " ];\n\nKalman_Theta = [ ");
	for (std::list<double>::iterator itx = TK.begin(); itx != TK.end(); ++itx)
	{
		std::fprintf(TMFile, "%.2f ", *itx);
		if (++k == NumCol){ std::fprintf(TMFile, "...\n"); k = 0; }
	}
	std::fprintf(TMFile, " ];\nTime = 0:1:length(MyTheta)-1;figure;\nplot(Time,MyTheta,'b-','LineWidth', 3.5);\ngrid on\n hold on\n");
	std::fprintf(TMFile, "plot(Time, Sonar_Theta, 'g-', 'LineWidth', 2);\nplot(Time, Kalman_Theta, 'r-', 'LineWidth', 2.5); \nplot(Time, Eular_Theta, 'c-', 'LineWidth', 2.5); \nlegend('MyTheta','Sonar_Theta','kalman Filter','Eular_Theta');\nhold off\n");
	/*k = 0;
	std::fprintf(TMFile, " ];\n\nSTime = [ ");
	for (std::list<double>::iterator itx = STime.begin(); itx != STime.end(); ++itx)
	{
	std::fprintf(TMFile, "%.2f ", *itx);
	if (++k == NumCol){ std::fprintf(TMFile, "...\n"); k = 0; }
	}*/
	k = 0;
	std::fprintf(TMFile, "\n\nSonar_2 = [ ");
	for (std::list<double>::iterator itx = S2.begin(); itx != S2.end(); ++itx)
	{
		std::fprintf(TMFile, "%.2f ", *itx);
		if (++k == NumCol){ std::fprintf(TMFile, "...\n"); k = 0; }
	}
	k = 0;
	std::fprintf(TMFile, " ];\n\nSonar_3 = [ ");
	for (std::list<double>::iterator itx = S3.begin(); itx != S3.end(); ++itx)
	{
		std::fprintf(TMFile, "%.2f ", *itx);
		if (++k == NumCol){ std::fprintf(TMFile, "...\n"); k = 0; }
	}
	
	std::fprintf(TMFile, " ];\nSTime = 0:1:length(Sonar_3)-1;\nfigure;\ngrid on\n hold on\nplot(STime,Sonar_3,'g-', 'LineWidth', 2.5);\n hold off;");
	fprintf(TMFile, "\ntitle('Robot Localization Correct System'); \nend\n");

	fcloseall();
}
