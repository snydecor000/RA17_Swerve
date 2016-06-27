/*
 * Drive.h
 *
 *  Created on: Jan 22, 2016
 *      Author: Taylor Horne
 */

#ifndef SRC_SWERVEDRIVE_H_
#define SRC_SWERVEDRIVE_H_
#include "WPILib.h"
#include "Logger.h"

class Drive
{
public:
	Drive(CANTalon * fr,CANTalon * fra, AnalogInput * fre);
	~Drive();
	void Swerve(double x, double y, double z, double gyro);
	void Log(Logger * logger);
	void SetupLogging(Logger * logger);
	void ReloadConfig();

private:
	CANTalon * FR;
	CANTalon * FRa;
	AnalogInput * FRe;
	PIDController * FRc;

	double SpeedP,SpeedI,SpeedD;
	double SteerP,SteerI,SteerD;
	double SteerSpeed,SteerTolerance,SteerEncMax,TurningSpeedFactor,DriveCIMMaxRPM;

	double length,width,diameter;
	double temp;
	double a,b,c,d;
	double ws1,ws2,ws3,ws4;
	double wa1,wa2,wa3,wa4;
	double max;

};

#endif /* SRC_SWERVEDRIVE_H_ */
