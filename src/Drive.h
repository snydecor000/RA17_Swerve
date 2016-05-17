/*
 * Drive.h
 *
 *  Created on: Jan 22, 2016
 *      Author: Taylor Horne
 */

#ifndef SRC_DRIVE_H_
#define SRC_DRIVE_H_
#include "WPILib.h"
#include "Logger.h"

class Drive
{
public:
	Drive(CANTalon * fr, CANTalon * br, CANTalon * fl, CANTalon * bl,
		CANTalon * fra, CANTalon * bra, CANTalon * fla, CANTalon * bla,
		AnalogInput * fre, AnalogInput * bre, AnalogInput * fle, AnalogInput * ble);
	~Drive();
	void Swerve(double x, double y, double z, double gyro);
	void Log(Logger * logger);
	void SetupLogging(Logger * logger);
	void ReloadConfig();

private:
	CANTalon * FL;
	CANTalon * BL;
	CANTalon * FR;
	CANTalon * BR;
	CANTalon * FLa;
	CANTalon * BLa;
	CANTalon * FRa;
	CANTalon * BRa;
	AnalogInput * FLe;
	AnalogInput * BLe;
	AnalogInput * FRe;
	AnalogInput * BRe;
	PIDController * FLc;
	PIDController * BLc;
	PIDController * FRc;
	PIDController * BRc;

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

#endif /* SRC_DRIVE_H_ */
