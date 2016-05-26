/*
 * SwerveModule.h
 *
 *  Created on: May 26, 2016
 *      Author: taylo
 */

#ifndef SRC_SWERVEMODULE_H_
#define SRC_SWERVEMODULE_H_

#include "WPILib.h"
#include "Logger.h"

class SwerveModule
{
public:
	enum Position { kFR, kFL, kBR, kBL };

	SwerveModule(CANTalon drive, CANTalon angle, AnalogInput encoder, Position position);
	void Swerve(double x, double y, double z, double gyro);
	void Log(Logger logger);
	void SetupLogging(Logger logger);
	void ReloadConfig();
private:
	CANTalon drive;
	CANTalon angle;
	AnalogInput e;
	Position p;
	double a,b,c,d;
	double ws,wa;
	double max;
};



#endif /* SRC_SWERVEMODULE_H_ */
