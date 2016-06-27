/*
 * Drive.cpp
 *
 *  Created on: Jan 22, 2016
 *      Author: Taylor Horne
 */

#include <SwerveDrive.h>
#include "WPILib.h"
#include "Logger.h"
#include "Config.h"
#include "PIDController.h"
#include <cmath>
#include <iostream>
using namespace std;

const double PI = 3.14159265358979;

Drive::Drive(CANTalon * fr,CANTalon * fra, AnalogInput * fre)
{
	SpeedP = Config::GetSetting("speedP",1);
	SpeedI = Config::GetSetting("speedI",0);
	SpeedD = Config::GetSetting("speedD",0);
	SteerP = Config::GetSetting("steerP",1);
	SteerI = Config::GetSetting("steerI",.01);
	SteerD = Config::GetSetting("steerD",0);
	SteerTolerance = Config::GetSetting("Steering%Tolerance", 0.5);
	SteerSpeed = Config::GetSetting("SteerSpeed", 1);
	SteerEncMax = Config::GetSetting("SteerEncMax",4.78);
	TurningSpeedFactor = Config::GetSetting("turningSpeedFactor", 1);
	DriveCIMMaxRPM = Config::GetSetting("driveCIMmaxRPM",4000);

	FR = fr;

	FRa = fra;
	FRa->SetControlMode(CANTalon::kPercentVbus);

	FRe = fre;

	FRc = new PIDController(SteerP,SteerI,SteerD,FRe,FRa);
	FRc->Disable();
	FRc->SetPIDSourceType(PIDSourceType::kDisplacement);
	FRc->SetContinuous(true);
	FRc->SetInputRange(0,SteerEncMax);
	FRc->SetOutputRange(-SteerSpeed,SteerSpeed);
	FRc->SetTolerance(SteerTolerance);
	FRc->SetSetpoint(0);

	FRc->Enable();

	length = Config::GetSetting("FrameLength",1);
	width = Config::GetSetting("FrameWidth",1);
	diameter = sqrt(pow(length,2)+pow(width,2));
	temp = 0.0;
	a = 0.0,b = 0.0,c = 0.0,d = 0.0;
	ws1 = 0.0,ws2 = 0.0,ws3 = 0.0,ws4 = 0.0;
	wa1 = 0.0,wa2 = 0.0,wa3 = 0.0,wa4 = 0.0;
	max = 0.0;
}

void Drive::Swerve(double x, double y, double z, double gyro)
{
	gyro *= PI/180.0f;
	z *= TurningSpeedFactor;

	temp = y * cos(gyro) + x * sin(gyro);
	x = -y * sin(gyro) + x * cos(gyro);
	y = temp;

	a = x - z * (length/diameter);
	b = x + z * (length/diameter);
	c = y - z * (width/diameter);
	d = y + z * (width/diameter);

	ws1 = sqrt(pow(b,2) + pow(c,2));
	ws2 = sqrt(pow(b,2) + pow(d,2));
	ws3 = sqrt(pow(a,2) + pow(d,2));
	ws4 = sqrt(pow(a,2) + pow(c,2));
	if(ws1 > max){max = ws1;}
	if(ws2 > max){max = ws2;}
	if(ws3 > max){max = ws3;}
	if(ws4 > max){max = ws4;}
	if(max > 1){ws1 /= max;ws2 /= max;ws3 /= max;ws4 /= max;}
	FR->Set(ws1);

	wa1 = atan2(b,c) * 180.0f/PI;
	wa2 = atan2(b,d) * 180.0f/PI;
	wa3 = atan2(a,d) * 180.0f/PI;
	wa4 = atan2(a,c) * 180.0f/PI;
	if(wa1 < 0){wa1 += 360;}
	if(wa2 < 0){wa2 += 360;}
	if(wa3 < 0){wa3 += 360;}
	if(wa4 < 0){wa4 += 360;}
	FRc->SetSetpoint(wa1*(SteerEncMax/360.0f));
	cout << "Angle Setpoint: " << (wa1*(SteerEncMax/360.0f)) << endl;
	cout << "Angle Actual: " << FRe->GetVoltage() << endl;
}

void Drive::SetupLogging(Logger * logger)
{
//	logger->AddAttribute("Time");
//	logger->AddAttribute("FRpos");
//	logger->AddAttribute("FRCurrent");
//	logger->AddAttribute("FRspeed");
//	logger->AddAttribute("FRApos");
//	logger->AddAttribute("FRACurrent");
//	logger->AddAttribute("FREncpos");
//	logger->AddAttribute("FREncSetpoint");
}

void Drive::Log(Logger* logger)
{
//	logger->Log("FRpos", FR->GetEncPosition());
//	logger->Log("FRCurrent", FR->GetOutputCurrent());
//	logger->Log("FRspeed", FR->GetSpeed());
//	logger->Log("FRApos", FRa->GetEncPosition());
//	logger->Log("FRACurrent", FRa->GetOutputCurrent());
//	logger->Log("FREncpos", FRe->GetVoltage());
//	logger->Log("FREncSetpoint", wa1*(SteerEncMax/360.0f));
//	logger->WriteLine();
}

void Drive::ReloadConfig()
{
	SpeedP = Config::GetSetting("speedP",1);
	SpeedI = Config::GetSetting("speedI",0);
	SpeedD = Config::GetSetting("speedD",0);
	FR->SetPID(SpeedP,SpeedI,SpeedD);
/////////////////////////////////////////////////////
	SteerP = Config::GetSetting("steerP",1);
	SteerI = Config::GetSetting("steerI",0);
	SteerD = Config::GetSetting("steerD",0);
	FRc->SetPID(SteerP,SteerI,SteerD);
///////////////////////////////////////////////////////////////////
	SteerTolerance = Config::GetSetting("Steering%Tolerance", 3);
	SteerSpeed = Config::GetSetting("SteerSpeed", .9);
	SteerEncMax = Config::GetSetting("SteerEncMax",5);

	FRc->SetInputRange(0,SteerEncMax);
	FRc->SetOutputRange(-SteerSpeed,SteerSpeed);
	FRc->SetTolerance(SteerTolerance);
/////////////////////////////////////////////////////
	TurningSpeedFactor = Config::GetSetting("turningSpeedFactor", 1);
	DriveCIMMaxRPM = Config::GetSetting("driveCIMmaxRPM",4000);
}
