/*
 * Drive.cpp
 *
 *  Created on: Jan 22, 2016
 *      Author: Taylor Horne
 */

#include "Drive.h"
#include "WPILib.h"
#include "Logger.h"
#include "Config.h"
#include "PIDController.h"
#include <cmath>

const double PI = 3.14159265358979;

Drive::Drive(CANTalon * fr, CANTalon * br, CANTalon * fl, CANTalon * bl,
		CANTalon * fra, CANTalon * bra, CANTalon * fla, CANTalon * bla,
		AnalogInput * fre, AnalogInput * bre, AnalogInput * fle, AnalogInput * ble)
{
	SpeedP = Config::GetSetting("speedP",1);
	SpeedI = Config::GetSetting("speedI",0);
	SpeedD = Config::GetSetting("speedD",0);
	SteerP = Config::GetSetting("steerP",1);
	SteerI = Config::GetSetting("steerI",0);
	SteerD = Config::GetSetting("steerD",0);
	SteerTolerance = Config::GetSetting("Steering%Tolerance", 3);
	SteerSpeed = Config::GetSetting("SteerSpeed", .9);
	SteerEncMax = Config::GetSetting("SteerEncMax",5);
	TurningSpeedFactor = Config::GetSetting("turningSpeedFactor", 1);
	DriveCIMMaxRPM = Config::GetSetting("driveCIMmaxRPM",4000);

	FR = fr;
	FR->SetControlMode(CANTalon::kSpeed);
	FR->ConfigEncoderCodesPerRev(80);
	FR->SetFeedbackDevice(CANTalon::QuadEncoder);
	//FR->SetCloseLoopRampRate()
	FR->SetPID(SpeedP,SpeedI,SpeedD);
	FR->EnableControl();

	BR = br;
	BR->SetControlMode(CANTalon::kSpeed);
	BR->ConfigEncoderCodesPerRev(80);
	BR->SetFeedbackDevice(CANTalon::QuadEncoder);
	//BR->SetCloseLoopRampRate()
	BR->SetPID(SpeedP,SpeedI,SpeedD);
	BR->EnableControl();

	FL = fl;
	FL->SetControlMode(CANTalon::kSpeed);
	FL->ConfigEncoderCodesPerRev(80);
	FL->SetFeedbackDevice(CANTalon::QuadEncoder);
	//FL->SetCloseLoopRampRate()
	FL->SetPID(SpeedP,SpeedI,SpeedD);
	FL->EnableControl();

	BL = bl;
	BL->SetControlMode(CANTalon::kSpeed);
	BL->ConfigEncoderCodesPerRev(80);
	BL->SetFeedbackDevice(CANTalon::QuadEncoder);
	//BL->SetCloseLoopRampRate()
	BL->SetPID(SpeedP,SpeedI,SpeedD);
	BL->EnableControl();

	FRa = fra;
	FRa->SetControlMode(CANTalon::kPercentVbus);
	BRa = bra;
	BRa->SetControlMode(CANTalon::kPercentVbus);
	FLa = fla;
	FLa->SetControlMode(CANTalon::kPercentVbus);
	BLa = bla;
	BLa->SetControlMode(CANTalon::kPercentVbus);

	FRe = fre;
	BRe = bre;
	FLe = fle;
	BLe = ble;

	FLc = new PIDController(SteerP,SteerI,SteerD,FLe,FLa,0.05);
	FLc->Disable();
	FLc->SetPIDSourceType(PIDSourceType::kDisplacement);
	FLc->SetContinuous(true);
	FLc->SetInputRange(0,SteerEncMax);
	FLc->SetOutputRange(-SteerSpeed,SteerSpeed);
	FLc->SetTolerance(SteerTolerance);
	FLc->SetSetpoint(0);
	FLc->Enable();

	FRc = new PIDController(SteerP,SteerI,SteerD,FRe,FRa);
	FRc->Disable();
	FRc->SetPIDSourceType(PIDSourceType::kDisplacement);
	FRc->SetContinuous(true);
	FRc->SetInputRange(0,SteerEncMax);
	FRc->SetOutputRange(-SteerSpeed,SteerSpeed);
	FRc->SetTolerance(SteerTolerance);
	FRc->SetSetpoint(0);
	FRc->Enable();

	BRc = new PIDController(SteerP,SteerI,SteerD,BRe,BRa);
	BRc->Disable();
	BRc->SetPIDSourceType(PIDSourceType::kDisplacement);
	BRc->SetContinuous(true);
	BRc->SetInputRange(0,SteerEncMax);
	BRc->SetOutputRange(-SteerSpeed,SteerSpeed);
	BRc->SetTolerance(SteerTolerance);
	BRc->SetSetpoint(0);
	BRc->Enable();

	BLc = new PIDController(SteerP,SteerI,SteerD,BLe,BLa);
	BLc->Disable();
	BLc->SetPIDSourceType(PIDSourceType::kDisplacement);
	BLc->SetContinuous(true);
	BLc->SetInputRange(0,SteerEncMax);
	BLc->SetOutputRange(-SteerSpeed,SteerSpeed);
	BLc->SetTolerance(SteerTolerance);
	BLc->SetSetpoint(0);
	BLc->Enable();

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
	FR->Set(ws1 * DriveCIMMaxRPM);
	FL->Set(ws2 * DriveCIMMaxRPM);
	BL->Set(ws3 * DriveCIMMaxRPM);
	BR->Set(ws4 * DriveCIMMaxRPM);

	wa1 = atan2(b,c) * 180.0f/PI;
	wa2 = atan2(b,d) * 180.0f/PI;
	wa3 = atan2(a,d) * 180.0f/PI;
	wa4 = atan2(a,c) * 180.0f/PI;
	if(wa1 < 0){wa1 += 360;}
	if(wa2 < 0){wa2 += 360;}
	if(wa3 < 0){wa3 += 360;}
	if(wa4 < 0){wa4 += 360;}
	FRc->SetSetpoint(wa1*(SteerEncMax/360.0f));
	FLc->SetSetpoint(wa2*(SteerEncMax/360.0f));
	BLc->SetSetpoint(wa3*(SteerEncMax/360.0f));
	BRc->SetSetpoint(wa4*(SteerEncMax/360.0f));
}

void Drive::SetupLogging(Logger * logger)
{
	logger->AddAttribute("Time");
	logger->AddAttribute("FLpos");
	logger->AddAttribute("FRpos");
	logger->AddAttribute("BLpos");
	logger->AddAttribute("BRpos");
	logger->AddAttribute("FLCurrent");
	logger->AddAttribute("FRCurrent");
	logger->AddAttribute("BLCurrent");
	logger->AddAttribute("BRCurrent");
	logger->AddAttribute("FLspeed");
	logger->AddAttribute("FRspeed");
	logger->AddAttribute("BLspeed");
	logger->AddAttribute("BRspeed");
	logger->AddAttribute("FLApos");
	logger->AddAttribute("FRApos");
	logger->AddAttribute("BLApos");
	logger->AddAttribute("BRApos");
	logger->AddAttribute("FLACurrent");
	logger->AddAttribute("FRACurrent");
	logger->AddAttribute("BLACurrent");
	logger->AddAttribute("BRACurrent");
	logger->AddAttribute("FLEncpos");
	logger->AddAttribute("FREncpos");
	logger->AddAttribute("BLEncpos");
	logger->AddAttribute("BREncpos");
	logger->AddAttribute("FLEncSetpoint");
	logger->AddAttribute("FREncSetpoint");
	logger->AddAttribute("BLEncSetpoint");
	logger->AddAttribute("BREncSetpoint");
}

void Drive::Log(Logger* logger)
{
	logger->Log("FLpos", FL->GetEncPosition());
	logger->Log("FRpos", FR->GetEncPosition());
	logger->Log("BLpos", BL->GetEncPosition());
	logger->Log("BRpos", BR->GetEncPosition());
	logger->Log("FLCurrent", FL->GetOutputCurrent());
	logger->Log("FRCurrent", FR->GetOutputCurrent());
	logger->Log("BLCurrent", BL->GetOutputCurrent());
	logger->Log("BRCurrent", BR->GetOutputCurrent());
	logger->Log("FLspeed", FL->GetSpeed());
	logger->Log("FRspeed", FR->GetSpeed());
	logger->Log("BLspeed", BL->GetSpeed());
	logger->Log("BRspeed", BR->GetSpeed());
	logger->Log("FLApos", FLa->GetEncPosition());
	logger->Log("FRApos", FRa->GetEncPosition());
	logger->Log("BLApos", BLa->GetEncPosition());
	logger->Log("BRApos", BRa->GetEncPosition());
	logger->Log("FLACurrent", FLa->GetOutputCurrent());
	logger->Log("FRACurrent", FRa->GetOutputCurrent());
	logger->Log("BLACurrent", BLa->GetOutputCurrent());
	logger->Log("BRACurrent", BRa->GetOutputCurrent());
	logger->Log("FLEncpos", FLe->GetVoltage());
	logger->Log("FREncpos", FRe->GetVoltage());
	logger->Log("BLEncpos", BLe->GetVoltage());
	logger->Log("BREncpos", BRe->GetVoltage());
	logger->Log("FLEncSetpoint", wa2*(SteerEncMax/360.0f));
	logger->Log("FREncSetpoint", wa1*(SteerEncMax/360.0f));
	logger->Log("BLEncSetpoint", wa3*(SteerEncMax/360.0f));
	logger->Log("BREncSetpoint", wa4*(SteerEncMax/360.0f));
	logger->WriteLine();
}

void Drive::ReloadConfig()
{
	SpeedP = Config::GetSetting("speedP",1);
	SpeedI = Config::GetSetting("speedI",0);
	SpeedD = Config::GetSetting("speedD",0);
	FR->SetPID(SpeedP,SpeedI,SpeedD);
	BR->SetPID(SpeedP,SpeedI,SpeedD);
	FL->SetPID(SpeedP,SpeedI,SpeedD);
	BL->SetPID(SpeedP,SpeedI,SpeedD);
/////////////////////////////////////////////////////
	SteerP = Config::GetSetting("steerP",1);
	SteerI = Config::GetSetting("steerI",0);
	SteerD = Config::GetSetting("steerD",0);
	FLc->SetPID(SteerP,SteerI,SteerD);
	FRc->SetPID(SteerP,SteerI,SteerD);
	BLc->SetPID(SteerP,SteerI,SteerD);
	BRc->SetPID(SteerP,SteerI,SteerD);
///////////////////////////////////////////////////////////////////
	SteerTolerance = Config::GetSetting("Steering%Tolerance", 3);
	SteerSpeed = Config::GetSetting("SteerSpeed", .9);
	SteerEncMax = Config::GetSetting("SteerEncMax",5);

	FLc->SetInputRange(0,SteerEncMax);
	FLc->SetOutputRange(-SteerSpeed,SteerSpeed);
	FLc->SetTolerance(SteerTolerance);

	FRc->SetInputRange(0,SteerEncMax);
	FRc->SetOutputRange(-SteerSpeed,SteerSpeed);
	FRc->SetTolerance(SteerTolerance);

	BRc->SetInputRange(0,SteerEncMax);
	BRc->SetOutputRange(-SteerSpeed,SteerSpeed);
	BRc->SetTolerance(SteerTolerance);

	BLc->SetInputRange(0,SteerEncMax);
	BLc->SetOutputRange(-SteerSpeed,SteerSpeed);
	BLc->SetTolerance(SteerTolerance);
/////////////////////////////////////////////////////
	TurningSpeedFactor = Config::GetSetting("turningSpeedFactor", 1);
	DriveCIMMaxRPM = Config::GetSetting("driveCIMmaxRPM",4000);
}
