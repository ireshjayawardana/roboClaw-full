/*
 * roboClaw.h
 *
 *  Created on: Sep 19, 2018
 *      Author: Sadeesh
 */

#ifndef ROBOCLAW_ROBOCLAW_H_
#define ROBOCLAW_ROBOCLAW_H_

#include "../Serial/Serial.h"
#include <stdarg.h>

typedef enum { false, true } bool;

typedef enum
{
  ROBOCLAW_OK       = 0x00U,
  ROBOCLAW_ERROR    = 0x01U,
  ROBOCLAW_BUSY     = 0x02U,
  ROBOCLAW_TIMEOUT  = 0x03U
}ROBOCLAW_StatusTypeDef;

enum {M1FORWARD = 0,
			M1BACKWARD = 1,
			SETMINMB = 2,
			SETMAXMB = 3,
			M2FORWARD = 4,
			M2BACKWARD = 5,
			M17BIT = 6,
			M27BIT = 7,
			MIXEDFORWARD = 8,
			MIXEDBACKWARD = 9,
			MIXEDRIGHT = 10,
			MIXEDLEFT = 11,
			MIXEDFB = 12,
			MIXEDLR = 13,
			GETM1ENC = 16,
			GETM2ENC = 17,
			GETM1SPEED = 18,
			GETM2SPEED = 19,
			RESETENC = 20,
			GETVERSION = 21,
			SETM1ENCCOUNT = 22,
			SETM2ENCCOUNT = 23,
			GETMBATT = 24,
			GETLBATT = 25,
			SETMINLB = 26,
			SETMAXLB = 27,
			SETM1PID = 28,
			SETM2PID = 29,
			GETM1ISPEED = 30,
			GETM2ISPEED = 31,
			M1DUTY = 32,
			M2DUTY = 33,
			MIXEDDUTY = 34,
			M1SPEED = 35,
			M2SPEED = 36,
			MIXEDSPEED = 37,
			M1SPEEDACCEL = 38,
			M2SPEEDACCEL = 39,
			MIXEDSPEEDACCEL = 40,
			M1SPEEDDIST = 41,
			M2SPEEDDIST = 42,
			MIXEDSPEEDDIST = 43,
			M1SPEEDACCELDIST = 44,
			M2SPEEDACCELDIST = 45,
			MIXEDSPEEDACCELDIST = 46,
			GETBUFFERS = 47,
			GETPWMS = 48,
			GETCURRENTS = 49,
			MIXEDSPEED2ACCEL = 50,
			MIXEDSPEED2ACCELDIST = 51,
			M1DUTYACCEL = 52,
			M2DUTYACCEL = 53,
			MIXEDDUTYACCEL = 54,
			READM1PID = 55,
			READM2PID = 56,
			SETMAINVOLTAGES = 57,
			SETLOGICVOLTAGES = 58,
			GETMINMAXMAINVOLTAGES = 59,
			GETMINMAXLOGICVOLTAGES = 60,
			SETM1POSPID = 61,
			SETM2POSPID = 62,
			READM1POSPID = 63,
			READM2POSPID = 64,
			M1SPEEDACCELDECCELPOS = 65,
			M2SPEEDACCELDECCELPOS = 66,
			MIXEDSPEEDACCELDECCELPOS = 67,
			SETM1DEFAULTACCEL = 68,
			SETM2DEFAULTACCEL = 69,
			SETPINFUNCTIONS = 74,
			GETPINFUNCTIONS = 75,
			SETDEADBAND	= 76,
			GETDEADBAND	= 77,
			GETENCODERS = 78,
			GETISPEEDS = 79,
			RESTOREDEFAULTS = 80,
			GETTEMP = 82,
			GETTEMP2 = 83,	//Only valid on some models
			GETERROR = 90,
			GETENCODERMODE = 91,
			SETM1ENCODERMODE = 92,
			SETM2ENCODERMODE = 93,
			WRITENVM = 94,
			READNVM = 95,	//Reloads values from Flash into Ram
			SETCONFIG = 98,
			GETCONFIG = 99,
			SETM1MAXCURRENT = 133,
			SETM2MAXCURRENT = 134,
			GETM1MAXCURRENT = 135,
			GETM2MAXCURRENT = 136,
			SETPWMMODE = 148,
			GETPWMMODE = 149,
			FLAGBOOTLOADER = 255};	//Only available via USB communications

typedef struct {
	uint16_t crc;
	uint32_t timeout;
	SERIAL_HandleTypeDef *hserial;
	uint8_t packetserial_address;

}RoboClaw_HandleTypeDef;


RoboClaw_HandleTypeDef hroboclaw_mc1;
//RoboClaw_StatusTypeDef roboClaw_init(RoboClaw_HandleTypeDef* hroboClaw);
//RoboClaw_StatusTypeDef ForwardM1(RoboClaw_HandleTypeDef* hroboClaw, uint8_t speed);
//RoboClaw_StatusTypeDef BackwardM1(RoboClaw_HandleTypeDef* hroboClaw, uint8_t speed);

ROBOCLAW_StatusTypeDef roboClaw_init(RoboClaw_HandleTypeDef* hroboClaw);


//write n functions//


ROBOCLAW_StatusTypeDef ForwardM1(RoboClaw_HandleTypeDef* hroboClaw, uint8_t speed);
ROBOCLAW_StatusTypeDef BackwardM1(RoboClaw_HandleTypeDef* hroboClaw, uint8_t speed);
ROBOCLAW_StatusTypeDef SetMinVoltageMainBattery(RoboClaw_HandleTypeDef* hroboClaw, uint8_t voltage);
ROBOCLAW_StatusTypeDef SetMaxVoltageMainBattery(RoboClaw_HandleTypeDef* hroboClaw, uint8_t voltage);
ROBOCLAW_StatusTypeDef ForwardM2(RoboClaw_HandleTypeDef* hroboClaw, uint8_t speed);
ROBOCLAW_StatusTypeDef BackwardM2(RoboClaw_HandleTypeDef* hroboClaw, uint8_t speed);
ROBOCLAW_StatusTypeDef ForwardBackwardM1(RoboClaw_HandleTypeDef* hroboClaw, uint8_t speed);
ROBOCLAW_StatusTypeDef ForwardBackwardM2(RoboClaw_HandleTypeDef* hroboClaw, uint8_t speed);
ROBOCLAW_StatusTypeDef ForwardMixed(RoboClaw_HandleTypeDef* hroboClaw, uint8_t speed);
ROBOCLAW_StatusTypeDef BackwardMixed(RoboClaw_HandleTypeDef* hroboClaw, uint8_t speed);
ROBOCLAW_StatusTypeDef TurnRightMixed(RoboClaw_HandleTypeDef* hroboClaw, uint8_t speed);
ROBOCLAW_StatusTypeDef TurnLeftMixed(RoboClaw_HandleTypeDef* hroboClaw, uint8_t speed);
ROBOCLAW_StatusTypeDef ForwardBackwardMixed(RoboClaw_HandleTypeDef* hroboClaw, uint8_t speed);
ROBOCLAW_StatusTypeDef LeftRightMixed(RoboClaw_HandleTypeDef* hroboClaw, uint8_t speed);
ROBOCLAW_StatusTypeDef SetM1VelocityPID(RoboClaw_HandleTypeDef* hroboClaw, float kp_fp, float ki_fp, float kd_fp, uint32_t qpps);
ROBOCLAW_StatusTypeDef SetM2VelocityPID(RoboClaw_HandleTypeDef* hroboClaw, float kp_fp, float ki_fp, float kd_fp, uint32_t qpps);
ROBOCLAW_StatusTypeDef ResetEncoders(RoboClaw_HandleTypeDef* hroboClaw);
ROBOCLAW_StatusTypeDef SetEncM1(RoboClaw_HandleTypeDef* hroboClaw, int32_t val);
ROBOCLAW_StatusTypeDef SetEncM2(RoboClaw_HandleTypeDef* hroboClaw, int32_t val);
ROBOCLAW_StatusTypeDef SetMinVoltageLogicBattery(RoboClaw_HandleTypeDef* hroboClaw, uint8_t voltage);
ROBOCLAW_StatusTypeDef SetMaxVoltageLogicBattery(RoboClaw_HandleTypeDef* hroboClaw, uint8_t voltage);
ROBOCLAW_StatusTypeDef DutyM1(RoboClaw_HandleTypeDef* hroboClaw, uint16_t duty);
ROBOCLAW_StatusTypeDef DutyM2(RoboClaw_HandleTypeDef* hroboClaw, uint16_t duty);
ROBOCLAW_StatusTypeDef DutyM1M2(RoboClaw_HandleTypeDef* hroboClaw, uint16_t duty1, uint16_t duty2);
ROBOCLAW_StatusTypeDef SpeedM1(RoboClaw_HandleTypeDef* hroboClaw, int32_t speed);
ROBOCLAW_StatusTypeDef SpeedM2(RoboClaw_HandleTypeDef* hroboClaw, int32_t speed);
ROBOCLAW_StatusTypeDef SpeedM1M2(RoboClaw_HandleTypeDef* hroboClaw, uint32_t speed1, uint32_t speed2);
ROBOCLAW_StatusTypeDef SpeedAccelM1(RoboClaw_HandleTypeDef* hroboClaw, uint32_t accel, uint32_t speed);
ROBOCLAW_StatusTypeDef SpeedAccelM2(RoboClaw_HandleTypeDef* hroboClaw, uint32_t accel, uint32_t speed);
ROBOCLAW_StatusTypeDef SpeedAccelM1M2(RoboClaw_HandleTypeDef* hroboClaw, uint32_t accel, uint32_t speed1, uint32_t speed2);
ROBOCLAW_StatusTypeDef SpeedDistanceM1(RoboClaw_HandleTypeDef* hroboClaw, uint32_t speed, uint32_t distance, uint8_t flag);
ROBOCLAW_StatusTypeDef SpeedDistanceM2(RoboClaw_HandleTypeDef* hroboClaw, uint32_t speed, uint32_t distance, uint8_t flag);
ROBOCLAW_StatusTypeDef SpeedDistanceM1M2(RoboClaw_HandleTypeDef* hroboClaw, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag);
ROBOCLAW_StatusTypeDef SpeedAccelDistanceM1(RoboClaw_HandleTypeDef* hroboClaw, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag);
ROBOCLAW_StatusTypeDef SpeedAccelDistanceM2(RoboClaw_HandleTypeDef* hroboClaw, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag);
ROBOCLAW_StatusTypeDef SpeedAccelDistanceM1M2(RoboClaw_HandleTypeDef* hroboClaw, uint32_t accel, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag);
ROBOCLAW_StatusTypeDef SpeedAccelM1M2_2(RoboClaw_HandleTypeDef* hroboClaw, uint32_t accel1, uint32_t speed1, uint32_t accel2, uint32_t speed2);
ROBOCLAW_StatusTypeDef SpeedAccelDistanceM1M2_2(RoboClaw_HandleTypeDef* hroboClaw, uint32_t accel1, uint32_t speed1, uint32_t distance1, uint32_t accel2, uint32_t speed2, uint32_t distance2, uint8_t flag);
ROBOCLAW_StatusTypeDef DutyAccelM1(RoboClaw_HandleTypeDef* hroboClaw, uint16_t duty, uint32_t accel);
ROBOCLAW_StatusTypeDef DutyAccelM2(RoboClaw_HandleTypeDef* hroboClaw, uint16_t duty, uint32_t accel);
ROBOCLAW_StatusTypeDef DutyAccelM1M2(RoboClaw_HandleTypeDef* hroboClaw, uint16_t duty1, uint32_t accel1, uint16_t duty2, uint32_t accel2);
ROBOCLAW_StatusTypeDef SetMainVoltages(RoboClaw_HandleTypeDef* hroboClaw,uint16_t min,uint16_t max);
ROBOCLAW_StatusTypeDef SetLogicVoltages(RoboClaw_HandleTypeDef* hroboClaw,uint16_t min,uint16_t max);
ROBOCLAW_StatusTypeDef SetM1PositionPID(RoboClaw_HandleTypeDef* hroboClaw,float kp_fp,float ki_fp,float kd_fp,uint32_t kiMax,uint32_t deadzone,uint32_t min,uint32_t max)	;
ROBOCLAW_StatusTypeDef SetM2PositionPID(RoboClaw_HandleTypeDef* hroboClaw,float kp_fp,float ki_fp,float kd_fp,uint32_t kiMax,uint32_t deadzone,uint32_t min,uint32_t max)	;
ROBOCLAW_StatusTypeDef SpeedAccelDeccelPositionM1(RoboClaw_HandleTypeDef* hroboClaw,uint32_t accel,uint32_t speed,uint32_t deccel,uint32_t position,uint8_t flag);
ROBOCLAW_StatusTypeDef SpeedAccelDeccelPositionM2(RoboClaw_HandleTypeDef* hroboClaw,uint32_t accel,uint32_t speed,uint32_t deccel,uint32_t position,uint8_t flag);
ROBOCLAW_StatusTypeDef SpeedAccelDeccelPositionM1M2(RoboClaw_HandleTypeDef* hroboClaw,uint32_t accel1,uint32_t speed1,uint32_t deccel1,uint32_t position1,uint32_t accel2,uint32_t speed2,uint32_t deccel2,uint32_t position2,uint8_t flag);
ROBOCLAW_StatusTypeDef SetM1DefaultAccel(RoboClaw_HandleTypeDef* hroboClaw, uint32_t accel);
ROBOCLAW_StatusTypeDef SetM2DefaultAccel(RoboClaw_HandleTypeDef* hroboClaw, uint32_t accel);
ROBOCLAW_StatusTypeDef SetPinFunctions(RoboClaw_HandleTypeDef* hroboClaw, uint8_t S3mode, uint8_t S4mode, uint8_t S5mode);
ROBOCLAW_StatusTypeDef SetDeadBand(RoboClaw_HandleTypeDef* hroboClaw, uint8_t Min, uint8_t Max);
ROBOCLAW_StatusTypeDef RestoreDefaults(RoboClaw_HandleTypeDef* hroboClaw);
ROBOCLAW_StatusTypeDef SetM1EncoderMode(RoboClaw_HandleTypeDef* hroboClaw,uint8_t mode);
ROBOCLAW_StatusTypeDef SetM2EncoderMode(RoboClaw_HandleTypeDef* hroboClaw,uint8_t mode);
ROBOCLAW_StatusTypeDef WriteNVM(RoboClaw_HandleTypeDef* hroboClaw);
ROBOCLAW_StatusTypeDef ReadNVM(RoboClaw_HandleTypeDef* hroboClaw);
ROBOCLAW_StatusTypeDef SetConfig(RoboClaw_HandleTypeDef* hroboClaw, uint16_t config);
ROBOCLAW_StatusTypeDef SetM1MaxCurrent(RoboClaw_HandleTypeDef* hroboClaw,uint32_t max);
ROBOCLAW_StatusTypeDef SetM2MaxCurrent(RoboClaw_HandleTypeDef* hroboClaw,uint32_t max);
ROBOCLAW_StatusTypeDef SetPWMMode(RoboClaw_HandleTypeDef* hroboClaw, uint8_t mode);


//read 4_1 functions//

uint32_t ReadEncM1(RoboClaw_HandleTypeDef* hroboClaw, uint8_t *status, bool *valid);
uint32_t ReadSpeedM1(RoboClaw_HandleTypeDef* hroboClaw, uint8_t *status,bool *valid);
uint32_t ReadEncM2(RoboClaw_HandleTypeDef* hroboClaw, uint8_t *status, bool *valid);
uint32_t ReadSpeedM2(RoboClaw_HandleTypeDef* hroboClaw, uint8_t *status,bool *valid);



#endif /* ROBOCLAW_ROBOCLAW_H_ */
