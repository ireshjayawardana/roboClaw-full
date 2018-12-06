/*
 * roboClaw.c
 *
 *  Created on: Sep 19, 2018
 *      Author: Sadeesh
 */


#include "roboClaw.h"

#define MAXTOUT 5
#define MAXRETRY 2
#define SEND_BYTE_LEN 1
#define SetDWORDval(arg) (uint8_t)(((uint32_t)arg)>>24),(uint8_t)(((uint32_t)arg)>>16),(uint8_t)(((uint32_t)arg)>>8),(uint8_t)arg
#define SetWORDval(arg) (uint8_t)(((uint16_t)arg)>>8),(uint8_t)arg

/** @addtogroup roboClaw_Private_Functions
 * @{
 */
static void crc_clear(RoboClaw_HandleTypeDef* hroboClaw);
static void crc_update (RoboClaw_HandleTypeDef* hroboClaw, uint8_t data);
static uint16_t crc_get(RoboClaw_HandleTypeDef* hroboClaw);
static ROBOCLAW_StatusTypeDef write_n(RoboClaw_HandleTypeDef* hroboClaw, uint8_t cnt, ...);
static uint32_t Read4_1(RoboClaw_HandleTypeDef* hroboClaw, uint8_t cmd, uint8_t *status, bool *valid);


static void write(RoboClaw_HandleTypeDef* hroboClaw, uint8_t *byte);
static int read(RoboClaw_HandleTypeDef* hroboClaw);
static int read_timeout(RoboClaw_HandleTypeDef* hroboClaw);
/**
 * @}
 */

ROBOCLAW_StatusTypeDef roboClaw_init(RoboClaw_HandleTypeDef* hroboClaw){

	if(hroboClaw == NULL) return ROBOCLAW_ERROR;
	if((hroboClaw->hserial == NULL) || (hroboClaw->packetserial_address == 0)) return ROBOCLAW_ERROR;
	if(!hroboClaw->timeout) hroboClaw->timeout = MAXTOUT;

	return ROBOCLAW_OK;
}

static void crc_clear(RoboClaw_HandleTypeDef* hroboClaw){
	hroboClaw->crc = 0;
}

static void crc_update (RoboClaw_HandleTypeDef* hroboClaw, uint8_t data){
	uint8_t i;
	hroboClaw->crc = hroboClaw->crc ^ ((uint16_t)data << 8);
	for (i=0; i<8; i++)
	{
		if (hroboClaw->crc & 0x8000)
			hroboClaw->crc = (hroboClaw->crc << 1) ^ 0x1021;
		else
			hroboClaw->crc <<= 1;
	}
}

static uint16_t crc_get(RoboClaw_HandleTypeDef* hroboClaw){
	return hroboClaw->crc;
}

static ROBOCLAW_StatusTypeDef write_n(RoboClaw_HandleTypeDef* hroboClaw, uint8_t cnt, ...){
	uint8_t trys=MAXRETRY;
	uint8_t data;
	uint8_t temp;
	do{
		crc_clear(hroboClaw);
		//send data with crc
		va_list marker;
		va_start( marker, cnt );     /* Initialize variable arguments. */
		for(uint8_t index=0; index<cnt ;index++){
			data = va_arg(marker, int);
			crc_update(hroboClaw, data);
			write(hroboClaw, &data);
		}
		va_end( marker );              /* Reset variable arguments.      */
		uint16_t crc = crc_get(hroboClaw);
		temp = crc>>8;
		write(hroboClaw, &temp);
		temp = (uint8_t)crc;
		write(hroboClaw,  &temp);
		if(read_timeout(hroboClaw)==0xFF)
			return  ROBOCLAW_OK ;
	}while(trys--);
	return  ROBOCLAW_ERROR;
}

static void write(RoboClaw_HandleTypeDef* hroboClaw, uint8_t *byte){
	serial_write(hroboClaw->hserial, byte, SEND_BYTE_LEN);
}

static int read(RoboClaw_HandleTypeDef* hroboClaw){
	return serial_read(hroboClaw->hserial);
}

static int read_timeout(RoboClaw_HandleTypeDef* hroboClaw){
	uint32_t start = HAL_GetTick();
	// Empty buffer?
	while(!serial_available(hroboClaw->hserial)){
		if((HAL_GetTick()-start)>=hroboClaw->timeout){
			return -1;
		}
	}
	return serial_read(hroboClaw->hserial);
}

static uint32_t Read4_1(RoboClaw_HandleTypeDef* hroboClaw, uint8_t cmd, uint8_t *status, bool *valid){

	if(valid)
		*valid = false;

	uint32_t value=0;
	uint8_t trys=MAXRETRY;
	int16_t data;
	do{

		crc_clear(hroboClaw);
		write(hroboClaw, &hroboClaw->packetserial_address);
		crc_update(hroboClaw, hroboClaw->packetserial_address);
		write(hroboClaw, &cmd);
		crc_update(hroboClaw, cmd);

		data = read_timeout(hroboClaw);
		crc_update(hroboClaw, data);
		value=(uint32_t)data<<24;

		if(data!=-1){
			data = read_timeout(hroboClaw);
			crc_update(hroboClaw, data);
			value|=(uint32_t)data<<16;
		}

		if(data!=-1){
			data = read_timeout(hroboClaw);
			crc_update(hroboClaw, data);
			value|=(uint32_t)data<<8;
		}

		if(data!=-1){
			data = read_timeout(hroboClaw);
			crc_update(hroboClaw, data);
			value|=(uint32_t)data;
		}

		if(data!=-1){
			data = read_timeout(hroboClaw);
			crc_update(hroboClaw, data);
			if(status)
				*status = data;
		}

		if(data!=-1){
			uint16_t ccrc;
			data = read_timeout(hroboClaw);
			if(data!=-1){
				ccrc = data << 8;
				data = read_timeout(hroboClaw);
				if(data!=-1){
					ccrc |= data;
					if(crc_get(hroboClaw)==ccrc){
						*valid = true;
						return value;
					}
				}
			}
		}
	}while(trys--);

	return false;
}


/* roboClaw set commands */

ROBOCLAW_StatusTypeDef ForwardM1(RoboClaw_HandleTypeDef* hroboClaw, uint8_t speed){
	return write_n(hroboClaw, 3, hroboClaw->packetserial_address, M1FORWARD, speed);
}

ROBOCLAW_StatusTypeDef BackwardM1(RoboClaw_HandleTypeDef* hroboClaw, uint8_t speed){
	return write_n(hroboClaw, 3, hroboClaw->packetserial_address,M1BACKWARD,speed);
}
ROBOCLAW_StatusTypeDef SetMinVoltageMainBattery(RoboClaw_HandleTypeDef* hroboClaw, uint8_t voltage){
	return write_n(hroboClaw, 3, hroboClaw->packetserial_address,SETMINMB,voltage);
}

ROBOCLAW_StatusTypeDef SetMaxVoltageMainBattery(RoboClaw_HandleTypeDef* hroboClaw, uint8_t voltage){
	return write_n(hroboClaw, 3, hroboClaw->packetserial_address,SETMAXMB,voltage);
}

ROBOCLAW_StatusTypeDef ForwardM2(RoboClaw_HandleTypeDef* hroboClaw, uint8_t speed){
	return write_n(hroboClaw, 3, hroboClaw->packetserial_address,M2FORWARD,speed);
}

ROBOCLAW_StatusTypeDef BackwardM2(RoboClaw_HandleTypeDef* hroboClaw, uint8_t speed){
	return write_n(hroboClaw, 3, hroboClaw->packetserial_address,M2BACKWARD,speed);
}

ROBOCLAW_StatusTypeDef ForwardBackwardM1(RoboClaw_HandleTypeDef* hroboClaw, uint8_t speed){
	return write_n(hroboClaw, 3, hroboClaw->packetserial_address,M17BIT,speed);
}

ROBOCLAW_StatusTypeDef ForwardBackwardM2(RoboClaw_HandleTypeDef* hroboClaw, uint8_t speed){
	return write_n(hroboClaw, 3, hroboClaw->packetserial_address,M27BIT,speed);
}

ROBOCLAW_StatusTypeDef ForwardMixed(RoboClaw_HandleTypeDef* hroboClaw, uint8_t speed){
	return write_n(hroboClaw, 3, hroboClaw->packetserial_address,MIXEDFORWARD,speed);
}

ROBOCLAW_StatusTypeDef BackwardMixed(RoboClaw_HandleTypeDef* hroboClaw, uint8_t speed){
	return write_n(hroboClaw, 3, hroboClaw->packetserial_address,MIXEDBACKWARD,speed);
}

ROBOCLAW_StatusTypeDef TurnRightMixed(RoboClaw_HandleTypeDef* hroboClaw, uint8_t speed){
	return write_n(hroboClaw, 3, hroboClaw->packetserial_address,MIXEDRIGHT,speed);
}

ROBOCLAW_StatusTypeDef TurnLeftMixed(RoboClaw_HandleTypeDef* hroboClaw, uint8_t speed){
	return write_n(hroboClaw, 3, hroboClaw->packetserial_address,MIXEDLEFT,speed);
}

ROBOCLAW_StatusTypeDef ForwardBackwardMixed(RoboClaw_HandleTypeDef* hroboClaw, uint8_t speed){
	return write_n(hroboClaw, 3, hroboClaw->packetserial_address,MIXEDFB,speed);
}

ROBOCLAW_StatusTypeDef LeftRightMixed(RoboClaw_HandleTypeDef* hroboClaw, uint8_t speed){
	return write_n(hroboClaw, 3, hroboClaw->packetserial_address,MIXEDLR,speed);
}

ROBOCLAW_StatusTypeDef SetM1VelocityPID(RoboClaw_HandleTypeDef* hroboClaw, float kp_fp, float ki_fp, float kd_fp, uint32_t qpps){
	uint32_t kp = kp_fp*65536;
	uint32_t ki = ki_fp*65536;
	uint32_t kd = kd_fp*65536;
	return write_n(hroboClaw, 18, hroboClaw->packetserial_address,SETM1PID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(qpps));
}
ROBOCLAW_StatusTypeDef SetM2VelocityPID(RoboClaw_HandleTypeDef* hroboClaw, float kp_fp, float ki_fp, float kd_fp, uint32_t qpps){
	uint32_t kp = kp_fp*65536;
	uint32_t ki = ki_fp*65536;
	uint32_t kd = kd_fp*65536;
	return write_n(hroboClaw, 18, hroboClaw->packetserial_address,SETM2PID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(qpps));
}
ROBOCLAW_StatusTypeDef ResetEncoders(RoboClaw_HandleTypeDef* hroboClaw){
	return write_n(hroboClaw, 2, hroboClaw->packetserial_address,RESETENC);
}
ROBOCLAW_StatusTypeDef SetEncM1(RoboClaw_HandleTypeDef* hroboClaw, int32_t val){
	return write_n(hroboClaw, 6, hroboClaw->packetserial_address,SETM1ENCCOUNT,SetDWORDval(val));
}

ROBOCLAW_StatusTypeDef SetEncM2(RoboClaw_HandleTypeDef* hroboClaw, int32_t val){
	return write_n(hroboClaw, 6, hroboClaw->packetserial_address,SETM2ENCCOUNT,SetDWORDval(val));
}
ROBOCLAW_StatusTypeDef SetMinVoltageLogicBattery(RoboClaw_HandleTypeDef* hroboClaw, uint8_t voltage){
	return write_n(hroboClaw, 3, hroboClaw->packetserial_address,SETMINLB,voltage);
}

ROBOCLAW_StatusTypeDef SetMaxVoltageLogicBattery(RoboClaw_HandleTypeDef* hroboClaw, uint8_t voltage){
	return write_n(hroboClaw, 3, hroboClaw->packetserial_address,SETMAXLB,voltage);
}
ROBOCLAW_StatusTypeDef DutyM1(RoboClaw_HandleTypeDef* hroboClaw, uint16_t duty){
	return write_n(hroboClaw, 4, hroboClaw->packetserial_address,M1DUTY,SetWORDval(duty));
}

ROBOCLAW_StatusTypeDef DutyM2(RoboClaw_HandleTypeDef* hroboClaw, uint16_t duty){
	return write_n(hroboClaw, 4, hroboClaw->packetserial_address,M2DUTY,SetWORDval(duty));
}

ROBOCLAW_StatusTypeDef DutyM1M2(RoboClaw_HandleTypeDef* hroboClaw, uint16_t duty1, uint16_t duty2){
	return write_n(hroboClaw, 6, hroboClaw->packetserial_address,MIXEDDUTY,SetWORDval(duty1),SetWORDval(duty2));
}

ROBOCLAW_StatusTypeDef SpeedM1(RoboClaw_HandleTypeDef* hroboClaw, int32_t speed){
	return write_n(hroboClaw, 6, hroboClaw->packetserial_address,M1SPEED,SetDWORDval(speed));
}

ROBOCLAW_StatusTypeDef SpeedM2(RoboClaw_HandleTypeDef* hroboClaw, int32_t speed){
	return write_n(hroboClaw, 6, hroboClaw->packetserial_address,M2SPEED,SetDWORDval(speed));
}

ROBOCLAW_StatusTypeDef SpeedM1M2(RoboClaw_HandleTypeDef* hroboClaw, uint32_t speed1, uint32_t speed2){
	return write_n(hroboClaw, 10, hroboClaw->packetserial_address,MIXEDSPEED,SetDWORDval(speed1),SetDWORDval(speed2));
}

ROBOCLAW_StatusTypeDef SpeedAccelM1(RoboClaw_HandleTypeDef* hroboClaw, uint32_t accel, uint32_t speed){
	return write_n(hroboClaw, 10, hroboClaw->packetserial_address,M1SPEEDACCEL,SetDWORDval(accel),SetDWORDval(speed));
}

ROBOCLAW_StatusTypeDef SpeedAccelM2(RoboClaw_HandleTypeDef* hroboClaw, uint32_t accel, uint32_t speed){
	return write_n(hroboClaw, 10, hroboClaw->packetserial_address,M2SPEEDACCEL,SetDWORDval(accel),SetDWORDval(speed));
}
ROBOCLAW_StatusTypeDef SpeedAccelM1M2(RoboClaw_HandleTypeDef* hroboClaw, uint32_t accel, uint32_t speed1, uint32_t speed2){
	return write_n(hroboClaw, 14, hroboClaw->packetserial_address,MIXEDSPEEDACCEL,SetDWORDval(accel),SetDWORDval(speed1),SetDWORDval(speed2));
}

ROBOCLAW_StatusTypeDef SpeedDistanceM1(RoboClaw_HandleTypeDef* hroboClaw, uint32_t speed, uint32_t distance, uint8_t flag){
	return write_n(hroboClaw, 11, hroboClaw->packetserial_address,M1SPEEDDIST,SetDWORDval(speed),SetDWORDval(distance),flag);
}

ROBOCLAW_StatusTypeDef SpeedDistanceM2(RoboClaw_HandleTypeDef* hroboClaw, uint32_t speed, uint32_t distance, uint8_t flag){
	return write_n(hroboClaw, 11, hroboClaw->packetserial_address,M2SPEEDDIST,SetDWORDval(speed),SetDWORDval(distance),flag);
}

ROBOCLAW_StatusTypeDef SpeedDistanceM1M2(RoboClaw_HandleTypeDef* hroboClaw, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag){
	return write_n(hroboClaw, 19, hroboClaw->packetserial_address,MIXEDSPEEDDIST,SetDWORDval(speed1),SetDWORDval(distance1),SetDWORDval(speed2),SetDWORDval(distance2),flag);
}

ROBOCLAW_StatusTypeDef SpeedAccelDistanceM1(RoboClaw_HandleTypeDef* hroboClaw, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag){
	return write_n(hroboClaw, 15, hroboClaw->packetserial_address,M1SPEEDACCELDIST,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(distance),flag);
}

ROBOCLAW_StatusTypeDef SpeedAccelDistanceM2(RoboClaw_HandleTypeDef* hroboClaw, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag){
	return write_n(hroboClaw, 15, hroboClaw->packetserial_address,M2SPEEDACCELDIST,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(distance),flag);
}

ROBOCLAW_StatusTypeDef SpeedAccelDistanceM1M2(RoboClaw_HandleTypeDef* hroboClaw, uint32_t accel, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag){
	return write_n(hroboClaw, 23, hroboClaw->packetserial_address,MIXEDSPEEDACCELDIST,SetDWORDval(accel),SetDWORDval(speed1),SetDWORDval(distance1),SetDWORDval(speed2),SetDWORDval(distance2),flag);
}
ROBOCLAW_StatusTypeDef SpeedAccelM1M2_2(RoboClaw_HandleTypeDef* hroboClaw, uint32_t accel1, uint32_t speed1, uint32_t accel2, uint32_t speed2){
	return write_n(hroboClaw, 18, hroboClaw->packetserial_address,MIXEDSPEED2ACCEL,SetDWORDval(accel1),SetDWORDval(speed1),SetDWORDval(accel2),SetDWORDval(speed2));
}

ROBOCLAW_StatusTypeDef SpeedAccelDistanceM1M2_2(RoboClaw_HandleTypeDef* hroboClaw, uint32_t accel1, uint32_t speed1, uint32_t distance1, uint32_t accel2, uint32_t speed2, uint32_t distance2, uint8_t flag){
	return write_n(hroboClaw, 27, hroboClaw->packetserial_address,MIXEDSPEED2ACCELDIST,SetDWORDval(accel1),SetDWORDval(speed1),SetDWORDval(distance1),SetDWORDval(accel2),SetDWORDval(speed2),SetDWORDval(distance2),flag);
}

ROBOCLAW_StatusTypeDef DutyAccelM1(RoboClaw_HandleTypeDef* hroboClaw, uint16_t duty, uint32_t accel){
	return write_n(hroboClaw, 8, hroboClaw->packetserial_address,M1DUTYACCEL,SetWORDval(duty),SetDWORDval(accel));
}

ROBOCLAW_StatusTypeDef DutyAccelM2(RoboClaw_HandleTypeDef* hroboClaw, uint16_t duty, uint32_t accel){
	return write_n(hroboClaw, 8, hroboClaw->packetserial_address,M2DUTYACCEL,SetWORDval(duty),SetDWORDval(accel));
}

ROBOCLAW_StatusTypeDef DutyAccelM1M2(RoboClaw_HandleTypeDef* hroboClaw, uint16_t duty1, uint32_t accel1, uint16_t duty2, uint32_t accel2){
	return write_n(hroboClaw, 14, hroboClaw->packetserial_address,MIXEDDUTYACCEL,SetWORDval(duty1),SetDWORDval(accel1),SetWORDval(duty2),SetDWORDval(accel2));
}
ROBOCLAW_StatusTypeDef SetMainVoltages(RoboClaw_HandleTypeDef* hroboClaw,uint16_t min,uint16_t max){
	return write_n(hroboClaw, 6, hroboClaw->packetserial_address,SETMAINVOLTAGES,SetWORDval(min),SetWORDval(max));
}

ROBOCLAW_StatusTypeDef SetLogicVoltages(RoboClaw_HandleTypeDef* hroboClaw,uint16_t min,uint16_t max){
	return write_n(hroboClaw, 6, hroboClaw->packetserial_address,SETLOGICVOLTAGES,SetWORDval(min),SetWORDval(max));
}
ROBOCLAW_StatusTypeDef SetM1PositionPID(RoboClaw_HandleTypeDef* hroboClaw,float kp_fp,float ki_fp,float kd_fp,uint32_t kiMax,uint32_t deadzone,uint32_t min,uint32_t max){
	uint32_t kp=kp_fp*1024;
	uint32_t ki=ki_fp*1024;
	uint32_t kd=kd_fp*1024;
	return write_n(hroboClaw, 30, hroboClaw->packetserial_address,SETM1POSPID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(kiMax),SetDWORDval(deadzone),SetDWORDval(min),SetDWORDval(max));
}

ROBOCLAW_StatusTypeDef SetM2PositionPID(RoboClaw_HandleTypeDef* hroboClaw,float kp_fp,float ki_fp,float kd_fp,uint32_t kiMax,uint32_t deadzone,uint32_t min,uint32_t max){
	uint32_t kp=kp_fp*1024;
	uint32_t ki=ki_fp*1024;
	uint32_t kd=kd_fp*1024;
	return write_n(hroboClaw, 30, hroboClaw->packetserial_address,SETM2POSPID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(kiMax),SetDWORDval(deadzone),SetDWORDval(min),SetDWORDval(max));
}
ROBOCLAW_StatusTypeDef SpeedAccelDeccelPositionM1(RoboClaw_HandleTypeDef* hroboClaw,uint32_t accel,uint32_t speed,uint32_t deccel,uint32_t position,uint8_t flag){
	return write_n(hroboClaw, 19, hroboClaw->packetserial_address,M1SPEEDACCELDECCELPOS,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(deccel),SetDWORDval(position),flag);
}

ROBOCLAW_StatusTypeDef SpeedAccelDeccelPositionM2(RoboClaw_HandleTypeDef* hroboClaw,uint32_t accel,uint32_t speed,uint32_t deccel,uint32_t position,uint8_t flag){
	return write_n(hroboClaw, 19, hroboClaw->packetserial_address,M2SPEEDACCELDECCELPOS,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(deccel),SetDWORDval(position),flag);
}

ROBOCLAW_StatusTypeDef SpeedAccelDeccelPositionM1M2(RoboClaw_HandleTypeDef* hroboClaw,uint32_t accel1,uint32_t speed1,uint32_t deccel1,uint32_t position1,uint32_t accel2,uint32_t speed2,uint32_t deccel2,uint32_t position2,uint8_t flag){
	return write_n(hroboClaw, 35, hroboClaw->packetserial_address,MIXEDSPEEDACCELDECCELPOS,SetDWORDval(accel1),SetDWORDval(speed1),SetDWORDval(deccel1),SetDWORDval(position1),SetDWORDval(accel2),SetDWORDval(speed2),SetDWORDval(deccel2),SetDWORDval(position2),flag);
}

ROBOCLAW_StatusTypeDef SetM1DefaultAccel(RoboClaw_HandleTypeDef* hroboClaw, uint32_t accel){
	return write_n(hroboClaw, 6, hroboClaw->packetserial_address,SETM1DEFAULTACCEL,SetDWORDval(accel));
}

ROBOCLAW_StatusTypeDef SetM2DefaultAccel(RoboClaw_HandleTypeDef* hroboClaw, uint32_t accel){
	return write_n(hroboClaw, 6, hroboClaw->packetserial_address,SETM2DEFAULTACCEL,SetDWORDval(accel));
}

ROBOCLAW_StatusTypeDef SetPinFunctions(RoboClaw_HandleTypeDef* hroboClaw, uint8_t S3mode, uint8_t S4mode, uint8_t S5mode){
	return write_n(hroboClaw, 5, hroboClaw->packetserial_address,SETPINFUNCTIONS,S3mode,S4mode,S5mode);
}
ROBOCLAW_StatusTypeDef SetDeadBand(RoboClaw_HandleTypeDef* hroboClaw, uint8_t Min, uint8_t Max){
	return write_n(hroboClaw, 4, hroboClaw->packetserial_address,SETDEADBAND,Min,Max);
}
ROBOCLAW_StatusTypeDef RestoreDefaults(RoboClaw_HandleTypeDef* hroboClaw){
	return write_n(hroboClaw, 2, hroboClaw->packetserial_address,RESTOREDEFAULTS);
}
ROBOCLAW_StatusTypeDef SetM1EncoderMode(RoboClaw_HandleTypeDef* hroboClaw,uint8_t mode){
	return write_n(hroboClaw, 3, hroboClaw->packetserial_address,SETM1ENCODERMODE,mode);
}

ROBOCLAW_StatusTypeDef SetM2EncoderMode(RoboClaw_HandleTypeDef* hroboClaw,uint8_t mode){
	return write_n(hroboClaw, 3, hroboClaw->packetserial_address,SETM2ENCODERMODE,mode);
}

ROBOCLAW_StatusTypeDef WriteNVM(RoboClaw_HandleTypeDef* hroboClaw){
	return write_n(hroboClaw, 6, hroboClaw->packetserial_address,WRITENVM, SetDWORDval(0xE22EAB7A) );
}

ROBOCLAW_StatusTypeDef ReadNVM(RoboClaw_HandleTypeDef* hroboClaw){
	return write_n(hroboClaw, 2, hroboClaw->packetserial_address,READNVM);
}

ROBOCLAW_StatusTypeDef SetConfig(RoboClaw_HandleTypeDef* hroboClaw, uint16_t config){
	return write_n(hroboClaw, 4, hroboClaw->packetserial_address,SETCONFIG,SetWORDval(config));
}
ROBOCLAW_StatusTypeDef SetM1MaxCurrent(RoboClaw_HandleTypeDef* hroboClaw,uint32_t max){
	return write_n(hroboClaw, 10, hroboClaw->packetserial_address,SETM1MAXCURRENT,SetDWORDval(max),SetDWORDval(0));
}

ROBOCLAW_StatusTypeDef SetM2MaxCurrent(RoboClaw_HandleTypeDef* hroboClaw,uint32_t max){
	return write_n(hroboClaw, 10, hroboClaw->packetserial_address,SETM2MAXCURRENT,SetDWORDval(max),SetDWORDval(0));
}
ROBOCLAW_StatusTypeDef SetPWMMode(RoboClaw_HandleTypeDef* hroboClaw, uint8_t mode){
	return write_n(hroboClaw, 3, hroboClaw->packetserial_address,SETPWMMODE,mode);
}



/* roboClaw get commands */

uint32_t ReadEncM1(RoboClaw_HandleTypeDef* hroboClaw, uint8_t *status, bool *valid){
	return Read4_1(hroboClaw, GETM1ENC, status, valid);
}

uint32_t ReadEncM2(RoboClaw_HandleTypeDef* hroboClaw, uint8_t *status, bool *valid){
	return Read4_1(hroboClaw, GETM2ENC, status, valid);
}

uint32_t ReadSpeedM1(RoboClaw_HandleTypeDef* hroboClaw, uint8_t *status,bool *valid){
	return Read4_1(hroboClaw, GETM1SPEED, status, valid);
}

uint32_t ReadSpeedM2(RoboClaw_HandleTypeDef* hroboClaw, uint8_t *status,bool *valid){
	return Read4_1(hroboClaw, GETM2SPEED, status, valid);
}

//ROBOCLAW_StatusTypeDef ReadPWMs(uint8_t address, int16_t &pwm1, int16_t &pwm2){
//	bool valid;
//	uint32_t value = Read4(address,GETPWMS,&valid);
//	if(valid){
//		pwm1 = value>>16;
//		pwm2 = value&0xFFFF;
//	}
//	return valid;
//}
