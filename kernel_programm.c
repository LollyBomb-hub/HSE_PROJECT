#include "libschsat.h"
#include <math.h>

//Structure declarations

typedef struct
{
	float mag_col_x[3],mag_col_y[3],mag_col_z[3],mag_usual[3];
	float P,I,D;
	int MAX_BATTERY;
	float CRITICAL_BATTERY;
	int IS_MAG_COLED;
	int TIME_STEP;
	int OFF_ALL[9];
	int16_t magEarth[3],magSol[3];
} COEFS;

enum SENSORS
{
	MAG_ON, 		// 0
	SOL_1_ON,		// 1
	SOL_2_ON,		// 2
	SOL_3_ON,		// 3
	SOL_4_ON,		// 4
	HYR_ON,			// 5
	MOT_ON,			// 6
	TRANSMIT_ON,	// 7
	CAM_ON,			// 8

	MAG_OFF,		// 9
	SOL_1_OFF,		// 10
	SOL_2_OFF,		// 11
	SOL_3_OFF,		// 12
	SOL_4_OFF,		// 13
	HYR_OFF,		// 14
	MOT_OFF,		// 15
	TRANSMIT_OFF,	// 16
	CAM_OFF,		// 17

	ELEMS			// 18
};

//Declaration ended

//Function declarations

int CHECK(int SENSOR);
int INIT_SENSOR(int ENUM_ID);
int MODULE_INITIALIZATION(int INIT_PARAMS[9]);
int IS_CRITICAL(COEFS cfs);
int __time_charging(int MAX_CHARGE);

float SPEED_FUNC(float k, int t, int V0);
float SPEED_DECREASING(int time_step,int c,int time,int V0);

void COEFS_INIT(COEFS *COEFSNT);
void SPEED_DOWN(COEFS cfs,int c,int V0);
void START_CHARGING(COEFS cfs);
void FIND_EARTH_SUN(COEFS cfs,int16_t *mag_Earthx,int16_t *mag_Earthy,int16_t *mag_Earthz,int16_t *mag_Solx,int16_t *mag_Soly,int16_t *mag_Solz);
void GET_MAGNET_DATA(COEFS cfs,int16_t *mag_x,int16_t *mag_y,int16_t *mag_z);
void MAG_EXPERIMENTAL(COEFS cfs,int16_t *mag_x,int16_t *mag_y,int16_t *mag_z);
void MAG_COLIBRATED(COEFS cfs,int16_t *mag_x,int16_t *mag_y,int16_t *mag_z);
void CHECK_SOLVE_CRITICAL(COEFS cfs);
void START_CHARGING(COEFS cfs);
void GET_SOLAR_DATA(COEFS cfs,uint16_t *d1,uint16_t *d2, int id);
void MOTOR(int speed);

//Declaration ended

int CHECK(int SENSOR)
{
	SENSOR %= 9;
	if(SENSOR == MAG_ON) return magnetometer_get_state(1);
	if(1 <= SENSOR <= 4) return sun_sensor_get_state((uint16_t)SENSOR);
	if(SENSOR == HYR_ON) return hyro_get_state(1);
	if(SENSOR == MOT_ON) return motor_get_state(1);
	if(SENSOR == TRANSMIT_ON) return transceiver_get_state(1);
	if(SENSOR == CAM_ON) return camera_get_state();
}

int INIT_SENSOR(int ENUM_ID)
{
	int v;
	if(ENUM_ID == -1) return 1;
	else
	{
		const int16_t DEFAULT_NUM = 1;
		if(0 <= ENUM_ID < (int)(ELEMS/2))
		{
			if(CHECK(ENUM_ID) == 1) return 1;
			if(ENUM_ID == MAG_ON) {v = magnetometer_turn_on(DEFAULT_NUM); Sleep(1); return v;}
			if(1 <= ENUM_ID <= 4) {v = sun_sensor_turn_on((uint16_t)ENUM_ID); Sleep(1); return v;}
			if(ENUM_ID == HYR_ON) {v = hyro_turn_on(DEFAULT_NUM); Sleep(1); return v;}
			if(ENUM_ID == MOT_ON) {v = motor_turn_on(DEFAULT_NUM); Sleep(1); return v;}
			if(ENUM_ID == TRANSMIT_ON) {v = transceiver_turn_on(DEFAULT_NUM); Sleep(1); return v;}
			if(ENUM_ID == CAM_ON) {v = camera_turn_on(); Sleep(1); return v;}
		}
		else
		{
			if(CHECK(ENUM_ID) == 0) return 1;
			if(ENUM_ID == MAG_OFF) {v = magnetometer_turn_off(DEFAULT_NUM); Sleep(1); return v;}
			if(10 <= ENUM_ID <= 13) {v = sun_sensor_turn_off((uint16_t)(ENUM_ID - 9)); Sleep(1); return v;}
			if(ENUM_ID == HYR_OFF) {v = hyro_turn_off(DEFAULT_NUM); Sleep(1); return v;}
			if(ENUM_ID == MOT_OFF) {v = motor_turn_off(DEFAULT_NUM); Sleep(1); return v;}
			if(ENUM_ID == TRANSMIT_OFF) {v = transceiver_turn_off(DEFAULT_NUM); Sleep(1); return v;}
			if(ENUM_ID == CAM_OFF) {v = camera_turn_off(); Sleep(1); return v;}
		}
	}
}

int MODULE_INITIALIZATION(int INIT_PARAMS[9])
{
	int val;
	int i;
	for(i = 0; i < 9; i++)
	{
		val = INIT_SENSOR(INIT_PARAMS[i]);
		if(val != 1) 
		{
			printf("Error occured while working with module!!!");
			return -1;
		}
	}
}

int IS_CRITICAL(COEFS cfs)
{
	if(battery_get_charge() < cfs.MAX_BATTERY * cfs.CRITICAL_BATTERY) return 1;
	return 0;
}

int __time_charging(int MAX_CHARGE)
{
	return (int)(((MAX_CHARGE - battery_get_charge())/battery_get_charging_current()) + 1) * 3600;
}

float SPEED_FUNC(float k, int t, int V0)
{
	return k*t*t + V0;
}

float SPEED_DECREASING(int time_step,int c,int time,int V0)
{
	float k = -V0/(time_step*c)*(time_step*c);
	return SPEED_FUNC(k,time,V0);
}

float EARTH_ANGLE1(COEFS cfs)
{
	int16_t mg_x = cfs.magEarth[0],mg_y = cfs.magEarth[1];
	int16_t cm_x,cm_y,buf;
	GET_MAGNET_DATA(cfs,&cm_x,&cm_y,&buf);
	float angle = (float)acos((cm_x*mg_x + cm_y*mg_y)/(sqrt(cm_x*cm_x + cm_y*cm_y)*sqrt(mg_x*mg_x + mg_y*mg_y)));
	return angle;
}

float EARTH_ANGLE2(COEFS cfs)
{
	int16_t x,y,z;
	GET_MAGNET_DATA(cfs,&x,&y,&z);
	float angle = 90 - (float)atan2(y,x)*180/M_PI;
	if(angle > 180) angle = -(360 - angle);
	angle = angle*M_PI/180;
	return angle;
}

float EARTH(COEFS cfs)
{
	if(cfs.IS_MAG_COLED == 1) return EARTH_ANGLE2(cfs);
	return EARTH_ANGLE1(cfs);
}

void TURN_AROUND(COEFS cfs,float radian_angle,int time,int c)
{
	int spd = (int)(radian_angle/time) + 1;
	MOTOR(spd);
	Sleep(time - 1);
	SPEED_DOWN(cfs,c,spd);
}

void SPEED_DOWN(COEFS cfs,int c,int V0)
{
	int16_t conf;
	int i;
	for(i = 1;i < c;i++)
	{
		V0 = SPEED_DECREASING(cfs.TIME_STEP,c,i*cfs.TIME_STEP,V0);
		if(i % 2 == 0) V0 = -V0;
		motor_set_speed(1,(int16_t)V0,&conf);
		if(i % 2 == 0) V0 = -V0;
		Sleep(cfs.TIME_STEP);
	}
	motor_set_speed(1,0,&conf);
}

void MOTOR(int speed)
{
	INIT_SENSOR(MOT_ON);
	int16_t conf;
	motor_set_speed(1,(int16_t)speed,&conf);
}

void FIND_EARTH_SUN(COEFS cfs,int16_t *mag_Earthx,int16_t *mag_Earthy,int16_t *mag_Earthz,int16_t *mag_Solx,int16_t *mag_Soly,int16_t *mag_Solz)
{
	if(cfs.IS_MAG_COLED == 1) GET_MAGNET_DATA(cfs,mag_Earthx,mag_Earthy,mag_Earthz);
	else
	{
		INIT_SENSOR(SOL_1_ON); // Считаем, что data1 от этого датчика получает данные слева от спутника
		INIT_SENSOR(SOL_3_ON); // Считаем, что data2 от этого датчика получает данные справа от спутника
		// в обоих случаях данные берутся вдоль оси ординат
		const int N = 360;
		int16_t magnet_data[N][3];
		uint16_t solar_data[N][2];
		uint16_t stf1,stf2;
		int spd = 2; // Здесь должна быть скорость 1 градус за 1 секунды
		int search1a = -1,search1b = 0;
		int search2a = -1,search2b = 0;
		int solar_max = -1,solar_min = -1;
		int buf1,buf2;
		MOTOR(spd);
		int i;
		for(i = 0;i < N; i ++)
		{
			GET_MAGNET_DATA(cfs,&magnet_data[i][0],&magnet_data[i][1],&magnet_data[i][2]);
			GET_SOLAR_DATA(cfs,&stf2,&stf1,SOL_1_ON);
			solar_data[i][0] = stf2;
			if(solar_min == -1) solar_min = stf2;
			else if(solar_min > stf2) solar_min = stf2;
			if(solar_max == -1) solar_max = stf2;
			else if(solar_max < stf2) solar_max = stf2;
			GET_SOLAR_DATA(cfs,&stf1,&stf2,SOL_3_ON);
			solar_data[i][1] = stf2;
			if(i < N - 1) Sleep(1);
		}
		SPEED_DOWN(cfs,20,spd);
		const int eps = 200; // Чисто предположение
		for(i = 0;i < N; i++)
		{
			buf1 = solar_data[i][0];
			buf2 = solar_data[i][1];
			if(abs(buf1-buf2) < eps)
			{
				if((buf1 < solar_max - eps) && (buf2 < solar_max - eps) && (buf1 > solar_min + eps) && (buf2 > solar_min + eps))
				{
					if(search1a == -1) search1a = i;
					else search1b ++;
				}
				else if((buf1 > solar_max - eps) && (buf2 > solar_max - eps))
				{
					if(search2a == -1) search2a = i;
					else search2b ++;
				}
			}
		}
		search1b += search1a;
		int indE = (int)((search1a + search1b)/2);
		int indS = (int)((search2a + search2b)/2);
		*mag_Earthx = magnet_data[indE][0];
		*mag_Earthy = magnet_data[indE][1];
		*mag_Earthz = magnet_data[indE][2];
		*mag_Solx = magnet_data[indS][0];
		*mag_Soly = magnet_data[indS][1];
		*mag_Solz = magnet_data[indS][2];
	}
	return;
}

void GET_SOLAR_DATA(COEFS cfs,uint16_t *d1,uint16_t *d2, int id)
{
	INIT_SENSOR(id);
	sun_sensor_request_raw(id,d1,d2);
	return;
}

void GET_MAGNET_DATA(COEFS cfs,int16_t *mag_x,int16_t *mag_y,int16_t *mag_z)
{
	INIT_SENSOR(MAG_ON);
	if(cfs.IS_MAG_COLED == 1) MAG_COLIBRATED(cfs,mag_x,mag_y,mag_z);
	else MAG_EXPERIMENTAL(cfs,mag_x,mag_y,mag_z);
}

void MAG_EXPERIMENTAL(COEFS cfs,int16_t *mag_x,int16_t *mag_y,int16_t *mag_z)
{
	magnetometer_request_raw(1,mag_x,mag_y,mag_z);
}

void MAG_COLIBRATED(COEFS cfs,int16_t *mag_x,int16_t *mag_y,int16_t *mag_z)
{
	int locales[3] = {*mag_x,*mag_y,*mag_z};
	int i;
	for(i = 0; i < 3 ; i++)
	{
		locales[i] += cfs.mag_usual[i];
	}
	float m_x,m_y,m_z;
	for(i = 0; i < 3; i++)
	{
		m_x += cfs.mag_col_x[i] * locales[i];
		m_y += cfs.mag_col_y[i] * locales[i];
		m_z += cfs.mag_col_z[i] * locales[i];
	}
	*mag_x = (int16_t)m_x + 1;
	*mag_y = (int16_t)m_y + 1;
	*mag_z = (int16_t)m_z + 1;
	return;
}

void CHECK_SOLVE_CRITICAL(COEFS cfs)
{
	if(IS_CRITICAL(cfs) == 1) START_CHARGING(cfs);
	return;
}

void COEFS_INIT(COEFS *COEFSNT)
{
	const int mag_x[3] = {0,0,0};
	const int mag_y[3] = {0,0,0};
	const int mag_z[3] = {0,0,0};
	const int usual[3] = {0,0,0};
	COEFSNT->P = 1;
	COEFSNT->I = 1;
	COEFSNT->D = 1;
	COEFSNT->IS_MAG_COLED = 0;
	int i;
	for(i = 0; i < 3; i++)
	{
		COEFSNT->mag_col_x[i] = mag_x[i];
		COEFSNT->mag_col_y[i] = mag_y[i];
		COEFSNT->mag_col_z[i] = mag_z[i];
		COEFSNT->mag_usual[i] = usual[i];
	}
	COEFSNT->MAX_BATTERY = 2000;
	COEFSNT->CRITICAL_BATTERY = 0.2;
}

void START_CHARGING(COEFS cfs)
{
	int off_all[9] = {MAG_OFF,SOL_1_OFF,SOL_2_OFF,SOL_3_OFF,SOL_4_OFF,HYR_OFF,MOT_OFF,TRANSMIT_OFF,CAM_OFF};
	MODULE_INITIALIZATION(off_all);
	Sleep(__time_charging(cfs.MAX_BATTERY));
	return;
}

void MAKE_PHOTO(uint16_t num)
{
	INIT_SENSOR(CAM_ON);
	camera_take_photo(num);
	INIT_SENSOR(CAM_OFF);
}

void SEND_PHOTO(uint16_t photo_num)
{
	INIT_SENSOR(TRANSMIT_ON);
	transmitter_transmit_photo(1,photo_num);
	INIT_SENSOR(TRANSMIT_OFF);
}

void control(void)
{
	COEFS cfsnts;
	COEFS_INIT(&cfsnts);
	CHECK_SOLVE_CRITICAL(cfsnts);
	FIND_EARTH_SUN(cfsnts,&cfsnts.magEarth[0],&cfsnts.magEarth[1],&cfsnts.magEarth[2],&cfsnts.magSol[0],&cfsnts.magSol[1],&cfsnts.magSol[2]);
	return;
}
