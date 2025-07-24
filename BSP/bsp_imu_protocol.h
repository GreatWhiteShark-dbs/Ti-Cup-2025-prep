#ifndef __BSP_IMU_PROTOCOL_H
#define __BSP_IMU_PROTOCOL_H
#include "stm32f10x.h"
#ifdef __cplusplus
extern "C"
{
#endif

/*------------------------------------------------MARCOS define------------------------------------------------*/


/*------------------------------------------------Type define--------------------------------------------------*/
typedef enum
{
	IMU_CRC_ERR = -3,
	IMU_DATA_LEN_ERR = -2,
	IMU_PARA_ERR = -1,
	IMU_ANALYSIS_OK = 0,
	IMU_ANALYSIS_DONE = 1
}IMU_Analysis_Result_t;

#pragma pack(1)

typedef struct
{
	unsigned char header1;	/*0x59*/
	unsigned char header2;	/*0x53*/
	unsigned short tid;		/*1 -- 60000*/
	unsigned char len;		/*length of payload, 0 -- 255*/
}IMU_Output_Header_t;

typedef struct
{
	unsigned char data_id;
	unsigned char data_len;
}IMU_Payload_Data_t;

typedef struct
{
	float accel_x;			/*unit: m/s2*/
	float accel_y;
	float accel_z;

	float angle_x;			/*unit: ° (deg)/s*/
	float angle_y;
	float angle_z;

	float mag_x;			/*unit: 归一化值*/
	float mag_y;
	float mag_z;

	float raw_mag_x;		/*unit: mGauss*/
	float raw_mag_y;
	float raw_mag_z;
	
	float pitch;			/*unit: ° (deg)*/
	float roll;
	float yaw;
	
	float quaternion_data0;
	float quaternion_data1;	
	float quaternion_data2;
	float quaternion_data3;
	
	double latitude;					/*unit: deg*/
	double longtidue;					/*unit: deg*/
	float altidue;						/*unit: m*/
	
	float vel_n;						/*unit: m/s */
	float vel_e;
	float vel_d;
	
	unsigned int sample_timestamp;		/*unit: us*/
	unsigned int data_ready_timestamp;	/*unit: us*/
}IMU_Protocol_Info_t;

#pragma pack()

/*------------------------------------------------------------------------------------------------------------*/
extern IMU_Protocol_Info_t g_imu_protocol_info;

/*------------------------------------------------Functions declare--------------------------------------------*/
int BSP_IMU_Protocol_AnalysisData(unsigned char *data, short len);
uint8_t BSP_IMU_Protocol_Callback(uint8_t recv);
unsigned char BSP_IMU_Protocol_CheckDataLen(unsigned char id, unsigned char len, unsigned char *data);
int BSP_IMU_Protocol_CalcChecksum(unsigned char *data, unsigned short len, unsigned short *checksum);

#ifdef __cplusplus
}
#endif

#endif