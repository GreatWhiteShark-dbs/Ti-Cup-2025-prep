/*
* 根据不同的平台和不同的驱动包含不同的头文件
**************************************************/
//#include "xxx.h"
#include <stddef.h>
#include "bsp_imu_protocol.h"

// 添加类型定义
typedef uint8_t u8;

u8 g_uart_rx_buf[130];
IMU_Payload_Data_t *payload;

/*------------------------------------------------MARCOS define------------------------------------------------*/
#define PROTOCOL_FIRST_BYTE			(unsigned char)0x59
#define PROTOCOL_SECOND_BYTE		(unsigned char)0x53

#define PROTOCOL_FIRST_BYTE_POS 		0
#define PROTOCOL_SECOND_BYTE_POS		1

#define PROTOCOL_TID_LEN				2
#define PROTOCOL_MIN_LEN				7	/*header(2B) + tid(2B) + len(1B) + CK1(1B) + CK2(1B)*/

#define CRC_CALC_START_POS				2
#define CRC_CALC_LEN(payload_len)		((payload_len) + 3)	/*3 = tid(2B) + len(1B)*/
#define PROTOCOL_CRC_DATA_POS(payload_len)			(CRC_CALC_START_POS + CRC_CALC_LEN(payload_len))

#define PAYLOAD_POS						5

#define SINGLE_DATA_BYTES				4

/*data id define*/
#define ACCEL_ID				(unsigned char)0x10
#define ANGLE_ID				(unsigned char)0x20
#define MAGNETIC_ID				(unsigned char)0x30     /*归一化值*/
#define RAW_MAGNETIC_ID			(unsigned char)0x31     /*原始值*/
#define EULER_ID				(unsigned char)0x40
#define QUATERNION_ID			(unsigned char)0x41
#define UTC_ID					(unsigned char)0x50
#define SAMPLE_TIMESTAMP_ID		(unsigned char)0x51
#define DATA_READY_TIMESTAMP_ID	(unsigned char)0x52
#define LOCATION_ID				(unsigned char)0x60
#define SPEED_ID				(unsigned char)0x70

/*length for specific data id*/
#define ACCEL_DATA_LEN					(unsigned char)12
#define ANGLE_DATA_LEN					(unsigned char)12
#define MAGNETIC_DATA_LEN				(unsigned char)12
#define MAGNETIC_RAW_DATA_LEN			(unsigned char)12
#define EULER_DATA_LEN					(unsigned char)12
#define QUATERNION_DATA_LEN				(unsigned char)16
#define UTC_DATA_LEN					(unsigned char)11
#define SAMPLE_TIMESTAMP_DATA_LEN		(unsigned char)4
#define DATA_READY_TIMESTAMP_DATA_LEN	(unsigned char)4
#define LOCATION_DATA_LEN				(unsigned char)12
#define SPEED_DATA_LEN          		(unsigned char)12

/*factor for sensor data*/
#define NOT_MAG_DATA_FACTOR			0.000001f
#define MAG_RAW_DATA_FACTOR			0.001f

/*factor for gnss data*/
#define LONG_LAT_DATA_FACTOR		0.0000001
#define ALT_DATA_FACTOR				0.001f
#define SPEED_DATA_FACTOR			0.001f

/*------------------------------------------------Variables define------------------------------------------------*/
IMU_Protocol_Info_t g_imu_protocol_info;

/*------------------------------------------------Functions declare------------------------------------------------*/
int get_signed_int(unsigned char *data);

/*-------------------------------------------------------------------------------------------------------------*/
unsigned char BSP_IMU_Protocol_CheckDataLen(unsigned char id, unsigned char len, unsigned char *data)
{
	unsigned char ret = 0xff;

	switch(id)
	{
		case ACCEL_ID:
		{
			if(ACCEL_DATA_LEN == len)
			{
				ret = (unsigned char)0x1;
				g_imu_protocol_info.accel_x = get_signed_int(data) * NOT_MAG_DATA_FACTOR;
				g_imu_protocol_info.accel_y = get_signed_int(data + SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
				g_imu_protocol_info.accel_z = get_signed_int(data + SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
			}
			else
			{
				ret = (unsigned char)0x00;
			}
		}
		break;

		case ANGLE_ID:
		{
			if(ANGLE_DATA_LEN == len)
			{
				ret = (unsigned char)0x1;
				g_imu_protocol_info.angle_x = get_signed_int(data) * NOT_MAG_DATA_FACTOR;
				g_imu_protocol_info.angle_y = get_signed_int(data + SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
				g_imu_protocol_info.angle_z = get_signed_int(data + SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
			}
			else
			{
				ret = (unsigned char)0x00;
			}
		}
		break;

		case MAGNETIC_ID:
		{
			if(MAGNETIC_DATA_LEN == len)
			{
				ret = (unsigned char)0x1;
				g_imu_protocol_info.mag_x = get_signed_int(data) * NOT_MAG_DATA_FACTOR;
				g_imu_protocol_info.mag_y = get_signed_int(data + SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
				g_imu_protocol_info.mag_z = get_signed_int(data + SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
			}
			else
			{
				ret = (unsigned char)0x00;
			}
		}
		break;

		case RAW_MAGNETIC_ID:
		{
			if(MAGNETIC_RAW_DATA_LEN == len)
			{
				ret = (unsigned char)0x1;
				g_imu_protocol_info.raw_mag_x = get_signed_int(data) * MAG_RAW_DATA_FACTOR;
				g_imu_protocol_info.raw_mag_y = get_signed_int(data + SINGLE_DATA_BYTES) * MAG_RAW_DATA_FACTOR;
				g_imu_protocol_info.raw_mag_z = get_signed_int(data + SINGLE_DATA_BYTES * 2) * MAG_RAW_DATA_FACTOR;
			}
			else
			{
				ret = (unsigned char)0x00;
			}
		}
		break;

		case EULER_ID:
		{
			if(EULER_DATA_LEN == len)
			{
				ret = (unsigned char)0x1;
				g_imu_protocol_info.pitch = get_signed_int(data) * NOT_MAG_DATA_FACTOR;
				g_imu_protocol_info.roll = get_signed_int(data + SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
				g_imu_protocol_info.yaw = get_signed_int(data + SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
			}
			else
			{
				ret = (unsigned char)0x00;
			}
		}
		break;

		case QUATERNION_ID:
		{
			if(QUATERNION_DATA_LEN == len)
			{
				ret = (unsigned char)0x1;
				g_imu_protocol_info.quaternion_data0 = get_signed_int(data) * NOT_MAG_DATA_FACTOR;
				g_imu_protocol_info.quaternion_data1 = get_signed_int(data + SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
				g_imu_protocol_info.quaternion_data2 = get_signed_int(data + SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
				g_imu_protocol_info.quaternion_data3 = get_signed_int(data + SINGLE_DATA_BYTES * 3) * NOT_MAG_DATA_FACTOR;
			}
			else
			{
				ret = (unsigned char)0x00;
			}
		}
		break;

		case LOCATION_ID:
		{
			if(LOCATION_DATA_LEN == len)
			{
				ret = (unsigned char)0x1;
				g_imu_protocol_info.latitude = *((int *)data) * LONG_LAT_DATA_FACTOR;
				g_imu_protocol_info.longtidue = *((int *)data + 1) * LONG_LAT_DATA_FACTOR;
				g_imu_protocol_info.altidue = *((int *)data + 2)  * ALT_DATA_FACTOR;
			}
			else
			{
				ret = (unsigned char)0x00;
			}
		}
		break;
		
		case SPEED_ID:
		{
			if(SPEED_DATA_LEN == len)
			{
				ret = (unsigned char)0x1;
				g_imu_protocol_info.vel_n = *((int *)data) * SPEED_DATA_FACTOR;
				g_imu_protocol_info.vel_e = *((int *)data + 1) * SPEED_DATA_FACTOR;
				g_imu_protocol_info.vel_d = *((int *)data + 2) * SPEED_DATA_FACTOR;
			}
			else
			{
				ret = (unsigned char)0x00;
			}
		}
		break;
		
		case SAMPLE_TIMESTAMP_ID:
		{
			if(SAMPLE_TIMESTAMP_DATA_LEN == len)
			{
				ret = (unsigned char)0x1;
				g_imu_protocol_info.sample_timestamp = *((unsigned int *)data);
			}
			else
			{
				ret = (unsigned char)0x00;
			}
		}
		break;

		case DATA_READY_TIMESTAMP_ID:
		{
			if(DATA_READY_TIMESTAMP_DATA_LEN == len)
			{
				ret = (unsigned char)0x1;
				g_imu_protocol_info.data_ready_timestamp = *((unsigned int *)data);
			}
			else
			{
				ret = (unsigned char)0x00;
			}
		}
		break;		
		
		default:
		break;
	}

	return ret;
}

/*--------------------------------------------------------------------------------------------------------------
* 输出协议为：header1(0x59) + header2(0x53) + tid(2B) + payload_len(1B) + payload_data(Nbytes) + ck1(1B) + ck2(1B)
* crc校验从TID开始到payload data的最后一个字节
*/
int BSP_IMU_Protocol_AnalysisData(unsigned char *data, short len)
{
	unsigned short payload_len = 0;
	unsigned short check_sum = 0;
	unsigned short pos = 0;
	unsigned char ret = 0xff;

	IMU_Output_Header_t *header = NULL;
	IMU_Payload_Data_t *payload_ptr = NULL;

	if(NULL == data || 0 >= len)
	{
		return IMU_PARA_ERR;
	}

	if(len < PROTOCOL_MIN_LEN)
	{
			return IMU_DATA_LEN_ERR;
	}

	/*judge protocol header*/
	if(PROTOCOL_FIRST_BYTE == data[PROTOCOL_FIRST_BYTE_POS] && \
		PROTOCOL_SECOND_BYTE == data[PROTOCOL_SECOND_BYTE_POS])
	{
		
		/*further check*/
		header = (IMU_Output_Header_t *)data;
		payload_len = header->len;

		if(payload_len + PROTOCOL_MIN_LEN > len)
		{
			
			return 	IMU_DATA_LEN_ERR;
		}
		/*checksum*/
		BSP_IMU_Protocol_CalcChecksum(data + CRC_CALC_START_POS, CRC_CALC_LEN(payload_len), &check_sum);
		if(check_sum != *((unsigned short *)(data + PROTOCOL_CRC_DATA_POS(payload_len))))
		{
			
			return IMU_CRC_ERR;
		}
		/*analysis payload data*/
		pos = PAYLOAD_POS;

		while(payload_len > 0)
		{
			
			payload_ptr = (IMU_Payload_Data_t *)(data + pos);
			ret = BSP_IMU_Protocol_CheckDataLen(payload_ptr->data_id, payload_ptr->data_len, (unsigned char *)payload_ptr + 2);
			if((unsigned char)0x01 == ret)
			{
				pos += payload_ptr->data_len + sizeof(IMU_Payload_Data_t);
				payload_len -= payload_ptr->data_len + sizeof(IMU_Payload_Data_t);
			}
			else
			{
				pos++;
				payload_len--;
			}
		}
//        uart0_send_char(payload_len);
		return IMU_ANALYSIS_OK;
	}
	else
	{
		
		return IMU_ANALYSIS_DONE;
	}
}

int get_signed_int(unsigned char *data)
{
	int temp = 0;

	temp = (int)((data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0]);

	return temp;
}

int BSP_IMU_Protocol_CalcChecksum(unsigned char *data, unsigned short len, unsigned short *checksum)
{
	unsigned char check_a = 0;
	unsigned char check_b = 0;
	unsigned short i;

	if(NULL == data || 0 == len || NULL == checksum)
	{
		return -1;
	}

	for(i = 0; i < len; i++)
	{
		check_a += data[i];
		check_b += check_a;
	}

	*checksum = ((unsigned short)(check_b << 8) | check_a);

	return 0;
}


uint8_t BSP_IMU_Protocol_Callback(uint8_t recv)
{

	uint8_t data_ready = 0;
	static uint8_t last_recv=0;
	unsigned short check_sum = 0;
	unsigned short pos = 0;
	unsigned char ret = 0xff;
	
	//帧头处理
	static uint8_t Head_buf[5];
	static uint8_t count=0; //需要复位的变量值
	
	//crc16处理
	static uint8_t CRC16_buf[2];
	static uint8_t crc16_count=0; //需要复位的变量值
	
	static uint8_t recv_counts=0;//接收到的数据计数值 //需要复位的变量值
	
	
	if( (recv==0x53 && last_recv==0x59)||count>0 ) //AHRS数据帧
	{
		count++;
	}
	g_uart_rx_buf[count] = recv;
	if(count==129)
	{
		pos = 3;//pos = PAYLOAD_POS;
		payload = (IMU_Payload_Data_t *)(g_uart_rx_buf+pos);
	}
	last_recv = recv;
	return data_ready;
}