#include "tle.h"
#include "systick.h "
#include "para.h"
#include <math.h>
#include "util.h"

tEncoder Encoder;

uint16_t SPI2_ReadWriteByte(uint16_t byte)
{
	uint16_t retry = 0;
	while(!LL_SPI_IsActiveFlag_TXE(SPI2))  //发送缓冲区非空
	{
		if( ++retry > 200 )
			return 0;                         //延迟一段时间后返回
	}
	
	/* Transmit data in 16 Bit mode */
	LL_SPI_TransmitData16(SPI2, byte);
	
	
	/* Check BSY flag */
	retry = 0;
	while(LL_SPI_IsActiveFlag_BSY(SPI1))   // check if BSY
	{
		if(++retry > 200)
			return 0;
	}
	
	retry = 0;
	while(!LL_SPI_IsActiveFlag_RXNE(SPI2))  //接收缓冲区为空
	{
		if(++retry > 200)
			return 0;
	}
	
	return LL_SPI_ReceiveData16(SPI2);      //读一下缓冲区，清标志
}


uint16_t tle_read_reg(uint16_t READ_RES)
{
	uint16_t result;

	// CS
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_12);
	
	result = SPI2_ReadWriteByte(READ_RES);
	
	// SPI_TX_OFF
	LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_15, LL_GPIO_OUTPUT_OPENDRAIN);
	
	result = SPI2_ReadWriteByte(0xffff);
	
	// NCS
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12);
	
	// SPI_TX_ON
	LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_15, LL_GPIO_OUTPUT_PUSHPULL);

	return (result);
}

float tle_read_angle(void)
{
	float val;
	uint16_t result;
	result = tle_read_reg(READ_ANGLE_VALUE);
	val = 360.0f / 32768.0f *((float)(result & 0x3FFF) - (float)(0x4000 * ((result & 0x4000)>>14)));
	return val;
}

int tle_read_cnt(void)
{
	int cnt;
	cnt = tle_read_reg(READ_ANGLE_VALUE);
	return cnt & 0x7FFF;
}

float tle_read_speed(float dt)
{
	float val;
  uint16_t result;  
	result = tle_read_reg(READ_SPEED_VALUE);
	val =  ((float)(result & 0x3FFF) - (float)(0x4000 * ((result & 0x4000)>>14))) / dt / 65536.0f; //turns per second
	return val;
}

void Encoder_Sample(float dt)
{
	Encoder.cnt = tle_read_cnt();

	// range from 0 ~ 2PI
	Encoder.angle = Encoder.cnt * 2 * PI / ENCODER_CPR;

	Encoder.elec_angle = fmod(Usr.pole_pairs * Encoder.angle, 2*PI);

	// Turns
    if(Encoder.cnt - Encoder.cnt_last > ENCODER_CPR/2){
        Encoder.turns -= 1;
    }else if (Encoder.cnt - Encoder.cnt_last < -ENCODER_CPR/2){
        Encoder.turns += 1;
    }

	Encoder.position = Encoder.turns + Encoder.angle / (2*PI);
	
	float vel = (Encoder.position - Encoder.position_last) / dt;
	Encoder.position_last = Encoder.position;
    
	float sum = vel;
	for (int i = 1; i < VEL_VEC_SIZE; i++){
			Encoder.vel_vec[VEL_VEC_SIZE - i] = Encoder.vel_vec[VEL_VEC_SIZE-i-1];
			sum += Encoder.vel_vec[VEL_VEC_SIZE-i];
	}
	Encoder.vel_vec[0] = vel;

  Encoder.velocity =  sum / ((float)VEL_VEC_SIZE); //turns per second
		
	Encoder.velocity_elec = 2.0f*PI * Encoder.velocity * Usr.pole_pairs;
	
	Encoder.vel_tle = tle_read_speed(dt);
	
	
	Encoder.cnt_last = Encoder.cnt;

}



