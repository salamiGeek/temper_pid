#include "max6675.h"
#include <stdint.h>

float temper;
//extern uint16_t Kms10;


// #define cs_1   GPIO_SetBits(GPIOC, GPIO_Pin_11)
#define cs_1   HAL_GPIO_WritePin(MAX6675_CS_GPIO_Port,MAX6675_CS_Pin,GPIO_PIN_SET)
#define cs_0   HAL_GPIO_WritePin(MAX6675_CS_GPIO_Port,MAX6675_CS_Pin,GPIO_PIN_RESET)

// #define sck_0   GPIO_ResetBits(GPIOC, GPIO_Pin_12)
#define sck_1  HAL_GPIO_WritePin(MAX6675_SCK_GPIO_Port,MAX6675_SCK_Pin,GPIO_PIN_SET)
#define sck_0  HAL_GPIO_WritePin(MAX6675_SCK_GPIO_Port,MAX6675_SCK_Pin,GPIO_PIN_RESET)

// #define so GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_2) 
#define so    HAL_GPIO_ReadPin(MAX6675_SO_GPIO_Port,MAX6675_SO_Pin)

void delay_us(uint16_t us)
{
	 uint8_t i;
  while(us--)
	{
	  for(i=0;i<6;i++)
		{
		
		}
	}
}

uint16_t read_max6675()  
{
  uint16_t d,i; 
  //so_1;
  cs_0;
  delay_us(2);
  for(i=0;i<16;i++)
  {
    sck_1;
    delay_us(2);
    sck_0;
    delay_us(2);
    d<<=1;
    if(so)
     d++;
  }
  cs_1;
  return d;
}

float read_temper(void)
{
  uint16_t d;
  float temper;
//	if(Kms10<20)  return 0;
	
  d=read_max6675();
  temper=((d>>4)&0x0fff)*0.25;
//  Kms10=0;	
  return temper;
}
