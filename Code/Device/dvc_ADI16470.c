/**
 * @file dvc_ADI16470.c
 * @author Qiaoyy@xlnav.com
 * @version V1.0
 * @date 2018-04-27
 * @brief  ADI 16470 Çý¶¯³ÌÐò
 * Copyright (c) 2018 XiLang Co. Ltd.
 * All rights reserved.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "dvc_ADI16470.h"
#include "stdio.h"
#include "usart.h"
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @brief  
 * @param
 * @return 
*/

//int ADI_16470_ReadDeviceNumber(ADI_16470_HandleTypeDef* pImu)
//{
//	uint8_t spi1_txbuf[2]={0x72,0x73};
//	uint8_t spi1_rxbuf[3]={0};
//	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_RESET);
//	while(HAL_GPIO_ReadPin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin) == GPIO_PIN_SET);
//	HAL_SPI_Transmit(pImu->phspi, (uint8_t*)&spi1_txbuf[0], 2,0xFFFF);
//	HAL_SPI_Receive(pImu->phspi, (uint8_t*)&spi1_rxbuf[0], 3,0xFFFF);
//	pImu->nProductID = spi1_rxbuf[1]<<8| spi1_rxbuf[2];
//	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_SET);
//	return 1;
//}
//int ADI_16470_ReadDeviceNumber(ADI_16470_HandleTypeDef* pImu)
//{
//	unsigned short buffer = ADI_16470_PROD_ID;
//	unsigned short cmd[2]={0};
//	pImu->usBuffer[0] = cmd[0] = 0x7200;
////	pImu->usBuffer[6] = cmd[1] = 0x7300;
//	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_RESET);
//	HAL_SPI_TransmitReceive(pImu->phspi, (uint8_t*)&cmd , (uint8_t*)&pImu->usBuffer,1,100);
//	pImu->nProductID = pImu->usBuffer[0];
//	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_SET);
//	return 1;
//}
void mydelay(uint32_t timecount);
int ADI_16470_ReadDeviceNumber(ADI_16470_HandleTypeDef* pImu)
{
	unsigned short buffer[2]={ ADI_16470_PROD_ID,0};
	
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_RESET);
 // while(HAL_GPIO_ReadPin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin) == GPIO_PIN_SET);
//	while(HAL_GPIO_ReadPin(DR_16470_GPIO_Port, DR_16470_Pin) == GPIO_PIN_RESET);
	HAL_SPI_Receive(pImu->phspi, (uint8_t*)&buffer[0], 1,0xFFFF);
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_SET);
	mydelay(50);
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_RESET);
	HAL_SPI_Receive(pImu->phspi, (uint8_t*)&buffer[1], 1,0xFFFF);
	pImu->nProductID = buffer[1];
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_SET);
	return 1;
}
//int ADI_16470_ReadDeviceNumber(ADI_16470_HandleTypeDef* pImu)
//{
//	unsigned short buffer=ADI_16470_PROD_ID;
//	
//	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_RESET);
// // while(HAL_GPIO_ReadPin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin) == GPIO_PIN_SET);
////	while(HAL_GPIO_ReadPin(DR_16470_GPIO_Port, DR_16470_Pin) == GPIO_PIN_RESET);
//	HAL_SPI_Receive(pImu->phspi, (uint8_t*)&buffer, 1,0xFFFF);
//	pImu->nProductID = buffer;
//	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_SET);
//	return 1;
//}
unsigned short outbuffer[10];
int ADI_16470_ReadburstImu(ADI_16470_HandleTypeDef* pImu)
{
	unsigned short cmd;
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_RESET);
	cmd=0x6800;
  HAL_SPI_Transmit(pImu->phspi, (uint8_t*)&cmd,1,100);
	HAL_SPI_Receive(pImu->phspi, (uint8_t*)&outbuffer[0], 10,100);
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_SET);
}
int ADI_16470_ReadImu(ADI_16470_HandleTypeDef* pImu)
{
	//HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_RESET);
	unsigned short cmd[14];
	unsigned char chang[2];
	int i;
	
	
	pImu->usBuffer[0] = cmd[0] = 0x0400;
	pImu->usBuffer[6] = cmd[1] = 0x0600;

	pImu->usBuffer[1] = cmd[2] = 0x0800;
	pImu->usBuffer[7] = cmd[3] = 0x0A00;

	pImu->usBuffer[2] = cmd[4] = 0x0C00;
	pImu->usBuffer[8] = cmd[5] = 0x0E00;

	pImu->usBuffer[3] = cmd[6] = 0x1000;
	pImu->usBuffer[9] = cmd[7] = 0x1200;

	pImu->usBuffer[4] = cmd[8] = 0x1400;
	pImu->usBuffer[10] = cmd[9] = 0x1600;

	pImu->usBuffer[5] = cmd[10] = 0x1800;
	pImu->usBuffer[11] = cmd[11] = 0x1A00;

	pImu->usBuffer[12] = cmd[12] = 0x1c00;
  pImu->usBuffer[13] = cmd[13] = 0x7200;
  for(i=0;i<14;i++)
	{
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_RESET);
	//while(HAL_GPIO_ReadPin(DR_16470_GPIO_Port, DR_16470_Pin) == GPIO_PIN_RESET);
	//while(HAL_GPIO_ReadPin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin) == GPIO_PIN_SET);
	//HAL_SPI_Transmit(pImu->phspi, (uint8_t*)&cmd[i], 1,100);
	//HAL_SPI_Receive(pImu->phspi,(uint8_t*)&pImu->usBuffer[i], 1,100);
	HAL_SPI_TransmitReceive(pImu->phspi, (uint8_t*)&cmd[i], (uint8_t*)&pImu->usBuffer[i], 1,100);	
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_SET);
//	HAL_Delay(1);
	mydelay(50);
	}
//	pImu->usBuffer[0] = cmd[0] = 0x0405;
//	pImu->usBuffer[1] = cmd[1] = 0x0607;

//	pImu->usBuffer[2] = cmd[2] = 0x0809;
//	pImu->usBuffer[3] = cmd[3] = 0x0A0B;

//	pImu->usBuffer[4] = cmd[4] = 0x0C0D;
//	pImu->usBuffer[5] = cmd[5] = 0x0E0F;

//	pImu->usBuffer[6] = cmd[6] = 0x1011;
//	pImu->usBuffer[7] = cmd[7] = 0x1213;

//	pImu->usBuffer[8] = cmd[8] = 0x1415;
//	pImu->usBuffer[9] = cmd[9] = 0x1617;

//	pImu->usBuffer[10] = cmd[10] = 0x1819;
//	pImu->usBuffer[11] = cmd[11] = 0x1A1B;

//	pImu->usBuffer[12] = cmd[12] = 0x1c1D;
//  pImu->usBuffer[13] = cmd[13] = 0x7273;

// for(i=0;i<14;i++)
//	{
//	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_RESET);
//	//while(HAL_GPIO_ReadPin(DR_16470_GPIO_Port, DR_16470_Pin) == GPIO_PIN_RESET);
//	while(HAL_GPIO_ReadPin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin) == GPIO_PIN_SET);
//	HAL_SPI_Receive(pImu->phspi, (uint8_t*)&pImu->usBuffer[i], 1,0xFFFF);
//	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_SET);
////	HAL_Delay(1);
//		mydelay(50);
//	}
	//HAL_SPI_TransmitReceive(pImu->phspi, (uint8_t*)&cmd , (uint8_t*)&pImu->usBuffer, 13,100);
	
	//HAL_SPI_TransmitReceive_IT(pImu->phspi, (uint8_t*)&cmd , (uint8_t*)&pImu->usBuffer, 14);
	//HAL_SPI_Receive_IT(pImu->phspi, (uint8_t*)&pImu->usBuffer, 14);
//	for(i=1;i<14;i++)
//	{
//		chang[0] = (unsigned char)(pImu->usBuffer[i]&0x00ff);
//		chang[1] = (unsigned char)(pImu->usBuffer[i]>>8);
//		pImu->usBuffer[i] = ((unsigned long)(chang[0])<< 8) + (unsigned long)(chang[1]);
//	}
//HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_SET);

	return 1;
}
void mydelay(uint32_t timecount)
{int i,j;
	for(i=0;i<timecount;i++)
	for(j=0;j<timecount;j++)
       {;}	
}
int count16470=0;
float dataResult[7][100];
uint16_t UpdateCRC16(uint16_t crcIn, uint8_t byte) 
{
    uint32_t crc = crcIn;
    uint32_t in = byte|0x100;
    do
    {
        crc <<= 1;
        in <<= 1;
        if (in&0x100)
            ++crc;   //????:crc |= 0x01
        if (crc&0x10000)
            crc ^= 0x1021; 
    }
    while (!(in&0x10000));//??8?,?byte??crc?
    return crc&0xffffu;
}

uint16_t Cal_CRC16(uint8_t* data, uint32_t size) 
{
    uint16_t crc = 0;

    const uint8_t* dataEnd = data+size;
    while (data<dataEnd)
        crc = UpdateCRC16((uint16_t)crc,*data++);

    crc = UpdateCRC16((uint16_t)crc,0);
    crc = UpdateCRC16((uint16_t)crc,0);

    return crc&0xffffu;
}
int ADI_16470_ParseImu(ADI_16470_HandleTypeDef* pImu)
{
	FILE *fp;
	int i,j;
	uint16_t crcRes;
  uint8_t send_data_buf[30];
	pImu->fGyrX = (float)((short)pImu->usBuffer[2]) * 0.1f + (float)((short)pImu->usBuffer[1])/65536.0f * 0.1f;
	pImu->fGyrY = (float)((short)pImu->usBuffer[4]) * 0.1f + (float)((short)pImu->usBuffer[3])/65536.0f * 0.1f;
	pImu->fGyrZ = (float)((short)pImu->usBuffer[6]) * 0.1f + (float)((short)pImu->usBuffer[5])/65536.0f * 0.1f;
	pImu->fAccX = (float)((short)pImu->usBuffer[8]) * 0.00125f + (float)((short)pImu->usBuffer[7])/65536.0f * 0.00125f;
	pImu->fAccY = (float)((short)pImu->usBuffer[10]) * 0.00125f + (float)((short)pImu->usBuffer[9])/65536.0f * 0.00125f;
	pImu->fAccZ = (float)((short)pImu->usBuffer[12]) * 0.00125f + (float)((short)pImu->usBuffer[11])/65536.0f * 0.00125f;
	pImu->fTempretrue = (float)(pImu->usBuffer[13]) * 0.1f;

 fp=fopen("c:\\a.txt","w");
 
// printf("%f %f %f %f %f %f %f\n",pImu->fGyrX,pImu->fGyrY,pImu->fGyrZ,pImu->fAccX,pImu->fAccY,pImu->fAccZ,pImu->fTempretrue);
 pImu->usBuffer[0]=0x55AA;
//	for(i=0;i<14;i++)
//	{
//	 printf("%d",pImu->usBuffer[i]);
//	}
 for(i=0;i<14;i++)
	{
		send_data_buf[i*2]=pImu->usBuffer[i]&0xFF;
		send_data_buf[i*2+1]=pImu->usBuffer[i]>>8&0xFF;	
	}
	crcRes=Cal_CRC16(&send_data_buf[0], 28);
	send_data_buf[28]=crcRes>>8&0xFF;
	send_data_buf[29]=crcRes&0xFF;
	//for(i=0;i<28;i++)
	//{
	HAL_UART_Transmit(&huart1,(uint8_t *)&send_data_buf[0],30,0xffff);
	//}
 fclose(fp); 
	if(count16470<100)
	{
    dataResult[0][count16470]=pImu->fGyrX;
    dataResult[1][count16470]=pImu->fGyrY;
		dataResult[2][count16470]=pImu->fGyrZ;
		dataResult[3][count16470]=pImu->fAccX;
		dataResult[4][count16470]=pImu->fAccY;
		dataResult[5][count16470]=pImu->fAccZ;
		dataResult[6][count16470]=pImu->fTempretrue;;
    count16470++;
	}
	else if(count16470==100)
	{ 
		fp=fopen("D:\\a.txt","w+");
		if (fp==NULL)
    {
        perror("cannot open file");
        //exit(-1);
    }
    for(i=0;i<7;i++)
		{
		  for(j=0;j<100;j++)
       {
				 fprintf(fp,"%f",dataResult[i][j]);
			 }
			fputc('\n', fp);
		 }
    fclose(fp);
		count16470++;
	}	
  else
	{
		return 1;
	}		
	return 1;
}
unsigned short ErrSYS;
int ADI_16470_Readreg(ADI_16470_HandleTypeDef* pImu)
{
 	unsigned short buffer[2] = {ADI_16470_DIAG_STAT,0};
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_RESET);
 // while(HAL_GPIO_ReadPin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin) == GPIO_PIN_SET);
//	while(HAL_GPIO_ReadPin(DR_16470_GPIO_Port, DR_16470_Pin) == GPIO_PIN_RESET);
	HAL_SPI_Receive(pImu->phspi, (uint8_t*)&buffer[0], 1,0xFFFF);
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_SET);
	mydelay(50);
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_RESET);
	HAL_SPI_Receive(pImu->phspi, (uint8_t*)&buffer[1], 1,0xFFFF);
	ErrSYS= buffer[1];
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_SET);
	return 1;
}

/************************ (C) COPYRIGHT XiLang CO. Ltd. ****** END OF FILE ****/
