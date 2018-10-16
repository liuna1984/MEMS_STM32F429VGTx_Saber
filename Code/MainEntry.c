#include "Global.h"
#include "Device.h"
#include "MainEntry.h"
#include "usart.h"
#include "stdio.h"
#include "dvcSaber.h"

extern unsigned char f10ms;

void HeartPulse(void);
extern SPI_HandleTypeDef hspi1;

extern unsigned char sscp_mac;
extern unsigned char process_num;
extern unsigned char send_over;

extern unsigned char measure_channel;
extern unsigned char message_channel;
extern unsigned char fifo_status_channel;
extern unsigned char message_buffer[1024];
extern unsigned char recieve_message_buffer[1024];
extern unsigned char fill_byte[5];
    


extern unsigned char switch_config[512];
extern  unsigned char set_packet_config[512];
extern unsigned char set_odr[512];
extern unsigned char get_packet_config[512];
extern unsigned char switch_measure[512];

extern unsigned char recieve_fifo_status_buffer[10 + SSCP_FILL];
extern unsigned char recieve_response_buffer[1024];

extern unsigned int response_byte;
extern unsigned int measure_byte;
extern unsigned char response_channel;

union
{
  unsigned short odr;
  unsigned char odr_char[2];
}update_rate;
    


unsigned short packet_session[42] = 
{
SESSION_NAME_TEMPERATURE,
SESSION_NAME_RAW_ACC,
SESSION_NAME_RAW_GYRO,
SESSION_NAME_RAW_MAG,
SESSION_NAME_CAL_ACC,
SESSION_NAME_KAL_ACC,
SESSION_NAME_LINEAR_ACC,
SESSION_NAME_HEAVE_MOTION,
SESSION_NAME_DELTA_V_ACC,
SESSION_NAME_CAL_GYRO,
SESSION_NAME_KAL_GYRO,
SESSION_NAME_DELTA_Q_GYRO,
SESSION_NAME_CAL_MAG,
SESSION_NAME_KAL_MAG,
SESSION_NAME_MAG_DEV,
SESSION_NAME_MAG_DIS_PCT,
SESSION_NAME_CAL_BARO,
SESSION_NAME_KAL_BARO,
SESSION_NAME_GNSS_PVT,
SESSION_NAME_GNSS_SATELITTE,
SESSION_NAME_GPS_DOP,
SESSION_NAME_GPS_SOL,
SESSION_NAME_GPS_TIME,
SESSION_NAME_GPS_SV,
SESSION_NAME_QUAT,
SESSION_NAME_EULER,
SESSION_NAME_ROTATION_M,
SESSION_NAME_POSITION_ALTITUDE,
SESSION_NAME_POSITION_ECEF,
SESSION_NAME_POSITION_LATLON,
SESSION_NAME_POSITION_VELOCITY,
SESSION_NAME_POSITION_DISTANCE,
SESSION_NAME_PACKET_COUNTER,
SESSION_NAME_UTC_TIME,
SESSION_NAME_OS_TIME,
SESSION_NAME_SAMPLE_TIME_FINE,
SESSION_NAME_SAMPLE_TIME_COARSE,
SESSION_NAME_ITOW,
SESSION_NAME_DELTA_T, 
SESSION_NAME_STATUS_WORD,
SESSION_NAME_RSSI,
SESSION_NAME_CPU_USAGE,
};

unsigned char packet_mux[42] = {0};

union
{
  unsigned short Packet_name_short;
  unsigned char Packet_name_char[2];
}packet_config_array[42];

void packet_config_init(void)
{
  unsigned int i = 0;
  for( i = 0 ; i < 42; i ++)
  {
    packet_config_array[i].Packet_name_short = packet_session[i];
  }
}

/*this function is used to build the payload of message "SetDataPacketConfig"*/
/*this function will return the payload len of message "SetDataPacketConfig" */
unsigned int  packet_config(unsigned char * packet)
{
  unsigned int i = 0,payload_len = 0;

  for(i = 0; i < 42; i ++)
  {
    if(packet_mux[i])
    {
      packet[payload_len++] = 0xff;
      packet[payload_len++] = 0xff;
      packet_config_array[i].Packet_name_short |= 0x8000; 
      packet[payload_len++] = packet_config_array[i].Packet_name_char[0];
      packet[payload_len++] = packet_config_array[i].Packet_name_char[1];
    }
    else
    {
      packet_config_array[i].Packet_name_short &= 0x7fff;
    }
  }

  return payload_len;
}
/*this function is used to get the CRC value                  */
/*the parameter data is an array of our protocol except the CRC and Tail  */
/*the parameter sum is the length of data                     */
unsigned int check_bcc(unsigned char * data,unsigned int sum)
{
  unsigned char bcc_num = 0;
  unsigned int i = 0;
  while(i < sum)
  {
    bcc_num ^= *data;
    data++;
    i++;
  }
  return bcc_num;
}
void GotoMeasure(void); 
int GetMeasurnCount(void);
unsigned short FramLength=0;
unsigned char CheckSum8(unsigned char* pBuffer, int nLength);
typedef union   
{  
    struct   
    {  
        unsigned char low_byte;  
        unsigned char mlow_byte;  
        unsigned char mhigh_byte;  
        unsigned char high_byte;  
     }float_byte;  
            
     float  value;  
}FLAOT_UNION;
FLAOT_UNION Temperature,Gyro[3],Acc[3];
/*RAW_DATA*/
typedef union
{
	char c[2];
	short s;
}uTos;
float GYRO_f[3],ACC_f[3];
void MainEntry(void)
{
   unsigned int index = 0,payload_len = 0,transfer_byte = 0;
   unsigned int packet_length=10;

   InitDevice();
   packet_config_init();
//   GotoMeasure();
//   GetMeasurnCount();
	
   //HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
   process_num = SWITCH_TO_CONFIG;
   sscp_mac=NONE_SSCP;
   send_over =1;
    while (1)
    {
	if(send_over)
    {
      index = 0;
      if(process_num == SWITCH_TO_CONFIG)
      {
       //send_over = 0;
        //when you want send a message with sscp
        //you should build a buffer,maybe named "message_buffer"
        //the first element must be the message channel (0xa1)
        //you can add the protocol that we provide follow the channel
        //below is a sample of send "SwitchToConfigMode" 
        
        memcpy(message_buffer,&message_channel,1);/*this is our message channel of SSCP*/
        
        switch_config[index++] = 0x41;/*    0x41 0x78 is our protocol header   */
        switch_config[index++] = 0x78;/*                                       */
        switch_config[index++] = 0xff;/*    0xff is our protocol maddr         */
        switch_config[index++] = 0x01;/*    0x01 is our protocol class ID      */
        switch_config[index++] = 0x02;/*    0x02 is our protocol message ID    */
        switch_config[index++] = 0x00;/*    0x00 is our protocol payload length*/
        switch_config[index] = check_bcc(switch_config,index);/*this is our protocol CRC, you can use the function*/
        index++;                                              /*check_bcc to get the value                        */
        switch_config[index] = 0x6d;  /*    0x6d is our protocol tail          */
        

        fill_byte[0] = ((index + 1) & 0xff);
        fill_byte[1] = ( ((index + 1) >> 8) & 0xff);
        
        memcpy(message_buffer + 1,fill_byte,5);
        
        memcpy(message_buffer + SSCP_FILL,switch_config,index+1);
        
        transfer_byte = index + 1 + SSCP_FILL;
        
  	
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		//HAL_SPI_Transmit(&hspi1, message_buffer,transfer_byte,100);
	    HAL_SPI_TransmitReceive(&hspi1, message_buffer, recieve_message_buffer, transfer_byte,100);
	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);	
    	//HAL_Delay(1000);
		//HAL_I2C_Master_Transmit_IT(&hi2c1,IIC_ADDR,message_buffer,index + 2);/*you can send this message by IIC*/
//           /*get unread_bytes_response and unread_bytes_measure*/
       sscp_mac=NONE_SSCP;
	   while(sscp_mac!=GET_FIFO_STATUS);
//		if(sscp_mac==GET_FIFO_STATUS)
		{
			response_byte |= recieve_fifo_status_buffer[0 + SSCP_FILL] << 0;
			response_byte |= recieve_fifo_status_buffer[1 + SSCP_FILL] << 8;
			response_byte |= recieve_fifo_status_buffer[2 + SSCP_FILL] << 16;
			response_byte |= recieve_fifo_status_buffer[3 + SSCP_FILL] << 24;

			measure_byte |= recieve_fifo_status_buffer[4 + SSCP_FILL] << 0;
			measure_byte |= recieve_fifo_status_buffer[5 + SSCP_FILL] << 8;
			measure_byte |= recieve_fifo_status_buffer[6 + SSCP_FILL] << 16;
			measure_byte |= recieve_fifo_status_buffer[7 + SSCP_FILL] << 24;
		  
			
			
			
			

			fill_byte[0] = response_byte & 0xff;
			fill_byte[1] = (response_byte >> 8) & 0xff;

			memcpy(message_buffer,&response_channel,1);
			memcpy(message_buffer + 1,fill_byte,5);
			transfer_byte = response_byte + SSCP_FILL;
			
		   
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
			HAL_SPI_TransmitReceive(&hspi1, message_buffer, recieve_response_buffer, transfer_byte,100);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	  }
	  sscp_mac = SEND_MESSAGE;
	  process_num=SET_DATA_PACKET;
      }
	   else if(process_num == SET_DATA_PACKET)
      {
        //send_over = 0;
        memcpy(message_buffer,&message_channel,1);
        
        /*you can select the packet whitch you want use packet_mux[x],1 means enable the packet x*/
        /*for example, you want choose temperature,raw acc,raw gyro,raw mag,you can do like below*/
        packet_mux[TEMPERATURE] = 1;
		packet_mux[CAL_ACC]=1;
		packet_mux[CAL_GYRO]=1;
		
    //    packet_mux[RAW_ACC] = 1;
    //    packet_mux[RAW_GYRO] = 1;
    //    packet_mux[RAW_MAG] = 1;

        
        set_packet_config[index++] = 0x41;
        set_packet_config[index++] = 0x78;
        set_packet_config[index++] = 0xff;
        set_packet_config[index++] = 0x06;
        set_packet_config[index++] = 0x0a;

        /*this function is used to build the payload of message "SetDataPacketConfig"*/
        /*this function will return the payload len of message "SetDataPacketConfig" */
        payload_len = packet_config(set_packet_config + 6);
        
        set_packet_config[index++] = payload_len;
        index += payload_len;
        set_packet_config[index] = check_bcc(set_packet_config,index);
        index++;
        set_packet_config[index] = 0x6d;
        
        fill_byte[0] = ((index + 1) & 0xff);
        fill_byte[1] = ( ((index + 1) >> 8) & 0xff);
        
        memcpy(message_buffer + 1,fill_byte,5);
        
        memcpy(message_buffer + SSCP_FILL,set_packet_config,index+1);
        
        transfer_byte = index + 1 + SSCP_FILL;
        
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive(&hspi1, message_buffer, recieve_message_buffer, transfer_byte,100);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);       
	    sscp_mac = SEND_MESSAGE;
		
       sscp_mac=NONE_SSCP;
	   while(sscp_mac!=GET_FIFO_STATUS);
//		if(sscp_mac==GET_FIFO_STATUS)
		{
			response_byte |= recieve_fifo_status_buffer[0 + SSCP_FILL] << 0;
			response_byte |= recieve_fifo_status_buffer[1 + SSCP_FILL] << 8;
			response_byte |= recieve_fifo_status_buffer[2 + SSCP_FILL] << 16;
			response_byte |= recieve_fifo_status_buffer[3 + SSCP_FILL] << 24;

			measure_byte |= recieve_fifo_status_buffer[4 + SSCP_FILL] << 0;
			measure_byte |= recieve_fifo_status_buffer[5 + SSCP_FILL] << 8;
			measure_byte |= recieve_fifo_status_buffer[6 + SSCP_FILL] << 16;
			measure_byte |= recieve_fifo_status_buffer[7 + SSCP_FILL] << 24;
		  
			
			
			
			

			fill_byte[0] = response_byte & 0xff;
			fill_byte[1] = (response_byte >> 8) & 0xff;

			memcpy(message_buffer,&response_channel,1);
			memcpy(message_buffer + 1,fill_byte,5);
			transfer_byte = response_byte + SSCP_FILL;
			
		   
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
			HAL_SPI_TransmitReceive(&hspi1, message_buffer, recieve_response_buffer, transfer_byte,100);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	  }
		process_num=SET_ODR;
		//process_num = SWITCH_TO_MEASURE;
		//HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

      }
      else if(process_num == SET_ODR)
      {
       //send_over = 0;
        memcpy(message_buffer,&message_channel,1);
        set_odr[index++] = 0x41;
        set_odr[index++] = 0x78;
        set_odr[index++] = 0xff;
        set_odr[index++] = 0x04;
        set_odr[index++] = 0x10;
        set_odr[index++] = 0x02;
        update_rate.odr = 100;
        set_odr[index++] = update_rate.odr_char[0];
        set_odr[index++] = update_rate.odr_char[1];
        set_odr[index] = check_bcc(set_odr,index);
        index++;
        set_odr[index] = 0x6d;

        
        fill_byte[0] = ((index + 1) & 0xff);
        fill_byte[1] = ( ((index + 1) >> 8) & 0xff);
        
        memcpy(message_buffer + 1,fill_byte,5);
        
        memcpy(message_buffer + SSCP_FILL,set_odr,index+1);
        
        transfer_byte = index + 1 + SSCP_FILL;
        
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive(&hspi1, message_buffer, recieve_message_buffer, transfer_byte,100);
         HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	    sscp_mac=NONE_SSCP;
	    while(sscp_mac!=GET_FIFO_STATUS);
//		if(sscp_mac==GET_FIFO_STATUS)
		{
			response_byte |= recieve_fifo_status_buffer[0 + SSCP_FILL] << 0;
			response_byte |= recieve_fifo_status_buffer[1 + SSCP_FILL] << 8;
			response_byte |= recieve_fifo_status_buffer[2 + SSCP_FILL] << 16;
			response_byte |= recieve_fifo_status_buffer[3 + SSCP_FILL] << 24;

			measure_byte |= recieve_fifo_status_buffer[4 + SSCP_FILL] << 0;
			measure_byte |= recieve_fifo_status_buffer[5 + SSCP_FILL] << 8;
			measure_byte |= recieve_fifo_status_buffer[6 + SSCP_FILL] << 16;
			measure_byte |= recieve_fifo_status_buffer[7 + SSCP_FILL] << 24;
		  
			
			
			
			

			fill_byte[0] = response_byte & 0xff;
			fill_byte[1] = (response_byte >> 8) & 0xff;

			memcpy(message_buffer,&response_channel,1);
			memcpy(message_buffer + 1,fill_byte,5);
			transfer_byte = response_byte + SSCP_FILL;
			
		   
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
			HAL_SPI_TransmitReceive(&hspi1, message_buffer, recieve_response_buffer, transfer_byte,100);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	  }
		sscp_mac = SEND_MESSAGE;
//		process_num = SWITCH_TO_MEASURE;
		process_num=GET_DATA_PACKET;
		
		

      }
      else if(process_num == GET_DATA_PACKET)
      {
       // send_over = 0;
        memcpy(message_buffer,&message_channel,1);
        get_packet_config[index++] = 0x41;
        get_packet_config[index++] = 0x78;
        get_packet_config[index++] = 0xff;
	 //  get_packet_config[index++] = 0x03;
     //  get_packet_config[index++] = 0x02;
        get_packet_config[index++] = 0x06;
       get_packet_config[index++] = 0x0b;
        get_packet_config[index++] = 0x00;
        get_packet_config[index] = check_bcc(get_packet_config,index);
        index++;
        get_packet_config[index] = 0x6d;

        
        fill_byte[0] = ((index + 1) & 0xff);
        fill_byte[1] = ( ((index + 1) >> 8) & 0xff);
        
        memcpy(message_buffer + 1,fill_byte,5);
        
        memcpy(message_buffer + SSCP_FILL,get_packet_config,index+1);
        
        transfer_byte = index + 1 + SSCP_FILL;
        
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive(&hspi1, message_buffer, recieve_message_buffer, transfer_byte,100);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
		//HAL_Delay(1000);
		sscp_mac=NONE_SSCP;
		while(sscp_mac!=GET_FIFO_STATUS);
		//if(sscp_mac==GET_FIFO_STATUS)
		{
			response_byte |= recieve_fifo_status_buffer[0 + SSCP_FILL] << 0;
			response_byte |= recieve_fifo_status_buffer[1 + SSCP_FILL] << 8;
			response_byte |= recieve_fifo_status_buffer[2 + SSCP_FILL] << 16;
			response_byte |= recieve_fifo_status_buffer[3 + SSCP_FILL] << 24;

			measure_byte |= recieve_fifo_status_buffer[4 + SSCP_FILL] << 0;
			measure_byte |= recieve_fifo_status_buffer[5 + SSCP_FILL] << 8;
			measure_byte |= recieve_fifo_status_buffer[6 + SSCP_FILL] << 16;
			measure_byte |= recieve_fifo_status_buffer[7 + SSCP_FILL] << 24;
		  
			
			
			
			

			fill_byte[0] = response_byte & 0xff;
			fill_byte[1] = (response_byte >> 8) & 0xff;

			memcpy(message_buffer,&response_channel,1);
			memcpy(message_buffer + 1,fill_byte,5);
			transfer_byte = response_byte + SSCP_FILL;
			
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
			HAL_SPI_TransmitReceive(&hspi1, message_buffer, recieve_response_buffer, transfer_byte,100);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	  }
		sscp_mac = SEND_MESSAGE;
		process_num = SWITCH_TO_MEASURE;

      }
      else if(process_num == SWITCH_TO_MEASURE)
      {
        //send_over = 0;
        memcpy(message_buffer,&message_channel,1);
        switch_measure[index++] = 0x41;
        switch_measure[index++] = 0x78;
        switch_measure[index++] = 0xff;
        switch_measure[index++] = 0x01;
        switch_measure[index++] = 0x03;
        switch_measure[index++] = 0x00;
        switch_measure[index] = check_bcc(switch_measure,index);
        index++;
        switch_measure[index] = 0x6d;
        fill_byte[0] = ((index + 1) & 0xff);
        fill_byte[1] = ( ((index + 1) >> 8) & 0xff);
        
        memcpy(message_buffer + 1,fill_byte,5);
        
        memcpy(message_buffer + SSCP_FILL,switch_measure,index+1);
        
        transfer_byte = index + 1 + SSCP_FILL;
        
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive(&hspi1, message_buffer, recieve_message_buffer, transfer_byte,100);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
		
	   sscp_mac=NONE_SSCP;
	   while(sscp_mac!=GET_FIFO_STATUS);
//		if(sscp_mac==GET_FIFO_STATUS)
		{
			response_byte |= recieve_fifo_status_buffer[0 + SSCP_FILL] << 0;
			response_byte |= recieve_fifo_status_buffer[1 + SSCP_FILL] << 8;
			response_byte |= recieve_fifo_status_buffer[2 + SSCP_FILL] << 16;
			response_byte |= recieve_fifo_status_buffer[3 + SSCP_FILL] << 24;

			measure_byte |= recieve_fifo_status_buffer[4 + SSCP_FILL] << 0;
			measure_byte |= recieve_fifo_status_buffer[5 + SSCP_FILL] << 8;
			measure_byte |= recieve_fifo_status_buffer[6 + SSCP_FILL] << 16;
			measure_byte |= recieve_fifo_status_buffer[7 + SSCP_FILL] << 24;
		  
			
			
			
			

			fill_byte[0] = response_byte & 0xff;
			fill_byte[1] = (response_byte >> 8) & 0xff;

			memcpy(message_buffer,&response_channel,1);
			memcpy(message_buffer + 1,fill_byte,5);
			transfer_byte = response_byte + SSCP_FILL;
			
		   
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
			HAL_SPI_TransmitReceive(&hspi1, message_buffer, recieve_response_buffer, transfer_byte,100);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	  }
		sscp_mac = SEND_MESSAGE;
		process_num=MEASURE_MODE;
		}
    }

		HeartPulse();
  	if (f10ms==1)
		{
			f10ms = 0;
		

		
		}
    }
}
void SendData()
{
	   unsigned char Txdata[32];
       unsigned char *pf;
       int i;
	   Txdata[0]=0x55;
		Txdata[1]=0xaa;
		Txdata[2]=0x01;
		Txdata[3]=FramLength;
		Txdata[4]=FramLength>>8;
		FramLength++;
		for(i=0;i<3;i++)
		{
			pf=(unsigned char*)&Gyro[i].value;
		    //pf=(unsigned char*)&GYRO_f[i];
			Txdata[5+i*4]=pf[0];
			Txdata[6+i*4]=pf[1];
		    Txdata[7+i*4]=pf[2];
			Txdata[8+i*4]=pf[3];
		}
        for(i=0;i<3;i++)
		{
			pf=(unsigned char*)&Acc[i].value;
			//pf=(unsigned char*)&ACC_f[i];
			Txdata[17+i*4]=pf[0];
			Txdata[18+i*4]=pf[1];
		    Txdata[19+i*4]=pf[2];
			Txdata[20+i*4]=pf[3];
		};
		Txdata[29]=(int16_t)Temperature.value;
		Txdata[30]=(int16_t)Temperature.value>>8;
		Txdata[31]=CheckSum8(&Txdata[2],29);
    	HAL_UART_Transmit_IT(&huart3,(uint8_t*)&Txdata[0],32);
}
unsigned char CheckSum8(unsigned char* pBuffer, int nLength)
{
	unsigned char retval = 0x00;
	int i;
	
	for(i=0;i<nLength;i++)
	{
		retval += pBuffer[i];
	}
	return retval;
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
	unsigned int transfer_byte = 0;
	unsigned int packet_length=37;
	int i=0;
	uTos data;
	if(process_num==MEASURE_MODE)
	{
		fill_byte[0] = ( (packet_length + 8) & 0xff);
		fill_byte[1] = ( ((packet_length + 8) >> 8) & 0xff);

		memcpy(message_buffer,&measure_channel,1);
		memcpy(message_buffer + 1,fill_byte,5);


		transfer_byte = packet_length + 8 + SSCP_FILL;

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	    HAL_SPI_TransmitReceive(&hspi1, message_buffer, recieve_message_buffer, transfer_byte,100);
		//HAL_SPI_TransmitReceive_DMA(&hspi1, message_buffer, recieve_message_buffer, transfer_byte);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);	
		sscp_mac = GET_MEASURE;

		Temperature.float_byte.high_byte=recieve_message_buffer[18];
		Temperature.float_byte.mhigh_byte=recieve_message_buffer[17];
		Temperature.float_byte.mlow_byte=recieve_message_buffer[16];
		Temperature.float_byte.low_byte=recieve_message_buffer[15];
		Temperature.value=Temperature.value*100;
/*deal with RAW dara*/	
//        data.c[0]=recieve_message_buffer[22];
//		data.c[1]=recieve_message_buffer[23];
//		ACC_f[0]=(float)data.s/32768*(1<<(2+1))*9.8;
//		data.c[0]=recieve_message_buffer[24];
//		data.c[1]=recieve_message_buffer[25];
//		ACC_f[1]=(float)data.s/32768*(1<<(2+1))*9.8;
//		data.c[0]=recieve_message_buffer[26];
//		data.c[1]=recieve_message_buffer[27];
//		ACC_f[2]=(float)data.s/32768*(1<<(2+1))*9.8;
//	    data.c[0]=recieve_message_buffer[31];
//		data.c[1]=recieve_message_buffer[32];
//		GYRO_f[0]=(float)data.s/1000*8.75*(1<<3);
//		data.c[0]=recieve_message_buffer[33];
//		data.c[1]=recieve_message_buffer[34];
//		GYRO_f[1]=(float)data.s/1000*8.75*(1<<3);
//		data.c[0]=recieve_message_buffer[35];
//		data.c[1]=recieve_message_buffer[36];
//		GYRO_f[2]=(float)data.s/1000*8.75*(1<<3);
/*deal with calibration dara*/		
		for(i=0;i<3;i++)
		{
		Acc[i].float_byte.high_byte=recieve_message_buffer[25+i*4];
		Acc[i].float_byte.mhigh_byte=recieve_message_buffer[24+i*4];
		Acc[i].float_byte.mlow_byte=recieve_message_buffer[23+i*4];
		Acc[i].float_byte.low_byte=recieve_message_buffer[22+i*4];
		Acc[i].value=Acc[i].value*9.8;
		}
		for(i=0;i<3;i++)
		{
		Gyro[i].float_byte.high_byte=recieve_message_buffer[40+i*4];
		Gyro[i].float_byte.mhigh_byte=recieve_message_buffer[39+i*4];
		Gyro[i].float_byte.mlow_byte=recieve_message_buffer[38+i*4];
		Gyro[i].float_byte.low_byte=recieve_message_buffer[37+i*4];
		}
		SendData();
   }
   else
   {
	    fill_byte[0] = 10;
		fill_byte[1] = 0;

		memcpy(message_buffer,&fifo_status_channel,1);
		memcpy(message_buffer + 1,fill_byte,5);


		transfer_byte = 10 + SSCP_FILL;

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive(&hspi1, message_buffer, recieve_fifo_status_buffer, transfer_byte,100);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
        sscp_mac = GET_FIFO_STATUS;	   
   }
	//sscp_mac = GET_MEASURE;
	
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
}
int GetMeasurnCount(void)
{
	int transfer_byte=0;
	fill_byte[0] = 10;
    fill_byte[1] = 0;
    
    memcpy(message_buffer,&fifo_status_channel,1);
    memcpy(message_buffer + 1,fill_byte,5);
    
    
    transfer_byte = 10 + SSCP_FILL;
    
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, message_buffer, recieve_message_buffer, transfer_byte,100);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}
void HeartPulse()
{
	static long counter = 0;
	counter++;
	if (counter>1000000)
	{
		counter = 0;
		LED_Toggle(LED1);
	}
}
