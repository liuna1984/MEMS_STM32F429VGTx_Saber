#include "dvcSaber.h"
unsigned char process_num = NONE_MODE;/*this variable is the state of routine*/
unsigned char sscp_mac = NONE_SSCP;/*this variable is the state of sscp*/

/*below is our channel of sscp*/
unsigned char measure_channel  = 0xa1;
unsigned char message_channel  = 0xa2;
unsigned char fifo_status_channel  = 0xa3;
unsigned char response_channel  = 0xa4;
unsigned char clear_channel  = 0xa5;
/*below is buffer of each channel*/
unsigned char message_buffer[1024] = {0};
unsigned char recieve_message_buffer[1024] = {0};
unsigned char recieve_fifo_status_buffer[10 + SSCP_FILL] = {0};
unsigned char recieve_response_buffer[1024] = {0};
unsigned char recieve_measure_buffer[1024] = {0};

unsigned char fill_byte[5] = {0,0,0xf0,0xf1,0xf2};

unsigned int response_byte = 0;/*this variable is used to load value of unread_bytes_response in fifo_status*/
unsigned int measure_byte = 0; /*this variable is used to load value of unread_bytes_measure in fifo_status*/

unsigned int packet_length = 0;/*this variable is used to load the payload length of measure packet*/

unsigned char send_over = 0;

/*below is the array of each message*/
unsigned char switch_config[512] = {0};
unsigned char set_packet_config[512] = {0};
unsigned char set_odr[512] = {0};
unsigned char get_packet_config[512] = {0};
unsigned char switch_measure[512] = {0};

unsigned char set_yaw_flag = 0;

unsigned int counter = 10;