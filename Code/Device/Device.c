#include "main.h"
#include "dvc_Led.h"
#include "Device.h"
extern TIM_HandleTypeDef htim6;

extern UART_HandleTypeDef huart1; //!< <-> Main Port
extern UART_HandleTypeDef huart3; //!< <-> Debug Port
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi4;



LED_DEFINE_TABLE_BEGIN
{ "Live", GPIO_PIN_12,GPIOB },
{ "IN",   GPIO_PIN_13,GPIOB },
{ "OUT",  GPIO_PIN_14,GPIOB },
LED_DEFINE_TABLE_END

CONSTRUCT_CIRCULAR_BUFFER_MEMORY_POOL(g_USART1_Buffer, 1024);
CONSTRUCT_CIRCULAR_BUFFER_MEMORY_POOL(g_USART3_Buffer, 128);
CONSTRUCT_DOUBLE_BUFFER_MEMORY_POOL(g_Usart1OutputBuffer, 1024);


void InitDevice(void)
{	
	LED_TurnOnAll();
	HAL_Delay(1000);
//    HAL_GPIO_WritePin(RST_Saber_GPIO_Port, RST_Saber_Pin, GPIO_PIN_RESET);
//	HAL_Delay(1000);
//	HAL_GPIO_WritePin(RST_Saber_GPIO_Port, RST_Saber_Pin, GPIO_PIN_SET);
//	HAL_Delay(1000);
	HAL_TIM_Base_Start_IT(&htim6);

}

unsigned char f10ms = 0;
unsigned char IOflag= 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim6.Instance)
	{
		///@todo �����ﲹ���������
		f10ms = 1;

	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
	}
	else if (huart->Instance == USART3)
	{
	}
	else
	{
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (HAL_UART_GetError(huart) == HAL_UART_ERROR_ORE)
	{
		huart->RxState = HAL_UART_STATE_READY;
	}
	else if (HAL_UART_GetError(huart) == HAL_UART_ERROR_DMA)
	{
		huart->RxState = HAL_UART_STATE_READY;
	}
	else
	{
	}

	if (huart->Instance == USART1)
	{
	}
	else if (huart->Instance == USART3)
	{
	}
	else
	{
	}
}

//int fputc(int ch, FILE* f)
//{
//	HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 100);
//	return ch;
//}

unsigned char rxCpltFlag = 0;
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if(hspi->Instance == SPI1)
	{
		
	}
}

//void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
//{
//	if(hspi->Instance == SPI1)
//	{
//		
//	}
//}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	if(hspi->Instance == SPI1)
	{
	}
}
