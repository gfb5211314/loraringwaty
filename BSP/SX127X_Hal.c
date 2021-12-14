/**
  ******************************************************************************
  * �ļ��� ��   SX127X_HAL.c
  * ����   ��   LSD RF Team
  * �汾   ��   V1.0.0
  * ʱ��   ��   15-Aug-2018
  * �ļ�������
  *     ���ļ�ΪSX127Xģ���Ӳ���㣬����MCU��SX127Xģ���SPI���ã�GPIO�ڳ�ʼ������
  *������SX127X�Ĵ�����FIFO��д������
  *    �ͻ���ʹ��ģ��ʱ����Ҫ��ֲ���ļ�����֤�����������������ββ���������
  *�����Լ���MCUƽ̨�޸ĺ������ݣ�ʹ�������ܿ��������С�Ӳ����ռ����Դ���£�
	*
  *SPI��������ʹ��STM32L4��SPI3������SX127Xģ��ͨ�š�
  *GPIO�ڣ�������ʹ�õ�GPIO���������£�
  *        PB1  ---> DIO0
  *        PC4  ---> DIO1
  *        PB2  ---> DIO2
  *        PB0  ---> TXE
  *        PC5  ---> RXE
	*        PA7  ---> RST
	*        PA15 ---> NSS
  *        PC12 ---> M0SI
  *        PC11 ---> MISO
  *        PC10 ---> SCK
*******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "SX127X_Hal.h"
#include "stm32l0xx_hal.h"
#include "main.h"
extern SPI_HandleTypeDef hspi1;
//-----------------------------GPIO-----------------------------//
//�ò��ֺ���Ϊϵͳ�õ���GPIO�ĳ�ʼ���������û������Լ���ƽ̨��Ӧ�޸�
//--------------------------------------------------------------//

/**
  * @��飺�ú���ΪDIO0�����ʼ�����жϡ����ȼ����ã�
  * @��������
  * @����ֵ����
  */
void SX127X_DIO0_INPUT()
{
    GPIO_InitTypeDef GPIO_InitStruct;
    __GPIOB_CLK_ENABLE();
//    GPIO_InitStruct.Pin = GPIO_PIN_1;
//    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
//    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//    HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
//	 GPIO_InitStruct.Pin = DIO0_Pin|DIO1_Pin|DIO2_Pin;
//   GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//   GPIO_InitStruct.Pull = GPIO_NOPULL;
//   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
/**
  * @��飺�ú���ΪDIO0�����жϿ���ʹ�ܣ�
  * @��������
  * @����ֵ����
  */
void SX127X_DIO0_INTENABLE()
{
//    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}
/**
  * @��飺�ú���ΪDIO0�����жϹر�ʹ�ܣ�
  * @��������
  * @����ֵ����
  */
void SX127X_DIO0_INTDISABLE()
{
//    HAL_NVIC_DisableIRQ(EXTI1_IRQn);
}
/**
  * @��飺�ú���ΪDIO0����״̬��ȡ��
  * @��������
  * @����ֵ��DIO0״̬"1"or"0"
  */
GPIO_PinState SX127X_DIO0_GetState()
{
    GPIO_PinState State;
    State = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
    return State;
}
/**
  * @��飺�ú���ΪDIO1�����ʼ�����жϡ����ȼ����ã�
  * @��������
  * @����ֵ����
  */
void SX127X_DIO1_INPUT()
{
    GPIO_InitTypeDef GPIO_InitStruct;
    __GPIOB_CLK_ENABLE();
//    GPIO_InitStruct.Pin = GPIO_PIN_4;
//    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
//    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    //HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 0);
//	 GPIO_InitStruct.Pin = DIO0_Pin|DIO1_Pin|DIO2_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
/**
  * @��飺�ú���ΪDIO2�����ʼ�����жϡ����ȼ����ã�
  * @��������
  * @����ֵ����
  */
void SX127X_DIO2_INPUT()
{
    GPIO_InitTypeDef GPIO_InitStruct;
    __GPIOB_CLK_ENABLE();
//    GPIO_InitStruct.Pin = GPIO_PIN_2;
//    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
//    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    //HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 0);
//	 GPIO_InitStruct.Pin = DIO0_Pin|DIO1_Pin|DIO2_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
/**
  * @��飺�ú���Ϊ��Ƶ����TXE������ƣ�
  * @������PinStateΪ"1"��ʾ����ߵ�ƽ��"0"����͵�ƽ��
  * @����ֵ����
  */
void SX127X_TXE_OUTPUT(GPIO_PinState PinState)
{
//    GPIO_InitTypeDef GPIO_InitStruct;
//    __HAL_RCC_GPIOB_CLK_ENABLE();
//    GPIO_InitStruct.Pin = GPIO_PIN_0;
//    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//    GPIO_InitStruct.Pull = GPIO_PULLUP;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, PinState);
}
/**
  * @��飺�ú���Ϊ��Ƶ����RXE������ƣ�
  * @������PinStateΪ"1"��ʾ����ߵ�ƽ��"0"����͵�ƽ��
  * @����ֵ����
  */
void SX127X_RXE_OUTPUT(GPIO_PinState PinState)
{
//    GPIO_InitTypeDef GPIO_InitStruct;
//    __HAL_RCC_GPIOC_CLK_ENABLE();
//    GPIO_InitStruct.Pin = GPIO_PIN_5;
//    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//    GPIO_InitStruct.Pull = GPIO_PULLUP;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, PinState);
}
/**
  * @��飺�ú���ΪSPI��Ƭѡ����NSS������ƣ�
  * @������PinStateΪ"1"��ʾ����ߵ�ƽ��"0"����͵�ƽ��
  * @����ֵ����
  */
void SX127X_NSS_OUTPUT(GPIO_PinState PinState)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, PinState);
}
/**
  * @��飺�ú���ΪSX127X��λ����NRST������ƣ�
  * @������PinStateΪ"1"��ʾ����ߵ�ƽ��"0"����͵�ƽ��
  * @����ֵ����
  */
void SX127X_RESET_OUTPUT(GPIO_PinState PinState)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    __HAL_RCC_GPIOB_CLK_ENABLE();
//    GPIO_InitStruct.Pin = GPIO_PIN_7;
//    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//    GPIO_InitStruct.Pull = GPIO_PULLUP;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, PinState);
//	GPIO_InitStruct.Pin = LORA_RST_Pin|LED_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
uint8_t DIO0_STATE()
{
	
	   GPIO_PinState State;
    State = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
    return State;
}
//-----------------------------SPI-----------------------------//
//�ò��ֺ���ΪMCU��SX127Xģ��SPIͨ�Ų��֣�����SPI�ڼ����ó�ʼ��
//--------------------------------------------------------------//

/**
  * @��飺�ú�������MCU��SPI��ӦIO�ڳ�ʼ����
  * @��������
  * @����ֵ����
  */
//void SX127X_SPIGPIO_Init()
//{
//    GPIO_InitTypeDef GPIO_InitStruct;
//    /* Configure the SX126X_NSS pin */
//    GPIO_InitStruct.Pin = GPIO_PIN_15;
//    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
//    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//    /* SPI SCK GPIO pin configuration  */
//    GPIO_InitStruct.Pin       = GPIO_PIN_10;
//    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull      = GPIO_PULLUP;
//    GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
//    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
//    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

//    /* SPI MISO GPIO pin configuration  */
//    GPIO_InitStruct.Pin = GPIO_PIN_11;
//    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
//    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//    /* SPI MoSi GPIO pin configuration  */
//    GPIO_InitStruct.Pin       = GPIO_PIN_12;
//    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull      = GPIO_PULLUP;
//    GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
//    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
//    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//}
/**
  * @��飺�ú�������MCU��SPI���ó�ʼ����
  * @��������
  * @����ֵ����
  */
//void SX127X_SPI_Init()
//{
//    __HAL_RCC_GPIOA_CLK_ENABLE();//PORTAʱ��ʹ��
//    __HAL_RCC_GPIOC_CLK_ENABLE();//PORTCʱ��ʹ��
//    __HAL_RCC_SPI3_CLK_ENABLE();//SPI2ʱ��ʹ��
//    SX127X_SPIGPIO_Init();

//    SPI3_InitStruct.Instance = SPI3; //ʹ��SPI3
//    SPI3_InitStruct.Init.Mode = SPI_MODE_MASTER;//SPIģʽ������ģʽ
//    SPI3_InitStruct.Init.Direction = SPI_DIRECTION_2LINES;//����ȫ˫��
//    SPI3_InitStruct.Init.DataSize = SPI_DATASIZE_8BIT;//���ݿ��ȣ�8λ
//    SPI3_InitStruct.Init.CLKPolarity = SPI_POLARITY_LOW; //����ͬ����ʱ�ӿ���Ϊ����ʱ��
//    SPI3_InitStruct.Init.CLKPhase = SPI_PHASE_1EDGE;      //CPOL=0;CPHA=0ģʽ
//    SPI3_InitStruct.Init.NSS = SPI_NSS_SOFT;//NSSD����������
//    SPI3_InitStruct.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;//�����ʷ�Ƶ��8��Ƶ
//    SPI3_InitStruct.Init.FirstBit = SPI_FIRSTBIT_MSB;//���ݴ�MSB��ʼ
//    SPI3_InitStruct.Init.TIMode = SPI_TIMODE_DISABLE;//SPI Motorola mode
//    SPI3_InitStruct.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;//CRCУ�鲻ʹ��
//    SPI3_InitStruct.Init.CRCPolynomial = 7;//CRCֵ����Ķ���ʽ
//    SPI3_InitStruct.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
//    SPI3_InitStruct.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;

//    if (HAL_SPI_Init(&SPI3_InitStruct) != HAL_OK)

//    {
//        while(1);
//    }
//    __HAL_SPI_ENABLE(&SPI3_InitStruct);
//}

//-----------------------SX127X Read and Write-------------------//
//�ò��ֺ���ΪMCU��SX127Xģ��Ĵ������ж�д
//--------------------------------------------------------------//

/**
  * @��飺SX127X  ��Ĵ�����ַ������������
  * @������uint8_t addr,�Ĵ�����ַ uint8_t *buffer,��������ָ�� uint8_t sizeָ�볤��
  * @����ֵ����
  */
unsigned char SX127X_ReadWriteByte(unsigned char data)
{
    unsigned char RxDat;
    HAL_SPI_TransmitReceive(&hspi1, &data, &RxDat, 1, 1000);
    return RxDat;
}
/**
  * @��飺SX127X  ��Ĵ�����ַ������������
  * @������uint8_t addr,�Ĵ�����ַ uint8_t *buffer,��������ָ�� uint8_t sizeָ�볤��
  * @����ֵ����
  */
void SX127X_WriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;
    SX127X_NSS_OUTPUT(GPIO_PIN_RESET);
    SX127X_ReadWriteByte(addr | 0x80);
    for( i = 0; i < size; i++ )
    {
        SX127X_ReadWriteByte(buffer[i]);
    }
    SX127X_NSS_OUTPUT(GPIO_PIN_SET);
}
/**
  * @��飺SX127X  ��Ĵ�����ַ����������
  * @������uint8_t addr,�Ĵ�����ַ uint8_t *buffer,��������ָ�� uint8_t sizeָ�볤��
  * @����ֵ�����ݷ��ص�*buffer��
  */
void SX127X_ReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;
    SX127X_NSS_OUTPUT(GPIO_PIN_RESET);
    SX127X_ReadWriteByte(addr & 0x7F);
    for( i = 0; i < size; i++ )
    {
        buffer[i] = SX127X_ReadWriteByte(0x00);
    }

    SX127X_NSS_OUTPUT(GPIO_PIN_SET);
}


