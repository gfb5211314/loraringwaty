#ifndef __STMFLASH_H__
#define __STMFLASH_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32L0xx_hal.h"

/* 类型定义 ------------------------------------------------------------------*/
/* 宏定义 --------------------------------------------------------------------*/
/************************** STM32 内部 FLASH 配置 *****************************/
#define STM32_FLASH_SIZE        64 // 所选STM32的FLASH容量大小(单位为K)
#define STM32_FLASH_WREN        1    // stm32芯片内容FLASH 写入使能(0，禁用;1，使能)

/* 扩展变量 ------------------------------------------------------------------*/
/* 函数声明 ------------------------------------------------------------------*/
uint16_t STMFLASH_ReadHalfWord(uint32_t faddr);		  //读出半字

void STMFLASH_WriteLenByte(uint32_t WriteAddr, uint32_t DataToWrite, uint16_t Len );	      //指定地址开始写入指定长度的数据
uint16_t STMFLASH_ReadLenByte(uint32_t ReadAddr, uint16_t Len );					                    	//指定地址开始读取指定长度数据
void STMFLASH_Write( uint32_t WriteAddr, uint32_t * pBuffer, uint32_t NumToWrite );		//从指定地址开始写入指定长度的数据
void STMFLASH_Read( uint32_t ReadAddr, uint32_t * pBuffer, uint32_t NumToRead );   	//从指定地址开始读出指定长度的数据




#endif /* __STMFLASH_H__ */

/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
