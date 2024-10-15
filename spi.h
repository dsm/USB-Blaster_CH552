
#pragma once
#include <stdint.h>
#include "ch554.h"

/********************************** (C) COPYRIGHT *******************************
* File Name          : SPI.H
* Author             : WCH
* Version            : V1.0
* Date               : 2017/07/05
* Description        : CH554 SPI主、从模式接口函数
注：片选有效时，从机会自动加载SPI0_S_PRE的预置值到发送移位缓冲区，所以最好可以在片选
有效前向SPI0_S_PRE寄存器写入预发值，或者在主机端丢弃首个接收字节，发送时注意主机会先 
取走SPI0_S_PRE里面的值产生一个S0_IF_BYTE中断。
如果片选从无效到有效，从机首先进行发送的话，最好把输出的首字节放到SPI0_S_PRE寄存器中；
如果已经处于片选有效的话，数据数据使用SPI0_DATA就可以
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
********************************************************************************/

// #define SPI_INTERRUPT   1

#define  SPI_CK_SET( n ) (SPI0_CK_SE = n)                                     //SPI时钟设置函数 

/*******************************************************************************
* Function Name  : SPIMasterModeSet( UINT8 mode ) 
* Description    : SPI主机模式初始化
* Input          : UINT8 mode						 
* Output         : None
* Return         : None
*******************************************************************************/
void SPIMasterModeSet(uint8_t mode);

/*******************************************************************************
* Function Name  : CH554SPIInterruptInit()
* Description    : CH554SPI中断初始化
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CH554SPIInterruptInit();

/*******************************************************************************
* Function Name  : CH554SPIMasterWrite(UINT8 dat)
* Description    : CH554硬件SPI写数据，主机模式
* Input          : UINT8 dat   数据
* Output         : None
* Return         : None
*******************************************************************************/
void CH554SPIMasterWrite(uint8_t dat);

/*******************************************************************************
* Function Name  : CH554SPIMasterRead( )
* Description    : CH554硬件SPI0读数据，主机模式
* Input          : None
* Output         : None
* Return         : UINT8 ret   
*******************************************************************************/
uint8_t CH554SPIMasterRead();

/*******************************************************************************
* Function Name  : SPISlvModeSet( ) 
* Description    : SPI从机模式初始化
* Input          : None						 
* Output         : None
* Return         : None
*******************************************************************************/
void SPISlvModeSet( );

/*******************************************************************************
* Function Name  : CH554SPISlvWrite(UINT8 dat)
* Description    : CH554硬件SPI写数据，从机模式
* Input          : UINT8 dat   数据
* Output         : None
* Return         : None
*******************************************************************************/
void CH554SPISlvWrite(uint8_t dat);

/*******************************************************************************
* Function Name  : CH554SPISlvRead( )
* Description    : CH554硬件SPI0读数据，从机模式
* Input          : None
* Output         : None
* Return         : UINT8 ret   
*******************************************************************************/
uint8_t CH554SPISlvRead();