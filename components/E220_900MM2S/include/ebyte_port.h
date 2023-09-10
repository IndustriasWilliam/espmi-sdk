#ifndef EBYTE_PORT_H_
#define EBYTE_PORT_H_

#include "stdint.h"

#include "ebyte_conf.h"


#define __weak	__attribute__((weak))


void Ebyte_Port_RstIoControl( uint8_t cmd );
void Ebyte_Port_TxenIoControl( uint8_t cmd );
void Ebyte_Port_RxenIoControl( uint8_t cmd );
void Ebyte_Port_DelayMs( uint32_t time );
void Ebyte_Port_SpiCsIoControl( uint8_t cmd );

uint8_t Ebyte_Port_BusyIoRead( void );
uint8_t Ebyte_Port_SpiTransmitAndReceivce( uint8_t send );


uint8_t Ebyte_BSP_SpiTransAndRecv(uint8_t data);
void Ebyte_BSP_RfSpiUnselected( void );
void Ebyte_BSP_RfSpiSelected( void );
void Ebyte_BSP_RfResetIoHigh( void );
void Ebyte_BSP_RfResetIoLow( void );
void Ebyte_BSP_RfTxIoEnable( void );
void Ebyte_BSP_RfTxIoDisable( void );
void Ebyte_BSP_RfRxIoEnable( void );
void Ebyte_BSP_RfRxIoDisable( void );
uint8_t Ebyte_BSP_RfBusyIoRead( void );
void Ebyte_Port_DelayMs( uint32_t time );


#endif
