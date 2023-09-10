#include "ebyte_port.h"

__weak uint8_t Ebyte_BSP_SpiTransAndRecv(uint8_t data){

	(void)data;
	return 0;
}
__weak void Ebyte_BSP_RfSpiUnselected( void ){

}
__weak void Ebyte_BSP_RfSpiSelected( void ){

}
__weak void Ebyte_BSP_RfResetIoHigh( void ){

}
__weak void Ebyte_BSP_RfResetIoLow( void ){

}
__weak void Ebyte_BSP_RfTxIoEnable( void ){

}
__weak void Ebyte_BSP_RfTxIoDisable( void ){

}
__weak void Ebyte_BSP_RfRxIoEnable( void ){

}
__weak void Ebyte_BSP_RfRxIoDisable( void ){

}
__weak uint8_t Ebyte_BSP_RfBusyIoRead( void ){
	return 0;
}
__weak void Ebyte_Port_DelayMs( uint32_t time ){
	(void)time;
}




uint8_t Ebyte_Port_SpiTransmitAndReceivce( uint8_t send )
{
    uint8_t result = 0;
    
    result = Ebyte_BSP_SpiTransAndRecv( send );
    
    return result;
}


void Ebyte_Port_SpiCsIoControl( uint8_t cmd )
{
    if ( cmd == 1 )
    {
       Ebyte_BSP_RfSpiUnselected(); 
    }
    else
    {
       Ebyte_BSP_RfSpiSelected( );
    }
}


void Ebyte_Port_RstIoControl( uint8_t cmd )
{
    if ( cmd == 1 )
    {
        Ebyte_BSP_RfResetIoHigh();
    }
    else
    {
        Ebyte_BSP_RfResetIoLow();
    }
}


void Ebyte_Port_TxenIoControl( uint8_t cmd )
{
    if ( cmd == 1 )
    {
        Ebyte_BSP_RfTxIoEnable();
    }
    else
    {
        Ebyte_BSP_RfTxIoDisable();
    }
}


void Ebyte_Port_RxenIoControl( uint8_t cmd )
{
    if ( cmd == 1 )
    {
        Ebyte_BSP_RfRxIoEnable();
    }
    else
    {
        Ebyte_BSP_RfRxIoDisable();
    }
}


uint8_t Ebyte_Port_BusyIoRead( void )
{
    uint8_t result = 0 ;
    
    result = Ebyte_BSP_RfBusyIoRead();
    return result;
}





