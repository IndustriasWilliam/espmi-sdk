#include <stdio.h>
#include "ebyte_callback.h"

/* !
 * @brief Ebyte_Port_TransmitCallback
 *
 * @param state
 *
 * @note E22x E220-900MM22S
 *         IRQ_TX_DONE                             = 0x0001,
 *         IRQ_RX_DONE                             = 0x0002,
 *         IRQ_PREAMBLE_DETECTED                   = 0x0004,
 *         IRQ_SYNCWORD_VALID                      = 0x0008,
 *         IRQ_HEADER_VALID                        = 0x0010,
 *         IRQ_HEADER_ERROR                        = 0x0020,
 *         IRQ_CRC_ERROR                           = 0x0040,
 *         IRQ_CAD_DONE                            = 0x0080,
 *         IRQ_CAD_ACTIVITY_DETECTED               = 0x0100,
 *         IRQ_RX_TX_TIMEOUT                       = 0x0200,
 */
__weak void Ebyte_Port_TransmitCallback( uint16_t state )
{
	printf("state %i\r\n", state);
    if( state &= 0x0001 )
    {
    }

    else if ( state &= 0x0200 )
    {

    }
    else
    {

        while( 1 );
    }
}

/* !
 * @brief Ebyte_Port_ReceiveCallback
 *
 * @param state
 *
 * @note E220-900MM22S
 *         IRQ_TX_DONE                             = 0x0001,
 *         IRQ_RX_DONE                             = 0x0002,
 *         IRQ_PREAMBLE_DETECTED                   = 0x0004,
 *         IRQ_SYNCWORD_VALID                      = 0x0008,
 *         IRQ_HEADER_VALID                        = 0x0010,
 *         IRQ_HEADER_ERROR                        = 0x0020,
 *         IRQ_CRC_ERROR                           = 0x0040,
 *         IRQ_CAD_DONE                            = 0x0080,
 *         IRQ_CAD_ACTIVITY_DETECTED               = 0x0100,
 *         IRQ_RX_TX_TIMEOUT                       = 0x0200,
 */
__weak void Ebyte_Port_ReceiveCallback(  uint16_t state, uint8_t *buffer, uint8_t length )
{

    if( state &= 0x0002 )
    {



    }
    else if ( state &= 0x0200 )
    {

    }
    else
    {

        while( 1 );
    }
}
