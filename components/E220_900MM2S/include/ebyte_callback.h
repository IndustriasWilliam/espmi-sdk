#ifndef EBYTE_CALLBACK_H_
#define EBYTE_CALLBACK_H_

#include "ebyte_port.h"

void Ebyte_Port_TransmitCallback( uint16_t state );
void Ebyte_Port_ReceiveCallback(  uint16_t state ,uint8_t *buffer, uint8_t length);

#endif
