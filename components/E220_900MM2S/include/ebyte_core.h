#ifndef EBYTE_CORE_H_
#define EBYTE_CORE_H_


#include "ebyte_conf.h"

#if   defined(EBYTE_E22_400M22S)||defined(EBYTE_E22_900M22S)
#include "ebyte_e22x.h"

#elif defined(EBYTE_E220_400M22S)||defined(EBYTE_E220_900M22S)
#include "ebyte_e220x.h"

#elif defined(EBYTE_E10_400M20S)||defined(EBYTE_E10_900M20S) 
#include "ebyte_e10x.h"

#elif defined(EBYTE_E07_900M10S)
#include "ebyte_e07x.h"

#elif defined(EBYTE_E49_400M20S)||defined(EBYTE_E49_900M20S)
#include "ebyte_e49x.h"

#else
#error No product selected !
#endif

typedef struct
{
    void   ( *Init )( void );
    void   ( *Send )( uint8_t *buffer, uint8_t size , uint32_t timeout);
    void   ( *EnterSleepMode )( uint8_t command);
    void   ( *EnterReceiveMode )( uint32_t timeout );
    void   ( *StartPollTask)( void );
    void   ( *InterruptTrigger)( void );
    uint32_t ( *GetName ) (void );
    uint8_t ( *GetDriverVersion ) (void );
}Ebyte_RF_t; 

extern const Ebyte_RF_t Ebyte_RF;


#endif
