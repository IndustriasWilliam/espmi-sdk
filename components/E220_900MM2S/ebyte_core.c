#include "ebyte_core.h"

#if defined(EBYTE_E22_400M22S)||defined(EBYTE_E22_900M22S)
const Ebyte_RF_t Ebyte_RF =
{
  Ebyte_E22x_Init,
  Ebyte_E22x_SendPayload,
  Ebyte_E22x_SetSleep,
  Ebyte_E22x_SetRx,  
  Ebyte_E22x_IntOrPollTask,
  Ebyte_E22x_InterruptTrigger,
  Ebyte_E22x_GetName,
  Ebyte_E22x_GetDriverVersion
};

#elif defined(EBYTE_E220_400M22S)||defined(EBYTE_E220_900M22S)
const Ebyte_RF_t Ebyte_RF =
{
  Ebyte_E220x_Init,
  Ebyte_E220x_SendPayload,
  Ebyte_E220x_SetSleep,
  Ebyte_E220x_SetRx,  
  Ebyte_E220x_IntOrPollTask,
  Ebyte_E220x_InterruptTrigger,
  Ebyte_E220x_GetName,
  Ebyte_E220x_GetDriverVersion
};

#elif defined(EBYTE_E07_900M10S)
const Ebyte_RF_t Ebyte_RF =
{
  Ebyte_E07x_Init,
  Ebyte_E07x_SendPayload,
  0,
  Ebyte_E07x_SetRx,  
  Ebyte_E07x_IntOrPollTask,
  0
};

#elif defined(EBYTE_E10_400M20S)||defined(EBYTE_E10_900M20S)
const Ebyte_RF_t Ebyte_RF =
{
  Ebyte_E10x_Init,
  Ebyte_E10x_SendPayload,
  0,
  Ebyte_E10x_SetRx,  
  Ebyte_E10x_IntOrPollTask,
  0
};

#elif defined(EBYTE_E49_400M20S)||defined(EBYTE_E49_900M20S)
const Ebyte_RF_t Ebyte_RF =
{
  Ebyte_E49x_Init,
  Ebyte_E49x_SendPayload,
  0,
  Ebyte_E49x_SetRx,  
  Ebyte_E49x_IntOrPollTask,
  0
};

#else
#error No product selected !
#endif






