#ifndef EBYTE_E220X_H_
#define EBYTE_E220X_H_


#include "ebyte_conf.h"
#include "ebyte_callback.h"


#if defined(EBYTE_E220_400M22S)
#define RF_FREQUENCY_START                          410125000 // Hz
#define RF_FREQUENCY_CH_STEP                        1000000   // Hz
#define RF_FREQUENCY_CH                             23        // Hz    410.125 + 23 * 1 = 433.125MHz

/* EBYTE_E220_900M22S ģ�����ͨ��Ƶ��  */
#elif defined(EBYTE_E220_900M22S) 
#define RF_FREQUENCY_START                          902125000 // Hz
#define RF_FREQUENCY_CH_STEP                        1000000   // Hz
#define RF_FREQUENCY_CH                             13        // Hz    902.125 + 13 * 1 = 915.125MHz

#endif

#define RF_TX_OUTPUT_POWER                          22        // dBm

#define RF_RX_BUFFER_SIZE                           255 


#define RF_TX_RX_MODE                               0         // 0=LoRa  1=FSK
   

#define LORA_MAC_PUBLIC_SYNCWORD                    0x574B    // 0x3444
   

/* LoRa BPS Air 0=0.3k  1=1.2k  2=2.4k  3=4.8k  4=9.6k  5=19.5k  6=38.4k  7=62.5k */
#define RF_LORA_AIR_BPS                             2

#define FSK_PREAMBLE_LENGTH                         20   
    
#define FSK_MAC_PUBLIC_SYNCWORD_LENGTH              2 
#define FSK_MAC_PUBLIC_SYNCWORD_7                   0x57
#define FSK_MAC_PUBLIC_SYNCWORD_6                   0x4B
#define FSK_MAC_PUBLIC_SYNCWORD_5                   0x00
#define FSK_MAC_PUBLIC_SYNCWORD_4                   0x00                  
#define FSK_MAC_PUBLIC_SYNCWORD_3                   0x00 
#define FSK_MAC_PUBLIC_SYNCWORD_2                   0x00 
#define FSK_MAC_PUBLIC_SYNCWORD_1                   0x00    
#define FSK_MAC_PUBLIC_SYNCWORD_0                   0x00    
      
/**
 * ���ù�ϵ��
 * 
 *   DataRate          FrenquencyDeviation        BW
 *    4.8kbps                16KHz                31KHz
 *    9.6kbps                20KHz                31KHz
 *     20kbps                30KHz                50KHz
 *     40kbps                40KHz              62.5KHz
 *    100kbps               100KHz               166KHz
 */                                                               
#define RF_FSK_DATA_RATE                            4800     // 4.8kbps  
#define RF_FSK_FREQUENCY_DEVIATION                  16000    // 16k    
#define RF_FSK_BANDWIDTH                            31000    // 31k   

   
void Ebyte_E220x_Init( void );
void Ebyte_E220x_SetRfFrequency( uint32_t frequency );
void Ebyte_E220x_SetRx( uint32_t timeout );
void Ebyte_E220x_SendPayload( uint8_t *payload, uint8_t size, uint32_t timeout );
void Ebyte_E220x_ClearIrqStatus( uint16_t irq );
void Ebyte_E220x_ClearDeviceErrors( void );
void Ebyte_E220x_SendPayload( uint8_t *payload, uint8_t size, uint32_t timeout );
void Ebyte_E220x_IntOrPollTask( void );
void Ebyte_E220x_InterruptTrigger(void);
void Ebyte_E220x_SetSleep( uint8_t  cmd );
uint32_t Ebyte_E220x_GetName(void);
uint8_t Ebyte_E220x_GetDriverVersion(void);


#endif
