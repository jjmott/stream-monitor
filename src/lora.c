/*

The sub-GHz radio can be set in LoRa, (G)MSK or (G)FSK transmit operation mode with the following steps:
1. Define the location of the transmit payload data in the data buffer, with Set_BufferBaseAddress().
2. Write the payload data to the transmit data buffer with Write_Buffer().
3. Select the packet type (generic or LoRa) with Set_PacketType().
4. Define the frame format with Set_PacketParams().
5. Define synchronization word in the associated packet type SUBGHZ_xSYNCR(n) with Write_Register().
6. Define the RF frequency with Set_RfFrequency().
7. Define the PA configuration with Set_PaConfig().
8. Define the PA output power and ramping with Set_TxParams().
9. Define the modulation parameters with Set_ModulationParams().
10. Enable TxDone and timeout interrupts by configuring IRQ with Cfg_DioIrq().
11. Start the transmission by setting the sub-GHz radio in TX mode with Set_Tx(). After the transmission is finished, the sub-GHz radio enters automatically the Standby mode.
12. Wait for sub-GHz radio IRQ interrupt and read interrupt status with Get_IrqStatus():
a) On a TxDone interrupt, the packet is successfully sent
b) On a timeout interrupt, the transmission is timeout.
13. Clear interrupt with Clr_IrqStatus().
14. Optionally, send a Set_Sleep() command to force the sub-GHz radio in Sleep mode.


void SUBGRF_Init( DioIrqHandler dioIrq );
void SUBGRF_SetBufferBaseAddress( uint8_t txBaseAddress, uint8_t rxBaseAddress );
void SUBGRF_WriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t size );
void SUBGRF_SetPacketType( RadioPacketTypes_t packetType );
void SUBGRF_SetPacketParams( PacketParams_t *packetParams );
uint8_t SUBGRF_SetSyncWord( uint8_t *syncWord );
void SUBGRF_SetRfFrequency( uint32_t frequency );
void SUBGRF_SetPaConfig( uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel, uint8_t paLut );
void SUBGRF_SetTxParams( uint8_t paSelect, int8_t power, RadioRampTimes_t rampTime );
void SUBGRF_SetModulationParams( ModulationParams_t *modulationParams );
void SUBGRF_SetDioIrqParams( uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask );
void SUBGRF_SetTx( uint32_t timeout );
uint16_t SUBGRF_GetIrqStatus( void );
void SUBGRF_ClearIrqStatus( uint16_t irq );
void SUBGRF_SetSleep( SleepParams_t sleepConfig );

*/

#include "radio_conf.h"
#include "radio_driver.h" 
#include "mw_log_conf.h"



void LoRaInit(void)
{

    RadioStatus_t Status;
RadioError_t Errors;

//DioIrqHandler dioIrq;
PacketParams_t PacketParams;
ModulationParams_t ModulationParams;
//PacketStatus_t PacketStatus;

    PacketParams.PacketType = PACKET_TYPE_LORA;
    PacketParams.Params.LoRa.CrcMode = LORA_CRC_ON;
    PacketParams.Params.LoRa.HeaderType = LORA_PACKET_EXPLICIT;
    PacketParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;
    PacketParams.Params.LoRa.PayloadLength = 4;
    PacketParams.Params.LoRa.PreambleLength = 2;

    ModulationParams.PacketType = PACKET_TYPE_LORA;
    ModulationParams.Params.LoRa.Bandwidth = LORA_BW_125;
    ModulationParams.Params.LoRa.CodingRate = LORA_CR_4_6;
    ModulationParams.Params.LoRa.LowDatarateOptimize = 0;
    ModulationParams.Params.LoRa.SpreadingFactor = LORA_SF7;




SUBGRF_Init( NULL) ;

SUBGRF_SetBufferBaseAddress( 0x00, 0x80 );

//SUBGRF_WriteBuffer( 0, &TXbuf[0], Size );

SUBGRF_SetPacketType( PacketParams.PacketType );

SUBGRF_SetPacketParams( &PacketParams );


//uint8_t SUBGRF_SetSyncWord( uint8_t *syncWord );


SUBGRF_SetRfFrequency( 900e6 );

//SUBGRF_SetPaConfig( 0x4, 0x7, 0, 0);


SUBGRF_SetTxParams( RFO_HP, 0x16, RADIO_RAMP_800_US );


SUBGRF_SetModulationParams( &ModulationParams );

Status =  SUBGRF_GetStatus();
Errors = SUBGRF_GetDeviceErrors();


/*

void SUBGRF_SetDioIrqParams( uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask );
void SUBGRF_SetTx( uint32_t timeout );
uint16_t SUBGRF_GetIrqStatus( void );
void SUBGRF_ClearIrqStatus( uint16_t irq );
void SUBGRF_SetSleep( SleepParams_t sleepConfig );

*/


}
