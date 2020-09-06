#ifndef LOWLEVEL_HPP_
#define LOWLEVEL_HPP_

#include <stm32f4xx_hal.h>
#include <cstdint>
#include <tim.h>
#include <adc.h>
#include <spi.h>
#include <main.h>
#include <usart.h>
#include <Protocol/fifo.hpp>
#include <BoardConfig/board_config.h>

#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */

void StartOnBoardLED();
void SetOnBoardLED(uint32_t duty);

void SetControlFunc(void (*funcPtr)());
void StartUartInterrupt();
void StartControlTimer();
void StartInverterPWM();
void SetInverterPWMDuty(uint32_t aDuty, uint32_t bDuty, uint32_t cDuty);
void StartEEPROM();
void SetUartFIFO(FIFO * _uartFIFO);

void _SendPacket(uint8_t *packet, uint32_t size);
uint8_t GetUartData();

void StartADC();
uint16_t GetSO1();
uint16_t GetSO2();
uint16_t GetDCVoltageRaw();
uint16_t GetMotorTempRaw();
uint16_t GetFETTempRaw();

void OnGateDriver();
void OffGateDriver();
uint8_t GateFault();
void ADCCalibration(uint8_t status);

void SetPhaseOrder(uint8_t _phaseOrder);

void SPITransmitPool(uint8_t *dataTx, uint32_t len);
void SPITransmit(uint8_t *dataTx, uint32_t len);
uint8_t* SPIReceive();

void StartTimer();
uint32_t GetTimerTick();
void TimerUpdate();
void ResetTimer();

uint32_t GetSector(uint32_t Address);
void UnLockEEPROM();
void WriteEEPROM(uint32_t address, uint32_t data);
uint32_t ReadEEPROM(uint32_t address);
void LockEEPROM();

void SetUartBuadRate(uint32_t buadRate);

#endif
