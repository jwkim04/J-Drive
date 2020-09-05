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

void StartOnBoardLED();
void SetOnBoardLED(uint32_t duty);

void SetControlFunc(void (*funcPtr)());
void StartUartInterrupt();
void StartControlTimer();
void StartInverterPWM();
void SetInverterPWMDuty(uint32_t aDuty, uint32_t bDuty, uint32_t cDuty);
void StartEEPROM();
void SetUartFIFO(FIFO * _uartFIFO);

void ReadEEPROM(uint16_t address, uint16_t *data);
void WriteEEPROM(uint16_t address, uint16_t data);

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

#endif
