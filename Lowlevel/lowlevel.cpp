#include <Lowlevel/lowlevel.hpp>
#include <FastMath/fast_math.hpp>

uint16_t SO1 = 0;
uint16_t SO2 = 0;
uint16_t ADC3Raw[3];
uint8_t adcIdx = 0;

uint8_t SPIDataRx[2];
uint8_t uartData;

FIFO *uartFIFO;
void (*Control)();

uint16_t bufferCount = 0;

uint8_t phaseOrder = 0;

uint32_t timer = 0;
uint8_t timerStatus = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM1)
	{
		__HAL_TIM_DISABLE_IT(&htim8, TIM_IT_UPDATE);
		__HAL_TIM_DISABLE(&htim8);
		__HAL_TIM_SET_COUNTER(&htim8, 0x0);
		__HAL_TIM_ENABLE_IT(&htim8, TIM_IT_UPDATE);
		__HAL_TIM_ENABLE(&htim8);
	}
	else if (htim->Instance == TIM8)
	{
		HAL_GPIO_TogglePin(TP1_GPIO_Port, TP1_Pin);

		if (phaseOrder)
		{
			SO1 = HAL_ADC_GetValue(&hadc1);
			SO2 = HAL_ADC_GetValue(&hadc2);
		}
		else
		{
			SO2 = HAL_ADC_GetValue(&hadc1);
			SO1 = HAL_ADC_GetValue(&hadc2);
		}

		ADC3Raw[adcIdx++] = HAL_ADC_GetValue(&hadc3);
		if (adcIdx >= 3)
			adcIdx = 0;

		Control();

		HAL_ADC_Start(&hadc1);
		HAL_ADC_Start(&hadc2);
		HAL_ADC_Start(&hadc3);

		HAL_GPIO_TogglePin(TP1_GPIO_Port, TP1_Pin);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_GPIO_WritePin(ControlBus_TXEN_GPIO_Port, ControlBus_TXEN_Pin, GPIO_PIN_RESET);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uartHandle)
{
	uartFIFO->buffer[uartFIFO->topIdx++] = uartData;

	if (uartFIFO->topIdx >= UART_FIFO_BUFFER_SIZE)
	{
		uartFIFO->topIdx = 0;
	}
	HAL_UART_Receive_DMA(&huart2, &uartData, 1);
}

void _SendPacket(uint8_t *packet, uint32_t size)
{
	HAL_GPIO_WritePin(ControlBus_TXEN_GPIO_Port, ControlBus_TXEN_Pin, GPIO_PIN_SET);
	HAL_UART_Transmit_IT(&huart2, packet, size);
}

void SetUartFIFO(FIFO *_uartFIFO)
{
	uartFIFO = _uartFIFO;
}

void StartUartInterrupt()
{
	HAL_UART_Receive_DMA(&huart2, &uartData, 1);
}

void StartOnBoardLED()
{
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}

void SetOnBoardLED(uint32_t duty)
{
	htim2.Instance->CCR4 = duty;
}

void SetControlFunc(void (*funcPtr)())
{
	Control = funcPtr;
}

void StartControlTimer()
{
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim8);
}

void StartInverterPWM()
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

void SetInverterPWMDuty(uint32_t aDuty, uint32_t bDuty, uint32_t cDuty)
{
	if (phaseOrder)
	{
		TIM1->CCR1 = aDuty;
		TIM1->CCR2 = bDuty;
		TIM1->CCR3 = cDuty;
	}
	else
	{
		TIM1->CCR1 = bDuty;
		TIM1->CCR2 = aDuty;
		TIM1->CCR3 = cDuty;
	}
}

uint8_t GetUartData()
{
	return uartData;
}

void StartADC()
{
	HAL_ADC_Start(&hadc1);
	HAL_ADC_Start(&hadc2);
	HAL_ADC_Start(&hadc3);
}

uint16_t GetSO1()
{
	return SO1;
}

uint16_t GetSO2()
{
	return SO2;
}

uint16_t GetDCVoltageRaw()
{
	return ADC3Raw[1];
}

uint16_t GetMotorTempRaw()
{
	return ADC3Raw[2];
}

uint16_t GetFETTempRaw()
{
	return ADC3Raw[0];
}

void OnGateDriver()
{
	HAL_GPIO_WritePin(EN_GATE_GPIO_Port, EN_GATE_Pin, GPIO_PIN_SET);
}

void OffGateDriver()
{
	HAL_GPIO_WritePin(EN_GATE_GPIO_Port, EN_GATE_Pin, GPIO_PIN_RESET);
}

uint8_t GateFault()
{
	return !HAL_GPIO_ReadPin(nFAULT_GPIO_Port, nFAULT_Pin);
}

void ADCCalibration(uint8_t status)
{
	if (status)
	{
		HAL_GPIO_WritePin(DC_CAL_GPIO_Port, DC_CAL_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(DC_CAL_GPIO_Port, DC_CAL_Pin, GPIO_PIN_RESET);
	}
}

void SetPhaseOrder(uint8_t _phaseOrder)
{
	phaseOrder = _phaseOrder;
}

void SPITransmitPool(uint8_t *dataTx, uint32_t len)
{
	HAL_GPIO_WritePin(Encoder_CS_GPIO_Port, Encoder_CS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Encoder_CS_GPIO_Port, Encoder_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi2, dataTx, SPIDataRx, len, 1);
}

void SPITransmit(uint8_t *dataTx, uint32_t len)
{
	HAL_GPIO_WritePin(Encoder_CS_GPIO_Port, Encoder_CS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Encoder_CS_GPIO_Port, Encoder_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive_IT(&hspi2, dataTx, SPIDataRx, len);
}

uint8_t* SPIReceive()
{
	return SPIDataRx;
}

void StartTimer()
{
	timer = 0;
	timerStatus = 1;
}

uint32_t GetTimerTick()
{
	return timer / 21;
}

void TimerUpdate()
{
	if (timerStatus)
	{
		timer++;
	}
}

void ResetTimer()
{
	timerStatus = 0;
	timer = 0;
}

uint32_t GetSector(uint32_t Address)
{
	uint32_t sector = 0;

	if ((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
	{
		sector = FLASH_SECTOR_0;
	}
	else if ((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
	{
		sector = FLASH_SECTOR_1;
	}
	else if ((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
	{
		sector = FLASH_SECTOR_2;
	}
	else if ((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
	{
		sector = FLASH_SECTOR_3;
	}
	else if ((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
	{
		sector = FLASH_SECTOR_4;
	}
	else if ((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
	{
		sector = FLASH_SECTOR_5;
	}
	else if ((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
	{
		sector = FLASH_SECTOR_6;
	}
	else
	{
		sector = FLASH_SECTOR_7;
	}

	return sector;
}

void UnLockEEPROM()
{
	FLASH_EraseInitTypeDef eraseInitStruct;
	uint32_t firstSector = 0;
	uint32_t nbOfSectors = 0;
	uint32_t sectorError = 0;

	firstSector = GetSector(ADDR_FLASH_SECTOR_7);
	nbOfSectors = GetSector(ADDR_FLASH_SECTOR_7) - firstSector + 1;

	eraseInitStruct.TypeErase = TYPEERASE_SECTORS;
	eraseInitStruct.VoltageRange = VOLTAGE_RANGE_3;
	eraseInitStruct.Sector = firstSector;
	eraseInitStruct.NbSectors = nbOfSectors;

	HAL_FLASH_Unlock();

	HAL_FLASHEx_Erase(&eraseInitStruct, &sectorError);
}

void WriteEEPROM(uint32_t address, uint32_t data)
{
	HAL_FLASH_Program(TYPEPROGRAM_WORD, ADDR_FLASH_SECTOR_7 + (address * 4), data);
}

uint32_t ReadEEPROM(uint32_t address)
{
	volatile uint32_t data = 0;
	uint32_t _address = ADDR_FLASH_SECTOR_7 + (address * 4);

	data = *(volatile uint32_t*) _address;

	return data;
}

void LockEEPROM()
{
	HAL_FLASH_Lock();
}

void SetUartBuadRate(uint32_t buadRate)
{
	  huart2.Instance = USART2;
	  huart2.Init.BaudRate = buadRate;
	  huart2.Init.WordLength = UART_WORDLENGTH_8B;
	  huart2.Init.StopBits = UART_STOPBITS_1;
	  huart2.Init.Parity = UART_PARITY_NONE;
	  huart2.Init.Mode = UART_MODE_TX_RX;
	  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	  HAL_UART_Init(&huart2);
}
