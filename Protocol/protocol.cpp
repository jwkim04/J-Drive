#include <Protocol/protocol.hpp>

uint16_t crc_table[256] = {
		0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
		0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
		0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
		0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
		0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
		0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
		0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
		0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
		0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
		0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
		0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
		0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
		0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
		0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
		0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
		0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
		0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
		0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
		0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
		0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
		0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
		0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
		0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
		0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
		0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
		0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
		0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
		0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
		0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
		0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
		0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
		0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
};

int a = 0;

void Protocol::Update()
{
	int8_t result = RxPacket();

	if (result == 1)
	{
		rxLength = 0;
		waitLength = 10;
		RunInstruction();
	}

	if (result == 2)
	{
		rxLength = 0;
		waitLength = 10;
	}

	else if (result == -1)
	{
		rxLength = 0;
		waitLength = 10;

		InitTx();
		SetErrorCode(ERROR_CRC);
		SendPacket();
	}
}

int8_t Protocol::CheckID(uint8_t packetID)
{
	uint32_t ID, secondaryID;

	controlTable.GetTable(2, &ID, 1);
	controlTable.GetTable(7, &secondaryID, 1);

	if (packetID == ID || packetID == secondaryID)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

void Protocol::RunInstruction()
{
	switch (rxPacket[PKT_INSTRUCTION])
	{
	case INST_PING:
		InstPing();
		break;

	case INST_READ:
		InstRead();
		break;

	case INST_WRITE:
		InstWrite();
		break;

	default:
		InitTx();
		SetErrorCode(ERROR_INST);
		SendPacket();
		break;
	}
}

void Protocol::InstPing()
{
	uint32_t modelNumber, FWVersion;

	controlTable.GetTable(0, &modelNumber, 2);
	controlTable.GetTable(1, &FWVersion, 1);

	InitTx();
	SetErrorCode(ERROR_NONE);

	AddTxParam(DXL_LOBYTE(modelNumber));
	AddTxParam(DXL_HIBYTE(modelNumber));
	AddTxParam(FWVersion);

	SendPacket();
}

void Protocol::InstRead()
{
	uint16_t address = DXL_MAKEWORD(rxPacket[PKT_PARAMETER0], rxPacket[PKT_PARAMETER0 + 1]);
	uint16_t dataLength = DXL_MAKEWORD(rxPacket[PKT_PARAMETER0 + 2], rxPacket[PKT_PARAMETER0 + 3]);
	uint32_t data;

	InitTx();
	if (controlTable.GetTable(address, &data, dataLength) == 0)
	{
		SetErrorCode(ERROR_NONE);

		switch (dataLength)
		{
		case 1:
			AddTxParam((uint8_t) data);
			break;

		case 2:
			AddTxParam((uint8_t) data);
			AddTxParam((uint8_t) (data >> 8));
			break;

		case 3:
			AddTxParam((uint8_t) data);
			AddTxParam((uint8_t) (data >> 8));
			AddTxParam((uint8_t) (data >> 16));
			break;

		case 4:
			AddTxParam((uint8_t) data);
			AddTxParam((uint8_t) (data >> 8));
			AddTxParam((uint8_t) (data >> 16));
			AddTxParam((uint8_t) (data >> 24));
			break;
		default:
			break;
		}

		SendPacket();
	}
}

void Protocol::InstWrite()
{
	uint16_t address = DXL_MAKEWORD(rxPacket[PKT_PARAMETER0], rxPacket[PKT_PARAMETER0 + 1]);
	uint16_t dataLength = DXL_MAKEWORD(rxPacket[PKT_LENGTH_L], rxPacket[PKT_LENGTH_H]) - 5;
	uint32_t data = 0;

	switch (dataLength)
	{
	case 1:
		data = rxPacket[PKT_PARAMETER0 + 2];
		break;

	case 2:
		data = rxPacket[PKT_PARAMETER0 + 2];
		data = ((uint32_t) rxPacket[PKT_PARAMETER0 + 3] << 8) | data;
		break;

	case 3:
		data = rxPacket[PKT_PARAMETER0 + 2];
		data = ((uint32_t) rxPacket[PKT_PARAMETER0 + 3] << 8) | data;
		data = ((uint32_t) rxPacket[PKT_PARAMETER0 + 4] << 16) | data;
		break;

	case 4:
		data = rxPacket[PKT_PARAMETER0 + 2];
		data = ((uint32_t) rxPacket[PKT_PARAMETER0 + 3] << 8) | data;
		data = ((uint32_t) rxPacket[PKT_PARAMETER0 + 4] << 16) | data;
		data = ((uint32_t) rxPacket[PKT_PARAMETER0 + 5] << 24) | data;
		break;

	default:
		break;
	}

	uint8_t result = controlTable.SetTable(address, data, dataLength);

	if (result == 2)
	{
		InitTx();
		SetErrorCode(ERROR_DATA_LENGTH);
		SendPacket();
	}
	else if (result == 1)
	{
		InitTx();
		SetErrorCode(ERROR_ACCESS);
		SendPacket();
	}
}

void Protocol::InitTx()
{
	uint32_t ID = rxPacket[PKT_ID];

	//controlTable.GetTable(2, &ID, 1);

	txParamIdx = 0;

	txPacket[PKT_HEADER0] = 0xFF;
	txPacket[PKT_HEADER1] = 0xFF;
	txPacket[PKT_HEADER2] = 0xFD;
	txPacket[PKT_RESERVED] = 0x00;
	txPacket[PKT_ID] = ID;
	txPacket[PKT_INSTRUCTION] = 0x55;
}

void Protocol::SetErrorCode(uint8_t errorCode)
{
	txPacket[PKT_ERROR] = errorCode;
}

void Protocol::AddTxParam(uint8_t data)
{
	txPacket[PKT_PARAMETER0 + 1 + txParamIdx++] = data;
}

void Protocol::SendPacket()
{
	uint16_t packetlength = txParamIdx + 4;

	txPacket[PKT_LENGTH_L] = DXL_LOBYTE(packetlength);
	txPacket[PKT_LENGTH_H] = DXL_HIBYTE(packetlength);

	AddStuffing(txPacket);

	uint16_t crc = CRC16(0, txPacket, 5 + DXL_MAKEWORD(txPacket[PKT_LENGTH_L], txPacket[PKT_LENGTH_H]));

	txPacket[5 + DXL_MAKEWORD(txPacket[PKT_LENGTH_L], txPacket[PKT_LENGTH_H])] = DXL_LOBYTE(crc);
	txPacket[6 + DXL_MAKEWORD(txPacket[PKT_LENGTH_L], txPacket[PKT_LENGTH_H])] = DXL_HIBYTE(crc);

	_SendPacket(txPacket, 7 + DXL_MAKEWORD(txPacket[PKT_LENGTH_L], txPacket[PKT_LENGTH_H]));
}

uint32_t Protocol::ReadPort(uint8_t *packet, uint32_t length)
{
	uint32_t rxLength = uartFIFO.topIdx - uartFIFO.bottomIdx;

	if (rxLength > length)
		rxLength = length;

	for (uint32_t i = 0; i < rxLength; i++)
	{
		packet[i] = uartFIFO.pop();
	}

	return rxLength;
}

int8_t Protocol::RxPacket()
{
	rxLength += ReadPort(&rxPacket[rxLength], waitLength - rxLength);

	if (rxLength >= waitLength)
	{
		uint32_t idx = 0;

		for (idx = 0; idx < (rxLength - 3); idx++)
		{
			if ((rxPacket[idx] == 0xFF) && (rxPacket[idx + 1] == 0xFF) && (rxPacket[idx + 2] == 0xFD) && (rxPacket[idx + 3] != 0xFD))
				break;
		}

		if (idx == 0)
		{
			if (rxPacket[PKT_RESERVED] != 0x00 || rxPacket[PKT_ID] > 0xFC || DXL_MAKEWORD(rxPacket[PKT_LENGTH_L], rxPacket[PKT_LENGTH_H]) > RXPACKET_MAX_LEN)
			{
				for (uint16_t s = 0; s < rxLength - 1; s++)
					rxPacket[s] = rxPacket[1 + s];
				rxLength -= 1;
			}

			if (!CheckID(rxPacket[PKT_ID]))
			{
				return 2;
			}

			if (waitLength != (uint32_t) (DXL_MAKEWORD(rxPacket[PKT_LENGTH_L], rxPacket[PKT_LENGTH_H]) + PKT_LENGTH_H + 1))
			{
				waitLength = DXL_MAKEWORD(rxPacket[PKT_LENGTH_L], rxPacket[PKT_LENGTH_H]) + PKT_LENGTH_H + 1;
			}
			else
			{
				uint16_t crc = DXL_MAKEWORD(rxPacket[waitLength - 2], rxPacket[waitLength - 1]);
				if (CRC16(0, rxPacket, waitLength - 2) == crc)
				{
					RemoveStuffing(rxPacket);
					return 1;
				}
				else
				{
					return -1;
				}
			}
		}
		else
		{
			memcpy(&rxPacket[0], &rxPacket[idx], rxLength - idx);
			rxLength -= idx;
		}
	}

	return 0;
}

void Protocol::AddStuffing(uint8_t *packet)
{
	int32_t packet_length_in = DXL_MAKEWORD(packet[PKT_LENGTH_L], packet[PKT_LENGTH_H]);
	int32_t packet_length_out = packet_length_in;

	if (packet_length_in < 8)
		return;

	uint8_t *packet_ptr;
	uint16_t packet_length_before_crc = packet_length_in - 2;
	for (uint16_t i = 3; i < packet_length_before_crc; i++)
	{
		packet_ptr = &packet[i + PKT_INSTRUCTION - 2];
		if (packet_ptr[0] == 0xFF && packet_ptr[1] == 0xFF && packet_ptr[2] == 0xFD)
			packet_length_out++;
	}

	if (packet_length_in == packet_length_out)
		return;

	uint16_t out_index = packet_length_out + 6 - 2;
	uint16_t in_index = packet_length_in + 6 - 2;
	while (out_index != in_index)
	{
		if (packet[in_index] == 0xFD && packet[in_index - 1] == 0xFF && packet[in_index - 2] == 0xFF)
		{
			packet[out_index--] = 0xFD;
			if (out_index != in_index)
			{
				packet[out_index--] = packet[in_index--];
				packet[out_index--] = packet[in_index--];
				packet[out_index--] = packet[in_index--];
			}
		}
		else
		{
			packet[out_index--] = packet[in_index--];
		}
	}

	packet[PKT_LENGTH_L] = DXL_LOBYTE(packet_length_out);
	packet[PKT_LENGTH_H] = DXL_HIBYTE(packet_length_out);

	return;
}

void Protocol::RemoveStuffing(uint8_t *packet)
{
	int32_t i = 0, index = 0;
	int32_t packetLengthIn = DXL_MAKEWORD(packet[PKT_LENGTH_L], packet[PKT_LENGTH_H]);
	int32_t packetLengthOut = packetLengthIn;

	index = PKT_INSTRUCTION;
	for (i = 0; i < packetLengthIn - 2; i++)
	{
		if (packet[i + PKT_INSTRUCTION] == 0xFD && packet[i + PKT_INSTRUCTION + 1] == 0xFD && packet[i + PKT_INSTRUCTION - 1] == 0xFF && packet[i + PKT_INSTRUCTION - 2] == 0xFF)
		{
			packetLengthOut--;
			i++;
		}
		packet[index++] = packet[i + PKT_INSTRUCTION];
	}
	packet[index++] = packet[PKT_INSTRUCTION + packetLengthIn - 2];
	packet[index++] = packet[PKT_INSTRUCTION + packetLengthIn - 1];

	packet[PKT_LENGTH_L] = DXL_LOBYTE(packetLengthOut);
	packet[PKT_LENGTH_H] = DXL_HIBYTE(packetLengthOut);
}

uint16_t Protocol::CRC16(uint16_t crcAccum, uint8_t *dataBlkPtr, uint16_t dataBlkSize)
{
	uint16_t i, j;

	for (j = 0; j < dataBlkSize; j++)
	{
		i = ((uint16_t) (crcAccum >> 8) ^ dataBlkPtr[j]) & 0xFF;
		crcAccum = (crcAccum << 8) ^ crc_table[i];
	}

	return crcAccum;
}
