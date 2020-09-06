#ifndef PROTOCOL_HPP_
#define PROTOCOL_HPP_

#include <Protocol/fifo.hpp>
#include <Lowlevel/lowlevel.hpp>
#include <Protocol/controlTable.hpp>
#include <cstring>
#include <stdio.h>

#define DXL_MAKEWORD(a, b)  ((uint16_t)(((uint8_t)(((uint64_t)(a)) & 0xff)) | ((uint16_t)((uint8_t)(((uint64_t)(b)) & 0xff))) << 8))
#define DXL_MAKEDWORD(a, b) ((uint32_t)(((uint16_t)(((uint64_t)(a)) & 0xffff)) | ((uint32_t)((uint16_t)(((uint64_t)(b)) & 0xffff))) << 16))
#define DXL_LOWORD(l)       ((uint16_t)(((uint64_t)(l)) & 0xffff))
#define DXL_HIWORD(l)       ((uint16_t)((((uint64_t)(l)) >> 16) & 0xffff))
#define DXL_LOBYTE(w)       ((uint8_t)(((uint64_t)(w)) & 0xff))
#define DXL_HIBYTE(w)       ((uint8_t)((((uint64_t)(w)) >> 8) & 0xff))

#define INST_PING           0x1
#define INST_READ           0x2
#define INST_WRITE          0x3
#define INST_REG_WRITE      0x4
#define INST_ACTION         0x5
#define INST_FACTORY_RESET  0x6
#define INST_SYNC_WRITE     0x83
#define INST_BULK_READ      0x92
#define INST_REBOOT         0x8
#define INST_CLEAR          0x10
#define INST_STATUS         0x55
#define INST_SYNC_READ      0x82
#define INST_BULK_WRITE     0x93

#define ERROR_NONE			0x00
#define ERROR_INST	0x02
#define ERROR_CRC 			0x03
#define ERROR_DATA_LENGTH	0x05
#define ERROR_ACCESS		0x07

#define PKT_HEADER0             0
#define PKT_HEADER1             1
#define PKT_HEADER2             2
#define PKT_RESERVED            3
#define PKT_ID                  4
#define PKT_LENGTH_L            5
#define PKT_LENGTH_H            6
#define PKT_INSTRUCTION         7
#define PKT_ERROR               8
#define PKT_PARAMETER0          8

#define RXPACKET_MAX_LEN    1024

class Protocol
{
public:
	FIFO uartFIFO = FIFO();
	ControlTable controlTable = ControlTable();
	void Update();

private:
	uint8_t rxPacket[RXPACKET_MAX_LEN];
	uint8_t txPacket[RXPACKET_MAX_LEN];

	uint32_t rxLength = 0;
	uint32_t waitLength = 10;
	uint32_t ReadPort(uint8_t *packet, uint32_t length);
	int8_t RxPacket();
	int8_t CheckID(uint8_t packetID);
	void RunInstruction();

	void InstPing();
	void InstRead();
	void InstWrite();

	int16_t txParamIdx = 0;
	void InitTx();
	void SetErrorCode(uint8_t errorCode);
	void AddTxParam(uint8_t data);
	void SendPacket();

	void AddStuffing(uint8_t *packet);
	void RemoveStuffing(uint8_t *packet);
	uint16_t CRC16(uint16_t crcAccum, uint8_t *dataBlkPtr, uint16_t dataBlkSize);

};

#endif
