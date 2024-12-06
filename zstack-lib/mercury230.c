#include "mercury230.h"
#include "Debug.h"
#include "OSAL.h"
#include "OnBoard.h"
#include "hal_led.h"
#include "hal_uart.h"

#ifndef MERCURY_PORT
#define MERCURY_PORT HAL_UART_PORT_1
#endif

#define MERCURY230_CV_RESPONSE_LENGTH 15
#define MERCURY230_E_RESPONSE_LENGTH 19
#define MERCURY230_READY_RESPONSE_LENGTH 4
#define MERCURY230_REQUEST_LENGTH 6

static void Mercury230_StartStopData(uint8 serial_num, uint8 cmd);
static bool Mercury230_CheckReady(void);

static void Mercury230_RequestMeasure(uint8 serial_num, uint8 cmd);

static current_values_t Mercury230_ReadCurrentValues(uint8 cmd);

static uint32 Mercury230_ReadEnergy(uint8 cmd);

static uint16 MODBUS_CRC16( const unsigned char *buf, unsigned int len );

extern zclMercury_t mercury230_dev = {&Mercury230_StartStopData, &Mercury230_CheckReady, &Mercury230_RequestMeasure, &Mercury230_ReadCurrentValues, &Mercury230_ReadEnergy};



static void Mercury230_StartStopData(uint8 serial_num, uint8 cmd)
{
  if (cmd == 1) {
    uint8 readMercury[11]  = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  
    readMercury[0] = serial_num;
    readMercury[1] = cmd; 
    readMercury[2] = 0x01; // уровень доступа
    
    // пароль
    for (int i = 3; (i <= 9); i++) 
    {
      readMercury[i] = 0x01;
    }
    
    uint16 crc = MODBUS_CRC16(readMercury, 9);

    readMercury[9] = crc & 0xFF;
    readMercury[10] = (crc>>8) & 0xFF;

    HalUARTWrite(MERCURY_PORT, readMercury, sizeof(readMercury) / sizeof(readMercury[0])); 
    
    LREP("Mercury sent: ");
    for (int i = 0; (i < sizeof(readMercury) / sizeof(readMercury[0])); i++) 
    {
      LREP("0x%X ", readMercury[i]);
    }
    LREP("\r\n");
  }
  else {
    uint8 readMercury[4]  = {0x00, 0x00, 0x00, 0x00};
  
    readMercury[0] = serial_num;
    readMercury[1] = cmd; 
    
    uint16 crc = MODBUS_CRC16(readMercury, 2);

    readMercury[2] = crc & 0xFF;
    readMercury[3] = (crc>>8) & 0xFF;

    HalUARTWrite(MERCURY_PORT, readMercury, sizeof(readMercury) / sizeof(readMercury[0])); 
    
    LREP("Mercury sent: ");
    for (int i = 0; (i < sizeof(readMercury) / sizeof(readMercury[0])); i++) 
    {
      LREP("0x%X ", readMercury[i]);
    }
    LREP("\r\n");
  }
  
}


static bool Mercury230_CheckReady()
{
  uint8 response[MERCURY230_READY_RESPONSE_LENGTH] = {0x00, 0x00, 0x00, 0x00};

  HalUARTRead(MERCURY_PORT, (uint8 *)&response, sizeof(response) / sizeof(response[0]));

  LREP("Mercury received: ");
  for (int i = 0; i <= MERCURY230_READY_RESPONSE_LENGTH - 1; i++) 
  {
    LREP("0x%X ", response[i]);
  }
  LREP("\r\n");

  uint16 crc = MODBUS_CRC16(response, MERCURY230_READY_RESPONSE_LENGTH - 2);

  LREP("Real CRC: ");
  LREP("0x%X ", crc & 0xFF);
  LREP("0x%X\r\n", (crc>>8) & 0xFF);
  
  if (response[MERCURY230_READY_RESPONSE_LENGTH - 2] != (crc & 0xFF) || response[MERCURY230_READY_RESPONSE_LENGTH - 1] != ((crc>>8) & 0xFF)) {
    LREPMaster("Invalid response\r\n");
    HalUARTRead(MERCURY_PORT, (uint8 *)&response, sizeof(response) / sizeof(response[0]));
    return FALSE;
  }
  else if (response[1] != 0) {
    LREPMaster("Data error\r\n");
    HalUARTRead(MERCURY_PORT, (uint8 *)&response, sizeof(response) / sizeof(response[0]));
    return FALSE;
  }
  
  return TRUE;
}


void Mercury230_RequestMeasure(uint8 serial_num, uint8 cmd) 
{
  uint8 readMercury[MERCURY230_REQUEST_LENGTH]  = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  readMercury[0] = serial_num;
  readMercury[3] = cmd;
  if ((cmd > 0x00) & (cmd < 0x05)) {
    readMercury[1] = 0x05;
    readMercury[2] = 0x00;
  }
  else {
    readMercury[1] = 0x08;
    readMercury[2] = 0x16;
  }
  
  uint16 crc = MODBUS_CRC16(readMercury, 4);
  
  readMercury[4] = crc & 0xFF;
  readMercury[5] = (crc>>8) & 0xFF;

  HalUARTWrite(MERCURY_PORT, readMercury, sizeof(readMercury) / sizeof(readMercury[0])); 
  
  LREP("Mercury sent: ");
  for (int i = 0; (i < sizeof(readMercury) / sizeof(readMercury[0])); i++) 
  {
    LREP("0x%X ", readMercury[i]);
  }
  LREP("\r\n");
}


current_values_t Mercury230_ReadCurrentValues(uint8 cmd) 
{
  
  current_values_t result = {
    {MERCURY_INVALID_RESPONSE, MERCURY_INVALID_RESPONSE, MERCURY_INVALID_RESPONSE}, 
    {MERCURY_INVALID_RESPONSE, MERCURY_INVALID_RESPONSE, MERCURY_INVALID_RESPONSE}, 
    {MERCURY_INVALID_RESPONSE, MERCURY_INVALID_RESPONSE, MERCURY_INVALID_RESPONSE}
  };
  
  uint8 length;
  uint8 shift;
  
  uint8 response[MERCURY230_CV_RESPONSE_LENGTH] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  
  HalUARTRead(MERCURY_PORT, (uint8 *)&response, sizeof(response) / sizeof(response[0]));

  LREP("Mercury received: ");
  for (int i = 0; i <= MERCURY230_CV_RESPONSE_LENGTH - 1; i++) 
  {
    LREP("0x%X ", response[i]);
  }
  LREP("\r\n");
  
  if (cmd == REQ_POWER) {
    length = MERCURY230_CV_RESPONSE_LENGTH;
    shift = 3;
  }
  else {
    length = MERCURY230_CV_RESPONSE_LENGTH - 3;
    shift = 0;
  };
    
  
  uint16 crc = MODBUS_CRC16(response, length - 2);
  
  LREP("Real CRC: ");
  LREP("0x%X ", crc & 0xFF);
  LREP("0x%X\r\n", (crc>>8) & 0xFF);
  
  if (response[length - 2] != (crc & 0xFF) || response[length - 1] != ((crc>>8) & 0xFF)) {
    LREPMaster("Invalid response\r\n");
    HalUARTRead(MERCURY_PORT, (uint8 *)&response, sizeof(response) / sizeof(response[0]));
    return result;
  }

  switch (cmd) {  
  case REQ_VOLTAGE:
    for (int i = 0; i <= 2; i++) 
      result.Voltage[i] = response[1 + i * 3 + shift] * 0x10000 + response[3 + i * 3 + shift] * 0x100 + response[2 + i * 3 + shift];
    break;
  case REQ_CURRENT:
    for (int i = 0; i <= 2; i++) 
      result.Current[i] = response[1 + i * 3 + shift] * 0x10000 + response[3 + i * 3 + shift] * 0x100 + response[2 + i * 3 + shift];
    break;
  case REQ_POWER:
    for (int i = 0; i <= 2; i++) {
      uint32 power = (uint32)(response[1 + i * 3 + shift] & 0x0f) * 0x10000 + (uint32)response[3 + i * 3 + shift] * 0x100 + response[2 + i * 3 + shift];
      LREP("power %ld\r\n", power);
      result.Power[i] = (int16)(power / 100);
    }
  }
  return result;
}

uint32 Mercury230_ReadEnergy(uint8 cmd) 
{
    uint32 result = 0xFFFFFFFF;//MERCURY_INVALID_RESPONSE;
    
    uint8 response[MERCURY230_E_RESPONSE_LENGTH] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    HalUARTRead(MERCURY_PORT, (uint8 *)&response, sizeof(response) / sizeof(response[0]));

    LREP("Mercury received: ");
    for (int i = 0; i <= MERCURY230_E_RESPONSE_LENGTH - 1; i++) 
    {
      LREP("0x%X ", response[i]);
    }
    LREP("\r\n");
    
    uint16 crc = MODBUS_CRC16(response, MERCURY230_E_RESPONSE_LENGTH - 2);

    LREP("Real CRC: ");
    LREP("0x%X ", crc & 0xFF);
    LREP("0x%X\r\n", (crc>>8) & 0xFF);
    
    if (response[MERCURY230_E_RESPONSE_LENGTH - 2] != (crc & 0xFF) || response[MERCURY230_E_RESPONSE_LENGTH - 1] != ((crc>>8) & 0xFF)) {
        LREPMaster("Invalid response\r\n");
        HalUARTRead(MERCURY_PORT, (uint8 *)&response, sizeof(response) / sizeof(response[0]));
        return result;
    }

    result =  (uint32)response[2] * 0x1000000 + (uint32)response[1] * 0x10000 + (uint32)response[4] * 0x100 + (uint32)response[3];
    LREP("Result: %ld\r\n", result);

    return result;
}

static uint16 MODBUS_CRC16 ( const unsigned char *buf, unsigned int len )
{
	uint16 crc = 0xFFFF;
	unsigned int i = 0;
	char bit = 0;

	for( i = 0; i < len; i++ )
	{
		crc ^= buf[i];

		for( bit = 0; bit < 8; bit++ )
		{
			if( crc & 0x0001 )
			{
				crc >>= 1;
				crc ^= 0xA001;
			}
			else
			{
				crc >>= 1;
			}
		}
	}

	return crc;
}

