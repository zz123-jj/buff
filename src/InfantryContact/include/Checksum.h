#ifndef _CRC_CHECK_H_
#define _CRC_CHECK_H_

#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus  */

unsigned char Crc8Checksum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8);
unsigned int Crc8Verify(unsigned char *pchMessage, unsigned int dwLength);
void Crc8Append(unsigned char *pchMessage, unsigned int dwLength);
uint16_t Crc16Checksum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
uint32_t Crc16Verify(uint8_t *pchMessage, uint32_t dwLength);
void Crc16Append(uint8_t * pchMessage, uint32_t dwLength);

#ifdef __cplusplus
}
#endif /* __cplusplus  */

#endif
