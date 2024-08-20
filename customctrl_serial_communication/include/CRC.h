/**
 * @file CRC.h
 * @author gjc
 * @brief
 * @version 0.1
 * @date 2024-01-11
 *
 * @copyright USTC-RoboWalker (c) 2024
 *
 */

#ifndef CRC_H
#define CRC_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/
unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
