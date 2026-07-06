#ifndef INC_CAN_H_
#define INC_CAN_H_

#include <stdint.h>

#include "main.h"


/* ================= CAN ID ================= */

/* 연결 / 인증 */
#define CAN_ID_MODULE_ANNOUNCE       0x110U   /* Module -> Base */
#define CAN_ID_LINK_RESULT           0x140U   /* Base -> Module */

/* Module A */
#define CAN_ID_GENERAL_PRESSURE      0x171U   /* Module A -> Base */


/* ================= Module 정보 ================= */

typedef enum
{
    eMODULE_NONE       = 0U,
    eMODULE_GENERAL    = 1U,
    eMODULE_COLD_CHAIN = 2U,
    eMODULE_UNKNOWN    = 0xFFU
} ModuleType_t;

#define MODULE_A_ID                  0x0000A001U


/* ================= 연결 상태 ================= */

typedef enum
{
    eMODULE_LINK_WAIT = 0U,
    eMODULE_LINK_ACCEPTED,
    eMODULE_LINK_REJECTED
} ModuleLinkState_t;


/* ================= CAN Frame ================= */

typedef struct
{
    uint16_t std_id;
    uint8_t  dlc;
    uint8_t  data[8];
} CAN_Frame_t;


/* ================= CAN Driver ================= */

void CAN_Init(void);

/* stm32f4xx_it.c의 CAN1_RX0_IRQHandler()에서 호출 */
void CAN_RxIrqHandler(void);

/* CAN Frame 직접 전송 */
uint8_t CAN_Send(const CAN_Frame_t *frame);


/* ================= Module A Protocol ================= */

/*
 * main while문에서 반복 호출
 *
 * pressure_raw
 * - 압력센서 ADC 또는 변환 전 raw 값
 * - 인증 성공 후 CAN 0x171로 전송
 */
void CAN_ModuleA_Process(uint16_t pressure_raw);

/* 현재 승인 상태 확인 */
ModuleLinkState_t CAN_ModuleA_GetLinkState(void);

uint8_t CAN_ModuleA_IsAccepted(void);

#endif /* INC_CAN_H_ */
