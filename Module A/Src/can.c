/*
 * can.c
 *
 * Module A STM32
 * Bare-metal CAN
 *
 * PD0 : CAN1_RX
 * PD1 : CAN1_TX
 *
 * CAN Bitrate : 500 kbps
 */

#include "can.h"


/* ================= 주기 ================= */

#define MODULE_ANNOUNCE_PERIOD_MS    100U
#define PRESSURE_SEND_PERIOD_MS      100U


/* ================= 내부 상태 ================= */

static volatile ModuleLinkState_t module_link_state =
    eMODULE_LINK_WAIT;

static uint32_t last_announce_tick = 0U;
static uint32_t last_pressure_tick = 0U;


/* ================= 내부 함수 선언 ================= */

static void CAN1_GPIO_Init(void);
static void CAN1_Peripheral_Init(void);
static void CAN1_Filter_Init(void);
static void CAN1_RX_Interrupt_Init(void);

static uint8_t CAN1_TrySend(const CAN_Frame_t *frame);

static void CAN_ModuleA_SendAnnounce(void);
static void CAN_ModuleA_SendPressure(uint16_t pressure_raw);

static void CAN_ModuleA_HandleLinkResult(const CAN_Frame_t *frame);


/* ================= CAN 초기화 ================= */

void CAN_Init(void)
{
    CAN1_GPIO_Init();
    CAN1_Peripheral_Init();
    CAN1_Filter_Init();
    CAN1_RX_Interrupt_Init();

    module_link_state = eMODULE_LINK_WAIT;

    last_announce_tick = HAL_GetTick();
    last_pressure_tick = HAL_GetTick();
}


/* ================= GPIO / CAN Register ================= */

static void CAN1_GPIO_Init(void)
{
    /* GPIOD Clock Enable */
    RCC->AHB1ENR |= (1U << 3);

    /* PD0, PD1 = Alternate Function */
    GPIOD->MODER &= ~((3U << 0) |
                      (3U << 2));

    GPIOD->MODER |=  ((2U << 0) |
                      (2U << 2));

    /* No Pull */
    GPIOD->PUPDR &= ~((3U << 0) |
                      (3U << 2));

    /* High Speed */
    GPIOD->OSPEEDR |= ((3U << 0) |
                       (3U << 2));

    /* PD0, PD1 = AF9 CAN1 */
    GPIOD->AFR[0] &= ~((0xFU << 0) |
                       (0xFU << 4));

    GPIOD->AFR[0] |=  ((9U << 0) |
                       (9U << 4));
}


static void CAN1_Peripheral_Init(void)
{
    /* CAN1 Clock Enable */
    RCC->APB1ENR |= (1U << 25);

    /* Sleep mode 해제 */
    CAN1->MCR &= ~(1U << 1);

    while (CAN1->MSR & (1U << 1))
    {
    }

    /* Initialization mode 진입 */
    CAN1->MCR |= (1U << 0);

    while ((CAN1->MSR & (1U << 0)) == 0U)
    {
    }

    /*
     * PCLK1 = 42 MHz
     *
     * Prescaler = 6
     * BS1 = 11 tq
     * BS2 = 2 tq
     *
     * 42 MHz / 6 / (1 + 11 + 2)
     * = 500 kbps
     */
    CAN1->BTR =
        (5U  << 0)  |
        (10U << 16) |
        (1U  << 20) |
        (0U  << 24);

    /*
     * NART = 0
     * 자동 재전송 사용
     */
    CAN1->MCR &= ~((1U << 7) |
                   (1U << 6) |
                   (1U << 5) |
                   (1U << 4) |
                   (1U << 3) |
                   (1U << 2));

    /* Normal mode 진입 */
    CAN1->MCR &= ~(1U << 0);

    while (CAN1->MSR & (1U << 0))
    {
    }
}


static void CAN1_Filter_Init(void)
{
    /* Filter Init Mode */
    CAN1->FMR |= (1U << 0);

    /* Filter 0 Disable */
    CAN1->FA1R &= ~(1U << 0);

    /* Mask Mode */
    CAN1->FM1R &= ~(1U << 0);

    /* 32-bit Filter */
    CAN1->FS1R |= (1U << 0);

    /* FIFO0 선택 */
    CAN1->FFA1R &= ~(1U << 0);

    /* 모든 CAN ID 허용 */
    CAN1->sFilterRegister[0].FR1 = 0x00000000U;
    CAN1->sFilterRegister[0].FR2 = 0x00000000U;

    /* Filter 0 Enable */
    CAN1->FA1R |= (1U << 0);

    /* Filter Init 종료 */
    CAN1->FMR &= ~(1U << 0);
}


static void CAN1_RX_Interrupt_Init(void)
{
    /* FIFO0 Message Pending Interrupt Enable */
    CAN1->IER |= (1U << 1);

    NVIC_SetPriority(CAN1_RX0_IRQn, 5);
    NVIC_EnableIRQ(CAN1_RX0_IRQn);
}


/* ================= CAN 송신 ================= */

uint8_t CAN_Send(const CAN_Frame_t *frame)
{
    if (frame == NULL)
    {
        return 0U;
    }

    if (frame->dlc > 8U)
    {
        return 0U;
    }

    return CAN1_TrySend(frame);
}


static uint8_t CAN1_TrySend(const CAN_Frame_t *frame)
{
    uint32_t mailbox;
    uint32_t tdlr;
    uint32_t tdhr;

    /* 빈 Tx Mailbox 탐색 */
    if (CAN1->TSR & (1U << 26))
    {
        mailbox = 0U;
    }
    else if (CAN1->TSR & (1U << 27))
    {
        mailbox = 1U;
    }
    else if (CAN1->TSR & (1U << 28))
    {
        mailbox = 2U;
    }
    else
    {
        return 0U;
    }

    tdlr =
        ((uint32_t)frame->data[0] << 0)  |
        ((uint32_t)frame->data[1] << 8)  |
        ((uint32_t)frame->data[2] << 16) |
        ((uint32_t)frame->data[3] << 24);

    tdhr =
        ((uint32_t)frame->data[4] << 0)  |
        ((uint32_t)frame->data[5] << 8)  |
        ((uint32_t)frame->data[6] << 16) |
        ((uint32_t)frame->data[7] << 24);

    /* DLC */
    CAN1->sTxMailBox[mailbox].TDTR = frame->dlc;

    /* Data */
    CAN1->sTxMailBox[mailbox].TDLR = tdlr;
    CAN1->sTxMailBox[mailbox].TDHR = tdhr;

    /* Standard ID + TXRQ */
    CAN1->sTxMailBox[mailbox].TIR =
        ((uint32_t)(frame->std_id & 0x7FFU) << 21) |
        (1U << 0);

    return 1U;
}


/* ================= CAN 수신 IRQ ================= */

void CAN_RxIrqHandler(void)
{
    CAN_Frame_t rx_frame;

    while ((CAN1->RF0R & 0x3U) != 0U)
    {
        uint32_t rir;
        uint32_t rdtr;
        uint32_t rdlr;
        uint32_t rdhr;

        rir  = CAN1->sFIFOMailBox[0].RIR;
        rdtr = CAN1->sFIFOMailBox[0].RDTR;
        rdlr = CAN1->sFIFOMailBox[0].RDLR;
        rdhr = CAN1->sFIFOMailBox[0].RDHR;

        /* FIFO0 해제 */
        CAN1->RF0R |= (1U << 5);

        /* Extended Frame, Remote Frame 무시 */
        if ((rir & (1U << 2)) ||
            (rir & (1U << 1)))
        {
            continue;
        }

        rx_frame.std_id =
            (uint16_t)((rir >> 21) & 0x7FFU);

        rx_frame.dlc =
            (uint8_t)(rdtr & 0x0FU);

        if (rx_frame.dlc > 8U)
        {
            rx_frame.dlc = 8U;
        }

        rx_frame.data[0] = (uint8_t)(rdlr & 0xFFU);
        rx_frame.data[1] = (uint8_t)((rdlr >> 8) & 0xFFU);
        rx_frame.data[2] = (uint8_t)((rdlr >> 16) & 0xFFU);
        rx_frame.data[3] = (uint8_t)((rdlr >> 24) & 0xFFU);

        rx_frame.data[4] = (uint8_t)(rdhr & 0xFFU);
        rx_frame.data[5] = (uint8_t)((rdhr >> 8) & 0xFFU);
        rx_frame.data[6] = (uint8_t)((rdhr >> 16) & 0xFFU);
        rx_frame.data[7] = (uint8_t)((rdhr >> 24) & 0xFFU);

        /* Base의 승인 / 거절 결과 처리 */
        if (rx_frame.std_id == CAN_ID_LINK_RESULT)
        {
            CAN_ModuleA_HandleLinkResult(&rx_frame);
        }
    }
}


/* ================= Module A Protocol ================= */

/*
 * main while문에서 계속 호출
 *
 * 인증 전 :
 * - 100ms마다 Module ID 송신
 *
 * 승인 후 :
 * - 100ms마다 압력값 송신
 */
void CAN_ModuleA_Process(uint16_t pressure_raw)
{
    uint32_t now_tick;

    now_tick = HAL_GetTick();

    /* 인증 대기 상태 */
    if (module_link_state == eMODULE_LINK_WAIT)
    {
        if ((now_tick - last_announce_tick) >=
            MODULE_ANNOUNCE_PERIOD_MS)
        {
            last_announce_tick = now_tick;

            CAN_ModuleA_SendAnnounce();
        }
    }

    /* 인증 성공 상태 */
    else if (module_link_state == eMODULE_LINK_ACCEPTED)
    {
        if ((now_tick - last_pressure_tick) >=
            PRESSURE_SEND_PERIOD_MS)
        {
            last_pressure_tick = now_tick;

            CAN_ModuleA_SendPressure(pressure_raw);
        }
    }

    /* eMODULE_LINK_REJECTED */
    else
    {
        /* 데이터 전송 안 함 */
    }
}


/*
 * CAN ID : 0x110
 *
 * data[0]    : Module Type
 * data[1~4]  : Module ID
 */
static void CAN_ModuleA_SendAnnounce(void)
{
    CAN_Frame_t frame = {0};

    frame.std_id = CAN_ID_MODULE_ANNOUNCE;
    frame.dlc = 5U;

    frame.data[0] = (uint8_t)eMODULE_GENERAL;

    frame.data[1] = (uint8_t)(MODULE_A_ID & 0xFFU);
    frame.data[2] = (uint8_t)((MODULE_A_ID >> 8) & 0xFFU);
    frame.data[3] = (uint8_t)((MODULE_A_ID >> 16) & 0xFFU);
    frame.data[4] = (uint8_t)((MODULE_A_ID >> 24) & 0xFFU);

    (void)CAN_Send(&frame);
}


/*
 * CAN ID : 0x171
 *
 * data[0] : pressure_raw LSB
 * data[1] : pressure_raw MSB
 */
static void CAN_ModuleA_SendPressure(uint16_t pressure_raw)
{
    CAN_Frame_t frame = {0};

    frame.std_id = CAN_ID_GENERAL_PRESSURE;
    frame.dlc = 2U;

    frame.data[0] = (uint8_t)(pressure_raw & 0xFFU);
    frame.data[1] = (uint8_t)((pressure_raw >> 8) & 0xFFU);

    (void)CAN_Send(&frame);
}


/*
 * CAN ID : 0x140
 *
 * data[0] : Module Type
 * data[1] : accepted
 * data[2] : reason
 */
static void CAN_ModuleA_HandleLinkResult(const CAN_Frame_t *frame)
{
    if (frame->dlc < 3U)
    {
        return;
    }

    /* Module A용 응답인지 확인 */
    if (frame->data[0] != (uint8_t)eMODULE_GENERAL)
    {
        return;
    }

    if (frame->data[1] == 1U)
    {
        module_link_state = eMODULE_LINK_ACCEPTED;
    }
    else
    {
        module_link_state = eMODULE_LINK_REJECTED;
    }
}


/* ================= 상태 조회 ================= */

ModuleLinkState_t CAN_ModuleA_GetLinkState(void)
{
    return module_link_state;
}


uint8_t CAN_ModuleA_IsAccepted(void)
{
    if (module_link_state == eMODULE_LINK_ACCEPTED)
    {
        return 1U;
    }

    return 0U;
}
