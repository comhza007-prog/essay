#include "CAN_receive.h"
#include "main.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

motor_measure_t motor_3508[2];
volatile uint32_t g_exp5_can_errors=0U;
volatile uint32_t g_exp5_can_rejected=0U;
extern volatile uint32_t g_exp5_drive_permit;

static HAL_StatusTypeDef CAN_SendData(CAN_HandleTypeDef *hcan,
                                      uint16_t std_id,
                                      uint8_t data[8])
{
    CAN_TxHeaderTypeDef tx_header = {
        .StdId = std_id,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = 8
    };
    uint32_t mail_box;
    uint8_t retry = 20U;

    while ((HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U) && (retry > 0U))
    {
        retry--;
    }
    if (retry == 0U)
    {
        return HAL_ERROR;
    }

    return HAL_CAN_AddTxMessage(hcan, &tx_header, data, &mail_box);
}

static void decode_motor_feedback(motor_measure_t *motor, const uint8_t data[8])
{
    int16_t delta;
    uint32_t cycles=DWT->CYCCNT,dt_cycles;
    uint16_t new_ecd = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);

    motor->speed_rpm = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    motor->given_current = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    motor->temperate = data[6];

    if (motor->rx_count == 0U)
    {
        motor->ecd = new_ecd;
        motor->last_ecd = new_ecd;
        motor->total_ecd = 0;
        motor->diag_encoder_rpm=0.0f;
    }
    else
    {
        motor->last_ecd = motor->ecd;
        motor->ecd = new_ecd;
        delta = (int16_t)motor->ecd - (int16_t)motor->last_ecd;

        if (delta < -4096)
        {
            delta += 8192;
        }
        else if (delta > 4096)
        {
            delta -= 8192;
        }
        motor->total_ecd += delta;
        dt_cycles=cycles-motor->diag_last_cycles;
        motor->diag_encoder_rpm=dt_cycles?
            (float)delta*(float)SystemCoreClock*60.0f/(8192.0f*(float)dt_cycles):0.0f;
    }

    motor->rotor_revolutions = motor->total_ecd / 8192.0f;
    motor->output_revolutions = motor->total_ecd / (8192.0f * 19.0f);
    motor->last_rx_ms = HAL_GetTick();
    motor->diag_last_cycles=cycles;
    motor->rx_count++;
}

void CAN_cmd_3508(int16_t M1, int16_t M2)
{
    uint8_t can_send_data[8] = {0U};

    /* Independent qualification guard: default disabled, bounds in counts. */
    if(M1>1200 || M1<-1200 || M2>1200 || M2<-1200) {
        g_exp5_can_errors++;g_exp5_drive_permit=0;
    }
    if(g_exp5_drive_permit!=1U) {M1=0;M2=0;}

    /* IDs 0x203 and 0x204 use slots M3 and M4 of command frame 0x200. */
    can_send_data[4] = (uint8_t)(M1 >> 8);
    can_send_data[5] = (uint8_t)M1;
    can_send_data[6] = (uint8_t)(M2 >> 8);
    can_send_data[7] = (uint8_t)M2;

    if(CAN_SendData(&hcan1, 0x200U, can_send_data)!=HAL_OK) g_exp5_can_errors++;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header={0};
    uint8_t rx_data[8]={0};

    /* These two motors are on CAN1. Ignore other buses/IDs, never decode
     * remote/short/extended frames as a complete motor measurement. */
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
    {
        if(hcan==&hcan1)g_exp5_can_rejected++;
        return;
    }
    /* Drain other buses without treating their measurements as these motors. */
    if(hcan!=&hcan1)return;

    if(rx_header.IDE!=CAN_ID_STD ||
       (rx_header.StdId!=0x203U && rx_header.StdId!=0x204U))return;
    if(rx_header.RTR!=CAN_RTR_DATA || rx_header.DLC!=8U ||
       (((uint32_t)rx_data[0]<<8)|(uint32_t)rx_data[1])>=8192U) {
        g_exp5_can_rejected++;
        return; /* old state not overwritten; task latches fault 4096 */
    }

    switch (rx_header.StdId)
    {
        case 0x203U:
            decode_motor_feedback(&motor_3508[0], rx_data);
            break;

        case 0x204U:
            decode_motor_feedback(&motor_3508[1], rx_data);
            break;

        default:
            break;
    }
}
