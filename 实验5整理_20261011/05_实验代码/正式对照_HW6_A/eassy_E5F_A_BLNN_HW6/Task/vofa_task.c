#include "vofa_task.h"
#include "motor_task.h"
#include "periodic_release.h"
#include <string.h>
static float values[VOFA_CHANNEL_COUNT];
static Vofa_t Vofa;
/* One persistent DMA buffer. Never overwritten until USART TC callback,
 * not merely DMA transfer-complete. No RTOS calls from UART callbacks. */
static volatile uint32_t tx_busy=0,tx_error=0;
static uint32_t tx_started=0,tx_task_errors=0,abort_failed=0;
volatile uint32_t g_exp5_dma_completed=0,g_exp5_dma_skipped=0;
volatile uint32_t g_exp5_uart_last_error=0;
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart) {
    if(uart==&huart6) {g_exp5_dma_completed++;__DMB();tx_busy=0;}
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart) {
    if(uart==&huart6) {g_exp5_uart_last_error=uart->ErrorCode;tx_error=1;}
}
void vofa_task(void const *argument) {
    TickType_t wake=xTaskGetTickCount();
    uint32_t sequence=0,rebase=0,now;
    HAL_StatusTypeDef status;
    (void)argument;Vofa_tail_init();
    for(;;) {
        now=HAL_GetTick();
        if(!abort_failed && (tx_error || (tx_busy && now-tx_started>=100U))) {
            /* Recovery is on the low-priority task, never in an ISR.
             * If abort fails the DMA buffer stays immutable until reset. */
            tx_task_errors++;
            status=HAL_UART_AbortTransmit(&huart6);
            if(status==HAL_OK) {tx_busy=0;tx_error=0;}
            else abort_failed=1;
        }
        g_exp5_uart_errors=tx_task_errors;
        if(tx_busy || abort_failed || huart6.gState!=HAL_UART_STATE_READY) {
            g_exp5_dma_skipped++;
        } else {
            e5_telemetry_snapshot(values);values[1]=(float)sequence;
            Vofa_send_justfloat(values);
            tx_busy=1;tx_started=now;__DMB();
            status=HAL_UART_Transmit_DMA(&huart6,Vofa.message,sizeof(Vofa.message));
            if(status!=HAL_OK) {
                tx_busy=0;tx_task_errors++;g_exp5_uart_errors=tx_task_errors;
                g_exp5_uart_last_error=huart6.ErrorCode;
            }
        }
        sequence++;
        wake=(TickType_t)e5_release_base((uint32_t)xTaskGetTickCount(),(uint32_t)wake,40U,&rebase);
        vTaskDelayUntil(&wake,40U);
    }
}
void Vofa_tail_init(void) {Vofa.tail[0]=0;Vofa.tail[1]=0;Vofa.tail[2]=0x80;Vofa.tail[3]=0x7f;}
void Vofa_send_justfloat(float *send_message) {
    memcpy(Vofa.message,send_message,VOFA_CHANNEL_COUNT*sizeof(float));
    memcpy(Vofa.message+VOFA_CHANNEL_COUNT*sizeof(float),Vofa.tail,4U);
}
