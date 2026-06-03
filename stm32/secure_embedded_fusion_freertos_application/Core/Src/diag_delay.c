/* Author: i fiksuari pas prog
 * Documentation date: 2026-05-28
 */

#include "diag_delay.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"

/* Delays diagnostic output with scheduler awareness. */
void diag_task_delay_ms(uint32_t delay_ms)
{
    /* Yield the task when the scheduler is running. */
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    } else {
        tim2_delay_ms(delay_ms);
    }
}
