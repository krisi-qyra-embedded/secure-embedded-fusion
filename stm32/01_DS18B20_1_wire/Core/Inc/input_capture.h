/* input_capture.h | Author: i fiksuari pas prog | Date: 22 February 2026 */

#ifndef INC_INPUT_CAPTURE_H_
#define INC_INPUT_CAPTURE_H_

#include"stm32f4xx.h"
#include"tim.h"

void tim3_input_capture_PA6(void);
void tim2_output_compare_PA5(void);
#define sys_clck_freq 96000000
#endif /* INC_INPUT_CAPTURE_H_ */
