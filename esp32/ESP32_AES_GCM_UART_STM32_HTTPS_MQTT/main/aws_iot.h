/* Author: i fiksuari pas prog
 * Documentation date: 2026-07-27
 */

#ifndef MAIN_AWS_IOT_H_
#define MAIN_AWS_IOT_H_
#include <stdbool.h>


#define CONFIG_AWS_EXAMPLE_CLIENT_ID "ESP32_SECURE_FUSION"
void aws_iot_start(void);
void aws_iot_stop(void);
bool aws_iot_is_running(void);

#endif /* MAIN_AWS_IOT_H_ */
