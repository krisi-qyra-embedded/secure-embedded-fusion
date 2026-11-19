#ifndef GLCD_ST7789_H
#define GLCD_ST7789_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define ST7789_TFTWIDTH   240U
#define ST7789_TFTHEIGHT  320U

#define ST7789_BLACK       0x0000U
#define ST7789_RED         0xF800U
#define ST7789_GREEN       0x07E0U
#define ST7789_BLUE        0x001FU
#define ST7789_ORANGE      0xFD60U
#define ST7789_LIGHTGREEN  0x87F0U

HAL_StatusTypeDef ST7789_Init(void);
HAL_StatusTypeDef ST7789_FillScreen(uint16_t color);
HAL_StatusTypeDef ST7789_FillRect(uint16_t x, uint16_t y, uint16_t width,
                                 uint16_t height, uint16_t color);

/* Row-major RGB565 values. Returns after all SPI bytes have been sent. */
HAL_StatusTypeDef ST7789_DrawImage(uint16_t x, uint16_t y, uint16_t width,
                                  uint16_t height, const uint16_t *pixels);

/* Text coordinates are 6-by-10 pixel character cells. */
uint32_t ST7789_DrawString(uint16_t column, uint16_t row, const char *text,
                           uint16_t color);
void ST7789_Drawaxes(uint16_t axis_color, uint16_t bg_color,
                     const char *x_label, const char *y_label1,
                     uint16_t label1_color, const char *y_label2,
                     uint16_t label2_color, int32_t ymax, int32_t ymin);
void ST7789_PlotPoint(int32_t value, uint16_t color);
void ST7789_PlotIncrement(void);

#endif
