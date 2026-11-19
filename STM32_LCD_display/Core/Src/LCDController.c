#include "LCDController.h"
#include "GLCD_ST7735.h" /* Your existing header declares the ST7789 functions. */
#include "lvgl.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MY_DISP_HOR_RES 240U
#define MY_DISP_VER_RES 320U
#define DRAW_BUFFER_ROWS 20U

#if LVGL_VERSION_MAJOR != 8 || LVGL_VERSION_MINOR != 3
#error "This display port requires LVGL 8.3.x."
#endif

#if LV_COLOR_DEPTH != 16 || LV_COLOR_16_SWAP != 0
#error "This ST7789 driver needs RGB565 with LV_COLOR_16_SWAP set to 0."
#endif

static lv_color_t draw_buffer[MY_DISP_HOR_RES * DRAW_BUFFER_ROWS];
static lv_disp_draw_buf_t draw_buf_dsc;
static lv_disp_drv_t disp_drv;
static bool disp_flush_enabled = true;

static void disp_flush(lv_disp_drv_t *driver,
                       const lv_area_t *area,
                       lv_color_t *pixels)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (disp_flush_enabled) {
        const uint16_t width = (uint16_t)(area->x2 - area->x1 + 1);
        const uint16_t height = (uint16_t)(area->y2 - area->y1 + 1);

        status = ST7789_DrawImage((uint16_t)area->x1,
                                  (uint16_t)area->y1,
                                  width, height,
                                  (const uint16_t *)pixels);
    }

    /* Blocking SPI has finished accessing the buffer before this call. */
    lv_disp_flush_ready(driver);

    if (status != HAL_OK) {
        for (;;) {
            /* Set a breakpoint here to diagnose an SPI transfer failure. */
        }
    }
}

HAL_StatusTypeDef lv_port_disp_init(void)
{
    HAL_StatusTypeDef status = ST7789_Init();

    if (status != HAL_OK) {
        return status;
    }

    /* LVGL 8 expects the buffer size in PIXELS here. */
    lv_disp_draw_buf_init(&draw_buf_dsc, draw_buffer, NULL,
                         MY_DISP_HOR_RES * DRAW_BUFFER_ROWS);

    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = MY_DISP_HOR_RES;
    disp_drv.ver_res = MY_DISP_VER_RES;
    disp_drv.draw_buf = &draw_buf_dsc;
    disp_drv.flush_cb = disp_flush;

    if (lv_disp_drv_register(&disp_drv) == NULL) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

void disp_enable_update(void)
{
    disp_flush_enabled = true;

    if (lv_disp_get_default() != NULL) {
        lv_obj_invalidate(lv_scr_act());
    }
}

void disp_disable_update(void)
{
    disp_flush_enabled = false;
}
