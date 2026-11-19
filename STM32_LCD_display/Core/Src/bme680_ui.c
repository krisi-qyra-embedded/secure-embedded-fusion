/**
 * @file bme680_ui.c
 * @brief LVGL 8.3 dashboard for ST7789 240x320 portrait panel.
 *
 * Black background, white parameter names, green live values.
 * This module is GUI-only; it does not communicate with the BME680.
 */
#include "bme680_ui.h"
#include "lvgl.h"
#include <stdint.h>
#include <stdio.h>
#include <stddef.h>

#define UI_WHITE   0xFFFFFFUL
#define UI_GREEN   0x00FF70UL
#define UI_GRAY    0xAAAAAAUL
#define UI_BLACK   0x000000UL

static lv_obj_t *temperature_value;
static lv_obj_t *humidity_value;
static lv_obj_t *pressure_value;
static lv_obj_t *gas_value;
static lv_obj_t *status_value;

/* A self-contained signed fixed-point formatter; no -u _printf_float needed. */
static void set_one_decimal(lv_obj_t *label, float value, const char *unit)
{
    char buf[36];
    const float scaled = value * 10.0f;
    const int32_t tenths = (int32_t)(scaled + (scaled >= 0.0f ? 0.5f : -0.5f));
    const uint32_t magnitude = (tenths < 0) ? (uint32_t)(-(int64_t)tenths) : (uint32_t)tenths;

    (void)snprintf(buf, sizeof(buf), "%s%lu.%lu %s",
                   (tenths < 0 ? "-" : ""),
                   (unsigned long)(magnitude / 10U),
                   (unsigned long)(magnitude % 10U),
                   unit);
    lv_label_set_text(label, buf);
}

static void set_gas(lv_obj_t *label, uint32_t gas_ohms)
{
    char buf[36];

    if (gas_ohms == 0U) {
        lv_label_set_text(label, "--");
        return;
    }

    /* kOhm to 1 decimal: 100 ohms = 0.1 kOhm. */
    const uint32_t k_tenths = gas_ohms / 100U + ((gas_ohms % 100U) >= 50U ? 1U : 0U);
    (void)snprintf(buf, sizeof(buf), "%lu.%lu kOhm",
                   (unsigned long)(k_tenths / 10U),
                   (unsigned long)(k_tenths % 10U));
    lv_label_set_text(label, buf);
}

static lv_obj_t *make_text(lv_obj_t *parent,
                           const char *text,
                           uint32_t rgb,
                           lv_coord_t x,
                           lv_coord_t y)
{
    lv_obj_t *obj = lv_label_create(parent);
    if (obj == NULL) {
        return NULL;
    }
    lv_label_set_text(obj, text);
    lv_obj_set_style_text_color(obj, lv_color_hex(rgb), 0);
    lv_obj_set_pos(obj, x, y);
    return obj;
}

static lv_obj_t *make_value(lv_obj_t *parent, lv_coord_t y)
{
    lv_obj_t *obj = make_text(parent, "--", UI_GREEN, 120, y);
    if (obj == NULL) {
        return NULL;
    }
    lv_obj_set_width(obj, 110);
    lv_label_set_long_mode(obj, LV_LABEL_LONG_CLIP);
    lv_obj_set_style_text_align(obj, LV_TEXT_ALIGN_RIGHT, 0);
    return obj;
}

bool bme680_ui_init(void)
{
    lv_obj_t *screen = lv_scr_act();
    lv_obj_t *title;
    lv_obj_t *subtitle;

    temperature_value = NULL;
    humidity_value = NULL;
    pressure_value = NULL;
    gas_value = NULL;
    status_value = NULL;

    lv_obj_clean(screen);
    lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(screen, 0, 0);
    lv_obj_set_style_border_width(screen, 0, 0);
    lv_obj_set_style_bg_color(screen, lv_color_hex(UI_BLACK), 0);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);

    title = lv_label_create(screen);
    if (title == NULL) return false;
    lv_label_set_text(title, "BME680");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(UI_WHITE), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 26);

    subtitle = lv_label_create(screen);
    if (subtitle == NULL) return false;
    lv_label_set_text(subtitle, "LIVE SENSOR DATA");
    lv_obj_set_style_text_color(subtitle, lv_color_hex(UI_GRAY), 0);
    lv_obj_align(subtitle, LV_ALIGN_TOP_MID, 0, 64);

    if (make_text(screen, "Temperature", UI_WHITE, 12, 108) == NULL) return false;
    if (make_text(screen, "Humidity",    UI_WHITE, 12, 146) == NULL) return false;
    if (make_text(screen, "Pressure",    UI_WHITE, 12, 184) == NULL) return false;
    if (make_text(screen, "Gas",         UI_WHITE, 12, 222) == NULL) return false;

    temperature_value = make_value(screen, 108);
    humidity_value    = make_value(screen, 146);
    pressure_value    = make_value(screen, 184);
    gas_value         = make_value(screen, 222);
    if (temperature_value == NULL || humidity_value == NULL ||
        pressure_value == NULL || gas_value == NULL) {
        return false;
    }

    status_value = lv_label_create(screen);
    if (status_value == NULL) return false;
    lv_label_set_text(status_value, "WAITING FOR BME680...");
    lv_obj_set_style_text_color(status_value, lv_color_hex(UI_GRAY), 0);
    lv_obj_align(status_value, LV_ALIGN_BOTTOM_MID, 0, -22);
    return true;
}

void bme680_ui_update(float temperature_c,
                      float humidity_percent,
                      float pressure_hpa,
                      uint32_t gas_resistance_ohms)
{
    if (status_value == NULL) return;

    set_one_decimal(temperature_value, temperature_c, "C");
    set_one_decimal(humidity_value, humidity_percent, "%");
    set_one_decimal(pressure_value, pressure_hpa, "hPa");
    set_gas(gas_value, gas_resistance_ohms);

    lv_label_set_text(status_value, "SENSOR LIVE");
    lv_obj_set_style_text_color(status_value, lv_color_hex(UI_GREEN), 0);
    lv_obj_align(status_value, LV_ALIGN_BOTTOM_MID, 0, -22);
}

void bme680_ui_waiting(void)
{
    if (status_value == NULL) return;
    lv_label_set_text(status_value, "WAITING FOR BME680...");
    lv_obj_set_style_text_color(status_value, lv_color_hex(UI_GRAY), 0);
    lv_obj_align(status_value, LV_ALIGN_BOTTOM_MID, 0, -22);
}

void bme680_ui_error(void)
{
    if (status_value == NULL) return;
    lv_label_set_text(status_value, "SENSOR ERROR");
    lv_obj_set_style_text_color(status_value, lv_color_hex(UI_WHITE), 0);
    lv_obj_align(status_value, LV_ALIGN_BOTTOM_MID, 0, -22);
}
