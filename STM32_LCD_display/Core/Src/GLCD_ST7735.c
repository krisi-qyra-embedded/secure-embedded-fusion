
#include "GLCD_ST7735.h"
#include "clock.h"
#include <stddef.h>
#include <string.h>

/* NUCLEO-F411RE. SPI2 uses PB10 (SCK) and PB15 (MOSI), both AF5. */
#define LCD_CS_PIN   GPIO_PIN_3
#define LCD_DC_PIN   GPIO_PIN_4
#define LCD_RST_PIN  GPIO_PIN_5
#define CMD_SLPOUT  0x11U
#define CMD_NORON   0x13U
#define CMD_INVOFF  0x20U
#define CMD_INVON   0x21U
#define CMD_DISPON  0x29U
#define CMD_CASET   0x2AU
#define CMD_RASET   0x2BU
#define CMD_RAMWR   0x2CU
#define CMD_MADCTL  0x36U
#define CMD_COLMOD  0x3AU

/* Full 240-by-320 portrait panel, zero offsets. Bit 3 selects BGR. */
#define LCD_MADCTL          0xC0U
#define LCD_INVERSION_CMD  CMD_INVON
#define LCD_RGB565         0x55U
#define SPI_TIMEOUT_MS     50U
#define TRANSFER_PIXELS    256U
#define CHAR_WIDTH   6U
#define CHAR_HEIGHT  8U
#define TEXT_LINE    10U
#define PLOT_LEFT    12U
#define PLOT_TOP     20U
#define PLOT_RIGHT   (ST7789_TFTWIDTH - 12U)
#define PLOT_BOTTOM  (ST7789_TFTHEIGHT - 22U)

static SPI_HandleTypeDef lcd_spi;
static uint8_t spi_pixels[2U * TRANSFER_PIXELS];
static uint16_t plot_x = PLOT_LEFT;
static uint16_t plot_bg_color;
static int32_t plot_min;
static int32_t plot_max;

/* Each byte in the course font stores one glyph column with its top pixel at
 * bit 0. The sixth column is blank for spacing. */
static const uint8_t font[][CHAR_WIDTH] = {
    {0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x5F, 0x00, 0x00},
    {0x00, 0x07, 0x00, 0x07, 0x00},
    {0x14, 0x7F, 0x14, 0x7F, 0x14},
    {0x24, 0x2A, 0x7F, 0x2A, 0x12},
    {0x23, 0x13, 0x08, 0x64, 0x62},
    {0x36, 0x49, 0x56, 0x20, 0x50},
    {0x00, 0x08, 0x07, 0x03, 0x00},
    {0x00, 0x1C, 0x22, 0x41, 0x00},
    {0x00, 0x41, 0x22, 0x1C, 0x00},
    {0x2A, 0x1C, 0x7F, 0x1C, 0x2A},
    {0x08, 0x08, 0x3E, 0x08, 0x08},
    {0x00, 0x80, 0x70, 0x30, 0x00},
    {0x08, 0x08, 0x08, 0x08, 0x08},
    {0x00, 0x00, 0x60, 0x60, 0x00},
    {0x20, 0x10, 0x08, 0x04, 0x02},
    {0x3E, 0x51, 0x49, 0x45, 0x3E},
    {0x00, 0x42, 0x7F, 0x40, 0x00},
    {0x72, 0x49, 0x49, 0x49, 0x46},
    {0x21, 0x41, 0x49, 0x4D, 0x33},
    {0x18, 0x14, 0x12, 0x7F, 0x10},
    {0x27, 0x45, 0x45, 0x45, 0x39},
    {0x3C, 0x4A, 0x49, 0x49, 0x31},
    {0x41, 0x21, 0x11, 0x09, 0x07},
    {0x36, 0x49, 0x49, 0x49, 0x36},
    {0x46, 0x49, 0x49, 0x29, 0x1E},
    {0x00, 0x00, 0x14, 0x00, 0x00},
    {0x00, 0x40, 0x34, 0x00, 0x00},
    {0x00, 0x08, 0x14, 0x22, 0x41},
    {0x14, 0x14, 0x14, 0x14, 0x14},
    {0x00, 0x41, 0x22, 0x14, 0x08},
    {0x02, 0x01, 0x59, 0x09, 0x06},
    {0x3E, 0x41, 0x5D, 0x59, 0x4E},
    {0x7C, 0x12, 0x11, 0x12, 0x7C},
    {0x7F, 0x49, 0x49, 0x49, 0x36},
    {0x3E, 0x41, 0x41, 0x41, 0x22},
    {0x7F, 0x41, 0x41, 0x41, 0x3E},
    {0x7F, 0x49, 0x49, 0x49, 0x41},
    {0x7F, 0x09, 0x09, 0x09, 0x01},
    {0x3E, 0x41, 0x41, 0x51, 0x73},
    {0x7F, 0x08, 0x08, 0x08, 0x7F},
    {0x00, 0x41, 0x7F, 0x41, 0x00},
    {0x20, 0x40, 0x41, 0x3F, 0x01},
    {0x7F, 0x08, 0x14, 0x22, 0x41},
    {0x7F, 0x40, 0x40, 0x40, 0x40},
    {0x7F, 0x02, 0x1C, 0x02, 0x7F},
    {0x7F, 0x04, 0x08, 0x10, 0x7F},
    {0x3E, 0x41, 0x41, 0x41, 0x3E},
    {0x7F, 0x09, 0x09, 0x09, 0x06},
    {0x3E, 0x41, 0x51, 0x21, 0x5E},
    {0x7F, 0x09, 0x19, 0x29, 0x46},
    {0x26, 0x49, 0x49, 0x49, 0x32},
    {0x03, 0x01, 0x7F, 0x01, 0x03},
    {0x3F, 0x40, 0x40, 0x40, 0x3F},
    {0x1F, 0x20, 0x40, 0x20, 0x1F},
    {0x3F, 0x40, 0x38, 0x40, 0x3F},
    {0x63, 0x14, 0x08, 0x14, 0x63},
    {0x03, 0x04, 0x78, 0x04, 0x03},
    {0x61, 0x59, 0x49, 0x4D, 0x43},
    {0x00, 0x7F, 0x41, 0x41, 0x41},
    {0x02, 0x04, 0x08, 0x10, 0x20},
    {0x00, 0x41, 0x41, 0x41, 0x7F},
    {0x04, 0x02, 0x01, 0x02, 0x04},
    {0x40, 0x40, 0x40, 0x40, 0x40},
    {0x00, 0x03, 0x07, 0x08, 0x00},
    {0x20, 0x54, 0x54, 0x78, 0x40},
    {0x7F, 0x28, 0x44, 0x44, 0x38},
    {0x38, 0x44, 0x44, 0x44, 0x28},
    {0x38, 0x44, 0x44, 0x28, 0x7F},
    {0x38, 0x54, 0x54, 0x54, 0x18},
    {0x00, 0x08, 0x7E, 0x09, 0x02},
    {0x18, 0xA4, 0xA4, 0x9C, 0x78},
    {0x7F, 0x08, 0x04, 0x04, 0x78},
    {0x00, 0x44, 0x7D, 0x40, 0x00},
    {0x20, 0x40, 0x40, 0x3D, 0x00},
    {0x7F, 0x10, 0x28, 0x44, 0x00},
    {0x00, 0x41, 0x7F, 0x40, 0x00},
    {0x7C, 0x04, 0x78, 0x04, 0x78},
    {0x7C, 0x08, 0x04, 0x04, 0x78},
    {0x38, 0x44, 0x44, 0x44, 0x38},
    {0xFC, 0x18, 0x24, 0x24, 0x18},
    {0x18, 0x24, 0x24, 0x18, 0xFC},
    {0x7C, 0x08, 0x04, 0x04, 0x08},
    {0x48, 0x54, 0x54, 0x54, 0x24},
    {0x04, 0x04, 0x3F, 0x44, 0x24},
    {0x3C, 0x40, 0x40, 0x20, 0x7C},
    {0x1C, 0x20, 0x40, 0x20, 0x1C},
    {0x3C, 0x40, 0x30, 0x40, 0x3C},
    {0x44, 0x28, 0x10, 0x28, 0x44},
    {0x4C, 0x90, 0x90, 0x90, 0x7C},
    {0x44, 0x64, 0x54, 0x4C, 0x44},
    {0x00, 0x08, 0x36, 0x41, 0x00},
    {0x00, 0x00, 0x77, 0x00, 0x00},
    {0x00, 0x41, 0x36, 0x08, 0x00},
    {0x02, 0x01, 0x02, 0x04, 0x02},
};

static HAL_StatusTypeDef lcd_spi_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    /* Verify the STM32 system and SPI2 bus clocks */
    if (SystemCoreClock != SYSCLK_96MHZ ||
        HAL_RCC_GetPCLK1Freq() != 48000000UL) {
        return HAL_ERROR;
    }
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_SPI2_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOB,
                      LCD_CS_PIN | LCD_RST_PIN,
                      GPIO_PIN_SET);
    gpio.Pin = LCD_CS_PIN | LCD_DC_PIN | LCD_RST_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = GPIO_PIN_10 | GPIO_PIN_15;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &gpio);
    lcd_spi.Instance = SPI2;
    lcd_spi.Init.Mode = SPI_MODE_MASTER;
    lcd_spi.Init.Direction = SPI_DIRECTION_1LINE;
    lcd_spi.Init.DataSize = SPI_DATASIZE_8BIT;
    lcd_spi.Init.CLKPolarity = SPI_POLARITY_LOW;
    lcd_spi.Init.CLKPhase = SPI_PHASE_1EDGE;
    lcd_spi.Init.NSS = SPI_NSS_SOFT;
    /* APB1 = 48 MHz / 8 = 6 MHz SPI clock */
    lcd_spi.Init.BaudRatePrescaler =
        SPI_BAUDRATEPRESCALER_8;
    lcd_spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    lcd_spi.Init.TIMode = SPI_TIMODE_DISABLED;
    lcd_spi.Init.CRCCalculation =
        SPI_CRCCALCULATION_DISABLED;
    lcd_spi.Init.CRCPolynomial = 10U;
    return HAL_SPI_Init(&lcd_spi);
}

/* CS is held low by the caller for the command and its data. */
static HAL_StatusTypeDef send_command(uint8_t command, uint8_t *data,
                                       uint16_t size)
{
    HAL_StatusTypeDef status;
    HAL_GPIO_WritePin(GPIOB, LCD_DC_PIN, GPIO_PIN_RESET);
    status = HAL_SPI_Transmit(&lcd_spi, &command, 1U, SPI_TIMEOUT_MS);
    if (status != HAL_OK || size == 0U) {
        return status;
    }
    HAL_GPIO_WritePin(GPIOB, LCD_DC_PIN, GPIO_PIN_SET);
    return HAL_SPI_Transmit(&lcd_spi, data, size, SPI_TIMEOUT_MS);
}

static HAL_StatusTypeDef set_window(uint16_t x0, uint16_t y0,
                                     uint16_t x1, uint16_t y1)
{
    uint8_t columns[] = {
        (uint8_t)(x0 >> 8), (uint8_t)x0, (uint8_t)(x1 >> 8), (uint8_t)x1
    };
    uint8_t rows[] = {
        (uint8_t)(y0 >> 8), (uint8_t)y0, (uint8_t)(y1 >> 8), (uint8_t)y1
    };
    HAL_StatusTypeDef status;
    status = send_command(CMD_CASET, columns, sizeof(columns));
    if (status == HAL_OK) {
        status = send_command(CMD_RASET, rows, sizeof(rows));
    }
    if (status == HAL_OK) {
        status = send_command(CMD_RAMWR, NULL, 0U);
    }
    return status;
}

/* A null pixel pointer selects a solid fill. The display owns the framebuffer. */
static HAL_StatusTypeDef write_rectangle(uint16_t x, uint16_t y,
                                          uint16_t width, uint16_t height,
                                          const uint16_t *pixels,
                                          uint16_t fill_color)
{
    uint32_t remaining = (uint32_t)width * height;
    HAL_StatusTypeDef status;
    if (width == 0U || height == 0U ||
        (uint32_t)x + width > ST7789_TFTWIDTH ||
        (uint32_t)y + height > ST7789_TFTHEIGHT) {
        return HAL_ERROR;
    }
    HAL_GPIO_WritePin(GPIOB, LCD_CS_PIN, GPIO_PIN_RESET);
    status = set_window(x, y, (uint16_t)(x + width - 1U),
                        (uint16_t)(y + height - 1U));
    HAL_GPIO_WritePin(GPIOB, LCD_DC_PIN, GPIO_PIN_SET);
    while (status == HAL_OK && remaining > 0U) {
        const uint16_t count = remaining > TRANSFER_PIXELS ?
                               TRANSFER_PIXELS : (uint16_t)remaining;
        for (uint16_t i = 0; i < count; ++i) {
            const uint16_t color = pixels != NULL ? *pixels++ : fill_color;
            spi_pixels[2U * i] = (uint8_t)(color >> 8);
            spi_pixels[2U * i + 1U] = (uint8_t)color;
        }
        status = HAL_SPI_Transmit(&lcd_spi, spi_pixels,
                                  (uint16_t)(2U * count), SPI_TIMEOUT_MS);
        remaining -= count;
    }
    HAL_GPIO_WritePin(GPIOB, LCD_CS_PIN, GPIO_PIN_SET);
    return status;
}

HAL_StatusTypeDef ST7789_Init(void)
{
    static const struct {
        uint8_t command;
        uint8_t param;
        uint8_t size;
        uint16_t delay_ms;
    } sequence[] = {
        {CMD_SLPOUT,        0U,          0U, 120U},
        {CMD_COLMOD,        LCD_RGB565,  1U,  10U},
        {CMD_MADCTL,        LCD_MADCTL,  1U,   0U},
        {LCD_INVERSION_CMD, 0U,          0U,   0U},
        {CMD_NORON,         0U,          0U,  10U},
        {CMD_DISPON,        0U,          0U,  20U}
    };
    HAL_StatusTypeDef status = lcd_spi_init();
    if (status != HAL_OK) {
        return status;
    }
    HAL_GPIO_WritePin(GPIOB, LCD_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(20U);
    HAL_GPIO_WritePin(GPIOB, LCD_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(120U);
    for (size_t i = 0; i < sizeof(sequence) / sizeof(sequence[0]); ++i) {
        uint8_t param = sequence[i].param;
        HAL_GPIO_WritePin(GPIOB, LCD_CS_PIN, GPIO_PIN_RESET);
        status = send_command(sequence[i].command, &param, sequence[i].size);
        HAL_GPIO_WritePin(GPIOB, LCD_CS_PIN, GPIO_PIN_SET);
        if (status != HAL_OK) {
            return status;
        }
        if (sequence[i].delay_ms != 0U) {
            HAL_Delay(sequence[i].delay_ms);
        }
    }
    return ST7789_FillScreen(ST7789_BLACK);
}

HAL_StatusTypeDef ST7789_FillRect(uint16_t x, uint16_t y, uint16_t width,
                                 uint16_t height, uint16_t color)
{
    return write_rectangle(x, y, width, height, NULL, color);
}

HAL_StatusTypeDef ST7789_FillScreen(uint16_t color)
{
    return ST7789_FillRect(0U, 0U, ST7789_TFTWIDTH, ST7789_TFTHEIGHT, color);
}

HAL_StatusTypeDef ST7789_DrawImage(uint16_t x, uint16_t y, uint16_t width,
                                  uint16_t height, const uint16_t *pixels)
{
    if (pixels == NULL) {
        return HAL_ERROR;
    }
    return write_rectangle(x, y, width, height, pixels, 0U);
}

static void draw_char(uint16_t x, uint16_t y, char ch,
                       uint16_t color, uint16_t bg)
{
    uint16_t glyph[CHAR_WIDTH * CHAR_HEIGHT];
    unsigned char code = (unsigned char)ch;
    if (code < ' ' || code > '~') {
        code = '?';
    }
    const uint8_t *columns = font[code - ' '];
    for (uint16_t row = 0U; row < CHAR_HEIGHT; ++row) {
        for (uint16_t col = 0U; col < CHAR_WIDTH; ++col) {
            glyph[row * CHAR_WIDTH + col] =
                (columns[col] & (1U << row)) != 0U ? color : bg;
        }
    }
    (void)ST7789_DrawImage(x, y, CHAR_WIDTH, CHAR_HEIGHT, glyph);
}

static uint32_t draw_text(uint16_t x, uint16_t y, const char *text,
                           uint16_t color, uint16_t bg)
{
    uint32_t count = 0U;
    while (*text != '\0' && x <= ST7789_TFTWIDTH - CHAR_WIDTH &&
           y <= ST7789_TFTHEIGHT - CHAR_HEIGHT) {
        draw_char(x, y, *text++, color, bg);
        x += CHAR_WIDTH;
        ++count;
    }
    return count;
}

uint32_t ST7789_DrawString(uint16_t column, uint16_t row, const char *text,
                           uint16_t color)
{
    if (text == NULL || column >= ST7789_TFTWIDTH / CHAR_WIDTH ||
        row > (ST7789_TFTHEIGHT - CHAR_HEIGHT) / TEXT_LINE) {
        return 0U;
    }
    return draw_text(column * CHAR_WIDTH, row * TEXT_LINE, text, color,
                     ST7789_BLACK);
}

static void draw_y_label(uint16_t y, uint16_t end_y, const char *text,
                          uint16_t color, uint16_t bg)
{
    while (*text != '\0' && y + CHAR_HEIGHT <= end_y) {
        draw_char(0U, y, *text++, color, bg);
        y += CHAR_HEIGHT;
    }
}

void ST7789_Drawaxes(uint16_t axis_color, uint16_t bg_color,
                     const char *x_label, const char *y_label1,
                     uint16_t label1_color, const char *y_label2,
                     uint16_t label2_color, int32_t ymax, int32_t ymin)
{
    const uint16_t x_axis = PLOT_BOTTOM + 1U;
    const uint16_t y_axis = PLOT_LEFT - 1U;
    const uint16_t mid_y = (PLOT_TOP + PLOT_BOTTOM) / 2U;
    uint16_t first_label_y = PLOT_TOP + 24U;
    plot_min = ymin;
    plot_max = ymax;
    plot_x = PLOT_LEFT;
    plot_bg_color = bg_color;
    (void)ST7789_FillScreen(bg_color);
    (void)ST7789_FillRect(y_axis, x_axis, PLOT_RIGHT - y_axis + 1U, 1U,
                          axis_color);
    (void)ST7789_FillRect(y_axis, PLOT_TOP, 1U, x_axis - PLOT_TOP + 1U,
                          axis_color);
    for (uint16_t x = PLOT_LEFT + 20U; x <= PLOT_RIGHT; x += 20U) {
        (void)ST7789_FillRect(x, x_axis + 1U, 1U, 2U, axis_color);
    }
    for (uint16_t y = PLOT_TOP; y < x_axis; y += 20U) {
        (void)ST7789_FillRect(y_axis - 2U, y, 2U, 1U, axis_color);
    }
    if (x_label != NULL) {
        size_t chars = strlen(x_label);
        if (chars > ST7789_TFTWIDTH / CHAR_WIDTH) {
            chars = ST7789_TFTWIDTH / CHAR_WIDTH;
        }
        const uint16_t x = (uint16_t)((ST7789_TFTWIDTH - chars * CHAR_WIDTH) / 2U);
        (void)draw_text(x, x_axis + 10U, x_label, axis_color, bg_color);
    }
    if (y_label2 != NULL && *y_label2 != '\0') {
        draw_y_label(first_label_y, mid_y, y_label2, label2_color, bg_color);
        first_label_y = mid_y;
    }
    if (y_label1 != NULL) {
        draw_y_label(first_label_y, x_axis, y_label1, label1_color, bg_color);
    }
}

void ST7789_PlotPoint(int32_t value, uint16_t color)
{
    const int64_t range = (int64_t)plot_max - plot_min;
    uint16_t y;
    if (range <= 0) {
        return;
    }
    if (value < plot_min || value > plot_max) {
        value = value < plot_min ? plot_min : plot_max;
        color = ST7789_RED;
    }
    y = PLOT_BOTTOM - (uint16_t)(((int64_t)value - plot_min) *
                                  (PLOT_BOTTOM - PLOT_TOP) / range);
    const uint16_t dot_top = (uint16_t)(y > PLOT_TOP ? y - 1U : y);
    (void)ST7789_FillRect(plot_x, dot_top, 1U, 2U, color);
}

void ST7789_PlotIncrement(void)
{
    plot_x = plot_x < PLOT_RIGHT ? plot_x + 1U : PLOT_LEFT;
    (void)ST7789_FillRect(plot_x, PLOT_TOP, 1U, PLOT_BOTTOM - PLOT_TOP + 1U,
                          plot_bg_color);
}
