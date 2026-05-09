/**
 * @file display_service.c
 * @brief 显示抽象层 —— 编译期切换 OLED/LCD 后端
 *
 * 通过 app.h 中的 APP_DISPLAY_TYPE 宏选择：
 *   DISPLAY_TYPE_OLED → OLED (SSD1306, oled_spi_V0.2)
 *   DISPLAY_TYPE_LCD  → LCD (TFT, lcd_model)
 */
#include "display_service.h"
#if APP_ENABLE_DISPLAY
#include <string.h>

#if (APP_DISPLAY_TYPE == DISPLAY_TYPE_OLED)

#include "oled_spi_V0.2.h"

static u8 to_page(uint16_t pixel_y)
{
    u8 page = (u8)(pixel_y / 8U);
    if (page > 7U) { page = 7U; }
    return page;
}

void DisplayService_Init(void)
{
    OLED_Init();
}

void DisplayService_Clear(void)
{
    OLED_Clear();
}

void DisplayService_ShowText(uint16_t x, uint16_t y, const char *text)
{
    if (text != NULL)
    {
        OLED_ShowString((u8)x, to_page(y), (char *)text);
    }
}

void DisplayService_ShowNumber(uint16_t x, uint16_t y, int32_t num, uint8_t len)
{
    u8 page = to_page(y);
    u32 display_val;
    char sign_char;

    if (num < 0)
    {
        sign_char = '-';
        display_val = (u32)(-num);
    }
    else
    {
        sign_char = ' ';
        display_val = (u32)num;
    }

    if ((x >= 8U) && (len > 1U))
    {
        OLED_ShowString((u8)(x - 8U), page, &sign_char);
        OLED_ShowNum((u8)x, page, display_val, (u8)(len - 1U), 12U);
    }
    else
    {
        OLED_ShowNum((u8)x, page, display_val, (u8)len, 12U);
    }
}

#elif (APP_DISPLAY_TYPE == DISPLAY_TYPE_LCD)

#include "lcd_st7789.h"

void DisplayService_Init(void)
{
    SPI_LCD_Init();
}

void DisplayService_Clear(void)
{
    LCD_Clear();
}

void DisplayService_ShowText(uint16_t x, uint16_t y, const char *text)
{
    if (text != NULL)
    {
        LCD_DisplayString(x, y, (char *)text);
    }
}

void DisplayService_ShowNumber(uint16_t x, uint16_t y, int32_t num, uint8_t len)
{
    LCD_DisplayNumber(x, y, num, len);
}

#else
#error "APP_DISPLAY_TYPE must be DISPLAY_TYPE_OLED or DISPLAY_TYPE_LCD"
#endif

#endif /* APP_ENABLE_DISPLAY */
