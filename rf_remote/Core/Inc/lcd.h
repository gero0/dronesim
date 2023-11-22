#include "stdint.h"

typedef void (*Delay_f)(uint16_t);

/**
 * @brief Initializes the display. Note that before you use the display, you need to define
 * ports and pins assigned to control pins of the screen.
 */
void LCD_init(Delay_f delay_function);

/**
 * @brief Clears the screen.
 */
void LCD_clear();

/**
 * @brief Write text to the screen
 * 
 * @param text String to write
 * @param len Length of the string. Must be <= 16.
 */
void LCD_write_text(const char* text, uint32_t len);

/**
 * @brief Set the position of cursor on the LCD
 * 
 * @param x x - position (column). Counted from 1.
 * @param y y - position (row). Counted from 1.
 */
void LCD_position(uint8_t x, uint8_t y);