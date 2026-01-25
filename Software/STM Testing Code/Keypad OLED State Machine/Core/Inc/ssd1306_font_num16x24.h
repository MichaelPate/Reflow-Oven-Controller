/*
 * ssd1306_font_num16x24.h
 *
 *  Created on: Jan 17, 2026
 *      Author: michaelp
 */

#ifndef SSD1306_FONT_NUM16X24_H
#define SSD1306_FONT_NUM16X24_H

#include <stdint.h>

#define NUM_FONT_WIDTH   16
#define NUM_FONT_PAGES   3   // 24px high

extern const uint8_t font_num_16x24[10][NUM_FONT_PAGES][NUM_FONT_WIDTH];

#endif
