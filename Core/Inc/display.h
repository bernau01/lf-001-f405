/*
 * display.h
 *
 *  Created on: Jul 27, 2023
 *      Author: Zain Irsyad
 */

#ifndef INC_DISPLAY_H_
#define INC_DISPLAY_H_

#include "oled.h"

typedef enum _DISP_Color {
	DISP_COLOR_WHITE,
	DISP_COLOR_BLACK,
	DISP_COLOR_TRANS,
} DISP_Color_Typedef;

typedef enum _DISP_Align {
	DISP_ALIGN_LEFT,
#define DISP_ALIGN_TOP 		DISP_ALIGN_LEFT
	DISP_ALIGN_RIGHT,
#define DISP_ALIGN_BOTTOM 	DISP_ALIGN_RIGHT
	DISP_ALIGN_CENTER
} DISP_Align_Typedef;

typedef struct {
	uint8_t origin_x;
	uint8_t origin_y;
	DISP_Color_Typedef color;
	uint8_t size_x;
	uint8_t size_y;
	DISP_Align_Typedef align_x;
	DISP_Align_Typedef align_y;
} DISP_Text_Typedef;

typedef struct {
	uint8_t origin_x;
	uint8_t origin_y;
	DISP_Color_Typedef color;
	uint8_t size_x;
	uint8_t size_y;
} DISP_Shape_Typedef;

void DISP_ClearBuff();
void DISP_DrawPixel(uint8_t x, uint8_t y, DISP_Color_Typedef color);
uint8_t DISP_DrawChar(char c, uint8_t x, uint8_t y, DISP_Color_Typedef color);
void DISP_DrawText(char* text, uint8_t x, uint8_t y, DISP_Color_Typedef color);
void DISP_DrawLogo(	uint8_t* logo,
					uint8_t width,
					uint8_t height,
					uint8_t x,
					uint8_t y,
					DISP_Color_Typedef color);
void DISP_DrawLineSimple(	uint8_t x1,
							uint8_t y1,
							uint8_t x2,
							uint8_t y2,
							DISP_Color_Typedef color);
void DISP_DrawRect(	uint8_t x1,
					uint8_t y1,
					uint8_t x2,
					uint8_t y2,
					DISP_Color_Typedef color);
void DISP_FillRect(	uint8_t x1,
					uint8_t y1,
					uint8_t x2,
					uint8_t y2,
					DISP_Color_Typedef color);
uint8_t DISP_CountCharString(char* str);
void DISP_DisplayText(DISP_Text_Typedef *s, char* text);
void DISP_OLEDUpdate(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);
void DISP_FillRect2(DISP_Shape_Typedef* s);
void DISP_DisplayFillRect(DISP_Shape_Typedef* s);
void DISP_Test();
void DISP_ClearAll();

#endif /* INC_DISPLAY_H_ */
