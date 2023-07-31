/*
 * display.c
 *
 *  Created on: Jul 27, 2023
 *      Author: Zain Irsyad
 */

#include "display.h"
#include "font5x7.h"
#include "itsr_logo.h"
#include "stdio.h"
#include "stdlib.h"
#include "memory.h"

uint8_t disp_buff[OLED_HEIGHT/8][OLED_WIDTH];

void DISP_ClearBuff() {
	memset(disp_buff, 0x00, OLED_WIDTH*OLED_HEIGHT/8);
}

void DISP_DrawPixel(uint8_t x, uint8_t y, DISP_Color_Typedef color) {
	uint8_t page = y/8;
	uint8_t seg = y%8;

	switch(color) {
	case DISP_COLOR_WHITE:
		disp_buff[page][x] |= (1<<seg);
		break;
	case DISP_COLOR_BLACK:
		disp_buff[page][x] &=~(1<<seg);
		break;
	case DISP_COLOR_TRANS:
		break;
	}
}

uint8_t DISP_DrawChar(char c, uint8_t x, uint8_t y, DISP_Color_Typedef color) {
	uint8_t page = (y/8)%(OLED_HEIGHT/8);
	uint8_t seg = y%8;

	for(int i=0; i<5; i++) {
		if(x+i >= OLED_WIDTH) return 1;
		uint8_t bit_char[2];
		bit_char[0] = Font5x7[((uint8_t)c - 0x20)*5 + i] << seg;
		bit_char[1] = Font5x7[((uint8_t)c - 0x20)*5 + i] >> (8-seg);

		for(int j=0; j<(1+(seg!=0)) && j+page<OLED_HEIGHT/8; j++) {
			switch(color) {
			case DISP_COLOR_WHITE:
				disp_buff[page+j][x+i] |= bit_char[j];
				break;
			case DISP_COLOR_BLACK:
				disp_buff[page+j][x+i] &=~bit_char[j];
				break;
			case DISP_COLOR_TRANS:
				break;
			}
		}
	}
	return 0;
}

void DISP_DrawText(char* text, uint8_t x, uint8_t y, DISP_Color_Typedef color) {
	uint8_t char_num = 0;
	while(*text)
		if(DISP_DrawChar(*text++, x+(6*char_num++), y, color)) break;
}

void DISP_DrawLogo(	uint8_t* logo,
					uint8_t width,
					uint8_t height,
					uint8_t x,
					uint8_t y,
					DISP_Color_Typedef color) {
//coming soon
}

void DISP_DrawLineSimple(	uint8_t x1,
							uint8_t y1,
							uint8_t x2,
							uint8_t y2,
							DISP_Color_Typedef color) {
	if((y2-y1 == 0)&&(x2-x1 == 0)) return;

	if(x2-x1 >= y2-y1) {
		float m = (float)(y2-y1)/(x2-x1);
		for(int i=0; i<=x2-x1; i++) {
			DISP_DrawPixel(x1+i, (m*i)+y1, color);
		}
	} else {
		float m = (float)(x2-x1)/(y2-y1);
		for(int i=0; i<=y2-y1; i++) {
			DISP_DrawPixel((m*i)+x1, i+y1, color);
		}
	}
}

void DISP_DrawRect(	uint8_t x1,
					uint8_t y1,
					uint8_t x2,
					uint8_t y2,
					DISP_Color_Typedef color) {
	DISP_DrawLineSimple(x1, y1, x2, y1, color);
	DISP_DrawLineSimple(x1, y1, x1, y2, color);
	DISP_DrawLineSimple(x2, y1, x2, y2, color);
	DISP_DrawLineSimple(x1, y2, x2, y2, color);
}

void DISP_FillRect(	uint8_t x1,
					uint8_t y1,
					uint8_t x2,
					uint8_t y2,
					DISP_Color_Typedef color) {
	uint8_t page1 = y1/8;
	uint8_t seg1 = y1%8;
	uint8_t page2 = y2/8;
	uint8_t seg2 = y2%8;

	for(int i=page1; i<=page2; i++) {
		for(int j=x1; j<=x2; j++) {
			uint8_t bit;

			if(i==page1 && i!=page2) {
				bit = (0xFF << seg1);
			}
			else if(page1==page2) {
				bit = (0xFF >> (7-seg2)) - (0xFF >> (8-seg1));
			}
			else if(i==page2) {
				bit = (0xFF >> (7-seg2));
			}
			else {
				bit = 0xFF;
			}

			switch(color) {
			case DISP_COLOR_WHITE:
				disp_buff[i][j] |= bit;
				break;
			case DISP_COLOR_BLACK:
				disp_buff[i][j] &=~bit;
				break;
			case DISP_COLOR_TRANS:
				break;
			}
		}
	}
}

uint8_t DISP_CountCharString(char* str) {
	int ct;
	for(ct=0; str[ct]!=0; ct++);
	return ct;
}

void DISP_DisplayText(DISP_Text_Typedef *s, char* text) {
	uint8_t len_char = DISP_CountCharString(text);
	uint8_t len_bit = len_char*6-1;
	uint8_t x, y;
	uint8_t fx2, fy2;

	if(s->size_x == 0)
		fx2 = s->origin_x+len_bit+1;
	else
		fx2 = s->origin_x+s->size_x;

	if(s->size_y < 8)
		fy2 = s->origin_y+8;
	else
		fy2 = s->origin_y+s->size_y;

	DISP_FillRect(
			s->origin_x,
			s->origin_y,
			fx2,
			fy2,
			(s->color==DISP_COLOR_BLACK)?DISP_COLOR_WHITE:DISP_COLOR_BLACK);

	switch(s->align_x) {
	case DISP_ALIGN_LEFT:
		x = s->origin_x + 1;
		break;
	case DISP_ALIGN_RIGHT:
		x = fx2 - len_bit;
		break;
	case DISP_ALIGN_CENTER:
		x = s->origin_x+((fx2-s->origin_x)/2)-(len_bit/2);
		break;
	}

	switch(s->align_y) {
	case DISP_ALIGN_LEFT:
		y = s->origin_y + 1;
		break;
	case DISP_ALIGN_RIGHT:
		y = fy2 - 7;
		break;
	case DISP_ALIGN_CENTER:
		y = s->origin_y+((fy2-s->origin_y)/2)-3;
		break;
	}

	DISP_DrawText(text, x, y, s->color);

	uint8_t page1 = s->origin_y/8;
	uint8_t page2 = fy2/8;

	for(int i=page1; i<=page2&&i<OLED_HEIGHT/8; i++) {
		OLED_SetPageAddr(i);
		OLED_SetColumn(s->origin_x);
		for(int j=s->origin_x; j<=fx2&&j<OLED_WIDTH; j++) {
			OLED_Data(disp_buff[i][j]);
		}
	}

}

void DISP_OLEDUpdate(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2) {
	uint8_t page1 = y1/8;
	uint8_t page2 = y2/8;

	for(int i=page1; i<=page2&&i<OLED_HEIGHT/8; i++) {
		OLED_SetPageAddr(i);
		OLED_SetColumn(x1);
		for(int j=x1; j<=x2&&i<OLED_WIDTH; j++) {
			OLED_Data(disp_buff[i][j]);
		}
	}
}

void DISP_FillRect2(DISP_Shape_Typedef* s) {
	DISP_FillRect(
			s->origin_x,
			s->origin_y,
			s->origin_x+s->size_x,
			s->origin_y+s->size_y,
			s->color);
}

void DISP_DisplayFillRect(DISP_Shape_Typedef* s) {

	DISP_FillRect(
			s->origin_x,
			s->origin_y,
			s->origin_x+s->size_x,
			s->origin_y+s->size_y,
			s->color);

	DISP_OLEDUpdate(
			s->origin_x,
			s->origin_y,
			s->origin_x+s->size_x,
			s->origin_y+s->size_y);
}

void DISP_Test() {
	DISP_DrawText ("OK! Rownle. Now, I", 0, 0, DISP_COLOR_WHITE);
	DISP_DrawText ("want to eat your", 0, 8, DISP_COLOR_WHITE);
	DISP_DrawText ("brain!", 0, 16, DISP_COLOR_WHITE);
	DISP_DrawRect(62, 30, 122, 62, DISP_COLOR_WHITE);
	DISP_FillRect(64, 32, 120, 60, DISP_COLOR_WHITE);
	DISP_DrawText ("HOW?", 81, 42, DISP_COLOR_BLACK);
	for(int i=0; i<8; i++) {
		OLED_SetPageAddr(i);
		OLED_SetColumn(0);
		for(int j=0; j<128; j++) {
			OLED_Data(disp_buff[i][j]);
		}
	}
	for(int i=0; i<ITSR_ARRAY_HEGIHT; i++) {
		OLED_SetPageAddr(i+4);
		OLED_SetColumn(2);
		for(int j=0; j<ITSR_ARRAY_WIDTH; j++) {
			OLED_Data(itsr_logo[j+i*ITSR_ARRAY_WIDTH]);
		}
	}

	HAL_Delay(500);
//	OLED_Command(OLED_DISP_OFF);
	OLED_ClearRAM();
	DISP_ClearBuff();
//	OLED_Command(OLED_DISP_ON);
//
//	DISP_Text_Typedef fix1 = {
//			2,2,
//			DISP_COLOR_BLACK,
//			30,20,
//			DISP_ALIGN_CENTER,DISP_ALIGN_CENTER};
//	DISP_DisplayText(&fix1, "FIXE");
//
//	DISP_Text_Typedef fix2 = {
//				64,27,
//				DISP_COLOR_BLACK};
////	DISP_DisplayText(&fix2, "OEEEA!");
//	int b=0;
//	char tbuff[23];

//	while(1) {
//		sprintf(tbuff, "%i", b);
//		DISP_DisplayText(&fix2, tbuff);
//		b++;
//		HAL_Delay(100);
//	}
}

void DISP_ClearAll() {
//	OLED_Command(OLED_DISP_OFF);
	OLED_ClearRAM();
	DISP_ClearBuff();
	OLED_Command(OLED_DISP_ON);
}

