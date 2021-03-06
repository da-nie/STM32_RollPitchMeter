//****************************************************************************************************
//???????????? ??????????
//****************************************************************************************************
#include "cdisplaymemory.h"

//****************************************************************************************************
//?????????? ??????????
//****************************************************************************************************

uint8_t VideoBuffer[IDisplay::DISPLAY_HEIGHT*IDisplay::DISPLAY_WIDTH];

//****************************************************************************************************
//????????????????
//****************************************************************************************************


//****************************************************************************************************
//?????????
//****************************************************************************************************

//****************************************************************************************************
//??????????? ? ??????????
//****************************************************************************************************

//----------------------------------------------------------------------------------------------------
//???????????
//----------------------------------------------------------------------------------------------------
CDisplayMemory::CDisplayMemory(void)
{ 
 Left=0;
 Right=0;	
 Top=0;
 Bottom=0;
 Px=0;
 Py=0;
}
//----------------------------------------------------------------------------------------------------
//??????????
//----------------------------------------------------------------------------------------------------
CDisplayMemory::~CDisplayMemory()
{
}

//****************************************************************************************************
//???????? ???????
//****************************************************************************************************


//****************************************************************************************************
//???????? ???????
//****************************************************************************************************

//----------------------------------------------------------------------------------------------------
//????????????? LCD-??????
//----------------------------------------------------------------------------------------------------
void CDisplayMemory::Init(void)
{
}
//----------------------------------------------------------------------------------------------------
//?????? ???? ?????? ????????
//----------------------------------------------------------------------------------------------------
void CDisplayMemory::SetWindow(uint16_t x_left,uint16_t y_top,uint16_t x_right,uint16_t y_bottom)
{
 Left=x_left;
 Top=y_top;	
 Right=x_right;
 Bottom=y_bottom;
 Px=x_left;
 Py=y_top;
}
//----------------------------------------------------------------------------------------------------
//???????? ???? ?????
//----------------------------------------------------------------------------------------------------
void CDisplayMemory::OutColor(uint16_t color)
{
 uint32_t offset=Px+Py*DISPLAY_WIDTH;	
 uint8_t c=0;
 if (color==COLOR_BLACK) c=0;
 if (color==COLOR_RED) c=1;
 if (color==COLOR_ORANGE) c=2;
 if (color==COLOR_YELLOW) c=3;
 if (color==COLOR_GREEN) c=4;
 if (color==COLOR_CYAN) c=5;
 if (color==COLOR_BLUE) c=6;
 if (color==COLOR_MAGENTA) c=7;
 if (color==COLOR_VIOLET) c=8;
 if (color==COLOR_WHITE) c=9;

 VideoBuffer[offset]=c;

 Px++;	
 if (Px==Right+1)
 {
  Py++;
  Px=Left;
	if (Py==Bottom+1)
	{
	 Py=Top;
	}
 }
}
