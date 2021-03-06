#ifndef I_DISPLAY_H
#define I_DISPLAY_H

//****************************************************************************************************
//????????? ???????
//****************************************************************************************************

//****************************************************************************************************
//???????????? ??????????
//****************************************************************************************************
#include <stdbool.h>
#include <stdint.h>

//****************************************************************************************************
//????????????????
//****************************************************************************************************

//****************************************************************************************************
//?????????
//****************************************************************************************************

//****************************************************************************************************
//??????????????? ??????????
//****************************************************************************************************

//****************************************************************************************************
//????????? ???????
//****************************************************************************************************
class IDisplay
{
 public:
  //-????????????---------------------------------------------------------------------------------------
  //???????????????? ????? 
  enum COLOR
	{
   COLOR_BLACK=0x0000,
   COLOR_RED=0xF800,
   COLOR_ORANGE=0xFBE0,
   COLOR_YELLOW=0xFFE0,
   COLOR_GREEN=0x07E0,
   COLOR_CYAN=0x07FF,
   COLOR_BLUE=0x001F,
   COLOR_MAGENTA=0xF81F,
   COLOR_VIOLET=0x881F,
   COLOR_WHITE=0xFFFF
	};
  //-?????????------------------------------------------------------------------------------------------
  //-?????????------------------------------------------------------------------------------------------
	static const int32_t DISPLAY_WIDTH=240;//?????? ??????
	static const int32_t DISPLAY_HEIGHT=320;//?????? ??????
 private:
  //-??????????-----------------------------------------------------------------------------------------
 public:
  //-??????????-----------------------------------------------------------------------------------------
  virtual ~IDisplay() {};
 public:
  //-???????? ???????-----------------------------------------------------------------------------------
  virtual void Init(void)=0;//????????????? LCD-??????
  virtual void SetWindow(uint16_t x_left,uint16_t y_top,uint16_t x_right,uint16_t y_bottom)=0;//?????? ???? ?????? ????????
  virtual void OutColor(uint16_t color)=0;//???????? ???? ?????
  virtual void PutPixel(uint16_t x,uint16_t y,uint16_t color)=0;//??????? ???? ???????
  //-???????? ??????????? ???????-----------------------------------------------------------------------
};

#endif

