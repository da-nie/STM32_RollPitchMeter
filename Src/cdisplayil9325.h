#ifndef C_DISPLAY_IL9325_H
#define C_DISPLAY_IL9325_H

//****************************************************************************************************
//????? ??????? IL9325
//****************************************************************************************************

//****************************************************************************************************
//???????????? ??????????
//****************************************************************************************************
#include "cdisplaybased.h"

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
//????? ??????? IL9325
//****************************************************************************************************
class CDisplayIL9325:public CDisplayBased
{
 public:
  //-????????????---------------------------------------------------------------------------------------
  //-?????????------------------------------------------------------------------------------------------
  //-?????????------------------------------------------------------------------------------------------
 private:
  //-??????????-----------------------------------------------------------------------------------------
 public:
  //-???????????----------------------------------------------------------------------------------------
  CDisplayIL9325(void);
  //-??????????-----------------------------------------------------------------------------------------
  virtual ~CDisplayIL9325();
 public:
  //-???????? ???????-----------------------------------------------------------------------------------
  virtual void Init(void);//????????????? LCD-??????
  virtual void SetWindow(uint16_t x_left,uint16_t y_top,uint16_t x_right,uint16_t y_bottom);//?????? ???? ?????? ????????
  virtual void OutColor(uint16_t color);//???????? ???? ?????
 private:
  //-???????? ???????-----------------------------------------------------------------------------------  
  void CS_One(void);//????????? ?? CS 1
  void CS_Zero(void);//????????? ?? CS 0

  void RS_One(void);//????????? ?? RS 1
  void RS_Zero(void);//????????? ?? RS 0

  void WR_One(void);//????????? ?? WR 1
  void WR_Zero(void);//????????? ?? WR 0

  void RST_One(void);//????????? ?? RST 1
  void RST_Zero(void);//????????? ?? RST 0

  void RD_One(void);//????????? ?? RD 1
  void RD_Zero(void);//????????? ?? RD 0

  void SetData(uint8_t byte);//?????? ???????? ?? ???? ??????

  void Write8(uint8_t byte);//???????? ????
  void Write16(uint16_t value);//???????? 16 ??? 
  void WriteRegister16(uint16_t reg,uint16_t value) ;//???????? ? ??????? 16 ???
  void Reset(void);//????????? ????? 
};

#endif
