#ifndef C_DISPLAY_BASED_H
#define C_DISPLAY_BASED_H

//****************************************************************************************************
//??????? ??????? ???????
//****************************************************************************************************

//****************************************************************************************************
//???????????? ??????????
//****************************************************************************************************
#include "idisplay.h"

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
//??????? ??????? ???????
//****************************************************************************************************
class CDisplayBased:public IDisplay
{
 public:
  //-????????????---------------------------------------------------------------------------------------
  //-?????????------------------------------------------------------------------------------------------
  //-?????????------------------------------------------------------------------------------------------
 private:
  //-??????????-----------------------------------------------------------------------------------------
 public:
  //-???????????----------------------------------------------------------------------------------------
  CDisplayBased(void);
  //-??????????-----------------------------------------------------------------------------------------
  virtual ~CDisplayBased();
 public:
  //-???????? ???????-----------------------------------------------------------------------------------
  virtual void PutPixel(uint16_t x,uint16_t y,uint16_t color);//??????? ???? ???????
 private:
  //-???????? ???????-----------------------------------------------------------------------------------  
};

#endif
