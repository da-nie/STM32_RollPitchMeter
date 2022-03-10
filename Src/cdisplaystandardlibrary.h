#ifndef C_DISPLAY_STANDARD_LIBRARY_H
#define C_DISPLAY_STANDARD_LIBRARY_H

//****************************************************************************************************
//����������� ���������� ��� �������
//****************************************************************************************************

//****************************************************************************************************
//������������ ����������
//****************************************************************************************************
#include "idisplay.h"

//****************************************************************************************************
//����������������
//****************************************************************************************************

//****************************************************************************************************
//���������
//****************************************************************************************************

//****************************************************************************************************
//��������������� ����������
//****************************************************************************************************

//****************************************************************************************************
//����������� ���������� ��� �������
//****************************************************************************************************
class CDisplayStandardLibrary
{
 public:
  //-������������---------------------------------------------------------------------------------------
  //-���������------------------------------------------------------------------------------------------
  //-���������------------------------------------------------------------------------------------------
  static const int32_t FONT_HEIGHT=14;//������ ������
  static const int32_t FONT_WIDTH=8;//������ ������
 private:
  //-����������-----------------------------------------------------------------------------------------
  int32_t PrintYPosition;//������� ������ ������
  uint16_t ClearColor;//���� ������� ������
  IDisplay *iDisplay_Ptr;//��������� �� ����� ������� � ������� ������������ ������
  bool OrientationVertical;//������������ ����������
 public:
  //-�����������----------------------------------------------------------------------------------------
  CDisplayStandardLibrary(IDisplay *iDisplay_Ptr_Set,bool orientation_vertical=true);
  //-����������-----------------------------------------------------------------------------------------
  ~CDisplayStandardLibrary();
 public:
  //-�������� �������-----------------------------------------------------------------------------------
  void PutSymbol(int32_t x,int32_t y,char symbol,uint16_t color);//����� ������� � �������
  void PutString(int32_t x,int32_t y,const char *string,uint16_t color);//����� ������� � �������
  void Print(const char *string,uint16_t color);//������� ����� � ������� �������
  void Clear(uint16_t color);//�������� ������� ������� � �������� ������� 
  void DrawLine(int32_t x1,int32_t y1,int32_t x2,int32_t y2,uint16_t color);//���������� �����
  void FillRectangle(int32_t x1,int32_t y1,int32_t x2,int32_t y2,uint16_t color);//���������� ����������� �������������
  void FillTriangle(int32_t ax,int32_t ay,int32_t bx,int32_t by,int32_t cx,int32_t cy,uint16_t color);//���������� ����������� �����������
 private:
  //-�������� �������-----------------------------------------------------------------------------------  
  void PutSymbolVertical(int32_t x,int32_t y,uint8_t symbol,uint16_t color);//����� ������� � ������� � ������������ ����������
  void PutSymbolHorizontal(int32_t x,int32_t y,uint8_t symbol,uint16_t color);//����� ������� � ������� � �������������� ����������
  void DrawTriangleLine(int32_t y,int32_t x1,int32_t x2,uint16_t color);//��������� ����� ������������
};

#endif