#ifndef C_DISPLAY_MEMORY_H
#define C_DISPLAY_MEMORY_H

//****************************************************************************************************
//����� ������� � ������
//****************************************************************************************************

//****************************************************************************************************
//������������ ����������
//****************************************************************************************************
#include "cdisplaybased.h"

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
//����� ������� � ������
//****************************************************************************************************
class CDisplayMemory:public CDisplayBased
{
 public:
  //-������������---------------------------------------------------------------------------------------
  //-���������------------------------------------------------------------------------------------------
  //-���������------------------------------------------------------------------------------------------
 private:
  //-����������-----------------------------------------------------------------------------------------
 uint16_t Left;//����� ������� ���� ������
 uint16_t Top;//������� ������� ���� ������
 uint16_t Right;//������ ������� ���� ������
 uint16_t Bottom;//������ ������� ���� ������

 uint16_t Px;//������� ������� �����
 uint16_t Py; 
 public:
  //-�����������----------------------------------------------------------------------------------------
  CDisplayMemory(void);
  //-����������-----------------------------------------------------------------------------------------
  virtual ~CDisplayMemory();
 public:
  //-�������� �������-----------------------------------------------------------------------------------
  virtual void Init(void);//������������� LCD-������
  virtual void SetWindow(uint16_t x_left,uint16_t y_top,uint16_t x_right,uint16_t y_bottom);//������ ���� ������ ��������
  virtual void OutColor(uint16_t color);//�������� ���� �����
 private:
  //-�������� �������-----------------------------------------------------------------------------------  
};

#endif