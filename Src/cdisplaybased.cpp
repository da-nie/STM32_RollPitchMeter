//****************************************************************************************************
//������������ ����������
//****************************************************************************************************
#include "cdisplaybased.h"

//****************************************************************************************************
//���������� ����������
//****************************************************************************************************

//****************************************************************************************************
//���������
//****************************************************************************************************

//****************************************************************************************************
//����������������
//****************************************************************************************************

//****************************************************************************************************
//����������� � ����������
//****************************************************************************************************

//----------------------------------------------------------------------------------------------------
//�����������
//----------------------------------------------------------------------------------------------------
CDisplayBased::CDisplayBased(void)
{
}
//----------------------------------------------------------------------------------------------------
//����������
//----------------------------------------------------------------------------------------------------
CDisplayBased::~CDisplayBased()
{
}

//****************************************************************************************************
//�������� �������
//****************************************************************************************************

//----------------------------------------------------------------------------------------------------
//
//----------------------------------------------------------------------------------------------------

//****************************************************************************************************
//�������� �������
//****************************************************************************************************

//----------------------------------------------------------------------------------------------------
//������� ���� �������
//----------------------------------------------------------------------------------------------------
void CDisplayBased::PutPixel(uint16_t x,uint16_t y,uint16_t color)
{
 SetWindow(x,y,x,y);
 OutColor(color);
}