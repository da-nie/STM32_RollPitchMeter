//****************************************************************************************************
//������������ ����������
//****************************************************************************************************
#include "main.h"
#include "stm32f4xx_hal.h"
//#include "cdisplayhx8347d.h"
//#include "cdisplayspfd5408.h"
#include "cdisplayil9325.h"
#include "cdisplaymemory.h"
#include "cdisplaystandardlibrary.h"
#include "mpu6050.h"
#include "i2c.h"
#include "cmathprocessing.h"

#include <math.h>
#include <stdlib.h>

//****************************************************************************************************
//����������������
//****************************************************************************************************

//****************************************************************************************************
//���������
//****************************************************************************************************

//****************************************************************************************************
//���������� ����������
//****************************************************************************************************
//CDisplayHX8347D cDisplay;
//CDisplaySPFD5408 cDisplay;
CDisplayIL9325 cDisplay;//�������
CDisplayMemory cDisplay_Memory;//������� � ������
IDisplay *iDisplay_Ptr=&cDisplay;//��������� �� �������
IDisplay *iDisplay_MemoryPtr=&cDisplay_Memory;//��������� �� �������
CDisplayStandardLibrary cDisplayStandardLibrary(iDisplay_MemoryPtr,false);//����������� ���������� �������

TIM_HandleTypeDef htim1;

CMathProcessing cMathProcessing;//����� �������������� ���������

extern uint8_t VideoBuffer[];

//****************************************************************************************************
//��������� �������
//****************************************************************************************************

void RCC_Init(void);//������������� RCC
void TIM1_Init(void);//������������� ������� T1

void ErrorHandler(void);//��������� ���������� ������
void _Error_Handler(char *file,int line);//���������� ���������� ������

//****************************************************************************************************
//���������� �������
//****************************************************************************************************

//----------------------------------------------------------------------------------------------------
//������� ������� ���������
//----------------------------------------------------------------------------------------------------
int main(void)
{ 	
 HAL_Init();
 RCC_Init();
 if (I2C_Init()==false) ErrorHandler();
 MPU6050_Init();
 TIM1_Init();
	
 HAL_TIM_Base_Start_IT(&htim1);	
	
 iDisplay_Ptr->Init();
	 
 while(1)
 {	 
  cDisplayStandardLibrary.Clear(IDisplay::COLOR_BLACK);
		  
	CMathProcessing::SValue sValue;
  sValue=cMathProcessing.GetValue();	 
	 
	char str[200];
	sprintf(str,"�����������:%.1f",sValue.Temper);
	cDisplayStandardLibrary.PutString(0,0,str,IDisplay::COLOR_YELLOW);
	sprintf(str,"����:%.1f ������:%.1f",sValue.CurrentAngle[0],sValue.CurrentAngle[1]);
	cDisplayStandardLibrary.PutString(0,24,str,IDisplay::COLOR_YELLOW);
	sprintf(str,"����,�:%.1f ������,�:%.1f",sValue.AmplitudeAngle[0],sValue.AmplitudeAngle[1]);
	cDisplayStandardLibrary.PutString(0,48,str,IDisplay::COLOR_YELLOW);
	sprintf(str,"����,C:%.1f ������,C:%.1f",sValue.ConstAngle[0],sValue.ConstAngle[1]);
	cDisplayStandardLibrary.PutString(0,64,str,IDisplay::COLOR_YELLOW);
	sprintf(str,"����,T:%.1f ������,T:%.1f",sValue.PeriodAngle[0],sValue.PeriodAngle[1]);
	cDisplayStandardLibrary.PutString(0,96,str,IDisplay::COLOR_YELLOW);

  //���������� �� ������  	 
	iDisplay_Ptr->SetWindow(0,0,IDisplay::DISPLAY_WIDTH-1,IDisplay::DISPLAY_HEIGHT-1); 
	for(uint32_t p=0;p<IDisplay::DISPLAY_WIDTH*IDisplay::DISPLAY_HEIGHT;p++) 
	{
   uint8_t c=VideoBuffer[p];
	 uint16_t color=0;
	 if (c==0) color=IDisplay::COLOR_BLACK;
   if (c==1) color=IDisplay::COLOR_RED;
   if (c==2) color=IDisplay::COLOR_ORANGE;
   if (c==3) color=IDisplay::COLOR_YELLOW;
   if (c==4) color=IDisplay::COLOR_GREEN;
   if (c==5) color=IDisplay::COLOR_CYAN;
   if (c==6) color=IDisplay::COLOR_BLUE;
   if (c==7) color=IDisplay::COLOR_MAGENTA;
   if (c==8) color=IDisplay::COLOR_VIOLET;
   if (c==9) color=IDisplay::COLOR_WHITE;
	 iDisplay_Ptr->OutColor(color);
	}	
 } 
}

//----------------------------------------------------------------------------------------------------
//������������� ��������� ����������
//----------------------------------------------------------------------------------------------------
void RCC_Init(void)
{
 RCC_OscInitTypeDef RCC_OscInitStruct;
 RCC_ClkInitTypeDef RCC_ClkInitStruct;
 __HAL_RCC_PWR_CLK_ENABLE();
 __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
 RCC_OscInitStruct.OscillatorType=RCC_OSCILLATORTYPE_HSE;
 RCC_OscInitStruct.HSEState=RCC_HSE_ON;
 RCC_OscInitStruct.PLL.PLLState=RCC_PLL_ON;
 RCC_OscInitStruct.PLL.PLLSource=RCC_PLLSOURCE_HSE;
 RCC_OscInitStruct.PLL.PLLM=4;
 RCC_OscInitStruct.PLL.PLLN=168;
 RCC_OscInitStruct.PLL.PLLP=RCC_PLLP_DIV2;
 RCC_OscInitStruct.PLL.PLLQ=7;
 if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!=HAL_OK) ErrorHandler();
 
 RCC_ClkInitStruct.ClockType=RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
 RCC_ClkInitStruct.SYSCLKSource=RCC_SYSCLKSOURCE_PLLCLK;
 RCC_ClkInitStruct.AHBCLKDivider=RCC_SYSCLK_DIV1;
 RCC_ClkInitStruct.APB1CLKDivider=RCC_HCLK_DIV4;
 RCC_ClkInitStruct.APB2CLKDivider=RCC_HCLK_DIV2;

 if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct,FLASH_LATENCY_5)!=HAL_OK) ErrorHandler();
 HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
 HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
 HAL_NVIC_SetPriority(SysTick_IRQn,0,0);
}
//----------------------------------------------------------------------------------------------------
//������������� �������
//----------------------------------------------------------------------------------------------------
void TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance=TIM1;
  htim1.Init.Prescaler=6400;
  htim1.Init.CounterMode=TIM_COUNTERMODE_UP;
  htim1.Init.Period=210*2;
  htim1.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter=0;
  if (HAL_TIM_Base_Init(&htim1)!=HAL_OK) ErrorHandler();
  sClockSourceConfig.ClockSource=TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1,&sClockSourceConfig)!=HAL_OK) ErrorHandler();
  sMasterConfig.MasterOutputTrigger=TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode=TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1,&sMasterConfig)!=HAL_OK) ErrorHandler();
	
	__HAL_TIM_CLEAR_FLAG(&htim1,TIM_SR_UIF);
}

 
//----------------------------------------------------------------------------------------------------
//���������� ������� (���������� 125 ��� � �������)
//----------------------------------------------------------------------------------------------------
//void HAL_SYSTICK_Callback(void)	
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
 if (htim->Instance!=TIM1)	return;

 //����������� �������� ��������� �����
 int16_t raw_gx=MPU6050_GetGyroX();
 int16_t raw_gy=MPU6050_GetGyroY();
 int16_t raw_gz=MPU6050_GetGyroZ();
 //����������� ��������� �� ����
 int16_t raw_ax=MPU6050_GetAcselX();
 int16_t raw_ay=MPU6050_GetAcselY();
 int16_t raw_az=MPU6050_GetAcselZ();
 //����������� �����������
 int16_t raw_temper=MPU6050_GetTemper();

 static const float dt=2.0f/125.0f;
	
 cMathProcessing.NewData(dt,raw_gx,raw_gy,raw_gz,raw_ax,raw_ay,raw_az,raw_temper);
}

//----------------------------------------------------------------------------------------------------
//��������� ���������� ������
//----------------------------------------------------------------------------------------------------
void ErrorHandler(void)
{
 while(1)
 {
 }
}
//----------------------------------------------------------------------------------------------------
//���������� ���������� ������
//----------------------------------------------------------------------------------------------------
void _Error_Handler(char *file,int line)
{
 ErrorHandler();
}
