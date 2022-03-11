//****************************************************************************************************
//подключаемые библиотеки
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
#include <string.h>

//****************************************************************************************************
//макроопределения
//****************************************************************************************************

//****************************************************************************************************
//константы
//****************************************************************************************************

//****************************************************************************************************
//глобальные переменные
//****************************************************************************************************
//CDisplayHX8347D cDisplay;
//CDisplaySPFD5408 cDisplay;
CDisplayIL9325 cDisplay;//дисплей
CDisplayMemory cDisplay_Memory;//дисплей в памяти
IDisplay *iDisplay_Ptr=&cDisplay;//указатель на дисплей
IDisplay *iDisplay_MemoryPtr=&cDisplay_Memory;//указатель на дисплей
CDisplayStandardLibrary cDisplayStandardLibrary(iDisplay_MemoryPtr,false);//стандартная библиотека дисплея

TIM_HandleTypeDef htim1;

CMathProcessing cMathProcessing;//класс математической обработки

extern uint8_t VideoBuffer[];

//****************************************************************************************************
//прототипы функций
//****************************************************************************************************

void RCC_Init(void);//инициализация RCC
void TIM1_Init(void);//инициализация таймера T1

void ErrorHandler(void);//локальный обработчик ошибок
void _Error_Handler(char *file,int line);//глобальный обработчик ошибок

//****************************************************************************************************
//реализация функций
//****************************************************************************************************

//----------------------------------------------------------------------------------------------------
//главная функция программы
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
	 /*
 uint32_t begin=HAL_GetTick();	
 for(size_t n=0;n<1000;n++)
 {	
  //запрашиваем скорости изменения углов
  int16_t raw_gx;
  int16_t raw_gy;
  int16_t raw_gz;
  //запрашиваем ускорения по осям
  int16_t raw_ax;
  int16_t raw_ay;
  int16_t raw_az;
  //запрашиваем температуру
  int16_t raw_temper;
  MPU6050_ReadAll(raw_gx,raw_gy,raw_gz,raw_ax,raw_ay,raw_az,raw_temper);
	
  static const float dt=1.0f/125.0f;	
  cMathProcessing.NewData(dt,raw_gx,raw_gy,raw_gz,raw_ax,raw_ay,raw_az,raw_temper);	 
 }
 uint32_t end=HAL_GetTick();
 double d=(end-begin);
 d=d/1000.0; 
	*/
	
 //рисуем интерфейс	
 cDisplayStandardLibrary.Clear(IDisplay::COLOR_BLACK);

 int32_t r1=50; 
 int32_t cx1=80;
 int32_t cy1=240-1-r1-24;
 int32_t r2=50;
 int32_t cx2=80+160;
 int32_t cy2=240-1-r2-24;
 
 cDisplayStandardLibrary.DrawCircle(cx1,cy1,r1,IDisplay::COLOR_BLUE);
 cDisplayStandardLibrary.DrawCircle(cx2,cy2,r2,IDisplay::COLOR_BLUE);
 cDisplayStandardLibrary.FillRectangle(0,0,159,cy1,IDisplay::COLOR_BLACK);
 cDisplayStandardLibrary.FillRectangle(160,0,cx2,239,IDisplay::COLOR_BLACK);
 for(int8_t n=-90;n<=90;n+=5)
 {
	float s1=sin(M_PI/180.0*(n+90));
	float c1=cos(M_PI/180.0*(n+90));   
	float s2=sin(M_PI/180.0*(n+0));
	float c2=cos(M_PI/180.0*(n+0));   
	int32_t len=5;  
	int16_t color=IDisplay::COLOR_VIOLET;
	if (n%10==0)
	{
	 len=10;  
	 color=IDisplay::COLOR_MAGENTA;
	 int8_t v=n;
	 if (v<0) v=-v;		
	 if (v==0 || v==20 || v==40 || v==60 || v==90)
	 {
 	  char str[25];
    sprintf(str,"%i",v);
    int32_t h=CDisplayStandardLibrary::FONT_HEIGHT/2;
		int32_t w=CDisplayStandardLibrary::FONT_WIDTH;
		if (v==0) w=w/2;
	  cDisplayStandardLibrary.PutString(cx1+(r1+len+8)*c1-w,cy1+(r1+len+8)*s1-h,str,IDisplay::COLOR_YELLOW);
		cDisplayStandardLibrary.PutString(cx2+(r2+len+8)*c2-w,cy2+(r2+len+8)*s2-h,str,IDisplay::COLOR_YELLOW);
	 }
	}
	cDisplayStandardLibrary.DrawLine(cx1+(r1+len)*c1,cy1+(r1+len)*s1,cx1+(r1-0)*c1,cy1+(r1-0)*s1,color);
	cDisplayStandardLibrary.DrawLine(cx2+(r2+len)*c2,cy2+(r2+len)*s2,cx2+(r2-0)*c2,cy2+(r2-0)*s2,color);		
 }
 cDisplayStandardLibrary.PutString((160-4*CDisplayStandardLibrary::FONT_WIDTH*2)/2,0,"КРЕН",IDisplay::COLOR_YELLOW,2);
 cDisplayStandardLibrary.PutString(160+(160-9*CDisplayStandardLibrary::FONT_WIDTH*2)/2,0,"ДИФФЕРЕНТ",IDisplay::COLOR_YELLOW,2);
 
 while(1)
 {		  
	CMathProcessing::SValue sValue;
  sValue=cMathProcessing.GetValue();	 
	
  //рисуем текущие углы по крену и тангажу
	float roll_y[4];
	float roll_x[4];
  float roll=sValue.CurrentAngle[0];
  float roll_const=sValue.ConstAngle[0];
  float roll_amplitude=sValue.AmplitudeAngle[0];
  float roll_period=sValue.PeriodAngle[0];
	 
	if (roll<-90) roll=-90;
  if (roll>90) roll=90;
	 
	if (roll_const<-90) roll_const=-90;
  if (roll_const>90) roll_const=90;

	if (roll_amplitude<-90) roll_amplitude=-90;
  if (roll_amplitude>90) roll_amplitude=90;
	 
	 
	float roll_len=r1*0.7;
	 
  roll_x[0]=cx1;
	roll_y[0]=cy1;	 
	roll_x[1]=cx1+roll_len*cos(M_PI/180.0*(-roll+90-10));   
	roll_y[1]=cy1+roll_len*sin(M_PI/180.0*(-roll+90-10));
	roll_x[2]=cx1+roll_len*cos(M_PI/180.0*(-roll+90+10));   
	roll_y[2]=cy1+roll_len*sin(M_PI/180.0*(-roll+90+10));
	roll_x[3]=cx1+(r1-1)*cos(M_PI/180.0*(-roll+90));   
	roll_y[3]=cy1+(r1-1)*sin(M_PI/180.0*(-roll+90));
	 
	cDisplayStandardLibrary.FillTriangle(roll_x[0],roll_y[0],roll_x[1],roll_y[1],roll_x[2],roll_y[2],IDisplay::COLOR_YELLOW); 
	cDisplayStandardLibrary.FillTriangle(roll_x[1],roll_y[1],roll_x[2],roll_y[2],roll_x[3],roll_y[3],IDisplay::COLOR_YELLOW); 	 
	 

	float pitch_y[4];
	float pitch_x[4];
  float pitch=sValue.CurrentAngle[1];
  float pitch_const=sValue.ConstAngle[1];
  float pitch_amplitude=sValue.AmplitudeAngle[1];
  float pitch_period=sValue.PeriodAngle[1];
	if (pitch<-90) pitch=-90;
  if (pitch>90) pitch=90;
	
	if (pitch_const<-90) pitch_const=-90;
  if (pitch_const>90) pitch_const=90;

	if (pitch_amplitude<-90) pitch_amplitude=-90;
  if (pitch_amplitude>90) pitch_amplitude=90;

	float pitch_len=r1*0.7;
	 
  pitch_x[0]=cx2;
	pitch_y[0]=cy2;	 
	pitch_x[1]=cx2+pitch_len*cos(M_PI/180.0*(-pitch+0-10));   
	pitch_y[1]=cy2+pitch_len*sin(M_PI/180.0*(-pitch+0-10));
	pitch_x[2]=cx2+pitch_len*cos(M_PI/180.0*(-pitch+0+10));   
	pitch_y[2]=cy2+pitch_len*sin(M_PI/180.0*(-pitch+0+10));
	pitch_x[3]=cx2+(r2-1)*cos(M_PI/180.0*(-pitch+0));   
	pitch_y[3]=cy2+(r2-1)*sin(M_PI/180.0*(-pitch+0));
	 
	cDisplayStandardLibrary.FillTriangle(pitch_x[0],pitch_y[0],pitch_x[1],pitch_y[1],pitch_x[2],pitch_y[2],IDisplay::COLOR_YELLOW); 
	cDisplayStandardLibrary.FillTriangle(pitch_x[1],pitch_y[1],pitch_x[2],pitch_y[2],pitch_x[3],pitch_y[3],IDisplay::COLOR_YELLOW); 	 
	 
	char str_roll[10];
	sprintf(str_roll,"%.1f",roll);
  cDisplayStandardLibrary.PutString((160-strlen(str_roll)*CDisplayStandardLibrary::FONT_WIDTH*2)/2,CDisplayStandardLibrary::FONT_HEIGHT*2,str_roll,IDisplay::COLOR_CYAN,2);

  char str_roll_const[20]="Пст:-";
	char str_roll_amplitude[20]="Амп:-";
	char str_roll_period[20]="Период:-";
	if (sValue.ConstAngleEnabled[0]==true) sprintf(str_roll_const,"Пст:%.1f",roll_const);
	if (sValue.AmplitudeAngleEnabled[0]==true) sprintf(str_roll_amplitude,"Амп:%.1f",roll_amplitude);
	if (sValue.PeriodAngleEnabled[0]==true) sprintf(str_roll_period,"Период:%.1f с",roll_period);
  cDisplayStandardLibrary.PutString(0,CDisplayStandardLibrary::FONT_HEIGHT*4,str_roll_const,IDisplay::COLOR_GREEN,1);
  cDisplayStandardLibrary.PutString(80,CDisplayStandardLibrary::FONT_HEIGHT*4,str_roll_amplitude,IDisplay::COLOR_GREEN,1);
  cDisplayStandardLibrary.PutString(0,CDisplayStandardLibrary::FONT_HEIGHT*5,str_roll_period,IDisplay::COLOR_GREEN,1);

	char str_pitch[10];
	sprintf(str_pitch,"%.1f",pitch);
  cDisplayStandardLibrary.PutString(160+(160-strlen(str_pitch)*CDisplayStandardLibrary::FONT_WIDTH*2)/2,CDisplayStandardLibrary::FONT_HEIGHT*2,str_pitch,IDisplay::COLOR_CYAN,2);
	
  char str_pitch_const[20]="Пст:-";
	char str_pitch_amplitude[20]="Амп:-";
	char str_pitch_period[20]="Период:-";
	if (sValue.ConstAngleEnabled[1]==true) sprintf(str_pitch_const,"Пст:%.1f",pitch_const);
	if (sValue.AmplitudeAngleEnabled[1]==true) sprintf(str_pitch_amplitude,"Амп:%.1f",pitch_amplitude);
	if (sValue.PeriodAngleEnabled[1]==true) sprintf(str_pitch_period,"Период:%.1f с",pitch_period);
  cDisplayStandardLibrary.PutString(160,CDisplayStandardLibrary::FONT_HEIGHT*4,str_pitch_const,IDisplay::COLOR_GREEN,1);
  cDisplayStandardLibrary.PutString(160+80,CDisplayStandardLibrary::FONT_HEIGHT*4,str_pitch_amplitude,IDisplay::COLOR_GREEN,1);
  cDisplayStandardLibrary.PutString(160,CDisplayStandardLibrary::FONT_HEIGHT*5,str_pitch_period,IDisplay::COLOR_GREEN,1);

	//sprintf(str,"Температура:%.1f",sValue.Temper);
	//cDisplayStandardLibrary.PutString(0,0,str,IDisplay::COLOR_YELLOW,2);
	
	/*
	sprintf(str,"Крен,А:%.1f Тангаж,А:%.1f",sValue.AmplitudeAngle[0],sValue.AmplitudeAngle[1]);
	cDisplayStandardLibrary.PutString(0,48,str,IDisplay::COLOR_YELLOW);
	sprintf(str,"Крен,C:%.1f Тангаж,C:%.1f",sValue.ConstAngle[0],sValue.ConstAngle[1]);
	cDisplayStandardLibrary.PutString(0,64,str,IDisplay::COLOR_YELLOW);
	sprintf(str,"Крен,T:%.1f Тангаж,T:%.1f",sValue.PeriodAngle[0],sValue.PeriodAngle[1]);
	cDisplayStandardLibrary.PutString(0,96,str,IDisplay::COLOR_YELLOW);
	 
  cDisplayStandardLibrary.DrawCircle(160,120,100,IDisplay::COLOR_MAGENTA);
	 */
	 
  //отображаем на экране  	 
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
	
	cDisplayStandardLibrary.FillTriangle(roll_x[0],roll_y[0],roll_x[1],roll_y[1],roll_x[2],roll_y[2],IDisplay::COLOR_BLACK); 
	cDisplayStandardLibrary.FillTriangle(roll_x[1],roll_y[1],roll_x[2],roll_y[2],roll_x[3],roll_y[3],IDisplay::COLOR_BLACK);

	cDisplayStandardLibrary.FillTriangle(pitch_x[0],pitch_y[0],pitch_x[1],pitch_y[1],pitch_x[2],pitch_y[2],IDisplay::COLOR_BLACK); 
	cDisplayStandardLibrary.FillTriangle(pitch_x[1],pitch_y[1],pitch_x[2],pitch_y[2],pitch_x[3],pitch_y[3],IDisplay::COLOR_BLACK); 	 
	
  cDisplayStandardLibrary.PutString((160-strlen(str_roll)*CDisplayStandardLibrary::FONT_WIDTH*2)/2,CDisplayStandardLibrary::FONT_HEIGHT*2,str_roll,IDisplay::COLOR_BLACK,2);
  cDisplayStandardLibrary.PutString(160+(160-strlen(str_pitch)*CDisplayStandardLibrary::FONT_WIDTH*2)/2,CDisplayStandardLibrary::FONT_HEIGHT*2,str_pitch,IDisplay::COLOR_BLACK,2);

  cDisplayStandardLibrary.PutString(0,CDisplayStandardLibrary::FONT_HEIGHT*4,str_roll_const,IDisplay::COLOR_BLACK,1);
  cDisplayStandardLibrary.PutString(80,CDisplayStandardLibrary::FONT_HEIGHT*4,str_roll_amplitude,IDisplay::COLOR_BLACK,1);
  cDisplayStandardLibrary.PutString(0,CDisplayStandardLibrary::FONT_HEIGHT*5,str_roll_period,IDisplay::COLOR_BLACK,1);
  cDisplayStandardLibrary.PutString(160,CDisplayStandardLibrary::FONT_HEIGHT*4,str_pitch_const,IDisplay::COLOR_BLACK,1);
  cDisplayStandardLibrary.PutString(160+80,CDisplayStandardLibrary::FONT_HEIGHT*4,str_pitch_amplitude,IDisplay::COLOR_BLACK,1);
  cDisplayStandardLibrary.PutString(160,CDisplayStandardLibrary::FONT_HEIGHT*5,str_pitch_period,IDisplay::COLOR_BLACK,1);
 } 
}

//----------------------------------------------------------------------------------------------------
//инициализация тактового генератора
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
//инициализация таймера
//----------------------------------------------------------------------------------------------------
void TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance=TIM1;
  htim1.Init.Prescaler=6400;
  htim1.Init.CounterMode=TIM_COUNTERMODE_UP;
  htim1.Init.Period=210;
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
//обработчик таймера (вызывается 125 раз в секунду)
//----------------------------------------------------------------------------------------------------
//void HAL_SYSTICK_Callback(void)	
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
 if (htim->Instance!=TIM1)	return;

 //запрашиваем скорости изменения углов
 int16_t raw_gx;
 int16_t raw_gy;
 int16_t raw_gz;
 //запрашиваем ускорения по осям
 int16_t raw_ax;
 int16_t raw_ay;
 int16_t raw_az;
 //запрашиваем температуру
 int16_t raw_temper;
 MPU6050_ReadAll(raw_gx,raw_gy,raw_gz,raw_ax,raw_ay,raw_az,raw_temper);

 static const float dt=1.0f/125.0f;
	
 cMathProcessing.NewData(dt,raw_gx,raw_gy,raw_gz,raw_ax,raw_ay,raw_az,raw_temper);
}


//----------------------------------------------------------------------------------------------------
//локальный обработчик ошибок
//----------------------------------------------------------------------------------------------------
void ErrorHandler(void)
{
 while(1)
 {
 }
}
//----------------------------------------------------------------------------------------------------
//глобальный обработчик ошибок
//----------------------------------------------------------------------------------------------------
void _Error_Handler(char *file,int line)
{
 ErrorHandler();
}
