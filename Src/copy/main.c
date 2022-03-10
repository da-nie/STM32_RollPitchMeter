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
#include "madgwickahrs.h"

#include <math.h>
#include <stdlib.h>

//****************************************************************************************************
//макроопределения
//****************************************************************************************************

#define M_PI 3.1415926535897932384626433832795
#define GRAD_TO_RAD(value) (M_PI/180.0*value)
#define RAD_TO_GRAD(value) (180.0*value/M_PI)

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

static TIM_HandleTypeDef htim1;

struct SParam
{
 float Angle[3];
 float ConstAngle[3];
 float AmpAngle[3];	 
 float PeriodAngle[3];
 float Temper;//температура
};
volatile SParam sParam;

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
	
 for(size_t n=0;n<3;n++)
 {	
  sParam.Angle[n]=0;
	sParam.ConstAngle[n]=0;
	sParam.AmpAngle[n]=0;
  sParam.PeriodAngle[n]=0;	 
 }
 
 while(1)
 {	 
  cDisplayStandardLibrary.Clear(IDisplay::COLOR_BLACK);
	
	char str[100];
	sprintf(str,"Температура:%.1f",sParam.Temper);
	cDisplayStandardLibrary.PutString(0,0,str,IDisplay::COLOR_YELLOW);
	sprintf(str,"Крен:%.1f Тангаж:%.1f",sParam.Angle[0],sParam.Angle[1]);
	cDisplayStandardLibrary.PutString(0,24,str,IDisplay::COLOR_YELLOW);
	sprintf(str,"Крен,А:%.1f Тангаж,А:%.1f",sParam.AmpAngle[0],sParam.AmpAngle[1]);
	cDisplayStandardLibrary.PutString(0,48,str,IDisplay::COLOR_YELLOW);
	sprintf(str,"Крен,C:%.1f Тангаж,C:%.1f",sParam.ConstAngle[0],sParam.ConstAngle[1]);
	cDisplayStandardLibrary.PutString(0,48,str,IDisplay::COLOR_YELLOW);
	sprintf(str,"Крен,T:%.1f Тангаж,T:%.1f",sParam.PeriodAngle[0],sParam.PeriodAngle[1]);
	cDisplayStandardLibrary.PutString(0,64,str,IDisplay::COLOR_YELLOW);
	
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

	 
 }
	
 	
	
	
	/*
 cDisplayStandardLibrary.PutString(0,0,"Тест печати",IDisplay::COLOR_YELLOW);

 cDisplayStandardLibrary.DrawLine(0,0,209,309,IDisplay::COLOR_BLUE);	
 cDisplayStandardLibrary.FillRectangle(30,40,50,90,IDisplay::COLOR_RED);
 cDisplayStandardLibrary.FillTriangle(10,10,50,40,10,120,IDisplay::COLOR_GREEN); 
		
 static const uint32_t QUART_SIZE=(IDisplay::DISPLAY_HEIGHT*IDisplay::DISPLAY_WIDTH)>>4;	
	
 while (1)
 {	 
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
 }
 */
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
  htim1.Init.Prescaler=1000;
  htim1.Init.CounterMode=TIM_COUNTERMODE_UP;
  htim1.Init.Period=1000;
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



#define Ntap 31

float fir1(float NewSample) {
	//return(NewSample);

    float FIRCoef[Ntap] = { 
        0.02139502659757795000,
        0.02273550034409449500,
        0.02411531931674383200,
        0.02553359910121268500,
        0.02698931267274278300,
        0.02848128945741259700,
        0.03000820845422270600,
        0.03156859783510958300,
        0.03316082863575674900,
        0.03478311499438121300,
        0.03643350851604675700,
        0.03810989925192471300,
        0.03981001084289421300,
        0.04153140234128065200,
        0.04327146423910160200,
        0.04414583479899514300,
        0.04327146423910160200,
        0.04153140234128065200,
        0.03981001084289421300,
        0.03810989925192471300,
        0.03643350851604675700,
        0.03478311499438121300,
        0.03316082863575674900,
        0.03156859783510958300,
        0.03000820845422270600,
        0.02848128945741259700,
        0.02698931267274278300,
        0.02553359910121268500,
        0.02411531931674383200,
        0.02273550034409449500,
        0.02139502659757795000
    };

    static float x[Ntap]; //input samples
    float y=0;            //output sample
    int n;

    //shift the old samples
    for(n=Ntap-1; n>0; n--)
       x[n] = x[n-1];

    //Calculate the new output
    x[0] = NewSample;
    for(n=0; n<Ntap; n++)
        y += FIRCoef[n] * x[n];
    
    return y;
}

float fir2(float NewSample) {
	//return(NewSample);
    float FIRCoef[Ntap] = { 
        0.02139502659757795000,
        0.02273550034409449500,
        0.02411531931674383200,
        0.02553359910121268500,
        0.02698931267274278300,
        0.02848128945741259700,
        0.03000820845422270600,
        0.03156859783510958300,
        0.03316082863575674900,
        0.03478311499438121300,
        0.03643350851604675700,
        0.03810989925192471300,
        0.03981001084289421300,
        0.04153140234128065200,
        0.04327146423910160200,
        0.04414583479899514300,
        0.04327146423910160200,
        0.04153140234128065200,
        0.03981001084289421300,
        0.03810989925192471300,
        0.03643350851604675700,
        0.03478311499438121300,
        0.03316082863575674900,
        0.03156859783510958300,
        0.03000820845422270600,
        0.02848128945741259700,
        0.02698931267274278300,
        0.02553359910121268500,
        0.02411531931674383200,
        0.02273550034409449500,
        0.02139502659757795000
    };

    static float x[Ntap]; //input samples
    float y=0;            //output sample
    int n;

    //shift the old samples
    for(n=Ntap-1; n>0; n--)
       x[n] = x[n-1];

    //Calculate the new output
    x[0] = NewSample;
    for(n=0; n<Ntap; n++)
        y += FIRCoef[n] * x[n];
    
    return y;
}

float fir3(float NewSample) {
	//return(NewSample);
    float FIRCoef[Ntap] = { 
        0.02139502659757795000,
        0.02273550034409449500,
        0.02411531931674383200,
        0.02553359910121268500,
        0.02698931267274278300,
        0.02848128945741259700,
        0.03000820845422270600,
        0.03156859783510958300,
        0.03316082863575674900,
        0.03478311499438121300,
        0.03643350851604675700,
        0.03810989925192471300,
        0.03981001084289421300,
        0.04153140234128065200,
        0.04327146423910160200,
        0.04414583479899514300,
        0.04327146423910160200,
        0.04153140234128065200,
        0.03981001084289421300,
        0.03810989925192471300,
        0.03643350851604675700,
        0.03478311499438121300,
        0.03316082863575674900,
        0.03156859783510958300,
        0.03000820845422270600,
        0.02848128945741259700,
        0.02698931267274278300,
        0.02553359910121268500,
        0.02411531931674383200,
        0.02273550034409449500,
        0.02139502659757795000
    };

    static float x[Ntap]; //input samples
    float y=0;            //output sample
    int n;

    //shift the old samples
    for(n=Ntap-1; n>0; n--)
       x[n] = x[n-1];

    //Calculate the new output
    x[0] = NewSample;
    for(n=0; n<Ntap; n++)
        y += FIRCoef[n] * x[n];
    
    return y;
}


 float max[3]={0,0,0};//запомненный максимум
 float min[3]={0,0,0};//запомненный минимум

 float cmax[3]={0,0,0};//текущий максимум
 float cmin[3]={0,0,0};//текущий минимум

 float ctmax[3]={0,0,0};//запомненное время максимума
 float ctmin[3]={0,0,0};//запомненное время минимума

 float tbeginmin[3]={0,0,0};//время первого горба
 float tendmin[3]={0,0,0};//время следующего горба

 float tbeginmax[3]={0,0,0};//время первого горба
 float tendmax[3]={0,0,0};//время следующего горба

 float period[3]={0,0,0};//период

 bool change_cmin[3]={true,true,true};//разрешение на смену минимума
 bool change_cmax[3]={true,true,true};//разрешение на смену максимума
 
 uint32_t countermin[3]={0,0,0};
 uint32_t countermax[3]={0,0,0};


 
//----------------------------------------------------------------------------------------------------
//системный такт (вызывается 1000 раз в секунду)
//----------------------------------------------------------------------------------------------------
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//void HAL_SYSTICK_Callback(void)	
{
 sParam.Temper+=1;
 return;
	
 //if (htim->Instance!=TIM1)	return;
	
//void HAL_SYSTICK_Callback(void)
//{
 static size_t tick=0;	
 	
 //static uint16_t counter=0;	
 //counter++;	
 //counter%=8;
 //if (counter!=0) return;	
 //частота вызова 125 Гц
 tick++;
 //запрашиваем скорости изменения углов
 int16_t raw_gx=MPU6050_GetGyroX();
 int16_t raw_gy=MPU6050_GetGyroY();
 int16_t raw_gz=MPU6050_GetGyroZ();
 //запрашиваем ускорения по осям
 int16_t raw_ax=MPU6050_GetAcselX();
 int16_t raw_ay=MPU6050_GetAcselY();
 int16_t raw_az=MPU6050_GetAcselZ();
 //запрашиваем температуру
 int16_t raw_temper=MPU6050_GetTemper();

 sParam.Temper=raw_temper;
 sParam.Temper/=10.0f;

 float gx=raw_gx;
 float gy=raw_gy;
 float gz=raw_gz;
 //переводим в градусы в секунду
 static const float kg=250.0f/32768.0f;//режим 250 градусов/секунда
 gx*=kg;
 gy*=kg;
 gz*=kg;
 //вычисляем ускорения по осям
 static const float ka=(2.0f*9.8f)/32768.0f;//режим 2G
 float ax=raw_ax;
 float ay=raw_ay;
 float az=raw_az;
 ax*=ka;
 ay*=ka;
 az*=ka;
 //применяем фильтр
 //угол a3-рысканье постоянно убегает. Так и должно быть без компаса.
 float imu[3];
 float quat[4];

 float oax=0.016027411;//смещения нулей акселерометров
 float oay=-0.035461063;
 float oaz=-0.686306836;
 float sax=9.826375802;//масштабный коэффициент
 float say=9.803080006;
 float saz=9.79243432;

 static const float dt=1.0f/125.0f;

 MadgwickAHRSupdateIMU(dt,GRAD_TO_RAD(gx),GRAD_TO_RAD(gy),GRAD_TO_RAD(gz),(ax-oax)/sax,(ay-oay)/say,(az-oaz)/saz);
 quat[0]=q0;
 quat[1]=q1;
 quat[2]=q2;
 quat[3]=q3;
 quat2Euler(&quat[0],&imu[0]);

 float angle[3];
 
 angle[0]=fir1(RAD_TO_GRAD(imu[0]));
 angle[1]=fir2(RAD_TO_GRAD(imu[1]));
 angle[2]=fir3(RAD_TO_GRAD(imu[2]));


 //считаем периоды и амплитуды колебаний

 for(size_t n=0;n<3;n++)
 {
  sParam.Angle[n]=angle[n];	 
  if (angle[n]<-0.5)
  {
   if (angle[n]<cmin[n]) //обновляем минимум
   {
    cmin[n]=angle[n];
    ctmin[n]=tick*dt;
   }
   if (change_cmax[n]==true)//делаем перенос положения максимума
	 {
	  max[n]=cmax[n];	 
	  tbeginmax[n]=tendmax[n];//сдвигаем старое время
	  tendmax[n]=ctmax[n];//задаём новое
	  if (countermax[n]<3) countermax[n]++;
	  if (countermax[n]==3) period[n]=tendmax[n]-tbeginmax[n];
	 }
	 cmax[n]=0;
	 change_cmax[n]=false;
	 change_cmin[n]=true;
  }
  if (angle[n]>0.5)
  {
   if (angle[n]>cmax[n])//обновляем максимум
   {
	  cmax[n]=angle[n];
	  ctmax[n]=tick*dt;
	 }
	 if (change_cmin[n]==true)//делаем перенос положения минимума
	 {
	  min[n]=cmin[n];
	  tbeginmin[n]=tendmin[n];//сдвигаем старое время
	  tendmin[n]=ctmin[n];//задаём новое
	  if (countermin[n]<3) countermin[n]++;
	  if (countermin[n]==3) period[n]=tendmin[n]-tbeginmin[n];
	 }
	 cmin[n]=0;
	 change_cmin[n]=false;
	 change_cmax[n]=true;
  }

  if (countermax[n]==3 || countermin[n]==3)
  {
   float const_a=(max[n]+min[n])/2;

	 if ((tick*dt-tbeginmin[n])>50 && (tick*dt-tbeginmax[n])>50) const_a=0;
	 	
	 sParam.ConstAngle[n]=const_a;
   float amp=(max[n]-min[n])/2;
   if ((tick*dt-tbeginmin[n])>50 && (tick*dt-tbeginmax[n])>50) amp=0;
   sParam.AmpAngle[n]=amp;
   float t_a=period[n];
   if ((tick*dt-tbeginmin[n])>50 && (tick*dt-tbeginmax[n])>50) t_a=0;
   sParam.PeriodAngle[n]=t_a; 
  }
 }	
	
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