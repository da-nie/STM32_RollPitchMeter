//****************************************************************************************************
//подключаемые библиотеки
//****************************************************************************************************
#include "cmathprocessing.h"
#include "lffilter.h"
#include "madgwickahrs.h"

//****************************************************************************************************
//глобальные переменные
//****************************************************************************************************

//****************************************************************************************************
//константы
//****************************************************************************************************

//****************************************************************************************************
//макроопределения
//****************************************************************************************************

//****************************************************************************************************
//конструктор и деструктор
//****************************************************************************************************

//----------------------------------------------------------------------------------------------------
//конструктор
//----------------------------------------------------------------------------------------------------
CMathProcessing::CMathProcessing(void)
{ 
 for(size_t n=0;n<AXIS_AMOUNT;n++)
 {	
  sValue.AmplitudeAngle[n]=0;
	sValue.CurrentAngle[n]=0;
  sValue.ConstAngle[n]=0;
  sValue.PeriodAngle[n]=0;
  sValue.Temper=0;	 
  	 	 
  sValue.AmplitudeAngleEnabled[n]=false;
	sValue.CurrentAngleEnabled[n]=false;
  sValue.ConstAngleEnabled[n]=false;
  sValue.PeriodAngleEnabled[n]=false;
 }	
 sValue.Temper=0;
 sValue.TemperEnabled=false; 
 
 static const uint32_t MIN_DUTY_CYCLES=3;//пропускаемые циклы измерения периода

 for(size_t n=0;n<AXIS_AMOUNT;n++)
 {
  sChannel[n].ChangeCMax=true;
	sChannel[n].ChangeCMin=true;
	 
  sChannel[n].CMax=0;
	sChannel[n].CMin=0;
	 
  sChannel[n].CounterMax=0;
	sChannel[n].CounterMin=0;

  sChannel[n].CTMax=0;
	sChannel[n].CTMin=0;
	 
  sChannel[n].Max=0;
	sChannel[n].Min=0;
	 
	sChannel[n].Period=0;
	 
	sChannel[n].CounterMax=MIN_DUTY_CYCLES;
	sChannel[n].CounterMin=MIN_DUTY_CYCLES;
 }
 
 Tick=0;
}
//----------------------------------------------------------------------------------------------------
//деструктор
//----------------------------------------------------------------------------------------------------
CMathProcessing::~CMathProcessing()
{
}

//****************************************************************************************************
//закрытые функции
//****************************************************************************************************

//----------------------------------------------------------------------------------------------------
//конвертировать показания гироскопа в градусы в секунду
//----------------------------------------------------------------------------------------------------
float CMathProcessing::ConvertGyro(int16_t raw)
{
 static const float kg=250.0f/32767.0f;//режим 250 градусов/секунда
 float ret=raw;
 ret*=kg;	
 return(ret);
}
//----------------------------------------------------------------------------------------------------
//конвертировать показания акселерометра в м/с^2
//----------------------------------------------------------------------------------------------------
float CMathProcessing::ConvertAcs(int16_t raw)
{
 static const float ka=(2.0f*9.8f)/32767.0f;//режим 2G	
 float ret=raw;
 ret*=ka;	
 return(ret);
}
//----------------------------------------------------------------------------------------------------
//конвертировать показания температуры в градусы Цельсия
//----------------------------------------------------------------------------------------------------
float CMathProcessing::ConvertTemper(int16_t raw)
{
 static const float kt=0.1;
 float ret=raw;
 ret*=kt;	
 return(ret);
}

//----------------------------------------------------------------------------------------------------
//поиск минимума и максимума колебаний
//----------------------------------------------------------------------------------------------------
void CMathProcessing::FindMaxMinOscillation(const float &angle,SChannel &sChannel_Current,size_t tick,float dt)
{
 //считается, что колебания проходят через ноль с допуском на минимальные колебания.
 //при переходе через ноль начинает обновляться значение минимума или максимума (в зависимости от половины,куда мы попадаем).
 //при переходе через ноль сбрасываются результаты минимума или максимума в противоположной половине.
 //при обновлении максимумов/минимумов запоминается их время,по	которому вычисляется период ( делается пропуск первых нескольких периодов).
	
 static const float MIN_ANGLE_OSC=0.5f;//минимальный допуск по углу качания	

 if (angle<-MIN_ANGLE_OSC)
 {
  if (angle<sChannel_Current.CMin) //обновляем минимум
  {
   sChannel_Current.CMin=angle;
   sChannel_Current.CTMin=Tick;
  }
  if (sChannel_Current.ChangeCMax==true)//делаем перенос положения максимума
  {
   sChannel_Current.Max=sChannel_Current.CMax;	 
	 sChannel_Current.TBeginMax=sChannel_Current.TEndMax;//сдвигаем старое время
	 sChannel_Current.TEndMax=sChannel_Current.CTMax;//задаём новое
	 if (sChannel_Current.CounterMax>0) sChannel_Current.CounterMax--;
	 if (sChannel_Current.CounterMax==0) sChannel_Current.Period=(sChannel_Current.TEndMax-sChannel_Current.TBeginMax)*dt;
	}
	sChannel_Current.CMax=0;
	sChannel_Current.ChangeCMax=false;
	sChannel_Current.ChangeCMin=true;
 }
 if (angle>MIN_ANGLE_OSC)
 {
  if (angle>sChannel_Current.CMax)//обновляем максимум
  {
	 sChannel_Current.CMax=angle;
	 sChannel_Current.CTMax=Tick;
	}
  if (sChannel_Current.ChangeCMin==true)//делаем перенос положения минимума
  {
	 sChannel_Current.Min=sChannel_Current.CMin;
	 sChannel_Current.TBeginMin=sChannel_Current.TEndMin;//сдвигаем старое время
	 sChannel_Current.TEndMin=sChannel_Current.CTMin;//задаём новое
	 if (sChannel_Current.CounterMin>0) sChannel_Current.CounterMin--;
	 if (sChannel_Current.CounterMin==0) sChannel_Current.Period=(sChannel_Current.TEndMin-sChannel_Current.TBeginMin)*dt;
	}
	sChannel_Current.CMin=0;
	sChannel_Current.ChangeCMin=false;
	sChannel_Current.ChangeCMax=true;
 }
}

//----------------------------------------------------------------------------------------------------
//поиск параметров движения
//----------------------------------------------------------------------------------------------------
void CMathProcessing::FindMoveParam(size_t tick,float dt)
{
 static const float MAX_TIME_NO_CHANGE_MAX_MIN_SEC=50.0f;//максимальное время без измерения максимума или минимума в секундах
 const size_t MAX_TICK_NO_CHANGE_MAX_MIN=(size_t)(MAX_TIME_NO_CHANGE_MAX_MIN_SEC/dt);
	
 for(size_t n=0;n<AXIS_AMOUNT;n++)
 {
	const float &angle=sValue.CurrentAngle[n];
	SChannel &sChannel_Current=sChannel[n];
  FindMaxMinOscillation(angle,sChannel_Current,tick,dt);
	//обновляем показания
  if (sChannel_Current.CounterMax==0 || sChannel_Current.CounterMin==0)
  {
	 //постоянное смещение
	 if ((tick-sChannel_Current.TBeginMin)>MAX_TICK_NO_CHANGE_MAX_MIN && (tick-sChannel_Current.TBeginMax)>MAX_TICK_NO_CHANGE_MAX_MIN)
	 {
		sValue.ConstAngle[n]=0;
    sValue.ConstAngleEnabled[n]=false;		 
		 
		sValue.AmplitudeAngle[n]=0;
    sValue.AmplitudeAngleEnabled[n]=false;		 

		sValue.PeriodAngle[n]=0;
    sValue.PeriodAngleEnabled[n]=false;		 
	 }
	 else
	 {
    float const_a=(sChannel_Current.Max+sChannel_Current.Min)/2.0f;//постоянное смещение		 
		float amp=(sChannel_Current.Max-sChannel_Current.Min)/2.0f;
		float t_a=sChannel_Current.Period;
		 
    sValue.ConstAngle[n]=const_a;
	  sValue.ConstAngleEnabled[n]=true;
		 
		sValue.AmplitudeAngle[n]=amp;
    sValue.AmplitudeAngleEnabled[n]=true;
		
		sValue.PeriodAngle[n]=t_a;
    sValue.PeriodAngleEnabled[n]=true;
	 }
  }
	else
	{
   sValue.ConstAngle[n]=0;
   sValue.ConstAngleEnabled[n]=false;		 
		 
	 sValue.AmplitudeAngle[n]=0;
   sValue.AmplitudeAngleEnabled[n]=false;		 

	 sValue.PeriodAngle[n]=0;
   sValue.PeriodAngleEnabled[n]=false;		 
	}
 }	

}

//****************************************************************************************************
//открытые функции
//****************************************************************************************************

//----------------------------------------------------------------------------------------------------
//передать новые данные
//----------------------------------------------------------------------------------------------------
void CMathProcessing::NewData(float dt,int16_t raw_gx,int16_t raw_gy,int16_t raw_gz,int16_t raw_ax,int16_t raw_ay,int16_t raw_az,int16_t raw_temper)
{
 //переводим показания в систему Си	
 float gx=ConvertGyro(raw_gx);
 float gy=ConvertGyro(raw_gy);
 float gz=ConvertGyro(raw_gz);

 float ax=ConvertAcs(raw_ax);
 float ay=ConvertAcs(raw_ay);
 float az=ConvertAcs(raw_az);

 //вычисляем температуру	
 sValue.Temper=ConvertTemper(raw_temper);
 sValue.TemperEnabled=true;

 //вычитаем смещения акселерометров и нормируем их по масштабному коэффициенту	
	
 static float ax0=0.016027411;//смещения нулей акселерометров
 static float ay0=-0.035461063;
 static float az0=-0.686306836;
 static float axs=9.826375802;//масштабный коэффициент акселерометров
 static float ays=9.803080006;
 static float azs=9.79243432;	
 	
 ax=(ax-ax0)/axs;
 ay=(ay-ay0)/ays;
 az=(az-az0)/azs;
 
 //вызываем фильтр

 MadgwickAHRSupdateIMU(dt,GRAD_TO_RAD(gx),GRAD_TO_RAD(gy),GRAD_TO_RAD(gz),ax,ay,az);
 
 float imu[3];
 float quat[4]; 
 
 quat[0]=q0;
 quat[1]=q1;
 quat[2]=q2;
 quat[3]=q3;
 quat2Euler(&quat[0],&imu[0]);

 //угол 3 (рысканье) без магнитометра будет вращаться

 //переводим результаты в градусы с НЧ-фильтром
 sValue.CurrentAngle[0]=LFFILTER_FilterChannel1(RAD_TO_GRAD(imu[0]));
 sValue.CurrentAngle[1]=LFFILTER_FilterChannel2(RAD_TO_GRAD(imu[1]));
 sValue.CurrentAngle[2]=LFFILTER_FilterChannel3(RAD_TO_GRAD(imu[2]));
 
 sValue.CurrentAngleEnabled[0]=true;
 sValue.CurrentAngleEnabled[1]=true;
 sValue.CurrentAngleEnabled[2]=true;
 
 //выполняем поиск параметров движения
 FindMoveParam(Tick,dt);
  
 Tick++;
}
//----------------------------------------------------------------------------------------------------
//получить результаты расчетов
//----------------------------------------------------------------------------------------------------
CMathProcessing::SValue CMathProcessing::GetValue(void)
{
 return(sValue);	
}
