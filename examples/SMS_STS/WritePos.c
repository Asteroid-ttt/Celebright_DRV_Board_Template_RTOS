#include "main.h"
#include <../../SCSLib/SCServo.h>

#define ANGLE_PRECISION 0.4f    // 舵机角度精度(度)
#define DELAY_MS       500       // 打点间隔(ms)
#define DISTANCE       1.0f      // 云台到平面距离(m)
#define MIN_RADIUS     10.0f     //最小有效半径





void setup(void)
{
	setEnd(1);//SCSCL舵机为大端存储结构
}

void examples(void)
{
	//舵机(ID1)以最高速度V=1500*0.059=88.5rpm，运行至P1=1000位置
	WritePos(1, 1000, 0, 1500);
	HAL_Delay((1000-20)*1000/(1500) + 100);//[(P1-P0)/V]*1000 + 100
	
	//舵机(ID1)以最高速度V=1500*0.059=88.5rpm，运行至P0=20位置
	WritePos(1, 20, 0, 1500);
	HAL_Delay((1000-20)*1000/(1500) + 100);//[(P1-P0)/V]*1000 + 100
}

void examples1(void)
{
	int Pos;
  int Speed;
  
	FeedBack(1);
  if(!getLastError()){
    Pos = ReadPos(-1);
    Speed = ReadSpeed(-1);
    
		
    HAL_Delay(10);
  }
  else{
	//	printf("FeedBack err\n");
    HAL_Delay(2000);
  }
  Pos = ReadPos(1);
  if(!getLastError()){
   // printf("Servo position:%d\n", Pos);
    HAL_Delay(10);
  }else{
    //printf("read position err\n");
    HAL_Delay(500);
  }
  
}
