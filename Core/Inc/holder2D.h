#ifndef __HOLDER2D_H__
#define __HOLDER2D_H__

#include "SCServo.h"

#define ANGLE_PRECISION 0.4f    // 舵机角度精度(度)
#define DELAY_MS       500       // 打点间隔(ms)
#define DISTANCE       1.0f      // 云台到平面距离(m)
#define MIN_RADIUS     10.0f     //最小有效半径


void setup_holder(void);
void set_servo_angle(uint8_t ID, float angle);
void set_servo_angle_offset(uint8_t ID, float angle);
void set_servo_angle_negative_holder(uint8_t ID, float angle);
void set_xy_position(float x, float y);
void draw_circle(float CENTER_X, float CENTER_Y,float r,int n);

#endif /* __HOLDER2D_H__ */
