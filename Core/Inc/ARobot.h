//
// Created by zhanng on 2025/5/19.
//

#ifndef FOURAXISPLANARTWOLINKROBOTICARM_AROBOT_H
#define FOURAXISPLANARTWOLINKROBOTICARM_AROBOT_H

#include "arm.h"

void initArm(void);
void initArmForRTOS(void);
void moveArm(uint16_t x, uint16_t y, uint16_t z);
void moveArmForRTOS(uint16_t x, uint16_t y, uint16_t z);



#endif //FOURAXISPLANARTWOLINKROBOTICARM_AROBOT_H
