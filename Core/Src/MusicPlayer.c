#include "MusicPlayer.h"

void MusicPlayer_Init(){
	HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
}

// 播放音符函数
void PlayNote(uint16_t freq, uint16_t duration) {
    if (freq == 0) { // 休止符处理
        __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 0); // 设置占空比为0（静音）
        HAL_Delay(duration);
    } else {
        // 更新PWM频率
        __HAL_TIM_SET_AUTORELOAD(&htim13, freq);
        __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, freq / 2); // 50%占空比
        
        // 应用新频率（立即生效）
        HAL_TIM_GenerateEvent(&htim13, TIM_EVENTSOURCE_UPDATE);
        
        HAL_Delay(duration); // 持续指定时长
    }
}

// 演奏音乐函数
void PlayMusic(const struct MusicNote Score[], uint16_t  ScoreLength) {
    for (uint16_t i = 0; i <  ScoreLength; i++) {
        PlayNote(Score[i].Frq, Score[i].length);
    }
    __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 0); // 播放结束关闭声音
}

// 演奏音乐函数
void playSpScoreNote(struct MusicNote Score[],uint16_t ScoreLength,uint16_t from,uint16_t to) {
    if (to >=  ScoreLength) to = ScoreLength;
	for (uint16_t i = from; i <  to; i++) {
        PlayNote(Score[i].Frq, Score[i].length);
    }
    __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 0); // 播放结束关闭声音
}

//Example:
//PlayMusic(TwinkleLittleStar, sizeof(TwinkleLittleStar)/sizeof(TwinkleLittleStar[0]));

