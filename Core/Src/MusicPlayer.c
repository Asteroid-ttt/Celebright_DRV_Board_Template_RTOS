#include "app.h"
#include "MusicPlayer.h"
#if APP_ENABLE_MUSIC

void MusicPlayer_Init(){
	HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
}

// ������������
void PlayNote(uint16_t freq, uint16_t duration) {
    if (freq == 0) { // ��ֹ������
        __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 0); // ����ռ�ձ�Ϊ0��������
        HAL_Delay(duration);
    } else {
        // ����PWMƵ��
        __HAL_TIM_SET_AUTORELOAD(&htim13, freq);
        __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, freq / 2); // 50%ռ�ձ�
        
        // Ӧ����Ƶ�ʣ�������Ч��
        HAL_TIM_GenerateEvent(&htim13, TIM_EVENTSOURCE_UPDATE);
        
        HAL_Delay(duration); // ����ָ��ʱ��
    }
}

// �������ֺ���
void PlayMusic(const struct MusicNote Score[], uint16_t  ScoreLength) {
    for (uint16_t i = 0; i <  ScoreLength; i++) {
        PlayNote(Score[i].Frq, Score[i].length);
    }
    __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 0); // ���Ž����ر�����
}

// �������ֺ���
void playSpScoreNote(struct MusicNote Score[],uint16_t ScoreLength,uint16_t from,uint16_t to) {
    if (to >=  ScoreLength) to = ScoreLength;
	for (uint16_t i = from; i <  to; i++) {
        PlayNote(Score[i].Frq, Score[i].length);
    }
    __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 0); // ���Ž����ر�����
}

//Example:
//PlayMusic(TwinkleLittleStar, sizeof(TwinkleLittleStar)/sizeof(TwinkleLittleStar[0]));


#endif /* APP_ENABLE_MUSIC */
