#ifndef __BUZZER_H
#define __BUZZER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "tim.h"

#define Cal_Frq 12000000 	//Unit:Hz
//MusicNote:
//Note Frequency = CPU_Frq / [defined value];
//H7_ means H7#;
//HHigh-reedit;
#define HH7 	Cal_Frq/3951	//3951 Hz;
#define HH6_ 	Cal_Frq/3729	//3729 Hz;
#define HH6 	Cal_Frq/3520	//3520 Hz;
#define HH5_ 	Cal_Frq/3322	//3322 Hz;
#define HH5 	Cal_Frq/3136 	//3136 Hz;
#define HH4_ 	Cal_Frq/2960	//2960 Hz;
#define HH4 	Cal_Frq/2794 	//2794 Hz;
#define HH3 	Cal_Frq/2637	//2637 Hz;...
#define HH2_ 	Cal_Frq/2489	//2489 Hz;
#define HH2 	Cal_Frq/2349	//2349 Hz;ExHigh-Re
#define HH1_ 	Cal_Frq/2217	//2217.5 Hz;
#define HH1 	Cal_Frq/2093 	//2093 Hz;ExHigh-Do
//High-reedit;
#define H7 	Cal_Frq/1976	//1976 Hz;
#define H6_ Cal_Frq/1865	//1865 Hz;
#define H6 	Cal_Frq/1760	//1760 Hz;
#define H5_ Cal_Frq/1661	//1661 Hz;
#define H5 	Cal_Frq/1568 	//1568 Hz;
#define H4_ Cal_Frq/1480	//1480 Hz;
#define H4 	Cal_Frq/1397 	//1397 Hz;
#define H3 	Cal_Frq/1318	//1318 Hz;...
#define H2_ Cal_Frq/1245	//1245 Hz;
#define H2 	Cal_Frq/1175	//1175 Hz;High-Re
#define H1_ Cal_Frq/1109	//1109 Hz;
#define H1 	Cal_Frq/1046 	//1046 Hz;High-Do
//Mid
#define M7 	Cal_Frq/988	//988 Hz;
#define M6_ Cal_Frq/932 //932 Hz;
#define M6 	Cal_Frq/880	//880 Hz;
#define M5_ Cal_Frq/831	//831 Hz;
#define M5 	Cal_Frq/784 //784 Hz;
#define M4_ Cal_Frq/740	//740 Hz;
#define M4  Cal_Frq/698	//698 Hz;
#define M3 	Cal_Frq/659	//659 Hz;...
#define M2_ Cal_Frq/622	//622 Hz;
#define M2 	Cal_Frq/587	//587 Hz;Mid-Re
#define M1_ Cal_Frq/554	//554 Hz;
#define M1 	Cal_Frq/523 //523 Hz;Mid-Do
//Low
#define L7 	Cal_Frq/494	//494 Hz;
#define L6_ Cal_Frq/466 //466 Hz;
#define L6 	Cal_Frq/440	//440 Hz;
#define L5_ Cal_Frq/415	//415 Hz;
#define L5  Cal_Frq/392	//392 Hz;
#define L4_ Cal_Frq/370	//370 Hz;
#define L4  Cal_Frq/349	//349 Hz;
#define L3 	Cal_Frq/330	//330 Hz;...
#define L2_ Cal_Frq/311 //311 Hz;
#define L2 	Cal_Frq/294	//294 Hz;Low-Re
#define L1_ Cal_Frq/277	//277 Hz;
#define L1  Cal_Frq/262 //262 Hz;Low-Do

//#define
struct MusicNote{
	uint16_t Frq, length;
};		     

//MusicPlayer���ƺ���

void MusicPlayer_Init(void);
void PlayNote(uint16_t freq, uint16_t duration);
void PlayMusic(const struct MusicNote Score[], uint16_t num_notes);
void playSpScoreNote(struct MusicNote Score[],uint16_t ScoreLength,uint16_t from,uint16_t to);

#ifdef __cplusplus
}
#endif

#endif
