#ifndef __TIM5_H__
#define __TIM5_H__

#define PWM1  TIM5->CCR1
#define PWM2  TIM5->CCR2   //���2500  ȡ2400

void TIM5_Configuration(void);

#endif
