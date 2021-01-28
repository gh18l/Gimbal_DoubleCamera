#ifndef __LED_H__
#define __LED_H__

/*----LED_GREEN----PC1-----'0' is on,'1' is off */
/*----LED_RED------PC2-----'0' is on,'1' is off */

void LED_Configuration(void);

#define  LED_GREEN_ON()      GPIO_ResetBits(GPIOC, GPIO_Pin_1)
#define  LED_GREEN_OFF()     GPIO_SetBits(GPIOC, GPIO_Pin_1)
#define  LED_GREEN_TOGGLE()  GPIO_ToggleBits(GPIOC, GPIO_Pin_1)

#define  LED_RED_ON()        GPIO_ResetBits(GPIOC, GPIO_Pin_2)
#define  LED_RED_OFF()       GPIO_SetBits(GPIOC, GPIO_Pin_2)
#define  LED_RED_TOGGLE()    GPIO_ToggleBits(GPIOC, GPIO_Pin_2)

#endif
