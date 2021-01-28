#include "USART1_Deal.h"
#include "Delay.h"
#include "os.h"

/*-----USART1_RX-----PB7----*/
/*-----DMA2 - Stream2 - Channel4*/ 
/*-----����ң����������-----for D-BUS*/

ST_DBUS g_stDBUS;
bool DUBS_Flag = FALSE;
EN_OPERATION_MODE g_emOperationMode;
extern volatile UCHAR8 sbus_rx_buffer[18];

/*ң�������ݽ���*/
void DMA2_Stream2_IRQHandler(void)
{
	CPU_SR         cpu_sr;	
	OS_CRITICAL_ENTER();                         
    OSIntEnter();
    OS_CRITICAL_EXIT();

    if(DMA_GetITStatus(DMA2_Stream2, DMA_IT_TCIF2))
    {
        DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
        DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2);

        /*ң�������ݽ���*/
		/*��ҡ��*/
		g_stDBUS.stRC.Ch0 = (sbus_rx_buffer[0]| (sbus_rx_buffer[1] << 8)) & 0x07ff;//Channe0����ˮƽͨ��
		g_stDBUS.stRC.Ch1 = ((sbus_rx_buffer[1] >> 3) | (sbus_rx_buffer[2] << 5)) & 0x07ff;//Channe1������ֱͨ��
		/*��ҡ��*/
        g_stDBUS.stRC.Ch2 = ((sbus_rx_buffer[2] >> 6) | (sbus_rx_buffer[3] << 2) | (sbus_rx_buffer[4] << 10)) & 0x07ff;//Channe2����ˮƽͨ��
        g_stDBUS.stRC.Ch3 = ((sbus_rx_buffer[4] >> 1) | (sbus_rx_buffer[5] << 7)) & 0x07ff;//Channe3������ֱͨ��
		/*��3λ����*/
        g_stDBUS.stRC.SW_L = ((sbus_rx_buffer[5] >> 4)& 0x000C) >> 2;//(�ϡ���1���С���3���¡���2)
		/*��3λ����*/
        g_stDBUS.stRC.SW_R = ((sbus_rx_buffer[5] >> 4)& 0x0003);//(�ϡ���1���С���3���¡���2)
		
		/*������ݽ���*/			
        g_stDBUS.stMouse.X = sbus_rx_buffer[6] | (sbus_rx_buffer[7] << 8);// X����
        g_stDBUS.stMouse.Y = sbus_rx_buffer[8] | (sbus_rx_buffer[9] << 8);//Y����
        g_stDBUS.stMouse.Z = sbus_rx_buffer[10] | (sbus_rx_buffer[11] << 8);//Z����
        g_stDBUS.stMouse.Left = sbus_rx_buffer[12];//���״̬��1�����£�0��û���£�
        g_stDBUS.stMouse.Right = sbus_rx_buffer[13];//�Ҽ�״̬��1�����£�0��û���£�

		/*�������ݽ���*/
        g_stDBUS.usKeyboard = sbus_rx_buffer[14] | (sbus_rx_buffer[15] << 8);//����ֵ
        //�������
        DUBS_Flag = TRUE;

		if(g_stDBUS.usKeyboard & KEY_PRESSED_OFFSET_W)
		{
		    /*���¼���W����Ӧ����*/
			
		}
		if(g_stDBUS.usKeyboard & KEY_PRESSED_OFFSET_S)
		{
		    /*���¼���S����Ӧ����*/
			
		}
		if(g_stDBUS.usKeyboard & KEY_PRESSED_OFFSET_A)
		{
		    /*���¼���A����Ӧ����*/
		}
		if(g_stDBUS.usKeyboard & KEY_PRESSED_OFFSET_D)
		{
		    /*���¼���D����Ӧ����*/
			
		}
		if(g_stDBUS.usKeyboard & KEY_PRESSED_OFFSET_Q)
		{
		    /*���¼���Q����Ӧ����*/
			
	    }
		if(g_stDBUS.usKeyboard & KEY_PRESSED_OFFSET_E)
		{
		    /*���¼���E����Ӧ����*/
			
		}
		if(g_stDBUS.usKeyboard & KEY_PRESSED_OFFSET_SHIFT)
		{
		    /*���¼���Shift����Ӧ����*/
			
		}
	    if(g_stDBUS.usKeyboard & KEY_PRESSED_OFFSET_CTRL)
		{
		    /*���¼���Ctrl����Ӧ����*/
			
		}



		/*ң�������Ͻ�3λ�����л�����ģʽ*/
		if(g_stDBUS.stRC.SW_R == RC_SW_UP)
		{
		    g_emOperationMode = KeyMouse_Mode;
		    
		}
		else if(g_stDBUS.stRC.SW_R == RC_SW_MID)
		{
		    g_emOperationMode = RC_Mode;
		}
		else if(g_stDBUS.stRC.SW_R == RC_SW_DOWM)
		{
		    g_emOperationMode = KeyRC_Mode;
		}
    }
	OSIntExit();	
}

