#include "USART1_Deal.h"
#include "Delay.h"
#include "os.h"

/*-----USART1_RX-----PB7----*/
/*-----DMA2 - Stream2 - Channel4*/ 
/*-----接收遥控器的数据-----for D-BUS*/

ST_DBUS g_stDBUS;
bool DUBS_Flag = FALSE;
EN_OPERATION_MODE g_emOperationMode;
extern volatile UCHAR8 sbus_rx_buffer[18];

/*遥控器数据接收*/
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

        /*遥控器数据解码*/
		/*右摇杆*/
		g_stDBUS.stRC.Ch0 = (sbus_rx_buffer[0]| (sbus_rx_buffer[1] << 8)) & 0x07ff;//Channe0——水平通道
		g_stDBUS.stRC.Ch1 = ((sbus_rx_buffer[1] >> 3) | (sbus_rx_buffer[2] << 5)) & 0x07ff;//Channe1——垂直通道
		/*左摇杆*/
        g_stDBUS.stRC.Ch2 = ((sbus_rx_buffer[2] >> 6) | (sbus_rx_buffer[3] << 2) | (sbus_rx_buffer[4] << 10)) & 0x07ff;//Channe2——水平通道
        g_stDBUS.stRC.Ch3 = ((sbus_rx_buffer[4] >> 1) | (sbus_rx_buffer[5] << 7)) & 0x07ff;//Channe3——垂直通道
		/*左3位开关*/
        g_stDBUS.stRC.SW_L = ((sbus_rx_buffer[5] >> 4)& 0x000C) >> 2;//(上——1；中——3；下——2)
		/*右3位开关*/
        g_stDBUS.stRC.SW_R = ((sbus_rx_buffer[5] >> 4)& 0x0003);//(上——1；中——3；下——2)
		
		/*鼠标数据解码*/			
        g_stDBUS.stMouse.X = sbus_rx_buffer[6] | (sbus_rx_buffer[7] << 8);// X坐标
        g_stDBUS.stMouse.Y = sbus_rx_buffer[8] | (sbus_rx_buffer[9] << 8);//Y坐标
        g_stDBUS.stMouse.Z = sbus_rx_buffer[10] | (sbus_rx_buffer[11] << 8);//Z坐标
        g_stDBUS.stMouse.Left = sbus_rx_buffer[12];//左键状态（1：按下；0：没按下）
        g_stDBUS.stMouse.Right = sbus_rx_buffer[13];//右键状态（1：按下；0：没按下）

		/*键盘数据解码*/
        g_stDBUS.usKeyboard = sbus_rx_buffer[14] | (sbus_rx_buffer[15] << 8);//键盘值
        //接收完毕
        DUBS_Flag = TRUE;

		if(g_stDBUS.usKeyboard & KEY_PRESSED_OFFSET_W)
		{
		    /*按下键盘W键相应处理*/
			
		}
		if(g_stDBUS.usKeyboard & KEY_PRESSED_OFFSET_S)
		{
		    /*按下键盘S键相应处理*/
			
		}
		if(g_stDBUS.usKeyboard & KEY_PRESSED_OFFSET_A)
		{
		    /*按下键盘A键相应处理*/
		}
		if(g_stDBUS.usKeyboard & KEY_PRESSED_OFFSET_D)
		{
		    /*按下键盘D键相应处理*/
			
		}
		if(g_stDBUS.usKeyboard & KEY_PRESSED_OFFSET_Q)
		{
		    /*按下键盘Q键相应处理*/
			
	    }
		if(g_stDBUS.usKeyboard & KEY_PRESSED_OFFSET_E)
		{
		    /*按下键盘E键相应处理*/
			
		}
		if(g_stDBUS.usKeyboard & KEY_PRESSED_OFFSET_SHIFT)
		{
		    /*按下键盘Shift键相应处理*/
			
		}
	    if(g_stDBUS.usKeyboard & KEY_PRESSED_OFFSET_CTRL)
		{
		    /*按下键盘Ctrl键相应处理*/
			
		}



		/*遥控器右上角3位开关切换操作模式*/
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

