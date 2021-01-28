#ifndef __DEBUG_TASK_H__
#define __DEBUG_TASK_H__

typedef union
{
	SINT32 siSave[256];
	FP32 fpSave[256];
}UN_SAVE;

/*数据存储命令*/
#define SaveInt(Data)   unSave.siSave[ucSavePit] = (SINT32)Data; ucSavePit++;
#define SaveFp(Data)	unSave.fpSave[ucSavePit] = Data; ucSavePit++;

/*接收数据*/
#define Receive(Data1, Data2, Data3)    fpTransition = *(FP32*)(Comm3RxBuf + 2);\
							  			Data1 = fpTransition;\
										fpTransition = *(FP32*)(Comm3RxBuf + 6);\
										Data2 = fpTransition;\
										fpTransition = *(FP32*)(Comm3RxBuf + 10);\
										Data3 = fpTransition
/*发送数据*/
#define Send(Data1, Data2, Data3)       fpTransition = Data1;\
										*(FP32*)Comm3TxData = fpTransition;\
										fpTransition = Data2;\
										*(FP32*)(Comm3TxData + 4) = fpTransition;\
										fpTransition = Data3;\
										*(FP32*)(Comm3TxData + 8) = fpTransition

/*绘图*/
#define Send2(Data1, Data2)             fpTransition = Data1;\
										*(FP32*)Comm3TxData = fpTransition;\
										fpTransition = Data2;\
										*(FP32*)(Comm3TxData + 4) = fpTransition

#define Save()\
SaveFp(g_stUndePan1.stVeltPID.KP);\
SaveFp(g_stUndePan1.stVeltPID.KI);\
SaveFp(g_stUndePan1.stVeltPID.KD);\
SaveFp(g_stUndePan2.stVeltPID.KP);\
SaveFp(g_stUndePan2.stVeltPID.KI);\
SaveFp(g_stUndePan2.stVeltPID.KD);\
SaveFp(g_stUndePan3.stVeltPID.KP);\
SaveFp(g_stUndePan3.stVeltPID.KI);\
SaveFp(g_stUndePan3.stVeltPID.KD);\
SaveFp(g_stUndePan4.stVeltPID.KP);\
SaveFp(g_stUndePan4.stVeltPID.KI);\
SaveFp(g_stUndePan4.stVeltPID.KD)




void Debug(void);
void EnableAllMotor(void);
void DisableAllMotor(void);
void SendVeltToMotor(void);

#endif
