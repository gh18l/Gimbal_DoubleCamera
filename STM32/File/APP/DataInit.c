#include "Flash.h" 
#include "DataInit.h"
#include "UnderPan_Task.h"  

void GlobalDataInit(void)
{
	UN_READ unRead;
	UCHAR8 ucReadPit = 0;//��ȡ��λ

	/*��ȡFlash����*/
	ReadFp(g_stUndePan1.stVeltPID.KP);
	ReadFp(g_stUndePan1.stVeltPID.KI);
	ReadFp(g_stUndePan1.stVeltPID.KD);
	ReadFp(g_stUndePan2.stVeltPID.KP);
	ReadFp(g_stUndePan2.stVeltPID.KI);
	ReadFp(g_stUndePan2.stVeltPID.KD);
	ReadFp(g_stUndePan3.stVeltPID.KP);
	ReadFp(g_stUndePan3.stVeltPID.KI);
	ReadFp(g_stUndePan3.stVeltPID.KD);
	ReadFp(g_stUndePan4.stVeltPID.KP);
	ReadFp(g_stUndePan4.stVeltPID.KI);
	ReadFp(g_stUndePan4.stVeltPID.KD);



}
