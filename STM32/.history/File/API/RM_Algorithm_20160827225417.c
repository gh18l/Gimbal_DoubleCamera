/*******************************************************************
版权声明： RoboMasters(HITCRT_iHiter 战队)
文件名： RM_Algorithm.h
最近修改日期： 2016/1/25
版本： 1.0
--------------------------------------------------------------------
模块描述：该模块定义了常用的算法。
--------------------------------------------------------------------
修改记录：
作者                              时间         版本   说明
彭季超、彭毓兴、张冰、李四林    2016/1/25     1.0     建立此文件
********************************************************************/
#include "main.h"
#include <math.h>
#include <float.h>

/**************************************************************************************************/
/*********************************数学运算相关算法****************************************************/
/**************************************************************************************************/


/*--------------------------------------------------------------------------------------------------
函数名称：Clip()
函数功能：削波函数，去除超出最大值与最小值之间的值，代之以最大或最小值
--------------------------------------------------------------------------------------------------*/
FP32 Clip(FP32 fpValue, SINT32 siMin, SINT32 siMax)
{
	if(fpValue < siMin)
	{
		return (FP32)(siMin);
	}
	else if(fpValue > siMax)
	{
		return (FP32)(siMax);
	}
	else 
	{
		return fpValue;
	}
}
/*--------------------------------------------------------------------------------------------------
函数名称：Round()
函数功能：将浮点数四舍五入，返回32位整型数
--------------------------------------------------------------------------------------------------*/
SINT32 Round(FP32 fpValue)
{   
    if (fpValue >= 0)
    {
    	return (SINT32)(fpValue + 0.5f);
    }
    else 
    {
    	return (SINT32)(fpValue - 0.5f);
    }
}

/*--------------------------------------------------------------------------------------------------
函数名称：IsNum()
函数功能：判断一个浮点型是否是数字
--------------------------------------------------------------------------------------------------*/
bool IsNum(DB64 x)
{
	if (x == x)
		return TRUE;
    else
		return FALSE;
}

/*--------------------------------------------------------------------------------------------------
函数名称：IsFiniteNum()
函数功能：判断一个浮点型是否是有限的即判断一个float既不是NAN也不是infinite
--------------------------------------------------------------------------------------------------*/
bool IsFiniteNum(DB64 x)
{
	if (x <= DBL_MAX && x >= -DBL_MAX)
		return TRUE;
	else
    	return FALSE;
}


/**************************************************************************************************/
/*********************************PID相关算法****************************************************/
/**************************************************************************************************/


/*--------------------------------------------------------------------------------------------------
函数名称：CalPID()  
函数功能：计算PID量，就是最最普通的PID
          增量型PID算法的计算公式：detU(k)=U(k)-U(k-1)=Kp(detE(k)+IE(k)+fpD*det(E(k))+det(E(k)))
          无遇限削弱式PID:U=U+KP*△E+KI*E+KD*[E+E(n-2)-2*E(n-1)]
--------------------------------------------------------------------------------------------------*/
void CalPID(volatile ST_PID *pstPid)    
{   
    UCHAR8 ucK;
   
	ucK=1;
	pstPid->U += pstPid->KP * (pstPid->E - pstPid->PreE) + ucK * pstPid->KI * pstPid->E + pstPid->KD * (pstPid->E - 2 * pstPid->PreE + pstPid->PrePreE);
	pstPid->PrePreE = pstPid->PreE;
	pstPid->PreE = pstPid->E;
    
    //判断是否是数字，如果不是数字，赋值为零,修复了遥控器先开启车不稳定的bug
    if (IsFiniteNum(pstPid->U) != TRUE)
        pstPid->U = 0;

    pstPid->U = Clip(pstPid->U,pstPid->UMin,pstPid->UMax);
}

void CalPID_Su(volatile ST_PID *pstPid)    
{   
    UCHAR8 ucK;
   
	ucK=1;
	pstPid->U = pstPid->KP * pstPid->E + ucK * pstPid->KI * (pstPid->E - pstPid->PreE);
	pstPid->PrePreE = pstPid->PreE;
	pstPid->PreE = pstPid->E;

	if (IsFiniteNum(pstPid->U) != TRUE)
    pstPid->U = 0;

    pstPid->U = Clip(pstPid->U,pstPid->UMin,pstPid->UMax);
}
/*--------------------------------------------------------------------------------------------------
函数名称：CalPID_IS() :Calculate PID Integral Separation
函数功能：积分分离式PID
           增量型PID算法的计算公式：detU(k)=U(k)-U(k-1)=Kp(detE(k)+IE(k)+D*det(E(k))+det(E(k)))
           积分分离式PID:U=U+KP*△E+ucK*KI*E+KD*[E+E(n-2)-2*E(n-1)]
           积分分离式PID，可防止过大超调量
--------------------------------------------------------------------------------------------------*/
void CalPID_IS(volatile ST_PID *pstPid)	
{   
    UCHAR8  ucK;
    
	if (fabs(pstPid->E) > pstPid->ELimit)
	{
		ucK=0; 
	}
	else 
	{
		ucK=1;
	}
	pstPid->U += pstPid->KP * (pstPid->E - pstPid->PreE) 
                 + ucK * pstPid->KI * pstPid->E 
                 + pstPid->KD * (pstPid->E - 2 * pstPid->PreE + pstPid->PrePreE);

	pstPid->PrePreE = pstPid->PreE;
	pstPid->PreE=pstPid->E;
}

/*--------------------------------------------------------------------------------------------------
函数名称：CalPID_WTCOL()  :Weaken The Case Of Limited
函数功能：计算PID量，遇限削弱式PID
          增量型PID算法的计算公式：detU(k)=U(k)-U(k-1)=Kp(detE(k)+IE(k)+fpD*det(E(k))+det(E(k)))
          遇限削弱式PID:U=U+KP*△E+ucK*KI*E+KD*[E+E(n-2)-2*E(n-1)]
          遇限削弱式PID，可防止PID运算结果长期饱和,带死区
--------------------------------------------------------------------------------------------------*/
void CalPID_WTCOL(volatile ST_PID *pstPid)  
{   
    UCHAR8 ucK;
	 
	if (((pstPid->U > pstPid->ULimit) && (pstPid->E > 0)) || ((pstPid->U < -pstPid->ULimit) && (pstPid->E < 0)))//若饱和，在能减小结果绝对值的前提下才进行积分
	{
	    ucK = 0;
	}
	else
	{ 
		ucK = 1;
	}

	if(abs(pstPid->E) <= pstPid->EDead)
	{
		pstPid->E = 0;
	}
	pstPid->U += pstPid->KP * (pstPid->E - pstPid->PreE) 
                 + ucK * pstPid->KI * pstPid->E 
                 + pstPid->KD * (pstPid->E - 2 * pstPid->PreE + pstPid->PrePreE);
	
	pstPid->PrePreE = pstPid->PreE;
	pstPid->PreE = pstPid->E;
}

/*--------------------------------------------------------------------------------------------------
函数名称：CalPID_WTCOLIS() 
函数功能：计算PID量
          增量型PID算法的计算公式：detU(k)=U(k)-U(k-1)=Kp(detE(k)+IE(k)+fpD*det(E(k))+det(E(k)))
          积分分离+遇限削弱式PID:U=U+KP*△E+ucK1*ucK2*KI*E+KD*[E+E(n-2)-2*E(n-1)]
          积分分离+遇限削弱式PID结合二者优点
--------------------------------------------------------------------------------------------------*/
void CalPID_WTCOLIS(volatile ST_PID *pstPid)  
{   
    UCHAR8 ucK1, ucK2;
    
    //遇限削弱
	if (((pstPid->U > pstPid->ULimit) && (pstPid->E > 0)) 
		|| ((pstPid->U < -pstPid->ULimit) && (pstPid->E < 0)))//根据上一次的pstPid运算结果来判断是否要有积分环节
	{
	    ucK1=0;
	}
	else
	{ 
		ucK1=1;
	}
	
	//积分分离
	if (fabs(pstPid->E) > pstPid->ELimit)
	{
		ucK2=0;
	}
	else
	{ 
		ucK2=1;
	}
	
	pstPid->U += pstPid->KP * (pstPid->E - pstPid->PreE) 
                 + ucK1 * ucK2 * pstPid->KI * pstPid->E 
                 + pstPid->KD * (pstPid->E - 2 * pstPid->PreE + pstPid->PrePreE);
	
	pstPid->PrePreE = pstPid->PreE;
	pstPid->PreE = pstPid->E;
}

