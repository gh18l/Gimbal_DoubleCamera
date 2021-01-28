#ifndef __RM_ALGORITHM_H__
#define __RM_ALGORITHM_H__

SINT32 Clip(SINT32 siValue, SINT32 siMin, SINT32 siMax);
SINT32 Round(FP32 fpValue);
void CalPID(volatile ST_PID *pstPid);
void CalPID_IS(volatile ST_PID *pstPid);
void CalPID_WTCOL(volatile ST_PID *pstPid);
void CalPID_WTCOLIS(volatile ST_PID *pstPid);
#endif 
