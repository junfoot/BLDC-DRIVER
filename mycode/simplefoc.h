#ifndef __SIMPLEFOC_H__
#define __SIMPLEFOC_H__

#include "main.h"
#include "foc.h"

typedef struct sSimple_para{
	float simple_P;
	float simple_I;
	float simple_D;
	
	float simple_Uq;
	
	
} tSimple_para;


extern tSimple_para simple_para;
void set_simple_para();
void simple_algorithm(FOCStruct *foc, tSimple_para *simple_para, float theta);

#endif