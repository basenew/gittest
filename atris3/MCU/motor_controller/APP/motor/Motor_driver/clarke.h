/* =================================================================================
File name:       CLARKE.H  
===================================================================================*/


#ifndef __CLARKE_H__
#define __CLARKE_H__

typedef struct {  
				float  As;  		// Input: phase-a stator variable
				float  Bs;			// Input: phase-b stator variable
				float  Cs;			// Input: phase-c stator variable  
				float  Alpha;		// Output: stationary d-axis stator variable 
				float  Beta;		// Output: stationary q-axis stator variable
				float  Temp1;
				float  Temp2;
		 	 	} CLARKE;

/*-----------------------------------------------------------------------------
	Default initalizer for the CLARKE object.
-----------------------------------------------------------------------------*/                     
#define CLARKE_DEFAULTS { 0, \
                          0, \
                          0, \
                          0, \
                          0, \
              			} 
#define CLARKE_MACRO(v)											\
																\
v.Alpha = v.As;													\
v.Beta = (v.As + 2*v.Bs) *0.57735f;						\
										

#endif // __CLARKE_H__

