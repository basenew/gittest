/* =================================================================================
File name:       PI.H 
===================================================================================*/


#ifndef __PI_H__
#define __PI_H__

typedef struct {  float  Ref;   			// Input: reference set-point
				  float  Fbk;   			// Input: feedback
			      float MechTheta;       // Input: Electrical angle (pu)
			      float ControlTheta;       // Input: control angle (pu)
				  float  Out;   			// Output: controller output 
				  float  Kp;				// Parameter: proportional loop gain
				  float  Ki;			    // Parameter: integral gain
				  float  Umax;			// Parameter: upper saturation limit
				  float  Umin;			// Parameter: lower saturation limit
				  float  up;				// Data: proportional term
				  float  ui;				// Data: integral term
				  float  v1;				// Data: pre-saturated controller output
				  float  i1;				// Data: integrator storage: ui(k-1)
				  float  w1;				// Data: saturation record: [u(k-1) - v(k-1)]
	              uint32_t BaseFreq;      // Parameter: Base speed in rpm (Q0) - independently with global Q
				} PI_CONTROLLER;


/*-----------------------------------------------------------------------------
Default initalisation values for the PI_GRANDO objects
-----------------------------------------------------------------------------*/                     

#define PI_CONTROLLER_DEFAULTS {		\
						   0, 			\
                           0, 			\
						   0, 			\
                           0,            \
                           0,           \
                           1.0f,	\
                           0,	\
                           1.0f,	\
                           1.0f, 	\
                           0.0f,	\
                           0, 	\
                           0,	\
                           0,	\
                           0, 	\
                           0,   \
              			  }


/*------------------------------------------------------------------------------
 	PI_GRANDO Macro Definition
------------------------------------------------------------------------------*/
#define PI_MACRO(v)                                             \
                                                                \
    /* proportional term */                                     \
    v.up = v.Ref - v.Fbk;                                       \
                                                                \
    /* integral term */                                         \
    v.ui = (v.Out == v.v1)?((v.Ki * v.up)+ v.i1) : v.i1;   		\
    v.i1 = v.ui;                                                \
                                                                \
    /* control output */                                        \
    v.v1 = v.Kp *(v.up + v.ui);                         		\
	if(v.v1 > v.Umax){					  						\
		v.Out = v.Umax;											\
	}															\
	else if(v.v1 < v.Umin){										\
		v.Out = v.Umin;											\
	}															\
	else{														\
		v.Out = v.v1;											\
	}

	
#endif // __PI_H__

