
#ifndef __COMPLEX_H_
#define __COMPLEX_H_
#ifdef __cplusplus
 extern "C" {
#endif
#define DF_RAD_SUB_DIVIDE     100
#define DF_SIN_COS_SUB_DIVIDE     180
#define DF_SQRT_DIVIDE     10
     
     
//     
//                     
//                                    ^ real(x)
//                                    |
//                                    |
//                                    |
//                                    |
//                                    |
//                                    |
//                                    |
//                                    |
//                 <------------------0
//                imag(y)
     
     
     
#define PI					3.14159265358979f
#define ANGLE_PI					180.0f
typedef struct
{
  double real;
  double imag; 
}COMPLEX_def;     



COMPLEX_def complexAdd(COMPLEX_def a,COMPLEX_def b);

COMPLEX_def complexSub(COMPLEX_def a,COMPLEX_def b);

COMPLEX_def complexMul(COMPLEX_def a,COMPLEX_def b);

COMPLEX_def complexDiv(COMPLEX_def a,COMPLEX_def b);

double getComplexMod(COMPLEX_def a);
double getComplexrad(COMPLEX_def a);
COMPLEX_def radModToComplex(double rad,double mod);


#ifdef __cplusplus
}
#endif



#endif
