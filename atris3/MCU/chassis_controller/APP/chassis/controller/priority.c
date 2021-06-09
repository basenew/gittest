#include "priority.h"
#include "rthw.h"
#include "math.h"

#define LOG_TAG              "prio"
#define LOG_LVL              LOG_LVL_DBG
#include <ulog.h>

volatile uint32_t priority_register;
volatile uint32_t brake_register;
volatile COMPLEX_def priority_LV;
volatile COMPLEX_def priority_RV;
volatile int calibration_dec = DF_CALIBRATION_OPEN_TIME;

/*brake by BA enable/disable. 1: brake by BA, 0: brake by parallel speed 0 control*/
extern u8 var_BA_flag;
rt_bool_t getCalibrationSwitchStatus(void){
    if(calibration_dec > 0)
        return RT_TRUE;
    else return RT_FALSE;
    
}
rt_bool_t controlChassis(COMPLEX_def LV,COMPLEX_def RV,int prio){
    if((priority_register & (~prio)) > prio)
        return RT_FALSE;

   
    int level=rt_hw_interrupt_disable();
    
    priority_register = priority_register | prio;
    
    if((fabs(LV.real) > DF_MIN_RUN_SPEED) || (fabs(LV.imag) > DF_MIN_RUN_SPEED) || (fabs(RV.imag) > DF_MIN_RUN_ANGLE_SPEED)){
        brake_register =  DF_PROIRITY_BRAKE_FULL;
        priority_LV = LV;
        priority_RV = RV;
       // rt_kprintf("%d %d %d %d\r\n",(int)(LV.real * 100),(int)(LV.imag * 100),(int)(fabs(RV.imag) * 100),(int)(fabs(RV.real) * 100)); 
    }else{
        priority_LV.real = 0.0f;
        priority_LV.imag = 0.0f;
        priority_RV.real = 0.0f;
        priority_RV.imag = 0.0f;
       // rt_kprintf("else :%d %d %d\r\n",(int)(LV.real * 100),(int)(LV.imag * 100),(int)(RV.imag * 100)); 
    }
    rt_hw_interrupt_enable(level) ;   
  // rt_kprintf("priority:%.8x,prio:%.8x,brake_register:%d %d\r\n",priority_register,prio,brake_register,(int)(RV.imag * 100)); 
    return RT_TRUE;
}

rt_bool_t getChassisSpeed(COMPLEX_def *LV,COMPLEX_def *RV){

    int level=rt_hw_interrupt_disable();

    *LV = priority_LV;
    *RV = priority_RV;
    if(priority_register <= 0)
        {
            priority_LV.real = 0.0f;
            priority_LV.imag = 0.0f;
            priority_RV.real = 0.0f;
            priority_RV.imag = 0.0f;
           // rt_kprintf("else :%d %d %d\r\n",(int)(LV.real * 100),(int)(LV.imag * 100),(int)(RV.imag * 100)); 
        }
    
    rt_hw_interrupt_enable(level) ;   

    if(brake_register <= DF_PROIRITY_RELEASE){
        RV->real = DF_RELEASE_ANGLE_REAL;
    }else if(brake_register < DF_PROIRITY_BRAKE){
				if(var_BA_flag !=0)
					RV->real = DF_BRAKE_ANGLE_REAL;
				else
					RV->real = 0;

    }
    return RT_TRUE;
}
/*shawn : brake. set the horizontal */
rt_bool_t ChassisBreak(void){
    priority_LV.real = 0.0f;
    priority_LV.imag = 0.0f;
	if(var_BA_flag !=0)
    priority_RV.real = 1.0f;
	else
		priority_RV.real = 0.0f;
	
    priority_RV.imag = 0.0f;   
    brake_register =  DF_PROIRITY_BRAKE;
}
/*what is the meanning of this function ?

8 ge poriority , proirity turn on and off 



*/
static void priority_thread_entry(void *_param)
{
    int current_priority = 0;
	/*shawn1: what is the meanning of calibration open time*/
    calibration_dec = DF_CALIBRATION_OPEN_TIME;
    current_priority = DF_PROIRITY_MIN;
    brake_register = 0;
    priority_register = DF_PROIRITY_MIN;
   while(1){
       rt_thread_mdelay(62);  
       if(calibration_dec > 0)
            calibration_dec--;
       
       int level=rt_hw_interrupt_disable();
       if((priority_register & (~current_priority)) > current_priority){
           if(current_priority != DF_PROIRITY_MAX)
                current_priority = current_priority << DF_PROIRITY_BIT_WIDTH;
           rt_hw_interrupt_enable(level); 
           //LOG_I("priority:%8x",current_priority);
           continue;
       }
       //rt_kprintf("1priority:%.8x,brake:%d\r\n",priority_register,brake_register); 
       if(priority_register & current_priority){
            priority_register-=(DF_PROIRITY_DEC & current_priority);
       }else{
           if(current_priority != DF_PROIRITY_MIN)
                current_priority = current_priority >> DF_PROIRITY_BIT_WIDTH;
           //LOG_I("priority:%8x",current_priority);
       }
       //rt_kprintf("2priority:%.8x,brake:%d\r\n",priority_register,brake_register); 
       if(current_priority == 0)
           current_priority = DF_PROIRITY_MIN;
       if(brake_register > 0)
        brake_register--;  
      //rt_kprintf("3priority:%.8x,brake:%d\r\n",priority_register,brake_register); 
       rt_hw_interrupt_enable(level);  

       // controlChassis(LV,RV,MASTER_CONTROL_PRIO) ;      
       //rt_kprintf("priority:%.8x,brake:%d\r\n",priority_register,brake_register); 
        
   }
    
}



int32_t priority_init(void)
{
if(DF_THREAD_STATIC_MEMORY == 0){
    rt_thread_t thread = rt_thread_create("priority_thread", priority_thread_entry, RT_NULL, TASK_STACK_SIZE_PRIORITY, TASK_PRIORITY_PRIORITY, 10);

    if (thread != RT_NULL) {
        rt_thread_startup(thread);
    }
    else{
        return -1;
    }
}else{
    static struct rt_thread priority_thread;
    ALIGN(RT_ALIGN_SIZE)
    static char priority_thread_stack[TASK_STACK_SIZE_PRIORITY]; 
    	rt_err_t result = RT_EOK;
    result = rt_thread_init(&priority_thread,
                            "priority_thread",
                            priority_thread_entry, RT_NULL,
                            &priority_thread_stack[0], sizeof(priority_thread_stack),
                            TASK_PRIORITY_CONTROLLER, 10);

    if (result == RT_EOK)
    	rt_thread_startup(&priority_thread);
    else
    	LOG_I("odom thread create failed.");
    
}    
    
    return 0;
}
