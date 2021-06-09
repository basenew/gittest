int64 timestamp
int8 chassis_abnor # 0:正常;1:警告(可以使用,性能变弱);2:异常(立即停机)
char[16] axle  # 轴系(四轮四转底盘有四个轴,每轴有一个转向电机,一个行进电机);左前:leftFront,右前:rightFront,左后:leftRear,右后:rightRear
int8 steer_error_code #转向电机异常码
int8 direct_error_code #直行电机异常码
#  异常码   		异常说明
#	1				编码器ABZ报警
#	2				编码器UVW报警
#	3				位置超差
#	4				失速
#	5				ADC零点异常
#	6				过载
#	7				功率电源欠压
#	8				功率电源过压
#	9				过流
#	A				瞬时放电报警
#	B				平均放电报警
#	C				参数读写异常
#	D				输入端口重复定义
#	E				断线保护
#	F				温度报警

float32 steer_485_angle #转向电机角度,单位:弧度
float32 steer_angle #转向电机角度,单位:弧度
float32 direct_speed #直行电机速度,单位:m/s

int8 steer_temperature #转向电机温度,单位:度
int8 direct_temperature #直行电机温度,单位:度

int16 steer_torque #转向电机力矩,单位:满量程的千分之
int16 direct_torque #直行电机力矩,单位:满量程的千分之
