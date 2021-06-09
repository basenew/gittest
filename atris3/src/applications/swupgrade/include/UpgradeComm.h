#ifndef UPGRADE_COMMON_H__
#define UPGRADE_COMMON_H__

typedef enum __up_stats
{
	UPGRADE_UNKNOWN,
    UPGRADE_START,
    UPGRADE_INPROGRESS,
    UPGRADE_FINISH,
    UPGRADE_FAIL
} up_stats;

typedef enum __product_index
{
	MONITOR,
	MOTOR
} product_index;

typedef struct __UP_RESULT
{
	int index;
	int upgrade_status;
	std::string product_name;
	int err_code;
	std::string err_string;
	int percentage;
	time_t time_stamp;
} UP_RESULT;

#endif