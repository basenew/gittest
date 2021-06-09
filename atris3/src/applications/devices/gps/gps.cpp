#include "gps.h"
#include <boost/thread/thread.hpp>
#include <thread>
#include <chrono>
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <unistd.h>

#define GPS_UART_PORT "/dev/ttyS0"
#define MAX_GPS_DATA_LEN 512

Gps::Gps() 
    :uart_fd_(-1)
    ,last_gps_valid_flag_(-1)
    ,speed_direction_(0)
    ,longitude_(0)
    ,latitude_(0)
    ,latitude_NS_(0)
    ,longitude_EW_(0)
    ,lat_deg_(0)
    ,lat_min_(0)
    ,lat_sec_(0)
    ,lon_deg_(0)
    ,lon_min_(0)
    ,lon_sec_(0)
{
	int iRet;
    log_info("********%s********", __FUNCTION__);
    log_info("********%s********", __FUNCTION__);
    log_info("********%s********", __FUNCTION__);

    diag_info_pub_ = nh_.advertise<atris_msgs::RobotInfo>(TOPIC_DIAGNOSTIC_ROBOT_INFO, 100);
    
    iRet = init();
    if(iRet < 0)
    {
        log_error("%s gps init failed!!! iRet = %d",__FUNCTION__, iRet);
    }
    else
    {
    	log_info("%s gps init success...",__FUNCTION__);
    }

    new boost::thread(boost::bind(&Gps::gps_data_proc, this));
}

// gps initialization
int Gps::init(void)
{
	int iRet;
    log_info("%s gps initialization",__FUNCTION__);
    iRet = open_port(GPS_UART_PORT);
    if(iRet < 0)
    {
        log_error("%s open gps uart port failed, iRet = %d", __FUNCTION__, iRet);
        return -1;
    }
    else
    {
        uart_fd_ = iRet;
        log_info("%s uard_fd : %d",__FUNCTION__, uart_fd_);
    }

    return 0;
}

// open gps uart port to recv gps data from it
// initialize uart device
int Gps::open_port(const char *port_device)
{
    struct termios options;

    //int fd = open(port_device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    int fd = open(port_device, O_RDWR | O_NOCTTY | O_NONBLOCK);

    tcgetattr(fd, &options);

    if (fd == -1)
    {
        log_debug("open_port: Unable to open SerialPort");
        return (-1);
    }

    if (fcntl(fd, F_SETFL, 0) < 0)
    {
        log_debug("fcntl failed\n");
    }
    else
    {
        fcntl(fd, F_SETFL, 0);
    }

    if (isatty(STDIN_FILENO) == 0)
    {
        log_debug("standard input is not a terminal device\n");
    }
    else
    {
        log_debug("isatty success!\n");
    }

    bzero(&options, sizeof(options));

    options.c_cflag = B57600 | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    options.c_cc[VTIME] = 0;
    options.c_cc[VMIN] = 0;
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &options);
    return (fd);
}


// send gps control command
// actually , here we do not need to send any command
int Gps::send_gps_ctrl_cmd(char * cmd)
{
	int ret_len;
	int ret;
	int len;

	if(uart_fd_ < 0)
	{
		log_error("%s gps uart cmd send not ok (uart fd not valid) , uart_fd_ : %d",__FUNCTION__, uart_fd_);
		return -1;
	}

	char strCmd[256] = {0};
	memset(strCmd, 0x00, sizeof(strCmd));
	len = strlen(cmd);
	log_info("%s cmd len : %d",__FUNCTION__, len);

	// copy cmd to command buf
    strncpy(strCmd, cmd, len);
    // add \r\n at the end if not ended with \r\n
    if(strCmd[len-1]!='\n' && strCmd[len-2]!='\r')
    {
        ret_len = sprintf(strCmd, "%s\r\n", cmd);
        strCmd[ret_len] = '\0';
        log_info("%s ret len : %d", __FUNCTION__, ret_len);
    }

    // send the command through uart
    ret = write(uart_fd_, strCmd, ret_len);
    log_info("%s write %d bytes to uart",__FUNCTION__, ret);
    return 0;
}

// check if gps frame valid
int Gps::isGpsFrameValid(char * frame)
{
	int i;
	int len = strlen(frame);
	//log_info("%s frame len = %d",__FUNCTION__,len);
	//log_info("%s frame content : %s",__FUNCTION__, frame);
	uint8_t check_sum, check_sum_calc;

	if(len < 4)
	{
        log_info("%s frame to short to deal with!!! len = %d",__FUNCTION__, len);
        return -1;
	}

	if(frame[0] != '$' || (frame[1] != 'G') || (frame[2] != 'P' && frame[2]!='N'))
	{
        //log_error("%s gps frame invalid!!!", __FUNCTION__);
        return -2;
	}

    check_sum = AsciiToHex((char *)&frame[len-2]);
    check_sum_calc = 0;

    for(i = 1;i<len -3;i++)
    {
        check_sum_calc ^= frame[i];
    }

    //log_info("%s check sum = %02x , calculated check sum = %02x", __FUNCTION__, check_sum, check_sum_calc);

    return check_sum == check_sum_calc;
}

// convert to ascii byte to a hex
uint8_t Gps::AsciiToHex(char * asc)
{
    uint8_t hex = 0;
    uint8_t tmp;
    int i;

    for(i = 0;i <2;i++)
    {
    	if(asc[i] >= '0' && asc[i] <= '9')
    	{
            tmp = asc[i] - '0';
    	}
    	else if(asc[i] >= 'a' && asc[i] <= 'f')
    	{
            tmp = asc[i] - 'a' + 0x0a;
    	}
    	else if(asc[i] >= 'A' && asc[i] <= 'F')
    	{
            tmp = asc[i] - 'A' + 0x0a;
    	}
    	else
    	{
            tmp = 0;
    	}

    	if(i == 0)
    	{
            hex = tmp<<4;
    	}
    	else
    	{
            hex |= tmp;
    	}
    }

    return hex;
}

// get the next comma field in the string
char * Gps::get_next_comma_field(char * data_ptr)
{
    char * ptr = NULL;

    if(!data_ptr)
    {
    	log_error("%s data poitner cannot be NULL",__FUNCTION__);
    	return NULL;
    }

    if((ptr = strchr(data_ptr,',')) == NULL)
    {
    	log_error("%s cannot find comma in the remained field",__FUNCTION__);
        return NULL;
    }

    ptr++; // we move the ptr forward to point to the next byte
    return (*ptr != '\0' ? ptr : NULL);
}

void Gps::printGpsContent(char * p_buf, int len)
{
    for(int i = 0; i < len;i++)
    {
    	log_info("%s buf[%d] : %c",__FUNCTION__, i, p_buf[i]);
    }

    log_info("%s\r\n",__FUNCTION__);
}


void Gps::gps_data_proc(void)
{
    log_info("********** %s *************\r\n",__FUNCTION__);
    char gps_data_buffer[MAX_GPS_DATA_LEN] = {0};
    static char frame_buffer[MAX_GPS_DATA_LEN];
    static uint16_t index = 0;
    static uint8_t cs_len = 0;
    static enum parse_status prstats = PS_HEAD; // initialize to parse data string head

    log_info("%s uart fd = %d", __FUNCTION__, uart_fd_);

    ssize_t len;
    int i;
    while(1)
    {
        if((len = read(uart_fd_, gps_data_buffer, MAX_GPS_DATA_LEN))<=0)
        {
        	//log_error("%s read gps data from uart port failed, len = %d, continue the next loop",__FUNCTION__, len);
        	continue;
        }
        // here we print the gps recv buffer for debug use
        //log_info("%s gps data : %s len : %d",__FUNCTION__, gps_data_buffer, len);
        //printGpsContent(gps_data_buffer, len);
        // we are ready to parse all the data in the buffer
        for(i = 0; i <len;i++)
        {
        	if(gps_data_buffer[i] == '$') // we initialize everything since we found the head of the data buffer
        	{
        		//log_info("%s receive frame head",__FUNCTION__);
                index = 0;
                frame_buffer[index++] = gps_data_buffer[i];
                prstats = PS_DATA;
                continue;
        	}

        	switch(prstats)
        	{
        		case PS_DATA:
        		    //log_info("%s receive frame data",__FUNCTION__);
        		    if((index + 1) > MAX_GPS_DATA_LEN) // if message is too big
        		    {
                        prstats = PS_HEAD;
        		    }
        		    else
        		    {
                        frame_buffer[index++] = gps_data_buffer[i];
                        if(gps_data_buffer[i] == '*')
                        {
                        	//log_info("%s receive frame tail",__FUNCTION__);
                            prstats = PS_CS;
                            cs_len = 0;
                        }
        		    }

        		    break;

        		case PS_CS:
        		    frame_buffer[index++] = gps_data_buffer[i];
        		    if(++cs_len==2)
        		    {
        		    	// here we receive a whole frame
        		    	//log_info("%s receive a whole frame",__FUNCTION__);
        		    	frame_buffer[index] = '\0';
        		    	gps_handle_frame(frame_buffer);
        		    	index = 0;
        		    	prstats = PS_HEAD;
        		    }

        		    break;

        		default:
        		    prstats = PS_HEAD;
        		    break;
        	}
        }
    }
}

// gps handle frame
void Gps::gps_handle_frame(char * frame)
{
	// first we check if the whole gps frame is valid
    if(!isGpsFrameValid(frame))
    {
        log_info("%s gps frame not valid",__FUNCTION__);
        return;
    }

    if(frame[3] == 'G' && frame[4] == 'G' && frame[5] == 'A')
    {
        //log_info("%s here we dont parse gga frame",__FUNCTION__);
    }
    else if(frame[3] == 'R' && frame[4] == 'M' && frame[5] == 'C')
    {
        //log_info("%s recv a gps rmc frame",__FUNCTION__);
        parse_gps_rmc_frame(frame);
    }
    else if(frame[3] == 'G' && frame[4] == 'S' && frame[5] == 'V')
    {
        //log_info("%s we dont parse the gsv frame",__FUNCTION__);
    }
}

// parse a gps rmc frame
// if the network is bad we can use gps time to correct the system time
void Gps::parse_gps_rmc_frame(char * frame)
{
    int valid_flag;
    char tmp_buf[32] = {0};
    char * ptr = frame;
    char * ptr1;
    int len;
    int hour,min,sec;

    if((ptr = get_next_comma_field(ptr))==NULL)
    {
    	log_error("%s 11111111, cannot get the next comma field",__FUNCTION__);
    	return;
    }

    //log_info("%s frame : %s", __FUNCTION__, frame);

    // here the pointer is point to date structure
    char time_flag = 0;
    if(*ptr != ',')
    {
        hour = num_asc_to_int((char *)&ptr[0],2);
        min = num_asc_to_int((char *)&ptr[2],2);
        sec = num_asc_to_int((char *)&ptr[4],2);
        //log_info("%s time struct valid : hour  : %d , min : %d , sec : %d",__FUNCTION__, hour, min, sec);
        time_flag = 1;
    }
    
    // try to get the next comma field
    if((ptr = get_next_comma_field(ptr))==NULL)
    {
    	log_error("%s 22222222, cannot get the next comma field",__FUNCTION__);
    	return;
    }

    if(*ptr == 'A')
    {
    	//log_info("%s gps position ok", __FUNCTION__);
    	valid_flag = 1;
    }
    else
    {
    	//log_info("%s gps position not ok", __FUNCTION__);
    	valid_flag = 0;
    }

    // parse latitude and longitude
    if((ptr = get_next_comma_field(ptr))==NULL)
    {
    	return;
    }

    if((ptr1 = get_next_comma_field(ptr))==NULL)
    {
    	return;
    }

    len = ptr1 - ptr -1;
    memcpy(tmp_buf, ptr, len);
    tmp_buf[len] = '\0';

    //latitude_ = (num_asc_to_int(&tmp_buf[0],2))*1000000;
    //latitude_ += get_gps_longitude_and_latitude(&tmp_buf[2]);
    parseLat(tmp_buf);
    latitude_ = parseLatDeg(tmp_buf);
    //log_info("%s gps latitude in deg: %d",__FUNCTION__, latitude_);

    if((ptr = get_next_comma_field(ptr))==NULL)
    {
    	return;
    }

    if(*ptr == 'N')
    {
        latitude_NS_ = 1;
    }
    else
    {
        latitude_NS_ = 0;
    }

    if((ptr = get_next_comma_field(ptr))==NULL)
    {
    	return;
    }

    if((ptr1 = get_next_comma_field(ptr))==NULL)
    {
    	return;
    }

    len = ptr1 - ptr -1;
    memcpy(tmp_buf, ptr, len);
    tmp_buf[len] = '\0';

    //longitude_ = (num_asc_to_int(&tmp_buf[0],3))*1000000;
    //longitude_ += get_gps_longitude_and_latitude(&tmp_buf[3]);

    parseLon(tmp_buf);
    longitude_ = parseLonDeg(tmp_buf);
    //log_info("%s gps longitude in deg: %d",__FUNCTION__, longitude_);

    if(*ptr == 'E')
    {
        longitude_EW_ = 1;
    }
    else
    {
        longitude_EW_ = 0;
    }

    if((ptr = get_next_comma_field(ptr))==NULL)
    {
    	return;
    }

    if((ptr1 = get_next_comma_field(ptr))==NULL)
    {
    	return;
    }

    len = ptr1 - ptr -1;
    memcpy(tmp_buf, ptr, len);
    tmp_buf[len] = '\0';
    //get_gps_speed; // ignore gps speed since it is not very accurate in slow motion

    if((ptr = get_next_comma_field(ptr))==NULL)
    {
    	return;
    }

    if((ptr1 = get_next_comma_field(ptr))==NULL)
    {
    	return;
    }

    len = ptr1 - ptr -1;
    memcpy(tmp_buf, ptr, len);
    tmp_buf[len] = '\0';

    speed_direction_ = get_gps_speed_direction(tmp_buf);
    //log_info("%s speed direction : %d",__FUNCTION__, speed_direction_);

    // if gps frame not valid , set longitude, latitude and speed direction to 0
    if(!valid_flag)
    {
    	longitude_ = 0;
    	latitude_ = 0;
    	speed_direction_ = 0;
    }

    sendGpsData(valid_flag);

    last_gps_valid_flag_ = valid_flag;

}

// publish gps info out to diag
void Gps::sendGpsData(int position_status)
{
    Json::Value root;
    atris_msgs::RobotInfo rbtInfo_gps_info;  
    Json::FastWriter fw;
    root["robot_info"]["gps"]["error"] = position_status?0:1;
    root["robot_info"]["gps"]["lati"] = std::to_string(latitude_);
    root["robot_info"]["gps"]["long"] = std::to_string(longitude_);

    rbtInfo_gps_info.json = fw.write(root);
    diag_info_pub_.publish(rbtInfo_gps_info);

}

unsigned int Gps::parseLatDeg(char * buf)
{
	double fw;
	fw = atof(buf);

	fw /=100;

    unsigned long dw = 1000000*(unsigned long)fw;

    fw=fw-(unsigned long)fw;

    fw=fw*100/60;

    fw*=1000000;

    return (dw+(unsigned int)fw);
}

int Gps::parseLat(char * buf)
{
	float lat_tmp;
	lat_tmp = atof(buf);
	//log_info("%s lat tmp %f",__FUNCTION__, lat_tmp);

	float lat_tmp1;
	lat_tmp1 = lat_tmp/100;

	lat_deg_ = (int)lat_tmp1;
	//log_info("%s lat deg = %d",__FUNCTION__, lat_deg_);

	float lat_tmp2;
	lat_tmp2 = (lat_tmp1-lat_deg_)*60;

	lat_min_ = (int)lat_tmp2;
	//log_info("%s lat min = %d",__FUNCTION__, lat_min_);

	float lat_tmp3;
	lat_tmp3 = (lat_tmp2-lat_min_)*60;

	lat_sec_ = (int)lat_tmp3;
	//log_info("%s lat sec = %d",__FUNCTION__, lat_sec_);
    return 0;
}

int Gps::parseLon(char * buf)
{
	float lon_tmp;
	lon_tmp = atof(buf);
	//log_info("%s lon tmp %f",__FUNCTION__, lon_tmp);

	float lon_tmp1;
	lon_tmp1 = lon_tmp/100;
	//log_info("%s lon : %f",__FUNCTION__,lon_tmp1);

	lon_deg_ = (int)lon_tmp1;
	//log_info("%s lon deg = %d",__FUNCTION__, lon_deg_);

	float lon_tmp2;
	lon_tmp2 = (lon_tmp1-lon_deg_)*60;

	lon_min_ = (int)lon_tmp2;
	//log_info("%s lon min = %d",__FUNCTION__, lon_min_);

	float lon_tmp3;
	lon_tmp3 = (lon_tmp2-lon_min_)*60;

	lon_sec_ = (int)lon_tmp3;
	//log_info("%s lon sec = %d",__FUNCTION__, lon_sec_);
	return 0;
}

unsigned int Gps::parseLonDeg(char * buf)
{
	double fw;
	fw = atof(buf);

	fw /=100;

    unsigned long dw = 1000000*(unsigned long)fw;

    fw=fw-(unsigned long)fw;

    fw=fw*100/60;

    fw*=1000000;

    return (dw+(unsigned int)fw);
}

// transform a asc number to integer
int Gps::num_asc_to_int(char * asc, int len)
{
    int num = 0;
    int i;

    for(i = 0; i<len;i++)
    {
        num = num *10 + (*asc - '0');
    }

    //log_info("%s num = %d\r\n",__FUNCTION__, num);

    return num;
}

int Gps::get_gps_longitude_and_latitude(char * gps_data)
{
    int lon_lat = 0;
    int find_decimal = 0;
    int power_of_ten = 1;

    while(*gps_data)
    {
    	//log_info("%c",*gps_data);
    	if(find_decimal == 0)
    	{
    		if(*gps_data == '.')
    		{
    			//log_info("found .");
    			find_decimal = 1;
                gps_data++;
                continue;
    		}
    	}
    	else
    	{
    		power_of_ten *= 10;
    	}

        lon_lat = lon_lat*10 + (*gps_data - '0');
    	gps_data++;
    }

    log_info("%s lon lat : %d power of 10 : %d",__FUNCTION__, lon_lat, power_of_ten);

    lon_lat = lon_lat/60*(1000000/power_of_ten);
    log_info("%s lon lat : %d",__FUNCTION__, lon_lat);
    return lon_lat;
}

unsigned short Gps::get_gps_speed_direction(char * gps_direction)
{
	unsigned short speed_direction = 0;
	while(*gps_direction)
	{
		if(*gps_direction=='.')
		{
			break;
		}

		speed_direction = speed_direction*10+(*gps_direction-'0');
		gps_direction++;
	}

	return speed_direction;
}