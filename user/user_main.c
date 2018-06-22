/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2016 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS ESP8266 only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include "ets_sys.h"
#include "osapi.h"
#include "ip_addr.h"
#include "espconn.h"
#include "mem.h"

#include "user_interface.h"
#include "smartconfig.h"
#include "airkiss.h"

#include "driver/uart.h"  //串口0需要的头文件
#include "gpio.h" 



#define DEVICE_TYPE 		"gh_9e2cff3dfa51" //wechat public number
#define DEVICE_ID 			"122475" //model ID

#define DEFAULT_LAN_PORT 	12476

#define STATION_IF 0x00
#define SOFTAP_IF 0x01

LOCAL esp_udp ssdp_udp;
LOCAL struct espconn pssdpudpconn;
LOCAL os_timer_t ssdp_time_serv;
struct espconn esp_conn;

ETSTimer connect_timer;
ETSTimer btntimer;
static uint8_t wifiStatus = STATION_IDLE, lastWifiStatus = STATION_IDLE;

uint8_t  lan_buf[200];
uint16_t lan_buf_len;
uint8 	 udp_sent_cnt = 0;

const airkiss_config_t akconf =
{
	(airkiss_memset_fn)&memset,
	(airkiss_memcpy_fn)&memcpy,
	(airkiss_memcmp_fn)&memcmp,
	0,
};

LOCAL void ICACHE_FLASH_ATTR
airkiss_wifilan_time_callback(void)
{
	uint16 i;
	airkiss_lan_ret_t ret;
	
	if ((udp_sent_cnt++) >30) {
		udp_sent_cnt = 0;
		os_timer_disarm(&ssdp_time_serv);//s
		//return;
	}

	ssdp_udp.remote_port = DEFAULT_LAN_PORT;
	ssdp_udp.remote_ip[0] = 255;
	ssdp_udp.remote_ip[1] = 255;
	ssdp_udp.remote_ip[2] = 255;
	ssdp_udp.remote_ip[3] = 255;
	lan_buf_len = sizeof(lan_buf);
	ret = airkiss_lan_pack(AIRKISS_LAN_SSDP_NOTIFY_CMD,
		DEVICE_TYPE, DEVICE_ID, 0, 0, lan_buf, &lan_buf_len, &akconf);
	if (ret != AIRKISS_LAN_PAKE_READY) {
		os_printf("Pack lan packet error!");
		return;
	}
	
	ret = espconn_sendto(&pssdpudpconn, lan_buf, lan_buf_len);
	if (ret != 0) {
		os_printf("UDP send error!");
	}
	os_printf("Finish send notify!\n");
}

LOCAL void ICACHE_FLASH_ATTR
airkiss_wifilan_recv_callbk(void *arg, char *pdata, unsigned short len)
{
	uint16 i;
	remot_info* pcon_info = NULL;
		
	airkiss_lan_ret_t ret = airkiss_lan_recv(pdata, len, &akconf);
	airkiss_lan_ret_t packret;
	
	switch (ret){
	case AIRKISS_LAN_SSDP_REQ:
		espconn_get_connection_info(&pssdpudpconn, &pcon_info, 0);
		os_printf("remote ip: %d.%d.%d.%d \r\n",pcon_info->remote_ip[0],pcon_info->remote_ip[1],
			                                    pcon_info->remote_ip[2],pcon_info->remote_ip[3]);
		os_printf("remote port: %d \r\n",pcon_info->remote_port);
      
        pssdpudpconn.proto.udp->remote_port = pcon_info->remote_port;
		os_memcpy(pssdpudpconn.proto.udp->remote_ip,pcon_info->remote_ip,4);
		ssdp_udp.remote_port = DEFAULT_LAN_PORT;
		
		lan_buf_len = sizeof(lan_buf);
		packret = airkiss_lan_pack(AIRKISS_LAN_SSDP_RESP_CMD,
			DEVICE_TYPE, DEVICE_ID, 0, 0, lan_buf, &lan_buf_len, &akconf);
		
		if (packret != AIRKISS_LAN_PAKE_READY) {
			os_printf("Pack lan packet error!");
			return;
		}

		os_printf("\r\n\r\n");
		for (i=0; i<lan_buf_len; i++)
			os_printf("%c",lan_buf[i]);
		os_printf("\r\n\r\n");
		
		packret = espconn_sendto(&pssdpudpconn, lan_buf, lan_buf_len);
		if (packret != 0) {
			os_printf("LAN UDP Send err!");
		}
		
		break;
	default:
		os_printf("Pack is not ssdq req!%d\r\n",ret);
		break;
	}
}

void ICACHE_FLASH_ATTR
airkiss_start_discover(void)
{
	ssdp_udp.local_port = DEFAULT_LAN_PORT;
	pssdpudpconn.type = ESPCONN_UDP;
	pssdpudpconn.proto.udp = &(ssdp_udp);
	espconn_regist_recvcb(&pssdpudpconn, airkiss_wifilan_recv_callbk);
	espconn_create(&pssdpudpconn);

	os_timer_disarm(&ssdp_time_serv);
	os_timer_setfn(&ssdp_time_serv, (os_timer_func_t *)airkiss_wifilan_time_callback, NULL);
	os_timer_arm(&ssdp_time_serv, 1000, 1);//1s
}

void ICACHE_FLASH_ATTR server_recv(void *arg,
	char *pdata,
	unsigned short len){//接收数据的回调函数
	os_printf("Received Data:%s\r\n",pdata);//将客户端发过来的数据打印出来
	espconn_sent((struct espconn *)arg,"Data received\r\n",strlen("Data received\r\n"));//往客户机发送数据
}
void ICACHE_FLASH_ATTR server_sent(void *arg){//发送数据成功的回调函数
	os_printf("send success\r\n");
}
void ICACHE_FLASH_ATTR server_discon(void *arg){//断开连接的回调函数
	os_printf("disconnected\r\n");
}

void ICACHE_FLASH_ATTR server_listen(void *arg){//服务器监听函数
	struct espconn *pespconn=arg;

	espconn_regist_recvcb(pespconn,server_recv);//注册一个接收数据的回调函数
	espconn_regist_sentcb(pespconn,server_sent);//注册一个发送数据成功的回调函数
	espconn_regist_disconcb(pespconn,server_discon);//注册一个断开连接的回调函数
}

void ICACHE_FLASH_ATTR server_recon(void *arg,sint8 err){//重新连接回调函数
	os_printf("connect error:%s\r\n",err);//输出重新连接的错误代码
}

void ICACHE_FLASH_ATTR my_server_init(struct ip_addr *local_ip,int port){
	LOCAL struct espconn esp_conn;
	esp_conn.type=ESPCONN_TCP;
	esp_conn.state=ESPCONN_NONE;
	esp_conn.proto.tcp=(esp_tcp *)os_malloc(sizeof(esp_tcp));

	os_memcpy(esp_conn.proto.tcp->local_ip,local_ip,4);
	esp_conn.proto.tcp->local_port=port;


	//注册连接成功的回调函数和连接失败重新连接的回调函数
	espconn_regist_connectcb(&esp_conn,server_listen);//注册一个连接成功回调函数
	espconn_regist_reconcb(&esp_conn,server_recon);//注册一个连接失败重新连接回调函数


	espconn_accept(&esp_conn);
}

void ICACHE_FLASH_ATTR wifi_conned()
{
	uint8 status = 0;
    static uint8 count = 0;
	struct ip_info info;
    count++;
	os_timer_disarm(&connect_timer);
    status = wifi_station_get_connect_status();
    if(status == STATION_GOT_IP)
    {
		os_printf("Wifi connect success!\r\n");
		wifi_get_ip_info(STATION_IF,&info);
		my_server_init(&info.ip,1213);
        return;
    }
    else
    {
        if(count >= 7)
        {
            os_printf("wifi connect failed！");
            return;
        }
	}
	os_timer_arm(&connect_timer, 2000, 0);
}

void ICACHE_FLASH_ATTR
smartconfig_done(sc_status status, void *pdata)
{
    switch(status) {
        case SC_STATUS_WAIT:
            os_printf("SC_STATUS_WAIT\n");
            break;
        case SC_STATUS_FIND_CHANNEL:
            os_printf("SC_STATUS_FIND_CHANNEL\n");
            break;
        case SC_STATUS_GETTING_SSID_PSWD:
            os_printf("SC_STATUS_GETTING_SSID_PSWD\n");
			sc_type *type = pdata;
            if (*type == SC_TYPE_ESPTOUCH) {
                os_printf("SC_TYPE:SC_TYPE_ESPTOUCH\n");
            } else {
                os_printf("SC_TYPE:SC_TYPE_AIRKISS\n");
            }
            break;
        case SC_STATUS_LINK:
            os_printf("SC_STATUS_LINK\n");
            struct station_config *sta_conf = pdata;
	
	        wifi_station_set_config(sta_conf);
	        wifi_station_disconnect();
	        wifi_station_connect();
            break;
		case SC_STATUS_LINK_OVER:
			status = 0;
            os_printf("SC_STATUS_LINK_OVER\n");
			smartconfig_stop();
			wifi_station_set_auto_connect(1);
			os_timer_setfn(&connect_timer, wifi_conned, NULL);
			os_timer_arm(&connect_timer, 2000, 0);
		
            break;
    }
	
}

/******************************************************************************
 * FunctionName : user_rf_cal_sector_set
 * Description  : SDK just reversed 4 sectors, used for rf init data and paramters.
 *                We add this function to force users to set rf cal sector, since
 *                we don't know which sector is free in user's application.
 *                sector map for last several sectors : ABCCC
 *                A : rf cal
 *                B : rf init data
 *                C : sdk parameters
 * Parameters   : none
 * Returns      : rf cal sector
*******************************************************************************/
uint32 ICACHE_FLASH_ATTR
user_rf_cal_sector_set(void)
{
    enum flash_size_map size_map = system_get_flash_size_map();
    uint32 rf_cal_sec = 0;

    switch (size_map) {
        case FLASH_SIZE_4M_MAP_256_256:
            rf_cal_sec = 128 - 5;
            break;

        case FLASH_SIZE_8M_MAP_512_512:
            rf_cal_sec = 256 - 5;
            break;

        case FLASH_SIZE_16M_MAP_512_512:
        case FLASH_SIZE_16M_MAP_1024_1024:
            rf_cal_sec = 512 - 5;
            break;

        case FLASH_SIZE_32M_MAP_512_512:
        case FLASH_SIZE_32M_MAP_1024_1024:
            rf_cal_sec = 1024 - 5;
            break;

        case FLASH_SIZE_64M_MAP_1024_1024:
            rf_cal_sec = 2048 - 5;
            break;
        case FLASH_SIZE_128M_MAP_1024_1024:
            rf_cal_sec = 4096 - 5;
            break;
        default:
            rf_cal_sec = 0;
            break;
    }

    return rf_cal_sec;
}


void ICACHE_FLASH_ATTR
user_rf_pre_init(void)
{
}

void ICACHE_FLASH_ATTR scan_done(void *arg,STATUS status)
{
	//
	if(status == OK)
	{
		
		os_timer_setfn(&connect_timer, wifi_conned, NULL);
		os_timer_arm(&connect_timer, 2000, 0);
	}
	else
	{
		
	}
}

void ICACHE_FLASH_ATTR
user_scan(void)
{
   struct scan_config config;
   struct station_config s_stacon;
   wifi_station_get_config_default(&s_stacon);
   os_memset(&config, 0, sizeof(config));

   config.ssid = s_stacon.ssid;

   wifi_station_scan(&config, scan_done);

}

void ICACHE_FLASH_ATTR btncheck()
{
	uint8 statu = 0;
	os_timer_disarm(&btntimer);
	if(GPIO_INPUT_GET(GPIO_ID_PIN(0))==0x00)
	{
		os_printf("start net config\r\n");
		while(GPIO_INPUT_GET(GPIO_ID_PIN(0))==0x00); 
		statu = 2;
		smartconfig_set_type(SC_TYPE_ESPTOUCH_AIRKISS); //SC_TYPE_ESPTOUCH,SC_TYPE_AIRKISS,SC_TYPE_ESPTOUCH_AIRKISS
		smartconfig_start(smartconfig_done);
		
	}
	os_timer_arm(&btntimer, 200, 0);
}

void ICACHE_FLASH_ATTR
user_init(void)
{
	
	uint8 opmode;
	struct softap_config config;
	struct station_config s_staconf;
	struct ip_info ipConfig;
	os_printf("SDK version:%s\n", system_get_sdk_version());
	wifi_set_opmode(STATION_MODE);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U,FUNC_GPIO0);//选择GPIO2
	GPIO_DIS_OUTPUT(GPIO_ID_PIN(2)); 
	
	wifi_station_get_config_default(&s_staconf);
	if (os_strlen(s_staconf.ssid) != 0) {
		os_printf("user_scan\n");
		system_init_done_cb(user_scan);
	} 
	
	os_timer_setfn(&btntimer,btncheck,NULL);
	os_timer_arm(&btntimer, 2000, 0);
}
