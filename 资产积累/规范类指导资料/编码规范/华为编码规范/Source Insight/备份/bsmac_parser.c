/*
 * File      : bsmac_parser.c
 * Change Logs:
 */

#include "bsmac_parser.h"
//#include "serial.h"
#include "bsmac_header.h"
#include "crc.h"
#include "nwk_protocol.h"
#include "bootcfg.h"
#include "msg_center.h"
#include "ptl_nwk.h"
#include "net_app.h"
#include "3g_watchdog.h"
#include "string.h"
#include "led_indicator.h"
#include <time.h>
#include <stdio.h>
#include <rtthread.h>
#include <mbus_app.h>
#include <sensor_magic.h>

#include "../../../../../version.h"
#include "3g_protocol.h"

//#define LOG_DEBUG
#include "3g_log.h"
#include <stm32f2xx.h>

/*****************************************************************************
*
* defines
*
*****************************************************************************/
//#define __STM32__


#define BSMAC_POOL_MAX   2048  			 // size of bsmac message dequeue

#define BSMAC_SEND_LIVE_TIMEOUT  5000   // send live every 1000 ms
#define BSMAC_POLL_PHONE_TIMEOUT  10000    // 10s timeout phone poll
#define BSMAC_LINK_TIMEOUT_COUNT 30     // if 30 no live, the link is down

#define BSMAC_SEND_DATA_TIMEOUT  600    // send data req  600 ms


#define BSMAC_LINK_LED_ON()
#define BSMAC_LINK_LED_OFF()

#define UARTSTATE_PREAMBLE_H 0
#define UARTSTATE_PREAMBLE_L 1
#define UARTSTATE_FRAME_CONTROL 2
#define UARTSTATE_RESERVERD 3
#define UARTSTATE_FRAME_COUNT_H 4
#define UARTSTATE_FRAME_COUNT_L 5
#define UARTSTATE_SRC_ADDR_H 6
#define UARTSTATE_SRC_ADDR_L 7
#define UARTSTATE_DST_ADDR_H 8
#define UARTSTATE_DST_ADDR_L 9
#define UARTSTATE_DATA_LEN_H 10
#define UARTSTATE_DATA_LEN_L 11
#define UARTSTATE_DATA 12

#define BSMAC_RX_LEN      512
#define DEBUG_BSMAC

/* max len */
/* use micros in bsmac_headr.h */
/* the bsmac tx and rx max len is 128 */

//#define BSMAC_MAX_TX_LEN 128
//#define BSMAC_MAX_RX_LEN 128

#define RT_DEBUG_BSMAC  RT_TRUE     // open or close debug log
/*****************************************************************************
*
* typedefs
*
*****************************************************************************/

typedef struct
{
    rt_device_t device;

    /* tx */
    rt_uint8_t tx_buf[BSMAC_TX_LEN_DEFAULT+10];
    rt_uint16_t tx_wantlen;
    rt_uint16_t tx_datalen;

    /* rx */
    rt_uint8_t rx_buf[UWB_RX_LEN_DEFAULT+10];      // point to rx_real_buf + MSG_HEADER_SIZE
    rt_uint16_t rx_wantlen;
    rt_uint16_t rx_datalen;

    rt_bool_t rx_ind;        // indicate the isr has received data

    /* state machine */
    rt_uint8_t state;

    TRANSMIT_ID_EM transmit_id;            // my transmit_id, uart4

    /*control */
    rt_bool_t use_ack;
    rt_uint16_t peer_addr; // mac address of device in uart 0 and 1
    rt_bool_t  peer_rdy;    //for now,  the peer is always ready

    rt_bool_t link_up; 	  				  // indicate link is up or down
    rt_uint16_t    linkStatus_counter;		 // link counter, 0-30

    /* status */
    //rt_bool_t on_tx;
    //  bool on_rx;

    /*frame counts */
    rt_uint16_t tx_frame_cnt;
    rt_uint16_t rx_frame_cnt;

} bsmac_hdl_t;

typedef struct
{
    rt_uint16_t phone_addr;
    rt_uint16_t station_panid;
    rt_tick_t tick;
    rt_bool_t info_ind;
    rt_uint8_t rx_buf[128];
} bsmac_phone_info_t;


/*****************************************************************************
*
* functions defination
*
*****************************************************************************/
void bsmac_init(void);;
void bsmac_set_device(const char* device_name,rt_uint8_t device_port);
rt_err_t bsmac_rx0_ind(rt_device_t dev, rt_size_t size);
rt_err_t bsmac_rx1_ind(rt_device_t dev, rt_size_t size);
rt_err_t bsmac_rx2_ind(rt_device_t dev, rt_size_t size);
void bsmac_tx_ind(rt_uint8_t port);
void bsmac_uart_recv(rt_uint8_t port);
void bsmac_parse_rx(rt_uint8_t* p, rt_uint16_t length, rt_uint8_t port);
rt_size_t bsmac_build_packet( unsigned char * pbuf,const unsigned char * pdata,
                                unsigned short len,const unsigned char frame_type, rt_uint8_t port);

static void bsmac_live_poll(void );
rt_err_t  bsmac_send_packet(rt_uint8_t* p, rt_uint16_t len, rt_uint8_t port);

rt_bool_t bsmac_parse_local_protocol(rt_uint8_t *p,rt_uint8_t port);

void _bsmac_analyser_callback(void *pvParam);
rt_uint8_t bsmac_search_phone_nwk(rt_uint16_t addr);
void station_state_report(void );


/*****************************************************************************
*
* variables
*
*****************************************************************************/
struct rt_messagequeue bsmac_mq;
static  rt_uint8_t bsmac_mq_pool[BSMAC_POOL_MAX];

static  bsmac_hdl_t bsmac_hdl[3];

static bsmac_phone_info_t bsmac_phone_info[10];


#ifdef DEBUG_BSMAC
static char bsmac_err_buf[256];
#endif

static rt_uint8_t bsmac_recv1_data_cnt = 0;
static rt_uint8_t bsmac_recv2_data_cnt = 0;

static rt_tick_t last_data_recv_tick;



/*****************************************************************************
*
* functions
*
*****************************************************************************/
void station_state_report(void )
{
	rt_uint8_t send_buf[MSG_COM_PKT_SIZE];
    	struct nwkhdr *pNwkHdr = (struct nwkhdr *)send_buf;
    	APP_HDR_T *pstAppHdr = (APP_HDR_T *)(pNwkHdr + 1);

	pNwkHdr->type = NWK_DATA;
	pNwkHdr->ttl = 1;
	pNwkHdr->src = sys_option.u32BsId;
	pNwkHdr->dst = 0xffff;
	pNwkHdr->len = sizeof(APP_HDR_T);

	pstAppHdr->protocoltype = APP_PROTOCOL_TYPE_SENSOR_STATION;
	pstAppHdr->msgtype = COMM_SENSOR_STATUS_REPORT;
	pstAppHdr->len = 0;
	bsmac_send_packet(send_buf, sizeof(struct nwkhdr) + pNwkHdr->len, 0);	
}


void bsmac_thread_entry(void* parameter)
{
    bsmac_init();
    station_state_report();
    while(1)
    {
        static rt_uint8_t rev_buf[MSG_MINITOR_SIZE];

         /* 网络线程启动后由网络线程喂狗 */
        if (!iwdg_net_feed_flag) {
            feed_watchdog();
        }

        if(rt_mq_recv(&bsmac_mq, rev_buf, sizeof(rev_buf), RT_WAITING_NO) == RT_EOK)
        {
		//todo:按port分发
		struct nwkhdr *pNwkHdr = (struct nwkhdr *)rev_buf;
		APP_HDR_T *pstAppHdr = (APP_HDR_T *)(pNwkHdr + 1);

		if(sizeof(struct nwkhdr) + pNwkHdr->len < MSG_MINITOR_SIZE)
		{              
			if((pNwkHdr->src == 0xFFFF) || ( pNwkHdr->src == sys_option.u32BsId))
			{
	                        rt_kprintf(" msgtype = %d\n",pstAppHdr->msgtype);
				bsmac_send_packet(rev_buf, sizeof(struct nwkhdr) + pNwkHdr->len, 0);
				rt_kprintf(" sensor data had send by usart\n");
			}
			else
			{
				if(APP_TOF_MSG_ALARM_ACK == pstAppHdr->msgtype)
				{
					bsmac_send_packet(rev_buf, sizeof(struct nwkhdr) + pNwkHdr->len, 0);
				}                        
				else if(pNwkHdr->dst > 39999)
				{
				    	bsmac_send_packet(rev_buf, sizeof(struct nwkhdr) + pNwkHdr->len, 0);
				}		           
			}
		}
		else
		{
			RT_DEBUG_LOG(RT_DEBUG_BSMAC, ("BSMac: bsmac_mq too long %d \n",sizeof(struct nwkhdr) + pNwkHdr->len ));
		}
        }
        else if (bsmac_hdl[0].rx_ind )
        {
        	//bsmac_hdl.rx_ind = RT_FALSE;
            
		bsmac_uart_recv(0);
                bsmac_hdl[0].rx_ind = RT_FALSE;
        }
        else
        {
		bsmac_live_poll();
		rt_thread_delay(1);
        }
    }
}

rt_bool_t bsmac_get_link_status(rt_uint8_t port)
{
    return bsmac_hdl[port].link_up;
}

rt_uint16_t bsmac_get_peer_addr(rt_uint8_t port)
{
    return bsmac_hdl[port].peer_addr;
}

void bsmac_init()
{
    //rt_hw_tick_get_microsecond();

    /* message queue */

    rt_mq_init(&bsmac_mq, "BSMACMQ", bsmac_mq_pool, MSG_MINITOR_SIZE,
               sizeof(bsmac_mq_pool), RT_IPC_FLAG_FIFO);
    /* bsmac hdl */
    memset(bsmac_hdl, 0, sizeof(bsmac_hdl));

    bsmac_hdl[0].use_ack  = RT_TRUE;

    /* uart 1*/
    bsmac_hdl[0].transmit_id = COM3_TRANSMIT_ID;

    bsmac_set_device("uart1",0);
  
    msg_analyser_register(COM3_TRANSMIT_ID, _bsmac_analyser_callback);
 
}

void bsmac_set_device(const char* device_name,rt_uint8_t device_port)
{
    bsmac_hdl[device_port].device = rt_device_find(device_name);

    if (bsmac_hdl[device_port].device != RT_NULL && rt_device_open(bsmac_hdl[device_port].device, RT_DEVICE_OFLAG_RDWR) == RT_EOK)
    {
        //rt_device_set_rx_indicate(bsmac_hdl[device_port].device, bsmac_rx_ind);
        switch(device_port)
        {
            case 0:
                rt_device_set_rx_indicate(bsmac_hdl[device_port].device, bsmac_rx0_ind);
                break;
            case 1:
                rt_device_set_rx_indicate(bsmac_hdl[device_port].device, bsmac_rx1_ind);
                break;
            case 2:
                rt_device_set_rx_indicate(bsmac_hdl[device_port].device, bsmac_rx2_ind);
                break;
            default:
                RT_DEBUG_LOG(RT_DEBUG_BSMAC,("device port error:%s\n", device_port));
        }
    }
    else
    {
        if (bsmac_hdl[device_port].device != RT_NULL)
        {
            rt_device_close(bsmac_hdl[device_port].device);
        }
        RT_DEBUG_LOG(RT_DEBUG_BSMAC,("can not open device:%s\n", device_name));
    }
}

/* run in isr */
static rt_err_t bsmac_rx0_ind(rt_device_t dev, rt_size_t size)
{
    /* release semaphore to let finsh thread rx data */
    bsmac_hdl[0].rx_ind = RT_TRUE;
    return RT_EOK;
}

/* run in isr */
static rt_err_t bsmac_rx1_ind(rt_device_t dev, rt_size_t size)
{
    /* release semaphore to let finsh thread rx data */
    bsmac_hdl[1].rx_ind = RT_TRUE;
    return RT_EOK;
}

/* run in isr */
static rt_err_t bsmac_rx2_ind(rt_device_t dev, rt_size_t size)
{
    /* release semaphore to let finsh thread rx data */
    bsmac_hdl[2].rx_ind = RT_TRUE;
    return RT_EOK;
}



rt_err_t  bsmac_send_packet(rt_uint8_t* p, rt_uint16_t len, rt_uint8_t port)
{
    rt_size_t size = bsmac_build_packet(bsmac_hdl[port].tx_buf, p, len, BSMAC_FRAME_TYPE_DATA, port);
    rt_kprintf(" size = %d\n",size);
    if(size >0 && size <= UWB_RX_LEN_DEFAULT)
    {
        if(rt_device_write(bsmac_hdl[port].device, 0, bsmac_hdl[port].tx_buf, size) != size)
        {
            RT_DEBUG_LOG(RT_DEBUG_BSMAC,("send pkt error %d\n", size));
        }
        return RT_EOK;
    }
    else
    {
        RT_DEBUG_LOG(RT_DEBUG_BSMAC,("send pkt err %x %d\n", p, len));

        return RT_ERROR;
    }
}



static  void bsmac_uart_recv(rt_uint8_t port)
{
    rt_uint8_t data;
    rt_size_t len;
    rt_uint16_t i;
    bsmac_hdl_t *pHdl;
    bsmac_header_t *pbsmac;
    rt_uint8_t read_continue = 1;

    static rt_uint8_t uart_buf[BSMAC_RX_LEN];

    pHdl = &bsmac_hdl[port];
    pbsmac =  ( bsmac_header_t *)bsmac_hdl[port].rx_buf;
    while(read_continue--)
    {
    len = rt_device_read(bsmac_hdl[port].device, 0, uart_buf, BSMAC_RX_LEN);
    if(len == BSMAC_RX_LEN)
        read_continue = 1;
    if (len > 0)
    {
        for(i=0; i<len; i++)
        {
            data = uart_buf[i];

            switch(pHdl->state)
            {
            case UARTSTATE_PREAMBLE_H:
            {
                if(data == BSMAC_PREAMBLE_H)
                {
                    pbsmac->preamble_H = data;
                    pHdl->state = UARTSTATE_PREAMBLE_L;
                }
                break;
            }
            case UARTSTATE_PREAMBLE_L:
            {
                if(data == BSMAC_PREAMBLE_L)
                {
                    pbsmac->preamble_L  = data;
                    pHdl->state = UARTSTATE_FRAME_CONTROL;
                }
                else
                {
                    pHdl->state = UARTSTATE_PREAMBLE_H;
                }
                break;
            }
            case UARTSTATE_FRAME_CONTROL:
            {
                pbsmac->frame_control= data;
                pHdl->state = UARTSTATE_RESERVERD;
                break;
            }
            case UARTSTATE_RESERVERD:
            {
                pbsmac->reserverd = data;
                pHdl->state = UARTSTATE_FRAME_COUNT_H;
                break;

            }
            case UARTSTATE_FRAME_COUNT_H:
            {
                pbsmac->frame_count_H = data;
                pHdl->state = UARTSTATE_FRAME_COUNT_L;
                break;
            }
            case UARTSTATE_FRAME_COUNT_L:
            {
                pbsmac->frame_count_L = data;
                pHdl->state = UARTSTATE_SRC_ADDR_H;
                break;
            }
            case UARTSTATE_SRC_ADDR_H:
            {
                pbsmac->src_addr_H= data;
                pHdl->state = UARTSTATE_SRC_ADDR_L;
                break;
            }

            case UARTSTATE_SRC_ADDR_L:
            {
                pbsmac->src_addr_L = data;
                pHdl->state = UARTSTATE_DST_ADDR_H;
                break;
            }
            case UARTSTATE_DST_ADDR_H:
            {
                pbsmac->dst_addr_H = data;
                pHdl->state = UARTSTATE_DST_ADDR_L;
                break;
            }

            case UARTSTATE_DST_ADDR_L:
            {
                pbsmac->dst_addr_L = data;
                pHdl->state = UARTSTATE_DATA_LEN_H;
                break;
            }
            case UARTSTATE_DATA_LEN_H:
            {
                pbsmac->data_len_H = data;
                pHdl->state = UARTSTATE_DATA_LEN_L;
                break;
            }
            case UARTSTATE_DATA_LEN_L:
            {
                pbsmac->data_len_L = data;

                pHdl->rx_wantlen = (pbsmac->data_len_H<<8 | pbsmac->data_len_L); // + 2; len is including crc
                rt_kprintf("wantlen %d\n",pHdl->rx_wantlen);
                if(pHdl->rx_wantlen <= (UWB_RX_LEN_DEFAULT - sizeof(bsmac_header_t)))
                {
                    pHdl->rx_datalen =0;
                    pHdl->state = UARTSTATE_DATA;
                }
                else
                {
                    pHdl->state = UARTSTATE_PREAMBLE_H;
                }
                break;
            }
            case UARTSTATE_DATA:
            {
                if (sizeof(bsmac_header_t) + pHdl->rx_datalen >= UWB_RX_LEN_DEFAULT)
                {
                    pHdl->state = UARTSTATE_PREAMBLE_H;
                    break;
                }
                pHdl->rx_buf[sizeof(bsmac_header_t) + pHdl->rx_datalen] = data;
                if(++pHdl->rx_datalen >= pHdl->rx_wantlen)
                {
					rt_kprintf("rxlen %d\n",pHdl->rx_datalen);
					bsmac_parse_rx( pHdl->rx_buf, sizeof(bsmac_header_t) + pHdl->rx_datalen,port);

                    pHdl->state = UARTSTATE_PREAMBLE_H;
                }
                else
                {
                    pHdl->state = UARTSTATE_DATA;
                }
                break;
            }
            }
        }
    }
}
}


void bsmac_parse_rx(rt_uint8_t* p, rt_uint16_t length,rt_uint8_t port)
{
	struct nwkhdr *pnwkhdr;
	bsmac_header_t *ph;
	char buf_rev[512];
	rt_uint8_t frame_type, len;
	//rt_uint8_t device_type;
	rt_uint16_t crc, crc_recv;
	rt_bool_t flag;
	rt_uint16_t rx_fc;
	MSG_CENTER_HEADER_T  * pCenterHeader;
	rt_err_t err = 0;
	static rt_uint8_t rssi_modules_link_state;

	 //struct nwkhdr *pNwkHdr;
	//app_header_t	*pAppHdr;


	if(p == RT_NULL|| length > UWB_RX_LEN_DEFAULT)
	{
		RT_DEBUG_LOG(RT_DEBUG_BSMAC,("BSmac: p or len error  \n"));
		return;
	}

	ph = (bsmac_header_t *) p;
	pnwkhdr = (struct nwkhdr *)(ph+1);

	// check preamble
	flag = (ph->preamble_H != (unsigned char) BSMAC_PREAMBLE_H)
		   || (ph->preamble_L != (unsigned char) BSMAC_PREAMBLE_L);
	if (flag)
	{
#ifdef DEBUG_BSMAC
			int report_len = (length < sizeof(bsmac_err_buf)) ?
				length : sizeof(bsmac_err_buf);
			memset(bsmac_err_buf, 0, sizeof(bsmac_err_buf));
			snprintf(bsmac_err_buf, sizeof(bsmac_err_buf),
				"BSmac: header error\n");
			net_report_running_state_msg(ARM_ERROR, bsmac_err_buf,
				sizeof(bsmac_err_buf));
			memset(bsmac_err_buf, 0, sizeof(bsmac_err_buf));
			memcpy(bsmac_err_buf, p, report_len);
			net_report_running_state_msg(ARM_ERROR, bsmac_err_buf,
				sizeof(bsmac_err_buf));
#endif
		RT_DEBUG_LOG(RT_DEBUG_BSMAC,("BSmac: header error \n"));
		return;
	}

	// check frame type and device type
	frame_type = BSMAC_GET_FRAMETYPE(ph->frame_control);
	//device_type = BSMAC_GET_DEVICETYPE(ph->frame_control);
	if ((frame_type > BSMAC_FRAME_TYPE_LIVE))
	{
#ifdef DEBUG_BSMAC
			int report_len = (length < sizeof(bsmac_err_buf)) ?
				length : sizeof(bsmac_err_buf);
			memset(bsmac_err_buf, 0, sizeof(bsmac_err_buf));
			snprintf(bsmac_err_buf, sizeof(bsmac_err_buf),
				"BSmac: frame type error\n");
			net_report_running_state_msg(ARM_ERROR, bsmac_err_buf,
				sizeof(bsmac_err_buf));
			memset(bsmac_err_buf, 0, sizeof(bsmac_err_buf));
			memcpy(bsmac_err_buf, p, report_len);
			net_report_running_state_msg(ARM_ERROR, bsmac_err_buf,
				sizeof(bsmac_err_buf));
#endif
		RT_DEBUG_LOG(RT_DEBUG_BSMAC,("BSmac: frame type error\n"));
		return;
	}

	//check len
	len = (ph->data_len_H << 8) | ph->data_len_L;
	
	if ((frame_type == BSMAC_FRAME_TYPE_LIVE))
	{
		if(len!=128 - BSMAC_HEADER_LEN)
		{
#ifdef DEBUG_BSMAC
			int report_len = (length < sizeof(bsmac_err_buf)) ?
				length : sizeof(bsmac_err_buf);
			memset(bsmac_err_buf, 0, sizeof(bsmac_err_buf));
			snprintf(bsmac_err_buf, sizeof(bsmac_err_buf),
				"BSmac: live len %d error\n", len);
			net_report_running_state_msg(ARM_ERROR, bsmac_err_buf,
				sizeof(bsmac_err_buf));
			memset(bsmac_err_buf, 0, sizeof(bsmac_err_buf));
			memcpy(bsmac_err_buf, p, report_len);
			net_report_running_state_msg(ARM_ERROR, bsmac_err_buf,
				sizeof(bsmac_err_buf));
#endif
			RT_DEBUG_LOG(RT_DEBUG_BSMAC,("BSmac: live len %d error\n", len));
			return;
		}
	}
	else if(frame_type == BSMAC_FRAME_TYPE_ACK)  // fixme: FPGA send a long ACK?
	{
		len = BSMAC_FOOTER_LEN;
	}
	else
	{
		if (len > (UWB_RX_LEN_DEFAULT - BSMAC_HEADER_LEN))
		{
#ifdef DEBUG_BSMAC
			int report_len = (length < sizeof(bsmac_err_buf)) ?
				length : sizeof(bsmac_err_buf);
			memset(bsmac_err_buf, 0, sizeof(bsmac_err_buf));
			snprintf(bsmac_err_buf, sizeof(bsmac_err_buf),
				"BSmac: phy len %d error\n", len);
			net_report_running_state_msg(ARM_ERROR, bsmac_err_buf,
				sizeof(bsmac_err_buf));
			memset(bsmac_err_buf, 0, sizeof(bsmac_err_buf));
			memcpy(bsmac_err_buf, p, report_len);
			net_report_running_state_msg(ARM_ERROR, bsmac_err_buf,
				sizeof(bsmac_err_buf));
#endif
			RT_DEBUG_LOG(RT_DEBUG_BSMAC,("BSmac: phy len %d error\n", len));
			return;
		}

	}
	// ack do not check crc
	if (frame_type == BSMAC_FRAME_TYPE_LIVE
			||frame_type == BSMAC_FRAME_TYPE_DATA)
	{
		crc = CRC16((unsigned char *) (p+ 2), len + BSMAC_HEADER_LEN
					- BSMAC_FOOTER_LEN - 2, 0xffff); // caculate header and payload
		crc_recv = ((*(p+len + BSMAC_HEADER_LEN - BSMAC_FOOTER_LEN) << 8)
					| *(p+len + BSMAC_HEADER_LEN - BSMAC_FOOTER_LEN + 1));

		if (crc != crc_recv)
		{
#ifdef DEBUG_BSMAC
			int report_len = (length < sizeof(bsmac_err_buf)) ?
				length : sizeof(bsmac_err_buf);
			memset(bsmac_err_buf, 0, sizeof(bsmac_err_buf));
			snprintf(bsmac_err_buf, sizeof(bsmac_err_buf),
				"bsmac rec crc error calc crc %d rec crc %d", crc, crc_recv);
			net_report_running_state_msg(ARM_ERROR, bsmac_err_buf,
				sizeof(bsmac_err_buf));
			memset(bsmac_err_buf, 0, sizeof(bsmac_err_buf));
			memcpy(bsmac_err_buf, p, report_len);
			net_report_running_state_msg(ARM_ERROR, bsmac_err_buf,
				sizeof(bsmac_err_buf));
#endif
			RT_DEBUG_LOG(RT_DEBUG_BSMAC,("BSmac: crc error %d,%d\n",crc,crc_recv));

			return;
		}
		

	}

	// check ready;
	bsmac_hdl[port].peer_rdy = BSMAC_GET_RDY(ph->frame_control);

	// check frame_cnt
	rx_fc = (ph->frame_count_H << 8) | ph->frame_count_L;

	if (frame_type != BSMAC_FRAME_TYPE_ACK)
	{
		bsmac_hdl[port].rx_frame_cnt = rx_fc;
	}

	switch (frame_type)
	{

		case (BSMAC_FRAME_TYPE_DATA):
		{
			/* 首先处理本地需要处理的消息，本地不处理的消息发送给analyser*/
			if(!bsmac_parse_local_protocol(p+sizeof(bsmac_header_t),port))
			{
			/* if reveive data, send it to	analyser */
				pCenterHeader = (MSG_CENTER_HEADER_T* )(p+sizeof(bsmac_header_t));

				/* 去掉mac头之后，直接前跳一个MSG_CENTER_HEADER_T，省去拷贝*/
				pCenterHeader--;

				pCenterHeader->u32MsgId = bsmac_hdl[port].transmit_id;
				rt_memcpy(buf_rev, p+sizeof(bsmac_header_t), sizeof(struct nwkhdr)+sizeof(app_header_t));				
				err = rt_mq_send(&msg_analyser_mq,(void*)pCenterHeader,MSG_HEADER_SIZE + len - 2);
#ifdef DEBUG_BSMAC
				if (err != RT_EOK)
				{
					memset(bsmac_err_buf, 0, sizeof(bsmac_err_buf));
					snprintf(bsmac_err_buf, sizeof(bsmac_err_buf),
						"bsmac send to msg_analyser_mq ret %d", err);
					net_report_running_state_msg(ARM_ERROR, bsmac_err_buf,
						sizeof(bsmac_err_buf));
				}
#endif
				if(err == -RT_EFULL)  // -2 remove crc
				{
					//ERROR_LOG("send to msg_analyser_mq full\n");
				}
				else if(err == -RT_ERROR)
				{
					//ERROR_LOG("send to msg_analyser_mq err\n");
				}
			}
			/* 如果有一个从机就将指示灯点亮 */
			/*if (rssi_modules_link_state)
			{
				light_up_rssi_modules_data_indicator();
				rssi_modules_link_state=0;
			}
			else
			{
				light_off_rssi_modules_data_indicator();
				rssi_modules_link_state=1;
			}*/

			break;
		}
		case (BSMAC_FRAME_TYPE_LIVE):
		{
			if(bsmac_hdl[port].use_ack)
			{
				// rt_thread_delay(1);
				//bsmac_send_ack(port);
			}
			break;
		}
		case (BSMAC_FRAME_TYPE_ACK):
		{
			bsmac_hdl[port].link_up = RT_TRUE;
			bsmac_hdl[port].linkStatus_counter = BSMAC_LINK_TIMEOUT_COUNT;
			BSMAC_LINK_LED_ON();
			break;
		}
	}

}

rt_size_t bsmac_build_packet( unsigned char * pbuf,
                              const unsigned char * pdata, unsigned short len,
                              const unsigned char frame_type, rt_uint8_t port)
{
    unsigned short tx_len;
    unsigned short crc;
    bsmac_header_t *ph;

    // add mac header
    if (pbuf == NULL || len > BSMAC_MINITOR_TX_PAYLOAD_LEN )
    {
        RT_DEBUG_LOG(RT_DEBUG_BSMAC,("Build Failed pbuf %X len%d\n", pbuf, len));
        return 0;
    }

    ph = (bsmac_header_t *) pbuf;

    ph->preamble_H = BSMAC_PREAMBLE_H;
    ph->preamble_L = BSMAC_PREAMBLE_L;

    BSMAC_SET_DEVICETYPE(ph->frame_control, BSMAC_DEVICE_TYPE_BS_EP);    // I am FPGA
    BSMAC_SET_RDY(ph->frame_control, 1);           							// always ready
    BSMAC_SET_FRAMETYPE(ph->frame_control, frame_type);
    BSMAC_SET_PRIORITY(ph->frame_control, 1);

    if (frame_type == BSMAC_FRAME_TYPE_ACK) // for ack, use recieved frame_cnt
    {
        ph->frame_count_H = (bsmac_hdl[port].rx_frame_cnt & 0xff00) >> 8;
        ph->frame_count_L = bsmac_hdl[port].rx_frame_cnt  & 0xff;
    }
    else
    {
        ph->frame_count_H = ( bsmac_hdl[port].tx_frame_cnt & 0xff00) >> 8; // framecnt_h
        ph->frame_count_L =  bsmac_hdl[port].tx_frame_cnt& 0xff; // framecnt_l
        bsmac_hdl[port].tx_frame_cnt++;
    }

    ph->src_addr_H = (sys_option.u32BsId >> 8) & 0xff;
    ph->src_addr_L = (sys_option.u32BsId) & 0xff;            // source mac address
    ph->dst_addr_H = 0;                                                     // dst address is useless
    ph->dst_addr_L = 0;
    ph->reserverd = port;

    /* ack do not need payload, Live may have payload */
    if (len != 0 && pdata && frame_type != BSMAC_FRAME_TYPE_ACK)
    {
        memcpy((void*) (pbuf + BSMAC_HEADER_LEN), pdata, len);
    }

    //LIVE packet needs to be a long frame
    if (frame_type == BSMAC_FRAME_TYPE_LIVE)
    {
        len = BSMAC_MAX_TX_PAYLOAD_LEN;
    }
    else if(frame_type == BSMAC_FRAME_TYPE_ACK)
    {
        len = 0;
    }

    tx_len = len + BSMAC_FOOTER_LEN; // length = payload+footer
    ph->data_len_H = (tx_len >> 8) & 0xff; //
    ph->data_len_L = tx_len & 0xff; //

    crc = CRC16((unsigned char *)(pbuf+2), len+BSMAC_HEADER_LEN-2, 0xffff);   // caculate header and payload

    // padding footer
    pbuf[len+BSMAC_HEADER_LEN] = (crc >> 8) & 0xff;
    pbuf[len+BSMAC_HEADER_LEN+1] = crc & 0xff;

    return sizeof(bsmac_header_t) + tx_len;
}


static void bsmac_live_poll(void )
{
	rt_tick_t tick, diff_tick;
	static rt_tick_t last_tick;
	static rt_uint8_t time;
		
	tick = rt_tick_get();
	diff_tick = tick  - last_tick;

	if(diff_tick > BSMAC_SEND_LIVE_TIMEOUT/(1000/RT_TICK_PER_SECOND))
	{
		last_tick = tick;
		station_state_report();
	}
	return;
}







rt_bool_t bsmac_parse_local_protocol(rt_uint8_t *p,rt_uint8_t port)
{
    struct nwkhdr *pNwkHdr;
    app_header_t  *pAppHdr;

    rt_bool_t parsed;
    //rt_uint8_t idx;
	//rt_uint8_t i,j;

    static rt_uint8_t pAckBuf[128];

    struct nwkhdr *pAckNwkHdr = (struct nwkhdr *)pAckBuf;
    app_header_t *pAckHeader = (app_header_t *)(pAckNwkHdr + 1);

    parsed = RT_FALSE;

    pNwkHdr = (struct nwkhdr *)p;
    pAppHdr = (app_header_t  *)(pNwkHdr+1);

    switch(pAppHdr->msgtype)
    {
  	  case APP_UWB_MSG_REPORT:
 	  {
  	      app_LSrfReport_t *pStationReport = (app_LSrfReport_t *)(pAppHdr + 1);

  	      if(pStationReport->reporttype == APP_LS_REPORT_LIVE)
 	      {
  	          app_LSrfReport_t *pAckStationReport = (app_LSrfReport_t *)(pAckHeader + 1);

            	   pAckNwkHdr->type = NWK_DATA;
         	   pAckNwkHdr->ttl = 1;
           	   pAckNwkHdr->src = sys_option.u32BsId;
         	   pAckNwkHdr->dst = pNwkHdr->src;

       	   	   pAckNwkHdr->len =  sizeof(app_header_t) + sizeof(app_LSrfReport_t);
	
	          pAckHeader->len = sizeof(app_LSrfReport_t);
                 pAckHeader->msgtype = APP_TOF_MSG_REPORT_ACK;
            //if(port)
                 pAckHeader->protocoltype = APP_PROTOCOL_TYPE_UWB_CARD;
            //else
               // pAckHeader->protocoltype = APP_PROTOCOL_TYPE_CARD;

            	  pAckStationReport->hdr.dstaddr =  pStationReport->hdr.srcaddr;
            	  pAckStationReport->hdr.srcaddr = sys_option.u32BsId;
            	  pAckStationReport->len = 0;

            	  pAckStationReport->reporttype = APP_LS_REPORT_LIVE;
            	  pAckStationReport->devtype = BSMAC_DEVICE_TYPE_LOC;
            	  pAckStationReport->seqnum = pStationReport->seqnum;

           	  bsmac_send_packet(pAckBuf, sizeof(struct nwkhdr) + pAckNwkHdr->len,port);

            	//rt_mq_send(&bsmac_mq, pAckBuf,
                      //sizeof(struct nwkhdr)+sizeof(app_header_t)+sizeof(app_LSrfReport_t));

                parsed = RT_TRUE;
        }
        break;
    }

#if 1
    case APP_UWB_MSG_DISTANCE:     //收到主站定位数据时，向副站要求定位数据
    {

        break;
    }
#endif

    }
    return parsed;
}


/* this function runs in msg_analyser thread !!*/
static void _bsmac_analyser_callback(void *pvParam)
{
    const MSG_CENTER_HEADER_T  *pCenterHeader = (MSG_CENTER_HEADER_T *)pvParam;
    static rt_uint8_t              au8Buf[MSG_ANALYSER_PKT_SIZE];
    rt_err_t err = 0;
	
    if (pvParam != RT_NULL && bsmac_hdl[0].transmit_id == pCenterHeader->u32MsgId)
    {
        NWK_HDR_T *pstNwkHd = (NWK_HDR_T*)((rt_uint8_t*)pvParam + MSG_HEADER_SIZE);
        app_header_t *pstAppHd = (app_header_t *)(pstNwkHd +1);

        rt_kprintf(" data send len = %d\n ",pstAppHd->len);
        rt_memcpy(au8Buf, pstNwkHd, sizeof(NWK_HDR_T)+pstNwkHd->len);
        
        err = rt_mq_send(&sensor_mq, au8Buf, sizeof(NWK_HDR_T)+pstNwkHd->len);
        if(err == -RT_EFULL)
        {
            ERROR_LOG("send to sensor_mq full\n");
        }
        else if(err == -RT_ERROR)
        {
            ERROR_LOG("send to sensor_mq err\n");
        }
    }
}

static rt_thread_t bsmac_thread = RT_NULL;

rt_bool_t start_bsmac_work()
{
    bsmac_thread = rt_thread_create("bsmac", bsmac_thread_entry,
                                    RT_NULL, 2048, 9, 20);

    if (bsmac_thread == RT_NULL)
    {
        ERROR_LOG("create bsmac work thread failed\n");
        return RT_FALSE;
    }

    rt_thread_startup(bsmac_thread);
    DEBUG_LOG("the bsmac thread start up\n");

    return RT_TRUE;
}

void stop_bsmac_work()
{
    rt_enter_critical();

    if (bsmac_thread != RT_NULL && bsmac_thread->stat != RT_THREAD_CLOSE)
    {
        rt_thread_delete(bsmac_thread);
    }

    rt_exit_critical();

    TIME_LOG(LOG_CRITICAL, "stop bsmac thread\n");
}

