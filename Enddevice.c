#include "OSAL.h"
#include "ZGlobals.h"
#include "AF.h"
#include "aps_groups.h"
#include "ZDApp.h"

#include "SampleApp.h"
#include "SampleAppHw.h"

#include "OnBoard.h"

#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "MT_UART.h"
#include "MT_APP.h"
#include "MT.h"
#include "ds18b20.h"


#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr)[0])
#define PWM P0_6

uint8 AppTitle[] = "ALD2530 DS18B20"; //应用程序名称 

const cId_t SampleApp_ClusterList[SAMPLEAPP_MAX_CLUSTERS] =
{
  SAMPLEAPP_PERIODIC_CLUSTERID,
  SAMPLEAPP_FLASH_CLUSTERID,
  SAMPLEAPP_P2P_CLUSTERID  
};

const SimpleDescriptionFormat_t SampleApp_SimpleDesc =
{
  SAMPLEAPP_ENDPOINT,              
  SAMPLEAPP_PROFID,                
  SAMPLEAPP_DEVICEID,              
  SAMPLEAPP_DEVICE_VERSION,        
  SAMPLEAPP_FLAGS,                 
  SAMPLEAPP_MAX_CLUSTERS,          
  (cId_t *)SampleApp_ClusterList,  
  SAMPLEAPP_MAX_CLUSTERS,          
  (cId_t *)SampleApp_ClusterList   
};


endPointDesc_t SampleApp_epDesc;


uint8 SampleApp_TaskID;   
.
devStates_t SampleApp_NwkState;

uint8 SampleApp_TransID;  

uint8 count=1;
char pluse=0;
char state1;
unsigned char uartbuf[128];
afAddrType_t SampleApp_Periodic_DstAddr; //广播
afAddrType_t SampleApp_Flash_DstAddr;    //组播
afAddrType_t SampleApp_P2P_DstAddr;      //点播

aps_Group_t SampleApp_Group;

uint8 SampleAppPeriodicCounter = 0;
uint8 SampleAppFlashCounter = 0;


void SampleApp_HandleKeys( uint8 shift, uint8 keys );
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void SampleApp_SendPeriodicMessage( void );
void SampleApp_SendFlashMessage( uint16 flashTime );
void SampleApp_Send_P2P_Message(void);
static void rxCB(uint8 port,uint8 event);

void SampleApp_Init( uint8 task_id )
{ 
  halUARTCfg_t uartConfig;
  SampleApp_TaskID = task_id;
  SampleApp_NwkState = DEV_INIT;
  SampleApp_TransID = 0;
  
  MT_UartInit();                  //串口初始化
  MT_UartRegisterTaskID(task_id); //注册串口任务
  P0SEL &= 0x7f;                  //DS18B20的io口初始化 p0.7
    P0DIR &=0xBF;
  state1=PWM;
  uartConfig.configured  = TRUE;
  uartConfig.baudRate  =HAL_UART_BR_115200;
  uartConfig.flowControl  =FALSE;
  uartConfig.callBackFunc  =rxCB;
  HalUARTOpen(0,&uartConfig);


 #if defined ( BUILD_ALL_DEVICES )

  if ( readCoordinatorJumper() )
    zgDeviceLogicalType = ZG_DEVICETYPE_COORDINATOR;
  else
    zgDeviceLogicalType = ZG_DEVICETYPE_ROUTER;
#endif 

#if defined ( HOLD_AUTO_START )

  ZDOInitDevice(0);
#endif

  SampleApp_Periodic_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
  SampleApp_Periodic_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Periodic_DstAddr.addr.shortAddr = 0xFFFF;

  SampleApp_Flash_DstAddr.addrMode = (afAddrMode_t)afAddrGroup;
  SampleApp_Flash_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Flash_DstAddr.addr.shortAddr = SAMPLEAPP_FLASH_GROUP;
  
  SampleApp_P2P_DstAddr.addrMode = (afAddrMode_t)Addr16Bit; //点播 
  SampleApp_P2P_DstAddr.endPoint = SAMPLEAPP_ENDPOINT; 
  SampleApp_P2P_DstAddr.addr.shortAddr = 0x0000;            //发给协调器

  SampleApp_epDesc.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_epDesc.task_id = &SampleApp_TaskID;
  SampleApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&SampleApp_SimpleDesc;
  SampleApp_epDesc.latencyReq = noLatencyReqs;

  afRegister( &SampleApp_epDesc );

  RegisterForKeys( SampleApp_TaskID );

  SampleApp_Group.ID = 0x0001;
  osal_memcpy( SampleApp_Group.name, "Group 1", 7 );
  aps_AddGroup( SAMPLEAPP_ENDPOINT, &SampleApp_Group );

#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "SampleApp", HAL_LCD_LINE_1 );
#endif
}
static void rxCB(uint8 port,uint8 event)
{
  HalUARTRead(0,uartbuf,5);
    AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_CONTROL_CLUSTERID,
                       5,
                       uartbuf,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS;
  osal_memset(uartbuf,0,5);
}

uint16 SampleApp_ProcessEvent( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  (void)task_id;  

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {

        case KEY_CHANGE:
          SampleApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        case AF_INCOMING_MSG_CMD:
          SampleApp_MessageMSGCB( MSGpkt );
          break;

        case ZDO_STATE_CHANGE:
          SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (SampleApp_NwkState == DEV_ROUTER)
              || (SampleApp_NwkState == DEV_END_DEVICE) )
          {
            osal_start_timerEx( SampleApp_TaskID,
                              SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
                              SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT );
          }
          else
          {
          }
          break;

        default:
          break;
      }

      osal_msg_deallocate( (uint8 *)MSGpkt );

      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    }

    return (events ^ SYS_EVENT_MSG);
  }


  if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT )
  {

    SampleApp_Send_P2P_Message();

    osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
        (SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT + (osal_rand() & 0x00FF)) );

    return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT);
  }

  return 0;
}


void SampleApp_HandleKeys( uint8 shift, uint8 keys )
{
  (void)shift; 
  
  if ( keys & HAL_KEY_SW_1 )
  {

    SampleApp_SendFlashMessage( SAMPLEAPP_FLASH_DURATION );
  }

  if ( keys & HAL_KEY_SW_2 )
  {

    aps_Group_t *grp;
    grp = aps_FindGroup( SAMPLEAPP_ENDPOINT, SAMPLEAPP_FLASH_GROUP );
    if ( grp )
    {

      aps_RemoveGroup( SAMPLEAPP_ENDPOINT, SAMPLEAPP_FLASH_GROUP );
    }
    else
    {
      aps_AddGroup( SAMPLEAPP_ENDPOINT, &SampleApp_Group );
    }
  }
}

void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  uint16 flashTime;
  unsigned char buffer[5]="  ";
  switch ( pkt->clusterId )
  {
    case SAMPLEAPP_P2P_CLUSTERID:
      HalUARTWrite(0, pkt->cmd.Data, pkt->cmd.DataLength); //输出接收到的数据
      break;    
    case SAMPLEAPP_PERIODIC_CLUSTERID:
      break;

    case SAMPLEAPP_FLASH_CLUSTERID:
      flashTime = BUILD_UINT16(pkt->cmd.Data[1], pkt->cmd.Data[2] );
      HalLedBlink( HAL_LED_4, 4, 50, (flashTime / 4) );
      break;
    case SAMPLEAPP_CONTROL_CLUSTERID:
      osal_memcpy(buffer,pkt->cmd.Data,5);
      if((buffer[0]=='0'&&buffer[1]=='1'&&buffer[2]=='1'&&buffer[3]=='0'&&buffer[4]=='1')||(buffer[0]=='1'&&buffer[1]=='0'&&buffer[2]=='1'&&buffer[3]=='0'&&buffer[4]=='1')) //灯灭
      {
        HalLedBlink(HAL_LED_2,0,101,500);
        count=1;
      }
      else if((buffer[0]=='0'&&buffer[1]=='1'&&buffer[2]=='1'&&buffer[3]=='0'&&buffer[4]=='0')||(buffer[0]=='1'&&buffer[1]=='0'&&buffer[2]=='1'&&buffer[3]=='0'&&buffer[4]=='0'))
      {
        count=0;
        HalLedBlink(HAL_LED_2,0,0,500);
      }
      break;
  }
}

void SampleApp_SendPeriodicMessage( void )
{
  if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_PERIODIC_CLUSTERID,
                       1,
                       (uint8*)&SampleAppPeriodicCounter,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  }
  else
  {
  }
}

void SampleApp_SendFlashMessage( uint16 flashTime )
{
  uint8 buffer[3];
  buffer[0] = (uint8)(SampleAppFlashCounter++);
  buffer[1] = LO_UINT16( flashTime );
  buffer[2] = HI_UINT16( flashTime );

  if ( AF_DataRequest( &SampleApp_Flash_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_FLASH_CLUSTERID,
                       3,
                       buffer,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  }
  else
  {
  }
}

void SampleApp_Send_P2P_Message( void )
{
  if (count==1){
  byte str[11];
  char strTemp[10];
  byte temp;
  byte save[5];
  char j=0;
  char k=0;
  char l=0;
  temp = ReadDs18B20();//读取温度数据
  str[0]='g';
  str[1] = temp/10+48;
  str[2] = temp%10+48;
   
       
  int math;
  math=str[1]-'0';
  math=math*10;
  math=math+str[2]-'0';
   if(PWM!=state1)
  {
    pluse++;
  }
  l=pluse/100;
  k=pluse%100;
  k=k/10;
  j=pluse%10;
  state1=PWM;
  save[0]=l+0x30;
  save[1]=k+0x30;
  save[2]=j+0x30;      
  osal_memcpy(strTemp, "TEMP:", 5);
  osal_memcpy(&strTemp[5], &str[1], 2);
  osal_memcpy(&strTemp[7], " C", 2);
  osal_memcpy(&strTemp[9], "\0", 1);
  HalLcdWriteString(strTemp, HAL_LCD_LINE_3); //LCD显示
 osal_memcpy(&str[3],&save, 3);
  str[6]='0';
  str[7]='0';
  str[8]='0';
  if(math>=30||math<=8)
  {
     HalLedBlink(HAL_LED_2,0,0,500);

  }
  else
  {
    AF_DataRequest( &SampleApp_P2P_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_P2P_CLUSTERID,
                       9,
                       str,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS );
  }
  }
  else
  {
  }
}

