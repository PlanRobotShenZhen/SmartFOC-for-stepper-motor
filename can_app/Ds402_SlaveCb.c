
#include "Ds402_SlaveCb.h"

//#include <main.h>
#include "common.h"
//#include "relay.h"
#include "can_driver.h"
extern s_BOARD SlaveBoard;
extern CO_Data Ds401_Slave_Data;
/*下面的一些回调，会在特定状态下进行触发，这里只是为了便于调试，加了一个调试日志*/
/*****************************************************************************/
UNS32 TestSlave_getMapData(UNS8 dataType, void * data)
{
    switch (dataType) {
        case  int8:     return (UNS32)(*(int8_t*)data);
        case  int16:    return (UNS32)(*(int16_t*)data);
        case  int32:    return (UNS32)(*(int32_t*)data);
        case  uint8:    return (UNS32)(*(uint8_t*)data);
        case  uint16:   return (UNS32)(*(uint16_t*)data);
        case  uint32:   return (UNS32)(*(uint32_t*)data);
        case  real32:   return (UNS32)(*(uint32_t*)data);
    };
		return 1;
}

//extern float targetSpeed;
int z=0;
//数据字典回调方法
UNS32 TestSlave_ODCallback(CO_Data* d, const indextable * idx_tbl, UNS8 bSubindex)
{
    //UNS32 errorCode;
    UNS32 cmd_data = 0;
    if (!d || !idx_tbl) {
        //SYS_DEBUG(("null pointer,please check your code."));
        return OD_READ_NOT_ALLOWED;
    }
    //SYS_DEBUG(("cob_id:%x",idx_tbl->index)); 
    if (idx_tbl->bSubCount <= bSubindex){
            return OD_VALUE_RANGE_EXCEEDED;
    }

    cmd_data = TestSlave_getMapData(idx_tbl->pSubindex[bSubindex].bDataType, idx_tbl->pSubindex[bSubindex].pObject);
    switch(idx_tbl->index) { //这个就是数据字典的索引
      case 0x6000: 
        if (idx_tbl->pSubindex[bSubindex].bAccessType & RW){ //先检查类型，在根据类型写。当然也可以通过写数据字典接口进行写数据
          //idx_tbl->pSubindex[bSubindex].pObject //这个就是数据字典存放的数据,要怎么改随便改
        }
        break;
      case 0x6042: break; //系统需要达到的转速
			 case 0x6040:
          z++;				
   			 break; //读取电机实际位置	
      case 0x6064: break; //读取电机实际位置
      case 0x607A: break; //目的位置
        
      case 0x6200: break;
        case 0x6220: //SYS_DEBUG((" set 0x6220 relay.,subindex:%d",bSubindex));
        if (cmd_data == 1){}
          //relay_ctrl(RELAY_ON);
        else if (cmd_data == 0){}
          //relay_ctrl(RELAY_OFF);
        break; //继电器
      case 0x606C:  //速度实际值
          break;
      case 0x60ff:  //目标速度
//            targetSpeed = cmd_data;  //设置目标速度
            //SYS_DEBUG((" set 0x60ff,target speed:%d",cmd_data));
            //set_pwm(24000, 100-targetSpeed);
            //OLED_ShowNum(85,18,targetSpeed,2,16,1); //更新目标速度
          break;
      case 0x6320: break;   
      case 0x6401: //SYS_DEBUG((" read 0x6401 relay."));
        break; //读温度值
      case 0x6411: break;
      default: break;
    }
		
		return 0;
		
}

//更新温度的任务
//void update_temp_task()
//{
//  //目前只有一路温度，这里读取温度，直接放到index为0
//  UNS16 temp_value = 0;
//  temp_value = temp_get_val();
//  SYS_DEBUG(("temp value:%d",temp_value));

//}
