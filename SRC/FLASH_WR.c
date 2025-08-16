#include "n32g45x_flash.h"
#include "FLASH_WR.h"
/**************************************************************************
函数功能：向指定地址写入数据
入口参数：addr 	写入的FLASH页的首地址
          p	  	被写入变量的地址（数组中的必须是uint8_t类型，元素个数必须是偶数）
          Count_To_Write 被写入变量的地址数
返 回 值：成功写入个数
**************************************************************************/
int MyFLASH_WriteWord(uint32_t faddr, uint16_t* p, uint16_t Count_To_Write)
{
    
    //写入数据
    uint32_t d;
    uint16_t dataIndex;
    //禁用中断防止Flash操作被打断
    __disable_irq();
    FLASH_Unlock(); //Flash操作解锁
    FLASH_ClearFlag(FLASH_FLAG_BUSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);//清除所有Flash操作标志位
    FLASH_EraseOnePage(faddr);  //擦除目标扇区（整个扇区将被擦除）

    
    for(dataIndex = 0; dataIndex < Count_To_Write; dataIndex++)// 循环写入所有数据
    {
        d = p[dataIndex];//组合两个16位数据为一个32位字
        dataIndex++;
        d |= p[dataIndex] << 16;
        FLASH_ProgramWord(faddr, d);//将32位数据写入Flash
        faddr += 4;//地址传到下一个32位位置
    }

    FLASH_Lock();//Flash操作加锁
    __enable_irq();//恢复中断
    return dataIndex;//返回数据
}

/**************************************************************************
函数功能：向指定地址写入SIGLE  
入口参数：addr 	写入的FLASH页的首地址
          p	  	被写入变量的地址（数组中的必须是uint8_t类型，元素个数必须是偶数）
          Count_To_Write 被写入变量的地址数
返 回 值：成功写入个数
**************************************************************************/

int MyFLASH_WriteWord_SIGLE(uint32_t faddr, uint16_t* p, uint16_t Count_To_Write, uint16_t add)
{
    //写入数据
    uint32_t d;
    //uint16_t dataIndex;
    __disable_irq();
    
    FLASH_Unlock();// Flash解锁
    FLASH_ClearFlag(FLASH_FLAG_BUSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);//清除Flash标志位
    //  FLASH_EraseOnePage(faddr);//全擦除

    d = p[add * 2];
    //dataIndex++;
    //d |= p[add] << 16;
    FLASH_ProgramWord((uint32_t)((uint16_t*)faddr + add), d); //写入Flash
    //faddr += 4;
    FLASH_Lock();
    __enable_irq();
    return 1;
}

/**************************************************************************
函数功能：从指定地址读取数据
入口参数：addr 从FLASH中读取的地址
                    p    读取后要存入变量的地址（数组中的必须是uint8_t类型）
                    Count_To_Write 要读出的字节数
返 回 值：无
**************************************************************************/
void MyFLASH_ReadByte(unsigned int addr, uint16_t* p, uint16_t Count_To_Read)
{
    //memcpy(p, (uint16_t*)addr, Count_To_Read);
    uint32_t d;
    uint32_t id = 0;
    int n = Count_To_Read >> 1;
    //循环读取每个32位字
    for(uint16_t i = 0; i < n; i++)
    {
        d = *((__IO uint32_t*)(addr));//分解32位数据为两个16位部分
        p[id++] = d;//分解32位数据为两个16位部分
        p[id++] = d >> 16;
        addr += 4; //地址递增到下一个32位位置
    }
}
