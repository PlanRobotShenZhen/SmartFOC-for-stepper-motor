#include "n32g45x_flash.h"
#include "FLASH_WR.h"
/**************************************************************************
�������ܣ���ָ����ַд������
��ڲ�����addr 	д���FLASHҳ���׵�ַ
          p	  	��д������ĵ�ַ�������еı�����uint8_t���ͣ�Ԫ�ظ���������ż����
          Count_To_Write ��д������ĵ�ַ��
�� �� ֵ���ɹ�д�����
**************************************************************************/
int MyFLASH_WriteWord(uint32_t faddr, uint16_t* p, uint16_t Count_To_Write)
{
    
    //д������
    uint32_t d;
    uint16_t dataIndex;
    //�����жϷ�ֹFlash���������
    __disable_irq();
    FLASH_Unlock(); //Flash��������
    FLASH_ClearFlag(FLASH_FLAG_BUSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);//�������Flash������־λ
    FLASH_EraseOnePage(faddr);  //����Ŀ��������������������������

    
    for(dataIndex = 0; dataIndex < Count_To_Write; dataIndex++)// ѭ��д����������
    {
        d = p[dataIndex];//�������16λ����Ϊһ��32λ��
        dataIndex++;
        d |= p[dataIndex] << 16;
        FLASH_ProgramWord(faddr, d);//��32λ����д��Flash
        faddr += 4;//��ַ������һ��32λλ��
    }

    FLASH_Lock();//Flash��������
    __enable_irq();//�ָ��ж�
    return dataIndex;//��������
}

/**************************************************************************
�������ܣ���ָ����ַд��SIGLE  
��ڲ�����addr 	д���FLASHҳ���׵�ַ
          p	  	��д������ĵ�ַ�������еı�����uint8_t���ͣ�Ԫ�ظ���������ż����
          Count_To_Write ��д������ĵ�ַ��
�� �� ֵ���ɹ�д�����
**************************************************************************/

int MyFLASH_WriteWord_SIGLE(uint32_t faddr, uint16_t* p, uint16_t Count_To_Write, uint16_t add)
{
    //д������
    uint32_t d;
    //uint16_t dataIndex;
    __disable_irq();
    
    FLASH_Unlock();// Flash����
    FLASH_ClearFlag(FLASH_FLAG_BUSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);//���Flash��־λ
    //  FLASH_EraseOnePage(faddr);//ȫ����

    d = p[add * 2];
    //dataIndex++;
    //d |= p[add] << 16;
    FLASH_ProgramWord((uint32_t)((uint16_t*)faddr + add), d); //д��Flash
    //faddr += 4;
    FLASH_Lock();
    __enable_irq();
    return 1;
}

/**************************************************************************
�������ܣ���ָ����ַ��ȡ����
��ڲ�����addr ��FLASH�ж�ȡ�ĵ�ַ
                    p    ��ȡ��Ҫ��������ĵ�ַ�������еı�����uint8_t���ͣ�
                    Count_To_Write Ҫ�������ֽ���
�� �� ֵ����
**************************************************************************/
void MyFLASH_ReadByte(unsigned int addr, uint16_t* p, uint16_t Count_To_Read)
{
    //memcpy(p, (uint16_t*)addr, Count_To_Read);
    uint32_t d;
    uint32_t id = 0;
    int n = Count_To_Read >> 1;
    //ѭ����ȡÿ��32λ��
    for(uint16_t i = 0; i < n; i++)
    {
        d = *((__IO uint32_t*)(addr));//�ֽ�32λ����Ϊ����16λ����
        p[id++] = d;//�ֽ�32λ����Ϊ����16λ����
        p[id++] = d >> 16;
        addr += 4; //��ַ��������һ��32λλ��
    }
}
