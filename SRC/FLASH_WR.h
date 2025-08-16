#ifndef __FLASH_WR_H
#define __FLASH_WR_H


#define FINAL_PAGE_ADDRESS 0x0803F800
/**************************************************************************
�������ܣ���ָ����ַд������
��ڲ�����addr 	д���FLASHҳ���׵�ַ
                    p	  	��д������ĵ�ַ�������еı�����uint8_t���ͣ�Ԫ�ظ���������ż����
                    Count_To_Write ��д������ĵ�ַ��
�� �� ֵ���ɹ�д�����
**************************************************************************/
int MyFLASH_WriteWord(uint32_t faddr, uint16_t* p, uint16_t Count_To_Write);
int MyFLASH_WriteWord_SIGLE(uint32_t faddr, uint16_t* p, uint16_t Count_To_Write, uint16_t add);

/**************************************************************************
�������ܣ���ָ����ַ��ȡ����
��ڲ�����addr ��FLASH�ж�ȡ�ĵ�ַ
                    p    ��ȡ��Ҫ��������ĵ�ַ�������еı�����uint8_t���ͣ�
                    Count_To_Write Ҫ�������ֽ���
�� �� ֵ����
**************************************************************************/
void MyFLASH_ReadByte(unsigned int addr, uint16_t* p, uint16_t Count_To_Read);
#endif
