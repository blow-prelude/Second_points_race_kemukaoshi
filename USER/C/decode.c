#include "usart.h"
#include "decode.h"

int flag=0;
int value=0;

int get_value(){

	flag = USART1_RX_STA ;		
	
	//������ܵ�������ֻ��1λ��ʼ����
	if(flag==1){
		//��ս���״̬
		USART1_RX_STA = 0;
		value = str_num-48;
	}
	//�쳣����,�ں������ú�Ҫ�жϷ���ֵ�Ƿ���INVADE
	else{
		return INVADE;
			
	}
	return value;
}