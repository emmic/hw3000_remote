
/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "hw3000.h"
#include "delay.h"
#include "gpio.h"
#include "control.h"

#define Sender 1
#define Receiver 0

#define LED_Red_on()     GPIO_WriteLow(LED1_PORT, LED1_IO)
#define LED_Red_off()    GPIO_WriteHigh(LED1_PORT, LED1_IO)
#define LED_Red_toggle() GPIO_WriteReverse(LED1_PORT, LED1_IO)
#define LED_GREEN_on()   GPIO_WriteLow(LED2_PORT, LED2_IO)
#define LED_GREEN_off()  GPIO_WriteHigh(LED2_PORT, LED2_IO)

#define Buf_MAX_LEN 4
#define Botton_MAX  6
#define Botton_Flash_addr  0x4010
#define Longpress_time   2500
#define Send_join_maxtime  7500
#define Sleep_time       2500
    
uint16_t time_2ms=0; 
uint8_t Self_UID[4];
uint8_t FIFO_buf[6];
uint8_t Rec_ID[Botton_MAX][Buf_MAX_LEN];
uint8_t join_key=0;
volatile uint8_t LED_display_state=0;

void Scan_key(void);
void broadcast_rcvd(uint8_t *d, uint8_t l);
void TIM4_Config(void);
void CopyFlash_ID(uint16_t ADDR);
void Flash_Write(uint16_t addr, uint8_t *buf, uint8_t len);
void Send_join(uint8_t key_press);
void Send_command(uint8_t key_num);
void Save_ID(void);
void Check_clear(uint8_t key_num);
void LED_Display(uint8_t STATE);
void GPIO_LOWPOWER(void);
void EXIT_halt(void);
 

void main(void)
{
  
  //CLK_HSICmd(ENABLE);
  //CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV8);
  MCU_gpio_init();
  CopyFlash_ID(Botton_Flash_addr);
  TIM4_Config();
  hw3000_init();
  hw3000_setRcvdCallback(broadcast_rcvd);
  enableInterrupts(); //开MCU全局中断 

  while(1){
    Scan_key();
    LED_Display(LED_display_state);
    
    if(time_2ms > 1000){
    GPIO_LOWPOWER();
    halt();
    EXIT_halt();
    } 
  }
}

void Scan_key(void)
{
  if((GPIO_ReadInputData(KEY1_PORT)&KEY1_IO) == 0){
    time_2ms = 0;
    HAL_Delay_nMS(2);
    if((GPIO_ReadInputData(KEY1_PORT)&KEY1_IO) == 0){
      LED_Red_on();
      while(((GPIO_ReadInputData(KEY1_PORT)&KEY1_IO) == 0) && (time_2ms < Longpress_time));
      LED_Red_off();
      if(time_2ms >= Longpress_time){
        Send_join(1);
        if((GPIO_ReadInputData(KEY1_PORT)&KEY1_IO) == 0){
          Check_clear(1);
        }    
      }
      else{
        Send_command(1);
      }
    }
    time_2ms = 0;
  }

  if((GPIO_ReadInputData(KEY2_PORT)&KEY2_IO) == 0){
      time_2ms = 0;
      HAL_Delay_nMS(2);
      if((GPIO_ReadInputData(KEY2_PORT)&KEY2_IO) == 0){
        LED_Red_on();
        while(((GPIO_ReadInputData(KEY2_PORT)&KEY2_IO) == 0) && (time_2ms < Longpress_time));
        LED_Red_off();
        if(time_2ms >= Longpress_time){
          Send_join(2);
          if((GPIO_ReadInputData(KEY2_PORT)&KEY2_IO) == 0){
            Check_clear(2);
          }    
        }
        else{
          Send_command(2);
        }
      }
      time_2ms = 0;
    }

  if((GPIO_ReadInputData(KEY3_PORT)&KEY3_IO) == 0){
      time_2ms = 0;
      HAL_Delay_nMS(2);
      if((GPIO_ReadInputData(KEY3_PORT)&KEY3_IO) == 0){
        LED_Red_on();
        while(((GPIO_ReadInputData(KEY3_PORT)&KEY3_IO) == 0) && (time_2ms < Longpress_time));
        LED_Red_off();
        if(time_2ms >= Longpress_time){
          Send_join(3);
          if((GPIO_ReadInputData(KEY1_PORT)&KEY1_IO) == 0){
            Check_clear(3);
          }    
        }
        else{
          Send_command(3);
        }
      }
      time_2ms = 0;
    }

  if((GPIO_ReadInputData(KEY4_PORT)&KEY4_IO) == 0){
      time_2ms = 0;
      HAL_Delay_nMS(2);
      if((GPIO_ReadInputData(KEY4_PORT)&KEY4_IO) == 0){
        LED_Red_on();
        while(((GPIO_ReadInputData(KEY4_PORT)&KEY4_IO) == 0) && (time_2ms < Longpress_time));
        LED_Red_off();
        if(time_2ms >= Longpress_time){
          Send_join(4);
          if((GPIO_ReadInputData(KEY1_PORT)&KEY1_IO) == 0){
            Check_clear(4);
          }    
        }
        else{
          Send_command(4);
        }
      }
      time_2ms = 0;
    }

  if((GPIO_ReadInputData(KEY5_PORT)&KEY5_IO) == 0){
      time_2ms = 0;
      HAL_Delay_nMS(2);
      if((GPIO_ReadInputData(KEY5_PORT)&KEY5_IO) == 0){
        LED_Red_on();
        while(((GPIO_ReadInputData(KEY5_PORT)&KEY5_IO) == 0) && (time_2ms < Longpress_time));
        LED_Red_off();
        if(time_2ms >= Longpress_time){
          Send_join(5);
          if((GPIO_ReadInputData(KEY1_PORT)&KEY1_IO) == 0){
            Check_clear(5);
          }    
        }
        else{
          Send_command(5);
        }
      }
      time_2ms = 0;
    }

  if((GPIO_ReadInputData(KEY6_PORT)&KEY6_IO) == 0){
      time_2ms = 0;
      HAL_Delay_nMS(2);
      if((GPIO_ReadInputData(KEY6_PORT)&KEY6_IO) == 0){
        LED_Red_on();
        while(((GPIO_ReadInputData(KEY6_PORT)&KEY6_IO) == 0) && (time_2ms < Longpress_time));
        LED_Red_off();
        if(time_2ms >= Longpress_time){
          Send_join(6);
          if((GPIO_ReadInputData(KEY1_PORT)&KEY1_IO) == 0){
            Check_clear(6);
          }    
        }
        else{
          Send_command(6);
        }
      }
      time_2ms = 0;
    }

  if((GPIO_ReadInputData(KEY7_PORT)&KEY7_IO) == 0){
    HAL_Delay_nMS(2);
    if((GPIO_ReadInputData(KEY7_PORT)&KEY7_IO) == 0){
      LED_Red_on();
      while((GPIO_ReadInputData(KEY7_PORT)&KEY7_IO) == 0);
      LED_Red_off();
      Send_command(7);
    }
    time_2ms = 0;
  }
  
  if((GPIO_ReadInputData(KEY8_PORT)&KEY8_IO) == 0){
    HAL_Delay_nMS(2);
    if((GPIO_ReadInputData(KEY8_PORT)&KEY8_IO) == 0){
      LED_Red_on();
      while((GPIO_ReadInputData(KEY8_PORT)&KEY8_IO) == 0);
      LED_Red_off();
      Send_command(8);
    }
    time_2ms = 0;
  }

}

void GPIO_LOWPOWER(void){
//  GPIO_Init(GPIOA,GPIO_PIN_ALL,GPIO_MODE_OUT_PP_LOW_SLOW);
//  GPIO_Init(GPIOB,GPIO_PIN_ALL,GPIO_MODE_OUT_PP_LOW_SLOW);
//  GPIO_Init(GPIOC,GPIO_PIN_ALL,GPIO_MODE_OUT_PP_LOW_SLOW);
//  GPIO_Init(GPIOD,GPIO_PIN_ALL,GPIO_MODE_OUT_PP_LOW_SLOW);
    
  GPIO_Init(HW3000_CSN_PORT, HW3000_CSN_IO,GPIO_MODE_OUT_PP_LOW_SLOW);
  GPIO_Init(HW3000_CLK_PORT, HW3000_CLK_IO,GPIO_MODE_OUT_PP_LOW_SLOW);
  GPIO_Init(HW3000_SDO_PORT, HW3000_SDO_IO,GPIO_MODE_OUT_PP_LOW_SLOW);
  GPIO_Init(HW3000_SDI_PORT, HW3000_SDI_IO,GPIO_MODE_OUT_PP_LOW_SLOW);
  GPIO_Init(HW3000_IRQ_PORT, HW3000_IRQ_IO,GPIO_MODE_OUT_PP_LOW_SLOW);
  GPIO_Init(HW3000_PDN_PORT, HW3000_PDN_IO,GPIO_MODE_OUT_PP_HIGH_FAST);  //RF ON/OFF
  

  
  //EXTI_SetTLISensitivity(EXTI_TLISENSITIVITY_FALL_ONLY);   
  //EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOC,EXTI_SENSITIVITY_FALL_ONLY);
  disableInterrupts();
  GPIO_Init(KEY1_PORT, KEY1_IO,GPIO_MODE_IN_PU_IT);
  GPIO_Init(KEY2_PORT, KEY2_IO,GPIO_MODE_IN_PU_IT);
  GPIO_Init(KEY3_PORT, KEY3_IO,GPIO_MODE_IN_PU_IT);
  GPIO_Init(KEY4_PORT, KEY4_IO,GPIO_MODE_IN_PU_IT);
  
  CLK_HSECmd(DISABLE);
  GPIO_Init(KEY5_PORT, KEY5_IO,GPIO_MODE_IN_FL_IT);//0.67MA
  
  GPIO_Init(KEY6_PORT, KEY6_IO,GPIO_MODE_IN_PU_IT);
  GPIO_Init(KEY7_PORT, KEY7_IO,GPIO_MODE_IN_PU_IT);
  GPIO_Init(KEY8_PORT, KEY8_IO,GPIO_MODE_IN_PU_IT);   
  
  GPIO_Init(LED1_PORT, LED1_IO,GPIO_MODE_OUT_PP_HIGH_FAST);
  GPIO_Init(LED2_PORT, LED2_IO,GPIO_MODE_OUT_PP_HIGH_FAST);
  
  EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOA,EXTI_SENSITIVITY_FALL_ONLY);
  EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOB,EXTI_SENSITIVITY_FALL_ONLY);
  EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOD,EXTI_SENSITIVITY_FALL_ONLY);
  enableInterrupts();
}

void EXIT_halt(void){
  disableInterrupts();
  TIM4_Config();
  hw3000_init();
  hw3000_setRcvdCallback(broadcast_rcvd);
  enableInterrupts();
}

void Check_clear(uint8_t key_num)
{
  for(uint8_t i=0; i<20; i++){
    LED_Red_toggle();
    HAL_Delay_nMS(100);
  }
  uint8_t clear_tab[Buf_MAX_LEN]={0,0,0,0};
  Flash_Write(Botton_Flash_addr+((key_num-1)*Buf_MAX_LEN), clear_tab, Buf_MAX_LEN); 
  
  for(uint8_t i=0; i<4; i++){
    Rec_ID[key_num-1][i] = 0;
  }
}

void Save_ID(void)
{
  if(join_key != 0){
  Flash_Write(Botton_Flash_addr+((join_key-1)*Buf_MAX_LEN),&Rec_ID[join_key-1][0],Buf_MAX_LEN);
  join_key = 0;
  LED_display_state = SEND_ID;
  }
}

void Send_join(uint8_t key_press)
{
  join_key = key_press;
  for(uint8_t i=0; i<4; i++){
        FIFO_buf[i] = 0x00;
      }
  FIFO_buf[4] = SEND_JOIN;
  FIFO_buf[5] = 0x16;
  time_2ms = 0;
  while((join_key != 0) && (time_2ms < Send_join_maxtime)){
    HAL_Delay_nMS(250);
    LED_Red_on();
    LED_GREEN_off();
    HAL_Delay_nMS(250);
    LED_Red_off();
    LED_GREEN_on();
    HAL_Delay_nMS(250);
    LED_Red_on();
    LED_GREEN_off();
    HAL_Delay_nMS(250);
    LED_Red_off();
    LED_GREEN_on();
    HAL_Delay_nMS(250);
    LED_Red_on();
    LED_GREEN_off();
    HAL_Delay_nMS(250);
    LED_Red_off();
    LED_GREEN_on();
    HAL_Delay_nMS(250);
    LED_Red_on();
    LED_GREEN_off();
    HAL_Delay_nMS(250);
    LED_Red_off();
    LED_GREEN_on();
    hw3000_send(FIFO_buf, 6);
  }
  LED_Red_off();
  LED_GREEN_off();
  if(join_key != 0)
    join_key = 0;
}

void Send_command(uint8_t key_num)
{
  join_key = key_num;
  switch(key_num){
    case 1: FIFO_buf[4] = SEND_TOGGLE; break;
    case 2: FIFO_buf[4] = SEND_TOGGLE; break;
    case 3: FIFO_buf[4] = SEND_TOGGLE; break;
    case 4: FIFO_buf[4] = SEND_TOGGLE; break;
    case 5: FIFO_buf[4] = SEND_TOGGLE; break;
    case 6: FIFO_buf[4] = SEND_TOGGLE; break;
    case 7: FIFO_buf[4] = SEND_ALL_OPEN; break;
    case 8: FIFO_buf[4] = SEND_ALL_CLOSE; break;
  }
  if(key_num < 7){
    for(uint8_t i=0; i<4; i++){
        FIFO_buf[i] = Rec_ID[key_num-1][i];
      }
    FIFO_buf[5] = 0x16;
    hw3000_send(FIFO_buf, sizeof(FIFO_buf));
  }
  else{
    LED_Red_on();
    for(uint8_t i=0; i<6; i++){
      for(uint8_t j=0; j<4; j++){
        FIFO_buf[j] = Rec_ID[i][j];
      }
      FIFO_buf[5] = 0x16;
      hw3000_send(FIFO_buf, sizeof(FIFO_buf));
      HAL_Delay_nMS(1500);
    }
    LED_Red_off();
  }
  
}

void CopyFlash_ID(uint16_t ADDR)
{
  for(uint8_t i=0; i<Botton_MAX; i++){
    for(uint8_t j=0; j<Buf_MAX_LEN; j++){
      Rec_ID[i][j] = *(uint8_t*)(ADDR+(i*Buf_MAX_LEN)+j);
    }
  }
}

void Flash_Write(uint16_t addr, uint8_t *buf, uint8_t len)
{
  FLASH_SetProgrammingTime((FLASH_ProgramTime_TypeDef)FLASH_PROGRAMTIME_STANDARD);
  FLASH_Unlock(FLASH_MEMTYPE_DATA);
  for(uint8_t i=0; i<len; i++)
  {
    FLASH_ProgramByte(addr+i, buf[i]);
    //FLASH_EraseByte(add);
    FLASH_WaitForLastOperation(FLASH_MEMTYPE_DATA);
  }
  FLASH_Lock(FLASH_MEMTYPE_DATA);
}

void TIM4_Config(void)
{
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER4, ENABLE);
  TIM4_DeInit();             
  TIM4_TimeBaseInit(TIM4_PRESCALER_16, 0xFA);       //x=2M/16    interrupt=0xfa/x   2ms
  TIM4_UpdateRequestConfig(TIM4_UPDATESOURCE_GLOBAL);
  TIM4_ClearFlag(TIM4_FLAG_UPDATE);
  TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);
  TIM4_Cmd(ENABLE);
}

void LED_Display(uint8_t STATE){
  if(STATE != 0){
    if(STATE == SEND_ID){
    LED_GREEN_on();
    HAL_Delay_nMS(4000);
    LED_GREEN_off();
    }
    else if(STATE == SEND_ACK_OPEN){
      HAL_Delay_nMS(200);
      LED_Red_on();
      HAL_Delay_nMS(200);
      LED_Red_off();
      HAL_Delay_nMS(200);
      LED_Red_on();
      HAL_Delay_nMS(200);
      LED_Red_off();
    }
    else if(STATE == SEND_ACK_CLOSE){
      HAL_Delay_nMS(200);
      LED_GREEN_on();
      HAL_Delay_nMS(200);
      LED_GREEN_off();
      HAL_Delay_nMS(200);
      LED_GREEN_on();
      HAL_Delay_nMS(200);
      LED_GREEN_off();
    }
    LED_display_state = 0;
    time_2ms = 0;
  }
}


void broadcast_rcvd(uint8_t *d, uint8_t l){
  static uint8_t *radio_data;
  static uint8_t radio_len;
  radio_data = d;
  radio_len = l;
  //GPIO_WriteReverse(LED_PORT, LED_IO);
  if(radio_data[radio_len-1] == 0x16){ 
       
    if(radio_data[radio_len-2] == SEND_ID){
      for(uint8_t i=0; i<Buf_MAX_LEN; i++){
        Rec_ID[join_key-1][i] = radio_data[i];
      }
      Save_ID();
    }

    if(radio_data[0] == Rec_ID[join_key-1][0] && radio_data[1] == Rec_ID[join_key-1][1] &&
      radio_data[2] == Rec_ID[join_key-1][2] && radio_data[3] == Rec_ID[join_key-1][3]){
        join_key = 0;
        if(radio_data[radio_len-2] == SEND_ACK_OPEN){
          LED_display_state = SEND_ACK_OPEN;
        }
        if(radio_data[radio_len-2] == SEND_ACK_CLOSE){
          LED_display_state = SEND_ACK_CLOSE;
        }           
    }
    
  }

}


