//Si4734 i2C functions     
//Roger Traylor 11.13.2011
//device driver for the si4734 chip.

// header files
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdlib.h>
#include <util/twi.h>
#include <avr/eeprom.h>
#include <util/delay.h>

#include "twi_master.h"        //my defines for TWCR_START, STOP, RACK, RNACK, SEND
#include "si4734.h"

uint8_t si4734_wr_buf[9];          //buffer for holding data to send to the si4734 
uint8_t si4734_rd_buf[15];         //buffer for holding data recieved from the si4734
uint8_t si4734_tune_status_buf[8]; //buffer for holding tune_status data  

enum radio_band{FM, AM, SW};
extern volatile enum radio_band current_radio_band;

extern uint16_t eeprom_fm_freq;
extern uint16_t eeprom_am_freq;
extern uint16_t eeprom_sw_freq;
extern uint8_t  eeprom_volume;

extern uint16_t current_fm_freq;
extern uint16_t current_am_freq;
extern uint16_t current_sw_freq;
extern uint8_t  current_volume;

//********************************************************************************
//                            get_int_status()
//

uint8_t get_int_status(){

//send get_int_status command
    si4734_wr_buf[0] = 0x14;
    twi_start_wr(SI4734_ADDRESS, si4734_wr_buf, 1);

   _delay_ms(5);
//get the interrupt status 
    twi_start_rd(SI4734_ADDRESS, si4734_rd_buf, 1);
    return(si4734_rd_buf[0]);
}
//********************************************************************************

//********************************************************************************
//                            fm_tune_freq()
//
//takes current_fm_freq and sends it to the radio chip
//

void fm_tune_freq(){

  si4734_wr_buf[0] = 0x20;  //fm tune command
  si4734_wr_buf[1] = 0x00;  //no FREEZE and no FAST tune
  si4734_wr_buf[2] = (uint8_t)(current_fm_freq >> 8); //freq high byte
  si4734_wr_buf[3] = (uint8_t)(current_fm_freq);      //freq low byte
  si4734_wr_buf[4] = 0x00;  //antenna tuning capactior

  //send fm tune command
  twi_start_wr(SI4734_ADDRESS, si4734_wr_buf, 5);

    _delay_ms(80);
//get the interrupt status 
//    return(get_int_status());
//    return(1);
}
//********************************************************************************

//********************************************************************************
//                            am_tune_freq()
//
//takes current_am_freq and sends it to the radio chip
//

void am_tune_freq(){

  si4734_wr_buf[0] = 0x40;  //am tune command
  si4734_wr_buf[1] = 0x00;  //no FAST tune
  si4734_wr_buf[2] = (uint8_t)(current_am_freq >> 8); //freq high byte
  si4734_wr_buf[3] = (uint8_t)(current_am_freq);      //freq low byte
  si4734_wr_buf[4] = 0x00;  //antenna tuning capactior high byte
  si4734_wr_buf[5] = 0x00;  //antenna tuning capactior low byte

  //send fm tune command
  twi_start_wr(SI4734_ADDRESS, si4734_wr_buf, 6);

    _delay_ms(80);
//get the interrupt status 
//    return(get_int_status());
//    return(1);
}
//********************************************************************************

//********************************************************************************
//                            sw_tune_freq()
//
//takes current_sw_freq and sends it to the radio chip
//antcap low byte is 0x01 as per datasheet

void sw_tune_freq(){

  si4734_wr_buf[0] = 0x40;  //am tune command
  si4734_wr_buf[1] = 0x00;  //no FAST tune
  si4734_wr_buf[2] = (uint8_t)(current_sw_freq >> 8); //freq high byte
  si4734_wr_buf[3] = (uint8_t)(current_sw_freq);      //freq low byte
  si4734_wr_buf[4] = 0x00;  //antenna tuning capactior high byte
  si4734_wr_buf[5] = 0x01;  //antenna tuning capactior low byte 

  //send am tune command
  twi_start_wr(SI4734_ADDRESS, si4734_wr_buf, 6);

    _delay_ms(80);
//get the interrupt status 
//    return(get_int_status());
//    return(1);
}

//********************************************************************************
//                            fm_pwr_up()
//

void fm_pwr_up(){
//restore the previous fm frequency  
 current_fm_freq = eeprom_read_word(&eeprom_fm_freq); //TODO: only this one does not work 
 current_volume  = eeprom_read_byte(&eeprom_volume); //TODO: only this one does not work 

//send fm power up command
  si4734_wr_buf[0] = 0x01;
  si4734_wr_buf[1] = 0x50; //GPO2OEN and XOSCEN selected
  si4734_wr_buf[2] = 0x05; //analog audio outputs
  twi_start_wr(SI4734_ADDRESS, si4734_wr_buf, 3);

  _delay_ms(120);   //startup delay as specified 
  
  set_property(GPO_IEN, (1<<GPO_IEN_STCIEN_SHFT));    //enable Seek/Tune Complete interrupt

  //get the interrupt status 
//  return(get_int_status());
//  return(1);
}
//********************************************************************************

//********************************************************************************
//                            am_pwr_up()
//

void am_pwr_up(){
//restore the previous am frequency  
  current_am_freq = eeprom_read_word(&eeprom_am_freq);
  current_volume  = eeprom_read_byte(&eeprom_volume); //TODO: only this one does not work 

//send am power up command
  si4734_wr_buf[0] = 0x01;
  si4734_wr_buf[1] = 0x51;//GPO2OEN and XOSCEN selected
  si4734_wr_buf[2] = 0x05;
  twi_start_wr(SI4734_ADDRESS, si4734_wr_buf, 3);

  _delay_ms(120);   

  set_property(GPO_IEN, (1<<GPO_IEN_STCIEN_SHFT));    //enable Seek/Tune Complete interrupt
  //get the interrupt status 
//  return(get_int_status());
}
//********************************************************************************

//********************************************************************************
//                            sw_pwr_up()
//

void sw_pwr_up(){
//restore the previous sw frequency  
  current_sw_freq = eeprom_read_word(&eeprom_sw_freq);
  current_volume  = eeprom_read_byte(&eeprom_volume); //TODO: only this one does not work 

//send sw power up command (same as am, only tuning rate is different)
    si4734_wr_buf[0] = 0x01;
    si4734_wr_buf[1] = 0x51;
    si4734_wr_buf[2] = 0x05;
    twi_start_wr(SI4734_ADDRESS, si4734_wr_buf, 3);

//get the interrupt status 
  _delay_ms(120);   
//  return(get_int_status());    
//  get_int_status();    

//set property to disable soft muting for shortwave broadcasts
  set_property(AM_SOFT_MUTE_MAX_ATTENUATION, 0x0000); //cut off soft mute  
//select 4khz filter BW and engage power line filter
  set_property(AM_CHANNEL_FILTER, (AM_CHFILT_4KHZ | AM_PWR_LINE_NOISE_REJT_FILTER)); 

//  si4734_wr_buf[0] = 0x12;  //set properity command     
//  si4734_wr_buf[1] = 0x00;  //
//  si4734_wr_buf[2] = 0x33;  //
//  si4734_wr_buf[3] = 0x02;  // am_soft_mute_max_attenuation
//  si4734_wr_buf[4] = 0x00;  //
//  si4734_wr_buf[5] = 0x00;  // disable soft muting
//  twi_start_wr(SI4734_ADDRESS, si4734_wr_buf, 6);

//enable Seek/Tune Complete interrupt
  set_property(GPO_IEN, (1<<GPO_IEN_STCIEN_SHFT));    

}
//********************************************************************************

//********************************************************************************
//                            radio_pwr_dwn()
//

void radio_pwr_dwn(){

//save current frequency to EEPROM
switch(current_radio_band){
  case(FM) : eeprom_write_word(&eeprom_fm_freq, current_fm_freq); break;
  case(AM) : eeprom_write_word(&eeprom_am_freq, current_am_freq); break;
  case(SW) : eeprom_write_word(&eeprom_sw_freq, current_sw_freq); break;
  default  : break;
}//switch      

  eeprom_write_byte(&eeprom_volume, current_volume); //save current volume level

//send fm power down command
    si4734_wr_buf[0] = 0x11;
    twi_start_wr(SI4734_ADDRESS, si4734_wr_buf, 1);

  _delay_us(310);
//get the interrupt status 
//  return(get_int_status());
}
//********************************************************************************

//********************************************************************************
//                            fm_rsq_status()
//

void fm_rsq_status(){

    si4734_wr_buf[0] = FM_RSQ_STATUS;            //fm_rsq_status command
    si4734_wr_buf[1] = FM_RSQ_STATUS_IN_INTACK;  //clear STCINT bit if set
    twi_start_wr(SI4734_ADDRESS, si4734_wr_buf, 2);
    _delay_ms(5);

//get the fm tune status 
    twi_start_rd(SI4734_ADDRESS, si4734_tune_status_buf, 8);
}


//********************************************************************************
//                            fm_tune_status()
//

void fm_tune_status(){

    si4734_wr_buf[0] = FM_TUNE_STATUS;            //fm_tune_status command
    si4734_wr_buf[1] = FM_TUNE_STATUS_IN_INTACK;  //clear STCINT bit if set
    twi_start_wr(SI4734_ADDRESS, si4734_wr_buf, 2);
    _delay_ms(5);

//get the fm tune status 
    twi_start_rd(SI4734_ADDRESS, si4734_tune_status_buf, 8);
}

//********************************************************************************
//                            am_tune_status()
//
//TODO: could probably just have one tune_status() function

void am_tune_status(){

    si4734_wr_buf[0] = AM_TUNE_STATUS;            //fm_tune_status command
    si4734_wr_buf[1] = AM_TUNE_STATUS_IN_INTACK;  //clear STCINT bit if set
    twi_start_wr(SI4734_ADDRESS, si4734_wr_buf, 2);
    _delay_ms(5);

//get the am tune status 
    twi_start_rd(SI4734_ADDRESS, si4734_tune_status_buf, 8);

}
//********************************************************************************
//                            am_rsq_status()
//

void am_rsq_status(){

    si4734_wr_buf[0] = AM_RSQ_STATUS;            //am_rsq_status command
    si4734_wr_buf[1] = AM_RSQ_STATUS_IN_INTACK;  //clear STCINT bit if set
    twi_start_wr(SI4734_ADDRESS, si4734_wr_buf, 2);
    _delay_ms(5);

//get the fm tune status 
    twi_start_rd(SI4734_ADDRESS, si4734_tune_status_buf, 8);
}

//********************************************************************************
//                            set_property()
//

void set_property(uint16_t property, uint16_t property_value){

    si4734_wr_buf[0] = SET_PROPERTY;                   //set property command
    si4734_wr_buf[1] = 0x00;                           //all zeros
    si4734_wr_buf[2] = (uint8_t)(property >> 8);       //property high byte
    si4734_wr_buf[3] = (uint8_t)(property);            //property low byte
    si4734_wr_buf[4] = (uint8_t)(property_value >> 8); //property value high byte
    si4734_wr_buf[5] = (uint8_t)(property_value);      //property value low byte
    twi_start_wr(SI4734_ADDRESS, si4734_wr_buf, 6);
    _delay_ms(10);  //set properties takes 10ms to complete
}//set_property()

