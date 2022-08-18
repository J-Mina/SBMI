#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include "serial_printf.h"
#include "lcd1602.h"
#include "i2c.h"
#include "rtc3231.h"


#define TSHOW 100
#define PASS 1000
#define T_ALARME 300000

#define BAUD 9300
#define BAUDGEN 103

#define S1_PIN 2
#define S2_PIN 3
#define S3_PIN 4
#define S4_PIN 5
#define S5_PIN 6
#define S6_PIN 7

#define C1_PIN 0
#define C2_PIN 1
#define C3_PIN 2
#define C4_PIN 3

#define L_PIN 4
#define S_PIN 5

#define B_PIN 1 
#define L_OK 0
#define S_UP 2

void set_Buzzer (int on)
{if(on) PORTC|=(1<<B_PIN);
else PORTC &= ~(1<<B_PIN);}

void set_LEDok (int on)
{if(on) PORTC|=(1<<L_OK);
else PORTC &= ~(1<<L_OK);}

void set_UP (int on)
{if(on) PORTC|=(1<<S_UP);
else PORTC &= ~(1<<S_UP);}


uint8_t read_BUTTON(int but)
{
  if(but == 1) 
  return !(PIND & (1<<S1_PIN));
  else if(but == 2) 
  return !(PIND & (1<<S2_PIN));
  else if(but == 3) 
  return !(PIND & (1<<S3_PIN));
  else if(but == 4) 
  return !(PIND & (1<<S4_PIN));
  else if(but == 5) 
  return !(PIND & (1<<S5_PIN));
  else if(but == 6) 
  return !(PIND & (1<<S6_PIN));
  else return 0;
}

uint8_t read_colunas(int col)
{
  if(col==1)
  return !(PINB & (1<<C1_PIN));
  if(col==2)
  return !(PINB & (1<<C2_PIN));
  if(col==3)
  return !(PINB & (1<<C3_PIN));
  if(col==4)
  return !(PINB & (1<<C4_PIN));
  else return 0;
}

void re_cp(void)
{
PORTB &= ~(1<<S_PIN);
PORTB|=(1<<S_PIN);
}


void set_J_K(int high)
{
  if(high) {
  PORTB|=(1<<L_PIN);
  re_cp();
  }

  else {
  PORTB &= ~(1<<L_PIN);
  re_cp();
  }
}


char Keypad(void)
{
  set_J_K(0);set_J_K(1);set_J_K(1);set_J_K(1);
  if(read_colunas(1))
  {return '1';}
  else if(read_colunas(2))
  {return '2';}
  else if(read_colunas(3))
  {return '3';}
  else if(read_colunas(4))
  {return 'A';}

  set_J_K(1);set_J_K(0);set_J_K(1);set_J_K(1);
  if(read_colunas(1))
  {return '4';}
  else if(read_colunas(2))
  {return '5';}
  else if(read_colunas(3))
  {return '6';}
  else if(read_colunas(4))
  {return 'B';}

  set_J_K(1);set_J_K(1);set_J_K(0);set_J_K(1);
  if(read_colunas(1))
  {return '7';}
  else if(read_colunas(2))
  {return '8';}
  else if(read_colunas(3))
  {return '9';}
  else if(read_colunas(4))
  {return 'C';}

  set_J_K(1);set_J_K(1);set_J_K(1);set_J_K(0);
  if(read_colunas(1))
  {return '*';}
  else if(read_colunas(2))
  {return '0';}
  else if(read_colunas(3))
  {return '#';}
  else if(read_colunas(4))
  {return 'D';}

  else return 'O'; 
}

#define T1BOTTOM 65535-250 // 1ms cycles

volatile uint16_t Tshow=0;
volatile uint16_t validate=0;
volatile uint16_t buzzer=0;


ISR(TIMER1_OVF_vect){
  TCNT1=T1BOTTOM;
  if(Tshow) Tshow--;
  if(validate) validate--;
  if(buzzer) buzzer--;
}

void tc1_init(void)
{
  cli();
  TCCR1B = 0;
  TIFR1 = (7<<TOV1) | (1<<ICF1);
  TCCR1A = 0;
  TCNT1 = T1BOTTOM;
  TIMSK1 = (1<<TOIE1);
  TCCR1B = 3; 
  sei();
}

uint16_t get_timer_time(volatile uint16_t t, uint16_t len)
{ uint16_t hey;
  cli();
  hey = len-t;
  sei();
  return hey;
}

void start_timer(volatile uint16_t *t, uint16_t f)
{ cli();
  *t=f;
  sei();
  return;
}

void io_init(void)
{
  DDRD &= ~(1 << S1_PIN); // S1 is an input 
  DDRD &= ~(1 << S2_PIN); // S2 is an input 
  DDRD &= ~(1 << S3_PIN); // S3 is an input 
  DDRD &= ~(1 << S4_PIN); // S4 is an input 
  DDRD &= ~(1 << S5_PIN); // S5 is an input
  DDRD &= ~(1 << S6_PIN); // Sensor is an input 

  DDRB &= ~(1 << C1_PIN); // C1 is an input 
  DDRB &= ~(1 << C2_PIN); // C2 is an input 
  DDRB &= ~(1 << C3_PIN); // C3 is an input 
  DDRB &= ~(1 << C4_PIN); // C4 is an input

  PORTB |= (1<<C1_PIN); // Pull-up
  PORTB |= (1<<C2_PIN); // Pull-up
  PORTB |= (1<<C3_PIN); // Pull-up
  PORTB |= (1<<C4_PIN); // Pull-up

  DDRB |= (1 << L_PIN); //clock shift register output
  DDRB |= (1 << S_PIN); //bit shift register output

}


int main(void)
{
  int  alarm_state = 0, vc_state = 0;
  int z1_state = 0, z2_state = 0, z3_state = 0, z4_state = 0, z5_state = 0, z6_state = 0;
  char codigo[3], inv_state[3];
  char date_ano[2],date_mes[2],date_dia[2],hour_hora[2],hour_min[2],hour_sec[2],zona[2];


  codigo[0]='1';
  codigo[1]='1';
  codigo[2]='1';
  codigo[3]='1';



  struct rtc_time tempo;
  struct rtc_date data;
  
  /*data.day = 4;
  data.month = 12;
  data.year = 20;
  tempo.hour = 14;
  tempo.min = 22;
  tempo.sec=0;*/
  
  tc1_init();
  io_init();                           
  sei();                              // Enable interrupts after every initialization

  start_timer(&Tshow,TSHOW);
  
  printf_init(); // Init the serial port to have the ability to printf
  i2c_init();  
  rtc3231_init();
  lcd1602_init();              


  //rtc3231_write_date(&data);
  //rtc3231_write_time(&tempo);

  uint8_t registo;
  uint8_t sec, min, hora, dia, mes, ano; 
  uint8_t old_S1,old_S2,old_S3,old_S4,old_S5,old_S6,S1,S2,S3,S4,S5,S6, RE_S1, RE_S2, RE_S3, RE_S4, RE_S5, RE_S6;
  char RE_K, old_K,K;
  int flag=0;

  
  if(eeprom_read_byte(7)!= 0 && eeprom_read_byte(7)!= 1 && eeprom_read_byte(7)!= 2 && eeprom_read_byte(7)!= 3 && eeprom_read_byte(7)!= 4 && eeprom_read_byte(7)!= 5)
  registo = 0;

  else  registo = eeprom_read_byte(7);

  lcd1602_clear();
  lcd1602_goto_xy(0,0);
  lcd1602_send_string("Insert the code!");


  while (1) {

    rtc3231_read_date(&data);
    rtc3231_init();
    rtc3231_read_time(&tempo);
    rtc3231_init(); 

    old_S1 = S1;
    old_S2 = S2;
    old_S3 = S3;
    old_S4 = S4;
    old_S5 = S5;
    old_S6 = S6;



    S1 = read_BUTTON(1);
    S2 = read_BUTTON(2);
    S3 = read_BUTTON(3);
    S4 = read_BUTTON(4);
    S5 = read_BUTTON(5);
    S6 = !read_BUTTON(6);


    RE_S1 = S1 && !old_S1;
    RE_S2 = S2 && !old_S2;
    RE_S3 = S3 && !old_S3;
    RE_S4 = S4 && !old_S4;
    RE_S5 = S5 && !old_S5;
    RE_S6 = S6 && !old_S6;

    old_K = K;

    K = Keypad();

    if(old_K != K && K != 'O')
    {RE_K=1;}
    else
    {RE_K=0;}

     
    hora= tempo.hour;
    min=tempo.min;
    sec= tempo.sec;
    dia=data.day;
    mes=data.month;
    ano=data.year;

    //------------------------------------------------------------------------------------Verifica código--------------------------------------------------------------------------//

    if(vc_state==0 && (alarm_state == 0 || alarm_state == 3) && RE_K && !(K == 'A') && (K == codigo[0]))
    {vc_state = 1;
    flag=0;
    inv_state[0]=0;
    inv_state[1]=0;
    inv_state[2]=0;
    inv_state[3]=0;
    lcd1602_clear();
    lcd1602_goto_xy(0,0);
    lcd1602_send_string("Insert the code!");
    lcd1602_goto_xy(7,1);
    lcd1602_send_string("*");}

    else if(vc_state==0 && (alarm_state == 0 || alarm_state == 3) && RE_K && !(K == 'A') && (K != codigo[0]))
    {vc_state = 1;
     inv_state[0] = 1;
    lcd1602_clear();
    lcd1602_goto_xy(0,0);
    lcd1602_send_string("Insert the code!");
    lcd1602_goto_xy(7,1);
    lcd1602_send_string("*");}

    else if(vc_state == 1 && RE_K && !(K == 'A') && (K == codigo[1]))
    {vc_state = 2;
    lcd1602_clear();
    lcd1602_goto_xy(0,0);
    lcd1602_send_string("Insert the code!");
    lcd1602_goto_xy(7,1);
    lcd1602_send_string("*");
    lcd1602_goto_xy(8,1);
    lcd1602_send_string("*");}

    else if(vc_state == 1 && RE_K && !(K == 'A') && (K != codigo[1]))
    {vc_state = 2;
    inv_state[1] = 1;
    lcd1602_clear();
    lcd1602_goto_xy(0,0);
    lcd1602_send_string("Insert the code!");
    lcd1602_goto_xy(7,1);
    lcd1602_send_string("*");
    lcd1602_goto_xy(8,1);
    lcd1602_send_string("*");}

    else if(vc_state == 1 && (K == 'A') && RE_K )
    {vc_state = 0;
     inv_state[0] = 0;
    lcd1602_clear();
    lcd1602_goto_xy(0,0);
    lcd1602_send_string("Insert the code!");}

    else if(vc_state == 2 && RE_K && !(K == 'A') && (K == codigo[1]))
    {vc_state = 3;
    lcd1602_clear();
    lcd1602_goto_xy(0,0);
    lcd1602_send_string("Insert the code!");
    lcd1602_goto_xy(7,1);
    lcd1602_send_string("*");
    lcd1602_goto_xy(8,1);
    lcd1602_send_string("*");
    lcd1602_goto_xy(9,1);
    lcd1602_send_string("*");}

    else if(vc_state == 2 && RE_K && !(K == 'A') && (K != codigo[1]))
    {vc_state = 3;
     inv_state[2]=1;
    lcd1602_clear();
    lcd1602_goto_xy(0,0);
    lcd1602_send_string("Insert the code!");
    lcd1602_goto_xy(7,1);
    lcd1602_send_string("*");
    lcd1602_goto_xy(8,1);
    lcd1602_send_string("*");
    lcd1602_goto_xy(9,1);
    lcd1602_send_string("*");}

    else if(vc_state == 2 && (K == 'A') && RE_K)
    {vc_state = 1;
     inv_state[1]=0;
    lcd1602_clear();
    lcd1602_goto_xy(0,0);
    lcd1602_send_string("Insert the code!");
    lcd1602_goto_xy(7,1);
    lcd1602_send_string("*");}

    else if(vc_state == 3 && RE_K && !(K == 'A') && (K == codigo[1]))
    {vc_state = 4;
    lcd1602_clear();
    lcd1602_goto_xy(0,0);
    lcd1602_send_string("Insert the code!");
    lcd1602_goto_xy(7,1);
    lcd1602_send_string("*");
    lcd1602_goto_xy(8,1);
    lcd1602_send_string("*");
    lcd1602_goto_xy(9,1);
    lcd1602_send_string("*");
    lcd1602_goto_xy(10,1);
    lcd1602_send_string("*");}

    else if(vc_state == 3 && RE_K && !(K == 'A') && (K != codigo[1]))
    {vc_state = 4;
     inv_state[3]=1;
    lcd1602_clear();
    lcd1602_goto_xy(0,0);
    lcd1602_send_string("Insert the code!");
    lcd1602_goto_xy(7,1);
    lcd1602_send_string("*");
    lcd1602_goto_xy(8,1);
    lcd1602_send_string("*");
    lcd1602_goto_xy(9,1);
    lcd1602_send_string("*");
    lcd1602_goto_xy(10,1);
    lcd1602_send_string("*");}

    else if(vc_state == 3 && (K == 'A') && RE_K)
    {vc_state = 2;
     inv_state[2]=0;
    lcd1602_clear();
    lcd1602_goto_xy(0,0);
    lcd1602_send_string("Insert the code!");
    lcd1602_goto_xy(7,1);
    lcd1602_send_string("*");
    lcd1602_goto_xy(8,1);
    lcd1602_send_string("*");}

    else if(vc_state == 4 && (K == 'A') && RE_K)
    {vc_state = 3;
     inv_state[3]=0;
    lcd1602_clear();
    lcd1602_goto_xy(0,0);
    lcd1602_send_string("Insert the code!");
    lcd1602_goto_xy(7,1);
    lcd1602_send_string("*");
    lcd1602_goto_xy(8,1);
    lcd1602_send_string("*");
    lcd1602_goto_xy(9,1);
    lcd1602_send_string("*");}

    else if(vc_state == 4 && (inv_state[0] == 0 && inv_state[1] == 0 && inv_state[2] == 0 && inv_state[3] == 0))
    {
      if(flag==0){
      lcd1602_clear();
      lcd1602_goto_xy(0,0);
      lcd1602_send_string("Valid code");
      start_timer(&validate,PASS);
      flag=1;}
      if(get_timer_time(validate,PASS) == PASS )
      {vc_state = 5;
      flag=0;}
    }

    else if(vc_state == 4 && ((inv_state[0] == 1 || inv_state[1] == 1|| inv_state[2] == 1 || inv_state[3] == 1)))
    {
      if(flag==0){
      lcd1602_clear();
      lcd1602_goto_xy(0,0);
      lcd1602_send_string("Invalid code");
      start_timer(&validate,PASS);
      flag=1;
      }
      if(get_timer_time(validate,PASS) == PASS )
      {vc_state = 0;
      flag=0;
      inv_state[0]=0;
      inv_state[1]=0;
      inv_state[2]=0;
      inv_state[3]=0;
      lcd1602_clear();
      lcd1602_goto_xy(0,0);
      lcd1602_send_string("Insert the code!");}
    }
    
    else if(vc_state == 5 && (alarm_state == 1 || alarm_state == 2 || alarm_state == 4))
    {vc_state =0;}





    //-------------------------------------------------------------------guardar na eeprom----------------------------------------------------------------------------------------//



    if(registo == 0 && alarm_state == 2 && ((z1_state == 1 && RE_S1) || (z2_state == 1 && RE_S2) || (z3_state == 1 && RE_S3)|| (z4_state == 1 && RE_S4) || (z5_state == 1 && RE_S5) || (z6_state == 1 && RE_S6))){
     eeprom_write_byte(0,data.year);
     eeprom_write_byte(1,data.month);
     eeprom_write_byte(2,data.day);
     eeprom_write_byte(3,tempo.hour);
     eeprom_write_byte(4,tempo.min);
     eeprom_write_byte(5,tempo.sec);

     if(z1_state == 1 && RE_S1)
     {eeprom_write_byte(6,1);} 

     else if(z2_state == 1 && RE_S2)
     {eeprom_write_byte(6,2);} 

     else if(z3_state == 1 && RE_S3)
     {eeprom_write_byte(6,3);} 

     else if(z4_state == 1 && RE_S4)
     {eeprom_write_byte(6,4);} 

     else if(z5_state == 1 && RE_S5)
     {eeprom_write_byte(6,5);} 

     else if(z6_state == 1 && RE_S6)
     {eeprom_write_byte(6,6);}

     eeprom_write_byte(7,1);
     registo=1;
     }

     else if(registo==1 && alarm_state == 2 && ((z1_state == 1 && RE_S1) || (z2_state == 1 && RE_S2) || (z3_state == 1 && RE_S3)|| (z4_state == 1 && RE_S4) || (z5_state == 1 && RE_S5) || (z6_state == 1 && RE_S6))){
     eeprom_write_byte(10,ano);
     eeprom_write_byte(11,mes);
     eeprom_write_byte(12,dia);
     eeprom_write_byte(13,hora);
     eeprom_write_byte(14,min);
     eeprom_write_byte(15,sec);

     if(z1_state == 1 && RE_S1)
     {eeprom_write_byte(16,1);} 

     else if(z2_state == 1 && RE_S2)
     {eeprom_write_byte(16,2);} 

     else if(z3_state == 1 && RE_S3)
     {eeprom_write_byte(16,3);} 

     else if(z4_state == 1 && RE_S4)
     {eeprom_write_byte(16,4);} 

     else if(z5_state == 1 && RE_S5)
     {eeprom_write_byte(16,5);} 

     else if(z6_state == 1 && RE_S6)
     {eeprom_write_byte(16,6);} 
     
     eeprom_write_byte(7,2);
     registo=2;
     }

     else if(registo==2 && alarm_state == 2 && ((z1_state == 1 && RE_S1) || (z2_state == 1 && RE_S2) || (z3_state == 1 && RE_S3)|| (z4_state == 1 && RE_S4) || (z5_state == 1 && RE_S5) || (z6_state == 1 && RE_S6))){
     eeprom_write_byte(20,ano);
     eeprom_write_byte(21,mes);
     eeprom_write_byte(22,dia);
     eeprom_write_byte(23,hora);
     eeprom_write_byte(24,min);
     eeprom_write_byte(25,sec);

     if(z1_state == 1 && RE_S1)
     {eeprom_write_byte(26,1);} 

     else if(z2_state == 1 && RE_S2)
     {eeprom_write_byte(26,2);} 

     else if(z3_state == 1 && RE_S3)
     {eeprom_write_byte(26,3);} 

     else if(z4_state == 1 && RE_S4)
     {eeprom_write_byte(26,4);} 

     else if(z5_state == 1 && RE_S5)
     {eeprom_write_byte(26,5);} 

     else if(z6_state == 1 && RE_S6)
     {eeprom_write_byte(26,6);} 

     eeprom_write_byte(7,3);
      registo=3;
     }

     else if(registo==3 && alarm_state == 2 && ((z1_state == 1 && RE_S1) || (z2_state == 1 && RE_S2) || (z3_state == 1 && RE_S3)|| (z4_state == 1 && RE_S4) || (z5_state == 1 && RE_S5) || (z6_state == 1 && RE_S6)) ){
     eeprom_write_byte(30,ano);
     eeprom_write_byte(31,mes);
     eeprom_write_byte(32,dia);
     eeprom_write_byte(33,hora);
     eeprom_write_byte(34,min);
     eeprom_write_byte(35,sec);

     if(z1_state == 1 && RE_S1)
     {eeprom_write_byte(36,1);} 

     else if(z2_state == 1 && RE_S2)
     {eeprom_write_byte(36,2);} 

     else if(z3_state == 1 && RE_S3)
     {eeprom_write_byte(36,3);} 

     else if(z4_state == 1 && RE_S4)
     {eeprom_write_byte(36,4);} 

     else if(z5_state == 1 && RE_S5)
     {eeprom_write_byte(36,5);} 

     else if(z6_state == 1 && RE_S6)
     {eeprom_write_byte(36,6);} 

     eeprom_write_byte(7,4);
     registo=4;
     }

     else if(registo==4 && alarm_state == 2 && ((z1_state == 1 && RE_S1) || (z2_state == 1 && RE_S2) || (z3_state == 1 && RE_S3)|| (z4_state == 1 && RE_S4) || (z5_state == 1 && RE_S5) || (z6_state == 1 && RE_S6))){
     eeprom_write_byte(40,ano);
     eeprom_write_byte(41,mes);
     eeprom_write_byte(42,dia);
     eeprom_write_byte(43,hora);
     eeprom_write_byte(44,min);
     eeprom_write_byte(45,sec);

     if(z1_state == 1 && RE_S1)
     {eeprom_write_byte(46,1);} 

     else if(z2_state == 1 && RE_S2)
     {eeprom_write_byte(46,2);} 

     else if(z3_state == 1 && RE_S3)
     {eeprom_write_byte(46,3);} 

     else if(z4_state == 1 && RE_S4)
     {eeprom_write_byte(46,4);} 

     else if(z5_state == 1 && RE_S5)
     {eeprom_write_byte(46,5);} 

     else if(z6_state == 1 && RE_S6)
     {eeprom_write_byte(46,6);} 

     eeprom_write_byte(7,5);
      registo=5;
     }

     else if(registo==5 && alarm_state == 2 && ((z1_state == 1 && RE_S1) || (z2_state == 1 && RE_S2) || (z3_state == 1 && RE_S3)|| (z4_state == 1 && RE_S4) || (z5_state == 1 && RE_S5) || (z6_state == 1 && RE_S6))){
     eeprom_write_byte(50,ano);
     eeprom_write_byte(51,mes);
     eeprom_write_byte(52,dia);
     eeprom_write_byte(53,hora);
     eeprom_write_byte(54,min);
     eeprom_write_byte(55,sec);

     if(z1_state == 1 && RE_S1)
     {eeprom_write_byte(56,1);} 

     else if(z2_state == 1 && RE_S2)
     {eeprom_write_byte(56,2);} 

     else if(z3_state == 1 && RE_S3)
     {eeprom_write_byte(56,3);} 

     else if(z4_state == 1 && RE_S4)
     {eeprom_write_byte(56,4);} 

     else if(z5_state == 1 && RE_S5)
     {eeprom_write_byte(56,5);} 

     else if(z6_state == 1 && RE_S6)
     {eeprom_write_byte(56,6);} 

     eeprom_write_byte(7,0);
     registo=0;
     }
//--------------------------------------------------------------------------Alarme----------------------------------------------------------------------------------------------//


    if(alarm_state == 0 && vc_state == 5 )
    {alarm_state = 4;
      lcd1602_clear();
      lcd1602_goto_xy(0,0);
      lcd1602_send_string("Press 1 - D.P.Z.");
      lcd1602_goto_xy(0,1);
      lcd1602_send_string("Press 2 - Reg.");}

    else if(alarm_state == 4 && K=='1')
    {
      alarm_state = 1;
      lcd1602_clear();
      lcd1602_goto_xy(0,0);
      lcd1602_send_string("Define protected");
      lcd1602_goto_xy(5,1);
      lcd1602_send_string("zones");
    }

    else if(alarm_state == 1 && K=='B' && RE_K)
    {
      alarm_state = 4;
      lcd1602_clear();
      lcd1602_goto_xy(0,0);
      lcd1602_send_string("Press 1 - D.P.Z.");
      lcd1602_goto_xy(0,1);
      lcd1602_send_string("Press 2 - Reg.");}


    else if(alarm_state == 5 && K=='B' && RE_K)
    {
      alarm_state = 4;
      lcd1602_clear();
      lcd1602_goto_xy(0,0);
      lcd1602_send_string("Press 1 - D.P.Z.");
      lcd1602_goto_xy(0,1);
      lcd1602_send_string("Press 2 - Reg.");}

    else if(alarm_state == 4 && K=='B')
    {
      alarm_state = 0;
      lcd1602_clear();
      lcd1602_goto_xy(0,0);
      lcd1602_send_string("Insert the code!");

    }


    else if(alarm_state == 4 && K=='2'&& RE_K)
    {
      alarm_state = 5;
      lcd1602_clear();
      lcd1602_goto_xy(0,0);
      lcd1602_send_string("Choose Register");
      lcd1602_goto_xy(7,1);
      lcd1602_send_string("1->6");
    }

    else if(alarm_state == 5 && K == '1' && RE_K)
    {
      alarm_state = 10;
      lcd1602_clear();
      lcd1602_goto_xy(0,0);
      sprintf(date_dia,"%u",eeprom_read_byte(2));
      lcd1602_send_string(date_dia);
      lcd1602_goto_xy(2,0);
      lcd1602_send_char('/');
      lcd1602_goto_xy(3,0);
      sprintf(date_mes,"%u",eeprom_read_byte(1));
      lcd1602_send_string(date_mes);
      lcd1602_goto_xy(5,0);
      lcd1602_send_char('/');
      lcd1602_goto_xy(6,0);
      sprintf(date_ano,"%u",eeprom_read_byte(0));
      lcd1602_send_string(date_ano);
      lcd1602_goto_xy(0,1);
      sprintf(hour_hora,"%u",eeprom_read_byte(3));
      lcd1602_send_string(hour_hora);
      lcd1602_goto_xy(2,1);
      lcd1602_send_char(':');
      lcd1602_goto_xy(3,1);
      sprintf(hour_min,"%u",eeprom_read_byte(4));
      lcd1602_send_string(hour_min);
      lcd1602_goto_xy(5,1);
      lcd1602_send_char(':');
      lcd1602_goto_xy(6,1);
      sprintf(hour_sec,"%u",eeprom_read_byte(5));
      lcd1602_send_string(hour_sec);
      lcd1602_goto_xy(10,0);
      lcd1602_send_string("Zone");
      lcd1602_goto_xy(12,1);
      sprintf(zona,"%u",eeprom_read_byte(6));
      lcd1602_send_string(zona);
    }

    else if(alarm_state == 10 && K=='B' && RE_K)
    {
      alarm_state = 5;
      lcd1602_clear();
      lcd1602_goto_xy(0,0);
      lcd1602_send_string("Choose Register");
      lcd1602_goto_xy(7,1);
      lcd1602_send_string("1->6");

    }

    else if(alarm_state == 5 && K == '2' && RE_K)
    {
      alarm_state = 11;
      lcd1602_clear();
      lcd1602_goto_xy(0,0);
      sprintf(date_dia,"%u",eeprom_read_byte(12));
      lcd1602_send_string(date_dia);
      lcd1602_goto_xy(2,0);
      lcd1602_send_char('/');
      lcd1602_goto_xy(3,0);
      sprintf(date_mes,"%u",eeprom_read_byte(11));
      lcd1602_send_string(date_mes);
      lcd1602_goto_xy(5,0);
      lcd1602_send_char('/');
      lcd1602_goto_xy(6,0);
      sprintf(date_ano,"%u",eeprom_read_byte(10));
      lcd1602_send_string(date_ano);
      lcd1602_goto_xy(0,1);
      sprintf(hour_hora,"%u",eeprom_read_byte(13));
      lcd1602_send_string(hour_hora);
      lcd1602_goto_xy(2,1);
      lcd1602_send_char(':');
      lcd1602_goto_xy(3,1);
      sprintf(hour_min,"%u",eeprom_read_byte(14));
      lcd1602_send_string(hour_min);
      lcd1602_goto_xy(5,1);
      lcd1602_send_char(':');
      lcd1602_goto_xy(6,1);
      sprintf(hour_sec,"%u",eeprom_read_byte(15));
      lcd1602_send_string(hour_sec);
      lcd1602_goto_xy(10,0);
      lcd1602_send_string("Zone");
      lcd1602_goto_xy(12,1);
      sprintf(zona,"%u",eeprom_read_byte(16));
      lcd1602_send_string(zona);
    }

    else if(alarm_state == 11 && K=='B' && RE_K)
    {
      alarm_state = 5;
      lcd1602_clear();
      lcd1602_goto_xy(0,0);
      lcd1602_send_string("Choose Register");
      lcd1602_goto_xy(7,1);
      lcd1602_send_string("1->6");

    }

    else if(alarm_state == 5 && K == '3' && RE_K)
    {
      alarm_state = 12;
      lcd1602_clear();
      lcd1602_goto_xy(0,0);
      sprintf(date_dia,"%u",eeprom_read_byte(22));
      lcd1602_send_string(date_dia);
      lcd1602_goto_xy(2,0);
      lcd1602_send_char('/');
      lcd1602_goto_xy(3,0);
      sprintf(date_mes,"%u",eeprom_read_byte(21));
      lcd1602_send_string(date_mes);
      lcd1602_goto_xy(5,0);
      lcd1602_send_char('/');
      lcd1602_goto_xy(6,0);
      sprintf(date_ano,"%u",eeprom_read_byte(20));
      lcd1602_send_string(date_ano);
      lcd1602_goto_xy(0,1);
      sprintf(hour_hora,"%u",eeprom_read_byte(23));
      lcd1602_send_string(hour_hora);
      lcd1602_goto_xy(2,1);
      lcd1602_send_char(':');
      lcd1602_goto_xy(3,1);
      sprintf(hour_min,"%u",eeprom_read_byte(24));
      lcd1602_send_string(hour_min);
      lcd1602_goto_xy(5,1);
      lcd1602_send_char(':');
      lcd1602_goto_xy(6,1);
      sprintf(hour_sec,"%u",eeprom_read_byte(25));
      lcd1602_send_string(hour_sec);
      lcd1602_goto_xy(10,0);
      lcd1602_send_string("Zone");
      lcd1602_goto_xy(12,1);
      sprintf(zona,"%u",eeprom_read_byte(26));
      lcd1602_send_string(zona);
    }

    else if(alarm_state == 12 && K=='B' && RE_K)
    {
      alarm_state = 5;
      lcd1602_clear();
      lcd1602_goto_xy(0,0);
      lcd1602_send_string("Choose Register");
      lcd1602_goto_xy(7,1);
      lcd1602_send_string("1->6");

    }

    else if(alarm_state == 5 && K == '4' && RE_K)
    {
      alarm_state = 13;
      lcd1602_clear();
      lcd1602_goto_xy(0,0);
      sprintf(date_dia,"%u",eeprom_read_byte(32));
      lcd1602_send_string(date_dia);
      lcd1602_goto_xy(2,0);
      lcd1602_send_char('/');
      lcd1602_goto_xy(3,0);
      sprintf(date_mes,"%u",eeprom_read_byte(31));
      lcd1602_send_string(date_mes);
      lcd1602_goto_xy(5,0);
      lcd1602_send_char('/');
      lcd1602_goto_xy(6,0);
      sprintf(date_ano,"%u",eeprom_read_byte(30));
      lcd1602_send_string(date_ano);
      lcd1602_goto_xy(0,1);
      sprintf(hour_hora,"%u",eeprom_read_byte(33));
      lcd1602_send_string(hour_hora);
      lcd1602_goto_xy(2,1);
      lcd1602_send_char(':');
      lcd1602_goto_xy(3,1);
      sprintf(hour_min,"%u",eeprom_read_byte(34));
      lcd1602_send_string(hour_min);
      lcd1602_goto_xy(5,1);
      lcd1602_send_char(':');
      lcd1602_goto_xy(6,1);
      sprintf(hour_sec,"%u",eeprom_read_byte(35));
      lcd1602_send_string(hour_sec);
      lcd1602_goto_xy(10,0);
      lcd1602_send_string("Zone");
      lcd1602_goto_xy(12,1);
      sprintf(zona,"%u",eeprom_read_byte(36));
      lcd1602_send_string(zona);
    }

    else if(alarm_state == 13 && K=='B' && RE_K)
    {
      alarm_state = 5;
      lcd1602_clear();
      lcd1602_goto_xy(0,0);
      lcd1602_send_string("Choose Register");
      lcd1602_goto_xy(7,1);
      lcd1602_send_string("1->6");

    }

    else if(alarm_state == 5 && K == '5' && RE_K)
    {
      alarm_state = 14;
      lcd1602_clear();
      lcd1602_goto_xy(0,0);
      sprintf(date_dia,"%u",eeprom_read_byte(42));
      lcd1602_send_string(date_dia);
      lcd1602_goto_xy(2,0);
      lcd1602_send_char('/');
      lcd1602_goto_xy(3,0);
      sprintf(date_mes,"%u",eeprom_read_byte(41));
      lcd1602_send_string(date_mes);
      lcd1602_goto_xy(5,0);
      lcd1602_send_char('/');
      lcd1602_goto_xy(6,0);
      sprintf(date_ano,"%u",eeprom_read_byte(40));
      lcd1602_send_string(date_ano);
      lcd1602_goto_xy(0,1);
      sprintf(hour_hora,"%u",eeprom_read_byte(43));
      lcd1602_send_string(hour_hora);
      lcd1602_goto_xy(2,1);
      lcd1602_send_char(':');
      lcd1602_goto_xy(3,1);
      sprintf(hour_min,"%u",eeprom_read_byte(44));
      lcd1602_send_string(hour_min);
      lcd1602_goto_xy(5,1);
      lcd1602_send_char(':');
      lcd1602_goto_xy(6,1);
      sprintf(hour_sec,"%u",eeprom_read_byte(45));
      lcd1602_send_string(hour_sec);
      lcd1602_goto_xy(10,0);
      lcd1602_send_string("Zone");
      lcd1602_goto_xy(12,1);
      sprintf(zona,"%u",eeprom_read_byte(46));
      lcd1602_send_string(zona);
    }

    else if(alarm_state == 14 && K=='B' && RE_K)
    {
      alarm_state = 5;
      lcd1602_clear();
      lcd1602_goto_xy(0,0);
      lcd1602_send_string("Choose Register");
      lcd1602_goto_xy(7,1);
      lcd1602_send_string("1->6");

    }

    else if(alarm_state == 5 && K == '6' && RE_K)
    {
      alarm_state = 15;
      lcd1602_clear();
      lcd1602_goto_xy(0,0);
      sprintf(date_dia,"%u",eeprom_read_byte(52));
      lcd1602_send_string(date_dia);
      lcd1602_goto_xy(2,0);
      lcd1602_send_char('/');
      lcd1602_goto_xy(3,0);
      sprintf(date_mes,"%u",eeprom_read_byte(51));
      lcd1602_send_string(date_mes);
      lcd1602_goto_xy(5,0);
      lcd1602_send_char('/');
      lcd1602_goto_xy(6,0);
      sprintf(date_ano,"%u",eeprom_read_byte(50));
      lcd1602_send_string(date_ano);
      lcd1602_goto_xy(0,1);
      sprintf(hour_hora,"%u",eeprom_read_byte(53));
      lcd1602_send_string(hour_hora);
      lcd1602_goto_xy(2,1);
      lcd1602_send_char(':');
      lcd1602_goto_xy(3,1);
      sprintf(hour_min,"%u",eeprom_read_byte(54));
      lcd1602_send_string(hour_min);
      lcd1602_goto_xy(5,1);
      lcd1602_send_char(':');
      lcd1602_goto_xy(6,1);
      sprintf(hour_sec,"%u",eeprom_read_byte(55));
      lcd1602_send_string(hour_sec);
      lcd1602_goto_xy(10,0);
      lcd1602_send_string("Zone");
      lcd1602_goto_xy(12,1);
      sprintf(zona,"%u",eeprom_read_byte(56));
      lcd1602_send_string(zona);
    }

    else if(alarm_state == 15 && K == 'B' && RE_K)
    {
      alarm_state = 5;
      lcd1602_clear();
      lcd1602_goto_xy(0,0);
      lcd1602_send_string("Choose Register");
      lcd1602_goto_xy(7,1);
      lcd1602_send_string("1->6");

    }


    else if(alarm_state == 1 && (Keypad() == 'D') &&  RE_K )
    {alarm_state = 2;
     lcd1602_clear();
     lcd1602_goto_xy(0,0);
     lcd1602_send_string("Alarm is working");
     lcd1602_goto_xy(0,1);
     lcd1602_send_string("PZ ->");
     if(z1_state == 1) 
     {lcd1602_goto_xy(5,1);
     lcd1602_send_char('1');
     lcd1602_goto_xy(6,1);
     lcd1602_send_char('/');}

     else if(z1_state == 0)
     {lcd1602_goto_xy(5,1);
     lcd1602_send_char('X');
     lcd1602_goto_xy(6,1);
     lcd1602_send_char('/');}

     if(z2_state == 1)
      {lcd1602_goto_xy(7,1);
      lcd1602_send_char('2');
      lcd1602_goto_xy(8,1);
      lcd1602_send_char('/');}

     else if(z2_state == 0)
     {lcd1602_goto_xy(7,1);
     lcd1602_send_char('X');
     lcd1602_goto_xy(8,1);
     lcd1602_send_char('/');}
     if(z3_state == 1)
      {lcd1602_goto_xy(9,1);
      lcd1602_send_char('3');
      lcd1602_goto_xy(10,1);
      lcd1602_send_char('/');}

     else if(z3_state == 0)
     {lcd1602_goto_xy(9,1);
     lcd1602_send_char('X');
     lcd1602_goto_xy(10,1);
     lcd1602_send_char('/');}

     if(z4_state == 1) 
     {lcd1602_goto_xy(11,1);
     lcd1602_send_char('4');
     lcd1602_goto_xy(12,1);
     lcd1602_send_char('/');}

     else if(z4_state == 0)
     {lcd1602_goto_xy(11,1);
     lcd1602_send_char('X');
     lcd1602_goto_xy(12,1);
     lcd1602_send_char('/');}

     if(z5_state == 1) 
     {lcd1602_goto_xy(13,1);
     lcd1602_send_char('5');
     lcd1602_goto_xy(14,1);
     lcd1602_send_char('/');}

     else if(z5_state == 0)
     {lcd1602_goto_xy(13,1);
     lcd1602_send_char('X');
     lcd1602_goto_xy(14,1);
     lcd1602_send_char('/');}

     if(z6_state == 1) 
     {lcd1602_goto_xy(15,1);
     lcd1602_send_char('6');}

     else if(z6_state == 0)
     {lcd1602_goto_xy(15,1);
     lcd1602_send_char('X');}}

    else if(alarm_state == 2 && (K == 'B') && RE_K)
    {alarm_state = 0;
     lcd1602_clear();
     lcd1602_goto_xy(0,0);
     lcd1602_send_string("Insert the code!");}

    else if(alarm_state == 2 && ((z1_state == 1 && RE_S1) || (z2_state == 1 && RE_S2) || (z3_state == 1 && RE_S3)|| (z4_state == 1 && RE_S4) || (z5_state == 1 && RE_S5) || (z6_state == 1 && RE_S6)))
    {alarm_state = 3;
     start_timer(&buzzer,T_ALARME);
     lcd1602_clear();
     lcd1602_goto_xy(0,0);
     lcd1602_send_string("!!!!ANOMALY!!!!");
     lcd1602_goto_xy(0,1);
     lcd1602_send_string("Ins. key 2 stop");
     }

    else if(alarm_state == 3 && (vc_state == 5 || (get_timer_time(buzzer,T_ALARME)==T_ALARME)))
    {alarm_state = 2;
     lcd1602_clear();
     lcd1602_goto_xy(0,0);
     lcd1602_send_string("Alarm is working");
     lcd1602_goto_xy(0,1);
     lcd1602_send_string("PZ ->");
     if(z1_state == 1) 
     {lcd1602_goto_xy(5,1);
     lcd1602_send_char('1');
     lcd1602_goto_xy(6,1);
     lcd1602_send_char('/');}

     else if(z1_state == 0)
     {lcd1602_goto_xy(5,1);
     lcd1602_send_char('X');
     lcd1602_goto_xy(6,1);
     lcd1602_send_char('/');}

     if(z2_state == 1) 
     {lcd1602_goto_xy(7,1);
     lcd1602_send_char('2');
     lcd1602_goto_xy(8,1);
     lcd1602_send_char('/');}

     else if(z2_state == 0)
     {lcd1602_goto_xy(7,1);
     lcd1602_send_char('X');
     lcd1602_goto_xy(8,1);
     lcd1602_send_char('/');}

     if(z3_state == 1) 
     {lcd1602_goto_xy(9,1);
     lcd1602_send_char('3');
     lcd1602_goto_xy(10,1);
     lcd1602_send_char('/');}

     else if(z3_state == 0)
     {lcd1602_goto_xy(9,1);
     lcd1602_send_char('X');
     lcd1602_goto_xy(10,1);
     lcd1602_send_char('/');}
     
     if(z4_state == 1) 
     {lcd1602_goto_xy(11,1);
     lcd1602_send_char('4');
     lcd1602_goto_xy(12,1);
     lcd1602_send_char('/');}

     else if(z4_state == 0)
     {lcd1602_goto_xy(11,1);
     lcd1602_send_char('X');
     lcd1602_goto_xy(12,1);
     lcd1602_send_char('/');}

     if(z5_state == 1)
      {lcd1602_goto_xy(13,1);
      lcd1602_send_char('5');
      lcd1602_goto_xy(14,1);
      lcd1602_send_char('/');}

     else if(z5_state == 0)
     {lcd1602_goto_xy(13,1);
     lcd1602_send_char('X');
     lcd1602_goto_xy(14,1);
     lcd1602_send_char('/');}

     if(z6_state == 1) 
     {lcd1602_goto_xy(15,1);
     lcd1602_send_char('6');}

     else if(z6_state == 0)
     {lcd1602_goto_xy(15,1);
     lcd1602_send_char('X');}
     
     }


//----------------------------------------------------------------------------- Zonas ativas --------------------------------------------------------------------------------------//
    if(z1_state == 0 && alarm_state == 1 && Keypad()== '1' && RE_K){z1_state = 1;}
    else if(z1_state == 1 && alarm_state == 0){z1_state = 0;}
    else if(z1_state == 1 && (Keypad() == 'C') && RE_K){z1_state = 0;}

    if(z2_state == 0 && alarm_state == 1 && Keypad()== '2' && RE_K ){z2_state = 1;}
    else if(z2_state == 1 && alarm_state == 0){z2_state = 0;}
    else if(z2_state == 1 && (Keypad() == 'C') && RE_K){z1_state = 0;}

    if(z3_state == 0 && alarm_state == 1 && Keypad()== '3' && RE_K){z3_state = 1;}
    else if(z3_state == 1 && alarm_state == 0){z3_state = 0;}
    else if(z3_state == 1 && (Keypad() == 'C') && RE_K){z3_state = 0;}

    if(z4_state == 0 && alarm_state == 1 && Keypad()== '4' && RE_K){z4_state = 1;}
    else if(z4_state == 1 && alarm_state == 0){z4_state = 0;}
    else if(z4_state == 1 && (Keypad() == 'C') && RE_K){z4_state = 0;}

    if(z5_state == 0 && alarm_state == 1 && Keypad()== '5' && RE_K){z5_state = 1;}
    else if(z5_state == 1 && alarm_state == 0){z5_state = 0;}
    else if(z5_state == 1 && (Keypad() == 'C') && RE_K){z5_state = 0;}

    if(z6_state == 0 && alarm_state == 1 && Keypad()== '6' && RE_K){z6_state = 1;}
    else if(z6_state == 1 && alarm_state == 0){z6_state = 0;}
    else if(z6_state == 1 && (Keypad() == 'C') && RE_K){z6_state = 0;}



    set_UP(alarm_state == 0 || alarm_state == 1 || alarm_state == 4 || alarm_state == 5 || alarm_state == 10 || alarm_state == 11 || alarm_state == 12 || alarm_state == 13 || alarm_state == 14 || alarm_state == 15);
    set_Buzzer(alarm_state == 3);
    set_LEDok(alarm_state == 2);

  
    if(get_timer_time(Tshow,TSHOW) == TSHOW)
    {start_timer(&Tshow,TSHOW);
    printf("Tempo %u:%u:%u Data %u/%u/%u  State_Alarm : %d  State_VC : %d  ", tempo.hour, tempo.min, tempo.sec, data.day, data.month,data.year,alarm_state,vc_state);
    printf("Registo 1: Data %u/%u/%u Hora %u:%u:%u Zona: %u ",eeprom_read_byte(2),eeprom_read_byte(1),eeprom_read_byte(0),eeprom_read_byte(3),eeprom_read_byte(4),eeprom_read_byte(5),eeprom_read_byte(6));
    printf("Registo 2: Data %u/%u/%u Hora %u:%u:%u Zona: %u registo: %u \n",eeprom_read_byte(12),eeprom_read_byte(11),eeprom_read_byte(10),eeprom_read_byte(13),eeprom_read_byte(14),eeprom_read_byte(15),eeprom_read_byte(16), registo);}
    
  }
    
}

