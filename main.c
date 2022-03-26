/***************************************************************************************************
   23/3/22 comienzo de seguimiento con git                                         
****************************************************************************************************

AHORA CON RTC DESDE 25/3/22


 * File:   
 * Version: 
 * Author: 
 * Website: 
 * Description:LCD in 8-bit mode. 

  8051 La frecuencia del oscilador se divide por 12 y luego se envía al controlador.
  El tiempo para incrementar la cuenta del temporizador en un se puede determinar como 
	se indica a continuación:
  tick = (1 / (Fosc / 12)
  $$ tick = 12 / Fosc $$ Para Fosc == 11.0592Mhz, el tiempo de tick será
  tick = 12 / 11.0592M = 1.085069444us = 1.085us

  Cálculo del temporizador para un retraso de 50 ms
  Fosc = 11.0592Mhz
  Delay = 50ms
  $$ RegValue = TimerMax-((Delay/1.085)* 10^6) $$RegValue =65536-(50ms/1.085)*10 ^6= 65536-46082=19453=0x4BFD

**************************************************************************************************/

 /*
 Interrupcion 	             Ubicación de ROM (hexa)	  Pin 	 Borrado de bandera	            Interrup. numero
Externa 0 (INT0)	                 0003	             P3,2 (12)	       Auto	                         0
Temporizador 0  (TF0)	             000B	                               Auto	                         1
Interrupción externa 1 (INT1)      0013	             P3,3 (13)	       Auto	                         2
Temporizador 1 (TF1)	             001B	                             	 Auto	                         3
Int. de com.serie (RI y TI)	       0023		                          Programa SW	                     4       */

#include <reg51.h>
#include <INTRINS.h>

//#include "delay.h"

typedef signed char     int8_t;
typedef signed char     sint8_t;
typedef unsigned char   uint8_t;

typedef signed int      int16_t;
typedef signed int      sint16_t;
typedef unsigned int    uint16_t;

typedef signed long int    int32_t;
typedef signed long int    sint32_t;
typedef unsigned long int  uint32_t;

#define LcdDataBus  P1
#define  NULL ((void)0)
#define on 		1
#define off 	0

#define Ds1307ReadMode   0xD1u  // DS1307 ID
#define Ds1307WriteMode  0xD0u  // DS1307 ID

#define Ds1307SecondRegAddress   0x00u    // Address to access Ds1307 SEC register
#define Ds1307DateRegAddress     0x04u    // Address to access Ds1307 DATE register
#define Ds1307ControlRegAddress  0x07u    // Address to access Ds1307 CONTROL register

#define DS18B20_SKIP_ROM  0xCC
#define DS18B20_CONVERT_T 0x44
#define DS18B20_READ_SCRATCHPAD 0xBE

sbit SCL_PIN = P3^1; //SCL Connected to P3.1
sbit SDA_PIN = P3^0; //SDA Connected to P3.0

#define SCL_Direction SCL_PIN
#define SDA_Direction SDA_PIN

uint8_t timer1Count=0;
uint8_t j = 1;

uint8_t timer0Count=0;

uint8_t mascara = 0;
uint8_t led_i = 0;
sbit LCD_RS = P3^5;
sbit LCD_RW = P3^7;
sbit LCD_EN = P3^6;
sbit buzer = P3^3;
sbit LED0 = P3^4;
sbit latch = P3^2;

sbit r1=P2^7;
sbit r2=P2^6;
sbit r3=P2^5;
sbit r4=P2^4;
sbit c1=P2^3;
sbit c2=P2^2;
sbit c3=P2^1;
sbit c4=P2^0;
sbit D1 = P0^7;
sbit D2 = P0^6;
sbit D3 = P0^5;
sbit D4 = P0^4;

//sbit DS18B20_PIN = P3^7;

uint8_t u_seg = 0;
uint8_t d_min = 0;
uint8_t u_hor = 0;
uint8_t d_seg = 0;
uint8_t u_min = 0;
uint8_t d_hor = 0;
uint8_t u_mes = 0;
uint8_t d_mes = 0;
uint8_t u_anio = 0;
uint8_t d_anio = 0;
uint8_t u_dia = 0;
uint8_t d_dia = 0;

uint8_t tecla ;

uint8_t sec;
uint8_t min;
uint8_t hour;
uint8_t weekDay;
uint8_t date;
uint8_t month;
uint8_t year;  

unsigned char readdata[2];
sbit DQ=P3^7;
unsigned char word1[20]={"TEMP.  "};		//display de 20 caracteres

/***************************************************************************************************
                           local function prototypes
 ***************************************************************************************************/
void delay_micros(int cnt);
void LCD_Dato(uint8_t dat);
void LCD_Comando(uint8_t cmd);
void reloj(void);	
void teclado(void);
void DELAY_us(uint16_t us_count);
void beep (void);

void mostrar_hora(int intensidad);
//void borrar_reloj(void);
void actualizar_hora(void);
static void i2c_Clock(void);
static void i2c_Ack(void);
static void i2c_NoAck(void);


/***************************************************************************************************
                             Function prototypes
***************************************************************************************************/
void LCD_mostrar(uint8_t *);
void LCD_inicio ( void);
void DELAY_ms(uint16_t ms_count);
void LCD_Dato(uint8_t dat);
void LCD_Comando(uint8_t cmd);
void Inicio_Timer_interrupt(void);
//void timer0_isr() interrupt 1;
//void timer1_isr() interrupt 3;
void RTC_Init(void);
void RTC_SetDateTime();
void RTC_GetDateTime();
void I2C_Init(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_Write(uint8_t );
uint8_t I2C_Read(uint8_t);

///////////////ds18b20/////////////////////
//Delay function
 
void delay(unsigned int i)
{
    while(i--);
}

void I2C_Init(void)
 {
 
 }

/***************************************************************************************************
                         void I2C_Start(void)
****************************************************************************************************
 * I/P Arguments: none.
 * Return value  : none
 * description  :This function is used to generate I2C Start Condition.
                 Start Condition: SDA goes low when SCL is High.
                               ____________
                SCL:          |            |
                      ________|            |______
                           _________
                SDA:      |         |
                      ____|         |____________
***************************************************************************************************/
void I2C_Start(void)
{
    SCL_PIN = 0;        // Pull SCL low
    SDA_PIN = 1;        // Pull SDA High
    DELAY_us(1);
    SCL_PIN = 1;        //Pull SCL high
    DELAY_us(1);
    SDA_PIN = 0;        //Now Pull SDA LOW, to generate the Start Condition
    DELAY_us(1);
    SCL_PIN = 0;        //Finally Clear the SCL to complete the cycle
}

/***************************************************************************************************
                         void I2C_Stop(void)
****************************************************************************************************
 * I/P Arguments: none.
 * Return value  : none
 * description  :This function is used to generate I2C Stop Condition.
                 Stop Condition: SDA goes High when SCL is High.
                               ____________
                SCL:          |            |
                      ________|            |______
                                 _________________
                SDA:            |
                      __________|
***************************************************************************************************/

void I2C_Stop(void)
{
    SCL_PIN = 0;            // Pull SCL low
    DELAY_us(1);
    SDA_PIN = 0;            // Pull SDA  low
    DELAY_us(1);
    SCL_PIN = 1;            // Pull SCL High
    DELAY_us(1);
    SDA_PIN = 1;            // Now Pull SDA High, to generate the Stop Condition
}

/***************************************************************************************************
                         void I2C_Write(uint8_t v_i2cData_u8)
****************************************************************************************************
 * I/P Arguments: uint8_t-->8bit data to be sent.
 * Return value  : none
 * description  :This function is used to send a byte on SDA line using I2C protocol
                 8bit data is sent bit-by-bit on each clock cycle.
                 MSB(bit) is sent first and LSB(bit) is sent at last.
                 Data is sent when SCL is low.
         ___     ___     ___     ___     ___     ___     ___     ___     ___     ___
 SCL:   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |
      __|   |___|   |___|   |___|   |___|   |___|   |___|   |___|   |___|   |___|   |___
 SDA:    D8       D7     D6      D5      D4       D3      D2      D1      D0     ACK
***************************************************************************************************/
void I2C_Write(uint8_t v_i2cData_u8)
{
    uint8_t i;

    for(i=0;i<8;i++)                   // loop 8 times to send 1-byte of data
    {
        SDA_PIN = v_i2cData_u8 & 0x80;     // Send Bit by Bit on SDA line
        i2c_Clock();                   // Generate Clock at SCL
        v_i2cData_u8 = v_i2cData_u8<<1;// Bring the next bit to be transmitted to MSB position
    }
                             
    i2c_Clock();
}

/***************************************************************************************************
                         uint8_t I2C_Read(uint8_t v_ackOption_u8)
****************************************************************************************************
 * I/P Arguments: none.
 * Return value  : uint8_t(received byte)
 * description :This fun is used to receive a byte on SDA line using I2C protocol.
               8bit data is received bit-by-bit each clock and finally packed into Byte.
               MSB(bit) is received first and LSB(bit) is received at last.
         ___     ___     ___     ___     ___     ___     ___     ___     ___     ___
SCL:    |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |
      __|   |___|   |___|   |___|   |___|   |___|   |___|   |___|   |___|   |___|   |__
 SDA:    D8       D7     D6      D5       D4     D3       D2      D1     D0      ACK
***************************************************************************************************/
uint8_t I2C_Read(uint8_t v_ackOption_u8)
{
    uint8_t i, v_i2cData_u8=0x00;

    SDA_PIN =1;               //Make SDA as I/P
    for(i=0;i<8;i++)     // loop 8times read 1-byte of data
    {
        DELAY_us(1);
        SCL_PIN = 1;         // Pull SCL High
        DELAY_us(1);

        v_i2cData_u8 = v_i2cData_u8<<1;    //v_i2cData_u8 is Shifted each time and
        v_i2cData_u8 = v_i2cData_u8 | SDA_PIN; //ORed with the received bit to pack into byte

        SCL_PIN = 0;         // Clear SCL to complete the Clock
    }
    if(v_ackOption_u8==1)  /*Send the Ack/NoAck depending on the user option*/
    {
        i2c_Ack();
    }
    else
    {
        i2c_NoAck();
    }

    return v_i2cData_u8;           // Finally return the received Byte*
}

/***************************************************************************************************
                         static void i2c_Clock()
****************************************************************************************************
 * I/P Arguments: none.
 * Return value  : none
 * description  :This function is used to generate a clock pulse on SCL line.
***************************************************************************************************/
static void i2c_Clock(void)
{
    DELAY_us(1);
    SCL_PIN = 1;            // Wait for Some time and Pull the SCL line High
    DELAY_us(1);        // Wait for Some time
    SCL_PIN = 0;            // Pull back the SCL line low to Generate a clock pulse
}

/***************************************************************************************************
                         static void i2c_Ack()
****************************************************************************************************
 * I/P Arguments: none.
 * Return value  : none
 * description  :This function is used to generate a the Positive ACK
                 pulse on SDA after receiving a byte.
***************************************************************************************************/
static void i2c_Ack(void)
{
    SDA_PIN = 0;        //Pull SDA low to indicate Positive ACK
    i2c_Clock();    //Generate the Clock
    SDA_PIN = 1;        // Pull SDA back to High(IDLE state)
}

/***************************************************************************************************
                        static void i2c_NoAck()
****************************************************************************************************
 * I/P Arguments: none.
 * Return value  : none
 * description  :This function is used to generate a the Negative/NO ACK
                 pulse on SDA after receiving all bytes.
***************************************************************************************************/
static void i2c_NoAck(void)
{
    SDA_PIN = 1;         //Pull SDA high to indicate Negative/NO ACK
    i2c_Clock();     // Generate the Clock  
    SCL_PIN = 1;         // Set SCL 
}

/***************************************************************************************************
                     void RTC_GetDateTime(rtc_t *rtc)
****************************************************************************************************
* Argumentos I/P: rtc_t *: Puntero a estructura de tipo rtc_t. La estructura contiene hora, min, seg, día, fecha, mes y año
 * Valor de retorno: ninguno
 * descripción: esta función se utiliza para obtener la fecha (d, m, a) de Ds1307 RTC.
    Nota: La lectura de fecha y hora de Ds1307 será de formato BCD,
          como 0x12,0x39,0x26 para 12 horas, 39 minutos y 26 segundos.
               0x15,0x08,0x47 para el día 15, el mes 8 y el año 47.              
***************************************************************************************************/
void RTC_GetDateTime()
{
    I2C_Start();                            // Start I2C communication

    I2C_Write(Ds1307WriteMode);        // connect to DS1307 by sending its ID on I2c Bus
    I2C_Write(Ds1307SecondRegAddress); // Request Sec RAM address at 00H

    I2C_Stop();                                // Stop I2C communication after selecting Sec Register

    I2C_Start();                            // Start I2C communication
    I2C_Write(Ds1307ReadMode);            // connect to DS1307(Read mode) by sending its ID

    sec = I2C_Read(1);                // read second and return Positive ACK
    min = I2C_Read(1);                 // read minute and return Positive ACK
    hour= I2C_Read(1);               // read hour and return Negative/No ACK
    weekDay = I2C_Read(1);           // read weekDay and return Positive ACK
    date= I2C_Read(1);              // read Date and return Positive ACK
    month=I2C_Read(1);            // read Month and return Positive ACK
    year =I2C_Read(0);             // read Year and return Negative/No ACK

    I2C_Stop();                              // Stop I2C communication after reading the Date
	
	  actualizar_hora();
}

/***************************************************************************************************
                    void RTC_SetDateTime(rtc_t *rtc)
****************************************************************************************************
* Argumentos I/P: rtc_t *: Puntero a estructura de tipo rtc_t. La estructura contiene hora, min, seg, día, fecha, mes y año
 * Valor de retorno: ninguno
 * descripción: esta función se utiliza para actualizar la fecha y la hora de Ds1307 RTC.
                 La nueva fecha y hora se actualizarán en la memoria no volátil de Ds1307.
        Nota: La fecha y la hora deben estar en formato BCD,
             como 0x12,0x39,0x26 para 12 horas, 39 minutos y 26 segundos.
                  0x15,0x08,0x47 para el día 15, el mes 8 y el año 47.                
***************************************************************************************************/
void RTC_SetDateTime()
{
    I2C_Start();                          // Start I2C communication

    I2C_Write(Ds1307WriteMode);      // connect to DS1307 by sending its ID on I2c Bus
    I2C_Write(Ds1307SecondRegAddress); // Request sec RAM address at 00H

    I2C_Write(sec);                    // Write sec from RAM address 00H
    I2C_Write(min);                    // Write min from RAM address 01H
    I2C_Write(hour);                    // Write hour from RAM address 02H
    I2C_Write(weekDay);                // Write weekDay on RAM address 03H
    I2C_Write(date);                    // Write date on RAM address 04H
    I2C_Write(month);                    // Write month on RAM address 05H
    I2C_Write(year);                    // Write year on RAM address 06h

    I2C_Stop();                              // Stop I2C communication after Setting the Date
}

/***************************************************************************************************
                         void RTC_Init(void)
****************************************************************************************************
 * I/P Arguments: none.
 * Return value    : none
 * description :This function is used to Initialize the Ds1307 RTC.
***************************************************************************************************/
void RTC_Init(void)
{
    I2C_Init();                             // Initialize the I2c module.
    I2C_Start();                            // Start I2C communication

    I2C_Write(Ds1307WriteMode);        // Connect to DS1307 by sending its ID on I2c Bus
    I2C_Write(Ds1307ControlRegAddress);// Select the Ds1307 ControlRegister to configure Ds1307

    I2C_Write(0x00);                        // Write 0x00 to Control register to disable SQW-Out

    I2C_Stop();                             // Stop I2C communication after initializing DS1307
}

//Initialization function
void Init_DS18B20(void)
{
    unsigned char x=0;
    DQ = 1;    //DQ reset
    delay(8);  //Slight delay
    DQ = 0;    //SCM will be pulled down DQ
    delay(80); //Accurate than 480us delay
    DQ = 1;    //Pulled the bus
    delay(14);
    x=DQ;      //After slight delay is initialized if x = 0 x = 1 is initialized successfully defeat
    delay(20);
}
 
//Reading a byte
unsigned char ReadOneChar(void)
{
    unsigned char i=0;
    unsigned char dat = 0;
    for (i=8;i>0;i--)
    {
      DQ = 0; // To the pulse signal
      dat>>=1;
      DQ = 1; // To the pulse signal
      if(DQ)
      dat|=0x80;		
      delay(4);
    }
    return(dat);
}
 
//Write a byte
void WriteOneChar(unsigned char dat)
{
    unsigned char i=0;
    for (i=8; i>0; i--)
    {
      DQ = 0;
      DQ = dat&0x01;
      delay(5);
      DQ = 1;
      dat>>=1;
    }
    delay(4);	
}
 
//Read temperature
void  ReadTemperature(void)
{
    Init_DS18B20();
    WriteOneChar(0xCC); // Skip read serial number column number of operations
    WriteOneChar(0x44); // Start temperature conversion
		DELAY_ms(750);
    Init_DS18B20();
    WriteOneChar(0xCC); //Skip read serial number column number of operations
    WriteOneChar(0xBE); //Read the temperature register, etc. (a total of 9 registers readable) is the temperature of the first two
    readdata[0]=ReadOneChar();
    readdata[1]=ReadOneChar();   
}
void Tempprocess() //Temperature Conversion, solo se procesa la temp positiva
{
    unsigned int t;
    float tt;
    unsigned char temp;
	
				t=readdata[1];
        t<<=8;
        t=t|readdata[0];
        t>>=4;
        word1[6]=((t/10)%10)+48;
        word1[7]=t%10+48;
        temp=readdata[0];
        temp=temp&0x0f;
        tt=temp*0.0625;
        word1[8]='.';
        word1[9]=(unsigned char )(tt*10);
        word1[10]=(unsigned char )(tt*100-word1[9]*10);
        word1[9]+=48;
        word1[10]+=48;
				//word1[11]=' ';
				word1[11]=0xdf;  // 11011111 simbolo de grados
				word1[12]=' ';
				word1[13]=' ';
				word1[14]=' ';
				word1[15]=' ';
				word1[16]=' ';
				word1[17]=' ';
			}

// función que muestra hora en el display de 7 segmentos			
void mostrar_hora(int intensidad)
{
						P0 = d_hor;
						D1 = on;
						delay_micros(intensidad);
						D1 = off;
						
						P0 = u_hor;
						D2 = on;
						delay_micros(intensidad);						
						D2 = off;
						
						P0 = d_min;
						D3 = on;
						delay_micros(intensidad);				
						D3 = off;
						
						P0 = u_min ;
						D4 = on;
						delay_micros(intensidad);			
						D4 = off; 
}

void beep (void)
	{
		buzer = on;					// prendo buzzer
		DELAY_ms(10);
		buzer = off;					// apago buzzer
}	

/* local function to generate delay */
void delay_micros(int cnt)
{
    int i;
    for(i=0;i<cnt;i++);
}

// funcion para mostrar hora en LCD
void reloj (void)
{
	RTC_GetDateTime();
	
	LCD_Comando(0xC0);  //Mover el cursor al comienzo de la segunda línea
	LCD_Dato(d_hor+48);  // se suma 48 para convertir los numeros en ascii
	LCD_Dato(u_hor+48);		// y poderlos escribir en el LCD
	LCD_Dato(':');
	LCD_Dato(d_min+48);
	LCD_Dato(u_min+48);
	LCD_Dato(':');
	LCD_Dato(d_seg+48);
	LCD_Dato(u_seg+48);
	
	LCD_Dato(' ');
	LCD_Dato(' ');
	
	LCD_Dato(d_dia+48);  // se suma 48 para convertir los numeros en ascii
	LCD_Dato(u_dia+48);		// y poderlos escribir en el LCD
	LCD_Dato('/');
	LCD_Dato(d_mes+48);
	LCD_Dato(u_mes+48);
	LCD_Dato('/');
	LCD_Dato(d_anio+48);
	LCD_Dato(u_anio+48);
	
	LCD_Comando(0x80); // Mover el cursor al comienzo de la primera línea
}

void teclado(void)
{
    c1=c2=c3=c4=1;
    r1=0;r2=1;r3=1;r4=1;
    if(c1==0){
        while(c1==0);
        tecla = '1';
				beep ();
    } else if(c2==0) {
        while(c2==0);
        tecla = '2';
				beep ();
    } else if(c3==0) {
        while(c3==0);
        tecla= '3';
				beep ();
    } else if(c4==0) {
        while(c4==0);
				beep ();
        tecla = 'A';
    }

    r1=1;r2=0;r3=1;r4=1;
    if(c1==0){
        while(c1==0);
        tecla = '4';
				beep ();
    } else if(c2==0) {
        while(c2==0);
        tecla = '5';
				beep ();
    } else if(c3==0) {
        while(c3==0);
        tecla ='6';
				beep ();
    } else if(c4==0) {
        while(c4==0);
				beep ();
        tecla ='B';
    }

    r1=1;r2=1;r3=0;r4=1;
    if(c1==0){
        while(c1==0);
        tecla = '7';
				beep ();
    } else if(c2==0) {
        while(c2==0);
        tecla = '8';
				beep ();
    } else if(c3==0) {
        while(c3==0);
        tecla = '9';
				beep ();
    } else if(c4==0) {
        while(c4==0);
         tecla = 'C';
				 beep ();
    }

    r1=1;r2=1;r3=1;r4=0;
    if(c1==0){
        while(c1==0);
        tecla ='*';  
				beep ();
    } else if(c2==0) {					
        while(c2==0);
        tecla ='0';
				beep ();
    } else if(c3==0) {
        while(c3==0);
        tecla ='#';
				beep ();
    } else if(c4==0) {
        while(c4==0);
        tecla = 'D';
				beep ();
    }
	
}

/***************************************************************************************************
                         void DELAY_us(uint16_t us_count)
****************************************************************************************************
 * I/P Arguments: uint16_t.
 * Return value    : none

 *  descripción:
         Esta función se utiliza para generar retraso en microsegundos.
         Genera un retraso de aproximadamente 10us por cada conteo,
         si se pasa 5000 como argumento, genera un retraso de 50 ms.         

***************************************************************************************************/
void DELAY_us(uint16_t us_count)
 {    
    while(us_count!=0)
      {
         us_count--;
       }
 }

/***********************************************************************************************************************************
 *** FUNCIONES GLOBALES AL MODULO
 **********************************************************************************************************************************/

// inicializo timers, puertos e interrupciones
void Inicio_Timer_interrupt(void)
{
	  buzer = off;					// apago buzzer
	  TMOD = 0x11;		// Timer 0 y 1 en modo 1, contador de 16 bits
	  TH0 = 0xF0;   // Load timer value for 50ms
    TL0 = 0x00;
    ET0 = 1;      // Enable Timer0 Interrupt
    TR0 = on;      // Enable Timer0 Interrupt
		TH1  = 0X4B;         // Load the timer value for 50ms
    TL1  = 0XFD;
		ET1 = 1;      // Enable Timer1 Interrupt
    TR1 = 1;      // Enable Timer1 Interrupt	
    EA  = 1;      // Enable Global Interrupt bit
}

// interrupcion de timer 1
void timer1_isr() interrupt 3
{  
	  TR1 = off;
		TH1  = 0xEA;         // ReLoad the timer value for 10ms
    TL1  = 0xC7;
		TR1 = on;
		teclado();
	
		mostrar_hora(10);                   //MUESTRA HORA EN DISPLAY 7 SEG
	          /*
						P0 = d_hor;
						D1 = on;
						delay_micros(20);
						D1 = off;
						
						P0 = u_hor;
						D2 = on;
						delay_micros(20);						
						D2 = off;
						
						P0 = d_min;
						D3 = on;
						delay_micros(20);				
						D3 = off;
						
						P0 = u_min ;
						D4 = on;
						delay_micros(20);			
						D4 = off; 
						
		// muestra la hora en el display de 7 segmentos
		  delay_micros(2);
	    switch (j) {
			  case 1 :
						D4 = off;
						P0 = d_hor;
						D1 = on;
						j = 2;
						break;
				case 2 :
						D1 = off;
						P0 = u_hor;
						D2 = on;
						j = 3;
						break;
				case 3 :
						D2 = off;
						P0 = d_min;
						D3 = on;
						j = 4;
						break;
				case 4 :
						D3 = off;
						P0 = u_min ;
						D4 = on;
						j = 1;
						break;
			
			}
			*/
															// count for 1sec delay(50msx20 = 1000ms = 1sec)
    if(timer1Count >= 150)     // count for 1sec delay(10msx100 = 1000ms = 1sec)
    {
        timer1Count = 0;		
			 
	      /*
			  if (i<7)
				{
				mascara = ~(mascara << 1);
				P0 = mascara;
				mascara = ~ mascara;
			  i++;
				}
				else
				{
				i = 0;
				mascara = 0x01;
				P0 = 0xFE;
				}
				latch = 0;
				delay_micros(1);
				latch = 1;
			*/
			
			}	        			
    else
    {
        timer1Count++;
    }
		
}
					
// interrupcion de timer 0
void timer0_isr() interrupt 1
{  
		if(timer0Count == 14)
    {
        TR0 = off;  
			timer0Count = 0;    //  1.085useg x(65535-19453)x 20 = 1.085us * 921640 = 1 seg
        LED0 =!LED0;			//  Toggle the LEDs every 1sec	
				TH0 = 0xF0;   		// 14 * 65535 + 4095 = 921585 * 1.085 uSEG = 1 SEG
        TL0 = 0x00;
				TR0 = on;
			
				reloj();			//muestra hora en LCD
					
			/*
				u_seg++;
				if(u_seg == 10){
						u_seg=0;
						d_seg++;
						if (d_seg == 6){
								d_seg=0;
								u_min++;
								if(u_min == 10 ){
										u_min=0;
										d_min++;
										if( d_min ==6){
												d_min=0;
											  u_hor++;
												if ( u_hor == 4 && d_hor ==2 ){
														u_hor=0;
														d_hor=0;
												}
												else if (u_hor == 10 ){
														u_hor=0;
														d_hor++;
												}
				}}}}
				*/
// desplaza led encendido de derecha a izquierda cada 1 segundo				
			if (led_i<7)
				{
				mascara = ~(mascara << 1);	
				P1 = mascara;
				delay_micros(5);
				latch = on;	
				mascara = ~ mascara;
			  led_i++;
				}
				else
				{
				led_i = 0;
				mascara = 0x01;					
				P1 = 0xFE;
				delay_micros(5);	
				latch = on;	
				}	
				latch = off;
		}
    else
    {
    timer0Count++;
    }		
}

// FUNCION PARA MOSTRAR UNA CADENA DE CARACTERES EN EL LCD
void LCD_mostrar(char *puntero)
{
	    LCD_Comando(0x80); // Mover el cursor al comienzo de la primera línea
			DELAY_ms(1);
			while(*puntero){
				LCD_Dato(*puntero++);
			}
}

// Funcion para Inicilizar el LCD
void LCD_inicio ( void)
{
		  uint8_t i = 0;
	    while (i<3){
			LCD_RW=0;
			LCD_RS = 0; 		//RS=0
			LCD_EN = 0;			//E=0
			DELAY_ms(1);
			P1 = 0x38;    //funcion set :8 bit, 2 lineas, 5x8 dots
			LCD_EN = 1;			//E=1
			DELAY_ms(1);
			LCD_EN = 0;			//E=0
			DELAY_ms(1);
			LCD_EN = 1;			//E=1
			P1 = 0x01;			// clear display, borra LCD y poner direccion DDRAM a cero
			DELAY_ms(50);
			LCD_EN = 0;			//E=0
			DELAY_ms(1);
			LCD_EN = 1;			//E=1
			DELAY_ms(1);
			P1 = 0x0C;		// display on/off control, prende LCD y cursor parpadea
			DELAY_ms(50);
			LCD_EN = 0;			//E=0
			DELAY_ms(1);
			LCD_EN = 1;			//E=1
			DELAY_ms(1);
			P1 = 0x06;		// entry mode set: se incrementa dir y se corre a la deracha
			DELAY_ms(50);
			LCD_EN = 0;			//E=0
			delay_micros(20);
			i++;
		}
}		

/* Function to send the command to LCD */
void LCD_Comando( uint8_t cmd)
{
  LCD_RS=0;          // Select the Command Register by pulling RS LOW
  LCD_RW=0;          // Select the Write Operation  by pulling RW LOW
  LCD_EN=1;          // Send a High-to-Low Pusle at Enable Pin
  
	LcdDataBus=cmd;    // Send the command to LCD
	delay_micros(10);
	LCD_EN=0;  
  delay_micros(10);
}

/* Function to send the Data to LCD */
void LCD_Dato( uint8_t dat)
{
   LCD_RS=1;	      // Select the Data Register by pulling RS HIGH
   LCD_RW=0;          // Select the Write Operation by pulling RW LOW
   LCD_EN=1;	      // Send a High-to-Low Pusle at Enable Pin
	 LcdDataBus=dat;	  // Send the data to LCD
   delay_micros(10);
		LCD_EN=0;	     
		LCD_RS=0;	
		delay_micros(10);
}



/***************************************************************************************************
                         void DELAY_ms(uint16_t ms_count)
****************************************************************************************************
 * I/P Arguments: uint16_t.
 * Return value    : none

 * descripción:
     Esta función se utiliza para generar retardo en ms.
     Genera un retraso de 1 ms para cada conteo,
     si se pasa 1000 como argumento, genera un retraso de 1000 ms (1 segundo)
***************************************************************************************************/
/* Cuenta para generar un retraso de 1 miliseg en el reloj de 11.0592Mhz.
  El valor se obtiene al alternar los pines del puerto para generar una onda cuadrada de 2Kz */


void DELAY_ms(uint16_t ms_count)
 {
     while(ms_count!=0)
      {
         DELAY_us(112u);     //DELAY_us is called to generate 1ms delay
          ms_count--;
      }

 }

void actualizar_hora(void)  // el dato que pasa el RTC esta en bcd hh:mm:ss , hay que pasarlos a d_hor u_hor : d_min u_min
{
		d_hor = ( hour >> 4);        //desplazo a derecha e inserto ceros a la izquierda
	  u_hor = (hour & 0x0f) ;      //borro nibble mas alto
	  d_min = ( min >> 4);         //desplazo a derecha e inserto ceros a la izquierda
	  u_min = (min & 0x0f) ;       //borro nibble mas alto
	  d_seg = (sec >> 4);
		u_seg = (sec & 0x0f) ;
	
	  d_dia = ( date >> 4);        //desplazo a derecha e inserto ceros a la izquierda
	  u_dia = (date & 0x0f) ;      //borro nibble mas alto
	  d_mes = ( month >> 4);         //desplazo a derecha e inserto ceros a la izquierda
	  u_mes = (month & 0x0f) ;       //borro nibble mas alto
	  d_anio = (year >> 4);
		u_anio = (year & 0x0f) ;
}

int main() 
{ 
	  // inicializo timers ,interrupciones y puertos
	  Inicio_Timer_interrupt();
	 	
		// LCD en pines P3.5 RS , P3.7 RW , P3.6 EN , DATOS P1.0 A P1.7
    // LCD  2 x 20 lineas, 8 BIT
		LCD_inicio();		// inicializo con funcion en delay.h
		LCD_mostrar("   MI RELOJ EN .C   ");
	
/*
    RTC_Init();
    hour = 0x20; //  10:40:20 am
    min =  0x58;
    sec =  0x00;

    date = 0x25; //  18/01/2022
    month = 0x03;
    year = 0x22;
    weekDay = 5; // Friday: 5th day of week considering monday as first day. 2 es martes.

    /*Establezca la hora y la fecha solo una vez. Una vez configurada la Hora y la Fecha, comenta estas lineas
         y reflashear el código. De lo contrario, la hora se configurará cada vez que se reinicie el controlador*/
				 
    //RTC_SetDateTime();  //  10:40:20 am, 1st Jan 2016


    while(1)
		{		
			
			  DELAY_ms(500);
			  if (tecla != NULL){
						LCD_Comando(0xD3);        // voy a escribir en la direccion 53h de la DDRAM del LCD 
				    LCD_Dato(tecla);         	// 1 - 1010011 el primer 1 es para indicar direccion											
				}
			/*
				switch (tecla) {
					case 'C' :					//poner todo a cero
						borrar_reloj();
					  break;
					
				  case 'D' :
						tecla = NULL;				
						//actualizar_hora();						
						DELAY_ms(2000);
				    LCD_mostrar("   MI RELOJ EN .C   ");     // mensaje: MI PRIMER RELOJ EN .C						
						break;
         */
			
				DELAY_ms(2000);
				LCD_mostrar("   MI RELOJ EN .C   ");     // mensaje: MI PRIMER RELOJ EN .C	
				
				
				ReadTemperature();
        Tempprocess();
				
				DELAY_ms(2000);
				
				LCD_mostrar(word1);
				
	}
}				

