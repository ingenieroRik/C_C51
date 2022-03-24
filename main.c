/***************************************************************************************************
   23/3/22 comienzo de seguimiento con git                                         
****************************************************************************************************
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

#define DS18B20_SKIP_ROM  0xCC
#define DS18B20_CONVERT_T 0x44
#define DS18B20_READ_SCRATCHPAD 0xBE

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
uint8_t tecla ;


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
void borrar_reloj(void);
void actualizar_hora(void);

/**************************************************************************************************/
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


///////////////ds18b20/////////////////////
//Delay function
 
void delay(unsigned int i)
{
    while(i--);
}
 
//Initialization function
void Init_DS18B20(void)
{
    unsigned char x=0;
    DQ = 1;    //DQ reset
		//DELAY_us(70);
    delay(8);  //Slight delay
    DQ = 0;    //SCM will be pulled down DQ
		//DELAY_us(480);
    delay(80); //Accurate than 480us delay
    DQ = 1;    //Pulled the bus
		//DELAY_us(410);
    delay(14);
    x=DQ;      //After slight delay is initialized if x = 0 x = 1 is initialized successfully defeat
		//DELAY_us(120);
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
			//DELAY_us(24);			
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
			//DELAY_us(24);
      delay(5);
      DQ = 1;
      dat>>=1;
    }
    delay(4);
		//DELAY_us(24);	
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
				word1[11]=' ';
				word1[12]=0xdf;  // 11011111 simbolo de grados
				word1[13]=' ';
				word1[14]=' ';
				word1[15]=' ';
				word1[16]=' ';
				word1[17]=' ';
				word1[18]=' ';
	
	
	
	
	/*
    if((readdata[1]&0x80)!=0)			//TEMP NEGATIVA
    {
			  
        t=readdata[1];
        t<<=8;
        t=t|readdata[0];
        t=t-1;
        t=~t;
        t>>=4;
        word1[6]=((t/10)%10)+48;
        word1[7]=t%10+48;
        temp=readdata[0];
        temp=temp-1;
        temp=~temp;
        temp=temp&0x0f;
        tt=temp*0.0625;
        word1[8]='.';
        word1[9]=(unsigned char )(tt*10);
        word1[10]=(unsigned char )(tt*100-word1[9]*10);
        word1[9]+=48;
        word1[10]+=48;
				word1[11]=' ';
				word1[12]=0xdf;  // 11011111 simbolo de grados
				word1[13]=' ';
				word1[14]=' ';
				word1[15]=' ';
				word1[16]=' ';
				word1[17]=' ';
				word1[18]=' ';
				
    }
    else				//TEMP POSITIVA
    {
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
				word1[11]=' ';
				word1[12]=0xdf;  // 11011111 simbolo de grados
				word1[13]=' ';
				word1[14]=' ';
				word1[15]=' ';
				word1[16]=' ';
				word1[17]=' ';
				word1[18]=' ';
    }
		*/
}
/////////////////////end  of ds18b20//////////////////



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
	LCD_Comando(0xC0);  //Mover el cursor al comienzo de la segunda línea
	LCD_Dato(d_hor+48);  // se suma 48 para convertir los numeros en ascii
	LCD_Dato(u_hor+48);		// y poderlos escribir en el LCD
	LCD_Dato(':');
	LCD_Dato(d_min+48);
	LCD_Dato(u_min+48);
	LCD_Dato(':');
	LCD_Dato(d_seg+48);
	LCD_Dato(u_seg+48);
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
    //TH1  = 0X4B;         // ReLoad the timer value for 50ms
    //TL1  = 0XFD;
		TH1  = 0xEA;         // ReLoad the timer value for 10ms
    TL1  = 0xC7;
		//TH1  = 0XB8;         // ReLoad the timer value for 20ms
    //TL1  = 0X08;
		TR1 = on;
		teclado();
	
		mostrar_hora(10);
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
			
				reloj();
					
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




int main() 
{ 
	  // inicializo timers ,interrupciones y puertos
	  Inicio_Timer_interrupt();
	 	
		// LCD en pines P3.5 RS , P3.7 RW , P3.6 EN , DATOS P1.0 A P1.7
    // LCD  2 x 20 lineas, 8 BIT
		LCD_inicio();		// inicializo con funcion en delay.h
		LCD_mostrar("   MI RELOJ EN .C   ");
	     
    while(1)
		{		
			  DELAY_ms(500);
			  if (tecla != NULL){
						LCD_Comando(0xD3);        // voy a escribir en la direccion 53h de la DDRAM del LCD 
				    LCD_Dato(tecla);         	// 1 - 1010011 el primer 1 es para indicar direccion											
				}
				switch (tecla) {
					case 'C' :					//poner todo a cero
						borrar_reloj();
					  break;
					
				  case 'D' :
						tecla = NULL;				
						actualizar_hora();						
						DELAY_ms(2000);
				    LCD_mostrar("   MI RELOJ EN .C   ");     // mensaje: MI PRIMER RELOJ EN .C						
						break;				
				}
				
				ReadTemperature();
        Tempprocess();
				
				DELAY_ms(500);
				
				LCD_mostrar(word1);
				
	}
}				



void borrar_reloj(void) {
						u_seg = 0;
						d_min = 0;
					  u_hor = 0;
						d_seg = 0;
						u_min = 0;
						d_hor = 0;
						tecla = NULL;
}			

void actualizar_hora(void) {
		while (tecla == NULL){
								LCD_mostrar("poner minutos       ");
						}
						while (tecla > '9'){
								LCD_mostrar("numero incorrecto   ");							
						}
						u_min = (tecla - 48);		// para pasar de ascii a numero
						tecla = NULL;
						while (tecla == NULL){
								LCD_mostrar("poner dec de minutos");
						}
						while (tecla > '6'){
								LCD_mostrar("numero incorrecto   ");	
						}						
						d_min = (tecla - 48);		// para pasar de ascii a numero
						tecla = NULL;
						while (tecla == NULL){
						LCD_mostrar("poner horas         ");
						}
						while (tecla > '9'){
								LCD_mostrar("numero incorrecto   ");
						}
						u_hor = (tecla - 48);		// para pasar de ascii a numero
						tecla = NULL;
						while (tecla == NULL){
						LCD_mostrar("poner dec de horas  ");	
						}
						while (tecla >= '3'){
								LCD_mostrar("numero incorrecto   ");
						}
						d_hor = (tecla - 48);		  // para pasar de ascii a numero
						LCD_mostrar("hora actualizada    ");
						tecla = NULL;
					}	
				

				
	