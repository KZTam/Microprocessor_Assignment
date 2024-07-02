/*
 * LCD_DISPLAY.c
 *
 * Created: 30/6/2024 9:23:38 AM
 * Author : KZ
 */ 

/*
*	DESCRIPTION:
*	This is a project about LCD interfacing with 4*4 keypad. 
*
*	PINOUT from ATmega32 to LCD and keypad:
*	PA5 = pin E of LCD
*	PA6 = pin RS of LCD
*	PA7 = error LED (red LED)
*	PB0 ~ PB7 = pin D0 ~ D7 LCD
*	PD0 ~ PD7 = pin from left to right of 4*4 keypad (normal facing at front)
*	GND = pin VSS, RW, K (or LED-) of LCD
*	5V = pin VDD, A (or LED+) of LCD
*	V0 (or VEE) = voltage divider of 10k ohm to 5V, 1k ohm to GND (can adjust the value to control the contrast)
*	
*	Reference link: https://circuitdigest.com/microcontroller-projects/keypad-interfacing-with-avr-atmega32
*/

#include <avr/io.h>

#define F_CPU 1000000	//must be defined at the start of program to enable the processor
#define E 5		//PA5
#define RS 6	//PA6

#include <util/delay.h>

typedef enum{
	SEND_COMMAND = 0,
	SEND_DATA
	}RS_t;
	
struct{
	int key;
	int keypressed;	
	}LCD;

/* prototype function */
void Init();
void write_LCD(unsigned char data, RS_t type);
void write_LCD_string(char *string_of_characters);
void read_keypad();
void reset_keypad();
void LCD_interface();

int main(void)
{
    /* Replace with your application code */
	Init();	
	
    while (1) 
    {
		if(PIND != 0x0F){	//execute when a button on keypad is pressed
			read_keypad();
			LCD_interface();
		}
    }
}

//Used for initialize LCD and ATmega32 parameters.
void Init(){
	DDRB = 0xFF;
	DDRA = 0xFF;
	DDRD = 0xFF;
	_delay_ms(1);
	PORTD = 0x00;
	_delay_ms(1);
	_delay_ms(50);
	LCD.key = 0;
	LCD.keypressed = 0;
	write_LCD(0x01,SEND_COMMAND);	//clear screen
	_delay_ms(50);
	write_LCD(0x38,SEND_COMMAND);	//set mode: 8 bit command data
	_delay_ms(50);
	write_LCD(0x0F,SEND_COMMAND);	//LCD screen on, cursor blink
	_delay_ms(50);
	write_LCD_string("PRESS A KEY");
//	_delay_ms(50);
	write_LCD(0xC0,SEND_COMMAND);	// "\n"
	reset_keypad();
	//DDRA = 0xF0;
	//_delay_ms(1);
	//PORTA = 0x0F;	//might have bug here
	//_delay_ms(1);
}

//Used for communicate with LCD device
void write_LCD(unsigned char data, RS_t type){
	PORTB = data;
	if(type == SEND_COMMAND){	//command
		PORTA &= ~(1<<RS);
	}
	else{	//data
		PORTA |= (1<<RS);
	}
	PORTA |= (1<<E);	//enable receive data or command
	_delay_ms(50);
	PORTA &= ~(1<<E);	//lock
	PORTB = 0x00;	
}

void write_LCD_string(char *string_of_characters){
	while(*string_of_characters > 0){
		write_LCD(*string_of_characters++,SEND_DATA);
	}
}

//Used to read keypad input data.
//NOTE: short press (about 10ms) will output character on LCD screen. Press and hold the button to output repeat character on LCD
void read_keypad(){
	_delay_ms(1);
	LCD.keypressed = ((PIND ^ 0xFF) & 0x0F);	//pin data (row)
	DDRD ^= 0xFF;
	_delay_ms(1);
	PORTD ^= 0xFF;
	_delay_ms(1);
	LCD.keypressed |= ((PIND ^ 0xFF) & 0xF0);	//row data (column)
}

void reset_keypad(){
	DDRD = 0xF0;
	_delay_ms(1);
	PORTD = 0x0F;
	_delay_ms(1);
}

//handles output data to LCD
void LCD_interface(){
	
	/*	keypad (normal facing front):
	*		1	2	3	A
	*		4	5	6	B
	*		7	8	9	C
	*		*	0	#	D
	*
	*	output to LCD based on LCD.keypressed value, LCD.keypressed: <column value (4 bit), row value (4 bit)>.
	*	example: 0x12 means column 1, row 2, means "4".
	*/
	switch (LCD.keypressed)	
	{
	case 0x11:
	write_LCD_string("1");
	LCD.key++;
	break;
	
	case 0x12:
	write_LCD_string("4");
	LCD.key++;
	break;
	
	case 0x14:
	write_LCD_string("7");
	LCD.key++;
	break;
	
	case 0x18:
	write_LCD_string("*");
	LCD.key++;
	break;
	
	case 0x21:
	write_LCD_string("2");
	LCD.key++;
	break;
	
	case 0x22:
	write_LCD_string("5");
	LCD.key++;
	break;
	
	case 0x24:
	write_LCD_string("8");
	LCD.key++;
	break;
	
	case 0x28:
	write_LCD_string("0");
	LCD.key++;
	break;
	
	case 0x41:
	write_LCD_string("3");
	LCD.key++;
	break;
	
	case 0x42:
	write_LCD_string("6");
	LCD.key++;
	break;
	
	case 0x44:
	write_LCD_string("9");
	LCD.key++;
	break;
	
	case 0x48:
	write_LCD_string("#");
	LCD.key++;
	break;
	
	case 0x81:
	write_LCD_string("A");
	LCD.key++;
	break;
	
	case 0x82:
	write_LCD_string("B");
	LCD.key++;
	break;
	
	case 0x84:
	write_LCD_string("C");
	LCD.key++;
	break;
	
	case 0x88:
	write_LCD_string("D");
	LCD.key++;
	break;
	
	default:	//other cases are ignored. Here using a red LED to determine an invalid input.
	PORTA ^= (1<<7);
	break;
	}
	
	LCD.keypressed = 0x00;
	reset_keypad();
	
	//if LCD is full, clear the LCD.
	if(LCD.key > 16){
		write_LCD(0x01,SEND_COMMAND);	//clear screen
		_delay_ms(50);
		write_LCD_string("PRESS A KEY");
		_delay_ms(50);
		write_LCD(0xC0,SEND_COMMAND);	// "\n"
		LCD.key = 0;
	}
	
	//while((PINA & 0x0F) != 0x00);	//debounce
	_delay_ms(500);	//
}

