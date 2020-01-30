/**

 */

#include <Arduino.h>
#include <LiquidCrystal.h>
#include <math.h>
#include <stdint.h>
#include "pid.h"

// LCD
#define LCD_COLS 16
#define LCD_LINES 2
LiquidCrystal lcd(11, 12, 5,6,7,8);


// polling timer
#define EVERY_MS(n) {\
	static uint32_t _next = 0; \
	uint32_t _cur = millis(); \
	if((int32_t)(_next - _cur) <= 0) { do {_next += n;} while(_next <= _cur);

#define END_EVERY_MS }}



// buttons
#define NUM_BUTTONS 3
static constexpr uint8_t button_io[] = {2, 3, 4};
static uint8_t button_counts[NUM_BUTTONS] = {0};
#define BUTTON_UP 0
#define BUTTON_DOWN 1
#define BUTTON_OK 2


// initialize buttons
static void init_buttons()
{
	for(uint8_t i = 0; i < NUM_BUTTONS; ++i)
	{
		pinMode(button_io[i], INPUT_PULLUP);
	}
}

// poll buttons
static uint8_t button_debounce_counter[NUM_BUTTONS] = {0};

#define BUTTON_DEBOUNCE_COUNT 4
#define BUTTON_INITIAL_REPEAT_DELAY 50
#define BUTTON_REPEAT_LIMIT 56


/**
 * button update handler called approx. 10ms
 */
static void button_update_handler()
{
	EVERY_MS(10)
		for(int i = 0; i < NUM_BUTTONS; i++)
		{
			if(!digitalRead(button_io[i]))
			{
				// physical button pressed
				uint8_t count = button_debounce_counter[i];
				count ++;
				if(count == BUTTON_DEBOUNCE_COUNT)
				{
					if(button_counts[i] < 255) button_counts[i] ++;
				}
				else if(count == BUTTON_REPEAT_LIMIT)
				{
					count = BUTTON_INITIAL_REPEAT_DELAY;
				}

				if(count == BUTTON_INITIAL_REPEAT_DELAY)
				{
					if(button_counts[i] < 255) button_counts[i] ++;
				}
				button_debounce_counter[i] = count;
			}
			else
			{
				// physical button released
				button_debounce_counter[i] = 0;
			}
		}
	END_EVERY_MS
}



// program opecodes
#define PROG_END    0
#define PROG_DWELL  1
#define PROG_SET_BOT_TEMP 2
#define PROG_WAIT_BOT_TEMP 3
#define PROG_SET_TOP_TEMP 4
#define PROG_WAIT_TOP_TEMP 5


#define ADC_VAL_OVERSAMPLE 64
#define ADC_VAL_MAX 1024
#define THERMISTOR_T0 298.15f // = 25 deg C
#define THERMISTOR_B  3950.0f
#define THERMISTOR_R0 100000.0f // = 100k
#define THERMISTOR_RP 4700.0f // pull up resistor = 4.7k
#define SET_POINT 160.0;
float bot_set_point = 0;
float top_set_point = 0;
float bot_temp = 0;
float top_temp = 0;

static void init_temps()
{
	bot_set_point = 0;
	top_set_point = 0;
	bot_temp = 0;
	top_temp = 0;
}

//#define PANIC_TEMPERATURE(X) ((X) < 0  || (X) > 330) // immidiate panic temperature (thermister failure/open/short)
//#define SUPRESS_TEMPERATURE(X) ((X) > 280) // temperature which needs heating suppression
#define PANIC_TEMPERATURE(X) false // immidiate panic temperature (thermister failure/open/short)
#define SUPRESS_TEMPERATURE(X) ((X) > 280) // temperature which needs heating suppression
#define TEMP_TARGETABLE_LOW 0 // temperature targetable range: low
#define TEMP_TARGETABLE_HIGH 220 // temperature targetable range: high
uint8_t heater_power = 0; // last heater power


static pid_controller_t bot_pid(8, 0.1, 10, 0.95, 40, 0, 127);
static pid_controller_t top_pid(4, 0.1, 40, 0.95, 40, 0, 127);

// output string to LCD
static void write_lcd(const char * p)
{
	lcd.setCursor(0, 0);
	uint8_t col = 0;
	uint8_t line = 0;
	while(*p)
	{
		if(*p == '\r')
		{
			/* do nothing */
		}
		else if(*p == '\n')
		{
			// new line
			while(col++ < LCD_COLS) lcd.write(' ');
			col = 0;
			++ line;
			if(line >= LCD_LINES) return;
		}
		else
		{
			lcd.write(*p);
			++col;
		}
		++p;
	}
	while(col++ < LCD_COLS) lcd.write(' ');
}

// display string to LCD/serial
static void display(const String &n)
{
	write_lcd(n.c_str());
	Serial.print(n);
	Serial.print(F("\r\n\r\n"));
}


// panic handler
static void panic(const String &n)
{
	pinMode(9, OUTPUT);
	pinMode(10, OUTPUT);
	digitalWrite(9, LOW); // disable heater
	digitalWrite(10, LOW); // disable heater
	display(String(F("!!!Panic!!!\r\n")) + n);
	Serial.flush();
	cli();
	for(;;) /**/ ;
}

// measure adc with oversampling
static float oversample_adc(int num)
{
	uint32_t accum = 0;
	for(int i = 0; i < ADC_VAL_OVERSAMPLE; ++i)
	{
		accum += analogRead(num);
	}
	return (float)accum *
		(1.0 / ((float)ADC_VAL_OVERSAMPLE * (float)ADC_VAL_MAX)) +
		(1.0/ADC_VAL_MAX/2.0);
}

// adc value to temperature in deg C
static float adc_val_to_temp(float adc_val_normalized)
{
	float r = - (THERMISTOR_RP * adc_val_normalized) / (adc_val_normalized - 1);

	float res = (THERMISTOR_T0 * THERMISTOR_B) /
		(THERMISTOR_T0 * log(r / THERMISTOR_R0) + THERMISTOR_B);
	return res - 273.15;
}



// temperature management
static void manage_temp()
{
	static uint8_t phase = 0;
	EVERY_MS(5)
		switch(phase)
		{
		case 0:
			// first, measure temperatures
			bot_temp = adc_val_to_temp(oversample_adc(0));
			if(PANIC_TEMPERATURE(bot_temp))
				panic(F("Bottom temp err"));
			break;

		case 1:
			top_temp = adc_val_to_temp(oversample_adc(1));
			if(PANIC_TEMPERATURE(top_temp))
				panic(F("Top temp err"));
			break;

		case 2:
			// update pid values
			bot_pid.set_set_point(bot_set_point);
			top_pid.set_set_point(top_set_point);

			// decide which temperature should to be reached
			uint8_t top_value, bot_value;
			top_value = (uint8_t)(int)top_pid.update(top_temp);
			bot_value = (uint8_t)(int)bot_pid.update(bot_temp);

			if(top_set_point > 0.0f)
			{
				// follow top set point
				heater_power = top_value;
			}
			else
			{
				// follow bottom set point
				heater_power = bot_value;
			}

			// needs suppression?
			if(SUPRESS_TEMPERATURE(bot_temp)||
				SUPRESS_TEMPERATURE(top_temp)
				)
			{
				heater_power = 0;
			}
			break;
		
		case 3:
		default:
			break;
		}
		++ phase;
		phase &= 3;
	END_EVERY_MS

	// Do PWM
	if(millis() %128  < heater_power)
	{
		digitalWrite(10, HIGH);
		digitalWrite( 9, HIGH);	
	}
	else
	{
		digitalWrite(10, LOW);
		digitalWrite( 9, LOW);	
	}
}

static void update_status_display()
{
	EVERY_MS(200)
		char buf[18*2];
		// first line:  B:XXX/XXX P:XXX
		// second line: T:XXX/XXX
		sprintf_P(buf, PSTR("B:%3d/%3d P:%3d\r\n" "T:%3d/%3d" ), (int)bot_set_point, (int)bot_temp, (int)heater_power , (int)top_set_point, (int)top_temp);
		display(buf);
	END_EVERY_MS
}

// simple continuation implementation
#define YIELD2(COUNTER) \
	do { \
	state = COUNTER; \
	return; \
	case COUNTER:; \
	} while(0)

#define YIELD YIELD2(__COUNTER__)


#define MENU_PROG1 0
#define MENU_PROG2 1
#define MENU_SET_BOT 2
#define MENU_SET_TOP 3

#define MAX_MENU_ITEM 5 // last menu item must be empty string
static String menu[MAX_MENU_ITEM];
static uint8_t menu_selected_index = 0;
static uint8_t menu_item_first_index = 0;

static void init_menu()
{
	for(uint8_t i = 0; i < MAX_MENU_ITEM; ++i) menu[i] = String();
	menu_selected_index = 0;
	menu_item_first_index = 0;
}

static void add_menu(const String &item)
{
	for(uint8_t i = 0; i < MAX_MENU_ITEM; ++i)
	{
		if(menu[i].length() == 0)
		{
			menu[i] = item;
			break;
		}
	}
}

static void justify_menu_show_range_up()
{
	if(menu_selected_index < menu_item_first_index)
		menu_item_first_index = menu_selected_index;
}
static void justify_menu_show_range_down()
{
	if(menu_item_first_index + LCD_LINES >= menu_selected_index)
		menu_item_first_index = menu_selected_index - LCD_LINES + 1;
}

#if 0 // unused
static void justify_menu_show_range()
{
	justify_menu_show_range_up();
	justify_menu_show_range_down();
}

static void set_selected_menu_index(uint8_t index)
{
	menu_selected_index = index;
	justify_menu_show_range();
}
#endif

static void show_menus()
{
	String lines;
	for(uint8_t i = 0; i < LCD_LINES; ++i)
	{
		if(i + menu_item_first_index == menu_selected_index) lines += '>'; else lines += ' ';
		lines += menu[i + menu_item_first_index] + F("\r\n");;
	}
	display(lines);
}

static void menu_handle_keys()
{
	// for only up/down
	while(button_counts[BUTTON_UP]--)
	{
		if(menu_selected_index > 0) --menu_selected_index;
		justify_menu_show_range_up();
	}
	button_counts[BUTTON_UP] = 0;
	while(button_counts[BUTTON_DOWN]--)
	{
		++menu_selected_index;
		if(menu[menu_selected_index].length() == 0) --menu_selected_index;
		justify_menu_show_range_down();
	}
	button_counts[BUTTON_DOWN] = 0;
}

static int16_t menu_temp = 0;
static void init_set_temp()
{
	menu_temp = TEMP_TARGETABLE_LOW;
}

static void handle_temp_keys()
{
	// for only up/down
	while(button_counts[BUTTON_UP]--)
	{
		if(menu_temp < TEMP_TARGETABLE_HIGH) ++menu_temp;
	}
	button_counts[BUTTON_UP] = 0;
	while(button_counts[BUTTON_DOWN]--)
	{
		if(menu_temp > TEMP_TARGETABLE_LOW) --menu_temp;
	}
	button_counts[BUTTON_DOWN] = 0;
}

static void show_temps(const String &n)
{
	String lines = n + F("\r\n") + String((int)menu_temp);
	display(lines);
}

static void set_menu_temp(uint8_t mode)
{
	if(mode == MENU_SET_BOT)
		menu_temp = bot_set_point;
	else if(mode == MENU_SET_TOP)
		menu_temp = top_set_point;
}

static void retarget_menu_temp(uint8_t mode)
{
	if(mode == MENU_SET_BOT)
		bot_set_point = menu_temp;
	else if(mode == MENU_SET_TOP)
		top_set_point = menu_temp;
}

static void handle_status_keys(uint8_t mode)
{
	set_menu_temp(mode);
	handle_temp_keys();
	retarget_menu_temp(mode);
}

static void ui_handler()
{
	static uint8_t state = 0;
	switch(state)
	{
	default:
		YIELD;

		for(;;)
		{
			// first, show main screen
			init_temps();
			init_menu();
			add_menu(F("prog 1")); // MENU_PROG1
			add_menu(F("prog 2")); // MENU_PROG2
			add_menu(F("Set bottom temp")); // MENU_SET_BOT
			add_menu(F("Set top temp")); // MENU_SET_TOP

			show_menus();
			YIELD;

			while(button_counts[BUTTON_OK] == 0)
			{
				menu_handle_keys();
				show_menus();
				YIELD;
			}
			button_counts[BUTTON_OK] = 0;
			static uint8_t m_ind;
			m_ind = menu_selected_index;

			if(m_ind == MENU_PROG1)
			{

			}
			else if(m_ind == MENU_PROG2)
			{
				
			}
			else if(m_ind == MENU_SET_BOT || m_ind == MENU_SET_TOP)
			{
				// set bottom/top temp
				init_set_temp();

				set_menu_temp(m_ind);

				while(button_counts[BUTTON_OK] == 0)
				{
					handle_temp_keys();
					if(m_ind == MENU_SET_BOT)
						show_temps(F("Bottom temp:"));
					else if(m_ind == MENU_SET_TOP)
						show_temps(F("Top temp:"));
					YIELD;
				}
				button_counts[BUTTON_OK] = 0;

				retarget_menu_temp(m_ind);

				while(button_counts[BUTTON_OK] == 0)
				{
					handle_status_keys(m_ind);
					update_status_display();
					YIELD;
				}
				button_counts[BUTTON_OK] = 0;
			}
		}

		YIELD;
		state = 0;
	}
}

void setup() {
	// put your setup code here, to run once:
	init_buttons();
	Serial.begin(115200);
	pinMode(9, OUTPUT);
	pinMode(10, OUTPUT);
	lcd.begin(LCD_COLS, LCD_LINES);

	display(F("welcome\r\nyakiimo"));
}

void loop() {
  // put your main code here, to run repeatedly:
	manage_temp();
	button_update_handler();
	ui_handler();

	while(Serial.available() > 0)
	{
		switch(Serial.read())
		{
		case '8':
			if(button_counts[BUTTON_UP] < 255) ++button_counts[BUTTON_UP];
			break;
		case '2':
			if(button_counts[BUTTON_DOWN] < 255) ++button_counts[BUTTON_DOWN];
			break;
		case '5':
			if(button_counts[BUTTON_OK] < 255) ++button_counts[BUTTON_OK];
			break;
		default:;
		}
	}


}
