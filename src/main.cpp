/**

 */

#include <Arduino.h>
#include <LiquidCrystal.h>
#include <math.h>
#include <stdint.h>
#include "pid.h"

// status LED
static void set_led(bool b)
{
	pinMode(13, OUTPUT);
	digitalWrite(13, b);
}

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




#define ADC_VAL_OVERSAMPLE 64
#define ADC_VAL_MAX 1024
#define THERMISTOR_T0 298.15f // = 25 deg C
#define THERMISTOR_B  3950.0f
#define THERMISTOR_R0 100000.0f // = 100k
#define THERMISTOR_RP 4700.0f // pull up resistor = 4.7k
#define SET_POINT 160.0;
static float bot_set_point = 0;
static float top_set_point = 0;
static float bot_temp = 0;
static float top_temp = 0;
static float env_temp = 0;
#define ENV_TEMP_IDX 7
#define TOP_TEMP_IDX 6
#define NUM_BOTTOM_HEATERS 6
#define TOTAL_HEATER_TEMP_SENSORS (NUM_BOTTOM_HEATERS + 2)// +2 = for top&env temperature
float temps[TOTAL_HEATER_TEMP_SENSORS] = {0}; 

static void init_temps()
{
	bot_set_point = 0;
	top_set_point = 0;
	bot_temp = 0;
	top_temp = 0;
	for(auto &&x : temps) x = 0;
}

#define PANIC_TEMPERATURE(X) ((X) < 0  || (X) > 330) // immidiate panic temperature (thermister failure/open/short)
#define SUPRESS_TEMPERATURE(X) ((X) > 280) // temperature which needs heating suppression
//#define PANIC_TEMPERATURE(X) false // immidiate panic temperature (thermister failure/open/short)
//#define SUPRESS_TEMPERATURE(X) ((X) > 280) // temperature which needs heating suppression
#define TEMP_TARGETABLE_LOW 0 // temperature targetable range: low
#define TEMP_TARGETABLE_HIGH 220 // temperature targetable range: high
#define TEMP_MAX_BOTTOM_HEATER_DIFFERENCE 60 // allowed difference between most hot heater and most cold heater
#define ANY_HOT_TEMP 50 // warning temperature if any sensor is avobe this
static uint8_t heater_power = 0; // last heater power
static bool any_hot;

static pid_controller_t bot_pid(8, 20, 50, 0.95, 40, 0, 127);
static pid_controller_t top_pid(4, 20, 100, 0.95, 40, 0, 127);

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
			lcd.setCursor(0, line);
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
	static String last_msg;
	if(last_msg != n)
	{
		last_msg = n;
		Serial.print(n);
		Serial.print(F("\r\n\r\n"));
	}
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
	for(;;)
	{
		set_led(false);
		_delay_ms(300);
		set_led(true);
		_delay_ms(300);
	}
}

#if 0
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
#endif

// measure adc with no oversampling
static float measure_adc(int num)
{
	uint32_t accum = 0;
	for(int i = 0; i < 1; ++i)
	{
		accum += analogRead(num);
	}
	return (float)accum *
		(1.0 / ((float)1.0 * (float)ADC_VAL_MAX)) +
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

// bit reverset - 8bit
static uint8_t bit_reverse(uint8_t v)
{
	v = ((v & 0b11110000) >> 4) | ((v & 0b00001111) << 4); 
	v = ((v & 0b11001100) >> 2) | ((v & 0b00110011) << 2); 
	v = ((v & 0b10101010) >> 1) | ((v & 0b01010101) << 1);
	return v; 
}


// temperature management
static void manage_temp()
{
	EVERY_MS(1)
		// measure temperatures
		static uint8_t temp_counts = 0;

		for(uint8_t i = 0; i < TOTAL_HEATER_TEMP_SENSORS; ++i)
		{
			float t = adc_val_to_temp(measure_adc(i));
			temps[i] += t;
		}

		++temp_counts;
		if(temp_counts >= ADC_VAL_OVERSAMPLE)
		{
			temp_counts = 0;
			// all sensors are sufficiently measured
			any_hot = false;

			// check bottom heaters
			float heater_min = temps[0]* (1.0 / ADC_VAL_OVERSAMPLE);
			float heater_max = temps[0]* (1.0 / ADC_VAL_OVERSAMPLE);
			float heater_avg = 0;
			for(uint8_t i = 0; i < NUM_BOTTOM_HEATERS; ++i)
			{
				float tmp = temps[i];
				tmp *= (1.0 / ADC_VAL_OVERSAMPLE);
				if(PANIC_TEMPERATURE(tmp))
				{
					panic(String(F("Bot heater ")) + String((int)i));
				}

				heater_avg += tmp;
				if(heater_min > tmp) heater_min = tmp;
				if(heater_max < tmp) heater_max = tmp;
				if(tmp >= ANY_HOT_TEMP) any_hot = true;
				Serial.print(F("H"));
				Serial.print((int)i);
				Serial.print(':');
				Serial.print(tmp);
				Serial.print(' ');
			}
			heater_avg *= (1.0 / NUM_BOTTOM_HEATERS);

			// check if most hot heater is far from most cold heater
			if(heater_max - heater_min >= TEMP_MAX_BOTTOM_HEATER_DIFFERENCE)
				panic(F("Too much diffs"));

			// store bottom temprature
			bot_temp = heater_avg;

			// check top heaters
			float tmp;
			tmp = temps[TOP_TEMP_IDX];
			tmp *= (1.0 / ADC_VAL_OVERSAMPLE);
			if(PANIC_TEMPERATURE(tmp))
				panic(F("Top heater"));
			if(tmp >= ANY_HOT_TEMP) any_hot = true;

			// store top temperature
			top_temp = tmp;
			Serial.print(F("T"));
			Serial.print(':');
			Serial.print(tmp);
			Serial.print(' ');

			// check env temperature
			tmp = temps[ENV_TEMP_IDX];
			tmp *= (1.0 / ADC_VAL_OVERSAMPLE);
			if(PANIC_TEMPERATURE(tmp)) // TODO: check env temp limit
				panic(F("Env heater"));
			if(tmp >= ANY_HOT_TEMP) any_hot = true;

			// store env temperature
			env_temp = tmp;
			Serial.print(F("E"));
			Serial.print(':');
			Serial.print(tmp);
			Serial.print(F("\r\n"));

			// clear all accumurators
			for(auto &&x : temps) x = 0;

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
		}
	END_EVERY_MS

	// Do PWM
	if(bit_reverse(millis() %128)  < heater_power)
	{
		digitalWrite(10, HIGH);
		digitalWrite( 9, HIGH);	
	}
	else
	{
		digitalWrite(10, LOW);
		digitalWrite( 9, LOW);	
	}

	// set status led and enable fan if any sensor detected hot condition
	set_led(any_hot);
}

static void update_status_display()
{
	EVERY_MS(200)
		char buf[18*2];
		// first line:  B:XXX/XXX P:XXX
		// second line: T:XXX/XXX
		sprintf_P(buf, PSTR("B:%3d/%3d P:%3d\r\n" "T:%3d/%3d" ), (int)(bot_set_point+0.5f), (int)(bot_temp+0.5f), (int)heater_power , (int)(top_set_point+0.5f), (int)(top_temp+0.5f));
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

// program opecodes
#define PROG_END    0
#define PROG_DWELL  1
#define PROG_SET_BOT_TEMP 2
#define PROG_WAIT_BOT_TEMP 3
#define PROG_SET_TOP_TEMP 4
#define PROG_WAIT_TOP_TEMP 5

#define MAKE_PROGRAM_WORD(OP, ARG) (((uint32_t)(ARG)<<3) | (OP))
#define OPCODE_FROM_WORD(CODE) (uint8_t)((CODE)&0x07)
#define ARG_FROM_WORD(CODE) (uint32_t)((CODE)>>3)

#define TEMP_MATCH_MARGIN 1


static PROGMEM const uint32_t PROG1[] = {
//	MAKE_PROGRAM_WORD(PROG_SET_BOT_TEMP, 165),
//	MAKE_PROGRAM_WORD(PROG_DWELL,        60*60*2),
	MAKE_PROGRAM_WORD(PROG_SET_BOT_TEMP, 75),
	MAKE_PROGRAM_WORD(PROG_DWELL,        60*60*1),
	MAKE_PROGRAM_WORD(PROG_SET_BOT_TEMP, 165),
	MAKE_PROGRAM_WORD(PROG_DWELL,        60*60*1),
	MAKE_PROGRAM_WORD(PROG_SET_BOT_TEMP, 75),
	MAKE_PROGRAM_WORD(PROG_DWELL,        60*60*2),
	MAKE_PROGRAM_WORD(PROG_SET_BOT_TEMP, 165),
	MAKE_PROGRAM_WORD(PROG_DWELL,        60*60*1),
	MAKE_PROGRAM_WORD(PROG_SET_BOT_TEMP, 75),
	MAKE_PROGRAM_WORD(PROG_DWELL,        60*60*2),
	MAKE_PROGRAM_WORD(PROG_SET_BOT_TEMP, 165),
	MAKE_PROGRAM_WORD(PROG_DWELL,        60*60*1),
	MAKE_PROGRAM_WORD(PROG_END,          0)
};
static PROGMEM const uint32_t PROG2[] = {
	MAKE_PROGRAM_WORD(PROG_SET_BOT_TEMP, 45),
	MAKE_PROGRAM_WORD(PROG_DWELL,        10),
	MAKE_PROGRAM_WORD(PROG_SET_BOT_TEMP, 35),
	MAKE_PROGRAM_WORD(PROG_DWELL,        10),
	MAKE_PROGRAM_WORD(PROG_END,          0)
};

static bool handle_prog_keys()
{
	if(button_counts[BUTTON_OK] != 0)
	{
		button_counts[BUTTON_OK] = 0;
		return false;
	}

	update_status_display();

	return true;
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
start:
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

			if(m_ind == MENU_PROG1 || m_ind == MENU_PROG2)
			{
				static const uint32_t *prog = nullptr;
				if(m_ind == MENU_PROG1)
					prog = PROG1;
				else if(m_ind == MENU_PROG2)
					prog = PROG2;

				for(;;)
				{
					YIELD;
					static uint32_t current_op;
					current_op = pgm_read_dword(prog);
					static uint8_t opcode;
					opcode = OPCODE_FROM_WORD(current_op);

					if(opcode == PROG_END)
					{
						goto start;
					}
					else if(opcode == PROG_DWELL)
					{
						static uint32_t secs;
						secs = ARG_FROM_WORD(current_op);
						while(secs --)
						{
							YIELD;
							static uint32_t m;
							m = millis() + 1000;

							while((int32_t)(millis() - m) <= 0)
							{
								YIELD;
								if(!handle_prog_keys()) goto start;
							}
						}
					}
					else if(opcode == PROG_SET_BOT_TEMP)
					{
						bot_set_point = ARG_FROM_WORD(current_op);
					}
					else if(opcode == PROG_SET_TOP_TEMP)
					{
						top_set_point = ARG_FROM_WORD(current_op);
					}
					else if(opcode == PROG_WAIT_BOT_TEMP || opcode == PROG_WAIT_TOP_TEMP)
					{
						static int16_t temp;
						temp = ARG_FROM_WORD(current_op);
						for(;;)
						{
							YIELD;
							if(opcode == PROG_WAIT_BOT_TEMP)
							{
								if(bot_temp - TEMP_MATCH_MARGIN <= temp && temp <= bot_temp + TEMP_MATCH_MARGIN ) break;
							}
							else if(opcode == PROG_SET_TOP_TEMP)
							{
								if(top_temp - TEMP_MATCH_MARGIN <= temp && temp <= top_temp + TEMP_MATCH_MARGIN ) break;
							}
							
							if(!handle_prog_keys()) goto start;
						}
					}
					++ prog;
				}

				goto start;
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
