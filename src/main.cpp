/**

 */

#include <Arduino.h>
#include <LiquidCrystal.h>
#include <math.h>
#include <stdint.h>
#include "pid.h"
#include <TimerOne.h>

// pins
#define HEATER_PIN 9
#define TONE_PIN 11
#define FAN_STATUS_PIN 13

// status LED
static void set_led(bool b)
{
	pinMode(FAN_STATUS_PIN, OUTPUT);
	digitalWrite(FAN_STATUS_PIN, b);
}

// LCD
#define LCD_COLS 16
#define LCD_LINES 2
LiquidCrystal lcd(10, 12, 5,6,7,8);


// polling timer
#define EVERY_MS(n) {\
	static uint32_t _next = 0; \
	uint32_t _cur = millis(); \
	if((int32_t)(_next - _cur) <= 0) { do {_next += n;} while(_next <= _cur);

#define END_EVERY_MS }}



// buttons
#define NUM_BUTTONS 3
static constexpr uint8_t button_io[] = {2, 4, 3};
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
		button_counts[i] = 0;
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




#define ADC_VAL_OVERSAMPLE 256
#define ADC_VAL_MAX 1024
#define THERMISTOR_T0 298.15f // = 25 deg C
#define THERMISTOR_B  3950.0f
#define THERMISTOR_R0 100000.0f // = 100k
#define THERMISTOR_RP 4700.0f // pull up resistor = 4.7k
#define SET_POINT 160.0;
static float heater_set_point = 0;
static float air_set_point = 0;
static float heater_temp = 0;
static float air_temp = 0;
static float env_temp = 0;
#define ENV_TEMP_IDX 2
#define AIR_TEMP_IDX 1
#define NUM_HEATER_SENSORS 1
#define TOTAL_HEATER_TEMP_SENSORS (NUM_HEATER_SENSORS + 2)// +2 = for air&env temperature; so, sensors are: 0:heater 1:air 2:env
float temps[TOTAL_HEATER_TEMP_SENSORS] = {0}; 
static uint16_t temp_counts = 0;

static void init_temps()
{
	heater_set_point = 0;
	air_set_point = 0;
	heater_temp = 0;
	air_temp = 0;
	temp_counts = 0;	
	for(auto &&x : temps) x = 0;
}

#define PANIC_TEMPERATURE(X) ((X) < -2  || (X) > 1050) // immidiate panic temperature (thermister failure/open/short)
#define SUPRESS_TEMPERATURE(X) ((X) > 800) // temperature which needs heating suppression
//#define PANIC_TEMPERATURE(X) false // immidiate panic temperature (thermister failure/open/short)
//#define SUPRESS_TEMPERATURE(X) ((X) > 280) // temperature which needs heating suppression
#define TEMP_TARGETABLE_LOW 0 // temperature targetable range: low
#define TEMP_TARGETABLE_HIGH 400 // temperature targetable range: high
#define TEMP_MAX_HEATER_DIFFERENCE 180 // allowed difference between most hot heater and most cold heater
#define ANY_HOT_TEMP 50 // warning temperature if any sensor is avobe this
static float heater_power_target = 0; // heater power designated by PID controller
static volatile float heater_power = 0; // last heater power
static bool any_hot = false;
#define AIR_TEMP_LPF_COEFF 0.2 // air temperature IIR LPF coeff
#define HEATER_POWER_MAX 256
#define HEATER_POWER_INCREMENT 90
#define HEATER_POWER_DECREMENT 70

static pid_controller_t heater_pid(6, 1, 1200, 512, 1, 0.5, 40, 0, HEATER_POWER_MAX);
#define AIR_BASE_P 30
#define AIR_BASE_I 1
#define AIR_BASE_D 600
static pid_controller_t air_pid(AIR_BASE_P, AIR_BASE_I, AIR_BASE_D, 512, 1, 0.5, 40, 0, HEATER_POWER_MAX);
#define AIR_PID_PARAM_ADJUST \
	air_pid.kp = air_pid.setpoint >= 140 ? AIR_BASE_P * 1 : (air_pid.setpoint >= 80 ? AIR_BASE_P * 1 : AIR_BASE_P); \
	air_pid.ki = air_pid.setpoint >= 140 ? AIR_BASE_I * 1 : (air_pid.setpoint >= 80 ? AIR_BASE_I * 1 : AIR_BASE_I);

#define PID_SETPOINT_OFFSET 0.0


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
	pinMode(HEATER_PIN, OUTPUT);
	digitalWrite(HEATER_PIN, LOW); // disable heater
	display(String(F("!!!Panic!!!\r\n")) + n);
	Serial.flush();
	cli();
	for(;;)
	{
		digitalWrite(HEATER_PIN, LOW); // disable heater
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

		for(uint8_t i = 0; i < TOTAL_HEATER_TEMP_SENSORS; ++i)
		{
			float t = measure_adc(i);
			temps[i] += t;
		}

		++temp_counts;
		if(temp_counts >= ADC_VAL_OVERSAMPLE)
		{
			temp_counts = 0;
			// all sensors are sufficiently measured
			any_hot = false;

			// convert adc value to temperature
			for(auto && v : temps)
			{
				v = adc_val_to_temp(v*(1.0 / ADC_VAL_OVERSAMPLE));
			}

			// check heaters
			float heater_min = temps[0];
			float heater_max = temps[0];
			float heater_avg = 0;
			for(uint8_t i = 0; i < NUM_HEATER_SENSORS; ++i)
			{
				float tmp = temps[i];

				heater_avg += tmp;
				if(heater_min > tmp) heater_min = tmp;
				if(heater_max < tmp) heater_max = tmp;
				if(tmp >= ANY_HOT_TEMP) any_hot = true;
				Serial.print(F("H"));
				Serial.print((int)i);
				Serial.print(':');
				Serial.print(tmp);
				Serial.print(' ');
				if(PANIC_TEMPERATURE(tmp))
				{
					panic(String(F("Heater ")) + String((int)i));
				}
			}
			heater_avg *= (1.0 / NUM_HEATER_SENSORS);

			// check if most hot heater is far from most cold heater
			if(heater_max - heater_min >= TEMP_MAX_HEATER_DIFFERENCE)
				panic(F("Too much diffs"));

			// store heater temprature
			heater_temp = heater_avg;

			// check air heaters
			float tmp;
			tmp = temps[AIR_TEMP_IDX];
			if(tmp >= ANY_HOT_TEMP) any_hot = true;

			// store air temperature
			air_temp += (tmp - air_temp) * AIR_TEMP_LPF_COEFF;
			Serial.print(F("A"));
			Serial.print(':');
			Serial.print(tmp);
			Serial.print(' ');
			if(PANIC_TEMPERATURE(tmp))
				panic(F("Air"));

			// check env temperature
			tmp = temps[ENV_TEMP_IDX];
			if(tmp >= ANY_HOT_TEMP) any_hot = true;

			// store env temperature
			env_temp = tmp;
			Serial.print(F("E"));
			Serial.print(':');
			Serial.print(tmp);
			if(PANIC_TEMPERATURE(tmp)) // TODO: check env temp limit
				panic(F("Env"));

			// clear all accumurators
			for(auto &&x : temps) x = 0;

			// update pid values
			heater_pid.set_set_point(heater_set_point + PID_SETPOINT_OFFSET);
			air_pid.set_set_point(air_set_point + PID_SETPOINT_OFFSET);
			AIR_PID_PARAM_ADJUST;

			// decide which temperature should to be reached
			float air_value, heater_value;
			air_value = air_pid.update(air_temp);
			heater_value = heater_pid.update(heater_temp);

			if(air_set_point > 0.0f)
			{
				// follow air set point
				heater_power_target = air_value;
			}
			else
			{
				// follow heater set point
				heater_power_target = heater_value;
			}

			// needs suppression?
			if(SUPRESS_TEMPERATURE(heater_temp)||
				SUPRESS_TEMPERATURE(air_temp)
				)
			{
				heater_power_target = 0;
			}

			// accumulate heater power
			float hp = heater_power;
			if(hp < heater_power_target)
			{
					hp += HEATER_POWER_INCREMENT;
					if(hp > heater_power_target) hp = heater_power_target;
					if(hp > HEATER_POWER_MAX) hp = HEATER_POWER_MAX;
			}
			else if(hp > heater_power_target)
			{
					hp -= HEATER_POWER_DECREMENT;
					if(hp < heater_power_target) hp = heater_power_target;
					if(hp < 0) hp = 0;
			}

			// flag any_hot if any heater is on
			if(hp > 0) any_hot = true;

			// write heater_power
			noInterrupts();
			heater_power = hp;
			interrupts();

			// dump
			Serial.print(F(" P:"));
			Serial.print((int)heater_power_target);
			Serial.print(F("/"));
			Serial.print((int)hp);
			Serial.print(F("\r\n"));

			if(air_set_point > 0.0f)
				air_pid.dump();
			else
				heater_pid.dump();
		}
	END_EVERY_MS

	// set status led and enable fan if any sensor detected hot condition
	set_led(any_hot);
}


void timer1_handler(void)
{
	// DO PWM
	static uint8_t count;
	++count;
	static_assert(HEATER_POWER_MAX == 256, "heater power max must be 256");
	if((uint16_t)bit_reverse(count % HEATER_POWER_MAX) < (uint16_t)heater_power)
	{
		digitalWrite(HEATER_PIN, HIGH);
	}
	else
	{
		digitalWrite(HEATER_PIN, LOW);
	}
}

static int32_t secs_remain;

static void update_status_display(const String & status)
{
	EVERY_MS(500)
		char buf[18*2];
		// first line:  B:XXX/XXX P:XXX
		// second line: T:XXX/XXX 
		sprintf_P(buf, PSTR("H:%3d/%3d P:%3d\r\n" "A:%3d/%3d %s" ), (int)(heater_set_point+0.5f), (int)(heater_temp+0.5f), (int)heater_power , (int)(air_set_point+0.5f), (int)(air_temp+0.5f), status.c_str());
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
#define MENU_SET_HEATER 2
#define MENU_SET_AIR 3

#define MAX_MENU_ITEM 5 // last menu item must be empty string
static String menu[MAX_MENU_ITEM];
static uint8_t menu_selected_index = 0;
static uint8_t menu_item_first_index = 0;

static void init_menu()
{
	init_buttons();
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
	if(mode == MENU_SET_HEATER)
		menu_temp = heater_set_point;
	else if(mode == MENU_SET_AIR)
		menu_temp = air_set_point;
}

static void retarget_menu_temp(uint8_t mode)
{
	if(mode == MENU_SET_HEATER)
		heater_set_point = menu_temp;
	else if(mode == MENU_SET_AIR)
		air_set_point = menu_temp;
}

static void handle_status_keys(uint8_t mode)
{
	set_menu_temp(mode);
	handle_temp_keys();
	retarget_menu_temp(mode);
}


static uint32_t tone_pattern = 0;
static bool tone_repeat = false;
static uint8_t tone_position = 0; 
static void tone_handler()
{
	EVERY_MS(100)
	{
		if(tone_pattern & ((uint32_t)1<<tone_position))
		{
			tone(TONE_PIN, 2000);
		}
		else
		{
			noTone(TONE_PIN);	
		}
		
		++ tone_position;
		if(tone_position >= 32)
		{
			if(tone_repeat)
				tone_position = 0;
			else
				tone_position = 0, tone_pattern = 0;
		}
			
	}
	END_EVERY_MS
}

static void set_tone_pattern(uint32_t pattern, bool repeat)
{
	tone_pattern = pattern;
	tone_repeat = repeat;
	tone_position = 0;
}

// program opecodes
#define PROG_END    0
#define PROG_DWELL  1
#define PROG_SET_HEATER_TEMP 2
#define PROG_WAIT_HEATER_TEMP 3
#define PROG_SET_AIR_TEMP 4
#define PROG_WAIT_AIR_TEMP 5
#define PROG_WAIT_BUTTON 6
#define PROG_SET_TONE_REPEAT 7
#define PROG_SET_TONE 8

#define MAKE_PROGRAM_WORD(OP, ARG) (((uint32_t)(ARG)<<3) | (OP))
#define OPCODE_FROM_WORD(CODE) (uint8_t)((CODE)&0x07)
#define ARG_FROM_WORD(CODE) (uint32_t)((CODE)>>3)

#define TEMP_MATCH_MARGIN 1.5


static PROGMEM const uint32_t PROG1[] = {

	MAKE_PROGRAM_WORD(PROG_SET_AIR_TEMP, 117),
	MAKE_PROGRAM_WORD(PROG_WAIT_AIR_TEMP, 117),

//	MAKE_PROGRAM_WORD(PROG_SET_TONE_REPEAT,  0b11100011100011100011100000000),
//	MAKE_PROGRAM_WORD(PROG_WAIT_BUTTON,  0),
//	MAKE_PROGRAM_WORD(PROG_SET_TONE_REPEAT,  0),

	MAKE_PROGRAM_WORD(PROG_SET_AIR_TEMP, 160),
	MAKE_PROGRAM_WORD(PROG_WAIT_AIR_TEMP, 160),
	MAKE_PROGRAM_WORD(PROG_DWELL,        60*5),

	MAKE_PROGRAM_WORD(PROG_SET_AIR_TEMP, 73),
	MAKE_PROGRAM_WORD(PROG_DWELL,        60*60*1),
	MAKE_PROGRAM_WORD(PROG_SET_AIR_TEMP, 151),
	MAKE_PROGRAM_WORD(PROG_WAIT_AIR_TEMP, 151),
	MAKE_PROGRAM_WORD(PROG_DWELL,        60*60*0.9),
	MAKE_PROGRAM_WORD(PROG_SET_AIR_TEMP, 74),
	MAKE_PROGRAM_WORD(PROG_DWELL,        60*60*1),

	MAKE_PROGRAM_WORD(PROG_SET_AIR_TEMP, 152),
	MAKE_PROGRAM_WORD(PROG_WAIT_AIR_TEMP, 152),
	MAKE_PROGRAM_WORD(PROG_DWELL,        60*60*0.7),
	MAKE_PROGRAM_WORD(PROG_SET_AIR_TEMP, 75),
	MAKE_PROGRAM_WORD(PROG_DWELL,        60*60*1),

	MAKE_PROGRAM_WORD(PROG_SET_TONE_REPEAT,  0b1111111111111000000000000000),
	MAKE_PROGRAM_WORD(PROG_DWELL,        5),
	MAKE_PROGRAM_WORD(PROG_SET_TONE_REPEAT,  0),
	MAKE_PROGRAM_WORD(PROG_END,          0)
};
static PROGMEM const uint32_t PROG2[] = {
	MAKE_PROGRAM_WORD(PROG_SET_AIR_TEMP, 73),
	MAKE_PROGRAM_WORD(PROG_DWELL,        60*60*2),
	MAKE_PROGRAM_WORD(PROG_SET_AIR_TEMP, 151),
	MAKE_PROGRAM_WORD(PROG_WAIT_AIR_TEMP, 151),
	MAKE_PROGRAM_WORD(PROG_DWELL,        60*60*0.9),
	MAKE_PROGRAM_WORD(PROG_SET_AIR_TEMP, 74),
	MAKE_PROGRAM_WORD(PROG_DWELL,        60*60*2),

	MAKE_PROGRAM_WORD(PROG_SET_AIR_TEMP, 152),
	MAKE_PROGRAM_WORD(PROG_WAIT_AIR_TEMP, 152),
	MAKE_PROGRAM_WORD(PROG_DWELL,        60*60*0.8),
	MAKE_PROGRAM_WORD(PROG_SET_AIR_TEMP, 75),
	MAKE_PROGRAM_WORD(PROG_DWELL,        60*60*2),

	MAKE_PROGRAM_WORD(PROG_SET_TONE_REPEAT,  0b1111111111111000000000000000),
	MAKE_PROGRAM_WORD(PROG_DWELL,        5),
	MAKE_PROGRAM_WORD(PROG_SET_TONE_REPEAT,  0),
	MAKE_PROGRAM_WORD(PROG_END,          0)
};

#define CANCEL_BUTTON_COUNT 2
#define CANCEL_BUTTON_DURATION 1000

static bool handle_prog_keys()
{
	static uint32_t last_button_pressed;
	static uint8_t button_pressed_count;
	if(button_counts[BUTTON_OK] != 0)
	{
		button_counts[BUTTON_OK] = 0;
		if((int32_t)(millis() - last_button_pressed) <= CANCEL_BUTTON_DURATION)
		{
			++ button_pressed_count;
			if(button_pressed_count >= CANCEL_BUTTON_COUNT - 1) return false;
		}
		last_button_pressed = millis();
	}

	if((int32_t)(millis() - last_button_pressed) > CANCEL_BUTTON_DURATION)
	{
		button_pressed_count = 0;
		last_button_pressed = millis() - CANCEL_BUTTON_DURATION * 2; // always update the timestamp; to avoid wraparound
	}


	if(secs_remain != 0)
		update_status_display(String(secs_remain));
	else
		update_status_display(String(F("Busy")));

	return true;
}


static bool handle_wait_button_keys()
{
	if(button_counts[BUTTON_OK] != 0)
	{
		button_counts[BUTTON_OK] = 0;
		return false;
	}

	update_status_display(F("Hit OK"));

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
			add_menu(F("Start Yakiimo")); // MENU_PROG1
			add_menu(F("Test Program")); // MENU_PROG2
			add_menu(F("Set heater temp")); // MENU_SET_HEATER
			add_menu(F("Set air temp")); // MENU_SET_AIR

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
					secs_remain = 0;

					if(opcode == PROG_END)
					{
						goto start;
					}
					else if(opcode == PROG_DWELL)
					{
						secs_remain = ARG_FROM_WORD(current_op);
						while(secs_remain --)
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
					else if(opcode == PROG_SET_HEATER_TEMP)
					{
						heater_set_point = ARG_FROM_WORD(current_op);
					}
					else if(opcode == PROG_SET_AIR_TEMP)
					{
						air_set_point = ARG_FROM_WORD(current_op);
					}
					else if(opcode == PROG_WAIT_HEATER_TEMP || opcode == PROG_WAIT_AIR_TEMP)
					{
						static int16_t temp;
						temp = ARG_FROM_WORD(current_op);
						for(;;)
						{
							YIELD;
							if(opcode == PROG_WAIT_HEATER_TEMP)
							{
								if(heater_temp - TEMP_MATCH_MARGIN <= temp && temp <= heater_temp + TEMP_MATCH_MARGIN ) break;
							}
							else if(opcode == PROG_WAIT_AIR_TEMP)
							{
								if(air_temp - TEMP_MATCH_MARGIN <= temp && temp <= air_temp + TEMP_MATCH_MARGIN ) break;
							}
							
							if(!handle_prog_keys()) goto start;
						}
					}
					else if(opcode == PROG_WAIT_BUTTON)
					{
						for(;;)
						{
							YIELD;
							if(!handle_wait_button_keys()) break;
						}
					}
					else if(opcode == PROG_SET_TONE)
					{
						set_tone_pattern(ARG_FROM_WORD(current_op), false);
					}
					else if(opcode == PROG_SET_TONE_REPEAT)
					{
						set_tone_pattern(ARG_FROM_WORD(current_op), true);
					}

					++ prog;
				}

				goto start;
			}
			else if(m_ind == MENU_SET_HEATER || m_ind == MENU_SET_AIR)
			{
				// set heater/air temp
				init_set_temp();

				set_menu_temp(m_ind);

				while(button_counts[BUTTON_OK] == 0)
				{
					handle_temp_keys();
					if(m_ind == MENU_SET_HEATER)
						show_temps(F("Heater temp:"));
					else if(m_ind == MENU_SET_AIR)
						show_temps(F("Air temp:"));
					YIELD;
				}
				button_counts[BUTTON_OK] = 0;

				retarget_menu_temp(m_ind);

				while(button_counts[BUTTON_OK] == 0)
				{
					handle_status_keys(m_ind);
					update_status_display(String());
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
	pinMode(HEATER_PIN, OUTPUT);
	lcd.begin(LCD_COLS, LCD_LINES);
	Timer1.initialize(1000000 / 110);
	Timer1.attachInterrupt(timer1_handler);

	display(F("welcome\r\nyakiimo"));
}

void loop() {
  // put your main code here, to run repeatedly:
	manage_temp();
	button_update_handler();
	tone_handler();
	EVERY_MS(20)
		ui_handler();
	END_EVERY_MS

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
