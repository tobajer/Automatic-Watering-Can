/*
 * Automatic Watering Can
 * main.c
 *
 * Created: 1/5/2025 8:06:56 PM
 *  Author: tomasz@bajraszewski.pl
 * 
 * Fuse settings:
 * ATtiny25/45/85
	 EXTENDED: 0xFF
	 HIGH: 0xD7
	 LOW: 0x62
 */ 

#define F_CPU 1000000UL

#include <xc.h>
#include <avr/io.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>
#include <util/delay.h>

/******************
Code compilation modes:
0 - automatic watering can (normal operation)
1 - period potentiometer calibration
2 - period potentiometer verification
3 - water volume potentiometer calibration
4 - water volume potentiometer verification
*******************/
#define CODE_MODE 0
/*****************/

// Pins definitions
#define LED_PIN PB1
#define PUMP_PIN PB3
#define ADC_PIN PB4
#define WATERING_PIN PB1
#define BUTTON_PIN PB0
// Some constants
#define WDT_MAX_COUNT 8
#define MAX_PERIOD_IN_HOUR 168

// initialize wdt_counter with highest value to run pump right after reset
volatile uint32_t wdt_counter = (uint32_t)(MAX_PERIOD_IN_HOUR + 1) * 60 * 60;

#define EE_PERIOD_BASE_ADDR 0	//location of first byte of 9 in EEPROM for watering pot adc points
#define EE_PERIOD_N 11
#define EE_WATERING_BASE_ADDR	(EE_PERIOD_BASE_ADDR + EE_PERIOD_N)	//location of first byte of 5 in EEPROM for watering pot adc points
#define EE_WATERING_N	7

/**/
const uint16_t target_p[EE_PERIOD_N] = {2, 6, 12, 18, 24, 48, 72, 96, 120, 144, 168};	//in hours
const uint16_t target_w[EE_WATERING_N] = {10, 50, 100, 150, 200, 250, 270};				//in millilitres


void init_adc_for_period(void) {
	ADMUX = (1 << MUX1) | (1 << ADLAR); // Vref = Vcc, ADC2 (PB4)
	ADCSRA = (1 << ADEN) | (1 << ADPS1) | (1 << ADPS0); // Enable ADC, prescaler 8
	_delay_ms(5);	//wait a while
}

void init_adc_for_watering(void) {
	ADMUX = (1 << MUX0) | (1 << ADLAR); // Vref = Vcc, ADC1 (PB2), left adj.
	ADCSRA = (1 << ADEN) | (1 << ADPS1) | (1 << ADPS0); // Enable ADC, prescaler 8
	_delay_ms(5);	//wait a while
}

void init_adc_for_battery_monitor(void) {
	
	ADMUX = (1 << REFS1) | (1 << MUX1) | (1 << MUX0); // Vref = 1.1V, ADC3 (PB3)
	ADCSRA = (1 << ADEN) | (1 << ADPS1) | (1 << ADPS0); // Enable ADC, prescaler 8
	_delay_ms(5);	//wait a while
}

uint8_t read_adch(void) {
	ADCSRA |= (1 << ADSC); // start conversion
	while (ADCSRA & (1 << ADSC)); // wait until end
	return ADCH;
}

uint16_t read_adc(void) {
	ADCSRA |= (1 << ADSC); // start conversion
	while (ADCSRA & (1 << ADSC)); // wait until end
	return ADCW;
}

inline void led_on(void){
	PORTB |= (1 << LED_PIN);
}

inline void led_off(void){
	PORTB &= ~(1 << LED_PIN);
}

void init_led(void) {
	DDRB |= (1 << LED_PIN); // 
	led_off();
}

void blink_led(uint8_t times) {
	for (uint8_t i = 0; i < times; i++) {
		led_on();
		_delay_ms(25);
		led_off();
		_delay_ms(100);
	}
}


void init_button(void)
{
    DDRB &= ~(1 << BUTTON_PIN); // Przycisk jako wejœcie
    PORTB |= (1 << BUTTON_PIN); // Pull-up na przycisk	
}

void wait_while_button_pressed_and_released(void)
{
	uint8_t cnt = 0;
	
	//wait on button pressed withn 5 samples (x20ms = 100ms)
	while(cnt < 5)
	{
		if(!(PINB & (1 << BUTTON_PIN)) )
		{
			cnt++;
		}
		else
		{
			cnt = 0;
		}
		_delay_ms(20);
	}

	cnt = 0;
	//wait until button released
	while(cnt < 5)
	{
		if(PINB & (1 << BUTTON_PIN))
		{
			cnt++;
		}
		else
		{
			cnt = 0;
		}
		_delay_ms(20);
	}
}

inline void init_wdt(void) {
	// WDT: set longest period (~8s), interrupt mode
	WDTCR = (1 << WDIE) | (1 << WDP3) | (1 << WDP0);
}

void init_pump_pin_as_output(void) {
	DDRB |= (1 << PUMP_PIN);
	PORTB &= ~(1 << PUMP_PIN);
}

void init_pump_pin_as_input(void) {
	DDRB &= ~(1 << PUMP_PIN); // pin as input for battery voltage read
	PORTB &= ~(1 << PUMP_PIN); // no pull-up, tri-state
}


uint16_t get_battery_level(void) {
	
	init_pump_pin_as_input();
	init_adc_for_battery_monitor();
	uint16_t adc_val = read_adc();	
	/*
	deltaAdc = 1100mV/1024 = 1.074218mV/bin
	Voltage divider coeff. with R1=100k|R2=1M => 11
	ADC Coeff = 1.074218 mV/bin
	Total ADC-to-mv-coeff = 11 * 1.0742 = 11,8163 = ~12
	Vbatt = adc_value * 12 [mV] //error is -1.55% which is acceptable
	*/
	
	return adc_val * 12; // in mV
}

uint8_t get_period_adc_value(void)
{
	init_adc_for_period();
	return read_adch();
}

uint8_t get_water_adc_value(void)
{
	init_adc_for_watering();
	return read_adch();	
}

uint16_t get_water_volume_in_ml(void) {
	
	uint16_t adc_val = get_water_adc_value();
	uint16_t result = 0;

	uint8_t cal, cal_prev;

	cal_prev = 0;
	for (uint8_t i = 1; i < EE_WATERING_N; i++)
	{
		cal = eeprom_read_byte((uint8_t *)(i + EE_WATERING_BASE_ADDR));
		if (adc_val <= cal)
		{
			result = target_w[i - 1] + ((target_w[i] - target_w[i - 1]) * (adc_val - cal_prev)) / (cal - cal_prev);
			break;
		}
		cal_prev = cal;
	}
	//if result = 0, it means the last point on scale is set below max ADC=255,
	//hence, set result at max value
	if (result == 0)
		result = target_w[EE_WATERING_N - 1];
	
	return result;
}

uint16_t volume_to_pump_sec(uint16_t volume_ml, uint16_t Vcc_mV) {
	
	/*
		pump efficiency measured at different voltage:
		Vcc=3.0V --> 200ml @ 157 sec
		Vcc=3.6V --> 200ml @ 125 sec
		Vcc=4.1V --> 200ml @ 105 sec
		
		Calculcated coefficient sec/ml is 
		Vcc=3.0V --> 200ml / 157 sec = 0.79
		Vcc=3.6V --> 200ml / 125 sec = 0.63
		Vcc=4.2V --> 200ml / 105 sec = 0.53
		
		There is linear dependence of time T of 'pump on' versus Vcc:
		T = a * Vcc + b, where a = -0.2374, b = 1.4916
		Lets make a = -0.25 (i.e., -1/4) - it is easy to calculate
		THen, b = 1.55 gives closest timing to experiment for 200ml water volume.
		Since Vcc is in mV, so b=1550
		
		T(Vcc)[sec] = (1550 - 1/4 * Vcc) * water_volume_ml / 1000
	
		To fit into uint16 calculations (ROM saving) and simplify operations, calcualations are reduced by 32 (2^5)
		Hence, b/32 = 49, (Vcc/32)/4 = Vcc>>7, and result (reduced by 32) is divided finally by 32 corresponding to /1000 (actually 1024) 
	
		Residual error of pump periods is ca. +5% which is well acceptable
		
	*/
	
	uint16_t T;
	T = ((49 - (Vcc_mV >> 7)) * volume_ml) >> 5;	//
	
	return T;
}

uint16_t get_pump_period_in_hour(void) {
	
	uint16_t adc_val = get_period_adc_value();
	uint16_t result = 0;

	uint8_t cal, cal_prev;

	cal_prev = 0;

	for (uint8_t i = 1; i < EE_PERIOD_N; i++)
	{
		cal = eeprom_read_byte((uint8_t *)(i + EE_PERIOD_BASE_ADDR));
		if (adc_val <= cal)
		{
			result = target_p[i - 1] + ((target_p[i] - target_p[i - 1]) * (adc_val - cal_prev)) / (cal - cal_prev);
			break;
		}
		cal_prev = cal;
	}
	//if result = 0, it means the last point on scale is set below max ADC=255, 
	//hence, set result at max value
	if (result == 0)
		result = target_p[EE_PERIOD_N - 1];
	
	return result;

}

#define EE_PERIOD_BASE_ADDR 0	//location of first byte of 9 in EEPROM for watering pot adc points
#define EE_PERIOD_N 11
#define EE_WATERING_BASE_ADDR	(EE_PERIOD_BASE_ADDR + EE_PERIOD_N)	//location of first byte of 5 in EEPROM for watering pot adc points
#define EE_WATERING_N	7

void store_default_calibration_points_to_eeprom(void)
{
	uint8_t cal_p0 = eeprom_read_byte((uint8_t *)(EE_PERIOD_BASE_ADDR));
	uint8_t cal_w0 = eeprom_read_byte((uint8_t *)(EE_WATERING_BASE_ADDR));

	//default calibration points for ideal pot, corresponding to scale on designed label
	const uint8_t def_cal_p[]={0,35,64,96,128,159,186,202,217,231,255};
	const uint8_t def_cal_w[]={0,55,103,142,185,224,255};

	//when unprogrammed, both values should comprise 0xff
	if ( (cal_p0 == 0xFF) && (cal_w0 == 0xFF))
	{
		eeprom_update_block((const void*)def_cal_p, (void *)(EE_PERIOD_BASE_ADDR), EE_PERIOD_N);
		eeprom_update_block((const void*)def_cal_w, (void *)(EE_WATERING_BASE_ADDR), EE_WATERING_N);
	}
	blink_led(4);
}

//hear bit. WDT interrupt is triggered every ~8sec
ISR(WDT_vect) {
	wdt_counter++;
}

//main loop
int main(void) {

#if CODE_MODE == 0
	init_led();
	init_wdt();
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	const uint16_t battery_low_threshold1 = 2954;	//mV corresponding to real 3000mV considering -1.55% adcerr conversion to mV
	const uint16_t battery_low_threshold2 = 2806;	//mV corresponding to real 2850mV considering -1.55% adcerr conversion to mV
	const uint16_t battery_low_threshold3 = 2658;	//mV corresponding to real 2700mV considering -1.55% adcerr conversion to mV

	//if EEPROM is unprogrammed, it is burned with default calibrations 
	store_default_calibration_points_to_eeprom();

	sei(); // enable interrupts
    
	while (1) {

		// Indicate it is alive
		blink_led(1);

		// check battery voltage
		uint16_t battery_level = get_battery_level();
			
		if (battery_level  < battery_low_threshold1) {
			blink_led(1); // double-blink: battery at level 1
		}
		else if (battery_level  < battery_low_threshold2) {
			blink_led(2); // 3-blink: battery at level 2
		}
		else if (battery_level  < battery_low_threshold3) {
			blink_led(3); // 4-blink: battery at level 3 (it is really time to charge/change the battery)
		}

		uint32_t pump_period = (uint32_t)get_pump_period_in_hour() * 3600;
		
		//compensate inaccuracy of WDT 8 sec
		//in my chip, the period was longer by 12.5%
		pump_period -= (pump_period >> 3);
			
		// check if it is time to run pump
		if (wdt_counter >= (pump_period / WDT_MAX_COUNT)) {

			wdt_counter = 0; // cnt reset

			//get water volume based on potentiometer read
			uint16_t water_volume = get_water_volume_in_ml();

			// Turn on the pump
			led_on();
			_delay_ms(1000);	//let voltage stabilize under load
			//check voltage under load (by LED which gets 1-3mA, not so much but something)
			//would be better to measure under pump running, but it shares the same pin
			uint16_t vcc_under_load = get_battery_level();
			//convert volume to seconds of pump activations
			uint16_t watering_period = volume_to_pump_sec(water_volume, vcc_under_load);

			init_pump_pin_as_output();
			PORTB |= (1 << PUMP_PIN);

			//run pump for certain time
			for (uint16_t i = 0; i < watering_period; i++) {
				_delay_ms(1000);
			}
			
			//turn-off the pump
			PORTB &= ~(1 << PUMP_PIN);
			led_off();
			init_pump_pin_as_input();
		}
	
		// go to sleep
		ADCSRA = 0;	//disable ADC to save power during sleep
		sleep_mode();
	}

/* calibraton of period pot */
#elif CODE_MODE == 1
	uintptr_t ee_addr;
	init_led();
	init_button();

	ee_addr = EE_PERIOD_BASE_ADDR;

	led_on();
	_delay_ms(10000);
	led_off();

	for (uint8_t i=0; i<EE_PERIOD_N; i++)
	{
		wait_while_button_pressed_and_released();
		uint8_t adc_val = get_period_adc_value();
		eeprom_update_byte((uint8_t *)ee_addr, adc_val ); // Zapis do EEPROM
		blink_led(10);
		ee_addr++;	//incremet eeprom address to store next point
	}

	//finish
	while(1)
	{
		led_on();
		_delay_ms(1000);
		led_off();
		_delay_ms(1000);
	}

/* verification of period pot */
#elif CODE_MODE == 2
	init_led();
	while(1)
	{
		uint32_t pump_period = get_pump_period_in_hour();
		uint8_t x = 0;
		for (uint8_t i=0; i< EE_PERIOD_N; i++)
		{
			if(pump_period == target_p[i])
				x++;
		}
		(x > 0) ? led_on() : led_off();
	
	}

/* calibraton of watering pot */
#elif CODE_MODE == 3
	uintptr_t ee_addr;
	init_led();
	init_button();
	
	ee_addr = EE_WATERING_BASE_ADDR;

	led_on();
	_delay_ms(10000);
	led_off();

	for (uint8_t i=0; i<EE_WATERING_N; i++)
	{
		wait_while_button_pressed_and_released();
		uint8_t adc_val = get_water_adc_value();
		eeprom_update_byte((uint8_t *)ee_addr, adc_val ); // Zapis do EEPROM
		blink_led(10);
		ee_addr++;	//incremet eeprom address to store next point
	}	

	//finish
	while(1)
	{
		led_on();
		_delay_ms(1000);
		led_off();
		_delay_ms(1000);
	}

/* verification of watering pot */
#elif CODE_MODE == 4
	init_led();
	while(1)
	{
		uint32_t water_volume = get_water_volume_in_ml();
		uint8_t x = 0;
		for (uint8_t i=0; i < EE_WATERING_N; i++)
		{
			if(water_volume == target_w[i])
			x++;
		}
		(x > 0) ? led_on() : led_off();
		
	}

#endif

}