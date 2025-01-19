/*
 * main.c
 *
 * Created: 1/5/2025 8:06:56 PM
 *  Author: tbajr
 */ 
#define F_CPU 600000UL

#include <xc.h>

#include <avr/io.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>
#include <util/delay.h>

/******************
Code mode:
0 - watering (normal operation)
1 - period potentiometer calibration
2 - water volume potentiometer calibration

*******************/
#define CODE_MODE 0

// Definicje pinów
#define LED_PIN PB1
#define PUMP_PIN PB3
#define ADC_PIN PB4
#define WATERING_PIN PB1

// Wartoœci liczników czasu
#define WDT_MAX_COUNT 8

//max. okres miêdzy podlewaniami, w godzinach
#define MAX_PERIOD_IN_HOUR 168

// Domyœlne czasy pracy pompki (w sekundach)
//uint8_t rst_flag = 1;
volatile uint32_t wdt_counter = (uint32_t)(MAX_PERIOD_IN_HOUR + 1) * 60 * 60;
//uint32_t pump_period;
volatile bool heartbit = 0;

void init_adc_for_period(void) {
	// Konfiguracja ADC z napiêciem odniesienia Vcc, wejœcie PB4
	ADCSRA = 0;	//wylacz ADC
	//_delay_ms(1);	//poczekaj chwile
	ADMUX = (1 << MUX1); // Vref = Vcc, ADC2 (PB4)
	ADCSRA = (1 << ADEN) | (1 << ADPS1) | (1 << ADPS0); // W³¹cz ADC, preskaler 8
	_delay_ms(5);	//poczekaj chwile
}

void init_adc_for_watering(void) {
	// Konfiguracja ADC z napiêciem odniesienia Vcc, wejœcie PB4
	ADCSRA = 0;	//wylacz ADC
	//_delay_ms(1);	//poczekaj chwile
	ADMUX = (1 << MUX0); // Vref = Vcc, ADC1 (PB2)
	ADCSRA = (1 << ADEN) | (1 << ADPS1) | (1 << ADPS0); // W³¹cz ADC, preskaler 8
	_delay_ms(5);	//poczekaj chwile
}

void init_adc_for_battery_monitor(void) {
	// Konfiguracja ADC z napiêciem odniesienia 1.1V, wejœcie PB4
	ADCSRA = 0;	//wylacz ADC
	//_delay_ms(1);	//poczekaj chwile
	ADMUX = (1 << REFS0) | (1 << MUX1) | (1 << MUX0); // Vref = 1.1V, ADC3 (PB3)
	ADCSRA = (1 << ADEN) | (1 << ADPS1) | (1 << ADPS0); // W³¹cz ADC, preskaler 8
	_delay_ms(5);	//poczekaj chwile
}

uint16_t read_adc(void) {
	ADCSRA |= (1 << ADSC); // Rozpocznij konwersjê
	while (ADCSRA & (1 << ADSC)); // Czekaj na zakoñczenie
	return ADC;
}

void init_led(void) {
	DDRB |= (1 << LED_PIN); // LED i pompka jako wyjœcia
	PORTB &= ~(1 << LED_PIN); // Wy³¹cz LED i pompkê
}

void init_pump_pin_as_output(void) {
	DDRB |= (1 << PUMP_PIN); // LED i pompka jako wyjœcia
	PORTB &= ~(1 << PUMP_PIN); // Wy³¹cz LED i pompkê
}

void init_pump_pin_as_input(void) {
	DDRB &= ~(1 << PUMP_PIN); // pin pompki jako wejscie do monioringu napiecia baterii
	PORTB &= ~(1 << PUMP_PIN); // no pull-up, tri-state
}

inline void init_wdt(void) {
	// WDT: najd³u¿szy okres (~8s), w trybie interrupt
	WDTCR = (1 << WDTIE) | (1 << WDP3) | (1 << WDP0);
}

inline void led_on(void){
	PORTB |= (1 << LED_PIN);
}

inline void led_off(void){
	PORTB &= ~(1 << LED_PIN);
}

void blink_led(uint8_t times) {
	for (uint8_t i = 0; i < times; i++) {
		led_on();
		_delay_ms(50);
		led_off();
		_delay_ms(100);
	}
}

// configuration - interwal pomiedzy kolejnymi podlewaniami
uint16_t get_battery_level(void) {
	
	init_pump_pin_as_input();
	init_adc_for_battery_monitor();
	uint16_t adc_val = read_adc();	
	init_pump_pin_as_output();
	/*
	deltaAdc = 1100mV/1024 = 1.074218mV/bin
	Voltage divider coeff. with R1=100k|R2=1M => 11
	ADC Coeff = 1.074218 mV/bin
	Total ADC-to-mv-coeff = 11 * 1.0742 = 11,8163 = ~12
	Vbatt = adc_value * 12 [mV] //error is -1.55%
	*/
	
	return adc_val * 12; // in mV
}

uint16_t get_water_volume_in_ml(void) {
	
	init_adc_for_watering();
	uint16_t adc_val = read_adc();

	/* design:
		ADC=0 --> 10ml
		ADC=1024 --> 266ml
		So it is easy to calculate:
		Volume_ml = 10 + (adc * 32) / 128
	*/
	uint16_t water_volume_ml;
	water_volume_ml = 10 + ((adc_val << 5) >> 7);	//calculations with values reduced by 10 to keep within u16
	return water_volume_ml;
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
	
	init_adc_for_period();
	uint16_t adc_value = read_adc();

    // calculate_power using linear interpolation at four points
    // for adc = 0 result 2h
	// for adc = 128 result 24h (1 day)
	// for adc = 192 result 72h	(3 days)
	// for adc = 255 result 168h (7 days)
	
	adc_value >>= 2;	//reduce noise
	adc_value++;
	
    if (adc_value <= 128) {
			return 2 + ((adc_value * (24 - 2)) / 128);
	    } else if (adc_value <= 192) {
		    return 24 + (((adc_value - 128) * (72 - 24)) / 64);
	    } else {
		    return 72 + (((adc_value - (MAX_PERIOD_IN_HOUR + 24)) * (MAX_PERIOD_IN_HOUR - 72)) / 64);
    }
}

ISR(WDT_vect) {
	wdt_counter++;
}

int main(void) {

#if CODE_MODE == 0
	init_led();
	init_wdt();
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	const uint16_t battery_low_threshold1 = 2954;	//mV corresponding to real 3000mV considering -1.55% adc conversion to mV
	const uint16_t battery_low_threshold2 = 2806;	//mV corresponding to real 2850mV considering -1.55% adc conversion to mV
	const uint16_t battery_low_threshold3 = 2658;	//mV corresponding to real 2700mV considering -1.55% adc conversion to mV

	sei(); // W³¹cz globalne przerwania
    
	while (1) {

		// Mrugniêcie LED jako wskaŸnik dzia³ania systemu
		blink_led(1);

		// Sprawdzenie napiêcia baterii
		uint16_t battery_level = get_battery_level();
			
		if (battery_level  < battery_low_threshold1) {
			blink_led(1); // Niski poziom baterii
		}
		else if (battery_level  < battery_low_threshold2) {
			blink_led(2); // Niski poziom baterii
		}
		else if (battery_level  < battery_low_threshold3) {
			blink_led(3); // Niski poziom baterii
		}

/*
		uint32_t pump_period = (uint32_t)get_pump_period_in_hour() * 3600;
*/

		//multiply by 3600 with save ROM
		uint32_t pump_period = get_pump_period_in_hour();
		pump_period = (pump_period << 11)	// x2048
					+ (pump_period << 10)	// x1024
					+ (pump_period << 9)	// x512
					+ (pump_period << 4);	//x16


		// check if it is time to run pump
		if (wdt_counter >= (pump_period / WDT_MAX_COUNT)) {
			uint16_t water_volume = get_water_volume_in_ml();
			uint16_t watering_period = volume_to_pump_sec(water_volume, battery_level);

			// W³¹cz pompkê
			PORTB |= (1 << PUMP_PIN);
			for (uint16_t i = 0; i < watering_period; i++) {
				_delay_ms(1000);
			}
			PORTB &= ~(1 << PUMP_PIN); // Wy³¹cz pompkê

			wdt_counter = watering_period / WDT_MAX_COUNT; // cnt reset
		}
	
		// Przejœcie w stan uœpienia
		sleep_mode();
	}

#elif CODE_MODE == 1
	init_led();
	while(1)
	{
		uint32_t pump_period = get_pump_period_in_hour();
		if (pump_period == 2)
			blink_led(1);
		else if (pump_period == 12)
			blink_led(1);
		else if (pump_period == 24)
			blink_led(1);
		else if (pump_period == 72)
			blink_led(1);
		else if (pump_period == 168)
			blink_led(1);
		
	}

#elif CODE_MODE == 2
	init_led();
	while(1)
	{
		uint16_t water_volume = get_water_volume_in_ml();

		if (water_volume == 10)
			blink_led(1);
		else if (water_volume == 50)
			blink_led(1);
		else if (water_volume == 100)
			blink_led(1);
		else if (water_volume == 150)
			blink_led(1);
		else if (water_volume == 200)
			blink_led(1);
		else if (water_volume == 250)
			blink_led(1);
		else if (water_volume == 265)
			blink_led(1);
	}

#endif

}