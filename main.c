/*
 * Automatic Watering Can
 * main.c
 *
 * Created: 1/5/2025 8:06:56 PM
 * Updated: 11/19/2025
 *  Author: tobajer@poczta.onet.pl
 * 
 * Fuse settings:
 * ATtiny25/45/85
	 EXTENDED: 0xFF
	 HIGH: 0xD7
	 LOW: 0x62
 */ 

#define F_CPU 1000000UL

//#include <xc.h>
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

// ============================================================================
// Pin Definitions
// ============================================================================
#define LED_PIN PB1          // LED indicator (also used as watering status indicator)
#define PUMP_PIN PB3          // Pump control pin (also used for battery monitoring ADC)
#define BUTTON_PIN PB0        // Button input pin

// ============================================================================
// Constants
// ============================================================================
#define WDT_MAX_COUNT 8
#define MAX_PERIOD_IN_HOUR 168

// ============================================================================
// EEPROM Address Definitions
// ============================================================================
#define EE_PERIOD_BASE_ADDR 0                    // Base address for period calibration points
#define EE_PERIOD_N 11                           // Number of period calibration points
#define EE_WATERING_BASE_ADDR (EE_PERIOD_BASE_ADDR + EE_PERIOD_N)  // Base address for watering calibration points
#define EE_WATERING_N 7                          // Number of watering calibration points

// ============================================================================
// Global Variables
// ============================================================================
// Initialize wdt_counter with highest value to run pump right after reset
volatile uint32_t wdt_counter = (uint32_t)(MAX_PERIOD_IN_HOUR + 1) * 60 * 60;

// ============================================================================
// Calibration Target Values
// ============================================================================
const uint16_t target_p[EE_PERIOD_N] = {2, 6, 12, 18, 24, 48, 72, 96, 120, 144, 168};  // Period targets in hours
const uint16_t target_w[EE_WATERING_N] = {10, 50, 100, 150, 200, 250, 270};            // Water volume targets in milliliters


// ============================================================================
// ADC Functions
// ============================================================================
void init_adc_for_period(void) {
	ADMUX = (1 << MUX1) | (1 << ADLAR);  // Vref = Vcc, ADC2 (PB4), left adjusted
	ADCSRA = (1 << ADEN) | (1 << ADPS1) | (1 << ADPS0);  // Enable ADC, prescaler 8
	_delay_ms(5);  // Wait for ADC to stabilize
}

void init_adc_for_watering(void) {
	ADMUX = (1 << MUX0) | (1 << ADLAR);  // Vref = Vcc, ADC1 (PB2), left adjusted
	ADCSRA = (1 << ADEN) | (1 << ADPS1) | (1 << ADPS0);  // Enable ADC, prescaler 8
	_delay_ms(5);  // Wait for ADC to stabilize
}

void init_adc_for_battery_monitor(void) {
	ADMUX = (1 << REFS1) | (1 << MUX1) | (1 << MUX0);  // Vref = 1.1V internal, ADC3 (PB3)
	ADCSRA = (1 << ADEN) | (1 << ADPS1) | (1 << ADPS0);  // Enable ADC, prescaler 8
	_delay_ms(5);  // Wait for ADC to stabilize
}

void enable_adc(void) {
	ADCSRA |= (1 << ADEN);  // Re-enable ADC (doesn't change other settings)
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

// ============================================================================
// LED Functions
// ============================================================================
static inline void led_on(void) {
	PORTB |= (1 << LED_PIN);
}

static inline void led_off(void) {
	PORTB &= ~(1 << LED_PIN);
}

void init_led(void) {
	DDRB |= (1 << LED_PIN);  // Set LED pin as output
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

// ============================================================================
// Button Functions
// ============================================================================
void init_button(void)
{
    DDRB &= ~(1 << BUTTON_PIN); // Przycisk jako wej�cie
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

// ============================================================================
// Watchdog Timer Functions
// ============================================================================
static inline void init_wdt(void) {
	// WDT: set longest period (~8s), interrupt mode
	WDTCR = (1 << WDIE) | (1 << WDP3) | (1 << WDP0);
}

// ============================================================================
// Pump Pin Control Functions
// ============================================================================
void init_pump_pin_as_output(void) {
	DDRB |= (1 << PUMP_PIN);      // Set pump pin as output
	PORTB &= ~(1 << PUMP_PIN);    // Set pump pin low (pump off)
}

void init_pump_pin_as_input(void) {
	DDRB &= ~(1 << PUMP_PIN);     // Set pump pin as input for battery voltage read
	PORTB &= ~(1 << PUMP_PIN);    // No pull-up, tri-state
}


// ============================================================================
// Battery Monitoring Functions
// ============================================================================
uint16_t get_battery_level(void) {
	init_pump_pin_as_input();
	init_adc_for_battery_monitor();
	uint16_t adc_val = read_adc();
	
	/*
	 * Battery voltage calculation:
	 * deltaAdc = 1100mV/1024 = 1.074218mV/bin
	 * Voltage divider coeff. with R1=100k|R2=1M => 11
	 * ADC Coeff = 1.074218 mV/bin
	 * Total ADC-to-mv-coeff = 11 * 1.0742 = 11.8163 = ~12
	 * Vbatt = adc_value * 12 [mV]
	 * Error is -1.55% which is acceptable
	 */
	
	return adc_val * 12;  // Return voltage in mV
}

// ============================================================================
// ADC Reading Functions
// ============================================================================
uint8_t get_period_adc_value(void) {
	init_adc_for_period();
	return read_adch();
}

uint8_t get_water_adc_value(void) {
	init_adc_for_watering();
	return read_adch();
}

// ============================================================================
// Water Volume Calculation Functions
// ============================================================================
uint16_t get_water_volume_in_ml(void) {
	uint16_t adc_val = get_water_adc_value();
	uint16_t result = 0;
	uint8_t cal, cal_prev;

	cal_prev = 0;
	for (uint8_t i = 1; i < EE_WATERING_N; i++) {
		cal = eeprom_read_byte((uint8_t *)(i + EE_WATERING_BASE_ADDR));
		
		// Validate calibration data is in ascending order
		if (cal <= cal_prev) {
			// Invalid calibration data - use previous target value
			result = target_w[i - 1];
			break;
		}
		
		if (adc_val <= cal) {
			// Prevent division by zero
			if (cal != cal_prev) {
				result = target_w[i - 1] + ((target_w[i] - target_w[i - 1]) * (adc_val - cal_prev)) / (cal - cal_prev);
			} else {
				result = target_w[i - 1];
			}
			break;
		}
		cal_prev = cal;
	}
	
	// If result = 0, it means the last point on scale is set below max ADC=255,
	// hence, set result at max value
	if (result == 0) {
		result = target_w[EE_WATERING_N - 1];
	}
	
	return result;
}

uint16_t volume_to_pump_sec(uint16_t volume_ml, uint16_t Vcc_mV) {
	/*
	 * Pump efficiency measured at different voltages:
	 * Vcc=3.0V --> 200ml @ 157 sec
	 * Vcc=3.6V --> 200ml @ 125 sec
	 * Vcc=4.1V --> 200ml @ 105 sec
	 * 
	 * Calculated coefficient sec/ml:
	 * Vcc=3.0V --> 200ml / 157 sec = 0.79
	 * Vcc=3.6V --> 200ml / 125 sec = 0.63
	 * Vcc=4.2V --> 200ml / 105 sec = 0.53
	 * 
	 * Linear dependence of time T of 'pump on' versus Vcc:
	 * T = a * Vcc + b, where a = -0.2374, b = 1.4916
	 * Let a = -0.25 (i.e., -1/4) for easy calculation
	 * Then, b = 1.55 gives closest timing to experiment for 200ml water volume.
	 * Since Vcc is in mV, b = 1550
	 * 
	 * T(Vcc)[sec] = (1550 - 1/4 * Vcc) * water_volume_ml / 1000
	 * 
	 * To fit into uint16 calculations (ROM saving) and simplify operations,
	 * calculations are reduced by 32 (2^5):
	 * b/32 = 49, (Vcc/32)/4 = Vcc>>7, and result (reduced by 32) is divided
	 * finally by 32 corresponding to /1000 (actually 1024)
	 * 
	 * Residual error of pump periods is ca. +5% which is well acceptable
	 */
	
	// Prevent underflow: if Vcc_mV > 6272, clamp the calculation
	uint16_t vcc_shifted = Vcc_mV >> 7;
	uint16_t coefficient;
	
	if (vcc_shifted >= 49) {
		// Very high voltage - use minimum coefficient (1)
		coefficient = 1;
	} else {
		coefficient = 49 - vcc_shifted;
	}
	
	uint16_t T = (coefficient * volume_ml) >> 5;
	
	return T;
}

// ============================================================================
// Pump Period Calculation Functions
// ============================================================================
uint16_t get_pump_period_in_hour(void) {
	uint16_t adc_val = get_period_adc_value();
	uint16_t result = 0;
	uint8_t cal, cal_prev;

	cal_prev = 0;
	for (uint8_t i = 1; i < EE_PERIOD_N; i++) {
		cal = eeprom_read_byte((uint8_t *)(i + EE_PERIOD_BASE_ADDR));
		
		// Validate calibration data is in ascending order
		if (cal <= cal_prev) {
			// Invalid calibration data - use previous target value
			result = target_p[i - 1];
			break;
		}
		
		if (adc_val <= cal) {
			// Prevent division by zero
			if (cal != cal_prev) {
				result = target_p[i - 1] + ((target_p[i] - target_p[i - 1]) * (adc_val - cal_prev)) / (cal - cal_prev);
			} else {
				result = target_p[i - 1];
			}
			break;
		}
		cal_prev = cal;
	}
	
	// If result = 0, it means the last point on scale is set below max ADC=255,
	// hence, set result at max value
	if (result == 0) {
		result = target_p[EE_PERIOD_N - 1];
	}
	
	return result;
}

// ============================================================================
// EEPROM Calibration Functions
// ============================================================================
void store_default_calibration_points_to_eeprom(void) {
	uint8_t cal_p0 = eeprom_read_byte((uint8_t *)(EE_PERIOD_BASE_ADDR));
	uint8_t cal_w0 = eeprom_read_byte((uint8_t *)(EE_WATERING_BASE_ADDR));

	// Default calibration points for ideal pot, corresponding to scale on designed label
	const uint8_t def_cal_p[] = {0, 35, 64, 96, 128, 159, 186, 202, 217, 231, 255};
	const uint8_t def_cal_w[] = {0, 55, 103, 142, 185, 224, 255};

	// When unprogrammed, both values should be 0xFF
	if ((cal_p0 == 0xFF) && (cal_w0 == 0xFF)) {
		eeprom_update_block((const void*)def_cal_p, (void *)(EE_PERIOD_BASE_ADDR), EE_PERIOD_N);
		eeprom_update_block((const void*)def_cal_w, (void *)(EE_WATERING_BASE_ADDR), EE_WATERING_N);
	}
	blink_led(4);
}

// ============================================================================
// Interrupt Service Routines
// ============================================================================
// WDT interrupt is triggered every ~8 seconds
ISR(WDT_vect) {
	wdt_counter++;
}

// ============================================================================
// Main Function
// ============================================================================
int main(void) {

#if CODE_MODE == 0
	// Normal operation mode - automatic watering can
	init_led();
	init_wdt();
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	
	// Battery voltage thresholds (mV) - corrected for -1.55% ADC error
	const uint16_t battery_low_threshold1 = 2954;  // Corresponds to real 3000mV
	const uint16_t battery_low_threshold2 = 2806;  // Corresponds to real 2850mV
	const uint16_t battery_low_threshold3 = 2658;  // Corresponds to real 2700mV

	// If EEPROM is unprogrammed, burn default calibrations
	store_default_calibration_points_to_eeprom();

	sei();  // Enable interrupts
    
	while (1) {
		// Indicate system is alive
		blink_led(1);

		// Check battery voltage
		uint16_t battery_level = get_battery_level();
		
		// Check battery level from lowest to highest threshold
		// (lowest voltage = most critical = more blinks)
		if (battery_level < battery_low_threshold3) {
			blink_led(3);  // 4-blink: battery at level 3 (critical - charge/change battery)
		} else if (battery_level < battery_low_threshold2) {
			blink_led(2);  // 3-blink: battery at level 2
		} else if (battery_level < battery_low_threshold1) {
			blink_led(1);  // 2-blink: battery at level 1
		}

		uint32_t pump_period = (uint32_t)get_pump_period_in_hour() * 3600;
		
		// Compensate for inaccuracy of WDT 8 sec period
		// In this chip, the period was longer by 12.5% (1/8)
		pump_period -= (pump_period >> 3);
			
		// Check if it is time to run pump
		if (wdt_counter >= (pump_period / WDT_MAX_COUNT)) {
			wdt_counter = 0;  // Reset counter

			// Get water volume based on potentiometer read
			uint16_t water_volume = get_water_volume_in_ml();

			// Turn on LED to indicate watering
			led_on();
			_delay_ms(1000);  // Let voltage stabilize under load
			
			// Check voltage under load (LED draws 1-3mA)
			// Note: Would be better to measure under pump running, but pump shares the same pin
			uint16_t vcc_under_load = get_battery_level();
			
			// Convert volume to seconds of pump activation
			uint16_t watering_period = volume_to_pump_sec(water_volume, vcc_under_load);

			// Activate pump
			init_pump_pin_as_output();
			PORTB |= (1 << PUMP_PIN);

			// Run pump for calculated time
			for (uint16_t i = 0; i < watering_period; i++) {
				_delay_ms(1000);
			}
			
			// Turn off pump
			PORTB &= ~(1 << PUMP_PIN);
			led_off();
			init_pump_pin_as_input();
		}
	
		// Go to sleep to save power
		ADCSRA = 0;  // Disable ADC to save power during sleep
		sleep_mode();
		// After wake-up, ADC will be re-enabled when needed by init_adc_* functions
	}

/* Calibration of period potentiometer */
#elif CODE_MODE == 1
	uintptr_t ee_addr;
	init_led();
	init_button();

	ee_addr = EE_PERIOD_BASE_ADDR;

	led_on();
	_delay_ms(10000);
	led_off();

	for (uint8_t i = 0; i < EE_PERIOD_N; i++) {
		wait_while_button_pressed_and_released();
		uint8_t adc_val = get_period_adc_value();
		eeprom_update_byte((uint8_t *)ee_addr, adc_val);  // Write to EEPROM
		blink_led(10);
		ee_addr++;  // Increment EEPROM address to store next point
	}

	// Finish - blink continuously
	while (1) {
		led_on();
		_delay_ms(1000);
		led_off();
		_delay_ms(1000);
	}

/* Verification of period potentiometer */
#elif CODE_MODE == 2
	init_led();
	while (1) {
		uint16_t pump_period = get_pump_period_in_hour();
		uint8_t match_found = 0;
		
		for (uint8_t i = 0; i < EE_PERIOD_N; i++) {
			if (pump_period == target_p[i]) {
				match_found = 1;
				break;
			}
		}
		(match_found > 0) ? led_on() : led_off();
	}

/* Calibration of watering potentiometer */
#elif CODE_MODE == 3
	uintptr_t ee_addr;
	init_led();
	init_button();
	
	ee_addr = EE_WATERING_BASE_ADDR;

	led_on();
	_delay_ms(10000);
	led_off();

	for (uint8_t i = 0; i < EE_WATERING_N; i++) {
		wait_while_button_pressed_and_released();
		uint8_t adc_val = get_water_adc_value();
		eeprom_update_byte((uint8_t *)ee_addr, adc_val);  // Write to EEPROM
		blink_led(10);
		ee_addr++;  // Increment EEPROM address to store next point
	}	

	// Finish - blink continuously
	while (1) {
		led_on();
		_delay_ms(1000);
		led_off();
		_delay_ms(1000);
	}

/* Verification of watering potentiometer */
#elif CODE_MODE == 4
	init_led();
	while (1) {
		uint16_t water_volume = get_water_volume_in_ml();
		uint8_t match_found = 0;
		
		for (uint8_t i = 0; i < EE_WATERING_N; i++) {
			if (water_volume == target_w[i]) {
				match_found = 1;
				break;
			}
		}
		(match_found > 0) ? led_on() : led_off();
	}

#endif

}