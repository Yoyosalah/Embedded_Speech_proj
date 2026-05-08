#define F_CPU 11059200UL
#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

// ==========================================
// UART SETUP
// ==========================================
void UART_init(long baudrate) { 
    UBRRL = BAUD_PRESCALE;
    UBRRH = (BAUD_PRESCALE >> 8);
    UCSRB = (1 << RXEN) | (1 << TXEN);
    UCSRC = (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1); 
}

int UART_getChar(FILE *stream){
    while ((UCSRA & (1 << RXC)) == 0);
    return (UDR);
}

int UART_putChar(char c , FILE *stream){
    while (!(UCSRA & (1 << UDRE)));
    UDR = c;
    return 0;
}

static FILE uart_str = FDEV_SETUP_STREAM(UART_putChar, UART_getChar, _FDEV_SETUP_RW);

// ==========================================
// ADC SETUP (Polling Mode, No Interrupts)
// ==========================================
void adc_init() { 
    ADMUX = (1 << REFS0); // AVCC reference, ADC0 (PA0 pin)
    // Enable ADC, Prescaler 64. No auto-trigger.
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1); 
}

uint16_t adc_read() {
    ADCSRA |= (1 << ADSC);         // Start a single conversion
    while (ADCSRA & (1 << ADSC));  // Wait for it to finish
    return ADC;                    // Return the 10-bit result
}

// ==========================================
// MAIN LOOP
// ==========================================
int main(void) {
    UART_init(9600);
    stdin = stdout = &uart_str;
    adc_init();
    
    printf("\r\n--- Microphone Hardware Test ---\r\n");
    _delay_ms(1000);

    while (1){
        uint16_t raw_adc = adc_read();
        
        // Calculate the centered value (assuming perfect 2.5V virtual ground)
        int16_t centered_val = raw_adc - 512;
        
        // Print both values to the terminal
        printf("Raw ADC: %4u   |   Centered: %4d\r\n", raw_adc, centered_val);
        
        // Wait 100ms. This prevents the UART crash!
        _delay_ms(100); 
    }
}