#define F_CPU 11059200UL
#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include <util/delay.h>
#include <stdio.h>
#include <inttypes.h>
#include "my_lcd.h"

#define DATA_COLLECTION_MODE 0  // Set to 1 to dump features via UART, 0 for classification

#define NUM_FRAMES 40           // How many frames make up one word
#define SAMPLES_PER_FRAME 200   // At 8kHz, 200 samples = 25ms per frame
#define NUM_FEATURES 2          // Energy and ZCR
#define NUM_COMMANDS 8

#define START_THRESHOLD 40      // Energy spike to start recording
#define END_THRESHOLD 10        // Energy drop to stop recording
#define REJECTION_THRESHOLD 2500 // Max DTW score to accept a match

const uint8_t template_0[NUM_FRAMES][NUM_FEATURES] PROGMEM = { 
{50, 43},
{53, 36},
{72, 31},
{89, 32},
{89, 30},
{89, 38},
{72, 29},
{49, 48},
{20, 45},
{17, 45},
{17, 37},
{15, 43},
{13, 60},
{13, 58},
{12, 51},
{12, 66},
{12, 61},
{13, 70},
{13, 69},
{12, 71},
{12, 60},
{13, 53},
{13, 66},
{11, 61},
{12, 52},
{14, 56},
{13, 67},
{12, 72},
{13, 61},
{13, 56},
{13, 63},
{12, 55},
{13, 68},
{14, 58},
{14, 69},
{13, 69},
{13, 58},
{13, 57},
{14, 72},
{12, 61} 
}; // e.g., "ON"
const uint8_t template_1[NUM_FRAMES][NUM_FEATURES] PROGMEM = { {0, 0} }; // e.g., "OFF"
const uint8_t template_2[NUM_FRAMES][NUM_FEATURES] PROGMEM = { {0, 0} }; // e.g., "UP"
const uint8_t template_3[NUM_FRAMES][NUM_FEATURES] PROGMEM = { {0, 0} }; // e.g., "DOWN"
const uint8_t template_4[NUM_FRAMES][NUM_FEATURES] PROGMEM = { {58, 45},
{61, 51},
{65, 45},
{43, 47},
{22, 63},
{27, 95},
{22, 77},
{23, 81},
{18, 95},
{22, 87},
{13, 72},
{13, 65},
{13, 76},
{13, 84},
{13, 63},
{13, 70},
{14, 83},
{12, 71},
{12, 62},
{15, 82},
{21, 81},
{19, 59},
{16, 74},
{12, 65},
{11, 80},
{11, 82},
{13, 61},
{13, 70},
{14, 66},
{13, 79},
{13, 79},
{12, 84},
{14, 57},
{13, 69},
{13, 68},
{13, 64},
{12, 61},
{12, 73},
{13, 74},
{14, 76} }; // e.g., "LEFT"
const uint8_t template_5[NUM_FRAMES][NUM_FEATURES] PROGMEM = { {0, 0} }; // e.g., "RIGHT"
const uint8_t template_6[NUM_FRAMES][NUM_FEATURES] PROGMEM = { {0, 0} }; // e.g., "START"
const uint8_t template_7[NUM_FRAMES][NUM_FEATURES] PROGMEM = { {0, 0} }; // e.g., "STOP"

const char* command_names[NUM_COMMANDS] = {
    "ON", "OFF", "UP", "DOWN", 
    "LEFT", "RIGHT", "START", "STOP"
};

const uint8_t* const command_templates[NUM_COMMANDS] PROGMEM = {
    (const uint8_t*)template_0,
    (const uint8_t*)template_1,
    (const uint8_t*)template_2,
    (const uint8_t*)template_3,
    (const uint8_t*)template_4,
    (const uint8_t*)template_5,
    (const uint8_t*)template_6,
    (const uint8_t*)template_7
};

volatile uint8_t live_features[NUM_FRAMES][NUM_FEATURES];
volatile uint8_t current_frame = 0;
volatile uint16_t sample_count = 0;

volatile int32_t energy_accumulator = 0;
volatile uint8_t zcr_accumulator = 0;
volatile int16_t last_sample = 0;

volatile uint8_t silence_counter = 0;
volatile uint8_t noise_counter = 0;

enum SystemState { IDLE, RECORDING, PROCESSING };
volatile enum SystemState current_state = IDLE;
volatile uint8_t recording_display_pending = 0;

// ==========================================
// UART STDIO SETUP
// ==========================================
void UART_init(long baudrate) { // baudrate parameter is shadowed by macro, but keeps compatibility
    UBRRL = BAUD_PRESCALE;
    UBRRH = (BAUD_PRESCALE >> 8);
    UCSRB = (1 << RXEN) | (1 << TXEN);
    UCSRC = (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1); // 8-bit char size
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
// HARDWARE SETUP
// ==========================================
void adc_init() {
    ADMUX = (1 << REFS0) | 0x00;
    ADCSRA = (1 << ADEN) | (1 << ADATE) | (1 << ADSC)
           | (1 << ADPS2) | (1 << ADPS1); // start free-running
}

void timer1_init() {
    // Set CTC mode
    TCCR1B = 0; 
    TCCR1B |= (1 << WGM12) | (1 << CS10); 
    OCR1A = 1381;
    TIMSK |= (1 << OCIE1A);
}

// ==========================================
// INTERRUPT: AUDIO SAMPLING & VAD
// ==========================================
ISR(TIMER1_COMPA_vect) {
//    ADCSRA |= (1 << ADSC);
//    while (ADCSRA & (1 << ADSC)); 
    
    // Center signal around 0 (Assuming 10-bit ADC, mid-point is 512)
    int16_t current_sample = ADC - 512; 
    
    // Extract Features (Zero Crossing & Energy of frame)
    energy_accumulator += abs(current_sample);
    if ((current_sample >= 0 && last_sample < 0) || (current_sample < 0 && last_sample >= 0)) {
        zcr_accumulator++;
    }
    last_sample = current_sample;
    sample_count++;

    // Process a completed frame
    if (sample_count >= SAMPLES_PER_FRAME) {
        uint8_t frame_energy = (uint8_t)(energy_accumulator / SAMPLES_PER_FRAME);
        
        if (current_state == IDLE) {
            if (frame_energy > START_THRESHOLD) {
                // saving just in case it was a frame of the word
                live_features[noise_counter][0] = frame_energy;
                live_features[noise_counter][1] = zcr_accumulator;

                noise_counter++;
                if (noise_counter >= 2) {
                    current_state = RECORDING;
                    current_frame = 2; 
                    recording_display_pending = 1;
                }
            } else {
                noise_counter = 0;
            }
        } 
        else if (current_state == RECORDING) {
            live_features[current_frame][0] = frame_energy;
            live_features[current_frame][1] = zcr_accumulator;
            current_frame++;
            
            if (frame_energy < END_THRESHOLD) {
                silence_counter++;
            } else {
                silence_counter = 0;
            }

            if (silence_counter >= 8 || current_frame >= NUM_FRAMES) {
                current_state = PROCESSING; 
                silence_counter = 0;
                noise_counter = 0;
            }
        }
        
        sample_count = 0;
        energy_accumulator = 0;
        zcr_accumulator = 0;
    }
}

uint16_t calculate_dtw_2row(const uint8_t* template_ptr, uint8_t actual_frames) {
    uint16_t row_prev[NUM_FRAMES + 1];
    uint16_t row_curr[NUM_FRAMES + 1];
    
    for(int i = 0; i <= NUM_FRAMES; i++) row_prev[i] = 65535; // inf
    row_prev[0] = 0;

    for (int i = 1; i <= actual_frames; i++) {
        row_curr[0] = 65535; 
        
        for (int j = 1; j <= NUM_FRAMES; j++) {
            uint8_t tmpl_energy = pgm_read_byte(&(template_ptr[(j-1)*NUM_FEATURES + 0]));
            uint8_t tmpl_zcr = pgm_read_byte(&(template_ptr[(j-1)*NUM_FEATURES + 1]));
            
            // Simple absolute distance
            uint16_t dist = abs(live_features[i-1][0] - tmpl_energy) + abs(live_features[i-1][1] - tmpl_zcr);
            
            uint16_t min_path = row_prev[j]; 
            if (row_curr[j-1] < min_path) min_path = row_curr[j-1]; 
            if (row_prev[j-1] < min_path) min_path = row_prev[j-1]; 
            
            row_curr[j] = dist + min_path;
        }
        for(int k = 0; k <= NUM_FRAMES; k++) row_prev[k] = row_curr[k];
    }
    return row_curr[NUM_FRAMES]; 
}

int main(void) {
    // Initialize UART and standard IO streams
    UART_init(9600);
    stdin = stdout = &uart_str;

    adc_init();
    timer1_init();
    
    LCD_Init();
    LCD_Clear();
    LCD_String("Vo System");
    LCD_Gotoxy(1, 0);
    LCD_String("Listening...");
    printf("=======Starting System========\r\n");
    sei();
    printf("=======in System========\r\n");
    while (1){
        if (recording_display_pending && current_state == PROCESSING) {
            LCD_Clear();
            LCD_String("Recording...");
            recording_display_pending = 0;
        }
        if (current_state == PROCESSING){
            
            if (DATA_COLLECTION_MODE) {
                // Using standard printf now!
                printf("===");
                for (int i = 0; i < current_frame; i++) {
                    printf("{%d, %d}, \r\n", live_features[i][0], live_features[i][1]);
                }
            } 
            else {
                uint16_t best_score = 65535;
                uint8_t best_cmd = 255;
                
                for (uint8_t c = 0; c < NUM_COMMANDS; c++) {
                    const uint8_t* temp_ptr = (const uint8_t*)pgm_read_word(&(command_templates[c]));
                    uint16_t score = calculate_dtw_2row(temp_ptr, current_frame);
                    
                    if (score < best_score) {
                        best_score = score;
                        best_cmd = c;
                    }
                }
                
                // Print Results to LCD
                LCD_Clear();
                
                if (best_score < REJECTION_THRESHOLD) {
                    LCD_String("Command:");
                    LCD_String_xy(1, 0, (char*)command_names[best_cmd]); 
                    
                    // You can now easily print debug data via UART too!
                    // printf("Match: %s, Score: %u\r\n", command_names[best_cmd], best_score);
                } else {
                    LCD_String("Unknown Noise"); 
                    // printf("Rejected. Lowest score: %u\r\n", best_score);
                }
                
                _delay_ms(1500); 
                
                LCD_Clear();
                LCD_String("Listening...");
            }
            current_state = IDLE;
        }
    }
}