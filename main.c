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

#define DATA_COLLECTION_MODE 1  // Set to 1 to dump features via UART, 0 for classification

#define NUM_FRAMES 40           // How many frames make up one word
#define SAMPLES_PER_FRAME 200   // At 8kHz, 200 samples = 25ms per frame
#define NUM_FEATURES 3          // Energy, ZCR, and SSC
#define NUM_COMMANDS 8

#define START_THRESHOLD 40      // Energy spike to start recording
#define END_THRESHOLD 25        // Energy drop to stop recording
#define REJECTION_THRESHOLD 3500 // Increased slightly because we are adding a 3rd distance score

// --- PADDED WITH 0s TO COMPILE. YOU MUST RE-RECORD THESE! ---
const uint8_t template_0[NUM_FRAMES][NUM_FEATURES] PROGMEM = { 
{53, 35, 83},
{84, 45, 51},
{96, 42, 50},
{137, 45, 44},
{128, 49, 50},
{122, 46, 47},
{120, 42, 45},
{87, 42, 53},
{69, 42, 52},
{60, 39, 53},
{37, 30, 60},
{21, 30, 80},
{9, 27, 88},
{10, 22, 98},
{7, 15, 92},
{7, 23, 91},
{5, 48, 108},
{8, 20, 105},
{7, 24, 96}
}; // e.g., "ON"

const uint8_t template_1[NUM_FRAMES][NUM_FEATURES] PROGMEM = { {0, 0, 0} }; // e.g., "OFF"
const uint8_t template_2[NUM_FRAMES][NUM_FEATURES] PROGMEM = { {0, 0, 0} }; // e.g., "UP"
const uint8_t template_3[NUM_FRAMES][NUM_FEATURES] PROGMEM = { {0, 0, 0} }; // e.g., "DOWN"

// --- PADDED WITH 0s TO COMPILE. YOU MUST RE-RECORD THESE! ---
const uint8_t template_4[NUM_FRAMES][NUM_FEATURES] PROGMEM = { 
{93, 27, 89},
{108, 28, 79},
{105, 24, 65},
{113, 28, 66},
{42, 34, 84},
{17, 63, 127},
{16, 85, 141},
{8, 44, 136},
{8, 20, 121},
{9, 14, 111},
{7, 15, 108},
{6, 7, 96},
{7, 10, 109}
}; // e.g., "LEFT"

const uint8_t template_5[NUM_FRAMES][NUM_FEATURES] PROGMEM = { {0, 0, 0} }; // e.g., "RIGHT"
const uint8_t template_6[NUM_FRAMES][NUM_FEATURES] PROGMEM = { {0, 0, 0} }; // e.g., "START"
const uint8_t template_7[NUM_FRAMES][NUM_FEATURES] PROGMEM = { {0, 0, 0} }; // e.g., "STOP"

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

const uint8_t template_lengths[NUM_COMMANDS] = {
    19, // Length of "ON"
    0,  // Length of "OFF" (Empty for now)
    0,  // Length of "UP"
    0,  // Length of "DOWN"
    13, // Length of "LEFT"
    0,  // RIGHT
    0,  // START
    0   // STOP
};

volatile uint8_t live_features[NUM_FRAMES][NUM_FEATURES];
volatile uint8_t current_frame = 0;
volatile uint16_t sample_count = 0;

volatile int32_t energy_accumulator = 0;
volatile uint8_t zcr_accumulator = 0;
volatile int16_t last_sample = 0;

volatile uint8_t silence_counter = 0;
volatile uint8_t noise_counter = 0;

volatile uint8_t ssc_accumulator = 0;
volatile int8_t last_slope = 0;

enum SystemState { IDLE, RECORDING, PROCESSING };
volatile enum SystemState current_state = IDLE;
volatile uint8_t recording_display_pending = 0;

// ==========================================
// UART STDIO SETUP
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
// HARDWARE SETUP
// ==========================================
void adc_init() {
    ADMUX = (1 << REFS0) | 0x00;
    ADCSRA = (1 << ADEN) | (1 << ADATE) | (1 << ADSC) | (1 << ADPS2) | (1 << ADPS1); 
}

void timer1_init() {
    TCCR1B = 0; 
    TCCR1B |= (1 << WGM12) | (1 << CS10); 
    OCR1A = 1381;
    TIMSK |= (1 << OCIE1A);
}

// ==========================================
// INTERRUPT: AUDIO SAMPLING & VAD
// ==========================================
ISR(TIMER1_COMPA_vect) {
    
    int16_t current_sample = ADC - 512; 
    
    // 1. Energy
    energy_accumulator += abs(current_sample);
    
    // 2. ZCR (Zero Crossing Rate)
    if ((current_sample >= 0 && last_sample < 0) || (current_sample < 0 && last_sample >= 0)) {
        zcr_accumulator++;
    }
    
    // 3. SSC (Sign of Slope Change) - NEW
    int8_t current_slope = (current_sample > last_sample) ? 1 : -1;
    if (current_slope != last_slope) {
        ssc_accumulator++;
    }
    
    // Update histories
    last_slope = current_slope;
    last_sample = current_sample;
    sample_count++;

    // Process a completed frame
    if (sample_count >= SAMPLES_PER_FRAME) {
        uint8_t frame_energy = (uint8_t)(energy_accumulator / SAMPLES_PER_FRAME);
        
        if (current_state == IDLE) {
            if (frame_energy > START_THRESHOLD) {
                // Save Pre-roll
                live_features[noise_counter][0] = frame_energy;
                live_features[noise_counter][1] = zcr_accumulator;
                live_features[noise_counter][2] = ssc_accumulator; // Save SSC

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
            live_features[current_frame][2] = ssc_accumulator; // Save SSC
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
        
        // Reset all accumulators for the next frame
        sample_count = 0;
        energy_accumulator = 0;
        zcr_accumulator = 0;
        ssc_accumulator = 0; // Reset SSC
    }
}

// ==========================================
// DTW ALGORITHM 
// ==========================================
uint16_t calculate_dtw_2row(const uint8_t* template_ptr, uint8_t tmpl_frames, uint8_t actual_frames) {
    uint16_t row_prev[NUM_FRAMES + 1];
    uint16_t row_curr[NUM_FRAMES + 1];
    
    for(int i = 0; i <= tmpl_frames; i++) row_prev[i] = 65535; 
    row_prev[0] = 0;

    for (int i = 1; i <= actual_frames; i++) {
        row_curr[0] = 65535; 
        
        for (int j = 1; j <= tmpl_frames; j++) { 
            // Read 3 features from Flash
            uint8_t tmpl_energy = pgm_read_byte(&(template_ptr[(j-1)*NUM_FEATURES + 0]));
            uint8_t tmpl_zcr = pgm_read_byte(&(template_ptr[(j-1)*NUM_FEATURES + 1]));
            uint8_t tmpl_ssc = pgm_read_byte(&(template_ptr[(j-1)*NUM_FEATURES + 2])); // Read SSC

            // Delta Energy Calculation
            int16_t live_delta_energy = (i > 1) ? ((int16_t)live_features[i-1][0] - (int16_t)live_features[i-2][0]) : 0;
            int16_t tmpl_delta_energy = (j > 1) ? ((int16_t)tmpl_energy - (int16_t)pgm_read_byte(&(template_ptr[(j-2)*NUM_FEATURES + 0]))) : 0;

            // Distances
            uint16_t energy_dist = abs(live_delta_energy - tmpl_delta_energy);
            uint16_t zcr_dist = abs(live_features[i-1][1] - tmpl_zcr);
            uint16_t ssc_dist = abs(live_features[i-1][2] - tmpl_ssc); // SSC Distance

            // The New Weighted Distance Formula!
            // We multiply ZCR and SSC to make the algorithm care more about frequency than pure volume.
            uint16_t dist = energy_dist + (zcr_dist * 2) + (ssc_dist * 2);
            
            uint16_t min_path = row_prev[j];    
            if (row_curr[j-1] < min_path) min_path = row_curr[j-1]; 
            if (row_prev[j-1] < min_path) min_path = row_prev[j-1]; 
            
            row_curr[j] = dist + min_path;
        }
        for(int k = 0; k <= tmpl_frames; k++) row_prev[k] = row_curr[k]; 
    }
    return row_curr[tmpl_frames]; 
}

// ==========================================
// MAIN LOOP
// ==========================================
int main(void) {
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

    while (1){
        if (recording_display_pending && current_state == PROCESSING) {
            LCD_Clear();
            LCD_String("Recording...");
            recording_display_pending = 0;
        }
        
        if (current_state == PROCESSING){
            
            if (DATA_COLLECTION_MODE) {
                // Now dumps all 3 features to UART!
                for (int i = 0; i < current_frame; i++) {
                    printf("{%d, %d, %d}, \r\n", live_features[i][0], live_features[i][1], live_features[i][2]);
                }
                printf("===");
            } 
            else {
                uint16_t best_score = 65535;
                uint8_t best_cmd = 255;
                
                for (uint8_t c = 0; c < NUM_COMMANDS; c++) {
    
                    if (template_lengths[c] == 0) continue; 

                    const uint8_t* temp_ptr = (const uint8_t*)pgm_read_word(&(command_templates[c]));
                    uint16_t score = calculate_dtw_2row(temp_ptr, template_lengths[c], current_frame);

                    if (score < best_score) {
                        best_score = score;
                        best_cmd = c;
                    }
                }
                
                LCD_Clear();
                
                if (best_score < REJECTION_THRESHOLD) {
                    LCD_String("Command:");
                    LCD_String_xy(1, 0, (char*)command_names[best_cmd]); 
                    printf("Match: %s, Score: %u\r\n", command_names[best_cmd], best_score);
                } else {
                    LCD_String("Unknown Noise"); 
                    printf("Rejected. Lowest score: %u\r\n", best_score);
                }
                
                _delay_ms(1500); 
                
                LCD_Clear();
                LCD_String("Listening...");
            }
            current_state = IDLE;
        }
    }
}