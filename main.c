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

#define NUM_FRAMES        40    // Max frames buffer
#define SAMPLES_PER_FRAME 200   // At ~8kHz, 200 samples = 25ms per frame
#define NUM_FEATURES      4     // Energy, ZCR, SSC, SER
#define NUM_COMMANDS      8

#define START_THRESHOLD      50    // Safely above your random noise spikes (like the 48)
#define END_THRESHOLD        20    // Allows the system to catch the quiet end of words
#define SILENCE_FRAMES       12    // Keeps the system patient for multi-syllable words
#define PREROLL_FRAMES       2     // Saves the split-second before you speak
#define REJECTION_THRESHOLD  4500  // Generous ceiling for the DTW score
#define NOISE_FLOOR          35    // CRITICAL: Ignores all the background +/- 20 hum for ZCR/SS
#define DTW_BAND_DIVISOR     4     // Sakoe-Chiba band = longer_len / this value
#define SSC_SHIFT            1     // Right-shift SSC by this to prevent saturation (divide by 2)

// ==========================================
// TEMPLATES (PROGMEM)
// ==========================================

const uint8_t template_0[40][NUM_FEATURES] PROGMEM = {
{55, 71, 54, 161}, {67, 76, 54, 164}, {63, 71, 54, 161}, {64, 85, 62, 168},
{71, 69, 54, 164}, {112, 43, 53, 111}, {182, 22, 52, 120}, {123, 26, 47, 116},
{17, 0, 11, 101}, {16, 0, 8, 82}, {12, 0, 2, 104}, {13, 0, 4, 100},
{13, 0, 6, 91}, {15, 0, 6, 85}, {15, 1, 7, 116}, {45, 30, 28, 159},
{86, 62, 53, 141}, {101, 74, 62, 164}, {84, 66, 55, 161}, {81, 80, 60, 173},
{85, 77, 61, 168}, {78, 81, 59, 164}, {83, 95, 63, 178}, {79, 79, 62, 168},
{76, 82, 59, 169}, {77, 78, 59, 156}, {67, 91, 60, 164}, {69, 83, 57, 175},
{32, 44, 46, 148}, {20, 3, 18, 107}, {16, 0, 8, 99}, {13, 0, 5, 98},
{14, 1, 6, 99}, {15, 0, 5, 85}, {13, 0, 5, 97}, {13, 1, 5, 110},
{13, 1, 2, 100}, {13, 0, 5, 94}, {13, 0, 4, 101}, {13, 0, 4, 100}
}; // SIX

const uint8_t template_1[26][NUM_FEATURES] PROGMEM = {
{67, 23, 41, 124}, {89, 25, 35, 125}, {101, 32, 37, 135}, {117, 33, 33, 141},
{126, 39, 29, 144}, {123, 31, 31, 141}, {112, 35, 31, 141}, {98, 30, 26, 138},
{88, 21, 26, 128}, {64, 18, 27, 129}, {54, 8, 26, 120}, {35, 4, 26, 113},
{27, 1, 20, 110}, {22, 1, 16, 108}, {17, 1, 9, 106}, {17, 0, 8, 91},
{13, 0, 5, 102}, {14, 0, 4, 97}, {13, 0, 1, 102}, {14, 0, 6, 95},
{15, 1, 5, 102}, {13, 0, 2, 96}, {12, 0, 4, 106}, {14, 0, 9, 90},
{13, 0, 4, 98}, {14, 1, 4, 101}
}; // NO

const uint8_t template_2[30][NUM_FEATURES] PROGMEM = {
{73, 16, 18, 149}, {156, 32, 24, 149}, {125, 23, 21, 143}, {40, 4, 13, 107},
{15, 0, 6, 87}, {13, 0, 5, 100}, {14, 1, 4, 96}, {12, 0, 3, 112},
{15, 1, 6, 89}, {13, 0, 5, 95}, {13, 1, 5, 105}, {12, 1, 3, 110},
{14, 0, 3, 90}, {15, 2, 5, 108}, {107, 21, 35, 87}, {120, 42, 49, 132},
{76, 49, 43, 159}, {28, 17, 23, 143}, {14, 1, 8, 99}, {13, 0, 5, 103},
{15, 0, 7, 89}, {12, 0, 3, 97}, {13, 0, 4, 104}, {14, 1, 3, 112},
{15, 0, 5, 86}, {13, 1, 3, 113}, {14, 0, 7, 105}, {14, 0, 3, 103},
{15, 0, 6, 95}, {13, 0, 2, 100}
}; // UP

const uint8_t template_3[33][NUM_FEATURES] PROGMEM = {
{54, 68, 51, 163}, {67, 80, 59, 169}, {64, 92, 60, 179}, {66, 84, 58, 172},
{57, 82, 57, 175}, {77, 23, 34, 116}, {127, 22, 25, 135}, {133, 28, 32, 144},
{145, 32, 35, 150}, {138, 29, 30, 149}, {150, 29, 27, 147}, {152, 29, 39, 142},
{149, 28, 36, 140}, {180, 24, 24, 134}, {43, 8, 25, 121}, {19, 9, 16, 118},
{18, 11, 17, 125}, {42, 11, 31, 114}, {59, 26, 40, 129}, {36, 13, 29, 126},
{23, 2, 18, 110}, {16, 0, 11, 98}, {15, 2, 6, 107}, {15, 0, 8, 97},
{14, 0, 3, 89}, {15, 0, 6, 85}, {12, 0, 5, 102}, {14, 0, 9, 102},
{14, 0, 4, 99}, {15, 0, 8, 89}, {14, 2, 3, 112}, {14, 0, 4, 94},
{14, 2, 6, 107}
}; // SEVEN

const uint8_t template_4[24][NUM_FEATURES] PROGMEM = {
    {58, 73, 54, 174}, {52, 83, 59, 167}, {14, 16, 9, 149}, {7, 0, 0, 106},
    {18, 13, 16, 154}, {66, 12, 28, 127}, {86, 16, 17, 134}, {86, 19, 21, 137},
    {66, 16, 17, 136}, {54, 11, 21, 140}, {33, 4, 15, 129}, {22, 2, 13, 129},
    {12, 0, 4, 120}, {9, 0, 2, 132}, {13, 2, 8, 105}, {7, 0, 0, 105},
    {7, 0, 0, 109}, {7, 0, 0, 106}, {7, 0, 0, 110}, {8, 0, 1, 115},
    {7, 0, 0, 92}, {7, 0, 0, 119}, {11, 5, 5, 142}, {11, 3, 2, 140}
}; // START

const uint8_t template_5[40][NUM_FEATURES] PROGMEM = {
{70, 8, 31, 130}, {87, 24, 30, 143}, {95, 26, 29, 148}, {52, 19, 38, 131},
{47, 39, 42, 147}, {76, 74, 56, 167}, {69, 75, 54, 169}, {65, 67, 52, 161},
{55, 59, 48, 165}, {61, 80, 55, 170}, {59, 59, 52, 159}, {52, 65, 52, 164},
{56, 82, 52, 166}, {59, 74, 53, 172}, {63, 89, 55, 169}, {58, 82, 57, 157},
{57, 72, 57, 168}, {50, 74, 47, 158}, {42, 72, 54, 166}, {49, 76, 49, 165},
{41, 62, 49, 155}, {38, 54, 45, 163}, {40, 60, 47, 160}, {32, 51, 36, 160},
{33, 54, 43, 158}, {32, 51, 41, 162}, {31, 52, 46, 145}, {26, 31, 30, 126},
{17, 8, 12, 124}, {14, 2, 7, 99}, {14, 2, 7, 107}, {14, 0, 3, 90},
{14, 0, 7, 103}, {14, 0, 5, 104}, {13, 0, 6, 100}, {15, 0, 8, 85},
{13, 0, 5, 100}, {13, 1, 4, 106}, {13, 0, 3, 101}, {14, 0, 4, 85}
}; // YES

const uint8_t template_6[34][NUM_FEATURES] PROGMEM = {
{59, 89, 64, 152}, {51, 98, 64, 170}, {23, 33, 29, 139}, {15, 1, 6, 97},
{23, 19, 19, 131}, {97, 24, 41, 130}, {178, 25, 29, 137}, {189, 40, 25, 148},
{120, 17, 24, 137}, {14, 0, 8, 106}, {13, 0, 5, 94}, {15, 0, 6, 95},
{14, 0, 8, 89}, {15, 0, 4, 96}, {14, 0, 6, 104}, {15, 0, 3, 99},
{15, 0, 6, 88}, {15, 1, 9, 104}, {116, 16, 29, 97}, {150, 45, 41, 130},
{104, 38, 35, 141}, {46, 22, 26, 140}, {17, 1, 12, 99}, {14, 0, 7, 99},
{14, 0, 7, 89}, {15, 0, 8, 97}, {14, 0, 6, 94}, {15, 0, 7, 96},
{15, 1, 9, 89}, {14, 0, 4, 95}, {14, 0, 5, 95}, {16, 2, 9, 102},
{15, 0, 7, 85}, {15, 2, 7, 111}
}; // STOP

const uint8_t template_7[25][NUM_FEATURES] PROGMEM = {
    {75, 15, 28, 132}, {90, 23, 24, 138}, {111, 21, 21, 149}, 
    {121, 28, 23, 151}, {125, 30, 23, 152}, {105, 29, 21, 152}, 
    {76, 22, 20, 146}, {50, 8, 20, 135}, {13, 1, 5, 111}, 
    {6, 0, 0, 106}, {6, 0, 0, 121}, {6, 0, 0, 113}, 
    {8, 1, 0, 121}, {6, 0, 0, 114}, {6, 0, 0, 114}, 
    {6, 0, 0, 117}, {6, 0, 0, 112}, {6, 0, 0, 108}, {6, 0, 0, 117}, 
    {152, 38, 36, 120}, {111, 45, 35, 149}, {71, 12, 25, 126}, 
    {31, 1, 16, 119}, {20, 0, 6, 112}, {15, 0, 4, 118}
}; // DROP

// ==========================================
// COMMAND TABLE (PROGMEM)
// ==========================================
const char cmd_SIX[]   PROGMEM = "SIX";
const char cmd_NO[]    PROGMEM = "NO";
const char cmd_UP[]    PROGMEM = "UP";
const char cmd_SEVEN[] PROGMEM = "SEVEN";
const char cmd_START[]    PROGMEM = "START";
const char cmd_YES[]   PROGMEM = "YES";
const char cmd_STOP[]  PROGMEM = "STOP";
const char cmd_DROP[]  PROGMEM = "DROP";

const char* const command_names[NUM_COMMANDS] PROGMEM = {
    cmd_SIX, cmd_NO, cmd_UP, cmd_SEVEN,
    cmd_START,  cmd_YES, cmd_STOP, cmd_DROP
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

// --- CORRECTED LENGTHS FOR NEW ARRAYS ---
const uint8_t template_lengths[NUM_COMMANDS] = {
    40,  // SIX
    26,  // NO
    30,  // UP
    33,  // SEVEN
    24,  // START
    40,  // YES
    34,  // STOP
    25   // DROP
};

// ==========================================
// GLOBALS
// ==========================================
volatile uint8_t  live_features[NUM_FRAMES][NUM_FEATURES];
volatile uint8_t  current_frame = 0;
volatile uint16_t sample_count  = 0;

volatile int32_t energy_accumulator   = 0;
volatile uint8_t zcr_accumulator      = 0;
volatile uint8_t ssc_accumulator      = 0;
volatile int16_t last_sample          = 0;
volatile int8_t  last_slope           = 0;

// Sub-band Energy Ratio (SER)
volatile int16_t ser_low_state        = 0;  // IIR filter state ? NOT reset between frames
volatile int32_t low_energy_acc       = 0;
volatile int32_t high_energy_acc      = 0;

volatile uint8_t silence_counter = 0;
volatile uint8_t noise_counter   = 0;

enum SystemState { IDLE, RECORDING, PROCESSING };
volatile enum SystemState current_state = IDLE;
volatile uint8_t recording_display_pending = 0;

// ==========================================
// UART
// ==========================================
void UART_init(void) {
    UBRRL = BAUD_PRESCALE;
    UBRRH = (BAUD_PRESCALE >> 8);
    UCSRB = (1 << RXEN) | (1 << TXEN);
    UCSRC = (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1);
}

int UART_getChar(FILE *stream) {
    while ((UCSRA & (1 << RXC)) == 0);
    return UDR;
}

int UART_putChar(char c, FILE *stream) {
    while (!(UCSRA & (1 << UDRE)));
    UDR = c;
    return 0;
}

static FILE uart_str = FDEV_SETUP_STREAM(UART_putChar, UART_getChar, _FDEV_SETUP_RW);

// ==========================================
// HARDWARE SETUP
// ==========================================
void adc_init(void) {
    // AVcc reference, channel 0, free-running, prescaler 64
    ADMUX  = (1 << REFS0) | 0x00;
    ADCSRA = (1 << ADEN) | (1 << ADATE) | (1 << ADSC)
           | (1 << ADPS2) | (1 << ADPS1);
}

void timer1_init(void) {
    // CTC mode, no prescaler, OCR1A = 1381 -> ~8kHz at 11.0592MHz
    TCCR1B  = 0;
    TCCR1B |= (1 << WGM12) | (1 << CS10);
    OCR1A   = 1381;
    TIMSK  |= (1 << OCIE1A);
}

// ==========================================
// ISR: AUDIO SAMPLING & VAD
// ==========================================
ISR(TIMER1_COMPA_vect) {
    int16_t current_sample = ADC - 512;

    // --- Feature accumulation ---
    energy_accumulator += abs(current_sample);

    // Sub-band split via IIR low-pass (cutoff ~1kHz at 8kHz)
    // low = low * (3/4) + sample * (1/4) ? integer shifts only, no float
    ser_low_state   = ser_low_state - (ser_low_state >> 2) + (current_sample >> 2);
    int16_t ser_high = current_sample - ser_low_state;
    low_energy_acc  += abs(ser_low_state);
    high_energy_acc += abs(ser_high);

    // Only count ZCR and SSC above the noise floor
    if (abs(current_sample) > NOISE_FLOOR || abs(last_sample) > NOISE_FLOOR) {
        if ((current_sample >= 0 && last_sample < 0) ||
            (current_sample < 0 && last_sample >= 0)) {
            zcr_accumulator++;
        }

        int8_t current_slope = (current_sample >= last_sample) ? 1 : -1;
        if (current_slope != last_slope) {
            ssc_accumulator++;
        }
        last_slope = current_slope;
    }

    last_sample = current_sample;
    sample_count++;

    if (sample_count >= SAMPLES_PER_FRAME) {
        uint8_t frame_energy = (uint8_t)(energy_accumulator / SAMPLES_PER_FRAME);
        uint8_t frame_zcr    = zcr_accumulator;
        uint8_t frame_ssc    = ssc_accumulator >> SSC_SHIFT; // SHIFT IS APPLIED HERE!

        // SER: high-band fraction of total energy, scaled to 0-255
        uint32_t total_band = low_energy_acc + high_energy_acc;
        uint8_t  frame_ser  = (total_band > 0)
                            ? (uint8_t)((high_energy_acc * 255UL) / total_band)
                            : 0;

        if (current_state == IDLE) {
            if (frame_energy > START_THRESHOLD) {
                // Save preroll frames so we don't miss word onset
                live_features[noise_counter][0] = frame_energy;
                live_features[noise_counter][1] = frame_zcr;
                live_features[noise_counter][2] = frame_ssc;
                live_features[noise_counter][3] = frame_ser;

                noise_counter++;
                if (noise_counter >= PREROLL_FRAMES) {
                    current_state   = RECORDING;
                    current_frame   = PREROLL_FRAMES;
                    recording_display_pending = 1;
                }
            } else {
                noise_counter = 0;
            }
        }
        else if (current_state == RECORDING) {
            live_features[current_frame][0] = frame_energy;
            live_features[current_frame][1] = frame_zcr;
            live_features[current_frame][2] = frame_ssc;
            live_features[current_frame][3] = frame_ser;
            current_frame++;

            if (frame_energy < END_THRESHOLD) {
                silence_counter++;
            } else {
                silence_counter = 0;
            }

            if (silence_counter >= SILENCE_FRAMES || current_frame >= NUM_FRAMES) {
                current_state   = PROCESSING;
                silence_counter = 0;
                noise_counter   = 0;
            }
        }

        // Reset per-frame accumulators
        // NOTE: ser_low_state is intentionally NOT reset ? it is a running IIR filter state
        sample_count       = 0;
        energy_accumulator = 0;
        zcr_accumulator    = 0;
        ssc_accumulator    = 0;
        low_energy_acc     = 0;
        high_energy_acc    = 0;
    }
}

// ==========================================
// DTW WITH SAKOE-CHIBA BAND & DELTA ENERGY
// ==========================================
uint16_t calculate_dtw_2row(const uint8_t* template_ptr,
                             uint8_t tmpl_frames,
                             uint8_t actual_frames) {
    uint16_t row_prev[NUM_FRAMES + 1];
    uint16_t row_curr[NUM_FRAMES + 1];

    // Reject immediately if lengths differ too much (2:1 ratio)
    uint8_t longer  = (actual_frames > tmpl_frames) ? actual_frames : tmpl_frames;
    uint8_t shorter = (actual_frames < tmpl_frames) ? actual_frames : tmpl_frames;
    if (longer > (uint16_t)shorter * 2) return 65535;

    // Sakoe-Chiba band width (minimum 2 to avoid over-constraining short words)
    uint8_t band = longer / DTW_BAND_DIVISOR;
    if (band < 2) band = 2;

    // Init first row to infinity, except [0] = 0
    for (int i = 0; i <= tmpl_frames; i++) row_prev[i] = 65535;
    row_prev[0] = 0;

    for (int i = 1; i <= actual_frames; i++) {
        row_curr[0] = 65535;

        for (int j = 1; j <= tmpl_frames; j++) {
            // Sakoe-Chiba: skip cells too far from the diagonal
            int diag = (i * tmpl_frames) / actual_frames;
            if (abs(j - diag) > (int)band) {
                row_curr[j] = 65535;
                continue;
            }

            // Read template features from flash (4 features now)
            uint8_t tmpl_energy = pgm_read_byte(&template_ptr[(j-1)*NUM_FEATURES + 0]);
            uint8_t tmpl_zcr    = pgm_read_byte(&template_ptr[(j-1)*NUM_FEATURES + 1]);
            uint8_t tmpl_ssc    = pgm_read_byte(&template_ptr[(j-1)*NUM_FEATURES + 2]);
            uint8_t tmpl_ser    = pgm_read_byte(&template_ptr[(j-1)*NUM_FEATURES + 3]);

            // --- DELTA ENERGY MATH ---
            // Calculate the change in energy from the previous frame. 
            // If it is the first frame (i==1 or j==1), we default to 0 to avoid reading negative array indexes.
            int16_t live_delta_energy = (i > 1) ? ((int16_t)live_features[i-1][0] - (int16_t)live_features[i-2][0]) : 0;
            int16_t tmpl_delta_energy = (j > 1) ? ((int16_t)tmpl_energy - (int16_t)pgm_read_byte(&template_ptr[(j-2)*NUM_FEATURES + 0])) : 0;

            // Weighted distance:
            // 1. Delta Energy (Shape of the volume)
            // 2. ZCR (High vs Low frequency crossings)
            // 3. SSC (High frequency texture)
            // 4. SER (Sub-band frequency ratio)
            uint16_t energy_dist = abs(live_delta_energy - tmpl_delta_energy);
            uint16_t zcr_dist    = abs((int16_t)live_features[i-1][1] - tmpl_zcr);
            uint16_t ssc_dist    = abs((int16_t)live_features[i-1][2] - tmpl_ssc);
            uint16_t ser_dist    = abs((int16_t)live_features[i-1][3] - tmpl_ser);

            // You can tune these weights! Currently heavily favoring frequency features over raw volume.
            uint16_t dist = (energy_dist * 4) + (zcr_dist * 2) + (ssc_dist * 1) + (ser_dist * 2); // 1 3 2 3

            // Pick minimum predecessor
            uint16_t min_path = row_prev[j];
            if (row_curr[j-1] < min_path) min_path = row_curr[j-1];
            if (row_prev[j-1] < min_path) min_path = row_prev[j-1];

            // Guard against overflow before adding
            if (min_path == 65535) {
                row_curr[j] = 65535;
            } else {
                uint32_t total = (uint32_t)dist + min_path;
                row_curr[j] = (total > 65534) ? 65535 : (uint16_t)total;
            }
        }

        for (int k = 0; k <= tmpl_frames; k++) row_prev[k] = row_curr[k];
    }

    return row_curr[tmpl_frames];
}

// ==========================================
// MAIN
// ==========================================
int main(void) {
    UART_init();
    stdin = stdout = &uart_str;

    adc_init();
    timer1_init();

    LCD_Init();
    LCD_Clear();
    LCD_String("Vo System");
    LCD_Gotoxy(1, 0);
    LCD_String("Listening...");

    printf("=== System Ready ===\r\n");
    sei();

    while (1) {
        if (recording_display_pending && current_state == PROCESSING && DATA_COLLECTION_MODE) {
            LCD_Clear();
            LCD_String("Recording...");
            recording_display_pending = 0;
        }

        if (current_state == PROCESSING) {

            if (DATA_COLLECTION_MODE) {
                for (int i = 0; i < current_frame; i++) {
                    printf("{%d, %d, %d, %d},\r\n",
                           live_features[i][0],
                           live_features[i][1],
                           live_features[i][2],
                           live_features[i][3]);
                }
                printf("===\r\n");

            } else {
                uint16_t best_score = 65535;
                uint8_t  best_cmd   = 255;

                for (uint8_t c = 0; c < NUM_COMMANDS; c++) {
                    if (template_lengths[c] == 0) continue;

                    const uint8_t* temp_ptr =
                        (const uint8_t*)pgm_read_word(&command_templates[c]);

                    uint16_t score = calculate_dtw_2row(temp_ptr,template_lengths[c],current_frame);
                    if (score < best_score) {
                        best_score = score;
                        best_cmd   = c;
                    }
                }

                LCD_Clear();

                if (best_score < REJECTION_THRESHOLD) {
                    // Read command name from flash into a small buffer
                    char name_buf[8];
                    strcpy_P(name_buf,
                             (const char*)pgm_read_word(&command_names[best_cmd]));

                    LCD_String("Command:");
                    LCD_String_xy(1, 0, name_buf);
                    printf("Match: %s, Score: %u\r\n", name_buf, best_score);
                } else {
                    LCD_String("Unknown Noise");
                    printf("Rejected. Best score: %u\r\n", best_score);
                }

                _delay_ms(1500);

                LCD_Clear();
                LCD_String("Listening...");
            }

            // Reset state ? must happen after reading current_frame
            current_frame = 0;
            current_state = IDLE;
        }
    }
}