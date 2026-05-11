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
#define REJECTION_THRESHOLD  10000  // Generous ceiling for the DTW score
#define NOISE_FLOOR          40    // CRITICAL: Ignores all the background +/- 20 hum for ZCR/SS
#define DTW_BAND_DIVISOR     4     // Sakoe-Chiba band = longer_len / this value
#define SSC_SHIFT            1     // Right-shift SSC by this to prevent saturation (divide by 2)

// ==========================================
// TEMPLATES (PROGMEM)
// ==========================================

const uint8_t template_0[40][NUM_FEATURES] PROGMEM = {
{51, 61, 50, 149},
{79, 69, 54, 161},
{82, 74, 59, 168},
{96, 71, 55, 171},
{107, 75, 55, 169},
{107, 77, 56, 171},
{82, 71, 60, 160},
{79, 39, 47, 112},
{117, 28, 47, 113},
{82, 20, 51, 112},
{33, 1, 17, 57},
{28, 0, 19, 52},
{24, 0, 14, 46},
{23, 2, 13, 56},
{28, 0, 21, 47},
{28, 0, 18, 51},
{22, 0, 11, 41},
{52, 37, 45, 120},
{84, 74, 57, 161},
{86, 64, 57, 151},
{79, 74, 58, 154},
{83, 73, 57, 145},
{83, 65, 61, 150},
{79, 70, 61, 156},
{76, 63, 56, 160},
{67, 71, 56, 170},
{79, 68, 58, 145},
{70, 69, 56, 150},
{70, 72, 60, 147},
{51, 60, 57, 145},
{41, 6, 32, 83},
{29, 3, 22, 54},
{23, 0, 12, 48},
{24, 2, 16, 58},
{28, 0, 20, 41},
{28, 0, 17, 50},
{22, 0, 10, 50},
{24, 1, 12, 52},
{28, 0, 18, 44},
{30, 0, 20, 48}
}; // SIX

const uint8_t template_1[40][NUM_FEATURES] PROGMEM = {
{63, 13, 38, 107},
{66, 25, 33, 122},
{76, 22, 34, 127},
{82, 27, 28, 138},
{93, 18, 27, 125},
{81, 21, 24, 136},
{76, 11, 18, 135},
{66, 13, 29, 117},
{60, 6, 31, 109},
{51, 4, 25, 112},
{42, 2, 21, 96},
{43, 3, 23, 101},
{42, 1, 28, 87},
{37, 1, 20, 79},
{30, 1, 18, 74},
{27, 0, 18, 60},
{30, 0, 18, 55},
{32, 0, 18, 61},
{25, 0, 15, 46},
{23, 0, 11, 42},
{29, 0, 22, 45},
{28, 0, 17, 48},
{22, 0, 13, 45},
{22, 0, 12, 45},
{29, 0, 18, 41},
{27, 0, 14, 52},
{23, 0, 16, 48},
{24, 1, 13, 57},
{28, 0, 19, 44},
{28, 0, 20, 46},
{22, 0, 14, 47},
{22, 0, 13, 48},
{28, 0, 18, 41},
{30, 1, 14, 55},
{21, 0, 16, 44},
{24, 1, 12, 61},
{29, 0, 19, 42},
{28, 0, 19, 45},
{22, 0, 14, 44},
{22, 0, 13, 47}
}; // NO

const uint8_t template_2[40][NUM_FEATURES] PROGMEM = {
{53, 10, 16, 119},
{92, 19, 24, 139},
{108, 12, 27, 127},
{37, 1, 15, 80},
{27, 0, 17, 41},
{22, 0, 15, 49},
{28, 0, 19, 45},
{24, 1, 14, 57},
{27, 0, 21, 40},
{23, 0, 13, 47},
{28, 0, 18, 46},
{24, 1, 13, 56},
{28, 1, 19, 48},
{22, 0, 13, 45},
{28, 0, 20, 44},
{207, 37, 44, 94},
{154, 59, 39, 163},
{86, 44, 33, 159},
{47, 8, 31, 97},
{27, 0, 15, 66},
{28, 0, 19, 39},
{23, 0, 13, 45},
{28, 0, 16, 44},
{24, 0, 17, 52},
{28, 0, 18, 43},
{23, 1, 14, 46},
{28, 0, 22, 44},
{24, 0, 11, 52},
{27, 0, 17, 40},
{23, 0, 12, 46},
{28, 0, 20, 46},
{24, 0, 13, 46},
{26, 0, 22, 41},
{21, 0, 14, 41},
{28, 0, 16, 42},
{23, 0, 13, 49},
{27, 0, 20, 47},
{22, 0, 11, 50},
{27, 0, 22, 42},
{24, 0, 15, 47}
}; // UP

const uint8_t template_3[40][NUM_FEATURES] PROGMEM = {
{62, 65, 56, 152},
{75, 54, 54, 140},
{95, 74, 58, 166},
{99, 79, 56, 163},
{94, 75, 54, 166},
{88, 85, 63, 162},
{95, 74, 53, 167},
{90, 81, 60, 169},
{83, 76, 55, 173},
{61, 40, 46, 147},
{117, 16, 29, 120},
{145, 21, 27, 136},
{122, 21, 35, 136},
{118, 24, 32, 138},
{122, 20, 26, 138},
{132, 24, 23, 137},
{102, 13, 27, 120},
{48, 5, 22, 100},
{31, 0, 20, 51},
{26, 0, 17, 55},
{49, 13, 35, 104},
{73, 22, 45, 123},
{67, 41, 44, 128},
{44, 17, 31, 111},
{36, 6, 27, 78},
{23, 0, 12, 50},
{30, 0, 21, 51},
{26, 0, 13, 46},
{28, 0, 19, 41},
{26, 0, 8, 49},
{28, 0, 18, 46},
{24, 0, 17, 41},
{29, 0, 20, 44},
{23, 0, 11, 47},
{30, 0, 17, 48},
{21, 0, 10, 43},
{27, 0, 19, 43},
{23, 0, 12, 49},
{26, 0, 16, 45},
{22, 0, 13, 45}
}; // SEVEN

const uint8_t template_4[40][NUM_FEATURES] PROGMEM = {
{68, 61, 53, 147},
{82, 55, 54, 156},
{77, 82, 64, 160},
{79, 71, 58, 150},
{85, 77, 61, 148},
{70, 79, 59, 175},
{60, 67, 59, 152},
{28, 5, 24, 51},
{29, 0, 18, 44},
{23, 0, 14, 46},
{25, 1, 19, 54},
{32, 10, 20, 75},
{144, 29, 30, 128},
{193, 27, 28, 142},
{188, 31, 25, 147},
{187, 33, 24, 149},
{175, 33, 22, 149},
{159, 30, 19, 147},
{133, 22, 25, 136},
{129, 23, 23, 144},
{131, 22, 21, 144},
{96, 22, 24, 144},
{45, 11, 20, 133},
{31, 2, 24, 60},
{34, 1, 21, 52},
{23, 0, 12, 48},
{26, 0, 15, 49},
{27, 0, 18, 42},
{30, 0, 21, 52},
{22, 0, 13, 43},
{25, 0, 17, 47},
{27, 0, 21, 43},
{35, 12, 31, 90},
{41, 45, 53, 123},
{30, 8, 23, 65},
{28, 0, 20, 43},
{29, 0, 16, 46},
{23, 0, 15, 42},
{25, 0, 15, 50},
{27, 0, 16, 38}
}; // START

const uint8_t template_5[40][NUM_FEATURES] PROGMEM = {
{60, 16, 41, 111},
{79, 20, 38, 128},
{72, 29, 41, 140},
{72, 17, 36, 122},
{37, 5, 25, 73},
{46, 56, 50, 156},
{63, 74, 50, 156},
{54, 63, 55, 163},
{65, 59, 56, 141},
{62, 62, 52, 144},
{62, 70, 56, 147},
{60, 63, 49, 155},
{54, 55, 56, 143},
{49, 52, 52, 135},
{50, 49, 48, 144},
{50, 53, 61, 129},
{49, 57, 50, 135},
{46, 47, 52, 128},
{43, 36, 42, 132},
{39, 49, 47, 134},
{40, 39, 45, 114},
{40, 35, 43, 108},
{36, 25, 35, 112},
{36, 25, 33, 112},
{41, 31, 39, 109},
{36, 17, 35, 101},
{33, 22, 33, 99},
{31, 13, 27, 80},
{31, 3, 25, 66},
{33, 9, 27, 65},
{28, 7, 20, 58},
{24, 0, 15, 50},
{29, 0, 20, 43},
{28, 1, 17, 50},
{24, 0, 16, 45},
{23, 0, 11, 43},
{28, 0, 18, 43},
{29, 1, 21, 50},
{23, 0, 14, 48},
{24, 0, 16, 51}
}; // YES

const uint8_t template_6[40][NUM_FEATURES] PROGMEM = {
{60, 70, 58, 150},
{74, 73, 59, 148},
{77, 72, 60, 144},
{71, 92, 63, 170},
{82, 87, 65, 157},
{36, 30, 37, 109},
{23, 0, 13, 43},
{25, 0, 14, 47},
{27, 0, 20, 44},
{101, 24, 37, 118},
{187, 25, 30, 137},
{129, 19, 23, 130},
{30, 0, 20, 45},
{28, 0, 18, 45},
{22, 0, 15, 50},
{23, 0, 13, 50},
{28, 0, 22, 42},
{28, 0, 22, 51},
{22, 0, 12, 50},
{25, 0, 13, 52},
{27, 0, 19, 44},
{28, 0, 18, 47},
{28, 2, 20, 65},
{197, 25, 43, 93},
{67, 14, 26, 124},
{37, 4, 28, 65},
{24, 0, 19, 50},
{25, 0, 19, 47},
{27, 0, 18, 42},
{30, 1, 22, 56},
{22, 0, 15, 47},
{26, 2, 17, 57},
{27, 0, 17, 39},
{29, 0, 18, 45},
{22, 0, 11, 48},
{24, 0, 19, 50},
{27, 0, 17, 41},
{29, 0, 19, 49},
{23, 0, 13, 47},
{25, 0, 15, 54}
}; // STOP

const uint8_t template_7[40][NUM_FEATURES] PROGMEM = {
{57, 54, 51, 139},
{68, 52, 48, 154},
{69, 66, 49, 164},
{59, 49, 46, 131},
{59, 63, 52, 143},
{77, 24, 45, 94},
{83, 9, 42, 100},
{74, 18, 50, 97},
{82, 18, 48, 100},
{112, 24, 55, 109},
{137, 17, 44, 116},
{164, 20, 29, 125},
{111, 14, 32, 116},
{60, 15, 35, 116},
{119, 17, 37, 120},
{145, 21, 27, 137},
{169, 24, 22, 138},
{135, 23, 26, 138},
{126, 14, 28, 136},
{96, 15, 26, 130},
{72, 14, 24, 124},
{64, 10, 25, 113},
{54, 8, 25, 115},
{53, 8, 24, 109},
{38, 4, 24, 102},
{43, 2, 29, 71},
{31, 0, 22, 87},
{31, 0, 24, 57},
{27, 0, 14, 43},
{32, 1, 18, 51},
{22, 0, 12, 45},
{26, 0, 20, 45},
{24, 0, 20, 43},
{29, 1, 18, 47},
{22, 1, 16, 48},
{28, 0, 22, 46},
{24, 0, 18, 42},
{28, 0, 19, 47},
{22, 0, 11, 44},
{27, 0, 20, 44}
}; // ZERO

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
const char cmd_ZERO[]  PROGMEM = "ZERO";

const char* const command_names[NUM_COMMANDS] PROGMEM = {
    cmd_SIX, cmd_NO, cmd_UP, cmd_SEVEN,
    cmd_START,  cmd_YES, cmd_STOP, cmd_ZERO
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
    40,  // NO
    40,  // UP
    40,  // SEVEN
    40,  // START
    40,  // YES
    40,  // STOP
    40   // ZERO
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
    // AVcc reference, channel 5, free-running, prescaler 64
    ADMUX  = (1 << REFS0) | 0x05;
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
// ==========================================
// ISR: AUDIO SAMPLING & VAD
// ==========================================
ISR(TIMER1_COMPA_vect) {
    int16_t current_sample = ADC - 512;

    // --- Feature accumulation ---
    energy_accumulator += abs(current_sample);

    // 1. Always run the IIR filter so it decays naturally to 0 during silence
    // low = low * (3/4) + sample * (1/4) ? integer shifts only
    ser_low_state = ser_low_state - (ser_low_state >> 2) + (current_sample >> 2);

    // 2. Only accumulate SER energy if the signal breaks the noise floor
    if (abs(current_sample) > NOISE_FLOOR) {
        int16_t ser_high = current_sample - ser_low_state;
        low_energy_acc  += abs(ser_low_state);
        high_energy_acc += abs(ser_high);
    }

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
        uint8_t frame_ssc    = ssc_accumulator >> SSC_SHIFT;

        // SER: high-band fraction of total energy, scaled to 0-255
        uint32_t total_band = low_energy_acc + high_energy_acc;
        uint8_t  frame_ser  = (total_band > 0)
                            ? (uint8_t)((high_energy_acc * 255UL) / total_band)
                            : 128;  // default to mid-range (neutral) during silence

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
        // NOTE: ser_low_state is intentionally NOT reset ? running IIR filter state
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