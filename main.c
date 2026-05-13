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

// --- SCALABILITY DEFINES ---
#define NUM_COMMANDS      8
#define TEMPLATES_PER_CMD 4     // Increased to 4 templates per command!
#define TOTAL_TEMPLATES   (NUM_COMMANDS * TEMPLATES_PER_CMD) // 32 Total

#define START_THRESHOLD      50    
#define END_THRESHOLD        25    // Raised to 35 to cut off breathy tails
#define SILENCE_FRAMES       12    
#define PREROLL_FRAMES       2     
#define REJECTION_THRESHOLD  6000 
#define NOISE_FLOOR          40    
#define DTW_BAND_DIVISOR     3     // Widened to allow more time-warping
#define SSC_SHIFT            1     

// ==========================================
// TEMPLATES (PROGMEM)
// ==========================================

// --- 0: SIX ---
const uint8_t template_SIX_1[][NUM_FEATURES] PROGMEM = {
{83, 68, 54, 175},
{100, 76, 55, 178},
{95, 89, 58, 178},
{97, 78, 59, 170},
{99, 105, 62, 187},
{98, 77, 63, 170},
{95, 80, 56, 160},
{88, 94, 61, 174},
{95, 83, 59, 173},
{98, 76, 57, 167},
{104, 80, 57, 172},
{110, 68, 57, 151},
{212, 31, 44, 116},
{245, 21, 44, 116},
{120, 42, 37, 125},
{11, 1, 2, 183},
{8, 0, 0, 128},
{10, 0, 0, 128},
{10, 0, 0, 128},
{9, 0, 0, 128},
{36, 22, 15, 181},
{88, 83, 61, 179},
{99, 83, 59, 174},
{100, 82, 63, 169},
{102, 87, 56, 178},
{101, 71, 58, 167},
{93, 81, 61, 167},
{102, 80, 58, 175},
{99, 84, 60, 172},
{103, 72, 57, 161},
{100, 79, 55, 175},
{92, 75, 60, 174},
{86, 82, 63, 179},
{81, 76, 56, 180},
{83, 79, 51, 175},
{67, 74, 52, 176},
{31, 31, 28, 185},
{14, 1, 4, 106},
{10, 0, 0, 128},
{9, 0, 0, 128}
};

const uint8_t template_SIX_2[][NUM_FEATURES] PROGMEM = {
{62, 77, 49, 179},
{89, 75, 53, 177},
{96, 74, 52, 176},
{103, 70, 56, 163},
{130, 56, 60, 141},
{249, 28, 37, 129},
{156, 29, 41, 131},
{15, 0, 4, 107},
{12, 1, 0, 195},
{9, 0, 0, 128},
{13, 0, 1, 188},
{69, 67, 47, 182},
{101, 91, 56, 174},
{115, 75, 60, 165},
{117, 78, 54, 170},
{99, 74, 57, 183},
{97, 82, 61, 175},
{73, 69, 56, 167},
{55, 83, 52, 181},
{39, 52, 38, 184},
{16, 12, 7, 175},
{9, 0, 0, 128},
{8, 0, 0, 128},
{8, 0, 0, 128},
{8, 1, 1, 190},
{8, 0, 0, 128},
{8, 0, 0, 128},
{8, 0, 0, 128},
{7, 0, 0, 128},
{8, 0, 0, 128},
{7, 0, 0, 128},
{7, 0, 0, 128}
};

const uint8_t template_SIX_3[][NUM_FEATURES] PROGMEM = { 
{53, 18, 34, 102},
{124, 26, 45, 119},
{142, 33, 45, 122},
{72, 21, 40, 118},
{13, 1, 1, 139},
{10, 0, 0, 128},
{11, 0, 1, 142},
{9, 0, 2, 151},
{11, 0, 1, 165},
{24, 26, 24, 187},
{22, 20, 20, 174},
{24, 33, 24, 184},
{25, 34, 25, 177},
{25, 25, 20, 173},
{19, 12, 12, 183},
{21, 22, 20, 176},
{19, 12, 15, 167},
{16, 5, 5, 139},
{12, 2, 1, 153},
{12, 1, 1, 151},
{10, 1, 1, 195},
{8, 0, 0, 128},
{8, 0, 0, 128},
{8, 0, 0, 128},
{8, 0, 0, 128}
};

const uint8_t template_SIX_4[][NUM_FEATURES] PROGMEM = { 
{63, 76, 53, 188},
{75, 68, 52, 173},
{86, 79, 61, 170},
{98, 87, 54, 172},
{91, 78, 57, 173},
{106, 77, 57, 172},
{79, 66, 56, 152},
{161, 27, 47, 105},
{191, 23, 49, 114},
{115, 19, 48, 107},
{8, 0, 0, 128},
{7, 0, 0, 128},
{8, 1, 0, 190},
{6, 0, 0, 128},
{6, 0, 0, 128},
{55, 68, 43, 186},
{82, 87, 55, 179},
{82, 75, 57, 160},
{84, 79, 60, 172},
{80, 85, 58, 183},
{81, 75, 59, 169},
{80, 83, 61, 170},
{75, 74, 55, 168},
{66, 76, 63, 171},
{59, 65, 53, 158},
{49, 72, 48, 190},
{32, 49, 33, 189},
{19, 21, 15, 175},
{10, 0, 2, 106},
{7, 0, 0, 128},
{7, 0, 0, 128},
{7, 0, 0, 128},
{7, 0, 0, 128},
{7, 0, 0, 128},
{7, 0, 0, 128},
{7, 0, 0, 128},
{6, 0, 0, 128}
};

// --- 1: NO ---
const uint8_t template_NO_1[][NUM_FEATURES] PROGMEM = {
{53, 3, 42, 83},
{52, 6, 46, 91},
{53, 4, 33, 87},
{52, 2, 34, 92},
{55, 1, 38, 91},
{61, 4, 33, 94},
{178, 36, 48, 129},
{208, 41, 42, 133},
{222, 31, 40, 136},
{241, 42, 28, 151},
{248, 33, 30, 145},
{2, 30, 27, 138},
{2, 27, 32, 137},
{16, 28, 34, 137},
{14, 26, 29, 132},
{23, 20, 32, 129},
{25, 20, 30, 127},
{31, 19, 31, 123},
{14, 19, 28, 124},
{253, 23, 26, 131},
{254, 22, 26, 133},
{238, 22, 26, 130},
{212, 26, 23, 132},
{212, 19, 22, 126},
{191, 18, 20, 125},
{180, 15, 17, 123},
{154, 16, 21, 123},
{152, 13, 17, 124},
{129, 13, 18, 120},
{120, 13, 17, 123},
{68, 7, 19, 129},
{27, 1, 10, 119},
{19, 1, 5, 144},
{13, 0, 2, 140},
{35, 1, 15, 101},
{18, 0, 5, 61},
{10, 0, 0, 128},
{10, 0, 0, 128},
{11, 0, 0, 128},
{11, 1, 0, 194}
};

const uint8_t template_NO_2[][NUM_FEATURES] PROGMEM = {
{174, 25, 41, 115},
{200, 27, 32, 131},
{222, 25, 30, 131},
{223, 26, 26, 133},
{231, 27, 28, 135},
{220, 24, 31, 134},
{205, 22, 23, 129},
{161, 20, 26, 125},
{69, 8, 25, 122},
{44, 3, 27, 108},
{23, 1, 11, 112},
{12, 4, 1, 200},
{10, 0, 0, 128},
{17, 1, 2, 155},
{10, 0, 0, 128},
{9, 0, 0, 128},
{9, 0, 0, 128},
{11, 2, 0, 200},
{9, 0, 0, 128},
{10, 0, 1, 193},
{10, 0, 0, 128},
{9, 0, 0, 128}
};

const uint8_t template_NO_3[][NUM_FEATURES] PROGMEM = { 
{71, 16, 32, 104},
{115, 21, 34, 126},
{147, 34, 29, 127},
{166, 34, 30, 130},
{179, 29, 27, 133},
{177, 24, 28, 126},
{141, 18, 29, 120},
{121, 14, 28, 118},
{67, 6, 29, 112},
{52, 3, 28, 103},
{41, 2, 23, 101},
{26, 3, 18, 126},
{17, 0, 7, 124},
{17, 2, 5, 139},
{11, 0, 0, 128},
{11, 0, 0, 128},
{11, 0, 0, 128},
{12, 2, 1, 216},
{10, 0, 0, 128},
{11, 1, 0, 177},
{12, 1, 2, 195},
{13, 0, 2, 187},
{10, 0, 0, 128},
{10, 0, 0, 128}
};

const uint8_t template_NO_4[][NUM_FEATURES] PROGMEM = { 
{70, 9, 28, 108},
{138, 20, 36, 125},
{179, 19, 33, 132},
{197, 27, 25, 134},
{205, 19, 22, 128},
{229, 17, 21, 125},
{225, 19, 17, 126},
{210, 17, 19, 127},
{165, 14, 13, 126},
{125, 15, 16, 126},
{41, 1, 14, 110},
{19, 1, 5, 98},
{10, 0, 0, 128},
{8, 0, 0, 128},
{8, 0, 0, 128},
{6, 0, 0, 128},
{6, 0, 0, 128},
{7, 0, 0, 128},
{6, 0, 0, 128},
{6, 0, 0, 128},
{7, 0, 0, 128},
{6, 0, 0, 128},
{6, 0, 0, 128}
};

// --- 2: UP ---
const uint8_t template_UP_1[][NUM_FEATURES] PROGMEM = {
{125, 30, 29, 159},
{251, 37, 27, 160},
{24, 33, 25, 156},
{246, 27, 29, 140},
{23, 1, 15, 109},
{13, 0, 0, 161},
{13, 0, 3, 183},
{10, 0, 1, 136},
{10, 0, 0, 128},
{10, 0, 0, 128},
{11, 0, 0, 128},
{10, 0, 1, 124},
{11, 1, 1, 187},
{8, 0, 0, 128},
{145, 11, 20, 38},
{215, 21, 41, 54},
{185, 46, 40, 118},
{105, 30, 25, 145},
{32, 6, 22, 124},
{13, 0, 3, 107},
{11, 0, 1, 160},
{9, 0, 0, 128},
{8, 0, 0, 128},
{7, 0, 0, 128},
{8, 0, 0, 128},
{9, 1, 1, 185},
{8, 0, 0, 128},
{8, 1, 1, 189},
{7, 0, 0, 128},
{9, 2, 1, 186},
{8, 0, 0, 128}
};

const uint8_t template_UP_2[][NUM_FEATURES] PROGMEM = {
{235, 39, 30, 158},
{250, 32, 29, 145},
{93, 5, 25, 110},
{15, 0, 3, 104},
{10, 0, 0, 128},
{12, 1, 1, 170},
{10, 0, 0, 105},
{10, 0, 0, 223},
{10, 0, 0, 128},
{9, 0, 0, 128},
{8, 0, 0, 128},
{9, 0, 0, 128},
{9, 0, 0, 128},
{8, 0, 0, 128},
{8, 0, 0, 128}
};

const uint8_t template_UP_3[][NUM_FEATURES] PROGMEM = { 
{115, 24, 23, 152},
{96, 18, 26, 141},
{56, 7, 17, 116},
{20, 0, 10, 67},
{12, 1, 1, 175},
{9, 0, 0, 128},
{8, 0, 0, 128},
{7, 0, 0, 128},
{7, 0, 0, 128},
{8, 0, 0, 128},
{8, 0, 0, 128},
{7, 0, 0, 128},
{9, 0, 0, 186},
{7, 0, 0, 128},
{26, 2, 3, 137},
{159, 33, 36, 136},
{134, 32, 26, 149},
{65, 10, 25, 133},
{24, 1, 12, 105},
{16, 0, 5, 83},
{10, 0, 0, 128},
{9, 1, 0, 190},
{9, 0, 0, 128},
{8, 2, 1, 194},
{7, 0, 0, 128},
{7, 0, 0, 128},
{7, 0, 0, 128},
{7, 0, 0, 128},
{6, 0, 0, 128},
{7, 0, 0, 128}
};

const uint8_t template_UP_4[][NUM_FEATURES] PROGMEM = { 
{252, 33, 30, 144},
{83, 9, 19, 108},
{10, 0, 0, 128},
{8, 0, 0, 128},
{8, 0, 0, 128},
{8, 0, 0, 128},
{8, 0, 0, 128},
{7, 0, 0, 128},
{7, 0, 0, 128},
{6, 0, 0, 128},
{7, 0, 0, 128},
{6, 0, 0, 128},
{6, 0, 0, 128},
{6, 0, 0, 128}
};

// --- 3: SEVEN ---
const uint8_t template_SEVEN_1[][NUM_FEATURES] PROGMEM = {
{79, 82, 54, 184},
{87, 78, 60, 175},
{96, 85, 55, 183},
{99, 95, 56, 183},
{94, 86, 60, 170},
{109, 83, 58, 174},
{86, 74, 61, 162},
{91, 79, 59, 176},
{96, 76, 52, 163},
{84, 88, 61, 169},
{148, 31, 43, 122},
{234, 28, 34, 138},
{242, 34, 33, 137},
{242, 30, 33, 147},
{1, 29, 25, 148},
{9, 31, 27, 145},
{10, 28, 25, 142},
{8, 27, 29, 136},
{210, 25, 31, 126},
{98, 26, 40, 132},
{55, 41, 48, 138},
{68, 42, 39, 160},
{123, 30, 35, 131},
{198, 33, 32, 140},
{150, 28, 34, 134},
{115, 42, 37, 143},
{93, 34, 40, 132},
{61, 13, 24, 139},
{41, 12, 13, 124},
{41, 11, 18, 123},
{63, 22, 25, 119},
{37, 13, 15, 151},
{45, 13, 13, 126},
{39, 18, 17, 135},
{38, 14, 12, 137},
{32, 12, 10, 152},
{33, 19, 10, 151},
{35, 16, 13, 140},
{29, 18, 11, 156},
{26, 12, 9, 145}
};

const uint8_t template_SEVEN_2[][NUM_FEATURES] PROGMEM = {
{52, 68, 47, 186},
{67, 90, 57, 179},
{75, 78, 57, 174},
{69, 74, 56, 157},
{82, 73, 52, 170},
{91, 76, 55, 179},
{67, 61, 52, 163},
{129, 21, 35, 119},
{205, 27, 37, 137},
{178, 30, 29, 141},
{161, 26, 33, 141},
{147, 19, 28, 134},
{41, 3, 18, 111},
{30, 11, 29, 126},
{34, 8, 27, 98},
{105, 24, 40, 129},
{149, 29, 43, 134},
{92, 28, 38, 133},
{46, 9, 25, 123},
{17, 0, 7, 82},
{16, 0, 5, 93},
{15, 0, 4, 93},
{12, 0, 0, 128},
{9, 0, 0, 128},
{8, 0, 0, 128},
{8, 0, 0, 128},
{9, 0, 0, 128},
{9, 1, 0, 188},
{7, 0, 0, 128},
{9, 1, 1, 188},
{8, 0, 0, 128}
};

const uint8_t template_SEVEN_3[][NUM_FEATURES] PROGMEM = { 
{82, 16, 33, 129},
{155, 25, 31, 140},
{154, 24, 26, 139},
{160, 25, 27, 134},
{71, 20, 29, 125},
{25, 2, 17, 97},
{14, 0, 1, 87},
{20, 2, 9, 102},
{68, 16, 33, 121},
{58, 9, 38, 129},
{38, 5, 18, 114},
{26, 0, 8, 82},
{24, 0, 13, 75},
{23, 1, 14, 83},
{22, 0, 14, 82},
{20, 0, 12, 84},
{17, 1, 4, 137},
{18, 0, 7, 66},
{13, 0, 1, 72},
{11, 0, 2, 150},
{11, 1, 1, 193},
{9, 0, 0, 128},
{7, 0, 0, 128},
{7, 0, 0, 128}
};

const uint8_t template_SEVEN_4[][NUM_FEATURES] PROGMEM = { 
{70, 76, 50, 186},
{94, 66, 49, 169},
{95, 69, 52, 167},
{100, 81, 52, 177},
{100, 80, 53, 174},
{103, 64, 53, 173},
{96, 69, 53, 173},
{96, 76, 53, 165},
{124, 48, 46, 128},
{247, 23, 30, 125},
{246, 25, 36, 125},
{1, 24, 24, 136},
{241, 21, 28, 124},
{119, 21, 25, 131},
{39, 10, 24, 101},
{41, 32, 35, 143},
{47, 22, 33, 136},
{135, 25, 32, 131},
{147, 26, 36, 134},
{85, 30, 34, 129},
{14, 2, 3, 201},
{12, 0, 1, 94},
{10, 0, 0, 128},
{8, 0, 0, 128},
{8, 0, 0, 128},
{7, 0, 0, 128},
{6, 0, 0, 128},
{6, 0, 0, 128},
{6, 0, 0, 128},
{6, 0, 0, 128},
{7, 0, 0, 186},
{7, 0, 0, 128} 
};

// --- 4: START ---
const uint8_t template_START_1[][NUM_FEATURES] PROGMEM = {
{84, 82, 61, 175},
{103, 73, 56, 174},
{117, 85, 56, 183},
{104, 74, 57, 168},
{112, 79, 57, 155},
{110, 86, 58, 162},
{93, 78, 51, 187},
{18, 18, 10, 177},
{8, 0, 0, 128},
{8, 0, 0, 128},
{7, 0, 0, 128},
{8, 0, 0, 128},
{56, 35, 33, 142},
{3, 25, 30, 125},
{5, 33, 29, 141},
{1, 36, 27, 146},
{2, 37, 26, 144},
{16, 37, 28, 144},
{22, 35, 29, 142},
{1, 44, 33, 147},
{12, 33, 25, 143},
{231, 29, 28, 139},
{0, 25, 29, 134},
{123, 21, 24, 143},
{108, 22, 31, 133},
{63, 11, 23, 124},
{37, 8, 25, 90},
{11, 2, 1, 195},
{9, 0, 0, 128},
{9, 0, 0, 128},
{9, 0, 0, 128},
{8, 0, 0, 128},
{8, 0, 0, 128},
{11, 1, 3, 171},
{85, 75, 51, 171},
{123, 77, 61, 164},
{91, 49, 56, 135},
{31, 2, 12, 117},
{8, 0, 0, 128},
{8, 0, 0, 128}
};

const uint8_t template_START_2[][NUM_FEATURES] PROGMEM = {
{66, 66, 50, 173},
{95, 87, 62, 173},
{100, 93, 54, 183},
{101, 90, 58, 163},
{99, 87, 60, 170},
{92, 86, 63, 168},
{24, 25, 29, 167},
{10, 0, 0, 128},
{9, 0, 0, 128},
{10, 1, 0, 192},
{9, 0, 0, 128},
{141, 40, 42, 132},
{19, 23, 32, 138},
{16, 27, 26, 139},
{253, 28, 29, 140},
{252, 33, 25, 147},
{253, 28, 26, 142},
{136, 19, 28, 126},
{93, 22, 29, 134},
{86, 20, 28, 131},
{16, 7, 6, 192},
{12, 2, 1, 184},
{11, 0, 1, 121},
{11, 2, 0, 194},
{10, 0, 0, 128},
{11, 0, 0, 128},
{11, 0, 0, 177},
{15, 10, 5, 193},
{54, 66, 47, 179},
{102, 83, 58, 159},
{89, 73, 56, 160},
{47, 24, 38, 124},
{16, 0, 4, 106},
{10, 0, 0, 128},
{9, 0, 0, 128},
{10, 0, 0, 128},
{10, 0, 0, 128},
{9, 0, 0, 128},
{8, 0, 0, 128},
{9, 0, 0, 128}
};

const uint8_t template_START_3[][NUM_FEATURES] PROGMEM = { 
{93, 18, 35, 137},
{99, 22, 27, 142},
{108, 25, 25, 149},
{112, 26, 24, 145},
{101, 21, 30, 145},
{105, 26, 24, 148},
{121, 26, 21, 146},
{110, 23, 22, 151},
{112, 20, 26, 145},
{94, 16, 29, 137},
{91, 19, 34, 134},
{37, 7, 30, 141},
{16, 6, 11, 183},
{14, 2, 1, 160},
{15, 3, 3, 177},
{14, 5, 6, 185},
{13, 1, 1, 155},
{13, 1, 2, 148},
{11, 1, 1, 154},
{11, 0, 0, 128},
{13, 2, 0, 197},
{11, 0, 0, 128},
{10, 0, 0, 128},
{11, 0, 0, 128} 
};

const uint8_t template_START_4[][NUM_FEATURES] PROGMEM = { 
{69, 95, 60, 177},
{102, 83, 58, 170},
{104, 96, 62, 173},
{105, 76, 59, 163},
{111, 83, 66, 162},
{96, 91, 63, 174},
{32, 52, 33, 177},
{8, 0, 0, 128},
{7, 0, 0, 128},
{12, 4, 4, 175},
{84, 54, 48, 148},
{5, 21, 31, 124},
{17, 27, 24, 140},
{10, 26, 29, 142},
{4, 27, 31, 141},
{11, 27, 29, 139},
{15, 26, 30, 136},
{9, 27, 31, 139},
{244, 27, 33, 138},
{234, 25, 29, 135},
{81, 9, 15, 127},
{69, 11, 22, 129},
{47, 5, 23, 104},
{15, 2, 6, 99},
{8, 0, 0, 128},
{8, 0, 0, 128},
{7, 0, 0, 128},
{7, 0, 0, 128},
{7, 0, 0, 128},
{8, 0, 0, 128},
{7, 0, 0, 128},
{10, 2, 1, 181},
{7, 0, 0, 128},
{7, 0, 0, 128},
{48, 42, 33, 145},
{161, 54, 53, 137},
{104, 33, 39, 125},
{36, 10, 19, 137},
{10, 0, 0, 143},
{7, 0, 0, 128}
};

// --- 5: YES ---
const uint8_t template_YES_1[][NUM_FEATURES] PROGMEM = {
{56, 23, 48, 113},
{88, 18, 41, 124},
{108, 19, 37, 130},
{123, 27, 37, 139},
{127, 19, 26, 138},
{48, 27, 40, 127},
{80, 73, 50, 178},
{73, 70, 53, 178},
{73, 71, 53, 173},
{77, 70, 58, 175},
{83, 71, 57, 172},
{82, 82, 59, 174},
{78, 82, 56, 171},
{78, 69, 55, 172},
{78, 78, 55, 175},
{82, 69, 57, 165},
{82, 79, 57, 172},
{76, 74, 52, 175},
{81, 78, 53, 176},
{83, 81, 57, 170},
{77, 61, 50, 178},
{77, 73, 51, 180},
{72, 71, 48, 170},
{74, 81, 54, 175},
{77, 79, 53, 176},
{84, 68, 56, 184},
{83, 79, 55, 181},
{76, 74, 54, 176},
{69, 68, 48, 178},
{61, 72, 52, 163},
{72, 86, 53, 190},
{61, 64, 50, 173},
{55, 62, 47, 168},
{43, 58, 39, 177},
{23, 30, 21, 178},
{13, 1, 1, 180},
{11, 0, 0, 128},
{10, 1, 1, 185},
{9, 0, 0, 128},
{11, 1, 1, 201}
};

const uint8_t template_YES_2[][NUM_FEATURES] PROGMEM = {
{93, 19, 52, 100},
{206, 20, 36, 118},
{222, 22, 37, 132},
{242, 27, 28, 135},
{111, 30, 41, 131},
{92, 83, 59, 180},
{98, 66, 55, 172},
{97, 69, 54, 160},
{96, 76, 59, 165},
{95, 78, 55, 178},
{102, 73, 54, 180},
{99, 70, 54, 167},
{84, 70, 55, 170},
{69, 76, 53, 175},
{55, 67, 51, 161},
{41, 48, 39, 170},
{17, 6, 6, 186},
{10, 1, 2, 145},
{10, 0, 0, 128},
{10, 0, 0, 128},
{11, 1, 1, 186},
{9, 0, 0, 128},
{10, 0, 0, 128},
{10, 0, 0, 128},
{10, 1, 1, 187},
{9, 0, 0, 128},
{9, 0, 0, 128},
{9, 0, 0, 128}
};

const uint8_t template_YES_3[][NUM_FEATURES] PROGMEM = { 
{65, 7, 44, 110},
{79, 17, 37, 131},
{89, 17, 38, 136},
{81, 18, 38, 135},
{36, 5, 27, 122},
{18, 12, 11, 176},
{26, 30, 27, 182},
{26, 31, 20, 174},
{27, 37, 23, 183},
{29, 36, 29, 185},
{26, 39, 29, 178},
{26, 37, 33, 180},
{25, 27, 26, 172},
{30, 36, 31, 180},
{21, 21, 20, 174},
{19, 13, 12, 170},
{20, 16, 13, 179},
{18, 20, 11, 184},
{14, 8, 5, 190},
{13, 3, 3, 176},
{10, 2, 0, 202},
{10, 0, 0, 128},
{10, 0, 0, 128},
{13, 0, 0, 128},
{11, 0, 0, 128},
{11, 0, 0, 128}
};

const uint8_t template_YES_4[][NUM_FEATURES] PROGMEM = {
{80, 24, 45, 115},
{131, 26, 43, 132},
{193, 30, 36, 142},
{142, 35, 39, 141},
{41, 19, 35, 116},
{64, 79, 54, 178},
{65, 94, 60, 185},
{64, 88, 59, 172},
{64, 82, 53, 180},
{78, 88, 62, 176},
{74, 102, 58, 184},
{80, 88, 59, 183},
{76, 80, 55, 179},
{74, 92, 61, 177},
{72, 86, 56, 181},
{66, 70, 60, 180},
{58, 74, 59, 182},
{58, 83, 56, 174},
{51, 69, 49, 173},
{48, 72, 48, 173},
{42, 68, 50, 162},
{33, 49, 38, 169},
{16, 14, 8, 167},
{9, 0, 0, 128},
{7, 0, 0, 128},
{7, 0, 0, 128},
{8, 0, 0, 128},
{7, 0, 0, 128},
{8, 0, 0, 128},
{7, 0, 0, 128},
{8, 0, 0, 188},
{7, 0, 0, 128},
{7, 0, 0, 128},
{7, 0, 0, 128}
};

// --- 6: STOP ---
const uint8_t template_STOP_1[][NUM_FEATURES] PROGMEM = {
{60, 81, 56, 184},
{87, 70, 56, 161},
{91, 69, 58, 164},
{99, 94, 63, 171},
{97, 80, 60, 169},
{95, 90, 61, 159},
{84, 80, 59, 167},
{81, 100, 65, 174},
{33, 37, 29, 180},
{11, 1, 1, 189},
{9, 0, 0, 128},
{10, 1, 0, 194},
{9, 0, 0, 128},
{10, 0, 0, 128},
{30, 15, 14, 178},
{233, 29, 38, 126},
{252, 27, 32, 133},
{195, 22, 28, 130},
{18, 0, 7, 71},
{14, 0, 2, 145},
{12, 0, 0, 128},
{12, 0, 2, 124},
{11, 0, 0, 128},
{11, 0, 0, 128},
{12, 0, 1, 186},
{10, 0, 0, 128},
{11, 0, 0, 128},
{14, 1, 2, 105},
{230, 9, 38, 33},
{218, 28, 39, 95},
{86, 15, 32, 118},
{26, 5, 17, 124},
{14, 1, 2, 179},
{11, 1, 2, 205},
{11, 0, 0, 128},
{11, 0, 0, 128},
{12, 1, 0, 196},
{11, 0, 0, 128},
{12, 1, 0, 186},
{9, 0, 0, 128}
};

const uint8_t template_STOP_2[][NUM_FEATURES] PROGMEM = {
{72, 80, 54, 172},
{98, 80, 55, 175},
{102, 79, 58, 168},
{95, 78, 53, 168},
{99, 87, 59, 177},
{105, 80, 57, 164},
{75, 74, 60, 154},
{17, 24, 13, 195},
{14, 5, 4, 154},
{11, 2, 0, 213},
{76, 37, 30, 151},
{17, 25, 36, 136},
{11, 28, 35, 138},
{248, 21, 32, 132},
{18, 0, 13, 94},
{14, 4, 3, 195},
{14, 1, 3, 157},
{11, 0, 1, 180},
{12, 2, 1, 189},
{11, 0, 0, 128},
{11, 1, 1, 152},
{13, 4, 3, 171},
{10, 0, 0, 128},
{12, 2, 1, 176},
{13, 4, 2, 179},
{11, 2, 0, 197}
};

const uint8_t template_STOP_3[][NUM_FEATURES] PROGMEM = { 
{78, 92, 65, 180},
{80, 84, 61, 181},
{89, 79, 56, 175},
{79, 75, 54, 176},
{49, 63, 44, 176},
{16, 12, 8, 184},
{9, 0, 0, 128},
{66, 51, 35, 173},
{241, 25, 35, 112},
{14, 23, 28, 127},
{25, 24, 36, 132},
{4, 22, 32, 130},
{201, 22, 27, 121},
{29, 2, 14, 104},
{14, 0, 4, 144},
{14, 0, 1, 103},
{12, 0, 0, 128},
{13, 0, 1, 121},
{11, 0, 0, 128},
{10, 0, 0, 128},
{11, 0, 0, 128},
{10, 0, 0, 128},
{9, 0, 0, 128},
{9, 0, 0, 128},
{50, 20, 30, 123},
{38, 10, 21, 143},
{24, 2, 12, 115},
{13, 0, 2, 152},
{9, 0, 0, 128},
{12, 1, 1, 192},
{9, 0, 0, 128},
{11, 0, 0, 128},
{10, 0, 0, 128},
{9, 0, 0, 128},
{9, 0, 0, 128},
{9, 0, 0, 128},
{8, 0, 0, 128},
{9, 0, 0, 128}
};

const uint8_t template_STOP_4[][NUM_FEATURES] PROGMEM = { 
{66, 101, 65, 184},
{71, 90, 63, 160},
{68, 81, 63, 156},
{27, 46, 28, 195},
{7, 0, 0, 128},
{6, 0, 0, 128},
{6, 0, 0, 128},
{5, 0, 0, 128},
{126, 22, 30, 124},
{246, 22, 30, 128},
{241, 22, 29, 129},
{97, 8, 15, 120},
{7, 0, 0, 128},
{6, 0, 0, 128},
{6, 0, 0, 128},
{7, 0, 0, 128},
{6, 0, 0, 128},
{6, 0, 0, 128},
{6, 0, 0, 128},
{7, 0, 0, 128},
{6, 0, 0, 128},
{6, 0, 0, 128},
{6, 0, 0, 128},
{7, 0, 1, 185}
};

// --- 7: ZERO ---
const uint8_t template_ZERO_1[][NUM_FEATURES] PROGMEM = {
{55, 18, 43, 126},
{69, 55, 48, 158},
{86, 73, 51, 173},
{89, 80, 56, 174},
{101, 76, 52, 175},
{113, 70, 56, 167},
{105, 76, 55, 175},
{106, 59, 60, 135},
{160, 49, 58, 114},
{132, 63, 63, 125},
{115, 61, 63, 127},
{127, 70, 66, 121},
{124, 69, 62, 120},
{152, 48, 61, 118},
{193, 39, 53, 114},
{0, 20, 37, 117},
{241, 19, 36, 109},
{117, 17, 34, 110},
{213, 18, 32, 110},
{245, 21, 30, 125},
{214, 20, 26, 129},
{205, 27, 24, 134},
{200, 22, 21, 130},
{125, 14, 22, 126},
{110, 14, 30, 126},
{76, 8, 26, 127},
{37, 3, 19, 126},
{30, 2, 16, 121},
{20, 1, 10, 137},
{11, 0, 2, 104},
{10, 2, 0, 200},
{9, 0, 0, 128},
{9, 0, 0, 128},
{9, 0, 0, 128},
{10, 2, 0, 192},
{9, 0, 0, 128},
{8, 0, 0, 128},
{9, 0, 0, 128},
{11, 0, 1, 185},
{10, 0, 0, 128}
};

const uint8_t template_ZERO_2[][NUM_FEATURES] PROGMEM = {
{56, 69, 51, 177},
{62, 61, 49, 166},
{76, 78, 55, 185},
{76, 81, 60, 171},
{86, 89, 61, 178},
{77, 81, 59, 174},
{78, 86, 56, 175},
{79, 75, 57, 171},
{85, 75, 59, 176},
{81, 74, 60, 159},
{90, 73, 56, 164},
{85, 82, 60, 180},
{89, 55, 57, 134},
{132, 22, 57, 100},
{144, 32, 54, 105},
{154, 40, 60, 109},
{157, 33, 64, 112},
{163, 38, 61, 112},
{176, 34, 53, 109},
{192, 24, 59, 108},
{188, 27, 51, 110},
{192, 26, 43, 120},
{112, 23, 39, 120},
{207, 19, 25, 123},
{202, 22, 20, 129},
{178, 21, 21, 131},
{156, 17, 19, 133},
{159, 21, 20, 136},
{126, 18, 23, 130},
{113, 13, 24, 126},
{86, 9, 18, 127},
{52, 5, 25, 122},
{21, 2, 12, 137},
{10, 0, 0, 128},
{12, 1, 1, 155},
{10, 0, 1, 149},
{9, 0, 0, 128},
{10, 2, 0, 192},
{9, 0, 0, 128},
{9, 0, 0, 128}
};

const uint8_t template_ZERO_3[][NUM_FEATURES] PROGMEM = { 
{56, 14, 43, 100},
{80, 13, 49, 91},
{96, 25, 56, 96},
{99, 36, 64, 99},
{107, 38, 53, 114},
{136, 35, 45, 126},
{193, 29, 43, 121},
{193, 17, 32, 115},
{98, 15, 24, 116},
{73, 14, 35, 107},
{195, 18, 33, 113},
{183, 16, 26, 121},
{175, 18, 24, 116},
{184, 16, 23, 119},
{171, 15, 20, 121},
{149, 15, 22, 125},
{124, 11, 23, 122},
{107, 12, 34, 116},
{89, 7, 33, 110},
{70, 7, 27, 110},
{57, 3, 17, 106},
{41, 3, 18, 105},
{34, 0, 22, 108},
{26, 0, 15, 111},
{13, 0, 3, 107},
{9, 0, 0, 128},
{10, 0, 0, 128},
{9, 0, 0, 128},
{10, 2, 1, 201},
{10, 0, 0, 128},
{10, 0, 0, 128},
{10, 0, 0, 128},
{9, 0, 0, 128},
{9, 0, 0, 128},
{10, 0, 0, 187},
{10, 0, 0, 128} 
};

const uint8_t template_ZERO_4[][NUM_FEATURES] PROGMEM = { 
{59, 82, 51, 175},
{80, 78, 53, 166},
{96, 71, 52, 170},
{87, 86, 59, 185},
{104, 81, 62, 177},
{84, 65, 57, 155},
{106, 45, 62, 114},
{150, 37, 64, 110},
{182, 41, 68, 111},
{182, 43, 63, 108},
{201, 41, 59, 112},
{246, 32, 49, 112},
{7, 25, 48, 107},
{245, 16, 46, 104},
{113, 33, 35, 127},
{240, 19, 29, 120},
{213, 18, 23, 118},
{178, 18, 24, 124},
{98, 12, 19, 124},
{66, 5, 16, 118},
{30, 1, 11, 113},
{8, 0, 0, 128},
{11, 1, 1, 188},
{10, 0, 0, 128},
{9, 0, 0, 128},
{8, 0, 0, 128},
{7, 0, 0, 128},
{7, 0, 0, 128},
{8, 0, 0, 128},
{7, 0, 0, 128},
{8, 0, 0, 128},
{8, 0, 0, 128},
{8, 0, 0, 128}
};

// ==========================================
// COMMAND TABLE (PROGMEM)
// ==========================================
const char cmd_SIX[]   PROGMEM = "SIX";
const char cmd_NO[]    PROGMEM = "NO";
const char cmd_UP[]    PROGMEM = "UP";
const char cmd_SEVEN[] PROGMEM = "SEVEN";
const char cmd_START[] PROGMEM = "START";
const char cmd_YES[]   PROGMEM = "YES";
const char cmd_STOP[]  PROGMEM = "STOP";
const char cmd_ZERO[]  PROGMEM = "ZERO";

const char* const command_names[NUM_COMMANDS] PROGMEM = {
    cmd_SIX, cmd_NO, cmd_UP, cmd_SEVEN,
    cmd_START, cmd_YES, cmd_STOP, cmd_ZERO
};

// Master array maps templates sequentially (4 slots per command)
const uint8_t* const all_templates[TOTAL_TEMPLATES] PROGMEM = {
    (const uint8_t*)template_SIX_1,   (const uint8_t*)template_SIX_2,   (const uint8_t*)template_SIX_3,   (const uint8_t*)template_SIX_4,
    (const uint8_t*)template_NO_1,    (const uint8_t*)template_NO_2,    (const uint8_t*)template_NO_3,    (const uint8_t*)template_NO_4,
    (const uint8_t*)template_UP_1,    (const uint8_t*)template_UP_2,    (const uint8_t*)template_UP_3,    (const uint8_t*)template_UP_4,
    (const uint8_t*)template_SEVEN_1, (const uint8_t*)template_SEVEN_2, (const uint8_t*)template_SEVEN_3, (const uint8_t*)template_SEVEN_4,
    (const uint8_t*)template_START_1, (const uint8_t*)template_START_2, (const uint8_t*)template_START_3, (const uint8_t*)template_START_4,
    (const uint8_t*)template_YES_1,   (const uint8_t*)template_YES_2,   (const uint8_t*)template_YES_3,   (const uint8_t*)template_YES_4,
    (const uint8_t*)template_STOP_1,  (const uint8_t*)template_STOP_2,  (const uint8_t*)template_STOP_3,  (const uint8_t*)template_STOP_4,
    (const uint8_t*)template_ZERO_1,  (const uint8_t*)template_ZERO_2,  (const uint8_t*)template_ZERO_3,  (const uint8_t*)template_ZERO_4
};

// --- CORRECTED MASKING LENGTHS ---
// These accurately reflect the exact frame count dynamically allocated above.
const uint8_t template_lengths[TOTAL_TEMPLATES] = {
    40, 32, 25, 37, // SIX
    40, 22, 24, 23, // NO
    31, 15, 30, 14, // UP
    40, 31, 24, 32, // SEVEN
    40, 40, 24, 40, // START
    40, 28, 26, 34, // YES
    40, 26, 38, 24, // STOP
    40, 40, 36, 33  // ZERO
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
volatile int16_t ser_low_state        = 0; 
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
    ADMUX  = (1 << REFS0) | 0x05;
    ADCSRA = (1 << ADEN) | (1 << ADATE) | (1 << ADSC) | (1 << ADPS2) | (1 << ADPS1);
}

void timer1_init(void) {
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

    energy_accumulator += abs(current_sample);

    ser_low_state = ser_low_state - (ser_low_state >> 2) + (current_sample >> 2);

    if (abs(current_sample) > NOISE_FLOOR) {
        int16_t ser_high = current_sample - ser_low_state;
        low_energy_acc  += abs(ser_low_state);
        high_energy_acc += abs(ser_high);
    }

    if (abs(current_sample) > NOISE_FLOOR || abs(last_sample) > NOISE_FLOOR) {
        if ((current_sample >= 0 && last_sample < 0) || (current_sample < 0 && last_sample >= 0)) {
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

        uint32_t total_band = low_energy_acc + high_energy_acc;
        uint8_t  frame_ser  = (total_band > 0)
                            ? (uint8_t)((high_energy_acc * 255UL) / total_band)
                            : 128; 

        if (current_state == IDLE) {
            if (frame_energy > START_THRESHOLD) {
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
                
                // COMMENTED OUT FOR NOW:
                // Uncomment this when you are ready to manually trim your templates down from 40 frames!
                // if (current_frame > silence_counter) {
                //     current_frame -= silence_counter; 
                // }

                current_state   = PROCESSING;
                silence_counter = 0;
                noise_counter   = 0;
            }
        }

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
uint16_t calculate_dtw_2row(const uint8_t* template_ptr, uint8_t tmpl_frames, uint8_t actual_frames) {
    uint16_t row_prev[NUM_FRAMES + 1];
    uint16_t row_curr[NUM_FRAMES + 1];

    uint8_t longer  = (actual_frames > tmpl_frames) ? actual_frames : tmpl_frames;
    uint8_t shorter = (actual_frames < tmpl_frames) ? actual_frames : tmpl_frames;
    
    // +6 buffer to give short words some breathing room
    if (longer > (uint16_t)(shorter * 2) + 6) return 65535;

    uint8_t band = longer / DTW_BAND_DIVISOR;
    if (band < 2) band = 2;

    for (int i = 0; i <= tmpl_frames; i++) row_prev[i] = 65535;
    row_prev[0] = 0;

    for (int i = 1; i <= actual_frames; i++) {
        row_curr[0] = 65535;

        for (int j = 1; j <= tmpl_frames; j++) {
            int diag = (i * tmpl_frames) / actual_frames;
            if (abs(j - diag) > (int)band) {
                row_curr[j] = 65535;
                continue;
            }

            uint8_t tmpl_energy = pgm_read_byte(&template_ptr[(j-1)*NUM_FEATURES + 0]);
            uint8_t tmpl_zcr    = pgm_read_byte(&template_ptr[(j-1)*NUM_FEATURES + 1]);
            uint8_t tmpl_ssc    = pgm_read_byte(&template_ptr[(j-1)*NUM_FEATURES + 2]);
            uint8_t tmpl_ser    = pgm_read_byte(&template_ptr[(j-1)*NUM_FEATURES + 3]);

            int16_t live_delta_energy = (i > 1) ? ((int16_t)live_features[i-1][0] - (int16_t)live_features[i-2][0]) : 0;
            int16_t tmpl_delta_energy = (j > 1) ? ((int16_t)tmpl_energy - (int16_t)pgm_read_byte(&template_ptr[(j-2)*NUM_FEATURES + 0])) : 0;

            uint16_t energy_dist = abs(live_delta_energy - tmpl_delta_energy);
            uint16_t zcr_dist    = abs((int16_t)live_features[i-1][1] - tmpl_zcr);
            uint16_t ssc_dist    = abs((int16_t)live_features[i-1][2] - tmpl_ssc);
            uint16_t ser_dist    = abs((int16_t)live_features[i-1][3] - tmpl_ser);

            // WEIGHTS REBALANCED: Heavily favors frequency (phonetics) over volume slopes
            uint16_t dist = (energy_dist * 3) + (zcr_dist * 4) + (ssc_dist * 3) + (ser_dist * 2);

            uint16_t min_path = row_prev[j];
            if (row_curr[j-1] < min_path) min_path = row_curr[j-1];
            if (row_prev[j-1] < min_path) min_path = row_prev[j-1];

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
                           live_features[i][0], live_features[i][1],
                           live_features[i][2], live_features[i][3]);
                }
                printf("===\r\n");

            } else {
                uint16_t best_score = 65535;
                uint8_t best_template_idx = 255;

                for (uint8_t t = 0; t < TOTAL_TEMPLATES; t++) {
                    // This elegantly skips your fake arrays where length == 0!
                    if (template_lengths[t] == 0) continue;

                    const uint8_t* temp_ptr = (const uint8_t*)pgm_read_word(&all_templates[t]);

                    uint16_t score = calculate_dtw_2row(temp_ptr, template_lengths[t], current_frame);
                    
                    if (score < best_score) {
                        best_score = score;
                        best_template_idx = t;
                    }
                }

                LCD_Clear();

                if (best_score < REJECTION_THRESHOLD) {
                    // Divide by 4 to map indices 0,1,2,3 -> Cmd 0; 4,5,6,7 -> Cmd 1, etc.
                    uint8_t best_cmd = best_template_idx / TEMPLATES_PER_CMD;

                    char name_buf[8];
                    strcpy_P(name_buf, (const char*)pgm_read_word(&command_names[best_cmd]));

                    LCD_String("Command:");
                    LCD_String_xy(1, 0, name_buf);
                    
                    printf("Match: %s, Score: %u (Tpl: %u)\r\n", name_buf, best_score, best_template_idx);
                } else {
                    LCD_String("Unknown Noise");
                    printf("Rejected. Best score: %u\r\n", best_score);
                }

                _delay_ms(1500);

                LCD_Clear();
                LCD_String("Listening...");
            }

            current_frame = 0;
            current_state = IDLE;
        }
    }
}