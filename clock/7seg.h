// jbclock - Copyright 2018, Jeffrey E. Bedard
#ifndef LIB_7SEG_H
#define LIB_7SEG_H
#include <stdint.h>
/*  7 segments:
    -a-
    b-c
    -d-
    e-f
    -g-
 */
// MAPPING:
enum {A, B, C, D, E, F, G, FIRST, SECOND, THIRD, FOURTH};
enum {SEV_0, SEV_1, SEV_2, SEV_3, SEV_4, SEV_5, SEV_6, SEV_7, SEV_8, SEV_9,
	SEV_DASH, SEV_D, SEV_A, SEV_T, SEV_E, SEV_H, SEV_R, SEV_M, SEV_Y,
	SEV_C, SEV_O, SEV_P, SEV_I, SEV_L, SEV_LL,
	SEV_NOTHING, SEV_S = SEV_5, SEV_N = SEV_M};
/* Flash DIGIT with symbol indexed by ID for duration DELAY.  */
void Seven_flash(const uint8_t digit, const uint8_t id,
	const uint16_t delay);
void Seven_flash_array(const uint8_t * a, const uint16_t delay);
/* Returns the pin of the specified digit index.  */
uint8_t Seven_get_digit(const uint8_t id);
/* Set pins mapped on array pinMap (numbers corresponding to segments a
 * through g.  */
void Seven_map(const uint8_t * pins);
/* Write the pattern indexed through id to the established map. */
void Seven_set(const uint8_t id);
#endif//!LIB_7SEG_H
