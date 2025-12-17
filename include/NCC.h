#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "pico/stdlib.h"

#define ROWS 6
#define COLS 6
#define NCH (ROWS * COLS)

extern const float TINY_EPS;

typedef struct {
    int8_t r0A, r1A, c0A, c1A;
    int8_t r0B, r1B, c0B, c1B;
    int8_t N;
} overlap_t;

bool ncc_compute_overlap(int8_t u, int8_t v, overlap_t *ov);
int32_t ncc_score(int8_t u, int8_t v, uint16_t *A, uint16_t *B, overlap_t *ov);

