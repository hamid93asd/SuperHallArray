#include "NCC.h"

int8_t u, v = 0;

int32_t ncc_score(int8_t u, int8_t v, *A, *B){

    int8_t r0A = (v > 0) ? v : 0;
    int8_t r1A = (ROWS + v > ROWS) ? ROWS : ROWS + v;
    int8_t c0A = (u > 0) ? u : 0;
    int8_t c1A = (COLS + u > COLS) ? COLS : COLS + u;  

    int8_t r0B = (-v > 0) ? -v : 0;
    int8_t r1B = (ROWS - v > ROWS) ? ROWS : ROWS - v;
    int8_t c0B = (-u > 0) ? -u : 0;
    int8_t c1B = (COLS - u > COLS) ? COLS : COLS - u; 

    N = (r1A - r0A) * (c1A - c0A);  // Number of overlapping pixels

    int32_t sumA, sumB = 0;
    int64_t sumAA, sumBB, sumAB = 0;


    for (int r = 0; r < (r1A - r0A - 1); r++){
        for (int c = 0; c < (c1A - c0A - 1); c++){
            a = A[r0A + r][c0A + c];
            b = B[r0B + r][c0B + c];

            sumA += a;
            sumB += b;
            sumAA += a * a;
            sumBB += b * b;
            sumAB += a * b;
        }
    }

    // Need to benchmark, PICO doesn't have hardware float

    float N = (float)NCH;

    float num = (float)sumAB - ((float)sumA * (float)sumB) / N;
    float denA = (float)sumAA - ((float)sumA * (float)sumA) / N;
    float denB = (float)sumBB - ((float)sumB * (float)sumB) / N;

    float score = ( denA <= TINY || denB <= TINY ) ? -1 : num / sqrt(denA * denB);
    return (int32_t)score;
}
