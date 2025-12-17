#include "NCC.h"

const float TINY_EPS = 1e-10f;

bool ncc_compute_overlap(int8_t u, int8_t v, overlap_t *ov){
    ov->r0A = (v > 0) ? v : 0;
    ov->r1A = (ROWS + v > ROWS) ? ROWS : ROWS + v;
    ov->c0A = (u > 0) ? u : 0;
    ov->c1A = (COLS + u > COLS) ? COLS : COLS + u;  

    ov->r0B = (-v > 0) ? -v : 0;
    ov->r1B = (ROWS - v > ROWS) ? ROWS : ROWS - v;
    ov->c0B = (-u > 0) ? -u : 0;
    ov->c1B = (COLS - u > COLS) ? COLS : COLS - u; 

    ov->N = (ov->r1A - ov->r0A) * (ov->c1A - ov->c0A);  // Number of overlapping pixels
    return ((ov->N >= 6) ? true : false);               // Require at least 6 overlapping pixels
}


int32_t ncc_score(int8_t u, int8_t v, uint16_t *A, uint16_t *B, overlap_t *ov){

    int32_t sumA, sumB = 0;
    int64_t sumAA, sumBB, sumAB = 0;

    for (int r = 0; r < (ov->r1A - ov->r0A - 1); r++){
        for (int c = 0; c < (ov->c1A - ov->c0A - 1); c++){
            uint16_t a = A[(ov->r0A + r) * COLS + (ov->c0A + c)];
            uint16_t b = B[(ov->r0B + r) * COLS + (ov->c0B + c)];

            sumA += a;
            sumB += b;
            sumAA += a * a;
            sumBB += b * b;
            sumAB += a * b;
        }
    }

    // Need to benchmark, PICO doesn't have hardware float

    float N = (float)(ov->N);

    float num = (float)sumAB - ((float)sumA * (float)sumB) / N;
    float denA = (float)sumAA - ((float)sumA * (float)sumA) / N;
    float denB = (float)sumBB - ((float)sumB * (float)sumB) / N;

    float score = ( denA <= TINY_EPS || denB <= TINY_EPS ) ? -1 : num / sqrt(denA * denB);
    return (int32_t)score;
}
