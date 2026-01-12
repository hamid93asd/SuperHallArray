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
    return ((ov->N >= MIN_OVERLAP) ? true : false);     // Require at least MIN_OVERLAP overlapping pixels
}


float ncc_score(int8_t u, int8_t v, int32_t *A, int32_t *B, overlap_t *ov){

    int32_t sumA = 0, sumB = 0;
    int64_t sumAA = 0, sumBB = 0, sumAB = 0;

    for (int r = 0; r < (ov->r1A - ov->r0A); r++){
        for (int c = 0; c < (ov->c1A - ov->c0A); c++){
            int32_t a = A[(ov->r0A + r) * COLS + (ov->c0A + c)];
            int32_t b = B[(ov->r0B + r) * COLS + (ov->c0B + c)];

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
    return score;
}
