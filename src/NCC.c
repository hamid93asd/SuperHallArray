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
    return (ov->N >= MIN_OVERLAP) ? true : false;     // Require at least MIN_OVERLAP overlapping pixels
}


uint64_t ncc_score(int8_t u, int8_t v, int32_t *A, int32_t *B, overlap_t *ov){

    int64_t sumA = 0, sumB = 0;
    int64_t sumAA = 0, sumBB = 0, sumAB = 0;

    for (int r = 0; r < (ov->r1A - ov->r0A); r++){
        for (int c = 0; c < (ov->c1A - ov->c0A); c++){
            int64_t a = A[(ov->r0A + r) * COLS + (ov->c0A + c)];
            int64_t b = B[(ov->r0B + r) * COLS + (ov->c0B + c)];

            sumA += a;
            sumB += b;
            sumAA += a * a;
            sumBB += b * b;
            sumAB += a * b;
        }
    }

    // Need to benchmark, PICO doesn't have hardware float

    int64_t N = (int64_t)(ov->N);
    int64_t crossAA = (sumA * sumA); // >> 20;
    int64_t crossBB = (sumB * sumB); // >> 20;
    int64_t crossAB = (sumA * sumB); // >> 20;
    
    sumAA = sumAA; // >> 20;
    sumBB = sumBB; // >> 20;
    sumAB = sumAB; // >> 20;

    int64_t num = (N * sumAB - crossAB);
    int64_t denA = (N * sumAA - crossAA);
    int64_t denB = (N * sumBB - crossBB);

    if (denA <= 0 || denB <= 0){
        return 0;
    }

    num = (num > 0) ? num : -num;

    num = num >> 30;    // Eliminates all fractional bits :( Q0
    denA = denA >> 40;
    denB = denB >> 40;

    uint64_t num2 = (num * num);
    uint64_t den = (denA * denB); // was >> 20, as written returns 0s
    
    if(den != 0){
        uint64_t score = num2 / den;
        return score;
    } else return 0;
}

double f_score(int8_t u, int8_t v, int32_t *A, int32_t *B, overlap_t *ov){
    double sumA = 0, sumB = 0, sumAA = 0, sumBB = 0, sumAB = 0;

        for (int r = 0; r < (ov->r1A - ov->r0A); r++){
            for (int c = 0; c < (ov->c1A - ov->c0A); c++){
                double a = (double)A[(ov->r0A + r) * COLS + (ov->c0A + c)];
                double b = (double)B[(ov->r0B + r) * COLS + (ov->c0B + c)];

                sumA += a;
                sumB += b;
                sumAA += a * a;
                sumBB += b * b;
                sumAB += a * b;
            }
        }

        double N = (double)(ov->N);
        double num = (N * sumAB - (sumA * sumB));
        double denA = (N * sumAA - (sumA * sumA));
        double denB = (N * sumBB - (sumB * sumB));

        double score = num / sqrt(denA * denB);
        return score;
}

int64_t isqrt(int64_t n) {
    if (n < 0) return 0;
    if (n < 2) return n;

    int64_t x = n;
    int64_t y = (x + 1) / 2;

    while (y < x){
        x = y;
        y = (x + n / x) / 2;
    }
    return x;
}
