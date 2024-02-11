#ifndef STATS_H
# define STATS_H

void calculateDiff(float *original, float *tracked, float *diff, int len);
void calculateDiff_CMSIS(float *original, float *tracked, float *diff, int len);

float calculateAverage(float* diff, int len);
float calculateAverage_CMSIS(float* diff, int len);

float calculateStd(float* input, int len);
float calculateStd_CMSIS(float* input, int len);

void calculateConv(float* original, float* tracked, float* result, int len);
void calculateConv_CMSIS(float* original, float* tracked, float* result, int len);

float calculateCorr(float* A, float* B, int len);
float calculateCorr_CMSIS(float* A, float* B, int len);

#endif
