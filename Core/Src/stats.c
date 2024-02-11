#include <math.h>
#include "arm_math.h"

void calculateDiff(float *original, float *tracked, float *diff, int len) {
	for (int i = 0; i < len; i++){
		diff[i] = original[i] - tracked[i];
	}
}

void calculateDiff_CMSIS(float *original, float* tracked, float* diff, int len) {
	arm_sub_f32(original,tracked,diff,len);
};

float calculateAverage(float* diff, int len) {
	float sum = 0.0;
	for (int i = 0; i < len; i++){
		sum += diff[i];
	}
	return sum / len;
}
float calculateAverage_CMSIS(float* diff, int len) {
	float32_t mean;
	arm_mean_f32(diff,len,&mean);
	return mean;
};

float calculateStd(float* input, int len) {
	float avg = calculateAverage(input, len);
	float var = 0.0;
	for (int i=0; i< len; i++) {
		var += pow(input[i] - avg, 2);
	}
	return sqrt(var / len);
}

float calculateStd_CMSIS(float* input, int len) {
	float std;
	arm_std_f32(input,len, &std);
	return std;
};

void calculateConv(float* original, float* tracked, float* result, int len) {
	for (int i = 0; i < len + len - 1; i ++){
		result[i] = 0.0;
		for (int j = 0; j < len; j++) {
			if(i - j >= 0 && i - j < len) {
				result[i] += original[j] * tracked[i - j];
			}
		}
	}
}

void calculateConv_CMSIS(float *A, float* B, float* result, int len) {
	arm_conv_f32(A,len,B,len,result);
};

float calculateCorr(float* A, float* B, int len) {
	float ave_A = calculateAverage(A, len);
	float ave_B = calculateAverage(B, len);
	float std_A = calculateStd(A, len);
	float std_B = calculateStd(B, len);
	float temp = 0;
	for(int i=0; i<len; i++){
		temp += (A[i]-ave_A)*(B[i]-ave_B);
	}
	float cov = temp/len;
	return cov/(std_A*std_B);
}

float calculateCorr_CMSIS(float* A, float* B, int len){
	float meanA = calculateAverage_CMSIS(A,len);
	float meanB = calculateAverage_CMSIS(B,len);
	float meanA_Arr[len];
	float meanB_Arr[len];
	float stdA = calculateStd_CMSIS(A,len);
	float stdB = calculateStd_CMSIS(B,len);
	float stdMul;
	float diffMean_A[len];
	float diffMean_B[len];
	float cov;

	calculateAverage_CMSIS(A,len);
	calculateAverage_CMSIS(B,len);
	for(int i=0; i<len; i++){
		meanA_Arr[i] = meanA;
		meanB_Arr[i] = meanB;
	}
	calculateDiff_CMSIS(A, meanA_Arr,diffMean_A,len);
	calculateDiff_CMSIS(B, meanB_Arr,diffMean_B,len);
	arm_dot_prod_f32(diffMean_A,diffMean_B,len,&cov);
	cov = cov/len;

	arm_mult_f32(&stdA,&stdB,&stdMul,1);
	return cov/stdMul;

}
