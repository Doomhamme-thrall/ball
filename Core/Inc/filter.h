#ifndef FILTER_H
#define FILTER_H
#include "main.h"

float lowpass_filter(float input, float alpha, float *state);
float highpass_filter(float input, float alpha, float *state, float *prev_input);
float moving_average_filter(float input, float *buffer, int buffer_size, int *index, float *sum);
float weighted_average_filter(float *inputs, float *weights, int size);
float median_filter(float *buffer, int size);
float exponential_moving_average(float input, float alpha, float *state);
float kalman_filter(float input, float *estimate, float *error_cov, float process_noise, float measurement_noise);

#endif /* FILTER_H */