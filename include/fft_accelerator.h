#pragma once
#include <stddef.h>

typedef struct
{
	double real;
	double imaginary;
} fft_double_complex;

int fft_c2c(const fft_double_complex* input, fft_double_complex* output, size_t size);
int fft_r2c(const double* input, fft_double_complex* output, size_t size);