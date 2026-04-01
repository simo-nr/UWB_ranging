#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

typedef struct {
    float *mag;
    size_t length;
    int rx_minus_tx;
    int peak_index;
    char fp_index_hex[32];
    double fp_index_samples;
} cir_data_t;

cir_data_t load_cir_data(const char *path);
void free_cir_data(cir_data_t *data);

