//
// Created by wirano on 2021/7/21.
//

#ifndef PAPER_H
#define PAPER_H

#include "stm32f4xx_hal.h"

typedef struct
{
    double cali_k;  // cali_k = ((freq_orig[i+1] - freq_cali[i+1]) - (freq_orig[i] - freq_cali[i])) / (freq_cali[i+1] - freq_cali[i])
    double cali_b;  // b = freq_orig[i] - cali_k[i] * freq_cali[i]
    uint32_t freq_divide;
} cali_line_st;

typedef struct
{
    cali_line_st *line_cali_1_65;
    cali_line_st *line_cali_55_90;
    uint8_t cnt_1_65;
    uint8_t cnt_55_90;
} cali_data_st;

typedef struct
{
    double linear_k;
    double linear_b;
    uint32_t freq_divide;
} linear_fit_st;

typedef struct
{
    linear_fit_st *linear;
    uint8_t cnt_linear;
} linear_data_st;

extern uint16_t paper_cnt;
extern uint32_t cnt_raw;
extern uint32_t cnt_sum;
extern uint32_t int_cnt;

extern uint8_t sample_cnt;

extern double multi_paper_fit[3];

extern uint8_t rsted;

extern uint32_t freq_cali[500];

extern uint8_t info;

extern double freq_calied;

void paper_cali(void);

#endif //PAPER_H
