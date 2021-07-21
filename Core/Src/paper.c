//
// Created by wirano on 2021/7/21.
//

#include "paper.h"
#include "easyflash.h"
#include "shell_port.h"
#include "UsartScreen.h"
#include "table.h"

double paper_fit;
uint16_t paper_cnt;

uint32_t freq_raw;
uint32_t cnt_raw;
uint64_t cnt_sum;
uint32_t int_cnt;
uint8_t sample_cnt;
double multi_paper_fit[3];
uint8_t rsted = 0;

uint8_t info = 0;

uint32_t freq_cali[100];
cali_line_st cali_1_65[65];
cali_line_st cali_55_90[36];
cali_data_st cali_data = {cali_1_65, cali_55_90, 0, 0};
double cali_delta_prev;
double cali_delta_next;
double cali_freq_delta; // cali_freq_delta = cali_k * freq_raw + cali_b
double freq_calied; // = freq_raw + cali_freq_delta


void paper_cali(void)
{
    uint8_t paper_freq_map_prev;

    cali_data.cnt_1_65 = 0;
    paper_freq_map_prev = 0;
    for (int i = 1; i <= 65; ++i) {
        if (freq_cali[i] != 0) {
            if (paper_freq_map_prev != 0) {
                if (paper_freq_map_prev < 55) {
                    cali_delta_prev = (double) freq_orig_1_65[paper_freq_map_prev - 1] -
                                      (double) freq_cali[paper_freq_map_prev];

                    cali_delta_next = (double) freq_orig_1_65[i - 1] - (double) freq_cali[i];

                    cali_data.line_cali_1_65[cali_data.cnt_1_65].cali_k =
                            (cali_delta_next - cali_delta_prev) /
                            (double) (freq_cali[i] - freq_cali[paper_freq_map_prev]);

                    cali_data.line_cali_1_65[cali_data.cnt_1_65].cali_b = cali_delta_next -
                                                                          cali_data.line_cali_1_65[cali_data.cnt_1_65].cali_k *
                                                                          (double) freq_cali[i];

                    cali_data.line_cali_1_65[cali_data.cnt_1_65].freq_divide = freq_cali[i];

                    paper_freq_map_prev = i;
                    cali_data.cnt_1_65++;

                    logDebug("1-65 cali_k:%f cali_b:%f",
                             cali_data.line_cali_1_65[cali_data.cnt_1_65 - 1].cali_k,
                             cali_data.line_cali_1_65[cali_data.cnt_1_65 - 1].cali_b);
                }
            } else {
                cali_data.line_cali_1_65[cali_data.cnt_1_65].cali_k = 0;
                cali_data.line_cali_1_65[cali_data.cnt_1_65].cali_b = 0;
                cali_data.line_cali_1_65[cali_data.cnt_1_65].freq_divide = freq_cali[i];
                paper_freq_map_prev = i;
                cali_data.cnt_1_65++;
            }
        }
    }

    cali_data.cnt_55_90 = 0;
    paper_freq_map_prev = 55;
    for (int i = 55; i <= 90; ++i) {
        if (freq_cali[i] != 0) {
            if (paper_freq_map_prev != 0) {
                if (paper_freq_map_prev < 55) {
                    cali_delta_prev = (double) freq_orig_55_90[paper_freq_map_prev - 55] -
                                      (double) freq_cali[paper_freq_map_prev];

                    cali_delta_next = (double) freq_orig_55_90[i - 55] - (double) freq_cali[i];

                    cali_data.line_cali_55_90[cali_data.cnt_55_90].cali_k =
                            (cali_delta_next - cali_delta_prev) /
                            (double) (freq_cali[i - 55] - freq_cali[paper_freq_map_prev]);

                    cali_data.line_cali_55_90[cali_data.cnt_55_90].cali_b = cali_delta_next -
                                                                            cali_data.line_cali_55_90[cali_data.cnt_55_90].cali_k *
                                                                            (double) freq_cali[i];

                    cali_data.line_cali_55_90[cali_data.cnt_55_90].freq_divide = freq_cali[i];

                    paper_freq_map_prev = i;
                    cali_data.cnt_55_90++;

                    logDebug("55-90 cali_k:%f cali_b:%f",
                             cali_data.line_cali_55_90[cali_data.cnt_55_90 - 55].cali_k,
                             cali_data.line_cali_55_90[cali_data.cnt_55_90 - 55].cali_b);
                }
            } else {
                cali_data.line_cali_55_90[cali_data.cnt_55_90].cali_k = 0;
                cali_data.line_cali_55_90[cali_data.cnt_55_90].cali_b = 0;
                cali_data.line_cali_55_90[cali_data.cnt_55_90].freq_divide = freq_cali[i - 55];
                paper_freq_map_prev = i;
                cali_data.cnt_55_90++;
            }
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static uint32_t p = 0;
    static uint32_t cnt;
    static uint32_t tmp[22];
    static uint32_t tmp_next[2];
    uint32_t ex_tmp;

    cali_line_st cali_used;
    static uint32_t freq_cali_sum = 0;

    cnt_raw = TIM2->CNT;
    TIM2->CNT = 0;
    cnt = cnt_raw;

    int_cnt++;
    if (int_cnt < 22) {
        tmp[int_cnt] = cnt;
    }

    if (int_cnt >= 22 && int_cnt <= 100) {

        tmp_next[p++] = cnt;

        if (p == 2) {
            p = 0;
            tmp[0] = tmp_next[0];
            tmp[21] = tmp_next[1];
        }
        for (int i = 0; i < 21; ++i) {
            for (int j = 0; j < 21 - i; ++j) {
                if (tmp[j] > tmp[j + 1]) {
                    ex_tmp = tmp[j];
                    tmp[j] = tmp[j + 1];
                    tmp[j] = ex_tmp;
                }
            }
        }
//        logDebug("tmp:%ld",tmp[0]);
        cnt_sum = 0;
        for (int i = 1; i < 21; ++i) {
            cnt_sum += tmp[i];
//            logDebug("tmp:%ld",tmp[i]);
        }
    }
#ifndef SAMPLING
    if (int_cnt == 100 && sample_cnt < 3) {

        int_cnt = 0;

        cali_used.cali_k = 0;
        cali_used.cali_b = 0;
        cali_used.freq_divide = 0;

//        logDebug("cali_used_k:%lf cali_used_b:%lf cali_freq_delta:%lf calied_freq:%ld", cali_used.cali_k,
//                 cali_used.cali_b, cali_freq_delta, freq_calied);

        if (cnt_sum < (freq_cali[60] == 0 ? (freq_orig_1_65[60 - 1] + freq_orig_55_90[60 - 55]) / 2 :
                       freq_cali[60])) {
            //0-60

            if (cnt_sum > (freq_cali[20] == 0 ? freq_orig_1_65[20 - 1] : freq_cali[20])) {
                for (int i = 0; i < cali_data.cnt_1_65; ++i) {
                    if ((double) cnt_sum < cali_data.line_cali_1_65[i].freq_divide) {
                        cali_used = cali_data.line_cali_1_65[i];
                        break;
                    }
                }
            }

            cali_freq_delta = cali_used.cali_k * (double) cnt_sum + cali_used.cali_b;
            freq_calied = (double) cnt_sum + cali_freq_delta;

//        paper_fit = a * exp(b * cnt_sum) + c * exp(d * cnt_sum);
//        paper_fit = p1 * pow(cnt_sum, 8) + p2 * pow(cnt_sum, 7) + p3 * pow(cnt_sum, 6) + p4 * pow(cnt_sum, 5) +
//                    p5 * pow(cnt_sum, 4) + p6 * pow(cnt_sum, 3) + p7 * pow(cnt_sum, 2) + p8 * pow(cnt_sum, 1) +
//                    p9 * pow(cnt_sum, 0);
//        paper_fit = p1 * pow(cnt_sum, 6) + p2 * pow(cnt_sum, 5) + p3 * pow(cnt_sum, 4) + p4 * pow(cnt_sum, 3) +
//                    p5 * pow(cnt_sum, 2) + p6 * pow(cnt_sum, 1) + p7 * pow(cnt_sum, 0);
//        paper_fit = p1 * pow(cnt_sum, 3) + p2 * pow(cnt_sum, 2) + p3 * pow(cnt_sum, 1) + p4 * pow(cnt_sum, 0);
//        paper_fit = p1 * pow(cnt_sum, 2) + p2 * pow(cnt_sum, 1) + p3 * pow(cnt_sum, 0);
            // 4阶高斯
//        paper_fit = a1 * exp(-pow(((cnt_sum - b1) / c1), 2)) + a2 * exp(-pow(((cnt_sum - b2) / c2), 2)) +
//                    a3 * exp(-pow(((cnt_sum - b3) / c3), 2)) + a4 * exp(-pow(((cnt_sum - b4) / c4), 2));
            // 5阶高斯
            paper_fit = a1 * exp(-pow(((freq_calied - b1) / c1), 2)) + a2 * exp(-pow(((freq_calied - b2) / c2), 2)) +
                        a3 * exp(-pow(((freq_calied - b3) / c3), 2)) + a4 * exp(-pow(((freq_calied - b4) / c4), 2)) +
                        a5 * exp(-pow(((freq_calied - b5) / c5), 2));
            // 8阶高斯
//        paper_fit = a1 * exp(-pow(((cnt_sum - b1) / c1), 2)) + a2 * exp(-pow(((cnt_sum - b2) / c2), 2)) +
//                    a3 * exp(-pow(((cnt_sum - b3) / c3), 2)) + a4 * exp(-pow(((cnt_sum - b4) / c4), 2)) +
//                    a5 * exp(-pow(((cnt_sum - b5) / c5), 2)) + a6 * exp(-pow(((cnt_sum - b6) / c6), 2)) +
//                    a7 * exp(-pow(((cnt_sum - b7) / c7), 2)) + a8 * exp(-pow(((cnt_sum - b8) / c8), 2));
//        paper_fit = a0 + a1 * cos(cnt_sum * w) + b1 * sin(cnt_sum * w) + a2 * cos(cnt_sum * w) + b2 * sin(cnt_sum * w) +
//                    a3 * cos(cnt_sum * w) + b3 * sin(cnt_sum * w) + a3 * cos(cnt_sum * w) + b3 * sin(cnt_sum * w);
        } else {
            //61-90

            for (int i = 0; i < cali_data.cnt_55_90; ++i) {
                if ((double) cnt_sum < cali_data.line_cali_55_90[i].freq_divide) {
                    cali_used = cali_data.line_cali_55_90[i];
                    break;
                }
            }

            cali_freq_delta = cali_used.cali_k * (double) cnt_sum + cali_used.cali_b;
            freq_calied = (double) cnt_sum + cali_freq_delta;

            // 5阶高斯
            paper_fit =
                    sa1 * exp(-pow(((freq_calied - sb1) / sc1), 2)) + sa2 * exp(-pow(((freq_calied - sb2) / sc2), 2)) +
                    sa3 * exp(-pow(((freq_calied - sb3) / sc3), 2)) + sa4 * exp(-pow(((freq_calied - sb4) / sc4), 2)) +
                    sa5 * exp(-pow(((freq_calied - sb5) / sc5), 2));
        }
        if (rsted) {
            multi_paper_fit[sample_cnt++] = paper_fit;
            if (sample_cnt <= 3) {
                freq_cali_sum += (uint32_t) cnt_sum;
            }
        }
    }

    if (sample_cnt == 3) {
        logDebug("%.2lf %.2lf %.2lf", multi_paper_fit[0], multi_paper_fit[1], multi_paper_fit[2]);
        if (round(multi_paper_fit[0]) == round(multi_paper_fit[1]) &&
            round(multi_paper_fit[0]) == round(multi_paper_fit[2]) &&
            round(multi_paper_fit[1]) == round(multi_paper_fit[2]) &&
            ScreenCmd.ScreenPage == 0) {
            rsted = 0;
            ScreenCmd.Start = 0;
            paper_cnt = (uint16_t) round(multi_paper_fit[0]);
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);
            info = 0;
            freq_raw = cnt_sum;


        } else if (ScreenCmd.ScreenPage == 1) {
            rsted = 0;
            ScreenCmd.Start = 0;
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);
            info = 0;
            freq_raw = freq_cali_sum / 3;
            freq_cali_sum = 0;

            SendScreenCorrectFre(freq_raw);

            if (ScreenCmd.CorrectNum == 1) {
                freq_cali[0] = freq_raw;
            } else {
                freq_cali[ScreenCmd.CorrectNum] = freq_raw;
            }
        } else {
            paper_cnt = 0;
//            rsted = 0;
            logDebug("not same");
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);
            freq_raw = 0;
        }
        sample_cnt++;
    }
#else
    if (int_cnt == 200) {
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);
    }
#endif
//    cnt_raw = TIM2->CNT;
//    TIM2->CNT = 0;
//    cnt = cnt_raw;
//
//    avg_tmp[p_start++] = cnt;
//    if (p_start == p_end) {
//        p_end++;
//        p_end %= 22;
//        p_start = p_end + 1;
//        avg_cnt++;
//    }
//    p_start %= 21;
//
//    memcpy(tmp,avg_tmp,sizeof(tmp));
//    for (int i = 0; i < 21; ++i) {
//        for (int j = 0; j < 21 - i; ++j) {
//            if(tmp[j] > tmp[j+1]){
//                ex_tmp = tmp[j];
//                tmp[j+1] = tmp[j];
//                tmp[j] = ex_tmp;
//            }
//        }
//    }
//
//    cnt_mid = tmp[10];
}

//void print_paper()
//{
//    info = 0;
//
//    for (int i = 0; i < 200; ++i) {
//        logInfo("%d %lld", i, paper[i]);
//    }
//}

//SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), printp, print_paper, print paper);

void show_cali_table()
{
    for (int i = 0; i < sizeof(freq_cali) / sizeof(uint32_t); ++i) {
        logInfo("%d %ld", i, freq_cali[i]);
    }
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), show_cali_table, show_cali_table,
                 show_cali_table);


void clear_cali_table()
{
    memset(freq_cali, 0, sizeof(freq_cali));
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), clear_cali_table, clear_cali_table,
                 clear_cali_table);

uint8_t write_cali_table()
{
    uint8_t len;

    len = ef_set_env_blob("freq_cali_table", freq_cali, sizeof(freq_cali));

    return len;
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), write_cali_table, write_cali_table,
                 write_cali_table);

uint8_t read_cali_table()
{
    uint8_t len;

    ef_get_env_blob("freq_cali_table", freq_cali, sizeof(freq_cali), &len);

    return len;
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), read_cali_table, read_cali_table,
                 read_cali_table);
