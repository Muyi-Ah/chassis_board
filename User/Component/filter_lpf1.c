#include "filter_lpf1.h"

float LPF1_compute(LPF1_t* lpf1, float input)
{
    lpf1->previous_output = lpf1->output;
    lpf1->output = lpf1->alpha * (input - lpf1->previous_output) + lpf1->previous_output;
    return lpf1->output;
}
