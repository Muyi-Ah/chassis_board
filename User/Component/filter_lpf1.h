#ifndef __FILTER_LPF1_H__
#define __FILTER_LPF1_H__

typedef struct LPF1_vtable_struct LPF1_vtable_t;

typedef struct
{
    float alpha;
    float input;
    float previous_output;
    float output;
    LPF1_vtable_t *vtable;
} LPF1_t;

struct LPF1_vtable_struct
{
    float (*compute)(LPF1_t* lpf1, float input);
};

float LPF1_compute(LPF1_t* lpf1, float input);

#endif /* __FILTER_LPF1_H__ */