#include "filter.h"

static float output = 0;

float lowpass_filter(float input, float alpha)
{
    output = alpha * input + (1 - alpha) * output;
    return output;
}
