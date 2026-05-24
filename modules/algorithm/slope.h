#pragma once

typedef struct
{
    /* data */
    float target;
    float last;

    /* config */
    float increase_step;    
    float decrease_step;
} Slope_s;

void SlopeInit(Slope_s *slope, const float increase_step, const float decrease_step);

float SlopeUpdate(Slope_s *slope, const float target, const float current);

