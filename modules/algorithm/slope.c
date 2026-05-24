#include "slope.h"

void SlopeInit(Slope_s *slope, const float increase_step, const float decrease_step)
{
    slope->target = 0;
    slope->last = 0;

    slope->increase_step = increase_step;
    slope->decrease_step = decrease_step;
}

float SlopeUpdate(Slope_s *slope, const float target, const float current)
{
    slope->target = target;
    
    if (slope->last > 0.f)
    {
        if (slope->target > slope->last)
        {
            if (slope->target - slope->last > slope->increase_step)
                slope->last += slope->increase_step;
            else
                slope->last = slope->target;
        }
        else if (slope->target < slope->last)
        {
            if (slope->last - slope->target > slope->decrease_step)
                slope->last -= slope->decrease_step;
            else
                slope->last = slope->target;
        }
    }
    else if (slope->last < 0.f)
    {
        // target < 0
        if (slope->target < slope->last)
        {
            if (slope->target - slope->last > slope->increase_step)
                slope->last -= slope->increase_step;
            else
                slope->last = slope->target;
        }
        else if (slope->target > slope->last)
        {
            if (slope->last - slope->target > slope->decrease_step)
                slope->last += slope->decrease_step;
            else
                slope->last = slope->target;
        }
    }
    else
    {
        if (slope->target > slope->last)
        {
            if (slope->target - slope->last > slope->increase_step)
                slope->last += slope->increase_step;
            else
                slope->last = slope->target;
        }
        else if (slope->target < slope->last)
        {
            if (slope->last - slope->target > slope->increase_step)
                slope->last -= slope->increase_step;
            else
                slope->last = slope->target;
        }
    }

    return slope->last;
}
