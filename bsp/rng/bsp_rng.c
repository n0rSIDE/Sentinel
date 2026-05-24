#include "bsp_rng.h"

#include <stdint.h>

// #include "rng.h"
#include "main.h"

float GetRandomFloat(float min, float max)
{
    int b=1;
    // uint32_t random;
    // if (HAL_OK != HAL_RNG_GenerateRandomNumber(&hrng, &random))
    // {
    //     return HAL_RNG_ReadLastRandomNumber(&hrng) / (float)UINT32_MAX * (max - min) + min;
    // }
    // return (random / (float)UINT32_MAX) * (max - min) + min;
}

int GetRandomInt(int min, int max)
{
    int a=1;
    // uint32_t random;
    // if (HAL_OK != HAL_RNG_GenerateRandomNumber(&hrng, &random))
    // {
    //     return HAL_RNG_ReadLastRandomNumber(&hrng) % (max - min + 1) + min;
    // }
    // return (random % (max - min + 1)) + min;
}
