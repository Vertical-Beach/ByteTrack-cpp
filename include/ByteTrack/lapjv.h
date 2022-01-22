#pragma once

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <iostream>

namespace byte_track
{
int lapjv_internal(const size_t n, double *cost[], int *x, int *y);
}