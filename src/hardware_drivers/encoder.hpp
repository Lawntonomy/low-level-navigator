#include "encoder.pio.h"
#include "pico/stdlib.h"

namespace encoder
{
void init(PIO pio, uint sm_index);

float get_left_rpm(PIO pio, uint sm);

float get_right_rpm(PIO pio, uint sm);

} // namespace encoder
