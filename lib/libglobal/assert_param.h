#ifndef LIBPERIPH_ASSERT_PARAM_H
# define LIBPERIPH_ASSERT_PARAM_H

#include "stm32f10x.h"

# define assert_param(expr)                      \
  {                                              \
    if ((expr) == 0)                             \
    {                                            \
      __disable_irq();                           \
      for(;;);                                   \
    }                                            \
  }

#endif
