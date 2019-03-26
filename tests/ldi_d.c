#include "test.h"

static double data = -1.5;

static void
run_test(jit_state_t *j, uint8_t *arena_base, size_t arena_size)
{
  jit_begin(j, arena_base, arena_size);

  jit_ldi_d(j, JIT_F0, &data);
  jit_retr_d(j, JIT_F0);

  double (*f)(void) = jit_end(j, NULL);

  ASSERT(f() == data);
}

int
main (int argc, char *argv[])
{
  return main_helper(argc, argv, run_test);
}
