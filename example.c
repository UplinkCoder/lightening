
#define _BSD_SOURCE 1
#include <stdio.h>
#include <assert.h>
#include <stdlib.h>
#include "pool.c"
#include "int_iter.c"
#include "lightening.h"
#include "lightening/lightening.c"
#include "linenoise/linenoise.h"
#include "linenoise/linenoise.c"
#include <sys/mman.h>

Pool gPool;
void* allocFromMainPool(size_t sz)
{
    PoolAllocationRecordIndex result = Pool_Allocate(&gPool, sz);
    if (result.value)
    {
        return gPool.recordPage[result.value].startMemory;
    }
    return (void*)0;
}

void freeFromMainPool(void* ptr)
{
    // TODO implement free
    return ;
}

typedef int(*func_t)();

/*
 *jit_begin(ctx, arena_base, arena_size);

    jit_movi(ctx, JIT_R0, 0);

    for(int i = 1; i < argc; i++)
    {
        int n = atoi(argv[i]);
        jit_addi(ctx, JIT_R0, JIT_R0, n);
    }

    jit_retr(ctx, JIT_R0);
    size_t code_size;
    func_t fn = ((func_t) jit_end(ctx, &code_size));
    printf("%d bytes generated\n", (int)code_size);

    printf("The result is: %d\n", fn());
 */

int main(int argc, const char* argv[])
{
    // first thing we have to do is to init the library;
    init_jit();
    // then we init our memory allocator
    Pool_Init(&gPool);

    jit_state_t* ctx =
        jit_new_state(allocFromMainPool, freeFromMainPool);

    const size_t arena_size = 4096;
    uint8_t *arena_base = mmap (NULL, arena_size,
           PROT_EXEC | PROT_READ | PROT_WRITE,
           MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);

	jit_begin(ctx, arena_base, arena_size);

    char* line;

    while((line = linenoise("lJIT> ")))
    {
        uint32_t sz = strlen(line);
		if (strcmp(line, ":q") == 0)
		{
			exit(0);
		}
		if (sz > 4 && memcmp(line, "add ", 4) == 0)
		{
            IntIter it;
            IntIter_FromBuffer(&it, line + 4, sz - 3/*inlude 0 terminator*/);
            int a, b;
            IntIter_NextInt(&it, &a);
            IntIter_NextInt(&it, &b);
            printf("we would now add %d and %d\n", a, b);
		}
		else
		{
			printf("Unrecogonized command '%s'\n", line);
		}
	}

    return 0;
}
