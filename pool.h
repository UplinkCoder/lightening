#pragma once

#include <stdint.h>

#define ALLOCATE(POOL, SIZE) Pool_AllocateWithStats(POOL, SIZE, __FILE__, __LINE__);

typedef struct PoolAllocationRecordIndex
{
    uint32_t value;
} PoolAllocationRecordIndex;

typedef struct PoolAllocationRecord
{
   uint8_t* startMemory;     // 8

   uint32_t sizeRequested;  // 12
   uint32_t sizeAllocated;  // 16
   uint8_t used;            // 17
   uint8_t pageRangeStart;  // 18
   uint8_t _pad[7];         // 32
} PoolAllocationRecord;

typedef struct Pool
{
    uint32_t wasted_bytes;
    uint32_t total_allocated;
    /// information private to the memory manager.

    PoolAllocationRecord* recordPage;
    uint8_t* allocationAreaStart;

    uint32_t n_allocation_records;
    uint32_t recordsLeft;
    uint32_t allocatedRecordPages;
    uint32_t n_allocated_extra_pages;
    uint32_t sizeLeftOnCurrentPage;

#ifdef __cplusplus
    Pool();
    PoolAllocationRecordIndex Allocate(size_t requested_size);
    PoolAllocationRecord* Reallocate(PoolAllocationRecord* par
                                   , size_t requested_new_size);
#endif
} Pool;

void Pool_Init(Pool* thisP);
PoolAllocationRecordIndex Pool_Allocate(Pool* thisP
                                       , uint32_t requested_size);
/*
PoolAllocationRecordIndex Pool_AllocateStats(Pool* thisP
                                       , uint32_t requested_size
                                       , char* filename
                                       , uint32_t line);
*/
PoolAllocationRecord* Pool_Reallocate(Pool* thisP
                                         , PoolAllocationRecord* par
                                         , uint32_t requested_new_size);
