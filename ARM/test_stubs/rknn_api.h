#ifndef TEST_STUBS_RKNN_API_H
#define TEST_STUBS_RKNN_API_H

#include <stdint.h>

typedef uint64_t rknn_context;

typedef struct {
    uint32_t n_input;
    uint32_t n_output;
} rknn_input_output_num;

typedef struct {
    uint32_t index;
} rknn_tensor_attr;

typedef struct {
    void *virt_addr;
} rknn_tensor_mem;

#endif /* TEST_STUBS_RKNN_API_H */
