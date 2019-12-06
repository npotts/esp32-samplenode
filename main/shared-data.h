#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"

enum SampleType {IntSampleType = 0, FloatSampleType = 1 };

typedef struct sample_t {
    uint32_t whence;
    enum SampleType type;
    union sample { int i; float f;} sample;
    esp_err_t error;
} sample_t;

typedef struct wx_data_t {
    struct sample_t p, pt, rh, rht;
} wx_data_t;

sample_t update_value_f(float v, esp_err_t error);
sample_t update_value_i(int v, esp_err_t error);
int json_sample_t(char * buffer, int n, sample_t t);
int json_wx_data_t(char * buffer, int n, wx_data_t wx);