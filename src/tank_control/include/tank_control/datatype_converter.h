#ifndef DATATYPE_CONVERTER_H
#define DATATYPE_CONVERTER_H

#include <cstdint>
#include <cstring>

template<typename T>
void int_to_bytes(T value, uint8_t* array);

template<typename T>
T bytes_to_integer(const uint8_t* array);

template<typename T>
void integer_to_bytes(T value, uint8_t* array);

template<typename T>
T bytes_to_unsigned_integer(const uint8_t* array);

void float_to_bytes(float value, uint8_t* array);

float bytes_to_float(const uint8_t* array);

#endif // DATATYPE_CONVERTER_H
