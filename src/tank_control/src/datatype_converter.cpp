    #include "tank_control/datatype_converter.h"
    
    template<typename T>
    void int_to_bytes(T value, uint8_t* array) {
        for (size_t i = 0; i < sizeof(T); ++i) {
            array[i] = static_cast<uint8_t>(value & 0xFF);
            value >>= 8;
        }
    }

    template<typename T>
    T bytes_to_integer(const uint8_t* array) {
        T value = 0;
        for (size_t i = 0; i < sizeof(T); ++i) {
            value |= static_cast<T>(array[i]) << (8 * i);
        }
        return value;
    }

    template<typename T>
    void integer_to_bytes(T value, uint8_t* array) {
        for (size_t i = 0; i < sizeof(T); ++i) {
            array[i] = static_cast<uint8_t>(value & 0xFF);
            value >>= 8;
        }
    }

    template<typename T>
    T bytes_to_unsigned_integer(const uint8_t* array) {
        T value = 0;
        for (size_t i = 0; i < sizeof(T); ++i) {
            value |= static_cast<T>(array[i]) << (8 * i);
        }
        return value;
    }

    void float_to_bytes(float value, uint8_t* array) {
        std::memcpy(array, &value, sizeof(float));
    }

    float bytes_to_float(const uint8_t* array) {
        float value;
        // 使用 memcpy 从字节数组复制到 float
        std::memcpy(&value, array, sizeof(float));
        return value;
    }


