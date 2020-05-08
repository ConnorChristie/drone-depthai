#include "msp.h"

namespace Msp
{

inline void concat_arrays(char* result, size_t size1, char* array2, size_t size2)
{
    for (auto i = size1; i < size1 + size2; i++)
    {
        result[i] = array2[i - size1];
    }
}

uint8_t calc_checksum(MspCommand command, char* params, size_t param_size)
{
    uint8_t crc = param_size ^ command;

    for (auto i = 0; i < param_size; i++)
    {
        crc ^= params[i];
    }

    return crc;
}

void print_bytes(char* data, size_t length)
{
    std::cout << "Raw data: ";

    // Print out raw bytes
    for (auto i = 0; i < length; i++)
        printf("%02X ", (unsigned char)data[i]);

    printf("\n");
}

#define CLEAR_AMOUNT 128

void drain_read_buffer(ceSerial* serial)
{
    char buf[CLEAR_AMOUNT];
    auto drained_amount = serial->Read(buf, CLEAR_AMOUNT);

    std::cout << "Drained " << drained_amount << " bytes from read buffer." << std::endl;

    // If we filled this buffer, there might be more to drain
    if (drained_amount == CLEAR_AMOUNT)
    {
        drain_read_buffer(serial);
    }
}

char* send_raw_command(ceSerial* serial, MspCommand command, char* param_data, uint8_t param_size)
{
    uint8_t send_param_size = param_data != NULL ? param_size : 0;

    MspRequest request = {
        .preamble = { '$', 'M' },
        .direction = '<',

        .size = send_param_size,
        .command = command,
    };

    size_t request_size = sizeof(MspRequest);
    size_t crc_size = sizeof(uint8_t);

    size_t data_size = request_size + send_param_size + crc_size;
    char data[data_size];

    concat_arrays(data, 0, (char*)& request, request_size);

    if (param_data != NULL)
    {
        // request + params
        concat_arrays(
            data, request_size,
            param_data, send_param_size
        );
    }

    uint8_t crc = calc_checksum(command, param_data, send_param_size);

    // + crc
    concat_arrays(
        data, request_size + send_param_size,
        (char*)& crc, crc_size
    );

    // Write to serial port
    size_t write_length = serial->Write(data, data_size);

    // std::cout << "Sending: " << std::endl;
    // print_bytes(data, data_size);

    if (write_length != data_size)
    {
        std::cout << "Error writing " << (int)data_size << " only wrote " << (int)write_length << std::endl;
        print_bytes(data, data_size);
        return NULL;
    }

    serial->Flush();

    // Size of request + params + checksum
    // If we are sending data, only an ack will be sent back (no data)
    size_t rcv_param_size = param_data == NULL ? param_size : 0;
    size_t rcv_size = request_size + rcv_param_size + crc_size;

    char* rcv_data = new char[rcv_size];
    size_t read_length = serial->Read(rcv_data, rcv_size);

    auto t_start = high_resolution_clock::now();
    while (read_length != rcv_size && duration_cast<milliseconds>(high_resolution_clock::now() - t_start).count() < 10)
    {
        read_length += serial->Read(rcv_data + read_length, rcv_size - read_length);
    }

    // Verify response
    // Verify checksum
    // Return params
    // $M> <size> <command> | <params> <checksum>

    MspRequest* rcv_request = reinterpret_cast<MspRequest*>(rcv_data);

    //slog::info << "Rcving: " << slog::endl;
    //print_bytes(rcv_data, rcv_size);

    if (read_length != rcv_size)
    {
        std::cout << "Error reading " << (int)rcv_size << " only read " << (int)read_length << " (response told us to read " << (int)rcv_request->size << " bytes)" << std::endl;
        print_bytes(rcv_data, rcv_size);
        drain_read_buffer(serial);
        return NULL;
    }

    // Verify direction is correct and command matches the one we requested
    if (rcv_request->direction != '>' || rcv_request->command != command)
    {
        std::cout << "Was expecting a response to our request but received different" << std::endl;
        print_bytes(rcv_data, rcv_size);
        drain_read_buffer(serial);
        return NULL;
    }

    if (rcv_request->size != rcv_param_size)
    {
        std::cout << "Was expecting a response size of " << (int)rcv_param_size << " but received " << (int)rcv_request->size << " instead" << std::endl;
        print_bytes(rcv_data, rcv_size);
        drain_read_buffer(serial);
        return NULL;
    }

    char* rcv_params = new char[rcv_param_size];
    memcpy(rcv_params, rcv_data + request_size, rcv_param_size);

    uint8_t calc_crc = calc_checksum(rcv_request->command, (char*)rcv_params, rcv_request->size);
    uint8_t actual_crc = (uint8_t) rcv_data[request_size + rcv_param_size];

    // Verify the checksum is correct
    if (calc_crc != actual_crc)
    {
        std::cout << "Bad checksum: calculated " << (int)calc_crc << ", recv " << (int)actual_crc << std::endl;
        return NULL;
    }

    delete[] rcv_data;

    return rcv_params;
}

void to_json(json& j, const MspMotor& d)
{
    j = json
    {
        {"motor", d.motor}
    };
}

void to_json(json& j, const MspReceiver& d)
{
    j = json
    {
        {"roll", d.roll},
        {"pitch", d.pitch},
        {"throttle", d.throttle},
        {"yaw", d.yaw},
        {"flight_mode", d.flight_mode},
        {"aux_2", d.aux_2},
        {"arm_mode", d.arm_mode},
        {"aux_4", d.aux_4},
        {"aux_5", d.aux_5},
        {"aux_6", d.aux_6},
        {"aux_7", d.aux_7},
        {"aux_8", d.aux_8},
        {"aux_9", d.aux_9},
        {"aux_10", d.aux_10},
        {"aux_11", d.aux_11},
        {"aux_12", d.aux_12},
        {"aux_13", d.aux_13},
        {"aux_14", d.aux_14},
    };
}

}
