#include "msp.h"

namespace Msp
{

char* concat_arrays(char* array1, size_t size1, char* array2, size_t size2)
{
    char* result = (char*)malloc(size1 + size2);
    std::copy(array1, array1 + size1, result);
    std::copy(array2, array2 + size2, result + size1);

    return result;
}

uint8_t calc_checksum(MspCommand command, char* params, size_t param_size)
{
    uint8_t crc = param_size ^ command;

    for (unsigned int i = 0; i < param_size; i++)
    {
        crc ^= params[i];
    }

    return crc;
}

void print_bytes(char* data, size_t length)
{
    std::cout << "Raw data: ";

    // Print out raw bytes
    for (unsigned int i = 0; i < length; i++)
        printf("%02X ", (unsigned char)data[i]);

    printf("\n");
}

constexpr int CLEAR_AMOUNT = 128;

void drain_read_buffer(ceSerial* serial)
{
    char* buf = new char[CLEAR_AMOUNT];
    auto drained_amount = serial->Read(buf, CLEAR_AMOUNT);
    delete[] buf;

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

    char* result = (char*)& request;
    size_t result_size = sizeof(request);

    if (param_data != NULL)
    {
        // request + params
        result = concat_arrays(
            result, result_size,
            param_data, send_param_size
        );
        result_size += send_param_size;
    }

    uint8_t crc = calc_checksum(command, param_data, send_param_size);

    // + crc
    size_t crc_size = sizeof(crc);
    result = concat_arrays(
        result, result_size,
        (char*)& crc, crc_size
    );
    result_size += crc_size;

    // Write to serial port
    size_t write_length = serial->Write(result, result_size);

    //slog::info << "Sending: " << slog::endl;
    //print_bytes(result, result_size);

    if (write_length != result_size)
    {
        std::cout << "Error writing " << (int)result_size << " only wrote " << (int)write_length << std::endl;
        print_bytes(result, result_size);
        return NULL;
    }

    serial->Flush();

    // Size of request + params + checksum
    // If we are sending data, only an ack will be sent back (no data)
    size_t rcv_param_size = param_data == NULL ? param_size : 0;
    size_t rcv_size = sizeof(MspRequest) + rcv_param_size + sizeof(uint8_t);

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

    MspRequest rcv_request;
    memcpy(&rcv_request, rcv_data, sizeof(MspRequest));

    //slog::info << "Rcving: " << slog::endl;
    //print_bytes(rcv_data, rcv_size);

    if (read_length != rcv_size)
    {
        std::cout << "Error reading " << (int)rcv_size << " only read " << (int)read_length << " (response told us to read " << (int)rcv_request.size << " bytes)" << std::endl;
        print_bytes(rcv_data, rcv_size);
        drain_read_buffer(serial);
        return NULL;
    }

    // Verify direction is correct and command matches the one we requested
    if (rcv_request.direction != '>' || rcv_request.command != command)
    {
        std::cout << "Was expecting a response to our request but received different" << std::endl;
        print_bytes(rcv_data, rcv_size);
        drain_read_buffer(serial);
        return NULL;
    }

    if (rcv_request.size != rcv_param_size)
    {
        std::cout << "Was expecting a response size of " << (int)rcv_param_size << " but received " << (int)rcv_request.size << " instead" << std::endl;
        print_bytes(rcv_data, rcv_size);
        drain_read_buffer(serial);
        return NULL;
    }

    char* rcv_params = new char[rcv_param_size];
    memcpy(rcv_params, rcv_data + sizeof(MspRequest), rcv_param_size);

    uint8_t calc_crc = calc_checksum(rcv_request.command, (char*)rcv_params, rcv_request.size);
    uint8_t actual_crc;

    memcpy(&actual_crc, rcv_data + sizeof(MspRequest) + rcv_param_size, sizeof(actual_crc));

    // Verify the checksum is correct
    if (calc_crc != actual_crc)
    {
        std::cout << "Bad checksum: calculated " << (int)calc_crc << ", recv " << (int)actual_crc << std::endl;
        return NULL;
    }

    free(result);
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
