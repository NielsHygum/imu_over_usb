//
// Created by nni on 22.10.18.
//

#include "ftdi_imu.h"
#include <android/log.h>
#include <chrono>

bool FTDI_IMU::initialize()
{
    return ((_ftdi = ftdi_new()) != 0);
}

bool FTDI_IMU::connect(const int file_descriptor)
{
    int ftdi_open_status = ftdi_usb_open_dev2(_ftdi, _dev, file_descriptor, _baud_rate);

    if(ftdi_open_status != 0)
    {
        __android_log_print(ANDROID_LOG_DEBUG, "FTDI_IMU","ftdi_usb_open2 returned error code: %d", ftdi_open_status);
    }

    return (ftdi_open_status == 0);
}

bool FTDI_IMU::open(const int file_descriptor)
{
    if(!_succesfully_opened)
    {
        _file_descriptor = file_descriptor;

        if(!initialize())
            return false;

        if(!connect(_file_descriptor))
            return false;

        _succesfully_opened = true;
    }

    return _succesfully_opened;
}

void FTDI_IMU::reader_loop()
{
    _continue_reader_loop = true;

    __android_log_print(ANDROID_LOG_DEBUG, "FTDI_IMU", "entering read loop");

    while(_continue_reader_loop)
    {
        int bytes_read = ftdi_read_data(_ftdi, _read_buffer, _read_buffer_size);

        if(bytes_read < 0)
        {
            __android_log_print(ANDROID_LOG_ERROR, "FTDI_IMU","failed reading ftdi data, code: %d", bytes_read);
        } else {
           //read new data
            read_new_data( _read_buffer, bytes_read);
        }

        //todo: maybe add sleep this is super pooling :(
        std::this_thread::sleep_for (std::chrono::milliseconds(1));
    }

    _reader_thread->join();
}

bool FTDI_IMU::start_read_thread()
{
    if(!_succesfully_opened)
        return false;

    if(_reader_thread == nullptr)
    {
        __android_log_print(ANDROID_LOG_DEBUG, "FTDI_IMU", "creating new thread");
        _reader_thread = new std::thread(&FTDI_IMU::reader_loop, this);
    }

    return true;
}

int FTDI_IMU::read_new_data(const unsigned char * read_data, int bytes_read)
{
    __android_log_print(ANDROID_LOG_DEBUG, "FTDI_IMU", "read %d bytes of data", bytes_read);
    __android_log_print(ANDROID_LOG_DEBUG, "FTDI_IMU", "data: %.*s", bytes_read, read_data);

    return 0;
}