//
// Created by nni on 22.10.18.
//

#ifndef BINAURALAUDIO_FTDI_IMU_H
#define BINAURALAUDIO_FTDI_IMU_H

#include "ftdi.h"
#include <thread>

class FTDI_IMU {
private:

    std::atomic<bool> _continue_reader_loop = {false};

    bool _succesfully_opened = false;

    struct ftdi_context *_ftdi;
    libusb_device *_dev;
    int _file_descriptor;

    std::thread * _reader_thread = nullptr;

    static constexpr size_t _read_buffer_size = 1024;
    unsigned char _read_buffer[_read_buffer_size];


    void reader_loop();

protected:

    int _baud_rate = 115200;

    bool initialize();

    bool connect(const int file_descriptor);

    virtual int read_new_data(const unsigned char * read_data, int bytes_read);

public:

    bool open(const int file_descriptor);
    bool start_read_thread();

};


#endif //BINAURALAUDIO_FTDI_IMU_H
