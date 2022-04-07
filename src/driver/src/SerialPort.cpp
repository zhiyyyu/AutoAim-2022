//
// Created by zhiyu on 2021/8/20.
//

#include "../include/SerialPort.h"

namespace ly
{
    void SerialPort::read_data(Params_ToSerialPort& params)
    {
        while (1)
        {
            try
            {
                this->readData(params.data);
                // DLOG(INFO) << "                                           recv yaw: " << SerialParam::recv_data.yaw << "  recv pitch: " << SerialParam::recv_data.pitch;
            }
            catch (...)
            {
                DLOG(ERROR) << "catch an error in SerialPort::read_data";
                break;
            }
            usleep(5000);
        }
        return;
    }

    SerialPort::SerialPort()
    {
        new (this) SerialPort("/dev/ttyUSB0");
    }

    SerialPort::SerialPort(const string &port_name)
    {
        try
        {
            this->_serial_port = new serial_port(_io_service, port_name);
            this->_serial_port->set_option(serial_port::baud_rate(115200));
            this->_serial_port->set_option(serial_port::flow_control(serial_port::flow_control::none));
            this->_serial_port->set_option(serial_port::parity(serial_port::parity::none));
            this->_serial_port->set_option(serial_port::stop_bits(serial_port::stop_bits::one));
            this->_serial_port->set_option(serial_port::character_size(8));

            _data_tmp = (uint8_t *)malloc((size_t)_data_len);
            pingpong = (uint8_t *)malloc((size_t)_data_len * 2);
        }
        catch (...)
        {
            LOG(ERROR) << "create serial port object error! ";
        }
    }

    SerialPort::~SerialPort()
    {
        free(_data_tmp);
        free(pingpong);
        delete _serial_port;
    }

    void SerialPort::serialPortRead(uint8_t *msg, uint8_t max_len)
    {
        try
        {
            read(*_serial_port, boost::asio::buffer(msg, max_len), _err);
        }
        catch (...)
        {
            LOG(ERROR) << "readData from serial port error! " << _err.message();
        }
    }

    void SerialPort::serialPortWrite(uint8_t *msg, int len)
    {
        try
        {
            write(*_serial_port, buffer(msg, (size_t)len));
        }
        catch (...)
        {
            LOG(ERROR) << "write to serial port error! ";
        }
    }

    void SerialPort::addCRC(SerialPortData *msg)
    {
        if (!msg)
            return;
        msg->crc = getCRC(msg);
    }

    void SerialPort::addCRC(unsigned char *msg)
    {
        if (!msg)
            return;
        msg[_data_len_write - 1] = getCRC(msg);
    }

    uint8_t SerialPort::getCRC(SerialPortData *data)
    {
        auto _data = reinterpret_cast<unsigned char *>(data);
        int dwLength = _data_len - 1;
        unsigned char ucCRC8 = CRC8_INIT;
        unsigned char ucIndex;
        while (dwLength--)
        {
            ucIndex = ucCRC8 ^ (*_data++);
            ucCRC8 = CRC8_TAB[ucIndex];
        }
        return ucCRC8;
    }

    uint8_t SerialPort::getCRC(unsigned char *data)
    {
        auto _data = reinterpret_cast<unsigned char *>(data);
        int dwLength = _data_len_write - 1;
        unsigned char ucCRC8 = CRC8_INIT;
        unsigned char ucIndex;
        while (dwLength--)
        {
            ucIndex = ucCRC8 ^ (*_data++);
            ucCRC8 = CRC8_TAB[ucIndex];
        }

        return ucCRC8;
    }

    bool SerialPort::verifyCRC(SerialPortData *data)
    {
        if (!data)
            return false;
        return data->crc == getCRC(data);
    }

    void SerialPort::readData(SerialPortData *imu_data)
    {
        serialPortRead(_data_tmp, _data_len);
        memcpy(pingpong + _data_len, _data_tmp, _data_len);
        for (int start_bit = 0; start_bit < _data_len; start_bit++)
        {
            if (pingpong[start_bit] == '!')
            {
                memcpy(_data_tmp, pingpong + start_bit, _data_len);
                if (verifyCRC(reinterpret_cast<SerialPortData *>(_data_tmp))) //CRC校验
                {
                    imu_data->flag = _data_tmp[1];
                    imu_data->pitch = (_data_tmp[2] << 8) | _data_tmp[3];
                    imu_data->yaw = (((int)_data_tmp[4]) << 24) | (((int)_data_tmp[5]) << 16) | (((int)_data_tmp[6]) << 8) |
                                    (_data_tmp[7]);
                    imu_data->shootflag = _data_tmp[8];
                    // DLOG(INFO) << "readData: yaw " << imu_data->yaw << " pitch " << imu_data->pitch;
                    break;
                }
            }
        }
        memcpy(pingpong, pingpong + _data_len, _data_len);
    }

    void SerialPort::writeData(SerialPortWriteData *_data_write)
    {
        msg[0] = '!';
        unsigned char *tmp;
        msg[1] = 0x05;
        tmp = (unsigned char *)(&_data_write->pitch);
        msg[2] = tmp[1];
        msg[3] = tmp[0];
        tmp = (unsigned char *)(&_data_write->yaw);
        msg[4] = tmp[3];
        msg[5] = tmp[2];
        msg[6] = tmp[1];
        msg[7] = tmp[0];
        addCRC(msg);
        
        serialPortWrite(msg, _data_len_write);
    }
}
