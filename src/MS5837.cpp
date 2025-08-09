#include "depth_port_manager/MS5837.hpp"

#include <fcntl.h>

#include <cmath>
#include <iostream>
#include <stdexcept>

#include "sonia_common_cpp/I2CConn.hpp"

namespace depth_port_manager
{
    MS5837::MS5837(std::shared_ptr<sonia_common_cpp::IConnection> connection) : IDepthDevice(std::move(connection))
    {
        std::cout << "Starting ctor of MS3837" << std::endl;
        sonia_common_cpp::I2CTram tram;
        tram.cmd = MS5837_RESET;
        tram.size = 0;
        int success = _conn->Transmit(tram);
        std::cout << "RESET Request sent" << std::endl;
	usleep(10000);

        uint8_t res[64];
        // Read calibration values and CRC
        for (uint8_t i = 0; i < 7; i++)
        {
            tram.cmd = MS5837_PROM_READ + i * 2;
            tram.size = 2;
            tram.data.assign(res, res + (uint8_t)tram.size);
            success += _conn->Transmit(tram);
            C[i] = (tram.data.at(0) << 8) | tram.data.at(1);
        }

        // Verify that data is correct with CRC
        uint8_t crcRead = C[0] >> 12;
        uint8_t crcCalculated = crc4(C);
        if (crcCalculated != crcRead)
        {
            std::string err = "CRC Failed with " + success;
            throw std::runtime_error("CRC Failed");
        }
        cptTemp = 50;
    }

    int MS5837::ReadDataCheck(char buffer[], int bufferSize)
    {
        (void)bufferSize;
        sonia_common_cpp::I2CTram tram;

        // Request depth
        tram.cmd = MS5837_CONVERT_D1_8192;
        tram.size = 0;
        int success = _conn->Transmit(tram);
        usleep(12500);  // Max conversion time per datasheet
        tram.cmd = MS5837_ADC_READ;
        tram.size = 3;
        success += _conn->Read(tram);

        // fetch depth
        buffer[0] = tram.data.at(0);
        buffer[1] = tram.data.at(1);
        buffer[2] = tram.data.at(2);
        // Get temp every 50 ticks
        if (cptTemp >= 50)
        {
            // Request D2 conversion
            tram.data.clear();
            tram.cmd = MS5837_CONVERT_D2_8192;
            tram.size = 0;
            _conn->Transmit(tram);
            // i2c_smbus_write_byte(file,MS5837_CONVERT_D2_8192);
            usleep(10000);  // Max conversion time per datasheet

            tram.cmd = MS5837_ADC_READ;
            tram.size = 3;
            _conn->Read(tram);
            // i2c_smbus_read_i2c_block_data(file,MS5837_ADC_READ,3,res);
            buffer[3] = tram.data.at(0);
            buffer[4] = tram.data.at(1);
            buffer[5] = tram.data.at(2);
            // D2_temp = 0;
            // D2_temp = uint32_t(res[0]) << 16 | uint32_t(res[1]) << 8 | uint32_t(res[2]);
            cptTemp = 0;
            return 6;  // number of bytes pressue + temp
        }
        else
        {
            ++cptTemp;
        }
        return 3;  // number of bytes just pressure
    }

    DepthData MS5837::ParseData(std::string data)
    {
        uint8_t* parse_data = (uint8_t*)data.c_str();
        int parse_size = data.size();

        // get raw depth
        D1_pres = uint32_t(parse_data[0]) << 16 | uint32_t(parse_data[1]) << 8 | uint32_t(parse_data[2]);

        if (parse_size == 6)
        {
            // Get Raw Temp
            D2_temp = 0;
            D2_temp = uint32_t(parse_data[3]) << 16 | uint32_t(parse_data[4]) << 8 | uint32_t(parse_data[5]);
        }
        calculate();
        DepthData ret_data;
        ret_data.press = pressure();
        ret_data.temp = temperature();
        ret_data.depth = depth();
        return ret_data;
    }

    void MS5837::Tare() { throw std::runtime_error("Not implemented now"); }

    void MS5837::calculate()
    {
        // Given C1-C6 and D1, D2, calculated TEMP and P
        // Do conversion first and then second order temp compensation

        int32_t dT = 0;
        int64_t SENS = 0;
        int64_t OFF = 0;
        int32_t SENSi = 0;
        int32_t OFFi = 0;
        int32_t Ti = 0;
        int64_t OFF2 = 0;
        int64_t SENS2 = 0;

        // Terms called
        dT = D2_temp - uint32_t(C[5]) * 256l;

        SENS = int64_t(C[1]) * 32768l + (int64_t(C[3]) * dT) / 256l;
        OFF = int64_t(C[2]) * 65536l + (int64_t(C[4]) * dT) / 128l;
        P = (D1_pres * SENS / (2097152l) - OFF) / (8192l);

        // Temp conversion
        TEMP = 2000l + int64_t(dT) * C[6] / 8388608LL;

        // Second order compensation
        if ((TEMP / 100) < 20)
        {  // Low temp
            Ti = (3 * int64_t(dT) * int64_t(dT)) / (8589934592LL);
            OFFi = (3 * (TEMP - 2000) * (TEMP - 2000)) / 2;
            SENSi = (5 * (TEMP - 2000) * (TEMP - 2000)) / 8;
            if ((TEMP / 100) < -15)
            {  // Very low temp
                OFFi = OFFi + 7 * (TEMP + 1500l) * (TEMP + 1500l);
                SENSi = SENSi + 4 * (TEMP + 1500l) * (TEMP + 1500l);
            }
        }
        else if ((TEMP / 100) >= 20)
        {  // High temp
            Ti = 2 * (dT * dT) / (137438953472LL);
            OFFi = (1 * (TEMP - 2000) * (TEMP - 2000)) / 16;
            SENSi = 0;
        }

        OFF2 = OFF - OFFi;  // Calculate pressure and temp second order
        SENS2 = SENS - SENSi;

        TEMP = (TEMP - Ti);

        P = (((D1_pres * SENS2) / 2097152l - OFF2) / 8192l);
    }

    float MS5837::pressure(float conversion) { return P * conversion / 10.0f; }

    float MS5837::temperature() { return TEMP / 100.0f; }

    // The pressure sensor measures absolute pressure, so it will measure the atmospheric pressure + water pressure
    // We subtract the atmospheric pressure to calculate the depth with only the water pressure
    // The average atmospheric pressure of 101300 pascal is used for the calcuation, but atmospheric pressure varies
    // If the atmospheric pressure is not 101300 at the time of reading, the depth reported will be offset
    // In order to calculate the correct depth, the actual atmospheric pressure should be measured once in air, and
    // that value should subtracted for subsequent depth calculations.
    float MS5837::depth() { return (pressure(MS5837::Pa) - 101300) / (_fluidDensity * 9.80665); }

    float MS5837::altitude() { return (1 - pow((pressure() / 1013.25), .190284)) * 145366.45 * .3048; }


    // uint8_t MS5837::crc4(uint16_t n_prom[])
    // {
    //     uint16_t n_rem = 0;

    //     n_prom[0] = ((n_prom[0]) & 0x0FFF);
    //     n_prom[7] = 0;

    //     for (uint8_t i = 0; i < 16; i++)
    //     {
    //         if (i % 2 == 1)
    //         {
    //             n_rem ^= (uint16_t)((n_prom[i >> 1]) & 0x00FF);
    //         }
    //         else
    //         {
    //             n_rem ^= (uint16_t)(n_prom[i >> 1] >> 8);
    //         }
    //         for (uint8_t n_bit = 8; n_bit > 0; n_bit--)
    //         {
    //             if (n_rem & 0x8000)
    //             {
    //                 n_rem = (n_rem << 1) ^ 0x3000;
    //             }
    //             else
    //             {
    //                 n_rem = (n_rem << 1);
    //             }
    //         }
    //     }

    //     n_rem = ((n_rem >> 12) & 0x000F);

    //     return n_rem ^ 0x00;
    // }

    uint8_t crc4(uint16_t n_prom[])  // n_prom defined as 8x unsigned int (n_prom[8])
    {
        int cnt;                 // simple counter
        unsigned int n_rem = 0;  // crc remainder
        unsigned char n_bit;
        n_prom[0] = ((n_prom[0]) & 0x0FFF);  // CRC byte is replaced by 0
        n_prom[7] = 0;                       // Subsidiary value, set to 0
        for (cnt = 0; cnt < 16; cnt++)       // operation is performed on bytes
        {                                    // choose LSB or MSB
            if (cnt % 2 == 1)
                n_rem ^= (unsigned short)((n_prom[cnt >> 1]) & 0x00FF);
            else
                n_rem ^= (unsigned short)(n_prom[cnt >> 1] >> 8);
            for (n_bit = 8; n_bit > 0; n_bit--)
            {
                if (n_rem & (0x8000))
                    n_rem = (n_rem << 1) ^ 0x3000;
                else
                    n_rem = (n_rem << 1);
            }
        }
        n_rem = ((n_rem >> 12) & 0x000F);  // final 4-bit remainder is CRC code
        return (n_rem ^ 0x00);
    }

}  // namespace depth_port_manager
