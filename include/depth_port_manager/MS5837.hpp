#pragma once

#include "IDepthDevice.hpp"

namespace depth_port_manager
{
    class MS5837 : public IDepthDevice
    {
        public:
        MS5837(std::shared_ptr<sonia_common_cpp::IConnection> connection);
        ~MS5837() = default;

        int ReadDataCheck(char buffer[], int bufferSize) override;
        DepthData ParseData(std::string data) override;
        void Tare() override;

        private:
        void calculate();

        float pressure(float conversion = 1.0f);

        /** Temperature returned in deg C.
         */
        float temperature();

        /** Depth returned in meters (valid for operation in incompressible
         *  liquids only. Uses density that is set for fresh or seawater.
         */
        float depth();

        /** Altitude returned in meters (valid for operation in air only).
         */
        float altitude();
        uint8_t crc4(uint16_t n_prom[]);

        const uint8_t MS5837_RESET = 0x1E;
        const uint8_t MS5837_ADC_READ = 0x00;
        const uint8_t MS5837_PROM_READ = 0xA0;
        const uint8_t MS5837_CONVERT_D1_8192 = 0x48;
        const uint8_t MS5837_CONVERT_D2_8192 = 0x58;

        const float Pa = 100.0f;
        const float bar = 0.001f;
        const float mbar = 1.0f;

        int cptTemp = 0;

        uint16_t C[8];
        uint32_t D1_pres, D2_temp;
        int32_t TEMP;
        int32_t P;
        uint8_t _model;

        float _fluidDensity = 1029;
    };
}  // namespace depth_port_manager
