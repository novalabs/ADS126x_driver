/* COPYRIGHT (c) 2016-2017 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#pragma once

#include <core/hw/GPIO.hpp>
#include <core/hw/EXT.hpp>
#include <core/hw/SPI.hpp>
#include <core/os/Thread.hpp>
#include "ADS126x_registers.hpp"

namespace core {
namespace ADS126x_driver {

#undef ADC
#undef ADC1
#undef ADC2

class ADC1;
class ADC1;
class IDAC;

class ADS126x
{
        friend class ADC1;
        friend class ADC2;
        friend class IDAC;
public:
        using DeviceType = registers::Register_ID::DevID;
public:
    ADS126x(
        core::hw::SPIDevice&  spi,
        core::hw::EXTChannel& ext,
        core::hw::Pad&        reset,
        core::hw::Pad&        start
    );

    virtual
    ~ADS126x();

    bool
    init();

    bool
    probe();

    bool
    configure();

    bool
    start();

    bool
    stop();

    DeviceType getType();

public:
    template <typename T>
    inline bool
    read(
        T& reg
    )
    {
        static_assert(T::ACCESS & registers::AccessType::READ_ONLY, "The register is write only");
        return _read(T::ADDRESS, reg);
    }

    template <typename T>
    inline bool
    write(
        const T& data
    )
    {
        static_assert(T::ACCESS & registers::AccessType::WRITE_ONLY, "The register is read only");
        return _write(T::ADDRESS, data);
    }

private:
    bool
    _read(
        uint8_t address,
        uint8_t& data
    );

    bool
    _write(
        uint8_t address,
        uint8_t data
    );

    void
    cmd(
        uint8_t command
    );

private:
    core::hw::SPIDevice&  _spi;
    core::hw::EXTChannel& _ext;
    core::hw::Pad&        _reset;
    core::hw::Pad&        _start;
    core::os::Time        _timestamp;
};


class IDAC {
    public:
        using IDAC1Mux       = registers::Register_IDACMUX::MuxIDAC1;
        using IDAC2Mux       = registers::Register_IDACMUX::MuxIDAC2;
        using IDAC1Magnitude = registers::Register_IDACMAG::MagIDAC1;
        using IDAC2Magnitude = registers::Register_IDACMAG::MagIDAC2;
    public:
        IDAC(ADS126x& device) : _device(device) {};

        bool
        setIDACMux(
            IDAC1Mux idac1mux,
            IDAC2Mux idac2mux
        );

        bool
        setIDACMagnitude(
            IDAC1Magnitude idac1mag,
            IDAC2Magnitude idac2mag
        );

    private:
        ADS126x& _device;
};

class ADC1 {
    public:
        using Gain     = registers::Register_MODE2::Gain;
        using DataRate = registers::Register_MODE2::DataRate;
        using Filter   = registers::Register_MODE1::Filter;
        using Delay    = registers::Register_MODE0::Delay;

        // Input
        using InputMuxPositive = registers::Register_INPMUX::MuxPositive;
        using InputMuxNegative = registers::Register_INPMUX::MuxNegative;

        // Reference
        using ReferenceMuxPositive = registers::Register_REFMUX::MuxPositive;
        using ReferenceMuxNegative = registers::Register_REFMUX::MuxNegative;

        // Status
        using Status         = registers::StatusByte;
public:
        ADC1(ADS126x& device);
        ~ADC1();

        bool setMode(bool); //pulse or continuous

        bool start();
        bool stop();

        bool setCallback();

        bool
        wait();

        bool
        update();

        Status
        getStatus() const;

        float
        get();

        int32_t
        getRaw();

        bool
        setPGA(
            bool enabled,
            Gain gain
        );

        bool
        setDataRate(
            DataRate data_rate
        );

        bool
        setFilter(
            Filter filter
        );

        bool
        setDelay(
            Delay delay
        );

        bool
        setInputMux(
            InputMuxPositive positive,
            InputMuxNegative negative
        );

        bool
        setReferenceMux(
            ReferenceMuxPositive positive,
            ReferenceMuxNegative negative
        );

        bool
        setReferencePolarityReversal(
            bool reverse
        );

        bool
        calibrateOffset();

    private:
        ADS126x& _device;
        int32_t _data;
        Status _status;
        core::os::Thread*     _runner;
};

class ADC2 {
    public:
        using Gain     = registers::Register_ADC2CFG::Gain;
        using DataRate = registers::Register_ADC2CFG::DataRate;

        // Input
        using InputMuxPositive = registers::Register_ADC2MUX::MuxPositive;
        using InputMuxNegative = registers::Register_ADC2MUX::MuxNegative;

        // Reference
        using ReferenceMux = registers::Register_ADC2CFG::ReferenceInput;

        // Status
        using Status         = registers::StatusByte;
public:
        ADC2(ADS126x& device);
        ~ADC2();

        bool start();
        bool stop();

        bool setCallback();

        bool
        wait();

        bool
        update();

        Status
        getStatus() const;

        float
        get();

        int32_t
        getRaw();

        bool
        setPGA(
            Gain gain
        );

        bool
        setDataRate(
            DataRate data_rate
        );

        bool
        setInputMux(
            InputMuxPositive positive,
            InputMuxNegative negative
        );

        bool
        setReferenceMux(
            ReferenceMux mux
        );

        bool
        calibrateOffset();

    private:
        ADS126x& _device;
        int32_t _data;
        Status _status;
};


class ADS1262 : public ADS126x {
    public:
        ADS1262(core::hw::SPIDevice&  spi,
                core::hw::EXTChannel& ext,
                core::hw::Pad&        reset,
                core::hw::Pad&        start);
        ~ADS1262();

        inline ADC1& adc1() {return _adc1;}
        inline IDAC& idac() {return _idac;}
    private:
        ADC1 _adc1;
        IDAC _idac;
};

class ADS1263 : public ADS126x {
    public:
        ADS1263(core::hw::SPIDevice&  spi,
                core::hw::EXTChannel& ext,
                core::hw::Pad&        reset,
                core::hw::Pad&        start);

        ~ADS1263();

        bool
        probe();

        inline ADC1& adc1() {return _adc1;}
        inline ADC2& adc2() {return _adc2;}
        inline IDAC& idac() {return _idac;}
    private:
        ADC1 _adc1;
        ADC2 _adc2;
        IDAC _idac;
};

}
}
