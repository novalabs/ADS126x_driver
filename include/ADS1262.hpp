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
#include "ADS1262_registers.hpp"

namespace core {
namespace ADS1262_driver {
class ADS1262
{
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

    // IDAC
    using IDAC1Mux       = registers::Register_IDACMUX::MuxIDAC1;
    using IDAC2Mux       = registers::Register_IDACMUX::MuxIDAC2;
    using IDAC1Magnitude = registers::Register_IDACMAG::MagIDAC1;
    using IDAC2Magnitude = registers::Register_IDACMAG::MagIDAC2;

    // Status
    using Status         = registers::StatusByte;

public:
    ADS1262(
        core::hw::SPIDevice&  spi,
        core::hw::EXTChannel& ext,
        core::hw::Pad&        reset,
        core::hw::Pad&        start
    );
    virtual
    ~ADS1262();

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
    setIDACMux(
        IDAC1Mux idac1mux,
        IDAC2Mux idac2mux
    );

    bool
    setIDACMagnitude(
        IDAC1Magnitude idac1mag,
        IDAC2Magnitude idac2mag
    );

    bool
    calibrateOffset();

protected:
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
    core::os::Thread*     _runner;
    core::os::Time        _timestamp;
    Status _status;
    int32_t    _data;
};
}
}
