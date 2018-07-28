/* COPYRIGHT (c) 2016-2018 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#include <core/ADS126x_driver/ADS126x.hpp>

#include <stdlib.h>

#include <functional>

#define CMD_NOP      0x00
#define CMD_RESET    0x06
#define CMD_START1   0x08
#define CMD_STOP1    0x0A
#define CMD_START2   0x0C
#define CMD_STOP2    0x0E
#define CMD_RDATA1   0x12
#define CMD_RDATA2   0x14
#define CMD_SYOCAL1  0x16
#define CMD_SYGCAL1  0x17
#define CMD_SFOCAL1  0x19
#define CMD_SYOCAL2  0x1B
#define CMD_SYGCAL2  0x1C
#define CMD_SFOCAL2  0x1E
#define CMD_RREG     0x20
#define CMD_WREG     0x40

#define MSG_DATA_READY  0x0D

namespace core {
namespace ADS126x_driver {
// --- ADS126x ----------------------------------------------------------------

ADS126x::ADS126x(
    core::hw::SPIDevice&  spi,
    core::hw::EXTChannel& ext,
    core::hw::Pad&        reset,
    core::hw::Pad&        start
) :
    _spi(spi), _ext(ext), _reset(reset), _start(start), _timestamp(0) {}

ADS126x::~ADS126x() {}

bool
ADS126x::init()
{
    return true;
}

bool
ADS126x::probe()
{
    bool success = true;

    _start.clear();
    _reset.clear();
    core::os::Thread::sleep(core::os::Time::ms(10));
    _reset.set();
    core::os::Thread::sleep(core::os::Time::ms(500));

    registers::Register_POWER power;

    read(power);

    success &= power.intref == 1;
    success &= power.reset == 1;

    return success;
} // ADS126x::probe

bool
ADS126x::configure()
{
    return true;
}

bool
ADS126x::start()
{
    _reset.set();

    core::os::Thread::sleep(core::os::Time::ms(500));

    // clear the reset bit, to detect HW resets.
    clearResetFlag();

    return true;
}

bool
ADS126x::stop()
{
    _start.clear();
    _reset.clear();

    core::os::Thread::sleep(core::os::Time::ms(10));

    return true;
}

bool
ADS126x::_read(
    uint8_t  address,
    uint8_t& data
)
{
    bool success = true;

    uint8_t txbuf[3];
    uint8_t rxbuf[3];

    txbuf[0] = CMD_RREG | address;
    txbuf[1] = 0x00;
    txbuf[2] = 0x00;

    _spi.acquireBus();
    _spi.select();
    _spi.exchange(3, txbuf, rxbuf);
    _spi.deselect();
    _spi.releaseBus();

    data = rxbuf[2];

    return success;
} // ADS126x::_read

bool
ADS126x::_write(
    uint8_t address,
    uint8_t data
)
{
    bool success = true;

    uint8_t txbuf[3];

    txbuf[0] = CMD_WREG | address;
    txbuf[1] = 0x00;
    txbuf[2] = data;

    _spi.acquireBus();
    _spi.select();
    _spi.send(3, txbuf);
    _spi.deselect();
    _spi.releaseBus();

    return success;
} // ADS126x::_write

void
ADS126x::cmd(
    uint8_t command
)
{
    _spi.acquireBus();
    _spi.select();
    _spi.send(1, &command);
    _spi.deselect();
    _spi.releaseBus();
}

ADS126x::DeviceType
ADS126x::getType()
{
    registers::Register_ID id;

    read(id);

    return id.dev_id;
}

bool
ADS126x::clearResetFlag()
{
	bool success = true;

    registers::Register_POWER power;

    success &= read(power);
    power.reset = 0;
    success &= write(power);

    return success;
}

// --- ADC1 -------------------------------------------------------------------

ADC1::ADC1(
    ADS126x& device
) : _device(device), _data(0), _status(0), _runner(nullptr) {}

ADC1::~ADC1() {}

bool
ADC1::start()
{
    _device._ext.setCallback([this](uint32_t channel) {
                chSysLockFromISR();

                if (_runner != nullptr) {
                    core::os::Thread::wake(*(this->_runner), MSG_DATA_READY);
                    _runner = nullptr;
                }

                chSysUnlockFromISR();
            });

    _device._ext.enable();

    if(_device._start.isNC()) {
        _device.cmd(CMD_START1);
    } else {
        _device._start.set();
    }

    return true;
} // ADC1::start

bool
ADC1::stop()
{
    if(_device._start.isNC()) {
        _device.cmd(CMD_STOP1);
    } else {
        _device._start.clear();
    }

    _device._ext.disable();

    if (_runner != nullptr) {
        //core::os::Thread::wake(*(this->_runner), 0); // Wake up consumer thread...
        _runner = nullptr;
    }

    return true;
}

bool
ADC1::setPGA(
    bool enabled,
    Gain gain
)
{
    registers::Register_MODE2 tmp;

    _device.read(tmp);

    tmp.bypass = enabled ? 0 : 1;
    tmp.gain   = gain;

    _device.write(tmp);

    return true;
}

bool
ADC1::setDataRate(
    DataRate data_rate
)
{
    registers::Register_MODE2 tmp;

    _device.read(tmp);

    tmp.dr = data_rate;

    _device.write(tmp);

    return true;
}

bool
ADC1::setFilter(
    Filter filter
)
{
    registers::Register_MODE1 tmp;

    _device.read(tmp);

    tmp.filter = filter;

    _device.write(tmp);

    return true;
}

bool
ADC1::setDelay(
    Delay delay
)
{
    registers::Register_MODE0 tmp;

    _device.read(tmp);

    tmp.delay = delay;

    _device.write(tmp);

    return true;
}

bool
ADC1::setInputMux(
    InputMuxPositive positive,
    InputMuxNegative negative
)
{
    registers::Register_INPMUX tmp;

    _device.read(tmp);

    tmp.muxp = positive;
    tmp.muxn = negative;

    _device.write(tmp);

    return true;
}

bool
ADC1::setReferenceMux(
    ReferenceMuxPositive positive,
    ReferenceMuxNegative negative
)
{
    registers::Register_REFMUX tmp;

    _device.read(tmp);

    tmp.rmuxp = positive;
    tmp.rmuxn = negative;

    _device.write(tmp);

    return true;
}

bool
ADC1::setReferencePolarityReversal(
    bool reverse
)
{
    registers::Register_MODE0 tmp;

    _device.read(tmp);

    tmp.refrev = reverse ? 1 : 0;

    _device.write(tmp);

    return true;
}

bool
ADC1::calibrateOffset()
{
    registers::Register_INPMUX inmux;
    registers::Register_INPMUX floating;

    floating.muxn = InputMuxNegative::FLOAT;
    floating.muxp = InputMuxPositive::FLOAT;

    _device.read(inmux);

    if(_device._start.isNC()) {
        _device.cmd(CMD_STOP1);
    } else {
        _device._start.clear();
    }

    _device.write(floating); // Float inputs

    if(_device._start.isNC()) {
        _device.cmd(CMD_START1);
    } else {
        _device._start.set();
    }

    core::os::Thread::sleep(core::os::Time::ms(100)); // I do not know if this is really needed

    _device.cmd(CMD_SFOCAL1);

    wait();

    _device.write(inmux); // Restore old mux configuration

    return true;
} // ADC1::calibrateOffset

bool
ADC1::update()
{
    uint8_t txbuf;
    uint8_t rxbuf[6];

    txbuf = CMD_RDATA1;

    _device._spi.acquireBus();
    _device._spi.select();
    _device._spi.send(1, &txbuf);
    _device._spi.receive(6, &rxbuf);
    _device._spi.deselect();
    _device._spi.releaseBus();

    _data = (rxbuf[1] << 24) | (rxbuf[2] << 16) | (rxbuf[3] << 8) | rxbuf[4];

    _status = rxbuf[0];

    return (_status.adc1 == 1) ? true : false; // return true if the data is new
} // ADC1::update

ADC1::Status
ADC1::getStatus() const
{
    return _status;
}

float
ADC1::get()
{
    return ((float)_data) * (1.0f / 0x7FFFFFFF);
}

int32_t
ADC1::getRaw()
{
    return _data;
}

bool
ADC1::wait(core::os::Time timeout)
{
    chSysLock();
    _runner = &core::os::Thread::self();
    core::os::Thread::Return msg = core::os::Thread::sleep_timeout(timeout);
    chSysUnlock();

    return msg == MSG_DATA_READY;
}

ADS126x&
ADC1::getDevice()
{
	return _device;
}

// ---- ADC2 ------------------------------------------------------------------

ADC2::ADC2(
    ADS126x& device
) : _device(device), _data(0), _status(0) {}

ADC2::~ADC2() {}

bool
ADC2::start()
{
    _device.cmd(CMD_START2);

    return true;
} // ADS1262::start

bool
ADC2::stop()
{
    _device.cmd(CMD_STOP2);

    return true;
}

bool
ADC2::setPGA(
    Gain gain
)
{
    registers::Register_ADC2CFG tmp;

    _device.read(tmp);

    tmp.gain2 = gain;

    _device.write(tmp);

    return true;
}

bool
ADC2::setDataRate(
    DataRate data_rate
)
{
    registers::Register_ADC2CFG tmp;

    _device.read(tmp);

    tmp.dr2 = data_rate;

    _device.write(tmp);

    return true;
} // ADS1262::setDataRate

bool
ADC2::setInputMux(
    InputMuxPositive positive,
    InputMuxNegative negative
)
{
    registers::Register_ADC2MUX tmp;

    _device.read(tmp);

    tmp.muxp = positive;
    tmp.muxn = negative;

    _device.write(tmp);

    return true;
}

bool
ADC2::setReferenceMux(
    ReferenceMux mux
)
{
    registers::Register_ADC2CFG tmp;

    _device.read(tmp);

    tmp.ref2 = mux;

    _device.write(tmp);

    return true;
}

bool
ADC2::calibrateOffset()
{
    registers::Register_ADC2MUX inmux;
    registers::Register_ADC2MUX floating;

    floating.muxn = InputMuxNegative::FLOAT;
    floating.muxp = InputMuxPositive::FLOAT;

    _device.read(inmux);

    _device.write(floating); // Float inputs

    core::os::Thread::sleep(core::os::Time::ms(100)); // I do not know if this is really needed

    _device.cmd(CMD_START2);

    core::os::Thread::sleep(core::os::Time::ms(200)); // I do not know if this is really needed

    _device.cmd(CMD_SFOCAL2);

    core::os::Thread::sleep(core::os::Time::ms(1800)); // Maximum time (for 10SPS). TODO: select the minimum time required (Table 33 of the datasheet)

    _device.cmd(CMD_STOP2);

    _device.write(inmux); // Restore old mux configuration

    return true;
} // ADC2::calibrateOffset

bool
ADC2::update()
{
    uint8_t txbuf;
    uint8_t rxbuf[6];

    txbuf = CMD_RDATA2;

    _device._spi.acquireBus();
    _device._spi.select();
    _device._spi.send(1, &txbuf);
    _device._spi.receive(6, &rxbuf);
    _device._spi.deselect();
    _device._spi.releaseBus();

    _data = (rxbuf[1] << 24) | (rxbuf[2] << 16) | (rxbuf[3] << 8) | rxbuf[4];

    _status = rxbuf[0];

    return (_status.adc2 == 1) ? true : false; // return true if the data is new
} // ADC2::update

ADC2::Status
ADC2::getStatus() const
{
    return _status;
}

float
ADC2::get()
{
    return ((float)_data) * (1.0f / 0x7FFFFFFF);
}

int32_t
ADC2::getRaw()
{
    return _data;
}

ADS126x&
ADC2::getDevice()
{
	return _device;
}

// ---- IDAC ------------------------------------------------------------------

bool
IDAC::setIDACMux(
    IDAC1Mux idac1mux,
    IDAC2Mux idac2mux
)
{
    registers::Register_IDACMUX tmp;

    _device.read(tmp);

    tmp.mux1 = idac1mux;
    tmp.mux2 = idac2mux;

    _device.write(tmp);

    return true;
}

bool
IDAC::setIDACMagnitude(
    IDAC1Magnitude idac1mag,
    IDAC2Magnitude idac2mag
)
{
    registers::Register_IDACMAG tmp;

    _device.read(tmp);

    tmp.mag1 = idac1mag;
    tmp.mag2 = idac2mag;

    _device.write(tmp);

    return true;
}

ADS126x&
IDAC::getDevice()
{
	return _device;
}

bool
TDAC::setTDACMagnitude(
    TDACNMagnitude tdacNmag,
    TDACPMagnitude tdacPmag
)
{
    registers::Register_TDACN tdacN;

    _device.read(tdacN);
    tdacN.magn = tdacNmag;
    _device.write(tdacN);

    registers::Register_TDACP tdacP;
    _device.read(tdacP);
    tdacP.magp = tdacPmag;
    _device.write(tdacP);

    return true;
}

ADS126x&
TDAC::getDevice()
{
	return _device;
}

// ---- ADS1262 ---------------------------------------------------------------
ADS1262::ADS1262(
    core::hw::SPIDevice&  spi,
    core::hw::EXTChannel& ext,
    core::hw::Pad&        reset,
    core::hw::Pad&        start
) : ADS126x::ADS126x(spi, ext, reset, start), _adc1(*this), _idac(*this), _tdac(*this) {}

ADS1262::~ADS1262() {}

ADS1263::ADS1263(
    core::hw::SPIDevice&  spi,
    core::hw::EXTChannel& ext,
    core::hw::Pad&        reset,
    core::hw::Pad&        start
) : ADS126x::ADS126x(spi, ext, reset, start), _adc1(*this), _adc2(*this), _idac(*this), _tdac(*this) {}

ADS1263::~ADS1263() {}

bool
ADS1263::probe()
{
    // Make sure we have ADC2!
    if (ADS126x::probe()) {
        return getType() == DeviceType::ADS1263;
    }

    return false;
}
}
}
