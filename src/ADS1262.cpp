/* COPYRIGHT (c) 2016-2017 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#include <core/ADS1262_driver/ADS1262.hpp>

#include <stdlib.h>

#include <functional>

#define CMD_NOP     0x00
#define CMD_RESET   0x06
#define CMD_START1    0x08
#define CMD_STOP1   0x0A
#define CMD_START2    0x0C
#define CMD_STOP2   0x0E
#define CMD_RDATA1    0x12
#define CMD_RDATA2    0x14
#define CMD_SYOCAL1   0x16
#define CMD_SYGCAL1   0x17
#define CMD_SFOCAL1   0x19
#define CMD_SYOCAL2   0x1B
#define CMD_SYGCAL2   0x1C
#define CMD_SFOCAL2   0x1E
#define CMD_RREG    0x20
#define CMD_WREG    0x40

#define MSG_DATA_READY  0x0D

namespace core {
namespace ADS1262_driver {
ADS1262::ADS1262(
    core::hw::SPIDevice&  spi,
    core::hw::EXTChannel& ext,
    core::hw::Pad&        reset,
    core::hw::Pad&        start
) :
    _spi(spi), _ext(ext), _reset(reset), _start(start), _runner(nullptr), _timestamp(0), _status(0), _data(0.0) {}

ADS1262::~ADS1262() {}

bool
ADS1262::init()
{
    return true;
}

bool
ADS1262::probe()
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
} // ADS1262::probe

bool
ADS1262::configure()
{
    return true;
}

bool
ADS1262::start()
{
    _ext.setCallback([this](uint32_t channel) {
                chSysLockFromISR();

                if (_runner != nullptr) {
                    _timestamp.now();
                    core::os::Thread::wake(*(this->_runner), MSG_DATA_READY);
                    _runner = nullptr;
                }

                chSysUnlockFromISR();
            });

    _ext.enable();
    _start.set();

    return true;
} // ADS1262::start

bool
ADS1262::stop()
{
    _start.clear();
    _ext.disable();

    if (_runner != nullptr) {
        core::os::Thread::wake(*(this->_runner), 0); // Wake up consumer thread...
        _runner = nullptr;
    }

    return true;
}

bool
ADS1262::setPGA(
    bool enabled,
    Gain gain
)
{
    registers::Register_MODE2 tmp;

    read(tmp);

    tmp.bypass = enabled ? 0 : 1;
    tmp.gain   = gain;

    write(tmp);

    return true;
} // ADS1262::setGain

bool
ADS1262::setDataRate(
    DataRate data_rate
)
{
    registers::Register_MODE2 tmp;

    read(tmp);

    tmp.dr = data_rate;

    write(tmp);

    return true;
} // ADS1262::setDataRate

bool
ADS1262::setFilter(
    Filter filter
)
{
    registers::Register_MODE1 tmp;

    read(tmp);

    tmp.filter = filter;

    write(tmp);

    return true;
}

bool
ADS1262::setDelay(
    Delay delay
)
{
    registers::Register_MODE0 tmp;

    read(tmp);

    tmp.delay = delay;

    write(tmp);

    return true;
}

bool
ADS1262::setInputMux(
    InputMuxPositive positive,
    InputMuxNegative negative
)
{
    registers::Register_INPMUX tmp;

    read(tmp);

    tmp.muxp = positive;
    tmp.muxn = negative;

    write(tmp);

    return true;
}

bool
ADS1262::setReferenceMux(
    ReferenceMuxPositive positive,
    ReferenceMuxNegative negative
)
{
    registers::Register_REFMUX tmp;

    read(tmp);

    tmp.rmuxp = positive;
    tmp.rmuxn = negative;

    write(tmp);

    return true;
}

bool
ADS1262::setReferencePolarityReversal(
    bool reverse
)
{
    registers::Register_MODE0 tmp;

    read(tmp);

    tmp.refrev = reverse ? 1 : 0;

    write(tmp);

    return true;
}

bool
ADS1262::setIDACMux(
    IDAC1Mux idac1mux,
    IDAC2Mux idac2mux
)
{
    registers::Register_IDACMUX tmp;

    read(tmp);

    tmp.mux1 = idac1mux;
    tmp.mux2 = idac2mux;

    write(tmp);

    return true;
}

bool
ADS1262::setIDACMagnitude(
    IDAC1Magnitude idac1mag,
    IDAC2Magnitude idac2mag
)
{
    registers::Register_IDACMAG tmp;

    read(tmp);

    tmp.mag1 = idac1mag;
    tmp.mag2 = idac2mag;

    write(tmp);

    return true;
}

bool
ADS1262::calibrateOffset()
{
    registers::Register_INPMUX inmux;
    registers::Register_INPMUX floating;

    floating.muxn = registers::Register_INPMUX::MuxNegative::FLOAT;
    floating.muxp = registers::Register_INPMUX::MuxPositive::FLOAT;

    read(inmux);

    _start.clear();
    write(floating); // Float inputs
    _start.set();

    core::os::Thread::sleep(core::os::Time::ms(100)); // I do not know if this is really needed

    cmd(CMD_SFOCAL1);
    wait();

    write(inmux); // Restore old mux configuration

    return true;
}

bool
ADS1262::_read(
    uint8_t address,
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
}

bool
ADS1262::_write(
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
} // ADS1262::write

void
ADS1262::cmd(
    uint8_t command
)
{
    _spi.acquireBus();
    _spi.select();
    _spi.send(1, &command);
    _spi.deselect();
    _spi.releaseBus();
}

bool
ADS1262::update()
{
    uint8_t txbuf;
    uint8_t rxbuf[6];

    txbuf = CMD_RDATA1;

    _spi.acquireBus();
    _spi.select();
    _spi.send(1, &txbuf);
    _spi.receive(6, &rxbuf);
    _spi.deselect();
    _spi.releaseBus();

    //data.t = _timestamp; // TODO: Fixs
    _data = (rxbuf[1] << 24) | (rxbuf[2] << 16) | (rxbuf[3] << 8) | rxbuf[4];

    _status = rxbuf[0];

    _status.reset   = 0; // Override don't care bits
    _status.extclk  = 0; // Override don't care bits
    _status.ref_alm = 0; // Override ref low voltage

    // TODO: verify checksum

    return _status == 0b01000000; // A new measure has been taken, without alarms
} // ADS1262::update

ADS1262::Status
ADS1262::getStatus() const
{
    return _status;
}

float
ADS1262::get()
{
    return ((float)_data) * (1.0f / 0x7FFFFFFF);
}

int32_t
ADS1262::getRaw()
{
    return _data;
}

bool
ADS1262::wait()
{
    chSysLock();
    _runner = &core::os::Thread::self();
    core::os::Thread::Return msg = core::os::Thread::sleep();
    chSysUnlock();

    return msg == MSG_DATA_READY;
}
}
}
