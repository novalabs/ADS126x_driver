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
    uint8_t power;

    _start.clear();
    _reset.clear();
    core::os::Thread::sleep(core::os::Time::ms(10));
    _reset.set();
    core::os::Thread::sleep(core::os::Time::ms(500));

    power = read(Register_POWER::ADDRESS);

    if (power != 0x11) {
        return false;
    }

    return true;
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
#if 0
    uint8_t tmp;

    tmp  = read(REG_MODE2);
    tmp &= ~(0x07 << 4);
    tmp |= (gain << 4);
    write(REG_MODE2, tmp);
#endif

    Register_MODE2 tmp = read(Register_MODE2::ADDRESS);

    tmp.bypass = enabled ? 0 : 1;
    tmp.gain   = gain;

    write(Register_MODE2::ADDRESS, tmp);

    return true;
} // ADS1262::setGain

bool
ADS1262::setDataRate(
    DataRate data_rate
)
{
#if 0
    uint8_t tmp;

    tmp  = read(REG_MODE2);
    tmp &= ~0x0F;
    tmp |= data_rate;
    write(REG_MODE2, tmp);
#endif

    Register_MODE2 tmp = read(Register_MODE2::ADDRESS);

    tmp.dr = data_rate;

    write(Register_MODE2::ADDRESS, tmp);

    return true;
} // ADS1262::setDataRate

bool
ADS1262::setFilter(
    Filter filter
)
{
    Register_MODE1 tmp = read(Register_MODE1::ADDRESS);

    tmp.filter = filter;

    write(Register_MODE1::ADDRESS, tmp);

    return true;
}

bool
ADS1262::setDelay(
    Delay delay
)
{
    Register_MODE0 tmp = read(Register_MODE0::ADDRESS);

    tmp.delay = delay;

    write(Register_MODE0::ADDRESS, tmp);

    return true;
}

bool
ADS1262::setInputMux(
    InputMuxPositive positive,
    InputMuxNegative negative
)
{
    Register_INPMUX tmp = read(Register_INPMUX::ADDRESS);

    tmp.muxp = positive;
    tmp.muxn = negative;

    write(Register_INPMUX::ADDRESS, tmp);

    return true;
}

bool
ADS1262::setReferenceMux(
    ReferenceMuxPositive positive,
    ReferenceMuxNegative negative
)
{
    Register_REFMUX tmp = read(Register_REFMUX::ADDRESS);

    tmp.rmuxp = positive;
    tmp.rmuxn = negative;

    write(Register_REFMUX::ADDRESS, tmp);

    return true;
}

bool
ADS1262::setReferencePolarityReversal(
    bool reverse
)
{
    Register_MODE0 tmp = read(Register_MODE0::ADDRESS);

    tmp.refrev = reverse ? 1 : 0;

    write(Register_MODE0::ADDRESS, tmp);

    return true;
}

bool
ADS1262::setIDACMux(
    IDAC1Mux idac1mux,
    IDAC2Mux idac2mux
)
{
    Register_IDACMUX tmp = read(Register_IDACMUX::ADDRESS);

    tmp.mux1 = idac1mux;
    tmp.mux2 = idac2mux;

    write(Register_IDACMUX::ADDRESS, tmp);

    return true;
}

bool
ADS1262::setIDACMagnitude(
    IDAC1Magnitude idac1mag,
    IDAC2Magnitude idac2mag
)
{
    Register_IDACMAG tmp = read(Register_IDACMAG::ADDRESS);

    tmp.mag1 = idac1mag;
    tmp.mag2 = idac2mag;

    write(Register_IDACMAG::ADDRESS, tmp);

    return true;
}

bool
ADS1262::calibrateOffset()
{
    Register_INPMUX inmux = read(Register_INPMUX::ADDRESS); // Save mux configuration

    _start.clear();
    write(Register_INPMUX::ADDRESS, 0xFF); // Float inputs
    _start.set();

    core::os::Thread::sleep(core::os::Time::ms(100)); // I do not know if this is really needed

    cmd(CMD_SFOCAL1);
    wait();

    write(Register_INPMUX::ADDRESS, inmux); // Restore old mux configuration

    return true;
}

uint8_t
ADS1262::read(
    uint8_t address
)
{
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

    return rxbuf[2];
}

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

void
ADS1262::write(
    uint8_t address,
    uint8_t data
)
{
    uint8_t txbuf[3];

    switch (address) {
/* READ ONLY REGISTERS */
      case Register_ID::ADDRESS:
          /* Ignore */
          return;

/* READ/WRITE REGISTERS */
      case Register_POWER::ADDRESS:
      case Register_INTERFACE::ADDRESS:
      case Register_MODE0::ADDRESS:
      case Register_MODE1::ADDRESS:
      case Register_MODE2::ADDRESS:
      case Register_INPMUX::ADDRESS:
      case Register_OFCAL0::ADDRESS:
      case Register_OFCAL1::ADDRESS:
      case Register_OFCAL2::ADDRESS:
      case Register_FSCAL0::ADDRESS:
      case Register_FSCAL1::ADDRESS:
      case Register_FSCAL2::ADDRESS:
      case Register_IDACMUX::ADDRESS:
      case Register_IDACMAG::ADDRESS:
      case Register_REFMUX::ADDRESS:
      case Register_TDACP::ADDRESS:
      case Register_TDACN::ADDRESS:
/*
     case Register_GPIOCON::ADDRESS:
     case Register_GPIODIR::ADDRESS:
     case Register_GPIODAT::ADDRESS:
     case Register_ADC2CFG::ADDRESS:
     case Register_ADC2MUX::ADDRESS:
     case Register_ADC2OFC0::ADDRESS:
     case Register_ADC2OFC1::ADDRESS:
     case Register_ADC2FSC0::ADDRESS:
     case Register_ADC2FSC1::ADDRESS:
 */
          txbuf[0] = CMD_WREG | address;
          txbuf[1] = 0x00;
          txbuf[2] = data;

          _spi.acquireBus();
          _spi.select();
          _spi.send(3, txbuf);
          _spi.deselect();
          _spi.releaseBus();

          break;

/* RESERVED REGISTERS */
      default:
          /* Reserved register must not be written! */
          CORE_ASSERT(FALSE);
          break;
    } // switch
} // ADS1262::write

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

StatusByte
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
