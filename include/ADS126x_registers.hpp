/* COPYRIGHT (c) 2016-2018 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#pragma once

#include <core/utils/Bitfield.hpp>

#undef CRC // Workaround for ST header files macro

namespace core {
namespace ADS126x_driver {
namespace registers {
/*! \brief Register Begin
 *
 * \param name Name of the register
 * \param address Address of the register
 * \param access Access type   core::ADS1262_driver::registers::AccessType
 */
#define REGISTER_BEGIN(__name__, __address__, __access__)  BEGIN_BITFIELD_TYPE(__name__, uint8_t) \
    enum {ADDRESS = __address__}; \
    enum {ACCESS = __access__};

/*! \brief Register Bits
 *
 * \param offset Field starting bit
 * \param len Field length
 * \param name Field name
 */
#define REGISTER_BITS(__offset__, __len__, __name__) ADD_BITFIELD_MEMBER(__name__, __offset__, __len__)

/*! \brief Register Enum
 *
 * \param offset Field starting bit
 * \param len Field length
 * \param name Field name
 * \param enum Type of the enum
 */
#define REGISTER_ENUM(__offset__, __len__, __name__, __enum__) ADD_ENUM_BITFIELD_MEMBER(__name__, __offset__, __len__, __enum__)

/*! \brief Register Reserved
 *
 * \param offset Field starting bit
 * \param len Field length
 * \param name Field name
 * \param value Unmodifiable value
 */
#define REGISTER_RSRV(__offset__, __len__, __name__, __value__) ADD_CONST_BITFIELD_MEMBER(__name__, __offset__, __len__, __value__)

/*! \brief Register End
 */
#define REGISTER_END() END_BITFIELD_TYPE()

/*! \brief 8 bit bitfield Begin
 *
 * \param name Name of the bitfield
 */
#define BYTE_BITFIELD_BEGIN(__name__)  BEGIN_BITFIELD_TYPE(__name__, uint8_t)

/*! \brief 8 bit Bitfield Bits
 *
 * \param offset Field starting bit
 * \param len Field length
 * \param name Field name
 */
#define BYTE_BITFIELD_BITS(__offset__, __len__, __name__) ADD_BITFIELD_MEMBER(__name__, __offset__, __len__)

/*! \brief 8 bit Bitfield Enum
 *
 * \param offset Field starting bit
 * \param len Field length
 * \param name Field name
 * \param enum Type of the enum
 */
#define BYTE_BITFIELD_ENUM(__offset__, __len__, __name__, __enum__) ADD_ENUM_BITFIELD_MEMBER(__name__, __offset__, __len__, __enum__)

/*! \brief 8 bit Bitfield Reserved
 *
 * \param offset Field starting bit
 * \param len Field length
 * \param name Field name
 * \param value Unmodifiable value
 */
#define BYTE_BITFIELD_RSRV(__offset__, __len__, __name__, __value__) ADD_CONST_BITFIELD_MEMBER(__name__, __offset__, __len__, __value__)

/*! \brief 8 bit Bitfield End
 */
#define BYTE_BITFIELD_END() END_BITFIELD_TYPE()

/*! \brief Register access type */
enum AccessType {
    READ_ONLY  = 0x01, //!< Read only
    WRITE_ONLY = 0x02, //!< Write only
    READ_WRITE = 0x03  //!< Read and Write
};

// ************************************************************************* //
// *** REGISTERS
// ************************************************************************* //

// --- Register_ID ------------------------------------------------------------
// ADDRESS: 0x00
REGISTER_BEGIN(Register_ID, 0x00, READ_ONLY)
enum class DevID : uint8_t {
    ADS1262 = 0x00,
    ADS1263 = 0x01,
};

REGISTER_ENUM(5, 3, dev_id, DevID);
REGISTER_BITS(0, 5, rev_id);
REGISTER_END()
// ----------------------------------------------------------------------------

// --- Register_POWER ---------------------------------------------------------
// ADDRESS: 0x01, RESET = 0x11
REGISTER_BEGIN(Register_POWER, 0x01, READ_WRITE)
REGISTER_RSRV(5, 3, __1, 0x00)
REGISTER_BITS(4, 1, reset) // [1] 1 = reset has occurred
REGISTER_RSRV(2, 2, __2, 0x00)
REGISTER_BITS(1, 1, vbias) // [0] 1 = vbias enabled on AINCOM
REGISTER_BITS(0, 1, intref) // [1] 1 = internal reference enabled
REGISTER_END()
// ----------------------------------------------------------------------------

// --- Register_INTERFACE -----------------------------------------------------
// ADDRESS: 0x02, RESET = 0x05
REGISTER_BEGIN(Register_INTERFACE, 0x02, READ_WRITE)
enum class CRC : uint8_t {
    DISABLED        = 0x00, // 00 = disabled
    ENABLE_CHECKSUM = 0x01,    // 01 = enable checksum byte in checksum mode
    ENABLE_CRC      = 0x02    // 10 = enable checksum byte in CRC mode
};

REGISTER_RSRV(4, 4, __1, 0x00)
REGISTER_BITS(3, 1, timeout) // [0] 1 = spi auto-timeout mode enabled
REGISTER_BITS(2, 1, status) // [1] 1 = status byte included in data read-back
REGISTER_ENUM(0, 2, crc, CRC) // [01] CRC
REGISTER_END()
// ----------------------------------------------------------------------------

// --- Register_MODE0 ---------------------------------------------------------
// ADDRESS: 0x03, RESET = 0x00
REGISTER_BEGIN(Register_MODE0, 0x03, READ_WRITE)
enum class Delay : uint8_t {
    NO_DELAY = 0x00,
    _8_7us   = 0x01,
    _17us    = 0x02,
    _35us    = 0x03,
    _69us    = 0x04,
    _139us   = 0x05,
    _278us   = 0x06,
    _555us   = 0x07,
    _1_1ms   = 0x08,
    _2_2ms   = 0x09,
    _4_4ms   = 0x0A,
    _8_8ms   = 0x0B
};

REGISTER_BITS(7, 1, refrev) // [0] 1 = reverse polarity of reference multiplexer output
REGISTER_BITS(6, 1, runmode) // [0] 0 = continuous conversion, 1 = one shot conversion
REGISTER_BITS(5, 1, idac_rotation) // [0] IDAC rotation
REGISTER_BITS(4, 1, input_chop) // [0] input chop
REGISTER_ENUM(0, 4, delay, Delay) // [00] delay
REGISTER_END()
// ----------------------------------------------------------------------------

// --- Register_MODE1 ---------------------------------------------------------
// ADDRESS: 0x04, RESET = 0x80
REGISTER_BEGIN(Register_MODE1, 0x04, READ_WRITE)
enum class Filter : uint8_t {
    SINC_1 = 0x00,
    SINC_2 = 0x01,
    SINC_3 = 0x02,
    SINC_4 = 0x03,
    FIR    = 0x04
};

REGISTER_ENUM(5, 3, filter, Filter) // [100] filter
REGISTER_BITS(4, 1, sbadc) // [0] 0 = sensor bias connected to ADC1 mux out, 1 = sensor bias connected to ADC1 mux out
REGISTER_BITS(3, 1, sbpol) // [0] 0 = sensor bias pull-up mode
REGISTER_BITS(0, 3, sbmag) // [000] 0 = no sensor bias current or resistor
REGISTER_END()
// ----------------------------------------------------------------------------

// --- Register_MODE2 ---------------------------------------------------------
// ADDRESS: 0x05, RESET = 0x04
REGISTER_BEGIN(Register_MODE2, 0x05, READ_WRITE)
enum class Gain : uint8_t {
    GAIN_1  = 0x00,
    GAIN_2  = 0x01,
    GAIN_4  = 0x02,
    GAIN_8  = 0x03,
    GAIN_16 = 0x04,
    GAIN_32 = 0x05,
};

enum class DataRate : uint8_t {
    DATARATE_2_5   = 0x00,
    DATARATE_5     = 0x01,
    DATARATE_10    = 0x02,
    DATARATE_16_6  = 0x03,
    DATARATE_20    = 0x04,
    DATARATE_50    = 0x05,
    DATARATE_60    = 0x06,
    DATARATE_100   = 0x07,
    DATARATE_400   = 0x08,
    DATARATE_1200  = 0x09,
    DATARATE_2400  = 0x0A,
    DATARATE_4800  = 0x0B,
    DATARATE_7200  = 0x0C,
    DATARATE_14400 = 0x0D,
    DATARATE_19200 = 0x0E,
    DATARATE_38400 = 0x0F,
};

REGISTER_BITS(7, 1, bypass) // [0] 0 = PGA enabled
REGISTER_ENUM(4, 3, gain, Gain) // [0] Gain
REGISTER_ENUM(0, 4, dr, DataRate) // [0] Data rate
REGISTER_END()
// ----------------------------------------------------------------------------

// --- Register_INPMUX ---------------------------------------------------------
// ADDRESS: 0x06, RESET = 0x01
REGISTER_BEGIN(Register_INPMUX, 0x06, READ_WRITE)
enum class MuxPositive : uint8_t {
    AIN_0       = 0x00,
    AIN_1       = 0x01,
    AIN_2       = 0x02,
    AIN_3       = 0x03,
    AIN_4       = 0x04,
    AIN_5       = 0x05,
    AIN_6       = 0x06,
    AIN_7       = 0x07,
    AIN_8       = 0x08,
    AIN_9       = 0x09,
    AIN_COM     = 0x0A,
    TEMP        = 0x0B,
    ANALOG_PWR  = 0x0C,
    DIGITAL_PWR = 0x0D,
    TDAC        = 0x0E,
    FLOAT       = 0x0F
};

enum class MuxNegative : uint8_t {
    AIN_0       = 0x00,
    AIN_1       = 0x01,
    AIN_2       = 0x02,
    AIN_3       = 0x03,
    AIN_4       = 0x04,
    AIN_5       = 0x05,
    AIN_6       = 0x06,
    AIN_7       = 0x07,
    AIN_8       = 0x08,
    AIN_9       = 0x09,
    AIN_COM     = 0x0A,
    TEMP        = 0x0B,
    ANALOG_PWR  = 0x0C,
    DIGITAL_PWR = 0x0D,
    TDAC        = 0x0E,
    FLOAT       = 0x0F
};

REGISTER_ENUM(4, 4, muxp, MuxPositive) // [0000] positive input multiplexer selection
REGISTER_ENUM(0, 4, muxn, MuxNegative) // [0001] negative input multiplexer selection
REGISTER_END()
// ----------------------------------------------------------------------------

// --- Register_OFCAL0 --------------------------------------------------------
// ADDRESS: 0x07, RESET = 0x00
REGISTER_BEGIN(Register_OFCAL0, 0x07, READ_WRITE)
REGISTER_BITS(0, 8, ofc_7_0) // [00000000] offset calibration LSB
REGISTER_END()
// ----------------------------------------------------------------------------

// --- Register_OFCAL1 --------------------------------------------------------
// ADDRESS: 0x08, RESET = 0x00
REGISTER_BEGIN(Register_OFCAL1, 0x08, READ_WRITE)
REGISTER_BITS(0, 8, ofc_15_8) // [00000000] offset calibration
REGISTER_END()
// ----------------------------------------------------------------------------

// --- Register_OFCAL2 --------------------------------------------------------
// ADDRESS: 0x09, RESET = 0x00
REGISTER_BEGIN(Register_OFCAL2, 0x09, READ_WRITE)
REGISTER_BITS(0, 8, ofc_23_16) // [00000000] offset calibration MSB
REGISTER_END()
// ----------------------------------------------------------------------------

// --- Register_FSCAL0 --------------------------------------------------------
// ADDRESS: 0x0A, RESET = 0x40
REGISTER_BEGIN(Register_FSCAL0, 0x0A, READ_WRITE)
REGISTER_BITS(0, 8, fscal_7_0) // [01000000] offset calibration LSB
REGISTER_END()
// ----------------------------------------------------------------------------

// --- Register_FSCAL1 --------------------------------------------------------
// ADDRESS: 0x0B, RESET = 0x00
REGISTER_BEGIN(Register_FSCAL1, 0x0B, READ_WRITE)
REGISTER_BITS(0, 8, fsc_15_8) // [00000000] offset calibration
REGISTER_END()
// ----------------------------------------------------------------------------

// --- Register_FSCAL2 --------------------------------------------------------
// ADDRESS: 0x0C, RESET = 0x00
REGISTER_BEGIN(Register_FSCAL2, 0x0C, READ_WRITE)
REGISTER_BITS(0, 8, fsc_23_16) // [00000000] offset calibration MSB
REGISTER_END()
// ----------------------------------------------------------------------------

// --- Register_IDACMUX ---------------------------------------------------------
// ADDRESS: 0x0D, RESET = 0xBB
REGISTER_BEGIN(Register_IDACMUX, 0x0D, READ_WRITE)
enum class MuxIDAC2 : uint8_t {
    AIN_0   = 0x00,
    AIN_1   = 0x01,
    AIN_2   = 0x02,
    AIN_3   = 0x03,
    AIN_4   = 0x04,
    AIN_5   = 0x05,
    AIN_6   = 0x06,
    AIN_7   = 0x07,
    AIN_8   = 0x08,
    AIN_9   = 0x09,
    AIN_COM = 0x0A,
    NONE    = 0x0B,
};

enum class MuxIDAC1 : uint8_t {
    AIN_0   = 0x00,
    AIN_1   = 0x01,
    AIN_2   = 0x02,
    AIN_3   = 0x03,
    AIN_4   = 0x04,
    AIN_5   = 0x05,
    AIN_6   = 0x06,
    AIN_7   = 0x07,
    AIN_8   = 0x08,
    AIN_9   = 0x09,
    AIN_COM = 0x0A,
    NONE    = 0x0B,
};

REGISTER_ENUM(4, 4, mux2, MuxIDAC2) // [1011] idac2 output multiplexer selection
REGISTER_ENUM(0, 4, mux1, MuxIDAC1) // [1011] idac1 output multiplexer selection
REGISTER_END()
// ----------------------------------------------------------------------------

// --- Register_IDACMAG ---------------------------------------------------------
// ADDRESS: 0x0E, RESET = 0x00
REGISTER_BEGIN(Register_IDACMAG, 0x0E, READ_WRITE)
enum class MagIDAC2 : uint8_t {
    OFF     = 0x00,
    _50uA   = 0x01,
    _100uA  = 0x02,
    _250uA  = 0x03,
    _500uA  = 0x04,
    _750uA  = 0x05,
    _1000uA = 0x06,
    _1500uA = 0x07,
    _2000uA = 0x08,
    _2500uA = 0x09,
    _3000uA = 0x0A,
};

enum class MagIDAC1 : uint8_t {
    OFF     = 0x00,
    _50uA   = 0x01,
    _100uA  = 0x02,
    _250uA  = 0x03,
    _500uA  = 0x04,
    _750uA  = 0x05,
    _1000uA = 0x06,
    _1500uA = 0x07,
    _2000uA = 0x08,
    _2500uA = 0x09,
    _3000uA = 0x0A,
};

REGISTER_ENUM(4, 4, mag2, MagIDAC2) // [0000] idac2 current magnitude
REGISTER_ENUM(0, 4, mag1, MagIDAC1) // [0000] idac1 current magnitude
REGISTER_END()
// ----------------------------------------------------------------------------

// --- Register_REFMUX --------------------------------------------------------
// ADDRESS: 0x0F, RESET = 0x00
REGISTER_BEGIN(Register_REFMUX, 0x0F, READ_WRITE)
enum class MuxPositive : uint8_t {
    INTERNAL_2V5  = 0x00,
    AIN_0         = 0x01,
    AIN_2         = 0x02,
    AIN_4         = 0x03,
    INTERNAL_AVDD = 0x04
};

enum class MuxNegative : uint8_t {
    INTERNAL_2V5  = 0x00,
    AIN_1         = 0x01,
    AIN_3         = 0x02,
    AIN_5         = 0x03,
    INTERNAL_AVSS = 0x04
};

REGISTER_RSRV(6, 2, __1, 0x00)
REGISTER_ENUM(3, 3, rmuxp, MuxPositive) // [000] positive reference multiplexer selection
REGISTER_ENUM(0, 3, rmuxn, MuxNegative) // [000] negative reference multiplexer selection
REGISTER_END()
// ----------------------------------------------------------------------------

// --- Register_TDACP --------------------------------------------------------
// ADDRESS: 0x10, RESET = 0x00
REGISTER_BEGIN(Register_TDACP, 0x10, READ_WRITE)
enum class MagPositive : uint8_t {
    _4_5V       = 0b01001,
    _3_5V       = 0b01000,
    _3_0V       = 0b00111,
    _2_75V      = 0b00110,
    _2_625V     = 0b00101,
    _2_5625V    = 0b00100,
    _2_53125V   = 0b00011,
    _2_515625V  = 0b00010,
    _2_5078125V = 0b00001,
    _2_5V       = 0b00000,
    _2_4921875V = 0b10001,
    _2_484375V  = 0b10010,
    _2_46875V   = 0b10011,
    _2_4375V    = 0b10100,
    _2_375V     = 0b10101,
    _2_25V      = 0b10110,
    _2_0V       = 0b10111,
    _1_5V       = 0b11000,
    _0_5V       = 0b11001
};

REGISTER_BITS(7, 1, outp) // [0] 1 = TDACP connected to AIN6
REGISTER_RSRV(5, 2, __1, 0x00)
REGISTER_ENUM(0, 5, magp, MagPositive) // [00000] TDACP output magnitude
REGISTER_END()
// ----------------------------------------------------------------------------

// --- Register_TDACN ---------------------------------------------------------
// ADDRESS: 0x11, RESET = 0x00
REGISTER_BEGIN(Register_TDACN, 0x11, READ_WRITE)
enum class MagNegative : uint8_t {
    _4_5V       = 0b01001,
    _3_5V       = 0b01000,
    _3_0V       = 0b00111,
    _2_75V      = 0b00110,
    _2_625V     = 0b00101,
    _2_5625V    = 0b00100,
    _2_53125V   = 0b00011,
    _2_515625V  = 0b00010,
    _2_5078125V = 0b00001,
    _2_5V       = 0b00000,
    _2_4921875V = 0b10001,
    _2_484375V  = 0b10010,
    _2_46875V   = 0b10011,
    _2_4375V    = 0b10100,
    _2_375V     = 0b10101,
    _2_25V      = 0b10110,
    _2_0V       = 0b10111,
    _1_5V       = 0b11000,
    _0_5V       = 0b11001
};

REGISTER_BITS(7, 1, outn)  // [0] 1 = TDACN connected to AIN6
REGISTER_RSRV(5, 2, __1, 0x00)
REGISTER_ENUM(0, 5, magn, MagNegative) // [00000] TDACN output magnitude
REGISTER_END()
// ----------------------------------------------------------------------------

// --- Register_GPIOCON -------------------------------------------------------
// ADDRESS: 0x12, RESET = 0x00
REGISTER_BEGIN(Register_GPIOCON, 0x12, READ_WRITE)
REGISTER_BITS(0, 7, con)  // TODO
REGISTER_END()
// ----------------------------------------------------------------------------

// --- Register_GPIODIR -------------------------------------------------------
// ADDRESS: 0x13, RESET = 0x00
REGISTER_BEGIN(Register_GPIODIR, 0x13, READ_WRITE)
REGISTER_BITS(0, 7, dir)  // TODO
REGISTER_END()
// ----------------------------------------------------------------------------

// --- Register_GPIODAT -------------------------------------------------------
// ADDRESS: 0x14, RESET = 0x00
REGISTER_BEGIN(Register_GPIODAT, 0x14, READ_WRITE)
REGISTER_BITS(0, 7, dat)  // TODO
REGISTER_END()
// ----------------------------------------------------------------------------

// --- Register_ADC2CFG -------------------------------------------------------
// ADDRESS: 0x15, RESET = 0x00
REGISTER_BEGIN(Register_ADC2CFG, 0x15, READ_WRITE)
enum class Gain : uint8_t {
    GAIN_1   = 0x00,
    GAIN_2   = 0x01,
    GAIN_4   = 0x02,
    GAIN_8   = 0x03,
    GAIN_16  = 0x04,
    GAIN_32  = 0x05,
    GAIN_64  = 0x06,
    GAIN_128 = 0x07,
};

enum class ReferenceInput : uint8_t {
    INTERNAL_2V5       = 0x00,
    AIN_0_1            = 0x01,
    AIN_2_3            = 0x02,
    AIN_4_5            = 0x03,
    INTERNAL_AVDD_AVSS = 0x04,
};

enum class DataRate : uint8_t {
    DATARATE_10  = 0x00,
    DATARATE_100 = 0x01,
    DATARATE_400 = 0x02,
    DATARATE_800 = 0x03,
};

REGISTER_ENUM(6, 2, dr2, DataRate) // [00] Data rate
REGISTER_ENUM(3, 3, ref2, ReferenceInput) // [000] Reference selection
REGISTER_ENUM(0, 3, gain2, Gain) // [000] Gain
REGISTER_END()
// ----------------------------------------------------------------------------

// --- Register_ADC2MUX -------------------------------------------------------
// ADDRESS: 0x16, RESET = 0x01
REGISTER_BEGIN(Register_ADC2MUX, 0x16, READ_WRITE)
enum class MuxPositive : uint8_t {
    AIN_0       = 0x00,
    AIN_1       = 0x01,
    AIN_2       = 0x02,
    AIN_3       = 0x03,
    AIN_4       = 0x04,
    AIN_5       = 0x05,
    AIN_6       = 0x06,
    AIN_7       = 0x07,
    AIN_8       = 0x08,
    AIN_9       = 0x09,
    AIN_COM     = 0x0A,
    TEMP        = 0x0B,
    ANALOG_PWR  = 0x0C,
    DIGITAL_PWR = 0x0D,
    TDAC        = 0x0E,
    FLOAT       = 0x0F
};

enum class MuxNegative : uint8_t {
    AIN_0       = 0x00,
    AIN_1       = 0x01,
    AIN_2       = 0x02,
    AIN_3       = 0x03,
    AIN_4       = 0x04,
    AIN_5       = 0x05,
    AIN_6       = 0x06,
    AIN_7       = 0x07,
    AIN_8       = 0x08,
    AIN_9       = 0x09,
    AIN_COM     = 0x0A,
    TEMP        = 0x0B,
    ANALOG_PWR  = 0x0C,
    DIGITAL_PWR = 0x0D,
    TDAC        = 0x0E,
    FLOAT       = 0x0F
};

REGISTER_ENUM(4, 4, muxp, MuxPositive) // [0000] positive input multiplexer selection
REGISTER_ENUM(0, 4, muxn, MuxNegative) // [0001] negative input multiplexer selection
REGISTER_END()
// ----------------------------------------------------------------------------

// --- Register_ADC2OFC0 ------------------------------------------------------
// ADDRESS: 0x17, RESET = 0x00
REGISTER_BEGIN(Register_ADC2OFC0, 0x17, READ_WRITE)
REGISTER_BITS(0, 8, ofc2_7_0) // [00000000] offset calibration LSB
REGISTER_END()
// ----------------------------------------------------------------------------

// --- Register_ADC2OFC1 ------------------------------------------------------
// ADDRESS: 0x18, RESET = 0x00
REGISTER_BEGIN(Register_ADC2OFC1, 0x18, READ_WRITE)
REGISTER_BITS(0, 8, ofc2_15_8) // [00000000] offset calibration MSB
REGISTER_END()
// ----------------------------------------------------------------------------

// --- Register_ADC2FSC0 ------------------------------------------------------
// ADDRESS: 0x19, RESET = 0x00
REGISTER_BEGIN(Register_ADC2FSC0, 0x19, READ_WRITE)
REGISTER_BITS(0, 8, fsc2_7_0) // [00000000] offset calibration LSB
REGISTER_END()
// ----------------------------------------------------------------------------

// --- Register_ADC2FSC1 ------------------------------------------------------
// ADDRESS: 0x1A, RESET = 0x40
REGISTER_BEGIN(Register_ADC2FSC1, 0x1A, READ_WRITE)
REGISTER_BITS(0, 8, fsc2_15_8) // [01000000] offset calibration MSB
REGISTER_END()
// ----------------------------------------------------------------------------

// --- STATUS Byte ------------------------------------------------------------
BYTE_BITFIELD_BEGIN(StatusByte)
BYTE_BITFIELD_BITS(7, 1, adc2)  // 1 = new data (ADC2)
BYTE_BITFIELD_BITS(6, 1, adc1)  // 1 = new data (ADC1)
BYTE_BITFIELD_BITS(5, 1, extclk)  // 0 = internal clock, 1 = external clock
BYTE_BITFIELD_BITS(4, 1, ref_alm)  // 1 = Low reference alarm
BYTE_BITFIELD_BITS(3, 1, pgal_alm)  // 1 = PGA Absolute output low alarm
BYTE_BITFIELD_BITS(2, 1, pgah_alm)  // 1 = PGA Absolute output high alarm
BYTE_BITFIELD_BITS(1, 1, pgad_alm)  // 1 = PGA Differential output alarm
BYTE_BITFIELD_BITS(0, 1, reset)  // 1 = Reset occurred
BYTE_BITFIELD_END()
// ----------------------------------------------------------------------------
}
}
}
