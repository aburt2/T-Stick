/******************************************************************** 
Defines presets for the T-Stick
********************************************************************/
//*************************4GW Presets*******************************
//#define tstick_4gw_lolin_capsense
//#define tstick_4gw_lolin_trill
//#define tstick_4gw_tinypico_capsense
//#define tstick_4gw_tinypico_trill

//*************************5GW Presets*******************************
//#define tstick_5gw_trill_beta
//#define tstick_5gw_enchanti_beta
//#define tstick_5gw_trill_main
//#define tstick_5gw_enchanti_main

//Custom
//#define tstick_custom


/******************************************************************** 
General Properties for the T-Stick
********************************************************************/
#include "imu-cal.h"
#include <vector>

// General includes for the Sensors
#include "imu.h"
#include "touch.h"
#include "fsr.h"
#include "button.h"
#include "led.h"
// Include standard sensors
Button button;
Fsr fsr;
Led led;

//#define TSTICK_SIZE 60
#define I2C_UPDATE_FREQ 400000 // Note that the I2C frequency is capped by Wire at 1MHz

// Feedback sensors
#define BATTERY_UPDATE_RATE 1000000 // us ( 1 Hz)

// Specific Properties
#ifdef tstick_4gw_lolin_capsense
    // Pin definitions
    #define SDA_PIN 22
    #define SCL_PIN 21
    #define FSR_PIN 33
    #define LED_PIN 5
    #define BUTTON_PIN 15
    #define SLEEP_PIN GPIO_NUM_15
    
    // Sleep pins (pins to isolate/wakeup when going in and out of deep sleep)
    std::vector<gpio_num_t> sleep_pins = {GPIO_NUM_22, GPIO_NUM_21, GPIO_NUM_33};
    #define NUM_ISOLATE_PINS 3

    // Boards + Sensors
    #define INDICATOR_LED
    #define imu_LSM9DS1
    #define touch_IDMIL
    #define fg_NONE

    // Initialise sensors
    #include <driver/rtc_io.h> // needed for sleep

    #include "IDMIL-touch/idmil_touch.h"
    #define TOUCH_MAX 1
    Capsense touch;
    idmil_touch_config tstick_touchconfig = {
        -1, // default touch device (not used)
        16, // default touch size
        -1, // noise threshold (not used)
        -1, // touch processing mode (not used)
        -1, // comm mode (not used)
    };

    #include "lsm9ds1/lsm9ds1_imu.h"
    lsm9ds1_config motion_config(Wire);
    LSM9DS1_IMU imu;
#endif

#ifdef tstick_4gw_lolin_trill
    // Pin definitions
    #define SDA_PIN 22
    #define SCL_PIN 21
    #define FSR_PIN 33
    #define LED_PIN 5
    #define BATTERY_PIN 35 // read battery voltage
    #define BUTTON_PIN 15
    #define SLEEP_PIN GPIO_NUM_15

    // Sleep pins (pins to isolate/wakeup when going in and out of deep sleep)
    std::vector<gpio_num_t> sleep_pins = {GPIO_NUM_22, GPIO_NUM_21, GPIO_NUM_33};
    #define NUM_ISOLATE_PINS 3

    // Boards + Sensors
    #define INDICATOR_LED
    #define imu_LSM9DS1
    #define touch_Trill
    #define fg_NONE

    // Initialise sensors
    #include <driver/rtc_io.h> // needed for sleep

    #include "Trill-touch/trill_touch.h"
    #define TOUCH_MAX 511
    TrillTouch touch;
    trill_config tstick_touchconfig = {
        Trill::TRILL_CRAFT, // default use the trill craft device
        TRILL_BASETOUCHSIZE, // default touch size
        0, // noise threshold
        -1, // touch processing mode (not used)
        -1, // comm mode (not used)
    };

    #include "lsm9ds1/lsm9ds1_imu.h"
    lsm9ds1_config motion_config(Wire);
    LSM9DS1_IMU imu;
#endif

#ifdef tstick_4gw_tinypico_capsense
    // Pin definitions
    #define SDA_PIN 22
    #define SCL_PIN 21
    #define FSR_PIN 33
    #define LED_PIN 5
    #define BUTTON_PIN 15
    #define SLEEP_PIN GPIO_NUM_15

    // Sleep pins (pins to isolate/wakeup when going in and out of deep sleep)
    std::vector<gpio_num_t> sleep_pins = {GPIO_NUM_22, GPIO_NUM_21, GPIO_NUM_33, GPIO_NUM_35};
    #define NUM_ISOLATE_PINS 4

    // Boards + Sensors
    #define imu_LSM9DS1
    #define touch_IDMIL
    #define fg_NONE

    // Iniliase tinypico
    #include "TinyPICO.h"
    TinyPICO tinypico = TinyPICO();

    // Initialise sensors
    #include <driver/rtc_io.h> // needed for sleep

    #include "IDMIL-touch/idmil_touch.h"
    #define TOUCH_MAX 1
    Capsense touch;
    idmil_touch_config tstick_touchconfig = {
        -1, // default touch device (not used)
        16, // default touch size
        -1, // noise threshold (not used)
        -1, // touch processing mode (not used)
        -1, // comm mode (not used)
    };

    #include "lsm9ds1/lsm9ds1_imu.h"
    lsm9ds1_config motion_config(Wire);
    LSM9DS1_IMU imu;
#endif

#ifdef tstick_4gw_tinypico_trill
    // Pin definitions
    #define SDA_PIN 22
    #define SCL_PIN 21
    #define FSR_PIN 33
    #define LED_PIN 5
    #define BATTERY_PIN 35 // read battery voltage
    #define BUTTON_PIN 15
    #define SLEEP_PIN GPIO_NUM_15

    // Sleep pins (pins to isolate/wakeup when going in and out of deep sleep)
    std::vector<gpio_num_t> sleep_pins = {GPIO_NUM_22, GPIO_NUM_21, GPIO_NUM_33, GPIO_NUM_35};
    #define NUM_ISOLATE_PINS 4

    // Boards + Sensors
    #define imu_LSM9DS1
    #define touch_Trill
    #define fg_NONE

    // Iniliase tinypico
    #include "TinyPICO.h"
    TinyPICO tinypico = TinyPICO();

    // Initialise sensors
    #include <driver/rtc_io.h> // needed for sleep

    #include "Trill-touch/trill_touch.h"
    #define TOUCH_MAX 511
    TrillTouch touch;
    trill_config tstick_touchconfig = {
        Trill::TRILL_CRAFT, // default use the trill craft device
        TRILL_BASETOUCHSIZE, // default touch size
        0, // noise threshold
        -1, // touch processing mode (not used)
        -1, // comm mode (not used)
    };

    #include "lsm9ds1/lsm9ds1_imu.h"
    lsm9ds1_config motion_config(Wire);
    LSM9DS1_IMU imu;
#endif


#ifdef tstick_5gw_dev
    // Pin definitions
    #define SDA_PIN 21
    #define SCL_PIN 14
    #define FSR_PIN 8
    #define LDO_PIN 39
    #define BUTTON_PIN 9
    #define FUELGAUE_INT_PIN 17
    #define IMU_INT_PIN 48
    #define LED_PIN 15
    #define ORANGE_LED 16
    #define SLEEP_PIN GPIO_NUM_9

    #define MULTIPLE_WIRE_BUS
    #define I2C2_UPDATE_FREQ 1000000
    #define SDA2_PIN 18
    #define SCL2_PIN 17

    // Sleep pins (pins to isolate/wakeup when going in and out of deep sleep)
    std::vector<gpio_num_t> sleep_pins = {GPIO_NUM_21, GPIO_NUM_14, GPIO_NUM_8};
    #define NUM_ISOLATE_PINS 3

    // Boards + Sensors
    #define LDO2
    #define INDICATOR_LED
    #define board_ENCHANTI_rev2
    #define imu_ICM20948
    #define touch_ENCHANTI
    #define fg_MAX17055

    // Initialise sensors
    #include <driver/rtc_io.h> // needed for sleep
    #include "Enchanti-touch/enchanti_touch.h"
    #define TOUCH_MAX 4095
    EnchantiTouch touch; // Wire
    enchanti_touch_config tstick_touchconfig = {
        -1, // default use the trill craft device
        ENCHANTI_BASETOUCHSIZE, // default touch size
        0, // noise threshold
        Mode::DIFF, // touch processing mode
        COMMS::I2C_MODE, // comm mode 
    };

    // Uses Wire1
    #include "MAX17055/fuelgauge_max17055.h"
    #define FUELGAUGE_WIRE Wire1
    MAX17055_FUELGAUGE fuelgauge;
    fuelgauge_config fg_config = {
        FUELGAUGE_WIRE, // wire class
        0x36, //i2c_addr
        2000, // capacity (mAh)
        50, // End of charge Current (mA)
        10, // rsense (mOhm)
        3, // empty voltage (V)
        3.88, //recovery voltage (V)
        0, // soc
        0, // rcomp
        0, // tempco
        0, // fullcap
        0, // fullcapnorm
        0, // Charge Cycles
    };

    #include "icm42670_mmc5633/icm42670_mmc5633_imu.h"
    #define IMU_SPI SPI
    #define IMU_WIRE Wire1
    icm42670_mmc5633_config motion_config(IMU_SPI, IMU_WIRE);
    ICM42670_MMC5633_IMU imu;
#endif

#ifdef tstick_5gw_trill_main
    // Pin definitions
    #define SDA_PIN 21
    #define SCL_PIN 14
    #define FSR_PIN 8
    #define BUTTON_PIN 9
    #define LED_PIN 15
    #define ORANGE_LED 16
    #define IMU_INT_PIN 48
    #define FUELGAUE_INT_PIN 17
    #define SLEEP_PIN GPIO_NUM_9

    // Sleep pins (pins to isolate/wakeup when going in and out of deep sleep)
    std::vector<gpio_num_t> sleep_pins = {GPIO_NUM_21, GPIO_NUM_14, GPIO_NUM_8};
    #define NUM_ISOLATE_PINS 3

    // Boards + Sensors
    #define INDICATOR_LED
    #define board_ENCHANTI_rev2
    #define imu_ICM20948
    #define touch_TRILL
    #define fg_MAX17055

    // Initialise sensors
    #include <driver/rtc_io.h> // needed for sleep

    #include "Trill-touch/trill_touch.h"
    #define TOUCH_MAX 511
    TrillTouch touch;
    trill_config tstick_touchconfig = {
        Trill::TRILL_CRAFT, // default use the trill craft device
        TRILL_BASETOUCHSIZE, // default touch size
        0, // noise threshold
        -1, // touch processing mode (not used)
        -1, // comm mode (not used)
    };

    #include "MAX17055/fuelgauge_max17055.h"
    MAX17055_FUELGAUGE fuelgauge;
    fuelgauge_config fg_config = {
        Wire, // wire class
        0x36, //i2c_addr
        2000, // capacity (mAh)
        50, // End of charge Current (mA)
        10, // rsense (mOhm)
        3, // empty voltage (V)
        3.88, //recovery voltage (V)
        0, // soc
        0, // rcomp
        0, // tempco
        0, // fullcap
        0, // fullcapnorm
        0, // Charge Cycles
    };

    #include "icm20948/icm20948_imu.h"
    icm20948_imu_config motion_config(Wire);
    ICM20948_IMU imu;
#endif

#ifdef tstick_5gw_enchanti_main
    // Pin definitions
    #define SDA_PIN 21
    #define SCL_PIN 14
    #define FSR_PIN 8
    #define LDO_PIN 39
    #define BUTTON_PIN 9
    #define FUELGAUE_INT_PIN 17
    #define IMU_INT_PIN 48
    #define LED_PIN 15
    #define ORANGE_LED 16
    #define SLEEP_PIN GPIO_NUM_9

    // Sleep pins (pins to isolate/wakeup when going in and out of deep sleep)
    std::vector<gpio_num_t> sleep_pins = {GPIO_NUM_21, GPIO_NUM_14, GPIO_NUM_8};
    #define NUM_ISOLATE_PINS 3

    // Boards + Sensors
    #define LDO2
    #define INDICATOR_LED
    #define board_ENCHANTI_rev2
    #define imu_ICM20948
    #define touch_ENCHANTI
    #define fg_MAX17055

    // Initialise sensors
    #include <driver/rtc_io.h> // needed for sleep
    #include "Enchanti-touch/enchanti_touch.h"
    #define TOUCH_MAX 4095
    EnchantiTouch touch;
    enchanti_touch_config tstick_touchconfig = {
        -1, // default use the trill craft device
        ENCHANTI_BASETOUCHSIZE, // default touch size
        0, // noise threshold
        Mode::DIFF, // touch processing mode
        COMMS::I2C_MODE, // comm mode 
    };

    #include "MAX17055/fuelgauge_max17055.h"
    MAX17055_FUELGAUGE fuelgauge;
    fuelgauge_config fg_config = {
        Wire, // wire class
        0x36, //i2c_addr
        2000, // capacity (mAh)
        50, // End of charge Current (mA)
        10, // rsense (mOhm)
        3, // empty voltage (V)
        3.88, //recovery voltage (V)
        0, // soc
        0, // rcomp
        0, // tempco
        0, // fullcap
        0, // fullcapnorm
        0, // Charge Cycles
    };

    #include "ICM20948/icm20948_imu.h"
    icm20948_imu_config motion_config(Wire);
    ICM20948_IMU imu;
#endif

#ifdef tstick_custom
    // Define pins
    // #define SDA_PIN 21
    // #define SCL_PIN 14
    // #define FSR_PIN 8
    // #define BUTTON_PIN 9
    // #define FUELGAUE_INT_PIN 17
    // #define LED_PIN 15
    // #define ORANGE_LED 16

    /// Define Board
    //#define board_ENCHANTI_rev1
    //#define board_ENCHANTI_rev2
    //#define board_ENCHANTI_rev3

    /// Define Sensors
    //#define touch_TRILL
    //#define touch_ENCHANTI
    //#define touch_IDMIL

    //#define TSTICK_IMU MIMUBOARD::mimu_ICM20948
    //#define TSTICK_IMU MIMUBOARD::mimu_LSM9DS1

    //#define fg_MAX17055
    //#define fg_NONE

    // #define INDICATOR_LED // use with LED_PIN if using a discrete LED and LED PWM library

    // Include the other sensors
    #ifdef touch_IDMIL
        #include "IDMIl-touch/idmil_touch.h"
        Capsense touch;
    #endif

    #ifdef touch_TRILL
        #include "Trill-touch/trill_touch.h"
        TrillTouch touch;
    #endif

    #ifdef touch_ENCHANTI
        #include "Enchanti-touch/enchanti_touch.h"
        EnchantiTouch touch;
    #endif


    #ifdef fg_MAX17055
        #include <batt.h>
        FUELGAUGE fuelgauge;
        fuelgauge_config fg_config = {
            0x36, //i2c_addr
            2000, // capacity (mAh)
            50, // End of charge Current (mA)
            10, // rsense (mOhm)
            3, // empty voltage (V)
            3.88, //recovery voltage (V)
            0, // soc
            0, // rcomp
            0, // tempco
            0, // fullcap
            0, // fullcapnorm
            0, // Charge Cycles
        };
    #endif
#endif
