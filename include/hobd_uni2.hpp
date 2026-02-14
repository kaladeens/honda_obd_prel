#include <Arduino.h>
#include <SoftwareSerial.h>
#include <stdint.h>
#include <SoftwareSerialWithHalfDuplex.h>


#define MSG_OFFSET 3
#define RPL_OFFSET 2
#define MAX_TMO 100

// ==========================
// HOBD constants
// ==========================
#define HOBD_CMD 0x20
#define HOBD_RST 0x21

// ==========================
// Offsets (match your table / map)
// ==========================
// Word (big-endian)
#define HOBD_OFF_RPM 0x00

// Bytes
#define HOBD_OFF_VSS 0x02
#define HOBD_OFF_FLAG_08 0x08
#define HOBD_OFF_FLAG_0B 0x0B

#define HOBD_OFF_ECT 0x10
#define HOBD_OFF_IAT 0x11
#define HOBD_OFF_MAP 0x12
#define HOBD_OFF_PA 0x13
#define HOBD_OFF_TPS 0x14
#define HOBD_OFF_O2 0x15
#define HOBD_OFF_VOLT 0x15
#define HOBD_OFF_BAT 0x17
#define HOBD_OFF_ALTF 0x18
#define HOBD_OFF_EL 0x19

#define HOBD_OFF_ERRORS1 0x40
#define HOBD_OFF_ERRORS2 0x50

#define HOBD_OFF_ECUID 0x76

// ECU ID bytes (from your image)
#define HOBD_OFF_ECUID_5 0x78
#define HOBD_OFF_ECUID_4 0x79
#define HOBD_OFF_ECUID_3 0x7A
#define HOBD_OFF_ECUID_2 0x7B
#define HOBD_OFF_ECUID_1 0x7C

// ==========================
// Flag bits (offset 0x08 and 0x0B)
// ==========================
#define HOBD_FLG_STARTER (1 << 0)
#define HOBD_FLG_AC_SWITCH (1 << 1)
#define HOBD_FLG_PAS_PRESS (1 << 2)
#define HOBD_FLG_BRAKE (1 << 3)
#define HOBD_FLG_VTEC_PRESS (1 << 7)

#define HOBD_FLG_MAIN_RELAY (1 << 0) // @0x0B
#define HOBD_FLG_CEL (1 << 5)        // @0x0B

struct EcuCmd
{
    uint8_t cmd;
    uint8_t txlen;
    uint8_t reg;
    uint8_t rxlen;
    uint8_t crc;
};

enum ErrCodes {
    ChecksumErr,
    TimeoutErr,
    DTCErr,
};

#define ErrLen 14
#define DataLen 20

extern const uint8_t startup[];


class ECUData
{

private:
    SoftwareSerialWithHalfDuplex &dlc;

public:
    ECUData(uint8_t obd_sel_in, SoftwareSerialWithHalfDuplex &dlc_in)
        : dlc(dlc_in), obd_sel(obd_sel_in)
    {
    }

    bool init();
    bool sendcmd(EcuCmd &cmd);
    bool scanDtc();
    bool resetEcu();
    static uint8_t mkcrc(const uint8_t *buf, uint8_t len);
    bool readLiveData();

    uint8_t dlcData[DataLen] = {0};
    uint8_t dtcErrs[ErrLen] = {0};
    ErrCodes Errs[ErrLen];
    size_t errLen = 0, dtcLen = 0;

    uint16_t dlctmo = 0;
    // ==============================
    // ECU Sensor Data (Raw Inputs)
    // ==============================
    int rpm = 0;  // engine speed
    int ect = 0;  // engine coolant temp
    int iat = 0;  // intake air temp
    int maps = 0; // manifold absolute pressure
    int baro = 0; // barometric pressure
    int tps = 0;  // throttle position
    int sft = 0;  // short term fuel trim
    int lft = 0;  // long term fuel trim
    int inj = 0;  // injector pulse width
    int ign = 0;  // ignition timing
    int lmt = 0;  // limiter flags
    int iacv = 0; // idle air control valve duty
    int knoc = 0; // knock count/value

    float volt = 0.0; // battery voltage
    float o2 = 0.0;   // primary O2 sensor voltage
    uint8_t vss = 0;  // vehicle speed sensor (raw)
    float alt_fr = 0.0; // alternator load
    float eld = 0.0; //amps Electrical load
    // 0B stuff
    bool main_relay = false;
    bool cel = false;

    // ==============================
    // Switch / State Flags
    // ==============================
    bool sw_aircon = false; // A/C switch signal
    bool sw_brake = false;  // brake switch
    bool sw_vtec = false;   // VTEC activation
    bool sw_starter = false;
    // ==============================
    // Additional External Sensors
    // ==============================
    float volt2 = 0.0; // second voltage input
    float th = 0.0;    // thermistor / external temp
    float afr = 0.0;   // wideband AFR
    float fp = 0.0;    // fuel pressure
    bool cp = false;   // clutch pedal switch

    // ==============================
    // Computed / Peak Values
    // ==============================
    int maf = 0;     // mass airflow (if used)
    int rpmtop = 0;  // peak rpm
    int volttop = 0; // peak voltage
    int mapstop = 0; // peak MAP
    int tpstop = 0;  // peak TPS
    int ecttop = 0;  // max coolant temp
    int iattop = 0;  // max intake air temp

    // ==============================
    // Runtime and Distance Tracking
    // ==============================
    unsigned long vsssum = 0;       // sum for VSS averaging
    unsigned long running_time = 0; // engine total runtime
    unsigned long idle_time = 0;    // idle-only runtime
    unsigned long distance = 0;     // distance traveled

    // ==============================
    // Vehicle State
    // ==============================
    uint8_t gear = 0;   // current gear (calculated)
    uint8_t vsstop = 0; // peak speed
    uint8_t vssavg = 0; // average speed buffer

    // ==============================
    // Configuration & Settings
    // ==============================
    uint8_t obd_sel = 1;      // 1 = OBD1, 2 = OBD2
    uint8_t pag_select = 1;   // LCD display page
    uint8_t ect_alarm = 98;   // coolant temp alarm (°C)
    uint8_t vss_alarm = 100;  // speed alarm (km/h)
    uint8_t th_threshold = 4; // thermistor threshold (°C)
};