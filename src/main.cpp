#include "hobd_uni2.hpp" // your ECUData + offsets + sendcmd + readLiveData + scanDtc

SoftwareSerialWithHalfDuplex dlcSerial(12, 12, false, false);

// ECU logic wrapper
ECUData ecu(1 /*obd_sel*/, dlcSerial);

enum MsgType : uint8_t
{
  MSG_LIVE = 0x81,
  MSG_DTC = 0x82,
  MSG_ACK = 0x83,
  MSG_ERR = 0x84
};

enum Cmd : uint8_t
{
  CMD_GET_LIVE = 0x01,
  CMD_GET_DTC = 0x02,
  CMD_RESET = 0x03
};

// ---- Protocol ----
static const uint8_t SOF1 = 0xAA;
static const uint8_t SOF2 = 0x55;

static void sendFrame(Stream &out, uint8_t type, const uint8_t *payload, uint8_t len)
{
  uint8_t header[4] = {SOF1, SOF2, type, len};
  uint8_t crc = 0;
  crc += ECUData::mkcrc(header, 4);
  crc += ECUData::mkcrc(payload, len);

  out.write(header, 4);
  if (len)
    out.write(payload, len);
  out.write((uint8_t)(crc & 0xFF));
}

static bool readCmdFrame(Stream &in, uint8_t &cmdOut)
{
  // Very simple: expects 1 byte command only.
  // You can upgrade later to full framed commands if needed.
  if (!in.available())
    return false;
  cmdOut = (uint8_t)in.read();
  return true;
}

// Pack floats as int16/int32 to keep payload small and easy.
// Scale choices are yours; these are sensible defaults.
static void pack_live(uint8_t *p, const ECUData &e)
{
  // layout:
  // rpm(u16), vss(u8), ect(i16*10), iat(i16*10), map(i16*10), tps(i16*10),
  // batt(u16*100), o2(u16*1000), flags(u8)
  uint16_t rpm = (uint16_t)max(0.0f, e.rpm);
  p[0] = rpm >> 8;
  p[1] = rpm & 0xFF;
  p[2] = e.vss;

  int16_t ect = (int16_t)(e.ect * 10.0f);
  int16_t iat = (int16_t)(e.iat * 10.0f);
  int16_t map = (int16_t)(e.maps * 10.0f);
  int16_t tps = (int16_t)(e.tps * 10.0f);

  p[3] = ect >> 8;
  p[4] = ect & 0xFF;
  p[5] = iat >> 8;
  p[6] = iat & 0xFF;
  p[7] = map >> 8;
  p[8] = map & 0xFF;
  p[9] = tps >> 8;
  p[10] = tps & 0xFF;

  uint16_t batt = (uint16_t)(e.volt * 100.0f);
  p[11] = batt >> 8;
  p[12] = batt & 0xFF;

  uint16_t o2mv = (uint16_t)(e.o2 * 1000.0f);
  p[13] = o2mv >> 8;
  p[14] = o2mv & 0xFF;

  uint8_t flags = 0;
  flags |= e.sw_aircon ? (1 << 0) : 0;
  flags |= e.sw_brake ? (1 << 1) : 0;
  flags |= e.sw_vtec ? (1 << 2) : 0;
  flags |= e.cel ? (1 << 3) : 0;
  p[15] = flags;
}

void loop()
{
  // Choose which link you want to listen on:
  // 1) USB Serial
  Stream &link = Serial;

  // If you want HC-05 instead, swap to:
  // Stream &link = btSerial;

  uint8_t cmd;
  if (!readCmdFrame(link, cmd))
    return;

  if (cmd == CMD_GET_LIVE)
  {
    if (!ecu.readLiveData())
    {
      const uint8_t err = 1;
      sendFrame(link, MSG_ERR, &err, 1);
      return;
    }
    uint8_t payload[16];
    pack_live(payload, ecu);
    sendFrame(link, MSG_LIVE, payload, sizeof(payload));
  }
  else if (cmd == CMD_GET_DTC)
  {
    if (!ecu.scanDtc())
    {
      const uint8_t err = 2;
      sendFrame(link, MSG_ERR, &err, 1);
      return;
    }
    // payload: [count, dtc0, dtc1, ...]
    uint8_t payload[1 + ErrLen];
    payload[0] = (uint8_t)ecu.dtcLen;
    for (uint8_t i = 0; i < payload[0] && i < ErrLen; ++i)
      payload[1 + i] = ecu.dtcErrs[i];
    sendFrame(link, MSG_DTC, payload, (uint8_t)(1 + payload[0]));
  }
  else if (cmd == CMD_RESET)
  {
    bool ok = ecu.resetEcu();
    uint8_t payload[1] = {ok ? 1 : 0};
    sendFrame(link, MSG_ACK, payload, 1);
  }
  else
  {
    const uint8_t err = 0xFF;
    sendFrame(link, MSG_ERR, &err, 1);
  }
}

void setup()
{
  // pin 12 for 1 wire
  Serial.begin(115200);
  dlcSerial.begin(9600);
  ecu.init();
  delay(1000);
}