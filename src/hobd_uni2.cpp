#include "hobd_uni2.hpp"

const uint8_t startup[] = {0x68, 0x6a, 0xf5, 0xaf, 0xbf, 0xb3, 0xb2, 0xc1, 0xdb, 0xb3, 0xe9};

bool ECUData::init(){
    // uint8_t n = sizeof(startup)/sizeof(startup[0]);
    // for (uint8_t i = 0; i < n; i++){
    //     dlc.write(startup[i]);
    // }
    dlc.write(0x68);
    dlc.write(0x6a);
    dlc.write(0xf5);
    dlc.write(0xaf);
    dlc.write(0xbf);
    dlc.write(0xb3);
    dlc.write(0xb2);
    dlc.write(0xc1);
    dlc.write(0xdb);
    dlc.write(0xb3);
    dlc.write(0xe9);
    delay(300);
    return true;
}

uint8_t ECUData::mkcrc(const uint8_t *buf, uint8_t len){
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++){
        crc = crc + buf[i];
    }
    return (uint8_t) (0xFF - (crc - 1));
}

bool ECUData::sendcmd(EcuCmd &ecmd){
    // Build TX frame: [cmd, txlen, reg, rxlen, crc]

    memset(dlcData, 0, sizeof(dlcData));

    uint8_t tx[5];
    tx[0] = ecmd.cmd;
    tx[1] = ecmd.txlen;
    tx[2] = ecmd.reg;
    tx[3] = ecmd.rxlen;
    // tx[4] = mkcrc(tx, 4); // checksum over first 4 bytes
    uint8_t crc = (0xFF - (ecmd.cmd + ecmd.txlen + ecmd.reg + ecmd.rxlen - 0x01));
    tx[4] = crc;
    ecmd.crc = tx[4];

    // TX
    dlc.listen();
    memset(dlcData, 0, sizeof(dlcData));

    // dlc.listen();
    // dlc.write(ecmd.cmd); // header/cmd read memory ??
    // dlc.write(ecmd.txlen); // num of bytes to send
    // dlc.write(ecmd.reg); // address
    // dlc.write(ecmd.rxlen); // num of bytes to read
    // dlc.write(crc); // checksum

    // Serial.print("Sending Command");
    // Serial.println(ecmd.reg);
    // reply: 00 len+3 data...
    // int i = 0;
    // while (i < (ecmd.rxlen + 3))
    // {
    //     if (dlc.available())
    //     {
    //         dlcData[i] = dlc.read();
    //         // Serial.print("rx data");
    //         // Serial.println(dlcData[i]);
    //         // if (dlcdata[i] != 0x00 && dlcdata[i+1] != (len+3)) continue; // ignore ?
    //         i++;
    //     }
    // }
    // Serial.print("Received data of len ");
    // Serial.println(i);
    // Serial.print("expected data of len ");
    // Serial.println(ecmd.rxlen);

    for (uint8_t i = 0; i < 5; ++i) dlc.write(tx[i]);

    const uint16_t expected = (uint16_t)ecmd.rxlen + (uint16_t)MSG_OFFSET;

    const uint32_t tStart = millis();
    const uint32_t timeoutMs = 200;

    uint16_t i = 0;
    while (i < expected/*&& (millis() - tStart) < timeoutMs*/)
    {
        if (dlc.available())
        {
            dlcData[i++] = (uint8_t)dlc.read();
        }
    }

    if (i < expected)
    {
        dlctmo++;
        Errs[errLen++] = TimeoutErr;
        return false;
    }

    // crc = 0;
    // for (uint8_t j = 0; j < i + 2; j++)
    // {
    //     crc = crc + dlcData[j];
    // }
    // crc = 0xFF - (crc - 1);

    // Verify checksum: last byte is checksum for entire response minus itself
    // e.g. checksum256(resp, resp_len_without_checksum) should equal last byte.
    const uint8_t rxCrc = dlcData[expected - 1];
    const uint8_t calc = mkcrc(dlcData, expected - 1);

    if (calc != rxCrc)
    {
        Errs[errLen++] = ChecksumErr;
        return false;
    }

    return true;
}

bool ECUData::scanDtc(){
    dtcLen = 0;

    EcuCmd cmd{};
    cmd.cmd = HOBD_CMD;
    cmd.txlen = 0x05;
    cmd.reg = HOBD_OFF_ERRORS1;
    cmd.rxlen = 0x10;

    if (!sendcmd(cmd))
        return false;

    const uint8_t maxCodes = sizeof(dtcErrs) / sizeof(dtcErrs[0]);
    uint8_t i;
    // Serial.print("dtc things");
    for (i = 0; i < 14; i++)
    {
        if (dlcData[i + 2] >> 4)
        {
            dtcErrs[i] = i * 2;
            dtcLen++;
        }
        if (dlcData[i + 2] & 0xf)
        {
            // haxx
            // if (errnum == 23) errnum = 22;
            // if (errnum == 24) errnum = 23;
            dtcErrs[i] = (i * 2) + 1;
            // haxx
            if (dtcErrs[i] == 23)
                dtcErrs[i] = 22;
            if (dtcErrs[i] == 24)
                dtcErrs[i] = 23;
            dtcLen++;
        }
    }
    // for (uint8_t i = 0; i < ErrLen; ++i)
    // {
    //     uint8_t b = dlcData[(MSG_OFFSET - 1) + i];

    //     if (b >> 4){
    //         dtcErrs[dtcLen++] = i * 2;
    //     }
    //     if (b & 0x0F){
    //         uint8_t errN = i * 2 + 1;
    //         if (errN == 23 || errN == 24)
    //             errN--;
    //         dtcErrs[dtcLen++] = errN;
    //     }
    // }
    // Serial.println(dtcLen);
    // Serial.print("");

    return true;
}

// Reset ECU
bool ECUData::resetEcu()
{
    EcuCmd cmd;
    cmd.cmd = HOBD_RST;
    cmd.txlen = 0x04;
    cmd.reg = 0x01;
    cmd.rxlen = 0x0;
    if (!sendcmd(cmd)) return false;
    return true;
}

static uint16_t rx_u16(uint8_t *buffer, uint8_t offset){
    return (uint16_t)(buffer[offset] << 8 | buffer[offset + 1]);
}

static float cnvt_tmp(float f){
    return 55.04149 - f * 3.0414878 + pow(f, 2) * 0.03952185 - pow(f, 3) * 0.00029383913 + pow(f, 4) * 0.0000010792568 - pow(f, 5) * 0.0000000015618437;
}

bool ECUData::readLiveData()
{
    float f;
    // -------- Row 1: reg 0x00, len 0x10
    {
        EcuCmd cmd{};
        cmd.cmd = HOBD_CMD;
        cmd.txlen = 0x05;
        cmd.reg = HOBD_OFF_RPM;
        cmd.rxlen = 0x10;

        if (!sendcmd(cmd))
            return false;

        // RPM (WORD at payload offset 0x00)
        uint16_t rawRpm = rx_u16(dlcData, RPL_OFFSET + HOBD_OFF_RPM);

        if (obd_sel == 1)
        {
            // Your original OBD1 formula
            rpm = 1875000.0f / ((float)rawRpm + 1.0f);
        }

        if (rpm < 0)
            rpm = 0;

        // VSS (BYTE at offset 0x02)
        vss = dlcData[RPL_OFFSET + HOBD_OFF_VSS];

        // Flags
        const uint8_t f08 = dlcData[RPL_OFFSET + HOBD_OFF_FLAG_08];
        const uint8_t f0B = dlcData[RPL_OFFSET + HOBD_OFF_FLAG_0B];

        sw_aircon = (f08 & HOBD_FLG_AC_SWITCH) != 0;
        sw_brake = (f08 & HOBD_FLG_BRAKE) != 0;
        sw_starter = (f08 & HOBD_FLG_STARTER) != 0;
        sw_vtec = (f08 & HOBD_FLG_VTEC_PRESS) != 0;

        main_relay = (f0B & HOBD_FLG_MAIN_RELAY) != 0;
        cel = (f0B & HOBD_FLG_CEL) != 0;
    }

    delay(1);

    // -------- Row 2: reg 0x10, len 0x10
    {
        EcuCmd cmd{};
        cmd.cmd = HOBD_CMD;
        cmd.txlen = 0x05;
        cmd.reg = HOBD_OFF_ECT;
        cmd.rxlen = 0x10;

        if (!sendcmd(cmd))
            return false;

        // Temps (BYTE at 0x10 CTS and 0x11 IAT)
        ect = cnvt_tmp(dlcData[RPL_OFFSET]);
        iat = cnvt_tmp(dlcData[RPL_OFFSET + HOBD_OFF_IAT - HOBD_OFF_ECT]);

        auto conv_kpa = [](uint8_t v) -> float
        {
            return v * 0.716f - 5.0f;
        };
        // MAP & BARO
        maps = conv_kpa(dlcData[RPL_OFFSET + HOBD_OFF_MAP - HOBD_OFF_ECT]);
        baro = conv_kpa(dlcData[RPL_OFFSET + HOBD_OFF_MAP + 1 - HOBD_OFF_ECT]);

        // TPS
        tps = (dlcData[RPL_OFFSET + HOBD_OFF_TPS - HOBD_OFF_ECT] - 24) / 2;

        // O2 voltage
        f = dlcData[RPL_OFFSET + HOBD_OFF_O2 - HOBD_OFF_ECT];
        o2 = f / 51.3f;

        // Battery voltage
        f = dlcData[RPL_OFFSET + HOBD_OFF_VOLT - HOBD_OFF_ECT];
        volt =  f / 10.45f;

        // Alternator raw byte (0x18)
        f = dlcData[RPL_OFFSET + HOBD_OFF_ALTF - HOBD_OFF_ECT];
        alt_fr = f / 2.55;

        f = dlcData[RPL_OFFSET + HOBD_OFF_EL - HOBD_OFF_ECT];
        eld = 77.06 - f / 2.5371;
    }

    delay(1);

    // -------- Row 3: reg 0x20, len 0x10
    {
        EcuCmd cmd{};
        cmd.cmd = HOBD_CMD;
        cmd.txlen = 0x05;
        cmd.reg = 0x20;
        cmd.rxlen = 0x10;

        if (!sendcmd(cmd))
            return false;

        // These are row3 payload indices, not global offsets (unless you have defines).
        // Keeping your original positions relative to row3:
        const uint8_t sft_raw = dlcData[RPL_OFFSET];
        const uint8_t lft_raw = dlcData[RPL_OFFSET + 1];

        sft = ((float)sft_raw / 128.0f - 1.0f) * 100.0f;
        lft = ((float)lft_raw / 128.0f - 1.0f) * 100.0f;

        // Inj (WORD)
        uint16_t injRaw = rx_u16(dlcData, RPL_OFFSET + 4);
        inj = ((float)injRaw) / 250.0f;

        // Ign and LMT
        f = dlcData[8];
        ign = (f - 24) / 4; // (degrees)
        f = dlcData[9];
        lmt = (f - 24) / 4;

        // IACV (%)
        iacv = dlcData[10] / 2.55f;
    }

    delay(1);

    // -------- Row 4: reg 0x30, len 0x10
    {
        EcuCmd cmd{};
        cmd.cmd = HOBD_CMD;
        cmd.txlen = 0x05;
        cmd.reg = 0x30;
        cmd.rxlen = 0x10;

        if (!sendcmd(cmd))
            return false;

        knoc = dlcData[14] / 51; // 0 to 5
    }

    int imap = rpm * maps / (iat + 273) / 2;
    maf = (imap / 60) * (80 / 100) * 1.595 * 28.9644 / 8.314472;

    return true;
}
