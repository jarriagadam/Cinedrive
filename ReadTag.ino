
#if 0
#include <SPI.h>
#include <PN532_SPI.h>
#include <PN532.h>
#include <NfcAdapter.h>

PN532_SPI pn532spi(SPI, 10);
NfcAdapter nfc = NfcAdapter(pn532spi);
#else

#include <Wire.h>
#include <PN532_I2C.h>
#include <PN532.h>
#include <NfcAdapter.h>

PN532_I2C pn532_i2c(Wire);
NfcAdapter nfc = NfcAdapter(pn532_i2c);
#endif

String RFID1 = "9A 1E 25 28";
String RFID2 = "3F 90 1E 29";
String RFID3 = "6A E7 23 D9";
String RFID4 = "24 FD 15 41";
String RFID5 = "F2 75 73 85"; //Tarjeta Bip

void setup(void) {
    Serial.begin(9600);
    Serial.println("NDEF Reader");
    nfc.begin();
}

void loop(void) {
    Serial.println("\nScan a NFC tag\n");
    if (nfc.tagPresent())
    {
        NfcTag tag = nfc.read();
        String uid = tag.getUidString();
        String RFID_value;
        if (uid == RFID1) {
          RFID_value = "1";
        } else if (uid == RFID2) {
          RFID_value = "2";
        } else if (uid == RFID3) {
          RFID_value = "3";
        } else if (uid == RFID4) {
          RFID_value = "4";
        } else if (uid == RFID5) {
          RFID_value = "5 ES UNA TARJETA BIP";
        } else {
          RFID_value = "unknown";
        }
        Serial.print("RFID Key Scanned: " + RFID_value + '\n');
        Serial.print(uid);
    }
    delay(1000);
}
