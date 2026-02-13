#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

#define MOSI_PIN 6
#define MISO_PIN 5
#define SCK_PIN  4

#define CE_PIN   3      // IMPORTANT: must match your wiring
#define CSN_PIN  7

RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "DRONE";

struct ControlPacket {
  uint16_t throttle, yaw, pitch, roll;
  uint8_t  flags, seq;
};

ControlPacket pkt;
unsigned long hb = 0;

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("RX boot");

  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CSN_PIN);

  radio.begin();
  Serial.println(radio.isChipConnected() ? "nRF24 CONNECTED" : "nRF24 NOT CONNECTED");
  if (!radio.isChipConnected()) while (true) delay(1000);

  // Force identical settings (match TX)
  radio.setAddressWidth(5);
  radio.setChannel(100);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setCRCLength(RF24_CRC_16);

  radio.disableDynamicPayloads();
  radio.setPayloadSize(sizeof(ControlPacket));

  // No-ACK debug mode
  radio.setAutoAck(true);

  radio.openReadingPipe(1, address);
  radio.startListening();

  Serial.println("RX ready");
  radio.printDetails();   // prints config to Serial
}

void loop() {
  if (millis() - hb > 1000) {
    hb = millis();
    Serial.println("waiting...");
  }

  while (radio.available()) {
    radio.read(&pkt, sizeof(pkt));
    Serial.printf("RX SEQ=%u THR=%u YAW=%u PIT=%u ROL=%u FLAGS=%u\n",
                  pkt.seq, pkt.throttle, pkt.yaw, pkt.pitch, pkt.roll, pkt.flags);
  }
}
