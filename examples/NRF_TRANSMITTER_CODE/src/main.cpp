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
uint8_t seqCounter = 0;

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("TX boot");

  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CSN_PIN);

  radio.begin();
  Serial.println(radio.isChipConnected() ? "nRF24 CONNECTED" : "nRF24 NOT CONNECTED");
  if (!radio.isChipConnected()) while (true) delay(1000);

  // Force identical settings (match RX)
  radio.setAddressWidth(5);
  radio.setChannel(100);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setCRCLength(RF24_CRC_16);

  radio.disableDynamicPayloads();
  radio.setPayloadSize(sizeof(ControlPacket));

  // No-ACK debug mode
  radio.setAutoAck(true);
  radio.setRetries(5, 15);

  radio.openWritingPipe(address);
  radio.stopListening();

  Serial.println("TX ready");
  radio.printDetails();
}

void loop() {
  pkt.throttle = 1011;
  pkt.yaw      = 1013;
  pkt.pitch    = 1014;
  pkt.roll     = 918;
  pkt.flags    = 0;
  pkt.seq      = seqCounter++;

  bool ok = radio.write(&pkt, sizeof(pkt));
  Serial.printf("TX %s SEQ=%u\n", ok ? "OK" : "FAIL", pkt.seq);

  delay(20);
}
