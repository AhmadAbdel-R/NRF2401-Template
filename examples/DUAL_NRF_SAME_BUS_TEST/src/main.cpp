#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <string.h>

//-- PIN ASSIGNMENTS
#define MOSI_PIN 6
#define MISO_PIN 5
#define SCK_PIN 4
#define CE_PIN_TX 3
#define CSN_PIN_TX 7
#define CE_PIN_RX 20
#define CSN_PIN_RX 21
// end of pin assignments

//-- NRF OBJECTS AND ADDRESS
RF24 radioTx(CE_PIN_TX, CSN_PIN_TX);
RF24 radioRx(CE_PIN_RX, CSN_PIN_RX);
const byte address[6] = "DRONE";
// end of nrf objects and address

//-- CONTROL PACKET FORMAT
#pragma pack(push, 1)
struct ControlPacket
{
  uint16_t magic;
  uint16_t throttle;
  uint16_t yaw;
  uint16_t pitch;
  uint16_t roll;
  uint8_t flags;
  uint8_t seq;
  uint16_t checksum;
};
#pragma pack(pop)
static_assert(sizeof(ControlPacket) <= 32, "nRF24 max payload is 32 bytes");
// end of control packet format

//-- TX HISTORY (INDEXED BY SEQ FOR RX MATCHING)
struct TxSnapshot
{
  bool valid;
  ControlPacket pkt;
  uint32_t packetNumber;
};
TxSnapshot txBySeq[256];
// end of tx history

//-- RUNTIME STATE
ControlPacket txPkt{};
ControlPacket rxPkt{};
uint8_t seqCounter = 0;
uint32_t txPacketCount = 0;
uint32_t rxPacketCount = 0;
uint32_t txAttempts = 0;
uint32_t txOk = 0;
uint32_t txFail = 0;
uint32_t rxMatch = 0;
uint32_t rxMismatch = 0;
bool txTestComplete = false;
uint32_t txCompleteMs = 0;
bool finalReportPrinted = false;
// end of runtime state

//-- TIMING AND LOGGING CONFIG
const uint32_t CTRL_PERIOD_MS = 20;
const uint32_t SUMMARY_PERIOD_MS = 1000;
const uint32_t TARGET_TX_PACKETS = 1000000UL;
const uint32_t POST_TX_DRAIN_MS = 5000;
const bool PRINT_PACKET_TABLE = false;
const uint8_t PRINT_EVERY_N_RX = 5;
uint32_t lastCtrlMs = 0;
uint32_t lastSummaryMs = 0;
// end of timing and logging config

//-- CHECKSUM HELPER
static uint16_t calcChecksum(const ControlPacket &p)
{
  uint16_t s = 0;
  s ^= p.magic;
  s ^= p.throttle;
  s ^= p.yaw;
  s ^= p.pitch;
  s ^= p.roll;
  s ^= (uint16_t)p.flags;
  s ^= (uint16_t)p.seq;
  return s;
}
// end of checksum helper

//-- NRF CONFIG HELPER
static void configureRadio(RF24 &r)
{
  r.setAddressWidth(5);
  r.setChannel(100);
  r.setDataRate(RF24_2MBPS);
  r.setCRCLength(RF24_CRC_16);
  r.setPALevel(RF24_PA_MAX);
  r.setAutoAck(false);
  r.setRetries(0, 0);
  r.disableDynamicPayloads();
  r.setPayloadSize(sizeof(ControlPacket));
}
// end of nrf config helper

//-- PACKET COMPARE HELPER
static bool packetsMatch(const ControlPacket &tx, const ControlPacket &rx)
{
  return tx.magic == rx.magic &&
         tx.throttle == rx.throttle &&
         tx.yaw == rx.yaw &&
         tx.pitch == rx.pitch &&
         tx.roll == rx.roll &&
         tx.flags == rx.flags &&
         tx.seq == rx.seq &&
         tx.checksum == rx.checksum;
}
// end of packet compare helper

//-- DEBUG TABLE HEADER
static void printDebugHeader()
{
  Serial.println();
  Serial.println("RX#   TX#   TX[seq thr  yaw  pit  rol  flg]    RX[seq thr  yaw  pit  rol  flg]    magic ck  match");
  Serial.println("------------------------------------------------------------------------------------------------------");
}
// end of debug table header

//-- DEBUG TABLE ROW
static void printDebugRow(uint32_t rxNumber,
                          const TxSnapshot *txRef,
                          const ControlPacket &rx,
                          bool magicOk,
                          bool checksumOk,
                          bool match)
{
  if (txRef != nullptr)
  {
    Serial.printf("%-5lu %-5lu [%3u %4u %4u %4u %4u %3u]    [%3u %4u %4u %4u %4u %3u]    %-5s %-3s %-5s\n",
                  (unsigned long)rxNumber,
                  (unsigned long)txRef->packetNumber,
                  txRef->pkt.seq,
                  txRef->pkt.throttle,
                  txRef->pkt.yaw,
                  txRef->pkt.pitch,
                  txRef->pkt.roll,
                  txRef->pkt.flags,
                  rx.seq,
                  rx.throttle,
                  rx.yaw,
                  rx.pitch,
                  rx.roll,
                  rx.flags,
                  magicOk ? "OK" : "BAD",
                  checksumOk ? "OK" : "BAD",
                  match ? "YES" : "NO");
  }
  else
  {
    Serial.printf("%-5lu %-5s [ -- ---- ---- ---- ---- ---]    [%3u %4u %4u %4u %4u %3u]    %-5s %-3s %-5s\n",
                  (unsigned long)rxNumber,
                  "NA",
                  rx.seq,
                  rx.throttle,
                  rx.yaw,
                  rx.pitch,
                  rx.roll,
                  rx.flags,
                  magicOk ? "OK" : "BAD",
                  checksumOk ? "OK" : "BAD",
                  "NO");
  }
}
// end of debug table row

void setup()
{
  //-- SERIAL INIT
  Serial.begin(115200);
  delay(1500);
  Serial.println("\nBoot: TX nRF -> RX nRF 1M packet test");
  Serial.printf("ControlPacket size: %u bytes\n", (unsigned)sizeof(ControlPacket));
  // end of serial init

  //-- GPIO DEFAULT LEVELS
  pinMode(CSN_PIN_TX, OUTPUT);
  pinMode(CSN_PIN_RX, OUTPUT);
  digitalWrite(CSN_PIN_TX, HIGH);
  digitalWrite(CSN_PIN_RX, HIGH);
  pinMode(CE_PIN_TX, OUTPUT);
  pinMode(CE_PIN_RX, OUTPUT);
  digitalWrite(CE_PIN_TX, LOW);
  digitalWrite(CE_PIN_RX, LOW);
  delay(10);
  // end of gpio default levels

  //-- SPI INIT
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN);
  // end of spi init

  //-- NRF TX INIT
  radioTx.begin();
  Serial.println(radioTx.isChipConnected() ? "nRF24_TX CONNECTED" : "nRF24_TX NOT CONNECTED");
  if (!radioTx.isChipConnected())
  {
    while (true)
    {
      delay(1000);
    }
  }
  configureRadio(radioTx);
  radioTx.openWritingPipe(address);
  radioTx.stopListening();
  Serial.println("TX ready");
  // end of nrf tx init

  //-- NRF RX INIT
  radioRx.begin();
  Serial.println(radioRx.isChipConnected() ? "nRF24_RX CONNECTED" : "nRF24_RX NOT CONNECTED");
  if (!radioRx.isChipConnected())
  {
    while (true)
    {
      delay(1000);
    }
  }
  configureRadio(radioRx);
  radioRx.openReadingPipe(1, address);
  radioRx.startListening();
  radioRx.flush_rx();
  Serial.println("RX ready");
  // end of nrf rx init

  //-- DEBUG TABLE INIT
  memset(txBySeq, 0, sizeof(txBySeq));
  Serial.printf("TEST target: %lu TX packets\n", (unsigned long)TARGET_TX_PACKETS);
  Serial.printf("TX period: %lu ms\n", (unsigned long)CTRL_PERIOD_MS);
  if (PRINT_PACKET_TABLE)
  {
    printDebugHeader();
  }
  // end of debug table init
}

void loop()
{
  const uint32_t now = millis();

  //-- TX PACKET BUILD + SEND
  if (!txTestComplete && txPacketCount < TARGET_TX_PACKETS && (now - lastCtrlMs >= CTRL_PERIOD_MS))
  {
    lastCtrlMs = now;

    txPkt.magic = 0xBEEF;
    txPkt.throttle = 1011;
    txPkt.yaw = 1013;
    txPkt.pitch = 1014;
    txPkt.roll = 918;
    txPkt.flags = 0;
    txPkt.seq = seqCounter++;
    txPkt.checksum = calcChecksum(txPkt);

    txAttempts++;
    const bool ok = radioTx.write(&txPkt, sizeof(txPkt));
    if (ok)
    {
      txOk++;
      txPacketCount++;
      txBySeq[txPkt.seq].valid = true;
      txBySeq[txPkt.seq].pkt = txPkt;
      txBySeq[txPkt.seq].packetNumber = txPacketCount;
    }
    else
    {
      txFail++;
    }

    if (txPacketCount >= TARGET_TX_PACKETS)
    {
      txTestComplete = true;
      txCompleteMs = now;
      Serial.printf("TX COMPLETE: sent %lu packets, draining RX for %lu ms...\n",
                    (unsigned long)txPacketCount,
                    (unsigned long)POST_TX_DRAIN_MS);
    }
  }
  // end of tx packet build + send

  //-- RX READ + TX/RX MATCH TABLE
  while (radioRx.available())
  {
    memset(&rxPkt, 0xA5, sizeof(rxPkt));
    radioRx.read(&rxPkt, sizeof(rxPkt));
    rxPacketCount++;

    const bool magicOk = (rxPkt.magic == 0xBEEF);
    const bool checksumOk = (rxPkt.checksum == calcChecksum(rxPkt));

    TxSnapshot *entry = &txBySeq[rxPkt.seq];
    TxSnapshot *txRef = entry->valid ? entry : nullptr;
    const bool payloadMatch = (txRef != nullptr) && packetsMatch(txRef->pkt, rxPkt);
    const bool fullMatch = magicOk && checksumOk && payloadMatch;

    if (fullMatch)
    {
      rxMatch++;
    }
    else
    {
      rxMismatch++;
    }

    const bool shouldPrint = PRINT_PACKET_TABLE && ((rxPacketCount % PRINT_EVERY_N_RX == 0) || !fullMatch);
    if (shouldPrint)
    {
      printDebugRow(rxPacketCount, txRef, rxPkt, magicOk, checksumOk, fullMatch);
    }
  }
  // end of rx read + tx/rx match table

  //-- ONCE PER SECOND SUMMARY
  if (now - lastSummaryMs >= SUMMARY_PERIOD_MS)
  {
    lastSummaryMs = now;
    const uint32_t txProgressX100 = (uint32_t)(((uint64_t)txPacketCount * 10000ULL) / TARGET_TX_PACKETS);
    Serial.printf("PROGRESS tx=%lu/%lu (%lu.%02lu%%) attempts=%lu fail=%lu rx=%lu match=%lu mismatch=%lu state=%s\n",
                  (unsigned long)txPacketCount,
                  (unsigned long)TARGET_TX_PACKETS,
                  (unsigned long)(txProgressX100 / 100),
                  (unsigned long)(txProgressX100 % 100),
                  (unsigned long)txAttempts,
                  (unsigned long)txFail,
                  (unsigned long)rxPacketCount,
                  (unsigned long)rxMatch,
                  (unsigned long)rxMismatch,
                  txTestComplete ? "DRAINING" : "RUNNING");
  }
  // end of once per second summary

  //-- FINAL REPORT + HALT
  if (txTestComplete && !finalReportPrinted && (now - txCompleteMs >= POST_TX_DRAIN_MS))
  {
    finalReportPrinted = true;

    const uint32_t deliveryX100 = (uint32_t)(((uint64_t)rxPacketCount * 10000ULL) / TARGET_TX_PACKETS);
    const uint32_t missing = (rxPacketCount < txPacketCount) ? (txPacketCount - rxPacketCount) : 0;

    Serial.println("==== FINAL TEST REPORT ====");
    Serial.printf("tx_target=%lu tx_sent=%lu tx_attempts=%lu tx_fail=%lu\n",
                  (unsigned long)TARGET_TX_PACKETS,
                  (unsigned long)txPacketCount,
                  (unsigned long)txAttempts,
                  (unsigned long)txFail);
    Serial.printf("rx_total=%lu rx_match=%lu rx_mismatch=%lu\n",
                  (unsigned long)rxPacketCount,
                  (unsigned long)rxMatch,
                  (unsigned long)rxMismatch);
    Serial.printf("delivery_vs_target=%lu.%02lu%% missing=%lu packets\n",
                  (unsigned long)(deliveryX100 / 100),
                  (unsigned long)(deliveryX100 % 100),
                  (unsigned long)missing);
    Serial.println("TEST DONE - MCU HALTED");
    Serial.flush();
  }

  if (finalReportPrinted)
  {
    while (true)
    {
      delay(1000);
    }
  }
  // end of final report + halt
}
