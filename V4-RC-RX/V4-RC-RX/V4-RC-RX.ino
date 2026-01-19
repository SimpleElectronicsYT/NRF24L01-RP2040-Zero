/*
 * ======================================================================================
 * PROJECT: RP2040 NRF24L01 RECEIVER (RX)
 * ======================================================================================
 * * DESCRIPTION:
 * Listens for wireless packets containing 3 byte values.
 * Maps those values to 3 Servos (0-180 degrees).
 * * KEY FEATURES:
 * 1. Polling Method: Checks the radio constantly for "Data Ready" signal.
 * 2. Instant Response: No delays in the loop means servos move instantly.
 * * WIRING:
 * - GP0 -> Servo 1 Signal
 * - GP1 -> Servo 2 Signal
 * - GP2 -> Servo 3 Signal
 * - GP4 -> NRF24L01 CSN
 * - GP3 -> NRF24L01 CE
 * - GP7 -> NRF24L01 SCK
 * - GP5 -> NRF24L01 MOSI
 * - GP6 -> NRF24L01 MISO
 */

#include <Arduino.h>
#include <Servo.h>

// --- PIN ASSIGNMENTS ---
const int CSN_PIN  = 4;
const int CE_PIN   = 3;
const int SCK_PIN  = 7;
const int MOSI_PIN = 5;
const int MISO_PIN = 6;

// Create Servo Objects
Servo s1, s2, s3;

// Address must match Transmitter exactly
byte address[5] = {0xAB, 0xCD, 0x12, 0x34, 0x56};

// ======================================================================================
// SECTION: BIT-BANG SPI FUNCTIONS
// Identical to TX side. Manually toggles pins for radio communication.
// ======================================================================================
byte bitbangSPI(byte data) {
  byte incoming = 0;
  for (int i = 7; i >= 0; i--) {
    digitalWrite(MOSI_PIN, (data >> i) & 0x01);
    delayMicroseconds(2);
    digitalWrite(SCK_PIN, HIGH);
    delayMicroseconds(2);
    if (digitalRead(MISO_PIN)) incoming |= (1 << i);
    digitalWrite(SCK_PIN, LOW);
    delayMicroseconds(2);
  }
  return incoming;
}

void nrfWriteReg(byte reg, byte value) {
  digitalWrite(CSN_PIN, LOW);
  bitbangSPI(0x20 | reg);
  bitbangSPI(value);
  digitalWrite(CSN_PIN, HIGH);
}

// Reads a value from a register
byte nrfReadReg(byte reg) {
  digitalWrite(CSN_PIN, LOW);
  bitbangSPI(0x00 | reg);
  byte val = bitbangSPI(0xFF); // Send dummy byte to read result
  digitalWrite(CSN_PIN, HIGH);
  return val;
}

// ======================================================================================
// SECTION: SETUP
// ======================================================================================
void setup() {
  Serial.begin(115200);
  
  // Wait a max of 3 seconds for serial monitor to open, then proceed anyway
  long startWait = millis();
  while (!Serial && (millis() - startWait < 3000));

  // Attach Servos to Pins
  s1.attach(0); 
  s2.attach(1); 
  s3.attach(2);
  
  // Setup Radio Pins
  pinMode(CE_PIN, OUTPUT);
  pinMode(CSN_PIN, OUTPUT);
  pinMode(SCK_PIN, OUTPUT);
  pinMode(MOSI_PIN, OUTPUT);
  pinMode(MISO_PIN, INPUT);

  digitalWrite(CSN_PIN, HIGH);
  digitalWrite(CE_PIN, LOW);

  Serial.println("--- RX STARTING ---");

  // --- RADIO CONFIGURATION ---
  // 0x00 CONFIG: Power Up, Enable CRC, RX (Receiver) Mode
  nrfWriteReg(0x00, 0x0F); 
  delay(5); 

  // 0x03 SETUP_AW: 5 Byte Address
  nrfWriteReg(0x03, 0x03); 

  // Set the Receive Address on Pipe 0
  digitalWrite(CSN_PIN, LOW);
  bitbangSPI(0x20 | 0x0A); // RX_ADDR_P0
  for(int i=0; i<5; i++) bitbangSPI(address[i]);
  digitalWrite(CSN_PIN, HIGH);

  // Payload Size (How many bytes are we expecting per packet?)
  // We are sending 3 bytes (Pot1, Pot2, Pot3)
  nrfWriteReg(0x11, 0x03); 

  // Tuning (Must match TX)
  nrfWriteReg(0x01, 0x3F); // Enable Auto-Ack
  nrfWriteReg(0x05, 0x4C); // Channel 76
  nrfWriteReg(0x06, 0x07); // 1Mbps, Max Power

  // Clear any existing interrupt flags
  nrfWriteReg(0x07, 0x70);
  
  // Set CE HIGH to start listening endlessly
  digitalWrite(CE_PIN, HIGH);
  Serial.println("Listening for TX...");
}

// ======================================================================================
// SECTION: MAIN LOOP
// ======================================================================================
void loop() {
  // 1. Check Radio Status
  // We read the STATUS register (0x07) to see if new data arrived.
  byte status = nrfReadReg(0x07);

  // 2. Check Bit 6 (RX_DR = Data Ready)
  // if (status & 0x40) means "Is the 6th bit a 1?"
  if (status & 0x40) { 
    
    // 3. Read the Data
    digitalWrite(CSN_PIN, LOW);
    bitbangSPI(0x61); // Command: Read RX Payload
    byte p1 = bitbangSPI(0xFF); // Read Byte 1
    byte p2 = bitbangSPI(0xFF); // Read Byte 2
    byte p3 = bitbangSPI(0xFF); // Read Byte 3
    digitalWrite(CSN_PIN, HIGH);

    // 4. Move Servos
    // Map the 0-255 radio byte to 0-180 servo degrees
    s1.write(map(p1, 0, 255, 0, 180));
    s2.write(map(p2, 0, 255, 0, 180));
    s3.write(map(p3, 0, 255, 0, 180));

    // Debugging (Optional - comment out if it slows things down too much)
    // Serial.print("RX: "); Serial.print(p1); Serial.print(" "); Serial.println(p3);

    // 5. Reset the Flag
    // Tell the radio "I have read the data, clear the alert."
    nrfWriteReg(0x07, 0x40);
  }
}