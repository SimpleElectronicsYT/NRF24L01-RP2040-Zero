/*
 * ======================================================================================
 * PROJECT: RP2040 NRF24L01 TRANSMITTER WITH OLED VISUALIZER
 * ======================================================================================
 * * DESCRIPTION:
 * Reads 3 potentiometers and sends their positions wirelessly to a receiver.
 * Simultaneously displays the values as a bar graph on an OLED screen.
 * * KEY FEATURES:
 * 1. Non-Blocking: Uses internal timers (millis) instead of delay() so the radio
 * and screen can run at different speeds without stopping each other.
 * 2. Hybrid Communication: 
 * - Radio uses "Bit-Banging" (manual pin control) for maximum compatibility.
 * - Screen uses "Hardware I2C" (slowed to 100kHz) for stability.
 * * WIRING (RP2040-Zero):
 * - GP26, GP27, GP28 -> Potentiometer Wipers (Middle Pins)
 * - GP4  -> NRF24L01 CSN  (Chip Select Not)
 * - GP3  -> NRF24L01 CE   (Chip Enable)
 * - GP7  -> NRF24L01 SCK  (Clock)
 * - GP5  -> NRF24L01 MOSI (Master Out Slave In)
 * - GP6  -> NRF24L01 MISO (Master In Slave Out)
 * - GP14 -> OLED SDA
 * - GP15 -> OLED SCL
 * * LIBRARIES REQUIRED:
 * - U8g2 (by olikraus)
 */

#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

// --- PIN DEFINITIONS ---
// These constants make it easy to change pins later if you rebuild the circuit.
const int CSN_PIN  = 4;   // NRF24L01 Chip Select
const int CE_PIN   = 3;   // NRF24L01 Radio Enable
const int SCK_PIN  = 7;   // NRF24L01 Clock
const int MOSI_PIN = 5;   // NRF24L01 Data OUT
const int MISO_PIN = 6;   // NRF24L01 Data IN
const int POT1     = 26;  // Analog Input 1
const int POT2     = 27;  // Analog Input 2
const int POT3     = 28;  // Analog Input 3

// --- OLED SCREEN SETUP ---
// We use the U8g2 library because it handles clone screens better than Adafruit.
// "2ND_HW_I2C" tells it to use the RP2040's secondary I2C hardware (Wire1).
// U8X8_PIN_NONE means we don't have a reset pin connected.
U8G2_SSD1306_128X64_NONAME_F_2ND_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// --- RADIO CONFIGURATION ---
// This 5-byte code is the "Frequency Address". 
// The Receiver MUST have the exact same address to hear us.
byte address[5] = {0xAB, 0xCD, 0x12, 0x34, 0x56};

// --- TIMING VARIABLES (NON-BLOCKING) ---
// We use these to track "when was the last time we did X?"
unsigned long previousRadioMillis = 0;
unsigned long previousOledMillis = 0;

// Update Rates:
// Radio: 20ms = 50 times per second (Fast, for smooth servo movement)
// Screen: 200ms = 5 times per second (Slow, to save processor power)
const long radioInterval = 20; 
const long oledInterval = 200; 

// Global variables to store pot values so both Radio and Screen can see them
byte val1, val2, val3;

// ======================================================================================
// SECTION: BIT-BANG SPI FUNCTIONS
// These functions manually toggle pins to talk to the radio. 
// We do this because Hardware SPI was unstable on this specific build.
// ======================================================================================

// Sends/Receives one byte of data
byte bitbangSPI(byte data) {
  byte incoming = 0;
  // Loop 8 times (once for each bit in the byte)
  for (int i = 7; i >= 0; i--) {
    // 1. Set the Output Pin (MOSI) to the correct bit (0 or 1)
    digitalWrite(MOSI_PIN, (data >> i) & 0x01);
    delayMicroseconds(2); // Short wait for signal stability
    
    // 2. Pulse the Clock (SCK) HIGH
    digitalWrite(SCK_PIN, HIGH);
    delayMicroseconds(2); 
    
    // 3. Read the Input Pin (MISO)
    if (digitalRead(MISO_PIN)) incoming |= (1 << i);
    
    // 4. Pulse the Clock (SCK) LOW
    digitalWrite(SCK_PIN, LOW);
    delayMicroseconds(2); 
  }
  return incoming; // Return the byte we received
}

// Helper to write a command to a specific NRF24L01 register
void nrfWriteReg(byte reg, byte value) {
  digitalWrite(CSN_PIN, LOW);  // Select the radio (Wake up)
  bitbangSPI(0x20 | reg);      // Send Command: "Write to Register" + Register Number
  bitbangSPI(value);           // Send the Value
  digitalWrite(CSN_PIN, HIGH); // Deselect (Sleep)
}

// ======================================================================================
// SECTION: SETUP (Runs once on power up)
// ======================================================================================
void setup() {
  Serial.begin(115200);

  // --- 1. HARDWARE I2C OLED CONFIGURATION ---
  // The RP2040 needs to be told which pins to use for I2C Wire1
  Wire1.setSDA(14);
  Wire1.setSCL(15);
  Wire1.begin();
  // CRITICAL FIX: Force speed to 100kHz. 
  // Clone OLEDs often glitch or turn to static if this is too fast.
  Wire1.setClock(100000); 

  // Initialize the screen driver
  u8g2.begin();
  u8g2.setBusClock(100000); // Tell the library to respect the slow speed
  u8g2.clearBuffer();       // Clear video memory
  u8g2.setFont(u8g2_font_ncenB08_tr); // Set a nice font
  u8g2.drawStr(0, 10, "Radio Init...");
  u8g2.sendBuffer();        // Push to screen

  // --- 2. RADIO PIN CONFIGURATION ---
  pinMode(CE_PIN, OUTPUT);
  pinMode(CSN_PIN, OUTPUT);
  pinMode(SCK_PIN, OUTPUT);
  pinMode(MOSI_PIN, OUTPUT);
  pinMode(MISO_PIN, INPUT);

  // Default states
  digitalWrite(CSN_PIN, HIGH); // CSN is active LOW, so start HIGH (off)
  digitalWrite(CE_PIN, LOW);   // CE is active HIGH, so start LOW (off)

  Serial.println("--- TX STARTING ---");
  delay(100); // Tiny pause to let power stabilize

  // --- 3. RADIO REGISTRY CONFIGURATION ---
  // 0x00 CONFIG: Power Up, Enable CRC, Transmitter Mode
  nrfWriteReg(0x00, 0x0E); 
  delay(5); // Required wait after Power Up

  // 0x03 SETUP_AW: Set Address Width to 5 bytes
  nrfWriteReg(0x03, 0x03); 
  
  // Set the "Transmit To" Address
  digitalWrite(CSN_PIN, LOW);
  bitbangSPI(0x20 | 0x10); // Write to TX_ADDR register
  for(int i=0; i<5; i++) bitbangSPI(address[i]);
  digitalWrite(CSN_PIN, HIGH);

  // Set the "Receive From" Address (Pipe 0)
  // Essential for Auto-Acknowledgment (The RX replies "I got it!")
  digitalWrite(CSN_PIN, LOW);
  bitbangSPI(0x20 | 0x0A); // Write to RX_ADDR_P0 register
  for(int i=0; i<5; i++) bitbangSPI(address[i]);
  digitalWrite(CSN_PIN, HIGH);

  // Radio Tuning
  nrfWriteReg(0x01, 0x3F); // Enable Auto-Ack on all pipes
  nrfWriteReg(0x05, 0x4C); // RF Channel 76 (2.476 GHz)
  nrfWriteReg(0x06, 0x07); // Data Rate 1Mbps, Power 0dBm (Max)

  Serial.println("Hardware Initialized.");
  
  // Update Screen to show we are ready
  u8g2.clearBuffer();
  u8g2.drawStr(0, 10, "System Ready");
  u8g2.sendBuffer();
}

// ======================================================================================
// SECTION: MAIN LOOP (Runs forever)
// ======================================================================================
void loop() {
  unsigned long currentMillis = millis(); // Get current time

  // --- TASK 1: READ SENSORS ---
  // We read these every single loop because it is very fast.
  // map() converts the 0-1023 (Analog) to 0-255 (Byte) for sending.
  val1 = map(analogRead(POT1), 0, 1023, 0, 255);
  val2 = map(analogRead(POT2), 0, 1023, 0, 255);
  val3 = map(analogRead(POT3), 0, 1023, 0, 255);

  // --- TASK 2: SEND RADIO PACKET (Every 20ms) ---
  if (currentMillis - previousRadioMillis >= radioInterval) {
    previousRadioMillis = currentMillis; // Reset timer
    
    // 1. Clear Interrupt Flags (Reset "Sent" status)
    nrfWriteReg(0x07, 0x70); 
    
    // 2. Flush TX Buffer (Delete old unsent data)
    digitalWrite(CSN_PIN, LOW); 
    bitbangSPI(0xE1); 
    digitalWrite(CSN_PIN, HIGH);

    // 3. Load Payload into Radio
    digitalWrite(CSN_PIN, LOW);
    bitbangSPI(0xA0); // Command: Write TX Payload
    bitbangSPI(val1); // Byte 1
    bitbangSPI(val2); // Byte 2
    bitbangSPI(val3); // Byte 3
    digitalWrite(CSN_PIN, HIGH);

    // 4. Pulse CE Pin to Fire the transmission
    digitalWrite(CE_PIN, HIGH);
    delayMicroseconds(15); // Radio needs at least 10us
    digitalWrite(CE_PIN, LOW);
  }

  // --- TASK 3: UPDATE OLED SCREEN (Every 200ms) ---
  if (currentMillis - previousOledMillis >= oledInterval) {
    previousOledMillis = currentMillis; // Reset timer

    u8g2.clearBuffer(); // Start with blank canvas
    
    // Draw Title
    u8g2.drawStr(0, 10, "TX Status: Active");

    // Math: Convert 0-255 value to 0-100 pixel width
    int w1 = map(val1, 0, 255, 0, 100);
    int w2 = map(val2, 0, 255, 0, 100);
    int w3 = map(val3, 0, 255, 0, 100);

    // Draw Bar 1
    u8g2.drawStr(0, 25, "S1");     // Label
    u8g2.drawFrame(20, 16, 102, 10); // Empty Box (x, y, width, height)
    u8g2.drawBox(22, 18, w1, 6);     // Filled Bar inside the box

    // Draw Bar 2
    u8g2.drawStr(0, 40, "S2");
    u8g2.drawFrame(20, 31, 102, 10);
    u8g2.drawBox(22, 33, w2, 6);

    // Draw Bar 3
    u8g2.drawStr(0, 55, "S3");
    u8g2.drawFrame(20, 46, 102, 10);
    u8g2.drawBox(22, 48, w3, 6);

    u8g2.sendBuffer(); // Actually update the pixels on the screen
  }
}