#include <SimpleZigBeeRadio.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal.h>

// (RX => DOUT, TX => DIN)
SoftwareSerial xbeeSerial(9, 10);

// Create the XBee object ...
SimpleZigBeeRadio xbee = SimpleZigBeeRadio();

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

SimpleIncomingZigBeePacket incomingPacket;
SimpleOutgoingZigBeePacket outgoingPacket;
SimpleZigBeeAddress64 addr64;
const int ledPin = 7;
const int actuatorButtonPin = 8;
const int actuatorLedPin = 13;
int actuatorReading;
int actuatorPrevious = LOW;
int actuatorState = HIGH;
long actuatorTime = 0;
long actuatorDebounce = 500;
long startTime = 0;
long endTime = 0;
long deltaTime = 0;
float voltage = 0;
float dMMVoltage = 0;
float celsius = 0;
float fahrenheit = 0;
uint16_t frameLength = 0;
uint8_t frameId = 0;
uint8_t frameType = 0;
uint8_t rxLength = 0;
uint16_t val = 0;
uint8_t rxData = 0;
uint32_t endDeviceMSB = 0x0013A200;
uint32_t endDeviceLSB = 0x4124264C;
uint32_t routerMSB = 0x0013A200;
uint32_t routerLSB = 0x40C1AF5A;
uint16_t router16 = 0x4CC2;
uint16_t routerCommand = 0;
uint8_t routerPayload = 0;
uint32_t dh = 0;
uint32_t dl = 0;

void setup() {

  // start serial communications
  Serial.begin(115200);
  xbeeSerial.begin(9600);

  // set the serial port for the XBee radio.
  xbee.setSerial(xbeeSerial);

  // set a non-zero frame id to receive Status and Response packets.
  xbee.setAcknowledgement(true);

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);

  // set up incoming packet LED (this LED lets the user know when an autosample is underway)
  pinMode(ledPin, OUTPUT);

  // set up the remote actuator button and LED pin
  pinMode(actuatorButtonPin, INPUT);
  pinMode(actuatorLedPin, OUTPUT);
}

void loop() {

  // manage the remote actuator state (local button and LED)
  actuatorReading = digitalRead(actuatorButtonPin);
  if (actuatorReading == HIGH && actuatorPrevious == LOW && millis() - actuatorTime > actuatorDebounce) {
    if (actuatorState == HIGH) {
      Serial.println("\nRemote Actuator Deactivated");
      actuatorState = LOW;
      routerPayload = 0x0205; // includes command option byte 0x02 (for update) and command parameter 0x05 (digital output HIGH)
    } else {
      Serial.println("\nRemote Actuator Activated");
      actuatorState = HIGH;
      routerPayload = 0x0204; // includes command option byte 0x02 (for update) and command parameter 0x04 (digital output LOW)
    }
    routerCommand = 0x31374430; // AT Command: 17D0 ASCII to HEX => 0x31374430
    xbee.prepareRemoteATCommand(routerMSB, routerLSB, router16, routerCommand, routerPayload);
    xbee.send();
    outgoingPacket = xbee.getOutgoingPacketObject();
    Serial.print("Outgoing Packet: ");
    printPacket(outgoingPacket);
    Serial.print("\n");
    xbee.flush();
    actuatorTime = millis();
  }
  digitalWrite(actuatorLedPin, actuatorState);
  actuatorPrevious = actuatorReading;

  // begin analyzing packet data when it becomes available
  if (xbee.available()) {

    // read the packet when it becomes available
    xbee.read();

    // filter out any incomplete packets
    if (xbee.isComplete()) {

      // get various SimpleZigBeeRadio packet parameters
      frameLength = xbee.getIncomingFrameType();
      frameId = xbee.getIncomingFrameID();
      frameType = xbee.getIncomingFrameType();
      addr64 = xbee.getRXAddress64();
      dh = addr64.getAddressMSB();
      dl = addr64.getAddressLSB();
      incomingPacket = xbee.getIncomingPacketObject();
      rxLength = xbee.getRXPayloadLength();

      // filter for end device (auto sampling device in cyclical sleep mode)
      if (dh == endDeviceMSB && dl == endDeviceLSB) {

        // turn on the incoming packet LED (autosample in progress)
        digitalWrite(ledPin, HIGH);

        // only print packets that have frame type 0x92
        if (frameType == 0x92) {

          // calculate the time since the last packet was received
          endTime = millis();
          deltaTime = endTime - startTime;
          startTime = millis();

          // filter out *repeat* packets (1 second threshold)
          if (deltaTime > 1000) {

            // print the time between packets to the serial monitor
            Serial.print("\nIncoming Packet Received from End Device (0x");
            Serial.print(addr64.getAddressMSB(), HEX);
            Serial.print(addr64.getAddressLSB(), HEX);
            Serial.println(")");
            Serial.print("Time since last packet ~ ");
            Serial.print(deltaTime);
            Serial.println(" ms (Sleepy End Device)");

            // print the time between packets to the LCD
            lcd.setCursor(0, 0);
            for (int i = 0; i < 16; ++i) {
              lcd.print(' ');
            }
            lcd.setCursor(0, 0);
            lcd.print("Time: ");
            lcd.print(deltaTime);
            lcd.print(" ms");

            // print zigbee packet
            Serial.print("Incoming Packet: ");
            printPacket(incomingPacket);

            Serial.print("Frame ID: ");
            Serial.println(frameId);

            // print the frame type
            Serial.println("Frame Type: 0x92 IO Data Sample RX Indicator");

            // print the payload to Serial and LCD
            Serial.print("Payload: ");
            for (int i = 0; i < rxLength; i++) {
              rxData = xbee.getRXPayload(i);
              Serial.print(rxData, HEX);
              Serial.print(" ");
            }
            Serial.print("\n");

            // get and print the length of the packet's payload
            Serial.print("Payload Length: ");
            Serial.println(rxLength);

            // get and print the sensor value
            val = word(xbee.getRXPayload(rxLength - 2), xbee.getRXPayload(rxLength - 1));
            Serial.print("Analog Sample Value (HEX): ");
            Serial.println(val, HEX);
            Serial.print("Analog Sample Value (DEC): ");
            Serial.println(val);

            // get the sensor's voltage (should use 1.2 for XBee, but 1.25 yields better accuracy)
            voltage = (val * 1.25) / 1023.0;
            Serial.print("Voltage (V): ");
            Serial.println(voltage);

            // get the DMM voltage (should use 1.2 for XBee, but 1.25 yields better accuracy)
            dMMVoltage = voltage * 1.25;
            Serial.print("DMM Voltage (V): ");
            Serial.println(dMMVoltage);

            // calculate degrees in C and F
            celsius = (voltage - .5) * 100.0;
            fahrenheit = ((celsius * 9.0) / 5.0) + 32.0;

            // print the temperature to the serial monitor
            Serial.print("Temperature (Degrees F): ");
            Serial.println(fahrenheit);
            Serial.print("Temperature (Degrees C): ");
            Serial.println(celsius);
            Serial.print("\n");

            // print the temperature to the LCD
            lcd.setCursor(0, 1);
            for (int i = 0; i < 16; ++i) {
              lcd.print(' ');
            }
            lcd.setCursor(0, 1);
            lcd.print(fahrenheit);
            lcd.print(" F  ");
            lcd.print(celsius);
            lcd.print(" C");
          }
          // turn off the incoming packet LED (autosample complete)
          digitalWrite(ledPin, LOW);
        } else {

          // print a message to let the user know the packet was not not frame type 0x92
          Serial.println("Packet is not Frame Type 0x92 (not an autosample packet)");
        }
      } else {

        Serial.print("Incoming Packet Received from Another Device (0x");
        Serial.print(addr64.getAddressMSB(), HEX);
        Serial.print(addr64.getAddressLSB(), HEX);
        Serial.println(")");

        // print zigbee packet
        Serial.print("Incoming Packet: ");
        printPacket(incomingPacket);
      }
      xbee.flush();
    }
  }
}

void printPacket(SimpleZigBeePacket & p){
  Serial.print(START, HEX);
  Serial.print(' ');
  Serial.print(p.getLengthMSB(), HEX);
  Serial.print(' ');
  Serial.print(p.getLengthLSB(), HEX);
  Serial.print(' ');
  // Frame Type and Frame ID are stored in Frame Data
  uint8_t checksum = 0;
  for(int i=0; i<p.getFrameLength(); i++) {
    Serial.print(p.getFrameData(i), HEX);
    Serial.print(' ');
    checksum += p.getFrameData(i);
  }
  // Calculate checksum based on summation of frame bytes
  checksum = 0xff - checksum;
  Serial.print(checksum, HEX);
  Serial.println();
}
