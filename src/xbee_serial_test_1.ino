#include <SimpleZigBeeRadio.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal.h>

// Create the XBee object ...
SimpleZigBeeRadio xbee = SimpleZigBeeRadio();

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

// (RX => DOUT, TX => DIN)
SoftwareSerial xbeeSerial(9, 10);

const int ledPin = 7;
long startTime = 0;
long endTime = 0;
long deltaTime = 0;

void setup() {

  // start serial communications
  Serial.begin(9600);
  xbeeSerial.begin(9600);

  // set the serial port for the XBee radio.
  xbee.setSerial(xbeeSerial);

  // set a non-zero frame id to receive Status and Response packets.
  xbee.setAcknowledgement(true);

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);

  // set up incoming packet LED
  pinMode(ledPin, OUTPUT);
}

void loop() {

  while (xbee.available()) {

    // read the packet when it becomes available
    xbee.read();

    if (xbee.isComplete()) {

      if (xbee.getIncomingFrameType() == 0x92) {

        // calculate the time since the last packet was received
        endTime = millis();
        deltaTime = endTime - startTime;
        startTime = millis();

        if (deltaTime > 1000) {

          // print the time between packets to the serial monitor
          // (filter out *repeat* messages that occur less than 1 second)
          Serial.println("Incoming XBee Message ...");
          Serial.print("Time since last packet ~ ");
          Serial.print(deltaTime);
          Serial.println(" ms");

          // print the time between packets to the LCD
          lcd.setCursor(0, 0);
          for (int i = 0; i < 16; ++i) {
            lcd.print(' ');
          }
          lcd.setCursor(0, 0);
          lcd.print("Time: ");
          lcd.print(deltaTime);
          lcd.print(" ms");
          //lcd.printf("Delta Time: 2%d s\n", (deltaTime / 1000));

          // print the frame type
          Serial.println("Frame Type: IO Data Sample RX Indicator");

          // turn on the LED
          digitalWrite(ledPin, HIGH);


          int id = xbee.getIncomingFrameID();
          Serial.print("Frame ID: ");
          Serial.println(id);

          // print zigbee packet
          Serial.print("ZigBee Packet: ");
          printPacket(xbee.getIncomingPacketObject());

          // get and print the length of the packet's payload
          uint8_t rxLength = xbee.getRXPayloadLength();
          Serial.print("Payload Length: ");
          Serial.println(rxLength);

          // get the sensor value
          uint16_t val = word(xbee.getRXPayload(rxLength - 2), xbee.getRXPayload(rxLength - 1));

          // print the payload to Serial and LCD
          Serial.print("Payload: ");
          //lcd.setCursor(0, 1);
          for (int i = 0; i < rxLength; i++) {
            uint8_t rxData = xbee.getRXPayload(i);
            Serial.print(rxData, HEX);
            Serial.print(' ');
            //lcd.print(rxData, HEX);
            //lcd.print(' ');
          }
          Serial.print("\n");

          // print the sensor value
          Serial.print("Analog Value (HEX): ");
          Serial.println(val, HEX);
          Serial.print("Analog Value (DEC): ");
          Serial.println(val);

          // get the sensor's voltage
          float voltage = (val * 1.25) / 1023.0;
          Serial.print("Voltage (V): ");
          Serial.println(voltage);

          // get the DMM voltage
          float dMMVoltage = voltage * 1.25;
          Serial.print("DMM Voltage (V): ");
          Serial.println(dMMVoltage);

          // calculate degrees in C and F
          float celsius = (voltage - .5) * 100.0;
          float fahrenheit = ((celsius * 9.0) / 5.0) + 32.0;

          // print the temperature to the serial monitor
          Serial.print("Temperature (Degrees F): ");
          Serial.println(fahrenheit);
          Serial.print("Temperature (Degrees C): ");
          Serial.println(celsius);

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
          Serial.print("\n");

          // turn off the LED
          digitalWrite(ledPin, LOW);
        }
      } else {
        Serial.println("Other Frame Type: Disregard");
      }
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
