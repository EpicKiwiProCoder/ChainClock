#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <CheapStepper.h>

IPAddress timeServer(129, 6, 15, 28);  // time.nist.gov NTP server

const int NTP_PACKET_SIZE = 48;  // NTP time stamp is in the first 48 bytes of the message

byte packetBuffer[NTP_PACKET_SIZE];  //buffer to hold incoming and outgoing packets

WiFiUDP Udp;

CheapStepper stepper (2,3,4,5); 

bool moveClockwise = false;
const long rotation = 4096; // 4096 rotating too little, actual position is lower
const long stepsPerHour = 2731;
long totalSteps = 0;
long rotationError = 0;
float lastHour = 0;
long lastAdjustment = 0;

bool lastSensorState = true;

unsigned long lastSeconds = 0;
unsigned long secondsOffset = 0;

unsigned long lastTimeRefresh = 0UL;
unsigned long lastUnixTime = 0UL;

int utcDifference = 1;

void setup() {
  Serial.begin(9600);
  stepper.setRpm(10); 
  stepper.setTotalSteps(rotation);
  pinMode(7, INPUT_PULLUP);

  home();
  //delay(10000);
  connectWifi("ABCM Not in Range", "marion013");
  Udp.begin(2390U);
  refreshTime();
}

void loop() {
  if (debouncedRead(7, true)) { // sensor is triggered
    if (lastSensorState == false && totalSteps - lastAdjustment > 2000) {
      lastSensorState = true;
      rotationError = getRotationError(totalSteps) - 60;
      Serial.println(totalSteps);
      totalSteps -= rotationError;
      lastAdjustment = totalSteps;
      Serial.println(totalSteps);
    }
  } else {
    lastSensorState = false;
  }
  if (seconds() - lastTimeRefresh > 3600) {
    refreshTime();
  }
  float currentHour = getHour();
  Serial.println(currentHour);
  if (currentHour < lastHour) { // Reset to 0 hours
    stepper.move(!moveClockwise, rotation / 4);
    home();
  }
  lastHour = currentHour;
  moveToPosition(currentHour*stepsPerHour);
  delay(5000);
}

void home() {
  if (!debouncedRead(7, false)) { // if home is called in activation zone
    stepper.move(moveClockwise, rotation/4);
  }
  while (!debouncedRead(7, true)) { // debouncedRead(7, true) == true if in contact
    stepper.move(moveClockwise, 1);
  }
  stepper.move(!moveClockwise, 60); // align fully
  lastSensorState = true;
  totalSteps = 0L;
}

long getRotationError(long steps) {
  int counter = 1;
  long least = steps;
  while (true) {
    if (abs(steps - (counter * rotation)) < abs(least)) {
      least = steps - (counter * rotation);
      counter++;
    }
    else {
      return least;
    }
  }
}

void moveToPosition(long steps) {
  long difference = steps - totalSteps;
  bool moveCw = moveClockwise ^ (difference < 0);

  long fullRotations = abs(difference) / rotation;
  long remainingSteps = abs(difference) % rotation;
  stepper.move(moveCw, fullRotations * rotation);
  stepper.move(moveCw, remainingSteps);
  totalSteps = steps;
}

unsigned long seconds() {
  unsigned long currentSeconds = millis() / 1000;
  if (currentSeconds < lastSeconds) { // Reset to zero
    secondsOffset += lastSeconds;
  }
  lastSeconds = currentSeconds;
  return currentSeconds + secondsOffset;
  // return millis() / 1000;
}

float getHour() { // 0.00 TO 11.99
  unsigned long currentTime = seconds() - lastTimeRefresh + lastUnixTime + (utcDifference*3600);
  float hour = (currentTime % 86400L) / 3600.0;
  if (hour > 12) {
    return hour - 12;
  } else {
    return hour;
  }
}

void refreshTime() {
  lastUnixTime = getUnixTime();
  lastTimeRefresh = seconds();

  int dayOfTheYear = lastUnixTime % 31556926UL / 86400UL;
  utcDifference = 1;
  if (dayOfTheYear >= 85 && dayOfTheYear < 302) {
    utcDifference = 2;
  }
}

unsigned long getUnixTime() {
  while (true) {
    sendNTPpacket(timeServer);  
    delay(1000);

    if (Udp.parsePacket()) {
      Udp.read(packetBuffer, NTP_PACKET_SIZE);
      unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
      unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
      unsigned long secsSince1900 = highWord << 16 | lowWord;
      return secsSince1900 - 2208988800UL;
    } else {
      Serial.println("Packet Failed");
    }
  }
}

void connectWifi(char ssid[], char pass[]) {
  int status = WL_IDLE_STATUS;
  while (status != WL_CONNECTED) {
    status = WiFi.begin(ssid, pass);
    delay(10000);
  }
}

unsigned long sendNTPpacket(IPAddress& address) {
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  packetBuffer[0] = 0b11100011;  // LI, Version, Mode
  packetBuffer[1] = 0;  // Stratum, or type of clock
  packetBuffer[2] = 6;  // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  Udp.beginPacket(address, 123);  //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

bool debouncedRead(int pinNumber, bool inverted) {
  int counter = 0;
  for (int i = 0; i < 10; i++) {
    counter += inverted ^ digitalRead(pinNumber);
  }
  return (counter == 10);
}