#ifndef SD_HANLDER
#define SD_HANDLER

#include <Arduino.h>
#include <SD.h>

// Trys Init
int initSD(int _ssPin) {
  if (!SD.begin(_ssPin)) {
    return -1;
  }

  // Test write/read
  File file = SD.open("/test.txt", FILE_WRITE);
  if (file) {
    file.println("Hello ESP32 SPI Flash SD!");
    file.close();
    Serial.println("File written successfully.");
  } else {
    Serial.println("Failed to open file for writing.");
  }

  file = SD.open("/test.txt");
  if (file) {
    Serial.println("Reading file:");
    while (file.available()) {
      Serial.write(file.read());
    }
    file.close();
    Serial.println();
  } else {
    Serial.println("Failed to open file for reading.");
  }
}





#endif