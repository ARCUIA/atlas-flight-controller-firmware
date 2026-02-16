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
  if (!file) {
    Serial.println("Failed to open file for writing.");
  }
  
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

  return 0;
}

#endif