#ifndef SDCard_HPP
#define SDCard_HPP

#include <Arduino.h>
#include <SD.h>


#define PATH_TO_SD_DATA "/rocket" 
#define SD_DATA_FILENAME "/rocket/data.txt"

#define BUFFER_SIZE 20

class SDCard {
    public:

        struct SD_card_data {
          float temp = 0.0f;
        };

        SDCard(int ss_pin) : ss_pin(ss_pin) {}


        bool begin(){
           if (!SD.begin(ss_pin)) {
            return false;
          }

          if (!SD.exists(PATH_TO_SD_DATA)) {
            SD.mkdir(PATH_TO_SD_DATA);
          }
        }


        bool buffered_write(){

          if (buffer_count <= 0){
            return false;
          }

          File file = SD.open(SD_DATA_FILENAME, FILE_WRITE);

          if (!file){
            return false;
          }
        
          for (int i = 0; i < buffer_count; i++){
            SD_card_data data = buffer[i];
            file.print(data.temp);
            file.print(",");
          }
          
          buffer_count = 0;
          file.close();
          return true;
        }


        bool save_to_buffer(const SD_card_data& data) {

          if (buffer_count >= buffer_size){
            return false;
          }

          buffer[buffer_count] = data;
          buffer_count++;
          return true;
      }

        int get_buffer_count(){
          return buffer_count;
        }


    private:
        int ss_pin;
        int buffer_count = 0;
        int buffer_size = BUFFER_SIZE;
        SD_card_data buffer[BUFFER_SIZE];
};

#endif