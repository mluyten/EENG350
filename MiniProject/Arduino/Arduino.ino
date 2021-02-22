#include <Wire.h>
#define SLAVE_ADDRESS 0x04
byte data[32];
byte pos[3] = {0, 0, 0};

int pos = 0;
int i = 0;
int j = 0;

void setup() {
  Serial.begin(115200); // start serial for output
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);
  
  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
}

void loop() {
  delay(1)
}
 
// callback for received data
void receiveData(int byteCount){
  i = 0;
  while(Wire.available()) {
    data[i] = Wire.read();
    i++;
  }
  i--;
  
}

void sendData() {
  if (i == 0) {
    if (data[0] == 1) {
      j = 0;
      data[0] = 0;
    }
    else {
      j++;
    }
    Wire.write(data[j]);
  }
}
