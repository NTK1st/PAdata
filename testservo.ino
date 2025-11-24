#include <IROVER.h>

void setup() {
  init(0x48); // set iKB1z address 0x48
}
void loop() {
  for (int  i = 1; i <= 180; i++) {
    // set servo port 10 degree to i
    servo(13, i);
    delay(20);
  }
  for (int  i = 180; i >= 1; i--) {
    // set servo port 10 degree to i
    servo(13, i);
    delay(20);
  }

}
