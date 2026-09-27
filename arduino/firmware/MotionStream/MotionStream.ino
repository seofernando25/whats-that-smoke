#include <Arduino_BMI270_BMM150.h>
#include <Wire.h>
#include <ArduinoBLE.h>
BLEService motion("cfb00001-7c42-4cb8-9ac0-27d96c562e11");
BLECharacteristic samples("cfb00002-7c42-4cb8-9ac0-27d96c562e11", BLERead | BLENotify, 20, true);
BLECharacteristic ack("cfb00003-7c42-4cb8-9ac0-27d96c562e11", BLEWrite | BLEWriteWithoutResponse, 4, true);
// ~2.5 seconds at 100 Hz. Cumulative ACKs release samples; retry retains timestamps.
uint8_t ring[256][20];
uint32_t head=0, tail=0, cursor=0, retryAt=0;
bool wasConnected=false;
bool readRegs(uint8_t reg, uint8_t *data, uint8_t n) {
 Wire1.beginTransmission(0x68); Wire1.write(reg);
 if(Wire1.endTransmission(false)!=0) return false;
 if(Wire1.requestFrom(0x68,(int)n)!=n) return false;
 for(int i=0;i<n;i++) data[i]=Wire1.read();
 return true;
}
void setup() {
 Serial.begin(115200);
 if(!IMU.begin(BOSCH_ACCELEROMETER_ONLY)) while(true){Serial.println("ERROR IMU");delay(1000);}
 Wire1.setClock(400000);
 if(!BLE.begin()) while(true) delay(1000);
 BLE.setLocalName("MotionLab-Nano");
 BLE.setAdvertisedService(motion); motion.addCharacteristic(samples); motion.addCharacteristic(ack); BLE.addService(motion);
 BLE.setConnectionInterval(6,12); BLE.advertise();
}
void loop() {
 BLE.poll();
 bool connected=BLE.connected() && samples.subscribed();
 if(connected != wasConnected) {tail=head;cursor=head;retryAt=millis();wasConnected=connected;}
 if(ack.written() && ack.valueLength()==4) {
  uint32_t last;ack.readValue((uint8_t*)&last,4);
  uint32_t advance=(last+1)-tail;
  if(advance>0 && advance<=head-tail){tail=last+1;retryAt=millis();}
  if(cursor-tail>head-tail) cursor=tail;
 }
 uint8_t status,data[12];
 if(connected && readRegs(0x03,&status,1) && (status&0x40) && readRegs(0x0C,data,12)) {
  uint32_t stamp=micros();
  if(head-tail>=256){tail++;if(cursor-tail>head-tail)cursor=tail;}
  uint8_t* packet=ring[head%256];
  memcpy(packet,&stamp,4);memcpy(packet+4,data,12);memcpy(packet+16,&head,4);head++;
 }
 if(connected && head!=tail) {
  if(millis()-retryAt>40){cursor=tail;retryAt=millis();}
  if(cursor!=head && cursor-tail<32) {
   if(samples.writeValue(ring[cursor%256],20)) cursor++;
  }
 }
}
