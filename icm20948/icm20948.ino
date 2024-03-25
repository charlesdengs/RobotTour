
#include "ICM_20948.h"

#define SERIAL_PORT Serial

#define WIRE_PORT Wire  // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

ICM_20948_I2C myICM;  // Otherwise create an ICM_20948_I2C object

void setup() {

  SERIAL_PORT.begin(115200);
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
  bool initialized = false;
  while (!initialized) {

    myICM.begin(WIRE_PORT, AD0_VAL);

    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok) {
      SERIAL_PORT.println("Trying again...");
      delay(500);
    } else {
      initialized = true;
    }
  }
}

void loop() {

  if (myICM.dataReady()) {
    myICM.getAGMT();           // The values are only updated when you call 'getAGMT'
    //printRawAGMT(myICM.agmt);  // Uncomment this to see the raw values, taken directly from the agmt structure
    printScaledAGMT(&myICM); // This function takes into account the scale settings from when the measurement was made to calculate the values with units
    //delay(30);
  } else {
    SERIAL_PORT.println("Waiting for data");
    delay(500);
  }
}

void printRawAGMT(ICM_20948_AGMT_t agmt) {
  /*
  SERIAL_PORT.print("Gyr [ ");
  Serial.print(agmt.gyr.axes.x);
  SERIAL_PORT.print(", ");
  Serial.print(agmt.gyr.axes.y);
  SERIAL_PORT.print(", ");
  Serial.print(agmt.gyr.axes.z);
  SERIAL_PORT.print(" ], Mag [ ");
  Serial.print(agmt.mag.axes.x);
  SERIAL_PORT.print(", ");
  Serial.print(agmt.mag.axes.y);
  SERIAL_PORT.print(", ");
  Serial.print(agmt.mag.axes.z);
  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
  */
  
}

void printScaledAGMT(ICM_20948_I2C *sensor) {
  /*
  SERIAL_PORT.print(" ], Gyr (DPS) [ ");
  printFormattedFloat(sensor->gyrX(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->gyrY(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->gyrZ(), 5, 2);
  SERIAL_PORT.print(" ], Mag (uT) [ ");
  printFormattedFloat(sensor->magX(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->magY(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->magZ(), 5, 2);
  SERIAL_PORT.println();
  */
  int x =sensor->accX();
  Serial.print(sensor->accX());
  Serial.println();
}
