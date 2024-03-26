#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  bool initialized = false;
  while (!initialized)
  {

    myICM.begin(Wire, 1);


    Serial.print(F("Initialization of the sensor returned: "));
    Serial.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      Serial.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }
  
  // Here we are doing a SW reset to make sure the device starts in a known state
  myICM.swReset();
  delay(250);
  myICM.sleep(false);
  myICM.lowPower(false);

  // Set Gyro and Accelerometer to a particular sample mode
  // options: ICM_20948_Sample_Mode_Continuous
  //          ICM_20948_Sample_Mode_Cycled
  myICM.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);
  
  // Set full scale ranges for both acc and gyr
  ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors

  myFSS.a = gpm2; // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                  // gpm2
                  // gpm4
                  // gpm8
                  // gpm16

  myICM.setFullScale(ICM_20948_Internal_Acc, myFSS);


  // Set up Digital Low-Pass Filter configuration
  ICM_20948_dlpcfg_t myDLPcfg;    // Similar to FSS, this uses a configuration structure for the desired sensors
  myDLPcfg.a = acc_d246bw_n265bw; // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
                                  // acc_d246bw_n265bw      - means 3db bandwidth is 246 hz and nyquist bandwidth is 265 hz
                                  // acc_d111bw4_n136bw
                                  // acc_d50bw4_n68bw8
                                  // acc_d23bw9_n34bw4
                                  // acc_d11bw5_n17bw
                                  // acc_d5bw7_n8bw3        - means 3 db bandwidth is 5.7 hz and nyquist bandwidth is 8.3 hz
                                  // acc_d473bw_n499bw

  myICM.setDLPFcfg(ICM_20948_Internal_Acc, myDLPcfg);
  ICM_20948_Status_e accDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Acc, false);
 
}

void loop()
{

  if (myICM.dataReady())
  {
    myICM.getAGMT();              // The values are only updated when you call 'getAGMT'
    //printRawAGMT( myICM.agmt ); // Uncomment this to see the raw values, taken directly from the agmt structure
    printScaledAGMT(&myICM);      // This function takes into account the scale settings from when the measurement was made to calculate the values with units
    delay(30);
  }
  else
  {
    Serial.println("Waiting for data");
    delay(500);
  }
}


void Sensor(ICM_20948_I2C *sensor)
{

  //Serial.print("Scaled. Acc (mg) [ ");
  /*
  if(fabs(sensor->accX()) <= 50.0 + sensor->temp()*.8)
  {
    Serial.println(0);
    
  }
  else
  {
  Serial.println(sensor->accX());
  }
  */
  Serial.println(sensor->accX());
}
