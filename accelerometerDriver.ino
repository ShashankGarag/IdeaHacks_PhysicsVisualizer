class accelDriver{
  public:
    accelDriver(int x, int y, int z){
      pinX = x;
      pinY = y;
      pinZ = z;
    }

    void initialize(){
      analogReadResolution(12);
    }

    void readRawData(int &accelX, int &accelY, int &accelZ) {
      accelX = analogRead(pinX);
      accelY = analogRead(pinY);
      accelZ = analogRead(pinZ);
    }

    void readAcceleration(int &accelX, int &accelY, int &accelZ) {
      float vX = analogRead(pinX) * (SYSTEM_VOLTAGE/ADC_RESOLUTION);
      float vY = analogRead(pinY) * (SYSTEM_VOLTAGE/ADC_RESOLUTION);
      float vZ = analogRead(pinZ) * (SYSTEM_VOLTAGE/ADC_RESOLUTION);

      float gX = (vX - ZERO_G_VOLTAGE)/SENSITIVITY;
      float gY = (vY - ZERO_G_VOLTAGE)/SENSITIVITY;
      float gZ = (vZ - ZERO_G_VOLTAGE)/SENSITIVITY;

      accelX = gX * GRAVITY;
      accelY = gY * GRAVITY;
      accelZ = gZ * GRAVITY;
    }
  
  private:
    int pinX;
    int pinY;
    int pinZ;

    const float SYSTEM_VOLTAGE = 3.3;
    const float ADC_RESOLUTION = 4095.0;
    const float ZERO_G_VOLTAGE = 1.65;
    const float SENSITIVITY = 0.8;
    const float GRAVITY = 9.81;

};

accelDriver accelerometer(35, 36, 37);
int aX, aY, aZ;
float forceX, forceY, forceZ;
int systemMass = 2;

void setup() {
  Serial.begin(115200);
  accelerometer.begin();
}

void loop() {
  accelerometer.readAcceleration(aX, aY, aZ);

  forceX = systemMass * aX;
  forceY = systemMass * aY;
  forceZ = systemMass * aZ;
  
  delay(100);
}
