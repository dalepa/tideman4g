//BME680
/*
 * Project helloworld
 * Description:
 * Author:
 * Date:
 */

#if defined(__AVR__) || defined(ESP8266)
// For UNO and others without hardware serial, we must use software serial...
// pin #2 is IN from sensor (WHITE wire)
// Set up the serial port to use softwareserial..
#include <SoftwareSerial.h>
SoftwareSerial mySerial(2, -1);

#else
// On Leonardo/M0/etc, others with hardware serial, use hardware serial!
// #0 is white wire, data input
#define mySerial Serial1

#endif


// SONAR
int count = 0;
int mDistance[101];
float avg;

int16_t distance;  // The last measured distance
int16_t distlow;  // The lowest measured distance
int16_t disthigh;  // The highest measured distance
int16_t distmean;  // The middle measured distance
float WaterLevelHigh;
float WaterLevelLow;
float WaterLevelAvg;
float WaterLevelMean;

float SENSORHEIGHTINFEET=61/12;

bool newData = false; // Whether new data is available from the sensor
uint8_t buffer[4];  // our buffer for storing data
uint8_t idx = 0;  // our idx into the storage buffer



// UDP


UDP udp;
int resp;
size_t bufferSize;
IPAddress loggingIP(72,14,127,113);    //* 72.14.127.113
unsigned int loggingPort = 8089;
//char loggingBuffer[] = "cpu value=777";
String loggingBuffer;



//BME680
#include "Adafruit_BME680.h"

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme; // I2C


double temperatureInC = 0;
double relativeHumidity = 0;
double pressureHpa = 0;
double gasResistanceKOhms = 0;
double approxAltitudeInM = 0;

//CELL and POWER
SystemPowerConfiguration spc;
FuelGauge fuel;
CellularSignal sig = Cellular.RSSI();


int led = D7;
// setup() runs once, when the device is first turned on.
void setup() {
  // Put initialization like pinMode and begin functions here.
  Particle.publish("hello", "world");

  Serial.begin(115200);
  while (!Serial) {
    delay(10); // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Hello World");

  pinMode(led, OUTPUT);


    if (!bme.begin()) {
    Particle.publish("Log", "Could not find a valid BME680 sensor, check wiring!");
  } else {
    Particle.publish("Log", "bme.begin() success =)");
    Serial.println("BME SUCCESS");
    // Set up oversampling and filter initialization
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320*C for 150 ms

    Particle.variable("temperature", &temperatureInC, DOUBLE);
    Particle.variable("humidity", &relativeHumidity, DOUBLE);
    Particle.variable("pressure", &pressureHpa, DOUBLE);
    Particle.variable("gas", &gasResistanceKOhms, DOUBLE);
    Particle.variable("altitude", &approxAltitudeInM, DOUBLE);
  }

  //SONAR
  mySerial.begin(9600);

}

int sort_desc(const void *cmp1, const void *cmp2)
{
  // Need to cast the void * to int *
  int a = *((int *)cmp1);
  int b = *((int *)cmp2);
  // The comparison
  return a > b ? -1 : (a < b ? 1 : 0);
  // A simpler, probably faster way:
  //return b - a;
}

float average (int * array, int len)  // assuming array is int.
{
  long sum = 0L ;  // sum will be larger than an item, long for safety.
  for (int i = 0 ; i < len ; i++)
    sum += array [i] ;
  return  ((float) sum) / len ;  // average will be fractional, so float may be appropriate.
}

void toInflux(String str)
{
    resp = udp.begin(loggingPort);

    loggingBuffer=str;
 
    bufferSize = strlen(loggingBuffer);
    resp = udp.sendPacket(loggingBuffer, bufferSize, loggingIP, loggingPort);
    
    udp.stop();

    Serial.printf(str + "\n");
    

}

void getSonarDistance()
{

  if (mySerial.available()) {

    uint8_t c = mySerial.read();
    // Serial.println(c, HEX);

    // See if this is a header byte
    if (idx == 0 && c == 0xFF) {
      buffer[idx++] = c;
    }
    // Two middle bytes can be anything
    else if ((idx == 1) || (idx == 2)) {
      buffer[idx++] = c;
    }
    else if (idx == 3) {
      uint8_t sum = 0;
      sum = buffer[0] + buffer[1] + buffer[2];
      if (sum == c) {
        distance = ((uint16_t)buffer[1] << 8) | buffer[2];
        newData = true;
      }
      idx = 0;
    }
  }
  
  if (newData) {

    // Serial.printf("getSonarDistance NEW DATA\n");

    mDistance[count] = distance;
    count++;
    
    //Serial.print("Distance: ");
    //Serial.print(distance/25.4);
    //Serial.println(" ft");
    newData = false;
  }


  if (count == 100) {

    digitalWrite(led,HIGH);

    qsort(mDistance, 100, sizeof(mDistance[0]), sort_desc);

    disthigh = mDistance[5]/25.4;
		distlow = mDistance[95]/25.4;
    distmean = mDistance[50]/25.4;

    avg = average(mDistance,count)/25.4;
    // Serial.print("Average 100 Distance: ");
    // Serial.println(avg);
    count = 0;
    
    WaterLevelHigh=SENSORHEIGHTINFEET-((distlow)/12);
    WaterLevelLow=SENSORHEIGHTINFEET-((disthigh)/12);
		WaterLevelAvg=SENSORHEIGHTINFEET-((avg)/12);
    WaterLevelMean=SENSORHEIGHTINFEET-((distmean)/12);

    toInflux("WaterLevelAvg value=" + String(WaterLevelAvg));
    toInflux("WaterLevelHigh value=" + String(WaterLevelHigh));
    toInflux("WaterLevelLow value=" + String(WaterLevelLow));
    toInflux("WaterLevelMean value=" + String(WaterLevelMean));
    toInflux("DistanceToWaterCM value=" + String(mDistance[50]));
    toInflux("DistanceToWaterFt value=" + String(avg/12));
    
    getCELL();
    getFUEL();
    getBME680();
   
  }
  
}

void getFUEL()
{

  
  toInflux("BORON-getSoC value=" + String(fuel.getSoC()));
  toInflux("BORON-getNormalizedSoC value=" + String(fuel.getNormalizedSoC()));


  toInflux("BORON-powerSource value=" + String(System.powerSource()));
  toInflux("BORON-uptime value=" + String(System.uptime()));
  toInflux("BORON-batterycharge value=" + String(System.batteryCharge()));
  toInflux("BORON-batteryState value=" + String(System.batteryState()));
  
}
void getCELL()
{

  sig = Cellular.RSSI();
  toInflux("BORON-signalstrength value=" + String(sig.getStrength()));
  toInflux("BORON-signalquality value=" + String(sig.getQuality()));

}

void getBME680()
{

    if (! bme.performReading()) {
    Particle.publish("Log", "Failed to perform reading of BME :(");
    Serial.print("Failed to perform reading on BME: ");
  } else {
    temperatureInC = bme.temperature;
    relativeHumidity = bme.humidity;
    pressureHpa = bme.pressure / 100.0;
    gasResistanceKOhms = bme.gas_resistance / 1000.0;
    approxAltitudeInM = bme.readAltitude(SEALEVELPRESSURE_HPA);

    String data = String::format(
      "{"
        "\"temperatureInC\":%.2f,"
        "\"humidityPercentage\":%.2f,"
        "\"pressureHpa\":%.2f,"
        "\"gasResistanceKOhms\":%.2f"
        "\"approxAltitudeInM\":%.2f"
      "}",
      temperatureInC,
      relativeHumidity,
      pressureHpa,
      gasResistanceKOhms,
      approxAltitudeInM);

    toInflux("BME680-temperature value=" + String((temperatureInC * 9/5) + 32));
    toInflux("BME680-humidity value=" + String(relativeHumidity));
    toInflux("BME680-pressure value=" + String(pressureHpa));
    toInflux("BME680-gas value=" + String(gasResistanceKOhms));
    toInflux("BME680-altitude value=" + String(approxAltitudeInM));

  }
}


// loop() runs over and over again, as quickly as it can execute.

void loop() {
  // The core of your code will likely live here.
 
  
  getSonarDistance();

  digitalWrite(led,LOW);
}

