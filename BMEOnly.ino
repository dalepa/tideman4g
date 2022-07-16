//BME680
/*
 * Project BME680
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


String myID;

//TIME
unsigned long startMillis;  //some global variables available anywhere in the program
unsigned long currentMillis;
const unsigned long period = 1000*60*60;  //1 hour
const unsigned long HOUR = 1000*60*60;  //1 hour
float startWaterLevel = 0;
float waterRise;




bool newData = false; // Whether new data is available from the sensor
uint8_t buffer[4];  // our buffer for storing data
uint8_t idx = 0;  // our idx into the storage buffer



// UDP


UDP udp;
int resp;
size_t bufferSize;
float totalBufferLen = 0.0;
float LoopCycleDuration = 0.0;

IPAddress loggingIP(18,191,146,167);    //* 72.14.127.113
unsigned int loggingPort = 8089;
//char loggingBuffer[] = "cpu value=777";
String loggingBuffer;



//BME680
#include "Adafruit_BME680.h"

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define SEALEVELPRESSURE_HPA (1022.25)

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

  myID = System.deviceID();

  Serial.println("Hello World " + myID);



  pinMode(led, OUTPUT);


    if (!bme.begin()) {
    Particle.publish("Log", "Could not find a valid BME680 sensor, check wiring!");
  } else {
    Particle.publish("Log", "bme.begin() ID=" + myID);
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

    startMillis = millis(); 
    currentMillis = millis();

  }

  //SONAR
  //mySerial.begin(9600);
  
  //Serial.println("Begin 9600 OK");
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

float average (float * array, int len)  // assuming array is int.
{
  long sum = 0L ;  // sum will be larger than an item, long for safety.
  for (int i = 0 ; i < len ; i++)
    sum += array [i] ;
  return  ((float) sum) / len ;  // average will be fractional, so float may be appropriate.
}

void toInflux(String str, String strvalue)
{
    resp = udp.begin(loggingPort);

    loggingBuffer=str + ",PID=" + myID + " value=" + strvalue;
 
    bufferSize = strlen(loggingBuffer);
    totalBufferLen +=  bufferSize;

    resp = udp.sendPacket(loggingBuffer, bufferSize, loggingIP, loggingPort);
    
    udp.stop();

    Serial.printf(str + "\n");
    

}


void getFUEL()
{

  
  toInflux("BORON-getSoC", String(fuel.getSoC()));
  toInflux("BORON-getNormalizedSoC", String(fuel.getNormalizedSoC()));


  toInflux("BORON-powerSource", String(System.powerSource()));
  toInflux("BORON-uptime", String(System.uptime()));
  toInflux("BORON-batterycharge", String(System.batteryCharge()));
  toInflux("BORON-batteryState", String(System.batteryState()));
  
}
void getCELL()
{

  sig = Cellular.RSSI();
  toInflux("BORON-signalstrength", String(sig.getStrength()));
  toInflux("BORON-signalquality", String(sig.getQuality()));

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

    toInflux("BME680-temperature", String((temperatureInC * 9/5) + 32));
    toInflux("BME680-humidity", String(relativeHumidity));
    toInflux("BME680-pressure", String(pressureHpa));
    toInflux("BME680-gas", String(gasResistanceKOhms));
    toInflux("BME680-altitude", String(approxAltitudeInM));

  }
}


// loop() runs over and over again, as quickly as it can execute.

void loop() {
  // The core of your code will likely live here.
 
  
    getCELL();
    getFUEL();
    getBME680();
	  


    LoopCycleDuration = (millis() - currentMillis) / 1000.0;

    toInflux("BoronLoopTimeMsec",String(LoopCycleDuration )); 
    

    // Calculate Total Bytes Sent
    toInflux("BORON-BPM", String((totalBufferLen/LoopCycleDuration) * 60.0 * 60.0 * 24.0 * 30.0));
    toInflux("BORON-BufLen", String(totalBufferLen));
    
    
    totalBufferLen = 0.0;
    currentMillis = millis();

    delay(1000*30);

    digitalWrite(led,LOW);
}

