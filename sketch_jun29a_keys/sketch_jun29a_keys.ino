//based on:
// Adafruit Adafruit_BME280_Library
// https://github.com/adafruit/Adafruit_BME280_Library
//and
// Astuder BMP085-template-library-Energia
// https://github.com/astuder/BMP085-template-library-Energia
//plus code for altitude and relative pressure
//by r7

#include <Wire.h>                                                       // required by BME280 library
#include <BME280_t.h>                                                   // import BME280 template library
#include "DHT.h"
#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
#include <FS.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>


#define ASCII_ESC 27

#define DHTPIN D2
#define DHTTYPE DHT22

// DHT instructions
// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

// BME instructions
// VIN - 5V
// GND - Ground
// SCL - D4
// SCA - D3

#define MYALTITUDE 80

char bufout[10];

BME280<> BMESensor;                                                     // instantiate sensor
DHT dht(DHTPIN, DHTTYPE);

#define PMS_PIN_SET D5
#define PMS_PIN_RX D7
#define PMS_PIN_TX D6
#define PMS_BAUDRATE 9600

// Data format
#define PMS_HEADER1 0 //0x42
#define PMS_HEADER2 1 // 0x4d
#define PMS_COMMAND1 2 // frame lenght high
#define PMS_COMMAND2 3 // frame lenght low
#define PMS_PM1C_HIGH 4
#define PMS_PM1C_LOW 5
#define PMS_PM25C_HIGH 6
#define PMS_PM25C_LOW 7
#define PMS_PM10C_HIGH 8
#define PMS_PM10C_LOW 9
#define PMS_PM1_HIGH 10
#define PMS_PM1_LOW 11
#define PMS_PM25_HIGH 12
#define PMS_PM25_LOW 13
#define PMS_PM10_HIGH 14
#define PMS_PM10_LOW 15
#define PMS_RES1_HIGH 16
#define PMS_RES1_LOW 17
#define PMS_RES2_HIGH 18
#define PMS_RES2_LOW 19
#define PMS_RES3_HIGH 20
#define PMS_RES3_LOW 21
#define PMS_CHECKSUM_HIGH 22
#define PMS_CHECKSUM_LOW 23

//define your default values here, if there are different values in config.json, they are overwritten.
char device_id[40] = "xxxxxxxxx"; //monitoring device ID, used for identification
char api_endpoint[129] = "nowytomysl.org"; //air monitoring API URL, default value
char upload_interval[10] = "0.5"; // how often readings should be uploaded

//flag for saving data
bool shouldSaveConfig = true;

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}


SoftwareSerial pms(PMS_PIN_RX, PMS_PIN_TX); // Initialize serial communiation with PMS 3003

#define pmsDataLen 24 // according to spec PMS3003 has 24 bytes long message
uint8_t buf[pmsDataLen];
int idx = 0;
long pm10 = 0;
long pm25 = 0;


void setup()
{
  Serial.begin(115200);                                                 // initialize serial
  Wire.begin(0,2);                                                      // initialize I2C that connects to sensor
  BMESensor.begin();                                                    // initalize bme280 sensor
  dht.begin();

//  WiFi.begin(ssid, password);
  
//  while (WiFi.status() != WL_CONNECTED) {
//    delay(500);
//    Serial.print(".");
//  }
//
//  Serial.println("");
//  Serial.println("WiFi connected");  
//  Serial.println("IP address: ");
//  Serial.println(WiFi.localIP());

  pms.begin(PMS_BAUDRATE); // PMS 3003 UART has baud rate 9600
  pinMode(PMS_PIN_SET, OUTPUT);
  digitalWrite(PMS_PIN_SET, LOW);
  Serial.println("PMS OFF");

  Serial.println("mounting FS...");

  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed json");

          strcpy(device_id, json["device_id"]);
          strcpy(api_endpoint, json["api_endpoint"]);
          strcpy(upload_interval, json["upload_interval"]);

        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }

  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom_device_id("device_id", "Device ID", device_id, 40);
  WiFiManagerParameter custom_api_endpoint("api_endpoint", "API endpoint", api_endpoint, 129);
  WiFiManagerParameter custom_upload_interval("upload_interval", "Upload interval [mins]", upload_interval, 10);

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //set static ip
  //wifiManager.setSTAStaticIPConfig(IPAddress(10,0,1,99), IPAddress(10,0,1,1), IPAddress(255,255,255,0));
  
  //add all your parameters here
  wifiManager.addParameter(&custom_device_id);
  wifiManager.addParameter(&custom_api_endpoint);
  wifiManager.addParameter(&custom_upload_interval);

  //reset settings - for testing
  //wifiManager.resetSettings();

  //set minimu quality of signal so it ignores AP's under that quality
  //defaults to 8%
  //wifiManager.setMinimumSignalQuality();
  
  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  //wifiManager.setTimeout(120);

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect("AutoConnectAP", "password")) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("Successfully connected to WiFi network");

  //read updated parameters
  strcpy(device_id, custom_device_id.getValue());
  strcpy(api_endpoint, custom_api_endpoint.getValue());

  //save the custom parameters to FS
  if (shouldSaveConfig) {
    Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["device_id"] = device_id;
    json["api_endpoint"] = api_endpoint;
    json["upload_interval"] = upload_interval;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }

    json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save
  }

  Serial.println("local ip");
  Serial.println(WiFi.localIP());

}


int i = 0;
void loop() {
  digitalWrite(PMS_PIN_SET, HIGH);
  Serial.println("PMS ON");
  delay(30 * 1000);

  BMESensor.refresh();                                                  // read current sensor data
  sprintf(bufout,"%c[1;0H",ASCII_ESC);
  Serial.print(bufout);

  Serial.println("");
  Serial.println("------------------------------------------------------");   
  Serial.println("------------------------------------------------------");   
  Serial.println("------------------------------------------------------");   
  Serial.println("");
  
  Serial.println("");
  Serial.println("--------------------------BME----------------------------");   
  Serial.println("");

  Serial.print("Temperature: ");
  Serial.print(BMESensor.temperature);                                  // display temperature in Celsius
  Serial.print("C ");

  Serial.print("Humidity: ");
  Serial.print(BMESensor.humidity);                                     // display humidity in %   
  Serial.print("% ");

  Serial.print("Pressure: ");
  Serial.print(BMESensor.pressure  / 100.0F);                           // display pressure in hPa
  Serial.print("hPa ");

  float relativepressure = BMESensor.seaLevelForAltitude(MYALTITUDE);
  Serial.print("RelPress: ");
  Serial.print(relativepressure  / 100.0F);                             // display relative pressure in hPa for given altitude
  Serial.print("hPa ");   

  Serial.print("Altitude: ");
  Serial.print(BMESensor.pressureToAltitude(relativepressure));         // display altitude in m for given pressure
  Serial.print("m");

  Serial.println("");
  Serial.println("--------------------------DHT----------------------------");   
  Serial.println("");
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.print(" %\t");
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.print(" *C ");


  Serial.println("");
  Serial.println("--------------------------PMS----------------------------");   
  Serial.println("");


  uint8_t c = 0;
  idx = 0;
  memset(buf, 0, pmsDataLen);

  while (true) {
    while (c != 0x42) {
      while (!pms.available());
      c = pms.read();
    }
    while (!pms.available());
    c = pms.read();
    if (c == 0x4d) {
      // now we got a correct header)
      buf[idx++] = 0x42;
      buf[idx++] = 0x4d;
      break;
    }
  }

  while (idx != pmsDataLen) {
    while(!pms.available());
    buf[idx++] = pms.read();
  }
  long pms_checksum = word(buf[PMS_CHECKSUM_HIGH], buf[PMS_CHECKSUM_LOW]);
  long pms_calc_checksum = buf[PMS_HEADER1] + buf[PMS_HEADER2] + 
                            buf[PMS_COMMAND1] + buf[PMS_COMMAND2] + 
                            buf[PMS_PM1C_HIGH] + buf[PMS_PM1C_LOW] + 
                            buf[PMS_PM25C_HIGH] + buf[PMS_PM25C_LOW] + 
                            buf[PMS_PM10C_HIGH] + buf[PMS_PM10C_LOW] + 
                            buf[PMS_PM1_HIGH] + buf[PMS_PM1_LOW] + 
                            buf[PMS_PM25_HIGH] + buf[PMS_PM25_LOW] + 
                            buf[PMS_PM10_HIGH] + buf[PMS_PM10_LOW] + 
                            buf[PMS_RES1_HIGH] + buf[PMS_RES1_LOW] + 
                            buf[PMS_RES2_HIGH] + buf[PMS_RES2_LOW] +
                            buf[PMS_RES3_HIGH] + buf[PMS_RES3_LOW];


  if (pms_calc_checksum == pms_checksum) {
    Serial.print("Checksum correct: ");
    Serial.print(pms_checksum);
    Serial.print(" / ");
    Serial.println(pms_calc_checksum);
  }
  else {
    Serial.print("Checksum incorrect: ");
    Serial.print(pms_checksum);
    Serial.print(" / ");
    Serial.println(pms_calc_checksum);
    Serial.println("Raw data dump:");
    for (int i=0; i< pmsDataLen;i++) {
      Serial.print(buf[i]);
      Serial.print(" ");
    }
    Serial.println();
    }

  pm25 = word(buf[12], buf[13]);
  pm10 = word(buf[14], buf[15]);
 
  Serial.print("pm2.5: ");
  Serial.print(pm25);
  Serial.print(" pm10: ");
  Serial.println(pm10);

  delay(1 * 1000);
  digitalWrite(PMS_PIN_SET, LOW);
  Serial.println("PMS OFF");

  Serial.println("");
  Serial.println("--------------------------Wyslanie wynikow----------------------------");   
  Serial.println("");
  
  WiFiClient client;
  const int httpPort = 80;
  if (!client.connect(api_endpoint, httpPort)) {
    Serial.println("connection failed");
    return;
  }
  
  // URL z wynikami
  String url = "/upload-v2.php";
  url += "?device_key=";
  url += device_id;
  url += "&hum_dht=";
  url += h;
  url += "&temp_dht=";
  url += t;
  url += "&hum=";
  url += BMESensor.humidity;
  url += "&temp=";
  url += BMESensor.temperature;
  url += "&press=";
  url += BMESensor.pressure  / 100.0F;
  url += "&pm25=";
  url += pm25;
  url += "&pm10=";
  url += pm10;
  url += "&rssi=";
  url += WiFi.RSSI();
  url += "&dev_timestamp=";
  url += i;
  
  Serial.print("Requesting URL: ");
  Serial.println(url);
  
  // This will send the request to the server
  client.print(String("GET ") + url + " HTTP/1.1\r\n" +
               "Host: " + api_endpoint + "\r\n" + 
               "Connection: close\r\n\r\n");
  unsigned long timeout = millis();
  while (client.available() == 0) {
    if (millis() - timeout > 5000) {
      Serial.println(">>> Client Timeout !");
      client.stop();
      return;
    }
  }
  
  // Read all the lines of the reply from server and print them to Serial
  while(client.available()){
    String line = client.readStringUntil('\r');
    Serial.print(line);
  }
  
  Serial.println();
  Serial.println("closing connection");
  Serial.println(WiFi.RSSI());

  // zwiekszenie wewnetrzengo iteratora
  i++;

  float minutes_upload_interval = atof(upload_interval);
  // delay przed nastepna petla
  delay(minutes_upload_interval * 60 * 1000);
}
