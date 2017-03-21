// Author: Min Jae Kim


//------------------------------------------------------------------------
// Include the ESP8266 library.
#include <ESP8266WiFi.h>

//------------------------------------------------------------------------
// Include the I2C library.
#include <Wire.h>

//------------------------------------------------------------------------
// Popular Arduino Library for MQTT handeling.
#include <PubSubClient.h>

//------------------------------------------------------------------------
// Include Adafruit Sensor and DHT library for the DHT11 Temperature
// and Humidity sensor.
#include <Adafruit_Sensor.h>
#include <DHT.h>

//------------------------------------------------------------------------
// Define Wi-Fi name to connect to and password.
// Please do not remember my Wi-Fi password...
#define ssid                 "MJK"
#define password             "A12086477198990"

//------------------------------------------------------------------------
// Define MQTT Broker (Server) hosted by Adafruit.
#define mqtt_server          "io.adafruit.com"
#define mqtt_server_port     1883

//------------------------------------------------------------------------
// Define user ID and key for account of adafruit MQTT Broker.
// Obtained by logging into Adafruit IO account.
#define mqtt_user            "david1196"
#define mqtt_key             "d93411ba0f224d25991fc1d8d9ae8cbc"

//------------------------------------------------------------------------
// Define the I2C address of the ADXL345 accelerometer.
// 0x1D is the address defined in the data sheet of ADXL345.
#define accel_module (0x1D)

//------------------------------------------------------------------------
// Values holds the data from ADXL345. datasheet of ADXL345 states that
// each x, y, and z axis outputs 2 bytes of data each, thus 6 bytes total.
byte values[6] ;

//------------------------------------------------------------------------
// String output. Used to output data to the serial port.
char output[512];

//------------------------------------------------------------------------
// Temporary data holder where it stores the magnitude of previous data
// value and subtract it to the current value. This will allow users to
// see if the value changed (final output != 0) or stayed the same (final
// output = 0).
int temporary = 0;

//------------------------------------------------------------------------
// Define MQTT feed address on the Adafruit IO server.
// Address: david1196/feeds/adxl345-data.
#define USERNAME             "david1196/"
#define PREAMBLE             "feeds/"
#define T_ACCELERATION       "adxl345-data"

//------------------------------------------------------------------------
// Define MQTT feed to subscribe to commands and status.
#define T_CLIENTSTATUS       "clientStatus"
#define T_COMMAND            "command"

//------------------------------------------------------------------------
// pin12 connected to the data pin on the DHT11 sensor.
#define pin12 12
DHT dht1(pin12, DHT11);
#define T_TEMPERATURE          "tempData"
#define T_HUMIDITY             "humidData"

//------------------------------------------------------------------------
// Variable declaration.
unsigned long entry;
byte clientStatus, prevClientStatus = 99;

float acceleration;
float temperature;
float humidity;

char valueStr[5];
char valueStr2[5];
char valueStr3[5];

//------------------------------------------------------------------------
// Create MQTT object.
WiFiClient WiFiClient;
PubSubClient client(WiFiClient);










void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Wire.begin();                            // Initiate I2C
  Serial.begin(115200);                    // Begin the serial connection with Baud of 115200

//------------------------------------------------------------------------
// According to the data sheet of the ADXL345, 0x2D register is power
// control register. Controls which power state the sensor is at.
// eg. Link, Auto_Sleep, Measure, Sleep, and Wakeup. We write 0 into
// the register 0x2D to clear it and reset it.
  Wire.beginTransmission(accel_module);
  Wire.write(0x2D);
  Wire.write(0);
  Wire.endTransmission();

//------------------------------------------------------------------------
// Begin another transmission with the same module (ADXL345). We write
// in the same register (0x2D). We write 16 to the register. Note this
// is same as binary value 0010000. This selects D3 (from D0, D1, ..., D7)
// which corresponds to Measure mode.
  Wire.beginTransmission(accel_module);
  Wire.write(0x2D);
  Wire.write(16);
  Wire.endTransmission();

//------------------------------------------------------------------------
// We also write 8 (0001000) to the same register. According to the data
// sheet, this is Auto_Sleep (D4) and will prevent it from going to sleep
// allowing us to get our data back. If this is not done, the sensor will
// measure but not transmit the data back as it will go into sleep.
  Wire.beginTransmission(accel_module);
  Wire.write(0x2D);
  Wire.write(8);
  Wire.endTransmission();

//------------------------------------------------------------------------
// Initiate the DHT11 Sensor.
  dht1.begin();

//------------------------------------------------------------------------
// 
  Serial.begin(115200);     // Check if I really need this (there's the same line above) @@@@@@@@@@@@@@@@@@@@@@@@@@@
  delay(100);

//------------------------------------------------------------------------
// Print some info about the status of Wi-Fi connection on Serial Monitor.
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

//------------------------------------------------------------------------
// Connect to the Wi-Fi with SSID and Password given.
  WiFi.begin(ssid, password);

//------------------------------------------------------------------------
// While not connected to the Wi-Fi, print "..." on the serial monitor to
// show that it is still waiting for connection. The red LED on the board
// will blink until the connection to the Wi-Fi is established.
  while (WiFi.status() != WL_CONNECTED) {
  digitalWrite(LED_BUILTIN, LOW);   // Turn the LED on (Note that LOW is the voltage level
                                    // but actually the LED is on; this is because 
                                    // it is acive low on the ESP-01)
  delay(1000);                      // Wait for a second
  digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED off by making the voltage HIGH
  delay(2000);                      // Wait for two seconds (to demonstrate the active low LED)

    Serial.print(".");
  }

//------------------------------------------------------------------------
// Print some info about the status of Wi-Fi connection on Serial Monitor.
// Print the IP address.
  Serial.println("");
  digitalWrite(LED_BUILTIN, LOW );
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  WiFi.printDiag(Serial);

//------------------------------------------------------------------------
// Define the MQTT Broker with the address and the port number. 
  client.setServer(mqtt_server, mqtt_server_port);
  client.setCallback(callback);
  
}


void loop() {
  
//------------------------------------------------------------------------
// Read DHT temperature and humidity values
//  float DHT11_t = dht1.readTemperature();
//  float DHT11_h = dht1.readHumidity();

//------------------------------------------------------------------------
// Implemented inside the ESP8266 Library. ESP8266 runs a lot of utility
// functions in the background, such as keeping the Wi-Fi connected,
// managing IP/TCP stack, etc. Without this line, ESP8266 may block these
// critical functions while running our functions. Yielding fixes this
// issue, avoiding resets and crashes.
  yield();

//------------------------------------------------------------------------
// Connecting to the MQTT Broker
  if (!client.connected()) {
    Serial.println("Attempting MQTT connection...");
    //Attempt to connect using the username and key of the MQTT Broker.
    if (client.connect("AccelTempSense", mqtt_user, mqtt_key)) {
      Serial.println("connected to MQTT Broker");
      // subscribe to a topic.
      client.subscribe(USERNAME PREAMBLE T_COMMAND, 1);
      client.subscribe(USERNAME PREAMBLE "test", 1);
    }
    else {
      // if failed to connect, client.state and try to connect in 5 sec.
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }

//------------------------------------------------------------------------
// Measure The temperature and acceleration. Only measure if the latest
// measurement has been done more than 1 second ago. This limits the the
// frequency of the number of measurements. Limit must be here as the
// MQTT Broker (Adafruit) states it will limit the number of publishes
// to 2 publishes per second.
  if (millis() - entry > 1000) {
    Serial.println("Measure");
    // Measure temperature.
    temperature = dht1.readTemperature();
    humidity = dht1.readHumidity();
    // Continuously print the data from DHT11 to the serial monitor every
    // 500 ms.
    Serial.print("DHT11  ");
    Serial.print(temperature,1); Serial.print(String(char(176))+"C  ");
    Serial.print(humidity,1); Serial.println("%RH");
    Serial.println();
    // Set current acceleration and output acceleration data separately.
//    int current, output;
//    // Current magnitude of g data.
//    // (Refer to movement() method below)
//    current = movement();
//
//    // ADXL345 represents the X, Y ,Z axis g data in a raw number
//    // between 0 to ~6000. The acceleration can be represented by the
//    // difference between previous acceleration value and the current
//    // value, but when the orientation is changed so that it jumps from
//    // 6000 to 0, it is represented as a big acceleration, while it
//    // may just have been a slight change. Example: if max value is 6000,
//    // 6000 to 0 is same acceleration as 2 to 3.
//    // To overcome this, if the current value is over 5000 and the previous
//    // value was less than 30, or vice versa, it will not register as an
//    // acceleration.
//    if (temporary < 30 and current > 5000)
//    {
//      acceleration = 0;
//    }
//    else if (current < 30 and temporary > 5000)
//    {
//      acceleration = 0;
//    }
//    else
//    {
//    acceleration = current - temporary;
//    temporary = current;
//      // I found that over 100 is a significant movement. This caps the
//      // acceleration value at 99 and -99. This gives a better
//      // representation on the graph. Also, since the values are required
//      // to be sent by string char (according to the MQTT Broker library),
//      // -99 is smaller in size than -100.
//      if (acceleration > 99) {
//        acceleration = 99;
//      }
//      else if (acceleration < -99) {
//        acceleration = -99;
//      }
//    }
    // Returns the number of milliseconds since the Arduino board began
    // running the current program.
    entry = millis();
  }

//------------------------------------------------------------------------
// If connected to the MQTT Broker, publish data.
  if (client.connected()) { 
    for (int j = 1; j <= 10; j++)
    {
      int current, output;
    // Current magnitude of g data.
    // (Refer to movement() method below)
    current = movement();

    // ADXL345 represents the X, Y ,Z axis g data in a raw number
    // between 0 to ~6000. The acceleration can be represented by the
    // difference between previous acceleration value and the current
    // value, but when the orientation is changed so that it jumps from
    // 6000 to 0, it is represented as a big acceleration, while it
    // may just have been a slight change. Example: if max value is 6000,
    // 6000 to 0 is same acceleration as 2 to 3.
    // To overcome this, if the current value is over 5000 and the previous
    // value was less than 30, or vice versa, it will not register as an
    // acceleration.
    if (temporary < 30 and current > 5000)
    {
      acceleration = 0;
    }
    else if (current < 30 and temporary > 5000)
    {
      acceleration = 0;
    }
    else
    {
    acceleration = current - temporary;
    temporary = current;
      // I found that over 100 is a significant movement. This caps the
      // acceleration value at 99 and -99. This gives a better
      // representation on the graph. Also, since the values are required
      // to be sent by string char (according to the MQTT Broker library),
      // -99 is smaller in size than -100.
      if (acceleration > 99) {
        acceleration = 99;
      }
      else if (acceleration < -99) {
        acceleration = -99;
      }
      
    }
    
      // Convert data (number) into the format required by the
      // library (string).
      Serial.println("Publish Acceleration");
      String hi = (String)acceleration;
      hi.toCharArray(valueStr, 4);
      // Publish acceleration data to the Broker.
      // Publish at address david1196/feeds/adxl345-data.
      client.publish(USERNAME PREAMBLE T_ACCELERATION, valueStr);
      // Delay required to meet the MQTT Broker specification.
      // (1 publish per second, 1 for temp, 1 for accel)
      delay(1000);
    }
    
    Serial.println("Publish Temperature");
    // Convert data (number) into the format required by the
    // library (string).
    String te = (String)temperature;
    te.toCharArray(valueStr2, 5);
    // Publish temperature data to the Broker.
    // Publish at address david1196/feeds/tempData.
    client.publish(USERNAME PREAMBLE T_TEMPERATURE, valueStr2);
    // Delay required to meet the MQTT Broker specification.
    // (1 publish per second, 1 for temp, 1 for accel)
    delay(1000);
    
    // Convert data (number) into the format required by the
    // library (string).
    Serial.println("Publish Humidity");
    String by = (String)humidity;
    by.toCharArray(valueStr3, 5);
    // Publish acceleration data to the Broker.
    // Publish at address david1196/feeds/adxl345-data.
    client.publish(USERNAME PREAMBLE T_HUMIDITY, valueStr3);
    // Delay required to meet the MQTT Broker specification.
    // (1 publish per second, 1 for temp, 1 for accel)
    delay(1000);
  }

  if (client.connected()&& prevClientStatus != clientStatus ) {
    Serial.println("Publish Status");
    String hi = (String)clientStatus;
    String te = (String)clientStatus;
    String by = (String)clientStatus;
    hi.toCharArray(valueStr, 5);
    te.toCharArray(valueStr2, 5);
    by.toCharArray(valueStr3, 5);
    client.publish(USERNAME PREAMBLE T_CLIENTSTATUS, valueStr);
    client.publish(USERNAME PREAMBLE T_CLIENTSTATUS, valueStr2);
    client.publish(USERNAME PREAMBLE T_CLIENTSTATUS, valueStr3);
    prevClientStatus = clientStatus;
  }
  client.loop();
}

void callback(char* topic, byte * data, unsigned int length) {
  //  handle message arrived {
  Serial.print(topic);
  Serial.print(": ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)data[i]);
  }
  Serial.println();
  if (data[1] == 'F')  {
    clientStatus = 0;
  } else {
    clientStatus = 1;
  }
}

int movement()
{
//------------------------------------------------------------------------
// xyz register is given the register address of 0x32. This is also
// determined by the data sheet of the ADXL345. Each X, Y, Z data are
// transmitted using 2 bytes of data. This means the total data size of
// X, Y, Z axis data is 6 bytes. The data sheet indicates that X-Axis
// Data 0 is DATAX0, X-Axis Data 1 is DATAX1, Y-Axis Data 0 is DATAY0,
// and so on. DATAX0 is located at register 0x32, DATAX1 at 0x33, and
// DATAZ1 is located at 0x37.
  int xyzregister = 0x32;
  int x, y, z;

//------------------------------------------------------------------------
// We connect with the ADXL module and wire to 0x32 on the register.
// We then read 6 bytes from 0x32 (thus, from 0x32 to 0x37).
  Wire.beginTransmission(accel_module);
  Wire.write(xyzregister);
  Wire.endTransmission();
  Wire.beginTransmission(accel_module);
  Wire.requestFrom(accel_module, 6);
  
//------------------------------------------------------------------------
// This while loop allows the each bit of the 6 bytes of data to be
// stored on the values variable.
  int i = 0;
  while(Wire.available()){
    values[i] = Wire.read();
    i++;
  }

//------------------------------------------------------------------------
// We now divide up that 6 bytes of data stored in the values variable
// into x, y, and z variable.
  Wire.endTransmission();
  x = (((int)values[1]) << 8) | values[0];
  y = (((int)values[3]) << 8) | values[2];
  z = (((int)values[5]) << 8) | values[4];
  Serial.write(10);
  delay(100);
  
//------------------------------------------------------------------------
// we return the magnitude of the x, y, z value by appling a simple
// 3D pythagorean theorem.
  return sqrt(pow(x,2)+pow(y,2)+pow(z,2));
}

