/*
    Setup Webserver & OTA
    Obtain Sensor data: 2 pins (12 & 14)
    Motor control through pin 5.
    Manual & automatic motor mode
    Check pin 12 for >80'C -> Alert mode
    Store original Motor state
    Diff pin 14 over 12 > 8'C -> Motor start
    Diff pin 14 over 12 < 2'C -> Motor stop
    Multiple SSID support
    if (Motor state changed, or last save > 1min ago){
     Store measurements and motor state and millis
     update last save
    }

    TODO:
    More Secure SSID storage, in FS.
    Moving webpage to separate static file, using JS to fetch data from REST-like output.

*/
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>
#include <FS.h>
#include <OneWire.h>

const int CONVERSIONTIME = 1000;
const int MEASUREMENTINTERVAL = 10000;
const int ALERTINTERVAL = 300000;
const int LOGINTERVAL = 60000;
const int BOILERPIN = 12;
const int COLLECTORPIN = 14;
const int MOTORPIN = 5;

const float MAXTEMP = 80.0;
const float DELTA_ON = 8.0;
const float DELTA_OFF = 2.0;

const char *logFile = "/sensorlog.csv";
const char *logCntType = "application/csv";

const char *ssid;
const char *password;
const char* ssid1 = "Secret_net1";
const char* password1 = "Secret_net1";

const char* ssid2 = "Secret_net2";
const char* password2 = "Secret_net2";

ESP8266WebServer server(80);
ESP8266HTTPUpdateServer httpUpdater;

OneWire  boiler(BOILERPIN);
OneWire  collector(COLLECTORPIN);

unsigned long lastMeasureStart = 0L;
unsigned long lastMeasurement  = 0L;
unsigned long lastLog  = 0L;
unsigned long prevMillis = 0L;
float btemp = 0.0;
float ctemp = 0.0;
bool motorRunning = false;
bool manualMotor = false;
bool overheated = false;

const String closePage = String("<br><br>(Page closes in 2 seconds!)<br><script type='text/javascript'>setTimeout(function ( ){self.close();}, 2000 );</script>");
String lastError = "";
      
void motorOn(bool manual) {
  if (manual || !manualMotor) {
    if (manual) manualMotor = true;
    digitalWrite(MOTORPIN, HIGH);
    motorRunning = true;
  }
}
void motorOff(bool manual) {
  if (manual || !manualMotor) {
    if (manual) manualMotor = true;
    digitalWrite(MOTORPIN, LOW);
    motorRunning = false;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(BOILERPIN, INPUT);
  pinMode(COLLECTORPIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(MOTORPIN, OUTPUT);
  motorOff(false);

  SPIFFS.begin();

  //Checking multiple SSIDs squentially.
  ssid = ssid1;
  password = password1;

  if (String(WiFi.SSID()) != String(ssid)) {
    Serial.printf("Connecting to %s\n", ssid);
    WiFi.begin(ssid, password);
  }
  while (WiFi.waitForConnectResult() != WL_CONNECTED && millis() < 60000) {
    if (ssid == ssid2) {
      ssid = ssid1;
      password = password1;
    } else {
      ssid = ssid2;
      password = password2;
    }
    Serial.printf("WiFi failed, retrying, Connecting to %s\n", ssid);
    WiFi.begin(ssid, password);
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("Failed to connect to WiFI");
    digitalWrite(LED_BUILTIN, LOW);  //Red LED is reverse level: LOW is on, HIGH is off.
  } else {
    Serial.println("");
    Serial.print("Connected! IP address: ");
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println(WiFi.localIP());

    server.on("/", HTTP_GET,  []() {
      String btStr = String(btemp, 2);
      String ctStr = String(ctemp, 2);
      String timeSinceStart = String(millis(), DEC);

      String txt = String("<html><head><title>Solar Boiler overview</title></head><body><h1>Solar Boiler overview</h1><br/>");
      txt = String(txt + "Hi there, I'm working okay! (for: " + timeSinceStart + " ms) boiler:<b>" + btStr + " 'C </b> collector:<b>" + ctStr + " 'C</b>");
      txt = String(txt + "Motor: " + (motorRunning ? "running" : "stopped") + " " + (manualMotor ? "(Manual control)" : "(Automatic control)") + " <br/>");
      txt = String(txt + "(last error:"+lastError+")<br>");
      txt = String(txt + "<br><a href='/update' target='_blank'>Update page</a><br/><hr><h2>CSV measurement file</h2>");
      txt = String(txt + "<br><a href='/csv' target='_blank'>Download CSV</a><br/><br/>");
      txt = String(txt + "<form action='/remove' target='_blank' method='post'><button type='submit'>Remove CSV</button></form><br/>");

      FSInfo fs_info;
      SPIFFS.info(fs_info);
      txt = String(txt + "Filesystem size: " + fs_info.totalBytes + " bytes<br>");
      txt = String(txt + "Filesystem used: " + fs_info.usedBytes + " bytes<br>");

      txt = String(txt + "<hr><h2>Motor controls</h2><br/>");
      txt = String(txt + "<form action='/on' target='_blank' method='post'><button type='submit'>Motor On</button></form><br/>");
      txt = String(txt + "<form action='/off' target='_blank' method='post'><button type='submit'>Motor Off</button></form><br/>");
      txt = String(txt + "<form action='/auto' target='_blank' method='post'><button type='submit'>Automatic Control</button></form>");

      txt = String(txt + "</body></html>");
      server.send(200, "text/html", txt);
      txt = String();
    });

    server.on("/on", HTTP_POST, []() {
      motorOn(true);
      server.send(200, "text/html", String("<html><body>Motor command done, manual override. Access /auto for returning to automatic control." + closePage + "</body></html>"));
    });

    server.on("/off", HTTP_POST, []() {
      motorOff(true);
      server.send(200, "text/html", String("<html><body>Motor command done, manual override. Access /auto for returning to automatic control." + closePage + "</body></html>"));
    });

    server.on("/auto", HTTP_POST, []() {
      manualMotor = false;
      overheated = false;
      server.send(200, "text/html", String("<html><body>Motor back in automatic control.!" + closePage + "</body></html>"));
    });

    server.on("/csv", HTTP_GET, [] {
      int unitSize = 1024;
      lastError = "";
      if (SPIFFS.exists(logFile)) {
        File file = SPIFFS.open(logFile, "r");
        server.setContentLength(file.size());
        server.send(200, logCntType, "");

        WiFiClient client = server.client();
        std::unique_ptr<uint8_t[]> buffer(new uint8_t[unitSize]);
        size_t size_sent = 0;

        while (true) {
          size_t left = file.available();
          if (!left) {
            lastError = String(lastError+" | file.available -> false triggered");
            break;
          }
          size_t will_send = (left < unitSize) ? left : unitSize;
          file.read(buffer.get(), will_send);
          size_t cb = client.write(buffer.get(), will_send);
          size_sent += cb;
          if (cb != will_send) {
            break;
          }
        }
        lastError = String(lastError + " | size_sent:" + size_sent);
        //server.streamFile(file, logCntType);
        file.close();
      } else {
        server.send(404, "text/plain", "File not Found");
      }
    });

    server.on("/remove", HTTP_POST, [] {
      SPIFFS.remove(logFile);
      server.send(200, "text/html", String("<html><body>Logfile removed!" + closePage + "</body></html>"));
    });

    server.on("/format", HTTP_POST, [] {
      SPIFFS.format();
      Serial.println("Spiffs formatted");
      server.send(200, "text/html", String("<html><body>Spiffs filesystem format running, might take up to 30s!" + closePage + "</body></html>"));
    });

    httpUpdater.setup(&server);
    server.begin();
    Serial.println("HTTP server started");
  }
}

bool startMeasurement(OneWire sensor, byte *addr) {
  bool found = false;
  if ( sensor.search(addr)) {
    found = true;
  }
  if (!found) {
    sensor.reset_search();
  } else {
    if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return false;  //Early return to get sensors back online!
    }
    if (addr[0] != 0x28 ) {
      Serial.println("Unknown sensor type found on one of the Pins!");
      return false;  //Early return to get sensors back online!
    }
    sensor.reset();
    sensor.select(addr);
    sensor.write(0x44);
  }
  return found;
}

float convert(byte *data) {
  int16_t raw = (data[1] << 8) | data[0];

  byte cfg = (data[4] & 0x60);
  // at lower res, the low bits are undefined, so let's zero them
  if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
  else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
  else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
  //// default is 12 bit resolution, 750 ms conversion time
  return (float)raw / 16.0;
}

bool getMeasurementData(OneWire sensor, byte *addr, float *temp) {
  byte i;
  byte data[12];

  sensor.reset();
  sensor.select(addr);
  sensor.write(0xBE);

  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = sensor.read();
  }

  if (OneWire::crc8(data, 8) != data[8]) {
    Serial.print("Data CRC is not valid: ");
    for ( i = 0; i < 9; i++) {           // we need 9 bytes
      Serial.print(data[i], HEX);
    }
    Serial.print(" CRC=");
    Serial.print(OneWire::crc8(data, 8), HEX);
    Serial.println();
    return false;  //Early return to get sensors back online!
  }
  *temp = convert(data);
  return true;
}

void doLogging(long timestamp) {
  File f = SPIFFS.open(logFile, "a");
  if (f) {
    String strTimestamp = String(timestamp);
    String strBtemp = String(btemp);
    String strCtemp = String(ctemp);
    String strMotorRunning = String(motorRunning);
    String strManualMotor = String(manualMotor);
    String txt = String(strTimestamp + "," + strBtemp + "," + strCtemp + "," + strMotorRunning + "," + strManualMotor);
    f.println(txt);
  }
  f.close();
}

void loop() {
  bool bfound = false;
  bool cfound = false;
  byte baddr[8];
  byte caddr[8];

  server.handleClient();

  //Measure temperatures
  unsigned long timestamp = millis();
  if (timestamp < prevMillis) {
    //millisRollover!
    if (lastMeasureStart > 0) {
      //make sure running measurement is done:
      delay(CONVERSIONTIME);
      timestamp = millis();
    }
    lastMeasureStart = 0L;
    lastMeasurement = 0L;
    lastLog = 0L;
  }
  prevMillis = timestamp;

  if (lastMeasureStart == 0 && timestamp - MEASUREMENTINTERVAL >= lastMeasurement) {
    //Start new measurement
    bfound = startMeasurement(boiler, baddr);
    cfound = startMeasurement(collector, caddr);

    if (!bfound || !cfound) {
      delay(250);
      return;  //Early return to get sensors back online!
    }
    lastMeasureStart = timestamp;
  }
  if (lastMeasureStart > 0 && timestamp - CONVERSIONTIME >= lastMeasureStart) {
    //Get values from thermometers, feedback through global variables.
    if (getMeasurementData(boiler, baddr, &btemp) && getMeasurementData(collector, caddr, &ctemp)) {
      Serial.print(timestamp);
      Serial.print(": boiler: ");
      Serial.print(btemp);
      Serial.print(" 'C collector: ");
      Serial.print(ctemp);
      Serial.println(" 'C");
      lastMeasurement = timestamp;
    } else {
      Serial.println("Failed to obtain temperature from one of the sensors");
    }
    lastMeasureStart = 0L;
  }
  if (timestamp == lastMeasurement) {
    bool originalMotor = motorRunning;
    if (btemp > MAXTEMP) {
      overheated = true;
      motorOff(true);
    }
    if (ctemp - btemp > DELTA_ON) {
      motorOn(false);
    }
    if (ctemp - btemp < DELTA_OFF) {
      motorOff(false);
    }
    if (originalMotor != motorRunning) {
      //Switched, doLogging();
      doLogging(timestamp);
    }
  }
  if (lastLog == 0 || timestamp - LOGINTERVAL >= lastLog) {
    doLogging(timestamp);
    lastLog = timestamp;
  }
  //if lastMeasurement is too long ago, switch off motor to manual
  if (motorRunning && !manualMotor && timestamp - ALERTINTERVAL >= lastMeasurement){
    motorOff(true);
    lastError = String("Missing measurements since:" + lastMeasurement);
  }
  //Power saving delay, you'll notice this delay on the webserver side....
  delay(250);
}
