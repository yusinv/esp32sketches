#include <WiFi.h>
#include <ArduinoOTA.h>
#include <driver/pcnt.h>
#include <PubSubClient.h>
#include <BlueDot_BME280.h>
#include <RCSwitch.h>

#define WIFI_SSID "yusin.me"
#define WIFI_PASSWORD "qwertyuiopasdfghjkl"

#define HOST_NAME "water_counter"

#define OTA_PASSWORD "Goni_Linx1"

#define GPIO_COLD_PULSE_PCNT 25
#define GPIO_HOT_PULSE_PCNT 35
#define GPIO_LIGHT_CNTL_OUT 13
#define GPIO_LIGHT_CNTL_IN  32
#define GPIO_FAN_CNTL_OUT 27
#define GPIO_BATHROOM_LIGHT_IN 16
#define GPIO_TOILET_LIGHT_IN 17

#define COLD_WATER_PCNT_UNIT PCNT_UNIT_0
#define HOT_WATER_PCNT_UNIT PCNT_UNIT_3

#define BME_PRESSURE_CORRECTION 0.0 // pressure correction

#define MQTT_SERVER "nas"
#define MQTT_USER ""
#define MQTT_PASSWORD ""

#define COLD_WATER_CNT_TOPIC "home/utility/water/cold"
#define HOT_WATER_CNT_TOPIC "home/utility/water/hot"
#define AVAILABILITY_TOPIC "home/utility/water/available"
#define LIVINGROOM_LIGHT_A_COMMAND_TOPIC "home/livingroom/light/a/set"
#define LIVINGROOM_LIGHT_B_COMMAND_TOPIC "home/livingroom/light/b/set"
#define LIVINGROOM_LIGHT_C_COMMAND_TOPIC "home/livingroom/light/c/set"
#define LIVINGROOM_LIGHT_A_STATE_TOPIC "home/livingroom/light/a"
#define LIVINGROOM_LIGHT_B_STATE_TOPIC "home/livingroom/light/b"
#define LIVINGROOM_LIGHT_C_STATE_TOPIC "home/livingroom/light/c"

#define BEDROOM_LIGHT_A_COMMAND_TOPIC "home/bedroom/light/a/set"
#define BEDROOM_LIGHT_B_COMMAND_TOPIC "home/bedroom/light/b/set"
#define BEDROOM_LIGHT_C_COMMAND_TOPIC "home/bedroom/light/c/set"
#define BEDROOM_LIGHT_A_STATE_TOPIC "home/bedroom/light/a"
#define BEDROOM_LIGHT_B_STATE_TOPIC "home/bedroom/light/b"
#define BEDROOM_LIGHT_C_STATE_TOPIC "home/bedroom/light/c"

#define BATHROOM_FAN_COMMAND_TOPIC "home/bathroom/fan/set"
#define BATHROOM_FAN_STATE_TOPIC "home/bathroom/fan"

#define BATHROOM_TEMPERATURE_TOPIC "home/bathroom/temperature"
#define BATHROOM_HUMIDITY_TOPIC "home/bathroom/humidity"
#define BATHROOM_PRESSURE_TOPIC "home/bathroom/pressure"

#define BATHROOM_LIGHT_STATE_TOPIC "home/bathroom/light"
#define TOILET_LIGHT_STATE_TOPIC "home/toilet/light"

#define PAYLOAD_AVAILABLE "online"
#define PAYLOAD_NOT_AVAILABLE "offline"

#define LIGHT_LIVINGROOM_A 12931075 //0xC55003  0000 0011
#define LIGHT_LIVINGROOM_B 12931264 //0xC550C0  1100 0000
#define LIGHT_LIVINGROOM_C 12931120 //0xC55030  0011 0000
#define LIGHT_LIVINGROOM_D 12931312 //0xC550F0  1111 0000
#define LIGHT_LIVINGROOM_E 12931084 //0xC5500C  0000 1100
#define LIGHT_LIVINGROOM_F 12931267 //0xC550C3  1100 0011

#define LIGHT_BEDROOM_A 13045507 //0xC70F03
#define LIGHT_BEDROOM_B 13045696 //0xC70FC0
#define LIGHT_BEDROOM_C 13045552
#define LIGHT_BEDROOM_D 13045744
#define LIGHT_BEDROOM_E 13045516
#define LIGHT_BEDROOM_F 13045699

long waterUpdateTime = 0;
long lightUpdateTime = 0;
long bmeUpdateTime = 0;
long rcLastRecivedTime = 0;
unsigned long rcLastRecivedCommand = 0;


bool lightState[2][4];
int  lightCommand[2][6] = {
  {LIGHT_LIVINGROOM_A,
   LIGHT_LIVINGROOM_B,
   LIGHT_LIVINGROOM_C,
   LIGHT_LIVINGROOM_D,
   LIGHT_LIVINGROOM_E,
   LIGHT_LIVINGROOM_F},
  {LIGHT_BEDROOM_A, 
   LIGHT_BEDROOM_B, 
   LIGHT_BEDROOM_C,
   LIGHT_BEDROOM_D,
   LIGHT_BEDROOM_E,
   LIGHT_BEDROOM_F}
};

bool bathroomLightState = false;
bool toiletLightState = false;

bool bmeSenosorConnected = false;

const char* lightStateTopic[2][3] = {
  {LIVINGROOM_LIGHT_A_STATE_TOPIC,LIVINGROOM_LIGHT_B_STATE_TOPIC,LIVINGROOM_LIGHT_C_STATE_TOPIC},
  {BEDROOM_LIGHT_A_STATE_TOPIC,BEDROOM_LIGHT_B_STATE_TOPIC,BEDROOM_LIGHT_C_STATE_TOPIC}
};

const char* lightCommandTopic[2][3] = {
  {LIVINGROOM_LIGHT_A_COMMAND_TOPIC,LIVINGROOM_LIGHT_B_COMMAND_TOPIC,LIVINGROOM_LIGHT_C_COMMAND_TOPIC},
  {BEDROOM_LIGHT_A_COMMAND_TOPIC,BEDROOM_LIGHT_B_COMMAND_TOPIC,BEDROOM_LIGHT_C_COMMAND_TOPIC}
};

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
RCSwitch rcSwitch = RCSwitch();
BlueDot_BME280 bme; // I2C

void setup() {
  Serial.begin(115200);
  Serial.println("Booting");
  setupWifi();
  setupOTA();
  setupCounters();
  setupMQTT();
  setupRC();
  if (!mqttClient.connected()) {
    reconnect();
  }
  setupBME();

  //bath fan relay
  pinMode(GPIO_FAN_CNTL_OUT,OUTPUT);
  digitalWrite(GPIO_FAN_CNTL_OUT,HIGH);

  //bath light detector
  pinMode(GPIO_BATHROOM_LIGHT_IN,INPUT);
  pinMode(GPIO_TOILET_LIGHT_IN,INPUT);
}

void loop() {

  if (!mqttClient.connected()) {
    reconnect();
  }
  mqttClient.loop();
  
  ArduinoOTA.handle();
  
  long now = millis();
  
  if (now - waterUpdateTime > 60000) {
    waterUpdateTime = now;
    waterCntStateUpdate();  
  }

  if (bmeSenosorConnected && (now - bmeUpdateTime > 5000)) {
    bmeUpdateTime = now;
    bmeMeasurements();  
  }

  if (now - lightUpdateTime > 500) {
    lightUpdateTime = now;
    lightStateUpdate();

   
    int bathLightValue = digitalRead(GPIO_BATHROOM_LIGHT_IN);
    //change from low to high
    if(bathLightValue==HIGH && !bathroomLightState){
      mqttClient.publish(BATHROOM_LIGHT_STATE_TOPIC, "1");
      bathroomLightState = true;   
    }

    //change from high low
    if(bathLightValue==LOW && bathroomLightState){
      mqttClient.publish(BATHROOM_LIGHT_STATE_TOPIC, "0");
      bathroomLightState = false;   
    }

    int toiletLightValue = digitalRead(GPIO_TOILET_LIGHT_IN);
    //change from low to high
    if(toiletLightValue==HIGH && !toiletLightState){
      mqttClient.publish(TOILET_LIGHT_STATE_TOPIC, "1");
      toiletLightState = true;   
    }

    //change from high low
    if(toiletLightValue==LOW && toiletLightState){
      mqttClient.publish(TOILET_LIGHT_STATE_TOPIC, "0");
      toiletLightState = false;   
    }
  }

  if (rcSwitch.available()) {
    lightExternalStateUpdate();
  }
}

void lightExternalStateUpdate()
{
  unsigned long recivedValue = rcSwitch.getReceivedValue();
  long now = millis();

  Serial.print("Received ");
  Serial.print( recivedValue );
  Serial.print(" / ");
  Serial.print( rcSwitch.getReceivedBitlength() );
  Serial.print("bit ");
  Serial.print("Protocol: ");
  Serial.println( rcSwitch.getReceivedProtocol() );

  if(now-rcLastRecivedTime<250 && recivedValue == rcLastRecivedCommand){
    rcLastRecivedTime = now;
    Serial.println( "Ignored" );
    rcSwitch.resetAvailable(); 
    return;
  }

  rcLastRecivedTime = now;
  rcLastRecivedCommand = recivedValue;
  
  //all off
  for(int i=0;i<2;i++){
    if(recivedValue==lightCommand[i][5]){
      for(int j=1;j<4;j++){
          lightState[i][j]=false;
      } 
    }
  }

  //all on
  for(int i=0;i<2;i++){
    if(recivedValue==lightCommand[i][4]){
      for(int j=1;j<4;j++){
          lightState[i][j]=true;
        } 
    }
  }

  for(int i=0;i<2;i++){
    for(int j=0;j<3;j++){
      if(recivedValue==lightCommand[i][j]){
        lightState[i][j+1]=!lightState[i][j+1];     
      }
    }
  }

  for(int i=0;i<2;i++){
    for(int j=0;j<3;j++){
        mqttClient.publish(lightStateTopic[i][j], lightState[i][j+1]?"1":"0");  
      } 
  }

  rcSwitch.resetAvailable(); 
}

void waterCntStateUpdate(){
  int16_t value = 0;
  
  mqttClient.publish(AVAILABILITY_TOPIC, PAYLOAD_AVAILABLE);
  
  ESP_ERROR_CHECK(pcnt_get_counter_value(COLD_WATER_PCNT_UNIT,&value));
  ESP_ERROR_CHECK(pcnt_counter_clear(COLD_WATER_PCNT_UNIT));
  Serial.print("Cold water Counter: ");
  Serial.println(value);
  mqttClient.publish(COLD_WATER_CNT_TOPIC, String(value*10).c_str());

  value = 0;

  ESP_ERROR_CHECK(pcnt_get_counter_value(HOT_WATER_PCNT_UNIT,&value));
  ESP_ERROR_CHECK(pcnt_counter_clear(HOT_WATER_PCNT_UNIT));
  Serial.print("Hot water Counter: ");
  Serial.println(value);
  mqttClient.publish(HOT_WATER_CNT_TOPIC, String(value*10).c_str()); 
}

void lightStateUpdate(){
  int count=0;
  for(int i=0;i<2;i++){
    if(lightState[i][0]){
      for(int j=1;j<4;j++){
        if(lightState[i][j]){
          count++;
        }
      }
      Serial.print("Number of on lights: ");
      Serial.println(count);      
      if(count<2){
        //turn off all
        rcSwitch.send(lightCommand[i][5], 24);
        //turn on the rest
        for(int j=0;j<3;j++){
          if(lightState[i][j+1]){
            delay(25);
            rcSwitch.send(lightCommand[i][j], 24); 
          }
        }
      }
      else{
        //turn on all
        rcSwitch.send(lightCommand[i][4], 24);
        //turn off the rest
        for(int j=0;j<3;j++){
          if(!lightState[i][j+1]){
            delay(25);
            rcSwitch.send(lightCommand[i][j], 24); 
          }
        }
      }
      lightState[i][0]=false;
      for(int j=0;j<3;j++)
      {
        mqttClient.publish(lightStateTopic[i][j], lightState[i][j+1]?"1":"0");  
      }
    }
  }
}


void setupWifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.setHostname(HOST_NAME);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
   
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  
}

void setupOTA() {
  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  ArduinoOTA.setHostname(HOST_NAME);

  // No authentication by default
  ArduinoOTA.setPassword(OTA_PASSWORD);

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
  Serial.println("OTA Enabled");
}

void setupCounters()
{
    pinMode(GPIO_COLD_PULSE_PCNT,INPUT);
    pinMode(GPIO_HOT_PULSE_PCNT,INPUT);
  
    // set up counter
    pcnt_config_t cold_pcnt_config = {
        .pulse_gpio_num = GPIO_COLD_PULSE_PCNT,
        .ctrl_gpio_num = PCNT_PIN_NOT_USED,
        .lctrl_mode = PCNT_MODE_KEEP,
        .hctrl_mode = PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_INC,  // count both rising and falling edges
        .neg_mode = PCNT_COUNT_DIS,
        .counter_h_lim =  32767,
        .counter_l_lim = -1,
        .unit = COLD_WATER_PCNT_UNIT,
        .channel = PCNT_CHANNEL_1,
    };

    ESP_ERROR_CHECK(pcnt_unit_config(&cold_pcnt_config));    
    ESP_ERROR_CHECK(pcnt_set_filter_value(COLD_WATER_PCNT_UNIT,1023));
    ESP_ERROR_CHECK(pcnt_filter_enable(COLD_WATER_PCNT_UNIT));
    ESP_ERROR_CHECK(pcnt_counter_clear(COLD_WATER_PCNT_UNIT));
    Serial.println("Cold water PCNT Enabled");

    // set up counter
    pcnt_config_t hot_pcnt_config = {
        .pulse_gpio_num = GPIO_HOT_PULSE_PCNT,
        .ctrl_gpio_num = PCNT_PIN_NOT_USED,
        .lctrl_mode = PCNT_MODE_KEEP,
        .hctrl_mode = PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_INC,  // count both rising and falling edges
        .neg_mode = PCNT_COUNT_DIS,
        .counter_h_lim =  32767,
        .counter_l_lim = -1,
        .unit = HOT_WATER_PCNT_UNIT,
        .channel = PCNT_CHANNEL_0,
    };

    ESP_ERROR_CHECK(pcnt_unit_config(&hot_pcnt_config));    
    ESP_ERROR_CHECK(pcnt_set_filter_value(HOT_WATER_PCNT_UNIT,1023));
    ESP_ERROR_CHECK(pcnt_filter_enable(HOT_WATER_PCNT_UNIT));
    ESP_ERROR_CHECK(pcnt_counter_clear(HOT_WATER_PCNT_UNIT));
    Serial.println("Hot water PCNT Enabled");
}


void reconnect() {

  // Loop until we're reconnected
  while (!mqttClient.connected()) {

    Serial.print("WiFi Status:");
    Serial.println(WiFi.status());
    //some problems with esp32 reconnect
    if(WiFi.status()!=WL_CONNECTED){
      Serial.println("Reconnect WIFI");
      for(int i=0;i<30 && WiFi.status()!=WL_CONNECTED;i++){
        if(!WiFi.reconnect()){
          Serial.println("WiFI Reconnect Failed! retry in 5 sec...");
          delay(5000);
        }
        else{
          Serial.println("WiFI waiting for Reconnect...");
          WiFi.waitForConnectResult();
        }
        Serial.print("WiFi Status:");
        Serial.println(WiFi.status());        
      }
      if(WiFi.status()!=WL_CONNECTED){
        Serial.println("Connection Failed! Rebooting...");
        delay(5000);
        ESP.restart(); 
      }
    }
      
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (mqttClient.connect(clientId.c_str(),AVAILABILITY_TOPIC,1,true,PAYLOAD_NOT_AVAILABLE)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      mqttClient.publish(AVAILABILITY_TOPIC, PAYLOAD_AVAILABLE);
      // ... and resubscribe
      mqttClient.subscribe(LIVINGROOM_LIGHT_A_COMMAND_TOPIC);
      mqttClient.subscribe(LIVINGROOM_LIGHT_B_COMMAND_TOPIC);
      mqttClient.subscribe(LIVINGROOM_LIGHT_C_COMMAND_TOPIC);
      mqttClient.subscribe(BEDROOM_LIGHT_A_COMMAND_TOPIC);
      mqttClient.subscribe(BEDROOM_LIGHT_B_COMMAND_TOPIC);
      mqttClient.subscribe(BEDROOM_LIGHT_C_COMMAND_TOPIC);
      mqttClient.subscribe(BATHROOM_FAN_COMMAND_TOPIC);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setupMQTT()
{
  mqttClient.setServer(MQTT_SERVER, 1883);
  mqttClient.setCallback(mqtt_callback); 
}

void setupRC()
{
  //rc light in
  pinMode(GPIO_LIGHT_CNTL_IN,INPUT);
  
  rcSwitch.enableTransmit(GPIO_LIGHT_CNTL_OUT);
  rcSwitch.enableReceive(digitalPinToInterrupt(GPIO_LIGHT_CNTL_IN));
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  for(int i=0;i<2;i++)
  {
    for(int j=0;j<3;j++)
    {
      if (strcmp(lightCommandTopic[i][j],topic)==0){
        lightState[i][j+1] = (char)payload[0] == '1' /* on */;
        lightState[i][0] = true;
      } 
    }
  }

  if(strcmp(BATHROOM_FAN_COMMAND_TOPIC,topic)==0){
    if((char)payload[0] == '1' /* on */){
      digitalWrite(GPIO_FAN_CNTL_OUT,LOW);
      mqttClient.publish(BATHROOM_FAN_STATE_TOPIC, "1");   
    } else {
      digitalWrite(GPIO_FAN_CNTL_OUT,HIGH);
      mqttClient.publish(BATHROOM_FAN_STATE_TOPIC, "0");
    }
  }
  
}

void setupBME() {
  
  bme.parameter.communication = 0;                    //I2C communication for Sensor
  bme.parameter.I2CAddress = 0x76;                    //I2C Address for Sensor
  bme.parameter.sensorMode = 0b11;                    //Setup Sensor mode for Sensor 1
  bme.parameter.IIRfilter = 0b100;                   //IIR Filter for Sensor 1
  bme.parameter.humidOversampling = 0b101;            //Humidity Oversampling for Sensor 1
  bme.parameter.tempOversampling = 0b101;              //Temperature Oversampling for Sensor 1
  bme.parameter.pressOversampling = 0b101;             //Pressure Oversampling for Sensor 1
  bme.parameter.pressureSeaLevel = 1013.25;            //default value of 1013.25 hPa (Sensor 2)
  bme.parameter.tempOutsideCelsius = 15;               //default value of 15°C
  bme.parameter.tempOutsideFahrenheit = 59;            //default value of 59°F
  //mqttClient.publish("home/test", "satart sensor" );
  //start sensor
  if (bme.init() == 0x60)
  {    
    bmeSenosorConnected = true;
    //mqttClient.publish("home/test", "sensor connected" );
  }
  else
  {
    bme.parameter.I2CAddress = 0x76;
    if (bme.init() == 0x60)
    {    
      bmeSenosorConnected = true;
      //mqttClient.publish("home/test", "sensor connected2" ); 
    }
    //mqttClient.publish("home/test", "sensor not conneced" );
    //mqttClient.publish("home/test",  String(bme.init()).c_str());
  }
  //mqttClient.publish("home/test", "sensor init end" );
}

void bmeMeasurements(){
  
  float temp_c = bme.readTempC();
  float hum = bme.readHumidity();
  float baro = bme.readPressure();

  Serial.print("New temperature:");
  Serial.print(String(temp_c) + " degC   ");
  mqttClient.publish(BATHROOM_TEMPERATURE_TOPIC, String(temp_c).c_str(), true);

  Serial.print("New humidity:");
  Serial.println(String(hum) + " %");
  mqttClient.publish(BATHROOM_HUMIDITY_TOPIC, String(hum).c_str(), true);

  float baro_hpa=baro+BME_PRESSURE_CORRECTION; // hPa corrected to sea level
  float baro_mmhg=(baro+BME_PRESSURE_CORRECTION)*0.75006375541921F; // mmHg corrected to sea level
  Serial.print("New barometer:");
  Serial.print(String(baro_hpa) + " hPa   ");
  Serial.println(String(baro_mmhg) + " mmHg");
  mqttClient.publish(BATHROOM_PRESSURE_TOPIC, String(baro_mmhg).c_str(), true);
  
}
