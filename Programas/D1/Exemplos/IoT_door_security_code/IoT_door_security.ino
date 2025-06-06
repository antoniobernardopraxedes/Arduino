#include <ESP8266WiFi.h>
#include <FirebaseESP8266.h>

#define FIREBASE_HOST "xxxxxxxxxxxxxxxx.firebaseio.com"   // Enter firebase DB URL without "https:// "  and "/" at the end of URL             
#define FIREBASE_AUTH "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"   // Real-time database secret key here

#define WIFI_SSID "WIFI SSID" // WIFI SSID here                                   
#define WIFI_PASSWORD "WIFI password"  // WIFI password here                  

// Declare the Firebase Data object in the global scope
FirebaseData firebaseData1;
FirebaseData firebaseData2;

//bool alarm_mode = 0;
int sensor = 5; //GPIO5 D1 Pin number on which sensor is connected
int buzzer = 15; //GPIO15 D8 Pin number on which buzzer is connected
int alarm_led_on = 4; //GPIO4 D2 Pin number on which alarm status led is connected
int alarm_led_off = 0; //GPIO0 D3 Pin number on which alarm status led is connected
int sensor_data;
String alarm_data;

void setup() 
{
  pinMode(sensor, INPUT_PULLUP);
  pinMode(buzzer, OUTPUT);
  pinMode(alarm_led_on, OUTPUT);
  pinMode(alarm_led_off, OUTPUT);

  Serial.begin(115200);
  Serial.println("Serial communication started\n");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);  //connect with wifi
  Serial.print("Connecting to ");
  Serial.print(WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) 
  {
    Serial.print(".");
    delay(100);
  }
  Serial.println();
  Serial.print("Connected to ");
  Serial.println(WIFI_SSID);
  Serial.print("IP Address is : ");
  Serial.println(WiFi.localIP());  //print local IP address
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);   // connect to firebase
  Firebase.reconnectWiFi(true);
  delay(1000);
}

void loop() 
{ 
  // Reading alarm data from firebase
   Firebase.getString(firebaseData2, "/door_sensor/alarm_data");
   alarm_data = firebaseData2.stringData(); 
   //Serial.print("ALARM Status:");
   //Serial.println(alarm_data);
   delay(100);
   
 if(digitalRead(sensor) == 0 && alarm_data == "0")
  {
   Serial.println("Door is Closed Alarm mode OFF");
   sensor_data = 0;
   Firebase.setInt(firebaseData1, "/door_sensor/door_status/", sensor_data);
      digitalWrite(alarm_led_on, LOW); 
      digitalWrite(alarm_led_off, HIGH);
      digitalWrite(buzzer, LOW);
  }
  else if(digitalRead(sensor) == 1 && alarm_data == "0")
  {
    //Serial.println("Door is Open Alarm mode OFF");
    sensor_data = 1;
    Firebase.setInt(firebaseData1, "/door_sensor/door_status/", sensor_data);
      digitalWrite(alarm_led_on, LOW); 
      digitalWrite(alarm_led_off, HIGH);
      digitalWrite(buzzer, LOW);
  }
  
  else if(digitalRead(sensor) == 0 && alarm_data == "1")
    {
      //Serial.println("Door is Closed Alarm mode ON");
      sensor_data = 0;
      Firebase.setInt(firebaseData1, "/door_sensor/door_status/", sensor_data);
      digitalWrite(alarm_led_on, HIGH); 
      digitalWrite(alarm_led_off, LOW);
      digitalWrite(buzzer, LOW);
    }
    
  else if(digitalRead(sensor) == 1 && alarm_data == "1")
    {
      sensor_data = 1;
      Firebase.setInt(firebaseData1, "/door_sensor/door_status/", sensor_data);
      //Serial.println("Door is Open Alarm mode ON");
      digitalWrite(alarm_led_on, HIGH); 
      digitalWrite(alarm_led_off, LOW); 
      digitalWrite(buzzer, HIGH); 
      delay(50);
      digitalWrite(buzzer, LOW);
  }
}
