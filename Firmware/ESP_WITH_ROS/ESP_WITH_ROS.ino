
/*
  Firmware For ESP Wifi Controller
  Repeatedly listens on UDP socket over Wifi to obtain commands from ROS
  Firmware Version v0.5
*/

/*
   This intend to connect to a Wifi Access Point
   and a rosserial socket server.
   You can launch the rosserial socket server with
   roslaunch rosserial_server socket.launch
   The default port is 11411

*/

/*
   Command
   $roslaunch rosserial_server socket.launch       (terminal 1)
   $rostopic list                                  (terminal 2)
   $rostopic echo chatter                          (terminal 3)
*/


/*
   Differnt between Example of Esp8266 and Esp32
  ESP8266 --> #include <ESP8266WiFi.h>
  EPS32 ----> #include <WiFi.h>
*/

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ros.h>
#include <std_msgs/String.h>

/*
   Preprocessor declarations
*/
#ifndef STASSID
//#define STASSID "TP-Link_2902"
//#define STAPSK  "66619222"

#define STASSID "TP-Link_040B"
#define STAPSK  "67572571"

//#define STASSID "That's What She SSID"
//#define STAPSK  "password"


//#define STASSID "JioFi_246085D"
//#define STAPSK  "zeus1234"



//#define STASSID "D-Link_DIR-816"
//#define STAPSK  "INSPIREDAFT1234"
#endif

//Pin Definitions
#define ESP_ENABLE 13

//#define BLUE_BOTS 5
#define TURQUOISE_BOTS 5

#define MAX_BOTS_PER_TEAM 12
#define DEBUG 1

/*
   Socket Variable definitions

*/

int count = 0;
#define dataFrameSize 12
unsigned long timer1 = 0;
byte data_struct[dataFrameSize];
String _BOT_ID[MAX_BOTS_PER_TEAM] = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11"} ;


#ifdef BLUE_BOTS
String temp = "/gyroDataBlue" + _BOT_ID[BLUE_BOTS];
const char* subscribeTopic = "/velocityDataBlue";
const char* publishTopic = temp.c_str();
#endif

#ifdef TURQUOISE_BOTS
String temp = "/gyroDataYellow" + _BOT_ID[TURQUOISE_BOTS];
const char* subscribeTopic = "/velocityDataYellow";
const char* publishTopic = temp.c_str();
#endif




/************ Set the rosserial socket server IP address****************/

// uncomment whichever IP is necessary for testing
IPAddress server(192, 168, 0, 177); //depaanshu ip 
//IPAddress server(192, 168, 0, 140);//shubam ip
//IPAddress server(192, 168, 0,101); //apna PC IP


// Set the rosserial socket server port
const uint16_t serverPort = 11411;

ros::NodeHandle nh;
// Make a chatter publisher
std_msgs::String str_msg;




//Serial Variables
#define COMMAND_SIZE 150
char command[COMMAND_SIZE];
int serial_count;
int no_data = 0;
long idle_time;
bool bytes_received = false;

//Callback funtion for subscriber

void chatterCallback(const std_msgs::String& msg) {
  String velData = msg.data;
//  Serial.println(velData.length());
  parse_socket_data(velData, velData.length());
}

void chatterCallback1(const std_msgs::String& msg) {
  //Serial.print("Callback Data : ");
  //Serial.println(msg.data);
}

ros::Publisher pub(publishTopic, &str_msg);

ros::Subscriber<std_msgs::String> sub(subscribeTopic, &chatterCallback);
//ros::Subscriber<std_msgs::String> sub1("/gyroData", &chatterCallback1);


void setup()
{
  // put your setup code here, to run once:

  Serial.begin(115200);
  //Serial.println();
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ESP_ENABLE, INPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);


  //  Serial.print("Connecting to ");
  //  Serial.println(STASSID);

  // Connect the ESP8266 the the wifi AP
  WiFi.begin(STASSID, STAPSK);
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_BUILTIN, LOW);
    delay(150);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(150);
  }
  //  Serial.println(".......");
  //  Serial.println("WiFi connected");
  //  Serial.println("IP address: ");
  //  Serial.println(WiFi.localIP());

  // Set the connection to rosserial socket server
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();

  // Another way to get IP
  //  Serial.print("IP = ");
  //  Serial.println(nh.getHardware()->getLocalIP());

  // Start to be polite
  nh.advertise(pub);
  nh.subscribe(sub);
  //nh.subscribe(sub1);

  while (!(digitalRead(ESP_ENABLE)))
  {
    delay(100);
  }
  digitalWrite(LED_BUILTIN,LOW);

}

void loop()
{
  // put your main code here, to run repeatedly:

  //read_serial();

  //Check if Wifi connection is lost
  if (WiFi.status() != WL_CONNECTED)
  {
    digitalWrite(LED_BUILTIN, HIGH);

    //if yes connect to Wifi again
    while (WiFi.status() != WL_CONNECTED)
    {
      //Serial.print('.');
      digitalWrite(LED_BUILTIN, LOW);
      delay(150);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(150);
    }
    digitalWrite(LED_BUILTIN, LOW);
  }

  nh.spinOnce();
  delay(10);

}

/*
   Function to parse data obtained from UDP socket and encode it into byte format
   Parameters --> data_frame[] -- array of data obtained in char/string format
              --> arraySize
*/
void parse_socket_data(String data_frame, int arraySize)
{
  int i = 0;
  String botID = "";
  String dummy = "";

#ifdef BLUE_BOTS
  dummy = _BOT_ID[BLUE_BOTS];
#endif

#ifdef TURQUOISE_BOTS
  dummy = _BOT_ID[TURQUOISE_BOTS];
#endif

  while (data_frame[i] != ',')
  {
    botID += data_frame[i];
    i++;
  }
  i++;


  if (botID == dummy)
  {
    //Serial.println("yes");

    //int i = 2;
    int j = 0;
    uint16_t angle = 0;
    for (byte k = 0; k < dataFrameSize; k++)
    {
      data_struct[k] = 0;
    }
    while (data_frame[i] != ';')
    {

      if (data_frame[i] == ',')j++;
      else if (j < 10)
      {
        data_struct[j] = data_struct[j] * 10 + (data_frame[i] - '0');
        if (data_struct[j] < 1)data_struct[j] = 2;
      }
      else if (j == 10)
      {
        angle = angle * 10 + (data_frame[i] - '0');
      }
      i++;
    }

    if (angle > 255) {
      data_struct[11] = angle - 255;
      data_struct[10] = 255;
    }
    else if (angle < 1)
    {
      data_struct[11] = 1;
      data_struct[10] = 1;
    }
    else
    {
      data_struct[11] = 1;
      data_struct[10] = (byte)angle;
    }

    //Send data to main controller
    for (byte k = 0; k < dataFrameSize; k++)
    {
      Serial.write(data_struct[k]);
    }
    Serial.write(';');
  }
}

void read_serial()
{
  if (Serial.available() > 0)
  {
    byte c = Serial.read();
    if (c == ';')
    {
      no_data++;
    }
    else
    {
      command[serial_count] = c;
      serial_count++;
      no_data = 0;
      bytes_received = true;
      idle_time = millis();
    }
  }
  //mark no data if nothing heard for 400 milliseconds
  else
  {
    if ((millis() - idle_time) >= 100)
    {
      no_data++;
      idle_time = millis();
    }
  }

  //if theres a pause or we got a real command, do it
  if (bytes_received && no_data )
  {
    process(command, serial_count);
    init_process_string();
  }
}

void init_process_string()
{
  //init our command
  for (byte i = 0; i < COMMAND_SIZE; i++) command[i] = 0;
  serial_count = 0;
  bytes_received = false;
  idle_time = millis();
}
void process(char instruction[], int arraySize)
{
 // digitalWrite(LED_BUILTIN, LOW);
  str_msg.data = instruction;
  pub.publish( &str_msg );
 // digitalWrite(LED_BUILTIN, HIGH);
}
/*
   Not used currently
*/
//void send_data(String data)
//{
//  byte data1[data.length()];
//  for (byte i = 0; i < data.length(); i++)
//  {
//    data1[i] = (byte)data[i];
//  }
//  parse_socket_data(data1, data.length());
//}
