
#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <SocketIoClient.h>
#define USE_SERIAL Serial

#define DEBUG

const char* creds = "{\"id\": \"85ec15c2-74f6-11eb-a296-08f1ea94b84e\", \"password\": \"85ec1474-74f6-11eb-a296-08f1ea94b84e\", \"auth_type\": \"login\"}";  ///credential user ID+Passward //////
const char* creds_id = "85ec15c2-74f6-11eb-a296-08f1ea94b84e";

String score = "{\"id\": \"85ec15c2-74f6-11eb-a296-08f1ea94b84e\",\"val\":\"[";

ESP8266WiFiMulti WiFiMulti;
SocketIoClient webSocket;

//Serial Variables
#define COMMAND_SIZE 150
char command[COMMAND_SIZE];
int serial_count;
int no_data = 0;
long idle_time;
bool bytes_received = false;


String serverMessage = "";

void event(const char * payload, size_t length) {
  //USE_SERIAL.printf("=================> Got message: %s\n", payload);

  serverMessage = String(payload);
  DynamicJsonBuffer jsonBuffer(200);

  JsonObject& response = jsonBuffer.parseObject(serverMessage);      //////Parse Data recieved from server///////////////////////////////////

  if (!response.success())
  {
#ifdef DEBUG
    Serial.print("parseObject(");
    Serial.println(") failed");
    return;
#endif
  }

  String command = String(response["event_data"]["command"].as<char*>());

#ifdef DEBUG
  Serial.println("parseObject() Success---->" );
  Serial.println(command);
  Serial.println("<<<<---------------------->>>>");
#endif

  if (command == "start")
  {


    Serial.println("start");                    ///////Robot start Command receive from server ///////////////////////////////////////////


  }

  else if (command == "stop") /////////////Stop Command recive rom server ////////////////////////////////////////////////
  {

    Serial.println("stop");

  }

  webSocket.emit("device-events", serverMessage.c_str());
}
void connect(const char * payload, size_t length) {
#ifdef DEBUG
  Serial.println("=================> Connection triggered");
#endif
  webSocket.emit("authenticate", creds);

}
void disconnect(const char * payload, size_t length) {
#ifdef DEBUG
  Serial.println("===============> Disconnected!!!");
#endif
}
void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
#ifdef DEBUG
  Serial.print("ESP_WEBSOCKET_CONNECTION");
#endif
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  WiFi.begin("Robotics-Gallery", "Cube@321");         ////////Wifi User Id +passward (Connected server //////////////////




  while (WiFi.status() != WL_CONNECTED)
  {
    digitalWrite(LED_BUILTIN, LOW);
    delay(150);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(150);
  }

#ifdef DEBUG
  Serial.print("Wifi COnnected");
  Serial.println("");
  Serial.println("WiFi connection Successful");
  Serial.print("The IP Address of ESP8266 Module is: ");
  Serial.print(WiFi.localIP());// Print the IP address
#endif

  webSocket.begin("10.10.1.17", 3000, "/socket.io/?transport=websocket");          ///////////////////Server IP Address/////////////////////////
  webSocket.on(creds_id, event);                     ////////////Event ID////////////////////////////////////
  webSocket.on("connect", connect);
  webSocket.on("disconnect", disconnect);

  digitalWrite(LED_BUILTIN, LOW);
}

void loop()
{

  read_serial();
  webSocket.loop();                    /////Websocket loop Executation ///////////////////////////////////////////////////////

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
    Serial.println(command);
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
  if (instruction[0] == 's')
  {
    if (instruction == "stop")
    {
#ifdef DEBUG
      Serial.println("stop received");
#endif

      webSocket.emit("device-events", serverMessage.c_str());
    }
    //webSocket.emit("score-event",
  }
  else if (instruction[0] == 'g')
  {

    char blueGoal = instruction[5];
    char yellowGoal = instruction[7];

    String dataToBePublished = score + String(instruction[5]) + "," + String(instruction[7]) + "]\"}";

#ifdef DEBUG
    Serial.println(dataToBePublished);
#endif
    webSocket.emit("score-event", dataToBePublished.c_str());

  }
}
