
#define DEBUG 1

#define ESP Serial1
#define RASP_PI Serial2

/*****************BOT specific variables***************/
#define MAX_BOTS_PER_TEAM 12
//#define BLUE_BOTS 2
#define TURQUOISE_BOTS 5


/********************Hardware Specific Variables*************/

#define NO_OF_MOTORS 4
#define STARTUP_DELAY 2000

#define PWM_PIN_1A 3
#define PWM_PIN_1B 4
#define PWM_PIN_2A 20
#define PWM_PIN_2B 21
#define PWM_PIN_3A 22
#define PWM_PIN_3B 23
#define PWM_PIN_4A 5
#define PWM_PIN_4B 6

#define PWM_FREQUENCY 58593
#define ANALOG_RESOLUTION 10


int pwm_pin2[NO_OF_MOTORS] = {PWM_PIN_1A, PWM_PIN_2A, PWM_PIN_3A, PWM_PIN_4A}; //fwd direction control pins of all motors
int pwm_pin1[NO_OF_MOTORS] = {PWM_PIN_1B, PWM_PIN_2B, PWM_PIN_3B, PWM_PIN_4B}; //bwd direction control pins of all motors

#define ESP_TX 1
#define ESP_RX 0

#define RASP_PI_TX 10
#define RASP_PI_RX 9

#define TIM0 3
#define TIM1 5

#define INDICATION_LED 13

//Kicker pins
#define _LT3750_donePin 39
#define _LT3750_chargePin 38
#define _solenoidTriggerPin 36

//Dribbler pin
#define DRIBBLER_PIN 35

#define ESP_ENABLE_PIN 2



//Serial Variables for ESP communication
#define COMMAND_SIZE 150
byte _command[COMMAND_SIZE];
int serial_count;
int no_data = 0;
long idle_time;
bool bytes_received = false;


//Serial Variables for Rasp_pi communication
#define COMMAND_SIZE_1 150
char _command_1[COMMAND_SIZE_1];
int serial_count_1;
int no_data_1 = 0;
long idle_time_1;
bool bytes_received_1 = false;

unsigned long lastTime = 0;
uint16_t rot_angle = 0;
float current_angle = 0, prev_angle = 0, target_angle = 0;
bool rotate = false;


///Kicker Variables
bool _kick = false;
bool _kickerCharged = false;
bool _chargeTriggered = false;


int calibrationValuesBlue[MAX_BOTS_PER_TEAM][6] = {{ -3007 , 1809 , 1337 , 63 , 45 , 6}, //0
  { -2908 , 287, 595 , 132, 85 , -4}, //1
  {1464, -5093, 772, 128, -22 , 23}, //2
  { -899, 771 , 1571,  91,  15 , -17}, //3
  {185, 330 , 1204 , 21 , 30 , 26}, //4
  {-1236, -3308, 870, 92 , 50 , -5},//5
  {0, 0, 0, 0, 0, 0},//6
  {0, 0, 0, 0, 0, 0},//7
  {0, 0, 0, 0, 0, 0},//8
  { -3001, 1805 , 1313 , 68 , 26 , 9},//9
  {0, 0, 0, 0, 0, 0},//10
  {0, 0, 0, 0, 0, 0}//11
};


int calibrationValuesTurquoise[MAX_BOTS_PER_TEAM][6] = {{216, 166, 733 , 7 , -48, -32},
  { -967 , -969 , 1692 , 8 , 12  , -9},
  { -790 , -206 , 1820 , -1403 , 1266 , 5},
  { -4589, 2078, 1490, -35, 21, 75},
  { -2649 , 2214 , 1902 , 123 , 66 , 14},
  {-53, -505,  1914 , -58, 35 , 7},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0}
};


String rpiDisplayCommands[10] = {"D:sad", "D:cry", "D:normal", "D:confused", "D:angry"};
String rpiAudioCommands[10] = {"A:baba", "A:banana", "A:batbat", "A:ahaha", "A:doo", "A:go", "A:hehehe", "A:hello", "A:hmm", "A:laugh"};
int graphicsRollCount = 0;
int audioRollCount = 0;
unsigned long lastGraphicsUpdateTime = 0;
unsigned long lastROSUpdateTime = 0;
