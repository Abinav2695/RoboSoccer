void _serial_init()
{
  ESP.setTX(ESP_TX);
  ESP.setRX(ESP_RX);
  ESP.begin(115200);

  RASP_PI.setTX(RASP_PI_TX);
  RASP_PI.setRX(RASP_PI_RX);
  RASP_PI.begin(115200);

}

void esp_data()
{
  if (ESP.available() > 0)
  {
    byte c = ESP.read();
    if (c == ';')
    {
      no_data++;
    }
    else
    {
      _command[serial_count] = c;
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
#ifdef DEBUG
  for(int i=0;i<serial_count;i++)
  {
    Serial.print(_command[i]);
  }
   Serial.println();
#endif
    process(_command, serial_count);
    init_process_string();
  }
}

void init_process_string()
{
  //init our command
  for (byte i = 0; i < COMMAND_SIZE; i++) _command[i] = 0;
  serial_count = 0;
  bytes_received = false;
  idle_time = millis();
}
void process(byte instruction[], int arraySize)
{
  // Data structure
  //[pwm1,pwm2,pwm3,pwm4,dir1,dir2,dir3,dir4,kick,dribble,angle(High byte),angle(low byte)]
  // Since we cannot send byte 0 in serial we use byte 2 instead of 0
  byte pwmOfWheels[4] = {0, 0, 0, 0};
  int pwmOfWheels_modified[4] = {0, 0, 0, 0};
  byte dirOfWheels[4] = {0, 0, 0, 0};
  byte kick = 0;
  byte dribble = 0;
  rot_angle = 0;


  pwmOfWheels[0] = instruction[0];
  pwmOfWheels[1] = instruction[1];
  pwmOfWheels[2] = instruction[2];
  pwmOfWheels[3] = instruction[3];
  dirOfWheels[0] = instruction[4];
  dirOfWheels[1] = instruction[5];
  dirOfWheels[2] = instruction[6];
  dirOfWheels[3] = instruction[7];
  kick = instruction[8];
  dribble = instruction[9];

  for (byte k = 0; k < 4; k++)
  {
    if (pwmOfWheels[k] < 5)pwmOfWheels[k] = 0;
    pwmOfWheels_modified[k] = map(pwmOfWheels[k], 0, 255, 0, 1023);
#ifdef DEBUG
    //Serial.println(pwmOfWheels_modified[k]);
#endif
  }

  if (instruction[11] == 1)
  {
    if (instruction[10] == 1) rot_angle = 0;
    else rot_angle = instruction[10];
  }
  else rot_angle = instruction[10] + instruction[11];

  ///check for kicker input
  if (kick == 1)_kick = true;
  else _kick = false;


  ///check for dribbler input
  if (dribble == 1)_pin_write(DRIBBLER_PIN, HIGH);
  else _pin_write(DRIBBLER_PIN, LOW);

  if (rot_angle > 0)
  {
    if (dirOfWheels[0] == 2)
    {
      target_angle = current_angle + float(rot_angle);
      if (target_angle >= 180)target_angle -= 360;
    }
    else
    {
      target_angle = current_angle - float(rot_angle);
      if (target_angle <= -180)target_angle += 360;
    }
#ifdef DEBUG
    Serial.print("Rotate Angle  : ");
    Serial.print(rot_angle);
    Serial.print("Current Angle : ");
    Serial.print(current_angle);
    Serial.print("   Target Angle : ");
    Serial.println(target_angle);
#endif
    rotate = true;
    //current_angle = 0;
  }

  _update_pwm_value(dirOfWheels, pwmOfWheels_modified);
}





///Raspberry Pi data processing


void rasp_pi_data()
{
  if (RASP_PI.available() > 0)
  {
    char c = RASP_PI.read();
    if (c == ';')
    {
      no_data_1++;
    }
    else
    {
      _command_1[serial_count_1] = c;
      serial_count_1++;
      no_data_1 = 0;
      bytes_received_1 = true;
      idle_time_1 = millis();
    }
  }
  //mark no data if nothing heard for 400 milliseconds
  else
  {
    if ((millis() - idle_time_1) >= 100)
    {
      no_data_1++;
      idle_time_1 = millis();
    }
  }

  //if theres a pause or we got a real command, do it
  if (bytes_received_1 && no_data_1 )
  {
    process_raspPi(_command_1, serial_count_1);
    init_process_string_raspPi();
  }
}

void init_process_string_raspPi()
{
  //init our command
  for (byte i = 0; i < COMMAND_SIZE_1; i++) _command_1[i] = 0;
  serial_count_1 = 0;
  bytes_received_1 = false;
  idle_time_1 = millis();
}

void process_raspPi(char instruction[], int arraySize)
{
#ifdef DEBUG
  Serial.println(instruction);
#endif
}
