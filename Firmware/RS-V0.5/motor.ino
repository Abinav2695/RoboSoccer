
// Write PWM value to the pin
void _pwm_write(int Pin, int value)
{
  analogWrite(Pin, value);
}

//Set digital value of the pin (HIGH/LOW)
void _pin_write(int Pin, bool _output)
{
  digitalWrite(Pin, _output);
}
bool _pin_read(int Pin)
{
  return (digitalRead(Pin));
}
//Set pinmode of the pin

void set_pinmode(int Pin, int Mode)
{
  pinMode(Pin, Mode);
}

//set pinMode and pwm values of all motor pins
void pwm_setup()
{
  // set timer frequency to 58.6 kHz
  analogWriteFrequency(TIM0, PWM_FREQUENCY);
  analogWriteFrequency(TIM1, PWM_FREQUENCY);

  //set analog Resolution
  analogWriteResolution(ANALOG_RESOLUTION);

  for (byte i = 0;  i < NO_OF_MOTORS; i++)
  {
    set_pinmode(pwm_pin2[i], OUTPUT);
    delay(1);
    set_pinmode(pwm_pin1[i], OUTPUT);
    delay(1);
    _pwm_write(pwm_pin1[i], 0);
    _pwm_write(pwm_pin2[i], 0);
    delay(1);
  }

#ifdef TEST
  for (byte i = 0;  i < NO_OF_MOTORS; i++)
  {
    _pwm_write(pwm_pin1[i], 1023);
    _pwm_write(pwm_pin2[i], 0);
    delay(1);
  }
  delay(2000);
  for (byte i = 0;  i < NO_OF_MOTORS; i++)
  {
    _pwm_write(pwm_pin1[i], 0);
    _pwm_write(pwm_pin2[i], 0);
    delay(1);
  }
  delay(1000);
  for (byte i = 0;  i < NO_OF_MOTORS; i++)
  {
    _pwm_write(pwm_pin1[i], 0);
    _pwm_write(pwm_pin2[i], 1023);
    delay(1);
  }
  delay(2000);

  for (byte i = 0;  i < NO_OF_MOTORS; i++)
  {
    _pwm_write(pwm_pin1[i], 0);
    _pwm_write(pwm_pin2[i], 0);
    delay(1);
  }
#endif

}


//Setup all Peripherals
void esp_setup()
{
  set_pinmode(ESP_ENABLE_PIN, OUTPUT);
  _pin_write(DRIBBLER_PIN, LOW);
}

void esp_enable(bool enFlag)
{
  if(enFlag)_pin_write(ESP_ENABLE_PIN, HIGH);
  else _pin_write(ESP_ENABLE_PIN, LOW);
}

void dribbler_setup()
{
  set_pinmode(DRIBBLER_PIN, OUTPUT);
  _pin_write(DRIBBLER_PIN, LOW);

}
void indicationLed_setup()
{
  set_pinmode(INDICATION_LED, OUTPUT);
  _pin_write(INDICATION_LED,LOW);
}

void led_enable(bool ledFlag)
{
  if(ledFlag)_pin_write(INDICATION_LED, HIGH);
  else _pin_write(INDICATION_LED, LOW);
}

void kicker_setup()
{
  set_pinmode(_LT3750_donePin, INPUT);
  set_pinmode(_LT3750_chargePin, OUTPUT);
  set_pinmode(_solenoidTriggerPin, OUTPUT);

  _pin_write(_LT3750_chargePin, LOW);
  _pin_write(_solenoidTriggerPin, LOW);
}

void _update_pwm_value(byte *dirOfWheels, int *pwmOfWheels_modified)
{
  for (byte k = 0; k < NO_OF_MOTORS; k++)
  {
    if (dirOfWheels[k] == 2)
    {
      _pwm_write(pwm_pin1[k], 0);
      _pwm_write(pwm_pin2[k], pwmOfWheels_modified[k]);
    }
    else
    {
      _pwm_write(pwm_pin1[k], pwmOfWheels_modified[k]);
      _pwm_write(pwm_pin2[k], 0);
    }
  }
}

void _stop()
{
  //  rotate = false;
  //  prev_angle = current_angle;
  for (byte i = 0;  i < NO_OF_MOTORS; i++)
  {
    _pwm_write(pwm_pin1[i], 0);
    _pwm_write(pwm_pin2[i], 0);
    delay(1);
  }
  //Serial.print(prev_angle);
}


void kick()
{
  while (1)
  {
    if (digitalRead(_LT3750_donePin) == LOW)
    {
      _kickerCharged = true;
      _chargeTriggered = false;
      _pin_write(_LT3750_chargePin, LOW);
    }

    if (!_kickerCharged && !_chargeTriggered && _kick)
    {
      _pin_write(_LT3750_chargePin, HIGH);
      _chargeTriggered = true;
    }

    else if (_kick && _kickerCharged )
    {
      _pin_write(_solenoidTriggerPin, HIGH);
      threads.delay(20);
      _pin_write(_solenoidTriggerPin, LOW);
      _kickerCharged = false;
      _kick=false;
    }
    threads.yield();
    threads.delay(1);
  }
}

void kick2()
{
  while (1)
  {
    if (digitalRead(_LT3750_donePin) == LOW)
    {
      _kickerCharged = true;
     
      _pin_write(_LT3750_chargePin, LOW);
    }

    if (!_kickerCharged)
    {
      _pin_write(_LT3750_chargePin, HIGH);
     
    }

    else if (_kick)
    {
      _pin_write(_LT3750_chargePin, LOW);
      threads.delay(20);
      _pin_write(_solenoidTriggerPin, HIGH);
      threads.delay(20);
      _pin_write(_solenoidTriggerPin, LOW);
      _kickerCharged = false;
      _kick=false;
    }
    threads.yield();
    threads.delay(1);
  }
}
