#include <functions.h>


unsigned long loopTimer, batteryVoltageTimer, ledTimer;
int lowVoltageAlarmCount;
bool previousIsArmed = false;


void setup2() {
  Wire.begin();
  Wire.setClock(1000000);

  mpu_6050_found = is_mpu_6050_found();
  Serial.print("mpu_6050_found=");
  Serial.println(mpu_6050_found);

  if (mpu_6050_found) {
    Serial.println("calibrating gyro");
    setup_mpu_6050_registers();
    calibrate_mpu_6050();
  }

  // pwm
  ledcSetup(SERVO_TOP_PWM_CHANNEL, PWM_FREQUENCY_SERVO, PWM_RESOLUTION_SERVO);
  ledcSetup(SERVO_BOTTOM_PWM_CHANNEL, PWM_FREQUENCY_SERVO, PWM_RESOLUTION_SERVO);
  ledcSetup(MOTOR_TOP_PWM_CHANNEL, PWM_FREQUENCY_ESC, PWM_RESOLUTION_ESC);
  ledcSetup(MOTOR_BOTTOM_PWM_CHANNEL, PWM_FREQUENCY_ESC, PWM_RESOLUTION_ESC);

  ledcAttachPin(SERVO_TOP_PIN, SERVO_TOP_PWM_CHANNEL);
  ledcAttachPin(SERVO_BOTTOM_PIN, SERVO_BOTTOM_PWM_CHANNEL);
  ledcAttachPin(MOTOR_TOP_PIN, MOTOR_TOP_PWM_CHANNEL);
  ledcAttachPin(MOTOR_BOTTOM_PIN, MOTOR_BOTTOM_PWM_CHANNEL);

  writeServoPWM(SERVO_TOP_PWM_CHANNEL, 0);
  writeServoPWM(SERVO_BOTTOM_PWM_CHANNEL, 0);
  writeEscPWM(MOTOR_TOP_PWM_CHANNEL, 0);
  writeEscPWM(MOTOR_BOTTOM_PWM_CHANNEL, 0);

  initLEDs();

  lowVoltageAlarmCount = 0;

  initValues();

  usedUpLoopTime = 0;
  batteryVoltageTimer = millis();
  ledTimer = millis();
  loopTimer = micros();    
}


void loop2() {
  unsigned long millisTimer = millis();

   //printChannels();

  int throttleSignal = channel[2]; 
  if (isValidSignal(throttleSignal)) {
    bool is_signal_detected = (throttleSignal > SIGNAL_LOST_PULSE);
    if (is_signal_detected) {
      signal_detected_count++;      
      if (signal_detected_count == SIGNALS_DETECTED_LOST_THRESHOLD) {
        signal_detected = true;
        playSignalDetected();
      } else if (signal_detected_count > SIGNALS_DETECTED_LOST_THRESHOLD) {
        signal_detected_count = SIGNALS_DETECTED_LOST_THRESHOLD;
        signal_lost_count = 0;        
      }
    } else {
      signal_lost_count++;      
      if (signal_lost_count == SIGNALS_DETECTED_LOST_THRESHOLD) {
        signal_detected = false;
        playSignalLost();
      } else if (signal_lost_count > SIGNALS_DETECTED_LOST_THRESHOLD) {
        signal_lost_count = SIGNALS_DETECTED_LOST_THRESHOLD;
        signal_detected_count = 0;
      }
    }
  } 

  flightMode = getFlightMode();

  int rollChannel = fixChannelDirection(getExpo(channel[0], rollExpoFactor), rollChannelReversed);
  int pitchChannel = fixChannelDirection(getExpo(channel[1], pitchExpoFactor), pitchChannelReversed);
  int throttleChannel = fixChannelDirection(channel[2], throttleChannelReversed);
  int yawChannel = fixChannelDirection(getExpo(channel[3], yawExpoFactor), yawChannelReversed);

  read_mpu_6050_data();
  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;
  
  calcAngles();

  gyro_roll_input = calcDegreesPerSecond(gyro_roll_input, gyro_roll.get());
  gyro_pitch_input = calcDegreesPerSecond(gyro_pitch_input, gyro_pitch.get());
  gyro_yaw_input = calcDegreesPerSecond(gyro_yaw_input, gyro_yaw.get());

  calcLevelAdjust(flightMode);

  double pid_roll_setpoint = calcPidSetPoint(rollChannel, roll_level_adjust);
  double pid_pitch_setpoint = calcPidSetPoint(pitchChannel, pitch_level_adjust);
  double pid_yaw_setpoint = 0.0;
  if (throttleChannel > 1050) pid_yaw_setpoint = calcPidSetPoint(yawChannel, 0.0);

  rollOutputPID.calc(gyro_roll_input, pid_roll_setpoint);
  pitchOutputPID.calc(gyro_pitch_input, pid_pitch_setpoint);
  yawOutputPID.calc(gyro_yaw_input, pid_yaw_setpoint);

  if (throttleChannel > MAX_THROTTLE) throttleChannel = MAX_THROTTLE;
  topEsc = limitEsc(throttleChannel - yawOutputPID.getOutput());
  bottomEsc = limitEsc(throttleChannel + yawOutputPID.getOutput());
  topServo = limitServo(MID_CHANNEL - rollOutputPID.getOutput() + topServoCenterOffset);
  bottomServo = limitServo(MID_CHANNEL + pitchOutputPID.getOutput() + bottomServoCenterOffset);
  // printMotorOutputs(topEsc, bottomEsc, topServo, bottomServo);

  bool actualIsArmed = isArmed();
  if (actualIsArmed && !previousIsArmed) {
    if (isArmingAllowed()) {
      initValues();
      playArmed();
    } else {
      actualIsArmed = false;
    }    
  } else if (!actualIsArmed && previousIsArmed) {
    initValues();
    playDisarmed();
  }
  previousIsArmed = actualIsArmed;

  if (signal_detected && actualIsArmed) {
    if (switchB.readPos() == 2) {
      writeEscPWM(MOTOR_TOP_PWM_CHANNEL, MIN_PULSE);
    } else {
      writeEscPWM(MOTOR_TOP_PWM_CHANNEL, topEsc);
    }
    if (switchD.readPos() == 2) {
      writeEscPWM(MOTOR_BOTTOM_PWM_CHANNEL, MIN_PULSE);
    } else {
      writeEscPWM(MOTOR_BOTTOM_PWM_CHANNEL, bottomEsc);
    }
    writeServoPWM(SERVO_TOP_PWM_CHANNEL, topServo);
    writeServoPWM(SERVO_BOTTOM_PWM_CHANNEL, bottomServo);    
  } else {
    writeEscPWM(MOTOR_TOP_PWM_CHANNEL, MIN_PULSE);
    writeEscPWM(MOTOR_BOTTOM_PWM_CHANNEL, MIN_PULSE);    
    writeServoPWM(SERVO_TOP_PWM_CHANNEL, MID_CHANNEL + topServoCenterOffset);
    writeServoPWM(SERVO_BOTTOM_PWM_CHANNEL, MID_CHANNEL + bottomServoCenterOffset);
  }

  // check battery voltage once per second
  if ((millisTimer - batteryVoltageTimer) > 1000) {
    if (isBootButtonPressed()) {
      waitForBootButtonClicked();
      calibrateESCs();
    }

    batteryVoltageTimer = millisTimer;
//    Serial.println(voltage);     
    if (voltage < LOW_VOLTAGE_ALARM) {
//      Serial.println("lowVoltageAlarmCount++"); 
      lowVoltageAlarmCount++;
      if (lowVoltageAlarmCount >= 10) {
//        Serial.println("lowVoltageAlarm!!"); 
        playLowVoltageAlarm(); 
      }
    } else {
      lowVoltageAlarmCount = 0;
    }
  }    

  // update LEDs
  if ((millisTimer - ledTimer) > 250) {
    ledTimer = millisTimer;
    updateLEDs(rollChannel, yawChannel);
  }

  rtttl::play();

  //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
  usedUpLoopTime = micros() - loopTimer;
  /*
  Serial.print(usedUpLoopTime);
  Serial.println();
  */
  while(micros() - loopTimer < LOOP_TIME) {
    vTaskDelay(1);
  };
  loopTimer = micros();           
}


void runOnCore2(void *parameter) {
  Serial.print("Core: ");
  Serial.println(xPortGetCoreID());
  setup2();
  for (;;) {
    loop2();
  }
}
