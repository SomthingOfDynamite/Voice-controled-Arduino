#include <LiquidCrystal.h>
#include <Servo.h>

// LED
const int sw_led = A0;
bool last_led_sw = HIGH;
const int led_pin = 2;
bool led_state = false;
// 0=solid, 1=fast blink, 2=slow blink
int led_mode = 0;
unsigned long led_prevMillis = 0;

// LCD
LiquidCrystal lcd(13, 12, 11, 10, 9, 8);
const int sw_lcd = A1;
bool last_lcd_sw = HIGH;
bool lcd_state = false;
// 0=welcome, 1=all ok, 2=resetting
int lcd_mode = 0;
unsigned long lcd_prevMillis = 0;

// DC motor
const int sw_motor = A2;
bool last_motor_sw = HIGH;
const int motor_pin = 3;
bool motor_state = false;
// 0=slow, 1=medium, 2=fast
int motor_mode = 0;

// Micro servo
Servo servo;
const int sw_servo = A3;
bool last_servo_sw = HIGH;
const int servo_pin = 7;
bool servo_state = false;
// 0=45°, 1=90°, 2=180°
int servo_mode = 0;

// Button
const int btn_select = A4;
bool lastbtn5_state = HIGH;

 // Mode Select button
const int btn_function = A5;  
bool lastbtn6_state = HIGH;
// stores index of selected ON component
int selected_component = -1;


// Setup
void setup() {
  Serial.begin(9600);

  pinMode(sw_led, INPUT_PULLUP);
  pinMode(sw_lcd, INPUT_PULLUP);
  pinMode(sw_motor, INPUT_PULLUP);
  pinMode(sw_servo, INPUT_PULLUP);

  pinMode(btn_select, INPUT_PULLUP);
  pinMode(btn_function, INPUT_PULLUP);

  pinMode(led_pin, OUTPUT);
  pinMode(motor_pin, OUTPUT);

  lcd.begin(16, 2);

  servo.attach(servo_pin);
  servo.write(0);
}


// Main loop
void loop() {
  //listen for voice commands
  readSerialCommands();

  // Component toggles
  checkSwitch(sw_led, led_state, 0);
  checkSwitch(sw_lcd, lcd_state, 1);
  checkSwitch(sw_motor, motor_state, 2);
  checkSwitch(sw_servo, servo_state, 3);


  // function Button
  bool btn5_state = digitalRead(btn_select);
  if(btn5_state == LOW && lastbtn5_state == HIGH && selected_component != -1){
    cycleFunction(selected_component);
  }
  lastbtn5_state = btn5_state;


  // mode Select Button
  bool btn6_state = digitalRead(btn_function);
  if(btn6_state == LOW && lastbtn6_state == HIGH && selected_component != -1){
    cycleComponentSelection();
  }
  lastbtn6_state = btn6_state;


  // functions
  updateLED();
  updateLCD();
  updateMotor();
  updateServo();
}

void readSerialCommands() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();

        if (cmd == "LED_ON") { digitalWrite(led_pin, HIGH); }
        else if (cmd == "LED_OFF") { digitalWrite(led_pin, LOW); }

        else if (cmd == "LCD_ON") { lcd_state = true; lcd_mode = 0; }
        else if (cmd == "LCD_OFF") { lcd_state = false; lcd.clear(); }

        else if (cmd == "MOTOR_ON") { digitalWrite(motor_pin, HIGH); }
        else if (cmd == "MOTOR_OFF") { digitalWrite(motor_pin, LOW); }

        else if (cmd == "SERVO_ON") { servo.write(90); }
        else if (cmd == "SERVO_OFF") { servo.write(0); }
    }
}

// switch on/off
void checkSwitch(int pin, bool &stateVar, int componentID){
  int curr = digitalRead(pin);
  //switch on
  if(curr == LOW && !stateVar){ 
    stateVar = true;
    selected_component = componentID;
    applyDefault(componentID);
  }
  // switch off
  else if(curr == HIGH && stateVar) {
    stateVar = false;
    shutdownComponent(componentID);
    if(selected_component == componentID) {}
      selected_component = -1;
    }
  }
}

// turn off component
void shutdownComponent(int comp) {
  switch(comp) {
    // LED
    case 0:
      digitalWrite(led_pin, LOW);
      led_mode = 0;
      break;
    // LCD
    case 1:
      lcd.clear();
      lcd.noDisplay();
      break;
    // Motor
    case 2:
      analogWrite(motor_pin, 0);
      motor_mode = 0;
      break;
    // Servo
    case 3:
      servo.write(0);
      servo_mode = 0;
      break;
  }
}


// default states
void applyDefault(int comp){
  switch(comp){
    case 0: 
      led_mode = 0;
      digitalWrite(led_pin, HIGH);
      break;
    case 1:
      lcd.display();
      lcd_mode = 0;
      lcd.clear();
      lcd.print("Welcome User");
      break;
    case 2:
      motor_mode = 0;
      analogWrite(motor_pin, 80);
      break;
    case 3:
      servo_mode = 0;
      servo.write(45);
      break;
  }
}



// mode Button: select component
void cycleComponentSelection(){
  int start = selected_component;
  for(int i = 0; i < 5; i++){
    start++;
    if(start > 4) start = 0;

    // Skip if single component is off
    if( (start==0 && !led_state) ||
        (start==1 && !lcd_state) ||
        (start==2 && !motor_state) ||
        (start==3 && !servo_state) ){
      continue;
    }
    // If start==4 (All), always allow
    selected_component = start;
    return;
  }
  // none if all components are off
  selected_component = -1;
}


// cycle functions after buttton press
void cycleFunction(int comp){
  if(comp == 4){
    if(led_state){
      led_mode = (led_mode + 1) % 3;
    }
    if(lcd_state){
      lcd_mode = (lcd_mode + 1) % 3;
    }
    if(motor_state){
      motor_mode = (motor_mode + 1);
    }
    if(servo_state){
      servo_mode = (servo_mode + 1) % 3;
    }
  } 
  // single component
  else {
    switch(comp){
      case 0: 
        led_mode = (led_mode + 1) % 3; break;
      case 1:
        lcd_mode = (lcd_mode + 1) % 3; break;
      case 2:
        motor_mode = (motor_mode + 1) % 3; break;
      case 3:
        servo_mode = (servo_mode + 1) % 3; break;
    }
  }
  
}



// LED events
void updateLED(){
  if(!led_state) return;
  unsigned long now = millis();
  if(led_mode == 0){
    digitalWrite(led_pin, HIGH);
  }
  // fast blink
  else if(led_mode == 1){
    if(now - led_prevMillis > 200){
      led_prevMillis = now;
      digitalWrite(led_pin, !digitalRead(led_pin));
    }
  }
  // slow blink
  else if(led_mode == 2){
    if(now - led_prevMillis > 500){
      led_prevMillis = now;
      digitalWrite(led_pin, !digitalRead(led_pin));
    }
  }
}



//LCD events
void updateLCD(){
  if(!lcd_state) return;
  static int lastMode = -1;
  //dont update if not needed
  if(lcd_mode == lastMode) return;
  lastMode = lcd_mode;
  lcd.clear();
  if(lcd_mode == 0) {
    lcd.print("Welcome User");
  }
  if(lcd_mode == 1) {
    lcd.print("All Systems OK");
  }
  if(lcd_mode == 2) {
    lcd.print("Resetting...");
  }
}



// MOTOR events
void updateMotor(){
  if(!motor_state) {
    return;
  }
  if(motor_mode == 0) {
    analogWrite(motor_pin, 80);
  }
  if(motor_mode == 1) {
    analogWrite(motor_pin, 150);
  }
  if(motor_mode == 2) {
    analogWrite(motor_pin, 220);
  }
}


// SERVO events
void updateServo(){
  if(!servo_state) {
    return;
  }
  if(servo_mode == 0) {
    servo.write(90); 
  }
  if(servo_mode == 1) {
    servo.write(135);
  }
  if(servo_mode == 2) {
    servo.write(180);
  }
}
