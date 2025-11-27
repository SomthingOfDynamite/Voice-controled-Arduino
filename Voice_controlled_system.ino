#include <LiquidCrystal.h>
#include <Servo.h>

// LED
const int sw_led = A0;
bool last_led_sw = HIGH;
const int led_pin = 2;
bool led_state = false;
int led_mode = 0;  // 0=solid, 1=fast blink, 2=slow blink
unsigned long led_prevMillis = 0;

// LCD
LiquidCrystal lcd(13, 12, 11, 10, 9, 8);
const int sw_lcd = A1;
bool last_lcd_sw = HIGH;
bool lcd_state = false;
int lcd_mode = 0;  // 0=welcome, 1=all ok, 2=resetting
unsigned long lcd_prevMillis = 0;

// DC motor
const int sw_motor = A2;
bool last_motor_sw = HIGH;
const int motor_pin = 11;
bool motor_state = false;
int motor_mode = 0;  // 0=slow, 1=medium, 2=fast

// Micro servo
Servo servo;
const int sw_servo = A3;
bool last_servo_sw = HIGH;
const int servo_pin = 7;
bool servo_state = false;
int servo_mode = 0; // 0=45°, 1=90°, 2=180°

// Action button
const int btn_select = A4;
bool lastbtn5_state = HIGH;

 // Mode Select button
const int btn_function = A5;  
bool lastbtn6_state = HIGH;

int selected_component = -1;  // stores index of selected ON component


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

  // Component toggles
  checkToggleButton(sw_led, last_led_sw, led_state, 0);
  checkToggleButton(sw_lcd, last_lcd_sw, lcd_state, 1);
  checkToggleButton(sw_motor, last_motor_sw, motor_state, 2);
  checkToggleButton(sw_servo, last_servo_sw, servo_state, 3);


  // Action Button
  bool btn5_state = digitalRead(btn_select);
  if(btn5_state == LOW && lastbtn5_state == HIGH && selected_component != -1){
    cycleAction(selected_component);
  }
  lastbtn5_state = btn5_state;


  // Mode Select Button
  bool btn6_state = digitalRead(btn_function);
  if(btn6_state == LOW && lastbtn6_state == HIGH && selected_component != -1){
    cycleComponentSelection();
  }
  lastbtn6_state = btn6_state;


  // Actions
  updateLED();
  updateLCD();
  updateMotor();
  updateServo();
}



// Button toggle on/off
void checkToggleButton(int pin, bool &lastState, bool &stateVar, int componentID){
  bool curr = digitalRead(pin);
  if(curr == LOW && lastState == HIGH){
    stateVar = !stateVar;
    if(stateVar){
      selected_component = componentID; // auto-select the one turned on
      applyDefault(componentID);
    }
    else {
      shutdownComponent(componentID);
    }
  }
  lastState = curr;
}

void shutdownComponent(int comp){
  switch(comp){
    case 0: // LED
      digitalWrite(led_pin, LOW);
      led_mode = 0;
      break;

    case 1: // LCD
      lcd.clear();
      lcd.noDisplay();
      break;

    case 2: // Motor
      analogWrite(motor_pin, 0);
      motor_mode = 0;
      break;

    case 3: // Servo
      servo.write(0);
      servo_mode = 0;
      break;
  }
}


// Default states
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



// Mode Button: select component
void cycleComponentSelection(){
  int start = selected_component;
  for(int i = 0; i < 5; i++){ // 0..4 (including All)
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
  // none active if all are off
  selected_component = -1;
}


// Action button: cycle actions
void cycleAction(int comp){
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
  } else { // Single component
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
    servo.write(45); 
  }
  if(servo_mode == 1) {
    servo.write(90);
  }
  if(servo_mode == 2) {
    servo.write(180);
  }
}