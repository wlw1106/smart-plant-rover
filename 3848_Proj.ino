#include <Arduino.h>
#include <Servo.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050_light.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels


// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     28 //4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
int oldV=1, newV=0;
#include <SoftwareSerial.h>
//UNO: (2, 3)
//SoftwareSerial mySerial(4, 6); // RX, TX
int pan = 90;
int tilt = 120;
int window_size = 0;
int BT_alive_cnt = 0;
int voltCount = 0;
#include <Servo.h>
Servo servo_pan;
Servo servo_tilt;
int servo_min = 20;
int servo_max = 160;

unsigned long time;

//FaBoPWM faboPWM;
int pos = 0;
int INT;
int MAX_VALUE = 2000;
int MIN_VALUE = 300;

//distance
unsigned long start_time = 0;
int done = 1;
long distance_in_cm;
long duration;

//soil sensor
int soilvalue = 0;

//light intensity
int lightvalue = 0;

//distance
bool arrived = false;

//communicate with raspberry pi 
char command;
Servo horizontal_servo;
Servo vertical_servo;
const int HORIZONTAL_PIN = 47;
const int VERTICAL_PIN = 48;
int horizontal_pos = 70;  //180-0/left-right
int vertical_pos = 70;  //180-0/down-up
const int step_size = 15;

// Define motor pins
#define PWMA 12    //Motor A PWM
#define DIRA1 34
#define DIRA2 35  //Motor A Direction
#define PWMB 8    //Motor B PWM
#define DIRB1 37
#define DIRB2 36  //Motor B Direction
#define PWMC 9   //Motor C PWM --> from 6 to 9
#define DIRC1 43
#define DIRC2 42  //Motor C Direction
#define PWMD 5    //Motor D PWM
#define DIRD1 A4  //26  
#define DIRD2 A5  //27  //Motor D Direction

int Tolerance = 100;           //tolerance for adc different, avoid oscillation
int pervious = 1000;

#define MOTORA_FORWARD(pwm)    do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,HIGH);analogWrite(PWMA,pwm);}while(0)
#define MOTORA_STOP(x)         do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,LOW); analogWrite(PWMA,0);}while(0)
#define MOTORA_BACKOFF(pwm)    do{digitalWrite(DIRA1,HIGH);digitalWrite(DIRA2,LOW); analogWrite(PWMA,pwm);}while(0)

#define MOTORB_FORWARD(pwm)    do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,HIGH);analogWrite(PWMB,pwm);}while(0)
#define MOTORB_STOP(x)         do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,LOW); analogWrite(PWMB,0);}while(0)
#define MOTORB_BACKOFF(pwm)    do{digitalWrite(DIRB1,HIGH);digitalWrite(DIRB2,LOW); analogWrite(PWMB,pwm);}while(0)

#define MOTORC_FORWARD(pwm)    do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,HIGH);analogWrite(PWMC,pwm);}while(0)
#define MOTORC_STOP(x)         do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,LOW); analogWrite(PWMC,0);}while(0)
#define MOTORC_BACKOFF(pwm)    do{digitalWrite(DIRC1,HIGH);digitalWrite(DIRC2,LOW); analogWrite(PWMC,pwm);}while(0)

#define MOTORD_FORWARD(pwm)    do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,HIGH);analogWrite(PWMD,pwm);}while(0)
#define MOTORD_STOP(x)         do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,LOW); analogWrite(PWMD,0);}while(0)
#define MOTORD_BACKOFF(pwm)    do{digitalWrite(DIRD1,HIGH);digitalWrite(DIRD2,LOW); analogWrite(PWMD,pwm);}while(0)

#define SERIAL  Serial
#define BTSERIAL Serial3

// Define Light Sensor
#define light_sensor A14

//Define ultra sonics
#define leftup_echopin 46
#define leftup_triggerpin 13

#define leftdown_echopin 10
#define leftdown_triggerpin 11

#define rightup_echopin 4
#define rightup_triggerpin 7

#define rightdown_echopin 44
#define rightdown_triggerpin 45

//Define Soil Sensor
#define soil_sensor A13

//Define Light
#define light_source A12
#define NUMPIXELS 12
Adafruit_NeoPixel pixels(NUMPIXELS, light_source, NEO_GRB + NEO_KHZ800);
#define DELAYVAL 500

//Define Pump
#define pump A15

//Define autotune
int turntimes = 0;
int last_turn = 0; //0 left 1 right;

//Define number of loops
int numloop = 0;

#define LOG_DEBUG

#ifdef LOG_DEBUG
  #define M_LOG SERIAL.print
#else
  #define M_LOG BTSERIAL.println
#endif

//PWM Definition
#define MAX_PWM   2000
#define MIN_PWM   300

int Motor_PWM = 50;
int D_PWM = 40;
MPU6050 mpu(Wire);

void BACK()
//    ↓A-----B↓
//     |  ↓  |
//     |  |  |
//    ↓C-----D↓
{
  MOTORA_BACKOFF(Motor_PWM); 
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM); 
  MOTORD_BACKOFF(D_PWM);
}

void ADVANCE()
//    ↑A-----B↑
//     |  |  |
//     |  ↑  |
//    ↑C-----D↑
{
  MOTORA_FORWARD(Motor_PWM); 
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM); 
  MOTORD_FORWARD(Motor_PWM);
}

void LEFT()
//    ↓A-----B↑
//     |  ←  |
//     |  ←  |
//    ↑C-----D↓
{
  MOTORA_BACKOFF(Motor_PWM); 
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM); 
  MOTORD_BACKOFF(Motor_PWM);
}


void RIGHT()
//    ↑A-----B↓
//     |  →  |
//     |  →  |
//    ↓C-----D↑
{
  MOTORA_FORWARD(Motor_PWM); 
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM); 
  MOTORD_FORWARD(Motor_PWM);
}


void rotate_r()  //tate_1(uint8_t pwm_A,uint8_t pwm_B,uint8_t pwm_C,uint8_t pwm_D)
//    ↑A-----B↓
//     | ↗ ↘ |
//     | ↖ ↙ |
//    ↑C-----D↓
{
  MOTORA_BACKOFF(Motor_PWM); 
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM); 
  MOTORD_FORWARD(D_PWM);
}

void rotate_l()  // rotate_2(uint8_t pwm_A,uint8_t pwm_B,uint8_t pwm_C,uint8_t pwm_D)
//    ↓A-----B↑
//     | ↙ ↖ |
//     | ↘ ↗ |
//    ↓C-----D↑
{
  MOTORA_FORWARD(Motor_PWM); 
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM); 
  MOTORD_BACKOFF(D_PWM);
}

void STOP()
//    =A-----B=
//     |  =  |
//     |  =  |
//    =C-----D=
{
  MOTORA_STOP(Motor_PWM);
  MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM);
  MOTORD_STOP(Motor_PWM);
}

void print_OLED(String text, String text2){
  display.clearDisplay();
  display.setTextSize(1.5);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  display.setCursor(0, 0);     // Start at top-left corner
  display.println(text);
  if (text2 != ""){
    display.println(text2);
  }
  display.display(); 
}

void print_OLED_int(int text){
  display.clearDisplay();
  display.setTextSize(1.5);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  display.setCursor(0, 0);     // Start at top-left corner
  display.println(text);
  display.display(); 
}

long measure_distance(int localEchoPin, int localTrigPin) {
  // Clear the trigger pin (set it low) for 2uS
  digitalWrite(localTrigPin, LOW);
  delayMicroseconds(2);
  
  // Send 10uS pulse
  digitalWrite(localTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(localTrigPin, LOW);
  
  // Read the echo pin travel time
  duration = pulseIn(localEchoPin, HIGH);
  
  // Calculate distance in cm (using the standard 58)
  // 58 microseconds per centimeter round trip
  distance_in_cm = duration / 58; 
  
  return distance_in_cm;
}

int a_dist = measure_distance(46,13);
int b_dist = measure_distance(4,7);
int c_dist = measure_distance(10,11);
int d_dist = measure_distance(44,45);

void turn_light(bool turnon){
  if (turnon){
    print_OLED("turning on","light");
    pixels.clear();
    for(int i=0; i<NUMPIXELS; i++) {
      pixels.setPixelColor(i, pixels.Color(238, 130, 238));

      pixels.show();  

      delay(DELAYVAL);
    }
  }
  else{
    print_OLED("turning off","light");
    pixels.clear();
    pixels.show();
  }
}

int lightintensity(){
  lightvalue = analogRead(light_sensor);
  return lightvalue;
}

int readsoil(){
  soilvalue = analogRead(soil_sensor);
  return soilvalue;
}

void activate_pump(){
  print_OLED("activating","pump");
  digitalWrite(pump, HIGH);
  delay(2500);
  digitalWrite(pump, LOW);
  delay(7000);
}

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  horizontal_servo.attach(HORIZONTAL_PIN);
  vertical_servo.attach(VERTICAL_PIN);

  horizontal_servo.write(horizontal_pos);
  vertical_servo.write(vertical_pos);

  pinMode(leftup_echopin, INPUT);
  pinMode(leftup_triggerpin, OUTPUT);

  pinMode(rightup_echopin, INPUT);
  pinMode(rightup_triggerpin, OUTPUT);

  pinMode(leftdown_echopin, INPUT);
  pinMode(leftdown_triggerpin, OUTPUT);

  pinMode(rightdown_echopin, INPUT);
  pinMode(rightdown_triggerpin, OUTPUT);

  pinMode(light_sensor,INPUT);

#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif

  pinMode(pump,OUTPUT);

  pinMode(soil_sensor, INPUT);

  Wire.begin();
  mpu.begin();
  pixels.begin();
  Serial.println();

  //OLED Setup//////////////////////////////////
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
  }
  turn_light(false);
}

void state1(){
  while (arrived == false){
  if (Serial.available() > 0){
    command =  Serial.read();
    print_OLED("State1...","");
    if (arrived == false){
      switch (command) {
        case 'd':
        print_OLED("camera moving","right");
        horizontal_pos -= step_size;
        // Constrain and write immediately
        horizontal_pos = constrain(horizontal_pos, 0, 180);
        horizontal_servo.write(horizontal_pos); 
        break;
        
       case 'a':
        print_OLED("camera moving","left");
        horizontal_pos += step_size;
        // Constrain and write immediately
        horizontal_pos = constrain(horizontal_pos, 0, 180);
        horizontal_servo.write(horizontal_pos); 
        break;     
        
        case 's':
        print_OLED("camera moving","down");
        vertical_pos += step_size;
        // Constrain and write immediately
        vertical_pos = constrain(vertical_pos, 0, 180);
        vertical_servo.write(vertical_pos); 
        break;
        
        case 'w':
        print_OLED("camera moving","up");
        vertical_pos -= step_size;
        // Constrain and write immediately
        vertical_pos = constrain(vertical_pos, 0, 180);
        vertical_servo.write(vertical_pos); 
        break;

        case 'l':
          print_OLED("moving","left");
          LEFT();
          delay(500);
          STOP();
          break;
        case 'r':
          print_OLED("moving","right");
          RIGHT();
          delay(500);
          STOP();
          break;
        case 'f':
          print_OLED("advancing","");
          ADVANCE();
          delay(500);
          STOP();
          a_dist = measure_distance(46,13);
          b_dist = measure_distance(4,7);
          print_OLED(String(a_dist), String(b_dist));
          if ((a_dist <= 8) and (b_dist <= 8)){
            print_OLED("ready to ","auto tune");
            arrived = true;
          }
          break;
        case 'b':
          print_OLED("moving","backwards");
          BACK();
          delay(500);
          STOP();
          break;
        case 'o':
          print_OLED("rotate","right");
          rotate_r();
          delay(250);
          STOP();
          break;
        case 'i':
          print_OLED("rotate","left");
          rotate_l();
          delay(250);
          STOP();
          break;
      }
    }
  }else{
    print_OLED("Still waiting for","RaspberryPi boss");
  }
  }
  delay(2000);
  if (arrived){
    while (true){
      a_dist = measure_distance(46,13);
      b_dist = measure_distance(4,7);
      print_OLED(String(a_dist),String(b_dist));
      if ((a_dist == b_dist) or (turntimes == 3)){
        break;
      }
      else if (a_dist < b_dist){
        if ((last_turn == 1) or ((last_turn == 0) and (turntimes <=2))){
          last_turn = 0;
          rotate_l();
          delay(100);
          STOP();
          turntimes ++;
        }
      }
      else if (b_dist < a_dist){
        if ((last_turn == 0) or ((last_turn == 1) and (turntimes <=2))){
          last_turn = 1;
          rotate_r();
          delay(200);
          STOP();
          turntimes ++;
        }
      }
    }
  }
}

void state2(){
  if (lightintensity()<1000){
    turn_light(true);
    turn_light(false);
  }
  if (readsoil() < 2000){
    activate_pump();
  }
}

void adjuststate(){
  print_OLED("Please adjust", "start in 5");
  delay(1000);
  print_OLED("Please adjust", "start in 4");
  delay(1000);
  print_OLED("Please adjust", "start in 3");
  delay(1000);
  print_OLED("Please adjust", "start in 2");
  delay(1000);
  print_OLED("Please adjust", "start in 1");
  delay(1000);
}

void state3(){
  arrived = false;
  BACK();
  delay(2000);
  STOP();
}

void loop() 
{
  if (numloop == 5){
    Serial.println("G");
    numloop = 0;
  }
  print_OLED("Preparing...","");
  delay(1500);
  Serial.println("S");
  print_OLED("State 1","");
  delay(1500);
  state1();
  print_OLED("State 2","");
  Serial.println("G");
  delay(1500);
  adjuststate();
  state2();
  print_OLED("State 3","");
  delay(1500);
  state3();
  arrived = false;
  print_OLED("Sleeping","zzzzzzzz");
  numloop ++;
  delay(5000);
  
}