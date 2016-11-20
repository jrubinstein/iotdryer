// This #include statement was automatically added by the Particle IDE.
#include "LiquidCrystal/LiquidCrystal.h"

// This #include statement was automatically added by the Particle IDE.
#include "PietteTech_DHT/PietteTech_DHT.h"

// This #include statement was automatically added by the Particle IDE.
#include "blynk/blynk.h"



// This #include statement was automatically added by the Particle IDE.
#include "PietteTech_DHT/PietteTech_DHT.h"

//Sample using LiquidCrystal library
//#include <LiquidCrystal.h>

#include "math.h" 
#define DHTTYPE DHT22       // Sensor type DHT11/21/22/AM2301/AM2302
#define DHTPIN A1     // what pin we're connected to
#define ON 0x7
#define OFF 0x0
#define BLYNK_PRINT Serial
#include "blynk/BlynkSimpleParticle.h"

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "useyourownauthtoken";






void dht_wrapper(); // must be declared before the lib initialization

PietteTech_DHT DHT(DHTPIN, DHTTYPE, dht_wrapper); //piettetech dht init
//initialize icons
byte icons[3][7] = {{ 0x0, 0x4, 0x6, 0x1f, 0x6, 0x4, 0x0},
{0x0, 0xe, 0x15, 0x17, 0x11, 0xe, 0x0},
{0x4, 0xe, 0xe, 0x1e, 0x1b, 0xb, 0x6}
  };
//! Index into the bitmap array for the  icons.
const int ARROW_ICON_IDX = 0;
const int CLOCK_ICON_IDX = 1;
const int FLAME_ICON_IDX = 2;

//constants
const long meas_interval = 3000;
const int motorPin = A0;
const int heaterPin = A2;
const int buzzerPin = A4;
const long heater_timeout_interval = 120000; //2 minute lockout
const long time_interval = 3000; //check how much time left every 3 seconds
const long temp_interval = 3000; //check what to set temp to every 3 seconds
//int button_out_pin = A2; 
const int arrow_buttons = A3;
const int start_stop_button = 0; //start stop button is on D0
// select the pins used on the LCD panel
//LiquidCrystal lcd(8, 9, 4, 5, 6, 7); //this is the default for the sainsmart lcd
LiquidCrystal lcd(2, 1, 6, 5, 4, 3);
// define some values used by the panel and buttons
int lcd_key     = 0;
int adc_key_in  = 0;
//int LCD_backlight = 0;//pin used to control the backlight
#define btnNONE   0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnRIGHT  5 




//variables
uint8_t clicked_buttons;
int menu = 0;
unsigned long previous_print_millis;
const unsigned long printInterval = 3000;
unsigned long heater_last_used_millis;
int motorState = LOW;
int heaterState = LOW;
boolean heater_timeout = false;
unsigned long previous_meas_millis = 0;
unsigned long previous_time_millis = 0;
unsigned long previous_temp_millis = 0;
float inside_temp;
float inside_moisture;
int time_remaining = 60;
int temp_setpoint = 101;
unsigned long currentMillis;
unsigned long cooldown_start_millis = 0;
unsigned long cooldown_interval = 300000;




//button variables
int buttonReading;
int value_holder;            
bool buttonPressed = false; 
bool lastButtonState = false;

bool startButtonPressed = false;
bool lastStartButtonState = false;
bool start_stop = true;
int vButton = 0;



//variables and constants for moving average of moisture
const int numReadings = 10;
int readings[numReadings];      // the readings from the analog input
int reading_idx = 0;                  // the index of the current reading
int total = 0;                  // the running total
int average_moisture = 0;                // the average

//variables for time and temp selections
String run_selected;
String temp_selected = "auto";
String time_selected = "auto";
String prev_temp_selected = 0;
String prev_time_selected = 0;
int run_type = 0;
int prev_time_remaining = 0;
unsigned long previous_run_millis = 0;
unsigned long time_elapsed = 0;
int prev_menu=-1;
unsigned long menu_last_clicked_millis=0;
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 100;    // the debounce time; increase if the output flickers




enum OperatingMode{
  STANDBY_MODE,
  RUN_MODE,
  COOL_MODE,
  STOP_MODE
};

OperatingMode currentMode = STANDBY_MODE;
OperatingMode lastMode = STANDBY_MODE;
const char* currentModeText[] = {"Standby","Run ","Cooldown","Stop"};


//const and vars for internet connection
SYSTEM_MODE(SEMI_AUTOMATIC)
  SYSTEM_THREAD(ENABLED)
  
  const uint32_t msRetryDelay = 5*60000; // retry every 5min
const uint32_t msRetryTime  =   10000; // stop trying after 10sec

bool retryRunning = false;
Timer retryTimer(msRetryDelay, retryConnect);  // timer to retry connecting
Timer stopTimer(msRetryTime, stopConnect);     // timer to stop a long running try

//*********************** setup ********************************//
void setup() 
{
  Serial.begin(9600);
  Particle.connect();
  if (!waitFor(Particle.connected, msRetryTime)){
    WiFi.off();                // no luck, no need for WiFi
  }
  delay(1000);
  lcd.begin(16, 2);
  //digitalWrite(LCD_backlight,HIGH);//turn on the backlight
  lcd.createChar(ARROW_ICON_IDX, icons[ARROW_ICON_IDX]);
  lcd.createChar(CLOCK_ICON_IDX, icons[CLOCK_ICON_IDX]);
  lcd.createChar(FLAME_ICON_IDX, icons[FLAME_ICON_IDX]);
  
  lcd.setCursor(0, 0);
  lcd.write(FLAME_ICON_IDX);
  lcd.write(CLOCK_ICON_IDX);
  lcd.print(" Welcome to ");
  lcd.write(CLOCK_ICON_IDX);
  lcd.write(FLAME_ICON_IDX);
  lcd.setCursor(2, 1);
  lcd.print("Dryerbot9000");
  pinMode(motorPin, OUTPUT);
  pinMode(heaterPin, OUTPUT);
  pinMode(start_stop_button, INPUT_PULLDOWN);
  pinMode(arrow_buttons, INPUT);
  for (int thisReading = 0; thisReading < numReadings; thisReading++) //initialize counter for moving average of moisture
    readings[thisReading] = 0;
  delay(2000);
  lcd.clear();
  menu_fun();
  Blynk.begin(auth);
  pinMode(7, OUTPUT);
}
//
//
//********************************* loop *****************************//
//
//
void loop() 
{
  delay(101);
  currentMillis = millis();
  //read the start/stop button
  //    digitalWrite(button_out_pin, HIGH);
  int reading = digitalRead(start_stop_button);
  if (reading+vButton >0) {startButtonPressed = true;}
  if (reading+vButton <= 0) {startButtonPressed = false;}
  if (startButtonPressed == true){
    lastStartButtonState = true;
    digitalWrite(7,HIGH);
  }
  if (startButtonPressed == false && lastStartButtonState == true){
    start_stop = true;
    lastStartButtonState = false;
    digitalWrite(7,LOW);
  }
  
  if(start_stop == true){
    Serial.println("start/stop button pressed");// change this to something meaningful
    buttonReading = !buttonReading; //change the button reading from HIGH to LOW or vice versa
    start_stop = false;
  }
  
  
  //read the lcd buttons
  int raw_button = read_LCD_buttons();
  if (raw_button != btnNONE) {buttonPressed = true;}
  if (raw_button == btnNONE){buttonPressed = false;}
  //else {buttonPressed = false;}
  
  if (buttonPressed == true){
    value_holder = raw_button;
    lastButtonState = true;
  }
  if (buttonPressed == false && lastButtonState == true){
    lcd_key = value_holder;
    lastButtonState = false;
  }
  
  //run the menu after reading the buttons
  menu_fun();
  
  if(buttonPressed==false && lastButtonState == false){
    lcd_key=btnNONE;
    buttonPressed == false;
    lastButtonState = false;
  }
  //set the button states after running the menu   
  lastButtonState = buttonPressed;
  
  digitalWrite(motorPin, motorState);
  digitalWrite(heaterPin, heaterState);
  evaluateSensors();  //force a state change on evaluation of your sensor data, you can even do this at an interval
  print_fun();
  Blynk.run();
  
  if (lastMode != currentMode)
  {
    //doSomething ?
    if(currentMode ==COOL_MODE){
      Blynk.notify("Cooldown Mode");
    }
    else if(currentMode == STOP_MODE){
      Blynk.notify("Stopping");
    }
    else if(currentMode == RUN_MODE){
      Blynk.notify("Starting");
    }
    lastMode = currentMode;
  }
  switch(currentMode){
  case (STANDBY_MODE):
    motorState = LOW;
    heaterState = LOW;
    //menu = 0;
    break;
  case (RUN_MODE):
    cooldown_start_millis = currentMillis;
    motorState = HIGH;
    heater_timeout_check();
    if (inside_temp <= temp_setpoint && heater_timeout == false) {
      heaterState = HIGH;
    }
    else {
      heaterState = LOW;
    }
    break;
  case (COOL_MODE):
    motorState = HIGH;
    // do COOL_MODE stuff
    break;
  case (STOP_MODE):
    motorState = LOW;
    heaterState = LOW;
    // do STOP_MODE stuff
    break;
  } 
  if (!retryRunning && !Particle.connected()){ 
    // if we have not already scheduled a retry and are not connected
    // Serial.println("schedule");
    stopTimer.start();         // set timeout for auto-retry by system
    retryRunning = true;
    retryTimer.start();        // schedula a retry
  }
  
}

//******end loop ******//
//*** read the buttons//
int read_LCD_buttons()
{
  adc_key_in = analogRead(arrow_buttons);      // read the value from the sensor
  // we add approx 50 to those values and check to see if we are close
  if (adc_key_in > 4000) return btnNONE; // We make this the 1st option for speed reasons since it will be the most likely result
  if (adc_key_in < 50)   return btnRIGHT; 
  if (adc_key_in < 700)  return btnUP;
  if (adc_key_in < 1600)  return btnDOWN;
  if (adc_key_in < 2500)  return btnLEFT;
  if (adc_key_in < 4000)  return btnSELECT;  
  return btnNONE;  // when all others fail, return this...
}


//*** connect to particle web ***//
void retryConnect()
{
  if (!Particle.connected())   // if not connected to cloud
  {
    Serial.println("reconnect");
    stopTimer.start();         // set of the timout time
    WiFi.on();
    Particle.connect();        // start a reconnectino attempt
  }
  else                         // if already connected
  {
    Serial.println("connected");
    retryTimer.stop();         // no further attempts required
    retryRunning = false;
  }
}

void stopConnect()
{
  Serial.println("stopped");
  
  if (!Particle.connected()) // if after retryTime no connection
    WiFi.off();              // stop trying and swith off WiFi
  stopTimer.stop();
}

/*wrapper for dht temp sensor*/
void dht_wrapper() {
  DHT.isrCallback();
}

/*blynk functions for setting time*/
BLYNK_WRITE(V6) //Button Widget is writing to pin V1
{
  int pinData = param.asInt(); 
  if (pinData == HIGH){
    time_selected = "auto";
  }
}

BLYNK_WRITE(V7) //Button Widget is writing to pin V1
{
  int pinData = param.asInt(); 
  if (pinData == HIGH){
    time_selected = 90;
  }
}

BLYNK_WRITE(V8) //Button Widget is writing to pin V1
{
  int pinData = param.asInt(); 
  if (pinData == HIGH){
    time_selected = 60;
  }
}

BLYNK_WRITE(V9) //Button Widget is writing to pin V1
{
  int pinData = param.asInt(); 
  if (pinData == HIGH){
    time_selected = 30;
  }
}

/*blynk functions for setting temp*/
BLYNK_WRITE(V10) //Button Widget is writing to pin V1
{
  int pinData = param.asInt(); 
  if (pinData == HIGH){
    temp_selected = "auto";
  }
}

BLYNK_WRITE(V11) //Button Widget is writing to pin V1
{
  int pinData = param.asInt(); 
  if (pinData == HIGH){
    temp_selected = 150;
  }
}

BLYNK_WRITE(V12) //Button Widget is writing to pin V1
{
  int pinData = param.asInt(); 
  if (pinData == HIGH){
    temp_selected = 120;
  }
}

BLYNK_WRITE(V13) //Button Widget is writing to pin V1
{
  int pinData = param.asInt(); 
  if (pinData == HIGH){
    temp_selected = 0;
  }
}

BLYNK_WRITE(V19) //Button Widget is writing to pin V1
{
  int pinData = param.asInt(); 
  if (pinData == HIGH){
    vButton = HIGH;
  }
  else{
    vButton = LOW;
  }
}


void sendDryness()
{
  // This function sends Arduino up time every 1 second to Virtual Pin (V5)
  // In the app, Widget's reading frequency should be set to PUSH
  // You can send anything with any interval using this construction
  // Don't send more that 10 values per second
  Blynk.virtualWrite(V0, currentModeText[currentMode]);
  Blynk.virtualWrite(V1, map(average_moisture,35,100,100,0));
  Blynk.virtualWrite(V2, time_remaining);
  Blynk.virtualWrite(V3, temp_setpoint);
  Blynk.virtualWrite(V14, buttonReading*255);
  Blynk.virtualWrite(V15, motorState*255);
  Blynk.virtualWrite(V16, heaterState*255);
  Blynk.virtualWrite(V17, temp_selected);
  Blynk.virtualWrite(V18, time_selected);
}

void evaluateSensors(){ 
  //read_on_off();  
  total = total - readings[reading_idx];
  if (currentMillis - previous_meas_millis >= meas_interval) {
    previous_meas_millis = currentMillis;
    int result = DHT.acquireAndWait();
    readings[reading_idx] = read_inside_moisture();
    read_inside_temp();
    check_time_remaining();
    check_temp_setting();
    sendDryness();
  }
  total = total + readings[reading_idx];
  reading_idx = reading_idx + 1;
  if (reading_idx >= numReadings){
    reading_idx = 0;
  }
  average_moisture = total / numReadings;
  
  if (buttonReading == LOW){
    currentMode = STANDBY_MODE;
  }
  
  if (buttonReading == HIGH && time_remaining >0){
    currentMode = RUN_MODE;
  }
  
  if (buttonReading == HIGH && time_remaining <=0 && time_remaining > -5){
    currentMode = COOL_MODE;
  }
  if (currentMode == COOL_MODE && (currentMillis - cooldown_start_millis) > 300000){
    currentMode = STOP_MODE;
  }
  if (buttonReading == HIGH && time_remaining <= -5) {
    currentMode = STOP_MODE;
  }
}

int check_time_remaining(){
  Serial.println("check_time_remaining");
  time_elapsed = currentMillis - previous_run_millis;
  if (prev_time_selected != time_selected){
    Serial.println("prev_time_selected != time_selected");
    prev_time_selected = time_selected;
    time_elapsed = 0;
    previous_run_millis = currentMillis;
    if(time_selected == "auto"){
      Serial.println("auto_time");
      auto_time();
    }
    else{ 
      Serial.println("time_remaining = time_selected.toInt()");
      time_remaining = time_selected.toInt();
    }
  }
  else if(prev_time_selected == "auto"){
    Serial.println("auto_time2");
    auto_time();
  }
  else{ 
    time_remaining = time_selected.toInt() - (time_elapsed/60000);
  }
  return time_remaining;
}

int check_temp_setting(){
  if (prev_temp_selected != temp_selected){
    prev_temp_selected = temp_selected;
    if(temp_selected =="auto"){
      auto_heat();
    }
    else{
      temp_setpoint = temp_selected.toInt();
    }
  }
  else if (temp_selected == "auto"){
    auto_heat();
  }
  else {
    temp_setpoint = temp_selected.toInt();
  }
  return temp_setpoint;
}


int auto_time(){
  Serial.println("auto time");
  time_remaining = constrain(map(average_moisture, 35,52,0,60),0,60);
  return time_remaining;
}


int auto_heat() {
  Serial.println("auto heat");
  if (average_moisture < 35)
  {
    temp_setpoint = 0;
  }
  else
  {
    temp_setpoint = 10 * (constrain(map(average_moisture, 35, 52, 100, 160), 100, 160) / 10);
  }
  return temp_setpoint;
}



float read_inside_temp() {
  // Read temperature as farenheit
  inside_temp = DHT.getFahrenheit();
  // Check if any reads failed and exit early (to try again).
  if (isnan(inside_temp)) {
    //lcd.print("Failed to read from DHT sensor!");
    Serial.println("Failed to read from DHT sensor!");
  }
  return inside_temp; 
}

float read_inside_moisture() {
  inside_moisture = DHT.getHumidity();
  if (isnan(inside_moisture)) {
    //lcd.print("Failed to read from DHT sensor!");
    Serial.println("Failed to read from DHT sensor!");
  }
  return inside_moisture;
}

boolean heater_timeout_check() {
  //heaterState = digitalRead(heaterPin);
  if (heaterState == HIGH) {
    heater_last_used_millis = currentMillis;
    heater_timeout = false;
  }
  
  else if (heaterState == LOW && (currentMillis - heater_last_used_millis >= heater_timeout_interval)) {
    heater_timeout = false;
  }
  else {
    heater_timeout = true;
  }
  return heater_timeout;
}


int menu_fun() {
  if (lcd_key != 0){menu_last_clicked_millis=currentMillis;}
  if ( (currentMillis - menu_last_clicked_millis >= 60000)){
    menu = 999;
  }
  
  if ( (currentMillis - menu_last_clicked_millis >= 600000)){
    //lcd.setBacklight(OFF);
    // digitalWrite(LCD_backlight,LOW);
  }  
  if(lcd_key !=0 ){
    //lcd.setBacklight(ON);
    //digitalWrite(LCD_backlight,HIGH);
    menu_last_clicked_millis = currentMillis;
    if(menu == 999){
      lcd.clear();
      menu = 0;
    }
  }
  if ( lcd_key == btnRIGHT) {
    menu++;
  }
  if (lcd_key ==  btnLEFT) {
    menu--;
  }
  if (menu > 2 && menu < 100) {
    menu = 0;
  }
  if (menu < 0) {
    menu = 2;
  }
  if (menu > 106 && menu < 200) {
    menu = 101;
  }
  if (menu < 101 && menu > 99) {
    menu = 106;
  }
  if (menu > 205 && menu < 300) {
    menu = 201;
  }
  if (menu < 201 && menu > 199) {
    menu = 205;
  }
  switch (menu) {
  case 0: //time
    //lcd.clear();
    lcd.setCursor(0,0);
    lcd.write(ARROW_ICON_IDX);
    lcd.print("time ");
    lcd.print("heat         ");
    //if (time_selected && temp_selected){
    lcd.setCursor(0,1);
    lcd.print("time:");
    lcd.print(time_selected);
    lcd.print(" temp:");
    lcd.print(temp_selected);//}
    if ( (lcd_key == btnSELECT ||lcd_key == btnDOWN)) {
      lcd.clear();
      menu=101;
    }
    break;
  case 1: //heat
    lcd.setCursor(0,0);
    lcd.print(" time");
    lcd.write(ARROW_ICON_IDX);
    lcd.print("heat        ");
    //if (time_selected && temp_selected){
    lcd.setCursor(0,1);
    lcd.print("time:");
    lcd.print(time_selected);
    lcd.print(" temp:");
    lcd.print(temp_selected);//}
    if ( (lcd_key == btnSELECT || lcd_key == btnDOWN)) {
      lcd.clear();
      menu = 201;
    }
    //prev_menu = 1;
    break;
  case 101: //time auto
    lcd.setCursor(0,0);
    lcd.write(CLOCK_ICON_IDX);
    lcd.print("time           ");
    lcd.setCursor(0, 1);
    lcd.write(ARROW_ICON_IDX);
    lcd.print("Auto 90 60 45 30 15");
    
    if (lcd_key == btnUP) {
      lcd.clear();
      menu = 0;
    }
    if ( lcd_key == btnSELECT) {
      time_selected = "auto";
      lcd.clear();
      lcd.print("Auto Time");
      delay(1000);
      lcd.clear();
      menu =1;
    }
    break;
  case 102: //time 90
    lcd.setCursor(0,0);
    lcd.write(CLOCK_ICON_IDX);
    lcd.print("time           ");
    lcd.setCursor(0, 1);
    lcd.print(" Auto");
    lcd.write(ARROW_ICON_IDX);
    lcd.print("90 60 45 30 15");
    if ( lcd_key == btnUP) {
      lcd.clear();
      menu = 0;
    }
    if ( lcd_key == btnSELECT) {
      time_selected = 90;
      lcd.clear();
      lcd.print("90 Minutes");
      delay(1000);
      lcd.clear();
      menu =1;
    }
    break;
  case 103: //time 60
    lcd.setCursor(0,0);
    lcd.write(CLOCK_ICON_IDX);
    lcd.print("time           ");
    lcd.setCursor(0, 1);
    lcd.print(" Auto 90");
    lcd.write(ARROW_ICON_IDX);
    lcd.print("60 45 30 15");
    if (lcd_key == btnUP) {
      lcd.clear();
      menu = 0;
    }
    if ( lcd_key == btnSELECT) {
      time_selected = 60;
      lcd.clear();
      lcd.print("60 Minutes");
      delay(1000);
      lcd.clear();
      menu = 1;
    }
    break;
  case 104: //time 45
    lcd.setCursor(0,0);
    lcd.write(CLOCK_ICON_IDX);
    lcd.print("time           ");
    lcd.setCursor(0, 1);
    lcd.print(" Auto 90 60");
    lcd.write(ARROW_ICON_IDX);
    lcd.print("45 30 15");
    if (lcd_key == btnUP) {
      lcd.clear();
      menu = 0;
    }
    if ( lcd_key == btnSELECT) {
      time_selected = 45;
      lcd.clear();
      lcd.print("45 Minutes");
      delay(1000);
      lcd.clear();
      menu =1;
    }
    break;
  case 105: //time30
    lcd.setCursor(0,0);
    lcd.write(CLOCK_ICON_IDX);
    lcd.print("time           ");
    lcd.setCursor(0, 1);
    lcd.print("90 60 45");
    lcd.write(ARROW_ICON_IDX);
    lcd.print("30 15  ");
    if (lcd_key == btnUP) {
      lcd.clear();
      menu = 0;
    }
    if ( lcd_key == btnSELECT) {
      time_selected = 30;
      lcd.clear();
      lcd.print("30 Minutes");
      delay(1000);
      lcd.clear();
      menu =1;
    }
    break;
  case 106:  //time 15
    lcd.setCursor(0,0);
    lcd.write(CLOCK_ICON_IDX);
    lcd.print("time           ");
    lcd.setCursor(0, 1);
    lcd.print("90 60 45 30");
    lcd.write(ARROW_ICON_IDX);
    lcd.print("15  ");
    if (lcd_key == btnUP) {
      lcd.clear();
      menu = 0;
    }
    if (lcd_key == btnSELECT) {
      time_selected = 15;
      lcd.clear();
      lcd.print("15 Minutes");
      delay(1000);
      lcd.clear();
      menu =1;
    }
    break;
  case 201: //heat auto
    lcd.setCursor(0,0);
    lcd.write(FLAME_ICON_IDX);
    lcd.print("heat           ");
    lcd.setCursor(0, 1);
    lcd.write(ARROW_ICON_IDX);
    lcd.print("Auto Hi Med Low Off");
    if (lcd_key == btnUP) {
      lcd.clear();
      menu = 0;
    }
    if (lcd_key == btnSELECT) {
      temp_selected = "auto";
      lcd.clear();
      lcd.print("Auto heat");
      delay(1000);
      lcd.clear();
      menu =0;
    }
    
    break;
  case 202:  //heat hi
    lcd.setCursor(0,0);
    lcd.write(FLAME_ICON_IDX);
    lcd.print("heat           ");
    lcd.setCursor(0, 1);
    lcd.print(" Auto");
    lcd.write(ARROW_ICON_IDX);
    lcd.print("Hi Med Low Off");
    if (lcd_key == btnUP) {
      lcd.clear();
      menu = 0;
    }
    if (lcd_key == btnSELECT) {
      temp_selected = 150;
      lcd.clear();
      lcd.print("High heat");
      delay(1000);
      lcd.clear();
      menu = 0;
    }
    break;
  case 203: //heat med
    lcd.setCursor(0,0);
    lcd.write(FLAME_ICON_IDX);
    lcd.print("heat           ");
    lcd.setCursor(0, 1);
    lcd.print(" Auto Hi");
    lcd.write(ARROW_ICON_IDX);
    lcd.print("Med Low Off");
    if ( lcd_key == btnUP) {
      lcd.clear();
      menu = 0;
    }
    if ( lcd_key == btnSELECT) {
      temp_selected = 130;
      lcd.clear();
      lcd.print("Medium heat");
      delay(1000);
      lcd.clear();
      menu = 0;
    }
    break;
  case 204:  //heat low
    lcd.setCursor(0,0);
    lcd.write(FLAME_ICON_IDX);
    lcd.print("heat           ");
    lcd.setCursor(0, 1);
    lcd.print("Hi Med");
    lcd.write(ARROW_ICON_IDX);
    lcd.print("Low Off  ");
    if ( lcd_key == btnUP) {
      lcd.clear();
      menu = 0;
    }
    if ( lcd_key == btnSELECT) {
      temp_selected = 110;
      lcd.clear();
      lcd.print("Low heat");
      delay(1000);
      lcd.clear();
      menu = 0;
      
    }
    break;
  case 205: //heat off
    lcd.setCursor(0,0);
    lcd.write(FLAME_ICON_IDX);
    lcd.print("heat           ");
    lcd.setCursor(0, 1);
    lcd.print("Hi Med Low");
    lcd.write(ARROW_ICON_IDX);
    lcd.print("Off  ");
    if ( lcd_key == btnUP) {
      lcd.clear();
      menu = 0;
    }
    if ( lcd_key == btnSELECT) {
      temp_selected = 32;
      lcd.clear();
      lcd.print("Heat Off");
      delay(1000);
      lcd.clear();
      menu = 0;
    }
    break;
    
  case 999: //display in run mode
    lcd.setCursor(0,0);
    lcd.write(CLOCK_ICON_IDX);
    lcd.print(":");
    lcd.print(time_remaining);
    lcd.print(" dry:");
    lcd.print(map(average_moisture,35,100,100,0));
    lcd.print(" ");
    lcd.print(currentModeText[currentMode]);
    //if (time_selected && temp_selected){
    lcd.setCursor(0,1);
    lcd.print("time:");
    lcd.print(time_selected);
    lcd.print(" temp:");
    lcd.print(temp_selected);//}
    if (lcd_key !=0 ) {
      lcd.clear();
      //menu=0;
    }
    break;
  }
}



void print_fun(){
  if (currentMillis - previous_print_millis >= printInterval) {
    previous_print_millis = currentMillis;
    
    Serial.print("time=");
    Serial.print(currentMillis / 1000);
    Serial.print(", ");
    Serial.print("currentMode=");
    Serial.print(currentModeText[currentMode]);
    Serial.print(", ");
    Serial.print("inside_temp=");
    Serial.print(inside_temp);
    Serial.print(", ");
    Serial.print("inside_moisture=");
    Serial.print(inside_moisture);
    Serial.print(", average= ");
    Serial.print(average_moisture);
    Serial.print(", ");
    Serial.print("time_selected=");
    Serial.print(time_selected);
    Serial.print(", ");
    Serial.print("time_remaining=");
    Serial.print(time_remaining);
    Serial.print(", ");
    Serial.print("temp_selected=");
    Serial.print(temp_selected);
    Serial.print(", ");
    Serial.print("temp_setpoint=");
    Serial.print(temp_setpoint);
    Serial.print(", ");
    Serial.print("heater on?=");
    Serial.print(heaterState);
    Serial.print(", ");
    Serial.print("motor on?=");
    Serial.print(motorState);
    Serial.print(", ");
    Serial.print("button pressed?=");
    Serial.print(buttonReading);
    Serial.print(", ");
    Serial.print("heater timeout?=");
    Serial.print(heater_timeout);
    Serial.print(", cooldown start=");
    Serial.println(cooldown_start_millis / 1000);
  }
}