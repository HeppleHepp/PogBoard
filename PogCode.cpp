/*/
/////////////////////////////////////////////////////////////////////////
|                                 LOKI                                  |
|                         Developed by Ben Hepple                       |
|                         Contact: coalescence.gp@gmail.com             |
/////////////////////////////////////////////////////////////////////////
/*/

#include <math.h>                             // Include the inbuilt Maths library
#include <SD.h>                               // Include the SD card communication library
#include <Wire.h>                             // Wire library for the LCD I2C Communication                
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,20,4);             // Tells the Arduino the size of the LCD display (0x27) is the default address of I2C port

/////////////////////////////////////////////////////////////////////////
/*                                                                     *\ 
                User variables to entered (Change these)
\*                                                                     */
/////////////////////////////////////////////////////////////////////////

const int SetCurrentLimit       = 21;         // 
const int OUTPUT_UPPER          = 255;        // If you are using Loki as a data logger then these are not required :)
const int OUTPUT_LOWER          = 30;         // The lower and higher values for the pwm output. Most PWM's e.g. the 4QD                                            
const float ref_voltage         = 5;          // 5 Volt ref for readings
const float APB                 = 0.31;       // Amps per Bit (A calibrated thing)
const float totalWheelCirumference = 1.194;   // Wheel diameter in meters (Tyre inflated)
const int Ratio                 = 3;          // calculate gear ratio using DrivenGear/SprocketGear  <<<<-------- YOU MUST CHANGE THIS

////////////////////0/////////////////////////////////////////////////////
/*                                                                     *\ 
                Variable definitions (Do not change)
\*                                                                     */
/////////////////////////////////////////////////////////////////////////


volatile unsigned long motorPoll       = 0;   // A counter that only increases and reset when the RPM function is called
unsigned long lastMotorSpeedPollTime   = 0;   
const int AMPSENSOR_CAL_DELAY          = 3000;
int  currentSensorOffset               = 0;    //offset value for the current sensor
unsigned long Atime         = 0;
float CurrentLimit          = 0;              //intial Current limit upon startup
float batteryVoltage        = 0;              // variable to store battery voltage function
float motorVoltage          = 0;
float Current               = 0;              // variable to store current function  
float prev                  = 0;
float mTarget               = 0;
float multiFac              = 0.5;                                                                                                              
float ATotal                = 0;
long pasttime               = 0;
int MotorRPM                = 0;
int pwm                     = 0;               
int warp                    = 0;
int gmem                    = 0;
int pmem                    = 0;
int tmem                    = 1;
int throtGo                 = 0;
int Boost                   = 1;
int throttle                = 1;
int BoostLimit              = 0;
String Mode                 = "#";
unsigned long Rtime         = 0;              // Timers\/\/\/\/
unsigned long lcdMillis     = 0;
unsigned long dataMillis    = 0;
unsigned long BoostMillis   = 0;
unsigned long contMillis    = 0;
//unsigned long RPM_Millis  = 0;
const int lcdInterval       = 500;
const int dataInterval      = 150;
const int BoostInterval     = 400;
const int contInterval      = 0;             // VERY IMPORTANT make sure this is set to below 200 to avoid stupid thangs.
//const int RPM_Interval    = 500;


          
#define VREF            A1                    // Input reference for Current sensor
#define chipSelect      10                    // define the chipselect for the SD card
#define Battery_Voltage A3                    // define battery voltage read pin
#define Boost_Pin       7                     // switch for override
#define throttlePin     6                     // PTM Throttle mode 
#define Amp_In          A0                    // Current sensor read pin
#define PWM_PIN         9                     // pwm output pin
#define ONE_WIRE_BUS    3                     // Temp sensor pin
#define Pot1_IN         A7
#define Motor_Voltage   A6         


/////////////////////////////////////////////////////////////////////////
/*                                                                     *\ 
                              SETUP
\*                                                                     */
/////////////////////////////////////////////////////////////////////////

void setup() 
{
  analogReference(EXTERNAL);                            // Uses the external reference of the 5V supplied by the TSR 1-2450
  
  Serial.begin(115200);                                               // set baud rate of the arduino (can be 9600)
  lcd.init();                                                         // initialise the lcd 
  lcd.backlight();                                                    // turn on the back light of the lcd
  
  attachInterrupt(2, RPM, RISING);                                    // Set up the external interrupt on pin 2 (Interrupt pins are labled 0,1,2 etc from the pin they start; in this case it's pin 2 so that is 0 and pin 3 is 1)
  
  pinMode(Amp_In, INPUT);                                             ////////////////////////////////// 
  pinMode(Battery_Voltage, INPUT);
  
  pinMode(VREF, INPUT);
  pinMode(chipSelect, OUTPUT);                                        //Set Pins to input or output

  
  pinMode(throttlePin, INPUT);
  pinMode(Boost_Pin, INPUT);
  
  pinMode(Pot1_IN, INPUT);
  pinMode(Motor_Voltage, INPUT);                                            //////////////////////////////////

  lcd.setCursor(0, 0);                                                // Display wake up to notify the board is working
  lcd.print(F("POGGERS"));
  delay(100);
  lcd.clear();  

  lcd.setCursor(2,0);

  delay(100);
  lcd.clear();
    
  if(!SD.begin(chipSelect))                                           // See if the card is present and can be initialized:
  {
    delay(30);
    //while(1);
    lcd.print(F("SD Failed"));                                        // You can activate this while statement if you want the program to terminate if the SD card is not inserted (I do not recommend this)
  }
  else{                                           
  
    lcd.print(F("SD Success"));                                       
    
  }

  long temp = 0;
  int nReadings = AMPSENSOR_CAL_DELAY / 100; //Ten Readings Per Second
  for (int i = 0; i < nReadings; i++)
  {
    temp += analogRead(Amp_In);
    delay(AMPSENSOR_CAL_DELAY / nReadings);
  }
  currentSensorOffset = (float) temp / nReadings;
  
  delay(100);
  lcd.clear();
  
   
  lcd.setCursor(0,1);
  lcd.print("Dababy: Let's GO!");                                                   // Notify successful boot sequence
  delay(500);
  
  lcd.clear();  
                                                                      //////////////////////////////////
  lcd.setCursor(0,0);
  lcd.print(F("A:"));

  lcd.setCursor(0,1);
  lcd.print(F("M:"));
  
  lcd.setCursor(11,0);
  lcd.print(F("MPH:"));
                                                                      // Formatting the LCD Display so we don't have to clear it everytime we want to update it
  lcd.setCursor(11,1);
  lcd.print(F("BTL:"));
  
  lcd.setCursor(0,2);
  lcd.print(F("V:"));
  
  lcd.setCursor(11,2);
  lcd.print(F("RPM:"));
  
  lcd.setCursor(11,3);
  lcd.print(F("PWM:"));                                               //////////////////////////////////
  delay(100);  

  digitalWrite(chipSelect,LOW);                                                     // Begin SPI interfacing by taking Chip Select LOW
  
  String Spacer = ",,,,,,,,";
  File dataFile = SD.open("datalog.txt", FILE_WRITE);                               // Write the datastring to the SD card 
  if(dataFile)                                                                      // check if the file has been created successfully or if it already exists
  {
    dataFile.println(Spacer); 
  }
  else
  {
    Serial.println("error opening datalog.txt");
  }
  
}

/////////////////////////////////////////////////////////////////////////
/*                                             *\ 
                    Main Loop
\*                                             */
/////////////////////////////////////////////////////////////////////////
  
void loop()
{


  batteryVoltage = readBatteryVoltage();
  motorVoltage = readMotorVoltage();
  Pot1();
  TimedEvents(); 
  readCurrent(); 


  throttle = digitalRead(throttlePin);
  Boost = digitalRead(Boost_Pin);

  CurrentLimit = SetCurrentLimit;
  warp = 2;
  
/*/
 if (millis() >= contMillis + contInterval)
 {
    contMillis += contInterval;
    pmem = 1;
    
 }    
 else
 {
    pmem = 0;  
 }
  /*/
  if (throttle == 0 || Boost == 0 )
  {   
    
    if (throtGo == 0 && Boost == 1 && Current<=CurrentLimit)   //if this is the first time in loop 
    {
        pwm = 135;                                              //roughly 45% throttle
        mTarget = 8;                   
        throtGo = 1; 
        Mode = "5"; 
        lcd.setCursor(0,3);
        lcd.print("AUTO");
    }   

    else if (Boost == 0)
    {
        warp = 3;
        Mode = "10"; 
        if(tmem == 1)
        {
          pwm = 135;                                            //roughly 45% output throttle
          tmem = 0; 
          lcd.setCursor(0,3);
          lcd.print("BOOST");          
        }
    }

    switch(warp)
    {         


          
    case 2:

        readCurrent();
        if (Current > CurrentLimit)
        {
            pwm -= 1;  
            break;                
        }
        else if (Current < CurrentLimit)
        {
          pwm += 1; 
          break;
        }
        
        
               
      case 3:
        if(Current>BoostLimit)
        {
            pwm -= 1;
            break;
        }
        else if (Current<=BoostLimit)
        {
          pwm += 1;
          break;
        }     
    }
  }
  else
  {
    tmem = 1;
    pwm = 0;
    throtGo = 0;
    gmem = 0;
    if (motorVoltage >= 2)
    {
        Mode = "40";
    }
    else{
        Mode = "0"; 
    }
    lcd.setCursor(0,3);               
    lcd.print("     "); 
  } 



  if (pwm >= OUTPUT_UPPER)
  {
    pwm = OUTPUT_UPPER;
  }
  if (pwm <= OUTPUT_LOWER)
  {
    pwm = OUTPUT_LOWER;
  }
  //Serial.println(pwm);
  analogWrite(PWM_PIN,pwm); //Upload calculated pwm value      
}
     


/////////////////////////////////////////////////////////////////////////
/*                                                                     *\ 
                    Functions
\*                                                                     */
/////////////////////////////////////////////////////////////////////////

void readCurrent ()
{
  for (uint8_t i = 0; i < 4; i++)
  {
    float difference = 0;                                         // Reset previous calculation
    float tempCurrent = analogRead(Amp_In);                       // Read the ADC value from the current sensor (2.5V is 0A --> 5V is 150A)
    int VRefRead = analogRead(VREF);                              // Read reference voltage to find the difference between the Amp_In pin and the VRef Pin which gives the 10 bit difference in voltage
    difference = (tempCurrent  - VRefRead) - 2;                   // calculate that difference
    tempCurrent = difference*APB;                                 // Multiply the difference by the amps per bit value. (This must be calculated) 
    tempCurrent = constrain(tempCurrent,0,250);                   // Constrain so there are no error values on the LCD screen
    prev += tempCurrent;                                          // Use the Current global variable to store the Current value   
  }
  Current = prev/4;
  prev = 0;
  calc_amp();
}

float readBatteryVoltage()
{ 
   float tempVoltage = analogRead(Battery_Voltage);               // Reads the 10 Bit ADC Value from the battery read pin (A0)

                                                                  // Calculates the voltage seen on the A0 Pin (Up to 5V given we are using the TSR 1-2450)
   tempVoltage = (tempVoltage/1024)*ref_voltage;                  // This is a multiplier which is related to the potential divider ratio. 
   tempVoltage = (tempVoltage * 6) + 0.35;
                                                                  // For us, this ratio is 1/6 which means we step the voltage from 30V --> 5V so the arduino can read. 
                                                                                                                                   
   tempVoltage = constrain(tempVoltage,0,30);                     // Gets rid of zero errors so the LCD display doesn't bug out
   return (tempVoltage);                                          // Returns the voltage to the function readBatteryVoltage() 
}

float readMotorVoltage()
{
  
   float tempVoltage = analogRead(Motor_Voltage);                 // Reads the 10 Bit ADC Value from the battery read pin (A0)

                                                                  // Calculates the voltage seen on the A0 Pin (Up to 5V given we are using the TSR 1-2450)
   tempVoltage = (tempVoltage/1024)*ref_voltage;                  // This is a multiplier which is related to the potential divider ratio. 
   tempVoltage = (tempVoltage * 6) + 0.2;
                                                                  // For us, this ratio is 1/6 which means we step the voltage from 30V --> 5V so the arduino can read. 
                                                                  // Gets rid of zero errors so the LCD display doesn't bug out
   tempVoltage = batteryVoltage - tempVoltage;
   float value = constrain(tempVoltage,0,30);
   return (value);                                                // Returns the voltage to the function readBatteryVoltage()
  
}

float readMotorRPM()
{
  int tempMotorPoll = motorPoll;                                                    // Records the number of interrupts that have occured since the last function recall  
  motorPoll = 0;                                                                    // Reset previous value
  
  unsigned long tempLastMotorPollTime = lastMotorSpeedPollTime;                     // Use variables to remember the time when the last RPM was calculated.
  
  lastMotorSpeedPollTime = millis();                                                // Record the current time to be used in the this cycle and the next.
  float motorRevolutions = tempMotorPoll / 4;                                       // Now calculate the number of revolutions of the motor shaft
  
  motorRevolutions = motorRevolutions * 60.0;                                       // Calculate the revolutions in one minute
  float timeDifference = (lastMotorSpeedPollTime - tempLastMotorPollTime)/1000.0;   // Find the time difference in seconds
    
  float ShaftRPM = motorRevolutions / timeDifference;                               // Finally calculate RPM 
  return (ShaftRPM);
}


void display_LCD()                                                                  // Function to update the LCD display 
{
    
  float Speed = (((MotorRPM/Ratio)*1.194*60)/1.609)/1000;                           // calculation to find the speed of the car in MPH using the Motor Shaft RPM   
  int pwmPerc = pwm - OUTPUT_LOWER;                                                 // calculating the percentage output of the pwm 
  pwmPerc = pwmPerc/1.6; 

  lcd.setCursor(6,2); 
  lcd.print(" ");
  lcd.setCursor(2,0);                                                               // lcd.setCursor updates the location of the 'Cursor'
  lcd.print(Current);                                                               // lcd.print will then display the variable or string chosen on that cursor location
 
  lcd.setCursor(15,3);
  lcd.print("   ");
  lcd.setCursor(15,3);
  lcd.print(pwmPerc);

  lcd.setCursor(15,0);
  lcd.print("     ");  
  lcd.setCursor(15,0);
  lcd.print(Speed);

  lcd.setCursor(15,2);
  lcd.print("     ");
  lcd.setCursor(15,2);
  lcd.print(MotorRPM);
     
  lcd.setCursor(6,2);
  lcd.print(" ");                                                                   // request the AmpHours be calculated
  lcd.setCursor(2,1);
  lcd.print(motorVoltage);
  
  lcd.setCursor(15,1); 
  lcd.print("    ");
  if (BoostLimit == 55)
  {
    lcd.setCursor(15,1); 
    lcd.print("MAX");
  }
  else
  {
    lcd.setCursor(15,1); 
    lcd.print(BoostLimit);
  }


  lcd.setCursor(6,2); 
  lcd.print(" ");
  lcd.setCursor(2,2);
  lcd.print(batteryVoltage);


}

void save_Data()                                                                    // Function to record all important data
{
  Rtime = millis();                                                                 // Find the current time the the data was recorded
  int pwmPerc = pwm - OUTPUT_LOWER;                                                 // calculating the percentage output of the pwm 
  pwmPerc = (pwmPerc/1.6);
  String com = ",";  
  digitalWrite(chipSelect,LOW);                                                     // Begin SPI interfacing by taking Chip Select LOW
  
  String (dataString) = String(Rtime)+(com)+String(batteryVoltage)+(com)+String(Current)+(com)+String(motorVoltage)+(com)+String(MotorRPM)+(com)+String(pwmPerc)+(com)+(Mode)+(com)+String(BoostLimit)+(com)+String(ATotal);   // Data to be written to SD card
    
  File dataFile = SD.open("datalog.txt", FILE_WRITE);                               // Write the datastring to the SD card 
  if(dataFile)                                                                      // check if the file has been created successfully or if it already exists
  {
    dataFile.println(dataString); 
    dataFile.close();
    //Serial.println(dataString);
  }
  
}

void Pot1()
{
  float PotIN = analogRead(Pot1_IN);
  PotIN = map(PotIN,0,1023,0,20);                                                     
  BoostLimit = 14 + PotIN;                                                                    
  if (PotIN <= 2)
  {
    BoostLimit = 10;
  }
  
}

void calc_amp()
{ 
  long nowtime = millis();
  float difference = nowtime - pasttime;
  pasttime = nowtime;
  
  float Atime = (Current*1000) * difference;
  float Ahours = (Atime/1000)/3600;

  ATotal = ATotal + Ahours;
}


void TimedEvents()                                                                  // Seperate function for timers
{
  

  //if (millis() >= RPM_Millis + RPM_Interval)                                        // millis() delay that allows code to run while waiting a set interval that is equal to RPM_Interval
  //{
    //RPM_Millis += RPM_Interval;                                                     // stack the interval to update the delay loop
    //MotorRPM = readMotorRPM();                                                      // read the motorRPM using the function
  //}

/*/
NOTE: using this type of temp sensor can create delays of up to 300ms when using a 12 bit ADC (we use 10 Bit so it's 70ms) which is quite significant when controlling the car and searching for throttle inputs.
      To get rid of such a delay, it would be wise to use a sensor that does not rely on another library to gather the data as that is when unknown delays can occur as it isn't your code.
/*/

  if (millis() >= lcdMillis + lcdInterval)                                          // Timer that updates the display and saves the data to the SD card every half a second (500ms)     
  {
     lcdMillis += lcdInterval;
     display_LCD();
     MotorRPM = readMotorRPM();      
  }

  if (millis() >= dataMillis + dataInterval)  
  {
    dataMillis += dataInterval;
    save_Data();
  }
  
}

// External interrupt triggered by the infrared sensor.
    
void RPM()                                                   
{
  motorPoll++;
}
