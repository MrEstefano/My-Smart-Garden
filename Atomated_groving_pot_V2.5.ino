/************************************************************************************************************************************************/
/*            File Name    : My Smart Garden - Automated growing pot system V2.25                                                                 */
/*                                                                                                                                              */
/*            Description  : This code is written for ATmbega328 microcontroller operating with values recording from                           */
/*                           capacitive Soil moisture sensor, interfaced to a water pump via L298N DC driver,                                   */
/*                           BME 280 Temp and Humid. sensor, interfaced to a cooling fan via PWM pulse by L298N DC motor driver,                */
/*                           HC-SR04 Ultrasonic distance meter checking height of water level in a tank and                                     */
/*                           level is interfaced via 74HC595 Shift Register lighting LED Array of 10 LED's and                                  */
/*                           at same time displayed on 16x2 LCD screen via I2C module, Resistive light intensity sensor via                     */
/*                           wide voltage LM393 comparator, interfaced to 3W growing LED's.                                                     */
/*                           There are two serial communication instructions, first is hardware accommodating FT232RL UART module               */
/*                           second is Software serial communication via Bluetooth module HC-06, communicating with Android                     */
/*                           device interfacing with user via custom developed smart device application on MUT App inventor.                    */
/*                           User can overwrite the code variables via App, choosing Manual mode or Auto mode.                                  */
/*                           This code is using "counters" to create delays and debouncing fast changing values, in order to                    */
/*                           minimize the main "loop" time cycle. In development the loop time has been improved from 10s to 50                 */
/*                                                                                                                                              */
/*             Version     : 2.25                                                                                                               */
/*                                                                                                                                              */
/*             Date        : 15 5.2023                                                                                                          */
/*                                                                                                                                              */
/*             Programmer  : Stefan Zakutansky ATU.ie student                                                                                   */
/*                                                                                                                                              */
/************************************************************************************************************************************************/

/************************   Libraries included to accomodate special functions for system and individual module   *******************************/

#include <NewPing.h>
#include <millisDelay.h>
#include <virtuabotixRTC.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

/************************************************ Constant definoitions   *********************************************************************/

#define lightSensor A0        /*Pin assigned to Light intensity sensor via LM393 comperator*/
#define soilMoisturePin A1    /*Pin assigned to Capacitive soiul moisture sensor*/
#define trigPin A2            /*Pin assigned to HC-SR04 Ultrasonic distance meter TRIGGER pin*/
#define echoPin A3            /*Pin assigned to HC-SR04 Ultrasonic distance meter ECHO pim*/
#define CLK 2                 /*Pin assigned to RCLK of 74HC595 Register shifer network*/
#define LATCH 3               /*Pin assigned to SRCLK of 74HC595 Register shifter network*/
#define DATA 4                /*Pin assigned to DS of 74HC595 Register shifter network*/
#define rxPin 6               /*Pin assigned to TX Blutooth HC-06 module*/
#define txPin 5               /*Pin assigned to RX Blutooth HC-06 module*/
#define relay 13              /*Pin assigned to NPN transistor operating Relay to control 3W Groving LED's*/
#define fan_PWM 9             /*Pin assigned to Cooling fan PWM Control connection*/
#define fan 12                /*Pin assigned to Chanel 1 on L298N DC motor driver */
#define waterPump 11          /*Pin assigned to Chanel 2 on L298N DC motor driver */

#define SEALEVELPRESSURE_HPA (1017.25)     /*definition of Sea level Preassure according to GEO location - Celbridge, 27/Apr/2023 */

/**********************************************  Declaration of variables and their initial values   *******************************************/

int auto_mode = 1;                                     /*The flag position of the auto or manual mode selected via app*/
int led_mode = 0;                                      /*The current state of growing LED*/
int fan_mode = 0;                                      /*The current state of cooling fan*/
int pump_mode = 0;                                     /*The current state of water pump*/
int winter_boost_mode;                                 /*The current state of summer or winter solstice*/
int page_counter = 0;                                  /*The counter value for screen switch timing */
int LedLightState = 0;                                 /*The state of LED */
int waterLevelCount = 0;                               /*The counter value for debouncing the water level*/
int moistureCount = 0;                                 /*The counter value for debouncing the soil moisture level*/
int ledCount = 0;                                      /*The counter value for LED On*/
int ledCountOff = 0;                                   /*The counter value for LED Off*/
int lastLightIntensity;                                /*The light intensity value from sensor*/
int wait_count = 11;                                   /*The counter value for delaying each page of the screen*/  
int moisterTreasholdvalue;                              /*The soil moisture value from sensor*/
int moisterValueDebounced;                              /*The counter value for debouncing fluctuating soil moisture values*/
int soilMoistureValue = 0;                              /*The soil moisture values after debounce*/   
int waterPumpState = 0;                                /*The state of water pump*/
int waterLevelValueDebounced = 0;                       /*The debounced value of water level*/   
int distance = 0;                                       /*The distance of the measured object is stored*/
int percentage = 0;                                     /*The percentage of water reservuar liquid*/
int totalColumns = 16;                                 /*variable storing LCD columns*/
int totalRows = 2;                                     /*variable storing LCD rows */
int airTemp;                                            /*The measured air temperature*/
int airHumidity;                                        /*The measured air humidity*/
int lightIntensity;                                     /*The light intensity value */
int ledState = 0;                                      /*The actual state of the LED lights*/
int pumpState = 0;                                     /*The actual state of the water pump*/
int fanState = 0;                                      /*The actual state of the cooling fan*/
int NrOfCycles = 0;                                    /*The count of cycles for Bluetooth duplex communication*/   
int connectionOK = 1;                                  /*The information, if the flag was raised, to transmit the data via Bluetooth*/
int BluetoothData = 0;                                 /*The character, when received from Android app via Bluetooth*/
const int drySoil = 445;                                /*Dry soil moisture fixed value gathered from calibration*/
const int wetSoil = 235;                                /*Wet soil moisture fixed value gathered from calibration*/
const char tank1Full = 2;                               /*Depth measured when the water tank is full*/
const char tank1Empty = 32;                             /*Depth measured when the water tank is empty*/
const char maxDistance = 34;                            /*Distance measured from HC-SR04 Ultrasonic distance meter to bottom of the water tank*/
char shortDelay = 50;                                   /*The value for short delay*/
unsigned long startUp;                                  /*The service routine for timing the main loop*/ 
unsigned long finished;                                 /*The storing service routine for timing the main loop*/
unsigned long elapsed;                                  /*The service routine for timing the main loop*/
unsigned long currentMillis;                            /*The service routine for timing the main loop*/    
String scrollingMessage = "...Program is loading...";   /*The text for static intro screen sequance*/
int barGraph[16] = {0x00001,0x0003,0x00007,0x0000F,     /*The array of vales used in LED array fro disp;laying the water level*/
                   0x00001F,0x0003F,0x0007F,0x000FF,
                   0x001FF,0x003FF,0x007FF,0x00FFF,
                   0x01FFF,0x03FFF,0x07FFF,0x0FFFF};
                   
/***********************************************   Objects/modules created and pins assigned to them  *******************************************************/

virtuabotixRTC myRTC(7, 8, 10);                         /*Software object for DS1302 Timing moduleL "clock,dat,rst" */ 
LiquidCrystal_I2C lcd(0x27, totalColumns, totalRows);   /*Software object created for LCD via I2C Commumication module*/                                 
SoftwareSerial hc06(rxPin,txPin);                       /*Software object created for Bluetooth*/                   
NewPing sonar1(trigPin, echoPin, maxDistance);          /*Software object Creates a HC-SR04 Ultrasonic distance meter*/
Adafruit_BME280 bme;                                    /*Software object Creates a BME 280 sensor*/      

/*************************************************************  User function prototypes *********************************************************************/

void updateLedArray();                                  /*Sub function prototype for updating water level*/
void loopDelay();                                       /*Sub-function prototype for custome delay function*/
void controlFan();                                      /*Sub-function prototype for controlling the Cooling Fan*/

/*********************************************************************  Setup sequance ***********************************************************************/
/*                                                                                                                                                           */
/*                                        Setup sequance is a  part of code, which runs only onece, when system is powered up                                */
/*                                                                                                                                                           */
/*************************************************************************************************************************************************************/
void setup(){
  
  /*                           Set the current date, and time in the following format:                                                                       */
  /*                      seconds, minutes, hours, day of the week, day of the month, month, year                                                            */
  /*                                                                                                                                                         */
  /* myRTC.setDS1302Time(20, 19, 17, 5, 21, 4, 202;                            /*!!! uncomment this line ,once the time calibrated !!!                       */
  
  Serial.begin(9600);                                                          /*Start the system Serial comunication*/ 
  hc06.begin(9600);                                                            /*Start the software serial comunication*/ 
  pinMode(rxPin, INPUT);                                                       /*Set RX pin as input*/ 
  pinMode(txPin, OUTPUT);                                                      /*Set TX pin as output*/ 
  pinMode(lightSensor,INPUT);                                                  /*Set Light sensor pin as input*/ 
  pinMode(relay,OUTPUT);                                                       /*Set LED control pin as output*/ 
  pinMode(trigPin,OUTPUT);                                                     /*Set Trigger pin as output*/
  pinMode(echoPin,INPUT);                                                      /*Set Echo pin as input*/
  pinMode(DATA,OUTPUT);                                                        /*Set DATA pin as Output*/
  pinMode(LATCH,OUTPUT);                                                       /*Set LATCH pin as output*/
  pinMode(CLK,OUTPUT);                                                         /*Set CLK pin as output*/
  pinMode(fan,OUTPUT);                                                         /*Set fan pin as output*/   
  pinMode(fan_PWM,OUTPUT);                                                     /*set fan_PWM as output*/
  pinMode(waterPump,OUTPUT);                                                   /*Set waterPump as output*/     
  pinMode(soilMoisturePin,INPUT);                                              /*Set soilMoisturePin as input*/                                
  digitalWrite(waterPump,0);                                                   /*Initialize the water pump*/
  digitalWrite(fan, 0);                                                        /*Initialize the cooling fan*/
  auto_mode = 1;                                                               /*Initializing the system to AUTO mode*/
  moisterValueDebounced = 90;                                                  /*Initializing soil moisture sensor value after debounce*/  
  moisterTreasholdvalue = 50;                                                  /*Initializing the moisture treashold value*/
  
/***********************************************  Welcome logo displayed with sliding text in second line of LCD ******************************************/ 

  lcd.init();                                                                  /*Initializing the LCD screen*/
  lcd.clear();                                                                 /*clearing the LCD*/
  lcd.backlight();                                                             /*turning the back light on*/
  lcd.setCursor(0, 0);                                                         /*seting the cursor to desired position*/
  lcd.print("My Smart Garden");                                                /*Branded text is desplayed*/
  for (int i=0; i < totalColumns; i++) {                                       /*The loop ads as many spaces as many there are Columns*/
    scrollingMessage = " " + scrollingMessage;                                 /*space is added to text and then assigned to same variable*/
  } 
  scrollingMessage = scrollingMessage + " ";                                   /*Single space is added to the end of the text*/
  for (int position = 0; position < scrollingMessage.length(); position++) {   /*loopwhich moves the text from right to lefte*/
    lcd.setCursor(0, 1);
    lcd.print(scrollingMessage.substring(position, position + totalColumns));
    delay(550);
  }  
  
  if(!bme.begin(0x76)){                                                        /*Checking if BME280 is available*/
    Serial.println("Could not find a valid BME280 sensor, check wiring!");     /*printe message to serial monitor if not*/
  }
  loopDelay(250);     
} 

/******************************************************************* Main loop ******************************************************************************/
void loop() {     
  Serial.println("Start...");                                        /*Printing to serial monitor when main loop starts*/
  startUp = millis();                                                /*Assigning the start of timing function to variable, this will be used at the end of main loop*/
  
/********************************************************************DATE AND TIME****************************************************************************/
  myRTC.updateTime();                                                /* This allows for the update of variables for time or accessing the individual elements*/

 /*************************************************************** SOIL MOISTURE MANAGMENT ********************************************************************/

  soilMoistureValue = analogRead(soilMoisturePin);                   /*Soil moisture sensor is read by analog pin, and assigned to variable*/ 
  soilMoistureValue = map(soilMoistureValue ,drySoil ,wetSoil ,0 ,100);     /*Using "map" function to convert the meassurament range into percantage range 0 - 100*/ 
  soilMoistureValue = constrain(soilMoistureValue,0,100);            /*"constrain" function to lock the scale to desired range 0 to 100, avoiding negative numbers*/
     
  moistureCount++;                                                   /*Debounce counter for moisture sensor reading increments every loop*/
  
  if(moistureCount == 30){                                           /*When counter reaches desired value,then....*/
    moisterValueDebounced = soilMoistureValue;                       /*measured soil moisture is assigned to new variable, for further processing*/    
    moistureCount = 0;                                               /*counter is reset by assigning 0 value to it*/
  }  

  
/*******************************************************************GATHERING TEMPERATURE DATA****************************************************************/
   
  
  airTemp = bme.readTemperature();                                   /*Read temperature and assigne to variable*/
  airHumidity = bme.readHumidity();                                  /*Read humidity and assigne to variable*/  
  loopDelay(shortDelay);                                             /*sending shortDelay to the sub function*/
  


  /*******************************************************************WATER LEVEL IN TANK**********************************************************************/
  
  distance = sonar1.ping_cm();                                       /*assigning received value from ultrasonic sensor Distance = (Time * Speed)/2 to a variable*/
  percentage = map(distance, tank1Empty, tank1Full, 0, 100);         /*Using "map" function to convert the distance into percantage scale*/ 
  percentage = constrain(percentage,0,100);                          /*Using "constrain" function to lock the scale to range 0 to 100, avoiding negative numbers*/

  waterLevelCount++;                                                 /*counter is used to Debounce moisture sensor reading*/
   
  if (waterLevelCount == 80){                                        /*When counter reaches desired value,then....*/
    waterLevelValueDebounced = percentage;                           /*measured vater level is assigned to the new variable, for further processing*/ 
    waterLevelCount = 0;                                             /*counter is reset by assigning 0 value to it*/
  }  
  
  updateLedArray(waterLevelValueDebounced);                          /*Debounced water level value passed to sub function to update the LED array*/
 
  if(percentage < 3){                                                /* Safety precautions - if water is lover then 3% disable the pump*/
    waterPumpState = 0;                                              /* when condition met, 0 is assigned to variable, keeping the pump off regardles*/
    pumpState = 0;                                                   /*update the actual state of the pump variable, which is then sent to App*/
  }
  else{
    waterPumpState = 1;                                              /*When water level is anything above 3 %, variable gets state 1*/
    pumpState = 1;                                                   /*update the actual state of the pump variable, which is then sent to App*/
  }

  /************************************************************* Light Intensity ********************************************************************************/

  lightIntensity = digitalRead(lightSensor);                         /*Reading light intensity sensor connected via wide voltage LM393 comparatotor, resulting in digital state*/
                                                                     /*Treshhold value is set by trimmer on wide voltage comparator LM393 module*/
 
 /******************************************** Instractions for recognition  if AUTO mode or Manual mode selected  **********************************************/
  
  switch(auto_mode){                                                 /*Check if user has switched between MANUAL and AUTO mode via App, using bluetooth communication*/
   case 0:                                                           /*In case it is 0, which means user has choosen MANUAL mode*/
    if(led_mode == 1){                                               /*Check if user turned the Growing LED's ON via App, if yes, then do following*/
      digitalWrite(relay,HIGH);                                      /*Turn ON Growing LED's*/
      ledState = 1;                                                  /*update the status of the Growing Led's so APP interface can be updated*/
    }
    else{                                                            /*When user turn's off growing LED's, the do following...*/
      digitalWrite(relay,LOW);                                       /*Turn OFF Growing LED's*/
      ledState = 0;                                                  /*update the status of the Growing Led's so APP interface can be updated*/
    }
    
    if(fan_mode == 1){                                               /*Check if user turned the Cooling Fan ON via App, if yes, then do following*/
      controlFan(255);                                               /*The cooling fan is turned ON and max PWM value is passed to subfunction*/
    }
    else{                                                            /*When user turn's offthe Cooling Fan, the do following...*/
      controlFan(0);                                                 /*The Cooling fan is turned OFF and min PWM value is passed to subfunction*/
    }

    if(pump_mode == 1){                                              /*Check if user turned the Water pump ON via App, if yes, then do following*/
      digitalWrite(waterPump,HIGH);                                  /*Turn the Water Pump ON*/
      pumpState = 1;                                                 /*Update the status of the Water pump so APP interface can be updated*/
    }
    else{                                                            /*When user turn's off growing LED's, the do following...*/
      digitalWrite(waterPump,LOW);                                   /*Turn the Water Pump OFF*/
      pumpState = 0;                                                 /*Update the status of the Water pump so APP interface can be updated*/
    }
  break;  
  case 1:                                                            /*In case it is 1, which means user has choosen AUTO mode*/
    if(5 > myRTC.hours > 21){                                        /*Check what time it is, If 10pm to 6am then night time is true, then do following.. */      
      digitalWrite(relay,LOW);                                       /*Turn OFF Growing LED's*/
      ledState = 0;                                                  /*update the status of the Growing Led's so APP interface can be updated*/
      }
    else{                                                            /*If the time is between 6am and 10pm then day time is true, do the following..  */      
      if(myRTC.month < 4 || myRTC.month > 9){                        /*Check what month is it, If from September to April then winter time is true */
        winter_boost_mode = 1;                                       /*Update the status of the Boost mode, so APP interface can be updated*/
        switch(myRTC.hours){                                         /*Check what time is it, whitin winter time*/
          case 5 ... 8:                                              /*In case of time is between 5am to 9am, do folllowing*/
            digitalWrite(relay,HIGH);                                /*Turn ON Growing LED's*/
            ledState = 1;                                            /*update the status of the Growing Led's so APP interface can be updated*/
          break;
          case 9 ... 16:                                             /*In case of time is between 9am to 5pm, do folllowing*/
            if(lightIntensity == HIGH ){                             /*Check light intensity, if sensor gives us digital high, do fllowing */                  
              ledCount++;                                            /*Counter is used to Debounce light intensity sensor reading*/
              if(ledCount == 60){                                    /*When counter reaches desired value,then....*/
                LedLightState = lightIntensity;                      /*Light intensity is assigned to variable with debounced state of light*/
                digitalWrite(relay,LedLightState);                   /*Turn ON Growing LED's after light intensity is debounced*/
                ledCount = 0;                                        /*Counter is reset by assigning 0 value to it*/
                ledState = 1;                                        /*Update the status of the Growing Led's so APP interface can be updated*/
              }                                  
            }  
            else{                                                    /*Check light intensity, if sensor gives us digital low, do fllowing */     
              ledCountOff ++;                                        /*Counter is used to Debounce light intensity sensor reading*/
              if(ledCountOff == 60){                                 /*When counter reaches desired value,then....*/
                LedLightState = lightIntensity;                      /*Light intensity is assigned to variable with debounced state of light*/
                digitalWrite(relay,LedLightState);                   /*Turn OFF Growing LED's after light intensity is debounced*/
                ledCountOff = 0;                                     /*Counter is reset by assigning 0 value to it*/
                ledState = 0;                                        /*Update the status of the Growing Led's so APP interface can be updated*/
              }
            }
          break;
          case 17 ... 20:                                            /*In case of time is between 5pm to 9pm,this is winter evening boost, do folllowing*/
            digitalWrite(relay,HIGH);                                /*Turn ON Growing LED's*/
            ledState = 1;                                            /*Update the status of the Growing Led's so APP interface can be updated*/
          break;           
          
          default:                                                   /*if non of the cases happens, do following*/
            digitalWrite(relay,LOW);                                 /*Turn ON Growing LED's*/  
            ledState = 0;                                            /*Update the status of the Growing Led's so APP interface can be updated*/
          break;         
        }
      }
      else if(myRTC.month > 3 && myRTC.month < 10){                  /*Check what month is it, If from April to September then summer time is true */
        winter_boost_mode = 0;                                       /*Update the status of the Boost mode, so APP interface can be updated*/
        switch(myRTC.hours){                                         /*Check what time is it, whitin winter time*/
          case 5 ... 6:                                              /*In case of time is between 5am to 7am, do folllowing*/
            digitalWrite(relay,HIGH);                                /*Turn ON Growing LED's*/
            ledState = 1;                                            /*Update the status of the Growing Led's so APP interface can be updated*/
          break;
          case 8 ... 19:                                             /*In case of time is between 5am to 7am, do folllowing*/
            if(lightIntensity==HIGH ){                               /*Check light intensity, if sensor gives us digital high, do fllowing */   
              ledCount++;                                            /*Counter is used to Debounce light intensity sensor reading*/
              if (ledCount == 60){                                   /*When counter reaches desired value,then....*/
                LedLightState = lightIntensity;                      /*Light intensity is assigned to variable with debounced state of light*/
                digitalWrite(relay,LedLightState);                   /*Turn ON Growing LED's after light intensity is debounced*/
                ledCount = 0;                                        /*Counter is reset by assigning 0 value to it*/
                ledState = 1;                                        /*Update the status of the Growing Led's so APP interface can be updated*/
              }              
            } 
            else{                                                    /*Check light intensity, if sensor gives us digital low, do fllowing */     
             ledCountOff ++;                                         /*Counter is used to Debounce light intensity sensor reading*/
              if(ledCountOff == 60){                                 /*When counter reaches desired value,then....*/
                LedLightState = lightIntensity;                      /*Light intensity is assigned to variable with debounced state of light*/
                digitalWrite(relay,LedLightState);                   /*Turn OFF Growing LED's after light intensity is debounced*/
                ledCountOff = 0;                                     /*Counter is reset by assigning 0 value to it*/
                ledState = 0;                                        /*Update the status of the Growing Led's so APP interface can be updated*/
              }
            }
          break;
          case 20:                                                   /*In case of time is between 8pm to 9pm,this is winter evening boost, do folllowing*/
            digitalWrite(relay,HIGH);                                /*Turn ON Growing LED's*/
            ledState = 1;                                            /*Update the status of the Growing Led's so APP interface can be updated*/
          break; 
          
          default:                                                   /*if non of the cases happens, do following*/
            digitalWrite(relay,LOW);                                 /*Turn ON Growing LED's*/  
            ledState = 0;                                            /*Update the status of the Growing Led's so APP interface can be updated*/
          break;               
        }   
      }
    }    
    if(moisterValueDebounced <= moisterTreasholdvalue){              /*Check if soil moisture drops below or equal the treashold value stored in variable*/
      digitalWrite(waterPump, waterPumpState);                       /*If it does so, activate the pump to water plants*/ 
      loopDelay(shortDelay);                                         /*sending shortDelay to the sub function*/
    }
    else if(moisterValueDebounced > moisterTreasholdvalue){          /*Check if soil moisture drops above the treashold value stored in variable*/
      digitalWrite(waterPump,0);                                     /*If it does so, de-activate the pump to water plants*/ 
      pumpState = 0;                                                 /*Update the status of the Water pump so APP interface can be updated*/ 
      loopDelay(shortDelay);                                         /*sending shortDelay to the sub function*/
     
    }
    controlFan(airTemp);                                             /*send thew actual temperature of enviroment to sub function in order to control Cooling fan*/
    break;     
    
    default:                                                         /*If non of the cases above, do nothing*/
    break;
  }        


/***************************************************************************** LCD LOOP **************************************************************************/
   
  
  if(wait_count == 24){                                             /*Checking if LCD pause counter has desired value (24 x 50 ms = 1.1 s)*/
    wait_count = 0;                                                 /*Zero the LCD pause counter*/ 
    page_counter++;                                                 /*Increase the LCD page counter after the pause*/
    lcd.clear();                                                    /*Clear the LCD after the pause*/
  }
  wait_count++;                                                     /*Increment the LCD pause counter using the main loop iteration*/
  
  switch (page_counter) {                                           /*Check what is the count of the LCD page counter*/
    case 1:                                                         /*In case it is LCD page counter is value 1*/
      lcd.setCursor(1,0);                                           /*Set the cursor*/
      lcd.print("Current Date:");                                   /*Print the text, shown in brackets*/
      lcd.setCursor(3,1);                                           /*Set the cursor*/
      lcd.print(myRTC.dayofmonth);                                  /*Take the curent value from the software object*/
      lcd.print("/");                                               /*Print the text, shown in brackets*/
      lcd.print(myRTC.month);                                       /*Take and display the curent value from the software object*/
      lcd.print("/");                                               /*Print the text, shown in brackets*/
      lcd.print(myRTC.year);                                        /*Take and display the curent value from the software object*/
      break;
    case 2: 
      lcd.setCursor(1,0);                                           /*Set the cursor*/
      lcd.print("Current Time:");                                   /*Print the text, shown in brackets*/
      lcd.setCursor(3,1);                                           /*Set the cursor*/
      lcd.print(myRTC.hours);                                       /*Take and display the curent value from the software object*/
      lcd.print(":");                                               /*Print the text, shown in brackets*/
      lcd.print(myRTC.minutes);                                     /*Take and display the curent value from the software object*/
      lcd.print(":");                                               /*Print the text, shown in brackets*/
      lcd.print(myRTC.seconds);                                     /*Take and display the curent value from the software object*/
      break;
    case 3:  
      lcd.setCursor(1,0);                                           /*Set the cursor*/
      lcd.print("Soil moisture");                                   /*Print the text, shown in brackets*/
      lcd.setCursor(4,1);                                           /*Set the cursor*/
      lcd.print(moisterValueDebounced);                             /*Take and display the curent value stored in the variable*/
      lcd.print(" %");                                              /*Print the text, shown in brackets*/
      break;
    case 4: 
      lcd.setCursor(2,0);                                           /*Set the cursor*/
      lcd.print("Water level");                                     /*Print the text, shown in brackets*/
      lcd.setCursor(4,1);                                           /*Set the cursor*/
      lcd.print(percentage);                                        /*Take and display the curent value stored in the variable*/
      lcd.print(" %");                                              /*Print the text, shown in brackets*/
      break;
    case 5: 
      lcd.setCursor(1,0);                                           /*Set the cursor*/
      lcd.print("Air Temperature");                                 /*Print the text, shown in brackets*/
      lcd.setCursor(4,1);                                           /*Set the cursor*/
      lcd.print(airTemp);                                           /*Take and display the curent value stored in the variable*/
      lcd.print(" C");                                              /*Print the text, shown in brackets*/
      break;
    case 6:  
      lcd.setCursor(1,0);                                           /*Set the cursor*/   
      lcd.print("Soil treshold");                                   /*Print the text, shown in brackets*/
      lcd.setCursor(4,1);                                           /*Set the cursor*/
      lcd.print(moisterTreasholdvalue);                             /*Take and display the curent value stored in the variable*/
      lcd.print(" %");                                              /*Print the text, shown in brackets*/
      break;    
    } 
  if(page_counter == 6){                                            /*Checking if LCD page counter is the count is 6*/
      page_counter = 0;                                             /*Zero the counter*/
    }   
        
/***************************************************************************************************************************************************************/  
   NrOfCycles++;                                                    /*Delaying transmition via software serial by use of main loop iteration*/
                                                                    /*increasing the counter, The Transmition can only execute after 4 receive cycles*/
   
/******************************************************************** *RECEIVING  ******************************************************************************/

  if(hc06.available()> 0){                                          /*Check if something has arrived on Software serial port / Bluetooth*/
   BluetoothData = hc06.read();                                     /*Read the buffer and assign the content to variable*/
   switch(BluetoothData){                                           /*Check what is received*/
    case 'a':                                                       /*In case it is, what's shown in brackets, do so .....*/ 
      moisterTreasholdvalue = 40;                                   /*set moisture treashold value to 40 %*/
      loopDelay(shortDelay);                                        /*Short custom delay*/
      break;
    case 'b':                                                       /*In case it is, what's shown in brackets, do so .....*/ 
      moisterTreasholdvalue = 60;                                   /*set moisture treashold value to 60 %*/
      loopDelay(shortDelay);                                        /*Short custom delay*/
      break;
    case 'c':                                                       /*In case it is, what's shown in brackets, do so .....*/ 
      moisterTreasholdvalue = 80;                                   /*set moisture treashold value to 80 %*/        
      loopDelay(shortDelay);                                        /*Short custom delay*/
      break;
    case 'd':                                                       /*In case it is, what's shown in brackets, do so .....*/ 
      led_mode = 1;                                                 /*update the variable with status of LEDs, its ON */
      loopDelay(shortDelay);                                        /*Short custom delay*/
      break;
    case 'e':                                                       /*In case it is, what's shown in brackets, do so .....*/ 
      led_mode = 0;                                                 /*update the variable with status of LEDs, its OFF */
      loopDelay(shortDelay);                                        /*Short custom delay*/
      break;
    case 'f':                                                       /*In case it is, what's shown in brackets, do so .....*/ 
      auto_mode = 1;                                                /*update the variable with status of control mode, Auto mode its ON */
      loopDelay(shortDelay);                                        /*Short custom delay*/
      break;
    case 'g':                                                       /*In case it is, what's shown in brackets, do so .....*/ 
      auto_mode = 0;                                                /*update the variable with status of control mode, Manual mode its ON */
      loopDelay(shortDelay);                                        /*Short custom delay*/
      break;
    case 'h':                                                       /*In case it is, what's shown in brackets, do so .....*/ 
      fan_mode = 1;                                                 /*update the variable with status of Cooling Fan, its ON */
      loopDelay(shortDelay);                                        /*Short custom delay*/
      break;  
    case 'i':                                                       /*In case it is, what's shown in brackets, do so .....*/ 
      fan_mode = 0;                                                 /*update the variable with status of Cooling Fan, its OFF */
      loopDelay(shortDelay);                                        /*Short custom delay*/
      break;
    case 'j':                                                       /*In case it is, what's shown in brackets, do so .....*/ 
      pump_mode = 1;                                                /*update the variable with status of Water pump, its ON */
      loopDelay(shortDelay);                                        /*Short custom delay*/
      break;
    case 'k':                                                       /*In case it is, what's shown in brackets, do so .....*/ 
      pump_mode = 0;                                                /*update the variable with status of Water pump, its OFF */
      loopDelay(shortDelay);                                        /*Short custom delay*/
      break;
    }
   
  }  

  /***************************************************** Bluetooth transmition of data to Android smart device *****************************************************/
  
  if(NrOfCycles == 4){                                              /*Transmission of data, execute after 4 receiving cycles for App to be ready to receive data*/
    hc06.print(moisterValueDebounced);                              /*Send to App the current value stored in variable in brackets*/
    hc06.print("|");                                                /*Send to App the standardised separator symbol shown in brackets*/
    hc06.print(waterLevelValueDebounced);                           /*Send to App the current value stored in variable in brackets*/ 
    hc06.print("|");                                                /*Send to App the standardised separator symbol shown in brackets*/
    hc06.print(airTemp);                                            /*Send to App the current value stored in variable in brackets*/
    hc06.print("|");                                                /*Send to App the standardised separator symbol shown in brackets*/
    hc06.print(airHumidity);                                        /*Send to App the current value stored in variable in brackets*/
    hc06.print("|");                                                /*Send to App the standardised separator symbol shown in brackets*/
    hc06.print(ledState);                                           /*Send to App the current value stored in variable in brackets*/
    hc06.print("|");                                                /*Send to App the standardised separator symbol shown in brackets*/
    hc06.print(pumpState);                                          /*Send to App the current value stored in variable in brackets*/
    hc06.print("|");                                                /*Send to App the standardised separator symbol shown in brackets*/
    hc06.print(fanState);                                           /*Send to App the current value stored in variable in brackets*/
    hc06.print("|");                                                /*Send to App the standardised separator symbol shown in brackets*/
    hc06.print(winter_boost_mode);                                  /*Send to App the current value stored in variable in brackets*/ 
    hc06.print("|");                                                /*Send to App the standardised separator symbol shown in brackets*/
    NrOfCycles = 0;                                                 /*Zero the Transmition delay counter*/  
    }  
    
  /************************************** Service routine for monitoring and troubleshooting, please comment out when not used **************************************/
  
  finished = millis();                                              /*Assigning the finish of timing function to variable, this will be used at the end of main loop*/
  Serial.println("Timing the loop has finished: ");                 /*Display the text in brackets in Serial Monitor screen*/
  elapsed = finished - startUp;                                     /*Equation to calculate the time of the one complete loop*/
  Serial.print(elapsed);                                            /*Display the current value stored in variable in Serial Monitor screen*/
  Serial.print(" milliseconds elapsed");                            /*Display the text in brackets in Serial Monitor screen*/
  if(auto_mode == 0){                                               /*Check if Auto mode is selected*/
    Serial.print("Automatic mode is ON");                           /*Display the text in brackets in Serial Monitor screen*/
    }
  else{   
    Serial.println("Manual mode is ON");                            /*Display the text in brackets in Serial Monitor screen*/
    }

  Serial.print("Water in storage tank = ");                         /*Display the text in brackets in Serial Monitor screen*/
  Serial.print(percentage);                                         /*Display the current value stored in variable in Serial Monitor screen*/ 
  Serial.println("%");                                              /*Display the text in brackets in Serial Monitor screen*/
  
  Serial.print("Soil moisture = ");                                 /*Display the text in brackets in Serial Monitor screen*/
  Serial.println(soilMoistureValue);                                /*Display the current value stored in variable in Serial Monitor screen*/
 
  Serial.print("Temperature = ");                                   /*Display the text in brackets in Serial Monitor screen*/ 
  Serial.print(bme.readTemperature());                              /*Display the current value stored in variable in Serial Monitor screen*/
  Serial.println("*C");                                             /*Display the text in brackets in Serial Monitor screen*/

  Serial.print("Pressure = ");                                      /*Display the text in brackets in Serial Monitor screen*/ 
  Serial.print(bme.readPressure() / 100.0F);                        /*Display the current value stored in variable in Serial Monitor screen*/
  Serial.println("hPa");                                            /*Display the text in brackets in Serial Monitor screen*/

  Serial.print("Approx. Altitude = ");                              /*Display the text in brackets in Serial Monitor screen*/
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));             /*Display the current value stored in variable in Serial Monitor screen*/
  Serial.println("m");                                              /*Display the text in brackets in Serial Monitor screen*/
 
  Serial.print("Humidity = ");                                      /*Display the text in brackets in Serial Monitor screen*/
  Serial.print(bme.readHumidity());                                 /*Display the current value stored in variable in Serial Monitor screen*/ 
  Serial.println("%");                                              /*Display the text in brackets in Serial Monitor screen*/
  
  if(ledState == 0){                                                /*Check if LEDs is selected*/
    Serial.println("Growing LED is OFF");                           /*Display the text in brackets in Serial Monitor screen*/ 
  }
  else{
    Serial.println("Growing LED is ON");                            /*Display the text in brackets in Serial Monitor screen*/ 
  }
  if(pumpState == 0){                                               /*Check if Water Pump is ON*/
    Serial.println("Water pump is OFF");                            /*Display the text in brackets in Serial Monitor screen*/
  }
  else{
    Serial.println("Water pump is ON");                             /*Display the text in brackets in Serial Monitor screen*/
  }
   if(fanState == 0){                                               /*Check if Cooling Fan is ON*/
    Serial.println("Cooloing fan is OFF");                          /*Display the text in brackets in Serial Monitor screen*/
  }
  else{
    Serial.println("Water pump is ON");                             /*Display the text in brackets in Serial Monitor screen*/
  }
  if(winter_boost_mode == 1){                                       /*Check if winter time is true*/
    Serial.println("Its a winter time");                            /*Display the text in brackets in Serial Monitor screen*/ 
  }
  else{
    Serial.println("Its a summer time");                            /*Display the text in brackets in Serial Monitor screen*/ 
  }
  if(lightIntensity == 0){                                          /*Check the intensity of light*/
    Serial.println("Light intensity is Low");                       /*Display the text in brackets in Serial Monitor screen*/ 
  }
  else{
    Serial.println("Light intensity is High");                      /*Display the text in brackets in Serial Monitor screen*/
  } 
  
}                                                                   /*Main loop closing bracket*/


/***************************************************************************************************************************************************************/
/*                                                                                                                                                             */
/*                                Function name  :  loopDelay                                                                                                  */
/*                                                                                                                                                             */
/*                             Input parameters  :  unsigned long length is time value in miliseconds                                                          */
/*                                                                                                                                                             */
/*                                      Returns  :  This function does not return any value                                                                    */
/*                                                                                                                                                             */
/*                          Purpose of function  :  This function is a custom delay, to prevent program being blocked by default delay function                */
/*                                                                                                                                                             */
/***************************************************************************************************************************************************************/        

void loopDelay(char length){                        /*assigning the value to local variable length*/
  char l;                                           /*variable*/
  for (l = 0; l < length; l++){                     /*do nothing, is used as an effective way for delay*/
    }
}

/***************************************************************************************************************************************************************/
/*                                                                                                                                                             */
/*                                Function name  :  controlFan                                                                                                 */
/*                                                                                                                                                             */
/*                             Input parameters  :  int airTemp is temperature value received from temp sensor                                                 */
/*                                                                                                                                                             */
/*                                      Returns  :  This function does not return any value                                                                    */
/*                                                                                                                                                             */
/*                          Purpose of function  :  This function controols coolinf fan speed, accordingly to received temperature value,                      */
/*                                                  and updates the stateFan integer to update smart device via Bluetooth                                      */
/*                                                                                                                                                             */
/***************************************************************************************************************************************************************/     

void controlFan(int airTemp){                      /*assigning the value to local variable airTemp*/
  if(airTemp <15){                                 /*Check the air temperature, if below 15 degree Celsius, then ......*/                
    digitalWrite(fan, 0);                          /*turn the power OFF the Cooling Fan*/
    loopDelay(shortDelay);                         /*Short custom delay*/
    fanState = 0;                                  /*update the state of the Cpooling Fan*/ 
  }
    
  else if(airTemp <= 17){                          /*Check the air temperature, if below or equal to 17 degree Celsius, then ......*/ 
    digitalWrite(fan, 1);                          /*turn ON the power to the Cooling Fan*/
    analogWrite(fan_PWM, 51);                      /*set the pulse width modulation value to Fan's PWM pin*/
    fanState = 0;                                  /*update the state of the Cpooling Fan*/ 
    loopDelay(shortDelay);                         /*Short custom delay*/
  }
    
  else if(airTemp <= 19){                          /*Check the air temperature, if below or equal to 19 degree Celsius, then ......*/ 
    digitalWrite(fan, 1);                          /*turn ON the power to the Cooling Fan*/
    analogWrite(fan_PWM, 102);                     /*set the pulse width modulation value to Fan's PWM pin*/
    fanState = 0;                                  /*update the state of the Cpooling Fan*/ 
    loopDelay(shortDelay);                         /*Short custom delay*/
  }
    
  else if(airTemp <= 21){                          /*Check the air temperature, if below or equal to 21 degree Celsius, then ......*/ 
    digitalWrite(fan, 1);                          /*turn ON the power to the Cooling Fan*/
    analogWrite(fan_PWM, 153);                     /*set the pulse width modulation value to Fan's PWM pin*/
    fanState = 0;                                  /*update the state of the Cpooling Fan*/ 
    loopDelay(shortDelay);                         /*Short custom delay*/
  }
    
  else if(airTemp <= 23){                          /*Check the air temperature, if below or equal to 23 degree Celsius, then ......*/ 
    digitalWrite(fan, 1);                          /*turn ON the power to the Cooling Fan*/
    analogWrite(fan_PWM, 204);                     /*set the pulse width modulation value to Fan's PWM pin*/
    fanState = 0;                                  /*update the state of the Cpooling Fan*/ 
    loopDelay(shortDelay);                         /*Short custom delay*/
  }
    
  else if(airTemp > 23){                           /*Check the air temperature, if above 23 degree Celsius, then ......*/ 
    digitalWrite(fan, 1);                          /*turn ON the power to the Cooling Fan*/
    analogWrite(fan_PWM, 255);                     /*set the pulse width modulation value to Fan's PWM pin*/ 
    fanState = 1;                                  /*update the state of the Cpooling Fan*/
    loopDelay(shortDelay);                         /*Short custom delay*/
  } 
}
/****************************************************************************************************************************************/
/*                                                                                                                                      */
/*         Function name  :  updateLedArray                                                                                             */
/*                                                                                                                                      */
/*      Input parameters  :  integer waterLevel is constrained value from 0 to 100 in percent units                                     */
/*                                                                                                                                      */
/*               Returns  :  This function does not return any value                                                                    */
/*                                                                                                                                      */
/*   Purpose of function  :  This function tests the waterlevel value and outputs the barGraph array to 2 shift registers, lighting     */
/*                           the LED bar array of 20 accordingly to actual state of water level                                         */
/*                                                                                                                                      */
/****************************************************************************************************************************************/     

void updateLedArray(int waterLevel){                 /*assigning the received value to local variable waterLevel*/
  switch(waterLevel){                                /*check the value stored in local variable*/
    case 0 ... 5:                                    /*If water remining in water tank is between 0% and 5%, then do ...*/ 
      digitalWrite(LATCH, LOW);                      /*Unlatch the previous Shift register*/ 
      shiftOut(DATA, CLK, MSBFIRST,barGraph[0]<<8);  /*.....Load the new data from barGraph array.......... */
      shiftOut(DATA, CLK, MSBFIRST,barGraph[0]<<8);  /*.....to shift register.............................. */
      shiftOut(DATA, CLK, MSBFIRST,barGraph[0]);     /*.....lighting 1 LED out of 20 LEDs on LED bar array..*/
      digitalWrite(LATCH, HIGH);                     /*Latch new data stored in shift register*/
      loopDelay(shortDelay);                         /*Short custom delay*/
      break;
    case 6 ... 10:                                   /*If water remining in water tank is between 6% and 10%, then do ...*/  
      digitalWrite(LATCH, LOW);                      /*Unlatch the previous Shift register*/
      shiftOut(DATA, CLK, MSBFIRST,barGraph[1]<<8);  /*.....Load the new data from barGraph array........... */
      shiftOut(DATA, CLK, MSBFIRST,barGraph[1]<<8);  /*.....to shift register............................... */
      shiftOut(DATA, CLK, MSBFIRST,barGraph[1]);     /*.....lighting 2 LEDs out of 20 LEDs on LED bar array..*/
      digitalWrite(LATCH, HIGH);                     /*Latch new data stored in shift register*/
      loopDelay(shortDelay);                         /*Short custom delay*/
      break;
    case 11 ... 15:                                  /*If water remining in water tank is between 11% and 15%, then do ...*/ 
      digitalWrite(LATCH, LOW);                      /*Unlatch the previous Shift register*/
      shiftOut(DATA, CLK, MSBFIRST,barGraph[2]<<8);  /*.....Load the new data from barGraph array........... */
      shiftOut(DATA, CLK, MSBFIRST,barGraph[2]<<8);  /*.....to shift register............................... */
      shiftOut(DATA, CLK, MSBFIRST,barGraph[2]);     /*.....lighting 3 LEDs out of 20 LEDs on LED bar array..*/
      digitalWrite(LATCH, HIGH);                     /*Latch new data stored in shift register*/
      loopDelay(shortDelay);                         /*Short custom delay*/
      break; 
    case 16 ... 20:                                  /*If water remining in water tank is between 16% and 20%, then do ...*/ 
      digitalWrite(LATCH, LOW);                      /*Unlatch the previous Shift register*/
      shiftOut(DATA, CLK, MSBFIRST,barGraph[3]<<8);  /*.....Load the new data from barGraph array........... */
      shiftOut(DATA, CLK, MSBFIRST,barGraph[3]<<8);  /*.....to shift register............................... */
      shiftOut(DATA, CLK, MSBFIRST,barGraph[3]);     /*.....lighting 4 LEDs out of 20 LEDs on LED bar array..*/
      digitalWrite(LATCH, HIGH);                     /*Latch new data stored in shift register*/
      loopDelay(shortDelay);                         /*Short custom delay*/
      break;      
    case 21 ... 25:                                  /*If water remining in water tank is between 21% and 25%, then do ...*/ 
      digitalWrite(LATCH, LOW);                      /*Unlatch the previous Shift register*/
      shiftOut(DATA, CLK, MSBFIRST,barGraph[4]<<8);  /*.....Load the new data from barGraph array........... */
      shiftOut(DATA, CLK, MSBFIRST,barGraph[4]<<8);  /*.....to shift register............................... */
      shiftOut(DATA, CLK, MSBFIRST,barGraph[4]);     /*.....lighting 5 LEDs out of 20 LEDs on LED bar array..*/
      digitalWrite(LATCH, HIGH);                     /*Latch new data stored in shift register*/              
      loopDelay(shortDelay);                         /*Short custom delay*/
      break; 
    case 26 ... 30:                                  /*If water remining in water tank is between 26% and 30%, then do ...*/ 
      digitalWrite(LATCH, LOW);                      /*Unlatch the previous Shift register*/
      shiftOut(DATA, CLK, MSBFIRST,barGraph[5]<<8);  /*.....Load the new data from barGraph array........... */
      shiftOut(DATA, CLK, MSBFIRST,barGraph[5]<<8);  /*.....to shift register............................... */
      shiftOut(DATA, CLK, MSBFIRST,barGraph[5]);     /*.....lighting 6 LEDs out of 20 LEDs on LED bar array..*/
      digitalWrite(LATCH, HIGH);                     /*Latch new data stored in shift register*/
      loopDelay(shortDelay);                         /*Short custom delay*/
      break; 
    case 31 ... 35:                                  /*If water remining in water tank is between 31% and 35%, then do ...*/          
      digitalWrite(LATCH, LOW);                      /*Unlatch the previous Shift register*/
      shiftOut(DATA, CLK, MSBFIRST,barGraph[6]<<8);  /*.....Load the new data from barGraph array........... */
      shiftOut(DATA, CLK, MSBFIRST,barGraph[6]<<8);  /*.....to shift register............................... */
      shiftOut(DATA, CLK, MSBFIRST,barGraph[6]);     /*.....lighting 7 LEDs out of 20 LEDs on LED bar array..*/
      digitalWrite(LATCH, HIGH);                     /*Latch new data stored in shift register*/
      loopDelay(shortDelay);                         /*Short custom delay*/
      break;  
    case 36 ... 40:                                  /*If water remining in water tank is between 36% and 40%, then do ...*/ 
      digitalWrite(LATCH, LOW);                      /*Unlatch the previous Shift register*/
      shiftOut(DATA, CLK, MSBFIRST,barGraph[7]<<8);  /*.....Load the new data from barGraph array........... */
      shiftOut(DATA, CLK, MSBFIRST,barGraph[7]<<8);  /*.....to shift register............................... */
      shiftOut(DATA, CLK, MSBFIRST,barGraph[7]);     /*.....lighting 8 LEDs out of 20 LEDs on LED bar array..*/
      digitalWrite(LATCH, HIGH);                     /*Latch new data stored in shift register*/
      loopDelay(shortDelay);                         /*Short custom delay*/
      break; 
    case 41 ... 45:                                  /*If water remining in water tank is between 41% and 45%, then do ...*/ 
      digitalWrite(LATCH, LOW);                      /*Unlatch the previous Shift register*/
      shiftOut(DATA, CLK, MSBFIRST,barGraph[0]<<8);  /*.....Load the new data from barGraph array........... */
      shiftOut(DATA, CLK, MSBFIRST,barGraph[0]);     /*.....to shift register............................... */  
      shiftOut(DATA, CLK, MSBFIRST,barGraph[7]);     /*.....lighting 9 LEDs out of 20 LEDs on LED bar array..*/
      digitalWrite(LATCH, HIGH);                     /*Latch new data stored in shift register*/
      loopDelay(shortDelay);                         /*Short custom delay*/
      break;
    case 46 ... 50:                                  /*If water remining in water tank is between 46% and 50%, then do ...*/ 
      digitalWrite(LATCH, LOW);                      /*Unlatch the previous Shift register*/
      shiftOut(DATA, CLK, MSBFIRST,barGraph[1]<<8);  /*.....Load the new data from barGraph array............ */
      shiftOut(DATA, CLK, MSBFIRST,barGraph[1]);     /*.....to shift register................................ */
      shiftOut(DATA, CLK, MSBFIRST,barGraph[7]);     /*.....lighting 10 LEDs out of 20 LEDs on LED bar array..*/
      digitalWrite(LATCH, HIGH);                     /*Latch new data stored in shift register*/
      loopDelay(shortDelay);                         /*Short custom delay*/
      break;
    case 51 ... 55:                                  /*If water remining in water tank is between 51% and 55%, then do ...*/  
      digitalWrite(LATCH, LOW);                      /*Unlatch the previous Shift register*/
      shiftOut(DATA, CLK, MSBFIRST,barGraph[2]<<8);  /*.....Load the new data from barGraph array............ */
      shiftOut(DATA, CLK, MSBFIRST,barGraph[2]);     /*.....to shift register................................ */
      shiftOut(DATA, CLK, MSBFIRST,barGraph[7]);     /*.....lighting 11 LEDs out of 20 LEDs on LED bar array..*/
      digitalWrite(LATCH, HIGH);                     /*Latch new data stored in shift register*/
      loopDelay(shortDelay);                         /*Short custom delay*/
      break;     
    case 56 ... 60:                                  /*If water remining in water tank is between 56% and 60%, then do ...*/ 
      digitalWrite(LATCH, LOW);                      /*Unlatch the previous Shift register*/
      shiftOut(DATA, CLK, MSBFIRST,barGraph[3]<<8);  /*.....Load the new data from barGraph array............ */
      shiftOut(DATA, CLK, MSBFIRST,barGraph[3]);     /*.....to shift register................................ */
      shiftOut(DATA, CLK, MSBFIRST,barGraph[7]);     /*.....lighting 12 LEDs out of 20 LEDs on LED bar array..*/
      digitalWrite(LATCH, HIGH);                     /*Latch new data stored in shift register*/
      loopDelay(shortDelay);                         /*Short custom delay*/
      break;
    case 61 ... 65:                                  /*If water remining in water tank is between 61% and 65%, then do ...*/ 
      digitalWrite(LATCH, LOW);                      /*Unlatch the previous Shift register*/
      shiftOut(DATA, CLK, MSBFIRST,barGraph[4]<<8);  /*.....Load the new data from barGraph array............ */
      shiftOut(DATA, CLK, MSBFIRST,barGraph[4]);     /*.....to shift register................................ */
      shiftOut(DATA, CLK, MSBFIRST,barGraph[7]);     /*.....lighting 13 LEDs out of 20 LEDs on LED bar array..*/
      digitalWrite(LATCH, HIGH);                     /*Latch new data stored in shift register*/
      loopDelay(shortDelay);                         /*Short custom delay*/
      break; 
    case 66 ... 70:                                  /*If water remining in water tank is between 66% and 70%, then do ...*/ 
      digitalWrite(LATCH, LOW);                      /*Unlatch the previous Shift register*/
      shiftOut(DATA, CLK, MSBFIRST,barGraph[5]<<8);  /*.....Load the new data from barGraph array............ */
      shiftOut(DATA, CLK, MSBFIRST,barGraph[5]);     /*.....to shift register................................ */
      shiftOut(DATA, CLK, MSBFIRST,barGraph[7]);     /*.....lighting 14 LEDs out of 20 LEDs on LED bar array..*/
      digitalWrite(LATCH, HIGH);                     /*Latch new data stored in shift register*/
      loopDelay(shortDelay);                         /*Short custom delay*/
      break; 
    case 71 ... 75:                                  /*If water remining in water tank is between 71% and 75%, then do ...*/ 
      digitalWrite(LATCH, LOW);                      /*Unlatch the previous Shift register*/
      shiftOut(DATA, CLK, MSBFIRST,barGraph[6]<<8);  /*.....Load the new data from barGraph array............ */
      shiftOut(DATA, CLK, MSBFIRST,barGraph[6]);     /*.....to shift register................................ */
      shiftOut(DATA, CLK, MSBFIRST,barGraph[7]);     /*.....lighting 15 LEDs out of 20 LEDs on LED bar array..*/
      digitalWrite(LATCH, HIGH);                     /*Latch new data stored in shift register*/
      loopDelay(shortDelay);                         /*Short custom delay*/
      break; 
    case 76 ... 80:                                  /*If water remining in water tank is between 76% and 80%, then do ...*/ 
      digitalWrite(LATCH, LOW);                      /*Unlatch the previous Shift register*/
      shiftOut(DATA, CLK, MSBFIRST,barGraph[7]<<8);  /*.....Load the new data from barGraph array............ */
      shiftOut(DATA, CLK, MSBFIRST,barGraph[7]);     /*.....to shift register................................ */
      shiftOut(DATA, CLK, MSBFIRST,barGraph[7]);     /*.....lighting 16 LEDs out of 20 LEDs on LED bar array..*/
      digitalWrite(LATCH, HIGH);                     /*Latch new data stored in shift register*/
      loopDelay(shortDelay);                         /*Short custom delay*/
      break;   
    case 81 ... 85:                                  /*If water remining in water tank is between 81% and 85%, then do ...*/ 
      digitalWrite(LATCH, LOW);                      /*Unlatch the previous Shift register*/
      shiftOut(DATA, CLK, MSBFIRST,barGraph[0]);     /*.....Load the new data from barGraph array............ */
      shiftOut(DATA, CLK, MSBFIRST,barGraph[7]);     /*.....to shift register................................ */
      shiftOut(DATA, CLK, MSBFIRST,barGraph[7]);     /*.....lighting 17 LEDs out of 20 LEDs on LED bar array..*/
      digitalWrite(LATCH, HIGH);                     /*Latch new data stored in shift register*/
      loopDelay(shortDelay);                         /*Short custom delay*/
      break;
    case 86 ... 90:                                  /*If water remining in water tank is between 86% and 90%, then do ...*/ 
      digitalWrite(LATCH, LOW);                      /*Unlatch the previous Shift register*/
      shiftOut(DATA, CLK, MSBFIRST,barGraph[1]);     /*.....Load the new data from barGraph array............ */
      shiftOut(DATA, CLK, MSBFIRST,barGraph[7]);     /*.....to shift register................................ */
      shiftOut(DATA, CLK, MSBFIRST,barGraph[7]);     /*.....lighting 18 LEDs out of 20 LEDs on LED bar array..*/
      digitalWrite(LATCH, HIGH);                     /*Latch new data stored in shift register*/
      loopDelay(shortDelay);                         /*Short custom delay*/
      break; 
    case 91 ... 95:                                  /*If water remining in water tank is between 91% and 95%, then do ...*/ 
      digitalWrite(LATCH, LOW);                      /*Unlatch the previous Shift register*/
      shiftOut(DATA, CLK, MSBFIRST,barGraph[2]);     /*.....Load the new data from barGraph array............ */
      shiftOut(DATA, CLK, MSBFIRST,barGraph[7]);     /*.....to shift register................................ */
      shiftOut(DATA, CLK, MSBFIRST,barGraph[7]);     /*.....lighting 19 LEDs out of 20 LEDs on LED bar array..*/
      digitalWrite(LATCH, HIGH);                     /*Latch new data stored in shift register*/
      loopDelay(shortDelay);                         /*Short custom delay*/
      break;   
    case 96 ... 100:                                 /*If water remining in water tank is between 96% and 100%, then do ...*/ 
      digitalWrite(LATCH, LOW);                      /*Unlatch the previous Shift register*/
      shiftOut(DATA, CLK, MSBFIRST,barGraph[3]);     /*.....Load the new data from barGraph array............ */
      shiftOut(DATA, CLK, MSBFIRST,barGraph[7]);     /*.....to shift register................................ */
      shiftOut(DATA, CLK, MSBFIRST,barGraph[7]);     /*.....lighting 20 LEDs out of 20 LEDs on LED bar array..*/
      digitalWrite(LATCH, HIGH);                     /*Latch new data stored in shift register*/
      loopDelay(shortDelay);                         /*Short custom delay*/
      break;       
   }
}
