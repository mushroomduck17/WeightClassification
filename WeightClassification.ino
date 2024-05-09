#include "HX711.h"
#include <HX711.h>
#include <math.h>
#include <Servo.h>
#include <DHT11.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SimpleKalmanFilter.h>

#define DOUT                     4
#define CLK                      5 
#define weight_point_min         20
#define weight_point_1           50 
#define weight_point_2           100
#define maxNumberPerRange        100
#define resume_pin               6
#define temp_error_per10C        3
#define standard_temperature     27

DHT11 dht11(2);
HX711 scale;
Servo Servo1;
Servo Servo2;
LiquidCrystal_I2C lcd(0x27,20,4);
SimpleKalmanFilter kalmanFilter(2, 2, 0.01);

int range1_count=0;
int range2_count=0;
int range3_count=0;
int flag=1;

void counting_display()
{
  lcd.setCursor(0,0);
  lcd.print("Bin1:");
  lcd.setCursor(6,0);
  lcd.print(range1_count);
  lcd.setCursor(8,0);
  lcd.print("Bin2:");
  lcd.setCursor(14,0);
  lcd.print(range2_count);
  lcd.setCursor(0,1);
  lcd.print("Bin3:");
  lcd.setCursor(6,1);
  lcd.print(range1_count);
}

void setup() {
  Servo1.attach(7);
  Servo1.write(90);
  Servo2.attach(8);
  Servo2.write(0);
  scale.begin(DOUT, CLK);
  Serial.begin(9600);
  scale.set_scale();
  scale.tare();
  pinMode(resume_pin,INPUT);
  lcd.init();
  
}

void loop() {
  if (flag==0)
  {
    lcd.setCursor(0,0);
    lcd.print("The bin is full");
    lcd.setCursor(0,1);
    lcd.print("Press the button to continue");
    if (digitalRead(resume_pin)==1)
    {
      flag=1;
      if ( range1_count ==100)
      {
        range1_count=0;
      }
      if ( range2_count ==100)
      {
        range2_count=0;
      }
      if ( range3_count ==100)
      {
        range3_count=0;
      }
    }
  }

  if (flag==1)
  {
  int temperature = dht11.readTemperature();
  int temperature_error= temp_error_per10C *(temperature -standard_temperature)/10  ;


  
  if (scale.is_ready()) {
      double raw_value;
      double filtered_value;
      for (int i=0;i<50;i++)
      {
        raw_value  = -(scale.read()-553800)/430.32;
        filtered_value = kalmanFilter.updateEstimate(raw_value);
        delay(20);
      }
      double final_value= filtered_value - temperature_error;
      
      Serial.print("Gia tri do duoc: ");
      Serial.println(filtered_value,4);
      if (final_value >= weight_point_min && final_value < weight_point_1  ) 
      {
        range1_count++;
        if (range1_count==maxNumberPerRange)
        {
          flag=0;
        }
        Servo2.write(60);
        Servo1.write(50);
        delay(1000);
        Servo2.write(0);        
      }
      if (final_value >= weight_point_1 && filtered_value < weight_point_2)
      {
         range2_count++;
         if (range2_count==maxNumberPerRange)
         {
           flag=0;
         }
        Servo2.write(60);
        Servo1.write(130);
        delay(1000);
        Servo2.write(0);
             
      }
      if (final_value >=  weight_point_2)
      {
        range3_count++;
        if (range3_count==maxNumberPerRange)
        {
          flag=0;
        }
        Servo2.write(60);
        Servo1.write(90);
        delay(1000);
        Servo2.write(0);
      }
      counting_display();
  }
  else {
    lcd.setCursor(0,0);
    lcd.print("HX711 ERROR");
  }
  delay(500);
  }
}
