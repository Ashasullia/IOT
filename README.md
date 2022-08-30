PROGRAMS OF IOT<br>
1.Traffic LED:<br>
https://wokwi.com/projects/333716331435131476<br>
2.RGB LED:<br>
https://wokwi.com/projects/333805328840786515<br>
3.Hello world:<br>
https://wokwi.com/projects/322062421191557714<br>
4.servometer:<br>
https://wokwi.com/projects/334978042856211028<br>
5.servomotor using potential slide :<br>
https://wokwi.com/projects/334978042856211028<br>
6.buzzer with resistor only :<br>
https://wokwi.com/projects/335065104791896658<br>
7.using buzzer and pushbutton :<br>
https://wokwi.com/projects/335067745393574483<br>
8.ultrasonic sensor :<br>
https://wokwi.com/projects/335070015851070035<br>
9.ultrasonic sensor with buzzer :<br>
https://wokwi.com/projects/335073357821117012<br>
10.ultrasonic sensor with buzzer and LED :<br>
https://wokwi.com/projects/335610028297814611<br>
11.2 ultrasonic with one buzzer and LED :<br>
https://wokwi.com/projects/335703435993154131<br>
12.Potentiometer with LED :<br>
https://wokwi.com/projects/335701299799523922<br>
13.srevometer with buzzer :<br>
https://wokwi.com/projects/335704534482420307<br>  


https://wokwi.com/projects/340775764469219922 - LED_CHASER<br>
https://wokwi.com/projects/340776572602548818 - LDR<br>
https://wokwi.com/projects/340776926585029204 - LDR_LED<br>


***********************************************************<br>
//<br.>
   ESP32
   https://wokwi.com/projects/336966830711112275 - LED
   https://wokwi.com/projects/336967978479256147 - 3 LED
   https://wokwi.com/projects/340880463446934098 - RGB
   https://wokwi.com/projects/340882358612787796 - LCD
   https://wokwi.com/projects/340886369600537172 - Servo Motor + Pushbutton
   https://wokwi.com/projects/340888468071645780 - Servo Motor + Sliding Potentiometer
   https://wokwi.com/projects/340890155914101331 - Buzzer + Pushbutton_
   https://wokwi.com/projects/340890489300451922 - Buzzer + UltraSonic Sensor
   https://wokwi.com/projects/340890896679567955 - Potentiometer + LED
   https://wokwi.com/projects/340892440485429842 - DHT22
   https://wokwi.com/projects/340893919446303316 - LED CHASER
   https://wokwi.com/projects/340936317213868626 - LDR
   https://wokwi.com/projects/340936847717827156 - LDR + LED

//HARDWARE

      ULTRASONIC_SENSOR

      const int trigPin = 13; //D7
      const int echoPin = 12; //D6

      long duration;
      float distanceCm;
      float distanceInch;

      void setup() {
              Serial.begin(9600); // Starting Serial Terminal
      }

      void loop() {
         long duration, inches, cm;
         pinMode(trigPin, OUTPUT);
         digitalWrite(trigPin, LOW);
         delayMicroseconds(2);
         digitalWrite(trigPin, HIGH);
         delayMicroseconds(10);
         digitalWrite(trigPin, LOW);
         pinMode(echoPin, INPUT);
         duration = pulseIn(echoPin, HIGH);
         inches = microsecondsToInches(duration);
         cm = microsecondsToCentimeters(duration);
         Serial.print(inches);
         Serial.print("in, ");
         Serial.print(cm);
         Serial.print("cm");
         Serial.println();
         delay(1000);
      }

      long microsecondsToInches(long microseconds) {
          return microseconds / 74 / 2;
      }

     long microsecondsToCentimeters(long microseconds) {
       return microseconds / 29 / 2;
      }


//<br.>
DHT11

       #include <Adafruit_Sensor.h>
       #include <DHT.h>;
       #define DHTPIN 13     // what pin we're connected to
       #define DHTTYPE DHT11   // DHT 22  (AM2302)
       DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino
       //Variables
       int chk;
       float hum;  //Stores humidity value
       float temp; //Stores temperature value
       void setup()
       {
         Serial.begin(9600);
         dht.begin();
       }
       void loop()
      {
          delay(2000);
          //Read data and store it to variables hum and temp
          hum = dht.readHumidity();
          temp= dht.readTemperature();
          //Print temp and humidity values to serial monitor
          Serial.print("Humidity: ");
          Serial.print(hum);
          Serial.print(" %, Temp: ");
          Serial.print(temp);
          Serial.println(" Celsius");
          delay(1000); //Delay 2 sec.
      }
 
 
 RGB
 
 int red = D1;
 int green = D6;
 int blue = D7;
 //GROUND IS CONNECTED TO 3V 
 void setup() {
   pinMode(red, OUTPUT);
   pinMode(green, OUTPUT);
   pinMode(blue, OUTPUT);

 }

 void loop() {
   displayColor(0b100); //RED
   delay(1000);
   displayColor(0b010); //GREEN
   delay(1000);
   displayColor(0b001); //BLUE
   delay(1000);
   displayColor(0b101); //MAGENTA
   delay(1000);
   displayColor(0b011); //CYAN
   delay(1000);
   displayColor(0b110); //YELLOW
   delay(1000);
   displayColor(0b111); //WHITE
   delay(1000);
    }

      void displayColor(byte color) {
      digitalWrite(red, !bitRead(color, 2));
      digitalWrite(green, !bitRead(color, 1));
      digitalWrite(blue, !bitRead(color, 0));
    }


IR_LED
     int ir=D7;
    int led=D5;
    void setup() {
      // put your setup code here, to run once:
      pinMode(ir,INPUT);
        pinMode(led,OUTPUT);
        Serial.begin(9600);

    }

    void loop() {
      // put your main code here, to run repeatedly:
      int irvalue=digitalRead(ir);
      if(irvalue==LOW)
      {
        Serial.println("LOW");
        digitalWrite(led,HIGH);
      }
      else
      {
        Serial.println("HIGH");
        digitalWrite(led,LOW);
      }
    delay(100);
    }
</br>
</br>
</br>
     LDR
     const int ldrPin=A0;
     void setup() {
       Serial.begin(9600);
       pinMode(ldrPin,INPUT);
     }
     void loop() {
       int rawData = analogRead(ldrPin);   
       Serial.println(rawData);
       delay(1000);
     }
</br>
</br>
</br>

 LDR_LED

 int ldr=A0;//Set A0(Analog Input) for LDR.
 int value=0;
 int led=D1;
 void setup() {
 Serial.begin(9600);
 pinMode(led,OUTPUT);
 }

 void loop() {
 value=analogRead(ldr);//Reads the Value of LDR(light).
 Serial.println("LDR value is :");//Prints the value of LDR to Serial Monitor.
 Serial.println(value);
 if(value<50)
   {
     digitalWrite(led,HIGH);//Makes the LED glow in Dark.
   }
   else
   {
     digitalWrite(led,LOW);//Turns the LED OFF in Light.
   }
   delay(1000);
 }\
 </br>
 </br>
 LED_CHASER
 
int pinsCount=6;                        // declaring the integer variable pinsCount
int pins[] = {D0,D1,D7,D5,D3,D2};          // declaring the array pins[]

void setup() {                
  for (int i=0; i<pinsCount; i=i+1){    // counting the variable i from 0 to 9
    pinMode(pins[i], OUTPUT);            // initialising the pin at index i of the array of pins as OUTPUT
  }
}

void loop() {
  for (int i=0; i<pinsCount; i=i+1){    // chasing right
    digitalWrite(pins[i], HIGH);         // switching the LED at index i on
    delay(100);                          // stopping the program for 100 milliseconds
    digitalWrite(pins[i], LOW);          // switching the LED at index i off
  }
  for (int i=pinsCount-1; i>0; i=i-1){   // chasing left (except the outer leds)
   digitalWrite(pins[i], HIGH);         // switching the LED at index i on
    delay(100);                          // stopping the program for 100 milliseconds
    digitalWrite(pins[i], LOW);          // switching the LED at index i off

  }
}
1. using LCD with DHT22 <br>
https://wokwi.com/projects/337604420660363858<br>
//HARDWARE<br>

   ULTRASONIC_SENSOR<br>

   const int trigPin = 13; //D7<br>
   const int echoPin = 12; //D6<br>

   long duration;<br>
   float distanceCm;<br>
   float distanceInch;<br>

   void setup() {<br>
           Serial.begin(9600); // Starting Serial Terminal<br>
   }<br>

   void loop() {<br>
      long duration, inches, cm;<br>
      pinMode(trigPin, OUTPUT);<br>
      digitalWrite(trigPin, LOW);<br>
      delayMicroseconds(2);<br>
      digitalWrite(trigPin, HIGH);<br>
      delayMicroseconds(10);<br>
      digitalWrite(trigPin, LOW);<br>
      pinMode(echoPin, INPUT);<br>
      duration = pulseIn(echoPin, HIGH);<br>
      inches = microsecondsToInches(duration);<br>
      cm = microsecondsToCentimeters(duration);<br>
      Serial.print(inches);<br>
      Serial.print("in, ");<br>
      Serial.print(cm);<br>
      Serial.print("cm");<br>
      Serial.println();<br>
      delay(1000);<br>
   }<br>

   long microsecondsToInches(long microseconds) {<br>
       return microseconds / 74 / 2;<br>
   }<br>

  long microsecondsToCentimeters(long microseconds) {<br>
    return microseconds / 29 / 2;<br>
   }<br>



DHT11<br>

    #include <Adafruit_Sensor.h><br>
    #include <DHT.h>;<br>
    #define DHTPIN 13     // what pin we're connected to<br>
    #define DHTTYPE DHT11   // DHT 22  (AM2302)<br>
    DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino<br>
    //Variables<br>
    int chk;<br>
    float hum;  //Stores humidity value<br>
    float temp; //Stores temperature value<br>
    void setup()<br>
    {<br>
      Serial.begin(9600);<br>
      dht.begin();<br>
    }<br>
    void loop()<br>
   {<br>
       delay(2000);<br>
       //Read data and store it to variables hum and temp<br>
       hum = dht.readHumidity();<br>
       temp= dht.readTemperature();<br>
       //Print temp and humidity values to serial monitor<br>
       Serial.print("Humidity: ");<br>
       Serial.print(hum);<br>
       Serial.print(" %, Temp: ");<br>
       Serial.print(temp);<br>
       Serial.println(" Celsius");<br>
       delay(1000); //Delay 2 sec.<br>
   }<br>
 
 
 RGB<br>
 
 int red = D1;<br>
 int green = D6;<br>
 int blue = D7;<br>
 //GROUND IS CONNECTED TO 3V <br>
 void setup() {<br>
   pinMode(red, OUTPUT);<br>
   pinMode(green, OUTPUT);<br>
   pinMode(blue, OUTPUT);<br>

 }<br>

 void loop() {<br>
   displayColor(0b100); //RED<br>
   delay(1000);<br>
   displayColor(0b010); //GREEN<br>
   delay(1000);<br>
   displayColor(0b001); //BLUE<br>
   delay(1000);<br>
   displayColor(0b101); //MAGENTA<br>
   delay(1000);<br>
   displayColor(0b011); //CYAN<br>
   delay(1000);<br>
   displayColor(0b110); //YELLOW<br>
   delay(1000);<br>
   displayColor(0b111); //WHITE<br>
   delay(1000);<br>
 }<br>

 void displayColor(byte color) {<br>
   digitalWrite(red, !bitRead(color, 2));<br>
   digitalWrite(green, !bitRead(color, 1));<br>
   digitalWrite(blue, !bitRead(color, 0));<br>
 }<br>
