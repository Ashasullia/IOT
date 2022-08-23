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
