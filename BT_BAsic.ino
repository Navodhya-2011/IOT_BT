#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

#define echoPin_1 17 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin_1 16 //attach pin D3 Arduino to pin Trig of HC-SR04

// defines variables
long duration_1; // variable for the duration of sound wave travel
int distance_1; // variable for the distance measurement

//Initialize Digital Pins
const int leftForward = 14;
const int leftBackward = 27;
const int rightForward = 26;
const int rightBackward = 25;

char data = 0;
char junk;                // use for bluetooth communication
String inputString = "";  // assigning bluetooth character string

//

#include <Wire.h>
#include <Adafruit_BMP085.h>
Adafruit_BMP085 bmp;
//

#include "DHT.h"
#define DHTPIN 33     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
//


int rain_Sensor = 4;
int rain_State = 1; // 1 = dry, 0 = rain

int red_LED = 32;
int blue_LED = 12;
int orange_LED = 13;
int green_LED = 23;
void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");

  pinMode(red_LED, OUTPUT);
  pinMode(blue_LED, OUTPUT);
  pinMode(orange_LED, OUTPUT);
  pinMode(green_LED, OUTPUT);

  // set control pins as Output
  pinMode(leftForward, OUTPUT);
  pinMode(leftBackward, OUTPUT);
  pinMode(rightForward, OUTPUT);
  pinMode(rightBackward, OUTPUT);
  pinMode(rain_Sensor, INPUT);

  digitalWrite(leftForward, LOW);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightForward, LOW);
  digitalWrite(rightBackward, LOW);

  pinMode(trigPin_1, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin_1, INPUT); // Sets the echoPin as an INPUT

  //
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085/BMP180 sensor, check wiring!");
    while (1) {}
  }

  dht.begin();
}

void loop() {

  if (SerialBT.available()) {
    char inChar = SerialBT.read(); //read the input
    inputString = inChar;              //assigning recived bluetooth character to varable called inputString
    Serial.println(inputString);
  }


  //  while (Serial.available() > 0)
  //  {
  //    junk = Serial.read() ;  // clear the serial buffer
  //  }


  if (inputString == "1") {
    // run forward
    digitalWrite(leftForward, HIGH);
    digitalWrite(leftBackward, LOW);
    digitalWrite(rightForward, HIGH);
    digitalWrite(rightBackward, LOW);

    read_Ultra_1();
    if (distance_1 < 10) {
      digitalWrite(leftForward, LOW);
      digitalWrite(leftBackward, LOW);
      digitalWrite(rightForward, LOW);
      digitalWrite(rightBackward, LOW);

      delay(4000);
    }
  }

  if (inputString == "2") {
    // run right
    digitalWrite(leftForward, LOW);
    digitalWrite(leftBackward, LOW);
    digitalWrite(rightForward, LOW);
    digitalWrite(rightBackward, LOW);
  }

  if (inputString == "3") {
    // run forward
    digitalWrite(leftForward, LOW);
    digitalWrite(leftBackward, HIGH);
    digitalWrite(rightForward, HIGH);
    digitalWrite(rightBackward, LOW);
  }

  if (inputString == "4") {
    // run left
    digitalWrite(leftForward, HIGH);
    digitalWrite(leftBackward, LOW);
    digitalWrite(rightForward, LOW);
    digitalWrite(rightBackward, HIGH);
  }
  ////
  read_BMP();
  read_DHT11();

  read_Rain();



  delay(20);
}

void read_Rain() {
  rain_State = digitalRead(rain_Sensor);
  Serial.print("Rain State = ");
  Serial.println(rain_State);
  if (rain_State == LOW) {
    digitalWrite(green_LED, HIGH);
  }
  else {
    digitalWrite(green_LED, LOW);
  }
}


void read_Ultra_1() {
  // Clears the trigPin condition
  digitalWrite(trigPin_1, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin_1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin_1, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration_1 = pulseIn(echoPin_1, HIGH);
  // Calculating the distance
  distance_1 = duration_1 * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance_1);
  Serial.println(" cm");
  delay(100);
}

void read_BMP() {
  Serial.print("Temperature = ");
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");

  if (bmp.readPressure() > 100600) {
    digitalWrite(orange_LED, HIGH);
  }
  else {
    digitalWrite(orange_LED, LOW);
  }

  // Calculate altitude assuming 'standard' barometric
  // pressure of 1013.25 millibar = 101325 Pascal
  Serial.print("Altitude = ");
  Serial.print(bmp.readAltitude());
  Serial.println(" meters");

  Serial.print("Pressure at sealevel (calculated) = ");
  Serial.print(bmp.readSealevelPressure());
  Serial.println(" Pa");

  // you can get a more precise measurement of altitude
  // if you know the current sea level pressure which will
  // vary with weather and such. If it is 1015 millibars
  // that is equal to 101500 Pascals.
  Serial.print("Real altitude = ");
  Serial.print(bmp.readAltitude(102000));
  Serial.println(" meters");

  Serial.println();
  // delay(500);
}

void read_DHT11() {
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);

  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
  Serial.print(F("°C "));
  Serial.print(f);
  Serial.print(F("°F  Heat index: "));
  Serial.print(hic);
  Serial.print(F("°C "));
  Serial.print(hif);
  Serial.println(F("°F"));

}
