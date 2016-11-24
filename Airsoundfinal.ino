#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <avr/pgmspace.h>

File myFile;
const int SDpin = 10;
String data;
char ble_data[70];

//*****DUST SENSOR VARIABLES******///
int measurePin = 5;
int ledPower = 6;
int samplingTime = 280;
int deltaTime = 40;
int sleepTime = 9680;
float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;
//*****DUST SENSORS VARIABLES*****//

//*****AIR SENSORS VARIABLES*****//
int no2_pin = 0;
int o3_pin = 1;
float no2_concentration = 0;
float o3_concentration = 0;
float o3_cal_voltage = 0;
//******************************//

//**SOUND SENSORS VARIABLES**/////
int soundPin = 5;
int soundValue = LOW;
//*********************************///

//****GPS*****//

char frame[100];
char GNSSrunstatus[1];
char Fixstatus[1];
char UTCdatetime[18];
char yearcheck[4];
char latitude[10];
char logitude[11];
char altitude[8];


SoftwareSerial mySerial(2, 3); // RX, TX



//****DONE GPS****//

//****BLE VARIBLES*****////
#define BUFFER_LENGTH 100
SoftwareSerial ble(7,8);        // For Uno, HM10 TX pin to Arduino Uno pin D2, HM10 RX pin to Arduino Uno pin D3

char buffer[BUFFER_LENGTH]; // Buffer to store response
int timeout=800;            // Wait 800ms each time for BLE to response, depending on your application, adjust this value accordingly
const PROGMEM long bauds[] = {9600,57600,115200,38400,2400,4800,19200}; // common baud rates, when using HM-10 module with SoftwareSerial, try not to go over 57600
long baud;

//***********************//
//*****B L E functions ***/////
long BLEAutoBaud() {
  int baudcount=sizeof(bauds)/sizeof(long);
  for(int i=0; i<baudcount; i++) {
      baud = pgm_read_word_near(bauds + i);
      for(int x=0; x<3; x++)  {  // test at least 3 times for each baud
      //Serial.print("Testing baud ");
      //Serial.println();
      ble.begin(baud);
      if (BLEIsReady()) {
        return baud;
      }
      }
  }
  return -1;
}

boolean BLEIsReady() {
  BLECmd(timeout, "AT" ,buffer);    // Send AT and store response to buffer 
  if (strcmp(buffer,"OK")==0){    
    return true;
  } else {
    return false;
  }  
}

boolean BLECmd(long timeout, char* command, char* temp) {
  long endtime;
  boolean found=false;
  endtime=millis()+timeout;     // 
  memset(temp,0,100);         // clear buffer
  found=true;
  //Serial.print("Arduino send = ");
  //Serial.println(command);
  ble.print(command);
//  
//  // The loop below wait till either a response is received or timeout
//  // The problem with this BLE Shield is the HM-10 module does not response with CR LF at the end of the response,
//  // so a timeout is required to detect end of response and also prevent the loop locking up.
//
  while(!ble.available()){
    if(millis()>endtime) {      // timeout, break
      found=false;
      break;
    }
  }  

  if (found) {            // response is available
    int i=0;
    while(ble.available()) {    // loop and read the data
      char a= ble.read();
      // Serial.print((char)a); // Uncomment this to see raw data from BLE
      temp[i]=a;          // save data to buffer
      i++;
      if (i>=BUFFER_LENGTH) break;  // prevent buffer overflow, need to break
      delay(1);           // give it a 2ms delay before reading next character
    }
    Serial.print("BLE reply= ");
    Serial.println(temp);
    return true;
  } else {
    Serial.println("BLE timeout!");
    return false;
  }
}
//**** A L L S E T U P S *****/////

void setup() {
  // Open serial communications and wait for port to open:
  pinMode(soundPin, INPUT);
  pinMode (no2_pin, INPUT);
  pinMode (o3_pin, INPUT);
  Serial.begin(9600);
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
  // ******* SD CARD Setup ******///
  if (!SD.begin(SDpin)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initializing SD card done !");
  // ******* Done SD setup *****///
  
    //***** G P S Setup ******///
  // set the data rate for the SoftwareSerial port
  
  //***** Done G P S Setup ******///
 
  //**** B L E Setup ****//

  long baudrate = BLEAutoBaud();

  if (baudrate>0) {
    Serial.print("Found BLE baud rate ");
    Serial.println(baudrate);
  } else {
    Serial.println("No BLE detected.");
    while(1){};           // No BLE found, just going to stop here
  }

  // The following commands are just to demonstrate the shield is working properly,
  // in actual application, only call those that are needed by your application.
  // Check HM-10 datasheet for the description of the commands.
  //BLECmd(timeout,"AT+NAME?",buffer);
  //BLECmd(timeout,"AT+BAUD?",buffer);
  //BLECmd(timeout,"AT+POWE?",buffer);
  //BLECmd(timeout,"AT+NOTI1",buffer);
  Serial.println("Waiting for remote connection...");
  ble.end();
  //***** Done B L E Setup ******///
  
}

void loop() {
  File myFile = SD.open("data.csv", FILE_WRITE);
//  
//  // Port begin for GPS to capture data here
  //get_GPS(myFile);
//  
//  // End when done
  //getDust(myFile);
  //get_NO2(myFile);
  //get_O3(myFile);
  //get_sound(myFile);
  //myFile.close();
  delay(1000);

  // Send data
  sendData();
}

//************************** M a i n F u n c t i o n s **************************//
void sendData() {
  myFile = SD.open("data.csv");
  if (myFile) {
    Serial.println("Opening file");
    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      data = myFile.readStringUntil('\n');
      data.toCharArray(ble_data, 70);
      if (ble_data[0] != ',') {
        Serial.println(ble_data); //Printing for debugging purpose
        ble.write(ble_data);
      } else {
        Serial.println("Data not complete.");
      }
      delay(500);       
    }
    myFile.close();
  }
}
int8_t get_GPS(File myFile) {
  mySerial.begin(9600);
  Serial.println("My Serial is opened");
  
  int8_t counter, answer;
  long previous;

  // First get the NMEA string
  // Clean the input buffer
  while ( mySerial.available() > 0) mySerial.read();
  // request Basic string
  mySerial.println("AT+CGNSINF"); 

  counter = 0;
  answer = 0;
  memset(frame, '\0', sizeof(frame));    // Initialize the string
  previous = millis();
  // this loop waits for the NMEA string
  do {

    if (mySerial.available() != 0) {
      frame[counter] = mySerial.read();
      counter++;
      // check if the desired answer is in the response of the module
      if (strstr(frame, "OK") != NULL)
      {
        answer = 1;
      }
    }
    // Waits for the asnwer with time out
  }
  while ((answer == 0) && ((millis() - previous) < 2000));

  frame[counter - 3] = '\0';
 
  // Parses the string
  strtok(frame, ":");
  strcpy(GNSSrunstatus, strtok(NULL, ",")); // Gets GNSS run status
  strcpy(Fixstatus, strtok(NULL, ",")); // Gets Fix status
  strcpy(UTCdatetime, strtok(NULL, ",")); // Gets UTC date & time
  strcpy(latitude, strtok(NULL, ",")); // Gets latitude
  strcpy(logitude, strtok(NULL, ",")); // Gets longitude
  strcpy(altitude, strtok(NULL, ",")); // Gets MSL altitude
  //Print to test string
  Serial.println("GNSS run status: ");
  Serial.println(GNSSrunstatus);
  Serial.print("Fix status: ");
  Serial.println(Fixstatus);
  Serial.print("UTC datetime: ");
  Serial.println(UTCdatetime);
  myFile.print(UTCdatetime);
  myFile.print(", ");
  Serial.print("Latitude: ");
  Serial.println(latitude);  
  myFile.print(latitude);
  myFile.print(", ");  
  Serial.print("Logitude: ");
  Serial.println(logitude);
  myFile.print(logitude);
  myFile.print(", ");  
  Serial.print("Altitude: ");
  Serial.println(altitude);
  return answer;
}
void getDust(File myFile) {
  delayMicroseconds(samplingTime);
  voMeasured = analogRead(measurePin); // rnameead the dust value
  delayMicroseconds(deltaTime);
  delayMicroseconds(sleepTime);
 
  // 0 - 3.3V mapped to 0 - 1023 integer values
  // recover voltage
  calcVoltage = voMeasured * (5.0 / 1024);
 
  // linear eqaution taken from http://www.howmuchsnow.com/arduino/airquality/
  // Chris Nafis (c) 2012
  dustDensity = (0.17 * calcVoltage - 0.1) * 1000;
  myFile.print(dustDensity);
  myFile.print(", ");
  
  Serial.print("Dust Density: ");
  Serial.println(dustDensity);
}
void get_NO2(File myFile) {
  //*****NO2*****//
  no2_concentration = (110/(analogRead(no2_pin)/1024.0) -22) / 22;
  myFile.print(no2_concentration);
  myFile.print(", ");
  Serial.print("NO2 concentration: ");
  Serial.println(no2_concentration);
  //*************//
}
void get_O3(File myFile) {
  //*****O3*****//
  o3_cal_voltage = 100 / (5 * 22 * 1024.0 / analogRead(no2_pin) - 22) / 22;
  o3_concentration = 1.15354664573491* pow(o3_cal_voltage, 3) + 95.0232835796925 * o3_cal_voltage +  3.82316977457257;
  myFile.print(o3_concentration);
  myFile.print(", ");
  Serial.print("O3 concentration: ");
  Serial.println(o3_concentration);
  //*************//
}
void get_sound(File myFile) {
  soundValue = digitalRead(soundPin) ; // read the sound alarm time
  if (soundValue == 1) {
    Serial.println("Safe");
    myFile.println(0);
  } else {
    Serial.println("Polluted");
    myFile.println(1);
  }
}

