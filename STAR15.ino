// Aerostar Launch demo
//   This simple demo reads data coming from the Future Engineers TechRise web-based
//     simulator.
//   It is designed to run on a Metro M4 Express and uses the on-board Neopixel.
//   The demo runs a simple update loop, and does three actions:
//     1 - Keeps track of the number of telemetry packets received*
//     2 - Monitors flight status and prints a message each time it changes
//     3 - Prints out every 1000th telemetry packet
//         * Note: Data is sent by the Aerostar flight computer at 10Hz,
//         which is to say: ten packets of data per second.
//    Important note: If the web simulator is run faster than 10x speed, data will
//      not be sent to the microcontroller, as this would flood the USB connection.

// Written by Mark DeLoura for Future Engineers
// Last Updated: 12/12/2022

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <TRSim_Aerostar.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <DFRobot_OzoneSensor.h>

#define COLLECT_NUMBER   20              // collect number, the collection range is 1-100
#define Ozone_IICAddress OZONE_ADDRESS_3
/*   iic slave Address, The default is ADDRESS_3
       ADDRESS_0               0x70      // iic device address
       ADDRESS_1               0x71
       ADDRESS_2               0x72
       ADDRESS_3               0x73
*/
DFRobot_OzoneSensor Ozone;


#define SEALEVELPRESSURE_HPA (1013.25)

float dataInterval = 5.5;
const int chipSelect = 10;

// Set up Neopixel hardware constants and object for the M4's on-board Neopixel
const unsigned int NEOPIXEL_COUNT = 1;
const unsigned int NEOPIXEL_BRIGHTNESS = 0.2;
Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, PIN_NEOPIXEL);

// Set up some basic color constants for use later
const unsigned int COLOR_RED = 0xff0000;
const unsigned int COLOR_GREEN = 0x00ff00;
const unsigned int COLOR_BLUE = 0x0000ff;
const unsigned int COLOR_YELLOW = 0xffff00;
const unsigned int COLOR_MAGENTA = 0xff00ff;
const unsigned int COLOR_CYAN = 0x00ffff;
const unsigned int COLOR_BLACK = 0x000000;
const unsigned int COLOR_GRAY = 0x7f7f7f;
const unsigned int COLOR_WHITE = 0xffffff;

// Set up flight status event Neopixel colors index
unsigned int statusColors[TRSim_Aerostar::STATUS_NUM_EVENTS];

// Set up simulator data library
TRSim_Aerostar::Simulator TRsim;

// Initialize flight status
int currStatus = TRSim_Aerostar::STATUS_UNKNOWN;
int prevStatus = currStatus;

// Variable for tracking number of full telemetry packets received
int numPackets = 0;


// TIMERS 
int launchIncr = 10;
int floatIncr = 30;
int incr = launchIncr;
int timerTrigger = millis()/1000 + incr;
int delayTime = 0;



// BME 
Adafruit_BME280 bme; // I2C

// setup()
//   Initialization functions
//
void setup() {

    // Open serial communications and wait for port to open:
 Serial.begin(SERIAL_BAUD);
 	delay(2000); // Improves chances of seeing first printed messages over USB serial
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  while(!Ozone.begin(Ozone_IICAddress)) {
    Serial.println("I2c device number error !");
    delay(1000);
  }  Serial.println("I2c connect success !");
/*   Set iic mode, active mode or passive mode
       MEASURE_MODE_AUTOMATIC            // active  mode
       MEASURE_MODE_PASSIVE              // passive mode
*/
    Ozone.setModes(MEASURE_MODE_PASSIVE);
    unsigned status;
    
    // default settings
    status = bme.begin();  
    // You can also pass in a Wire library object like &Wire2
    // status = bme.begin(0x76, &Wire2)
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1) delay(10);
    }
    
    Serial.println("-- Default Test --");
    int delayTime = 1000;

    Serial.println();

  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");


	Serial.println("Running Aerostar Launch demo");

	// Set up simulator data library
	//   By default, PBF pin = 2, GO pin = 3. If your wiring is different, see
	//   documentation for how to change pins using this function call.
	TRsim.init();

	// Set up status colors
	statusColors[TRSim_Aerostar::STATUS_UNKNOWN] = COLOR_GRAY;
	statusColors[TRSim_Aerostar::STATUS_INITIALIZING] = COLOR_YELLOW;
	statusColors[TRSim_Aerostar::STATUS_LAUNCHING] = COLOR_GREEN;
	statusColors[TRSim_Aerostar::STATUS_FLOATING] = COLOR_CYAN;
	statusColors[TRSim_Aerostar::STATUS_DESCENDING] = COLOR_BLUE;

	// Display flight status (unknown) on Neopixel
	pixels.begin();
	pixels.setBrightness(255 * NEOPIXEL_BRIGHTNESS);
	pixels.fill(statusColors[currStatus], 0, NEOPIXEL_COUNT);
	pixels.show();

  String dataString = "";
  dataString += "Time" + ",";
  dataString += "BME_TEMP" + ",";
  dataString += "BME_PRESS"+ ",";
  dataString += "BME_ALT"+ ",";
  dataString += "BME_HUM" + ",";
  dataString += "UV"+ ",";
  dataString += "OZONE" + ",";
  dataString += "ALT"+ ",";
  dataString += "LONG" + ",";
  dataString += "LAT";


  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }


}

// loop()
//   Do forever, start the main loop
//
void loop() {
	// Pointer for keeping track of incoming data
	unsigned char *data;

	// Call Simulator update() function to catch serial input and check PBF.
	//   This must be called at the top of the loop.
	TRsim.update();

	// Check if the PBF header is closed (False). If it is, light Neopixel red.
	//   If it is open (True), we are free to do in-flight work!
	if (TRsim.getPBF() == LOW) {
		// Light the neopixel red to highlight that the PBF is inserted
		pixels.fill(COLOR_RED, 0, NEOPIXEL_COUNT);
		pixels.show();
	}
  else {
		// PBF header is open, we are flying and can do some work!

		// TRsim.isStreaming() will be True while valid data is incoming
		// If data is paused for more than 1.5 seconds it will become False
		if (TRsim.isStreaming() == true) {
			// TRsim.isNewData() will be True after a new packet arrives
			// When TRsim.getData() is called TRsim.isNewData() will become False.
			if (TRsim.isNewData() == true) {
				// Got a new telemetry packet, let's count it!
				numPackets += 1;

				// Grab new data - NOTE this sets isNewData() to False
				data = TRsim.getData();

				// You can add code here that needs to execute each time new
				//   telemetry data is received.

				// Get current flight status and check to see if it has changed
				currStatus = TRsim.getStatus();
				// See if the status has changed by comparing with previous value
				if (currStatus != prevStatus) {
					// If it has changed, save the current status to prevStatus
					prevStatus = currStatus;
					// Since the event changed, print something to indicate change.
					// You can initiate activity for your payload in this
					//   section, since it will only execute on status change.
					//   However, when running the simulator, please note that
					//   this section may execute again if you pause and unpause
					//   the simulator.
					if (currStatus == TRSim_Aerostar::STATUS_UNKNOWN) {
						Serial.println("We are unknown");
					} else if (currStatus == TRSim_Aerostar::STATUS_INITIALIZING) {
						Serial.println("We are initializing");
					} else if (currStatus == TRSim_Aerostar::STATUS_LAUNCHING) {
						Serial.println("We are launching");
            incr = launchIncr;
            timerTrigger = millis()/1000 + incr;            
					} else if (currStatus == TRSim_Aerostar::STATUS_FLOATING) {
						Serial.println("We are floating");
            incr = floatIncr;
            timerTrigger = millis()/1000 + incr;      
					} else if (currStatus == TRSim_Aerostar::STATUS_DESCENDING) {
						Serial.println("We are descending");
					}
					// Indicate the new event with a color from the status list
					pixels.fill(statusColors[currStatus], 0, NEOPIXEL_COUNT);
					pixels.show();
				}

				// Print every 1000th packet to verify data
				if ((numPackets % 1000) == 1) {
					TRsim.serialPrintCurrentPacket();
				}
			}
      if (millis() / 1000 > timerTrigger) {
        timerTrigger = millis()/1000 + incr;
        int16_t ozoneConcentration = Ozone.readOzoneData(COLLECT_NUMBER);
        Serial.print("Ozone concentration is ");
        Serial.print(ozoneConcentration);
        Serial.println(" PPB.");
        printValues();
          delay(delayTime);
        // put your main code here, to run repeatedly:
        int sensorValue = analogRead(A0);
        float voltage = sensorValue*3.3/65536;
        float uvIndex = voltage/0.1;

        Serial.print("UV Index :");
        Serial.println(uvIndex);

        //      dataString += ltr.readUVS();
        String dataString = "";
        dataString += String(millis() / 1000) + ",";
        dataString += String(bme.readTemperature()) + ","
        dataString += String(bme.readPressure() / 100.0F)+ ","
        dataString += String(bme.readAltitude(SEALEVELPRESSURE_HPA))+ ","
        dataString += String(bme.readHumidity())+ ","
        dataString += String(uvIndex)+ ","
        dataString += String(ozoneConcentration)
        dataString += String(TRsim.getAltitude()) + ",";
        dataString += String(TRsim.getLongitude()) + ",";
        dataString += String(TRsim.getLatitude());


        File dataFile = SD.open("datalog.txt", FILE_WRITE);  //send datastring to sd card
        if (dataFile) {
          dataFile.println(dataString);
          dataFile.close();
        }
        // if the file isn't open, pop up an error:
        else {
          Serial.println("error opening datalog.txt");
        }

		} 
    else {	// not streaming
			// Data stream has stopped for 1.5s, fill pixels with unknown (idle) color
			pixels.fill(statusColors[TRSim_Aerostar::STATUS_UNKNOWN], 0, NEOPIXEL_COUNT);
			pixels.show();

			// Reset the previous status to unknown (idle)
			prevStatus = TRSim_Aerostar::STATUS_UNKNOWN;
		}
	}
}
	delay(10);
}




void printValues() {
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" °C");

    Serial.print("Pressure = ");

    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");

    Serial.println();
}