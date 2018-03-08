#include <stdio.h>
#include <LiquidCrystal.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <max6675.h>
#include <MsTimer2.h>
#include <MenuBackend.h>
#include <PID_v1_jwm.h>
#include <EEPROM.h>
#include <EEPROMAnything.h>

// config parameters
	const boolean serialEnable = true;
	const boolean serialProcessing = false;
	const boolean debug = true;


// PID controller
  
// config parameters saved to EEPROM
	#define CONFIG_VERSION "p03" // version number 
						//***** (make sure to increment when changing structure)
	#define CONFIG_START 0 //memory address for config
	struct configStruct { // structure to hold settings
		char version[4]; // settings structure version
		float tuningBand;
		float kp;
		float ki;
		float kd;
		float kf;
		float Kp;
		float Ki;
		float Kd;
		float Kf;
		float Wi;
		float ff_zero;
		double setpoint;
	};
	  
	configStruct configurationDefault = { //define defaults
		CONFIG_VERSION,
		0.75, // tuningBand
		200.0, // kp
		2.0, // ki
		100.0, // kd
		5.0, // kf
		5000.0, // Kp
		0, // Ki
		0, // Kd
		0, //Kf
		400, // windup i
		25.0, // ff_zero point for feed forward; proportional to (setpoint - ff_zero)
		55.0 // setpoint
	};
	configStruct configuration = configurationDefault; // initialize to default params
  
	void configRead (){
		if (EEPROM.read(CONFIG_START + 0) == CONFIG_VERSION[0] &&
			EEPROM.read(CONFIG_START + 1) == CONFIG_VERSION[1] &&
			EEPROM.read(CONFIG_START + 2) == CONFIG_VERSION[2] )
			EEPROM_readAnything(CONFIG_START, configuration);
	}

	void configWrite (){
		EEPROM_writeAnything(CONFIG_START, configuration);
	}
	void configDefault(){
	  configuration = configurationDefault;
	}
  
// PID constants
	const int timeLoopOut = 4000; // time for PWM loop (ms)
	const int timeLoopPID = 1500; // time for PID loop (ms)
	const float setpointMax = 80; // setpoint limit (deg C)
	const float setpointMin = 40; // setpoint limit (deg C)
	const float stepTemp = 0.25;  // step size for temp (deg C)
	const float stepK = 0.5; // step size for gain values
  
// PID variables
	double Output = 0; // initial output
	double Input; 
	unsigned long timeStartOut;
	unsigned long serialTime;
	boolean fail = false;  
	boolean pidEngaged = false;
	volatile byte set = 0;
  
  // define the controller
PID pid(&Input, &Output, &configuration.setpoint, 
	configuration.kp, configuration.ki,configuration.kd, 
	configuration.kf, DIRECT);

// PIN DEFINITIONS
	// LCD with 4-bit bus
	const int pinLCDd4 = A2;
	const int pinLCDd5 = A3;
	const int pinLCDd6 = A4;
	const int pinLCDd7 = A5;
	const int pinLCDenable = A1;
	const int pinLCDrs = A0;

	// encoder A & B signals (must be 2 and 3 for interrupt-driven encoder)
	const int encoderPinA = 3;
	const int encoderPinB = 2;

	// Three buttons
	const int pinButtonEnc = 7;
	const int pinButtonBlk = 8;
	const int pinButtonRed = 6;

	const int pinOutput = 5; // PID output

	// thermocouple breakout board MAX6675
	const int pinThemroDO = 12;
	const int pinThemroCS = 4;
	const int pinThemroCLK = 13;

	// OneWire sensor
	const int pinOneWire = 9;

// OneWire definitions
	#define TEMPERATURE_PRECISION 12
	unsigned int milTempUpdateLoop = 750; // min. milliseconds between temp readings
	byte numOWDev; // Number of temperature devices found
	DeviceAddress tempDeviceAddress;
	float tempOW;
	unsigned long milTempUpdateStart;
	// set up the DS18B20 temp sensor OneWire instance
	OneWire oneWire(pinOneWire);

	// Pass oneWire reference to Dallas Temperature. 
	DallasTemperature sensors(&oneWire);

// Thermocouple definitions
	float tempTC;
	boolean thermocoupleActive = true;
	// set up the MAX6675 thermocouple interface
	MAX6675 thermocouple(pinThemroCLK, pinThemroCS, pinThemroDO);


// encoder variables
	volatile boolean A_set = false;
	volatile boolean B_set = false;
	volatile int ctEnc = 0;
	int ctEncLastChunk = 0;

// button variables
	volatile boolean butEnc = false;
	volatile boolean butEncOld = false;
	volatile boolean butBlk = false;
	volatile boolean butBlkOld = false;
	volatile boolean butRed = false;
	volatile boolean butRedOld = false;

// set up the LCD
	LiquidCrystal lcd(pinLCDrs, pinLCDenable, pinLCDd4, pinLCDd5, pinLCDd6, pinLCDd7);
	// make a cute degree symbol
	uint8_t degree[8]  = {140,146,146,140,128,128,128,128};


// MENUS
	volatile boolean doMenuUse = false;
	volatile boolean doMenuRight = false;
	volatile boolean doMenuLeft = false;
	volatile boolean doConfigRead = false;
	volatile boolean doConfigWrite = false;

	MenuBackend menu = MenuBackend(menuUseEvent,menuChangeEvent);
	
	//list of menu items needed to build the menu
	MenuItem menuPID = MenuItem("Toggle PID");
	MenuItem menuSetpoint = MenuItem("Setpoint");
	MenuItem menuConfig = MenuItem("Config");
	
		MenuItem menuConfigRead = MenuItem("Read Cfg");
		MenuItem menuConfigWrite = MenuItem("Write Cfg");
		MenuItem menuConfigDefault = MenuItem("Default Cfg");
		MenuItem menuConfigPar = MenuItem("PID tuning");
	
			MenuItem menuConfigPar_kp = MenuItem("kp");
			MenuItem menuConfigPar_ki = MenuItem("ki");
			MenuItem menuConfigPar_kd = MenuItem("kd");
			MenuItem menuConfigPar_Kp = MenuItem("Kp");
			MenuItem menuConfigPar_Ki = MenuItem("Ki");
			MenuItem menuConfigPar_Kd = MenuItem("Kd");
			MenuItem menuConfigPar_band = MenuItem("Tb");
			MenuItem menuConfigPar_kf = MenuItem("kf");
			MenuItem menuConfigPar_Kf = MenuItem("Kf");
			MenuItem menuConfigPar_Wi = MenuItem("Wi");
			MenuItem menuConfigPar_ff_zero = MenuItem("ff zero");
      
	void menuSetup()
	{
		if(serialEnable && !serialProcessing) Serial.println("Setting up menu...");
		// add the to the menu root
		menu.getRoot().add(menuPID);
		menu.getRoot().add(menuSetpoint);
		menu.getRoot().add(menuConfig);

		// movement for level 0
		menuPID.addBefore(menuConfig);
		menuPID.addAfter(menuSetpoint);
		
		menuSetpoint.addBefore(menuPID);
		menuSetpoint.addAfter(menuConfig);
		
		menuConfig.addBefore(menuSetpoint);
		menuConfig.addAfter(menuPID);
		menuConfig.addRight(menuConfigRead); // next level
		
		// movement for level Config
			menuConfigRead.addBefore(menuConfigPar);
			menuConfigRead.addAfter(menuConfigWrite);
			
			menuConfigWrite.addBefore(menuConfigRead);
			menuConfigWrite.addAfter(menuConfigDefault);
			
			menuConfigDefault.addBefore(menuConfigWrite);
			menuConfigDefault.addAfter(menuConfigPar);
			
			menuConfigPar.addBefore(menuConfigDefault);
			menuConfigPar.addAfter(menuConfigRead);
			menuConfigPar.addRight(menuConfigPar_kp); // next level
			
			// left movement points back from anywhere
			menuConfigRead.addLeft(menuConfig);
			menuConfigWrite.addLeft(menuConfig);
			menuConfigDefault.addLeft(menuConfig);
			menuConfigPar.addLeft(menuConfig);
		
		// movement for level ConfigPar
			menuConfigPar_kp.addBefore(menuConfigPar_ff_zero);
			menuConfigPar_kp.addAfter(menuConfigPar_ki);
			
			menuConfigPar_ki.addBefore(menuConfigPar_kp);
			menuConfigPar_ki.addAfter(menuConfigPar_kd);
			
			menuConfigPar_kd.addBefore(menuConfigPar_ki);
			menuConfigPar_kd.addAfter(menuConfigPar_kf);
			
			menuConfigPar_kf.addBefore(menuConfigPar_kd);
			menuConfigPar_kf.addAfter(menuConfigPar_Kp);
			
			menuConfigPar_Kp.addBefore(menuConfigPar_kf);
			menuConfigPar_Kp.addAfter(menuConfigPar_Ki);
			
			menuConfigPar_Ki.addBefore(menuConfigPar_Kp);
			menuConfigPar_Ki.addAfter(menuConfigPar_Kd);
			
			menuConfigPar_Kd.addBefore(menuConfigPar_Ki);
			menuConfigPar_Kd.addAfter(menuConfigPar_Kf);
			
			menuConfigPar_Kf.addBefore(menuConfigPar_Kd);
			menuConfigPar_Kf.addAfter(menuConfigPar_band);
				
			menuConfigPar_band.addBefore(menuConfigPar_Kf);
			menuConfigPar_band.addAfter(menuConfigPar_Wi);
			
			menuConfigPar_Wi.addBefore(menuConfigPar_band);
			menuConfigPar_Wi.addAfter(menuConfigPar_ff_zero);
			
			menuConfigPar_ff_zero.addBefore(menuConfigPar_Wi);
			menuConfigPar_ff_zero.addAfter(menuConfigPar_kp);
			
			// left to move up a level
			menuConfigPar_kp.addLeft(menuConfigPar);
			menuConfigPar_ki.addLeft(menuConfigPar);
			menuConfigPar_kd.addLeft(menuConfigPar);
			menuConfigPar_Kp.addLeft(menuConfigPar);
			menuConfigPar_Ki.addLeft(menuConfigPar);
			menuConfigPar_Kd.addLeft(menuConfigPar);
			menuConfigPar_band.addLeft(menuConfigPar);
			menuConfigPar_kf.addLeft(menuConfigPar);
			menuConfigPar_Kf.addLeft(menuConfigPar);
			menuConfigPar_Wi.addLeft(menuConfigPar);
			menuConfigPar_ff_zero.addLeft(menuConfigPar);
	}

	//  define menu-item behaviors
	void menuUseEvent(MenuUseEvent used)
	{
		if (used.item == menuPID) {  
			lcd.setCursor(17, 1); 
			if (!fail) {
				if (pid.GetMode() == 0){
					pid.SetMode(AUTOMATIC);
					pidEngaged = true;
					lcd.print(" On");
				}
				else if (pid.GetMode() == 1){
					pid.SetMode(MANUAL);
					Output = 0;
					pidEngaged = false;
					lcd.print("Off");
				}
			}
			else if (fail) {
				if (pidEngaged){
					Output = 0;
					pidEngaged = false;
					lcd.print("Off");
				}
				else {
					pidEngaged = true;
					lcd.print(" On");
				}
			}
		} // end menuPID
		else if (used.item == menuSetpoint) setTest(1);
		else if (used.item == menuConfig || used.item == menuConfigPar)  menu.moveRight();
		else if (used.item == menuConfigRead) configRead();
		else if (used.item == menuConfigWrite) configWrite();
		else if (used.item == menuConfigDefault) configDefault();
		else if (used.item == menuConfigPar_kp) setTest(2);
		else if (used.item == menuConfigPar_ki) setTest(3);
		else if (used.item == menuConfigPar_kd) setTest(4);
		else if (used.item == menuConfigPar_Kp) setTest(5);
		else if (used.item == menuConfigPar_Ki) setTest(6);
		else if (used.item == menuConfigPar_Kd) setTest(7);
		else if (used.item == menuConfigPar_band) setTest(8);
		else if (used.item == menuConfigPar_kf) setTest(9);
		else if (used.item == menuConfigPar_Kf) setTest(10);
		else if (used.item == menuConfigPar_Wi) setTest(11);
		else if (used.item == menuConfigPar_ff_zero) setTest(12);
	}
	void setTest(byte typeS){
		if (set != typeS) {
			set = typeS;
			lcd.setCursor(0, 1);
			lcd.print("**");
		}
		else if (typeS == 1) set = 255; // exit set mode, redraw SP
		else set = 254; // exit set mode
	}

	//  notification when the menu changes
	void menuChangeEvent(MenuChangeEvent changed)
	{
		lcd.setCursor(0,0);
		lcd.print("           ");
		lcd.setCursor(0,0);
		lcd.print(changed.to.getName());
		
		if (changed.to == menuConfigPar_kp) setAny(2,0);
		else if (changed.to == menuConfigPar_ki) setAny(3,0);
		else if (changed.to == menuConfigPar_kd) setAny(4,0);
		else if (changed.to == menuConfigPar_Kp) setAny(5,0);
		else if (changed.to == menuConfigPar_Ki) setAny(6,0);
		else if (changed.to == menuConfigPar_Kd) setAny(7,0);
		else if (changed.to == menuConfigPar_band) setAny(8,0);
		else if (changed.to == menuConfigPar_kf) setAny(9,0);
		else if (changed.to == menuConfigPar_Kf) setAny(10,0);
		else if (changed.to == menuConfigPar_Wi) setAny(11,0);
		else if (changed.to == menuConfigPar_ff_zero) setAny(12,0);
		else if (changed.to == menuConfigPar) set = 255;
	}

// FAILSAFE and RECOVER 	
	void failsafe() {
		fail = true;
		pid.SetMode(MANUAL);
		lcd.setCursor(17, 1); 
		lcd.print("Err");
		if (pidEngaged) Output = configuration.kf * (configuration.setpoint - configuration.ff_zero);
		else Output = 0;
	}

	void recover() {
		fail = false;
		lcd.setCursor(17, 1);
		if (pidEngaged) {
			pid.SetMode(AUTOMATIC);
			lcd.print(" On");
		} else lcd.print("Off");
	}
  
void setup() {
	pinMode(pinOutput, OUTPUT);
	pinMode(encoderPinA, INPUT);  digitalWrite(encoderPinA, LOW);
	pinMode(encoderPinB, INPUT);  digitalWrite(encoderPinB, LOW);
	
	pinMode(pinButtonEnc, INPUT);  digitalWrite(pinButtonEnc, LOW);
	pinMode(pinButtonBlk, INPUT);  digitalWrite(pinButtonBlk, LOW);
	pinMode(pinButtonRed, INPUT);  digitalWrite(pinButtonRed, LOW);

	// encoder pin interrupts
	attachInterrupt(0, doEncoderA, CHANGE);
	attachInterrupt(1, doEncoderB, CHANGE);

	// set up the LCD's number of columns and rows: 
	lcd.begin(20, 4);
	lcd.createChar(0, degree);
	// write fixed text to the LCD.
	//  lcd.setCursor(0,0); // row 1
	//  lcd.print("");
	lcd.setCursor(0,1); // row 2
	lcd.print("SP:");
	lcd.setCursor(0,2); // row 3
	lcd.print("T1:");
	lcd.setCursor(0,3); // row 4
	lcd.print("T2:");

	// PID Setup
	// read the configuration structure if available
	configRead();
	// set tunings from config
	pid.SetTunings(configuration.kp, configuration.ki, configuration.kd, configuration.kf); 
	pid.SetOutputLimits(0,timeLoopOut); //PID output from 0 to timeLoopOut (so full output = one loop) 
	pid.SetSampleTime(timeLoopPID); // PID sample time
	pid.SetWindupI(configuration.Wi); // PID integral windup limit
	pid.SetFFzero(configuration.ff_zero); // PID feedforward zero point
	//  pid.SetMode(AUTOMATIC); // uncomment to make the controller start by default
	lcd.setCursor(17,1);
	pid.GetMode() ? lcd.print(" On") : lcd.print("Off");

	// Setpoint = configuration.setpointInit; // initialize the setpoint
	lcd.setCursor(4, 1);
	lcd.print(configuration.setpoint);
	
	// serial
	if (serialEnable||serialProcessing||debug) Serial.begin(115200);
	
	// temp sensor
	sensors.begin();
	sensors.setWaitForConversion(false); // asynchronous mode
	numOWDev = sensors.getDeviceCount();
	if(serialEnable) {
	Serial.print("Locating devices...");
	Serial.print("Found ");
	Serial.print(numOWDev, DEC);
	Serial.println(" devices.");
	Serial.print("Parasite power is: "); 
	if (sensors.isParasitePowerMode()) Serial.println("ON");
	else Serial.println("OFF");
	}
	// Loop through each device, print out address
	if (numOWDev){
		for(byte i=0;i<numOWDev; i++)
		{
			// Search the wire for address
			if(sensors.getAddress(tempDeviceAddress, i)){
				sensors.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION);
				if(serialEnable) {
				Serial.print("Found device ");
				Serial.print(i, DEC);
				Serial.print(" with address: ");
				printAddress(tempDeviceAddress);
				Serial.println();
				Serial.print("Setting resolution to ");
				Serial.println(TEMPERATURE_PRECISION,DEC);
				Serial.print("Resolution actually set to: ");
				Serial.print(sensors.getResolution(tempDeviceAddress), DEC); 
				Serial.println();
			}
			}
			else if(serialEnable) {
				Serial.print("Found ghost device at ");
				Serial.print(i, DEC);
				Serial.print(" but could not detect address. Check power and cabling");
			} 
		} // end onewire search    
	} // end if (numOWDev)
	else {
		lcd.setCursor(8,2);
		lcd.print("n/a");
	}
	// menus
	menuSetup();
	lcd.setCursor(0,0);
	menu.moveDown();
	
	// set up the timer for detecting button pushes
	MsTimer2::set(10,timer2);
	MsTimer2::start();
	
} // end setup

void timer2() {
	// 
	// PIND7 = d7 (encoder push) -> 7
	// PINB0 = d8 (black)
	// PINC4 = a4 (red) [v1] || PIND6 = d6 (red) [v2]
	butEnc = bitRead(PIND,7); // read enc push
	if (butEnc != butEncOld) { // encoder button changed
		if (butEnc == true) doMenuUse = true;
		butEncOld = butEnc;
	}
	butBlk = bitRead(PINB,0); // read black
	if (butBlk != butBlkOld) { // black changed
		if (butBlk == true && set == 0) doMenuLeft = true;
		butBlkOld = butBlk;
	}
	butRed = bitRead(PIND,6); // read red
	if (butRed != butRedOld) { // red changed
		if (butRed == true && set == 0) doMenuRight = true;
		butRedOld = butRed;
	}
	//// create a slow PWM output
	unsigned long now = millis();
	// advance the window if past the old window
	if (now - timeStartOut > timeLoopOut) timeStartOut += timeLoopOut; 
	// set the output
	if (Output > now - timeStartOut) digitalWrite(pinOutput,HIGH);
	else digitalWrite(pinOutput,LOW);
} // end timer2 ISR

void loop() {
	
	// timer in upper right corner
	char dispTimer[10] ; //  "000:00:00"
	unsigned long timeS = millis()/1000;
	unsigned long timeS_last;
	unsigned long timeH = timeS/3600;
	unsigned long timeM = (timeS - timeH*3600) / 60;
	timeS = timeS - timeH*3600 - timeM*60;
	if (timeS != timeS_last){
		sprintf(dispTimer, "%3lu:%02lu:%02lu", timeH, timeM, timeS);
		lcd.setCursor(11, 0); 
		lcd.print(dispTimer); 
		timeS_last = timeS;
	}
	
	// check if menu actions pending
	if (doMenuUse)  {
		menu.use();
		doMenuUse = false;
	}
	else if (doMenuRight) {
		menu.use(); // if menu has subitems, then move right in the menuUseEvent function
		doMenuRight = false;
	}
	else if (doMenuLeft)  {
		if (set == 0) menu.moveLeft();
		else set = 255;
		doMenuLeft = false;
	}  
	// parameter adjustment
	if (set == 1 || set == 8) setAny(set, stepTemp);
	else if (set >= 2 && set <= 200) setAny(set, stepK);
	else if (set == 255){ // redraw setpoint
		lcd.setCursor(0, 1);
		lcd.print("SP: ");      
		lcd.print(configuration.setpoint);
		set = 0;
		if(serialEnable) Serial.println("set = 0 (none)");
	}
	else if (set == 254){  // exit set mode
		lcd.setCursor(0,1);
		lcd.print("  ");
		set = 0;
	}
	// advance up/dn menu items
	else  if (abs(ctEnc - ctEncLastChunk) > 8) {
		if (ctEnc - ctEncLastChunk > 8)  menu.moveDown();
		else if (ctEncLastChunk - ctEnc > 8)  menu.moveUp();
		ctEncLastChunk = ctEnc;  // reset encoder chunk
	} //end menu scrolling 
	
	// temperature update timer
	if (millis() - milTempUpdateStart > milTempUpdateLoop)
	{
		milTempUpdateStart += milTempUpdateLoop;
		
		// thermocouple temp  
		// if (thermocoupleActive){
			char dispTC[9];
			if(serialEnable) Serial.print("Reading thermocouple:"); 
			tempTC = thermocouple.readCelsius();
			if (isnan(tempTC))
			{
				if(serialEnable) Serial.println("n/a");
				if (thermocoupleActive) {
				lcd.setCursor(3,3);
				lcd.print("          ");
				lcd.setCursor(4,3);
				lcd.print("n/a");
				thermocoupleActive = false;
				}
			}
			else
			{
				if(serialEnable) Serial.println(tempTC);
				lcd.setCursor(3,3);
				lcd.print(dtostrf(tempTC,6,2,dispTC));
				lcd.print(" ");
				lcd.write((uint8_t)0);
				lcd.print("C");
			}
		// } // end thermocouple section
		
		// OneWire temp
		if (numOWDev)
		{
			// get temperature
			if(serialEnable) Serial.print("Requesting temperatures...");
			sensors.requestTemperatures(); // update all sensors
			tempOW = sensors.getTempCByIndex(0); // retrieve last sensor read of index 0
			if(serialEnable) Serial.println(tempOW);
			lcd.setCursor(9,2);
			lcd.print("  ");
			lcd.setCursor(4,2);
			lcd.print(tempOW,3);
			lcd.write((uint8_t)0);
			lcd.print("C");
		} // end OneWire temp
		
		// temperature handling and failsafe/recovery
		if (numOWDev && (int)tempOW + 127 > 0){
			Input = tempOW;
			if (fail) recover();
		}
		// OneWire sensor has failed but worked at startup
		else if (numOWDev) failsafe();
		// use thermocouple for PID
		else if (thermocoupleActive) Input = tempTC;
		else failsafe();
	} // end update temps 
	
	// PID SECTION
	// set the tuning parameters based on error
	if (abs(configuration.setpoint-Input) > configuration.tuningBand) 
	pid.SetTunings(configuration.Kp, configuration.Ki, configuration.Kd, configuration.Kf); //aggressive
	else 
	pid.SetTunings(configuration.kp, configuration.ki, configuration.kd, configuration.kf); //comparatively moderate
	
	// compute PID if needed
	pid.Compute();
	// indicate output percentage on LCD
	lcd.setCursor(10,1);
	LCDPrintFloat(Output*100/timeLoopOut, 1); // prints a space-padded float with one-decimal precision
	lcd.print("%");
	
	// interface with processing front end
	if(millis()>serialTime && serialProcessing)
	{
		SerialReceive();
		SerialSend();
		serialTime+=500;
	}
}

// Interrupt on A changing state
void doEncoderA()
{
	// Test transition
	A_set = digitalRead(encoderPinA) == HIGH;
	// and adjust counter + if A leads B
	ctEnc += (A_set != B_set) ? +1 : -1;
}

// Interrupt on B changing state
void doEncoderB()
{
	// Test transition
	B_set = digitalRead(encoderPinB) == HIGH;
	// and adjust counter + if B follows A
	ctEnc += (A_set == B_set) ? +1 : -1;
}

void setAny (byte set, float step) {
	float temp;
	byte change;
	char dispValue[9];
	
	if (ctEnc - ctEncLastChunk >= 3) {
		change = 1;
		ctEncLastChunk = ctEnc;  // reset encoder chunk
	}
	else if (ctEncLastChunk - ctEnc >= 3) {
		change = 2;
		ctEncLastChunk = ctEnc;  // reset encoder chunk
	}
	else change = 0;
	
	if (step == 0) {
		lcd.setCursor(0,1);
		lcd.print("  ");
	}
	
	if (digitalRead(pinButtonRed) == 1) step = 100*step; // fast with btn held
	else if (digitalRead(pinButtonBlk) == 1) step = 10*step; // fast with btn held
	
	if (change == 2) step = step * -1;
	else if (change == 0) step = 0;
	
	switch (set) {
	case 1: // setpoint
		configuration.setpoint = constrain(configuration.setpoint + step, setpointMin, setpointMax);
		temp = configuration.setpoint;
		break;
	case 2: // kp
		configuration.kp += step;
		temp = configuration.kp;
		break;
	case 3: // ki
		configuration.ki += step;
		temp = configuration.ki;
		break;
	case 4: // kd
		configuration.kd += step;
		temp = configuration.kd;
		break;
	case 5: // Kp
		configuration.Kp += step;
		temp = configuration.Kp;
		break;
	case 6: // Ki
		configuration.Ki += step;
		temp = configuration.Ki;
		break;
	case 7: // Kd
		configuration.Kd += step;
		temp = configuration.Kd;
		break;
	case 8: // band
		configuration.tuningBand += step;
		temp = configuration.tuningBand;
		break;
	case 9: //kf
		configuration.kf += step;
		temp = configuration.kf;
		break;
	case 10: //Kf
		configuration.Kf += step;
		temp = configuration.Kf;
		break;
	case 11: // Wi
		configuration.Wi += step;
		temp = configuration.Wi;
		break;
	case 12: // ff_zero
		configuration.ff_zero += step;
		temp = configuration.ff_zero;
		break;
	} // end switch-case
	// print value regardless of  change
	lcd.setCursor(4, 1);
	lcd.print(dtostrf(temp,-8,2,dispValue));
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
	for (uint8_t i = 0; i < 8; i++){
		if (deviceAddress[i] < 16) Serial.print("0");
		Serial.print(deviceAddress[i], HEX);
	}
}

void LCDPrintFloat( float f, byte n){ // LCDPrintFloat(input, num_digits);
	if ((int)f < 10) lcd.print("  ");
	else if ((int)f < 100) lcd.print(" ");
	lcd.print((int)f);
	lcd.print(".");
	int temp;
	if (n == 2)      temp = (int)((f - (int)f) * 100);
	else if (n == 1) temp = (int)((f - (int)f) * 10);
	lcd.print( abs(temp) );
}

void LCDnum3(int data){
	if (data < 10)       lcd.print("  ");
	else if (data < 100) lcd.print(" ");
	lcd.print(data);
}

 
/********************************************
 * Serial Communication functions / helpers for PID connection to Processing
 ********************************************/


union { 			// This Data structure lets
	byte asBytes[24];	// us take the byte array
	float asFloat[6];	// sent from processing and
}				// easily convert it to a
foo;				// float array

// getting float values from processing into the arduino
// was no small task.  the way this program does it is
// as follows:
//  * a float takes up 4 bytes.  in processing, convert
//    the array of floats we want to send, into an array
//    of bytes.
//  * send the bytes to the arduino
//  * use a data structure known as a union to convert
//    the array of bytes back into an array of floats

//  the bytes coming from the arduino follow the following
//  format:
//  0: 0=Manual, 1=Auto, else = ? error ?
//  1-4: float setpoint
//  5-8: float input
//  9-12: float output  
//  13-16: float P_Param
//  17-20: float I_Param
//  21-24: float D_Param

void SerialReceive()
{
	// read the bytes sent from Processing
	int index=0;
	byte Auto_Man = -1;
	byte Direct_Reverse = -1;
	while(Serial.available()&&index<26)
	{
		if(index==0) Auto_Man = Serial.read();
		else if(index==1) Direct_Reverse = Serial.read();
		else foo.asBytes[index-2] = Serial.read();
		index++;
	} 
	
	// if the information we got was in the correct format, 
	// read it into the system
	if(index==26  && (Auto_Man==0 || Auto_Man==1)&& (Direct_Reverse==0 || Direct_Reverse==1))
	{
		configuration.setpoint=double(foo.asFloat[0]);
		//Input=double(foo.asFloat[1]);       // * the user has the ability to send the 
						//   value of "Input"  in most cases (as 
						//   in this one) this is not needed.
		if(Auto_Man==0)                       // * only change the output if we are in 
		{                                     //   manual mode.  otherwise we'll get an
			Output=double(foo.asFloat[2]);      //   output blip, then the controller will 
		}                                     //   overwrite.
		
		// double p, i, d;                       // * read in and set the controller tunings
		configuration.kp = double(foo.asFloat[3]);           //
		configuration.ki = double(foo.asFloat[4]);           //
		configuration.kd = double(foo.asFloat[5]);           //
		pid.SetTunings(configuration.kp, configuration.ki, configuration.kd, configuration.kf);            //
		
		if(Auto_Man==0) pid.SetMode(MANUAL);// * set the controller mode
		else pid.SetMode(AUTOMATIC);             //
		
		if(Direct_Reverse==0) pid.SetControllerDirection(DIRECT);// * set the controller Direction
		else pid.SetControllerDirection(REVERSE);          //
	}
	Serial.flush();                         // * clear any random data from the serial buffer
}

// unlike our tiny microprocessor, the processing ap
// has no problem converting strings into floats, so
// we can just send strings.  much easier than getting
// floats from processing to here no?
void SerialSend()
{
	Serial.print("PID ");
	Serial.print(configuration.setpoint);   
	Serial.print(" ");
	Serial.print(Input);   
	Serial.print(" ");
	Serial.print(Output);   
	Serial.print(" ");
	Serial.print(pid.GetKp());   
	Serial.print(" ");
	Serial.print(pid.GetKi());   
	Serial.print(" ");
	Serial.print(pid.GetKd());   
	Serial.print(" ");
	if(pid.GetMode()==AUTOMATIC) Serial.print("Automatic");
	else Serial.print("Manual");
	Serial.print(" ");
	if(pid.GetDirection()==DIRECT) Serial.println("Direct");
	else Serial.println("Reverse");
}

