#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <LiquidCrystal_I2C.h>
#include <MINDSi.h>
#include "logo.h"
#include "greatcircle.h"
#include "MS5611.h"
#include "MPU.h"
#include "NMEA.h"

#define testMode 2
// #define radio true

const int I2C_ADDR = 0x20;
const int BACKLIGHT_PIN = 7;
const int En_pin = 4;
const int Rw_pin = 5;
const int Rs_pin = 6;
const int D4_pin = 0;
const int D5_pin = 1;
const int D6_pin = 2;
const int D7_pin = 3;
LiquidCrystal_I2C  lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);

const int pingPins[5] = {12,12,12,12,12};
const int radioPins[3] = {A1,A2,A1}; //radio Drive, Steer, Emergency shutoff
const int servoPins[2] = {10,11}; //Drive, steer
const int buttonPin = A0;
const String pingLabel[5] = {"Port     ","Left     ","Front    ","Right    ","Starboard"};
const String mindsi =   "MINDS-i  ";
const String robotics = "Robotics ";
const int msInInch = 148;
const int msInCm = 58;
float RPMtoMPH = ((5. *PI)*60.)/(63360.*(37./13.));
float STPtoFeet = (5. *PI)/(12.*100.*(37./13.));

const int numWindows = 8;
void (*window[numWindows])() = { pingExpo, pingDetail, GPS, weather, Compass, Auto, Manual, IMU};
enum charSet currentCharSet;
enum buttonSet currentButton;
bool onRise=false, longRise = false;
bool radioFound=false;

MsBarometer baro;
Servo drive, steer;
NMEA nmea(Serial1);

#ifndef testMode
	int active=0;
#else
	int active = testMode;
#endif

void setup(){
	lcd.begin (20,4);
	lcd.setBacklightPin(BACKLIGHT_PIN,NEGATIVE);
	lcd.setBacklight(true);
	lcd.noCursor();
	lcd.backlight();
	lcd.noAutoscroll();
	#ifndef testMode
		Logo();
	#else
		lcd.clear();
		delay(100);
	#endif
	noInterrupts();           // disable all interrupts
	TCCR4A = 0;
	TCCR4B = 0;
	TCCR4B |= (1 << CS11);    // 8 prescaler
	TIMSK4 |= (1 << TOIE1);   // enable timer overflow interrupt
	interrupts();

	Serial1.begin(38400);
	pinMode(27,OUTPUT);
	beginCompass();
	baro.init();
	InitMPU();
	startEncoder();
	getRadio(radioPins[0]); if(timeLastSignal() != 0) radioFound = true;
	drive.attach(servoPins[0]); drive.write(90);
	steer.attach(servoPins[1]); steer.write(90);
}

ISR(TIMER4_OVF_vect){
	int tmpButton = analogRead(buttonPin)/125;
	onRise |= (currentButton != tmpButton);
	switch(tmpButton){
		case 0:
			currentButton = LEFT;
			break;
		case 1:
			currentButton = UP;
			break;
		case 2:
			currentButton = DOWN;
			break;
		case 4:
			currentButton = RIGHT;
			break;
		case 5:
			currentButton = START;
			break;
		default:
			currentButton = FREE;
	}
}

void loop(){
	longRise = onRise;
	if(onRise && currentButton == LEFT) {
		active -= 1;
		lcd.clear();
	}
	else if(onRise && currentButton == RIGHT) {
		active +=1;
		lcd.clear();
	}
	onRise = false;
	if(active >= numWindows) active = 0;
	if(active < 0) active = (numWindows-1);

	if(radioFound){
		if(timeLastSignal()+200 < millis()) radioFound = false;
		if(getRadio(radioPins[2]) > 135) radioOverride();
		else window[active]();
	} else {
		window[active]();
	}

	lcd.setCursor(8,0);
		lcd.print("Pwr");
		printInPlace(3, ((drive.read()-90)/.9) );
		lcd.print("Str");
		printInPlace(3, steer.read());
	if(window[active]!=Auto) radioDrive();
}

void radioOverride(){
	lcd.setCursor(0,0);
	lcd.clear();
	drive.write(90);
	lcd.print("Radio lock engaged; motion halted");
	while(getRadio(radioPins[2]) > 150) delay(1);
	onRise = false;
	lcd.clear();
}

void Logo(){
	setCustomChars(EYE);
	lcd.home();
	const int frames = 40;
	const int slide = 200;

	for(int pos=-3; pos< 5; pos++){
		for(int i=0; i<4; i++){
			if(pos+i >= 0){
				lcd.setCursor(pos+i,1);
				lcd.send(i,1);
				lcd.setCursor(pos+i,2);
				lcd.send(i+4,1);
			}
		}
		if(pos-1 >= 0){
			lcd.setCursor(pos-1,1);
			lcd.print(" ");
			lcd.setCursor(pos-1,2);
			lcd.print(" ");
		}
		delay(slide);
	}
	for(int pos=20; pos>7; pos--){
		for(int i=0; i<9; i++){
			if(pos+i < 20){
				lcd.setCursor(pos+i, 1);
				lcd.print(mindsi[i]);
				lcd.setCursor(pos+i, 2);
				lcd.print(robotics[i]);
			}
		}
		delay(slide);
	}

	for(int step = 2; step<8; step++){
		setCustomChars((charSet) step);
		delay(frames);
	}
	delay(frames);
	for(int step = 7; step>0; step--){
		setCustomChars((charSet) step);
		delay(frames);
	}

	lcd.clear();
}
/////////// Windows /////////////////////////////
void pingExpo(){
	const static int SCALE = 200;
	setCustomChars(VERT);
	lcd.home();
	lcd.setCursor(0,0);
	lcd.print("P");
	lcd.setCursor(0,1);
	lcd.print("I");
	lcd.setCursor(0,2);
	lcd.print("N");
	lcd.setCursor(0,3);
	lcd.print("G");

	int port = 678;
	int left = getPing(12);
	int front = 1800;
	int right = 32;
	int star = 789;

	verticalBarGraph(0,1,  3,3, port/SCALE);
	verticalBarGraph(1,5,  3,7, left/SCALE);
	verticalBarGraph(1,9,  3,11, front/SCALE);
	verticalBarGraph(1,13, 3,15, right/SCALE);
	verticalBarGraph(1,17, 3,19, star/SCALE);

	delay(10);
}

void pingDetail(){
	setCustomChars(HORZ);
	static int activePing = 2;

	if(longRise && currentButton == UP) activePing++;
	else if(longRise && currentButton == DOWN) activePing--;
	if(activePing < 0) activePing = 4;
	else if(activePing > 4) activePing = 0;

	lcd.setCursor(0,0);
		lcd.print("Ping #");
		lcd.print(activePing);
		// lcd.print(", ");
		// lcd.print(pingLabel[activePing]);

	int tmp = getPing(pingPins[activePing]);
	horizontalBarGraph(1,0, 1,19, (tmp/50));

	lcd.setCursor(0,2);
		printInPlace(5,tmp);
		lcd.print(" ms");
		lcd.setCursor(15, 2);
		printInPlace(2,tmp/msInInch);
		lcd.print(" in");
	lcd.setCursor(1,3);
		printInPlace(3, tmp/msInCm);
		lcd.print(" cm");
		lcd.setCursor(13,3);
		lcd.print("up/down");

	delay(10);
}

void GPS(){
	nmea.update();
	lcd.setCursor(0,0);
		lcd.print("GPS ");
		if(nmea.getWarning()) lcd.print("(C)");//onnected)");
		else lcd.print("(N)");//o Signal)");
	lcd.setCursor(0,1);
		lcd.print("Latitude: ");
		printInPlace(10, nmea.getLatitude());
	lcd.setCursor(0,2);
		lcd.print("Longitude:");
		printInPlace(10, nmea.getLongitude());
	lcd.setCursor(0,3);
		lcd.print("Mag Decl: ");
		printInPlace(10, nmea.getMagVar());
}

void weather(){
	lcd.setCursor(0,0);
	lcd.print("Baro");//meter Data:");
	baro.takeReading();
	lcd.setCursor(0,1);
		printInPlace(5, baro.lastTemp());
		lcd.print(" C");
		lcd.setCursor(12,1);
		lcd.print( (baro.lastTemp()*1.8)+32 );
		lcd.print(" F");
	lcd.setCursor(0,2);
		lcd.print(baro.lastPressure());
		lcd.print("hPa ");
		lcd.setCursor(11,2);
		lcd.print(baro.lastPressure()*0.0295333727);
		lcd.print("inHg");
	lcd.setCursor(0,3);
		float tmp = baro.lastPressure();
		tmp = log10(tmp/1013.25)/5.2558797;
		tmp = pow(10, tmp) -1;
		tmp = tmp/(-0.0000068755856);
		lcd.print(tmp);
		lcd.print(" ft (rough)");
}

void Compass(){
	static int rawX, rawY, rawZ;
	rawCompass(&rawX, &rawY, &rawZ);
	lcd.setCursor(0,0);
		lcd.print("Comp");//3-axis Compass");
	lcd.setCursor(0,1);
		lcd.print("Raw Values");
	lcd.setCursor(0,2);
		lcd.print("X:");
		printInPlace(4,rawX);
		lcd.print(" Y:");
		printInPlace(4,rawY);
		lcd.print(" Z:");
		printInPlace(4,rawZ);
	lcd.setCursor(0,3);
		lcd.print("Heading: ");
		lcd.print(getHeading(0,1,0,1)+nmea.getMagVar());
}

void Auto(){
	setCustomChars(HORZ);
	String msg;
	AutonoDrive(msg);

	lcd.setCursor(0,0);
		lcd.print("Auto");//nomous mode");
	lcd.setCursor(0,1);
		lcd.print("Pwr:");
		printInPlace(3, ((drive.read()-90)/.9) );
		lcd.print("% Steer: ");
		printInPlace(3, steer.read());
		lcd.send(6,1);
	lcd.setCursor(0,2);
		lcd.print("Mot: ");
		printInPlace(4, getRPM());
		lcd.print("RPM, ");
		printInPlace(4, getSteps()*STPtoFeet);
		lcd.print("Ft");
	lcd.setCursor(0,3);
		lcd.print(msg);
}

void Manual(){
	setCustomChars(HORZ);
	//radioDrive();

	lcd.setCursor(0,0);
		lcd.print("Manual");// driving mode");
	lcd.setCursor(0,1);
		lcd.print("Pwr:");
		printInPlace(3, ((drive.read()-90)/.9) );
		lcd.print("% Steer: ");
		printInPlace(3, steer.read());
		lcd.send(6,1);
	lcd.setCursor(0,2);
		lcd.print("Spd: ");
		printInPlace(3, getRPM()*RPMtoMPH);
		lcd.print("MPH, ");
		printInPlace(4, getRPM());
		lcd.print("RPM");
	lcd.setCursor(0,3);
		lcd.print("Dist: ");
		printInPlace(5, getSteps()*STPtoFeet);
		lcd.print("ft, ");
		printInPlace(4, getSteps()*STPtoFeet/5280.);
		lcd.print("M");
}

void IMU(){
	lcd.setCursor(0,0);
	lcd.print("IMU");//Accel & Gyro Raw");
	lcd.setCursor(0,1);
		lcd.print("X:");
		printInPlace(6,MPU_Ax());
		lcd.print("G");
		lcd.setCursor(11,1);
		printInPlace(6,MPU_Gx());
		lcd.print("rps");
	lcd.setCursor(0,2);
		lcd.print("Y:");
		printInPlace(6,MPU_Ay());
		lcd.print("G");
		lcd.setCursor(11,2);
		printInPlace(6,MPU_Gy());
		lcd.print("rps");
	lcd.setCursor(0,3);
		lcd.print("Z:");
		printInPlace(6,MPU_Az());
		lcd.print("G");
		lcd.setCursor(11,3);
		printInPlace(6,MPU_Gz());
		lcd.print("rps");
	delay(50);
}
/////////// Driving /////////////////////////////////
void radioDrive(){
	static float driveOut, steerOut;
	static float driveIn, steerIn;
	if(radioFound){
		driveIn = (driveIn*.6) + (getRadio(radioPins[0])*.4);
		steerIn = (steerIn*.6) + (getRadio(radioPins[1])*.4);

		driveOut = driveIn-90;
		steerOut = steerIn-90;
		driveOut = driveOut*driveOut*driveOut/8100; //cubic motor curve
		steerOut = steerOut*steerOut*steerOut/8100; //729000 = 90^3
		driveOut += 90;
		steerOut += 90;

		if((driveOut > 95 || driveOut < 85)) steer.write(driveOut);
		else steer.write(90);
		if((steerOut > 95 || steerOut < 85)) drive.write(steerOut);
		else drive.write(90);
	}
}

void AutonoDrive(String& msg){
	const static int CENTER = 90;
	const static int TURN = 45;
	const static int FWDSPEED = 108;
	const static int REVSPEED = 75;
	const static int WAIT = 6;
	static int rightc, right;
	static int leftc, left;
	static int frontc, front;
	static int backing=0;

	if((rightc = getPing(pingPins[2])) != 0) right = rightc;
	if((leftc = getPing(pingPins[4])) != 0) left = leftc;
	delay(WAIT);
	if((frontc = getPing(pingPins[3])) != 0) front = frontc;
	delay(WAIT);

	if(backing !=0){
		int delta = millis()-backing;
		if(delta < 750) {
			drive.write(0);
			msg = "Stopping            ";
		}
		else if(delta < 1000){
			steer.write( (left > right) ? (CENTER+TURN) : (CENTER-TURN) );
			drive.write(95);
			msg = "Releasing brake     ";
		}
		else{
			if( front<5500 && steer.read()!=CENTER) {
				drive.write(REVSPEED);
				// steer.write( (left > right) ? (CENTER+TURN) : (CENTER-TURN) );
				backing = millis()-1000;
				msg = "aquiring clear path ";
			}
			else if(delta < 1800){
				drive.write(90);
				steer.write(CENTER);
				msg = "Stopping            ";
			}
			else {
				backing = 0;
			}
		}
	}
	else if (front < 3000 || left < 650 || right < 650){
		backing = millis();
	}
	else{
		cap(left, 4000); cap(right,4000);
		int steervalue = map(left-right, -4000, 4000, CENTER+TURN, CENTER-TURN);
		steer.write(steervalue);
		drive.write(FWDSPEED);
		msg = "Driving Forward     ";
	}
}
////////////////////////////////////////////////////
void inline writeToChars(byte in[][8]){
	for(int i=0; i< 8; i++) lcd.createChar(i, in[i]);
}

void setCustomChars(enum charSet set){
	if(currentCharSet != set){
		switch(set){
			case EYE:
				writeToChars(eye);
				break;
			case EYE2:
				writeToChars(eye2);
				break;
			case EYE3:
				writeToChars(eye3);
				break;
			case EYE4:
				writeToChars(eye4);
				break;
			case EYE5:
				writeToChars(eye5);
				break;
			case EYE6:
				writeToChars(eye6);
				break;
			case EYE7:
				writeToChars(eye7);
				break;
			case VERT:
				byte data[8];
				for(int i=0; i<8; i++) data[i] = 0;
				for(int row=0; row<8; row++){
					data[7-row] = 0xFF;
					lcd.createChar(row, data);
				}
				break;
			case HORZ:
				byte map[8];
				for(int i=0; i<8; i++) map[i] = 0;
				for(int clmn=0; clmn<5; clmn++){
					for(int i=0; i<8; i++) map[i] |= (0x10>>clmn);
					lcd.createChar(clmn, map);
				}
				lcd.createChar(6, degree);
				break;
		}
		currentCharSet = set;
	}
}

void verticalBarGraph(int top, int left, int bottom, int right, int value){
	for(int line=(bottom); line>=top; line--){
		lcd.setCursor(left,line);
		if(value > 8) {
			for(int i=0; i<=(right-left); i++) lcd.send(7,1);
			value -= 8;
		}
		else if(value != 0) {
			for(int i=0; i<=(right-left); i++) lcd.send((value%8 -1), 1);
			value = 0;
		}
		else {
			for(int i=0; i<=(right-left); i++) lcd.print(' ');
		}
	}
}

void horizontalBarGraph(int top, int left, int bottom, int right, int value){
	for(int colm=left; colm <= right; colm++){
		if(value > 5) {
			for(int i=top; i<=bottom; i++) {
				lcd.setCursor(colm, i);
				lcd.send(4, 1);
			}
			value -= 5;
		}
		else if(value !=0){
			for(int i=top; i<=bottom; i++) {
				lcd.setCursor(colm, i);
				lcd.send((value%5 -1), 1);
			}
			value = 0;
		}
		else {
			for(int i=top; i<=bottom; i++) {
				lcd.setCursor(colm, i);
				lcd.print(' ');
			}
		}

	}
}

void printInPlace(int places, int number){
	bool hit = false;
	long i=1;
	for(int j=1; j<places; j++) i*=10;
	if(number < 0) {
		number = abs(number);
		lcd.print('-');
		i/=10;
	}
	cap(number, (i*10)-1);
	for(i; i>0; i/=10){
		if( hit |= ((number/i)%10 != 0) )
			lcd.print( (number/i)%10 );
		else lcd.print(' ');
	}
}

void printInPlace(int places, float number){
	float i=1;
	for(int j=1; j<places; j++) {
		if( abs(number/i) < 1.) break;
		i*=10;
	}
	if(number < 0) {
		number = abs(number);
		lcd.print('-');
		places--;
	}
	cap(number, (i)-1);
	for(int j=0; j<places; j++){
		if(i==1) {
			i/=10;
			lcd.print('.');
		}
		else{
			i/=10;
			if(i<1) lcd.print( (int)(number/i) %10 );
			else lcd.print( ((int)(number/i)) %10 );
		}
	}
}

void printInPlace(int places, double number){
	printInPlace(places, (float) number);
}

void inline cap(int&   var, long  max){ if(var > max) var = max; }
void inline cap(float& var, float max){ if(var > max) var = max; }
