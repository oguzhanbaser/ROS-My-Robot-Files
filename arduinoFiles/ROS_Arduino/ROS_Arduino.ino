#include "Arduino.h"
/*
 Name:		robotReceiverArduino.ino
 Created:	9/3/2016 9:40:50 PM
 Author:	baser
 */

//#include <Adafruit_NeoPixel.h>

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

#include <EEPROM.h>

#elif defined(_SAM3XA_)

#include <DueFlashStorage.h>
DueFlashStorage dueFlashStorage;
#endif

#include <HardwareSerial.h>

#include "configuration.h"
#include "motor.h"

#ifdef USE_MPU6050
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;
#endif

HardwareSerial *mSerial;

double measureTemp(int pin, int numSamples = NUM_SAMPLES);
double measureVoltage(int pin, int numSamples = NUM_SAMPLES);
double measureLight(int pin, int numSamples = NUM_SAMPLES);
double measureCurrent(int pin, int numSamples = NUM_SAMPLES);

uint8_t blueVal = 0, redVal = 0, greenVal = 0;
uint8_t MAXSPEED = 170;

motor solMotor(MOTOR_LEFT_PWM, MOTOR_LEFT_DIR1, MOTOR_LEFT_DIR2);
motor sagMotor(MOTOR_RIGHT_PWM, MOTOR_RIGHT_DIR1, MOTOR_RIGHT_DIR2);

unsigned long lastTime = 0, lastTime2 = 0, lastTimeEncoder = 0;
unsigned long lastTimeRelay = 0, lastTimeImu = 0;

bool blinkStatusLed1 = false;
bool blinkStatusLed2 = true;
bool getImuData = false, getEncoderData = false, lDirection = FORWARD, rDirection = FORWARD;

bool manuelControl = true, ledStatus = true, guiControl = false;
bool redStatus = false, greenStatus = false, blueStatus = false;
int statServo = 0;
volatile long sagMotorEncoder = 0, solMotorEncoder = 0;
double mVelLeft = 0, mVelRight = 0;
uint8_t oldLDR = 100;
double ldrVal = 0.5;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

void changeMAXSpeed(int newVal) {
	if (newVal != MAXSPEED && newVal != 0) {
		MAXSPEED = newVal;
		writeEEPROM(MOTOR_ADRESS, MAXSPEED);
	}
}

void changeLDRVal(double newVal) {
	uint8_t ldr_eeprom = newVal * 255;
	if (ldr_eeprom != oldLDR) {
		oldLDR = ldr_eeprom;
		writeEEPROM(LDR_ADRESS, oldLDR);
	}
}

void pciSetup(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

volatile bool mpuInterrupt = false;
ISR (PCINT0_vect) // handle pin change interrupt for D8 to D13 here
{
	if(digitalRead(MPU6050_INT))
	{
		mpuInterrupt = true;
	}
}

void(* resetFunc) (void) = 0;

//rf alıcı
void dataRead(HardwareSerial *ser) {
	char c = ser->read();
	int val1, val2, newMAXSPEED;
	int valX, valY;
	String my_msg;

	switch (c) {

	//ilk karakter okunmu�sa parse ba�la
	case '#':
		manuelControl = true;

		val1 = ser->parseInt();
		val2 = ser->parseInt();
		valX = ser->parseInt();
		valY = ser->parseInt();
		statServo = ser->parseInt();

		//my_msg = String(val1) + " " + String(val2) + " " + String(valX) + " "+ String(valY) + " " + String(statServo);

		newMAXSPEED = ser->parseInt();

		if (newMAXSPEED != MAXSPEED) {
			changeMAXSpeed(newMAXSPEED);
		}

		hizAyarla(val1 - 512, val2 - 512);
		//servo switch i aktif de�ilken kumanda kontrol� aktif
		if (!statServo) {

			//servoHor.write(servo_msg.angle_x);
			//servoVer.write(servo_msg.angle_y);
			//delay(15);

			//servo set angle was here
			digitalWrite(ONBOARD_LED, LOW);
		}
		break;
	case '&':
		val1 = ser->parseInt();

		manuelControl = false;
		digitalWrite(ONBOARD_LED, HIGH);

		if (!manuelControl) {
			sagMotor.dur();
			solMotor.dur();
		}
		break;
	}

	//1 sn boyunca data okunmam��sa motoru durdur
	lastTime = millis();
}

void setup() {

#ifdef MAKE_TEST
	mSerial = &Serial;
	mSerial->begin(9600);
#else
	mSerial = &Serial3;

	mSerial->begin(115200);
	Serial.begin(115200);
#endif

#ifdef USE_MPU6050

	// join I2C bus (I2Cdev library doesn't do this automatically)
	#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
		Wire.begin();
		Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
	#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
		Fastwire::setup(400, true);
	#endif

	// initialize device
	mpu.initialize();
    pinMode(MPU6050_INT, INPUT);

    bool mpuConnStatus = mpu.testConnection() ? true : false;

    if(!mpuConnStatus)
    {
    	while(true)
    	{
    		Serial.println("MPU CONNECT ERROR");
    		mSerial->print("#|err|0");
    		digitalWrite(ONBOARD_LED, !digitalRead(ONBOARD_LED));
    		delay(1000);
    	}
    }else{
    	Serial.println("MPU CONNECTED");
    }

    // load and configure the DMP
    devStatus = mpu.dmpInitialize();

	// supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXGyroOffset(220);
	mpu.setYGyroOffset(76);
	mpu.setZGyroOffset(-85);
	mpu.setZAccelOffset(1788);

	// make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
    	Serial.println("Enabling DMP...");
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println("Enabling interrupt detection (Arduino external interrupt 0)...");
        pciSetup(MPU6050_INT);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println("DMP ready!");
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
    	Serial.println("DMP Initialization failed (code ");
    	Serial.println(devStatus);
    	Serial.println(")");
    	while(true)
		{
			Serial.println("MPU INIT ERROR");
    		mSerial->print("#|err|1");
			digitalWrite(ONBOARD_LED, !digitalRead(ONBOARD_LED));
			delay(1000);
		}
    }

#endif

	//ros 57600 baud rate ile ba�lar tekrar ekleme!!!
	Serial2.begin(9600);

	mSerial->flush();

	//Serial2.begin(9600);

	pinMode(STDNBY, OUTPUT);
	pinMode(LEDPIN, OUTPUT);
	pinMode(blinkLed1, OUTPUT);
	pinMode(blinkLed2, OUTPUT);
	//pixels.begin();

	//pciSetup(LEFT_ENCODER_PIN);
	//pciSetup(RIGHT_ENCODER_PIN);
	attachInterrupt(4, leftMotorEncoderInt, RISING);		//sol motor
	attachInterrupt(5, rightMotorEncoderInt, RISING);		//sag motor

	//EEPROM 0 MAXSPEED adresi
	MAXSPEED = readEEPROM(MOTOR_ADRESS);
	//writeEEPROM(LDR_ADRESS, 150);
	oldLDR = readEEPROM(LDR_ADRESS);

	ldrVal = (double) oldLDR / 255.0;

	digitalWrite(STDNBY, HIGH);

#if MAKE_MOTOR_TEST
	sagMotor.duzAyarla();
	solMotor.duzAyarla();

	test:

	sagMotor.hizAyarla(100);
	solMotor.hizAyarla(0);
	delay(2000);

	sagMotor.hizAyarla(0);
	solMotor.hizAyarla(100);
	delay(2000);

	goto test;
#endif

	sagMotor.dur();
	solMotor.dur();

	pinMode(ONBOARD_LED, OUTPUT);
	pinMode(PW_LED, OUTPUT);

	mSerial->print("#|start|\n");
}


void leftMotorEncoderInt() {
	solMotorEncoder += lDirection ? +1 : -1;
}

void rightMotorEncoderInt() {
	sagMotorEncoder += rDirection ? +1 : -1;
}


void loop() {

	if(mSerial->available() > 0)
	{
		char c = mSerial->read();


		if(c == '#')
		{

			String cmdName = mSerial->readStringUntil('|');

			//digitalWrite(ONBOARD_LED, !digitalRead(ONBOARD_LED));

			//first right motorSpeed
			if(cmdName == "spd")
			{
				digitalWrite(STDNBY, HIGH);

				int valR = mSerial->readStringUntil('|').toInt();
				int valL = mSerial->readStringUntil('|').toInt();

				Serial.print(valR);
				Serial.print(" ");
				Serial.println(valL);

				if(valR > MAXSPEED) valR = MAXSPEED;
				if(valL > MAXSPEED) valL = MAXSPEED;

				if(valR < -MAXSPEED) valR = -MAXSPEED;
				if(valL < -MAXSPEED) valL = -MAXSPEED;

				if (valR >= 0)
				{
				  rDirection = FORWARD;
				  sagMotor.duzAyarla();
				  sagMotor.hizAyarla(valR);
				} else {
				  rDirection = BACKWARD;
				  sagMotor.geriAyarla();
				  sagMotor.hizAyarla(-valR);
				}

				if (valL >= 0)
				{
				  lDirection = FORWARD;
				  solMotor.duzAyarla();
				  solMotor.hizAyarla(valL);
				} else {
				  lDirection = BACKWARD;
				  solMotor.geriAyarla();
				  solMotor.hizAyarla(-valL);
				}

				lastTime = millis();
			}else if(cmdName == "pwLed")
			{
				int val = mSerial->readStringUntil('|').toInt();
				digitalWrite(PW_LED, val);
			}else if(cmdName == "maxSpd")
			{
				int val = mSerial->readStringUntil('|').toInt();
				changeMAXSpeed(val);
			}else if(cmdName == "imu")
			{
				int val = mSerial->readStringUntil('|').toInt();
				getImuData = val;
			}else if(cmdName == "enc")
			{
				int val = mSerial->readStringUntil('|').toInt();
				getEncoderData = val;
			}
			else if(cmdName == "res")
			{
				resetFunc();
			}else if(cmdName == "man")
			{
				int val = mSerial->readStringUntil('|').toInt();
				manuelControl = val;
			}
		}
	}

	//sensör değerleri
	double tempature[2], voltage[1], light[1], current[1];

	tempature[0] = measureTemp(T_SENSOR1_PIN);
	tempature[1] = measureTemp(T_SENSOR2_PIN);

	voltage[0] = measureVoltage(V_SENSOR_PIN);

	current[0] = measureCurrent(C_SENSOR_PIN, C_SENSOR_SAMPLE);

#ifdef USE_LDR
	light[0] = measureLight(LDR_PIN);

	if ((light[0] < ldrVal) && !manuelControl) {
		digitalWrite(PW_LED, HIGH);
		lastTimeRelay = millis();
	} else if (!manuelControl) {
		if ((millis() - lastTimeRelay > PW_LED_WAIT)) {
			digitalWrite(PW_LED, LOW);
		}
	}
#endif

	String tempString = "#|" + String(tempature[0], DECIMAL_POINTS) + "|"
			+ String(tempature[1], DECIMAL_POINTS) + "|"
			+ String(voltage[0], DECIMAL_POINTS) + "|"
			+ String(current[0], DECIMAL_POINTS) + "|"
			+ String()
#ifdef USE_LDR
			//+ String(ldrVal, DECIMAL_POINTS) + "|"
			+ String(light[0], DECIMAL_POINTS) + "|"
#endif
			+ "\n";

	if(millis() - lastTime2 > 1000)
	{
		mSerial->print(tempString);

		lastTime2 = millis();
	}

	//kumanda haberleşmesi
	if (Serial2.available()) {
		dataRead(&Serial2);
	}

	//manuel vs otomatik kontrol geçiş
	if (manuelControl) {
		//autoControl = false;
		digitalWrite(LEDPIN, LOW);
		digitalWrite(PW_LED, LOW);
		if (ledStatus) {
			//Serial.println("Manuel Control");
			//animateLed(2);
			ledStatus = false;
			//animateLed(1);
		}
	} else {
		//servo default cordinates
		//...
		digitalWrite(LEDPIN, HIGH);
	}

	//1 sn boyunca data okunmam��sa motoru durdur
	//millis ilgili yerlerde kaydedilmi�
	if (millis() - lastTime > 1500) {
		sagMotor.dur();
		solMotor.dur();
		digitalWrite(PW_LED, LOW);
	}

	/*
	 if (sagMotorEncoder >= FULLTOUR)
	 sagMotorEncoder = 0;

	 if (solMotorEncoder >= FULLTOUR)
	 solMotorEncoder = 0;
	 */

	if ((millis() - lastTimeEncoder > ENCODERINTERVAL) && getEncoderData) {
		/*
		double leftDistance = 0, rightDistance = 0;
		if (solMotorEncoder != 0)
			leftDistance = PERIMETER * solMotorEncoder;

		if (sagMotorEncoder != 0)
			rightDistance = PERIMETER * sagMotorEncoder;

		mVelLeft = leftDistance / ((millis() - lastTimeEncoder) * (0.1));
		mVelRight = rightDistance / ((millis() - lastTimeEncoder) * (0.1));

		sagMotorEncoder = 0;
		solMotorEncoder = 0;
		lastTimeEncoder = millis();*/

		String sendEncoder = "e|" + String(solMotorEncoder) + "|" + String(sagMotorEncoder) + "|\n";
		mSerial->print(sendEncoder);
		//Serial.print(sendEncoder);

		//debugPublisher("Speed: " + String(mVelLeft) + " " + String(mVelRight));
	}

	if(false)
		readMPU();

}

void hizAyarla(int val1, int val2) {
	val1 = map(val1, -512, 512, -MAXSPEED, MAXSPEED);
	val2 = map(val2, 512, -512, -MAXSPEED, MAXSPEED);

	if (val1 > MAXSPEED)
		val1 = MAXSPEED;
	if (val1 < -MAXSPEED)
		val1 = -MAXSPEED;

	if (val2 > MAXSPEED)
		val2 = MAXSPEED;
	if (val2 < -MAXSPEED)
		val2 = -MAXSPEED;

	//h�z de�eri 70 den b�y�kken hareket ettir
	if (val2 < 70 && val2 > -70) {
		val2 = 0;
		digitalWrite(STDNBY, LOW);
		//sagMotor.dur();
		//solMotor.dur();
	} else {
		digitalWrite(STDNBY, HIGH);
	}

	//g�r�l�t�y� s�f�rla
	if (val1 < 10 && val1 > -10)
		val1 = 0;

#if MOTOR_LEFT_RIGHT_INV
	val1 = -val1;
#endif

	if (val2 < 0) {
		val2 = -val2;
		lDirection = FORWARD;
		rDirection = FORWARD;
		if (val1 > 0) {
			sagMotor.duzAyarla();
			solMotor.duzAyarla();
			sagMotor.hizAyarla((val2 - val1) > 0 ? (val2 - val1) : 0);
			solMotor.hizAyarla(val2 > 0 ? val2 : 0);
			//analogWrite(sol, val2);
			//analogWrite(sag, val2 - val1);
		} else {
			sagMotor.duzAyarla();
			solMotor.duzAyarla();
			sagMotor.hizAyarla(val2 > 0 ? val2 : 0);
			solMotor.hizAyarla((val2 + val1) > 0 ? (val2 + val1) : 0);
			//analogWrite(sag, val2);
			//analogWrite(sol, val2 + val1);
		}
	} else {
		lDirection = BACKWARD;
		rDirection = BACKWARD;
		if (val1 > 0) {
			sagMotor.geriAyarla();
			solMotor.geriAyarla();
			sagMotor.hizAyarla((val2 - val1) > 0 ? (val2 - val1) : 0);
			solMotor.hizAyarla(val2 > 0 ? val2 : 0);
			//analogWrite(sol, val2);
			//analogWrite(sag, val2 - val1);
		} else {
			sagMotor.geriAyarla();
			solMotor.geriAyarla();
			sagMotor.hizAyarla(val2 > 0 ? val2 : 0);
			solMotor.hizAyarla((val2 + val1) > 0 ? (val2 + val1) : 0);
			//analogWrite(sag, val2);
			//analogWrite(sol, val2 + val1);
		}
	}
}

double measureLight(int pin, int numSamples) {
	int sum = 0;
	for (int i = 0; i < numSamples; i++) {
		sum += analogRead(pin);
	}

	sum /= 3.0;
	return sum / 1023.0;
}

double measureCurrent(int pin, int numSamples) {
	int sum = 0;
	for (int i = 0; i < numSamples; i++) {
		sum += analogRead(pin);
	}

	double RawValue = sum / (double) numSamples;
	double Voltage = (RawValue / 1024.0) * 5000;
	double Amps = ((Voltage - ACSOFFSEET) / MVPERAMP);

	return Amps;
}

double measureTemp(int pin, int numSamples) {
	double sum = 0;
	for (int i = 0; i < numSamples; i++) {
		sum += analogRead(pin);
	}
	return (sum / (double) numSamples) * 0.48828125;
}

double measureVoltage(int pin, int numSamples) {
	unsigned long resSum = RES1 + RES2;
	double t_current = 12.0 / (double) resSum;

	double ref_voltage = RES2 * t_current;

	double sum = 0;
	for (int i = 0; i < numSamples; i++) {
		sum += analogRead(pin);
	}

	sum /= (double) numSamples;
	double m_voltage = sum * 5.0 / 1023.0;

	return (m_voltage * 12.0) / ref_voltage;
	//return (sum / 3.0) * 12.54 / 460.0;
}

void controlMotors(int hiz) {
	//int hiz = cmd_msg.speed;

	/*
	 if (cmd_msg.maxSpeed != MAXSPEED)
	 {
	 changeMAXSpeed(cmd_msg.maxSpeed);
	 return;
	 }*/

	if (hiz > MAXSPEED)
		hiz = MAXSPEED;
	if (hiz < -MAXSPEED)
		hiz = -MAXSPEED;

	//autoControl for opencv
	if (!manuelControl) {
		digitalWrite(ONBOARD_LED, HIGH);
		digitalWrite(STDNBY, HIGH);
		sagMotor.duzAyarla();
		solMotor.duzAyarla();

		if (hiz < 0) {
			sagMotor.hizAyarla(MAXSPEED + hiz);
			solMotor.hizAyarla(MAXSPEED);
		} else {
			solMotor.hizAyarla(MAXSPEED - hiz);
			sagMotor.hizAyarla(MAXSPEED);
		}
	} else {
		digitalWrite(ONBOARD_LED, LOW);
	}

	//1 sn boyunca data okunmam��sa motoru durdur
	lastTime = millis();
}

void writeEEPROM(uint16_t p_addr, uint8_t p_val) {
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
	EEPROM.write(p_addr, p_val);
#elif defined(_SAM3XA_)
	dueFlashStorage.write(p_addr, p_val);
#endif
}

uint8_t readEEPROM(uint16_t p_addr) {
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
	return EEPROM.read(p_addr);
#elif defined(_SAM3XA_)
	return dueFlashStorage.read(p_addr);
#endif
}

void readMPU()
{
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

		// display Euler angles in degrees
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
#ifdef MPU6050_YPR

		double yprF1 = ypr[0] * 180/M_PI;
		double yprF2 = ypr[1] * 180/M_PI;
		double yprF3 = ypr[2] * 180/M_PI;

	    char str_t1[10], str_t2[10], str_t3[10];

	    dtostrf(yprF1, 4, 4, str_t1);
	    dtostrf(yprF2, 4, 4, str_t2);
	    dtostrf(yprF3, 4, 4, str_t3);


	    String sendMpuStr = "$|" + String(str_t1) + "|"
	    						 + String(str_t2) + "|"
								 + String(str_t3) + "|"
								 + "\n\r";
#endif

#ifdef MPU6050_Q

	    double qx = q.x;
		double qy = q.y;
		double qz = q.z;
		double qw = q.w;

		char str_t1[10], str_t2[10], str_t3[10], str_t4[10];

		dtostrf(qx, 4, 4, str_t1);
		dtostrf(qy, 4, 4, str_t2);
		dtostrf(qz, 4, 4, str_t3);
		dtostrf(qw, 4, 4, str_t4);


		String sendMpuStr = "$|" + String(str_t1) + "|"
								 + String(str_t2) + "|"
								 + String(str_t3) + "|"
								 + String(str_t4) + "|"
								 + "\n\r";
#endif

	    //Serial.print(sendMpuStr);

	    if(millis() - lastTimeImu > MPUINTERVAL)
	    {
		    mSerial->println(sendMpuStr);
		    lastTimeImu = millis();
	    }

		/*
		Serial.print("ypr\t");
		Serial.print(ypr[0] * 180/M_PI);
		Serial.print("\t");
		Serial.print(ypr[1] * 180/M_PI);
		Serial.print("\t");
		Serial.println(ypr[2] * 180/M_PI);*/

    }
}

