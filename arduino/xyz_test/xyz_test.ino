
#include "I2Cdev.h"						// I2C device library
#include "MPU6050_6Axis_MotionApps20.h"	// MPU6050 "driver" and 3D motion funtions
#include <PID_v1.h>						// PID Library

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#ifdef TESTING
    #define TEST_PRINT(x) Serial.print(x, y)
    #define TEST_PRINTF(x, y) Serial.print(x, y)
    #define TEST_PRINTLN(x) Serial.println(x)
    #define TEST_PRINTLNF(x, y) Serial.println(x, y)
#else
    #define TEST_PRINT(x)
    #define TEST_PRINTF(x, y)
    #define TEST_PRINTLN(x)
    #define TEST_PRINTLNF(x, y)
#endif

#define TESTING

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:
   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

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
float euler[3] = {0.0}; // [psi, theta, phi]    Euler angle container
float ypr[3] = {0.0};   // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high


//Define Variables we'll be connecting to
double setPID, inPID, outPID;

//Specify the links and initial tuning parameters
double Kp=2.0, Ki=5.0, Kd=1.0;
PID rollPID(&inPID, &outPID, &setPID, Kp, Ki, Kd, DIRECT);

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}

void setup()
{
	// join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties with TWBR.
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin()

    // initialize serial communication
    Serial.begin(38400);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    TEST_PRINTLN(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    TEST_PRINTLN(F("Testing device connections..."));
    TEST_PRINTLN(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    TEST_PRINTLN(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    TEST_PRINTLN(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    /* TODO: Figure out offsets for own device */
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for jrowberg's test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        TEST_PRINTLN(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        TEST_PRINTLN(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        TEST_PRINTLN(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        TEST_PRINT(F("DMP Initialization failed (code "));
        TEST_PRINT(devStatus);
        TEST_PRINTLN(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
	
    // Configure PID Controller
    setPID = 0;		// We want a zero roll angle, right?
    rollPID.SetMode(AUTOMATIC);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        TEST_PRINTLN(F("FIFO overflow!"));

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
    	TEST_PRINT("ypr\t");
	    TEST_PRINT(ypr[0] * 180/M_PI);
	    TEST_PRINT("\t");
	    TEST_PRINT(ypr[1] * 180/M_PI);
	    TEST_PRINT("\t");
	    TEST_PRINTLN(ypr[2] * 180/M_PI);

	    // For a single axis gimbal all that's needed is the roll angle
	    // (assuming that the orientation of the IMU is correct)
	    inPID = abs((ypr[2] * 255.0) / M_PI);
	    rollPID.Compute();
	    digitalWrite(motorDirPin, (ypr[2] > 0 ? HIGH : LOW));
	    analogWrite(motorSpeedPin, outPID);

	    TEST_PRINT("PID I,O,Dir\t");
	    TEST_PRINT(inPID);
	    TEST_PRINT("\t");
	    TEST_PRINT(outPID);
	    TEST_PRINT("\t");
	    (ypr[2] > 0 ? TEST_PRINTLN("L") : TEST_PRINTLN("R"));
        
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}
