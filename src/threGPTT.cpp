#include <TinyGPSPlus.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <EEPROM.h>
#include <NewPing.h>
#include <Arduino.h>

#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define MOTOR1_PIN 18
#define MOTOR2_PIN 19
#define MOTOR3_PIN 21
#define MOTOR4_PIN 22
#define US_FRONT_TRIG_PIN 23
#define US_FRONT_ECHO_PIN 24
#define US_BACK_TRIG_PIN 25
#define US_BACK_ECHO_PIN 26
#define US_LEFT_TRIG_PIN 27
#define US_LEFT_ECHO_PIN 14
#define US_RIGHT_TRIG_PIN 12
#define US_BOTTOM_TRIG_PIN 32
#define US_BOTTOM_ECHO_PIN 33
#define US_RIGHT_ECHO_PIN 34

const double EARTH_RADIUS = 6371.0;
const double TARGET_ALTITUDE = 400.0;
const double LANDING_DISTANCE_THRESHOLD = 0.01;
const unsigned int MAX_DISTANCE = 200;

TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
Servo motor1, motor2, motor3, motor4;

NewPing us_front(US_FRONT_TRIG_PIN, US_FRONT_ECHO_PIN, MAX_DISTANCE);
NewPing us_back(US_BACK_TRIG_PIN, US_BACK_ECHO_PIN, MAX_DISTANCE);
NewPing us_left(US_LEFT_TRIG_PIN, US_LEFT_ECHO_PIN, MAX_DISTANCE);
NewPing us_right(US_RIGHT_TRIG_PIN, US_RIGHT_ECHO_PIN, MAX_DISTANCE);
NewPing us_bottom(US_BOTTOM_TRIG_PIN, US_BOTTOM_ECHO_PIN, MAX_DISTANCE);

double targetLatitude = 0.0;
double targetLongitude = 0.0;
bool targetSet = false;
bool isLanding = false;
double currentAltitude = TARGET_ALTITUDE;

void parseCoordinates(String input);
void navigateToTarget(double currentLat, double currentLon);
double calculateHeading(double currentLat, double currentLon, double targetLat, double targetLon);
double calculateDistance(double currentLat, double currentLon, double targetLat, double targetLon);
void controlMotors(double heading, int speed);
void stopMotors();
void landDrone();
void handleErrors(String errorMsg);
void saveTargetToEEPROM(double lat, double lon);
void loadTargetFromEEPROM();
void printGPSInfo(double lat, double lon);
void performSelfCheck();
void checkForObstacles();
void avoidObstacle(String direction);
float readTemperature();

void setup() {
    Serial.begin(115200);
    gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

    motor1.attach(MOTOR1_PIN);
    motor2.attach(MOTOR2_PIN);
    motor3.attach(MOTOR3_PIN);
    motor4.attach(MOTOR4_PIN);

    stopMotors();

    Serial.println("Enter coordinates that you want to go in format: lat,lon");
    loadTargetFromEEPROM();
    performSelfCheck();
}

void loop() {
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }

    if (gps.location.isUpdated()) {
        double currentLatitude = gps.location.lat();
        double currentLongitude = gps.location.lng();

        printGPSInfo(currentLatitude, currentLongitude);

        if (targetSet && !isLanding) {
            checkForObstacles();
            navigateToTarget(currentLatitude, currentLongitude);
        } else if (isLanding) {
            landDrone();
        }
    }

    float temperature = readTemperature();
    int digitalNumber = 123;

    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" C");

    Serial.print("Digital Number: ");
    Serial.println(digitalNumber);

    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        parseCoordinates(input);
    }

    delay(2000);
}

void parseCoordinates(String input) {
    int commaIndex = input.indexOf(',');
    if (commaIndex > 0) {
        targetLatitude = input.substring(0, commaIndex).toDouble();
        targetLongitude = input.substring(commaIndex + 1).toDouble();
        targetSet = true;
        saveTargetToEEPROM(targetLatitude, targetLongitude);
        Serial.print("New target set to: ");
        Serial.print("Lat: ");
        Serial.print(targetLatitude, 6);
        Serial.print(" Lon: ");
        Serial.println(targetLongitude, 6);
    } else {
        handleErrors("Invalid coordinate format.");
    }
}

void navigateToTarget(double currentLat, double currentLon) {
    double heading = calculateHeading(currentLat, currentLon, targetLatitude, targetLongitude);
    double distance = calculateDistance(currentLat, currentLon, targetLatitude, targetLongitude);

    if (distance > LANDING_DISTANCE_THRESHOLD) {
        int speed = map(distance, 0, 100, 1000, 2000);
        controlMotors(heading, speed);
    } else {
        isLanding = true;
    }
}

double calculateHeading(double currentLat, double currentLon, double targetLat, double targetLon) {
    double dLon = targetLon - currentLon;
    double y = sin(dLon) * cos(targetLat);
    double x = cos(currentLat) * sin(targetLat) - sin(currentLat) * cos(targetLat) * cos(dLon);
    return atan2(y, x) * 180 / PI;
}

double calculateDistance(double currentLat, double currentLon, double targetLat, double targetLon) {
    double dLat = radians(targetLat - currentLat);
    double dLon = radians(targetLon - currentLon);
    double a = sin(dLat / 2) * sin(dLat / 2) + cos(radians(currentLat)) * cos(radians(targetLat)) * sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return EARTH_RADIUS * c;
}

void controlMotors(double heading, int speed) {
    motor1.writeMicroseconds(speed);
    motor2.writeMicroseconds(speed);
    motor3.writeMicroseconds(speed);
    motor4.writeMicroseconds(speed);
}

void stopMotors() {
    motor1.writeMicroseconds(400);
    motor2.writeMicroseconds(400);
    motor3.writeMicroseconds(400);
    motor4.writeMicroseconds(400);
}

void landDrone() {
    Serial.println("Landing...");

    while (currentAltitude > 0) {
        Serial.print("Altitude: ");
        Serial.println(currentAltitude);
        int speed = map(currentAltitude, 0, TARGET_ALTITUDE, 1000, 1200);
        controlMotors(0, speed);
        delay(1000);
        currentAltitude -= 10;
    }

    stopMotors();
    Serial.println("Landing complete.");
    targetSet = false;
    isLanding = false;
}

void handleErrors(String errorMsg) {
    Serial.print("Error: ");
    Serial.println(errorMsg);
}

void saveTargetToEEPROM(double lat, double lon) {
    EEPROM.begin(512);
    EEPROM.put(0, lat);
    EEPROM.put(sizeof(double), lon);
    EEPROM.end();
}

void loadTargetFromEEPROM() {
    EEPROM.begin(512);
    EEPROM.get(0, targetLatitude);
    EEPROM.get(sizeof(double), targetLongitude);
    EEPROM.end();

    if (targetLatitude != 0.0 && targetLongitude != 0.0) {
        targetSet = true;
        Serial.print("Loaded target: ");
        Serial.print("Lat: ");
        Serial.print(targetLatitude, 6);
        Serial.print(" Lon: ");
        Serial.println(targetLongitude, 6);
    }
}

void printGPSInfo(double lat, double lon) {
    Serial.print("Lat: ");
    Serial.print(lat, 6);
    Serial.print(" Lon: ");
    Serial.println(lon, 6);
}

void performSelfCheck() {
    Serial.println("Performing self-check...");
    if (motor1.attached()) {
        Serial.println("Motor 1 attached.");
    } else {
        Serial.println("Motor 1 not attached.");
    }
    Serial.println("Self-check complete.");
}

void checkForObstacles() {
    int frontDistance = us_front.ping_cm();
    int backDistance = us_back.ping_cm();
    int leftDistance = us_left.ping_cm();
    int rightDistance = us_right.ping_cm();
    int bottomDistance = us_bottom.ping_cm();

    frontDistance = frontDistance % 50;
    backDistance = backDistance % 50;
    leftDistance = leftDistance % 50;
    rightDistance = rightDistance % 50;
    bottomDistance = bottomDistance % 1000;

    if (frontDistance < 20 && frontDistance > 0) {
        avoidObstacle("front");
    } else if (backDistance < 20 && backDistance > 0) {
        avoidObstacle("back");
    } else if (leftDistance < 20 && leftDistance > 0) {
        avoidObstacle("left");
    } else if (rightDistance < 20 && rightDistance > 0) {
        avoidObstacle("right");
    } else if (bottomDistance < 10 && bottomDistance > 0) {
        avoidObstacle("bottom");
    }
}

void avoidObstacle(String direction) {
    Serial.print("Avoiding obstacle: ");
    Serial.println(direction);

    if (direction == "front") {
        controlMotors(180, 1200);
    } else if (direction == "back") {
        controlMotors(0, 1200);
    } else if (direction == "left") {
        controlMotors(90, 1200);
    } else if (direction == "right") {
        controlMotors(270, 1200);
    } else if (direction == "bottom") {
        controlMotors(0, 1500);
    }

    delay(1000);
    stopMotors();
}