#include <NewPing.h>
#include <Wire.h>
#include <MPU6050.h>

// Inisialisasi sensor MPU6050
MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;
float angleX, angleY, angleZ;

// Inisialisasi pin untuk driver L298N
#define MOTOR_RIGHT_A 2 //MOTOR KIRI
#define MOTOR_RIGHT_B 3
#define MOTOR_LEFT_A 4 //MOTOR KANAN
#define MOTOR_LEFT_B 5
#define MOTOR_MIDLE_A 6 //MOTOR TENGAH
#define MOTOR_MIDLE_B 7

// Inisialisasi sensor jarak
#define TRIGGER_PIN_FRONT 8 //SENSOR DEPAN
#define ECHO_PIN_FRONT 9
#define TRIGGER_PIN_RIGHT 10 //SENSOR KANAN
#define ECHO_PIN_RIGHT 11
#define TRIGGER_PIN_LEFT 12 //SENSOR KIRI
#define ECHO_PIN_LEFT 13
#define TRIGGER_PIN_UP 14 //SENSOR ATAS 
#define ECHO_PIN_UP 15
#define TRIGGER_PIN_DOWN 16 //SENSOR BAWAH
#define ECHO_PIN_DOWN 17

long ECHOTIME_FRONT; 
float DISTANCE_FRONT; //JARAK DEPAN
long ECHOTIME_RIGHT; 
float DISTANCE_RIGHT; //JARAK KANAN 
long ECHOTIME_LEFT; 
float DISTANCE_LEFT; //JARAK KIRI
long ECHOTIME_UP; 
float DISTANCE_UP; //JARAK ATAS
long ECHOTIME_DOWN; 
float DISTANCE_DOWN; //JARAK BAWAH

#define MAX_DISTANCE 50 //JARAK MAKS


void setup() {

  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  // Cek koneksi sensor MPU6050
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  //mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  //mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  //mpu.setDLPFMode(MPU6050_DLPF_BW_20);
  
  // Inisialisasi pin untuk driver L298N
  pinMode(MOTOR_LEFT_A, OUTPUT);
  pinMode(MOTOR_LEFT_B, OUTPUT);
  pinMode(MOTOR_RIGHT_A, OUTPUT);
  pinMode(MOTOR_RIGHT_B, OUTPUT);
  pinMode(MOTOR_MIDLE_A, OUTPUT);
  pinMode(MOTOR_MIDLE_B, OUTPUT);
  
  pinMode(TRIGGER_PIN_FRONT, OUTPUT); //SENSOR DEPAN
  pinMode(ECHO_PIN_FRONT, INPUT);
  digitalWrite(TRIGGER_PIN_FRONT, LOW);
  
  pinMode(TRIGGER_PIN_RIGHT, OUTPUT); //SENSOR KANAN
  pinMode(ECHO_PIN_RIGHT, INPUT);
  digitalWrite(TRIGGER_PIN_RIGHT, LOW);
  
  pinMode(TRIGGER_PIN_LEFT, OUTPUT); //SENSOR KIRI
  pinMode(ECHO_PIN_LEFT, INPUT);
  digitalWrite(TRIGGER_PIN_LEFT, LOW);

  pinMode(TRIGGER_PIN_UP, OUTPUT); //SENSOR ATAS
  pinMode(ECHO_PIN_UP, INPUT);
  digitalWrite(TRIGGER_PIN_UP, LOW);

  pinMode(TRIGGER_PIN_DOWN, OUTPUT); //SENSOR KIRI
  pinMode(ECHO_PIN_DOWN, INPUT);
  digitalWrite(TRIGGER_PIN_DOWN, LOW);
}


void loop() {
  
  //SENSOR DEPAN
  digitalWrite(TRIGGER_PIN_FRONT, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN_FRONT, LOW);
  ECHOTIME_FRONT = pulseIn(ECHO_PIN_FRONT, HIGH);
  DISTANCE_FRONT = 0.0001*((float)ECHOTIME_FRONT*1480)/2.0;
  Serial.println("Depan : " +String(DISTANCE_FRONT));

  //SENSOR KANAN
  digitalWrite(TRIGGER_PIN_RIGHT, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN_RIGHT, LOW);
  ECHOTIME_RIGHT = pulseIn(ECHO_PIN_RIGHT, HIGH);
  DISTANCE_RIGHT = 0.0001*((float)ECHOTIME_RIGHT*1480)/2.0;
  Serial.println("Kanan : " +String(DISTANCE_RIGHT));

  //SENSOR KIRI
  digitalWrite(TRIGGER_PIN_LEFT, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN_LEFT, LOW);
  ECHOTIME_LEFT = pulseIn(ECHO_PIN_LEFT, HIGH);
  DISTANCE_LEFT = 0.0001*((float)ECHOTIME_LEFT*1480)/2.0;
  Serial.println("Kiri : " +String(DISTANCE_LEFT));

  //SENSOR ATAS
  digitalWrite(TRIGGER_PIN_UP, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN_UP, LOW);
  ECHOTIME_UP = pulseIn(ECHO_PIN_UP, HIGH);
  DISTANCE_UP = 0.0001*((float)ECHOTIME_UP*1480)/2.0;
  Serial.println("Atas : "+String(DISTANCE_UP));

  //SENSOR BAWAH
  digitalWrite(TRIGGER_PIN_DOWN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN_DOWN, LOW);
  ECHOTIME_DOWN = pulseIn(ECHO_PIN_DOWN, HIGH);
  DISTANCE_DOWN = 0.0001*((float)ECHOTIME_DOWN*1480)/2.0;
  Serial.println("Bawah : "+String(DISTANCE_DOWN));

  
  if (DISTANCE_UP < MAX_DISTANCE) {
    stopMoving();
    delay(100);
    moveDown();
    delay(1000);
  }
  if (DISTANCE_DOWN < MAX_DISTANCE) {
    stopMoving();
    delay(100);
    moveUp();
    delay(1000);
  }
  if (DISTANCE_FRONT < MAX_DISTANCE) {
    stopMoving();
    delay(100);
    turnRight();
    delay(1000);
  }
  // jika ada objek di depan dan di kiri maka putar ke kanan
  if ((DISTANCE_FRONT < MAX_DISTANCE) && (DISTANCE_LEFT < MAX_DISTANCE)) {
    stopMoving();
    delay(100);
    turnRight();
    delay(1000);
  }
  // jika ada objek di depan dan di kanan dan putar ke kiri
  if ((DISTANCE_FRONT < MAX_DISTANCE) && (DISTANCE_RIGHT < MAX_DISTANCE)) {
    stopMoving();
    delay(100);
    turnLeft();
    delay(1000);
  }
  // jika ada objek di kiri, belok ke kanan
  else if (DISTANCE_LEFT < MAX_DISTANCE) {
    stopMoving();
    delay(100);
    turnRight();
    delay(1000);
  }
  // jika ada objek di kanan, belok ke kiri
  else if (DISTANCE_RIGHT < MAX_DISTANCE) {
    stopMoving();
    delay(100);
    turnLeft();
    delay(1000);
  }
  // jika tidak ada objek, maju
  else {
    moveForward();
  }


  // Baca nilai sensor MPU6050
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Konversi nilai sensor MPU6050 ke derajat
  angleX = atan2(ay, sqrt(ax * ax + az * az)) * 180 / PI;
  angleY = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;

  // Debugging nilai sensor MPU6050 
  Serial.print("Angle X: ");
  Serial.println(angleX);
  Serial.print("\tAngle Y: ");
  Serial.println(angleY);
  delay(100);
 
  // Jika robot miring ke kanan, gerakkan motor kanan maju dan motor kiri mundur
  if (angleX < -5) {
    digitalWrite(MOTOR_RIGHT_A, HIGH);
    digitalWrite(MOTOR_RIGHT_B, LOW);
    digitalWrite(MOTOR_LEFT_A, LOW);
    digitalWrite(MOTOR_LEFT_B, HIGH);
  } 
    // Jika robot miring ke kiri, gerakkan motor kiri maju dan motor kanan mundur
  else if (angleX > 5) {
    digitalWrite(MOTOR_RIGHT_A, LOW);
    digitalWrite(MOTOR_RIGHT_B, HIGH);
    digitalWrite(MOTOR_LEFT_A, HIGH);
    digitalWrite(MOTOR_LEFT_B, LOW);
  } 
  // Jika robot seimbang atau kemiringan sumbu X kurang dari 5 derajat, hentikan motor kiri-kanan
  else {
    digitalWrite(MOTOR_RIGHT_A, LOW);
    digitalWrite(MOTOR_RIGHT_B, LOW);
    digitalWrite(MOTOR_LEFT_A, LOW);
    digitalWrite(MOTOR_LEFT_B, LOW);
  }
  
  // Jika robot miring ke depan, gerakkan motor naik-turun
  if (angleY < -5) {
  digitalWrite(MOTOR_MIDLE_A, LOW);
  digitalWrite(MOTOR_MIDLE_B, HIGH);
  }
  // Jika robot miring ke belakang, gerakkan motor naik-turun ke atas
  else if (angleY > 5) {
  digitalWrite(MOTOR_MIDLE_A, HIGH);
  digitalWrite(MOTOR_MIDLE_B, LOW);
  }
  // Jika robot seimbang atau kemiringan sumbu Y kurang dari 5 derajat, hentikan motor naik-turun
  else {
  digitalWrite(MOTOR_MIDLE_A, LOW);
  digitalWrite(MOTOR_MIDLE_B, LOW);
  }
}

void moveForward() {
  digitalWrite(MOTOR_LEFT_A, HIGH);
  digitalWrite(MOTOR_LEFT_B, LOW);
  digitalWrite(MOTOR_RIGHT_A, LOW);
  digitalWrite(MOTOR_RIGHT_B, HIGH);
}

void turnRight() {
  digitalWrite(MOTOR_LEFT_A, HIGH);
  digitalWrite(MOTOR_LEFT_B, LOW);
  digitalWrite(MOTOR_RIGHT_A, LOW);
  digitalWrite(MOTOR_RIGHT_B, LOW);
}

void turnLeft() {
  digitalWrite(MOTOR_LEFT_A, LOW);
  digitalWrite(MOTOR_LEFT_B, LOW);
  digitalWrite(MOTOR_RIGHT_A, HIGH);
  digitalWrite(MOTOR_RIGHT_B, LOW);
}

void stopMoving() {
  digitalWrite(MOTOR_LEFT_A, LOW);
  digitalWrite(MOTOR_LEFT_B, LOW);
  digitalWrite(MOTOR_RIGHT_A, LOW);
  digitalWrite(MOTOR_RIGHT_B, LOW);
}

void moveUp() {
  digitalWrite(MOTOR_MIDLE_A, HIGH);
  digitalWrite(MOTOR_MIDLE_B, LOW);
}

void moveDown() {
  digitalWrite(MOTOR_MIDLE_A, LOW);
  digitalWrite(MOTOR_MIDLE_B, HIGH);
}
