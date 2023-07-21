/*
Compounents : 
  1. Ultrasonic Sensor
  2. Infrared Sensor
  3. UV Tron
  4. Flame Detector
  5. TCS 3200
  6. Arduino Mega
  7. Servo Motor
  8. Relay & Pump DC
*/


#include <NewPing.h>
#include <Servo.h>
#include <FlexiTimer2.h>

//mendifinisikan port 
Servo servo[4][3];
const int servo_pin[4][3] = { {2, 3, 4}, {5, 6, 7}, {8, 9, 10}, {11, 12, 13} };

/* ukuran kaki sudut ---------------------------------------------------------*/
const float length_a = 55; //55
const float length_b = 77.5; //77.5
const float length_c = 27.5; //27.5
const float length_side = 61;//61
const float z_absolute = -28;//-28

/* Constants untuk pergerakan -------------------------------------------------*/
const float z_default = -50, z_up = -40, z_boot = z_absolute;
const float x_default = 62, x_offset = 0;
const float y_start = 0, y_step = 40;

/* variables untuk gerakan ----------------------------------------------------*/
volatile float site_now[4][3];  //real-time koordinat untuk mengakhiri setiap kaki
volatile float site_expect[4][3];  //expected koodinat untuk mengakhiri setiap kaki
float temp_speed[4][3];  //kecepatan setiap axis, untuk recalculated sebelum bergerak
float move_speed;   //kecepatan gerak
float speed_multiple = 8; //multiple kecepatan gerak
const float spot_turn_speed = 20; //3
const float leg_move_speed = 20; //10
const float body_move_speed = 5; //3
const float stand_seat_speed = 2;
volatile int rest_counter; //+1/0.02s, otomatis istirahat

//parameter fungsi
const float KEEP = 255;

//perhitungan definisi PI
const float pi = 3.1415926;

/* inisialisasi konstata untuk bergiliran --------------------------------------------------------*/
//temp length
const float temp_a = sqrt(pow(2 * x_default + length_side, 2) + pow(y_step, 2));
const float temp_b = 2 * (y_start + y_step) + length_side;
const float temp_c = sqrt(pow(2 * x_default + length_side, 2) + pow(2 * y_start + y_step + length_side, 2));
const float temp_alpha = acos((pow(temp_a, 2) + pow(temp_b, 2) - pow(temp_c, 2)) / 2 / temp_a / temp_b);

//gerakan bergiliran
const float turn_x1 = (temp_a - length_side) / 2;
const float turn_y1 = y_start + y_step / 2;
const float turn_x0 = turn_x1 - temp_b * cos(temp_alpha);
const float turn_y0 = temp_b * sin(temp_alpha) - turn_y1 - length_side;

//ultrasonik depan 
#define trig1  22
#define echo1  23
#define MAX_DISTANCE1 200
NewPing sonarD (trig1,echo1,100);
//ultrasonik samping kanan
#define trig2  24
#define echo2  25
#define MAX_DISTANCE2 200
NewPing sonarR (trig2,echo2,100);
//ultrasonik samping kiri
#define trig3  26
#define echo3  27
#define MAX_DISTANCE3 200
NewPing sonarL (trig3,echo3,100);
//ultrasonik belakang kanan
#define trig4  28
#define echo4  29
#define MAX_DISTANCE4 200
NewPing sonarBR (trig4,echo4,100);
//ultrasonik depan belakang kiri
#define trig5  30
#define echo5  31
#define MAX_DISTANCE5 200
NewPing sonarBL (trig5,echo5,100); 

//definisi pin spray dan lainnya
//const int relay = 35;
int thresh = 20;
long randNumber;

//sensor IR
#define IR_D 32   
#define IR_B 33
float sensorValueD, inchesD, cmD;  
float sensorValueB, inchesB, cmB;  

//sensor TCS3200 
const int s0 = 16;
const int s1 = 17;
const int s2 = 18;
const int s3 = 19;
const int out = 20;

//Variables TCS3200
int red = 0;
int green = 0;
int blue = 0;

////sensor uvtron
int uvtron1 = 32;
int uvtron2 = 33;

// Inisialisasi pin servo
const int servoPin1 = 9;
const int servoPin2 = 10;

// Objek Servo
Servo servo1;
Servo servo2;

// Sudut servo saat lengan gripper terbuka dan tertutup
const int openAngle = 90;   // Sudut saat terbuka
const int closeAngle = 180; // Sudut saat tertutup


void setup (){

  // Menghubungkan objek Servo dengan pin servo
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  
  // Mengatur posisi awal lengan gripper terbuka
  servo1.write(openAngle);
  servo2.write(openAngle);
  

  //inisialisai lengan parameter di awal
  arm1.attach (14); // naik-turun
  arm2.attach (15); // capit
  arm1.write(90);
  delay(1000);

  pinMode(uvtron1, OUTPUT);
  pinMode(uvtron2, INPUT);

  //pinMode(relay, OUTPUT);
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(out, INPUT);
  
  digitalWrite(s0, HIGH);
  digitalWrite(s1, HIGH);
  
  Serial.begin(9600);
  Serial.println("Robot mulai inisialisasi");

  //inisialisasi kaki parameter di awal
  set_site(0, x_default - x_offset, y_start + y_step, z_boot);
  set_site(1, x_default - x_offset, y_start + y_step, z_boot);
  set_site(2, x_default + x_offset, y_start, z_boot);
  set_site(3, x_default + x_offset, y_start, z_boot);
 
  for (int i = 0; i < 4; i++){
    for (int j = 0; j < 3; j++){
      site_now[i][j] = site_expect[i][j]; }}
  
  // servo mulai service
  FlexiTimer2::set(20, servo_service);
  FlexiTimer2::start();
  Serial.println("Servo service started");
  
  // inisialisasi servo
  servo_attach();
  Serial.println("servo Terinialisasi");
  Serial.println("Robot Terinialisasi Baik");

  randomSeed(analogRead(0));
  
        Serial.println("Berdiri");
        berdiri();
        delay(1000);
}

void servo_attach(void){
  for (int i = 0; i < 4; i++){
    for (int j = 0; j < 3; j++){
      delay(50);
      servo[i][j].attach(servo_pin[i][j]);
      delay(50); }}}
      
void servo_detach(void){
  for (int i = 0; i < 4; i++){
    for (int j = 0; j < 3; j++){
      servo[i][j].detach();
      delay(50); }}}


void loop(){
  //sensor mengukur dan membaca jarak
  int uSD = sonarD.ping_cm();  //ultra depan
  Serial.println ("Front : " +String(uSD));
  int uSR = sonarR.ping_cm();  //ultra kanan
  Serial.println ("Right : " +String(uSR));
  int uSL = sonarL.ping_cm();  //ultra kiri
  Serial.println ("Left : " +String(uSL));
  int uSBR = sonarBR.ping_cm();  //ultra belakang kanan
  Serial.println ("Back-R : " +String(uSBR));
  int uSBL = sonarBL.ping_cm();  //ultra belakang kiri
  Serial.println ("Back-L : " +String(uSBL));

  int api1 = analogRead(A1);
  int api2 = analogRead(A2);
  int api3 = analogRead(A3);
  int api4 = analogRead(A4);
  int api5 = analogRead(A5);

  //int set_site();
  //deteksi korban
  if (red < blue && red < green && red < 25){
    if ((green - blue >= -5 && green - blue <= 5 && green - ( 2 * red ) <= 5 ) && (uSD>1 && uSD<5)){
      Serial.println("Orange Color");
      wait_all_reach();
      int mundur(unsigned int step);
      delay(50);
      int belok_kiri(unsigned int step);
      if ((green - blue >= -5 && green - blue <= 5 && green - ( 2 * red ) <= 5 ) && (uSD>1 && uSD<5)){
        Serial.println("Orange Color");
        moveServos(closeAngle); //menggerakkan lengan gripper ke bawah
        delay(1000); // Tunggu 1 detik
        openServos(); //membuka servo
        delay(2000); // Tunggu 2 detik
        closeServos(); //menutup servo
        delay(1000); // Tunggu 1 detik
        moveServos(openAngle); //menggerakkan lengan gripper ke atas
        delay(1000); // Tunggu 1 detik
        //tangkap();
        int belok_kanan(unsigned int step);
        delay(50);
        int maju(unsigned int step); }}} 

  //deteksi area putih, jingke (ngangkat)
  else if ((blue < red && blue < green && (blue && red && green) < 25) && (uSD>3 && uSD<5)){
    if (red - green <= 5 && red - green >= 0 && ((green - blue) || (red - blue)) < 5 && blue < 50){
      Serial.println("White Color");
      wait_all_reach();
      stand();
      wait_all_reach();}}

  //deteksi area biru, naikan speed
  else if (((red > green &&  blue < green) && blue < 25 && red > 40) && (uS1>3 && uS1<5)){
    Serial.println("Blue Color");
    wait_all_reach();
    unsigned int naik_speed();}

  //mencari safety zone
  if (red == 106 && green ==109 && blue ==122){
    Serial.println("gray color");
    wait_all_reach();
    int balik_kiri(unsigned int step);    
    moveServos(closeAngle); // Menggerakkan lengan gripper ke bawah
    delay(1000); // Tunggu 1 detik
    openServos(); // Membuka servo
    delay(2000); // Tunggu 2 detik
    //lepas();
    //delay(1000);
    }

  
  //api terdeteksi, eksekusi
  if ((api2>800 || api3>800 || api4>800) && (uvtron1, HIGH) && (uS1>5 && uS1<10)) { //ubah jarak penyemprotan
    digitalWrite(relay, HIGH);
    wait_all_reach(); 
    delay(1000);
    semprot(); 
    delay(2000);
    cek_kondisi();  
    delay(1000);
    }

  //api tidak terdeteksi
  else{
    digitalWrite(relay, LOW);
    delay(1000);}
  
  //deteksi warna selain arena (putih) maka mundur
  else if ((blue < red && blue < green && (blue && red && green) < 25) && (uS1>3 && uS1<5)){
    if (red - green <= 5 && red - green >= 0 && ((green - blue) || (red - blue)) < 5 && blue < 50){
      Serial.println("White Color");
      if (uS3 > 5){
        belok_kiri();}
      else if (uS4 > 5 ){
        belok_kanan();}
      else{
        balik_kiri();}
    }}
 
  //inisialiasi gerak robot
  if (uSBR < thresh && uSBR > 1){
    mundur(3);
    delay(500);}
  
  else if (uSL < 10 && uSL > 1){
    int belok_kanan(unsigned int step);
    //unsigned int belok_kanan();
    delay(100);}

  else if (uSR < 10 && uSR > 1){
    int belok_kiri(unsigned int step);
    delay(100);}

  else if(uSD < 10 && uSD > 1){
    if (uSL < uSR){
    int balik_kanan(unsigned int step);
    delay(800);}
 
  else if (uSR > uSL){
    int balik_kiri(unsigned int step);
    delay(800);}}

  randNumber = random(1,3);
  Serial.println(randNumber);
    if(randNumber == 1){
      belok_kanan(5);
      delay(200);}
            
  else if(randNumber == 2){
    belok_kiri(5);
    delay(200);}}    

  else {
    maju(1);}  
}




//inisialisasi program panggilan dan gerakan
void color()
{
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);
  //count OUT, pRed, RED
  red = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);
  digitalWrite(s3, HIGH);
  //count OUT, pBLUE, BLUE
  blue = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);
  digitalWrite(s2, HIGH);
  //count OUT, pGreen, GREEN
  green = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);
}

void semprot(){
  digitalWrite(relay, HIGH); 
  delay(5000);
  digitalWrite(relay, LOW); 
  delay(1000); 
}

void cek_kondisi(){
  if (uvtron1, HIGH){
  semprot();}
}

  //Fungsi untuk menggerakkan servo 1 dan servo 2
  void moveServos(int angle) {
    servo1.write(angle);
    servo2.write(angle);
  }
  
  //Fungsi untuk membuka servo 1 dan servo 2
  void openServos() {
    servo1.write(openAngle);
    servo2.write(openAngle);
  }
  
  //Fungsi untuk menutup servo 1 dan servo 2
  void closeServos() {
    servo1.write(closeAngle);
    servo2.write(closeAngle);
  }


void diam(){
  wait_all_reach();
  delay(1000);
}


void stand(){
  int stand_speed = 4; 
  move_speed = stand_speed;
  for (int leg = 0; leg < 4; leg++){
    set_site(leg, KEEP, KEEP, z_default);}
  wait_all_reach();
  unsigned int maju();
}


void naik_speed(unsigned int step){
  int leg_move_speed1 = 10;
  int body_move_speed1 = 6;
  move_speed = leg_move_speed1;
  while (step-- > 0){
    if (site_now[2][1] == y_start){
      //kaki 2 dan 1 gerak
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed1;
      set_site(0, x_default + x_offset, y_start, z_default);
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed1;
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_default);
      wait_all_reach();}
    
    else{
      //kaki 0&3 gerak
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed1;
      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed1;
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();
}}}


 
void berdiri (void){
  move_speed = stand_seat_speed;
  for (int leg = 0; leg < 4; leg++){
    set_site(leg, KEEP, KEEP, z_default);}
  wait_all_reach();
}


void belok_kiri(unsigned int step)
{
  move_speed = spot_turn_speed;
  while (step-- > 0)
  {
    if (site_now[3][1] == y_start)
    {
      //kaki 3 dn 1 gerak
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x1 - x_offset, turn_y1, z_default);
      set_site(1, turn_x0 - x_offset, turn_y0, z_default);
      set_site(2, turn_x1 + x_offset, turn_y1, z_default);
      set_site(3, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(3, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x1 + x_offset, turn_y1, z_default);
      set_site(1, turn_x0 + x_offset, turn_y0, z_default);
      set_site(2, turn_x1 - x_offset, turn_y1, z_default);
      set_site(3, turn_x0 - x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(1, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_default);
      set_site(1, x_default + x_offset, y_start, z_up);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      set_site(1, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //kaki 0 dan 2 gerak
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_up);
      set_site(1, turn_x1 + x_offset, turn_y1, z_default);
      set_site(2, turn_x0 - x_offset, turn_y0, z_default);
      set_site(3, turn_x1 - x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x0 - x_offset, turn_y0, z_default);
      set_site(1, turn_x1 - x_offset, turn_y1, z_default);
      set_site(2, turn_x0 + x_offset, turn_y0, z_default);
      set_site(3, turn_x1 + x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(2, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_up);
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();

      set_site(2, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}


void belok_kanan(unsigned int step)
{
  move_speed = spot_turn_speed;
  while (step-- > 0)
  {
    if (site_now[2][1] == y_start)
    {
      //kaki 2 dan 0 gerak
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x0 - x_offset, turn_y0, z_default);
      set_site(1, turn_x1 - x_offset, turn_y1, z_default);
      set_site(2, turn_x0 + x_offset, turn_y0, z_up);
      set_site(3, turn_x1 + x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(2, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_default);
      set_site(1, turn_x1 + x_offset, turn_y1, z_default);
      set_site(2, turn_x0 - x_offset, turn_y0, z_default);
      set_site(3, turn_x1 - x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_up);
      set_site(1, x_default + x_offset, y_start, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //kaki 1 dan 3 gerak
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x1 + x_offset, turn_y1, z_default);
      set_site(1, turn_x0 + x_offset, turn_y0, z_up);
      set_site(2, turn_x1 - x_offset, turn_y1, z_default);
      set_site(3, turn_x0 - x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(1, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x1 - x_offset, turn_y1, z_default);
      set_site(1, turn_x0 - x_offset, turn_y0, z_default);
      set_site(2, turn_x1 + x_offset, turn_y1, z_default);
      set_site(3, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(3, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

                                                                                                                                                            
void balik_kiri(){
 for(int p=0;p<2;p++){
  unsigned int belok_kiri();
}}


void balik_kanan() {
  for(int p=0;p<2;p++){
   unsigned int belok_kanan();
}}

    
void maju(unsigned int step)
{
  move_speed = leg_move_speed;
  while (step-- > 0)
  {
    if (site_now[2][1] == y_start)
    {
      //kaki 2 dan 1 gerak
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default + x_offset, y_start, z_default);
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //kaki 0&3 gerak
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}


void mundur(unsigned int step)
{
  move_speed = leg_move_speed;
  while (step-- > 0)
  {
    if (site_now[3][1] == y_start)
    {
      //kaki 3&0 gerak
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(1, x_default + x_offset, y_start, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //kaki 1&2 gerak
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}


void body_left(int i)
{
  set_site(0, site_now[0][0] + i, KEEP, KEEP);
  set_site(1, site_now[1][0] + i, KEEP, KEEP);
  set_site(2, site_now[2][0] - i, KEEP, KEEP);
  set_site(3, site_now[3][0] - i, KEEP, KEEP);
  wait_all_reach();
}



void body_right(int i)
{
  set_site(0, site_now[0][0] - i, KEEP, KEEP);
  set_site(1, site_now[1][0] - i, KEEP, KEEP);
  set_site(2, site_now[2][0] + i, KEEP, KEEP);
  set_site(3, site_now[3][0] + i, KEEP, KEEP);
  wait_all_reach();
}


void hand_wave(int i)
{
  float x_tmp;
  float y_tmp;
  float z_tmp;
  move_speed = 3;
  if (site_now[3][1] == y_start)
  {
    body_right(20);
    x_tmp = site_now[2][0];
    y_tmp = site_now[2][1];
    z_tmp = site_now[2][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(2, turn_x1, turn_y1, 50);
      wait_all_reach();
      set_site(2, turn_x0, turn_y0, 50);
      wait_all_reach();
    }
    set_site(2, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_left(15);
  }
  else
  {
    body_left(20);
    x_tmp = site_now[0][0];
    y_tmp = site_now[0][1];
    z_tmp = site_now[0][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(0, turn_x1, turn_y1, 50);
      wait_all_reach();
      set_site(0, turn_x0, turn_y0, 50);
      wait_all_reach();
    }
    set_site(0, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_right(15);
  }
}



void hand_shake(int i)
{
  float x_tmp;
  float y_tmp;
  float z_tmp;
  move_speed = 1;
  if (site_now[3][1] == y_start)
  {
    body_right(15);
    x_tmp = site_now[2][0];
    y_tmp = site_now[2][1];
    z_tmp = site_now[2][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(2, x_default - 30, y_start + 2 * y_step, 55);
      wait_all_reach();
      set_site(2, x_default - 30, y_start + 2 * y_step, 10);
      wait_all_reach();
    }
    set_site(2, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_left(15);
  }
  else
  {
    body_left(15);
    x_tmp = site_now[0][0];
    y_tmp = site_now[0][1];
    z_tmp = site_now[0][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(0, x_default - 30, y_start + 2 * y_step, 55);
      wait_all_reach();
      set_site(0, x_default - 30, y_start + 2 * y_step, 10);
      wait_all_reach();
    }
    set_site(0, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_right(15);
  }
}


void servo_service(void)
{
  sei();
  static float alpha, beta, gamma;
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      if (abs(site_now[i][j] - site_expect[i][j]) >= abs(temp_speed[i][j]))
        site_now[i][j] += temp_speed[i][j];
      else
        site_now[i][j] = site_expect[i][j];
    }

    cartesian_to_polar(alpha, beta, gamma, site_now[i][0], site_now[i][1], site_now[i][2]);
    polar_to_servo(i, alpha, beta, gamma);
  }

  rest_counter++;
}



void set_site(int leg, float x, float y, float z)
{
  float length_x = 0, length_y = 0, length_z = 0;

  if (x != KEEP)
    length_x = x - site_now[leg][0];
  if (y != KEEP)
    length_y = y - site_now[leg][1];
  if (z != KEEP)
    length_z = z - site_now[leg][2];

  float length = sqrt(pow(length_x, 2) + pow(length_y, 2) + pow(length_z, 2));

  temp_speed[leg][0] = length_x / length * move_speed * speed_multiple;
  temp_speed[leg][1] = length_y / length * move_speed * speed_multiple;
  temp_speed[leg][2] = length_z / length * move_speed * speed_multiple;

  if (x != KEEP)
    site_expect[leg][0] = x;
  if (y != KEEP)
    site_expect[leg][1] = y;
  if (z != KEEP)
    site_expect[leg][2] = z;
}


void wait_reach(int leg)
{
  while (1)
    if (site_now[leg][0] == site_expect[leg][0])
      if (site_now[leg][1] == site_expect[leg][1])
        if (site_now[leg][2] == site_expect[leg][2])
          break;
}



void wait_all_reach(void)
{
  for (int i = 0; i < 4; i++)
    wait_reach(i);
}

/*
  - transisi site dari cartesian ke polar
   ---------------------------------------------------------------------------*/
void cartesian_to_polar(volatile float &alpha, volatile float &beta, volatile float &gamma, volatile float x, volatile float y, volatile float z)
{
  //calculate w-z degree
  float v, w;
  w = (x >= 0 ? 1 : -1) * (sqrt(pow(x, 2) + pow(y, 2)));
  v = w - length_c;
  alpha = atan2(z, v) + acos((pow(length_a, 2) - pow(length_b, 2) + pow(v, 2) + pow(z, 2)) / 2 / length_a / sqrt(pow(v, 2) + pow(z, 2)));
  beta = acos((pow(length_a, 2) + pow(length_b, 2) - pow(v, 2) - pow(z, 2)) / 2 / length_a / length_b);
  //calculate x-y-z degree
  gamma = (w >= 0) ? atan2(y, x) : atan2(-y, -x);
  //trans degree pi->180
  alpha = alpha / pi * 180;
  beta = beta / pi * 180;
  gamma = gamma / pi * 180;
}

/*
  - transisi site dari polar ke microservos
   ---------------------------------------------------------------------------*/
void polar_to_servo(int leg, float alpha, float beta, float gamma)
{
  if (leg == 0)
  {
    alpha = 90 - alpha;
    beta = beta;
    gamma += 90;
  }
  else if (leg == 1)
  {
    alpha += 90;
    beta = 180 - beta;
    gamma = 90 - gamma;
  }
  else if (leg == 2)
  {
    alpha += 90;
    beta = 180 - beta;
    gamma = 90 - gamma;
  }
  else if (leg == 3)
  {
    alpha = 90 - alpha;
    beta = beta;
    gamma += 90;
  }
  servo[leg][0].write(alpha);
  servo[leg][1].write(beta);
  servo[leg][2].write(gamma);
}
