#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>

void wifi();                                  //Untuk menghubungkan ke WiFi
void read_response(WiFiClient *client);       //Membaca perintah masukan
void kirim_koordinat(WiFiClient *client);     //Kirim koordinat melalui WiFi untuk ditampilkan pada UI
void kirim_info(WiFiClient *client, int sen0, int sen1, int sen2, int sen3, int sen4, int sen5, int ppx, int ppy, unsigned long time);  //Kirim berbagai info untuk ditampilkan pada UI

void baca_sensor_ir();                        //Membaca nilai sensor infrared
void baca_sensor_ultrasonic();                //Membaca nilai sensor ultrasonic
void kalibrasi();                             //Kalibrasi nilai sensor infrared untuk membedakan hitam dan putih

void bobot_sensor();                          //Bobot sensor infrared untuk nilai error/kemiringan robot untuk PID
void motor_maju();                            //Motor bergerak maju dengan PID
void motor_belok_kanan();                     //Motor berbelok ke arah kanan
void motor_belok_kiri();                      //Motor berbelok ke arah kiri
void motor_berputar();                        //Motor berputar balik dengan arah putaran menyesuaikan posisi robot
void motor_stop();                            //Motor berhenti
void motor_stop_delay();                      //Motor berhenti dengan delay
void go_to_koordinat();                       //Robot bergerak menuju koordinat selanjutnya dari posisi awal robot

void PSO();                                   //Fungsi PSO untuk menghasilkan koordinat yang akan dilewati robot
void bobot_awal_lintasan();                   //Memberi bobot 1 pada tiap garis lintasan
void deteksi_rintangan(WiFiClient *client);   //Deteksi rintangan, mengubah bobot lintasan, dan menjalankan PSO

//Variabel WiFi
IPAddress ip(192, 168, 0, 0);       //Sesuaikan dengan IP Address server (PC) (dapat berubah-ubah)
const char* HOST = "192.168.0.0";   //IP Address digunakan untuk komunikasi PC dan Mikrokontroller  
const uint16_t PORT = 1234;        

//Variabel motor driver & ultrasonic
#define IN1 18
#define IN2 5                         //Yang bikin motor jalan waktu booting
#define IN3 4
#define IN4 2
#define TRIGGER 26
#define ECHO 25
float jarak_cm;                       //Variabel jarak ultrasonic dalam satuan cm                  

//Variabel sensor IR
int pin_sensor[6] = {36, 39, 33, 32, 35, 34};
int nilai_sensor[6];                          //Nilai analog sensor IR
int nilai_ref[6];                             //Nilai referensi pembanding hitam dan putih
byte nilai_digital[6];                        //Konversi nilai analog ke nilai digital berdasarkan nilai referensi
int last_sensor_value_0, last_sensor_value_1; //Nilai digital sensor IR terakhir. Berguna untuk mendeteksi rising edge
bool ready_update_koordinat = false;          //Mendeteksi apakah koordinat dapat di update
int state_robot = 0;                          //Status robot: 0 = diam, 1 = jalan maju, 2 = berbelok

//Variabel PID
int bobot_posisi;                                 //Bobot sensor infrared untuk nilai error/kemiringan robot untuk PID
double error = 0, sum_error = 0, last_error = 0;  //Nilai error PID
double base_pwm = 128;                            //Kecepatan dasar PWM
double kp = 20, ki = 0.1, kd = 0;                 //Nilai kp, ki, dan kd untuk PID
double ts = 1;                                    //Nilai time settling

//Variabel Preferences
Preferences preferences;  //Variabel preferences untuk penyimpanan non-volatile
uint cek_pref;            //Cek isi preferences

//Variabel input QT
String input;             //Variabel raw input
String command;           //Variabel perintah
bool task_finish = true;  //Jika perintah telah diselesaikan, maka dapat menerima perintah baru

//Variabel waktu
unsigned long prev_time;
unsigned long waktu_tempuh = 0;     //Waktu tempuh robot dari awal hingga tujuan
unsigned long waktu_pso = 0;        //Durasi yang dibutuhkan PSO untuk memberi koordinat
unsigned long waktu;                //Waktu millis
bool update_waktu;                  //Jika 'true' maka variabel prev_time akan di update
unsigned long previous_connect = 0; //Non-block delay untuk mengkoneksikan ulang ke WiFi

//Variabel PSO
#define JUMLAH_KOORDINAT 7          //Banyaknya titik koordinat yang akan dibawa oleh tiap partikel
#define XY 2                        //Baris, Kolom
#define MAX_ITER 9999               //Iterasi maksimal
#define GRID_X_MAX 6                //Ukuran baris grid (vertikal)
#define GRID_Y_MAX 6                //Ukuran kolom grid (horizontal)
int jumlah_partikel = 0;            //Jumlah partikel yang akan di generate

uint posisi_awal[2] = {0, 0};       //Posisi awal robot
uint posisi_robot[2] = {0, 0};      //Posisi robot saat ini
uint posisi_tujuan[2] = {0, 0};     //Posisi yang akan dituju MAX:(6, 6)

int bobot_baris_lintasan[GRID_X_MAX][GRID_Y_MAX + 1];   //Bobot baris (X) lintasan untuk menentukan panjang lintasan
int bobot_kolom_lintasan[GRID_X_MAX + 1][GRID_Y_MAX];   //Bobot kolom (Y) lintasan untuk menentukan panjang lintasan

int path_plan[JUMLAH_KOORDINAT][XY];  //Koordinat terbaik yang akan dilewati robot
int total_belokan;                    //Total belokan koordinat terbaik PSO
int total_jarak;                      //Total jarak koordinat terbaik PSO
int direction = 0;                    //Arah robot: 0 = Utara, 90 = Timur, 180 = Selatan, 270 = Barat (simpan di preference)
int next_koordinat = 0;               //Agar robot mengikuti koordinat secara urut
int blockx;                           //Posisi rintangan pada sumbu X
int blocky;                           //Posisi rintangan pada sumbu Y
int state_block;                      //Orientasi rintangan: 0 = tidak ada, 1 = baris, dan 2 = kolom
int total_iterasi = 0;                //Jumlah total iterasi PSO

void setup()
{
  //Inisialisasi Serial
  Serial.begin(115200);

  //Inisialisasi pin IN1-IN4 (motor driver)
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  //Inisialisasi Wifi
  wifi();

  //Pemberian bobot lintasan
  bobot_awal_lintasan();

  // PSO();
  // while(true);

  //Inisialisasi dan cek isi preferences untuk nilai referensi sensor
  preferences.begin("ref_sensor", false); 
  cek_pref = preferences.getUInt("ref_1", 0);
  if(cek_pref != 0)
  {
    nilai_ref[0] = preferences.getUInt("ref_1");
    nilai_ref[1] = preferences.getUInt("ref_2");
    nilai_ref[2] = preferences.getUInt("ref_3");
    nilai_ref[3] = preferences.getUInt("ref_4");
    nilai_ref[4] = preferences.getUInt("ref_5");
    nilai_ref[5] = preferences.getUInt("ref_6");
  }

  //Inisialisasi pin trigger dan echo (ultrasonik)
  pinMode(TRIGGER, OUTPUT);
  pinMode(ECHO, INPUT);
}

void loop()
{
  //Class WiFiClient
  WiFiClient client;

  //Runtime ESP32
  waktu = millis();

  //Coba menghubungkan dan print "Connection failed" jika gagal
  if((waktu - previous_connect) > 3000)
  {
    if(!client.connect(HOST, PORT))
    {
      previous_connect = waktu;
      motor_stop();
      Serial.println("Connection failed, wait 3 sec...");
    }
  }

  //Baca sensor ir dan ultrasonic
  baca_sensor_ir();
  baca_sensor_ultrasonic();

  Serial.printf("%d\t%d\t%d\t%d\t%d\t%d", nilai_sensor[0], nilai_sensor[1],nilai_sensor[2],
                nilai_sensor[3], nilai_sensor[4], nilai_sensor[5]);
  Serial.printf("SONIC: %2f", jarak_cm);

  //Jalankan perintah jika perintah sebelumnya telah selesai
  if(task_finish)
  {
    read_response(&client);
    Serial.println(input);
    update_waktu = true;
    waktu_pso = 0;

    //Kalibrasi jika diberi command "cal"
    if(command.compareTo("cal") == 0)
    {
      kalibrasi();
      kirim_info(&client, nilai_ref[0], nilai_ref[1], nilai_ref[2], nilai_ref[3], nilai_ref[4], nilai_ref[5], 0, 0, 0);
    }

    //Simpan nilai referensi sensor pada preferences
    if(command.compareTo("sve") == 0)
    {
      preferences.clear();
      preferences.putUInt("ref_1", nilai_ref[0]);
      preferences.putUInt("ref_2", nilai_ref[1]);
      preferences.putUInt("ref_3", nilai_ref[2]);
      preferences.putUInt("ref_4", nilai_ref[3]);
      preferences.putUInt("ref_5", nilai_ref[4]);
      preferences.putUInt("ref_6", nilai_ref[5]);
      cek_pref = preferences.getUInt("ref_1", 0);
      kirim_info(&client, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    }

    //Jalankan PSO jika diberi command "pso"
    if(command.compareTo("pso") == 0) 
    {
      total_iterasi = 0;
      PSO();
      while(total_jarak > 100) PSO();
      kirim_koordinat(&client);
      task_finish = false;
    }
  }
  else
  {
    //Update prev_time jika perintah sebelumnya telah selesai
    if(update_waktu)
    {
      prev_time = millis();
      update_waktu = false;
    }
    waktu_tempuh = millis() - prev_time;

    //Cek rintangan setelah robot berbelok atau saat diam
    if(state_robot != 1)
    {
      deteksi_rintangan(&client);
    }

    //Cek jika robot telah sampai tujuan
    if(posisi_robot[0] == posisi_tujuan[0] && posisi_robot[1] == posisi_tujuan[1])
    {
      //Atur arah robot jika berada di koordinat awal (0,0)
      if(posisi_robot[0] == posisi_awal[0] && posisi_robot[1] == posisi_awal[1])
      {
        if(direction == 180) {motor_stop_delay(); motor_berputar();}
        else if(direction == 270) {motor_stop_delay(); motor_belok_kanan();}
        else if(direction == 90) {motor_stop_delay(); motor_belok_kiri();}
        direction = 0;
      }
      motor_stop_delay();
      task_finish = true;
      next_koordinat = 0;
      update_waktu = true;
      state_block = 0;
      bobot_awal_lintasan();
    }
    else
    {
      //Update posisi koordinat robot
      if(nilai_digital[0] != last_sensor_value_0 || nilai_digital[5] != last_sensor_value_1)
      {
        if(((nilai_digital[0] == 0 && nilai_digital[5] == 1) || (nilai_digital[0] == 1 && nilai_digital[5] == 0) || (nilai_digital[0] == 1 && nilai_digital[5] == 1)) && !ready_update_koordinat)
        {
          if(state_robot == 1)
          {
            ready_update_koordinat = true;
            if(direction == 0) posisi_robot[0] += 1;
            else if(direction == 180) posisi_robot[0] -= 1;
            else if(direction == 90) posisi_robot[1] +=1;
            else if(direction == 270) posisi_robot[1] -= 1;
          }
          deteksi_rintangan(&client);
        }
      }
      last_sensor_value_0 = nilai_digital[0]; last_sensor_value_1 = nilai_digital[5];
      if(nilai_digital[0] == 0 && nilai_digital[5] == 0) ready_update_koordinat = false;
      go_to_koordinat();
    }
    kirim_info(&client, nilai_digital[0], nilai_digital[1], nilai_digital[2], nilai_digital[3], nilai_digital[4], nilai_digital[5], path_plan[next_koordinat + 1][0], path_plan[next_koordinat + 1][1], waktu_tempuh);
  }
}

/// WiFi ///
void wifi()
{
  //Memulai WiFi ESP dan koneksikan ke SSID
  // WiFi.begin("Raditya", "Radit261");
  WiFi.begin("ID", "PASS");
  Serial.print("Connecting");

  //Print "...." jika WiFi belum terhubung
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  //Print IP address ESP
  Serial.println();
  Serial.print("Connected, IP address: ");
  Serial.println(WiFi.localIP());

  Serial.print("connecting to ");
  Serial.print(HOST);
  Serial.print(':');
  Serial.println(PORT);
}

void read_response(WiFiClient *client)
{
  unsigned long timeout = millis();
  while(client->available() == 0)
  {
    if(millis() - timeout > 5000)
    {
      Serial.println(">>> Client Timeout !");
      input = "";
      client->stop();
      return;
    }
  }

  // Read all the lines of the reply from server and print them to Serial
  while(client->available())
  {
    String line = client->readStringUntil('\n');
    String x;
    String y;
    String z;
    command = line.substring(0, 3);
    if(command.compareTo("pso") == 0)
    {
      int a = line.indexOf(";", 5);
      int b = line.indexOf(";", 7);
      x = line.substring(4, a);
      y = line.substring(a + 1);
      z = line.substring(b + 1);
      posisi_tujuan[0] = x.toInt();
      posisi_tujuan[1] = y.toInt();
      jumlah_partikel = z.toInt();
    }
    input = line;
  }
  Serial.printf("\nClosing connection\n\n");
}

void kirim_koordinat(WiFiClient *client)
{
  client->printf("Coord;%d,%d;%d,%d;%d,%d;%d,%d;%d,%d;%d,%d;%d,%d;%d;%d;%d,%d;%d;%lu;%d\n",
                    path_plan[0][0], 
                    path_plan[0][1], 
                    path_plan[1][0], 
                    path_plan[1][1], 
                    path_plan[2][0], 
                    path_plan[2][1],
                    path_plan[3][0],
                    path_plan[3][1],
                    path_plan[4][0],
                    path_plan[4][1],
                    path_plan[5][0],
                    path_plan[5][1],
                    path_plan[6][0],
                    path_plan[6][1],
                    total_belokan,
                    total_jarak,
                    blockx,
                    blocky,
                    state_block,
                    waktu_pso,
                    total_iterasi);
}

void kirim_info(WiFiClient *client, int sen0, int sen1, int sen2, int sen3, int sen4, int sen5, int ppx, int ppy, unsigned long time)
{
  client->printf("Info;%d,%d;%f;%d;%d;%d;%d;%d;%d;%d;%d,%d;%lu\n",
                        posisi_robot[0],
                        posisi_robot[1], 
                        jarak_cm, 
                        direction,
                        sen0,
                        sen1,
                        sen2,
                        sen3, 
                        sen4,
                        sen5,
                        ppx,
                        ppy,
                        time);
}
////////////

/// SENSOR & KALIBRASI ///
void baca_sensor_ir()
{
  for(int i = 0; i < 6; i++)
  {
    nilai_sensor[i] = analogRead(pin_sensor[i]);
    if(nilai_ref[i] == 0) continue;
    else if(cek_pref != 0)
    {
      if(nilai_sensor[i] > nilai_ref[i]) nilai_digital[i] = 0;
      else nilai_digital[i] = 1;
    }
  }
}

void baca_sensor_ultrasonic()
{
  //Pulse LOW -> HIGH
  digitalWrite(TRIGGER, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER, LOW);
  
  //Durasi waktu pulsa dikirim hingga diterima
  long durasi = pulseIn(ECHO, HIGH);
  
  //Hitung jarak berdasarkan waktu pulsa diterima dan kecepatan suara (343 m/s)
  jarak_cm = durasi * 0.034/2;
}

void kalibrasi()
{
  //Inisialisasi array nilai warna putih dan hitam
  int nilai_putih[] = {0, 0, 0, 0, 0, 0};
  int nilai_hitam[] = {0, 0, 0, 0, 0, 0};

  //Baca sensor 1x dan simpan nilai inisial putih dan hitam
  cek_pref = 0;
  baca_sensor_ir();
  for(int i = 0; i < 6; i++)
  {
    nilai_putih[i] = nilai_sensor[i];
    nilai_hitam[i] = nilai_putih[i]/2;
    nilai_ref[i] = 0;
  }

  //Loop untuk mencari nilai referensi
  while(true)
  {
    baca_sensor_ir();
    for(int j = 0; j < 6; j++)
    {
      if(nilai_sensor[j] < nilai_hitam[j])
      {
        nilai_ref[j] = (nilai_putih[j] + nilai_sensor[j])/2;
      }
    }
    if(nilai_ref[0] && nilai_ref[5] != 0) break;
  }
}
//////////////////////////

/// AKTUATOR & PID ///
void motor_maju()
{
  state_robot = 1;
  bobot_sensor();
  int set_point = 0;
  error = set_point - bobot_posisi;
  double delta_error = error - last_error;
  sum_error += last_error;
  double p = kp * error;
  double i = ki * sum_error * ts;
  double d = ((kd/ts) * delta_error);
  last_error = error;
  int output_pid = p + i + d;
  double motor_kanan = base_pwm - output_pid;
  double motor_kiri = base_pwm + output_pid;

  if(motor_kanan > 255) motor_kanan = 255;
  if(motor_kanan < 0) motor_kanan = 0;
  if(motor_kiri > 255) motor_kiri = 255;
  if(motor_kiri < 0) motor_kiri = 0;

  analogWrite(IN1, 0);
  analogWrite(IN2, motor_kanan);
  analogWrite(IN3, 0);
  analogWrite(IN4, motor_kiri);
}

void motor_belok_kanan()
{
  state_robot = 2;
  analogWrite(IN1, 128);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, 128);
  delay(400);
  baca_sensor_ir();
  while(nilai_digital[2] == 1 || nilai_digital[3] == 1)
  {
    delay(1); baca_sensor_ir();
  }
  baca_sensor_ir();
  while(nilai_digital[2] != 1)
  {
    delay(1); baca_sensor_ir();
  }
  motor_stop();
}

void motor_belok_kiri()
{
  state_robot = 2;
  analogWrite(IN1, 0);
  analogWrite(IN2, 128);
  analogWrite(IN3, 128);
  analogWrite(IN4, 0);
  delay(400);
  baca_sensor_ir();
  while(nilai_digital[2] == 1 || nilai_digital[3] == 1)
  {
    delay(1); baca_sensor_ir();
  }
  baca_sensor_ir();
  while(nilai_digital[3] != 1)
  {
    delay(1); baca_sensor_ir();
  }
  motor_stop();
}

void motor_berputar()
{
  if(posisi_robot[0] == 0 && posisi_robot[1] == 0)
  {
    if(direction == 0 || direction == 270)
    {
      motor_belok_kanan();
      motor_belok_kanan();
    }
    else if(direction == 90 || direction == 180)
    {
      motor_belok_kiri();
      motor_belok_kiri();
    }
  }
  else if(posisi_robot[0] == 0 && posisi_robot[1] == GRID_Y_MAX)
  {
    if(direction == 180 || direction == 270)
    {
      motor_belok_kanan();
      motor_belok_kanan();
    }
    else if(direction == 0 || direction == 90)
    {
      motor_belok_kiri();
      motor_belok_kiri();
    }
  }
  else if(posisi_robot[0] == GRID_X_MAX && posisi_robot[1] == 0)
  {
    if(direction == 0 || direction == 90)
    {
      motor_belok_kanan();
      motor_belok_kanan();
    }
    else if(direction == 180 || direction == 270)
    {
      motor_belok_kiri();
      motor_belok_kiri();
    }
  }
  else if(posisi_robot[0] == GRID_X_MAX && posisi_robot[1] == GRID_Y_MAX)
  {
    if(direction == 90 || direction == 180)
    {
      motor_belok_kanan();
      motor_belok_kanan();
    }
    else if(direction == 0 || direction == 270)
    {
      motor_belok_kiri();
      motor_belok_kiri();
    }
  }
  else if(posisi_robot[0] == 0 && posisi_robot[1] != 0)
  {
    if(direction == 180 || direction == 270)
    {
      motor_belok_kanan();
      motor_belok_kanan();
    }
    else if(direction == 0 || direction == 90)
    {
      motor_belok_kiri();
      motor_belok_kiri();
    }
  }
  else if(posisi_robot[0] == GRID_X_MAX && posisi_robot[1] != 0)
  {
    if(direction == 0 || direction == 90)
    {
      motor_belok_kanan();
      motor_belok_kanan();
    }
    else if(direction == 180 || direction == 270)
    {
      motor_belok_kiri();
      motor_belok_kiri();
    }
  }
  else if(posisi_robot[0] != 0 && posisi_robot[1] == 0)
  {
    if(direction == 0 || direction == 90)
    {
      motor_belok_kanan();
      motor_belok_kanan();
    }
    else if(direction == 180 || direction == 270)
    {
      motor_belok_kiri();
      motor_belok_kiri();
    }
  }
  else if(posisi_robot[0] != 0 && posisi_robot[1] == GRID_Y_MAX)
  {
    if(direction == 180 || direction == 270)
    {
      motor_belok_kanan();
      motor_belok_kanan();
    }
    else if(direction == 0 || direction == 90)
    {
      motor_belok_kiri();
      motor_belok_kiri();
    }
  }
  else
  {
    motor_belok_kanan();
    motor_belok_kanan();
  }
}

void motor_stop()
{
  state_robot = 0;
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, 0);  
}

void motor_stop_delay()
{
  state_robot = 0;
  delay(700);
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, 0);
}

void bobot_sensor()
{
  int nilai_sensor = (0 << 1) | (nilai_digital[1] << 1) | (nilai_digital[2] << 2) | (nilai_digital[3] << 3) | (nilai_digital[4] << 4) | (0 << 1);
  switch (nilai_sensor)
  {
    case 0b000010: bobot_posisi = 4; break;
    case 0b000110: bobot_posisi = 2; break;
    case 0b000100: bobot_posisi = 1; break;
    case 0b001110: bobot_posisi = 0; break;
    case 0b001100: bobot_posisi = 0; break;
    case 0b011110: bobot_posisi = 0; break;
    case 0b011100: bobot_posisi = 0; break;
    case 0b001000: bobot_posisi = -1; break;
    case 0b011000: bobot_posisi = -2; break;
    case 0b010000: bobot_posisi = -4; break;
  }
}
//////////////////////

/// PSO, DETEKSI RINTANGAN, & LOGIKA JALAN ROBOT ///
void PSO()
{
  //Inisialisasi variabel
  unsigned long prev_time_pso = millis();
  float partikel[jumlah_partikel][JUMLAH_KOORDINAT][XY];    //Partikel yang membawa koordinat saat ini
  float nilai_fitness[jumlah_partikel] = {0};               //Nilai fitness dari masing-masing partikel saat ini
  float pbest_fitness[jumlah_partikel] = {0};               //Nilai fitness dari pbest masing-masing partikel
  float posisi_best[jumlah_partikel][JUMLAH_KOORDINAT][XY]; //Posisi terbaik tiap partikel dari iterasi sebelumnya

  float global_best[JUMLAH_KOORDINAT][XY];                  //Koordinat terbaik dari seluruh iterasi
  float local_best[JUMLAH_KOORDINAT][XY];                   //Koodinat terbaik di iterasi saat ini

  int jumlah_belokan[jumlah_partikel];                      //Variabel jumlah belokan
  float jarak_lintasan[jumlah_partikel];                    //Variabel jarak lintasan

  float last_minimum, new_minimum;          //Nilai fitness minimum (terbaik) sebelumnya dan saat ini
  float alpha = 3, beta = 1;                //Bobot jarak (a) dan belokan (b) pada fitness function
  float c1 = 2.05, c2 = 2.05;               //Learning rates atau kemampuan kognitif
  float w, w_max = 0.9, w_min = 0.4;           //Rentang nilai inersia yang umum digunakan (sumber: buku Pak Budi)

  //Generate kecepatan awal secara random
  //Generate sejumlah partikel dengan masing-masing partikel membawa koordinat random
  float velocity[jumlah_partikel][JUMLAH_KOORDINAT][XY];    //Variabel kecepatan tiap partikel
  float vt_rand = random(0, 10);
  vt_rand = vt_rand/10;
  for(int i = 0; i < jumlah_partikel; i++)
  {
    for(int j = 0; j < JUMLAH_KOORDINAT; j++)
    {
      for(int k = 0; k < XY; k++)
      {
        velocity[i][j][k] = vt_rand;
        if(j == 0) partikel[i][j][k] = posisi_robot[k];
        else if(j == JUMLAH_KOORDINAT - 1) partikel[i][j][k] = posisi_tujuan[k];
        else
        {
          partikel[i][j][0] = random(GRID_X_MAX + 1);
          partikel[i][j][1] = random(GRID_Y_MAX + 1);
        }
      }
    }
  }

  //Iterasi hingga MAX_ITER atau stopping kriteria terpenuhi
  //Stopping kriteria: perbedaan nilai fitness tiap partikel < 1e-8 (tiap partikel konvergen)
  int iter = 0;
  while(true)
  {        
    if(iter > 1)
    {
      int y = 0;
      for(int i = 0; i < jumlah_partikel - 1; i++)
      {
        if(nilai_fitness[i] == nilai_fitness[i+1]) y++;   //Stopping criteria ambil standar deviasi
      }
      if(y == jumlah_partikel - 1 || iter > MAX_ITER) break;
    }

    float rand1 = random(0, 10), rand2 = random(0, 10);   //Nilai rand1 dan rand2 pada perhitungan velocity
    rand1 = rand1/10; rand2 = rand2/10;
    
    w = w_max - (((w_max - w_min)/MAX_ITER) * iter);  //Nilai inersia pada perhitungan velocity
    c1 = ((-3* iter)/MAX_ITER) + 3.5;
    c2 = ((3* iter)/MAX_ITER) + 0.5;

    //Hitung jumlah belokan dan jarak
    for(int i = 0; i < jumlah_partikel; i++)
    {
      jumlah_belokan[i] = 0;
      jarak_lintasan[i] = 0;
      int current_dir = direction;
      for(int j = 0; j < JUMLAH_KOORDINAT - 1; j++)
      {
        int belok = 0;
        int x0 = partikel[i][j][0];
        int y0 = partikel[i][j][1];
        int x1 = partikel[i][j+1][0];
        int y1 = partikel[i][j+1][1];
        
        if(x0 == x1 && y0 == y1) continue;
        else
        {
          //Cek belokan koordinat x
          if(current_dir == 0 && x0 != x1 && x0 > x1)
          {
            current_dir = 180;
            belok += 2;
          }
          else if(current_dir == 180 && x0 != x1 && x0 < x1)
          {
            current_dir = 0;
            belok += 2;
          }
          else if((current_dir == 90 || current_dir == 270) && x0 != x1 && x0 < x1)
          {
            current_dir = 0;
            belok += 1;
          }
          else if((current_dir == 90 || current_dir == 270) && x0 != x1 && x0 > x1)
          {
            current_dir = 180;
            belok += 1;
          }
          else belok += 0;

          //Cek belokan koordinat y
          if((current_dir == 0 || current_dir == 180) && y0 != y1 && y0 > y1)
          {
            current_dir = 270;
            belok += 1;
          }
          else if((current_dir == 0 || current_dir == 180) && y0 != y1 && y0 < y1)
          {
            current_dir = 90;
            belok += 1;
          }
          else if(current_dir == 90 && y0 != y1 && y0 > y1)
          {
            current_dir = 270;
            belok += 2;
          }
          else if(current_dir == 270 && y0 != y1 && y0 < y1)
          {
            current_dir = 90;
            belok += 2;
          }
          else belok += 0;
          jumlah_belokan[i] += belok;

          //Cek jarak koordinat x
          if(x0 < x1)
          {
            for(x0; x0 < x1; x0++)
            {
              jarak_lintasan[i] += bobot_baris_lintasan[x0][y0];
            }
          }
          else if(x0 > x1)
          {
            x0--;
            for(x0; x0 >= x1; x0--)
            {
              jarak_lintasan[i] += bobot_baris_lintasan[x0][y0];
            }
          }
          else jarak_lintasan[i] += 0;
          
          //Cek jarak koordinat y
          if(y0 < y1)
          {
            for(y0; y0 < y1; y0++)
            {
              jarak_lintasan[i] += bobot_kolom_lintasan[x1][y0];
            }
          }
          else if(y0 > y1)
          {
            y0--;
            for(y0; y0 >= y1; y0--)
            {
              jarak_lintasan[i] += bobot_kolom_lintasan[x1][y0];
            }
          }
          else jarak_lintasan[i] += 0;
        }
      }
    }

    //Hitung nilai fitness masing-masing partikel berdasarkan jarak dan jumlah belokan
    for(int i = 0; i < jumlah_partikel; i++)
    {
      nilai_fitness[i] = (alpha * jarak_lintasan[i]) + (beta * jumlah_belokan[i]); //Persamaan fitness function yang digunakan
    }

    //Ambil partikel dengan nilai fitness minimum
    new_minimum = nilai_fitness[0];
    for(int i = 0; i < jumlah_partikel; i++)
    {
      if (nilai_fitness[i] < new_minimum || i == 0)
      {
        new_minimum = nilai_fitness[i];
        for(int j = 0; j < JUMLAH_KOORDINAT; j++)
        {
          for(int k = 0; k < XY; k++)
          {
            local_best[j][k] = partikel[i][j][k];
          }
        }
      }
    }

    //Update Pbest berdasarkan fitness lama dan baru
    for(int i = 0; i < jumlah_partikel; i++)
    {
      for(int j = 0; j < JUMLAH_KOORDINAT; j++)
      {
        for(int k = 0; k < XY; k++)
        {
          if(iter == 0 || nilai_fitness[i] < pbest_fitness[i])
          {
            pbest_fitness[i] = nilai_fitness[i];
            posisi_best[i][j][k] = partikel[i][j][k];
          }
        }
      }
    }

    //Simpan nilai partikel dengan nilai fitness minimum sebagai gbest
    if(iter == 0 || new_minimum < last_minimum)         //NOTE: gbest bisa lebih dari 1
    { 
      last_minimum = new_minimum;
      for(int j = 0; j < JUMLAH_KOORDINAT; j++)
      {
        for(int k = 0; k < XY; k++)
        {
          global_best[j][k] = local_best[j][k];
        }
      }
    }

    //Tes print
    // Serial.printf("\n>>%d\n>>Gbest: ", iter);
    // Serial.println(last_minimum);
    // for(int i = 0; i < jumlah_partikel; i++)
    // {
    //   Serial.printf("%d\t", i);
    //   for(int j = 0; j < JUMLAH_KOORDINAT; j++)
    //   {
    //     Serial.print(abs(round(partikel[i][j][0])));
    //     Serial.print(",");
    //     Serial.print(abs(round(partikel[i][j][1])));
    //     Serial.print("|");
    //   }
    //   Serial.print('\t');
    //   Serial.print(nilai_fitness[i]); if(nilai_fitness[i] == new_minimum) Serial.print("--(Local Best)");
    //   Serial.printf("\tBelokan, jarak: %d, ", jumlah_belokan[i]);
    //   Serial.println(jarak_lintasan[i]);
    // }
    
    //Cari kecepatan dan update posisi masing-masing partikel
    bool cond = true;  
    for(int i = 0; i < jumlah_partikel; i++)
    {
      if(nilai_fitness[i] == last_minimum && cond)
      {
        cond = false;
        continue;
      }
      for(int j = 0; j < JUMLAH_KOORDINAT; j++)
      {
        if(j == 0 || j == JUMLAH_KOORDINAT - 1) continue;
        for(int k = 0; k < XY; k++)
        {
          //Cari kecepatan
          velocity[i][j][k] = ((w * velocity[i][j][k]) + (c1 * rand1 * (posisi_best[i][j][k] - partikel[i][j][k])) + (c2 * rand2 * (global_best[j][k] - partikel[i][j][k])));
          //Persamaan: vi(t+1) = (w*vit) + c1*rand1*(Pbest - xit) + c2*rand2*(Gbest - xit)
          
          //Update posisi dengan batas 0<x<=6 dan 0<y<=6
          partikel[i][j][k] = round(partikel[i][j][k] + velocity[i][j][k]); //Update posisi
          if(partikel[i][j][k] < 0) partikel[i][j][k] = 0;
          if(k == 0 && partikel[i][j][k] > GRID_X_MAX) partikel[i][j][k] = GRID_X_MAX;
          else if(k == 1 && partikel[i][j][k] > GRID_Y_MAX) partikel[i][j][k] = GRID_Y_MAX;
          
        }
      }
    }
    // Serial.println();
    // Serial.print(iter);
    // for(int i = 0; i < jumlah_partikel; i++)
    // {
    //   Serial.print(";");
    //   Serial.print(pbest_fitness[i]);
    // }
    iter++;
  }
  
  for(int i = 0; i < JUMLAH_KOORDINAT; i++)
  {
    for(int j = 0; j < XY; j++)
    {
      path_plan[i][j] = global_best[i][j];
      total_belokan = jumlah_belokan[0];
      total_jarak = jarak_lintasan[0];
    }
  }
  total_iterasi += iter;
  waktu_pso += millis() - prev_time_pso;
}

void deteksi_rintangan(WiFiClient *client)
{
  if(jarak_cm <= 30 && !(direction == 270 && posisi_robot[1] == 0) && !(direction == 90 && posisi_robot[1] == GRID_Y_MAX) && !(direction == 180 && posisi_robot[0] == 0) && !(direction == 0 && posisi_robot[0] == GRID_X_MAX))
  {
    if(state_robot != 2) motor_stop_delay();
    motor_stop();
    
    if(direction == 0)
    {
      bobot_baris_lintasan[posisi_robot[0]][posisi_robot[1]] = 100;
      blockx = posisi_robot[0];
      blocky = posisi_robot[1];
      state_block = 1;
    }
    else if(direction == 180)
    {
      bobot_baris_lintasan[posisi_robot[0] - 1][posisi_robot[1]] = 100;
      blockx = posisi_robot[0] - 1;
      blocky = posisi_robot[1];
      state_block = 1;
    }
    else if(direction == 90)
    {
      bobot_kolom_lintasan[posisi_robot[0]][posisi_robot[1]] = 100;
      blockx = posisi_robot[0];
      blocky = posisi_robot[1];
      state_block = 2;
    }
    else if(direction == 270)
    {
      bobot_kolom_lintasan[posisi_robot[0]][posisi_robot[1] - 1] = 100;
      blockx = posisi_robot[0];
      blocky = posisi_robot[1] - 1;
      state_block = 2;
    }

    int jarak_lintasan;
    bool block_path = false;
    for(int i = 0; i < JUMLAH_KOORDINAT - 1; i++)
    {
      int x0 = path_plan[i][0];
      int y0 = path_plan[i][1];
      int x1 = path_plan[i+1][0];
      int y1 = path_plan[i+1][1];
        
      if(x0 == x1 && y0 == y1) continue;
      else
      {
        //Cek jarak koordinat x
        if(x0 < x1)
        {
          for(x0; x0 < x1; x0++)
          {
            if(bobot_baris_lintasan[x0][y0] >= 100)
            {
              block_path = true;
              break;
            }
          }
        }
        else if(x0 > x1)
        {
          x0--;
          for(x0; x0 >= x1; x0--)
          {
            if(bobot_baris_lintasan[x0][y0] >= 100)
            {
              block_path = true;
              break;
            }
          }
        }
          
        //Cek jarak koordinat y
        if(!block_path)
        {
          if(y0 < y1)
          {
            for(y0; y0 < y1; y0++)
            {
              if(bobot_kolom_lintasan[x1][y0] >= 100) 
              {
                block_path = true;
                break;
              }
            }
          }
          else if(y0 > y1)
          {
            y0--;
            for(y0; y0 >= y1; y0--)
            {
              if(bobot_kolom_lintasan[x1][y0] >= 100)
              {
                block_path = true;
                break;
              }
            }
          }
        }
      }
      if(block_path) break;
    }
    
    if(block_path)
    {
      total_iterasi = 0;
      PSO();
      while(total_jarak > 100) PSO();
      next_koordinat = 0;
      state_robot = 0;
    }
    kirim_koordinat(client);
  }
  else return;
}

void go_to_koordinat()
{
  //Cek Posisi
  if(posisi_robot[0] != path_plan[next_koordinat + 1][0] && posisi_robot[0] < path_plan[next_koordinat + 1][0]) 
  {
    if(direction == 0) motor_maju();
    else
    {
      motor_stop_delay();
      if(direction == 180) motor_berputar();
      else if(direction == 90) motor_belok_kiri();
      else if(direction == 270) motor_belok_kanan();
      direction = 0;
    }
  }
  else if(posisi_robot[0] != path_plan[next_koordinat + 1][0] && posisi_robot[0] > path_plan[next_koordinat + 1][0]) 
  {
    if(direction == 180) motor_maju();
    else
    {
      motor_stop_delay();
      if(direction == 0) motor_berputar();
      else if(direction == 270) motor_belok_kiri();
      else if(direction == 90) motor_belok_kanan();
      direction = 180;
    }
  }
  else if(posisi_robot[1] != path_plan[next_koordinat + 1][1] && posisi_robot[1] < path_plan[next_koordinat + 1][1])
  {
    if(direction == 90) motor_maju();
    else
    {
      motor_stop_delay();
      if(direction == 270) motor_berputar();
      else if(direction == 180) motor_belok_kiri();
      else if(direction == 0) motor_belok_kanan();
      direction = 90;
    }
  }
  else if(posisi_robot[1] != path_plan[next_koordinat + 1][1] && posisi_robot[1] > path_plan[next_koordinat + 1][1])
  {
    if(direction == 270) motor_maju();
    else
    {
      motor_stop_delay();
      if(direction == 90) motor_berputar();
      else if(direction == 0) motor_belok_kiri();
      else if(direction == 180) motor_belok_kanan();
      direction = 270;
    }
  }
  else next_koordinat++;
}

void bobot_awal_lintasan()
{
  //Inisialisasi bobot baris lintasan
  for(int i = 0; i < GRID_X_MAX; i++)
  {
    for(int j = 0; j < GRID_Y_MAX + 1; j++)
    {
      bobot_baris_lintasan[i][j] = 1;
    }
  }

  //Inisialisasi bobot kolom lintasan
  for(int i = 0; i < GRID_X_MAX + 1; i++)
  {
    for(int j = 0; j < GRID_Y_MAX; j++)
    {
      bobot_kolom_lintasan[i][j] = 1;
    }
  }
}
////////////////////////////////////////////////////