//Regulator lotu - Grzegorz Sajdok

#include <Wire.h>   //Biblioteka Arduino do komunikacji typu I2C

//Definicje adresów czujników
#define L3G4200 0x69          //Adres czujnika L3G4200
#define GYRO_CTRL_REG1 0x20   //Adres rejestru 1 - odpowiedzialny za stan czujnika
#define GYRO_CTRL_REG4 0x23   //Adres rejestru 4 - odpowiedzalny za skale pomiarow i funkcje Block Data Update

#define ADXL345 0x53          //Adres czujnika ADXL345
#define ACC_POWER_CTL 0x2D    //Adres rejestru POWER CONTROL - odpowiedzialny za stan czujnika
#define ACC_DATA_FORMAT 0x31  //Adres rejestru DATA_FORMAT - odpowiedzialny za rozdzielczosc pomiarow

//Definicje ograniczen joystickow
#define STICK_HIGH 2000       //Maksymalna wartosc odchylenia sie stickow
#define STICK_HIGH_LIMIT 1950 //Od kiedy maksymalna wartosc bedzie obowiazywac
#define STICK_LOW 1000        //Minimalna wartosc odchylenia sie stickow
#define STICK_LOW_LIMIT 1050  //Od kiedy minimalna wartosc bedzie obowiazywac

//Definicja ograniczenia kanalu throttle w kalkulacjach ESC
#define MAX_THROTTLE 1800
#define MINIMUM_THROTTLE 1100

//Definicje wartosci konwersji
#define GYRO_RESOLUTION 65.5            //Jaka wartosc sygnalu odpowiada wartosci 1-go stopnia na sekunde - dla 500 dps, 16 bit
#define RAW_ANGLE_CONVERSION 0.0000763  //Wartosc rowna 1 / (GYRO_RESOLUTION * Czestotliwosc petli drona), dla czestotliwosci rownej 200Hz.
#define R2D 0.0000001331 //Poniewaz funkcje trygonometryczne arduino sa w radianach, potrzebna jest ta stala w celu konwersji. Rowna RAW_ANGLE_CONVERSION * (3.14/180)

//Definicja sily auto-level
#define AUTO_LEVEL_STRENGTH 15

//Definicja deadband (w jedna strone) w mikrosekundach
#define DEADBAND 10

//Definicja dzielnika, ktory wplywa na maksymalna wartosc predkosci katowych do ktorych bedzie dazyc dron
#define AIM_ANGLE_DIVISOR 3.0

//Zmienne regulatorow PID
//Zmienne dla rownan pid dla kata typu roll
float pid_gain_pRoll = 1.2;   //Wzmocnienie czlonu proporcjonalnego
float pid_gain_iRoll = 0.02;  //Wzmocnienie czlonu calkujacego
float pid_gain_dRoll = 15.0;  //Wzmocnienie czlonu rozniczkujacego
float pid_maxDRoll = 200;     //Ograniczenie maksymalnej wartosci czlonu calkujacego
float pid_maxRoll = 200;      //Ograniczenie maksymalnej wartosci na wyjsciu regulatora

//Zmienne dla rownan pid dla kata typu pitch
float pid_gain_pPitch = 1.2;  //Wzmocnienie czlonu proporcjonalnego
float pid_gain_iPitch = 0.02; //Wzmocnienie czlonu calkujacego
float pid_gain_dPitch = 15.0; //Wzmocnienie czlonu rozniczkujacego
float pid_maxDPitch = 200;    //Ograniczenie maksymalnej wartosci czlonu calkujacego
float pid_maxPitch = 200;     //Ograniczenie maksymalnej wartosci na wyjsciu regulatora

//Zmienne dla rownan pid dla kata typu yaw
float pid_gain_pYaw = 3.0;    //Wzmocnienie czlonu proporcjonalnego
float pid_gain_iYaw = 0.02;   //Wzmocnienie czlonu calkujacego
float pid_gain_dYaw = 0;      //Wzmocnienie czlonu rozniczkujacego
float pid_maxDYaw = 200;      //Ograniczenie maksymalnej wartosci czlonu calkujacego
float pid_maxYaw = 200;       //Ograniczenie maksymalnej wartosci na wyjsciu regulatora

//Deklaracja zmiennych programu
int start;                              //Zmienna dotyczaca trybu kontroli drona. Opis znajduje sie w funkcji setup

int receiver_input[11];                 //Wektor zawierajacy 10 kanalow sterowania z czujnika. Deklarowane jest 11 elementow w celu prostszego indeksowania wartosci
int calculatedThrottle;                 //Bazowa wartosc wysylana do wszystkich ESC. Jest rownoznaczna z kanalem 3cim (throttle), ograniczana do wartosci zdefiniowanej jako MAX_THROTTLE

byte data[6] = {0};                     //Wektor do tymczasowego zapisu danych z czujnikow
float gyroInputPitchSpeed, gyroInputRollSpeed, gyroInputYawSpeed;     //Wartosci predkosci katowych z czujnikow
int gyroAxis[4], accAxis[4];            //Wektory zawierajace wartosci z czujnikow ADXL345 i L3G4200. Deklarowane sa po 4 elementy w celu prostszego indeksowania wartosci
long accX, accY, accZ;                  //Wartosci przyspieszen na osiach X, Y i Z drona.
long accVectorValue;                    //Wartosc wektora przyspieszenia
float accAngleRoll, accAnglePitch;      //Wartosci katow roll i pitch obliczone na podstawie czujnika przyspieszen
double gyroPitch, gyroRoll, gyroYaw;    //Wartosci predkosci katowych pitch, roll i yaw drona.
double gyroAxisOffset[4];               //Wartosci offsetow zyroskopa
boolean calibrate_flag = false;         //Flaga okreslajaca czy czujniki zostaly skalibrowane. Dzieki niej, przy kalibracji czujnikow nie bedzie dodawana wartosc offsetow.
float droneAnglePitch, droneAngleRoll;  //Obliczone wartosci katow drona

float pitchCorrection, rollCorrection;            //Wartosci korekcji katow roll i pitch w celu stabilizacji drona (auto-level)
float pid_errorPitch, pid_errorRoll, pid_errorYaw;//Wartosci bledow katow
float pid_aimRoll, pid_aimPitch, pid_aimYaw;      //Wartosc katow, do ktorych bedzie dazyc regulator PID.
float pid_pPitch, pid_iPitch, pid_dPitch;         //Wartosci wyjsc regulatorw P, I i D dla kata typu pitch
float pid_pRoll, pid_iRoll, pid_dRoll;            //Wartosci wyjsc regulatorw P, I i D dla kata typu roll
float pid_pYaw, pid_iYaw, pid_dYaw;               //Wartosci wyjsc regulatorw P, I i D dla kata typu yaw
float pid_outPitch, pid_outRoll, pid_outYaw;      //Wartosci wyjsc regulatorw PID dla wszystkich katow
float pid_lastErrorPitch, pid_lastErrorRoll, pid_lastErrorYaw;  //Wartosci bledow z poprzedniej petli dla wszystkich katow


uint8_t ibusIndex = 0;                  //Wartosc do indeksowania, przy ktorym pakiecie ibus znajduje sie program
uint8_t ibusData[32] = {0};             //Wektor do zapisu danych z komunikacji z odbiornikiem
boolean ibusFrameFlag;                  //Flaga okreslajaca czy skonczylo sie odczytywanie calego pakietu z odbiornika
unsigned long ibusTimeout = 0;          //Zmienna zapisujaca czas w funkcji odczytywania pakietu ibus
unsigned long ibusTimeoutLength = 400;  //Maksymalny czas spedzony w funkcji ibus

unsigned long actualTime;                                       //Zmienna do zapisu aktualnego czasu
int esc_1, esc_2, esc_3, esc_4;                                 //Zmienne obliczonej dlugosci sygnalu na moduly ESC
unsigned long escTimer_1, escTimer_2, escTimer_3, escTimer_4;   //Obliczenie bewzglednego czasu (wzgledem funkcji micros()), dla ktorego sygnal wysylany do ESC zamieniony zostanie na niski

void setup(){
  //Funkcja setup - konfiguracja wejsc/wyjsc, ustawienie rejestrow czujnikow, kalibracja zyroskopu, 
  //Serial.begin(115200);
  //Serial.println("Poczatek programu");

  //Ustawienia odpowiednich portow jako wyjscia. 
  pinMode(10, OUTPUT);        //Sterowanie ESC 1 (prawy front, CCW)
  pinMode(11, OUTPUT);        //Sterowanie ESC 2 (prawy tyl, CW)
  pinMode(12, OUTPUT);        //Sterowanie ESC 3 (lewy tyl, CCW)
  pinMode(13, OUTPUT);        //Sterowanie ESC 4 (lewy front, CW)

  //Serial.println("Piny ustawione");
  
                          //Arduino posiada funkcje digitalWrite(pin, mode) w celu ustalenia czy na porcie napiecie 5v lub 0v.
                          //Funkcja ta dodaje jednak dodatkowe opoznienie, na ktore nie mozna pozwolic przy wysylaniu sygnalow do modulow ESC
                          //Wykorzystanie zostanie rejestr PORTB, ktory ma rozmiar 8 bitow. Odpowiada on za wyjscia cyfrowe 10 - 13.
                          //Przykladowo, w celu wlaczenia portow 10, 11, 12, 13 do 5V, zastosowac mozna nastepujace rownanie
                          //PORTD |= B11110000
                          //Aby wylaczyc te cztery porty, zastosowac mozna nastepujace rownanie
                          //PORTD &= B00001111
                          //Zastosowanie operatorow |= i &= pozwoli na zmiane tylko tych portow. Dzieki temu nie nalezy zmieniac programu gdy wykorzystane zostana inne bity
                          //ktorych tyczy ten rejestr.
                          
  start = 0;              //Ustal tryb startu na 0; 
                          //Tryb 0 - wszystkie silniki wylaczone.
                          //Tryb 1 - aktywowany poprzez ustawienie joysticka throttle na minimum, wszystkie silniki wylaczone. 
                          //Tryb 2 - aktywowane poprzez ustawienie joysticka throttle na minimum i sticka yaw na lewo, wszystkie silniki wlaczone

  Serial1.begin(115200);  //Wlaczenie komunikacji typu serial
  while(!Serial1){}       //Oczekiwanie na poprawne polaczenie. Bez tego polaczenia, funkcja setup zatrzymuje sie tutaj.
  
  //Serial.println("Komunikacja Serial rozpoczeta");
  
  Wire.begin();           //Wlaczenie komunikacji typu I2C
  Wire.setClock(400000);  //Ustawienie czestotliwosci I2C do maksymalnej wartosci 400kHz

  //Serial.println("Wlaczona komunikacja Serial i I2C");

  //batteryVoltage = map(analogRead(8), 0, 1023, 0, 1260);      //Funkcja do odczytywania napiecia na baterii. Mozliwa do wykorzystana w przyszlosci do kompensacji silnikow od stanu napiecia.

  sensorsRegisters();     //Przeprowadzenie konfiguracji czujnikow

  //Serial.println("Czujniki skonfigurowane");

  //delay(2000);            //Po konfiguracji czujnikow, czekaj 2 sekundy
  for (int i=0; i < 400; i++) {
    //Zamiast funkcji delay, wykorzystywana jest ta petla for w celu wyslania wstepnego sygnalu sterowego do synchronizacji modulow ESC
    //Poniewaz funkcja Delay nie jest idealna, do silnikow bedzie wysylany sygnal minimalnie dluzszy od 1000 mikrosekund. Dzieki temu
    //silniki beda probowaly sie obracac, co bedzie widocznym znakiem osiagnieca tego etapu programu.
    PORTB |= B11110000;
    delayMicroseconds(1000);
    PORTB &= B00001111;
    delayMicroseconds(4000); 
  }

  //Serial.println("Kalibracja czujnikow");
  //Przeprowadzenie 1000 pomiarow zyroskopu z czestotliwoscia ~200Hz przez 5 sekund
  for (int i = 0; i < 1000; i++){
    sensorsRead();
    gyroAxisOffset[1] += gyroAxis[1];
    gyroAxisOffset[2] += gyroAxis[2];
    gyroAxisOffset[3] += gyroAxis[3];
    PORTB |= B11110000;
    delayMicroseconds(1000);
    PORTB &= B00001111;
    delayMicroseconds(4000); 
  }

  gyroAxisOffset[1] /= 1000;     //Podzielenie zsumowanych odczytow przez ilosc probek
  gyroAxisOffset[2] /= 1000;
  gyroAxisOffset[3] /= 1000;

  calibrate_flag = true;      //Zakonczenie kalibracji - teraz wartosci czujnika odjete zostana od tych offsetow

  //Serial.println("Kalibracja czujnikow zakonczona");
  readIBUS();               //Odczytaj stan stickow
  while(receiver_input[3] > 1100){
    //Dron bedzie czekal, az throttle (receiver_input[3]) bedzie na najnizszym poziomie. Za kazdym razem, gdy warunek nie jest spelniony, czytana bedzie wartosc na stickach.
    readIBUS();
    PORTB |= B11110000;
    delayMicroseconds(1000);
    PORTB &= B00001111;
    delayMicroseconds(4000); 
  }

  //Serial.println("Koniec funkcji setup.");
  
  actualTime = micros();  //Okresl aktualny czas przed rozpoczeciem funkcji loop
}

void loop(){

  gyroInputRollSpeed =   gyroRoll / GYRO_RESOLUTION;         //Konwersja sygnalow z czujnika zyroskopu na stopnie na sekunde
  gyroInputPitchSpeed =  gyroPitch / GYRO_RESOLUTION;
  gyroInputYawSpeed =    gyroYaw / GYRO_RESOLUTION;

  droneAnglePitch +=  gyroPitch * RAW_ANGLE_CONVERSION;  //Funkcja calkujaca predkosc katowa na katy
  droneAngleRoll +=   gyroRoll * RAW_ANGLE_CONVERSION;
 
  droneAnglePitch -=  droneAngleRoll * sin(gyroYaw * R2D);   //Funkcja odpowiadajaca za poprawnie rejestrowane katy w razie zmiany kata typu Yaw drona
  droneAngleRoll +=   droneAnglePitch * sin(gyroYaw * R2D);

  accVectorValue =  sqrt((accX*accX)+(accY*accY)+(accZ*accZ));      //Obliczenie wartosci kata przyspieszeniax
  
  if(abs(accY) < accVectorValue){                                   //Zapobiegniecie funkcji asin przed bledna odpowiedzia
    accAnglePitch = asin((float)accY/accVectorValue)* 57.296;       //Obliczenie kata typu pitch 
  }
  if(abs(accX) < accVectorValue){                                   //Zapobiegniecie funkcji asin przed bledna odpowiedzia
    accAngleRoll = asin((float)accX/accVectorValue)* -57.296;       //Obliczenie kata typu roll
  }
  
  droneAnglePitch = droneAnglePitch * 0.95 + accAnglePitch * 0.05;  //Filtr komplementarny 
  droneAngleRoll = droneAngleRoll * 0.95 + accAngleRoll * 0.05;
  
  pitchCorrection = droneAnglePitch * AUTO_LEVEL_STRENGTH;          //Obliczenie wartosci katow korekcji w celu ustalenia automatycznego poziomu drona
  rollCorrection = droneAngleRoll * AUTO_LEVEL_STRENGTH;

  if(receiver_input[3] < 1050 && receiver_input[4] < 1050)start = 1;        //Wlaczenie drona. Stick throttle na minimalny poziom, stick yaw odchylony na lewo
  if(start == 1 && receiver_input[3] < 1050 && receiver_input[4] > 1450){   //Czy stick wrocil do minimalnego throttle i brak odchylenia yaw?
    //W tej funkcji zresetowane zostana katy drona do katow obliczonych przez czujnik przyspieszen oraz zresetowane zostana wartosci na regulatorze PID.
    //Pozwoli to na czysty start w wypadku ponownego startu drona.
    start = 2;                                                              //Tryb 2 drona - wlaczone silniki, pelna kontrola.
    
    droneAnglePitch = accAnglePitch;
    droneAngleRoll = accAngleRoll;

    pid_iPitch = 0; pid_iRoll = 0; pid_iYaw = 0;
    pid_lastErrorPitch = 0; pid_lastErrorRoll = 0; pid_lastErrorYaw = 0;
  }

  if(start == 2 && receiver_input[3] < 1050 && receiver_input[4] > 1950)start = 0;    //Wylaczenie silnikow drona. Stick throttle na minimalnym poziomie, stick yaw odchylony na prawo.

  //Wyzerowanie docelowych predkosci katowych
  pid_aimRoll = 0;      
  pid_aimPitch = 0;
  pid_aimYaw = 0;

  //Funkcje dla marginesow przy zerowym wychyleniu stickow
  if(receiver_input[1] > (1500 + DEADBAND))pid_aimRoll = receiver_input[1] - (1500 + DEADBAND);
  else if(receiver_input[1] < (1500 - DEADBAND))pid_aimRoll = receiver_input[1] - (1500 - DEADBAND);
  
  if(receiver_input[2] > (1500 + DEADBAND))pid_aimPitch = receiver_input[2] - (1500 + DEADBAND);
  else if(receiver_input[2] < (1500 - DEADBAND))pid_aimPitch = receiver_input[2] - (1500 - DEADBAND);

  if(receiver_input[3] > 1050) { //Kat typu yaw nie bedzie sie zmienial gdy silniki sa wylaczone
    if(receiver_input[4] > (1500 + DEADBAND))pid_aimYaw = (receiver_input[4] - (1500 + DEADBAND));
    else if(receiver_input[4] < (1500 - DEADBAND))pid_aimYaw = (receiver_input[4] - (1500 - DEADBAND));
  }

  pid_aimRoll =   (pid_aimRoll - rollCorrection) / AIM_ANGLE_DIVISOR;
  pid_aimPitch =  (pid_aimPitch - pitchCorrection) / AIM_ANGLE_DIVISOR;
  pid_aimYaw =    pid_aimYaw / AIM_ANGLE_DIVISOR;

  //Idea tych poprzednich rownan bazuje na tym, ze predkosc katowa, do ktorego dazymy zalezy od: odsuniecia odpowiednich joystickow kontrolera (wartosci od -500 do 500), 
  //wartosci obliczonych katow drona pomnozonych o stala AUTO_LEVEL_STRENGTH i ostatecznie od stalej AIM_ANGLE_DIVISOR, ktora ogranicza maksymalny zakres
  //tych wartosci.

  //Teraz mozna przeprowadzic obliczenia PID
  calculatePID();

  if (start == 2) {                                                             //Dron jest w trybie 2gim z pelna gotowoscia do lotu.
    if (receiver_input[3] > MAX_THROTTLE) calculatedThrottle = MAX_THROTTLE;    //Aby zapobiec saturacji sticka throttle, jego sygnal jest ograniczony do wartosci MAX_THROTTLE. Inaczej w wypadku pelnej mocy, wszystkie silniki
                                                                                //dostawaly by sygnal 2000 i nie istniala by mozliwosc kontroli drona.
    //Kalkulacje dlugosci pulsow dla modulow ESC.
    esc_1 = MAX_THROTTLE - pid_outPitch + pid_outRoll - pid_outYaw; //ESC 1 - prawy front, CCW
    esc_2 = MAX_THROTTLE + pid_outPitch + pid_outRoll + pid_outYaw; //ESC 2 - prawy tyl, CW
    esc_3 = MAX_THROTTLE + pid_outPitch - pid_outRoll - pid_outYaw; //ESC 3 - lewy tyl, CCW
    esc_4 = MAX_THROTTLE - pid_outPitch - pid_outRoll + pid_outYaw; //ESC 4 - lewy front, CW

    //Nie chcemy, by silniki przestawaly dzialac w czasie lotu, dlatego ich minimalna wartosc bedzie zawsze rowna MINIMUM_THROTTLE
    //Nie chcemy rowniez by wartosc sygnalu byla wieksza od maksymalnej wartosci 2000 mikrosekund.
    if (esc_1 < MINIMUM_THROTTLE) esc_1 = MINIMUM_THROTTLE;
    else if (esc_1 > 2000) esc_1 = 2000;
    if (esc_2 < MINIMUM_THROTTLE) esc_2 = MINIMUM_THROTTLE;
    else if (esc_2 > 2000) esc_2 = 2000;
    if (esc_3 < MINIMUM_THROTTLE) esc_3 = MINIMUM_THROTTLE;
    else if (esc_3 > 2000) esc_3 = 2000;
    if (esc_4 < MINIMUM_THROTTLE) esc_4 = MINIMUM_THROTTLE;
    else if (esc_4 > 2000) esc_4 = 2000;
  }

  else {      //W wypadku, gdy dron jest w trybie 1-szym lub 0-wym
    //Silniki beda wylaczone
    esc_1 = 1000; esc_2 = 1000; esc_3 = 1000; esc_4 = 1000;
  }
   
  while(micros() - actualTime < 5000);      //Czekamy, az minie 5000 mikrosekund. Wszystkie nastepne operacje rozpoczna sie natychmiast po jednej petli rownej
                                            //5000 mikrosekund. Dzieki temu petla bedzie dzialac ze stala czestotliwoscia 200Hz
  actualTime = micros();                    //Zapisywany jest aktualny bezwzgledny czas

  PORTB |= B11110000;                       //Wszystkie wyjscia na ESC sa przelaczane na tryb 5V
  
  escTimer_1 = esc_1 + actualTime;          //Obliczany jest bezwzgledny czas dla kazdego modulu ESC, w ktorym poszczegolne wyjscie przelaczone zostanie na 
  escTimer_2 = esc_2 + actualTime;          //tryb niski
  escTimer_3 = esc_3 + actualTime;
  escTimer_4 = esc_4 + actualTime;

  //Poniewaz sygnaly zawsze musza byc na stanie wyzszym (5v) przez 1000 mikrosekund, oznacza to ze posiadamy 1000 mikrosekund na przeprowadzenie roznych kalkulacji.
  readIBUS();       //Odczytanie wychylen stickow
  sensorsRead();    //Odczytanie wartosci z czujnikow.

  while(PORTB >= 16) {                                //Program bedzie w tej petli dopoki wszystkie sygnaly na ESC nie beda w stanie niskim
    actualTime = micros();                            //Zapisanie aktualnego bezwzglednego czasu
    if(escTimer_1 <= actualTime)PORTB &= B11101111;   //Te rownania sprawdzaja, czy minal czas na wylaczenie sygnalow na ESC, po czym przelaczaly sygnal na niski (0V).
    if(escTimer_2 <= actualTime)PORTB &= B11011111;
    if(escTimer_3 <= actualTime)PORTB &= B10111111;
    if(escTimer_4 <= actualTime)PORTB &= B01111111;
  }
}

void sensorsRegisters(){
  //Ustawienie czujnika L3G4200 - wlaczenie czujnika, wlaczenie funkcji BDU i ustalenie rozdzielczosci do 500dps (stopni na sekunde)
  Wire.beginTransmission(L3G4200);
  Wire.write(GYRO_CTRL_REG1);
  Wire.write(0x0F);                 //wlaczenie czujnika na wszystkich osiach
  Wire.endTransmission();
  
  Wire.beginTransmission(L3G4200);
  Wire.write(GYRO_CTRL_REG4);
  Wire.write(0x90);                 //wlaczenie opcji BDU i ustalenie rozdzielczosci do 500dps
  Wire.endTransmission();

  //Ustawienie czujnika ADXL345 - wlaczenie czujnika i ustalenie rozdzielczosci do +- 8g
  Wire.beginTransmission(ADXL345);
  Wire.write(ACC_POWER_CTL);
  Wire.write(0x08);                 //wlaczenie czujnika na wszystkich osiach
  Wire.endTransmission();
  
  Wire.beginTransmission(ADXL345);
  Wire.write(ACC_DATA_FORMAT);
  Wire.write(0x02);                 //ustalenie rozdzielczosci do +- 8g
  Wire.endTransmission();
}

void sensorsRead(){
  //Odczyt danych od czujnika L3G4200
  Wire.beginTransmission(L3G4200);
  Wire.write(0xA8);                       //Odczytywanie pomiarow z adresu 0xA8. Wedlug datasheeta, pomiary znajduja sie na adresie 0x28, jednak w celu wlaczenia inkrementacji pomiarow, pierwszy bit musi byc zamieniony na stan wyzszy - czyli 0xA8
  Wire.endTransmission();
  Wire.requestFrom(L3G4200, 6);           //Odczyt 6 bajtow od czujnika - po dwa bajty na kazda os
  data[0]=Wire.read();
  data[1]=Wire.read();
  data[2]=Wire.read();
  data[3]=Wire.read();
  data[4]=Wire.read();
  data[5]=Wire.read();
  gyroAxis[1] = data[1]*256 + data[0];   //Zapis pobranych danych do dwobajtowych wartosci - X, Y, Z
  gyroAxis[2] = data[3]*256 + data[2];
  gyroAxis[3] = data[5]*256 + data[4];

  //Odczyt danych od czujnika ADXL345
  Wire.beginTransmission(ADXL345);
  Wire.write(0x32);                       //Odczytanie pomiarow z adresu 0x32. Czujnik od razu pozwala na inkrementacje
  Wire.endTransmission();
  Wire.requestFrom(ADXL345, 6);              //Odczyt 6 bajtow od czujnika - po dwa bajty na kazda os
  data[0]=Wire.read();
  data[1]=Wire.read();
  data[2]=Wire.read();
  data[3]=Wire.read();
  data[4]=Wire.read();
  data[5]=Wire.read();
  accAxis[1] = data[1]*256 + data[0];    //Zapis pobranych danych do dwobajtowych wartosci - X, Y, Z
  accAxis[2] = data[3]*256 + data[2];
  accAxis[3] = data[5]*256 + data[4];
  
  if(calibrate_flag){
    gyroAxis[1] -= gyroAxisOffset[1];
    gyroAxis[2] -= gyroAxisOffset[2];
    gyroAxis[3] -= gyroAxisOffset[3];
  }
  
  gyroPitch = gyroAxis[1]*(1);          //Predkosc katowa X z czujnika to predkosc katowa pitch
  gyroRoll = gyroAxis[2]*(1);           //Predkosc katowa Y z czujnika to predkosc katowa roll
  gyroYaw = gyroAxis[3]*(-1);           //Predkosc katowa Z z czujnika to predkosc katowa yaw, odwrocona

  accX = accAxis[1]*(1);                //Os X z czujnika to os X drona
  accY = accAxis[2]*(1);                //Os Y z czujnika to os Y drona
  accZ = accAxis[3]*(-1);               //Os Z z czujnika to os Z drona, odwrocona
}

void readIBUS() { 
  //Funkcja odczyty stanow stickow na podstawie komunikacji typu Serial dla pakietow typu IBUS
  uint16_t checksum;
  uint16_t sum;
  unsigned long currenttime = 0;                                  //Aby zapobiec blokadzie programu w tej funkcji, obliczany bedzie rowniez czas.
  ibusFrameFlag = true;
  if (Serial1.available() >= 32) {                                //Czy w pamieci Serial1 dostepne sa 32 bajty?
    ibusTimeout = micros();                                       //Zapis aktualnego czasu
    ibusData[1] = Serial1.read();                                 //Przeczytaj pierwszy pakiet, zapisz do zmiennej drugiego pakietu
    while (ibusFrameFlag == true) {                               //W tej petli bedzie szukana wartosc headera 0x2040
      ibusData[0] = ibusData[1];                                  //Wartosc pakietu przesuwana jest do packet1
      ibusData[1] = Serial1.read();                               //Czytany jest kolejny pakiet w kolejnosci
      if (ibusData[0] == 0x20 && ibusData[1] == 0x40) {           //Czy po operacjach odczytywania i przesuwania, czy znaleziony zostala kombinacja pakietow odpowiadajaca wartosciom headera?
                                                                  //Odczytywanie calej ramki ibusa
        for(int i=2;i<32;i++) ibusData[i] = Serial1.read();       //Poniewaz header to pierwszy i drugi bajt, petla zaczyna sie od wartosci 2
        checksum= 0xFFFF;                                         //Ostatnie dwa bajty ramki to checksum. Aby zweryfikowac poprawnosc odebranej informacji, od wartosci 0xFFFF nalezy odjac wartosci pierwszych 30 bajtow
        for (int i = 0; i < 30; i++) checksum -= ibusData[i];
        sum = ibusData[30] | (ibusData[31] << 8);                 //Zapisanie wartosci checksum z ramki
        if (checksum == sum) {                                    //Czy obliczona checksum jest rowna checksum z ramki?
          receiver_input[1] = (ibusData[ 3] << 8) | ibusData[ 2]; //Zapisanie 10 kanalow z ramki ibus do kanalow wektora wartosci odbiornika.
          receiver_input[2] = (ibusData[ 5] << 8) | ibusData[ 4];
          receiver_input[3] = (ibusData[ 7] << 8) | ibusData[ 6];
          receiver_input[4] = (ibusData[ 9] << 8) | ibusData[ 8];
          receiver_input[5] = (ibusData[11] << 8) | ibusData[10];
          receiver_input[6] = (ibusData[13] << 8) | ibusData[12];
          receiver_input[7] = (ibusData[15] << 8) | ibusData[14];
          receiver_input[8] = (ibusData[17] << 8) | ibusData[16];
          receiver_input[9] = (ibusData[19] << 8) | ibusData[18];
          receiver_input[10] = (ibusData[21] << 8) | ibusData[20];
          stickLimits();                                          //Zawolanie funkcji do stworzenia marginesow dla stickow
        }
        else {}                                                   //Obliczona checksum nie jest rowna checksum z ramki. Nie zapisujemy zadnych danych.
        ibusFrameFlag = false;                                    //Koniec petli
      }
      else {                                                      //Nie wykryto headera
        currenttime = micros();                                   //Sprawdzenie aktualnego czasu
        if (currenttime - ibusTimeout > ibusTimeoutLength) {      //Czy minal juz timeout?
          ibusFrameFlag = false;                                  //Koniec petli
        }
      }
    }
  }
}

void stickLimits() {
  //Funkcja dla ktorej wartosci stickow znajdujace sie poza zakresemi STICK_HIGH_LIMIT i STICK_LOW_LIMIT zostana zamienione na wartosci STICK_HIGH i STICK_LOW
  if(receiver_input[1] < STICK_LOW_LIMIT) receiver_input[1] = STICK_LOW;
  if(receiver_input[2] < STICK_LOW_LIMIT) receiver_input[2] = STICK_LOW;
  if(receiver_input[3] < STICK_LOW_LIMIT) receiver_input[3] = STICK_LOW;
  if(receiver_input[4] < STICK_LOW_LIMIT) receiver_input[4] = STICK_LOW;
  if(receiver_input[1] > STICK_HIGH_LIMIT) receiver_input[1] = STICK_HIGH;
  if(receiver_input[2] > STICK_HIGH_LIMIT) receiver_input[2] = STICK_HIGH;
  if(receiver_input[3] > STICK_HIGH_LIMIT) receiver_input[3] = STICK_HIGH;
  if(receiver_input[4] > STICK_HIGH_LIMIT) receiver_input[4] = STICK_HIGH;
}

void calculatePID(){
  //Obliczenia PID dla kata pitch
  //Obliczenie bledu
  pid_errorPitch = gyroInputPitchSpeed - pid_aimPitch;                                    //Wartosc bledu (error) to roznica miedzy odczytana wartoscia predkosci katowej od wartosci predkosci katowej do ktorej chcemy dazyc.

  //Obliczenie regulatora P (proporcjonalny)
  pid_pPitch = pid_gain_pPitch * pid_errorPitch;                                     //Jest to wartosc bledu pomnozona przez wzmocnienie regulatora P
  //Obliczenie regulatora I (calkujacy)
  pid_iPitch += pid_gain_iPitch * pid_errorPitch;                                    //Do sumy wartosci I dodana jest wartosc bledu wzmocniona o wartosc wzmocnienia regulatora I
  //Obliczenie regulatora D (rozniczkujacy)
  pid_dPitch = pid_gain_dPitch * (pid_errorPitch - pid_lastErrorPitch);              //Jest to roznica miedzy aktualnym bledem a poprzednio zmierzonym bledem, pomnozona przez wzmocnienie regulatora D
  
  if(pid_iPitch > pid_maxPitch) pid_iPitch = pid_maxPitch;                           //Wartosc na wyjsciu regulatora I ograniczona jest do +- pid_max
  else if(pid_iPitch < pid_maxPitch * -1) pid_iPitch = pid_maxPitch * -1;            //Calkowanie jest ograniczone poniewaz bardzo latwo dron moze calkowac przez dluzsza ilosc czasu, tworzac bardzo wysoka wartosc na tym wzmocneniu.
  //nie jest wykorzystywana funkcja clamp() arduino gdyz dodaje ona niepotrzebne opoznienia

  pid_outPitch = pid_pPitch + pid_iPitch + pid_dPitch;                               //Wyjscie regulatora PID to suma wyjsc wszystkich trzech regulatorow.

  if(pid_outPitch > pid_maxPitch)pid_outPitch = pid_maxPitch;                        //Wartosc na wyjsciu regulatora PID jest ograniczona do +- pid_max
  else if(pid_outPitch < pid_maxPitch * -1)pid_outPitch = pid_maxPitch * -1;

  pid_lastErrorPitch = pid_errorPitch;                                              //Zapisanie aktualnej wartosci bledu w celu obliczenia roznicy czesci rozniczkujacej w nastepnym cyklu

  //Obliczenia PID dla kata roll
  //Obliczenie bledu
  pid_errorRoll = gyroInputRollSpeed - pid_aimRoll;                                    //Wartosc bledu (error) to roznica miedzy odczytana wartoscia predkosci katowej od wartosci predkosci katowej do ktorej chcemy dazyc.

  //Obliczenie regulatora P (proporcjonalny)
  pid_pRoll = pid_gain_pRoll * pid_errorRoll;                                     //Jest to wartosc bledu pomnozona przez wzmocnienie regulatora P
  //Obliczenie regulatora I (calkujacy)
  pid_iRoll += pid_gain_iRoll * pid_errorRoll;                                    //Do sumy wartosci I dodana jest wartosc bledu wzmocniona o wartosc wzmocnienia regulatora I
  //Obliczenie regulatora D (rozniczkujacy)
  pid_dRoll = pid_gain_dRoll * (pid_errorRoll - pid_lastErrorRoll);               //Jest to roznica miedzy aktualnym bledem a poprzednio zmierzonym bledem, pomnozona przez wzmocnienie regulatora D
  
  if(pid_iRoll > pid_maxRoll) pid_iRoll = pid_maxRoll;                            //Wartosc na wyjsciu regulatora I ograniczona jest do +- pid_max
  else if(pid_iRoll < pid_maxRoll * -1) pid_iRoll = pid_maxRoll * -1;             //Calkowanie jest ograniczone poniewaz bardzo latwo dron moze calkowac przez dluzsza ilosc czasu, tworzac bardzo wysoka wartosc na tym wzmocneniu.
  //nie jest wykorzystywana funkcja clamp() arduino gdyz dodaje ona niepotrzebne opoznienia

  pid_outRoll = pid_pRoll + pid_iRoll + pid_dRoll;                                //Wyjscie regulatora PID to suma wyjsc wszystkich trzech regulatorow.

  if(pid_outRoll > pid_maxRoll)pid_outRoll = pid_maxRoll;                         //Wartosc na wyjsciu regulatora PID jest ograniczona do +- pid_max
  else if(pid_outRoll < pid_maxRoll * -1)pid_outRoll = pid_maxRoll * -1;

  pid_lastErrorRoll = pid_errorRoll;                                              //Zapisanie aktualnej wartosci bledu w celu obliczenia roznicy czesci rozniczkujacej w nastepnym cyklu

  //Obliczenia PID dla kata yaw
  //Obliczenie bledu
  pid_errorYaw = gyroInputYawSpeed - pid_aimYaw;                                    //Wartosc bledu (error) to roznica miedzy odczytana wartoscia predkosci katowej od wartosci predkosci katowej do ktorej chcemy dazyc.

  //Obliczenie regulatora P (proporcjonalny)
  pid_pYaw = pid_gain_pYaw * pid_errorYaw;                                     //Jest to wartosc bledu pomnozona przez wzmocnienie regulatora P
  //Obliczenie regulatora I (calkujacy)
  pid_iYaw += pid_gain_iYaw * pid_errorYaw;                                    //Do sumy wartosci I dodana jest wartosc bledu wzmocniona o wartosc wzmocnienia regulatora I
  //Obliczenie regulatora D (rozniczkujacy)
  pid_dYaw = pid_gain_dYaw * (pid_errorYaw - pid_lastErrorYaw);               //Jest to roznica miedzy aktualnym bledem a poprzednio zmierzonym bledem, pomnozona przez wzmocnienie regulatora D
  
  if(pid_iYaw > pid_maxYaw) pid_iYaw = pid_maxYaw;                            //Wartosc na wyjsciu regulatora I ograniczona jest do +- pid_max
  else if(pid_iYaw < pid_maxYaw * -1) pid_iYaw = pid_maxYaw * -1;             //Calkowanie jest ograniczone poniewaz bardzo latwo dron moze calkowac przez dluzsza ilosc czasu, tworzac bardzo wysoka wartosc na tym wzmocneniu.
  //nie jest wykorzystywana funkcja clamp() arduino gdyz dodaje ona niepotrzebne opoznienia

  pid_outYaw = pid_pYaw + pid_iYaw + pid_dYaw;                                //Wyjscie regulatora PID to suma wyjsc wszystkich trzech regulatorow.

  if(pid_outYaw > pid_maxYaw)pid_outYaw = pid_maxYaw;                         //Wartosc na wyjsciu regulatora PID jest ograniczona do +- pid_max
  else if(pid_outYaw < pid_maxYaw * -1)pid_outYaw = pid_maxYaw * -1;

  pid_lastErrorYaw = pid_errorYaw;                                              //Zapisanie aktualnej wartosci bledu w celu obliczenia roznicy czesci rozniczkujacej w nastepnym cyklu
}
