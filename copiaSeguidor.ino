#include <QTRSensors.h>

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

int velHIGH = 0;
int velLOW = 255;
int A1A = 12;
int A1B = A0;
int B1A = 11;
int B1B = A1; 

int limiar = 700; //limiar do sensor de luz

int sensorFim = 10; 
int contFim = 8; // 4; //Conta marcas de início, fim e cruzamentos
boolean flagFim = false;

//Controle PID
int   PID = 0;
int   somatorioErro = 0; //Variação do erro
int   deltaErro = 0;     //Somatório do erro
int   erroAnterior = 0;  //Erro delta tempo anterior

const int velInercia = 30; //Velocidade mínima para romper inércia dos motores  (0 <= velPista <= 255)
int       velMotor;        //Velocidade aplicada nos motores (velPista + PID)

const int velPista   = 200; //Velocidade média de pista (0 <= velPista <= 100)

//Constantes PID
const int Kp = 50;          //Ganho Proposcional
const int Kd = 25;          //Ganho Derivativo
const int Ki = 0.02;          //Ganho Integral

void setup()
{
  Serial.begin(9600);
  
  //Pino sensor fim
  pinMode(sensorFim, INPUT);

  //Pinos para controle do motor B
  pinMode(B1A, OUTPUT);  
  pinMode(B1B, OUTPUT);  
  //Pinos para controle do motor A
  pinMode(A1A, OUTPUT);  
  pinMode(A1B, OUTPUT); 

  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){2, 3, 4, 5, 6, 7, 8, 9}, SensorCount);
  //qtr.setEmitterPin(10);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  //Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(2000);
}

void frenteEsq(int vel){
 //digitalWrite(B1A, LOW);
 //digitalWrite(B1B, HIGH);

  digitalWrite(A1A, LOW);
  analogWrite(A1B, vel);

 //analogWrite(ENB, vel);
}

void frenteDir(int vel){
 //digitalWrite(A1A, HIGH);
 //digitalWrite(A1B, LOW);
 
 digitalWrite(B1A, LOW);
 analogWrite(B1B, vel);
 
 //analogWrite(ENA, vel);

}

void trasEsq(int vel){
  /*digitalWrite(B1A, HIGH);
  digitalWrite(B1B, LOW);
  analogWrite(ENB, vel);*/
  digitalWrite(A1A, HIGH);
  analogWrite(A1B, vel);
}

void trasDir(int vel){
 /*digitalWrite(A1A, LOW);
 digitalWrite(A1B, HIGH);
 analogWrite(ENA, vel);*/
  digitalWrite(B1A, HIGH);
  analogWrite(B1B, vel);
}

void para_motores() {
  //MotorA_para
  digitalWrite(A1A, HIGH);  //A 
  digitalWrite(A1B, HIGH);  //A  
  //MotorB_para 
  digitalWrite(B1A, HIGH);  //B
  digitalWrite(B1B, HIGH);  //B   
}

void calculaPID () {
  //Lê sensores e determina posição entre 0 e 7000
  uint16_t position = qtr.readLineWhite(sensorValues); 
  
  //autoCalibration ();
  //Calcula Erro
  float erro = ((float)position - 3500)/1000; // Erro entre -3,5 e +3,5
  //Erro - Deriva para esquerda 
  //Erro + Deriva para direita    
  somatorioErro = somatorioErro + erro;
  deltaErro = erroAnterior - erro;
  erroAnterior = erro;
  //Calcula controle PID
  PID = Kp*erro + Ki*somatorioErro + Kd*deltaErro;

  //Serial.print("PID: ");
  //Serial.println(PID);
  /*
  Serial.println("--------");
  Serial.print("position: ");
  Serial.println(position);  
  Serial.println("--------");
  Serial.print("erro: ");
  Serial.println(erro);
  Serial.println("--------");
  Serial.print("PID: ");
  Serial.println(PID);
  Serial.println("--------");
  */
}

void segueLinha () {
  if ((velPista + PID) >= 0 && (velPista + PID) <= 100) {   
    velMotor = map((velPista + PID), 0, 100, velInercia, 255);
    frenteDir(velMotor); //Erro +: MotorDir frente
  }
  else if ((velPista + PID) < 0) { //Vel negativa
    velMotor = map(-(velPista + PID), 0, 100, velInercia, 255);
    trasDir(velMotor); // MotorDir tras
  }
  else if ((velPista + PID) > 100) { //Vel máxima
    frenteDir(255); //MotorDir frente vmáx
  }
  if ((velPista - PID) >= 0 && (velPista - PID) <= 100) {
    velMotor = map((velPista - PID), 0, 100, velInercia, 255);
    frenteEsq(velMotor); //Erro -: MotorEsq frente
  }
  else if ((velPista - PID) < 0){ //Vel negativa
    velMotor = map(-(velPista - PID), 0, 100, velInercia, 255);
    trasEsq(velMotor); //MotorEsq tras
  }
  else if ((velPista - PID) > 100) { //Vel máxima
    frenteEsq(255); //MotorDir frente vmáx
  }
}

void loop()
{
    
    if(contFim <= 0) {
      para_motores();

      Serial.println("Parou");
    } else {
      //Calcula PID
      calculaPID();
      
      segueLinha();

      //Verificar se o sensor de parada passou pela linha branca
      int leituraSensorFim = digitalRead(sensorFim);
      Serial.print("Sensor parada: ");
      Serial.println(leituraSensorFim);


      if(leituraSensorFim == 0) //O é o branco
        flagFim = true;
      
      if(leituraSensorFim == 1 && flagFim) { //1 é o preto
        contFim--;
        flagFim = false;

        if(contFim == 0)
          delay(1000);
      }
        
    }


/*
  //Segue linha
  while(contFim > 0){
    
    
    //sensorFim = analogRead(3);
    sensorFim = digitalRead(10);

    Serial.println(sensorFim);
    delay(1000);

    if (sensorFim <= limiar)
      flagFim = true; 

    if (sensorFim > limiar && flagFim == true){
      flagFim=false;
      contFim--;
   }
  }
*/
  
  
  
}
