/* 
 * File:   CarProject.ino
 * Author: Judah Holanda Correia Lima <judahholanda7@gmail.com>
 *
 * Created on March 31, 2014, 10:37 PM
 */

#define constantFrontSonarTriggerPin 5
#define constantFrontSonarEchoPin 3 

#define constantRightSonarTriggerPin 4
#define constantRightSonarEchoPin 2

#define constantLeftSonarTriggerPin 6
#define constantLeftSonarEchoPin 7

#define constantSonarTriggerDelay 10

#define constantRightMotorPin0 12
#define constantRightMotorPin1 13

#define constantLeftMotorPin0 9
#define constantLeftMotorPin1 8

#define constantEnableRightMotorPin 11
#define constantEnableLeftMotorPin 10

#define constantSideNoSide 0
#define constantSideFront 1 
#define constantSideRight 2 
#define constantSideLeft 3 
#define constantSideBack 4 

#define constantFullSpeed 9999
#define constantMaximumSpeed 100       

#define constantMaximumObjectDistance  10//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////Teste de constante
#define constantMinimumObjectDistance 2//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////Teste de constante
#define constantMaximumSonarRange  150
#define constantMinimumSonarRange  3

#define constantMaximumObjectDistanceTimesFactor 8

#define constantSerialBaudRate  9600

#define constantCourseTimeEnd 40000

#define constantForwardWalkInitFactor 100
#define constantForwardWalkFactor 50

#define constantTurnRightDelayFactor 300//ver isso!!!
#define constantTurnLeftDelayFactor 250//ver isso!!!

#define constantRefreshDelayFactor 500

#define constantMicrosecondsToCentimetersFactor 58.2

#define constantBooleanSize 2

#define constantCarStatusStart 0
#define constantCarStatusNoObstacleStart 1
#define constantCarStatusNoObstacle 2
#define constantCarStatusAvoidObstacle 3

int carStatus;

boolean avoidSide;
boolean wallSide;
boolean flagAvoidObstacle;

unsigned long timeFrontSonar;
unsigned long timeLeftSonar;
unsigned long timeRightSonar;

unsigned long courseTime;

int leftMotorSpeedVariation;
int rightMotorSpeedVariation;

int leftMotorSpeed;
int rightMotorSpeed;

boolean leftMotorWay;//0 forward
boolean rightMotorWay;//1 backward

boolean goingForward;
boolean goingLeft;
boolean goingRight;
boolean goingBackward;

boolean firstMeasureFront;
boolean firstMeasureRight;
boolean firstMeasureLeft;

int lastDistanceFront;
int lastDistanceRight;
int lastDistanceLeft;

/*
** ===================================================================
 **     Method      :  void setup()
 **
 **     Description :
 **         Este método tem como função inicicializar todas as variáveis
 **         globais e pinos usados.
 ** ===================================================================
 */

void setup() {
  Serial.begin(constantSerialBaudRate);
  pinMode(constantFrontSonarTriggerPin, OUTPUT);
  pinMode(constantRightSonarTriggerPin, OUTPUT);
  pinMode(constantLeftSonarTriggerPin, OUTPUT);

  pinMode(constantRightMotorPin0, OUTPUT);
  pinMode(constantRightMotorPin1, OUTPUT);
  pinMode(constantLeftMotorPin0, OUTPUT);
  pinMode(constantLeftMotorPin1, OUTPUT);

  pinMode(constantEnableRightMotorPin, OUTPUT);
  pinMode(constantEnableLeftMotorPin, OUTPUT);

  pinMode(constantFrontSonarEchoPin, INPUT);
  pinMode(constantLeftSonarEchoPin, INPUT);
  pinMode(constantRightSonarEchoPin, INPUT);

  attachInterrupt(1, pulseRisingFrontSonarInterruption, RISING);// interrupcao do sonar frontal
  attachInterrupt(0, pulseRisingRightSonarInterruption, RISING);// interrupcao do sonar direito
  //attachInterrupt(2, pulseRisingLeftSonarInterruption, RISING);// interrupcao do sonar esquerdo

  runForward(constantForwardWalkInitFactor);
  stopCar();

  avoidSide = random(constantBooleanSize);// aleatoriza primeiro desvio
  flagAvoidObstacle = false;
  firstMeasureFront=1;
  firstMeasureRight=1;
  firstMeasureLeft=1;
  timeFrontSonar = 0;
  timeRightSonar = 0;
  timeLeftSonar = 0;
  courseTime = 0;
  leftMotorSpeedVariation=0;
  leftMotorSpeedVariation=0;
  carStatus=constantCarStatusNoObstacle;
  wallSide=constantSideRight-2;//lado direito
}

/*
** ===================================================================
 **     Method      :  void loop()
 **
 **     Description :
 **         Este método tem como função rodar o programa até estorar o 
 **         tempo definido pela constante constantCourseTimeEnd. Ao terminar 
 **         terminar ele para o carro.
 **         
 **         A cada rodada ele verifica se a flag de desvio de obstáculo
 **         foi ativada. 
 **         - Caso positivo ele para o caro e executa a tarefa
 **         de desvio pela função avoidObstacle(), a qual recebe o lado
 **         desejado como parâmetro, e troca o lado do próximo desvio que
 **         possa futurament ocorrer
 **         - Caso negativo ele "trigga" o ultrassom, cujo o eco vem por
 **         interrupção, e continua a executar seu objetivo.
 ** ===================================================================
 */

void loop() {
  if (courseTime <= constantCourseTimeEnd) {
    switch(carStatus){
    case constantCarStatusStart:

      break;

    case constantCarStatusNoObstacleStart:

      break;

    case constantCarStatusNoObstacle:
      runForward(constantForwardWalkFactor);// segue para o objetivo
      //delay(100);
      //stopCar();
      //checkSide(wallSide, true);
      //checkSide(!wallSide, false);
      break;

    case constantCarStatusAvoidObstacle:
      avoidObstacle(!wallSide);// desvia de obstáculo pelo lado indicado TODO: alterar este método
      break;
    }
    courseTime = millis();
  }
  stopCar();
}

boolean isFirstMeasure(int side){
  switch (side) {
  case constantSideFront:
    return firstMeasureFront;
    break;
  case constantSideRight:
    return firstMeasureRight;
    break;
  case constantSideLeft:
    return firstMeasureLeft;
    break;
  case constantSideBack:
    break;
  default:
    break;
  }
  return true;
}

int getLastDistance(int side){
  switch (side) {
  case constantSideFront:
    return lastDistanceFront;
    break;
  case constantSideRight:
    return lastDistanceRight;
    break;
  case constantSideLeft:
    return lastDistanceLeft;
    break;
  }
}

void setLastDistance(int side, int distance){
  switch (side) {
  case constantSideFront:
    lastDistanceFront=distance;
    break;
  case constantSideRight:
    lastDistanceRight=distance;
    break;
  case constantSideLeft:
    lastDistanceLeft=distance;
    break;
  }
}

void incrementSideMotorSpeedVariation(int side, int value){
  if(side==constantSideRight){
    if(leftMotorSpeedVariation+leftMotorSpeed>value*10){
      leftMotorSpeedVariation=leftMotorSpeedVariation-value*10;
    }
    if((rightMotorSpeedVariation+value*10+rightMotorSpeed) < constantMaximumSpeed){
      rightMotorSpeedVariation=rightMotorSpeedVariation+value*10;
    }
  }
  else{
    if(leftMotorSpeedVariation+value*10+leftMotorSpeed<constantMaximumSpeed){
      leftMotorSpeedVariation=leftMotorSpeedVariation+value*10;
    }
    if(rightMotorSpeedVariation+rightMotorSpeed>value*10){
      rightMotorSpeedVariation=rightMotorSpeedVariation-value*10;
    }
  }
}

void decrementSideMotorSpeedVariation(int side, int value){
  if(side==constantSideRight){
    if(leftMotorSpeedVariation+value+leftMotorSpeed<constantMaximumSpeed){
      leftMotorSpeedVariation=leftMotorSpeedVariation+value;
    }
    if(rightMotorSpeedVariation+rightMotorSpeed>value){
      rightMotorSpeedVariation=rightMotorSpeedVariation-value;
    }
  }
  else{
    if(leftMotorSpeedVariation+leftMotorSpeed>value){
      leftMotorSpeedVariation=leftMotorSpeedVariation-value;
    }
    if(rightMotorSpeedVariation+value+rightMotorSpeed<constantMaximumSpeed){
      rightMotorSpeedVariation=rightMotorSpeedVariation+value;
    }
  }
}

boolean checkFront(){// retorna se existe obstaculo
  return (sonarFrontDistance()<=constantMaximumObjectDistance);
}

void checkSide(int side, boolean wallSide){// retorna se existe obstaculo
  int newDistance=sonarDistance(side);
  int lastDistance=getLastDistance(side);
  
  //if(isFirstMeasure(side)){
  if(wallSide){
    if(lastDistance<newDistance){
      decrementSideMotorSpeedVariation(side,newDistance-lastDistance);
    }
    if(lastDistance>newDistance){
      incrementSideMotorSpeedVariation(side,lastDistance-newDistance);
    }
    if(lastDistance==newDistance){
      if(newDistance>=constantMaximumObjectDistance){
        incrementSideMotorSpeedVariation(side,10);//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////Teste de constante
      }

      if(newDistance<=constantMinimumObjectDistance){
        decrementSideMotorSpeedVariation(side,10);//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////Teste de constante
      }
    }
  }
  setLastDistance(side, newDistance);
}

void checkSideByInterruption(int side, boolean wallSide){// retorna se existe obstaculo
  int newDistance=sonarDistanceByInterruption(side);
  int lastDistance=getLastDistance(side);
  
  //if(isFirstMeasure(side)){
  if(wallSide){
    if(lastDistance<newDistance){
      decrementSideMotorSpeedVariation(side,newDistance-lastDistance);
    }
    if(lastDistance>newDistance){
      incrementSideMotorSpeedVariation(side,lastDistance-newDistance);
    }
    if(lastDistance==newDistance){
      if(newDistance>=constantMaximumObjectDistance){
        incrementSideMotorSpeedVariation(side,10);//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////Teste de constante
      }

      if(newDistance<=constantMinimumObjectDistance){
        decrementSideMotorSpeedVariation(side,10);//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////Teste de constante
      }
    }
  }
  setLastDistance(side, newDistance);
}

/*
** ===================================================================
 **     Method      :  int getSideTriggerPin( int side )
 **
 **     Description :
 **         Este método tem como função retornar o pino de trigger do
 **         sonar do lado especificado pela variável de entrada side.
 ** ===================================================================
 */

int getSideTriggerPin(int side){
  switch(side){
    case constantSideRight:  return constantRightSonarTriggerPin;  break;
    case constantSideLeft:  return constantLeftSonarTriggerPin;  break;
    case constantSideFront:  return constantFrontSonarTriggerPin;  break;
  }
}

/*
** ===================================================================
 **     Method      :  int getSideEchoPin( int side )
 **
 **     Description :
 **         Este método tem como função retornar o pino de echo do
 **         sonar do lado especificado pela variável de entrada side.
 ** ===================================================================
 */

int getSideEchoPin(int side){
  switch(side){
    case constantSideRight:  return constantRightSonarEchoPin;  break;
    case constantSideLeft:  return constantLeftSonarEchoPin;  break;
    case constantSideFront:  return constantFrontSonarEchoPin;  break;
  }
}

/*
** ===================================================================
 **     Method      :  void pulseFallingSonarInterruption()
 **
 **     Description :
 **         Este método tem como função calcular e tratar o sinal de echo
 **         do sonar frontal. Esta função é ativada por interrupção.
 ** ===================================================================
 */

void pulseFallingFrontSonarInterruption() {
  long duration, distance = 0;
  unsigned long timeSonarEnd = micros();
  unsigned long timeSonarTotal = timeSonarEnd - timeFrontSonar;

  distance = microsecondsToCentimeters(timeSonarTotal);
  //Serial.println(distance);
  if ((distance <= constantMaximumObjectDistance)){//
    stopCar();
    carStatus=constantCarStatusAvoidObstacle;
  }
  attachInterrupt(1, pulseRisingFrontSonarInterruption, RISING);
}

void pulseFallingRightSonarInterruption() {////////////////////////////////////////////////////////////TODO
  long duration, distance = 0;
  unsigned long timeSonarEnd = micros();
  unsigned long timeSonarTotal = timeSonarEnd - timeRightSonar;

  distance = microsecondsToCentimeters(timeSonarTotal);
  //Serial.println(distance);
  if ((distance <= constantMaximumObjectDistance)){//
    //stopCar();
    //carStatus=constantCarStatusAvoidObstacle;
  }
  attachInterrupt(0, pulseRisingRightSonarInterruption, RISING);
}

void pulseFallingLeftSonarInterruption() {
  long duration, distance = 0;
  unsigned long timeSonarEnd = micros();
  unsigned long timeSonarTotal = timeSonarEnd - timeLeftSonar;

  distance = microsecondsToCentimeters(timeSonarTotal);
  //Serial.println(distance);
  if ((distance <= constantMaximumObjectDistance)){//
    //stopCar();
    //carStatus=constantCarStatusAvoidObstacle;
  }
  //attachInterrupt(2, pulseRisingLeftSonarInterruption, RISING);
}

int sonarDistanceByInterruption(int  side){
  int distance=0;
  //switch(side){
    //case 
  //}
  Serial.print("sonar");
  Serial.print(side);
  Serial.print(":");
  Serial.println(distance);
  return distance;
}

int sonarDistance(int  side){
  sendSonarTrigger(getSideTriggerPin(side));
  int duration = pulseIn(getSideEchoPin(side), HIGH);
  int distance =  microsecondsToCentimeters(duration);
  Serial.print("sonar");
  Serial.print(side);
  Serial.print(":");
  Serial.println(distance);
  return distance;
}

int sonarFrontDistance(){
  sendSonarTrigger(constantFrontSonarTriggerPin);
  int duration = pulseIn(constantFrontSonarEchoPin, HIGH);
  return microsecondsToCentimeters(duration);
}

/*
** ===================================================================
 **     Method      :  void courseObstacleWithHole(boolean  side, int walkUnit)
 **
 **     Description :
 **         Este método tem como função percorrer um obstáculo até sua
 **         borda. Cujo o lado e unidade de caminho de loop são especificados
 **         em suas variáveis de entrada.
 **         Composta de dois loop's cujo o primeiro percorre até encontrar
 **         o obstáculo, e o segundo cujo o objetivo é percorrer até o
 **         fim do obstáculo encontrado.
 **         Em ambos os loop's são tratados o envio e recebimento do sinal
 **         dos sonares,
 ** ===================================================================
 */

int courseObstacleWithHole(int  side, int walkUnit){
  int triggerPin=getSideTriggerPin(side);
  int echoPin=getSideEchoPin(side);
  int distance=0;
  int courseObstacleTimes=0;

  do{
    sendSonarTrigger(triggerPin);
    int duration = pulseIn(echoPin, HIGH);
    distance = microsecondsToCentimeters(duration);

    runForward(constantForwardWalkFactor); 
    delay(walkUnit);
    courseObstacleTimes++;
    stopCar();
  }
  while(distance > constantMaximumObjectDistance); 

  do{
    sendSonarTrigger(triggerPin);
    int duration = pulseIn(echoPin, HIGH);
    distance = microsecondsToCentimeters(duration);

    runForward(constantForwardWalkFactor); 
    delay(walkUnit);
    courseObstacleTimes++;
    stopCar();
  }
  while(distance <= constantMaximumObjectDistance*constantMaximumObjectDistanceTimesFactor); 

  return courseObstacleTimes;
}

/*
** ===================================================================
 **     Method      :  void repeatWalk(int courseObstacleTimes, int walkUnit)
 **
 **     Description :
 **         Este método tem como função voltar a posição inicial horizontal
 **         do momento em que se encontrou o obstáculo, a partir do tamanho
 **         do passo utilizado e quantas vezes foi utilizado pela função que
 **         o percorreu, as quais são suas variáveis de entrada.
 ** ===================================================================
 */

void repeatWalk(int courseObstacleTimes, int walkUnit){
  for(int i=0;i<walkUnit;i++){
    runForward(constantForwardWalkFactor); 
    delay(courseObstacleTimes);
    stopCar();
  }
}

/*
** ===================================================================
 **     Method      :  void turnSide(boolean side)
 **
 **     Description :
 **         Este método tem como função virar 90 graus para o lado especificado
 **         em sua variável de entrada.
 ** ===================================================================
 */

void turnSide(int side){
  if(side==constantSideRight){
    runSide(side, 215);
    delay(constantTurnRightDelayFactor);
    stopCar();
  }
  else{
    runSide(side, 215);
    delay(constantTurnLeftDelayFactor);
    stopCar();
  }
}

/*
** ===================================================================
 **     Method      :  void avoidObstacle(boolean  side)
 **
 **     Description :
 **         Este método tem como função contornar o obstáculo encontrado
 **         pelo lado especificado em sua variável de entrada.
 **         Percorrendo inicialmente sua largura a partir do ponto de 
 **         encontro, salvando-a em termos de uma unidade de percurso,
 **         depois seu comprimento e ao final percorre sua largura até
 **         o ponto horizontal de início, a partir da variável de percurso
 **         salva. Completando assim seu desvio.
 ** ===================================================================
 */

void avoidObstacle(int  side){
  int courseObstacleTimes=0;
  int walkUnit=constantForwardWalkFactor;
  turnSide(side);
}

/*
** ===================================================================
 **     Method      :  void pulseRisingSonarInterruption()
 **
 **     Description :
 **         Este método tem como função tratar o inicio do sinal de echo
 **         do sonar frontal, iniciar o timer e ativar a interrupção de 
 **         fim e calculo do sinal. Esta função é ativada por interrupção.
 ** ===================================================================
 */

void pulseRisingFrontSonarInterruption() {
  timeFrontSonar = micros();
  attachInterrupt(1, pulseFallingFrontSonarInterruption, FALLING);
}

void pulseRisingRightSonarInterruption() {
  timeRightSonar = micros();
  attachInterrupt(0, pulseFallingRightSonarInterruption, FALLING);
}

void pulseRisingLeftSonarInterruption() {
  //timeLeftSonar = micros();
  //attachInterrupt(2, pulseFallingLeftSonarInterruption, FALLING);
}

/*
** ===================================================================
 **     Method      :  void sendSonarTrigger(int triggerPin)
 **
 **     Description :
 **         Este método tem como função enviar sinal de trigger para o
 **         o pino de trigger (sonar) especificado por sua variável de entrada.
 ** ===================================================================
 */

void sendSonarTrigger(int triggerPin) {
  digitalWrite(triggerPin, LOW); 
  delayMicroseconds(2); 

  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(constantSonarTriggerDelay);

  digitalWrite(triggerPin, LOW);
}

/*
** ===================================================================
 **     Method      :  long microsecondsToCentimeters(long microseconds)
 **
 **     Description :
 **         Este método tem como função tranformar o sinal de echo do sonar,
 **         a qual seu tempo é a variável de entrada, em centímetros e o retornar. 
 ** ===================================================================
 */

long microsecondsToCentimeters(long microseconds) {
  return microseconds / constantMicrosecondsToCentimetersFactor;
}

/*
** ===================================================================
 **     Method      :  long microsecondsToCentimeters(long microseconds)
 **
 **     Description :
 **         Este método tem como função fazer o carro andar para o lado
 **         especificado em sua variável de entrada.
 ** ===================================================================
 */

void run(int side) {
  switch (side) {
  case 1:
    runForward();
    break;
  case 2:
    runRight();
    break;
  case 3:
    runLeft();
    break;
  case 4:
    runBackward();
    break;
  default:
    stopCar();
    break;
  }

}

/*
** ===================================================================
 **     Method      :  void stopCar()
 **
 **     Description :
 **         Este método tem como função fazer o carro parar de andar.
 ** ===================================================================
 */

void stopCar() {
  digitalWrite(constantRightMotorPin0, LOW);
  digitalWrite(constantRightMotorPin1, LOW);
  digitalWrite(constantLeftMotorPin0, LOW);
  digitalWrite(constantLeftMotorPin1, LOW);
  goingForward=0;
  goingLeft=0;
  goingRight=0;
  goingBackward=0;
}

/*
** ===================================================================
 **     Method      :  void runForward()
 **
 **     Description :
 **         Este método tem como função fazer o carro andar para frente.
 ** ===================================================================
 */

void runForward() {
  digitalWrite(constantRightMotorPin0, HIGH);
  digitalWrite(constantRightMotorPin1, LOW);
  digitalWrite(constantLeftMotorPin0, HIGH);
  digitalWrite(constantLeftMotorPin1, LOW);
  goingForward=1;
  goingLeft=0;
  goingRight=0;
  goingBackward=0;
}

/*
** ===================================================================
 **     Method      :  void runBackward()
 **
 **     Description :
 **         Este método tem como função fazer o carro andar de ré.
 ** ===================================================================
 */

void runBackward() {
  digitalWrite(constantRightMotorPin0, LOW);
  digitalWrite(constantRightMotorPin1, HIGH);
  digitalWrite(constantLeftMotorPin0, LOW);
  digitalWrite(constantLeftMotorPin1, HIGH);
  goingForward=0;
  goingLeft=0;
  goingRight=0;
  goingBackward=1;
}

/*
** ===================================================================
 **     Method      :  void runRight()
 **
 **     Description :
 **         Este método tem como função fazer o carro dobrar para direita.
 ** ===================================================================
 */

void runRight() {
  digitalWrite(constantRightMotorPin0, HIGH);
  digitalWrite(constantRightMotorPin1, LOW);
  digitalWrite(constantLeftMotorPin0, LOW);
  digitalWrite(constantLeftMotorPin1, HIGH);
  goingForward=0;
  goingLeft=0;
  goingRight=1;
  goingBackward=0;
}

/*
** ===================================================================
 **     Method      :  void runLeft()
 **
 **     Description :
 **         Este método tem como função fazer o carro dobrar para esquerda.
 ** ===================================================================
 */

void runLeft() {
  digitalWrite(constantRightMotorPin0, LOW);
  digitalWrite(constantRightMotorPin1, HIGH);
  digitalWrite(constantLeftMotorPin0, HIGH);
  digitalWrite(constantLeftMotorPin1, LOW);
  goingForward=0;
  goingLeft=1;
  goingRight=0;
  goingBackward=0;
}

/*
** ===================================================================
 **     Method      :  void enableMotors(int speed)
 **
 **     Description :
 **         Este método tem como função ativar o pwm em ambos os motores
 **         na velocidade especificada em sua variável de entrada.
 ** ===================================================================
 */

void enableMotors(int speed){
  enableLeftMotor(speed);
  enableRightMotor(speed);
}

/*
** ===================================================================
 **     Method      :  void enableLeftMotor(int speed)
 **
 **     Description :
 **         Este método tem como função ativar a recebida na variável 
 **         de entrada potência no motor esquedo.
 ** ===================================================================
 */

void enableLeftMotor(int speed){
  analogWrite(constantEnableLeftMotorPin, speed+leftMotorSpeedVariation);
  leftMotorSpeed=speed;
}

/*
** ===================================================================
 **     Method      :  void enableRightMotor(int speed)
 **
 **     Description :
 **         Este método tem como função ativar a recebida na variável 
 **         de entrada potência no motor direito.
 ** ===================================================================
 */

void enableRightMotor(int speed){
  analogWrite(constantEnableRightMotorPin, speed+rightMotorSpeedVariation);
  rightMotorSpeed=speed;
}

/*
** ===================================================================
 **     Method      :  void enableLeftMotor()
 **
 **     Description :
 **         Este método tem como função ativar toda a potência no motor
 **         esquerdo.
 ** ===================================================================
 */

void enableLeftMotor(){
  digitalWrite(constantEnableLeftMotorPin, HIGH);
  leftMotorSpeed=constantFullSpeed;
}

/*
** ===================================================================
 **     Method      :  void enableRightMotor()
 **
 **     Description :
 **         Este método tem como função ativar toda a potência no motor
 **         direito.
 ** ===================================================================
 */

void enableRightMotor(){
  digitalWrite(constantEnableRightMotorPin, HIGH);
  rightMotorSpeed=constantFullSpeed;
}

/*
** ===================================================================
 **     Method      :  void enableMotors()
 **
 **     Description :
 **         Este método tem como função ativar toda a potência em ambos
 **         os motores.
 ** ===================================================================
 */

void enableMotors(){
  enableRightMotor();
  enableLeftMotor();
}


/*
** ===================================================================
 **     Method      :  void disableLeftMotor()
 **
 **     Description :
 **         Este método tem como função desativar toda a potência no motor
 **         esquerdo.
 ** ===================================================================
 */

void disableLeftMotor(){
  digitalWrite(constantEnableLeftMotorPin, LOW);
  leftMotorSpeed=0;
}

/*
** ===================================================================
 **     Method      :  void disableRightMotor()
 **
 **     Description :
 **         Este método tem como função desativar toda a potência no motor
 **         direito.
 ** ===================================================================
 */

void disableRightMotor(){
  digitalWrite(constantEnableRightMotorPin, LOW);
  rightMotorSpeed=0;
}

/*
** ===================================================================
 **     Method      :  void disableMotors()
 **
 **     Description :
 **         Este método tem como função travar ambos os motores.
 ** ===================================================================
 */

void disableMotors(){
  disableLeftMotor();
  disableRightMotor();
}

/*
** ===================================================================
 **     Method      :  void runForward(int speed)
 **
 **     Description :
 **         Este método tem como função fazer o carro andar para frente
 **         na velocidade especificada em sua variável de entrada.
 ** ===================================================================
 */

void runForward(int speed) {
  enableMotors(speed);
  runForward();
}

/*
** ===================================================================
 **     Method      :  void runBackward(int speed)
 **
 **     Description :
 **         Este método tem como função fazer o carro andar de ré
 **         na velocidade especificada em sua variável de entrada.
 ** ===================================================================
 */

void runBackward(int speed) {
  enableMotors(speed);
  runBackward();
}

/*
** ===================================================================
 **     Method      :  void runSide(boolean side, int speed)
 **
 **     Description :
 **         Este método tem como função fazer o carro dobrar para o lado
 **         especificado em sua variável de entrada. E na velocidade dada
 **         por sua outra variavel de entrada.
 ** ===================================================================
 */

void runSide(int side, int speed) {
  enableMotors(speed);
  switch(side){
    case  constantSideRight:  runRight();  break;
    case  constantSideLeft:  runLeft();  break;
    case  constantSideFront:  runForward();  break;
    case  constantSideBack:  runBackward();  break;
    case  constantSideNoSide:  stopCar();  break;
  }
}

/*
** ===================================================================
 **     Method      :  void runRight(int speed)
 **
 **     Description :
 **         Este método tem como função fazer o carro dobrar para direita
 **         na velocidade especificada em sua variável de entrada.
 ** ===================================================================
 */

void runRight(int speed) {
  enableMotors(speed);

  runRight();
}

/*
** ===================================================================
 **     Method      :  void runLeft(int speed)
 **
 **     Description :
 **         Este método tem como função fazer o carro dobrar para esquerda
 **         na velocidade especificada em sua variável de entrada.
 ** ===================================================================
 */

void runLeft(int speed) {
  enableMotors(speed);

  runRight();
}

/*
** ===================================================================
 **     Method      :  void stopCar(int speed)
 **
 **     Description :
 **         Este método tem como função fazer o carro parar de andar.
 **         travando a roda pelo nivel especificado em sua variável de 
 **         entrada.
 ** ===================================================================
 */

void stopCar(int speed) {
  enableMotors(speed);

  stopCar();
}

/*
** ===================================================================
 **     Method      :  void runFullSpeedForward()
 **
 **     Description :
 **         Este método tem como função fazer o carro andar para frente
 **         em potência máxima.
 ** ===================================================================
 */

void runFullSpeedForward() {
  enableMotors();

  runForward();
}

/*
** ===================================================================
 **     Method      :  void runFullSpeedBackward()
 **
 **     Description :
 **         Este método tem como função fazer o carro andar de ré
 **         em potência máxima.
 ** ===================================================================
 */

void runFullSpeedBackward() {
  enableMotors();

  runBackward();
}

/*
** ===================================================================
 **     Method      :  void runFullSpeedRight()
 **
 **     Description :
 **         Este método tem como função fazer o carro dobrar à direita
 **         em potência máxima.
 ** ===================================================================
 */

void runFullSpeedRight() {
  enableMotors();

  runRight();
}

/*
** ===================================================================
 **     Method      :  void runFullSpeedLeft()
 **
 **     Description :
 **         Este método tem como função fazer o carro dobrar à esquerda
 **         em potência máxima.
 ** ===================================================================
 */

void runFullSpeedLeft() {
  enableMotors();

  runRight();
}

/*
** ===================================================================
 **     Method      :  void stopCarFullSpeed()
 **
 **     Description :
 **         Este método tem como função fazer o carro parar e travar em
 **         potência máxima.
 ** ===================================================================
 */

void stopCarFullSpeed() {
  disableMotors();

  stopCar();
}


