#include "config.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <cstdlib>

// Define os pinos conectados à ponte H
const int motorA1 = 2;
const int motorA2 = 4;
const int motorAenable = 15;
const int channelA = 0;

const int motorB1 = 25;
const int motorB2 = 26;
const int motorBenable = 32;
const int channelB = 1;

const int delay_time = 25;

//Dado recebido pelo MQTT
char data[15];

// Criar variáveis para armazenar partes da string
char data_command[2];
char data_angle[8];
char data_distance[7];

char commandValue;
int velocityValue, delayValue;
float angle_value, distance_value;


//PID
int max_velocity = 255; 
int min_velocity = -255;
int min_velocity_bin = 100;
float kp_angle = 3;
float kp_dist = 2;
unsigned long Time ;
unsigned long timePrev = millis() ;
float dt = 200;
float erroAngle ;
float erroDist ; 
int velocity;
float setpoint = 0;
void PID(float angle_value, float distance_value, float kp_angle, float kp_dist);
int limitador(int velocidade);
float k_dist[3] = {8.5,-8.8,0.4};
float k_angle[3] = {7,-7.6,0.7};

// Informações da rede Wi-Fi
const char *ssid = WIFI_SSID;
const char *password = WIFI_PASSWORD;

// Informações do broker MQTT
const char *mqttBroker = MQTT_BROKER;
const int mqttPort = MQTT_PORT;
const char *mqttTopic = MQTT_TOPIC;

// Objeto para a conexão Wi-Fi
WiFiClient espClient;

// Objeto para o cliente MQTT
PubSubClient client(espClient);

// Função callback chamada quando uma mensagem MQTT é recebida
void callback(char *topic, byte *payload, unsigned int length) {
  Serial.println("Mensagem recebida no tópico: " + String(topic));
  Serial.print("Conteúdo: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    data[i] = (char)payload[i];
  }
  Serial.println();
}

void connectToMQTT(){
    if (client.connect("robot-1")) {
      Serial.println("Reconectado ao broker MQTT!");
      client.subscribe(mqttTopic);
    } else {
      Serial.print(client.state());
      Serial.println("Falha na reconexão. Tentando novamente em 5 segundos...");
      delay(5000);
    }
}

void setup() {
  Serial.begin(115200);

  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);

  ledcSetup(channelA, 5000, 8);
  ledcAttachPin(motorAenable, channelA);
  ledcSetup(channelB, 5000, 8);
  ledcAttachPin(motorBenable, channelB);

  stopMotors();

  // Conexão à rede Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Conectando ao WiFi...");
  }

  Serial.println("Conectado ao WiFi!");

  // Configuração do broker MQTT
  client.setServer(mqttBroker, mqttPort);
  client.setCallback(callback);

  // Conexão ao broker MQTT
  while (!client.connected()) {
    Serial.println("Conectando ao broker MQTT...");
    connectToMQTT();
  }
}

void loop() {
  // Verificação da conexão com o broker MQTT
  if (!client.connected()) {
    Serial.println("Conexão perdida. Tentando reconectar...");
    connectToMQTT();
  }

  // Verificação de mensagens MQTT recebidas
  client.loop();


  // Captura o tempo atual em milissegundos
  Time = millis();

  // Verifica se o tempo decorrido desde a última iteração é maior que o intervalo desejado (dt)
  if ((Time - timePrev) > dt) {
      // Atualiza o tempo da última iteração para o tempo atual
      timePrev = millis();

      // readData(commandValue, velocityValue, delayValue);
      readData(commandValue, angle_value, distance_value);


      Serial.println("Executando ação!");
      Serial.println(data_command);
      Serial.println(data_command[0]);
      Serial.println(commandValue);
      Serial.println(angle_value);
      Serial.println(distance_value);

      if(commandValue == 's'){
        stopMotors();
        Serial.println("Motores parados!");
        delay(100);
      } else{
        // Chama a função PID
        controle(angle_value, distance_value);        
        //PID(angle_value, distance_value , kp_angle, kp_dist);        
      }

  }
}

void readData(char &commandValue, float &angle_value, float &distance_value){
  strncpy(data_command, data, 1);
  data_command[1] = '\0';

  strncpy(data_angle, data + 1, 7);
  data_angle[7] = '\0';

  strncpy(data_distance, data + 8, 6);
  data_distance[6] = '\0';

  // Converter as partes para inteiros
  commandValue = data_command[0];
  angle_value = atof(data_angle);
  distance_value = atof(data_distance);



}

void moveForward(int speed) {
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  ledcWrite(channelA, speed);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
  ledcWrite(channelB, speed);
}

void turnRight(int speed) {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  ledcWrite(channelA, speed);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
  ledcWrite(channelB, speed);
}

void turnLeft(int speed) {
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  ledcWrite(channelA, speed);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);
  ledcWrite(channelB, speed);
}

void moveBackward() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);
}


void stopMotors() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, LOW);
}

void move(int speedA, int speedB) {
  if(speedA> 0 && speedB> 0){
    digitalWrite(motorA1, HIGH);
    digitalWrite(motorA2, LOW);
    ledcWrite(channelA, speedA);
    digitalWrite(motorB1, HIGH);
    digitalWrite(motorB2, LOW);
    ledcWrite(channelB, speedB);
    Serial.println("Frente");
  } else if(speedA > 0 && speedB < 0){
    speedB = abs(speedB);
    digitalWrite(motorA1, HIGH);
    digitalWrite(motorA2, LOW);
    ledcWrite(channelA, speedA);
    digitalWrite(motorB1, LOW);
    digitalWrite(motorB2, HIGH);
    ledcWrite(channelB, speedB);
    Serial.println("Gira pra esquerda");
  }else if(speedA < 0 && speedB > 0){
    speedA = abs(speedA);
    digitalWrite(motorA1, LOW);
    digitalWrite(motorA2,  HIGH);
    ledcWrite(channelA, speedA);
    digitalWrite(motorB1, HIGH);
    digitalWrite(motorB2, LOW );
    ledcWrite(channelB, speedB);
    Serial.println("Gira pra direita");
  }else if(speedA < 0 && speedB <0){
    speedA = abs(speedA);
    speedB = abs(speedB);
    digitalWrite(motorA1, LOW);
    digitalWrite(motorA2, HIGH);
    ledcWrite(channelA, speedA);
    digitalWrite(motorB1, LOW);
    digitalWrite(motorB2, HIGH);
    ledcWrite(channelB, speedB);
    Serial.println("Anda de ré");
  }
}


// A faixa de saída é limitada entre -255 e -150, e entre 150 e 255.
int limitador(int velocidade) {
    if (velocidade >= max_velocity) { // Verifica se a velocidade é maior ou igual à velocidade máxima permitida.
        velocidade = max_velocity;
    } 
    else if (velocidade <= min_velocity) {// Verifica se a velocidade é menor ou igual à velocidade mínima permitida.
        velocidade = min_velocity;
    } 
    else if ((velocidade <= min_velocity_bin) && (velocidade > 0)) {// Verifica se a velocidade está na faixa entre 0 e 150.
        velocidade = min_velocity_bin;
    } 
    else if ((velocidade >= -min_velocity_bin) && (velocidade < 0)) {// Verifica se a velocidade está na faixa entre -150 e 0.
        velocidade = -min_velocity_bin;
    } 
    else { // Caso a velocidade esteja dentro das faixas permitidas, não é necessário alterar o valor.
        velocidade = velocidade;
    }
    // Retorna a velocidade limitada.
    return velocidade;
}


void PID(float angle_value, float distance_value, float kp_angle, float kp_dist) {
   // Calcula o erro do ângulo subtraindo o valor desejado (setpoint) do valor atual (angle_value)
      erroAngle = setpoint - angle_value;  
    // Verifica se o erro do ângulo está dentro da faixa de -10° a 10°
    if(distance_value > 7){
      if ((erroAngle < 10) && (erroAngle > -10)) {
          // Se o ângulo estiver dentro da faixa, calcula o erro de distância
          erroDist = -distance_value; // Calcula o erro referente à distância 
          velocity = erroDist * kp_dist; // Calcula a velocidade proporcional ao erro de distância
          velocity = limitador(velocity); // Limita a velocidade
          moveForward(velocity); // Move para a frente com a velocidade calculada
      } else { 
          // Caso contrário, tenta ajustar o ângulo utilizando o erro
          velocity = erroAngle * kp_angle; // Calcula a velocidade proporcional ao erro do ângulo
          velocity = limitador(velocity); // Limita a velocidade
          if (velocity < 0) { // Se a velocidade for negativa, vira para a direita
              velocity = -velocity;
              turnRight(velocity); // Vira para a direita com a velocidade calculada
          } else { // Se a velocidade for positiva, vira para a esquerda
              turnLeft(velocity); // Vira para a esquerda com a velocidade calculada
          }
      }
    }
    else{
      stopMotors();
    }

}


void controle(float angle_value, float distance_value){

  // 𝑘𝜌 > 0; 𝑘𝛽 < 0; 𝑘𝛼 − 𝑘𝛽 > 0
  float kr = 1.5;
  float ka = 3;
  float kb = -4;

  float rho = distance_value;
  float alpha = angle_value;
  float beta = -alpha;
  
  if(rho > 5){

      float v = kr * rho;
      float w = ka*alpha + kb*beta;

      float wr = v + w/2;
      float wl = v - w/2;

      wr = limitador(wr);
      wl = limitador(wl);

      move(wr,wl);
  } else{
      stopMotors();

  }


}
