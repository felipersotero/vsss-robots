//ESP_NOW na configuração escravo
//Adaptado por: Alan Fernandes Soares da Silva
//Última atualização: 15/03/2024
//Fonte: https://dronebotworkshop.com/esp-now/

// Incluindo bibliotecas
#include <esp_now.h>
#include <WiFi.h>
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

// Variáveis para armazenar fatias das strings de dados
char data_command[2];
char data_wr[5];
char data_wl[5];

char command_value;
int wr, wl;

char aux[9];

// Estrutura de dados
typedef struct struct_message {
  char data[9];
} struct_message;
 
// Objeto de dados que serão enviados 
struct_message myData;

// Função de retorno
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));

  // aux = myData.a;
  strncpy(aux, myData.data, sizeof(myData.data));

  // Fatiando dados
  strncpy(data_command, aux, 1);
  data_command[1] = '\0';

  strncpy(data_wr, aux + 1, 4);
  data_wr[4] = '\0';

  strncpy(data_wl, aux + 5, 4);
  data_wl[4] = '\0';

  command_value = data_command[0];
  wr = atoi(data_wr);
  wl = atoi(data_wl);

  Serial.println(command_value);
  Serial.println(wr);
  Serial.println(wl);

  if(command_value == 'f'){
    move(wr,wl);
  } else{
    stopMotors();
  }

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

void stopMotors() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, LOW);
}

void setup() {
  // Inicialização da comunicação serial
  Serial.begin(115200);
 
  // Inicialização da estação wifi
  WiFi.mode(WIFI_STA);
 
  // Inicialização do ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Erro ao inicializar ESP-NOW");
    return;
  }
 
  // Registrando a função de retorno
  esp_now_register_recv_cb(OnDataRecv);

  //Definindo pinos
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);

  ledcSetup(channelA, 5000, 8);
  ledcAttachPin(motorAenable, channelA);
  ledcSetup(channelB, 5000, 8);
  ledcAttachPin(motorBenable, channelB);

  stopMotors();
}
 
void loop() {
 
}