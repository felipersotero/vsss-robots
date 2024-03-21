//ESP_NOW na configuração escravo
//Adaptado por: Alan Fernandes Soares da Silva
//Última atualização: 15/03/2024
//Fonte: https://dronebotworkshop.com/esp-now/

// Incluindo bibliotecas
#include <esp_now.h>
#include <WiFi.h>
#include <cstdlib>

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

  if(strcmp(aux, "f+255+255") == 0){
    digitalWrite(2,HIGH);
  }else if(strcmp(aux, "s+255+255") == 0){
    digitalWrite(2,LOW);
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
  pinMode(2,OUTPUT);
}
 
void loop() {
 
}