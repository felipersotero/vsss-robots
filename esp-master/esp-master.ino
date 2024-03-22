//ESP_NOW na configuração mestre
//Adaptado por: Alan Fernandes Soares da Silva
//Última atualização: 15/03/2024
//Fonte: https://dronebotworkshop.com/esp-now/
 
// Incluindo bibliotecas
#include <esp_now.h>
#include <WiFi.h>
 
// Variavel de envio
int int_value;

 
// MAC Address - Broadcast
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

//Aux
char aux[9];

// Estrutura de dados
typedef struct struct_message {
  char data[9];
} struct_message;
 
// Objeto de dados que serão enviados 
struct_message myData;
 
// Peer info
esp_now_peer_info_t peerInfo;
 
//Retorno do envio 
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Status de envio: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Enviado" : "Falha");
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
  esp_now_register_send_cb(OnDataSent);
  
  // Registro dispositivo
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
   //Pareamento     
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Falha ao parear");
    return;
  }
}
 
void loop() {
 
  //recebendo dados da serial
  if(Serial.available() != 0){

    for(int i = 0; i < 9; i ++){
      aux[i] = Serial.read();
    }
      // myData.data = aux;

      strncpy(myData.data, aux, sizeof(myData.data));

      for(int i = 0; i < sizeof(myData); i ++){
        Serial.println(myData.data[i]);
      }
      //envio da mensagem via ESP-NOW
      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, 
      sizeof(myData));

    if (result == ESP_OK) {
        Serial.println("Envio confirmado");
      }
      else {
        Serial.println("Erro no envio");
      }

  }

  delay(10);
}