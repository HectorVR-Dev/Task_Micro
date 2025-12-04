#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

// --- CONFIGURACIÓN DE PINES Y SENSORES ---
#define LED_PIN 2   // El pin del LED integrado
#define LDR1_PIN 35 // Pin del fotoresistor (ADC1_0)
#define LDR2_PIN 34 // Pin del fotoresistor (ADC1_3)
#define LDR3_PIN 39 // Pin del fotoresistor (ADC1_6)
#define LDR4_PIN 36 // Pin del fotoresistor (ADC1_7)

// --- CREDENCIALES Y MQTT ---
#define WIFI_SSID "XXXXXXXXXXXX"       
#define WIFI_PASSWORD "XXXXXXXXX"          
#define MQTT_SERVER "broker.hivemq.com"   
#define MQTT_PORT 1883                    

// Topics
#define MQTT_TOPIC_SUB "HectorVR/Micro/SetIntervalo" // Topic para escuchar (SUSCRIPCIÓN)
#define MQTT_TOPIC_PUB_A "HectorVR/Micro/Angulo"          // Topic para enviar datos (PUBLICACIÓN)
#define MQTT_TOPIC_PUB_P "HectorVR/Micro/Potencia"        // Topic para enviar datos (PUBLICACIÓN)
#define MQTT_TOPIC_STATUS "HectorVR/Micro/Status"    // Topic de estado

WiFiClient espClient;
PubSubClient client(espClient);

// Variables para el temporizador (enviar datos sin bloquear)
unsigned long lastMsg = 0;
int interval = 250; // Enviar cada 250ms (0.25 segundos)
int mode = 0; // Modo inicial

// ---- Filtro LowPass----
template <int order>
class LowPass {
  private:
  float a[order];
  float b[order+1];
  float omega0;
  float dt;
  bool adapt;
  float tn1 = 0;
  float x[order+1];
  float y[order+1];
  
  public:
  LowPass(float f0, float fs, bool adaptive){
    omega0 = 6.28318530718*f0;
    dt = 1.0/fs;
    adapt = adaptive;
    tn1 = -dt;
    for(int k = 0; k < order+1; k++){
      x[k] = 0;
      y[k] = 0;
    }
    setCoef();
  }
  
  void setCoef(){
    if(adapt){
      float t = micros()/1.0e6;
      dt = t - tn1;
      tn1 = t;
    }
    float alpha = omega0*dt;
    if(order==1){
      a[0] = -(alpha - 2.0)/(alpha+2.0);
      b[0] = alpha/(alpha+2.0);
      b[1] = alpha/(alpha+2.0);
    }
    if(order==2){
      float alphaSq = alpha*alpha;
      float beta[] = {1, sqrt(2), 1};
      float D = alphaSq*beta[0] + 2*alpha*beta[1] + 4*beta[2];
      b[0] = alphaSq/D;
      b[1] = 2*b[0];
      b[2] = b[0];
      a[0] = -(2*alphaSq*beta[0] - 8*beta[2])/D;
      a[1] = -(beta[0]*alphaSq - 2*beta[1]*alpha + 4*beta[2])/D;
    }
  }
  
  float filt(float xn){
    if(adapt){ setCoef(); }
    y[0] = 0;
    x[0] = xn;
    for(int k = 0; k < order; k++){
      y[0] += a[k]*y[k+1] + b[k]*x[k];
    }
    y[0] += b[order]*x[order];
    for(int k = order; k > 0; k--){
      y[k] = y[k-1];
      x[k] = x[k-1];
    }
    return y[0];
  }
};

// Filtro paso bajo de orden 2, fc=3Hz, fs=1000Hz, adaptativo
LowPass<2> f_LDR1(0.5, 1, true); 
LowPass<2> f_LDR2(0.5, 1, true);
LowPass<2> f_LDR3(0.5, 1, true);
LowPass<2> f_LDR4(0.5, 1, true);

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Mensaje Recibido [");
  Serial.print(topic);
  Serial.print("]: ");

  String msg;
  char statusMsg[50];

  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }
  Serial.println(msg);
  
  if (String(topic) == MQTT_TOPIC_SUB) {
    int newInterval = msg.toInt();
    if (newInterval > 250) { // Mínimo 250 ms
      interval = newInterval;
      snprintf(statusMsg, sizeof(statusMsg), "Intervalo actualizado a %d ms", interval);
      client.publish(MQTT_TOPIC_STATUS, statusMsg); 
    } else {
      snprintf(statusMsg, sizeof(statusMsg), "Intervalo demasiado bajo, debe ser al menos 250 ms");
      client.publish(MQTT_TOPIC_STATUS, statusMsg); 
    }
  }
  

}

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Conectando a ");
  Serial.print(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); 
    delay(500);
    Serial.print(".");
  }

  digitalWrite(LED_PIN, LOW);
  Serial.println("");
  Serial.println("WiFi conectado");
  Serial.print("Dirección IP: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) { 
    Serial.print("Intentando conexión MQTT...");
    // ID de cliente único, usa uno random o fijo
    if (client.connect("ESP32Client_HectorVR_Micro")) {
      Serial.println("conectado");
      // Suscribirse solo al topic de comandos
      client.subscribe(MQTT_TOPIC_SUB); 
      
      // Publicar mensaje de estado inicial
      client.publish(MQTT_TOPIC_STATUS, "Estaca Conectada"); 
    } else {
      Serial.print("Fallo, rc=");
      Serial.print(client.state());
      Serial.println(" intentando en 5 segundos");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200); 
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(LDR1_PIN, INPUT);
  pinMode(LDR2_PIN, INPUT);
  pinMode(LDR3_PIN, INPUT);
  pinMode(LDR4_PIN, INPUT);

  setup_wifi(); 
  client.setServer(MQTT_SERVER, MQTT_PORT); 
  client.setCallback(callback); 
}

void loop() {
  if (!client.connected()) { 
    reconnect(); 
  }
  client.loop(); // IMPORTANTE: Mantiene vivo el MQTT

  // --- BLOQUE DE ENVÍO DE SENSOR ---
  unsigned long now = millis();
  if (now - lastMsg > interval) {
    lastMsg = now;
    
    // 1. Leer sensores
    int LDR1 = analogRead(LDR1_PIN); // Valor entre 0 y 4095
    int LDR2 = analogRead(LDR2_PIN);
    int LDR3 = analogRead(LDR3_PIN);
    int LDR4 = analogRead(LDR4_PIN);

    int LDR1_F = f_LDR1.filt(LDR1); // Valor entre 0 y 4095
    int LDR2_F = f_LDR2.filt(LDR2);
    int LDR3_F = f_LDR3.filt(LDR3);
    int LDR4_F = f_LDR4.filt(LDR4);
    
    /*
    Serial.print(">LDR1:");
    Serial.println(LDR1);
    Serial.print(">LDR1_F:");
    Serial.println(LDR1_F);
    
    Serial.print(">LDR2:");
    Serial.println(LDR2);
    Serial.print(">LDR2_F:");
    Serial.println(LDR2_F);
    
    Serial.print(">LDR3:");
    Serial.println(LDR3);
    Serial.print(">LDR3_F:");
    Serial.println(LDR3_F);
    
    Serial.print(">LDR4:");
    Serial.println(LDR4);
    Serial.print(">LDR4_F:");
    Serial.println(LDR4_F);
    */
    
    float N1 = LDR1_F / 4095.0;
    float N2 = LDR2_F / 4095.0;
    float N3 = LDR3_F / 4095.0;
    float N4 = LDR4_F / 4095.0;

    float S = N1 + N2 + N3 + N4;
    
    if (S < 0.0001) S = 0.0001; 

    float vX = ((N2 + N3) - (N1 + N4)) / S;
    float vY = ((N4 + N3) - (N1 + N2)) / S;

    // ángulo y magnitud
    float angle = atan2(vY, vX) * 180.0 / PI;
    float intensity = S * 25; // Promedio normalizado
  
    char msgBufferA[10];
    snprintf(msgBufferA, 10, "%d", (int)angle); // Convierte el entero a texto
    client.publish(MQTT_TOPIC_PUB_A, msgBufferA);

    char msgBufferP[10];
    snprintf(msgBufferP, 10, "%d", (int)intensity); // Convierte el entero a texto
    client.publish(MQTT_TOPIC_PUB_P, msgBufferP);
  }
}
