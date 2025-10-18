#include <BluetoothSerial.h>

BluetoothSerial SerialBT;
const int LED = 2;

volatile int ledState = LOW;
SemaphoreHandle_t ledSemaphore;

//Tarea 1: Leer Bluetooth y actualizar estado
void bluetoothTask(void *parameter) {
  String buffer = "";
  
  while (1) {
    if (SerialBT.available()) {
      char c = SerialBT.read();
      
      if (c == '\n' || c == '\r') {
        buffer.trim();
        buffer.toUpperCase();
        
        // Actualizar variable compartida con semáforo
        if (xSemaphoreTake(ledSemaphore, portMAX_DELAY)) {
          if (buffer == "ON") {
            ledState = HIGH;
            SerialBT.println("LED: ON");
            Serial.println("Comando ON");
          } 
          else if (buffer == "OFF") {
            ledState = LOW;
            SerialBT.println("LED: OFF");
            Serial.println("Comando OFF");
          } 
          else if (buffer.length() > 0) {
            SerialBT.println("Use: ON o OFF");
          }
          xSemaphoreGive(ledSemaphore);
        }
        
        buffer = "";
      } else {
        buffer += c;
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

//Tarea 2: Actualizar LED según estado
void ledTask(void *parameter) {
  int lastState = -1;
  
  while (1) {
    // Leer variable compartida con semáforo
    if (xSemaphoreTake(ledSemaphore, portMAX_DELAY)) {
      int currentState = ledState;
      xSemaphoreGive(ledSemaphore);
      
      // Actualizar LED solo si cambió el estado
      if (currentState != lastState) {
        digitalWrite(LED, currentState);
        lastState = currentState;
        Serial.println(currentState ? "LED encendido" : "LED apagado");
      }
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  
  // Crear semáforo
  ledSemaphore = xSemaphoreCreateMutex();
  
  // Iniciar Bluetooth
  SerialBT.begin("ESP32_HVR");
  Serial.println("Bluetooth: ESP32_HVR");
  
  // Crear tareas
  xTaskCreate(bluetoothTask, "BT", 4096, NULL, 1, NULL);
  xTaskCreate(ledTask, "LED", 2048, NULL, 1, NULL);
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}
