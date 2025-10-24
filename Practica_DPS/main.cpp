#include <Arduino.h>

const int POT_PIN = 34;              // GPIO34 (ADC1_CH6)
const int PERIODO_MUESTREO = 5; //Milisegundos

// ========== FILTRO DE MEDIA MÓVIL ==========
const int NUM_LECTURAS = 20;
int lecturas[NUM_LECTURAS];
int indiceLectura = 0;
int total = 0;
int promedio = 0;

// ========== VARIABLES GLOBALES ==========
unsigned long ultimaLectura = 0;

// ========== FUNCIONES ==========

void inicializarFiltro() {
  for (int i = 0; i < NUM_LECTURAS; i++) {
    lecturas[i] = 0;
  }
  total = 0;
  indiceLectura = 0;
}

int leerADCFiltrado(int pin) {
  // Restar la lectura más antigua
  total = total - lecturas[indiceLectura];
  
  // Leer nuevo valor
  lecturas[indiceLectura] = analogRead(pin);
  
  // Sumar la nueva lectura
  total = total + lecturas[indiceLectura];
  
  // Avanzar al siguiente índice
  indiceLectura = (indiceLectura + 1) % NUM_LECTURAS;
  
  // Calcular promedio
  return total / NUM_LECTURAS;
}


// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  while (!Serial) {;}
  analogReadResolution(12);
  
  // Configurar atenuación del ADC para rango completo 0-3.3V
  analogSetAttenuation(ADC_11db);
  
  // Inicializar filtro
  inicializarFiltro();

  delay(2000);

  Serial.println("Iniciando monitoreo...\n");
}

// ========== LOOP ==========
void loop() {
  unsigned long tiempoActual = millis();
  
  // Verificar si es tiempo de tomar una nueva lectura
  if (tiempoActual - ultimaLectura >= PERIODO_MUESTREO) {
    ultimaLectura = tiempoActual;
    
    // Leer valor crudo del ADC
    int valorCrudo = analogRead(POT_PIN);
    
    // Aplicar filtro de media móvil
    int valorFiltrado = leerADCFiltrado(POT_PIN);
    
    // ========== Plotter ==========
    Serial.print(">Crudo:");
    Serial.println(valorCrudo);
    
    Serial.print(">Filtrado:");
    Serial.println(valorFiltrado);
  }
}
