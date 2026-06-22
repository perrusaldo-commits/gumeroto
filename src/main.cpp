/*
Curso de especialización IoT
  Centros docentes públicos Castilla y León
  
        PROYECTO: ** GUMER **
  
  IES "Giner de los Ríos" León
  dpto. de electrónica (curso 2024/2025)
                    ¡ACTUALIZADO Junio 2026!
  
  Nivel de aplicación:
 
        ESP32 S3 con 5 sensores analógicos (pH, EC, OD, ORP y temperatura)  
 
Sensor OD:                      Gravity: Analog Dissolved Oxygen SEN0237
Sensor pH:                      Gravity: Analog pH SEN0161-V2
Sensor ORP:                     Gravity: Analog ORP SEN0464
Sensor Conductividad/Temp:      Gravity: Analog EC + TEMP SEN0451

*/
#include <Arduino.h>
#include <Preferences.h>
#include <RadioLib.h>

#define DEBUG Serial

constexpr uint8_t N = 20;                               // Tamaño del buffer para el filtro lecturas sensores
constexpr float VREF = 3.3f;
constexpr uint8_t ADC_BITS = 12;                        // 12 bits
constexpr uint16_t ADC_MAX = (1 << ADC_BITS) - 1;       // 4095 calculado automáticamente

// -----------------------------------------------------------------------------
//    CONFIGURACIÓN DE PINES (XIAO ESP32-S3 + Wio SX1262)
// -----------------------------------------------------------------------------

constexpr gpio_num_t RADIO_NSS =     GPIO_NUM_41;
constexpr gpio_num_t RADIO_DIO1 =    GPIO_NUM_39;
constexpr gpio_num_t RADIO_BUSY =    GPIO_NUM_40;
constexpr gpio_num_t RADIO_RST =     GPIO_NUM_42;
constexpr gpio_num_t RADIO_DIO2 =    GPIO_NUM_38;
constexpr gpio_num_t LED_LORA =      GPIO_NUM_48;
constexpr gpio_num_t BOTON_USUARIO = GPIO_NUM_21;

// -----------------------------------------------------------------------------
//    CREDENCIALES LORAWAN
// -----------------------------------------------------------------------------

constexpr uint64_t joinEUI = 0x6AC576F3CCB72F72;
constexpr uint64_t devEUI  = 0x70B3D57ED007805A;
constexpr uint8_t appKey[] = { 0x15, 0x39, 0x9D, 0xDE, 0xEB, 0x85, 0x1D, 0xD2, 0xDA, 0x3B, 0x73, 0xC2, 0xC2, 0x9F, 0x50, 0xBB };
constexpr uint8_t nwkKey[] = { 0x15, 0x39, 0x9D, 0xDE, 0xEB, 0x85, 0x1D, 0xD2, 0xDA, 0x3B, 0x73, 0xC2, 0xC2, 0x9F, 0x50, 0xBB };
constexpr uint8_t fPort = 2;

// -------------------------------------------------------------------------------
// SENSOR ANALÓGICO
// -------------------------------------------------------------------------------

struct AnalogSensor {
  int pin;                // Pin analógico del sensor
  float buffer[N];        // Buffer circular para el filtro
  int index;              // Índice para el buffer circular
  bool initialized;       // Indica si el buffer ha sido inicializado

  AnalogSensor(int p) {   // Constructor para inicializar el sensor
    pin = p;              // El pin se asigna al crear el objeto
    index = 0;            // El índice comienza en 0
    initialized = false;  // El buffer se inicializará con la primera lectura del sensor
  }
};

// -----------------------------------------------------------------------------
//    CONFIGURACIÓN DE BATERÍA E INTERVALOS DINÁMICOS
// -----------------------------------------------------------------------------

#define PIN_BATERIA A5
constexpr float VOLTAJE_MINIMO_OPERACION = 3.2f;
constexpr float VOLTAJE_CARGA_COMPLETA = 5.1f;  // 4.1f
constexpr float FACTOR_DIV = 2.0f;              // DIVISOR 470K + 470K
constexpr uint32_t INTERVALOS[4] = {            // Intervalos en minutos según voltaje de batería
                                    5,          // Batería alta (> 4.0V)
                                    12,         // Batería media (3.6V - 4.0V)
                                    25,         // Batería baja (3.2V - 3.6V)
                                    60          // Batería crítica (< 3.2V) - Modo supervivencia
                                };

// -----------------------------------------------------------------------------
// SENSORES
// -----------------------------------------------------------------------------
AnalogSensor phSensor   {A0};       // pH en A0
AnalogSensor ecSensor   {A1};       // EC en A1
AnalogSensor tempSensor {A2};       // Temperatura en A2
AnalogSensor odSensor   {A3};       // OD en A3
AnalogSensor orpSensor  {A4};       // ORP en A4

// -----------------------------------------------------------------------------
// PH   
// -----------------------------------------------------------------------------
float ph_offset = 0.0; 
float ph_slope  = -5.7;
float ph_vol4 = 0;
float ph_vol7 = 2.5;

// -----------------------------------------------------------------------------
// EC
// -----------------------------------------------------------------------------
float ec_k = 1.0;

// -----------------------------------------------------------------------------
// OD
// -----------------------------------------------------------------------------
float od_v1 = 1500;
float od_t1 = 25;

constexpr uint16_t tablaOD[41] = {
 14460,14220,13820,13440,13090,12740,12420,12110,11810,11530,
 11260,11010,10770,10530,10300,10080,9860,9660,9460,9270,
 9080,8900,8730,8570,8410,8250,8110,7960,7820,7690,
 7560,7430,7300,7180,7070,6950,6840,6730,6630,6530,6410
};

// -----------------------------------------------------------------------------
// ORP
// -----------------------------------------------------------------------------
float orp_offset = 0;

// -----------------------------------------------------------------------------
//    VARIABLES GLOBALES DE LA RADIO
// -----------------------------------------------------------------------------

SX1262 radio = new Module(RADIO_NSS, RADIO_DIO1, RADIO_RST, RADIO_BUSY);
LoRaWANNode nodo(&radio, &EU868);
Preferences backup;

RTC_DATA_ATTR uint16_t bootCount = 0;                                   // número de reinicios
RTC_DATA_ATTR uint8_t LWsession[RADIOLIB_LORAWAN_SESSION_BUF_SIZE];

uint8_t uplinkPayload[12];  // telemetría: temp (2 bytes), ph (2 bytes), ec (2 bytes), od (2 bytes), orp (2 bytes), bateria (2 bytes) = 12 bytes total

// -----------------------------------------------------------------------------
//    PROTOTIPOS
// -----------------------------------------------------------------------------

void inicializarRadio();
float leerBateria();
void construirPayload(float temp, float ph, float ec, float od, float orp, float bateria);
uint32_t calcularIntervaloSueno(float vbat);
bool bateriaSaludable(float vbat);
void enviarDatos();
float filtro(AnalogSensor &s, float nueva);
bool estable(AnalogSensor &s, float umbral);  
float leerVoltaje(int pin);
float leerTemp();
float leerPH(float temp);
float leerEC(float temp);
float leerOD(float temp);
float leerORP();
void leerCalibraciones();
void salvarCalibracionORP(float valor);
void salvarCalibracionEC(float valor, float temp);
void salvarCalibracionOD(float temp);
void salvarCalibracionPh();
void comandos();
void imprimirMenu();
float compensarPH(float ph, float temp);

// -----------------------------------------------------------------------------
//    SETUP
// -----------------------------------------------------------------------------

void setup() {
    DEBUG.begin(115200);
    pinMode(LED_LORA, OUTPUT);
    digitalWrite(LED_LORA, LOW);
    pinMode(BOTON_USUARIO, INPUT_PULLUP);
    analogReadResolution(ADC_BITS);
    analogSetAttenuation(ADC_11db);
    //leerCalibraciones();

    delay(2000);
    
    bootCount++;
    DEBUG.printf("[BOOT] #%u\n", bootCount);
    
    // Mostrar causa de wake
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
            if (cause == ESP_SLEEP_WAKEUP_TIMER) {
                DEBUG.println("[WAKE] Timer");
            } else {
                DEBUG.printf("[WAKE] Cold boot: %d\n", cause);
            }
    
    // Inicializar radio y restaurar sesión
    inicializarRadio();
    
    // Verificar sesión
    uint32_t devAddr = nodo.getDevAddr();
        if (devAddr == 0) {
            DEBUG.println("[ERROR] Sesión inválida");
            delay(1000);
            ESP.restart();
        }
    
    DEBUG.printf("[LoRaWAN] DevAddr=0x%08X FCnt=%lu\n", devAddr, nodo.getFCntUp());
    DEBUG.println("[SETUP] ✅ Listo");
}

// -----------------------------------------------------------------------------
//    LOOP - Ciclo principal
// -----------------------------------------------------------------------------

void loop() {
    
    float vbat = leerBateria();                 // Leer batería
    DEBUG.printf("🔋[BATERÍA] %.2f V\n", vbat); // mostrar tensión de la batería 
    
// Protección por batería baja
    if (!bateriaSaludable(vbat)) {
        DEBUG.println("[BATERÍA] ⚠️ Batería baja - Sleep 1h");
        radio.sleep();
        delay(50);
        esp_sleep_enable_timer_wakeup(3600ULL * 1000000ULL);
        esp_deep_sleep_start();
    }
    
// -----------------------------
// LECTURAS SENSORES
// -----------------------------
  float temp =    leerTemp();       // 23.5
  float ph   =    leerPH(temp);     // 7.2
  float ec   =    leerEC(temp);     // 1250.0
  float od   =    leerOD(temp);     // 6.8
  float orp  =    leerORP();        // 245.0
    
    
construirPayload(temp, ph, ec, od, orp, vbat);              // Construir payload
    
    if (nodo.getDevAddr() == 0) {                           // Verificar sesión antes de enviar
        DEBUG.println("[LoRaWAN] ❌ Sesión inválida");
        delay(500);
        ESP.restart();
    }
      
enviarDatos();                                              // Enviar datos
    
uint32_t minutos_sleep = calcularIntervaloSueno(vbat);      // Calcular intervalo dinámico según batería
    DEBUG.printf("[SLEEP] %u minutos\n", minutos_sleep);

// -----------------------------
// DORMIR
// -----------------------------
    radio.sleep();
    delay(50);
    esp_sleep_enable_timer_wakeup(minutos_sleep * 60ULL * 1000000ULL);
    esp_deep_sleep_start();
}







// -----------------------------------------------------------------------------
//    FUNCIONES IMPLEMENTADAS
// -----------------------------------------------------------------------------

void inicializarRadio() {
    int state;
    bool coldBoot = (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_UNDEFINED);
    
    // Reset en cold boot
    if (coldBoot) {
        DEBUG.println("[RADIO] Cold boot - reset");
        pinMode(RADIO_RST, OUTPUT);
        digitalWrite(RADIO_RST, LOW);
        delay(20);
        digitalWrite(RADIO_RST, HIGH);
        delay(150);
    }
    
    // Inicializar radio
    state = radio.begin();
    if (state != RADIOLIB_ERR_NONE) {
        DEBUG.printf("[RADIO] ❌ Init fallo: %d\n", state);
        ESP.restart();
    }
    
    radio.setDio2AsRfSwitch(true);
    radio.setTCXO(1.8);
    DEBUG.println("[RADIO] ✅ Inicializada");
    
    // Configurar LoRaWAN
    nodo.beginOTAA(joinEUI, devEUI, nwkKey, appKey);
    backup.begin("radiolib", false);
    
    DEBUG.println("[LoRaWAN] Intentando restaurar sesión...");
    
    // Restaurar NONCES y sesión
    if (backup.isKey("nonces")) {
        uint8_t nonces[RADIOLIB_LORAWAN_NONCES_BUF_SIZE];
        backup.getBytes("nonces", nonces, sizeof(nonces));
        nodo.setBufferNonces(nonces);
        
        state = nodo.setBufferSession(LWsession);
        if (state == RADIOLIB_ERR_NONE) {
            DEBUG.println("[LoRaWAN] ✅ Sesión restaurada");
            state = nodo.activateOTAA();
            if (state == RADIOLIB_LORAWAN_SESSION_RESTORED) {
                DEBUG.println("[LoRaWAN] ✅ Activación correcta");
                backup.end();
                return;
            }
        }
    }
    
    // JOIN nuevo si no hay sesión
    DEBUG.println("[LoRaWAN] 🔄 JOIN necesario");
    while (true) {
        state = nodo.activateOTAA();
        if (state == RADIOLIB_LORAWAN_NEW_SESSION) {
            DEBUG.println("[LoRaWAN] ✅ JOIN OK");
            
            // Guardar NONCES
            uint8_t nonces[RADIOLIB_LORAWAN_NONCES_BUF_SIZE];
            memcpy(nonces, nodo.getBufferNonces(), sizeof(nonces));
            backup.putBytes("nonces", nonces, sizeof(nonces));
            break;
        }
        DEBUG.printf("[LoRaWAN] ❌ Join fallo: %d\n", state);
        delay(10000);
    }
    
    backup.end();
    DEBUG.println("[LoRaWAN] Esperando RX windows...");
    delay(3000);
}

// -----------------------------------------------------------------------------
//⚠️ OD y EC tienen problemas de escalado

void construirPayload(float temp, float ph, float ec, float od, float orp, float bateria) {
    int16_t batt_i  = (bateria < 3.0 || bateria > 4.9) ? -1 : (int16_t)(bateria * 100); // 3.72 * 100 = 372 (int16_t convierte a entero, 2 bytes)
    int16_t temp_i  = (temp < -50 || temp > 100) ? -1 : (int16_t)(temp * 10);           // 23.4°C -> 234
    int16_t ph_i    = (ph < 0 || ph > 14) ? -1 : (int16_t)(ph * 100);                   // 7.23  -> 723
    int16_t ec_i    = (ec < 0 || ec > 20000) ? -1 : (int16_t)(ec * 10);                 // depende unidad (ej: µS/cm)
    int16_t od_i    = (od < 0 || od > 20) ? -1 : (int16_t)(od * 100);                   // Cambiado a *100 para más precisión
    int16_t orp_i   = (orp < -2000 || orp > 2000) ? -1 : (int16_t)orp;                  // ORP ya suele venir en mV (entero) 
    
    uplinkPayload[0] = (temp_i >> 8) & 0xFF;
    uplinkPayload[1] = temp_i & 0xFF;
    uplinkPayload[2] = (ph_i >> 8) & 0xFF;
    uplinkPayload[3] = ph_i & 0xFF;
    uplinkPayload[4] = (ec_i >> 8) & 0xFF;
    uplinkPayload[5] = ec_i & 0xFF;
    uplinkPayload[6] = (od_i >> 8) & 0xFF;
    uplinkPayload[7] = od_i & 0xFF;
    uplinkPayload[8] = (orp_i >> 8) & 0xFF;
    uplinkPayload[9] = orp_i & 0xFF;
    uplinkPayload[10] = (batt_i >> 8) & 0xFF;
    uplinkPayload[11] = batt_i & 0xFF;
}

// -----------------------------------------------------------------------------

float leerBateria() {   
    uint32_t suma = 0;
    
    for (int i = 0; i < N; i++) {
        suma += analogRead(PIN_BATERIA);
        delayMicroseconds(100);
    }
    
    float raw = suma / (float)N;
    float voltaje = (raw / (float)ADC_MAX) * 3.3 * FACTOR_DIV;
    
    return constrain(voltaje, VOLTAJE_MINIMO_OPERACION, VOLTAJE_CARGA_COMPLETA);
}

// -----------------------------------------------------------------------------

bool bateriaSaludable(float vbat) {
    if (vbat < VOLTAJE_MINIMO_OPERACION) {
        return false;
    }
    
    static float ultimo_vbat = 0;
    if (ultimo_vbat > VOLTAJE_MINIMO_OPERACION + 0.5 && vbat < VOLTAJE_MINIMO_OPERACION) {
        DEBUG.println("[ALERTA] Caída súbita de batería");
        return false;
    }
    
    ultimo_vbat = vbat;
    return true;
}

// -----------------------------------------------------------------------------

uint32_t calcularIntervaloSueno(float vbat) {
    if (vbat >= 4.0) return INTERVALOS[0];
    if (vbat >= 3.6) return INTERVALOS[1];
    if (vbat >= VOLTAJE_MINIMO_OPERACION) return INTERVALOS[2];
    return INTERVALOS[3];
}

// -----------------------------------------------------------------------------

void enviarDatos() {
    DEBUG.println("[LoRaWAN] 📡 Enviando...");
    
    int estado = nodo.sendReceive(uplinkPayload, sizeof(uplinkPayload), fPort);
    
    if (estado == RADIOLIB_ERR_NONE || estado == 1 || estado == -1101) {
        DEBUG.printf("[OK] TX enviado FCnt=%lu\n", nodo.getFCntUp());
        memcpy(LWsession, nodo.getBufferSession(), RADIOLIB_LORAWAN_SESSION_BUF_SIZE);
    } else {
        DEBUG.printf("[ERROR] TX fallo: %d\n", estado);
    }
}

// ************************************************************
// FILTRO GENERICO
// ************************************************************
float filtro(AnalogSensor &s, float nueva) {

  if (nueva < 0.05 || nueva > 3.2) {                                // ignorar valores claramente inválidos que podrían corromper el buffer.
    return s.initialized ? s.buffer[(s.index - 1 + N) % N] : nueva;
  }
  
  s.buffer[s.index] = nueva;  // llenar progresivamente el buffer circular con las lecturas del sensor. El índice s.index se incrementa con cada nueva lectura, y cuando alcanza el tamaño del buffer N, vuelve a 0, sobrescribiendo las lecturas más antiguas. Esto permite mantener un historial de las últimas N lecturas para aplicar el filtro de media móvil.
  s.index++;

  if (s.index >= N) {
    s.index = 0;
    s.initialized = true;
  }

  int count = s.initialized ? N : s.index;

  float sum = 0;
  for (int i = 0; i < count; i++) sum += s.buffer[i];

  return sum / count;
}

// ************************************************************
// ESTABILIDAD
// ************************************************************
bool estable(AnalogSensor &s, float umbral) {

  float maxV = s.buffer[0];
  float minV = s.buffer[0];

  for(int i = 1; i < N; i++){
    if(s.buffer[i]>maxV) maxV = s.buffer[i];
    if(s.buffer[i]<minV) minV = s.buffer[i];
  }

  return (maxV - minV) < umbral;
}

// ************************************************************
// UTILIDADES
// ************************************************************
float leerVoltaje(int pin){
  float sum = 0;
  for(int i = 0; i < N; i++){     // Promediar varias lecturas para mayor estabilidad
    sum += analogRead(pin);       // Leer valor ADC del pin analógico N veces y acumularlo para luego promediar. Esto ayuda a reducir el ruido en la lectura, especialmente importante para sensores analógicos que pueden tener fluctuaciones momentáneas.
    delayMicroseconds(200);       // Pequeña pausa entre lecturas para evitar interferencias y permitir que el ADC se estabilice, especialmente importante si los sensores tienen una alta impedancia de salida (como suele ser el caso de los sensores de pH y ORP). Sin esta pausa, las lecturas podrían ser más ruidosas o inestables.
  }

  return (sum /(float)N) * (VREF /(float)ADC_MAX);  // Convertir el valor promedio del ADC a voltaje real (0 - 3.3V). Este paso es crucial para interpretar correctamente las lecturas de los sensores, ya que cada sensor tiene una relación específica entre el voltaje de salida y la magnitud que mide (pH, EC, etc.).
}

//----------------------------------------------------------------------------

float leerTemp(){
  float v = leerVoltaje(tempSensor.pin);
  v = filtro(tempSensor, v);    // Filtrado recomendado para mejorar estabilidad de la lectura de temperatura, que suele ser más sensible a ruido y fluctuaciones. El filtro suaviza las variaciones rápidas y proporciona un valor más estable para los cálculos posteriores (pH, EC, OD). Dado que la temperatura afecta directamente a las otras mediciones, es crucial tener una lectura de temperatura lo más precisa y estable posible para compensar correctamente el pH y la conductividad.
  if (v < 0.05 || v > 3.2) return 25.0; // Protección

  // --- Conversión empírica optimizada para ESP32 ---
  // Ajustada a comportamiento del SEN0451
  float temp = 0;
  temp = 107.8 * v - 60.0;    // Región útil (~0.6V - 1.1V)
  temp += -15.0 * (v - 0.8) * (v - 0.8);  // Corrección fina (curvatura real del PT1000 + electrónica)

   DEBUG.print(F("Temp: "));
   DEBUG.print(temp, 1);  
   DEBUG.println(F(" °C"));

  return temp;
}

//----------------------------------------------------------------------------

float compensarPH(float ph, float temp){
  float factor = (temp + 273.15) / (25.0 + 273.15);
  return 7 + (ph - 7) * factor;
}

// ************************************************************
// PH
// ************************************************************
float leerPH(float temp){

  float v = leerVoltaje(phSensor.pin);
  if (v < 0.1 || v > 3.2) {
      DEBUG.print(F(" ⚠️ ERROR sensor"));
      DEBUG.println();
      return -1;
  }
  float f = filtro(phSensor, v);

  float ph_raw = ph_slope * f + ph_offset;
  float ph = compensarPH(ph_raw, temp);

  DEBUG.print(F("pH: "));
  DEBUG.print(ph, 2);

  if(!estable(phSensor, 0.12))
    DEBUG.print(F(" ⚠️"));

  DEBUG.println();
  return ph;
}

// ************************************************************
// EC
// ************************************************************
float ecRaw(float v){
  return 133.42 * v * v * v - 255.86 * v * v + 857.39 * v;
}

float leerEC(float temp){

  float v = leerVoltaje(ecSensor.pin);
      if (v < 0.1 || v > 3.2) {
      DEBUG.print(F(" ⚠️ ERROR sensor"));
      DEBUG.println();
      return -1;
      }
  float f = filtro(ecSensor, v);
  float ec = ecRaw(f);
  float tempCoef = 0.02;                    // Coeficiente de temperatura típico para agua (2% por °C)
  ec = ec / (1 + tempCoef * (temp - 25));   // Compensación de temperatura para EC, ajustando el valor medido a lo que sería a 25°C.
                                            // Esto es importante porque la conductividad del agua varía con la temperatura, y la mayoría de las tablas de conversión EC a TDS o ppm asumen una temperatura de referencia de 25°C.
                                            // Al compensar el EC a esta temperatura, se obtiene una lectura más precisa y comparable, independientemente de las condiciones ambientales actuales.
  ec *= ec_k;

  DEBUG.print(F("EC: "));
  DEBUG.print(ec, 1);
  DEBUG.print(F(" uS | T: "));
  DEBUG.println(temp, 1);

  return ec;
}


// ************************************************************
// OD
// ************************************************************
float calcDO(float mv, int t){
  float vsat = od_v1 + 35 * t - od_t1 * 35;
  if (vsat < 1) return 0;
return (mv * tablaOD[t] / vsat) / 1000.0f;  // La tabla tablaOD está basada en mV, por eso se multiplica mv por 1000 para convertirlo a mV antes de usar la tabla. Esto asegura que el valor de voltaje se interprete correctamente en la función de cálculo de oxígeno disuelto, proporcionando una lectura precisa de DO en mg/L.

}

float leerOD(float temp){
  float v = leerVoltaje(odSensor.pin);
    if (v < 0.1 || v > 3.2) {
        DEBUG.print(F(" ⚠️ ERROR sensor"));
        DEBUG.println();
        return -1;
      }
  float f = filtro(odSensor, v) * 1000.0f;  // Convertir a mV para la fórmula de OD, ya que la tabla tablaOD está basada en mV. Esto asegura que el valor de voltaje se interprete correctamente en la función de cálculo de oxígeno disuelto, proporcionando una lectura precisa de DO en mg/L.
  int t = constrain((int)temp, 0, 40);      // La tabla de oxígeno disuelto (tablaOD) está definida para temperaturas entre 0°C y 40°C. Al convertir la temperatura a un índice entero y usar constrain, nos aseguramos de que siempre se utilice un valor válido dentro del rango de la tabla, evitando errores de índice fuera de rango y garantizando que el cálculo de DO sea preciso para las condiciones actuales del agua.

  float do_val = calcDO(f, t);

  DEBUG.print(F("DO: "));
  DEBUG.print(do_val, 2);
  DEBUG.println(F(" mg/L"));

  return do_val;
}

// ************************************************************
// ORP
// ************************************************************
float leerORP(){
  float v = leerVoltaje(orpSensor.pin);
    if (v < 0.1 || v > 3.2) {
        DEBUG.print(F(" ⚠️ ERROR sensor"));
        DEBUG.println();
        return -1;
      }
  float f = filtro(orpSensor, v);

  float orp = f * 1000 + orp_offset;

  DEBUG.print(F("ORP: "));
  DEBUG.print(orp, 1);
  DEBUG.println(F(" mV"));

  return orp;
}

// ************************************************************
// MENÚ
// ************************************************************
void imprimirMenu(){ 
  DEBUG.println(F("\n================ SISTEMA SENSORES AGUA ================"));
  DEBUG.println(F("Comandos disponibles:\n"));

  DEBUG.println(F("---- pH ----"));
  DEBUG.println(F("ph7        -> Calibrar punto pH 7"));
  DEBUG.println(F("ph4        -> Calibrar punto pH 4"));
  DEBUG.println(F("phc        -> Calcular calibración (4 y 7)")); 

  DEBUG.println(F("\n---- Conductividad (EC) ----"));
  DEBUG.println(F("ec X       -> Calibrar EC con valor conocido (ej: ec 2.5 mS/cm)"));

  DEBUG.println(F("\n---- ORP ----"));
  DEBUG.println(F("orp X      -> Calibrar ORP (ej: orp 225 mV)"));

  DEBUG.println(F("\n---- Oxígeno disuelto (OD) ----"));
  DEBUG.println(F("od         -> Calibrar OD en agua aireada (100% saturación)"));

  DEBUG.println(F("\n=======================================================\n"));
}

// ************************************************************
// CALIBRACIONES
// ************************************************************
void salvarCalibracionORP(float valor){
  float v = filtro(orpSensor, leerVoltaje(orpSensor.pin)) * 1000;
  orp_offset = valor - v;

  backup.begin("orp", false);
  backup.putFloat("offset", orp_offset);
  backup.end();

  DEBUG.println(F("✅ ORP calibrado"));
}

//--------------------------------------------------------------------------------------------------

void salvarCalibracionEC(float valor, float temp){            // valor = patrón
  float v = filtro(ecSensor, leerVoltaje(ecSensor.pin));
  float ec = ecRaw(v);
  float ec25 = ec / (1 + 0.02 * (temp - 25));
      if (ec25 < 0.001) {
      DEBUG.println(F("❌ Error calibración EC (señal muy baja)"));
    return;
    }
  ec_k = valor/ec25;

  backup.begin("ec", false);
    backup.putFloat("k", ec_k);    // salvar calibración
  backup.end();

  DEBUG.println(F("✅ EC calibrado"));
}

//--------------------------------------------------------------------------------------------------

void salvarCalibracionOD(float temp){                           // con agua saturada al 100%
  od_v1 = filtro(odSensor, leerVoltaje(odSensor.pin)) * 1000;
  od_t1 = temp;

  backup.begin("od", false);
    backup.putFloat("v1", od_v1);
    backup.putFloat("t1", od_t1);
  backup.end();

  DEBUG.println(F("✅ OD calibrado"));
}

//--------------------------------------------------------------------------------------------------

void salvarCalibracionPh(){

  if(ph_vol4 != 0 && ph_vol7 != 0 && ph_vol4 != ph_vol7){
    ph_slope = (4.0 - 7.0) / (ph_vol4 - ph_vol7);
    ph_offset = 7.0 - ph_slope*ph_vol7;

    backup.begin("ph", false);               // Guardar calibración de pH (offset y slope)
    backup.putFloat("offset", ph_offset);
    backup.putFloat("slope", ph_slope);
    backup.end();

    DEBUG.println(F("✅ pH calibrado"));
  }
      else {
        DEBUG.println(F("❌ Error calibración pH"));
  }
}

//----------------------------------------------------------------------------------------------------------------------------

void comandos(){

  if(!DEBUG.available()) return;

  String c = DEBUG.readStringUntil('\n');  // Eliminar espacios y saltos de línea
  c.trim();                                 // Convertir a minúsculas para facilitar comparación

  // PH
  if(c=="ph7"){
    ph_vol7 = 0;
    for(int i = 0; i < 10; i++){
    ph_vol7 += leerVoltaje(phSensor.pin);  
    }
    ph_vol7 /= 10;                          // Promediar las 10 lecturas para mayor estabilidad
    ph_vol7 = filtro(phSensor, ph_vol7);    // Aplicar filtro al valor de calibración
    DEBUG.println(F("✅ punto pH7 guardado"));
  }
  else if(c=="ph4"){
    ph_vol4 = 0;
    for(int i = 0; i < 10; i++){
      ph_vol4 += leerVoltaje(phSensor.pin);
    }
    ph_vol4 /= 10;
    ph_vol4 = filtro(phSensor, ph_vol4);
    DEBUG.println(F("✅ punto pH4 guardado"));
  }
  else if(c == "phc"){
    salvarCalibracionPh();
  }

  // EC
  else if(c.startsWith("ec ")){
    float temp = leerTemp();
    salvarCalibracionEC(c.substring(3).toFloat(), temp); // sumergir en el patrón
  }

  // ORP
  else if(c.startsWith("orp ")){
    salvarCalibracionORP(c.substring(4).toFloat());
  }

  // OD
  else if(c=="od"){
    float temp = leerTemp();      // un punto de calibración (es suficiente)
    salvarCalibracionOD(temp);    // mantener el agua saturada al 100%
  }

  else if(c=="help" || c=="?"){
  imprimirMenu();
  }
  
else{
    DEBUG.println(F("❌ comando no valido"));
  }
}

//----------------------------------------------------------------------------------------------------------------------------

void leerCalibraciones(){
  // ORP
  backup.begin("orp", true);                       // Leer calibración de ORP (offset)
    orp_offset = backup.getFloat("offset", 0.0);
  backup.end();

  // EC
  backup.begin("ec", true);                        // Leer calibración de EC (coeficiente k)
    ec_k = backup.getFloat("k", 1.0);
  backup.end();

  // pH
  backup.begin("ph", true);                        // Leer calibración de pH (offset y slope)
    ph_offset = backup.getFloat("offset", 0.0);
    ph_slope  = backup.getFloat("slope", -5.7);
  backup.end();

  // OD
  backup.begin("od", true);                        // Leer calibración de OD (valores v1 y t1)
    od_v1 = backup.getFloat("v1", 1500);
    od_t1 = backup.getFloat("t1", 25);
  backup.end();
}
