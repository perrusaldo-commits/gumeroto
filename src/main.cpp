#include <Arduino.h>
#include <Preferences.h>
#include <RadioLib.h>
#include "esp_task_wdt.h"
#include "driver/rtc_io.h"

#define DEBUG Serial

// -----------------------------------------------------------------------------
//    CONFIGURACIÓN DE PINES (Especí­fico para XIAO ESP32-S3 + Wio SX1262)
// -----------------------------------------------------------------------------

constexpr gpio_num_t RADIO_NSS =     GPIO_NUM_41;  // selección SPI del chip de radio
constexpr gpio_num_t RADIO_DIO1 =    GPIO_NUM_39;  // interrupción de radio (para avisar de eventos como transmisión completa, recepción de mensaje, etc.)
constexpr gpio_num_t RADIO_BUSY =    GPIO_NUM_40;  // estado del chip de radio
constexpr gpio_num_t RADIO_RST =     GPIO_NUM_42;
constexpr gpio_num_t RADIO_DIO2 =    GPIO_NUM_38;  // interruptor de antena RF (ya está implementado en el módulo de radio, no se usa directamente en el código)
constexpr gpio_num_t LED_LORA =      GPIO_NUM_48;  // LED para indicar actividad de la radio
constexpr gpio_num_t BOTON_USUARIO = GPIO_NUM_21;

// --------------------------------------------------------------
//    CONFIGURACIÓN DEL SISTEMA - SENSORES
//---------------------------------------------------------------

constexpr uint32_t BAUDIOS = 115200UL;                                          // depuración
constexpr int N = 20;   
constexpr float VREF = 3.3f;
constexpr int ADC_RES = 4095;

struct AnalogSensor {
  int pin;                                                                      // Pin analógico del sensor
  float buffer[N];                                                              // Buffer circular para el filtro
  int index;                                                                    // índice para el buffer circular
  bool initialized;                                                             // Indica si el buffer ha sido inicializado

  AnalogSensor(int p) {                                                         // Constructor para inicializar el sensor
    pin = p;                                                                    // El pin se asigna al crear el objeto
    index = 0;                                                                  // El í­ndice comienza en 0
    initialized = false;                                                        // El buffer se inicializará con la primera lectura del sensor
  }
};

AnalogSensor phSensor   {A0};       // pH en A0
AnalogSensor ecSensor   {A1};       // EC en A1
AnalogSensor tempSensor {A2};       // Temperatura en A2
AnalogSensor odSensor   {A3};       // OD en A3
AnalogSensor orpSensor  {A4};       // ORP en A4

// ------------------------------------
//     VARIABLES DE LA RADIO
// ------------------------------------

SX1262      radio = new Module (RADIO_NSS, RADIO_DIO1, RADIO_RST, RADIO_BUSY);
LoRaWANNode nodo (&radio, &EU868);
uint8_t payload[12];

// -----------------------------------------------------------------------------
//         CREDENCIALES LORAWAN
// -----------------------------------------------------------------------------

// Device EUI (RadioLib usa MSB por defecto para SX1262)

constexpr uint64_t joinEUI = 0x6AC576F3CCB72F72; // A veces llamado AppEUI
constexpr uint64_t devEUI  = 0x70B3D57ED007805A; // Dispositivo Device EUI
constexpr uint8_t appKey[] = { 0x15, 0x39, 0x9D, 0xDE, 0xEB, 0x85, 0x1D, 0xD2, 0xDA, 0x3B, 0x73, 0xC2, 0xC2, 0x9F, 0x50, 0xBB }; // AppKey (MSB)
constexpr uint8_t nwkKey[] = { 0x15, 0x39, 0x9D, 0xDE, 0xEB, 0x85, 0x1D, 0xD2, 0xDA, 0x3B, 0x73, 0xC2, 0xC2, 0x9F, 0x50, 0xBB }; // Para LoRaWAN 1.0.x igual a AppKey
constexpr uint8_t fPort = 2; 



// ------------------------------------
//     VARIABLES DE ENERGÍA
// ------------------------------------

constexpr float VOLTAJE_MINIMO_OPERACION = 3.2f;  // Voltaje mínimo para operar
constexpr float VOLTAJE_CARGA_COMPLETA = 4.1f;    // Voltaje de batería llena
#define PIN_BATERIA A5                            // El pin A5 se usa para medir el voltaje de la baterí­a a través de un divisor de tensión.
constexpr float FACTOR_DIV = 2.0f;                // divisor 470k/470k
constexpr uint32_t INTERVALOS[4] = {              // Configuración dinámica de intervalos (en minutos) Panel pequeño (10W)
    5,                                            // Batería alta (> 4.0V) - Muestreo frecuente
    12,                                           // Batería media (3.6V - 4.0V)
    25,                                           // Batería baja (3.2V - 3.6V)
    60                                            // Batería crítica (< 3.2V) - Modo supervivencia
};

// ------------------------------------
//     RTC
// ------------------------------------

struct RTC_Data {
    uint32_t magic;                               // Palabra mágica 0xDEADBEEF
    uint32_t contador_muestras;
    uint32_t envios_exitosos;
    uint16_t fallos_consecutivos;
    uint32_t ultimo_tiempo_activo;
    uint16_t vbat_minimo;                         // Voltaje mínimo registrado (x100)
    uint16_t vbat_maximo;                         // Voltaje máximo registrado (x100)
    uint16_t crc;
} __attribute__((aligned(4)));

RTC_DATA_ATTR RTC_Data rtc;
RTC_DATA_ATTR uint8_t LWsession[RADIOLIB_LORAWAN_SESSION_BUF_SIZE];
RTC_DATA_ATTR uint16_t bootCount = 0;
RTC_DATA_ATTR uint32_t lastFCntUp = 0;
RTC_DATA_ATTR uint32_t lastBackupFcnt = 0;

Preferences backup;


// ============================================
//    PROTOTIPOS DE FUNCIONES
// ============================================

void inicializar_radio();
float leerBateria();
uint8_t construirPayload(float temp, float ph, float ec, float od, float orp , float bateria);
uint16_t calcularCRC(RTC_Data* data);
void inicializarSistema();
void guardarBackupEmergencia();
void recuperarDeBackup();
uint32_t calcularIntervaloSueno(float vbat);
void configurarPinesDeepSleep();
bool bateriaSaludable(float vbat);
void imprimirEstadisticas();
bool sessionEsConsistente();

void setup() {
  DEBUG.begin(BAUDIOS);
  delay(3000);                   // Esperar a que el puerto serie conecte
  
  bootCount++;                   // ==1 forzar join. >1 mantener sesión 

  inicializarSistema();         // Inicializar sistema (RTC, backups, etc.)

  if (bootCount == 1) {         // Solo limpiar LWsession en el primer arranque (bootCount == 1)
    memset(LWsession, 0, RADIOLIB_LORAWAN_SESSION_BUF_SIZE);
    DEBUG.println(F("[INIT] LWsession inicializado (primer boot)"));
  } else {
    DEBUG.printf("[INIT] LWsession preservado de sesión anterior (boot #%u)\n", bootCount);
  }
  
  // Mostrar causa del despertar
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  switch(cause) {
    case ESP_SLEEP_WAKEUP_TIMER:
      DEBUG.printf("[WAKE] Muestreo #%u (boot #%u)\n", rtc.contador_muestras + 1, bootCount);
      break;
    case ESP_SLEEP_WAKEUP_UNDEFINED:
      DEBUG.printf("[WAKE] Primer inicio o reset (boot #%u)\n", bootCount);
      break;
    default:
      DEBUG.printf("[WAKE] Causa: %d (boot #%u)\n", cause, bootCount);
  }
  
  // Mostrar información de la sesión guardada en RTC
  bool session_valida = false;
  for (int i = 0; i < RADIOLIB_LORAWAN_SESSION_BUF_SIZE; i++) {
    if (LWsession[i] != 0) {
      session_valida = true;
      break;
    }
  }
  
  if (session_valida && bootCount > 1) {
    DEBUG.println(F("[RTC] Sesión encontrada en memoria RTC - intentando restaurar"));
  } else {
    DEBUG.println(F("[RTC] No hay sesión válida en RTC - se hará join completo"));
  }
  
  inicializar_radio();    // Inicializar radio (restaurará sesión si es posible)
  
  DEBUG.println(F("[SETUP] Listo para loop()"));
}










void loop(){
  delay(10);
  float vbat = leerBateria();
        
  // Protección batería
  if (!bateriaSaludable(vbat)) {
    DEBUG.printf("[BATERÍA] Crítica: %.2fV - Modo supervivencia\n", vbat);
    lastFCntUp = 0;
    guardarBackupEmergencia(); 
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);     
    esp_sleep_enable_timer_wakeup(3600ULL * 1000000ULL);
    esp_deep_sleep_start();
  }
    
  // Estadísticas batería
  rtc.vbat_minimo = min(rtc.vbat_minimo, (uint16_t)(vbat * 100));
  rtc.vbat_maximo = max(rtc.vbat_maximo, (uint16_t)(vbat * 100));

  // Lecturas (reemplazar con sensores reales)
  float temp = 23.5;
  float ph   = 7.2;
  float ec   = 1250.0;
  float od   = 6.8;
  float orp  = 245.0;
  
  DEBUG.print(F("Tensión Batería: "));
  DEBUG.print(vbat, 2);
  DEBUG.println(F(" V"));

  // Construir payload
  uint8_t payload_len = construirPayload(temp, ph, ec, od, orp, vbat);
  
  // ⭐ Verificar DevAddr antes de transmitir
  DEBUG.printf("[INFO] DevAddr: 0x%08X\n", nodo.getDevAddr());
  DEBUG.printf("[INFO] FCntUp actual: %lu\n", nodo.getFCntUp());
  
  // Enviar datos
  int estado_radio = nodo.sendReceive(payload, payload_len, fPort);
  
  if (estado_radio == RADIOLIB_ERR_NONE) {

  rtc.envios_exitosos++;
  rtc.fallos_consecutivos = 0;

  uint32_t fcnt = nodo.getFCntUp();

  DEBUG.println(F("[App] ✅ Transmisión completada."));
  DEBUG.printf("[INFO] FCntUp: %lu\n", fcnt);

  // =========================================================
// ✅ VALIDACIÓN FCntUp (ROBUSTA)
// =========================================================
bool session_valida_actual = true;

if (fcnt == 0 || fcnt > 1000000) {

  DEBUG.println("[FCNT] ❌ Valor inválido → limpiando sesión");

  memset(LWsession, 0, RADIOLIB_LORAWAN_SESSION_BUF_SIZE);

  backup.begin("radiolib", false);
  backup.clear();
  backup.end();

  lastFCntUp = 0;
  lastBackupFcnt = 0;
  session_valida_actual = false;

} else if (lastFCntUp > 0 && fcnt <= lastFCntUp) {

  DEBUG.println("[FCNT] ⚠️ Desync detectado → limpiando sesión");

  memset(LWsession, 0, RADIOLIB_LORAWAN_SESSION_BUF_SIZE);

  backup.begin("radiolib", false);
  backup.clear();
  backup.end();

  lastFCntUp = 0;
  lastBackupFcnt = 0;
  session_valida_actual = false;

}
else if (lastFCntUp > 0 && (fcnt - lastFCntUp) > 1000) {

  DEBUG.println("[FCNT] ⚠️ Salto anómalo → limpiando sesión");

  memset(LWsession, 0, RADIOLIB_LORAWAN_SESSION_BUF_SIZE);

  backup.begin("radiolib", false);
  backup.clear();
  backup.end();

  lastFCntUp = 0;
  lastBackupFcnt = 0;
  session_valida_actual = false;
}
else {

  lastFCntUp = fcnt;

  uint8_t* persist = nodo.getBufferSession();
  memcpy(LWsession, persist, RADIOLIB_LORAWAN_SESSION_BUF_SIZE);

  DEBUG.println(F("[SESSION] ✅ Guardada en RTC"));
}

// Backup SOLO si sesión válida

if (session_valida_actual && fcnt >= lastBackupFcnt && (fcnt - lastBackupFcnt) >= 20){
  backup.begin("radiolib", false);
  backup.putBytes("session_backup", LWsession, RADIOLIB_LORAWAN_SESSION_BUF_SIZE);
  backup.putULong("fcnt", fcnt);
  backup.end();

  lastBackupFcnt = fcnt;

  DEBUG.println(F("[BACKUP] ✅ Sesión guardada en flash"));
}


} else {

  DEBUG.print(F("[App] ❌ Error en transmisión, código: "));
  DEBUG.println(estado_radio);

  rtc.fallos_consecutivos++;

  // =========================================================
  // ✅ LÓGICA MÁS INTELIGENTE DE RECUPERACIÓN
  // =========================================================
  if (rtc.fallos_consecutivos > 8) {

    DEBUG.println(F("[LoRaWAN] ⚠️ Muchos fallos → forzando nuevo JOIN"));

    memset(LWsession, 0, RADIOLIB_LORAWAN_SESSION_BUF_SIZE);

    backup.begin("radiolib", false);
    backup.clear();
    backup.end();
    lastFCntUp = 0;   // 🔥 MUY IMPORTANTE
    lastBackupFcnt = 0;
    rtc.fallos_consecutivos = 0;
  }
}
  // Actualizar contadores
  rtc.contador_muestras++;
  rtc.ultimo_tiempo_activo = millis();

  // Backup periódico de estadísticas
  if (rtc.contador_muestras % 100 == 0) {
    guardarBackupEmergencia();
  }
    
  // Intervalo dinámico
  uint32_t minutos_sleep = calcularIntervaloSueno(vbat);
    
  // Debug
  imprimirEstadisticas();
  rtc.crc = calcularCRC(&rtc);
    
  DEBUG.printf("[SLEEP] Durmiendo %u minutos (%.2fV)\n\n", minutos_sleep, vbat);
  
  configurarPinesDeepSleep();
  radio.sleep();   
  delay(150);
  
  // Ir a deep sleep
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_sleep_enable_timer_wakeup(minutos_sleep * 60ULL * 1000000ULL);

  esp_deep_sleep_start();
}














//================= FUNCIONES AUXILIARES ======================

void inicializar_radio() {

  bool nonces_validos = false;
  esp_reset_reason_t reason = esp_reset_reason();

  // Reset físico solo en arranque real
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_UNDEFINED) {
    pinMode(RADIO_RST, OUTPUT);
    digitalWrite(RADIO_RST, LOW);
    delay(50);
    digitalWrite(RADIO_RST, HIGH);
    delay(300);
  }

  int state = radio.begin();
  if (state != RADIOLIB_ERR_NONE) {
    DEBUG.print(F("[Radio] Fallo al iniciar, código: "));
    DEBUG.println(state);
    while (true);
  }

  radio.setDio2AsRfSwitch(true);
  radio.setTCXO(1.8);
  delay(50);

  DEBUG.println(F("[LoRaWAN] Configurando OTAA..."));
  nodo.beginOTAA(joinEUI, devEUI, nwkKey, appKey);

  // =========================================================
  //   RESTAURAR NONCES DESDE FLASH
  // =========================================================
  backup.begin("radiolib", false);

  if (backup.isKey("nonces") &&
      backup.getBytesLength("nonces") == RADIOLIB_LORAWAN_NONCES_BUF_SIZE) {

    uint8_t buffer[RADIOLIB_LORAWAN_NONCES_BUF_SIZE];
    backup.getBytes("nonces", buffer, RADIOLIB_LORAWAN_NONCES_BUF_SIZE);

    state = nodo.setBufferNonces(buffer);

    if (state == RADIOLIB_ERR_NONE) {
      nonces_validos = true;
      DEBUG.println(F("[NONCES] ✅ Restaurados correctamente"));
    } else {
      DEBUG.println(F("[NONCES] ❌ Error restaurando"));
    }

  } else {
    DEBUG.println(F("[NONCES] ⚠️ No encontrados"));
  }

  // =========================================================
  //   VALIDAR SESIÓN (RTC)
  // =========================================================
  bool session_valida = false;

  for (int i = 0; i < RADIOLIB_LORAWAN_SESSION_BUF_SIZE; i++) {
    if (LWsession[i] != 0) {
      session_valida = true;
      break;
    }
  }

  // Reset real → invalidar sesión
  if (reason == ESP_RST_POWERON || reason == ESP_RST_BROWNOUT) {
    DEBUG.println("[RESET] ⚠️ Reset real → limpiando sesión");
    memset(LWsession, 0, RADIOLIB_LORAWAN_SESSION_BUF_SIZE);
    session_valida = false;
  }

  state = nodo.setBufferSession(LWsession);

  // =========================================================
  //   INTENTAR RESTAURAR SESIÓN
  // =========================================================
  if (state == RADIOLIB_ERR_NONE && session_valida && nonces_validos) {

    DEBUG.println(F("[LoRaWAN] Intentando restaurar sesión..."));

    state = nodo.activateOTAA();

    if (state == RADIOLIB_LORAWAN_SESSION_RESTORED) {

      DEBUG.println(F("[LoRaWAN] ✅ Sesión restaurada"));

      if (!sessionEsConsistente()) {

        DEBUG.println("[SESSION] ❌ Inconsistente → forzar JOIN");

        memset(LWsession, 0, RADIOLIB_LORAWAN_SESSION_BUF_SIZE);
        lastFCntUp = 0;
        lastBackupFcnt = 0;

      } else {

        DEBUG.printf("[SESSION] ✅ OK (FCntUp=%lu)\n", nodo.getFCntUp());
        backup.end();
        return;
      }

    } else {

      DEBUG.printf("[LoRaWAN] ❌ Falló restauración: %d\n", state);

      memset(LWsession, 0, RADIOLIB_LORAWAN_SESSION_BUF_SIZE);
      lastFCntUp = 0;
      lastBackupFcnt = 0;
    }
  }

  backup.end();

  // =========================================================
  //   HACER JOIN NUEVO
  // =========================================================
  DEBUG.println(F("[LoRaWAN] Haciendo JOIN..."));

  while (true) {

    state = nodo.activateOTAA();

    if (state == RADIOLIB_LORAWAN_NEW_SESSION) {
      DEBUG.println(F("[LoRaWAN] ✅ JOIN exitoso"));
      break;
    }

    DEBUG.printf("[LoRaWAN] Join falló: %d - Reintento...\n", state);
    delay(10000);
  }

  // =========================================================
  //   GUARDAR NONCES EN FLASH
  // =========================================================
  backup.begin("radiolib", false);

  uint8_t nonces_buffer[RADIOLIB_LORAWAN_NONCES_BUF_SIZE];
  uint8_t* persist = nodo.getBufferNonces();

  memcpy(nonces_buffer, persist, RADIOLIB_LORAWAN_NONCES_BUF_SIZE);
  backup.putBytes("nonces", nonces_buffer, RADIOLIB_LORAWAN_NONCES_BUF_SIZE);

  DEBUG.println(F("[NONCES] ✅ Guardados en flash"));

  backup.end();

  delay(3000);
}


//-------------------------------------------------------------------------------------------------------------------------------------------------

uint8_t construirPayload(float temp, float ph, float ec, float od, float orp , float bateria) {

  // Escalado para evitar floats

  int16_t batt_i  = (bateria < 3.0 || bateria > 4.3) ? -1 : bateria * 100;  // 3.72 * 100 = 372 (int16_t convierte a entero, 2 bytes)
  int16_t temp_i  = (temp < -50 || temp > 100) ? -1 : temp * 10;            // 23.4Â°C -> 234
  int16_t ph_i    = (ph < 0 || ph > 14) ? -1 : ph * 100;                    // 7.23  -> 723
  int16_t ec_i    = (ec < 0 || ec > 20000) ? -1 : ec * 10;                  // depende unidad (ej: ÂµS/cm)
  int16_t od_i    = (od < 0 || od > 20) ? -1 : od * 10;                     // oxÃ­geno disuelto
  int16_t orp_i   = (orp < -2000 || orp > 2000) ? -1 : orp;                 // ORP ya suele venir en mV (entero) 

          payload[0] = highByte(temp_i);      // Temperatura (2 bytes)
          payload[1] = lowByte(temp_i);
          payload[2] = highByte(ph_i);        // pH (2 bytes)
          payload[3] = lowByte(ph_i);
          payload[4] = highByte(ec_i);        // EC (2 bytes)
          payload[5] = lowByte(ec_i);
          payload[6] = highByte(od_i);        // OD (2 bytes)
          payload[7] = lowByte(od_i);
          payload[8] = highByte(orp_i);       // ORP (2 bytes)
          payload[9] = lowByte(orp_i);
          payload[10] = highByte(batt_i);     // Baterí­a (2 bytes)
          payload[11] = lowByte(batt_i);

  return 12;                                  // total bytes
}

//--------------------------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------------------------
float leerBateria() {
    // Configurar ADC para mejor precisión
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    
    // Múltiples lecturas para estabilizar
    const int NUM_LECTURAS = 20;
    uint32_t suma = 0;
    
    for (int i = 0; i < NUM_LECTURAS; i++) {
        suma += analogRead(PIN_BATERIA);
        delayMicroseconds(100);
    }
    
    float raw = suma / (float)NUM_LECTURAS;
    float voltaje = (raw / 4095.0) * 3.3 * FACTOR_DIV;
    
    return constrain(voltaje, 2.5, 4.3);
}

//-------------------------------------------------------------------------------------------------------------------

bool bateriaSaludable(float vbat) {
    // Si está por debajo del mínimo, no operar
    if (vbat < VOLTAJE_MINIMO_OPERACION) {
        return false;
    }
    
    // Detectar si viene de un brownout reciente
    static float ultimo_vbat = 0;
    if (ultimo_vbat > VOLTAJE_MINIMO_OPERACION + 0.5 && vbat < VOLTAJE_MINIMO_OPERACION) {
        DEBUG.println("[ALERTA] Caída súbita de batería detectada");
        return false;
    }
    
    ultimo_vbat = vbat;
    return true;
}

//------------------------------------------------------------------------------------------------------------------------

uint32_t calcularIntervaloSueno(float vbat) {
    if (vbat >= 4.0) return INTERVALOS[0];                        // Batería llena 
    if (vbat >= 3.6) return INTERVALOS[1];                        // Batería media
    if (vbat >= VOLTAJE_MINIMO_OPERACION) return INTERVALOS[2];   // Batería baja
    
    return INTERVALOS[3];                                         // Modo supervivencia
}

//---------------------------------------------------------------------------------------------------------------

uint16_t calcularCRC(RTC_Data* data) {
    uint16_t crc = 0xFFFF;
    uint8_t* ptr = (uint8_t*)data;
    
    for (int i = 0; i < offsetof(RTC_Data, crc); i++) {
        crc ^= ptr[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0x8408;
            else crc >>= 1;
        }
    }
    return crc;
}

//---------------------------------------------------------------------------------------------------------------
void inicializarSistema() {
    
    if (rtc.magic != 0xDEADBEEF || calcularCRC(&rtc) != rtc.crc) {    // Verificar integridad de RTC
        DEBUG.println(F("[RTC] Corrupto - Recuperando de backup..."));
        recuperarDeBackup();
        
        if (rtc.magic != 0xDEADBEEF || calcularCRC(&rtc) != rtc.crc) {                              // Si sigue corrupto, inicializar todo desde cero
            DEBUG.println(F("[RTC] Inicialización completa"));
            rtc.magic = 0xDEADBEEF;
            rtc.contador_muestras = 0;
            rtc.envios_exitosos = 0;
            rtc.fallos_consecutivos = 0;
            rtc.ultimo_tiempo_activo = 0;
            rtc.vbat_minimo = 500;            // 5.00V (valor alto)
            rtc.vbat_maximo = 0;
            rtc.crc = calcularCRC(&rtc);
        }
    } else {
        DEBUG.printf("[RTC] Válido - Muestras: %u, Éxitos: %u\n", 
                     rtc.contador_muestras, rtc.envios_exitosos);
    }
}

//------------------------------------------------------------------------------------------------------------------

void guardarBackupEmergencia() {
    
    if (rtc.magic != 0xDEADBEEF) return;    // Solo guardar si RTC es válido
    
    rtc.crc = calcularCRC(&rtc);            // Actualizar CRC antes de guardar
    
    backup.begin("solar_node", false);
    backup.putBytes("rtc", &rtc, sizeof(rtc));
    backup.putULong("timestamp", millis());
    backup.end();
    
    DEBUG.println(F("[BACKUP] Estado guardado en flash"));
}
//------------------------------------------------------------------------------------------------------------------

void recuperarDeBackup() {
    backup.begin("solar_node", true);
    
    if (backup.isKey("rtc")) {
        size_t size = backup.getBytesLength("rtc");
        if (size == sizeof(rtc)) {
            backup.getBytes("rtc", &rtc, sizeof(rtc));
                        
            if (calcularCRC(&rtc) == rtc.crc) {                   // Verificar CRC después de recuperar
                DEBUG.printf("[RECUPERACIÓN] Exitosa - Muestras: %u\n",
                              rtc.contador_muestras);
                backup.end();
                return;
            }
        }
    }
    
    memset(&rtc, 0, sizeof(rtc));   // No hay backup válido limpiar
    backup.end();
}

//--------------------------------------------------------------------------------------------------------
void configurarPinesDeepSleep() {
  
    // Configurar pines no usados para mínimo consumo
    // gpio_config_t io_conf = {
    //     .pin_bit_mask = ~0ULL,
    //     .mode = GPIO_MODE_INPUT,
    //     .pull_up_en = GPIO_PULLUP_DISABLE,
    //     .pull_down_en = GPIO_PULLDOWN_DISABLE,
    //     .intr_type = GPIO_INTR_DISABLE
    // };
    // gpio_config(&io_conf);
    

  // No intentes configurar todos los pines - es más seguro NO hacer nada
  // El ESP32-S3 maneja los pines por defecto correctamente
  
  // Solo configura los pines que USAS activamente a su estado de reposo
  
  
  pinMode(RADIO_NSS, OUTPUT);             // Dejar el pin CS de la radio en HIGH para que no consuma
  digitalWrite(RADIO_NSS, HIGH);
  pinMode(LED_LORA, OUTPUT);              // LED: apagado y como salida para evitar flotar
  digitalWrite(LED_LORA, LOW);
  
  // Pines de sensores analógicos: dejar como están (no configurar)
  // El ADC ya los maneja correctamente
}

//-------------------------------------------------------------------------------------------------

bool sessionEsConsistente() {
  uint32_t fcnt = nodo.getFCntUp();

  // Valores absurdos → sesión corrupta
  if (fcnt == 0 || fcnt > 1000000) {
    DEBUG.printf("[SESSION] FCntUp inválido: %lu\n", fcnt);
    return false;
  }

  // Opcional: detectar saltos raros
  if (fcnt < rtc.envios_exitosos) {
    DEBUG.printf("[SESSION] Desync detectado (FCntUp=%lu < envios=%u)\n",
                 fcnt, rtc.envios_exitosos);
    return false;
  }

  return true;
}
//---------------------------------------------------------------------------------------------------------
void imprimirEstadisticas() {
    DEBUG.println(F("\n========== ESTADÍSTICAS =========="));
    DEBUG.printf("Muestras totales:    %u\n", rtc.contador_muestras);
    DEBUG.printf("Envíos exitosos:     %u\n", rtc.envios_exitosos);
    DEBUG.printf("Fallos consecutivos: %u\n", rtc.fallos_consecutivos);
    DEBUG.printf("Vbat mínimo:         %.2fV\n", rtc.vbat_minimo / 100.0);
    DEBUG.printf("Vbat máximo:         %.2fV\n", rtc.vbat_maximo / 100.0);
    
    float eficiencia = (rtc.envios_exitosos * 100.0) / max((uint32_t)1, rtc.contador_muestras);
    DEBUG.printf("Eficiencia:          %.1f%%\n", eficiencia);
    DEBUG.println(F("===================================\n"));
}