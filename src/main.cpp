#include <Arduino.h>
#include <Preferences.h>
#include <RadioLib.h>

#define DEBUG Serial

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

// -----------------------------------------------------------------------------
//    CONFIGURACIÓN DE BATERÍA E INTERVALOS DINÁMICOS
// -----------------------------------------------------------------------------

constexpr float VOLTAJE_MINIMO_OPERACION = 3.2f;
constexpr float VOLTAJE_CARGA_COMPLETA = 4.1f;
#define PIN_BATERIA A5
constexpr float FACTOR_DIV = 2.0f;

// Intervalos en minutos según voltaje de batería
constexpr uint32_t INTERVALOS[4] = {
    5,   // Batería alta (> 4.0V)
    12,  // Batería media (3.6V - 4.0V)
    25,  // Batería baja (3.2V - 3.6V)
    60   // Batería crítica (< 3.2V) - Modo supervivencia
};

// -----------------------------------------------------------------------------
//    VARIABLES GLOBALES
// -----------------------------------------------------------------------------

SX1262 radio = new Module(RADIO_NSS, RADIO_DIO1, RADIO_RST, RADIO_BUSY);
LoRaWANNode nodo(&radio, &EU868);
Preferences backup;

RTC_DATA_ATTR uint16_t bootCount = 0;
RTC_DATA_ATTR uint8_t LWsession[RADIOLIB_LORAWAN_SESSION_BUF_SIZE];

// Payload global de 12 bytes
uint8_t uplinkPayload[12];

// -----------------------------------------------------------------------------
//    PROTOTIPOS
// -----------------------------------------------------------------------------

void inicializarRadio();
float leerBateria();
void construirPayload(float temp, float ph, float ec, float od, float orp, float bateria);
uint32_t calcularIntervaloSueno(float vbat);
bool bateriaSaludable(float vbat);
void enviarDatos();

// -----------------------------------------------------------------------------
//    SETUP
// -----------------------------------------------------------------------------

void setup() {
    DEBUG.begin(115200);
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
    // 1. Leer batería
    float vbat = leerBateria();
    DEBUG.printf("[BATERÍA] %.2f V\n", vbat);
    
    // 2. Protección por batería baja
    if (!bateriaSaludable(vbat)) {
        DEBUG.println("[BATERÍA] ⚠️ Batería baja - Sleep 1h");
        radio.sleep();
        delay(50);
        esp_sleep_enable_timer_wakeup(3600ULL * 1000000ULL);
        esp_deep_sleep_start();
    }
    
    // 3. DATOS FIJOS (simulando sensores)
    float temp = 23.5;
    float ph   = 7.2;
    float ec   = 1250.0;
    float od   = 6.8;
    float orp  = 245.0;
    
    // 4. Construir payload
    construirPayload(temp, ph, ec, od, orp, vbat);
    
    // 5. Verificar sesión antes de enviar
    if (nodo.getDevAddr() == 0) {
        DEBUG.println("[LoRaWAN] ❌ Sesión inválida");
        delay(500);
        ESP.restart();
    }
    
    // 6. Enviar datos
    enviarDatos();
    
    // 7. Calcular intervalo dinámico según batería
    uint32_t minutos_sleep = calcularIntervaloSueno(vbat);
    DEBUG.printf("[SLEEP] %u minutos\n", minutos_sleep);
    
    // 8. Dormir
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
    int16_t batt_i  = (bateria < 3.0 || bateria > 4.9) ? -1 : (int16_t)(bateria * 100);
    int16_t temp_i  = (temp < -50 || temp > 100) ? -1 : (int16_t)(temp * 10);
    int16_t ph_i    = (ph < 0 || ph > 14) ? -1 : (int16_t)(ph * 100);
    int16_t ec_i    = (ec < 0 || ec > 20000) ? -1 : (int16_t)(ec * 10);
    int16_t od_i    = (od < 0 || od > 20) ? -1 : (int16_t)(od * 100);  // Cambiado a *100 para más precisión
    int16_t orp_i   = (orp < -2000 || orp > 2000) ? -1 : (int16_t)orp;
    
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
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    
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