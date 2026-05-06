// ============================================================
//  Sistema de monitoreo Parkinson — ESP32
//  Sensores: MPU6050 (IMU) + MyoWare (EMG)
//  Protocolo: MQTT sobre WiFi
// ============================================================

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <MPU6050.h>
#include <time.h>

// ===== CONFIGURACION =========================================
// En produccion, mover estas credenciales a un archivo
// separado que NO se suba a git (.gitignore).
#define SSID        "PARKINSON"
#define PASS        "P4RK1NS0N"
#define MQTT_HOST   "192.168.0.100"
#define MQTT_PORT   1883

#define DEVICE_ID   "esp32_01"
#define PATIENT_ID  "patient_01"

#define PIN_EMG     34

// Frecuencias de muestreo:
//   EMG necesita >=1000 Hz (señal de hasta 500 Hz — Teorema de Nyquist)
//   IMU: 100 Hz es suficiente para temblor Parkinson (3-12 Hz)
//   Envio MQTT: 50 Hz (cada 20 ms)
#define EMG_INTERVAL_US   1000   // 1000 Hz  (en microsegundos)
#define IMU_INTERVAL_MS     10   // 100  Hz
#define MQTT_INTERVAL_MS    20   //  50  Hz

// Escalas del MPU6050 en configuracion por defecto
#define ACCEL_SCALE  16384.0f   // LSB/g  para rango ±2g
#define GYRO_SCALE     131.0f   // LSB/(°/s) para rango ±250°/s
#define ADC_MAX       4095.0f   // 12 bits
#define VREF             3.3f   // Voltaje de referencia ESP32 (V)

// NTP — ajustar segun zona horaria del paciente
#define NTP_SERVER   "pool.ntp.org"
#define UTC_OFFSET   (-5 * 3600)   // Colombia UTC-5


// ===== BUFFER CIRCULAR PARA EMG ==============================
// Guarda las ultimas N muestras para calcular RMS de ventana corta.
// 20 muestras a 1000 Hz = ventana de 20 ms.
#define EMG_BUF_SIZE  20
static uint16_t emg_buf[EMG_BUF_SIZE];
static uint8_t  emg_idx = 0;

// Ultimo valor EMG en mV y RMS de la ventana
static float emg_last_mv = 0.0f;
static float emg_rms_mv  = 0.0f;

// ===== FILTRO COMPLEMENTARIO (orientacion) ===================
static float roll  = 0.0f;
static float pitch = 0.0f;
static const float ALPHA = 0.96f;   // 96% giroscopio / 4% acelerometro
static unsigned long lastFilterUs = 0;

// ===== DATOS IMU (compartidos entre funciones) ===============
static float imu_ax = 0, imu_ay = 0, imu_az = 0;
static float imu_gx = 0, imu_gy = 0, imu_gz = 0;
static float imu_mag = 0;   // magnitud del acelerometro en g

// ===== CONTROL DE TIEMPO =====================================
static unsigned long lastEmgUs   = 0;
static unsigned long lastImuMs   = 0;
static unsigned long lastMqttMs  = 0;

// ===== SESION ================================================
static char session_id[32];
static char topic_data[64];

// ===== OBJETOS ===============================================
WiFiClient   espClient;
PubSubClient mqtt(espClient);
MPU6050      mpu;


// ─────────────────────────────────────────────────────────────
// WiFi: conexion inicial con timeout (no reinicia el ESP).
// En loop() se reintenta si se pierde la señal.
static void setup_wifi() {
  Serial.printf("[WiFi] Conectando a %s...\n", SSID);
  WiFi.begin(SSID, PASS);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.isConnected()) {
    Serial.printf("\n[WiFi] OK — IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    // No reiniciamos: el sistema sigue midiendo aunque no haya red.
    // Los datos se perderan hasta reconectar (mejora futura: buffer en SPIFFS).
    Serial.println("\n[WiFi] Timeout — continua sin red");
  }
}

// ─────────────────────────────────────────────────────────────
// MQTT: intento de reconexion no bloqueante.
// Se llama desde loop() pero solo ejecuta cada 5 segundos.
static void try_reconnect_mqtt() {
  static unsigned long lastAttempt = 0;
  if (millis() - lastAttempt < 5000) return;
  lastAttempt = millis();

  char clientId[32];
  snprintf(clientId, sizeof(clientId), "ESP32-%08X", (uint32_t)(ESP.getEfuseMac()));

  if (mqtt.connect(clientId)) {
    Serial.println("[MQTT] Conectado");
  } else {
    Serial.printf("[MQTT] Fallo rc=%d — reintento en 5s\n", mqtt.state());
  }
}

// ─────────────────────────────────────────────────────────────
// EMG: muestreo a 1000 Hz en buffer circular.
// Convierte el valor ADC a milivolts (mV).
static void sample_emg() {
  unsigned long now_us = micros();
  if (now_us - lastEmgUs < EMG_INTERVAL_US) return;
  lastEmgUs = now_us;

  uint16_t raw = (uint16_t)analogRead(PIN_EMG);
  emg_buf[emg_idx] = raw;
  emg_idx = (emg_idx + 1) % EMG_BUF_SIZE;

  // Convertir ultima muestra a mV
  emg_last_mv = (raw / ADC_MAX) * VREF * 1000.0f;
}

// ─────────────────────────────────────────────────────────────
// EMG: calcula el RMS de las N muestras del buffer en mV.
// RMS es la metrica estandar de "intensidad" de la señal EMG.
static void compute_emg_rms() {
  float sum_sq = 0.0f;
  for (int i = 0; i < EMG_BUF_SIZE; i++) {
    float v_mv = (emg_buf[i] / ADC_MAX) * VREF * 1000.0f;
    sum_sq += v_mv * v_mv;
  }
  emg_rms_mv = sqrtf(sum_sq / EMG_BUF_SIZE);
}

// ─────────────────────────────────────────────────────────────
// Filtro complementario: fusiona acelerometro + giroscopio
// para obtener roll y pitch estables en grados.
static void update_filter(float ax, float ay, float az, float gx, float gy) {
  unsigned long now_us = micros();
  float dt = (now_us - lastFilterUs) / 1e6f;
  lastFilterUs = now_us;
  if (dt <= 0.0f || dt > 0.5f) return;

  float roll_acc  = atan2f(ay, az) * 180.0f / PI;
  float pitch_acc = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / PI;

  roll  = ALPHA * (roll  + gx * dt) + (1.0f - ALPHA) * roll_acc;
  pitch = ALPHA * (pitch + gy * dt) + (1.0f - ALPHA) * pitch_acc;
}

// ─────────────────────────────────────────────────────────────
// IMU: lectura y conversion a unidades fisicas.
// La magnitud del acelerometro (imu_mag) es util para ML:
// en reposo vale ~1.0g; el desvio detecta movimiento/temblor.
static void read_imu() {
  int16_t ax_r, ay_r, az_r, gx_r, gy_r, gz_r;
  mpu.getMotion6(&ax_r, &ay_r, &az_r, &gx_r, &gy_r, &gz_r);

  imu_ax = ax_r / ACCEL_SCALE;
  imu_ay = ay_r / ACCEL_SCALE;
  imu_az = az_r / ACCEL_SCALE;
  imu_gx = gx_r / GYRO_SCALE;
  imu_gy = gy_r / GYRO_SCALE;
  imu_gz = gz_r / GYRO_SCALE;
  imu_mag = sqrtf(imu_ax*imu_ax + imu_ay*imu_ay + imu_az*imu_az);

  update_filter(imu_ax, imu_ay, imu_az, imu_gx, imu_gy);
}

// ─────────────────────────────────────────────────────────────
// Publica un JSON con todos los datos de la muestra actual.
static void publish_data(unsigned long now_ms) {
  compute_emg_rms();

  // Timestamp: si hay NTP disponible, usar epoch Unix (ms).
  // Esto permite correlacionar sesiones en la base de datos.
  time_t epoch = time(nullptr);
  unsigned long ts = (epoch > 1700000000UL)
                     ? (unsigned long)epoch * 1000UL + (now_ms % 1000UL)
                     : now_ms;

  char payload[620];
  int len = snprintf(payload, sizeof(payload),
    "{"
      "\"device_id\":\"%s\","
      "\"patient_id\":\"%s\","
      "\"session_id\":\"%s\","
      "\"ts\":%lu,"
      "\"emg\":{\"mv\":%.2f,\"rms_mv\":%.2f},"
      "\"accel\":{\"x\":%.4f,\"y\":%.4f,\"z\":%.4f,\"mag\":%.4f},"
      "\"gyro\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},"
      "\"orientation\":{\"roll\":%.2f,\"pitch\":%.2f}"
    "}",
    DEVICE_ID, PATIENT_ID, session_id,
    ts,
    emg_last_mv, emg_rms_mv,
    imu_ax, imu_ay, imu_az, imu_mag,
    imu_gx, imu_gy, imu_gz,
    roll, pitch
  );

  if (len >= (int)sizeof(payload)) {
    Serial.println("[JSON] ADVERTENCIA: payload truncado — aumentar buffer");
    return;
  }

  if (mqtt.connected()) {
    if (!mqtt.publish(topic_data, payload)) {
      Serial.println("[MQTT] Publish fallo — buffer MQTT lleno?");
    }
  }
}


// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(200);

  // ADC: 12 bits (0-4095), rango 0-3.3V
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  setup_wifi();

  // Sincronizar hora real por NTP
  if (WiFi.isConnected()) {
    configTime(UTC_OFFSET, 0, NTP_SERVER);
    Serial.print("[NTP] Sincronizando");
    struct tm t;
    if (getLocalTime(&t, 6000)) {
      Serial.printf(" OK — %02d:%02d:%02d\n", t.tm_hour, t.tm_min, t.tm_sec);
    } else {
      Serial.println(" fallo — usando millis()");
    }
  }

  // Session ID: MAC del dispositivo + tiempo de arranque.
  // Unico por dispositivo y por sesion.
  uint32_t mac_part = (uint32_t)(ESP.getEfuseMac() & 0xFFFFFFFF);
  time_t epoch = time(nullptr);
  if (epoch > 1700000000UL) {
    snprintf(session_id, sizeof(session_id), "%08X_%lu", mac_part, (unsigned long)epoch);
  } else {
    snprintf(session_id, sizeof(session_id), "%08X_%lu", mac_part, millis());
  }
  snprintf(topic_data, sizeof(topic_data), "parkinson/%s/data", PATIENT_ID);

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setBufferSize(640);

  Wire.begin();
  mpu.initialize();

  // DLPF (filtro digital interno del MPU6050):
  // Modo 4 = 20 Hz de ancho de banda.
  // Elimina vibracion mecanica > 20 Hz y reduce aliasing.
  // El temblor de Parkinson esta en 3-12 Hz, queda dentro.
  mpu.setDLPFMode(MPU6050_DLPF_BW_20);

  if (!mpu.testConnection()) {
    Serial.println("[MPU6050] ERROR: sensor no detectado — verificar SDA/SCL");
  } else {
    Serial.println("[MPU6050] OK");
  }

  Serial.println("[MPU6050] Calibrando — mantener quieto ~3 segundos...");
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  Serial.println("[MPU6050] Calibracion completa");

  lastFilterUs = micros();
  lastEmgUs    = micros();

  Serial.printf("[Sistema] Listo | Sesion: %s\n", session_id);
  Serial.printf("[Sistema] Topic MQTT: %s\n", topic_data);
}

// ─────────────────────────────────────────────────────────────
void loop() {
  unsigned long now_ms = millis();

  // 1. EMG — maxima prioridad: muestrear a 1000 Hz
  //    (no bloqueante, regresa inmediatamente si no toca)
  sample_emg();

  // 2. Mantener conexiones — ninguno bloquea el loop
  if (WiFi.status() != WL_CONNECTED) {
    static unsigned long lastWifiRetry = 0;
    if (now_ms - lastWifiRetry > 10000) {
      lastWifiRetry = now_ms;
      Serial.println("[WiFi] Reconectando...");
      WiFi.reconnect();
    }
  }
  if (!mqtt.connected() && WiFi.isConnected()) {
    try_reconnect_mqtt();
  }
  mqtt.loop();

  // 3. IMU — 100 Hz
  if (now_ms - lastImuMs >= IMU_INTERVAL_MS) {
    lastImuMs = now_ms;
    read_imu();
  }

  // 4. Publicar por MQTT — 50 Hz
  if (now_ms - lastMqttMs >= MQTT_INTERVAL_MS) {
    lastMqttMs = now_ms;
    publish_data(now_ms);
  }
}
