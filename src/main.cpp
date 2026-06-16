/**
 * @file main.cpp
 * @brief Entry point — Adaptive Smart Traffic Light with Emergency Vehicle Preemption.
 *        FreeRTOS ESP32, Wokwi simulation, Percepio Tracealyzer instrumentation.
 *
 * ============================================================
 * REFERENSI LAPORAN:
 *   • Section 4.7 — Integrasi Tema Kontemporer: Multicore & Core Affinity
 *   • Tabel 4.2   — Task parameters (periode, prioritas, stack)
 *   • Section 4.4.3 — Priority Inheritance via xSemaphoreCreateMutex()
 *
 * ARSITEKTUR MULTICORE (Section 4.7):
 *
 *   Core 0 (PRO_CPU) — Arduino loop() + IDF WiFi/BT stack:
 *     • TaskMonitoring (P=2) — non-kritis, boleh di-preempt lebih sering
 *
 *   Core 1 (APP_CPU) — Task real-time:
 *     • TaskEmergencyHandler (P=5) — sporadic, latensi kritis
 *     • TaskTrafficController (P=4) — periodik 20ms
 *     • TaskTrafficLight (P=3) — periodik 100ms
 *
 *   Alasan ISR dipasang di Core 1:
 *     - setup() dan loop() berjalan di Core 1 pada ESP32-Arduino.
 *     - attachInterrupt() memasang ISR pada core yang memanggil fungsi tsb.
 *     - ISR_Emergency harus di Core 1 agar portYIELD_FROM_ISR() langsung
 *       men-schedule TaskEmergencyHandler (P=5) di core yang sama,
 *       meminimalkan inter-core IPC overhead dan memenuhi NFR-02 ≤ 100 ms.
 *
 * CATATAN WOKWI + DUAL-CORE:
 *   Wokwi mensimulasikan ESP32 dual-core secara logis, tetapi eksekusi
 *   di simulator bersifat single-thread (tidak parallel sejati).
 *   → Timing WCET yang terukur di Wokwi lebih optimistis dari hardware asli.
 *   → Verifikasi NFR-01, NFR-02, NFR-03 wajib dilakukan pada hardware ESP32 fisik.
 *
 * CATATAN TRACEALYZER:
 *   1. Pastikan library TraceRecorder sudah di-build (cek platformio.ini).
 *   2. vTraceEnable(TRC_START) dipanggil SEBELUM xTaskCreate agar semua
 *      task events ter-capture dari awal.
 *   3. Streaming via Serial: data binary keluar bersama Serial.print() biasa.
 *      Gunakan dedicated Serial port atau filter di Tracealyzer host software.
 *      Alternatif: nonaktifkan Serial.print() debug saat capture Tracealyzer.
 *   4. Tambahkan di platformio.ini jika perlu:
 *        build_flags = -D TRC_CFG_INCLUDE_USER_EVENTS=1
 *
 * ============================================================
 * CHECKLIST VERIFIKASI MANUAL SETELAH BUILD:
 *
 * [ ] 1. PlatformIO build sukses tanpa error/warning kritis
 * [ ] 2. firmware.bin terbentuk di .pio/build/esp32dev/
 * [ ] 3. Wokwi simulator bisa load firmware.bin (F1 → Wokwi: Start)
 * [ ] 4. Serial Monitor menampilkan pesan init semua task
 * [ ] 5. LED berputar urutan N→E→S→W di Wokwi
 * [ ] 6. Tombol biru (kendaraan normal) mempercepat/memperlambat fase hijau
 * [ ] 7. Tombol merah (emergency) memicu preemption segera
 * [ ] 8. LCD I2C menampilkan data bergantian tiap 500ms
 * [ ] 9. Stack HWM tidak muncul warning < 20% setelah 5 menit simulasi
 * [ ] 10. TraceRecorder: cek apakah binary stream muncul di Serial
 *         (Wokwi mungkin tidak support Tracealyzer streaming langsung —
 *          gunakan mode SNAPSHOT jika streaming bermasalah:
 *          ubah TRC_CFG_RECORDER_MODE=1 di platformio.ini)
 * [ ] 11. VERIFIKASI DI HARDWARE FISIK: NFR-01 (≤5ms), NFR-02 (≤100ms)
 *         menggunakan logic analyzer atau Tracealyzer streaming via USB-UART
 * [ ] 12. Wokwi dual-core: verifikasi bahwa Core 0 menjalankan TaskMonitoring
 *         (lihat Serial output "[MON] started (Core 0...)")
 * ============================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "config.h"
#include "sync_objects.h"
#include "tasks.h"

#if defined(MABUTRACE_ENABLED)
#include <mabutrace.h>
#endif

/* ── Tracealyzer ─────────────────────────────────────────── */
#if defined(TRC_CFG_RECORDER_MODE)
  #include "trcRecorder.h"
  /* Channel handles untuk vTracePrint() di masing-masing modul */
  traceString trcVehicleISRChannel;
  traceString trcEmergencyISRChannel;
  traceString trcControllerChannel;
  traceString trcEmergencyChannel;
  traceString trcLightChannel;
  traceString trcMonitoringChannel;
#endif

/* ─────────────────────────────────────────────────────────────
 * DEFINISI VARIABEL GLOBAL (deklarasi extern ada di sync_objects.h)
 * ──────────────────────────────────────────────────────────── */

/* ── Objek sinkronisasi ── */
QueueHandle_t     vehicleQueue   = NULL;
SemaphoreHandle_t emergencySem   = NULL;
SemaphoreHandle_t stateMutex     = NULL;
SemaphoreHandle_t monitorMutex   = NULL;

/* ── Shared state (volatile karena diakses dari ISR & multiple task) ── */
volatile TrafficState   gTrafficState   = {
    .activeLane           = LANE_NORTH,
    .lanePhase            = LIGHT_RED,
    .greenDuration_ms     = BASE_GREEN_MS,
    .greenRemaining_ms    = BASE_GREEN_MS,
    .emergencyActive      = false,
    .emergencyLane        = LANE_NORTH,
    .savedLane            = LANE_NORTH,
    .savedPhase           = LIGHT_RED,
    .savedGreenRemaining_ms = BASE_GREEN_MS
};

volatile MonitoringData gMonitoringData = {
    .vehicleCount   = {0, 0, 0, 0},
    .lightStatus    = {LIGHT_RED, LIGHT_RED, LIGHT_RED, LIGHT_RED},
    .emergencyActive = false,
    .emergencyLane  = LANE_NORTH,
    .updateCount    = 0
};

volatile Lane     gEmergencyLane = LANE_NORTH;
portMUX_TYPE      gEmergencyMux  = portMUX_INITIALIZER_UNLOCKED;

/* ── LCD instance (dipakai di task_monitoring.cpp) ── */
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);

/* ─────────────────────────────────────────────────────────────
 * initGPIO() — Konfigurasi semua pin
 * ──────────────────────────────────────────────────────────── */
void initGPIO(void) {
    /* ── LED: Output ── */
    pinMode(PIN_LED_N_RED, OUTPUT); digitalWrite(PIN_LED_N_RED, LOW);
    pinMode(PIN_LED_N_YEL, OUTPUT); digitalWrite(PIN_LED_N_YEL, LOW);
    pinMode(PIN_LED_N_GRN, OUTPUT); digitalWrite(PIN_LED_N_GRN, LOW);

    pinMode(PIN_LED_S_RED, OUTPUT); digitalWrite(PIN_LED_S_RED, LOW);
    pinMode(PIN_LED_S_YEL, OUTPUT); digitalWrite(PIN_LED_S_YEL, LOW);
    pinMode(PIN_LED_S_GRN, OUTPUT); digitalWrite(PIN_LED_S_GRN, LOW);

    pinMode(PIN_LED_E_RED, OUTPUT); digitalWrite(PIN_LED_E_RED, LOW);
    pinMode(PIN_LED_E_YEL, OUTPUT); digitalWrite(PIN_LED_E_YEL, LOW);
    pinMode(PIN_LED_E_GRN, OUTPUT); digitalWrite(PIN_LED_E_GRN, LOW);

    pinMode(PIN_LED_W_RED, OUTPUT); digitalWrite(PIN_LED_W_RED, LOW);
    pinMode(PIN_LED_W_YEL, OUTPUT); digitalWrite(PIN_LED_W_YEL, LOW);
    pinMode(PIN_LED_W_GRN, OUTPUT); digitalWrite(PIN_LED_W_GRN, LOW);

    /* ── Tombol Normal: INPUT_PULLUP (active-low) atau INPUT_PULLDOWN (active-high) ── */
#if BUTTON_ACTIVE_LOW
    pinMode(PIN_BTN_N, INPUT_PULLUP);
    pinMode(PIN_BTN_S, INPUT_PULLUP);
    pinMode(PIN_BTN_E, INPUT_PULLUP);
    pinMode(PIN_BTN_W, INPUT_PULLUP);
#else
    pinMode(PIN_BTN_N, INPUT_PULLDOWN);
    pinMode(PIN_BTN_S, INPUT_PULLDOWN);
    pinMode(PIN_BTN_E, INPUT_PULLDOWN);
    pinMode(PIN_BTN_W, INPUT_PULLDOWN);
#endif

    /* ── Tombol Emergency: INPUT (resistor eksternal wajib) ── */
    pinMode(PIN_BTN_EMG_N, INPUT);
    pinMode(PIN_BTN_EMG_S, INPUT);
    pinMode(PIN_BTN_EMG_E, INPUT);
    pinMode(PIN_BTN_EMG_W, INPUT);

    Serial.println("[INIT] GPIO configured (12 LEDs + 8 buttons).");
}

/* ─────────────────────────────────────────────────────────────
 * initSyncObjects() — Buat semua objek FreeRTOS
 * ──────────────────────────────────────────────────────────── */
void initSyncObjects(void) {
    /* ── Queue kendaraan: kapasitas 10, item = Lane (4 byte) ── */
    vehicleQueue = xQueueCreate(10, sizeof(Lane));
    configASSERT(vehicleQueue != NULL);

    /* ── Binary semaphore untuk sinyal emergency (ISR → Task) ── */
    /* Gunakan xSemaphoreCreateBinary (BUKAN mutex) karena ini signaling */
    emergencySem = xSemaphoreCreateBinary();
    configASSERT(emergencySem != NULL);

    /* ── Mutex dengan Priority Inheritance untuk TrafficState ── */
    /* xSemaphoreCreateMutex() mendukung PI; binary semaphore tidak. */
    /* Ref: Section 4.4.3 — mencegah priority inversion P=3 vs P=5    */
    stateMutex = xSemaphoreCreateMutex();
    configASSERT(stateMutex != NULL);

    /* ── Mutex untuk MonitoringData ── */
    monitorMutex = xSemaphoreCreateMutex();
    configASSERT(monitorMutex != NULL);

    Serial.println("[INIT] Sync objects created: 1 queue, 1 semaphore, 2 mutexes.");
}

/* ─────────────────────────────────────────────────────────────
 * setup() — Inisialisasi sistem (berjalan di Core 1)
 * ──────────────────────────────────────────────────────────── */
void setup() {
    /* ── 1. Serial Monitor ── */
    Serial.begin(115200);
#if defined(MABUTRACE_ENABLED)
    mabutrace_init();
#endif
    /* Tunggu Serial siap — timeout 2 detik untuk hardware ESP32.
     * Pada hardware, UART langsung ready; pada USB CDC board (ESP32-S2/S3),
     * mungkin butuh waktu. Timeout mencegah infinite hang. */
    {
        uint32_t serialWaitStart = millis();
        while (!Serial && (millis() - serialWaitStart < 2000)) { delay(10); }
    }
    delay(500); /* Beri waktu terminal terhubung */

    Serial.println("\n========================================");
    Serial.println(" Adaptive Smart Traffic Light System");
    Serial.println(" FreeRTOS ESP32 + Wokwi Simulation");
    Serial.println("========================================");

    /* ── 2. Tracealyzer — Inisialisasi SEBELUM task dibuat ── */
#if defined(TRC_CFG_RECORDER_MODE)
    vTraceEnable(TRC_INIT);  /* Inisialisasi buffer trace */

    /* Daftarkan channel string untuk vTracePrint() */
    trcVehicleISRChannel   = xTraceRegisterString("VehicleISR");
    trcEmergencyISRChannel = xTraceRegisterString("EmergencyISR");
    trcControllerChannel   = xTraceRegisterString("Controller");
    trcEmergencyChannel    = xTraceRegisterString("Emergency");
    trcLightChannel        = xTraceRegisterString("TrafficLight");
    trcMonitoringChannel   = xTraceRegisterString("Monitoring");

    Serial.println("[TRACE] Tracealyzer initialized (mode: STREAMING via Serial).");
    Serial.println("[TRACE] Note: Binary trace data mixed with Serial text output.");
    Serial.println("[TRACE] For clean capture: disable Serial.print() debug messages.");
#else
    Serial.println("[TRACE] Tracealyzer NOT compiled (TRC_CFG_RECORDER_MODE not defined).");
#endif

    /* ── 3. GPIO ── */
    initGPIO();

    /* ── 4. Sync objects ── */
    initSyncObjects();

    /* ── 5. LCD I2C ── */
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    lcd.begin(LCD_COLS, LCD_ROWS);
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Traffic System  ");
    lcd.setCursor(0, 1);
    lcd.print("Initializing... ");
    Serial.println("[INIT] LCD I2C initialized at address 0x27.");

    /* ── 6. Attach ISR (setelah GPIO dan sync objects siap) ── */
    /* ISR dipasang di Core 1 (karena setup() berjalan di Core 1)  */
    /* Ref: Section 4.7 — ISR Emergency harus Core 1 untuk minimal */
    /* inter-core overhead saat xSemaphoreGiveFromISR() dipanggil.  */
    attachInterrupts();

    /* ── 7. Buat task dengan xTaskCreatePinnedToCore() ── */
    /* Ref: Section 4.7 — Core affinity untuk determinisme timing  */

    BaseType_t ret;

    /* TaskEmergencyHandler → Core 1, P=5 */
    ret = xTaskCreatePinnedToCore(
        TaskEmergencyHandler,       /* Fungsi task */
        "EmergencyHandler",         /* Nama (untuk debugging) */
        STACK_EMERGENCY,            /* Stack dalam WORD (1 word = 4 byte di ESP32) */
        NULL,                       /* Parameter */
        PRIORITY_EMERGENCY,         /* Prioritas = 5 */
        NULL,                       /* Handle (tidak dipakai) */
        CORE_APP                    /* Core 1 */
    );
    configASSERT(ret == pdPASS);

    /* TaskTrafficController → Core 1, P=4 */
    ret = xTaskCreatePinnedToCore(
        TaskTrafficController,
        "TrafficController",
        STACK_CONTROLLER,
        NULL,
        PRIORITY_CONTROLLER,        /* Prioritas = 4 */
        NULL,
        CORE_APP                    /* Core 1 */
    );
    configASSERT(ret == pdPASS);

    /* TaskTrafficLight → Core 1, P=3 */
    ret = xTaskCreatePinnedToCore(
        TaskTrafficLight,
        "TrafficLight",
        STACK_LIGHT,
        NULL,
        PRIORITY_LIGHT,             /* Prioritas = 3 */
        NULL,
        CORE_APP                    /* Core 1 */
    );
    configASSERT(ret == pdPASS);

    /* TaskMonitoring → Core 0, P=2 */
    /* Core 0 (PRO_CPU) untuk isolasi dari task real-time di Core 1.       */
    /* Monitoring boleh di-preempt oleh IDF internal tasks di Core 0.      */
    /* Ini acceptable karena Monitoring bukan safety-critical path.         */
    ret = xTaskCreatePinnedToCore(
        TaskMonitoring,
        "Monitoring",
        STACK_MONITORING,
        NULL,
        PRIORITY_MONITORING,        /* Prioritas = 2 */
        NULL,
        CORE_PRO                    /* Core 0 */
    );
    configASSERT(ret == pdPASS);

    Serial.println("[INIT] All 4 tasks created successfully.");
    Serial.println("[INIT] Scheduler running. setup() complete.\n");

    /* ── 8. Start Tracealyzer recording ── */
#if defined(TRC_CFG_RECORDER_MODE)
    vTraceEnable(TRC_START);
    Serial.println("[TRACE] Trace recording STARTED.");
#endif

    /* LCD splash selesai */
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("System Ready!   ");
    lcd.setCursor(0, 1);
    lcd.print("Tasks: 4 active ");
}

/* ─────────────────────────────────────────────────────────────
 * loop() — Kosong; semua kerja dilakukan oleh FreeRTOS tasks.
 *
 * Catatan: loop() berjalan sebagai task Arduino di Core 1 dengan
 * prioritas 1 (lebih rendah dari semua task kita).
 * vTaskDelay panjang memastikan loop() tidak membuang CPU time.
 * ──────────────────────────────────────────────────────────── */
void loop() {
    /* Semua logic ada di FreeRTOS tasks — loop() tidak melakukan apa-apa */
    vTaskDelay(pdMS_TO_TICKS(10000)); /* Tidur 10 detik, hampir tidak pernah jalan */
}