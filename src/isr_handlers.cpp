/**
 * @file isr_handlers.cpp
 * @brief Implementasi ISR untuk tombol kendaraan normal & darurat.
 *
 * ============================================================
 * REFERENSI LAPORAN:
 *   • Section 4.5.2 — Pola Deferred Interrupt Processing
 *   • NFR-01 (Tabel 3.2): Latensi ISR → task ≤ 5 ms
 *     → ISR hanya melakukan debounce + queue send (minimal overhead)
 *     → Semua pemrosesan lanjutan di-defer ke TaskTrafficController / TaskEmergencyHandler
 *   • NFR-02 (Tabel 3.2): Emergency end-to-end ≤ 100 ms
 *     → Semaphore give dari ISR → TaskEmergencyHandler blocking-take → langsung aksi
 *   • Section 4.7: Pertimbangan SMP dual-core
 *     → portENTER_CRITICAL_ISR diperlukan untuk atomic write di SMP
 * ============================================================
 *
 * CATATAN TRACEALYZER:
 *   Instrumentasi vTracePrint() di ISR digunakan untuk mengukur:
 *   (a) Waktu ISR dipicu (timestamp hardware interrupt)
 *   (b) Jarak ke task yang menerima event (NFR-01 measurement)
 *   vTracePrint() aman dipanggil dari ISR dalam mode Streaming.
 *   Untuk mode Snapshot, gunakan vTraceStoreISRBegin/End.
 * ============================================================
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "config.h"
#include "sync_objects.h"
#include "tasks.h"

/* ── Tracealyzer (kondisional, aktif jika library tersedia) ── */
#if defined(TRC_CFG_RECORDER_MODE)
  #include "trcRecorder.h"
  #define TRACE_ISR_VEHICLE(lane)    vTracePrint(trcVehicleISRChannel,   "VehicleISR lane=" #lane)
  #define TRACE_ISR_EMERGENCY(lane)  vTracePrint(trcEmergencyISRChannel, "EmergencyISR lane=" #lane)
  /* Channel handles — didefinisikan di main.cpp */
  extern traceString trcVehicleISRChannel;
  extern traceString trcEmergencyISRChannel;
#else
  #define TRACE_ISR_VEHICLE(lane)    /* no-op */
  #define TRACE_ISR_EMERGENCY(lane)  /* no-op */
#endif

/* ─────────────────────────────────────────────────────────────
 * DEBOUNCE — Array per-pin, volatile, dalam milidetik
 * Menggunakan millis() (aman dari ISR di ESP32, akses atomik 32-bit).
 * Alternative: xTaskGetTickCountFromISR() → kalikan PERIOD_MS/portTICK_PERIOD_MS
 * ──────────────────────────────────────────────────────────── */

/** Timestamp millis() terakhir ISR kendaraan normal dipicu, per jalur */
static volatile uint32_t lastVehicleISR_ms[LANE_COUNT]   = {0, 0, 0, 0};

/** Timestamp millis() terakhir ISR kendaraan darurat dipicu, per jalur */
static volatile uint32_t lastEmergencyISR_ms[LANE_COUNT] = {0, 0, 0, 0};

/* ─────────────────────────────────────────────────────────────
 * MACRO HELPER — Implementasi ISR kendaraan normal
 * Mengurangi duplikasi kode; di-expand menjadi fungsi IRAM_ATTR terpisah.
 * ──────────────────────────────────────────────────────────── */
#define IMPLEMENT_VEHICLE_ISR(funcName, lane)                              \
    void IRAM_ATTR funcName(void) {                                        \
        /* ── [NFR-01] Debounce: tolak jika < DEBOUNCE_MS dari last ISR ─ */ \
        uint32_t now = (uint32_t)millis();                                 \
        if ((now - lastVehicleISR_ms[(lane)]) < DEBOUNCE_MS) return;      \
        lastVehicleISR_ms[(lane)] = now;                                   \
                                                                           \
        /* ── Tracealyzer: catat event ISR untuk pengukuran latensi ───── */ \
        TRACE_ISR_VEHICLE(lane);                                           \
                                                                           \
        /* ── Kirim ID jalur ke vehicleQueue (non-blocking) ────────────── */ \
        Lane laneVal = (lane);                                             \
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;                    \
        xQueueSendFromISR(vehicleQueue, &laneVal, &xHigherPriorityTaskWoken); \
                                                                           \
        /* ── Yield jika ada task prioritas lebih tinggi yang siap ──────── */ \
        /* portYIELD_FROM_ISR = esp_task_wdt compatible, aman di SMP ───── */ \
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);                     \
    }

/* ─────────────────────────────────────────────────────────────
 * MACRO HELPER — Implementasi ISR kendaraan darurat
 * ──────────────────────────────────────────────────────────── */
#define IMPLEMENT_EMERGENCY_ISR(funcName, lane)                            \
    void IRAM_ATTR funcName(void) {                                        \
        /* ── [NFR-01] Debounce ─────────────────────────────────────────── */ \
        uint32_t now = (uint32_t)millis();                                 \
        if ((now - lastEmergencyISR_ms[(lane)]) < DEBOUNCE_MS) return;    \
        lastEmergencyISR_ms[(lane)] = now;                                 \
                                                                           \
        /* ── Tracealyzer: catat event ISR darurat ──────────────────────── */ \
        TRACE_ISR_EMERGENCY(lane);                                         \
                                                                           \
        /* ── Atomic write gEmergencyLane (SMP-safe) ────────────────────── */ \
        /* portENTER_CRITICAL_ISR disables interrupts pada CORE yang berjalan */ \
        /* Wajib di ESP32 SMP agar Core 1 tidak baca nilai setengah-ditulis   */ \
        portENTER_CRITICAL_ISR(&gEmergencyMux);                           \
        gEmergencyLane = (lane);                                           \
        portEXIT_CRITICAL_ISR(&gEmergencyMux);                            \
                                                                           \
        /* ── Signal TaskEmergencyHandler via binary semaphore ─────────── */ \
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;                    \
        xSemaphoreGiveFromISR(emergencySem, &xHigherPriorityTaskWoken);   \
                                                                           \
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);                     \
    }

/* ─────────────────────────────────────────────────────────────
 * EKSPANSI MACRO — 4 ISR kendaraan normal
 * ──────────────────────────────────────────────────────────── */
IMPLEMENT_VEHICLE_ISR(isrVehicle_N, LANE_NORTH)
IMPLEMENT_VEHICLE_ISR(isrVehicle_S, LANE_SOUTH)
IMPLEMENT_VEHICLE_ISR(isrVehicle_E, LANE_EAST)
IMPLEMENT_VEHICLE_ISR(isrVehicle_W, LANE_WEST)

/* ─────────────────────────────────────────────────────────────
 * EKSPANSI MACRO — 4 ISR kendaraan darurat
 * ──────────────────────────────────────────────────────────── */
IMPLEMENT_EMERGENCY_ISR(isrEmergency_N, LANE_NORTH)
IMPLEMENT_EMERGENCY_ISR(isrEmergency_S, LANE_SOUTH)
IMPLEMENT_EMERGENCY_ISR(isrEmergency_E, LANE_EAST)
IMPLEMENT_EMERGENCY_ISR(isrEmergency_W, LANE_WEST)

/* ─────────────────────────────────────────────────────────────
 * attachInterrupts() — Pasang semua ISR ke GPIO
 * Dipanggil dari setup() setelah initGPIO() dan initSyncObjects().
 *
 * Mode FALLING dipilih karena:
 *   - Tombol ditekan → pin HIGH→LOW (ext. pull-up atau internal pull-up)
 *   - FALLING lebih deterministic daripada CHANGE (satu event per tekan)
 *   - RISING bisa terpicu saat release (noise)
 *
 * Catatan Core: ESP32-Arduino menjalankan ISR pada core yang sama
 * dengan tempat attachInterrupt() dipanggil. setup() berjalan di Core 1
 * (APP_CPU), sehingga semua ISR ini berjalan di Core 1 — sama dengan
 * TaskEmergencyHandler, meminimalkan inter-core signaling overhead.
 * ──────────────────────────────────────────────────────────── */
void attachInterrupts(void) {
    /* Tentukan trigger edge berdasarkan BUTTON_ACTIVE_LOW */
#if BUTTON_ACTIVE_LOW
    const int triggerEdge = FALLING;
#else
    const int triggerEdge = RISING;
#endif

    /* ── ISR Kendaraan Normal ── */
    attachInterrupt(digitalPinToInterrupt(PIN_BTN_N), isrVehicle_N, triggerEdge);
    attachInterrupt(digitalPinToInterrupt(PIN_BTN_S), isrVehicle_S, triggerEdge);
    attachInterrupt(digitalPinToInterrupt(PIN_BTN_E), isrVehicle_E, triggerEdge);
    attachInterrupt(digitalPinToInterrupt(PIN_BTN_W), isrVehicle_W, triggerEdge);

    /* ── ISR Kendaraan Darurat ── */
#if defined(ENABLE_EMERGENCY_BUTTONS) && (ENABLE_EMERGENCY_BUTTONS == 1)
    attachInterrupt(digitalPinToInterrupt(PIN_BTN_EMG_N), isrEmergency_N, triggerEdge);
    attachInterrupt(digitalPinToInterrupt(PIN_BTN_EMG_S), isrEmergency_S, triggerEdge);
    attachInterrupt(digitalPinToInterrupt(PIN_BTN_EMG_E), isrEmergency_E, triggerEdge);
    attachInterrupt(digitalPinToInterrupt(PIN_BTN_EMG_W), isrEmergency_W, triggerEdge);
    Serial.printf("[ISR] 8 interrupt handlers attached (4 vehicle, 4 emergency, %s edge).\n",
                  BUTTON_ACTIVE_LOW ? "FALLING" : "RISING");
#else
    Serial.printf("[ISR] 4 interrupt handlers attached (4 vehicle, emergency disabled, %s edge).\n",
                  BUTTON_ACTIVE_LOW ? "FALLING" : "RISING");
#endif
}