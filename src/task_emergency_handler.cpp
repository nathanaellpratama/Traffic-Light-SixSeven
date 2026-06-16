/**
 * @file task_emergency_handler.cpp
 * @brief Implementasi TaskEmergencyHandler — Preemption kendaraan darurat.
 *
 * ============================================================
 * REFERENSI LAPORAN:
 *   • Tabel 4.1  — Task: EmergencyHandler, P=5 (tertinggi), sporadic,
 *                  T=100ms (periode atas), C=2ms WCET
 *   • Section 4.2.3 — Sporadic task model: diaktifkan oleh event (semaphore),
 *                     bukan timer periodik. Worst-case period = 100 ms.
 *   • FR-02 — Kendaraan darurat mendapat lampu hijau segera saat terdeteksi.
 *   • FR-07 — Setelah emergency selesai, sistem kembali ke fase sebelum preemption.
 *   • NFR-02 (Tabel 3.2) — End-to-end latency dari tombol ditekan hingga
 *                           LED darurat menyala ≤ 100 ms.
 *   • Section 4.4.3 — Priority Inheritance via stateMutex (xSemaphoreCreateMutex)
 *
 * ALUR EKSEKUSI:
 *   ISR_Emergency → xSemaphoreGiveFromISR(emergencySem)
 *      → TaskEmergencyHandler di-unblock (P=5, preempt semua task lain)
 *          → Ambil stateMutex
 *          → Save state saat ini (FR-07)
 *          → Set jalur darurat = GREEN, semua jalur lain = RED
 *          → Set emergencyActive = true
 *          → Release stateMutex
 *          → Update monitorMutex
 *          → Aktuasi LED darurat (via TaskTrafficLight berprioritas lebih rendah,
 *            ATAU langsung digitalWrite dari sini — pilih langsung untuk
 *            memenuhi NFR-02 ≤ 100 ms tanpa menunggu TaskTrafficLight scheduled)
 *          → vTaskDelay(EMERGENCY_HOLD_MS)  — "kendaraan melewati persimpangan"
 *          → Ambil stateMutex
 *          → Restore saved state, set emergencyActive = false
 *          → Release stateMutex
 *          → Kembali blocking pada emergencySem
 *
 * CATATAN TRACEALYZER:
 *   Ukur timestamp antara semaphore diterima (t1) dan LED darurat menyala (t2).
 *   Delta (t2 - t1) harus ≤ 100 ms (NFR-02).
 * ============================================================
 */

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "config.h"
#include "sync_objects.h"
#include "tasks.h"

#if defined(MABUTRACE_ENABLED)
#include <mabutrace.h>
#endif

/* ── Tracealyzer ─────────────────────────────────────────── */
#if defined(TRC_CFG_RECORDER_MODE)
  #include "trcRecorder.h"
  extern traceString trcEmergencyChannel;
  #define TRACE_EMG(msg)  vTracePrint(trcEmergencyChannel, (msg))
#else
  #define TRACE_EMG(msg)  /* no-op */
#endif

/* ─────────────────────────────────────────────────────────────
 * HELPER — Set semua LED ke state yang sesuai dengan kondisi emergency.
 * Dipanggil LANGSUNG dari TaskEmergencyHandler untuk memenuhi NFR-02.
 *
 * Catatan: Operasi digitalWrite tidak membutuhkan mutex karena:
 *   1. TaskTrafficLight (P=3) akan di-preempt oleh task ini (P=5).
 *   2. Setelah Emergency set LED, TaskTrafficLight membaca emergencyActive=true
 *      dan tidak akan override LED (lihat task_traffic_light.cpp).
 * ──────────────────────────────────────────────────────────── */
static const uint8_t PIN_LED_RED[LANE_COUNT]   = { PIN_LED_N_RED,  PIN_LED_E_RED,  PIN_LED_S_RED,  PIN_LED_W_RED  };
static const uint8_t PIN_LED_YEL[LANE_COUNT]   = { PIN_LED_N_YEL,  PIN_LED_E_YEL,  PIN_LED_S_YEL,  PIN_LED_W_YEL  };
static const uint8_t PIN_LED_GRN[LANE_COUNT]   = { PIN_LED_N_GRN,  PIN_LED_E_GRN,  PIN_LED_S_GRN,  PIN_LED_W_GRN  };

/**
 * @brief Set output LED untuk semua jalur sesuai array lightColor[].
 * @param lightColor Array LightColor per jalur [LANE_COUNT]
 */
static void setAllLEDs(const LightColor lightColor[LANE_COUNT]) {
    for (int i = 0; i < LANE_COUNT; i++) {
        digitalWrite(PIN_LED_RED[i], (lightColor[i] == LIGHT_RED)    ? HIGH : LOW);
        digitalWrite(PIN_LED_YEL[i], (lightColor[i] == LIGHT_YELLOW || lightColor[i] == LIGHT_YELLOW_TO_GREEN) ? HIGH : LOW);
        digitalWrite(PIN_LED_GRN[i], (lightColor[i] == LIGHT_GREEN)  ? HIGH : LOW);
    }
}

/* ─────────────────────────────────────────────────────────────
 * TASK IMPLEMENTATION
 * ──────────────────────────────────────────────────────────── */

void TaskEmergencyHandler(void *pvParameters) {
    (void)pvParameters;

    Serial.println("[EMG] TaskEmergencyHandler started (Core 1, P=5, sporadic).");

    for (;;) {
        /* ─────────────────────────────────────────────────────
         * BLOCKING WAIT — tunggu sinyal emergency dari ISR.
         * portMAX_DELAY = tunggu selamanya tanpa timeout.
         * Task ini tidak menggunakan CPU sama sekali saat tidak ada emergency.
         * ──────────────────────────────────────────────────── */
        xSemaphoreTake(emergencySem, portMAX_DELAY);

        {
#if defined(MABUTRACE_ENABLED)
            TRACE_SCOPE("TaskEmergencyHandler");
#endif

            /* ── [Tracealyzer] t1: semaphore diterima ── */
            TRACE_EMG("EMG_UNBLOCKED");
            TickType_t t1 = xTaskGetTickCount();

            /* ─────────────────────────────────────────────────────
             * AMBIL jalur emergency (tulis ISR, baca task, dilindungi mux)
             * ──────────────────────────────────────────────────── */
            Lane emergLane;
            portENTER_CRITICAL(&gEmergencyMux);
            emergLane = gEmergencyLane;
            portEXIT_CRITICAL(&gEmergencyMux);

            Serial.printf("[EMG] Emergency on lane %d! Preempting normal traffic.\n", emergLane);

            /* ─────────────────────────────────────────────────────
             * STEP 1: Ambil stateMutex, simpan state saat ini (FR-07)
             * Timeout 50 ms: jika tidak bisa ambil mutex, log error tapi lanjutkan.
             * ──────────────────────────────────────────────────── */
            TrafficState savedState;
            bool mutexOK = (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE);

            if (mutexOK) {
                /* ── Save state sebelum preemption (untuk restore FR-07) ── */
                savedState.activeLane           = gTrafficState.activeLane;
                savedState.lanePhase            = gTrafficState.lanePhase;
                savedState.greenRemaining_ms    = gTrafficState.greenRemaining_ms;
                savedState.greenDuration_ms     = gTrafficState.greenDuration_ms;

                /* ── Set emergency state awal ── */
                gTrafficState.emergencyActive   = true;
                gTrafficState.emergencyLane     = emergLane;

                xSemaphoreGive(stateMutex);
            } else {
                Serial.println("[EMG] ERROR: stateMutex timeout after 50ms! Emergency state may be inconsistent.");
                /* Fallback: tetap lanjutkan dengan best-effort */
                savedState.activeLane        = LANE_NORTH;
                savedState.lanePhase         = LIGHT_RED;
                savedState.greenRemaining_ms = BASE_GREEN_MS;
                savedState.greenDuration_ms  = BASE_GREEN_MS;
            }

            Lane prevActiveLane = savedState.activeLane;
            LightColor prevPhase = savedState.lanePhase;

            /* ─────────────────────────────────────────────────────
             * STEP 2: Transisi Emergency — seperti lalu lintas nyata
             *
             * Durasi kuning 1 detik agar terlihat jelas transisinya.
             * ──────────────────────────────────────────────────── */
            const uint32_t EMG_YELLOW_MS = 1000; // Kuning 1 detik saat emergency

            Serial.printf("[EMG] prevActiveLane=%d prevPhase=%d emergLane=%d\n",
                          (int)prevActiveLane, (int)prevPhase, (int)emergLane);

            if (prevActiveLane == emergLane && prevPhase == LIGHT_GREEN) {
                /* Jalur darurat sudah hijau — tidak perlu transisi apapun */
                Serial.println("[EMG] STEP2: Emergency lane already GREEN, no transition needed.");
                if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    gTrafficState.activeLane = emergLane;
                    gTrafficState.lanePhase  = LIGHT_GREEN;
                    xSemaphoreGive(stateMutex);
                }
                LightColor emergLEDState[LANE_COUNT];
                for (int i = 0; i < LANE_COUNT; i++) {
                    emergLEDState[i] = (i == (int)emergLane) ? LIGHT_GREEN : LIGHT_RED;
                }
                setAllLEDs(emergLEDState);

            } else {
                /* ── Transisi 1: Jalur aktif → KUNING → MERAH ── */
                if (prevPhase == LIGHT_GREEN || prevPhase == LIGHT_YELLOW_TO_GREEN) {
                    Serial.printf("[EMG] STEP2-T1: Lane %d GREEN -> YELLOW (%ums)\n",
                                  (int)prevActiveLane, EMG_YELLOW_MS);
                    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                        gTrafficState.activeLane = prevActiveLane;
                        gTrafficState.lanePhase  = LIGHT_YELLOW;
                        xSemaphoreGive(stateMutex);
                    }
                    LightColor yellowState[LANE_COUNT];
                    for (int i = 0; i < LANE_COUNT; i++) {
                        yellowState[i] = (i == (int)prevActiveLane) ? LIGHT_YELLOW : LIGHT_RED;
                    }
                    setAllLEDs(yellowState);
                    vTaskDelay(pdMS_TO_TICKS(EMG_YELLOW_MS));
                    Serial.printf("[EMG] STEP2-T1: Lane %d YELLOW -> RED\n", (int)prevActiveLane);
                }

                // Semua merah sebentar (pengosongan simpang)
                Serial.println("[EMG] STEP2: All RED (200ms clearance)");
                {
                    LightColor allRedState[LANE_COUNT];
                    for (int i = 0; i < LANE_COUNT; i++) {
                        allRedState[i] = LIGHT_RED;
                    }
                    setAllLEDs(allRedState);
                    vTaskDelay(pdMS_TO_TICKS(200));
                }

                /* ── Transisi 2: Jalur emergency MERAH → KUNING → HIJAU ── */
                Serial.printf("[EMG] STEP2-T2: Lane %d RED -> YELLOW (%ums)\n",
                              (int)emergLane, EMG_YELLOW_MS);
                if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    gTrafficState.activeLane = emergLane;
                    gTrafficState.lanePhase  = LIGHT_YELLOW_TO_GREEN;
                    xSemaphoreGive(stateMutex);
                }
                LightColor prepState[LANE_COUNT];
                for (int i = 0; i < LANE_COUNT; i++) {
                    prepState[i] = (i == (int)emergLane) ? LIGHT_YELLOW_TO_GREEN : LIGHT_RED;
                }
                setAllLEDs(prepState);
                vTaskDelay(pdMS_TO_TICKS(EMG_YELLOW_MS));

                // Hijaukan jalur emergency
                Serial.printf("[EMG] STEP2-T2: Lane %d YELLOW -> GREEN\n", (int)emergLane);
                if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    gTrafficState.activeLane = emergLane;
                    gTrafficState.lanePhase  = LIGHT_GREEN;
                    xSemaphoreGive(stateMutex);
                }
                LightColor greenState[LANE_COUNT];
                for (int i = 0; i < LANE_COUNT; i++) {
                    greenState[i] = (i == (int)emergLane) ? LIGHT_GREEN : LIGHT_RED;
                }
                setAllLEDs(greenState);
                Serial.printf("[EMG] STEP2: Emergency lane %d is now GREEN.\n", (int)emergLane);
            }

            /* ── [Tracealyzer] t2: LED menyala/transisi dimulai — hitung delta untuk NFR-02 ── */
            TRACE_EMG("EMG_LED_ON");
            TickType_t t2 = xTaskGetTickCount();
            uint32_t latencyMs = (t2 - t1) * portTICK_PERIOD_MS;
            Serial.printf("[EMG] LED transition latency: %u ms (NFR-02 limit: 100 ms) %s\n",
                          latencyMs, latencyMs <= 100 ? "OK" : "VIOLATION!");

            /* ─────────────────────────────────────────────────────
             * STEP 3: Update MonitoringData — set flag emergency
             * ──────────────────────────────────────────────────── */
            if (xSemaphoreTake(monitorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                gMonitoringData.emergencyActive = true;
                gMonitoringData.emergencyLane   = emergLane;
                xSemaphoreGive(monitorMutex);
            }

            /* ─────────────────────────────────────────────────────
             * STEP 4: Tahan selama EMERGENCY_HOLD_MS
             * ──────────────────────────────────────────────────── */
            TRACE_EMG("EMG_HOLDING");
            vTaskDelay(pdMS_TO_TICKS(EMERGENCY_HOLD_MS));

            /* ─────────────────────────────────────────────────────
             * STEP 5: Restore Transitions & State (FR-07)
             * ──────────────────────────────────────────────────── */
            TRACE_EMG("EMG_RESTORE");

            if (prevActiveLane == emergLane && prevPhase == LIGHT_GREEN) {
                // Jika tidak ada pergantian jalur, kembalikan langsung ke normal
                if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    gTrafficState.emergencyActive    = false;
                    gTrafficState.activeLane         = savedState.activeLane;
                    gTrafficState.lanePhase          = savedState.lanePhase;
                    gTrafficState.greenRemaining_ms  = savedState.greenRemaining_ms;
                    gTrafficState.greenDuration_ms   = savedState.greenDuration_ms;
                    xSemaphoreGive(stateMutex);
                }
                LightColor restoreLEDState[LANE_COUNT];
                for (int i = 0; i < LANE_COUNT; i++) {
                    restoreLEDState[i] = (i == (int)prevActiveLane) ? LIGHT_GREEN : LIGHT_RED;
                }
                setAllLEDs(restoreLEDState);
            } else {
                // 1. Kuningkan jalur emergency yang tadinya hijau
                if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    gTrafficState.activeLane = emergLane;
                    gTrafficState.lanePhase  = LIGHT_YELLOW;
                    xSemaphoreGive(stateMutex);
                }
                LightColor yellowState[LANE_COUNT];
                for (int i = 0; i < LANE_COUNT; i++) {
                    yellowState[i] = (i == (int)emergLane) ? LIGHT_YELLOW : LIGHT_RED;
                }
                setAllLEDs(yellowState);
                vTaskDelay(pdMS_TO_TICKS(YELLOW_MS));

                // 2. Set semua merah (All RED) untuk pengosongan simpang
                if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    gTrafficState.activeLane = emergLane;
                    gTrafficState.lanePhase  = LIGHT_RED;
                    xSemaphoreGive(stateMutex);
                }
                LightColor allRedState[LANE_COUNT];
                for (int i = 0; i < LANE_COUNT; i++) {
                    allRedState[i] = LIGHT_RED;
                }
                setAllLEDs(allRedState);
                vTaskDelay(pdMS_TO_TICKS(500)); // Buffer all-red selama 500ms

                // 3. Kuningkan persiapan (YEL->G) jalur normal yang akan aktif
                if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    gTrafficState.activeLane = prevActiveLane;
                    gTrafficState.lanePhase  = LIGHT_YELLOW_TO_GREEN;
                    xSemaphoreGive(stateMutex);
                }
                LightColor prepState[LANE_COUNT];
                for (int i = 0; i < LANE_COUNT; i++) {
                    prepState[i] = (i == (int)prevActiveLane) ? LIGHT_YELLOW_TO_GREEN : LIGHT_RED;
                }
                setAllLEDs(prepState);
                vTaskDelay(pdMS_TO_TICKS(YELLOW_TO_GREEN_MS));

                // 4. Kembalikan state ke normal dan aktifkan jalur hijau normal
                if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    gTrafficState.emergencyActive    = false;
                    gTrafficState.activeLane         = savedState.activeLane;
                    gTrafficState.lanePhase          = LIGHT_GREEN; // Mulai langsung di hijau
                    gTrafficState.greenRemaining_ms  = savedState.greenRemaining_ms;
                    gTrafficState.greenDuration_ms   = savedState.greenDuration_ms;
                    xSemaphoreGive(stateMutex);
                }
                LightColor normalGreenState[LANE_COUNT];
                for (int i = 0; i < LANE_COUNT; i++) {
                    normalGreenState[i] = (i == (int)prevActiveLane) ? LIGHT_GREEN : LIGHT_RED;
                }
                setAllLEDs(normalGreenState);
            }

            /* ─────────────────────────────────────────────────────
             * STEP 6: Update MonitoringData — clear flag emergency
             * ──────────────────────────────────────────────────── */
            if (xSemaphoreTake(monitorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                gMonitoringData.emergencyActive = false;
                xSemaphoreGive(monitorMutex);
            }

            TRACE_EMG("EMG_DONE");
            Serial.printf("[EMG] Emergency on lane %d cleared. Restoring normal operation.\n", emergLane);
        }

        /* Kembali ke blocking wait untuk emergency berikutnya */
    }

    vTaskDelete(NULL);
}