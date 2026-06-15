/**
 * @file task_traffic_light.cpp
 * @brief Implementasi TaskTrafficLight — State machine aktuasi LED.
 *
 * ============================================================
 * REFERENSI LAPORAN:
 *   • Tabel 4.1  — Task: TrafficLight, P=3, T=100ms, C=2ms
 *   • Tabel 4.2  — Stack = 3072 byte = 768 word
 *   • FR-04 — Siklus lampu otomatis: GREEN → YELLOW → RED → (jalur berikut)
 *   • FR-06 — Mutual exclusion: HANYA SATU JALUR HIJAU pada satu waktu.
 *             Implementasi: state machine terpusat, satu task yang menulis LED,
 *             stateMutex untuk atomicity pembacaan greenDuration.
 *   • Section 4.6.3 — GPIO operations non-blocking (digitalWrite tidak block)
 *
 * STATE MACHINE NORMAL (saat emergencyActive == false):
 *
 *   currentPhase = GREEN
 *     ├── Hitung timeInPhase += PERIOD_LIGHT_MS setiap siklus
 *     ├── Jika timeInPhase >= greenRemaining_ms → transisi ke YELLOW
 *     │
 *   currentPhase = YELLOW
 *     ├── Tahan selama YELLOW_MS
 *     ├── Setelah YELLOW_MS → semua jalur RED, advance ke jalur berikutnya
 *     │
 *   currentPhase = RED (fase antar-jalur, set jalur aktif baru ke GREEN)
 *     └── Mulai fase GREEN untuk jalur berikutnya
 *
 * ROTASI JALUR: N(0) → E(1) → S(2) → W(3) → N(0) → ...
 *   (sesuai urutan enum Lane)
 *
 * EMERGENCY MODE (emergencyActive == true):
 *   - Skip state machine normal
 *   - Baca emergencyLane dari TrafficState
 *   - Set jalur darurat = GREEN, semua lain = RED
 *   - LED sudah di-set oleh TaskEmergencyHandler, task ini cukup mirror
 *     status ke MonitoringData
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
  extern traceString trcLightChannel;
  #define TRACE_LIGHT(msg)  vTracePrint(trcLightChannel, (msg))
#else
  #define TRACE_LIGHT(msg)  /* no-op */
#endif

/* ─────────────────────────────────────────────────────────────
 * TABEL PIN LED — indeks sesuai enum Lane (N=0, E=1, S=2, W=3)
 * ──────────────────────────────────────────────────────────── */
static const uint8_t PIN_LED_R[LANE_COUNT] = { PIN_LED_N_RED, PIN_LED_E_RED, PIN_LED_S_RED, PIN_LED_W_RED };
static const uint8_t PIN_LED_Y[LANE_COUNT] = { PIN_LED_N_YEL, PIN_LED_E_YEL, PIN_LED_S_YEL, PIN_LED_W_YEL };
static const uint8_t PIN_LED_G[LANE_COUNT] = { PIN_LED_N_GRN, PIN_LED_E_GRN, PIN_LED_S_GRN, PIN_LED_W_GRN };

/* Nama jalur untuk Serial log */
static const char* LANE_NAMES[LANE_COUNT] = { "NORTH", "EAST", "SOUTH", "WEST" };

/**
 * @brief Set output LED untuk satu jalur ke warna tertentu.
 *        Jalur lain tidak diubah (hanya jalur 'laneIdx' yang diset).
 *
 * @param laneIdx  Indeks jalur (0-3, sesuai enum Lane)
 * @param color    Warna yang diinginkan
 */
static void setLaneLED(int laneIdx, LightColor color) {
    digitalWrite(PIN_LED_R[laneIdx], (color == LIGHT_RED)    ? HIGH : LOW);
    digitalWrite(PIN_LED_Y[laneIdx], (color == LIGHT_YELLOW || color == LIGHT_YELLOW_TO_GREEN) ? HIGH : LOW);
    digitalWrite(PIN_LED_G[laneIdx], (color == LIGHT_GREEN)  ? HIGH : LOW);
}

/**
 * @brief Set SEMUA jalur ke RED (digunakan saat transisi antar jalur).
 *        Memenuhi FR-06: tidak ada dua jalur hijau bersamaan.
 */
static void setAllRed(void) {
    for (int i = 0; i < LANE_COUNT; i++) {
        setLaneLED(i, LIGHT_RED);
    }
}

/**
 * @brief Update MonitoringData dengan status lampu saat ini.
 * @param lightColors Array LightColor per jalur [LANE_COUNT]
 */
static void updateMonitorLightStatus(const LightColor lightColors[LANE_COUNT]) {
    if (xSemaphoreTake(monitorMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        for (int i = 0; i < LANE_COUNT; i++) {
            gMonitoringData.lightStatus[i] = lightColors[i];
        }
        xSemaphoreGive(monitorMutex);
    }
    /* Jika timeout: tidak update monitoring (tidak kritis) */
}

/* ─────────────────────────────────────────────────────────────
 * TASK IMPLEMENTATION
 * ──────────────────────────────────────────────────────────── */

void TaskTrafficLight(void *pvParameters) {
    (void)pvParameters;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(PERIOD_LIGHT_MS); /* 100 ms */

    /* ── State machine internal ── */
    Lane    currentLane  = LANE_NORTH;  /* Jalur yang sedang mendapat fase aktif */
    LightColor currentPhase = LIGHT_GREEN;  /* Fase saat ini */
    uint32_t timeInPhase_ms = 0;        /* Waktu yang telah dihabiskan di fase ini */
    uint32_t greenDuration_ms = BASE_GREEN_MS; /* Durasi hijau dari Controller */

    /* ── Inisialisasi awal: semua RED, lalu North GREEN ── */
    setAllRed();
    setLaneLED((int)currentLane, LIGHT_GREEN);

    /* ── Update TrafficState awal ── */
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        gTrafficState.activeLane        = currentLane;
        gTrafficState.lanePhase         = currentPhase;
        gTrafficState.greenDuration_ms  = greenDuration_ms;
        gTrafficState.greenRemaining_ms = greenDuration_ms;
        gTrafficState.emergencyActive   = false;
        xSemaphoreGive(stateMutex);
    }

    Serial.println("[LIGHT] TaskTrafficLight started (Core 1, P=3, T=100ms).");
    Serial.printf("[LIGHT] Initial phase: %s GREEN\n", LANE_NAMES[(int)currentLane]);

    bool wasEmergency = false; /* Flag untuk mendeteksi pemulihan dari emergency */

    for (;;) {
        {
#if defined(MABUTRACE_ENABLED)
            TRACE_SCOPE("TaskTrafficLight");
#endif
            TRACE_LIGHT("LIGHT_START");

        /* ─────────────────────────────────────────────────────
         * CEK EMERGENCY — baca dari TrafficState (timeout singkat)
         * ──────────────────────────────────────────────────── */
        bool emergencyActive = false;
        Lane emergencyLane   = LANE_NORTH;

        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            emergencyActive = gTrafficState.emergencyActive;
            emergencyLane   = gTrafficState.emergencyLane;
            xSemaphoreGive(stateMutex);
        }

        if (emergencyActive) {
            /* ──────────────────────────────────────────────────
             * MODE DARURAT — LED sudah di-set oleh TaskEmergencyHandler.
             * Task ini cukup mirror status ke MonitoringData.
             * Jangan override LED! (Emergency Handler sudah set dengan benar)
             * ─────────────────────────────────────────────────*/
            TRACE_LIGHT("LIGHT_EMERGENCY_MODE");

            LightColor emergColors[LANE_COUNT];
            for (int i = 0; i < LANE_COUNT; i++) {
                emergColors[i] = (i == (int)emergencyLane) ? LIGHT_GREEN : LIGHT_RED;
            }
            updateMonitorLightStatus(emergColors);

            /* Reset timer saat emergency agar setelah restore, fase dimulai fresh */
            timeInPhase_ms = 0;
            wasEmergency = true;

        } else {
            /* ──────────────────────────────────────────────────
             * MODE NORMAL — State Machine
             * ─────────────────────────────────────────────────*/

            /* Jika baru pulih dari emergency, pastikan semua lampu kembali merah dulu */
            if (wasEmergency) {
                setAllRed();
                wasEmergency = false;
            }

            /* Ambil greenDuration terbaru dari Controller (timeout pendek) */
            if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                greenDuration_ms = gTrafficState.greenDuration_ms;
                /* Update sisa waktu hijau ke TrafficState */
                uint32_t remaining = (timeInPhase_ms < greenDuration_ms)
                                     ? (greenDuration_ms - timeInPhase_ms) : 0;
                gTrafficState.greenRemaining_ms = remaining;
                gTrafficState.activeLane        = currentLane;
                gTrafficState.lanePhase         = currentPhase;
                xSemaphoreGive(stateMutex);
            }

            /* ── Lanjutkan state machine berdasarkan fase saat ini ── */
            LightColor currentColors[LANE_COUNT];
            for (int i = 0; i < LANE_COUNT; i++) {
                currentColors[i] = LIGHT_RED; /* Default: semua merah */
            }

            switch (currentPhase) {

                /* ── FASE HIJAU ─────────────────────────────── */
                case LIGHT_GREEN:
                    timeInPhase_ms += PERIOD_LIGHT_MS;

                    if (timeInPhase_ms >= greenDuration_ms) {
                        /* Transisi ke YELLOW */
                        TRACE_LIGHT("LIGHT_TRANS_GREEN_TO_YELLOW");
                        currentPhase   = LIGHT_YELLOW;
                        timeInPhase_ms = 0;
                        setLaneLED((int)currentLane, LIGHT_YELLOW);
                        Serial.printf("[LIGHT] %s: GREEN → YELLOW\n", LANE_NAMES[(int)currentLane]);
                    } else {
                        /* Tetap di GREEN */
                        setLaneLED((int)currentLane, LIGHT_GREEN);
                    }

                    currentColors[(int)currentLane] = currentPhase;
                    break;

                /* ── FASE KUNING ─────────────────────────────── */
                case LIGHT_YELLOW:
                    timeInPhase_ms += PERIOD_LIGHT_MS;

                    if (timeInPhase_ms >= YELLOW_MS) {
                        /* Transisi ke RED untuk jalur ini, advance ke jalur berikutnya */
                        TRACE_LIGHT("LIGHT_TRANS_YELLOW_TO_RED");

                        /* FR-06: Semua jalur RED sebelum jalur berikutnya GREEN */
                        setAllRed();

                        /* Advance ke jalur berikutnya (rotasi modular) */
                        currentLane    = (Lane)(((int)currentLane + 1) % LANE_COUNT);
                        currentPhase   = LIGHT_YELLOW_TO_GREEN;
                        timeInPhase_ms = 0;

                        /* Update state (masih persiapan, belum dapat durasi hijau dari kalkulasi) */
                        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                            gTrafficState.activeLane = currentLane;
                            gTrafficState.lanePhase  = LIGHT_YELLOW_TO_GREEN;
                            xSemaphoreGive(stateMutex);
                        }

                        /* Set jalur baru ke YELLOW_TO_GREEN */
                        setLaneLED((int)currentLane, LIGHT_YELLOW_TO_GREEN);
                        Serial.printf("[LIGHT] %s: YELLOW → %s: YEL->G (prep %ums)\n",
                                      LANE_NAMES[((int)currentLane - 1 + LANE_COUNT) % LANE_COUNT],
                                      LANE_NAMES[(int)currentLane], YELLOW_TO_GREEN_MS);
                    } else {
                        /* Tetap di YELLOW */
                        setLaneLED((int)currentLane, LIGHT_YELLOW);
                    }

                    currentColors[(int)currentLane] = currentPhase;
                    break;

                /* ── FASE PERSIAPAN (KUNING MENUJU HIJAU) ────── */
                case LIGHT_YELLOW_TO_GREEN:
                    timeInPhase_ms += PERIOD_LIGHT_MS;

                    if (timeInPhase_ms >= YELLOW_TO_GREEN_MS) {
                        /* Transisi ke GREEN */
                        TRACE_LIGHT("LIGHT_TRANS_YELLOW_TO_GREEN");
                        currentPhase   = LIGHT_GREEN;
                        timeInPhase_ms = 0;

                        /* Baca greenDuration untuk jalur baru */
                        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                            greenDuration_ms = gTrafficState.greenDuration_ms;
                            gTrafficState.activeLane = currentLane;
                            gTrafficState.lanePhase  = LIGHT_GREEN;
                            xSemaphoreGive(stateMutex);
                        }

                        /* Set jalur baru ke GREEN */
                        setLaneLED((int)currentLane, LIGHT_GREEN);
                        Serial.printf("[LIGHT] %s: YEL->G → GREEN (dur=%ums)\n",
                                      LANE_NAMES[(int)currentLane], greenDuration_ms);
                    } else {
                        /* Tetap di YELLOW_TO_GREEN */
                        setLaneLED((int)currentLane, LIGHT_YELLOW_TO_GREEN);
                    }

                    currentColors[(int)currentLane] = currentPhase;
                    break;

                default:
                    /* Fase tidak valid — reset ke GREEN jalur pertama */
                    currentPhase   = LIGHT_GREEN;
                    currentLane    = LANE_NORTH;
                    timeInPhase_ms = 0;
                    setAllRed();
                    setLaneLED(LANE_NORTH, LIGHT_GREEN);
                    break;
            }

            updateMonitorLightStatus(currentColors);
        }

        TRACE_LIGHT("LIGHT_END");

        }

        /* ── Tunda hingga periode berikutnya ── */
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }

    vTaskDelete(NULL);
}