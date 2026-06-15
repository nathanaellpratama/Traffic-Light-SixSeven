/**
 * @file task_traffic_controller.cpp
 * @brief Implementasi TaskTrafficController — Kalkulasi durasi fase adaptif.
 *
 * ============================================================
 * REFERENSI LAPORAN:
 *   • Tabel 4.1  — Task: TrafficController, P=4, T=20ms, C=2ms
 *   • Tabel 4.2  — Stack = 4096 byte = 1024 word (ESP32 32-bit)
 *   • Section 4.2.3 — Justifikasi T=20 ms:
 *       T_controller = T_light / 5 = 100/5 = 20 ms
 *       Dengan jitter toleransi ≤ 5% dari T_light = 5 ms,
 *       controller dapat menyelesaikan kalkulasi (C=2ms) dengan margin
 *       baik sebelum TaskTrafficLight membaca greenDuration.
 *   • FR-03 — Durasi hijau adaptif proporsional terhadap kepadatan kendaraan.
 *   • Section 4.4.3 — stateMutex + monitorMutex dengan priority inheritance.
 * ============================================================
 *
 * ALGORITMA PROPORSIONAL (FR-03):
 *   Setiap siklus 20 ms, drain vehicleQueue dan update counter per jalur.
 *   Untuk jalur yang sedang aktif (activeLane di TrafficState):
 *
 *     ratio  = vehicleCount[activeLane] / totalVehicles
 *     range  = MAX_GREEN_MS - MIN_GREEN_MS
 *     green  = BASE_GREEN_MS + (ratio × range)
 *     green  = clamp(green, MIN_GREEN_MS, MAX_GREEN_MS)
 *
 *   Jika totalVehicles == 0, gunakan BASE_GREEN_MS (default).
 *   Hasilnya ditulis ke gTrafficState.greenDuration_ms via stateMutex.
 * ============================================================
 */

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
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
  extern traceString trcControllerChannel;
  #define TRACE_CTRL(msg)  vTracePrint(trcControllerChannel, (msg))
#else
  #define TRACE_CTRL(msg)  /* no-op */
#endif

/* ─────────────────────────────────────────────────────────────
 * VARIABEL LOKAL — counter kendaraan "working copy" per siklus
 * (tidak perlu extern; TaskMonitoring baca dari gMonitoringData)
 * ──────────────────────────────────────────────────────────── */
static uint32_t vehicleCount[LANE_COUNT] = {0, 0, 0, 0};

/**
 * @brief Hitung durasi hijau proporsional untuk jalur tertentu.
 *
 * @param targetLane  Jalur yang sedang menunggu fase hijau.
 * @return uint32_t   Durasi hijau dalam milidetik, di-clamp ke [MIN, MAX].
 */
static uint32_t calculateGreenDuration(Lane targetLane) {
    uint32_t total = 0;
    for (int i = 0; i < LANE_COUNT; i++) {
        total += vehicleCount[i];
    }

    if (total == 0) {
        return BASE_GREEN_MS; /* Tidak ada data kendaraan → durasi default */
    }

    float ratio   = (float)vehicleCount[targetLane] / (float)total;
    float range   = (float)(MAX_GREEN_MS - MIN_GREEN_MS);
    uint32_t dur  = (uint32_t)(BASE_GREEN_MS + ratio * range);

    /* Clamp ke [MIN_GREEN_MS, MAX_GREEN_MS] */
    if (dur < MIN_GREEN_MS) dur = MIN_GREEN_MS;
    if (dur > MAX_GREEN_MS) dur = MAX_GREEN_MS;

    return dur;
}

/* ─────────────────────────────────────────────────────────────
 * TASK IMPLEMENTATION
 * ──────────────────────────────────────────────────────────── */

void TaskTrafficController(void *pvParameters) {
    (void)pvParameters; /* Tidak digunakan */

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(PERIOD_CONTROLLER_MS); /* 20 ms */

    Serial.println("[CTRL] TaskTrafficController started (Core 1, P=4, T=20ms).");

    for (;;) {
        {
#if defined(MABUTRACE_ENABLED)
            TRACE_SCOPE("TaskTrafficController");
#endif
            /* ── [Tracealyzer] Tandai awal eksekusi task (untuk WCET Tabel 6.2) ── */
        TRACE_CTRL("CTRL_START");

        /* ─────────────────────────────────────────────────────
         * CEK EMERGENCY: Jika emergency aktif, skip kalkulasi normal.
         * TaskEmergencyHandler (P=5) akan menangani state; Controller
         * tetap jalan untuk drain queue agar tidak overflow.
         * ──────────────────────────────────────────────────── */
        bool emergencyActive = false;
        {
            /* Baca flag emergency dengan timeout singkat */
            if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                emergencyActive = gTrafficState.emergencyActive;
                xSemaphoreGive(stateMutex);
            }
        }

        /* ─────────────────────────────────────────────────────
         * STEP 1: Drain vehicleQueue — ambil SEMUA item yang tersedia
         *         dalam satu siklus (maksimal 10, sesuai kapasitas queue).
         *         Timeout = 0 (non-blocking: jika queue kosong, lanjut).
         * ──────────────────────────────────────────────────── */
        Lane receivedLane;
        int itemsDrained = 0;

        while (xQueueReceive(vehicleQueue, &receivedLane, 0) == pdTRUE) {
            if (receivedLane < LANE_COUNT) {
                vehicleCount[receivedLane]++;
                itemsDrained++;
            }
            if (itemsDrained >= 10) break; /* Batas satu siklus = kapasitas queue */
        }

        /* ─────────────────────────────────────────────────────
         * STEP 2: Update monitorMutex — vehicleCount kumulatif
         *         hanya jika ada kendaraan baru dalam siklus ini.
         * ──────────────────────────────────────────────────── */
        if (itemsDrained > 0) {
            if (xSemaphoreTake(monitorMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                for (int i = 0; i < LANE_COUNT; i++) {
                    gMonitoringData.vehicleCount[i] = vehicleCount[i];
                }
                xSemaphoreGive(monitorMutex);
            }
            /* Jika timeout mutex (>5ms): data monitoring mungkin stale 1 siklus, */
            /* tapi tidak kritis — monitoring bukan safety-critical path.          */
        }

        /* ─────────────────────────────────────────────────────
         * STEP 3: Hitung & tulis greenDuration (skip jika emergency)
         * ──────────────────────────────────────────────────── */
        if (!emergencyActive) {
            /* Baca jalur aktif saat ini untuk kalkulasi proporsional */
            Lane activeLane = LANE_NORTH;
            if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                activeLane = gTrafficState.activeLane;
                xSemaphoreGive(stateMutex);
            }

            /* Hitung durasi hijau proporsional untuk jalur berikutnya */
            uint32_t newGreenDuration = calculateGreenDuration(activeLane);

            /* ── Tulis hasil ke TrafficState (dilindungi stateMutex) ── */
            if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                gTrafficState.greenDuration_ms = newGreenDuration;
                xSemaphoreGive(stateMutex);
            } else {
                /* Timeout mutex — log warning, lanjutkan (nilai lama tetap dipakai) */
                Serial.println("[CTRL] WARNING: stateMutex timeout! Using previous greenDuration.");
            }
        }

        /* ── [Tracealyzer] Tandai akhir eksekusi (ukur WCET) ── */
        TRACE_CTRL("CTRL_END");

        }

        /* ── Tunda hingga periode berikutnya (vTaskDelayUntil = presisi) ── */
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
    /* Task tidak seharusnya keluar — delete diri sendiri jika terjadi */
    vTaskDelete(NULL);
}