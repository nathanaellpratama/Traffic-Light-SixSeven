/**
 * @file task_monitoring.cpp
 * @brief Implementasi TaskMonitoring — Laporan statistik via Serial & LCD.
 *
 * ============================================================
 * REFERENSI LAPORAN:
 *   • Tabel 4.1  — Task: Monitoring, P=2 (terendah), T=500ms
 *   • Tabel 4.2  — Stack = 6144 byte = 1536 word
 *   • FR-05 — Laporan jumlah kendaraan kumulatif per jalur.
 *   • FR-08 — Display status lampu lalu lintas secara real-time.
 *   • Section 4.6.2 — Stack monitoring via uxTaskGetStackHighWaterMark():
 *       Panggil setiap 10 siklus (= 5 detik).
 *       Log warning ke Serial jika sisa stack < 20% dari ukuran awal.
 *
 * FORMAT OUTPUT SERIAL (tabel sederhana, mudah di-parse):
 * ┌─────────────────────────────────────────────┐
 * │ [MON] ===== Traffic Status @ 5000ms =====   │
 * │ [MON] Lane  | Vehicles | Light  | Emergency │
 * │ [MON] NORTH |    12    | GREEN  |           │
 * │ [MON] EAST  |     7    | RED    |           │
 * │ [MON] SOUTH |    19    | RED    |           │
 * │ [MON] WEST  |     4    | RED    |           │
 * │ [MON] Emergency: INACTIVE                   │
 * └─────────────────────────────────────────────┘
 *
 * FORMAT OUTPUT LCD 16x2 (bergantian tiap update 500ms):
 *   Update ganjil (A):  Line1: "N:12 E:7 S:19 W:4"   (kendaraan)
 *                       Line2: "N:GRN E:RED S:RED W:R"(status)
 *   Update genap  (B):  Line1: "Emergency: INACTIVE"
 *                       Line2: "Active: NORTH      "
 * ============================================================
 */

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
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
  extern traceString trcMonitoringChannel;
  #define TRACE_MON(msg)  vTracePrint(trcMonitoringChannel, (msg))
#else
  #define TRACE_MON(msg)  /* no-op */
#endif

/* ── LCD instance (extern dideklarasikan di main.cpp) ── */
extern LiquidCrystal_I2C lcd;

/* ── Nama jalur dan warna untuk display ── */
static const char* LANE_NAMES[LANE_COUNT]  = { "NORTH", "EAST ", "SOUTH", "WEST " };
static const char* LIGHT_NAMES[4]          = { "RED   ", "YELLOW", "GREEN ", "YEL->G" };
static const char* LANE_SHORT[LANE_COUNT]  = { "N", "E", "S", "W" };
static const char* LIGHT_SHORT[4]          = { "RED", "YEL", "GRN", "Y2G" };

/* ─────────────────────────────────────────────────────────────
 * HELPER — Tampilkan tabel ke Serial Monitor
 * ──────────────────────────────────────────────────────────── */
static void printSerialTable(const MonitoringData *data, uint32_t uptimeMs) {
    Serial.printf("\n[MON] ===== Traffic Status @ %ums =====\n", uptimeMs);
    Serial.println("[MON] Lane  | Vehicles | Light  | Emg");
    Serial.println("[MON] ------+----------+--------+----");
    for (int i = 0; i < LANE_COUNT; i++) {
        Serial.printf("[MON] %s |   %4u   | %s |%s\n",
                      LANE_NAMES[i],
                      data->vehicleCount[i],
                      LIGHT_NAMES[data->lightStatus[i]],
                      (data->emergencyActive && data->emergencyLane == (Lane)i) ? " !" : "  ");
    }
    Serial.printf("[MON] Emergency: %s",
                  data->emergencyActive ? "ACTIVE → " : "INACTIVE\n");
    if (data->emergencyActive) {
        Serial.printf("Lane %s\n", LANE_NAMES[(int)data->emergencyLane]);
    }
}

/* ─────────────────────────────────────────────────────────────
 * HELPER — Tampilkan ke LCD 16x2 (bergantian A/B)
 * ──────────────────────────────────────────────────────────── */
static void updateLCD(const MonitoringData *data) {
    /* CATATAN HARDWARE: lcd.clear() dipanggil setiap 500ms menyebabkan
     * flicker yang terlihat pada LCD fisik. Gunakan setCursor + overwrite
     * dengan padding spasi agar karakter lama tertimpa tanpa clear. */

    /* Line 0: "N:12    E:7" */
    char line0[17];
    snprintf(line0, sizeof(line0), "N:%-5u E:%-5u",
             data->vehicleCount[LANE_NORTH],
             data->vehicleCount[LANE_EAST]);
    lcd.setCursor(0, 0);
    lcd.print(line0);

    /* Line 1: "S:19    W:4" */
    char line1[17];
    snprintf(line1, sizeof(line1), "S:%-5u W:%-5u",
             data->vehicleCount[LANE_SOUTH],
             data->vehicleCount[LANE_WEST]);
    lcd.setCursor(0, 1);
    lcd.print(line1);
}


/* ─────────────────────────────────────────────────────────────
 * TASK IMPLEMENTATION
 * ──────────────────────────────────────────────────────────── */

void TaskMonitoring(void *pvParameters) {
    (void)pvParameters;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(PERIOD_MONITORING_MS); /* 500 ms */

    /* ── Stack monitoring ── */
    const UBaseType_t stackSizeWords = STACK_MONITORING;
    const UBaseType_t warnThreshold  = stackSizeWords * STACK_WARNING_THRESHOLD_PCT / 100;
    uint32_t cycleCount  = 0;

    Serial.println("[MON] TaskMonitoring started (Core 0, P=2, T=500ms). Press 't' or 'T' to dump MabuTrace JSON.");

    for (;;) {
        /* Cek input serial untuk trigger dump trace MabuTrace */
#if defined(MABUTRACE_ENABLED)
        bool shouldDump = false;
        if (Serial.available() > 0) {
            char c = Serial.read();
            if (c == 't' || c == 'T') {
                shouldDump = true;
            }
            while (Serial.available() > 0) { Serial.read(); } // Clear buffer
        }

        if (shouldDump) {

            // Suspend tasks agar log normal tidak bertabrakan dan merusak format JSON dump
            if (hTaskTrafficLight != NULL) { vTaskSuspend(hTaskTrafficLight); }
            if (hTaskTrafficController != NULL) { vTaskSuspend(hTaskTrafficController); }
            if (hTaskEmergencyHandler != NULL) { vTaskSuspend(hTaskEmergencyHandler); }

            Serial.println(F("\n--- MABUTRACE JSON DUMP START ---"));
            get_json_trace_chunked(NULL, [](void* ctx, const char* chunk, size_t size) {
                Serial.print(chunk);
            });
            Serial.println(F("\n--- MABUTRACE JSON DUMP END ---"));

            // Resume tasks kembali setelah dump selesai
            if (hTaskEmergencyHandler != NULL) { vTaskResume(hTaskEmergencyHandler); }
            if (hTaskTrafficController != NULL) { vTaskResume(hTaskTrafficController); }
            if (hTaskTrafficLight != NULL) { vTaskResume(hTaskTrafficLight); }
        }
#endif

        {
#if defined(MABUTRACE_ENABLED)
            TRACE_SCOPE("TaskMonitoring");
#endif
            TRACE_MON("MON_START");
        cycleCount++;

        /* ─────────────────────────────────────────────────────
         * STEP 1: Baca MonitoringData (dilindungi monitorMutex)
         *         Timeout 20 ms: jika tidak bisa, gunakan data stale.
         * ──────────────────────────────────────────────────── */
        MonitoringData snapshot;
        bool gotData = false;

        if (xSemaphoreTake(monitorMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            /* Copy atomik (di bawah mutex) — gunakan memcpy karena gMonitoringData volatile */
            memcpy(&snapshot, (const void*)&gMonitoringData, sizeof(MonitoringData));
            snapshot.updateCount = cycleCount;
            xSemaphoreGive(monitorMutex);
            gotData = true;
        } else {
            Serial.println("[MON] WARNING: monitorMutex timeout! Displaying stale data.");
        }

        /* ─────────────────────────────────────────────────────
         * STEP 2: Tampilkan ke Serial Monitor
         * ──────────────────────────────────────────────────── */
        if (gotData) {
            uint32_t uptimeMs = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
            printSerialTable(&snapshot, uptimeMs);
        }

        /* ─────────────────────────────────────────────────────
         * STEP 3: Tampilkan ke LCD I2C
         * ──────────────────────────────────────────────────── */
        if (gotData) {
            updateLCD(&snapshot);
        }

        /* ─────────────────────────────────────────────────────
         * STEP 4: Stack monitoring (setiap 10 siklus = 5 detik)
         *         Ref: Section 4.6.2
         * ──────────────────────────────────────────────────── */
        if (cycleCount % 10 == 0) {
            UBaseType_t hwm = uxTaskGetStackHighWaterMark(NULL);

            Serial.printf("[MON] Stack HWM: %u words remaining (min safe: %u words)\n",
                          hwm, warnThreshold);

            if (hwm < warnThreshold) {
                Serial.printf("[MON] *** WARNING: Stack < %u%% remaining! hwm=%u words ***\n",
                              STACK_WARNING_THRESHOLD_PCT, hwm);
                /* Pertimbangkan tambah STACK_MONITORING di config.h */
            }

            /* ── Opsional: log stack semua task ── */
            /* TaskStatus_t taskList[10];
               UBaseType_t count = uxTaskGetSystemState(taskList, 10, NULL);
               for (int i = 0; i < count; i++) {
                   Serial.printf("[MON] Task '%s' HWM: %u\n",
                                 taskList[i].pcTaskName, taskList[i].usStackHighWaterMark);
               } */
        }

            TRACE_MON("MON_END");
        }

        /* ── Tunda hingga periode berikutnya ── */
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }

    vTaskDelete(NULL);
}