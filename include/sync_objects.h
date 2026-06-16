/**
 * @file sync_objects.h
 * @brief Deklarasi extern objek sinkronisasi FreeRTOS & struktur shared state.
 *
 * ============================================================
 * REFERENSI LAPORAN:
 *   • Tabel 4.4  — Inter-Task Communication & Synchronization Objects
 *   • Section 4.3 — Queue & Semaphore design rationale
 *   • Section 4.4.3 — Priority Inheritance (mutex vs binary semaphore)
 * ============================================================
 *
 * LIFECYCLE OBJEK:
 *   Semua objek di-CREATE di setup() (main.cpp) melalui initSyncObjects().
 *   FreeRTOS mengalokasikan dari heap dinamis (heap_4 — best-fit + coalescence).
 *   Tidak ada objek yang di-create di dalam task (menghindari heap fragmentation).
 *
 * CATATAN MUTEX vs BINARY SEMAPHORE:
 *   stateMutex & monitorMutex menggunakan xSemaphoreCreateMutex()
 *   (BUKAN xSemaphoreCreateBinary()) karena:
 *     1. Mutex mendukung Priority Inheritance (PI) — mencegah Priority Inversion
 *        antara TaskTrafficLight (P=3) dan TaskEmergencyHandler (P=5).
 *     2. Sesuai Section 4.4.3 laporan: PI wajib untuk proteksi TrafficState.
 *   emergencySem menggunakan Binary Semaphore karena:
 *     - Ini mekanisme SIGNALING (ISR → Task), bukan proteksi resource.
 *     - Binary semaphore TIDAK memiliki konsep "owner", cocok untuk give-dari-ISR.
 * ============================================================
 */

#ifndef SYNC_OBJECTS_H
#define SYNC_OBJECTS_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "config.h"

/* ─────────────────────────────────────────────────────────────
 * 1. QUEUE
 * ──────────────────────────────────────────────────────────── */

/**
 * @brief Queue kendaraan normal — ISR → TaskTrafficController
 *
 * Tabel 4.4:
 *   Producer : isrVehicle_N/S/E/W (ISR, via xQueueSendFromISR)
 *   Consumer : TaskTrafficController (drain tiap 20 ms, non-blocking)
 *
 * Kapasitas : 10 item (mencegah overflow burst kendaraan)
 * Item size : sizeof(Lane) = 4 byte (uint32_t / enum 32-bit)
 *
 * Dibuat di: initSyncObjects() → xQueueCreate(10, sizeof(Lane))
 */
extern QueueHandle_t vehicleQueue;

/* ─────────────────────────────────────────────────────────────
 * 2. SEMAPHORE
 * ──────────────────────────────────────────────────────────── */

/**
 * @brief Binary semaphore darurat — ISR_Emergency → TaskEmergencyHandler
 *
 * Tabel 4.4:
 *   Producer : isrEmergency_N/S/E/W (ISR, via xSemaphoreGiveFromISR)
 *   Consumer : TaskEmergencyHandler (blocking xSemaphoreTake, portMAX_DELAY)
 *
 * Binary semaphore (BUKAN mutex) karena digunakan sebagai sinyal event,
 * bukan sebagai proteksi resource. ISR tidak punya "owner".
 *
 * Dibuat di: initSyncObjects() → xSemaphoreCreateBinary()
 */
extern SemaphoreHandle_t emergencySem;

/* ─────────────────────────────────────────────────────────────
 * 3. MUTEX
 * ──────────────────────────────────────────────────────────── */

/**
 * @brief Mutex proteksi TrafficState (shared state fase lampu & durasi hijau)
 *
 * Tabel 4.4:
 *   Writer : TaskEmergencyHandler (set emergency state, restore state)
 *            TaskTrafficController (update greenDuration)
 *   Reader : TaskTrafficLight (baca greenDuration & emergencyActive)
 *
 * Dibuat di: initSyncObjects() → xSemaphoreCreateMutex()
 * Priority Inheritance: AKTIF (mencegah priority inversion P=3 vs P=5)
 */
extern SemaphoreHandle_t stateMutex;

/**
 * @brief Mutex proteksi MonitoringData (buffer statistik kumulatif)
 *
 * Tabel 4.4:
 *   Writer : TaskTrafficController (update vehicleCount)
 *            TaskTrafficLight (update lightStatus per jalur)
 *            TaskEmergencyHandler (set emergencyActive flag)
 *   Reader : TaskMonitoring (baca semua field untuk display)
 *
 * Dibuat di: initSyncObjects() → xSemaphoreCreateMutex()
 */
extern SemaphoreHandle_t monitorMutex;

/* ─────────────────────────────────────────────────────────────
 * 4. STRUCT SHARED STATE
 * ──────────────────────────────────────────────────────────── */

/**
 * @struct TrafficState
 * @brief Shared state fase lampu lalu lintas.
 *
 * Diproteksi oleh stateMutex.
 * Dibaca/tulis oleh: TaskEmergencyHandler, TaskTrafficController, TaskTrafficLight.
 *
 * FR-07: savedLane & savedGreenRemaining_ms memungkinkan restore
 *        state setelah preemption darurat selesai.
 */
typedef struct {
    Lane         activeLane;              ///< Jalur yang sedang mendapat fase hijau (atau kuning)
    LightColor   lanePhase;               ///< Fase saat ini dari activeLane (GREEN/YELLOW/RED)
    uint32_t     greenDuration_ms;        ///< Durasi hijau hasil kalkulasi proporsional (ms)
    uint32_t     greenRemaining_ms;       ///< Sisa waktu hijau untuk activeLane (ms)
    bool         emergencyActive;         ///< true jika sedang dalam mode darurat
    Lane         emergencyLane;           ///< Jalur yang mendapat preemption darurat

    /* ── Saved state untuk FR-07 (restore setelah emergency) ── */
    Lane         savedLane;              ///< Fase aktif sebelum emergency di-preempt
    LightColor   savedPhase;            ///< Fase warna sebelum emergency
    uint32_t     savedGreenRemaining_ms; ///< Sisa waktu hijau yang tersimpan
} TrafficState;

extern volatile TrafficState gTrafficState; ///< Instance global, diproteksi stateMutex

/**
 * @struct MonitoringData
 * @brief Buffer statistik kumulatif untuk TaskMonitoring.
 *
 * Diproteksi oleh monitorMutex.
 * Dibaca oleh: TaskMonitoring (Serial + LCD output).
 *
 * FR-05: Laporan jumlah kendaraan per jalur.
 * FR-08: Status lampu real-time untuk display.
 */
typedef struct {
    uint32_t   vehicleCount[LANE_COUNT]; ///< Jumlah kendaraan kumulatif [N, E, S, W]
    LightColor lightStatus[LANE_COUNT];  ///< Status lampu saat ini per jalur
    bool       emergencyActive;          ///< Mirror dari TrafficState.emergencyActive
    Lane       emergencyLane;            ///< Mirror dari TrafficState.emergencyLane
    uint32_t   updateCount;              ///< Jumlah update monitoring (untuk stack check interval)
} MonitoringData;

extern volatile MonitoringData gMonitoringData; ///< Instance global, diproteksi monitorMutex

/* ─────────────────────────────────────────────────────────────
 * 5. VARIABEL GLOBAL EMERGENCY (tulis dari ISR)
 *    Diproteksi portENTER_CRITICAL_ISR / portEXIT_CRITICAL_ISR
 *    karena ESP32 adalah SMP dual-core (volatile tidak cukup).
 * ──────────────────────────────────────────────────────────── */

/**
 * @brief Jalur yang memicu emergency terakhir, ditulis oleh ISR.
 *
 * Dibaca oleh TaskEmergencyHandler setelah mengambil emergencySem.
 * Akses dari ISR dilindungi portENTER_CRITICAL_ISR() di isr_handlers.cpp.
 */
extern volatile Lane gEmergencyLane;

/**
 * @brief Spinlock untuk proteksi gEmergencyLane di SMP.
 *        Diinisialisasi di initSyncObjects() dengan portMUX_INITIALIZER_UNLOCKED.
 */
extern portMUX_TYPE gEmergencyMux;

/* ─────────────────────────────────────────────────────────────
 * 6. TASK HANDLES (untuk vTaskSuspend/vTaskResume)
 * ──────────────────────────────────────────────────────────── */

/**
 * @brief Handle TaskTrafficLight — digunakan oleh TaskEmergencyHandler
 *        untuk men-suspend task ini selama transisi LED emergency,
 *        mencegah race condition penulisan GPIO.
 */
extern TaskHandle_t hTaskTrafficLight;

#endif /* SYNC_OBJECTS_H */