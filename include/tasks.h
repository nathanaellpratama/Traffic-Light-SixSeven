/**
 * @file tasks.h
 * @brief Deklarasi prototype untuk 4 task FreeRTOS, fungsi setup, dan ISR handler.
 *
 * ============================================================
 * REFERENSI LAPORAN:
 *   • Tabel 4.1  — Task Description Summary
 *   • Section 4.5.2 — Pola Deferred Interrupt Processing
 *   • Section 4.7  — Integrasi Multicore (core affinity)
 * ============================================================
 *
 * PENDEKATAN ISR:
 *   Dipilih: 8 ISR TERPISAH (satu per tombol), bukan parametrized ISR.
 *
 *   Alasan:
 *     PROS 8 ISR terpisah:
 *       + Tidak ada overhead lookup "pin mana yang triggered" di ISR context
 *       + Setiap ISR langsung push Lane enum yang sudah diketahui
 *       + Stack lebih kecil per ISR (tidak perlu parameter struct)
 *       + Lebih mudah di-attach: attachInterrupt(pin, isr, FALLING)
 *       + Setiap ISR punya state debounce sendiri (array volatile static)
 *
 *     CONS 8 ISR terpisah:
 *       - Kode repetitif (tapi dapat dikurangi dengan macro helper)
 *
 *     PROS Parametrized ISR (satu ISR + digitalRead mana yang LOW):
 *       + Kode lebih ringkas
 *
 *     CONS Parametrized ISR:
 *       - attachInterrupt() ESP32 TIDAK mendukung passing parameter ke ISR
 *         (callback signature harus void isr(void), tanpa argumen)
 *       - Workaround: gunakan lambda/closure atau global flag — menambah
 *         kompleksitas dan potensi race condition di SMP
 *       → Dengan demikian, 8 ISR terpisah adalah PENDEKATAN VALID dan DIREKOMENDASIKAN
 *         untuk ESP32 Arduino framework.
 * ============================================================
 */

#ifndef TASKS_H
#define TASKS_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "config.h"
#include "sync_objects.h"

/* ─────────────────────────────────────────────────────────────
 * 1. TASK FUNCTIONS
 * ──────────────────────────────────────────────────────────── */

/**
 * @brief Task Emergency Handler — prioritas tertinggi (P=5)
 *
 * Tabel 4.1:
 *   Tipe      : Sporadic (event-driven via semaphore)
 *   Periode   : T = 100 ms (batas atas sporadic period, WCET C = 2 ms)
 *   Core      : CORE_APP (Core 1)
 *   Fungsi    : Deteksi & respons kendaraan darurat, preempt fase normal,
 *               set jalur darurat ke hijau, restore state setelah selesai.
 *   FR-02     : Kendaraan darurat mendapat prioritas lampu hijau segera.
 *   FR-07     : Restore ke state sebelum emergency setelah kendaraan lewat.
 *   NFR-02    : Latensi preemption ≤ 100 ms dari tekan tombol ke LED hijau.
 */
void TaskEmergencyHandler(void *pvParameters);

/**
 * @brief Task Traffic Controller — kalkulasi durasi fase proporsional (P=4)
 *
 * Tabel 4.1:
 *   Tipe      : Periodic (T = 20 ms)
 *   Core      : CORE_APP (Core 1)
 *   Fungsi    : Drain vehicleQueue, update counter kendaraan, hitung
 *               greenDuration proporsional (FR-03), tulis ke TrafficState.
 *   FR-03     : Durasi hijau adaptif berdasarkan rasio kendaraan per jalur.
 *   Ref       : Section 4.2.3 (justifikasi T=20ms karena jitter <5% dari T_light)
 */
void TaskTrafficController(void *pvParameters);

/**
 * @brief Task Traffic Light — aktuasi LED state machine (P=3)
 *
 * Tabel 4.1:
 *   Tipe      : Periodic (T = 100 ms)
 *   Core      : CORE_APP (Core 1)
 *   Fungsi    : State machine GREEN→YELLOW→RED per jalur, rotasi N→E→S→W.
 *               Saat emergency: bypass state machine, set jalur darurat hijau.
 *   FR-04     : Siklus lampu otomatis dengan urutan yang benar.
 *   FR-06     : Mutual exclusion — hanya SATU jalur hijau pada satu waktu.
 */
void TaskTrafficLight(void *pvParameters);

/**
 * @brief Task Monitoring — laporan Serial & LCD (P=2, prioritas terendah)
 *
 * Tabel 4.1:
 *   Tipe      : Periodic (T = 500 ms)
 *   Core      : CORE_PRO (Core 0)
 *   Fungsi    : Baca MonitoringData, tampilkan ke Serial & LCD I2C.
 *               Monitor stack watermark tiap 10 siklus (5 detik).
 *   FR-05     : Laporan jumlah kendaraan kumulatif per jalur.
 *   FR-08     : Display status lampu real-time.
 *   Ref       : Section 4.6.2 (stack overflow prevention).
 */
void TaskMonitoring(void *pvParameters);

/* ─────────────────────────────────────────────────────────────
 * 2. SETUP FUNCTIONS
 * ──────────────────────────────────────────────────────────── */

/**
 * @brief Buat semua objek sinkronisasi FreeRTOS.
 *        Dipanggil di setup() SEBELUM xTaskCreatePinnedToCore().
 *        Alokasi dari heap_4 (FreeRTOS default ESP32-IDF).
 */
void initSyncObjects(void);

/**
 * @brief Setup semua pin GPIO (LED sebagai OUTPUT, Buttons sebagai INPUT/PULLUP).
 *        Dipanggil di setup() sebelum attachInterrupts().
 */
void initGPIO(void);

/**
 * @brief Attach ke-8 ISR ke GPIO masing-masing, mode FALLING.
 *        Dipanggil setelah initGPIO() dan initSyncObjects().
 *        ISR dijalankan di Core yang sama dengan loop() (Core 1 pada ESP32-Arduino).
 */
void attachInterrupts(void);

/* ─────────────────────────────────────────────────────────────
 * 3. ISR — KENDARAAN NORMAL
 *    Mode     : FALLING (tombol tekan → pin LOW)
 *    Debounce : 50 ms per-pin (array static volatile)
 *    Action   : xQueueSendFromISR(vehicleQueue, &lane, &woken)
 *    Ref      : Section 4.5.2 Deferred Interrupt Processing
 *               NFR-01: Latency ≤ 5 ms (ISR minimal, defer ke task)
 * ──────────────────────────────────────────────────────────── */
void IRAM_ATTR isrVehicle_N(void); ///< ISR tombol kendaraan normal — Jalur Utara (GPIO 32)
void IRAM_ATTR isrVehicle_S(void); ///< ISR tombol kendaraan normal — Jalur Selatan (GPIO 33)
void IRAM_ATTR isrVehicle_E(void); ///< ISR tombol kendaraan normal — Jalur Timur (GPIO 2)
void IRAM_ATTR isrVehicle_W(void); ///< ISR tombol kendaraan normal — Jalur Barat (GPIO 0)

/* ─────────────────────────────────────────────────────────────
 * 4. ISR — KENDARAAN DARURAT
  *    Mode     : FALLING (tombol tekan → pin LOW via ext/int pull-up)
 *    Debounce : 50 ms per-pin
 *    Action   : portENTER_CRITICAL_ISR → tulis gEmergencyLane → portEXIT_CRITICAL_ISR
 *               → xSemaphoreGiveFromISR(emergencySem, &woken)
 *    Ref      : NFR-02: Emergency end-to-end ≤ 100 ms
 *    SMP Note : portENTER_CRITICAL_ISR wajib di ESP32 dual-core untuk
 *               atomic write variabel yang dibaca Core lain.
 * ──────────────────────────────────────────────────────────── */
void IRAM_ATTR isrEmergency_N(void); ///< ISR kendaraan darurat — Jalur Utara (GPIO 34)
void IRAM_ATTR isrEmergency_S(void); ///< ISR kendaraan darurat — Jalur Selatan (GPIO 35)
void IRAM_ATTR isrEmergency_E(void); ///< ISR kendaraan darurat — Jalur Timur (GPIO 36)
void IRAM_ATTR isrEmergency_W(void); ///< ISR kendaraan darurat — Jalur Barat (GPIO 39)

#endif /* TASKS_H */