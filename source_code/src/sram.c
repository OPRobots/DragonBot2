#include "sram.h"

#include <string.h>

#include <libopencm3/stm32/timer.h>

/* ──────────────────────────────────────────────────────────────────────────
 * Definiciones del dispositivo SRAM
 * ────────────────────────────────────────────────────────────────────────── */
#define SRAM_CMD_READ 0x03
#define SRAM_CMD_WRITE 0x02
#define SRAM_CMD_RDMR 0x05
#define SRAM_CMD_WRMR 0x01

#define SRAM_MODE_REG_BYTE 0x00
#define SRAM_MODE_REG_SEQUENTIAL 0x40
#define SRAM_MODE_REG_PAGE 0x80

#define SRAM_SIZE_BYTES (512UL * 1024UL)
#define SRAM_HEADER_ADDR 0x000000UL
#define SRAM_RECORDS_START 0x000010UL

#define SRAM_USABLE_END (SRAM_SIZE_BYTES - 3U)
#define RECORDS_AREA_SIZE (SRAM_SIZE_BYTES - SRAM_RECORDS_START)

#define SRAM_MAGIC 0x6D17U
#define SRAM_TYPE_WRAP 0xFFU

typedef struct __attribute__((packed)) {
  uint16_t magic;
  uint16_t num_records;
  uint32_t head_pos;
  uint32_t tail_pos;
  uint32_t bytes_used;
} sram_header_t;

typedef struct __attribute__((packed)) {
  uint8_t type;
  uint16_t payload_size;
} sram_record_wrap_t;

/* ──────────────────────────────────────────────────────────────────────────
 * Sistema de logs: ring de RAM, configuración y estado
 * ────────────────────────────────────────────────────────────────────────── */

#if ((SRAM_LOG_RING_SIZE) == 0u) || (((SRAM_LOG_RING_SIZE) & ((SRAM_LOG_RING_SIZE) - 1u)) != 0u)
#error "SRAM_LOG_RING_SIZE debe ser potencia de 2 y mayor que 0"
#endif

#if (SRAM_LOG_RING_SIZE) > 32768u
#error "SRAM_LOG_RING_SIZE debe ser <= 32768 para usar indices de 16 bits"
#endif

#if (SRAM_LOG_MAX_PAYLOAD) == 0u
#error "SRAM_LOG_MAX_PAYLOAD debe ser mayor que 0"
#endif

#if (SRAM_LOG_FLUSH_BATCH_MAX) == 0u
#error "SRAM_LOG_FLUSH_BATCH_MAX debe ser mayor que 0"
#endif

#define SRAM_LOG_RING_MASK ((uint16_t)((SRAM_LOG_RING_SIZE) - 1u))

/* Comprobaciones en tiempo de compilación de que el buffer del flusher cubre todos los structs de log. Si falla, aumentar SRAM_LOG_MAX_PAYLOAD. */
typedef char sram_static_assert_control[(sizeof(control_log_t) <= (SRAM_LOG_MAX_PAYLOAD)) ? 1 : -1];
typedef char sram_static_assert_log[(sizeof(sram_log_t) <= (SRAM_LOG_MAX_PAYLOAD)) ? 1 : -1];

static uint8_t sram_ring[SRAM_LOG_RING_SIZE];
static volatile uint16_t sram_ring_head = 0;
static volatile uint16_t sram_ring_tail = 0;
static volatile uint16_t sram_log_dropped_count = 0;
static volatile bool sram_log_paused = false;
static volatile bool sram_store_busy = false;

/* ── Diagnóstico ── */
static volatile uint32_t sram_stat_flush_calls = 0;
static volatile uint32_t sram_stat_flush_written = 0;
static volatile uint32_t sram_stat_err_not_init = 0;
static volatile uint32_t sram_stat_err_full = 0;
static volatile uint32_t sram_stat_err_param = 0;
static volatile uint32_t sram_stat_write_last_cycles = 0;
static volatile uint32_t sram_stat_write_max_cycles = 0;
static volatile uint32_t sram_stat_reformats = 0;

/* ── Caché del header en RAM (evita leerlo en cada escritura) ── */
static sram_header_t sram_cached_header;
static bool sram_cached_header_valid = false;
static bool sram_cached_header_dirty = false;

/* Guarda/restaura el estado de interrupciones (PRIMASK). */
static inline uint32_t sram_irq_save(void) {
  uint32_t primask;
  __asm__ volatile("mrs %0, primask" : "=r"(primask));
  __asm__ volatile("cpsid i" ::: "memory");
  return primask;
}

static inline void sram_irq_restore(uint32_t primask) {
  if ((primask & 1u) == 0u) {
    __asm__ volatile("cpsie i" ::: "memory");
  }
}

static uint16_t sram_ring_write(const void *src, uint16_t len, uint16_t pos) {
  const uint8_t *p = (const uint8_t *)src;
  uint16_t first = (uint16_t)(SRAM_LOG_RING_SIZE - pos);
  if (first > len) {
    first = len;
  }
  memcpy(&sram_ring[pos], p, first);
  if (len > first) {
    memcpy(&sram_ring[0], p + first, (size_t)(len - first));
  }
  return (uint16_t)((pos + len) & SRAM_LOG_RING_MASK);
}

static uint16_t sram_ring_read(void *dst, uint16_t len, uint16_t pos) {
  uint8_t *p = (uint8_t *)dst;
  uint16_t first = (uint16_t)(SRAM_LOG_RING_SIZE - pos);
  if (first > len) {
    first = len;
  }
  memcpy(p, &sram_ring[pos], first);
  if (len > first) {
    memcpy(p + first, &sram_ring[0], (size_t)(len - first));
  }
  return (uint16_t)((pos + len) & SRAM_LOG_RING_MASK);
}

/* ──────────────────────────────────────────────────────────────────────────
 * Acceso de bajo nivel al chip SRAM
 * ────────────────────────────────────────────────────────────────────────── */

static void sram_cs_select(void) {
  gpio_clear(GPIOC, GPIO4);
#if (SRAM_CS_DELAY_US) > 0u
  delay_us(SRAM_CS_DELAY_US);
#endif
}

static void sram_cs_deselect(void) {
  gpio_set(GPIOC, GPIO4);
#if (SRAM_CS_DELAY_US) > 0u
  delay_us(SRAM_CS_DELAY_US);
#endif
}

/**
 * @brief Acceso SPI en línea, tocando registros directamente, para minimizar el overhead por byte que tiene spi_xfer().
 *
 */
static inline void sram_spi_wait_txe(void) {
  while ((SPI_SR(SPI3) & SPI_SR_TXE) == 0) {
  }
}

static inline void sram_spi_wait_idle(void) {
  while (SPI_SR(SPI3) & SPI_SR_BSY) {
  }
}

static inline uint8_t sram_spi_byte(uint8_t b) {
  sram_spi_wait_txe();
  SPI_DR(SPI3) = b;
  while ((SPI_SR(SPI3) & SPI_SR_RXNE) == 0) {
  }
  return (uint8_t)SPI_DR(SPI3);
}

static void sram_write_bytes(uint32_t addr, const void *buf, uint16_t len) {
  const uint8_t *p = (const uint8_t *)buf;
  sram_cs_select();

  //? Es una escritura: no hace falta esperar RXNE entre bytes, solo TXE.
  //? Sin llamadas de libopencm3 para intentar optimizar lo máximo posible la velocidad
  sram_spi_wait_txe();
  SPI_DR(SPI3) = SRAM_CMD_WRITE;
  sram_spi_wait_txe();
  SPI_DR(SPI3) = (uint8_t)((addr >> 16) & 0xFF);
  sram_spi_wait_txe();
  SPI_DR(SPI3) = (uint8_t)((addr >> 8) & 0xFF);
  sram_spi_wait_txe();
  SPI_DR(SPI3) = (uint8_t)(addr & 0xFF);
  for (uint16_t i = 0; i < len; i++) {
    sram_spi_wait_txe();
    SPI_DR(SPI3) = p[i];
  }
  sram_spi_wait_txe();
  sram_spi_wait_idle();
  (void)SPI_DR(SPI3);

  sram_cs_deselect();
}

static void sram_read_bytes(uint32_t addr, void *buf, uint16_t len) {
  uint8_t *p = (uint8_t *)buf;
  sram_cs_select();
  (void)sram_spi_byte(SRAM_CMD_READ);
  (void)sram_spi_byte((uint8_t)((addr >> 16) & 0xFF));
  (void)sram_spi_byte((uint8_t)((addr >> 8) & 0xFF));
  (void)sram_spi_byte((uint8_t)(addr & 0xFF));
  for (uint16_t i = 0; i < len; i++) {
    p[i] = sram_spi_byte(0x00);
  }
  sram_spi_wait_idle();
  sram_cs_deselect();
}

static void sram_read_header(sram_header_t *header) {
  sram_read_bytes(SRAM_HEADER_ADDR, header, (uint16_t)sizeof(sram_header_t));
}

static void sram_write_header(const sram_header_t *header) {
  sram_write_bytes(SRAM_HEADER_ADDR, header, (uint16_t)sizeof(sram_header_t));
}

/* Máximo de iteraciones del bucle de overwrite (cota de seguridad). */
#define SRAM_OVERWRITE_MAX_ITER ((uint32_t)(RECORDS_AREA_SIZE / (sizeof(sram_record_wrap_t) + 1u)))

/**
 * @brief Comprueba que los campos del header son coherentes.
 *
 * magic por sí solo no basta: una colisión en el bus puede corromper
 * head_pos/tail_pos/bytes_used dejando el magic intacto.
 */
static bool sram_header_is_valid(const sram_header_t *header) {
  if (header->magic != SRAM_MAGIC) {
    return false;
  }
  if (header->head_pos < SRAM_RECORDS_START || header->head_pos > SRAM_USABLE_END) {
    return false;
  }
  if (header->tail_pos < SRAM_RECORDS_START || header->tail_pos > SRAM_USABLE_END) {
    return false;
  }
  if (header->bytes_used > RECORDS_AREA_SIZE) {
    return false;
  }
  return true;
}

/**
 * @brief Escribe en la SRAM el header que tenemos cacheado
 *
 */
static void sram_cache_store(void) {
  sram_write_header(&sram_cached_header);
  sram_cached_header_dirty = false;
}

/**
 * @brief Carga el header en la caché de RAM. Si está corrupto, formatea.
 * @return true si la caché queda con un header válido.
 */
static bool sram_cache_load(void) {
  sram_read_header(&sram_cached_header);
  if (sram_header_is_valid(&sram_cached_header)) {
    sram_cached_header_valid = true;
    sram_cached_header_dirty = false;
    return true;
  }

  sram_stat_reformats++;
  sram_format();
  return sram_cached_header_valid;
}

static void sram_set_sequential_mode(void) {
  sram_cs_select();
  sram_spi_byte(SRAM_CMD_WRMR);
  sram_spi_byte(SRAM_MODE_REG_SEQUENTIAL);
  sram_cs_deselect();
}

static void sram_write_wrap(sram_header_t *header) {
  uint16_t remaining = (uint16_t)(SRAM_SIZE_BYTES - header->head_pos - sizeof(sram_record_wrap_t));
  sram_record_wrap_t wrap = {
      .type = SRAM_TYPE_WRAP,
      .payload_size = remaining,
  };
  sram_write_bytes(header->head_pos, &wrap, (uint16_t)sizeof(wrap));
  header->bytes_used += (uint32_t)(SRAM_SIZE_BYTES - header->head_pos);
  header->head_pos = SRAM_RECORDS_START;
}

static bool sram_overwrite_oldest(sram_header_t *header) {
  if (header->num_records == 0) {
    return false;
  }

  sram_record_wrap_t record;
  sram_read_bytes(header->tail_pos, &record, (uint16_t)sizeof(record));

  if (record.type == SRAM_TYPE_WRAP) {
    header->bytes_used -= (uint32_t)(sizeof(record) + record.payload_size);
    header->tail_pos = SRAM_RECORDS_START;
    sram_read_bytes(header->tail_pos, &record, (uint16_t)sizeof(record));
  }

  header->bytes_used -= (uint32_t)(sizeof(record) + record.payload_size);
  header->tail_pos += (uint32_t)(sizeof(record) + record.payload_size);
  header->num_records--;
  return true;
}

static sram_result_t sram_write_record(sram_record_type_t type, const void *data, uint16_t size) {
  if (type == SRAM_TYPE_ANY || type == SRAM_TYPE_INVALID || (uint8_t)type == SRAM_TYPE_WRAP) {
    sram_stat_err_param++;
    return SRAM_ERR_PARAM;
  }
  if (data == NULL || size == 0) {
    sram_stat_err_param++;
    return SRAM_ERR_PARAM;
  }

  if (!sram_cached_header_valid) {
    sram_cache_load();
    if (!sram_cached_header_valid) {
      sram_stat_err_not_init++;
      return SRAM_ERR_NOT_INIT;
    }
  }

  uint32_t needed = (uint32_t)sizeof(sram_record_wrap_t) + size;

  if (needed > RECORDS_AREA_SIZE) {
    sram_stat_err_param++;
    return SRAM_ERR_PARAM;
  }

  if (sram_cached_header.head_pos + needed > SRAM_USABLE_END) {
    sram_write_wrap(&sram_cached_header);
  }

  uint32_t overwrite_guard = 0;
  while (sram_cached_header.bytes_used + needed > RECORDS_AREA_SIZE) {
    if (!sram_overwrite_oldest(&sram_cached_header) || ++overwrite_guard > SRAM_OVERWRITE_MAX_ITER) {
      sram_stat_err_full++;
      return SRAM_ERR_FULL;
    }
  }

  /* Una sola transacción SPI: cabecera de registro + payload contiguos.
   * El header global se escribe aparte (diferido) desde sram_store. */
  uint8_t frame[sizeof(sram_record_wrap_t) + SRAM_LOG_MAX_PAYLOAD];
  sram_record_wrap_t record_header = {
      .type = (uint8_t)type,
      .payload_size = size,
  };
  memcpy(frame, &record_header, sizeof(record_header));
  memcpy(&frame[sizeof(record_header)], data, size);
  sram_write_bytes(sram_cached_header.head_pos, frame, (uint16_t)(sizeof(record_header) + size));

  sram_cached_header.bytes_used += needed;
  sram_cached_header.head_pos += needed;
  sram_cached_header.num_records++;
  sram_cached_header_dirty = true;
  return SRAM_OK;
}

void sram_init(void) {
  sram_set_sequential_mode();
  sram_cache_load();
  printf("SRAM Usage: %s\n", sram_free_bytes());
}

void sram_format(void) {
  sram_cached_header.magic = SRAM_MAGIC;
  sram_cached_header.num_records = 0;
  sram_cached_header.head_pos = SRAM_RECORDS_START;
  sram_cached_header.tail_pos = SRAM_RECORDS_START;
  sram_cached_header.bytes_used = 0;
  sram_cached_header_valid = true;
  sram_cache_store();
}

void sram_iterator_init(sram_iterator_t *iterator, sram_record_type_t type) {
  sram_header_t header;
  sram_read_header(&header);
  iterator->filter = type;
  if (sram_header_is_valid(&header)) {
    iterator->_pos = header.tail_pos;
    iterator->_remaining = header.num_records;
  } else {
    iterator->_pos = SRAM_RECORDS_START;
    iterator->_remaining = 0;
  }
}

bool sram_iter_next(sram_iterator_t *iterator, void *buf, uint16_t buf_size, uint16_t *out_size, sram_record_type_t *out_type) {
  while (iterator->_remaining > 0) {
    sram_record_wrap_t record;
    sram_read_bytes(iterator->_pos, &record, (uint16_t)sizeof(record));

    if (record.type == SRAM_TYPE_WRAP) {
      iterator->_pos = SRAM_RECORDS_START;
      sram_read_bytes(iterator->_pos, &record, (uint16_t)sizeof(record));
    }

    iterator->_pos += (uint32_t)sizeof(record);
    iterator->_remaining--;

    if ((iterator->filter == SRAM_TYPE_ANY) || ((sram_record_type_t)record.type == iterator->filter)) {
      if (out_size != NULL) {
        *out_size = record.payload_size;
      }
      if (out_type != NULL) {
        *out_type = (sram_record_type_t)record.type;
      }
      if (buf != NULL && buf_size > 0) {
        uint16_t copy = (record.payload_size < buf_size) ? record.payload_size : buf_size;
        sram_read_bytes(iterator->_pos, buf, copy);
      }
      iterator->_pos += record.payload_size;
      return true;
    }

    iterator->_pos += record.payload_size;
  }
  return false;
}

uint16_t sram_get_total_records(void) {
  sram_header_t header;
  sram_read_header(&header);
  return sram_header_is_valid(&header) ? header.num_records : 0;
}

uint16_t sram_count_records(sram_record_type_t type) {
  sram_header_t header;
  sram_read_header(&header);
  if (!sram_header_is_valid(&header) || header.num_records == 0) {
    return 0;
  }
  if (type == SRAM_TYPE_ANY) {
    return header.num_records;
  }

  uint16_t count = 0;
  sram_iterator_t iterator;
  sram_iterator_init(&iterator, type);

  uint16_t dummy_size;
  while (sram_iter_next(&iterator, NULL, 0, &dummy_size, NULL)) {
    count++;
  }
  return count;
}

char *sram_free_bytes(void) {
  sram_header_t header;
  sram_read_header(&header);
  if (!sram_header_is_valid(&header) || header.bytes_used >= RECORDS_AREA_SIZE) {
    return "0";
  }
  static char buf[32];
  snprintf(buf, sizeof(buf), "%ld/%ld Bytes (%.2f%%)", header.bytes_used, RECORDS_AREA_SIZE, (((100.0f * header.bytes_used) / RECORDS_AREA_SIZE)));
  return buf;
}

/* ──────────────────────────────────────────────────────────────────────────
 * Sistema de logs: productores
 * ────────────────────────────────────────────────────────────────────────── */

bool sram_log(sram_record_type_t type, const void *data, uint16_t size) {
  if (data == NULL || size == 0u || size > (uint16_t)SRAM_LOG_MAX_PAYLOAD || type == SRAM_TYPE_ANY || type == SRAM_TYPE_INVALID ||
      (uint8_t)type == SRAM_TYPE_WRAP) {
    sram_log_dropped_count++;
    return false;
  }

  uint16_t need = (uint16_t)(sizeof(sram_record_wrap_t) + size);

  uint32_t primask = sram_irq_save();
  uint16_t head = sram_ring_head;
  uint16_t tail = sram_ring_tail;
  uint16_t free_bytes = (uint16_t)((tail - head - 1u) & SRAM_LOG_RING_MASK);

  if (need > free_bytes) {
    sram_log_dropped_count++;
    sram_irq_restore(primask);
    return false;
  }

  sram_record_wrap_t frame = {
      .type = (uint8_t)type,
      .payload_size = size,
  };
  head = sram_ring_write(&frame, (uint16_t)sizeof(frame), head);
  head = sram_ring_write(data, size, head);
  __asm__ volatile("" ::: "memory");
  sram_ring_head = head;

  sram_irq_restore(primask);
  return true;
}

uint16_t sram_log_pending(void) {
  return (uint16_t)((sram_ring_head - sram_ring_tail) & SRAM_LOG_RING_MASK);
}

uint16_t sram_log_dropped(void) {
  return sram_log_dropped_count;
}

void sram_log_set_paused(bool paused) {
  sram_log_paused = paused;
}

bool sram_log_is_paused(void) {
  return sram_log_paused;
}

/* ──────────────────────────────────────────────────────────────────────────
 * Sistema de logs: volcado
 * ────────────────────────────────────────────────────────────────────────── */

void sram_store(uint32_t budget_cycles) {
  sram_stat_flush_calls++;

  if (sram_log_paused || sram_store_busy) {
    return;
  }
  sram_store_busy = true;

  uint8_t payload[SRAM_LOG_MAX_PAYLOAD];
  uint16_t budget = SRAM_LOG_FLUSH_BATCH_MAX;
  uint32_t start = read_cycle_counter();

  while (budget > 0u) {
    if ((read_cycle_counter() - start) >= budget_cycles) {
      break;
    }

    uint16_t tail = sram_ring_tail;
    if (tail == sram_ring_head) {
      break;
    }

    sram_record_wrap_t frame;
    uint16_t next = sram_ring_read(&frame, (uint16_t)sizeof(frame), tail);

    if (frame.payload_size == 0u || frame.payload_size > (uint16_t)sizeof(payload)) {
      sram_log_dropped_count++;
      sram_ring_tail = (uint16_t)((next + frame.payload_size) & SRAM_LOG_RING_MASK);
      continue;
    }

    next = sram_ring_read(payload, frame.payload_size, next);

    uint32_t t0 = read_cycle_counter();
    sram_result_t result = sram_write_record((sram_record_type_t)frame.type, payload, frame.payload_size);
    uint32_t elapsed = read_cycle_counter() - t0;

    sram_stat_write_last_cycles = elapsed;
    if (elapsed > sram_stat_write_max_cycles) {
      sram_stat_write_max_cycles = elapsed;
    }

    if (result != SRAM_OK) {
      break;
    }

    sram_stat_flush_written++;
    sram_ring_tail = next;
    budget--;
  }

  /* El header global se escribe una sola vez por pasada (diferido). */
  if (sram_cached_header_dirty) {
    sram_cache_store();
  }

  sram_store_busy = false;
}

/* ──────────────────────────────────────────────────────────────────────────
 * Diagnóstico
 * ────────────────────────────────────────────────────────────────────────── */

// static void sram_debug_report(void) {
//   sram_header_t header;
//   sram_read_header(&header);

//   const uint32_t slot_cycles = (uint32_t)(SYSCLK_FREQUENCY_HZ / SYSTICK_FREQUENCY_HZ);

//   printf("--- SRAM header ---\n");
//   printf("magic=0x%04X (esperado 0x%04X)%s\n", header.magic, (uint16_t)SRAM_MAGIC, (header.magic == SRAM_MAGIC) ? "" : "  <-- CORRUPTO");
//   printf("num_records=%u head_pos=%lu tail_pos=%lu bytes_used=%lu\n", header.num_records, (unsigned long)header.head_pos,
//          (unsigned long)header.tail_pos, (unsigned long)header.bytes_used);
//   printf("header_valido=%s reformats=%lu\n", sram_header_is_valid(&header) ? "si" : "NO", (unsigned long)sram_stat_reformats);
//   printf("free=%s\n", sram_free_bytes());

//   printf("--- ring ---\n");
//   printf("pending=%u bytes  dropped=%u\n", sram_log_pending(), sram_log_dropped());

//   printf("--- flusher ---\n");
//   printf("calls=%lu written=%lu\n", (unsigned long)sram_stat_flush_calls, (unsigned long)sram_stat_flush_written);
//   printf("err not_init=%lu full=%lu param=%lu\n", (unsigned long)sram_stat_err_not_init, (unsigned long)sram_stat_err_full,
//          (unsigned long)sram_stat_err_param);
//   printf("write last=%lu cyc max=%lu cyc (1 slot=%lu cyc)\n", (unsigned long)sram_stat_write_last_cycles,
//          (unsigned long)sram_stat_write_max_cycles, (unsigned long)slot_cycles);
// }

// static void sram_debug_measure_write(void) {
//   const uint32_t slot_cycles = (uint32_t)(SYSCLK_FREQUENCY_HZ / SYSTICK_FREQUENCY_HZ);
//   const uint32_t us_div = (uint32_t)(SYSCLK_FREQUENCY_HZ / 1000000u);
//   const uint32_t scratch = (uint32_t)(SRAM_SIZE_BYTES - 128u); /* zona no usada por el log */

//   /* Baseline: leer los mismos bytes que mueve una escritura. */
//   uint8_t buffer[84];
//   uint32_t primask = sram_irq_save();
//   uint32_t t0 = read_cycle_counter();
//   sram_read_bytes(SRAM_RECORDS_START, buffer, (uint16_t)sizeof(buffer));
//   uint32_t t_read = read_cycle_counter() - t0;
//   sram_irq_restore(primask);

//   /* Registro de prueba (cabecera de registro + payload) a la zona scratch.
//    * No toca el log ni el header: medición no intrusiva. */
//   uint8_t frame[sizeof(sram_record_wrap_t) + sizeof(control_log_t)];
//   sram_record_wrap_t rec = {
//       .type = SRAM_TYPE_CONTROL,
//       .payload_size = sizeof(control_log_t),
//   };
//   memcpy(frame, &rec, sizeof(rec));
//   memset(&frame[sizeof(rec)], 0, sizeof(control_log_t));

//   primask = sram_irq_save();
//   t0 = read_cycle_counter();
//   sram_write_bytes(scratch, frame, (uint16_t)sizeof(frame));
//   uint32_t t_rec = read_cycle_counter() - t0;
//   sram_irq_restore(primask);

//   /* Header global (mismo tamaño que el real). */
//   uint8_t header_bytes[sizeof(sram_header_t)] = {0};
//   primask = sram_irq_save();
//   t0 = read_cycle_counter();
//   sram_write_bytes(scratch, header_bytes, (uint16_t)sizeof(header_bytes));
//   uint32_t t_hdr = read_cycle_counter() - t0;
//   sram_irq_restore(primask);

//   uint32_t t_total = t_rec + t_hdr;
//   uint32_t cap_slot = (t_rec > 0u && slot_cycles > t_hdr) ? ((slot_cycles - t_hdr) / t_rec) : 0u;

//   printf("--- medida atomica (IRQ off, no intrusiva) ---\n");
//   printf("read 84B =%6lu cyc (%5.2f slots, %3lu us)\n", (unsigned long)t_read, (double)t_read / (double)slot_cycles,
//          (unsigned long)(t_read / us_div));
//   printf("record   =%6lu cyc (%5.2f slots, %3lu us)\n", (unsigned long)t_rec, (double)t_rec / (double)slot_cycles,
//          (unsigned long)(t_rec / us_div));
//   printf("header   =%6lu cyc (%5.2f slots, %3lu us)\n", (unsigned long)t_hdr, (double)t_hdr / (double)slot_cycles,
//          (unsigned long)(t_hdr / us_div));
//   printf("1rec+hdr =%6lu cyc (%5.2f slots, %3lu us)  -> %lu regs por slot\n", (unsigned long)t_total,
//          (double)t_total / (double)slot_cycles, (unsigned long)(t_total / us_div), (unsigned long)cap_slot);
// }

static void sram_print_control_log(const void *data, uint16_t size) {
  if (size != sizeof(control_log_t)) {
    printf("ERROR  [control size mismatch %u vs %u]\n", size, (uint16_t)sizeof(control_log_t));
    return;
  }
  const control_log_t *log = (const control_log_t *)data;
  printf("t=%ld,target_linear_speed=%d,ideal_linear_speed=%d,measured_linear_speed=%.2f,left_measured_linear_speed=%.2f,right_measured_linear_speed=%.2f,ideal_angular_speed=%.2f,measured_angular_speed=%.2f,pwm_left=%d,pwm_right=%d,battery_voltage=%.2f\n", log->timestamp_ms,
         (int)log->target_linear_speed, (int)log->ideal_linear_speed, log->measured_linear_speed, log->left_measured_linear_speed,
         log->right_measured_linear_speed, log->ideal_angular_speed, log->measured_angular_speed, log->pwm_left, log->pwm_right,
         log->battery_voltage);
}

void sram_print_logs(sram_record_type_t type) {
  bool was_paused = sram_log_is_paused();
  sram_log_set_paused(true);

  // sram_debug_measure_write();
  // sram_debug_report();
  // printf("sram_print_logs (%d)\n", sram_count_records(type));

  sram_iterator_t iterator;
  sram_iterator_init(&iterator, type);

  sram_log_t buffer;

  uint16_t stored_size;
  sram_record_type_t rec_type;
  while (sram_iter_next(&iterator, &buffer, sizeof(buffer), &stored_size, &rec_type)) {
    if (stored_size > sizeof(buffer)) {
      printf("ERROR  [truncated type 0x%02X: %u bytes > buffer %u]\n", rec_type, stored_size, (uint16_t)sizeof(buffer));
      continue;
    }
    switch (rec_type) {
      case SRAM_TYPE_CONTROL:
        sram_print_control_log(&buffer.control, stored_size);
        break;
      default:
        printf("ERROR  [unknown type 0x%02X, %u bytes]\n", rec_type, stored_size);
        break;
    }
  }

  sram_log_set_paused(was_paused);
}
