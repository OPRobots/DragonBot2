#include "sram.h"

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

static void sram_cs_select(void) {
  gpio_clear(GPIOC, GPIO4);
  delay_us(1);
}

static void sram_cs_deselect(void) {
  gpio_set(GPIOC, GPIO4);
  delay_us(1);
}

static uint8_t sram_spi_byte(uint8_t b) {
  return (uint8_t)spi_xfer(SPI3, b);
}

static void sram_send_addr(uint32_t addr) {
  sram_spi_byte((uint8_t)((addr >> 16) & 0xFF));
  sram_spi_byte((uint8_t)((addr >> 8) & 0xFF));
  sram_spi_byte((uint8_t)(addr & 0xFF));
}

static void sram_write_bytes(uint32_t addr, const void *buf, uint16_t len) {
  const uint8_t *p = (const uint8_t *)buf;
  sram_cs_select();
  sram_spi_byte(SRAM_CMD_WRITE);
  sram_send_addr(addr);
  for (uint16_t i = 0; i < len; i++) {
    sram_spi_byte(p[i]);
  }
  sram_cs_deselect();
}

static void sram_read_bytes(uint32_t addr, void *buf, uint16_t len) {
  uint8_t *p = (uint8_t *)buf;
  sram_cs_select();
  sram_spi_byte(SRAM_CMD_READ);
  sram_send_addr(addr);
  for (uint16_t i = 0; i < len; i++) {
    p[i] = sram_spi_byte(0x00);
  }
  sram_cs_deselect();
}

static void sram_read_header(sram_header_t *header) {
  sram_read_bytes(SRAM_HEADER_ADDR, header, (uint16_t)sizeof(sram_header_t));
}

static void sram_write_header(const sram_header_t *header) {
  sram_write_bytes(SRAM_HEADER_ADDR, header, (uint16_t)sizeof(sram_header_t));
}

static void sram_set_sequential_mode(void) {
  sram_cs_select();
  sram_spi_byte(SRAM_CMD_WRMR);
  sram_spi_byte(SRAM_MODE_REG_SEQUENTIAL);
  sram_cs_deselect();
}

static bool sram_is_initialized(void) {
  sram_header_t header;
  sram_read_header(&header);
  return header.magic == SRAM_MAGIC;
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

void sram_init(void) {
  sram_set_sequential_mode();
  if (!sram_is_initialized()) {
    sram_format();
  }
  printf("SRAM Usage: %s\n", sram_free_bytes());
}

void sram_format(void) {
  sram_header_t header = {
      .magic = SRAM_MAGIC,
      .num_records = 0,
      .head_pos = SRAM_RECORDS_START,
      .tail_pos = SRAM_RECORDS_START,
      .bytes_used = 0,
  };
  sram_write_header(&header);
}

sram_result_t sram_write_record(sram_record_type_t type, const void *data, uint16_t size) {
  if (type == SRAM_TYPE_ANY || type == SRAM_TYPE_INVALID || (uint8_t)type == SRAM_TYPE_WRAP) {
    return SRAM_ERR_PARAM;
  }
  if (data == NULL || size == 0) {
    return SRAM_ERR_PARAM;
  }

  sram_header_t header;
  sram_read_header(&header);
  if (header.magic != SRAM_MAGIC) {
    return SRAM_ERR_NOT_INIT;
  }

  uint32_t needed = (uint32_t)sizeof(sram_record_wrap_t) + size;

  if (needed > RECORDS_AREA_SIZE) {
    return SRAM_ERR_PARAM;
  }

  if (header.head_pos + needed > SRAM_USABLE_END) {
    sram_write_wrap(&header);
  }

  while (header.bytes_used + needed > RECORDS_AREA_SIZE) {
    if (!sram_overwrite_oldest(&header)) {
      return SRAM_ERR_FULL;
    }
  }

  sram_record_wrap_t record_header = {
      .type = (uint8_t)type,
      .payload_size = size,
  };
  sram_write_bytes(header.head_pos, &record_header, (uint16_t)sizeof(record_header));
  sram_write_bytes(header.head_pos + (uint32_t)sizeof(record_header), data, size);

  header.bytes_used += needed;
  header.head_pos += needed;
  header.num_records++;

  sram_write_header(&header);
  return SRAM_OK;
}

void sram_iterator_init(sram_iterator_t *iterator, sram_record_type_t type) {
  sram_header_t header;
  sram_read_header(&header);
  iterator->filter = type;
  iterator->_pos = (header.magic == SRAM_MAGIC) ? header.tail_pos : SRAM_RECORDS_START;
  iterator->_remaining = (header.magic == SRAM_MAGIC) ? header.num_records : 0;
}

bool sram_iter_next(sram_iterator_t *iterator, void *buf, uint16_t buf_size, uint16_t *out_size) {
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
  return (header.magic == SRAM_MAGIC) ? header.num_records : 0;
}

uint16_t sram_count_records(sram_record_type_t type) {
  sram_header_t header;
  sram_read_header(&header);
  if (header.magic != SRAM_MAGIC || header.num_records == 0) {
    return 0;
  }
  if (type == SRAM_TYPE_ANY) {
    return header.num_records;
  }

  uint16_t count = 0;
  sram_iterator_t iterator;
  sram_iterator_init(&iterator, type);

  uint16_t dummy_size;
  while (sram_iter_next(&iterator, NULL, 0, &dummy_size)) {
    count++;
  }
  return count;
}

char *sram_free_bytes(void) {
  sram_header_t header;
  sram_read_header(&header);
  if (header.magic != SRAM_MAGIC || header.bytes_used >= RECORDS_AREA_SIZE) {
    return "0";
  }
  static char buf[32];
  snprintf(buf, sizeof(buf), "%ld/%ld Bytes (%.2f%%)", header.bytes_used, RECORDS_AREA_SIZE, (((100.0f * header.bytes_used) / RECORDS_AREA_SIZE)));
  return buf;
}

// void sram_test(void) {
//   for (uint8_t i = 0; i < 5; i++) {
//     sensor_log_t log = {
//         .timestamp_ms = get_clock_ticks(),
//         .line_sensors = {i * 100u, i * 200u, i * 300u, i * 400u},
//         .speed_left = (int16_t)(100 + i * 10),
//         .speed_right = (int16_t)(100 - i * 10),
//         .battery_pct = (uint8_t)(80 - i),
//     };
//     sram_write_record(SRAM_TYPE_SENSOR, &log, sizeof(log));
//     delay(10);
//   }

//   printf("Total records: %u\n", sram_get_total_records());
//   printf("  SENSOR : %u\n", sram_count_records(SRAM_TYPE_SENSOR));

//   sram_iterator_t iterator;

//   printf("\n-- Sensor logs --\n");
//   sram_iterator_init(&iterator, SRAM_TYPE_SENSOR);

//   sensor_log_t log;
//   uint16_t stored_size;
//   while (sram_iter_next(&iterator, &log, sizeof(log), &stored_size)) {
//     if (stored_size != sizeof(log)) {
//       printf("  [skipped - size mismatch %u vs %u]\n", stored_size, (uint16_t)sizeof(log));
//       continue;
//     }
//     printf("  t=%lums  L=%d R=%d  bat=%u%%\n", log.timestamp_ms, (int)log.speed_left, (int)log.speed_right, log.battery_pct);
//   }
// }
