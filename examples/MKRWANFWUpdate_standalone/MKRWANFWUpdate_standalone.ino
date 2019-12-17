/* 
 *  STANDALONE FIRMWARE UPDATE FOR MKR WAN 1300/1310
 *  This sketch implements STM32 bootloader protocol
 *  It is based on stm32flash (mirrored here git@github.com:facchinm/stm32flash.git) 
 *  with as little modifications as possible.
 *  
 *  To generate it after a firmware update, execute
 *  
 *  echo -n "const " > fw.h && xxd -i mlm32l07x01.bin >> fw.h
 *  
 */


#include "fw.h"
#include "stm32.h"
#include "serial_arduino.h"
#include <MKRWAN.h>

/* device globals */
stm32_t    *stm    = NULL;
void       *p_st   = NULL;

int ret = -1;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  while (!Serial);

  struct port_interface port;
  struct port_options port_opts = {
    .baudRate       = 115200,
    .serial_mode    = SERIAL_8E1
  };

  port.flags =  PORT_CMD_INIT | PORT_GVR_ETX | PORT_BYTE | PORT_RETRY;
  port.dev   =  &Serial2;
  port.ops   =  &port_opts;

  assignCallbacks(&port);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LORA_BOOT0, OUTPUT);
  digitalWrite(LORA_BOOT0, HIGH);
  pinMode(LORA_RESET, OUTPUT);
  digitalWrite(LORA_RESET, HIGH);
  delay(200);
  digitalWrite(LORA_RESET, LOW);
  delay(200);
  digitalWrite(LORA_RESET, HIGH);
  delay(200);

  Serial.println("Press a key to start FW update");
  port.open(&port);
  port.flush(&port);

  stm = stm32_init(&port, 1);

  fprintf(diag, "Version      : 0x%02x\n", stm->bl_version);
  if (port.flags & PORT_GVR_ETX) {
    fprintf(diag, "Option 1     : 0x%02x\n", stm->option1);
    fprintf(diag, "Option 2     : 0x%02x\n", stm->option2);
  }
  fprintf(diag, "Device ID    : 0x%04x (%s)\n", stm->pid, stm->dev->name);
  fprintf(diag, "- RAM        : Up to %dKiB  (%db reserved by bootloader)\n", (stm->dev->ram_end - 0x20000000) / 1024, stm->dev->ram_start - 0x20000000);
  fprintf(diag, "- Flash      : Up to %dKiB (size first sector: %dx%d)\n", (stm->dev->fl_end - stm->dev->fl_start ) / 1024, stm->dev->fl_pps, stm->dev->fl_ps[0]);
  fprintf(diag, "- Option RAM : %db\n", stm->dev->opt_end - stm->dev->opt_start + 1);
  fprintf(diag, "- System RAM : %dKiB\n", (stm->dev->mem_end - stm->dev->mem_start) / 1024);

  uint8_t   buffer[256];
  uint32_t  addr, start, end;
  unsigned int  len;
  int   failed = 0;
  int   first_page, num_pages;

  int npages = mlm32l07x01_bin_len / 128 + 1;
  int spage = 0;
  bool verify = 1;
  int retry = 10;
  bool reset_flag = 0;
  bool exec_flag = 1;
  int execute = 0;  // address

  /*
    Cleanup addresses:

    Starting from options
     start_addr, readwrite_len, spage, npages
    and using device memory size, compute
     start, end, first_page, num_pages
  */
  if (!npages) {
    start = stm->dev->fl_start;
    end = stm->dev->fl_end;
    first_page = 0;
    num_pages = STM32_MASS_ERASE;
  } else {
    first_page = spage;
    start = flash_page_to_addr(first_page);
    if (start > stm->dev->fl_end) {
      fprintf(stderr, "Address range exceeds flash size.\n");
      ret = -1;
      return;
    }

    if (npages) {
      num_pages = npages;
      end = flash_page_to_addr(first_page + num_pages);
      if (end > stm->dev->fl_end)
        end = stm->dev->fl_end;
    } else {
      end = stm->dev->fl_end;
      num_pages = flash_addr_to_page_ceil(end) - first_page;
    }

    if (!first_page && end == stm->dev->fl_end)
      num_pages = STM32_MASS_ERASE;
  }

  ret = 0;
  int s_err;

#if 0
  fprintf(diag, "Erasing flash\n");

  if (num_pages != STM32_MASS_ERASE &&
      (start != flash_page_to_addr(first_page)
       || end != flash_page_to_addr(first_page + num_pages))) {
    fprintf(stderr, "Specified start & length are invalid (must be page aligned)\n");
    ret = 1;
    return;
  }

  s_err = stm32_erase_memory(stm, first_page, num_pages);
  if (s_err != STM32_ERR_OK) {
    fprintf(stderr, "Failed to erase memory\n");
    ret = 1;
    return;
  }
  ret = 0;

#endif

  fprintf(diag, "Write to memory\n");

  off_t   offset = 0;
  ssize_t r;
  unsigned int size;
  unsigned int max_wlen, max_rlen;

#define STM32_MAX_RX_FRAME  256 /* cmd read memory */
#define STM32_MAX_TX_FRAME  (1 + 256 + 1) /* cmd write memory */

  max_wlen = STM32_MAX_TX_FRAME - 2;  /* skip len and crc */
  max_wlen &= ~3; /* 32 bit aligned */

  max_rlen = STM32_MAX_RX_FRAME;
  max_rlen = max_rlen < max_wlen ? max_rlen : max_wlen;

  /* Assume data from stdin is whole device */
  size = end - start;

  // TODO: It is possible to write to non-page boundaries, by reading out flash
  //       from partial pages and combining with the input data
  // if ((start % stm->dev->fl_ps[i]) != 0 || (end % stm->dev->fl_ps[i]) != 0) {
  //  fprintf(stderr, "Specified start & length are invalid (must be page aligned)\n");
  //  goto close;
  // }

  // TODO: If writes are not page aligned, we should probably read out existing flash
  //       contents first, so it can be preserved and combined with new data
  if (num_pages) {
    fprintf(diag, "Erasing memory\n");
    s_err = stm32_erase_memory(stm, first_page, num_pages);
    if (s_err != STM32_ERR_OK) {
      fprintf(stderr, "Failed to erase memory\n");
      ret = -1;
      return;
    }
  }

  addr = start;
  while (addr < end && offset < size) {
    uint32_t left = end - addr;
    len   = max_wlen > left ? left : max_wlen;
    len   = len > size - offset ? size - offset : len;

    memcpy(buffer, &mlm32l07x01_bin[offset], len);

    if (len == 0) {
      fprintf(stderr, "Failed to read input file\n");
      ret = -1;
      return;
    }

again:
    s_err = stm32_write_memory(stm, addr, buffer, len);
    if (s_err != STM32_ERR_OK) {
      fprintf(stderr, "Failed to write memory at address 0x%08x\n", addr);
      ret = -1;
      return;
    }

    if (verify) {
      uint8_t compare[len];
      unsigned int offset, rlen;

      offset = 0;
      while (offset < len) {
        rlen = len - offset;
        rlen = rlen < max_rlen ? rlen : max_rlen;
        s_err = stm32_read_memory(stm, addr + offset, compare + offset, rlen);
        if (s_err != STM32_ERR_OK) {
          fprintf(stderr, "Failed to read memory at address 0x%08x\n", addr + offset);
          ret = -1;
          return;
        }
        offset += rlen;
      }

      for (r = 0; r < len; ++r)
        if (buffer[r] != compare[r]) {
          if (failed == retry) {
            fprintf(stderr, "Failed to verify at address 0x%08x, expected 0x%02x and found 0x%02x\n",
                    (uint32_t)(addr + r),
                    buffer [r],
                    compare[r]
                   );
            ret = -1;
            return;
          }
          ++failed;
          goto again;
        }

      failed = 0;
    }

    addr  += len;
    offset  += len;

    fprintf(diag,
            "Wrote %saddress 0x%08x (%d%%)\n ",
            verify ? "and verified " : "",
            addr,
            100 * offset / size
           );

  }

  fprintf(diag, "Done.\n");
  ret = 0;

  if (stm && exec_flag && ret == 0) {
    if (execute == 0)
      execute = stm->dev->fl_start;

    fprintf(diag, "\nStarting execution at address 0x%08x... ", execute);
    if (stm32_go(stm, execute) == STM32_ERR_OK) {
      reset_flag = 0;
      fprintf(diag, "done.\n");
    } else
      fprintf(diag, "failed.\n");
  }
}

void resetModuleRunning() {
  digitalWrite(LORA_BOOT0, LOW);
  Serial2.end();
  Serial2.begin(19200);
  delay(100);
  digitalWrite(LORA_RESET, HIGH);
  delay(100);
  digitalWrite(LORA_RESET, LOW);
  delay(100);
  digitalWrite(LORA_RESET, HIGH);
  delay(100);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (ret == 0) {
    Serial.println("Flashing ok :)");
    LoRaModem* modem = new LoRaModem();
    modem->begin(EU868);
    Serial.println(modem->version());
  }
  while (1);
}

static int is_addr_in_ram(uint32_t addr)
{
  return addr >= stm->dev->ram_start && addr < stm->dev->ram_end;
}

static int is_addr_in_flash(uint32_t addr)
{
  return addr >= stm->dev->fl_start && addr < stm->dev->fl_end;
}

static int is_addr_in_opt_bytes(uint32_t addr)
{
  /* option bytes upper range is inclusive in our device table */
  return addr >= stm->dev->opt_start && addr <= stm->dev->opt_end;
}

static int is_addr_in_sysmem(uint32_t addr)
{
  return addr >= stm->dev->mem_start && addr < stm->dev->mem_end;
}

/* returns the page that contains address "addr" */
static int flash_addr_to_page_floor(uint32_t addr)
{
  int page;
  uint32_t *psize;

  if (!is_addr_in_flash(addr))
    return 0;

  page = 0;
  addr -= stm->dev->fl_start;
  psize = stm->dev->fl_ps;

  while (addr >= psize[0]) {
    addr -= psize[0];
    page++;
    if (psize[1])
      psize++;
  }

  return page;
}

/* returns the first page whose start addr is >= "addr" */
int flash_addr_to_page_ceil(uint32_t addr)
{
  int page;
  uint32_t *psize;

  if (!(addr >= stm->dev->fl_start && addr <= stm->dev->fl_end))
    return 0;

  page = 0;
  addr -= stm->dev->fl_start;
  psize = stm->dev->fl_ps;

  while (addr >= psize[0]) {
    addr -= psize[0];
    page++;
    if (psize[1])
      psize++;
  }

  return addr ? page + 1 : page;
}

/* returns the lower address of flash page "page" */
static uint32_t flash_page_to_addr(int page)
{
  int i;
  uint32_t addr, *psize;

  addr = stm->dev->fl_start;
  psize = stm->dev->fl_ps;

  for (i = 0; i < page; i++) {
    addr += psize[0];
    if (psize[1])
      psize++;
  }

  return addr;
}
