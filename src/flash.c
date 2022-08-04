#include "flash.h"

#include <string.h>

#include "drv_fmc.h"
#include "drv_serial.h"
#include "failloop.h"
#include "io/vtx.h"
#include "profile.h"
#include "project.h"
#include "util/cbor_helper.h"

extern const profile_t default_profile;
extern profile_t profile;

flash_storage_t flash_storage;
rx_bind_storage_t bind_storage;

CBOR_START_STRUCT_ENCODER(rx_bind_storage_t)
CBOR_ENCODE_MEMBER(bind_saved, uint8)
CBOR_ENCODE_BSTR_MEMBER(raw, BIND_RAW_STORAGE_SIZE)
CBOR_END_STRUCT_ENCODER()

CBOR_START_STRUCT_DECODER(rx_bind_storage_t)
CBOR_DECODE_MEMBER(bind_saved, uint8)
CBOR_DECODE_BSTR_MEMBER(raw, BIND_RAW_STORAGE_SIZE)
CBOR_END_STRUCT_DECODER()

void flash_save() {
  fmc_unlock();
  fmc_erase();

  {
    fmc_write(TARGET_STORAGE_OFFSET, FMC_MAGIC | TARGET_STORAGE_OFFSET);

    const uint32_t offset = TARGET_STORAGE_OFFSET + FMC_MAGIC_SIZE;
    const uint32_t size = TARGET_STORAGE_SIZE - FMC_MAGIC_SIZE;

    uint8_t buffer[size];
    memset(buffer, 0, size);

    cbor_value_t enc;
    cbor_encoder_init(&enc, buffer, size);

    cbor_result_t res = cbor_encode_target_t(&enc, &target);
    if (res < CBOR_OK) {
      fmc_lock();
      failloop(FAILLOOP_FAULT);
    }

    fmc_write_buf(offset, buffer, size);
  }

  {
    fmc_write(FLASH_STORAGE_OFFSET, FMC_MAGIC | FLASH_STORAGE_OFFSET);

    const uint32_t offset = FLASH_STORAGE_OFFSET + FMC_MAGIC_SIZE;
    const uint32_t size = FLASH_STORAGE_SIZE - FMC_MAGIC_SIZE;

    uint8_t buffer[size];
    memcpy(buffer, (uint8_t *)&flash_storage, sizeof(flash_storage_t));
    fmc_write_buf(offset, buffer, size);
  }

  {
    fmc_write(BIND_STORAGE_OFFSET, FMC_MAGIC | BIND_STORAGE_OFFSET);

    const uint32_t offset = BIND_STORAGE_OFFSET + FMC_MAGIC_SIZE;
    const uint32_t size = BIND_STORAGE_SIZE - FMC_MAGIC_SIZE;

    uint8_t buffer[size];
    memset(buffer, 0, size);

    if (bind_storage.bind_saved == 0) {
      // reset all bind data
      memset(bind_storage.raw, 0, BIND_RAW_STORAGE_SIZE);
    }

    memcpy(buffer, (uint8_t *)&bind_storage, sizeof(rx_bind_storage_t));

    fmc_write_buf(offset, buffer, size);
  }

  {
    fmc_write(PROFILE_STORAGE_OFFSET, FMC_MAGIC | PROFILE_STORAGE_OFFSET);

    const uint32_t offset = PROFILE_STORAGE_OFFSET + FMC_MAGIC_SIZE;
    const uint32_t size = PROFILE_STORAGE_SIZE - FMC_MAGIC_SIZE;

    uint8_t buffer[size];

    cbor_value_t enc;
    cbor_encoder_init(&enc, buffer, size);

    cbor_result_t res = cbor_encode_profile_t(&enc, &profile);
    if (res < CBOR_OK) {
      fmc_lock();
      failloop(FAILLOOP_FAULT);
    }

    fmc_write_buf(offset, buffer, size);
  }

  {
    fmc_write(TARGET_STORAGE_OFFSET, FMC_MAGIC | VTX_STORAGE_OFFSET);

    const uint32_t offset = VTX_STORAGE_OFFSET + FMC_MAGIC_SIZE;
    const uint32_t size = VTX_STORAGE_SIZE - FMC_MAGIC_SIZE;

    uint8_t buffer[size];

    cbor_value_t enc;
    cbor_encoder_init(&enc, buffer, size);

    cbor_result_t res = cbor_encode_vtx_settings_t(&enc, &vtx_settings);
    if (res < CBOR_OK) {
      fmc_lock();
      failloop(FAILLOOP_FAULT);
    }

    fmc_write_buf(offset, buffer, size);
  }

  fmc_lock();
}

void flash_load() {

  if (fmc_read(TARGET_STORAGE_OFFSET) == (FMC_MAGIC | TARGET_STORAGE_OFFSET)) {
    const uint32_t offset = TARGET_STORAGE_OFFSET + FMC_MAGIC_SIZE;
    const uint32_t size = TARGET_STORAGE_SIZE - FMC_MAGIC_SIZE;

    uint8_t buffer[size];
    fmc_read_buf(offset, buffer, size);

    cbor_value_t dec;
    cbor_decoder_init(&dec, buffer, size);
    cbor_decode_target_t(&dec, &target);
  }

  if (fmc_read(FLASH_STORAGE_OFFSET) == (FMC_MAGIC | FLASH_STORAGE_OFFSET)) {
    const uint32_t offset = FLASH_STORAGE_OFFSET + FMC_MAGIC_SIZE;
    const uint32_t size = FLASH_STORAGE_SIZE - FMC_MAGIC_SIZE;

    uint8_t buffer[size];

    fmc_read_buf(offset, buffer, size);
    memcpy((uint8_t *)&flash_storage, buffer, sizeof(flash_storage_t));
  }

  if (fmc_read(BIND_STORAGE_OFFSET) == (FMC_MAGIC | BIND_STORAGE_OFFSET)) {
    const uint32_t offset = BIND_STORAGE_OFFSET + FMC_MAGIC_SIZE;
    const uint32_t size = BIND_STORAGE_SIZE - FMC_MAGIC_SIZE;

    uint8_t buffer[size];

    fmc_read_buf(offset, buffer, size);
    memcpy((uint8_t *)&bind_storage, buffer, sizeof(rx_bind_storage_t));

#ifdef RX_BAYANG_PROTOCOL_TELEMETRY_AUTOBIND
    extern int rx_bind_load;
    rx_bind_load = bind_storage.bind_saved;
#endif
  } else {
#ifdef EXPRESS_LRS_UID
    const uint8_t uid[6] = {EXPRESS_LRS_UID};
    bind_storage.bind_saved = 1;

    bind_storage.elrs.is_set = 0x1;
    bind_storage.elrs.magic = 0x37;
    memcpy(bind_storage.elrs.uid, uid, 6);
#endif
  }

  if (fmc_read(PROFILE_STORAGE_OFFSET) == (FMC_MAGIC | PROFILE_STORAGE_OFFSET)) {
    const uint32_t offset = PROFILE_STORAGE_OFFSET + FMC_MAGIC_SIZE;
    const uint32_t size = PROFILE_STORAGE_SIZE - FMC_MAGIC_SIZE;

    uint8_t buffer[size];
    fmc_read_buf(offset, buffer, size);

    cbor_value_t dec;
    cbor_decoder_init(&dec, buffer, size);

    cbor_result_t res = cbor_decode_profile_t(&dec, &profile);
    if (res < CBOR_OK) {
      fmc_lock();
      failloop(FAILLOOP_FAULT);
    }
  }

  if (fmc_read(VTX_STORAGE_OFFSET) == (FMC_MAGIC | VTX_STORAGE_OFFSET)) {
    const uint32_t offset = VTX_STORAGE_OFFSET + FMC_MAGIC_SIZE;
    const uint32_t size = VTX_STORAGE_SIZE - FMC_MAGIC_SIZE;

    uint8_t buffer[size];

    fmc_read_buf(offset, buffer, size);

    cbor_value_t dec;
    cbor_decoder_init(&dec, buffer, size);

    cbor_result_t res = cbor_decode_vtx_settings_t(&dec, &vtx_settings);
    if (res < CBOR_OK) {
      fmc_lock();
      failloop(FAILLOOP_FAULT);
    }
  }
}
