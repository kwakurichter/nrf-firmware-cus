/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

/* The nRF51822-QFAA has 16K of RAM at 0x20000000.
 *
 * Stock firmware starts the application at 0x20001fe8 because the S130
 * softdevice claims the low 8168 bytes while it is enabled, leaving only 8216
 * bytes for the application. This firmware never enables the softdevice --
 * BLE is gone and there are no sd_* calls left -- so that reservation is dead
 * space and the application can have the whole 16K.
 *
 * The softdevice image stays in flash: the MBR at 0x0 is what the boot chain
 * (MBR -> MBS -> bootloader -> firmware) runs through, and it is write
 * protected. Only the RAM reservation is reclaimed here.
 *
 * The first word of RAM is NOT ours. The nRF51 is a Cortex-M0 with no VTOR,
 * so the vector table cannot be relocated; Nordic works around that by having
 * the MBR catch every interrupt and forward it through a pointer stored at
 * 0x20000000. That word is live whether or not the softdevice is enabled, and
 * overwriting it sends the first interrupt after startup into garbage. Only
 * four bytes are reserved, but the origin is moved to 0x20000008 to keep the
 * 8-byte alignment that .data wants.
 *
 * Consequence: sd_softdevice_enable() must never be called in this build. It
 * would corrupt whatever the application has placed in the low 8K.
 */
MEMORY
{
  FLASH (rx) : ORIGIN = 0x1b000, LENGTH = 0x25000
  RAM (rwx) :  ORIGIN = 0x20000008, LENGTH = 0x3ff8
}

SECTIONS
{
  .fs_data :
  {
    PROVIDE(__start_fs_data = .);
    KEEP(*(.fs_data))
    PROVIDE(__stop_fs_data = .);
  } > RAM
  .pwr_mgmt_data :
  {
    PROVIDE(__start_pwr_mgmt_data = .);
    KEEP(*(.pwr_mgmt_data))
    PROVIDE(__stop_pwr_mgmt_data = .);
  } > RAM
} INSERT AFTER .data;

INCLUDE "nrf5x_common.ld"
