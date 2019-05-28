/*
 * Copyright 2019, Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All Rights Reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software"), is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

#ifdef OTA_FW_UPGRADE_SFLASH_COPY

#include "bt_types.h"
#include "wiced.h"
#include "wiced_hal_sflash.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_trace.h"
#include "wiced_transport.h"
#include "wiced_bt_ota_firmware_upgrade.h"
#include "wiced_firmware_upgrade.h"
#include "wiced_hal_eflash.h"
#include "wiced_hal_wdog.h"
#include "wiced_memory.h"
#include "wiced_platform.h"
#include "spar_utils.h"

#define EFLASH_PAGE 0x200

extern GIVES void* dynamic_memory_AllocatePermanent(    UINT32 size_bytes,
                                                        BOOL32 allow_use_by_minidriver );

#define lpo_ctl_adr             0x00338148
#define wdogcontrol_adr         0x00329008
#define prc_brk_out0_adr        0x00310000
#define prc_patch_reg_en0_adr   0x00310404
#define wdogload_adr            0x00329000
#define wdogintclr_adr          0x0032900c
#define LPO_CTL_LPO_POWER_DOWN_SEL_1_SET (0x01<<9)


void init_wiced_firmware_update_copy_sflash(void);
UINT32 wiced_firmware_update_copy_sflash(int stage);

/* create a DS2 header, it just calls into the entry function (no patch code) */
typedef struct __attribute__((__packed__)) tag_ds_struct
{
    char signature[8];
    uint32_t fill;
    uint32_t size;
    uint8_t call[3];
    void (*entry_function)(void);
    uint8_t terminator[3];
} ds_struct_t;

__attribute__((section (".rodata"),used,nocommon)) const static ds_struct_t ds2 =
{
    .signature = "BRCMcfgD",
    .fill = 0,
    .size = 10,
    .call = {0x06,0x01,0x04},
    .entry_function = init_wiced_firmware_update_copy_sflash,
    .terminator = {0xfe, 0x00, 0x00},
};

//==================================================================================================
// Types
//==================================================================================================
//! Structure for FOUNDATION_CONFIG_ITEM_ID_CONFIG_LAYOUT.
#pragma pack(1)
typedef struct
{
    //! Base address or offset of the failsafe (not upgradable) dynamic section base.  This field
    //! must be present.
    UINT32 failsafe_ds_base;

    //! Base address or offset of the upgradable dynamic section base.  This field is optional for
    //! media types for which DFU is supported.
    UINT32 upgradable_ds_base;

    //! Base address or offset to the area reserved for volatile section copy 1.  Whether this is an
    //! address or offset depends on the media type, and is an internal detail of those media types'
    //! access functions.  Double-buffering of the volatile section alternates between the two
    //! copies when the active copy fills up and has to be consolidated to the other.  The volatile
    //! section stores information that is mutable at runtime, and is therefore subject to loss if a
    //! write operation is interrupted by loss of power.  Only an item that is currently being
    //! written is subject to loss.  Generally, NVRAM media with large page sizes (like flash) use
    //! double-buffering, while media with small page sizes (like EEPROM) allocate one or more
    //! complete pages per volatile section item.
    UINT32 vs_copy1_base;

    //! Base address or offset to the area reserved for volatile section copy 2.  Whether this is an
    //! address or offset depends on the media type, and is an internal detail of those media types'
    //! access functions.  See the documentation for vs_copy1_base, but note that not all media
    //! types use double-buffering.
    UINT32 vs_copy2_base;

    //! Length in bytes per copy of the area reserved for each volatile section copy.  If the target
    //! media uses double buffering to protect against loss, the total space used by the volatile
    //! section is twice this amount.  See the documentation for vs_copy1_base and vs_copy1_base.
    UINT32 vs_length_per_copy;

    //! Block size for volatile section items.  For media with small page sizes (like EEPROM) which
    //! allocate one or more pages per volatile section item, blocks must be a multiple of the media
    //! page size.
    UINT32 vs_block_size;

    //! Media page size.  This info is needed for managing volatile section contents.
    UINT32 media_page_size;
} FOUNDATION_CONFIG_ITEM_CONFIG_LAYOUT_t;
#pragma pack()

//! Enumeration used to specify one of the three sections of config data.
//!                                                                                         <br><br>
//! If config data is stored in NVRAM:
//!                                                                                         <br><br>
//! Static section is written once during manufacturing, and never again.  This section includes
//! per-device information like crystal trimming information and an assigned address like BD_ADDR
//! for Bluetooth devices or a MAC address for ethernet or WLAN devices.  The static section also
//! includes key layout information like whether a volatile section is present and if so, where it
//! is located.
//!                                                                                         <br><br>
//! Dynamic section is written during manufacturing.  This section might be subject to upgrades in
//! the field, by the end user.  An example of such an upgrade process is USB device firmware
//! upgrade.  If this section is subject to upgrade in the field, then a failsafe config must be
//! present, which if present would either force the device into an upgrade-only mode, or fall back
//! to the un-upgraded behavior it would have exhibited when it left the factory.
//!                                                                                         <br><br>
//! Volatile section is used to hold information that can change at runtime, for example storing
//! pairing information for pairing with other devices.  The volatile section is implemented as
//! failsafe as possible for the target media, such that the most recently written "nugget" of
//! information is subject to loss, but contents that were present before a given write operation
//! will be preserved.
//!                                                                                         <br><br>
//! The "volatile" nomenclature is somewhat misleading because this section is only ever present on
//! NVRAM (nonvolatile memory).  The "volatile" nomenclature is simply used to highlight the fact
//! that the contents are subject to loss.  This is generally a non-issue, but if multiple "nuggets"
//! of information are interdependent but written independently, then it is possible for one
//! "nugget" in the interdependent set to be lost, in which case the firmware that uses this
//! information needs to be ready to recognize that situation and take appropriate action to discard
//! or if possible repair the rest of the set.  If no "nuggets" of volatile information form
//! interdependent sets then loss of power during a write operation is functionally equivalent to
//! loss of power immediately before the write operation was initiated.
//!                                                                                         <br><br>
//! If config data is stored in RAM (downloaded by the host):
//!                                                                                         <br><br>
//! Only the static and dynamic sections are relevant.  The distinction between the two halves is
//! more or less irrelevant, merely being a reflection of the NVRAM organization.  Nonetheless, the
//! location in which certain pieces of information are stored is influenced by the NVRAM
//! organization.  A volatile section should never be specified for RAM config data.
typedef enum
{
    //! Configuration data section containing per-device information and key layout information.
    //! The layout information communicates to firmware where to find the rest of the configuration
    //! data.  See the documentation for the config_section_id_t enumeration as a whole for more
    //! complete info.
    CONFIG_STATIC,

    //! Configuration data section containing per-product or product family information.  See the
    //! documentation for the config_section_id_t enumeration as a whole for more complete info.
    CONFIG_DYNAMIC,

    //! Configuration data section in NVRAM containing information that can be changed at runtime.
    //! This refers to info that needs to be preserved across resets or power cycles.  See the
    //! documentation for the config_section_id_t enumeration as a whole for more complete info,
    //! including where the seemingly contradictory name comes from.
    CONFIG_VOLATILE
} config_section_id_t;

//! \internal
//! Structure used internally by the config module to achieve config media abstraction.  It stores
//! layout information for any supported config data media type, as well as media-specific function
//! pointers for various tasks.
typedef struct
{
    //! Access function pointer to read raw data from the media on which config data is stored.
    void    (*fp_ReadRaw)( int offset,
                            config_section_id_t which_section,
                            BYTE* buffer,
                            int length);

    //! Access function pointer to write raw data to the media on which config data is stored.
    void    (*fp_WriteRaw)(int offset,
                            config_section_id_t which_section,
                            BYTE* buffer,
                            int length);

    //! Address of the static section.
    UINT32 ss_base;

    //! Function to handle when the layout config item below has been filled in.  It will have been
    //! filled in using content from the static section, then this function will be called.
    void    (*fp_ConfigLayoutHasBeenSet)(void);

    //! Address of the valid dynamic section (which might be the failsafe copy, or might be the
    //! upgradable copy).
    UINT32 active_ds_base;

    //! Access function pointer to read a volatile section item from config data.  The function is
    //! presented as being specific to the type of media, but it really reflects the partitioning
    //! scheme used by this media as dictated by its physical page size.  The truly media-specific
    //! access function is in fp_ReadRaw.
    UINT16  (*fp_ReadVolatileSectionItem)( UINT16 group_id,
                                            UINT16 sub_id_in_group,
                                            BYTE* buffer,
                                            UINT16 max_length);

    //! Access function pointer to write a volatile section item to config data.  The function is
    //! presented as being specific to the type of media, but it really reflects the partitioning
    //! scheme used by this media as dictated by its physical page size.  The truly media-specific
    //! access function is in fp_WriteRaw.
    void    (*fp_WriteVolatileSectionItem)(UINT16 group_id,
                                            UINT16 sub_id_in_group,
                                            BYTE* buffer,
                                            UINT16 length);

    //! Layout info, retrieved from the static section.
    FOUNDATION_CONFIG_ITEM_CONFIG_LAYOUT_t layout;

    //! Checksum/CRC info for validating segment by segment in the dynamic section.
    UINT32 checksum;
    UINT32 crc32;
    BOOL8 valid_crc32;

    //! Used to allow faster acces to the config if it is memory mapped (not in serial flash for exmaple)
    BOOL8 direct_access;

    //! Whether a valid DS section was found or not.
    BOOL8 valid_ds_found;
} CONFIG_INFO_t;

extern CONFIG_INFO_t        g_config_Info;

typedef PACKED union
{
    UINT8 byte;
    PACKED struct
    {
        UINT8 extra_wrsr : 1;       // workaround for atmel serial flash
        UINT8 page_size_128 : 1; // ATMEL AT25F512 page_size=128, other ATMEL and MXIC, STM page_size=256
        UINT8 poll_btw_prog : 1;    // 1, poll status 0, fixed delay
        UINT8 poll_when_wrsr : 1;   // 1, if need to be conservative
        UINT8 use_ewsr : 1;         // 1, if need ewsr before executing wrsr
        UINT8 fast_read : 1;        // 1, if fast read supported
        UINT8 page_write : 1;       // 1, if page write supported
        UINT8 deep_pwr_dn : 1;      // 1, if can do deep power down

    } attr;

} SF_ATTRIB;
extern SF_ATTRIB sf_attr;

typedef PACKED struct
{
    UINT8 detected : 1;     // 1, if serial flash found
    UINT8 reserved : 7;    //
} SF_STATUS;

extern SF_STATUS sf_status;
extern UINT32 (*boot_init_PostConfigReplacement)(int stage);

void init_wiced_firmware_update_copy_sflash(void)
{
    boot_init_PostConfigReplacement = wiced_firmware_update_copy_sflash;
}

UINT32 wiced_firmware_update_copy_sflash(int stage)
{
    uint8_t *buffer;
    int32_t offset = 0;
    uint32_t write_base = g_config_Info.layout.failsafe_ds_base - 0x500000;

    if(stage != 4)
        return 0;

    // set up uart for debug print
    wiced_hal_puart_init();

    if(0 == memcmp(PLATFORM, "CYBT_213", 8))
    {
        wiced_hal_puart_select_uart_pads(WICED_P28, WICED_P00, 0, 0);
        // set up gpio for sflash
        wiced_hal_gpio_select_function(WICED_P10, WICED_SPI_2_MOSI);
        wiced_hal_gpio_select_function(WICED_P02, WICED_SPI_2_CLK);
        wiced_hal_gpio_select_function(WICED_P15, WICED_SPI_2_CS);
        wiced_hal_gpio_select_function(WICED_P08, WICED_SPI_2_MISO);
    }
    else
    {
        wiced_hal_puart_select_uart_pads(WICED_P37, WICED_P32, 0, 0);
        // set up gpio for sflash
        wiced_hal_gpio_select_function(WICED_P06, WICED_SPI_2_MOSI);
        wiced_hal_gpio_select_function(WICED_P09, WICED_SPI_2_CLK);
        wiced_hal_gpio_select_function(WICED_P11, WICED_SPI_2_CS);
        wiced_hal_gpio_select_function(WICED_P17, WICED_SPI_2_MISO);
    }
    wiced_hal_puart_print("!!!!! wiced_firmware_update_copy_sflash\n");

    /* disable watchdog, set up SWD, wait for attach if ENABLE_DEBUG */
    SETUP_APP_FOR_DEBUG_IF_DEBUG_ENABLED();
    BUSY_WAIT_TILL_MANUAL_CONTINUE_IF_DEBUG_ENABLED();

    REG32(wdogload_adr) = 128000; // pet watchdog

   // set up sflash via spi2
    wiced_hal_sflash_init();
    sf_attr.byte = 0xEC;
    sf_status.detected = 1;

    do
    {
        buffer = dynamic_memory_AllocatePermanent(EFLASH_PAGE, 0);
        if(NULL == buffer)
            break;

        // read in first block from eflash and get size
        wiced_hal_puart_print("!!!!! read eflash\n");
        if(WICED_SUCCESS != wiced_hal_eflash_read(write_base, buffer, EFLASH_PAGE))
        {
            wiced_hal_puart_print("!!!!! bad eflash read\n");
            break;
        }
        memcpy((uint8_t *)&offset, buffer, sizeof(offset));
        if((offset < 0x1a) || (offset > FLASH_SIZE))
        {
            wiced_hal_puart_print("!!!!! bad update length read from eflash\n");
            break;
        }

        wiced_hal_puart_print("!!!!! got len\n");

        // start at end, also reading and writing to end of page past data length
        offset &= ~(EFLASH_PAGE - 1);

        while(offset >= 0)
        {
            REG32(wdogload_adr) = 128000; // pet watchdog
            // copy to eflash block by block and continue with remaining
            if(EFLASH_PAGE != wiced_hal_sflash_read(offset, EFLASH_PAGE, buffer))
            {
                wiced_hal_puart_print("!!!!! bad sflash read\n");
                break;
            }
            if(WICED_SUCCESS != wiced_hal_eflash_erase(write_base+offset, EFLASH_PAGE))
            {
                wiced_hal_puart_print("!!!!! bad erase\n");
                break;
            }
            if(WICED_SUCCESS != wiced_hal_eflash_write(write_base+offset, buffer, EFLASH_PAGE))
            {
                wiced_hal_puart_print("!!!!! bad write\n");
                break;
            }
            offset -= EFLASH_PAGE;
           // wiced_hal_puart_print("!!!!! wrote DS1 chunk\n");
        }

        // the data is committed to eflash a page at a time, cannot just commit signature at end
        wiced_hal_puart_print("!!!!! all data transferred to eflash\n");

        // nmi is the vector instead of watchdog, need to cut off watchdog from coredump
        // opcode 0xe7fe is "B here"
        REG32(0x270000) = 0xe7fee7fe;
        REG32(prc_brk_out0_adr) = 0x178 >> 2;
        REG32(prc_patch_reg_en0_adr) = 1;
    } while(0);

    wiced_hal_puart_print("!!!!! exit wiced_firmware_update_copy_sflash\n");

    REG32(lpo_ctl_adr) &= ~LPO_CTL_LPO_POWER_DOWN_SEL_1_SET;
    // turn off watchdog interrupt
    REG32(wdogcontrol_adr) = 0;
    REG32(wdogload_adr) = 5; // small delay
    REG32(wdogintclr_adr) = 0;
    REG32(wdogcontrol_adr) = 3;
    while(1);                  /* wait until reset */

    return 0;
}

#endif // OTA_FW_UPGRADE_SFLASH_COPY
