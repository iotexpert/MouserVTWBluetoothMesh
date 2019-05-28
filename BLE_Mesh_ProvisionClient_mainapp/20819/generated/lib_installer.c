// /Applications/ModusToolbox_1.1/tools/wiced-scripts-1.0/wiced-gen-lib-installer.pl
// args: 
// output: /Users/arh/Documents/mtw/mouservtw/BLE_Mesh_ProvisionClient_mainapp/20819/generated/lib_installer.c
#include <stdint.h>
typedef struct tag_PATCH_TABLE_ENTRY_t {
	uint32_t breakout;
	uint32_t replacement;
} PATCH_TABLE_ENTRY_t;
void patch_autoInstall(uint32_t old_address, uint32_t new_address);
void patch_autoReplace(uint32_t breakout_address, uint32_t replacement);
void patch_autoReplaceData(uint32_t breakout_address, uint32_t replacement);
void install_libs(void);

void install_libs(void)
{
}
