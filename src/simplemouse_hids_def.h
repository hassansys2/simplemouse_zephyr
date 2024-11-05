#ifndef SIMPLEMOUSE_HIDS_DEF_H
#define SIMPLEMOUSE_HIDS_DEF_H

#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>

/* Protocol Mode */
enum
{
    SIMPLEMOUSE_HIDS_PM_BOOT    = 0x00,
    SIMPLEMOUSE_HIDS_PM_REPORT  = 0x01,

    SIMPLEMOUSE_HIDS_PM_N  
};
typedef uint8_t simplemouse_hids_prot_mode_t;

/*
** --------------------------------------------------------
** Mouse reports descriptor (custom and boot).
** NOTE: these are the packed structs being sent over-the-air
** --------------------------------------------------------
*/

/* Mouse Input Report format is defined according to the Report Map characteristic:
 *  8 bits - pressed buttons bitmask (2 LSB used + 6 MSB padding)
 *  16 bits - x movement
 *  16 bits - y movement
 *  8 bits - vertical wheel rotation (device specific)
 */
typedef struct
{
    uint8_t         buttons_bitmask;
    uint8_t         move_x_lsb;
    uint8_t         move_x_msb;
    uint8_t         move_y_lsb;
    uint8_t         move_y_msb;
    uint8_t         scroll_v;
} __packed simplemouse_hids_report_mouse_t;

/* Boot Report is fixed to 4 bytes and CANNOT be modified:
 *  8 bits - pressed buttons bitmask
 *  8 bits - x movement
 *  8 bits - y movement
 *  8 bits - vertical wheel rotation (device specific)
 */
typedef struct
{
    uint8_t         buttons_bitmask;
    int8_t          move_x;
    int8_t          move_y;
    int8_t          scroll_v;

} __packed simplemouse_hids_report_mouse_boot_t;


#ifdef __cplusplus
}
#endif

#endif /* SIMPLEMOUSE_HIDS_DEF_H */
