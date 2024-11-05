#ifndef SIMPLEMOUSE_HIDS_H
#define SIMPLEMOUSE_HIDS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <bluetooth/conn.h>

#include "simplemouse_hids_def.h"

void simplemouse_hids_connected(struct bt_conn *conn);
void simplemouse_hids_disconnected(void);

simplemouse_hids_prot_mode_t simplemouse_hids_get_prot_mode(void);

bool simplemouse_hids_is_mouse_report_writable(void);
void simplemouse_hids_mouse_notify_input(const void* data, uint8_t dataLen);
void simplemouse_hids_mouse_notify_boot(const void* data, uint8_t dataLen);

#ifdef __cplusplus
}
#endif

#endif /* SIMPLEMOUSE_HIDS_H */
