
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <sys/byteorder.h>
#include <zephyr.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>

#include "simplemouse_hids.h"

#define REPORT_MOUSE_SIZE           sizeof(simplemouse_hids_report_mouse_t)
#define BOOT_REPORT_MOUSE_SIZE      sizeof(simplemouse_hids_report_mouse_boot_t)

enum
{
    HIDS_REPORT_ID_MOUSE          =  0x01,
};
typedef uint8_t hids_report_id_t;

/* HIDS Info flags */
enum
{
    HIDS_REMOTE_WAKE            = BIT(0),
    HIDS_NORMALLY_CONNECTABLE   = BIT(1),
};

/* Control Point possible values */
enum
{
    HIDS_CONTROL_POINT_SUSPEND      = 0x00,
    HIDS_CONTROL_POINT_EXIT_SUSPEND = 0x01,

    HIDS_CONTROL_POINT_N
};
typedef uint8_t hids_ctrl_point_t;

/* HID Information Characteristic value */
typedef struct
{
    uint16_t version;   /* version number of base USB HID Specification */
    uint8_t code;       /* country HID Device hardware is localized for. */
    uint8_t flags;

} __packed hids_info_t;

/* Type of reports */
enum
{
    HIDS_REPORT_INPUT   = 0x01,
    HIDS_REPORT_OUTPUT  = 0x02,
    HIDS_REPORT_FEATURE = 0x03,
};
typedef uint8_t hids_report_type_t;

/* HID Report Characteristic Descriptor */
typedef struct
{
    uint8_t id;                 /* report id */
    hids_report_type_t type;    /* report type */

} __packed hids_report_desc_t;

typedef struct
{
    uint8_t* const report_ref;
    uint8_t report_len;

} hids_report_info_t;

static const hids_info_t hids_info =
{
    .version = 0x0101,
    .code = 0x00,
    .flags = HIDS_NORMALLY_CONNECTABLE,
};

/* Reports descriptors (read-only) */
static const hids_report_desc_t mse_input_desc =
{
    .id = HIDS_REPORT_ID_MOUSE,
    .type = HIDS_REPORT_INPUT,
};

/* Module working data */
static struct bt_conn *active_conn;

/* Characteristics and descriptors cached values */
static bool mse_input_rep_notif_enabled;
static bool mse_boot_input_rep_notif_enabled;

static hids_ctrl_point_t ctrl_point;
static simplemouse_hids_prot_mode_t prot_mode;
static uint8_t mse_input_report[REPORT_MOUSE_SIZE];
static uint8_t mse_boot_input_report[BOOT_REPORT_MOUSE_SIZE];

/* Report map */
static const uint8_t report_map[] =
{
    /* MOUSE INPUT REPORT MAP */
    0x05, 0x01,                 // Usage Page (Generic Desktop Ctrls)
    0x09, 0x02,                 // Usage (Mouse)
    0xA1, 0x01,                 // Collection (Application)
    0x85, HIDS_REPORT_ID_MOUSE, //   Report ID (1)
    0x09, 0x01,                 //   Usage (Pointer)
    0xA1, 0x00,                 //   Collection (Physical)
    0x95, 0x02,                 //     Report Count (2)
    0x75, 0x01,                 //     Report Size (1)
    0x15, 0x00,                 //     Logical Minimum (0)
    0x25, 0x01,                 //     Logical Maximum (1)
    0x05, 0x09,                 //     Usage Page (Button)
    0x19, 0x01,                 //     Usage Minimum (0x01)
    0x29, 0x02,                 //     Usage Maximum (0x02)
    0x81, 0x02,                 //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x95, 0x01,                 //     Report Count (1)
    0x75, 0x06,                 //     Report Size (6)
    0x81, 0x03,                 //     Input (Cnst, Var, Abs)
    0x05, 0x01,                 //     Usage Page (Generic Desktop Ctrls)
    0x16, 0x00, 0x80,           //     Logical Minimum (-32768)
    0x26, 0xFF, 0x7F,           //     Logical Maximum (32767)
    0x75, 0x10,                 //     Report Size (16)
    0x95, 0x02,                 //     Report Count (2)
    0x09, 0x30,                 //     Usage (X)
    0x09, 0x31,                 //     Usage (Y)
    0x81, 0x06,                 //     Input (Data,Var,Rel,No Wrap,Linear,Preferred State,No Null Position)
    0x15, 0x81,                 //     Logical Minimum (-127)
    0x25, 0x7F,                 //     Logical Maximum (127)
    0x75, 0x08,                 //     Report Size (8)
    0x95, 0x01,                 //     Report Count (1)
    0x09, 0x38,                 //     Usage (Wheel)
    0x81, 0x06,                 //     Input (Data,Var,Rel,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,                       //   End Collection
    0xC0,                       // End Collection
};

static const hids_report_info_t mse_input_rep_info =
{
    .report_ref = mse_input_report,
    .report_len = sizeof(mse_input_report)
};

static const hids_report_info_t mse_boot_input_rep_info =
{
    .report_ref = mse_boot_input_report,
    .report_len = sizeof(mse_boot_input_report)
};

/*
** --------------------------------------------------------
** Private Functions
** --------------------------------------------------------
*/

static ssize_t s_read_info( struct bt_conn *conn,
                            const struct bt_gatt_attr *attr, void *buf,
                            uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &hids_info, sizeof(hids_info_t));
}

static ssize_t s_read_report_map(   struct bt_conn *conn,
                                    const struct bt_gatt_attr *attr, void *buf,
                                    uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, report_map, sizeof(report_map));
}

static ssize_t s_read_report(   struct bt_conn *conn,
                                const struct bt_gatt_attr *attr, void *buf,
                                uint16_t len, uint16_t offset)
{
    const hids_report_info_t* rep_info_ref = (const hids_report_info_t*) attr->user_data;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, rep_info_ref->report_ref, rep_info_ref->report_len);
}

static ssize_t s_read_prot_mode(struct bt_conn *conn,
                                const struct bt_gatt_attr *attr, void *buf,
                                uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &prot_mode, sizeof(prot_mode));
}

static ssize_t s_read_report_desc(  struct bt_conn *conn,
                                    const struct bt_gatt_attr *attr, void *buf,
                                    uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data, sizeof(hids_report_desc_t));
}

static void s_mse_boot_input_rep_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    mse_boot_input_rep_notif_enabled = (value == BT_GATT_CCC_NOTIFY) ? true : false;
}

static void s_mse_input_rep_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    mse_input_rep_notif_enabled = (value == BT_GATT_CCC_NOTIFY) ? true : false;
}

static ssize_t s_write_ctrl_point(  struct bt_conn *conn,
                                    const struct bt_gatt_attr *attr,
                                    const void *buf, uint16_t len, uint16_t offset,
                                    uint8_t flags)
{
    uint8_t* const ctrl_point_ref = (uint8_t* const)&ctrl_point;
    const uint8_t new_ctrl_pnt = *((const uint8_t *) buf);

    /* Validate flags */
    if (!(flags & BT_GATT_WRITE_FLAG_CMD))
    {
        /* Only write without response accepted */
        return BT_GATT_ERR(BT_ATT_ERR_WRITE_REQ_REJECTED);
    }

    /* Validate length */
    if ((offset + len) > sizeof(ctrl_point))
    {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    /* Validate value */
    if (new_ctrl_pnt >= HIDS_CONTROL_POINT_N)
    {
        return BT_GATT_ERR(BT_ATT_ERR_NOT_SUPPORTED);
    }

    memcpy(ctrl_point_ref + offset, buf, len);

    return len;
}

static ssize_t s_write_prot_mode(   struct bt_conn *conn,
                                    const struct bt_gatt_attr *attr,
                                    const void *buf, uint16_t len, uint16_t offset,
                                    uint8_t flags)
{
    uint8_t* const prot_mode_ref = (uint8_t* const) &prot_mode;
    const simplemouse_hids_prot_mode_t newPm = *((const simplemouse_hids_prot_mode_t *) buf);

    /* Validate flags */
    if (!(flags & BT_GATT_WRITE_FLAG_CMD))
    {
        /* Only write without response accepted */
        return BT_GATT_ERR(BT_ATT_ERR_WRITE_REQ_REJECTED);
    }

    /* Validate length */
    if ((offset + len) > sizeof(prot_mode))
    {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    /* Validate value received */
    if (newPm >= SIMPLEMOUSE_HIDS_PM_N)
    {
        return BT_GATT_ERR(BT_ATT_ERR_NOT_SUPPORTED);
    }

    memcpy(prot_mode_ref + offset, buf, len);

    return len;
}

/* HID Service Definition */
BT_GATT_SERVICE_DEFINE(simplemouse_svc,

    BT_GATT_PRIMARY_SERVICE(BT_UUID_HIDS),

    /* Information Characteristic */
    BT_GATT_CHARACTERISTIC( BT_UUID_HIDS_INFO,
                            BT_GATT_CHRC_READ,
                            BT_GATT_PERM_READ_ENCRYPT,
                            s_read_info, NULL, NULL),

    /* Control Point Characteristic */
    BT_GATT_CHARACTERISTIC( BT_UUID_HIDS_CTRL_POINT,
                            BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                            BT_GATT_PERM_WRITE_ENCRYPT,
                            NULL, s_write_ctrl_point, NULL),

    /* Report Map Characteristic */
    BT_GATT_CHARACTERISTIC( BT_UUID_HIDS_REPORT_MAP,
                            BT_GATT_CHRC_READ,
                            BT_GATT_PERM_READ_ENCRYPT,
                            s_read_report_map, NULL, NULL),

    /* Protocol Mode Characteristic */
    BT_GATT_CHARACTERISTIC( BT_UUID_HIDS_PROTOCOL_MODE,
                            BT_GATT_CHRC_READ |
                            BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                            BT_GATT_PERM_READ_ENCRYPT |
                            BT_GATT_PERM_WRITE_ENCRYPT,
                            s_read_prot_mode, s_write_prot_mode, NULL),

    /* Boot Mouse Input Report Characteristic */
    BT_GATT_CHARACTERISTIC( BT_UUID_HIDS_BOOT_MOUSE_IN_REPORT,
                            BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                            BT_GATT_PERM_READ_ENCRYPT,
                            s_read_report, NULL,
                            (hids_report_info_t*)&mse_boot_input_rep_info),
    BT_GATT_CCC(s_mse_boot_input_rep_ccc_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE_ENCRYPT),

    /* Mouse Input Report Characteristic (+ descriptor) */
    BT_GATT_CHARACTERISTIC( BT_UUID_HIDS_REPORT,
                            BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                            BT_GATT_PERM_READ_ENCRYPT,
                            s_read_report, NULL,
                           (hids_report_info_t*)&mse_input_rep_info),
    BT_GATT_CCC(s_mse_input_rep_ccc_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE_ENCRYPT),
    BT_GATT_DESCRIPTOR( BT_UUID_HIDS_REPORT_REF,
                        BT_GATT_PERM_READ,
                        s_read_report_desc, NULL,
                        (hids_report_desc_t*) &mse_input_desc),
);

void simplemouse_hids_connected(struct bt_conn *conn)
{
    active_conn = conn;
    
    /* Set Protocol Mode back to default (Report), as dictated by the spec */
    prot_mode = SIMPLEMOUSE_HIDS_PM_REPORT;
}

void simplemouse_hids_disconnected(void)
{
    active_conn = NULL;
}

simplemouse_hids_prot_mode_t simplemouse_hids_get_prot_mode(void)
{
    return prot_mode;
}

bool simplemouse_hids_is_mouse_report_writable(void)
{
    /* Return false if connection is still not encrypted */
    if((active_conn == NULL) || (bt_conn_get_security(active_conn) < BT_SECURITY_L2))
    {
        return false;
    }

    if(prot_mode == SIMPLEMOUSE_HIDS_PM_BOOT)
    {
        return mse_boot_input_rep_notif_enabled;
    }
    else
    {
        return mse_input_rep_notif_enabled;
    }
}

void simplemouse_hids_mouse_notify_input(const void* data, uint8_t dataLen)
{
    __ASSERT_NO_MSG(dataLen == sizeof(mse_input_report));
    __ASSERT_NO_MSG(mse_input_rep_notif_enabled);
    __ASSERT_NO_MSG(bt_conn_get_security(active_conn) >= BT_SECURITY_L2);

    int err;
    struct bt_gatt_notify_params params = {0};

    memcpy(mse_input_report, data, dataLen);

    params.uuid = BT_UUID_HIDS_REPORT;
    params.data = data;
    params.len = dataLen;
    params.func = NULL;
    err = bt_gatt_notify_cb(active_conn, &params);
}

void simplemouse_hids_mouse_notify_boot(const void* data, uint8_t dataLen)
{
    __ASSERT_NO_MSG(dataLen == sizeof(mse_boot_input_report));
    __ASSERT_NO_MSG(mse_boot_input_rep_notif_enabled);
    __ASSERT_NO_MSG(bt_conn_get_security(active_conn) >= BT_SECURITY_L2);

    int err;
    struct bt_gatt_notify_params params = {0};

    memcpy(mse_boot_input_report, data, dataLen);

    params.uuid = BT_UUID_HIDS_BOOT_MOUSE_IN_REPORT;
    params.data = data;
    params.len = dataLen;
    params.func = NULL;
    err = bt_gatt_notify_cb(active_conn, &params);
}
