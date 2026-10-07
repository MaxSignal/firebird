#include <assert.h>
#include <string.h>

#include <mutex>

#include "emu.h"
#include "misc.h"
#include "keypad.h"
#include "schedule.h"
#include "interrupt.h"
#include "mem.h"

/* 900E0000: Keypad controller */
keypad_state keypad;

static std::recursive_mutex keypad_mut;

void keypad_int_check() {
    std::lock_guard<std::recursive_mutex> lg(keypad_mut);
    if(keypad.touchpad_contact != keypad.touchpad_last_contact)
        keypad.touchpad_irq_state |= 0x4;
    if(keypad.touchpad_down != keypad.touchpad_last_down)
        keypad.touchpad_irq_state |= 0x8;

    keypad.touchpad_last_contact = keypad.touchpad_contact;
    keypad.touchpad_last_down = keypad.touchpad_down;

    int_set(INT_KEYPAD, (keypad.kpc.int_enable & keypad.kpc.int_active)
                      | (keypad.kpc.gpio_int_enable & keypad.kpc.gpio_int_active));
}

void keypad_on_pressed() {
    // TODO: The CX II may have an enable bit somewhere
    if(!emulate_cx2 && pmu.on_irq_enabled) {
        pmu.int_status |= 1;
        int_set(INT_POWER, true);
    }

    if(cpu_events & EVENT_SLEEP) {
        assert(emulate_cx2);
        cpu_reset();
    }
}

uint32_t keypad_read(uint32_t addr) {
    std::lock_guard<std::recursive_mutex> lg(keypad_mut);
    cycle_count_delta += 1000; // avoid slowdown with polling loops
    switch (addr & 0x7F) {
        case 0x00: return keypad.kpc.control;
        case 0x04: return keypad.kpc.size;
        case 0x08: return keypad.kpc.int_active;
        case 0x0C: return keypad.kpc.int_enable;
        case 0x10: case 0x14: case 0x18: case 0x1C:
        case 0x20: case 0x24: case 0x28: case 0x2C:
            return *(uint32_t *)((uint8_t *)keypad.kpc.data + ((addr - 0x10) & 31));
        case 0x30: return 0; // GPIO direction?
        case 0x34: return 0; // GPIO output?
        case 0x38: return 0; // GPIO input?
        case 0x3C: return 0;
        case 0x40: return keypad.kpc.gpio_int_enable;
        case 0x44: return keypad.kpc.gpio_int_active;
        case 0x48: return 0;
    }
    return bad_read_word(addr);
}
void keypad_write(uint32_t addr, uint32_t value) {
    std::lock_guard<std::recursive_mutex> lg(keypad_mut);
    switch (addr & 0x7F) {
        case 0x00:
            keypad.kpc.control = value;
            if (keypad.kpc.control & 2) {
                if (!(keypad.kpc.control >> 2 & 0x3FFF))
                    error("keypad time between rows = 0");
                event_set(SCHED_KEYPAD, (keypad.kpc.control >> 16) + (keypad.kpc.control >> 2 & 0x3FFF));
            } else {
                event_clear(SCHED_KEYPAD);
            }
            return;
        case 0x04: keypad.kpc.size = value; return;
        case 0x08: keypad.kpc.int_active &= ~value; keypad_int_check(); return;
        case 0x0C: keypad.kpc.int_enable = value & 7; keypad_int_check(); return;

        case 0x30: return;
        case 0x34: return;
        case 0x3C: return;
        case 0x40: keypad.kpc.gpio_int_enable = value; keypad_int_check(); return;
        case 0x44: keypad.kpc.gpio_int_active &= ~value; keypad_int_check(); return;
        case 0x48: return;
    }
    bad_write_word(addr, value);
}
// Scan next row of keypad, if scanning is enabled
static void keypad_scan_event(int index) {
    std::lock_guard<std::recursive_mutex> lg(keypad_mut);
    if (keypad.kpc.current_row >= 16)
        error("too many keypad rows");

    uint16_t row = ~keypad.key_map[keypad.kpc.current_row];
    row &= ~(0x80000 >> keypad.kpc.current_row); // Emulate weird diagonal glitch
    row |= ~0u << (keypad.kpc.size >> 8 & 0xFF);  // Unused columns read as 1

    // Touchpad and CX keypads don't actually handle the On key,
    // that's done by the PMU. Its state is tracked in key_map though,
    // so mask it out.
    if(keypad.kpc.current_row == 0)
        row |= (1 << 9);

    if (emulate_cx)
        row = ~row;

    if (keypad.kpc.data[keypad.kpc.current_row] != row) {
        keypad.kpc.data[keypad.kpc.current_row] = row;
        keypad.kpc.int_active |= 2;
    }

    keypad.kpc.current_row++;
    if (keypad.kpc.current_row < (keypad.kpc.size & 0xFF)) {
        event_repeat(index, keypad.kpc.control >> 2 & 0x3FFF);
    } else {
        keypad.kpc.current_row = 0;
        keypad.kpc.int_active |= 1;
        if (keypad.kpc.control & 1) {
            event_repeat(index, (keypad.kpc.control >> 16) + (keypad.kpc.control >> 2 & 0x3FFF));
        } else {
            // If in single scan mode, go to idle mode
            keypad.kpc.control &= ~3;
        }
    }
    keypad_int_check();
}
void keypad_reset() {
    std::lock_guard<std::recursive_mutex> lg(keypad_mut);
    memset(&keypad.kpc, 0, sizeof keypad.kpc);
    /* No keys pressed: the classic keypads are active low */
    if (!emulate_cx)
        memset(keypad.kpc.data, 0xFF, sizeof keypad.kpc.data);
    keypad.touchpad_page = 0x04;
    sched.items[SCHED_KEYPAD].clock = CLOCK_APB;
    sched.items[SCHED_KEYPAD].second = -1;
    sched.items[SCHED_KEYPAD].proc = keypad_scan_event;
}

static void touchpad_captivate_write(uint8_t value) {
    switch(keypad.tpad_captivate.current_cmd) {
    default:
        gui_debug_printf("Unknown captivate write at cmd %x\n", keypad.tpad_captivate.current_cmd);
        /* fallthrough */
    case 0x0: // Next command
        keypad.tpad_captivate.current_cmd = value;
        keypad.tpad_captivate.byte_offset = 0;
        break;
    case 0x3: // Some config byte?
    case 0x0C: // No idea, maybe calibration or reset
        keypad.tpad_captivate.current_cmd = 0;
        break;
    }
}

static uint8_t touchpad_captivate_read() {
    switch(keypad.tpad_captivate.current_cmd) {
    case 0x01: // "WHY_BOTHER_ME"
    {
        uint8_t response[6] = {0};
        response[1] = (keypad.touchpad_contact << 3) // Actually whether something changed
                     | (!keypad.touchpad_contact << 2) // Not sure why.
                     | (keypad.touchpad_contact << 1)
                     | keypad.touchpad_down;
        response[2] = keypad.touchpad_x & 0xFF;
        response[3] = keypad.touchpad_x >> 8;
        response[4] = keypad.touchpad_y & 0xFF;
        response[5] = keypad.touchpad_y >> 8;
        if(keypad.tpad_captivate.byte_offset == sizeof(response) - 1)
            keypad.tpad_captivate.current_cmd = 0;

        return response[keypad.tpad_captivate.byte_offset++];
    }
    case 0x06: // status
    {
        uint8_t response[] = {0, 1, 0, 0, 0, 0}; // is configured
        if(keypad.tpad_captivate.byte_offset == sizeof(response) - 1)
            keypad.tpad_captivate.current_cmd = 0;
        return response[keypad.tpad_captivate.byte_offset++];
    }
    case 0x07: // size, maybe?
    {
        uint8_t response[] = {0, 0,
                              TOUCHPAD_X_MAX & 0xFF, TOUCHPAD_X_MAX >> 8,
                              TOUCHPAD_Y_MAX & 0xFF, TOUCHPAD_Y_MAX >> 8};
        if(keypad.tpad_captivate.byte_offset == sizeof(response) - 1)
            keypad.tpad_captivate.current_cmd = 0;
        return response[keypad.tpad_captivate.byte_offset++];
    }
    case 0x08: // firmware version
    {
        uint8_t response[] = {0, 0, 1, 0, 0, 4};
        if(keypad.tpad_captivate.byte_offset == sizeof(response) - 1)
            keypad.tpad_captivate.current_cmd = 0;
        return response[keypad.tpad_captivate.byte_offset++];
    }
    case 0x0A: // No idea
        if(keypad.tpad_captivate.byte_offset == 5)
            keypad.tpad_captivate.current_cmd = 0;
        keypad.tpad_captivate.byte_offset++;
        return 0;
    default:
        gui_debug_printf("Unknown captivate read at cmd %x\n", keypad.tpad_captivate.current_cmd);
        return 0;
    }
}

static void touchpad_write(uint8_t addr, uint8_t value) {
    if (addr == 0xFF)
        keypad.touchpad_page = value;
}
static uint8_t touchpad_read(uint8_t addr) {
    if (addr == 0xFF)
        return keypad.touchpad_page;

    if (keypad.touchpad_page == 0x02 && addr == 0x05)
        return 0x06; /* product number, as checked by the legacy Linux port */

    if (keypad.touchpad_page == 0x10) {
        switch (addr) {
            case 0x04: return TOUCHPAD_X_MAX >> 8;
            case 0x05: return TOUCHPAD_X_MAX & 0xFF;
            case 0x06: return TOUCHPAD_Y_MAX >> 8;
            case 0x07: return TOUCHPAD_Y_MAX & 0xFF;
            default: gui_debug_printf("FIXME: TPAD read 10%02x\n", addr);
        }
    } else if (keypad.touchpad_page == 0x04) {
        switch (addr) {
            case 0x00: return keypad.touchpad_down || keypad.touchpad_contact; // contact
            case 0x01: return keypad.touchpad_down ? 100 : keypad.touchpad_contact ? 0x2F : 0; // proximity
            case 0x02: return keypad.touchpad_x >> 8;
            case 0x03: return keypad.touchpad_x & 0xFF;
            case 0x04: return keypad.touchpad_y >> 8;
            case 0x05: return keypad.touchpad_y & 0xFF;
            case 0x06: // relative x
            {
                uint8_t a = keypad.touchpad_rel_x;
                keypad.touchpad_rel_x = 0;
                return a;
            }
            case 0x07: // relative y
            {
                uint8_t a = keypad.touchpad_rel_y;
                keypad.touchpad_rel_y = 0;
                return a;
            }
            case 0x08: return 0x0; // ?
            case 0x09: return 0x0; // ?
            case 0x0A: return keypad.touchpad_down; // down
            case 0x0B: // IRQ status
            {
                uint8_t ret = keypad.touchpad_irq_state;
                keypad.touchpad_irq_state = 0x53; // Reading resets IRQs, those few bits are always active
                return ret;
            }
            case 0xE4: return 1; // firmware version
            case 0xE5: return 6; // firmware version
            case 0xE6: return 0; // firmware version
            case 0xE7: return 0; // firmware version
            default: gui_debug_printf("FIXME: TPAD read 04%02x\n", addr);
        }
    }
    return 0;
}

void touchpad_gpio_reset() {
    keypad.touchpad_gpio.prev_clock = 1;
    keypad.touchpad_gpio.prev_data = 1;
    keypad.touchpad_gpio.state = 0;
    keypad.touchpad_gpio.byte = 0;
    keypad.touchpad_gpio.bitcount = 0;
    keypad.touchpad_gpio.port = 0;
}
void touchpad_gpio_change() {
    uint8_t value = gpio.input.b[0] & (gpio.output.b[0] | gpio.direction.b[0]) & 0xA;
    uint8_t clock = value >> 1 & 1;
    uint8_t data  = value >> 3 & 1;

    if (keypad.touchpad_gpio.prev_clock == 1 && clock == 1) {
        if (data < keypad.touchpad_gpio.prev_data) {
            //printf("I2C start\n");
            keypad.touchpad_gpio.bitcount = 0;
            keypad.touchpad_gpio.byte = 0xFF;
            keypad.touchpad_gpio.state = 2;
        }
        if (data > keypad.touchpad_gpio.prev_data) {
            //printf("I2C stop\n");
            keypad.touchpad_gpio.state = 0;
        }
    }

    if (clock != keypad.touchpad_gpio.prev_clock) {
        if (keypad.touchpad_gpio.state == 0) {
            // idle, do nothing
        } else if (keypad.touchpad_gpio.bitcount < 8) {
            if (!clock) {
                gpio.input.b[0] &= ~8;
                gpio.input.b[0] |= keypad.touchpad_gpio.byte >> 4 & 8;
            } else {
                // bit transferred, shift the register
                keypad.touchpad_gpio.byte = keypad.touchpad_gpio.byte << 1 | data;
                keypad.touchpad_gpio.bitcount++;
            }
        } else switch (keypad.touchpad_gpio.state | clock) {
            case 2: // C->T address
                if ((keypad.touchpad_gpio.byte >> 1) != 0x20) {
                    // Wrong address
                    keypad.touchpad_gpio.state = 0;
                    break;
                }
                gpio.input.b[0] &= ~8;
                break;
            case 3: // C->T address
                if (!(keypad.touchpad_gpio.byte & 1)) {
                    keypad.touchpad_gpio.bitcount = 0;
                    keypad.touchpad_gpio.byte = 0xFF;
                    keypad.touchpad_gpio.state = 4;
                    break;
                }
read_again:
                keypad.touchpad_gpio.bitcount = 0;
                keypad.touchpad_gpio.byte = touchpad_read(keypad.touchpad_gpio.port);
                if (keypad.touchpad_gpio.port != 0xFF)
                    keypad.touchpad_gpio.port++;
                keypad.touchpad_gpio.state = 8;
                break;
            case 4: // C->T port
                keypad.touchpad_gpio.port = keypad.touchpad_gpio.byte;
                gpio.input.b[0] &= ~8;
                break;
            case 5: // C->T port
            case 7: // C->T value
                keypad.touchpad_gpio.bitcount = 0;
                keypad.touchpad_gpio.byte = 0xFF;
                keypad.touchpad_gpio.state = 6;
                break;
            case 6: // C->T value
                touchpad_write(keypad.touchpad_gpio.port, keypad.touchpad_gpio.byte);
                if (keypad.touchpad_gpio.port != 0xFF)
                    keypad.touchpad_gpio.port++;
                gpio.input.b[0] &= ~8;
                break;
            case 8: // T->C value
                gpio.input.b[0] |= 8;
                break;
            case 9: // T->C value
                if (!data)
                    goto read_again;
                keypad.touchpad_gpio.state = 0;
                break;
        }
    }

    keypad.touchpad_gpio.prev_clock = clock;
    keypad.touchpad_gpio.prev_data  = data;
}

/* 90050000 */
#define DW_INTR_RX_UNDER  (1 << 0)
#define DW_INTR_RX_FULL   (1 << 2)
#define DW_INTR_TX_EMPTY  (1 << 4)
#define DW_INTR_TX_ABRT   (1 << 6)
#define DW_INTR_ACTIVITY  (1 << 8)
#define DW_INTR_STOP_DET  (1 << 9)
#define DW_INTR_START_DET (1 << 10)
#define DW_FIFO_DEPTH     16
#define DW_TOUCHPAD_ADDR  0x20

static uint32_t dw_raw_intr()
{
    auto &dw = keypad.touchpad_cx;
    uint32_t raw = dw.raw_intr;
    /* Level-triggered bits. Transmission is instant: the TX FIFO is empty. */
    if (dw.enable & 1)
        raw |= DW_INTR_TX_EMPTY;
    if (dw.rx_count > dw.rx_tl)
        raw |= DW_INTR_RX_FULL;
    return raw;
}

static void dw_int_check()
{
    int_set(20, dw_raw_intr() & keypad.touchpad_cx.intr_mask);
}

static void dw_end_transfer()
{
    auto &dw = keypad.touchpad_cx;
    dw.in_transfer = false;
    dw.state = 0;
    dw.raw_intr |= DW_INTR_STOP_DET;
}

static void dw_data_cmd(uint32_t value)
{
    auto &dw = keypad.touchpad_cx;
    if (!(dw.enable & 1))
        return;

    if (!dw.in_transfer || (value & (1 << 10))) {
        /* (Re)start condition */
        dw.raw_intr |= DW_INTR_START_DET;
        if (!dw.in_transfer)
            dw.state = 0;
        dw.in_transfer = true;
    }

    if ((dw.tar & 0x3FF) != DW_TOUCHPAD_ADDR) {
        /* Nobody acknowledges the address */
        dw.abort_source |= 1;
        dw.raw_intr |= DW_INTR_TX_ABRT;
        dw_end_transfer();
        return;
    }

    bool captivate = emulate_cx2 && (features & 1);
    if (value & (1 << 8)) {
        uint8_t byte = captivate ? touchpad_captivate_read() : touchpad_read(dw.port++);
        if (dw.rx_count < DW_FIFO_DEPTH)
            dw.rx_fifo[dw.rx_count++] = byte;
    } else if (captivate) {
        touchpad_captivate_write(value & 0xFF);
    } else if (dw.state == 0) {
        dw.port = value;
        dw.state = 1;
    } else {
        touchpad_write(dw.port++, value);
    }

    if (value & (1 << 9))
        dw_end_transfer();
}

void touchpad_cx_reset(void) {
    std::lock_guard<std::recursive_mutex> lg(keypad_mut);
    keypad.touchpad_cx = {};
    keypad.touchpad_cx.con = 0x7F;
    keypad.touchpad_cx.tar = 0x55;
}
uint32_t touchpad_cx_read(uint32_t addr) {
    std::lock_guard<std::recursive_mutex> lg(keypad_mut);
    auto &dw = keypad.touchpad_cx;
    uint32_t ret;
    switch (addr & 0xFFFF) {
        case 0x0000: return dw.con;
        case 0x0004: return dw.tar;
        case 0x0010:
            if (!dw.rx_count) {
                dw.raw_intr |= DW_INTR_RX_UNDER;
                dw_int_check();
                return 0;
            }
            ret = dw.rx_fifo[0];
            memmove(dw.rx_fifo, dw.rx_fifo + 1, --dw.rx_count);
            dw_int_check();
            return ret;
        case 0x0014: case 0x0018: case 0x001C: case 0x0020:
            return dw.hcnt_lcnt[((addr & 0xFF) - 0x14) >> 2];
        case 0x002C: return dw_raw_intr() & dw.intr_mask;
        case 0x0030: return dw.intr_mask;
        case 0x0034: return dw_raw_intr();
        case 0x0038: return dw.rx_tl;
        case 0x003C: return dw.tx_tl;
        case 0x0040: /* CLR_INTR: all edge-triggered bits */
            dw.raw_intr = 0;
            dw.abort_source = 0;
            dw_int_check();
            return 0;
        case 0x0044: dw.raw_intr &= ~DW_INTR_RX_UNDER; dw_int_check(); return 0;
        case 0x0048: dw.raw_intr &= ~(1 << 1); dw_int_check(); return 0;
        case 0x004C: dw.raw_intr &= ~(1 << 3); dw_int_check(); return 0;
        case 0x0050: dw.raw_intr &= ~(1 << 5); dw_int_check(); return 0;
        case 0x0054:
            dw.raw_intr &= ~DW_INTR_TX_ABRT;
            dw.abort_source = 0;
            dw_int_check();
            return 0;
        case 0x0058: dw.raw_intr &= ~(1 << 7); dw_int_check(); return 0;
        case 0x005C: dw.raw_intr &= ~DW_INTR_ACTIVITY; dw_int_check(); return 0;
        case 0x0060: dw.raw_intr &= ~DW_INTR_STOP_DET; dw_int_check(); return 0;
        case 0x0064: dw.raw_intr &= ~DW_INTR_START_DET; dw_int_check(); return 0;
        case 0x0068: dw.raw_intr &= ~(1 << 11); dw_int_check(); return 0;
        case 0x006C: return dw.enable;
        case 0x0070: /* STATUS: TFNF | TFE | RFNE */
            return 0x6 | (dw.rx_count ? 0x8 : 0);
        case 0x0074: return 0;
        case 0x0078: return dw.rx_count;
        case 0x007C: return dw.sda_hold;
        case 0x0080: return dw.abort_source;
        case 0x0094: return dw.sda_setup;
        case 0x009C: return dw.enable & 1;
        case 0x00A0: return dw.fs_spklen;
        case 0x00F4: /* COMP_PARAM_1: 16 deep FIFOs, fast mode, 32 bit APB */
            return ((DW_FIFO_DEPTH - 1) << 16) | ((DW_FIFO_DEPTH - 1) << 8) | (2 << 2) | 2;
        case 0x00F8: return 0x3131312A; /* 1.11a */
        case 0x00FC: return 0x44570140;
        default:
            return 0;
    }
}
void touchpad_cx_write(uint32_t addr, uint32_t value) {
    std::lock_guard<std::recursive_mutex> lg(keypad_mut);
    auto &dw = keypad.touchpad_cx;
    switch (addr & 0xFFFF) {
        case 0x0000: dw.con = value; return;
        case 0x0004: dw.tar = value; return;
        case 0x0010: dw_data_cmd(value); break;
        case 0x0014: case 0x0018: case 0x001C: case 0x0020:
            dw.hcnt_lcnt[((addr & 0xFF) - 0x14) >> 2] = value;
            return;
        case 0x0030: dw.intr_mask = value & 0xFFF; break;
        case 0x0038:
            dw.rx_tl = value & 0xFF;
            /* The TI OS writes this before every transfer */
            dw.state = 0;
            break;
        case 0x003C: dw.tx_tl = value & 0xFF; break;
        case 0x006C:
            dw.enable = value & 1;
            if (!dw.enable) {
                dw.rx_count = 0;
                dw.in_transfer = false;
                dw.state = 0;
            }
            break;
        case 0x007C: dw.sda_hold = value; return;
        case 0x0094: dw.sda_setup = value; return;
        case 0x00A0: dw.fs_spklen = value; return;
        default:
            return;
    }
    dw_int_check();
}

bool keypad_suspend(emu_snapshot *snapshot)
{
    return snapshot_write(snapshot, &keypad, sizeof(keypad));
}

bool keypad_resume(const emu_snapshot *snapshot)
{
    return snapshot_read(snapshot, &keypad, sizeof(keypad));
}

void keypad_set_key(int row, int col, bool state)
{
    std::lock_guard<std::recursive_mutex> lg(keypad_mut);

    assert(row < KEYPAD_ROWS);
    assert(col < KEYPAD_COLS);

    if(state)
        keypad.key_map[row] |= 1 << col;
    else
        keypad.key_map[row] &= ~(1 << col);

    if(state && row == 0 && col == 9)
        keypad_on_pressed();
}

void touchpad_set_state(float x, float y, bool contact, bool down)
{
    std::lock_guard<std::recursive_mutex> lg(keypad_mut);
    if(contact || down)
    {
        int new_x = x * TOUCHPAD_X_MAX,
            new_y = TOUCHPAD_Y_MAX - (y * TOUCHPAD_Y_MAX);

        if(new_x < 0)
            new_x = 0;
        if(new_x > TOUCHPAD_X_MAX)
            new_x = TOUCHPAD_X_MAX;

        if(new_y < 0)
            new_y = 0;
        if(new_y > TOUCHPAD_Y_MAX)
            new_y = TOUCHPAD_Y_MAX;

        /* On a move, update the rel registers */
        if(keypad.touchpad_contact)
        {
            int vel_x = new_x - keypad.touchpad_x;
            int vel_y = new_y - keypad.touchpad_y;

            /* The OS's cursor uses this, but it's a bit too quick */
            vel_x /= 4;
            vel_y /= 4;

            keypad.touchpad_rel_x += vel_x;
            keypad.touchpad_rel_y += vel_y;
        }
        else
        {
            keypad.touchpad_rel_x = 0;
            keypad.touchpad_rel_y = 0;
        }

        keypad.touchpad_x = new_x;
        keypad.touchpad_y = new_y;
    }

    keypad.touchpad_down = down;
    keypad.touchpad_contact = contact;

    keypad.kpc.gpio_int_active |= 0x800;
    keypad_int_check();
}
