#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include <deque>
#include <string>
#include <vector>

#include "core/debug.h"
#include "core/emu.h"
#include "core/flash.h"
#include "core/keypad.h"
#include "core/lcd.h"
#include "core/mem.h"
#include "core/misc.h"
#include "core/mmu.h"
#include "core/usblink.h"
#include "core/usblink_queue.h"

/*
 * Script mode: lines read from stdin are either sent to the emulated
 * serial port, or, when starting with '!', are emulator commands:
 *
 *   !wait TEXT        stop processing the script until TEXT shows up on
 *                     the serial output
 *   !delay MS         stop processing the script for MS virtual milliseconds
 *   !key ROW COL 0|1  release/press a key of the keypad matrix
 *   !on 0|1           release/press the ON key
 *   !touch X Y C D    set the touchpad state (X/Y in 0..1, contact, down)
 *   !screenshot FILE  save the screen as a PPM image
 *   !usbconnect       connect the emulated USB link (usblink)
 *   !usbls PATH       list a directory of the OS's filesystem
 *   !usbget PATH FILE copy a file of the OS's filesystem to FILE
 *   !usbput FILE PATH copy FILE to the OS's filesystem
 *   !usbdel PATH      delete a file of the OS's filesystem
 *                     (the script waits until each is done; results go
 *                     to stderr, as "[usb] ...")
 *   !dumpmem HEXADDR HEXSIZE FILE  save physical memory to FILE
 *   !regs             print pc, lr, sp and cpsr to stderr
 *   !quit [CODE]      exit
 */

static std::deque<std::string> script;
static std::deque<char> serial_tx;      /* pending bytes to the guest */
static std::string wait_for, out_tail;
static int delay_ticks;
static int timeout_ticks = -1;
static bool echo_script;
static bool realtime;
static bool save_flash;
static bool usb_wait;

static void usb_progress(int progress, void *what)
{
    if (progress < 0 || progress == 100)
        fprintf(stderr, "[usb] %s: %s\n", (const char *)what, progress < 0 ? "error" : "done");
}

static void usb_dirent(struct usblink_file *f, bool is_error, void *)
{
    if (f)
        fprintf(stderr, "[usb] %s %u%s\n", f->filename, f->size, f->is_dir ? " dir" : "");
    else
        fprintf(stderr, "[usb] list %s\n", is_error ? "error" : "end");
}

static void finish(int code)
{
    fflush(stdout);
    if (save_flash)
        flash_save_changes();
    _exit(code);
}

static void script_step();

void gui_do_stuff(bool wait)
{
    (void)wait;

    /* Called every virtual 10ms */
    if (timeout_ticks >= 0 && --timeout_ticks < 0) {
        fprintf(stderr, "\n[firebird] timeout\n");
        finish(124);
    }

    if (delay_ticks > 0)
        --delay_ticks;

    script_step();
}

void do_stuff(int i)
{
    (void)i;
}

void gui_debug_printf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);

    gui_debug_vprintf(fmt, ap);

    va_end(ap);
}

void gui_debug_vprintf(const char *fmt, va_list ap)
{
    vfprintf(stderr, fmt, ap);
}

void gui_status_printf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);

    gui_debug_vprintf(fmt, ap);

    fputc('\n', stderr);

    va_end(ap);
}

void gui_perror(const char *msg)
{
    gui_debug_printf("%s: %s\n", msg, strerror(errno));
}

void gui_debugger_entered_or_left(bool entered) { (void)entered; }

void gui_debugger_request_input(debug_input_cb callback)
{
    if(!callback) return;
    static char debug_in[40];
    if (!fgets(debug_in, sizeof(debug_in), stdin))
        exit(1);
    callback(debug_in);
}

void gui_putchar(char c)
{
    putc(c, stdout);

    /* Output since the last line was sent, for !wait */
    out_tail.push_back(c);
    if (out_tail.size() > 65536)
        out_tail.erase(0, out_tail.size() - 65536);
    if (!wait_for.empty() && out_tail.find(wait_for) != std::string::npos) {
        wait_for.clear();
        out_tail.clear();
    }
}

int gui_getchar() { return -1; }
void gui_set_busy(bool busy) { (void)busy; }
void gui_show_speed(double d) { (void)d; }
void gui_usblink_changed(bool state) { (void)state; }
void throttle_timer_off() {}
void throttle_timer_on() {}
/* Only used with --realtime: keep virtual time in step with the host clock */
void throttle_timer_wait(unsigned int usec) { usleep(usec); }

/* The screen as the LCD shows it: RGB565 on the CX, 4 bit levels of
 * darkness (0 is white) on the classic models */
static void save_screenshot(const char *path)
{
    static uint16_t frame[320 * 240];
    lcd_cx_draw_frame(frame);

    FILE *f = fopen(path, "wb");
    if (!f) {
        perror(path);
        return;
    }
    fprintf(f, "P6\n320 240\n255\n");
    for (unsigned int i = 0; i < 320 * 240; i++) {
        uint16_t px = frame[i];
        uint8_t rgb[3];
        if (emulate_cx) {
            rgb[0] = (px >> 11 & 0x1F) << 3;
            rgb[1] = (px >> 5 & 0x3F) << 2;
            rgb[2] = (px & 0x1F) << 3;
        } else {
            rgb[0] = rgb[1] = rgb[2] = (15 - (px & 0xF)) * 17;
        }
        fwrite(rgb, 3, 1, f);
    }
    fclose(f);
}

static void script_command(const std::string &line)
{
    char cmd[16] = "";
    sscanf(line.c_str() + 1, "%15s", cmd);
    const char *arg = line.c_str() + 1 + strlen(cmd);
    while (*arg == ' ')
        arg++;

    if (echo_script)
        fprintf(stderr, "[firebird] %s\n", line.c_str());

    if (!strcmp(cmd, "wait")) {
        if (out_tail.find(arg) != std::string::npos)
            out_tail.clear();
        else
            wait_for = arg;
    } else if (!strcmp(cmd, "delay")) {
        delay_ticks = (atoi(arg) + 9) / 10;
    } else if (!strcmp(cmd, "key")) {
        int row, col, state;
        if (sscanf(arg, "%d %d %d", &row, &col, &state) == 3)
            keypad_set_key(row, col, state);
    } else if (!strcmp(cmd, "on")) {
        keypad_set_key(0, 9, atoi(arg));
    } else if (!strcmp(cmd, "touch")) {
        float x, y;
        int contact, down;
        if (sscanf(arg, "%f %f %d %d", &x, &y, &contact, &down) == 4)
            touchpad_set_state(x, y, contact, down);
    } else if (!strcmp(cmd, "screenshot")) {
        save_screenshot(arg);
    } else if (!strcmp(cmd, "dumpmem")) {
        unsigned long addr, size; char file[512];
        if (sscanf(arg, "%lx %lx %511s", &addr, &size, file) == 3) {
            FILE *f = fopen(file, "wb");
            for (unsigned long a = addr; f && a < addr + size; a += 4) {
                uint32_t *p = (uint32_t *)phys_mem_ptr(a, 4);
                uint32_t v = p ? *p : 0;
                fwrite(&v, 4, 1, f);
            }
            if (f) fclose(f);
        }
    } else if (!strcmp(cmd, "usbconnect")) {
        usblink_connect();
    } else if (!strcmp(cmd, "usbls")) {
        usblink_queue_dirlist(arg, usb_dirent, nullptr);
        usb_wait = true;
    } else if (!strcmp(cmd, "usbget")) {
        char remote[512], local[512];
        if (sscanf(arg, "%511s %511s", remote, local) == 2)
            usblink_queue_download(remote, local, usb_progress, (void *)"get");
        usb_wait = true;
    } else if (!strcmp(cmd, "usbput")) {
        char local[512], remote[512];
        if (sscanf(arg, "%511s %511s", local, remote) == 2)
            usblink_queue_put_file(local, remote, usb_progress, (void *)"put");
        usb_wait = true;
    } else if (!strcmp(cmd, "usbdel")) {
        usblink_queue_delete(arg, false, usb_progress, (void *)"delete");
        usb_wait = true;
    } else if (!strcmp(cmd, "regs")) {
        fprintf(stderr, "[firebird] pc=%08x lr=%08x sp=%08x cpsr=%08x\n", arm.reg[15], arm.reg[14], arm.reg[13], get_cpsr());
    } else if (!strcmp(cmd, "quit")) {
        finish(atoi(arg));
    } else {
        fprintf(stderr, "[firebird] unknown command '%s'\n", line.c_str());
    }
}

static void script_step()
{
    /* Feed the serial port one byte per tick once the guest took the last */
    if (!serial_tx.empty()) {
        if (serial_rx_empty()) {
            serial_byte_in(serial_tx.front());
            serial_tx.pop_front();
        }
        return;
    }

    if (usb_wait) {
        if (usblink_queue_size())
            return;
        usb_wait = false;
    }

    while (wait_for.empty() && delay_ticks == 0 && serial_tx.empty() && !script.empty()) {
        std::string line = script.front();
        script.pop_front();

        if (!line.empty() && line[0] == '!') {
            script_command(line);
        } else {
            out_tail.clear();
            for (char c : line)
                serial_tx.push_back(c);
            serial_tx.push_back('\r');
        }
    }
}

static bool load_file(const char *path, uint32_t addr, size_t *size_out)
{
    FILE *f = fopen(path, "rb");
    if (!f) {
        perror(path);
        return false;
    }

    fseek(f, 0, SEEK_END);
    size_t size = ftell(f);
    rewind(f);

    void *target = phys_mem_ptr(addr, size);
    if (!target) {
        fprintf(stderr, "%s does not fit in RAM at 0x%08x\n", path, addr);
        fclose(f);
        return false;
    }

    bool ok = fread(target, size, 1, f) == 1;
    fclose(f);
    if (!ok)
        perror(path);
    else if (size_out)
        *size_out = size;

    return ok;
}

static bool create_flash(const char *path, const char *model)
{
    unsigned int product, features = 0;
    bool is_cx;

    if (!strcmp(model, "cx")) {
        product = 0x100;
        features = 0x005; /* HW-A */
        is_cx = true;
    } else if (!strcmp(model, "tp") || !strcmp(model, "clp")) {
        product = 0x0E0;
        is_cx = false;
    } else {
        fprintf(stderr, "Unknown model '%s' (cx, tp, clp)\n", model);
        return false;
    }

    const char *preload[4] = { nullptr, nullptr, nullptr, nullptr };
    uint8_t *nand_data = nullptr;
    size_t nand_size;

    if (!flash_create_new(is_cx, preload, product, features, is_cx, &nand_data, &nand_size)) {
        free(nand_data);
        return false;
    }

    FILE *f = fopen(path, "wb");
    bool ok = f && fwrite(nand_data, nand_size, 1, f) == 1;
    if (f)
        fclose(f);
    free(nand_data);

    if (!ok)
        perror(path);

    return ok;
}

static const char OPT_BOOT1[]              = "--boot1";
static const char OPT_FLASH[]              = "--flash";
static const char OPT_MODEL[]              = "--model";
static const char OPT_SNAPSHOT[]           = "--snapshot";
static const char OPT_RAMPAYLOAD[]         = "--rampayload";
static const char OPT_RAMPAYLOAD_ADDR[]    = "--rampayload-address";
static const char OPT_KERNEL[]             = "--kernel";
static const char OPT_DTB[]                = "--dtb";
static const char OPT_TIMEOUT[]            = "--timeout";
static const char OPT_ECHO[]               = "--echo-script";
static const char OPT_REALTIME[]           = "--realtime";
static const char OPT_LOAD[]               = "--load";
static const char OPT_SAVE_FLASH[]         = "--save-flash";
static const char OPT_DEBUG_ON_START[]     = "--debug-on-start";
static const char OPT_DEBUG_ON_WARN[]      = "--debug-on-warn";
static const char OPT_PRINT_ON_WARN[]      = "--print-on-warn";
static const char OPT_DIAGS[]              = "--diags";
static const char OPT_HELP[]               = "--help";
static const uint32_t default_rampayload_base = 0x10000000;

/*
 * Mirror what a Linux loader running on the calculator does: the kernel
 * image goes into SDRAM, away from where it decompresses itself to, and the
 * device tree after it.
 */
static const uint32_t kernel_base = 0x10800000;
static const uint32_t dtb_base = 0x11000000;

void show_help_menu(void)
{
	fprintf(stderr, "firebird-headless:\n");
	fprintf(stderr, "  %-24s Show this help menu\n", OPT_HELP);
	fprintf(stderr, "  %-24s Path to Boot1 image (optional with %s)\n", OPT_BOOT1, OPT_KERNEL);
	fprintf(stderr, "  %-24s Path to Flash image (required)\n", OPT_FLASH);
	fprintf(stderr, "  %-24s Create the flash image first if it does not exist: cx, tp, clp\n", OPT_MODEL);
	fprintf(stderr, "  %-24s Path to snapshot image (optional)\n", OPT_SNAPSHOT);
	fprintf(stderr, "  %-24s Path to RAM payload (optional)\n", OPT_RAMPAYLOAD);
	fprintf(stderr, "  %-24s Address to load RAM payload at (default: 0x%x)\n",
	       OPT_RAMPAYLOAD_ADDR, default_rampayload_base);
	fprintf(stderr, "  %-24s Boot this Linux zImage directly, without Boot1/Boot2/OS\n", OPT_KERNEL);
	fprintf(stderr, "  %-24s Device tree blob passed to the kernel\n", OPT_DTB);
	fprintf(stderr, "  %-24s Exit with code 124 after this many virtual seconds\n", OPT_TIMEOUT);
	fprintf(stderr, "  %-24s Print script commands to stderr\n", OPT_ECHO);
	fprintf(stderr, "  %-24s Run at real speed (the RTC follows the host clock)\n", OPT_REALTIME);
	fprintf(stderr, "  %-24s ADDR=FILE: also load FILE into RAM at ADDR (e.g. an initrd)\n", OPT_LOAD);
	fprintf(stderr, "  %-24s Write the changes to the flash image back on exit\n", OPT_SAVE_FLASH);
	fprintf(stderr, "  %-24s Enter debugger on start\n", OPT_DEBUG_ON_START);
	fprintf(stderr, "  %-24s Enter debugger on warnings\n", OPT_DEBUG_ON_WARN);
	fprintf(stderr, "  %-24s Print warnings to console\n", OPT_PRINT_ON_WARN);
	fprintf(stderr, "  %-24s Use diagnostics boot order\n", OPT_DIAGS);
	fprintf(stderr, "Lines on stdin go to the serial port; see main.cpp for '!' commands.\n");
}

int main(int argc, char *argv[])
{
	const char *boot1 = nullptr, *flash = nullptr, *snapshot = nullptr, *rampayload = nullptr;
	const char *model = nullptr, *kernel = nullptr, *dtb = nullptr;
	uint32_t rampayload_base = default_rampayload_base;
	std::vector<std::pair<uint32_t, std::string>> extra_loads;

	setvbuf(stdout, nullptr, _IONBF, 0);

	for(int argi = 1; argi < argc; ++argi)
	{
		if(argi + 1 >= argc && argv[argi][0] == '-' && strcmp(argv[argi], OPT_HELP)
		   && strcmp(argv[argi], OPT_DEBUG_ON_START) && strcmp(argv[argi], OPT_DEBUG_ON_WARN)
		   && strcmp(argv[argi], OPT_PRINT_ON_WARN) && strcmp(argv[argi], OPT_DIAGS)
		   && strcmp(argv[argi], OPT_ECHO) && strcmp(argv[argi], OPT_REALTIME)
		   && strcmp(argv[argi], OPT_SAVE_FLASH))
		{
			fprintf(stderr, "Missing value for '%s'.\n", argv[argi]);
			return 1;
		}

		if(strcmp(argv[argi], OPT_BOOT1) == 0)
			boot1 = argv[++argi];
		else if(strcmp(argv[argi], OPT_FLASH) == 0)
			flash = argv[++argi];
		else if(strcmp(argv[argi], OPT_MODEL) == 0)
			model = argv[++argi];
		else if(strcmp(argv[argi], OPT_SNAPSHOT) == 0)
			snapshot = argv[++argi];
		else if(strcmp(argv[argi], OPT_RAMPAYLOAD) == 0)
			rampayload = argv[++argi];
		else if(strcmp(argv[argi], OPT_RAMPAYLOAD_ADDR) == 0)
			rampayload_base = strtol(argv[++argi], nullptr, 0);
		else if(strcmp(argv[argi], OPT_KERNEL) == 0)
			kernel = argv[++argi];
		else if(strcmp(argv[argi], OPT_DTB) == 0)
			dtb = argv[++argi];
		else if(strcmp(argv[argi], OPT_TIMEOUT) == 0)
			timeout_ticks = atoi(argv[++argi]) * 100;
		else if(strcmp(argv[argi], OPT_ECHO) == 0)
			echo_script = true;
		else if(strcmp(argv[argi], OPT_REALTIME) == 0)
			realtime = true;
		else if(strcmp(argv[argi], OPT_SAVE_FLASH) == 0)
			save_flash = true;
		else if(strcmp(argv[argi], OPT_LOAD) == 0)
		{
			const char *spec = argv[++argi], *eq = strchr(spec, '=');
			if (!eq) {
				fprintf(stderr, "%s expects ADDR=FILE.\n", OPT_LOAD);
				return 1;
			}
			extra_loads.emplace_back(strtoul(spec, nullptr, 0), std::string(eq + 1));
		}
		else if(strcmp(argv[argi], OPT_DEBUG_ON_START) == 0)
			debug_on_start = true;
		else if(strcmp(argv[argi], OPT_DEBUG_ON_WARN) == 0)
			debug_on_warn = true;
		else if(strcmp(argv[argi], OPT_PRINT_ON_WARN) == 0)
			print_on_warn = true;
		else if(strcmp(argv[argi], OPT_DIAGS) == 0)
			boot_order = ORDER_DIAGS;
		else if (strcmp(argv[argi], OPT_HELP) == 0)
		{
			show_help_menu();
			return 0;
		}
		else
		{
			fprintf(stderr, "Unknown argument '%s'.\n", argv[argi]);
			show_help_menu();
			return 1;
		}
	}

	if (!flash || (!boot1 && !kernel))
	{
		fprintf(stderr, "You need to specify a Flash image and either Boot1 or a kernel.\n");
		show_help_menu();
		return 2;
	}

	if (kernel && !dtb)
	{
		fprintf(stderr, "%s needs %s.\n", OPT_KERNEL, OPT_DTB);
		return 2;
	}

	struct stat st;
	if (model && stat(flash, &st) != 0 && !create_flash(flash, model))
		return 1;

	/* Read the script, unless stdin is a terminal */
	if (!isatty(STDIN_FILENO)) {
		char line[1024];
		while (fgets(line, sizeof(line), stdin)) {
			line[strcspn(line, "\r\n")] = '\0';
			script.push_back(line);
		}
	}

	path_boot1 = boot1 ? boot1 : "";
	path_flash = flash;

	if(!emu_start(0, 0, snapshot))
		return 1;

	if(rampayload)
	{
		if (!load_file(rampayload, rampayload_base, nullptr))
			return 3;

		// Jump to payload
		arm.reg[15] = rampayload_base;
	}

	for (auto &load : extra_loads)
		if (!load_file(load.second.c_str(), load.first, nullptr))
			return 3;

	if(kernel)
	{
		if (!load_file(kernel, kernel_base, nullptr) || !load_file(dtb, dtb_base, nullptr))
			return 3;

		/*
		 * The OS has normally set the clocks before a loader runs. Use the
		 * default from the manufacturing data (CX: 132/66 MHz), applied
		 * through the PMU so the emulated clock rates follow, and drop
		 * the resulting "clock change done" interrupt request.
		 */
		if (emulate_cx) {
			pmu_write(0x900B0000, 0x561002);
			pmu_write(0x900B000C, 4);
			pmu_write(0x900B0014, 3);
		}

		/* ARM Linux boot protocol: r0 = 0, r1 = machine (unused with DT), r2 = DTB */
		arm.reg[0] = 0;
		arm.reg[1] = ~0u;
		arm.reg[2] = dtb_base;
		arm.reg[15] = kernel_base;
	}

	turbo_mode = !realtime;
	emu_loop(false);

	return 0;
}
