// zzzzz
/*
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * The software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */
#include "ch.h"
#include "hal.h"

#include "usbcfg.h"
#include "nanovna.h"

#include <chprintf.h>
#include <string.h>
#include <math.h>
#include "asp_func.h"

freq_t frequencyStart;
freq_t frequencyStop;
int32_t frequencyExtra;

/*
 *  Shell settings
 */
// If need run shell as thread (use more amount of memory fore stack), after
// enable this need reduce spi_buffer size, by default shell run in main thread
// #define VNA_SHELL_THREAD

static BaseSequentialStream *shell_stream;
threads_queue_t shell_thread;

// Shell new line
#define VNA_SHELL_NEWLINE_STR    "\r\n"
// Shell command promt
#define VNA_SHELL_PROMPT_STR     "ch> "
// Shell max arguments
#define VNA_SHELL_MAX_ARGUMENTS   4
// Shell max command line size
#define VNA_SHELL_MAX_LENGTH     48

// Shell command functions prototypes
typedef void (*vna_shellcmd_t)(int argc, char *argv[]);
#define VNA_SHELL_FUNCTION(command_name) \
      static void command_name(int argc, char *argv[])

// Shell command line buffer, args, nargs, and function ptr
static char shell_line[VNA_SHELL_MAX_LENGTH];
static char *shell_args[VNA_SHELL_MAX_ARGUMENTS + 1];
static uint16_t shell_nargs;
static volatile vna_shellcmd_t  shell_function = 0;

//#define ENABLED_DUMP
// Allow get threads debug info
#define ENABLE_THREADS_COMMAND
// Enable vbat_offset command, allow change battery voltage correction in config
#define ENABLE_VBAT_OFFSET_COMMAND
// Info about NanoVNA, need fore soft
#define ENABLE_INFO_COMMAND
// Enable color command, allow change config color for traces, grid, menu
#define ENABLE_COLOR_COMMAND
#ifdef __USE_SERIAL_CONSOLE__
#define ENABLE_USART_COMMAND
#endif

#ifdef __USE_SD_CARD__
#ifdef __DISABLE_HOT_INSERT__
uint16_t sd_card_inserted_at_boot = false;
#endif
// Enable SD card console command
#define ENABLE_SD_CARD_CMD
#endif

void update_frequencies(void);
static void set_frequencies(freq_t start, freq_t stop, uint16_t points);
static bool sweep(bool break_on_operation);
static long_t my_atoi(const char *p);

uint8_t sweep_mode = SWEEP_ENABLE;
uint16_t sweep_once_count = 1;
uint16_t redraw_request = 0; // contains REDRAW_XXX flags
// Version text, displayed in Config->Version menu, also send by info command
const char *info_about[]={
  BOARD_NAME,
  "2019-2022 Copyright @Erik Kaashoek",
  "2016-2020 Copyright @edy555",
  "SW licensed under GPL. See: https://github.com/erikkaashoek/tinySA",
  "Version: " VERSION,
  "Build Time: " __DATE__ " - " __TIME__,
  "Kernel: " CH_KERNEL_VERSION,
  "Compiler: " PORT_COMPILER_NAME,
  "Architecture: " PORT_ARCHITECTURE_NAME " Core Variant: " PORT_CORE_VARIANT_NAME,
  "Port Info: " PORT_INFO,
  "Platform: " PLATFORM_NAME,
  0 // sentinel
};

bool dirty = true;
int32_t scan_after_dirty = 0;
uint8_t completed = false;
uint8_t enable_after_complete = 0;

void clear_backup(void) {
  uint32_t *f = &backup;     // Clear backup when no valid config data
  int i = USED_BACKUP_SIZE;
  while (i--)
    *f++ = 0;
}


#ifdef TINYSA4
static THD_WORKING_AREA(waThread1, 1224);
#else
static THD_WORKING_AREA(waThread1, 768);
bool has_esd = false;
#endif
static THD_FUNCTION(Thread1, arg)
{
  (void)arg;
  chRegSetThreadName("sweep");
  // Init UI and plot grid
  area_height = AREA_HEIGHT_NORMAL;
  ui_init();
  //Initialize graph plotting
  plot_init();

#ifdef __SD_CARD_LOAD__
  sd_card_load_config("autoload.ini");
#endif

//#ifndef TINYSA4
//  ui_process();
//#endif
#if 1    // zzzzz
  static const freq_t g_freq_mhz_list[] = 
  {
    900, 910,
    1200, 1210,
    2400, 2410, 2420, 2430, 2440, 2450, 2460, 2470, 
    5200, 5220, 
    5760, 5780, 5800, 5820
  }; 
#define countof(x) sizeof(x)/sizeof(x[0])
  int i_freq = 0;
  freq_t freq_min = 2380000000UL;       // To use 2390 MHz video source near my location, use 2400 for WiFi
  freq_t freq_max = 2500000000UL;

  freq_min = 2400000000UL;       // To use 2390 MHz video source near my location, use 2400 for WiFi
  freq_max = 2490000000UL;

  freq_t f0 = freq_min;
  f0 = g_freq_mhz_list[i_freq]*1000000ULL;
  set_frequencies(f0,f0,sweep_points);
  sweep_mode |= SWEEP_ENABLE;
  sweep_mode |= SWEEP_UI_MODE;
  graph_bottom = 0;     // for full-screen waterfall 
  graph_bottom = 100;
  ili9341_set_background(LCD_BG_COLOR);
  ili9341_clear_screen();
  set_sweep_points(POINTS_COUNT);    //
#endif

  while (1) {
//  START_PROFILE
#if 1    // zzzzz
     set_sweep_frequency(ST_CW, f0);
#endif
    if (sweep_mode&(SWEEP_ENABLE|SWEEP_ONCE)) {
      backup_t b;
      b.frequency0 = setting.frequency0;
      b.frequency1 = setting.frequency1;
      if (setting.auto_attenuation)
        b.attenuation = 0;
      else
        b.attenuation = setting.attenuate_x2+1;
      if (setting.auto_reflevel || setting.unit != U_DBM)
        b.reflevel = 0;
      else
        b.reflevel = setting.reflevel + 140;
      if (setting.rbw_x10 == 0)
        b.RBW = 0;
      else
#ifdef TINYSA4
        b.RBW = SI4463_rbw_selected+1;
#else
        b.RBW = SI4432_rbw_selected+1;
#endif
      b.mode = setting.mode;
      uint32_t *f = (uint32_t *)&b;
      uint32_t *t = &backup;
      int i = USED_BACKUP_SIZE;
      while (i--)
        *t++ = *f++;

#ifdef __LISTEN__
      if (setting.listen && markers[active_marker].enabled == M_ENABLED) {
        perform(false, 0, getFrequency(markers[active_marker].index), false);
        SI4432_Listen(MODE_SELECT(setting.mode));
      } else
#endif
      {
        completed = sweep(true);
        if (sweep_once_count>1) {
          sweep_once_count--;
        } else
          sweep_mode&=~SWEEP_ONCE;


      }
    } else if (sweep_mode & SWEEP_SELFTEST) {
      // call from lowest level to save stack space
      self_test(setting.test);
      completed = true;
//      sweep_mode = SWEEP_ENABLE;
#ifdef __SINGLE_LETTER__
      } else if (sweep_mode & SWEEP_REMOTE) {
      sweep_remote();
#endif
#ifdef __CALIBRATE__
      } else if (sweep_mode & SWEEP_CALIBRATE) {
      // call from lowest level to save stack space
        calibrate();
        sweep_mode = SWEEP_ENABLE;
#endif
#ifdef TINYSA4
      } else if (sweep_mode & SWEEP_CALIBRATE_HARMONIC) {
      // call from lowest level to save stack space
        calibrate_harmonic();
        sweep_mode = SWEEP_ENABLE;
#endif
      } else {
//      if (setting.mode != -1)
        __WFI();
    }
//  STOP_PROFILE
#if 1
    // Run Shell command in sweep thread
    if (shell_function) {
      operation_requested = OP_NONE; // otherwise commands  will be aborted
      do {
        shell_function(shell_nargs - 1, &shell_args[1]);
        shell_function = 0;
        // Resume shell thread
        osalThreadDequeueNextI(&shell_thread, MSG_OK);
      } while (shell_function);
      if (dirty) {
        if (MODE_OUTPUT(setting.mode))
          draw_menu();    // update screen if in output mode and dirty
        else
          redraw_request |= REDRAW_CAL_STATUS | REDRAW_AREA | REDRAW_FREQUENCY;
      }
    }
//    START_PROFILE
    // Process UI inputs
    if (!(sweep_mode & SWEEP_SELFTEST)) {
      sweep_mode|= SWEEP_UI_MODE;
      ui_process();
      sweep_mode&=~SWEEP_UI_MODE;
    }
    // Process collected data, calculate trace coordinates and plot only if scan
    // completed
    if (completed) {
      // Enable traces at sweep complete for redraw
      if (enable_after_complete){
        TRACE_ENABLE(enable_after_complete);
        enable_after_complete = 0;
      }
//      START_PROFILE;
      // Prepare draw graphics, cache all lines, mark screen cells for redraw
      plot_into_index(measured);
      redraw_request |= REDRAW_CELLS | REDRAW_BATTERY;
//      STOP_PROFILE;
      if (uistat.marker_tracking) {
        int i = marker_search_max(active_marker);
        if (i != -1 && active_marker != MARKER_INVALID) {
          set_marker_index(active_marker, i);
          redraw_request |= REDRAW_MARKER;
        }
      }
    }
#endif
#if 0

    // plot trace and other indications as raster
    draw_all(completed);  // flush markmap only if scan completed to prevent
#else
// zzzzz
    asp_save_file();
    asp_update_band(f0/1000000UL);
    if (++i_freq >= countof(g_freq_mhz_list))
//    f0 +=               5000000UL;
//    if (f0 >= freq_max)
    {
  //      f0 = freq_min;
        i_freq = 0;
        ili9341_set_foreground(LCD_BRIGHT_COLOR_GREEN);
        if (completed) 
        {
    //    update_level_meter();
    //    draw_cal_status();                      // calculates the actual sweep time, must be before draw_frequencies
    //    draw_frequencies();
            draw_battery_status();
            asp_update_waterfall();
            //ili9341_drawstring_7x13("Scanning... ", 50, 10);
            //ili9341_drawstring("zzzzzzzzzzz", 50, LCD_HEIGHT - FONT_GET_HEIGHT);
        }
    }
    f0 = g_freq_mhz_list[i_freq]*1000000ULL;
#endif

                          // remaining traces
//    STOP_PROFILE
  }

}

#pragma GCC push_options
#pragma GCC optimize ("Os")

void enableTracesAtComplete(uint8_t mask){
  // Disable this traces
  TRACE_DISABLE(mask);
  enable_after_complete|=mask;
  redraw_request|=REDRAW_AREA;
}

int
is_paused(void)
{
  return !(sweep_mode & (SWEEP_ENABLE|SWEEP_ONCE));
}

static inline void
pause_sweep(void)
{
  sweep_mode &= ~SWEEP_ENABLE;
}

static inline void
resume_sweep(void)
{
  sweep_mode |= SWEEP_ENABLE;
}

void
resume_once(uint16_t c)
{
  sweep_once_count = c;
  sweep_mode |= SWEEP_ONCE;
}

void
toggle_sweep(void)
{
  sweep_mode ^= SWEEP_ENABLE;
}

// Shell commands output
int shell_printf(const char *fmt, ...)
{
  if (shell_stream == NULL) return 0;
  va_list ap;
  int formatted_bytes = 0;
  va_start(ap, fmt);
  formatted_bytes = chvprintf(shell_stream, fmt, ap);
  va_end(ap);
  return formatted_bytes;
}

// Shell commands output
int usage_printf(const char *fmt, ...)
{
  if (shell_stream == NULL) return 0;
  va_list ap;
  int formatted_bytes = 0;
  va_start(ap, fmt);
  shell_printf("usage: ");
  formatted_bytes += chvprintf(shell_stream, fmt, ap);
  va_end(ap);
  return formatted_bytes;
}

#ifdef __USE_SERIAL_CONSOLE__
// Serial Shell commands output
int shell_serial_printf(const char *fmt, ...)
{
  va_list ap;
  int formatted_bytes;
  va_start(ap, fmt);
  formatted_bytes = chvprintf((BaseSequentialStream *)&SD1, fmt, ap);
  va_end(ap);
  return formatted_bytes;
}
#endif

//
// Function used for search substring v in list
// Example need search parameter "center" in "start|stop|center|span|cw" getStringIndex return 2
// If not found return -1
// Used for easy parse command arguments
static int get_str_index(const char *v, const char *list)
{
  int i = 0;
  while (1) {
    const char *p = v;
    while (1) {
      char c = *list;
      if (c == '|') c = 0;
      if (c == *p++) {
        // Found, return index
        if (c == 0) return i;
        list++;    // Compare next symbol
        continue;
      }
      break;  // Not equal, break
    }
    // Set new substring ptr
    while (1) {
      // End of string, not found
      if (*list == 0) return -1;
      if (*list++ == '|') break;
    }
    i++;
  }
  return -1;
}

VNA_SHELL_FUNCTION(cmd_pause)
{
  (void)argc;
  (void)argv;
  pause_sweep();
  draw_cal_status();
}

VNA_SHELL_FUNCTION(cmd_status)
{
  (void)argc;
  (void)argv;
  if (is_paused())
    shell_printf("Paused\r\n");
  else
    shell_printf("Resumed\r\n");
}

VNA_SHELL_FUNCTION(cmd_resume)
{
  (void)argc;
  (void)argv;
  uint16_t c = 0;
  // restore frequencies array and cal
  // if (dirty)
    update_frequencies();
  if (argc == 1) {
    c = my_atoi(argv[0]);
    resume_once(c) ;
  } else
    resume_sweep();
}

VNA_SHELL_FUNCTION(cmd_repeat)
{
  (void)argc;
  (void)argv;
  uint16_t c = 0;
  if (argc == 1) {
    c = my_atoi(argv[0]);
    set_repeat(c);
  } else
    set_repeat(1);
}

VNA_SHELL_FUNCTION(cmd_reset)
{
  (void)argc;
  (void)argv;
#ifndef TINYSA4
  if (argc == 1) {
    if (get_str_index(argv[0], "dfu") == 0) {
      shell_printf("Performing reset to DFU mode\r\n");
      enter_dfu();
      return;
    }
  }
#endif
  shell_printf("Performing reset\r\n");
  rccEnableWWDG(FALSE);
  WWDG->CFR = 0x60;
  WWDG->CR = 0xff;

  /* wait forever */
  while (1)
    ;
}

int set_frequency(freq_t freq)
{
  (void) freq;
  return 1;
}

// Use macro, std isdigit more big
#define _isdigit(c) (c >= '0' && c <= '9')
// Rewrite universal standart str to value functions to more compact
//
// Convert string to int32
static long_t my_atoi(const char *p)
{
  long_t value = 0;
  uint32_t c;
  bool neg = false;

  if (*p == '-') {neg = true; p++;}
  if (*p == '+') p++;
  while ((c = *p++ - '0') < 10)
    value = value * 10 + c;
  switch (*(--p)) {
  case 'k': value *= 1000; break;
  case 'M': value *= 1000000; break;
  case 'G': value *= 1000000000; break;
  }
  return neg ? -value : value;
}

// Convert string to uint32
//  0x - for hex radix
//  0o - for oct radix
//  0b - for bin radix
//  default dec radix
freq_t my_atoui(const char *p)
{
  int d = 1;
  freq_t value = 0, radix = 10, c;
  if (*p == '+') p++;
  if (*p == '0') {
    switch (p[1]) {
      case 'x': radix = 16; break;
      case 'o': radix =  8; break;
      case 'b': radix =  2; break;
      default:  goto calculate;
    }
    p+=2;
  }
calculate:
  while (1) {
    c = *p++;
    if (c == '.') { d = 0; continue; }
    c = c - '0';
    if (c >= 'A' - '0') c = (c&(~0x20)) - ('A' - '0') + 10;
    if (c >= radix) break;
    if (value < (~(freq_t)0)/radix) {
      if (d<=0) d--;
      value = value * radix + c;
    }
  }
  if (d == 1)
    d = 0;
  switch (*(--p)) {
  case 'k': d += 3; break;
  case 'M': d += 6; break;
  case 'G': d += 9; break;
  }
  while (d < 0) {
    value /= radix;
    d++;
  }
  while (d-->0)
    value *= radix;
  return value;
}

float
my_atof(const char *p)
{
  int neg = FALSE;
  if (*p == '-')
    neg = TRUE;
  if (*p == '-' || *p == '+')
    p++;
  float x = my_atoi(p);
  while (_isdigit((int)*p))
    p++;
  if (*p == 'k' || *p == 'M' || *p == 'G')
    p++;
  if (*p == '.' || *p == ',') {
    float d = 1.0;
    p++;
    while (_isdigit((int)*p)) {
      d /= 10;
      x += d * (*p - '0');
      p++;
    }
  }
  if (*p == 'e' || *p == 'E') {
    p++;
    int exp = my_atoi(p);
    while (exp > 0) {
      x *= 10;
      exp--;
    }
    while (exp < 0) {
      x /= 10;
      exp++;
    }
  }
  switch (*p) {
  case 'k': x *= 1e+3; break;
  case 'M': x *= 1e+6; break;
  case 'G': x *= 1e+9; break;
  case 'm': x /= 1e+3; break;
  case 'u': x /= 1e+6; break;
  case 'n': x /= 1e+9; break;
  case 'p': x /= 1e+12; break;
  }

  if (neg)
    x = -x;
  return x;
}

VNA_SHELL_FUNCTION(cmd_freq)
{
  if (argc != 1 || argv[0][0] == '?') {
    goto usage;
  }
  freq_t freq = my_atoui(argv[0]);

  pause_sweep();
  set_frequency(freq);
  return;
usage:
  usage_printf("freq {frequency(Hz)}\r\n");
}

#ifdef __USE_RTC__
VNA_SHELL_FUNCTION(cmd_time)
{
  (void)argc;
  (void)argv;
  uint32_t  dt_buf[2];
  dt_buf[0] = rtc_get_tr_bcd(); // TR should be read first for sync
  dt_buf[1] = rtc_get_dr_bcd(); // DR should be read second
  static const uint8_t idx_to_time[] = {6,5,4,2,  1,  0};
  static const char       time_cmd[] = "y|m|d|h|min|sec";
  //            0    1   2       4      5     6
  // time[] ={sec, min, hr, 0, day, month, year, 0}
  uint8_t   *time = (uint8_t*)dt_buf;
  if (argc == 3 &&  get_str_index(argv[0], "b") == 0){
    rtc_set_time(my_atoui(argv[1]), my_atoui(argv[2]));
    return;
  }
  if (argc!=2 || argv[0][0] == '?') goto usage;
  int idx = get_str_index(argv[0], time_cmd);
  uint32_t val = my_atoui(argv[1]);
  if (idx < 0 || val > 99)
    goto usage;
  // Write byte value in struct
  time[idx_to_time[idx]] = ((val/10)<<4)|(val%10); // value in bcd format
  rtc_set_time(dt_buf[1], dt_buf[0]);
  return;
usage:
  usage_printf("time {[%s] 0-99} or {b 0xYYMMDD 0xHHMMSS}\r\n"\
               "20%02x/%02x/%02x %02x:%02x:%02x\r\n", time_cmd, time[6], time[5], time[4], time[2], time[1], time[0]);
}
#endif

VNA_SHELL_FUNCTION(cmd_dac)
{
  uint32_t value;
  if (argc != 1 || argv[0][0] == '?') {
    usage_printf("dac {value(0-4095)}\r\n"\
                 "current value: %d\r\n", config.dac_value);
    return;
  }
  value = my_atoui(argv[0]) & 0xFFF;
  config.dac_value = value;
  DAC->DHR12R2 = value;
}

VNA_SHELL_FUNCTION(cmd_saveconfig)
{
  (void)argc;
  (void)argv;
  config_save();
  shell_printf("Config saved.\r\n");
}

VNA_SHELL_FUNCTION(cmd_clearconfig)
{
  if (argc != 1 || argv[0][0] == '?') {
    usage_printf("clearconfig {protection key}\r\n");
    return;
  }

  if (get_str_index(argv[0], "1234") != 0) {
    shell_printf("Key unmatched.\r\n");
    return;
  }

  clear_all_config_prop_data();
  shell_printf("Config and all cal data cleared.\r\n"\
               "Do reset manually to take effect. Then do touch cal and save.\r\n");
}

#ifdef __AUDIO__
static struct {
  int16_t rms[2];
  int16_t ave[2];
  int callback_count;

#if 1
  int32_t last_counter_value;
  int32_t interval_cycles;
  int32_t busy_cycles;
#endif
} stat;
int16_t rx_buffer[AUDIO_BUFFER_LEN * 2];

#ifdef ENABLED_DUMP
int16_t dump_buffer[AUDIO_BUFFER_LEN];
int16_t dump_selection = 0;
#endif
volatile uint8_t wait_count = 0;
volatile uint8_t accumerate_count = 0;
#endif

#ifdef __VNA__
const int8_t bandwidth_accumerate_count[] = {
  1, // 1kHz
  3, // 300Hz
  10, // 100Hz
  33, // 30Hz
  100 // 10Hz
};

float measured[2][POINTS_COUNT][2];
#endif
measurement_t measured;
#ifdef __AUDIO__
#ifdef ENABLED_DUMP
static void
duplicate_buffer_to_dump(int16_t *p)
{
  if (dump_selection == 1)
    p = samp_buf;
  else if (dump_selection == 2)
    p = ref_buf;
  memcpy(dump_buffer, p, sizeof dump_buffer);
}
#endif
#ifdef __AUDIO__
void i2s_end_callback(I2SDriver *i2sp, size_t offset, size_t n)
{
#if PORT_SUPPORTS_RT
  int32_t cnt_s = port_rt_get_counter_value();
  int32_t cnt_e;
#endif
  int16_t *p = &rx_buffer[offset];
  (void)i2sp;
  (void)n;

  if (wait_count > 1) {
    --wait_count;
  } else if (wait_count > 0) {
    if (accumerate_count > 0) {
#ifndef TINYSA4
	//     dsp_process(p, n);
#endif
      accumerate_count--;
    }
#ifdef ENABLED_DUMP
    duplicate_buffer_to_dump(p);
#endif
  }

#if PORT_SUPPORTS_RT
  cnt_e = port_rt_get_counter_value();
  stat.interval_cycles = cnt_s - stat.last_counter_value;
  stat.busy_cycles = cnt_e - cnt_s;
  stat.last_counter_value = cnt_s;
#endif
  stat.callback_count++;
}

static const I2SConfig i2sconfig = {
  NULL, // TX Buffer
  rx_buffer, // RX Buffer
  AUDIO_BUFFER_LEN * 2,
  NULL, // tx callback
  i2s_end_callback, // rx callback
  0, // i2scfgr
  2 // i2spr
};
#endif
#endif

#define MAX_DATA    2
VNA_SHELL_FUNCTION(cmd_data)
{
  int i;
  int sel = 0;
  if (argc != 1 || argv[0][0] == '?')
    goto usage;
  sel = my_atoi(argv[0]);

  if (sel >= 0 && sel <= MAX_DATA) {
    static const uint8_t sel_conv[]={TRACE_TEMP, TRACE_STORED, TRACE_ACTUAL};
    float *data = measured[sel_conv[sel]];
    for (i = 0; i < sweep_points; i++)
      shell_printf("%e\r\n", value(data[i]));
    return;
  }
usage:
  usage_printf("data [0-2]\r\n");
}

#ifdef ENABLED_DUMP
VNA_SHELL_FUNCTION(cmd_dump)
{
  int i, j;
  int len;

  if (argc == 1)
    dump_selection = my_atoi(argv[0]);

  wait_dsp(3);

  len = AUDIO_BUFFER_LEN;
  if (dump_selection == 1 || dump_selection == 2)
    len /= 2;
  for (i = 0; i < len; ) {
    for (j = 0; j < 16; j++, i++) {
      shell_printf("%04x ", 0xffff & (int)dump_buffer[i]);
    }
    shell_printf("\r\n");
  }
}
#endif

#ifdef __REMOTE_DESKTOP__
uint8_t remote_mouse_down = false;
uint8_t auto_capture = false;

void send_region(remote_region_t *rd, uint8_t * buf, uint16_t size)
{
  if (SDU1.config->usbp->state == USB_ACTIVE) {
    streamWrite(shell_stream, (void*) rd, sizeof(remote_region_t));
    streamWrite(shell_stream, (void*) buf, size);
    streamWrite(shell_stream, (void*)"ch> ", 4);
  }
  else
    auto_capture = false;
}

VNA_SHELL_FUNCTION(cmd_refresh)
{
// read pixel count at one time (PART*2 bytes required for read buffer)
  int m = generic_option_cmd("refresh", "off|on", argc, argv[0]);
  if (m>=0) {
    auto_capture = m;
  }
}
VNA_SHELL_FUNCTION(cmd_touch)
{
  if (argc != 2) return;
  touch_set(my_atoi(argv[0]), my_atoi(argv[1]));
  remote_mouse_down = 1;
  handle_touch_interrupt();
}

VNA_SHELL_FUNCTION(cmd_release)
{
  if (argc == 2)
    touch_set(my_atoi(argv[0]), my_atoi(argv[1]));
  remote_mouse_down = 2;
  handle_touch_interrupt();
}

#endif

VNA_SHELL_FUNCTION(cmd_capture)
{
  // read pixel count at one time (PART*2 bytes required for read buffer)
  (void)argc;
  (void)argv;
  int y;
#ifdef TINYSA4  
#if SPI_BUFFER_SIZE < (2*LCD_WIDTH)
#error "Low size of spi_buffer for cmd_capture"
#endif
#else
#if SPI_BUFFER_SIZE < (3*LCD_WIDTH + 1)
#error "Low size of spi_buffer for cmd_capture"
#endif
#endif
  // read 2 row pixel time (read buffer limit by 2/3 + 1 from spi_buffer size)
  for (y = 0; y < LCD_HEIGHT; y += 2) {
    // use uint16_t spi_buffer[2048] (defined in ili9341) for read buffer
    uint8_t *buf = (uint8_t *)spi_buffer;
    ili9341_read_memory(0, y, LCD_WIDTH, 2, spi_buffer);
    streamWrite(shell_stream, (void*)buf, 2 * LCD_WIDTH * sizeof(uint16_t));
  }
}

#ifdef ENABLE_SD_CARD_CMD
#ifndef __USE_SD_CARD__
#error "Need enable SD card support __USE_SD_CARD__ in nanovna.h, for use ENABLE_SD_CARD_CMD"
#endif

static FRESULT cmd_sd_card_mount(void){
  const FRESULT res = f_mount(fs_volume, "", 1);
  if (res != FR_OK)
    shell_printf("err: (%d) no card\r\n",res);
  return res;
}

VNA_SHELL_FUNCTION(cmd_sd_list)
{
  (void)argc;
  (void)argv;

  DIR dj;
  FILINFO fno;
  FRESULT res;
  if (cmd_sd_card_mount() != FR_OK)
    return;
  char *search;
  switch (argc){
    case 0: search =   "*.*";break;
    case 1: search = argv[0];break;
    default: usage_printf("sd_list {pattern}\r\n"); return;
  }
//  shell_printf("sd_list:\r\n");
  res = f_findfirst(&dj, &fno, "", search);
  while (res == FR_OK && fno.fname[0])
  {
    shell_printf("%s %u\r\n", fno.fname, fno.fsize);
    res = f_findnext(&dj, &fno);
  }
  if (res != FR_OK)
    shell_printf("err: (%d)\r\n",res);
  f_closedir(&dj);
}

VNA_SHELL_FUNCTION(cmd_sd_read)
{
//  DIR dj;
//  FILINFO fno;
  FRESULT res;
  char *buf = (char *)spi_buffer;
  if (argc != 1 || argv[0][0] == '?')
  {
     usage_printf("usage: sd_read {filename}\r\n");
     return;
  }
  const char *filename = argv[0];
  if (cmd_sd_card_mount() != FR_OK)
    return;

 // res = f_findfirst(&dj, &fno, "", filename);
 // if (res != FR_OK || fno.fname[0] == 0)
 //   goto error;

  if ((res = f_open(fs_file, filename, FA_OPEN_EXISTING | FA_READ)) != FR_OK){
//error:
    shell_printf("err: (%d) no file\r\n", res);
    return;
  }
  // number of bytes to follow (file size)
  uint32_t filesize = f_size(fs_file);
  streamWrite(shell_stream, (void *)&filesize, 4);
  UINT size = 0;
  // file data (send all data from file)
  while ((res=f_read(fs_file, buf, 512, &size)) == FR_OK && size > 0)
    streamWrite(shell_stream, (void *)buf, size);
//  if (res != FR_OK)
//    goto error;
  f_close(fs_file);
  return;
}

VNA_SHELL_FUNCTION(cmd_sd_delete)
{
  DIR dj;
  FILINFO fno;
  FRESULT res;
  if (argc != 1 || argv[0][0] == '?') {
     usage_printf("sd_delete {filename}\r\n");
     return;
  }
  if (cmd_sd_card_mount() != FR_OK)
    return;
  res = f_findfirst(&dj, &fno, "", argv[0]);
  while (res == FR_OK && fno.fname[0])
  {
    res = f_unlink(fno.fname);
    shell_printf("delete: %s %s\r\n", fno.fname, res == FR_OK ? "OK" : "err");
    res = f_findnext(&dj, &fno);
  }

  return;
}
#endif

config_t config = {
  .magic =             CONFIG_MAGIC,
  .dac_value =         1922,
//  .touch_cal =         { 693, 605, 124, 171 },  // 2.4 inch LCD panel
#ifdef TINYSA3
  .touch_cal =          { 534, 741, 3458, 3434 }, // 2.8 inch LCD panel
#endif
#ifdef TINYSA4
  .touch_cal =          { 444, 715, 3552, 3499 }, // 4 inch panel
#endif
  ._mode     = _MODE_USB | _MODE_AUTO_FILENAME,
  ._serial_speed = 1843200,//19200,//SERIAL_DEFAULT_BITRATE,
  
  .lcd_palette = LCD_DEFAULT_PALETTE,
#ifdef TINYSA3
  .vbat_offset = 500,
  .low_level_offset =       0,    // Uncalibrated
  .high_level_offset =      0,    // Uncalibrated
  .correction_frequency = { { 10000, 100000, 200000, 500000, 30000000, 140000000, 200000000, 300000000, 330000000, 350000000 },
                            { 240000000, 280000000, 300000000, 400000000, 500000000, 600000000, 700000000, 800000000, 900000000, 960000000 }},
#ifdef __ULTRA__
  .correction_value = { { +6.0, +2.8, +1.6, -0.4, 0.0, -0.4, +0.4, +0.4, +0.4, +0.4 },
                        { 0, 0, 0, 0, 0.0, 0, 0, 0, 0, 0 } },
#else
  .correction_value = { { +6.0, +2.8, +1.6, -0.4, 0.0, -0.4, +0.4, +3.0, +4.0, +8.1 },
                        { 0, 0, 0, 0, 0.0, 0, 0, 0, 0, 0 } },
#endif
  .setting_frequency_10mhz = 10000000,
  .cor_am = 0,// -10,
  .cor_wfm = 0, //-18,
  .cor_nfm = 0, //-18,
  .ext_zero_level = 128,
#ifdef __ULTRA
  .ultra_start = 350000000,
  .ultra = false,
#endif
#endif
#ifdef TINYSA4
  ._brightness  = DEFAULT_BRIGHTNESS,
  .vbat_offset = 300,
  .frequency_IF1 = DEFAULT_IF,
  .frequency_IF2 = 0,
  .ultra_start = ULTRA_AUTO,
  .low_level_offset =       0,    // Uncalibrated
  .high_level_offset =      0,      // Uncalibrated
  .lna_level_offset = 0,
  .low_level_output_offset =   0,    // Uncalibrated
  .high_level_output_offset =  0,    // Uncalibrated, but checking code is not yet present
  .harmonic_level_offset = 10.5,
  .shift1_level_offset = 0.5,
  .shift2_level_offset = 3,
  .shift3_level_offset = 0,
  .drive1_level_offset = 0,
  .drive2_level_offset = -1.5,
  .drive3_level_offset = -0.5,
  .direct_level_offset =       30.0,    // Uncalibrated
  .ultra_level_offset =        0.0,    // Uncalibrated
  .direct_lna_level_offset = 0,
  .ultra_lna_level_offset = 0,
  .adf_level_offset = 0,
  .correction_frequency =
  {
   /* low */   {   10000,  40000,  100000,     300000,     500000,     900000,     4000000,    6000000,    30000000,   90000000,   210000000,  300000000,  380000000,  510000000,  600000000,  690000000,  780000000,  810000000,  820000000,  830000000},
   /* low lna */   {   10000,  40000,  100000,     300000,     500000,     900000,     4000000,    6000000,    30000000,   90000000,   150000000,  300000000,  380000000,  570000000,  600000000,  750000000,  770000000,  810000000,  820000000,  830000000},
   /* ultra */ {   30000000,   700000000,  880000000,  1540000000,     1660000000,     1850000000,     2210000000,     2420000000,     2650000000,     2800000000,     2810000000,     3420000000,     3900000000,     4340000000,     4490000000,     4880000000,     4970000000,     5300000000,     5490000000,     6000000000},
   /* ultra lna */ {   30000000,   700000000,  1720000000,     2270000000,     2530000000,     2800000000,     2810000000,     3160000000,     3420000000,     3780000000,     4260000000,     4340000000,     4450000000,     4540000000,     4940000000,     5150000000,     5280000000,     5380000000,     5640000000,     6000000000},
   /* direct */    {   140000000,  150000000,  160000000,  180000000,  280000000,  380000000,  390000000,  410000000,  490000000,  560000000,  830000000,  840000000,  890000000,  900000000,  920000000,  940000000,  980000000,  1020000000,     1070000000,     1130000000},
   /* direct lna */    {   140000000,  160000000,  180000000,  280000000,  300000000,  380000000,  390000000,  450000000,  490000000,  560000000,  830000000,  880000000,  920000000,  930000000,  950000000,  1010000000,     1040000000,     1070000000,     1120000000,     1130000000},
   /* out */   {   10000,  20000,  50000,  100000,     200000,     500000,     2000000,    8000000,    30000000,   150000000,  390000000,  470000000,  640000000,  740000000,  770000000,  800000000,  810000000,  820000000,  823000000,  830000000,  },
   /* direct */    {   500000000,  823000000,  830000000,  840000000,  890000000,  920000000,  930000000,  940000000,  950000000,  970000000,  990000000,  1000000000,     1020000000,     1030000000,     1040000000,     1050000000,     1060000000,     1080000000,     1110000000,     1130000000,     },
   /* adf */   {   500000000,  700000000,  823000000,  980000000,  1050000000,     1300000000,     1550000000,     1850000000,     2030000000,     2800000000,     2810000000,     2970000000,     3100000000,     3210000000,     3470000000,     3780000000,     3910000000,     4060000000,     4190000000,     4400000000,     },
   /* ultra */ {   10000,  20000,  70000,  200000,     500000,     3000000,    140000000,  480000000,  910000000,  1210000000,     1580000000,     2000000000,     2480000000,     2800000000,     2810000000,     3710000000,     4230000000,     4940000000,     5200000000,     5400000000,     },
  },
  .correction_value =
  {
   /* low */   {   14.18,  9.5,    7.6,    3.78,   1.47,   0.12,   0.29,   0,  0,  -0.77,  0.68,   0.37,   1.42,   0.97,   0.78,   0.69,   2.62,   5.22,   6.9,    9.54},
   /* low lna */   {   12.89,  9.66,   7.23,   5.19,   3.51,   0.97,   0.86,   0.28,   0,  0.05,   -0.1,   0.88,   1.3,    0.41,   0.8,    1.7,    2.33,   5.94,   7.03,   9.74},
   /* ultra */ {   0,  0.46,   0.63,   2.59,   2.7,    2.3,    2.59,   3.34,   3.48,   2.89,   3.98,   6.47,   7.47,   7.34,   8.68,   11.35,  11.6,   11,     11.09,  13.21},
   /* ultra lna */ {   0,  0.74,   1.93,   1.81,   2.64,   1.55,   2.45,   5.1,    7.78,   9.46,   13.98,  15.34,  18.41,  19.96,  23.21,  21.82,  22.09,  24.23,  25.42,  26.02},
   /* direct */    {   4.98,   3.43,   2.6,    0,  -8.46,  -14.19,     -13.67,     -15.69,     -18.7,  -21.64,     -30.68,     -30.3,  -30.13,     -29.18,     -29.46,     -28.56,     -27.77,     -26.79,     -25.11,     -24.06},
   /* direct lna */    {   4.98,   3,  0,  -9.4,   -11,    -15.63,     -16.11,     -18.7,  -20.79,     -23.52,     -31.62,     -31.58,     -31.03,     -30.36,     -30.43,     -28.56,     -27.31,     -27.05,     -25.41,     -25.41},
   /* out */   {   8.21,   5.86,   3.02,   0.76,   -1.17,  -2.75,  -3.77,  -4.3,   -4.58,  -4.66,  -2.6,   -3.27,  -3.38,  -2.34,  -1.47,  0.51,   1.48,   2.83,   3.41,   5.57,   },
   /* direct */    {   -8.86,  -4.93,  -4.84,  -4.74,  -4.2,   -3.88,  -3.78,  -3.63,  -3.51,  -3.23,  -2.95,  -2.77,  -2.56,  -2.46,  -2.33,  -2.21,  -2.08,  -1.74,  -1.48,  -1.22,  },
   /* adf */   {   -1.88,  -4.15,  -6.6,   -6.7,   -7.92,  -4.23,  -1.31,  -0.36,  -0.79,  0.19,   -0.95,  -1.8,   -0.71,  -0.97,  0.98,   5.56,   7.16,   7.49,   6.83,   6.68,   },
   /* ultra */ {   7.83,   5.55,   1.5,    -1.54,  -2.96,  -4.15,  -4.87,  -3.68,  -3.78,  -2.51,  -0.46,  -2.17,  -0.7,   1.5,    0.51,   2.84,   1.87,   7.65,   6.73,   7.13,   },
  },
  .setting_frequency_30mhz = 30000000ULL * FREQ_MULTIPLIER,
  .cor_am = 0,
  .cor_wfm = 0,
  .cor_nfm = 0,
  .ultra = false,
  .input_is_calibrated = false,
  .output_is_calibrated = false,
#ifndef __NEW_SWITCHES__
  .high_out_adf4350 = true,
#endif
  .ext_zero_level = 174,
  .receive_switch_offset = 0.0,
#ifdef TINYSA4
  .out_switch_offset = 0.0,
#endif
#ifdef __NOISE_FIGURE__
  .noise_figure = 5.0,
#endif
#endif
  .sweep_voltage = 3.3,
  .switch_offset = 0.0,
#ifdef TINYSA4
  .direct_start = 965000000UL,
  .direct_stop  = 985000000UL,
  .hide_21MHz = false,
#endif
};

//properties_t current_props;
//properties_t *active_props = &current_props;



static const marker_t def_markers[MARKERS_MAX] = {
    {M_TRACKING, M_ENABLED,               0, TRACE_ACTUAL, 30, 0 },
    {M_NORMAL,   M_DISABLED,              0, TRACE_ACTUAL, 40, 0 },
    {M_NORMAL,   M_DISABLED,              0, TRACE_ACTUAL, 60, 0 },
    {M_NORMAL,   M_DISABLED,              0, TRACE_ACTUAL, 80, 0 }
};

// Load propeties default settings
void load_LCD_properties(void)
{
//Magic add on caldata_save
//setting.magic = CONFIG_MAGIC;
  setting._sweep_points = POINTS_COUNT;
  setting.trace_scale = 10.0;
  setting.trace_refpos = 0;
  setting.waterfall = W_OFF;
  setting.level_meter = false;
  setting._traces = TRACE_ACTUAL_FLAG;
  memcpy(setting._markers, def_markers, sizeof(def_markers));
#ifdef __LIMITS__
  memset(setting.limits, 0, sizeof(setting.limits));
#endif
  setting._active_marker   = 0;
  reset_settings(M_LOW);
//Checksum add on caldata_save
//setting.checksum = 0;
}

#include "sa_core.c"
#ifdef __AUDIO__
#define DSP_START(delay) wait_count = delay;
#define DSP_WAIT_READY   while (wait_count) __WFI();
#endif

void set_sweep_points(uint16_t points){
  if (points == sweep_points || points > POINTS_COUNT)
    return;

  sweep_points = points;
  update_frequencies();
}

VNA_SHELL_FUNCTION(cmd_scan)
{
  freq_t start = get_sweep_frequency(ST_START);
  freq_t stop  = get_sweep_frequency(ST_STOP);
  uint32_t old_points = sweep_points;
  uint32_t i;
  if (argc == 0)
    goto do_scan;
  if (argc < 2 || argc > 4) {
    usage_printf("scan {start(Hz)} {stop(Hz)} [points] [outmask]\r\n");
    return;
  }

  start = my_atoui(argv[0]);
  stop = my_atoui(argv[1]);
  if (start > stop) {
      shell_printf("frequency range is invalid\r\n");
      return;
  }
  if (argc >= 3) {
    int points = my_atoi(argv[2]);
    if (points <= 0 || points > POINTS_COUNT) {
      shell_printf("sweep points exceeds range "define_to_STR(POINTS_COUNT)"\r\n");
      return;
    }
    sweep_points = points;
  }
  set_frequencies(start, stop, sweep_points);
do_scan:
  pause_sweep();
  setting.sweep = true;         // prevent abort
  sweep(false);
  setting.sweep = false;
  // Output data after if set (faster data recive)
  if (argc == 4) {
    uint16_t mask = my_atoui(argv[3]);
    if (mask) {
      for (i = 0; i < sweep_points; i++) {
        if (mask & 1) shell_printf("%U ", getFrequency(i));
        if (mask & 2) shell_printf("%e %f ", value(measured[TRACE_ACTUAL][i]), 0.0);
        if (mask & 4) shell_printf("%e %f ", value(measured[TRACE_STORED][i]), 0.0);
        if (mask & 8) shell_printf("%e %f ", value(measured[TRACE_TEMP][i]), 0.0);
        shell_printf("\r\n");
      }
    }
  }
  sweep_points = old_points;
}

#ifdef TINYSA4
VNA_SHELL_FUNCTION(cmd_hop)
{
  freq_t start, stop, step = 0;
  if (argc < 1 || argc > 4) {
    usage_printf("hop {start(Hz)} {stop(Hz)} {step(Hz) | points} [outmask]\r\n");
    return;
  }

  start = my_atoui(argv[0]);
  if (argc > 1)
    stop = my_atoui(argv[1]);
  else {
    stop = start;
    step = 1; // just to stop the loop
  }

  if (argc >= 3) {
    step = my_atoui(argv[2]);
    if (step <= POINTS_COUNT && step > 0) {
      step = (stop - start) / step;
    }
  } else
    step = 1;
  int old_sweep = sweep_mode;
  if (old_sweep & SWEEP_ENABLE)
    pause_sweep();
  else
    dirty = true;
  // Output data after if set (faster data recive)
  uint16_t mask = 3;
  if (argc == 4) {
    mask = my_atoui(argv[3]);
  }

  if (argc==2) {
    mask = stop;
    stop = start;
  }

  if (start > stop) {
      shell_printf("frequency range is invalid\r\n");
      return;
  }
  if (mask) {
    int old_vbwSteps = vbwSteps;
//    vbwSteps = 1;
    for (freq_t f = start; f <= stop; f += step) {
        if (mask & 1) shell_printf("%U ", f);
        float v = PURE_TO_float(perform(false, 0, f, false));
        if (mask & 2) shell_printf("%f ", v);
        shell_printf("\r\n");
        if (operation_requested)
          break;
    }
    vbwSteps = old_vbwSteps;
  }
  if (old_sweep & SWEEP_ENABLE)
    resume_sweep();
}
#endif

static void
update_markers_index(void)
{
  int m, idx;
  freq_t fstart = get_sweep_frequency(ST_START);
  freq_t fstop  = get_sweep_frequency(ST_STOP);
  for (m = 0; m < MARKERS_MAX; m++) {
    if (!markers[m].enabled)
      continue;
    if (markers[m].mtype & M_STORED)
      continue;
    freq_t f = markers[m].frequency;
    if (f == 0) idx = markers[m].index; // Not need update index in no freq
    else if (f < fstart) idx = 0;
    else if (f >= fstop) idx = sweep_points-1;
    else { // Search frequency index for marker frequency
#if 1
      for (idx = 1; idx < sweep_points; idx++) {
        if (getFrequency(idx) <= f) continue;
        if (f < (getFrequency(idx-1)/2 + getFrequency(idx)/2)) idx--; // Correct closest idx
        break;
      }
#else
      float r = ((float)(f - fstart))/(fstop - fstart);
      idx = r * (sweep_points-1);
#endif
    }
    set_marker_index(m, idx);
  }
}

void
set_marker_index(int m, int16_t idx)
{
  if ((uint32_t)m >= MARKERS_MAX || (uint16_t)idx >= sweep_points) return;
  markers[m].index = idx;
  markers[m].frequency = getFrequency(idx);
}

void set_marker_frequency(int m, freq_t f)
{
  if (m == MARKER_INVALID || !markers[m].enabled)
    return;
  int i = 1;
  markers[m].mtype &= ~M_TRACKING;
  freq_t s = (getFrequency(1) - getFrequency(0))/2;
  while (i< sweep_points - 2){
    if (getFrequency(i)-s  <= f && f < getFrequency(i+1)-s) {     // Avoid rounding error in s!!!!!!!
      markers[m].index = i;
      markers[m].frequency = f;
      return;
    }
    i++;
  }
}

void set_marker_time(int m, float f)
{
  if (m == MARKER_INVALID || !markers[m].enabled)
    return;
  markers[m].mtype &= ~M_TRACKING;
  int i = f * (float)(sweep_points-1)* ONE_SECOND_TIME / setting.actual_sweep_time_us;
  if (i >= sweep_points)
    return;
  markers[m].index = i;
  markers[m].frequency = 0;
}

/*
 * Frequency list functions
 */
#ifdef __USE_FREQ_TABLE__
freq_t frequencies[POINTS_COUNT];
static void
set_frequencies(freq_t start, freq_t stop, uint16_t points)
{
  uint32_t i;
  freq_t step = (points - 1);
  freq_t span = stop - start;
  freq_t delta = span / step;
  freq_t error = span % step;
  freq_t f = start, df = step>>1;
  for (i = 0; i <= step; i++, f+=delta) {
    frequencies[i] = f;
    if ((df+=error) >= step) {f++; df-= step;}
  }
  // disable at out of sweep range
  for (; i < POINTS_COUNT; i++)
    frequencies[i] = 0;
  setting.frequency_step = delta;
  dirty = true;
}
#ifndef getFrequency
freq_t getFrequency(uint16_t idx) {return frequencies[idx];}
#endif
#else
static freq_t   _f_start;
static freq_t   _f_delta;
static freq_t   _f_error;
static uint16_t _f_count;

static void
set_frequencies(freq_t start, freq_t stop, uint16_t points)
{
  freq_t span = stop - start;
  _f_start = start;
  _f_count = (points - 1);
  _f_delta = span / _f_count;
  _f_error = span % _f_count;
  setting.frequency_step = _f_delta;
  dirty = true;
}
freq_t getFrequency(uint16_t idx) {return _f_start + _f_delta * idx + (_f_count / 2 + _f_error * idx) / _f_count;}
#endif


void
update_frequencies(void)
{
  freq_t start, stop;
  start = get_sweep_frequency(ST_START);
  stop  = get_sweep_frequency(ST_STOP);

  set_frequencies(start, stop, sweep_points);
  // operation_requested|= OP_FREQCHANGE;

  update_markers_index();

  // set grid layout
  update_grid();
}

void
set_sweep_frequency(int type, freq_t freq)
{
  // Check frequency for out of bounds (minimum SPAN can be any value)
  if (type != ST_SPAN && freq < START_MIN)
    freq = START_MIN;
  if (freq > STOP_MAX)
    freq = STOP_MAX;
  bool cw_mode = FREQ_IS_CW(); // remember old mode
  freq_t center, span=0;
  switch (type) {
    case ST_START:
      setting.freq_mode &= ~FREQ_MODE_CENTER_SPAN;
      setting.frequency0 = freq;
      // if start > stop then make start = stop
      if (setting.frequency1 < freq) setting.frequency1 = freq;
      span = (setting.frequency1 - setting.frequency0)/2;
      break;
    case ST_STOP:
      setting.freq_mode &= ~FREQ_MODE_CENTER_SPAN;
      setting.frequency1 = freq;
      // if start > stop then make start = stop
      if (setting.frequency0 > freq) setting.frequency0 = freq;
      span = (setting.frequency1 - setting.frequency0)/2;
      break;
    case ST_CENTER:
      setting.freq_mode |= FREQ_MODE_CENTER_SPAN;
      center = setting.frequency0/2 + setting.frequency1/2;
      span   = (setting.frequency1 - setting.frequency0)/2;
      if (freq < START_MIN + span)
        span = (freq - START_MIN);
      if (freq > STOP_MAX - span)
        span = (STOP_MAX - freq);
      setting.frequency0 = freq - span;
      setting.frequency1 = freq + span;
      break;
    case ST_SPAN:
      setting.freq_mode |= FREQ_MODE_CENTER_SPAN;
      center = setting.frequency0/2 + setting.frequency1/2;
      span = freq/2;
      if (center < START_MIN + span)
        center = START_MIN + span;
      if (center > STOP_MAX - span)
        center = STOP_MAX - span;
      setting.frequency0 = center - span;
      setting.frequency1 = center + span;
      break;
    case ST_CW:
    force_cw:
      setting.freq_mode |= FREQ_MODE_CENTER_SPAN;
      setting.frequency0 = freq;
      setting.frequency1 = freq;
      span = 0;
      break;
  }
  if (span !=0 && span < sweep_points/2) {
    freq = (setting.frequency1 + setting.frequency0)/2;
    if (freq & 0x0001)
      freq++;
    goto force_cw;
  }
  if (!cw_mode && FREQ_IS_CW()) // switch to CW mode
    setting.sweep_time_us = 0;  // use minimum as start
  update_frequencies();
}

freq_t
get_sweep_frequency(int type)
{
  // Obsolete, ensure correct start/stop, start always must be < stop
  if (setting.frequency0 > setting.frequency1) {
    freq_t t = setting.frequency0;
    setting.frequency0 = setting.frequency1;
    setting.frequency1 = t;
  }
  switch (type) {
    case ST_START:  return setting.frequency0;
    case ST_STOP:   return setting.frequency1;
    case ST_CENTER: return setting.frequency0/2 + setting.frequency1/2;
    case ST_SPAN:   return setting.frequency1 - setting.frequency0;
    case ST_CW:     return setting.frequency0;
  }
  return 0;
}

VNA_SHELL_FUNCTION(cmd_sweep)
{
  if (argc == 0) {
    shell_printf("%D %D %d\r\n", get_sweep_frequency(ST_START), get_sweep_frequency(ST_STOP), sweep_points);
    return;
  } else if (argc > 3) {
    goto usage;
  }
  if (argv[0][0] == '?')
    goto usage;
  freq_t value0 = 0;
  freq_t value1 = 0;
  freq_t value2 = 0;
  if (argc >= 1) value0 = my_atoui(argv[0]);
  if (argc >= 2) value1 = my_atoui(argv[1]);
  if (argc >= 3) value2 = my_atoui(argv[2]);
#if MAX_FREQ_TYPE != 5
#error "Sweep mode possibly changed, check cmd_sweep function"
#endif
  // Parse sweep {start|stop|center|span|cw} {freq(Hz)}
  // get enum ST_START, ST_STOP, ST_CENTER, ST_SPAN, ST_CW
  static const char sweep_cmd[] = "start|stop|center|span|cw";
  int type = get_str_index(argv[0], sweep_cmd);
  if (type >=0) {
    set_sweep_frequency(type, value1);
    return;
  }
  // Parse sweep {go|abort}
  static const char sweep_cmd2[] = "normal|precise|fast|noise|go|abort";
  int type2 = get_str_index(argv[0], sweep_cmd2);
  if (type2 >=0 && type2 <= 3) { set_step_delay(type2);return;}
  if (type2==4) { setting.sweep = true; return;}
  if (type2==5) { setting.sweep = false; return;}
  //  Parse sweep {start(Hz)} [stop(Hz)]
  set_sweep_frequency(ST_START, value0);
  if (value1)
    set_sweep_frequency(ST_STOP, value1);
  if (value2)
    set_sweep_points(value2);
  return;
usage:
  usage_printf("sweep {start(Hz)} [stop(Hz)] [points]\r\n"\
                    "\tsweep {%s}\r\n"\
                    "\tsweep {%s} {freq(Hz)}\r\n", sweep_cmd2, sweep_cmd);
}

VNA_SHELL_FUNCTION(cmd_save)
{
  if (argc != 1 || argv[0][0] == '?')
    goto usage;

  int id = my_atoi(argv[0]);
  if (id < 0 || id >= SAVEAREA_MAX)
    goto usage;
  caldata_save(id);
  redraw_request |= REDRAW_CAL_STATUS;
  return;

 usage:
  shell_printf("save {id}\r\n");
}

VNA_SHELL_FUNCTION(cmd_recall)
{
  if (argc != 1 || argv[0][0] == '?')
    goto usage;

  int id = my_atoi(argv[0]);
  if (id < 0 || id >= SAVEAREA_MAX)
    goto usage;
  // Check for success
  if (caldata_recall(id) == -1)
    shell_printf("Err, default load\r\n");
  update_frequencies();
  redraw_request |= REDRAW_CAL_STATUS;
  return;
 usage:
  shell_printf("recall {id}\r\n");
}

const char * const trc_channel_name[TRACES_MAX] = {
  [TRACE_ACTUAL] = "MEASURED",
  [TRACE_STORED] = "STORED",
  [TRACE_TEMP]   = "RAW",
};

void set_trace_scale(float scale)
{
  if (setting.trace_scale == scale) return;
  setting.trace_scale = scale;
  redraw_request |= REDRAW_AREA | REDRAW_CAL_STATUS;
}

void set_trace_refpos(float refpos)
{
  if (setting.trace_refpos == refpos) return;
  setting.trace_refpos = refpos;
  redraw_request |= REDRAW_AREA | REDRAW_CAL_STATUS;
}

VNA_SHELL_FUNCTION(cmd_trace)
{
  int t = 0;
  bool do_one = false;
  if (argc==1 && argv[0][0] == '?')
    goto usage;
  if (argc == 0) {
    for (t = 0; t < TRACES_MAX; t++) {
show_one:
      if (IS_TRACE_ENABLE(t)) {
        const char *type = unit_string[setting.unit]; // get_trace_typename(t);
//        const char *channel = trc_channel_name[t];
        float scale = get_trace_scale();
        float refpos = get_trace_refpos();
        shell_printf("%d: %s %f %f \r\n", t+1, type, refpos, scale, (setting.stored[t]?", frozen":""));
      }
      if (do_one) break;
    }
    return;
  }
  int next_arg = 0;
  if ('0' <= argv[0][0] && argv[0][0] <= '9') {
    t = my_atoi(argv[0]) - 1;
    next_arg++;
    argc--;
    if (t < 0 || t >= TRACES_MAX)
      goto usage;
    if (argc >= 1)
      goto process;
    do_one = true;
    goto show_one;
  }
#if MAX_UNIT_TYPE != 6
#error "Unit type enum possibly changed, check cmd_trace function"
#endif
  static const char cmd_type_list[] = "dBm|dBmV|dBuV|RAW|V|W";
  if (argc == 1) {
    int type = get_str_index(argv[0], cmd_type_list);
    if (type >= 0) {
      set_unit(type);
      goto update;
    }
//    goto usage;
  }
  //                                            0      1
  static const char cmd_scale_ref_list[] = "scale|reflevel";
  if (argc == 2) {
    switch (get_str_index(argv[0], cmd_scale_ref_list)) {
    case 0:
      if (get_str_index(argv[1],"auto") == 0) {
        set_auto_reflevel(true);
      } else {
        user_set_scale(my_atof(argv[1]));
      }
      goto update;
    case 1:
      //trace[t].refpos = my_atof(argv[2]);
      if (get_str_index(argv[1],"auto") == 0) {
        set_auto_reflevel(true);
      } else {
        user_set_reflevel(my_atof(argv[1]));
      }
      goto update;
    }
  }

  static const char cmd_value_list[] = "value";
  process:
  if (argc == 1) {
    int type = get_str_index(argv[next_arg], cmd_value_list);
    if (type >= 0) {
      switch(type) {
      case 0:
        for (int i=0;i<sweep_points;i++) {
          shell_printf("trace %d value %d %.2f\r\n", t+1, i, measured[t][i]);
        }
      }
    }
//    goto usage;
  }
  static const char cmd_load_list[] = "copy|freeze|subtract|view|value";
  if (argc >= 2) {
    switch (get_str_index(argv[next_arg++], cmd_load_list)) {
    case 0:
      store_trace(t, my_atoi(argv[next_arg++])-1); // copy {trace}
      goto update;
    case 1:
      setting.stored[t]= (get_str_index(argv[next_arg++], "off|on") == 1); // freeze {off|on}
      goto update;
    case 2:
      subtract_trace(t,my_atoi(argv[next_arg++])-1);
      goto update;
    case 3:
      if (get_str_index(argv[next_arg++], "off|on") == 1)
        { TRACE_ENABLE(1<<t); }
      else
        { TRACE_DISABLE(1<<t);}
      goto update;
    case 4:
      {
      int i = my_atoi(argv[next_arg++]);
      if (i>= sweep_points)
        goto usage;
      float v = my_atof(argv[next_arg]);
      measured[t][i] = v;
      return;
      }
    }
    goto usage;
  }
update:
redraw_request |= REDRAW_CAL_STATUS;
  return;
usage:
  shell_printf("trace {%s}\r\n"\
               "trace {%s} auto|{value}\r\n"\
               "trace [{trace#}] value\r\n"\
               "trace [{trace#}] {%s} {trace#}|off|on|[{index} {value}]\r\n"\
               , cmd_type_list,cmd_scale_ref_list, cmd_load_list);
}

VNA_SHELL_FUNCTION(cmd_marker)
{
  int t;
  if (argc == 0) {
    for (t = 0; t < MARKERS_MAX; t++) {
      if (markers[t].enabled) {
        shell_printf("%d %d %D %.2e\r\n", t+1, markers[t].index, markers[t].frequency, marker_to_value(t));
      }
    }
    return;
  }
  if (argv[0][0] == '?')
    goto usage;
  redraw_request |= REDRAW_MARKER;
  if (get_str_index(argv[0], "off") == 0) {
    active_marker = MARKER_INVALID;
    for (t = 0; t < MARKERS_MAX; t++)
      markers[t].enabled = FALSE;
    return;
  }
  t = my_atoi(argv[0])-1;
  if (t < 0 || t >= MARKERS_MAX)
    goto usage;
  if (argc == 1) {
  display_marker:
    shell_printf("%d %d %D %.2e\r\n", t+1, markers[t].index, markers[t].frequency, marker_to_value(t));
    active_marker = t;
    // select active marker
    markers[t].enabled = TRUE;
    return;
  }
  int tr;
  static const char cmd_marker_list[] = "on|off|peak|delta|noise|tracking|trace|trace_aver";
  static const char cmd_marker_on_off[] = "off|on";
  int marker_mask = 0;
  switch (get_str_index(argv[1], cmd_marker_list)) {
    case 0: markers[t].enabled = TRUE; active_marker = t; return;
    case 1: markers[t].enabled =FALSE; if (active_marker == t) active_marker = MARKER_INVALID; return;
    case 2: markers[t].enabled = TRUE; active_marker = t;
      int i = marker_search_max(active_marker);
      if (i == -1) i = 0;
      set_marker_index(active_marker, i);
      goto display_marker;
    default:
      // select active marker and move to index or frequency
      markers[t].enabled = TRUE;
      if (argv[1][0] < '0' || argv[1][0] > '9' )
        goto usage;
      freq_t value = my_atoui(argv[1]);
      markers[t].mtype &= ~M_TRACKING;
      active_marker = t;
      if (value > sweep_points)
        set_marker_frequency(active_marker, value);
      else
        set_marker_index(t, value);
      return;
      //      M_NORMAL=0,M_REFERENCE=1, M_DELTA=2, M_NOISE=4, M_STORED=8, M_AVER=16, M_TRACKING=32, M_DELETE=64  // Tracking must be last.
    case 3:
      tr=0;
      if (argc == 3 && argv[2][0] >= '1' && argv[2][0] <= '9') {
        tr = my_atoui(argv[2])-1;
        markers[t].mtype |= M_DELTA;
        markers[t].ref= tr;
      } else if (get_str_index(argv[2],cmd_marker_on_off) == 0) {
        markers[t].mtype &= ~M_DELTA;
      }
      return;
    case 4:
      marker_mask = M_NOISE;
      goto set_mask;
    case 5:
      marker_mask = M_TRACKING;
      goto set_mask;
    case 6:
      tr=0;
      if (argc == 3 && argv[2][0] >= '1' && argv[2][0] <= '9') {
        tr = my_atoui(argv[2])-1;
      }
      markers[t].trace= tr;
      return;
    case 7:
      marker_mask = M_AVER;
    set_mask:
      if (argc == 3) {
        switch (get_str_index(argv[2],cmd_marker_on_off)) {
        default: goto usage;
        case 0: markers[t].mtype &= ~marker_mask; return;
        case 1: markers[t].mtype |= marker_mask; return;
        }
      }
      return;
  }
usage:
  shell_printf("marker [n] [%s|{freq}|{index}] [{n}|%s]\r\n", cmd_marker_list, cmd_marker_on_off);
}

VNA_SHELL_FUNCTION(cmd_touchcal)
{
  (void)argc;
  (void)argv;
  //extern int16_t touch_cal[4];
  int i;

  shell_printf("first touch upper left, then lower right...");
  touch_cal_exec();
  shell_printf("done\r\n");

  shell_printf("touch cal params: ");
  for (i = 0; i < 4; i++) {
    shell_printf("%d ", config.touch_cal[i]);
  }
  shell_printf("\r\n");
}

VNA_SHELL_FUNCTION(cmd_touchtest)
{
  (void)argc;
  (void)argv;
  do {
    touch_draw_test();
  } while (argc);
}

VNA_SHELL_FUNCTION(cmd_frequencies)
{
  int i;
  (void)argc;
  (void)argv;
  for (i = 0; i < sweep_points; i++)
    shell_printf("%U\r\n", getFrequency(i));
}

VNA_SHELL_FUNCTION(cmd_test)
{
  (void)argc;
  (void)argv;

#if 0
  int i;
  for (i = 0; i < 100; i++) {
    palClearPad(GPIOB, GPIOB_LED);
    set_frequency(10000000);
    palSetPad(GPIOB, GPIOB_LED);
    chThdSleepMilliseconds(50);

    palClearPad(GPIOB, GPIOB_LED);
    set_frequency(90000000);
    palSetPad(GPIOB, GPIOB_LED);
    chThdSleepMilliseconds(50);
  }
#endif

#if 0
  int i;
  int mode = 0;
  if (argc >= 1)
    mode = my_atoi(argv[0]);

  for (i = 0; i < 20; i++) {
    palClearPad(GPIOB, GPIOB_LED);
    ili9341_test(mode);
    palSetPad(GPIOB, GPIOB_LED);
    chThdSleepMilliseconds(50);
  }
#endif

#if 0
  //extern adcsample_t adc_samples[2];
  //shell_printf("adc: %d %d\r\n", adc_samples[0], adc_samples[1]);
  int i;
  int x, y;
  for (i = 0; i < 50; i++) {
    test_touch(&x, &y);
    shell_printf("adc: %d %d\r\n", x, y);
    chThdSleepMilliseconds(200);
  }
  //extern int touch_x, touch_y;
  //shell_printf("adc: %d %d\r\n", touch_x, touch_y);
#endif

  while (argc > 1) {
    int x, y;
    touch_position(&x, &y);
    shell_printf("touch: %d %d\r\n", x, y);
    chThdSleepMilliseconds(200);
  }
}

#ifndef VERSION
#define VERSION "unknown"
#endif

const char TINYSA_VERSION[] = VERSION;

#ifdef TINYSA4
typedef struct version_t {
  const uint16_t min_adc;
  const uint16_t max_adc;
  const char *text;
} version_t;

#define MAX_VERSION_TEXT    1
const version_t hw_version_text[MAX_VERSION_TEXT] =
{
 { 160, 169,    "V0.4.5.1"}
};

const char *get_hw_version_text(void)
{
  int v = adc1_single_read(0);
  for (int i=0; i<MAX_VERSION_TEXT;i++) {
    if (hw_version_text[i].min_adc <= v && v <= hw_version_text[i].max_adc)
      return hw_version_text[i].text;
  }
  return "Unknown";
}
#endif

VNA_SHELL_FUNCTION(cmd_version)
{
  (void)argc;
  (void)argv;
#ifdef TINYSA4
  shell_printf("%s\r\nHW Version:%s\r\n", TINYSA_VERSION, get_hw_version_text());
#else
  shell_printf("%s\r\n", TINYSA_VERSION);
#endif
}

VNA_SHELL_FUNCTION(cmd_vbat)
{
  (void)argc;
  (void)argv;
  shell_printf("%d mV\r\n", adc_vbat_read());
}

#ifdef ENABLE_VBAT_OFFSET_COMMAND
VNA_SHELL_FUNCTION(cmd_vbat_offset)
{
  if (argc != 1 || argv[0][0] == '?') {
    shell_printf("%d\r\n", config.vbat_offset);
    return;
  }
  config.vbat_offset = (int16_t)my_atoi(argv[0]);
}
#endif

#ifdef ENABLE_INFO_COMMAND
VNA_SHELL_FUNCTION(cmd_info)
{
  (void)argc;
  (void)argv;
  int i = 0;
  while (info_about[i])
    shell_printf("%s\r\n", info_about[i++]);
#ifdef TINYSA3
  if (has_esd)
    shell_printf("ESD protected\r\n");
#endif
}
#endif

#ifdef ENABLE_COLOR_COMMAND
VNA_SHELL_FUNCTION(cmd_color)
{
  uint32_t color;
  int i;
  if (argc != 2) {
    usage_printf("color {id} {rgb24}\r\n");
    for (i=0; i < MAX_PALETTE; i++) {
      color = GET_PALTETTE_COLOR(i);
      color = HEXRGB(color);
      shell_printf(" %2d: 0x%06x\r\n", i, color);
    }
    return;
  }
  i = my_atoi(argv[0]);
  if (i >= MAX_PALETTE)
    return;
  color = RGBHEX(my_atoui(argv[1]));
  config.lcd_palette[i] = color;

  // Redraw all
  redraw_request|= REDRAW_AREA;
}
#endif

#ifdef ENABLE_THREADS_COMMAND
#if CH_CFG_USE_REGISTRY == FALSE
#error "Threads Requite enabled CH_CFG_USE_REGISTRY in chconf.h"
#endif
const char *states[] = {CH_STATE_NAMES};
VNA_SHELL_FUNCTION(cmd_threads) 
{
  thread_t *tp;
  (void)argc;
  (void)argv;
  shell_printf("stklimit|        |stk free|    addr|refs|prio|    state|        name"VNA_SHELL_NEWLINE_STR);
  tp = chRegFirstThread();
  do {
    uint32_t max_stack_use = 0U;
#if (CH_DBG_ENABLE_STACK_CHECK == TRUE) || (CH_CFG_USE_DYNAMIC == TRUE)
    uint32_t stklimit = (uint32_t)tp->wabase;
#if CH_DBG_FILL_THREADS == TRUE
    uint8_t *p = (uint8_t *)tp->wabase; while(p[max_stack_use]==CH_DBG_STACK_FILL_VALUE) max_stack_use++;
#endif
#else
    uint32_t stklimit = 0U;
#endif
    shell_printf("%08x|%08x|%08x|%08x|%4u|%4u|%9s|%12s"VNA_SHELL_NEWLINE_STR,
             stklimit, (uint32_t)tp->ctx.sp, max_stack_use, (uint32_t)tp,
             (uint32_t)tp->refs - 1, (uint32_t)tp->prio, states[tp->state],
             tp->name == NULL ? "" : tp->name);
    tp = chRegNextThread(tp);
  } while (tp != NULL);
}
#endif

#ifdef ENABLE_USART_COMMAND
VNA_SHELL_FUNCTION(cmd_usart_cfg)
{
  if (argc != 1 || argv[0][0] == '?') goto result;
  uint32_t speed = my_atoui(argv[0]);
  if (speed < 300) speed = 300;
  config._serial_speed = speed;
  shell_update_speed();
result:
  shell_printf("Serial: %u baud\r\n", config._serial_speed);
}

VNA_SHELL_FUNCTION(cmd_usart)
{
  uint32_t time = 2000; // 200ms wait answer by default
  if (argc == 0 || argc > 2 || (config._mode & _MODE_SERIAL)) return;
  if (argc == 2) time = my_atoui(argv[1])*10;
  sdWriteTimeout(&SD1, (uint8_t *)argv[0], strlen(argv[0]), time);
  sdWriteTimeout(&SD1, (uint8_t *)VNA_SHELL_NEWLINE_STR, sizeof(VNA_SHELL_NEWLINE_STR)-1, time);
  uint32_t size;
  uint8_t buffer[64];
  while ((size = sdReadTimeout(&SD1, buffer, sizeof(buffer), time)))
    streamWrite(&SDU1, buffer, size);
}
#endif
#include "sa_cmd.c"

//=============================================================================
VNA_SHELL_FUNCTION(cmd_help);

#pragma pack(push, 2)
typedef struct {
  const char           *sc_name;
  vna_shellcmd_t    sc_function;
  uint16_t flags;
} VNAShellCommand;
#pragma pack(pop)

// Some commands can executed only in sweep thread, not in main cycle
#define CMD_WAIT_MUTEX  1
#define CMD_RUN_IN_LOAD 2
#define CMD_RUN_IN_UI   4
static const VNAShellCommand commands[] =
{
    {"version"     , cmd_version     , 0},
    {"reset"       , cmd_reset       , 0},
    {"freq"        , cmd_freq        , CMD_WAIT_MUTEX | CMD_RUN_IN_LOAD},
#ifdef __USE_RTC__
    {"time"        , cmd_time        , CMD_RUN_IN_LOAD},
#endif
    {"dac"         , cmd_dac         , CMD_RUN_IN_LOAD},
    {"sweep_voltage",cmd_sweep_voltage,CMD_RUN_IN_LOAD},
#ifdef __NOISE_FIGURE__
    {"nf",          cmd_nf,            CMD_RUN_IN_LOAD},
#endif
    {"saveconfig"  , cmd_saveconfig  , CMD_RUN_IN_LOAD},
    {"clearconfig" , cmd_clearconfig , CMD_RUN_IN_LOAD},
    {"data"        , cmd_data        , CMD_WAIT_MUTEX},
#ifdef ENABLED_DUMP
    {"dump"        , cmd_dump        , 0},
#endif
    {"frequencies" , cmd_frequencies , 0},
//  {"gamma"       , cmd_gamma       , 0},
    {"scan"        , cmd_scan        , CMD_WAIT_MUTEX},
#ifdef TINYSA4
    {"hop"         , cmd_hop         , CMD_WAIT_MUTEX},
#endif
    {"scanraw"     , cmd_scanraw     , CMD_WAIT_MUTEX},
    {"zero"        , cmd_zero        , CMD_WAIT_MUTEX | CMD_RUN_IN_LOAD},   // Will set the scanraw measured value offset (128 or 174)
    {"sweep"       , cmd_sweep       , 0},
    {"test"        , cmd_test        , 0},
    {"touchcal"    , cmd_touchcal    , CMD_WAIT_MUTEX},
    {"touchtest"   , cmd_touchtest   , CMD_WAIT_MUTEX},
    {"pause"       , cmd_pause       , CMD_WAIT_MUTEX | CMD_RUN_IN_LOAD},
    {"resume"      , cmd_resume      , CMD_WAIT_MUTEX | CMD_RUN_IN_LOAD},
    {"repeat"      , cmd_repeat      , CMD_RUN_IN_LOAD},
    {"status"      , cmd_status      , CMD_RUN_IN_LOAD},
    {"caloutput"   , cmd_caloutput   , CMD_RUN_IN_LOAD},
    {"save"        , cmd_save        , CMD_RUN_IN_LOAD},
    {"recall"      , cmd_recall      , CMD_WAIT_MUTEX | CMD_RUN_IN_LOAD},
    {"trace"       , cmd_trace       , CMD_WAIT_MUTEX | CMD_RUN_IN_LOAD},
    {"trigger"     , cmd_trigger     , CMD_RUN_IN_LOAD},
    {"marker"      , cmd_marker      , CMD_RUN_IN_LOAD},
#ifdef __DRAW_LINE__
    {"line"        , cmd_line        , CMD_RUN_IN_LOAD},
#endif
#ifdef ENABLE_USART_COMMAND
    {"usart"       , cmd_usart       , CMD_WAIT_MUTEX},
    {"usart_cfg"   , cmd_usart_cfg   , CMD_WAIT_MUTEX | CMD_RUN_IN_LOAD},
#endif
    {"capture"     , cmd_capture     , CMD_WAIT_MUTEX | CMD_RUN_IN_UI},
#ifdef __REMOTE_DESKTOP__
    {"refresh"     , cmd_refresh     , 0},
    {"touch"       , cmd_touch       , 0},
    {"release"     , cmd_release     , 0},
#endif
    {"vbat"        , cmd_vbat        , CMD_WAIT_MUTEX},     // Uses same adc as touch!!!!!
#ifdef ENABLE_VBAT_OFFSET_COMMAND
    {"vbat_offset" , cmd_vbat_offset , CMD_RUN_IN_LOAD},
#endif
    {"help"        , cmd_help        , 0},
#ifdef ENABLE_INFO_COMMAND
    {"info"        , cmd_info        , 0},
#endif
#ifdef ENABLE_COLOR_COMMAND
    {"color"       , cmd_color       , CMD_RUN_IN_LOAD},
#endif
    { "if", cmd_if,    CMD_WAIT_MUTEX | CMD_RUN_IN_LOAD },
#ifdef TINYSA4
    { "if1", cmd_if1,    CMD_RUN_IN_LOAD },
    { "lna2", cmd_lna2,    CMD_RUN_IN_LOAD },
    { "agc", cmd_agc,    CMD_RUN_IN_LOAD },
#endif
    { "actual_freq", cmd_actual_freq, CMD_WAIT_MUTEX | CMD_RUN_IN_LOAD },
#ifdef TINYSA4
    { "freq_corr", cmd_freq_correction, CMD_WAIT_MUTEX | CMD_RUN_IN_LOAD },
#endif
    { "attenuate", cmd_attenuate,    CMD_WAIT_MUTEX | CMD_RUN_IN_LOAD },
    { "level", cmd_level,    CMD_WAIT_MUTEX | CMD_RUN_IN_LOAD },
    { "sweeptime", cmd_sweeptime,    CMD_WAIT_MUTEX | CMD_RUN_IN_LOAD },
    { "leveloffset", cmd_leveloffset,    CMD_RUN_IN_LOAD },
    { "levelchange", cmd_levelchange,    CMD_RUN_IN_LOAD },
    { "modulation", cmd_modulation,    CMD_RUN_IN_LOAD },
    { "rbw", cmd_rbw,    CMD_WAIT_MUTEX | CMD_RUN_IN_LOAD },
    { "mode", cmd_mode,    CMD_WAIT_MUTEX | CMD_RUN_IN_LOAD},
#ifdef __SPUR__
    { "spur", cmd_spur,    CMD_WAIT_MUTEX | CMD_RUN_IN_LOAD },
#endif
#ifdef TINYSA4
    { "lna", cmd_lna,    CMD_WAIT_MUTEX | CMD_RUN_IN_LOAD },
    { "direct", cmd_direct, CMD_WAIT_MUTEX | CMD_RUN_IN_LOAD },
#endif
#ifdef __ULTRA__
    { "ultra", cmd_ultra,    CMD_WAIT_MUTEX | CMD_RUN_IN_LOAD },
//    { "ultra_start", cmd_ultra_start, CMD_WAIT_MUTEX | CMD_RUN_IN_LOAD },
#endif
    { "load", cmd_load,   CMD_RUN_IN_LOAD },
    { "ext_gain", cmd_ext_gain, CMD_RUN_IN_LOAD},
    { "output", cmd_output,    CMD_RUN_IN_LOAD },
    { "deviceid", cmd_deviceid,    CMD_RUN_IN_LOAD },
    { "selftest", cmd_selftest,    0 },
    { "correction", cmd_correction,   CMD_RUN_IN_LOAD },
    { "calc", cmd_calc, CMD_WAIT_MUTEX | CMD_RUN_IN_LOAD},
#ifdef ENABLE_SD_CARD_CMD
    { "sd_list",   cmd_sd_list,   CMD_WAIT_MUTEX },
    { "sd_read",   cmd_sd_read,   CMD_WAIT_MUTEX },
    { "sd_delete", cmd_sd_delete, CMD_WAIT_MUTEX },
#endif
#ifdef ENABLE_THREADS_COMMAND
    {"threads"     , cmd_threads     , 0},
#endif
#ifdef __SINGLE_LETTER__
   { "y", cmd_y,    CMD_WAIT_MUTEX },
   { "i", cmd_i,	CMD_WAIT_MUTEX },
   { "v", cmd_v,	CMD_WAIT_MUTEX },
   { "a", cmd_a,	CMD_WAIT_MUTEX },
   { "b", cmd_b,	CMD_WAIT_MUTEX },
   { "t", cmd_t,    CMD_WAIT_MUTEX },
#ifdef TINYSA4
   { "k", cmd_k,    CMD_WAIT_MUTEX },
#endif
   { "e", cmd_e,	CMD_WAIT_MUTEX },
   { "s", cmd_s,	CMD_WAIT_MUTEX },
   { "m", cmd_m,	0 },
   { "p", cmd_p,    CMD_WAIT_MUTEX },
   { "w", cmd_w,	CMD_WAIT_MUTEX },
   { "o", cmd_o,    CMD_WAIT_MUTEX },
   { "d", cmd_d,    CMD_WAIT_MUTEX },
   { "f", cmd_f,    CMD_WAIT_MUTEX },
   { "u", cmd_u,    CMD_WAIT_MUTEX },
#endif
#ifdef TINYSA4
   { "q", cmd_q,    CMD_WAIT_MUTEX },
   { "g", cmd_g,    CMD_WAIT_MUTEX },
   { "n", cmd_n,    CMD_WAIT_MUTEX },
   { "z", cmd_z,    CMD_WAIT_MUTEX },
   { "r", cmd_r,    CMD_WAIT_MUTEX },
   { "c", cmd_c,    CMD_WAIT_MUTEX },
   { "h", cmd_h,    CMD_WAIT_MUTEX },
#endif
#ifdef __ADF4351__
    { "x", cmd_x,    CMD_WAIT_MUTEX },
#endif
    {NULL          , NULL            , 0}
};

VNA_SHELL_FUNCTION(cmd_help)
{
  (void)argc;
  (void)argv;
  const VNAShellCommand *scp = commands;
//#ifdef TINYSA4
  shell_printf("commands:");
//#endif
  while (scp->sc_name != NULL
#ifdef __SINGLE_LETTER__
      && scp->sc_function != cmd_y
#endif
      )   {
#ifdef TINYSA4
    if (scp->flags & CMD_RUN_IN_LOAD)
#endif
      shell_printf(" %s", scp->sc_name);
    scp++;
  }
#ifdef TINYSA4
  scp = commands;
  shell_printf("\r\nOther commands:");
  while (scp->sc_name != NULL
#ifdef __SINGLE_LETTER__
      && scp->sc_function != cmd_y
#endif
      )   {
    if (!(scp->flags & CMD_RUN_IN_LOAD))
    shell_printf(" %s", scp->sc_name);
    scp++;
  }
#endif
  shell_printf("\r\n");
  return;
}

/*
 * VNA shell functions
 */
// Check USB connection status
static bool usb_IsActive(void){
  return usbGetDriverStateI(&USBD1) == USB_ACTIVE;
}

// Check active connection for Shell
static bool shell_check_connect(void){
#ifdef __USE_SERIAL_CONSOLE__
  // Serial connection always active
  if (config._mode & _MODE_SERIAL)
    return true;
#endif
  // USB connection can be USB_SUSPENDED
  return usb_IsActive();
}

// Check Serial connection requirements
#ifdef __USE_SERIAL_CONSOLE__
#if HAL_USE_SERIAL == FALSE
#error "For serial console need HAL_USE_SERIAL as TRUE in halconf.h"
#endif

// Before start process command from shell, need select input stream
#define PREPARE_STREAM shell_stream = (config._mode&_MODE_SERIAL) ? (BaseSequentialStream *)&SD1 : (BaseSequentialStream *)&SDU1;

// Update Serial connection speed and settings
void shell_update_speed(void){
  // Update Serial speed settings
  SerialConfig s_config = {config._serial_speed, 0, USART_CR2_STOP1_BITS, 0 };
  sdStop(&SD1);
  sdStart(&SD1, &s_config);  // USART config
}

void shell_reset_console(void){
  // Reset I/O queue over USB (for USB need also connect/disconnect)
  if (usb_IsActive()){
    if (config._mode & _MODE_SERIAL)
      sduDisconnectI(&SDU1);
    else
      sduConfigureHookI(&SDU1);
  }
  // Reset I/O queue over Serial
  oqResetI(&SD1.oqueue);
  iqResetI(&SD1.iqueue);
//  qResetI(&SD1.oqueue);
//  qResetI(&SD1.iqueue);

}


static void shell_init_connection(void) {
/*
 * Init shell thread object (need for switch threads)
 */
  osalThreadQueueObjectInit(&shell_thread);
/*
 * Initializes and start serial-over-USB CDC driver SDU1, connected to USBD1
 */
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);

/*
 * Set Serial speed settings for SD1
 */
  shell_update_speed();

/*
 * Activates the USB driver and then the USB bus pull-up on D+.
 * Note, a delay is inserted in order to not have to disconnect the cable
 * after a reset.
 */
  usbDisconnectBus(&USBD1);
  chThdSleepMilliseconds(100);
  usbStart(&USBD1, &usbcfg);
  usbConnectBus(&USBD1);

/*
 *  Set I/O stream (SDU1 or SD1) for shell
 */
  PREPARE_STREAM;
}

#else
// Only USB console, shell_stream always on USB
#define PREPARE_STREAM shell_stream = (BaseSequentialStream *)&SDU1;

#if 0           // Not used
// Check connection as Active, if no suspend input
static bool shell_check_connect(void){
  return SDU1.config->usbp->state == USB_ACTIVE;
}
#endif

// Init shell I/O connection over USB
static void shell_init_connection(void){
/*
 * Initializes and start serial-over-USB CDC driver SDU1, connected to USBD1
 */
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);

/*
 * Activates the USB driver and then the USB bus pull-up on D+.
 * Note, a delay is inserted in order to not have to disconnect the cable
 * after a reset.
 */
  usbDisconnectBus(&USBD1);
  chThdSleepMilliseconds(100);
  usbStart(&USBD1, &usbcfg);
  usbConnectBus(&USBD1);

/*
 *  Set I/O stream SDU1 for shell
 */
  PREPARE_STREAM;
}
#endif

bool global_abort = false;

static inline char* vna_strpbrk(char *s1, const char *s2) {
  do {
    const char *s = s2;
    do {
      if (*s == *s1) return s1;
      s++;
    } while (*s);
    s1++;
  } while(*s1);
  return s1;
}

/*
 * Split line by arguments, return arguments count
 */
int parse_line(char *line, char* args[], int max_cnt) {
  char *lp = line, c;
  const char *brk;
  uint16_t nargs = 0;
  while ((c = *lp) != 0) {                   // While not end
    if (c != ' ' && c != '\t') {             // Skipping white space and tabs.
      if (c == '"') {lp++; brk = "\""; }     // string end is next quote or end
      else          {      brk = " \t";}     // string end is tab or space or end
      if (nargs < max_cnt) args[nargs] = lp; // Put pointer in args buffer (if possible)
      nargs++;                               // Substring count
      lp = vna_strpbrk(lp, brk);             // search end
      if (*lp == 0) break;                   // Stop, end of input string
      *lp = 0;                               // Set zero at the end of substring
    }
    lp++;
  }
  return nargs;
}

static const VNAShellCommand *VNAShell_parceLine(char *line){
  // Parse and execute line
  shell_nargs = parse_line(line, shell_args, ARRAY_COUNT(shell_args));
  if (shell_nargs > ARRAY_COUNT(shell_args)) {
    shell_printf("too many arguments, max " define_to_STR(VNA_SHELL_MAX_ARGUMENTS) "" VNA_SHELL_NEWLINE_STR);
    return NULL;
  }
  if (shell_nargs > 0) {
    const VNAShellCommand *scp;
    for (scp = commands; scp->sc_name != NULL; scp++)
      if (get_str_index(scp->sc_name, shell_args[0]) == 0)
        return scp;
  }
  return NULL;
}

//
// Read command line from shell_stream
//
static int VNAShell_readLine(char *line, int max_size)
{
  // send backspace, space for erase, backspace again
  char backspace[] = {0x08, 0x20, 0x08, 0x00};
  uint8_t c;
  // Prepare I/O for shell_stream
  PREPARE_STREAM;
  uint16_t j = 0;
  // Return 0 only if stream not active
  while (streamRead(shell_stream, &c, 1)) {
    // Backspace or Delete
    if (c == 0x08 || c == 0x7f) {
      if (j > 0) {shell_printf(backspace); j--;}
      continue;
    }
    // New line (Enter)
    if (c == '\r') {
      shell_printf(VNA_SHELL_NEWLINE_STR);
      line[j] = 0;
      return 1;
    }
    // Others (skip) or too long - skip
    if (c < ' ' || j >= max_size - 1) continue;
    streamPut(shell_stream, c); // Echo
    line[j++] = (char)c;
  }
  return 0;
}

//
// Parse and run command line
//
static void VNAShell_executeLine(char *line)
{
  // Execute line
  const VNAShellCommand *scp = VNAShell_parceLine(line);
  if (scp) {
    uint16_t cmd_flag = scp->flags;
    // Skip wait mutex if process UI
    if ((cmd_flag & CMD_RUN_IN_UI) && (sweep_mode&SWEEP_UI_MODE)) cmd_flag&=~CMD_WAIT_MUTEX;
    if (cmd_flag & CMD_WAIT_MUTEX) {
      shell_function = scp->sc_function;
      operation_requested|=OP_CONSOLE;      // this will abort current sweep to give priority to the new request
      // Wait execute command in sweep thread
      do {
        osalThreadEnqueueTimeoutS(&shell_thread, TIME_INFINITE);
      } while (shell_function);
    } else {
      operation_requested = false; // otherwise commands  will be aborted
      scp->sc_function(shell_nargs - 1, &shell_args[1]);
      if (dirty) {
        operation_requested = true;   // ensure output is updated
        if (MODE_OUTPUT(setting.mode))
          draw_menu();    // update screen if in output mode and dirty
        else
          redraw_request |= REDRAW_CAL_STATUS | REDRAW_AREA | REDRAW_FREQUENCY;
      }
    }
    return;
  }
  shell_printf("%s?" VNA_SHELL_NEWLINE_STR, shell_args[0]);
}

void shell_executeCMDLine(char *line) {
  // Disable shell output (not allow shell_printf write, but not block other output!!)
  shell_stream = NULL;
  const VNAShellCommand *scp = VNAShell_parceLine(line);
  if (scp && (scp->flags & CMD_RUN_IN_LOAD))
    scp->sc_function(shell_nargs - 1, &shell_args[1]);
  PREPARE_STREAM;
}

#ifdef __SD_CARD_LOAD__
#ifndef __USE_SD_CARD__
#error "Need enable SD card support __USE_SD_CARD__ in nanovna.h, for use ENABLE_SD_CARD_CMD"
#endif
void sd_card_load_config(char *filename){
  // Mount card
  if (f_mount(fs_volume, "", 1) != FR_OK)
    return;

  if (f_open(fs_file, filename, FA_OPEN_EXISTING | FA_READ) != FR_OK)
    return;
  // Reset IO stream
  shell_stream = NULL;
  char *buf = (char *)spi_buffer;
  UINT size = 0;

  uint16_t j = 0, i;
  while (f_read(fs_file, buf, 512, &size) == FR_OK && size > 0){
    i = 0;
    while (i < size) {
      uint8_t c = buf[i++];
      // New line (Enter)
      if (c == '\r') {
//        shell_line[j  ] = '\r';
//        shell_line[j+1] = '\n';
//        shell_line[j+2] = 0;
//        shell_printf(shell_line);
        shell_line[j] = 0; j = 0;
        const VNAShellCommand *scp = VNAShell_parceLine(shell_line);
        if (scp && (scp->flags&CMD_RUN_IN_LOAD))
          scp->sc_function(shell_nargs - 1, &shell_args[1]);
        continue;
      }
      // Others (skip)
      if (c < 0x20) continue;
      // Store
      if (j < VNA_SHELL_MAX_LENGTH - 1)
        shell_line[j++] = (char)c;
    }
  }
  f_close(fs_file);
  // Prepare I/O for shell_stream
  PREPARE_STREAM;
  return;
}
#endif

#ifdef VNA_SHELL_THREAD
static THD_WORKING_AREA(waThread2, /* cmd_* max stack size + alpha */442);
THD_FUNCTION(myshellThread, p)
{
  (void)p;
  chRegSetThreadName("shell");
  shell_printf(VNA_SHELL_NEWLINE_STR"tinySA Shell"VNA_SHELL_NEWLINE_STR);
  while (true) {
    shell_printf(VNA_SHELL_PROMPT_STR);
    if (VNAShell_readLine(shell_line, VNA_SHELL_MAX_LENGTH))
      VNAShell_executeLine(shell_line);
    else // Putting a delay in order to avoid an endless loop trying to read an unavailable stream.
      osalThreadSleepMilliseconds(100);
  }
}
#endif

#pragma GCC pop_options

static const GPTConfig gpt4cfg = {
  8000000, // 8 MHz timer clock.
  NULL, // No callback
  0, 0
};

#ifdef TINYSA4
#define DELAY_TIMER GPTD4
#else
#define DELAY_TIMER GPTD14
#endif

void my_microsecond_delay(int t)
{
  while (t >= 0x1000) { // 16 bit timer
    gptPolledDelay(&DELAY_TIMER, 0x1000<<3); // t us delay
    t -= 0x1000;
  }
  if (t>1) gptPolledDelay(&DELAY_TIMER, t<<3); // t us delay
}

void my_veryfast_delay(int t) // In 8MHz ticks
{
  if (t>0) gptPolledDelay(&DELAY_TIMER, t);
}

/* Main thread stack size defined in makefile USE_PROCESS_STACKSIZE = 0x200
 * Profile stack usage (enable threads command by def ENABLE_THREADS_COMMAND) show:
 *Stack maximum usage = 472 bytes (need test more and run all commands), free stack = 40 bytes
 */
static void dac_init(void){
  rccEnableDAC1(false); // Enable DAC1
}

#ifdef TINYSA4__
#undef PULSE
#define PULSE {   palClearPad(GPIOB, 14); my_microsecond_delay(2); palSetPad(GPIOB, 14); }
#else
#undef PULSE
#define PULSE
#endif

int main(void)
{
  halInit();
  /*
   *  Initiate 1/8 micro second timer
   */
  gptStart(&DELAY_TIMER, &gpt4cfg);
  gptPolledDelay(&DELAY_TIMER, 80); // 10 us delay

  PULSE
  chSysInit();
  PULSE
 /*
  * Initialize RTC library (not used ChibiOS RTC module)
  */
  #ifdef __USE_RTC__
    rtc_init();
  #endif
  PULSE

  //palSetPadMode(GPIOB, 8, PAL_MODE_ALTERNATE(1) | PAL_STM32_OTYPE_OPENDRAIN);
  //palSetPadMode(GPIOB, 9, PAL_MODE_ALTERNATE(1) | PAL_STM32_OTYPE_OPENDRAIN);
#ifdef __VNA__
  i2cStart(&I2CD1, &i2ccfg);
  si5351_init();
#endif

#ifdef TINYSA3
  has_esd = ((palReadPort(GPIOB) & (1<<12)) ? false : true );
  bool has_new_switch = ((palReadPort(GPIOA) & (1<<5)) ? false : true ) || ((palReadPort(GPIOB) & (1<<12)) ? false : true );
#endif


#ifdef __SI4432__
 /*
  * Powercycle the RF part to reset SI4432
  */

#if 0
  palClearPad(GPIOB, GPIOA_RF_PWR);
  chThdSleepMilliseconds(200);
#endif
  palSetPad(GPIOB, GPIO_RF_PWR);
  chThdSleepMilliseconds(500);
#endif

#if 0
  palSetPadMode(GPIOA, 9, PAL_MODE_INPUT_ANALOG);
  palSetPadMode(GPIOA, 10, PAL_MODE_OUTPUT_PUSHPULL);
  int s;

  adc_stop();

  // drive high to low on Y line (coordinates from left to right)
  palSetPad(GPIOB, GPIOB_YN);
  palClearPad(GPIOA, GPIOA_YP);
  // Set Y line as output
  palSetPadMode(GPIOB, GPIOB_YN, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOA, GPIOA_YP, PAL_MODE_OUTPUT_PUSHPULL);
  // Set X line as input
  palSetPadMode(GPIOB, GPIOB_XN, PAL_MODE_INPUT);        // Hi-z mode
  palSetPadMode(GPIOA, GPIOA_XP, PAL_MODE_INPUT_ANALOG); // <- ADC_TOUCH_X channel


  while (1) {
//      palSetPad(GPIOA, 10);
//      shell_printf("%d\n\r", adc_single_read(ADC_CHSELR_CHSEL9));
//      palClearPad(GPIOA, 10);
      shell_printf("%d\n\r", adc_single_read(ADC_TOUCH_X));
  }
#endif

  spi_init();
  PULSE


  /*
   * Set LCD display brightness (use DAC2 for control)
   * Starting DAC1 driver, setting up the output pin as analog as suggested by the Reference Manual.
   */
  dac_init();
  PULSE
  DAC->CR|= DAC_CR_EN1 | DAC_CR_EN2; // Use DAC: CH1 and CH2
  #ifdef  __LCD_BRIGHTNESS__
    lcd_setBrightness(DEFAULT_BRIGHTNESS);
  #endif
  DAC->DHR12R1 = 0;                  // Setup DAC: CH1 value

  /*
 * SPI LCD Initialize
 */
  ili9341_init();
  PULSE

#ifdef TINYSA4
  ili9341_set_foreground(LCD_FG_COLOR);
  PULSE
  ili9341_drawstring_7x13("Starting...", 0, 0);
  PULSE
#ifdef __DISABLE_HOT_INSERT__
  sd_card_inserted_at_boot = SD_Inserted();
#endif
  disk_initialize(0);
  PULSE
//  SD_PowerOn();
#endif


/*
 *  Initiate 1 micro second timer
 */
 gptStart(&DELAY_TIMER, &gpt4cfg);
 gptPolledDelay(&DELAY_TIMER, 80); // 10 us delay

/* restore config */
#ifdef TINYSA3
  if (has_new_switch)
    config.switch_offset = -5.0;
#endif
  if(config_recall()) {
    clear_backup();
  }
  config.cor_am = 0;        // Should be removed from config
  config.cor_nfm = 0;
  config.cor_wfm = 0;
  ili9341_flip(config.flip);

  if (caldata_recall(0) == -1) {
    load_LCD_properties();
  }
#ifdef __ULTRA__
  else
    ultra_start = (config.ultra_start == ULTRA_AUTO ? DEFAULT_ULTRA_THRESHOLD : config.ultra_start);
  // zzzzz
  config.ultra = true;
  setting.lna = 1;
#endif


/*
 * Init Shell console connection data (after load config for settings)
 */

  shell_init_connection();

  set_sweep_points(POINTS_COUNT);

  #ifdef __AUDIO__
/*
 * I2S Initialize
 */
  tlv320aic3204_init();
  i2sInit();
  i2sObjectInit(&I2SD2);
  i2sStart(&I2SD2, &i2sconfig);
  i2sStartExchange(&I2SD2);
#endif

  setup_sa();
//  if (setting.mode != -1) {
//    menu_mode_cb(setting.mode,0);
//  }
  redraw_frame();
#ifdef TINYSA3
  set_mode(M_HIGH);
  set_sweep_frequency(ST_STOP, (freq_t) 30000000);
  sweep(false);
  osalThreadSleepMilliseconds(100);

  set_mode(M_LOW);
  set_sweep_frequency(ST_STOP, (freq_t) 4000000);
  sweep(false);
#endif

  if (caldata_recall(0) == -1) {
    load_LCD_properties();
  }
  ui_mode_normal();
  if (!(config._mode & _MODE_DONT_SAVE_STATE)) {
    backup_t b;
    uint32_t *f = &backup;
    uint32_t *t = (uint32_t *)&b;
    int i = USED_BACKUP_SIZE;
    while (i--)
      *t++ = *f++;
#ifdef TINYSA4       // Set mode not working reliably
    set_mode(b.mode);
    switch (b.mode) {
    case M_LOW:
    case M_HIGH:
      break;
    case M_GENLOW:
      menu_push_submenu(menu_mode);
      menu_push_submenu(menu_lowoutputmode);
      break;
    case M_GENHIGH:
      menu_push_submenu(menu_mode);
      menu_push_submenu(menu_highoutputmode);
      break;
    }
#endif
    if (b.frequency0 != 0 || b.frequency1 != 0) {
    if (b.mode <= M_HIGH){
      set_sweep_frequency(ST_START, b.frequency0);
      set_sweep_frequency(ST_STOP, b.frequency1);
    } else {
      set_sweep_frequency(ST_CW, (b.frequency0 + b.frequency1)/2);
      set_sweep_frequency(ST_SPAN, (b.frequency1 - b.frequency0));
      ui_mode_menu();
    }
    if (b.attenuation == 0)
      set_auto_attenuation();
    else {
      set_attenuation((b.attenuation - 1)/2.0);
    }
    if (b.reflevel == 0)
      set_auto_reflevel(true);
    else {
      set_auto_reflevel(false);
      user_set_reflevel((float)(b.reflevel-140));
    }
    if (b.RBW == 0)
      setting.rbw_x10 = 0;
    else {
      set_RBW(force_rbw(b.RBW-1));
    }
  }
  }
  set_refer_output(-1);
//  ui_mode_menu();       // Show menu when autostarting mode

  /*
   * Set LCD display brightness (use DAC2 for control)
   * Starting DAC1 driver, setting up the output pin as analog as suggested by the Reference Manual.
   */
   #ifdef  __LCD_BRIGHTNESS__
    lcd_setBrightness(config._brightness);
  #else
    DAC->DHR12R2 = config.dac_value; // Setup DAC: CH2 value
  #endif

  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO-1, Thread1, NULL);

  while (1) {
//    if (SDU1.config->usbp->state == USB_ACTIVE) {
    if (shell_check_connect()) {
#ifdef VNA_SHELL_THREAD
#if CH_CFG_USE_WAITEXIT == FALSE
#error "VNA_SHELL_THREAD use chThdWait, need enable CH_CFG_USE_WAITEXIT in chconf.h"
#endif
      thread_t *shelltp = chThdCreateStatic(waThread2, sizeof(waThread2),
                                            NORMALPRIO + 1,
                                            myshellThread, NULL);
      chThdWait(shelltp);
#else
      shell_printf(VNA_SHELL_NEWLINE_STR"tinySA Shell"VNA_SHELL_NEWLINE_STR);
      do {
        shell_printf(VNA_SHELL_PROMPT_STR);
        if (VNAShell_readLine(shell_line, VNA_SHELL_MAX_LENGTH))
          VNAShell_executeLine(shell_line);
        else
          chThdSleepMilliseconds(200);
//      } while (SDU1.config->usbp->state == USB_ACTIVE);
      } while (shell_check_connect());
#endif
    }
    chThdSleepMilliseconds(1000);
  }
}

/* The prototype shows it is a naked function - in effect this is just an
assembly function. */
void HardFault_Handler(void);

void hard_fault_handler_c(uint32_t *sp) __attribute__((naked));


void HardFault_Handler(void)
{
  uint32_t *sp;
  //__asm volatile ("mrs %0, msp \n\t": "=r" (sp) );
  __asm volatile("mrs %0, psp \n\t" : "=r"(sp));
  hard_fault_handler_c(sp);
}

void hard_fault_handler_c(uint32_t *sp)
{
#ifdef TINYSA4
  uint32_t r0  = sp[0];
  uint32_t r1  = sp[1];
  uint32_t r2  = sp[2];
  uint32_t r3  = sp[3];
  register uint32_t  r4 __asm("r4");
  register uint32_t  r5 __asm("r5");
  register uint32_t  r6 __asm("r6");
  register uint32_t  r7 __asm("r7");
  register uint32_t  r8 __asm("r8");
  register uint32_t  r9 __asm("r9");
  register uint32_t r10 __asm("r10");
  register uint32_t r11 __asm("r11");
  uint32_t r12 = sp[4];
  uint32_t lr  = sp[5];
  uint32_t pc  = sp[6];
  uint32_t psr = sp[7];
  int y = 0;
  int x = OFFSETX + 1;
  ili9341_set_background(LCD_BG_COLOR);
  ili9341_set_foreground(LCD_FG_COLOR);
  lcd_printf(x, y+=FONT_STR_HEIGHT, "SP  0x%08x",  (uint32_t)sp);
  lcd_printf(x, y+=FONT_STR_HEIGHT, "R0  0x%08x",  r0);
  lcd_printf(x, y+=FONT_STR_HEIGHT, "R1  0x%08x",  r1);
  lcd_printf(x, y+=FONT_STR_HEIGHT, "R2  0x%08x",  r2);
  lcd_printf(x, y+=FONT_STR_HEIGHT, "R3  0x%08x",  r3);
  lcd_printf(x, y+=FONT_STR_HEIGHT, "R4  0x%08x",  r4);
  lcd_printf(x, y+=FONT_STR_HEIGHT, "R5  0x%08x",  r5);
  lcd_printf(x, y+=FONT_STR_HEIGHT, "R6  0x%08x",  r6);
  lcd_printf(x, y+=FONT_STR_HEIGHT, "R7  0x%08x",  r7);
  lcd_printf(x, y+=FONT_STR_HEIGHT, "R8  0x%08x",  r8);
  lcd_printf(x, y+=FONT_STR_HEIGHT, "R9  0x%08x",  r9);
  lcd_printf(x, y+=FONT_STR_HEIGHT, "R10 0x%08x", r10);
  lcd_printf(x, y+=FONT_STR_HEIGHT, "R11 0x%08x", r11);
  lcd_printf(x, y+=FONT_STR_HEIGHT, "R12 0x%08x", r12);
  lcd_printf(x, y+=FONT_STR_HEIGHT, "LR  0x%08x",  lr);
  lcd_printf(x, y+=FONT_STR_HEIGHT, "PC  0x%08x",  pc);
  lcd_printf(x, y+=FONT_STR_HEIGHT, "PSR 0x%08x", psr);
#ifdef ENABLE_THREADS_COMMAND
  thread_t *tp;
  tp = chRegFirstThread();
  do {
    uint32_t max_stack_use = 0U;
#if (CH_DBG_ENABLE_STACK_CHECK == TRUE) || (CH_CFG_USE_DYNAMIC == TRUE)
    uint32_t stklimit = (uint32_t)tp->wabase;
#if CH_DBG_FILL_THREADS == TRUE
    uint8_t *p = (uint8_t *)tp->wabase; while(p[max_stack_use]==CH_DBG_STACK_FILL_VALUE) max_stack_use++;
#endif
#else
    uint32_t stklimit = 0U;
#endif
    lcd_printf(x, y+=FONT_STR_HEIGHT, "%08x|%08x|%08x|%08x|%4u|%4u|%9s|%12s",
             stklimit, (uint32_t)tp->ctx.sp, max_stack_use, (uint32_t)tp,
             (uint32_t)tp->refs - 1, (uint32_t)tp->prio, states[tp->state],
             tp->name == NULL ? "" : tp->name);
    tp = chRegNextThread(tp);
  } while (tp != NULL);
#endif
  shell_printf("===================================\r\n");
#else
  ili9341_set_background(LCD_BG_COLOR);
  ili9341_set_foreground(LCD_FG_COLOR);
  ili9341_drawstring("FATAL ERROR", OFFSETX, 120);
   (void)sp;
#endif
  while (true) {
  }
}



