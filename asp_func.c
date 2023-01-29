/** 31.12.2022 @file
*  
*/

#include "asp_func.h"
#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "nanovna.h"

static FATFS fatfs;
static FIL g_file;

FRESULT asp_create_file(char *fs_filename)
{
//  shell_printf("S file\r\n");
  FRESULT res = f_mount(&fatfs, "", 1);
  if (res != FR_OK)
    return res;
  res = f_open(&g_file, fs_filename, FA_CREATE_ALWAYS | FA_READ | FA_WRITE);
//  shell_printf("Open %s, = %d\r\n", fs_filename, res);
  return res;
}

void asp_save_file() 
{
  static int opened = 0;
  uint16_t *buf_16;
  int i, y;
  UINT size;
  char fs_filename[FF_LFN_BUF];
#ifdef __DISABLE_HOT_INSERT__
  if (!sd_card_inserted_at_boot) {
    drawMessageBox("Warning:", "Restart tinySA to use SD card", 2000);
    return;
  }
#endif

  // Prepare filename and open for write
  plot_printf(fs_filename, FF_LFN_BUF, "%08x.%s", rtc_get_FAT(), "asp");

//  UINT total_size = 0;
//  systime_t time = chVTGetSystemTimeX();
  // Prepare filename = .bmp / .csv and open for write
  FRESULT res = FR_OK;
  if (!opened)
  {
    res = asp_create_file(fs_filename);
    if (res != FR_OK) 
    {
      drawMessageBox("SD CARD CAN'T OPEN", fs_filename , 2000);
    }
  }
  
  if (res == FR_OK) 
  {
        opened = 1;
        
//        for (i = 0; i < sweep_points && res == FR_OK; i++) {
//            res = f_write(&g_file, (char *)&measured[TRACE_ACTUAL][i], sizeof(float), &size);

//          char *buf = (char *)spi_buffer;
//           buf += plot_printf(buf, 100, "%f\r\n", value(measured[TRACE_ACTUAL][i]));
//          res = f_write(fs_file, (char *)spi_buffer, buf - (char *)spi_buffer, &size);

//            }

            res = f_write(&g_file, (char *)measured[TRACE_ACTUAL], sweep_points*sizeof(float), &size);
            f_sync(&g_file);
          
//        for (i = 0; i < sweep_points && res == FR_OK; i++) {
//          char *buf = (char *)spi_buffer;
//           buf += plot_printf(buf, 100, "%U, ", getFrequency(i));
//           buf += plot_printf(buf, 100, "%f ", value(measured[TRACE_ACTUAL][i]));
//           buf += plot_printf(buf, 100, "%f ", value(measured[TRACE_STORED][i]));
//           buf += plot_printf(buf, 100, "%f ", value(measured[TRACE_STORED2][i]));
//           buf += plot_printf(buf, 100, "%f", value(measured[TRACE_TEMP][i]));
//          buf += plot_printf(buf, 100, "\r\n");
//          res = f_write(fs_file, (char *)spi_buffer, buf - (char *)spi_buffer, &size);
//        }
        
//    f_close(fs_file);
  }
#if 0
  plot_printf(fs_filename, FF_LFN_BUF, "%d %d %d", i, size, res);

  if (res != FR_OK)
//      drawMessageBox("SD CARD SAVE", res == FR_OK ? fs_filename : "  Fail write  ", 2000);
      drawMessageBox("SD CARD SAVE", fs_filename , 2000);
  else
     drawMessageBox("SD CARD SAVE", res == FR_OK ? fs_filename : "  Fail write  ", 2000);
#endif
//  redraw_request|= REDRAW_AREA|REDRAW_FREQUENCY;
//  ui_mode_normal();
}




// Set type for y resolution
#if LCD_HEIGHT < 256
typedef uint8_t   index_y_t;
#else
typedef uint16_t  index_y_t;
#endif




//
// Optimized by speed/size array processing of value(const float v) function
// on screen need calculate as:
// y = (ref-v)/scale
// and align by top/bottom
static void
trace_into_index_y_array(index_y_t *y, float *array, int points)
{
  float scale = GRIDY / get_trace_scale();
  float ref   = get_trace_refpos();
  float mult = 0, vmult = 1.0;
  float ref_shift = 0;
  switch (setting.unit){
  case U_DBM: break;
#ifdef TINYSA4
  case U_RAW: break;
#endif
    case U_DBMV: ref_shift = 30.0 + 20.0*log10f(sqrtf(50.0));break;
    case U_DBUV: ref_shift = 90.0 + 20.0*log10f(sqrtf(50.0));break;
    case U_VOLT: vmult = powf(10, -30.0/20.0) * sqrtf(50.0); mult = logf(10.0)/20.0;break;
    case U_WATT: vmult = 1.0/1000.0;                         mult = logf(10.0)/10.0;break;
    default:
    return;
  }
  // Universal formula look like this:
  // v = (refpos - (mult ? expf(value*mult) : value) - ref_shift) * vmult) * scale;
  // v = ((refpos - ref_shift) * scale) - (mult ? expf(value*mult) : value) * (vmult * scale)
  // Made precalculated constants:
  ref = (ref - ref_shift) * scale + 0.5; // add 0.5 for floor on int convert
  scale  = scale * vmult;
  int max = NGRIDY * GRIDY, i;
  for (i=0;i<points;i++){
    float value = array[i];
    if (mult) value = fast_expf(value*mult);
    value = ref - value * scale;
    int v = value;
         if (v <   0) v = 0;
    else if (v > max) v = max;
    y[i] = v;
  }
  return;
}

static index_y_t trace_index_y[450];
#define _USE_WATERFALL_PALETTE
#ifdef  _USE_WATERFALL_PALETTE
#include "waterfall.c"
#endif

#   define MAX( x, y )          ( (x)>(y)?(x):(y) )
#   define MIN( x, y )          ( (x)<(y)?(x):(y) )

static void v_minmax(const float *v, int n, float * pmin, float * pmax)
{
    int i;
    float min = v[0];
    float max = v[0];
    for (i = 1; i < n; i++)
    {
        min = MIN(min, v[i]);
        max = MAX(max, v[i]);
    }
    *pmin = min;
    *pmax = max;
}
static float v_sum(const float *v, int n)
{
    int i;
    float s = 0;
    for (i = 0; i < n; i++)
    {
        s += v[i];
    }
    return s;
}
static float v_mean(const float *v, int n)
{
    return v_sum(v, n) / n;
}
static float v_acf1(const float *v, int n)
{
    float m = v_mean(v, n);
    float s2 = 0;
    float s12 = 0;
    int i;
    float a;
    for (i = 1; i < n; i++)
    {
        s12 += (v[i] - m)*(v[i-1] - m);
        s2 += (v[i] - m)*(v[i] - m);
    }
    a = s12/s2;
    a = MAX(a, 0);
    a = MIN(a, 1);
    return a;
}
#   define ABS( x )             ( (x)>=0?(x):-(x) )

static float prob_signal(const float *v, int n, float * ppower)
{
    float mean = v_mean(v, n);
    float min, max, mid;
    v_minmax(v, n, &min, &max);
    mid = (max*1 + min*3)/4;
    *ppower = mean;
    *ppower = mean - min;
    int i, count_big = 0;
    float prob, prob2, sad0 = 0, sad1 = 0;
    for (i = 1; i < n; i++)
    {
        sad0 += ABS(v[i] - mean);
        sad1 += ABS(v[i] - v[i-1]);
        count_big += v[i] > mid;
    }
    prob = 1 - sad1 / sad0;
//    prob = prob*2 - 1;
    prob = prob*2 - 0.5;
    prob = MAX(prob, 0);
    prob = MIN(prob, 1);

    prob2 = (float)count_big / n;
    prob2 = prob2*3 - 1.5;
    prob2 = MAX(prob2, 0);
    prob2 = MIN(prob2, 1);

    return prob*prob2;
}
  /*
static float prob_signal1(const float *array, int n, float * ppower)
{
    int i, count_big = 0;
    float min, max, thr, power_big = 0, p_signal;
    v_minmax(array, n, &min, &max);
    thr = (min*3 + max)/4;
//    thr = (min + max)/2;
    for (i = 0; i < n; i++)
    {
        if (array[i] > thr)
        {
            count_big++;
            power_big += array[i];
        }
    }
    if (count_big)
    {
        power_big /= count_big;
    }
    *ppower = power_big;
    p_signal = (float)count_big/n;
    p_signal = p_signal*2 - 1;
p_signal *= v_acf1(array, n)*2;
    p_signal = MAX(p_signal, 0);
    p_signal = MIN(p_signal, 1);
    return p_signal;
}
*/
static uint16_t color_signal(const float *array, int n, float * ppower_normed, float*pprob_signal, float*ppowewr_dbm)
{
    float power, scale, intensity;
    float p = prob_signal(array, n, &power);
    *ppowewr_dbm = power;
    *pprob_signal = p;;
    uint16_t color, r,g,b;

//    scale = (power + 110) / 50;
    scale = (power + 140) / 40;
    scale = power / 40;
//    prob = prob*2 - 0.5;
    scale = MAX(scale, 0);
    scale = MIN(scale, 1);

//p=1;
    //*ppower_normed = scale;
    *ppower_normed = p;

*ppower_normed = 1;  /// 1

    intensity = scale;

//    scale = 1;
//    p = 1;
//p*=scale;
    r = MAX((intensity - 0.5), 0) *2 * 255;

    r = p*255;
//    g = MIN(intensity, 0.5) *2 * 255;
    g = intensity*255;
    b = intensity*255;


    if (p > 0.5)
    {
        g = b = 0;
    }

//    r *= scale;
//    g *= scale;
//    b *= scale;

    color = RGB565(r,g,b);
    return color;
}

#define MAX_BANDS 40
static uint16_t g_band_color[MAX_BANDS];
static float    g_band_width[MAX_BANDS];
static float    g_band_detection_prob[MAX_BANDS];
static float    g_band_detection_prob_new[MAX_BANDS];
static float    g_band_detection_prob_prev[MAX_BANDS];
static float    g_band_power[MAX_BANDS];
static float    g_band_power_prev[MAX_BANDS];
static int      g_band_mhz[MAX_BANDS];
static int      g_band_count;
static int      g_dac = 0xffff;
static int      g_call_count = 1;
static float    g_smooth = 0.25f;
static int      g_warning_flag;
//static int      g_dac = 0x7fff;

void asp_update_band(int band_mhz)
{
    int i;
    for (i = 0; i < g_band_count; i++)
    {
        if (g_band_mhz[i] == band_mhz) break;
    }
    if (i == g_band_count)
    {
        g_band_mhz[g_band_count++] = band_mhz;
    }
    float prob;
    g_band_color[i] = color_signal(measured[TRACE_ACTUAL], sweep_points, &g_band_width[i], &prob, &g_band_power[i]);

//    g_band_detection_prob[i] += (prob - g_band_detection_prob[i])*g_smooth;
    g_band_detection_prob_new[i] = prob;
}

int v_max_idx(const float * v, int n)
{
    int i, imax = 0;
    for (i = 0; i < n; i++)
    {
        if (v[i] > v[imax])
        {
            imax = i;
        }
    }
    return imax;
}

void asp_update_jump_band()
{
    int i;
    int i_new = v_max_idx(g_band_detection_prob_new, g_band_count);
    int i_prev = v_max_idx(g_band_detection_prob_prev, g_band_count);
    float sum_big = g_band_power[i_new] + g_band_power_prev[i_prev];
    float sum_small = g_band_power[i_prev] + g_band_power_prev[i_new];
    float diff_new = g_band_power[i_new] + g_band_power_prev[i_new];
    float diff_prev = g_band_power_prev[i_prev] - g_band_power[i_prev];
    if (sum_big - sum_small > 0.5 * (diff_new + diff_prev))
    {
        g_band_detection_prob[i_new] = MAX(g_band_detection_prob[i_new], g_band_detection_prob[i_prev]);
    }
    for (i = 0; i < g_band_count; i++)
    {
        g_band_detection_prob[i] += (g_band_detection_prob_new[i] - g_band_detection_prob[i])*g_smooth;
        g_band_detection_prob_prev[i] = g_band_detection_prob_new[i];
        g_band_power_prev[i] = g_band_power[i];
    }
}

void asp_update_waterfall(void)
{
  int i, b, x = 0;
#define NDOTS 8
  int w_width = area_width < WIDTH ? area_width : WIDTH;

  g_call_count++;
  if (g_call_count >= 20)
  {
    g_smooth = 0.05f;
  }
  else
  {
    g_smooth = 1.f/(g_call_count+1);
  }
  asp_update_jump_band();

  for (i = CHART_BOTTOM-1; i >=graph_bottom+1; i--) 
  {	// Scroll down
    ili9341_read_memory(OFFSETX, i, w_width, 1, spi_buffer);
    ili9341_bulk(OFFSETX, i+NDOTS, w_width, 1);
  }

  int k, kmax = 0;
  // print top line
  for (k = 0; k < g_band_count; k++)
  {
    if (g_band_mhz[k] % 10 == 0)
    {
        lcd_printf(OFFSETX + k*w_width/g_band_count, graph_bottom - 15, "%3d", g_band_mhz[k]);
    }
    if (g_band_detection_prob[k] > g_band_detection_prob[kmax])
    {
      kmax = k;
    }
  }
//  lcd_set_font(FONT_BIG);
  char mess[65];
  memset(mess,' ', 64);
  mess[64] = 0;
  
//  ili9341_set_background(LCD_BRIGHT_COLOR_RED);
//  ili9341_clear_screen();
  if (g_band_detection_prob[kmax] > 0.5)
  {
      plot_printf(mess, sizeof(mess), "ATAKA %d MHz BEP.=%d%% YPOBEH=%d dBm    ", g_band_mhz[kmax], (int)(g_band_detection_prob[kmax]*100), (int)(g_band_power[kmax]));
      lcd_setBrightness(0);  // Flash screen, or use DAC->DHR12R2 = 0;
      for (k=0;k<100;k++)    // beep
      {
        DAC->DHR12R1 = g_dac;
        g_dac ^= 0xffff;
        chThdSleepMilliseconds(1);
//        shell_serial_printf("\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\n");
//        shell_serial_printf("\xff\xff\xff\xff\xff\xff\xff\xff\xff\n");
//        shell_serial_printf("\xff\xff\xff\xff\xff");
        shell_serial_printf("\xaa\xaa\xaa\xaa\xaa");
      }
      lcd_setBrightness(100);  // Flash screen
      ili9341_set_foreground(LCD_LOW_BAT_COLOR);
      ili9341_drawstring_10x14(mess, OFFSETX+5, 10);
      ili9341_set_foreground(LCD_BRIGHT_COLOR_GREEN);
      g_warning_flag = 1;
  }
  else
  {
      if (g_warning_flag)
      {
          ili9341_drawstring_10x14(mess, OFFSETX+5, 10);   // clear old message
          lcd_setBrightness(DEFAULT_BRIGHTNESS); 
          g_warning_flag = 0;
      }
      plot_printf(mess, sizeof(mess), "MAKC %d MHz BEP.=%d%% YPOBEH=%d dBm    ", g_band_mhz[kmax], (int)(g_band_detection_prob[kmax]*100), (int)(g_band_power[kmax]));
      ili9341_drawstring_7x13(mess, OFFSETX+5, 10);
  }
//  ili9341_set_background(LCD_BG_COLOR);
//  ili9341_clear_screen();

  
  for (k = 0; k < g_band_count; k++)
  { 
   // lcd_printf(k*WIDTH/g_band_count, graph_bottom - 15, "%3d", g_band_mhz[k]/10);
    int cellw = (k+1)*WIDTH/g_band_count - (k)*WIDTH/g_band_count;

    for (i = 0; i < cellw && x < WIDTH; i++)
    {
      spi_buffer[x++] = i < cellw*g_band_width[k] ? g_band_color[k] : 0;
    }
  }

  for (k = 0; k < WIDTH; k++)
  { 
//    spi_buffer[k] = g_band_color[k*g_band_count/WIDTH];
  }
  for (k = 0; k < NDOTS; k++)
  {
      ili9341_bulk(OFFSETX, graph_bottom+1+k, w_width, 1);
  }
#if 0
  trace_into_index_y_array(trace_index_y, measured[TRACE_ACTUAL], sweep_points);

  index_y_t *index = NULL;
  for (int t=0;t<TRACES_MAX;t++) {                      // Find trace with active measurement
    if (IS_TRACE_ENABLE(t) && setting.average[t] == AV_OFF) {
      index = trace_index_y;
      break;
    }
  }
  if (index == NULL) {
    for (int t=0;t<TRACES_MAX;t++) {                      // Find trace with measurement
      if (IS_TRACE_ENABLE(t)) {
        index = trace_index_y;
        break;
      }
    }
  }
  if (index == NULL)
    return;
  int j = 0;
  for (i=0; i< sweep_points; i++) {			// Add new topline
    uint16_t color;
#ifdef _USE_WATERFALL_PALETTE
    uint16_t y = _PALETTE_ALIGN(256 - graph_bottom + index[i]); // should be always in range 0 - graph_bottom


//    y = (uint8_t)i;  // for test
    color = waterfall_palette[y];
#elif 1
    uint16_t y = index[i]; // should be always in range 0 - graph_bottom
    uint16_t ratio = (graph_bottom - y)*4;
//    ratio = (i*2);    // Uncomment for testing the waterfall colors
    int16_t b = 255 - ratio;
    if (b > 255) b = 255;
    if (b < 0) b = 0;
    int16_t r = ratio - 255;
    if (r > 255) r = 255;
    if (r < 0) r = 0;
    int16_t g = 255 - b - r;
#define gamma_correct(X) X = (X < 64 ? X * 2 : X < 128 ? 128 + (X-64) : X < 192 ? 192 + (X - 128)/2 : 225 + (X - 192) / 4)
    gamma_correct(r);
    gamma_correct(g);
    gamma_correct(b);
    color = RGB565(r, g, b);
#else
    uint16_t y = SMALL_WATERFALL - index[i]* (graph_bottom == BIG_WATERFALL ? 2 : 1); // should be always in range 0 - graph_bottom *2 depends on height of scroll
    // Calculate gradient palette for range 0 .. 192
    // idx     r   g   b
    //   0 - 127   0   0
    //  32 - 255 127   0
    //  64 - 255 255 127
    //  96 - 255 255 255
    // 128 - 127 255 255
    // 160 -   0 127 255
    // 192 -   0   0 127
    // 224 -   0   0   0
//    y = (uint8_t)i;  // for test
         if (y <  32) color = RGB565( 127+((y-  0)*4),   0+((y-  0)*4),               0);
    else if (y <  64) color = RGB565(             255, 127+((y- 32)*4),   0+((y- 32)*4));
    else if (y <  96) color = RGB565(             255,             255, 127+((y- 64)*4));
    else if (y < 128) color = RGB565( 252-((y- 96)*4),             255,             255);
    else if (y < 160) color = RGB565( 124-((y-128)*4), 252-((y-128)*4),             255);
    else              color = RGB565(               0, 124-((y-160)*4), 252-((y-160)*4));

#endif
    while (j * sweep_points  < (i+1) * WIDTH) {   // Scale waterfall to WIDTH points
      spi_buffer[j++] = color;
    }
  }
      int k;
      for (k = 0; k < NDOTS; k++)
      {
          ili9341_bulk(OFFSETX, graph_bottom+1+k, w_width, 1);
      }
//  ili9341_bulk(OFFSETX, graph_bottom+1, w_width, 1);
//  STOP_PROFILE;
#endif
}

#if 0
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


void asp_loop()
{
  set_frequencies(start, stop, sweep_points);
}

#endif