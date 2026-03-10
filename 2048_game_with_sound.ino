/*
 * 2048 for Waveshare ESP32-C6 Touch LCD 1.83"
 *
 * Board : esp32 by Espressif 3.2.0
 * Libs  : lvgl 8.4, Arduino_GFX_Library, Arduino_DriveBus_Library
 *
 * lv_conf.h: LV_COLOR_16_SWAP 0, LV_DPI_DEF 100
 *
 * Audio : ES8311 codec via I2S. Tones synthesised in software (no audio files).
 *         BOOT button (GPIO9) toggles sound on/off.
 */

#include <Arduino.h>
#include <lvgl.h>
#include <Arduino_GFX_Library.h>
#include "Arduino_DriveBus_Library.h"
#include "lv_conf.h"
#include "HWCDC.h"
#include "ESP_I2S.h"
#include "es8311.h"
#include <math.h>

#include <Preferences.h>
Preferences prefs;

HWCDC USBSerial;

// ---------------------------------------------------------------------------
// Display & touch pins
// ---------------------------------------------------------------------------
#define LCD_SCK  1
#define LCD_DIN  2
#define LCD_CS   5
#define LCD_DC   3
#define LCD_RST  4
#define LCD_BL   6
#define IIC_SDA  7
#define IIC_SCL  8
#define TP_INT   11
#define LCD_W    240
#define LCD_H    284

// ---------------------------------------------------------------------------
// Audio pins & config
// ---------------------------------------------------------------------------
#define I2S_MCK_PIN   19
#define I2S_BCK_PIN   20
#define I2S_LRCK_PIN  22
#define I2S_DOUT_PIN  23
#define I2S_DIN_PIN   21
#define PA_CTRL_PIN    0   // power amplifier enable
#define BOOT_PIN       9   // BOOT button (active LOW) — cycles sound level
#define SAMPLE_RATE   12000

// Sound levels: Off, Low, Med, High
static const uint8_t SOUND_LEVELS[]      = { 0, 55, 65, 75 };
static const char*   SOUND_LEVEL_NAMES[] = { "OFF", "LOW", "MED", "HIGH" };
static const int     SOUND_LEVEL_COUNT   = 4;
static int  soundLevelIdx = 1;   // default: Low (55)
#define SOUND_VOLUME (SOUND_LEVELS[soundLevelIdx])

// ---------------------------------------------------------------------------
// Hardware objects
// ---------------------------------------------------------------------------
Arduino_DataBus *bus = new Arduino_HWSPI(LCD_DC, LCD_CS, LCD_SCK, LCD_DIN);
Arduino_GFX    *gfx  = new Arduino_ST7789(bus, LCD_RST, 0, true, LCD_W, LCD_H);

std::shared_ptr<Arduino_IIC_DriveBus> IIC_Bus =
    std::make_shared<Arduino_HWIIC>(IIC_SDA, IIC_SCL, &Wire);
void Arduino_IIC_Touch_Interrupt(void);
std::unique_ptr<Arduino_IIC> tp(new Arduino_CST816x(
    IIC_Bus, CST816T_DEVICE_ADDRESS, -1, TP_INT, Arduino_IIC_Touch_Interrupt));
void Arduino_IIC_Touch_Interrupt(void) { tp->IIC_Interrupt_Flag = true; }

I2SClass i2s;

// ---------------------------------------------------------------------------
// LVGL
// ---------------------------------------------------------------------------
static lv_disp_draw_buf_t draw_buf;
static lv_color_t         line_buf[LCD_W * 20];
static lv_disp_drv_t      disp_drv;
static lv_indev_drv_t     indev_drv;
static int16_t tp_x = 0, tp_y = 0;

void disp_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_p) {
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);
#if (LV_COLOR_16_SWAP != 0)
    gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#else
    gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#endif
    lv_disp_flush_ready(drv);
}

void touch_read(lv_indev_drv_t *drv, lv_indev_data_t *d) {
    int f = (int)tp->IIC_Read_Device_Value(
        Arduino_IIC_Touch::Value_Information::TOUCH_FINGER_NUMBER);
    if (f > 0) {
        tp_x = (int16_t)tp->IIC_Read_Device_Value(
            Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X);
        tp_y = (int16_t)tp->IIC_Read_Device_Value(
            Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y);
        d->point.x = tp_x; d->point.y = tp_y;
        d->state = LV_INDEV_STATE_PR;
    } else {
        d->state = LV_INDEV_STATE_REL;
    }
}

// ---------------------------------------------------------------------------
// Audio synthesis
// ---------------------------------------------------------------------------
static bool lastBootBtn = true;   // HIGH = not pressed

// Write a buffer of int16 mono samples to I2S (duplicated to stereo)
// ---------------------------------------------------------------------------
// Audio synthesis
// ---------------------------------------------------------------------------
// Pre-render all notes for the fanfare into one contiguous buffer and send
// to I2S in a single large write.  This eliminates the inter-note gaps and
// I2S underruns that caused crackling in the per-note approach.
//
// Buffer size: 3 notes * 150ms + 2 gaps * 20ms = 490ms max
// At 24000 Hz stereo int16: 490ms * 24000 * 2 ch * 2 bytes = ~47 KB
// We allocate from PSRAM / heap to avoid stack overflow.
#define FANFARE_MAX_SAMPLES  13500   // 1125ms mono — enough for win fanfare
static int16_t  fanfare_mono[FANFARE_MAX_SAMPLES];
static int16_t  fanfare_stereo[FANFARE_MAX_SAMPLES * 2];

// Write stereo int16 frames directly; large single write minimises underruns
static void i2sWriteStereo(const int16_t *stereo, int frame_count) {
    i2s.write((uint8_t *)stereo, frame_count * 4);
}

// Render a sine note into mono buffer starting at offset, return new offset
static int renderNote(int16_t *buf, int buf_len, int offset,
                      float freq, int dur_ms,
                      int attack_ms, int release_ms) {
    int n        = (SAMPLE_RATE * dur_ms)     / 1000;
    int attack   = (SAMPLE_RATE * attack_ms)  / 1000;
    int rel      = (SAMPLE_RATE * release_ms) / 1000;
    float vol    = (SOUND_VOLUME / 100.0f) * 18000.0f;
    float inc    = 2.0f * M_PI * freq / SAMPLE_RATE;
    float phase  = 0.0f;
    for (int i = 0; i < n && (offset + i) < buf_len; i++) {
        float env;
        if      (i < attack)     env = (float)i / attack;
        else if (i > n - rel)    env = (float)(n - i) / rel;
        else                     env = 1.0f;
        buf[offset + i] = (int16_t)(sinf(phase) * vol * env);
        phase += inc;
    }
    return offset + n;
}

// Render silence into mono buffer
static int renderSilence(int16_t *buf, int buf_len, int offset, int dur_ms) {
    int n = (SAMPLE_RATE * dur_ms) / 1000;
    for (int i = 0; i < n && (offset + i) < buf_len; i++)
        buf[offset + i] = 0;
    return offset + n;
}

// Short "clack" — exponentially decaying tone, rendered and sent in one write
static void playClack() {
    if (SOUND_VOLUME == 0) return;
    const int nsamples = (SAMPLE_RATE * 55) / 1000;   // 55ms
    const float freq   = 900.0f;
    const float vol    = (SOUND_VOLUME / 100.0f) * 14000.0f;
    float phase = 0.0f;
    float inc   = 2.0f * M_PI * freq / SAMPLE_RATE;
    // Render mono into fanfare_mono (reuse buffer - clack never overlaps fanfare)
    for (int i = 0; i < nsamples; i++) {
        float env  = expf(-6.0f * i / nsamples);
        float wave = sinf(phase) * 0.7f + sinf(phase * 2.0f) * 0.3f;
        fanfare_mono[i] = (int16_t)(wave * vol * env);
        phase += inc;
    }
    // Interleave to stereo
    for (int i = 0; i < nsamples; i++) {
        fanfare_stereo[i * 2]     = fanfare_mono[i];
        fanfare_stereo[i * 2 + 1] = fanfare_mono[i];
    }
    i2sWriteStereo(fanfare_stereo, nsamples);
}

// Play the 3-note rising fanfare E->G->C
// levelIdx 0=tile 4, 1=tile 8, 2=tile 16, ... (whole tone per level)
static void playFanfare(int levelIdx) {
    if (SOUND_VOLUME == 0) return;
    const float base[3] = { 329.63f, 392.00f, 523.25f };   // E4, G4, C5
    float shift = powf(2.0f, (levelIdx * 2) / 12.0f);

    // Render all three notes + gaps into one mono buffer
    int pos = 0;
    for (int i = 0; i < 3; i++) {
        pos = renderNote(fanfare_mono, FANFARE_MAX_SAMPLES, pos,
                         base[i] * shift,
                         140,   // note duration ms
                         8,     // attack ms  — short, punchy
                         30);   // release ms — gentle tail
        if (i < 2)
            pos = renderSilence(fanfare_mono, FANFARE_MAX_SAMPLES, pos, 15);
    }

    // Interleave mono -> stereo
    for (int i = 0; i < pos; i++) {
        fanfare_stereo[i * 2]     = fanfare_mono[i];
        fanfare_stereo[i * 2 + 1] = fanfare_mono[i];
    }

    // Single I2S write — no inter-chunk gaps, no underruns
    i2sWriteStereo(fanfare_stereo, pos);
}

// Descending 4-note "wah wah wah wahhh" — played on game over
static void playGameOver() {
    if (SOUND_VOLUME == 0) return;
    // G4, E4, C#4, Bb3 — descending minor thirds, slow and mournful
    const float notes[4] = { 392.00f, 329.63f, 277.18f, 233.08f };
    int pos = 0;
    for (int i = 0; i < 4; i++) {
        // Longer notes with a slow attack and long release for a drooping feel
        pos = renderNote(fanfare_mono, FANFARE_MAX_SAMPLES, pos,
                         notes[i],
                         i < 3 ? 200 : 350,  // last note held longer
                         30,                  // slow attack
                         80);                 // long release — trails off sadly
        if (i < 3)
            pos = renderSilence(fanfare_mono, FANFARE_MAX_SAMPLES, pos, 20);
    }
    for (int i = 0; i < pos; i++) {
        fanfare_stereo[i * 2]     = fanfare_mono[i];
        fanfare_stereo[i * 2 + 1] = fanfare_mono[i];
    }
    i2sWriteStereo(fanfare_stereo, pos);
}

// Ascending 5-note major arpeggio — played on reaching 2048
static void playWin() {
    // Play at full volume regardless of sound setting — you've earned it!
    // C4, E4, G4, C5, E5 — bright triumphant major arpeggio
    const float notes[5] = { 261.63f, 329.63f, 392.00f, 523.25f, 659.25f };
    // Temporarily override volume in the buffer by scaling after render
    float savedVol = SOUND_VOLUME;  // note: SOUND_VOLUME is a macro, scale manually
    float volScale = 75.0f / 100.0f * 18000.0f;  // full volume regardless of setting
    int pos = 0;
    for (int i = 0; i < 5; i++) {
        int dur  = (i == 4) ? 450 : 160;   // last note held long
        int att  = 10;
        int rel  = (i == 4) ? 120 : 40;
        int n    = (SAMPLE_RATE * dur)  / 1000;
        int atts = (SAMPLE_RATE * att)  / 1000;
        int rels = (SAMPLE_RATE * rel)  / 1000;
        float inc = 2.0f * M_PI * notes[i] / SAMPLE_RATE;
        float phase = 0.0f;
        for (int j = 0; j < n && (pos + j) < FANFARE_MAX_SAMPLES; j++) {
            float env;
            if      (j < atts)      env = (float)j / atts;
            else if (j > n - rels)  env = (float)(n - j) / rels;
            else                    env = 1.0f;
            fanfare_mono[pos + j] = (int16_t)(sinf(phase) * volScale * env);
            phase += inc;
        }
        pos += n;
        if (i < 4)
            pos = renderSilence(fanfare_mono, FANFARE_MAX_SAMPLES, pos, 20);
    }
    for (int i = 0; i < pos; i++) {
        fanfare_stereo[i * 2]     = fanfare_mono[i];
        fanfare_stereo[i * 2 + 1] = fanfare_mono[i];
    }
    i2sWriteStereo(fanfare_stereo, pos);
}

// ---------------------------------------------------------------------------
// Game logic
// ---------------------------------------------------------------------------
#define N 4
static int  board[N][N];
static int  score;
static int  hiScore;
static bool won, over;

// Track which tile values have already triggered a fanfare this game
static uint32_t fanfarePlayed;   // bitmask: bit k set if value 2^(k+2) already played

static void spawn() {
    int ex[N*N], ey[N*N], n = 0;
    for (int r = 0; r < N; r++)
        for (int c = 0; c < N; c++)
            if (board[r][c] == 0) { ey[n] = r; ex[n] = c; n++; }
    if (n == 0) return;
    int i = random(0, n);
    board[ey[i]][ex[i]] = (random(0, 10) < 9) ? 2 : 4;
}

static void newGame() {
    memset(board, 0, sizeof(board));
    score = 0; won = false; over = false;
    fanfarePlayed = 0;
    spawn(); spawn();
}

static void newHiScore() {
    if (score > hiScore) {
        hiScore = score;
        prefs.putInt("hiScore", hiScore);
    }
}

static bool slideRow(int a[N]) {
    int tmp[N] = {0}, p = 0;
    for (int i = 0; i < N; i++) if (a[i]) tmp[p++] = a[i];
    for (int i = 0; i < N-1; i++) {
        if (tmp[i] && tmp[i] == tmp[i+1]) {
            tmp[i] *= 2; score += tmp[i];
            if (tmp[i] == 2048) won = true;
            tmp[i+1] = 0;
        }
    }
    int result[N] = {0}; p = 0;
    for (int i = 0; i < N; i++) if (tmp[i]) result[p++] = tmp[i];
    bool moved = false;
    for (int i = 0; i < N; i++) { if (result[i] != a[i]) moved = true; a[i] = result[i]; }
    return moved;
}

static bool doMove(int dir) {
    bool moved = false; int row[N];
    if (dir == 0) {
        for (int r = 0; r < N; r++) {
            for (int c = 0; c < N; c++) row[c] = board[r][c];
            if (slideRow(row)) { moved = true; for (int c = 0; c < N; c++) board[r][c] = row[c]; }
        }
    } else if (dir == 1) {
        for (int r = 0; r < N; r++) {
            for (int c = 0; c < N; c++) row[c] = board[r][N-1-c];
            if (slideRow(row)) { moved = true; for (int c = 0; c < N; c++) board[r][N-1-c] = row[c]; }
        }
    } else if (dir == 2) {
        for (int c = 0; c < N; c++) {
            for (int r = 0; r < N; r++) row[r] = board[r][c];
            if (slideRow(row)) { moved = true; for (int r = 0; r < N; r++) board[r][c] = row[r]; }
        }
    } else {
        for (int c = 0; c < N; c++) {
            for (int r = 0; r < N; r++) row[r] = board[N-1-r][c];
            if (slideRow(row)) { moved = true; for (int r = 0; r < N; r++) board[N-1-r][c] = row[r]; }
        }
    }
    if (moved) spawn();
    return moved;
}

static bool canMove() {
    for (int r = 0; r < N; r++) for (int c = 0; c < N; c++) {
        if (board[r][c] == 0) return true;
        if (c < N-1 && board[r][c] == board[r][c+1]) return true;
        if (r < N-1 && board[r][c] == board[r+1][c]) return true;
    }
    return false;
}

// Find the highest new tile value just created, play fanfare if first time
// Returns levelIdx (0-based) if fanfare needed, else -1
static int checkFanfare() {
    int best = -1;
    for (int r = 0; r < N; r++) for (int c = 0; c < N; c++) {
        int v = board[r][c];
        if (v < 4) continue;
        // levelIdx: 4->0, 8->1, 16->2, ...
        int li = 0; int tmp = v; while (tmp > 4) { tmp >>= 1; li++; }
        uint32_t bit = (1u << li);
        if (!(fanfarePlayed & bit)) {
            if (li > best) best = li;
        }
    }
    if (best >= 0) {
        // Mark all levels up to best as played
        for (int i = 0; i <= best; i++) fanfarePlayed |= (1u << i);
    }
    return best;
}

// ---------------------------------------------------------------------------
// Rendering
// ---------------------------------------------------------------------------
static lv_color_t canvas_buf[LCD_W * LCD_H];
static lv_obj_t  *canvas;

static const uint32_t TILE_BG[12] = {
    0xCDC1B4, 0xEEE4DA, 0xEDE0C8, 0xF2B179, 0xF59563, 0xF67C5F,
    0xF65E3B, 0xEDCF72, 0xEDCC61, 0xEDC850, 0xEDC53F, 0xEDC22E,
};

static lv_color_t hex(uint32_t h) {
    return lv_color_make((h >> 16) & 0xFF, (h >> 8) & 0xFF, h & 0xFF);
}
static int tileIdx(int v) {
    if (!v) return 0; int i = 0; while (v > 1) { v >>= 1; i++; }
    return (i > 11) ? 11 : i;
}
static void canvasRect(int x, int y, int w, int h, int r, lv_color_t col) {
    lv_draw_rect_dsc_t d; lv_draw_rect_dsc_init(&d);
    d.bg_color = col; d.bg_opa = LV_OPA_COVER;
    d.radius = r; d.border_width = 0; d.shadow_width = 0;
    lv_canvas_draw_rect(canvas, x, y, w, h, &d);
}
static void canvasText(const char *s, int x, int y, int w,
                       lv_color_t col, const lv_font_t *font, lv_text_align_t align) {
    lv_draw_label_dsc_t d; lv_draw_label_dsc_init(&d);
    d.color = col; d.font = font; d.align = align;
    lv_canvas_draw_text(canvas, x, y, w, &d, s);
}

#define TILE_SZ  51
#define TILE_GAP  4
#define GRID_X    8
#define GRID_Y   60
#define GRID_PX  (N * TILE_SZ + (N + 1) * TILE_GAP)

static void tileFont(int v, const lv_font_t **font, int *yoff) {
    if (v >= 1000) { *font = &lv_font_montserrat_18; *yoff = 16; }
    else if (v >= 100) { *font = &lv_font_montserrat_22; *yoff = 14; }
    else               { *font = &lv_font_montserrat_26; *yoff = 12; }
}

static void redraw() {
    canvasRect(0, 0, LCD_W, LCD_H, 0, hex(0xFAF8EF));

    // ── Header: 3 equal columns, 60px tall ─────────────────────────────────
    // Left   (x=0..79):   "Top Score" label, hi score number
    // Centre (x=80..159): "Sound: X" label, "2048" title (or New Game button)
    // Right  (x=160..239): "Score" label, current score number

    // Left: hi score
    canvasText("Top Score", 0, 4, 80, hex(0x776E65), &lv_font_montserrat_10, LV_TEXT_ALIGN_CENTER);
    {
        char hbuf[24];
        snprintf(hbuf, sizeof(hbuf), "%d", hiScore);
        const lv_font_t *hi_font;
        int hi_y;
        if      (hiScore < 1000)   { hi_font = &lv_font_montserrat_22; hi_y = 16; }
        else if (hiScore < 10000)  { hi_font = &lv_font_montserrat_18; hi_y = 18; }
        else if (hiScore < 100000) { hi_font = &lv_font_montserrat_14; hi_y = 22; }
        else                       { hi_font = &lv_font_montserrat_12; hi_y = 24; }
        canvasText(hbuf, 0, hi_y, 80, hex(0x776E65), hi_font, LV_TEXT_ALIGN_CENTER);
    }

    // Centre: "Sound: X" label top; "2048" title or New Game button below
    {
        char svol[20];
        snprintf(svol, sizeof(svol), "Sound: %s", SOUND_LEVEL_NAMES[soundLevelIdx]);
        canvasText(svol, 80, 4, 80, hex(0xBBADA0), &lv_font_montserrat_10, LV_TEXT_ALIGN_CENTER);
    }
    if (over || won) {
        // New Game button — flush to top in centre col, rounded bottom corners only
        canvasRect(80, 0, 80, 26, 5, hex(0x8F7A66));
        canvasRect(80, 0, 80, 10, 0, hex(0x8F7A66));   // square top corners
        canvasText("New Game", 80, 6, 80, hex(0xF9F6F2), &lv_font_montserrat_12, LV_TEXT_ALIGN_CENTER);
        canvasText("2048", 80, 28, 80, hex(0xBBADA0), &lv_font_montserrat_28, LV_TEXT_ALIGN_CENTER);
    } else {
        canvasText("2048", 80, 28, 80, hex(0x776E65), &lv_font_montserrat_28, LV_TEXT_ALIGN_CENTER);
    }

    // Right: current score
    canvasText("Score", 160, 4, 80, hex(0x776E65), &lv_font_montserrat_10, LV_TEXT_ALIGN_CENTER);
    {
        char sbuf[24];
        snprintf(sbuf, sizeof(sbuf), "%d", score);
        const lv_font_t *score_font;
        int score_y;
        if      (score < 1000)   { score_font = &lv_font_montserrat_22; score_y = 16; }
        else if (score < 10000)  { score_font = &lv_font_montserrat_18; score_y = 18; }
        else if (score < 100000) { score_font = &lv_font_montserrat_14; score_y = 22; }
        else                     { score_font = &lv_font_montserrat_12; score_y = 24; }
        canvasText(sbuf, 160, score_y, 80, hex(0x776E65), score_font, LV_TEXT_ALIGN_CENTER);
    }

    // Grid
    canvasRect(GRID_X, GRID_Y, GRID_PX, GRID_PX, 6, hex(0xBBADA0));
    for (int r = 0; r < N; r++) {
        for (int c = 0; c < N; c++) {
            int v  = board[r][c];
            int tx = GRID_X + TILE_GAP + c * (TILE_SZ + TILE_GAP);
            int ty = GRID_Y + TILE_GAP + r * (TILE_SZ + TILE_GAP);
            canvasRect(tx, ty, TILE_SZ, TILE_SZ, 5, hex(TILE_BG[tileIdx(v)]));
            if (v > 0) {
                char num[8]; snprintf(num, sizeof(num), "%d", v);
                lv_color_t tcol = (v <= 4) ? hex(0x776E65) : hex(0xF9F6F2);
                const lv_font_t *font; int yoff;
                tileFont(v, &font, &yoff);
                canvasText(num, tx, ty + yoff, TILE_SZ, tcol, font, LV_TEXT_ALIGN_CENTER);
            }
        }
    }
    lv_obj_invalidate(canvas);
}

// ---------------------------------------------------------------------------
// Touch / swipe
// ---------------------------------------------------------------------------
#define SWIPE_MIN 20
static int16_t swipe_x0, swipe_y0;
static bool    swiping = false, wasPressed = false;

static void onTouchUp(int ex, int ey) {
    // New Game button: only active when game has ended
    if ((over || won) &&
        swipe_x0 >= 80 && swipe_x0 <= 160 &&
        swipe_y0 >= 0  && swipe_y0 <= 26) {
        newGame(); redraw(); return;
    }
    int dx = ex - swipe_x0, dy = ey - swipe_y0;
    if (abs(dx) < SWIPE_MIN && abs(dy) < SWIPE_MIN) return;
    if (over && !won) return;
    int dir = (abs(dx) > abs(dy)) ? (dx > 0 ? 1 : 0) : (dy > 0 ? 3 : 2);
    if (doMove(dir)) {
        // Play clack immediately (tile hit edge / merged)
        playClack();
        if (won) {
            // 2048 reached — skip tile fanfare, play the win fanfare instead
            newHiScore();
            playWin();
        } else {
            // Check if a new tile level was reached
            int li = checkFanfare();
            if (li >= 0) playFanfare(li);
        }
        if (!canMove()) { over = true; newHiScore(); playGameOver(); }
        redraw();
    }
}

// ---------------------------------------------------------------------------
// ES8311 init
// ---------------------------------------------------------------------------
static esp_err_t initES8311() {
    es8311_handle_t handle = es8311_create(I2C_NUM_0, ES8311_ADDRRES_0);
    if (!handle) return ESP_FAIL;

    const es8311_clock_config_t clk = {
        .mclk_inverted    = false,
        .sclk_inverted    = false,
        .mclk_from_mclk_pin = true,
        .mclk_frequency   = (uint32_t)(SAMPLE_RATE * 256),
        .sample_frequency = SAMPLE_RATE,
    };
    ESP_ERROR_CHECK(es8311_init(handle, &clk, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16));
    ESP_ERROR_CHECK(es8311_sample_frequency_config(handle,
        SAMPLE_RATE * 256, SAMPLE_RATE));
    ESP_ERROR_CHECK(es8311_voice_volume_set(handle, SOUND_VOLUME, NULL));
    ESP_ERROR_CHECK(es8311_microphone_config(handle, false));
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// Setup & loop
// ---------------------------------------------------------------------------
void setup() {
    USBSerial.begin(115200);

    // Display
    pinMode(LCD_BL, OUTPUT); digitalWrite(LCD_BL, HIGH);
    gfx->begin(); gfx->fillScreen(BLACK);

    // BOOT button for sound toggle
    pinMode(BOOT_PIN, INPUT_PULLUP);

    // Power amplifier (off until audio ready)
    pinMode(PA_CTRL_PIN, OUTPUT); digitalWrite(PA_CTRL_PIN, LOW);

    randomSeed(esp_random());

    // Touch
    while (!tp->begin()) { USBSerial.println("Touch init failed..."); delay(500); }
    tp->IIC_Write_Device_State(
        tp->Arduino_IIC_Touch::Device::TOUCH_DEVICE_INTERRUPT_MODE,
        tp->Arduino_IIC_Touch::Device_Mode::TOUCH_DEVICE_INTERRUPT_PERIODIC);
    USBSerial.println("Touch OK");

    // I2S
    i2s.setPins(I2S_BCK_PIN, I2S_LRCK_PIN, I2S_DOUT_PIN, I2S_DIN_PIN, I2S_MCK_PIN);
    if (!i2s.begin(I2S_MODE_STD, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT,
                   I2S_SLOT_MODE_STEREO, I2S_STD_SLOT_BOTH)) {
        USBSerial.println("I2S init failed!");
    } else {
        USBSerial.println("I2S OK");
    }

    // ES8311 codec
    Wire.begin(IIC_SDA, IIC_SCL);
    if (initES8311() == ESP_OK) {
        digitalWrite(PA_CTRL_PIN, HIGH);   // enable speaker
        USBSerial.println("ES8311 OK");
    } else {
        USBSerial.println("ES8311 init failed!");
    }

    // High score
    prefs.begin("2048", false);  // Open namespace "2048"
    hiScore = prefs.getInt("hiScore", 0);  // Default to 0 if not found

    // LVGL
    lv_init();
    esp_timer_handle_t tt;
    const esp_timer_create_args_t ta = {
        .callback=[](void*){ lv_tick_inc(2); }, .arg=nullptr,
        .dispatch_method=ESP_TIMER_TASK, .name="lv_tick",
        .skip_unhandled_events=true
    };
    esp_timer_create(&ta, &tt);
    esp_timer_start_periodic(tt, 2000);

    lv_disp_draw_buf_init(&draw_buf, line_buf, NULL, LCD_W * 20);
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_W; disp_drv.ver_res = LCD_H;
    disp_drv.flush_cb = disp_flush; disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touch_read;
    lv_indev_drv_register(&indev_drv);

    canvas = lv_canvas_create(lv_scr_act());
    lv_canvas_set_buffer(canvas, canvas_buf, LCD_W, LCD_H, LV_IMG_CF_TRUE_COLOR);
    lv_obj_set_pos(canvas, 0, 0);
    lv_obj_clear_flag(canvas, LV_OBJ_FLAG_SCROLLABLE);

    newGame();
    redraw();
}

void loop() {
    lv_timer_handler();

    // BOOT button: cycle sound level on falling edge (Off->Low->Med->High->Off)
    bool bootBtn = digitalRead(BOOT_PIN);
    if (!bootBtn && lastBootBtn) {
        soundLevelIdx = (soundLevelIdx + 1) % SOUND_LEVEL_COUNT;
        // Update hardware volume immediately
        es8311_handle_t h = es8311_create(I2C_NUM_0, ES8311_ADDRRES_0);
        if (h) es8311_voice_volume_set(h, SOUND_VOLUME, NULL);
        redraw();   // update sound level indicator
    }
    lastBootBtn = bootBtn;

    // Touch swipe
    int  f       = (int)tp->IIC_Read_Device_Value(
                       Arduino_IIC_Touch::Value_Information::TOUCH_FINGER_NUMBER);
    bool pressed = (f > 0);
    if (pressed && !wasPressed) {
        swipe_x0 = (int16_t)tp->IIC_Read_Device_Value(
            Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X);
        swipe_y0 = (int16_t)tp->IIC_Read_Device_Value(
            Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y);
        swiping = true;
    } else if (!pressed && wasPressed && swiping) {
        onTouchUp(tp_x, tp_y);
        swiping = false;
    }
    wasPressed = pressed;
    delay(5);
}
