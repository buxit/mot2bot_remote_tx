#include <Adafruit_LiquidCrystal.h>
#include <LCDMenuLib2.h>

// settings for lcd
#define _LCDML_DISP_cols  16
#define _LCDML_DISP_rows  2

#define _LCDML_DISP_cfg_cursor                     0x7E   // cursor Symbol
#define _LCDML_DISP_cfg_scrollbar                  1      // enable a scrollbar

#ifdef TAVBOT
#else
extern Adafruit_LiquidCrystal lcd;
extern Adafruit_MCP23017 mcp;
#endif

const uint8_t scroll_bar[7][8] = {
  {B10001, B10001, B10001, B10001, B10001, B10001, B10001, B10001}, // scrollbar empty
  {B11111, B11111, B10001, B10001, B10001, B10001, B10001, B10001}, // scroll state 1
  {B10001, B10001, B11111, B11111, B10001, B10001, B10001, B10001}, // scroll state 2
  {B10001, B10001, B10001, B10001, B11111, B11111, B10001, B10001}, // scroll state 3
  {B10001, B10001, B10001, B10001, B10001, B10001, B11111, B11111}, // scrollbar bottom
  {B00000, B00000, B00000, B00000, B00000, B00000, B10101, B00000}, // ellipsis ...
  {B00000, B11111, B10001, B10101, B10101, B10001, B11111, B00000}  // filled square, use with japanese sqare (0xdb)
};

void lcdml_menu_display();
void lcdml_menu_clear();
void lcdml_menu_control();

LCDMenuLib2_menu LCDML_0 (255, 0, 0, NULL, NULL); // root menu element (do not change)
LCDMenuLib2 LCDML(LCDML_0, _LCDML_DISP_rows, _LCDML_DISP_cols, lcdml_menu_display, lcdml_menu_clear, lcdml_menu_control);

// LCDML_add(id, prev_layer, new_num, lang_char_array, callback_function)
// LCDML_addAdvanced(id, parent, child, condetion, content, callback, param, settings);

LCDML_add         ( 0, LCDML_0         , 1, "Menu",            NULL);
LCDML_addAdvanced ( 1, LCDML_0_1       , 1, NULL, "",          mFunc_gear,         0, _LCDML_TYPE_dynParam);
LCDML_addAdvanced ( 2, LCDML_0_1       , 2, NULL, "",          mFunc_light,        0, _LCDML_TYPE_dynParam);
LCDML_addAdvanced ( 3, LCDML_0_1       , 3, NULL, "",          mFunc_mode,         0, _LCDML_TYPE_dynParam);
LCDML_add         ( 4, LCDML_0_1       , 4, "Power off",       mFunc_power_off);
LCDML_add         ( 5, LCDML_0_1       , 5, "Remote TX",       NULL);
LCDML_addAdvanced ( 6, LCDML_0_1_5     , 1, NULL, "",          mFunc_tx_id,        0, _LCDML_TYPE_dynParam);
LCDML_addAdvanced ( 7, LCDML_0_1_5     , 2, NULL, "",          mFunc_tx_freq,      0, _LCDML_TYPE_dynParam);
LCDML_addAdvanced ( 8, LCDML_0_1_5     , 3, NULL, "",          mFunc_tx_group,     0, _LCDML_TYPE_dynParam);
LCDML_add         ( 9, LCDML_0_1_5     , 4, "Reset",           mFunc_tx_reset);
LCDML_add         (10, LCDML_0_1_5     , 5, "Save to FLASH",   mFunc_tx_save);
// LCDML_addAdvanced (11, LCDML_0_1       , 6, NULL, "",          mFunc_test_dyn,     0, _LCDML_TYPE_dynParam);
// LCDML_add         (3 , LCDML_0_1       , 9  , "Screensaver"      , mFunc_screensaver);       // this menu function can be found on "LCDML_display_menuFunction" tab

// this value must be the same as the last menu element
#define _LCDML_DISP_cnt    10

// create menu
LCDML_createMenu(_LCDML_DISP_cnt);

uint8_t g_dynParam = 0;
extern bool g_light;

static uint8_t g_cur_snd = 0;

void mFunc_sounds(uint8_t n)
{
  if (LCDML.BT_checkAny()) {
    Serial.println("any");
    if (LCDML.BT_checkEnter()) {
    }
    if (LCDML.BT_checkUp()) {
      g_cur_snd--;
      LCDML.BT_resetUp();
    }
    if (LCDML.BT_checkDown()) {
      g_cur_snd++;
      LCDML.BT_resetDown();
    }
  }
  lcd.setCursor(1, n);
  lcd.print("sound ");
  lcd.print(g_cur_snd);
}

void mFunc_gear(uint8_t n)
{
  if (n == LCDML.MENU_getCursorPos()) {
    if (LCDML.BT_checkAny()) {
      if (LCDML.BT_checkEnter()) {
        LCDML.BT_resetEnter();
      }
      if (LCDML.BT_checkLeft()) {
        if (g_gear > 0)
          g_gear--;
        LCDML.BT_resetLeft();
      }
      if (LCDML.BT_checkRight()) {
        if (g_gear < 2)
          g_gear++;
        LCDML.BT_resetRight();
      }
    }
  }
  lcd.setCursor(1, n);
  lcd.print(F("Gear: "));
  lcd.print(g_gear + 1);
}

void mFunc_light(uint8_t n)
{
  bool l = sdata.mode & MODE_BLINK_ON;
  if (n == LCDML.MENU_getCursorPos()) {
    if (LCDML.BT_checkAny()) {
      if (LCDML.BT_checkEnter()) {
        l = ! l;
        LCDML.BT_resetEnter();
      }
      if (LCDML.BT_checkLeft()) {
        l = ! l;
        LCDML.BT_resetLeft();
      }
      if (LCDML.BT_checkRight()) {
        l = ! l;
        LCDML.BT_resetRight();
      }
    }
    if (l) {
      sdata.mode |= MODE_BLINK_ON;
    } else {
      sdata.mode &= ~MODE_BLINK_ON;
    }
  }
  lcd.setCursor(1, n);
  lcd.print(F("Light "));
  lcd.write(l ? 0x06 : 0xdb);
}

void mFunc_mode(uint8_t n)
{
  uint8_t m = sdata.mode & 0x7;
  if (n == LCDML.MENU_getCursorPos()) {
    if (LCDML.BT_checkAny()) {
      if (LCDML.BT_checkEnter()) {
        LCDML.BT_resetEnter();
      }
      if (LCDML.BT_checkLeft()) {
        if (m > 1)
          m--;
        LCDML.BT_resetLeft();
      }
      if (LCDML.BT_checkRight()) {
        if (m < 3)
          m++;
        LCDML.BT_resetRight();
      }
    }
    sdata.mode &= ~0x7;
    sdata.mode |= m;
  }
  lcd.setCursor(1, n);
  lcd.print(F("Mode: "));
  switch (m) {
    case 0:
      lcd.print(F("###"));
      break;
    case 1:
      lcd.print(F("DRIVE"));
      break;
    case 2:
      lcd.print(F("CRAWL"));
      break;
    case 3:
      lcd.print(F("CAM"));
      break;
  }
}

void mFunc_test_dyn(uint8_t n)
{
  if (n == LCDML.MENU_getCursorPos()) {
    if (LCDML.BT_checkAny()) {
      if (LCDML.BT_checkEnter()) {
      }
      if (LCDML.BT_checkLeft()) {
        g_dynParam--;
        LCDML.BT_resetLeft();
      }
      if (LCDML.BT_checkRight()) {
        g_dynParam++;
        LCDML.BT_resetRight();
      }
    }
  }
  lcd.setCursor(1, n);
  lcd.print("dyn: ");
  lcd.print(g_dynParam);
  lcd.print(":");
  lcd.write(g_dynParam);
}

void menu_setup()
{
  Serial.println(F(_LCDML_VERSION)); // only for examples

  // LCD Begin
  lcd.begin(_LCDML_DISP_cols, _LCDML_DISP_rows);
  // set special chars for scrollbar
  for (int i = 0; i < sizeof(scroll_bar) / sizeof(scroll_bar[0]); i++) {
    lcd.createChar(i, (uint8_t*)scroll_bar[i]);
  }

  LCDML_setup(_LCDML_DISP_cnt);
  LCDML.MENU_enRollover();
}

#if(_LCDML_DISP_rows > _LCDML_DISP_cfg_max_rows)
#error change value of _LCDML_DISP_cfg_max_rows in LCDMenuLib2.h
#endif
