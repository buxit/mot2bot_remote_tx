#define RF69_COMPAT 0

//#define TAVBOT
//#define USE_MASSAGER

#include <JeeLib.h>
#include <util/crc16.h>
#include <LCDMenuLib2.h>

#ifdef USE_MASSAGER
#define MASSAGE_PARSER_BUFFERSIZE 48
#define MASSAGE_PACKER_BUFFERSIZE 48

#include <SlipMassagePacker.h>
#include <SlipMassageParser.h>
#endif

#ifdef TAVBOT
#include <Adafruit_RGBLCDShield.h>
#else
#define PRINT_JOYSTICKS
#include <Adafruit_MCP23017.h>
#include <Adafruit_LiquidCrystal.h>
#endif

#ifdef TAVBOT
#define RED    0x1
#define GREEN  0x2
#define YELLOW 0x3
#define BLUE   0x4
#define VIOLET 0x5
#define TEAL   0x6
#define WHITE  0x7

Adafruit_RGBLCDShield lcd;
#define BUTTON_BACK   0x20

Port vertical2(1);
Port horizontal2(2);

Port vertical(3);
Port horizontal(4);
#else
#define BUTTON_LEFT   0x01
#define BUTTON_DOWN   0x02
#define BUTTON_RIGHT  0x04
#define BUTTON_UP     0x08
#define BUTTON_BACK   0x10
#define BUTTON_SELECT 0x20

#define RED     1
#define YELLOW  1
#define GREEN   1
#define TEAL    1
#define BLUE    1
#define VIOLET  1
#define WHITE   1
byte arduinoIntPin = 3;
volatile bool buttonPressed = false;

Adafruit_LiquidCrystal lcd(4);  // Set the LCD I2C address
Adafruit_MCP23017 mcp;

Port vertical2(2);
Port horizontal2(1);

Port vertical(3);
Port horizontal(4);
#endif

extern LCDMenuLib2 LCDML;

#define MODE_DRIVE       0x01
#define MODE_CRAWL       0x02
#define MODE_CAM         0x04
#define MODE_CAM_CENTER  0x08
#define MODE_BEEP_ON     0x10
#define MODE_BLINK_ON    0x40
#define MODE_SHUTDOWN   0x100
#define MODE_CAM_SNAP   0x200

#define MAGIC_ROBOT 6435437

short h1_center = 0;
short v1_center = 0;
short h2_center = 0;
short v2_center = 0;

byte channel = 2;

short vert_1_btn = 0;
uint8_t vert_btn = 0;
short horiz_btn = 0;

struct RemoteData {
  char magic[4] = { 'm', '2', 'b', '\000' };
  short h1;
  short v1;
  short h2;
  short v2;
  short mode;
};

struct RobotData {
  long magic;
  uint8_t cmd;
  float bat_percent;
  float bat_voltage;
  int8_t rssi;
};

struct RobotSound {
  long magic = 6435437;
  uint8_t cmd = 1;
  uint8_t id;
  char name [16];
};


RemoteData sdata;
RobotData rdata;
RobotSound rsound;

uint8_t i = 0;

struct RF12Config {
  uint8_t node_id;
  uint8_t group;
  uint8_t format;
  uint8_t flags;
  uint16_t frequency_offset;
  byte pad[RF12_EEPROM_SIZE - 8];
  word crc;
};

RF12Config g_rf12_cfg;

String rf12_configStr(RF12Config &cfg);
void menu_setup();


void led(bool on) {
  horizontal2.digiWrite(on);
}

void setup() {
  Serial.begin(115200);                // start serial
  menu_setup();
  lcd.setBacklight(WHITE);
  lcd.setCursor(4, 0);
  lcd.print(F("mot2bot"));
  lcd.setCursor(0, 1);
  delay(200);
  memset(&g_rf12_cfg, 0, sizeof(g_rf12_cfg));
  read_config(g_rf12_cfg);
  String cfg_str(rf12_configStr(g_rf12_cfg));
  lcd.print(cfg_str);
  Serial.println(cfg_str);
  rf12_initialize(g_rf12_cfg.node_id, g_rf12_cfg.node_id >> 6, g_rf12_cfg.group, g_rf12_cfg.frequency_offset);

  channel = (g_rf12_cfg.node_id & RF12_HDR_MASK)-1;
  //rf12_configSilent();

  vertical.digiWrite(1);
  vertical2.digiWrite(1);
  horizontal.digiWrite(1);
  horizontal2.mode(OUTPUT);

  h1_center = horizontal.anaRead();
  v1_center = vertical.anaRead();
  h2_center = horizontal2.anaRead();
  v2_center = vertical2.anaRead();

#ifdef TAVBOT
  delay(1000);
#else
  delay(200);
  mcp.begin();
  // We mirror INTA and INTB, so that only one line is required between MCP and Arduino for int reporting
  // The INTA/B will not be Floating
  // INTs will be signaled with a LOW
  mcp.setupInterrupts(true, false, LOW);

  pinMode(arduinoIntPin, INPUT_PULLUP);

  for (i = 0; i < 16; i++) {
    mcp.pinMode(i, INPUT);
    mcp.pullUp(i, HIGH);  // turn on a 100K pullup internally
    mcp.setupInterruptPin(i, CHANGE);
  }
  i = 0;
  attachInterrupt(digitalPinToInterrupt(arduinoIntPin), intCallBack, FALLING);
#endif
  lcd.clear();
  sdata.mode = MODE_DRIVE;
  //  openScreen(3);
}

#ifndef TAVBOT
void intCallBack() {
  buttonPressed = true;
}
#endif

float bat_percent = 0;
short lastRcv = 0;
bool beep = false;
bool blink = false;
byte current_menu = 0;

short g_gear = 2;

MilliTimer sendTimer;
byte g_needToSend;
uint8_t buttons = 0x0;

short _h1, _h2, _v1, _v2;

#ifdef USE_MASSAGER
SlipMassageParser inbound;
SlipMassagePacker outbound;

uint8_t tryRecv(unsigned short line) {
  if (rf12_recvDone() && rf12_crc == 0) {
    uint8_t n = rf12_len;
    inbound.flush();
    for (int8_t i = 0; i < n; i++) {
      if (inbound.parse(rf12_data[i])) {
        Serial.println(F("massage parsed."));
        if (inbound.fullMatch("m2b")) {
          Serial.println("m2b");
        }
      }
    }
  }
}
#else
uint8_t tryRecv(unsigned short line) {
  /*
  Serial.print("tryRecv(): ");
  Serial.print(line);
  Serial.print(": ");
  Serial.println(millis());
  */
  RobotData *rd = 0;
  uint8_t cmd = 0;
  do {
    //Serial.println("tryRecv(__LINE__)");
    if (rf12_recvDone() && rf12_crc == 0) {
      //Serial.println("rf12_recvDone() crc ok.");
      rd = (RobotData*)rf12_data;
      if(rd->magic == MAGIC_ROBOT) {
        cmd = rd->cmd;
        switch(cmd) {
          case 0: {
            memcpy(&rdata, (void*)rf12_data, sizeof(RobotData));
            lastRcv = millis();
            if (bat_percent != rdata.bat_percent) {
              bat_percent = rdata.bat_percent;
            }
            break;
          }
          case 1: {
            memcpy(&rsound, (void*)rf12_data, sizeof(RobotSound));
            Serial.print(F("sound: "));
            Serial.print(rsound.id);
            Serial.print(" '");
            Serial.print(rsound.name);
            Serial.println("'");
            break;
          }
        }
      } else {
        Serial.print(F("Ignoring package with magic "));
        Serial.println(rdata.magic);
      }
    }
  } while(cmd == 1);
}
#endif
void loop() {
  LCDML.loop();

  tryRecv(__LINE__);

  short h1 = horizontal.anaRead();
  if (h1 < h1_center)
    h1 = map(h1, 0, h1_center - 1, -511, 0);
  else
    h1 = map(h1, h1_center, 1023, 0, 512);
  short v1 = vertical.anaRead();
  if (v1 < v1_center)
    v1 = map(v1, 0, v1_center - 1, -511, 0);
  else
    v1 = map(v1, v1_center, 1023, 0, 512);

  short h2 = horizontal2.anaRead();
  if (h2 < h2_center)
    h2 = map(h2, 0, h2_center - 1, -511, 0);
  else
    h2 = map(h2, h2_center, 1023, 0, 512);
  short v2 = vertical2.anaRead();
  if (v2 < v1_center)
    v2 = map(v2, 0, v2_center - 1, -511, 0);
  else
    v2 = map(v2, v2_center, 1023, 0, 512);

  if (abs(h1) < 30)
    h1 = 0;
  if (abs(v1) < 30)
    v1 = 0;

  if (abs(h2) < 10)
    h2 = 0;
  if (abs(v2) < 10)
    v2 = 0;

#ifdef TAVBOT
  h1 *= -1;
  v1 *= -1;
#else
  h2 *= -1;
  v2 *= -1;
#endif
  h1 /= 3 - g_gear;
  v1 /= 3 - g_gear;

  h2 /= 3 - g_gear;
  v2 /= 3 - g_gear;

  tryRecv(__LINE__);

  if (vertical2.digiRead() == 0) {
    sdata.mode |= MODE_CAM_CENTER;
    g_needToSend = 1;
  } else {
    sdata.mode &= ~MODE_CAM_CENTER;
  }

#ifdef TAVBOT
  buttons = lcd.readButtons();
  if (horizontal.digiRead() == 0) {
    buttons |= BUTTON_BACK;
  }
  tryRecv(__LINE__);
#else
  if (buttonPressed) {
    buttonPressed = false;
    uint8_t pin = mcp.getLastInterruptPin();
    uint8_t val = mcp.getLastInterruptPinValue();
    buttons = ~(mcp.readGPIO(1));
    Serial.print(F("p="));
    Serial.print(pin);
    Serial.print(F(" v="));
    Serial.print(val);
    Serial.print(F(" buttons="));
    Serial.println(buttons);
  }
#endif

  if (sendTimer.poll(100)) {
    g_needToSend = 1;
  }

  if (g_needToSend && rf12_canSend()) {
    sdataZusammenschrauben(h1, v1, h2, v2);
    /*
      Serial.print("send on chan: ");
      Serial.println(channel + 1);
    */
    led(true);
    rf12_sendStart((channel) | RF12_HDR_DST, &sdata, sizeof(sdata));
    rf12_sendWait(0);
    led(false);

    tryRecv(__LINE__);

    if (LCDML.MENU_getLayer() == 0) {
      char bp[6];
      if (buttons & BUTTON_UP) {
        sdata.mode |= MODE_CAM_SNAP;
        g_needToSend = 1;
      } else {
        sdata.mode &= ~MODE_CAM_SNAP;
      }

      lcd.setCursor(0, 0);
      tryRecv(__LINE__);
      lcd.print(F("M"));
      tryRecv(__LINE__);
      lcd.print((sdata.mode & 0x7));
      tryRecv(__LINE__);
      lcd.print(F(" G"));
      tryRecv(__LINE__);
      lcd.print(g_gear + 1);
      tryRecv(__LINE__);
      lcd.setCursor(6, 0);
      tryRecv(__LINE__);
      lcd.print(rdata.rssi);
      tryRecv(__LINE__);
      lcd.setCursor(10, 0);
      dtostrf(bat_percent, 5, 2, bp);
      tryRecv(__LINE__);
      lcd.print(bp);
      tryRecv(__LINE__);
      lcd.print("%");
#ifdef PRINT_JOYSTICKS
      tryRecv(__LINE__);
      lcd.setCursor(0, 1);
      tryRecv(__LINE__);
      lcd.print(l2s(h2, 4));
      tryRecv(__LINE__);
      lcd.setCursor(4, 1);
      tryRecv(__LINE__);
      lcd.print(l2s(v2, 4));
      tryRecv(__LINE__);
      lcd.setCursor(8, 1);
      tryRecv(__LINE__);
      lcd.print(l2s(h1, 4));
      tryRecv(__LINE__);
      lcd.setCursor(12, 1);
      tryRecv(__LINE__);
      lcd.print(l2s(v1, 4));
#endif
    }
    g_needToSend = 0;
  }
}

String stringAlign(short num, byte width) {
  String s(num);
  String r;
  byte i;
  for (i = s.length(); i < width; i++)
  {
    r += " ";
  }
  r += s;
  return r;
}

void sdataZusammenschrauben(short h1, short v1, short h2, short v2) {
  if(sdata.mode & MODE_CRAWL) {
    sdata.h1 = h2;
    sdata.h2 = h1;
    sdata.v1 = v2;
    sdata.v2 = v1;
  } else {
    sdata.h1 = h1;
    sdata.h2 = h2;
    sdata.v1 = v1;
    sdata.v2 = v2;
  }
}

static word calcCrc (const void* ptr, byte len) {
  word crc = ~0;
  for (byte i = 0; i < len; ++i)
    crc = _crc16_update(crc, ((const byte*) ptr)[i]);
  return crc;
}

void read_config(RF12Config &cfg) {
  cfg.node_id = eeprom_read_byte(RF12_EEPROM_ADDR);
  cfg.group = eeprom_read_byte(RF12_EEPROM_ADDR + 1);
  cfg.flags = eeprom_read_byte(RF12_EEPROM_ADDR + 3);
  cfg.frequency_offset = eeprom_read_word((uint16_t*) (RF12_EEPROM_ADDR + 4));
  channel = (g_rf12_cfg.node_id & RF12_HDR_MASK)-1;
}

static void save_config(RF12Config &cfg) {
  cfg.format = RF12_EEPROM_VERSION;
  cfg.crc = calcCrc(&cfg, sizeof(cfg) - 2);
  // eeprom_write_block(&config, RF12_EEPROM_ADDR, sizeof(config));
  // this uses 170 bytes less flash than eeprom_write_block(), no idea why
  eeprom_write_byte(RF12_EEPROM_ADDR, ((byte*) &cfg)[0]);
  for (byte i = 0; i < sizeof(cfg); ++ i)
    eeprom_write_byte(RF12_EEPROM_ADDR + i, ((byte*) &cfg)[i]);
}

String rf12_configStr (RF12Config &cfg) {
  String s;
  s.reserve(30);
  s += ("i");
  s += (cfg.node_id & RF12_HDR_MASK);
  if (cfg.flags & 0x04)
    s += ('*');
  s += (" g");
  s += (cfg.group);
  s += (" +");
  s += (cfg.frequency_offset);

  return s;
}

void mFunc_power_off(uint8_t n)
{
  sdata.mode |= MODE_SHUTDOWN;
}

void mFunc_tx_freq(uint8_t n)
{
  if (n == LCDML.MENU_getCursorPos()) {
    if (LCDML.BT_checkAny()) {
      if (LCDML.BT_checkEnter()) {
        rf12_initialize(g_rf12_cfg.node_id, g_rf12_cfg.node_id >> 6, g_rf12_cfg.group, g_rf12_cfg.frequency_offset);
      }
      if (LCDML.BT_checkLeft()) {
        g_rf12_cfg.frequency_offset--;
        LCDML.BT_resetLeft();
      }
      if (LCDML.BT_checkRight()) {
        g_rf12_cfg.frequency_offset++;
        LCDML.BT_resetRight();
      }
    }
  }
  lcd.setCursor(1, n);
  lcd.print("freq: ");
  lcd.print(g_rf12_cfg.frequency_offset);
}

void mFunc_tx_id(uint8_t n)
{
  if (n == LCDML.MENU_getCursorPos()) {
    if (LCDML.BT_checkAny()) {
      if (LCDML.BT_checkEnter()) {
        rf12_initialize(g_rf12_cfg.node_id, g_rf12_cfg.node_id >> 6, g_rf12_cfg.group, g_rf12_cfg.frequency_offset);
        channel = (g_rf12_cfg.node_id & RF12_HDR_MASK)-1;
      }
      if (LCDML.BT_checkLeft()) {
        g_rf12_cfg.node_id--;
        if (g_rf12_cfg.node_id < 0x81)
          g_rf12_cfg.node_id = 0x81;
        LCDML.BT_resetLeft();
      }
      if (LCDML.BT_checkRight()) {
        g_rf12_cfg.node_id++;
        if (g_rf12_cfg.node_id > 0x81 + 30)
          g_rf12_cfg.node_id = 0x81 + 30;
        LCDML.BT_resetRight();
      }
    }
  }
  lcd.setCursor(1, n);
  lcd.print("node id: ");
  lcd.print(g_rf12_cfg.node_id & RF12_HDR_MASK);
}

void mFunc_tx_group(uint8_t n)
{
  if (n == LCDML.MENU_getCursorPos()) {
    if (LCDML.BT_checkAny()) {
      if (LCDML.BT_checkEnter()) {
        rf12_initialize(g_rf12_cfg.node_id, g_rf12_cfg.node_id >> 6, g_rf12_cfg.group, g_rf12_cfg.frequency_offset);
      }
      if (LCDML.BT_checkLeft()) {
        g_rf12_cfg.group--;
        LCDML.BT_resetLeft();
      }
      if (LCDML.BT_checkRight()) {
        g_rf12_cfg.group++;
        LCDML.BT_resetRight();
      }
    }
  }
  lcd.setCursor(1, n);
  lcd.print(F("group: "));
  lcd.print(g_rf12_cfg.group);
}

void mFunc_tx_save(uint8_t param)
{
  if (LCDML.FUNC_setup())
  {
    save_config(g_rf12_cfg);
    String cfg_str(rf12_configStr(g_rf12_cfg));
    lcd.setCursor(0, 0);
    lcd.print(F("saved:"));
    lcd.setCursor(0, 1);
    lcd.print(cfg_str);
  }

  if (LCDML.FUNC_loop())
  {
    if (LCDML.BT_checkAny()) {
      LCDML.FUNC_goBackToMenu();
    }
  }
}

void mFunc_tx_reset(uint8_t param)
{
  if (LCDML.FUNC_setup())
  {
    save_config(g_rf12_cfg);
    lcd.setCursor(0, 0);
    lcd.print(F("ENTER \x7e reset to"));
    lcd.setCursor(0, 1);
    lcd.print(F("i2 g212 +1600"));
  }

  if (LCDML.FUNC_loop())
  {
    if (LCDML.BT_checkEnter()) {
      g_rf12_cfg.frequency_offset = 1600;
      g_rf12_cfg.group = 212;
      g_rf12_cfg.node_id = 0x82;
      rf12_initialize(g_rf12_cfg.node_id, g_rf12_cfg.node_id >> 6, g_rf12_cfg.group, g_rf12_cfg.frequency_offset);
      channel = (g_rf12_cfg.node_id & RF12_HDR_MASK)-1;
    }
    if (LCDML.BT_checkAny()) {
      LCDML.FUNC_goBackToMenu();
    }
  }
}

char *l2s(long x, unsigned d)
{
  unsigned long a = abs(x);
  static char b[8];
  char *p;

  // if (digits > d) d = digits; // uncomment to allow more digits than spec'd
  *(p = b + d) = '\0';
  do *--p = a % 10 + '0'; while (a /= 10);
  if (x < 0) *--p = '-';
  while (p != b) *--p = ' ';
  return b;
}

