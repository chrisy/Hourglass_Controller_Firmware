#include <Lixie_II.h>            // https://github.com/connornishijima/Lixie_II

#include <ESP8266WiFi.h>         // https://github.com/esp8266/Arduino
#include <DNSServer.h>           //  |
#include <ESP8266WebServer.h>    //  |
#include <WiFiUdp.h>             //  |
#include <FS.h>                  // <

#include <WiFiManager.h>         // https://github.com/tzapu/WiFiManager/tree/development **USES DEVELOPMENT BRANCH!** (NOT THE VERSION IN ARDUINO LIBRARY MANAGER)
#include <NTPClient.h>           // https://github.com/arduino-libraries/NTPClient
#include <ArduinoJson.h>         // https://github.com/bblanchon/ArduinoJson

// USER SETTINGS //////////////////////////////////////////////////////////////////
#define DATA_PIN                  D6      // Lixie DIN connects to this pin
#define NUM_DIGITS                6

#define NTP_UPDATE_INTERVAL_MIN   1

#define HOUR_BUTTON               D7      // These are pulled up internally, and should be
#define HUE_BUTTON                D2      // tied to GND through momentary switches

#define BUZZER                    D8      // OPTIONAL
#define COLOR_CYCLE               D1      // OPTIONAL
#define NIGHT_DIMMING             D5      // OPTIONAL

#define BRIGHTNESS_SLEW_RATE      0.05    // Rate per clock update for brightness slewing
#define BRIGHTNESS_HIGH           1.0     // Full brightness; float from 0.0 to 1.0
#define BRIGHTNESS_LOW            0.4     // Low brightness
///////////////////////////////////////////////////////////////////////////////////

ADC_MODE(ADC_VCC);

Lixie_II lix(DATA_PIN, NUM_DIGITS);
WiFiUDP ntp_UDP;
NTPClient time_client(ntp_UDP, "ntp.flirble.org", 0, 60000 * NTP_UPDATE_INTERVAL_MIN);
Ticker loading_run;
Ticker lix_run;
Ticker tick_run;
Ticker serial_run;

#define SECONDS_PER_HOUR 3600
bool lix_looping = false;
uint8_t load_pos = 0;

uint8_t device_debug_index = 0;
uint32_t ticks_passed = 0;
uint32_t last_tick = 0;
uint32_t last_time = 0;
bool tick_debug_printed = true;
bool debug_output = false; // enabled by "ENABLE_DEBUG\n" over serial
bool wifi_lost = false;
bool sync_fail = false;

struct conf {
  int16_t time_zone_shift = -5;
  uint8_t hour_12_mode = false;
  uint8_t base_hue = 0;
  uint8_t current_mode = 0;
  uint8_t six_digit_clock = true;
};
conf clock_config; // <- global configuration object

float base_hue_f = 0;
uint32_t settings_last_update = 0;
bool settings_changed = false;
const char* settings_file = "/settings.json";

uint32_t t_now = 0;
uint8_t last_seconds = 0;

#define STABLE 0
#define RISE   1
#define FALL   2

uint8_t hh = 0;
uint8_t mm = 0;
uint8_t ss = 0;

bool color_cycle_state   = HIGH;
bool night_dimming_state = HIGH;

bool hour_button_state   = HIGH;
bool hue_button_state  = HIGH;
bool hour_button_state_last   = HIGH;
bool hue_button_state_last = HIGH;
uint8_t hour_button_edge   = STABLE;
uint8_t hue_button_edge = STABLE;
uint32_t hour_button_last_hit = 0;
uint32_t hue_button_last_hit = 0;
uint32_t hour_button_start = 0;
uint16_t hour_button_wait = 1000;
bool hour_mode_started = false;
uint32_t hue_button_start = 0;
uint8_t button_debounce_ms = 100;

bool double_hold = false;
uint32_t double_hold_start = 0;
bool double_hold_set = false;

uint16_t hue_countdown = 255;
uint16_t hue_push_wait = 500;

bool tick = false;

#define NUM_MODES 7

#define MODE_SOLID          0
#define MODE_GRADIENT       1
#define MODE_DUAL           2
#define MODE_NIXIE          3
#define MODE_INCANDESCENT   4
#define MODE_VFD            5
#define MODE_WHITE          6

void setup() {
  Serial.begin(230400);
  Serial.println("\n\n");
  serial_run.attach_ms(100, check_serial);
  Serial.println("HOURGLASS");
  delay(1000);
  print_info("Welcome to the Hourglass NTP clock for Lixies!\n");
  print_info("If you're poking into the Serial output of this clock because you've");
  print_info("run into an issue, please email us at team@lixielabs.com, or try our");
  print_info("GUI debugging monitor built specifically for this clock, found on GitHub:\n");
  print_info("https://github.com/connornishijima/Hourglass-Debugging-Monitor/releases\n");
  print_info("This firmware ver. 1.0.0bis was compiled by Bodshal.\n");

  print_info("Runtime data is accessible through the Hourglass Debugging Monitor.\n\n");

  print_debug("BOOT_STATUS", "INIT");
  init_fs();
  load_settings();

  print_full_device_info();

  init_displays();
  init_buttons();
  init_wifi();
  init_ntp();
  print_debug("BOOT_STATUS", "OK");
}

void loop() {
  run_clock();
  yield();
}

void lix_loop() {
  color_for_mode();
  check_buttons();
}

void run_clock() {
  t_now = millis();

  if (ticks_passed < 2 && t_now - last_tick >= 1000) {
    // Used to delay display for 2 seconds
    last_tick = t_now;
    ticks_passed++;
  }

  if (t_now - last_time >= 50) {
    last_time = t_now;
    // Update the display every 50ms; this way the seconds value is
    // closely synchronized with the actual time
    if (ticks_passed >= 2) {
      show_time();
    }

    if (ss != last_seconds) {
      // A new second has started
      last_seconds = ss;

      if (ticks_passed >= 2) {
        tick_tock();
      }
      tick_debug_printed = false;
    }
  }

  bool did_get_time = time_client.update();

  if (!did_get_time && time_client.getPacketSent()) {
    // Did not get time, and an NTP packet was sent, presumably
    // without reply
    print_error("NTP SYNC FAILED!");
    sync_fail = true;
  }
  else if (sync_fail == true) {
    print_info("NTP re-synchronized!");
    sync_fail = false;
  }

  if (WiFi.RSSI() == 31 && !wifi_lost) {
    wifi_lost = true;
    print_debug("WIFI_CONNECTION", "RECONNECTING");
    print_error("WiFi connection has been lost, attempting to reconnect...");
  }
  else if (WiFi.RSSI() != 31 && wifi_lost) {
    wifi_lost = false;
    print_debug("WIFI_CONNECTION", "CONNECTED");
    print_info("WiFi reconnected.");

    // Attempt to get the time immediately, if we didn't just get it
    if (!did_get_time)
      did_get_time = time_client.forceUpdate();
  }

  if (t_now - last_tick >= 500 && tick_debug_printed == false) {
    tick_debug_printed = true;
    print_full_device_info();
  }

  if (!lix_looping) {
    print_info("Clock started!");
    lix_looping = true;
    lix_run.attach_ms(20, lix_loop);
  }

  if (t_now - settings_last_update > 15000 && settings_changed == true) {
    settings_changed = false;
    save_settings();
  }
}

void check_serial() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    if (command == "RESET_DEBUG") {
      print_error("MANUAL RESET CALLED!");
      ESP.reset();
    }
    else if (command == "ENABLE_DEBUG") {
      debug_output = true;
      print_error("DEBUG OUTPUT ENABLED!");
    }
  }
}

void save_settings() {
  //print_info("save_settings()");
  print_info("Saving new settings to file...");
  // Delete existing file, otherwise the configuration is appended to the file
  SPIFFS.remove(settings_file);

  // Open file for writing
  File file = SPIFFS.open(settings_file, "w+");
  if (!file) {
    print_debug("SAVE_SETTINGS", "FAILED");
    print_error("Failed to open /settings.json file for saving!");
    return;
  }
  else {
    print_debug("SAVE_SETTINGS", "OPEN");
  }

  // Allocate a temporary JsonDocument
  StaticJsonDocument<512> doc_out;

  // Set the values in the document
  doc_out["base_hue"] = clock_config.base_hue;
  doc_out["current_mode"] = clock_config.current_mode;
  doc_out["time_zone_shift"] = clock_config.time_zone_shift;
  doc_out["hour_12_mode"] = clock_config.hour_12_mode;
  doc_out["six_digit_clock"] = clock_config.six_digit_clock;

  // Serialize JSON to file
  if (serializeJson(doc_out, file) == 0) {
    print_debug("SAVE_SETTINGS", "FAILED");
    print_error("Failed to write settings to /settings.json file!");
  }
  else {
    print_debug("SAVE_SETTINGS", "OK");
    print_info("Settings saved.");
  }

  // Close the file
  file.close();
}

void load_settings() {
  //print_info("load_settings()");
  print_info("Loading clock configuration from /settings.json file...");
  // Open file for reading
  File file = SPIFFS.open(settings_file, "r");

  // Allocate a temporary JsonDocument
  StaticJsonDocument<512> doc_in;

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc_in, file);
  if (error) {
    print_debug("LOAD_SETTINGS", "FAILED");
    print_error("Failed to read /settings.json file, using default configuration");
  }
  else {
    print_debug("LOAD_SETTINGS", "OK");
  }

  // Copy values from the JsonDocument to the Config
  clock_config.base_hue = doc_in["base_hue"];
  base_hue_f            = doc_in["base_hue"];
  clock_config.current_mode = doc_in["current_mode"];
  clock_config.time_zone_shift = doc_in["time_zone_shift"];
  clock_config.hour_12_mode = doc_in["hour_12_mode"];
  clock_config.six_digit_clock = doc_in["six_digit_clock"];

  // Close the file (Curiously, File's destructor doesn't close the file)
  file.close();
}

void show_time() {
  unsigned long epoch = time_client.getEpochTime();

  hh = (epoch % 86400L) / 3600;
  mm = (epoch % 3600) / 60;
  ss = epoch % 60;

  print_debug("HOUR", hh);
  print_debug("MIN",  mm);
  print_debug("SEC",  ss);

  // Brightness is managed with a linear slew towards the
  // target value. Since we're called every 50ms this should
  // cause smooth changes.

  static float brightness = 0.0;
  float brightness_target;

  if (night_dimming_state == HIGH && (hh < 6 || hh >= 21)) {
    // Dim overnight from 9PM to 6AM (21:00 to 06:00)
    brightness_target = BRIGHTNESS_LOW;
  }
  else {
    brightness_target = BRIGHTNESS_HIGH;
  }

  if (fabs(brightness - brightness_target) > BRIGHTNESS_SLEW_RATE) {
    if (brightness < brightness_target) {
      // Need to increase the brightness!
      brightness += BRIGHTNESS_SLEW_RATE;
    }
    else {
      // Need to decrease it
      brightness -= BRIGHTNESS_SLEW_RATE;
    }
  }
  else {
    // Difference is small; we're at the target
    brightness = brightness_target;
  }

  lix.brightness(brightness);

  // 12 hour format conversion
  if (clock_config.hour_12_mode == true) {
    if (hh > 12) {
      hh -= 12;
    }
    if (hh == 0) {
      hh = 12;
    }
  }

  uint32_t t_lixie = 1000000; // "1000000" is used to get zero-padding on hours digits
  // This turns a time of 22:34:57 into the integer (1)223457, whose leftmost numeral (1) will not be shown
  t_lixie += (hh * 10000);
  t_lixie += (mm * 100);
  t_lixie += ss;

  if (!clock_config.six_digit_clock) {
    t_lixie /= 100; // Eliminate second places if using a 4 digit clock
  }

  lix.write(t_lixie); // Update numerals

  //Serial.print("TIME: ");
  //Serial.println((hh * 10000) + (mm * 100) + ss);
}

void color_for_mode() {
  if (color_cycle_state == HIGH) {
    base_hue_f += 0.017; // Fully cycles the color wheel every 5 minutes
  }

  clock_config.base_hue = base_hue_f;
  uint8_t temp_hue = clock_config.base_hue + hue_countdown;

  if (clock_config.current_mode == MODE_SOLID) {
    lix.color_all(ON, CHSV(temp_hue, 255, 255));
    lix.color_all(OFF, CRGB(0, 0, 0));
  }
  else if (clock_config.current_mode == MODE_GRADIENT) { // Gradient is calculated manually to allow for resizing the clock (4/6-digit)
    CRGB col1 = CHSV(temp_hue, 255, 255);
    CRGB col2 = CHSV(temp_hue + 90, 255, 255);
    uint8_t digit_count = 4;
    if (clock_config.six_digit_clock) {
      digit_count = 6;
    }

    for (uint8_t i = 0; i < digit_count; i++) {
      float progress = i / float(digit_count);
      uint8_t r_val = col1.r * progress + col2.r * (1 - progress);
      uint8_t g_val = col1.g * progress + col2.g * (1 - progress);
      uint8_t b_val = col1.b * progress + col2.b * (1 - progress);

      lix.color_display(i, ON, CRGB(
                          r_val,
                          g_val,
                          b_val
                        ));
    }
    lix.color_all(OFF, CRGB(0, 0, 0));
  }
  else if (clock_config.current_mode == MODE_DUAL) {
    lix.color_all_dual(ON, CHSV(temp_hue, 255, 255), CHSV(temp_hue + 90, 255, 255));
    lix.color_all(OFF, CRGB(0, 0, 0));
  }
  else if (clock_config.current_mode == MODE_NIXIE) {
    lix.color_all(ON, CRGB(255, 70, 7));
    CRGB col_off = CRGB(0, 100, 255);
    const uint8_t nixie_aura_level = 8;

    col_off.r *= (nixie_aura_level / 255.0);
    col_off.g *= (nixie_aura_level / 255.0);
    col_off.b *= (nixie_aura_level / 255.0);

    lix.color_all(OFF, col_off);
  }
  else if (clock_config.current_mode == MODE_INCANDESCENT) {
    lix.color_all(ON, CRGB(255, 100, 25));
    lix.color_all(OFF, CRGB(0, 0, 0));
  }
  else if (clock_config.current_mode == MODE_VFD) {
    lix.color_all(ON, CRGB(100, 255, 100));
    lix.color_all(OFF, CRGB(0, 0, 0));
  }
  else if (clock_config.current_mode == MODE_WHITE) {
    lix.color_all(ON, CRGB(255, 255, 255));
  }
}

void check_buttons() {
  hour_button_state   = digitalRead(HOUR_BUTTON);
  hue_button_state  = digitalRead(HUE_BUTTON);

  color_cycle_state  = !digitalRead(COLOR_CYCLE);
  night_dimming_state  = !digitalRead(NIGHT_DIMMING);

  if (!double_hold) {
    if (hour_button_state > hour_button_state_last) {
      hour_button_edge = RISE;
      print_debug("HOUR_BUTTON", 0);
    }
    else if (hour_button_state < hour_button_state_last) {
      if (t_now - hour_button_last_hit >= button_debounce_ms) {
        hour_button_last_hit = t_now;
        hour_button_edge = FALL;
        print_debug("HOUR_BUTTON", 1);
      }
    }
    else {
      hour_button_edge = STABLE;
    }

    if (hue_button_state > hue_button_state_last) {
      hue_button_edge = RISE;
      print_debug("HUE_BUTTON", 0);
    }
    else if (hue_button_state < hue_button_state_last) {
      if (t_now - hue_button_last_hit >= button_debounce_ms) {
        hue_button_last_hit = t_now;
        hue_button_edge = FALL;
        print_debug("HUE_BUTTON", 1);
      }
    }
    else {
      hue_button_edge = STABLE;
    }
  }

  if (hour_button_state == LOW && hue_button_state == LOW) {
    if (hour_button_edge == STABLE && hue_button_state == STABLE) {
      if (!double_hold) {
        double_hold = true;
        double_hold_start = t_now;
      }
    }
  }
  else {
    if (double_hold) {
      uint32_t duration = t_now - double_hold_start;
      if (duration < 1000) {
        //Serial.println("QUICK DOUBLE");
      }
    }
    double_hold = false;
    double_hold_set = false;
  }

  parse_buttons();

  hour_button_state_last   = hour_button_state;
  hue_button_state_last  = hue_button_state;
}

void parse_buttons() {
  if (!double_hold) {
    if (hour_button_edge == FALL) { // PRESS STARTED
      hour_button_start = t_now;
    }
    else if (hour_button_edge == RISE) { // PRESS ENDED
      uint16_t hour_button_duration = t_now - hour_button_start;
      if (hour_button_duration < hour_button_wait) { // RELEASED QUICKLY
        //Serial.println("UP");
        print_info("UTC offset incremented!");
        clock_config.time_zone_shift += 1;

        if (clock_config.time_zone_shift >= 12) {
          print_info("(UTC offset wrapped around from +12 to -12)");
          clock_config.time_zone_shift = -12;
        }

        time_client.setTimeOffset(clock_config.time_zone_shift * SECONDS_PER_HOUR);
        hh = time_client.getHours();
        if (hh == 0) {
          beep(1000, 100);
        }
        else {
          beep(2000, 100);
        }
        show_time();
        update_settings();
      }
      else { // RELEASED AFTER LONG PRESS
        hour_mode_started = false;
      }
    }

    if (hue_button_edge == FALL) { // PRESS STARTED
      //Serial.println("HUE");
      hue_button_start = t_now;
    }
    else if (hue_button_edge == RISE) { // PRESS ENDED
      uint16_t hue_button_duration = t_now - hue_button_start;
      if (hue_button_duration < hue_push_wait) { // RELEASED QUICKLY
        //Serial.println("NEXT MODE");
        print_info("Color mode incremented!");
        clock_config.current_mode++;
        if (clock_config.current_mode >= NUM_MODES) {
          print_info("(Color mode wrapped back to 0)");
          clock_config.current_mode = 0;
          beep(1000, 100);
        }
        else {
          beep(2000, 100);
        }
        hue_countdown = 127;
        update_settings();
      }
    }

    if (hue_button_state == LOW) { // CURRENTLY PRESSING
      uint16_t hue_button_duration = t_now - hue_button_start;
      if (hue_button_duration >= hue_push_wait) {
        base_hue_f++;
        //Serial.print("HUE: ");
        //Serial.println(base_hue_f);
        update_settings();
      }
    }

    if (hour_button_state == LOW) { // CURRENTLY PRESSING
      uint16_t hour_button_duration = t_now - hour_button_start;
      if (hour_button_duration >= hour_button_wait && hour_mode_started == false) {
        hour_mode_started = true;
        //Serial.println("CHANGE HOUR MODE");
        beep(4000, 250);
        clock_config.hour_12_mode = !clock_config.hour_12_mode;

        if (clock_config.hour_12_mode) {
          print_info("Clock set to 12-hour mode");
        }
        else {
          print_info("Clock set to 24-hour mode");
        }
        update_settings();
      }
    }

    if (hue_countdown < 255) {
      hue_countdown += 6;
      if (hue_countdown > 255) {
        hue_countdown = 255;
      }
    }
  }
  else {
    if (t_now - double_hold_start >= 1000) {
      if (!double_hold_set) {
        double_hold_set = true;
        //Serial.println("DOUBLE HOLD");
        beep(1000, 200);
        clock_config.six_digit_clock = !clock_config.six_digit_clock;
        if (clock_config.six_digit_clock) {
          //Serial.println("SET TO 6-DIGIT MODE");
          print_info("Clock set to 6-digit mode");
        }
        else {
          //Serial.println("SET TO 4-DIGIT MODE");
          print_info("Clock set to 4-digit mode");
        }
        update_settings();
      }
    }
  }
}

void update_settings() {
  //print_info("update_settings()");
  settings_changed = true;
  settings_last_update = t_now;
}

void enter_config_mode() {
  //print_info("enter_config_mode()");
  lix.color_all(ON, CRGB(64, 0, 255));
  beep_dual(2000, 1000, 500);
}

void config_mode_callback (WiFiManager *myWiFiManager) {
  //print_info("config_mode_callback()");
  print_debug("WIFI_CONNECTION", "CONFIG");
  print_error("WiFi failed to connect, entered config mode!");
  enter_config_mode();
  //Serial.println("Entered config mode");
  //Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  //Serial.println(myWiFiManager->getConfigPortalSSID());
}

void save_wifi_callback () {
  //print_info("save_wifi_callback()");
  print_info("New credentials connected successfully!");
  beep(4000, 100);
  delay(100);
  beep(4000, 100);
  delay(100);
  beep(4000, 100);
}

void pre_save_wifi_callback () {
  //print_info("pre_save_wifi_callback()");
  print_info("New WiFi credentials captured. Attempting connection...");
  lix.color_all(ON, CRGB(255, 64, 0));
  beep(1000, 500);
  delay(100);
}

void init_wifi() {
  //print_info("init_wifi()");
  WiFiManager wifiManager;
  wifiManager.setAPCallback(config_mode_callback);
  wifiManager.setAPStaticIPConfig(IPAddress(8, 8, 8, 8), IPAddress(8, 8, 8, 8), IPAddress(255, 255, 255, 0));
  wifiManager.setCustomHeadElement("<style>body{background: #242424; color:#cccccc;}h1,h2,h3{color:#cccccc;}button{transition: 0.3s;opacity: 0.8;cursor: pointer;border:0;border-radius:1rem;background-color:#1dca79;color:#fff;line-height:2.4rem;font-size:1.2rem;width:100%;}button:hover {opacity: 1}button[type=\"submit\"]{margin-top: 15px;margin-bottom: 10px;font-weight: bold;text-transform: capitalize;}input{height: 30px;font-family:verdana;margin-top: 5px;background-color: rgb(253, 253, 253);border: 0px;-webkit-box-shadow: 2px 2px 5px 0px rgba(0,0,0,0.75);-moz-box-shadow: 2px 2px 5px 0px rgba(0,0,0,0.75);box-shadow: 2px 2px 5px 0px rgba(0,0,0,0.75);}div{color: #14a762;}div a{text-decoration: none;color: #14a762;}div[style*=\"text-align:left;\"]{color:#cccccc;}, div[class*=\"c\"]{border: 0px;}a[href*=\"wifi\"]{border: 2px solid #1dca79;text-decoration: none;color: #1dca79;padding: 10px 30px 10px 30px;font-family: verdana;font-weight: bolder;transition: 0.3s;border-radius: 5rem;}a[href*=\"wifi\"]:hover{background: #1dca79;color: white;}</style>");
  wifiManager.setTimeout(300); // Five minutes
  wifiManager.setSaveConfigCallback(save_wifi_callback);
  wifiManager.setWiFiAutoReconnect(true);
  wifiManager.setShowInfoErase(true);
  wifiManager.setPreSaveConfigCallback(pre_save_wifi_callback);
  if (debug_output) {
    wifiManager.setDebugOutput(true);
  }
  else {
    wifiManager.setDebugOutput(false);
  }

  print_debug("SELF_TEST", "NONE");

  start_load();

  lix.color_all(ON, CRGB(255, 64, 0));

  if (digitalRead(HOUR_BUTTON) == LOW) {
    //wifiManager.resetSettings();
    print_error("HOUR BUTTON held on boot, entering WiFi config mode!");
    print_debug("WIFI_CONNECTION", "CONFIG");
    if (!wifiManager.startConfigPortal("LIXIE CONFIG")) {
      print_debug("WIFI_CONNECTION", "CONFIG_TIMEOUT");
      print_error("Failed to connect and hit config portal timeout");
      delay(3000);
      //reset and try again
      ESP.reset();
    }
  }
  else if (digitalRead(HUE_BUTTON) == LOW) {
    print_error("HUE BUTTON held on boot, entering self-test!");
    serial_run.detach();
    end_load();
    print_debug("SELF_TEST", "INITIATED");
    self_test();
    print_debug("SELF_TEST", "COMPLETE");
    start_load();
    serial_run.attach_ms(100, check_serial);
  }

  beep(2000, 100);
  print_debug("WIFI_MAC_ADDRESS", WiFi.macAddress());
  print_debug("WIFI_CONNECTION", "WAITING");
  print_info("Attempting to connect to stored WiFi network...");
  while (!wifiManager.autoConnect("LIXIE CONFIG")) {
    //wait
    delay(1);
  }
  print_info("Connected sucessfully!");
  print_debug("WIFI_CONNECTION", "CONNECTED");
  print_debug("WIFI_SSID", wifiManager.getWiFiSSID());
  print_debug("WIFI_IP_ADDRESS", WiFi.localIP().toString());
  print_debug("WIFI_RSSI", WiFi.RSSI());

  print_debug("SAVE_SETTINGS", "UNSAVED");
  print_debug("HOUR_BUTTON", 0);
  print_debug("HUE_BUTTON", 0);

  end_load();

  beep_dual(1000, 2000, 100);
  lix.sweep_color(CRGB(0, 255, 127), 20, 3, false);
  lix.clear(); // Remove current display before fading in the time
  color_for_mode();
}

void init_ntp() {
  //print_info("init_ntp()");
  print_info("Synchronizing to NTP time server...");
  time_client.begin();
  time_client.setTimeOffset(clock_config.time_zone_shift * SECONDS_PER_HOUR);
  time_client.setFractionalTime(true);
  print_debug("NTP_TIME", "INITIALIZED");
}

void init_fs() {
  //print_info("init_fs()");
  print_debug("SPIFFS", "INITIALIZING");
  print_info("Inititalizing SPIFFS filesystem...");
  if (SPIFFS.begin()) {
    print_debug("SPIFFS", "OK");
  }
  else {
    print_error("Inititalizing the SPIFFS filesystem has failed!");
    print_debug("SPIFFS", "FAILED");
  }
}

void init_buttons() {
  //print_info("init_buttons()");
  pinMode(HOUR_BUTTON, INPUT_PULLUP);
  pinMode(HUE_BUTTON,  INPUT_PULLUP);

  pinMode(BUZZER, OUTPUT);

  pinMode(COLOR_CYCLE,  INPUT_PULLUP);
  pinMode(NIGHT_DIMMING,  INPUT_PULLUP);
}

void load_loop() {
  //print_info("load_loop()");
  //yield();

  if (!clock_config.six_digit_clock) {
    if (load_pos == 0) {
      lix.write(".   ");
    }
    else if (load_pos == 1) {
      lix.write(" .  ");
    }
    else if (load_pos == 2) {
      lix.write("  . ");
    }
    else if (load_pos == 3) {
      lix.write("   .");
    }

    load_pos++;
    if (load_pos >= 4) {
      load_pos = 0;
    }
  }
  else { // SIX DIGIT CLOCK
    if (load_pos == 0) {
      lix.write(".     ");
    }
    else if (load_pos == 1) {
      lix.write(" .    ");
    }
    else if (load_pos == 2) {
      lix.write("  .   ");
    }
    else if (load_pos == 3) {
      lix.write("   .  ");
    }
    else if (load_pos == 4) {
      lix.write("    . ");
    }
    else if (load_pos == 5) {
      lix.write("     .");
    }

    load_pos++;
    if (load_pos >= 6) {
      load_pos = 0;
    }
  }
}

void init_displays() {
  //print_info("init_displays()");
  lix.begin();
  lix.max_power(5, 2000);
  //lix.transition_time(200);
  lix.write(888888);
  print_info("Lixie displays inititalized...");
}

void beep(uint16_t freq, uint16_t len) {
  //print_info("beep()");
  uint32_t period = (F_CPU / freq) / 2;
  uint32_t cycle_ms = F_CPU / 1000;
  uint32_t b_start = ESP.getCycleCount();
  uint32_t last_flip = b_start;
  uint32_t b_end = b_start + (len * cycle_ms);
  uint32_t b_now = b_start;
  bool state = LOW;
  while (b_now < b_end) {
    b_now = ESP.getCycleCount();
    if (b_now - last_flip >= period) {
      last_flip += period;
      state = !state;

      if (state) {
        GPOS = (1 << BUZZER);
      }
      else {
        GPOC = (1 << BUZZER);
      }
    }
  }
  digitalWrite(BUZZER, LOW);
}

void beep_dual(uint16_t del1, uint16_t del2, uint16_t len) {
  //print_info("beep_dual()");
  beep(del1, len);
  beep(del2, len);
}

void tick_tock() {
  //print_info("tick_tock()");
  tick = !tick;

  if (tick) {
    blip(2);
  }
  else {
    blip(5);
  }

}

void blip(uint32_t len_us) {
  //print_info("blip()");
  const static uint32_t us_cycles = F_CPU / 1000000;
  GPOS = (1 << BUZZER);
  uint32_t t_start = ESP.getCycleCount();
  uint32_t t_end = t_start + len_us * us_cycles;
  while (ESP.getCycleCount() < t_end) {
    // wait
  }
  GPOC = (1 << BUZZER);
}

void self_test_numeral_run(CRGB col, uint16_t ms_per) {
  lix.color_all(ON, col);
  lix.write("000000");
  delay(ms_per);
  lix.write("111111");
  delay(ms_per);
  lix.write("222222");
  delay(ms_per);
  lix.write("333333");
  delay(ms_per);
  lix.write("444444");
  delay(ms_per);
  lix.write("555555");
  delay(ms_per);
  lix.write("666666");
  delay(ms_per);
  lix.write("777777");
  delay(ms_per);
  lix.write("888888");
  delay(ms_per);
  lix.write("999999");
  delay(ms_per);
  lix.write("......");
  delay(ms_per);
}

void self_test_beep_beep() {
  lix.clear();
  beep(500, 500);
  yield();
  beep(1000, 500);
  yield();
  beep(2000, 500);
  yield();
  beep(4000, 500);
  yield();
  beep(8000, 500);
  yield();
}

void self_test() {
  self_test_beep_beep();
  self_test_numeral_run(CRGB(255, 0, 0), 500);
  self_test_numeral_run(CRGB(0, 255, 0), 500);
  self_test_numeral_run(CRGB(0, 0, 255), 500);
}

void start_load() {
  loading_run.attach_ms(250, load_loop);
}

void end_load() {
  loading_run.detach();
}

void print_debug(char* key, char* val) {
  if (debug_output) {
    Serial.print("*HG: ");
    Serial.print(key);
    Serial.print(" = ");
    Serial.println(val);
  }
}

void print_debug(char* key, String val) {
  if (debug_output) {
    Serial.print("*HG: ");
    Serial.print(key);
    Serial.print(" = ");
    Serial.println(val);
  }
}

void print_debug(char* key, int32_t val) {
  if (debug_output) {
    Serial.print("*HG: ");
    Serial.print(key);
    Serial.print(" = ");
    Serial.println(val);
  }
}

void print_debug(char* key, uint32_t val) {
  if (debug_output) {
    Serial.print("*HG: ");
    Serial.print(key);
    Serial.print(" = ");
    Serial.println(val);
  }
}

void print_debug(char* key, float val) {
  if (debug_output) {
    Serial.print("*HG: ");
    Serial.print(key);
    Serial.print(" = ");
    Serial.println(val);
  }
}

void print_error(char* err) {
  Serial.print("*HG: ");
  Serial.print("ERROR");
  Serial.print(" = ");
  Serial.println(err);
}

void print_info(char* inf) {
  Serial.print("*HG: ");
  Serial.print("INFO");
  Serial.print(" = ");
  Serial.println(inf);
}

void print_info(uint32_t val) {
  Serial.print("*HG: ");
  Serial.print("INFO");
  Serial.print(" = ");
  Serial.println(val);
}

void print_full_device_info() {
  //print_info("print_full_device_info()");
  print_debug("RESET_REASON",       ESP.getResetReason());
  print_debug("FREE_HEAP",          ESP.getFreeHeap());
  print_debug("HEAP_FRAGMENT",      ESP.getHeapFragmentation());
  print_debug("MAX_FREE_BLOCK",     ESP.getMaxFreeBlockSize());
  print_debug("CHIP_ID",            ESP.getChipId());
  print_debug("CORE_VERSION",       ESP.getCoreVersion());
  print_debug("SDK_VERSION",        ESP.getSdkVersion());
  print_debug("CPU_FREQ",           ESP.getCpuFreqMHz());
  print_debug("FLASH_ID",           ESP.getFlashChipId());
  print_debug("FLASH_SIZE",         ESP.getFlashChipSize());
  print_debug("FLASH_SPEED",        ESP.getFlashChipSpeed());
  print_debug("VCC",                float(ESP.getVcc() / 1000.0));
  print_debug("WIFI_RSSI",          WiFi.RSSI());
  print_debug("UP_TIME",            float(millis() / 1000.0));
  print_debug("UTC_OFFSET",         clock_config.time_zone_shift);
  print_debug("12_HOUR_MODE",       clock_config.hour_12_mode);
  print_debug("HUE",                clock_config.base_hue);
  print_debug("HUE_MODE",           clock_config.current_mode);
  print_debug("6_DIGIT_CLOCK",      clock_config.six_digit_clock);
  print_debug("NIGHT_DIMMING",      !digitalRead(NIGHT_DIMMING));
  print_debug("COLOR_CYCLE",        !digitalRead(COLOR_CYCLE));
}
