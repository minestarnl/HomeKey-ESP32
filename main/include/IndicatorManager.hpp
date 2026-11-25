#pragma once
/**
 * @file IndicatorManager.hpp
 * @brief LEDs and a passive buzzer driven remotely over MQTT.
 *
 * Lets an external system (e.g. Home Assistant) control status LEDs and a
 * buzzer on any free GPIO without configuring them in the web UI first.
 *
 * Topics (prefix = MQTT client ID):
 *   <clientId>/output/<pin>/set   LED command
 *   <clientId>/buzzer/<pin>/set   buzzer command
 *
 * LED payloads:
 *   "ON" / "OFF"
 *   {"state":"ON","brightness":0-255,"effect":"none|blink|pulse",
 *    "on_ms":500,"off_ms":500,"count":0,"inverted":false}
 *   count = 0  -> becomes the steady state of the pin
 *   count > 0  -> plays the effect `count` times, then returns to the
 *                 steady state (handy for accept/deny flashes); a steady
 *                 state that arrives meanwhile is applied afterwards
 *
 * Buzzer payloads (passive buzzer):
 *   "OFF"   stops the background (looping) sound
 *   "STOP"  stops everything
 *   "accepted"                      built-in tone name
 *   "name:d=8,o=6,b=180:c,e,g"      RTTTL melody
 *   {"tone":"accepted"} | {"rtttl":"..."} | {"freq":2000,"ms":150}
 *   optional: "repeat": n (default 1, 0 = until stopped), "volume": 0-100
 *   repeat 0 sets the background sound (e.g. entry-delay beeping); a finite
 *   repeat plays on top of it and the background resumes afterwards.
 *   built-in tones: beep, double, accepted, denied, chime, exit, entry, alarm
 *
 * Pins are claimed through GPIOAllocator, so pins in use by the NFC reader,
 * ethernet or other actions are refused.
 */
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "GPIOAllocator.hpp"

class IndicatorPwm;

class IndicatorManager {
public:
  enum class Target : uint8_t { Led, Buzzer };

  IndicatorManager();
  ~IndicatorManager();

  void begin();

  /** Queue a command; safe to call from any task (e.g. the MQTT task). */
  void submit(Target target, uint8_t pin, const std::string& payload);

private:
  struct Command {
    Target target;
    uint8_t pin;
    std::string* payload;
  };

  enum class Effect : uint8_t { None, Blink, Pulse };

  struct LedMode {
    bool on = false;
    float level = 1.0f;
    Effect effect = Effect::None;
    uint32_t onMs = 500;
    uint32_t offMs = 500;
    uint32_t count = 0;
  };

  struct Led {
    std::unique_ptr<IndicatorPwm> pwm;
    GPIOAllocator::GPIOLease lease;
    LedMode base;            // steady state
    LedMode active;          // what is playing now (base or a counted overlay)
    bool overlay = false;
    int64_t startMs = 0;
    float lastDuty = -1.0f;
  };

  struct Note {
    uint16_t freq;           // 0 = rest
    uint16_t ms;
  };

  struct Sequence {
    std::vector<Note> notes;
    size_t idx = 0;
    int64_t noteEnd = 0;
    int32_t repeatsLeft = 0; // -1 = forever
    float volume = 1.0f;
    bool playing = false;
  };

  struct Buzzer {
    std::unique_ptr<IndicatorPwm> pwm;
    GPIOAllocator::GPIOLease lease;
    Sequence background;     // repeat = 0 (loops until OFF), e.g. entry delay
    Sequence foreground;     // finite repeat, plays over the background
  };

  static void taskEntry(void* arg);
  void run();
  void handle(const Command& cmd);
  void handleLed(uint8_t pin, const std::string& payload);
  void handleBuzzer(uint8_t pin, const std::string& payload);
  void updateLeds(int64_t now);
  void updateBuzzers(int64_t now);
  void startNote(Buzzer& bz, Sequence& seq, int64_t now);
  static bool advance(Sequence& seq);

  Led* ledFor(uint8_t pin, std::optional<bool> inverted);
  Buzzer* buzzerFor(uint8_t pin);

  static bool parseRtttl(const std::string& text, std::vector<Note>& out);
  static const char* builtinTone(const std::string& name);

  std::map<uint8_t, Led> m_leds;
  std::map<uint8_t, Buzzer> m_buzzers;
  QueueHandle_t m_queue = nullptr;
  TaskHandle_t m_task = nullptr;

  static const char* TAG;
};
