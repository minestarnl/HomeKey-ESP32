#include "IndicatorManager.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <optional>
#include "PwmPin.h"
#include "cJSON.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"

const char* IndicatorManager::TAG = "Indicators";

// ---------------------------------------------------------------------------
// PWM output built on HomeSpan's LedC allocator, so LEDC channels and timers
// are shared safely with HomeSpan's own PWM users (e.g. the status LED).
// ---------------------------------------------------------------------------
class IndicatorPwm : public LedC {
public:
  // `tone` = true reserves an exclusive timer (odd start frequency) with a
  // fixed 10-bit resolution so the frequency can be changed per note.
  IndicatorPwm(uint8_t pin, bool tone, bool inverted)
      : LedC(pin, tone ? kToneStartFreq : kLedFreq, inverted) {
    if (!channel) return;
    if (tone) {
      timer->duty_resolution = LEDC_TIMER_10_BIT;
      if (ledc_timer_config(timer) != ESP_OK) {
        ESP_LOGE("Indicators", "Could not configure tone timer for GPIO %d", pin);
      }
    }
    channel->duty = 0;
    ledc_channel_config(channel);
  }

  bool ok() const { return channel != nullptr; }

  // duty: 0.0 - 1.0
  void setDuty(float duty) {
    if (!channel) return;
    duty = std::clamp(duty, 0.0f, 1.0f);
    const uint32_t max = (1u << timer->duty_resolution) - 1;
    const uint32_t v = static_cast<uint32_t>(std::lround(duty * max));
    channel->duty = v;
    ledc_set_duty(channel->speed_mode, channel->channel, v);
    ledc_update_duty(channel->speed_mode, channel->channel);
  }

  void setFreq(uint32_t hz) {
    if (!channel || hz == 0) return;
    ledc_set_freq(channel->speed_mode, timer->timer_num, hz);
  }

  void setInverted(bool inverted) {
    if (!channel || channel->flags.output_invert == inverted) return;
    channel->flags.output_invert = inverted;
    ledc_channel_config(channel);
  }

private:
  static constexpr uint16_t kLedFreq = 5000;
  static constexpr uint16_t kToneStartFreq = 2047;
};

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------
namespace {

std::string trim(const std::string& s) {
  size_t b = 0, e = s.size();
  while (b < e && std::isspace(static_cast<unsigned char>(s[b]))) ++b;
  while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) --e;
  return s.substr(b, e - b);
}

std::string lower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

int64_t nowMs() { return esp_timer_get_time() / 1000; }

double jsonNumber(cJSON* obj, const char* key, double fallback) {
  cJSON* it = cJSON_GetObjectItemCaseSensitive(obj, key);
  return cJSON_IsNumber(it) ? it->valuedouble : fallback;
}

const char* jsonString(cJSON* obj, const char* key) {
  cJSON* it = cJSON_GetObjectItemCaseSensitive(obj, key);
  return cJSON_IsString(it) ? it->valuestring : nullptr;
}

uint32_t clampMs(double v) {
  return static_cast<uint32_t>(std::clamp(v, 10.0, 600000.0));
}

// Parses JSON, also accepting Python-style dict text ({'a': True}) which
// Home Assistant can produce when a template result is turned into a dict
// and back into a string before it is published.
cJSON* parseLenient(const std::string& text) {
  if (cJSON* root = cJSON_ParseWithLength(text.c_str(), text.size())) return root;
  std::string fixed;
  fixed.reserve(text.size());
  for (size_t i = 0; i < text.size(); ++i) {
    const char c = text[i];
    auto startsWith = [&](const char* w) { return text.compare(i, strlen(w), w) == 0; };
    if (c == '\'') { fixed += '"'; }
    else if (startsWith("True")) { fixed += "true"; i += 3; }
    else if (startsWith("False")) { fixed += "false"; i += 4; }
    else if (startsWith("None")) { fixed += "null"; i += 3; }
    else { fixed += c; }
  }
  return cJSON_ParseWithLength(fixed.c_str(), fixed.size());
}

float gammaCorrect(float level) {
  return std::pow(std::clamp(level, 0.0f, 1.0f), 2.2f);
}

}  // namespace

// ---------------------------------------------------------------------------
// lifecycle
// ---------------------------------------------------------------------------
IndicatorManager::IndicatorManager() = default;

IndicatorManager::~IndicatorManager() {
  if (m_task) vTaskDelete(m_task);
  if (m_queue) {
    Command cmd;
    while (xQueueReceive(m_queue, &cmd, 0) == pdTRUE) delete cmd.payload;
    vQueueDelete(m_queue);
  }
}

void IndicatorManager::begin() {
  if (m_queue) return;
  m_queue = xQueueCreate(16, sizeof(Command));
  xTaskCreate(taskEntry, "indicators", 4096, this, 3, &m_task);
  ESP_LOGI(TAG, "Remote LED/buzzer control ready");
}

void IndicatorManager::submit(Target target, uint8_t pin, const std::string& payload) {
  if (!m_queue) return;
  Command cmd{target, pin, new std::string(payload)};
  if (xQueueSend(m_queue, &cmd, 0) != pdTRUE) {
    ESP_LOGW(TAG, "Command queue full, dropping command for GPIO %u", pin);
    delete cmd.payload;
  }
}

void IndicatorManager::taskEntry(void* arg) {
  static_cast<IndicatorManager*>(arg)->run();
}

void IndicatorManager::run() {
  for (;;) {
    Command cmd;
    if (xQueueReceive(m_queue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE) {
      do {
        handle(cmd);
        delete cmd.payload;
      } while (xQueueReceive(m_queue, &cmd, 0) == pdTRUE);
    }
    const int64_t now = nowMs();
    updateLeds(now);
    updateBuzzers(now);
    updateInputs(now);
  }
}

void IndicatorManager::handle(const Command& cmd) {
  if (!cmd.payload) return;
  switch (cmd.target) {
    case Target::Led: handleLed(cmd.pin, *cmd.payload); break;
    case Target::Buzzer: handleBuzzer(cmd.pin, *cmd.payload); break;
    case Target::Input: handleInput(cmd.pin, *cmd.payload); break;
  }
}

// ---------------------------------------------------------------------------
// pin claiming
// ---------------------------------------------------------------------------
IndicatorManager::Led* IndicatorManager::ledFor(uint8_t pin, std::optional<bool> inverted) {
  if (auto it = m_leds.find(pin); it != m_leds.end()) {
    if (inverted) it->second.pwm->setInverted(*inverted);
    return &it->second;
  }
  if (m_buzzers.contains(pin) || m_inputs.contains(pin)) {
    ESP_LOGW(TAG, "GPIO %u is already used as a buzzer or input", pin);
    return nullptr;
  }
  auto lease = GPIOAllocator::instance().acquire(
      static_cast<gpio_num_t>(pin), GPIO_MODE_OUTPUT, GPIOAllocator::PinRole::GpioOut,
      GPIOAllocator::PinConsumer::Hardware, "mqtt-led");
  if (!lease.has_value() || !lease->valid()) {
    ESP_LOGW(TAG, "GPIO %u is not available for an LED", pin);
    return nullptr;
  }
  gpio_hold_dis(static_cast<gpio_num_t>(pin));
  auto pwm = std::make_unique<IndicatorPwm>(pin, false, inverted.value_or(false));
  if (!pwm->ok()) {
    ESP_LOGE(TAG, "No free PWM channel for LED on GPIO %u", pin);
    return nullptr;
  }
  Led led;
  led.pwm = std::move(pwm);
  led.lease = std::move(*lease);
  ESP_LOGI(TAG, "LED output on GPIO %u%s", pin, inverted.value_or(false) ? " (inverted)" : "");
  return &m_leds.emplace(pin, std::move(led)).first->second;
}

IndicatorManager::Buzzer* IndicatorManager::buzzerFor(uint8_t pin) {
  if (auto it = m_buzzers.find(pin); it != m_buzzers.end()) return &it->second;
  if (m_leds.contains(pin) || m_inputs.contains(pin)) {
    ESP_LOGW(TAG, "GPIO %u is already used as an LED or input", pin);
    return nullptr;
  }
  auto lease = GPIOAllocator::instance().acquire(
      static_cast<gpio_num_t>(pin), GPIO_MODE_OUTPUT, GPIOAllocator::PinRole::GpioOut,
      GPIOAllocator::PinConsumer::Hardware, "mqtt-buzzer");
  if (!lease.has_value() || !lease->valid()) {
    ESP_LOGW(TAG, "GPIO %u is not available for a buzzer", pin);
    return nullptr;
  }
  gpio_hold_dis(static_cast<gpio_num_t>(pin));
  auto pwm = std::make_unique<IndicatorPwm>(pin, true, false);
  if (!pwm->ok()) {
    ESP_LOGE(TAG, "No free PWM channel/timer for buzzer on GPIO %u", pin);
    return nullptr;
  }
  Buzzer bz;
  bz.pwm = std::move(pwm);
  bz.lease = std::move(*lease);
  ESP_LOGI(TAG, "Buzzer output on GPIO %u", pin);
  return &m_buzzers.emplace(pin, std::move(bz)).first->second;
}

// ---------------------------------------------------------------------------
// LEDs
// ---------------------------------------------------------------------------
void IndicatorManager::handleLed(uint8_t pin, const std::string& raw) {
  const std::string payload = trim(raw);
  LedMode mode;
  std::optional<bool> inverted;

  if (payload.empty()) return;
  if (payload.front() == '{') {
    cJSON* root = parseLenient(payload);
    if (!root) {
      ESP_LOGW(TAG, "GPIO %u: invalid JSON: %s", pin, payload.c_str());
      return;
    }
    const char* state = jsonString(root, "state");
    mode.on = !(state && lower(state) == "off");
    mode.level = static_cast<float>(std::clamp(jsonNumber(root, "brightness", 255.0), 0.0, 255.0) / 255.0);
    if (const char* effect = jsonString(root, "effect")) {
      const std::string e = lower(effect);
      if (e == "blink") mode.effect = Effect::Blink;
      else if (e == "pulse") mode.effect = Effect::Pulse;
      else mode.effect = Effect::None;
    }
    mode.onMs = clampMs(jsonNumber(root, "on_ms", 500));
    mode.offMs = clampMs(jsonNumber(root, "off_ms", 500));
    mode.count = static_cast<uint32_t>(std::clamp(jsonNumber(root, "count", 0), 0.0, 10000.0));
    cJSON* inv = cJSON_GetObjectItemCaseSensitive(root, "inverted");
    if (cJSON_IsBool(inv)) inverted = cJSON_IsTrue(inv);
    cJSON_Delete(root);
  } else {
    const std::string p = lower(payload);
    if (p == "on" || p == "1" || p == "true") mode.on = true;
    else if (p == "off" || p == "0" || p == "false") mode.on = false;
    else {
      ESP_LOGW(TAG, "GPIO %u: unknown LED payload: %s", pin, payload.c_str());
      return;
    }
  }

  Led* led = ledFor(pin, inverted);
  if (!led) return;

  if (mode.count > 0 && mode.on) {
    led->active = mode;
    led->overlay = true;
  } else {
    mode.count = 0;
    led->base = mode;
    if (led->overlay) return;  // applied when the running overlay finishes
    led->active = mode;
  }
  led->startMs = nowMs();
  led->lastDuty = -1.0f;  // force refresh
}

void IndicatorManager::updateLeds(int64_t now) {
  for (auto& [pin, led] : m_leds) {
    LedMode& m = led.active;
    const int64_t t = now - led.startMs;
    const uint32_t period = std::max<uint32_t>(m.onMs + m.offMs, 1);

    // counted overlay finished -> back to the steady state
    if (led.overlay) {
      const int64_t total = (m.effect == Effect::None)
                                ? static_cast<int64_t>(m.onMs) * m.count
                                : static_cast<int64_t>(period) * m.count;
      if (t >= total) {
        led.active = led.base;
        led.overlay = false;
        led.startMs = now;
        led.lastDuty = -1.0f;
        continue;  // picked up next tick
      }
    }

    float level = 0.0f;
    if (m.on) {
      switch (m.effect) {
        case Effect::None:
          level = m.level;
          break;
        case Effect::Blink:
          level = (t % period) < m.onMs ? m.level : 0.0f;
          break;
        case Effect::Pulse: {
          const uint32_t phase = static_cast<uint32_t>(t % period);
          const float frac = phase < m.onMs
                                 ? static_cast<float>(phase) / std::max<uint32_t>(m.onMs, 1)
                                 : 1.0f - static_cast<float>(phase - m.onMs) / std::max<uint32_t>(m.offMs, 1);
          level = m.level * frac;
          break;
        }
      }
    }
    const float duty = gammaCorrect(level);
    if (std::fabs(duty - led.lastDuty) > 0.0005f) {
      led.pwm->setDuty(duty);
      led.lastDuty = duty;
    }
  }
}

// ---------------------------------------------------------------------------
// Buzzer
// ---------------------------------------------------------------------------
const char* IndicatorManager::builtinTone(const std::string& name) {
  static const std::map<std::string, const char*> tones = {
      {"beep", "beep:d=16,o=6,b=200:a"},
      {"double", "double:d=16,o=6,b=200:a,p,a"},
      {"accepted", "accepted:d=16,o=6,b=200:c,e,g"},
      {"denied", "denied:d=8,o=5,b=180:c,p,c"},
      {"chime", "chime:d=8,o=6,b=140:e,c"},
      {"exit", "exit:d=16,o=6,b=120:a,4p,8p"},   // ~1 beep per second
      {"entry", "entry:d=16,o=7,b=200:c,p"},     // fast beeping
      {"alarm", "alarm:d=8,o=6,b=200:a,e"},      // two-tone siren
  };
  auto it = tones.find(lower(name));
  return it == tones.end() ? nullptr : it->second;
}

bool IndicatorManager::parseRtttl(const std::string& text, std::vector<Note>& out) {
  // name:d=4,o=6,b=63:notes
  const size_t c1 = text.find(':');
  if (c1 == std::string::npos) return false;
  const size_t c2 = text.find(':', c1 + 1);
  if (c2 == std::string::npos) return false;

  int defDur = 4, defOct = 6, bpm = 63;
  {
    std::string settings = lower(text.substr(c1 + 1, c2 - c1 - 1));
    size_t pos = 0;
    while (pos < settings.size()) {
      size_t comma = settings.find(',', pos);
      if (comma == std::string::npos) comma = settings.size();
      std::string kv = trim(settings.substr(pos, comma - pos));
      if (kv.size() > 2 && kv[1] == '=') {
        const int v = std::atoi(kv.c_str() + 2);
        if (kv[0] == 'd' && v > 0) defDur = v;
        else if (kv[0] == 'o' && v >= 3 && v <= 8) defOct = v;
        else if (kv[0] == 'b' && v > 0) bpm = v;
      }
      pos = comma + 1;
    }
  }

  // semitone offsets from C for c d e f g a b
  static const int kSemis[7] = {0, 2, 4, 5, 7, 9, 11};
  const double wholeMs = 60000.0 * 4.0 / bpm;
  const std::string notes = lower(text.substr(c2 + 1));

  size_t i = 0;
  out.clear();
  while (i < notes.size()) {
    while (i < notes.size() && (notes[i] == ',' || std::isspace(static_cast<unsigned char>(notes[i])))) ++i;
    if (i >= notes.size()) break;

    int dur = 0;
    while (i < notes.size() && std::isdigit(static_cast<unsigned char>(notes[i]))) dur = dur * 10 + (notes[i++] - '0');
    if (dur == 0) dur = defDur;

    if (i >= notes.size()) break;
    const char letter = notes[i++];
    int semi = -1;
    if (letter >= 'a' && letter <= 'g') {
      static const int idx[7] = {5, 6, 0, 1, 2, 3, 4};  // a b c d e f g -> index into kSemis
      semi = kSemis[idx[letter - 'a']];
    } else if (letter == 'h') {
      semi = 11;  // some dialects use h for b
    } else if (letter != 'p') {
      // skip unknown token
      while (i < notes.size() && notes[i] != ',') ++i;
      continue;
    }

    bool dotted = false;
    if (i < notes.size() && notes[i] == '#') { if (semi >= 0) ++semi; ++i; }
    if (i < notes.size() && notes[i] == '.') { dotted = true; ++i; }
    int oct = defOct;
    if (i < notes.size() && std::isdigit(static_cast<unsigned char>(notes[i]))) oct = notes[i++] - '0';
    if (i < notes.size() && notes[i] == '.') { dotted = true; ++i; }
    while (i < notes.size() && notes[i] != ',') ++i;  // ignore trailing junk

    double ms = wholeMs / dur;
    if (dotted) ms *= 1.5;
    const uint16_t total = static_cast<uint16_t>(std::clamp(ms, 1.0, 60000.0));

    if (semi < 0) {
      out.push_back({0, total});
    } else {
      // A4 = 440 Hz is semitone 9 of octave 4
      const double freq = 440.0 * std::pow(2.0, (oct - 4) + (semi - 9) / 12.0);
      const uint16_t sound = static_cast<uint16_t>(std::max(1.0, total * 0.9));
      out.push_back({static_cast<uint16_t>(std::clamp(freq, 20.0, 20000.0)), sound});
      if (total > sound) out.push_back({0, static_cast<uint16_t>(total - sound)});
    }
  }
  return !out.empty();
}

void IndicatorManager::handleBuzzer(uint8_t pin, const std::string& raw) {
  const std::string payload = trim(raw);
  if (payload.empty()) return;

  Buzzer* bz = buzzerFor(pin);
  if (!bz) return;

  const std::string p = lower(payload);
  if (p == "stop") {
    bz->foreground.playing = false;
    bz->background.playing = false;
    bz->pwm->setDuty(0);
    return;
  }
  if (p == "off" || p == "0" || p == "false") {
    bz->background.playing = false;
    if (!bz->foreground.playing) bz->pwm->setDuty(0);
    return;
  }

  std::vector<Note> notes;
  int repeat = 1;
  int volume = 100;

  if (payload.front() == '{') {
    cJSON* root = parseLenient(payload);
    if (!root) {
      ESP_LOGW(TAG, "Buzzer GPIO %u: invalid JSON: %s", pin, payload.c_str());
      return;
    }
    repeat = static_cast<int>(jsonNumber(root, "repeat", 1));
    volume = static_cast<int>(jsonNumber(root, "volume", 100));
    if (const char* tone = jsonString(root, "tone")) {
      if (const char* r = builtinTone(tone)) parseRtttl(r, notes);
      else ESP_LOGW(TAG, "Buzzer GPIO %u: unknown tone '%s'", pin, tone);
    } else if (const char* rtttl = jsonString(root, "rtttl")) {
      parseRtttl(rtttl, notes);
    } else if (cJSON_IsNumber(cJSON_GetObjectItemCaseSensitive(root, "freq"))) {
      const uint16_t f = static_cast<uint16_t>(std::clamp(jsonNumber(root, "freq", 2000), 20.0, 20000.0));
      const uint16_t ms = static_cast<uint16_t>(std::clamp(jsonNumber(root, "ms", 150), 1.0, 60000.0));
      notes.push_back({f, ms});
      if (const double gap = jsonNumber(root, "gap_ms", 0); gap > 0) {
        notes.push_back({0, static_cast<uint16_t>(std::clamp(gap, 1.0, 60000.0))});
      }
    }
    cJSON_Delete(root);
  } else if (payload.find(':') != std::string::npos) {
    parseRtttl(payload, notes);
  } else if (const char* r = builtinTone(payload)) {
    parseRtttl(r, notes);
  }

  if (notes.empty()) {
    ESP_LOGW(TAG, "Buzzer GPIO %u: nothing to play for '%s'", pin, payload.c_str());
    return;
  }

  const bool loop = repeat <= 0;
  Sequence& seq = loop ? bz->background : bz->foreground;
  seq.notes = std::move(notes);
  seq.idx = 0;
  seq.repeatsLeft = loop ? -1 : repeat;
  seq.volume = static_cast<float>(std::clamp(volume, 0, 100)) / 100.0f;
  seq.playing = true;
  // a new background waits silently while a foreground sound is playing
  if (!loop || !bz->foreground.playing) startNote(*bz, seq, nowMs());
}

void IndicatorManager::startNote(Buzzer& bz, Sequence& seq, int64_t now) {
  const Note& n = seq.notes[seq.idx];
  if (n.freq == 0) {
    bz.pwm->setDuty(0);
  } else {
    bz.pwm->setFreq(n.freq);
    bz.pwm->setDuty(0.5f * seq.volume);  // 50% duty = loudest for a piezo
  }
  seq.noteEnd = now + n.ms;
}

// Moves `seq` to its next note. Returns false when the sequence has ended.
bool IndicatorManager::advance(Sequence& seq) {
  if (++seq.idx >= seq.notes.size()) {
    if (seq.repeatsLeft < 0 || --seq.repeatsLeft > 0) {
      seq.idx = 0;
    } else {
      seq.playing = false;
      return false;
    }
  }
  return true;
}

void IndicatorManager::updateBuzzers(int64_t now) {
  for (auto& [pin, bz] : m_buzzers) {
    if (bz.foreground.playing) {
      if (now < bz.foreground.noteEnd) continue;
      if (advance(bz.foreground)) {
        startNote(bz, bz.foreground, now);
      } else if (bz.background.playing) {
        bz.background.idx = 0;  // resume the background from its start
        startNote(bz, bz.background, now);
      } else {
        bz.pwm->setDuty(0);
      }
    } else if (bz.background.playing) {
      if (now < bz.background.noteEnd) continue;
      if (advance(bz.background)) startNote(bz, bz.background, now);
      else bz.pwm->setDuty(0);
    }
  }
}

// ---------------------------------------------------------------------------
// Inputs (e.g. door reed switch)
// ---------------------------------------------------------------------------
void IndicatorManager::handleInput(uint8_t pin, const std::string& raw) {
  const std::string payload = trim(raw);

  // empty payload = remove the input
  if (payload.empty() || lower(payload) == "off") {
    if (m_inputs.erase(pin) > 0) {
      ESP_LOGI(TAG, "Input on GPIO %u removed", pin);
      if (m_discovery) m_discovery(pin, "", "");
    }
    return;
  }
  if (m_leds.contains(pin) || m_buzzers.contains(pin)) {
    ESP_LOGW(TAG, "GPIO %u is already used as an output", pin);
    return;
  }

  std::string name = "GPIO " + std::to_string(pin);
  std::string deviceClass = "door";
  std::string pull = "up";
  bool invert = false;
  uint32_t debounce = 50;
  if (payload.front() == '{') {
    cJSON* root = parseLenient(payload);
    if (!root) {
      ESP_LOGW(TAG, "Input GPIO %u: invalid JSON: %s", pin, payload.c_str());
      return;
    }
    if (const char* v = jsonString(root, "name")) name = v;
    if (const char* v = jsonString(root, "device_class")) deviceClass = v;
    if (const char* v = jsonString(root, "pull")) pull = lower(v);
    cJSON* inv = cJSON_GetObjectItemCaseSensitive(root, "invert");
    invert = cJSON_IsTrue(inv);
    debounce = static_cast<uint32_t>(std::clamp(jsonNumber(root, "debounce_ms", 50), 0.0, 5000.0));
    cJSON_Delete(root);
  } else if (lower(payload) != "on") {
    name = payload;  // plain text = name
  }

  auto it = m_inputs.find(pin);
  if (it == m_inputs.end()) {
    auto lease = GPIOAllocator::instance().acquire(
        static_cast<gpio_num_t>(pin), GPIO_MODE_INPUT, GPIOAllocator::PinRole::GpioIn,
        GPIOAllocator::PinConsumer::Hardware, "mqtt-input");
    if (!lease.has_value() || !lease->valid()) {
      ESP_LOGW(TAG, "GPIO %u is not available for an input", pin);
      return;
    }
    Input in;
    in.lease = std::move(*lease);
    it = m_inputs.emplace(pin, std::move(in)).first;
    ESP_LOGI(TAG, "Input on GPIO %u ('%s')", pin, name.c_str());
  }

  const gpio_num_t gpio = static_cast<gpio_num_t>(pin);
  gpio_set_direction(gpio, GPIO_MODE_INPUT);
  if (pull == "down") gpio_set_pull_mode(gpio, GPIO_PULLDOWN_ONLY);
  else if (pull == "none" || pull == "off") gpio_set_pull_mode(gpio, GPIO_FLOATING);
  else gpio_set_pull_mode(gpio, GPIO_PULLUP_ONLY);

  Input& in = it->second;
  in.name = name;
  in.deviceClass = deviceClass;
  in.invert = invert;
  in.debounceMs = debounce;
  in.stableLevel = gpio_get_level(gpio);
  in.pendingLevel = in.stableLevel;

  if (m_discovery) m_discovery(pin, in.name, in.deviceClass);
  publishInput(pin, in, nowMs());  // (re)publish current state, e.g. after reconnect
}

void IndicatorManager::publishInput(uint8_t pin, Input& in, int64_t now) {
  in.lastPublish = now;
  if (!m_publish || in.stableLevel < 0) return;
  const bool on = (in.stableLevel != 0) != in.invert;
  m_publish("input/" + std::to_string(pin) + "/state", on ? "ON" : "OFF", true);
}

void IndicatorManager::updateInputs(int64_t now) {
  for (auto& [pin, in] : m_inputs) {
    const int level = gpio_get_level(static_cast<gpio_num_t>(pin));
    if (level != in.pendingLevel) {
      in.pendingLevel = level;
      in.pendingSince = now;
    }
    if (in.pendingLevel != in.stableLevel && now - in.pendingSince >= in.debounceMs) {
      in.stableLevel = in.pendingLevel;
      ESP_LOGI(TAG, "Input GPIO %u -> %s", pin, ((in.stableLevel != 0) != in.invert) ? "ON" : "OFF");
      publishInput(pin, in, now);
    } else if (now - in.lastPublish >= 60000) {
      publishInput(pin, in, now);  // periodic refresh (retained)
    }
  }
}
