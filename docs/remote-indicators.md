# Remote LEDs and buzzer over MQTT

LEDs and a passive buzzer on any free GPIO can be driven directly over MQTT,
for example from Home Assistant. Nothing has to be configured in the web UI:
the first command for a pin claims it (pins used by the NFC reader, ethernet
or other actions are refused).

Topics use the MQTT client ID as prefix:

| Topic | Purpose |
|---|---|
| `<clientId>/output/<pin>/set` | LED on that GPIO |
| `<clientId>/buzzer/<pin>/set` | passive buzzer on that GPIO |

## LED

Plain `ON` / `OFF`, or JSON:

```json
{"state": "ON", "brightness": 255, "effect": "blink", "on_ms": 500, "off_ms": 500, "count": 0, "inverted": false}
```

- `effect`: `none` (solid), `blink`, `pulse` (fade up in `on_ms`, down in `off_ms`)
- `count: 0` makes this the pin's steady state
- `count > 0` plays the effect that many times, then returns to the steady state
  (e.g. a green double blink on top of whatever the LED was showing). A steady
  state sent while it plays is applied once it finishes.
- `inverted: true` for LEDs wired to 3.3V (active low); remembered per pin

## Buzzer (passive)

- `OFF` stops the background sound, `STOP` stops everything
- a built-in tone name: `beep`, `double`, `accepted`, `denied`, `chime`, `exit`, `entry`, `alarm`
- an RTTTL melody: `name:d=8,o=6,b=180:c,e,g`
- JSON: `{"tone": "entry", "repeat": 0, "volume": 80}`, `{"rtttl": "..."}`
  or `{"freq": 2000, "ms": 150, "gap_ms": 50}`
  - `repeat`: times to play (default 1), `0` = until `OFF`
  - `volume`: 0-100

A sound with `repeat: 0` becomes the background (e.g. entry-delay beeping).
A sound with a finite `repeat` plays on top of it, and the background resumes
afterwards.

Payloads may also be Python-style dict text (`{'tone': 'entry'}`), which Home
Assistant sometimes publishes for templated JSON.

## Inputs (door contact, button, …)

Configure an input with a **retained** message, so it is restored after every
reboot/reconnect:

| Topic | Payload |
|---|---|
| `<clientId>/input/<pin>/config` | `{"name": "Server room door", "device_class": "door", "pull": "up", "invert": false, "debounce_ms": 50}` |

- The input appears in Home Assistant as a `binary_sensor` on the reader's
  device (MQTT discovery), state on `<clientId>/input/<pin>/state` (`ON`/`OFF`, retained).
- Default: internal pull-up, `ON` when the pin is HIGH. A reed switch between
  the pin and GND therefore reads `OFF` (closed) with the magnet near and `ON`
  (open) when the door opens. `invert: true` flips this.
- `pull`: `up` (default), `down` or `none`.
- An empty retained payload on the config topic removes the input and its entity.
- On the classic ESP32, GPIO 34–39 have no internal pull-ups; use another pin
  or an external resistor.
