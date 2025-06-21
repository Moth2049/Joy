import time
from evdev import InputDevice, ecodes, ff

# Set your correct input device path
dev = InputDevice('/dev/input/event22')
print(f"Using device: {dev.name} ({dev.path})")

duration_ms = 1000  # Duration of the effect in milliseconds
interval_ms = 500   # Time between effects

effect_types = {
    "1": "RUMBLE",
    "2": "CONSTANT",
    "3": "PERIODIC"
}

def get_int(prompt, min_val, max_val, default=None):
    while True:
        val = input(f"{prompt} [{min_val}-{max_val}]{' (default: '+str(default)+')' if default is not None else ''}: ").strip()
        if not val and default is not None:
            return default
        try:
            val = int(val, 0)
            if min_val <= val <= max_val:
                return val
            else:
                print(f"❌ Value must be between {min_val} and {max_val}.")
        except ValueError:
            print("❌ Please enter a valid number.")

print("\nSelect force feedback effect type:")
for k, v in effect_types.items():
    print(f"  {k}: {v}")

print("Type -1 to exit.\n")

while True:
    try:
        effect_choice = input("Select effect type (1/2/3 or -1 to quit): ").strip()
        if effect_choice == '-1':
            print("Exiting...")
            break
        if effect_choice not in effect_types:
            print("❌ Invalid choice.")
            continue

        effect_type = effect_types[effect_choice]

        if effect_type == "RUMBLE":
            intensity = get_int("Enter strong magnitude", 0, 0xFFFF, 0x8000)
            weak = get_int("Enter weak magnitude", 0, 0xFFFF, intensity // 2)
            rumble = ff.Rumble(strong_magnitude=intensity, weak_magnitude=weak)
            effect = ff.Effect(
                ecodes.FF_RUMBLE,
                -1,
                0,
                ff.Trigger(0, 0),
                ff.Replay(duration_ms, 0),
                ff.EffectType(ff_rumble_effect=rumble)
            )
        elif effect_type == "CONSTANT":
            level = get_int("Enter constant level (-32768 to 32767)", -32768, 32767, 0x4000)
            envelope = ff.Envelope(0, 0, 0, 0)
            constant = ff.Constant(level=level, envelope=envelope)
            effect = ff.Effect(
                ecodes.FF_CONSTANT,
                -1,
                0,
                ff.Trigger(0, 0),
                ff.Replay(duration_ms, 0),
                ff.EffectType(ff_constant_effect=constant)
            )
        elif effect_type == "PERIODIC":
            waveform = get_int("Enter waveform (0=SINE, 1=SQUARE, 2=TRIANGLE, 3=SAW_UP, 4=SAW_DOWN)", 0, 4, 0)
            period = get_int("Enter period (ms)", 1, 10000, 100)
            magnitude = get_int("Enter magnitude (-32768 to 32767)", -32768, 32767, 0x4000)
            offset = get_int("Enter offset (-32768 to 32767)", -32768, 32767, 0)
            phase = get_int("Enter phase (0-35999)", 0, 35999, 0)
            envelope = ff.Envelope(0, 0, 0, 0)
            periodic = ff.Periodic(
                waveform=waveform,
                period=period,
                magnitude=magnitude,
                offset=offset,
                phase=phase,
                envelope=envelope,
                custom_len=0,
                custom_data=None
            )
            effect = ff.Effect(
                ecodes.FF_PERIODIC,
                -1,
                0,
                ff.Trigger(0, 0),
                ff.Replay(duration_ms, 0),
                ff.EffectType(ff_periodic_effect=periodic)
            )
        else:
            print("❌ Unsupported effect type.")
            continue

        effect_id = dev.upload_effect(effect)
        dev.write(ecodes.EV_FF, effect_id, 1)

        print(f"✅ Sent {effect_type} effect (id {effect_id}).")
        time.sleep(duration_ms / 1000)

        dev.erase_effect(effect_id)
        time.sleep(interval_ms / 1000)

    except ValueError:
        print("❌ Please enter a valid number.")
    except KeyboardInterrupt:
        print("\nInterrupted by user. Exiting...")
        break
    except Exception as e:
        print(f"⚠️ Error: {e}")
        break

# ff_types = {
#     ecodes.FF_CONSTANT,
#     ecodes.FF_PERIODIC,
#     ecodes.FF_RAMP,
#     ecodes.FF_SPRING,
#     ecodes.FF_FRICTION,
#     ecodes.FF_DAMPER,
#     ecodes.FF_RUMBLE,
#     ecodes.FF_INERTIA,
#     ecodes.FF_CUSTOM,
# }