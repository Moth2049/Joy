import time
from evdev import InputDevice, ecodes, ff

# Set your correct input device path
dev = InputDevice('/dev/input/event22')
print(f"Using device: {dev.name} ({dev.path})")

duration_ms = 1000  # Duration of the vibration in milliseconds
interval_ms = 500  # Time between vibrations

print("\nEnter intensity (0 - 65535, hex like 0xFFFF also accepted). Type -1 to exit.\n")

while True:
    try:
        user_input = input("Enter intensity (or -1 to quit): ").strip()

        if user_input == '-1':
            print("Exiting...")
            break

        # Support both decimal and hex input
        intensity = int(user_input, 0)

        if not (0 <= intensity <= 0xFFFF):
            print("❌ Invalid intensity. Please enter a value between 0 and 65535 (0x0000 to 0xFFFF).")
            continue

        print(f"✅ Vibrating with intensity: 0x{intensity:04X} ({intensity})")

        rumble = ff.Rumble(strong_magnitude=intensity, weak_magnitude=intensity // 2)
        effect = ff.Effect(
            ecodes.FF_RUMBLE,
            -1,
            0,
            ff.Trigger(0, 0),
            ff.Replay(duration_ms, 0),
            ff.EffectType(ff_rumble_effect=rumble)
        )

        effect_id = dev.upload_effect(effect)
        dev.write(ecodes.EV_FF, effect_id, 1)

        time.sleep(duration_ms / 1000)

        dev.erase_effect(effect_id)
        time.sleep(interval_ms / 1000)

    except ValueError:
        print("❌ Please enter a valid number (e.g., 30000 or 0x7530).")
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