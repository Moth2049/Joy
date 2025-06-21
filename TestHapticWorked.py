import time
from evdev import InputDevice, ecodes, ff

# Replace 'event5' with your actual event number (check with `evtest` or `ls /dev/input/event*`)
dev = InputDevice('/dev/input/event22')
print(dev)
# Define the rumble effect
rumble = ff.Rumble(strong_magnitude=0x0000, weak_magnitude=0xffff)

effect = ff.Effect(
    ecodes.FF_RUMBLE,  # Effect type
    -1,                # Effect ID (auto-assign)
    0,                 # Direction (0 = default)
    ff.Trigger(0, 0),  # No special trigger
    ff.Replay(1000, 0),  # Duration in ms (1000 ms = 1 sec), no delay
    ff.EffectType(ff_rumble_effect=rumble) #Need ff_rumble_effect to worked
)

# Upload and activate the effect
effect_id = dev.upload_effect(effect)
dev.write(ecodes.EV_FF, effect_id, 1)

# Let it run for 1 second, then stop
time.sleep(1)

# Remove the effect
dev.erase_effect(effect_id)
