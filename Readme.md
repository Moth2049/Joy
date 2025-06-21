Adaptive Feedback is worked fine by wireless connection

ff_types = {
     ecodes.FF_CONSTANT,
     ecodes.FF_PERIODIC,
     ecodes.FF_RAMP,
     ecodes.FF_SPRING,
     ecodes.FF_FRICTION,
     ecodes.FF_DAMPER,
     ecodes.FF_RUMBLE,
     ecodes.FF_INERTIA,
     ecodes.FF_CUSTOM,
 }

AdaptiveFFTest.py is allow you to input vibration intensity between 0-65535 or 0x0000-0xFFFF (minimum typically at int 300)
