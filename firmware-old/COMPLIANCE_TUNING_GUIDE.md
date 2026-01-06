# Human Backdrive Compliance Tuning Guide

## Overview

The servo now implements a compliance system that allows it to detect and yield to human backdrive while maintaining strong control during commanded motion. The system uses three operating modes:

- **MOVE**: Full torque when actively moving to a new setpoint
- **HOLD**: Reduced torque when holding position steady
- **YIELD**: Temporary compliance when backdrive is detected

## Key Concepts

### Mode Transitions

```
MOVE → HOLD: When position settles (error < 5°, velocity < 10°/s for 300ms)
HOLD → MOVE: When setpoint changes or error/velocity exceeds thresholds
HOLD → YIELD: When backdrive detected (high velocity opposing control)
YIELD → HOLD: After 200ms yield duration expires
```

### Current Limiting

The system uses dual current limits:
- **Move mode**: 800mA (default) - Full power for commanded motion
- **Hold mode**: 150mA (default) - Reduced power when holding position

### Error-Based Duty Cap (Hold Mode Only)

In addition to current limiting, HOLD mode applies a duty cycle cap that increases with error:
- At 0-5° error: 20% duty cap
- At 5-15° error: Linear ramp from 20% to 45%
- Above 15° error: 45% duty cap

This prevents the "push harder → fight harder" escalation that can damage gears.

## Tuning Parameters

### Current Limits

```bash
# Adjust move mode current limit (100-1500mA)
compliance move_ma 800

# Adjust hold mode current limit (100-800mA)  
compliance hold_ma 150
```

**Tuning tips:**
- Start conservative (150mA hold) to protect gears
- Increase hold limit if servo feels too weak when holding position
- Decrease if gears still slip when backdriving

### Velocity Thresholds (in code)

```rust
// Mode transition thresholds
const HOLD_ENTER_VEL_DPS10: i16 = 100;    // 10°/s to enter HOLD
const HOLD_EXIT_VEL_DPS10: i16 = 150;     // 15°/s to exit HOLD
const BACKDRIVE_VEL_THRESHOLD: i16 = 300;  // 30°/s triggers YIELD
```

**Tuning tips:**
- Increase `BACKDRIVE_VEL_THRESHOLD` if servo yields too easily
- Decrease if servo fights too hard before yielding
- Add hysteresis (different enter/exit values) to prevent mode flapping

### Timing Parameters (in code)

```rust
const SETPOINT_SETTLE_TICKS: u32 = 4000;  // 400ms to consider settled
const HOLD_ENTRY_TICKS: u32 = 3000;       // 300ms of settled before HOLD
const YIELD_DURATION_TICKS: u32 = 2000;   // 200ms yield duration
```

**Tuning tips:**
- Increase `HOLD_ENTRY_TICKS` if servo enters HOLD too early during moves
- Adjust `YIELD_DURATION_TICKS` for feel (longer = more compliant)

### Error Thresholds (in code)

```rust
const HOLD_ENTER_ERROR_CDEG: i16 = 500;   // 5° error to enter HOLD
const HOLD_EXIT_ERROR_CDEG: i16 = 700;    // 7° error to exit HOLD
```

**Tuning tips:**
- Tighter thresholds = more precise but may oscillate between modes
- Wider thresholds = more stable but less responsive

## Debug Commands

Monitor compliance behavior:

```bash
# Show current mode and velocity
compliance show

# Example output:
# mode: HOLD, vel: 2 dps
# move: 800mA, hold: 350mA
```

## Tuning Process

1. **Start with defaults**
   - Move: 800mA, Hold: 150mA
   - Test basic operation

2. **Tune hold current**
   - Set servo to hold position
   - Try backdriving gently
   - Should resist but yield smoothly
   - Reduce if gears slip, increase if too weak

3. **Tune backdrive detection**
   - In HOLD mode, push the horn
   - Should detect backdrive and coast briefly
   - Adjust `BACKDRIVE_VEL_THRESHOLD` if needed

4. **Tune mode transitions**
   - Command moves and observe mode changes
   - Should enter HOLD after settling
   - Should return to MOVE on new commands

5. **Fine-tune duty cap curve**
   - In HOLD mode, push gradually harder
   - Force should increase but plateau
   - Adjust `HOLD_DUTY_MIN/MAX` if needed

## Troubleshooting

### Servo too weak when holding
- Increase `hold_ma` limit
- Check if entering HOLD too early (increase `HOLD_ENTRY_TICKS`)

### Gears still slip when backdriving
- Reduce `hold_ma` limit
- Lower `BACKDRIVE_VEL_THRESHOLD` for earlier detection
- Reduce `HOLD_DUTY_MAX` cap

### Servo oscillates between modes
- Add more hysteresis (increase gap between enter/exit thresholds)
- Increase `HOLD_ENTRY_TICKS` for stability

### Servo doesn't yield to backdrive
- Check `compliance show` - is it in HOLD mode?
- Reduce `BACKDRIVE_VEL_THRESHOLD`
- Verify velocity estimation is working

### Servo feels "dead" when yielding
- Normal for first 100ms (pure coast)
- After 100ms, small duty (5%) is applied
- Adjust `YIELD_DURATION_TICKS` for different feel

## Safety Notes

- Always start with conservative current limits
- Test thoroughly before increasing limits
- Monitor for gear wear/damage
- Consider mechanical limits of your specific hardware