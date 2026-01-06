# PID Tuning Guide for Open Servo

## Systematic PID Tuning Method for Servo Position Control

### Goal
Eliminate oscillations while maintaining responsive position control for your servo.

## Phase 1: Baseline Assessment

1. **Record current PID values** using `pid pos`
2. **Test current behavior** with `set sp 9000` (90°) and observe:
   - Oscillation frequency and amplitude
   - Overshoot amount
   - Settling time

## Phase 2: Ziegler-Nichols-inspired Manual Tuning

### Step 1: P-only Control (Find Critical Gain)

```bash
pid pos set 1.0 0 0      # Start with low P, no I or D
set sp 9000              # Test movement
state                    # Check response
```

- Gradually increase Kp: 1.0 → 2.0 → 4.0 → 8.0 → 16.0...
- Find Kc (critical gain) where sustained oscillations just begin
- Record this value and set Kp = 0.6 * Kc

### Step 2: Add Derivative Damping

```bash
pid pos set <0.6*Kc> 0 <0.1*Kc>   # Start with Kd = Kp/6
```

- Test and adjust Kd to eliminate oscillations
- Increase Kd if still oscillating (try 2x, 3x increments)
- Decrease Kd if response becomes too sluggish

### Step 3: Fine-tune Proportional

- Once oscillations stop, gradually increase Kp for faster response
- Stop when oscillations just start to return
- Back off by 20%

### Step 4: Add Integral (if needed)

```bash
pid pos set <Kp> 0.001 <Kd>   # Very small Ki
```

- Only add if steady-state error persists
- Increase slowly: 0.001 → 0.01 → 0.05
- Watch for integral windup (increasing overshoot)

## Phase 3: Response Optimization

### Test Suite Commands:

```bash
# Small step test
set sp 4500    # 45°
state
# Wait 2 seconds
set sp 5500    # 55°
state

# Large step test  
set sp 0       # 0°
state
# Wait 2 seconds
set sp 18000   # 180°
state

# Return to center
set sp 9000    # 90°
```

### Optimization Goals:

- **No oscillation**: Position doesn't cross setpoint more than once
- **Minimal overshoot**: < 5% of movement range
- **Fast settling**: Reaches ±2% of target within 200ms
- **No steady-state error**: Final position matches setpoint

## Phase 4: Robustness Testing

1. **Load variation test**: Apply gentle pressure while holding position
2. **Rapid command test**: Quick succession of setpoint changes
3. **Full range test**: 0° → 180° → 0° sweep

## Recommended Starting Values

Based on typical servo characteristics:

### For High-Speed Servo (low inertia):
```bash
pid pos set 10.0 0 3.0
```

### For Standard Servo (medium inertia):
```bash
pid pos set 5.0 0 2.0  
```

### For High-Torque Servo (high inertia):
```bash
pid pos set 3.0 0.01 1.5
```

## Troubleshooting Guide

| Problem | Solution |
|---------|----------|
| High-frequency oscillation | Reduce Kp by 50%, increase Kd by 50% |
| Low-frequency oscillation | Reduce Ki to 0, reduce Kp by 30% |
| Overshoot without oscillation | Increase Kd by 50% |
| Sluggish response | Increase Kp by 30%, ensure Ki is small |
| Steady-state error | Add small Ki (0.01-0.05) |
| Noisy/jittery | Reduce Kd, check derivative mode |

## Final Optimization

Once stable, try switching derivative mode:
```bash
pid pos mode meas    # Use measurement derivative (often smoother)
```

This reduces noise amplification from setpoint changes.

## Debug REPL Command Reference

### PID Commands:
- `pid pos` - Show current PID configuration
- `pid pos set kp <value>` - Set proportional gain
- `pid pos set ki <value>` - Set integral gain  
- `pid pos set kd <value>` - Set derivative gain
- `pid pos set <kp> <ki> <kd>` - Set all three gains at once
- `pid pos mode err|meas` - Set derivative mode

### Testing Commands:
- `set sp <cdeg>` - Set setpoint in centidegrees (9000 = 90°)
- `state` or `s` - Show system state (position, error, etc.)
- `fault` - Show fault status
- `fault clear` - Clear latched faults

### Safety Limit Commands:
- `limit` - Show all safety limits
- `limit current [mA]` - Get/set current limit
- `limit temp [dC]` - Get/set temperature limit
- `limit pos <min> <max>` - Set position bounds
- `limit stall [ticks]` - Set stall timeout
- `limit error [cdeg]` - Set max position error
- `limit reset` - Restore default limits

## Common Servo PID Values

Typical ranges for different servo types:

- **Kp (Proportional)**: 5-30 
  - Lower for high-power/high-inertia systems
  - Higher for low-power/quick-response systems

- **Ki (Integral)**: 0-0.1
  - Often 0 for position servos
  - Small values (0.01-0.05) only if steady-state error exists

- **Kd (Derivative)**: 1-10
  - Usually Kp/5 to Kp/2
  - Higher values for more damping
  - Lower if signal is noisy

## Recording Your Results

### Template for Documentation:

```markdown
## Tuned PID Values

Date: [DATE]
Servo Model: [MODEL]
Load Configuration: [DESCRIPTION]

### Final PID Values:
- Kp: [VALUE]
- Ki: [VALUE]  
- Kd: [VALUE]
- Derivative Mode: [err/meas]

### Performance Metrics:
- Overshoot: [%]
- Settling Time: [ms]
- Steady-State Error: [centidegrees]
- Oscillation: [none/damped/sustained]

### Test Conditions:
- Voltage: [V]
- Temperature: [°C]
- Load: [description]

### Notes:
[Any special observations or considerations]
```

## Tips for Success

1. **Be Patient**: Take time between adjustments to observe the full response
2. **Start Conservative**: It's easier to increase gains than recover from instability
3. **One at a Time**: Only adjust one parameter per test
4. **Document Everything**: Record what works and what doesn't
5. **Test Full Range**: A tune that works at 90° might oscillate at 0° or 180°
6. **Consider the Load**: PID values change with mechanical load
7. **Temperature Effects**: Motor and electronics behave differently when warm

## Advanced Tuning

### Gain Scheduling
For systems with varying dynamics across the range:
- Use different PID values for different position ranges
- Implement smooth transitions between gain sets

### Feed-forward Control
For predictable disturbances:
- Add velocity feed-forward for smoother motion
- Add acceleration feed-forward for very dynamic systems

### Anti-windup Strategies
If using integral control:
- Implement integral clamping
- Use conditional integration (only integrate near setpoint)
- Reset integral on large setpoint changes

## Torque Limiter Tuning

The torque limiter protects servo gears by dynamically limiting PWM duty based on motor current. It's especially important for SG90-style servos with fragile plastic gears.

### Configuration Parameters

Located in `TorqueConfig`:

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `limit_ma` | 500mA | 200-1000mA | Current threshold for triggering limiting |
| `hysteresis_ma` | 50mA | 20-100mA | Hysteresis band to prevent oscillation |
| `deglitch_samples` | 3 | 1-10 | Consecutive over-limit samples before action |
| `backoff_factor_q8` | 225 (0.88) | 128-250 | Duty reduction factor when limiting (Q8 format) |
| `recovery_rate` | 3277/s (10%/s) | 328-6554 | Duty recovery rate in units per second |

### Tuning Process

#### Step 1: Determine Safe Current Limit
```bash
# Monitor current during normal operation
state
# Apply moderate load and observe current
# Set limit ~20% above normal operating current
```

**Guidelines by servo type:**
- SG90 clones: 400-500mA (fragile plastic gears)
- MG90S: 600-800mA (metal gears, more robust)
- High-torque servos: 800-1200mA

#### Step 2: Set Deglitch Samples
- **Fast response** (1-2 samples): Risk of false triggers from transients
- **Balanced** (3 samples): Good for most applications
- **Conservative** (5+ samples): Slower response but fewer false positives

#### Step 3: Configure Backoff Factor
- **Aggressive** (0.7-0.8): Quick torque reduction, may cause position loss
- **Moderate** (0.85-0.9): **Recommended** - balanced protection
- **Gentle** (0.92-0.95): Maintains control but slower protection

#### Step 4: Set Recovery Rate
- **Fast** (20-30%/s): Quick recovery, risk of limit cycling
- **Moderate** (10%/s): **Recommended** - smooth recovery
- **Slow** (5%/s): Very stable but sluggish response

#### Step 5: Configure Hysteresis
- **Narrow** (20-30mA): More responsive but may oscillate
- **Medium** (40-60mA): **Recommended** - good stability
- **Wide** (80-100mA): Very stable but large dead band

### Testing the Limiter

#### Load Test
```bash
# Set position and gradually increase load
set sp 9000
# Apply increasing resistance by hand
# Monitor state - look for 'torque_limited: true'
state
```

#### Stall Test (Careful!)
```bash
# Command movement against mechanical stop
# Limiter should prevent excessive current
# Watch for duty cap reduction in telemetry
```

#### Recovery Test
```bash
# After limiting event, remove load
# Observe smooth recovery to full duty
# Should take (100% / recovery_rate) seconds
```

### Tuning for Different Scenarios

#### Protecting Fragile Gears (SG90)
```rust
TorqueConfig {
    limit_ma: 500,           // Low limit for protection
    hysteresis_ma: 50,       // Medium hysteresis
    deglitch_samples: 3,     // Quick response
    backoff_factor_q8: 225,  // 0.88 - moderate backoff
    recovery_rate: 3277,     // 10%/s recovery
}
```

#### High-Performance (Metal Gears)
```rust
TorqueConfig {
    limit_ma: 800,           // Higher limit for performance
    hysteresis_ma: 60,       // Wider band for stability
    deglitch_samples: 5,     // More filtering
    backoff_factor_q8: 230,  // 0.90 - gentler backoff
    recovery_rate: 6554,     // 20%/s faster recovery
}
```

#### Ultra-Safe Mode
```rust
TorqueConfig {
    limit_ma: 400,           // Very conservative
    hysteresis_ma: 40,       // Tighter control
    deglitch_samples: 2,     // Fast protection
    backoff_factor_q8: 204,  // 0.80 - aggressive backoff
    recovery_rate: 1638,     // 5%/s slow recovery
}
```

### Integration with PID

The torque limiter works by dynamically adjusting PID output limits:

1. **Normal operation**: PID has full range (-32767 to 32767)
2. **Current spike detected**: Duty cap reduced by backoff factor
3. **PID limits updated**: Prevents integral windup
4. **Recovery phase**: Limits gradually expand back to normal

### Telemetry and Debugging

Monitor limiter state through system telemetry:
- `torque_limited`: Boolean flag when limiting active
- `duty_cap`: Current duty cycle limit (0-32767)
- `limit_state`: Normal/Limiting/Recovering

### Common Issues and Solutions

| Problem | Cause | Solution |
|---------|-------|----------|
| Frequent false triggers | Limit too low or deglitch too short | Increase limit or deglitch samples |
| Gear damage still occurs | Limit too high or response too slow | Lower limit, reduce deglitch |
| Position loss during limiting | Backoff too aggressive | Increase backoff_factor (closer to 1.0) |
| Oscillating limit cycles | Recovery too fast vs hysteresis | Slow recovery rate or widen hysteresis |
| Never recovers fully | Recovery too slow | Increase recovery_rate |

### Direction Change Blanking

The limiter includes 1ms blanking after direction changes to ignore current spikes from motor inductance:
- Prevents false triggers during reversals
- Automatically applied, no tuning needed
- Can be monitored via `is_blanking()` method

### Best Practices

1. **Start Conservative**: Begin with lower current limits and adjust up
2. **Test Under Load**: Tune with realistic mechanical loads
3. **Monitor Temperature**: Current limits may need adjustment when motor is hot
4. **Consider Duty Cycle**: High duty cycle = more heat = lower safe current
5. **Verify Recovery**: Ensure servo returns to full performance after limiting
6. **Log Events**: Track limiting events to identify mechanical issues

### Safety Considerations

- Limiter protects gears but may cause temporary position errors
- Not a substitute for proper mechanical design
- Should complement, not replace, stall detection
- Regular limiting indicates mechanical problems - investigate root cause