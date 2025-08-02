# 🚁 ESP32 Flight Controller - Quick Setup Guide

## 📋 Pre-Flight Setup

### 1. Hardware Preparation

- [ ] **Battery**: Ensure 3S LiPo is charged (>11.5V)
- [ ] **Propellers**: Remove all propellers for initial testing
- [ ] **Connections**: Verify all sensor connections
- [ ] **ESCs**: Check ESC-to-motor wiring is correct
- [ ] **Secure Mounting**: Ensure drone is properly secured for testing

### 2. Software Upload

```bash
# Upload the flight controller code
pio run --target upload

# Monitor serial output
pio device monitor
```

### 3. Initial Testing Sequence

#### Phase 1: Component Verification

1. **Power up** and check serial output for component status
2. **Open web dashboard** at `http://[ESP32_IP]`
3. **Verify sensor readings** - all values should be reasonable
4. **Test LED/Buzzer** - ensure status indicators work

#### Phase 2: Motor Testing (NO PROPELLERS!)

1. **ARM motors** using web interface
2. **Test individual motor sliders** - each motor should respond
3. **Check motor directions** match configuration:
   - Motor 1 (Front Right): GPIO 13
   - Motor 2 (Back Right): GPIO 12
   - Motor 3 (Front Left): GPIO 14
   - Motor 4 (Back Left): GPIO 27
4. **Test emergency stop** - should immediately cut power
5. **DISARM motors** when complete

#### Phase 3: PID Tuning

1. **Start with conservative P values** (Roll/Pitch: 2.0, Yaw: 1.0)
2. **Enable STABILIZE mode**
3. **Gradually increase throttle** and observe response
4. **Tune PID parameters** using web interface
5. **Look for oscillations** - reduce gains if present

## 🔧 Initial PID Values (Starting Point)

```cpp
// Conservative starting values - tune based on drone response
Roll/Pitch PID:  P=2.0, I=0.0, D=0.1
Yaw PID:        P=1.0, I=0.0, D=0.05
Altitude PID:   P=1.0, I=0.0, D=0.1
```

## ⚠️ Safety Checklist

### Before Each Test

- [ ] Battery voltage >11.5V
- [ ] No propellers attached (initial testing)
- [ ] Drone securely mounted/held
- [ ] Emergency stop button accessible
- [ ] Clear area around test setup
- [ ] Serial monitor open for debugging

### During Testing

- [ ] Monitor battery voltage continuously
- [ ] Watch for excessive motor heat
- [ ] Check for abnormal vibrations
- [ ] Verify control response is correct
- [ ] Test emergency procedures

### Emergency Procedures

1. **Emergency Stop**: Press web interface button or power cycle
2. **Communication Loss**: Automatic emergency landing after 2 seconds
3. **Low Battery**: Automatic emergency landing at <10%
4. **Excessive Tilt**: Automatic emergency stop at ±60°

## 📊 Expected Performance

### Control Loop Performance

- **Control Frequency**: 100Hz (10ms periods)
- **Sensor Update**: 200Hz (5ms periods)
- **Web Dashboard**: 10Hz updates
- **Latency**: <50ms command to motor response

### Flight Characteristics

- **Roll/Pitch Authority**: ±30° maximum angle
- **Yaw Rate**: ±180°/sec maximum
- **Throttle Response**: Linear 0-100%
- **Altitude Hold**: ±0.5m accuracy target

## 🐛 Troubleshooting

### Common Issues

1. **Motors don't respond**: Check arming sequence and throttle position
2. **Oscillations**: Reduce P gain, increase D gain slightly
3. **Slow response**: Increase P gain gradually
4. **Altitude drift**: Check barometric sensor calibration
5. **Web interface slow**: Check WiFi signal strength

### Debug Information

```bash
# Check task performance
System running - Free heap: XXXXX bytes
Motors armed: YES/NO
Flight Mode: MANUAL/STABILIZE/ALTITUDE_HOLD
Roll: XX.X°, Pitch: XX.X°, Alt: XX.Xm
```

## 📈 Tuning Process

### Step 1: P-Only Tuning

1. Set I=0, D=0
2. Increase P until slight oscillation appears
3. Reduce P by 20-30%

### Step 2: Add Derivative

1. Add small D term (start with 0.1)
2. Increase D to reduce overshoot
3. Stop when noise becomes excessive

### Step 3: Add Integral (if needed)

1. Add very small I term (start with 0.01)
2. Use only if steady-state error exists
3. Too much I causes slow oscillations

Remember: **Safety first!** Always start with low gains and increase gradually. Test without propellers until you're confident in the system behavior.
