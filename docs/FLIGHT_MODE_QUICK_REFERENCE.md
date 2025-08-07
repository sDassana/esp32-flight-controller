# 🚁 Quick Reference - Flight Mode Controls

## Toggle Switch Functions

| Switch       | Position | Function            | Result                            |
| ------------ | -------- | ------------------- | --------------------------------- |
| **Toggle 1** | OFF (0)  | **DISARM**          | Motors stopped, safe state        |
| **Toggle 1** | ON (1)   | **ARM**             | Motors armed, ready for flight    |
| **Toggle 2** | OFF (0)  | **Manual Mode**     | Direct control, no PID assistance |
| **Toggle 2** | ON (1)   | **Stabilized Mode** | PID-assisted self-leveling        |

## Flight Sequence

1. **Pre-flight:** Set Toggle 2 to desired flight mode
2. **ARM:** Toggle 1 to ON position
3. **Fly:** Use joysticks for control
4. **Switch Modes:** Toggle 2 during flight as needed
5. **Land:** Reduce throttle to land
6. **DISARM:** Toggle 1 to OFF position

## Serial Monitor Messages

```
🔓 MOTORS ARMED - STABILIZED MODE ACTIVE (PID ON)
🔓 MOTORS ARMED - MANUAL MODE ACTIVE (PID OFF)
🛡️ FLIGHT MODE: STABILIZED (PID ON)
🎯 FLIGHT MODE: MANUAL (PID OFF)
```

## Safety Notes

- **Toggle 1 = Primary Safety** - Always disarm when not flying
- **1-second timeout** - Motors stop if remote loses connection
- **Mode switching** - Can be done safely during flight
- **Emergency landing** - Toggle 1 OFF stops motors immediately

---

_For complete documentation, see [FLIGHT_MODE_CONTROL.md](FLIGHT_MODE_CONTROL.md)_
