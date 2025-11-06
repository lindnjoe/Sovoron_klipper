# Installation Checklist & Helper Commands

## Pre-Installation

### Backup Your Files
```bash
# Create backup directory
mkdir -p ~/phase1_phase5_backup

# Backup files we'll modify
cp ~/printer_data/config/AFC/extras/openams_integration.py ~/phase1_phase5_backup/
cp ~/printer_data/config/AFC/extras/AFC_OpenAMS.py ~/phase1_phase5_backup/
cp ~/klipper/klippy/extras/oams.py ~/phase1_phase5_backup/

echo "Backup created in ~/phase1_phase5_backup/"
ls -lh ~/phase1_phase5_backup/
```

### Verify Current Setup
```bash
# Check if files exist
echo "Checking files..."
ls -l ~/printer_data/config/AFC/extras/openams_integration.py
ls -l ~/printer_data/config/AFC/extras/AFC_OpenAMS.py
ls -l ~/klipper/klippy/extras/oams.py

# Check Klipper is running
systemctl status klipper | grep Active
```

---

## Installation Steps

### □ Step 1: Replace openams_integration.py

```bash
# Copy new file
cp /path/to/openams_integration.py ~/printer_data/config/AFC/extras/openams_integration.py

# Verify
ls -lh ~/printer_data/config/AFC/extras/openams_integration.py
# Should be ~30KB

# Check syntax
python3 ~/printer_data/config/AFC/extras/openams_integration.py
# Should show no errors (or just import errors which are OK)
```

**✓ Completed:** [ ]

---

### □ Step 2: Patch AFC_OpenAMS.py

Follow `AFC_OpenAMS_phase1_phase5_integration_guide.md`

**Required Changes:**

1. **Imports** (line ~34-38)
   ```python
   # Add: LaneRegistry, AMSEventBus
   ```

2. **__init__** (line ~225-268)
   ```python
   # Add: self.registry and self.event_bus
   # Remove: comment out self._lane_by_index
   ```

3. **handle_connect** (line ~347-364)
   ```python
   # Replace: _lane_by_index loop with registry.register_lane()
   ```

4. **_find_lane_by_spool** (line ~1524-1526)
   ```python
   # Replace: use registry instead of _lane_by_index
   ```

5. **Add event handlers** (before load_config_prefix)
   ```python
   # Add: _handle_spool_loaded_event()
   # Add: _handle_spool_unloaded_event()
   ```

6. **Optional: Reduce polling** (line ~230-234)
   ```python
   # Increase: SYNC_INTERVAL from 2.0 to 4.0
   ```

**Verification Commands:**
```bash
# Check syntax
python3 ~/printer_data/config/AFC/extras/AFC_OpenAMS.py
# Should not crash (import errors are OK)

# Count changes (should see your additions)
grep -n "LaneRegistry" ~/printer_data/config/AFC/extras/AFC_OpenAMS.py
grep -n "AMSEventBus" ~/printer_data/config/AFC/extras/AFC_OpenAMS.py
grep -n "def _handle_spool_loaded_event" ~/printer_data/config/AFC/extras/AFC_OpenAMS.py
```

**✓ Completed:** [ ]

---

### □ Step 3: Patch oams.py (Optional but Recommended)

Follow `oams_phase5_event_integration_guide.md`

**Required Changes:**

1. **Imports** (line ~16-19)
   ```python
   # Add: from extras.openams_integration import AMSEventBus
   ```

2. **__init__** (line ~158-169)
   ```python
   # Add: self.event_bus = AMSEventBus.get_instance()
   ```

3. **load_spool** (line ~595-608)
   ```python
   # Add: event_bus.publish("spool_loaded", ...)
   ```

4. **unload_spool** (line ~627-640)
   ```python
   # Add: event_bus.publish("spool_unloaded", ...)
   ```

**Verification Commands:**
```bash
# Check syntax
python3 ~/klipper/klippy/extras/oams.py
# Should not crash

# Count changes
grep -n "AMSEventBus" ~/klipper/klippy/extras/oams.py
grep -n "event_bus.publish" ~/klipper/klippy/extras/oams.py
```

**✓ Completed:** [ ]

---

### □ Step 4: Restart Klipper

```bash
# Restart Klipper
sudo systemctl restart klipper

# Wait a few seconds, then check status
sleep 5
systemctl status klipper | grep Active
# Should show "active (running)"

# Check for errors in log
tail -50 ~/printer_data/logs/klippy.log | grep -i error
# Should be empty or only old errors
```

**✓ Completed:** [ ]

---

## Post-Installation Verification

### □ Test 1: Registry is Initialized

```python
# In Klipper console (http://your-printer/console)
registry = printer.lookup_object("AFC_OpenAMS AMS_1").registry
print(f"Registry: {registry}")
# Should show: <LaneRegistry object at 0x...>
```

**✓ Passed:** [ ]

---

### □ Test 2: Lanes are Registered

```python
# In Klipper console
registry = printer.lookup_object("AFC_OpenAMS AMS_1").registry
lanes = registry.get_all_lanes()
print(f"Registered lanes: {[l.lane_name for l in lanes]}")
# Should show: ['lane4', 'lane6', 'lane7', 'lane8'] or similar
```

**Expected in logs:**
```
LaneRegistry: Registered lane: lane4 → AMS_1[0] → T4 (extruder=extruder4, fps=fps1)
LaneRegistry: Registered lane: lane6 → AMS_1[1] → T6 (extruder=extruder4, fps=fps1)
...
```

**✓ Passed:** [ ]

---

### □ Test 3: Registry Lookups Work

```python
# In Klipper console
registry = printer.lookup_object("AFC_OpenAMS AMS_1").registry

# Lookup by group
lane_info = registry.get_by_group("T4")
print(f"T4 -> {lane_info.lane_name if lane_info else 'NOT FOUND'}")
# Should show: T4 -> lane4

# Lookup by spool index
lane_info = registry.get_by_spool("AMS_1", 0)
print(f"AMS_1[0] -> {lane_info.lane_name if lane_info else 'NOT FOUND'}")
# Should show: AMS_1[0] -> lane4

# Lookup by lane name
lane_info = registry.get_by_lane("lane4")
print(f"lane4 -> group={lane_info.group if lane_info else 'NOT FOUND'}")
# Should show: lane4 -> group=T4
```

**✓ Passed:** [ ]

---

### □ Test 4: Event Bus is Initialized

```python
# In Klipper console
event_bus = printer.lookup_object("AFC_OpenAMS AMS_1").event_bus
print(f"Event bus: {event_bus}")
# Should show: <AMSEventBus object at 0x...>

# Check for subscribers
print(f"Subscribers: {list(event_bus._subscribers.keys())}")
# Should show: ['spool_loaded', 'spool_unloaded']
```

**✓ Passed:** [ ]

---

### □ Test 5: Events are Published

```python
# In Klipper console - set up test callback
def test_callback(event_type, **kwargs):
    print(f"TEST: {event_type} - {kwargs}")

event_bus = printer.lookup_object("AFC_OpenAMS AMS_1").event_bus
event_bus.subscribe("spool_loaded", test_callback)
event_bus.subscribe("spool_unloaded", test_callback)

# Now manually load/unload a spool and watch console
# Or check history:
history = event_bus.get_history()
print(f"Total events: {len(history)}")
for event_type, time, data in history[-5:]:
    print(f"  {event_type}: {data}")
```

**✓ Passed:** [ ]

---

### □ Test 6: Actual Spool Load/Unload

```bash
# In Klipper console or via Mainsail/Fluidd
OAMS_LOAD_SPOOL SPOOL=0

# Check console output and logs
# Should see event published

# Then unload
OAMS_UNLOAD_SPOOL

# Check again
```

**Expected in logs:**
```
AMSEventBus: Published 'spool_loaded': 2 handlers notified
AFC_OpenAMS: Received spool_loaded event: unit=oams1, spool=0
```

**✓ Passed:** [ ]

---

### □ Test 7: Performance Check

```bash
# Monitor CPU usage before and after
top -b -n 1 | grep klipper

# Should see lower CPU usage when idle
# Before: typically 0.5-1.0%
# After: typically <0.1%
```

**✓ Passed:** [ ]

---

## Troubleshooting Commands

### If Registry Not Working

```python
# Check if registry module loaded
import sys
print("openams_integration" in sys.modules)
# Should be True

# Check registry instance
from extras.openams_integration import LaneRegistry
print(LaneRegistry._instances)
# Should show at least one entry

# Manual test
registry = LaneRegistry.for_printer(printer)
registry.register_lane("test_lane", "test_unit", 0, "T99", "extruder4")
info = registry.get_by_group("T99")
print(f"Manual test: {info.lane_name if info else 'FAILED'}")
# Should show: Manual test: test_lane
```

### If Events Not Working

```python
# Check event bus
from extras.openams_integration import AMSEventBus
event_bus = AMSEventBus.get_instance()

# Manual publish test
def test_handler(event_type, **kwargs):
    print(f"RECEIVED: {event_type}")

event_bus.subscribe("test_event", test_handler)
count = event_bus.publish("test_event", test_data="hello")
print(f"Handlers notified: {count}")
# Should show: Handlers notified: 1
# Should see: RECEIVED: test_event

# Check history
history = event_bus.get_history(event_type="test_event")
print(f"Test events: {len(history)}")
# Should show: Test events: 1
```

### If Syntax Errors

```bash
# Check Python syntax
python3 -m py_compile ~/printer_data/config/AFC/extras/openams_integration.py
python3 -m py_compile ~/printer_data/config/AFC/extras/AFC_OpenAMS.py
python3 -m py_compile ~/klipper/klippy/extras/oams.py

# Should complete without errors
```

### View Full Error Log

```bash
# See all errors since last restart
journalctl -u klipper -b | grep -i error

# Real-time log watching
tail -f ~/printer_data/logs/klippy.log
```

---

## Rollback Procedure

If something goes wrong:

```bash
# Stop Klipper
sudo systemctl stop klipper

# Restore backups
cp ~/phase1_phase5_backup/openams_integration.py ~/printer_data/config/AFC/extras/
cp ~/phase1_phase5_backup/AFC_OpenAMS.py ~/printer_data/config/AFC/extras/
cp ~/phase1_phase5_backup/oams.py ~/klipper/klippy/extras/

# Start Klipper
sudo systemctl start klipper

# Verify it's working
systemctl status klipper
```

---

## Success Criteria

All of these should be ✓:

- [ ] Klipper starts without errors
- [ ] Registry initialized and shows lanes
- [ ] Registry lookups return correct info
- [ ] Event bus initialized
- [ ] Events published on spool load/unload
- [ ] Actual loading/unloading still works
- [ ] CPU usage lower when idle
- [ ] No errors in logs

---

## Optional: Enable Debug Logging

```python
# In Klipper console
SET_LOG_LEVEL COMPONENT=LaneRegistry LEVEL=DEBUG
SET_LOG_LEVEL COMPONENT=AMSEventBus LEVEL=DEBUG

# Now watch logs for detailed info
# To disable: SET_LOG_LEVEL COMPONENT=LaneRegistry LEVEL=INFO
```

---

## Performance Baseline

### Before Installation

```bash
# Record current CPU usage
top -b -n 10 -d 1 | grep klipper | awk '{print $9}' > /tmp/cpu_before.txt
cat /tmp/cpu_before.txt
```

### After Installation

```bash
# Record new CPU usage
top -b -n 10 -d 1 | grep klipper | awk '{print $9}' > /tmp/cpu_after.txt
cat /tmp/cpu_after.txt

# Compare
echo "Before:"
awk '{s+=$1}END{print s/NR"%"}' /tmp/cpu_before.txt
echo "After:"
awk '{s+=$1}END{print s/NR"%"}' /tmp/cpu_after.txt
```

---

## Documentation Reference

Quick links to docs:

1. **Start here**: `QUICK_REFERENCE.md`
2. **Full guide**: `README_PHASE1_PHASE5.md`
3. **AFC patching**: `AFC_OpenAMS_phase1_phase5_integration_guide.md`
4. **OAMS patching**: `oams_phase5_event_integration_guide.md`
5. **Architecture**: `ARCHITECTURE_DIAGRAMS.md`
6. **Summary**: `IMPLEMENTATION_SUMMARY.md`

---

## Support

If you need help:

1. Check logs: `tail -100 ~/printer_data/logs/klippy.log`
2. Run verification tests above
3. Try rollback procedure
4. Check documentation files
5. Enable debug logging and capture output

---

## Completion

Date completed: _________________

Notes:
_____________________________________
_____________________________________
_____________________________________

Issues encountered:
_____________________________________
_____________________________________
_____________________________________

Performance improvement observed:
_____________________________________
_____________________________________
_____________________________________
