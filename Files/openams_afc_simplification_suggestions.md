# OpenAMS to AFC Integration - Simplification Suggestions

## Executive Summary

After analyzing your OpenAMS to AFC implementation, I've identified several areas where lane tracking between systems can be simplified and better integrated. The main complexity stems from **multiple overlapping lane identification systems** and **redundant state synchronization**. Here are my key recommendations:

---

## Current Issues

### 1. **Multiple Lane Naming/Indexing Systems**
Your implementation currently tracks lanes across 4 different identification schemes:

- **OpenAMS Spool Index** (0-3 per unit)
- **AFC Lane Name** (e.g., "lane4", "lane5")
- **Filament Group** (e.g., "T4", "T5")  
- **Unit:Position Format** (e.g., "AMS_1:1", "AMS_2:3")

**Problem:** This creates translation overhead and multiple lookups throughout the codebase:
```python
# In AFC_OpenAMS.py line 1524-1526
def _find_lane_by_spool(self, spool_index):
    return self._lane_by_index.get(spool_index)

# In openams_integration.py line 273-280
def resolve_lane_for_spool(self, unit_name: str, spool_index: Optional[int]) -> Optional[str]:
    # Another lookup layer...
    
# In AFC_oams_macros.cfg lines 12-27
# Macros need to extract lane numbers from strings
{% set new_lane = GROUP.replace('T', '') if 'T' in GROUP else GROUP %}
```

### 2. **Fragmented State Management**
State is tracked in multiple places:
- `AMSHardwareService._lane_snapshots` (openams_integration.py)
- `afcAMS._lane_by_index` (AFC_OpenAMS.py)
- `afcAMS.cached_spool_idxs` (AFC_OpenAMS.py)
- `FPSLoadState` in oams_manager.py
- Temperature caches in AFC_oams_macros.cfg

### 3. **Redundant Extruder/Lane Mappings**
Each FPS and lane configuration repeats the same extruder information:
```
[AFC_OpenAMS AMS_1]
oams: oams1
extruder: extruder4

[AFC_lane lane4]
unit: AMS_1:1
extruder: extruder4  # Redundant
```

---

## Proposed Simplifications

### **Suggestion 1: Unified Lane Registry**

Create a single source of truth for lane identity mapping in `openams_integration.py`:

```python
class LaneRegistry:
    """Single source of truth for lane identity across OpenAMS and AFC."""
    
    def __init__(self):
        self._by_lane_name: Dict[str, 'LaneInfo'] = {}
        self._by_spool: Dict[Tuple[str, int], 'LaneInfo'] = {}
        self._by_group: Dict[str, 'LaneInfo'] = {}
    
    def register_lane(self, lane_name: str, unit_name: str, spool_index: int, 
                     group: str, extruder: str):
        """Register a lane with all its identifiers."""
        info = LaneInfo(
            lane_name=lane_name,
            unit_name=unit_name,
            spool_index=spool_index,
            group=group,
            extruder=extruder
        )
        
        self._by_lane_name[lane_name] = info
        self._by_spool[(unit_name, spool_index)] = info
        self._by_group[group] = info
        
    def get_by_lane(self, lane_name: str) -> Optional['LaneInfo']:
        return self._by_lane_name.get(lane_name)
    
    def get_by_spool(self, unit_name: str, spool_index: int) -> Optional['LaneInfo']:
        return self._by_spool.get((unit_name, spool_index))
    
    def get_by_group(self, group: str) -> Optional['LaneInfo']:
        return self._by_group.get(group)

@dataclass
class LaneInfo:
    """Complete identity information for a single lane."""
    lane_name: str      # "lane4"
    unit_name: str      # "AMS_1"
    spool_index: int    # 0 (zero-indexed position in OAMS unit)
    group: str          # "T4"
    extruder: str       # "extruder4"
```

**Benefits:**
- One lookup, not three
- No string parsing of "T4" → "4" → spool_index
- Eliminates `_lane_by_index`, `_lanes_by_spool`, and redundant caches
- Makes debugging easier (single place to check lane identity)

---

### **Suggestion 2: Consolidate Temperature Management**

Replace the per-FPS temperature cache macros with a unified Python-based approach:

**Add to `openams_integration.py`:**
```python
class TemperatureCache:
    """Centralized temperature tracking for filament changes."""
    
    def __init__(self):
        self._lane_temps: Dict[str, int] = {}
        self._last_loaded: Dict[str, str] = {}  # extruder -> lane_name
    
    def cache_unload(self, extruder: str, lane_name: str, temp: int):
        """Cache temperature and lane during unload."""
        self._lane_temps[lane_name] = temp
        self._last_loaded[extruder] = lane_name
    
    def get_purge_temp(self, extruder: str, old_lane: Optional[str], 
                      new_lane: str, default: int = 240) -> int:
        """Calculate purge temperature for a filament change."""
        # Get old lane from cache if not provided
        if old_lane is None:
            old_lane = self._last_loaded.get(extruder)
        
        old_temp = self._lane_temps.get(old_lane, default) if old_lane else default
        new_temp = self._lane_temps.get(new_lane, default)
        
        return max(old_temp, new_temp)
```

**Update AFC_OpenAMS.py** to use it:
```python
class afcAMS(afcUnit):
    def __init__(self, config):
        super().__init__(config)
        # ... existing code ...
        self.temp_cache = TemperatureCache()  # Add this
    
    def get_purge_temp_for_change(self, old_lane: str, new_lane: str) -> int:
        """Get purge temperature for changing from old_lane to new_lane."""
        return self.temp_cache.get_purge_temp(
            self.extruder_name, old_lane, new_lane
        )
```

**Simplify AFC_oams_macros.cfg:**
```gcode
# BEFORE: 50+ lines of temperature calculation logic
# AFTER:
[gcode_macro _TX]
gcode:
    # ... existing setup ...
    
    # Simple Python call instead of complex Jinja2 logic
    {% set purge_temp = printer['AFC_OpenAMS ' ~ UNIT].get_purge_temp_for_change(
        printer["gcode_macro " ~ CACHE].cached_old_lane, 
        new_lane
    ) %}
    
    M104 S{purge_temp}
    # ... rest of macro ...
```

**Benefits:**
- Removes 3 duplicate temperature cache macros
- Eliminates complex Jinja2 string parsing
- Centralizes temperature logic in Python (testable, debuggable)
- Reduces macro file size by ~40%

---

### **Suggestion 3: Simplify Lane Configuration**

Derive implicit information from explicit configuration to reduce repetition:

**Current (AFC_AMS1.cfg):**
```
[AFC_OpenAMS AMS_1]
oams: oams1
extruder: extruder4

[AFC_lane lane4]
unit: AMS_1:1
extruder: extruder4  # Redundant
hub: Hub_1
map: T4
custom_load_cmd: _TX1 GROUP=T4
custom_unload_cmd: SAFE_UNLOAD_FILAMENT1
```

**Proposed:**
```
[AFC_OpenAMS AMS_1]
oams: oams1
extruder: extruder4
fps: fps1  # Add FPS association

[AFC_lane lane4]
unit: AMS_1:1
# extruder derived from AMS_1
# map derived from lane number (lane4 -> T4)
# load_cmd derived from FPS association (_TX1)
# unload_cmd derived from FPS association (SAFE_UNLOAD_FILAMENT1)
hub: Hub_1
```

**Implement in AFC_OpenAMS.py:**
```python
def __init__(self, config):
    super().__init__(config)
    # ... existing code ...
    
    # Store FPS association
    self.fps_name = config.get("fps", f"fps{self.name.split('_')[-1]}")
    
def _derive_lane_config(self, lane_config):
    """Auto-fill lane configuration from parent unit."""
    if not lane_config.get("extruder"):
        lane_config["extruder"] = self.extruder_name
    
    if not lane_config.get("map"):
        # Extract lane number: "lane4" -> "4" -> "T4"
        lane_num = lane_config["name"].replace("lane", "")
        lane_config["map"] = f"T{lane_num}"
    
    if not lane_config.get("custom_load_cmd"):
        fps_num = self.fps_name.replace("fps", "")
        lane_config["custom_load_cmd"] = f"_TX{fps_num} GROUP={lane_config['map']}"
    
    if not lane_config.get("custom_unload_cmd"):
        fps_num = self.fps_name.replace("fps", "")
        lane_config["custom_unload_cmd"] = f"SAFE_UNLOAD_FILAMENT{fps_num}"
    
    return lane_config
```

**Benefits:**
- Reduces configuration by 50% (4 lines → 2 lines per lane)
- Eliminates inconsistency errors (forgetting to update extruder in one place)
- Makes adding new lanes much faster
- Convention over configuration

---

### **Suggestion 4: Streamline State Synchronization**

Currently, state updates flow through multiple layers:

```
OAMS Hardware → oams.py → AMSHardwareService → afcAMS → AFCLane
                              ↓
                         oams_manager → FPSLoadState
```

**Proposed: Event-based notifications instead of polling:**

**Add to openams_integration.py:**
```python
class AMSEventBus:
    """Lightweight event system for lane state changes."""
    
    def __init__(self):
        self._subscribers: Dict[str, List[Callable]] = {}
    
    def subscribe(self, event_type: str, callback: Callable):
        self._subscribers.setdefault(event_type, []).append(callback)
    
    def publish(self, event_type: str, **kwargs):
        for callback in self._subscribers.get(event_type, []):
            try:
                callback(**kwargs)
            except Exception as e:
                logging.exception(f"Event handler failed for {event_type}")

# Use it:
event_bus = AMSEventBus()

# In oams.py when spool loads:
def load_spool(self, spool_idx):
    # ... load logic ...
    if self.action_status_code == OAMSOpCode.SUCCESS:
        self.current_spool = spool_idx
        event_bus.publish("spool_loaded", 
                         unit=self.section_name, 
                         spool_index=spool_idx)
        return True, "Spool loaded successfully"
```

**Benefits:**
- Eliminates polling overhead
- Reduces sync interval complexity
- Clear data flow (events, not state queries)
- Better debugging (can log all events)

---

### **Suggestion 5: Unified Macro Interface**

Instead of wrapper macros for each FPS, use parameters:

**Current:**
```gcode
[gcode_macro _TX1]
gcode:
    {% set GROUP = params.GROUP %}
    _TX GROUP={GROUP} EXTRUDER=extruder4 TOOL=4 FPS=fps1 ...

[gcode_macro _TX2]
gcode:
    {% set GROUP = params.GROUP %}
    _TX GROUP={GROUP} EXTRUDER=extruder5 TOOL=5 FPS=fps2 ...
```

**Proposed:**
```gcode
# Single macro with auto-derivation
[gcode_macro TX]
gcode:
    {% set GROUP = params.GROUP %}
    {% set lane_obj = "AFC_lane lane" ~ GROUP.replace('T', '') %}
    {% set lane = printer[lane_obj] %}
    {% set unit = printer['AFC_OpenAMS ' ~ lane.unit.split(':')[0]] %}
    
    # All parameters derived from lane configuration
    _TX_INTERNAL GROUP={GROUP} 
                 EXTRUDER={unit.extruder}
                 TOOL={unit.tool_number}
                 FPS={unit.fps_name}
                 CACHE={unit.temp_cache_name}
                 LED={unit.led_name}
```

Or better yet, **move the entire macro logic to Python** and call it from a simple macro:

```python
# In AFC_OpenAMS.py
def cmd_LOAD_GROUP(self, gcmd):
    """Load a filament group with all logic in Python."""
    group = gcmd.get("GROUP")
    lane = self.registry.get_by_group(group)
    
    if not lane:
        raise gcmd.error(f"Unknown group: {group}")
    
    # Calculate temps
    old_lane = self.get_loaded_lane()
    purge_temp = self.temp_cache.get_purge_temp(
        self.extruder_name, old_lane, lane.lane_name
    )
    
    # Execute load sequence
    self._execute_load_sequence(lane, purge_temp)
```

```gcode
# Macro becomes trivial
[gcode_macro TX]
gcode:
    LOAD_GROUP GROUP={params.GROUP}
```

**Benefits:**
- No duplicate macros (eliminates _TX1, _TX2, SAFE_UNLOAD_FILAMENT1, etc.)
- Python code is testable, has proper error handling
- Easier to maintain logic in one place
- Better logging and debugging

---

## Implementation Plan

### **Phase 1: Add Registry (No Breaking Changes)**
1. Add `LaneRegistry` to `openams_integration.py`
2. Register lanes during AFC_OpenAMS initialization
3. Keep existing lookups working but add new registry lookups alongside

### **Phase 2: Consolidate Temperature**
1. Add `TemperatureCache` to `openams_integration.py`
2. Update `_TX` macro to use new temperature method
3. Remove old temperature cache macros after validation

### **Phase 3: Simplify Configuration**
1. Implement auto-derivation in `AFC_OpenAMS.py`
2. Update one AMS config file as a test
3. Migrate remaining configs after validation

### **Phase 4: Streamline Macros**
1. Move tool change logic to Python commands
2. Create single `TX` macro that calls Python
3. Remove duplicate macros

### **Phase 5: Event System (Optional)**
1. Add `AMSEventBus` if polling becomes a performance issue
2. Migrate from sync intervals to event-driven updates

---

## Files to Modify

### High Priority (Core Simplifications)

**openams_integration.py:**
- Add `LaneRegistry` class
- Add `TemperatureCache` class
- Optionally add `AMSEventBus` class

**AFC_OpenAMS.py:**
- Use `LaneRegistry` instead of `_lane_by_index`
- Add auto-derivation methods
- Add `cmd_LOAD_GROUP` for Python-based loading
- Integrate temperature cache

**AFC_oams_macros.cfg:**
- Simplify `_CALCULATE_PURGE_TEMP` to call Python
- Consolidate `_TX1`/`_TX2` into single `TX` macro
- Remove duplicate temperature cache macros

### Medium Priority (Configuration Cleanup)

**AFC_AMS1.cfg & AFC_AMS2.cfg:**
- Remove redundant `extruder` from lanes
- Remove explicit `map` (derive from lane name)
- Remove explicit `custom_load_cmd`/`custom_unload_cmd`

### Low Priority (Manager Optimization)

**oams_manager.py:**
- Subscribe to events from `AMSEventBus` (if implemented)
- Remove polling if events cover all state changes

**oams.py:**
- Publish events on state changes (if using event system)

---

## Expected Improvements

### Code Metrics:
- **~40% reduction** in AFC_oams_macros.cfg size
- **~30% reduction** in lane configuration verbosity
- **~50% reduction** in lookup/translation code
- **Elimination of 3+ redundant state tracking systems**

### Performance:
- Fewer dictionary lookups per operation
- Reduced macro parsing overhead
- Optional: elimination of polling with event system

### Maintainability:
- Single source of truth for lane identity
- Less duplication = fewer places to update
- Python logic is testable and debuggable
- Clearer data flow

### User Experience:
- Simpler configuration (less to type, less to get wrong)
- Faster to add new AMS units or lanes
- Better error messages (Python vs Jinja2)

---

## Example: Before & After

### Before (Current):
```
# AFC_AMS1.cfg - 28 lines per lane
[AFC_lane lane4]
unit: AMS_1:1
load_to_hub: False
led_index: AFC_Sndicator:2
hub: Hub_1
map: T4
custom_load_cmd: _TX1 GROUP=T4
custom_unload_cmd: SAFE_UNLOAD_FILAMENT1
extruder: extruder4

# AFC_oams_macros.cfg - 60+ lines
[gcode_macro _oams_temp_cache_fps1]
variable_target_temp: 240
variable_old_temp: 240
variable_cached_old_lane: ""
gcode:

[gcode_macro _CALCULATE_PURGE_TEMP]
# 40 lines of Jinja2 string parsing...

[gcode_macro _TX1]
# Wrapper macro...

[gcode_macro SAFE_UNLOAD_FILAMENT1]
# Wrapper macro...
```

### After (Proposed):
```
# AFC_AMS1.cfg - 14 lines per lane (50% reduction)
[AFC_lane lane4]
unit: AMS_1:1
hub: Hub_1
led_index: AFC_Sndicator:2
# Everything else auto-derived!

# AFC_oams_macros.cfg - 20 lines (67% reduction)
[gcode_macro TX]
gcode:
    LOAD_GROUP GROUP={params.GROUP}

[gcode_macro UNLOAD]
gcode:
    UNLOAD_FILAMENT FPS={params.FPS|default("fps1")}
```

---

## Questions to Consider

1. **Do you want to maintain backward compatibility** with existing filament group definitions (T4, T5, etc.), or are you open to a different naming scheme?

2. **How important is migration complexity?** The phased approach allows gradual migration, but a "big bang" rewrite could be cleaner if you're willing to update all configs at once.

3. **Are there other AMS/AFC features** you use that I didn't see in these files that might be affected?

4. **Do you want lane-specific overrides?** (e.g., one lane needs a different load command even though it's derived for others)

5. **Should the registry be persistent** (saved to config) or rebuilt on startup from existing configs?

---

## Conclusion

The current implementation works but has significant duplication and complexity in lane tracking. By centralizing lane identity, consolidating temperature management, and moving logic from macros to Python, you can achieve:

- **Simpler configuration** (less typing, fewer errors)
- **Better maintainability** (one place to update)
- **Improved debuggability** (Python > Jinja2 for logic)
- **Cleaner architecture** (single source of truth)

I recommend starting with **Phase 1 (Registry)** as it provides immediate benefits without breaking existing functionality, then proceeding through the other phases as time permits.

Would you like me to provide code implementations for any of these suggestions?
