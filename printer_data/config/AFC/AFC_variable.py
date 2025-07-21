[virtual_input my_filament_pin]
initial_value: 1                     # 1 = filament detected at startup

[virtual_filament_sensor my_sensor]
pin: my_filament_pin                 # name of the virtual_input section
pause_on_runout: True                # other filament sensor options work here
# runout_gcode: ...
# insert_gcode: ...
