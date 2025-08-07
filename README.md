This is an example with 2 ams units that are hooked up to indivdual tool heads. This is for use with AFC_klipper_addon

Put the pins.py file in klipper/klippy replacing the original file *this will result in a "dirty" klipper install*

Put auto_ams_update.py and virtual_input_pin.py in klipper/klippy/extras

AFC_AMS1.cfg and AFC_AMS2.cfg are example AFC configs that go in printer_data/config/AFC.

Add [virtual_input_pin enable] in your cfg, or create at least one virtual pin to enable the module


## AMS virtual pins

