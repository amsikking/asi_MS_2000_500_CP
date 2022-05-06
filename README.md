# asi_MS_2000_500_CP
Python device adaptor: ASI MS-2000-500-CP multi-axis stage controller.
## Quick start:
- Install the correct Silicon Labs VCP USB driver: https://www.asiimaging.com/support/downloads/usb-support-on-ms-2000-wk-controllers/ (or here: https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers).
- Check what devices are attached to the controller and then run the .py script with the correct configuration (see test block). ***WARNING: the test block will move the XY stage and Z stage! (if attached) -> remove samples and objective before testing!***
## Details:
- The MS-2000 (or MS-2000-WK) controllers from ASI come in many variants, and with the possibility of connecting many different devices (stages, light sources, autofocus etc). See the online manual for a guide to what's available: https://asiimaging.com/docs/products/ms2000 (a static .pdf is included here).
- This device/adaptor was tested with a MS-2000-500-CP model (hence the repository name) with firmware version 'USB-9.2k', along with a 'MS-2000 flat top XY automated stage' (standard lead screw), an 'LS-50 linear stage' for Z (fine lead screw) and an ASI pwm transmitted light LED (MIM-LED-LAMP?). With this hardware the python adaptor has run without fail for > 10,000 iterations of the 'random input testing' block in the .py file.
- It is expected that this adaptor can be adapted (with minimal effort) to many other configurations of the MS-2000 controller + attached hardware.
