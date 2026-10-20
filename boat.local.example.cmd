@echo off
rem Where the Raspberry Pi is, for "boat <pixhawk|stm32> deploy" and "run".
rem Copy this file to boat.local.cmd (not committed) and fill it in.
rem Set up an SSH key first (ssh-copy-id, or append your public key to the Pi's
rem ~/.ssh/authorized_keys), so deploy and run do not ask for the password each time.
set "PI_HOST=pi@raspberrypi.local"
rem Directory on the Pi, relative to the home directory or absolute.
set "PI_DIR=vedc/Rasp_Pi"

rem For scripts\stm32\*.bat (test_station, test_light, run_all):
rem   the shore station's COM port and ESP-IDF's export.bat
set "ESP_PORT=COM5"
set "IDF_EXPORT=C:\Espressif\frameworks\esp-idf-v5.3\export.bat"
rem   with the Windows installer: IDF_EXPORT=<Espressif>\idf_cmd_init.bat and its esp-idf id here
set "IDF_EXPORT_ARGS="
rem   the Worker serving the dashboard (wrangler dev locally, or the deployed one)
set "CLOUD_URL=http://127.0.0.1:8787"
rem   the Worker's ADMIN_API_TOKEN secret, for test_light.bat (leave empty if unused)
set "ADMIN_API_TOKEN="
rem   the Pi's sound output, as audio.device in Rasp_Pi/config/settings.yaml
set "AUDIO_DEVICE=plughw:CARD=Headphones,DEV=0"
