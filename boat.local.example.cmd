@echo off
rem Where the Raspberry Pi is, for "boat <pixhawk|stm32> deploy" and "run".
rem Copy this file to boat.local.cmd (not committed) and fill it in.
rem Set up an SSH key first (ssh-copy-id, or append your public key to the Pi's
rem ~/.ssh/authorized_keys), so deploy and run do not ask for the password each time.
set "PI_HOST=pi@raspberrypi.local"
rem Directory on the Pi, relative to the home directory or absolute.
set "PI_DIR=vedc/Rasp_Pi"
