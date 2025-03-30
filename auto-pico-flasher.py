import os
import shutil
import time
import importlib
import subprocess
import sys

"""
Checks if the pywin32 module and tkinter library are installed and installs them if necessary.
"""
launch_again = False

try:
    importlib.import_module('win32api')
    importlib.import_module('win32com.client')
except (ImportError, ModuleNotFoundError):
    subprocess.check_call([sys.executable, '-m', 'pip', 'install', 'pywin32'])
    launch_again = True

try:
    importlib.import_module('tkinter')
except (ImportError, ModuleNotFoundError):
    subprocess.check_call([sys.executable, '-m', 'pip', 'install', 'tkinter'])
    launch_again = True


if launch_again:
    subprocess.check_call([sys.executable, 'auto-pico-flasher.py'])
    quit()
    
import win32api
import win32com.client
from tkinter import Tk, filedialog


def select_uf2_file():
    print("Locate the uf2 file to be flashed onto pico...")
    root = Tk()
    root.withdraw()
    file_path = filedialog.askopenfilename(title="Select UF2 File", filetypes=[("UF2 Files", "*.uf2")])
    return file_path


def detect_pico_drive():
    """
    Detects when the Pico drive is connected.
    """
    volume_name = "RPI-RP2"
    while True:
        try:
            drives = win32api.GetLogicalDriveStrings()
            drives = drives.split('\000')[:-1]
            for drive in drives:
                drive = drive.strip()
                volume_info = win32api.GetVolumeInformation(drive)
                if volume_info[0] == volume_name:
                    print(f"Detected Pico drive at: {drive}")
                    return drive
        except Exception as e:
            print(f"Error occurred: {e}")
            pass
        time.sleep(1)


def copy_uf2_to_pico(source, destination):
    """
    Copies the UF2 file to the Pico drive.
    """
    try:
        shutil.copy(source, destination)
        print(f"UF2 file copied successfully at {time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime())}")
    except Exception as e:
        print(f"Failed to copy UF2 file: {str(e)}")


def flash_pico():
    """
    Flashes the Pico with the UF2 file.
    """
    # Code for flashing the Pico goes here
    print("Flashing Pico...")
    time.sleep(1)  # Simulating flashing process

def select_destination():
    root = Tk()
    root.withdraw()
    print("Select optional location to copy .uf2 file to.")
    dir_path = filedialog.askdirectory(title="Select Destination Folder")
    return dir_path

def copy_uf2_to_destination(source, destination):
    """
    Copies the UF2 file to the destination folder.
    """
    try:
        shutil.copy(source, destination)
        print("UF2 file copied to destination successfully.")
    except Exception as e:
        print(f"Failed to copy UF2 file to destination: {str(e)}")


def main():

    uf2_file = select_uf2_file()
    if uf2_file:
        destination = select_destination()
        while True:
            # Wait for Pico drive to be connected
            print("Waiting for Pico drive to be connected...")
            pico_path = detect_pico_drive()

            # Copy UF2 file to Pico drive
            copy_uf2_to_pico(uf2_file, pico_path)

            # If destination copy exists, copy to there as well
            if destination:
                copy_uf2_to_destination(uf2_file,destination)

            # Flash the Pico
            flash_pico()

            # Wait for a few seconds before looping again
            time.sleep(3)
    else:
        print("No UF2 file selected.")


if __name__ == "__main__":
    main()
