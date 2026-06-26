import subprocess
import argparse
import time
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--target", default="final.elf", help="Output ELF file.")
args=parser.parse_args()

arg_path=Path(args.target)
if arg_path.exists():
    print(f"Executable binary passed: {arg_path.name}")
else:
    print(f"{arg_path.name} doesn't exists: Terminating Flash.....")
    print(f"Failed to launch GDB.....")
    exit(1)

openocd_ret=subprocess.Popen(["openocd","-f","board/stm32f4discovery.cfg"])

time.sleep(5)

gdb_flash = """
target remote localhost:3333
monitor reset halt
load
monitor reset run
monitor shutdown 
quit
"""

with open("flash_run.gdb","w") as f:
    f.write(gdb_flash)

subprocess.run(["gdb-multiarch",
    str(arg_path),
    "-x",
    "flash_run.gdb"])

