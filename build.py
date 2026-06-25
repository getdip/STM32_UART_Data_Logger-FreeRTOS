import argparse
import subprocess
from pathlib import Path

parser = argparse.ArgumentParser()

parser.add_argument("--target", default="final.elf", help="Expected output binary name")

args=parser.parse_args()

target_path = Path(args.target)

print("Cleaning project...")
subprocess.run(["make","clean"])

print(f"Building target: {target_path.name}...")
buildRet=subprocess.run(["make"],capture_output=True,text=True)

print(buildRet.stdout)

if buildRet.returncode !=0:
    print("Build Failed")
    print(buildRet.stderr)
    
else:
    if target_path.exists():
        print(f"Buid Successfull........{target_path.name} created")
    else:
        print(f"Build Failed: Make was successful but {target_path.name} is missing")



