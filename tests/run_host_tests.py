"""Run production control/transport C with simulated CAN hardware and clock."""
from pathlib import Path
import os
import re
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'build' / 'host_tests'
OUT.mkdir(parents=True, exist_ok=True)
CC = shutil.which('gcc')
if not CC:
    raise SystemExit('Host gcc unavailable; tests NOT run')
os.environ['PATH'] = str(Path(CC).parent) + os.pathsep + os.environ.get('PATH', '')
INCLUDES = ['tests/stubs', 'application', 'application/chassis', 'modules/algorithm',
            'modules/motor/DJImotor', 'modules/motor/LKmotor', 'modules/motor', 'modules/daemon', 'modules/can_comm',
            'modules/message_center', 'modules/super_cap', 'modules/master_machine', 'modules']
COMMON = [CC, '-std=gnu11', '-O1', '-g', '-Wall', '-Wextra', '-Wno-unused-parameter',
          '-Werror=implicit-function-declaration', '-ffunction-sections', '-fdata-sections']
COMMON += ['-I'+str(ROOT/p) for p in INCLUDES]
SOURCES = ['tests/chassis_test.c', 'modules/motor/DJImotor/dji_motor.c', 'modules/motor/LKmotor/LK9025.c',
           'modules/can_comm/can_comm.c', 'modules/daemon/daemon.c',
           'modules/algorithm/crc8.c', 'modules/algorithm/controller.c', 'modules/algorithm/user_lib.c']

header = (ROOT/'application/robot_def.h').read_text(encoding='utf-8')
for drive in ['SWERVE', 'MECANUM', 'OMNI_X']:
    variant = re.sub(r'^#define CHASSIS_DRIVE_SWERVE\b.*$', '#define CHASSIS_DRIVE_'+drive, header, flags=re.M)
    folder = OUT/drive
    folder.mkdir(exist_ok=True)
    (folder/'robot_def.h').write_text(variant, encoding='utf-8')
    exe = folder/'chassis_test.exe'
    subprocess.run(COMMON[:1] + ['-I'+str(folder)] + COMMON[1:] + [str(ROOT/p) for p in SOURCES]
                   + ['-lm', '-Wl,--gc-sections', '-o', str(exe)], cwd=ROOT, check=True)
    print(drive+': '+subprocess.check_output([str(exe)], text=True), end='')

exe = OUT/'protocol.exe'
subprocess.run(COMMON + ['-DROBOT_DEF_SOURCE="'+(ROOT/'application/robot_def.h').as_posix()+'"',
               str(ROOT/'tests/protocol_test.c'), '-o', str(exe)], cwd=ROOT, check=True)
layout = subprocess.check_output([str(exe)], text=True)
assert layout == '24 65 49 53 57 61 4\n', layout
print('PASS shared 4-byte App_Fault_s in chassis/gimbal/shoot; command 24 bytes; feedback 65 bytes including IMU; fault/vx/vy/wz offsets 49/53/57/61')
