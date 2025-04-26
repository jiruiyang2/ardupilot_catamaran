import os
import re

import os
import re

TARGET_DIR = "modules/mavlink/include/mavlink/v2.0/ardupilotmega"

def patch_packet_arrow_to_dot(filepath):
    with open(filepath, 'r') as f:
        lines = f.readlines()

    patched_lines = []
    packet_declared_as_struct = False

    for line in lines:
        if re.search(r'\b(mavlink_\w+_t)\s+packet;', line):
            packet_declared_as_struct = True

        # If packet is a struct, fix invalid '->'
        if packet_declared_as_struct and 'packet->' in line:
            line = line.replace('packet->', 'packet.')

        patched_lines.append(line)

    with open(filepath, 'w') as f:
        f.writelines(patched_lines)
    print(f"✅ Patched: {filepath}")

def main():
    for root, _, files in os.walk(TARGET_DIR):
        for file in files:
            if file.startswith("mavlink_msg_") and file.endswith(".h"):
                patch_packet_arrow_to_dot(os.path.join(root, file))

if __name__ == "__main__":
    main()

