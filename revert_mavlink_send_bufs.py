import os

TARGET_DIR = "modules/mavlink/include/mavlink/v2.0/ardupilotmega"

def revert_arrow_to_dot(filepath):
    with open(filepath, 'r') as file:
        content = file.read()

    # Revert . back to -> in patch lines
    reverted = content.replace("packet.", "packet->")

    with open(filepath, 'w') as file:
        file.write(reverted)
    print(f"✅ Reverted {filepath}")

def main():
    for root, _, files in os.walk(TARGET_DIR):
        for fname in files:
            if fname.startswith("mavlink_msg_") and fname.endswith(".h"):
                revert_arrow_to_dot(os.path.join(root, fname))

if __name__ == "__main__":
    main()
