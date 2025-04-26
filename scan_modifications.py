import os
import filecmp
import shutil
import subprocess

YOUR_PROJECT_PATH = "/home/jiruiyang/ardupilot"
TEMP_CLEAN_PATH = "/tmp/ardupilot_clean_copy"
ARDUPILOT_REPO_URL = "https://github.com/ArduPilot/ardupilot.git"


def clone_original_source():
    if os.path.exists(TEMP_CLEAN_PATH):
        print("Original source already cloned.")
    else:
        print("Cloning official ArduPilot source code...")
        subprocess.run(["git", "clone", "--depth", "1", ARDUPILOT_REPO_URL, TEMP_CLEAN_PATH], check=True)


def compare_directories(dir1, dir2):
    comparison = filecmp.dircmp(dir1, dir2)
    
    # New files
    if comparison.left_only:
        print("\nNew files (only in your project):")
        for item in comparison.left_only:
            print(f"  [NEW] {os.path.join(dir1, item)}")

    # Modified files
    if comparison.diff_files:
        print("\nModified files:")
        for item in comparison.diff_files:
            print(f"  [MODIFIED] {os.path.join(dir1, item)}")

    # Recursively check subdirectories
    for common_dir in comparison.common_dirs:
        compare_directories(os.path.join(dir1, common_dir), os.path.join(dir2, common_dir))


def main():
    print("===== SCANNING YOUR MODIFICATIONS =====\n")
    clone_original_source()

    print("\nComparing project folders...")
    compare_directories(YOUR_PROJECT_PATH, TEMP_CLEAN_PATH)

    print("\n===== SCAN COMPLETE =====")


if __name__ == "__main__":
    main()
