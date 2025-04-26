#!/bin/bash

echo "🔍 Scanning for MAVLink usage in project..."

# Step 1: Look for includes
echo -e "\n📦 MAVLink includes:"
grep -r --include=\*.cpp --include=\*.h "mavlink.h" . || echo "  ❌ No includes found."

# Step 2: Look for mavlink_system usage
echo -e "\n🧩 Files referencing 'mavlink_system':"
grep -r --include=\*.cpp --include=\*.h "mavlink_system" . || echo "  ❌ No references found."

# Step 3: Look for definitions of mavlink_system (should only be in one .cpp)
echo -e "\n🚨 Potential definitions of 'mavlink_system':"
grep -r --include=\*.cpp "mavlink_system_t mavlink_system" . || echo "  ✅ No duplicate definitions found."

# Step 4: Look for other common MAVLink types
echo -e "\n📌 Files using common MAVLink types (e.g. mavlink_message_t, mavlink_finalize_message):"
grep -r --include=\*.cpp --include=\*.h -E "mavlink_message_t|_mav_finalize|mavlink_finalize_message" . || echo "  ❌ No types found."

echo -e "\n✅ Scan complete."
