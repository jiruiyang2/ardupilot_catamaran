#!/bin/bash

echo "🔍 Scanning for MAVLink integration issues..."

# Step 1: Check for files using `mavlink_system` but missing proper includes
echo -e "\n🔎 Checking for 'mavlink_system' usage without '#include <mavlink.h>'..."
grep -r --include=*.cpp --include=*.h "mavlink_system" . | while read -r line; do
    file=$(echo "$line" | cut -d: -f1)
    if ! grep -q "mavlink.h" "$file"; then
        echo "⚠️  '$file' uses mavlink_system but is missing mavlink.h include"
    fi
done

# Step 2: Identify files that may define `mavlink_system`
echo -e "\n📦 Checking for definitions of 'mavlink_system'..."
grep -r --include=*.cpp "mavlink_system_t mavlink_system" . || echo "✅ No unexpected definitions found"

# Step 3: Find files that use `mavlink_message_t` but don’t include mavlink headers
echo -e "\n🔎 Checking for use of 'mavlink_message_t' without proper includes..."
grep -r --include=*.cpp --include=*.h "mavlink_message_t" . | while read -r line; do
    file=$(echo "$line" | cut -d: -f1)
    if ! grep -q "mavlink.h" "$file"; then
        echo "⚠️  '$file' uses mavlink_message_t but is missing mavlink.h include"
    fi
done

# Step 4: Show all mavlink includes
echo -e "\n📋 All files including MAVLink:"
grep -r --include=*.cpp --include=*.h "mavlink.h" . | cut -d: -f1 | sort | uniq

# Step 5: Summarize results
echo -e "\n✅ Diagnostic scan complete. Check above for warnings (⚠️)."

