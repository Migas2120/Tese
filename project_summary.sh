#!/bin/bash

echo "Project Summary for: $(basename $(pwd))"
echo "--------------------------------------"
echo ""

# Count files by type
echo "File type counts:"
find . -type f | sed -n 's/.*\.\([a-zA-Z0-9]*\)$/\1/p' | sort | uniq -c | sort -nr
echo ""

# Show directory structure
echo "Directory tree (depth 2):"
tree -L 3
echo ""

# List Python and C# files with top-level function/class definitions
echo "Scanning for function and class definitions..."
echo ""

for file in $(find . -name "*.py" -o -name "*.cs"); do
  echo "File: $file"
  grep -E '^\s*(def |class |public class|private class|public void|private void|public .*|private .*)' "$file" | sed 's/^/  /'
  echo ""
done

# Optional: list ROS 2 packages (if in a ROS 2 workspace)
if [ -f ./src/COLCON_IGNORE ] || [ -d src ]; then
  echo "ROS 2 packages detected (colcon workspace):"
  find src -name package.xml | xargs -n1 dirname | sed 's/^/  /'
  echo ""
fi
