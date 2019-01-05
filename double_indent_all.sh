# Converts stupid WPILib 2 space indents to proper 4 space indents
# IT IS NOT SMART!!! Every single time this script gets run, the size of the indents double
# This script runs on all source files.
# DO NOT RUN THIS SCRIPT A SECOND TIME.

# Make sure this script is run in the src directory.
# List all files, pipe to grep to filter out only java files, and then pipe to sed to replace 2 spaces with 4 spaces
find . | grep -e "\.java$" | xargs sed -i 's/^\(\(  \)\+\)/\1\1/g'
