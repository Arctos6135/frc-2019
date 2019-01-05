# Converts stupid WPILib 2 space indents to proper 4 space indents
# IT IS NOT SMART!!! Every single time this script gets run, the size of the indents double
# This script only runs on one file.

sed -i 's/^\(\(  \)\+\)/\1\1/g' $1
