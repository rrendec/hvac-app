#!/bin/bash

echo -e "\e[0;93m$(date)\e[0m\n"

REPLACE_OFF='s/^\(\s*#.*\n.*1\)$/\o033\[91m\0\o033\[0m/'
REPLACE_ON='s/^\(\s*#.*\n.*0\)$/\o033\[92m\0\o033\[0m/'
sed "\$!N;$REPLACE_OFF;$REPLACE_ON;P;D" gpio.txt
