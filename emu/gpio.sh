#!/bin/bash

echo -e "\e[0;93m$(date)\e[0m\n"

REPLACE_OFF='s/^(([0-9]+\s+){2}1.*)$/\o033\[91m\0\o033\[0m/'
REPLACE_ON='s/^(([0-9]+\s+){2}0.*)$/\o033\[92m\0\o033\[0m/'
sed -r "$REPLACE_OFF;$REPLACE_ON" gpio.txt
