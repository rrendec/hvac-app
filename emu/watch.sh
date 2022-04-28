#!/bin/bash

cd -- "$(dirname -- "${BASH_SOURCE[0]}")"

if [[ ! -f gpio.txt ]]; then
	exit 1
fi

watch --interval 0.5 --precise --no-title --color ./gpio.sh
