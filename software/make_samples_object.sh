#!/bin/bash -e
if [[ $# -ne 1 ]]; then
    echo 'Usage: <script> [CSV Ping Recording]'
    exit -1
fi

python csv_to_blob.py $1
arm-none-eabi-objcopy -I binary -B arm samples_blob -O elf32-littlearm build/emulator/samples_blob.o
