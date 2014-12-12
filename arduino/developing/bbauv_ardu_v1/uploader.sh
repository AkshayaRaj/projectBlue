#!/bin/sh

echo "OK, starting now..."
sftp bbauvsbc1@bbauv <<EOF
#Replace the name with your own directory
put /home/gohew/sketchbook/bbauv_ardu_v1/Release/bbauv_ardu_v1.hex mega.hex
bye
EOF
ssh bbauvsbc1@bbauv "avrdude -p atmega2560 -b 115200 -c stk500v2 -P/dev/ttyArduino -U flash:w:mega.hex"

