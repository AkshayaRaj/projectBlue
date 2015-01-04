#!/bin/sh
echo "This is " $(hostname)
if [ $(hostname) = asus ]; then
	echo "copying asus rules to package.."
	rm ./rules/72-auv*
	cp /etc/udev/rules.d/72-auv-dev.rules ./rules
else
	echo "updating current udev rules to sbc.."
	rm /etc/udev/rules.d/72-auv*
	cp ./rules/72-auv* /etc/udev/rules.d/

fi
echo "done !"
