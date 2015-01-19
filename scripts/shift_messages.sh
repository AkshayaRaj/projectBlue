#!/bin/bash

#echo "This is $(hostname)"
#if[$(hostname) = auv ]; then
	source ~/srmauv/projectBlue/devel/setup.bash

	source ~/srmauv/devel/setup.bash


roscd srmauv_msgs
cd srv
cp -f *.srv bkp
rm *.srv
cd ../msg
cp -f *.msg bkp
cd ../action
cp -f *.action bkp
rm *.action

roscd srmauv_msgs
cd ../../scripts
source generate_msgs.sh 

roscd srmauv_msgs 
cd srv
cp -f bkp/*.srv .
cd ../action
cp -f bkp/*.action .





