#!/bin/bash

# g++ compiler

g++ main.cpp euler2quarternions.cpp euler2quarternions.h satellite.cpp satellite.h tibquat.cpp tibquat.h sensor.cpp sensor.h navigation.cpp navigation.h common.h control.cpp control.h -o out.out

./out.out > stateout.txt

#rm common.h.gch control.h.gch euler2quarternions.h.gch navigation.h.gch satellite.h.gch sensor.h.gch tibquat.h.gch
