# 2022-spaceshot-flight-systems
This project is a combination of several different systems:
1. A container flight computer, based around a Teensy 4.1
2. A payload flight computer, based on the same hardware
3. A groundstation software suite, designed to be hardware agnostic

The flight computers are programmed in Arduino C++, and the groundstation in Python. They communictate via serial, bridged over air by several xBee radio modules. Communication mainly consists of regular telemetry updates, as well as several custom commands, both for simulation and testing purposes, as well as general flightOps and contingencies.

More information about the competition specifications may be found [here](https://www.cansatcompetition.com/docs/CanSat_Mission_Guide_2022.pdf).
