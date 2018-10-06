# uBMS 

Electric Vehicle Battery Management System based on the LTC6913 battery monitor IC and ST32L microprocessor.
The aim is to create a BMS that is small enough to suit ebike battery packs, yet powerful enough to be used 
on full-blown EVs (motorcycles or cars.)

Features:
* 12-18 cell operation, with the ability to split off the HV cell monitor (Batter Monitor Board) and expand to hundreds of cells.
* Galvanic isolation between cells and processor.
* 3mV accurate brick voltage sense.
* ?mA accurate pack current sense with shunt.
* 100mA cell balancing current
* 2x thermistor inputs per BMB
* powerful IO: CAN bus, UART, 3x isolated outputs, and simple dash interface (3x leds and switches)
* Built in DC/DC for 8-56V LV supply, directly interface to cells (12s-14s) or to a vehcle LV bus.
* SD card for logging battery performance

This repository contains code, calculations and documentation for the project.
The project was created with Atollic TrueSTUDIO for STM32, initialization code was generated with STM32cubeMX.
PCB/schematic in circuitmaker: https://workspace.circuitmaker.com/Projects/Details/Jarrod-Tuma/EVBMS

Project log: https://hackaday.io/project/159341-bms