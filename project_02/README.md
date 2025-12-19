# Project 02 - Etch-a-Sketch PCB

## Overview
This project contains the PCB design files for an Etch-a-Sketch device using a PocketBeagle microcontroller. The board features a USB connection, potentiometers for X/Y control, push buttons, and LEDs.

## Project Structure

### `/EAGLE`
Contains the EAGLE PCB design files:
- `etchasketch.sch` - Schematic design
- `etchasketch.brd` - Board layout
- `etchasketch.lbr` - Custom component library

### `/MFG`
Manufacturing files ready for PCB fabrication:

#### `/CAMOutputs/GerberFiles`
- Gerber files for PCB layers (top/bottom copper, soldermask, silkscreen, paste, profile)
- `gerber_job.gbrjob` - Job configuration file

#### `/CAMOutputs/DrillFiles`
- Drill files for PCB holes

#### `/CAMOutputs/Assembly`
- `etchasketch.txt` - Assembly drawing
- `PnP_etchasketch_front.txt` - Front pick-and-place file
- `PnP_etchasketch_back.txt` - Back pick-and-place file

### `/docs`
- `BOM` - Bill of Materials with component specifications

## Bill of Materials

Key components include:
- 1x PocketBeagle microcontroller
- 2x Potentiometers (for X/Y control)
- 2x Push buttons
- 3x LEDs
- 1x USB Type A receptacle connector
- Various resistors
- 4x Mounting holes
- 6x Fiducials

## Getting Started

### Design Files
Open the EAGLE schematic and board files in Autodesk EAGLE to view or modify the design:
1. Open `EAGLE/etchasketch.sch` for schematic
2. Open `EAGLE/etchasketch.brd` for board layout

### Manufacturing
The `/MFG/CAMOutputs` directory contains all files needed to manufacture the PCB:
- Send Gerber files to your PCB manufacturer
- Use the BOM to source components
- Use pick-and-place files for assembly
