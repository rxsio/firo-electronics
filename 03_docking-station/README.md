# Docking Station
## Overview

This project focuses on the electronics system for the FIRO rover's docking station. The docking station is a flat module similar to a floor cable cover, featuring two exposed copper plates. When the rover docks, it deploys two copper probes to establish an electrical connection with these plates. The rover can dock from either side of the station. 

This design addresses uncertainties regarding the docking station's placement inside the FLASH accelerator. It can be positioned anywhere without creating obstructions or safety hazards. However, due to its easy accessibility and exposed copper plates, the docking station must only be powered when used by the rover, to prevent accidental activation by humans. It should also be protected against short circuits caused by metal tools left on the station.

To maintain a flat profile and avoid obstructing the corridor, the electronics system is housed in a separate module that can be placed conveniently on the side of the corridor.

<!--## Usage
Provide instructions for user how to use or interact with the PCB 

### Pinout
Include a pinout schematic. This can help users understand how to interface with the PCB.

### Troubleshooting
Describe common issues that may arise during the PCB usage. Include guidance on identifying and resolving these issues. -->

## Design
### Requirements
- The docking station must be completely powered off when not in use by the rover.
  - It should power off automatically after the rover stops charging.
  - Activation cannot be triggered accidentally by humans:
    - The rover must initially power the station to start charging.
- The docking station should support charging from both sides.
  - It should adjust the polarization of the copper pads based on the rover's initialization pulse.
- The docking station must cut off power in case of a short circuit between the copper pads.


<!-- ### Components
This section should highlight the most important components and their functions. 

### Schematic
Include an exported image of the PCB schematic.  
Optionally describe any relevant details about the schematic design

### Layout
Include exported images of the PCB layouts  
- Combined layout with dimensions. Include all important dimensions: PCB width and height, mounting holes size and position, etc.  
- Combined layout without dimensions  
- Separate layouts for each layer  
Optionally describe any relevant details about the layout

### Reference Documentation
Include links to relevant reference documents such as datasheets for the components. When possible, host the documents in the project repository. It will guarantee they are easily accessible in the future, even If the original links would be no longer valid.

## Firmware
### Overview
Briefly describe function of the firmware. You may also describe firmware architecture, key components and dependencies.

### Usage
Describe how to use the firmware. Include information about:

- user interfaces or commands available  
- settings, or parameters that can be customized

### Development  
- Installing dependencies required for the development  
- Setting up development environment  
- Uploading firmware to the target device

## Manufacturing
### Manufacturing Considerations
Include any manufacturing considerations for the PCB design, such as panelization, solder mask requirements, or impedance control.

### Assembly Instructions
Include any specific guidelines or precautions that need to be followed during the assembly process.

### Testing
Describe the testing procedures and methodologies used to verify the functionality of the PCB. 

## Conclusions
Summarize the key findings of the PCB project. Provide recommendations for future improvements. -->