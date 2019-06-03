# AutomationShield Library [![Build Status](https://travis-ci.org/gergelytakacs/AutomationShield.svg?branch=master)](https://travis-ci.org/gergelytakacs/AutomationShield) [![CodeFactor](https://www.codefactor.io/repository/github/gergelytakacs/automationshield/badge)](https://www.codefactor.io/repository/github/gergelytakacs/automationshield)



## About AutomationShield

Arduino, MATLAB and Simulink IDE for the AutomationShield expansion boards for control engineering education. You can read more about the AutomationShield project in our [Wiki page](https://github.com/gergelytakacs/AutomationShield/wiki).

## Downloading and Installing the AutomationShield Library

### I don't know Git
If you are not familiar with Git, please download the latest release of the library from the [Releases](https://github.com/gergelytakacs/AutomationShield/releases) section, as the production code download does not include certain dependencies. Search for the `AutomationShield_vX.Y.tar` file in the Assets, where `vX.Y` is the major and minor version number of the release. Do not use the Source Code files in the Assets, since these lack the dependent code as well.

### I use Git
For those who wish to use Git, this repository contains submodules, therefore you should use `git clone --recursive git://github.com/gergelytakacs/AutomationShield.git` to get these as well. In case you have already cloned the repository, the submodule directories in `src/lib/` may be empty. In this case, you have to initialize it by calling `submodule update --init --recursive`.

## Interfaces

### Arduino IDE

Open the Arduino IDE, click on the *Sketch* menu, and find the *Include Library* option. This opens another sub-menu from which you shold select *Add .ZIP Library...* that will open the file browser. Locate the renamed library .zip file and click *Open*.

### MATLAB

Launch the `installMatlabAndSimulink.m` file from the root directory or the `installForMATLAB.m` from the `matlab` directory from the MATLAB command line. This adds and saves the correct paths to your MATLAB installation. The MATLAB API for ArduinoShield requires the installation of the  [MATLAB Support Package for Arduino Hardware](https://www.mathworks.com/hardware-support/arduino-matlab.html).


### Simulink
Launch the `installMatlabAndSimulink.m` file from the root directory or the `installForSimulink.m` from the `simulink` directory from the MATLAB command line. This adds and saves the correct paths to your MATLAB installation. The MATLAB API for ArduinoShield requires the installation of the  [Simulink Support Package for Arduino Hardware](https://www.mathworks.com/hardware-support/arduino-simulink.html). The algorithmic blocks for the boards should appear in your library.


## Library usage

This is a minimal summary for library usage. The harware library for a particular board is initialized by the board name followed by `begin` for example to initialize the OptoShield call:
```
OptoShield.begin();
```
The onboard sensor is read by calling the `sensorRead()` method, in case of the aformentioned OptoShield this would be 
```
y=OptoShield.sensorRead();
```
returning the reading to the variable `y`. Finally, the actuator is set by the `actuatorWrite()` method, that is 
```
OptoShield.actuatorWrite(u);
```
sets the actuator to `u`. Inputs and outputs are floating point numbers.

Please consult our [Wiki page](https://github.com/gergelytakacs/AutomationShield/wiki) for more details and board specific instructions.

## Examples

### Arduino IDE

The library contains examples in the `\examples\` directory that will also show up in the Examples collection of the Arduino IDE. You can access this by clicking *File*, *Examples* and look for the *AutomationShield* library. Examples showcase the use of the AutomationShield library in Arduino sketches.

### MATLAB IDE

Worked examples for the AutomationShield MATLAB IDE are located in the `\matlab\examples\` directory.

### Simulink IDE

Worked examples for the AutomationShield Simulink IDE are located in the `\simulink\examples\` directory.

