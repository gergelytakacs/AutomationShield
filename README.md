# AutomationShield Library and Examples

## About AutomationShield

Arduino library for the AutomationShield Arduino expansion boards for control engineering education. You can read more about the AutomationShield project in our [Wiki page](https://github.com/gergelytakacs/AutomationShield/wiki).

## Installing the library

## Library usage

The harware library for a particular board is initialized by the board name followed by `begin` for example to initialize the OpticalShield call:
```
Optical.begin();
```

# OptoShield

The board is initialized by calling
```
Optical.begin();
```
this will initialize the pins necessary for the feedback experiment (LDR, LED) and the potentiometer for user input.

# MotoShield
