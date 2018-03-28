# AutomationShield Library and Examples

## About AutomationShield

Arduino library for the AutomationShield Arduino expansion boards for control engineering education. You can read more about the AutomationShield project in our [Wiki page](https://github.com/gergelytakacs/AutomationShield/wiki).

## Installing the library


## Library usage

The harware library for a particular board is initialized by the board name followed by `begin` for example to initialize the OptoShield call:
```
Opto.begin();
```

# Common functions

# mapFloat()
To linearly map a floating point number `value` from a given input range to a given output range, you should use  
```
AutomationShield.mapFloat(value, fromLow, fromHigh, toLow, toHigh);
```
where `fromLow` is the start of the input range and `fromHigh` the end, while `toLow` is the start of the output range and `toHigh` is its end.

# OptoShield

The board is initialized by calling
```
Opto.begin();
```
this will initialize the pins necessary for the feedback experiment (LDR, LED) and the potentiometer for user input.

# MotoShield
