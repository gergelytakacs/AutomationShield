CC = gcc
FLAGS = -Os -Wall -Wstrict-prototypes
OPT = -O3 -funroll-loops
STD = -std=c89

all: lcmpc motor aircraft

lcmpc:
	make -C cmpc

motor: main_motor.c lcmpc
	$(CC) $(FLAGS) $(OPT) $(STD) -static main_motor.c -Lcmpc -lcmpc -o motor

aircraft: main_aircraft.c lcmpc
	$(CC) $(FLAGS) $(OPT) $(STD) -static main_aircraft.c -Lcmpc -lcmpc -o aircraft

clean:
	rm -rf cmpc python __pycache__ motor aircraft
