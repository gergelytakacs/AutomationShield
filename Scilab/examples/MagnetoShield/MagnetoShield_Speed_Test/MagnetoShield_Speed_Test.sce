/*  MAGNETOSHIELD SCILAB MINIMAL COMMUNICATION SPEED TEST

   This example tests for the absolute minimal sampling period that is 
   achievable with this shield using Scilab. At the time of writing this code
   the Scilab Arduino interface does not support I2C communication that is
   necessary to operate the MagnetoShield. A provisional I2C part has been
   written to the original server script, this can be found in the
   MagnetoShield_Speed_Test directory. First, you must upload this code to your
   device. Scilab will also require the "serial" package. If you don't have this
   installed, call atomsInstall("serial"), then load the macros.
   The script just tests reading the analog input and sending data to the I2C 
   bus without attempting feedback control, as these are the most essential 
   operations required for control. The functionality of the I2C part has not
   been tested, the script assumes that using the same serial communication
   logic, it would take to write 4 characters to the serial port at each 
   iteration (address, command and 2x data). This script has not been tested
   using xCos, as rhoughly the same speeds are expected and xCos does not 
   contain the I2C block either.

   This code is part of the AutomationShield hardware and software
   ecosystem. Visit http://www.automationshield.com for more
   details. This code is licensed under a Creative Commons
   Attribution-NonCommercial 4.0 International License.

   If you have found any use of this code, please cite our work in your
   academic publications, such as thesis, conference articles or journal
   papers. A list of publications connected to the AutomationShield
   project is available at: 
   https://github.com/gergelytakacs/AutomationShield/wiki/Publications

   Created by:       Gergely Takács
   Created on:       03.11.2020
   Last updated by:  Gergely Takács
   Last update on:   03.11.2020
   Tested on:        Arduino Uno, Scilab 6.0.2, Modified server script v5 for 
                     the v 1.8 of the "arduino" ATOMS package, v 0.5 of the 
                     "serial" ATOMS package
*/


clc                                               // Clears screen

no_samples = 500;                                 // Desired number of samples
address = 60;
command = 64;
data1 = 0;
data2 = 0;
closeserial(A);
A = openserial(5,"115200,n,8,1");                 // Opens serial, must have script installed, see above

disp("Testing...")                                // Message.

tet = zeros(no_samples,1);                        // Pre-allocate memory for execution time measurements
for i = 1:no_samples                              // Performs several measurements
    tic();                                        // Starts stopwatch
    err = writeserial(A, "A3");                   // Sends command to read analog pin
    y = readserial(A);                            // Saves result to variable y, so far a string representing the binary equivalent
    err = writeserial("I");                       // Sends command to activate provisional I2C functionality
    err = writeserial(ascii(address));            // Sends address
    err = writeserial(ascii(command));            // Sends command
    err = writeserial(ascii(data1));              // Sends data1
    err = writeserial(ascii(data2));              // Sends data2
    tet(i,1) = toc();                             // Stops stopwatch and notes execution time
end
closeserial(A);                                   // Close the serial port

disp('Done.')                                     // Message.

t = 1:no_samples;                                 // Vector for axis
plot(t,tet','o')                                  // Plot results
ylabel('Total execution time [s]')                // Y axis label
xlabel('Sample (-)')                              // X axis label
legend('MagnetoShield Scilab Communication Speed Test')    // Legend

filename = ['MagnetoShield_Speed_Test_Data_Uno.dat'];      // Create filename for SCILAB
filenameMAT = ['MagnetoShield_Speed_Test_Data_Uno.mat'];   // Create filename for MATLAB
save(filename,'tet')                                       // Save data (Scilab)
savematfile(filenameMAT,'tet','-v7')                       // Save data (MATLAB)


