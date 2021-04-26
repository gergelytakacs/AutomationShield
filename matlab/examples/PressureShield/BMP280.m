%   MATLAB function to read temperature and pressure
%   from BMP280 temperature and pressure sensor.
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
%
%   Created by Martin Staron.
%   Last update: 26.4.2021.

function y = BMP280(dev)
persistent i;
global dig_T1 dig_T2 dig_T3 dig_P1 dig_P2 dig_P3 dig_P4 dig_P5 dig_P6 dig_P7 dig_P8 dig_P9 adc_t t_fine;
if (isempty(i))
    i = 1
end
        
if (i==1)
   
            UT_MSB = readRegister(dev,'FA','uint8');
            UT_LSB = readRegister(dev,'FB','uint8');
            UT_XLSB = readRegister(dev,'FC','uint8');
            
            UT_MSBbit = bitget(UT_MSB,8:-1:1);
            UT_LSBbit = bitget(UT_LSB,8:-1:1);
            UT_XLSBbit = bitget(UT_XLSB,8:-1:5);
            bitsT = [UT_MSBbit UT_LSBbit UT_XLSBbit];
            adc_t = bi2de(bitsT,'left-msb');    
            
            dig_T1M = readRegister(dev,'89','uint8');
            dig_T1L = readRegister(dev,'88','uint8');
            dig_T1Mbit = bitget(dig_T1M,8:-1:1);
            dig_T1Lbit = bitget(dig_T1L,8:-1:1);
            bitsT1 = [dig_T1Mbit dig_T1Lbit];
            dig_T1 = bi2de(bitsT1,'left-msb');

            dig_T2M = readRegister(dev,'8B','uint8');
            dig_T2L = readRegister(dev,'8A','uint8');
            dig_T2Mbit = bitget(dig_T2M,8:-1:1);
            dig_T2Lbit = bitget(dig_T2L,8:-1:1);
            bitsT2 = [dig_T2Mbit dig_T2Lbit];
            dig_T2 = bi2de(bitsT2,'left-msb');
            
            dig_T3M = readRegister(dev,'8D','uint8');
            dig_T3L = readRegister(dev,'8C','uint8');
            dig_T3Mbit = bitget(dig_T3M,8:-1:1);
            dig_T3Lbit = bitget(dig_T3L,8:-1:1);
            bitsT3 = [dig_T3Mbit dig_T3Lbit];
            dig_T3 = bi2de(bitsT3,'left-msb');
           
            dig_P1M = readRegister(dev,'8F','uint8');
            dig_P1L = readRegister(dev,'8E','uint8');
            dig_P1Mbit = bitget(dig_P1M,8:-1:1);
            dig_P1Lbit = bitget(dig_P1L,8:-1:1);
            bitsP1 = [dig_P1Mbit dig_P1Lbit];
            dig_P1 = bi2de(bitsP1,'left-msb');
              
            dig_P2M = readRegister(dev,'91','uint8');
            dig_P2L = readRegister(dev,'90','uint8');
            dig_P2Mbit = bitget(dig_P2M,8:-1:1);
            dig_P2Lbit = bitget(dig_P2L,8:-1:1);
            bitsP2 = [dig_P2Mbit dig_P2Lbit];
            dig_P2 = bi2de(bitsP2,'left-msb');
              
            dig_P3M = readRegister(dev,'93','uint8');
            dig_P3L = readRegister(dev,'92','uint8');
            dig_P3Mbit = bitget(dig_P3M,8:-1:1);
            dig_P3Lbit = bitget(dig_P3L,8:-1:1);
            bitsP3 = [dig_P3Mbit dig_P3Lbit];
            dig_P3 = bi2de(bitsP3,'left-msb');
              
            dig_P4M = readRegister(dev,'95','uint8');
            dig_P4L = readRegister(dev,'94','uint8');
            dig_P4Mbit = bitget(dig_P4M,8:-1:1);
            dig_P4Lbit = bitget(dig_P4L,8:-1:1);
            bitsP4 = [dig_P4Mbit dig_P4Lbit];
            dig_P4 = bi2de(bitsP4,'left-msb');
              
            dig_P5M = readRegister(dev,'97','uint8');
            dig_P5L = readRegister(dev,'96','uint8');
            dig_P5Mbit = bitget(dig_P5M,8:-1:1);
            dig_P5Lbit = bitget(dig_P5L,8:-1:1);
            bitsP5 = [dig_P5Mbit dig_P5Lbit];
            dig_P5 = bi2de(bitsP5,'left-msb');
              
            dig_P6M = readRegister(dev,'99','uint8');
            dig_P6L = readRegister(dev,'98','uint8');
            dig_P6Mbit = bitget(dig_P6M,8:-1:1);
            dig_P6Lbit = bitget(dig_P6L,8:-1:1);
            bitsP6 = [dig_P6Mbit dig_P6Lbit];
            dig_P6 = bi2de(bitsP6,'left-msb');
            
            dig_P7M = readRegister(dev,'9B','uint8');
            dig_P7L = readRegister(dev,'9A','uint8');
            dig_P7Mbit = bitget(dig_P7M,8:-1:1);
            dig_P7Lbit = bitget(dig_P7L,8:-1:1);
            bitsP7 = [dig_P7Mbit dig_P7Lbit];
            dig_P7 = bi2de(bitsP7,'left-msb');
             
            dig_P8M = readRegister(dev,'9D','uint8');
            dig_P8L = readRegister(dev,'9C','uint8');
            dig_P8Mbit = bitget(dig_P8M,8:-1:1);
            dig_P8Lbit = bitget(dig_P8L,8:-1:1);
            bitsP8 = [dig_P8Mbit dig_P8Lbit];
            dig_P8 = bi2de(bitsP8,'left-msb');
              
            dig_P9M = readRegister(dev,'9F','uint8');
            dig_P9L = readRegister(dev,'9E','uint8');
            dig_P9Mbit = bitget(dig_P9M,8:-1:1);
            dig_P9Lbit = bitget(dig_P9L,8:-1:1);
            bitsP9 = [dig_P9Mbit dig_P9Lbit];
            dig_P9 = bi2de(bitsP9,'left-msb');
              
            var_1 = ((adc_t)/16384-(dig_T1)/1024)*(dig_T2);
            var_2 = (((adc_t)/131072-(dig_T1)/8192)*((adc_t)/131072-(dig_T1)/8192))*(dig_T3);
            t_fine = var_1 + var_2;

            i=2
end       

            UP_MSB = readRegister(dev,'F7','uint8');
            UP_LSB = readRegister(dev,'F8','uint8');
            UP_XLSB = readRegister(dev,'F9','uint8');
             
            UP_MSBbit = bitget(UP_MSB,8:-1:1);
            UP_LSBbit = bitget(UP_LSB,8:-1:1);
            UP_XLSBbit = bitget(UP_XLSB,8:-1:5);
            bitsP = [UP_MSBbit UP_LSBbit UP_XLSBbit];
            adc_p = bi2de(bitsP,'left-msb');
            
            var1 = (t_fine/2)-64000;
            var2 = var1*var1*dig_P6/32768;
            var2 = var2+var1*dig_P5*2;
            var2 = (var2/4)+(dig_P4*65536);       
            var1 = (dig_P3*var1*var1/524288+dig_P2*var1)/524288;
            var1 = (1+var1/32768)*dig_P1;
            p = 1048576-adc_p;
            p = (p-(var2/4096))*6250/var1;
            var1 = dig_P9*p*p/2147483648;
            var2 = p*dig_P8/32768;
            y = int32((p+(var1+var2+dig_P7)/16));

end