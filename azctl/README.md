# AAduino Zero Control

The azctl tool flashes applications to your AAduino Zero. Build one of the examples adding `bin` to generate a bin file and flash:

```
% cd examples/lowpower
% make bin

...

% azctl.py -d /dev/ttyUSB0 -u main.bin 
Download progress: 100% 
```
