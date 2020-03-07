Pi calculation program for ESP32
====

This program can calculate Pi more then 100,000 under decimal digits.
Using assembler and dual core feature of ESP32, it takes about 150 seconds.
The calculation method are various arctangent formula like Machin and Euler, etc.

# How to compile and execute

```
idf.py -p serial flash monitor 
```

# Sample output
![output](/esp32_pi_output.png "output")
