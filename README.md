# Padauk-TPS
Padauk version of Elektronic-labor TPS 

This a port of the TPS system to a Padauk PFS154-S16 written in 'Mini-C'.<br>
As this chip has no onboard EEPROM an external 24C02 I2C EEPROM is used.<br>

<b>The port mapping is as follows:</b>
<ol>
<li><b>PA0</b> = PWM output
<li><b>PA4</b> = ADC using the built in comparator 
<li><b>PA3</b> = I2C Clock
<li><b>PA5</b> = I2C Data
<li><b>PA6</b> = Switch1
<li><b>PA7</b> = Switch2
<br>
<li><b>PB0-PB3</b> = A1-A4 and LED outputs<br> 
<li><b>PB4-PB7</b> = E1-E4 inputs <br>
</ol>

Timer 16 is used as a 1 Millisecond interrupt for the wait instructions<br>
Instruction 5-9 is now PWM with the A register representing  the pulse width<br>
Instruction 5-10 gives servo style PWM between ~ 1 an 2 milli seconds.<br>



