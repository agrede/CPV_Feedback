* Closed loop tracking algorithm for CPV test setup  *
* Michael Lipski				     *
* Summer 2016					     *

feedback_revised is the sketch that was used to successfully track the sun for over 2 hours during outdoor testing.
Uses a simple "perturb and observe" algorithm to attempt to maximize the voltage tied to an analog input pin on
the Arduino.  

Check to make sure the correct analog reference voltage is being used: if the input voltage is between 0-5 V, the default reference voltage can be used.  If the maximum input voltage is lower than 5 V, it is better to use an external reference voltage.  Analog voltages must not exceed reference voltage or fall outside the range of 0-5 V!

Before compiling, copy Zaber_X folder into your Arduino/libraries folder.