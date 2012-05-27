.. highlight:: cpp

.. _arduino-pinmode:

pinMode()
=========

.. doxygenfunction:: pinMode

.. doxygenenum:: WiringPinMode

Discussion
----------

pinMode() is usually called within :ref:`arduino-setup` in order to
configure a pin for a certain usage (although it may be called from
anywhere).


Example
-------

 ::

    
    int ledPin = 13;                 // LED connected to digital pin 13
    
    void setup()
    {
      pinMode(ledPin, OUTPUT);      // sets the digital pin as output
    }
    
    void loop()
    {
      digitalWrite(ledPin, HIGH);   // sets the LED on
      delay(1000);                  // waits for a second
      digitalWrite(ledPin, LOW);    // sets the LED off
      delay(1000);                  // waits for a second
    }



Arduino Compatibility Note
--------------------------

The libmaple implementation of pinMode() supports OUTPUT and INPUT
modes with a meaning identical to that of the Arduino function.

INPUT_ANALOG and PWM modes were added because the Maple does not
distinguish between analog and digital pins the same way the Arduino
does.  Unlike the Arduino, you **must call pinMode**\ () to set up a pin
for these purposes before a call to, e.g., :ref:`arduino-analogRead`.
In practice, this should only add a few lines of pinMode() calls to
your :ref:`arduino-setup` function.

OUTPUT_OPEN_DRAIN, INPUT_PULLUP, INPUT_PULLDOWN, and PWM_OPEN_DRAIN
modes represent functionality not currently available on Arduino
boards.

See also
--------

-  :ref:`arduino-constants`
-  :ref:`arduino-digitalwrite`
-  :ref:`arduino-digitalread`
-  Maple :ref:`GPIO <gpio>` reference page
-  Arduino `pinMode() <http://www.arduino.cc/en/Reference/PinMode>`_ reference
