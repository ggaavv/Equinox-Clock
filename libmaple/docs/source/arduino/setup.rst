.. _arduino-setup:

setup()
=======

The setup() function is called when a sketch starts. Use it to
initialize variables, pin modes, start using libraries, etc. The
setup function will only run once, after each powerup or reset of
the Arduino board.



Example
~~~~~~~

::

     
    int buttonPin = 3;
    
    void setup()
    {
      Serial.begin(9600);
      pinMode(buttonPin, INPUT);
    }
    
    void loop()
    {
      // ...
    }

