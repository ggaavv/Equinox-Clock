.. _arduino-return:

return
======

Terminate a function and return a value from a function to the
calling function, if desired.



Syntax:
-------

return;



return value; // both forms are valid



Parameters
----------

value: any variable or constant type



Examples:
---------

A function to compare a sensor input to a threshold

::

     int checkSensor(){       
        if (analogRead(0) > 400) {
            return 1;
        else{
            return 0;
        }
    }



The return keyword is handy to test a section of code without
having to "comment out" large sections of possibly buggy code.



::

    void loop(){
    
    // brilliant code idea to test here
    
    return;
    
    // the rest of a dysfunctional sketch here
    // this code will never be executed
    }



See also
--------

`comments <http://arduino.cc/en/Reference/Comments>`_

