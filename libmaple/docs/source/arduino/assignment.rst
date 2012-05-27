.. highlight:: cpp

.. _arduino-assignment:

= assignment operator (single equal sign)
=========================================

Stores the value to the right of the equal sign in the variable to
the left of the equal sign.


The single equal sign in the C++ programming language is called the
assignment operator. It has a different meaning than in algebra
class, where it indicated an equation or equality. The assignment
operator tells the microcontroller to evaluate whatever value or
expression is on the right side of the equal sign, and store it in
the variable to the left of the equal sign [#fgross]_.



Example
-------

::

     int sensVal;                  // declare an integer variable named sensVal
     senVal = analogRead(0);       // store the (digitized) input voltage at analog pin 0 in SensVal



Programming Tips
----------------

The variable on the left side of the assignment operator ( = sign )
needs to be able to hold the value stored in it. If it is not large
enough to hold a value, the value stored in the variable will be
incorrect.



Don't confuse the assignment operator [ = ] (single equal sign)
with the comparison operator [ == ] (double equal signs), which
evaluates whether two expressions are equal.



See Also
--------


-  `if (comparison operators) <http://arduino.cc/en/Reference/If>`_
-  `char <http://arduino.cc/en/Reference/Char>`_
-  `int <http://arduino.cc/en/Reference/Int>`_
-  `long <http://arduino.cc/en/Reference/Long>`_


.. rubric:: Footnotes

.. [#fgross] Experienced C++ programmers know this to be an
   oversimplification of what happens when the variable on the left
   hand side is an object.  See Richard Gillam's wonderful and scary
   `The Anatomy of the Assignment Operator
   <http://icu-project.org/docs/papers/cpp_report/the_anatomy_of_the_assignment_operator.html>`_
   for more information.
