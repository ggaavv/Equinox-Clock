.. highlight:: sh

.. _troubleshooting:

Troubleshooting
===============

This page documents common problems and their solutions.

.. contents:: Contents
   :local:

===================
 Hardware problems
===================

The status LED is throbbing and I can't upload my program!
----------------------------------------------------------

The LED throbs when there has been a failed software :ref:`assertion
<language-assert>`.

You can still reprogram by resetting the board and uploading during
the short window when the bootloader waits for a program.

To make this window longer (it can be hard to get the timing right),
use :ref:`perpetual bootloader mode
<troubleshooting-perpetual-bootloader>`.

My board is bricked! I can't upload via the bootloader no matter what!
----------------------------------------------------------------------

Use the hardcoded serial STM32 serial bootloader to re-flash the
bootloader.

.. TODO insert docs on reflashing the bootloader

If it really is bricked, and you think it's our fault, `contact us
<http://leaflabs.com/contact>`_\ !

My 5v peripheral doesn't work! (I2C, SPI, USART, etc)
-----------------------------------------------------

Yup, the Maple is a 3.3v board. You may need to use a level
converter. See the :ref:`compatibility <compatibility>`, :ref:`GPIO
<gpio>`, or other :ref:`hardware specific documentation <hardware>`
for more information.

The reset and D38/serial buttons don't seem to work reliably!
-------------------------------------------------------------

A few rev3 boards shipped in May-June 2010 may have had unreliable
buttons; see the :ref:`errata page <errata>` for details.  `We're
happy to replace these for you <http://leaflabs.com/contact>`_\ !


.. _troubleshooting-ide-install:

=======================
 Installation problems
=======================

I don't have root/administrator access!
---------------------------------------

There are probably hacks or work-arounds to getting programs uploaded
without higher level system permissions.  If you can access USB
character devices (ACM or ttyUSB style), you should be able to
communicate with the Maple and reprogram using an FTDI converter and
the serial bootloader, but we haven't tried.

.. TODO: be more helpful

[Linux] I don't use udev!
-------------------------

There is probably a simple way to get autoconfiguration working with
devfs; in the meantime, you could try running the entire IDE as root.

.. TODO: be more helpful

.. _troubleshooting-ide-usage:

==============
 IDE problems
==============

[Mac OS X] The "Board" and "Serial Port" menu items are missing!
----------------------------------------------------------------

Sometimes this happens if you try to compile or upload without having
a board selected.  The work-around is to restart the IDE. Mysterious!

.. _troubleshooting-compilation:

==========================
 Common compiler problems
==========================

``NullPointerException``
------------------------

A classic! Make sure you have selected a board from the pulldown menu. 

.. TODO: remove when Python version is released

``undefined reference to setup()/loop()``
-----------------------------------------

Your sketch/program either does not include one of the `setup
<setup()>`_ or `loop <loop()>`_ functions, or it was not found by the
compiler. Your program must include both ``void setup()`` and ``void
loop()`` functions; they don't have to do anything, but they **must**
be there.

You can start with an example program (to see one in the IDE, click on
File > Examples > Stubs > BareMinimum) to get the basic structure.
See also the :ref:`language <language>` documentation.

This is a common error when your entire sketch is blank.

``error: 'Serial' was not declared in this scope``
--------------------------------------------------

The classic Arduino has only one USART device and uses the unique name
"Serial" to control it. Larger devices like the Arduino Mega and the
Maple have multiple USARTS referred to as ``Serial1``, ``Serial2``,
etc. You probably want ``Serial2`` on the Maple; that's the one
connected to pins D0 and D1. See also the :ref:`USART docs <usart>`.

``File(s) not found``
---------------------

There is an intermittent bug with the temporary directory build system
that on occasion will lose many of the ``#include``\ d libmaple
files. If you recompile everything, it should be fine.

.. TODO remove when the Python version is released

.. _troubleshooting-upload:

========================
 Common upload problems
========================

My program is too large!
------------------------

First, make sure you're using the FLASH target instead of RAM; there
is several times more FLASH memory available for user programs.

``No DFU capable USB device found``
-----------------------------------

This probably means you don't have a Maple plugged in or powered on! 

I have multiple Maples installed; how do I know which one will get flashed?
---------------------------------------------------------------------------

Because the Maple IDE uses DFU to upload programs, you can't select a
particular Maple from the Serial Port menu to upload to a particular
board. There's no solution to this problem for now: you'll have to
just plug in the Maples one at a time. If this is a real problem let
us know and we'll see if we can come up with a better solution.

.. _troubleshooting-shell:

=====================
 Command-line issues
=====================

My flash programs don't seem to stick; they behave like they are RAM!
---------------------------------------------------------------------

Make sure you :command:`make clean` when switching between FLASH and
RAM targets; :command:`make` isn't smart enough to rebuild everything
that needs to be for the new target.

[Linux] ``cdc_acm 3-1:1.0: no more free acm devices``
-----------------------------------------------------

This is a nasty one! It means that all 32 possible CDC_ACM serial
devices (:file:`/dev/ttyACM25`, etc.) have been used up.

The usual cause is using a serial port monitor and not closing it
before restarting the Maple or uploading a new program.  The operating
system doesn't like that, and locks up that device. After reset the
Maple comes back up as a new device. If you develop heavily and don't
restart you'll blow right through all 32 devices.

The lazy solution is to always close the monitor before restarting,
and if you get this error in :file:`dmesg` after a dfu-util "Not
Found" error, restart you machine.

The hacker solution is to restart your cdc_acm kernel module. On
Ubuntu 09.10, this goes a little something like::

  $ sudo rmmod cdc-acm
  $ sudo insmod /lib/modules/2.6.31-20-generic/kernel/drivers/usb/class/cdc-acm.ko

.. _troubleshooting-tips-tricks:

=================
 Tips and Tricks
=================

.. _troubleshooting-perpetual-bootloader:

Perpetual Bootloader Mode
-------------------------

In this mode, Maple stays a DFU device and does not jump to user code
until the next reset.  This is useful for guaranteeing that your Maple
will be available for reprogramming.

To put your Maple into perpetual bootloader mode:

#. Plug your Maple into the USB port.

#. Hit the reset button on your Maple (it's the small button at the
   bottom left, labeled RESET).  Notice that it blinks quickly 6
   times, then blinks slowly a few more times.

#. Hit reset again, and this time push and hold the other button
   during the 6 fast blinks (the button is on the top right; it is
   labeled BUT). You can release it once the slow blinks start.

