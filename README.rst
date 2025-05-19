Overview
********

A simple sample that can be used with any :ref:`supported board <boards>.

Building and Running
********************

This application can be built and executed on QEMU as follows:

.. zephyr-app-commands::
   :zephyr-app: samples/my_app
   :host-os: unix
   :board: nucleo_f767zi
   :goals: run
   :compact:

To build for another board, change "qemu_x86" above to that board's name.

Exit QEMU by pressing :kbd:`CTRL+A` :kbd:`x`.
