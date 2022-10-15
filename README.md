# Pinecil Interrupts Demo

Timer1 is set to fire an interrupt every second. An interrupt handler for 
timer1 checks whether the right button of Pinecil is pressed and displays
the button's status as well as the amount of interrupts handled so far.
The board will go to sleep whenever it's not handling an interrupt.
