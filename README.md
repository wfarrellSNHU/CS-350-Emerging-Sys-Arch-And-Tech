# CS-350-Emerging-Sys-Arch-And-Tech

The SOS module was created as a milestone assignemnt that helped me understand State Machine development in C.  It uses various steps within the state to ensure that timing of the character being displayed in Morse code would work correctly.  Its based on substates of which character as well as which dot or dash or pause required by that character that its currently processing within that time period.  I used a button callback which would then switch the message from SOS to OK after its finished writing the current message and after a pause.

The Heater module was created as a final project that helped integrate a timer flag and many subsystems of the TI Launchpad.  We used GPIO for handling the buttons and led features.  The I2C for integrating with the temperature sensor built into the board.  Finally, we used the UART with a terminal connection so that we could read the current temp, set point, heating flag and how many seconds it had been polling since starting the application on the board.

I purposely used Bit shifting on the SOS to help improve those skills since I've only ever used bit flag type mechanisms in my C and C++ applications.

I was able to use the TI Code Compose application to help develop in an Eclipse like IDE for the assignemnts with this class.

Understanding how to work with subsystems of a board and comprehending the driver API documentation is so important on projects like these.  Being able to swim instead of sink when integrating with systems like these are a primary building factor of successful application development.  Making mental breakthroughs by iterative development is super important when coding for foreign systems and drivers as I was at the beginning of this class.  

I ensured during my development that I was to use standards as well as design standards as was exampled in our classbook, articles and documentation.  Between these, I was able to get terrific grades on all of my assignemnts in this class.  I had a really fun and challengine time with this course!

Thank you Professor Morales!
