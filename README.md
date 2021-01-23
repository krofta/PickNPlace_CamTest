Das Repository enthält alle Codeprojekte für eine einfache Pick and Place Maschine.
Unter Anderem gehören dazu:

Firmware für Computer Vision Modul mit STM32H730 mit 8Mb external SDRAM und DCMI interface für OV7670 camera
Codeinspiration für das Camera Interface von:<br />
https://github.com/iwatake2222/DigitalCamera_STM32 <br />
https://github.com/erikandre/stm32-ov7670 <br />
http://supuntharanga.blogspot.co.uk/2014/04/stm32f4-discovery-board-ov7660-or.html <br />
http://embeddedprogrammer.blogspot.co.uk/2012/07/hacking-ov7670-camera-module-sccb-cheat.html <br />
http://www.urel.feec.vutbr.cz/MPOA/2014/cam-ov7670 <br />
Code für den SDRAM aus Beispielcode des Discovery-Board STM32F429I-DISC1.
Inspiration für die Ansteuerung von WS2812 LEDs per DMA:<br />
https://github.com/Crazy-Geeks/STM32-WS2812B-DMA<br />
https://narodstream.ru/stm-urok-119-ws2812b-lenta-na-umnyx-svetodiodax-rgb-chast-2/<br />
Ansteuerung des SPI-Displays Waveshare 2" 320x240px ST7789:<br />
https://github.com/Floyd-Fish/ST7789-STM32<br />
https://github.com/ananevilya/Arduino-ST7789-Library<br />
https://github.com/afiskon/stm32-st7735<br />
All image-processing algorithms are self programmed without opencv just in c. Spacial Thanks to this guy: yuxianguo
who helped me programming a fast blob coloring algorithm. <br />
https://www.programmersought.com/article/36133540459/ <br />
Special thanks to Alyssa Quek who gave me clever advices to calculate orientation of a blob: <br />
https://alyssaq.github.io/2015/computing-the-axes-or-orientation-of-a-blob/ <br />
Firmware of the controller module to execute reduces GCODE.
Move motors and communicate with feeders and cv-modules via can-bus.
SD-Card for parameters and other stuff.

Firmware of the feeders. Communicate with controller board via can-bus. 
Move motors to feed tape. SPI-Flash to store bar codes.

Qt-Desktop program to read manufacturing data linke BOM, Pick and Place data, dxf files of PCB.
Generate GCODE for the machine. General pick and place parameters. This software dont know any special 
parameters about the machine (not like openPNP) and has no own computer vision. It just cares about the pick and place components. 
The rest is made by the machine itself, like calculate xy offset and rotation of picked ic, 
transformation of the coordinates with the position of fiducials.