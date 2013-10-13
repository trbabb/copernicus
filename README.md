Overview
========

This module provides an API for controlling the Trimble Copernicus II GPS 
module from an Arduino microcontroller. 

Installation
============

Copy the `copernicus` folder (the one containing `copernicus.h`) into the 
Arduino [library folder](http://arduino.cc/en/Guide/Libraries). To use the 
library, simply

    #include <copernicus.h>
    
in your sketch.

Minimum connections
===================

<table>
  <tr>
    <th>Pin name</th>
    <th>Copernicus pin</th>
    <th>Sparkfun breakout pin</th>
    <th>Connect to</th>
  </tr>
  <tr>
    <td>Open</td>
    <td>7</td>
    <td>3</td>
    <td>LOW</td>
  </tr>
</table>

h1. Further information

The full manual describing the features and communication protocol of the 
Trimble Copernicus module is available 
[here](http://dlnmh9ip6v2uc.cloudfront.net/datasheets/Sensors/GPS/63530-10_Rev-B_Manual_Copernicus-II.pdf).
