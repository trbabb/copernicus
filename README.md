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

Documentation
=============

See: http://trbabb.github.io/copernicus/html/modules.html

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
    <td>OPEN</td>
    <td>7</td>
    <td>3</td>
    <td>GND</td>
  </tr>
  <tr>
    <td>RESERVED2 <br>("boot" on some breakouts)</td>
    <td>10</td>
    <td>6</td>
    <td>VCC</td>
  </tr>
  <tr>
    <td>VCC</td>
    <td>12</td>
    <td>8</td>
    <td>Supply voltage (3.3v)</td>
  </tr>
  <tr>
    <td>GND</td>
    <td>13</td>
    <td>9</td>
    <td>Ground (0v)</td>
  </tr>
  <tr>
    <td>XSTBY</td>
    <td>14</td>
    <td>10</td>
    <td>VCC</td>
  </tr>
  <tr><th colspan=4></th></tr>
  <tr>
    <td>TX-A</td>
    <td>23</td>
    <td>7</td>
    <td>Arduino serial RX pin</td>
  </tr>
  <tr>
    <td>RX-A</td>
    <td>21</td>
    <td>5</td>
    <td>Arduino serial TX pin, via voltage divider or logic level shifter.</td>
  </tr>
  <tr>
    <td>PPS</td>
    <td>19</td>
    <td>3</td>
    <td>Any Arduino interrupt pin (required for timing only).</td>
  </tr>
</table>

Further information
===================

The full manual describing the pins, features, and communication protocol 
of the Trimble Copernicus module is available 
[here](http://dlnmh9ip6v2uc.cloudfront.net/datasheets/Sensors/GPS/63530-10_Rev-B_Manual_Copernicus-II.pdf).
