SunCycle- a two-person solar charged data gathering cruising trike. 

current features:
Side by side seating with independent gearing and braking, 1000W 48v hub motor
Student designed two axis solar tracker made from auto parts.
Live streaming of humidity, temperature, and ambient light to xively

=========

everything we need for the Macomb Suncycle in one place (in theory).

Arduino Wifi Shield firmware has been updated to version 1.1 to allow us to push our sensor values over the network. 
The process should not have to be repeated, nor do I recommend it.

As of winter semester 2014, a xively account has been created and necessary libraries included in this repo
to allow us to post sensor values and graph them in real time at xively.com
the feed for this is:   

https://xively.com/feeds/1927926610

Currently, temperature/ humidity and ambient light readings are streamable. More info for setting streams are on the 
xively example sketch. GPS is the next step, though we have been unable to work much on this this semester.

The simpletest_GPS sketch can receive raw partial $GPRMC sentences, though parsing the data has proven difficult.



