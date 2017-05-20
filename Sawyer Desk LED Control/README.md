This is the arduino side of this code. The arduino will take the messages and perform the appropriate tasks

The arduino will take a custom message of type led_demo::led which contains the following members:

bool ON
bool self_control

int16[] rgb

int16 rate

bool blink

int8[] zone

EXPLANATIONS

zone[] contains the zone's that will be affected by this message. E.g if you wanted to affect zones 1,2 and 3 you would fill zones as such {1, 2, 3} the order is unimportant so {2, 3, 1} is also valid. If no existant zones are added the message will be ignored.

Setting ON to true will enable any other setting to take effect. Setting ON to false will turn off all the selected zones.

Setting self_control to true will allow you to directly set the RGB colour. When this is true the base will go to the colour set in the rgb array. Once true it will not try to perform any other actions other than keeping the selected zones at the selected colour. 

rgb contains the colour code for the lights, the array will be filled in the form {r, g, b}

setting blink to true (and self_control to false) will cause the lights to flash at the colour written in rgb  and at the rate given. Setting blink to false (and self_control to false) will cause the lights to pulse at the colour written in rgb and at the rate given.

rate contains the value in milliseconds that the blink or pulse actions will occur at. For blink it will be the time to change from on to off and vice versa. Eg, 200ms rate and blink is true. The base will be on for 200ms then off for 200ms. For pulse, the rate determines how long it will take the lights to do a full pulse.  That is, from 0 brightness to max brightness and back down to zero brightness.
