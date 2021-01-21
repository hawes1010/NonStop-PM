# NonStop PM
 Nonstop PM Pump Controller V2

# The Nonstop PM sensor project has a few caveats.
We need to use two versions of teensy that are the same for now.  I could not get a 4.0 and 3.2 to work together as master and slave devices
PM_PumpControl_NonStop_v4 is the latest version.  
The code in Read_PS() is where most of the data reading happens.
We don't have a careful timing system for getting data from the I2C sensor.  We will need one if we want
to use one bus for all sensors and communications.

Datasheet for i2c sensor on the "allsensors" describes how data is collected from the device