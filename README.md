# alexadoorbellvolumecontrol

This is a small project I made for a friend. An ESP32-S2 simulates an Amazon Alexa compatible device that can turn on and off. The connected servo motor controls the volume panel on a door bell. So you can easily turn on and off your door bell sound.

![Example video](res/alexa.mp4)

On first start the device created an access point and you have to connect to your local WiFi. When the builtin led turns off the device is connected to your WiFi and has an IP address. The alexa service is based on [fauxmoESP](https://github.com/vintlabs/fauxmoESP) so you can find all relevant info how to connect it to your alexa on their project page.

The on and off commands will move the motor to `SERVO_MIN` and `SERVO_MAX` posititions defined in the code. You can also add an encoder to your ESP32 to control it manually. The servo motor goes into sleep mode after a few seconds so it doesn't waste energy and does not make annoying noises.