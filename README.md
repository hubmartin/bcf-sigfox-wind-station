# Sigfox wind station

This project was created for model airport to know when are good conditions for flying. This project consists of hardware, this firmware and backend Python script which sends data to ThingSpeak account.

Right now the station is offline and no graphs are shown https://thingspeak.com/channels/310705

Photos of the hardware and installation https://photos.app.goo.gl/9pplbKDJL1qy198v1

## Hardware
This weather stations has rainfall sensor and temperature/humidity electronics with 433 transmitter. This rainfall and transmitter is not used in this project. It is possible to find a shop which sells just separate parts and build can be then cheaper.
https://www.conrad.cz/kombinovany-senzor-teploty-vlhkosti-vetru-srazek-wh14c.k1414630

  - Core Module
  - Sigfox Module
  - Sensor Module
  - Battery Module
  - Kombinovaný senzor teploty/vlhkosti/větru/srážek WH14C

The wind speed/direction measurement is wired to the Sensor Module. The wiring can be found in this PDF https://www.sparkfun.com/datasheets/Sensors/Weather/Weather%20Sensor%20Assembly..pdf

## Backend script
https://github.com/hubmartin/bch-sigfox-wind-station
