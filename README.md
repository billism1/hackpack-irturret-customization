# IRTurret Control Code

This is the code from Crunchlabs' first [Hack Pack](https://www.crunchlabs.com/products/hack-pack-subscription) box - [the IRTurret](https://www.crunchlabs.com/products/ir-turret).

I copied the [the code from the CrunchLabs browser-based IDE](https://ide.crunchlabs.com/editor/8718988640487) into [the default folder here](./src/default). I modified it slightly (by adding the prototype function definitions at the top) so that the code can build outside of the Arduino IDE.

Note: The default code includes the header file, "PinDefinitionsAndMore.h" which (as far as I can tell) is not needed and the code builds fine without it. However, I asked the "Mark RoBot" AI chatbot what is in that file, and [the contents here in the PinDefinitionsAndMore.h file I created](./src/default/PinDefinitionsAndMore.h) is what it gave me. I'm sure this was just an LLM halucination though, as it seems to not be needed.

## Useful Links Related to the CrunchLabs Hack Pack IRTurret

- [Hack Pack Subscription](https://www.crunchlabs.com/products/hack-pack-subscription)
- [IRTurret Information Page](https://www.crunchlabs.com/products/ir-turret)
- [CrunchLabs Provided IRTurret Source Code and In-Browser IDE](https://ide.crunchlabs.com/editor/8718988640487)
- [Official Hack Pack Discord Server](https://discord.gg/gKvCPtbmcg)

## Customizations

I created [a copy of the code](./src/custom) as a starting point for any customizations. I plan on flashing and operating the IR Turret with an ESP32 which will provide more compute power, more GPIO, and has built-in wifi and bluetooth transceivers. This should make it fairly easy to allow controlling the turret from a phone, for example.

The Servo library (used by the code included in the IT Turret) is incompatible with ESP32. The ESP32Servo library (included in the "custom") code is a drop-in replacement for this, at least for the needs of this project.
