# DHT22 Humidity and Temperature Sensor Driver
## For the RP2040 microcontroller
Based on **rp2040_hal** and **embedded_hal**.  

Communicates though a single data wire which requires some internal pin handling for bi-directional state.  

Reference:
<https://cdn-shop.adafruit.com/datasheets/Digital+humidity+and+temperature+sensor+AM2302.pdf>


## Example:
```rust

use lh_dht22_rp as dht22;
...
// Hal Boilerplate ...
...
let dht_pin = pins.gpio1;
let mut dht = dht22::DHT22::new(dht_pin, timer);

match dht.read() {
   Ok((humidity, temperature)) => {
         println!("Humidity   : {:.1} %RH", humidity);
         println!("Temperature: {:.1} C", temperature); },
   Err(e) => println!("Err: {e}"),
}
```