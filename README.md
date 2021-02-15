# sgp40-rs
Platform agnostic Rust device driver for Sensirion SGP40

[![Build status][workflow-badge]][workflow]
[![Crates.io Version][crates-io-badge]][crates-io]
[![Crates.io Downloads][crates-io-download-badge]][crates-io-download]
![No Std][no-std-badge]

Platform agnostic Rust driver for Sensirion SGP40 gas sensor using the [`embedded-hal`](https://github.com/japaric/embedded-hal) traits.

## Sensirion SGP40

Sensirion SGP40 is 2nd generation low-power accurate gas sensor for air quality application. The sensor uses I²C interface and measures VOC (*Total Volatile Organic Compounds*)

Datasheet: https://www.sensirion.com/file/datasheet_sgp40

## Development status
The sensor is feature complete and the future development evolves:
- Storing and setting states haven't been tested yet.
- VOC index follows pretty closely with C implementation values but the jury is still out there to ponder if the algoritm truly calculates things properly.
- Serial number fetching is not working yet.
- Moving into using Embedded-hal 1.x


## License

Licensed under either of

 * Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
   http://www.apache.org/licenses/LICENSE-2.0)
 * MIT license ([LICENSE-MIT](LICENSE-MIT) or
   http://opensource.org/licenses/MIT) at your option.


### Contributing

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall
be dual licensed as above, without any additional terms or conditions.

<!-- Badges -->
[workflow]: https://github.com/mjaakkol/sgp40-rs/actions?query=workflow%3ARust
[workflow-badge]: https://img.shields.io/github/workflow/status/mjaakkol/sgp40-rs/Rust/master
[crates-io]: https://crates.io/crates/sgp40
[crates-io-badge]: https://img.shields.io/crates/v/sgp40.svg?maxAge=3600
[crates-io-download]: https://crates.io/crates/sgp40
[crates-io-download-badge]: https://img.shields.io/crates/d/sgp40.svg?maxAge=3600
[no-std-badge]: https://img.shields.io/badge/no__std-yes-blue
