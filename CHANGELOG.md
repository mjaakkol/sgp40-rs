# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/en/1.0.0/)
and this project adheres to [Semantic Versioning](http://semver.org/spec/v2.0.0.html).

## 1.0.0 - 2025-02-28

### Changed

- Uses embedded hal 1.0.0 now.
- Upgraded crates.
- Small code improvements.
- Upgraded to use 2024 edition.
- Added cargo.lock into library

## 0.0.4 - 2021-03-17

### Fixed

- Software is no_std now.

### Changed

- VOC index algorithm is using Fixed-point math now to make it closer to the original and enable quick no_std implementation. The new implementation should also be a bit more accurate but time will tell.


## 0.0.3 - 2021-03-12

### Fixed

- Concat removed to get the software to compile for no-std.


## 0.0.2 - 2021-03-11

### Changed

- Moved VOC index calculation behind feature flag as it is no-std. The feature is turned on as default.


## 0.0.1 - 2021-02-09

Initial release to crates.io.

[1.0.0]: https://github.com/mjaakkol/sgp40-rs/compare/v0.0.4...v1.0.0

[0.0.4]: https://github.com/mjaakkol/sgp40-rs/compare/v0.0.3...v0.0.4

[0.0.3]: https://github.com/mjaakkol/sgp40-rs/compare/v0.0.2...v0.0.3

[0.0.2]: https://github.com/mjaakkol/sgp40-rs/compare/v0.0.1...v0.0.2

[i5]: https://github.com/mjaakkol/sgp40-rs/pull/5
