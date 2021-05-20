# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/).

## Unreleased
### Changed
- Selectors: Uniform selector now treats rotate as factor between two consecutive points, instead of rotating the whole path.

## 1.1.0 - 2021-05-18
### Added
- Automatic versioning of the package.
- Optimizers: Braghin supports parameter `path_reduction` for reducing the path prior to the interpolation.
- *Selectors* as a class of methods used for retrieving group centers from the centerline.
- Selectors: Uniform selector that supports selection rotation.
- Script 'ng_run' for invoking the algorithm directly from Terminal.

### Changed
- Optimizers: Braghin's parameters `endpoint_distance` and `endpoint_accuracy` are modifiable.
- Optimizers: Braghin during its transformation works with the full length of the centerline instead of a subset.

### Fixed
- Exception message is shown when parsing JSON fails.
- Variate run is using string formatting to properly display non-int variables.

## 1.0.0 - 2021-03-16
### Added
- Reimplementation of `ng_trajectory` as a standalone Python package.
