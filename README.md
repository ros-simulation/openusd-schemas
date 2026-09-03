# openusd-schemas

Interoperable OpenUSD ROS core and extension schemas supplemented by conversion and compliance tooling

## Tools

### Compliance Checker

Validates OpenUSD assets against the REP-0158 interoperability standard.
Ships in two interchangeable forms backed by the same `plugInfo.json`:

- **`ros-usd-check`** — pure-Python CLI installable from PyPI; no OpenUSD build
  required.
- **`usdRosValidators`** — USD plugin discoverable by stock `usdchecker`
  on a full OpenUSD install (≥ v26.05).

See the [compliance checker README](tools/compliance_checker/README.md) for
installation and usage details.
