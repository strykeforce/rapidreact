# Deadeye Vision System for Rapid React
This subproject provides a custom Deadeye pipeline (`HubPipeline`) for the Rapid React robot that is installed as an overlay to a [default Deadeye](https://github.com/strykeforce/deadeye) installation.

## Configuration
The pipeline daemon is configured as `deadeye-a` on the robot and can be accessed via the [admin web console](http://10.27.67.10). The current pipeline configuration is archived as [`config/deadeye-config.json`](config/deadeye-config.json).

## Installation
See the [README](./ansible/README.md) in the `ansible/` subdirectory for installation instructions.