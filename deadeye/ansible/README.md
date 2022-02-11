# Ansible Deployment Playbook

This [Ansible](https://docs.ansible.com) playbook will deploy the customized Deadeye pipeline daemon to the robot. It synchronizes the local deadeye directory to the remote host for building.

**Important:** This will only update the pipeline daemon (`deadeyed`) so make sure you first have a functioning installation of the base [Deadeye](https://github.com/strykeforce/deadeye) system.

## Usage

```shell
$ poetry run ansible-playbook -i inventory/robots.yaml playbooks/deploy.yaml -l deadeye-a
```

## Installation

1. Install [Poetry](https://python-poetry.org) if you are setting up Ansible to run in the default Python virtual environment.
2. Run `poetry install` to install Ansible.