# Ansible Deployment Playbook

This [Ansible](https://docs.ansible.com) playbook will deploy the customized Deadeye pipeline daemon to the robot.

## Usage

```shell
$ poetry run ansible-playbook -i inventory/robots.yaml --vault-password-file=.vaultpw playbooks/deploy.yaml
```

## Installation

1. Install [Poetry](https://python-poetry.org) if you are setting up Ansible to run in the default Python virtual environment.
2. Run `poetry install` to install Ansible.

## Deploying From Private Repo

During the active season we may be using a private GitHub repo. Use a [deploy key](https://docs.github.com/en/developers/overview/managing-deploy-keys#deploy-keys) to allow the playbook to access the repo.

1. Create a new deployment SSH key-pair. GitHub provides [instructions](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent#generating-a-new-ssh-key).
2. Set the `github_key_src` Ansible variable to the path of the secret key and encrypt it with `ansible-vault encrypt <path-to-key-file>`.