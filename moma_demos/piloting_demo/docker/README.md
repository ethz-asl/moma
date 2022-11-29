# PILOTING Docker Setup

## Usage

Ensure to have "tmux" installed and a ~/.tmux.conf with the following content:

```
setw -g mouse on
```

### Simple

```bash
docker-compose up
```

Then, access the host on http port 80.

### System service

See the systemd file `piloting-dashboard.service`
