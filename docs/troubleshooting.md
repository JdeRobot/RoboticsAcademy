### [Back to main README.](../README.md)

# Troubleshooting Robotics Academy

## Docker does not launch

If the Robotics Academy Docker container does not launch properly and shows an error saying it could not connect to the database, **only restart the Robotics Academy container**, not the database container that was launched separately via `docker compose`.

---

## PostgreSQL Port Conflict When Starting Database Container

### Issue

When following the [official installation instructions](https://jderobot.github.io/RoboticsAcademy/user_guide/#installation) to set up the Robotics Academy environment using Docker, you might encounter an error when launching the PostgreSQL container:

```bash
Error starting userland proxy: listen tcp4 0.0.0.0:5432: bind: address already in use
```

This usually occurs because port 5432 is already in use, commonly by a locally installed PostgreSQL service.

### Example Scenario

Suppose you successfully start the PostgreSQL Docker container once. Later, after stopping the container and rebooting your machine, your system's local PostgreSQL service might start automatically and occupy port 5432. If you then try to restart the container, Docker will fail to bind to the port.

## Solution – Disable Local PostgreSQL Temporarily

### Step 1: Stop and Disable Local PostgreSQL
Run the following commands before launching the PostgreSQL container:

```bash
sudo systemctl stop postgresql
sudo systemctl disable postgresql
```

After this follow the steps provided in [official installation instructions](https://jderobot.github.io/RoboticsAcademy/user_guide/#installation).
