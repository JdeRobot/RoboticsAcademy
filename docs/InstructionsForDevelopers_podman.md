### [Back to main README.][]

[Back to main README.]: ../README.md

# Using Podman (rootless alternative to Docker)

Podman runs containers without a daemon and without root. It is an alternative to the Docker setup, not a replacement.

This setup works on Linux only. Windows and macOS users should use Docker.

Run these commands from the repository root.

**Requirements:** Ubuntu 24.04 or greater, and an NVIDIA GPU if you want GPU acceleration. Other Linux distributions should work, but the package versions below are for Ubuntu 24.04.

**Important:** never run `podman` or `podman-compose` under `sudo`. Rootless operation is the point of this setup, and `sudo` switches to a separate root-owned container store. Both launcher scripts refuse to run as root.

## Install Podman

```
sudo apt install podman podman-compose
```

That is the whole installation. The three packages Ubuntu 24.04 ships (Podman 4.9.3, podman-compose 1.0.6 and crun 1.14.1) work together. You do not need a PPA, a pip install, or any OCI-runtime changes.

Confirm:

```
podman --version
podman-compose --version
```

**_NOTE_**: If you have installed a newer Podman by other means (Homebrew, a PPA, a static build), `podman-compose` 1.0.6 is **not** compatible with Podman 6.x. It fails with `cannot index slice/array with type string`. Either keep the matched `apt` pair, or upgrade both together.

## NVIDIA GPU setup (optional)

Skip this section if you only need CPU rendering.

Install the NVIDIA proprietary driver and the NVIDIA Container Toolkit following their official documentation.

You must generate the CDI specification yourself; the launcher scripts only check that one exists and stop with instructions if it does not.

### Generate a compatible CDI specification

Current toolkit versions emit a CDI specification containing an `additionalGids` field, which requires CDI schema v0.7.0. Podman 4.9.3 cannot parse it. It rejects the entire specification and reports no GPU devices, with `json: unknown field "additionalGids"`. Generate the specification without that field. The result is a v0.5.0 schema that Podman 4.9.3 reads correctly.

**If `/etc/nvidia-container-toolkit/nvidia-cdi-refresh.env` exists**, a service regenerates the specification automatically after driver updates. Set the flag there so it survives those regenerations, and let the service own the file:

```
echo 'NVIDIA_CTK_CDI_GENERATE_FEATURE_FLAGS=no-additional-gids-for-device-nodes' \
  | sudo tee -a /etc/nvidia-container-toolkit/nvidia-cdi-refresh.env
sudo systemctl restart nvidia-cdi-refresh.service
```

Do **not** also generate a specification into `/etc/cdi` on these systems. Podman searches both `/etc/cdi` and `/var/run/cdi`, and two specifications declaring the same devices will drift apart after the next driver update.

**If that file does not exist**, generate the specification yourself. Re-run this after every driver upgrade:

```
sudo nvidia-ctk cdi generate \
  --feature-flag no-additional-gids-for-device-nodes \
  --output=/etc/cdi/nvidia.yaml
```

Either way, confirm the schema version:

```
grep -m1 cdiVersion /etc/cdi/nvidia.yaml /var/run/cdi/nvidia.yaml 2>/dev/null
# expect: cdiVersion: 0.5.0
```

### Join the video and render groups

```
sudo usermod -aG video,render "$USER"
```

Log out and back in for this to take effect.

The flag above removes the mechanism that would otherwise give the container access to the render node, so this step is required. On a desktop your login session may mask the problem by granting temporary device access; that will not exist on a server or after a reboot into a different session.

### Test the GPU before launching

```
podman run --rm --device nvidia.com/gpu=all ubuntu nvidia-smi
```

This must exit cleanly and print your GPU. If it fails here, no compose configuration will fix it.

You do not need to bind-mount `/dev/dri`. CDI injects the render nodes.

## Launch the stack

Pass `-p` to the existing launcher scripts. You do not select a compose file yourself; the script picks one and copies it to `docker-compose.yaml`.

| Command                              | Mode        | Acceleration | Compose file used                |
| ------------------------------------ | ----------- | ------------ | -------------------------------- |
| `./scripts/develop_academy.sh -p -n` | Development | NVIDIA       | `dev_humble_podman_nvidia.yaml`  |
| `./scripts/develop_academy.sh -p`    | Development | CPU          | `dev_humble_cpu.yaml`            |
| `./scripts/run_academy.sh -p -n`     | User        | NVIDIA       | `user_humble_podman_nvidia.yaml` |
| `./scripts/run_academy.sh -p`        | User        | CPU          | `user_humble_cpu.yaml`           |

The CPU path uses the shared Docker CPU configuration. Those files declare no `runtime:`, no `deploy:` reservations and no `NVIDIA_*` variables, so Podman runs them unchanged.

The scripts reject `-p -g` with an error. Intel and AMD passthrough is not available on the Podman path. The Docker `-g` path bind-mounts `/dev/dri` directly, which under rootless Podman depends on host device permissions the project cannot assume, and no Podman compose configuration exists for it. Use `-p -n` for NVIDIA or `-p` for CPU.

The script runs `podman-compose -p roboticsacademy up`. It does not pass the `--compatibility` flag used by the Docker NVIDIA path.

Before launching, make sure you are not running as root, that `podman` and `podman-compose` are on `PATH`, and, when using `-n`, that a CDI specification exists in `/var/run/cdi` or `/etc/cdi`.

Then open [http://localhost:7164/](http://localhost:7164/).

**_NOTE_**: On SELinux hosts (Fedora, RHEL, CentOS Stream), `develop_academy.sh -p` bind-mounts the repository into the container, and the shared CPU compose file carries no SELinux label directive, so expect permission denials on those mounts. The NVIDIA Podman files are unaffected, and so is `run_academy.sh -p`, which mounts nothing.

**_NOTE_**: `scripts/radi_monitor/` does not work under Podman. It shells out to `docker stats`, `docker exec` and `docker ps`.

## Verifying GPU acceleration worked

Two independent checks:

1. The exercise Status Bar in the browser reads `GPU: NVIDIA`. This comes from the render device detected inside the container; `GPU: OFF` means none was found.
2. Inside the container, `vglrun glxinfo` reports your GPU rather than `llvmpipe`.

## Stopping the stack

Ctrl+C does not stop the stack. Stop it explicitly from a second terminal:

```
podman-compose -p roboticsacademy down
```

## Migrating from an existing Docker setup

Podman keeps its own image store. Images built or pulled under Docker are not visible to Podman. Re-pull them or move them from your local docker images using the following command replacing `DOCKER_IMAGE_NAME` with the image name like `robotics-database:latest` or `robotics-academy:latest`:

```bash
docker images --format docker-daemon:{{.Repository}}:{{.Tag}} | grep DOCKER_IMAGE_NAME | xargs podman pull
```

Exercise workspace files left behind by earlier runs can block autosave. The symptom is an "Error saving file" popup with no message: the file content writes successfully, but the permission reset afterwards fails with `EPERM`. The fix depends on which engine created the files.

Files from earlier **rootful Docker** runs are owned by real root:

```
sudo chown -R "$(id -u):$(id -g)" .
```

Files from earlier **rootless Podman** runs are owned by a mapped subordinate UID, which plain `chown` cannot reach:

```
podman unshare chown -R 0:0 filesystem/
```
