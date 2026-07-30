# EV3 Wi-Fi onboarding

This runbook moves the EV3 transport from the trusted mini-USB link to Wi-Fi
without giving up the USB recovery path. It is written for the ordered
Olimex `MOD-WIFI-AR9271(-ANT)` adapter: a 2.4 GHz AR9271 USB device using
Linux's `ath9k_htc` driver.

Wi-Fi removes the short cable. It does **not** make a design that opens a new
SSH and Python process for every robot action responsive. The host therefore
also reuses short, motion-free SSH connections, while the real motor
supervisor continues to use its own explicit foreground session and local
fail-stop rules.

## Safety and network assumptions

- Keep the EV3 mini-USB cable connected until Wi-Fi SSH has passed every
  check below.
- For the first test, use a visible 2.4 GHz SSID with
  WPA2-Personal/AES on a private trusted LAN. Avoid a WPA3-only or
  client-isolated guest network for this old ev3dev image.
- Do not expose SSH with router port forwarding.
- Do not put the Wi-Fi passphrase in this repository, a command-line
  argument, an environment file, or a screenshot.
- Keep physical motion disabled throughout onboarding.
- Do not run a broad `apt upgrade`. If firmware is missing, record the exact
  image, kernel, and package state first and prepare a narrow recovery.

## 1. Pass the credential gate over USB

Do **not** join the home LAN while the ev3dev default `robot` / `maker`
credential may still work. A client on the LAN could otherwise bypass the
host-side robot API and reach EV3 files and sysfs directly.

With Wi-Fi still disconnected:

1. Keep one working USB SSH session open as recovery.
2. Change the `robot` account to a unique password with interactive
   `passwd`. Never paste or store it in this repository.
3. Verify a Mac SSH key works in a second, non-interactive USB session:

   ```sh
   ssh -o BatchMode=yes 'robot@<USB-host>' true
   ```

4. With the recovery session still open, configure SSH for public-key login
   only: `PubkeyAuthentication yes`, `PasswordAuthentication no`,
   `ChallengeResponseAuthentication no`, and `PermitRootLogin no`.
5. Validate before reload with `sudo /usr/sbin/sshd -t`; do not reload an
   invalid configuration. Then inspect the effective policy rather than
   assuming the edited file won:

   ```sh
   sudo /usr/sbin/sshd -T | \
     grep -E '^(pubkeyauthentication|passwordauthentication|challengeresponseauthentication|permitrootlogin) '
   ```

   It must report public-key authentication enabled and password,
   challenge-response, and root login disabled.
6. Reload SSH, then prove a **new** USB session works with keys only:

   ```sh
   ssh -o BatchMode=yes \
     -o PreferredAuthentications=publickey \
     -o PasswordAuthentication=no \
     'robot@<USB-host>' true
   ```

We will perform the configuration edit interactively when the brick is
available, after backing up `/etc/ssh/sshd_config`. The existing recovery
session stays open until the second key-only login passes. A dedicated key
with a forced command remains mandatory before any motion-enabled supervisor
is introduced; Wi-Fi onboarding does not enable motion.

## 2. Boot with both recovery paths available

1. Power the EV3 down.
2. Insert the Wi-Fi dongle in the EV3's USB host port.
3. Leave the mini-USB cable between the EV3 and Mac connected.
4. Boot ev3dev and confirm the existing USB SSH target still works.

The brick can be reached through its USB link-local target:

```sh
ssh 'robot@<USB-link-local-address>%<Mac-interface>'
```

The interface suffix can change when the Mac's ports or network services
change. Discover it again rather than assuming `en9` forever.

## 3. Run the read-only readiness inventory

From the repository root on the Mac:

```sh
PYTHONPATH=src python3 -m robot_agent.ev3_wifi_preflight_cli \
  --ssh-target 'robot@<USB-host>' \
  --pretty
```

The command changes no EV3 network state. It records:

- OS, kernel, hostname, and machine ID;
- attached USB IDs;
- network interfaces, their bound driver modules, addresses, and routes;
- whether `ath9k_htc` is available, loaded, and actually owns a wireless
  interface;
- which firmware names the driver declares, which AR9271 firmware candidates
  exist, and the narrow `firmware-atheros` package state;
- read-only ConnMan technology and service output.

After the adapter binds correctly, `network.ath9k_htc_interfaces` must
contain a wireless interface and `onboarding_ready` must be `true`. A generic
Wi-Fi interface driven by some other module is deliberately insufficient.
Save the JSON as experiment evidence, but do not commit machine-specific
network details without reviewing them.

If the USB device appears but no wireless interface does, stop there. Inspect
the reported driver and firmware fields before installing or upgrading
anything.

`onboarding_ready` describes adapter/driver/ConnMan readiness only. It does
not replace the mandatory credential gate above.

## 4. Join Wi-Fi interactively

Use the trusted USB SSH session:

```sh
ssh 'robot@<USB-host>'
connmanctl
```

Then, at the `connmanctl>` prompt:

```text
enable wifi
scan wifi
services
agent on
connect <the-complete-wifi-service-id>
quit
```

Tab completion can fill the service ID. ConnMan asks for the passphrase
interactively, so it does not become a shell argument. It should remember the
connection for later boots.

Brickman's **Wireless and Networks → Wi-Fi** menu is an alternative, but the
interactive command-line path is easier for long passphrases and gives us
better evidence.

## 5. Bind Wi-Fi to the same physical brick

While still connected over USB, record the trusted identity:

```sh
cat /etc/machine-id
for key in /etc/ssh/ssh_host_*_key.pub; do
  ssh-keygen -lf "$key"
done
ip -4 -o address show
```

On the Mac, connect interactively to the new Wi-Fi address. Compare the host
key fingerprint shown by SSH with the fingerprint obtained through USB
before accepting it:

```sh
ssh 'robot@<Wi-Fi-IP>'
```

Then compare `/etc/machine-id` through both paths. The application uses
`StrictHostKeyChecking=yes`; it will intentionally refuse an unseen or
changed host key instead of silently trusting a different machine.

Do not rely on the generic `ev3dev.local` name once a second EV3 exists.
Each controller will need its own stable DHCP reservation or hostname plus
its expected host key and controller identity.

## 6. Prove the link before removing USB

Over the Wi-Fi target:

1. Run the read-only Wi-Fi preflight again.
2. Run inventory and IR reads only; do not move motors.
3. Run at least ten short requests and record latency and failures.
4. Keep one bounded persistent session open and confirm it detects a forced
   Wi-Fi disconnect.
5. Confirm the Mac's default Internet route still uses normal Mac Wi-Fi, not
   the EV3 USB interface.
6. Reboot the EV3 once and verify ConnMan reconnects automatically.

Only after those checks pass should the mini-USB cable be removed.

## 7. Measure what actually became faster

Record these separately over USB and Wi-Fi:

- first/cold SSH connection;
- warm multiplexed SSH command;
- warm persistent sensor request;
- model inference;
- TTS command acceptance;
- actual audio playback duration.

This prevents a five-second spoken sentence from being reported as
five seconds of control latency. The current motion-free short-command
transport uses OpenSSH `ControlMaster=auto`, a control socket under
`~/.ssh`, and a 60-second idle lifetime. The first command can still be slow;
later short commands reuse its encrypted connection.

The final interactive runtime should keep bounded foreground sensor,
supervisor, and speech channels open concurrently. TTS must not block motor
heartbeat or fresh perception.

The EV3 USB host port is USB 1.1 with a theoretical 12 Mbit/s ceiling. The
adapter's advertised 150 Mbit/s therefore is not an achievable EV3 transfer
rate. The link is still ample for control messages, sensor telemetry, and
compressed speech audio; measure rather than inferring latency from the
dongle packaging.

## Recovery

If Wi-Fi setup fails:

1. Leave or reconnect mini-USB.
2. Use the USB SSH target to rerun the read-only preflight.
3. Inspect `connmanctl technologies` and `connmanctl services`.
4. Power down before removing or reseating the dongle.
5. Do not delete the working USB network profile.

No step in the preflight modifies ConnMan, loads a module, installs a package,
or changes routes, so it has no rollback of its own.

## Primary references

- [Olimex MOD-WIFI-AR9271 product and Linux driver details](https://www.olimex.com/Products/USB-Modules/WiFi/MOD-WIFI-AR9271/)
- [Linux Wireless `ath9k_htc` driver and firmware documentation](https://wireless.docs.kernel.org/en/latest/en/users/drivers/ath9k_htc.html)
- [ev3dev networking and supported adapter guidance](https://www.ev3dev.org/docs/networking/)
- [ev3dev EV3 USB host-port limits](https://www.ev3dev.org/docs/kernel-hackers-notebook/ev3-usb-host-port/)
- [ev3dev interactive ConnMan Wi-Fi setup](https://www.ev3dev.org/docs/tutorials/setting-up-wifi-using-the-command-line/)
- [ev3dev SSH connection reuse](https://www.ev3dev.org/docs/tutorials/reusing-ssh-connections/)
- [Debian Stretch `firmware-atheros` manifest](https://sources.debian.org/src/firmware-nonfree/20161130-5/debian/config/atheros/defines/)
