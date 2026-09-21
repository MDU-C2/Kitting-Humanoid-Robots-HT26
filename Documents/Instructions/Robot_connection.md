# Connecting a Host Computer to the Unitree G1

Instructions for connecting a host computer to the Unitree G1 over Ethernet, gaining internet access on the robot's devloper computer, and transferring and running code on it. 

## Network overview

| Device | IP address | Notes |
|--------|-----------|-------|
| Locomotion computer | `192.168.123.161` | Not accessible |
| Developer computer | `192.168.123.164` | User: `unitree`, Password: `123` |
| 360° Lidar | `192.168.123.20` | |
| Host computer | e.g. `192.168.123.222` | Static IP you set yourself |

---

## 1. Physical Connection 

Connect an Ethernet cable from your host to port **[4]** or **[5]** on the Unitree G1. If you are unsure which port is which, consult the Unitree G1 [developer guide](https://support.unitree.com/home/en/G1_developer).

---

## 2. Test the Link

Run these in a terminal to test the connections:

```bash
ping 192.168.123.161    # Locomotion computer (can't access)
ping 192.168.123.164    # Developer computer (User: unitree, Password: 123)
ping 192.168.123.20     # 360° Lidar
```
If you are connecting a new computer, you may need to configure its IP address. The robot doesn't have a built-in DHCP server, so you will likely need to set a **static IP** (e.g. `192.168.123.222`). Once configured, this should persist automatically. It only breaks if the network profile itself gets deleted or the adapter changes. Read the official Unitree G1 documentation for how to set the IP address.

---

## 3. SSH into the G1

Remote access to the developer computer:

```bash
ssh -X unitree@192.168.123.164
```

`-X` enables X11 forwarding, so graphical programs on the robot can display on your screen.

You may be asked to choose a ROS distribution. Select `1` for **Foxy**.

---

## 4. Internet Access on the Dev Computer (Optional)

This is needed for installs with `apt` or `pip` on the dev computer. You can skip this section if you don't need to install or update anything.

> **This setup does NOT persist.** It resets whenever the dev computer reboots, and it also resets on your host computer if it sleeps or reconnects to Wi-Fi.

**On your computer** (host), run:

```bash
sudo sysctl -w net.ipv4.ip_forward=1
sudo iptables -t nat -A POSTROUTING -o wlp0s20f3 -j MASQUERADE
sudo iptables -A FORWARD -i enx50a03000b0dc -o wlp0s20f3 -j ACCEPT
sudo iptables -A FORWARD -i wlp0s20f3 -o enx50a03000b0dc -m state --state RELATED,ESTABLISHED -j ACCEPT
```

**On the dev computer** (over your SSH session), run:

```bash
sudo ip route add default via 192.168.123.222
echo "nameserver 8.8.8.8" | sudo tee /etc/resolv.conf
```

**Verify the connection** (on the dev computer):

```bash
ping -c 3 8.8.8.8
ping -c 3 pypi.org
```

If no errors come up, the dev computer has internet access.

---

## 5. Transferring Code/Files to the Unitree G1

**Send a file to the robot:**

To run code on the robot, you have to first send a copy of the file to the robot (run on your computer):

```bash
cd ~/<FILE_LOCATION>
scp ~/<FILE_NAME> unitree@192.168.123.164:~/
```

**Edit a file on the robot:**

```bash
nano <FILE_NAME>
```

Save and exit with `Ctrl + O`, `Enter`, then `Ctrl + X`.

**Remove a file on the robot:**

```bash
rm <FILE_NAME>
```

**Install or update packages on the robot** (requires internet access, see section 4):

```bash
sudo apt update
sudo apt install <PACKAGE_NAME>
```

---

## 6. Running Code

Example: running `G1_data_collector.py` to connect to the depth camera on the G1.

```bash
source ~/miniconda3/bin/activate teleimager
sudo systemctl stop teleimager.service   # turn off previous process holding the camera port
python3 G1_data_collector.py
```

- The first line activates the `teleimager` conda environment.
- The second stops the background service that otherwise holds the camera port.
- The third runs the script.