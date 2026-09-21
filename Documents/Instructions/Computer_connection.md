# Connecting a Windows PC to the Linux Machine

This guide explains how to reach a Linux machine from a Windows PC, transfer files, and run Docker on it. 

## Connection details

| Item | Value | Note |
|------|-------|------|
| Target IP | `130.243.66.215` | Automatic (DHCP) |
| Username | `rosdev` | Static |
| Hostname | `rosdev-15` | Static |

All commands marked **PowerShell** are run on the Windows PC. Commands marked **Linux** are run on the Ubuntu machine (after logging in with SSH). No extra software is needed.

---

## 1. Check that the machine is reachable

**PowerShell**

```powershell
ping 130.243.66.215 #Linux machine IP
```
Sends test packets to the Linux machine. Replies mean it is on and reachable.

```powershell
Test-NetConnection 130.243.66.215 -Port 22
```
Checks if port 22 (SSH) accepth connection. `TcpTestSucceeded : True` means SSH is available.

---

## 2. Log in with SSH

**PowerShell**
```powershell
ssh rosdev@130.243.66.215
```
or
```powershell
ssh rosdev@rosdev-15
```
The first time, type `yes` to accept the host key, then enter the password of the `roddev` account, which is `c2master` in our case.

After logging in, the prompt changes to `rosdev@rosdev-15:~$`. Everything you type now runs on the Linux machine.

To leave the session, type:
```bash
exit
```
Or press `Ctrl+D`.

---

## 3. Test the connection

**Create a file on Linux from Windows (PowerShell)**

```powershell
ssh rosdev@130.243.66.215 "echo 'Hello from Windows' > ~/test.txt"
```
Runs one command on the Linux machine without opening a full session. It writes a line of text to `test.txt` in the home folder.

**Check the result (Linux)**

```bash
cat ~/test.txt
```
Prints the file. You should see `Hello from Windows`.

**Clean up (Linux)**

```bash
rm ~/test.txt
```
Deletes the test file.

---

## 4. Use Docker on the Linux machine

Log in with SSH first, then **Linux**:

```bash
docker ps
```
Lists running containers. If you get a "permission denied" error, your user is not in the `docker` group. Someone with sudo access runs the following, and you log in again afterwards:

```bash
sudo usermod -aG docker rosdev
```

Useful Docker commands (**Linux**):

```bash
docker ps -a                              # list all containers, including stopped ones
docker images                             # list downloaded images
docker run -it --rm ubuntu:22.04 bash     # start a temporary container with a shell
docker run -it --rm --network host IMAGE  # container shares the host network (needed for ROS 2 discovery)
docker exec -it CONTAINER_NAME bash       # open a shell inside a running container
docker stop CONTAINER_NAME                # stop a container
```
