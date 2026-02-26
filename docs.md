# Robot Code Docs

### Elastic Auto Launch

Setting up Elastic auto launch is easy. Just hold `Windows` + `R`, type `C:\Users\Public\Documents\FRC`, and press `ENTER` into the text box to launch explorer to the FRC configuration path. Next, open `FRC DS Data Storage.ini` in notepad and find `DashboardCmdLine` and replace the entire line with the following:

```
DashboardCmdLine = ""C:\\Users\\Public\\wpilib\\2026\\tools\\Elastic.exe""
```

### Prune Swerve Configuration In Deploy Folder

Sometimes, pruning the swerve configuration deploy folder may be necessary. First hold `Windows` + `R`, type `cmd`, and press `ENTER` to open command prompt. Next, SSH into the robot. If you are connected via USB, run `ssh admin@172.22.11.2`. If you are connected through the radio, run `ssh admin@10.10.99.2`. If asked about the fingerprint, type `yes` and press `ENTER`.

Run the following command to remove the swerve configuration from the deploy folder.
```
cd /home/lvuser/deploy && rm -rf swerve
```

Next redeploy and the swerve configuration will repopulate on the roborio.

\- Jake McQuade

### Resources
- Swerve Config Generator https://yet-another-software-suite.github.io/YAGSL/config_generator/
