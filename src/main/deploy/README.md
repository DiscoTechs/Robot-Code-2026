# Deploy Folder
Files placed in this directory will be deployed to the RoboRIO into the
'deploy' directory in the home folder. Use the 'Filesystem.getDeployDirectory' wpilib function
to get a proper path relative to the deploy directory.

## REMEMBER TO PRUNE DEPLOY FOLDER CACHE
ssh admin@172.22.11.2
YES fingerprint

Run this command. BEWARE OF RM COMMAND THIS FORCES DELETION BE CAREFUL!!
```
cd /home/lvuser/deploy && rm -rf swerve
```
