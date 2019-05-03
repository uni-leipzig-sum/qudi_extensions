# Notes on the implementation of Attocube Qudi modules

## Qudi setup with user accounts

Qudi is setup in such a way, that each user gets a linux user account (with
his/her home directory located under /home). The main qudi code is installed in
/opt/qudi and is a clean clone of the main distribution
(https://github.com/Ulm-IQO/qudi). The code in this directory should never be
changed locally. Only updates from the official repository are allowed here.

The local changes to qudi are located in /opt/qudi_extensions.

Each user can add/modify modules of their own by creating a qudi_extensions
directory in their home directory and by including it in their personal qudi
config file.

Each user has his/her own qudi configuration. The configuration is located in
~/.qudi/default.cfg and by default, it points to the default configuration file
written for the setup. Each user can replace the link by a custom configuration
file

```sh
rm ~/.qudi/default.cfg
touch ~/.qudi/default.cfg
```

Look at the default global configuration file to get a good starting point.

The qudi icon in the application menu uses this configuration file by default.
If you want to run qudi using a different configuration file, in the qudi
manager window, click on Menu->Load configuration. 
