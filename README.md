# Eurobot-Localization
> Eurobot localization workspace for 2023

## Install
```bash=1
git clone -b vive https://github.com/DIT-ROBOTICS/Eurobot-Localization.git --recursive
./install.sh
source ~/.bashrc
sudo apt-get install qtbase5-dev
sudo apt-get install qtdeclarative5-dev
sudo apt-get install libarmadillo-dev
```
## Calibrate
- 
```bash=1
roslaunch vive vive_calibrate.launch
```

- Run
```bash=1
roslaunch vive vive_localization.launch
```
