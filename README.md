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
cd ~/libsurvive
make
```

## complie
```bash=1
catkin_make -DCATKIN_WHITELIST_PACKAGES="vive"
```
將libsurvive內找不到路徑的handerfile改成絕對路徑
> `../libs/cnkalman/libs/cnmatrix/include/cnmatrix/cn_matrix.h`
## Calibrate
- 
```bash=1
roslaunch vive vive_calibrate.launch
```

- Run
```bash=1
roslaunch vive vive_localization.launch
```
