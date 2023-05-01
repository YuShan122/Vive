# Eurobot-Localization
> Eurobot localization workspace for 2023

## Install
```bash=1
# create a new catkin_workspace, if you need to.
cd ~/catkin_workspace/src
git clone -b vive-only https://github.com/DIT-ROBOTICS/Eurobot-Localization.git --recursive vive

cd vive
chmod 777 vive_install.sh
./vive_install.sh

# sudo apt-get install qtbase5-dev
# sudo apt-get install qtdeclarative5-dev
# sudo apt-get install libarmadillo-dev
```

## complie
```bash=1
cd ~/catkin_workspace
catkin_make -DCATKIN_WHITELIST_PACKAGES="obstacle_detector"
catkin_make -DCATKIN_WHITELIST_PACKAGES="vive"
```
將libsurvive內找不到路徑的handerfile改成絕對路徑（已經被改好了！如果編譯時沒有出現錯誤就不用改）
應該會有三個以上的檔案要改：`linmath.h、cn_matrix.h、cn_matrix.hpp`
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

### rival
- Open `vive_rival.launch`, check and modify args: 
  - `rival1_active` / `rival2_active` 
  - `rival1_tracker` / `rival2_tracker`
  - `side`
  - `lowpass_active_` : to determined the lowpass filter active. 
- Launch.
```bash=1
roslaunch vive vive_rival.launch
```