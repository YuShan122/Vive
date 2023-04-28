if [ $(rospack find obstacle_detector) ]; then
    echo "Has obstacle detector"
else 
    echo "Missing package : obstacle_detector"
    echo "Directly insall obstacle_detector from github"
    git clone https://github.com/tysik/obstacle_detector.git src/obstacle_detector
fi
