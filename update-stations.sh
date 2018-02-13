for i in serl@station1 serl@station3 serl@station4 station@station5 serl@azeroth
do
    echo $i
    ssh $i << 'ENDSSH'
cd ~/Documents/detection
git checkout master
git pull origin master
source devel/setup.bash
catkin_make
ENDSSH
done
