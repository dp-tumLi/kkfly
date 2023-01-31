## Simulation instructions

- Press "play" in Unity
- `roslaunch process_manager process_manager.launch`
- `cd fla-gtsc/fla_root/groundsation && ./runfms.sh 127.0.0.1`
- `roslaunch simulation mission.launch`


Link to Unity for Linux:
http://download.unity3d.com/download_unity/linux/unity-editor-5.4.0p1+20160810_amd64.deb

Link to Unity Sim Project:
https://www.dropbox.com/s/5jpb6f77dma9t0z/2017-04-14_Building_77.tar.gz?dl=0

08.01-Yu: catkin_make_isolated 先下载位于cmakelist中的externalpackage的环境地图，地图环境之后还需要更新，目前用的是别人的。位于simulation文件夹下的几个node如果无法roslaunch，先cd到当前目录，然后terminal加chmod +x ：node_name 指令将此node变成excutable文件，在当前目录下使用command：ls -l node_name 获知node的状态，末尾有x的为成功修改为excutable。之后建议用此格式添加更新日记或者注意事项。
