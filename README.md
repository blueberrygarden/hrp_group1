# hrp_group1

```
$ ssh -XC -J %r@i61jump.itec.uni-karlsruhe.de hrp1@i61pc046.itec.uka.de

$ ssh -p 22 -J %r@i61jump.itec.uni-karlsruhe.de -L 13389:[localhost]:3389 hrp1@i61pc046.itec.uka.de -N

$ rdesktop -k de -b -P -z -x 0x8F -g 1680x1024 localhost:13389

$ cd ./repos/mmmtools/build/bin
$ ./MMMViewer

$ cd ./repos/simox/build/bin
$ ./RobotViewer

$ qtcreator -noload Welcome -noload QmlDesigner -noload QmlProfiler
```
