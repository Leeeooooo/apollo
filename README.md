# GIT操作步骤

1. 在小车或者工控机上直接修改代码
2. 提交修改
```shell
git pull #拉取远程服务器上的更新，有冲突的文件部分需要手动合并
#若有新增文件，则需输入git add，若只是修改文件，可以直接git commit -a
git add <代码文件夹> <或者文件> ... #多个文件夹或文件之间以空格分开,全部添加直接输入git add .
git commit -a #添加注释和版本号
```
3. 上传代码到远程服务器

```shell
git push
```

注：日志，录包等文件不会上传

## Apollo操作

1. 如果配置过环境变量，可以直接输入`dev_start.sh`启动docker，然后输入`dev_into.sh`进入docker
2. 否则输入`bash docker/scripts/dev_start.sh`及`bash docker/scripts/dev_into.sh`
3. 进入docker后，输入`bash scripts/boostrap.sh`启动Dreamview（DV）
4. 在DV右上角选择车型和地图，之后在DV的modules下启动需要的模块
5. 部分参数修改后需要重启DV，输入`bash scripts/boostrap.sh stop`关闭DV再重新启动
6. 部分代码或参数修改后需要重新编译，可以使用`bash apollo.sh build <模块名>`来编译单个模块，感知模块需使用build_gpu这个参数
7. 硬件相关参数修改后最好整车下电重启（待验证）