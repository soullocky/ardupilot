#!/bin/bash
# ardupilot根目录执行
# 用于解决切换分支导致的权限问题

# 赋予waf权限
chmod +x waf

# 转换所有源码文件格式
find . -type f -name "*.py" -o -name "*.sh" -o -name "*.cpp" -o -name "*.h" | xargs dos2unix

# 重新设置执行权限
find Tools -name "*.py" -exec chmod +x {} \;
find Tools -name "*.sh" -exec chmod +x {} \;

# 清理编译文件
./waf clean
./waf configure --board CUAV-x7
./waf copter

echo "修复完成，可以重新编译了！"