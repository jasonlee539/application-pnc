#!/bin/bash

# 检查是否传入了参数
if [ $# -ne 1 ]; then
    echo "Usage: $0 <name>"
    exit 1
fi

# 获取参数作为文件名
name=$1

# 打包文件
tar -zcvf ${name}.tar.gz modules/planning/ profiles/default/

echo "打包完成，文件名为：${name}.tar.gz"
