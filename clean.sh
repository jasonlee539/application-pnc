#!/bin/bash

# 跳转到 log 目录
cd data

cd log

# 删除当前目录下所有包含 INFO 的文件
rm -rf *.log.INFO.*
