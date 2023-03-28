#!/bin/bash

rm -rf temp.yml

cp occShell.yml temp.yml

while IFS= read -r line; do
  # 给每一行添加缩进
  indented_line="  $line"
  # 写入到B文件底部
  echo "$indented_line" >> temp.yml
done < additionCpp.cpp

sudo docker run   --rm   -it   -v "$(pwd):/src"   -u "$(id -u):$(id -g)"   donalffons/opencascade.js  /src/temp.yml