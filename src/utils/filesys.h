//
// Created by fyy on 23-9-9.
//

#ifndef SEU_LIDARLOC_FILESYS_H
#define SEU_LIDARLOC_FILESYS_H

#include <unistd.h>

#include <string>
//判断文件或者目录是否存在
bool IsFileDirExist(const std::string& path){
    int state = !access(path.c_str(), F_OK);
    if(state==0){
        return false;
    }
    else{
        return true;
    }
}

void CreateDirWithDelete(const std::string& path){
    if(IsFileDirExist(path)){
        std::system(std::string("rm -rf" + path).c_str());
    }
    std::system(std::string("mkdir -p "+path).c_str());
};

void CreateDirWithoutDelete(const std::string& path) {
    if (!IsFileDirExist(path)) {
        std::system(std::string("mkdir -p " + path).c_str());
    }
}
#endif //SEU_LIDARLOC_FILESYS_H
