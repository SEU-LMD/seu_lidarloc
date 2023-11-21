//
// Created by slam on 23-10-21.
//

#ifndef SEU_LIDARLOC_UDP_HELPER_H
#define SEU_LIDARLOC_UDP_HELPER_H

#include <string>
#include "Eigen/Core"           //eigen
#include <Eigen/Geometry>       //quaterniond

class Vis_Odometry{
public:
    std::string type;//gn li dr 三选一
    Eigen::Vector3d t;
    Eigen::Quaterniond q;
    
    std::string ToString(){
        return type+" "+std::to_string(t[0])+" "+std::to_string(t[1])+" "+std::to_string(t[2])+
        " "+std::to_string(q.x())+" "+std::to_string(q.y())+" "+std::to_string(q.z())+" "+std::to_string(q.w());
    }

    static void fromString(std::string msg, Vis_Odometry& out_struct){
        const char* str = msg.c_str();

        char type[3];
        double value[7];

        std::sscanf(str,"%s %lf %lf %lf %lf %lf %lf %lf",&type,&value[0],&value[1],&value[2],
                    &value[3],&value[4],&value[5],&value[6]);



        out_struct.type=type;

        out_struct.t[0]=static_cast<double>(value[0]);
        out_struct.t[1]=static_cast<double>(value[1]);
        out_struct.t[2]=static_cast<double>(value[2]);

        out_struct.q.x()=static_cast<double>(value[3]);
        out_struct.q.y()=static_cast<double>(value[4]);
        out_struct.q.z()=static_cast<double>(value[5]);
        out_struct.q.w()=static_cast<double>(value[6]);

//        out_struct.q.normalize();

    }
};









//

#endif //SEU_LIDARLOC_UDP_HELPER_H
