#include "data_check_plot.hpp"


int main (int argc, char** argv){

    Plot plot("/home/today/reborn/seu_lidarloc/src/data_check_plot");
    plot.readLidar();
    plot.checkLidar();
    plot.readGNss();
    plot.checkGnss();
}