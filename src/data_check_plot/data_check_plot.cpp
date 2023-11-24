#include "data_check_plot.hpp"
#include "abs_current_path.h"

int main (int argc, char** argv){
    Plot plot(ABS_CURRENT_SOURCE_PATH+"/../datacheck");
//    plot.readLidar();
//    plot.checkLidar();
    plot.readGNss();
    plot.checkGnss();
}