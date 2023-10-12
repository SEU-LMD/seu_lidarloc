//
// Created by today on 23-9-2.
//

#ifndef SRC_TIMER_H
#define SRC_TIMER_H


#include <ctime>
#include <chrono>
class TicToc
{
public:
    TicToc()
    {
        tic();
    }

    void tic()
    {
        start = std::chrono::steady_clock::now();
    }

    double toc()
    {
        end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }

private:
    std::chrono::time_point<std::chrono::steady_clock> start, end;
};
#endif //SRC_TIMER_H
