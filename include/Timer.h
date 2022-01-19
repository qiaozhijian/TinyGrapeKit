//
// Created by qzj on 2022/1/12.
//

#ifndef FILTERFUSION_TIMER_H
#define FILTERFUSION_TIMER_H

#include <ctime>
#include <cstdlib>
#include <chrono>
#include <iostream>

class TicToc {
public:
    TicToc(std::string head = "", bool screen = false) {
        tic();
        head_ = head;
        screen_ = screen;
        meanTime = 0.0;
        maxTime = 0.0;
        sumTime = 0.0;
        cnt = 0;
    }

    void tic() {
        start = std::chrono::system_clock::now();
    }

    double toc() {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        curr_time = elapsed_seconds.count();
        cnt++;
        sumTime = sumTime + curr_time;
        meanTime = sumTime / cnt;
        if(maxTime < curr_time)
            maxTime = curr_time;
        print();
        return curr_time;
    }

    void print() {
        if(!screen_)
            return;
        std::cout << head_ << " " << "cur: " << curr_time << " " << "mean: " << meanTime << " " << "max: " << maxTime <<std::endl;
    }

private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
    double meanTime, maxTime, sumTime, curr_time;
    uint64_t cnt;
    bool screen_;
    std::string head_;
};

#endif //FILTERFUSION_TIMER_H
