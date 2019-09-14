#ifndef CAM_RECV_2__HELPER_
#define CAM_RECV_2__HELPER_

#include <chrono>
#include <iostream>
#include <string>

namespace helper {

    void print_time_stamp(std::string comment = "timestamp");

    std::string get_time_stamp_str();


    class counter {
    public:
        counter() : first_time(true), loss(0), absolute_count(0) {}

        long int getLoss();

        unsigned long getCurrentSeq();

        unsigned long getCount();

        void updateSeq(unsigned long);


    private:
        unsigned long current_seq;
        unsigned long previous_seq;
        unsigned long absolute_count;
        long int loss;
        bool first_time;
    };

}

#endif
