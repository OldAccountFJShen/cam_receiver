#include <chrono>
#include "cam_receiver/helper.hpp"
#include <iostream>
#include <sstream>
#include <string>


void helper::print_time_stamp(std::string comment){
    std::cout<< comment<<" "
	     << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count()
	     <<"\t";
};


std::string helper::get_time_stamp_str(){
    std::stringstream ss;
    ss << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    return ss.str();
}

namespace helper{
    void counter::updateSeq(unsigned long value){
        ++absolute_count;

        if(first_time){
            current_seq = value;
            first_time = false;
            return;
        }
        else{
            previous_seq = current_seq;
            current_seq = value;
            loss += (current_seq - previous_seq - 1);
            return;
        }
    }

    long int counter::getLoss(){
        return loss;
    }

    unsigned long counter::getCurrentSeq(){
        return current_seq;
    }

    unsigned long counter::getCount() {
        return absolute_count;
    }

}
