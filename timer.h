#ifndef TIMER_H_INCLUDED
#define TIMER_H_INCLUDED
#include <ctime>

class wtimer{

    public:
            wtimer(){ first = std::clock(); };

            double getTime(){
                second = std::clock();
                difference = (second - first)/(double)CLOCKS_PER_SEC;
                first = std::clock();
                return difference;

            };


    private:
            double difference;
            std::clock_t first;
            std::clock_t second;







};





#endif // TIMER_H_INCLUDED
