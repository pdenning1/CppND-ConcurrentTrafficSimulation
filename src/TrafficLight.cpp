#include <iostream>
#include "TrafficLight.h"

/* Implementation of class "MessageQueue" */

// random number generation for cycle duration
std::random_device g_dev;
std::mt19937 rng(g_dev());
std::uniform_int_distribution<std::mt19937::result_type> g_dist6(4,6); // distribution in range [1, 6]

template <typename T>
T MessageQueue<T>::receive()
{
    // FP.5a : The method receive should use std::unique_lock<std::mutex> and _condition.wait() 
    // to wait for and receive new messages and pull them from the queue using move semantics. 
    // The received object should then be returned by the receive function. 
    
    // perform vector modification under the lock
    std::unique_lock<std::mutex> uLock(_mutex);
    _cond.wait(uLock, [this] { return !_queue.empty(); }); // pass unique lock to condition variable

    // remove last vector element from queue
    T m = std::move(_queue.back());
    _queue.pop_back();

    return m;
}

template <typename T>
void MessageQueue<T>::send(T &&msg)
{
    // FP.4a : The method send should use the mechanisms std::lock_guard<std::mutex> 
    // as well as _condition.notify_one() to add a new message to the queue and afterwards send a notification.

    // perform queue modification under the lock
    std::lock_guard<std::mutex> uLock(_mutex);

    _queue.clear();

    std::cout << "Message will be added to the queue" << std::endl;
    _queue.emplace_back(std::move(msg));
    _cond.notify_one(); // notify client after pushing new message into deque
}


/* Implementation of class "TrafficLight" */


TrafficLight::TrafficLight()
{
    _currentPhase = TrafficLightPhase::red;
}

TrafficLight::~TrafficLight()
{

}

void TrafficLight::waitForGreen()
{
    // FP.5b : add the implementation of the method waitForGreen, in which an infinite while-loop 
    // runs and repeatedly calls the receive function on the message queue. 
    // Once it receives TrafficLightPhase::green, the method returns.
    while (true)
    {
        // popBack wakes up when a new element is available in the queue
        TrafficLightPhase phase = std::move(_msgQueue.receive());
        if(phase == green)
        {
            _mutex.lock();
            std::cout << "Traffic light # "<< _id <<" has turned green "<< std::endl;
            _mutex.unlock();
            return;
        }
    }
}

TrafficLightPhase TrafficLight::getCurrentPhase()
{
    return _currentPhase;
}

void TrafficLight::simulate()
{
    // FP.2b : Finally, the private method „cycleThroughPhases“ should be started in a thread when the public method „simulate“ is called. To do this, use the thread queue in the base class. 
    threads.emplace_back(std::thread(&TrafficLight::cycleThroughPhases, this));
}

// virtual function which is executed in a thread
void TrafficLight::cycleThroughPhases()
{
    // FP.2a : Implement the function with an infinite loop that measures the time between two loop cycles 
    // and toggles the current phase of the traffic light between red and green and sends an update method 
    // to the message queue using move semantics. The cycle duration should be a random value between 4 and 6 seconds. 
    // Also, the while-loop should use std::this_thread::sleep_for to wait 1ms between two cycles. 

    std::chrono::time_point<std::chrono::system_clock> lastUpdate;
    // init stop watch
    lastUpdate = std::chrono::system_clock::now();

    double cycleDuration = (g_dist6(rng) * 1000);

    while(true)
    {
        // sleep for 1ms between cycles
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        long timeSinceLastUpdate = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - lastUpdate).count();
        if (timeSinceLastUpdate >= cycleDuration)
        {
            cycleDuration = (g_dist6(rng) * 1000); // generate a new random cycle duration
            lastUpdate = std::chrono::system_clock::now();

            if(_currentPhase == red)
            {
                _currentPhase = green;                
            }
            else
            {
                 _currentPhase = red;
            }
            _msgQueue.send(std::move(_currentPhase));
        }


    }
}

