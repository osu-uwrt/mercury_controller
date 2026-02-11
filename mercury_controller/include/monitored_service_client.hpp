#pragma once

#include <rclcpp/rclcpp.hpp>

#include <deque>
#include <functional>
#include <memory>
#include <string>




template<typename ServiceT>
class MonitoredServiceClient{

    //node that is using this and the Client that will be used to call the service when needed
    std::shared_ptr<ControllerOverseer> node;
    typename rclcpp::Client<ServiceT>::SharedPtr client;

    //timer and log of when a service timer started
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Time startTime;

    using Request  = typename ServiceT::Request::SharedPtr;
    using Response = typename ServiceT::Response::SharedPtr;
    using Future   = typename rclcpp::Client<ServiceT>::SharedFutureWithRequest;

    //pair of request and function callback
    struct servicePair{
        Request request;
        std::function<void (Response)> callback;
        servicePair() = default;
        servicePair(Request req, std::function<void (Response)> cb):request(req), callback(cb){};
    };

    //queue of service calls
    std::deque<servicePair> waitingRequests;

    //pair of the client currently working
    bool waitingForClient;
    servicePair activeClient; //only allowed to be used when waitingForClient is true

    //future that stores the upcoming response from an active service
    Future activeFuture;

    public:

    /*
        Responsible for adding a service to the queue.
        returns if the service was successfully added.
    */
    bool scheduleCall(Request request, std::function<void (Response)> callback);

    /*
        Timer callback function.
        Should check if there is a current client and if that client has timed out.
    */
    void timerCallback();

    /*
        Service callback function.
        Should take the callback from the queue and call it.
    */
    void serviceCallback(Future future);

    //constructor go brrrr
    MonitoredServiceClient(std::shared_ptr<ControllerOverseer> node, std::string serviceName);
};
