#pragma once

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <optional>


template<typename ServiceT>
class MonitoredServiceClient{

    //node that is using this and the Client that will be used to call the service when needed
    //this is a raw pointer bc only the Node functions are needed so no need to include the full ControllerOverseer
    rclcpp::Node* node;
    typename rclcpp::Client<ServiceT>::SharedPtr client;

    //timer and log of when a service timer started
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Time startTime;

    using Request  = typename ServiceT::Request::SharedPtr;
    using Response = typename ServiceT::Response::SharedPtr;
    using Future = typename rclcpp::Client<ServiceT>::SharedFutureWithRequest;
    using FutureAndId = typename rclcpp::Client<ServiceT>::SharedFutureWithRequestAndRequestId;
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
    std::optional<FutureAndId> activeFuture;

    public:

    /*
        Responsible for adding a service to the queue.
        returns if the service was successfully added.
    */
    bool scheduleCall(Request request, std::function<void (Response)> callback){
        if(waitingRequests.size() < 10){
            servicePair pair(request, callback);
            waitingRequests.push_front(pair);
            return true;
        }

        RCLCPP_ERROR(node->get_logger(), "Reached limit of calls for service %s", client->get_service_name());
        return false;

    }

    /*
        Timer callback function.
        Should check if there is a current client and if that client has timed out.
    */
    void timerCallback(){

        if(waitingForClient){
            if((node->get_clock()->now() - startTime).seconds() >= 3){
                RCLCPP_ERROR(node->get_logger(), "Call to service %s timed out", client->get_service_name());
                client->remove_pending_request(*activeFuture);
                waitingForClient = false;
            }
        } else{
            if(waitingRequests.size() > 0){
                activeClient = waitingRequests.back();
                waitingRequests.pop_back();
                // RCLCPP_INFO(node->get_logger(), "Making call to %s", client->get_service_name());
                client->wait_for_service();
                activeFuture = client->async_send_request(activeClient.request, std::bind(
                                                                                    &MonitoredServiceClient<ServiceT>::serviceCallback,
                                                                                    this,
                                                                                    std::placeholders::_1));
                startTime = node->get_clock()->now();
                waitingForClient = true;
                    
                }

        }

}

    /*
        Service callback function.
        Should take the callback from the queue and call it.
    */
    void serviceCallback(Future future){
        if(waitingForClient){
            Response res = future.get().second;
            activeClient.callback(res);
            waitingForClient = false;
        }else{
            RCLCPP_ERROR(node->get_logger(), "MonitoredServiceClient received a callback but doesn't know where it came from");
        } 
    }

    //true if client has not waiting requests or active request
    bool isIdle(){
        return !waitingForClient && waitingRequests.empty();
    }

    //constructor go brrrr
    MonitoredServiceClient(rclcpp::Node *node, std::string serviceName) : node(node), waitingForClient(false){
        client = node->create_client<ServiceT>(serviceName);
        timer = node->create_wall_timer(std::chrono::milliseconds(500), std::bind(&MonitoredServiceClient<ServiceT>::timerCallback, this));

    }
};
