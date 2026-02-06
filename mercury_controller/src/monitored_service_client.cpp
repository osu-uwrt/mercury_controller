#include "monitored_service_client.hpp"



template<typename T>
bool MonitoredServiceClient<T>::scheduleCall(Request request, std::function<void (Response)> callback){
        if(waitingRequests.size() < 10){
            servicePair pair(request, callback);
            waitingRequests.push_front(pair);
            return true;
        }

        RCLCPP_ERROR(node->get_logger(), "Reached limit of calls for service %s", client->get_service_name());
        return false;

}

template<typename T>
void MonitoredServiceClient<T>::timerCallback(){

    if(waitingForClient){
        if((node->get_clock()->now() - startTime).seconds() >= 3){
            RCLCPP_ERROR(node->get_logger(), "Call to service %s timed out", client->get_service_name());
            client->remove_pending_request(activeFuture);
            waitingForClient = false;
        }
    } else{
        if(waitingRequests.size() > 0){
            activeClient = waitingRequests.back();
            waitingRequests.pop_back();
            RCLCPP_INFO(node->get_logger(), "Making call to %s", client->get_service_name());
            client->wait_for_service();
            activeFuture = client->async_send_request(activeClient.request, std::bind(
                                                                                &MonitoredServiceClient<T>::serviceCallback,
                                                                                this,
                                                                                std::placeholders::_1));
            startTime = node->get_clock()->now();
            waitingForClient = true;
                
            }

        }

}

template<typename T>
void MonitoredServiceClient<T>::serviceCallback(typename rclcpp::Client<T>::SharedFutureWithRequest future){
    if(waitingForClient){
        Response res = future.get().second;
        activeClient.callback(res);
        waitingForClient = false;
    }else{
        RCLCPP_ERROR(node->get_logger(), "MonitoredServiceClient received a callback but doesn't know where it came from");
    } 
}

template<typename T>
MonitoredServiceClient<T>::MonitoredServiceClient(rclcpp::Node::SharedPtr node, std::string serviceName) : node(node), waitingForClient(false){
        client = node->create_client<T>(serviceName);
        timer = node->create_wall_timer(std::chrono::milliseconds(500), std::bind(&MonitoredServiceClient<T>::timerCallback, this));

}