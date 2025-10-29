# demo_client_service

This package provides a 'correct' demonstration of service and client calls within ros2. The documentation example stalls a thread until the response is sent back, while fine on a single thread, ros gets angry when this method occurs during a callback. 

This package demonstrates a better method in which the response is bound to a seperate callback. The client will stall till the server is online, afterwards it requests the sum of two ints. Clients are useful for two way communication between nodes, in which one requests information from another. 


Nodes included:
* int_addition_client
* int_addition_server



### Launching the client 

    ros2 run demo_client_service int_addition_client 


### Launching the server

    ros2 run demo_client_service int_addition_server 

