#include <iostream>
#include <deque>
#include "ros_queues/lib_queue/dynamic_virtual_queue.hpp"
#include "ros_queues/lib_queue/dynamic_queue.hpp"

using namespace std;

int main()
{
    string debug_string;

    InConVirtualQueue vq1(100);
    cout<<"Inequality constraint virtual queue"<<endl;
    cout<<"Queue size"<<vq1.getSize()<<endl;
    vq1.udpate(4,3);
    cout<<"New arrival of 4 and departure of 3. New size: "<<vq1.getSize()<<endl;
    vq1.udpate(4,3);
    cout<<"New arrival of 4 and departure of 3.  New size: "<<vq1.getSize()<<endl;
    vq1.udpate(0,20);
    cout<<"New arrival of 0 and departure of 20.  New size: "<<vq1.getSize()<<endl<<endl;
    
    EqConVirtualQueue vq2(100);
    cout<<"Equality constraint virtual queue"<<endl;
    cout<<"Queue size"<<vq2.getSize()<<endl;
    vq2.udpate(4,3);
    cout<<"New arrival of 4 and departure of 3. New size: "<<vq2.getSize()<<endl;
    vq2.udpate(4,3);
    cout<<"New arrival of 4 and departure of 3.  New size: "<<vq2.getSize()<<endl;
    vq2.udpate(0,20);
    cout<<"New arrival of 0 and departure of 20.  New size: "<<vq2.getSize()<<endl<<endl;

    // Real data queues
    deque<int> queue_to_insert = {1,2,4,30};
    cout<<"Inserting queue created with size"<< queue_to_insert.size()<<endl;

    deque<int> big_queue ={1,2,4,30,30,30,30};

    deque<int> empty_queue;

    DynamicQueue<int> q1(6);
    cout<<"Dynamic queue"<<endl;
    cout<<"Queue size: "<<q1.getSize()<<endl;
    q1.update(queue_to_insert, 0);
    cout<<"Queue size: "<<q1.getSize()<<endl;
    q1.update(queue_to_insert, 5);
    cout<<"Queue size: l"<<q1.getSize()<<endl;
    q1.update(empty_queue, 5);
    cout<<"Queue size: "<<q1.getSize()<<endl;
    q1.update(empty_queue, 30);
    cout<<"Queue size: "<<q1.getSize()<<endl;
    q1.update(big_queue,0);
    cout<<"Queue size: "<<q1.getSize()<<endl;
    return 0;
}