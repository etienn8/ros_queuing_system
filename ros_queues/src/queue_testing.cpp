#include <iostream>
#include <deque>
#include "ros_queues/lib_queue/dynamic_virtual_queue.hpp"
#include "ros_queues/lib_queue/dynamic_queue.hpp"

using namespace std;

template<typename TQueueElementType>
void objectToBytesConversion(deque<TQueueElementType>& arriving_queue, deque<ElementWithConvertedSize<TQueueElementType>>& converted_queue)
{
    for(typename deque<TQueueElementType>::iterator it = arriving_queue.begin(); it != arriving_queue.end(); ++it)
    {
        ElementWithConvertedSize<TQueueElementType> convertedElement(*it, sizeof(*it));
        converted_queue.push_back(convertedElement);
    }
}

class Position
{
    public:
        Position(double x, double y, double z):x_(x), y_(y), z_(z) {};
        double x_;
        double y_;
        double z_;
};

int main()
{
    string debug_string;


    VirtualQueue vq0;
    vq0.setSize(3);

    InConVirtualQueue vq1(100);
    cout<<"Inequality constraint virtual queue"<<endl;
    cout<<"Queue size"<<vq1.getSize()<<endl;
    vq1.update(4,3);
    cout<<"New arrival of 4 and departure of 3. New size: "<<vq1.getSize()<<endl;
    vq1.update(4,3);
    cout<<"New arrival of 4 and departure of 3.  New size: "<<vq1.getSize()<<endl;
    vq1.update(0,20);
    cout<<"New arrival of 0 and departure of 20.  New size: "<<vq1.getSize()<<endl<<endl;
    vq1.update(vq0,0);
    cout<<"New arrival of 3 and departure of 0.  New size: "<<vq1.getSize()<<endl<<endl;

    EqConVirtualQueue vq2(100);
    cout<<"Equality constraint virtual queue"<<endl;
    cout<<"Queue size"<<vq2.getSize()<<endl;
    vq2.update(4,3);
    cout<<"New arrival of 4 and departure of 3. New size: "<<vq2.getSize()<<endl;
    vq2.update(4,3);
    cout<<"New arrival of 4 and departure of 3.  New size: "<<vq2.getSize()<<endl;
    vq2.update(0,20);
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
    
    // Real converted data queue

    DynamicConvertedQueue<int> q2(6,objectToBytesConversion<int>);
    cout<<"Dynamic converted queue"<<endl;
    cout<<"Queue size: "<<q2.getSize()<<" and Queue converted size: "<<q2.getConvertedSize()<<endl;
    q2.update(queue_to_insert, 0);
    cout<<"Queue size: "<<q2.getSize()<<" and Queue converted size: "<<q2.getConvertedSize()<<endl;
    q2.update(queue_to_insert, 5);
    cout<<"Queue size: "<<q2.getSize()<<" and Queue converted size: "<<q2.getConvertedSize()<<endl;
    q2.update(empty_queue, 5);
    cout<<"Queue size: "<<q2.getSize()<<" and Queue converted size: "<<q2.getConvertedSize()<<endl;
    q2.update(empty_queue, 30);
    cout<<"Queue size: "<<q2.getSize()<<" and Queue converted size: "<<q2.getConvertedSize()<<endl;
    q2.update(big_queue,0);
    cout<<"Queue size: "<<q2.getSize()<<" and Queue converted size: "<<q2.getConvertedSize()<<endl;


    // Position queue test
    deque<Position> small_pos_queue = {Position(0.1,0.1,0.1), Position(0.2,0.2,0.2), Position(0.4,0.4,0.4), Position(3.0,3.0,3.0)};
    
    cout<<"Addinfg position queue of size: "<< small_pos_queue.size()<<endl;

    deque<Position> big_pos_queue ={Position(1.1,0.1,0.1), Position(1.2,0.2,0.2), Position(1.4,0.4,0.4), Position(4.0,3.0,3.0), 
                            Position(1.1,0.1,0.1), Position(1.2,0.2,0.2), Position(1.4,0.4,0.4), Position(4.0,3.0,3.0), 
                            Position(100.0, 100.0, 100.0)};


    deque<Position> empty_pos_queue;
    DynamicConvertedQueue<Position> q3(6,objectToBytesConversion<Position>);
    cout<<"Dynamic converted queue"<<endl;
    cout<<"Queue size: "<<q3.getSize()<<" and Queue converted size: "<<q3.getConvertedSize()<<endl;
    q3.update(small_pos_queue, 0);
    cout<<"Queue size: "<<q3.getSize()<<" and Queue converted size: "<<q3.getConvertedSize()<<endl;
    q3.update(small_pos_queue, 5);
    cout<<"Queue size: "<<q3.getSize()<<" and Queue converted size: "<<q3.getConvertedSize()<<endl;
    q3.update(empty_pos_queue, 5);
    cout<<"Queue size: "<<q3.getSize()<<" and Queue converted size: "<<q3.getConvertedSize()<<endl;
    q3.update(empty_pos_queue, 30);
    cout<<"Queue size: "<<q3.getSize()<<" and Queue converted size: "<<q3.getConvertedSize()<<endl;
    q3.update(big_pos_queue,0);
    cout<<"Queue size: "<<q3.getSize()<<" and Queue converted size: "<<q3.getConvertedSize()<<endl;
    
    InConVirtualQueue vq20(10);

    VirtualQueue vq_arrival;
    vq_arrival.setSize(5);

    vq20.update(4,3);
    vq20.update(4,3);

    vq20.update(vq_arrival, 2);

    return 0;
}