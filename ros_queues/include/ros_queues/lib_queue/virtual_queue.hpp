#pragma once

#include <mutex>

class IVirtualQueue
{
    public:
        virtual int  size()=0;
        virtual bool empty()=0;
        virtual void push(const int& nb_element)=0;
        virtual void pop(const int& nb_element)=0;
        virtual void setSize(const int& new_size)=0;
    protected:
        int queue_size_ = 0;
        std::mutex queue_manipulation_mutex_;
};

class VirtualQueue: public IVirtualQueue
{
    public:
        virtual int  size() override;
        virtual bool empty() override;
        virtual void push(const int& nb_element) override;
        virtual void pop(const int& nb_element) override;
        virtual void setSize(const int& new_size) override;
};

class NVirtualQueue: public IVirtualQueue
{
    public:
        virtual int  size() override;
        virtual bool empty() override;
        virtual void push(const int& nb_element) override;
        virtual void pop(const int& nb_element) override;
        virtual void setSize(const int& new_size) override;
};