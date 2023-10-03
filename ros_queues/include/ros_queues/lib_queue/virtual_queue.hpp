#pragma once

class IVirtualQueue
{
    public:
        virtual int size() const=0;
        virtual bool empty() const=0;
        virtual void push(const int& nb_element)=0;
        virtual void pop(const int& nb_element)=0;
        virtual void setSize(const int& new_size)=0;
    protected:
        int queue_size_ = 0;
};

class VirtualQueue: public IVirtualQueue
{
    public:
        inline virtual int size() const override {return queue_size_;};
        inline virtual bool empty() const override {return queue_size_==0;};
        inline virtual void push(const int& nb_element) override {queue_size_ += nb_element;};
        virtual void pop(const int& nb_element) override
        {
            if (nb_element > queue_size_)
            {
                queue_size_ = 0;
            }
            else
            {
                queue_size_ -= nb_element;
            }
        }
        inline virtual void setSize(const int& new_size) override {queue_size_ = new_size;}
};

class NVirtualQueue: public IVirtualQueue
{
    public:
        inline virtual int size() const override {return queue_size_;};
        inline virtual bool empty() const override {return queue_size_==0;};
        inline virtual void push(const int& nb_element) override {queue_size_ += nb_element;};
        inline virtual void pop(const int& nb_element) override {queue_size_ -= nb_element;};
        inline virtual void setSize(const int& new_size) override {queue_size_ = new_size;}
};