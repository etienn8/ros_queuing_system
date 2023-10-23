#pragma once

/**
 * @brief Interface class that defines the primitive of a virtual queue that is similar to a deque but it contains not element and only the size of the queue is used.
 */
class IVirtualQueue
{
    public:
        /**
         * @brief Get size of the virtual queue.
         * @return Size of the virtual queue.
         */
        virtual int size()=0;

        /**
         * @brief Checks if the queue has zero elements (empty).
         * @return Wether or not the queue is empty.
         */
        virtual bool empty()=0;

        /**
         * @brief Add a specified number of elements in the queue. It increase the queue by this size.
         * @param nb_element Number of elements to add to the queue.
         */
        virtual void push(const int& nb_element)=0;

        /**
         * @brief Remove a specified number of elements in the queue. It decrease the queue by this size.
         * @param nb_element Number of elements to remove from the queue.
         */
        virtual void pop(const int& nb_element)=0;

        /**
         * @brief Set the size of the internal queue.
         * @param new_size Set the size of the queue to this value.
         */
        virtual void setSize(const int& new_size)=0;

    protected:
        /** 
         * @brief Size of the virtual queue. 
        */
        int queue_size_ = 0;
};

/**
 * @brief Virtual queue class that contains a non-negative number of elements.
 */
class VirtualQueue: public IVirtualQueue
{
    public:
        /**
         * @brief Get size of the virtual queue.
         * @return Size of the virtual queue.
         */
        virtual int  size() override;

        /**
         * @brief Checks if the queue has zero elements (empty).
         * @return Wether or not the queue is empty.
         */
        virtual bool empty() override;

        /**
         * @brief Add a specified number of elements in the queue. It increase the queue by this size.
         * @param nb_element Number of elements to add to the queue.
         */
        virtual void push(const int& nb_element) override;

        /**
         * @brief Remove a specified number of elements in the queue without going below a size of zero. It decrease the queue by this size.
         * @param nb_element Number of elements to remove from the queue.
         */
        virtual void pop(const int& nb_element) override;

        /**
         * @brief Set the size of the internal queue.
         * @param new_size Set the size of the queue to this value.
         * @throw Throws an invalid_argument exception if the new_size is a negative value since this type of virtual queue should always have a positive size.
         */
        virtual void setSize(const int& new_size) override;
};

/**
 * @brief Virtual queue class that can have a size that can be negative (Negative-Virtual queue -> NVirtualQUeue)
 */
class NVirtualQueue: public IVirtualQueue
{
    public:
        /**
         * @brief Get size of the virtual queue.
         * @return Size of the virtual queue.
         */
        virtual int  size() override;

        /**
         * @brief Checks if the queue has zero elements (empty).
         * @return Wether or not the queue is empty.
         */
        virtual bool empty() override;

        /**
         * @brief Add a specified number of elements in the queue. It increase the queue by this size.
         * @param nb_element Number of elements to add to the queue.
         */
        virtual void push(const int& nb_element) override;

        /**
         * @brief Remove a specified number of elements in the queue. It decrease the queue by this size and it can go below zero.
         * @param nb_element Number of elements to remove from the queue.
         */
        virtual void pop(const int& nb_element) override;

        /**
         * @brief Set the size of the internal queue.
         * @param new_size Set the size of the queue to this value.
         */
        virtual void setSize(const int& new_size) override;
};