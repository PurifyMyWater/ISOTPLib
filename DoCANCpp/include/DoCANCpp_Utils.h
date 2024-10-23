#ifndef DOCANCPP_UTILS_H
#define DOCANCPP_UTILS_H

#include "DoCANCpp.h"

template <typename T>
class DoCANCpp_CircularBuffer
{
    public:
    struct BufferElement
    {
        T data;
        bool valid;
    };
    private:
    struct BufferElement* buffer;
    int32_t readPos;
    int32_t writePos;
    int32_t bufferElems;
    bool empty;

    bool advanceReadPos();

    public:
    explicit DoCANCpp_CircularBuffer(int32_t maxElems);
    ~DoCANCpp_CircularBuffer();

    void push(T item);
    std::optional<T> peek();
    void pop();
    void remove(int32_t pos);

    struct Iterator
    {
        using iterator_category = std::forward_iterator_tag;
        using difference_type   = struct BufferElement;
        using raw_type          = struct BufferElement;
        using value_type        = T;
        using pointer           = value_type*;
        using raw_pointer       = raw_type*;
        using reference         = value_type&;

        Iterator(int32_t currentPos, int32_t lastPos, int32_t module, raw_pointer baseAddr, bool isEmpty) : module(module), currentPos(currentPos), lastPos(lastPos), baseAddr(baseAddr)
        {
            m_ptr = isEmpty ? nullptr : baseAddr + (currentPos % module);
            while (m_ptr != nullptr && m_ptr->valid == false)
            {
                ++(*this);
            }
        }

        reference operator*() const
        {
            return m_ptr->data;
        }

        pointer operator->()
        {
            return m_ptr == nullptr ? nullptr : &m_ptr->data;
        }

        // Prefix increment
        Iterator& operator++()
        {
            currentPos = (currentPos + 1) % module;
            if(currentPos == lastPos)
            {
                m_ptr = nullptr;
            }
            else
            {
                m_ptr = baseAddr + currentPos;
                if (m_ptr->valid == false)
                {
                    return ++(*this);
                }
            }
            return *this;
        }

        // Postfix increment
        Iterator operator++(int)
        {
            Iterator tmp = *this;
            ++(*this);
            return tmp;
        }

        friend bool operator== (const Iterator& a, const Iterator& b)
        {
            return a.m_ptr == b.m_ptr;
        };

        friend bool operator!= (const Iterator& a, const Iterator& b)
        {
            return a.m_ptr != b.m_ptr;
        };

        int32_t getCurrentPos()
        {
            return currentPos;
        };

        private:
        raw_pointer m_ptr;
        int32_t module;
        int32_t currentPos;
        int32_t lastPos;
        raw_pointer baseAddr;
    };

    Iterator begin()
    {
        auto it = Iterator(readPos, (writePos)%bufferElems, bufferElems, buffer, empty);
        return it;
    }
    Iterator end()
    {
        return Iterator(readPos, readPos, bufferElems, buffer, true);
    }
};

#endif //DOCANCPP_UTILS_H
