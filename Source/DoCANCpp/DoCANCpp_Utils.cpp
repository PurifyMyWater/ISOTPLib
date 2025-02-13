#include "DoCANCpp_Utils.h"

template<typename T>
DoCANCpp_CircularBuffer<T>::DoCANCpp_CircularBuffer(int32_t maxElems)
{
    buffer = static_cast<BufferElement*>(operator new(maxElems * sizeof(BufferElement))); // thx to Jerry Coffin (https://stackoverflow.com/questions/63310723/circular-queue-using-stl-queue)
    readPos = 0;
    writePos = 0;
    bufferElems = maxElems;
    empty = true;
}

template<typename T>
DoCANCpp_CircularBuffer<T>::~DoCANCpp_CircularBuffer()
{
    while (!empty)
        pop();

    operator delete(buffer);
}

template<typename T>
void DoCANCpp_CircularBuffer<T>::push(T item)
{
    if (readPos == writePos && !empty)
    {
        advanceReadPos();
    }

    new (&buffer[writePos]) BufferElement{T(item), true}; // Copy constructor the item into the buffer
    empty = false;

    writePos++;
    writePos %= bufferElems;
}

template<typename T>
std::optional<T> DoCANCpp_CircularBuffer<T>::peek()
{
    if (empty)
    {
        return std::nullopt;
    }
    while (buffer[readPos].valid == false)
    {
        advanceReadPos();
    }
    return buffer[readPos].data;
}

template<typename T>
bool DoCANCpp_CircularBuffer<T>::advanceReadPos()
{
    bool res = true;
    if (!empty)
    {
        int32_t tmp = readPos;

        readPos++;
        readPos %= bufferElems;

        if (readPos == writePos)
        {
            empty = true;
        }

        res = (buffer[tmp].valid == true);
        if (res)
        {
            buffer[tmp].data.~T(); // destroy the object:
        }
    }
    return res;
}

template<typename T>
void DoCANCpp_CircularBuffer<T>::pop()
{
    while (!advanceReadPos())
        ;
}

template<typename T>
void DoCANCpp_CircularBuffer<T>::remove(int32_t pos)
{
    if (pos < 0 || pos >= bufferElems)
    {
        return;
    }
    buffer[pos].valid = false;
    buffer[readPos].data.~T(); // destroy the object:
}
