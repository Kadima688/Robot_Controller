#pragma once
#include <mutex>
#include <condition_variable>
class m_RWMutex
{

private:
    int ReaderNum;
    bool WriteringFlag = false;
    int WriteNum;
    std::mutex m;
    std::condition_variable cv;

public:
    m_RWMutex();
    ~m_RWMutex();
    void LockWrite();
    void UnlockWrite();

    void LockRead();
    void UnlockRead();
};

