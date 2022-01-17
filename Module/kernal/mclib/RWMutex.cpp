#include"RWMutex.h"
m_RWMutex::m_RWMutex():ReaderNum(0),WriteringFlag(false),WriteNum(0)
{

}
m_RWMutex::~m_RWMutex()
{
    
}
void m_RWMutex::LockWrite()
{
    std::unique_lock<std::mutex> lock(m);
    WriteNum++;
    while (ReaderNum != 0 || WriteringFlag == true)
    {
        cv.wait(lock);
    }
    WriteringFlag = true;
}
void m_RWMutex::UnlockWrite()
{
    std::unique_lock<std::mutex> lock(m);
    WriteringFlag = false;
    WriteNum--;
    cv.notify_all();
}

void m_RWMutex::LockRead()
{
    std::unique_lock<std::mutex> lock(m);
    while (WriteringFlag == true || WriteNum > 0)
    {
        cv.wait(lock);
    }
    ReaderNum++;
}
void m_RWMutex::UnlockRead()
{

    std::unique_lock<std::mutex> lock(m);
    ReaderNum--;
    if (ReaderNum == 0)
    {
        cv.notify_all();
    }
}