#pragma once
#ifndef MCKERNELTHREADMANAGER
#define MCKERNELTHREADMANAGER
#ifdef XENOMAI
#include<alchemy/task.h>
#include<alchemy/sem.h>
#include<alchemy/mutex.h>
#else
#include<thread>
#include<semaphore.h>
#include<mutex>
#endif
#include<string>

class m_Thread
{
    #ifdef XENOMAI
    typedef RT_TASK ThreadHandle;
    #else
    typedef  std::thread ThreadHandle ;
    #endif
public:
    m_Thread(void (*Task)(void * ),int Priority,void * Context,const std::string ThreadName);
    ~m_Thread();
    int  Start();
    int  join();
private:
    ThreadHandle        *m_TaskHandle;
    void (*m_Task)(void * );
    int                 m_Priority;
    std::string         m_ThreadName;
    void *               m_Context;
};


class  COsCriticalSection
{
#ifndef XENOMAI
 	typedef std::mutex COsCriticalSection_Impl;
#else 
	typedef RT_MUTEX  COsCriticalSection_Impl;
#endif
public: 
	COsCriticalSection(std::string MutexName="");
	~COsCriticalSection();
	
	void Enter();	
	void Leave();
	
private:
	COsCriticalSection_Impl* m_pImpl;		
};

class m_Sem
{
#ifdef XENOMAI
typedef RT_SEM  SemHandle;
#else
typedef  sem_t     SemHandle;
#endif

private:
    SemHandle m_SemHandle;
public:
    m_Sem(std::string SemName="");
    ~m_Sem();
    void Post();
    void Wait();

};








#endif
