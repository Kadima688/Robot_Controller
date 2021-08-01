#include"ThreadManage.h"
#include"os.h"





m_Thread::m_Thread(void (*Task)(void * ),int Priority,void * Context,const std::string ThreadName):
m_Task(Task),m_Priority(Priority),m_ThreadName(ThreadName),m_Context(Context)
{
    #ifdef XENOMAI
    m_TaskHandle=new ThreadHandle;
    #endif
}


int  m_Thread::Start()
{
    int err=-1;
#ifndef XENOMAI
    m_TaskHandle=new std::thread{m_Task,m_Context};
#else
    err = rt_task_create(m_TaskHandle, m_ThreadName.c_str(), 4096, m_Priority, T_JOINABLE);
    if (err)
        return err+1000;
    else
    {
        err = rt_task_start(m_TaskHandle, m_Task, m_Context);
        if (err)
            return err+2000;
    }
    
#endif
    EC_TRACE("{}:Thread Start successfully!",m_ThreadName);
    return err;
}
int  m_Thread::join()
{
    int nErr = -1;
    #ifndef XENOMAI
    m_TaskHandle->join();
    #else

	nErr = rt_task_unblock(m_TaskHandle);
	nErr = rt_task_join(m_TaskHandle);
	nErr = rt_task_delete(m_TaskHandle);
    #endif
    delete m_TaskHandle;
    EC_TRACE("{}:Thread join successfully!",m_ThreadName);
    return nErr;
}
m_Thread::~m_Thread()
{
    if(m_TaskHandle) join();
}



COsCriticalSection::COsCriticalSection(std::string MutexName)
{
	m_pImpl = new COsCriticalSection_Impl;
#ifndef XENOMAI

#else
	rt_mutex_create(m_pImpl,MutexName.c_str());
#endif
}
void COsCriticalSection::Enter()
{
	#ifndef XENOMAI
	    m_pImpl->lock();
	#else
	    rt_mutex_acquire(m_pImpl,TM_INFINITE);
	#endif
}

void COsCriticalSection::Leave()
{
	#ifndef XENOMAI
	    m_pImpl->unlock();
	#else
	    rt_mutex_release(m_pImpl);
	#endif
}
COsCriticalSection::~COsCriticalSection()
{
	#ifndef XENOMAI
	#else
	    rt_mutex_release(m_pImpl);
	    rt_mutex_delete(m_pImpl);
	#endif
    delete m_pImpl;
}




m_Sem::m_Sem(std::string SemName)
{
#ifdef XENOMAI
	if (rt_sem_create(m_SemHandle, SemName.c_str(),0, 0))
	{
		EC_ERROR("Sem initilized failed");
	}
#else
	if (sem_init(&m_SemHandle,0,0))
	{
		EC_ERROR("Sem initilized failed");
	}
#endif 
}
void  m_Sem::Post()
{
#ifdef XENOMAI
	if (rt_sem_v(&m_SemHandle))
	{
		EC_ERROR("Sem post failed");
	}
#else
	if (sem_post(&m_SemHandle))
	{
		EC_ERROR("Sem post failed");
	}
#endif 
}
void m_Sem::Wait()
{
#ifdef XENOMAI
	if (rt_sem_p(&m_SemHandle, TM_INFINITE))
	{
		EC_ERROR("Sem wait failed");
	}
#else
	if (sem_wait(&m_SemHandle))
	{
		EC_ERROR("Sem wait failed");
	}
#endif 
}
m_Sem::~m_Sem()
{
#ifdef XENOMAI
	if (rt_sem_delete(&m_SemHandle))
	{
		EC_ERROR("Sem delete failed");
	}
#else
	if (sem_destroy(&m_SemHandle))
	{
		EC_ERROR("Sem delete failed");
	}
#endif 
}