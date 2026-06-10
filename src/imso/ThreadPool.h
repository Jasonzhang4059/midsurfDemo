#pragma once
#include <Windows.h>
#include <queue>
#include <list>
#include <memory>

using std::queue;
using std::list;
using std::shared_ptr;

#define THRESHOLE_OF_WAIT_TASK  20;

typedef int(*TaskFun)(PVOID param);
typedef void(*TaskCallBackFun)(int result);

//线程池类
class ThreadPool
{
private:
	//内置的线程类
	class Thread
	{
	public:
		Thread(ThreadPool * theadpool);
		~Thread();

		//是否繁忙
		bool isBussy();
		//执行任务
		void executeTask(TaskFun task, PVOID param, TaskCallBackFun taskCallBack);
	private:
		ThreadPool *threadPool;
		bool busy;
		bool exit;
		HANDLE thread;
		TaskFun task;
		PVOID param;
		TaskCallBackFun taskCb;
		static unsigned int __stdcall ThreadProc(PVOID pM); //线程函数
	};

	//IOCP的通知种类
	enum WAIT_OPERTATION_TYPE
	{
		GET_TASK,
		EXIT
	};

	//待执行的任务类
	class WaitTask
	{
	public:
		WaitTask(TaskFun task, PVOID param, TaskCallBackFun taskCb, bool plong)
		{
			this->task = task;
			this->param = param;
			this->taskCb = taskCb;
			this->plong = plong;
		}
		~WaitTask() { task = NULL; param = NULL; taskCb = NULL; plong = FALSE; }
		TaskFun task;
		PVOID param;
		TaskCallBackFun taskCb;
		bool plong;
	};

	//从任务列表获取任务的线程函数
	static unsigned int __stdcall GetTaskThreadProc(PVOID pM)
	{
		ThreadPool *threadPool = (ThreadPool *)pM;
		bool bRet = FALSE;
		DWORD dwBytes = 0;
		WAIT_OPERTATION_TYPE opType;
		OVERLAPPED *ol;
		while (WAIT_OBJECT_0 != WaitForSingleObject(threadPool->stopEvevt, 0))
		{
			bool bRet = GetQueuedCompletionStatus(threadPool->completionPort, &dwBytes, (PULONG_PTR)&opType, &ol, INFINITE);
			if (EXIT == (DWORD)opType)
			{
				break;
			}
			else if (GET_TASK == (DWORD)opType)
			{
				threadPool->getTaskExecute();
			}
		}
		return 0;
	}

	//线程临界区锁
	class CriticalSectionLock
	{
	public:
		CriticalSectionLock() { InitializeCriticalSection(&cs); }
		~CriticalSectionLock() { DeleteCriticalSection(&cs); }
		void Lock() { EnterCriticalSection(&cs); }
		void unLock() { LeaveCriticalSection(&cs); }
	private:
		CRITICAL_SECTION cs;//临界区
	};

public:
	ThreadPool(size_t maxNumofThread=10,size_t minNumofThread=2);
	~ThreadPool();
	//任务入队
	bool QueueTaskItem(TaskFun task, PVOID param, TaskCallBackFun taskCb = NULL, bool longFun = FALSE);

private:
	//获取线程池当前线程数
	size_t GetCurNumofThread() { return getIdleThreadNum() + getBusyThreadNum(); }
	//获取线程池最大线程数
	size_t GetMaxNumofThread() { return maxNumofThread - numofLongFun; }
	//获取线程池最小线程数
	size_t GetMinNumofThread() { return minNumofThread; }
	//设置线程池中最小线程数
	void setMinNumofThread(size_t size) { minNumofThread = size; }
	//设置线程池中最小线程数
	void setMaxNumofThread(size_t size)//设置线程池中最大线程数
	{
		if (size < numofLongFun)
		{
			maxNumofThread = size + numofLongFun;
		}
		else
		{
			maxNumofThread = size;
		}
	}
	//获取闲置线程的数目
	size_t getIdleThreadNum() { return idelThreadList.size(); }
	//获取繁忙线程的数目
	size_t getBusyThreadNum() { return busyThreadList.size(); }

	void deleteIdleThread(size_t);//删除一个空闲线程
	void createIdleTHread(size_t);//创建一个空闲线程

	Thread * GetIdleThread();//获取一个空闲线程
	void MoveBusyThreadToIdleList(Thread * busyThread);//移动繁忙线程到空闲列表
	void MoveThreadToBusyList(Thread * thread);//移动线程到繁忙列表
	void getTaskExecute();//从任务队列中获取任务并执行
	WaitTask * GetTask();//从任务队列中取任务

	//空闲线程列表锁
	CriticalSectionLock idelThreadLock;
	//空闲线程列表
	list<Thread *> idelThreadList;
	//忙碌线程列表锁
	CriticalSectionLock busyThreadLock;
	//忙碌线程列表
	list<Thread *> busyThreadList;
	//任务线程列所锁
	CriticalSectionLock waitTaskLock;
	//任务线程列表
	list<WaitTask *> waitTaskList;

	//分发任务线程
	HANDLE dispathchThread;
	//通知线程退出的时间
	HANDLE stopEvevt;
	//完成端口
	HANDLE completionPort;
	//线程池中最大的线程数量
	size_t maxNumofThread;
	//线程池中最小的线程数量
	size_t minNumofThread;
	//线程池中现在的线程数量
	size_t numofLongFun;
};

