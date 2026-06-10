#include "ThreadPool.h"
#include <process.h>


ThreadPool::ThreadPool(size_t maxNumofThread, size_t minNumofThread)
{
	if (minNumofThread < 2)
	{
		this->minNumofThread = 2;
	}
	else
	{
		this->minNumofThread = minNumofThread;
	}
	if (maxNumofThread < this->minNumofThread * 2)
	{
		this->maxNumofThread = this->minNumofThread * 2;
	}
	else
	{
		this->maxNumofThread = maxNumofThread;
	}

	stopEvevt = CreateEvent(NULL, TRUE, FALSE, NULL);
	completionPort = CreateIoCompletionPort(INVALID_HANDLE_VALUE, NULL, 0, 1);
	idelThreadList.clear();
	createIdleTHread(this->minNumofThread);
	busyThreadList.clear();

	dispathchThread=(HANDLE) _beginthreadex(0, 0, GetTaskThreadProc, this, 0, 0);
	numofLongFun = 0;
}



ThreadPool::~ThreadPool()
{
	SetEvent(stopEvevt);
	PostQueuedCompletionStatus(completionPort, 0, (DWORD)EXIT, NULL);

	CloseHandle(stopEvevt);
}

bool ThreadPool::QueueTaskItem(TaskFun task, PVOID param, TaskCallBackFun taskCb, bool longfunc)
{
	waitTaskLock.Lock();
	WaitTask * waittask = new WaitTask(task, param, taskCb, longfunc);
	waitTaskList.push_back(waittask);
	waitTaskLock.unLock();
	PostQueuedCompletionStatus(completionPort, 0, (DWORD)GET_TASK, NULL);
	return true;
}

void ThreadPool::createIdleTHread(size_t size)
{
	idelThreadLock.Lock();
	for (int i = 0; i < size; ++i)
	{
		idelThreadList.push_back(new Thread(this));
	}
	idelThreadLock.unLock();
}

void ThreadPool::deleteIdleThread(size_t size)
{
	idelThreadLock.Lock();
	int t = idelThreadList.size();
	if (t > size)
	{
		for (int i = 0; i < size; ++i)
		{
			auto thread = idelThreadList.back();
			delete thread;
			idelThreadList.pop_back();
		}
	}
	else
	{
		for (int i = 0; i < t; ++i)
		{
			auto thread = idelThreadList.back();
			delete thread;
			idelThreadList.pop_back();
		}
	}
	idelThreadLock.unLock();
}

ThreadPool::Thread * ThreadPool::GetIdleThread()
{
	Thread * thread = NULL;
	idelThreadLock.Lock();
	if (idelThreadList.size() > 0)
	{
		thread = idelThreadList.front();
		idelThreadList.pop_front();
	}
	idelThreadLock.unLock();

	if (thread == NULL && GetCurNumofThread() < maxNumofThread)
	{
		thread = new Thread(this);
	}

	if (thread == NULL && waitTaskList.size() > 20 )
	{
		thread = new Thread(this);
		InterlockedIncrement(&maxNumofThread);
	}

	return thread;
}

void ThreadPool::MoveBusyThreadToIdleList(Thread * thread)
{
	idelThreadLock.Lock();
	idelThreadList.push_back(thread);
	idelThreadLock.unLock();

	busyThreadLock.Lock();
	for (auto i = busyThreadList.begin(); i != busyThreadList.end(); ++i)
	{
		if (*i = thread)
		{
			busyThreadList.erase(i);
			break;
		}
	}
	busyThreadLock.unLock();


	if (maxNumofThread > 0 && idelThreadList.size() > maxNumofThread*0.8)
	{
		deleteIdleThread(idelThreadList.size() / 2);
	}

	PostQueuedCompletionStatus(completionPort, 0, (DWORD)GET_TASK, NULL);
}

void ThreadPool::MoveThreadToBusyList(Thread * thread)
{
	busyThreadLock.Lock();
	busyThreadList.push_back(thread);
	busyThreadLock.unLock();
}

void ThreadPool::getTaskExecute()
{
	Thread *thread = NULL;
	WaitTask * waittask = NULL;

	waittask = GetTask();
	if (waittask == NULL)
	{
		return;
	}

	if (waittask->plong)
	{
		if (idelThreadList.size() > minNumofThread)
		{
			thread = GetIdleThread();
		}
		else
		{
			thread = new Thread(this);
			InterlockedIncrement(&numofLongFun);
			InterlockedIncrement(&maxNumofThread);
		}
	}
	else
	{
		thread = GetIdleThread();
	}
	if (thread != NULL)
	{
		thread->executeTask(waittask->task, waittask->param, waittask->taskCb);
		delete waittask;
		MoveThreadToBusyList(thread);
	}
	else
	{
		waitTaskLock.Lock();
		waitTaskList.push_back(waittask);
		waitTaskLock.unLock();
	}
}

ThreadPool::WaitTask * ThreadPool::GetTask()
{
	WaitTask *waittask = NULL;
	waitTaskLock.Lock();
	if (waitTaskList.size() > 0)
	{
		waittask = waitTaskList.front();
		waitTaskList.pop_front();
	}
	waitTaskLock.unLock();
	return waittask;
}

ThreadPool::Thread::Thread(ThreadPool * threadPol):
	busy(false),
	thread(INVALID_HANDLE_VALUE),
	task(NULL),
	taskCb(NULL),
	exit(FALSE),
	threadPool(threadPol)
{
	thread = (HANDLE)_beginthreadex(0, 0, ThreadProc, this, CREATE_SUSPENDED, 0);
}

ThreadPool::Thread::~Thread()
{
	exit = true;
	task = NULL;
	taskCb = NULL;
	ResumeThread(thread);
	WaitForSingleObject(thread, INFINITE);
	CloseHandle(thread);
}

bool ThreadPool::Thread::isBussy()
{
	return busy;
}

void ThreadPool::Thread::executeTask(TaskFun task, PVOID param, TaskCallBackFun taskCb)
{
	busy = true;
	this->task = task;
	this->param = param;
	this->taskCb = taskCb;
	ResumeThread(thread);
}

unsigned int ThreadPool::Thread::ThreadProc(PVOID pM)
{
	Thread * pthread = (Thread *)pM;

	while (true)
	{
		if (pthread->exit)
			break;
		if (pthread->task == NULL && pthread->taskCb == NULL)
		{
			pthread->busy = false;
			pthread->threadPool->MoveBusyThreadToIdleList(pthread);
			SuspendThread(pthread);
			continue;
		}
		int result = pthread->task(pthread->param);
		if (pthread->taskCb)
			pthread->taskCb(result);
		WaitTask * waittask = pthread->threadPool->GetTask();
		if (waittask != NULL)
		{
			pthread->task = waittask->task;
			pthread->taskCb = waittask->taskCb;
			delete waittask;
			continue;
		}
		else
		{
			pthread->task = NULL;
			pthread->taskCb = NULL;
			pthread->busy = false;
			pthread->busy = false;
			pthread->threadPool->MoveBusyThreadToIdleList(pthread);
			SuspendThread(pthread);
		}
	}
	return 0;
}