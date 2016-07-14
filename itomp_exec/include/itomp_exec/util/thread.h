/*
 * pthread wrapper class for the purpose of convenient deferred cancellation without consideration of memory deallocation
 */

#ifndef ITOMP_EXEC_THREAD_H
#define ITOMP_EXEC_THREAD_H


#include <pthread.h>
#include <map>
#include <vector>


namespace itomp_exec
{

class Thread;

namespace internal
{

class ThreadManager
{
public:

    ThreadManager();
    ~ThreadManager();

    Thread* findByID(pthread_t id);

    void add(pthread_t id, Thread* thread);
    void remove(pthread_t id);

private:

    std::map<pthread_t, Thread*> threads_;
    pthread_mutex_t mutex_;
};

}

class Thread
{
public:

    static Thread* self();

private:

    static internal::ThreadManager thread_manager_;
    static void* start_routine_(void* arg);

    struct routine_arg
    {
        void* (*routine)(void *);
        void* arg;
        Thread* thread;
    };

public:

    Thread(void *(*start_routine)(void *), void* arg);
    ~Thread();

    // called in creator thread
    void cancel();
    void join();

    // called in created thread
    void testCancel();
    void cleanupPush(void (*routine)(void *), void* arg);
    void cleanupPop(int execute);

private:

    // pthread id
    pthread_t thread_;

    routine_arg* r_;

    bool cancel_request_;
    pthread_mutex_t cancel_mutex_;

    std::vector<std::pair<void (*)(void *), void*> > cleanup_routines_;
};

}


#endif // ITOMP_EXEC_THREAD_H
