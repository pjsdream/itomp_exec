#include <itomp_exec/util/thread.h>
#include <stdio.h>


namespace itomp_exec
{

internal::ThreadManager Thread::thread_manager_;

internal::ThreadManager::ThreadManager()
{
    pthread_mutex_init(&mutex_, NULL);
}

internal::ThreadManager::~ThreadManager()
{
    pthread_mutex_destroy(&mutex_);
}

Thread* internal::ThreadManager::findByID(pthread_t id)
{
    Thread* thread = 0;

    pthread_mutex_lock(&mutex_);
    std::map<pthread_t, Thread*>::iterator it = threads_.find(id);
    std::map<pthread_t, Thread*>::iterator end = threads_.end();
    if (it != end)
        thread = it->second;
    pthread_mutex_unlock(&mutex_);

    return thread;
}

void internal::ThreadManager::add(pthread_t id, Thread *thread)
{
    pthread_mutex_lock(&mutex_);
    threads_[id] = thread;
    pthread_mutex_unlock(&mutex_);
}

void internal::ThreadManager::remove(pthread_t id)
{
    pthread_mutex_lock(&mutex_);
    threads_.erase(id);
    pthread_mutex_unlock(&mutex_);
}

Thread* Thread::self()
{
    return thread_manager_.findByID(pthread_self());
}

void* Thread::start_routine_(void* arg)
{
    routine_arg* r = (routine_arg*)arg;
    thread_manager_.add(pthread_self(), r->thread);
    r->routine(r->arg);
}

Thread::Thread(void *(*start_routine)(void *), void* arg)
    : cancel_request_(false)
{
    pthread_mutex_init(&cancel_mutex_, NULL);

    r_ = new routine_arg({ start_routine, arg, this });
    const int error_number = pthread_create(&thread_, NULL, start_routine_, r_);

    if (error_number != 0)
    {
        fprintf(stderr, "Error occurred creating a thread\n");

        switch (error_number)
        {
        case EAGAIN:
            fprintf(stderr, "Insufficient resources to create another thread.\n");
            break;

        case EINVAL:
            fprintf(stderr, "Invalid settings in attr.\n");
            break;

        case EPERM:
            fprintf(stderr, "No permission to set the scheduling policy and parameters specified in attr.\n");
            break;
        }
    }
}

Thread::~Thread()
{
    pthread_mutex_destroy(&cancel_mutex_);
    delete r_;
}

void Thread::join()
{
    for (int i=(int)cleanup_routines_.size() - 1; i>=0; i--)
        cleanup_routines_[i].first(cleanup_routines_[i].second);
    cleanup_routines_.clear();

    const int error_number = pthread_join(thread_, NULL);

    if (error_number != 0)
    {
        fprintf(stderr, "Error occurred joining a thread");

        switch (error_number)
        {
        case EDEADLK:
            fprintf(stderr, "A deadlock was detected (e.g., two threads tried to join with each other); or thread %lu specifies the calling thread.", thread_);
            break;

        case EINVAL:
            fprintf(stderr, "thread %lu is not a joinable thread. Or another thread is already waiting to join with this thread.", thread_);
            break;

        case ESRCH:
            fprintf(stderr, "No thread with the ID %lu could be found.", thread_);
            break;
        }
    }

    thread_manager_.remove(thread_);
}

void Thread::cancel()
{
    pthread_mutex_lock(&cancel_mutex_);
    cancel_request_ = true;
    pthread_mutex_unlock(&cancel_mutex_);
}

void Thread::testCancel()
{
    bool cancel_request;

    pthread_mutex_lock(&cancel_mutex_);
    cancel_request = cancel_request_;
    pthread_mutex_unlock(&cancel_mutex_);

    if (cancel_request)
    {
        for (int i=(int)cleanup_routines_.size() - 1; i>=0; i--)
            cleanup_routines_[i].first(cleanup_routines_[i].second);
        cleanup_routines_.clear();

        pthread_exit(NULL);
    }
}

void Thread::cleanupPush(void (*routine)(void *), void* arg)
{
    cleanup_routines_.push_back(std::make_pair(routine, arg));
}

}
