#ifndef RESOURCEMANAGER_H
#define RESOURCEMANAGER_H

#include <functional>
#include <memory>
#include <future>
#include <chrono>
#include <map>
#include <assert.h>
//#include "XPLMUtilities.h"

template <typename T>
class ResourceManager
{
public:
    using ResourceHandle = std::shared_ptr<T>;
    using Future = std::future<ResourceHandle>;

    using ResourceCache = std::map<std::string, std::weak_ptr<T>>;
    using FutureCache = std::map<std::string, Future>;

    class TransientState
    {
        friend ResourceManager;
        typename ResourceCache::iterator m_resourceIt;
        typename FutureCache::iterator m_futureIt;
    };

    ResourceManager(std::function<Future(std::string)> factory) : m_factory(factory) {}

    std::shared_ptr<T> get(const std::string &name, TransientState *state)
    {
        auto &resourceIt = state->m_resourceIt;
        auto &futureIt = state->m_futureIt;

        std::shared_ptr<T> resource;
        if (isSingular(resourceIt) || resourceIt == m_resourceCache.end())
            resourceIt = m_resourceCache.find(name);
        if (resourceIt != m_resourceCache.end())
        {
            resource = resourceIt->second.lock();
        }
        if (resource) { return resource; }

        if (isSingular(futureIt) || futureIt == m_futureCache.end())
            futureIt = m_futureCache.find(name);
        if (futureIt == m_futureCache.end())
        {
            futureIt = m_futureCache.emplace(name, m_factory(name)).first;
        }
        auto &future = futureIt->second;

        assert(future.valid());
        if (future.wait_for(std::chrono::duration<int>::zero()) != std::future_status::ready) // not yet finished loading
        {
            return nullptr;
        }
        resource = future.get();
        m_futureCache.erase(name);
        m_resourceCache[name] = resource;
        *state = {};
        return resource;
    }

private:
    std::function<Future(std::string)> m_factory;
    ResourceCache m_resourceCache;
    FutureCache m_futureCache;

    template <typename Iterator>
    static bool isSingular(Iterator i) { return i == Iterator(); }
};

#endif
