#pragma once

#include "Utility.hpp"
#include <iostream>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

class ActorTriggers
{
  private:
    std::string name;
    std::vector<std::string> trigger_names;
    std::vector<unsigned int> trigger_counts;

  public:
    friend std::ostream &operator<<(std::ostream &os, const ActorTriggers &n);

    ActorTriggers(const std::string &name, std::vector<std::string> &&callers) : name(name)
    {
        trigger_names.reserve(callers.size());
        trigger_counts.reserve(callers.size());
        for (auto &&c : callers)
        {
            trigger_names.push_back(std::move(c));
            trigger_counts.push_back(1);
        }
    }

    ActorTriggers(std::string &&name, std::vector<std::string> &&callers, std::vector<unsigned int> &&trigs)
        : name(std::move(name)), trigger_names(std::move(callers)), trigger_counts(std::move(trigs))
    {
    }

    ActorTriggers(const std::string &name, std::unordered_map<std::string, unsigned int> &&callers) : name(name)
    {
        trigger_names.reserve(callers.size());
        trigger_counts.reserve(callers.size());
        for (auto &&pr : callers)
        {
            trigger_names.push_back(std::move(pr.first));
            trigger_counts.push_back(std::move(pr.second));
        }
    }

    ~ActorTriggers(){};

    ActorTriggers(const ActorTriggers &a)
        : name(a.name), trigger_names(a.trigger_names), trigger_counts(a.trigger_counts)
    {
    }

    ActorTriggers(ActorTriggers &&a)
        : name(std::move(a.name)), trigger_names(std::move(a.trigger_names)),
          trigger_counts(std::move(a.trigger_counts))
    {
    }

    ActorTriggers &operator=(const ActorTriggers &a)
    {
        name = a.name;
        trigger_names = a.trigger_names;
        trigger_counts = a.trigger_counts;
        return *this;
    }

    ActorTriggers &operator=(ActorTriggers &&a)
    {
        name = std::move(a.name);
        trigger_names = std::move(a.trigger_names);
        trigger_counts = std::move(a.trigger_counts);
        return *this;
    }

    std::unordered_map<std::string, unsigned int> getTriggersAsMap() const
    {
        std::unordered_map<std::string, unsigned int> r;
        for (size_t i = 0; i < trigger_names.size(); i++)
        {
            r.emplace(trigger_names[i], trigger_counts[i]);
        }

        return r;
    }

    void addTrigger(const std::string &name)
    {
        for (size_t i = 0; i < trigger_names.size(); i++)
        {
            if (trigger_names[i] == name)
            {
                trigger_counts[i] += 1;
                return;
            }
        }

        throw std::runtime_error("If calling through a port name, the entry must have been created!");
    }

    void increaseAllByOne()
    {
        for (auto &pr : trigger_counts)
        {
            pr += 1;
        }
    }

    bool canExecute() const
    {
        for (const auto &pr : trigger_counts)
        {
            if (pr == 0)
            {
                return false;
            }
        }
        return true;
    }

    size_t canExecuteNTimes() const
    {
        size_t min = std::numeric_limits<unsigned int>::max();

        for (const auto &pr : trigger_counts)
        {
            if (pr < min)
            {
                min = pr;
            }
        }

        return min;
    }

    bool decreaseTriggers()
    {
        if (canExecute())
        {
            throw std::runtime_error("Call to decrease triggers when it would underflow");
        }

        for (auto &pr : trigger_counts)
        {
            pr -= 1;
        }

        return true;
    }

    void reset()
    {
        for (auto &pr : trigger_counts)
        {
            pr = 0;
        }
    }

    const std::string &getNameRef() const { return name; }

    struct upcxx_serialization
    {
        template <typename Writer> static void serialize(Writer &writer, ActorTriggers const &object)
        {
            writer.write(object.name);
            writer.write(object.trigger_names);
            writer.write(object.trigger_counts);
            return;
        }

        template <typename Reader> static ActorTriggers *deserialize(Reader &reader, void *storage)
        {
            std::string name = reader.template read<std::string>();
            std::vector<std::string> trg = reader.template read<std::vector<std::string>>();
            std::vector<unsigned int> trg2 = reader.template read<std::vector<unsigned int>>();
            ActorTriggers *att = ::new (storage) ActorTriggers(std::move(name), std::move(trg), std::move(trg2));

            return att;
        }
    };
};
