#pragma once
#include "actorlib/Actor.hpp"
#include "actorlib/ActorData.hpp"
#include "actorlib/InPort.hpp"
#include "actorlib/OutPort.hpp"
#include "actorlib/Utility.hpp"
#include <string>
#include <variant>
#include <vector>

class StubActor : public Actor
{
  private:
    int inpcount;
    int outpcount;
    int sends;
    static int max;
    std::vector<InPort<std::vector<float>, 32> *> ins;   // recons
    std::vector<OutPort<std::vector<float>, 32> *> outs; // recons
    void makePorts(int out, int in);

  public:
    StubActor(std::string name, int outgoingPortCount = 1, int incomingPortCount = 1);
    StubActor(std::string &&name, int stub1, int stub2, int stub3);
    StubActor(const StubActor &other) = delete;
    StubActor(StubActor &&other);
    StubActor(ActorData &&data);
    StubActor(std::string &&name, std::variant<std::monostate> &&emtpyvar);
    StubActor(const std::string &name, const std::variant<std::monostate> &emtpyvar);
    void act() override final;

    struct upcxx_serialization
    {
        template <typename Writer> static void serialize(Writer &writer, StubActor const &object)
        {
            const Actor *ai = dynamic_cast<const Actor *>(&object);
            writer.write(*ai);
            writer.write(object.inpcount);
            writer.write(object.outpcount);
        }

        template <typename Reader> static StubActor *deserialize(Reader &reader, void *storage)
        {
            ActorData *ad = util::read<Reader, Actor>(reader);
            int inpcount = reader.template read<int>();
            int outpcount = reader.template read<int>();

            StubActor *v = ::new (storage) StubActor(std::move(*ad));
            v->inpcount = inpcount;
            v->outpcount = outpcount;
            return v;
        }
    };
};