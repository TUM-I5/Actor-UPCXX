

#include <memory>
#include <upcxx/upcxx.hpp>
#include <vector>

template <class Reader, class TypeToRead>
std::unique_ptr<upcxx::deserialized_type_t<TypeToRead>> read_unique(Reader &reader)
{
    auto storage = new typename std::aligned_storage<sizeof(upcxx::deserialized_type_t<TypeToRead>),
                                                     alignof(upcxx::deserialized_type_t<TypeToRead>)>::type;

    reader.template read_into<TypeToRead>(storage);
    upcxx::deserialized_type_t<TypeToRead> *ad = reinterpret_cast<upcxx::deserialized_type_t<TypeToRead> *>(storage);

    static_assert(sizeof(typename std::aligned_storage<sizeof(upcxx::deserialized_type_t<TypeToRead>),
                                                       alignof(upcxx::deserialized_type_t<TypeToRead>)>::type) ==
                  sizeof(upcxx::deserialized_type_t<TypeToRead>));

    std::unique_ptr<upcxx::deserialized_type_t<TypeToRead>> uptr(ad);
    return uptr;
}

class Capsule
{
  public:
    std::vector<float> v;

    Capsule(std::vector<float> &&v) : v(std::move(v)) {}

    struct upcxx_serialization
    {
        template <typename Writer> static void serialize(Writer &writer, Capsule const &object)
        {
            writer.write(object.v);
        }

        template <typename Reader> static Capsule *deserialize(Reader &reader, void *storage)
        {
            std::vector<float> v = reader.template read<std::vector<float>>();

            Capsule *c = ::new (storage) Capsule(std::move(v));
            return c;
        }
    };
};

class CapsuleCapsule
{
  public:
    std::unique_ptr<Capsule> c;

    CapsuleCapsule(std::unique_ptr<Capsule> &&c) : c(std::move(c)) {}

    struct upcxx_serialization
    {
        template <typename Writer> static void serialize(Writer &writer, CapsuleCapsule const &object)
        {
            writer.write(*object.c.get());
        }

        template <typename Reader> static CapsuleCapsule *deserialize(Reader &reader, void *storage)
        {
            std::unique_ptr<Capsule> c = read_unique<Reader, Capsule>(reader);

            CapsuleCapsule *cc = ::new (storage) CapsuleCapsule(std::move(c));
            return cc;
        }
    };
};

int main()
{
    upcxx::init();

    std::vector<float> v(1000000, 1.0);
    Capsule *cptr = new Capsule(std::move(v));
    std::unique_ptr<Capsule> c(cptr);
    CapsuleCapsule cc(std::move(c));

    if (upcxx::rank_me() == 0)
    {
        for (unsigned int i = 0; i < 12; i++)
        {
            upcxx::rpc_ff(
                1,
                [](upcxx::view<CapsuleCapsule> vw)
                {
                    auto storage =
                        new typename std::aligned_storage<sizeof(CapsuleCapsule), alignof(CapsuleCapsule)>::type;
                    CapsuleCapsule *cc = vw.begin().deserialize_into(storage);

                    std::cout << cc->c->v[2000] << std::endl;

                    delete cc;
                },
                upcxx::make_view(std::make_move_iterator(&cc), std::make_move_iterator(&cc + 1)));
        }
    }

    upcxx::barrier();

    upcxx::finalize();

    return 0;
}