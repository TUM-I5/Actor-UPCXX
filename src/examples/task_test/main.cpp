#include <iostream>
#include <upcxx/upcxx.hpp>

using namespace upcxx;

struct MyStruct
{
  public:
    int x;
    persona myPersona;
    std::string name;

    MyStruct(int x, std::string name) : x(x), name(name) { std::cout << "created myStruct" << std::endl; }
};

int main(int argc, const char **argv)
{
    init();
    std::list<MyStruct> memory;
    memory.emplace_back(1, std::string("MS1"));
    memory.emplace_back(2, std::string("MS2"));
    memory.emplace_back(3, std::string("MS2"));
    memory.emplace_back(4, std::string("MS4"));
    memory.emplace_back(5, std::string("MS5"));
    auto atcsPtr = std::make_unique<MyStruct *[]>(5);
    MyStruct **atcs = atcsPtr.get();
    {
        int i = 0;
        for (auto &elem : memory)
        {
            atcs[i++] = &elem;
        }
    }
#pragma omp parallel
    {
#pragma omp master
        {
            while (true)
            {
                progress();
                for (size_t i = 0; i < 5; i++)
                {
#pragma omp task depend(inout : atcs[i]) default(none) firstprivate(i) mergeable shared(atcs, std::cout)
                    {
                        MyStruct *s = atcs[i];
                        auto scope = persona_scope(s->myPersona);
                        progress();
                        std::cout << "Struct persona " << s->name << " made progress" << std::endl;
                    }
                }
            }
        }
    }

    finalize();
}
