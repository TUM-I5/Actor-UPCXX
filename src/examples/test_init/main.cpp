#include "PingPongActor.hpp"
#include "StubActor.hpp"
#include "actorlib/Distribution.hpp"
#include "actorlib/MultipleActorAGraph.hpp"
#include "actorlib/SingleActorAGraph.hpp"
#include "actorlib/Utility.hpp"
#include "upcxx/upcxx.hpp"
#include <iostream>
#include <optional>
#include <string>
#include <variant>

static_assert(std::is_base_of<Actor, StubActor>::value);

/*

    Calls various init methods of single and multiple actor graph through initializer lists

*/

int main()
{
    upcxx::init();

    // wait for empty rpc?
    upcxx::future<> hmm = upcxx::rpc(((upcxx::rank_me() + 1) % upcxx::rank_n()), []() { return; });
    upcxx::future<> copy = hmm;

    upcxx::progress();
    hmm.wait();
    copy.then([] { std::cout << "Yes, you can wait on a copy of a future" << std::endl; });

    std::cout << "can wait for empty futures" << std::endl;
    auto f = upcxx::make_future();
    f.wait();
    ;
    std::cout << "can wait for empty futures2" << std::endl;

    std::vector<std::string> names = {"A", "B", "C", "D"};
    std::vector<std::tuple<std::string, std::string, std::string, std::string>> connections = {
        {"A", "OUT", "B", "IN"}, {"B", "OUT", "C", "IN"}, {"C", "OUT", "D", "IN"}, {"D", "OUT", "A", "IN"}};

    std::vector<std::string> names2 = {"A", "B", "C", "D"};
    std::vector<std::tuple<std::string, std::string, std::string, std::string>> connections2 = {
        {"A", "OUT", "B", "IN"}, {"B", "OUT", "C", "IN"}, {"C", "OUT", "D", "IN"}, {"D", "OUT", "A", "IN"}};

    std::optional<std::vector<bool>> empty;
    SingleActorAGraph<StubActor> dag; //= //SingleActorAGraph<StubActor>::create<bool>(std::move(names),
                                      // std::move(connections), std::move(empty));
    MultipleActorAGraph<StubActor, PingPongActor> mag;

    MultipleActorAGraph<StubActor, PingPongActor> deaddag;
    std::vector<size_t> typeindices = {1, 1, 1, 1};
    std::vector<std::monostate> x;
    std::variant<std::monostate> var;
    std::vector<std::tuple<std::variant<std::monostate>>> x2 = {var, var, var, var};
    std::cout << "ch a1" << std::endl;
    dag.initFromList(std::move(names), std::move(connections));
    std::cout << "ch a2" << std::endl;
    mag.initFromList(std::move(names2), std::move(connections2), std::move(typeindices), std::move(x2));
    std::cout << "ch a3" << std::endl;

    auto ff = dag.prettyPrint();
    std::cout << ff << std::endl;

    SingleActorAGraph<PingPongActor>
        dag2; // = //SingleActorAGraph<StubActor>::create<bool>(std::move(names), std::move(connections));
    std::vector<std::string> names3 = {"A", "B", "C", "D"};
    std::vector<std::tuple<std::string, std::string, std::string, std::string>> connections3 = {
        {"A", "OUT", "B", "IN"}, {"B", "OUT", "C", "IN"}, {"C", "OUT", "D", "IN"}, {"D", "OUT", "A", "IN"}};
    std::vector<std::tuple<int, int, int>> args3 = {{1, 1, 1}, {2, 2, 2}, {3, 3, 3}, {4, 4, 4}};
    std::cout << "ch n1" << std::endl;
    dag2.initFromList(std::move(names3), std::move(connections3), std::move(args3));
    std::cout << "ch n2" << std::endl;

    upcxx::finalize();
    return 0;
}
