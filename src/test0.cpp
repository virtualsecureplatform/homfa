#include "error.hpp"
#include "graph.hpp"

#include <cassert>
#include <iostream>
#include <queue>
#include <random>
#include <sstream>

#include <spdlog/spdlog.h>
#include <spot/misc/bddlt.hh>
#include <spot/misc/minato.hh>
#include <spot/tl/parse.hh>
#include <spot/twaalgos/hoa.hh>
#include <spot/twaalgos/translate.hh>

std::string expected_dump(const std::vector<std::string>& s)
{
    std::stringstream ss;
    for (size_t i = 0; i < s.size(); i += 3)
        ss << s.at(i) << "\t" << s.at(i + 1) << "\t" << s.at(i + 2) << "\n";
    return ss.str();
}

void test_graph_dump()
{
    {
        Graph gr = Graph::from_file("test/02.spec");
        std::stringstream ss;
        gr.dump(ss);
        assert(ss.str() == expected_dump({
                               ">0", "1", "0",  //
                               "1*", "2", "1",  //
                               "2*", "0", "1",  //
                           }));
    }
    {
        Graph gr = Graph::from_file("test/03.spec");
        std::stringstream ss;
        gr.dump(ss);
        assert(ss.str() == expected_dump({
                               "0*", "1", "0",   //
                               ">1*", "0", "1",  //
                           }));
    }
    {
        Graph gr = Graph::from_file("test/04.spec");
        std::stringstream ss;
        gr.dump(ss);
        assert(ss.str() == expected_dump({
                               ">0", "1", "2",  //
                               "1", "0", "3",   //
                               "2*", "4", "5",  //
                               "3*", "4", "5",  //
                               "4*", "4", "5",  //
                               "5", "5", "5",   //
                           }));
    }
    {
        Graph gr = Graph::from_file("test/06.spec");
        std::stringstream ss;
        gr.dump(ss);
        assert(ss.str() == expected_dump({
                               ">0", "1", "0",  //
                               "1*", "2", "3",  //
                               "2*", "0", "4",  //
                               "3*", "3", "3",  //
                               "4*", "5", "4",  //
                               "5",  "6", "7",  //
                               "6",  "4", "0",  //
                               "7",  "7", "7",  //
                           }));
    }
    {
        Graph gr = Graph::from_file("test/07.spec");
        std::stringstream ss;
        gr.dump(ss);
        assert(ss.str() == expected_dump({
                               ">0*", "1", "2",  //
                               "1",   "3", "4",  //
                               "2",   "3", "5",  //
                               "3",   "3", "3",  //
                               "4",   "6", "7",  //
                               "5*",  "8", "8",  //
                               "6",   "4", "3",  //
                               "7",   "5", "3",  //
                               "8",   "5", "5",  //
                           }));
    }
    {
        Graph gr = Graph::from_file("test/08.spec");
        std::stringstream ss;
        gr.dump(ss);
        assert(ss.str() == expected_dump({
                               ">0*", "1", "2",  //
                               "1*",  "0", "0",  //
                               "2*",  "3", "3",  //
                               "3*",  "4", "5",  //
                               "4*",  "0", "6",  //
                               "5*",  "3", "6",  //
                               "6",   "6", "6",  //
                           }));
    }
    {
        Graph gr = Graph::from_file("test/09.spec");
        std::stringstream ss;
        gr.dump(ss);
        assert(ss.str() == expected_dump({
                               ">0*", "1", "2",  //
                               "1",   "0", "0",  //
                               "2",   "3", "3",  //
                               "3*",  "4", "5",  //
                               "4",   "0", "6",  //
                               "5",   "3", "6",  //
                               "6",   "6", "6",  //
                           }));
    }
}

void test_graph_reversed()
{
    {
        Graph gr = Graph::from_file("test/01.spec");
        Graph rgr = gr.reversed();
        std::stringstream ss;
        rgr.dump(ss);
        assert(ss.str() == expected_dump({
                               ">0*", "1", "2",  //
                               "1",   "3", "4",  //
                               "2",   "3", "5",  //
                               "3",   "3", "3",  //
                               "4",   "6", "7",  //
                               "5*",  "8", "8",  //
                               "6",   "4", "3",  //
                               "7",   "5", "3",  //
                               "8",   "5", "5",  //
                           }));
    }
    {
        Graph gr = Graph::from_file("test/02.spec");
        Graph rgr = gr.reversed();
        std::stringstream ss;
        rgr.dump(ss);
        assert(ss.str() == expected_dump({
                               ">0", "1", "0",  //
                               "1*", "2", "3",  //
                               "2*", "0", "4",  //
                               "3*", "3", "3",  //
                               "4*", "5", "4",  //
                               "5",  "6", "7",  //
                               "6",  "4", "0",  //
                               "7",  "7", "7",  //
                           }));
    }
}

void test_graph_minimized()
{
    {
        Graph gr = Graph::from_file("test/02.spec");
        Graph mgr = gr.minimized();
        std::stringstream ss;
        mgr.dump(ss);
        assert(ss.str() == expected_dump({
                               ">0", "1", "0",  //
                               "1*", "2", "1",  //
                               "2*", "0", "1",  //
                           }));
    }

    {
        Graph gr = Graph::from_file("test/03.spec");
        Graph mgr = gr.minimized();
        std::stringstream ss;
        mgr.dump(ss);
        assert(ss.str() == expected_dump({
                               ">0*", "0", "0",  //
                           }));
    }

    {
        Graph gr = Graph::from_file("test/04.spec");
        Graph mgr = gr.minimized();
        std::stringstream ss;
        mgr.dump(ss);
        assert(ss.str() == expected_dump({
                               ">0", "0", "1",  //
                               "1*", "1", "2",  //
                               "2", "2", "2",   //
                           }));
    }

    {
        Graph gr = Graph::from_file("test/05.spec");
        Graph mgr = gr.minimized();
        std::stringstream ss;
        mgr.dump(ss);
        assert(ss.str() == expected_dump({
                               ">0", "0", "1",  //
                               "1*", "1", "1",  //
                           }));
    }
}

void test_monitor()
{
    {
        Graph gr = Graph::from_ltl_formula("!F(red & X(yellow))", 2);
        std::stringstream ss;
        gr.dump(ss);
        assert(ss.str() == expected_dump({
                               ">0*", "1", "2",  //
                               "1",   "0", "0",  //
                               "2",   "3", "3",  //
                               "3*",  "4", "5",  //
                               "4",   "0", "6",  //
                               "5",   "3", "6",  //
                               "6",   "6", "6",  //
                           }));
    }
    {
        Graph gr = Graph::from_ltl_formula("G(press -> red W green)", 3);
        std::stringstream ss;
        gr.dump(ss);
        assert(ss.str() == expected_dump({
                               ">0*", "1", "2",   //
                               "1",   "3", "3",   //
                               "2",   "4", "5",   //
                               "3",   "0", "0",   //
                               "4",   "6", "0",   //
                               "5",   "7", "0",   //
                               "6",   "6", "6",   //
                               "7*",  "8", "8",   //
                               "8",   "9", "10",  //
                               "9",   "6", "0",   //
                               "10",  "7", "0",   //
                           }));
    }
}

void test_negated()
{
    {
        Graph gr = Graph::from_file("test/02.spec").negated();
        std::stringstream ss;
        gr.dump(ss);
        assert(ss.str() == expected_dump({
                               ">0*", "1", "0",  //
                               "1", "2", "1",    //
                               "2", "0", "1",    //
                           }));
    }
    {
        Graph gr = Graph::from_file("test/03.spec").negated();
        std::stringstream ss;
        gr.dump(ss);
        assert(ss.str() == expected_dump({
                               "0", "1", "0",   //
                               ">1", "0", "1",  //
                           }));
    }
    {
        Graph gr = Graph::from_file("test/04.spec").negated();
        std::stringstream ss;
        gr.dump(ss);
        assert(ss.str() == expected_dump({
                               ">0*", "1", "2",  //
                               "1*", "0", "3",   //
                               "2", "4", "5",    //
                               "3", "4", "5",    //
                               "4", "4", "5",    //
                               "5*", "5", "5",   //
                           }));
    }
    {
        Graph gr = Graph::from_file("test/06.spec").negated();
        std::stringstream ss;
        gr.dump(ss);
        assert(ss.str() == expected_dump({
                               ">0*", "1", "0",  //
                               "1",   "2", "3",  //
                               "2",   "0", "4",  //
                               "3",   "3", "3",  //
                               "4",   "5", "4",  //
                               "5*",  "6", "7",  //
                               "6*",  "4", "0",  //
                               "7*",  "7", "7",  //
                           }));
    }
    {
        Graph gr = Graph::from_file("test/07.spec").negated();
        std::stringstream ss;
        gr.dump(ss);
        assert(ss.str() == expected_dump({
                               ">0", "1", "2",  //
                               "1*", "3", "4",  //
                               "2*", "3", "5",  //
                               "3*", "3", "3",  //
                               "4*", "6", "7",  //
                               "5",  "8", "8",  //
                               "6*", "4", "3",  //
                               "7*", "5", "3",  //
                               "8*", "5", "5",  //
                           }));
    }
    {
        Graph gr = Graph::from_file("test/08.spec").negated();
        std::stringstream ss;
        gr.dump(ss);
        assert(ss.str() == expected_dump({
                               ">0", "1", "2",  //
                               "1",  "0", "0",  //
                               "2",  "3", "3",  //
                               "3",  "4", "5",  //
                               "4",  "0", "6",  //
                               "5",  "3", "6",  //
                               "6*", "6", "6",  //
                           }));
    }
    {
        Graph gr = Graph::from_file("test/09.spec").negated();
        std::stringstream ss;
        gr.dump(ss);
        assert(ss.str() == expected_dump({
                               ">0", "1", "2",  //
                               "1*", "0", "0",  //
                               "2*", "3", "3",  //
                               "3",  "4", "5",  //
                               "4*", "0", "6",  //
                               "5*", "3", "6",  //
                               "6*", "6", "6",  //
                           }));
    }
}

int main()
{
    test_graph_dump();
    test_graph_reversed();
    test_graph_minimized();
    test_monitor();
    test_negated();
}
