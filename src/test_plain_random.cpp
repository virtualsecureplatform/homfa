#include "error.hpp"
#include "graph.hpp"

#include <cassert>
#include <iostream>
#include <queue>
#include <random>
#include <sstream>

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/spdlog.h>
#include <spot/misc/bddlt.hh>
#include <spot/misc/minato.hh>
#include <spot/tl/parse.hh>
#include <spot/twaalgos/hoa.hh>
#include <spot/twaalgos/translate.hh>

template <class T>
std::string vec2str(const std::vector<T>& src)
{
    std::stringstream ss;
    for (auto&& b : src) {
        ss << b << ",";
    }
    return ss.str();
}

bool check_if_accept(spot::twa_graph_ptr aut, const Graph& graph,
                     const std::vector<bool>& src)
{
    Graph::State q = graph.initial_state();
    for (bool b : src)
        q = graph.next_state(q, b);
    return graph.is_final_state(q);
}

// aut should be deterministic
bool check_if_accept(spot::twa_graph_ptr aut, const std::vector<bool>& src,
                     size_t input_kind_size)
{
    std::unordered_map<int, unsigned> var2idx;
    {
        spot::bdd_dict_ptr dict = aut->get_dict();
        for (size_t i = 0; i < input_kind_size; i++) {
            std::stringstream ss;
            ss << "p" << i;
            const std::string varname = ss.str();
            auto it = dict->var_map.find(spot::formula::ap(varname));
            if (it != dict->var_map.end())
                var2idx.emplace(it->second, i);
        }
    }

    size_t input_size = src.size() / input_kind_size;
    std::vector<unsigned> qs;
    qs.push_back(aut->get_init_state_number());
    for (size_t i = 0; i < input_size; i++) {
        std::vector<unsigned> next_qs;
        for (unsigned q : qs) {
            for (const auto& t : aut->out(q)) {
                bdd n = t.cond;
                size_t offset = i * input_kind_size;
                while (n != bddtrue && n != bddfalse) {
                    size_t index = offset + var2idx.at(bdd_var(n));
                    if (src.at(index))
                        n = bdd_high(n);
                    else
                        n = bdd_low(n);
                }
                if (n == bddtrue) {
                    next_qs.push_back(t.dst);
                    break;
                }
            }
        }
        if (next_qs.empty())
            return false;
        qs = std::move(next_qs);
    }

    for (unsigned q : qs)
        if (aut->state_is_accepting(q))
            return true;
    return false;
}

std::vector<bool> int2bvec(size_t i, size_t multiple)
{
    size_t size = log2(i) + 1;
    while (size % multiple != 0)
        size++;
    std::vector<bool> in;
    for (size_t b = 0; b < size; b++)
        in.push_back((i & (1 << b)) != 0);
    return in;
}

template <class RG>
std::vector<bool> random_bvec(RG& rgen, size_t multiple)
{
    std::uniform_int_distribution<> length_dist(100, 1000);
    std::bernoulli_distribution bool_dist;

    size_t length = length_dist(rgen);
    length += multiple - length % multiple;
    std::vector<bool> ret(length);
    for (size_t i = 0; i < length; i++)
        ret.at(i) = bool_dist(rgen);

    return ret;
}

std::string bvec2str(const std::vector<bool>& src)
{
    std::stringstream ss;
    for (bool b : src) {
        ss << b << ",";
    }
    return ss.str();
}

std::vector<size_t> create_table_to_permutate_input_bits(
    const spot::twa_graph_ptr& aut, size_t num_ap)
{
    spot::bdd_dict_ptr dict = aut->get_dict();
    std::unordered_map<int, size_t> var2idx;
    for (size_t i = 0; i < num_ap; i++) {
        std::stringstream ss;
        ss << "p" << i;
        auto it = dict->var_map.find(spot::formula::ap(ss.str()));
        if (it != dict->var_map.end())
            var2idx.emplace(it->second, i);
    }

    std::vector<size_t> in_idx;
    bdd all = aut->ap_vars();
    while (all != bddtrue) {
        int v = bdd_var(all);
        all = bdd_high(all);
        auto it = var2idx.find(v);
        if (it != var2idx.end())
            in_idx.push_back(it->second);
    }

    while (in_idx.size() < num_ap)  // FIXME: correct?
        in_idx.push_back(0);
    assert(in_idx.size() == num_ap);

    return in_idx;
}

std::vector<bool> permutate_input_bits(const std::vector<size_t>& tbl,
                                       const std::vector<bool>& src)
{
    std::vector<bool> in;
    size_t num_ap = tbl.size();
    size_t input_size = src.size() / num_ap;
    for (size_t i = 0; i < input_size; i++) {
        for (size_t j : tbl) {
            size_t index = i * num_ap + j;
            in.push_back(src.at(index));
        }
    }
    return in;
}

template <class Logger>
void test_from_ltl_formula(std::istream& is, size_t num_ap, size_t num_test,
                           Logger& log1, Logger& log2)
{
    std::mt19937 rgen;
    std::string fml;
    size_t cnt = 0;
    while (std::getline(is, fml)) {
        if (++cnt % 1000 == 0)
            std::cerr << ".";
        log2->info(fml);
        log2->flush();

        rgen.seed();

        Graph gr = Graph::from_ltl_formula(fml, num_ap), mgr = gr.minimized(),
              rgr = gr.reversed(), mrgr = rgr.minimized(),
              rgr2 = Graph::from_ltl_formula_reversed(fml, num_ap),
              mrgr2 = rgr2.minimized();

        spot::parsed_formula pf = spot::parse_infix_psl(fml);
        assert(!pf.format_errors(std::cerr));
        spot::translator trans;
        trans.set_type(spot::postprocessor::Monitor);
        trans.set_pref(spot::postprocessor::Deterministic);
        spot::twa_graph_ptr aut = trans.run(pf.f);

        std::vector<size_t> perm_tbl =
            create_table_to_permutate_input_bits(aut, num_ap);

        log1->info("{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}", fml, gr.size(),
                   mgr.size(), aut->ap().size(), aut->num_states(), rgr.size(),
                   mrgr.size(), rgr2.size(), mrgr2.size());

        for (size_t i = 0; i < num_test; i++) {
            std::vector<bool> in_src = i < 100 ? int2bvec(i + 1, num_ap)
                                               : random_bvec(rgen, num_ap),
                              in = permutate_input_bits(perm_tbl, in_src),
                              rin(in.rbegin(), in.rend());

            bool expected = check_if_accept(aut, in_src, num_ap);

            {
                bool got = check_if_accept(aut, gr, in);
                if (expected != got)
                    error::die("[{}] [{}] [{}] {} != {}", fml, i + 1,
                               bvec2str(in), expected, got);
            }
            {
                bool got = check_if_accept(aut, mgr, in);
                if (expected != got)
                    error::die("[{}] [{}] [{}] {} != {}", fml, i + 1,
                               bvec2str(in), expected, got);
            }
            {
                bool got = check_if_accept(aut, rgr, rin);
                if (expected != got)
                    error::die("[{}] [{}] [{}] {} != {}", fml, i + 1,
                               bvec2str(rin), expected, got);
            }
            {
                bool got = check_if_accept(aut, mrgr, rin);
                if (expected != got)
                    error::die("[{}] [{}] [{}] {} != {}", fml, i + 1,
                               bvec2str(rin), expected, got);
            }
            {
                bool got = check_if_accept(aut, rgr2, rin);
                if (expected != got)
                    error::die("[{}] [{}] [{}] {} != {}", fml, i + 1,
                               bvec2str(rin), expected, got);
            }
            {
                bool got = check_if_accept(aut, mrgr2, rin);
                if (expected != got)
                    error::die("[{}] [{}] [{}] {} != {}", fml, i + 1,
                               bvec2str(rin), expected, got);
            }
        }
    }
}

int main(int argc, char** argv)
{
    if (argc != 3) {
        spdlog::error("Usage: {} LOG-FILE1 LOG-FILE2", argv[0]);
        return 1;
    }

    auto max_size = 1024 * 1024 * 1;
    auto logger1 = spdlog::basic_logger_mt("random log1", argv[1]);
    auto logger2 =
        spdlog::rotating_logger_mt("random log2", argv[2], max_size, 1);
    logger1->set_pattern("%v");
    logger2->set_pattern("%v");

    test_from_ltl_formula(std::cin, 5, 300, logger1, logger2);
    return 0;
}
