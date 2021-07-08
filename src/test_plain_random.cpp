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

bool check_if_accept(const Graph& graph, const std::vector<bool>& src)
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
    std::set<unsigned> qs;
    qs.insert(aut->get_init_state_number());
    for (size_t i = 0; i < input_size; i++) {
        std::set<unsigned> next_qs;
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
                if (n == bddtrue)
                    next_qs.insert(t.dst);
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
                           Logger& log1)
{
    std::vector<std::vector<bool>> rand_bvec;
    {
        std::mt19937 rgen;
        for (size_t i = 0; i < num_test - 100; i++)
            rand_bvec.push_back(random_bvec(rgen, num_ap));
    }

    std::string fml;
    size_t cnt = 0;
    while (std::getline(is, fml)) {
        if (++cnt % 1000 == 0)
            std::cerr << ".";

        Graph gr = Graph::from_ltl_formula(fml, num_ap), rgr = gr.reversed(),
              rgr2 = Graph::from_ltl_formula_reversed(fml, num_ap);
        std::optional<Graph> mgr, mrgr, mrgr2;
        if (gr.size() < 10000)
            mgr.emplace(gr.minimized());
        if (rgr.size() < 10000)
            mrgr.emplace(rgr.minimized());
        if (rgr2.size() < 10000)
            mrgr2.emplace(rgr2.minimized());

        spot::twa_graph_ptr aut, det_aut;
        {
            spot::parsed_formula pf = spot::parse_infix_psl(fml);
            assert(!pf.format_errors(std::cerr));
            spot::translator trans;
            trans.set_type(spot::postprocessor::Monitor);
            aut = trans.run(pf.f);
            trans.set_pref(spot::postprocessor::Deterministic);
            det_aut = trans.run(pf.f);
        }

        std::vector<size_t> perm_tbl =
            create_table_to_permutate_input_bits(aut, num_ap);

        {
            int gr_size = gr.size(), rgr_size = rgr.size(),
                rgr2_size = rgr2.size(), mgr_size = mgr ? mgr->size() : -1,
                mrgr_size = mrgr ? mrgr->size() : -1,
                mrgr2_size = mrgr2 ? mrgr2->size() : -1,
                mon_size = aut->num_states(), mon_num_ap = aut->ap().size(),
                mon_prop_det = aut->prop_universal().val(),
                det_mon_size = det_aut->num_states(),
                det_mon_num_ap = det_aut->ap().size(),
                det_mon_prop_det = det_aut->prop_universal().val();
            log1->info("{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}",
                       fml,               // 1
                       gr_size,           // 2
                       mgr_size,          // 3
                       mon_size,          // 4
                       mon_num_ap,        // 5
                       mon_prop_det,      // 6
                       det_mon_size,      // 7
                       det_mon_num_ap,    // 8
                       det_mon_prop_det,  // 9
                       rgr_size,          // 10
                       mrgr_size,         // 11
                       rgr2_size,         // 12
                       mrgr2_size         // 13
            );
        }

        for (size_t i = 0; i < num_test; i++) {
            std::vector<bool> in_src = i < 100 ? int2bvec(i + 1, num_ap)
                                               : rand_bvec.at(i - 100),
                              in = permutate_input_bits(perm_tbl, in_src),
                              rin(in.rbegin(), in.rend());

            bool expected = check_if_accept(aut, in_src, num_ap);

            {
                bool got = check_if_accept(gr, in);
                if (expected != got)
                    error::die("[{}] [{}] [{}] {} != {}", fml, i + 1,
                               bvec2str(in), expected, got);
            }
            if (mgr) {
                bool got = check_if_accept(*mgr, in);
                if (expected != got)
                    error::die("[{}] [{}] [{}] {} != {}", fml, i + 1,
                               bvec2str(in), expected, got);
            }
            {
                bool got = check_if_accept(rgr, rin);
                if (expected != got)
                    error::die("[{}] [{}] [{}] {} != {}", fml, i + 1,
                               bvec2str(rin), expected, got);
            }
            if (mrgr) {
                bool got = check_if_accept(*mrgr, rin);
                if (expected != got)
                    error::die("[{}] [{}] [{}] {} != {}", fml, i + 1,
                               bvec2str(rin), expected, got);
            }
            {
                bool got = check_if_accept(rgr2, rin);
                if (expected != got)
                    error::die("[{}] [{}] [{}] {} != {}", fml, i + 1,
                               bvec2str(rin), expected, got);
            }
            if (mrgr2) {
                bool got = check_if_accept(*mrgr2, rin);
                if (expected != got)
                    error::die("[{}] [{}] [{}] {} != {}", fml, i + 1,
                               bvec2str(rin), expected, got);
            }
        }
    }
}

int main(int argc, char** argv)
{
    if (argc != 2) {
        spdlog::error("Usage: {} LOG-FILE1", argv[0]);
        return 1;
    }

    auto max_size = 1024 * 1024 * 1;
    auto logger1 = spdlog::basic_logger_mt("random log1", argv[1]);
    logger1->set_pattern("%v");

    test_from_ltl_formula(std::cin, 5, 300, logger1);
    return 0;
}
