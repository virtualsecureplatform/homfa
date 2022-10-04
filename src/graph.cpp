#include "graph.hpp"
#include "error.hpp"

#include <cassert>
#include <fstream>
#include <map>
#include <numeric>
#include <optional>
#include <queue>
#include <regex>
#include <set>
#include <sstream>

#include <spdlog/spdlog.h>
#include <spot/misc/bddlt.hh>
#include <spot/misc/minato.hh>
#include <spot/tl/parse.hh>
#include <spot/twaalgos/hoa.hh>
#include <spot/twaalgos/translate.hh>

// Thanks to:
// https://helloacm.com/the-union-find-disjoint-set-implementation-in-java/
class UnionFind {
private:
    std::vector<int> g_;

public:
    UnionFind(int size) : g_(size)
    {
        std::iota(g_.begin(), g_.end(), 0);
    }

    int find(int x)
    {
        if (x == g_.at(x))
            return x;
        return g_.at(x) = find(g_.at(x));
    }

    void unite(int x, int y)
    {
        int px = find(x), py = find(y);
        if (px != py)
            g_.at(px) = py;
    }

    int size()
    {
        int ans = 0;
        for (int i = 0; i < g_.size(); i++)
            if (i == g_.at(i))
                ans++;
        return ans;
    }
};

void nfa_dump_dot(std::ostream& os, const std::set<Graph::State>& init_sts,
                  const std::set<Graph::State>& final_sts,
                  const Graph::NFADelta& delta)
{
    os << "digraph \"graph\" {\n"
       << "rankdir=\"LR\""
       << "  node[shape=\"circle\"]\n"
       << "  I [label=\"\", style=invis, width=0]\n"
       << "  I -> I0\n"
       << "  I0 [label=\"\"]\n";
    for (Graph::State q : init_sts)
        os << "  I0 -> " << q << " [label=\"ε\"]\n";
    for (auto&& [q, q0s, q1s] : delta) {
        os << "  " << q << " [label=\"" << q << "\""
           << (final_sts.contains(q) ? ", peripheries=2" : "") << "]";
        for (Graph::State q0 : q0s) {
            if (std::find(q1s.begin(), q1s.end(), q0) == q1s.end())
                os << "  " << q << " -> " << q0 << " [label=\"0\"]\n";
            else
                os << "  " << q << " -> " << q0 << " [label=\"0,1\"]\n";
        }
        for (Graph::State q1 : q1s)
            if (std::find(q0s.begin(), q0s.end(), q1) == q0s.end())
                os << "  " << q << " -> " << q1 << " [label=\"1\"]\n";
    }
    os << "}\n";
}

Graph::Graph()
{
}

Graph::Graph(State init_st, const std::set<State>& final_sts,
             const DFADelta& delta)
    : delta_(delta),
      parents0_(delta.size()),
      parents1_(delta.size()),
      states_at_depth_(),
      final_state_(final_sts),
      final_state_vec_(delta.size(), false),
      init_state_(init_st)
{
    for (auto&& [q, q0, q1] : delta) {
        parents0_.at(q0).push_back(q);
        parents1_.at(q1).push_back(q);
    }
    for (State q : final_sts)
        final_state_vec_.at(q) = true;
}

Graph Graph::from_istream(std::istream& is)
{
    std::string field;
    auto load_comma_separated_states =
        [](const std::string& s) -> std::vector<State> {
        if (s == "_")
            return {};

        // Thanks to: https://faithandbrave.hateblo.jp/entry/2014/05/01/171631
        std::vector<State> ret;
        std::istringstream iss{s};
        std::string field;
        try {
            while (std::getline(iss, field, ','))
                ret.push_back(std::stoi(field));
        }
        catch (const std::invalid_argument& e) {
            error_die(
                "Expected number as comma separted state, but got {} ({})",
                field, e.what());
        }
        return ret;
    };

    std::set<State> init_sts, final_sts;
    NFADelta delta;
    std::regex re(R"(^(>)?(\d+)(\*)?\s+(_|[\d,]+)\s+(_|[\d,]+)$)");
    std::smatch match;
    bool is_dfa = true;

    std::string line;
    assert(is);
    while (std::getline(is, line)) {
        if (!std::regex_match(line, match, re)) {
            spdlog::info("Skip line \"{}\"", line);
            continue;
        }

        bool initial = match[1].matched, final = match[3].matched;
        State q = delta.size();
        std::vector<State> q0s = load_comma_separated_states(match[4].str()),
                           q1s = load_comma_separated_states(match[5].str());

        // validate
        if (q != std::stoi(match[2].str()))
            error_die("Invalid state number: {} != {}",
                      std::stoi(match[2].str()), q);

        if (initial)
            init_sts.insert(q);
        if (final)
            final_sts.insert(q);
        delta.emplace_back(q, q0s, q1s);

        if (q0s.size() != 1 || q1s.size() != 1)
            is_dfa = false;
    }
    if (init_sts.size() == 0)
        init_sts.insert(0);
    if (init_sts.size() != 1)
        is_dfa = false;

    /*
    for (auto &&[q, q0s, q1s] : delta) {
        std::stringstream ss;
        ss << q << "\t|";
        for (Graph::State q0 : q0s)
            ss << q0 << ",";
        ss << "|\t|";
        for (Graph::State q1 : q1s)
            ss << q1 << ",";
        ss << "|";
        spdlog::info(ss.str());
    }
    */

    if (is_dfa) {
        assert(init_sts.size() == 1);
        DFADelta new_delta;
        for (auto&& [q, q0s, q1s] : delta) {
            assert(q0s.size() == 1 && q1s.size() == 1);
            new_delta.emplace_back(q, q0s.at(0), q1s.at(0));
        }
        return Graph{*init_sts.begin(), final_sts, new_delta};
    }

    return Graph::from_nfa(init_sts, final_sts, delta);
}

Graph Graph::from_file(const std::string& filename)
{
    std::ifstream ifs{filename};
    assert(ifs);
    return Graph::from_istream(ifs);
}

Graph Graph::from_att_istream(std::istream& is)
{
    assert(is);

    std::set<State> init_sts = {0}, final_sts = {};
    NFADelta delta;

    std::regex re1(R"(^([0-9]+)\t([0-9]+)\t(0|1)$)"), re2(R"(^([0-9]+)$)");
    std::smatch match;
    std::string line;
    while (std::getline(is, line)) {
        if (std::regex_match(line, match, re1)) {  // Format: "from to ch"
            int from = std::stoi(match[1].str()),
                to = std::stoi(match[2].str()), ch = std::stoi(match[3].str());
            while (delta.size() <= from)
                delta.push_back({delta.size(), {}, {}});
            if (ch == 0)
                std::get<1>(delta.at(from)).push_back(to);
            else if (ch == 1)
                std::get<2>(delta.at(from)).push_back(to);
            else
                error_die("Invalid letter: {}", ch);
        }
        else if (std::regex_match(line, match, re2)) {  // Format: st
            int st = std::stoi(match[1].str());
            final_sts.insert(st);
        }
        else {
            error_die("Invalid line: {}", line);
        }
    }

    return Graph::from_nfa(init_sts, final_sts, delta);
}

Graph Graph::from_att_file(const std::string& filename)
{
    std::ifstream ifs{filename};
    if (!ifs)
        error_die("Invalid filename: {}", filename);
    return Graph::from_att_istream(ifs);
}

// Input  NFA Mn: (Qn, {0, 1}, dn, q0n, Fn)
// Output DFA Md: (Qd, {0, 1}, df, q0f, Ff)
Graph Graph::from_nfa(const std::set<State>& q0n, const std::set<State>& Fn,
                      const NFADelta& dn)
{
    using StateSubset = std::set<State>;

    // 2^{Qn} -> Qd
    std::map<StateSubset, State> st_map;
    // df
    DFADelta df;
    // Ff
    std::set<State> Ff;

    auto get_or_create_state = [&](const StateSubset& qs) {
        auto it = st_map.find(qs);
        if (it != st_map.end())
            return it->second;
        State qsd = df.size();
        df.emplace_back(qsd, -1, -1);
        st_map.emplace(qs, qsd);
        return qsd;
    };

    /*
    auto dump_qs = [](const auto &qs, std::string prefix = "") {
        std::stringstream ss;
        ss << prefix << "[";
        for (State q : qs)
            ss << q << " ";
        ss << "]";
        spdlog::info("{}", ss.str());
    };
    */

    std::set<StateSubset> visited;
    std::queue<StateSubset> que;
    que.push(q0n);
    while (!que.empty()) {
        StateSubset qs = que.front();
        que.pop();
        if (visited.contains(qs))
            continue;

        StateSubset qs0, qs1;
        for (State q : qs) {
            auto&& [q_, dst0, dst1] = dn.at(q);
            qs0.insert(dst0.begin(), dst0.end());
            qs1.insert(dst1.begin(), dst1.end());
        }
        bool final = std::any_of(qs.begin(), qs.end(),
                                 [&Fn](size_t q) { return Fn.contains(q); });

        State qsd = get_or_create_state(qs), qs0d = get_or_create_state(qs0),
              qs1d = get_or_create_state(qs1);
        df.at(qsd) = std::make_tuple(qsd, qs0d, qs1d);

        visited.insert(qs);
        if (final)
            Ff.insert(qsd);
        que.push(qs0);
        que.push(qs1);
    }

    return Graph{get_or_create_state(q0n), Ff, df};
}

Graph Graph::from_ltl_formula(const std::string& formula, size_t var_size,
                              bool make_all_live_states_final)
{
    auto [init_sts, final_sts, delta] =
        ltl_to_nfa_tuple(formula, var_size, make_all_live_states_final);
    return Graph::from_nfa(init_sts, final_sts, delta);
}

Graph Graph::from_ltl_formula_reversed(const std::string& formula,
                                       size_t var_size,
                                       bool make_all_live_states_final)
{
    auto [init_sts, final_sts, delta] =
        ltl_to_nfa_tuple(formula, var_size, make_all_live_states_final);
    NFADelta delta_rev = reversed_nfa_delta(delta);
    return Graph::from_nfa(final_sts, init_sts, delta_rev);
}

size_t Graph::size() const
{
    return delta_.size();
}

bool Graph::is_final_state(State state) const
{
    // return final_state_.contains(state);
    return final_state_vec_.at(state);
}

Graph::State Graph::next_state(State state, bool input) const
{
    auto& t = delta_.at(state);
    return input ? std::get<2>(t) : std::get<1>(t);
}

const std::vector<Graph::State>& Graph::prev_states(State state,
                                                    bool input) const
{
    if (input)
        return parents1_.at(state);
    else
        return parents0_.at(state);
}

Graph::State Graph::transition64(State src, uint64_t input, int length) const
{
    Graph::State dst = src;
    for (size_t i = 0; i < length; i++)
        dst = next_state(dst, (input & (1 << i)) != 0);
    return dst;
}

Graph::State Graph::initial_state() const
{
    return init_state_;
}

void Graph::reserve_states_at_depth(size_t depth)
{
    states_at_depth_.clear();
    states_at_depth_.shrink_to_fit();
    states_at_depth_.reserve(depth);

    std::vector<bool> sts(size(), false);
    sts.at(initial_state()) = true;
    std::vector<State> tmp;
    tmp.reserve(size());
    for (size_t i = 0; i < depth; i++) {
        for (Graph::State st = 0; st < size(); st++) {
            if (sts.at(st))
                tmp.push_back(st);
            sts.at(st) = false;
        }
        states_at_depth_.push_back(tmp);

        for (State st : tmp) {
            sts.at(next_state(st, false)) = true;
            sts.at(next_state(st, true)) = true;
        }

        tmp.clear();
    }
}

std::vector<Graph::State> Graph::states_at_depth(size_t depth) const
{
    return states_at_depth_.at(depth);
}

std::vector<Graph::State> Graph::all_states() const
{
    std::vector<State> ret(size());
    std::iota(ret.begin(), ret.end(), 0);
    return ret;
}

std::vector<std::vector<Graph::State>> Graph::track_live_states(
    const std::vector<Graph::State>& init_live_states, size_t max_depth)
{
    std::vector<std::vector<Graph::State>> at_depth;
    at_depth.push_back(init_live_states);

    std::set<Graph::State> tmp1(init_live_states.begin(),
                                init_live_states.end()),
        tmp2;
    for (size_t i = 0; i < max_depth; i++) {
        tmp2.clear();
        for (Graph::State q : tmp1) {
            tmp2.insert(next_state(q, true));
            tmp2.insert(next_state(q, false));
        }
        {
            using std::swap;
            swap(tmp1, tmp2);
        }
        at_depth.emplace_back(tmp1.begin(), tmp1.end());
    }

    return at_depth;
}

Graph Graph::reversed() const
{
    NFADelta delta(size());
    for (State q : all_states()) {
        delta.at(q) =
            std::make_tuple(q, prev_states(q, false), prev_states(q, true));
    }
    return Graph::from_nfa(final_state_, {initial_state()}, delta);
}

Graph Graph::minimized() const
{
    return removed_unreachable().grouped_nondistinguishable();
}

Graph Graph::removed_unreachable() const
{
    std::set<State> reachable;
    std::queue<State> que;
    que.push(initial_state());
    while (!que.empty()) {
        State q = que.front();
        que.pop();
        auto [it, inserted] = reachable.insert(q);
        if (!inserted)
            continue;
        que.push(next_state(q, false));
        que.push(next_state(q, true));
    }

    std::unordered_map<State, State> old2new;
    for (State old : reachable)
        old2new.emplace(old, old2new.size());

    State init_st = old2new.at(initial_state());
    std::set<State> final_sts;
    for (State q : final_state_)
        if (reachable.contains(q))
            final_sts.insert(old2new.at(q));
    DFADelta delta(reachable.size());
    for (auto&& [index, child0, child1] : delta_) {
        if (!reachable.contains(index))
            continue;
        State q = old2new.at(index), q0 = old2new.at(child0),
              q1 = old2new.at(child1);
        delta.at(q) = std::make_tuple(q, q0, q1);
    }

    return Graph{init_st, final_sts, delta};
}

Graph Graph::grouped_nondistinguishable() const
{
    // Table-filling algorithm
    size_t siz = size();
    std::vector<bool> table(siz * siz, false);  // FIXME: use only half
    std::queue<std::pair<State, State>> que;
    for (State qa = 0; qa < siz; qa++) {
        for (State qb = qa + 1; qb < siz; qb++) {
            bool qa_final = is_final_state(qa), qb_final = is_final_state(qb);
            if ((qa_final && !qb_final) || (!qa_final && qb_final)) {
                que.emplace(qa, qb);
                table.at(qa + qb * siz) = true;
            }
        }
    }
    while (!que.empty()) {
        auto [ql, qr] = que.front();
        que.pop();
        assert(ql < qr);
        const static bool bv[] = {true, false};
        for (bool in : bv) {
            for (State qa : prev_states(ql, in)) {
                for (State qb : prev_states(qr, in)) {
                    if (qa < qb && !table.at(qa + siz * qb)) {
                        table.at(qa + siz * qb) = true;
                        que.emplace(qa, qb);
                    }
                    else if (qa > qb && !table.at(qb + siz * qa)) {
                        table.at(qb + siz * qa) = true;
                        que.emplace(qb, qa);
                    }
                }
            }
        }
    }

    // Group the states
    // FIXME: must find more efficient way. maybe union-find?
    UnionFind uf(siz);
    for (State qa = 0; qa < siz; qa++) {
        for (State qb = qa + 1; qb < siz; qb++) {
            bool equiv = !table.at(qa + qb * siz);
            if (!equiv)
                continue;
            uf.unite(qa, qb);
        }
    }
    std::vector<int> uf2st(siz, -1);
    std::vector<std::set<State>> st(uf.size());
    {
        size_t next = 0;
        for (State q : all_states()) {
            int uf_i = uf.find(q);
            if (uf2st.at(uf_i) == -1)
                uf2st.at(uf_i) = next++;
            st.at(uf2st.at(uf_i)).insert(q);
        }
    }

    std::optional<State> init_st;
    std::set<State> final_sts;
    DFADelta delta;
    for (size_t q = 0; q < st.size(); q++) {
        std::set<State>& s = st.at(q);
        State repr = *s.begin();

        bool initial = std::any_of(s.begin(), s.end(), [this](State q) {
            return initial_state() == q;
        });
        bool final = is_final_state(repr);
        if (initial)
            init_st.emplace(q);
        if (final)
            final_sts.insert(q);

        State q0 = uf2st.at(uf.find(next_state(repr, false))),
              q1 = uf2st.at(uf.find(next_state(repr, true)));
        delta.emplace_back(q, q0, q1);
    }

    return Graph{init_st.value(), final_sts, delta};
}

Graph Graph::negated() const
{
    std::set<State> new_final;
    for (Graph::State q : all_states())
        if (!is_final_state(q))
            new_final.insert(q);
    return Graph{init_state_, new_final, delta_};
}

void Graph::dump(std::ostream& os) const
{
    for (Graph::State q : all_states()) {
        if (initial_state() == q)
            os << ">";
        os << q;
        if (is_final_state(q))
            os << "*";
        os << "\t";
        os << next_state(q, false);
        os << "\t";
        os << next_state(q, true);
        os << "\n";
    }
}

void Graph::dump_dot(std::ostream& os) const
{
    os << "digraph \"graph\" {\n"
       << "rankdir=\"LR\""
       << "  node[shape=\"circle\"]\n"
       << "  I [label=\"\", style=invis, width=0]\n"
       << "  I -> " << initial_state() << "\n";
    for (Graph::State q : all_states()) {
        Graph::State q0 = next_state(q, false), q1 = next_state(q, true);
        os << "  " << q << " [label=\"" << q << "\""
           << (is_final_state(q) ? ", peripheries=2" : "") << "]";
        if (q0 == q1) {
            os << "  " << q << " -> " << q0 << " [label=\"0,1\"]\n";
        }
        else {
            os << "  " << q << " -> " << q0 << " [label=\"0\"]\n";
            os << "  " << q << " -> " << q1 << " [label=\"1\"]\n";
        }
    }
    os << "}\n";
}

void Graph::dump_att(std::ostream& os) const
{
    for (Graph::State q : all_states()) {
        os << q << "\t" << next_state(q, false) << "\t0\n"
           << q << "\t" << next_state(q, true) << "\t1\n";
        if (is_final_state(q))
            os << q << "\n";
    }
}

spot::twa_graph_ptr ltl_to_monitor(const std::string& formula, size_t var_size)
{
    spot::parsed_formula pf = spot::parse_infix_psl(formula);
    assert(!pf.format_errors(std::cerr));
    spot::bdd_dict_ptr dict = spot::make_bdd_dict();
    spot::twa_graph_ptr aut = spot::make_twa_graph(dict);
    for (size_t i = 0; i < var_size; i++)
        aut->register_ap(fmt::format("p{}", i));
    spot::translator trans{dict};
    trans.set_type(spot::postprocessor::Monitor);
    trans.set_pref(spot::postprocessor::Any);
    return trans.run(pf.f);
}

std::tuple<std::set<Graph::State>, std::set<Graph::State>, Graph::NFADelta>
Graph::ltl_to_nfa_tuple(const std::string& formula, size_t var_size,
                        bool make_all_live_states_final)
{
    spot::twa_graph_ptr aut = ltl_to_monitor(formula, var_size);

    std::unordered_map<int, size_t> var2idx;
    {
        spot::bdd_dict_ptr dict = aut->get_dict();
        for (size_t i = 0; i < var_size; i++) {
            auto it =
                dict->var_map.find(spot::formula::ap(fmt::format("p{}", i)));
            if (it != dict->var_map.end())
                var2idx.emplace(it->second, i);
        }
    }

    size_t ns = aut->num_states();
    NFADelta delta;
    for (State i = 0; i < ns; i++)
        delta.push_back({i, {}, {}});
    for (State src = 0; src < ns; src++) {
        spdlog::debug("{}/{}", src, ns);
        for (auto& t : aut->out(src)) {
            State dst = t.dst;

            spdlog::debug("{} {}", src, t.dst);

            std::vector<
                std::tuple<State, std::vector<State>, std::vector<State>, bdd>>
                reversed;
            std::map<bdd, State, spot::bdd_less_than> bdd2rst;
            {
                auto get = [&](bdd b) {
                    auto it = bdd2rst.find(b);
                    if (it != bdd2rst.end())
                        return it->second;
                    State q = reversed.size();
                    reversed.push_back({q, {}, {}, b});
                    bdd2rst.emplace(b, q);
                    return q;
                };

                std::queue<bdd> que;
                que.push(t.cond);
                reversed.push_back({0, {}, {}, t.cond});
                bdd2rst.emplace(t.cond, 0);
                std::set<bdd, spot::bdd_less_than> visited;
                while (!que.empty()) {
                    bdd node = que.front();
                    que.pop();

                    if (visited.contains(node))
                        continue;
                    visited.emplace(node);

                    if (node == bddtrue || node == bddfalse)
                        continue;

                    bdd low = bdd_low(node), high = bdd_high(node);
                    State q = bdd2rst.at(node), q0 = get(low), q1 = get(high);
                    std::get<1>(reversed.at(q0)).push_back(q);
                    std::get<2>(reversed.at(q1)).push_back(q);
                    que.push(low);
                    que.push(high);
                }
            }

            /*
            for (size_t i = 0; i < reversed.size(); i++) {
                auto&& [i_, q0s, q1s, b_] = reversed.at(i);
                std::cerr << i << "\t" << i_ << "\t|";
                for (State q0 : q0s)
                    std::cerr << q0 << ",";
                std::cerr << "|\t|";
                for (State q1 : q1s)
                    std::cerr << q1 << ",";
                std::cerr << "|\n";
            }
            std::cerr << "\n";
            */

            std::unordered_map<State, State> rst2st;
            auto get = [&](State rq) {
                auto it = rst2st.find(rq);
                if (it != rst2st.end())
                    return it->second;
                State q = delta.size();
                delta.push_back({q, {}, {}});
                rst2st.emplace(rq, q);
                return q;
            };

            std::set<State> visited;
            std::queue<std::tuple<State, size_t, State>> que;
            que.push({bdd2rst.at(bddtrue), var_size, dst});
            std::optional<State> src_alt;
            while (!que.empty()) {
                auto [cur_rq, cur_var_idx, cur_q] = que.front();
                que.pop();

                if (visited.contains(cur_rq))
                    continue;
                visited.insert(cur_rq);

                auto&& [q_, q0s, q1s, b_] = reversed.at(cur_rq);

                if (cur_var_idx == 0) {
                    assert(q0s.empty() && q1s.empty() && !src_alt);
                    src_alt.emplace(cur_q);
                    continue;
                }

                if (q0s.empty() && q1s.empty()) {
                    assert(!src_alt);
                    while (cur_var_idx > 0) {
                        cur_var_idx--;
                        State q = delta.size();
                        delta.push_back({q, {cur_q}, {cur_q}});
                        cur_q = q;
                    }
                    src_alt.emplace(cur_q);
                    continue;
                }

                for (State q0 : q0s) {
                    bdd b = std::get<3>(reversed.at(q0));
                    size_t q0_var_idx = var2idx.at(bdd_var(b));
                    size_t cidx = cur_var_idx;
                    State cq = cur_q;
                    spdlog::debug("{}", cidx);
                    while (cidx > q0_var_idx + 1) {
                        cidx--;
                        State q = delta.size();
                        delta.push_back({q, {cq}, {cq}});
                        cq = q;
                        spdlog::debug("{}", cidx);
                    }
                    State next = get(q0);
                    std::get<1>(delta.at(next)).push_back(cq);
                    que.push({q0, q0_var_idx, next});
                }
                for (State q1 : q1s) {
                    bdd b = std::get<3>(reversed.at(q1));
                    size_t q1_var_idx = var2idx.at(bdd_var(b));
                    size_t cidx = cur_var_idx;
                    State cq = cur_q;
                    while (cidx > q1_var_idx + 1) {
                        cidx--;
                        State q = delta.size();
                        delta.push_back({q, {cq}, {cq}});
                        cq = q;
                    }
                    State next = get(q1);
                    std::get<2>(delta.at(next)).push_back(cq);
                    que.push({q1, q1_var_idx, next});
                }
            }
            {
                assert(src_alt);
                spdlog::debug("{}", *src_alt);
                auto [src_alt_, q0s, q1s] = delta.at(*src_alt);
                assert(q0s.size() <= 1 && q1s.size() <= 1);
                if (q0s.size() == 1)
                    std::get<1>(delta.at(src)).push_back(q0s.at(0));
                if (q1s.size() == 1)
                    std::get<2>(delta.at(src)).push_back(q1s.at(0));

                spdlog::debug("{} {} END TABLE BEGIN", src, dst);
                /*
                for (size_t i = 0; i < delta.size(); i++) {
                    auto&& [i_, q0s, q1s] = delta.at(i);
                    std::cerr << i << "\t" << i_ << "\t|";
                    for (State q0 : q0s)
                        std::cerr << q0 << ",";
                    std::cerr << "|\t|";
                    for (State q1 : q1s)
                        std::cerr << q1 << ",";
                    std::cerr << "|\n";
                }
                */
                spdlog::debug("{} {} END TABLE END", src, dst);
            }
        }
    }

    /*
    for (auto&& [i, q0s, q1s] : delta) {
        std::cerr << i << "\t";
        for (State q0 : q0s)
            std::cerr << q0 << ",";
        std::cerr << "\t";
        for (State q1 : q1s)
            std::cerr << q1 << ",";
        std::cerr << "\n";
    }
    */

    std::set<State> init_sts = {static_cast<State>(
                        aut->get_init_state_number())},
                    final_sts;
    if (make_all_live_states_final) {
        for (State i = 0; i < delta.size(); i++)
            final_sts.insert(i);
    }
    else {
        for (State i = 0; i < ns; i++)
            final_sts.insert(i);
    }

    return std::make_tuple(init_sts, final_sts, delta);
}

Graph::NFADelta Graph::reversed_nfa_delta(const NFADelta& src)
{
    std::vector<std::vector<State>> prev0(src.size()), prev1(src.size());
    for (State q = 0; q < src.size(); q++) {
        auto&& [q_, q0s, q1s] = src.at(q);
        for (State q0 : q0s)
            prev0.at(q0).push_back(q);
        for (State q1 : q1s)
            prev1.at(q1).push_back(q);
    }

    NFADelta ret(src.size());
    for (State q = 0; q < src.size(); q++)
        ret.at(q) = std::make_tuple(q, prev0.at(q), prev1.at(q));
    return ret;
}
