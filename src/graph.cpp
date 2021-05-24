#include "graph.hpp"

#include <cassert>
#include <fstream>
#include <numeric>

Graph::Graph()
{
}

Graph::Graph(const std::string &filename)
{
    std::ifstream ifs{filename};
    assert(ifs);
    int N;
    ifs >> N;
    for (int i = 0; i < N; i++) {
        std::string no;
        int s0, s1;
        ifs >> no;
        ifs >> s0 >> s1;
        if (no.at(no.size() - 1) == '*')
            final_state_.insert(i);
        table_.push_back(TableItem{i, s0, s1, {}, {}, 0, 0});
    }
    for (auto &&item : table_) {
        table_.at(item.child0).parents0.push_back(item.index);
        table_.at(item.child1).parents1.push_back(item.index);
        if (final_state_.contains(item.child0))
            item.gain0 = 1;
        if (final_state_.contains(item.child1))
            item.gain1 = 1;
    }
}

size_t Graph::size() const
{
    return table_.size();
}

bool Graph::is_final_state(State state) const
{
    return final_state_.contains(state);
}

Graph::State Graph::next_state(State state, bool input) const
{
    auto &t = table_.at(state);
    return input ? t.child1 : t.child0;
}

std::vector<Graph::State> Graph::prev_states(State state, bool input) const
{
    auto &t = table_.at(state);
    return input ? t.parents1 : t.parents0;
}

Graph::State Graph::initial_state() const
{
    return 0;
}

uint64_t Graph::gain(State from, bool input) const
{
    auto &s = table_.at(from);
    return input ? s.gain1 : s.gain0;
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
