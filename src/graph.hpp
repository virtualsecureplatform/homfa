#ifndef HOMFA_GRAPH_HPP
#define HOMFA_GRAPH_HPP

#include <cstdint>
#include <set>
#include <string>
#include <vector>

class Graph {
public:
    using State = int;

private:
    // When on state `index`,
    //   input 0 -> next state is `child0`,
    //              next weight is (w_{org} * X^k + gain0)
    //   input 1 -> next state is `child1`
    //              next weight is (w_{org} * X^k + gain1)
    struct TableItem {
        State index, child0, child1;
        std::vector<State> parents0, parents1;
        uint64_t gain0, gain1;
    };
    std::vector<TableItem> table_;
    std::vector<std::vector<State>> states_at_depth_;
    std::set<State> final_state_;

public:
    Graph();
    Graph(const std::string &filename);
    size_t size() const;
    bool is_final_state(State state) const;
    State next_state(State state, bool input) const;
    std::vector<State> prev_states(State state, bool input) const;
    State initial_state() const;
    uint64_t gain(State from, bool input) const;
    void reserve_states_at_depth(size_t depth);
    std::vector<State> states_at_depth(size_t depth) const;
    std::vector<State> all_states() const;
};

#endif
