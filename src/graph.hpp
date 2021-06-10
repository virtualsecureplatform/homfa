#ifndef HOMFA_GRAPH_HPP
#define HOMFA_GRAPH_HPP

#include <cstdint>
#include <set>
#include <string>
#include <vector>

class Graph {
public:
    using State = int;
    using DFADelta = std::vector<std::tuple<State, State, State>>;
    using NFADelta =
        std::vector<std::tuple<State, std::vector<State>, std::vector<State>>>;

private:
    struct TableItem {
        State index, child0, child1;
        std::vector<State> parents0, parents1;
    };
    std::vector<TableItem> table_;
    std::vector<std::vector<State>> states_at_depth_;
    std::set<State> final_state_;
    State init_state_;

public:
    Graph();
    Graph(State init_st, const std::set<State>& final_sts,
          const DFADelta& delta);

    static Graph from_file(const std::string& filename);
    static Graph from_nfa(const std::set<State>& init_sts,
                          const std::set<State>& final_sts,
                          const NFADelta& delta);

    size_t size() const;
    bool is_final_state(State state) const;
    State next_state(State state, bool input) const;
    std::vector<State> prev_states(State state, bool input) const;
    State initial_state() const;
    void reserve_states_at_depth(size_t depth);
    std::vector<State> states_at_depth(size_t depth) const;
    std::vector<State> all_states() const;
    Graph reversed() const;
    Graph minimized() const;
    Graph removed_unreachable() const;
    Graph grouped_nondistinguishable() const;
    void dump(std::ostream& os) const;
    void dump_dot(std::ostream& os) const;
};

Graph::NFADelta reversed_nfa_delta(const Graph::NFADelta& src);

#endif
