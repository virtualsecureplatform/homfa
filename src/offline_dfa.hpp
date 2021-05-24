#ifndef HOMFA_OFFLINE_DFA
#define HOMFA_OFFLINE_DFA

#include "graph.hpp"
#include "tfhepp_util.hpp"

class OfflineDFARunner {
    // Interval for bootstrapping
    const static size_t BOOT_INTERVAL = 8000;

private:
    const Graph &graph_;
    InputStream<TRGSWLvl1FFT> &input_stream_;
    std::vector<TRLWELvl1> weight_;
    bool has_evaluated_;
    std::shared_ptr<GateKey> gate_key_;

public:
    OfflineDFARunner(const Graph &graph,
                     InputStream<TRGSWLvl1FFT> &input_stream,
                     std::shared_ptr<GateKey> gate_key = nullptr);

    TLWELvl1 result() const;
    void eval();

private:
    void next_weight(TRLWELvl1 &out, int j, Graph::State from,
                     bool input) const;
    void bootstrapping_of_weight();
};

#endif
