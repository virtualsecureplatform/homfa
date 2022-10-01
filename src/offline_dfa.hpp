#ifndef HOMFA_OFFLINE_DFA
#define HOMFA_OFFLINE_DFA

#include "backstream_dfa_runner.hpp"

class OfflineDFARunner {
private:
    BackstreamDFARunner runner_;

public:
    OfflineDFARunner(Graph graph, size_t input_size, size_t boot_interval,
                     std::shared_ptr<GateKey> gate_key, bool sanitize_result);

    const Graph& graph() const
    {
        return runner_.graph();
    }

    TLWELvl1 result() const;
    void eval_one(const TRGSWLvl1FFT& input);
};

#endif
