#ifndef HOMFA_OFFLINE_DFA
#define HOMFA_OFFLINE_DFA

#include "backstream_dfa_runner.hpp"

class OfflineDFARunner {
private:
    BackstreamDFARunner runner_;
    InputStream<TRGSWLvl1FFT> &input_stream_;

public:
    OfflineDFARunner(Graph graph, InputStream<TRGSWLvl1FFT> &input_stream,
                     std::shared_ptr<GateKey> gate_key = nullptr);

    const Graph &graph() const
    {
        return runner_.graph();
    }

    TLWELvl1 result() const;
    void eval();
};

#endif
