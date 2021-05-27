#ifndef HOMFA_OFFLINE_DFA
#define HOMFA_OFFLINE_DFA

#include "graph.hpp"
#include "tfhepp_util.hpp"

#include <cufhe.h>
#include <cufhe_gpu.cuh>

class OfflineDFARunner {
    // Interval for bootstrapping
    const static size_t BOOT_INTERVAL = 8000;

private:
    const Graph &graph_;
    InputStream<TRGSWLvl1FFT> &input_stream_;
    std::vector<TRLWELvl1> weight_;
    bool has_evaluated_;
    std::shared_ptr<GateKeyFFT> gate_key_;

public:
    OfflineDFARunner(const Graph &graph,
                     InputStream<TRGSWLvl1FFT> &input_stream,
                     std::shared_ptr<GateKeyFFT> gate_key = nullptr);

    TLWELvl1 result() const;
    void eval();

private:
    void next_weight(TRLWELvl1 &out, int j, Graph::State from,
                     bool input) const;
    void bootstrap_weight();
};

class GPUOfflineDFARunner {
    // Interval for bootstrapping
    const static size_t BOOT_INTERVAL = 8000;

private:
    const Graph &graph_;
    InputStream<TRGSWLvl1NTT> &input_stream_;
    const bool can_bootstrap_;
    TLWELvl1 res_;
    bool has_evaluated_;

public:
    GPUOfflineDFARunner(const Graph &graph,
                        InputStream<TRGSWLvl1NTT> &input_stream,
                        bool can_bootstrap = false);

    TLWELvl1 result() const;
    void eval();

private:
    // void bootstrap_weight();
};

#endif
