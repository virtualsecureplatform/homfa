#ifndef HOMFA_OFFLINE_DFA
#define HOMFA_OFFLINE_DFA

#include "graph.hpp"
#include "tfhepp_util.hpp"

#include <cufhe.h>
#include <cufhe_gpu.cuh>

class OfflineDFARunner {
    // Interval for bootstrapping
    const static size_t BOOT_INTERVAL = 8000;
    // Number of cuFHE's streams
    const static size_t CUFHE_STREAM_SIZE = 800;

private:
    const Graph &graph_;
    InputStream<TRGSWLvl1FFT> &input_stream_;
    std::vector<TRLWELvl1> weight_;
    bool has_evaluated_;
    std::shared_ptr<GateKeyFFT> gate_key_;

    bool use_gpu_;
    std::vector<cufhe::Stream> streams_;
    // Workspace for cuFHE's bootstrapping
    std::vector<cufhe::cuFHETRLWElvl1> temp_cufhe_trlwe_;
    // Workspace for cuFHE's bootstrapping
    std::vector<cufhe::Ctxt> temp_cufhe_ctxt_;

public:
    OfflineDFARunner(const Graph &graph,
                     InputStream<TRGSWLvl1FFT> &input_stream, bool use_gpu,
                     std::shared_ptr<GateKeyFFT> gate_key = nullptr);

    TLWELvl1 result() const;
    void eval();

private:
    void next_weight(TRLWELvl1 &out, int j, Graph::State from,
                     bool input) const;
    void bootstrap_weight();

    // Don't allow copying this class
    OfflineDFARunner(const OfflineDFARunner &) = delete;
    OfflineDFARunner &operator=(OfflineDFARunner &) = delete;
};

#endif
