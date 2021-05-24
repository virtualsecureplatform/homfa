#ifndef HOMFA_DFA_HPP
#define HOMFA_DFA_HPP

#include "graph.hpp"

#include <set>
#include <vector>

#include <tfhe++.hpp>

using Lvl0 = TFHEpp::lvl0param;
using TLWELvl0 = TFHEpp::TLWE<Lvl0>;
using Lvl1 = TFHEpp::lvl1param;
using TLWELvl1 = TFHEpp::TLWE<Lvl1>;
using TRGSWLvl1FFT = TFHEpp::TRGSWFFT<Lvl1>;
using TRLWELvl1 = TFHEpp::TRLWE<Lvl1>;
using PolyLvl1 = TFHEpp::Polynomial<Lvl1>;
using SecretKey = TFHEpp::SecretKey;
using GateKey = TFHEpp::GateKey;

template <class T>
class InputStream {
public:
    InputStream()
    {
    }
    virtual ~InputStream()
    {
    }

    virtual size_t size() const = 0;
    virtual T next() = 0;
};

class TRGSWLvl1InputStreamFromCtxtFile : public InputStream<TRGSWLvl1FFT> {
private:
    std::vector<TRGSWLvl1FFT> data_;
    std::vector<TRGSWLvl1FFT>::iterator head_;

public:
    TRGSWLvl1InputStreamFromCtxtFile(const std::string &filename);

    size_t size() const override;
    TRGSWLvl1FFT next() override;
};

class ReversedTRGSWLvl1InputStreamFromCtxtFile
    : public InputStream<TRGSWLvl1FFT> {
private:
    std::vector<TRGSWLvl1FFT> data_;
    std::vector<TRGSWLvl1FFT>::reverse_iterator head_;

public:
    ReversedTRGSWLvl1InputStreamFromCtxtFile(const std::string &filename);

    size_t size() const override;
    TRGSWLvl1FFT next() override;
};

class OfflineFARunner {
    // Interval for bootstrapping
    const static size_t BOOT_INTERVAL = 8000;

private:
    const Graph &graph_;
    InputStream<TRGSWLvl1FFT> &input_stream_;
    std::vector<TRLWELvl1> weight_;
    bool has_evaluated_;
    std::shared_ptr<GateKey> gate_key_;

public:
    OfflineFARunner(const Graph &graph, InputStream<TRGSWLvl1FFT> &input_stream,
                    std::shared_ptr<GateKey> gate_key = nullptr);

    TLWELvl1 result() const;
    void eval();

private:
    void next_weight(TRLWELvl1 &out, int j, Graph::State from,
                     bool input) const;
    void bootstrapping_of_weight();
};

class OnlineDFARunner {
private:
    const Graph &graph_;
    std::vector<TRLWELvl1> weight_;
    std::shared_ptr<GateKey> gate_key_;
    size_t bootstrap_interval_, num_processed_inputs_;

public:
    OnlineDFARunner(const Graph &graph,
                    std::shared_ptr<GateKey> gate_key = nullptr);

    TLWELvl1 result();
    void eval_one(const TRGSWLvl1FFT &input);

private:
    void bootstrap_weight();
};

bool between_25_75(uint32_t n);
uint32_t phase_of_TLWELvl1(const TLWELvl1 &src, const SecretKey &skey);
TRGSWLvl1FFT encrypt_bit_to_TRGSWLvl1FFT(bool b, const SecretKey &skey);

#endif
