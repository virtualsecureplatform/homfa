#include "tfhepp_util.hpp"
#include "archive.hpp"

TRGSWLvl1InputStreamFromCtxtFile::TRGSWLvl1InputStreamFromCtxtFile(
    const std::string &filename)
{
    std::ifstream ifs{filename};
    assert(ifs);
    data_ = read_from_archive<std::vector<TRGSWLvl1FFT>>(filename);
    head_ = data_.begin();
}

size_t TRGSWLvl1InputStreamFromCtxtFile::size() const
{
    return data_.end() - head_;
}

TRGSWLvl1FFT TRGSWLvl1InputStreamFromCtxtFile::next()
{
    assert(size() != 0);
    return *(head_++);
}

ReversedTRGSWLvl1InputStreamFromCtxtFile::
    ReversedTRGSWLvl1InputStreamFromCtxtFile(const std::string &filename)
{
    std::ifstream ifs{filename};
    assert(ifs);
    data_ = read_from_archive<std::vector<TRGSWLvl1FFT>>(filename);
    head_ = data_.rbegin();
}

size_t ReversedTRGSWLvl1InputStreamFromCtxtFile::size() const
{
    return data_.rend() - head_;
}

TRGSWLvl1FFT ReversedTRGSWLvl1InputStreamFromCtxtFile::next()
{
    assert(size() != 0);
    return *(head_++);
}

TRLWELvl1 trivial_TRLWELvl1(const PolyLvl1 &src)
{
    TRLWELvl1 ret = {};
    ret[1] = src;
    return ret;
}

TRLWELvl1 trivial_TRLWELvl1_zero()
{
    TRLWELvl1 ret = {};
    return ret;
}

TLWELvl0 trivial_TLWELvl0_minus_1over8()
{
    TLWELvl0 ret = {};
    ret[Lvl0::n] = -(1u << 29);  // -1/8
    return ret;
}

TLWELvl0 trivial_TLWELvl0_1over8()
{
    TLWELvl0 ret = {};
    ret[Lvl0::n] = (1u << 29);  // 1/8
    return ret;
}

TLWELvl1 trivial_TLWELvl1_minus_1over8()
{
    TLWELvl1 ret = {};
    ret[Lvl1::n] = -(1u << 29);  // -1/8
    return ret;
}

TLWELvl1 trivial_TLWELvl1_1over8()
{
    TLWELvl1 ret = {};
    ret[Lvl1::n] = (1u << 29);  // 1/8
    return ret;
}

TRLWELvl1 trivial_TRLWELvl1_minus_1over8()
{
    TRLWELvl1 ret = trivial_TRLWELvl1_zero();
    ret[1][0] = -(1u << 29);  // -1/8
    return ret;
}

TRLWELvl1 trivial_TRLWELvl1_1over8()
{
    TRLWELvl1 ret = trivial_TRLWELvl1_zero();
    ret[1][0] = (1u << 29);  // 1/8
    return ret;
}

void TLWELvl0_add(TLWELvl0 &lhs, const TLWELvl0 &rhs)
{
    for (size_t i = 0; i <= Lvl0::n; i++)
        lhs[i] += rhs[i];
}

void TLWELvl1_add(TLWELvl1 &lhs, const TLWELvl1 &rhs)
{
    for (size_t i = 0; i <= Lvl1::n; i++)
        lhs[i] += rhs[i];
}

// out += src
void TRLWELvl1_add(TRLWELvl1 &out, const TRLWELvl1 &src)
{
    for (size_t i = 0; i < Lvl1::n; i++) {
        out[0][i] += src[0][i];
        out[1][i] += src[1][i];
    }
}

namespace {
void PolyLvl1_mult_X_k(PolyLvl1 &out, const PolyLvl1 &src, size_t k)
{
    constexpr size_t n = Lvl1::n;

    if (k == 0) {
        out = src;
    }
    else if (k < n) {
        for (size_t i = 0; i < k; i++)
            out[i] = -src[i - k + n];
        for (size_t i = 0; i < n - k; i++)
            out[i + k] = src[i];
    }
    else {
        const size_t k2 = k - n;
        for (size_t i = 0; i < k2; i++)
            out[i] = src[i - k2 + n];
        for (size_t i = 0; i < n - k2; i++)
            out[i + k2] = -src[i];
    }
}
}  // namespace

// out = src * X^k
void TRLWELvl1_mult_X_k(TRLWELvl1 &out, const TRLWELvl1 &src, size_t k)
{
    PolyLvl1_mult_X_k(out[0], src[0], k);
    PolyLvl1_mult_X_k(out[1], src[1], k);
}

uint32_t phase_of_TLWELvl1(const TLWELvl1 &src, const SecretKey &skey)
{
    uint32_t phase = src[Lvl1::n];
    for (size_t i = 0; i < Lvl1::n; i++)
        phase -= src[i] * skey.key.lvl1[i];
    return phase;
}

PolyLvl1 phase_of_TRLWELvl1(const TRLWELvl1 &src, const SecretKey &skey)
{
    PolyLvl1 as;
    TFHEpp::PolyMul<Lvl1>(as, src[0], skey.key.lvl1);
    PolyLvl1 phase = src[1];
    for (size_t i = 0; i < Lvl1::n; i++)
        phase[i] -= as[i];
    return phase;
}

// w = w |> SEI |> IKS(gk) |> GateBootstrappingTLWE2TRLWE(gk)
void do_SEI_IKS_GBTLWE2TRLWE(TRLWELvl1 &w, const GateKey &gk)
{
    TLWELvl1 tlwel1;
    TFHEpp::SampleExtractIndex<Lvl1>(tlwel1, w, 0);
    TLWELvl0 tlwel0;
    TFHEpp::IdentityKeySwitch<TFHEpp::lvl10param>(tlwel0, tlwel1, gk.ksk);
    TFHEpp::GateBootstrappingTLWE2TRLWEFFT<TFHEpp::lvl01param>(w, tlwel0,
                                                               gk.bkfftlvl01);
}

TRGSWLvl1FFT encrypt_bit_to_TRGSWLvl1FFT(bool b, const SecretKey &skey)
{
    return TFHEpp::trgswfftSymEncrypt<Lvl1>({b}, Lvl1::α, skey.key.lvl1);
}

PolyLvl1 uint2weight(uint64_t n)
{
    PolyLvl1 w;
    const uint32_t mu = 1u << 31;
    for (size_t i = 0; i < Lvl1::n; i++)
        if (i < 64)
            w[i] = ((n >> i) & 1u) ? mu : 0;
        else
            w[i] = 0;
    return w;
}

bool between_25_75(uint32_t n)
{
    const uint32_t mu25 = 1u << 30, mu75 = (1u << 30) + (1u << 31);
    return mu25 <= n && n <= mu75;
}

void dump_weight(std::ostream &os, const PolyLvl1 &w)
{
    for (size_t i = 0; i < Lvl1::n; i++) {
        if (i % 32 == 0)
            os << "\n";
        else if (i % 8 == 0)
            os << " ";
        if (between_25_75(w[i]))
            os << 1;
        else
            os << 0;
    }
    os << "\n";
}

std::string weight2bitstring(const PolyLvl1 &w)
{
    std::stringstream ss;
    dump_weight(ss, w);
    return ss.str();
}
