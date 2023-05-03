#include "tfhepp_util.hpp"
#include "archive.hpp"

////////// TRGSWLvl1FFTSerializer

TRGSWLvl1FFTSerializer::TRGSWLvl1FFTSerializer(std::ostream& os) : os_(os)
{
    assert(os);

    // Assume little endian
    // FIXME: Add support for big endian
    // Thanks to:
    // https://github.com/USCiLab/cereal/blob/46a4a910077bf9e9f8327c8f6ea761c89b06da53/include/cereal/archives/portable_binary.hpp#L42
    static std::int32_t test = 1;
    assert(*reinterpret_cast<std::int8_t*>(&test) == 1);
}

void TRGSWLvl1FFTSerializer::save_binary(const void* data, size_t size)
{
    // Thanks to:
    // https://github.com/USCiLab/cereal/blob/46a4a910077bf9e9f8327c8f6ea761c89b06da53/include/cereal/archives/portable_binary.hpp#L132
    size_t written_size =
        os_.rdbuf()->sputn(reinterpret_cast<const char*>(data), size);
    assert(written_size == size);
}

void TRGSWLvl1FFTSerializer::save_double(double src)
{
    static_assert(std::numeric_limits<double>::is_iec559);
    save_binary(&src, sizeof(src));
}

void TRGSWLvl1FFTSerializer::save(const TRGSWLvl1FFT& src)
{
    for (size_t i = 0; i < 2 * Lvl1::l; i++)
        for (size_t j = 0; j < 2; j++)
            for (size_t k = 0; k < Lvl1::n; k++)
                save_double(src[i][j][k]);
}

////////// TRGSWLvl1FFTDeserializer

TRGSWLvl1FFTDeserializer::TRGSWLvl1FFTDeserializer(std::istream& is) : is_(is)
{
    assert(is);

    // Assume little endian
    // FIXME: Add support for big endian
    // Thanks to:
    // https://github.com/USCiLab/cereal/blob/46a4a910077bf9e9f8327c8f6ea761c89b06da53/include/cereal/archives/portable_binary.hpp#L42
    static std::int32_t test = 1;
    assert(*reinterpret_cast<std::int8_t*>(&test) == 1);
}

void TRGSWLvl1FFTDeserializer::load_binary(void* const data, size_t size)
{
    // Thanks to:
    // https://github.com/USCiLab/cereal/blob/master/include/cereal/archives/portable_binary.hpp#L239
    size_t read_size = is_.rdbuf()->sgetn(reinterpret_cast<char*>(data), size);
    assert(read_size == size);
}

void TRGSWLvl1FFTDeserializer::load_double(double& out)
{
    static_assert(std::numeric_limits<double>::is_iec559);
    load_binary(&out, sizeof(out));
}

size_t TRGSWLvl1FFTDeserializer::tell() const
{
    size_t org = is_.tellg();
    assert(org % BLOCK_SIZE == 0);
    return org / BLOCK_SIZE;
}

bool TRGSWLvl1FFTDeserializer::is_beg() const
{
    return is_.tellg() == is_.beg;
}

bool TRGSWLvl1FFTDeserializer::is_end() const
{
    return is_.tellg() == is_.end;
}

void TRGSWLvl1FFTDeserializer::seek(int off, std::ios_base::seekdir dir)
{
    is_.seekg(off * BLOCK_SIZE, dir);
}

void TRGSWLvl1FFTDeserializer::load(TRGSWLvl1FFT& out)
{
    for (size_t i = 0; i < 2 * Lvl1::l; i++)
        for (size_t j = 0; j < 2; j++)
            for (size_t k = 0; k < Lvl1::n; k++)
                load_double(out[i][j][k]);
}

////////// TRGSWLvl1InputStreamFromCtxtFile

TRGSWLvl1InputStreamFromCtxtFile::TRGSWLvl1InputStreamFromCtxtFile(
    const std::string& filename)
    : ifs_(filename),
      deser_(ifs_),
      pool_(1),
      running_(),
      loaded_(),
      current_size_()
{
    deser_.seek(0, std::ios_base::end);
    current_size_ = deser_.tell();
    deser_.seek(0, std::ios_base::beg);
    if (current_size_ > 0)
        start_loading_next();
}

size_t TRGSWLvl1InputStreamFromCtxtFile::size() const
{
    return current_size_;
}

void TRGSWLvl1InputStreamFromCtxtFile::start_loading_next()
{
    assert(!running_.valid());
    running_ = pool_.enqueue([this] {
        deser_.load(loaded_);
        return true;
    });
}

TRGSWLvl1FFT TRGSWLvl1InputStreamFromCtxtFile::next()
{
    assert(current_size_ > 0);
    running_.get();
    TRGSWLvl1FFT ret = loaded_;
    current_size_--;
    if (current_size_ > 0)
        start_loading_next();
    return ret;
}

////////// ReversedTRGSWLvl1InputStreamFromCtxtFile

ReversedTRGSWLvl1InputStreamFromCtxtFile::
    ReversedTRGSWLvl1InputStreamFromCtxtFile(const std::string& filename)
    : ifs_(filename), deser_(ifs_), pool_(1), current_size_(0)
{
    deser_.seek(0, std::ios_base::end);
    current_size_ = deser_.tell();
    if (!deser_.is_beg())
        start_loading_next();
}

size_t ReversedTRGSWLvl1InputStreamFromCtxtFile::size() const
{
    return current_size_;
}

void ReversedTRGSWLvl1InputStreamFromCtxtFile::start_loading_next()
{
    assert(!running_.valid());
    running_ = pool_.enqueue([this] {
        assert(!deser_.is_beg());
        deser_.seek(-1, std::ios_base::cur);
        deser_.load(loaded_);
        deser_.seek(-1, std::ios_base::cur);
        return true;
    });
}

TRGSWLvl1FFT ReversedTRGSWLvl1InputStreamFromCtxtFile::next()
{
    assert(current_size_ > 0);
    running_.get();
    TRGSWLvl1FFT ret = loaded_;
    if (!deser_.is_beg())
        start_loading_next();
    current_size_--;
    return ret;
}

//////////

TRLWELvl1 trivial_TRLWELvl1(const PolyLvl1& src)
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

TRLWELvl1 trivial_TRLWELvl1_1over2()
{
    TRLWELvl1 ret = trivial_TRLWELvl1_zero();
    ret[1][0] = (1u << 31);  // 1/2
    return ret;
}

void TLWELvl0_add(TLWELvl0& lhs, const TLWELvl0& rhs)
{
    for (size_t i = 0; i <= Lvl0::n; i++)
        lhs[i] += rhs[i];
}

void TLWELvl1_add(TLWELvl1& lhs, const TLWELvl1& rhs)
{
    for (size_t i = 0; i <= Lvl1::n; i++)
        lhs[i] += rhs[i];
}

// out += src
void TRLWELvl1_add(TRLWELvl1& out, const TRLWELvl1& src)
{
    for (size_t i = 0; i < Lvl1::n; i++) {
        out[0][i] += src[0][i];
        out[1][i] += src[1][i];
    }
}

namespace {
void PolyLvl1_mult_X_k(PolyLvl1& out, const PolyLvl1& src, size_t k)
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
void TRLWELvl1_mult_X_k(TRLWELvl1& out, const TRLWELvl1& src, size_t k)
{
    PolyLvl1_mult_X_k(out[0], src[0], k);
    PolyLvl1_mult_X_k(out[1], src[1], k);
}

uint32_t phase_of_TLWELvl1(const TLWELvl1& src, const SecretKey& skey)
{
    uint32_t phase = src[Lvl1::n];
    for (size_t i = 0; i < Lvl1::n; i++)
        phase -= src[i] * skey.key.lvl1[i];
    return phase;
}

PolyLvl1 phase_of_TRLWELvl1(const TRLWELvl1& src, const SecretKey& skey)
{
    PolyLvl1 as;
    TFHEpp::PolyMul<Lvl1>(as, src[0], skey.key.lvl1);
    PolyLvl1 phase = src[1];
    for (size_t i = 0; i < Lvl1::n; i++)
        phase[i] -= as[i];
    return phase;
}

// w = w |> SEI |> IKS(gk) |> GateBootstrappingTLWE2TRLWE(gk)
void do_SEI_IKS_GBTLWE2TRLWE(TRLWELvl1& w, const EvalKey& ek)
{
    TLWELvl1 tlwel1;
    TFHEpp::SampleExtractIndex<Lvl1>(tlwel1, w, 0);
    TLWELvl0 tlwel0;
    TFHEpp::IdentityKeySwitch<TFHEpp::lvl10param>(
        tlwel0, tlwel1, ek.getiksk<TFHEpp::lvl10param>());
    TFHEpp::BlindRotate<TFHEpp::lvl01param>(
        w, tlwel0, ek.getbkfft<TFHEpp::lvl01param>(),
        TFHEpp::μpolygen<TFHEpp::lvl1param, TFHEpp::lvl1param::μ>());
}

// BootstrappingTLWE-to-TRLWE
// {0, 1/2} -> {0, 1/2}
// NOTE: src is MODIFIED for efficiency!
void BS_TLWE_0_1o2_to_TRLWE_0_1o2(TRLWELvl1& out, TLWELvl0& src,
                                  const EvalKey& ek)
{
    using namespace TFHEpp;

    // Convert {0, 1/2} to {-1/4, 1/4}
    src[Lvl0::n] -= (1 << 30);  // 1/4
    // Bootstrapping without changing plaintext space
    BlindRotate<lvl01param>(out, src, ek.getbkfft<lvl01param>(),
                            μpolygen<Lvl1, (1 << 30) /* 1/4 */>());
    // Convert {-1/4, 1/4} to {0, 1/2}
    out[1][0] += (1 << 30);  // 1/4
}

// BootstrappingTLWE-to-TRLWE
// {0, 1/2} -> {-1/8, 1/8}
// NOTE: src is MODIFIED for efficiency!
void BS_TLWE_0_1o2_to_TRLWE_m1o8_1o8(TRLWELvl1& out, TLWELvl0& src,
                                     const EvalKey& ek)
{
    using namespace TFHEpp;

    // Convert {0, 1/2} to {-1/4, 1/4}
    src[Lvl0::n] -= (1 << 30);  // 1/4
    // Bootstrapping and convert {-1/4, 1/4} to {-1/8, 1/8}
    BlindRotate<lvl01param>(out, src, ek.getbkfft<lvl01param>(),
                            μpolygen<Lvl1, (1 << 29) /* 1/8 */>());
}

// w = w |> SEI |> IKS(gk) |> GateBootstrappingTLWE2TRLWE(gk)
// {0, 1/2} -> {0, 1/2}
void do_SEI_IKS_GBTLWE2TRLWE_2(TRLWELvl1& w, const EvalKey& ek)
{
    using namespace TFHEpp;

    TLWELvl1 tlwel1;
    SampleExtractIndex<Lvl1>(tlwel1, w, 0);

    TLWELvl0 tlwel0;
    IdentityKeySwitch<lvl10param>(tlwel0, tlwel1, ek.getiksk<lvl10param>());

    BS_TLWE_0_1o2_to_TRLWE_0_1o2(w, tlwel0, ek);
}

// w = w |> SEI |> IKS(gk) |> GateBootstrappingTLWE2TRLWE(gk)
// {0, 1/2} -> {-1/8, 1/8}
void do_SEI_IKS_GBTLWE2TRLWE_3(TRLWELvl1& w, const EvalKey& ek)
{
    using namespace TFHEpp;

    TLWELvl1 tlwel1;
    SampleExtractIndex<Lvl1>(tlwel1, w, 0);

    TLWELvl0 tlwel0;
    IdentityKeySwitch<lvl10param>(tlwel0, tlwel1, ek.getiksk<lvl10param>());
    BS_TLWE_0_1o2_to_TRLWE_m1o8_1o8(w, tlwel0, ek);
}

TRGSWLvl1FFT encrypt_bit_to_TRGSWLvl1FFT(bool b, const SecretKey& skey)
{
    return TFHEpp::trgswfftSymEncrypt<Lvl1>({b}, Lvl1::α, skey.key.lvl1);
}

bool decrypt_TLWELvl1_to_bit(const TLWELvl1& c, const SecretKey& skey)
{
    // Use {0, 1/2} as message space
    uint32_t phase = c[Lvl1::n];
    for (size_t i = 0; i < Lvl1::n; i++)
        phase -= c[i] * skey.key.lvl1[i];
    return (phase + (1u << 30 /* 1/4 */)) > (1u << 31 /* 1/2 */);
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

void dump_weight(std::ostream& os, const PolyLvl1& w)
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

std::string weight2bitstring(const PolyLvl1& w)
{
    std::stringstream ss;
    dump_weight(ss, w);
    return ss.str();
}

void CircuitBootstrappingFFTLvl11(TRGSWLvl1FFT& out, const TLWELvl1& src,
                                  const EvalKey& ek)
{
    TFHEpp::CircuitBootstrappingFFT<TFHEpp::lvl10param, TFHEpp::lvl02param,
                                    TFHEpp::lvl21param>(out, src, ek);
}

void HomXORwoSE(TRLWELvl1& out, const TLWELvl0& lhs, const TLWELvl0& rhs,
                const EvalKey& ek)
{
    TLWELvl0 temp;
    for (int i = 0; i <= Lvl0::n; i++)
        temp[i] = 2 * lhs[i] + 2 * rhs[i];
    temp[Lvl0::n] += 2 * Lvl0::μ;
    TFHEpp::BlindRotate<TFHEpp::lvl01param>(
        out, temp, ek.getbkfft<TFHEpp::lvl01param>(),
        TFHEpp::μpolygen<TFHEpp::lvl1param, TFHEpp::lvl1param::μ>());
}
