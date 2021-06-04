#include "offline_dfa.hpp"

#include <execution>
#include <queue>

#include <spdlog/spdlog.h>

/* OfflineDFARunner */

OfflineDFARunner::OfflineDFARunner(const Graph &graph,
                                   InputStream<TRGSWLvl1FFT> &input_stream,
                                   std::shared_ptr<GateKeyFFT> gate_key)
    : graph_(graph),
      input_stream_(input_stream),
      weight_(graph_.size(), trivial_TRLWELvl1_zero()),
      has_evaluated_(false),
      gate_key_(std::move(gate_key))
{
    for (Graph::State st = 0; st < graph_.size(); st++)
        if (graph_.is_final_state(st))
            weight_.at(st)[1][0] = (1u << 29);  // 1/8
        else
            weight_.at(st)[1][0] = -(1u << 29);  // -1/8

    spdlog::info("Parameter:");
    spdlog::info("\tMode:\t{}", "Offline FA Runner");
    spdlog::info("\tInput size:\t{}", input_stream.size());
    spdlog::info("\tState size:\t{}", graph_.size());
    spdlog::info("\tWeight size:\t{}", weight_.size());
    spdlog::info("\tConcurrency:\t{}", std::thread::hardware_concurrency());

    {
        size_t total_cnt_cmux = 0;
        for (size_t j = 0; j < input_stream_.size(); j++)
            total_cnt_cmux += graph_.states_at_depth(j).size();
        spdlog::info("\tTotal #CMUX:\t{}", total_cnt_cmux);
    }

    spdlog::info("");
}

TLWELvl1 OfflineDFARunner::result() const
{
    assert(has_evaluated_);
    TRLWELvl1 w = weight_.at(graph_.initial_state());
    if (gate_key_)
        do_SEI_IKS_GBTLWE2TRLWE(w, *gate_key_);
    TLWELvl1 ret;
    TFHEpp::SampleExtractIndex<Lvl1>(ret, w, 0);
    return ret;
}

void OfflineDFARunner::eval()
{
    assert(!has_evaluated_);
    has_evaluated_ = true;

    size_t input_size = input_stream_.size();
    std::vector<TRLWELvl1> out(weight_.size(), trivial_TRLWELvl1_zero());
    for (int j = input_size - 1; j >= 0; --j) {
        auto states = graph_.states_at_depth(j);
        TRGSWLvl1FFT input;
        input_stream_.next(input);
        std::for_each(std::execution::par, states.begin(), states.end(),
                      [&](auto &&q) {
                          TRLWELvl1 w0, w1;
                          next_weight(w1, j, q, true);
                          next_weight(w0, j, q, false);
                          TFHEpp::CMUXFFT<Lvl1>(out.at(q), input, w1, w0);
                      });
        {
            using std::swap;
            swap(out, weight_);
        }

        if (size_t cur = input_size - j;
            gate_key_ && cur != 0 && cur % BOOT_INTERVAL == 0) {
            spdlog::info("Bootstrapping occurred");
            bootstrap_weight();
        }

        spdlog::debug("[{}] #CMUX : {}", j, states.size());
    }
}

void OfflineDFARunner::next_weight(TRLWELvl1 &out, int j, Graph::State from,
                                   bool input) const
{
    Graph::State to = graph_.next_state(from, input);
    out = weight_.at(to);
}

void OfflineDFARunner::bootstrap_weight()
{
    assert(gate_key_);
    std::for_each(
        std::execution::par, weight_.begin(), weight_.end(),
        [&](TRLWELvl1 &w) { do_SEI_IKS_GBTLWE2TRLWE(w, *gate_key_); });
}

/* GPUOfflineDFARunner */

class CUDARunner {
    struct Node {
        size_t input, state, timeToReady;
        std::vector<Node *> out;

        Node() : input(-1), state(-1), timeToReady(-1), out()
        {
        }
    };

    struct Stream {
        cufhe::Stream st;
        bool running;
        Node *n;  // Running node

        Stream() : st(), running(false), n(nullptr)
        {
            st.Create();
        }
        ~Stream()
        {
            st.Destroy();
        }

    private:
        // Make Stream un-copyable
        Stream(const Stream &) = delete;
        Stream &operator=(const Stream &) = delete;
    };

private:
    std::vector<std::shared_ptr<cufhe::cuFHETRLWElvl1>> weight_;
    std::vector<Node> nodes_;
    std::vector<Stream> streams_;
    std::queue<Node *> runnable_;
    std::vector<cufhe::cuFHETRGSWNTTlvl1> in_;
    const Graph &graph_;
    size_t current_in_size_, num_gates_to_finish_;

public:
    CUDARunner(size_t in_block_size, const Graph &graph)
        : weight_((in_block_size + 1) * graph.size()),
          nodes_(in_block_size * graph.size()),
          streams_(800),
          runnable_(),
          in_(in_block_size),
          graph_(graph),
          current_in_size_(0)
    {
        for (auto &&w : weight_)
            w = std::make_shared<cufhe::cuFHETRLWElvl1>();

        for (size_t i = 0; i < in_.size(); i++) {
            for (Graph::State q : graph.all_states()) {
                auto &n = nodes_.at(i * graph.size() + q);
                n.input = i;
                n.state = q;
            }
        }
    }

    cufhe::cuFHETRLWElvl1 &result()
    {
        cufhe::cuFHETRLWElvl1 &w = *weight_.at(
            current_in_size_ * graph_.size() + graph_.initial_state());
        cufhe::TRLWElvl1CopyD2H(w, 0);
        cufhe::Synchronize();
        return w;
    }

    void make_ouroboros(CUDARunner &that)
    {
        for (size_t i = 0; i < graph_.size(); i++) {
            weight_.at(i) = that.weight_.at(in_.size() * graph_.size() + i);
            that.weight_.at(i) = weight_.at(in_.size() * graph_.size() + i);
        }
    }

    void set_initial_weight()
    {
        for (Graph::State st : graph_.all_states()) {
            weight_.at(st)->trlwehost = graph_.is_final_state(st)
                                            ? trivial_TRLWELvl1_1over8()
                                            : trivial_TRLWELvl1_minus_1over8();
            cufhe::TRLWElvl1CopyH2D(*weight_.at(st), 0);
        }
        cufhe::Synchronize();
    }

    void prepare_next_run(InputStream<TRGSWLvl1NTT> &in_stream)
    {
        assert(runnable_.size() == 0);

        for (auto &&n : nodes_) {
            n.out.clear();
            n.timeToReady = 2;
        }

        current_in_size_ = 0;
        num_gates_to_finish_ = 0;
        for (size_t i = 0; i < in_.size(); i++, current_in_size_++) {
            if (in_stream.size() == 0)
                break;
            int j = in_stream.size() - 1;

            in_stream.next(in_.at(i).trgswhost);
            cufhe::TRGSWNTTlvl1CopyH2D(in_.at(i), streams_.at(0).st);

            for (Graph::State q : graph_.states_at_depth(j)) {
                num_gates_to_finish_++;
                Node &n = nodes_.at(i * graph_.size() + q);

                if (i == 0) {
                    runnable_.push(&n);
                    continue;
                }

                Graph::State q0 = graph_.next_state(q, false),
                             q1 = graph_.next_state(q, true);
                Node &n0 = nodes_.at((i - 1) * graph_.size() + q0),
                     &n1 = nodes_.at((i - 1) * graph_.size() + q1);
                n0.out.push_back(&n);
                n1.out.push_back(&n);
            }
        }

        cudaStreamSynchronize(streams_.at(0).st.st());
    }

    void run()
    {
        while (num_gates_to_finish_ != 0) {
            for (Stream &st : streams_) {
                if (st.running) {
                    if (!cufhe::StreamQuery(st.st))
                        continue;

                    for (Node *n : st.n->out) {
                        if (--n->timeToReady != 0)
                            continue;
                        runnable_.push(n);
                    }

                    num_gates_to_finish_--;
                    st.n = nullptr;
                    st.running = false;
                }
                else if (!runnable_.empty()) {
                    Node *n = runnable_.front();
                    runnable_.pop();

                    Graph::State q0 = graph_.next_state(n->state, false),
                                 q1 = graph_.next_state(n->state, true);
                    size_t in0_index = n->input * graph_.size() + q0,
                           in1_index = n->input * graph_.size() + q1,
                           out_index =
                               (n->input + 1) * graph_.size() + n->state;
                    cufhe::gCMUXNTT(*weight_.at(out_index), in_.at(n->input),
                                    *weight_.at(in1_index),
                                    *weight_.at(in0_index), st.st);

                    st.n = n;
                    st.running = true;
                }
            }
        }
    }
};

class CUDAGraphBuilder {
    friend class CUDAGraph;

public:
    using HostFunc = std::function<void()>;

private:
    cudaGraph_t graph_;
    std::vector<std::shared_ptr<HostFunc>> hostFuncInsts_;
    std::set<std::pair<cudaGraphNode_t, cudaGraphNode_t>> edges_;

private:
    static void handlerHostNode(void *userData)
    {
        HostFunc *body = static_cast<HostFunc *>(userData);
        (*body)();
    }

public:
    CUDAGraphBuilder()
    {
        cudaGraphCreate(&graph_, 0);
    }

    ~CUDAGraphBuilder()
    {
        cudaGraphDestroy(graph_);
    }

    void addDep(cudaGraphNode_t from, cudaGraphNode_t to)
    {
        if (edges_.contains(std::make_pair(from, to)))
            return;
        cudaGraphAddDependencies(graph_, &from, &to, 1);
        assert(cudaGetLastError() == cudaSuccess);
        edges_.emplace(from, to);
    }

    template <class Fun>
    cudaGraphNode_t addKernel(const std::vector<cudaGraphNode_t> &deps, Fun fun)
    {
        cufhe::Stream stream;
        stream.Create();

        cudaGraph_t emb_graph;
        cudaStreamBeginCapture(stream.st(), cudaStreamCaptureModeGlobal);
        fun(stream);
        cudaStreamEndCapture(stream.st(), &emb_graph);
        cudaGraphNode_t node;
        cudaGraphAddChildGraphNode(&node, graph_, deps.data(), deps.size(),
                                   emb_graph);

        stream.Destroy();
        return node;
    }

    template <class Fun>
    cudaGraphNode_t addKernel(Fun &&fun)
    {
        return addKernel({}, std::forward<Fun>(fun));
    }

    cudaGraphNode_t addHost(const std::vector<cudaGraphNode_t> &deps,
                            const std::function<void()> &fun)
    {
        cudaGraphNode_t node;
        auto funptr = std::make_shared<HostFunc>(fun);
        hostFuncInsts_.push_back(funptr);
        const cudaHostNodeParams param = {handlerHostNode,
                                          static_cast<void *>(funptr.get())};
        cudaGraphAddHostNode(&node, graph_, deps.data(), deps.size(), &param);
        return node;
    }

    template <class... Args>
    cudaGraphNode_t addHost(Args &&... args)
    {
        return addHost({}, std::forward<Args>(args)...);
    }

    cudaGraphExec_t instantiate()
    {
        cudaGraphExec_t instance;
        cudaGraphInstantiate(&instance, graph_, NULL, NULL, 0);
        return instance;
    }

private:
    // Make CUDAGraphBuilder un-copyable
    CUDAGraphBuilder(const CUDAGraphBuilder &) = delete;
    CUDAGraphBuilder &operator=(const CUDAGraphBuilder &) = delete;
};

class CUDAGraph {
private:
    cudaGraphExec_t instance_;
    std::vector<std::shared_ptr<CUDAGraphBuilder::HostFunc>> hostFuncInsts_;

public:
    CUDAGraph(CUDAGraphBuilder &&b)
        : hostFuncInsts_(std::move(b.hostFuncInsts_))
    {
        cudaGraphInstantiate(&instance_, b.graph_, NULL, NULL, 0);
    }

    ~CUDAGraph()
    {
        cudaGraphExecDestroy(instance_);
    }

    void runAsync(cudaStream_t st)
    {
        cudaGraphLaunch(instance_, st);
        assert(cudaGetLastError() == cudaSuccess);
    }

    void runSync(cudaStream_t st)
    {
        cudaGraphLaunch(instance_, st);
        assert(cudaGetLastError() == cudaSuccess);
        cudaStreamSynchronize(st);
    }

private:
    // Make CUDAGraph un-copyable
    CUDAGraph(const CUDAGraph &) = delete;
    CUDAGraph &operator=(const CUDAGraph &) = delete;
};

class GPUCMUXBlock {
private:
    std::vector<std::shared_ptr<cufhe::cuFHETRLWElvl1>> out_;
    std::vector<cufhe::cuFHETRGSWNTTlvl1> in_;
    const Graph &graph_;
    std::vector<cudaGraphNode_t> node_;
    size_t current_in_size_;
    std::shared_ptr<CUDAGraph> cgraph_;
    cufhe::Stream stream_;

public:
    GPUCMUXBlock(size_t in_block_size, const Graph &graph)
        : out_((in_block_size + 1) * graph.size()),
          in_(in_block_size),
          graph_(graph),
          node_(in_block_size * graph.size()),
          current_in_size_(0),
          cgraph_(nullptr),
          stream_()
    {
        stream_.Create();
        for (auto &&o : out_)
            o = std::make_shared<cufhe::cuFHETRLWElvl1>();
    }

    void run_async()
    {
        assert(cgraph_);
        cgraph_->runAsync(stream_.st());
    }

    void synchronize()
    {
        cudaStreamSynchronize(stream_.st());
    }

    cufhe::cuFHETRLWElvl1 &result()
    {
        return *out_.at(current_in_size_ * graph_.size() +
                        graph_.initial_state());
    }

    void make_ouroboros(GPUCMUXBlock &that)
    {
        for (size_t i = 0; i < graph_.size(); i++) {
            out_.at(i) = that.out_.at(in_.size() * graph_.size() + i);
            that.out_.at(i) = out_.at(in_.size() * graph_.size() + i);
        }
    }

    void set_initial_weight()
    {
        for (Graph::State st : graph_.all_states()) {
            out_.at(st)->trlwehost = graph_.is_final_state(st)
                                         ? trivial_TRLWELvl1_1over8()
                                         : trivial_TRLWELvl1_minus_1over8();
        }
    }

    void make_cgraph()
    {
        assert(cgraph_ == nullptr);

        CUDAGraphBuilder bld;
        for (size_t i = 0; i < in_.size(); i++) {
            for (Graph::State q : graph_.all_states()) {
                Graph::State q0 = graph_.next_state(q, false),
                             q1 = graph_.next_state(q, true);
                cudaGraphNode_t n = bld.addKernel([&](auto &&s) {
                    size_t in0_index = i * graph_.size() + q0,
                           in1_index = i * graph_.size() + q1,
                           out_index = (i + 1) * graph_.size() + q;
                    cufhe::CMUXNTT(*out_.at(out_index), in_.at(i),
                                   *out_.at(in1_index), *out_.at(in0_index), s);
                });
                node_.at(i * graph_.size() + q) = n;
                if (i != 0) {
                    bld.addDep(node_.at((i - 1) * graph_.size() + q0), n);
                    bld.addDep(node_.at((i - 1) * graph_.size() + q1), n);
                }
            }
        }
        cgraph_ = std::make_shared<CUDAGraph>(std::move(bld));
    }

    void read_input(int initial_j, InputStream<TRGSWLvl1NTT> &in_stream)
    {
        current_in_size_ = 0;
        for (size_t i = 0; i < in_.size(); i++, current_in_size_++) {
            int j = initial_j - i;
            if (j < 0)
                return;

            // Get input
            in_stream.next(in_.at(i).trgswhost);
        }
    }
};

GPUOfflineDFARunner::GPUOfflineDFARunner(
    const Graph &graph, InputStream<TRGSWLvl1NTT> &input_stream,
    bool can_bootstrap)
    : graph_(graph),
      input_stream_(input_stream),
      can_bootstrap_(can_bootstrap),
      res_(),
      has_evaluated_(false)
{
    spdlog::info("Parameter:");
    spdlog::info("\tMode:\t{}", "GPU Offline FA Runner");
    spdlog::info("\tInput size:\t{}", input_stream.size());
    spdlog::info("\tState size:\t{}", graph_.size());

    {
        size_t total_cnt_cmux = 0;
        for (size_t j = 0; j < input_stream_.size(); j++)
            total_cnt_cmux += graph_.states_at_depth(j).size();
        spdlog::info("\tTotal #CMUX:\t{}", total_cnt_cmux);
    }

    spdlog::info("");
}

TLWELvl1 GPUOfflineDFARunner::result() const
{
    assert(has_evaluated_);
    return res_;
}

void GPUOfflineDFARunner::eval()
{
    assert(!has_evaluated_);

    std::vector<cufhe::Stream> sts(1 << 9);
    for (auto &&st : sts)
        st.Create();
    const size_t mask = (1 << 9) - 1;
    size_t next_st_index = 0;
    auto next_st = [&next_st_index, &sts, mask]() -> cufhe::Stream & {
        return sts.at(next_st_index++ & mask);
    };

    std::vector<cufhe::cuFHETRLWElvl1> out1(graph_.size()), out2(graph_.size());
    for (Graph::State q : graph_.all_states()) {
        out1.at(q).trlwehost = graph_.is_final_state(q)
                                   ? trivial_TRLWELvl1_1over8()
                                   : trivial_TRLWELvl1_minus_1over8();
        cufhe::TRLWElvl1CopyH2D(out1.at(q), next_st());
    }
    cufhe::Synchronize();

    cufhe::cuFHETRGSWNTTlvl1 in;
    size_t input_size = input_stream_.size();
    for (int j = input_size - 1; j >= 0; --j) {
        input_stream_.next(in.trgswhost);
        cufhe::TRGSWNTTlvl1CopyH2D(in, next_st());
        cufhe::Synchronize();
        auto states = graph_.states_at_depth(j);
        for (Graph::State q : states) {
            Graph::State q0 = graph_.next_state(q, false),
                         q1 = graph_.next_state(q, true);
            cufhe::gCMUXNTT(out2.at(q), in, out1.at(q1), out1.at(q0),
                            next_st());
        }
        {
            using std::swap;
            swap(out1, out2);
        }
        cufhe::Synchronize();
        // spdlog::debug("CMUX {}", j);
    }

    cufhe::cuFHETRLWElvl1 &w = out1.at(graph_.initial_state());
    cufhe::TRLWElvl1CopyD2H(w, 0);
    cufhe::Synchronize();
    TFHEpp::SampleExtractIndex<Lvl1>(res_, w.trlwehost, 0);

    /*
    auto run0 = std::make_shared<CUDARunner>(3, graph_);
    auto run1 = std::make_shared<CUDARunner>(3, graph_);
    run0->make_ouroboros(*run1);
    run0->set_initial_weight();
    run0->prepare_next_run(input_stream_);
    while (input_stream_.size() != 0) {
        run0->run();
        run1->prepare_next_run(input_stream_);

        using std::swap;
        swap(run0, run1);
    }
    run0->run();
    TFHEpp::SampleExtractIndex<Lvl1>(res_, run0->result().trlwehost, 0);
    */

    /*
    auto blk0 = std::make_shared<GPUCMUXBlock>(1, graph_);
    auto blk1 = std::make_shared<GPUCMUXBlock>(1, graph_);
    blk0->make_ouroboros(*blk1);
    blk0->make_cgraph();
    blk1->make_cgraph();

    blk0->set_initial_weight();
    blk0->read_input(input_stream_.size() - 1, input_stream_);
    spdlog::debug("start");
    while (input_stream_.size() != 0) {
        blk0->run_async();
        blk1->read_input(input_stream_.size() - 1, input_stream_);
        blk0->synchronize();

        using std::swap;
        swap(blk0, blk1);
    }
    blk0->run_async();
    blk0->synchronize();
    spdlog::debug("done");
    */

    // if (can_bootstrap_) {
    //    cufhe::Ctxt tmp1;
    //    cufhe::cuFHETRLWElvl1 tmp2;
    //    cufhe::SampleExtractAndKeySwitch(tmp1, blk0->result(), 0);
    //    cufhe::GateBootstrappingTLWE2TRLWElvl01NTT(tmp2, tmp1, 0);
    //    cufhe::Synchronize();
    //    TFHEpp::SampleExtractIndex<Lvl1>(res_, tmp2.trlwehost, 0);
    //}
    // else {
    // TFHEpp::SampleExtractIndex<Lvl1>(res_, blk0->result().trlwehost, 0);
    //}

    has_evaluated_ = true;
}
