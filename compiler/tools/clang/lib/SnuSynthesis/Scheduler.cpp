/*****************************************************************************/
/*                                                                           */
/* Copyright (c) 2020 Seoul National University.                             */
/* All rights reserved.                                                      */
/*                                                                           */
/* Redistribution and use in source and binary forms, with or without        */
/* modification, are permitted provided that the following conditions        */
/* are met:                                                                  */
/*   1. Redistributions of source code must retain the above copyright       */
/*      notice, this list of conditions and the following disclaimer.        */
/*   2. Redistributions in binary form must reproduce the above copyright    */
/*      notice, this list of conditions and the following disclaimer in the  */
/*      documentation and/or other materials provided with the distribution. */
/*   3. Neither the name of Seoul National University nor the names of its   */
/*      contributors may be used to endorse or promote products derived      */
/*      from this software without specific prior written permission.        */
/*                                                                           */
/* THIS SOFTWARE IS PROVIDED BY SEOUL NATIONAL UNIVERSITY "AS IS" AND ANY    */
/* EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED */
/* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE    */
/* DISCLAIMED. IN NO EVENT SHALL SEOUL NATIONAL UNIVERSITY BE LIABLE FOR ANY */
/* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL        */
/* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS   */
/* OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)     */
/* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       */
/* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  */
/* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           */
/* POSSIBILITY OF SUCH DAMAGE.                                               */
/*                                                                           */
/* Contact information:                                                      */
/*   Center for Manycore Programming                                         */
/*   Department of Computer Science and Engineering                          */
/*   Seoul National University, Seoul 08826, Korea                           */
/*   http://aces.snu.ac.kr                                                   */
/*                                                                           */
/* Contributors:                                                             */
/*   Gangwon Jo, Heehoon Kim, Jeesoo Lee, and Jaejin Lee                     */
/*                                                                           */
/*****************************************************************************/

#include "clang/SnuSynthesis/Scheduler.h"
#include "clang/Basic/LLVM.h"
#include "clang/SnuSynthesis/DataflowGraph.h"
#include "clang/SnuSynthesis/PlatformContext.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/DenseSet.h"
#include "llvm/ADT/SmallVector.h"
#include <algorithm>
#include <lpsolve/lp_lib.h>
#include <vector>

namespace clang {

namespace snu {

// SchedulerBase

void SchedulerBase::ScheduleAll(ControlDataflowGraph *Graph) {
  for (ControlDataflowGraph::iterator I = Graph->begin(), E = Graph->end();
       I != E; ++I) {
    if (DataflowGraph *Graph = *I) {
      Schedule(Graph);
    }
  }
}

unsigned SchedulerBase::GetTstart(DFGNode *Node) const {
  assert(Tstart.count(Node) && "Tstart[Node] is not yet evaluated");
  return Tstart.lookup(Node);
}

unsigned SchedulerBase::GetTend(DFGNode *Node) const {
  assert(Tend.count(Node) && "Tend[Node] is not yet evaluated");
  return Tend.lookup(Node);
}

unsigned SchedulerBase::GetMaxPredTend(DFGNode *Node) const {
  unsigned MaxPredTend = 0;
  for (DFGNode::pred_iterator P = Node->pred_begin(), PEnd = Node->pred_end();
       P != PEnd; ++P) {
    MaxPredTend = std::max(MaxPredTend, GetTend(*P));
  }
  return MaxPredTend;
}

unsigned SchedulerBase::GetMinSuccTstart(DFGNode *Node) const {
  unsigned MinSuccTstart = (unsigned)-1;
  for (DFGNode::succ_iterator S = Node->succ_begin(), SEnd = Node->succ_end();
       S != SEnd; ++S) {
    MinSuccTstart = std::min(MinSuccTstart, GetTstart(*S));
  }
  assert(MinSuccTstart != (unsigned)-1);
  return MinSuccTstart;
}

void SchedulerBase::DelayNode(DFGNode *Node, unsigned Shift) {
  Tstart[Node] = GetTstart(Node) + Shift;
  Tend[Node] = GetTend(Node) + Shift;
}

void SchedulerBase::RegisterNewNode(DFGNode *Node, unsigned Ts) {
  assert(!Tstart.count(Node) && !Tend.count(Node) && "Node already exists");
  unsigned Te = Ts + PCtx.getLmax(Node);
  assert(GetMaxPredTend(Node) <= Ts && Te <= GetMinSuccTstart(Node));
  Tstart[Node] = Ts;
  Tend[Node] = Te;
}

bool SchedulerBase::RescheduleASAP(DataflowGraph *Graph) {
  bool Updated = false;
  for (DataflowGraph::iterator I = Graph->t_begin(), E = Graph->t_end();
       I != E; ++I) {
    DFGNode *Node = *I;
    unsigned MaxPredTend = GetMaxPredTend(Node);
    if (!Tstart.count(Node) || Tstart[Node] < MaxPredTend) {
      Updated = true;
      Tstart[Node] = MaxPredTend;
      Tend[Node] = MaxPredTend + PCtx.getLmax(Node);
    }
  }
  return Updated;
}

namespace {

class ScheduleProblem {
  const ASTContext &ASTCtx;
  PlatformContext &PCtx;
  DataflowGraph *Graph;

  typedef std::pair<DFGNode*, DFGNode*> DFGEdge;
  typedef llvm::DenseMap<DFGNode*, unsigned> NodeVarMapTy;
  typedef llvm::DenseMap<DFGEdge, unsigned> EdgeVarMapTy;

  unsigned NumVariables;
  NodeVarMapTy Vnode;
  EdgeVarMapTy Vedge;

  lprec *lp_;
  bool RowMode;
  std::vector<unsigned> Solution;

  void EnableRowMode();
  void DisableRowMode();

  class RowVector {
    REAL *Row;

    RowVector() {}
    RowVector(const RowVector&) {}

  public:
    explicit RowVector(unsigned size) {
      Row = new REAL[size + 1];
      assert(Row != NULL);
      std::fill(Row, Row + size + 1, 0);
    }
    ~RowVector() {
      delete [] Row;
    }
    const REAL &operator[](unsigned index) const { return Row[index]; }
    REAL &operator[](unsigned index) { return Row[index]; }
    const REAL *raw() const { return Row; }
    REAL *raw() { return Row; }
  };

public:
  ScheduleProblem(const ASTContext &AC, PlatformContext &PC, DataflowGraph *G,
                  unsigned AdditionalVariables = 0);
  ~ScheduleProblem();

  bool empty() const {
    return NumVariables == 0;
  }

  void AddPathLengthConstraints(unsigned L);
  void MinimizeWidthSum();
  bool Solve();

  unsigned GetVnode(DFGNode *Node);
  unsigned GetVedge(DFGNode *Pred, DFGNode *Succ);

private:
  void AddPathLengthConstraints(DFGNode *Node, RowVector &Row, unsigned Bias,
                                unsigned L);
};

void ScheduleProblem::EnableRowMode() {
  assert(lp_ != NULL);
  if (!RowMode) {
    set_add_rowmode(lp_, TRUE);
    RowMode = true;
  }
}

void ScheduleProblem::DisableRowMode() {
  assert(lp_ != NULL);
  if (RowMode) {
    set_add_rowmode(lp_, FALSE);
    RowMode = false;
  }
}

ScheduleProblem::ScheduleProblem(const ASTContext &AC, PlatformContext &PC,
                                 DataflowGraph *G,
                                 unsigned AdditionalVariables)
  : ASTCtx(AC), PCtx(PC), Graph(G), NumVariables(AdditionalVariables),
    lp_(NULL), RowMode(false) {
  for (DataflowGraph::iterator I = Graph->begin(), E = Graph->end();
       I != E; ++I) {
    DFGNode *Node = *I;
    if (isa<DFGConstNode>(Node)) continue;
    for (DFGNode::succ_iterator S = Node->succ_begin(), SEnd = Node->succ_end();
         S != SEnd; ++S) {
      DFGNode *Succ = *S;
      if (isa<DFGScatterNode>(Succ)) continue;
      Vedge[DFGEdge(Node, Succ)] = ++NumVariables;
    }
    if (!Node->getType().isNull() && Node->succ_size() > 0) {
      Vnode[Node] = ++NumVariables;
    }
  }
  if (NumVariables == 0) {
    return;
  }
  lp_ = make_lp(0, NumVariables);
  assert(lp_ != NULL);
  set_verbose(lp_, IMPORTANT);
  for (unsigned V = 1; V <= NumVariables; ++V) {
    int ret = set_int(lp_, V, true);
    assert(ret);
  }
}

ScheduleProblem::~ScheduleProblem() {
  if (lp_ != NULL) {
    delete_lp(lp_);
  }
}

void ScheduleProblem::AddPathLengthConstraints(unsigned L) {
  if (empty()) return;
  EnableRowMode();
  RowVector Row(NumVariables);
  AddPathLengthConstraints(Graph->getSource(), Row, 0, L);
}

void ScheduleProblem::AddPathLengthConstraints(DFGNode *Node, RowVector &Row,
                                               unsigned Bias, unsigned L) {
  Bias += PCtx.getLmax(Node);
  if (Node == Graph->getSink()) {
    assert(lp_ != NULL);
    assert(Bias <= L);
    int ret = add_constraint(lp_, Row.raw(), EQ, L - Bias);
    assert(ret);
  } else {
    if (Vnode.count(Node)) Row[Vnode[Node]] = 1;
    for (DFGNode::succ_iterator S = Node->succ_begin(), SEnd = Node->succ_end();
         S != SEnd; ++S) {
      DFGEdge Edge(Node, *S);
      if (Vedge.count(Edge)) Row[Vedge[Edge]] = 1;
      AddPathLengthConstraints(*S, Row, Bias, L);
      if (Vedge.count(Edge)) Row[Vedge[Edge]] = 0;
    }
    if (Vnode.count(Node)) Row[Vnode[Node]] = 0;
  }
}

void ScheduleProblem::MinimizeWidthSum() {
  if (empty()) return;
  DisableRowMode();
  RowVector Row(NumVariables);
  for (NodeVarMapTy::const_iterator I = Vnode.begin(), E = Vnode.end();
       I != E; ++I) {
    DFGNode *Node = I->first;
    QualType NodeTy = Node->getType();
    Row[I->second] = (NodeTy.isNull() ? 1 : ASTCtx.getTypeSize(NodeTy));
  }
  for (EdgeVarMapTy::const_iterator I = Vedge.begin(), E = Vedge.end();
       I != E; ++I) {
    DFGEdge Edge = I->first;
    QualType EdgeTy = Edge.first->getType();
    Row[I->second] = (EdgeTy.isNull() ? 1 : ASTCtx.getTypeSize(EdgeTy));
  }
  assert(lp_ != NULL);
  int ret = set_obj_fn(lp_, Row.raw());
  assert(ret);
}

bool ScheduleProblem::Solve() {
  if (empty()) {
    return false;
  }
  DisableRowMode();
  assert(lp_ != NULL);
  int ret = solve(lp_);
  if (ret == OPTIMAL) {
  } else if (ret == SUBOPTIMAL) {
    llvm::outs() << "Warning: the LP solver found a sub-optimal solution.\n";
  } else {
    llvm::outs() << "Error: the LP solver cannot find a solution.\n";
    return false;
  }

  Solution.resize(NumVariables + 1);
  RowVector Row(NumVariables);
  get_variables(lp_, Row.raw() + 1);
  for (unsigned V = 1; V <= NumVariables; ++V) {
    Solution[V] = (unsigned)Row[V];
  }
  return true;
}

unsigned ScheduleProblem::GetVnode(DFGNode *Node) {
  if (Vnode.count(Node)) {
    assert(Vnode[Node] < Solution.size());
    return Solution[Vnode[Node]];
  } else {
    return 0;
  }
}

unsigned ScheduleProblem::GetVedge(DFGNode *Pred, DFGNode *Succ) {
  DFGEdge Edge(Pred, Succ);
  if (Vedge.count(Edge)) {
    assert(Vedge[Edge] < Solution.size());
    return Solution[Vedge[Edge]];
  } else {
    return 0;
  }
}

} // anonymous namespace

void SchedulerBase::ScheduleILP(DataflowGraph *Graph,
                                const ASTContext &ASTCtx) {
  Tstart.clear();
  Tend.clear();
  RescheduleASAP(Graph);

  ScheduleProblem SP(ASTCtx, PCtx, Graph);
  SP.AddPathLengthConstraints(GetTend(Graph->getSink()));
  SP.MinimizeWidthSum();
  if (!SP.Solve()) {
    return;
  }

  for (DataflowGraph::iterator I = Graph->t_begin(), E = Graph->t_end();
       I != E; ++I) {
    DFGNode *Node = *I;
    unsigned NodeTstart = 0;
    for (DFGNode::pred_iterator P = Node->pred_begin(), PEnd = Node->pred_end();
         P != PEnd; ++P) {
      DFGNode *Pred = *P;
      unsigned PredTend = GetTend(Pred) + SP.GetVnode(Pred) +
                          SP.GetVedge(Pred, Node);
      NodeTstart = std::max(NodeTstart, PredTend);
    }
    Tstart[Node] = NodeTstart;
    Tend[Node] = NodeTstart + PCtx.getLmax(Node);
  }
}

void SchedulerBase::Synthesis(DataflowGraph *Graph) {
  // Inserting FIFO queues according to Tstart[*] and Tend[*]
  for (DataflowGraph::iterator I = Graph->t_begin(), E = Graph->t_end();
       I != E; ++I) {
    DFGNode *Node = *I;
    if (isa<DFGConstNode>(Node)) continue;

    /* Pred1 -\         /-> Succ1      Pred1 -> Queue -\                  /->
     * Pred2 ---> Node ---> Succ2  =>  Pred2 -> Queue ---> Node -> Queue --->
     * Pred3 -/         \-> Succ3      Pred3 -> Queue -/                  \->
     */
    unsigned NodeTstart = GetTstart(Node);
    for (DFGNode::pred_iterator P = Node->pred_begin(),
                                PEnd = Node->pred_end();
         P != PEnd; ++P) {
      DFGNode *Pred = *P;
      if (isa<DFGConstNode>(Pred)) continue;
      unsigned PredTend = GetTend(Pred);
      if (PredTend != NodeTstart) {
        assert(PredTend < NodeTstart);
        unsigned QueueSize = NodeTstart - PredTend;
        DFGQueueNode *Queue = InsertQueue(Graph, Pred, Node, QueueSize);
        RegisterNewNode(Queue, PredTend);
      }
    }
    if (!Node->getType().isNull() && Node->succ_size() > 0) {
      unsigned NodeTend = GetTend(Node);
      unsigned MinSuccTstart = GetMinSuccTstart(Node);
      if (NodeTend != MinSuccTstart) {
        assert(NodeTend < MinSuccTstart);
        unsigned QueueSize = MinSuccTstart - NodeTend;
        DFGQueueNode *Queue = AppendQueue(Graph, Node, QueueSize);
        RegisterNewNode(Queue, NodeTend);
      }
    }
  }
  Graph->Redefine();

  // Check integrity of the DFG
  for (DataflowGraph::iterator I = Graph->begin(), E = Graph->end();
       I != E; ++I) {
    DFGNode *Node = *I;
    if (isa<DFGConstNode>(Node)) continue;
    for (DFGNode::pred_iterator P = Node->pred_begin(),
                                PEnd = Node->pred_end();
         P != PEnd; ++P) {
      DFGNode *Pred = *P;
      if (isa<DFGConstNode>(Pred)) continue;
      assert(GetTend(Pred) == GetTstart(Node));
    }
  }
}

DFGQueueNode *SchedulerBase::InsertQueue(DataflowGraph *Graph, DFGNode *Pred,
                                         DFGNode *Succ, unsigned Size) {
  Pred->removeSuccessor(Succ, Graph);
  DFGQueueNode *Queue = new (Graph) DFGQueueNode(Graph, Pred, Size);
  unsigned NumEdges = Succ->replacePredecessor(Pred, Queue);
  assert(NumEdges > 0);
  while (NumEdges > 0) {
    Queue->addSuccessor(Succ, Graph);
    NumEdges--;
  }
  return Queue;
}

DFGQueueNode *SchedulerBase::AppendQueue(DataflowGraph *Graph, DFGNode *Pred,
                                         unsigned Size) {
  DFGQueueNode *Queue = new (Graph) DFGQueueNode(Graph, Pred, Size);
  for (DFGNode::succ_iterator S = Pred->succ_begin(), SEnd = Pred->succ_end();
       S + 1 != SEnd; ++S) {
    DFGNode *Succ = *S;
    bool Replaced = Succ->replacePredecessorOnce(Pred, Queue);
    assert(Replaced);
    Queue->addSuccessor(Succ, Graph);
  }
  Pred->removeAllSuccessors();
  Pred->addSuccessor(Queue, Graph);
  return Queue;
}

// DefaultScheduler

void DefaultScheduler::Schedule(DataflowGraph *Graph) {
  ScheduleILP(Graph, ASTCtx);
  Synthesis(Graph);
}

} // namespace snu

} // namespace clang
