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

#include "clang/SnuSynthesis/StructuralAnalysis.h"
#include "clang/Basic/LLVM.h"
#include "clang/SnuAST/WCFG.h"
#include "clang/SnuAnalysis/Dominator.h"
#include "clang/SnuAnalysis/Loop.h"
#include "clang/SnuSupport/OrderedDenseADT.h"
#include "clang/SnuSynthesis/ControlFlowConstraint.h"
#include "clang/SnuSynthesis/DataflowGraph.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/DenseSet.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/Analysis/DominatorInternals.h"
#include "llvm/Analysis/Dominators.h"
#include <queue>

namespace clang {

namespace snu {

ControlTreeNode::ControlTreeNode(ControlTreeNodeClass C, unsigned NumSuccs)
  : Class(C), Succs(NumSuccs, NULL), LiveOuts(NumSuccs), IsProgramExit(false),
    HasBarrier(false), HasCondBarrier(false) {
}

ControlTreeNode::child_range ControlTreeNode::children() {
  switch (getClass()) {
#define NODE(type) \
  case CT##type##Class: \
    return static_cast<CT##type*>(this)->children();
  NODE(BasicBlock)
  NODE(Sequential)
  NODE(IfThen)
  NODE(IfThenElse)
  NODE(Acyclic)
  NODE(SelfLoop)
  NODE(WhileLoop)
  NODE(NaturalLoop)
#undef NODE
  default:
    llvm_unreachable("invalid node class");
  }
}

bool ControlTreeNode::contains(DataflowGraph *dfg) const {
  if (const CTBasicBlock *BB = dyn_cast<CTBasicBlock>(this)) {
    return BB->getDFG() == dfg;
  } else {
    for (const_child_iterator I = child_begin(), E = child_end();
         I != E; ++I) {
      if ((*I)->contains(dfg)) {
        return true;
      }
    }
    return false;
  }
}

DataflowGraph *ControlTreeNode::getEntry() const {
  if (const CTBasicBlock *BB = dyn_cast<CTBasicBlock>(this)) {
    return BB->getDFG();
  } else {
    assert(child_begin() != child_end());
    return (*child_begin())->getEntry();
  }
}

void ControlTreeNode::AddControlFlowConstraint(const ControlFlowConstraint &C) {
  CFConstraint.Merge(C);
}

// CTBasicBlock

CTBasicBlock::CTBasicBlock(ControlDataflowGraph *cdfg, DataflowGraph *dfg_)
  : ControlTreeNode(CTBasicBlockClass, dfg_->getNumSuccs()), dfg(dfg_) {
  for (unsigned Index = 0, NumSuccs = getNumSuccs();
       Index != NumSuccs; ++Index) {
    SetSucc(Index, dfg->getSucc(cdfg, Index));
  }
  SetLiveIns(dfg->live_in_begin(), dfg->live_in_end());
  for (unsigned Index = 0, NumSuccs = getNumSuccs();
       Index != NumSuccs; ++Index) {
    SetLiveOuts(Index, dfg->live_out_begin(), dfg->live_out_end());
  }
  if (cdfg->getExit() == dfg) {
    IsProgramExit = true;
  }
  HasBarrier = dfg->hasBarrier();
  HasCondBarrier = false;
}

// CTSequential

CTSequential::CTSequential(ControlTreeNode *First, ControlTreeNode *Second)
  : ControlTreeNode(CTSequentialClass, Second->getNumSuccs()) {
  Children[FIRST] = First;
  Children[SECOND] = Second;
  assert(First->getNumSuccs() == 1);
  for (unsigned Index = 0, NumSuccs = getNumSuccs();
       Index != NumSuccs; ++Index) {
    SetSucc(Index, Second->getSucc(Index));
  }
  SetLiveIns(First->live_in_begin(), First->live_in_end());
  for (unsigned Index = 0, NumSuccs = getNumSuccs();
       Index != NumSuccs; ++Index) {
    SetLiveOuts(Index, Second->live_out_begin(Index),
                Second->live_out_end(Index));
  }
  assert(!First->isProgramExit());
  IsProgramExit = Second->isProgramExit();
  HasBarrier = (First->hasBarrier() || Second->hasBarrier());
  HasCondBarrier = (First->hasCondBarrier() || Second->hasCondBarrier());
}

// CTIfThen

CTIfThen::CTIfThen(ControlTreeNode *If, ControlTreeNode *Then)
  : ControlTreeNode(CTIfThenClass, 1) {
  Children[IF] = If;
  Children[THEN] = Then;
  assert(If->getNumSuccs() == 2 && Then->getNumSuccs() == 1);
  SetSucc(0, Then->getSucc(0));
  SetLiveIns(If->live_in_begin(), If->live_in_end());
  assert(!If->isProgramExit() && !Then->isProgramExit());
  HasBarrier = (If->hasBarrier() || Then->hasBarrier());
  HasCondBarrier = (If->hasCondBarrier() || Then->hasBarrier());
}

// CTIfThenElse

CTIfThenElse::CTIfThenElse(ControlTreeNode *If, ControlTreeNode *Then,
                           ControlTreeNode *Else)
  : ControlTreeNode(CTIfThenElseClass, 1) {
  Children[IF] = If;
  Children[THEN] = Then;
  Children[ELSE] = Else;
  assert(If->getNumSuccs() == 2);
  assert(Then->getNumSuccs() == 1 && Else->getNumSuccs() == 1 &&
         Then->getSucc(0) == Else->getSucc(0));
  SetSucc(0, Then->getSucc(0));
  SetLiveIns(If->live_in_begin(), If->live_in_end());
  assert(!If->isProgramExit() && !Then->isProgramExit() &&
         !Else->isProgramExit());
  HasBarrier = (If->hasBarrier() || Then->hasBarrier() || Else->hasBarrier());
  HasCondBarrier = (If->hasCondBarrier() || Then->hasBarrier() || Else->hasBarrier());
}

// CTAcyclic

CTAcyclic::CTAcyclic()
  : ControlTreeNode(CTAcyclicClass) {
  EdgePtrs.push_back(0);
}

unsigned CTAcyclic::getNumEdges(unsigned NI) const {
  assert(NI + 1 < EdgePtrs.size());
  return EdgePtrs[NI + 1] - EdgePtrs[NI];
}

bool CTAcyclic::isInternalEdge(unsigned NI, unsigned EI) const {
  assert(NI < EdgePtrs.size() && EdgePtrs[NI] + EI < Edges.size());
  return (Edges[EdgePtrs[NI] + EI] & 0x1UL) == 0;
}

ControlTreeNode *CTAcyclic::getInternalEdge(unsigned NI, unsigned EI) const {
  assert(NI < EdgePtrs.size() && EdgePtrs[NI] + EI < Edges.size());
  assert((Edges[EdgePtrs[NI] + EI] & 0x1UL) == 0);
  return (ControlTreeNode*)Edges[EdgePtrs[NI] + EI];
}

bool CTAcyclic::isExternalEdge(unsigned NI, unsigned EI) const {
  assert(NI < EdgePtrs.size() && EdgePtrs[NI] + EI < Edges.size());
  return (Edges[EdgePtrs[NI] + EI] & 0x1UL) == 1;
}

unsigned CTAcyclic::getExternalEdge(unsigned NI, unsigned EI) const {
  assert(NI < EdgePtrs.size() && EdgePtrs[NI] + EI < Edges.size());
  assert((Edges[EdgePtrs[NI] + EI] & 0x1UL) == 1);
  return (unsigned)(Edges[EdgePtrs[NI] + EI] >> 1);
}

void CTAcyclic::AddNode(ControlTreeNode *Node) {
  Nodes.push_back(Node);
  EdgePtrs.push_back(Edges.size());
  IsProgramExit = (IsProgramExit || Node->isProgramExit());
  HasBarrier = (HasBarrier || Node->hasBarrier());
  if (Nodes.size() == 1) { // Entry node
    HasCondBarrier = Node->hasCondBarrier();
  } else {
    HasCondBarrier = (HasCondBarrier || Node->hasBarrier());
  }
}

void CTAcyclic::AddEdge(ControlTreeNode *From, ControlTreeNode *To) {
  assert(From == Nodes.back());
  assert(((size_t)To & 0x1UL) == 0);
  Edges.push_back((size_t)To);
  EdgePtrs.back()++;
}

void CTAcyclic::AddExit(ControlTreeNode *From, DataflowGraph *To) {
  assert(From == Nodes.back());
  assert(EdgePtrs.size() > 0);
  unsigned EdgeIndex = getNumEdges(Nodes.size() - 1);
  unsigned SuccIndex;
  for (SuccIndex = 0; SuccIndex != Succs.size(); SuccIndex++) {
    if (Succs[SuccIndex] == To) {
      break;
    }
  }
  Edges.push_back(((size_t)SuccIndex << 1) | 0x1UL);
  EdgePtrs.back()++;
  if (SuccIndex == Succs.size()) {
    Succs.push_back(To);
    LiveOuts.push_back(LiveVarListTy());
    SetLiveOuts(SuccIndex, To->live_in_begin(), To->live_in_end());
  }
}

// CTSelfLoop

CTSelfLoop::CTSelfLoop(ControlTreeNode *body, unsigned loopback_index,
                       bool fixed_bound)
  : ControlTreeNode(CTSelfLoopClass, 1), Body(body),
    LoopBackIndex(loopback_index), FixedBound(fixed_bound) {
  assert(Body->getNumSuccs() == 2);
  SetSucc(0, Body->getSucc(getExitIndex()));
  SetLiveOuts(0, Body->live_out_begin(getExitIndex()),
              Body->live_out_end(getExitIndex()));
  assert(!Body->isProgramExit());
  HasBarrier = Body->hasBarrier();
  HasCondBarrier = (Body->hasCondBarrier() || (!FixedBound && HasBarrier));
}

// CTWhileLoop

CTWhileLoop::CTWhileLoop(ControlTreeNode *Cond, unsigned loopback_index,
                         ControlTreeNode *Body, bool fixed_bound)
  : ControlTreeNode(CTWhileLoopClass, 1), LoopBackIndex(loopback_index),
    FixedBound(fixed_bound) {
  Children[COND] = Cond;
  Children[BODY] = Body;
  assert(Cond->getNumSuccs() == 2 && Body->getNumSuccs() == 1);
  SetSucc(0, Cond->getSucc(getExitIndex()));
  SetLiveOuts(0, Cond->live_out_begin(getExitIndex()),
              Cond->live_out_end(getExitIndex()));
  assert(!Cond->isProgramExit() && !Body->isProgramExit());
  HasBarrier = (Cond->hasBarrier() || Body->hasBarrier());
  HasCondBarrier = (Cond->hasCondBarrier() || Body->hasCondBarrier() ||
                    (!FixedBound && HasBarrier));
}

// CTNaturalLoop

CTNaturalLoop::CTNaturalLoop(ControlTreeNode *body, unsigned loopback_index)
  : ControlTreeNode(CTNaturalLoopClass, body->getNumSuccs() - 1), Body(body),
    LoopBackIndex(loopback_index) {
  assert(Body->getNumSuccs() >= 2);
  for (unsigned Index = 0, NumSuccs = getNumSuccs();
       Index != NumSuccs; ++Index) {
    unsigned BodyIndex = Index + (Index >= LoopBackIndex);
    SetSucc(Index, Body->getSucc(BodyIndex));
    SetLiveOuts(Index, Body->live_out_begin(BodyIndex),
                Body->live_out_end(BodyIndex));
  }
  assert(!Body->isProgramExit());
  HasBarrier = Body->hasBarrier();
  HasCondBarrier = Body->hasBarrier();
}

// ControlTreeBuilder

namespace {

class CDFGLoopFinder : public LoopVisitor<CDFGLoopFinder> {
  class CDFGLoop {
    DataflowGraph *Head;
    DataflowGraph *Tail;
    llvm::DenseSet<DataflowGraph*> Body;
    size_t UniqueExit;
    SmallVector<size_t, 8> Exits;

    bool HasFixedBound;

  public:
    CDFGLoop() : Head(NULL), Tail(NULL), UniqueExit(0), HasFixedBound(false) {}

    DataflowGraph *getHead() const { return Head; }
    DataflowGraph *getTail() const { return Tail; }
    bool contains(DataflowGraph *Node) const { return Body.count(Node); }

    bool hasUniqueExit() const { return UniqueExit != 0; }
    DataflowGraph *getUniqueExit() const {
      assert(UniqueExit != 0);
      return (DataflowGraph*)(UniqueExit & ~0x1UL);
    }
    unsigned getUniqueExitIndex() const {
      assert(UniqueExit != 0);
      return (unsigned)(UniqueExit & 0x1UL);
    }

    unsigned getNumExits() const { return Exits.size(); }
    DataflowGraph *getExit(unsigned I) const {
      return (DataflowGraph*)(Exits[I] & ~0x1UL);
    }
    unsigned getExitIndex(unsigned I) const {
      return (unsigned)(Exits[I] & 0x1UL);
    }

    void setHead(DataflowGraph *head) {
      assert(Head == NULL);
      Head = head;
    }
    void setTail(DataflowGraph *tail) {
      assert(Tail == NULL);
      Tail = tail;
    }
    void addBody(DataflowGraph *Node) {
      assert(!Body.count(Node));
      Body.insert(Node);
    }

    void addExit(DataflowGraph *Node, unsigned SuccIndex) {
      assert(((size_t)Node & 0x1UL) == 0 && SuccIndex < 2);
      size_t NewExit = (size_t)Node | (size_t)SuccIndex;
      if (Exits.empty()) {
        UniqueExit = NewExit;
      } else {
        UniqueExit = 0;
      }
      Exits.push_back(NewExit);
    }

    bool hasFixedBound() const { return HasFixedBound; }
    void setHasFixedBound(bool V) {
      HasFixedBound = V;
    }
  };

private:
  ControlDataflowGraph *cdfg;
  llvm::DenseMap<DataflowGraph*, CDFGLoop> Loops;

public:
  CDFGLoopFinder(ControlDataflowGraph *cdfg_)
    : cdfg(cdfg_) {}

  void VisitLoop(Loop &L);

  bool IsLoopHead(DataflowGraph *Head) const {
    return Loops.count(Head);
  }

  CDFGLoop operator[](DataflowGraph *Head) const {
    assert(Loops.count(Head));
    return Loops.lookup(Head);
  }
};

void CDFGLoopFinder::VisitLoop(Loop &L) {
  DataflowGraph *Head = cdfg->getDFG(L.getHead());
  CDFGLoop &CurrentLoop = Loops[Head];
  CurrentLoop.setHead(Head);

  llvm::DenseMap<DataflowGraph*, unsigned> Color;
  SmallVector<DataflowGraph*, 256> TraverseQueue;
  Color[Head] = 0;
  TraverseQueue.push_back(Head);
  while (!TraverseQueue.empty()) {
    DataflowGraph *G = TraverseQueue.back();
    Color[G] = 1;
    TraverseQueue.pop_back();
    CurrentLoop.addBody(G);

    assert(G->getNumSuccs() <= 2);
    for (unsigned Index = 0, NumSuccs = G->getNumSuccs();
          Index != NumSuccs; ++Index) {
      DataflowGraph *Succ = G->getSucc(cdfg, Index);
      if (!L.isLoopBody(Succ->getEntryBlock())) {
        CurrentLoop.addExit(G, Index);
      } else {
        if (Succ == Head) {
          CurrentLoop.setTail(G);
        }
        if (!Color.count(Succ)) {
          Color[Succ] = 0;
          TraverseQueue.push_back(Succ);
        }
      }
    }
  }
  assert(CurrentLoop.getTail() != NULL && CurrentLoop.getNumExits() > 0);

  // Loop analysis
  CurrentLoop.setHasFixedBound(L.hasFixedBound());
}

class ControlTreeBuilder {
  ControlDataflowGraph *cdfg;

  CDFGLoopFinder LF;
  WCFGDominatorTree DT;
  WCFGDominanceFrontier DF;

public:
  ControlTreeBuilder(ControlDataflowGraph *cdfg_);

  ControlTreeNode *buildFirstSubtree(DataflowGraph *Entry,
                                     DataflowGraph *OuterLoop = NULL,
                                     DataflowGraph *Until = NULL);
  ControlTreeNode *build(DataflowGraph *Entry, DataflowGraph *OuterLoop = NULL,
                         DataflowGraph *Until = NULL,
                         bool IgnoreAcyclic = false);

private:
  bool IsDominates(DataflowGraph *A, DataflowGraph *B) {
    return DT.dominates(A->getEntryBlock(), B->getEntryBlock());
  }
  unsigned GetNumDomFrontiers(DataflowGraph *A) {
    return DF.num_frontiers(A->getEntryBlock());
  }
  DataflowGraph *GetUniqueDomFrontier(DataflowGraph *A) {
    if (DF.num_frontiers(A->getEntryBlock()) != 1) {
      return NULL;
    }
    return cdfg->getDFG(*DF.frontier_begin(A->getEntryBlock()));
  }

  bool WithinTheSameNode(DataflowGraph *Entry, DataflowGraph *Succ,
                         DataflowGraph *OuterLoop) {
    if (Entry == OuterLoop) {
      return IsDominates(Entry, Succ) && LF[OuterLoop].contains(Succ) &&
             Succ != OuterLoop;
    } else {
      return IsDominates(Entry, Succ);
    }
  }
};

ControlTreeBuilder::ControlTreeBuilder(ControlDataflowGraph *cdfg_)
  : cdfg(cdfg_), LF(cdfg_), DT(false) {
  LF.Visit(cdfg->getOriginal());
  DT.recalculate(*cdfg->getOriginal());
  DF.recalculate(DT);
}

ControlTreeNode *ControlTreeBuilder::buildFirstSubtree(DataflowGraph *Entry,
                                                       DataflowGraph *OuterLoop,
                                                       DataflowGraph *Until) {
  if (LF.IsLoopHead(Entry) && Entry != OuterLoop) {
    if (LF[Entry].hasUniqueExit()) {
      DataflowGraph *Tail = LF[Entry].getTail();
      DataflowGraph *ExitFrom = LF[Entry].getUniqueExit();
      assert(ExitFrom->getNumSuccs() == 2);
      unsigned LoopBackIndex = 1 - LF[Entry].getUniqueExitIndex();

      if (ExitFrom == Tail) {
        return (new CTSelfLoop(
            build(Entry, Entry, Tail), LoopBackIndex,
            LF[Entry].hasFixedBound()))
            -> WithLiveIns(Entry->live_in_begin(), Entry->live_in_end());
      } else if (IsDominates(ExitFrom, Tail)) {
        return (new CTWhileLoop(
            build(Entry, Entry, ExitFrom), LoopBackIndex,
            build(ExitFrom->getSucc(cdfg, LoopBackIndex), Entry, Tail),
            LF[Entry].hasFixedBound()))
            -> WithLiveIns(Entry->live_in_begin(), Entry->live_in_end());
      }
    }

    ControlTreeNode *Body = build(Entry, Entry);
    assert(Body->getNumSuccs() >= 2);
    unsigned LoopBackIndex = (unsigned)-1;
    for (unsigned Index = 0, NumSuccs = Body->getNumSuccs();
         Index != NumSuccs; ++Index) {
      if (Body->getSucc(Index) == Entry) {
        assert(LoopBackIndex == (unsigned)-1);
        LoopBackIndex = Index;
      }
    }
    assert(LoopBackIndex != (unsigned)-1);
    return (new CTNaturalLoop(Body, LoopBackIndex))
        -> WithLiveIns(Entry->live_in_begin(), Entry->live_in_end());
  }

  if (Entry->getNumSuccs() == 2 && Entry != Until) {
    DataflowGraph *TrueSucc = Entry->getSucc(cdfg, 0);
    DataflowGraph *FalseSucc = Entry->getSucc(cdfg, 1);
    assert(Entry != TrueSucc && Entry != FalseSucc);

    bool IsDomTrue = IsDominates(Entry, TrueSucc);
    bool IsDomFalse = IsDominates(Entry, FalseSucc);
    DataflowGraph *TrueDomFrontier = GetUniqueDomFrontier(TrueSucc);
    DataflowGraph *FalseDomFrontier = GetUniqueDomFrontier(FalseSucc);
    assert(Entry != TrueDomFrontier && Entry != FalseDomFrontier);

    if (IsDomTrue && TrueDomFrontier == FalseSucc) {
      return (new CTIfThen(new CTBasicBlock(cdfg, Entry),
                           build(TrueSucc, OuterLoop)))
        -> WithLiveOuts(0, TrueDomFrontier->live_in_begin(),
                        TrueDomFrontier->live_in_end());
    } else if (IsDomTrue && IsDomFalse && TrueDomFrontier != NULL &&
               TrueDomFrontier == FalseDomFrontier) {
      return (new CTIfThenElse(new CTBasicBlock(cdfg, Entry),
                               build(TrueSucc, OuterLoop),
                               build(FalseSucc, OuterLoop)))
        -> WithLiveOuts(0, TrueDomFrontier->live_in_begin(),
                        TrueDomFrontier->live_in_end());
    }
  }

  return new CTBasicBlock(cdfg, Entry);
}

ControlTreeNode *ControlTreeBuilder::build(DataflowGraph *Entry,
                                           DataflowGraph *OuterLoop,
                                           DataflowGraph *Until,
                                           bool IgnoreAcyclic) {
  ControlTreeNode *First = buildFirstSubtree(Entry, OuterLoop, Until);

  if (First->getNumSuccs() == 0 || Entry == Until) {
    return First;

  } else if (First->getNumSuccs() == 1) {
    DataflowGraph *Succ = First->getSucc(0);
    if (WithinTheSameNode(Entry, Succ, OuterLoop)) {
      return new CTSequential(First,
                              build(Succ, OuterLoop, Until, IgnoreAcyclic));
    } else {
      return First;
    }

  } else {
    bool IsAcyclic = false;
    for (unsigned Index = 0, NumSuccs = First->getNumSuccs();
         Index != NumSuccs; ++Index) {
      IsAcyclic |= WithinTheSameNode(Entry, First->getSucc(Index), OuterLoop);
    }
    if (IsAcyclic && !IgnoreAcyclic) {
      CTAcyclic *Acyclic = new CTAcyclic();
      Acyclic->SetLiveIns(First->live_in_begin(), First->live_in_end());

      llvm::DenseMap<DataflowGraph*, ControlTreeNode*> Impl;
      OrderedDenseMap<ControlTreeNode*, unsigned> Indegree;
      SmallVector<DataflowGraph*, 16> TraverseQueue;
      Impl[Entry] = First;
      Indegree[First] = 0;
      TraverseQueue.push_back(Entry);

      while (!TraverseQueue.empty()) {
        DataflowGraph *G = TraverseQueue.back();
        TraverseQueue.pop_back();
        ControlTreeNode *Node = Impl[G];
        if (G == Until) {
          continue;
        }
        for (unsigned SuccIndex = 0, NumSuccs = Node->getNumSuccs();
             SuccIndex != NumSuccs; ++SuccIndex) {
          DataflowGraph *Succ = Node->getSucc(SuccIndex);
          if (WithinTheSameNode(Entry, Succ, OuterLoop)) {
            if (!Impl.count(Succ)) {
              Impl[Succ] = build(Succ, OuterLoop, Until, true);
              Indegree[Impl[Succ]] = 0;
              TraverseQueue.push_back(Succ);
            }
            Indegree[Impl[Succ]]++;
          }
        }
      }

      while (!Indegree.empty()) {
        ControlTreeNode *Node = NULL;
        for (OrderedDenseMap<ControlTreeNode*, unsigned>::const_iterator
                 I = Indegree.begin(), E = Indegree.end();
             I != E; ++I) {
          if (I->second == 0) {
            Node = I->first;
            Indegree.erase(I->first);
            break;
          }
        }
        assert(Node != NULL);
        Acyclic->AddNode(Node);
        for (unsigned SuccIndex = 0, NumSuccs = Node->getNumSuccs();
             SuccIndex != NumSuccs; ++SuccIndex) {
          DataflowGraph *Succ = Node->getSucc(SuccIndex);
          if (WithinTheSameNode(Entry, Succ, OuterLoop)) {
            assert(Impl.count(Succ));
            Acyclic->AddEdge(Node, Impl[Succ]);
            assert(Indegree.count(Impl[Succ]));
            Indegree[Impl[Succ]]--;
          } else {
            Acyclic->AddExit(Node, Succ);
          }
        }
      }
      return Acyclic;
    } else {
      return First;
    }
  }
}

} // anonymous namespace

ControlTreeNode *ControlTreeNode::Build(ControlDataflowGraph *cdfg) {
  ControlTreeBuilder builder(cdfg);
  ControlTreeNode *Result = builder.build(cdfg->getEntry());
  return Result;
}

void ControlTreeNode::print(raw_ostream &OS) const {
  switch (getClass()) {
    case CTBasicBlockClass: {
      const CTBasicBlock *BB = static_cast<const CTBasicBlock*>(this);
      DataflowGraph *dfg = BB->getDFG();
      OS << "BB" << dfg->getEntryBlock()->getBlockID() << "-"
         << dfg->getExitBlock()->getBlockID();
      break;
    }

    case CTSequentialClass: {
      const CTSequential *Seq = static_cast<const CTSequential*>(this);
      OS << "Seq(";
      Seq->getFirst()->print(OS);
      OS << ", ";
      Seq->getSecond()->print(OS);
      OS << ")";
      break;
    }

    case CTIfThenClass: {
      const CTIfThen *IfThen = static_cast<const CTIfThen*>(this);
      OS << "IfThen(";
      IfThen->getIf()->print(OS);
      OS << ", ";
      IfThen->getThen()->print(OS);
      OS << ")";
      break;
    }

    case CTIfThenElseClass: {
      const CTIfThenElse *IfThenElse = static_cast<const CTIfThenElse*>(this);
      OS << "IfThenElse(";
      IfThenElse->getIf()->print(OS);
      OS << ", ";
      IfThenElse->getThen()->print(OS);
      OS << ", ";
      IfThenElse->getElse()->print(OS);
      OS << ")";
      break;
    }

    case CTAcyclicClass: {
      const CTAcyclic *Acyclic = static_cast<const CTAcyclic*>(this);
      OS << "Acyclic(";
      for (unsigned NI = 0, NumNodes = Acyclic->getNumNodes();
           NI != NumNodes; ++NI) {
        if (NI != 0) OS << ", ";
        OS << '[' << Acyclic->getNode(NI) << "] ";
        Acyclic->getNode(NI)->print(OS);
      }
      OS << "; ";
      bool FirstEdge = true;
      for (unsigned NI = 0, NumNodes = Acyclic->getNumNodes();
           NI != NumNodes; ++NI) {
        for (unsigned EI = 0, NumEdges = Acyclic->getNumEdges(NI);
             EI != NumEdges; ++EI) {
          if (!FirstEdge) OS << ", ";
          if (Acyclic->isInternalEdge(NI, EI)) {
            OS << '[' << Acyclic->getNode(NI) << "]->["
               << Acyclic->getInternalEdge(NI, EI) << ']';
          } else {
            assert(Acyclic->isExternalEdge(NI, EI));
            OS << '[' << Acyclic->getNode(NI) << "]->[Succ "
               << Acyclic->getExternalEdge(NI, EI) << ']';
          }
          FirstEdge = false;
        }
      }
      OS << ")";
      break;
    }

    case CTSelfLoopClass: {
      const CTSelfLoop *Loop = static_cast<const CTSelfLoop*>(this);
      OS << "SelfLoop(";
      Loop->getBody()->print(OS);
      if (Loop->hasFixedBound()) {
        OS << ", fixed";
      }
      OS << ")";
      break;
    }

    case CTWhileLoopClass: {
      const CTWhileLoop *Loop = static_cast<const CTWhileLoop*>(this);
      OS << "WhileLoop(";
      Loop->getCond()->print(OS);
      OS << ", ";
      Loop->getBody()->print(OS);
      if (Loop->hasFixedBound()) {
        OS << ", fixed";
      }
      OS << ")";
      break;
    }

    case CTNaturalLoopClass: {
      const CTNaturalLoop *Loop = static_cast<const CTNaturalLoop*>(this);
      OS << "NaturalLoop(";
      Loop->getBody()->print(OS);
      OS << ")";
      break;
    }
  }
}

} // namespace snu

} // namespace clang
