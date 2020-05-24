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

#include "clang/SnuSynthesis/ControlFlowConstraint.h"
#include "CGCommon.h"
#include "clang/Basic/LLVM.h"
#include "clang/Basic/AddressSpaces.h"
#include "clang/SnuAST/WAST.h"
#include "clang/SnuSynthesis/DataflowGraph.h"
#include "clang/SnuSynthesis/PlatformContext.h"
#include "clang/SnuSynthesis/StructuralAnalysis.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/DenseSet.h"
#include "llvm/ADT/StringRef.h"
#include <algorithm>

namespace clang {

namespace snu {

// ControlFlowConstraintContext

unsigned ControlFlowConstraintContext::getWorkGroupSize() {
  return 1 << PCtx.getPortLIDWidth();
}

unsigned ControlFlowConstraintContext::Analysis(
    DataflowGraph *Graph,
    unsigned (*Transfer)(unsigned Pred, PlatformContext& PCtx, DFGNode *Node),
    unsigned (*Join)(unsigned A, unsigned B)) {
  DFGNode *Source = Graph->getSource();
  DFGNode *Sink = Graph->getSink();
  llvm::DenseMap<DFGNode*, unsigned> Result;
  Result[Source] = Transfer(0, PCtx, Source);
  for (DataflowGraph::iterator N = Graph->t_begin() + 1, NEnd = Graph->t_end();
       N != NEnd; ++N) {
    if (Result.count(*N) || isa<DFGConstNode>(*N)) continue;
    DFGNode *Node;
    if (DFGCompoundNode *Compound = Graph->getCompoundOf(*N)) {
      Node = Compound;
    } else {
      Node = *N;
    }

    unsigned Result_Node = 0;
    bool HasPreds = false;
    for (DFGNode::const_pred_iterator P = Node->pred_begin(),
                                      PEnd = Node->pred_end();
         P != PEnd; ++P) {
      if (Result.count(*P)) {
        Result_Node = (HasPreds ? Join(Result_Node, Result[*P]) : Result[*P]);
        HasPreds = true;
      }
    }
    if (!HasPreds) continue;

    if (!Node->isVirtualKind()) {
      Result_Node = Transfer(Result_Node, PCtx, Node);
    }
    if (DFGCompoundNode *Compound = dyn_cast<DFGCompoundNode>(Node)) {
      for (DFGCompoundNode::iterator I = Compound->begin(), E = Compound->end();
           I != E; ++I) {
        Result[*I] = Result_Node;
      }
    } else {
      Result[Node] = Result_Node;
    }
  }
  return Result.count(Sink) ? Result[Sink] : 0;
}

unsigned ControlFlowConstraintContext::Analysis(
    ControlTreeNode *Node, CTCacheTy &Cache,
    unsigned (*Transfer)(unsigned Pred, PlatformContext& PCtx, DFGNode *Node),
    unsigned (*Join_Op)(unsigned A, unsigned B),
    unsigned (*Join_Branch)(unsigned A, unsigned B)) {
  if (!Cache.count(Node)) {
    switch (Node->getClass()) {
      case ControlTreeNode::CTBasicBlockClass: {
        CTBasicBlock *BB = static_cast<CTBasicBlock*>(Node);
        Cache[Node] = Analysis(BB->getDFG(), Transfer, Join_Op);
        break;
      }
      case ControlTreeNode::CTSequentialClass: {
        CTSequential *Seq = static_cast<CTSequential*>(Node);
        unsigned Result_First = Analysis(Seq->getFirst(), Cache, Transfer,
                                         Join_Op, Join_Branch);
        unsigned Result_Second = Analysis(Seq->getSecond(), Cache, Transfer,
                                          Join_Op, Join_Branch);
        Cache[Node] = Result_First + Result_Second;
        break;
      }
      case ControlTreeNode::CTIfThenClass: {
        CTIfThen *IfThen = static_cast<CTIfThen*>(Node);
        unsigned Result_If = Analysis(IfThen->getIf(), Cache, Transfer,
                                      Join_Op, Join_Branch);
        unsigned Result_Then = Analysis(IfThen->getThen(), Cache, Transfer,
                                        Join_Op, Join_Branch);
        if (Join_Branch) {
          Cache[Node] = Result_If + Join_Branch(Result_Then, 0);
        } else {
          Cache[Node] = Result_If + Result_Then;
        }
        break;
      }
      case ControlTreeNode::CTIfThenElseClass: {
        CTIfThenElse *IfThenElse = static_cast<CTIfThenElse*>(Node);
        unsigned Result_If = Analysis(IfThenElse->getIf(), Cache, Transfer,
                                      Join_Op, Join_Branch);
        unsigned Result_Then = Analysis(IfThenElse->getThen(), Cache, Transfer,
                                        Join_Op, Join_Branch);
        unsigned Result_Else = Analysis(IfThenElse->getElse(), Cache, Transfer,
                                        Join_Op, Join_Branch);
        if (Join_Branch) {
          Cache[Node] = Result_If + Join_Branch(Result_Then, Result_Else);
        } else {
          Cache[Node] = Result_If + Result_Then + Result_Else;
        }
        break;
      }
      case ControlTreeNode::CTAcyclicClass: {
        CTAcyclic *Acyclic = static_cast<CTAcyclic*>(Node);
        unsigned Result = 0;
        if (Join_Branch) {
          llvm::DenseMap<ControlTreeNode*, unsigned> Distance;
          Distance[Acyclic->getEntry()] = 0;
          for (unsigned NI = 0, NumNodes = Acyclic->getNumNodes();
               NI != NumNodes; ++NI) {
            ControlTreeNode *N = Acyclic->getNode(NI);
            assert(Distance.count(N));
            unsigned NDistance = Distance[N] + Analysis(N, Cache, Transfer,
                                                        Join_Op, Join_Branch);
            Result = Join_Branch(Result, NDistance);
            for (unsigned EI = 0, NumEdges = Acyclic->getNumEdges(NI);
                 EI != NumEdges; ++EI) {
              if (Acyclic->isInternalEdge(NI, EI)) {
                ControlTreeNode *Next = Acyclic->getInternalEdge(NI, EI);
                if (!Distance.count(Next)) {
                  Distance[Next] = NDistance;
                } else {
                  Distance[Next] = Join_Branch(Distance[Next], NDistance);
                }
              }
            }
          }
        } else {
          for (unsigned NI = 0, NumNodes = Acyclic->getNumNodes();
               NI != NumNodes; ++NI) {
            Result += getNmax(Acyclic->getNode(NI));
          }
        }
        Cache[Node] = Result;
        break;
      }
      case ControlTreeNode::CTSelfLoopClass: {
        CTSelfLoop *Loop = static_cast<CTSelfLoop*>(Node);
        unsigned Result_Body = Analysis(Loop->getBody(), Cache, Transfer,
                                        Join_Op, Join_Branch);
        Cache[Node] = Result_Body;
        break;
      }
      case ControlTreeNode::CTWhileLoopClass: {
        CTWhileLoop *Loop = static_cast<CTWhileLoop*>(Node);
        unsigned Result_Cond = Analysis(Loop->getCond(), Cache, Transfer,
                                        Join_Op, Join_Branch);
        unsigned Result_Body = Analysis(Loop->getBody(), Cache, Transfer,
                                        Join_Op, Join_Branch);
        if (Join_Branch) {
          Cache[Node] = Result_Cond + Join_Branch(Result_Body, 0);
        } else {
          Cache[Node] = Result_Cond + Result_Body;
        }
        break;
      }
      case ControlTreeNode::CTNaturalLoopClass: {
        CTNaturalLoop *Loop = static_cast<CTNaturalLoop*>(Node);
        unsigned Result_Body = Analysis(Loop->getBody(), Cache, Transfer,
                                        Join_Op, Join_Branch);
        Cache[Node] = Result_Body;
        break;
      }
    }
  }
  return Cache[Node];
}

namespace {

unsigned Transfer_Nmax(unsigned Pred, PlatformContext &PCtx, DFGNode *Node) {
  return Pred + PCtx.getLmax(Node) + 1;
}
unsigned Transfer_Nusual(unsigned Pred, PlatformContext &PCtx, DFGNode *Node) {
  return Pred + PCtx.getLmax(Node);
}
unsigned Transfer_Nstall(unsigned Pred, PlatformContext &PCtx, DFGNode *Node) {
  return Pred + PCtx.getNstall(Node);
}
unsigned Join_Min(unsigned A, unsigned B) {
  return (A < B) ? A : B;
}
unsigned Join_Max(unsigned A, unsigned B) {
  return (A > B) ? A : B;
}

} // anonymous namespace

unsigned ControlFlowConstraintContext::getNmax(ControlTreeNode *Node) {
  if (NmaxCache.count(Node)) {
    return NmaxCache[Node];
  } else {
    unsigned Result = Analysis(Node, NmaxCache, Transfer_Nmax, Join_Min, NULL);
    NmaxCache[Node] = Result;
    return Result;
  }
}

unsigned ControlFlowConstraintContext::getNusual(ControlTreeNode *Node) {
  if (NusualCache.count(Node)) {
    return NusualCache[Node];
  } else {
    unsigned Result = Analysis(Node, NusualCache, Transfer_Nusual, Join_Min,
                               Join_Max);
    NusualCache[Node] = Result;
    return Result;
  }
}

unsigned ControlFlowConstraintContext::getNstall(DataflowGraph *Graph) {
  if (NstallCache.count(Graph)) {
    return NstallCache[Graph];
  } else {
    unsigned Result = Analysis(Graph, Transfer_Nstall, Join_Min);
    NstallCache[Graph] = Result;
    return Result;
  }
}

// ControlFlowConstraint

ControlFlowConstraint::ControlFlowConstraint(unsigned wgGranularity,
                                             bool wgSingle,
                                             unsigned wgOrderingQueueSize,
                                             unsigned wiLimit,
                                             unsigned wiLimitIncrementQueueSize)
  : WGGranularity(wgGranularity), WGSingle(wgSingle),
    WGOrderingQueueSize(wgOrderingQueueSize), WILimit(wiLimit),
    WILimitIncrementQueueSize(wiLimitIncrementQueueSize) {
  assert(WGGranularity == 0 || (WGGranularity & (WGGranularity - 1)) == 0);
}

unsigned ControlFlowConstraint::getWorkGroupConstraintLSB() const {
  assert(WGGranularity != 0 && (WGGranularity & (WGGranularity - 1)) == 0);
  unsigned G = WGGranularity, LogG = 0;
  while (G > 1) {
    G >>= 1;
    LogG++;
  }
  return LogG;
}

void ControlFlowConstraint::Merge(const ControlFlowConstraint &RHS) {
  if (RHS.WGGranularity == 0) {
    // Do nothing
  } else if (WGGranularity == 0 ||
             WGGranularity > RHS.WGGranularity) {
    WGGranularity = RHS.WGGranularity;
  }
  if (WGSingle || RHS.WGSingle) {
    WGSingle = true;
    WGOrderingQueueSize = 0;
  } else {
    WGSingle = false;
    if (WGOrderingQueueSize < RHS.WGOrderingQueueSize) {
      WGOrderingQueueSize = RHS.WGOrderingQueueSize;
    }
  }

  if (RHS.WILimit == 0) {
    // Do nothing
  } else if (WILimit == 0 || WILimit > RHS.WILimit) {
    WILimit = RHS.WILimit;
  }
  if (WILimitIncrementQueueSize < RHS.WILimitIncrementQueueSize) {
    WILimitIncrementQueueSize = RHS.WILimitIncrementQueueSize;
  }
}

// DeadlockPrevention

void DeadlockPrevention::Synthesis() {
  assert(Program != NULL && cdfg != NULL);
  FindLoop(Program);
}

void DeadlockPrevention::FindLoop(ControlTreeNode *Root) {
  if (Root->isLoop()) {
    unsigned ShortestStallCycle = FindShortestStallCycle(Root);
    if (ShortestStallCycle > 0) {
      assert(ShortestStallCycle > 1);
      unsigned Nusual = Ctx.getNusual(Root);
      if (Root->hasBarrier()) {
        // Allowing up to 2^LIDWidth work-items is enough
        Nusual = Ctx.getWorkGroupSize();
      }
      if (ShortestStallCycle - 1 < Nusual) {
        Root->AddControlFlowConstraint(ControlFlowConstraint::WorkItemLimit(
            ShortestStallCycle - 1,
            Nusual - (ShortestStallCycle - 1)));
      } else {
        Root->AddControlFlowConstraint(ControlFlowConstraint::WorkItemLimit(
            ShortestStallCycle - 1));
      }
    }
  }

  for (ControlTreeNode::child_iterator I = Root->child_begin(),
                                       E = Root->child_end();
       I != E; ++I) {
    FindLoop(*I);
  }
}

unsigned DeadlockPrevention::FindShortestStallCycle(ControlTreeNode *Loop) {
  assert(Loop->isLoop());
  unsigned ShortestCycle = (unsigned)-1;

  llvm::DenseMap<DataflowGraph*, unsigned> Distance;
  llvm::DenseSet<DataflowGraph*> TraverseSet;
  Distance[Loop->getEntry()] = 0;
  TraverseSet.insert(Loop->getEntry());
  while (!TraverseSet.empty()) {
    DataflowGraph *Current = NULL;
    for (llvm::DenseSet<DataflowGraph*>::iterator I = TraverseSet.begin(),
                                                  E = TraverseSet.end();
         I != E; ++I) {
      assert(Distance.count(*I));
      if (Current == NULL || Distance[Current] > Distance[*I]) {
        Current = *I;
      }
    }
    assert(Current != NULL);
    TraverseSet.erase(Current);
    if (Current->hasBarrier()) {
      continue;
    }
    unsigned DistanceSucc = Distance[Current] + Ctx.getNstall(Current);
    for (unsigned Index = 0, NumSuccs = Current->getNumSuccs();
         Index != NumSuccs; ++Index) {
      if (DataflowGraph *Succ = Current->getSucc(cdfg, Index)) {
        if (Succ == Loop->getEntry()) {
          if (ShortestCycle > DistanceSucc) {
            ShortestCycle = DistanceSucc;
          }
        } else if (Loop->contains(Succ)) {
          if (!Distance.count(Succ) || Distance[Succ] > DistanceSucc) {
            Distance[Succ] = DistanceSucc;
            TraverseSet.insert(Succ);
          }
        }
      }
    }
  }
  // There could be no cycle if the loop body contains a barrier
  return ((ShortestCycle == (unsigned)-1) ? 0 : ShortestCycle);
}

// BarrierOrdering

void BarrierOrdering::Synthesis() {
  assert(Program != NULL);
  FindBarrier(Program);
}

void BarrierOrdering::FindBarrier(ControlTreeNode *Root) {
  if (CTBasicBlock *BB = dyn_cast<CTBasicBlock>(Root)) {
    if (BB->hasBarrier()) {
      EnforceOrder(Program, BB->getDFG());
    }
  } else {
    for (ControlTreeNode::child_iterator I = Root->child_begin(),
                                         E = Root->child_end();
         I != E; ++I) {
      if ((*I)->hasBarrier()) {
        FindBarrier(*I);
      }
    }
  }
}

void BarrierOrdering::EnforceOrder(ControlTreeNode *Root) {
  switch (Root->getClass()) {
    case ControlTreeNode::CTBasicBlockClass:
      break;

    case ControlTreeNode::CTSequentialClass: {
      CTSequential *Seq = static_cast<CTSequential*>(Root);
      EnforceOrder(Seq->getFirst());
      EnforceOrder(Seq->getSecond());
      break;
    }

    case ControlTreeNode::CTIfThenClass: {
      Root->AddControlFlowConstraint(ControlFlowConstraint::WorkGroupOrdering(
          1, Ctx.getNmax(Root)));
      CTIfThen *IfThen = static_cast<CTIfThen*>(Root);
      EnforceOrder(IfThen->getIf());
      EnforceOrder(IfThen->getThen());
      break;
    }

    case ControlTreeNode::CTIfThenElseClass: {
      Root->AddControlFlowConstraint(ControlFlowConstraint::WorkGroupOrdering(
          1, Ctx.getNmax(Root)));
      CTIfThenElse *IfThenElse = static_cast<CTIfThenElse*>(Root);
      EnforceOrder(IfThenElse->getIf());
      EnforceOrder(IfThenElse->getThen());
      EnforceOrder(IfThenElse->getElse());
      break;
    }

    case ControlTreeNode::CTSelfLoopClass: {
      CTSelfLoop *Loop = static_cast<CTSelfLoop*>(Root);
      if (Loop->hasFixedBound()) {
        EnforceOrder(Loop->getBody());
      } else {
        Root->AddControlFlowConstraint(ControlFlowConstraint::SingleWorkGroup(1));
      }
      break;
    }

    case ControlTreeNode::CTWhileLoopClass: {
      CTWhileLoop *Loop = static_cast<CTWhileLoop*>(Root);
      if (Loop->hasFixedBound()) {
        EnforceOrder(Loop->getCond());
        EnforceOrder(Loop->getBody());
      } else {
        Root->AddControlFlowConstraint(ControlFlowConstraint::SingleWorkGroup(1));
      }
      break;
    }

    default: {
      Root->AddControlFlowConstraint(ControlFlowConstraint::SingleWorkGroup(1));
      break;
    }
  }
}

void BarrierOrdering::EnforceOrder(ControlTreeNode *Root,
                                   DataflowGraph *Destination) {
  assert(Root->contains(Destination));
  switch (Root->getClass()) {
    case ControlTreeNode::CTBasicBlockClass:
      break;

    case ControlTreeNode::CTSequentialClass: {
      CTSequential *Seq = static_cast<CTSequential*>(Root);
      ControlTreeNode *First = Seq->getFirst();
      ControlTreeNode *Second = Seq->getSecond();
      if (First->contains(Destination)) {
        EnforceOrder(First, Destination);
      } else {
        EnforceOrder(First);
        EnforceOrder(Second, Destination);
      }
      break;
    }

    case ControlTreeNode::CTIfThenClass: {
      CTIfThen *IfThen = static_cast<CTIfThen*>(Root);
      ControlTreeNode *If = IfThen->getIf();
      ControlTreeNode *Then = IfThen->getThen();
      if (If->contains(Destination)) {
        EnforceOrder(If, Destination);
      } else {
        EnforceOrder(If);
        EnforceOrder(Then, Destination);
      }
      break;
    }

    case ControlTreeNode::CTIfThenElseClass: {
      CTIfThenElse *IfThenElse = static_cast<CTIfThenElse*>(Root);
      ControlTreeNode *If = IfThenElse->getIf();
      ControlTreeNode *Then = IfThenElse->getThen();
      ControlTreeNode *Else = IfThenElse->getElse();
      if (If->contains(Destination)) {
        EnforceOrder(If, Destination);
      } else if (Then->contains(Destination)) {
        EnforceOrder(If);
        EnforceOrder(Then, Destination);
      } else {
        EnforceOrder(If);
        EnforceOrder(Else, Destination);
      }
      break;
    }

    case ControlTreeNode::CTSelfLoopClass: {
      CTSelfLoop *Loop = static_cast<CTSelfLoop*>(Root);
      Root->AddControlFlowConstraint(ControlFlowConstraint::SingleWorkGroup(1));
      break;
    }

    case ControlTreeNode::CTWhileLoopClass: {
      CTWhileLoop *Loop = static_cast<CTWhileLoop*>(Root);
      Root->AddControlFlowConstraint(ControlFlowConstraint::SingleWorkGroup(1));
      break;
    }

    default: {
      Root->AddControlFlowConstraint(ControlFlowConstraint::SingleWorkGroup(1));
      break;
    }
  }
}

void LocalMemoryOrdering::Synthesis() {
  assert(Program != NULL);
  HasLocalMemoryAccess = false;
  FindLocalMemory(Program);
  if (HasLocalMemoryAccess) {
    unsigned GoodNumWorkItems = Ctx.getNusual(Program);
    unsigned GoodNumWorkGroups = 1;
    while (GoodNumWorkGroups < GoodNumWorkItems / Ctx.getWorkGroupSize()) {
      GoodNumWorkGroups *= 2;
    }
    Program->AddControlFlowConstraint(ControlFlowConstraint::SingleWorkGroup(
        GoodNumWorkGroups));
  }
}

void LocalMemoryOrdering::FindLocalMemory(ControlTreeNode *Root) {
  if (CTBasicBlock *BB = dyn_cast<CTBasicBlock>(Root)) {
    DataflowGraph *dfg = BB->getDFG();
    for (DataflowGraph::iterator I = dfg->begin(), E = dfg->end();
         I != E; ++I) {
      if (DFGMemoryAccessNode *Access = dyn_cast<DFGMemoryAccessNode>(*I)) {
        if (Access->getAddressSpace() == LangAS::opencl_local) {
          HasLocalMemoryAccess = true;
        }
      }
    }
  } else {
    for (ControlTreeNode::child_iterator I = Root->child_begin(),
                                         E = Root->child_end();
         I != E; ++I) {
      FindLocalMemory(*I);
    }
  }
}

} // namespace snu

} // namespace clang
