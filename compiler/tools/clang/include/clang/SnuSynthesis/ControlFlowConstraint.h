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

#ifndef LLVM_CLANG_SNU_SYNTHESIS_CONTROLFLOWCONSTRAINT_H
#define LLVM_CLANG_SNU_SYNTHESIS_CONTROLFLOWCONSTRAINT_H

#include "clang/Basic/LLVM.h"
#include "clang/SnuAST/WAST.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/StringRef.h"

namespace clang {

namespace snu {

class ControlDataflowGraph;
class ControlTreeNode;
class DataflowGraph;
class DFGNode;
class PlatformContext;

class ControlFlowConstraintContext {
  PlatformContext &PCtx;

  typedef llvm::DenseMap<DataflowGraph*, unsigned> BBCacheTy;
  typedef llvm::DenseMap<ControlTreeNode*, unsigned> CTCacheTy;
  CTCacheTy NmaxCache;
  CTCacheTy NusualCache;
  BBCacheTy NstallCache;

public:
  explicit ControlFlowConstraintContext(PlatformContext &C) : PCtx(C) {}

  unsigned getWorkGroupSize();

  // # of work-items in different situations
  // N_max: (near-)max # of work-items
  // N_usual: # of work-items in a peaceful case
  // N_stall: min # of work-items when the basic block strongly stalls
  unsigned getNmax(ControlTreeNode *Node);
  unsigned getNusual(ControlTreeNode *Node);
  unsigned getNstall(DataflowGraph *Graph);

private:
  unsigned Analysis(DataflowGraph *Graph,
                    unsigned (*Transfer)(unsigned, PlatformContext&, DFGNode*),
                    unsigned (*Join)(unsigned, unsigned));
  unsigned Analysis(ControlTreeNode *Node, CTCacheTy &Cache,
                    unsigned (*Transfer)(unsigned, PlatformContext&, DFGNode*),
                    unsigned (*Join_Op)(unsigned, unsigned),
                    unsigned (*Join_Branch)(unsigned, unsigned));
};

class ControlFlowConstraint {
  // Work-group-level constraints
  unsigned WGGranularity;
  bool WGSingle;
  unsigned WGOrderingQueueSize;

  // Work-item-level constraints
  unsigned WILimit;
  unsigned WILimitIncrementQueueSize;

  ControlFlowConstraint(unsigned wgGranularity, bool wgSingle,
                        unsigned wgOrderingQueueSize, unsigned wiLimit,
                        unsigned wiLimitIncrementQueueSize);

public:
  ControlFlowConstraint()
    : WGGranularity(0), WGSingle(false), WGOrderingQueueSize(0),
      WILimit(0), WILimitIncrementQueueSize(0) {}

  bool hasSingleWorkGroupConstraint() const {
    return WGSingle;
  }
  bool hasWorkGroupOrderingConstraint() const {
    return WGOrderingQueueSize > 0;
  }
  unsigned getWorkGroupGranularity() const {
    return WGGranularity;
  }
  unsigned getWorkGroupOrderingQueueSize() const {
    return WGOrderingQueueSize;
  }
  unsigned getWorkGroupConstraintLSB() const;

  bool hasWorkItemLimitConstraint() const {
    return WILimit > 0;
  }
  unsigned getWorkItemLimit() const {
    assert(hasWorkItemLimitConstraint());
    return WILimit;
  }
  unsigned getWorkItemLimitIncrementQueueSize() const {
    assert(hasWorkItemLimitConstraint());
    return WILimitIncrementQueueSize;
  }

  void Merge(const ControlFlowConstraint &RHS);

  static ControlFlowConstraint SingleWorkGroup(unsigned Granularity) {
    return ControlFlowConstraint(Granularity, true, 0, 0, 0);
  }
  static ControlFlowConstraint WorkGroupOrdering(unsigned Granularity,
                                                 unsigned QueueSize) {
    return ControlFlowConstraint(Granularity, false, QueueSize, 0, 0);
  }
  static ControlFlowConstraint WorkItemLimit(unsigned Limit,
                                             unsigned Increment = 0) {
    return ControlFlowConstraint(0, false, 0, Limit, Increment);
  }
};

class DeadlockPrevention {
  ControlFlowConstraintContext &Ctx;
  ControlTreeNode *Program;
  ControlDataflowGraph *cdfg;

public:
  DeadlockPrevention(ControlFlowConstraintContext &C, ControlTreeNode *P,
                     ControlDataflowGraph *cdfg_)
    : Ctx(C), Program(P), cdfg(cdfg_) {}

  void Synthesis();

private:
  void FindLoop(ControlTreeNode *Root);
  unsigned FindShortestStallCycle(ControlTreeNode *Loop);
};

class BarrierOrdering {
  ControlFlowConstraintContext &Ctx;
  ControlTreeNode *Program;

public:
  BarrierOrdering(ControlFlowConstraintContext &C, ControlTreeNode *P)
    : Ctx(C), Program(P) {}

  void Synthesis();

private:
  void FindBarrier(ControlTreeNode *Root);
  void EnforceOrder(ControlTreeNode *Root);
  void EnforceOrder(ControlTreeNode *Root, DataflowGraph *Destination);
};

class LocalMemoryOrdering {
  ControlFlowConstraintContext &Ctx;
  ControlTreeNode *Program;
  bool HasLocalMemoryAccess;

public:
  LocalMemoryOrdering(ControlFlowConstraintContext &C, ControlTreeNode *P)
    : Ctx(C), Program(P) {}

  void Synthesis();

private:
  void FindLocalMemory(ControlTreeNode *Root);
};

} // namespace snu

} // namespace clang

#endif // LLVM_CLANG_SNU_SYNTHESIS_CONTROLFLOWCONSTRAINT_H
