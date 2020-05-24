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

#ifndef LLVM_CLANG_SNU_SYNTHESIS_SCHEDULER_H
#define LLVM_CLANG_SNU_SYNTHESIS_SCHEDULER_H

#include "clang/Basic/LLVM.h"
#include "clang/SnuSynthesis/DataflowGraph.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/DenseSet.h"
#include "llvm/ADT/SmallVector.h"
#include <lpsolve/lp_lib.h>

namespace clang {

namespace snu {

class PlatformContext;

class SchedulerBase {
protected:
  PlatformContext &PCtx;

  llvm::DenseMap<DFGNode*, unsigned> Tstart;
  llvm::DenseMap<DFGNode*, unsigned> Tend;

  explicit SchedulerBase(PlatformContext &Ctx)
    : PCtx(Ctx) {}

public:
  void ScheduleAll(ControlDataflowGraph *Graph);
  virtual void Schedule(DataflowGraph *Graph) = 0;

  unsigned GetTstart(DFGNode *Node) const;
  unsigned GetTend(DFGNode *Node) const;
  unsigned GetMaxPredTend(DFGNode *Node) const;
  unsigned GetMinSuccTstart(DFGNode *Node) const;

  void DelayNode(DFGNode *Node, unsigned Shift);
  void RegisterNewNode(DFGNode *Node, unsigned Ts);

protected:
  bool RescheduleASAP(DataflowGraph *Graph);
  void ScheduleILP(DataflowGraph *Graph, const ASTContext &ASTCtx);
  void Synthesis(DataflowGraph *Graph);

  DFGQueueNode *InsertQueue(DataflowGraph *Graph, DFGNode *Pred, DFGNode *Succ,
                            unsigned Size);
  DFGQueueNode *AppendQueue(DataflowGraph *Graph, DFGNode *Pred, unsigned Size);
};

class DefaultScheduler : public SchedulerBase {
  const ASTContext &ASTCtx;

public:
  explicit DefaultScheduler(PlatformContext &PC, const ASTContext &AC)
    : SchedulerBase(PC), ASTCtx(AC) {}

  virtual void Schedule(DataflowGraph *Graph);
};

} // namespace snu

} // namespace clang

#endif // LLVM_CLANG_SNU_SYNTHESIS_SCHEDULER_H
