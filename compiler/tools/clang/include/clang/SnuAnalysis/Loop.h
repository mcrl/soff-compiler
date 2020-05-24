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

#ifndef LLVM_CLANG_SNU_ANALYSIS_LOOP_H
#define LLVM_CLANG_SNU_ANALYSIS_LOOP_H

#include "clang/Basic/LLVM.h"
#include "clang/AST/ASTContext.h"
#include "clang/AST/OperationKinds.h"
#include "clang/SnuAST/WAST.h"
#include "clang/SnuAST/WCFG.h"
#include "clang/SnuAnalysis/Invariance.h"
#include "llvm/ADT/DenseSet.h"
#include "llvm/ADT/SmallVector.h"

namespace clang {

namespace snu {

class Loop {
  WCFGBlock *Head;
  WCFGBlock *Tail;
  llvm::DenseSet<WCFGBlock*> Body;

public:
  struct InductionVariable {
    IndexedVarDecl *IndVar;
    IndexedVarDecl *IncVar;
    IndexedVarDecl *InitVar;
    // WARNING: Step could be (WExpr*)0x1
    WExpr *Step; BinaryOperatorKind StepOp;
    WExpr *Bound; BinaryOperatorKind BoundOp;

    InductionVariable()
      : IndVar(NULL), IncVar(NULL), InitVar(NULL), Step(NULL), StepOp(BO_Comma),
        Bound(NULL), BoundOp(BO_Comma) {}
    InductionVariable(IndexedVarDecl *ind, IndexedVarDecl *inc,
                      IndexedVarDecl *init, WExpr *step,
                      BinaryOperatorKind stepOp, WExpr *bound,
                      BinaryOperatorKind boundOp)
      : IndVar(ind), IncVar(inc), InitVar(init), Step(step), StepOp(stepOp),
        Bound(bound), BoundOp(boundOp) {}

    operator bool() const {
      return (IndVar != NULL);
    }

    WExpr *getOrAllocStep(ASTContext &Ctx, WCFG *Program) const {
      if (Step == (WExpr*)0x1) {
        return new (*Program) WIntegerLiteral(Ctx, 1, IndVar->getType());
      } else {
        return Step;
      }
    }
    bool hasBound() const { return Bound != NULL; }
    bool isIncrement() const { return StepOp == BO_Add; }
    bool isInclusive() const { return BoundOp == BO_LE || BoundOp == BO_GE; }
  };

private:
  unsigned NumLoopExits;
  SmallVector<InductionVariable, 16> IndVars;
  bool HasBarrier;
  bool HasFixedBound;

  bool ValidNumLoopExits;
  bool ValidIndVars;
  bool ValidHasBarrier;
  bool ValidHasFixedBound;

public:
  Loop(WCFGBlock *head, WCFGBlock *tail);

  WCFG *getParent() const { return Head->getParent(); }
  WCFGBlock *getHead() const { return Head; }
  WCFGBlock *getTail() const { return Tail; }
  WStmt *getLoopStmt() const { return Tail->getLoopTarget(); }
  const llvm::DenseSet<WCFGBlock*> &getBody() const { return Body; }

  typedef llvm::DenseSet<WCFGBlock*>::const_iterator body_iterator;
  body_iterator body_begin() const { return Body.begin(); }
  body_iterator body_end() const { return Body.end(); }

  bool isLoopBody(WCFGBlock *B) const { return Body.count(B); }

  unsigned getNumLoopExits() {
    if (!ValidNumLoopExits) ComputeNumLoopExits();
    return NumLoopExits;
  }
  bool hasBarrier() {
    if (!ValidHasBarrier) ComputeHasBarrier();
    return HasBarrier;
  }
  bool hasFixedBound() {
    if (!ValidHasFixedBound) ComputeHasFixedBound();
    return HasFixedBound;
  }

  typedef SmallVectorImpl<InductionVariable>::const_iterator ind_var_iterator;
  ind_var_iterator ind_var_begin() {
    if (!ValidIndVars) ComputeInductionVariables();
    return IndVars.begin();
  }
  ind_var_iterator ind_var_end() {
    if (!ValidIndVars) ComputeInductionVariables();
    return IndVars.end();
  }

  void ComputeNumLoopExits();
  void ComputeInductionVariables(AdditionalInvariance *AInv = NULL);
  void ComputeHasBarrier();
  void ComputeHasFixedBound();
};

template<typename Derived>
class LoopVisitor {
  Derived &getDerived() { return *static_cast<Derived*>(this); }

public:
  void Visit(WCFG *cfg) {
    for (WCFG::const_iterator B = cfg->begin(), BEnd = cfg->end();
         B != BEnd; ++B) {
      WCFGBlock *Tail = *B;
      if (Tail->getLoopTarget() != NULL) {
        assert(Tail->real_succ_size() == 1);
        WCFGBlock *Head = *(Tail->real_succ_begin());
        assert(Head != NULL);
        Loop L(Head, Tail);
        getDerived().VisitLoop(L);
      }
    }
  }
};

} // namespace snu

} // namespace clang

#endif // LLVM_CLANG_SNU_ANALYSIS_LOOP_H
