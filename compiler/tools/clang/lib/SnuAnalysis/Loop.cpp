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

#include "clang/SnuAnalysis/Loop.h"
#include "clang/AST/Expr.h"
#include "clang/Basic/LLVM.h"
#include "clang/SnuAST/WAST.h"
#include "clang/SnuAST/WCFG.h"
#include "clang/SnuAnalysis/Invariance.h"
#include "llvm/ADT/DenseSet.h"
#include "llvm/ADT/SmallVector.h"
#include <utility>

namespace clang {

namespace snu {

Loop::Loop(WCFGBlock *head, WCFGBlock *tail)
  : Head(head), Tail(tail), NumLoopExits(0), HasBarrier(false),
    HasFixedBound(false), ValidNumLoopExits(false), ValidIndVars(false),
    ValidHasBarrier(false), ValidHasFixedBound(false) {
  // S. Muchnick, Advanced Compiler Design and Implementation, 1997, p. 192.
  Body.insert(Tail);
  Body.insert(Head);
  SmallVector<WCFGBlock*, 256> TraverseQueue;
  TraverseQueue.push_back(Tail);
  while (!TraverseQueue.empty()) {
    WCFGBlock *B = TraverseQueue.back();
    TraverseQueue.pop_back();
    for (WCFGBlock::pred_iterator P = B->pred_begin(), PEnd = B->pred_end();
         P != PEnd; ++P) {
      if (!Body.count(*P)) {
        Body.insert(*P);
        TraverseQueue.push_back(*P);
      }
    }
  }
}

void Loop::ComputeNumLoopExits() {
  NumLoopExits = 0;
  for (llvm::DenseSet<WCFGBlock*>::iterator B = Body.begin(), BEnd = Body.end();
       B != BEnd; ++B) {
    WCFGBlock *Block = *B;
    for (WCFGBlock::succ_iterator S = Block->succ_begin(),
                                  SEnd = Block->succ_end();
         S != SEnd; ++S) {
      if (!Body.count(*S)) {
        NumLoopExits++;
      }
    }
  }
  ValidNumLoopExits = true;
}

namespace {

class IVDetector {
  WCFG *Program;
  WStmt *Loop;
  WCFGBlock *Head;
  RegionInvariance Inv;

  typedef Loop::InductionVariable InductionVariable;

public:
  IVDetector(WCFG *program, WStmt *loop, WCFGBlock *head,
             const llvm::DenseSet<WCFGBlock*> &body,
             AdditionalInvariance *AInv = NULL)
    : Program(program), Loop(loop), Head(head), Inv(program, body, AInv) {}

  void Run(SmallVectorImpl<InductionVariable> &Output);
  InductionVariable GetInductionVariableFrom(WPhiFunction *PF);
  std::pair<WExpr*, BinaryOperatorKind> GetInductionStep(IndexedVarDecl *IncVar,
                                                         IndexedVarDecl *IndVar);
  std::pair<WExpr*, BinaryOperatorKind> GetInductionBound(IndexedVarDecl *IndVar,
                                                          WExpr *Cond);
};

void IVDetector::Run(SmallVectorImpl<InductionVariable> &Output) {
  Output.clear();
  for (WCFGBlock::iterator S = Head->begin(), SEnd = Head->end();
       S != SEnd; ++S) {
    WPhiFunction *PF = dyn_cast<WPhiFunction>(*S);
    if (PF == NULL) {
      break;
    }
    if (InductionVariable IndVar = GetInductionVariableFrom(PF)) {
      Output.push_back(IndVar);
    }
  }
}

Loop::InductionVariable IVDetector::GetInductionVariableFrom(WPhiFunction *PF) {
  // PF is IndVar = phi(IncVar, InitVar, InitVar, ..., InitVar)
  assert(PF->getNumArgs() >= 2);
  IndexedVarDecl *IndVar = PF->getIndexedLHSDecl();
  IndexedVarDecl *IncVar = PF->getIndexedArgDecl(0);
  IndexedVarDecl *InitVar = PF->getIndexedArgDecl(1);
  assert(IndVar != NULL);
  if (IncVar == NULL || InitVar == NULL) {
    return InductionVariable();
  }
  for (unsigned Index = 2, NumArgs = PF->getNumArgs();
       Index != NumArgs; ++Index) {
    if (PF->getIndexedArgDecl(Index) != InitVar) {
      return InductionVariable();
    }
  }
  // IndVar is an integer or a pointer
  if (!IndVar->getType()->isIntegerType() && !IndVar->getType()->isPointerType()) {
    return InductionVariable();
  }
  // IndVar and IncVar are loop-variant while InitVar is loop-invariant
  if (Inv.isInvariant(IndVar) || Inv.isInvariant(IncVar) || !Inv.isInvariant(InitVar)) {
    return InductionVariable();
  }

  std::pair<WExpr*, BinaryOperatorKind> Step = GetInductionStep(IncVar, IndVar);
  if (Step.first == NULL) {
    return InductionVariable();
  }
  std::pair<WExpr*, BinaryOperatorKind> Bound;
  if (Loop == NULL) {
    Bound = std::pair<WExpr*, BinaryOperatorKind>(NULL, BO_Comma);
  } else if (WWhileStmt *While = dyn_cast<WWhileStmt>(Loop)) {
    Bound = GetInductionBound(IndVar, While->getCond());
  } else if (WForStmt *For = dyn_cast<WForStmt>(Loop)) {
    Bound = GetInductionBound(IndVar, For->getCond());
  } else {
    // We currently do not handle do-while loops
    Bound = std::pair<WExpr*, BinaryOperatorKind>(NULL, BO_Comma);
  }
  return InductionVariable(IndVar, IncVar, InitVar, Step.first, Step.second,
                           Bound.first, Bound.second);
}

IndexedVarDecl *GetIndexedUseDecl(WExpr *E) {
  E = E->IgnoreParenCasts();
  if (WDeclRefExpr *DRE = dyn_cast<WDeclRefExpr>(E)) {
    if (IndexedVarDeclRef DRERef = DRE->getIndexedUseDecl()) {
      return DRERef.getSingleVar();
    }
  } else if (WMemberExpr *ME = dyn_cast<WMemberExpr>(E)) {
    if (IndexedVarDeclRef MERef = ME->getIndexedUseDecl()) {
      return MERef.getSingleVar();
    }
  } else if (WExtVectorElementExpr *VEE = dyn_cast<WExtVectorElementExpr>(E)) {
    if (IndexedVarDeclRef VEERef = VEE->getIndexedUseDecl()) {
      return VEERef.getSingleVar();
    }
  }
  return NULL;
}

std::pair<WExpr*, BinaryOperatorKind> IVDetector::GetInductionStep(
    IndexedVarDecl *IncVar, IndexedVarDecl *IndVar) {
  WStmt *IncStmt = IncVar->getDefinedStmt();
  if (IncStmt == NULL) {
    return std::pair<WExpr*, BinaryOperatorKind>(NULL, BO_Comma);
  }

  if (WUnaryOperator *UO = dyn_cast<WUnaryOperator>(IncStmt)) {
    assert(UO->isIncrementDecrementOp());
    if (GetIndexedUseDecl(UO->getSubExpr()) == IndVar) {
      // (IndVar -> IncVar)++
      BinaryOperatorKind Op = (UO->isIncrementOp() ? BO_Add : BO_Sub);
      return std::pair<WExpr*, BinaryOperatorKind>((WExpr*)0x1, Op);
    }
  } else if (WCompoundAssignOperator *CAO = dyn_cast<WCompoundAssignOperator>(IncStmt)) {
    if (CAO->getOpcode() == BO_AddAssign || CAO->getOpcode() == BO_SubAssign) {
      if (GetIndexedUseDecl(CAO->getLHS()) == IndVar && Inv.isInvariant(CAO->getRHS())) {
        // (IndVar -> IncVar) += loop-invariant expression
        BinaryOperatorKind Op = (CAO->getOpcode() == BO_AddAssign ? BO_Add : BO_Sub);
        return std::pair<WExpr*, BinaryOperatorKind>(CAO->getRHS(), Op);
      }
    }
  } else if (WBinaryOperator *BO = dyn_cast<WBinaryOperator>(IncStmt)) {
    if (BO->getOpcode() == BO_Add) {
      if (GetIndexedUseDecl(BO->getLHS()) == IndVar && Inv.isInvariant(BO->getRHS())) {
        // IncVar = IndVar + loop-invariant expression
        return std::pair<WExpr*, BinaryOperatorKind>(BO->getRHS(), BO_Add);
      }
      if (GetIndexedUseDecl(BO->getRHS()) == IndVar && Inv.isInvariant(BO->getLHS())) {
        // IncVar = loop-invariant expression + IndVar
        return std::pair<WExpr*, BinaryOperatorKind>(BO->getLHS(), BO_Add);
      }
    } else if (BO->getOpcode() == BO_Sub) {
      if (GetIndexedUseDecl(BO->getLHS()) == IndVar && Inv.isInvariant(BO->getRHS())) {
        // IncVar = IndVar - loop-invariant expression
        return std::pair<WExpr*, BinaryOperatorKind>(BO->getRHS(), BO_Sub);
      }
    }
  }
  return std::pair<WExpr*, BinaryOperatorKind>(NULL, BO_Comma);
}

std::pair<WExpr*, BinaryOperatorKind> IVDetector::GetInductionBound(
    IndexedVarDecl *IndVar, WExpr *Cond) {
  if (Cond == NULL) {
    return std::pair<WExpr*, BinaryOperatorKind>(NULL, BO_Comma);
  }
  Cond = Cond->IgnoreParenCasts();
  if (WBinaryOperator *BO = dyn_cast<WBinaryOperator>(Cond)) {
    if (BO->getOpcode() == BO_LT || BO->getOpcode() == BO_GT ||
        BO->getOpcode() == BO_LE || BO->getOpcode() == BO_GE ||
        BO->getOpcode() == BO_NE) {
      if (GetIndexedUseDecl(BO->getLHS()) == IndVar && Inv.isInvariant(BO->getRHS())) {
        // IndVar < loop-invariant expression
        return std::pair<WExpr*, BinaryOperatorKind>(BO->getRHS(), BO->getOpcode());
      }
      if (GetIndexedUseDecl(BO->getRHS()) == IndVar && Inv.isInvariant(BO->getLHS())) {
        // loop-invariant expression > IndVar
        BinaryOperatorKind Op;
        switch (BO->getOpcode()) {
        case BO_LT: Op = BO_GT; break; // E < V => V > E
        case BO_GT: Op = BO_LT; break; // E > V => V < E
        case BO_LE: Op = BO_GE; break; // E <= V => V >= E
        case BO_GE: Op = BO_LE; break; // E >= V => V <= E
        case BO_NE: Op = BO_NE; break; // E != V => V != E
        default:
          llvm_unreachable("invalid opcode for a loop bound");
        }
        return std::pair<WExpr*, BinaryOperatorKind>(BO->getLHS(), Op);
      }
    }
  }
  return std::pair<WExpr*, BinaryOperatorKind>(NULL, BO_Comma);
}

} // anonymous namespace

void Loop::ComputeInductionVariables(AdditionalInvariance *AInv) {
  if (getNumLoopExits() == 1) {
    IVDetector Impl(getParent(), getLoopStmt(), Head, Body, AInv);
    Impl.Run(IndVars);
  }
  ValidIndVars = true;
}

void Loop::ComputeHasBarrier() {
  HasBarrier = false;
  for (llvm::DenseSet<WCFGBlock*>::iterator B = Body.begin(), BEnd = Body.end();
       B != BEnd; ++B) {
    if ((*B)->hasBarrier()) {
      HasBarrier = true;
      break;
    }
  }
  ValidHasBarrier = true;
}

void Loop::ComputeHasFixedBound() {
  WorkItemInvariance Inv(getParent());
  HasFixedBound = false;
  for (ind_var_iterator I = ind_var_begin(), E = ind_var_end(); I != E; ++I) {
    if (Inv.isInvariant(I->InitVar) &&
        (I->Step == (WExpr*)0x1 || Inv.isInvariant(I->Step)) &&
        I->hasBound() && Inv.isInvariant(I->Bound)) {
      HasFixedBound = true;
      break;
    }
  }
  ValidHasFixedBound = true;
}

} // namespace snu

} // namespace clang
