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

#include "clang/SnuAnalysis/Invariance.h"
#include "clang/Basic/LLVM.h"
#include "clang/SnuAST/WAST.h"
#include "clang/SnuAST/WCFG.h"
#include "llvm/ADT/DenseSet.h"

namespace clang {

namespace snu {

InvarianceImplBase::InvarianceImplBase(WCFG *program, AdditionalInvariance *AI)
  : Program(program), AInv(AI) {
}

void InvarianceImplBase::ComputeInvariantVars() {
  bool Updated;
  do {
    Updated = false;
    for (WCFG::ssa_var_iterator V = Program->ssa_var_begin(),
                                VEnd = Program->ssa_var_end();
         V != VEnd; ++V) {
      IndexedVarDecl *Var = *V;
      if (!InvariantVars.count(Var)) {
        if (Var->getAnyDefinedStmt() &&
            isInvariantDefinition(Var->getAnyDefinedStmt())) {
          InvariantVars.insert(Var);
          Updated = true;
        }
      }
    }
  } while (Updated);
}

bool InvarianceImplBase::isInvariantDefinition(WStmt *S) {
  assert(S != NULL);

  if (WDeclStmt *DS = dyn_cast<WDeclStmt>(S)) {
    assert(DS->hasSingleInit());
    return isInvariant(DS->getSingleInit());

  } else if (WUnaryOperator *UO = dyn_cast<WUnaryOperator>(S)) {
    assert(UO->isIncrementDecrementOp());
    return isInvariant(UO->getSubExpr());

  } else if (WCompoundAssignOperator *CAO =
               dyn_cast<WCompoundAssignOperator>(S)) {
    return isInvariant(CAO->getLHS()) && isInvariant(CAO->getRHS());

  } else if (WBinaryOperator *BO = dyn_cast<WBinaryOperator>(S)) {
    assert(BO->isAssignmentOp());
    return isInvariant(BO->getRHS());

  } else if (isa<WPhiFunction>(S)) {
    return false;

  } else {
    llvm_unreachable("invalid definition");
  }
}

bool InvarianceImplBase::isInvariant(WExpr *E) {
  if (!E) {
    return false;
  }
  if (AInv && AInv->handleExpr(E)) {
    return true;
  }

  if (WDeclRefExpr *DRE = dyn_cast<WDeclRefExpr>(E)) {
    if (IndexedVarDeclRef IVarRef = DRE->getIndexedUseDecl()) {
      return isInvariant(IVarRef);
    }
    return false;

  } else if (WUnaryOperator *UO = dyn_cast<WUnaryOperator>(E)) {
    if (UO->getOpcode() == UO_Deref) {
      return false;
    } else {
      return isInvariant(UO->getSubExpr());
    }

  } else if (isa<WUnaryExprOrTypeTraitExpr>(E)) {
    return true;

  } else if (isa<WArraySubscriptExpr>(E)) {
    return false;

  } else if (WCallExpr *CE = dyn_cast<WCallExpr>(E)) {
    if (CE->isBuiltin() && !CE->hasSideEffect()) {
      for (unsigned Index = 0, NumArgs = CE->getNumArgs();
           Index != NumArgs; ++Index) {
        if (!isInvariant(CE->getArg(Index))) {
          return false;
        }
      }
      return true;
    } else {
      return false;
    }

  } else if (WMemberExpr *ME = dyn_cast<WMemberExpr>(E)) {
    if (ME->isArrow()) {
      return false;
    } else {
      if (IndexedVarDeclRef IVarRef = ME->getIndexedUseDecl()) {
        return isInvariant(IVarRef);
      } else {
        return isInvariant(ME->getBase());
      }
    }

  } else if (WBinaryOperator *BO = dyn_cast<WBinaryOperator>(E)) {
    if (BO->isAssignmentOp() && !BO->isCompoundAssignmentOp()) {
      return isInvariant(BO->getRHS());
    } else {
      return isInvariant(BO->getLHS()) && isInvariant(BO->getRHS());
    }

  } else if (WExtVectorElementExpr *VEE = dyn_cast<WExtVectorElementExpr>(E)) {
    if (IndexedVarDeclRef IVarRef = VEE->getIndexedUseDecl()) {
      return isInvariant(IVarRef);
    } else {
      return isInvariant(VEE->getBase());
    }

  } else if (WWorkItemFunction *WIF = dyn_cast<WWorkItemFunction>(E)) {
    return false;

  } else {
    for (WStmt::child_iterator C = E->child_begin(), CEnd = E->child_end();
         C != CEnd; ++C) {
      if (!isInvariant(dyn_cast<WExpr>(*C))) {
        return false;
      }
    }
    return true;
  }
}

bool InvarianceImplBase::isInvariant(IndexedVarDeclRef IVarRef) {
  if (IVarRef.isSingleVar()) {
    return InvariantVars.count(*IVarRef);
  } else {
    assert(IVarRef.isCompound());
    for (unsigned Index = 0, NumSubVars = IVarRef.getNumSubVars();
         Index != NumSubVars; ++Index) {
      if (InvariantVars.count(IVarRef[Index]) == 0) {
        return false;
      }
    }
    return true;
  }
}

namespace {

class WorkItemFunctionInvariance : public AdditionalInvariance {
  AdditionalInvariance *Next;

public:
  WorkItemFunctionInvariance() : Next(NULL) {}
  explicit WorkItemFunctionInvariance(AdditionalInvariance *next) : Next(next) {}

  virtual bool handleExpr(WExpr *E) {
    if (WWorkItemFunction *WIF = dyn_cast<WWorkItemFunction>(E)) {
      switch (WIF->getFunctionKind()) {
        case WWorkItemFunction::WIF_get_global_size:
        case WWorkItemFunction::WIF_get_local_size:
        case WWorkItemFunction::WIF_get_num_groups:
          return true;
        default:
          break;
      }
    }
    return (Next != NULL ? Next->handleExpr(E) : false);
  }
};

} // anonymous namespace

WorkItemInvariance::WorkItemInvariance(WCFG *program, AdditionalInvariance *AI)
  : InvarianceImplBase(program, new WorkItemFunctionInvariance(AI)) {
  assert(AInv != NULL);
  for (WCFG::ssa_var_iterator V = Program->ssa_var_begin(),
                              VEnd = Program->ssa_var_end();
       V != VEnd; ++V) {
    if ((*V)->isParameter()) {
      InvariantVars.insert(*V);
    }
  }
  ComputeInvariantVars();
}

WorkItemInvariance::~WorkItemInvariance() {
  delete AInv;
}

RegionInvariance::RegionInvariance(WCFG *program,
                                   const llvm::DenseSet<WCFGBlock*> &Region,
                                   AdditionalInvariance *AI)
  : InvarianceImplBase(program, AI) {
  for (WCFG::ssa_var_iterator V = Program->ssa_var_begin(),
                              VEnd = Program->ssa_var_end();
       V != VEnd; ++V) {
    InvariantVars.insert(*V);
  }
  for (llvm::DenseSet<WCFGBlock*>::const_iterator B = Region.begin(),
                                                  BEnd = Region.end();
       B != BEnd; ++B) {
    WCFGBlock *Block = *B;
    for (WCFGBlock::iterator I = Block->begin(), E = Block->end();
         I != E; ++I) {
      ExcludeDefinedVariables(*I, true);
    }
  }
  ComputeInvariantVars();
}

void RegionInvariance::ExcludeDefinedVariables(WStmt *S, bool Outermost) {
  if (!S)
    return;

  if (WDeclStmt *DS = dyn_cast<WDeclStmt>(S)) {
    assert(DS->isSingleDecl());
    if (IndexedVarDeclRef IVarRef = DS->getIndexedDecl()) {
      ExcludeDefinedVariable(IVarRef);
    }
    if (DS->hasSingleInit()) {
      ExcludeDefinedVariables(DS->getSingleInit());
    }

  } else if (WDeclRefExpr *DRE = dyn_cast<WDeclRefExpr>(S)) {
    if (IndexedVarDeclRef IVarRef = DRE->getIndexedDefDecl()) {
      ExcludeDefinedVariable(IVarRef);
    }

  } else if (WCallExpr *CE = dyn_cast<WCallExpr>(S)) {
    if (Outermost) {
      ExcludeDefinedVariables(CE->getCallee());
      for (unsigned Index = 0, NumArgs = CE->getNumArgs();
           Index != NumArgs; ++Index) {
        ExcludeDefinedVariables(CE->getArg(Index));
      }
    }

  } else if (WMemberExpr *ME = dyn_cast<WMemberExpr>(S)) {
    if (ME->isArrow()) {
      ExcludeDefinedVariables(ME->getBase());
    } else {
      if (IndexedVarDeclRef IVarRef = ME->getIndexedDefDecl()) {
        ExcludeDefinedVariable(IVarRef);
      }
      ExcludeDefinedVariables(ME->getBase());
    }

  } else if (WBinaryOperator *BO = dyn_cast<WBinaryOperator>(S)) {
    if (BO->isLogicalOp() || BO->getOpcode() == BO_Comma) {
      // Do nothing
    } else {
      ExcludeDefinedVariables(BO->getLHS());
      ExcludeDefinedVariables(BO->getRHS());
    }

  } else if (isa<WConditionalOperator>(S)) {
    // Do nothing

  } else if (WExtVectorElementExpr *VEE = dyn_cast<WExtVectorElementExpr>(S)) {
    if (IndexedVarDeclRef IVarRef = VEE->getIndexedDefDecl()) {
      ExcludeDefinedVariable(IVarRef);
    }
    ExcludeDefinedVariables(VEE->getBase());

  } else if (WPhiFunction *PF = dyn_cast<WPhiFunction>(S)) {
    ExcludeDefinedVariable(PF->getIndexedLHSDecl());

  } else {
    for (WStmt::child_iterator I = S->child_begin(), E = S->child_end();
         I != E; ++I) {
      ExcludeDefinedVariables(*I);
    }
  }
}

void RegionInvariance::ExcludeDefinedVariable(IndexedVarDeclRef IVarRef) {
  if (IVarRef.isSingleVar()) {
    InvariantVars.erase(*IVarRef);
  } else {
    assert(IVarRef.isCompound());
    for (unsigned Index = 0, NumSubVars = IVarRef.getNumSubVars();
         Index != NumSubVars; ++Index) {
      InvariantVars.erase(IVarRef[Index]);
    }
  }
}

} // namespace snu

} // namespace clang
