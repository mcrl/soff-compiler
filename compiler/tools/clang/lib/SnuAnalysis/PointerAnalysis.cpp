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

#include "clang/SnuAnalysis/PointerAnalysis.h"
#include "clang/AST/ASTContext.h"
#include "clang/AST/Expr.h"
#include "clang/AST/Type.h"
#include "clang/Basic/AddressSpaces.h"
#include "clang/Basic/LLVM.h"
#include "clang/SnuAST/WAST.h"
#include "clang/SnuAST/WCFG.h"
#include "clang/SnuAnalysis/MemoryAccess.h"
#include "clang/SnuSupport/DisjointSet.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/DenseSet.h"

namespace clang {

namespace snu {

namespace {

class PointerAnalysisImpl {
  WCFG *Program;
  const MemoryAccessTrace &Trace;

  // Elements of PointsToSets are either IndexedVarDecl* or WVarDecl*
  DisjointSet<const void*> PointsToSets;
  typedef DisjointSet<const void*>::SubsetTy SubsetTy;

  typedef OrderedDenseSet<WVarDecl*> VarSetTy;
  typedef llvm::DenseMap<MemoryAccess*, SubsetTy> AccessSubsetMapTy;
  VarSetTy ParamVars;
  VarSetTy MemVars;
  AccessSubsetMapTy MemoryAccessPointsTo;

public:
  PointerAnalysisImpl(WCFG *program, const MemoryAccessTrace &trace);
  void Analysis(AliasSet &AS);

private:
  void CalculatePointsToSets();
  void CalculateMemoryAccessPointsTo();
  void PruneMissingSourcePointsToSets();
  void ApplyToAliasSet(AliasSet &AS);

public:
  SubsetTy MemVarLocation(WVarDecl *Var);
  SubsetTy SSAVarPointsTo(IndexedVarDecl *Var);
  SubsetTy ExprPointsTo(WExpr *E);
  SubsetTy AssignedValuePointsTo(WStmt *S);

private:
  SubsetTy DeclRefExprPointsTo(WDeclRefExpr *Node);
  SubsetTy UnaryOperatorPointsTo(UnaryOperator::Opcode Op, WExpr *SubExpr);
  SubsetTy ArraySubscriptExprPointsTo(WArraySubscriptExpr *Node);
  SubsetTy MemberExprPointsTo(WMemberExpr *Node);
  SubsetTy BinaryOperatorPointsTo(WBinaryOperator *Node);
  SubsetTy ConditionalOperatorPointsTo(WConditionalOperator *Node);
};

PointerAnalysisImpl::PointerAnalysisImpl(WCFG *program,
                                         const MemoryAccessTrace &trace)
  : Program(program), Trace(trace) {
  assert(Program != NULL && Program->isSSA());
  assert(Trace.isFullTrace() && "pointer analysis only works on a full trace");
}

void PointerAnalysisImpl::Analysis(AliasSet &AS) {
  CalculatePointsToSets();
  CalculateMemoryAccessPointsTo();
  PruneMissingSourcePointsToSets();
  ApplyToAliasSet(AS);
}

void PointerAnalysisImpl::CalculatePointsToSets() {
  PointsToSets.clear();
  PointsToSets.create_set(NULL); // a set for unknown accesses
  ParamVars.clear();
  MemVars.clear();
  for (WCFG::ssa_var_iterator V = Program->ssa_var_begin(),
                              VEnd = Program->ssa_var_end();
       V != VEnd; ++V) {
    IndexedVarDecl *Var = *V;
    if (WStmt *DefinedStmt = Var->getDefinedStmt()) {
      PointsToSets.union_set(SSAVarPointsTo(Var),
                             AssignedValuePointsTo(DefinedStmt));
    }
  }
}

void PointerAnalysisImpl::CalculateMemoryAccessPointsTo() {
  MemoryAccessPointsTo.clear();
  for (MemoryAccessTrace::iterator I = Trace.begin(), E = Trace.end();
       I != E; ++I) {
    MemoryAccess *Entry = *I;
    if (Entry->isAccessFromVariable()) {
      MemoryAccessPointsTo[Entry] = MemVarLocation(Entry->getVariable());
    } else if (Entry->isAccessFromPointer()) {
      SubsetTy Subset = ExprPointsTo(Entry->getPointer());
      if (PointsToSets.is_same_set(Subset, NULL)) {
        PointsToSets.union_all();
      }
      MemoryAccessPointsTo[Entry] = Subset;
    } else {
      assert(Entry->isInheritedAccess());
      assert(MemoryAccessPointsTo.count(Entry->getParent()));
      MemoryAccessPointsTo[Entry] = MemoryAccessPointsTo[Entry->getParent()];
    }
  }
}

void PointerAnalysisImpl::PruneMissingSourcePointsToSets() {
  llvm::DenseSet<SubsetTy> ValidSubset;
  ValidSubset.insert(PointsToSets[NULL]);
  for (VarSetTy::const_iterator P = ParamVars.begin(), PEnd = ParamVars.end();
       P != PEnd; ++P) {
    ValidSubset.insert(PointsToSets[*P]);
  }
  for (VarSetTy::const_iterator V = MemVars.begin(), VEnd = MemVars.end();
       V != VEnd; ++V) {
    ValidSubset.insert(PointsToSets[*V]);
  }

  for (WCFG::ssa_var_iterator V = Program->ssa_var_begin(),
                              VEnd = Program->ssa_var_end();
       V != VEnd; ++V) {
    if (PointsToSets.exist(*V) && !ValidSubset.count(PointsToSets[*V])) {
      PointsToSets.union_set(PointsToSets[*V], PointsToSets[NULL]);
    }
  }
}

void PointerAnalysisImpl::ApplyToAliasSet(AliasSet &AS) {
  unsigned NumAliases = 0;
  llvm::DenseMap<SubsetTy, unsigned> SubsetID;

  for (MemoryAccessTrace::iterator A = Trace.begin(), AEnd = Trace.end();
       A != AEnd; ++A) {
    MemoryAccess *Entry = *A;
    SubsetTy Subset = PointsToSets[MemoryAccessPointsTo[Entry]];
    if (PointsToSets.is_same_set(Subset, NULL)) {
      if (!SubsetID.count(Subset)) {
        SubsetID[Subset] = 0;
      }
    } else {
      if (!SubsetID.count(Subset)) {
        SubsetID[Subset] = ++NumAliases;
      }
    }
    AS.setAlias(Entry->getAccessExpr(), SubsetID[Subset]);
  }

  for (VarSetTy::const_iterator P = ParamVars.begin(), PEnd = ParamVars.end();
       P != PEnd; ++P) {
    SubsetTy Subset = PointsToSets[*P];
    if (SubsetID.count(Subset)) {
      // register only if the parameter has been used to access the memory
      AS.setAlias(*P, SubsetID[Subset]);
    }
  }
  for (VarSetTy::const_iterator V = MemVars.begin(), VEnd = MemVars.end();
       V != VEnd; ++V) {
    SubsetTy Subset = PointsToSets[*V];
    if (SubsetID.count(Subset)) {
      AS.setAlias(*V, SubsetID[Subset]);
    }
  }
}

PointerAnalysisImpl::SubsetTy PointerAnalysisImpl::MemVarLocation(
    WVarDecl *Var) {
  if (!MemVars.count(Var)) {
    PointsToSets.create_set(Var);
    MemVars.insert(Var);
  }
  return PointsToSets[Var];
}

PointerAnalysisImpl::SubsetTy PointerAnalysisImpl::SSAVarPointsTo(
    IndexedVarDecl *Var) {
  if (!PointsToSets.exist(Var)) {
    PointsToSets.create_set(Var);
    if (Var->isParameter()) {
      WParmVarDecl *P = static_cast<WParmVarDecl*>(Var->getDecl());
      PointsToSets.create_set(P);
      PointsToSets.union_set(Var, P);
      ParamVars.insert(P);
    }
  }
  return PointsToSets[Var];
}

PointerAnalysisImpl::SubsetTy PointerAnalysisImpl::ExprPointsTo(WExpr *E) {
  if (WDeclRefExpr *DRE = dyn_cast<WDeclRefExpr>(E)) {
    return DeclRefExprPointsTo(DRE);
  } else if (WParenExpr *PE = dyn_cast<WParenExpr>(E)) {
    return ExprPointsTo(PE->getSubExpr());
  } else if (WUnaryOperator *UO = dyn_cast<WUnaryOperator>(E)) {
    return UnaryOperatorPointsTo(UO->getOpcode(), UO->getSubExpr());
  } else if (WArraySubscriptExpr *ASE = dyn_cast<WArraySubscriptExpr>(E)) {
    return ArraySubscriptExprPointsTo(ASE);
  } else if (WMemberExpr *ME = dyn_cast<WMemberExpr>(E)) {
    return MemberExprPointsTo(ME);
  } else if (WCastExpr *CE = dyn_cast<WCastExpr>(E)) {
    return ExprPointsTo(CE->getSubExpr());
  } else if (WBinaryOperator *BO = dyn_cast<WBinaryOperator>(E)) {
    return BinaryOperatorPointsTo(BO);
  } else if (WConditionalOperator *CO = dyn_cast<WConditionalOperator>(E)) {
    return ConditionalOperatorPointsTo(CO);
  } else {
    return PointsToSets[NULL];
  }
}

PointerAnalysisImpl::SubsetTy PointerAnalysisImpl::AssignedValuePointsTo(
    WStmt *S) {
  if (WDeclStmt *DS = dyn_cast<WDeclStmt>(S)) {
    assert(DS->hasSingleInit());
    return ExprPointsTo(DS->getSingleInit());

  } else if (WUnaryOperator *UO = dyn_cast<WUnaryOperator>(S)) {
    assert(UO->isIncrementDecrementOp());
    return ExprPointsTo(UO->getSubExpr());

  } else if (WCompoundAssignOperator *CAO =
               dyn_cast<WCompoundAssignOperator>(S)) {
    return ExprPointsTo(CAO);

  } else if (WBinaryOperator *BO = dyn_cast<WBinaryOperator>(S)) {
    assert(BO->isAssignmentOp());
    return ExprPointsTo(BO->getRHS());

  } else if (WPhiFunction *PF = dyn_cast<WPhiFunction>(S)) {
    for (unsigned Index = 0, NumArgs = PF->getNumArgs();
         Index != NumArgs; ++Index) {
      SubsetTy ArgSubset = SSAVarPointsTo(PF->getIndexedArgDecl(Index));
      if (PointsToSets.is_same_set(ArgSubset, NULL)) {
        return PointsToSets[NULL];
      }
    }
    SubsetTy Subset = SSAVarPointsTo(PF->getIndexedArgDecl(0));
    for (unsigned Index = 1, NumArgs = PF->getNumArgs();
         Index != NumArgs; ++Index) {
      SubsetTy ArgSubset = SSAVarPointsTo(PF->getIndexedArgDecl(Index));
      Subset = PointsToSets.union_set(Subset, ArgSubset);
    }
    return Subset;

  } else {
    llvm_unreachable("invalid assignment statement");
  }
}

PointerAnalysisImpl::SubsetTy PointerAnalysisImpl::DeclRefExprPointsTo(
    WDeclRefExpr *Node) {
  if (IndexedVarDeclRef SSAVarRef = Node->getIndexedUseDecl()) {
    if (SSAVarRef.isSingleVar()) {
      return SSAVarPointsTo(*SSAVarRef);
    }
  } else if (WVarDecl *MemVar = Node->getVarDecl()) {
    if (MemVar->getType()->isArrayType()) {
      // An array name points to its location
      return MemVarLocation(MemVar);
    }
  }
  return PointsToSets[NULL];
}

PointerAnalysisImpl::SubsetTy PointerAnalysisImpl::UnaryOperatorPointsTo(
    UnaryOperator::Opcode Op, WExpr *SubExpr) {
  switch (Op) {
    case UO_PostInc:
    case UO_PostDec:
    case UO_PreInc:
    case UO_PreDec:
    case UO_Plus: {
      return ExprPointsTo(SubExpr);
    }
    case UO_AddrOf: {
      SubExpr = SubExpr->IgnoreParenCasts();
      // &A == location(A)
      if (WDeclRefExpr *DRE = dyn_cast<WDeclRefExpr>(SubExpr)) {
        assert(!DRE->getIndexedUseDecl());
        if (WVarDecl *MemVar = DRE->getVarDecl()) {
          return MemVarLocation(MemVar);
        }
      // &*A == A
      } else if (WUnaryOperator *UO = dyn_cast<WUnaryOperator>(SubExpr)) {
        if (UO->getOpcode() == UO_Deref) {
          return ExprPointsTo(UO->getSubExpr());
        }
      // &A[B] == A + B
      } else if (WArraySubscriptExpr *ASE =
                   dyn_cast<WArraySubscriptExpr>(SubExpr)) {
        return ExprPointsTo(ASE->getBase());
      // &(A.p) == &A + offset(p)
      // &(A->p) == A + offset(p)
      } else if (WMemberExpr *ME = dyn_cast<WMemberExpr>(SubExpr)) {
        if (ME->isArrow()) {
          return ExprPointsTo(ME->getBase());
        } else {
          return UnaryOperatorPointsTo(UO_AddrOf, ME->getBase());
        }
      } else {
        return PointsToSets[NULL];
      }
    }
    default:
      return PointsToSets[NULL];
  }
}

PointerAnalysisImpl::SubsetTy PointerAnalysisImpl::ArraySubscriptExprPointsTo(
    WArraySubscriptExpr *Node) {
  // A[B] == A + B
  if (Node->getType()->isArrayType()) {
    return ExprPointsTo(Node->getBase());
  // A[B] == *(A + B)
  } else {
    return PointsToSets[NULL];
  }
}

PointerAnalysisImpl::SubsetTy PointerAnalysisImpl::MemberExprPointsTo(
    WMemberExpr *Node) {
  if (Node->isArrow()) {
    // A->p == A + offset(p)
    if (Node->getType()->isArrayType()) {
      return ExprPointsTo(Node->getBase());
    // A->p == *&(A->p) == *(A + offset(p))
    } else {
      return PointsToSets[NULL];
    }
  } else {
    if (IndexedVarDeclRef SSAVarRef = Node->getIndexedUseDecl()) {
      if (SSAVarRef.isSingleVar()) {
        return SSAVarPointsTo(*SSAVarRef);
      }
    }
    if (Node->getType()->isArrayType()) {
      // A.p = &A + offset(p)
      return UnaryOperatorPointsTo(UO_AddrOf, Node->getBase());
    } else {
      return PointsToSets[NULL];
    }
  }
}

PointerAnalysisImpl::SubsetTy PointerAnalysisImpl::BinaryOperatorPointsTo(
    WBinaryOperator *Node) {
  switch (Node->getOpcode()) {
    case BO_Add:
    case BO_AddAssign: {
      if (Node->getLHS()->getType()->isPointerType()) {
        return ExprPointsTo(Node->getLHS());
      } else if (Node->getRHS()->getType()->isPointerType()) {
        return ExprPointsTo(Node->getRHS());
      } else {
        SubsetTy LHSSubset = ExprPointsTo(Node->getLHS());
        SubsetTy RHSSubset = ExprPointsTo(Node->getRHS());
        if (LHSSubset != PointsToSets[NULL] && RHSSubset != PointsToSets[NULL]) {
          return PointsToSets.union_set(LHSSubset, RHSSubset);
        } else if (LHSSubset != PointsToSets[NULL]) {
          return LHSSubset;
        } else if (RHSSubset != PointsToSets[NULL]) {
          return RHSSubset;
        } else {
          return PointsToSets[NULL];
        }
      }
    }
    case BO_Sub:
    case BO_SubAssign: {
      return ExprPointsTo(Node->getLHS());
    }
    case BO_Assign:
    case BO_Comma: {
      return ExprPointsTo(Node->getRHS());
    }
    default:
      return PointsToSets[NULL];
  }
}

PointerAnalysisImpl::SubsetTy PointerAnalysisImpl::ConditionalOperatorPointsTo(
    WConditionalOperator *Node) {
  SubsetTy LHSSubset = ExprPointsTo(Node->getLHS());
  SubsetTy RHSSubset = ExprPointsTo(Node->getRHS());
  return PointsToSets.union_set(LHSSubset, RHSSubset);
}

} // anonymous namespace

AliasSet::AliasSet(WCFG *program)
  : Program(program), MaxAlias(0) {
  MemoryAccessTrace Trace(Program);
  PointerAnalysisImpl Impl(Program, Trace);
  Impl.Analysis(*this);
}

AliasSet::AliasSet(WCFG *program, MemoryAccessTrace::FilterTy Filter)
  : Program(program), MaxAlias(0) {
  MemoryAccessTrace Trace(Program, Filter);
  PointerAnalysisImpl Impl(Program, Trace);
  Impl.Analysis(*this);
}

bool AliasSet::isUsed(const WVarDecl *V) const {
  return VarAlias.count(V);
}

unsigned AliasSet::getAlias(const WVarDecl *V) const {
  assert(VarAlias.count(V));
  return VarAlias.lookup(V);
}

bool AliasSet::isMemoryAccess(const WExpr *E) const {
  E = E->IgnoreParenCasts();
  if (ExprAlias.count(E)) {
    return true;
  } else if (const WDeclRefExpr *DRE = dyn_cast<WDeclRefExpr>(E)) {
    return VarAlias.count(DRE->getVarDecl());
  } else if (const WMemberExpr *ME = dyn_cast<WMemberExpr>(E)) {
    if (!ME->isArrow()) {
      return isMemoryAccess(ME->getBase());
    }
  } else if (const WExtVectorElementExpr *VEE =
      dyn_cast<WExtVectorElementExpr>(E)) {
    return isMemoryAccess(VEE->getBase());
  }
  return false;
}

unsigned AliasSet::getAlias(const WExpr *E) const {
  E = E->IgnoreParenCasts();
  if (ExprAlias.count(E)) {
    return ExprAlias.lookup(E);
  } else if (const WMemberExpr *ME = dyn_cast<WMemberExpr>(E)) {
    if (!ME->isArrow()) {
      return getAlias(ME->getBase());
    }
  } else if (const WExtVectorElementExpr *VEE =
      dyn_cast<WExtVectorElementExpr>(E)) {
    return getAlias(VEE->getBase());
  }
  llvm_unreachable("not a memory access");
}

bool AliasSet::isPointsToParameter(unsigned Alias) const {
  return PointsToParameter.count(Alias);
}

bool AliasSet::isPointsToParameter(const WVarDecl *V) const {
  assert(VarAlias.count(V));
  return PointsToParameter.count(VarAlias.lookup(V));
}

bool AliasSet::isPointsToParameter(const WExpr *E) const {
  assert(ExprAlias.count(E));
  return PointsToParameter.count(ExprAlias.lookup(E));
}

void AliasSet::setAlias(const WVarDecl *V, unsigned A) {
  VarAlias[V] = A;
  if (V->isParameter() && A != 0) {
    PointsToParameter.insert(A);
  }
  if (MaxAlias < A) {
    MaxAlias = A;
  }
}

void AliasSet::setAlias(const WExpr *E, unsigned A) {
  ExprAlias[E] = A;
  if (MaxAlias < A) {
    MaxAlias = A;
  }
}

MemoryAccessRelation *AliasSet::CreateRelation(
    const MemoryAccessTrace &Trace) const {
  MemoryAccessRelation *R = new MemoryAccessRelation(Trace);
  for (MemoryAccessTrace::iterator F = Trace.begin(), FEnd = Trace.end();
       F != FEnd; ++F) {
    MemoryAccess *First = *F;
    if (!isMemoryAccess(First->getAccessExpr())) continue;
    unsigned FirstAlias = getAlias(First->getAccessExpr());
    R->insert(First, First);
    for (MemoryAccessTrace::iterator S = F + 1, SEnd = Trace.end();
        S != SEnd; ++S) {
      MemoryAccess *Second = *S;
      if (!isMemoryAccess(Second->getAccessExpr())) continue;
      unsigned SecondAlias = getAlias(Second->getAccessExpr());
      if (FirstAlias == SecondAlias) {
        R->insert(First, Second);
        R->insert(Second, First);
      }
    }
  }
  return R;
}

void AliasSet::print(raw_ostream &OS, const ASTContext &ASTCtx) const {
  for (unsigned Alias = 0; Alias <= MaxAlias; Alias++) {
    OS << "Alias " << Alias;
    if (Alias == 0) OS << " (unknown accesses)";
    OS << ":\n";
    for (VarAliasMapTy::const_iterator V = VarAlias.begin(),
                                       VEnd = VarAlias.end();
         V != VEnd; ++V) {
      if (V->second == Alias) {
        OS << "  Variable \"" << V->first->getName() << "\"\n";
      }
    }
    for (ExprAliasMapTy::const_iterator E = ExprAlias.begin(),
                                        EEnd = ExprAlias.end();
         E != EEnd; ++E) {
      if (E->second == Alias) {
        OS << "  Expression \"";
        E->first->print(OS, ASTCtx.getPrintingPolicy());
        OS << "\"\n";
      }
    }
  }
}

} // namespace snu

} // namespace clang
