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

#include "clang/SnuAnalysis/MemoryAccess.h"
#include "clang/AST/Type.h"
#include "clang/Basic/AddressSpaces.h"
#include "clang/Basic/LLVM.h"
#include "clang/SnuAST/WAST.h"
#include "clang/SnuAST/WCFG.h"
#include "clang/SnuAnalysis/Loop.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/SmallVector.h"

namespace clang {

namespace snu {

MemoryAccess::MemoryAccess(WExpr *expr, QualType type, MemoryAccessKind kind,
                           WCFGBlock *bb, WVarDecl *variable)
  : AccessExpr(expr), AccessType(type), AccessKind(kind), BasicBlock(bb),
    Variable(variable), Pointer(NULL), Index(NULL), Parent(NULL), Child(NULL) {
}

MemoryAccess::MemoryAccess(WExpr *expr, QualType type, MemoryAccessKind kind,
                           WCFGBlock *bb, WExpr *pointer)
  : AccessExpr(expr), AccessType(type), AccessKind(kind), BasicBlock(bb),
    Variable(NULL), Pointer(pointer), Index(NULL), Parent(NULL), Child(NULL) {
}

MemoryAccess::MemoryAccess(WExpr *expr, QualType type, MemoryAccessKind kind,
                           WCFGBlock *bb, WExpr *pointer, WExpr *index)
  : AccessExpr(expr), AccessType(type), AccessKind(kind), BasicBlock(bb),
    Variable(NULL), Pointer(pointer), Index(index), Parent(NULL), Child(NULL) {
}

MemoryAccess::MemoryAccess(WExpr *expr, QualType type, MemoryAccessKind kind,
                           WCFGBlock *bb, MemoryAccess *parent)
  : AccessExpr(expr), AccessType(type), AccessKind(kind), BasicBlock(bb),
    Variable(NULL), Pointer(NULL), Index(NULL), Parent(parent), Child(NULL) {
  assert(!Parent->hasChild());
  Parent->Child = this;
}

bool MemoryAccessTrace::AlwaysTrue(QualType PointeeTy) {
  return true;
}

bool MemoryAccessTrace::GlobalAddressSpaceOnly(QualType PointeeTy) {
  return PointeeTy.getAddressSpace() == LangAS::opencl_global ||
         PointeeTy.getAddressSpace() == LangAS::opencl_constant;
}

bool MemoryAccessTrace::ZeroAddressSpaceOnly(QualType PointeeTy) {
  return PointeeTy.getAddressSpace() == 0;
}

MemoryAccessTrace::MemoryAccessTrace(const WCFG *Program, FilterTy filter)
  : IsFullTrace(true), Filter(filter), CurrentBlock(NULL) {
  assert(Filter);
  for (WCFG::const_iterator B = Program->begin(), BEnd = Program->end();
       B != BEnd; ++B) {
    FindMemoryAccesses(*B);
  }
}

MemoryAccessTrace::MemoryAccessTrace(const WCFGBlock *Block, FilterTy filter)
  : IsFullTrace(false), Filter(filter), CurrentBlock(NULL) {
  assert(Filter);
  FindMemoryAccesses(const_cast<WCFGBlock*>(Block));
}

MemoryAccessTrace::MemoryAccessTrace(const Loop &L, FilterTy filter)
  : IsFullTrace(false), Filter(filter), CurrentBlock(NULL) {
  assert(Filter);
  for (Loop::body_iterator B = L.body_begin(), BEnd = L.body_end();
       B != BEnd; ++B) {
    FindMemoryAccesses(*B);
  }
}

MemoryAccessTrace::~MemoryAccessTrace() {
  for (SmallVectorImpl<MemoryAccess*>::iterator I = Trace.begin(),
                                                E = Trace.end();
       I != E; ++I) {
    delete (*I);
  }
}

void MemoryAccessTrace::FindMemoryAccesses(WCFGBlock *Block) {
  CurrentBlock = Block;
  for (WCFGBlock::const_iterator S = Block->begin(), SEnd = Block->end();
       S != SEnd; ++S) {
    FindMemoryAccesses(*S, true);
  }
  CurrentBlock = NULL;
}

void MemoryAccessTrace::FindMemoryAccesses(WStmt *S, bool Outermost) {
  if (!S)
    return;

  if (WDeclStmt *DS = dyn_cast<WDeclStmt>(S)) {
    assert(DS->isSingleDecl());
    if (DS->hasSingleInit()) {
      FindMemoryAccesses(DS->getSingleInit());
    }

  } else if (WUnaryOperator *UO = dyn_cast<WUnaryOperator>(S)) {
    if (UO->isIncrementDecrementOp()) {
      FindMemoryAccessesInLHS(UO->getSubExpr(), true);
    } else {
      FindMemoryAccesses(UO->getSubExpr());
    }

  } else if (WCallExpr *CE = dyn_cast<WCallExpr>(S)) {
    if (Outermost) {
      FindMemoryAccesses(CE->getCallee());
      for (unsigned Index = 0, NumArgs = CE->getNumArgs();
           Index != NumArgs; ++Index) {
        FindMemoryAccesses(CE->getArg(Index));
      }
      if (CE->isAtomic()) {
        assert(CE->getNumArgs() >= 1);
        if (IsValidPointerType(CE->getArg(0)->getType())) {
          AddMemoryAccess(CE, CE->getType(), MemoryAccess::AK_ATOMIC,
                          CE->getArg(0));
        }
      } else if (CE->isVectorLoad()) {
        assert(CE->getNumArgs() == 2);
        if (IsValidPointerType(CE->getArg(1)->getType())) {
          AddMemoryAccess(CE, CE->getType(), MemoryAccess::AK_READ,
                          CE->getArg(1));
        }
      } else if (CE->isVectorStore()) {
        assert(CE->getNumArgs() == 3);
        if (IsValidPointerType(CE->getArg(2)->getType())) {
          AddMemoryAccess(CE, CE->getArg(0)->getType(), MemoryAccess::AK_WRITE,
                          CE->getArg(2));
        }
      }
    }

  } else if (WImplicitCastExpr *ICE = dyn_cast<WImplicitCastExpr>(S)) {
    if (ICE->getCastKind() == CK_LValueToRValue) {
      FindMemoryAccessesInRHS(ICE->getSubExpr());
    } else {
      FindMemoryAccesses(ICE->getSubExpr());
    }

  } else if (WBinaryOperator *BO = dyn_cast<WBinaryOperator>(S)) {
    if (BO->isAssignmentOp()) {
      FindMemoryAccesses(BO->getRHS());
      FindMemoryAccessesInLHS(BO->getLHS(), BO->isCompoundAssignmentOp());
    } else if (BO->isLogicalOp() || BO->getOpcode() == BO_Comma) {
      // Do nothing
    } else {
      FindMemoryAccesses(BO->getLHS());
      FindMemoryAccesses(BO->getRHS());
    }

  } else if (isa<WConditionalOperator>(S)) {
    // Do nothing

  } else if (WAsTypeExpr *ASE = dyn_cast<WAsTypeExpr>(S)) {
    FindMemoryAccessesInRHS(ASE->getSrcExpr());

  } else {
    for (WStmt::child_iterator I = S->child_begin(), E = S->child_end();
         I != E; ++I) {
      FindMemoryAccesses(*I);
    }
  }
}

void MemoryAccessTrace::FindMemoryAccessesInLHS(WExpr *E, bool Compound) {
  E = E->IgnoreParenCasts();

  MemoryAccessKind AccessKind =
      (Compound ? MemoryAccess::AK_READWRITE : MemoryAccess::AK_WRITE);

  if (WDeclRefExpr *DRE = dyn_cast<WDeclRefExpr>(E)) {
    if (DRE->getIndexedDefDecl()) {
      // Do nothing
    } else if (WVarDecl *Variable = DRE->getVarDecl()) {
      if (IsValidVariableType(Variable->getType())) {
        AddMemoryAccess(DRE, DRE->getType(), AccessKind, Variable);
      }
    } else {
      // Do nothing
    }

  } else if (WUnaryOperator *UO = dyn_cast<WUnaryOperator>(E)) {
    FindMemoryAccesses(UO->getSubExpr());
    if (UO->getOpcode() == UO_Deref &&
        IsValidPointerType(UO->getSubExpr()->getType())) {
      AddMemoryAccess(UO, UO->getType(), AccessKind, UO->getSubExpr());
    }

  } else if (WArraySubscriptExpr *ASE = dyn_cast<WArraySubscriptExpr>(E)) {
    FindMemoryAccesses(ASE->getLHS());
    FindMemoryAccesses(ASE->getRHS());
    if (IsValidPointerType(ASE->getBase()->getType())) {
      AddMemoryAccess(ASE, ASE->getType(), AccessKind, ASE->getBase(),
                      ASE->getIdx());
    }

  } else if (WMemberExpr *ME = dyn_cast<WMemberExpr>(E)) {
    if (ME->isArrow()) {
      FindMemoryAccesses(ME->getBase());
      if (IsValidPointerType(ME->getBase()->getType())) {
        AddMemoryAccess(ME, ME->getType(), AccessKind, ME->getBase());
      }
    } else {
      if (ME->getIndexedDefDecl()) {
       // Do nothing
      } else {
        FindMemoryAccessesInLHS(ME->getBase(), Compound);
        InheritMemoryAccess(ME, ME->getType(), AccessKind, ME->getBase());
      }
    }

  } else if (WExtVectorElementExpr *VEE = dyn_cast<WExtVectorElementExpr>(E)) {
    if (VEE->getIndexedDefDecl()) {
      // Do nothing
    } else {
      FindMemoryAccessesInLHS(VEE->getBase(), Compound);
    }

  } else {
    FindMemoryAccesses(E);
  }
}

void MemoryAccessTrace::FindMemoryAccessesInRHS(WExpr *E, bool Innermost) {
  E = E->IgnoreParenCasts();

  MemoryAccessKind AccessKind = MemoryAccess::AK_READ;

  if (WDeclRefExpr *DRE = dyn_cast<WDeclRefExpr>(E)) {
    if (DRE->getIndexedUseDecl()) {
      // Do nothing
    } else if (WVarDecl *Variable = DRE->getVarDecl()) {
      if (IsValidVariableType(Variable->getType())) {
        AddMemoryAccess(DRE, DRE->getType(), AccessKind, Variable);
      }
    } else {
      // Do nothing
    }

  } else if (WUnaryOperator *UO = dyn_cast<WUnaryOperator>(E)) {
    if (!Innermost) {
      FindMemoryAccesses(UO->getSubExpr());
    }
    if (UO->getOpcode() == UO_Deref &&
        IsValidPointerType(UO->getSubExpr()->getType())) {
      AddMemoryAccess(UO, UO->getType(), AccessKind, UO->getSubExpr());
    }

  } else if (WArraySubscriptExpr *ASE = dyn_cast<WArraySubscriptExpr>(E)) {
    if (!Innermost) {
      FindMemoryAccesses(ASE->getLHS());
      FindMemoryAccesses(ASE->getRHS());
    }
    if (IsValidPointerType(ASE->getBase()->getType())) {
      AddMemoryAccess(ASE, ASE->getType(), AccessKind, ASE->getBase(),
                      ASE->getIdx());
    }

  } else if (WMemberExpr *ME = dyn_cast<WMemberExpr>(E)) {
    if (ME->isArrow()) {
      if (!Innermost) {
        FindMemoryAccesses(ME->getBase());
      }
      if (IsValidPointerType(ME->getBase()->getType())) {
        AddMemoryAccess(ME, ME->getType(), AccessKind, ME->getBase());
      }
    } else {
      if (ME->getIndexedUseDecl()) {
        // Do nothing
      } else {
        FindMemoryAccessesInRHS(ME->getBase(), Innermost);
        InheritMemoryAccess(ME, ME->getType(), AccessKind, ME->getBase());
      }
    }

  } else if (WConditionalOperator *CO = dyn_cast<WConditionalOperator>(E)) {
    // We need to traverse LHS and RHS in LValueToRValue(Cond ? LHS : RHS)
    // but their children should not be traversed
    FindMemoryAccessesInRHS(CO->getLHS(), true);
    FindMemoryAccessesInRHS(CO->getRHS(), true);

  } else if (WExtVectorElementExpr *VEE = dyn_cast<WExtVectorElementExpr>(E)) {
    if (VEE->getIndexedUseDecl()) {
      // Do nothing
    } else {
      FindMemoryAccessesInRHS(VEE->getBase(), Innermost);
    }

  } else {
    if (!Innermost) {
      FindMemoryAccesses(E);
    }
  }
}

void MemoryAccessTrace::AddMemoryAccess(WExpr *Access, QualType AccessType,
                                        MemoryAccessKind AccessKind,
                                        WVarDecl *Variable) {
  assert(CurrentBlock != NULL);
  MemoryAccess *Entry = new MemoryAccess(Access, AccessType, AccessKind,
                                         CurrentBlock, Variable);
  Trace.push_back(Entry);
  assert(!AccessMap.count(Access));
  AccessMap[Access] = Entry;
}

void MemoryAccessTrace::AddMemoryAccess(WExpr *Access, QualType AccessType,
                                        MemoryAccessKind AccessKind,
                                        WExpr *Pointer) {
  assert(CurrentBlock != NULL);
  MemoryAccess *Entry = new MemoryAccess(Access, AccessType, AccessKind,
                                         CurrentBlock, Pointer);
  Trace.push_back(Entry);
  assert(!AccessMap.count(Access));
  AccessMap[Access] = Entry;
}

void MemoryAccessTrace::AddMemoryAccess(WExpr *Access, QualType AccessType,
                                        MemoryAccessKind AccessKind,
                                        WExpr *Pointer, WExpr *Index) {
  assert(CurrentBlock != NULL);
  MemoryAccess *Entry = new MemoryAccess(Access, AccessType, AccessKind,
                                         CurrentBlock, Pointer, Index);
  Trace.push_back(Entry);
  assert(!AccessMap.count(Access));
  AccessMap[Access] = Entry;
}

void MemoryAccessTrace::InheritMemoryAccess(WExpr *Access, QualType AccessType,
                                            MemoryAccessKind AccessKind,
                                            WExpr *Parent) {
  Parent = Parent->IgnoreParenCasts();
  if (AccessMap.count(Parent)) {
    assert(CurrentBlock != NULL);
    MemoryAccess *Entry = new MemoryAccess(Access, AccessType, AccessKind,
                                           CurrentBlock, AccessMap[Parent]);
    Trace.push_back(Entry);
    assert(!AccessMap.count(Access));
    AccessMap[Access] = Entry;
  }
}

bool MemoryAccessRelation::containsAtFirst(MemoryAccess *First) const {
  for (PairSetTy::const_iterator I = Elements.begin(), E = Elements.end();
       I != E; ++I) {
    if (I->first == First) {
      return true;
    }
  }
  return false;
}

bool MemoryAccessRelation::containsAtSecond(MemoryAccess *Second) const {
  for (PairSetTy::const_iterator I = Elements.begin(), E = Elements.end();
       I != E; ++I) {
    if (I->second == Second) {
      return true;
    }
  }
  return false;
}

bool MemoryAccessRelation::ReadRead(MemoryAccess *First, MemoryAccess *Second) {
  return First->isRead() && Second->isRead();
}

bool MemoryAccessRelation::ReadWrite(MemoryAccess *First, MemoryAccess *Second) {
  return First->isRead() && Second->isWrite();
}

bool MemoryAccessRelation::WriteRead(MemoryAccess *First, MemoryAccess *Second) {
  return First->isWrite() && Second->isRead();
}

bool MemoryAccessRelation::WriteWrite(MemoryAccess *First, MemoryAccess *Second) {
  return First->isWrite() && Second->isWrite();
}

void MemoryAccessRelation::filter(FilterTy Filter) {
  PairSetTy NewElements;
  for (PairSetTy::iterator I = Elements.begin(), E = Elements.end();
       I != E; ++I) {
    if (Filter(I->first, I->second)) {
      NewElements.insert(*I);
    }
  }
  Elements = NewElements;
}

void MemoryAccessRelation::join(const MemoryAccessRelation &RHS) {
  PairSetTy NewElements;
  for (PairSetTy::iterator I = Elements.begin(), E = Elements.end();
       I != E; ++I) {
    if (RHS.Elements.count(*I)) {
      NewElements.insert(*I);
    }
  }
  Elements = NewElements;
}

MemoryAccessRelation *MemoryAccessRelation::CreateReachability(
    const MemoryAccessTrace &Trace, const WCFG *Program) {
  MemoryAccessRelation *R = new MemoryAccessRelation(Trace);
  for (MemoryAccessTrace::iterator F = Trace.begin(), FEnd = Trace.end();
       F != FEnd; ++F) {
    for (MemoryAccessTrace::iterator S = Trace.begin(), SEnd = Trace.end();
        S != SEnd; ++S) {
      MemoryAccess *First = *F;
      MemoryAccess *Second = *S;
      if (First == Second ||
          Program->isPotentiallyReachable(First->getAccessExpr(),
                                          Second->getAccessExpr())) {
        R->insert(First, Second);
      }
    }
  }
  return R;
}

} // namespace snu

} // namespace clang
