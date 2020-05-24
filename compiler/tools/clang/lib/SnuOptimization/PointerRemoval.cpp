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

#include "clang/SnuOptimization/PointerRemoval.h"
#include "clang/AST/ASTContext.h"
#include "clang/AST/Type.h"
#include "clang/Basic/LLVM.h"
#include "clang/SnuAST/WAST.h"
#include "clang/SnuAST/WCFG.h"
#include "clang/SnuAnalysis/MemoryAccess.h"
#include "clang/SnuAnalysis/PointerAnalysis.h"
#include "llvm/ADT/DenseSet.h"
#include "llvm/ADT/DenseMap.h"

namespace clang {

namespace snu {

namespace {

unsigned Log2(uint64_t x) {
  unsigned logx = 0;
  while (x > 1) {
    x >>= 1;
    logx++;
  }
  return logx;
}

} // anonymous namespace

PointerRemoval::PointerRemoval(ASTContext &ctx, WCFG *program)
  : Ctx(ctx), Program(program),
    Aliases(program, MemoryAccessTrace::ZeroAddressSpaceOnly), Verbose(false) {
}

void PointerRemoval::Run() {
  // # of variables in each alias
  llvm::DenseMap<unsigned, unsigned> VarCount;
  for (AliasSet::var_iterator V = Aliases.var_begin(), VEnd = Aliases.var_end();
       V != VEnd; ++V) {
    if (!VarCount.count(V->second)) {
      VarCount[V->second] = 0;
    }
    VarCount[V->second]++;
  }

  for (AliasSet::var_iterator V = Aliases.var_begin(), VEnd = Aliases.var_end();
       V != VEnd; ++V) {
    assert(VarCount.count(V->second));
    if (V->second != 0 && VarCount[V->second] == 1) {
      WVarDecl *Var = const_cast<WVarDecl*>(V->first);
      if (Var->getType()->isScalarType() || Var->getType()->isVectorType() ||
          Var->getType()->isStructureType()) {
        if (RemoveScalarPointers(Var)) {
          Program->IncrementSSAVariable(Var);
        }
      } else if (Var->getType()->isArrayType()) {
        if (RemoveArrayPointers(Var)) {
          Program->IncrementSSAVariable(Var);
        }
      }
    }
  }
}

bool PointerRemoval::IsMemVarDeclRef(WVarDecl *MemVar, WExpr *E) {
  if (WDeclRefExpr *DRE = dyn_cast<WDeclRefExpr>(E)) {
    if (DRE->getVarDecl() == MemVar) {
      return true;
    }
  }
  return false;
}

WExpr *PointerRemoval::CreateZeroBasedAddress(WExpr *Index, QualType Ty) {
  assert(Ty->isPointerType());
  QualType SizeTy = Ctx.getSizeType();
  WExpr *Address;
  if (Index == NULL) {
    Address = new (*Program) WIntegerLiteral(Ctx, 0, SizeTy);
  } else {
    if (Index->getType() != SizeTy) {
      Index = new (*Program) WCStyleCastExpr(Index, CK_IntegralCast, SizeTy);
    }
    uint64_t Step = Ctx.getTypeSize(Ty->getPointeeType()) / Ctx.getCharWidth();
    if (Step == 1) {
      Address = Index;
    } else if ((Step & (Step - 1)) == 0) {
      unsigned Log2Step = Log2(Step);
      Address = new (*Program) WBinaryOperator(
          Index, new (*Program) WIntegerLiteral(Ctx, Log2Step, SizeTy), BO_Shl,
          SizeTy);
    } else {
      Address = new (*Program) WBinaryOperator(
          Index, new (*Program) WIntegerLiteral(Ctx, Step, SizeTy), BO_Mul,
          SizeTy);
    }
  }
  return new (*Program) WCStyleCastExpr(Address, CK_IntegralToPointer, Ty);
}

WExpr *PointerRemoval::CreateZeroBasedAccess(WVarDecl *MemVar, WExpr *Address) {
  QualType SizeTy = Ctx.getSizeType();
  if (Address == NULL) {
    return new (*Program) WDeclRefExpr(MemVar);
  } else {
    assert(Address->getType()->isPointerType());
    assert(MemVar->getType()->isArrayType());
    QualType ElementTy = MemVar->getType()->getAsArrayTypeUnsafe()->
                         getElementType();
    // MemVar[Address / sizeof(ElementTy)]
    Address = new (*Program) WCStyleCastExpr(Address, CK_PointerToIntegral,
                                             SizeTy);
    uint64_t Step = Ctx.getTypeSize(ElementTy) / Ctx.getCharWidth();
    WExpr *Index;
    if (Step == 1) {
      Index = Address;
    } else if ((Step & (Step - 1)) == 0) {
      unsigned Log2Step = Log2(Step);
      Index = new (*Program) WBinaryOperator(
          Address, new (*Program) WIntegerLiteral(Ctx, Log2Step, SizeTy), BO_Shr,
          SizeTy);
    } else {
      Index = new (*Program) WBinaryOperator(
          Address, new (*Program) WIntegerLiteral(Ctx, Step, SizeTy), BO_Mul,
          SizeTy);
    }
    return new (*Program) WArraySubscriptExpr(
        new (*Program) WDeclRefExpr(MemVar), Index, ElementTy);
  }
}

bool PointerRemoval::ResolvePointerExpr(WVarDecl *MemVar, WExpr *E,
                                        bool AllowArithmetic) {
  if (!E) {
    return false;
  }
  E = E->IgnoreParens();
  if (!E->getType()->isPointerType()) {
    return false;
  }

  if (WDeclRefExpr *DRE = dyn_cast<WDeclRefExpr>(E)) {
    if (IndexedVarDeclRef SSAVars = DRE->getIndexedUseDecl()) {
      if (SSAVars.isSingleVar() && PointerVariables.count(*SSAVars)) {
        // ptr => ptr
        return true;
      }
    }
    return false;
  } else if (WUnaryOperator *UO = dyn_cast<WUnaryOperator>(E)) {
    if (UO->isIncrementDecrementOp()) {
      if (AllowArithmetic) {
        // ptr++ => ptr++
        assert(UO->getSubExpr()->getType()->isPointerType());
        return ResolvePointerExpr(MemVar, UO->getSubExpr(), AllowArithmetic);
      }
    } else if (UO->getOpcode() == UO_AddrOf) {
      WExpr *SubExpr = UO->getSubExpr()->IgnoreParenImpCasts();
      if (IsMemVarDeclRef(MemVar, SubExpr)) {
        // &MemVar => (Ty*)0
        Replacements[E] = CreateZeroBasedAddress(NULL, E->getType());
        return true;
      } else if (WUnaryOperator *SubUO = dyn_cast<WUnaryOperator>(SubExpr)) {
        if (SubUO->getOpcode() == UO_Deref) {
          if (ResolvePointerExpr(MemVar, SubUO->getSubExpr(),
                                 AllowArithmetic)) {
            // &*ptr => ptr
            assert(SubUO->getSubExpr()->getType() == E->getType());
            Replacements[E] = SubUO->getSubExpr();
            return true;
          }
        }
      } else if (WArraySubscriptExpr *SubASE =
                     dyn_cast<WArraySubscriptExpr>(SubExpr)) {
        if (AllowArithmetic) {
          if (SubASE->getBase()->getType()->isPointerType()) {
            if (ResolvePointerExpr(MemVar, SubASE->getBase(),
                                   AllowArithmetic)) {
              // &ptr[Index] => ptr + Index
              assert(SubASE->getBase()->getType() == E->getType());
              Replacements[E] = new (*Program) WBinaryOperator(
                  SubASE->getBase(), SubASE->getIdx(), BO_Add, E->getType());
              return true;
            }
          } else if (SubASE->getBase()->getType()->isArrayType()) {
            if (IsMemVarDeclRef(MemVar, SubASE->getBase())) {
              // &MemVar[Index] => (Ty*)(Index * sizeof(Ty))
              Replacements[E] = CreateZeroBasedAddress(SubASE->getIdx(),
                                                       E->getType());
              return true;
            }
          }
        }
      }
    }
  } else if (WCastExpr *CE = dyn_cast<WCastExpr>(E)) {
    if (CE->getCastKind() == CK_LValueToRValue ||
        CE->getCastKind() == CK_NoOp) {
      return ResolvePointerExpr(MemVar, CE->getSubExpr(), AllowArithmetic);
    } else if (CE->getCastKind() == CK_ArrayToPointerDecay) {
      if (IsMemVarDeclRef(MemVar, CE->getSubExpr())) {
        // MemVar => (Ty*)0
        Replacements[E] = CreateZeroBasedAddress(NULL, E->getType());
        return true;
      }
    }
  } else if (WBinaryOperator *BO = dyn_cast<WBinaryOperator>(E)) {
    if (BO->getOpcode() == BO_Add) {
      if (AllowArithmetic) {
        // ptr + RHS => ptr + RHS
        if (BO->getLHS()->getType()->isPointerType()) {
          return ResolvePointerExpr(MemVar, BO->getLHS(), AllowArithmetic);
        } else {
          assert(BO->getRHS()->getType()->isPointerType());
          return ResolvePointerExpr(MemVar, BO->getRHS(), AllowArithmetic);
        }
      }
    }
  }
  return false;
}

bool PointerRemoval::ResolveAccess(WVarDecl *MemVar, WExpr *E,
                                   bool AllowArithmetic) {
  if (IsMemVarDeclRef(MemVar, E)) {
    // MemVar => MemVar
    return true;
  } else if (WUnaryOperator *UO = dyn_cast<WUnaryOperator>(E)) {
    if (UO->getOpcode() == UO_Deref) {
      if (ResolvePointerExpr(MemVar, UO->getSubExpr(), AllowArithmetic)) {
        // *ptr => MemVar[ptr / sizeof(Element)] ... if AllowArithmetic
        //      => MemVar                        ... otherwise
        if (AllowArithmetic) {
          Replacements[E] = CreateZeroBasedAccess(MemVar, UO->getSubExpr());
        } else {
          Replacements[E] = new (*Program) WDeclRefExpr(MemVar);
        }
        return true;
      }
    }
  } else if (WArraySubscriptExpr *ASE = dyn_cast<WArraySubscriptExpr>(E)) {
    if (AllowArithmetic) {
      if (ASE->getBase()->getType()->isPointerType()) {
        if (ResolvePointerExpr(MemVar, ASE->getBase(), AllowArithmetic)) {
          // ptr[Index] => MemVar[(ptr + Index) / sizeof(Element)]
          WExpr *NewAddress = new (*Program) WBinaryOperator(
              ASE->getBase(), ASE->getIdx(), BO_Add, ASE->getBase()->getType());
          Replacements[E] = CreateZeroBasedAccess(MemVar, NewAddress);
          return true;
        }
      } else {
        assert(ASE->getBase()->getType()->isArrayType());
        if (IsMemVarDeclRef(MemVar, ASE->getBase())) {
          // MemVar[Index] => MemVar[Index]
          return true;
        }
      }
    }
  } else if (WMemberExpr *ME = dyn_cast<WMemberExpr>(E)) {
    if (ME->isArrow()) {
      if (ResolvePointerExpr(MemVar, ME->getBase(), AllowArithmetic)) {
        // ptr->F => MemVar[ptr / sizeof(Element)].F ... if AllowArithmetic
        //        => MemVar_F                        ... otherwise
        if (AllowArithmetic) {
          Replacements[E] = new (*Program) WMemberExpr(
              CreateZeroBasedAccess(MemVar, ME->getBase()), ME->getMemberDecl(),
              false, ME->getType());
        } else {
          Replacements[E] = new (*Program) WMemberExpr(
              new (*Program) WDeclRefExpr(MemVar), ME->getMemberDecl(), false,
              ME->getType(), MemVar->getSubVarOfStructure(ME->getMemberDecl()));
        }
        return true;
      }
    } else {
      if (ResolveAccess(MemVar, ME->getBase(), AllowArithmetic)) {
        // (*ptr).F => (MemVar[...]).F ... if AllowArithmetic
        //          => MemVar_F        ... otherwise
        if (AllowArithmetic) {
          // Do nothing
        } else {
          Replacements[E] = new (*Program) WMemberExpr(
              new (*Program) WDeclRefExpr(MemVar), ME->getMemberDecl(), false,
              ME->getType(), MemVar->getSubVarOfStructure(ME->getMemberDecl()));
        }
        return true;
      }
    }
  } else if (WExtVectorElementExpr *VEE = dyn_cast<WExtVectorElementExpr>(E)) {
    if (ResolveAccess(MemVar, VEE->getBase(), AllowArithmetic)) {
      // (*ptr).x => (MemVar[...]).x ... if AllowArithmetic
      //          => MemVar_x        ... otherwise
      if (AllowArithmetic) {
        // Do nothing
      } else {
        SmallVector<unsigned, 16> Elements;
        VEE->getEncodedElementAccess(Elements);
        WVarDecl *Var;
        if (Elements.size() == 1) {
          Var = MemVar->getSubVar(Elements[0]);
        } else {
          Var = new (*Program) WTemporaryVectorVarDecl(MemVar, Elements,
                                                       VEE->getType(), Ctx);
        }
        Replacements[E] = new (*Program) WExtVectorElementExpr(
            VEE->getBase(), VEE->getAccessor(), VEE->getType(), Var);
      }
      return true;
    }
  }
  return false;
}

void PointerRemoval::DiscoverPointerVariables(WVarDecl *MemVar,
                                              bool AllowArithmetic) {
  PointerVariables.clear();
  bool Updated;
  do {
    Updated = false;
    for (WCFG::ssa_var_iterator V = Program->ssa_var_begin(),
                                VEnd = Program->ssa_var_end();
         V != VEnd; ++V) {
      if (!(*V)->getType()->isPointerType() || PointerVariables.count(*V)) {
        continue;
      }
      bool IsPointerVariable = false;
      if (WStmt *DefinedStmt = (*V)->getDefinedStmt()) {
        if (WDeclStmt *DS = dyn_cast<WDeclStmt>(DefinedStmt)) {
          if (ResolvePointerExpr(MemVar, DS->getSingleInit(), AllowArithmetic)) {
            IsPointerVariable = true;
          }
        } else if (WUnaryOperator *UO = dyn_cast<WUnaryOperator>(DefinedStmt)) {
          if (ResolvePointerExpr(MemVar, UO->getSubExpr(), AllowArithmetic)) {
            IsPointerVariable = true;
          }
        } else if (WBinaryOperator *BO = dyn_cast<WBinaryOperator>(DefinedStmt)) {
          if (ResolvePointerExpr(MemVar, BO->getRHS(), AllowArithmetic)) {
            IsPointerVariable = true;
          }
        } else if (WPhiFunction *PF = dyn_cast<WPhiFunction>(DefinedStmt)) {
          IsPointerVariable = true;
          for (unsigned Index = 0, NumArgs = PF->getNumArgs();
               Index != NumArgs; ++Index) {
            IndexedVarDecl *Arg = PF->getIndexedArgDecl(Index);
            if (!PointerVariables.count(Arg)) {
              IsPointerVariable = false;
            }
          }
        }
      }
      if (IsPointerVariable) {
        PointerVariables.insert(*V);
        Updated = true;
      }
    }
  } while (Updated);
}

void PointerRemoval::ApplyReplacements() {
  for (llvm::DenseMap<WExpr*, WExpr*>::iterator R = Replacements.begin(),
                                                REnd = Replacements.end();
       R != REnd; ++R) {
    if (Verbose) {
      llvm::outs() << "[PointerRemoval] replacing statements\n";
      llvm::outs() << "  From: ";
      R->first->print(llvm::outs(), Ctx.getPrintingPolicy());
      llvm::outs() << "\n";
      llvm::outs() << "  To: ";
      R->second->print(llvm::outs(), Ctx.getPrintingPolicy());
      llvm::outs() << "\n";
    }
    Program->replaceStmt(R->first, R->second);
    for (llvm::DenseMap<WExpr*, WExpr*>::iterator N = R; N != REnd; ++N) {
      if (N != R) {
        N->second->replace(R->first, R->second);
      }
    }
  }
}

bool PointerRemoval::RemoveScalarPointers(WVarDecl *MemVar) {
  assert(MemVar->getType()->isScalarType() ||
         MemVar->getType()->isVectorType() ||
         MemVar->getType()->isStructureType());
  Replacements.clear();
  DiscoverPointerVariables(MemVar, false);
  unsigned Alias = Aliases.getAlias(MemVar);
  for (AliasSet::expr_iterator E = Aliases.expr_begin(),
                               EEnd = Aliases.expr_end();
       E != EEnd; ++E) {
    if (E->second == Alias) {
      if (!ResolveAccess(MemVar, const_cast<WExpr*>(E->first), false)) {
        return false;
      }
    }
  }
  if (Verbose) {
    llvm::outs() << "[PointerRemoval] indirect accesses to "
                 << MemVar->getName()
                 << " are replaced by direct accesses\n";
  }
  ApplyReplacements();
  return true;
}

bool PointerRemoval::RemoveArrayPointers(WVarDecl *MemVar) {
  assert(MemVar->getType()->isArrayType());
  Replacements.clear();
  DiscoverPointerVariables(MemVar, true);
  unsigned Alias = Aliases.getAlias(MemVar);
  for (AliasSet::expr_iterator E = Aliases.expr_begin(),
                               EEnd = Aliases.expr_end();
       E != EEnd; ++E) {
    if (E->second == Alias) {
      if (!ResolveAccess(MemVar, const_cast<WExpr*>(E->first), true)) {
        return false;
      }
    }
  }
  if (Verbose) {
    llvm::outs() << "[PointerRemoval] indirect accesses to "
                 << MemVar->getName()
                 << " are replaced by direct element accesses\n";
  }
  ApplyReplacements();
  return true;
}

} // namespace snu

} // namespace clang
