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

#include "clang/SnuAST/WCFG.h"
#include "clang/AST/Decl.h"
#include "clang/AST/Expr.h"
#include "clang/AST/Stmt.h"
#include "clang/AST/Type.h"
#include "clang/Analysis/CFG.h"
#include "clang/Analysis/Support/BumpVector.h"
#include "clang/Basic/LLVM.h"
#include "clang/SnuAST/CFGAnalysis.h"
#include "clang/SnuAST/WAST.h"
#include "clang/SnuAnalysis/Dominator.h"
#include "clang/SnuSupport/OrderedDenseADT.h"
#include "llvm/ADT/BitVector.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/Optional.h"
#include "llvm/ADT/SmallSet.h"
#include "llvm/Support/Allocator.h"
#include "llvm/Support/Format.h"
#include "llvm/Support/raw_ostream.h"
#include <set>

namespace clang {

namespace snu {

WExpr *WCFGBlock::getTerminatorCondition() const {
  if (!Terminator)
    return NULL;

  if (WIfStmt *If = dyn_cast<WIfStmt>(Terminator)) {
    return If->getCond();
  } else if (WSwitchStmt *Switch = dyn_cast<WSwitchStmt>(Terminator)) {
    return Switch->getCond();
  } else if (WWhileStmt *While = dyn_cast<WWhileStmt>(Terminator)) {
    return While->getCond();
  } else if (WDoStmt *Do = dyn_cast<WDoStmt>(Terminator)) {
    return Do->getCond();
  } else if (WForStmt *For = dyn_cast<WForStmt>(Terminator)) {
    return For->getCond();
  } else if (isa<WGotoStmt>(Terminator)) {
    return NULL;
  } else if (isa<WContinueStmt>(Terminator)) {
    return NULL;
  } else if (isa<WBreakStmt>(Terminator)) {
    return NULL;
  } else if (isa<WReturnStmt>(Terminator)) {
    return NULL;
  } else if (WBinaryOperator *BO = dyn_cast<WBinaryOperator>(Terminator)) {
    assert(BO->isLogicalOp());
    return BO->getLHS();
  } else if (WConditionalOperator *CO =
               dyn_cast<WConditionalOperator>(Terminator)) {
    return CO->getCond();
  } else {
    llvm_unreachable("invalid terminator node");
  }
}

unsigned WCFGBlock::getPredIndex(const WCFGBlock *Block) const {
  unsigned Index = 0;
  for (const_pred_iterator P = pred_begin(), PEnd = pred_end();
       P != PEnd; ++P, ++Index) {
    if (*P == Block) {
      return Index;
    }
  }
  llvm_unreachable("no predecessor");
  return 0;
}

unsigned WCFGBlock::getSuccIndex(const WCFGBlock *Block) const {
  unsigned Index = 0;
  for (const_succ_iterator S = succ_begin(), SEnd = succ_end();
       S != SEnd; ++S, ++Index) {
    if (*S == Block) {
      return Index;
    }
  }
  llvm_unreachable("no successor");
  return 0;
}

bool WCFGBlock::hasPhiFunction(const WVarDecl *Var) const {
  for (const_iterator I = begin(), E = end(); I != E; ++I) {
    if (WPhiFunction *PF = dyn_cast<WPhiFunction>(*I)) {
      if (PF->getIndexedLHSDecl()->getDecl() == Var)
        return true;
    } else {
      break;
    }
  }
  return false;
}

bool WCFGBlock::hasPhiFunction(const IndexedVarDecl *Var) const {
  if (!Var->getDefinedStmt() || !isa<WPhiFunction>(Var->getDefinedStmt()))
    return false;

  for (const_iterator I = begin(), E = end(); I != E; ++I) {
    if (WPhiFunction *PF = dyn_cast<WPhiFunction>(*I)) {
      if (PF->getIndexedLHSDecl() == Var)
        return true;
    } else {
      break;
    }
  }
  return false;
}

WPhiFunction *WCFGBlock::getPhiFunction(const WVarDecl *Var) const {
  for (const_iterator I = begin(), E = end(); I != E; ++I) {
    if (WPhiFunction *PF = dyn_cast<WPhiFunction>(*I)) {
      if (PF->getIndexedLHSDecl()->getDecl() == Var)
        return PF;
    } else {
      break;
    }
  }
  return NULL;
}

WPhiFunction *WCFGBlock::getPhiFunction(const IndexedVarDecl *Var) const {
  if (!Var->getDefinedStmt() || !isa<WPhiFunction>(Var->getDefinedStmt())) {
    return NULL;
  }

  for (const_iterator I = begin(), E = end(); I != E; ++I) {
    if (WPhiFunction *PF = dyn_cast<WPhiFunction>(*I)) {
      if (PF->getIndexedLHSDecl() == Var)
        return PF;
    } else {
      break;
    }
  }
  return NULL;
}

namespace {

typedef llvm::DenseMap<const Stmt*, WStmt*> WrappedStmtMapTy;

void DiscoverWrappedStmts(WStmt *Node, WrappedStmtMapTy &Map) {
  Map[Node->getOriginal()] = Node;

  if (WDeclStmt *DS = dyn_cast<WDeclStmt>(Node)) {
    for (DeclStmt::decl_iterator D = DS->decl_begin(),
                                 DEnd = DS->decl_end();
         D != DEnd; ++D) {
      if (WVarDecl *Var = DS->getVarDecl(D)) {
        if (Var->hasInit()) {
          DiscoverWrappedStmts(Var->getInit(), Map);
        }
      }
    }
  } else {
    for (WStmt::child_iterator C = Node->child_begin(),
                               CEnd = Node->child_end();
         C != CEnd; ++C) {
      if (*C) {
        DiscoverWrappedStmts(*C, Map);
      }
    }
  }
}

} // anonymous namespace

WCFGBlock::WCFGBlock(WCFG *parent, unsigned id)
  : Parent(parent), BlockID(id), BVCtx(parent->getBumpVectorContext()),
    Elements(BVCtx, 256), Label(NULL), Terminator(NULL), LoopTarget(NULL),
    Preds(BVCtx, 4), Succs(BVCtx, 4), RealSuccs(BVCtx, 4), NumBarriers(0) {}

WCFGBlock::iterator WCFGBlock::findStmt(WStmt *S) {
  for (iterator I = begin(), E = end(); I != E; ++I) {
    if ((*I)->contains(S)) {
      return I;
    }
  }
  return end();
}

WCFGBlock::const_iterator WCFGBlock::findStmt(WStmt *S) const {
  for (const_iterator I = begin(), E = end(); I != E; ++I) {
    if ((*I)->contains(S)) {
      return I;
    }
  }
  return end();
}

int WCFGBlock::compareLocation(WStmt *LHS, WStmt *RHS) const {
  for (ElementListTy::const_iterator I = Elements.begin(), E = Elements.end();
       I != E; ++I) {
    bool ContainsLHS = (*I)->contains(LHS);
    bool ContainsRHS = (*I)->contains(RHS);
    if (ContainsLHS && ContainsRHS) {
      return 0;
    } else if (ContainsLHS && !ContainsRHS) {
      return -1;
    } else if (!ContainsLHS && ContainsRHS) {
      return 1;
    }
  }
  return 0;
}

void WCFGBlock::appendStmt(WStmt *S) {
  Elements.push_back(S, BVCtx);
  if (WCallExpr *CE = dyn_cast<WCallExpr>(S)) {
    if (CE->isBarrier()) {
      NumBarriers++;
    }
  }
  assert(NumBarriers <= 1 && "a basic block may contain only a single barrier");
}

void WCFGBlock::insertStmtAtFront(WStmt *S) {
  ElementListTy::iterator I = Elements.begin();
  if (!isa<WPhiFunction>(S)) {
    while (I != Elements.end() && isa<WPhiFunction>(*I)) {
      ++I;
    }
  }
  insertStmtAt(S, I);
}

void WCFGBlock::insertStmtAt(WStmt *S, iterator I) {
  Elements.insert(I, 1, S, BVCtx);
  if (WCallExpr *CE = dyn_cast<WCallExpr>(S)) {
    if (CE->isBarrier()) {
      NumBarriers++;
    }
  }
}

void WCFGBlock::replaceStmt(WStmt *From, WStmt *To) {
  int NumBarriersDiff = 0;
  if (WCallExpr *CE = dyn_cast<WCallExpr>(From)) {
    if (CE->isBarrier()) {
      NumBarriersDiff--;
    }
  }
  if (WCallExpr *CE = dyn_cast<WCallExpr>(To)) {
    if (CE->isBarrier()) {
      NumBarriersDiff++;
    }
  }
  for (ElementListTy::iterator I = Elements.begin(), E = Elements.end();
       I != E; ++I) {
    if (*I == From) {
      *I = To;
      NumBarriers += NumBarriersDiff;
    } else {
      (*I)->replace(From, To);
    }
  }
  assert(NumBarriers <= 1 && "a basic block may contain only a single barrier");
}

WCFGBlock *WCFG::findStmt(WStmt *S) {
  for (WCFGBlockListTy::iterator I = Blocks.begin(), E = Blocks.end();
       I != E; ++I) {
    WCFGBlock *B = *I;
    if (B->findStmt(S) != B->end()) {
      return B;
    }
  }
  return NULL;
}

const WCFGBlock *WCFG::findStmt(WStmt *S) const {
  for (WCFGBlockListTy::const_iterator I = Blocks.begin(), E = Blocks.end();
       I != E; ++I) {
    const WCFGBlock *B = *I;
    if (B->findStmt(S) != B->end()) {
      return B;
    }
  }
  return NULL;
}

bool WCFG::isPotentiallyReachable(const WCFGBlock *From,
                                  const WCFGBlock *To) const {
  assert(From->getParent() == this && To->getParent() == this);
  CFGAnalysisContext Ctx(this);
  CFGDataflowWorklist Worklist;
  Worklist.enqueue(From);
  while (!Worklist.empty()) {
    const WCFGBlock *Block = Worklist.dequeueForever();
    assert(Block != NULL);
    if (Block == To) {
      return true;
    }
    Worklist.enqueueSuccessors(Block);
  }
  return false;
}

bool WCFG::isPotentiallyReachable(WStmt *From, WStmt *To) const {
  const WCFGBlock *FromBlock = findStmt(From);
  const WCFGBlock *ToBlock = findStmt(To);
  assert(FromBlock && ToBlock);
  if (FromBlock == ToBlock) {
    return FromBlock->compareLocation(From, To) <= 0;
  } else {
    return isPotentiallyReachable(FromBlock, ToBlock);
  }
}

WCFGBlock *WCFG::addBlock() {
  WCFGBlock *Block = getAllocator().Allocate<WCFGBlock>();
  new (Block) WCFGBlock(this, NumBlockIDs++);
  Blocks.push_back(Block, BVCtx);
  return Block;
}

void WCFG::markAsEntry(WCFGBlock *B) {
  assert(B->getParent() == this);
  assert(Entry == NULL);
  Entry = B;
}

void WCFG::markAsExit(WCFGBlock *B) {
  assert(B->getParent() == this);
  assert(Exit == NULL);
  Exit = B;
}

void WCFG::replaceStmt(WStmt *From, WStmt *To) {
  for (WCFGBlockListTy::iterator B = Blocks.begin(), BEnd = Blocks.end();
       B != BEnd; ++B) {
    (*B)->replaceStmt(From, To);
  }
}

bool WCFG::hasBarrier() const {
  for (WCFGBlockListTy::const_iterator B = Blocks.begin(), BEnd = Blocks.end();
       B != BEnd; ++B) {
    if ((*B)->hasBarrier()) {
      return true;
    }
  }
  return false;
}

WCFG *WCFG::WrapClangCFG(WDeclContext &DeclCtx, WStmt *AST, CFG *cfg) {
  WCFG *wcfg = new WCFG(cfg);

  WrappedStmtMapTy WrappedStmts;
  DiscoverWrappedStmts(AST, WrappedStmts);
  for (CFG::synthetic_stmt_iterator I = cfg->synthetic_stmt_begin(),
                                    E = cfg->synthetic_stmt_end();
       I != E; ++I) {
    const DeclStmt *DS = I->first;
    assert(DS->isSingleDecl());
    WVarDecl *WVD = NULL;
    if (const VarDecl *VD = dyn_cast<VarDecl>(DS->getSingleDecl())) {
      WVD = DeclCtx.Lookup(VD);
      assert(WVD != NULL);
    }
    WDeclStmt *WDS = new (*wcfg) WDeclStmt(const_cast<DeclStmt*>(DS), WVD);
    DiscoverWrappedStmts(WDS, WrappedStmts);
  }

  llvm::DenseMap<CFGBlock*, WCFGBlock*> WrappedFirstBlock;
  llvm::DenseMap<CFGBlock*, WCFGBlock*> WrappedLastBlock;
  for (CFG::iterator I = cfg->begin(), E = cfg->end();
       I != E; ++I) {
    CFGBlock *Block = *I;
    WCFGBlock *WBlock = wcfg->addBlock();

    WrappedFirstBlock[Block] = WBlock;
    if (&(cfg->getEntry()) == Block) {
      wcfg->markAsEntry(WBlock);
    }
    if (const Stmt *Label = Block->getLabel()) {
      assert(WrappedStmts.count(Label));
      WBlock->setLabel(WrappedStmts[Label]);
    }

    for (CFGBlock::iterator BI = Block->begin(), BEnd = Block->end();
         BI != BEnd; ++BI) {
      if (Optional<CFGStmt> SE = BI->getAs<CFGStmt>()) {
        const Stmt *Stmt = SE->getStmt();
        assert(WrappedStmts.count(Stmt));
        if (WCallExpr *CE = dyn_cast<WCallExpr>(WrappedStmts[Stmt])) {
          if (CE->isBarrier() && !WBlock->empty()) {
            WCFGBlock *NextBlock = wcfg->addBlock();
            WBlock->addSuccessor(NextBlock);
            NextBlock->addPredecessor(WBlock);
            WBlock = NextBlock;
          }
        }
        WBlock->appendStmt(WrappedStmts[Stmt]);
      } else {
        llvm_unreachable("invalid CFG element");
      }
    }

    if (const Stmt *Terminator = Block->getTerminator().getStmt()) {
      assert(WrappedStmts.count(Terminator));
      WBlock->setTerminator(WrappedStmts[Terminator]);
    }
    if (const Stmt *LoopTarget = Block->getLoopTarget()) {
      assert(WrappedStmts.count(LoopTarget));
      WBlock->setLoopTarget(WrappedStmts[LoopTarget]);
    }
    if (&(cfg->getExit()) == Block) {
      wcfg->markAsExit(WBlock);
    }
    WrappedLastBlock[Block] = WBlock;
  }

  for (CFG::iterator I = cfg->begin(), E = cfg->end();
       I != E; ++I) {
    CFGBlock *Block = *I;
    WCFGBlock *WBlock;
    
    WBlock = WrappedFirstBlock[Block];
    assert(WBlock->pred_empty());
    for (CFGBlock::pred_iterator P = Block->pred_begin(),
                                 PEnd = Block->pred_end();
         P != PEnd; ++P) {
      assert(WrappedLastBlock.count(*P));
      WBlock->addPredecessor(WrappedLastBlock[*P]);
    }

    WBlock = WrappedLastBlock[Block];
    assert(WBlock->succ_empty());
    for (CFGBlock::succ_iterator S = Block->succ_begin(),
                                 SEnd = Block->succ_end();
         S != SEnd; ++S) {
      if (*S) {
        assert(WrappedFirstBlock.count(*S));
        WBlock->addSuccessor(WrappedFirstBlock[*S]);
      } else {
        WBlock->addSuccessor(NULL);
      }
    }
  }

  return wcfg;
}


// SSA form

namespace {

class SSABuilder {
  typedef OrderedDenseSet<WVarDecl*> VarSet;
  typedef llvm::DenseMap<WVarDecl*, CFGBlockBitVector> VarBlockSetMap;
  typedef llvm::DenseMap<WVarDecl*, unsigned> VarIndexMap;
  typedef llvm::DenseMap<WVarDecl*, SmallVector<IndexedVarDecl*, 8> >
          VarIVarStackMap;

  class DefUseListEntry {
  public:
    WStmt *RefStmt;
    WVarDecl *Var;
    bool isDef;
    WStmt *DefStmt;

    DefUseListEntry(WStmt *refStmt, WVarDecl *var)
      : RefStmt(refStmt), Var(var), isDef(false), DefStmt(NULL) {}
    DefUseListEntry(WStmt *refStmt, WVarDecl *var, WStmt *defStmt)
      : RefStmt(refStmt), Var(var), isDef(true), DefStmt(defStmt) {}
  };
  typedef SmallVector<DefUseListEntry, 16> DefUseList;

  WCFG *cfg;

  VarSet VariableSet;
  VarSet PreInitVariableSet;
  VarSet BailoutVariableSet;

  VarBlockSetMap DefNodes;
  VarBlockSetMap PhiNodes;
  VarIndexMap Count;
  VarIVarStackMap Stack;

public:
  void build(WCFG *cfg);
  void increment(WCFG *cfg, WVarDecl *var);

private:
  void DiscoverVariables();
  void FindAllVariables(WStmt *S, bool Outermost = false);
  void FindBailoutVariables(WExpr *E);
  void UpdateVariableSet(WVarDecl *Var);

  void InsertPhiFunctions(WCFGDominanceFrontier &DF);
  void FindDefNodes(WCFGBlock *Block, WStmt *S, bool Outermost = false);
  WVarDecl *GetLHSVar(WExpr *E);

  void RenameVariables(WCFGDominatorTree &DT);
  void RenameVariables(WCFGDomTreeNode *Node);
  void FindDefsAndUses(DefUseList &DUList, WStmt *S, bool Outermost = false);
  void FindDefsAndUsesInLHS(DefUseList &DUList, WExpr *E, WExpr *AssignmentExpr,
                            bool Compound);

  void SetInsert(VarSet &Set, WVarDecl *Var);
  bool SetContains(VarSet &Set, WVarDecl *Var);

  IndexedVarDeclRef GetCurrentIndexedVar(WVarDecl *Var);
  IndexedVarDeclRef GetNextIndexedVar(WVarDecl *Var);
  void PopCurrentIndexedVar(WVarDecl *Var);
  IndexedVarDecl *CreateIndexedVar(WVarDecl *Var, unsigned Index);
};

void SSABuilder::build(WCFG *cfg) {
  VariableSet.clear();
  PreInitVariableSet.clear();
  BailoutVariableSet.clear();
  DefNodes.clear();
  PhiNodes.clear();
  Count.clear();
  Stack.clear();

  if (!cfg->isSSA()) {
    this->cfg = cfg;
    CFGAnalysisContext Ctx(cfg);

    WCFGDominatorTree DT(false);
    DT.recalculate(*cfg);
    WCFGDominanceFrontier DF;
    DF.recalculate(DT);

    DiscoverVariables();
    InsertPhiFunctions(DF);
    RenameVariables(DT);
    cfg->setSSA();
  }
}

void SSABuilder::increment(WCFG *cfg, WVarDecl *var) {
  VariableSet.clear();
  PreInitVariableSet.clear();
  BailoutVariableSet.clear();
  DefNodes.clear();
  PhiNodes.clear();
  Count.clear();
  Stack.clear();

  assert(cfg->isSSA());

  this->cfg = cfg;
  CFGAnalysisContext Ctx(cfg);

  WCFGDominatorTree DT(false);
  DT.recalculate(*cfg);
  WCFGDominanceFrontier DF;
  DF.recalculate(DT);

  UpdateVariableSet(var);
  InsertPhiFunctions(DF);
  RenameVariables(DT);
}

void SSABuilder::DiscoverVariables() {
  for (WCFG::const_iterator B = cfg->begin(), BEnd = cfg->end();
       B != BEnd; ++B) {
    WCFGBlock *Block = *B;
    for (WCFGBlock::iterator S = Block->begin(), SEnd = Block->end();
         S != SEnd; ++S) {
      FindAllVariables(*S, true);
    }
  }

  for (VarSet::const_iterator I = BailoutVariableSet.begin(),
                              E = BailoutVariableSet.end();
       I != E; ++I) {
    VariableSet.erase(*I);
    PreInitVariableSet.erase(*I);
  }
}

void SSABuilder::FindAllVariables(WStmt *S, bool Outermost) {
  if (!S)
    return;

  if (WDeclStmt *DS = dyn_cast<WDeclStmt>(S)) {
    assert(DS->isSingleDecl());
    if (WVarDecl *Var = DS->getSingleVarDecl()) {
      UpdateVariableSet(Var);
      if (Var->hasInit()) {
        FindAllVariables(Var->getInit());
      }
    }

  } else if (WDeclRefExpr *DRE = dyn_cast<WDeclRefExpr>(S)) {
    if (WVarDecl *Var = DRE->getVarDecl()) {
      UpdateVariableSet(Var);
    }

  } else if (WUnaryOperator *UO = dyn_cast<WUnaryOperator>(S)) {
    FindAllVariables(UO->getSubExpr());
    if (UO->getOpcode() == UO_AddrOf) {
      FindBailoutVariables(UO->getSubExpr());
    }

  } else if (WCallExpr *CE = dyn_cast<WCallExpr>(S)) {
    if (Outermost) {
      FindAllVariables(CE->getCallee());
      for (unsigned Index = 0, NumArgs = CE->getNumArgs();
           Index != NumArgs; ++Index) {
        FindAllVariables(CE->getArg(Index));
      }
    }

  } else if (WMemberExpr *ME = dyn_cast<WMemberExpr>(S)) {
    if (ME->isArrow()) {
      FindAllVariables(ME->getBase());
    } else {
      if (WVarDecl *Var = ME->getVarDecl()) {
        UpdateVariableSet(Var);
      } else {
        FindAllVariables(ME->getBase());
      }
    }

  } else if (WCastExpr *CE = dyn_cast<WCastExpr>(S)) {
    FindAllVariables(CE->getSubExpr());
    if (CE->getCastKind() == CK_ArrayToPointerDecay) {
      FindBailoutVariables(CE->getSubExpr());
    }

  } else if (WBinaryOperator *BO = dyn_cast<WBinaryOperator>(S)) {
    if (BO->isLogicalOp() || BO->getOpcode() == BO_Comma) {
      // Do nothing
    } else {
      FindAllVariables(BO->getLHS());
      FindAllVariables(BO->getRHS());
    }

  } else if (isa<WConditionalOperator>(S)) {
    // Do nothing

  } else if (WExtVectorElementExpr *VEE = dyn_cast<WExtVectorElementExpr>(S)) {
    if (WVarDecl *Var = VEE->getVarDecl()) {
      UpdateVariableSet(Var);
    } else {
      FindAllVariables(VEE->getBase());
    }

  } else {
    for (WStmt::child_iterator I = S->child_begin(), E = S->child_end();
         I != E; ++I) {
      FindAllVariables(*I);
    }
  }
}

void SSABuilder::FindBailoutVariables(WExpr *E) {
  if (!E)
    return;
  E = E->IgnoreParenCasts();
  if (WDeclRefExpr *DRE = dyn_cast<WDeclRefExpr>(E)) {
    if (WVarDecl *Var = DRE->getVarDecl()) {
      SetInsert(BailoutVariableSet, Var);
    }
  } else if (WArraySubscriptExpr *ASE = dyn_cast<WArraySubscriptExpr>(E)) {
    if (ASE->getBase()->getType()->isArrayType()) {
      FindBailoutVariables(ASE->getBase());
    }
  } else if (WMemberExpr *ME = dyn_cast<WMemberExpr>(E)) {
    if (WVarDecl *Var = ME->getVarDecl()) {
      SetInsert(BailoutVariableSet, Var);
    } else {
      FindBailoutVariables(ME->getBase());
    }
  }
  // Note that &(WExtVectorElementExpr) is not defined.
}

void SSABuilder::UpdateVariableSet(WVarDecl *Var) {
  if (Var->hasAddressSpace()) return;

  QualType Ty = Var->getType();
  if (Ty->isScalarType() || Ty->isVectorType()) {
   // pass

  } else if (Ty->isStructureType()) {
    const RecordType *StructTy = Ty->getAsStructureType();
    assert(StructTy != NULL);
    RecordDecl *Struct = StructTy->getDecl();
    assert(Struct != NULL);
    Struct = Struct->getDefinition();
    assert(Struct != NULL && Struct->isCompleteDefinition());
    for (RecordDecl::field_iterator F = Struct->field_begin(),
                                    FEnd = Struct->field_end();
         F != FEnd; ++F) {
      assert((*F) != NULL);
      if ((*F)->getType()->isScalarType()) {
        // pass
      } else {
        return;
      }
    }

  } else if (Ty->isConstantArrayType()) {
    const ConstantArrayType *ArrayTy = dyn_cast<ConstantArrayType>(Ty->getAsArrayTypeUnsafe());
    assert(ArrayTy != NULL);
    QualType ElementTy = ArrayTy->getElementType();
    if (ElementTy->isScalarType() || ElementTy->isVectorType()) {
      // pass
    } else {
      return;
    }

  } else {
    return;
  }

  SetInsert(VariableSet, Var);
  if (Var->isParameter()) {
    SetInsert(PreInitVariableSet, Var);
  }
}

void SSABuilder::InsertPhiFunctions(WCFGDominanceFrontier &DF) {
  for (WCFG::const_iterator B = cfg->begin(), BEnd = cfg->end();
       B != BEnd; ++B) {
    WCFGBlock *Block = *B;
    for (WCFGBlock::iterator S = Block->begin(), SEnd = Block->end();
         S != SEnd; ++S) {
      FindDefNodes(Block, *S, true);
    }
  }

  for (VarSet::const_iterator I = VariableSet.begin(), E = VariableSet.end();
       I != E; ++I) {
    WVarDecl *VD = *I;
    if (!DefNodes.count(VD)) {
      continue;
    }
    CFGBlockBitVector WorkSet = DefNodes[VD];

    while (WorkSet.any()) {
      int BlockID = WorkSet.find_first();
      assert(BlockID != -1);
      WorkSet.reset((unsigned)BlockID);

      WCFGBlock *Block = cfg->getBlock(BlockID);
      for (WCFGDominanceFrontier::frontier_iterator
               F = DF.frontier_begin(Block), FEnd = DF.frontier_end(Block);
           F != FEnd; ++F) {
        WCFGBlock *Frontier = *F;
        unsigned FrontierID = Frontier->getBlockID();
        if (!PhiNodes[VD].test(FrontierID)) {
          WPhiFunction *PF = WPhiFunction::Create(Frontier, VD);
          Frontier->insertStmtAtFront(PF);
          PhiNodes[VD].set(FrontierID);
          if (!DefNodes[VD].test(FrontierID)) {
            WorkSet.set(FrontierID);
          }
        }
      }
    }
  }
}

void SSABuilder::FindDefNodes(WCFGBlock *Block, WStmt *S, bool Outermost) {
  if (!S)
    return;

  WVarDecl *DefVar = NULL;

  if (WDeclStmt *DS = dyn_cast<WDeclStmt>(S)) {
    assert(DS->isSingleDecl());
    if (DS->hasSingleInit()) {
      FindDefNodes(Block, DS->getSingleInit());
      DefVar = DS->getSingleVarDecl();
    }

  } else if (WUnaryOperator *UO = dyn_cast<WUnaryOperator>(S)) {
    FindDefNodes(Block, UO->getSubExpr());
    if (UO->isIncrementDecrementOp()) {
      DefVar = GetLHSVar(UO->getSubExpr());
    }

  } else if (WCallExpr *CE = dyn_cast<WCallExpr>(S)) {
    if (Outermost) {
      FindDefNodes(Block, CE->getCallee());
      for (unsigned Index = 0, NumArgs = CE->getNumArgs();
           Index != NumArgs; ++Index) {
        FindDefNodes(Block, CE->getArg(Index));
      }
    }

  } else if (WBinaryOperator *BO = dyn_cast<WBinaryOperator>(S)) {
    if (BO->isAssignmentOp()) {
      FindDefNodes(Block, BO->getRHS());
      FindDefNodes(Block, BO->getLHS());
      DefVar = GetLHSVar(BO->getLHS());

    } else if (BO->isLogicalOp() || BO->getOpcode() == BO_Comma) {
      // Do nothing

    } else {
      FindDefNodes(Block, BO->getLHS());
      FindDefNodes(Block, BO->getRHS());
    }

  } else if (isa<WConditionalOperator>(S)) {
    // Do nothing

  } else {
    for (WStmt::child_iterator I = S->child_begin(), E = S->child_end();
         I != E; ++I) {
      FindDefNodes(Block, *I);
    }
  }

  if (DefVar) {
    if (DefVar->isCompound()) {
      for (unsigned Index = 0, NumSubVars = DefVar->getNumSubVars();
           Index != NumSubVars; ++Index) {
        WVarDecl *SubVar = DefVar->getSubVar(Index);
        if (VariableSet.count(SubVar)) {
          DefNodes[SubVar].set(Block->getBlockID());
        }
      }
    } else {
      if (VariableSet.count(DefVar)) {
        DefNodes[DefVar].set(Block->getBlockID());
      }
    }
  }
}

WVarDecl *SSABuilder::GetLHSVar(WExpr *E) {
  E = E->IgnoreParenCasts();
  if (WDeclRefExpr *DRE = dyn_cast<WDeclRefExpr>(E)) {
    if (WVarDecl *VD = DRE->getVarDecl()) {
      return VD;
    }
  } else if (WArraySubscriptExpr *ASE = dyn_cast<WArraySubscriptExpr>(E)) {
    if (ASE->getBase()->getType()->isArrayType()) {
      return GetLHSVar(ASE->getBase());
    }
  } else if (WMemberExpr *ME = dyn_cast<WMemberExpr>(E)) {
    if (!ME->isArrow()) {
      if (WVarDecl *VD = ME->getVarDecl()) {
        return VD;
      } else {
        return GetLHSVar(ME->getBase());
      }
    }
  } else if (WExtVectorElementExpr *VEE = dyn_cast<WExtVectorElementExpr>(E)) {
    if (WVarDecl *VD = VEE->getVarDecl()) {
      return VD;
    } else {
      return GetLHSVar(VEE->getBase());
    }
  }
  return NULL;
}

void SSABuilder::RenameVariables(WCFGDominatorTree &DT) {
  for (VarSet::const_iterator I = VariableSet.begin(), E = VariableSet.end();
       I != E; ++I) {
    assert(!(*I)->isCompound());
    if (PreInitVariableSet.count(*I)) {
      Count[*I] = 1;
      Stack[*I].push_back(CreateIndexedVar(*I, 1));
    } else {
      Count[*I] = 0;
      Stack[*I].push_back(CreateIndexedVar(*I, 0));
    }
  }
  RenameVariables(DT.getRootNode());
}

void SSABuilder::RenameVariables(WCFGDomTreeNode *Node) {
  WCFGBlock *Block = Node->getBlock();

  DefUseList DUList;
  for (WCFGBlock::iterator I = Block->begin(), E = Block->end();
       I != E; ++I) {
    FindDefsAndUses(DUList, *I, true);
  }

  for (DefUseList::iterator I = DUList.begin(), E = DUList.end();
       I != E; ++I) {
    DefUseListEntry DU = *I;
    if (DU.isDef) {
      IndexedVarDeclRef IVarRef = GetNextIndexedVar(DU.Var);
      IVarRef.setDefinedStmt(DU.DefStmt);
      if (WDeclRefExpr *DRE = dyn_cast<WDeclRefExpr>(DU.RefStmt)) {
        DRE->setIndexedDefDecl(IVarRef);
      } else if (WMemberExpr *ME = dyn_cast<WMemberExpr>(DU.RefStmt)) {
        ME->setIndexedDefDecl(IVarRef);
      } else if (WExtVectorElementExpr *VEE =
          dyn_cast<WExtVectorElementExpr>(DU.RefStmt)) {
        VEE->setIndexedDefDecl(IVarRef);
      } else if (WDeclStmt *DS = dyn_cast<WDeclStmt>(DU.RefStmt)) {
        DS->setIndexedDecl(IVarRef);
      } else if (WPhiFunction *PF = dyn_cast<WPhiFunction>(DU.RefStmt)) {
        assert(IVarRef.isSingleVar());
        PF->setIndexedLHSDecl(*IVarRef);
      } else {
        llvm_unreachable("invalid statement");
      }
    } else {
      IndexedVarDeclRef IVarRef = GetCurrentIndexedVar(DU.Var);
      if (WDeclRefExpr *DRE = dyn_cast<WDeclRefExpr>(DU.RefStmt)) {
        DRE->setIndexedUseDecl(IVarRef);
      } else if (WMemberExpr *ME = dyn_cast<WMemberExpr>(DU.RefStmt)) {
        ME->setIndexedUseDecl(IVarRef);
      } else if (WExtVectorElementExpr *VEE =
          dyn_cast<WExtVectorElementExpr>(DU.RefStmt)) {
        VEE->setIndexedUseDecl(IVarRef);
      } else {
        llvm_unreachable("invalid statement");
      }
    }
  }

  for (WCFGBlock::succ_iterator S = Block->real_succ_begin(),
                                SEnd = Block->real_succ_end();
       S != SEnd; ++S) {
    for (WCFGBlock::iterator SI = (*S)->begin(), SIEnd = (*S)->end();
         SI != SIEnd; ++SI) {
      if (WPhiFunction *PF = dyn_cast<WPhiFunction>(*SI)) {
        WVarDecl *Var = PF->getVarDecl();
        if (VariableSet.count(Var)) {
          assert(!Stack[Var].empty());
          PF->setIndexedArgDecl((*S)->getPredIndex(Block), Stack[Var].back());
        }
      } else {
        break;
      }
    }
  }

  for (WCFGDomTreeNode::iterator C = Node->begin(), CEnd = Node->end();
       C != CEnd; ++C) {
    RenameVariables(*C);
  }

  for (DefUseList::reverse_iterator I = DUList.rbegin(), E = DUList.rend();
       I != E; ++I) {
    if ((*I).isDef) {
      PopCurrentIndexedVar((*I).Var);
    }
  }
}


void SSABuilder::FindDefsAndUses(DefUseList &DUList, WStmt *S, bool Outermost) {
  if (!S)
    return;

  if (WDeclStmt *DS = dyn_cast<WDeclStmt>(S)) {
    assert(DS->isSingleDecl());
    if (DS->hasSingleInit()) {
      FindDefsAndUses(DUList, DS->getSingleInit());

      WVarDecl *Var = DS->getSingleVarDecl();
      assert(Var != NULL);
      if (SetContains(VariableSet, Var)) {
        DUList.push_back(DefUseListEntry(DS, Var, DS));
      }
    }

  } else if (WDeclRefExpr *DRE = dyn_cast<WDeclRefExpr>(S)) {
    if (WVarDecl *Var = DRE->getVarDecl()) {
      if (SetContains(VariableSet, Var)) {
        DUList.push_back(DefUseListEntry(DRE, Var));
      }
    }

  } else if (WUnaryOperator *UO = dyn_cast<WUnaryOperator>(S)) {
    if (UO->isIncrementDecrementOp()) {
      FindDefsAndUsesInLHS(DUList, UO->getSubExpr(), UO, true);
    } else {
      FindDefsAndUses(DUList, UO->getSubExpr());
    }

  } else if (WCallExpr *CE = dyn_cast<WCallExpr>(S)) {
    if (Outermost) {
      FindDefsAndUses(DUList, CE->getCallee());
      for (unsigned Index = 0, NumArgs = CE->getNumArgs();
           Index != NumArgs; ++Index) {
        FindDefsAndUses(DUList, CE->getArg(Index));
      }
    }

  } else if (WMemberExpr *ME = dyn_cast<WMemberExpr>(S)) {
    if (ME->isArrow()) {
      FindDefsAndUses(DUList, ME->getBase());
    } else {
      if (WVarDecl *Var = ME->getVarDecl()) {
        if (SetContains(VariableSet, Var)) {
          DUList.push_back(DefUseListEntry(ME, Var));
        }
      } else {
        FindDefsAndUses(DUList, ME->getBase());
      }
    }

  } else if (WBinaryOperator *BO = dyn_cast<WBinaryOperator>(S)) {
    if (BO->isAssignmentOp()) {
      FindDefsAndUses(DUList, BO->getRHS());
      FindDefsAndUsesInLHS(DUList, BO->getLHS(), BO,
                           BO->isCompoundAssignmentOp());

    } else if (BO->isLogicalOp() || BO->getOpcode() == BO_Comma) {
      // Do nothing

    } else {
      FindDefsAndUses(DUList, BO->getLHS());
      FindDefsAndUses(DUList, BO->getRHS());
    }

  } else if (isa<WConditionalOperator>(S)) {
    // Do nothing

  } else if (WExtVectorElementExpr *VEE = dyn_cast<WExtVectorElementExpr>(S)) {
    if (WVarDecl *Var = VEE->getVarDecl()) {
      if (SetContains(VariableSet, Var)) {
        DUList.push_back(DefUseListEntry(VEE, Var));
      }
    } else {
      FindDefsAndUses(DUList, VEE->getBase());
    }

  } else if (WPhiFunction *PF = dyn_cast<WPhiFunction>(S)) {
    WVarDecl *Var = PF->getVarDecl();
    if (SetContains(VariableSet, Var)) {
      DUList.push_back(DefUseListEntry(PF, Var, PF));
    }
    // do not add arguments to the def-use list

  } else {
    for (WStmt::child_iterator I = S->child_begin(), E = S->child_end();
         I != E; ++I) {
      FindDefsAndUses(DUList, *I);
    }
  }
}

void SSABuilder::FindDefsAndUsesInLHS(DefUseList &DUList, WExpr *E,
                                      WExpr *AssignmentExpr, bool Compound) {
  E = E->IgnoreParenCasts();

  if (WDeclRefExpr *DRE = dyn_cast<WDeclRefExpr>(E)) {
    if (WVarDecl *Var = DRE->getVarDecl()) {
      if (SetContains(VariableSet, Var)) {
        if (Compound || Var->getType()->isArrayType()) {
          DUList.push_back(DefUseListEntry(DRE, Var));
          DUList.push_back(DefUseListEntry(DRE, Var, AssignmentExpr));
        } else {
          DUList.push_back(DefUseListEntry(DRE, Var, AssignmentExpr));
          DUList.push_back(DefUseListEntry(DRE, Var));
        }
      }
    }

  } else if (WArraySubscriptExpr *ASE = dyn_cast<WArraySubscriptExpr>(E)) {
    if (ASE->getBase()->getType()->isArrayType()) {
      FindDefsAndUsesInLHS(DUList, ASE->getBase(), AssignmentExpr, Compound);
      FindDefsAndUses(DUList, ASE->getIdx());
    } else {
      FindDefsAndUses(DUList, ASE->getBase());
      FindDefsAndUses(DUList, ASE->getIdx());
    }

  } else if (WMemberExpr *ME = dyn_cast<WMemberExpr>(E)) {
    if (ME->isArrow()) {
      FindDefsAndUses(DUList, ME->getBase());
    } else {
      if (WVarDecl *Var = ME->getVarDecl()) {
        if (SetContains(VariableSet, Var)) {
          if (Compound) {
            DUList.push_back(DefUseListEntry(ME, Var));
            DUList.push_back(DefUseListEntry(ME, Var, AssignmentExpr));
          } else {
            DUList.push_back(DefUseListEntry(ME, Var, AssignmentExpr));
            DUList.push_back(DefUseListEntry(ME, Var));
          }
        }
      } else {
        FindDefsAndUsesInLHS(DUList, ME->getBase(), AssignmentExpr, Compound);
      }
    }

  } else if (WExtVectorElementExpr *VEE = dyn_cast<WExtVectorElementExpr>(E)) {
    if (WVarDecl *Var = VEE->getVarDecl()) {
      if (SetContains(VariableSet, Var)) {
        if (Compound) {
          DUList.push_back(DefUseListEntry(VEE, Var));
          DUList.push_back(DefUseListEntry(VEE, Var, AssignmentExpr));
        } else {
          DUList.push_back(DefUseListEntry(VEE, Var, AssignmentExpr));
          DUList.push_back(DefUseListEntry(VEE, Var));
        }
      }
    } else {
      FindDefsAndUsesInLHS(DUList, VEE->getBase(), AssignmentExpr, Compound);
    }

  } else {
    FindDefsAndUses(DUList, E);
  }
}

void SSABuilder::SetInsert(VarSet &Set, WVarDecl *Var) {
  if (Var->isCompound()) {
    for (unsigned Index = 0, NumSubVars = Var->getNumSubVars();
         Index != NumSubVars; ++Index) {
      Set.insert(Var->getSubVar(Index));
    }
  } else {
    Set.insert(Var);
  }
}

bool SSABuilder::SetContains(VarSet &Set, WVarDecl *Var) {
  if (Var->isCompound()) {
    assert(Var->getNumSubVars() > 0);
    bool Contained = Set.count(Var->getSubVar(0));
    for (unsigned Index = 1, NumSubVars = Var->getNumSubVars();
         Index != NumSubVars; ++Index) {
      assert((bool)Set.count(Var->getSubVar(Index)) == Contained);
    }
    return Contained;
  } else {
    return Set.count(Var);
  }
}

IndexedVarDeclRef SSABuilder::GetCurrentIndexedVar(WVarDecl *Var) {
  if (Var->isCompound()) {
    SmallVector<IndexedVarDecl*, 8> IVars;
    for (unsigned Index = 0, NumSubVars = Var->getNumSubVars();
         Index != NumSubVars; ++Index) {
      WVarDecl *SubVar = Var->getSubVar(Index);
      assert(VariableSet.count(SubVar));
      assert(!Stack[SubVar].empty());
      IVars.push_back(Stack[SubVar].back());
    }
    return IndexedVarDeclRef(cfg->getAllocator(), IVars);
  } else {
    assert(VariableSet.count(Var));
    assert(!Stack[Var].empty());
    return IndexedVarDeclRef(Stack[Var].back());
  }
}

IndexedVarDeclRef SSABuilder::GetNextIndexedVar(WVarDecl *Var) {
  if (Var->isCompound()) {
    SmallVector<IndexedVarDecl*, 8> IVars;
    for (unsigned Index = 0, NumSubVars = Var->getNumSubVars();
         Index != NumSubVars; ++Index) {
      WVarDecl *SubVar = Var->getSubVar(Index);
      assert(VariableSet.count(SubVar));
      unsigned NewIndex = ++Count[SubVar];
      IndexedVarDecl *NewIVar = CreateIndexedVar(SubVar, NewIndex);
      Stack[SubVar].push_back(NewIVar);
      IVars.push_back(NewIVar);
    }
    return IndexedVarDeclRef(cfg->getAllocator(), IVars);
  } else {
    assert(VariableSet.count(Var));
    unsigned NewIndex = ++Count[Var];
    IndexedVarDecl *NewIVar = CreateIndexedVar(Var, NewIndex);
    Stack[Var].push_back(NewIVar);
    return IndexedVarDeclRef(NewIVar);
  }
}

void SSABuilder::PopCurrentIndexedVar(WVarDecl *Var) {
  if (Var->isCompound()) {
    for (unsigned Index = 0, NumSubVars = Var->getNumSubVars();
         Index != NumSubVars; ++Index) {
      WVarDecl *SubVar = Var->getSubVar(Index);
      assert(VariableSet.count(SubVar));
      assert(!Stack[SubVar].empty());
      Stack[SubVar].pop_back();
    }
  } else {
    assert(VariableSet.count(Var));
    assert(!Stack[Var].empty());
    Stack[Var].pop_back();
  }
}

IndexedVarDecl *SSABuilder::CreateIndexedVar(WVarDecl *Var, unsigned Index) {
  void *Mem = cfg->getAllocator().Allocate<IndexedVarDecl>();
  IndexedVarDecl *IVD = new (Mem) IndexedVarDecl(Var, Index);
  cfg->addSSAVar(IVD);
  return IVD;
}

} // anonymous namespace

void WCFG::MakeSSAForm() {
  SSABuilder builder;
  builder.build(this);
}

void WCFG::IncrementSSAVariable(WVarDecl *Var) {
  SSABuilder builder;
  builder.increment(this, Var);
}


// A very simple CFG printer

void WCFGBlock::print(raw_ostream &OS, const LangOptions &LO) const {
  OS << "[B" << getBlockID();
  if (this == Parent->getEntry()) {
    OS << " (ENTRY)]\n";
  } else if (this == Parent->getExit()) {
    OS << " (EXIT)]\n";
  } else {
    OS << "]\n";
  }

  if (WStmt *Label = getLabel()) {
    if (WLabelStmt *L = dyn_cast<WLabelStmt>(Label)) {
      OS << L->getName();
    } else if (WCaseStmt *C = dyn_cast<WCaseStmt>(Label)) {
      OS << "case ";
      C->getLHS()->print(OS, PrintingPolicy(LO));
    } else if (isa<WDefaultStmt>(Label)) {
      OS << "default";
    } else {
      llvm_unreachable("invalid label statement in WCFGBlock");
    }
    OS << ":\n";
  }
  unsigned StmtIndex = 1;
  for (const_iterator I = begin(), E = end(); I != E; ++I, ++StmtIndex) {
    OS << ' ' << llvm::format("%3d", StmtIndex) << ": ";
    if ((*I)->isClangStmt()) {
      if (isa<WExpr>(*I)) {
        (*I)->print(OS, PrintingPolicy(LO));
        OS << '\n';
      } else {
        (*I)->print(OS, PrintingPolicy(LO));
      }
    } else {
      (*I)->print(OS, PrintingPolicy(LO));
      OS << '\n';
    }
  }
  if (WStmt *Terminator = getTerminator()) {
    OS << "   T: " << Terminator->getStmtClassName() << '\n';
  }

  if (!pred_empty()) {
    OS << "  Preds (" << pred_size() << "):";
    for (const_pred_iterator I = pred_begin(), E = pred_end(); I != E; ++I) {
      OS << " B" << (*I)->getBlockID();
    }
    OS << '\n';
  }
  if (!succ_empty()) {
    OS << "  Succs (" << succ_size() << "):";
    for (const_succ_iterator I = succ_begin(), E = succ_end(); I != E; ++I) {
      if (*I) {
        OS << " B" << (*I)->getBlockID();
      } else {
        OS << " NULL";
      }
    }
    OS << '\n';
  }
}

void WCFG::print(raw_ostream &OS, const LangOptions &LO) const {
  getEntry()->print(OS, LO);
  for (const_iterator I = begin(), E = end(); I != E; ++I) {
    if (*I == getEntry() || *I == getExit())
      continue;
    (*I)->print(OS, LO);
  }
  getExit()->print(OS, LO);
  OS << '\n';
  OS.flush();
}

} // namespace snu

} // namespace clang
