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

#include "clang/SnuAnalysis/LiveVariables.h"
#include "clang/Basic/LLVM.h"
#include "clang/SnuAST/CFGAnalysis.h"
#include "clang/SnuAST/WAST.h"
#include "clang/SnuAST/WCFG.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/ImmutableSet.h"
#include "llvm/Support/raw_ostream.h"

namespace clang {

namespace snu {

namespace {

typedef LiveSSAVariables::LivenessValues LivenessValues;
typedef LiveSSAVariables::LivenessMapTy LivenessMapTy;

class LiveSSAVariablesImpl {
  LivenessValues::Factory SetFactory;
  LivenessMapTy &LiveIn;
  LivenessMapTy &LiveOut;
  LivenessMapTy Def, Use, Phi;
  AdditionalLiveness *AUses;
  AdditionalLiveness *ADefs;

public:
  LiveSSAVariablesImpl(LivenessMapTy &liveIn, LivenessMapTy &liveOut)
    : LiveIn(liveIn), LiveOut(liveOut), AUses(NULL), ADefs(NULL) {}

  void Analysis(WCFG *cfg, AdditionalLiveness *AU = NULL,
                AdditionalLiveness *AD = NULL);

private:
  void AddToLivenessValues(LivenessValues &set, IndexedVarDeclRef ref);
  void MergeLiveIn(LivenessValues &set, const WCFGBlock *block,
                   const WCFGBlock *succ);
  LivenessValues Transfer(LivenessValues out, LivenessValues def,
                          LivenessValues use);

  void DiscoverDefsAndUses(WCFGBlock *Block, WStmt *S, bool Outermost = false);
};

void LiveSSAVariablesImpl::Analysis(WCFG *cfg, AdditionalLiveness *AU,
                                    AdditionalLiveness *AD) {
  LiveIn.clear();
  LiveOut.clear();
  Def.clear();
  Use.clear();
  Phi.clear();

  assert(cfg && cfg->isSSA());
  CFGAnalysisContext Ctx(cfg);

  for (WCFG::const_iterator B = cfg->begin(), BEnd = cfg->end();
       B != BEnd; ++B) {
    Def[*B] = LivenessValues();
    Use[*B] = LivenessValues();
    Phi[*B] = LivenessValues();
  }
  AUses = AU;
  ADefs = AD;
  for (WCFG::const_iterator B = cfg->begin(), BEnd = cfg->end();
       B != BEnd; ++B) {
    WCFGBlock *Block = *B;
    for (WCFGBlock::reverse_iterator S = Block->rbegin(), SEnd = Block->rend();
         S != SEnd; ++S) {
      DiscoverDefsAndUses(Block, *S, true);
    }
  }

  CFGDataflowWorklist Worklist;
  CFGBlockBitVector EverAnalyzed;
  Worklist.enqueue(cfg->getExit());
  while (const WCFGBlock *Block = Worklist.dequeue()) {
    LivenessValues Lives;

    for (WCFGBlock::const_succ_iterator S = Block->real_succ_begin(),
                                        SEnd = Block->real_succ_end();
         S != SEnd; ++S) {
      assert(*S != NULL);
      MergeLiveIn(Lives, Block, *S);
    }

    if (EverAnalyzed[Block] && LiveOut[Block] == Lives) {
      continue;
    }

    EverAnalyzed[Block] = true;
    LiveOut[Block] = Lives;
    LiveIn[Block] = Transfer(LiveOut[Block], Def[Block], Use[Block]);
    Worklist.enqueuePredecessors(Block);
  }
}

void LiveSSAVariablesImpl::AddToLivenessValues(LivenessValues &set,
                                               IndexedVarDeclRef ref) {
  if (ref.isNull()) {
    // Do nothing
  } else if (ref.isSingleVar()) {
    if (ref->getIndex() > 0) {
      set = SetFactory.add(set, *ref);
    }
  } else {
    assert(ref.isCompound());
    for (unsigned Index = 0, NumSubVars = ref.getNumSubVars();
         Index != NumSubVars; ++Index) {
      if (ref[Index]->getIndex() > 0) {
        set = SetFactory.add(set, ref[Index]);
      }
    }
  }
}

void LiveSSAVariablesImpl::MergeLiveIn(LivenessValues &set,
                                       const WCFGBlock *block,
                                       const WCFGBlock *succ) {
  unsigned pred_index = succ->getPredIndex(block);

  for (LivenessValues::iterator I = LiveIn[succ].begin(),
                                E = LiveIn[succ].end();
       I != E; ++I) {
    const IndexedVarDecl *var = *I;
    if (Phi[succ].contains(var)) {
      assert(isa<WPhiFunction>(var->getDefinedStmt()));
      WPhiFunction *PF = static_cast<WPhiFunction*>(var->getDefinedStmt());
      set = SetFactory.add(set, PF->getIndexedArgDecl(pred_index));
    } else {
      set = SetFactory.add(set, var);
    }
  }
}

LivenessValues LiveSSAVariablesImpl::Transfer(LivenessValues out,
                                              LivenessValues def,
                                              LivenessValues use) {
  LivenessValues in = out;
  for (LivenessValues::iterator I = use.begin(), E = use.end();
       I != E; ++I) {
    in = SetFactory.add(in, *I);
  }
  for (LivenessValues::iterator I = def.begin(), E = def.end();
       I != E; ++I) {
    in = SetFactory.remove(in, *I);
  }
  return in;
}

void LiveSSAVariablesImpl::DiscoverDefsAndUses(WCFGBlock *Block, WStmt *S,
                                               bool Outermost) {
  if (!S)
    return;

  if (AUses) AddToLivenessValues(Use[Block], AUses->handleBlock(Block));
  if (ADefs) AddToLivenessValues(Def[Block], ADefs->handleBlock(Block));

#define CHECK_ADDITIONAL_LIVENESS(S) \
  if (AUses || ADefs) { \
    bool Halt = false; \
    if (AUses) AddToLivenessValues(Use[Block], AUses->handleStmt(S, Halt)); \
    if (ADefs) AddToLivenessValues(Def[Block], ADefs->handleStmt(S, Halt)); \
    if (Halt) return; \
  }

  if (WDeclStmt *DS = dyn_cast<WDeclStmt>(S)) {
    CHECK_ADDITIONAL_LIVENESS(S);
    if (DS->hasSingleInit()) {
      DiscoverDefsAndUses(Block, DS->getSingleInit());
    }
    AddToLivenessValues(Def[Block], DS->getIndexedDecl());

  } else if (WDeclRefExpr *DRE = dyn_cast<WDeclRefExpr>(S)) {
    CHECK_ADDITIONAL_LIVENESS(S);
    AddToLivenessValues(Use[Block], DRE->getIndexedUseDecl());
    AddToLivenessValues(Def[Block], DRE->getIndexedDefDecl());

  } else if (WCallExpr *CE = dyn_cast<WCallExpr>(S)) {
    if (Outermost) {
      CHECK_ADDITIONAL_LIVENESS(S);
      DiscoverDefsAndUses(Block, CE->getCallee());
      for (unsigned Index = 0, NumArgs = CE->getNumArgs();
           Index != NumArgs; ++Index) {
        DiscoverDefsAndUses(Block, CE->getArg(Index));
      }
    }

  } else if (WMemberExpr *ME = dyn_cast<WMemberExpr>(S)) {
    CHECK_ADDITIONAL_LIVENESS(S);
    DiscoverDefsAndUses(Block, ME->getBase());
    AddToLivenessValues(Use[Block], ME->getIndexedUseDecl());
    AddToLivenessValues(Def[Block], ME->getIndexedDefDecl());

  } else if (WBinaryOperator *BO = dyn_cast<WBinaryOperator>(S)) {
    if (BO->isLogicalOp() || BO->getOpcode() == BO_Comma) {
      // Do nothing
    } else {
      CHECK_ADDITIONAL_LIVENESS(S);
      DiscoverDefsAndUses(Block, BO->getLHS());
      DiscoverDefsAndUses(Block, BO->getRHS());
    }

  } else if (isa<WConditionalOperator>(S)) {
    // Do nothing

  } else if (WExtVectorElementExpr *VEE = dyn_cast<WExtVectorElementExpr>(S)) {
    CHECK_ADDITIONAL_LIVENESS(S);
    DiscoverDefsAndUses(Block, VEE->getBase());
    AddToLivenessValues(Use[Block], VEE->getIndexedUseDecl());
    AddToLivenessValues(Def[Block], VEE->getIndexedDefDecl());

  } else if (WPhiFunction *PF = dyn_cast<WPhiFunction>(S)) {
    CHECK_ADDITIONAL_LIVENESS(S);
    AddToLivenessValues(Phi[Block], PF->getIndexedLHSDecl());

  } else {
    CHECK_ADDITIONAL_LIVENESS(S);
    for (WStmt::child_iterator I = S->child_begin(), E = S->child_end();
         I != E; ++I) {
      DiscoverDefsAndUses(Block, *I);
    }
  }
}

} // anonymous namespace

LiveSSAVariables::~LiveSSAVariables() {
  delete (LiveSSAVariablesImpl*)Impl;
}

LiveSSAVariables *LiveSSAVariables::Create(WCFG *cfg, AdditionalLiveness *AUses,
                                           AdditionalLiveness *ADefs) {
  LiveSSAVariables *LV = new LiveSSAVariables(cfg);
  LiveSSAVariablesImpl *Impl = new LiveSSAVariablesImpl(LV->LiveIn,
                                                        LV->LiveOut);
  Impl->Analysis(cfg, AUses, ADefs);
  LV->Impl = Impl;
  return LV;
}

void LiveSSAVariables::dump(raw_ostream &OS) const {
  for (LivenessMapTy::const_iterator BI = LiveIn.begin(), BE = LiveIn.end();
       BI != BE; ++BI) {
    const WCFGBlock *block = BI->first;
    LivenessValues live = BI->second;
    assert(block != NULL);
    OS << "LiveIn[B" << block->getBlockID() << "] = ";
    OS << '{';
    for (LivenessValues::iterator VI = live.begin(), VE = live.end();
         VI != VE; ++VI) {
      if (VI != live.begin()) OS << ", ";
      (*VI)->printName(OS);
    }
    OS << "}\n";
  }

  for (LivenessMapTy::const_iterator BI = LiveOut.begin(), BE = LiveOut.end();
       BI != BE; ++BI) {
    const WCFGBlock *block = BI->first;
    LivenessValues live = BI->second;
    assert(block != NULL);
    OS << "LiveOut[B" << block->getBlockID() << "] = ";
    OS << '{';
    for (LivenessValues::iterator VI = live.begin(), VE = live.end();
         VI != VE; ++VI) {
      if (VI != live.begin()) OS << ", ";
      (*VI)->printName(OS);
    }
    OS << "}\n";
  }

  OS << '\n';
  OS.flush();
}

} // namespace snu

} // namespace clang
