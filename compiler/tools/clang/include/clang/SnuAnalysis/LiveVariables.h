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

#ifndef LLVM_CLANG_SNU_ANALYSIS_LIVEVARIABLES_H
#define LLVM_CLANG_SNU_ANALYSIS_LIVEVARIABLES_H

#include "clang/Basic/LLVM.h"
#include "clang/SnuAST/WAST.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/ImmutableSet.h"

namespace clang {

namespace snu {

class WCFG;
class WCFGBlock;

class AdditionalLiveness {
public:
  virtual ~AdditionalLiveness() {}
  virtual IndexedVarDeclRef handleStmt(WStmt *S, bool &Halt) = 0;
  virtual IndexedVarDeclRef handleBlock(WCFGBlock *B) = 0;
};

class LiveSSAVariables {
public:
  class LivenessValues : public llvm::ImmutableSet<const IndexedVarDecl*> {
  public:
    typedef llvm::ImmutableSet<const IndexedVarDecl*>::Factory Factory;

    LivenessValues()
      : llvm::ImmutableSet<const IndexedVarDecl*>(NULL) {}
    LivenessValues(const llvm::ImmutableSet<const IndexedVarDecl*> &X)
      : llvm::ImmutableSet<const IndexedVarDecl*>(X) {}
    LivenessValues(const LivenessValues &X)
      : llvm::ImmutableSet<const IndexedVarDecl*>(X) {}
  };

  typedef llvm::DenseMap<const WCFGBlock*, LivenessValues> LivenessMapTy;

private:
  WCFG *Program;
  LivenessMapTy LiveIn;
  LivenessMapTy LiveOut;
  void *Impl;

  LiveSSAVariables(WCFG *cfg) : Program(cfg), Impl(NULL) {}

public:
  ~LiveSSAVariables();

  WCFG *getProgram() const { return Program; }

  bool isLiveIn(const WCFGBlock *B, const IndexedVarDecl *V) const {
    return LiveIn.lookup(B).contains(V);
  }
  bool isLiveOut(const WCFGBlock *B, const IndexedVarDecl *V) const {
    return LiveOut.lookup(B).contains(V);
  }

  typedef LivenessValues::iterator iterator;

  iterator live_in_begin(const WCFGBlock *B) const {
    return LiveIn.lookup(B).begin();
  }
  iterator live_in_end(const WCFGBlock* B) const {
    return LiveIn.lookup(B).end();
  }
  iterator live_out_begin(const WCFGBlock *B) const {
    return LiveOut.lookup(B).begin();
  }
  iterator live_out_end(const WCFGBlock *B) const {
    return LiveOut.lookup(B).end();
  }

  void dump(raw_ostream &OS) const;

  static LiveSSAVariables *Create(WCFG *cfg, AdditionalLiveness *AUses = NULL,
                                  AdditionalLiveness *ADefs = NULL);
};

} // namespace snu

} // namespace clang

#endif // LLVM_CLANG_SNU_ANALYSIS_LIVEVARIABLES_H
