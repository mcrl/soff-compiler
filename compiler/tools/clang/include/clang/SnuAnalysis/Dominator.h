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

#ifndef LLVM_CLANG_SNU_ANALYSIS_DOMINATOR_H
#define LLVM_CLANG_SNU_ANALYSIS_DOMINATOR_H

#include "clang/Basic/LLVM.h"
#include "clang/SnuAST/WCFG.h"
#include "llvm/Analysis/DominatorInternals.h"
#include "llvm/Analysis/Dominators.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/SmallSet.h"

namespace clang {

namespace snu {

typedef llvm::DominatorTreeBase<WCFGBlock> WCFGDominatorTree;
typedef llvm::DomTreeNodeBase<WCFGBlock> WCFGDomTreeNode;

class WCFGDominanceFrontier {
  typedef llvm::SmallSet<WCFGBlock*, 8> DomSetTy;
  typedef llvm::DenseMap<WCFGBlock*, DomSetTy> DomSetMapTy;

  DomSetMapTy Frontiers;

public:
  void recalculate(WCFGDominatorTree &DT);

  unsigned num_frontiers(WCFGBlock *B) const {
    return Frontiers.lookup(B).size();
  }

  typedef DomSetTy::iterator frontier_iterator;
  typedef DomSetTy::const_iterator const_frontier_iterator;

  frontier_iterator frontier_begin(WCFGBlock *B) {
    return Frontiers[B].begin();
  }
  frontier_iterator frontier_end(WCFGBlock *B) {
    return Frontiers[B].end();
  }
  const_frontier_iterator frontier_begin(WCFGBlock *B) const {
    return Frontiers.find(B)->second.begin();
  }
  const_frontier_iterator frontier_end(WCFGBlock *B) const {
    return Frontiers.find(B)->second.end();
  }

private:
  void calculateInternal(WCFGDominatorTree &DT, const WCFGDomTreeNode *Node);
};

} // namespace snu

} // namespace clang

#endif // LLVM_CLANG_SNU_ANALYSIS_DOMINATOR_H
