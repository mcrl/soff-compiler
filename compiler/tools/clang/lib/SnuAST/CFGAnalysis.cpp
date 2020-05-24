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

#include "clang/SnuAST/CFGAnalysis.h"
#include "clang/SnuAST/WCFG.h"
#include "llvm/ADT/BitVector.h"

namespace clang {

namespace snu {

namespace {

const WCFG *currentCFG = NULL;

} // anonymous namespace

CFGAnalysisContext::CFGAnalysisContext(const WCFG *cfg)
  : previousCFG(currentCFG) {
  currentCFG = cfg;
}

CFGAnalysisContext::~CFGAnalysisContext() {
  currentCFG = previousCFG;
}

unsigned CFGAnalysisContext::getNumBlocks() {
  return currentCFG ? currentCFG->size() : 0;
}

void CFGDataflowWorklist::enqueue(const WCFGBlock *block) {
  if (block && !enqueued[block]) {
    enqueued[block] = true;
    worklist.push_back(block);
  }
}

void CFGDataflowWorklist::enqueueSuccessors(const WCFGBlock *block) {
  for (WCFGBlock::const_succ_iterator I = block->succ_begin(),
                                      E = block->succ_end();
       I != E; ++I) {
    enqueue(*I);
  }
}

void CFGDataflowWorklist::enqueuePredecessors(const WCFGBlock *block) {
  for (WCFGBlock::const_pred_iterator I = block->pred_begin(),
                                      E = block->pred_end();
       I != E; ++I) {
    enqueue(*I);
  }
}

const WCFGBlock *CFGDataflowWorklist::dequeue() {
  if (worklist.empty())
    return 0;

  const WCFGBlock *block = worklist.pop_back_val();
  enqueued[block] = false;
  return block;
}

const WCFGBlock *CFGDataflowWorklist::dequeueForever() {
  if (worklist.empty())
    return 0;

  const WCFGBlock *block = worklist.pop_back_val();
  return block;
}

} // namespace snu

} // namespace clang
