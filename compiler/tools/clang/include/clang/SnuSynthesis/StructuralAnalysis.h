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

#ifndef LLVM_CLANG_SNU_SYNTHESIS_STRUCTURALANALYSIS_H
#define LLVM_CLANG_SNU_SYNTHESIS_STRUCTURALANALYSIS_H

#include "clang/Basic/LLVM.h"
#include "clang/SnuSynthesis/ControlFlowConstraint.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/SmallVector.h"
#include <utility>

namespace clang {

namespace snu {

class ControlDataflowGraph;
class DataflowGraph;
class IndexedVarDecl;

class ControlTreeNode {
public:
  enum ControlTreeNodeClass {
    CTBasicBlockClass = 1,
    CTSequentialClass,
    CTIfThenClass,
    CTIfThenElseClass,
    CTAcyclicClass,
    CTSelfLoopClass,
    CTWhileLoopClass,
    CTNaturalLoopClass
  };

private:
  ControlTreeNodeClass Class;
  ControlFlowConstraint CFConstraint;

protected:
  SmallVector<DataflowGraph*, 4> Succs;

  typedef SmallVector<IndexedVarDecl*, 64> LiveVarListTy;
  LiveVarListTy LiveIns;
  SmallVector<LiveVarListTy, 4> LiveOuts;

  bool IsProgramExit;
  bool HasBarrier;
  bool HasCondBarrier;

  explicit ControlTreeNode(ControlTreeNodeClass C, unsigned NumSuccs = 0);

public:
  ControlTreeNodeClass getClass() const { return Class; }
  bool isLoop() const {
    return Class >= CTSelfLoopClass && Class <= CTNaturalLoopClass;
  }

  typedef ControlTreeNode** child_iterator;
  typedef ControlTreeNode* const* const_child_iterator;

  typedef std::pair<child_iterator, child_iterator> child_range;
  typedef std::pair<const_child_iterator, const_child_iterator>
      const_child_range;

  child_range children();
  const_child_range children() const {
    child_range range = const_cast<ControlTreeNode*>(this)->children();
    return const_child_range(range.first, range.second);
  }

  child_iterator child_begin() { return children().first; }
  child_iterator child_end() { return children().second; }

  const_child_iterator child_begin() const { return children().first; }
  const_child_iterator child_end() const { return children().second; }

  typedef SmallVectorImpl<DataflowGraph*>::iterator succ_iterator;
  typedef SmallVectorImpl<DataflowGraph*>::const_iterator const_succ_iterator;
  succ_iterator succ_begin() { return Succs.begin(); }
  succ_iterator succ_end() { return Succs.end(); }
  const_succ_iterator succ_begin() const { return Succs.begin(); }
  const_succ_iterator succ_end() const { return Succs.end(); }

  typedef LiveVarListTy::iterator live_var_iterator;
  typedef LiveVarListTy::const_iterator const_live_var_iterator;
  live_var_iterator live_in_begin() { return LiveIns.begin(); }
  live_var_iterator live_in_end() { return LiveIns.end(); }
  const_live_var_iterator live_in_begin() const { return LiveIns.begin(); }
  const_live_var_iterator live_in_end() const { return LiveIns.end(); }

  live_var_iterator live_out_begin(unsigned Index) {
    assert(Index < LiveOuts.size());
    return LiveOuts[Index].begin();
  }
  live_var_iterator live_out_end(unsigned Index) {
    assert(Index < LiveOuts.size());
    return LiveOuts[Index].end();
  }
  const_live_var_iterator live_out_begin(unsigned Index) const {
    assert(Index < LiveOuts.size());
    return LiveOuts[Index].begin();
  }
  const_live_var_iterator live_out_end(unsigned Index) const {
    assert(Index < LiveOuts.size());
    return LiveOuts[Index].end();
  }

  unsigned getNumSuccs() const { return Succs.size(); }
  DataflowGraph *getSucc(unsigned Index) const {
    assert(Index < Succs.size() && Succs[Index] != NULL);
    return Succs[Index];
  }

  void SetSucc(unsigned Index, DataflowGraph *S) {
    assert(Index < Succs.size() && Succs[Index] == NULL);
    Succs[Index] = S;
  }

  template<typename iter>
  void SetLiveIns(iter begin, iter end) {
    assert(LiveIns.empty());
    LiveIns.append(begin, end);
  }

  template<typename iter>
  void SetLiveOuts(unsigned Index, iter begin, iter end) {
    assert(Index < LiveOuts.size() && LiveOuts[Index].empty());
    LiveOuts[Index].append(begin, end);
  }

  template<typename iter>
  ControlTreeNode *WithLiveIns(iter begin, iter end) {
    SetLiveIns(begin, end);
    return this;
  }

  template<typename iter>
  ControlTreeNode *WithLiveOuts(unsigned Index, iter begin, iter end) {
    SetLiveOuts(Index, begin, end);
    return this;
  }

  bool isProgramExit() const { return IsProgramExit; }
  bool hasBarrier() const { return HasBarrier; }
  bool hasCondBarrier() const { return HasCondBarrier; }
  bool contains(DataflowGraph *dfg) const;
  DataflowGraph *getEntry() const;

  const ControlFlowConstraint &getControlFlowConstraint() const {
    return CFConstraint;
  }
  void AddControlFlowConstraint(const ControlFlowConstraint &C);

  void print(raw_ostream &OS) const;

  static ControlTreeNode *Build(ControlDataflowGraph *cdfg);

private:
  static ControlTreeNode *Build(ControlDataflowGraph *cdfg,
                                DataflowGraph *Entry);
};

class CTBasicBlock : public ControlTreeNode {
  DataflowGraph *dfg;

public:
  CTBasicBlock(ControlDataflowGraph *cdfg, DataflowGraph *dfg_);

  DataflowGraph *getDFG() const { return dfg; }

  child_range children() { return child_range(NULL, NULL); }

  static bool classof(const ControlTreeNode *T) {
    return T->getClass() == ControlTreeNode::CTBasicBlockClass;
  }
};

class CTSequential : public ControlTreeNode {
  enum { FIRST, SECOND, END_CHILDREN };
  ControlTreeNode *Children[END_CHILDREN];

public:
  CTSequential(ControlTreeNode *First, ControlTreeNode *Second);

  ControlTreeNode *getFirst() const { return Children[FIRST]; }
  ControlTreeNode *getSecond() const { return Children[SECOND]; }

  child_range children() {
    return child_range(Children, Children + END_CHILDREN);
  }

  static bool classof(const ControlTreeNode *T) {
    return T->getClass() == ControlTreeNode::CTSequentialClass;
  }
};

class CTIfThen : public ControlTreeNode {
  enum { IF, THEN, END_CHILDREN };
  ControlTreeNode *Children[END_CHILDREN];

public:
  CTIfThen(ControlTreeNode *If, ControlTreeNode *Then);

  ControlTreeNode *getIf() const { return Children[IF]; }
  ControlTreeNode *getThen() const { return Children[THEN]; }

  child_range children() {
    return child_range(Children, Children + END_CHILDREN);
  }

  static bool classof(const ControlTreeNode *T) {
    return T->getClass() == ControlTreeNode::CTIfThenClass;
  }
};

class CTIfThenElse : public ControlTreeNode {
  enum { IF, THEN, ELSE, END_CHILDREN };
  ControlTreeNode *Children[END_CHILDREN];

public:
  CTIfThenElse(ControlTreeNode *If, ControlTreeNode *Then,
               ControlTreeNode *Else);

  ControlTreeNode *getIf() const { return Children[IF]; }
  ControlTreeNode *getThen() const { return Children[THEN]; }
  ControlTreeNode *getElse() const { return Children[ELSE]; }

  child_range children() {
    return child_range(Children, Children + END_CHILDREN);
  }

  static bool classof(const ControlTreeNode *T) {
    return T->getClass() == ControlTreeNode::CTIfThenElseClass;
  }
};

class CTAcyclic : public ControlTreeNode {
  SmallVector<ControlTreeNode*, 16> Nodes;
  SmallVector<size_t, 64> Edges;
  SmallVector<size_t, 16> EdgePtrs;

public:
  CTAcyclic();

  // NI: node index; EI: edge index;
  unsigned getNumNodes() const { return Nodes.size(); }
  ControlTreeNode *getNode(unsigned NI) const {
    assert(NI < Nodes.size());
    return Nodes[NI];
  }
  unsigned getNumEdges(unsigned NI) const;
  bool isInternalEdge(unsigned NI, unsigned EI) const;
  ControlTreeNode *getInternalEdge(unsigned NI, unsigned EI) const;
  bool isExternalEdge(unsigned NI, unsigned EI) const;
  unsigned getExternalEdge(unsigned NI, unsigned EI) const;

  ControlTreeNode *getEntry() const { return Nodes[0]; }

  void AddNode(ControlTreeNode *Node);
  void AddEdge(ControlTreeNode *From, ControlTreeNode *To);
  void AddExit(ControlTreeNode *From, DataflowGraph *To);

  child_range children() {
    return child_range(Nodes.begin(), Nodes.end());
  }

  static bool classof(const ControlTreeNode *T) {
    return T->getClass() == ControlTreeNode::CTAcyclicClass;
  }
};

class CTSelfLoop : public ControlTreeNode {
  ControlTreeNode *Body;
  unsigned LoopBackIndex;
  bool FixedBound;

public:
  CTSelfLoop(ControlTreeNode *body, unsigned loopback_index,
             bool fixed_bound = false);

  ControlTreeNode *getBody() const { return Body; }
  unsigned getLoopBackIndex() const { return LoopBackIndex; }
  unsigned getExitIndex() const { return 1 - LoopBackIndex; }

  bool hasFixedBound() const { return FixedBound; }

  child_range children() { return child_range(&Body, &Body + 1); }

  static bool classof(const ControlTreeNode *T) {
    return T->getClass() == ControlTreeNode::CTSelfLoopClass;
  }
};

class CTWhileLoop : public ControlTreeNode {
  enum { COND, BODY, END_CHILDREN };
  ControlTreeNode *Children[END_CHILDREN];
  unsigned LoopBackIndex;
  bool FixedBound;

public:
  CTWhileLoop(ControlTreeNode *Cond, unsigned loopback_index,
              ControlTreeNode *Body, bool fixed_bound = false);

  ControlTreeNode *getCond() const { return Children[COND]; }
  unsigned getLoopBackIndex() const { return LoopBackIndex; }
  unsigned getExitIndex() const { return 1 - LoopBackIndex; }
  ControlTreeNode *getBody() const { return Children[BODY]; }

  bool hasFixedBound() const { return FixedBound; }

  child_range children() {
    return child_range(Children, Children + END_CHILDREN);
  }

  static bool classof(const ControlTreeNode *T) {
    return T->getClass() == ControlTreeNode::CTWhileLoopClass;
  }
};

class CTNaturalLoop : public ControlTreeNode {
  ControlTreeNode *Body;
  unsigned LoopBackIndex;

public:
  CTNaturalLoop(ControlTreeNode *body, unsigned loopback_index);

  ControlTreeNode *getBody() const { return Body; }
  unsigned getLoopBackIndex() const { return LoopBackIndex; }

  child_range children() { return child_range(&Body, &Body + 1); }

  static bool classof(const ControlTreeNode *T) {
    return T->getClass() == ControlTreeNode::CTNaturalLoopClass;
  }
};

} // namespace snu

} // namespace clang

#endif // LLVM_CLANG_SNU_SYNTHESIS_STRUCTURALANALYSIS_H
