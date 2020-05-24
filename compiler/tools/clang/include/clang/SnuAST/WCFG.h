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

#ifndef LLVM_CLANG_SNU_AST_WCFG_H
#define LLVM_CLANG_SNU_AST_WCFG_H

#include "clang/AST/Stmt.h"
#include "clang/Analysis/CFG.h"
#include "clang/Analysis/Support/BumpVector.h"
#include "clang/Basic/LLVM.h"
#include "clang/Basic/LangOptions.h"
#include "clang/SnuAST/WAST.h"
#include "llvm/ADT/GraphTraits.h"
#include "llvm/Support/Allocator.h"

namespace clang {

namespace snu {

class WCFG;

class WCFGBlock {
  WCFG *Parent;
  unsigned BlockID;
  BumpVectorContext &BVCtx;

  typedef BumpVector<WStmt*> ElementListTy;
  ElementListTy Elements;

  WStmt *Label;
  WStmt *Terminator;
  WStmt *LoopTarget;

  typedef BumpVector<WCFGBlock*> AdjacentBlockListTy;
  AdjacentBlockListTy Preds;
  AdjacentBlockListTy Succs;
  AdjacentBlockListTy RealSuccs; // Non-NULL successors

  unsigned NumBarriers;

public:
  WCFGBlock(WCFG *parent, unsigned id);
  ~WCFGBlock() {}

  typedef ElementListTy::iterator iterator;
  typedef ElementListTy::const_iterator const_iterator;
  typedef ElementListTy::reverse_iterator reverse_iterator;
  typedef ElementListTy::const_reverse_iterator const_reverse_iterator;
  typedef ElementListTy::const_reference const_reference;

  const_reference front() const { return Elements.front(); }
  const_reference back() const { return Elements.back(); }

  iterator begin() { return Elements.begin(); }
  iterator end() { return Elements.end(); }
  const_iterator begin() const { return Elements.begin(); }
  const_iterator end() const { return Elements.end(); }
  reverse_iterator rbegin() { return Elements.rbegin(); }
  reverse_iterator rend() { return Elements.rend(); }
  const_reverse_iterator rbegin() const { return Elements.rbegin(); }
  const_reverse_iterator rend() const { return Elements.rend(); }

  size_t size() const { return Elements.size(); }
  bool empty() const { return Elements.empty(); }

  typedef AdjacentBlockListTy::iterator pred_iterator;
  typedef AdjacentBlockListTy::const_iterator const_pred_iterator;
  typedef AdjacentBlockListTy::reverse_iterator pred_reverse_iterator;
  typedef AdjacentBlockListTy::const_reverse_iterator const_pred_reverse_iterator;

  typedef AdjacentBlockListTy::iterator succ_iterator;
  typedef AdjacentBlockListTy::const_iterator const_succ_iterator;
  typedef AdjacentBlockListTy::reverse_iterator succ_reverse_iterator;
  typedef AdjacentBlockListTy::const_reverse_iterator const_succ_reverse_iterator;

  pred_iterator pred_begin() { return Preds.begin(); }
  pred_iterator pred_end() { return Preds.end(); }
  const_pred_iterator pred_begin() const { return Preds.begin(); }
  const_pred_iterator pred_end() const { return Preds.end(); }
  pred_reverse_iterator pred_rbegin() { return Preds.rbegin(); }
  pred_reverse_iterator pred_rend() { return Preds.rend(); }
  const_pred_reverse_iterator pred_rbegin() const { return Preds.rbegin(); }
  const_pred_reverse_iterator pred_rend() const { return Preds.rend(); }

  succ_iterator succ_begin() { return Succs.begin(); }
  succ_iterator succ_end() { return Succs.end(); }
  const_succ_iterator succ_begin() const { return Succs.begin(); }
  const_succ_iterator succ_end() const { return Succs.end(); }
  succ_reverse_iterator succ_rbegin() { return Succs.rbegin(); }
  succ_reverse_iterator succ_rend() { return Succs.rend(); }
  const_succ_reverse_iterator succ_rbegin() const { return Succs.rbegin(); }
  const_succ_reverse_iterator succ_rend() const { return Succs.rend(); }

  succ_iterator real_succ_begin() { return RealSuccs.begin(); }
  succ_iterator real_succ_end() { return RealSuccs.end(); }
  const_succ_iterator real_succ_begin() const { return RealSuccs.begin(); }
  const_succ_iterator real_succ_end() const { return RealSuccs.end(); }
  succ_reverse_iterator real_succ_rbegin() { return RealSuccs.rbegin(); }
  succ_reverse_iterator real_succ_rend() { return RealSuccs.rend(); }
  const_succ_reverse_iterator real_succ_rbegin() const {
    return RealSuccs.rbegin();
  }
  const_succ_reverse_iterator real_succ_rend() const {
    return RealSuccs.rend();
  }

  unsigned pred_size() const { return Preds.size(); }
  bool pred_empty() const { return Preds.empty(); }

  unsigned succ_size() const { return Succs.size(); }
  bool succ_empty() const { return Succs.empty(); }

  unsigned real_succ_size() const { return RealSuccs.size(); }
  bool real_succ_empty() const { return RealSuccs.empty(); }

  void setLabel(WStmt *S) { Label = S; }
  void setTerminator(WStmt *S) { Terminator = S; }
  void setLoopTarget(WStmt *S) { LoopTarget = S; }

  WStmt *getLabel() const { return Label; }
  WStmt *getTerminator() const { return Terminator; }
  WExpr *getTerminatorCondition() const;
  WStmt *getLoopTarget() const { return LoopTarget; }

  WCFG *getParent() const { return Parent; }
  unsigned getBlockID() const { return BlockID; }

  unsigned getPredIndex(const WCFGBlock *Block) const;
  unsigned getSuccIndex(const WCFGBlock *Block) const;

  bool hasBarrier() const { return NumBarriers > 0; }

  iterator findStmt(WStmt *S);
  const_iterator findStmt(WStmt *S) const;
  int compareLocation(WStmt *LHS, WStmt *RHS) const;

  void appendStmt(WStmt *S);
  void insertStmtAtFront(WStmt *S);
  void insertStmtAt(WStmt *S, iterator I);
  void replaceStmt(WStmt *From, WStmt *To);

  void addPredecessor(WCFGBlock *Block) {
    Preds.push_back(Block, BVCtx);
  }

  void addSuccessor(WCFGBlock *Block) {
    Succs.push_back(Block, BVCtx);
    if (Block != NULL) {
      RealSuccs.push_back(Block, BVCtx);
    }
  }

  llvm::BumpPtrAllocator &getAllocator() const {
    return BVCtx.getAllocator();
  }

  bool hasPhiFunction(const WVarDecl *Var) const;
  bool hasPhiFunction(const IndexedVarDecl *Var) const;
  WPhiFunction *getPhiFunction(const WVarDecl *Var) const;
  WPhiFunction *getPhiFunction(const IndexedVarDecl *Var) const;

  void print(raw_ostream &OS, const LangOptions &LO) const;
};

class WCFG {
  CFG *Original;

  WCFGBlock *Entry;
  WCFGBlock *Exit;
  unsigned NumBlockIDs;

  mutable BumpVectorContext BVCtx;
  typedef BumpVector<WCFGBlock*> WCFGBlockListTy;
  WCFGBlockListTy Blocks;

  bool IsSSA;
  typedef BumpVector<IndexedVarDecl*> IndexedVarDeclListTy;
  IndexedVarDeclListTy SSAVars;

  WCFG(CFG *cfg)
    : Original(cfg), Entry(NULL), Exit(NULL), NumBlockIDs(0),
      Blocks(BVCtx, cfg->size()), IsSSA(false), SSAVars(BVCtx, 128) {}

public:
  typedef WCFGBlockListTy::iterator iterator;
  typedef WCFGBlockListTy::const_iterator const_iterator;
  typedef WCFGBlockListTy::reverse_iterator reverse_iterator;
  typedef WCFGBlockListTy::const_reverse_iterator const_reverse_iterator;

  const WCFGBlock &front() const { return *Blocks.front(); }
  const WCFGBlock &back() const { return *Blocks.back(); }
  WCFGBlock &front() { return *Blocks.front(); }
  WCFGBlock &back() { return *Blocks.back(); }

  iterator begin() { return Blocks.begin(); }
  iterator end() { return Blocks.end(); }
  const_iterator begin() const { return Blocks.begin(); }
  const_iterator end() const { return Blocks.end(); }
  reverse_iterator rbegin() { return Blocks.rbegin(); }
  reverse_iterator rend() { return Blocks.rend(); }
  const_reverse_iterator rbegin() const { return Blocks.rbegin(); }
  const_reverse_iterator rend() const { return Blocks.rend(); }

  template<class ImplTy>
  class graph_iterator_base {
    ImplTy I;

  public:
    typedef const WCFGBlock value_type;
    typedef value_type& reference;
    typedef value_type* pointer;

    graph_iterator_base(const ImplTy &i) : I(i) {}

    bool operator==(const graph_iterator_base<ImplTy> &X) const {
      return I == X.I;
    }
    bool operator!=(const graph_iterator_base<ImplTy> &X) const {
      return I != X.I;
    }

    reference operator*() const { return **I; }
    pointer operator->() const { return *I; }
    operator WCFGBlock*() { return *I; }

    graph_iterator_base<ImplTy> &operator++() { ++I; return *this; }
    graph_iterator_base<ImplTy> &operator--() { --I; return *this; }
  };

  typedef graph_iterator_base<WCFGBlockListTy::iterator> graph_iterator;
  typedef graph_iterator_base<WCFGBlockListTy::const_iterator> const_graph_iterator;

  graph_iterator nodes_begin() { return graph_iterator(Blocks.begin()); }
  graph_iterator nodes_end() { return graph_iterator(Blocks.end()); }
  const_graph_iterator nodes_begin() const {
    return const_graph_iterator(Blocks.begin());
  }
  const_graph_iterator nodes_end() const {
    return const_graph_iterator(Blocks.end());
  }

  unsigned size() const { return NumBlockIDs; }

  WCFGBlock *getEntry() const { return Entry; }
  WCFGBlock *getExit() const { return Exit; }
  WCFGBlock *getBlock(unsigned ID) const {
    assert(Blocks[ID]->getBlockID() == ID);
    return Blocks[ID];
  }

  WCFGBlock *findStmt(WStmt *S);
  const WCFGBlock *findStmt(WStmt *S) const;

  bool isPotentiallyReachable(const WCFGBlock *From, const WCFGBlock *To) const;
  bool isPotentiallyReachable(WStmt *From, WStmt *To) const;

  WCFGBlock *addBlock();
  void markAsEntry(WCFGBlock *B);
  void markAsExit(WCFGBlock *B);
  void replaceStmt(WStmt *From, WStmt *To);

  bool isSSA() const { return IsSSA; }
  void setSSA() { IsSSA = true; }
  void addSSAVar(IndexedVarDecl *Var) {
    SSAVars.push_back(Var, BVCtx);
  }

  void MakeSSAForm();
  void IncrementSSAVariable(WVarDecl *Var);

  typedef IndexedVarDeclListTy::iterator ssa_var_iterator;
  typedef IndexedVarDeclListTy::const_iterator const_ssa_var_iterator;
  typedef IndexedVarDeclListTy::reverse_iterator ssa_var_reverse_iterator;
  typedef IndexedVarDeclListTy::const_reverse_iterator
      const_ssa_var_reverse_iterator;

  ssa_var_iterator ssa_var_begin() { return SSAVars.begin(); }
  ssa_var_iterator ssa_var_end() { return SSAVars.end(); }
  const_ssa_var_iterator ssa_var_begin() const { return SSAVars.begin(); }
  const_ssa_var_iterator ssa_var_end() const { return SSAVars.end(); }
  ssa_var_reverse_iterator ssa_var_rbegin() { return SSAVars.rbegin(); }
  ssa_var_reverse_iterator ssa_var_rend() { return SSAVars.rend(); }
  const_ssa_var_reverse_iterator ssa_var_rbegin() const { return SSAVars.rbegin(); }
  const_ssa_var_reverse_iterator ssa_var_rend() const { return SSAVars.rend(); }

  bool hasBarrier() const;

  llvm::BumpPtrAllocator &getAllocator() const {
    return BVCtx.getAllocator();
  }
  BumpVectorContext &getBumpVectorContext() const {
    return BVCtx;
  }

  void print(raw_ostream &OS, const LangOptions &LO) const;

  static WCFG *WrapClangCFG(WDeclContext &DeclCtx, WStmt *AST, CFG *cfg);
};

template<typename ImplClass, typename RetTy=void>
class WCFGBlockVisitor {
public:
#define DISPATCH(type) \
  return static_cast<ImplClass*>(this)->Visit ## type(static_cast<W##type*>(S))

  RetTy VisitStmt(WStmt *S, bool Outermost = false) {
    if (isa<WCompoundAssignOperator>(S)) {
      DISPATCH(CompoundAssignOperator);
    }

    if (S->isClangStmt()) {
      switch (S->getStmtClass()) {
#define STMT(type) \
      case Stmt::type##Class: DISPATCH(type);
      STMT(DeclStmt)
      STMT(ReturnStmt)
      STMT(DeclRefExpr)
      STMT(PredefinedExpr)
      STMT(IntegerLiteral)
      STMT(CharacterLiteral)
      STMT(FloatingLiteral)
      STMT(StringLiteral)
      STMT(ParenExpr)
      STMT(UnaryOperator)
      STMT(UnaryExprOrTypeTraitExpr)
      STMT(ArraySubscriptExpr)
      case Stmt::CallExprClass: {
        if (Outermost) {
          DISPATCH(CallExpr);
        } else {
          WCallExpr *CE = static_cast<WCallExpr*>(S);
          return static_cast<ImplClass*>(this)->VisitVirtualCallExpr(CE);
        }
      }
      STMT(MemberExpr)
      STMT(CompoundLiteralExpr)
      STMT(ImplicitCastExpr)
      STMT(CStyleCastExpr)
      case Stmt::BinaryOperatorClass: {
        WBinaryOperator *BO = static_cast<WBinaryOperator*>(S);
        if (BO->isLogicalOp() || BO->getOpcode() == BO_Comma) {
          return static_cast<ImplClass*>(this)->VisitVirtualBinaryOperator(BO);
        } else {
          DISPATCH(BinaryOperator);
        }
      }
      case Stmt::ConditionalOperatorClass: {
        WConditionalOperator *CO = static_cast<WConditionalOperator*>(S);
        return static_cast<ImplClass*>(this)->
                 VisitVirtualConditionalOperator(CO);
      }
      STMT(InitListExpr)
      STMT(DesignatedInitExpr)
      STMT(ImplicitValueInitExpr)
      STMT(ParenListExpr)
      STMT(ExtVectorElementExpr)
      STMT(CXXBoolLiteralExpr)
      STMT(BlockExpr)
      STMT(AsTypeExpr)
#undef STMT
      default:
        llvm_unreachable("invalid statement class");
      }
    } else {
      switch (S->getWStmtClass()) {
#define WSTMT(type) \
      case WStmt::type##Class: DISPATCH(type);
      EXTENDED_WSTMTS()
#undef WSTMT
      default:
        llvm_unreachable("invalid statement class");
      }
    }
  }

#undef DISPATCH
};

} // namespace snu

} // namespace clang

inline void *operator new(size_t Bytes, const clang::snu::WCFG &G,
                          size_t Alignment = 8) {
  return G.getAllocator().Allocate(Bytes, Alignment);
}

inline void operator delete(void *Ptr, const clang::snu::WCFG &G, size_t) {
  G.getAllocator().Deallocate(Ptr);
}

inline void *operator new[](size_t Bytes, const clang::snu::WCFG &G,
                            size_t Alignment = 8) {
  return G.getAllocator().Allocate(Bytes, Alignment);
}

inline void operator delete[](void *Ptr, const clang::snu::WCFG &G, size_t) {
  G.getAllocator().Deallocate(Ptr);
}


// GraphTraits specializations for WCFG basic block graphs

namespace llvm {

template <> struct GraphTraits< ::clang::snu::WCFGBlock *> {
  typedef ::clang::snu::WCFGBlock NodeType;
  typedef ::clang::snu::WCFGBlock::succ_iterator ChildIteratorType;

  static NodeType* getEntryNode(::clang::snu::WCFGBlock *BB) {
   return BB;
  }

  static inline ChildIteratorType child_begin(NodeType *N) {
    return N->real_succ_begin();
  }
  static inline ChildIteratorType child_end(NodeType *N) {
    return N->real_succ_end();
  }
};

template <> struct GraphTraits<const ::clang::snu::WCFGBlock *> {
  typedef const ::clang::snu::WCFGBlock NodeType;
  typedef ::clang::snu::WCFGBlock::const_succ_iterator ChildIteratorType;

  static NodeType* getEntryNode(const clang::snu::WCFGBlock *BB) {
    return BB;
  }

  static inline ChildIteratorType child_begin(NodeType *N) {
    return N->real_succ_begin();
  }
  static inline ChildIteratorType child_end(NodeType *N) {
    return N->real_succ_end();
  }
};

template <> struct GraphTraits<Inverse< ::clang::snu::WCFGBlock*> > {
  typedef ::clang::snu::WCFGBlock NodeType;
  typedef ::clang::snu::WCFGBlock::const_pred_iterator ChildIteratorType;

  static NodeType *getEntryNode(Inverse< ::clang::snu::WCFGBlock*> G) {
    return G.Graph;
  }

  static inline ChildIteratorType child_begin(NodeType *N) {
    return N->pred_begin();
  }
  static inline ChildIteratorType child_end(NodeType *N) {
    return N->pred_end();
  }
};

template <> struct GraphTraits<Inverse<const ::clang::snu::WCFGBlock*> > {
  typedef const ::clang::snu::WCFGBlock NodeType;
  typedef ::clang::snu::WCFGBlock::const_pred_iterator ChildIteratorType;

  static NodeType *getEntryNode(Inverse<const ::clang::snu::WCFGBlock*> G) {
    return G.Graph;
  }

  static inline ChildIteratorType child_begin(NodeType *N) {
    return N->pred_begin();
  }
  static inline ChildIteratorType child_end(NodeType *N) {
    return N->pred_end();
  }
};

template <> struct GraphTraits< ::clang::snu::WCFG*>
  : public GraphTraits< ::clang::snu::WCFGBlock*> {

  typedef ::clang::snu::WCFG::graph_iterator nodes_iterator;

  static NodeType *getEntryNode(::clang::snu::WCFG *F) {
    return F->getEntry();
  }

  static nodes_iterator nodes_begin(::clang::snu::WCFG *F) {
    return F->nodes_begin();
  }
  static nodes_iterator nodes_end(::clang::snu::WCFG *F) {
    return F->nodes_end();
  }

  static unsigned size(::clang::snu::WCFG *F) {
    return F->size();
  }
};

template <> struct GraphTraits<const ::clang::snu::WCFG*>
  : public GraphTraits<const ::clang::snu::WCFGBlock*> {

  typedef ::clang::snu::WCFG::const_graph_iterator nodes_iterator;

  static NodeType *getEntryNode(const ::clang::snu::WCFG *F) {
    return F->getEntry();
  }
  static nodes_iterator nodes_begin(const ::clang::snu::WCFG *F) {
    return F->nodes_begin();
  }
  static nodes_iterator nodes_end(const ::clang::snu::WCFG *F) {
    return F->nodes_end();
  }

  static unsigned size(const ::clang::snu::WCFG *F) {
    return F->size();
  }
};

template <> struct GraphTraits<Inverse< ::clang::snu::WCFG*> >
  : public GraphTraits<Inverse< ::clang::snu::WCFGBlock*> > {

  typedef ::clang::snu::WCFG::graph_iterator nodes_iterator;

  static NodeType *getEntryNode(::clang::snu::WCFG *F) {
    return F->getExit();
  }

  static nodes_iterator nodes_begin(::clang::snu::WCFG *F) {
    return F->nodes_begin();
  }
  static nodes_iterator nodes_end(::clang::snu::WCFG *F) {
    return F->nodes_end();
  }
};

template <> struct GraphTraits<Inverse<const ::clang::snu::WCFG*> >
  : public GraphTraits<Inverse<const ::clang::snu::WCFGBlock*> > {

  typedef ::clang::snu::WCFG::const_graph_iterator nodes_iterator;

  static NodeType *getEntryNode(const ::clang::snu::WCFG *F) {
    return F->getExit();
  }

  static nodes_iterator nodes_begin(const ::clang::snu::WCFG *F) {
    return F->nodes_begin();
  }
  static nodes_iterator nodes_end(const ::clang::snu::WCFG *F) {
    return F->nodes_end();
  }
};

} // namespace llvm

#endif // LLVM_CLANG_SNU_AST_WCFG_H
