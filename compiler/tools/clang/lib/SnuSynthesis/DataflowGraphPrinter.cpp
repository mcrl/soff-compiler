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

#include "clang/SnuSynthesis/DataflowGraph.h"
#include "clang/AST/ASTContext.h"
#include "clang/AST/Decl.h"
#include "clang/AST/Type.h"
#include "clang/Analysis/Support/BumpVector.h"
#include "clang/Basic/LLVM.h"
#include "clang/SnuAST/WAST.h"
#include "clang/SnuAST/WCFG.h"
#include "llvm/ADT/APFloat.h"
#include "llvm/ADT/APInt.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/Support/Allocator.h"
#include <utility>

namespace clang {

namespace snu {

namespace {

class DFGNodePrinter: public DFGNodeVisitor<DFGNodePrinter, void> {
  raw_ostream &OS;

public:
  DFGNodePrinter(raw_ostream &os)
    : OS(os) {}

  void VisitSourceNode(DFGSourceNode *Node);
  void VisitSinkNode(DFGSinkNode *Node);
  void VisitIntConstNode(DFGIntConstNode *Node);
  void VisitFloatConstNode(DFGFloatConstNode *Node);
  void VisitUndefinedConstNode(DFGUndefinedConstNode *Node);
  void VisitPlatformConstNode(DFGPlatformConstNode *Node);
  void VisitAddrOfNode(DFGAddrOfNode *Node);
  void VisitUnaryOpNode(DFGUnaryOpNode *Node);
  void VisitBinaryOpNode(DFGBinaryOpNode *Node);
  void VisitTernaryOpNode(DFGTernaryOpNode *Node);
  void VisitRangeOpNode(DFGRangeOpNode *Node);
  void VisitVariableRangeOpNode(DFGVariableRangeOpNode *Node);
  void VisitConcatOpNode(DFGConcatOpNode *Node);
  void VisitSubstituteOpNode(DFGSubstituteOpNode *Node);
  void VisitLoadNode(DFGLoadNode *Node);
  void VisitStoreNode(DFGStoreNode *Node);
  void VisitNullaryAtomicNode(DFGNullaryAtomicNode *Node);
  void VisitUnaryAtomicNode(DFGUnaryAtomicNode *Node);
  void VisitBinaryAtomicNode(DFGBinaryAtomicNode *Node);
  void VisitFunctionCallNode(DFGFunctionCallNode *Node);
  void VisitShiftRegisterNode(DFGShiftRegisterNode *Node);
  void VisitQueueNode(DFGQueueNode *Node);
  void VisitBarrierNode(DFGBarrierNode *Node);
  void VisitScatterNode(DFGScatterNode *Node);
  void VisitCompoundNode(DFGCompoundNode *Node);

  void PrintSuccessors(DFGNode *Node);
};

void DFGNodePrinter::VisitSourceNode(DFGSourceNode *Node) {
  OS << "  n" << Node << " [shape=box,label=\"source\"];\n";
}

void DFGNodePrinter::VisitSinkNode(DFGSinkNode *Node) {
  OS << "  n" << Node << " [shape=box,label=\"sink\"];\n";
}

void DFGNodePrinter::VisitIntConstNode(DFGIntConstNode *Node) {
  OS << "  n" << Node << " [shape=box,label=\"" << Node->getValue() << "\"];\n";
}

void DFGNodePrinter::VisitFloatConstNode(DFGFloatConstNode *Node) {
  OS << "  n" << Node << " [shape=box,label=\""
     << Node->getValue().bitcastToAPInt() << "\"];\n";
}

void DFGNodePrinter::VisitUndefinedConstNode(DFGUndefinedConstNode *Node) {
  OS << "  n" << Node << " [shape=box,label=\"0\"];\n";
}

void DFGNodePrinter::VisitPlatformConstNode(DFGPlatformConstNode *Node) {
  OS << "  n" << Node << " [shape=box,label=\"" << Node->getName() << "\"];\n";
}

void DFGNodePrinter::VisitAddrOfNode(DFGAddrOfNode *Node) {
  OS << "  n" << Node << " [shape=box,label=\"&";
  Node->getVariable()->printName(OS);
  OS << "\"];\n";
}

void DFGNodePrinter::VisitUnaryOpNode(DFGUnaryOpNode *Node) {
  OS << "  n" << Node << " [shape=ellipse,label=\"";
  switch (Node->getOpcode()) {
    case DFGUnaryOpNode::DUO_Minus:  OS << '-'; break;
    case DFGUnaryOpNode::DUO_Not:    OS << '~'; break;
    case DFGUnaryOpNode::DUO_LNot:   OS << '!'; break;
    case DFGUnaryOpNode::DUO_LNot_V: OS << "V!"; break;
    case DFGUnaryOpNode::DUO_ReinterpretCast: OS << "reinterpret"; break;
    case DFGUnaryOpNode::DUO_IntCast:         OS << "itoi"; break;
    case DFGUnaryOpNode::DUO_IntToFloatCast:  OS << "itof"; break;
    case DFGUnaryOpNode::DUO_FloatToIntCast:  OS << "ftoi"; break;
    case DFGUnaryOpNode::DUO_FloatCast:       OS << "ftof"; break;
    case DFGUnaryOpNode::DUO_Cos:   OS << "cos"; break;
    case DFGUnaryOpNode::DUO_Sin:   OS << "sin"; break;
    case DFGUnaryOpNode::DUO_Tan:   OS << "tan"; break;
    case DFGUnaryOpNode::DUO_Acos:  OS << "acos"; break;
    case DFGUnaryOpNode::DUO_Asin:  OS << "asin"; break;
    case DFGUnaryOpNode::DUO_Atan:  OS << "atan"; break;
    case DFGUnaryOpNode::DUO_Exp:   OS << "exp"; break;
    case DFGUnaryOpNode::DUO_Log:   OS << "log"; break;
    case DFGUnaryOpNode::DUO_Log2:  OS << "log2"; break;
    case DFGUnaryOpNode::DUO_Log10: OS << "log10"; break;
    case DFGUnaryOpNode::DUO_Sqrt:  OS << "sqrt"; break;
    case DFGUnaryOpNode::DUO_RSqrt: OS << "rsqrt"; break;
    case DFGUnaryOpNode::DUO_Ceil:  OS << "ceil"; break;
    case DFGUnaryOpNode::DUO_Floor: OS << "floor"; break;
    case DFGUnaryOpNode::DUO_Abs:   OS << "abs"; break;
    default: llvm_unreachable("invalid unary operation");
  }
  OS << "\"];\n";
}

void DFGNodePrinter::VisitBinaryOpNode(DFGBinaryOpNode *Node) {
  OS << "  n" << Node << " [shape=ellipse,label=\"";
  switch (Node->getOpcode()) {
    case DFGBinaryOpNode::DBO_Mul:    OS << '*'; break;
    case DFGBinaryOpNode::DBO_Div:    OS << '/'; break;
    case DFGBinaryOpNode::DBO_Rem:    OS << '%'; break;
    case DFGBinaryOpNode::DBO_Add:    OS << '+'; break;
    case DFGBinaryOpNode::DBO_Sub:    OS << '-'; break;
    case DFGBinaryOpNode::DBO_Shl:    OS << "<<"; break;
    case DFGBinaryOpNode::DBO_Shr:    OS << ">>"; break;
    case DFGBinaryOpNode::DBO_LT:     OS << '<'; break;
    case DFGBinaryOpNode::DBO_GT:     OS << '>'; break;
    case DFGBinaryOpNode::DBO_LE:     OS << "<="; break;
    case DFGBinaryOpNode::DBO_GE:     OS << ">="; break;
    case DFGBinaryOpNode::DBO_LT_V:   OS << "V<"; break;
    case DFGBinaryOpNode::DBO_GT_V:   OS << "V>"; break;
    case DFGBinaryOpNode::DBO_LE_V:   OS << "V<="; break;
    case DFGBinaryOpNode::DBO_GE_V:   OS << "V>="; break;
    case DFGBinaryOpNode::DBO_EQ:     OS << "=="; break;
    case DFGBinaryOpNode::DBO_NE:     OS << "!="; break;
    case DFGBinaryOpNode::DBO_EQ_V:   OS << "V=="; break;
    case DFGBinaryOpNode::DBO_NE_V:   OS << "V!="; break;
    case DFGBinaryOpNode::DBO_And:    OS << '&'; break;
    case DFGBinaryOpNode::DBO_Xor:    OS << '^'; break;
    case DFGBinaryOpNode::DBO_Or:     OS << '|'; break;
    case DFGBinaryOpNode::DBO_LAnd:   OS << "&&"; break;
    case DFGBinaryOpNode::DBO_LAnd_V: OS << "V&&"; break;
    case DFGBinaryOpNode::DBO_LOr:    OS << "||"; break;
    case DFGBinaryOpNode::DBO_LOr_V:  OS << "V||"; break;
    case DFGBinaryOpNode::DBO_Pow:    OS << "pow"; break;
    case DFGBinaryOpNode::DBO_Fmod:   OS << "fmod"; break;
    case DFGBinaryOpNode::DBO_Max:    OS << "max"; break;
    case DFGBinaryOpNode::DBO_Min:    OS << "min"; break;
    case DFGBinaryOpNode::DBO_Mul24:  OS << "mul24"; break;
    default: llvm_unreachable("invalid binary operation");
  }
  OS << "\"];\n";
}

void DFGNodePrinter::VisitTernaryOpNode(DFGTernaryOpNode *Node) {
  OS << "  n" << Node << " [shape=ellipse,label=\"";
  switch (Node->getOpcode()) {
    case DFGTernaryOpNode::DTO_Conditional: OS << "?:"; break;
    default: llvm_unreachable("invalid ternary operation");
  }
  OS << "\"];\n";
}

void DFGNodePrinter::VisitRangeOpNode(DFGRangeOpNode *Node) {
  OS << "  n" << Node << " [shape=ellipse,label=\"[" << Node->getUpperIndex()
     << ':' << Node->getLowerIndex() << "]\"];\n";
}

void DFGNodePrinter::VisitVariableRangeOpNode(DFGVariableRangeOpNode *Node) {
  OS << "  n" << Node << " [shape=ellipse,label=\"[+:" << Node->getWidth()
     << "]\"];\n";
}

void DFGNodePrinter::VisitConcatOpNode(DFGConcatOpNode *Node) {
  OS << "  n" << Node << " [shape=ellipse,label=\"concat\"];\n";
}

void DFGNodePrinter::VisitSubstituteOpNode(DFGSubstituteOpNode *Node) {
  OS << "  n" << Node << " [shape=ellipse,label=\"[+:" << Node->getWidth()
     << "] =\"];\n";
}

void DFGNodePrinter::VisitLoadNode(DFGLoadNode *Node) {
  OS << "  n" << Node << " [shape=ellipse,label=\"";
  OS << "load(" << Node->getAliasGroup() << ')';
  OS << "\"];\n";
}

void DFGNodePrinter::VisitStoreNode(DFGStoreNode *Node) {
  OS << "  n" << Node << " [shape=ellipse,label=\"";
  OS << "store(" << Node->getAliasGroup() << ')';
  OS << "\"];\n";
}

void DFGNodePrinter::VisitNullaryAtomicNode(DFGNullaryAtomicNode *Node) {
  OS << "  n" << Node << " [shape=ellipse,label=\"";
  switch (Node->getOpcode()) {
    case DFGNullaryAtomicNode::DNA_Inc: OS << "atomic_inc"; break;
    case DFGNullaryAtomicNode::DNA_Dec: OS << "atomic_dec"; break;
    default: llvm_unreachable("invalid operation");
  }
  OS << "\"];\n";
}

void DFGNodePrinter::VisitUnaryAtomicNode(DFGUnaryAtomicNode *Node) {
  OS << "  n" << Node << " [shape=ellipse,label=\"";
  switch (Node->getOpcode()) {
    case DFGUnaryAtomicNode::DUA_Add:  OS << "atomic_add"; break;
    case DFGUnaryAtomicNode::DUA_Sub:  OS << "atomic_sub"; break;
    case DFGUnaryAtomicNode::DUA_Xchg: OS << "atomic_xchg"; break;
    case DFGUnaryAtomicNode::DUA_Min:  OS << "atomic_min"; break;
    case DFGUnaryAtomicNode::DUA_Max:  OS << "atomic_max"; break;
    case DFGUnaryAtomicNode::DUA_And:  OS << "atomic_and"; break;
    case DFGUnaryAtomicNode::DUA_Or:   OS << "atomic_or"; break;
    case DFGUnaryAtomicNode::DUA_Xor:  OS << "atomic_xor"; break;
    default: llvm_unreachable("invalid operation");
  }
  OS << "\"];\n";
}

void DFGNodePrinter::VisitBinaryAtomicNode(DFGBinaryAtomicNode *Node) {
  OS << "  n" << Node << " [shape=ellipse,label=\"";
  switch (Node->getOpcode()) {
    case DFGBinaryAtomicNode::DBA_CmpXchg: OS << "atomic_cmpxchg"; break;
    default: llvm_unreachable("invalid operation");
  }
  OS << "\"];\n";
}

void DFGNodePrinter::VisitFunctionCallNode(DFGFunctionCallNode *Node) {
  OS << "  n" << Node << " [shape=ellipse,label=\"";
  Node->getFunction()->printName(OS);
  OS << "\"];\n";
}

void DFGNodePrinter::VisitShiftRegisterNode(DFGShiftRegisterNode *Node) {
  OS << "  n" << Node << " [shape=plaintext,label=\"" << Node->getSize()
     << " cycles R\"];\n";
}

void DFGNodePrinter::VisitQueueNode(DFGQueueNode *Node) {
  OS << "  n" << Node << " [shape=plaintext,label=\"" << Node->getSize()
     << " cycles Q\"];\n";
}

void DFGNodePrinter::VisitBarrierNode(DFGBarrierNode *Node) {
  OS << "  n" << Node << " [shape=ellipse,label=\"barrier\"];\n";
}

void DFGNodePrinter::VisitScatterNode(DFGScatterNode *Node) {
  OS << "  n" << Node << " [shape=ellipse,label=\"out\"];\n";
}

void DFGNodePrinter::VisitCompoundNode(DFGCompoundNode *Node) {
  for (DFGCompoundNode::iterator N = Node->begin(), NEnd = Node->end();
       N != NEnd; ++N) {
    OS << "  ";
    Visit(*N);
  }
}

void DFGNodePrinter::PrintSuccessors(DFGNode *Node) {
  unsigned NumAlignedSuccs = 0;
  for (DFGNode::const_succ_iterator S = Node->succ_begin(),
                                    SEnd = Node->succ_end();
       S != SEnd; ++S) {
    OS << "  n" << Node << " -> n" << (*S) << ";\n";
    if (isa<DFGConstNode>(*S) || isa<DFGScatterNode>(*S)) {
      NumAlignedSuccs++;
    }
  }
  if (NumAlignedSuccs >= 2) {
    OS << "  { rank = same; ";
    for (DFGNode::const_succ_iterator S = Node->succ_begin(),
                                      SEnd = Node->succ_end();
         S != SEnd; ++S) {
      if (isa<DFGConstNode>(*S) || isa<DFGScatterNode>(*S)) {
        OS << 'n' << *S << "; ";
      }
    }
    OS << "}\n";
  }
}

} // anonymous namespace

void DataflowGraph::print(raw_ostream &OS) const {
  OS << "subgraph cluster" << EntryBlock->getBlockID() << " {\n";
  OS << "  style = filled;\n";
  OS << "  color = lightgrey;\n";
  OS << "  label = \"B" << EntryBlock->getBlockID() << "-B"
     << ExitBlock->getBlockID() << "\";\n";
  DFGNodePrinter P(OS);
  for (DFGNodeListTy::const_iterator N = Nodes.begin(), NEnd = Nodes.end();
       N != NEnd; ++N) {
    if (!NodesInAnyCompound.count(*N)) {
      P.Visit(*N);
    }
  }
  for (CompoundListTy::const_iterator C = Compounds.begin(),
                                      CEnd = Compounds.end();
       C != CEnd; ++C) {
    OS << "  subgraph cluster" << EntryBlock->getBlockID() << "_c" << *C << " {\n";
    OS << "    style = dashed;\n";
    OS << "    color = red;\n";
    OS << "    label = \"\";\n";
    P.Visit(*C);
    OS << "  }\n";
  }
  for (DFGNodeListTy::const_iterator N = Nodes.begin(), NEnd = Nodes.end();
       N != NEnd; ++N) {
    P.PrintSuccessors(*N);
  }
  OS << "}\n\n";
}

void ControlDataflowGraph::print(raw_ostream &OS) const {
  OS << "digraph cdfg_" << this << " {\n";
  for (unsigned Index = 0, NumBlocks = cfg->size();
       Index != NumBlocks; ++Index) {
    if (DataflowGraph *dfg = DataflowGraphMap[Index]) {
      dfg->print(OS);

      WCFGBlock *ExitBlock = dfg->getExitBlock();
      for (WCFGBlock::const_succ_iterator S = ExitBlock->real_succ_begin(),
                                          SEnd = ExitBlock->real_succ_end();
           S != SEnd; ++S) {
        DataflowGraph *succ = DataflowGraphMap[(*S)->getBlockID()];
        assert(succ != NULL);
        OS << "n" << dfg->getSink() << " -> n" << succ->getSource()
           << " [style=bold];\n";
      }
      OS << '\n';
    }
  }
  OS << "}\n";
  OS.flush();
}


} // namespace snu

} // namespace clang
