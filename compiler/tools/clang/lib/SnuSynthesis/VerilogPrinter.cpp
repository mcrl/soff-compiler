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

#include "clang/SnuSynthesis/Verilog.h"
#include "clang/Basic/LLVM.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/Support/raw_ostream.h"
#include <ctime>
#include <map>
#include <string>

namespace clang {

namespace snu {

void VerilogFile::print(raw_ostream &OS) const {
  std::time_t now = std::time(NULL);
  OS << "// " << getName() << "\n";
  OS << "// Generated at " << std::asctime(std::localtime(&now)) << '\n';
  OS << '\n';

  for (const_define_iterator D = define_begin(), DEnd = define_end();
       D != DEnd; ++D) {
    (*D)->print(OS);
  }
  if (num_defines() > 0) {
    OS << '\n';
  }

  for (const_module_iterator M = module_begin(), MEnd = module_end();
       M != MEnd; ++M) {
    (*M)->print(OS);
    OS << '\n';
  }
}

void VerilogParameter::print(raw_ostream &OS) const {
  switch (getKind()) {
    case PK_Param: OS << "parameter "; break;
    case PK_LocalParam: OS << "localparam "; break;
  }
  OS << getName() << " = " << getDefaultValue();
}

void VerilogParamConst::print(raw_ostream &OS) const {
  if (isParametrized()) {
    OS << getParameter()->getName();
    if (int Delta = getParameterDelta()) {
      if (Delta > 0) {
        OS << " + " << Delta;
      } else {
        OS << " - " << -Delta;
      }
    }
  } else {
    OS << getValue();
  }
}

void VerilogDefine::print(raw_ostream &OS) const {
  OS << "`define " << getName() << ' ';
  getValue()->print(OS);
  OS << '\n';
}

void VerilogSignal::print(raw_ostream &OS) const {
  if (hasAnyAttribute()) {
    bool IsTrailingAttribute = false;
    OS << "(* ";
    if (hasAttribute(SA_DontMerge)) {
      if (IsTrailingAttribute) OS << ", ";
      OS << "dont_merge";
      IsTrailingAttribute = true;
    }
    if (hasAttribute(SA_DontTouch)) {
      if (IsTrailingAttribute) OS << ", ";
      OS << "dont_touch = \"yes\"";
      IsTrailingAttribute = true;
    }
    OS << " *) ";
  }
  switch (getPortKind()) {
    case SP_Input:  OS << "input "; break;
    case SP_Output: OS << "output "; break;
    default: break;
  }
  switch (getKind()) {
    case SK_Wire: OS << "wire "; break;
    case SK_Reg:  OS << "reg "; break;
  }
  if (isSigned()) {
    OS << "signed ";
  }
  if (isVector()) {
    OS << '[';
    VerilogParamConst(getVectorWidth(), -1).print(OS);
    OS << ":0] ";
  }
  OS << getName();
  if (isArray()) {
    OS << " [0:";
    VerilogParamConst(getArrayWidth(), -1).print(OS);
    OS << ']';
  }
}

void VerilogModule::print(raw_ostream &OS) const {
  OS << "module " << getName() << ' ';

  if (num_params() > 0) {
    OS << "#(\n";
    bool IsAnyParamPrinted = false;
    for (const_param_iterator P = param_begin(), PEnd = param_end();
         P != PEnd; ++P) {
      if ((*P)->getKind() == PK_Param) {
        if (IsAnyParamPrinted) OS << ",\n";
        IsAnyParamPrinted = true;
        OS << "  ";
        (*P)->print(OS);
      }
    }
    OS << "\n) ";
  }

  OS << "(\n";
  bool IsAnyPortPrinted = false;
  for (const_signal_iterator S = signal_begin(), SEnd = signal_end();
       S != SEnd; ++S) {
    if ((*S)->isPort()) {
      if (IsAnyPortPrinted) OS << ",\n";
      IsAnyPortPrinted = true;
      OS << "  ";
      (*S)->print(OS);
    }
  }
  OS << "\n);\n";

  for (const_param_iterator P = param_begin(), PEnd = param_end();
       P != PEnd; ++P) {
    if ((*P)->getKind() == PK_LocalParam) {
      (*P)->print(OS);
      OS << ";\n";
    }
  }
  OS << '\n';

  for (const_signal_iterator S = signal_begin(), SEnd = signal_end();
       S != SEnd; ++S) {
    if (!(*S)->isPort()) {
      (*S)->print(OS);
      OS << ";\n";
    }
  }
  OS << '\n';

  for (const_module_inst_iterator M = module_inst_begin(),
                                  MEnd = module_inst_end();
       M != MEnd; ++M) {
    (*M)->print(OS);
    OS << '\n';
  }

  for (const_body_iterator I = body_begin(), E = body_end();
       I != E; ++I) {
    (*I)->print(OS);
  }
  OS << '\n';

  OS << "endmodule // module " << getName() << '\n';
}

void VerilogModuleInstance::print(raw_ostream &OS) const {
  assert(getTarget() != NULL);
  OS << getTarget()->getName() << ' ';
  if (param_arg_size() > 0) {
    OS << "#(\n";
    for (const_param_arg_iterator P = param_arg_begin(), PEnd = param_arg_end();
         P != PEnd; ++P) {
      if (P != param_arg_begin()) {
        OS << ",\n";
      }
      OS << "  ." << (*P).first->getName() << '(';
      (*P).second.print(OS);
      OS << ')';
    }
    OS << "\n) ";
  }
  OS << getName() << "(\n";
  if (arg_size() > 0) {
    for (const_arg_iterator A = arg_begin(), AEnd = arg_end();
         A != AEnd; ++A) {
      if (A != arg_begin()) {
        OS << ",\n";
      }
      OS << "  ." << (*A).first->getName() << '(';
      (*A).second->print(OS);
      OS << ')';
    }
    OS << '\n';
  }
  OS << ");\n";
}

void VerilogEvent::print(raw_ostream &OS) const {
  switch (getKind()) {
    case EvK_Change: break;
    case EvK_Posedge: OS << "posedge "; break;
    case EvK_Negedge: OS << "negedge "; break;
  }
  getSignal()->print(OS);
}

namespace {

class VerilogPrinter {
  raw_ostream &OS;
  unsigned IndentLevel;

public:
  VerilogPrinter(raw_ostream &os)
    : OS(os), IndentLevel(0) {}

  void Visit(VerilogStmt *Node);

  void VisitAlways(VerilogAlways *Node);
  void VisitAssign(VerilogAssign *Node);
  void VisitCompound(VerilogCompound *Node);
  void VisitIf(VerilogIf *Node);
  void VisitInitial(VerilogInitial *Node);
  void VisitSwitch(VerilogSwitch *Node);
  void VisitConst(VerilogConst *Node);
  void VisitParamRef(VerilogParamRef *Node);
  void VisitDefineRef(VerilogDefineRef *Node);
  void VisitSignalRef(VerilogSignalRef *Node);
  void VisitPartSelect(VerilogPartSelect *Node);
  void VisitConcat(VerilogConcat *Node);
  void VisitReplication(VerilogReplication *Node);
  void VisitUnaryOperator(VerilogUnaryOperator *Node);
  void VisitBinaryOperator(VerilogBinaryOperator *Node);
  void VisitConditionalOperator(VerilogConditionalOperator *Node);

private:
  raw_ostream &Indent(int Delta = 0) {
    for (int i = 0, e = IndentLevel + Delta; i < e; ++i) {
      OS << "  ";
    }
    return OS;
  }

  void PrintStmt(VerilogStmt *S, int SubIndent = 1) {
    if (isa<VerilogExpr>(S)) {
      Indent();
      Visit(S);
      OS << ";\n";
    } else {
      Visit(S);
    }
  }

  void PrintRawIf(VerilogIf *Node);
  void PrintRawCompound(VerilogCompound *Node);
  void PrintRawCompoundOrSingleStmt(VerilogStmt *Node);
  void PrintOperandExpr(VerilogExpr *Node);

  bool RequireOrdering(VerilogExpr *Node) {
    switch (Node->getClass()) {
      case VerilogStmt::VS_UnaryOperatorClass:
      case VerilogStmt::VS_BinaryOperatorClass:
      case VerilogStmt::VS_ConditionalOperatorClass:
        return true;
      default:
        return false;
    }
  }
};

void VerilogPrinter::PrintRawCompound(VerilogCompound *Node) {
  OS << "begin\n";
  IndentLevel++;
  for (VerilogCompound::iterator I = Node->begin(), E = Node->end();
       I != E; ++I) {
    PrintStmt(*I);
  }
  IndentLevel--;
  Indent() << "end\n";
}

void VerilogPrinter::PrintRawCompoundOrSingleStmt(VerilogStmt *Node) {
  if (VerilogCompound *C = dyn_cast<VerilogCompound>(Node)) {
    PrintRawCompound(C);
  } else {
    Visit(Node);
  }
}

void VerilogPrinter::PrintOperandExpr(VerilogExpr *Node) {
  if (RequireOrdering(Node)) {
    OS << '(';
    Visit(Node);
    OS << ')';
  } else {
    Visit(Node);
  }
}

void VerilogPrinter::Visit(VerilogStmt *Node) {
  switch (Node->getClass()) {
#define NODE(type) \
  case VerilogStmt::VS_##type##Class: \
    return Visit ## type(static_cast<Verilog##type*>(Node));
  NODE(Always)
  NODE(Assign)
  NODE(Compound)
  NODE(If)
  NODE(Initial)
  NODE(Switch)
  NODE(Const)
  NODE(ParamRef)
  NODE(DefineRef)
  NODE(SignalRef)
  NODE(PartSelect)
  NODE(Concat)
  NODE(Replication)
  NODE(UnaryOperator)
  NODE(BinaryOperator)
  NODE(ConditionalOperator)
#undef NODE
  default:
    llvm_unreachable("invalid stmt class");
  }
}

void VerilogPrinter::VisitAlways(VerilogAlways *Node) {
  Indent() << "always @(";
  for (VerilogAlways::event_iterator E = Node->event_begin(),
                                     EEnd = Node->event_end();
       E != EEnd; ++E) {
    (*E).print(OS);
    if (E + 1 != EEnd) {
      OS << " or ";
    }
  }
  OS << ") ";
  PrintRawCompound(Node->getBody());
}

void VerilogPrinter::VisitAssign(VerilogAssign *Node) {
  Indent();
  if (Node->isTopLevel()) {
    OS << "assign ";
  }
  Visit(Node->getLHS());
  if (Node->isTopLevel() || Node->isBlocking())
    OS << " = ";
  else
    OS << " <= ";
  Visit(Node->getRHS());
  OS << ";\n";
}

void VerilogPrinter::VisitCompound(VerilogCompound *Node) {
  Indent();
  PrintRawCompound(Node);
}

void VerilogPrinter::PrintRawIf(VerilogIf *Node) {
  OS << "if (";
  Visit(Node->getCond());
  OS << ") ";
  PrintRawCompoundOrSingleStmt(Node->getThen());

  if (Node->getElse()) {
    Indent() << "else ";
    if (VerilogIf *ElseIf = dyn_cast<VerilogIf>(Node->getElse())) {
      PrintRawIf(ElseIf);
    } else {
      PrintRawCompoundOrSingleStmt(Node->getElse());
    }
  }
}

void VerilogPrinter::VisitIf(VerilogIf *Node) {
  Indent();
  PrintRawIf(Node);
}

void VerilogPrinter::VisitInitial(VerilogInitial *Node) {
  Indent() << "initial ";
  PrintRawCompound(Node->getBody());
}

void VerilogPrinter::VisitSwitch(VerilogSwitch *Node) {
  Indent() << "case (";
  Visit(Node->getCond());
  OS << ")\n";

  IndentLevel++;
  for (unsigned Index = 0, NumCases = Node->getNumCases();
       Index != NumCases; ++Index) {
    Indent();
    Visit(Node->getCase(Index));
    OS << ": ";
    PrintRawCompoundOrSingleStmt(Node->getCaseStmt(Index));
  }
  if (Node->hasDefaultStmt()) {
    Indent() << "default: ";
    PrintRawCompoundOrSingleStmt(Node->getDefaultStmt());
  }
  IndentLevel--;
}

void VerilogPrinter::VisitConst(VerilogConst *Node) {
  OS << Node->getAsString();
}

void VerilogPrinter::VisitParamRef(VerilogParamRef *Node) {
  OS << Node->getParameter()->getName();
}

void VerilogPrinter::VisitDefineRef(VerilogDefineRef *Node) {
  OS << Node->getDefine()->getName();
}

void VerilogPrinter::VisitSignalRef(VerilogSignalRef *Node) {
  OS << Node->getSignal()->getName();
  if (Node->hasArrayIndex()) {
    OS << '[';
    Visit(Node->getArrayIndex());
    OS << ']';
  }
  if (Node->hasVectorRange()) {
    OS << '[';
    Node->getMSB().print(OS);
    if (Node->getMSB() != Node->getLSB()) {
      OS << ':';
      Node->getLSB().print(OS);
    }
    OS << ']';
  }
}

void VerilogPrinter::VisitPartSelect(VerilogPartSelect *Node) {
  OS << Node->getSignal()->getName();
  OS << '[';
  Visit(Node->getOffset());
  OS << "+:";
  Node->getWidth().print(OS);
  OS << ']';
}

void VerilogPrinter::VisitConcat(VerilogConcat *Node) {
  OS << '{';
  for (unsigned Index = 0, NumOperands = Node->getNumOperands();
       Index != NumOperands; ++Index) {
    if (Index > 0) OS << ", ";
    Visit(Node->getOperand(Index));
  }
  OS << '}';
}

void VerilogPrinter::VisitReplication(VerilogReplication *Node) {
  OS << '{' << Node->getNumber() << '{';
  Visit(Node->getOperand());
  OS << "}}";
}

void VerilogPrinter::VisitUnaryOperator(VerilogUnaryOperator *Node) {
  switch (Node->getOpcode()) {
    case VUO_PLUS:  OS << '+';  break;
    case VUO_MINUS: OS << '-';  break;
    case VUO_LNOT:  OS << '!';  break;
    case VUO_NOT:   OS << '~';  break;
    case VUO_RAND:  OS << '&';  break;
    case VUO_RNAND: OS << "~&"; break;
    case VUO_ROR:   OS << '|';  break;
    case VUO_RNOR:  OS << "~|"; break;
    case VUO_RXOR:  OS << '^';  break;
    case VUO_RXNOR: OS << "~^"; break;
    default: llvm_unreachable("impossible");
  }
  PrintOperandExpr(Node->getOperand());
}

void VerilogPrinter::VisitBinaryOperator(VerilogBinaryOperator *Node) {
  PrintOperandExpr(Node->getLHS());
  switch (Node->getOpcode()) {
    case VBO_ADD:  OS << " + ";   break;
    case VBO_SUB:  OS << " - ";   break;
    case VBO_MUL:  OS << " * ";   break;
    case VBO_DIV:  OS << " / ";   break;
    case VBO_MOD:  OS << " % ";   break;
    case VBO_GT:   OS << " > ";   break;
    case VBO_GE:   OS << " >= ";  break;
    case VBO_LT:   OS << " < ";   break;
    case VBO_LE:   OS << " <= ";  break;
    case VBO_LAND: OS << " && ";  break;
    case VBO_LOR:  OS << " || ";  break;
    case VBO_LEQ:  OS << " == ";  break;
    case VBO_LNE:  OS << " != ";  break;
    case VBO_CEQ:  OS << " === "; break;
    case VBO_CNE:  OS << " !== "; break;
    case VBO_AND:  OS << " & ";   break;
    case VBO_OR:   OS << " | ";   break;
    case VBO_XOR:  OS << " ^ ";   break;
    case VBO_XNOR: OS << " ~^ ";  break;
    case VBO_LSHL: OS << " << ";  break;
    case VBO_LSHR: OS << " >> ";  break;
    case VBO_ASHL: OS << " <<< "; break;
    case VBO_ASHR: OS << " >>> "; break;
  }
  PrintOperandExpr(Node->getRHS());
}

void VerilogPrinter::VisitConditionalOperator(
    VerilogConditionalOperator *Node) {
  PrintOperandExpr(Node->getCond());
  OS << " ? ";
  PrintOperandExpr(Node->getLHS());
  OS << " : ";
  PrintOperandExpr(Node->getRHS());
}

} // anonymous namespace

void VerilogStmt::print(raw_ostream &OS) const {
  VerilogPrinter P(OS);
  P.Visit(const_cast<VerilogStmt*>(this));
}

} // namespace snu

} // namespace clang
