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

#include "clang/SnuAST/WAST.h"
#include "clang/AST/ASTContext.h"
#include "clang/AST/Decl.h"
#include "clang/AST/Expr.h"
#include "clang/AST/PrettyPrinter.h"
#include "clang/AST/Stmt.h"
#include "clang/AST/Type.h"
#include "clang/Basic/LLVM.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/Support/raw_ostream.h"
#include <string>
#include <utility>

namespace clang {

namespace snu {

namespace {

class WStmtPrinter: public WStmtVisitor<WStmtPrinter, void> {
  raw_ostream &OS;
  PrintingPolicy Policy;
  unsigned IndentLevel;

public:
  WStmtPrinter(raw_ostream &os, const PrintingPolicy &policy)
    : OS(os), Policy(policy), IndentLevel(0) {}

#define STMT(type) \
  void Visit##type(W##type *Node);
  CLANG_WSTMTS()
#undef STMT

#define WSTMT(type) \
  void Visit##type(W##type *Node);
  EXTENDED_WSTMTS()
#undef WSTMT

private:
  raw_ostream &Indent(int Delta = 0) {
    for (int i = 0, e = IndentLevel + Delta; i < e; ++i) {
      OS << "  ";
    }
    return OS;
  }

  void PrintStmt(WStmt *S) {
    PrintStmt(S, Policy.Indentation);
  }

  void PrintStmt(WStmt *S, int SubIndent) {
    IndentLevel += SubIndent;
    if (S && isa<WExpr>(S)) {
      Indent();
      Visit(S);
      OS << ";\n";
    } else if (S) {
      Visit(S);
    } else {
      Indent() << "<<<NULL STATEMENT>>>\n";
    }
    IndentLevel -= SubIndent;
  }

  void PrintExpr(WExpr *E) {
    if (E)
      Visit(E);
    else
      OS << "<null expr>";
  }

  void PrintCompoundStmtOrSingleStmt(WStmt *Node, bool HasTrailing = false);
  void PrintRawDeclStmt(WDeclStmt *Node);
  void PrintRawIfStmt(WIfStmt *Node);
};

void WStmtPrinter::PrintCompoundStmtOrSingleStmt(WStmt *Node,
                                                 bool HasTrailing) {
  // while (...) {         while (...)
  //   statements;   vs.     statement;
  // }

  // if HasTrailing is true,
  // if (...) {            if (...)
  //   statements;   vs.     statements;
  // } else ...            else ...

  if (WCompoundStmt *CS = dyn_cast<WCompoundStmt>(Node)) {
    OS << " {\n";
    for (WCompoundStmt::child_iterator I = CS->child_begin(),
                                       E = CS->child_end();
         I != E; ++I) {
      PrintStmt(*I);
    }
    Indent() << '}' << (HasTrailing ? ' ' : '\n');
  } else {
    OS << '\n';
    PrintStmt(Node);
    if (HasTrailing)
      Indent();
  }
}

void WStmtPrinter::PrintRawDeclStmt(WDeclStmt *Node) {
  SmallVector<Decl*, 2> Decls;
  for (DeclStmt::const_decl_iterator D = Node->decl_begin(),
                                     DEnd = Node->decl_end();
       D != DEnd; ++D) {
    Decls.push_back(*D);
  }

  Decl::printGroup(Decls.data(), Decls.size(), OS, Policy, IndentLevel);

  if (Node->getIndexedDecl()) {
    OS << " /* ";
    Node->getIndexedDecl().printName(OS);
    OS << " */";
  }

  if (Node->hasSingleInit()) {
    OS << " /* = ";
    PrintExpr(Node->getSingleInit());
    OS << " */";
  }
}

void WStmtPrinter::VisitDeclStmt(WDeclStmt *Node) {
  Indent();
  PrintRawDeclStmt(Node);
  OS << ";\n";
}

void WStmtPrinter::VisitNullStmt(WNullStmt *Node) {
  Indent() << ";\n";
}

void WStmtPrinter::VisitCompoundStmt(WCompoundStmt *Node) {
  Indent() << "{\n";
  for (WCompoundStmt::child_iterator I = Node->child_begin(),
                                     E = Node->child_end();
       I != E; ++I) {
    PrintStmt(*I);
  }
  Indent() << "}\n";
}

void WStmtPrinter::VisitCaseStmt(WCaseStmt *Node) {
  Indent(-1) << "case ";
  PrintExpr(Node->getLHS());
  OS << ":\n";
  PrintStmt(Node->getSubStmt(), 0);
}

void WStmtPrinter::VisitDefaultStmt(WDefaultStmt *Node) {
  Indent(-1) << "default:\n";
  PrintStmt(Node->getSubStmt(), 0);
}

void WStmtPrinter::VisitLabelStmt(WLabelStmt *Node) {
  Indent(-1) << Node->getName() << ":\n";
  PrintStmt(Node->getSubStmt(), 0);
}

void WStmtPrinter::VisitAttributedStmt(WAttributedStmt *Node) {
  OS << "[[";
  bool first = true;
  for (ArrayRef<const Attr*>::iterator A = Node->getAttrs().begin(),
                                       AEnd = Node->getAttrs().end();
       A != AEnd; ++A) {
    if (!first) {
      OS << ", ";
    }
    first = false;
    (*A)->printPretty(OS, Policy);
  }
  OS << "]] ";
  PrintStmt(Node->getSubStmt(), 0);
}

void WStmtPrinter::PrintRawIfStmt(WIfStmt *Node) {
  OS << "if (";
  PrintExpr(Node->getCond());
  OS << ')';
  PrintCompoundStmtOrSingleStmt(Node->getThen(), Node->getElse() != NULL);

  if (Node->getElse()) {
    OS << "else";

    if (WIfStmt *ElseIf = dyn_cast<WIfStmt>(Node->getElse())) {
      OS << ' ';
      PrintRawIfStmt(ElseIf);
    } else {
      PrintCompoundStmtOrSingleStmt(Node->getElse());
    }
  }
}

void WStmtPrinter::VisitIfStmt(WIfStmt *Node) {
  Indent();
  PrintRawIfStmt(Node);
}

void WStmtPrinter::VisitSwitchStmt(WSwitchStmt *Node) {
  Indent() << "switch (";
  PrintExpr(Node->getCond());
  OS << ')';
  PrintCompoundStmtOrSingleStmt(Node->getBody());
}

void WStmtPrinter::VisitWhileStmt(WWhileStmt *Node) {
  Indent() << "while (";
  PrintExpr(Node->getCond());
  OS << ')';
  PrintCompoundStmtOrSingleStmt(Node->getBody());
}

void WStmtPrinter::VisitDoStmt(WDoStmt *Node) {
  Indent() << "do";
  PrintCompoundStmtOrSingleStmt(Node->getBody(), true);
  OS << "while (";
  PrintExpr(Node->getCond());
  OS << ");\n";
}

void WStmtPrinter::VisitForStmt(WForStmt *Node) {
  Indent() << "for (";
  if (Node->getInit()) {
    if (WDeclStmt *DS = dyn_cast<WDeclStmt>(Node->getInit())) {
      PrintRawDeclStmt(DS);
    } else {
      PrintExpr(static_cast<WExpr*>(Node->getInit()));
    }
  }
  OS << ';';
  if (Node->getCond()) {
    OS << ' ';
    PrintExpr(Node->getCond());
  }
  OS << ';';
  if (Node->getInc()) {
    OS << ' ';
    PrintExpr(Node->getInc());
  }
  OS << ')';
  PrintCompoundStmtOrSingleStmt(Node->getBody());
}

void WStmtPrinter::VisitGotoStmt(WGotoStmt *Node) {
  Indent() << "goto " << Node->getLabel()->getName() << ";\n";
}

void WStmtPrinter::VisitContinueStmt(WContinueStmt *Node) {
  Indent() << "continue;\n";
}

void WStmtPrinter::VisitBreakStmt(WBreakStmt *Node) {
  Indent() << "break;\n";
}

void WStmtPrinter::VisitReturnStmt(WReturnStmt *Node) {
  Indent() << "return";
  if (Node->getRetExpr()) {
    OS << ' ';
    PrintExpr(Node->getRetExpr());
  }
  OS << ";\n";
}

void WStmtPrinter::VisitDeclRefExpr(WDeclRefExpr *Node) {
  OS << Node->getName();
  if (Node->getIndexedDefDecl() && Node->getIndexedUseDecl()) {
    OS << " /* ";
    Node->getIndexedDefDecl().printName(OS);
    OS << "<-";
    Node->getIndexedUseDecl().printName(OS);
    OS << " */";
  } else if (Node->getIndexedDefDecl()) {
    OS << " /* ";
    Node->getIndexedDefDecl().printName(OS);
    OS << " */";
  } else if (Node->getIndexedUseDecl()) {
    OS << " /* ";
    Node->getIndexedUseDecl().printName(OS);
    OS << " */";
  }
}

void WStmtPrinter::VisitPredefinedExpr(WPredefinedExpr *Node) {
  switch (Node->getIdentType()) {
    case PredefinedExpr::Func:
      OS << "__func__";
      break;
    case PredefinedExpr::Function:
      OS << "__FUNCTION__";
      break;
    case PredefinedExpr::FuncDName:
      OS << "__FUNCDNAME__";
      break;
    case PredefinedExpr::LFunction:
      OS << "L__FUNCTION__";
      break;
    case PredefinedExpr::PrettyFunction:
      OS << "__PRETTY_FUNCTION__";
      break;
    default:
      llvm_unreachable("unknown case");
  }
}

void WStmtPrinter::VisitIntegerLiteral(WIntegerLiteral *Node) {
  if (Node->getOriginal()) {
    Node->getOriginal()->printPretty(OS, NULL, Policy);
  } else {
    Node->getValue().print(OS, Node->getType()->isSignedIntegerType());
  }
}

void WStmtPrinter::VisitCharacterLiteral(WCharacterLiteral *Node) {
  if (Node->getOriginal()) {
    Node->getOriginal()->printPretty(OS, NULL, Policy);
  } else {
    OS << Node->getValue();
  }
}

void WStmtPrinter::VisitFloatingLiteral(WFloatingLiteral *Node) {
  Node->getOriginal()->printPretty(OS, NULL, Policy);
}

void WStmtPrinter::VisitStringLiteral(WStringLiteral *Node) {
  Node->getOriginal()->printPretty(OS, NULL, Policy);
}

void WStmtPrinter::VisitParenExpr(WParenExpr *Node) {
  OS << '(';
  PrintExpr(Node->getSubExpr());
  OS << ')';
}

void WStmtPrinter::VisitUnaryOperator(WUnaryOperator *Node) {
  if (!Node->isPostfix()) {
    OS << UnaryOperator::getOpcodeStr(Node->getOpcode());
    if ((Node->getOpcode() == UO_Plus || Node->getOpcode() == UO_Minus) &&
        isa<WUnaryOperator>(Node->getSubExpr())) {
      OS << ' ';
    }
  }
  PrintExpr(Node->getSubExpr());
  if (Node->isPostfix()) {
    OS << UnaryOperator::getOpcodeStr(Node->getOpcode());
  }
}

void WStmtPrinter::VisitUnaryExprOrTypeTraitExpr(WUnaryExprOrTypeTraitExpr *Node) {
  switch (Node->getKind()) {
    case UETT_SizeOf: OS << "sizeof"; break;
    case UETT_VecStep: OS << "vec_step"; break;
    default:
      llvm_unreachable("unknown case");
  }
  if (Node->isArgumentType()) {
    OS << '(';
    Node->getArgumentType().print(OS, Policy);
    OS << ')';
  } else {
    OS << ' ';
    PrintExpr(Node->getArgumentExpr());
  }
}

void WStmtPrinter::VisitArraySubscriptExpr(WArraySubscriptExpr *Node) {
  PrintExpr(Node->getLHS());
  OS << '[';
  PrintExpr(Node->getRHS());
  OS << ']';
}

void WStmtPrinter::VisitCallExpr(WCallExpr *Node) {
  PrintExpr(Node->getCallee());
  OS << '(';
  for (unsigned Index = 0, NumArgs = Node->getNumArgs();
       Index != NumArgs; ++Index) {
    if (Index) OS << ", ";
    PrintExpr(Node->getArg(Index));
  }
  OS << ')';
}

void WStmtPrinter::VisitMemberExpr(WMemberExpr *Node) {
  PrintExpr(Node->getBase());

  WMemberExpr *ParentMember = dyn_cast<WMemberExpr>(Node->getBase());
  FieldDecl *ParentDecl = (ParentMember ? ParentMember->getMemberDecl() : NULL);

  if (!ParentDecl || !ParentDecl->isAnonymousStructOrUnion())
    OS << (Node->isArrow() ? "->" : ".");

  if (Node->getMemberDecl()->isAnonymousStructOrUnion())
    return;

  if (Node->getOriginal()) {
    OS << Node->getOriginal()->getMemberNameInfo();
  } else {
    OS << Node->getMemberDecl()->getDeclName();
  }

  if (Node->getIndexedDefDecl() && Node->getIndexedUseDecl()) {
    OS << " /* ";
    Node->getIndexedDefDecl().printName(OS);
    OS << "<-";
    Node->getIndexedUseDecl().printName(OS);
    OS << " */";
  } else if (Node->getIndexedDefDecl()) {
    OS << " /* ";
    Node->getIndexedDefDecl().printName(OS);
    OS << " */";
  } else if (Node->getIndexedUseDecl()) {
    OS << " /* ";
    Node->getIndexedUseDecl().printName(OS);
    OS << " */";
  }
}

void WStmtPrinter::VisitCompoundLiteralExpr(WCompoundLiteralExpr *Node) {
  OS << '(';
  Node->getType().print(OS, Policy);
  OS << ')';
  PrintExpr(Node->getInitializer());
}

void WStmtPrinter::VisitImplicitCastExpr(WImplicitCastExpr *Node) {
  PrintExpr(Node->getSubExpr());
}

void WStmtPrinter::VisitCStyleCastExpr(WCStyleCastExpr *Node) {
  OS << '(';
  if (Node->getOriginal()) {
    Node->getTypeAsWritten().print(OS, Policy);
  } else {
    Node->getType().print(OS, Policy);
  }
  OS << ')';
  PrintExpr(Node->getSubExpr());
}

void WStmtPrinter::VisitBinaryOperator(WBinaryOperator *Node) {
  PrintExpr(Node->getLHS());
  OS << ' ' << BinaryOperator::getOpcodeStr(Node->getOpcode()) << ' ';
  PrintExpr(Node->getRHS());
}

void WStmtPrinter::VisitCompoundAssignOperator(WCompoundAssignOperator *Node) {
  PrintExpr(Node->getLHS());
  OS << ' ' << BinaryOperator::getOpcodeStr(Node->getOpcode()) << ' ';
  PrintExpr(Node->getRHS());
}

void WStmtPrinter::VisitConditionalOperator(WConditionalOperator *Node) {
  PrintExpr(Node->getCond());
  OS << " ? ";
  PrintExpr(Node->getLHS());
  OS << " : ";
  PrintExpr(Node->getRHS());
}

void WStmtPrinter::VisitInitListExpr(WInitListExpr *Node) {
  OS << "{ ";
  for (unsigned Index = 0, NumInits = Node->getNumInits();
       Index != NumInits; ++Index) {
    if (Index) OS << ", ";
    if (Node->getInit(Index)) {
      PrintExpr(Node->getInit(Index));
    } else {
      OS << '0';
    }
  }
  OS << " }";
}

void WStmtPrinter::VisitDesignatedInitExpr(WDesignatedInitExpr *Node) {
  for (DesignatedInitExpr::designators_iterator D = Node->designators_begin(),
                                                DEnd = Node->designators_end();
       D != DEnd; ++D) {
    if (D->isFieldDesignator()) {
      if (D->getDotLoc().isInvalid()) {
        OS << D->getFieldName()->getName() << ':';
      } else {
        OS << '.' << D->getFieldName()->getName();
      }
    } else {
      assert(D->isArrayDesignator());
      OS << '[';
      PrintExpr(Node->getArrayIndex(*D));
      OS << ']';
    }
  }
  OS << " = ";
  PrintExpr(Node->getInit());
}

void WStmtPrinter::VisitImplicitValueInitExpr(WImplicitValueInitExpr *Node) {
  OS << "/*implicit*/(";
  Node->getType().print(OS, Policy);
  OS << ')';
  if (Node->getType()->isRecordType())
    OS << "{}";
  else
    OS << '0';
}

void WStmtPrinter::VisitParenListExpr(WParenListExpr *Node) {
  OS << "( ";
  for (unsigned Index = 0, NumExprs = Node->getNumExprs();
       Index != NumExprs; ++Index) {
    if (Index) OS << ", ";
    PrintExpr(Node->getExpr(Index));
  }
  OS << " )";
}

void WStmtPrinter::VisitExtVectorElementExpr(WExtVectorElementExpr *Node) {
  PrintExpr(Node->getBase());
  OS << '.' << Node->getAccessor().getName();
  if (Node->getIndexedDefDecl() && Node->getIndexedUseDecl()) {
    OS << " /* ";
    Node->getIndexedDefDecl().printName(OS);
    OS << "<-";
    Node->getIndexedUseDecl().printName(OS);
    OS << " */";
  } else if (Node->getIndexedDefDecl()) {
    OS << " /* ";
    Node->getIndexedDefDecl().printName(OS);
    OS << " */";
  } else if (Node->getIndexedUseDecl()) {
    OS << " /* ";
    Node->getIndexedUseDecl().printName(OS);
    OS << " */";
  }
}

void WStmtPrinter::VisitCXXBoolLiteralExpr(WCXXBoolLiteralExpr *Node) {
  OS << (Node->getValue() ? "true" : "false");
}

void WStmtPrinter::VisitCXXDefaultArgExpr(WCXXDefaultArgExpr *Node) {
  // Nothing to print: we picked up the default argument.
}

void WStmtPrinter::VisitCXXConstructExpr(WCXXConstructExpr *Node) {
  if (Node->getOriginal()->isListInitialization()) {
    OS << "{ ";
  }
  for (unsigned Index = 0, NumArgs = Node->getNumArgs();
       Index != NumArgs; ++Index) {
    if (isa<WCXXDefaultArgExpr>(Node->getArg(Index))) {
      break;
    }
    if (Index) OS << ", ";
    PrintExpr(Node->getArg(Index));
  }
  if (Node->getOriginal()->isListInitialization()) {
    OS << " }";
  }
}

void WStmtPrinter::VisitBlockExpr(WBlockExpr *Node) {
  OS << "<block expr>";
}

void WStmtPrinter::VisitAsTypeExpr(WAsTypeExpr *Node) {
  OS << "as_";
  Node->getType().print(OS, Policy);
  OS << '(';
  PrintExpr(Node->getSrcExpr());
  OS << ')';
}

void WStmtPrinter::VisitPhiFunction(WPhiFunction *Node) {
  OS << "/* ";
  if (Node->getIndexedLHSDecl()) {
    Node->getIndexedLHSDecl()->printName(OS);
  } else {
    OS << "<unknown>";
  }
  OS << " = phi(";
  for (unsigned Index = 0, NumArgs = Node->getNumArgs();
       Index != NumArgs; ++Index) {
    if (Index) OS << ", ";
    if (Node->getIndexedArgDecl(Index)) {
      Node->getIndexedArgDecl(Index)->printName(OS);
    } else {
      OS << "<unknown>";
    }
  }
  OS << ") */";
}

void WStmtPrinter::VisitWorkItemFunction(WWorkItemFunction *Node) {
  switch (Node->getFunctionKind()) {
    case WWorkItemFunction::WIF_get_global_size:
      OS << "get_global_size(" << Node->getArg() << ')';
      break;
    case WWorkItemFunction::WIF_get_global_id:
      OS << "get_global_id(" << Node->getArg() << ')';
      break;
    case WWorkItemFunction::WIF_get_local_size:
      OS << "get_local_size(" << Node->getArg() << ')';
      break;
    case WWorkItemFunction::WIF_get_local_id:
      OS << "get_local_id(" << Node->getArg() << ')';
      break;
    case WWorkItemFunction::WIF_get_num_groups:
      OS << "get_num_groups(" << Node->getArg() << ')';
      break;
    case WWorkItemFunction::WIF_get_group_id:
      OS << "get_group_id(" << Node->getArg() << ')';
      break;
  }
}

void WStmtPrinter::VisitBaseMuFunction(WBaseMuFunction *Node) {
  OS << "/* mu(...) */";
}

void WStmtPrinter::VisitAuxiliaryMuFunction(WAuxiliaryMuFunction *Node) {
  OS << "/* mu(...) */";
}

} // anonymous namespace

void WStmt::print(raw_ostream &OS, const PrintingPolicy &Policy) const {
  WStmtPrinter P(OS, Policy);
  P.Visit(const_cast<WStmt*>(this));
}

void WVarDecl::printName(raw_ostream &OS) const {
  OS << getName();
}

void IndexedVarDecl::printName(raw_ostream &OS) const {
  Decl->printName(OS);
  OS << '_' << Index;
}

void IndexedVarDeclRef::printName(raw_ostream &OS) const {
  if (isNull()) {
    OS << "<null>";
  } else if (isSingleVar()) {
    getSingleVar()->printName(OS);
  } else {
    assert(isCompound());
    for (unsigned Index = 0; Index < NumSubVars; ++Index) {
      if (Index) OS << ", ";
      getSubVar(Index)->printName(OS);
    }
  }
}

} // namespace snu

} // namespace clang
