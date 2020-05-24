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

#include "clang/SnuFrontend/FrontendActions.h"
#include "clang/AST/ASTConsumer.h"
#include "clang/AST/ASTContext.h"
#include "clang/AST/Attr.h"
#include "clang/AST/Decl.h"
#include "clang/AST/PrettyPrinter.h"
#include "clang/AST/Stmt.h"
#include "clang/Analysis/CFG.h"
#include "clang/Basic/Diagnostic.h"
#include "clang/Basic/LangOptions.h"
#include "clang/Basic/LLVM.h"
#include "clang/Driver/DriverDiagnostic.h"
#include "clang/Frontend/ASTConsumers.h"
#include "clang/Frontend/CodeGenOptions.h"
#include "clang/Frontend/CompilerInstance.h"
#include "clang/Frontend/FrontendAction.h"
#include "clang/Frontend/FrontendDiagnostic.h"
#include "clang/Frontend/FrontendOptions.h"
#include "clang/SnuAST/WAST.h"
#include "clang/SnuAST/WCFG.h"
#include "clang/SnuAnalysis/MemoryAccess.h"
#include "clang/SnuAnalysis/PointerAnalysis.h"
#include "clang/SnuFrontend/Options.h"
#include "clang/SnuOptimization/PointerRemoval.h"
#include "clang/SnuPreprocess/FunctionNormalizer.h"
#include "clang/SnuSynthesis/CodeGenerator.h"
#include "clang/SnuSynthesis/ControlFlowConstraint.h"
#include "clang/SnuSynthesis/DataflowGraph.h"
#include "clang/SnuSynthesis/PlatformContext.h"
#include "clang/SnuSynthesis/Scheduler.h"
#include "clang/SnuSynthesis/StructuralAnalysis.h"
#include "clang/SnuSynthesis/Verilog.h"
#include "clang/SnuSynthesis/VirtualVariables.h"
#include "llvm/ADT/OwningPtr.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/Support/FileSystem.h"
#include "llvm/Support/raw_ostream.h"

using namespace clang;

namespace clang {

namespace snu {

// AST Consumers

class IdleConsumer : public ASTConsumer {
  std::string InFile;
  DiagnosticsEngine &Diags;
  const LangOptions &LangOpts;
  const CodeGenOptions &CodeGenOpts;

public:
  IdleConsumer(const std::string &infile,
               DiagnosticsEngine &diags,
               const LangOptions &langopts,
               const CodeGenOptions &codegenopts) :
    InFile(infile),
    Diags(diags),
    LangOpts(langopts),
    CodeGenOpts(codegenopts) {}

  void Initialize(ASTContext &Ctx);
  void HandleTranslationUnit(ASTContext &Ctx);
  void HandleKernel(FunctionDecl *FD, ASTContext &Ctx);
};

void IdleConsumer::Initialize(ASTContext &Ctx) {
}

void IdleConsumer::HandleTranslationUnit(ASTContext &Ctx) {
  if (Diags.hasErrorOccurred())
    return;

  TranslationUnitDecl *TU = Ctx.getTranslationUnitDecl();
  {
    FunctionNormalizer Normalizer(Ctx);
    Normalizer.EnableNormalizeDecl();
    Normalizer.EnableNormalizeVarDeclInit();
    Normalizer.EnableInlineAll();
    Normalizer.EnableKernelOnly();
    Normalizer.NormalizeAll(TU);
  }

  for (DeclContext::decl_iterator D = TU->decls_begin(), DEnd = TU->decls_end();
       D != DEnd; ++D) {
    if (FunctionDecl *FD = dyn_cast<FunctionDecl>(*D)) {
      if (FD->hasBody() && FD->hasAttr<OpenCLKernelAttr>()) {
        HandleKernel(FD, Ctx);
      }
    }
  }
}

void IdleConsumer::HandleKernel(FunctionDecl *FD, ASTContext &Ctx) {
  assert(FD->hasBody() && FD->hasAttr<OpenCLKernelAttr>());

  Stmt *Body = FD->getBody();
  CFG::BuildOptions BO;
  CFG *cfg = CFG::buildCFG(NULL, FD->getBody(), &Ctx, BO);

  WDeclContext DeclCtx;
  WStmt *WBody = WStmt::WrapClangAST(Ctx, DeclCtx, Body);
  WCFG *Wcfg = WCFG::WrapClangCFG(DeclCtx, WBody, cfg);
  Wcfg->MakeSSAForm();

  Wcfg->print(llvm::outs(), LangOpts);
}

class SnuCLFCompilerConsumer : public ASTConsumer {
  CompilerInstance &CI;
  std::string InFile;
  DiagnosticsEngine &Diags;
  const LangOptions &LangOpts;
  const CodeGenOptions &CodeGenOpts;
  const SnuCLOptions &SnuCLOpts;

public:
  SnuCLFCompilerConsumer(CompilerInstance &ci,
                         const std::string &infile,
                         DiagnosticsEngine &diags,
                         const LangOptions &langopts,
                         const CodeGenOptions &codegenopts,
                         const SnuCLOptions &snuclopts) :
    CI(ci),
    InFile(infile),
    Diags(diags),
    LangOpts(langopts),
    CodeGenOpts(codegenopts),
    SnuCLOpts(snuclopts) {}

  void Initialize(ASTContext &Ctx);
  void HandleTranslationUnit(ASTContext &Ctx);
  void HandleKernel(FunctionDecl *FD, ASTContext &Ctx, PlatformContext &PCtx,
                    WFPCodeGenerator &CodeGen);
  raw_ostream *CreateOutputFile(StringRef Postfix);
};

void SnuCLFCompilerConsumer::Initialize(ASTContext &Ctx) {
}

void SnuCLFCompilerConsumer::HandleTranslationUnit(ASTContext &Ctx) {
  if (Diags.hasErrorOccurred())
    return;

  TranslationUnitDecl *TU = Ctx.getTranslationUnitDecl();
  {
    FunctionNormalizer Normalizer(Ctx);
    Normalizer.EnableInlineAll();
    Normalizer.EnableKernelOnly();
    Normalizer.NormalizeAll(TU);
  }

  PlatformVariables PVars(SnuCLOpts.TargetFile);
  PlatformIPCollection PIPs(Ctx, PVars);
  WFPCodeGenerator CodeGen(Ctx, SnuCLOpts);
  if (!SnuCLOpts.Prefix.empty()) {
    if (llvm::sys::fs::create_directory(SnuCLOpts.Prefix) != llvm::errc::success) {
      Diags.Report(diag::err_drv_no_such_file) << SnuCLOpts.Prefix;
      return;
    }
  }
  for (DeclContext::decl_iterator D = TU->decls_begin(), DEnd = TU->decls_end();
       D != DEnd; ++D) {
    if (FunctionDecl *FD = dyn_cast<FunctionDecl>(*D)) {
      if (FD->hasBody() &&
          (FD->hasAttr<OpenCLKernelAttr>() || FD->hasAttr<CUDAGlobalAttr>())) {
        PlatformContext PCtx(Ctx, PVars, PIPs, FD);
        HandleKernel(FD, Ctx, PCtx, CodeGen);
        if (raw_ostream *OS = CreateOutputFile(FD->getNameAsString() + ".v")) {
          CodeGen.ExportSingleKernelFile(FD)->print(*OS);
        }
      }
    }
  }
  if (raw_ostream *OS = CreateOutputFile("__all__.v")) {
    CodeGen.ExportAllKernelFile()->print(*OS);
  }
  if (raw_ostream *OS = CreateOutputFile("__meta__.json")) {
    CodeGen.PrintMetadata(*OS);
  }
}

void SnuCLFCompilerConsumer::HandleKernel(FunctionDecl *FD, ASTContext &Ctx,
                                          PlatformContext &PCtx,
                                          WFPCodeGenerator &CodeGen) {
  assert(FD->hasBody());
  assert(FD->hasAttr<OpenCLKernelAttr>() || FD->hasAttr<CUDAGlobalAttr>());

  if (SnuCLOpts.isVerbose()) {
    llvm::outs() << "====================\n";
    llvm::outs() << "Compiling " << FD->getName() << "()...\n\n";
  }

  Stmt *Body = FD->getBody();
  CFG::BuildOptions BO;
  CFG *cfg = CFG::buildCFG(NULL, FD->getBody(), &Ctx, BO);

  WDeclContext DeclCtx;
  WStmt *WBody = WStmt::WrapClangAST(Ctx, DeclCtx, Body);
  WCFG *Wcfg = WCFG::WrapClangCFG(DeclCtx, WBody, cfg);
  Wcfg->MakeSSAForm();

  if (SnuCLOpts.VerboseCFG) {
    llvm::outs() << "--------------------\n";
    llvm::outs() << "Control flow graph:\n\n";
    Wcfg->print(llvm::outs(), LangOpts);
    llvm::outs() << "\n";
  }

  // Optimizations: CFG -> CFG
  if (SnuCLOpts.PointerRemoval) {
    PointerRemoval Opt(Ctx, Wcfg);
    if (SnuCLOpts.VerboseOpt) Opt.setVerbose();
    Opt.Run();
  }

  if (SnuCLOpts.VerboseCFG) {
    llvm::outs() << "--------------------\n";
    llvm::outs() << "Control flow graph after applying optimizations:\n\n";
    Wcfg->print(llvm::outs(), LangOpts);
    llvm::outs() << "\n";
  }

  AliasSet Aliases(Wcfg);
  if (SnuCLOpts.VerboseAlias) {
    llvm::outs() << "--------------------\n";
    llvm::outs() << "Alias analysis:\n\n";
    Aliases.print(llvm::outs(), Ctx);
    llvm::outs() << "\n";
  }

  ControlDataflowGraph *cdfg = ControlDataflowGraph::MakeCDFG(
      Wcfg, Ctx, SnuCLOpts, Aliases);
  ControlTreeNode *CT = ControlTreeNode::Build(cdfg);

  DefaultScheduler Scheduler(PCtx, Ctx);
  Scheduler.ScheduleAll(cdfg);

  ControlFlowConstraintContext CFCCtx(PCtx);
  { DeadlockPrevention CFC(CFCCtx, CT, cdfg); CFC.Synthesis(); }
  { BarrierOrdering CFC(CFCCtx, CT); CFC.Synthesis(); }
  { LocalMemoryOrdering CFC(CFCCtx, CT); CFC.Synthesis(); }

  CodeGen.HandleKernel(FD, cdfg, CT, DeclCtx, Aliases, PCtx);

  if (raw_ostream *OS = CreateOutputFile(FD->getNameAsString() + "_cdfg.dot")) {
    cdfg->print(*OS);
  }
}

raw_ostream *SnuCLFCompilerConsumer::CreateOutputFile(StringRef Postfix) {
  std::string OutFile;
  if (SnuCLOpts.Prefix.empty()) {
    OutFile += InFile;
    OutFile += "__";
  } else {
    OutFile += SnuCLOpts.Prefix;
  }
  OutFile += Postfix;
  return CI.createOutputFile(OutFile, false, false, InFile, "", false);
}

// Frontend Actions

ASTConsumer *IdleAction::CreateASTConsumer(CompilerInstance &CI,
                                           StringRef InFile) {
  return new IdleConsumer(InFile, CI.getDiagnostics(), CI.getLangOpts(),
                          CI.getCodeGenOpts());
}

ASTConsumer *SnuCLFCompilerAction::CreateASTConsumer(CompilerInstance &CI,
                                                     StringRef InFile) {
  return new SnuCLFCompilerConsumer(CI, InFile, CI.getDiagnostics(),
                                    CI.getLangOpts(), CI.getCodeGenOpts(),
                                    SnuCLOpts);
}


// Driver API

bool ExecuteSnuCLCompilerInvocation(CompilerInstance *Clang,
                                    const SnuCLOptions &Opts) {
  if (Clang->getFrontendOpts().ShowHelp ||
      Clang->getFrontendOpts().ShowVersion) {
    llvm::outs() << "SnuCL Compiler: http://snucl.snu.ac.kr\n";
    return true;
  }
  if (Clang->getDiagnostics().hasErrorOccurred())
    return false;
  OwningPtr<FrontendAction> Act(new SnuCLFCompilerAction(Opts));
  if (!Act)
    return false;
  bool Success = Clang->ExecuteAction(*Act);
  if (Clang->getFrontendOpts().DisableFree)
    Act.take();
  return Success;
}

} // namespace snu

} // namespace clang
