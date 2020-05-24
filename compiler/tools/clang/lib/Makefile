##===- lib/Makefile ----------------------------------------*- Makefile -*-===##
# 
#                     The LLVM Compiler Infrastructure
#
# This file is distributed under the University of Illinois Open Source
# License. See LICENSE.TXT for details.
# 
##===----------------------------------------------------------------------===##
CLANG_LEVEL := ..

# ARCMigrate and Rewrite are always needed because of libclang.
PARALLEL_DIRS = Headers Basic Lex Parse AST Sema CodeGen Analysis Frontend \
                FrontendTool Tooling Driver Format Edit Rewrite Serialization \
                Index

# SnuCL Compiler
PARALLEL_DIRS += SnuPreprocess
PARALLEL_DIRS += SnuAST
PARALLEL_DIRS += SnuAnalysis
PARALLEL_DIRS += SnuOptimization
PARALLEL_DIRS += SnuSynthesis
PARALLEL_DIRS += SnuFrontend

include $(CLANG_LEVEL)/../../Makefile.config

ifeq ($(ENABLE_CLANG_REWRITER),1)
PARALLEL_DIRS += ASTMatchers
endif

ifeq ($(ENABLE_CLANG_STATIC_ANALYZER),1)
PARALLEL_DIRS += StaticAnalyzer
endif

ifeq ($(ENABLE_CLANG_ARCMT),1)
PARALLEL_DIRS += ARCMigrate
endif

include $(CLANG_LEVEL)/Makefile
