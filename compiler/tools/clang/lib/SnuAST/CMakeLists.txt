add_clang_library(clangSnuAST
  CFGAnalysis.cpp
  WAST.cpp
  WASTPrinter.cpp
  WCFG.cpp
  )

add_dependencies(clangSnuAST
  ClangStmtNodes
  )

target_link_libraries(clangSnuAST
  clangAST
  clangAnalysis
  clangBasic
  )
