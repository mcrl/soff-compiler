add_clang_library(clangSnuOptimization
  PointerRemoval.cpp
  )

add_dependencies(clangSnuOptimization
  ClangStmtNodes
  )

target_link_libraries(clangSnuOptimization
  clangAST
  clangAnalysis
  clangBasic
  clangSnuAST
  clangSnuAnalysis
  )
