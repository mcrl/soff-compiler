add_clang_library(clangSnuAnalysis
  Dominator.cpp
  Invariance.cpp
  LiveVariables.cpp
  Loop.cpp
  MemoryAccess.cpp
  PointerAnalysis.cpp
  )

add_dependencies(clangSnuAnalysis
  ClangStmtNodes
  )

target_link_libraries(clangSnuAnalysis
  clangAST
  clangAnalysis
  clangBasic
  clangSnuAST
  )
