add_clang_library(clangSnuFrontend
  FrontendActions.cpp
  Options.cpp
  )

add_dependencies(clangSnuFrontend
  ClangStmtNodes
  )

target_link_libraries(clangSnuFrontend
  clangAST
  clangAnalysis
  clangBasic
  clangSnuSynthesis
  clangSnuAnalysis
  clangSnuAST
  clangSnuPreprocess
  )
