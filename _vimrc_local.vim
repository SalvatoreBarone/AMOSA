let g:syntastic_cpp_config_file = '.syntastic_config'
let g:syntastic_c_config_file = '.syntastic_config'
let g:syntastic_c_compiler_options = '-O0 -Wall -Wextra -pedantic -ggdb3 --std=c++11'
let g:syntastic_cpp_compiler_options = '-O0 -Wall -Wextra -pedantic -ggdb3 --std=c++11'
let g:clang_compilation_database = './build'
let g:clang_c_options = '-O0 -Wall -Wextra -pedantic -ggdb3 --std=c++11'
let g:clang_cpp_options = '-O0 -Wall -Wextra -pedantic -ggdb3 --std=c++11'

set colorcolumn=121
highlight ColorColumn ctermbg=darkgray
