from __future__ import print_function
import sys

import clang.cindex
from clang.cindex import Index
from clang.cindex import Config

file_names = []
header_string = ''

def print_c_header(fl = sys.stderr):
    print('// auto generated file', file=fl)
    print('// md5sum: %s'%(header_string), file=fl)
    print('#include <string>', file=fl)
    print('#include <iostream>', file=fl)
    print('#include <vector>', file=fl)
    print('', file=fl)
    print('extern "C" {', file=fl)
    print('#include "shm_common.c"', file=fl)
    print('#include "../include/wheel_shm.h"', file=fl)
    print('}', file=fl)
    print('', file=fl)
    print('int checkrange(int i, int min, int max) { // check int range', file=fl)
    print('  if ( i < min ) {', file=fl)
    print('    std::cerr << "index out of range " << i << " < " << min << std::endl;', file=fl)
    print('    i = min;', file=fl)
    print('  } else if ( i > max ) {', file=fl)
    print('    std::cerr << "index out of range " << i << " > " << max << std::endl;', file=fl)
    print('    i = max;', file=fl)
    print('  }', file=fl)
    print('  return i; }', file=fl)
    print('', file=fl)

def print_c_write_main(name, type_name, size_list, fl = sys.stderr):
    print('typedef %s current_type;'%(type_name), file=fl)
    print('int main(int argc, char **argv) {', file=fl)
    print('  bool debug = false;', file=fl)
    print('  std::vector<int> args;', file=fl)
    print('  std::vector<current_type> vals;', file=fl)
    print('', file=fl)
    print('  for(int i = 1; i < argc; i++) {', file=fl)
    print('    std::string arg(argv[i]);', file=fl)
    print('    if(arg == "--help") {', file=fl)
    print('      std::cerr << "write_command: " << argv[0];', file=fl)
    print('      std::cerr << " [--debug] [--help] ";', file=fl)
    if len(size_list) == 0:
        print('      std::cerr << " arg";', file=fl)
    if len(size_list) > 1:
        print('      std::cerr << " i";', file=fl)
    if len(size_list) > 0:
        print('      std::cerr << " n";', file=fl)
        print('      std::cerr << " arg_0 ... arg_%s //";'%(size_list[0]-1), file=fl)
        if len(size_list) > 1:
            print('      std::cerr << " 0 <= i <= %d";'%(size_list[1]), file=fl)
        print('      std::cerr << " 0 <= n <= %d";'%(size_list[0]), file=fl)
    print('      std::cerr << std::endl;', file=fl)
    print('      exit(-1);', file=fl)
    print('    } else if(arg == "--debug") {', file=fl)
    print('      debug = true;', file=fl)
    print('    } else if(args.size() < %s) {'%(len(size_list)), file=fl)
    print('      long d = std::stol(arg, nullptr, 0);', file=fl)
    print('      args.push_back(d);', file=fl)
    print('    } else {', file=fl)
    if type_name == 'float' or type_name == 'double':
        print('      double d = std::stod(arg);', file=fl)
    else:
        print('      long d = std::stol(arg, nullptr, 0);', file=fl)
    print('      current_type val = static_cast<current_type>(d);', file=fl)
    print('      vals.push_back(val);', file=fl)
    print('    }', file=fl)
    print('  }', file=fl)

    print('    struct servo_shm *shm = (struct servo_shm *)shm_alloc(5555, sizeof(struct servo_shm));', file=fl)

    if len(size_list) == 0:
        pass
    elif len(size_list) == 1:
        print('  int val_n = 0;', file=fl)
        print('  if (args.size() > 0) val_n = checkrange(args[0], 0, %d);'%(size_list[0]), file=fl)
    elif len(size_list) == 2:
        print('  int val_i = 0;', file=fl)
        print('  int val_n = 0;', file=fl)
        print('  if (args.size() > 1) {', file=fl)
        print('    val_i = checkrange(args[0], 0, %d); }'%(size_list[1]), file=fl)
        print('    val_n = checkrange(args[1], 0, %d);'%(size_list[0]), file=fl)

    else:
        # error
        pass

    if len(size_list) == 0:
        print('  if (vals.size() > 0) {', file=fl)
        print('    shm->%s = vals[0];'%(name), file=fl)
        print('    if(debug) std::cerr << "write[%s]: " << shm->%s << std::endl;'%(name, name), file=fl)
        print('  }', file=fl)
    elif len(size_list) > 0:
        print('  if (debug) std::cerr << "write[%s]: ";'%(name), file=fl)
        print('  for(int n = 0; (n < val_n) && (n < vals.size()); n ++) {', file=fl)
        if len(size_list) == 1:
            print('    shm->%s[n] = vals[n];'%(name), file=fl)
            print('    if(debug) std::cerr << shm->%s[n] << " " << std::endl;'%(name), file=fl)
        elif len(size_list) == 2:
            print('    shm->%s[val_i][n] = vals[n];'%(name), file=fl)
            print('    if(debug) std::cerr << shm->%s[val_i][n] << " " << std::endl;'%(name), file=fl)
        print('  }', file=fl)
        print('  if (debug) std::cerr << std::endl;', file=fl)
    print('}', file=fl)

def print_c_read_main(name, type_name, size_list, fl = sys.stderr):
    print('typedef %s current_type;'%(type_name), file=fl)
    print('int main(int argc, char **argv) {', file=fl)
    print('  bool debug = false;', file=fl)
    print('  std::vector<int> args;', file=fl)
    print('', file=fl)
    print('  for(int i = 1; i < argc; i++) {', file=fl)
    print('    std::string arg(argv[i]);', file=fl)
    print('    if(arg == "--help") {', file=fl)
    print('      std::cerr << "read_command: " << argv[0];', file=fl)
    print('      std::cerr << " [--debug] [--help] ";', file=fl)
    if len(size_list) > 1:
        print('      std::cerr << " i";', file=fl)
    if len(size_list) > 0:
        print('      std::cerr << " n //";', file=fl)
        if len(size_list) > 1:
            print('      std::cerr << " 0 <= i <= %d";'%(size_list[1]), file=fl)
        print('      std::cerr << " 0 <= n <= %d";'%(size_list[0]), file=fl)
    print('      std::cerr << std::endl;', file=fl)
    print('      exit(-1);', file=fl)
    print('    } else if(arg == "--debug") {', file=fl)
    print('      debug = true;', file=fl)
    print('    } else if(args.size() < %s) {'%(len(size_list)), file=fl)
    print('      long d = std::stol(arg, nullptr, 0);', file=fl)
    print('      args.push_back(d);', file=fl)
    print('    }', file=fl)
    print('  }', file=fl)

    print('    struct servo_shm *shm = (struct servo_shm *)shm_alloc(5555, sizeof(struct servo_shm));', file=fl)

    if len(size_list) == 0:
        pass
    elif len(size_list) == 1:
        print('  int val_n = %d;'%(size_list[0]), file=fl)
        print('  if (args.size() > 0) val_n = checkrange(args[0], 0, %d);'%(size_list[0]), file=fl)
    elif len(size_list) == 2:
        print('  int val_i = 0;', file=fl)
        print('  int val_n = %d;'%(size_list[0]), file=fl)
        print('  if (args.size() > 1) {', file=fl)
        print('    val_i = checkrange(args[0], 0, %d); }'%(size_list[1]), file=fl)
        print('    val_n = checkrange(args[1], 0, %d);'%(size_list[0]), file=fl)

    else:
        # error
        pass

    if len(size_list) == 0:
        print('  std::cout << shm->%s << std::endl;'%(name), file=fl)
    elif len(size_list) > 0:
        print('  for(int n = 0; n < val_n; n ++) {', file=fl)
        if len(size_list) == 1:
            print('    std::cout << shm->%s[n] << " ";'%(name), file=fl)
        elif len(size_list) == 2:
            print('    std::cout << shm->%s[val_i][n] << " ";'%(name), file=fl)
        print('  }', file=fl)
        print('  std::cout << std::endl;', file=fl)
    print('}', file=fl)

def print_c_function(name, type_name, size_list):
    ## open file
    with open('read_%s.cpp'%(name), mode='w') as fl:
        print_c_header(fl)
        print_c_read_main(name, type_name, size_list, fl = fl)
        file_names.append('read_%s'%(name))

    with open('write_%s.cpp'%(name), mode='w') as fl:
        print_c_header(fl)
        print_c_write_main(name, type_name, size_list, fl = fl)
        file_names.append('write_%s'%(name))

    return True

def parse_field_decl(node):
    if node.kind == clang.cindex.CursorKind.FIELD_DECL:
        sz_lst = []
        tpname = node.type.spelling
        sz = node.type.get_array_size()
        tp = node.type.get_array_element_type()
        while sz > 0:
            sz_lst.append(sz)
            sz = tp.get_array_size()
            if sz < 1:
                tpname = tp.spelling
            tp = tp.get_array_element_type()
        print((node.spelling, tpname, sz_lst))

        print_c_function(node.spelling, tpname, sz_lst)

    for child in node.get_children():
        parse_field_decl(child)

def parse_struct_decl(node):
    if node.kind != clang.cindex.CursorKind.STRUCT_DECL:
        return
    name = node.type.get_canonical().spelling
    print('// struct name => %s'%(name))

    parse_field_decl(node)

def print_struct_recur(start_file, depth, node):
    if str(node.extent.start.file) == start_file:
        parse_struct_decl(node)
    for child in node.get_children():
        print_struct_recur(start_file, depth+1, child)

def print_makefile(file_names):
    with open('Makefile.gen_command_line', mode='w') as fl:
        print('TARGETS = %s'%(' '.join(file_names)), file=fl)
        print('SRCS = $(addsuffix .cpp, $(TARGETS))', file=fl)
        print('all: $(TARGETS)', file=fl)
        print('', file=fl)
        print('%: %.cpp', file=fl)
        print('	g++ -std=c++11 -o $@ -O2 $<', file=fl)
        print('', file=fl)
        print('.PHONY: clean', file=fl)
        print('clean:', file=fl)
        print('	rm -rf $(TARGETS)', file=fl)
        print('', file=fl)
        print('.PHONY: clean_all', file=fl)
        print('clean_all: clean', file=fl)
        print('	rm -rf $(SRCS)', file=fl)


if __name__ == '__main__':
    target_file = None
    import sys
    if len(sys.argv) > 1:
        target_file = sys.argv[-1]
    # target_file = '../include/wheel_shm.h'
    import os
    if not os.path.isfile(target_file):
        raise()

    index = Index.create()
    tu    = index.parse(target_file)

    import subprocess
    header_string = subprocess.check_output(['md5sum', target_file])

    print_struct_recur(target_file, 0, tu.cursor)

    #print(file_names)
    print_makefile(file_names)
