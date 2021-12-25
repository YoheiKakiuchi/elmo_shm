from __future__ import print_function
import sys

import clang.cindex
from clang.cindex import Index
from clang.cindex import Config

argmap = {'int'   :  ':integer',
          'short' :  ':integer',
          'char'  :  ':integer',
          'long'  :  ':integer',
          'unsigned int'   : ':integer',
          'unsigned short' : ':integer',
          'unsigned char'  : ':integer',
          'unsigned long'  : ':integer',
          'int *'    : ':string',
          'short *'  : ':string',
          'char *'   : ':string',
          'long *'   : ':string',
          'float *'  : ':string',
          'double *' : ':string',
          'void *'   : ':string',
          'void'   : '',
          'float'  : ':float32',
          'double' : ':float', ## double???
          }

c_file = None
eus_file = None

def print_c_header(target_file, fl = sys.stderr):
    print('// auto generated file', file=fl)
    print('// md5sum: %s'%(header_string), file=fl)
    print('#include "%s"'%(target_file), file=fl)
    print('#include <stdio.h>', file=fl)
    print('#include "shm_common.c"', file=fl)
    print('struct servo_shm *shm;', file=fl) ## HOTFIX struct name
    ### for shm
    #print('void *shmalloc(key_t _key, size_t _size) {', file=fl)
    #print('  int shm_id; void *ptr; int err;', file=fl)
    #print('  shm_id=shmget(_key, _size, 0666|IPC_CREAT);', file=fl)
    #print('  err = errno;', file=fl)
    #print('  if(shm_id==-1) {', file=fl)
    #print('    fprintf(stderr, "shmget failed, key=%d, size=%d, errno=%d\\n", _key, (int)_size, err);', file=fl)
    #print('    return NULL; }', file=fl)
    #print('  ptr=(struct shared_data *)shmat(shm_id, (void *)0, 0);', file=fl)
    #print('  if(ptr==(void *)-1) {', file=fl)
    #print('    int err=errno;', file=fl)
    #print('    fprintf(stderr, "shmat failed, key=%d, size=%d, shm_id=%d, errno=%d\\n", _key, (int)_size, shm_id, err);', file=fl)
    #print('    return NULL; }', file=fl)
    #print('  return ptr; }', file=fl)
    ##
    print('int initialize_sharedmemory(int skey) {', file=fl)
    print('  shm = (struct servo_shm *)shm_alloc(skey, sizeof(struct servo_shm));', file=fl)
    print('  return 0; }', file=fl)
    ##
    print('int checkrange(int i, int min, int max) { // check int range', file=fl)
    print('  if ( i < min ) {', file=fl)
    print('    fprintf(stderr, "index out of range %d < %d\\n", i, min);', file=fl)
    print('    i = min;', file=fl)
    print('  } else if ( i > max ) {', file=fl)
    print('    fprintf(stderr, "index out of range %d > %d\\n", i, max);', file=fl)
    print('    i = max;', file=fl)
    print('  }', file=fl)
    print('  return i; }', file=fl)

    print('int copy_float2double ( int n, float* src, double *dst ) {', file=fl)
    print('  for (int i = 0; i < n; i++ ) dst[i] = src[i];', file=fl)
    print('  return n; }', file=fl)

    print('int copy_double2float ( int n, double* src, float *dst ) {', file=fl)
    print('  for (int i = 0; i < n; i++ ) dst[i] = src[i];', file=fl)
    print('  return n; }', file=fl)

    for tp in ('char', 'short', 'int'):
        print('int copy_long2%s ( int n, long* src, %s *dst ) {'%(tp, tp), file=fl)
        print('  for (int i = 0; i < n; i++ ) dst[i] = src[i];', file=fl)
        print('  return n; }', file=fl)
        print('int copy_%s2long ( int n, %s* src, long *dst ) {'%(tp, tp), file=fl)
        print('  for (int i = 0; i < n; i++ ) dst[i] = src[i];', file=fl)
        print('  return n; }', file=fl)

def print_c_footer(fl = sys.stderr):
    print('// for compile', file=fl)
    print('// gcc -fPIC -O3 -c libeus_shm.c', file=fl)
    print('// gcc -o libeus_shm.so -shared libeus_shm.o', file=fl)

def print_eus_header(target_file, fl = sys.stderr):
    print(';; auto generated file', file=fl)
    print(';; md5sum: %s'%(header_string), file=fl)
    print('(if (not (find-package "EUS_SHM")) (make-package "EUS_SHM"))', file=fl)
    print('(In-package "EUS_SHM")', file=fl)
    print('(let ((lib (load-foreign "libeus_shm.so")))', file=fl)
    print('  (defforeign _initialize-sharedmemory lib "initialize_sharedmemory" (:integer) :integer)', file=fl)
    print('  (defun initialize-sharedmemory (&optional (id 5555)) (_initialize-sharedmemory id))', file=fl)
    #print(')'

def print_eus_footer(fl = sys.stderr):
    print('  )', file=fl)
    print('(In-package "USER")', file=fl)

def print_eus_function(name, type_name, size_list, fl = sys.stderr):
    if len(size_list) == 0:
        if type_name in argmap:
            r = argmap[type_name]
        else:
            return False
        if r == ':float32':
            r = ':float'
        print ('  (defforeign read-%s  lib "read_%s"  () %s)'%(name, name, r), file=fl)
        print ('  (defforeign write-%s lib "write_%s" (%s) %s)'%(name, name, r, r), file=fl)
    elif len(size_list) == 1:
        if type_name in argmap:
            r = argmap[type_name]
        else:
            return False
        print ('  (defforeign read-%s  lib "read_%s"  (:integer :string) :integer)'%(name, name), file=fl)
        print ('  (defforeign write-%s lib "write_%s" (:integer :string) :integer)'%(name, name), file=fl)
    elif len(size_list) == 2:
        if type_name in argmap:
            r = argmap[type_name]
        else:
            return False
        print ('  (defforeign read-%s  lib "read_%s"  (:integer :integer :string) :integer)'%(name, name), file=fl)
        print ('  (defforeign write-%s lib "write_%s" (:integer :integer :string) :integer)'%(name, name), file=fl)

def print_c_function(name, type_name, size_list, fl = sys.stderr):
    if len(size_list) == 0:
        if type_name in argmap:
            r = argmap[type_name]
        else:
            print('unknown type %s'%(type_name), file=sys.stderr)
            return False

        if r == ':integer':
            print('long read_%s () { return shm->%s; }'%(name, name), file=fl)
            print('long write_%s (long in) { shm->%s = (%s)in; return (long)(shm->%s); }'%(name, name, type_name, name), file=fl)
        elif r == ':float' or r == ':float32':
            print('double read_%s () { return shm->%s; }'%(name, name), file=fl)
            print('double write_%s (double in) { shm->%s = (%s)in; return (double)(shm->%s); }'%(name, name, type_name, name), file=fl)
        else:
            print('unknown type %s'%(type_name), file=sys.stderr)
            return False
    elif len(size_list) == 1:
        if type_name in argmap:
            r = argmap[type_name]
        else:
            print('unknown type %s'%(type_name), file=sys.stderr)
            return False

        if r == ':integer':
            print('int read_%s (int n, long* out) { int nn = checkrange(n, 0, %d); return copy_%s2long(nn, &(shm->%s[0]), out); }'%(name, size_list[0], type_name, name), file=fl)
            print('int write_%s (int n, long* in) { int nn = checkrange(n, 0, %d); return copy_long2%s(nn, in, &(shm->%s[0])); }'%(name, size_list[0], type_name, name), file=fl)
        elif r == ':float' or r == ':float32':
            print('int read_%s (int n, double* out) { int nn = checkrange(n, 0, %d); return copy_%s2double(nn, &(shm->%s[0]), out); }'%(name, size_list[0], type_name, name), file=fl)
            print('int write_%s (int n, double * in) { int nn = checkrange(n, 0, %d); return copy_double2%s(nn, in, &(shm->%s[0])); }'%(name, size_list[0], type_name, name), file=fl)
        else:
            print('unknown type %s'%(type_name), file=sys.stderr)
            return False
    elif len(size_list) == 2:
        if type_name in argmap:
            r = argmap[type_name]
        else:
            print('unknown type %s'%(type_name), file=sys.stderr)
            return False

        if r == ':integer':
            print('long read_%s (int i, int n, long* out) { int ni = checkrange(i, 0, %d); int nn = checkrange(n, 0, %d); return copy_%s2long(nn, &(shm->%s[ni][0]), out); }'%(name, size_list[0], size_list[1], type_name, name), file=fl)
            print('long write_%s (int i, int n, long* in) { int ni = checkrange(i, 0, %d); int nn = checkrange(n, 0, %d); return copy_long2%s(nn, in, &(shm->%s[ni][0])); }'%(name, size_list[0], size_list[1], type_name, name), file=fl)
        elif r == ':float' or r == ':float32':
            print('long read_%s (int i, int n, double* out) { int ni = checkrange(i, 0, %d); int nn = checkrange(n, 0, %d); copy_%s2double(nn, &(shm->%s[ni][0]), out); }'%(name, size_list[0], size_list[1], type_name, name), file=fl)
            print('long write_%s (int i, int n, double * in) { int ni = checkrange(i, 0, %d); int nn = checkrange(n, 0, %d); copy_double2%s(nn, in, &(shm->%s[ni][0])); }'%(name, size_list[0], size_list[1], type_name, name), file=fl)
        else:
            print('unknown type %s'%(type_name), file=sys.stderr)
            return False
    else:
        print('%s has more than 3 depth of list?'%(name), file=sys.stderr)
        return False

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

        print_c_function(node.spelling, tpname, sz_lst, fl = c_file)
        print_eus_function(node.spelling, tpname, sz_lst, fl = eus_file)

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

def print_makefile(fl = sys.stderr):
    pass

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

    c_file = open('libeus_shm.c', mode='w')
    eus_file = open('eus-shm.l', mode='w')

    print_c_header(target_file, fl = c_file)
    print_eus_header(target_file, fl = eus_file)

    print_struct_recur(target_file, 0, tu.cursor)

    print_c_footer(fl = c_file)
    print_eus_footer(fl = eus_file)

    c_file.close()
    eus_file.close()
