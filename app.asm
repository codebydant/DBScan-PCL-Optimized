
app:     file format elf64-x86-64


Disassembly of section .init:

0000000000003000 <_init>:
    3000:	50                   	push   %rax
    3001:	e8 ba 15 00 00       	call   45c0 <frame_dummy>
    3006:	e8 d5 c1 00 00       	call   f1e0 <__do_global_ctors_aux>
    300b:	58                   	pop    %rax
    300c:	c3                   	ret    

Disassembly of section .plt:

0000000000003010 <_ZNSo9_M_insertImEERSoT_@plt-0x10>:
    3010:	ff 35 a2 0d 01 00    	push   0x10da2(%rip)        # 13db8 <_GLOBAL_OFFSET_TABLE_+0x8>
    3016:	ff 25 a4 0d 01 00    	jmp    *0x10da4(%rip)        # 13dc0 <_GLOBAL_OFFSET_TABLE_+0x10>
    301c:	0f 1f 40 00          	nopl   0x0(%rax)

0000000000003020 <_ZNSo9_M_insertImEERSoT_@plt>:
    3020:	ff 25 a2 0d 01 00    	jmp    *0x10da2(%rip)        # 13dc8 <_ZNSo9_M_insertImEERSoT_@Base>
    3026:	68 00 00 00 00       	push   $0x0
    302b:	e9 e0 ff ff ff       	jmp    3010 <_init+0x10>

0000000000003030 <_ZNSt9basic_iosIcSt11char_traitsIcEE4initEPSt15basic_streambufIcS1_E@plt>:
    3030:	ff 25 9a 0d 01 00    	jmp    *0x10d9a(%rip)        # 13dd0 <_ZNSt9basic_iosIcSt11char_traitsIcEE4initEPSt15basic_streambufIcS1_E@Base>
    3036:	68 01 00 00 00       	push   $0x1
    303b:	e9 d0 ff ff ff       	jmp    3010 <_init+0x10>

0000000000003040 <exit@plt>:
    3040:	ff 25 92 0d 01 00    	jmp    *0x10d92(%rip)        # 13dd8 <exit@Base>
    3046:	68 02 00 00 00       	push   $0x2
    304b:	e9 c0 ff ff ff       	jmp    3010 <_init+0x10>

0000000000003050 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE7reserveEm@plt>:
    3050:	ff 25 8a 0d 01 00    	jmp    *0x10d8a(%rip)        # 13de0 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE7reserveEm@Base>
    3056:	68 03 00 00 00       	push   $0x3
    305b:	e9 b0 ff ff ff       	jmp    3010 <_init+0x10>

0000000000003060 <_ZNSt11range_errorC1EPKc@plt>:
    3060:	ff 25 82 0d 01 00    	jmp    *0x10d82(%rip)        # 13de8 <_ZNSt11range_errorC1EPKc@Base>
    3066:	68 04 00 00 00       	push   $0x4
    306b:	e9 a0 ff ff ff       	jmp    3010 <_init+0x10>

0000000000003070 <_ZNKSt5ctypeIcE13_M_widen_initEv@plt>:
    3070:	ff 25 7a 0d 01 00    	jmp    *0x10d7a(%rip)        # 13df0 <_ZNKSt5ctypeIcE13_M_widen_initEv@Base>
    3076:	68 05 00 00 00       	push   $0x5
    307b:	e9 90 ff ff ff       	jmp    3010 <_init+0x10>

0000000000003080 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE14_M_replace_auxEmmmc@plt>:
    3080:	ff 25 72 0d 01 00    	jmp    *0x10d72(%rip)        # 13df8 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE14_M_replace_auxEmmmc@Base>
    3086:	68 06 00 00 00       	push   $0x6
    308b:	e9 80 ff ff ff       	jmp    3010 <_init+0x10>

0000000000003090 <_ZSt18_Rb_tree_decrementPSt18_Rb_tree_node_base@plt>:
    3090:	ff 25 6a 0d 01 00    	jmp    *0x10d6a(%rip)        # 13e00 <_ZSt18_Rb_tree_decrementPSt18_Rb_tree_node_base@Base>
    3096:	68 07 00 00 00       	push   $0x7
    309b:	e9 70 ff ff ff       	jmp    3010 <_init+0x10>

00000000000030a0 <_ZNSt8ios_baseD2Ev@plt>:
    30a0:	ff 25 62 0d 01 00    	jmp    *0x10d62(%rip)        # 13e08 <_ZNSt8ios_baseD2Ev@Base>
    30a6:	68 08 00 00 00       	push   $0x8
    30ab:	e9 60 ff ff ff       	jmp    3010 <_init+0x10>

00000000000030b0 <_ZSt18_Rb_tree_incrementPSt18_Rb_tree_node_base@plt>:
    30b0:	ff 25 5a 0d 01 00    	jmp    *0x10d5a(%rip)        # 13e10 <_ZSt18_Rb_tree_incrementPSt18_Rb_tree_node_base@Base>
    30b6:	68 09 00 00 00       	push   $0x9
    30bb:	e9 50 ff ff ff       	jmp    3010 <_init+0x10>

00000000000030c0 <__cxa_end_catch@plt>:
    30c0:	ff 25 52 0d 01 00    	jmp    *0x10d52(%rip)        # 13e18 <__cxa_end_catch@Base>
    30c6:	68 0a 00 00 00       	push   $0xa
    30cb:	e9 40 ff ff ff       	jmp    3010 <_init+0x10>

00000000000030d0 <__cxa_allocate_exception@plt>:
    30d0:	ff 25 4a 0d 01 00    	jmp    *0x10d4a(%rip)        # 13e20 <__cxa_allocate_exception@Base>
    30d6:	68 0b 00 00 00       	push   $0xb
    30db:	e9 30 ff ff ff       	jmp    3010 <_init+0x10>

00000000000030e0 <_ZSt18_Rb_tree_incrementPKSt18_Rb_tree_node_base@plt>:
    30e0:	ff 25 42 0d 01 00    	jmp    *0x10d42(%rip)        # 13e28 <_ZSt18_Rb_tree_incrementPKSt18_Rb_tree_node_base@Base>
    30e6:	68 0c 00 00 00       	push   $0xc
    30eb:	e9 20 ff ff ff       	jmp    3010 <_init+0x10>

00000000000030f0 <_ZNSt16invalid_argumentC1EPKc@plt>:
    30f0:	ff 25 3a 0d 01 00    	jmp    *0x10d3a(%rip)        # 13e30 <_ZNSt16invalid_argumentC1EPKc@Base>
    30f6:	68 0d 00 00 00       	push   $0xd
    30fb:	e9 10 ff ff ff       	jmp    3010 <_init+0x10>

0000000000003100 <_ZNSt8ios_baseC2Ev@plt>:
    3100:	ff 25 32 0d 01 00    	jmp    *0x10d32(%rip)        # 13e38 <_ZNSt8ios_baseC2Ev@Base>
    3106:	68 0e 00 00 00       	push   $0xe
    310b:	e9 00 ff ff ff       	jmp    3010 <_init+0x10>

0000000000003110 <_ZNSt6localeC1Ev@plt>:
    3110:	ff 25 2a 0d 01 00    	jmp    *0x10d2a(%rip)        # 13e40 <_ZNSt6localeC1Ev@Base>
    3116:	68 0f 00 00 00       	push   $0xf
    311b:	e9 f0 fe ff ff       	jmp    3010 <_init+0x10>

0000000000003120 <_ZStlsISt11char_traitsIcEERSt13basic_ostreamIcT_ES5_PKc@plt>:
    3120:	ff 25 22 0d 01 00    	jmp    *0x10d22(%rip)        # 13e48 <_ZStlsISt11char_traitsIcEERSt13basic_ostreamIcT_ES5_PKc@Base>
    3126:	68 10 00 00 00       	push   $0x10
    312b:	e9 e0 fe ff ff       	jmp    3010 <_init+0x10>

0000000000003130 <_ZNSo9_M_insertIdEERSoT_@plt>:
    3130:	ff 25 1a 0d 01 00    	jmp    *0x10d1a(%rip)        # 13e50 <_ZNSo9_M_insertIdEERSoT_@Base>
    3136:	68 11 00 00 00       	push   $0x11
    313b:	e9 d0 fe ff ff       	jmp    3010 <_init+0x10>

0000000000003140 <memcpy@plt>:
    3140:	ff 25 12 0d 01 00    	jmp    *0x10d12(%rip)        # 13e58 <memcpy@Base>
    3146:	68 12 00 00 00       	push   $0x12
    314b:	e9 c0 fe ff ff       	jmp    3010 <_init+0x10>

0000000000003150 <__stack_chk_fail@plt>:
    3150:	ff 25 0a 0d 01 00    	jmp    *0x10d0a(%rip)        # 13e60 <__stack_chk_fail@Base>
    3156:	68 13 00 00 00       	push   $0x13
    315b:	e9 b0 fe ff ff       	jmp    3010 <_init+0x10>

0000000000003160 <_ZNSo5flushEv@plt>:
    3160:	ff 25 02 0d 01 00    	jmp    *0x10d02(%rip)        # 13e68 <_ZNSo5flushEv@Base>
    3166:	68 14 00 00 00       	push   $0x14
    316b:	e9 a0 fe ff ff       	jmp    3010 <_init+0x10>

0000000000003170 <isspace@plt>:
    3170:	ff 25 fa 0c 01 00    	jmp    *0x10cfa(%rip)        # 13e70 <isspace@Base>
    3176:	68 15 00 00 00       	push   $0x15
    317b:	e9 90 fe ff ff       	jmp    3010 <_init+0x10>

0000000000003180 <_ZNKSt7__cxx1115basic_stringbufIcSt11char_traitsIcESaIcEE3strEv@plt>:
    3180:	ff 25 f2 0c 01 00    	jmp    *0x10cf2(%rip)        # 13e78 <_ZNKSt7__cxx1115basic_stringbufIcSt11char_traitsIcESaIcEE3strEv@Base>
    3186:	68 16 00 00 00       	push   $0x16
    318b:	e9 80 fe ff ff       	jmp    3010 <_init+0x10>

0000000000003190 <_ZNSo3putEc@plt>:
    3190:	ff 25 ea 0c 01 00    	jmp    *0x10cea(%rip)        # 13e80 <_ZNSo3putEc@Base>
    3196:	68 17 00 00 00       	push   $0x17
    319b:	e9 70 fe ff ff       	jmp    3010 <_init+0x10>

00000000000031a0 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE9_M_assignERKS4_@plt>:
    31a0:	ff 25 e2 0c 01 00    	jmp    *0x10ce2(%rip)        # 13e88 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE9_M_assignERKS4_@Base>
    31a6:	68 18 00 00 00       	push   $0x18
    31ab:	e9 60 fe ff ff       	jmp    3010 <_init+0x10>

00000000000031b0 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE10_M_replaceEmmPKcm@plt>:
    31b0:	ff 25 da 0c 01 00    	jmp    *0x10cda(%rip)        # 13e90 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE10_M_replaceEmmPKcm@Base>
    31b6:	68 19 00 00 00       	push   $0x19
    31bb:	e9 50 fe ff ff       	jmp    3010 <_init+0x10>

00000000000031c0 <_ZSt25__throw_bad_function_callv@plt>:
    31c0:	ff 25 d2 0c 01 00    	jmp    *0x10cd2(%rip)        # 13e98 <_ZSt25__throw_bad_function_callv@Base>
    31c6:	68 1a 00 00 00       	push   $0x1a
    31cb:	e9 40 fe ff ff       	jmp    3010 <_init+0x10>

00000000000031d0 <_ZSt16__throw_bad_castv@plt>:
    31d0:	ff 25 ca 0c 01 00    	jmp    *0x10cca(%rip)        # 13ea0 <_ZSt16__throw_bad_castv@Base>
    31d6:	68 1b 00 00 00       	push   $0x1b
    31db:	e9 30 fe ff ff       	jmp    3010 <_init+0x10>

00000000000031e0 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE9_M_appendEPKcm@plt>:
    31e0:	ff 25 c2 0c 01 00    	jmp    *0x10cc2(%rip)        # 13ea8 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE9_M_appendEPKcm@Base>
    31e6:	68 1c 00 00 00       	push   $0x1c
    31eb:	e9 20 fe ff ff       	jmp    3010 <_init+0x10>

00000000000031f0 <_ZNSt13runtime_errorC1EPKc@plt>:
    31f0:	ff 25 ba 0c 01 00    	jmp    *0x10cba(%rip)        # 13eb0 <_ZNSt13runtime_errorC1EPKc@Base>
    31f6:	68 1d 00 00 00       	push   $0x1d
    31fb:	e9 10 fe ff ff       	jmp    3010 <_init+0x10>

0000000000003200 <__cxa_begin_catch@plt>:
    3200:	ff 25 b2 0c 01 00    	jmp    *0x10cb2(%rip)        # 13eb8 <__cxa_begin_catch@Base>
    3206:	68 1e 00 00 00       	push   $0x1e
    320b:	e9 00 fe ff ff       	jmp    3010 <_init+0x10>

0000000000003210 <__cxa_rethrow@plt>:
    3210:	ff 25 aa 0c 01 00    	jmp    *0x10caa(%rip)        # 13ec0 <__cxa_rethrow@Base>
    3216:	68 1f 00 00 00       	push   $0x1f
    321b:	e9 f0 fd ff ff       	jmp    3010 <_init+0x10>

0000000000003220 <__cxa_throw@plt>:
    3220:	ff 25 a2 0c 01 00    	jmp    *0x10ca2(%rip)        # 13ec8 <__cxa_throw@Base>
    3226:	68 20 00 00 00       	push   $0x20
    322b:	e9 e0 fd ff ff       	jmp    3010 <_init+0x10>

0000000000003230 <_ZNSt6localeD1Ev@plt>:
    3230:	ff 25 9a 0c 01 00    	jmp    *0x10c9a(%rip)        # 13ed0 <_ZNSt6localeD1Ev@Base>
    3236:	68 21 00 00 00       	push   $0x21
    323b:	e9 d0 fd ff ff       	jmp    3010 <_init+0x10>

0000000000003240 <__libc_start_main@plt>:
    3240:	ff 25 92 0c 01 00    	jmp    *0x10c92(%rip)        # 13ed8 <__libc_start_main@Base>
    3246:	68 22 00 00 00       	push   $0x22
    324b:	e9 c0 fd ff ff       	jmp    3010 <_init+0x10>

0000000000003250 <_ZNSt8ios_base4InitC1Ev@plt>:
    3250:	ff 25 8a 0c 01 00    	jmp    *0x10c8a(%rip)        # 13ee0 <_ZNSt8ios_base4InitC1Ev@Base>
    3256:	68 23 00 00 00       	push   $0x23
    325b:	e9 b0 fd ff ff       	jmp    3010 <_init+0x10>

0000000000003260 <_ZNSt9exceptionD2Ev@plt>:
    3260:	ff 25 82 0c 01 00    	jmp    *0x10c82(%rip)        # 13ee8 <_ZNSt9exceptionD2Ev@Base>
    3266:	68 24 00 00 00       	push   $0x24
    326b:	e9 a0 fd ff ff       	jmp    3010 <_init+0x10>

0000000000003270 <memcmp@plt>:
    3270:	ff 25 7a 0c 01 00    	jmp    *0x10c7a(%rip)        # 13ef0 <memcmp@Base>
    3276:	68 25 00 00 00       	push   $0x25
    327b:	e9 90 fd ff ff       	jmp    3010 <_init+0x10>

0000000000003280 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE4swapERS4_@plt>:
    3280:	ff 25 72 0c 01 00    	jmp    *0x10c72(%rip)        # 13ef8 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE4swapERS4_@Base>
    3286:	68 26 00 00 00       	push   $0x26
    328b:	e9 80 fd ff ff       	jmp    3010 <_init+0x10>

0000000000003290 <_ZNSt13runtime_errorC1ERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEE@plt>:
    3290:	ff 25 6a 0c 01 00    	jmp    *0x10c6a(%rip)        # 13f00 <_ZNSt13runtime_errorC1ERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEE@Base>
    3296:	68 27 00 00 00       	push   $0x27
    329b:	e9 70 fd ff ff       	jmp    3010 <_init+0x10>

00000000000032a0 <_Znwm@plt>:
    32a0:	ff 25 62 0c 01 00    	jmp    *0x10c62(%rip)        # 13f08 <_Znwm@Base>
    32a6:	68 28 00 00 00       	push   $0x28
    32ab:	e9 60 fd ff ff       	jmp    3010 <_init+0x10>

00000000000032b0 <_ZNSt8__detail15_List_node_base11_M_transferEPS0_S1_@plt>:
    32b0:	ff 25 5a 0c 01 00    	jmp    *0x10c5a(%rip)        # 13f10 <_ZNSt8__detail15_List_node_base11_M_transferEPS0_S1_@Base>
    32b6:	68 29 00 00 00       	push   $0x29
    32bb:	e9 50 fd ff ff       	jmp    3010 <_init+0x10>

00000000000032c0 <_ZSt24__throw_out_of_range_fmtPKcz@plt>:
    32c0:	ff 25 52 0c 01 00    	jmp    *0x10c52(%rip)        # 13f18 <_ZSt24__throw_out_of_range_fmtPKcz@Base>
    32c6:	68 2a 00 00 00       	push   $0x2a
    32cb:	e9 40 fd ff ff       	jmp    3010 <_init+0x10>

00000000000032d0 <strtof@plt>:
    32d0:	ff 25 4a 0c 01 00    	jmp    *0x10c4a(%rip)        # 13f20 <strtof@Base>
    32d6:	68 2b 00 00 00       	push   $0x2b
    32db:	e9 30 fd ff ff       	jmp    3010 <_init+0x10>

00000000000032e0 <__cxa_atexit@plt>:
    32e0:	ff 25 42 0c 01 00    	jmp    *0x10c42(%rip)        # 13f28 <__cxa_atexit@Base>
    32e6:	68 2c 00 00 00       	push   $0x2c
    32eb:	e9 20 fd ff ff       	jmp    3010 <_init+0x10>

00000000000032f0 <_ZNSt8__detail15_List_node_base7_M_hookEPS0_@plt>:
    32f0:	ff 25 3a 0c 01 00    	jmp    *0x10c3a(%rip)        # 13f30 <_ZNSt8__detail15_List_node_base7_M_hookEPS0_@Base>
    32f6:	68 2d 00 00 00       	push   $0x2d
    32fb:	e9 10 fd ff ff       	jmp    3010 <_init+0x10>

0000000000003300 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE9_M_createERmm@plt>:
    3300:	ff 25 32 0c 01 00    	jmp    *0x10c32(%rip)        # 13f38 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE9_M_createERmm@Base>
    3306:	68 2e 00 00 00       	push   $0x2e
    330b:	e9 00 fd ff ff       	jmp    3010 <_init+0x10>

0000000000003310 <_ZNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEC1Ev@plt>:
    3310:	ff 25 2a 0c 01 00    	jmp    *0x10c2a(%rip)        # 13f40 <_ZNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEC1Ev@Base>
    3316:	68 2f 00 00 00       	push   $0x2f
    331b:	e9 f0 fc ff ff       	jmp    3010 <_init+0x10>

0000000000003320 <_ZdlPvm@plt>:
    3320:	ff 25 22 0c 01 00    	jmp    *0x10c22(%rip)        # 13f48 <_ZdlPvm@Base>
    3326:	68 30 00 00 00       	push   $0x30
    332b:	e9 e0 fc ff ff       	jmp    3010 <_init+0x10>

0000000000003330 <strlen@plt>:
    3330:	ff 25 1a 0c 01 00    	jmp    *0x10c1a(%rip)        # 13f50 <strlen@Base>
    3336:	68 31 00 00 00       	push   $0x31
    333b:	e9 d0 fc ff ff       	jmp    3010 <_init+0x10>

0000000000003340 <_ZSt20__throw_length_errorPKc@plt>:
    3340:	ff 25 12 0c 01 00    	jmp    *0x10c12(%rip)        # 13f58 <_ZSt20__throw_length_errorPKc@Base>
    3346:	68 32 00 00 00       	push   $0x32
    334b:	e9 c0 fc ff ff       	jmp    3010 <_init+0x10>

0000000000003350 <_ZNSolsEi@plt>:
    3350:	ff 25 0a 0c 01 00    	jmp    *0x10c0a(%rip)        # 13f60 <_ZNSolsEi@Base>
    3356:	68 33 00 00 00       	push   $0x33
    335b:	e9 b0 fc ff ff       	jmp    3010 <_init+0x10>

0000000000003360 <_Unwind_Resume@plt>:
    3360:	ff 25 02 0c 01 00    	jmp    *0x10c02(%rip)        # 13f68 <_Unwind_Resume@GCC_3.0>
    3366:	68 34 00 00 00       	push   $0x34
    336b:	e9 a0 fc ff ff       	jmp    3010 <_init+0x10>

0000000000003370 <_ZSt29_Rb_tree_insert_and_rebalancebPSt18_Rb_tree_node_baseS0_RS_@plt>:
    3370:	ff 25 fa 0b 01 00    	jmp    *0x10bfa(%rip)        # 13f70 <_ZSt29_Rb_tree_insert_and_rebalancebPSt18_Rb_tree_node_baseS0_RS_@Base>
    3376:	68 35 00 00 00       	push   $0x35
    337b:	e9 90 fc ff ff       	jmp    3010 <_init+0x10>

0000000000003380 <_ZNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEED1Ev@plt>:
    3380:	ff 25 f2 0b 01 00    	jmp    *0x10bf2(%rip)        # 13f78 <_ZNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEED1Ev@Base>
    3386:	68 36 00 00 00       	push   $0x36
    338b:	e9 80 fc ff ff       	jmp    3010 <_init+0x10>

0000000000003390 <__errno_location@plt>:
    3390:	ff 25 ea 0b 01 00    	jmp    *0x10bea(%rip)        # 13f80 <__errno_location@Base>
    3396:	68 37 00 00 00       	push   $0x37
    339b:	e9 70 fc ff ff       	jmp    3010 <_init+0x10>

00000000000033a0 <_ZSt16__ostream_insertIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_PKS3_l@plt>:
    33a0:	ff 25 e2 0b 01 00    	jmp    *0x10be2(%rip)        # 13f88 <_ZSt16__ostream_insertIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_PKS3_l@Base>
    33a6:	68 38 00 00 00       	push   $0x38
    33ab:	e9 60 fc ff ff       	jmp    3010 <_init+0x10>

00000000000033b0 <__cxa_free_exception@plt>:
    33b0:	ff 25 da 0b 01 00    	jmp    *0x10bda(%rip)        # 13f90 <__cxa_free_exception@Base>
    33b6:	68 39 00 00 00       	push   $0x39
    33bb:	e9 50 fc ff ff       	jmp    3010 <_init+0x10>

00000000000033c0 <_ZSt19__throw_logic_errorPKc@plt>:
    33c0:	ff 25 d2 0b 01 00    	jmp    *0x10bd2(%rip)        # 13f98 <_ZSt19__throw_logic_errorPKc@Base>
    33c6:	68 3a 00 00 00       	push   $0x3a
    33cb:	e9 40 fc ff ff       	jmp    3010 <_init+0x10>

Disassembly of section .plt.got:

00000000000033d0 <__cxa_finalize@plt>:
    33d0:	ff 25 ca 0b 01 00    	jmp    *0x10bca(%rip)        # 13fa0 <__cxa_finalize@Base>
    33d6:	66 90                	xchg   %ax,%ax

00000000000033d8 <__register_frame_info@plt>:
    33d8:	ff 25 da 0b 01 00    	jmp    *0x10bda(%rip)        # 13fb8 <__register_frame_info@GCC_3.0>
    33de:	66 90                	xchg   %ax,%ax

00000000000033e0 <__deregister_frame_info@plt>:
    33e0:	ff 25 ea 0b 01 00    	jmp    *0x10bea(%rip)        # 13fd0 <__deregister_frame_info@GCC_3.0>
    33e6:	66 90                	xchg   %ax,%ax

Disassembly of section .text:

00000000000033f0 <_ZSt26__throw_bad_variant_accessPKc>:
    33f0:	53                   	push   %rbx
    33f1:	48 89 fb             	mov    %rdi,%rbx
    33f4:	bf 10 00 00 00       	mov    $0x10,%edi
    33f9:	e8 d2 fc ff ff       	call   30d0 <__cxa_allocate_exception@plt>
    33fe:	48 89 c7             	mov    %rax,%rdi
    3401:	48 8d 05 68 04 01 00 	lea    0x10468(%rip),%rax        # 13870 <_ZTVSt18bad_variant_access+0x10>
    3408:	48 89 07             	mov    %rax,(%rdi)
    340b:	48 89 5f 08          	mov    %rbx,0x8(%rdi)
    340f:	48 8d 15 ba 2e 00 00 	lea    0x2eba(%rip),%rdx        # 62d0 <_ZNSt18bad_variant_accessD1Ev>
    3416:	48 8d 35 13 07 01 00 	lea    0x10713(%rip),%rsi        # 13b30 <_ZTVN10__cxxabiv120__si_class_type_infoE@Base>
    341d:	e8 fe fd ff ff       	call   3220 <__cxa_throw@plt>

0000000000003422 <_ZSt26__throw_bad_variant_accessb>:
    3422:	50                   	push   %rax
    3423:	40 84 ff             	test   %dil,%dil
    3426:	74 0c                	je     3434 <_ZSt26__throw_bad_variant_accessb+0x12>
    3428:	48 8d 3d e1 cc 00 00 	lea    0xcce1(%rip),%rdi        # 10110 <_fini+0xeef>
    342f:	e8 bc ff ff ff       	call   33f0 <_ZSt26__throw_bad_variant_accessPKc>
    3434:	48 8d 3d f5 cc 00 00 	lea    0xccf5(%rip),%rdi        # 10130 <_fini+0xf0f>
    343b:	e8 b0 ff ff ff       	call   33f0 <_ZSt26__throw_bad_variant_accessPKc>

0000000000003440 <_ZZNSt8__detail9__variant17_Move_assign_baseILb0EJSt8functionIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEES2_IFvSB_EEEEaSEOSG_ENUlOT_T0_E_clIRSD_St17integral_constantImLm0EEEEDaSJ_SK_.isra.0.cold>:
    3440:	31 ff                	xor    %edi,%edi
    3442:	e8 db ff ff ff       	call   3422 <_ZSt26__throw_bad_variant_accessb>
    3447:	90                   	nop

0000000000003448 <_ZNSt7variantIJSt8functionIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEES0_IFvS9_EEEE7emplaceILm0EJN8argparse7details12parse_numberIiLi0EEEEEENSt9enable_ifIX18is_constructible_vINSt19variant_alternativeIXT_ESE_E4typeEDpT0_EERSN_E4typeEDpOSO_.isra.0.cold>:
    3448:	31 ff                	xor    %edi,%edi
    344a:	e8 d3 ff ff ff       	call   3422 <_ZSt26__throw_bad_variant_accessb>

000000000000344f <_ZN8argparselsERSoRKNS_8ArgumentE.isra.0.cold>:
    344f:	48 8b 05 7a 05 01 00 	mov    0x1057a(%rip),%rax        # 139d0 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x10>
    3456:	48 8b 35 7b 05 01 00 	mov    0x1057b(%rip),%rsi        # 139d8 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x18>
    345d:	48 89 44 24 40       	mov    %rax,0x40(%rsp)
    3462:	48 8b 40 e8          	mov    -0x18(%rax),%rax
    3466:	48 89 74 04 40       	mov    %rsi,0x40(%rsp,%rax,1)
    346b:	48 c7 44 24 48 00 00 	movq   $0x0,0x48(%rsp)
    3472:	00 00 
    3474:	c5 f8 77             	vzeroupper 
    3477:	48 8b 7c 24 18       	mov    0x18(%rsp),%rdi
    347c:	48 8d 05 65 04 01 00 	lea    0x10465(%rip),%rax        # 138e8 <_ZTVSt9basic_iosIcSt11char_traitsIcEE+0x10>
    3483:	48 89 84 24 c0 00 00 	mov    %rax,0xc0(%rsp)
    348a:	00 
    348b:	e8 10 fc ff ff       	call   30a0 <_ZNSt8ios_baseD2Ev@plt>
    3490:	48 89 df             	mov    %rbx,%rdi
    3493:	e8 c8 fe ff ff       	call   3360 <_Unwind_Resume@plt>
    3498:	48 8b 7c 24 20       	mov    0x20(%rsp),%rdi
    349d:	48 39 df             	cmp    %rbx,%rdi
    34a0:	0f 84 ac 00 00 00    	je     3552 <_ZN8argparselsERSoRKNS_8ArgumentE.isra.0.cold+0x103>
    34a6:	48 8b 44 24 30       	mov    0x30(%rsp),%rax
    34ab:	48 8d 70 01          	lea    0x1(%rax),%rsi
    34af:	c5 f8 77             	vzeroupper 
    34b2:	e8 69 fe ff ff       	call   3320 <_ZdlPvm@plt>
    34b7:	48 8b 3c 24          	mov    (%rsp),%rdi
    34bb:	e8 c0 fe ff ff       	call   3380 <_ZNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEED1Ev@plt>
    34c0:	4c 89 e7             	mov    %r12,%rdi
    34c3:	e8 98 fe ff ff       	call   3360 <_Unwind_Resume@plt>
    34c8:	48 8b 7c 24 20       	mov    0x20(%rsp),%rdi
    34cd:	48 39 df             	cmp    %rbx,%rdi
    34d0:	74 13                	je     34e5 <_ZN8argparselsERSoRKNS_8ArgumentE.isra.0.cold+0x96>
    34d2:	48 8b 44 24 30       	mov    0x30(%rsp),%rax
    34d7:	48 8d 70 01          	lea    0x1(%rax),%rsi
    34db:	c5 f8 77             	vzeroupper 
    34de:	e8 3d fe ff ff       	call   3320 <_ZdlPvm@plt>
    34e3:	eb d2                	jmp    34b7 <_ZN8argparselsERSoRKNS_8ArgumentE.isra.0.cold+0x68>
    34e5:	c5 f8 77             	vzeroupper 
    34e8:	eb cd                	jmp    34b7 <_ZN8argparselsERSoRKNS_8ArgumentE.isra.0.cold+0x68>
    34ea:	4c 89 e7             	mov    %r12,%rdi
    34ed:	c5 f8 77             	vzeroupper 
    34f0:	e8 6b 4e 00 00       	call   8360 <_ZNSt7__cxx1115basic_stringbufIcSt11char_traitsIcESaIcEED1Ev>
    34f5:	48 8b 05 cc 04 01 00 	mov    0x104cc(%rip),%rax        # 139c8 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x8>
    34fc:	48 8b 35 ed 04 01 00 	mov    0x104ed(%rip),%rsi        # 139f0 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x30>
    3503:	48 8b 40 e8          	mov    -0x18(%rax),%rax
    3507:	48 89 74 04 40       	mov    %rsi,0x40(%rsp,%rax,1)
    350c:	48 8b 05 cd 04 01 00 	mov    0x104cd(%rip),%rax        # 139e0 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x20>
    3513:	48 8b 35 ce 04 01 00 	mov    0x104ce(%rip),%rsi        # 139e8 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x28>
    351a:	48 89 44 24 50       	mov    %rax,0x50(%rsp)
    351f:	48 8b 40 e8          	mov    -0x18(%rax),%rax
    3523:	48 89 74 04 50       	mov    %rsi,0x50(%rsp,%rax,1)
    3528:	48 8b 05 a1 04 01 00 	mov    0x104a1(%rip),%rax        # 139d0 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x10>
    352f:	48 8b 35 a2 04 01 00 	mov    0x104a2(%rip),%rsi        # 139d8 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x18>
    3536:	48 89 44 24 40       	mov    %rax,0x40(%rsp)
    353b:	48 8b 40 e8          	mov    -0x18(%rax),%rax
    353f:	48 89 74 04 40       	mov    %rsi,0x40(%rsp,%rax,1)
    3544:	48 c7 44 24 48 00 00 	movq   $0x0,0x48(%rsp)
    354b:	00 00 
    354d:	e9 25 ff ff ff       	jmp    3477 <_ZN8argparselsERSoRKNS_8ArgumentE.isra.0.cold+0x28>
    3552:	c5 f8 77             	vzeroupper 
    3555:	e9 5d ff ff ff       	jmp    34b7 <_ZN8argparselsERSoRKNS_8ArgumentE.isra.0.cold+0x68>

000000000000355a <_ZZN8argparse14ArgumentParserC4ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsEENKUlRKT_E_clIS6_EEDaSA_.isra.0.cold>:
    355a:	4c 89 e7             	mov    %r12,%rdi
    355d:	c5 f8 77             	vzeroupper 
    3560:	e8 1b fe ff ff       	call   3380 <_ZNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEED1Ev@plt>
    3565:	48 89 ef             	mov    %rbp,%rdi
    3568:	e8 f3 fd ff ff       	call   3360 <_Unwind_Resume@plt>
    356d:	48 8b 3c 24          	mov    (%rsp),%rdi
    3571:	48 8d 44 24 10       	lea    0x10(%rsp),%rax
    3576:	48 39 c7             	cmp    %rax,%rdi
    3579:	74 21                	je     359c <_ZZN8argparse14ArgumentParserC4ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsEENKUlRKT_E_clIS6_EEDaSA_.isra.0.cold+0x42>
    357b:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
    3580:	48 8d 70 01          	lea    0x1(%rax),%rsi
    3584:	c5 f8 77             	vzeroupper 
    3587:	e8 94 fd ff ff       	call   3320 <_ZdlPvm@plt>
    358c:	4c 89 e7             	mov    %r12,%rdi
    358f:	e8 ec fd ff ff       	call   3380 <_ZNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEED1Ev@plt>
    3594:	48 89 ef             	mov    %rbp,%rdi
    3597:	e8 c4 fd ff ff       	call   3360 <_Unwind_Resume@plt>
    359c:	c5 f8 77             	vzeroupper 
    359f:	eb eb                	jmp    358c <_ZZN8argparse14ArgumentParserC4ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsEENKUlRKT_E_clIS6_EEDaSA_.isra.0.cold+0x32>

00000000000035a1 <main.cold>:
    35a1:	31 ff                	xor    %edi,%edi
    35a3:	e8 7a fe ff ff       	call   3422 <_ZSt26__throw_bad_variant_accessb>
    35a8:	48 8b bd 20 fd ff ff 	mov    -0x2e0(%rbp),%rdi
    35af:	48 3b bd 18 fc ff ff 	cmp    -0x3e8(%rbp),%rdi
    35b6:	74 36                	je     35ee <main.cold+0x4d>
    35b8:	48 8b 85 30 fd ff ff 	mov    -0x2d0(%rbp),%rax
    35bf:	48 8d 70 01          	lea    0x1(%rax),%rsi
    35c3:	c5 f8 77             	vzeroupper 
    35c6:	e8 55 fd ff ff       	call   3320 <_ZdlPvm@plt>
    35cb:	4c 89 e7             	mov    %r12,%rdi
    35ce:	e8 ad fd ff ff       	call   3380 <_ZNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEED1Ev@plt>
    35d3:	49 89 dc             	mov    %rbx,%r12
    35d6:	4c 89 ef             	mov    %r13,%rdi
    35d9:	e8 b2 5b 00 00       	call   9190 <_ZN8argparse14ArgumentParserD1Ev>
    35de:	4c 89 e7             	mov    %r12,%rdi
    35e1:	e8 7a fd ff ff       	call   3360 <_Unwind_Resume@plt>
    35e6:	49 89 c4             	mov    %rax,%r12
    35e9:	c5 f8 77             	vzeroupper 
    35ec:	eb e8                	jmp    35d6 <main.cold+0x35>
    35ee:	c5 f8 77             	vzeroupper 
    35f1:	eb d8                	jmp    35cb <main.cold+0x2a>
    35f3:	48 8b bd 30 fe ff ff 	mov    -0x1d0(%rbp),%rdi
    35fa:	48 3b bd 28 fc ff ff 	cmp    -0x3d8(%rbp),%rdi
    3601:	74 5e                	je     3661 <main.cold+0xc0>
    3603:	48 8b 85 40 fe ff ff 	mov    -0x1c0(%rbp),%rax
    360a:	48 8d 70 01          	lea    0x1(%rax),%rsi
    360e:	c5 f8 77             	vzeroupper 
    3611:	e8 0a fd ff ff       	call   3320 <_ZdlPvm@plt>
    3616:	eb be                	jmp    35d6 <main.cold+0x35>
    3618:	48 8b 85 30 fe ff ff 	mov    -0x1d0(%rbp),%rax
    361f:	48 85 c0             	test   %rax,%rax
    3622:	74 38                	je     365c <main.cold+0xbb>
    3624:	31 d2                	xor    %edx,%edx
    3626:	4c 89 e6             	mov    %r12,%rsi
    3629:	bf 03 00 00 00       	mov    $0x3,%edi
    362e:	c5 f8 77             	vzeroupper 
    3631:	ff d0                	call   *%rax
    3633:	49 89 dc             	mov    %rbx,%r12
    3636:	eb 9e                	jmp    35d6 <main.cold+0x35>
    3638:	48 8b 85 20 fd ff ff 	mov    -0x2e0(%rbp),%rax
    363f:	48 85 c0             	test   %rax,%rax
    3642:	74 1d                	je     3661 <main.cold+0xc0>
    3644:	48 8b b5 20 fc ff ff 	mov    -0x3e0(%rbp),%rsi
    364b:	31 d2                	xor    %edx,%edx
    364d:	bf 03 00 00 00       	mov    $0x3,%edi
    3652:	c5 f8 77             	vzeroupper 
    3655:	ff d0                	call   *%rax
    3657:	e9 7a ff ff ff       	jmp    35d6 <main.cold+0x35>
    365c:	c5 f8 77             	vzeroupper 
    365f:	eb d2                	jmp    3633 <main.cold+0x92>
    3661:	c5 f8 77             	vzeroupper 
    3664:	e9 6d ff ff ff       	jmp    35d6 <main.cold+0x35>
    3669:	48 ff c8             	dec    %rax
    366c:	0f 85 8e 00 00 00    	jne    3700 <main.cold+0x15f>
    3672:	c5 f8 77             	vzeroupper 
    3675:	e8 86 fb ff ff       	call   3200 <__cxa_begin_catch@plt>
    367a:	48 89 c7             	mov    %rax,%rdi
    367d:	48 8b 00             	mov    (%rax),%rax
    3680:	ff 50 10             	call   *0x10(%rax)
    3683:	48 89 c6             	mov    %rax,%rsi
    3686:	48 8d 3d f3 0a 01 00 	lea    0x10af3(%rip),%rdi        # 14180 <_ZSt4cerr@@Base>
    368d:	e8 8e fa ff ff       	call   3120 <_ZStlsISt11char_traitsIcEERSt13basic_ostreamIcT_ES5_PKc@plt>
    3692:	48 89 c7             	mov    %rax,%rdi
    3695:	e8 66 0f 00 00       	call   4600 <_ZSt4endlIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_.isra.0>
    369a:	4c 89 ee             	mov    %r13,%rsi
    369d:	48 8d 3d dc 0a 01 00 	lea    0x10adc(%rip),%rdi        # 14180 <_ZSt4cerr@@Base>
    36a4:	e8 77 26 00 00       	call   5d20 <_ZN8argparselsERSoRKNS_14ArgumentParserE.isra.0>
    36a9:	e8 12 fa ff ff       	call   30c0 <__cxa_end_catch@plt>
    36ae:	e9 00 0c 00 00       	jmp    42b3 <main+0xac3>
    36b3:	48 8b 05 16 03 01 00 	mov    0x10316(%rip),%rax        # 139d0 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x10>
    36ba:	48 8b 0d 17 03 01 00 	mov    0x10317(%rip),%rcx        # 139d8 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x18>
    36c1:	48 89 85 30 fe ff ff 	mov    %rax,-0x1d0(%rbp)
    36c8:	48 8b 40 e8          	mov    -0x18(%rax),%rax
    36cc:	48 89 8c 05 30 fe ff 	mov    %rcx,-0x1d0(%rbp,%rax,1)
    36d3:	ff 
    36d4:	48 c7 85 38 fe ff ff 	movq   $0x0,-0x1c8(%rbp)
    36db:	00 00 00 00 
    36df:	c5 f8 77             	vzeroupper 
    36e2:	48 8d 05 ff 01 01 00 	lea    0x101ff(%rip),%rax        # 138e8 <_ZTVSt9basic_iosIcSt11char_traitsIcEE+0x10>
    36e9:	4c 89 ff             	mov    %r15,%rdi
    36ec:	48 89 85 b0 fe ff ff 	mov    %rax,-0x150(%rbp)
    36f3:	49 89 dc             	mov    %rbx,%r12
    36f6:	e8 a5 f9 ff ff       	call   30a0 <_ZNSt8ios_baseD2Ev@plt>
    36fb:	e9 d6 fe ff ff       	jmp    35d6 <main.cold+0x35>
    3700:	49 89 fc             	mov    %rdi,%r12
    3703:	c5 f8 77             	vzeroupper 
    3706:	e9 cb fe ff ff       	jmp    35d6 <main.cold+0x35>
    370b:	49 89 c4             	mov    %rax,%r12
    370e:	c5 f8 77             	vzeroupper 
    3711:	e8 aa f9 ff ff       	call   30c0 <__cxa_end_catch@plt>
    3716:	e9 bb fe ff ff       	jmp    35d6 <main.cold+0x35>
    371b:	48 8b bd 20 fd ff ff 	mov    -0x2e0(%rbp),%rdi
    3722:	48 8d 85 30 fd ff ff 	lea    -0x2d0(%rbp),%rax
    3729:	48 39 c7             	cmp    %rax,%rdi
    372c:	74 3e                	je     376c <main.cold+0x1cb>
    372e:	48 8b 85 30 fd ff ff 	mov    -0x2d0(%rbp),%rax
    3735:	48 8d 70 01          	lea    0x1(%rax),%rsi
    3739:	c5 f8 77             	vzeroupper 
    373c:	e8 df fb ff ff       	call   3320 <_ZdlPvm@plt>
    3741:	48 8b bd 30 fe ff ff 	mov    -0x1d0(%rbp),%rdi
    3748:	48 8d 85 40 fe ff ff 	lea    -0x1c0(%rbp),%rax
    374f:	48 39 c7             	cmp    %rax,%rdi
    3752:	74 10                	je     3764 <main.cold+0x1c3>
    3754:	48 8b 85 40 fe ff ff 	mov    -0x1c0(%rbp),%rax
    375b:	48 8d 70 01          	lea    0x1(%rax),%rsi
    375f:	e8 bc fb ff ff       	call   3320 <_ZdlPvm@plt>
    3764:	4c 89 e7             	mov    %r12,%rdi
    3767:	e8 f4 fb ff ff       	call   3360 <_Unwind_Resume@plt>
    376c:	c5 f8 77             	vzeroupper 
    376f:	eb d0                	jmp    3741 <main.cold+0x1a0>
    3771:	48 8b bd f0 fb ff ff 	mov    -0x410(%rbp),%rdi
    3778:	c5 f8 77             	vzeroupper 
    377b:	e8 e0 4b 00 00       	call   8360 <_ZNSt7__cxx1115basic_stringbufIcSt11char_traitsIcESaIcEED1Ev>
    3780:	48 8b 05 41 02 01 00 	mov    0x10241(%rip),%rax        # 139c8 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x8>
    3787:	48 8b 0d 62 02 01 00 	mov    0x10262(%rip),%rcx        # 139f0 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x30>
    378e:	48 8b 40 e8          	mov    -0x18(%rax),%rax
    3792:	48 89 8c 05 30 fe ff 	mov    %rcx,-0x1d0(%rbp,%rax,1)
    3799:	ff 
    379a:	48 8b 05 3f 02 01 00 	mov    0x1023f(%rip),%rax        # 139e0 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x20>
    37a1:	48 8b 0d 40 02 01 00 	mov    0x10240(%rip),%rcx        # 139e8 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x28>
    37a8:	48 89 85 40 fe ff ff 	mov    %rax,-0x1c0(%rbp)
    37af:	48 8b 40 e8          	mov    -0x18(%rax),%rax
    37b3:	48 89 8c 05 40 fe ff 	mov    %rcx,-0x1c0(%rbp,%rax,1)
    37ba:	ff 
    37bb:	48 8b 05 0e 02 01 00 	mov    0x1020e(%rip),%rax        # 139d0 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x10>
    37c2:	48 8b 0d 0f 02 01 00 	mov    0x1020f(%rip),%rcx        # 139d8 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x18>
    37c9:	48 89 85 30 fe ff ff 	mov    %rax,-0x1d0(%rbp)
    37d0:	48 8b 40 e8          	mov    -0x18(%rax),%rax
    37d4:	48 89 8c 05 30 fe ff 	mov    %rcx,-0x1d0(%rbp,%rax,1)
    37db:	ff 
    37dc:	48 c7 85 38 fe ff ff 	movq   $0x0,-0x1c8(%rbp)
    37e3:	00 00 00 00 
    37e7:	e9 f6 fe ff ff       	jmp    36e2 <main.cold+0x141>
    37ec:	0f 1f 40 00          	nopl   0x0(%rax)

00000000000037f0 <main>:
    37f0:	4c 8d 54 24 08       	lea    0x8(%rsp),%r10
    37f5:	48 83 e4 e0          	and    $0xffffffffffffffe0,%rsp
    37f9:	41 ff 72 f8          	push   -0x8(%r10)
    37fd:	ba 26 00 00 00       	mov    $0x26,%edx
    3802:	55                   	push   %rbp
    3803:	48 89 e5             	mov    %rsp,%rbp
    3806:	41 57                	push   %r15
    3808:	41 56                	push   %r14
    380a:	41 55                	push   %r13
    380c:	41 54                	push   %r12
    380e:	4c 8d a5 30 fe ff ff 	lea    -0x1d0(%rbp),%r12
    3815:	41 52                	push   %r10
    3817:	53                   	push   %rbx
    3818:	48 81 ec e0 03 00 00 	sub    $0x3e0,%rsp
    381f:	89 bd 14 fc ff ff    	mov    %edi,-0x3ec(%rbp)
    3825:	48 89 b5 08 fc ff ff 	mov    %rsi,-0x3f8(%rbp)
    382c:	48 8d 3d 0d 08 01 00 	lea    0x1080d(%rip),%rdi        # 14040 <_ZSt4cout@@Base>
    3833:	48 8d 35 ae cb 00 00 	lea    0xcbae(%rip),%rsi        # 103e8 <_fini+0x11c7>
    383a:	64 48 8b 04 25 28 00 	mov    %fs:0x28,%rax
    3841:	00 00 
    3843:	48 89 45 c8          	mov    %rax,-0x38(%rbp)
    3847:	31 c0                	xor    %eax,%eax
    3849:	e8 52 fb ff ff       	call   33a0 <_ZSt16__ostream_insertIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_PKS3_l@plt>
    384e:	48 8d 3d eb 07 01 00 	lea    0x107eb(%rip),%rdi        # 14040 <_ZSt4cout@@Base>
    3855:	e8 a6 0d 00 00       	call   4600 <_ZSt4endlIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_.isra.0>
    385a:	ba 24 00 00 00       	mov    $0x24,%edx
    385f:	48 8d 35 aa cb 00 00 	lea    0xcbaa(%rip),%rsi        # 10410 <_fini+0x11ef>
    3866:	48 8d 3d d3 07 01 00 	lea    0x107d3(%rip),%rdi        # 14040 <_ZSt4cout@@Base>
    386d:	e8 2e fb ff ff       	call   33a0 <_ZSt16__ostream_insertIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_PKS3_l@plt>
    3872:	48 8d 3d c7 07 01 00 	lea    0x107c7(%rip),%rdi        # 14040 <_ZSt4cout@@Base>
    3879:	e8 82 0d 00 00       	call   4600 <_ZSt4endlIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_.isra.0>
    387e:	ba 25 00 00 00       	mov    $0x25,%edx
    3883:	48 8d 35 ae cb 00 00 	lea    0xcbae(%rip),%rsi        # 10438 <_fini+0x1217>
    388a:	48 8d 3d af 07 01 00 	lea    0x107af(%rip),%rdi        # 14040 <_ZSt4cout@@Base>
    3891:	e8 0a fb ff ff       	call   33a0 <_ZSt16__ostream_insertIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_PKS3_l@plt>
    3896:	48 8d 3d a3 07 01 00 	lea    0x107a3(%rip),%rdi        # 14040 <_ZSt4cout@@Base>
    389d:	e8 5e 0d 00 00       	call   4600 <_ZSt4endlIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_.isra.0>
    38a2:	48 8d 35 4b c9 00 00 	lea    0xc94b(%rip),%rsi        # 101f4 <_fini+0xfd3>
    38a9:	4c 89 e7             	mov    %r12,%rdi
    38ac:	e8 3f 10 00 00       	call   48f0 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEC2IS3_EEPKcRKS3_.constprop.0>
    38b1:	48 8d 85 20 fd ff ff 	lea    -0x2e0(%rbp),%rax
    38b8:	48 8d 35 39 c9 00 00 	lea    0xc939(%rip),%rsi        # 101f8 <_fini+0xfd7>
    38bf:	48 89 c7             	mov    %rax,%rdi
    38c2:	48 89 85 20 fc ff ff 	mov    %rax,-0x3e0(%rbp)
    38c9:	48 89 c3             	mov    %rax,%rbx
    38cc:	e8 1f 10 00 00       	call   48f0 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEC2IS3_EEPKcRKS3_.constprop.0>
    38d1:	4c 8d ad 40 fd ff ff 	lea    -0x2c0(%rbp),%r13
    38d8:	b9 03 00 00 00       	mov    $0x3,%ecx
    38dd:	4c 89 e2             	mov    %r12,%rdx
    38e0:	48 89 de             	mov    %rbx,%rsi
    38e3:	4c 89 ef             	mov    %r13,%rdi
    38e6:	e8 45 9a 00 00       	call   d330 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE>
    38eb:	48 8b bd 20 fd ff ff 	mov    -0x2e0(%rbp),%rdi
    38f2:	48 8d 85 30 fd ff ff 	lea    -0x2d0(%rbp),%rax
    38f9:	48 89 85 18 fc ff ff 	mov    %rax,-0x3e8(%rbp)
    3900:	48 39 c7             	cmp    %rax,%rdi
    3903:	74 10                	je     3915 <main+0x125>
    3905:	48 8b 85 30 fd ff ff 	mov    -0x2d0(%rbp),%rax
    390c:	48 8d 70 01          	lea    0x1(%rax),%rsi
    3910:	e8 0b fa ff ff       	call   3320 <_ZdlPvm@plt>
    3915:	48 8b bd 30 fe ff ff 	mov    -0x1d0(%rbp),%rdi
    391c:	48 8d 85 40 fe ff ff 	lea    -0x1c0(%rbp),%rax
    3923:	48 89 85 28 fc ff ff 	mov    %rax,-0x3d8(%rbp)
    392a:	48 39 c7             	cmp    %rax,%rdi
    392d:	74 10                	je     393f <main+0x14f>
    392f:	48 8b 85 40 fe ff ff 	mov    -0x1c0(%rbp),%rax
    3936:	48 8d 70 01          	lea    0x1(%rax),%rsi
    393a:	e8 e1 f9 ff ff       	call   3320 <_ZdlPvm@plt>
    393f:	ba 04 00 00 00       	mov    $0x4,%edx
    3944:	48 8d 35 b5 c8 00 00 	lea    0xc8b5(%rip),%rsi        # 10200 <_fini+0xfdf>
    394b:	48 8d 3d ee 06 01 00 	lea    0x106ee(%rip),%rdi        # 14040 <_ZSt4cout@@Base>
    3952:	e8 49 fa ff ff       	call   33a0 <_ZSt16__ostream_insertIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_PKS3_l@plt>
    3957:	48 8d 3d e2 06 01 00 	lea    0x106e2(%rip),%rdi        # 14040 <_ZSt4cout@@Base>
    395e:	e8 9d 0c 00 00       	call   4600 <_ZSt4endlIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_.isra.0>
    3963:	ba 0f 00 00 00       	mov    $0xf,%edx
    3968:	48 8d 35 96 c8 00 00 	lea    0xc896(%rip),%rsi        # 10205 <_fini+0xfe4>
    396f:	48 8d 3d ca 06 01 00 	lea    0x106ca(%rip),%rdi        # 14040 <_ZSt4cout@@Base>
    3976:	e8 25 fa ff ff       	call   33a0 <_ZSt16__ostream_insertIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_PKS3_l@plt>
    397b:	48 8d 3d be 06 01 00 	lea    0x106be(%rip),%rdi        # 14040 <_ZSt4cout@@Base>
    3982:	e8 79 0c 00 00       	call   4600 <_ZSt4endlIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_.isra.0>
    3987:	48 8d 35 87 c8 00 00 	lea    0xc887(%rip),%rsi        # 10215 <_fini+0xff4>
    398e:	4c 89 ef             	mov    %r13,%rdi
    3991:	e8 0a ac 00 00       	call   e5a0 <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_>
    3996:	80 88 d9 00 00 00 02 	orb    $0x2,0xd9(%rax)
    399d:	4c 8d b5 40 fc ff ff 	lea    -0x3c0(%rbp),%r14
    39a4:	48 8d 35 76 c8 00 00 	lea    0xc876(%rip),%rsi        # 10221 <_fini+0x1000>
    39ab:	4c 89 f7             	mov    %r14,%rdi
    39ae:	48 89 c3             	mov    %rax,%rbx
    39b1:	e8 3a 0f 00 00       	call   48f0 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEC2IS3_EEPKcRKS3_.constprop.0>
    39b6:	48 8d 7b 28          	lea    0x28(%rbx),%rdi
    39ba:	4c 89 f6             	mov    %r14,%rsi
    39bd:	e8 ae 0c 00 00       	call   4670 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEaSEOS4_.isra.0>
    39c2:	48 8b bd 40 fc ff ff 	mov    -0x3c0(%rbp),%rdi
    39c9:	48 8d 85 50 fc ff ff 	lea    -0x3b0(%rbp),%rax
    39d0:	48 39 c7             	cmp    %rax,%rdi
    39d3:	74 10                	je     39e5 <main+0x1f5>
    39d5:	48 8b 85 50 fc ff ff 	mov    -0x3b0(%rbp),%rax
    39dc:	48 8d 70 01          	lea    0x1(%rax),%rsi
    39e0:	e8 3b f9 ff ff       	call   3320 <_ZdlPvm@plt>
    39e5:	48 8d 35 46 c8 00 00 	lea    0xc846(%rip),%rsi        # 10232 <_fini+0x1011>
    39ec:	4c 89 ef             	mov    %r13,%rdi
    39ef:	e8 ac ab 00 00       	call   e5a0 <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_>
    39f4:	4c 8d b5 38 fc ff ff 	lea    -0x3c8(%rbp),%r14
    39fb:	48 89 c7             	mov    %rax,%rdi
    39fe:	4c 89 f6             	mov    %r14,%rsi
    3a01:	c7 85 38 fc ff ff 78 	movl   $0x78,-0x3c8(%rbp)
    3a08:	00 00 00 
    3a0b:	e8 90 5c 00 00       	call   96a0 <_ZN8argparse8Argument13default_valueIiEERS0_OT_>
    3a10:	48 8d b8 88 00 00 00 	lea    0x88(%rax),%rdi
    3a17:	48 89 c3             	mov    %rax,%rbx
    3a1a:	e8 91 1d 00 00       	call   57b0 <_ZNSt7variantIJSt8functionIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEES0_IFvS9_EEEE7emplaceILm0EJN8argparse7details12parse_numberIiLi0EEEEEENSt9enable_ifIX18is_constructible_vINSt19variant_alternativeIXT_ESE_E4typeEDpT0_EERSN_E4typeEDpOSO_.isra.0>
    3a1f:	4c 8d bd 60 fc ff ff 	lea    -0x3a0(%rbp),%r15
    3a26:	48 8d 35 12 c8 00 00 	lea    0xc812(%rip),%rsi        # 1023f <_fini+0x101e>
    3a2d:	4c 89 ff             	mov    %r15,%rdi
    3a30:	e8 bb 0e 00 00       	call   48f0 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEC2IS3_EEPKcRKS3_.constprop.0>
    3a35:	48 8d 7b 28          	lea    0x28(%rbx),%rdi
    3a39:	4c 89 fe             	mov    %r15,%rsi
    3a3c:	e8 2f 0c 00 00       	call   4670 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEaSEOS4_.isra.0>
    3a41:	48 8b bd 60 fc ff ff 	mov    -0x3a0(%rbp),%rdi
    3a48:	48 8d 85 70 fc ff ff 	lea    -0x390(%rbp),%rax
    3a4f:	48 39 c7             	cmp    %rax,%rdi
    3a52:	74 10                	je     3a64 <main+0x274>
    3a54:	48 8b 85 70 fc ff ff 	mov    -0x390(%rbp),%rax
    3a5b:	48 8d 70 01          	lea    0x1(%rax),%rsi
    3a5f:	e8 bc f8 ff ff       	call   3320 <_ZdlPvm@plt>
    3a64:	48 8d 35 e6 c7 00 00 	lea    0xc7e6(%rip),%rsi        # 10251 <_fini+0x1030>
    3a6b:	4c 89 ef             	mov    %r13,%rdi
    3a6e:	e8 2d ab 00 00       	call   e5a0 <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_>
    3a73:	4c 8d bd b0 fe ff ff 	lea    -0x150(%rbp),%r15
    3a7a:	4c 89 ff             	mov    %r15,%rdi
    3a7d:	48 89 c3             	mov    %rax,%rbx
    3a80:	e8 7b f6 ff ff       	call   3100 <_ZNSt8ios_baseC2Ev@plt>
    3a85:	48 8d 05 5c fe 00 00 	lea    0xfe5c(%rip),%rax        # 138e8 <_ZTVSt9basic_iosIcSt11char_traitsIcEE+0x10>
    3a8c:	48 89 85 b0 fe ff ff 	mov    %rax,-0x150(%rbp)
    3a93:	31 c0                	xor    %eax,%eax
    3a95:	66 89 45 90          	mov    %ax,-0x70(%rbp)
    3a99:	c5 f9 ef c0          	vpxor  %xmm0,%xmm0,%xmm0
    3a9d:	48 8b 05 2c ff 00 00 	mov    0xff2c(%rip),%rax        # 139d0 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x10>
    3aa4:	c5 fe 7f 45 98       	vmovdqu %ymm0,-0x68(%rbp)
    3aa9:	48 89 85 30 fe ff ff 	mov    %rax,-0x1d0(%rbp)
    3ab0:	48 8b 0d 21 ff 00 00 	mov    0xff21(%rip),%rcx        # 139d8 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x18>
    3ab7:	48 8b 40 e8          	mov    -0x18(%rax),%rax
    3abb:	48 c7 45 88 00 00 00 	movq   $0x0,-0x78(%rbp)
    3ac2:	00 
    3ac3:	48 89 8c 05 30 fe ff 	mov    %rcx,-0x1d0(%rbp,%rax,1)
    3aca:	ff 
    3acb:	48 c7 85 38 fe ff ff 	movq   $0x0,-0x1c8(%rbp)
    3ad2:	00 00 00 00 
    3ad6:	48 8b 05 f3 fe 00 00 	mov    0xfef3(%rip),%rax        # 139d0 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x10>
    3add:	31 f6                	xor    %esi,%esi
    3adf:	48 8b 78 e8          	mov    -0x18(%rax),%rdi
    3ae3:	4c 01 e7             	add    %r12,%rdi
    3ae6:	c5 f8 77             	vzeroupper 
    3ae9:	e8 42 f5 ff ff       	call   3030 <_ZNSt9basic_iosIcSt11char_traitsIcEE4initEPSt15basic_streambufIcS1_E@plt>
    3aee:	48 8b 05 eb fe 00 00 	mov    0xfeeb(%rip),%rax        # 139e0 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x20>
    3af5:	48 8b bd 28 fc ff ff 	mov    -0x3d8(%rbp),%rdi
    3afc:	48 89 85 40 fe ff ff 	mov    %rax,-0x1c0(%rbp)
    3b03:	48 03 78 e8          	add    -0x18(%rax),%rdi
    3b07:	48 8b 05 da fe 00 00 	mov    0xfeda(%rip),%rax        # 139e8 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x28>
    3b0e:	31 f6                	xor    %esi,%esi
    3b10:	48 89 07             	mov    %rax,(%rdi)
    3b13:	e8 18 f5 ff ff       	call   3030 <_ZNSt9basic_iosIcSt11char_traitsIcEE4initEPSt15basic_streambufIcS1_E@plt>
    3b18:	48 8b 05 a9 fe 00 00 	mov    0xfea9(%rip),%rax        # 139c8 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x8>
    3b1f:	48 8b 0d ca fe 00 00 	mov    0xfeca(%rip),%rcx        # 139f0 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x30>
    3b26:	48 8b 40 e8          	mov    -0x18(%rax),%rax
    3b2a:	48 89 8c 05 30 fe ff 	mov    %rcx,-0x1d0(%rbp,%rax,1)
    3b31:	ff 
    3b32:	48 8d 05 7f ff 00 00 	lea    0xff7f(%rip),%rax        # 13ab8 <_ZTVNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x18>
    3b39:	48 89 85 30 fe ff ff 	mov    %rax,-0x1d0(%rbp)
    3b40:	48 83 c0 50          	add    $0x50,%rax
    3b44:	48 89 85 b0 fe ff ff 	mov    %rax,-0x150(%rbp)
    3b4b:	48 83 e8 28          	sub    $0x28,%rax
    3b4f:	48 89 85 40 fe ff ff 	mov    %rax,-0x1c0(%rbp)
    3b56:	48 8d 05 d3 fd 00 00 	lea    0xfdd3(%rip),%rax        # 13930 <_ZTVSt15basic_streambufIcSt11char_traitsIcEE+0x10>
    3b5d:	48 89 85 48 fe ff ff 	mov    %rax,-0x1b8(%rbp)
    3b64:	48 8d 85 80 fe ff ff 	lea    -0x180(%rbp),%rax
    3b6b:	48 89 c7             	mov    %rax,%rdi
    3b6e:	48 89 85 00 fc ff ff 	mov    %rax,-0x400(%rbp)
    3b75:	48 c7 85 50 fe ff ff 	movq   $0x0,-0x1b0(%rbp)
    3b7c:	00 00 00 00 
    3b80:	48 c7 85 58 fe ff ff 	movq   $0x0,-0x1a8(%rbp)
    3b87:	00 00 00 00 
    3b8b:	48 c7 85 60 fe ff ff 	movq   $0x0,-0x1a0(%rbp)
    3b92:	00 00 00 00 
    3b96:	48 c7 85 68 fe ff ff 	movq   $0x0,-0x198(%rbp)
    3b9d:	00 00 00 00 
    3ba1:	48 c7 85 70 fe ff ff 	movq   $0x0,-0x190(%rbp)
    3ba8:	00 00 00 00 
    3bac:	48 c7 85 78 fe ff ff 	movq   $0x0,-0x188(%rbp)
    3bb3:	00 00 00 00 
    3bb7:	e8 54 f5 ff ff       	call   3110 <_ZNSt6localeC1Ev@plt>
    3bbc:	48 8d 05 5d fe 00 00 	lea    0xfe5d(%rip),%rax        # 13a20 <_ZTVNSt7__cxx1115basic_stringbufIcSt11char_traitsIcESaIcEEE+0x10>
    3bc3:	48 89 85 48 fe ff ff 	mov    %rax,-0x1b8(%rbp)
    3bca:	48 8d 85 a0 fe ff ff 	lea    -0x160(%rbp),%rax
    3bd1:	48 89 85 f8 fb ff ff 	mov    %rax,-0x408(%rbp)
    3bd8:	48 89 85 90 fe ff ff 	mov    %rax,-0x170(%rbp)
    3bdf:	48 8d 85 48 fe ff ff 	lea    -0x1b8(%rbp),%rax
    3be6:	48 89 c6             	mov    %rax,%rsi
    3be9:	4c 89 ff             	mov    %r15,%rdi
    3bec:	c7 85 88 fe ff ff 18 	movl   $0x18,-0x178(%rbp)
    3bf3:	00 00 00 
    3bf6:	48 c7 85 98 fe ff ff 	movq   $0x0,-0x168(%rbp)
    3bfd:	00 00 00 00 
    3c01:	c6 85 a0 fe ff ff 00 	movb   $0x0,-0x160(%rbp)
    3c08:	48 89 85 f0 fb ff ff 	mov    %rax,-0x410(%rbp)
    3c0f:	e8 1c f4 ff ff       	call   3030 <_ZNSt9basic_iosIcSt11char_traitsIcEE4initEPSt15basic_streambufIcS1_E@plt>
    3c14:	c5 fb 10 05 f4 cb 00 	vmovsd 0xcbf4(%rip),%xmm0        # 10810 <_ZTSN8argparse7details12parse_numberIfLNS0_12chars_formatE3EEE+0xb0>
    3c1b:	00 
    3c1c:	48 8b bd 28 fc ff ff 	mov    -0x3d8(%rbp),%rdi
    3c23:	e8 08 f5 ff ff       	call   3130 <_ZNSo9_M_insertIdEERSoT_@plt>
    3c28:	48 8b 85 18 fc ff ff 	mov    -0x3e8(%rbp),%rax
    3c2f:	48 c7 85 28 fd ff ff 	movq   $0x0,-0x2d8(%rbp)
    3c36:	00 00 00 00 
    3c3a:	48 89 85 20 fd ff ff 	mov    %rax,-0x2e0(%rbp)
    3c41:	48 8b 85 70 fe ff ff 	mov    -0x190(%rbp),%rax
    3c48:	c6 85 30 fd ff ff 00 	movb   $0x0,-0x2d0(%rbp)
    3c4f:	48 85 c0             	test   %rax,%rax
    3c52:	0f 84 2c 07 00 00    	je     4384 <main+0xb94>
    3c58:	4c 8b 85 60 fe ff ff 	mov    -0x1a0(%rbp),%r8
    3c5f:	48 8b 8d 68 fe ff ff 	mov    -0x198(%rbp),%rcx
    3c66:	4c 39 c0             	cmp    %r8,%rax
    3c69:	0f 87 9d 06 00 00    	ja     430c <main+0xb1c>
    3c6f:	48 8b bd 20 fc ff ff 	mov    -0x3e0(%rbp),%rdi
    3c76:	49 29 c8             	sub    %rcx,%r8
    3c79:	31 d2                	xor    %edx,%edx
    3c7b:	31 f6                	xor    %esi,%esi
    3c7d:	e8 2e f5 ff ff       	call   31b0 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE10_M_replaceEmmPKcm@plt>
    3c82:	48 8d 05 2f fe 00 00 	lea    0xfe2f(%rip),%rax        # 13ab8 <_ZTVNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x18>
    3c89:	48 89 85 30 fe ff ff 	mov    %rax,-0x1d0(%rbp)
    3c90:	c5 fa 7e 0d 10 ff 00 	vmovq  0xff10(%rip),%xmm1        # 13ba8 <_ZTVNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE@@Base+0x108>
    3c97:	00 
    3c98:	48 83 c0 50          	add    $0x50,%rax
    3c9c:	48 89 85 b0 fe ff ff 	mov    %rax,-0x150(%rbp)
    3ca3:	48 8b bd 90 fe ff ff 	mov    -0x170(%rbp),%rdi
    3caa:	48 8d 05 6f fd 00 00 	lea    0xfd6f(%rip),%rax        # 13a20 <_ZTVNSt7__cxx1115basic_stringbufIcSt11char_traitsIcESaIcEEE+0x10>
    3cb1:	c4 e3 f1 22 c0 01    	vpinsrq $0x1,%rax,%xmm1,%xmm0
    3cb7:	c5 f9 7f 85 40 fe ff 	vmovdqa %xmm0,-0x1c0(%rbp)
    3cbe:	ff 
    3cbf:	48 3b bd f8 fb ff ff 	cmp    -0x408(%rbp),%rdi
    3cc6:	74 10                	je     3cd8 <main+0x4e8>
    3cc8:	48 8b 85 a0 fe ff ff 	mov    -0x160(%rbp),%rax
    3ccf:	48 8d 70 01          	lea    0x1(%rax),%rsi
    3cd3:	e8 48 f6 ff ff       	call   3320 <_ZdlPvm@plt>
    3cd8:	48 8b bd 00 fc ff ff 	mov    -0x400(%rbp),%rdi
    3cdf:	48 8d 05 4a fc 00 00 	lea    0xfc4a(%rip),%rax        # 13930 <_ZTVSt15basic_streambufIcSt11char_traitsIcEE+0x10>
    3ce6:	48 89 85 48 fe ff ff 	mov    %rax,-0x1b8(%rbp)
    3ced:	e8 3e f5 ff ff       	call   3230 <_ZNSt6localeD1Ev@plt>
    3cf2:	48 8b 05 cf fc 00 00 	mov    0xfccf(%rip),%rax        # 139c8 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x8>
    3cf9:	48 8b 0d f0 fc 00 00 	mov    0xfcf0(%rip),%rcx        # 139f0 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x30>
    3d00:	48 8b 40 e8          	mov    -0x18(%rax),%rax
    3d04:	4c 89 ff             	mov    %r15,%rdi
    3d07:	48 89 8c 05 30 fe ff 	mov    %rcx,-0x1d0(%rbp,%rax,1)
    3d0e:	ff 
    3d0f:	48 8b 05 ca fc 00 00 	mov    0xfcca(%rip),%rax        # 139e0 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x20>
    3d16:	48 8b 0d cb fc 00 00 	mov    0xfccb(%rip),%rcx        # 139e8 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x28>
    3d1d:	48 89 85 40 fe ff ff 	mov    %rax,-0x1c0(%rbp)
    3d24:	48 8b 40 e8          	mov    -0x18(%rax),%rax
    3d28:	48 89 8c 05 40 fe ff 	mov    %rcx,-0x1c0(%rbp,%rax,1)
    3d2f:	ff 
    3d30:	48 8b 05 99 fc 00 00 	mov    0xfc99(%rip),%rax        # 139d0 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x10>
    3d37:	48 8b 0d 9a fc 00 00 	mov    0xfc9a(%rip),%rcx        # 139d8 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x18>
    3d3e:	48 89 85 30 fe ff ff 	mov    %rax,-0x1d0(%rbp)
    3d45:	48 8b 40 e8          	mov    -0x18(%rax),%rax
    3d49:	48 89 8c 05 30 fe ff 	mov    %rcx,-0x1d0(%rbp,%rax,1)
    3d50:	ff 
    3d51:	48 8d 05 90 fb 00 00 	lea    0xfb90(%rip),%rax        # 138e8 <_ZTVSt9basic_iosIcSt11char_traitsIcEE+0x10>
    3d58:	48 c7 85 38 fe ff ff 	movq   $0x0,-0x1c8(%rbp)
    3d5f:	00 00 00 00 
    3d63:	48 89 85 b0 fe ff ff 	mov    %rax,-0x150(%rbp)
    3d6a:	e8 31 f3 ff ff       	call   30a0 <_ZNSt8ios_baseD2Ev@plt>
    3d6f:	48 8b 95 20 fd ff ff 	mov    -0x2e0(%rbp),%rdx
    3d76:	48 8b 7b 58          	mov    0x58(%rbx),%rdi
    3d7a:	48 3b 95 18 fc ff ff 	cmp    -0x3e8(%rbp),%rdx
    3d81:	0f 84 a0 05 00 00    	je     4327 <main+0xb37>
    3d87:	48 8d 73 68          	lea    0x68(%rbx),%rsi
    3d8b:	48 8b 8d 30 fd ff ff 	mov    -0x2d0(%rbp),%rcx
    3d92:	48 8b 85 28 fd ff ff 	mov    -0x2d8(%rbp),%rax
    3d99:	48 39 f7             	cmp    %rsi,%rdi
    3d9c:	0f 84 bb 05 00 00    	je     435d <main+0xb6d>
    3da2:	c4 e1 f9 6e d0       	vmovq  %rax,%xmm2
    3da7:	c4 e3 e9 22 c1 01    	vpinsrq $0x1,%rcx,%xmm2,%xmm0
    3dad:	48 8b 73 68          	mov    0x68(%rbx),%rsi
    3db1:	48 89 53 58          	mov    %rdx,0x58(%rbx)
    3db5:	c5 fa 7f 43 60       	vmovdqu %xmm0,0x60(%rbx)
    3dba:	48 85 ff             	test   %rdi,%rdi
    3dbd:	0f 84 ae 05 00 00    	je     4371 <main+0xb81>
    3dc3:	48 89 bd 20 fd ff ff 	mov    %rdi,-0x2e0(%rbp)
    3dca:	48 89 b5 30 fd ff ff 	mov    %rsi,-0x2d0(%rbp)
    3dd1:	48 8b 85 20 fd ff ff 	mov    -0x2e0(%rbp),%rax
    3dd8:	48 c7 85 28 fd ff ff 	movq   $0x0,-0x2d8(%rbp)
    3ddf:	00 00 00 00 
    3de3:	c6 00 00             	movb   $0x0,(%rax)
    3de6:	48 8b bd 20 fd ff ff 	mov    -0x2e0(%rbp),%rdi
    3ded:	48 3b bd 18 fc ff ff 	cmp    -0x3e8(%rbp),%rdi
    3df4:	74 10                	je     3e06 <main+0x616>
    3df6:	48 8b 85 30 fd ff ff 	mov    -0x2d0(%rbp),%rax
    3dfd:	48 8d 70 01          	lea    0x1(%rax),%rsi
    3e01:	e8 1a f5 ff ff       	call   3320 <_ZdlPvm@plt>
    3e06:	48 8d 05 53 24 00 00 	lea    0x2453(%rip),%rax        # 6260 <_ZNSt3any17_Manager_internalIfE9_S_manageENS_3_OpEPKS_PNS_4_ArgE>
    3e0d:	4c 8d 7b 48          	lea    0x48(%rbx),%r15
    3e11:	48 89 85 30 fe ff ff 	mov    %rax,-0x1d0(%rbp)
    3e18:	48 c7 85 38 fe ff ff 	movq   $0x42200000,-0x1c8(%rbp)
    3e1f:	00 00 20 42 
    3e23:	4d 39 e7             	cmp    %r12,%r15
    3e26:	74 41                	je     3e69 <main+0x679>
    3e28:	48 8b 43 48          	mov    0x48(%rbx),%rax
    3e2c:	48 85 c0             	test   %rax,%rax
    3e2f:	74 14                	je     3e45 <main+0x655>
    3e31:	31 d2                	xor    %edx,%edx
    3e33:	4c 89 fe             	mov    %r15,%rsi
    3e36:	bf 03 00 00 00       	mov    $0x3,%edi
    3e3b:	ff d0                	call   *%rax
    3e3d:	48 c7 43 48 00 00 00 	movq   $0x0,0x48(%rbx)
    3e44:	00 
    3e45:	4c 89 bd 38 fc ff ff 	mov    %r15,-0x3c8(%rbp)
    3e4c:	4c 89 f2             	mov    %r14,%rdx
    3e4f:	4c 89 e6             	mov    %r12,%rsi
    3e52:	bf 04 00 00 00       	mov    $0x4,%edi
    3e57:	ff 95 30 fe ff ff    	call   *-0x1d0(%rbp)
    3e5d:	48 8b 85 30 fe ff ff 	mov    -0x1d0(%rbp),%rax
    3e64:	48 85 c0             	test   %rax,%rax
    3e67:	74 0c                	je     3e75 <main+0x685>
    3e69:	31 d2                	xor    %edx,%edx
    3e6b:	4c 89 e6             	mov    %r12,%rsi
    3e6e:	bf 03 00 00 00       	mov    $0x3,%edi
    3e73:	ff d0                	call   *%rax
    3e75:	48 8d 0d 54 25 00 00 	lea    0x2554(%rip),%rcx        # 63d0 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIfLNSB_12chars_formatE3EEEE10_M_managerERSt9_Any_dataRKSG_St18_Manager_operation>
    3e7c:	48 8d 05 2d 3c 00 00 	lea    0x3c2d(%rip),%rax        # 7ab0 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIfLNSB_12chars_formatE3EEEE9_M_invokeERKSt9_Any_dataS8_>
    3e83:	c4 e1 f9 6e c1       	vmovq  %rcx,%xmm0
    3e88:	c4 e3 f9 22 c0 01    	vpinsrq $0x1,%rax,%xmm0,%xmm0
    3e8e:	48 8d bb 88 00 00 00 	lea    0x88(%rbx),%rdi
    3e95:	4c 89 e6             	mov    %r12,%rsi
    3e98:	c5 f9 7f 85 40 fe ff 	vmovdqa %xmm0,-0x1c0(%rbp)
    3e9f:	ff 
    3ea0:	c6 85 50 fe ff ff 00 	movb   $0x0,-0x1b0(%rbp)
    3ea7:	e8 04 18 00 00       	call   56b0 <_ZZNSt8__detail9__variant17_Move_assign_baseILb0EJSt8functionIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEES2_IFvSB_EEEEaSEOSG_ENUlOT_T0_E_clIRSD_St17integral_constantImLm0EEEEDaSJ_SK_.isra.0>
    3eac:	0f b6 95 50 fe ff ff 	movzbl -0x1b0(%rbp),%edx
    3eb3:	4c 89 e6             	mov    %r12,%rsi
    3eb6:	4c 89 f7             	mov    %r14,%rdi
    3eb9:	48 8d 05 d0 f9 00 00 	lea    0xf9d0(%rip),%rax        # 13890 <_ZNSt8__detail9__variant12__gen_vtableIvOZNS0_16_Variant_storageILb0EJSt8functionIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEES3_IFvSC_EEEE8_M_resetEvEUlOT_E_JRSt7variantIJSE_SG_EEEE9_S_vtableE>
    3ec0:	ff 14 d0             	call   *(%rax,%rdx,8)
    3ec3:	80 bb a8 00 00 00 00 	cmpb   $0x0,0xa8(%rbx)
    3eca:	0f 85 d1 f6 ff ff    	jne    35a1 <main.cold>
    3ed0:	4c 8d bd 80 fc ff ff 	lea    -0x380(%rbp),%r15
    3ed7:	48 8d 35 79 c3 00 00 	lea    0xc379(%rip),%rsi        # 10257 <_fini+0x1036>
    3ede:	4c 89 ff             	mov    %r15,%rdi
    3ee1:	e8 0a 0a 00 00       	call   48f0 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEC2IS3_EEPKcRKS3_.constprop.0>
    3ee6:	48 8d 7b 28          	lea    0x28(%rbx),%rdi
    3eea:	4c 89 fe             	mov    %r15,%rsi
    3eed:	e8 7e 07 00 00       	call   4670 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEaSEOS4_.isra.0>
    3ef2:	48 8b bd 80 fc ff ff 	mov    -0x380(%rbp),%rdi
    3ef9:	48 8d 85 90 fc ff ff 	lea    -0x370(%rbp),%rax
    3f00:	48 39 c7             	cmp    %rax,%rdi
    3f03:	74 10                	je     3f15 <main+0x725>
    3f05:	48 8b 85 90 fc ff ff 	mov    -0x370(%rbp),%rax
    3f0c:	48 8d 70 01          	lea    0x1(%rax),%rsi
    3f10:	e8 0b f4 ff ff       	call   3320 <_ZdlPvm@plt>
    3f15:	48 8d 35 49 c3 00 00 	lea    0xc349(%rip),%rsi        # 10265 <_fini+0x1044>
    3f1c:	4c 89 ef             	mov    %r13,%rdi
    3f1f:	e8 7c a6 00 00       	call   e5a0 <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_>
    3f24:	48 89 c7             	mov    %rax,%rdi
    3f27:	4c 89 f6             	mov    %r14,%rsi
    3f2a:	c7 85 38 fc ff ff 05 	movl   $0x5,-0x3c8(%rbp)
    3f31:	00 00 00 
    3f34:	e8 67 57 00 00       	call   96a0 <_ZN8argparse8Argument13default_valueIiEERS0_OT_>
    3f39:	48 8d b8 88 00 00 00 	lea    0x88(%rax),%rdi
    3f40:	48 89 c3             	mov    %rax,%rbx
    3f43:	e8 68 18 00 00       	call   57b0 <_ZNSt7variantIJSt8functionIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEES0_IFvS9_EEEE7emplaceILm0EJN8argparse7details12parse_numberIiLi0EEEEEENSt9enable_ifIX18is_constructible_vINSt19variant_alternativeIXT_ESE_E4typeEDpT0_EERSN_E4typeEDpOSO_.isra.0>
    3f48:	4c 8d bd a0 fc ff ff 	lea    -0x360(%rbp),%r15
    3f4f:	48 8d 35 1b c3 00 00 	lea    0xc31b(%rip),%rsi        # 10271 <_fini+0x1050>
    3f56:	4c 89 ff             	mov    %r15,%rdi
    3f59:	e8 92 09 00 00       	call   48f0 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEC2IS3_EEPKcRKS3_.constprop.0>
    3f5e:	48 8d 7b 28          	lea    0x28(%rbx),%rdi
    3f62:	4c 89 fe             	mov    %r15,%rsi
    3f65:	e8 06 07 00 00       	call   4670 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEaSEOS4_.isra.0>
    3f6a:	48 8b bd a0 fc ff ff 	mov    -0x360(%rbp),%rdi
    3f71:	48 8d 85 b0 fc ff ff 	lea    -0x350(%rbp),%rax
    3f78:	48 39 c7             	cmp    %rax,%rdi
    3f7b:	74 10                	je     3f8d <main+0x79d>
    3f7d:	48 8b 85 b0 fc ff ff 	mov    -0x350(%rbp),%rax
    3f84:	48 8d 70 01          	lea    0x1(%rax),%rsi
    3f88:	e8 93 f3 ff ff       	call   3320 <_ZdlPvm@plt>
    3f8d:	48 8d 35 f5 c2 00 00 	lea    0xc2f5(%rip),%rsi        # 10289 <_fini+0x1068>
    3f94:	4c 89 ef             	mov    %r13,%rdi
    3f97:	e8 04 a6 00 00       	call   e5a0 <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_>
    3f9c:	48 89 c7             	mov    %rax,%rdi
    3f9f:	4c 89 f6             	mov    %r14,%rsi
    3fa2:	c7 85 38 fc ff ff 05 	movl   $0x5,-0x3c8(%rbp)
    3fa9:	00 00 00 
    3fac:	e8 ef 56 00 00       	call   96a0 <_ZN8argparse8Argument13default_valueIiEERS0_OT_>
    3fb1:	48 8d b8 88 00 00 00 	lea    0x88(%rax),%rdi
    3fb8:	48 89 c3             	mov    %rax,%rbx
    3fbb:	e8 f0 17 00 00       	call   57b0 <_ZNSt7variantIJSt8functionIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEES0_IFvS9_EEEE7emplaceILm0EJN8argparse7details12parse_numberIiLi0EEEEEENSt9enable_ifIX18is_constructible_vINSt19variant_alternativeIXT_ESE_E4typeEDpT0_EERSN_E4typeEDpOSO_.isra.0>
    3fc0:	4c 8d bd c0 fc ff ff 	lea    -0x340(%rbp),%r15
    3fc7:	48 8d 35 c4 c2 00 00 	lea    0xc2c4(%rip),%rsi        # 10292 <_fini+0x1071>
    3fce:	4c 89 ff             	mov    %r15,%rdi
    3fd1:	e8 1a 09 00 00       	call   48f0 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEC2IS3_EEPKcRKS3_.constprop.0>
    3fd6:	48 8d 7b 28          	lea    0x28(%rbx),%rdi
    3fda:	4c 89 fe             	mov    %r15,%rsi
    3fdd:	e8 8e 06 00 00       	call   4670 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEaSEOS4_.isra.0>
    3fe2:	48 8b bd c0 fc ff ff 	mov    -0x340(%rbp),%rdi
    3fe9:	48 8d 85 d0 fc ff ff 	lea    -0x330(%rbp),%rax
    3ff0:	48 39 c7             	cmp    %rax,%rdi
    3ff3:	74 10                	je     4005 <main+0x815>
    3ff5:	48 8b 85 d0 fc ff ff 	mov    -0x330(%rbp),%rax
    3ffc:	48 8d 70 01          	lea    0x1(%rax),%rsi
    4000:	e8 1b f3 ff ff       	call   3320 <_ZdlPvm@plt>
    4005:	48 8d 15 95 c2 00 00 	lea    0xc295(%rip),%rdx        # 102a1 <_fini+0x1080>
    400c:	48 8d 35 9b c2 00 00 	lea    0xc29b(%rip),%rsi        # 102ae <_fini+0x108d>
    4013:	4c 89 ef             	mov    %r13,%rdi
    4016:	e8 25 8b 00 00       	call   cb40 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_>
    401b:	48 8d 35 8f c2 00 00 	lea    0xc28f(%rip),%rsi        # 102b1 <_fini+0x1090>
    4022:	4c 89 e7             	mov    %r12,%rdi
    4025:	49 89 c7             	mov    %rax,%r15
    4028:	e8 c3 08 00 00       	call   48f0 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEC2IS3_EEPKcRKS3_.constprop.0>
    402d:	4c 89 e6             	mov    %r12,%rsi
    4030:	4c 89 ff             	mov    %r15,%rdi
    4033:	e8 18 4b 00 00       	call   8b50 <_ZN8argparse8Argument13default_valueINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEERS0_OT_>
    4038:	4c 8d bd e0 fc ff ff 	lea    -0x320(%rbp),%r15
    403f:	48 8d 35 6d c2 00 00 	lea    0xc26d(%rip),%rsi        # 102b3 <_fini+0x1092>
    4046:	4c 89 ff             	mov    %r15,%rdi
    4049:	48 89 c3             	mov    %rax,%rbx
    404c:	e8 9f 08 00 00       	call   48f0 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEC2IS3_EEPKcRKS3_.constprop.0>
    4051:	48 8d 7b 28          	lea    0x28(%rbx),%rdi
    4055:	4c 89 fe             	mov    %r15,%rsi
    4058:	e8 13 06 00 00       	call   4670 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEaSEOS4_.isra.0>
    405d:	48 8b bd e0 fc ff ff 	mov    -0x320(%rbp),%rdi
    4064:	48 8d 85 f0 fc ff ff 	lea    -0x310(%rbp),%rax
    406b:	48 39 c7             	cmp    %rax,%rdi
    406e:	74 10                	je     4080 <main+0x890>
    4070:	48 8b 85 f0 fc ff ff 	mov    -0x310(%rbp),%rax
    4077:	48 8d 70 01          	lea    0x1(%rax),%rsi
    407b:	e8 a0 f2 ff ff       	call   3320 <_ZdlPvm@plt>
    4080:	48 8b bd 30 fe ff ff 	mov    -0x1d0(%rbp),%rdi
    4087:	48 3b bd 28 fc ff ff 	cmp    -0x3d8(%rbp),%rdi
    408e:	74 10                	je     40a0 <main+0x8b0>
    4090:	48 8b 85 40 fe ff ff 	mov    -0x1c0(%rbp),%rax
    4097:	48 8d 70 01          	lea    0x1(%rax),%rsi
    409b:	e8 80 f2 ff ff       	call   3320 <_ZdlPvm@plt>
    40a0:	48 8d 35 28 c2 00 00 	lea    0xc228(%rip),%rsi        # 102cf <_fini+0x10ae>
    40a7:	4c 89 ef             	mov    %r13,%rdi
    40aa:	e8 f1 a4 00 00       	call   e5a0 <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_>
    40af:	48 8d 35 1f c2 00 00 	lea    0xc21f(%rip),%rsi        # 102d5 <_fini+0x10b4>
    40b6:	4c 89 e7             	mov    %r12,%rdi
    40b9:	49 89 c7             	mov    %rax,%r15
    40bc:	e8 2f 08 00 00       	call   48f0 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEC2IS3_EEPKcRKS3_.constprop.0>
    40c1:	4c 89 e6             	mov    %r12,%rsi
    40c4:	4c 89 ff             	mov    %r15,%rdi
    40c7:	e8 84 4a 00 00       	call   8b50 <_ZN8argparse8Argument13default_valueINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEERS0_OT_>
    40cc:	4c 8d bd 00 fd ff ff 	lea    -0x300(%rbp),%r15
    40d3:	48 8d 35 86 c3 00 00 	lea    0xc386(%rip),%rsi        # 10460 <_fini+0x123f>
    40da:	4c 89 ff             	mov    %r15,%rdi
    40dd:	48 89 c3             	mov    %rax,%rbx
    40e0:	e8 0b 08 00 00       	call   48f0 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEC2IS3_EEPKcRKS3_.constprop.0>
    40e5:	48 8d 7b 28          	lea    0x28(%rbx),%rdi
    40e9:	4c 89 fe             	mov    %r15,%rsi
    40ec:	e8 7f 05 00 00       	call   4670 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEaSEOS4_.isra.0>
    40f1:	48 8b bd 00 fd ff ff 	mov    -0x300(%rbp),%rdi
    40f8:	48 8d 85 10 fd ff ff 	lea    -0x2f0(%rbp),%rax
    40ff:	48 39 c7             	cmp    %rax,%rdi
    4102:	74 10                	je     4114 <main+0x924>
    4104:	48 8b 85 10 fd ff ff 	mov    -0x2f0(%rbp),%rax
    410b:	48 8d 70 01          	lea    0x1(%rax),%rsi
    410f:	e8 0c f2 ff ff       	call   3320 <_ZdlPvm@plt>
    4114:	48 8b bd 30 fe ff ff 	mov    -0x1d0(%rbp),%rdi
    411b:	48 3b bd 28 fc ff ff 	cmp    -0x3d8(%rbp),%rdi
    4122:	74 10                	je     4134 <main+0x944>
    4124:	48 8b 85 40 fe ff ff 	mov    -0x1c0(%rbp),%rax
    412b:	48 8d 70 01          	lea    0x1(%rax),%rsi
    412f:	e8 ec f1 ff ff       	call   3320 <_ZdlPvm@plt>
    4134:	48 8d 15 9e c1 00 00 	lea    0xc19e(%rip),%rdx        # 102d9 <_fini+0x10b8>
    413b:	48 8d 35 a1 c1 00 00 	lea    0xc1a1(%rip),%rsi        # 102e3 <_fini+0x10c2>
    4142:	4c 89 ef             	mov    %r13,%rdi
    4145:	e8 f6 89 00 00       	call   cb40 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_>
    414a:	48 89 c7             	mov    %rax,%rdi
    414d:	4c 89 f6             	mov    %r14,%rsi
    4150:	c6 85 38 fc ff ff 00 	movb   $0x0,-0x3c8(%rbp)
    4157:	e8 74 46 00 00       	call   87d0 <_ZN8argparse8Argument13default_valueIbEERS0_OT_>
    415c:	48 89 c7             	mov    %rax,%rdi
    415f:	4c 89 e6             	mov    %r12,%rsi
    4162:	4c 8d 3d c7 1f 00 00 	lea    0x1fc7(%rip),%r15        # 6130 <_ZNSt3any17_Manager_internalIbE9_S_manageENS_3_OpEPKS_PNS_4_ArgE>
    4169:	4c 89 bd 30 fe ff ff 	mov    %r15,-0x1d0(%rbp)
    4170:	48 c7 85 38 fe ff ff 	movq   $0x1,-0x1c8(%rbp)
    4177:	01 00 00 00 
    417b:	e8 c0 3a 00 00       	call   7c40 <_ZN8argparse8Argument14implicit_valueESt3any>
    4180:	48 8b bd 20 fc ff ff 	mov    -0x3e0(%rbp),%rdi
    4187:	48 8d 35 02 c3 00 00 	lea    0xc302(%rip),%rsi        # 10490 <_fini+0x126f>
    418e:	48 89 c3             	mov    %rax,%rbx
    4191:	e8 5a 07 00 00       	call   48f0 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEC2IS3_EEPKcRKS3_.constprop.0>
    4196:	48 8b b5 20 fc ff ff 	mov    -0x3e0(%rbp),%rsi
    419d:	48 8d 7b 28          	lea    0x28(%rbx),%rdi
    41a1:	e8 ca 04 00 00       	call   4670 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEaSEOS4_.isra.0>
    41a6:	48 8b bd 20 fd ff ff 	mov    -0x2e0(%rbp),%rdi
    41ad:	48 3b bd 18 fc ff ff 	cmp    -0x3e8(%rbp),%rdi
    41b4:	74 10                	je     41c6 <main+0x9d6>
    41b6:	48 8b 85 30 fd ff ff 	mov    -0x2d0(%rbp),%rax
    41bd:	48 8d 70 01          	lea    0x1(%rax),%rsi
    41c1:	e8 5a f1 ff ff       	call   3320 <_ZdlPvm@plt>
    41c6:	48 8b 85 30 fe ff ff 	mov    -0x1d0(%rbp),%rax
    41cd:	48 85 c0             	test   %rax,%rax
    41d0:	74 0c                	je     41de <main+0x9ee>
    41d2:	31 d2                	xor    %edx,%edx
    41d4:	4c 89 e6             	mov    %r12,%rsi
    41d7:	bf 03 00 00 00       	mov    $0x3,%edi
    41dc:	ff d0                	call   *%rax
    41de:	48 8d 35 01 c1 00 00 	lea    0xc101(%rip),%rsi        # 102e6 <_fini+0x10c5>
    41e5:	4c 89 ef             	mov    %r13,%rdi
    41e8:	e8 b3 a3 00 00       	call   e5a0 <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_>
    41ed:	48 89 c7             	mov    %rax,%rdi
    41f0:	4c 89 f6             	mov    %r14,%rsi
    41f3:	c6 85 38 fc ff ff 00 	movb   $0x0,-0x3c8(%rbp)
    41fa:	e8 d1 45 00 00       	call   87d0 <_ZN8argparse8Argument13default_valueIbEERS0_OT_>
    41ff:	48 8b b5 20 fc ff ff 	mov    -0x3e0(%rbp),%rsi
    4206:	48 89 c7             	mov    %rax,%rdi
    4209:	4c 89 bd 20 fd ff ff 	mov    %r15,-0x2e0(%rbp)
    4210:	48 c7 85 28 fd ff ff 	movq   $0x1,-0x2d8(%rbp)
    4217:	01 00 00 00 
    421b:	e8 20 3a 00 00       	call   7c40 <_ZN8argparse8Argument14implicit_valueESt3any>
    4220:	48 8d 35 91 c2 00 00 	lea    0xc291(%rip),%rsi        # 104b8 <_fini+0x1297>
    4227:	4c 89 e7             	mov    %r12,%rdi
    422a:	48 89 c3             	mov    %rax,%rbx
    422d:	e8 be 06 00 00       	call   48f0 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEC2IS3_EEPKcRKS3_.constprop.0>
    4232:	48 8d 7b 28          	lea    0x28(%rbx),%rdi
    4236:	4c 89 e6             	mov    %r12,%rsi
    4239:	e8 32 04 00 00       	call   4670 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEaSEOS4_.isra.0>
    423e:	48 8b bd 30 fe ff ff 	mov    -0x1d0(%rbp),%rdi
    4245:	48 3b bd 28 fc ff ff 	cmp    -0x3d8(%rbp),%rdi
    424c:	74 10                	je     425e <main+0xa6e>
    424e:	48 8b 85 40 fe ff ff 	mov    -0x1c0(%rbp),%rax
    4255:	48 8d 70 01          	lea    0x1(%rax),%rsi
    4259:	e8 c2 f0 ff ff       	call   3320 <_ZdlPvm@plt>
    425e:	48 8b 85 20 fd ff ff 	mov    -0x2e0(%rbp),%rax
    4265:	48 85 c0             	test   %rax,%rax
    4268:	74 10                	je     427a <main+0xa8a>
    426a:	48 8b b5 20 fc ff ff 	mov    -0x3e0(%rbp),%rsi
    4271:	31 d2                	xor    %edx,%edx
    4273:	bf 03 00 00 00       	mov    $0x3,%edi
    4278:	ff d0                	call   *%rax
    427a:	ba 04 00 00 00       	mov    $0x4,%edx
    427f:	48 8d 35 7a bf 00 00 	lea    0xbf7a(%rip),%rsi        # 10200 <_fini+0xfdf>
    4286:	48 8d 3d b3 fd 00 00 	lea    0xfdb3(%rip),%rdi        # 14040 <_ZSt4cout@@Base>
    428d:	e8 0e f1 ff ff       	call   33a0 <_ZSt16__ostream_insertIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_PKS3_l@plt>
    4292:	48 8d 3d a7 fd 00 00 	lea    0xfda7(%rip),%rdi        # 14040 <_ZSt4cout@@Base>
    4299:	e8 62 03 00 00       	call   4600 <_ZSt4endlIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_.isra.0>
    429e:	48 8b 95 08 fc ff ff 	mov    -0x3f8(%rbp),%rdx
    42a5:	8b b5 14 fc ff ff    	mov    -0x3ec(%rbp),%esi
    42ab:	4c 89 ef             	mov    %r13,%rdi
    42ae:	e8 ad 7e 00 00       	call   c160 <_ZN8argparse14ArgumentParser10parse_argsEiPKPKc>
    42b3:	ba 04 00 00 00       	mov    $0x4,%edx
    42b8:	48 8d 35 41 bf 00 00 	lea    0xbf41(%rip),%rsi        # 10200 <_fini+0xfdf>
    42bf:	48 8d 3d 7a fd 00 00 	lea    0xfd7a(%rip),%rdi        # 14040 <_ZSt4cout@@Base>
    42c6:	e8 d5 f0 ff ff       	call   33a0 <_ZSt16__ostream_insertIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_PKS3_l@plt>
    42cb:	48 8d 3d 6e fd 00 00 	lea    0xfd6e(%rip),%rdi        # 14040 <_ZSt4cout@@Base>
    42d2:	e8 29 03 00 00       	call   4600 <_ZSt4endlIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_.isra.0>
    42d7:	4c 89 ef             	mov    %r13,%rdi
    42da:	e8 b1 4e 00 00       	call   9190 <_ZN8argparse14ArgumentParserD1Ev>
    42df:	48 8b 45 c8          	mov    -0x38(%rbp),%rax
    42e3:	64 48 2b 04 25 28 00 	sub    %fs:0x28,%rax
    42ea:	00 00 
    42ec:	0f 85 c0 00 00 00    	jne    43b2 <main+0xbc2>
    42f2:	48 81 c4 e0 03 00 00 	add    $0x3e0,%rsp
    42f9:	5b                   	pop    %rbx
    42fa:	41 5a                	pop    %r10
    42fc:	41 5c                	pop    %r12
    42fe:	41 5d                	pop    %r13
    4300:	41 5e                	pop    %r14
    4302:	41 5f                	pop    %r15
    4304:	31 c0                	xor    %eax,%eax
    4306:	5d                   	pop    %rbp
    4307:	49 8d 62 f8          	lea    -0x8(%r10),%rsp
    430b:	c3                   	ret    
    430c:	48 29 c8             	sub    %rcx,%rax
    430f:	48 8b bd 20 fc ff ff 	mov    -0x3e0(%rbp),%rdi
    4316:	49 89 c0             	mov    %rax,%r8
    4319:	31 d2                	xor    %edx,%edx
    431b:	31 f6                	xor    %esi,%esi
    431d:	e8 8e ee ff ff       	call   31b0 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE10_M_replaceEmmPKcm@plt>
    4322:	e9 5b f9 ff ff       	jmp    3c82 <main+0x492>
    4327:	48 8b 95 28 fd ff ff 	mov    -0x2d8(%rbp),%rdx
    432e:	48 85 d2             	test   %rdx,%rdx
    4331:	74 1d                	je     4350 <main+0xb60>
    4333:	48 83 fa 01          	cmp    $0x1,%rdx
    4337:	74 63                	je     439c <main+0xbac>
    4339:	48 8b b5 18 fc ff ff 	mov    -0x3e8(%rbp),%rsi
    4340:	e8 fb ed ff ff       	call   3140 <memcpy@plt>
    4345:	48 8b 7b 58          	mov    0x58(%rbx),%rdi
    4349:	48 8b 95 28 fd ff ff 	mov    -0x2d8(%rbp),%rdx
    4350:	48 89 53 60          	mov    %rdx,0x60(%rbx)
    4354:	c6 04 17 00          	movb   $0x0,(%rdi,%rdx,1)
    4358:	e9 74 fa ff ff       	jmp    3dd1 <main+0x5e1>
    435d:	c4 e1 f9 6e d8       	vmovq  %rax,%xmm3
    4362:	48 89 53 58          	mov    %rdx,0x58(%rbx)
    4366:	c4 e3 e1 22 c1 01    	vpinsrq $0x1,%rcx,%xmm3,%xmm0
    436c:	c5 fa 7f 43 60       	vmovdqu %xmm0,0x60(%rbx)
    4371:	48 8b 85 18 fc ff ff 	mov    -0x3e8(%rbp),%rax
    4378:	48 89 85 20 fd ff ff 	mov    %rax,-0x2e0(%rbp)
    437f:	e9 4d fa ff ff       	jmp    3dd1 <main+0x5e1>
    4384:	48 8b bd 20 fc ff ff 	mov    -0x3e0(%rbp),%rdi
    438b:	48 8d b5 90 fe ff ff 	lea    -0x170(%rbp),%rsi
    4392:	e8 09 ee ff ff       	call   31a0 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE9_M_assignERKS4_@plt>
    4397:	e9 e6 f8 ff ff       	jmp    3c82 <main+0x492>
    439c:	0f b6 85 30 fd ff ff 	movzbl -0x2d0(%rbp),%eax
    43a3:	88 07                	mov    %al,(%rdi)
    43a5:	48 8b 7b 58          	mov    0x58(%rbx),%rdi
    43a9:	48 8b 95 28 fd ff ff 	mov    -0x2d8(%rbp),%rdx
    43b0:	eb 9e                	jmp    4350 <main+0xb60>
    43b2:	e8 99 ed ff ff       	call   3150 <__stack_chk_fail@plt>
    43b7:	48 89 c3             	mov    %rax,%rbx
    43ba:	e9 e9 f1 ff ff       	jmp    35a8 <main.cold+0x7>
    43bf:	49 89 c4             	mov    %rax,%r12
    43c2:	e9 2c f2 ff ff       	jmp    35f3 <main.cold+0x52>
    43c7:	49 89 c4             	mov    %rax,%r12
    43ca:	e9 24 f2 ff ff       	jmp    35f3 <main.cold+0x52>
    43cf:	48 89 c3             	mov    %rax,%rbx
    43d2:	e9 41 f2 ff ff       	jmp    3618 <main.cold+0x77>
    43d7:	49 89 c4             	mov    %rax,%r12
    43da:	e9 59 f2 ff ff       	jmp    3638 <main.cold+0x97>
    43df:	48 89 c7             	mov    %rax,%rdi
    43e2:	48 89 d0             	mov    %rdx,%rax
    43e5:	e9 7f f2 ff ff       	jmp    3669 <main.cold+0xc8>
    43ea:	48 89 c3             	mov    %rax,%rbx
    43ed:	c5 f8 77             	vzeroupper 
    43f0:	e9 d6 f1 ff ff       	jmp    35cb <main.cold+0x2a>
    43f5:	48 89 c3             	mov    %rax,%rbx
    43f8:	c5 f8 77             	vzeroupper 
    43fb:	e9 e2 f2 ff ff       	jmp    36e2 <main.cold+0x141>
    4400:	e9 e1 f1 ff ff       	jmp    35e6 <main.cold+0x45>
    4405:	49 89 c4             	mov    %rax,%r12
    4408:	e9 0e f3 ff ff       	jmp    371b <main.cold+0x17a>
    440d:	48 89 c3             	mov    %rax,%rbx
    4410:	e9 5c f3 ff ff       	jmp    3771 <main.cold+0x1d0>
    4415:	49 89 c4             	mov    %rax,%r12
    4418:	c5 f8 77             	vzeroupper 
    441b:	e9 21 f3 ff ff       	jmp    3741 <main.cold+0x1a0>
    4420:	48 89 c3             	mov    %rax,%rbx
    4423:	e9 8b f2 ff ff       	jmp    36b3 <main.cold+0x112>
    4428:	0f 1f 84 00 00 00 00 	nopl   0x0(%rax,%rax,1)
    442f:	00 

0000000000004430 <_GLOBAL__sub_I_main>:
    4430:	48 83 ec 08          	sub    $0x8,%rsp
    4434:	48 8d 3d b5 fe 00 00 	lea    0xfeb5(%rip),%rdi        # 142f0 <_ZStL8__ioinit>
    443b:	e8 10 ee ff ff       	call   3250 <_ZNSt8ios_base4InitC1Ev@plt>
    4440:	48 8b 3d 79 fb 00 00 	mov    0xfb79(%rip),%rdi        # 13fc0 <_ZNSt8ios_base4InitD1Ev@Base>
    4447:	48 8d 15 b2 fb 00 00 	lea    0xfbb2(%rip),%rdx        # 14000 <__dso_handle>
    444e:	48 8d 35 9b fe 00 00 	lea    0xfe9b(%rip),%rsi        # 142f0 <_ZStL8__ioinit>
    4455:	48 83 c4 08          	add    $0x8,%rsp
    4459:	e9 82 ee ff ff       	jmp    32e0 <__cxa_atexit@plt>

000000000000445e <_start>:
    445e:	48 31 ed             	xor    %rbp,%rbp
    4461:	48 89 e7             	mov    %rsp,%rdi
    4464:	48 8d 35 45 f7 00 00 	lea    0xf745(%rip),%rsi        # 13bb0 <_DYNAMIC>
    446b:	48 83 e4 f0          	and    $0xfffffffffffffff0,%rsp
    446f:	e8 00 00 00 00       	call   4474 <_start_c>

0000000000004474 <_start_c>:
    4474:	48 8b 37             	mov    (%rdi),%rsi
    4477:	48 8d 57 08          	lea    0x8(%rdi),%rdx
    447b:	45 31 c9             	xor    %r9d,%r9d
    447e:	4c 8d 05 9c ad 00 00 	lea    0xad9c(%rip),%r8        # f221 <_fini>
    4485:	48 8d 0d 74 eb ff ff 	lea    -0x148c(%rip),%rcx        # 3000 <_init>
    448c:	48 8d 3d 5d f3 ff ff 	lea    -0xca3(%rip),%rdi        # 37f0 <main>
    4493:	e9 a8 ed ff ff       	jmp    3240 <__libc_start_main@plt>
    4498:	0f 1f 84 00 00 00 00 	nopl   0x0(%rax,%rax,1)
    449f:	00 

00000000000044a0 <deregister_tm_clones>:
    44a0:	48 8d 3d 71 fb 00 00 	lea    0xfb71(%rip),%rdi        # 14018 <__TMC_END__>
    44a7:	48 8d 05 6a fb 00 00 	lea    0xfb6a(%rip),%rax        # 14018 <__TMC_END__>
    44ae:	48 39 f8             	cmp    %rdi,%rax
    44b1:	74 15                	je     44c8 <deregister_tm_clones+0x28>
    44b3:	48 8b 05 ee fa 00 00 	mov    0xfaee(%rip),%rax        # 13fa8 <_ITM_deregisterTMCloneTable@Base>
    44ba:	48 85 c0             	test   %rax,%rax
    44bd:	74 09                	je     44c8 <deregister_tm_clones+0x28>
    44bf:	ff e0                	jmp    *%rax
    44c1:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
    44c8:	c3                   	ret    
    44c9:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)

00000000000044d0 <register_tm_clones>:
    44d0:	48 8d 3d 41 fb 00 00 	lea    0xfb41(%rip),%rdi        # 14018 <__TMC_END__>
    44d7:	48 8d 35 3a fb 00 00 	lea    0xfb3a(%rip),%rsi        # 14018 <__TMC_END__>
    44de:	48 29 fe             	sub    %rdi,%rsi
    44e1:	48 89 f0             	mov    %rsi,%rax
    44e4:	48 c1 ee 3f          	shr    $0x3f,%rsi
    44e8:	48 c1 f8 03          	sar    $0x3,%rax
    44ec:	48 01 c6             	add    %rax,%rsi
    44ef:	48 d1 fe             	sar    %rsi
    44f2:	74 14                	je     4508 <register_tm_clones+0x38>
    44f4:	48 8b 05 e5 fa 00 00 	mov    0xfae5(%rip),%rax        # 13fe0 <_ITM_registerTMCloneTable@Base>
    44fb:	48 85 c0             	test   %rax,%rax
    44fe:	74 08                	je     4508 <register_tm_clones+0x38>
    4500:	ff e0                	jmp    *%rax
    4502:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)
    4508:	c3                   	ret    
    4509:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)

0000000000004510 <__do_global_dtors_aux>:
    4510:	80 3d 89 fd 00 00 00 	cmpb   $0x0,0xfd89(%rip)        # 142a0 <completed.2>
    4517:	0f 85 93 00 00 00    	jne    45b0 <__do_global_dtors_aux+0xa0>
    451d:	55                   	push   %rbp
    451e:	48 83 3d 7a fa 00 00 	cmpq   $0x0,0xfa7a(%rip)        # 13fa0 <__cxa_finalize@Base>
    4525:	00 
    4526:	48 89 e5             	mov    %rsp,%rbp
    4529:	41 54                	push   %r12
    452b:	53                   	push   %rbx
    452c:	74 0c                	je     453a <__do_global_dtors_aux+0x2a>
    452e:	48 8b 3d cb fa 00 00 	mov    0xfacb(%rip),%rdi        # 14000 <__dso_handle>
    4535:	e8 96 ee ff ff       	call   33d0 <__cxa_finalize@plt>
    453a:	48 8d 05 0f f3 00 00 	lea    0xf30f(%rip),%rax        # 13850 <__DTOR_LIST__>
    4541:	48 8d 1d 10 f3 00 00 	lea    0xf310(%rip),%rbx        # 13858 <__DTOR_END__>
    4548:	48 29 c3             	sub    %rax,%rbx
    454b:	49 89 c4             	mov    %rax,%r12
    454e:	48 8b 05 53 fd 00 00 	mov    0xfd53(%rip),%rax        # 142a8 <dtor_idx.1>
    4555:	48 c1 fb 03          	sar    $0x3,%rbx
    4559:	48 83 eb 01          	sub    $0x1,%rbx
    455d:	48 39 d8             	cmp    %rbx,%rax
    4560:	73 21                	jae    4583 <__do_global_dtors_aux+0x73>
    4562:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)
    4568:	48 83 c0 01          	add    $0x1,%rax
    456c:	48 89 05 35 fd 00 00 	mov    %rax,0xfd35(%rip)        # 142a8 <dtor_idx.1>
    4573:	41 ff 14 c4          	call   *(%r12,%rax,8)
    4577:	48 8b 05 2a fd 00 00 	mov    0xfd2a(%rip),%rax        # 142a8 <dtor_idx.1>
    457e:	48 39 d8             	cmp    %rbx,%rax
    4581:	72 e5                	jb     4568 <__do_global_dtors_aux+0x58>
    4583:	e8 18 ff ff ff       	call   44a0 <deregister_tm_clones>
    4588:	48 83 3d 40 fa 00 00 	cmpq   $0x0,0xfa40(%rip)        # 13fd0 <__deregister_frame_info@GCC_3.0>
    458f:	00 
    4590:	74 0c                	je     459e <__do_global_dtors_aux+0x8e>
    4592:	48 8d 3d 87 c5 00 00 	lea    0xc587(%rip),%rdi        # 10b20 <__EH_FRAME_BEGIN__>
    4599:	e8 42 ee ff ff       	call   33e0 <__deregister_frame_info@plt>
    459e:	5b                   	pop    %rbx
    459f:	41 5c                	pop    %r12
    45a1:	c6 05 f8 fc 00 00 01 	movb   $0x1,0xfcf8(%rip)        # 142a0 <completed.2>
    45a8:	5d                   	pop    %rbp
    45a9:	c3                   	ret    
    45aa:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)
    45b0:	c3                   	ret    
    45b1:	66 66 2e 0f 1f 84 00 	data16 cs nopw 0x0(%rax,%rax,1)
    45b8:	00 00 00 00 
    45bc:	0f 1f 40 00          	nopl   0x0(%rax)

00000000000045c0 <frame_dummy>:
    45c0:	48 83 3d f0 f9 00 00 	cmpq   $0x0,0xf9f0(%rip)        # 13fb8 <__register_frame_info@GCC_3.0>
    45c7:	00 
    45c8:	74 26                	je     45f0 <frame_dummy+0x30>
    45ca:	55                   	push   %rbp
    45cb:	48 8d 35 ee fc 00 00 	lea    0xfcee(%rip),%rsi        # 142c0 <object.0>
    45d2:	48 8d 3d 47 c5 00 00 	lea    0xc547(%rip),%rdi        # 10b20 <__EH_FRAME_BEGIN__>
    45d9:	48 89 e5             	mov    %rsp,%rbp
    45dc:	e8 f7 ed ff ff       	call   33d8 <__register_frame_info@plt>
    45e1:	5d                   	pop    %rbp
    45e2:	e9 e9 fe ff ff       	jmp    44d0 <register_tm_clones>
    45e7:	66 0f 1f 84 00 00 00 	nopw   0x0(%rax,%rax,1)
    45ee:	00 00 
    45f0:	e9 db fe ff ff       	jmp    44d0 <register_tm_clones>
    45f5:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
    45fc:	00 00 00 
    45ff:	90                   	nop

0000000000004600 <_ZSt4endlIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_.isra.0>:
    4600:	41 54                	push   %r12
    4602:	55                   	push   %rbp
    4603:	48 83 ec 08          	sub    $0x8,%rsp
    4607:	48 8b 07             	mov    (%rdi),%rax
    460a:	48 8b 40 e8          	mov    -0x18(%rax),%rax
    460e:	4c 8b a4 07 f0 00 00 	mov    0xf0(%rdi,%rax,1),%r12
    4615:	00 
    4616:	4d 85 e4             	test   %r12,%r12
    4619:	74 44                	je     465f <_ZSt4endlIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_.isra.0+0x5f>
    461b:	41 80 7c 24 38 00    	cmpb   $0x0,0x38(%r12)
    4621:	48 89 fd             	mov    %rdi,%rbp
    4624:	74 1d                	je     4643 <_ZSt4endlIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_.isra.0+0x43>
    4626:	41 0f be 74 24 43    	movsbl 0x43(%r12),%esi
    462c:	48 89 ef             	mov    %rbp,%rdi
    462f:	e8 5c eb ff ff       	call   3190 <_ZNSo3putEc@plt>
    4634:	48 83 c4 08          	add    $0x8,%rsp
    4638:	5d                   	pop    %rbp
    4639:	48 89 c7             	mov    %rax,%rdi
    463c:	41 5c                	pop    %r12
    463e:	e9 1d eb ff ff       	jmp    3160 <_ZNSo5flushEv@plt>
    4643:	4c 89 e7             	mov    %r12,%rdi
    4646:	e8 25 ea ff ff       	call   3070 <_ZNKSt5ctypeIcE13_M_widen_initEv@plt>
    464b:	49 8b 04 24          	mov    (%r12),%rax
    464f:	be 0a 00 00 00       	mov    $0xa,%esi
    4654:	4c 89 e7             	mov    %r12,%rdi
    4657:	ff 50 30             	call   *0x30(%rax)
    465a:	0f be f0             	movsbl %al,%esi
    465d:	eb cd                	jmp    462c <_ZSt4endlIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_.isra.0+0x2c>
    465f:	e8 6c eb ff ff       	call   31d0 <_ZSt16__throw_bad_castv@plt>
    4664:	66 66 2e 0f 1f 84 00 	data16 cs nopw 0x0(%rax,%rax,1)
    466b:	00 00 00 00 
    466f:	90                   	nop

0000000000004670 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEaSEOS4_.isra.0>:
    4670:	55                   	push   %rbp
    4671:	48 89 fd             	mov    %rdi,%rbp
    4674:	53                   	push   %rbx
    4675:	48 89 f3             	mov    %rsi,%rbx
    4678:	48 8d 43 10          	lea    0x10(%rbx),%rax
    467c:	48 83 ec 08          	sub    $0x8,%rsp
    4680:	48 8b 36             	mov    (%rsi),%rsi
    4683:	48 8b 3f             	mov    (%rdi),%rdi
    4686:	48 8b 53 08          	mov    0x8(%rbx),%rdx
    468a:	48 39 c6             	cmp    %rax,%rsi
    468d:	74 53                	je     46e2 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEaSEOS4_.isra.0+0x72>
    468f:	48 8d 4d 10          	lea    0x10(%rbp),%rcx
    4693:	48 39 cf             	cmp    %rcx,%rdi
    4696:	74 35                	je     46cd <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEaSEOS4_.isra.0+0x5d>
    4698:	48 89 75 00          	mov    %rsi,0x0(%rbp)
    469c:	48 89 55 08          	mov    %rdx,0x8(%rbp)
    46a0:	48 8b 4d 10          	mov    0x10(%rbp),%rcx
    46a4:	48 8b 53 10          	mov    0x10(%rbx),%rdx
    46a8:	48 89 55 10          	mov    %rdx,0x10(%rbp)
    46ac:	48 85 ff             	test   %rdi,%rdi
    46af:	74 2c                	je     46dd <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEaSEOS4_.isra.0+0x6d>
    46b1:	48 89 3b             	mov    %rdi,(%rbx)
    46b4:	48 89 4b 10          	mov    %rcx,0x10(%rbx)
    46b8:	48 8b 03             	mov    (%rbx),%rax
    46bb:	48 c7 43 08 00 00 00 	movq   $0x0,0x8(%rbx)
    46c2:	00 
    46c3:	c6 00 00             	movb   $0x0,(%rax)
    46c6:	48 83 c4 08          	add    $0x8,%rsp
    46ca:	5b                   	pop    %rbx
    46cb:	5d                   	pop    %rbp
    46cc:	c3                   	ret    
    46cd:	48 89 75 00          	mov    %rsi,0x0(%rbp)
    46d1:	48 89 55 08          	mov    %rdx,0x8(%rbp)
    46d5:	48 8b 53 10          	mov    0x10(%rbx),%rdx
    46d9:	48 89 55 10          	mov    %rdx,0x10(%rbp)
    46dd:	48 89 03             	mov    %rax,(%rbx)
    46e0:	eb d6                	jmp    46b8 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEaSEOS4_.isra.0+0x48>
    46e2:	48 85 d2             	test   %rdx,%rdx
    46e5:	74 13                	je     46fa <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEaSEOS4_.isra.0+0x8a>
    46e7:	48 83 fa 01          	cmp    $0x1,%rdx
    46eb:	74 17                	je     4704 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEaSEOS4_.isra.0+0x94>
    46ed:	e8 4e ea ff ff       	call   3140 <memcpy@plt>
    46f2:	48 8b 7d 00          	mov    0x0(%rbp),%rdi
    46f6:	48 8b 53 08          	mov    0x8(%rbx),%rdx
    46fa:	48 89 55 08          	mov    %rdx,0x8(%rbp)
    46fe:	c6 04 17 00          	movb   $0x0,(%rdi,%rdx,1)
    4702:	eb b4                	jmp    46b8 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEaSEOS4_.isra.0+0x48>
    4704:	0f b6 43 10          	movzbl 0x10(%rbx),%eax
    4708:	88 07                	mov    %al,(%rdi)
    470a:	48 8b 7d 00          	mov    0x0(%rbp),%rdi
    470e:	48 8b 53 08          	mov    0x8(%rbx),%rdx
    4712:	eb e6                	jmp    46fa <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEaSEOS4_.isra.0+0x8a>
    4714:	66 66 2e 0f 1f 84 00 	data16 cs nopw 0x0(%rax,%rax,1)
    471b:	00 00 00 00 
    471f:	90                   	nop

0000000000004720 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE8_M_eraseEPSt13_Rb_tree_nodeISA_E.isra.0>:
    4720:	41 57                	push   %r15
    4722:	41 56                	push   %r14
    4724:	41 55                	push   %r13
    4726:	41 54                	push   %r12
    4728:	55                   	push   %rbp
    4729:	53                   	push   %rbx
    472a:	48 83 ec 28          	sub    $0x28,%rsp
    472e:	48 89 7c 24 10       	mov    %rdi,0x10(%rsp)
    4733:	48 85 ff             	test   %rdi,%rdi
    4736:	0f 84 9b 01 00 00    	je     48d7 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE8_M_eraseEPSt13_Rb_tree_nodeISA_E.isra.0+0x1b7>
    473c:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
    4741:	4c 8b 70 18          	mov    0x18(%rax),%r14
    4745:	4d 85 f6             	test   %r14,%r14
    4748:	0f 84 67 01 00 00    	je     48b5 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE8_M_eraseEPSt13_Rb_tree_nodeISA_E.isra.0+0x195>
    474e:	4d 8b 7e 18          	mov    0x18(%r14),%r15
    4752:	4d 85 ff             	test   %r15,%r15
    4755:	0f 84 3c 01 00 00    	je     4897 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE8_M_eraseEPSt13_Rb_tree_nodeISA_E.isra.0+0x177>
    475b:	49 8b 47 18          	mov    0x18(%r15),%rax
    475f:	48 89 44 24 08       	mov    %rax,0x8(%rsp)
    4764:	48 85 c0             	test   %rax,%rax
    4767:	0f 84 0c 01 00 00    	je     4879 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE8_M_eraseEPSt13_Rb_tree_nodeISA_E.isra.0+0x159>
    476d:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
    4772:	48 8b 68 18          	mov    0x18(%rax),%rbp
    4776:	48 85 ed             	test   %rbp,%rbp
    4779:	0f 84 af 00 00 00    	je     482e <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE8_M_eraseEPSt13_Rb_tree_nodeISA_E.isra.0+0x10e>
    477f:	4c 8b 6d 18          	mov    0x18(%rbp),%r13
    4783:	4d 85 ed             	test   %r13,%r13
    4786:	74 64                	je     47ec <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE8_M_eraseEPSt13_Rb_tree_nodeISA_E.isra.0+0xcc>
    4788:	4d 8b 65 18          	mov    0x18(%r13),%r12
    478c:	4d 85 e4             	test   %r12,%r12
    478f:	74 7f                	je     4810 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE8_M_eraseEPSt13_Rb_tree_nodeISA_E.isra.0+0xf0>
    4791:	4d 8b 4c 24 18       	mov    0x18(%r12),%r9
    4796:	4d 85 c9             	test   %r9,%r9
    4799:	0f 84 b1 00 00 00    	je     4850 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE8_M_eraseEPSt13_Rb_tree_nodeISA_E.isra.0+0x130>
    479f:	49 8b 59 18          	mov    0x18(%r9),%rbx
    47a3:	48 85 db             	test   %rbx,%rbx
    47a6:	74 29                	je     47d1 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE8_M_eraseEPSt13_Rb_tree_nodeISA_E.isra.0+0xb1>
    47a8:	48 8b 7b 18          	mov    0x18(%rbx),%rdi
    47ac:	4c 89 4c 24 18       	mov    %r9,0x18(%rsp)
    47b1:	e8 6a ff ff ff       	call   4720 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE8_M_eraseEPSt13_Rb_tree_nodeISA_E.isra.0>
    47b6:	48 89 df             	mov    %rbx,%rdi
    47b9:	48 8b 5b 10          	mov    0x10(%rbx),%rbx
    47bd:	be 38 00 00 00       	mov    $0x38,%esi
    47c2:	e8 59 eb ff ff       	call   3320 <_ZdlPvm@plt>
    47c7:	48 85 db             	test   %rbx,%rbx
    47ca:	4c 8b 4c 24 18       	mov    0x18(%rsp),%r9
    47cf:	75 d7                	jne    47a8 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE8_M_eraseEPSt13_Rb_tree_nodeISA_E.isra.0+0x88>
    47d1:	49 8b 59 10          	mov    0x10(%r9),%rbx
    47d5:	be 38 00 00 00       	mov    $0x38,%esi
    47da:	4c 89 cf             	mov    %r9,%rdi
    47dd:	e8 3e eb ff ff       	call   3320 <_ZdlPvm@plt>
    47e2:	48 85 db             	test   %rbx,%rbx
    47e5:	74 69                	je     4850 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE8_M_eraseEPSt13_Rb_tree_nodeISA_E.isra.0+0x130>
    47e7:	49 89 d9             	mov    %rbx,%r9
    47ea:	eb b3                	jmp    479f <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE8_M_eraseEPSt13_Rb_tree_nodeISA_E.isra.0+0x7f>
    47ec:	4c 8b 65 10          	mov    0x10(%rbp),%r12
    47f0:	be 38 00 00 00       	mov    $0x38,%esi
    47f5:	48 89 ef             	mov    %rbp,%rdi
    47f8:	e8 23 eb ff ff       	call   3320 <_ZdlPvm@plt>
    47fd:	4d 85 e4             	test   %r12,%r12
    4800:	74 2c                	je     482e <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE8_M_eraseEPSt13_Rb_tree_nodeISA_E.isra.0+0x10e>
    4802:	4c 89 e5             	mov    %r12,%rbp
    4805:	e9 75 ff ff ff       	jmp    477f <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE8_M_eraseEPSt13_Rb_tree_nodeISA_E.isra.0+0x5f>
    480a:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)
    4810:	4d 8b 65 10          	mov    0x10(%r13),%r12
    4814:	be 38 00 00 00       	mov    $0x38,%esi
    4819:	4c 89 ef             	mov    %r13,%rdi
    481c:	e8 ff ea ff ff       	call   3320 <_ZdlPvm@plt>
    4821:	4d 85 e4             	test   %r12,%r12
    4824:	74 c6                	je     47ec <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE8_M_eraseEPSt13_Rb_tree_nodeISA_E.isra.0+0xcc>
    4826:	4d 89 e5             	mov    %r12,%r13
    4829:	e9 5a ff ff ff       	jmp    4788 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE8_M_eraseEPSt13_Rb_tree_nodeISA_E.isra.0+0x68>
    482e:	48 8b 7c 24 08       	mov    0x8(%rsp),%rdi
    4833:	be 38 00 00 00       	mov    $0x38,%esi
    4838:	48 8b 6f 10          	mov    0x10(%rdi),%rbp
    483c:	e8 df ea ff ff       	call   3320 <_ZdlPvm@plt>
    4841:	48 85 ed             	test   %rbp,%rbp
    4844:	74 33                	je     4879 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE8_M_eraseEPSt13_Rb_tree_nodeISA_E.isra.0+0x159>
    4846:	48 89 6c 24 08       	mov    %rbp,0x8(%rsp)
    484b:	e9 1d ff ff ff       	jmp    476d <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE8_M_eraseEPSt13_Rb_tree_nodeISA_E.isra.0+0x4d>
    4850:	49 8b 44 24 10       	mov    0x10(%r12),%rax
    4855:	be 38 00 00 00       	mov    $0x38,%esi
    485a:	4c 89 e7             	mov    %r12,%rdi
    485d:	48 89 44 24 18       	mov    %rax,0x18(%rsp)
    4862:	e8 b9 ea ff ff       	call   3320 <_ZdlPvm@plt>
    4867:	48 8b 44 24 18       	mov    0x18(%rsp),%rax
    486c:	48 85 c0             	test   %rax,%rax
    486f:	74 9f                	je     4810 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE8_M_eraseEPSt13_Rb_tree_nodeISA_E.isra.0+0xf0>
    4871:	49 89 c4             	mov    %rax,%r12
    4874:	e9 18 ff ff ff       	jmp    4791 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE8_M_eraseEPSt13_Rb_tree_nodeISA_E.isra.0+0x71>
    4879:	49 8b 5f 10          	mov    0x10(%r15),%rbx
    487d:	be 38 00 00 00       	mov    $0x38,%esi
    4882:	4c 89 ff             	mov    %r15,%rdi
    4885:	e8 96 ea ff ff       	call   3320 <_ZdlPvm@plt>
    488a:	48 85 db             	test   %rbx,%rbx
    488d:	74 08                	je     4897 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE8_M_eraseEPSt13_Rb_tree_nodeISA_E.isra.0+0x177>
    488f:	49 89 df             	mov    %rbx,%r15
    4892:	e9 c4 fe ff ff       	jmp    475b <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE8_M_eraseEPSt13_Rb_tree_nodeISA_E.isra.0+0x3b>
    4897:	49 8b 5e 10          	mov    0x10(%r14),%rbx
    489b:	be 38 00 00 00       	mov    $0x38,%esi
    48a0:	4c 89 f7             	mov    %r14,%rdi
    48a3:	e8 78 ea ff ff       	call   3320 <_ZdlPvm@plt>
    48a8:	48 85 db             	test   %rbx,%rbx
    48ab:	74 08                	je     48b5 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE8_M_eraseEPSt13_Rb_tree_nodeISA_E.isra.0+0x195>
    48ad:	49 89 de             	mov    %rbx,%r14
    48b0:	e9 99 fe ff ff       	jmp    474e <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE8_M_eraseEPSt13_Rb_tree_nodeISA_E.isra.0+0x2e>
    48b5:	48 8b 7c 24 10       	mov    0x10(%rsp),%rdi
    48ba:	be 38 00 00 00       	mov    $0x38,%esi
    48bf:	48 8b 5f 10          	mov    0x10(%rdi),%rbx
    48c3:	e8 58 ea ff ff       	call   3320 <_ZdlPvm@plt>
    48c8:	48 85 db             	test   %rbx,%rbx
    48cb:	74 0a                	je     48d7 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE8_M_eraseEPSt13_Rb_tree_nodeISA_E.isra.0+0x1b7>
    48cd:	48 89 5c 24 10       	mov    %rbx,0x10(%rsp)
    48d2:	e9 65 fe ff ff       	jmp    473c <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE8_M_eraseEPSt13_Rb_tree_nodeISA_E.isra.0+0x1c>
    48d7:	48 83 c4 28          	add    $0x28,%rsp
    48db:	5b                   	pop    %rbx
    48dc:	5d                   	pop    %rbp
    48dd:	41 5c                	pop    %r12
    48df:	41 5d                	pop    %r13
    48e1:	41 5e                	pop    %r14
    48e3:	41 5f                	pop    %r15
    48e5:	c3                   	ret    
    48e6:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
    48ed:	00 00 00 

00000000000048f0 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEC2IS3_EEPKcRKS3_.constprop.0>:
    48f0:	41 55                	push   %r13
    48f2:	4c 8d 6f 10          	lea    0x10(%rdi),%r13
    48f6:	41 54                	push   %r12
    48f8:	55                   	push   %rbp
    48f9:	53                   	push   %rbx
    48fa:	48 83 ec 18          	sub    $0x18,%rsp
    48fe:	64 48 8b 04 25 28 00 	mov    %fs:0x28,%rax
    4905:	00 00 
    4907:	48 89 44 24 08       	mov    %rax,0x8(%rsp)
    490c:	31 c0                	xor    %eax,%eax
    490e:	4c 89 2f             	mov    %r13,(%rdi)
    4911:	48 85 f6             	test   %rsi,%rsi
    4914:	0f 84 89 00 00 00    	je     49a3 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEC2IS3_EEPKcRKS3_.constprop.0+0xb3>
    491a:	48 89 fb             	mov    %rdi,%rbx
    491d:	48 89 f7             	mov    %rsi,%rdi
    4920:	48 89 f5             	mov    %rsi,%rbp
    4923:	e8 08 ea ff ff       	call   3330 <strlen@plt>
    4928:	48 89 04 24          	mov    %rax,(%rsp)
    492c:	49 89 c4             	mov    %rax,%r12
    492f:	48 83 f8 0f          	cmp    $0xf,%rax
    4933:	77 3e                	ja     4973 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEC2IS3_EEPKcRKS3_.constprop.0+0x83>
    4935:	48 83 f8 01          	cmp    $0x1,%rax
    4939:	75 31                	jne    496c <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEC2IS3_EEPKcRKS3_.constprop.0+0x7c>
    493b:	0f b6 45 00          	movzbl 0x0(%rbp),%eax
    493f:	88 43 10             	mov    %al,0x10(%rbx)
    4942:	48 8b 04 24          	mov    (%rsp),%rax
    4946:	48 8b 13             	mov    (%rbx),%rdx
    4949:	48 89 43 08          	mov    %rax,0x8(%rbx)
    494d:	c6 04 02 00          	movb   $0x0,(%rdx,%rax,1)
    4951:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
    4956:	64 48 2b 04 25 28 00 	sub    %fs:0x28,%rax
    495d:	00 00 
    495f:	75 3d                	jne    499e <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEC2IS3_EEPKcRKS3_.constprop.0+0xae>
    4961:	48 83 c4 18          	add    $0x18,%rsp
    4965:	5b                   	pop    %rbx
    4966:	5d                   	pop    %rbp
    4967:	41 5c                	pop    %r12
    4969:	41 5d                	pop    %r13
    496b:	c3                   	ret    
    496c:	48 85 c0             	test   %rax,%rax
    496f:	74 d1                	je     4942 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEC2IS3_EEPKcRKS3_.constprop.0+0x52>
    4971:	eb 1b                	jmp    498e <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEC2IS3_EEPKcRKS3_.constprop.0+0x9e>
    4973:	48 89 e6             	mov    %rsp,%rsi
    4976:	31 d2                	xor    %edx,%edx
    4978:	48 89 df             	mov    %rbx,%rdi
    497b:	e8 80 e9 ff ff       	call   3300 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE9_M_createERmm@plt>
    4980:	48 89 03             	mov    %rax,(%rbx)
    4983:	49 89 c5             	mov    %rax,%r13
    4986:	48 8b 04 24          	mov    (%rsp),%rax
    498a:	48 89 43 10          	mov    %rax,0x10(%rbx)
    498e:	4c 89 e2             	mov    %r12,%rdx
    4991:	48 89 ee             	mov    %rbp,%rsi
    4994:	4c 89 ef             	mov    %r13,%rdi
    4997:	e8 a4 e7 ff ff       	call   3140 <memcpy@plt>
    499c:	eb a4                	jmp    4942 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEC2IS3_EEPKcRKS3_.constprop.0+0x52>
    499e:	e8 ad e7 ff ff       	call   3150 <__stack_chk_fail@plt>
    49a3:	48 8d 3d 56 b6 00 00 	lea    0xb656(%rip),%rdi        # 10000 <_fini+0xddf>
    49aa:	e8 11 ea ff ff       	call   33c0 <_ZSt19__throw_logic_errorPKc@plt>
    49af:	90                   	nop

00000000000049b0 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0>:
    49b0:	41 57                	push   %r15
    49b2:	49 89 c9             	mov    %rcx,%r9
    49b5:	49 89 ff             	mov    %rdi,%r15
    49b8:	41 56                	push   %r14
    49ba:	41 55                	push   %r13
    49bc:	49 89 f5             	mov    %rsi,%r13
    49bf:	41 54                	push   %r12
    49c1:	55                   	push   %rbp
    49c2:	53                   	push   %rbx
    49c3:	48 89 f3             	mov    %rsi,%rbx
    49c6:	48 c1 e3 05          	shl    $0x5,%rbx
    49ca:	48 83 ec 68          	sub    $0x68,%rsp
    49ce:	48 89 54 24 28       	mov    %rdx,0x28(%rsp)
    49d3:	48 01 fb             	add    %rdi,%rbx
    49d6:	4c 8d 73 10          	lea    0x10(%rbx),%r14
    49da:	64 48 8b 04 25 28 00 	mov    %fs:0x28,%rax
    49e1:	00 00 
    49e3:	48 89 44 24 58       	mov    %rax,0x58(%rsp)
    49e8:	31 c0                	xor    %eax,%eax
    49ea:	48 8d 42 ff          	lea    -0x1(%rdx),%rax
    49ee:	49 89 c0             	mov    %rax,%r8
    49f1:	49 c1 e8 3f          	shr    $0x3f,%r8
    49f5:	49 01 c0             	add    %rax,%r8
    49f8:	49 d1 f8             	sar    %r8
    49fb:	4c 39 c6             	cmp    %r8,%rsi
    49fe:	0f 8d 8c 05 00 00    	jge    4f90 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x5e0>
    4a04:	49 89 f4             	mov    %rsi,%r12
    4a07:	eb 78                	jmp    4a81 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0xd1>
    4a09:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
    4a10:	48 0f 43 d0          	cmovae %rax,%rdx
    4a14:	48 0f 43 d9          	cmovae %rcx,%rbx
    4a18:	49 0f 43 eb          	cmovae %r11,%rbp
    4a1c:	49 c1 e4 05          	shl    $0x5,%r12
    4a20:	48 8b 03             	mov    (%rbx),%rax
    4a23:	4d 01 fc             	add    %r15,%r12
    4a26:	48 8d 4b 10          	lea    0x10(%rbx),%rcx
    4a2a:	49 8b 3c 24          	mov    (%r12),%rdi
    4a2e:	48 39 c8             	cmp    %rcx,%rax
    4a31:	0f 84 d1 00 00 00    	je     4b08 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x158>
    4a37:	4c 39 f7             	cmp    %r14,%rdi
    4a3a:	0f 84 28 03 00 00    	je     4d68 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x3b8>
    4a40:	49 89 04 24          	mov    %rax,(%r12)
    4a44:	49 89 54 24 08       	mov    %rdx,0x8(%r12)
    4a49:	49 8b 74 24 10       	mov    0x10(%r12),%rsi
    4a4e:	48 8b 43 10          	mov    0x10(%rbx),%rax
    4a52:	49 89 44 24 10       	mov    %rax,0x10(%r12)
    4a57:	48 85 ff             	test   %rdi,%rdi
    4a5a:	0f 84 1a 03 00 00    	je     4d7a <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x3ca>
    4a60:	48 89 3b             	mov    %rdi,(%rbx)
    4a63:	48 89 73 10          	mov    %rsi,0x10(%rbx)
    4a67:	48 c7 43 08 00 00 00 	movq   $0x0,0x8(%rbx)
    4a6e:	00 
    4a6f:	c6 07 00             	movb   $0x0,(%rdi)
    4a72:	4c 39 c5             	cmp    %r8,%rbp
    4a75:	0f 8d e8 00 00 00    	jge    4b63 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x1b3>
    4a7b:	49 89 ce             	mov    %rcx,%r14
    4a7e:	49 89 ec             	mov    %rbp,%r12
    4a81:	49 8d 4c 24 01       	lea    0x1(%r12),%rcx
    4a86:	4c 8d 1c 09          	lea    (%rcx,%rcx,1),%r11
    4a8a:	49 8d 6b ff          	lea    -0x1(%r11),%rbp
    4a8e:	48 89 eb             	mov    %rbp,%rbx
    4a91:	48 c1 e3 05          	shl    $0x5,%rbx
    4a95:	48 c1 e1 06          	shl    $0x6,%rcx
    4a99:	4c 01 fb             	add    %r15,%rbx
    4a9c:	4c 01 f9             	add    %r15,%rcx
    4a9f:	48 8b 41 08          	mov    0x8(%rcx),%rax
    4aa3:	48 8b 53 08          	mov    0x8(%rbx),%rdx
    4aa7:	48 39 d0             	cmp    %rdx,%rax
    4aaa:	0f 85 60 ff ff ff    	jne    4a10 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x60>
    4ab0:	48 85 d2             	test   %rdx,%rdx
    4ab3:	0f 84 97 03 00 00    	je     4e50 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x4a0>
    4ab9:	48 8b 33             	mov    (%rbx),%rsi
    4abc:	48 8b 39             	mov    (%rcx),%rdi
    4abf:	4c 89 44 24 20       	mov    %r8,0x20(%rsp)
    4ac4:	4c 89 4c 24 18       	mov    %r9,0x18(%rsp)
    4ac9:	4c 89 5c 24 10       	mov    %r11,0x10(%rsp)
    4ace:	48 89 54 24 08       	mov    %rdx,0x8(%rsp)
    4ad3:	48 89 0c 24          	mov    %rcx,(%rsp)
    4ad7:	e8 94 e7 ff ff       	call   3270 <memcmp@plt>
    4adc:	48 8b 0c 24          	mov    (%rsp),%rcx
    4ae0:	4c 8b 5c 24 10       	mov    0x10(%rsp),%r11
    4ae5:	85 c0                	test   %eax,%eax
    4ae7:	48 0f 49 d9          	cmovns %rcx,%rbx
    4aeb:	49 0f 49 eb          	cmovns %r11,%rbp
    4aef:	48 8b 54 24 08       	mov    0x8(%rsp),%rdx
    4af4:	4c 8b 4c 24 18       	mov    0x18(%rsp),%r9
    4af9:	4c 8b 44 24 20       	mov    0x20(%rsp),%r8
    4afe:	e9 19 ff ff ff       	jmp    4a1c <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x6c>
    4b03:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)
    4b08:	48 85 d2             	test   %rdx,%rdx
    4b0b:	74 36                	je     4b43 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x193>
    4b0d:	48 83 fa 01          	cmp    $0x1,%rdx
    4b11:	0f 84 99 03 00 00    	je     4eb0 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x500>
    4b17:	48 89 ce             	mov    %rcx,%rsi
    4b1a:	4c 89 44 24 10       	mov    %r8,0x10(%rsp)
    4b1f:	4c 89 4c 24 08       	mov    %r9,0x8(%rsp)
    4b24:	48 89 0c 24          	mov    %rcx,(%rsp)
    4b28:	e8 13 e6 ff ff       	call   3140 <memcpy@plt>
    4b2d:	48 8b 53 08          	mov    0x8(%rbx),%rdx
    4b31:	49 8b 3c 24          	mov    (%r12),%rdi
    4b35:	4c 8b 44 24 10       	mov    0x10(%rsp),%r8
    4b3a:	4c 8b 4c 24 08       	mov    0x8(%rsp),%r9
    4b3f:	48 8b 0c 24          	mov    (%rsp),%rcx
    4b43:	49 89 54 24 08       	mov    %rdx,0x8(%r12)
    4b48:	c6 04 17 00          	movb   $0x0,(%rdi,%rdx,1)
    4b4c:	48 c7 43 08 00 00 00 	movq   $0x0,0x8(%rbx)
    4b53:	00 
    4b54:	48 8b 3b             	mov    (%rbx),%rdi
    4b57:	c6 07 00             	movb   $0x0,(%rdi)
    4b5a:	4c 39 c5             	cmp    %r8,%rbp
    4b5d:	0f 8c 18 ff ff ff    	jl     4a7b <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0xcb>
    4b63:	48 8b 44 24 28       	mov    0x28(%rsp),%rax
    4b68:	a8 01                	test   $0x1,%al
    4b6a:	75 1d                	jne    4b89 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x1d9>
    4b6c:	49 89 c6             	mov    %rax,%r14
    4b6f:	49 83 ee 02          	sub    $0x2,%r14
    4b73:	4c 89 f0             	mov    %r14,%rax
    4b76:	48 c1 e8 3f          	shr    $0x3f,%rax
    4b7a:	49 01 c6             	add    %rax,%r14
    4b7d:	49 d1 fe             	sar    %r14
    4b80:	49 39 ee             	cmp    %rbp,%r14
    4b83:	0f 84 6f 03 00 00    	je     4ef8 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x548>
    4b89:	4d 8b 11             	mov    (%r9),%r10
    4b8c:	4c 8d 64 24 40       	lea    0x40(%rsp),%r12
    4b91:	49 8d 41 10          	lea    0x10(%r9),%rax
    4b95:	4c 89 64 24 30       	mov    %r12,0x30(%rsp)
    4b9a:	49 39 c2             	cmp    %rax,%r10
    4b9d:	0f 84 3d 03 00 00    	je     4ee0 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x530>
    4ba3:	49 8b 51 10          	mov    0x10(%r9),%rdx
    4ba7:	4c 89 54 24 30       	mov    %r10,0x30(%rsp)
    4bac:	48 89 54 24 40       	mov    %rdx,0x40(%rsp)
    4bb1:	49 89 01             	mov    %rax,(%r9)
    4bb4:	48 8d 45 ff          	lea    -0x1(%rbp),%rax
    4bb8:	49 89 c6             	mov    %rax,%r14
    4bbb:	49 8b 71 08          	mov    0x8(%r9),%rsi
    4bbf:	49 c1 ee 3f          	shr    $0x3f,%r14
    4bc3:	49 01 c6             	add    %rax,%r14
    4bc6:	48 89 74 24 38       	mov    %rsi,0x38(%rsp)
    4bcb:	49 c7 41 08 00 00 00 	movq   $0x0,0x8(%r9)
    4bd2:	00 
    4bd3:	41 c6 41 10 00       	movb   $0x0,0x10(%r9)
    4bd8:	49 d1 fe             	sar    %r14
    4bdb:	4c 39 ed             	cmp    %r13,%rbp
    4bde:	0f 8f 8d 00 00 00    	jg     4c71 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x2c1>
    4be4:	e9 a4 02 00 00       	jmp    4e8d <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x4dd>
    4be9:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
    4bf0:	0f 86 d2 02 00 00    	jbe    4ec8 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x518>
    4bf6:	48 c1 e5 05          	shl    $0x5,%rbp
    4bfa:	4c 8b 03             	mov    (%rbx),%r8
    4bfd:	4c 01 fd             	add    %r15,%rbp
    4c00:	48 8d 73 10          	lea    0x10(%rbx),%rsi
    4c04:	48 8b 7d 00          	mov    0x0(%rbp),%rdi
    4c08:	49 39 f0             	cmp    %rsi,%r8
    4c0b:	0f 84 ef 01 00 00    	je     4e00 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x450>
    4c11:	48 39 f9             	cmp    %rdi,%rcx
    4c14:	0f 84 be 01 00 00    	je     4dd8 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x428>
    4c1a:	4c 89 45 00          	mov    %r8,0x0(%rbp)
    4c1e:	48 89 55 08          	mov    %rdx,0x8(%rbp)
    4c22:	48 8b 45 10          	mov    0x10(%rbp),%rax
    4c26:	48 8b 53 10          	mov    0x10(%rbx),%rdx
    4c2a:	48 89 55 10          	mov    %rdx,0x10(%rbp)
    4c2e:	48 85 ff             	test   %rdi,%rdi
    4c31:	0f 84 b1 01 00 00    	je     4de8 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x438>
    4c37:	48 89 3b             	mov    %rdi,(%rbx)
    4c3a:	48 89 43 10          	mov    %rax,0x10(%rbx)
    4c3e:	48 89 f1             	mov    %rsi,%rcx
    4c41:	49 8d 56 ff          	lea    -0x1(%r14),%rdx
    4c45:	48 c7 43 08 00 00 00 	movq   $0x0,0x8(%rbx)
    4c4c:	00 
    4c4d:	48 89 d0             	mov    %rdx,%rax
    4c50:	c6 07 00             	movb   $0x0,(%rdi)
    4c53:	48 c1 e8 3f          	shr    $0x3f,%rax
    4c57:	48 01 d0             	add    %rdx,%rax
    4c5a:	48 8b 74 24 38       	mov    0x38(%rsp),%rsi
    4c5f:	48 d1 f8             	sar    %rax
    4c62:	4d 39 f5             	cmp    %r14,%r13
    4c65:	0f 8d 1d 02 00 00    	jge    4e88 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x4d8>
    4c6b:	4c 89 f5             	mov    %r14,%rbp
    4c6e:	49 89 c6             	mov    %rax,%r14
    4c71:	4c 89 f3             	mov    %r14,%rbx
    4c74:	48 c1 e3 05          	shl    $0x5,%rbx
    4c78:	4c 01 fb             	add    %r15,%rbx
    4c7b:	48 8b 53 08          	mov    0x8(%rbx),%rdx
    4c7f:	48 39 d6             	cmp    %rdx,%rsi
    4c82:	0f 85 68 ff ff ff    	jne    4bf0 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x240>
    4c88:	4c 8b 54 24 30       	mov    0x30(%rsp),%r10
    4c8d:	48 85 d2             	test   %rdx,%rdx
    4c90:	0f 84 8a 01 00 00    	je     4e20 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x470>
    4c96:	4c 8b 03             	mov    (%rbx),%r8
    4c99:	4c 89 d6             	mov    %r10,%rsi
    4c9c:	4c 89 c7             	mov    %r8,%rdi
    4c9f:	48 89 4c 24 18       	mov    %rcx,0x18(%rsp)
    4ca4:	48 89 54 24 10       	mov    %rdx,0x10(%rsp)
    4ca9:	4c 89 54 24 08       	mov    %r10,0x8(%rsp)
    4cae:	4c 89 04 24          	mov    %r8,(%rsp)
    4cb2:	e8 b9 e5 ff ff       	call   3270 <memcmp@plt>
    4cb7:	85 c0                	test   %eax,%eax
    4cb9:	4c 8b 04 24          	mov    (%rsp),%r8
    4cbd:	4c 8b 54 24 08       	mov    0x8(%rsp),%r10
    4cc2:	48 8b 54 24 10       	mov    0x10(%rsp),%rdx
    4cc7:	48 8b 4c 24 18       	mov    0x18(%rsp),%rcx
    4ccc:	0f 85 b6 00 00 00    	jne    4d88 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x3d8>
    4cd2:	48 c1 e5 05          	shl    $0x5,%rbp
    4cd6:	49 8d 1c 2f          	lea    (%r15,%rbp,1),%rbx
    4cda:	48 8b 3b             	mov    (%rbx),%rdi
    4cdd:	4d 39 e2             	cmp    %r12,%r10
    4ce0:	0f 84 8a 02 00 00    	je     4f70 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x5c0>
    4ce6:	48 8b 44 24 40       	mov    0x40(%rsp),%rax
    4ceb:	48 39 f9             	cmp    %rdi,%rcx
    4cee:	0f 84 6c 01 00 00    	je     4e60 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x4b0>
    4cf4:	c4 e1 f9 6e ca       	vmovq  %rdx,%xmm1
    4cf9:	c4 e3 f1 22 c0 01    	vpinsrq $0x1,%rax,%xmm1,%xmm0
    4cff:	48 8b 4b 10          	mov    0x10(%rbx),%rcx
    4d03:	4c 89 13             	mov    %r10,(%rbx)
    4d06:	c5 fa 7f 43 08       	vmovdqu %xmm0,0x8(%rbx)
    4d0b:	48 85 ff             	test   %rdi,%rdi
    4d0e:	0f 84 5f 01 00 00    	je     4e73 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x4c3>
    4d14:	48 89 7c 24 30       	mov    %rdi,0x30(%rsp)
    4d19:	48 89 4c 24 40       	mov    %rcx,0x40(%rsp)
    4d1e:	48 c7 44 24 38 00 00 	movq   $0x0,0x38(%rsp)
    4d25:	00 00 
    4d27:	c6 07 00             	movb   $0x0,(%rdi)
    4d2a:	48 8b 7c 24 30       	mov    0x30(%rsp),%rdi
    4d2f:	4c 39 e7             	cmp    %r12,%rdi
    4d32:	74 0e                	je     4d42 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x392>
    4d34:	48 8b 44 24 40       	mov    0x40(%rsp),%rax
    4d39:	48 8d 70 01          	lea    0x1(%rax),%rsi
    4d3d:	e8 de e5 ff ff       	call   3320 <_ZdlPvm@plt>
    4d42:	48 8b 44 24 58       	mov    0x58(%rsp),%rax
    4d47:	64 48 2b 04 25 28 00 	sub    %fs:0x28,%rax
    4d4e:	00 00 
    4d50:	0f 85 b8 02 00 00    	jne    500e <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x65e>
    4d56:	48 83 c4 68          	add    $0x68,%rsp
    4d5a:	5b                   	pop    %rbx
    4d5b:	5d                   	pop    %rbp
    4d5c:	41 5c                	pop    %r12
    4d5e:	41 5d                	pop    %r13
    4d60:	41 5e                	pop    %r14
    4d62:	41 5f                	pop    %r15
    4d64:	c3                   	ret    
    4d65:	0f 1f 00             	nopl   (%rax)
    4d68:	49 89 04 24          	mov    %rax,(%r12)
    4d6c:	49 89 54 24 08       	mov    %rdx,0x8(%r12)
    4d71:	48 8b 43 10          	mov    0x10(%rbx),%rax
    4d75:	49 89 44 24 10       	mov    %rax,0x10(%r12)
    4d7a:	48 89 0b             	mov    %rcx,(%rbx)
    4d7d:	48 89 cf             	mov    %rcx,%rdi
    4d80:	e9 e2 fc ff ff       	jmp    4a67 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0xb7>
    4d85:	0f 1f 00             	nopl   (%rax)
    4d88:	0f 89 85 02 00 00    	jns    5013 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x663>
    4d8e:	48 c1 e5 05          	shl    $0x5,%rbp
    4d92:	4c 01 fd             	add    %r15,%rbp
    4d95:	48 8d 73 10          	lea    0x10(%rbx),%rsi
    4d99:	48 8b 7d 00          	mov    0x0(%rbp),%rdi
    4d9d:	49 39 f0             	cmp    %rsi,%r8
    4da0:	0f 85 6b fe ff ff    	jne    4c11 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x261>
    4da6:	48 83 fa 01          	cmp    $0x1,%rdx
    4daa:	74 5f                	je     4e0b <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x45b>
    4dac:	48 89 34 24          	mov    %rsi,(%rsp)
    4db0:	e8 8b e3 ff ff       	call   3140 <memcpy@plt>
    4db5:	48 8b 53 08          	mov    0x8(%rbx),%rdx
    4db9:	48 8b 7d 00          	mov    0x0(%rbp),%rdi
    4dbd:	48 8b 34 24          	mov    (%rsp),%rsi
    4dc1:	48 89 55 08          	mov    %rdx,0x8(%rbp)
    4dc5:	c6 04 17 00          	movb   $0x0,(%rdi,%rdx,1)
    4dc9:	48 89 f1             	mov    %rsi,%rcx
    4dcc:	48 8b 3b             	mov    (%rbx),%rdi
    4dcf:	e9 6d fe ff ff       	jmp    4c41 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x291>
    4dd4:	0f 1f 40 00          	nopl   0x0(%rax)
    4dd8:	4c 89 45 00          	mov    %r8,0x0(%rbp)
    4ddc:	48 89 55 08          	mov    %rdx,0x8(%rbp)
    4de0:	48 8b 43 10          	mov    0x10(%rbx),%rax
    4de4:	48 89 45 10          	mov    %rax,0x10(%rbp)
    4de8:	48 89 33             	mov    %rsi,(%rbx)
    4deb:	48 89 f1             	mov    %rsi,%rcx
    4dee:	48 89 f7             	mov    %rsi,%rdi
    4df1:	e9 4b fe ff ff       	jmp    4c41 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x291>
    4df6:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
    4dfd:	00 00 00 
    4e00:	48 85 d2             	test   %rdx,%rdx
    4e03:	74 bc                	je     4dc1 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x411>
    4e05:	48 83 fa 01          	cmp    $0x1,%rdx
    4e09:	75 a1                	jne    4dac <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x3fc>
    4e0b:	0f b6 43 10          	movzbl 0x10(%rbx),%eax
    4e0f:	88 07                	mov    %al,(%rdi)
    4e11:	48 8b 53 08          	mov    0x8(%rbx),%rdx
    4e15:	48 8b 7d 00          	mov    0x0(%rbp),%rdi
    4e19:	eb a6                	jmp    4dc1 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x411>
    4e1b:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)
    4e20:	48 c1 e5 05          	shl    $0x5,%rbp
    4e24:	49 8d 1c 2f          	lea    (%r15,%rbp,1),%rbx
    4e28:	48 8b 3b             	mov    (%rbx),%rdi
    4e2b:	4d 39 e2             	cmp    %r12,%r10
    4e2e:	0f 85 b2 fe ff ff    	jne    4ce6 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x336>
    4e34:	48 89 53 08          	mov    %rdx,0x8(%rbx)
    4e38:	c6 04 17 00          	movb   $0x0,(%rdi,%rdx,1)
    4e3c:	48 8b 7c 24 30       	mov    0x30(%rsp),%rdi
    4e41:	e9 d8 fe ff ff       	jmp    4d1e <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x36e>
    4e46:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
    4e4d:	00 00 00 
    4e50:	48 89 cb             	mov    %rcx,%rbx
    4e53:	4c 89 dd             	mov    %r11,%rbp
    4e56:	e9 c1 fb ff ff       	jmp    4a1c <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x6c>
    4e5b:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)
    4e60:	c4 e1 f9 6e d2       	vmovq  %rdx,%xmm2
    4e65:	4c 89 13             	mov    %r10,(%rbx)
    4e68:	c4 e3 e9 22 c0 01    	vpinsrq $0x1,%rax,%xmm2,%xmm0
    4e6e:	c5 fa 7f 43 08       	vmovdqu %xmm0,0x8(%rbx)
    4e73:	4c 89 64 24 30       	mov    %r12,0x30(%rsp)
    4e78:	4c 8d 64 24 40       	lea    0x40(%rsp),%r12
    4e7d:	4c 89 e7             	mov    %r12,%rdi
    4e80:	e9 99 fe ff ff       	jmp    4d1e <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x36e>
    4e85:	0f 1f 00             	nopl   (%rax)
    4e88:	4c 8b 54 24 30       	mov    0x30(%rsp),%r10
    4e8d:	48 8b 3b             	mov    (%rbx),%rdi
    4e90:	4d 39 e2             	cmp    %r12,%r10
    4e93:	0f 85 89 01 00 00    	jne    5022 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x672>
    4e99:	48 85 f6             	test   %rsi,%rsi
    4e9c:	0f 85 c1 00 00 00    	jne    4f63 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x5b3>
    4ea2:	31 d2                	xor    %edx,%edx
    4ea4:	eb 8e                	jmp    4e34 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x484>
    4ea6:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
    4ead:	00 00 00 
    4eb0:	0f b6 43 10          	movzbl 0x10(%rbx),%eax
    4eb4:	88 07                	mov    %al,(%rdi)
    4eb6:	48 8b 53 08          	mov    0x8(%rbx),%rdx
    4eba:	49 8b 3c 24          	mov    (%r12),%rdi
    4ebe:	e9 80 fc ff ff       	jmp    4b43 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x193>
    4ec3:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)
    4ec8:	48 c1 e5 05          	shl    $0x5,%rbp
    4ecc:	4c 8b 54 24 30       	mov    0x30(%rsp),%r10
    4ed1:	49 8d 1c 2f          	lea    (%r15,%rbp,1),%rbx
    4ed5:	eb b6                	jmp    4e8d <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x4dd>
    4ed7:	66 0f 1f 84 00 00 00 	nopw   0x0(%rax,%rax,1)
    4ede:	00 00 
    4ee0:	c4 c1 7a 6f 59 10    	vmovdqu 0x10(%r9),%xmm3
    4ee6:	4d 89 e2             	mov    %r12,%r10
    4ee9:	c5 f9 7f 5c 24 40    	vmovdqa %xmm3,0x40(%rsp)
    4eef:	e9 bd fc ff ff       	jmp    4bb1 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x201>
    4ef4:	0f 1f 40 00          	nopl   0x0(%rax)
    4ef8:	48 8d 6c 2d 01       	lea    0x1(%rbp,%rbp,1),%rbp
    4efd:	49 89 ec             	mov    %rbp,%r12
    4f00:	49 c1 e4 05          	shl    $0x5,%r12
    4f04:	4d 01 fc             	add    %r15,%r12
    4f07:	49 8b 04 24          	mov    (%r12),%rax
    4f0b:	4d 8d 74 24 10       	lea    0x10(%r12),%r14
    4f10:	48 8b 3b             	mov    (%rbx),%rdi
    4f13:	49 8b 54 24 08       	mov    0x8(%r12),%rdx
    4f18:	4c 39 f0             	cmp    %r14,%rax
    4f1b:	0f 84 a7 00 00 00    	je     4fc8 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x618>
    4f21:	48 39 cf             	cmp    %rcx,%rdi
    4f24:	0f 84 85 00 00 00    	je     4faf <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x5ff>
    4f2a:	48 89 03             	mov    %rax,(%rbx)
    4f2d:	48 89 53 08          	mov    %rdx,0x8(%rbx)
    4f31:	48 8b 4b 10          	mov    0x10(%rbx),%rcx
    4f35:	49 8b 44 24 10       	mov    0x10(%r12),%rax
    4f3a:	48 89 43 10          	mov    %rax,0x10(%rbx)
    4f3e:	48 85 ff             	test   %rdi,%rdi
    4f41:	74 7c                	je     4fbf <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x60f>
    4f43:	49 89 3c 24          	mov    %rdi,(%r12)
    4f47:	49 89 4c 24 10       	mov    %rcx,0x10(%r12)
    4f4c:	49 c7 44 24 08 00 00 	movq   $0x0,0x8(%r12)
    4f53:	00 00 
    4f55:	4c 89 f1             	mov    %r14,%rcx
    4f58:	c6 07 00             	movb   $0x0,(%rdi)
    4f5b:	4c 89 e3             	mov    %r12,%rbx
    4f5e:	e9 26 fc ff ff       	jmp    4b89 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x1d9>
    4f63:	48 89 f2             	mov    %rsi,%rdx
    4f66:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
    4f6d:	00 00 00 
    4f70:	48 83 fa 01          	cmp    $0x1,%rdx
    4f74:	74 25                	je     4f9b <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x5eb>
    4f76:	4c 89 e6             	mov    %r12,%rsi
    4f79:	e8 c2 e1 ff ff       	call   3140 <memcpy@plt>
    4f7e:	48 8b 54 24 38       	mov    0x38(%rsp),%rdx
    4f83:	48 8b 3b             	mov    (%rbx),%rdi
    4f86:	e9 a9 fe ff ff       	jmp    4e34 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x484>
    4f8b:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)
    4f90:	4c 89 f1             	mov    %r14,%rcx
    4f93:	48 89 f5             	mov    %rsi,%rbp
    4f96:	e9 c8 fb ff ff       	jmp    4b63 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x1b3>
    4f9b:	0f b6 44 24 40       	movzbl 0x40(%rsp),%eax
    4fa0:	88 07                	mov    %al,(%rdi)
    4fa2:	48 8b 54 24 38       	mov    0x38(%rsp),%rdx
    4fa7:	48 8b 3b             	mov    (%rbx),%rdi
    4faa:	e9 85 fe ff ff       	jmp    4e34 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x484>
    4faf:	48 89 03             	mov    %rax,(%rbx)
    4fb2:	48 89 53 08          	mov    %rdx,0x8(%rbx)
    4fb6:	49 8b 44 24 10       	mov    0x10(%r12),%rax
    4fbb:	48 89 43 10          	mov    %rax,0x10(%rbx)
    4fbf:	4d 89 34 24          	mov    %r14,(%r12)
    4fc3:	4c 89 f7             	mov    %r14,%rdi
    4fc6:	eb 84                	jmp    4f4c <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x59c>
    4fc8:	48 85 d2             	test   %rdx,%rdx
    4fcb:	74 1e                	je     4feb <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x63b>
    4fcd:	48 83 fa 01          	cmp    $0x1,%rdx
    4fd1:	74 29                	je     4ffc <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x64c>
    4fd3:	4c 89 f6             	mov    %r14,%rsi
    4fd6:	4c 89 0c 24          	mov    %r9,(%rsp)
    4fda:	e8 61 e1 ff ff       	call   3140 <memcpy@plt>
    4fdf:	49 8b 54 24 08       	mov    0x8(%r12),%rdx
    4fe4:	48 8b 3b             	mov    (%rbx),%rdi
    4fe7:	4c 8b 0c 24          	mov    (%rsp),%r9
    4feb:	48 89 53 08          	mov    %rdx,0x8(%rbx)
    4fef:	c6 04 17 00          	movb   $0x0,(%rdi,%rdx,1)
    4ff3:	49 8b 3c 24          	mov    (%r12),%rdi
    4ff7:	e9 50 ff ff ff       	jmp    4f4c <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x59c>
    4ffc:	41 0f b6 44 24 10    	movzbl 0x10(%r12),%eax
    5002:	88 07                	mov    %al,(%rdi)
    5004:	49 8b 54 24 08       	mov    0x8(%r12),%rdx
    5009:	48 8b 3b             	mov    (%rbx),%rdi
    500c:	eb dd                	jmp    4feb <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x63b>
    500e:	e8 3d e1 ff ff       	call   3150 <__stack_chk_fail@plt>
    5013:	48 89 eb             	mov    %rbp,%rbx
    5016:	48 c1 e3 05          	shl    $0x5,%rbx
    501a:	4c 01 fb             	add    %r15,%rbx
    501d:	e9 b8 fc ff ff       	jmp    4cda <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x32a>
    5022:	48 89 f2             	mov    %rsi,%rdx
    5025:	e9 bc fc ff ff       	jmp    4ce6 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x336>
    502a:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)

0000000000005030 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0>:
    5030:	41 57                	push   %r15
    5032:	49 89 c9             	mov    %rcx,%r9
    5035:	49 89 ff             	mov    %rdi,%r15
    5038:	41 56                	push   %r14
    503a:	41 55                	push   %r13
    503c:	49 89 f5             	mov    %rsi,%r13
    503f:	41 54                	push   %r12
    5041:	55                   	push   %rbp
    5042:	53                   	push   %rbx
    5043:	48 89 f3             	mov    %rsi,%rbx
    5046:	48 c1 e3 05          	shl    $0x5,%rbx
    504a:	48 83 ec 68          	sub    $0x68,%rsp
    504e:	48 89 54 24 28       	mov    %rdx,0x28(%rsp)
    5053:	48 01 fb             	add    %rdi,%rbx
    5056:	4c 8d 73 10          	lea    0x10(%rbx),%r14
    505a:	64 48 8b 04 25 28 00 	mov    %fs:0x28,%rax
    5061:	00 00 
    5063:	48 89 44 24 58       	mov    %rax,0x58(%rsp)
    5068:	31 c0                	xor    %eax,%eax
    506a:	48 8d 42 ff          	lea    -0x1(%rdx),%rax
    506e:	49 89 c0             	mov    %rax,%r8
    5071:	49 c1 e8 3f          	shr    $0x3f,%r8
    5075:	49 01 c0             	add    %rax,%r8
    5078:	49 d1 f8             	sar    %r8
    507b:	4c 39 c6             	cmp    %r8,%rsi
    507e:	0f 8d 8c 05 00 00    	jge    5610 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x5e0>
    5084:	49 89 f4             	mov    %rsi,%r12
    5087:	eb 78                	jmp    5101 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0xd1>
    5089:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
    5090:	48 0f 43 d0          	cmovae %rax,%rdx
    5094:	48 0f 43 d9          	cmovae %rcx,%rbx
    5098:	49 0f 43 eb          	cmovae %r11,%rbp
    509c:	49 c1 e4 05          	shl    $0x5,%r12
    50a0:	48 8b 03             	mov    (%rbx),%rax
    50a3:	4d 01 fc             	add    %r15,%r12
    50a6:	48 8d 4b 10          	lea    0x10(%rbx),%rcx
    50aa:	49 8b 3c 24          	mov    (%r12),%rdi
    50ae:	48 39 c8             	cmp    %rcx,%rax
    50b1:	0f 84 d1 00 00 00    	je     5188 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x158>
    50b7:	4c 39 f7             	cmp    %r14,%rdi
    50ba:	0f 84 28 03 00 00    	je     53e8 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x3b8>
    50c0:	49 89 04 24          	mov    %rax,(%r12)
    50c4:	49 89 54 24 08       	mov    %rdx,0x8(%r12)
    50c9:	49 8b 74 24 10       	mov    0x10(%r12),%rsi
    50ce:	48 8b 43 10          	mov    0x10(%rbx),%rax
    50d2:	49 89 44 24 10       	mov    %rax,0x10(%r12)
    50d7:	48 85 ff             	test   %rdi,%rdi
    50da:	0f 84 1a 03 00 00    	je     53fa <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x3ca>
    50e0:	48 89 3b             	mov    %rdi,(%rbx)
    50e3:	48 89 73 10          	mov    %rsi,0x10(%rbx)
    50e7:	48 c7 43 08 00 00 00 	movq   $0x0,0x8(%rbx)
    50ee:	00 
    50ef:	c6 07 00             	movb   $0x0,(%rdi)
    50f2:	4c 39 c5             	cmp    %r8,%rbp
    50f5:	0f 8d e8 00 00 00    	jge    51e3 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x1b3>
    50fb:	49 89 ce             	mov    %rcx,%r14
    50fe:	49 89 ec             	mov    %rbp,%r12
    5101:	49 8d 4c 24 01       	lea    0x1(%r12),%rcx
    5106:	4c 8d 1c 09          	lea    (%rcx,%rcx,1),%r11
    510a:	49 8d 6b ff          	lea    -0x1(%r11),%rbp
    510e:	48 89 eb             	mov    %rbp,%rbx
    5111:	48 c1 e3 05          	shl    $0x5,%rbx
    5115:	48 c1 e1 06          	shl    $0x6,%rcx
    5119:	4c 01 fb             	add    %r15,%rbx
    511c:	4c 01 f9             	add    %r15,%rcx
    511f:	48 8b 41 08          	mov    0x8(%rcx),%rax
    5123:	48 8b 53 08          	mov    0x8(%rbx),%rdx
    5127:	48 39 d0             	cmp    %rdx,%rax
    512a:	0f 85 60 ff ff ff    	jne    5090 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x60>
    5130:	48 85 d2             	test   %rdx,%rdx
    5133:	0f 84 97 03 00 00    	je     54d0 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x4a0>
    5139:	48 8b 33             	mov    (%rbx),%rsi
    513c:	48 8b 39             	mov    (%rcx),%rdi
    513f:	4c 89 44 24 20       	mov    %r8,0x20(%rsp)
    5144:	4c 89 4c 24 18       	mov    %r9,0x18(%rsp)
    5149:	4c 89 5c 24 10       	mov    %r11,0x10(%rsp)
    514e:	48 89 54 24 08       	mov    %rdx,0x8(%rsp)
    5153:	48 89 0c 24          	mov    %rcx,(%rsp)
    5157:	e8 14 e1 ff ff       	call   3270 <memcmp@plt>
    515c:	48 8b 0c 24          	mov    (%rsp),%rcx
    5160:	4c 8b 5c 24 10       	mov    0x10(%rsp),%r11
    5165:	85 c0                	test   %eax,%eax
    5167:	48 0f 49 d9          	cmovns %rcx,%rbx
    516b:	49 0f 49 eb          	cmovns %r11,%rbp
    516f:	48 8b 54 24 08       	mov    0x8(%rsp),%rdx
    5174:	4c 8b 4c 24 18       	mov    0x18(%rsp),%r9
    5179:	4c 8b 44 24 20       	mov    0x20(%rsp),%r8
    517e:	e9 19 ff ff ff       	jmp    509c <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x6c>
    5183:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)
    5188:	48 85 d2             	test   %rdx,%rdx
    518b:	74 36                	je     51c3 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x193>
    518d:	48 83 fa 01          	cmp    $0x1,%rdx
    5191:	0f 84 99 03 00 00    	je     5530 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x500>
    5197:	48 89 ce             	mov    %rcx,%rsi
    519a:	4c 89 44 24 10       	mov    %r8,0x10(%rsp)
    519f:	4c 89 4c 24 08       	mov    %r9,0x8(%rsp)
    51a4:	48 89 0c 24          	mov    %rcx,(%rsp)
    51a8:	e8 93 df ff ff       	call   3140 <memcpy@plt>
    51ad:	48 8b 53 08          	mov    0x8(%rbx),%rdx
    51b1:	49 8b 3c 24          	mov    (%r12),%rdi
    51b5:	4c 8b 44 24 10       	mov    0x10(%rsp),%r8
    51ba:	4c 8b 4c 24 08       	mov    0x8(%rsp),%r9
    51bf:	48 8b 0c 24          	mov    (%rsp),%rcx
    51c3:	49 89 54 24 08       	mov    %rdx,0x8(%r12)
    51c8:	c6 04 17 00          	movb   $0x0,(%rdi,%rdx,1)
    51cc:	48 c7 43 08 00 00 00 	movq   $0x0,0x8(%rbx)
    51d3:	00 
    51d4:	48 8b 3b             	mov    (%rbx),%rdi
    51d7:	c6 07 00             	movb   $0x0,(%rdi)
    51da:	4c 39 c5             	cmp    %r8,%rbp
    51dd:	0f 8c 18 ff ff ff    	jl     50fb <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0xcb>
    51e3:	48 8b 44 24 28       	mov    0x28(%rsp),%rax
    51e8:	a8 01                	test   $0x1,%al
    51ea:	75 1d                	jne    5209 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x1d9>
    51ec:	49 89 c6             	mov    %rax,%r14
    51ef:	49 83 ee 02          	sub    $0x2,%r14
    51f3:	4c 89 f0             	mov    %r14,%rax
    51f6:	48 c1 e8 3f          	shr    $0x3f,%rax
    51fa:	49 01 c6             	add    %rax,%r14
    51fd:	49 d1 fe             	sar    %r14
    5200:	49 39 ee             	cmp    %rbp,%r14
    5203:	0f 84 6f 03 00 00    	je     5578 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x548>
    5209:	4d 8b 11             	mov    (%r9),%r10
    520c:	4c 8d 64 24 40       	lea    0x40(%rsp),%r12
    5211:	49 8d 41 10          	lea    0x10(%r9),%rax
    5215:	4c 89 64 24 30       	mov    %r12,0x30(%rsp)
    521a:	49 39 c2             	cmp    %rax,%r10
    521d:	0f 84 3d 03 00 00    	je     5560 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x530>
    5223:	49 8b 51 10          	mov    0x10(%r9),%rdx
    5227:	4c 89 54 24 30       	mov    %r10,0x30(%rsp)
    522c:	48 89 54 24 40       	mov    %rdx,0x40(%rsp)
    5231:	49 89 01             	mov    %rax,(%r9)
    5234:	48 8d 45 ff          	lea    -0x1(%rbp),%rax
    5238:	49 89 c6             	mov    %rax,%r14
    523b:	49 8b 71 08          	mov    0x8(%r9),%rsi
    523f:	49 c1 ee 3f          	shr    $0x3f,%r14
    5243:	49 01 c6             	add    %rax,%r14
    5246:	48 89 74 24 38       	mov    %rsi,0x38(%rsp)
    524b:	49 c7 41 08 00 00 00 	movq   $0x0,0x8(%r9)
    5252:	00 
    5253:	41 c6 41 10 00       	movb   $0x0,0x10(%r9)
    5258:	49 d1 fe             	sar    %r14
    525b:	4c 39 ed             	cmp    %r13,%rbp
    525e:	0f 8f 8d 00 00 00    	jg     52f1 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x2c1>
    5264:	e9 a4 02 00 00       	jmp    550d <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x4dd>
    5269:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
    5270:	0f 86 d2 02 00 00    	jbe    5548 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x518>
    5276:	48 c1 e5 05          	shl    $0x5,%rbp
    527a:	4c 8b 03             	mov    (%rbx),%r8
    527d:	4c 01 fd             	add    %r15,%rbp
    5280:	48 8d 73 10          	lea    0x10(%rbx),%rsi
    5284:	48 8b 7d 00          	mov    0x0(%rbp),%rdi
    5288:	49 39 f0             	cmp    %rsi,%r8
    528b:	0f 84 ef 01 00 00    	je     5480 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x450>
    5291:	48 39 f9             	cmp    %rdi,%rcx
    5294:	0f 84 be 01 00 00    	je     5458 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x428>
    529a:	4c 89 45 00          	mov    %r8,0x0(%rbp)
    529e:	48 89 55 08          	mov    %rdx,0x8(%rbp)
    52a2:	48 8b 45 10          	mov    0x10(%rbp),%rax
    52a6:	48 8b 53 10          	mov    0x10(%rbx),%rdx
    52aa:	48 89 55 10          	mov    %rdx,0x10(%rbp)
    52ae:	48 85 ff             	test   %rdi,%rdi
    52b1:	0f 84 b1 01 00 00    	je     5468 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x438>
    52b7:	48 89 3b             	mov    %rdi,(%rbx)
    52ba:	48 89 43 10          	mov    %rax,0x10(%rbx)
    52be:	48 89 f1             	mov    %rsi,%rcx
    52c1:	49 8d 56 ff          	lea    -0x1(%r14),%rdx
    52c5:	48 c7 43 08 00 00 00 	movq   $0x0,0x8(%rbx)
    52cc:	00 
    52cd:	48 89 d0             	mov    %rdx,%rax
    52d0:	c6 07 00             	movb   $0x0,(%rdi)
    52d3:	48 c1 e8 3f          	shr    $0x3f,%rax
    52d7:	48 01 d0             	add    %rdx,%rax
    52da:	48 8b 74 24 38       	mov    0x38(%rsp),%rsi
    52df:	48 d1 f8             	sar    %rax
    52e2:	4d 39 f5             	cmp    %r14,%r13
    52e5:	0f 8d 1d 02 00 00    	jge    5508 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x4d8>
    52eb:	4c 89 f5             	mov    %r14,%rbp
    52ee:	49 89 c6             	mov    %rax,%r14
    52f1:	4c 89 f3             	mov    %r14,%rbx
    52f4:	48 c1 e3 05          	shl    $0x5,%rbx
    52f8:	4c 01 fb             	add    %r15,%rbx
    52fb:	48 8b 53 08          	mov    0x8(%rbx),%rdx
    52ff:	48 39 d6             	cmp    %rdx,%rsi
    5302:	0f 85 68 ff ff ff    	jne    5270 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x240>
    5308:	4c 8b 54 24 30       	mov    0x30(%rsp),%r10
    530d:	48 85 d2             	test   %rdx,%rdx
    5310:	0f 84 8a 01 00 00    	je     54a0 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x470>
    5316:	4c 8b 03             	mov    (%rbx),%r8
    5319:	4c 89 d6             	mov    %r10,%rsi
    531c:	4c 89 c7             	mov    %r8,%rdi
    531f:	48 89 4c 24 18       	mov    %rcx,0x18(%rsp)
    5324:	48 89 54 24 10       	mov    %rdx,0x10(%rsp)
    5329:	4c 89 54 24 08       	mov    %r10,0x8(%rsp)
    532e:	4c 89 04 24          	mov    %r8,(%rsp)
    5332:	e8 39 df ff ff       	call   3270 <memcmp@plt>
    5337:	85 c0                	test   %eax,%eax
    5339:	4c 8b 04 24          	mov    (%rsp),%r8
    533d:	4c 8b 54 24 08       	mov    0x8(%rsp),%r10
    5342:	48 8b 54 24 10       	mov    0x10(%rsp),%rdx
    5347:	48 8b 4c 24 18       	mov    0x18(%rsp),%rcx
    534c:	0f 85 b6 00 00 00    	jne    5408 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x3d8>
    5352:	48 c1 e5 05          	shl    $0x5,%rbp
    5356:	49 8d 1c 2f          	lea    (%r15,%rbp,1),%rbx
    535a:	48 8b 3b             	mov    (%rbx),%rdi
    535d:	4d 39 e2             	cmp    %r12,%r10
    5360:	0f 84 8a 02 00 00    	je     55f0 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x5c0>
    5366:	48 8b 44 24 40       	mov    0x40(%rsp),%rax
    536b:	48 39 f9             	cmp    %rdi,%rcx
    536e:	0f 84 6c 01 00 00    	je     54e0 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x4b0>
    5374:	c4 e1 f9 6e ca       	vmovq  %rdx,%xmm1
    5379:	c4 e3 f1 22 c0 01    	vpinsrq $0x1,%rax,%xmm1,%xmm0
    537f:	48 8b 4b 10          	mov    0x10(%rbx),%rcx
    5383:	4c 89 13             	mov    %r10,(%rbx)
    5386:	c5 fa 7f 43 08       	vmovdqu %xmm0,0x8(%rbx)
    538b:	48 85 ff             	test   %rdi,%rdi
    538e:	0f 84 5f 01 00 00    	je     54f3 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x4c3>
    5394:	48 89 7c 24 30       	mov    %rdi,0x30(%rsp)
    5399:	48 89 4c 24 40       	mov    %rcx,0x40(%rsp)
    539e:	48 c7 44 24 38 00 00 	movq   $0x0,0x38(%rsp)
    53a5:	00 00 
    53a7:	c6 07 00             	movb   $0x0,(%rdi)
    53aa:	48 8b 7c 24 30       	mov    0x30(%rsp),%rdi
    53af:	4c 39 e7             	cmp    %r12,%rdi
    53b2:	74 0e                	je     53c2 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x392>
    53b4:	48 8b 44 24 40       	mov    0x40(%rsp),%rax
    53b9:	48 8d 70 01          	lea    0x1(%rax),%rsi
    53bd:	e8 5e df ff ff       	call   3320 <_ZdlPvm@plt>
    53c2:	48 8b 44 24 58       	mov    0x58(%rsp),%rax
    53c7:	64 48 2b 04 25 28 00 	sub    %fs:0x28,%rax
    53ce:	00 00 
    53d0:	0f 85 b8 02 00 00    	jne    568e <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x65e>
    53d6:	48 83 c4 68          	add    $0x68,%rsp
    53da:	5b                   	pop    %rbx
    53db:	5d                   	pop    %rbp
    53dc:	41 5c                	pop    %r12
    53de:	41 5d                	pop    %r13
    53e0:	41 5e                	pop    %r14
    53e2:	41 5f                	pop    %r15
    53e4:	c3                   	ret    
    53e5:	0f 1f 00             	nopl   (%rax)
    53e8:	49 89 04 24          	mov    %rax,(%r12)
    53ec:	49 89 54 24 08       	mov    %rdx,0x8(%r12)
    53f1:	48 8b 43 10          	mov    0x10(%rbx),%rax
    53f5:	49 89 44 24 10       	mov    %rax,0x10(%r12)
    53fa:	48 89 0b             	mov    %rcx,(%rbx)
    53fd:	48 89 cf             	mov    %rcx,%rdi
    5400:	e9 e2 fc ff ff       	jmp    50e7 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0xb7>
    5405:	0f 1f 00             	nopl   (%rax)
    5408:	0f 89 85 02 00 00    	jns    5693 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x663>
    540e:	48 c1 e5 05          	shl    $0x5,%rbp
    5412:	4c 01 fd             	add    %r15,%rbp
    5415:	48 8d 73 10          	lea    0x10(%rbx),%rsi
    5419:	48 8b 7d 00          	mov    0x0(%rbp),%rdi
    541d:	49 39 f0             	cmp    %rsi,%r8
    5420:	0f 85 6b fe ff ff    	jne    5291 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x261>
    5426:	48 83 fa 01          	cmp    $0x1,%rdx
    542a:	74 5f                	je     548b <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x45b>
    542c:	48 89 34 24          	mov    %rsi,(%rsp)
    5430:	e8 0b dd ff ff       	call   3140 <memcpy@plt>
    5435:	48 8b 53 08          	mov    0x8(%rbx),%rdx
    5439:	48 8b 7d 00          	mov    0x0(%rbp),%rdi
    543d:	48 8b 34 24          	mov    (%rsp),%rsi
    5441:	48 89 55 08          	mov    %rdx,0x8(%rbp)
    5445:	c6 04 17 00          	movb   $0x0,(%rdi,%rdx,1)
    5449:	48 89 f1             	mov    %rsi,%rcx
    544c:	48 8b 3b             	mov    (%rbx),%rdi
    544f:	e9 6d fe ff ff       	jmp    52c1 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x291>
    5454:	0f 1f 40 00          	nopl   0x0(%rax)
    5458:	4c 89 45 00          	mov    %r8,0x0(%rbp)
    545c:	48 89 55 08          	mov    %rdx,0x8(%rbp)
    5460:	48 8b 43 10          	mov    0x10(%rbx),%rax
    5464:	48 89 45 10          	mov    %rax,0x10(%rbp)
    5468:	48 89 33             	mov    %rsi,(%rbx)
    546b:	48 89 f1             	mov    %rsi,%rcx
    546e:	48 89 f7             	mov    %rsi,%rdi
    5471:	e9 4b fe ff ff       	jmp    52c1 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x291>
    5476:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
    547d:	00 00 00 
    5480:	48 85 d2             	test   %rdx,%rdx
    5483:	74 bc                	je     5441 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x411>
    5485:	48 83 fa 01          	cmp    $0x1,%rdx
    5489:	75 a1                	jne    542c <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x3fc>
    548b:	0f b6 43 10          	movzbl 0x10(%rbx),%eax
    548f:	88 07                	mov    %al,(%rdi)
    5491:	48 8b 53 08          	mov    0x8(%rbx),%rdx
    5495:	48 8b 7d 00          	mov    0x0(%rbp),%rdi
    5499:	eb a6                	jmp    5441 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x411>
    549b:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)
    54a0:	48 c1 e5 05          	shl    $0x5,%rbp
    54a4:	49 8d 1c 2f          	lea    (%r15,%rbp,1),%rbx
    54a8:	48 8b 3b             	mov    (%rbx),%rdi
    54ab:	4d 39 e2             	cmp    %r12,%r10
    54ae:	0f 85 b2 fe ff ff    	jne    5366 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x336>
    54b4:	48 89 53 08          	mov    %rdx,0x8(%rbx)
    54b8:	c6 04 17 00          	movb   $0x0,(%rdi,%rdx,1)
    54bc:	48 8b 7c 24 30       	mov    0x30(%rsp),%rdi
    54c1:	e9 d8 fe ff ff       	jmp    539e <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x36e>
    54c6:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
    54cd:	00 00 00 
    54d0:	48 89 cb             	mov    %rcx,%rbx
    54d3:	4c 89 dd             	mov    %r11,%rbp
    54d6:	e9 c1 fb ff ff       	jmp    509c <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x6c>
    54db:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)
    54e0:	c4 e1 f9 6e d2       	vmovq  %rdx,%xmm2
    54e5:	4c 89 13             	mov    %r10,(%rbx)
    54e8:	c4 e3 e9 22 c0 01    	vpinsrq $0x1,%rax,%xmm2,%xmm0
    54ee:	c5 fa 7f 43 08       	vmovdqu %xmm0,0x8(%rbx)
    54f3:	4c 89 64 24 30       	mov    %r12,0x30(%rsp)
    54f8:	4c 8d 64 24 40       	lea    0x40(%rsp),%r12
    54fd:	4c 89 e7             	mov    %r12,%rdi
    5500:	e9 99 fe ff ff       	jmp    539e <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x36e>
    5505:	0f 1f 00             	nopl   (%rax)
    5508:	4c 8b 54 24 30       	mov    0x30(%rsp),%r10
    550d:	48 8b 3b             	mov    (%rbx),%rdi
    5510:	4d 39 e2             	cmp    %r12,%r10
    5513:	0f 85 89 01 00 00    	jne    56a2 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x672>
    5519:	48 85 f6             	test   %rsi,%rsi
    551c:	0f 85 c1 00 00 00    	jne    55e3 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x5b3>
    5522:	31 d2                	xor    %edx,%edx
    5524:	eb 8e                	jmp    54b4 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x484>
    5526:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
    552d:	00 00 00 
    5530:	0f b6 43 10          	movzbl 0x10(%rbx),%eax
    5534:	88 07                	mov    %al,(%rdi)
    5536:	48 8b 53 08          	mov    0x8(%rbx),%rdx
    553a:	49 8b 3c 24          	mov    (%r12),%rdi
    553e:	e9 80 fc ff ff       	jmp    51c3 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x193>
    5543:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)
    5548:	48 c1 e5 05          	shl    $0x5,%rbp
    554c:	4c 8b 54 24 30       	mov    0x30(%rsp),%r10
    5551:	49 8d 1c 2f          	lea    (%r15,%rbp,1),%rbx
    5555:	eb b6                	jmp    550d <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x4dd>
    5557:	66 0f 1f 84 00 00 00 	nopw   0x0(%rax,%rax,1)
    555e:	00 00 
    5560:	c4 c1 7a 6f 59 10    	vmovdqu 0x10(%r9),%xmm3
    5566:	4d 89 e2             	mov    %r12,%r10
    5569:	c5 f9 7f 5c 24 40    	vmovdqa %xmm3,0x40(%rsp)
    556f:	e9 bd fc ff ff       	jmp    5231 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x201>
    5574:	0f 1f 40 00          	nopl   0x0(%rax)
    5578:	48 8d 6c 2d 01       	lea    0x1(%rbp,%rbp,1),%rbp
    557d:	49 89 ec             	mov    %rbp,%r12
    5580:	49 c1 e4 05          	shl    $0x5,%r12
    5584:	4d 01 fc             	add    %r15,%r12
    5587:	49 8b 04 24          	mov    (%r12),%rax
    558b:	4d 8d 74 24 10       	lea    0x10(%r12),%r14
    5590:	48 8b 3b             	mov    (%rbx),%rdi
    5593:	49 8b 54 24 08       	mov    0x8(%r12),%rdx
    5598:	4c 39 f0             	cmp    %r14,%rax
    559b:	0f 84 a7 00 00 00    	je     5648 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x618>
    55a1:	48 39 cf             	cmp    %rcx,%rdi
    55a4:	0f 84 85 00 00 00    	je     562f <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x5ff>
    55aa:	48 89 03             	mov    %rax,(%rbx)
    55ad:	48 89 53 08          	mov    %rdx,0x8(%rbx)
    55b1:	48 8b 4b 10          	mov    0x10(%rbx),%rcx
    55b5:	49 8b 44 24 10       	mov    0x10(%r12),%rax
    55ba:	48 89 43 10          	mov    %rax,0x10(%rbx)
    55be:	48 85 ff             	test   %rdi,%rdi
    55c1:	74 7c                	je     563f <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x60f>
    55c3:	49 89 3c 24          	mov    %rdi,(%r12)
    55c7:	49 89 4c 24 10       	mov    %rcx,0x10(%r12)
    55cc:	49 c7 44 24 08 00 00 	movq   $0x0,0x8(%r12)
    55d3:	00 00 
    55d5:	4c 89 f1             	mov    %r14,%rcx
    55d8:	c6 07 00             	movb   $0x0,(%rdi)
    55db:	4c 89 e3             	mov    %r12,%rbx
    55de:	e9 26 fc ff ff       	jmp    5209 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x1d9>
    55e3:	48 89 f2             	mov    %rsi,%rdx
    55e6:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
    55ed:	00 00 00 
    55f0:	48 83 fa 01          	cmp    $0x1,%rdx
    55f4:	74 25                	je     561b <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x5eb>
    55f6:	4c 89 e6             	mov    %r12,%rsi
    55f9:	e8 42 db ff ff       	call   3140 <memcpy@plt>
    55fe:	48 8b 54 24 38       	mov    0x38(%rsp),%rdx
    5603:	48 8b 3b             	mov    (%rbx),%rdi
    5606:	e9 a9 fe ff ff       	jmp    54b4 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x484>
    560b:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)
    5610:	4c 89 f1             	mov    %r14,%rcx
    5613:	48 89 f5             	mov    %rsi,%rbp
    5616:	e9 c8 fb ff ff       	jmp    51e3 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x1b3>
    561b:	0f b6 44 24 40       	movzbl 0x40(%rsp),%eax
    5620:	88 07                	mov    %al,(%rdi)
    5622:	48 8b 54 24 38       	mov    0x38(%rsp),%rdx
    5627:	48 8b 3b             	mov    (%rbx),%rdi
    562a:	e9 85 fe ff ff       	jmp    54b4 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x484>
    562f:	48 89 03             	mov    %rax,(%rbx)
    5632:	48 89 53 08          	mov    %rdx,0x8(%rbx)
    5636:	49 8b 44 24 10       	mov    0x10(%r12),%rax
    563b:	48 89 43 10          	mov    %rax,0x10(%rbx)
    563f:	4d 89 34 24          	mov    %r14,(%r12)
    5643:	4c 89 f7             	mov    %r14,%rdi
    5646:	eb 84                	jmp    55cc <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x59c>
    5648:	48 85 d2             	test   %rdx,%rdx
    564b:	74 1e                	je     566b <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x63b>
    564d:	48 83 fa 01          	cmp    $0x1,%rdx
    5651:	74 29                	je     567c <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x64c>
    5653:	4c 89 f6             	mov    %r14,%rsi
    5656:	4c 89 0c 24          	mov    %r9,(%rsp)
    565a:	e8 e1 da ff ff       	call   3140 <memcpy@plt>
    565f:	49 8b 54 24 08       	mov    0x8(%r12),%rdx
    5664:	48 8b 3b             	mov    (%rbx),%rdi
    5667:	4c 8b 0c 24          	mov    (%rsp),%r9
    566b:	48 89 53 08          	mov    %rdx,0x8(%rbx)
    566f:	c6 04 17 00          	movb   $0x0,(%rdi,%rdx,1)
    5673:	49 8b 3c 24          	mov    (%r12),%rdi
    5677:	e9 50 ff ff ff       	jmp    55cc <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x59c>
    567c:	41 0f b6 44 24 10    	movzbl 0x10(%r12),%eax
    5682:	88 07                	mov    %al,(%rdi)
    5684:	49 8b 54 24 08       	mov    0x8(%r12),%rdx
    5689:	48 8b 3b             	mov    (%rbx),%rdi
    568c:	eb dd                	jmp    566b <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x63b>
    568e:	e8 bd da ff ff       	call   3150 <__stack_chk_fail@plt>
    5693:	48 89 eb             	mov    %rbp,%rbx
    5696:	48 c1 e3 05          	shl    $0x5,%rbx
    569a:	4c 01 fb             	add    %r15,%rbx
    569d:	e9 b8 fc ff ff       	jmp    535a <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x32a>
    56a2:	48 89 f2             	mov    %rsi,%rdx
    56a5:	e9 bc fc ff ff       	jmp    5366 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0+0x336>
    56aa:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)

00000000000056b0 <_ZZNSt8__detail9__variant17_Move_assign_baseILb0EJSt8functionIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEES2_IFvSB_EEEEaSEOSG_ENUlOT_T0_E_clIRSD_St17integral_constantImLm0EEEEDaSJ_SK_.isra.0>:
    56b0:	55                   	push   %rbp
    56b1:	48 89 f5             	mov    %rsi,%rbp
    56b4:	53                   	push   %rbx
    56b5:	48 89 fb             	mov    %rdi,%rbx
    56b8:	48 83 ec 48          	sub    $0x48,%rsp
    56bc:	64 48 8b 04 25 28 00 	mov    %fs:0x28,%rax
    56c3:	00 00 
    56c5:	48 89 44 24 38       	mov    %rax,0x38(%rsp)
    56ca:	31 c0                	xor    %eax,%eax
    56cc:	80 7f 20 00          	cmpb   $0x0,0x20(%rdi)
    56d0:	75 7e                	jne    5750 <_ZZNSt8__detail9__variant17_Move_assign_baseILb0EJSt8functionIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEES2_IFvSB_EEEEaSEOSG_ENUlOT_T0_E_clIRSD_St17integral_constantImLm0EEEEDaSJ_SK_.isra.0+0xa0>
    56d2:	48 8b 44 24 28       	mov    0x28(%rsp),%rax
    56d7:	c5 fa 7e 46 10       	vmovq  0x10(%rsi),%xmm0
    56dc:	48 8b 56 18          	mov    0x18(%rsi),%rdx
    56e0:	48 c7 46 10 00 00 00 	movq   $0x0,0x10(%rsi)
    56e7:	00 
    56e8:	48 89 46 18          	mov    %rax,0x18(%rsi)
    56ec:	c5 f9 6f 54 24 10    	vmovdqa 0x10(%rsp),%xmm2
    56f2:	c5 fa 6f 0e          	vmovdqu (%rsi),%xmm1
    56f6:	c5 fa 7f 16          	vmovdqu %xmm2,(%rsi)
    56fa:	c5 fa 6f 1f          	vmovdqu (%rdi),%xmm3
    56fe:	48 8b 47 10          	mov    0x10(%rdi),%rax
    5702:	48 8b 4f 18          	mov    0x18(%rdi),%rcx
    5706:	c4 e3 f9 22 c2 01    	vpinsrq $0x1,%rdx,%xmm0,%xmm0
    570c:	48 89 44 24 20       	mov    %rax,0x20(%rsp)
    5711:	48 89 4c 24 28       	mov    %rcx,0x28(%rsp)
    5716:	c5 f9 7f 5c 24 10    	vmovdqa %xmm3,0x10(%rsp)
    571c:	c5 fa 7f 0f          	vmovdqu %xmm1,(%rdi)
    5720:	c5 fa 7f 47 10       	vmovdqu %xmm0,0x10(%rdi)
    5725:	48 85 c0             	test   %rax,%rax
    5728:	74 0f                	je     5739 <_ZZNSt8__detail9__variant17_Move_assign_baseILb0EJSt8functionIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEES2_IFvSB_EEEEaSEOSG_ENUlOT_T0_E_clIRSD_St17integral_constantImLm0EEEEDaSJ_SK_.isra.0+0x89>
    572a:	48 8d 7c 24 10       	lea    0x10(%rsp),%rdi
    572f:	ba 03 00 00 00       	mov    $0x3,%edx
    5734:	48 89 fe             	mov    %rdi,%rsi
    5737:	ff d0                	call   *%rax
    5739:	48 8b 44 24 38       	mov    0x38(%rsp),%rax
    573e:	64 48 2b 04 25 28 00 	sub    %fs:0x28,%rax
    5745:	00 00 
    5747:	75 5d                	jne    57a6 <_ZZNSt8__detail9__variant17_Move_assign_baseILb0EJSt8functionIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEES2_IFvSB_EEEEaSEOSG_ENUlOT_T0_E_clIRSD_St17integral_constantImLm0EEEEDaSJ_SK_.isra.0+0xf6>
    5749:	48 83 c4 48          	add    $0x48,%rsp
    574d:	5b                   	pop    %rbx
    574e:	5d                   	pop    %rbp
    574f:	c3                   	ret    
    5750:	48 8d 7c 24 0f       	lea    0xf(%rsp),%rdi
    5755:	48 89 de             	mov    %rbx,%rsi
    5758:	e8 73 0a 00 00       	call   61d0 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFvOZNS0_16_Variant_storageILb0EJSt8functionIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEES4_IFvSD_EEEE8_M_resetEvEUlOT_E_RSt7variantIJSF_SH_EEEJEEESt16integer_sequenceImJLm1EEEE14__visit_invokeESM_SP_>
    575d:	c6 43 20 00          	movb   $0x0,0x20(%rbx)
    5761:	48 c7 43 10 00 00 00 	movq   $0x0,0x10(%rbx)
    5768:	00 
    5769:	48 8b 53 18          	mov    0x18(%rbx),%rdx
    576d:	c5 fa 6f 23          	vmovdqu (%rbx),%xmm4
    5771:	48 8b 45 10          	mov    0x10(%rbp),%rax
    5775:	48 c7 45 10 00 00 00 	movq   $0x0,0x10(%rbp)
    577c:	00 
    577d:	48 89 43 10          	mov    %rax,0x10(%rbx)
    5781:	48 8b 45 18          	mov    0x18(%rbp),%rax
    5785:	48 89 55 18          	mov    %rdx,0x18(%rbp)
    5789:	c5 fa 6f 45 00       	vmovdqu 0x0(%rbp),%xmm0
    578e:	c5 fa 7f 65 00       	vmovdqu %xmm4,0x0(%rbp)
    5793:	80 7b 20 00          	cmpb   $0x0,0x20(%rbx)
    5797:	48 89 43 18          	mov    %rax,0x18(%rbx)
    579b:	c5 fa 7f 03          	vmovdqu %xmm0,(%rbx)
    579f:	74 98                	je     5739 <_ZZNSt8__detail9__variant17_Move_assign_baseILb0EJSt8functionIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEES2_IFvSB_EEEEaSEOSG_ENUlOT_T0_E_clIRSD_St17integral_constantImLm0EEEEDaSJ_SK_.isra.0+0x89>
    57a1:	e9 9a dc ff ff       	jmp    3440 <_ZZNSt8__detail9__variant17_Move_assign_baseILb0EJSt8functionIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEES2_IFvSB_EEEEaSEOSG_ENUlOT_T0_E_clIRSD_St17integral_constantImLm0EEEEDaSJ_SK_.isra.0.cold>
    57a6:	e8 a5 d9 ff ff       	call   3150 <__stack_chk_fail@plt>
    57ab:	90                   	nop
    57ac:	0f 1f 40 00          	nopl   0x0(%rax)

00000000000057b0 <_ZNSt7variantIJSt8functionIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEES0_IFvS9_EEEE7emplaceILm0EJN8argparse7details12parse_numberIiLi0EEEEEENSt9enable_ifIX18is_constructible_vINSt19variant_alternativeIXT_ESE_E4typeEDpT0_EERSN_E4typeEDpOSO_.isra.0>:
    57b0:	55                   	push   %rbp
    57b1:	48 8d 0d f8 0b 00 00 	lea    0xbf8(%rip),%rcx        # 63b0 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIiLi0EEEE10_M_managerERSt9_Any_dataRKSF_St18_Manager_operation>
    57b8:	c4 e1 f9 6e c1       	vmovq  %rcx,%xmm0
    57bd:	53                   	push   %rbx
    57be:	48 89 fb             	mov    %rdi,%rbx
    57c1:	48 83 ec 48          	sub    $0x48,%rsp
    57c5:	64 48 8b 04 25 28 00 	mov    %fs:0x28,%rax
    57cc:	00 00 
    57ce:	48 89 44 24 38       	mov    %rax,0x38(%rsp)
    57d3:	48 8d 05 36 97 00 00 	lea    0x9736(%rip),%rax        # ef10 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIiLi0EEEE9_M_invokeERKSt9_Any_dataS8_>
    57da:	48 8d 6c 24 10       	lea    0x10(%rsp),%rbp
    57df:	48 89 ee             	mov    %rbp,%rsi
    57e2:	c6 44 24 30 00       	movb   $0x0,0x30(%rsp)
    57e7:	c4 e3 f9 22 c0 01    	vpinsrq $0x1,%rax,%xmm0,%xmm0
    57ed:	c5 f9 7f 44 24 20    	vmovdqa %xmm0,0x20(%rsp)
    57f3:	e8 b8 fe ff ff       	call   56b0 <_ZZNSt8__detail9__variant17_Move_assign_baseILb0EJSt8functionIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEES2_IFvSB_EEEEaSEOSG_ENUlOT_T0_E_clIRSD_St17integral_constantImLm0EEEEDaSJ_SK_.isra.0>
    57f8:	0f b6 54 24 30       	movzbl 0x30(%rsp),%edx
    57fd:	48 8d 7c 24 0f       	lea    0xf(%rsp),%rdi
    5802:	48 89 ee             	mov    %rbp,%rsi
    5805:	48 8d 05 84 e0 00 00 	lea    0xe084(%rip),%rax        # 13890 <_ZNSt8__detail9__variant12__gen_vtableIvOZNS0_16_Variant_storageILb0EJSt8functionIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEES3_IFvSC_EEEE8_M_resetEvEUlOT_E_JRSt7variantIJSE_SG_EEEE9_S_vtableE>
    580c:	ff 14 d0             	call   *(%rax,%rdx,8)
    580f:	80 7b 20 00          	cmpb   $0x0,0x20(%rbx)
    5813:	0f 85 2f dc ff ff    	jne    3448 <_ZNSt7variantIJSt8functionIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEES0_IFvS9_EEEE7emplaceILm0EJN8argparse7details12parse_numberIiLi0EEEEEENSt9enable_ifIX18is_constructible_vINSt19variant_alternativeIXT_ESE_E4typeEDpT0_EERSN_E4typeEDpOSO_.isra.0.cold>
    5819:	48 8b 44 24 38       	mov    0x38(%rsp),%rax
    581e:	64 48 2b 04 25 28 00 	sub    %fs:0x28,%rax
    5825:	00 00 
    5827:	75 07                	jne    5830 <_ZNSt7variantIJSt8functionIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEES0_IFvS9_EEEE7emplaceILm0EJN8argparse7details12parse_numberIiLi0EEEEEENSt9enable_ifIX18is_constructible_vINSt19variant_alternativeIXT_ESE_E4typeEDpT0_EERSN_E4typeEDpOSO_.isra.0+0x80>
    5829:	48 83 c4 48          	add    $0x48,%rsp
    582d:	5b                   	pop    %rbx
    582e:	5d                   	pop    %rbp
    582f:	c3                   	ret    
    5830:	e8 1b d9 ff ff       	call   3150 <__stack_chk_fail@plt>
    5835:	66 66 2e 0f 1f 84 00 	data16 cs nopw 0x0(%rax,%rax,1)
    583c:	00 00 00 00 

0000000000005840 <_ZN8argparselsERSoRKNS_8ArgumentE.isra.0>:
    5840:	55                   	push   %rbp
    5841:	48 89 e5             	mov    %rsp,%rbp
    5844:	41 57                	push   %r15
    5846:	49 89 f7             	mov    %rsi,%r15
    5849:	41 56                	push   %r14
    584b:	49 89 fe             	mov    %rdi,%r14
    584e:	41 55                	push   %r13
    5850:	41 54                	push   %r12
    5852:	53                   	push   %rbx
    5853:	48 83 e4 e0          	and    $0xffffffffffffffe0,%rsp
    5857:	48 81 ec e0 01 00 00 	sub    $0x1e0,%rsp
    585e:	64 48 8b 04 25 28 00 	mov    %fs:0x28,%rax
    5865:	00 00 
    5867:	48 89 84 24 d8 01 00 	mov    %rax,0x1d8(%rsp)
    586e:	00 
    586f:	31 c0                	xor    %eax,%eax
    5871:	48 8d 84 24 c0 00 00 	lea    0xc0(%rsp),%rax
    5878:	00 
    5879:	48 8d 5c 24 40       	lea    0x40(%rsp),%rbx
    587e:	48 89 c7             	mov    %rax,%rdi
    5881:	48 89 1c 24          	mov    %rbx,(%rsp)
    5885:	48 89 44 24 18       	mov    %rax,0x18(%rsp)
    588a:	e8 71 d8 ff ff       	call   3100 <_ZNSt8ios_baseC2Ev@plt>
    588f:	31 c0                	xor    %eax,%eax
    5891:	66 89 84 24 a0 01 00 	mov    %ax,0x1a0(%rsp)
    5898:	00 
    5899:	48 8d 0d 48 e0 00 00 	lea    0xe048(%rip),%rcx        # 138e8 <_ZTVSt9basic_iosIcSt11char_traitsIcEE+0x10>
    58a0:	48 89 8c 24 c0 00 00 	mov    %rcx,0xc0(%rsp)
    58a7:	00 
    58a8:	c5 f9 ef c0          	vpxor  %xmm0,%xmm0,%xmm0
    58ac:	48 8b 0d 1d e1 00 00 	mov    0xe11d(%rip),%rcx        # 139d0 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x10>
    58b3:	c5 fe 7f 84 24 a8 01 	vmovdqu %ymm0,0x1a8(%rsp)
    58ba:	00 00 
    58bc:	48 8b 41 e8          	mov    -0x18(%rcx),%rax
    58c0:	48 89 4c 24 40       	mov    %rcx,0x40(%rsp)
    58c5:	48 8b 0d 0c e1 00 00 	mov    0xe10c(%rip),%rcx        # 139d8 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x18>
    58cc:	48 c7 84 24 98 01 00 	movq   $0x0,0x198(%rsp)
    58d3:	00 00 00 00 00 
    58d8:	48 89 4c 04 40       	mov    %rcx,0x40(%rsp,%rax,1)
    58dd:	48 c7 44 24 48 00 00 	movq   $0x0,0x48(%rsp)
    58e4:	00 00 
    58e6:	48 8b 0d e3 e0 00 00 	mov    0xe0e3(%rip),%rcx        # 139d0 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x10>
    58ed:	31 f6                	xor    %esi,%esi
    58ef:	48 03 59 e8          	add    -0x18(%rcx),%rbx
    58f3:	48 89 df             	mov    %rbx,%rdi
    58f6:	c5 f8 77             	vzeroupper 
    58f9:	e8 32 d7 ff ff       	call   3030 <_ZNSt9basic_iosIcSt11char_traitsIcEE4initEPSt15basic_streambufIcS1_E@plt>
    58fe:	48 8b 0d db e0 00 00 	mov    0xe0db(%rip),%rcx        # 139e0 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x20>
    5905:	4c 8d 6c 24 50       	lea    0x50(%rsp),%r13
    590a:	48 89 4c 24 50       	mov    %rcx,0x50(%rsp)
    590f:	48 8b 49 e8          	mov    -0x18(%rcx),%rcx
    5913:	31 f6                	xor    %esi,%esi
    5915:	4c 01 e9             	add    %r13,%rcx
    5918:	48 89 cf             	mov    %rcx,%rdi
    591b:	48 8b 0d c6 e0 00 00 	mov    0xe0c6(%rip),%rcx        # 139e8 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x28>
    5922:	48 89 0f             	mov    %rcx,(%rdi)
    5925:	e8 06 d7 ff ff       	call   3030 <_ZNSt9basic_iosIcSt11char_traitsIcEE4initEPSt15basic_streambufIcS1_E@plt>
    592a:	48 8b 0d 97 e0 00 00 	mov    0xe097(%rip),%rcx        # 139c8 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x8>
    5931:	48 8b 35 b8 e0 00 00 	mov    0xe0b8(%rip),%rsi        # 139f0 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x30>
    5938:	48 8b 41 e8          	mov    -0x18(%rcx),%rax
    593c:	48 8d 0d 75 e1 00 00 	lea    0xe175(%rip),%rcx        # 13ab8 <_ZTVNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x18>
    5943:	48 89 74 04 40       	mov    %rsi,0x40(%rsp,%rax,1)
    5948:	48 8d 71 50          	lea    0x50(%rcx),%rsi
    594c:	48 89 4c 24 40       	mov    %rcx,0x40(%rsp)
    5951:	48 83 c1 28          	add    $0x28,%rcx
    5955:	48 89 4c 24 50       	mov    %rcx,0x50(%rsp)
    595a:	48 8d 8c 24 90 00 00 	lea    0x90(%rsp),%rcx
    5961:	00 
    5962:	48 89 cf             	mov    %rcx,%rdi
    5965:	48 8d 1d c4 df 00 00 	lea    0xdfc4(%rip),%rbx        # 13930 <_ZTVSt15basic_streambufIcSt11char_traitsIcEE+0x10>
    596c:	48 89 b4 24 c0 00 00 	mov    %rsi,0xc0(%rsp)
    5973:	00 
    5974:	48 89 4c 24 08       	mov    %rcx,0x8(%rsp)
    5979:	48 89 5c 24 58       	mov    %rbx,0x58(%rsp)
    597e:	48 c7 44 24 60 00 00 	movq   $0x0,0x60(%rsp)
    5985:	00 00 
    5987:	48 c7 44 24 68 00 00 	movq   $0x0,0x68(%rsp)
    598e:	00 00 
    5990:	48 c7 44 24 70 00 00 	movq   $0x0,0x70(%rsp)
    5997:	00 00 
    5999:	48 c7 44 24 78 00 00 	movq   $0x0,0x78(%rsp)
    59a0:	00 00 
    59a2:	48 c7 84 24 80 00 00 	movq   $0x0,0x80(%rsp)
    59a9:	00 00 00 00 00 
    59ae:	48 c7 84 24 88 00 00 	movq   $0x0,0x88(%rsp)
    59b5:	00 00 00 00 00 
    59ba:	4c 8d 64 24 58       	lea    0x58(%rsp),%r12
    59bf:	e8 4c d7 ff ff       	call   3110 <_ZNSt6localeC1Ev@plt>
    59c4:	48 8d 35 55 e0 00 00 	lea    0xe055(%rip),%rsi        # 13a20 <_ZTVNSt7__cxx1115basic_stringbufIcSt11char_traitsIcESaIcEEE+0x10>
    59cb:	48 8b 7c 24 18       	mov    0x18(%rsp),%rdi
    59d0:	48 8d 8c 24 b0 00 00 	lea    0xb0(%rsp),%rcx
    59d7:	00 
    59d8:	48 89 74 24 58       	mov    %rsi,0x58(%rsp)
    59dd:	4c 89 e6             	mov    %r12,%rsi
    59e0:	c7 84 24 98 00 00 00 	movl   $0x18,0x98(%rsp)
    59e7:	18 00 00 00 
    59eb:	48 89 4c 24 10       	mov    %rcx,0x10(%rsp)
    59f0:	48 89 8c 24 a0 00 00 	mov    %rcx,0xa0(%rsp)
    59f7:	00 
    59f8:	48 c7 84 24 a8 00 00 	movq   $0x0,0xa8(%rsp)
    59ff:	00 00 00 00 00 
    5a04:	c6 84 24 b0 00 00 00 	movb   $0x0,0xb0(%rsp)
    5a0b:	00 
    5a0c:	e8 1f d6 ff ff       	call   3030 <_ZNSt9basic_iosIcSt11char_traitsIcEE4initEPSt15basic_streambufIcS1_E@plt>
    5a11:	49 8b 1f             	mov    (%r15),%rbx
    5a14:	49 8b 47 08          	mov    0x8(%r15),%rax
    5a18:	48 29 d8             	sub    %rbx,%rax
    5a1b:	49 89 c4             	mov    %rax,%r12
    5a1e:	49 c1 fc 05          	sar    $0x5,%r12
    5a22:	48 85 c0             	test   %rax,%rax
    5a25:	7e 35                	jle    5a5c <_ZN8argparselsERSoRKNS_8ArgumentE.isra.0+0x21c>
    5a27:	66 0f 1f 84 00 00 00 	nopw   0x0(%rax,%rax,1)
    5a2e:	00 00 
    5a30:	48 8b 53 08          	mov    0x8(%rbx),%rdx
    5a34:	48 8b 33             	mov    (%rbx),%rsi
    5a37:	4c 89 ef             	mov    %r13,%rdi
    5a3a:	e8 61 d9 ff ff       	call   33a0 <_ZSt16__ostream_insertIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_PKS3_l@plt>
    5a3f:	ba 01 00 00 00       	mov    $0x1,%edx
    5a44:	48 8d 35 3e a7 00 00 	lea    0xa73e(%rip),%rsi        # 10189 <_fini+0xf68>
    5a4b:	4c 89 ef             	mov    %r13,%rdi
    5a4e:	e8 4d d9 ff ff       	call   33a0 <_ZSt16__ostream_insertIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_PKS3_l@plt>
    5a53:	48 83 c3 20          	add    $0x20,%rbx
    5a57:	49 ff cc             	dec    %r12
    5a5a:	75 d4                	jne    5a30 <_ZN8argparselsERSoRKNS_8ArgumentE.isra.0+0x1f0>
    5a5c:	48 8b 84 24 80 00 00 	mov    0x80(%rsp),%rax
    5a63:	00 
    5a64:	48 8d 5c 24 30       	lea    0x30(%rsp),%rbx
    5a69:	48 89 5c 24 20       	mov    %rbx,0x20(%rsp)
    5a6e:	48 c7 44 24 28 00 00 	movq   $0x0,0x28(%rsp)
    5a75:	00 00 
    5a77:	c6 44 24 30 00       	movb   $0x0,0x30(%rsp)
    5a7c:	48 8d 7c 24 20       	lea    0x20(%rsp),%rdi
    5a81:	48 85 c0             	test   %rax,%rax
    5a84:	0f 84 46 02 00 00    	je     5cd0 <_ZN8argparselsERSoRKNS_8ArgumentE.isra.0+0x490>
    5a8a:	4c 8b 44 24 70       	mov    0x70(%rsp),%r8
    5a8f:	48 8b 4c 24 78       	mov    0x78(%rsp),%rcx
    5a94:	4c 39 c0             	cmp    %r8,%rax
    5a97:	0f 87 d3 01 00 00    	ja     5c70 <_ZN8argparselsERSoRKNS_8ArgumentE.isra.0+0x430>
    5a9d:	49 29 c8             	sub    %rcx,%r8
    5aa0:	31 d2                	xor    %edx,%edx
    5aa2:	31 f6                	xor    %esi,%esi
    5aa4:	e8 07 d7 ff ff       	call   31b0 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE10_M_replaceEmmPKcm@plt>
    5aa9:	48 8b 54 24 28       	mov    0x28(%rsp),%rdx
    5aae:	48 8b 74 24 20       	mov    0x20(%rsp),%rsi
    5ab3:	4c 89 f7             	mov    %r14,%rdi
    5ab6:	e8 e5 d8 ff ff       	call   33a0 <_ZSt16__ostream_insertIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_PKS3_l@plt>
    5abb:	ba 01 00 00 00       	mov    $0x1,%edx
    5ac0:	48 8d 35 d2 a6 00 00 	lea    0xa6d2(%rip),%rsi        # 10199 <_fini+0xf78>
    5ac7:	48 89 c7             	mov    %rax,%rdi
    5aca:	49 89 c4             	mov    %rax,%r12
    5acd:	e8 ce d8 ff ff       	call   33a0 <_ZSt16__ostream_insertIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_PKS3_l@plt>
    5ad2:	49 8b 57 30          	mov    0x30(%r15),%rdx
    5ad6:	49 8b 77 28          	mov    0x28(%r15),%rsi
    5ada:	4c 89 e7             	mov    %r12,%rdi
    5add:	e8 be d8 ff ff       	call   33a0 <_ZSt16__ostream_insertIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_PKS3_l@plt>
    5ae2:	48 8b 7c 24 20       	mov    0x20(%rsp),%rdi
    5ae7:	48 39 df             	cmp    %rbx,%rdi
    5aea:	74 0e                	je     5afa <_ZN8argparselsERSoRKNS_8ArgumentE.isra.0+0x2ba>
    5aec:	48 8b 44 24 30       	mov    0x30(%rsp),%rax
    5af1:	48 8d 70 01          	lea    0x1(%rax),%rsi
    5af5:	e8 26 d8 ff ff       	call   3320 <_ZdlPvm@plt>
    5afa:	49 83 7f 48 00       	cmpq   $0x0,0x48(%r15)
    5aff:	0f 84 83 01 00 00    	je     5c88 <_ZN8argparselsERSoRKNS_8ArgumentE.isra.0+0x448>
    5b05:	49 83 7f 30 00       	cmpq   $0x0,0x30(%r15)
    5b0a:	74 14                	je     5b20 <_ZN8argparselsERSoRKNS_8ArgumentE.isra.0+0x2e0>
    5b0c:	ba 01 00 00 00       	mov    $0x1,%edx
    5b11:	48 8d 35 71 a6 00 00 	lea    0xa671(%rip),%rsi        # 10189 <_fini+0xf68>
    5b18:	4c 89 f7             	mov    %r14,%rdi
    5b1b:	e8 80 d8 ff ff       	call   33a0 <_ZSt16__ostream_insertIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_PKS3_l@plt>
    5b20:	ba 0a 00 00 00       	mov    $0xa,%edx
    5b25:	48 8d 35 6f a6 00 00 	lea    0xa66f(%rip),%rsi        # 1019b <_fini+0xf7a>
    5b2c:	4c 89 f7             	mov    %r14,%rdi
    5b2f:	e8 6c d8 ff ff       	call   33a0 <_ZSt16__ostream_insertIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_PKS3_l@plt>
    5b34:	49 8b 57 60          	mov    0x60(%r15),%rdx
    5b38:	49 8b 77 58          	mov    0x58(%r15),%rsi
    5b3c:	4c 89 f7             	mov    %r14,%rdi
    5b3f:	e8 5c d8 ff ff       	call   33a0 <_ZSt16__ostream_insertIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_PKS3_l@plt>
    5b44:	48 89 c7             	mov    %rax,%rdi
    5b47:	ba 01 00 00 00       	mov    $0x1,%edx
    5b4c:	48 8d 35 5c a6 00 00 	lea    0xa65c(%rip),%rsi        # 101af <_fini+0xf8e>
    5b53:	e8 48 d8 ff ff       	call   33a0 <_ZSt16__ostream_insertIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_PKS3_l@plt>
    5b58:	ba 01 00 00 00       	mov    $0x1,%edx
    5b5d:	48 8d 35 62 a6 00 00 	lea    0xa662(%rip),%rsi        # 101c6 <_fini+0xfa5>
    5b64:	4c 89 f7             	mov    %r14,%rdi
    5b67:	e8 34 d8 ff ff       	call   33a0 <_ZSt16__ostream_insertIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_PKS3_l@plt>
    5b6c:	48 8d 05 45 df 00 00 	lea    0xdf45(%rip),%rax        # 13ab8 <_ZTVNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x18>
    5b73:	48 89 44 24 40       	mov    %rax,0x40(%rsp)
    5b78:	c5 fa 7e 0d 28 e0 00 	vmovq  0xe028(%rip),%xmm1        # 13ba8 <_ZTVNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE@@Base+0x108>
    5b7f:	00 
    5b80:	48 83 c0 50          	add    $0x50,%rax
    5b84:	48 89 84 24 c0 00 00 	mov    %rax,0xc0(%rsp)
    5b8b:	00 
    5b8c:	48 8b bc 24 a0 00 00 	mov    0xa0(%rsp),%rdi
    5b93:	00 
    5b94:	48 8d 05 85 de 00 00 	lea    0xde85(%rip),%rax        # 13a20 <_ZTVNSt7__cxx1115basic_stringbufIcSt11char_traitsIcESaIcEEE+0x10>
    5b9b:	c4 e3 f1 22 c0 01    	vpinsrq $0x1,%rax,%xmm1,%xmm0
    5ba1:	c5 f9 7f 44 24 50    	vmovdqa %xmm0,0x50(%rsp)
    5ba7:	48 3b 7c 24 10       	cmp    0x10(%rsp),%rdi
    5bac:	74 11                	je     5bbf <_ZN8argparselsERSoRKNS_8ArgumentE.isra.0+0x37f>
    5bae:	48 8b 84 24 b0 00 00 	mov    0xb0(%rsp),%rax
    5bb5:	00 
    5bb6:	48 8d 70 01          	lea    0x1(%rax),%rsi
    5bba:	e8 61 d7 ff ff       	call   3320 <_ZdlPvm@plt>
    5bbf:	48 8b 7c 24 08       	mov    0x8(%rsp),%rdi
    5bc4:	48 8d 05 65 dd 00 00 	lea    0xdd65(%rip),%rax        # 13930 <_ZTVSt15basic_streambufIcSt11char_traitsIcEE+0x10>
    5bcb:	48 89 44 24 58       	mov    %rax,0x58(%rsp)
    5bd0:	e8 5b d6 ff ff       	call   3230 <_ZNSt6localeD1Ev@plt>
    5bd5:	48 8b 05 ec dd 00 00 	mov    0xddec(%rip),%rax        # 139c8 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x8>
    5bdc:	48 8b 15 0d de 00 00 	mov    0xde0d(%rip),%rdx        # 139f0 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x30>
    5be3:	48 8b 40 e8          	mov    -0x18(%rax),%rax
    5be7:	48 8b 0d fa dd 00 00 	mov    0xddfa(%rip),%rcx        # 139e8 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x28>
    5bee:	48 89 54 04 40       	mov    %rdx,0x40(%rsp,%rax,1)
    5bf3:	48 8b 05 e6 dd 00 00 	mov    0xdde6(%rip),%rax        # 139e0 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x20>
    5bfa:	48 8b 1d d7 dd 00 00 	mov    0xddd7(%rip),%rbx        # 139d8 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x18>
    5c01:	48 89 44 24 50       	mov    %rax,0x50(%rsp)
    5c06:	48 8b 40 e8          	mov    -0x18(%rax),%rax
    5c0a:	48 8b 7c 24 18       	mov    0x18(%rsp),%rdi
    5c0f:	48 89 4c 04 50       	mov    %rcx,0x50(%rsp,%rax,1)
    5c14:	48 8b 05 b5 dd 00 00 	mov    0xddb5(%rip),%rax        # 139d0 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x10>
    5c1b:	48 89 44 24 40       	mov    %rax,0x40(%rsp)
    5c20:	48 8b 40 e8          	mov    -0x18(%rax),%rax
    5c24:	48 89 5c 04 40       	mov    %rbx,0x40(%rsp,%rax,1)
    5c29:	48 8d 05 b8 dc 00 00 	lea    0xdcb8(%rip),%rax        # 138e8 <_ZTVSt9basic_iosIcSt11char_traitsIcEE+0x10>
    5c30:	48 89 84 24 c0 00 00 	mov    %rax,0xc0(%rsp)
    5c37:	00 
    5c38:	48 c7 44 24 48 00 00 	movq   $0x0,0x48(%rsp)
    5c3f:	00 00 
    5c41:	e8 5a d4 ff ff       	call   30a0 <_ZNSt8ios_baseD2Ev@plt>
    5c46:	48 8b 84 24 d8 01 00 	mov    0x1d8(%rsp),%rax
    5c4d:	00 
    5c4e:	64 48 2b 04 25 28 00 	sub    %fs:0x28,%rax
    5c55:	00 00 
    5c57:	0f 85 85 00 00 00    	jne    5ce2 <_ZN8argparselsERSoRKNS_8ArgumentE.isra.0+0x4a2>
    5c5d:	48 8d 65 d8          	lea    -0x28(%rbp),%rsp
    5c61:	5b                   	pop    %rbx
    5c62:	41 5c                	pop    %r12
    5c64:	41 5d                	pop    %r13
    5c66:	41 5e                	pop    %r14
    5c68:	41 5f                	pop    %r15
    5c6a:	5d                   	pop    %rbp
    5c6b:	c3                   	ret    
    5c6c:	0f 1f 40 00          	nopl   0x0(%rax)
    5c70:	48 29 c8             	sub    %rcx,%rax
    5c73:	49 89 c0             	mov    %rax,%r8
    5c76:	31 d2                	xor    %edx,%edx
    5c78:	31 f6                	xor    %esi,%esi
    5c7a:	e8 31 d5 ff ff       	call   31b0 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE10_M_replaceEmmPKcm@plt>
    5c7f:	e9 25 fe ff ff       	jmp    5aa9 <_ZN8argparselsERSoRKNS_8ArgumentE.isra.0+0x269>
    5c84:	0f 1f 40 00          	nopl   0x0(%rax)
    5c88:	41 f6 87 d9 00 00 00 	testb  $0x2,0xd9(%r15)
    5c8f:	02 
    5c90:	0f 84 c2 fe ff ff    	je     5b58 <_ZN8argparselsERSoRKNS_8ArgumentE.isra.0+0x318>
    5c96:	49 83 7f 30 00       	cmpq   $0x0,0x30(%r15)
    5c9b:	74 14                	je     5cb1 <_ZN8argparselsERSoRKNS_8ArgumentE.isra.0+0x471>
    5c9d:	ba 01 00 00 00       	mov    $0x1,%edx
    5ca2:	48 8d 35 e0 a4 00 00 	lea    0xa4e0(%rip),%rsi        # 10189 <_fini+0xf68>
    5ca9:	4c 89 f7             	mov    %r14,%rdi
    5cac:	e8 ef d6 ff ff       	call   33a0 <_ZSt16__ostream_insertIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_PKS3_l@plt>
    5cb1:	ba 0a 00 00 00       	mov    $0xa,%edx
    5cb6:	48 8d 35 e9 a4 00 00 	lea    0xa4e9(%rip),%rsi        # 101a6 <_fini+0xf85>
    5cbd:	4c 89 f7             	mov    %r14,%rdi
    5cc0:	e8 db d6 ff ff       	call   33a0 <_ZSt16__ostream_insertIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_PKS3_l@plt>
    5cc5:	e9 8e fe ff ff       	jmp    5b58 <_ZN8argparselsERSoRKNS_8ArgumentE.isra.0+0x318>
    5cca:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)
    5cd0:	48 8d b4 24 a0 00 00 	lea    0xa0(%rsp),%rsi
    5cd7:	00 
    5cd8:	e8 c3 d4 ff ff       	call   31a0 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE9_M_assignERKS4_@plt>
    5cdd:	e9 c7 fd ff ff       	jmp    5aa9 <_ZN8argparselsERSoRKNS_8ArgumentE.isra.0+0x269>
    5ce2:	e8 69 d4 ff ff       	call   3150 <__stack_chk_fail@plt>
    5ce7:	48 89 c3             	mov    %rax,%rbx
    5cea:	c5 f8 77             	vzeroupper 
    5ced:	e9 85 d7 ff ff       	jmp    3477 <_ZN8argparselsERSoRKNS_8ArgumentE.isra.0.cold+0x28>
    5cf2:	49 89 c4             	mov    %rax,%r12
    5cf5:	e9 9e d7 ff ff       	jmp    3498 <_ZN8argparselsERSoRKNS_8ArgumentE.isra.0.cold+0x49>
    5cfa:	49 89 c4             	mov    %rax,%r12
    5cfd:	e9 c6 d7 ff ff       	jmp    34c8 <_ZN8argparselsERSoRKNS_8ArgumentE.isra.0.cold+0x79>
    5d02:	49 89 c4             	mov    %rax,%r12
    5d05:	c5 f8 77             	vzeroupper 
    5d08:	e9 aa d7 ff ff       	jmp    34b7 <_ZN8argparselsERSoRKNS_8ArgumentE.isra.0.cold+0x68>
    5d0d:	48 89 c3             	mov    %rax,%rbx
    5d10:	e9 d5 d7 ff ff       	jmp    34ea <_ZN8argparselsERSoRKNS_8ArgumentE.isra.0.cold+0x9b>
    5d15:	48 89 c3             	mov    %rax,%rbx
    5d18:	e9 32 d7 ff ff       	jmp    344f <_ZN8argparselsERSoRKNS_8ArgumentE.isra.0.cold>
    5d1d:	0f 1f 00             	nopl   (%rax)

0000000000005d20 <_ZN8argparselsERSoRKNS_14ArgumentParserE.isra.0>:
    5d20:	55                   	push   %rbp
    5d21:	ba 07 00 00 00       	mov    $0x7,%edx
    5d26:	48 89 e5             	mov    %rsp,%rbp
    5d29:	41 57                	push   %r15
    5d2b:	41 56                	push   %r14
    5d2d:	49 89 fe             	mov    %rdi,%r14
    5d30:	41 55                	push   %r13
    5d32:	41 54                	push   %r12
    5d34:	49 89 f4             	mov    %rsi,%r12
    5d37:	48 8d 35 73 a4 00 00 	lea    0xa473(%rip),%rsi        # 101b1 <_fini+0xf90>
    5d3e:	53                   	push   %rbx
    5d3f:	48 8b 07             	mov    (%rdi),%rax
    5d42:	48 83 e4 e0          	and    $0xffffffffffffffe0,%rsp
    5d46:	48 8b 58 e8          	mov    -0x18(%rax),%rbx
    5d4a:	48 01 fb             	add    %rdi,%rbx
    5d4d:	83 4b 18 20          	orl    $0x20,0x18(%rbx)
    5d51:	e8 4a d6 ff ff       	call   33a0 <_ZSt16__ostream_insertIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_PKS3_l@plt>
    5d56:	49 8b 54 24 08       	mov    0x8(%r12),%rdx
    5d5b:	49 8b 34 24          	mov    (%r12),%rsi
    5d5f:	4c 89 f7             	mov    %r14,%rdi
    5d62:	e8 39 d6 ff ff       	call   33a0 <_ZSt16__ostream_insertIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_PKS3_l@plt>
    5d67:	48 89 c7             	mov    %rax,%rdi
    5d6a:	48 8d 35 48 a4 00 00 	lea    0xa448(%rip),%rsi        # 101b9 <_fini+0xf98>
    5d71:	e8 aa d3 ff ff       	call   3120 <_ZStlsISt11char_traitsIcEERSt13basic_ostreamIcT_ES5_PKc@plt>
    5d76:	49 8b 9c 24 e0 00 00 	mov    0xe0(%r12),%rbx
    5d7d:	00 
    5d7e:	48 85 db             	test   %rbx,%rbx
    5d81:	0f 84 19 01 00 00    	je     5ea0 <_ZN8argparselsERSoRKNS_14ArgumentParserE.isra.0+0x180>
    5d87:	49 8b bc 24 d0 00 00 	mov    0xd0(%r12),%rdi
    5d8e:	00 
    5d8f:	4d 8d ac 24 c0 00 00 	lea    0xc0(%r12),%r13
    5d96:	00 
    5d97:	31 db                	xor    %ebx,%ebx
    5d99:	4c 39 ef             	cmp    %r13,%rdi
    5d9c:	0f 84 fe 00 00 00    	je     5ea0 <_ZN8argparselsERSoRKNS_14ArgumentParserE.isra.0+0x180>
    5da2:	c5 fd 6f 15 f6 a9 00 	vmovdqa 0xa9f6(%rip),%ymm2        # 107a0 <_ZTSN8argparse7details12parse_numberIfLNS0_12chars_formatE3EEE+0x40>
    5da9:	00 
    5daa:	c5 fd 6f 25 0e aa 00 	vmovdqa 0xaa0e(%rip),%ymm4        # 107c0 <_ZTSN8argparse7details12parse_numberIfLNS0_12chars_formatE3EEE+0x60>
    5db1:	00 
    5db2:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)
    5db8:	48 8b 47 30          	mov    0x30(%rdi),%rax
    5dbc:	48 8b 70 18          	mov    0x18(%rax),%rsi
    5dc0:	48 8b 40 10          	mov    0x10(%rax),%rax
    5dc4:	48 39 c6             	cmp    %rax,%rsi
    5dc7:	0f 84 1d 02 00 00    	je     5fea <_ZN8argparselsERSoRKNS_14ArgumentParserE.isra.0+0x2ca>
    5dcd:	48 8d 4e e0          	lea    -0x20(%rsi),%rcx
    5dd1:	48 29 c1             	sub    %rax,%rcx
    5dd4:	49 89 c8             	mov    %rcx,%r8
    5dd7:	48 89 c2             	mov    %rax,%rdx
    5dda:	49 c1 e8 05          	shr    $0x5,%r8
    5dde:	48 81 f9 a0 03 00 00 	cmp    $0x3a0,%rcx
    5de5:	0f 86 06 02 00 00    	jbe    5ff1 <_ZN8argparselsERSoRKNS_14ArgumentParserE.isra.0+0x2d1>
    5deb:	48 83 e1 80          	and    $0xffffffffffffff80,%rcx
    5def:	48 01 c1             	add    %rax,%rcx
    5df2:	c5 e1 ef db          	vpxor  %xmm3,%xmm3,%xmm3
    5df6:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
    5dfd:	00 00 00 
    5e00:	c5 fe 6f 42 08       	vmovdqu 0x8(%rdx),%ymm0
    5e05:	c5 fe 6f 4a 48       	vmovdqu 0x48(%rdx),%ymm1
    5e0a:	62 f2 ed 28 7e 82 28 	vpermt2q 0x28(%rdx),%ymm2,%ymm0
    5e11:	00 00 00 
    5e14:	62 f2 ed 28 7e 8a 68 	vpermt2q 0x68(%rdx),%ymm2,%ymm1
    5e1b:	00 00 00 
    5e1e:	62 f2 ed 28 7e c1    	vpermt2q %ymm1,%ymm2,%ymm0
    5e24:	c5 fd d4 c4          	vpaddq %ymm4,%ymm0,%ymm0
    5e28:	48 83 ea 80          	sub    $0xffffffffffffff80,%rdx
    5e2c:	c5 e5 d4 d8          	vpaddq %ymm0,%ymm3,%ymm3
    5e30:	48 39 d1             	cmp    %rdx,%rcx
    5e33:	75 cb                	jne    5e00 <_ZN8argparselsERSoRKNS_14ArgumentParserE.isra.0+0xe0>
    5e35:	c5 f9 6f c3          	vmovdqa %xmm3,%xmm0
    5e39:	62 f3 fd 28 39 db 01 	vextracti64x2 $0x1,%ymm3,%xmm3
    5e40:	c5 f9 d4 c3          	vpaddq %xmm3,%xmm0,%xmm0
    5e44:	c5 f1 73 d8 08       	vpsrldq $0x8,%xmm0,%xmm1
    5e49:	49 83 e0 fc          	and    $0xfffffffffffffffc,%r8
    5e4d:	c5 f9 d4 c1          	vpaddq %xmm1,%xmm0,%xmm0
    5e51:	49 c1 e0 05          	shl    $0x5,%r8
    5e55:	c4 e1 f9 7e c2       	vmovq  %xmm0,%rdx
    5e5a:	4c 01 c0             	add    %r8,%rax
    5e5d:	0f 1f 00             	nopl   (%rax)
    5e60:	48 8b 48 08          	mov    0x8(%rax),%rcx
    5e64:	48 83 c0 20          	add    $0x20,%rax
    5e68:	48 8d 54 0a 01       	lea    0x1(%rdx,%rcx,1),%rdx
    5e6d:	48 39 c6             	cmp    %rax,%rsi
    5e70:	75 ee                	jne    5e60 <_ZN8argparselsERSoRKNS_14ArgumentParserE.isra.0+0x140>
    5e72:	48 39 d3             	cmp    %rdx,%rbx
    5e75:	48 0f 42 da          	cmovb  %rdx,%rbx
    5e79:	c5 f8 77             	vzeroupper 
    5e7c:	e8 5f d2 ff ff       	call   30e0 <_ZSt18_Rb_tree_incrementPKSt18_Rb_tree_node_base@plt>
    5e81:	49 39 c5             	cmp    %rax,%r13
    5e84:	c5 fd 6f 15 14 a9 00 	vmovdqa 0xa914(%rip),%ymm2        # 107a0 <_ZTSN8argparse7details12parse_numberIfLNS0_12chars_formatE3EEE+0x40>
    5e8b:	00 
    5e8c:	c5 fd 6f 25 2c a9 00 	vmovdqa 0xa92c(%rip),%ymm4        # 107c0 <_ZTSN8argparse7details12parse_numberIfLNS0_12chars_formatE3EEE+0x60>
    5e93:	00 
    5e94:	48 89 c7             	mov    %rax,%rdi
    5e97:	0f 85 1b ff ff ff    	jne    5db8 <_ZN8argparselsERSoRKNS_14ArgumentParserE.isra.0+0x98>
    5e9d:	c5 f8 77             	vzeroupper 
    5ea0:	4d 8b bc 24 88 00 00 	mov    0x88(%r12),%r15
    5ea7:	00 
    5ea8:	4d 8d ac 24 88 00 00 	lea    0x88(%r12),%r13
    5eaf:	00 
    5eb0:	4d 39 ef             	cmp    %r13,%r15
    5eb3:	74 32                	je     5ee7 <_ZN8argparselsERSoRKNS_14ArgumentParserE.isra.0+0x1c7>
    5eb5:	0f 1f 00             	nopl   (%rax)
    5eb8:	49 8b 47 10          	mov    0x10(%r15),%rax
    5ebc:	4c 89 f7             	mov    %r14,%rdi
    5ebf:	48 8b 50 08          	mov    0x8(%rax),%rdx
    5ec3:	48 8b 30             	mov    (%rax),%rsi
    5ec6:	e8 d5 d4 ff ff       	call   33a0 <_ZSt16__ostream_insertIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_PKS3_l@plt>
    5ecb:	48 89 c7             	mov    %rax,%rdi
    5ece:	ba 01 00 00 00       	mov    $0x1,%edx
    5ed3:	48 8d 35 af a2 00 00 	lea    0xa2af(%rip),%rsi        # 10189 <_fini+0xf68>
    5eda:	e8 c1 d4 ff ff       	call   33a0 <_ZSt16__ostream_insertIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_PKS3_l@plt>
    5edf:	4d 8b 3f             	mov    (%r15),%r15
    5ee2:	4d 39 fd             	cmp    %r15,%r13
    5ee5:	75 d1                	jne    5eb8 <_ZN8argparselsERSoRKNS_14ArgumentParserE.isra.0+0x198>
    5ee7:	48 8d 35 d7 a2 00 00 	lea    0xa2d7(%rip),%rsi        # 101c5 <_fini+0xfa4>
    5eee:	4c 89 f7             	mov    %r14,%rdi
    5ef1:	e8 2a d2 ff ff       	call   3120 <_ZStlsISt11char_traitsIcEERSt13basic_ostreamIcT_ES5_PKc@plt>
    5ef6:	49 8b 54 24 48       	mov    0x48(%r12),%rdx
    5efb:	48 85 d2             	test   %rdx,%rdx
    5efe:	0f 85 3d 01 00 00    	jne    6041 <_ZN8argparselsERSoRKNS_14ArgumentParserE.isra.0+0x321>
    5f04:	4d 3b ac 24 88 00 00 	cmp    0x88(%r12),%r13
    5f0b:	00 
    5f0c:	0f 84 e6 00 00 00    	je     5ff8 <_ZN8argparselsERSoRKNS_14ArgumentParserE.isra.0+0x2d8>
    5f12:	48 8d 35 af a2 00 00 	lea    0xa2af(%rip),%rsi        # 101c8 <_fini+0xfa7>
    5f19:	4c 89 f7             	mov    %r14,%rdi
    5f1c:	e8 ff d1 ff ff       	call   3120 <_ZStlsISt11char_traitsIcEERSt13basic_ostreamIcT_ES5_PKc@plt>
    5f21:	4d 8b bc 24 88 00 00 	mov    0x88(%r12),%r15
    5f28:	00 
    5f29:	4d 39 fd             	cmp    %r15,%r13
    5f2c:	0f 84 c6 00 00 00    	je     5ff8 <_ZN8argparselsERSoRKNS_14ArgumentParserE.isra.0+0x2d8>
    5f32:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)
    5f38:	49 8b 06             	mov    (%r14),%rax
    5f3b:	49 8d 77 10          	lea    0x10(%r15),%rsi
    5f3f:	48 8b 40 e8          	mov    -0x18(%rax),%rax
    5f43:	4c 89 f7             	mov    %r14,%rdi
    5f46:	49 89 5c 06 10       	mov    %rbx,0x10(%r14,%rax,1)
    5f4b:	e8 f0 f8 ff ff       	call   5840 <_ZN8argparselsERSoRKNS_8ArgumentE.isra.0>
    5f50:	4d 8b 3f             	mov    (%r15),%r15
    5f53:	4d 39 fd             	cmp    %r15,%r13
    5f56:	75 e0                	jne    5f38 <_ZN8argparselsERSoRKNS_14ArgumentParserE.isra.0+0x218>
    5f58:	4d 8d bc 24 a0 00 00 	lea    0xa0(%r12),%r15
    5f5f:	00 
    5f60:	4d 39 bc 24 a0 00 00 	cmp    %r15,0xa0(%r12)
    5f67:	00 
    5f68:	74 67                	je     5fd1 <_ZN8argparselsERSoRKNS_14ArgumentParserE.isra.0+0x2b1>
    5f6a:	4d 3b ac 24 88 00 00 	cmp    0x88(%r12),%r13
    5f71:	00 
    5f72:	48 8d 35 21 a2 00 00 	lea    0xa221(%rip),%rsi        # 1019a <_fini+0xf79>
    5f79:	48 8d 05 46 a2 00 00 	lea    0xa246(%rip),%rax        # 101c6 <_fini+0xfa5>
    5f80:	48 0f 45 f0          	cmovne %rax,%rsi
    5f84:	4c 89 f7             	mov    %r14,%rdi
    5f87:	e8 94 d1 ff ff       	call   3120 <_ZStlsISt11char_traitsIcEERSt13basic_ostreamIcT_ES5_PKc@plt>
    5f8c:	48 89 c7             	mov    %rax,%rdi
    5f8f:	48 8d 35 49 a2 00 00 	lea    0xa249(%rip),%rsi        # 101df <_fini+0xfbe>
    5f96:	e8 85 d1 ff ff       	call   3120 <_ZStlsISt11char_traitsIcEERSt13basic_ostreamIcT_ES5_PKc@plt>
    5f9b:	4d 8b ac 24 a0 00 00 	mov    0xa0(%r12),%r13
    5fa2:	00 
    5fa3:	4d 39 fd             	cmp    %r15,%r13
    5fa6:	74 29                	je     5fd1 <_ZN8argparselsERSoRKNS_14ArgumentParserE.isra.0+0x2b1>
    5fa8:	0f 1f 84 00 00 00 00 	nopl   0x0(%rax,%rax,1)
    5faf:	00 
    5fb0:	49 8b 06             	mov    (%r14),%rax
    5fb3:	49 8d 75 10          	lea    0x10(%r13),%rsi
    5fb7:	48 8b 40 e8          	mov    -0x18(%rax),%rax
    5fbb:	4c 89 f7             	mov    %r14,%rdi
    5fbe:	49 89 5c 06 10       	mov    %rbx,0x10(%r14,%rax,1)
    5fc3:	e8 78 f8 ff ff       	call   5840 <_ZN8argparselsERSoRKNS_8ArgumentE.isra.0>
    5fc8:	4d 8b 6d 00          	mov    0x0(%r13),%r13
    5fcc:	4d 39 fd             	cmp    %r15,%r13
    5fcf:	75 df                	jne    5fb0 <_ZN8argparselsERSoRKNS_14ArgumentParserE.isra.0+0x290>
    5fd1:	49 8b 54 24 68       	mov    0x68(%r12),%rdx
    5fd6:	48 85 d2             	test   %rdx,%rdx
    5fd9:	75 3c                	jne    6017 <_ZN8argparselsERSoRKNS_14ArgumentParserE.isra.0+0x2f7>
    5fdb:	48 8d 65 d8          	lea    -0x28(%rbp),%rsp
    5fdf:	5b                   	pop    %rbx
    5fe0:	41 5c                	pop    %r12
    5fe2:	41 5d                	pop    %r13
    5fe4:	41 5e                	pop    %r14
    5fe6:	41 5f                	pop    %r15
    5fe8:	5d                   	pop    %rbp
    5fe9:	c3                   	ret    
    5fea:	31 d2                	xor    %edx,%edx
    5fec:	e9 81 fe ff ff       	jmp    5e72 <_ZN8argparselsERSoRKNS_14ArgumentParserE.isra.0+0x152>
    5ff1:	31 d2                	xor    %edx,%edx
    5ff3:	e9 68 fe ff ff       	jmp    5e60 <_ZN8argparselsERSoRKNS_14ArgumentParserE.isra.0+0x140>
    5ff8:	4d 8d bc 24 a0 00 00 	lea    0xa0(%r12),%r15
    5fff:	00 
    6000:	48 8d 35 93 a1 00 00 	lea    0xa193(%rip),%rsi        # 1019a <_fini+0xf79>
    6007:	4d 39 bc 24 a0 00 00 	cmp    %r15,0xa0(%r12)
    600e:	00 
    600f:	0f 85 6f ff ff ff    	jne    5f84 <_ZN8argparselsERSoRKNS_14ArgumentParserE.isra.0+0x264>
    6015:	eb ba                	jmp    5fd1 <_ZN8argparselsERSoRKNS_14ArgumentParserE.isra.0+0x2b1>
    6017:	49 8b 74 24 60       	mov    0x60(%r12),%rsi
    601c:	4c 89 f7             	mov    %r14,%rdi
    601f:	e8 7c d3 ff ff       	call   33a0 <_ZSt16__ostream_insertIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_PKS3_l@plt>
    6024:	48 8d 65 d8          	lea    -0x28(%rbp),%rsp
    6028:	5b                   	pop    %rbx
    6029:	41 5c                	pop    %r12
    602b:	41 5d                	pop    %r13
    602d:	41 5e                	pop    %r14
    602f:	41 5f                	pop    %r15
    6031:	48 89 c7             	mov    %rax,%rdi
    6034:	48 8d 35 8a a1 00 00 	lea    0xa18a(%rip),%rsi        # 101c5 <_fini+0xfa4>
    603b:	5d                   	pop    %rbp
    603c:	e9 df d0 ff ff       	jmp    3120 <_ZStlsISt11char_traitsIcEERSt13basic_ostreamIcT_ES5_PKc@plt>
    6041:	49 8b 74 24 40       	mov    0x40(%r12),%rsi
    6046:	4c 89 f7             	mov    %r14,%rdi
    6049:	e8 52 d3 ff ff       	call   33a0 <_ZSt16__ostream_insertIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_PKS3_l@plt>
    604e:	48 89 c7             	mov    %rax,%rdi
    6051:	48 8d 35 6d a1 00 00 	lea    0xa16d(%rip),%rsi        # 101c5 <_fini+0xfa4>
    6058:	e8 c3 d0 ff ff       	call   3120 <_ZStlsISt11char_traitsIcEERSt13basic_ostreamIcT_ES5_PKc@plt>
    605d:	e9 a2 fe ff ff       	jmp    5f04 <_ZN8argparselsERSoRKNS_14ArgumentParserE.isra.0+0x1e4>
    6062:	66 66 2e 0f 1f 84 00 	data16 cs nopw 0x0(%rax,%rax,1)
    6069:	00 00 00 00 
    606d:	0f 1f 00             	nopl   (%rax)

0000000000006070 <_ZZN8argparse14ArgumentParserC4ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsEENKUlRKT_E_clIS6_EEDaSA_.isra.0>:
    6070:	41 54                	push   %r12
    6072:	55                   	push   %rbp
    6073:	48 89 fd             	mov    %rdi,%rbp
    6076:	48 81 ec b8 01 00 00 	sub    $0x1b8,%rsp
    607d:	4c 8d 64 24 20       	lea    0x20(%rsp),%r12
    6082:	4c 89 e7             	mov    %r12,%rdi
    6085:	64 48 8b 04 25 28 00 	mov    %fs:0x28,%rax
    608c:	00 00 
    608e:	48 89 84 24 a8 01 00 	mov    %rax,0x1a8(%rsp)
    6095:	00 
    6096:	31 c0                	xor    %eax,%eax
    6098:	e8 73 d2 ff ff       	call   3310 <_ZNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEC1Ev@plt>
    609d:	48 8d 7c 24 30       	lea    0x30(%rsp),%rdi
    60a2:	48 89 ee             	mov    %rbp,%rsi
    60a5:	e8 76 fc ff ff       	call   5d20 <_ZN8argparselsERSoRKNS_14ArgumentParserE.isra.0>
    60aa:	48 89 e7             	mov    %rsp,%rdi
    60ad:	48 8d 74 24 38       	lea    0x38(%rsp),%rsi
    60b2:	e8 c9 d0 ff ff       	call   3180 <_ZNKSt7__cxx1115basic_stringbufIcSt11char_traitsIcESaIcEE3strEv@plt>
    60b7:	48 8b 54 24 08       	mov    0x8(%rsp),%rdx
    60bc:	48 8b 34 24          	mov    (%rsp),%rsi
    60c0:	48 8d 3d 79 df 00 00 	lea    0xdf79(%rip),%rdi        # 14040 <_ZSt4cout@@Base>
    60c7:	e8 d4 d2 ff ff       	call   33a0 <_ZSt16__ostream_insertIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_PKS3_l@plt>
    60cc:	48 8b 3c 24          	mov    (%rsp),%rdi
    60d0:	48 8d 44 24 10       	lea    0x10(%rsp),%rax
    60d5:	48 39 c7             	cmp    %rax,%rdi
    60d8:	74 0e                	je     60e8 <_ZZN8argparse14ArgumentParserC4ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsEENKUlRKT_E_clIS6_EEDaSA_.isra.0+0x78>
    60da:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
    60df:	48 8d 70 01          	lea    0x1(%rax),%rsi
    60e3:	e8 38 d2 ff ff       	call   3320 <_ZdlPvm@plt>
    60e8:	4c 89 e7             	mov    %r12,%rdi
    60eb:	e8 90 d2 ff ff       	call   3380 <_ZNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEED1Ev@plt>
    60f0:	31 ff                	xor    %edi,%edi
    60f2:	e8 49 cf ff ff       	call   3040 <exit@plt>
    60f7:	48 89 c5             	mov    %rax,%rbp
    60fa:	e9 5b d4 ff ff       	jmp    355a <_ZZN8argparse14ArgumentParserC4ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsEENKUlRKT_E_clIS6_EEDaSA_.isra.0.cold>
    60ff:	48 89 c5             	mov    %rax,%rbp
    6102:	e9 66 d4 ff ff       	jmp    356d <_ZZN8argparse14ArgumentParserC4ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsEENKUlRKT_E_clIS6_EEDaSA_.isra.0.cold+0x13>
    6107:	48 89 c5             	mov    %rax,%rbp
    610a:	c5 f8 77             	vzeroupper 
    610d:	e9 7a d4 ff ff       	jmp    358c <_ZZN8argparse14ArgumentParserC4ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsEENKUlRKT_E_clIS6_EEDaSA_.isra.0.cold+0x32>
    6112:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
    6119:	00 00 00 
    611c:	0f 1f 40 00          	nopl   0x0(%rax)

0000000000006120 <_ZNKSt18bad_variant_access4whatEv>:
    6120:	48 8b 47 08          	mov    0x8(%rdi),%rax
    6124:	c3                   	ret    
    6125:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
    612c:	00 00 00 
    612f:	90                   	nop

0000000000006130 <_ZNSt3any17_Manager_internalIbE9_S_manageENS_3_OpEPKS_PNS_4_ArgE>:
    6130:	83 ff 02             	cmp    $0x2,%edi
    6133:	74 4b                	je     6180 <_ZNSt3any17_Manager_internalIbE9_S_manageENS_3_OpEPKS_PNS_4_ArgE+0x50>
    6135:	77 19                	ja     6150 <_ZNSt3any17_Manager_internalIbE9_S_manageENS_3_OpEPKS_PNS_4_ArgE+0x20>
    6137:	85 ff                	test   %edi,%edi
    6139:	74 3d                	je     6178 <_ZNSt3any17_Manager_internalIbE9_S_manageENS_3_OpEPKS_PNS_4_ArgE+0x48>
    613b:	48 8d 05 ce d7 00 00 	lea    0xd7ce(%rip),%rax        # 13910 <_ZTIb@@Base>
    6142:	48 89 02             	mov    %rax,(%rdx)
    6145:	c3                   	ret    
    6146:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
    614d:	00 00 00 
    6150:	83 ff 04             	cmp    $0x4,%edi
    6153:	75 1b                	jne    6170 <_ZNSt3any17_Manager_internalIbE9_S_manageENS_3_OpEPKS_PNS_4_ArgE+0x40>
    6155:	48 8b 02             	mov    (%rdx),%rax
    6158:	0f b6 4e 08          	movzbl 0x8(%rsi),%ecx
    615c:	48 8b 16             	mov    (%rsi),%rdx
    615f:	88 48 08             	mov    %cl,0x8(%rax)
    6162:	48 89 10             	mov    %rdx,(%rax)
    6165:	48 c7 06 00 00 00 00 	movq   $0x0,(%rsi)
    616c:	c3                   	ret    
    616d:	0f 1f 00             	nopl   (%rax)
    6170:	c3                   	ret    
    6171:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
    6178:	48 83 c6 08          	add    $0x8,%rsi
    617c:	48 89 32             	mov    %rsi,(%rdx)
    617f:	c3                   	ret    
    6180:	48 8b 02             	mov    (%rdx),%rax
    6183:	0f b6 56 08          	movzbl 0x8(%rsi),%edx
    6187:	88 50 08             	mov    %dl,0x8(%rax)
    618a:	48 8b 16             	mov    (%rsi),%rdx
    618d:	48 89 10             	mov    %rdx,(%rax)
    6190:	c3                   	ret    
    6191:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
    6198:	00 00 00 
    619b:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)

00000000000061a0 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFvOZNS0_16_Variant_storageILb0EJSt8functionIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEES4_IFvSD_EEEE8_M_resetEvEUlOT_E_RSt7variantIJSF_SH_EEEJEEESt16integer_sequenceImJLm0EEEE14__visit_invokeESM_SP_>:
    61a0:	48 8b 46 10          	mov    0x10(%rsi),%rax
    61a4:	48 85 c0             	test   %rax,%rax
    61a7:	74 17                	je     61c0 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFvOZNS0_16_Variant_storageILb0EJSt8functionIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEES4_IFvSD_EEEE8_M_resetEvEUlOT_E_RSt7variantIJSF_SH_EEEJEEESt16integer_sequenceImJLm0EEEE14__visit_invokeESM_SP_+0x20>
    61a9:	48 83 ec 08          	sub    $0x8,%rsp
    61ad:	48 89 f7             	mov    %rsi,%rdi
    61b0:	ba 03 00 00 00       	mov    $0x3,%edx
    61b5:	ff d0                	call   *%rax
    61b7:	48 83 c4 08          	add    $0x8,%rsp
    61bb:	c3                   	ret    
    61bc:	0f 1f 40 00          	nopl   0x0(%rax)
    61c0:	c3                   	ret    
    61c1:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
    61c8:	00 00 00 
    61cb:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)

00000000000061d0 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFvOZNS0_16_Variant_storageILb0EJSt8functionIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEES4_IFvSD_EEEE8_M_resetEvEUlOT_E_RSt7variantIJSF_SH_EEEJEEESt16integer_sequenceImJLm1EEEE14__visit_invokeESM_SP_>:
    61d0:	48 8b 46 10          	mov    0x10(%rsi),%rax
    61d4:	48 85 c0             	test   %rax,%rax
    61d7:	74 17                	je     61f0 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFvOZNS0_16_Variant_storageILb0EJSt8functionIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEES4_IFvSD_EEEE8_M_resetEvEUlOT_E_RSt7variantIJSF_SH_EEEJEEESt16integer_sequenceImJLm1EEEE14__visit_invokeESM_SP_+0x20>
    61d9:	48 83 ec 08          	sub    $0x8,%rsp
    61dd:	48 89 f7             	mov    %rsi,%rdi
    61e0:	ba 03 00 00 00       	mov    $0x3,%edx
    61e5:	ff d0                	call   *%rax
    61e7:	48 83 c4 08          	add    $0x8,%rsp
    61eb:	c3                   	ret    
    61ec:	0f 1f 40 00          	nopl   0x0(%rax)
    61f0:	c3                   	ret    
    61f1:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
    61f8:	00 00 00 
    61fb:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)

0000000000006200 <_ZNSt3any17_Manager_internalIiE9_S_manageENS_3_OpEPKS_PNS_4_ArgE>:
    6200:	83 ff 02             	cmp    $0x2,%edi
    6203:	74 4b                	je     6250 <_ZNSt3any17_Manager_internalIiE9_S_manageENS_3_OpEPKS_PNS_4_ArgE+0x50>
    6205:	77 19                	ja     6220 <_ZNSt3any17_Manager_internalIiE9_S_manageENS_3_OpEPKS_PNS_4_ArgE+0x20>
    6207:	85 ff                	test   %edi,%edi
    6209:	74 3d                	je     6248 <_ZNSt3any17_Manager_internalIiE9_S_manageENS_3_OpEPKS_PNS_4_ArgE+0x48>
    620b:	48 8d 05 7e d8 00 00 	lea    0xd87e(%rip),%rax        # 13a90 <_ZTIi@@Base>
    6212:	48 89 02             	mov    %rax,(%rdx)
    6215:	c3                   	ret    
    6216:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
    621d:	00 00 00 
    6220:	83 ff 04             	cmp    $0x4,%edi
    6223:	75 1b                	jne    6240 <_ZNSt3any17_Manager_internalIiE9_S_manageENS_3_OpEPKS_PNS_4_ArgE+0x40>
    6225:	48 8b 02             	mov    (%rdx),%rax
    6228:	8b 4e 08             	mov    0x8(%rsi),%ecx
    622b:	48 8b 16             	mov    (%rsi),%rdx
    622e:	89 48 08             	mov    %ecx,0x8(%rax)
    6231:	48 89 10             	mov    %rdx,(%rax)
    6234:	48 c7 06 00 00 00 00 	movq   $0x0,(%rsi)
    623b:	c3                   	ret    
    623c:	0f 1f 40 00          	nopl   0x0(%rax)
    6240:	c3                   	ret    
    6241:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
    6248:	48 83 c6 08          	add    $0x8,%rsi
    624c:	48 89 32             	mov    %rsi,(%rdx)
    624f:	c3                   	ret    
    6250:	48 8b 02             	mov    (%rdx),%rax
    6253:	8b 56 08             	mov    0x8(%rsi),%edx
    6256:	89 50 08             	mov    %edx,0x8(%rax)
    6259:	48 8b 16             	mov    (%rsi),%rdx
    625c:	48 89 10             	mov    %rdx,(%rax)
    625f:	c3                   	ret    

0000000000006260 <_ZNSt3any17_Manager_internalIfE9_S_manageENS_3_OpEPKS_PNS_4_ArgE>:
    6260:	83 ff 02             	cmp    $0x2,%edi
    6263:	74 4b                	je     62b0 <_ZNSt3any17_Manager_internalIfE9_S_manageENS_3_OpEPKS_PNS_4_ArgE+0x50>
    6265:	77 19                	ja     6280 <_ZNSt3any17_Manager_internalIfE9_S_manageENS_3_OpEPKS_PNS_4_ArgE+0x20>
    6267:	85 ff                	test   %edi,%edi
    6269:	74 3d                	je     62a8 <_ZNSt3any17_Manager_internalIfE9_S_manageENS_3_OpEPKS_PNS_4_ArgE+0x48>
    626b:	48 8d 05 8e d6 00 00 	lea    0xd68e(%rip),%rax        # 13900 <_ZTIf@@Base>
    6272:	48 89 02             	mov    %rax,(%rdx)
    6275:	c3                   	ret    
    6276:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
    627d:	00 00 00 
    6280:	83 ff 04             	cmp    $0x4,%edi
    6283:	75 1b                	jne    62a0 <_ZNSt3any17_Manager_internalIfE9_S_manageENS_3_OpEPKS_PNS_4_ArgE+0x40>
    6285:	48 8b 02             	mov    (%rdx),%rax
    6288:	c5 fa 10 46 08       	vmovss 0x8(%rsi),%xmm0
    628d:	48 8b 16             	mov    (%rsi),%rdx
    6290:	c5 fa 11 40 08       	vmovss %xmm0,0x8(%rax)
    6295:	48 89 10             	mov    %rdx,(%rax)
    6298:	48 c7 06 00 00 00 00 	movq   $0x0,(%rsi)
    629f:	c3                   	ret    
    62a0:	c3                   	ret    
    62a1:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
    62a8:	48 83 c6 08          	add    $0x8,%rsi
    62ac:	48 89 32             	mov    %rsi,(%rdx)
    62af:	c3                   	ret    
    62b0:	48 8b 02             	mov    (%rdx),%rax
    62b3:	c5 fa 10 46 08       	vmovss 0x8(%rsi),%xmm0
    62b8:	48 8b 16             	mov    (%rsi),%rdx
    62bb:	c5 fa 11 40 08       	vmovss %xmm0,0x8(%rax)
    62c0:	48 89 10             	mov    %rdx,(%rax)
    62c3:	c3                   	ret    
    62c4:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
    62cb:	00 00 00 
    62ce:	66 90                	xchg   %ax,%ax

00000000000062d0 <_ZNSt18bad_variant_accessD1Ev>:
    62d0:	48 8d 05 99 d5 00 00 	lea    0xd599(%rip),%rax        # 13870 <_ZTVSt18bad_variant_access+0x10>
    62d7:	48 89 07             	mov    %rax,(%rdi)
    62da:	e9 81 cf ff ff       	jmp    3260 <_ZNSt9exceptionD2Ev@plt>
    62df:	90                   	nop

00000000000062e0 <_ZNSt18bad_variant_accessD0Ev>:
    62e0:	48 8d 05 89 d5 00 00 	lea    0xd589(%rip),%rax        # 13870 <_ZTVSt18bad_variant_access+0x10>
    62e7:	55                   	push   %rbp
    62e8:	48 89 07             	mov    %rax,(%rdi)
    62eb:	48 89 fd             	mov    %rdi,%rbp
    62ee:	e8 6d cf ff ff       	call   3260 <_ZNSt9exceptionD2Ev@plt>
    62f3:	48 89 ef             	mov    %rbp,%rdi
    62f6:	be 10 00 00 00       	mov    $0x10,%esi
    62fb:	5d                   	pop    %rbp
    62fc:	e9 1f d0 ff ff       	jmp    3320 <_ZdlPvm@plt>
    6301:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
    6308:	00 00 00 
    630b:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)

0000000000006310 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse8Argument8m_actionMUlS8_E_EE10_M_managerERSt9_Any_dataRKSE_St18_Manager_operation>:
    6310:	85 d2                	test   %edx,%edx
    6312:	74 0c                	je     6320 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse8Argument8m_actionMUlS8_E_EE10_M_managerERSt9_Any_dataRKSE_St18_Manager_operation+0x10>
    6314:	83 fa 01             	cmp    $0x1,%edx
    6317:	75 03                	jne    631c <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse8Argument8m_actionMUlS8_E_EE10_M_managerERSt9_Any_dataRKSE_St18_Manager_operation+0xc>
    6319:	48 89 37             	mov    %rsi,(%rdi)
    631c:	31 c0                	xor    %eax,%eax
    631e:	c3                   	ret    
    631f:	90                   	nop
    6320:	48 8d 05 31 d8 00 00 	lea    0xd831(%rip),%rax        # 13b58 <_ZTVN10__cxxabiv117__class_type_infoE@Base>
    6327:	48 89 07             	mov    %rax,(%rdi)
    632a:	31 c0                	xor    %eax,%eax
    632c:	c3                   	ret    
    632d:	0f 1f 00             	nopl   (%rax)

0000000000006330 <_ZNSt17_Function_handlerIFvRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEZN8argparse14ArgumentParserC4ES5_S5_NS9_17default_argumentsEEUlRKT_E_E10_M_managerERSt9_Any_dataRKSH_St18_Manager_operation>:
    6330:	85 d2                	test   %edx,%edx
    6332:	74 14                	je     6348 <_ZNSt17_Function_handlerIFvRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEZN8argparse14ArgumentParserC4ES5_S5_NS9_17default_argumentsEEUlRKT_E_E10_M_managerERSt9_Any_dataRKSH_St18_Manager_operation+0x18>
    6334:	83 fa 01             	cmp    $0x1,%edx
    6337:	74 1f                	je     6358 <_ZNSt17_Function_handlerIFvRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEZN8argparse14ArgumentParserC4ES5_S5_NS9_17default_argumentsEEUlRKT_E_E10_M_managerERSt9_Any_dataRKSH_St18_Manager_operation+0x28>
    6339:	83 fa 02             	cmp    $0x2,%edx
    633c:	74 22                	je     6360 <_ZNSt17_Function_handlerIFvRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEZN8argparse14ArgumentParserC4ES5_S5_NS9_17default_argumentsEEUlRKT_E_E10_M_managerERSt9_Any_dataRKSH_St18_Manager_operation+0x30>
    633e:	31 c0                	xor    %eax,%eax
    6340:	c3                   	ret    
    6341:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
    6348:	48 8d 05 19 d8 00 00 	lea    0xd819(%rip),%rax        # 13b68 <_ZTVN10__cxxabiv117__class_type_infoE@Base>
    634f:	48 89 07             	mov    %rax,(%rdi)
    6352:	31 c0                	xor    %eax,%eax
    6354:	c3                   	ret    
    6355:	0f 1f 00             	nopl   (%rax)
    6358:	48 89 37             	mov    %rsi,(%rdi)
    635b:	31 c0                	xor    %eax,%eax
    635d:	c3                   	ret    
    635e:	66 90                	xchg   %ax,%ax
    6360:	48 8b 06             	mov    (%rsi),%rax
    6363:	48 89 07             	mov    %rax,(%rdi)
    6366:	eb d6                	jmp    633e <_ZNSt17_Function_handlerIFvRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEZN8argparse14ArgumentParserC4ES5_S5_NS9_17default_argumentsEEUlRKT_E_E10_M_managerERSt9_Any_dataRKSH_St18_Manager_operation+0xe>
    6368:	0f 1f 84 00 00 00 00 	nopl   0x0(%rax,%rax,1)
    636f:	00 

0000000000006370 <_ZNSt17_Function_handlerIFvRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEZN8argparse14ArgumentParserC4ES5_S5_NS9_17default_argumentsEEUlRKT_E0_E10_M_managerERSt9_Any_dataRKSH_St18_Manager_operation>:
    6370:	85 d2                	test   %edx,%edx
    6372:	74 14                	je     6388 <_ZNSt17_Function_handlerIFvRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEZN8argparse14ArgumentParserC4ES5_S5_NS9_17default_argumentsEEUlRKT_E0_E10_M_managerERSt9_Any_dataRKSH_St18_Manager_operation+0x18>
    6374:	83 fa 01             	cmp    $0x1,%edx
    6377:	74 1f                	je     6398 <_ZNSt17_Function_handlerIFvRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEZN8argparse14ArgumentParserC4ES5_S5_NS9_17default_argumentsEEUlRKT_E0_E10_M_managerERSt9_Any_dataRKSH_St18_Manager_operation+0x28>
    6379:	83 fa 02             	cmp    $0x2,%edx
    637c:	74 22                	je     63a0 <_ZNSt17_Function_handlerIFvRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEZN8argparse14ArgumentParserC4ES5_S5_NS9_17default_argumentsEEUlRKT_E0_E10_M_managerERSt9_Any_dataRKSH_St18_Manager_operation+0x30>
    637e:	31 c0                	xor    %eax,%eax
    6380:	c3                   	ret    
    6381:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
    6388:	48 8d 05 e9 d7 00 00 	lea    0xd7e9(%rip),%rax        # 13b78 <_ZTVN10__cxxabiv117__class_type_infoE@Base>
    638f:	48 89 07             	mov    %rax,(%rdi)
    6392:	31 c0                	xor    %eax,%eax
    6394:	c3                   	ret    
    6395:	0f 1f 00             	nopl   (%rax)
    6398:	48 89 37             	mov    %rsi,(%rdi)
    639b:	31 c0                	xor    %eax,%eax
    639d:	c3                   	ret    
    639e:	66 90                	xchg   %ax,%ax
    63a0:	48 8b 06             	mov    (%rsi),%rax
    63a3:	48 89 07             	mov    %rax,(%rdi)
    63a6:	eb d6                	jmp    637e <_ZNSt17_Function_handlerIFvRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEZN8argparse14ArgumentParserC4ES5_S5_NS9_17default_argumentsEEUlRKT_E0_E10_M_managerERSt9_Any_dataRKSH_St18_Manager_operation+0xe>
    63a8:	0f 1f 84 00 00 00 00 	nopl   0x0(%rax,%rax,1)
    63af:	00 

00000000000063b0 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIiLi0EEEE10_M_managerERSt9_Any_dataRKSF_St18_Manager_operation>:
    63b0:	85 d2                	test   %edx,%edx
    63b2:	74 0c                	je     63c0 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIiLi0EEEE10_M_managerERSt9_Any_dataRKSF_St18_Manager_operation+0x10>
    63b4:	83 fa 01             	cmp    $0x1,%edx
    63b7:	75 03                	jne    63bc <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIiLi0EEEE10_M_managerERSt9_Any_dataRKSF_St18_Manager_operation+0xc>
    63b9:	48 89 37             	mov    %rsi,(%rdi)
    63bc:	31 c0                	xor    %eax,%eax
    63be:	c3                   	ret    
    63bf:	90                   	nop
    63c0:	48 8d 05 c1 d7 00 00 	lea    0xd7c1(%rip),%rax        # 13b88 <_ZTVN10__cxxabiv117__class_type_infoE@Base>
    63c7:	48 89 07             	mov    %rax,(%rdi)
    63ca:	31 c0                	xor    %eax,%eax
    63cc:	c3                   	ret    
    63cd:	0f 1f 00             	nopl   (%rax)

00000000000063d0 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIfLNSB_12chars_formatE3EEEE10_M_managerERSt9_Any_dataRKSG_St18_Manager_operation>:
    63d0:	85 d2                	test   %edx,%edx
    63d2:	74 0c                	je     63e0 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIfLNSB_12chars_formatE3EEEE10_M_managerERSt9_Any_dataRKSG_St18_Manager_operation+0x10>
    63d4:	83 fa 01             	cmp    $0x1,%edx
    63d7:	75 03                	jne    63dc <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIfLNSB_12chars_formatE3EEEE10_M_managerERSt9_Any_dataRKSG_St18_Manager_operation+0xc>
    63d9:	48 89 37             	mov    %rsi,(%rdi)
    63dc:	31 c0                	xor    %eax,%eax
    63de:	c3                   	ret    
    63df:	90                   	nop
    63e0:	48 8d 05 b1 d7 00 00 	lea    0xd7b1(%rip),%rax        # 13b98 <_ZTVN10__cxxabiv117__class_type_infoE@Base>
    63e7:	48 89 07             	mov    %rax,(%rdi)
    63ea:	31 c0                	xor    %eax,%eax
    63ec:	c3                   	ret    
    63ed:	0f 1f 00             	nopl   (%rax)

00000000000063f0 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EEUlRKSM_E_RSt7variantIJSt8functionIFSt3anyRSG_EESU_IFvSW_EEEEEJEEESt16integer_sequenceImJLm1EEEE14__visit_invokeESS_S12_>:
    63f0:	55                   	push   %rbp
    63f1:	48 89 f0             	mov    %rsi,%rax
    63f4:	53                   	push   %rbx
    63f5:	48 83 ec 38          	sub    $0x38,%rsp
    63f9:	64 48 8b 14 25 28 00 	mov    %fs:0x28,%rdx
    6400:	00 00 
    6402:	48 89 54 24 28       	mov    %rdx,0x28(%rsp)
    6407:	31 d2                	xor    %edx,%edx
    6409:	48 8d 5c 24 10       	lea    0x10(%rsp),%rbx
    640e:	48 83 7e 10 00       	cmpq   $0x0,0x10(%rsi)
    6413:	48 89 1c 24          	mov    %rbx,(%rsp)
    6417:	48 c7 44 24 08 00 00 	movq   $0x0,0x8(%rsp)
    641e:	00 00 
    6420:	c6 44 24 10 00       	movb   $0x0,0x10(%rsp)
    6425:	74 37                	je     645e <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EEUlRKSM_E_RSt7variantIJSt8functionIFSt3anyRSG_EESU_IFvSW_EEEEEJEEESt16integer_sequenceImJLm1EEEE14__visit_invokeESS_S12_+0x6e>
    6427:	48 89 e6             	mov    %rsp,%rsi
    642a:	48 89 c7             	mov    %rax,%rdi
    642d:	ff 50 18             	call   *0x18(%rax)
    6430:	48 8b 3c 24          	mov    (%rsp),%rdi
    6434:	48 39 df             	cmp    %rbx,%rdi
    6437:	74 0e                	je     6447 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EEUlRKSM_E_RSt7variantIJSt8functionIFSt3anyRSG_EESU_IFvSW_EEEEEJEEESt16integer_sequenceImJLm1EEEE14__visit_invokeESS_S12_+0x57>
    6439:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
    643e:	48 8d 70 01          	lea    0x1(%rax),%rsi
    6442:	e8 d9 ce ff ff       	call   3320 <_ZdlPvm@plt>
    6447:	48 8b 44 24 28       	mov    0x28(%rsp),%rax
    644c:	64 48 2b 04 25 28 00 	sub    %fs:0x28,%rax
    6453:	00 00 
    6455:	75 0c                	jne    6463 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EEUlRKSM_E_RSt7variantIJSt8functionIFSt3anyRSG_EESU_IFvSW_EEEEEJEEESt16integer_sequenceImJLm1EEEE14__visit_invokeESS_S12_+0x73>
    6457:	48 83 c4 38          	add    $0x38,%rsp
    645b:	5b                   	pop    %rbx
    645c:	5d                   	pop    %rbp
    645d:	c3                   	ret    
    645e:	e8 5d cd ff ff       	call   31c0 <_ZSt25__throw_bad_function_callv@plt>
    6463:	e8 e8 cc ff ff       	call   3150 <__stack_chk_fail@plt>
    6468:	48 89 c5             	mov    %rax,%rbp
    646b:	48 8b 3c 24          	mov    (%rsp),%rdi
    646f:	48 39 df             	cmp    %rbx,%rdi
    6472:	74 19                	je     648d <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EEUlRKSM_E_RSt7variantIJSt8functionIFSt3anyRSG_EESU_IFvSW_EEEEEJEEESt16integer_sequenceImJLm1EEEE14__visit_invokeESS_S12_+0x9d>
    6474:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
    6479:	48 8d 70 01          	lea    0x1(%rax),%rsi
    647d:	c5 f8 77             	vzeroupper 
    6480:	e8 9b ce ff ff       	call   3320 <_ZdlPvm@plt>
    6485:	48 89 ef             	mov    %rbp,%rdi
    6488:	e8 d3 ce ff ff       	call   3360 <_Unwind_Resume@plt>
    648d:	c5 f8 77             	vzeroupper 
    6490:	eb f3                	jmp    6485 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EEUlRKSM_E_RSt7variantIJSt8functionIFSt3anyRSG_EESU_IFvSW_EEEEEJEEESt16integer_sequenceImJLm1EEEE14__visit_invokeESS_S12_+0x95>
    6492:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
    6499:	00 00 00 
    649c:	0f 1f 40 00          	nopl   0x0(%rax)

00000000000064a0 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EEUlRKSM_E_RSt7variantIJSt8functionIFSt3anyRSG_EESU_IFvSW_EEEEEJEEESt16integer_sequenceImJLm0EEEE14__visit_invokeESS_S12_>:
    64a0:	55                   	push   %rbp
    64a1:	53                   	push   %rbx
    64a2:	48 83 ec 48          	sub    $0x48,%rsp
    64a6:	64 48 8b 04 25 28 00 	mov    %fs:0x28,%rax
    64ad:	00 00 
    64af:	48 89 44 24 38       	mov    %rax,0x38(%rsp)
    64b4:	31 c0                	xor    %eax,%eax
    64b6:	48 8d 5c 24 20       	lea    0x20(%rsp),%rbx
    64bb:	48 83 7e 10 00       	cmpq   $0x0,0x10(%rsi)
    64c0:	48 89 5c 24 10       	mov    %rbx,0x10(%rsp)
    64c5:	48 c7 44 24 18 00 00 	movq   $0x0,0x18(%rsp)
    64cc:	00 00 
    64ce:	c6 44 24 20 00       	movb   $0x0,0x20(%rsp)
    64d3:	74 52                	je     6527 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EEUlRKSM_E_RSt7variantIJSt8functionIFSt3anyRSG_EESU_IFvSW_EEEEEJEEESt16integer_sequenceImJLm0EEEE14__visit_invokeESS_S12_+0x87>
    64d5:	48 89 e5             	mov    %rsp,%rbp
    64d8:	48 8d 54 24 10       	lea    0x10(%rsp),%rdx
    64dd:	48 89 ef             	mov    %rbp,%rdi
    64e0:	ff 56 18             	call   *0x18(%rsi)
    64e3:	48 8b 04 24          	mov    (%rsp),%rax
    64e7:	48 85 c0             	test   %rax,%rax
    64ea:	74 0c                	je     64f8 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EEUlRKSM_E_RSt7variantIJSt8functionIFSt3anyRSG_EESU_IFvSW_EEEEEJEEESt16integer_sequenceImJLm0EEEE14__visit_invokeESS_S12_+0x58>
    64ec:	31 d2                	xor    %edx,%edx
    64ee:	48 89 ee             	mov    %rbp,%rsi
    64f1:	bf 03 00 00 00       	mov    $0x3,%edi
    64f6:	ff d0                	call   *%rax
    64f8:	48 8b 7c 24 10       	mov    0x10(%rsp),%rdi
    64fd:	48 39 df             	cmp    %rbx,%rdi
    6500:	74 0e                	je     6510 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EEUlRKSM_E_RSt7variantIJSt8functionIFSt3anyRSG_EESU_IFvSW_EEEEEJEEESt16integer_sequenceImJLm0EEEE14__visit_invokeESS_S12_+0x70>
    6502:	48 8b 44 24 20       	mov    0x20(%rsp),%rax
    6507:	48 8d 70 01          	lea    0x1(%rax),%rsi
    650b:	e8 10 ce ff ff       	call   3320 <_ZdlPvm@plt>
    6510:	48 8b 44 24 38       	mov    0x38(%rsp),%rax
    6515:	64 48 2b 04 25 28 00 	sub    %fs:0x28,%rax
    651c:	00 00 
    651e:	75 0c                	jne    652c <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EEUlRKSM_E_RSt7variantIJSt8functionIFSt3anyRSG_EESU_IFvSW_EEEEEJEEESt16integer_sequenceImJLm0EEEE14__visit_invokeESS_S12_+0x8c>
    6520:	48 83 c4 48          	add    $0x48,%rsp
    6524:	5b                   	pop    %rbx
    6525:	5d                   	pop    %rbp
    6526:	c3                   	ret    
    6527:	e8 94 cc ff ff       	call   31c0 <_ZSt25__throw_bad_function_callv@plt>
    652c:	e8 1f cc ff ff       	call   3150 <__stack_chk_fail@plt>
    6531:	48 89 c5             	mov    %rax,%rbp
    6534:	48 8b 7c 24 10       	mov    0x10(%rsp),%rdi
    6539:	48 39 df             	cmp    %rbx,%rdi
    653c:	74 19                	je     6557 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EEUlRKSM_E_RSt7variantIJSt8functionIFSt3anyRSG_EESU_IFvSW_EEEEEJEEESt16integer_sequenceImJLm0EEEE14__visit_invokeESS_S12_+0xb7>
    653e:	48 8b 44 24 20       	mov    0x20(%rsp),%rax
    6543:	48 8d 70 01          	lea    0x1(%rax),%rsi
    6547:	c5 f8 77             	vzeroupper 
    654a:	e8 d1 cd ff ff       	call   3320 <_ZdlPvm@plt>
    654f:	48 89 ef             	mov    %rbp,%rdi
    6552:	e8 09 ce ff ff       	call   3360 <_Unwind_Resume@plt>
    6557:	c5 f8 77             	vzeroupper 
    655a:	eb f3                	jmp    654f <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EEUlRKSM_E_RSt7variantIJSt8functionIFSt3anyRSG_EESU_IFvSW_EEEEEJEEESt16integer_sequenceImJLm0EEEE14__visit_invokeESS_S12_+0xaf>
    655c:	0f 1f 40 00          	nopl   0x0(%rax)

0000000000006560 <_ZNSt17_Function_handlerIFvRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEZN8argparse14ArgumentParserC4ES5_S5_NS9_17default_argumentsEEUlRKT_E0_E9_M_invokeERKSt9_Any_dataS7_>:
    6560:	48 83 ec 08          	sub    $0x8,%rsp
    6564:	48 8b 07             	mov    (%rdi),%rax
    6567:	48 8d 3d d2 da 00 00 	lea    0xdad2(%rip),%rdi        # 14040 <_ZSt4cout@@Base>
    656e:	48 8b 50 28          	mov    0x28(%rax),%rdx
    6572:	48 8b 70 20          	mov    0x20(%rax),%rsi
    6576:	e8 25 ce ff ff       	call   33a0 <_ZSt16__ostream_insertIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_PKS3_l@plt>
    657b:	48 89 c7             	mov    %rax,%rdi
    657e:	e8 7d e0 ff ff       	call   4600 <_ZSt4endlIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_.isra.0>
    6583:	31 ff                	xor    %edi,%edi
    6585:	e8 b6 ca ff ff       	call   3040 <exit@plt>
    658a:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)

0000000000006590 <_ZNSt3any17_Manager_externalINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEE9_S_manageENS_3_OpEPKS_PNS_4_ArgE>:
    6590:	41 56                	push   %r14
    6592:	41 55                	push   %r13
    6594:	41 54                	push   %r12
    6596:	55                   	push   %rbp
    6597:	53                   	push   %rbx
    6598:	48 83 ec 10          	sub    $0x10,%rsp
    659c:	64 48 8b 04 25 28 00 	mov    %fs:0x28,%rax
    65a3:	00 00 
    65a5:	48 89 44 24 08       	mov    %rax,0x8(%rsp)
    65aa:	31 c0                	xor    %eax,%eax
    65ac:	4c 8b 66 08          	mov    0x8(%rsi),%r12
    65b0:	83 ff 04             	cmp    $0x4,%edi
    65b3:	77 1f                	ja     65d4 <_ZNSt3any17_Manager_externalINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEE9_S_manageENS_3_OpEPKS_PNS_4_ArgE+0x44>
    65b5:	48 89 d5             	mov    %rdx,%rbp
    65b8:	89 ff                	mov    %edi,%edi
    65ba:	48 8d 15 6b 9a 00 00 	lea    0x9a6b(%rip),%rdx        # 1002c <_fini+0xe0b>
    65c1:	48 63 04 ba          	movslq (%rdx,%rdi,4),%rax
    65c5:	48 89 f3             	mov    %rsi,%rbx
    65c8:	48 01 d0             	add    %rdx,%rax
    65cb:	ff e0                	jmp    *%rax
    65cd:	0f 1f 00             	nopl   (%rax)
    65d0:	4c 89 65 00          	mov    %r12,0x0(%rbp)
    65d4:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
    65d9:	64 48 2b 04 25 28 00 	sub    %fs:0x28,%rax
    65e0:	00 00 
    65e2:	0f 85 39 01 00 00    	jne    6721 <_ZNSt3any17_Manager_externalINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEE9_S_manageENS_3_OpEPKS_PNS_4_ArgE+0x191>
    65e8:	48 83 c4 10          	add    $0x10,%rsp
    65ec:	5b                   	pop    %rbx
    65ed:	5d                   	pop    %rbp
    65ee:	41 5c                	pop    %r12
    65f0:	41 5d                	pop    %r13
    65f2:	41 5e                	pop    %r14
    65f4:	c3                   	ret    
    65f5:	0f 1f 00             	nopl   (%rax)
    65f8:	4d 85 e4             	test   %r12,%r12
    65fb:	74 d7                	je     65d4 <_ZNSt3any17_Manager_externalINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEE9_S_manageENS_3_OpEPKS_PNS_4_ArgE+0x44>
    65fd:	49 8b 3c 24          	mov    (%r12),%rdi
    6601:	49 8d 44 24 10       	lea    0x10(%r12),%rax
    6606:	48 39 c7             	cmp    %rax,%rdi
    6609:	74 0e                	je     6619 <_ZNSt3any17_Manager_externalINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEE9_S_manageENS_3_OpEPKS_PNS_4_ArgE+0x89>
    660b:	49 8b 44 24 10       	mov    0x10(%r12),%rax
    6610:	48 8d 70 01          	lea    0x1(%rax),%rsi
    6614:	e8 07 cd ff ff       	call   3320 <_ZdlPvm@plt>
    6619:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
    661e:	64 48 2b 04 25 28 00 	sub    %fs:0x28,%rax
    6625:	00 00 
    6627:	0f 85 f4 00 00 00    	jne    6721 <_ZNSt3any17_Manager_externalINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEE9_S_manageENS_3_OpEPKS_PNS_4_ArgE+0x191>
    662d:	48 83 c4 10          	add    $0x10,%rsp
    6631:	5b                   	pop    %rbx
    6632:	5d                   	pop    %rbp
    6633:	4c 89 e7             	mov    %r12,%rdi
    6636:	41 5c                	pop    %r12
    6638:	41 5d                	pop    %r13
    663a:	be 20 00 00 00       	mov    $0x20,%esi
    663f:	41 5e                	pop    %r14
    6641:	e9 da cc ff ff       	jmp    3320 <_ZdlPvm@plt>
    6646:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
    664d:	00 00 00 
    6650:	48 8b 45 00          	mov    0x0(%rbp),%rax
    6654:	4c 89 60 08          	mov    %r12,0x8(%rax)
    6658:	48 8b 45 00          	mov    0x0(%rbp),%rax
    665c:	48 8b 16             	mov    (%rsi),%rdx
    665f:	48 89 10             	mov    %rdx,(%rax)
    6662:	48 c7 06 00 00 00 00 	movq   $0x0,(%rsi)
    6669:	e9 66 ff ff ff       	jmp    65d4 <_ZNSt3any17_Manager_externalINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEE9_S_manageENS_3_OpEPKS_PNS_4_ArgE+0x44>
    666e:	66 90                	xchg   %ax,%ax
    6670:	48 8d 05 d1 d4 00 00 	lea    0xd4d1(%rip),%rax        # 13b48 <_ZTVN10__cxxabiv117__class_type_infoE@Base>
    6677:	48 89 45 00          	mov    %rax,0x0(%rbp)
    667b:	e9 54 ff ff ff       	jmp    65d4 <_ZNSt3any17_Manager_externalINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEE9_S_manageENS_3_OpEPKS_PNS_4_ArgE+0x44>
    6680:	bf 20 00 00 00       	mov    $0x20,%edi
    6685:	e8 16 cc ff ff       	call   32a0 <_Znwm@plt>
    668a:	48 8d 78 10          	lea    0x10(%rax),%rdi
    668e:	48 89 38             	mov    %rdi,(%rax)
    6691:	49 89 c5             	mov    %rax,%r13
    6694:	4d 8b 34 24          	mov    (%r12),%r14
    6698:	4d 8b 64 24 08       	mov    0x8(%r12),%r12
    669d:	4c 89 f0             	mov    %r14,%rax
    66a0:	4c 01 e0             	add    %r12,%rax
    66a3:	74 05                	je     66aa <_ZNSt3any17_Manager_externalINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEE9_S_manageENS_3_OpEPKS_PNS_4_ArgE+0x11a>
    66a5:	4d 85 f6             	test   %r14,%r14
    66a8:	74 7c                	je     6726 <_ZNSt3any17_Manager_externalINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEE9_S_manageENS_3_OpEPKS_PNS_4_ArgE+0x196>
    66aa:	4c 89 24 24          	mov    %r12,(%rsp)
    66ae:	49 83 fc 0f          	cmp    $0xf,%r12
    66b2:	77 3c                	ja     66f0 <_ZNSt3any17_Manager_externalINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEE9_S_manageENS_3_OpEPKS_PNS_4_ArgE+0x160>
    66b4:	49 83 fc 01          	cmp    $0x1,%r12
    66b8:	75 2e                	jne    66e8 <_ZNSt3any17_Manager_externalINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEE9_S_manageENS_3_OpEPKS_PNS_4_ArgE+0x158>
    66ba:	41 0f b6 06          	movzbl (%r14),%eax
    66be:	41 88 45 10          	mov    %al,0x10(%r13)
    66c2:	4d 89 65 08          	mov    %r12,0x8(%r13)
    66c6:	42 c6 04 27 00       	movb   $0x0,(%rdi,%r12,1)
    66cb:	48 8b 45 00          	mov    0x0(%rbp),%rax
    66cf:	4c 89 68 08          	mov    %r13,0x8(%rax)
    66d3:	48 8b 45 00          	mov    0x0(%rbp),%rax
    66d7:	48 8b 13             	mov    (%rbx),%rdx
    66da:	48 89 10             	mov    %rdx,(%rax)
    66dd:	e9 f2 fe ff ff       	jmp    65d4 <_ZNSt3any17_Manager_externalINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEE9_S_manageENS_3_OpEPKS_PNS_4_ArgE+0x44>
    66e2:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)
    66e8:	4d 85 e4             	test   %r12,%r12
    66eb:	74 d5                	je     66c2 <_ZNSt3any17_Manager_externalINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEE9_S_manageENS_3_OpEPKS_PNS_4_ArgE+0x132>
    66ed:	eb 1d                	jmp    670c <_ZNSt3any17_Manager_externalINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEE9_S_manageENS_3_OpEPKS_PNS_4_ArgE+0x17c>
    66ef:	90                   	nop
    66f0:	48 89 e6             	mov    %rsp,%rsi
    66f3:	31 d2                	xor    %edx,%edx
    66f5:	4c 89 ef             	mov    %r13,%rdi
    66f8:	e8 03 cc ff ff       	call   3300 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE9_M_createERmm@plt>
    66fd:	49 89 45 00          	mov    %rax,0x0(%r13)
    6701:	48 89 c7             	mov    %rax,%rdi
    6704:	48 8b 04 24          	mov    (%rsp),%rax
    6708:	49 89 45 10          	mov    %rax,0x10(%r13)
    670c:	4c 89 e2             	mov    %r12,%rdx
    670f:	4c 89 f6             	mov    %r14,%rsi
    6712:	e8 29 ca ff ff       	call   3140 <memcpy@plt>
    6717:	4c 8b 24 24          	mov    (%rsp),%r12
    671b:	49 8b 7d 00          	mov    0x0(%r13),%rdi
    671f:	eb a1                	jmp    66c2 <_ZNSt3any17_Manager_externalINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEE9_S_manageENS_3_OpEPKS_PNS_4_ArgE+0x132>
    6721:	e8 2a ca ff ff       	call   3150 <__stack_chk_fail@plt>
    6726:	48 8d 3d d3 98 00 00 	lea    0x98d3(%rip),%rdi        # 10000 <_fini+0xddf>
    672d:	e8 8e cc ff ff       	call   33c0 <_ZSt19__throw_logic_errorPKc@plt>
    6732:	48 89 c5             	mov    %rax,%rbp
    6735:	be 20 00 00 00       	mov    $0x20,%esi
    673a:	4c 89 ef             	mov    %r13,%rdi
    673d:	c5 f8 77             	vzeroupper 
    6740:	e8 db cb ff ff       	call   3320 <_ZdlPvm@plt>
    6745:	48 89 ef             	mov    %rbp,%rdi
    6748:	e8 13 cc ff ff       	call   3360 <_Unwind_Resume@plt>
    674d:	0f 1f 00             	nopl   (%rax)

0000000000006750 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse8Argument8m_actionMUlS8_E_EE9_M_invokeERKSt9_Any_dataS8_>:
    6750:	41 55                	push   %r13
    6752:	49 89 fd             	mov    %rdi,%r13
    6755:	41 54                	push   %r12
    6757:	55                   	push   %rbp
    6758:	53                   	push   %rbx
    6759:	48 83 ec 48          	sub    $0x48,%rsp
    675d:	48 8b 2a             	mov    (%rdx),%rbp
    6760:	4c 8b 62 08          	mov    0x8(%rdx),%r12
    6764:	64 48 8b 04 25 28 00 	mov    %fs:0x28,%rax
    676b:	00 00 
    676d:	48 89 44 24 38       	mov    %rax,0x38(%rsp)
    6772:	31 c0                	xor    %eax,%eax
    6774:	48 89 e8             	mov    %rbp,%rax
    6777:	48 8d 5c 24 20       	lea    0x20(%rsp),%rbx
    677c:	4c 01 e0             	add    %r12,%rax
    677f:	48 89 5c 24 10       	mov    %rbx,0x10(%rsp)
    6784:	74 09                	je     678f <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse8Argument8m_actionMUlS8_E_EE9_M_invokeERKSt9_Any_dataS8_+0x3f>
    6786:	48 85 ed             	test   %rbp,%rbp
    6789:	0f 84 01 01 00 00    	je     6890 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse8Argument8m_actionMUlS8_E_EE9_M_invokeERKSt9_Any_dataS8_+0x140>
    678f:	4c 89 64 24 08       	mov    %r12,0x8(%rsp)
    6794:	49 83 fc 0f          	cmp    $0xf,%r12
    6798:	0f 87 a2 00 00 00    	ja     6840 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse8Argument8m_actionMUlS8_E_EE9_M_invokeERKSt9_Any_dataS8_+0xf0>
    679e:	49 83 fc 01          	cmp    $0x1,%r12
    67a2:	0f 85 88 00 00 00    	jne    6830 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse8Argument8m_actionMUlS8_E_EE9_M_invokeERKSt9_Any_dataS8_+0xe0>
    67a8:	0f b6 45 00          	movzbl 0x0(%rbp),%eax
    67ac:	88 44 24 20          	mov    %al,0x20(%rsp)
    67b0:	48 89 d8             	mov    %rbx,%rax
    67b3:	4c 89 64 24 18       	mov    %r12,0x18(%rsp)
    67b8:	42 c6 04 20 00       	movb   $0x0,(%rax,%r12,1)
    67bd:	48 8d 05 cc fd ff ff 	lea    -0x234(%rip),%rax        # 6590 <_ZNSt3any17_Manager_externalINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEE9_S_manageENS_3_OpEPKS_PNS_4_ArgE>
    67c4:	49 89 45 00          	mov    %rax,0x0(%r13)
    67c8:	49 c7 45 08 00 00 00 	movq   $0x0,0x8(%r13)
    67cf:	00 
    67d0:	bf 20 00 00 00       	mov    $0x20,%edi
    67d5:	e8 c6 ca ff ff       	call   32a0 <_Znwm@plt>
    67da:	48 8d 50 10          	lea    0x10(%rax),%rdx
    67de:	48 89 10             	mov    %rdx,(%rax)
    67e1:	48 8b 54 24 10       	mov    0x10(%rsp),%rdx
    67e6:	48 39 da             	cmp    %rbx,%rdx
    67e9:	0f 84 91 00 00 00    	je     6880 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse8Argument8m_actionMUlS8_E_EE9_M_invokeERKSt9_Any_dataS8_+0x130>
    67ef:	48 89 10             	mov    %rdx,(%rax)
    67f2:	48 8b 54 24 20       	mov    0x20(%rsp),%rdx
    67f7:	48 89 50 10          	mov    %rdx,0x10(%rax)
    67fb:	48 8b 54 24 18       	mov    0x18(%rsp),%rdx
    6800:	49 89 45 08          	mov    %rax,0x8(%r13)
    6804:	48 89 50 08          	mov    %rdx,0x8(%rax)
    6808:	48 8b 44 24 38       	mov    0x38(%rsp),%rax
    680d:	64 48 2b 04 25 28 00 	sub    %fs:0x28,%rax
    6814:	00 00 
    6816:	0f 85 80 00 00 00    	jne    689c <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse8Argument8m_actionMUlS8_E_EE9_M_invokeERKSt9_Any_dataS8_+0x14c>
    681c:	48 83 c4 48          	add    $0x48,%rsp
    6820:	5b                   	pop    %rbx
    6821:	5d                   	pop    %rbp
    6822:	41 5c                	pop    %r12
    6824:	4c 89 e8             	mov    %r13,%rax
    6827:	41 5d                	pop    %r13
    6829:	c3                   	ret    
    682a:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)
    6830:	4d 85 e4             	test   %r12,%r12
    6833:	75 6c                	jne    68a1 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse8Argument8m_actionMUlS8_E_EE9_M_invokeERKSt9_Any_dataS8_+0x151>
    6835:	48 89 d8             	mov    %rbx,%rax
    6838:	e9 76 ff ff ff       	jmp    67b3 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse8Argument8m_actionMUlS8_E_EE9_M_invokeERKSt9_Any_dataS8_+0x63>
    683d:	0f 1f 00             	nopl   (%rax)
    6840:	48 8d 7c 24 10       	lea    0x10(%rsp),%rdi
    6845:	48 8d 74 24 08       	lea    0x8(%rsp),%rsi
    684a:	31 d2                	xor    %edx,%edx
    684c:	e8 af ca ff ff       	call   3300 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE9_M_createERmm@plt>
    6851:	48 89 44 24 10       	mov    %rax,0x10(%rsp)
    6856:	48 89 c7             	mov    %rax,%rdi
    6859:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
    685e:	48 89 44 24 20       	mov    %rax,0x20(%rsp)
    6863:	4c 89 e2             	mov    %r12,%rdx
    6866:	48 89 ee             	mov    %rbp,%rsi
    6869:	e8 d2 c8 ff ff       	call   3140 <memcpy@plt>
    686e:	4c 8b 64 24 08       	mov    0x8(%rsp),%r12
    6873:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
    6878:	e9 36 ff ff ff       	jmp    67b3 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse8Argument8m_actionMUlS8_E_EE9_M_invokeERKSt9_Any_dataS8_+0x63>
    687d:	0f 1f 00             	nopl   (%rax)
    6880:	c5 f9 6f 44 24 20    	vmovdqa 0x20(%rsp),%xmm0
    6886:	c5 fa 7f 40 10       	vmovdqu %xmm0,0x10(%rax)
    688b:	e9 6b ff ff ff       	jmp    67fb <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse8Argument8m_actionMUlS8_E_EE9_M_invokeERKSt9_Any_dataS8_+0xab>
    6890:	48 8d 3d 69 97 00 00 	lea    0x9769(%rip),%rdi        # 10000 <_fini+0xddf>
    6897:	e8 24 cb ff ff       	call   33c0 <_ZSt19__throw_logic_errorPKc@plt>
    689c:	e8 af c8 ff ff       	call   3150 <__stack_chk_fail@plt>
    68a1:	48 89 df             	mov    %rbx,%rdi
    68a4:	eb bd                	jmp    6863 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse8Argument8m_actionMUlS8_E_EE9_M_invokeERKSt9_Any_dataS8_+0x113>
    68a6:	48 89 c5             	mov    %rax,%rbp
    68a9:	48 8b 7c 24 10       	mov    0x10(%rsp),%rdi
    68ae:	48 39 df             	cmp    %rbx,%rdi
    68b1:	74 19                	je     68cc <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse8Argument8m_actionMUlS8_E_EE9_M_invokeERKSt9_Any_dataS8_+0x17c>
    68b3:	48 8b 44 24 20       	mov    0x20(%rsp),%rax
    68b8:	48 8d 70 01          	lea    0x1(%rax),%rsi
    68bc:	c5 f8 77             	vzeroupper 
    68bf:	e8 5c ca ff ff       	call   3320 <_ZdlPvm@plt>
    68c4:	48 89 ef             	mov    %rbp,%rdi
    68c7:	e8 94 ca ff ff       	call   3360 <_Unwind_Resume@plt>
    68cc:	c5 f8 77             	vzeroupper 
    68cf:	eb f3                	jmp    68c4 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse8Argument8m_actionMUlS8_E_EE9_M_invokeERKSt9_Any_dataS8_+0x174>
    68d1:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
    68d8:	00 00 00 
    68db:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)

00000000000068e0 <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0>:
    68e0:	41 57                	push   %r15
    68e2:	41 56                	push   %r14
    68e4:	41 55                	push   %r13
    68e6:	41 54                	push   %r12
    68e8:	55                   	push   %rbp
    68e9:	48 8d 6f 10          	lea    0x10(%rdi),%rbp
    68ed:	53                   	push   %rbx
    68ee:	48 83 ec 48          	sub    $0x48,%rsp
    68f2:	64 48 8b 04 25 28 00 	mov    %fs:0x28,%rax
    68f9:	00 00 
    68fb:	48 89 44 24 38       	mov    %rax,0x38(%rsp)
    6900:	31 c0                	xor    %eax,%eax
    6902:	48 8d 44 24 20       	lea    0x20(%rsp),%rax
    6907:	48 89 44 24 08       	mov    %rax,0x8(%rsp)
    690c:	48 89 44 24 10       	mov    %rax,0x10(%rsp)
    6911:	48 8b 07             	mov    (%rdi),%rax
    6914:	48 39 e8             	cmp    %rbp,%rax
    6917:	0f 84 5b 02 00 00    	je     6b78 <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0x298>
    691d:	48 89 44 24 10       	mov    %rax,0x10(%rsp)
    6922:	48 8b 47 10          	mov    0x10(%rdi),%rax
    6926:	48 89 44 24 20       	mov    %rax,0x20(%rsp)
    692b:	4c 8b 7f 08          	mov    0x8(%rdi),%r15
    692f:	48 89 2f             	mov    %rbp,(%rdi)
    6932:	4c 89 7c 24 18       	mov    %r15,0x18(%rsp)
    6937:	48 c7 47 08 00 00 00 	movq   $0x0,0x8(%rdi)
    693e:	00 
    693f:	c6 47 10 00          	movb   $0x0,0x10(%rdi)
    6943:	eb 5d                	jmp    69a2 <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0xc2>
    6945:	0f 1f 00             	nopl   (%rax)
    6948:	0f 86 8a 01 00 00    	jbe    6ad8 <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0x1f8>
    694e:	4c 8b 6d d0          	mov    -0x30(%rbp),%r13
    6952:	48 8d 5d e0          	lea    -0x20(%rbp),%rbx
    6956:	48 8b 7d f0          	mov    -0x10(%rbp),%rdi
    695a:	4c 39 eb             	cmp    %r13,%rbx
    695d:	0f 84 1c 01 00 00    	je     6a7f <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0x19f>
    6963:	48 39 ef             	cmp    %rbp,%rdi
    6966:	0f 84 4c 01 00 00    	je     6ab8 <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0x1d8>
    696c:	48 8b 13             	mov    (%rbx),%rdx
    696f:	48 8b 43 20          	mov    0x20(%rbx),%rax
    6973:	4c 89 6b 10          	mov    %r13,0x10(%rbx)
    6977:	4c 89 63 18          	mov    %r12,0x18(%rbx)
    697b:	48 89 53 20          	mov    %rdx,0x20(%rbx)
    697f:	48 85 ff             	test   %rdi,%rdi
    6982:	0f 84 3f 01 00 00    	je     6ac7 <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0x1e7>
    6988:	48 89 7b f0          	mov    %rdi,-0x10(%rbx)
    698c:	48 89 03             	mov    %rax,(%rbx)
    698f:	48 c7 43 f8 00 00 00 	movq   $0x0,-0x8(%rbx)
    6996:	00 
    6997:	c6 07 00             	movb   $0x0,(%rdi)
    699a:	48 89 dd             	mov    %rbx,%rbp
    699d:	4c 8b 7c 24 18       	mov    0x18(%rsp),%r15
    69a2:	4c 8b 65 d8          	mov    -0x28(%rbp),%r12
    69a6:	48 8d 5d f0          	lea    -0x10(%rbp),%rbx
    69aa:	4d 39 fc             	cmp    %r15,%r12
    69ad:	75 99                	jne    6948 <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0x68>
    69af:	4c 8b 74 24 10       	mov    0x10(%rsp),%r14
    69b4:	4d 85 ff             	test   %r15,%r15
    69b7:	74 1a                	je     69d3 <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0xf3>
    69b9:	4c 8b 6d d0          	mov    -0x30(%rbp),%r13
    69bd:	4c 89 fa             	mov    %r15,%rdx
    69c0:	4c 89 ee             	mov    %r13,%rsi
    69c3:	4c 89 f7             	mov    %r14,%rdi
    69c6:	e8 a5 c8 ff ff       	call   3270 <memcmp@plt>
    69cb:	85 c0                	test   %eax,%eax
    69cd:	0f 85 95 00 00 00    	jne    6a68 <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0x188>
    69d3:	48 8b 3b             	mov    (%rbx),%rdi
    69d6:	4c 3b 74 24 08       	cmp    0x8(%rsp),%r14
    69db:	0f 84 0a 01 00 00    	je     6aeb <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0x20b>
    69e1:	48 8b 44 24 20       	mov    0x20(%rsp),%rax
    69e6:	48 39 ef             	cmp    %rbp,%rdi
    69e9:	0f 84 39 01 00 00    	je     6b28 <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0x248>
    69ef:	c4 c1 f9 6e cf       	vmovq  %r15,%xmm1
    69f4:	c4 e3 f1 22 c0 01    	vpinsrq $0x1,%rax,%xmm1,%xmm0
    69fa:	48 8b 53 10          	mov    0x10(%rbx),%rdx
    69fe:	4c 89 33             	mov    %r14,(%rbx)
    6a01:	c5 fa 7f 43 08       	vmovdqu %xmm0,0x8(%rbx)
    6a06:	48 85 ff             	test   %rdi,%rdi
    6a09:	0f 84 2c 01 00 00    	je     6b3b <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0x25b>
    6a0f:	48 89 7c 24 10       	mov    %rdi,0x10(%rsp)
    6a14:	48 89 54 24 20       	mov    %rdx,0x20(%rsp)
    6a19:	48 c7 44 24 18 00 00 	movq   $0x0,0x18(%rsp)
    6a20:	00 00 
    6a22:	c6 07 00             	movb   $0x0,(%rdi)
    6a25:	48 8b 7c 24 10       	mov    0x10(%rsp),%rdi
    6a2a:	48 3b 7c 24 08       	cmp    0x8(%rsp),%rdi
    6a2f:	74 0e                	je     6a3f <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0x15f>
    6a31:	48 8b 44 24 20       	mov    0x20(%rsp),%rax
    6a36:	48 8d 70 01          	lea    0x1(%rax),%rsi
    6a3a:	e8 e1 c8 ff ff       	call   3320 <_ZdlPvm@plt>
    6a3f:	48 8b 44 24 38       	mov    0x38(%rsp),%rax
    6a44:	64 48 2b 04 25 28 00 	sub    %fs:0x28,%rax
    6a4b:	00 00 
    6a4d:	0f 85 49 01 00 00    	jne    6b9c <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0x2bc>
    6a53:	48 83 c4 48          	add    $0x48,%rsp
    6a57:	5b                   	pop    %rbx
    6a58:	5d                   	pop    %rbp
    6a59:	41 5c                	pop    %r12
    6a5b:	41 5d                	pop    %r13
    6a5d:	41 5e                	pop    %r14
    6a5f:	41 5f                	pop    %r15
    6a61:	c3                   	ret    
    6a62:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)
    6a68:	0f 89 65 ff ff ff    	jns    69d3 <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0xf3>
    6a6e:	48 8d 5d e0          	lea    -0x20(%rbp),%rbx
    6a72:	48 8b 7d f0          	mov    -0x10(%rbp),%rdi
    6a76:	4c 39 eb             	cmp    %r13,%rbx
    6a79:	0f 85 e4 fe ff ff    	jne    6963 <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0x83>
    6a7f:	4d 85 e4             	test   %r12,%r12
    6a82:	74 1d                	je     6aa1 <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0x1c1>
    6a84:	49 83 fc 01          	cmp    $0x1,%r12
    6a88:	0f 84 d2 00 00 00    	je     6b60 <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0x280>
    6a8e:	4c 89 e2             	mov    %r12,%rdx
    6a91:	4c 89 ee             	mov    %r13,%rsi
    6a94:	e8 a7 c6 ff ff       	call   3140 <memcpy@plt>
    6a99:	4d 8b 65 f8          	mov    -0x8(%r13),%r12
    6a9d:	49 8b 7d 10          	mov    0x10(%r13),%rdi
    6aa1:	4d 89 65 18          	mov    %r12,0x18(%r13)
    6aa5:	42 c6 04 27 00       	movb   $0x0,(%rdi,%r12,1)
    6aaa:	49 8b 7d f0          	mov    -0x10(%r13),%rdi
    6aae:	e9 dc fe ff ff       	jmp    698f <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0xaf>
    6ab3:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)
    6ab8:	48 8b 03             	mov    (%rbx),%rax
    6abb:	4c 89 6b 10          	mov    %r13,0x10(%rbx)
    6abf:	4c 89 63 18          	mov    %r12,0x18(%rbx)
    6ac3:	48 89 43 20          	mov    %rax,0x20(%rbx)
    6ac7:	48 89 5b f0          	mov    %rbx,-0x10(%rbx)
    6acb:	48 89 df             	mov    %rbx,%rdi
    6ace:	e9 bc fe ff ff       	jmp    698f <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0xaf>
    6ad3:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)
    6ad8:	4c 8b 74 24 10       	mov    0x10(%rsp),%r14
    6add:	48 8b 3b             	mov    (%rbx),%rdi
    6ae0:	4c 3b 74 24 08       	cmp    0x8(%rsp),%r14
    6ae5:	0f 85 f6 fe ff ff    	jne    69e1 <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0x101>
    6aeb:	4d 85 ff             	test   %r15,%r15
    6aee:	74 1f                	je     6b0f <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0x22f>
    6af0:	49 83 ff 01          	cmp    $0x1,%r15
    6af4:	0f 84 8e 00 00 00    	je     6b88 <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0x2a8>
    6afa:	48 8b 74 24 08       	mov    0x8(%rsp),%rsi
    6aff:	4c 89 fa             	mov    %r15,%rdx
    6b02:	e8 39 c6 ff ff       	call   3140 <memcpy@plt>
    6b07:	4c 8b 7c 24 18       	mov    0x18(%rsp),%r15
    6b0c:	48 8b 3b             	mov    (%rbx),%rdi
    6b0f:	4c 89 7b 08          	mov    %r15,0x8(%rbx)
    6b13:	42 c6 04 3f 00       	movb   $0x0,(%rdi,%r15,1)
    6b18:	48 8b 7c 24 10       	mov    0x10(%rsp),%rdi
    6b1d:	e9 f7 fe ff ff       	jmp    6a19 <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0x139>
    6b22:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)
    6b28:	c4 c1 f9 6e d7       	vmovq  %r15,%xmm2
    6b2d:	4c 89 33             	mov    %r14,(%rbx)
    6b30:	c4 e3 e9 22 c0 01    	vpinsrq $0x1,%rax,%xmm2,%xmm0
    6b36:	c5 fa 7f 43 08       	vmovdqu %xmm0,0x8(%rbx)
    6b3b:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
    6b40:	48 89 44 24 10       	mov    %rax,0x10(%rsp)
    6b45:	48 8d 44 24 20       	lea    0x20(%rsp),%rax
    6b4a:	48 89 44 24 08       	mov    %rax,0x8(%rsp)
    6b4f:	48 89 c7             	mov    %rax,%rdi
    6b52:	e9 c2 fe ff ff       	jmp    6a19 <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0x139>
    6b57:	66 0f 1f 84 00 00 00 	nopw   0x0(%rax,%rax,1)
    6b5e:	00 00 
    6b60:	41 0f b6 45 00       	movzbl 0x0(%r13),%eax
    6b65:	88 07                	mov    %al,(%rdi)
    6b67:	4d 8b 65 f8          	mov    -0x8(%r13),%r12
    6b6b:	49 8b 7d 10          	mov    0x10(%r13),%rdi
    6b6f:	e9 2d ff ff ff       	jmp    6aa1 <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0x1c1>
    6b74:	0f 1f 40 00          	nopl   0x0(%rax)
    6b78:	c5 fa 6f 5f 10       	vmovdqu 0x10(%rdi),%xmm3
    6b7d:	c5 f9 7f 5c 24 20    	vmovdqa %xmm3,0x20(%rsp)
    6b83:	e9 a3 fd ff ff       	jmp    692b <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0x4b>
    6b88:	0f b6 44 24 20       	movzbl 0x20(%rsp),%eax
    6b8d:	88 07                	mov    %al,(%rdi)
    6b8f:	4c 8b 7c 24 18       	mov    0x18(%rsp),%r15
    6b94:	48 8b 3b             	mov    (%rbx),%rdi
    6b97:	e9 73 ff ff ff       	jmp    6b0f <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0x22f>
    6b9c:	e8 af c5 ff ff       	call   3150 <__stack_chk_fail@plt>
    6ba1:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
    6ba8:	00 00 00 
    6bab:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)

0000000000006bb0 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0>:
    6bb0:	41 57                	push   %r15
    6bb2:	41 56                	push   %r14
    6bb4:	41 55                	push   %r13
    6bb6:	41 54                	push   %r12
    6bb8:	55                   	push   %rbp
    6bb9:	53                   	push   %rbx
    6bba:	48 83 ec 58          	sub    $0x58,%rsp
    6bbe:	48 89 74 24 10       	mov    %rsi,0x10(%rsp)
    6bc3:	64 48 8b 04 25 28 00 	mov    %fs:0x28,%rax
    6bca:	00 00 
    6bcc:	48 89 44 24 48       	mov    %rax,0x48(%rsp)
    6bd1:	31 c0                	xor    %eax,%eax
    6bd3:	48 39 f7             	cmp    %rsi,%rdi
    6bd6:	0f 84 fc 01 00 00    	je     6dd8 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x228>
    6bdc:	48 8d 47 20          	lea    0x20(%rdi),%rax
    6be0:	49 89 fd             	mov    %rdi,%r13
    6be3:	48 39 c6             	cmp    %rax,%rsi
    6be6:	0f 84 ec 01 00 00    	je     6dd8 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x228>
    6bec:	48 8d 44 24 30       	lea    0x30(%rsp),%rax
    6bf1:	48 89 44 24 08       	mov    %rax,0x8(%rsp)
    6bf6:	4c 8d 7f 30          	lea    0x30(%rdi),%r15
    6bfa:	eb 25                	jmp    6c21 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x71>
    6bfc:	0f 1f 40 00          	nopl   0x0(%rax)
    6c00:	0f 82 fa 01 00 00    	jb     6e00 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x250>
    6c06:	48 89 ef             	mov    %rbp,%rdi
    6c09:	e8 d2 fc ff ff       	call   68e0 <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0>
    6c0e:	4d 8d 77 10          	lea    0x10(%r15),%r14
    6c12:	49 83 c7 20          	add    $0x20,%r15
    6c16:	4c 39 74 24 10       	cmp    %r14,0x10(%rsp)
    6c1b:	0f 84 b7 01 00 00    	je     6dd8 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x228>
    6c21:	49 8b 5f f8          	mov    -0x8(%r15),%rbx
    6c25:	49 8d 6f f0          	lea    -0x10(%r15),%rbp
    6c29:	49 3b 5d 08          	cmp    0x8(%r13),%rbx
    6c2d:	75 d1                	jne    6c00 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x50>
    6c2f:	48 85 db             	test   %rbx,%rbx
    6c32:	74 d2                	je     6c06 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x56>
    6c34:	49 8b 7f f0          	mov    -0x10(%r15),%rdi
    6c38:	49 8b 75 00          	mov    0x0(%r13),%rsi
    6c3c:	48 89 da             	mov    %rbx,%rdx
    6c3f:	48 89 7c 24 18       	mov    %rdi,0x18(%rsp)
    6c44:	e8 27 c6 ff ff       	call   3270 <memcmp@plt>
    6c49:	85 c0                	test   %eax,%eax
    6c4b:	79 b9                	jns    6c06 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x56>
    6c4d:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
    6c52:	48 8b 7c 24 18       	mov    0x18(%rsp),%rdi
    6c57:	48 89 44 24 20       	mov    %rax,0x20(%rsp)
    6c5c:	4c 39 ff             	cmp    %r15,%rdi
    6c5f:	0f 84 b2 01 00 00    	je     6e17 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x267>
    6c65:	49 8b 07             	mov    (%r15),%rax
    6c68:	4c 29 ed             	sub    %r13,%rbp
    6c6b:	49 89 ec             	mov    %rbp,%r12
    6c6e:	48 89 7c 24 20       	mov    %rdi,0x20(%rsp)
    6c73:	48 89 44 24 30       	mov    %rax,0x30(%rsp)
    6c78:	48 89 5c 24 28       	mov    %rbx,0x28(%rsp)
    6c7d:	4d 89 7f f0          	mov    %r15,-0x10(%r15)
    6c81:	49 c7 47 f8 00 00 00 	movq   $0x0,-0x8(%r15)
    6c88:	00 
    6c89:	41 c6 07 00          	movb   $0x0,(%r15)
    6c8d:	4d 8d 77 10          	lea    0x10(%r15),%r14
    6c91:	49 c1 fc 05          	sar    $0x5,%r12
    6c95:	48 85 ed             	test   %rbp,%rbp
    6c98:	0f 8e ac 00 00 00    	jle    6d4a <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x19a>
    6c9e:	49 8d 5f e0          	lea    -0x20(%r15),%rbx
    6ca2:	eb 44                	jmp    6ce8 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x138>
    6ca4:	0f 1f 40 00          	nopl   0x0(%rax)
    6ca8:	48 8d 43 20          	lea    0x20(%rbx),%rax
    6cac:	48 39 c7             	cmp    %rax,%rdi
    6caf:	74 77                	je     6d28 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x178>
    6cb1:	48 89 53 18          	mov    %rdx,0x18(%rbx)
    6cb5:	48 8b 13             	mov    (%rbx),%rdx
    6cb8:	48 8b 43 20          	mov    0x20(%rbx),%rax
    6cbc:	48 89 6b 10          	mov    %rbp,0x10(%rbx)
    6cc0:	48 89 53 20          	mov    %rdx,0x20(%rbx)
    6cc4:	48 85 ff             	test   %rdi,%rdi
    6cc7:	74 6e                	je     6d37 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x187>
    6cc9:	48 89 7b f0          	mov    %rdi,-0x10(%rbx)
    6ccd:	48 89 03             	mov    %rax,(%rbx)
    6cd0:	48 8b 43 f0          	mov    -0x10(%rbx),%rax
    6cd4:	48 c7 43 f8 00 00 00 	movq   $0x0,-0x8(%rbx)
    6cdb:	00 
    6cdc:	c6 00 00             	movb   $0x0,(%rax)
    6cdf:	48 83 eb 20          	sub    $0x20,%rbx
    6ce3:	49 ff cc             	dec    %r12
    6ce6:	74 58                	je     6d40 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x190>
    6ce8:	48 8b 6b f0          	mov    -0x10(%rbx),%rbp
    6cec:	48 8b 7b 10          	mov    0x10(%rbx),%rdi
    6cf0:	48 8b 53 f8          	mov    -0x8(%rbx),%rdx
    6cf4:	48 39 eb             	cmp    %rbp,%rbx
    6cf7:	75 af                	jne    6ca8 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0xf8>
    6cf9:	48 85 d2             	test   %rdx,%rdx
    6cfc:	74 12                	je     6d10 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x160>
    6cfe:	48 83 fa 01          	cmp    $0x1,%rdx
    6d02:	0f 84 88 01 00 00    	je     6e90 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x2e0>
    6d08:	48 89 de             	mov    %rbx,%rsi
    6d0b:	e8 30 c4 ff ff       	call   3140 <memcpy@plt>
    6d10:	48 8b 45 f8          	mov    -0x8(%rbp),%rax
    6d14:	48 8b 55 10          	mov    0x10(%rbp),%rdx
    6d18:	48 89 45 18          	mov    %rax,0x18(%rbp)
    6d1c:	c6 04 02 00          	movb   $0x0,(%rdx,%rax,1)
    6d20:	eb ae                	jmp    6cd0 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x120>
    6d22:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)
    6d28:	48 8b 03             	mov    (%rbx),%rax
    6d2b:	48 89 6b 10          	mov    %rbp,0x10(%rbx)
    6d2f:	48 89 53 18          	mov    %rdx,0x18(%rbx)
    6d33:	48 89 43 20          	mov    %rax,0x20(%rbx)
    6d37:	48 89 5b f0          	mov    %rbx,-0x10(%rbx)
    6d3b:	eb 93                	jmp    6cd0 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x120>
    6d3d:	0f 1f 00             	nopl   (%rax)
    6d40:	48 8b 7c 24 20       	mov    0x20(%rsp),%rdi
    6d45:	48 8b 5c 24 28       	mov    0x28(%rsp),%rbx
    6d4a:	4d 8b 4d 00          	mov    0x0(%r13),%r9
    6d4e:	48 3b 7c 24 08       	cmp    0x8(%rsp),%rdi
    6d53:	0f 84 ff 00 00 00    	je     6e58 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x2a8>
    6d59:	49 8d 55 10          	lea    0x10(%r13),%rdx
    6d5d:	48 8b 44 24 30       	mov    0x30(%rsp),%rax
    6d62:	49 39 d1             	cmp    %rdx,%r9
    6d65:	0f 84 35 01 00 00    	je     6ea0 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x2f0>
    6d6b:	c4 e1 f9 6e cb       	vmovq  %rbx,%xmm1
    6d70:	c4 e3 f1 22 c0 01    	vpinsrq $0x1,%rax,%xmm1,%xmm0
    6d76:	49 8b 55 10          	mov    0x10(%r13),%rdx
    6d7a:	49 89 7d 00          	mov    %rdi,0x0(%r13)
    6d7e:	c4 c1 7a 7f 45 08    	vmovdqu %xmm0,0x8(%r13)
    6d84:	4d 85 c9             	test   %r9,%r9
    6d87:	0f 84 28 01 00 00    	je     6eb5 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x305>
    6d8d:	4c 89 4c 24 20       	mov    %r9,0x20(%rsp)
    6d92:	48 89 54 24 30       	mov    %rdx,0x30(%rsp)
    6d97:	48 c7 44 24 28 00 00 	movq   $0x0,0x28(%rsp)
    6d9e:	00 00 
    6da0:	41 c6 01 00          	movb   $0x0,(%r9)
    6da4:	48 8b 7c 24 20       	mov    0x20(%rsp),%rdi
    6da9:	48 3b 7c 24 08       	cmp    0x8(%rsp),%rdi
    6dae:	0f 84 5e fe ff ff    	je     6c12 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x62>
    6db4:	48 8b 44 24 30       	mov    0x30(%rsp),%rax
    6db9:	49 83 c7 20          	add    $0x20,%r15
    6dbd:	48 8d 70 01          	lea    0x1(%rax),%rsi
    6dc1:	e8 5a c5 ff ff       	call   3320 <_ZdlPvm@plt>
    6dc6:	4c 39 74 24 10       	cmp    %r14,0x10(%rsp)
    6dcb:	0f 85 50 fe ff ff    	jne    6c21 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x71>
    6dd1:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
    6dd8:	48 8b 44 24 48       	mov    0x48(%rsp),%rax
    6ddd:	64 48 2b 04 25 28 00 	sub    %fs:0x28,%rax
    6de4:	00 00 
    6de6:	0f 85 ff 00 00 00    	jne    6eeb <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x33b>
    6dec:	48 83 c4 58          	add    $0x58,%rsp
    6df0:	5b                   	pop    %rbx
    6df1:	5d                   	pop    %rbp
    6df2:	41 5c                	pop    %r12
    6df4:	41 5d                	pop    %r13
    6df6:	41 5e                	pop    %r14
    6df8:	41 5f                	pop    %r15
    6dfa:	c3                   	ret    
    6dfb:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)
    6e00:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
    6e05:	49 8b 7f f0          	mov    -0x10(%r15),%rdi
    6e09:	48 89 44 24 20       	mov    %rax,0x20(%rsp)
    6e0e:	4c 39 ff             	cmp    %r15,%rdi
    6e11:	0f 85 4e fe ff ff    	jne    6c65 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0xb5>
    6e17:	c4 c1 7a 6f 1f       	vmovdqu (%r15),%xmm3
    6e1c:	4c 29 ed             	sub    %r13,%rbp
    6e1f:	49 89 ec             	mov    %rbp,%r12
    6e22:	48 89 5c 24 28       	mov    %rbx,0x28(%rsp)
    6e27:	4d 89 7f f0          	mov    %r15,-0x10(%r15)
    6e2b:	49 c7 47 f8 00 00 00 	movq   $0x0,-0x8(%r15)
    6e32:	00 
    6e33:	41 c6 07 00          	movb   $0x0,(%r15)
    6e37:	c5 f9 7f 5c 24 30    	vmovdqa %xmm3,0x30(%rsp)
    6e3d:	4d 8d 77 10          	lea    0x10(%r15),%r14
    6e41:	49 c1 fc 05          	sar    $0x5,%r12
    6e45:	48 85 ed             	test   %rbp,%rbp
    6e48:	0f 8f 50 fe ff ff    	jg     6c9e <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0xee>
    6e4e:	4d 8b 4d 00          	mov    0x0(%r13),%r9
    6e52:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)
    6e58:	48 85 db             	test   %rbx,%rbx
    6e5b:	74 1f                	je     6e7c <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x2cc>
    6e5d:	48 83 fb 01          	cmp    $0x1,%rbx
    6e61:	74 64                	je     6ec7 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x317>
    6e63:	48 8b 74 24 08       	mov    0x8(%rsp),%rsi
    6e68:	48 89 da             	mov    %rbx,%rdx
    6e6b:	4c 89 cf             	mov    %r9,%rdi
    6e6e:	e8 cd c2 ff ff       	call   3140 <memcpy@plt>
    6e73:	48 8b 5c 24 28       	mov    0x28(%rsp),%rbx
    6e78:	4d 8b 4d 00          	mov    0x0(%r13),%r9
    6e7c:	49 89 5d 08          	mov    %rbx,0x8(%r13)
    6e80:	41 c6 04 19 00       	movb   $0x0,(%r9,%rbx,1)
    6e85:	4c 8b 4c 24 20       	mov    0x20(%rsp),%r9
    6e8a:	e9 08 ff ff ff       	jmp    6d97 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x1e7>
    6e8f:	90                   	nop
    6e90:	0f b6 03             	movzbl (%rbx),%eax
    6e93:	88 07                	mov    %al,(%rdi)
    6e95:	e9 76 fe ff ff       	jmp    6d10 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x160>
    6e9a:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)
    6ea0:	c4 e1 f9 6e d3       	vmovq  %rbx,%xmm2
    6ea5:	49 89 7d 00          	mov    %rdi,0x0(%r13)
    6ea9:	c4 e3 e9 22 c0 01    	vpinsrq $0x1,%rax,%xmm2,%xmm0
    6eaf:	c4 c1 7a 7f 45 08    	vmovdqu %xmm0,0x8(%r13)
    6eb5:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
    6eba:	48 89 44 24 20       	mov    %rax,0x20(%rsp)
    6ebf:	49 89 c1             	mov    %rax,%r9
    6ec2:	e9 d0 fe ff ff       	jmp    6d97 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x1e7>
    6ec7:	0f b6 44 24 30       	movzbl 0x30(%rsp),%eax
    6ecc:	41 88 01             	mov    %al,(%r9)
    6ecf:	48 8b 5c 24 28       	mov    0x28(%rsp),%rbx
    6ed4:	4d 8b 4d 00          	mov    0x0(%r13),%r9
    6ed8:	49 89 5d 08          	mov    %rbx,0x8(%r13)
    6edc:	41 c6 04 19 00       	movb   $0x0,(%r9,%rbx,1)
    6ee1:	4c 8b 4c 24 20       	mov    0x20(%rsp),%r9
    6ee6:	e9 ac fe ff ff       	jmp    6d97 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x1e7>
    6eeb:	e8 60 c2 ff ff       	call   3150 <__stack_chk_fail@plt>

0000000000006ef0 <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0>:
    6ef0:	41 57                	push   %r15
    6ef2:	41 56                	push   %r14
    6ef4:	41 55                	push   %r13
    6ef6:	41 54                	push   %r12
    6ef8:	55                   	push   %rbp
    6ef9:	48 8d 6f 10          	lea    0x10(%rdi),%rbp
    6efd:	53                   	push   %rbx
    6efe:	48 83 ec 48          	sub    $0x48,%rsp
    6f02:	64 48 8b 04 25 28 00 	mov    %fs:0x28,%rax
    6f09:	00 00 
    6f0b:	48 89 44 24 38       	mov    %rax,0x38(%rsp)
    6f10:	31 c0                	xor    %eax,%eax
    6f12:	48 8d 44 24 20       	lea    0x20(%rsp),%rax
    6f17:	48 89 44 24 08       	mov    %rax,0x8(%rsp)
    6f1c:	48 89 44 24 10       	mov    %rax,0x10(%rsp)
    6f21:	48 8b 07             	mov    (%rdi),%rax
    6f24:	48 39 e8             	cmp    %rbp,%rax
    6f27:	0f 84 5b 02 00 00    	je     7188 <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0x298>
    6f2d:	48 89 44 24 10       	mov    %rax,0x10(%rsp)
    6f32:	48 8b 47 10          	mov    0x10(%rdi),%rax
    6f36:	48 89 44 24 20       	mov    %rax,0x20(%rsp)
    6f3b:	4c 8b 7f 08          	mov    0x8(%rdi),%r15
    6f3f:	48 89 2f             	mov    %rbp,(%rdi)
    6f42:	4c 89 7c 24 18       	mov    %r15,0x18(%rsp)
    6f47:	48 c7 47 08 00 00 00 	movq   $0x0,0x8(%rdi)
    6f4e:	00 
    6f4f:	c6 47 10 00          	movb   $0x0,0x10(%rdi)
    6f53:	eb 5d                	jmp    6fb2 <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0xc2>
    6f55:	0f 1f 00             	nopl   (%rax)
    6f58:	0f 86 8a 01 00 00    	jbe    70e8 <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0x1f8>
    6f5e:	4c 8b 6d d0          	mov    -0x30(%rbp),%r13
    6f62:	48 8d 5d e0          	lea    -0x20(%rbp),%rbx
    6f66:	48 8b 7d f0          	mov    -0x10(%rbp),%rdi
    6f6a:	4c 39 eb             	cmp    %r13,%rbx
    6f6d:	0f 84 1c 01 00 00    	je     708f <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0x19f>
    6f73:	48 39 ef             	cmp    %rbp,%rdi
    6f76:	0f 84 4c 01 00 00    	je     70c8 <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0x1d8>
    6f7c:	48 8b 13             	mov    (%rbx),%rdx
    6f7f:	48 8b 43 20          	mov    0x20(%rbx),%rax
    6f83:	4c 89 6b 10          	mov    %r13,0x10(%rbx)
    6f87:	4c 89 63 18          	mov    %r12,0x18(%rbx)
    6f8b:	48 89 53 20          	mov    %rdx,0x20(%rbx)
    6f8f:	48 85 ff             	test   %rdi,%rdi
    6f92:	0f 84 3f 01 00 00    	je     70d7 <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0x1e7>
    6f98:	48 89 7b f0          	mov    %rdi,-0x10(%rbx)
    6f9c:	48 89 03             	mov    %rax,(%rbx)
    6f9f:	48 c7 43 f8 00 00 00 	movq   $0x0,-0x8(%rbx)
    6fa6:	00 
    6fa7:	c6 07 00             	movb   $0x0,(%rdi)
    6faa:	48 89 dd             	mov    %rbx,%rbp
    6fad:	4c 8b 7c 24 18       	mov    0x18(%rsp),%r15
    6fb2:	4c 8b 65 d8          	mov    -0x28(%rbp),%r12
    6fb6:	48 8d 5d f0          	lea    -0x10(%rbp),%rbx
    6fba:	4d 39 fc             	cmp    %r15,%r12
    6fbd:	75 99                	jne    6f58 <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0x68>
    6fbf:	4c 8b 74 24 10       	mov    0x10(%rsp),%r14
    6fc4:	4d 85 ff             	test   %r15,%r15
    6fc7:	74 1a                	je     6fe3 <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0xf3>
    6fc9:	4c 8b 6d d0          	mov    -0x30(%rbp),%r13
    6fcd:	4c 89 fa             	mov    %r15,%rdx
    6fd0:	4c 89 ee             	mov    %r13,%rsi
    6fd3:	4c 89 f7             	mov    %r14,%rdi
    6fd6:	e8 95 c2 ff ff       	call   3270 <memcmp@plt>
    6fdb:	85 c0                	test   %eax,%eax
    6fdd:	0f 85 95 00 00 00    	jne    7078 <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0x188>
    6fe3:	48 8b 3b             	mov    (%rbx),%rdi
    6fe6:	4c 3b 74 24 08       	cmp    0x8(%rsp),%r14
    6feb:	0f 84 0a 01 00 00    	je     70fb <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0x20b>
    6ff1:	48 8b 44 24 20       	mov    0x20(%rsp),%rax
    6ff6:	48 39 ef             	cmp    %rbp,%rdi
    6ff9:	0f 84 39 01 00 00    	je     7138 <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0x248>
    6fff:	c4 c1 f9 6e cf       	vmovq  %r15,%xmm1
    7004:	c4 e3 f1 22 c0 01    	vpinsrq $0x1,%rax,%xmm1,%xmm0
    700a:	48 8b 53 10          	mov    0x10(%rbx),%rdx
    700e:	4c 89 33             	mov    %r14,(%rbx)
    7011:	c5 fa 7f 43 08       	vmovdqu %xmm0,0x8(%rbx)
    7016:	48 85 ff             	test   %rdi,%rdi
    7019:	0f 84 2c 01 00 00    	je     714b <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0x25b>
    701f:	48 89 7c 24 10       	mov    %rdi,0x10(%rsp)
    7024:	48 89 54 24 20       	mov    %rdx,0x20(%rsp)
    7029:	48 c7 44 24 18 00 00 	movq   $0x0,0x18(%rsp)
    7030:	00 00 
    7032:	c6 07 00             	movb   $0x0,(%rdi)
    7035:	48 8b 7c 24 10       	mov    0x10(%rsp),%rdi
    703a:	48 3b 7c 24 08       	cmp    0x8(%rsp),%rdi
    703f:	74 0e                	je     704f <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0x15f>
    7041:	48 8b 44 24 20       	mov    0x20(%rsp),%rax
    7046:	48 8d 70 01          	lea    0x1(%rax),%rsi
    704a:	e8 d1 c2 ff ff       	call   3320 <_ZdlPvm@plt>
    704f:	48 8b 44 24 38       	mov    0x38(%rsp),%rax
    7054:	64 48 2b 04 25 28 00 	sub    %fs:0x28,%rax
    705b:	00 00 
    705d:	0f 85 49 01 00 00    	jne    71ac <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0x2bc>
    7063:	48 83 c4 48          	add    $0x48,%rsp
    7067:	5b                   	pop    %rbx
    7068:	5d                   	pop    %rbp
    7069:	41 5c                	pop    %r12
    706b:	41 5d                	pop    %r13
    706d:	41 5e                	pop    %r14
    706f:	41 5f                	pop    %r15
    7071:	c3                   	ret    
    7072:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)
    7078:	0f 89 65 ff ff ff    	jns    6fe3 <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0xf3>
    707e:	48 8d 5d e0          	lea    -0x20(%rbp),%rbx
    7082:	48 8b 7d f0          	mov    -0x10(%rbp),%rdi
    7086:	4c 39 eb             	cmp    %r13,%rbx
    7089:	0f 85 e4 fe ff ff    	jne    6f73 <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0x83>
    708f:	4d 85 e4             	test   %r12,%r12
    7092:	74 1d                	je     70b1 <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0x1c1>
    7094:	49 83 fc 01          	cmp    $0x1,%r12
    7098:	0f 84 d2 00 00 00    	je     7170 <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0x280>
    709e:	4c 89 e2             	mov    %r12,%rdx
    70a1:	4c 89 ee             	mov    %r13,%rsi
    70a4:	e8 97 c0 ff ff       	call   3140 <memcpy@plt>
    70a9:	4d 8b 65 f8          	mov    -0x8(%r13),%r12
    70ad:	49 8b 7d 10          	mov    0x10(%r13),%rdi
    70b1:	4d 89 65 18          	mov    %r12,0x18(%r13)
    70b5:	42 c6 04 27 00       	movb   $0x0,(%rdi,%r12,1)
    70ba:	49 8b 7d f0          	mov    -0x10(%r13),%rdi
    70be:	e9 dc fe ff ff       	jmp    6f9f <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0xaf>
    70c3:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)
    70c8:	48 8b 03             	mov    (%rbx),%rax
    70cb:	4c 89 6b 10          	mov    %r13,0x10(%rbx)
    70cf:	4c 89 63 18          	mov    %r12,0x18(%rbx)
    70d3:	48 89 43 20          	mov    %rax,0x20(%rbx)
    70d7:	48 89 5b f0          	mov    %rbx,-0x10(%rbx)
    70db:	48 89 df             	mov    %rbx,%rdi
    70de:	e9 bc fe ff ff       	jmp    6f9f <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0xaf>
    70e3:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)
    70e8:	4c 8b 74 24 10       	mov    0x10(%rsp),%r14
    70ed:	48 8b 3b             	mov    (%rbx),%rdi
    70f0:	4c 3b 74 24 08       	cmp    0x8(%rsp),%r14
    70f5:	0f 85 f6 fe ff ff    	jne    6ff1 <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0x101>
    70fb:	4d 85 ff             	test   %r15,%r15
    70fe:	74 1f                	je     711f <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0x22f>
    7100:	49 83 ff 01          	cmp    $0x1,%r15
    7104:	0f 84 8e 00 00 00    	je     7198 <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0x2a8>
    710a:	48 8b 74 24 08       	mov    0x8(%rsp),%rsi
    710f:	4c 89 fa             	mov    %r15,%rdx
    7112:	e8 29 c0 ff ff       	call   3140 <memcpy@plt>
    7117:	4c 8b 7c 24 18       	mov    0x18(%rsp),%r15
    711c:	48 8b 3b             	mov    (%rbx),%rdi
    711f:	4c 89 7b 08          	mov    %r15,0x8(%rbx)
    7123:	42 c6 04 3f 00       	movb   $0x0,(%rdi,%r15,1)
    7128:	48 8b 7c 24 10       	mov    0x10(%rsp),%rdi
    712d:	e9 f7 fe ff ff       	jmp    7029 <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0x139>
    7132:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)
    7138:	c4 c1 f9 6e d7       	vmovq  %r15,%xmm2
    713d:	4c 89 33             	mov    %r14,(%rbx)
    7140:	c4 e3 e9 22 c0 01    	vpinsrq $0x1,%rax,%xmm2,%xmm0
    7146:	c5 fa 7f 43 08       	vmovdqu %xmm0,0x8(%rbx)
    714b:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
    7150:	48 89 44 24 10       	mov    %rax,0x10(%rsp)
    7155:	48 8d 44 24 20       	lea    0x20(%rsp),%rax
    715a:	48 89 44 24 08       	mov    %rax,0x8(%rsp)
    715f:	48 89 c7             	mov    %rax,%rdi
    7162:	e9 c2 fe ff ff       	jmp    7029 <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0x139>
    7167:	66 0f 1f 84 00 00 00 	nopw   0x0(%rax,%rax,1)
    716e:	00 00 
    7170:	41 0f b6 45 00       	movzbl 0x0(%r13),%eax
    7175:	88 07                	mov    %al,(%rdi)
    7177:	4d 8b 65 f8          	mov    -0x8(%r13),%r12
    717b:	49 8b 7d 10          	mov    0x10(%r13),%rdi
    717f:	e9 2d ff ff ff       	jmp    70b1 <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0x1c1>
    7184:	0f 1f 40 00          	nopl   0x0(%rax)
    7188:	c5 fa 6f 5f 10       	vmovdqu 0x10(%rdi),%xmm3
    718d:	c5 f9 7f 5c 24 20    	vmovdqa %xmm3,0x20(%rsp)
    7193:	e9 a3 fd ff ff       	jmp    6f3b <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0x4b>
    7198:	0f b6 44 24 20       	movzbl 0x20(%rsp),%eax
    719d:	88 07                	mov    %al,(%rdi)
    719f:	4c 8b 7c 24 18       	mov    0x18(%rsp),%r15
    71a4:	48 8b 3b             	mov    (%rbx),%rdi
    71a7:	e9 73 ff ff ff       	jmp    711f <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0+0x22f>
    71ac:	e8 9f bf ff ff       	call   3150 <__stack_chk_fail@plt>
    71b1:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
    71b8:	00 00 00 
    71bb:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)

00000000000071c0 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0>:
    71c0:	41 57                	push   %r15
    71c2:	41 56                	push   %r14
    71c4:	41 55                	push   %r13
    71c6:	41 54                	push   %r12
    71c8:	55                   	push   %rbp
    71c9:	53                   	push   %rbx
    71ca:	48 83 ec 58          	sub    $0x58,%rsp
    71ce:	48 89 74 24 10       	mov    %rsi,0x10(%rsp)
    71d3:	64 48 8b 04 25 28 00 	mov    %fs:0x28,%rax
    71da:	00 00 
    71dc:	48 89 44 24 48       	mov    %rax,0x48(%rsp)
    71e1:	31 c0                	xor    %eax,%eax
    71e3:	48 39 f7             	cmp    %rsi,%rdi
    71e6:	0f 84 fc 01 00 00    	je     73e8 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x228>
    71ec:	48 8d 47 20          	lea    0x20(%rdi),%rax
    71f0:	49 89 fd             	mov    %rdi,%r13
    71f3:	48 39 c6             	cmp    %rax,%rsi
    71f6:	0f 84 ec 01 00 00    	je     73e8 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x228>
    71fc:	48 8d 44 24 30       	lea    0x30(%rsp),%rax
    7201:	48 89 44 24 08       	mov    %rax,0x8(%rsp)
    7206:	4c 8d 7f 30          	lea    0x30(%rdi),%r15
    720a:	eb 25                	jmp    7231 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x71>
    720c:	0f 1f 40 00          	nopl   0x0(%rax)
    7210:	0f 82 fa 01 00 00    	jb     7410 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x250>
    7216:	48 89 ef             	mov    %rbp,%rdi
    7219:	e8 d2 fc ff ff       	call   6ef0 <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0>
    721e:	4d 8d 77 10          	lea    0x10(%r15),%r14
    7222:	49 83 c7 20          	add    $0x20,%r15
    7226:	4c 39 74 24 10       	cmp    %r14,0x10(%rsp)
    722b:	0f 84 b7 01 00 00    	je     73e8 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x228>
    7231:	49 8b 5f f8          	mov    -0x8(%r15),%rbx
    7235:	49 8d 6f f0          	lea    -0x10(%r15),%rbp
    7239:	49 3b 5d 08          	cmp    0x8(%r13),%rbx
    723d:	75 d1                	jne    7210 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x50>
    723f:	48 85 db             	test   %rbx,%rbx
    7242:	74 d2                	je     7216 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x56>
    7244:	49 8b 7f f0          	mov    -0x10(%r15),%rdi
    7248:	49 8b 75 00          	mov    0x0(%r13),%rsi
    724c:	48 89 da             	mov    %rbx,%rdx
    724f:	48 89 7c 24 18       	mov    %rdi,0x18(%rsp)
    7254:	e8 17 c0 ff ff       	call   3270 <memcmp@plt>
    7259:	85 c0                	test   %eax,%eax
    725b:	79 b9                	jns    7216 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x56>
    725d:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
    7262:	48 8b 7c 24 18       	mov    0x18(%rsp),%rdi
    7267:	48 89 44 24 20       	mov    %rax,0x20(%rsp)
    726c:	4c 39 ff             	cmp    %r15,%rdi
    726f:	0f 84 b2 01 00 00    	je     7427 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x267>
    7275:	49 8b 07             	mov    (%r15),%rax
    7278:	4c 29 ed             	sub    %r13,%rbp
    727b:	49 89 ec             	mov    %rbp,%r12
    727e:	48 89 7c 24 20       	mov    %rdi,0x20(%rsp)
    7283:	48 89 44 24 30       	mov    %rax,0x30(%rsp)
    7288:	48 89 5c 24 28       	mov    %rbx,0x28(%rsp)
    728d:	4d 89 7f f0          	mov    %r15,-0x10(%r15)
    7291:	49 c7 47 f8 00 00 00 	movq   $0x0,-0x8(%r15)
    7298:	00 
    7299:	41 c6 07 00          	movb   $0x0,(%r15)
    729d:	4d 8d 77 10          	lea    0x10(%r15),%r14
    72a1:	49 c1 fc 05          	sar    $0x5,%r12
    72a5:	48 85 ed             	test   %rbp,%rbp
    72a8:	0f 8e ac 00 00 00    	jle    735a <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x19a>
    72ae:	49 8d 5f e0          	lea    -0x20(%r15),%rbx
    72b2:	eb 44                	jmp    72f8 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x138>
    72b4:	0f 1f 40 00          	nopl   0x0(%rax)
    72b8:	48 8d 43 20          	lea    0x20(%rbx),%rax
    72bc:	48 39 c7             	cmp    %rax,%rdi
    72bf:	74 77                	je     7338 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x178>
    72c1:	48 89 53 18          	mov    %rdx,0x18(%rbx)
    72c5:	48 8b 13             	mov    (%rbx),%rdx
    72c8:	48 8b 43 20          	mov    0x20(%rbx),%rax
    72cc:	48 89 6b 10          	mov    %rbp,0x10(%rbx)
    72d0:	48 89 53 20          	mov    %rdx,0x20(%rbx)
    72d4:	48 85 ff             	test   %rdi,%rdi
    72d7:	74 6e                	je     7347 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x187>
    72d9:	48 89 7b f0          	mov    %rdi,-0x10(%rbx)
    72dd:	48 89 03             	mov    %rax,(%rbx)
    72e0:	48 8b 43 f0          	mov    -0x10(%rbx),%rax
    72e4:	48 c7 43 f8 00 00 00 	movq   $0x0,-0x8(%rbx)
    72eb:	00 
    72ec:	c6 00 00             	movb   $0x0,(%rax)
    72ef:	48 83 eb 20          	sub    $0x20,%rbx
    72f3:	49 ff cc             	dec    %r12
    72f6:	74 58                	je     7350 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x190>
    72f8:	48 8b 6b f0          	mov    -0x10(%rbx),%rbp
    72fc:	48 8b 7b 10          	mov    0x10(%rbx),%rdi
    7300:	48 8b 53 f8          	mov    -0x8(%rbx),%rdx
    7304:	48 39 eb             	cmp    %rbp,%rbx
    7307:	75 af                	jne    72b8 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0xf8>
    7309:	48 85 d2             	test   %rdx,%rdx
    730c:	74 12                	je     7320 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x160>
    730e:	48 83 fa 01          	cmp    $0x1,%rdx
    7312:	0f 84 88 01 00 00    	je     74a0 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x2e0>
    7318:	48 89 de             	mov    %rbx,%rsi
    731b:	e8 20 be ff ff       	call   3140 <memcpy@plt>
    7320:	48 8b 45 f8          	mov    -0x8(%rbp),%rax
    7324:	48 8b 55 10          	mov    0x10(%rbp),%rdx
    7328:	48 89 45 18          	mov    %rax,0x18(%rbp)
    732c:	c6 04 02 00          	movb   $0x0,(%rdx,%rax,1)
    7330:	eb ae                	jmp    72e0 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x120>
    7332:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)
    7338:	48 8b 03             	mov    (%rbx),%rax
    733b:	48 89 6b 10          	mov    %rbp,0x10(%rbx)
    733f:	48 89 53 18          	mov    %rdx,0x18(%rbx)
    7343:	48 89 43 20          	mov    %rax,0x20(%rbx)
    7347:	48 89 5b f0          	mov    %rbx,-0x10(%rbx)
    734b:	eb 93                	jmp    72e0 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x120>
    734d:	0f 1f 00             	nopl   (%rax)
    7350:	48 8b 7c 24 20       	mov    0x20(%rsp),%rdi
    7355:	48 8b 5c 24 28       	mov    0x28(%rsp),%rbx
    735a:	4d 8b 4d 00          	mov    0x0(%r13),%r9
    735e:	48 3b 7c 24 08       	cmp    0x8(%rsp),%rdi
    7363:	0f 84 ff 00 00 00    	je     7468 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x2a8>
    7369:	49 8d 55 10          	lea    0x10(%r13),%rdx
    736d:	48 8b 44 24 30       	mov    0x30(%rsp),%rax
    7372:	49 39 d1             	cmp    %rdx,%r9
    7375:	0f 84 35 01 00 00    	je     74b0 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x2f0>
    737b:	c4 e1 f9 6e cb       	vmovq  %rbx,%xmm1
    7380:	c4 e3 f1 22 c0 01    	vpinsrq $0x1,%rax,%xmm1,%xmm0
    7386:	49 8b 55 10          	mov    0x10(%r13),%rdx
    738a:	49 89 7d 00          	mov    %rdi,0x0(%r13)
    738e:	c4 c1 7a 7f 45 08    	vmovdqu %xmm0,0x8(%r13)
    7394:	4d 85 c9             	test   %r9,%r9
    7397:	0f 84 28 01 00 00    	je     74c5 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x305>
    739d:	4c 89 4c 24 20       	mov    %r9,0x20(%rsp)
    73a2:	48 89 54 24 30       	mov    %rdx,0x30(%rsp)
    73a7:	48 c7 44 24 28 00 00 	movq   $0x0,0x28(%rsp)
    73ae:	00 00 
    73b0:	41 c6 01 00          	movb   $0x0,(%r9)
    73b4:	48 8b 7c 24 20       	mov    0x20(%rsp),%rdi
    73b9:	48 3b 7c 24 08       	cmp    0x8(%rsp),%rdi
    73be:	0f 84 5e fe ff ff    	je     7222 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x62>
    73c4:	48 8b 44 24 30       	mov    0x30(%rsp),%rax
    73c9:	49 83 c7 20          	add    $0x20,%r15
    73cd:	48 8d 70 01          	lea    0x1(%rax),%rsi
    73d1:	e8 4a bf ff ff       	call   3320 <_ZdlPvm@plt>
    73d6:	4c 39 74 24 10       	cmp    %r14,0x10(%rsp)
    73db:	0f 85 50 fe ff ff    	jne    7231 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x71>
    73e1:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
    73e8:	48 8b 44 24 48       	mov    0x48(%rsp),%rax
    73ed:	64 48 2b 04 25 28 00 	sub    %fs:0x28,%rax
    73f4:	00 00 
    73f6:	0f 85 ff 00 00 00    	jne    74fb <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x33b>
    73fc:	48 83 c4 58          	add    $0x58,%rsp
    7400:	5b                   	pop    %rbx
    7401:	5d                   	pop    %rbp
    7402:	41 5c                	pop    %r12
    7404:	41 5d                	pop    %r13
    7406:	41 5e                	pop    %r14
    7408:	41 5f                	pop    %r15
    740a:	c3                   	ret    
    740b:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)
    7410:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
    7415:	49 8b 7f f0          	mov    -0x10(%r15),%rdi
    7419:	48 89 44 24 20       	mov    %rax,0x20(%rsp)
    741e:	4c 39 ff             	cmp    %r15,%rdi
    7421:	0f 85 4e fe ff ff    	jne    7275 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0xb5>
    7427:	c4 c1 7a 6f 1f       	vmovdqu (%r15),%xmm3
    742c:	4c 29 ed             	sub    %r13,%rbp
    742f:	49 89 ec             	mov    %rbp,%r12
    7432:	48 89 5c 24 28       	mov    %rbx,0x28(%rsp)
    7437:	4d 89 7f f0          	mov    %r15,-0x10(%r15)
    743b:	49 c7 47 f8 00 00 00 	movq   $0x0,-0x8(%r15)
    7442:	00 
    7443:	41 c6 07 00          	movb   $0x0,(%r15)
    7447:	c5 f9 7f 5c 24 30    	vmovdqa %xmm3,0x30(%rsp)
    744d:	4d 8d 77 10          	lea    0x10(%r15),%r14
    7451:	49 c1 fc 05          	sar    $0x5,%r12
    7455:	48 85 ed             	test   %rbp,%rbp
    7458:	0f 8f 50 fe ff ff    	jg     72ae <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0xee>
    745e:	4d 8b 4d 00          	mov    0x0(%r13),%r9
    7462:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)
    7468:	48 85 db             	test   %rbx,%rbx
    746b:	74 1f                	je     748c <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x2cc>
    746d:	48 83 fb 01          	cmp    $0x1,%rbx
    7471:	74 64                	je     74d7 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x317>
    7473:	48 8b 74 24 08       	mov    0x8(%rsp),%rsi
    7478:	48 89 da             	mov    %rbx,%rdx
    747b:	4c 89 cf             	mov    %r9,%rdi
    747e:	e8 bd bc ff ff       	call   3140 <memcpy@plt>
    7483:	48 8b 5c 24 28       	mov    0x28(%rsp),%rbx
    7488:	4d 8b 4d 00          	mov    0x0(%r13),%r9
    748c:	49 89 5d 08          	mov    %rbx,0x8(%r13)
    7490:	41 c6 04 19 00       	movb   $0x0,(%r9,%rbx,1)
    7495:	4c 8b 4c 24 20       	mov    0x20(%rsp),%r9
    749a:	e9 08 ff ff ff       	jmp    73a7 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x1e7>
    749f:	90                   	nop
    74a0:	0f b6 03             	movzbl (%rbx),%eax
    74a3:	88 07                	mov    %al,(%rdi)
    74a5:	e9 76 fe ff ff       	jmp    7320 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x160>
    74aa:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)
    74b0:	c4 e1 f9 6e d3       	vmovq  %rbx,%xmm2
    74b5:	49 89 7d 00          	mov    %rdi,0x0(%r13)
    74b9:	c4 e3 e9 22 c0 01    	vpinsrq $0x1,%rax,%xmm2,%xmm0
    74bf:	c4 c1 7a 7f 45 08    	vmovdqu %xmm0,0x8(%r13)
    74c5:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
    74ca:	48 89 44 24 20       	mov    %rax,0x20(%rsp)
    74cf:	49 89 c1             	mov    %rax,%r9
    74d2:	e9 d0 fe ff ff       	jmp    73a7 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x1e7>
    74d7:	0f b6 44 24 30       	movzbl 0x30(%rsp),%eax
    74dc:	41 88 01             	mov    %al,(%r9)
    74df:	48 8b 5c 24 28       	mov    0x28(%rsp),%rbx
    74e4:	4d 8b 4d 00          	mov    0x0(%r13),%r9
    74e8:	49 89 5d 08          	mov    %rbx,0x8(%r13)
    74ec:	41 c6 04 19 00       	movb   $0x0,(%r9,%rbx,1)
    74f1:	4c 8b 4c 24 20       	mov    0x20(%rsp),%r9
    74f6:	e9 ac fe ff ff       	jmp    73a7 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0+0x1e7>
    74fb:	e8 50 bc ff ff       	call   3150 <__stack_chk_fail@plt>

0000000000007500 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE>:
    7500:	31 c0                	xor    %eax,%eax
    7502:	48 85 ff             	test   %rdi,%rdi
    7505:	0f 84 b5 02 00 00    	je     77c0 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x2c0>
    750b:	80 3e 2d             	cmpb   $0x2d,(%rsi)
    750e:	74 08                	je     7518 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x18>
    7510:	c3                   	ret    
    7511:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
    7518:	48 89 f9             	mov    %rdi,%rcx
    751b:	48 ff c9             	dec    %rcx
    751e:	74 f0                	je     7510 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x10>
    7520:	48 83 ec 08          	sub    $0x8,%rsp
    7524:	0f b6 46 01          	movzbl 0x1(%rsi),%eax
    7528:	3c 30                	cmp    $0x30,%al
    752a:	0f 84 f0 01 00 00    	je     7720 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x220>
    7530:	0f 8f 0a 01 00 00    	jg     7640 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x140>
    7536:	3c 2e                	cmp    $0x2e,%al
    7538:	0f 85 f2 00 00 00    	jne    7630 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x130>
    753e:	48 83 c6 02          	add    $0x2,%rsi
    7542:	48 83 ef 02          	sub    $0x2,%rdi
    7546:	0f 84 e4 00 00 00    	je     7630 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x130>
    754c:	0f b6 06             	movzbl (%rsi),%eax
    754f:	83 e8 30             	sub    $0x30,%eax
    7552:	83 f8 09             	cmp    $0x9,%eax
    7555:	0f 87 d5 00 00 00    	ja     7630 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x130>
    755b:	48 89 f8             	mov    %rdi,%rax
    755e:	48 c1 f8 02          	sar    $0x2,%rax
    7562:	48 85 c0             	test   %rax,%rax
    7565:	0f 8e 24 04 00 00    	jle    798f <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x48f>
    756b:	48 8d 0c 86          	lea    (%rsi,%rax,4),%rcx
    756f:	48 89 f0             	mov    %rsi,%rax
    7572:	0f be 10             	movsbl (%rax),%edx
    7575:	83 ea 30             	sub    $0x30,%edx
    7578:	83 fa 09             	cmp    $0x9,%edx
    757b:	77 13                	ja     7590 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x90>
    757d:	0f be 50 01          	movsbl 0x1(%rax),%edx
    7581:	83 ea 30             	sub    $0x30,%edx
    7584:	83 fa 09             	cmp    $0x9,%edx
    7587:	0f 86 34 02 00 00    	jbe    77c1 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x2c1>
    758d:	48 ff c0             	inc    %rax
    7590:	48 89 c2             	mov    %rax,%rdx
    7593:	48 29 f2             	sub    %rsi,%rdx
    7596:	48 39 fa             	cmp    %rdi,%rdx
    7599:	0f 87 ae 04 00 00    	ja     7a4d <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x54d>
    759f:	48 29 d7             	sub    %rdx,%rdi
    75a2:	0f 84 6d 01 00 00    	je     7715 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x215>
    75a8:	0f b6 10             	movzbl (%rax),%edx
    75ab:	83 e2 df             	and    $0xffffffdf,%edx
    75ae:	80 fa 45             	cmp    $0x45,%dl
    75b1:	75 7d                	jne    7630 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x130>
    75b3:	48 ff c0             	inc    %rax
    75b6:	48 8d 4f ff          	lea    -0x1(%rdi),%rcx
    75ba:	48 85 c9             	test   %rcx,%rcx
    75bd:	74 71                	je     7630 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x130>
    75bf:	0f b6 10             	movzbl (%rax),%edx
    75c2:	83 ea 2b             	sub    $0x2b,%edx
    75c5:	81 e2 fd 00 00 00    	and    $0xfd,%edx
    75cb:	74 51                	je     761e <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x11e>
    75cd:	0f b6 10             	movzbl (%rax),%edx
    75d0:	83 ea 30             	sub    $0x30,%edx
    75d3:	83 fa 09             	cmp    $0x9,%edx
    75d6:	77 58                	ja     7630 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x130>
    75d8:	48 89 ca             	mov    %rcx,%rdx
    75db:	48 c1 fa 02          	sar    $0x2,%rdx
    75df:	48 85 d2             	test   %rdx,%rdx
    75e2:	0f 8e 03 04 00 00    	jle    79eb <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x4eb>
    75e8:	48 8d 3c 90          	lea    (%rax,%rdx,4),%rdi
    75ec:	48 89 c2             	mov    %rax,%rdx
    75ef:	0f be 32             	movsbl (%rdx),%esi
    75f2:	83 ee 30             	sub    $0x30,%esi
    75f5:	83 fe 09             	cmp    $0x9,%esi
    75f8:	77 13                	ja     760d <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x10d>
    75fa:	0f be 72 01          	movsbl 0x1(%rdx),%esi
    75fe:	83 ee 30             	sub    $0x30,%esi
    7601:	83 fe 09             	cmp    $0x9,%esi
    7604:	0f 86 5e 02 00 00    	jbe    7868 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x368>
    760a:	48 ff c2             	inc    %rdx
    760d:	48 29 c2             	sub    %rax,%rdx
    7610:	48 39 d1             	cmp    %rdx,%rcx
    7613:	0f 82 61 04 00 00    	jb     7a7a <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x57a>
    7619:	0f 95 c0             	setne  %al
    761c:	eb 17                	jmp    7635 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x135>
    761e:	48 ff c0             	inc    %rax
    7621:	48 ff c9             	dec    %rcx
    7624:	75 a7                	jne    75cd <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0xcd>
    7626:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
    762d:	00 00 00 
    7630:	b8 01 00 00 00       	mov    $0x1,%eax
    7635:	48 83 c4 08          	add    $0x8,%rsp
    7639:	c3                   	ret    
    763a:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)
    7640:	83 e8 31             	sub    $0x31,%eax
    7643:	3c 08                	cmp    $0x8,%al
    7645:	77 e9                	ja     7630 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x130>
    7647:	48 89 c8             	mov    %rcx,%rax
    764a:	48 c1 f8 02          	sar    $0x2,%rax
    764e:	4c 8d 46 01          	lea    0x1(%rsi),%r8
    7652:	48 85 c0             	test   %rax,%rax
    7655:	7e 7a                	jle    76d1 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x1d1>
    7657:	4c 8d 4c 86 01       	lea    0x1(%rsi,%rax,4),%r9
    765c:	4c 89 c2             	mov    %r8,%rdx
    765f:	0f be 02             	movsbl (%rdx),%eax
    7662:	83 e8 30             	sub    $0x30,%eax
    7665:	83 f8 09             	cmp    $0x9,%eax
    7668:	77 13                	ja     767d <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x17d>
    766a:	0f be 42 01          	movsbl 0x1(%rdx),%eax
    766e:	83 e8 30             	sub    $0x30,%eax
    7671:	83 f8 09             	cmp    $0x9,%eax
    7674:	0f 86 2e 01 00 00    	jbe    77a8 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x2a8>
    767a:	48 ff c2             	inc    %rdx
    767d:	4c 29 c2             	sub    %r8,%rdx
    7680:	48 39 d1             	cmp    %rdx,%rcx
    7683:	0f 82 dc 03 00 00    	jb     7a65 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x565>
    7689:	48 29 d1             	sub    %rdx,%rcx
    768c:	4c 01 c2             	add    %r8,%rdx
    768f:	48 83 f9 ff          	cmp    $0xffffffffffffffff,%rcx
    7693:	75 7b                	jne    7710 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x210>
    7695:	0f b6 02             	movzbl (%rdx),%eax
    7698:	89 c1                	mov    %eax,%ecx
    769a:	83 e1 df             	and    $0xffffffdf,%ecx
    769d:	80 f9 45             	cmp    $0x45,%cl
    76a0:	0f 84 7f 01 00 00    	je     7825 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x325>
    76a6:	48 8d 72 01          	lea    0x1(%rdx),%rsi
    76aa:	48 c7 c7 fe ff ff ff 	mov    $0xfffffffffffffffe,%rdi
    76b1:	3c 2e                	cmp    $0x2e,%al
    76b3:	0f 85 77 ff ff ff    	jne    7630 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x130>
    76b9:	0f b6 42 01          	movzbl 0x1(%rdx),%eax
    76bd:	83 e8 30             	sub    $0x30,%eax
    76c0:	83 f8 09             	cmp    $0x9,%eax
    76c3:	0f 86 83 fe ff ff    	jbe    754c <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x4c>
    76c9:	48 89 f0             	mov    %rsi,%rax
    76cc:	e9 d7 fe ff ff       	jmp    75a8 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0xa8>
    76d1:	4d 89 c1             	mov    %r8,%r9
    76d4:	48 8d 14 3e          	lea    (%rsi,%rdi,1),%rdx
    76d8:	48 89 d0             	mov    %rdx,%rax
    76db:	4c 29 c8             	sub    %r9,%rax
    76de:	48 83 f8 02          	cmp    $0x2,%rax
    76e2:	0f 84 be 02 00 00    	je     79a6 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x4a6>
    76e8:	48 83 f8 03          	cmp    $0x3,%rax
    76ec:	0f 84 a5 02 00 00    	je     7997 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x497>
    76f2:	48 ff c8             	dec    %rax
    76f5:	75 86                	jne    767d <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x17d>
    76f7:	41 0f be 01          	movsbl (%r9),%eax
    76fb:	83 e8 30             	sub    $0x30,%eax
    76fe:	83 f8 0a             	cmp    $0xa,%eax
    7701:	49 0f 43 d1          	cmovae %r9,%rdx
    7705:	e9 73 ff ff ff       	jmp    767d <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x17d>
    770a:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)
    7710:	48 85 c9             	test   %rcx,%rcx
    7713:	75 66                	jne    777b <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x27b>
    7715:	31 c0                	xor    %eax,%eax
    7717:	e9 19 ff ff ff       	jmp    7635 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x135>
    771c:	0f 1f 40 00          	nopl   0x0(%rax)
    7720:	48 83 ef 02          	sub    $0x2,%rdi
    7724:	74 ef                	je     7715 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x215>
    7726:	48 89 f8             	mov    %rdi,%rax
    7729:	48 c1 f8 02          	sar    $0x2,%rax
    772d:	48 8d 4e 02          	lea    0x2(%rsi),%rcx
    7731:	48 85 c0             	test   %rax,%rax
    7734:	0f 8e 80 02 00 00    	jle    79ba <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x4ba>
    773a:	48 8d 74 86 02       	lea    0x2(%rsi,%rax,4),%rsi
    773f:	48 89 ca             	mov    %rcx,%rdx
    7742:	0f be 02             	movsbl (%rdx),%eax
    7745:	83 e8 30             	sub    $0x30,%eax
    7748:	83 f8 09             	cmp    $0x9,%eax
    774b:	77 13                	ja     7760 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x260>
    774d:	0f be 42 01          	movsbl 0x1(%rdx),%eax
    7751:	83 e8 30             	sub    $0x30,%eax
    7754:	83 f8 09             	cmp    $0x9,%eax
    7757:	0f 86 b3 00 00 00    	jbe    7810 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x310>
    775d:	48 ff c2             	inc    %rdx
    7760:	49 89 d0             	mov    %rdx,%r8
    7763:	49 29 c8             	sub    %rcx,%r8
    7766:	48 89 f9             	mov    %rdi,%rcx
    7769:	4c 39 c7             	cmp    %r8,%rdi
    776c:	0f 82 1d 03 00 00    	jb     7a8f <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x58f>
    7772:	4c 29 c1             	sub    %r8,%rcx
    7775:	0f 84 b5 fe ff ff    	je     7630 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x130>
    777b:	0f b6 02             	movzbl (%rdx),%eax
    777e:	3c 2e                	cmp    $0x2e,%al
    7780:	74 71                	je     77f3 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x2f3>
    7782:	3c 2d                	cmp    $0x2d,%al
    7784:	0f 8e a6 fe ff ff    	jle    7630 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x130>
    778a:	83 e0 df             	and    $0xffffffdf,%eax
    778d:	3c 45                	cmp    $0x45,%al
    778f:	0f 85 9b fe ff ff    	jne    7630 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x130>
    7795:	48 ff c9             	dec    %rcx
    7798:	48 8d 42 01          	lea    0x1(%rdx),%rax
    779c:	e9 19 fe ff ff       	jmp    75ba <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0xba>
    77a1:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
    77a8:	0f be 42 02          	movsbl 0x2(%rdx),%eax
    77ac:	83 e8 30             	sub    $0x30,%eax
    77af:	83 f8 09             	cmp    $0x9,%eax
    77b2:	76 26                	jbe    77da <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x2da>
    77b4:	48 83 c2 02          	add    $0x2,%rdx
    77b8:	e9 c0 fe ff ff       	jmp    767d <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x17d>
    77bd:	0f 1f 00             	nopl   (%rax)
    77c0:	c3                   	ret    
    77c1:	0f be 50 02          	movsbl 0x2(%rax),%edx
    77c5:	83 ea 30             	sub    $0x30,%edx
    77c8:	83 fa 09             	cmp    $0x9,%edx
    77cb:	0f 86 82 00 00 00    	jbe    7853 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x353>
    77d1:	48 83 c0 02          	add    $0x2,%rax
    77d5:	e9 b6 fd ff ff       	jmp    7590 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x90>
    77da:	0f be 42 03          	movsbl 0x3(%rdx),%eax
    77de:	83 e8 30             	sub    $0x30,%eax
    77e1:	83 f8 09             	cmp    $0x9,%eax
    77e4:	0f 86 93 00 00 00    	jbe    787d <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x37d>
    77ea:	48 83 c2 03          	add    $0x3,%rdx
    77ee:	e9 8a fe ff ff       	jmp    767d <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x17d>
    77f3:	48 ff c9             	dec    %rcx
    77f6:	48 8d 72 01          	lea    0x1(%rdx),%rsi
    77fa:	48 89 cf             	mov    %rcx,%rdi
    77fd:	0f 85 b6 fe ff ff    	jne    76b9 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x1b9>
    7803:	31 c0                	xor    %eax,%eax
    7805:	e9 2b fe ff ff       	jmp    7635 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x135>
    780a:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)
    7810:	0f be 42 02          	movsbl 0x2(%rdx),%eax
    7814:	83 e8 30             	sub    $0x30,%eax
    7817:	83 f8 09             	cmp    $0x9,%eax
    781a:	76 74                	jbe    7890 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x390>
    781c:	48 83 c2 02          	add    $0x2,%rdx
    7820:	e9 3b ff ff ff       	jmp    7760 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x260>
    7825:	0f b6 4a 01          	movzbl 0x1(%rdx),%ecx
    7829:	48 8d 42 01          	lea    0x1(%rdx),%rax
    782d:	83 e9 2b             	sub    $0x2b,%ecx
    7830:	81 e1 fd 00 00 00    	and    $0xfd,%ecx
    7836:	48 c7 c1 fe ff ff ff 	mov    $0xfffffffffffffffe,%rcx
    783d:	0f 85 8a fd ff ff    	jne    75cd <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0xcd>
    7843:	48 8d 42 02          	lea    0x2(%rdx),%rax
    7847:	48 c7 c1 fd ff ff ff 	mov    $0xfffffffffffffffd,%rcx
    784e:	e9 7a fd ff ff       	jmp    75cd <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0xcd>
    7853:	0f be 50 03          	movsbl 0x3(%rax),%edx
    7857:	83 ea 30             	sub    $0x30,%edx
    785a:	83 fa 09             	cmp    $0x9,%edx
    785d:	76 46                	jbe    78a5 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x3a5>
    785f:	48 83 c0 03          	add    $0x3,%rax
    7863:	e9 28 fd ff ff       	jmp    7590 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x90>
    7868:	0f be 72 02          	movsbl 0x2(%rdx),%esi
    786c:	83 ee 30             	sub    $0x30,%esi
    786f:	83 fe 09             	cmp    $0x9,%esi
    7872:	76 77                	jbe    78eb <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x3eb>
    7874:	48 83 c2 02          	add    $0x2,%rdx
    7878:	e9 90 fd ff ff       	jmp    760d <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x10d>
    787d:	48 83 c2 04          	add    $0x4,%rdx
    7881:	49 39 d1             	cmp    %rdx,%r9
    7884:	0f 85 d5 fd ff ff    	jne    765f <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x15f>
    788a:	e9 45 fe ff ff       	jmp    76d4 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x1d4>
    788f:	90                   	nop
    7890:	0f be 42 03          	movsbl 0x3(%rdx),%eax
    7894:	83 e8 30             	sub    $0x30,%eax
    7897:	83 f8 09             	cmp    $0x9,%eax
    789a:	76 64                	jbe    7900 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x400>
    789c:	48 83 c2 03          	add    $0x3,%rdx
    78a0:	e9 bb fe ff ff       	jmp    7760 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x260>
    78a5:	48 83 c0 04          	add    $0x4,%rax
    78a9:	48 39 c1             	cmp    %rax,%rcx
    78ac:	0f 85 c0 fc ff ff    	jne    7572 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x72>
    78b2:	48 8d 04 3e          	lea    (%rsi,%rdi,1),%rax
    78b6:	48 89 c2             	mov    %rax,%rdx
    78b9:	48 29 ca             	sub    %rcx,%rdx
    78bc:	48 83 fa 02          	cmp    $0x2,%rdx
    78c0:	0f 84 12 01 00 00    	je     79d8 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x4d8>
    78c6:	48 83 fa 03          	cmp    $0x3,%rdx
    78ca:	0f 84 fa 00 00 00    	je     79ca <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x4ca>
    78d0:	48 ff ca             	dec    %rdx
    78d3:	0f 85 b7 fc ff ff    	jne    7590 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x90>
    78d9:	0f be 11             	movsbl (%rcx),%edx
    78dc:	83 ea 30             	sub    $0x30,%edx
    78df:	83 fa 0a             	cmp    $0xa,%edx
    78e2:	48 0f 43 c1          	cmovae %rcx,%rax
    78e6:	e9 a5 fc ff ff       	jmp    7590 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x90>
    78eb:	0f be 72 03          	movsbl 0x3(%rdx),%esi
    78ef:	83 ee 30             	sub    $0x30,%esi
    78f2:	83 fe 09             	cmp    $0x9,%esi
    78f5:	76 4f                	jbe    7946 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x446>
    78f7:	48 83 c2 03          	add    $0x3,%rdx
    78fb:	e9 0d fd ff ff       	jmp    760d <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x10d>
    7900:	48 83 c2 04          	add    $0x4,%rdx
    7904:	48 39 d6             	cmp    %rdx,%rsi
    7907:	0f 85 35 fe ff ff    	jne    7742 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x242>
    790d:	48 8d 14 39          	lea    (%rcx,%rdi,1),%rdx
    7911:	48 89 d0             	mov    %rdx,%rax
    7914:	48 29 f0             	sub    %rsi,%rax
    7917:	48 83 f8 02          	cmp    $0x2,%rax
    791b:	0f 84 e8 00 00 00    	je     7a09 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x509>
    7921:	48 83 f8 03          	cmp    $0x3,%rax
    7925:	0f 84 d0 00 00 00    	je     79fb <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x4fb>
    792b:	48 ff c8             	dec    %rax
    792e:	0f 85 2c fe ff ff    	jne    7760 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x260>
    7934:	0f be 06             	movsbl (%rsi),%eax
    7937:	83 e8 30             	sub    $0x30,%eax
    793a:	83 f8 0a             	cmp    $0xa,%eax
    793d:	48 0f 43 d6          	cmovae %rsi,%rdx
    7941:	e9 1a fe ff ff       	jmp    7760 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x260>
    7946:	48 83 c2 04          	add    $0x4,%rdx
    794a:	48 39 d7             	cmp    %rdx,%rdi
    794d:	0f 85 9c fc ff ff    	jne    75ef <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0xef>
    7953:	48 89 fe             	mov    %rdi,%rsi
    7956:	48 8d 14 08          	lea    (%rax,%rcx,1),%rdx
    795a:	48 89 d7             	mov    %rdx,%rdi
    795d:	48 29 f7             	sub    %rsi,%rdi
    7960:	48 83 ff 02          	cmp    $0x2,%rdi
    7964:	0f 84 c8 00 00 00    	je     7a32 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x532>
    796a:	48 83 ff 03          	cmp    $0x3,%rdi
    796e:	0f 84 b0 00 00 00    	je     7a24 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x524>
    7974:	48 ff cf             	dec    %rdi
    7977:	0f 85 90 fc ff ff    	jne    760d <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x10d>
    797d:	0f be 3e             	movsbl (%rsi),%edi
    7980:	83 ef 30             	sub    $0x30,%edi
    7983:	83 ff 0a             	cmp    $0xa,%edi
    7986:	48 0f 43 d6          	cmovae %rsi,%rdx
    798a:	e9 7e fc ff ff       	jmp    760d <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x10d>
    798f:	48 89 f1             	mov    %rsi,%rcx
    7992:	e9 1b ff ff ff       	jmp    78b2 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x3b2>
    7997:	41 0f be 01          	movsbl (%r9),%eax
    799b:	83 e8 30             	sub    $0x30,%eax
    799e:	83 f8 09             	cmp    $0x9,%eax
    79a1:	77 1f                	ja     79c2 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x4c2>
    79a3:	49 ff c1             	inc    %r9
    79a6:	41 0f be 01          	movsbl (%r9),%eax
    79aa:	83 e8 30             	sub    $0x30,%eax
    79ad:	83 f8 09             	cmp    $0x9,%eax
    79b0:	77 10                	ja     79c2 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x4c2>
    79b2:	49 ff c1             	inc    %r9
    79b5:	e9 3d fd ff ff       	jmp    76f7 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x1f7>
    79ba:	48 89 ce             	mov    %rcx,%rsi
    79bd:	e9 4b ff ff ff       	jmp    790d <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x40d>
    79c2:	4c 89 ca             	mov    %r9,%rdx
    79c5:	e9 b3 fc ff ff       	jmp    767d <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x17d>
    79ca:	0f be 11             	movsbl (%rcx),%edx
    79cd:	83 ea 30             	sub    $0x30,%edx
    79d0:	83 fa 09             	cmp    $0x9,%edx
    79d3:	77 1e                	ja     79f3 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x4f3>
    79d5:	48 ff c1             	inc    %rcx
    79d8:	0f be 11             	movsbl (%rcx),%edx
    79db:	83 ea 30             	sub    $0x30,%edx
    79de:	83 fa 09             	cmp    $0x9,%edx
    79e1:	77 10                	ja     79f3 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x4f3>
    79e3:	48 ff c1             	inc    %rcx
    79e6:	e9 ee fe ff ff       	jmp    78d9 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x3d9>
    79eb:	48 89 c6             	mov    %rax,%rsi
    79ee:	e9 63 ff ff ff       	jmp    7956 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x456>
    79f3:	48 89 c8             	mov    %rcx,%rax
    79f6:	e9 95 fb ff ff       	jmp    7590 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x90>
    79fb:	0f be 06             	movsbl (%rsi),%eax
    79fe:	83 e8 30             	sub    $0x30,%eax
    7a01:	83 f8 09             	cmp    $0x9,%eax
    7a04:	77 16                	ja     7a1c <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x51c>
    7a06:	48 ff c6             	inc    %rsi
    7a09:	0f be 06             	movsbl (%rsi),%eax
    7a0c:	83 e8 30             	sub    $0x30,%eax
    7a0f:	83 f8 09             	cmp    $0x9,%eax
    7a12:	77 08                	ja     7a1c <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x51c>
    7a14:	48 ff c6             	inc    %rsi
    7a17:	e9 18 ff ff ff       	jmp    7934 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x434>
    7a1c:	48 89 f2             	mov    %rsi,%rdx
    7a1f:	e9 3c fd ff ff       	jmp    7760 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x260>
    7a24:	0f be 3e             	movsbl (%rsi),%edi
    7a27:	83 ef 30             	sub    $0x30,%edi
    7a2a:	83 ff 09             	cmp    $0x9,%edi
    7a2d:	77 16                	ja     7a45 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x545>
    7a2f:	48 ff c6             	inc    %rsi
    7a32:	0f be 3e             	movsbl (%rsi),%edi
    7a35:	83 ef 30             	sub    $0x30,%edi
    7a38:	83 ff 09             	cmp    $0x9,%edi
    7a3b:	77 08                	ja     7a45 <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x545>
    7a3d:	48 ff c6             	inc    %rsi
    7a40:	e9 38 ff ff ff       	jmp    797d <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x47d>
    7a45:	48 89 f2             	mov    %rsi,%rdx
    7a48:	e9 c0 fb ff ff       	jmp    760d <_ZN8argparse8Argument11is_optionalESt17basic_string_viewIcSt11char_traitsIcEE+0x10d>
    7a4d:	48 89 f9             	mov    %rdi,%rcx
    7a50:	48 8d 35 e9 85 00 00 	lea    0x85e9(%rip),%rsi        # 10040 <_fini+0xe1f>
    7a57:	48 8d 3d 02 86 00 00 	lea    0x8602(%rip),%rdi        # 10060 <_fini+0xe3f>
    7a5e:	31 c0                	xor    %eax,%eax
    7a60:	e8 5b b8 ff ff       	call   32c0 <_ZSt24__throw_out_of_range_fmtPKcz@plt>
    7a65:	48 8d 35 d4 85 00 00 	lea    0x85d4(%rip),%rsi        # 10040 <_fini+0xe1f>
    7a6c:	48 8d 3d ed 85 00 00 	lea    0x85ed(%rip),%rdi        # 10060 <_fini+0xe3f>
    7a73:	31 c0                	xor    %eax,%eax
    7a75:	e8 46 b8 ff ff       	call   32c0 <_ZSt24__throw_out_of_range_fmtPKcz@plt>
    7a7a:	48 8d 35 bf 85 00 00 	lea    0x85bf(%rip),%rsi        # 10040 <_fini+0xe1f>
    7a81:	48 8d 3d d8 85 00 00 	lea    0x85d8(%rip),%rdi        # 10060 <_fini+0xe3f>
    7a88:	31 c0                	xor    %eax,%eax
    7a8a:	e8 31 b8 ff ff       	call   32c0 <_ZSt24__throw_out_of_range_fmtPKcz@plt>
    7a8f:	4c 89 c2             	mov    %r8,%rdx
    7a92:	48 8d 35 a7 85 00 00 	lea    0x85a7(%rip),%rsi        # 10040 <_fini+0xe1f>
    7a99:	48 8d 3d c0 85 00 00 	lea    0x85c0(%rip),%rdi        # 10060 <_fini+0xe3f>
    7aa0:	31 c0                	xor    %eax,%eax
    7aa2:	e8 19 b8 ff ff       	call   32c0 <_ZSt24__throw_out_of_range_fmtPKcz@plt>
    7aa7:	66 0f 1f 84 00 00 00 	nopw   0x0(%rax,%rax,1)
    7aae:	00 00 

0000000000007ab0 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIfLNSB_12chars_formatE3EEEE9_M_invokeERKSt9_Any_dataS8_>:
    7ab0:	41 55                	push   %r13
    7ab2:	41 54                	push   %r12
    7ab4:	49 89 fc             	mov    %rdi,%r12
    7ab7:	55                   	push   %rbp
    7ab8:	53                   	push   %rbx
    7ab9:	48 83 ec 18          	sub    $0x18,%rsp
    7abd:	4c 8b 6a 08          	mov    0x8(%rdx),%r13
    7ac1:	64 48 8b 04 25 28 00 	mov    %fs:0x28,%rax
    7ac8:	00 00 
    7aca:	48 89 44 24 08       	mov    %rax,0x8(%rsp)
    7acf:	31 c0                	xor    %eax,%eax
    7ad1:	48 8b 2a             	mov    (%rdx),%rbp
    7ad4:	49 83 fd 01          	cmp    $0x1,%r13
    7ad8:	76 46                	jbe    7b20 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIfLNSB_12chars_formatE3EEEE9_M_invokeERKSt9_Any_dataS8_+0x70>
    7ada:	66 81 7d 00 30 78    	cmpw   $0x7830,0x0(%rbp)
    7ae0:	74 08                	je     7aea <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIfLNSB_12chars_formatE3EEEE9_M_invokeERKSt9_Any_dataS8_+0x3a>
    7ae2:	66 81 7d 00 30 58    	cmpw   $0x5830,0x0(%rbp)
    7ae8:	75 36                	jne    7b20 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIfLNSB_12chars_formatE3EEEE9_M_invokeERKSt9_Any_dataS8_+0x70>
    7aea:	bf 10 00 00 00       	mov    $0x10,%edi
    7aef:	e8 dc b5 ff ff       	call   30d0 <__cxa_allocate_exception@plt>
    7af4:	48 89 c7             	mov    %rax,%rdi
    7af7:	48 8d 35 9a 85 00 00 	lea    0x859a(%rip),%rsi        # 10098 <_fini+0xe77>
    7afe:	48 89 c5             	mov    %rax,%rbp
    7b01:	e8 ea b5 ff ff       	call   30f0 <_ZNSt16invalid_argumentC1EPKc@plt>
    7b06:	48 8b 15 a3 c4 00 00 	mov    0xc4a3(%rip),%rdx        # 13fb0 <_ZNSt16invalid_argumentD1Ev@Base>
    7b0d:	48 8d 35 04 c0 00 00 	lea    0xc004(%rip),%rsi        # 13b18 <_ZTISt16invalid_argument@@Base>
    7b14:	48 89 ef             	mov    %rbp,%rdi
    7b17:	e8 04 b7 ff ff       	call   3220 <__cxa_throw@plt>
    7b1c:	0f 1f 40 00          	nopl   0x0(%rax)
    7b20:	0f b6 7d 00          	movzbl 0x0(%rbp),%edi
    7b24:	89 fb                	mov    %edi,%ebx
    7b26:	e8 45 b6 ff ff       	call   3170 <isspace@plt>
    7b2b:	85 c0                	test   %eax,%eax
    7b2d:	0f 85 e8 00 00 00    	jne    7c1b <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIfLNSB_12chars_formatE3EEEE9_M_invokeERKSt9_Any_dataS8_+0x16b>
    7b33:	80 fb 2b             	cmp    $0x2b,%bl
    7b36:	0f 84 df 00 00 00    	je     7c1b <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIfLNSB_12chars_formatE3EEEE9_M_invokeERKSt9_Any_dataS8_+0x16b>
    7b3c:	e8 4f b8 ff ff       	call   3390 <__errno_location@plt>
    7b41:	c7 00 00 00 00 00    	movl   $0x0,(%rax)
    7b47:	48 89 c3             	mov    %rax,%rbx
    7b4a:	48 89 e6             	mov    %rsp,%rsi
    7b4d:	48 89 ef             	mov    %rbp,%rdi
    7b50:	e8 7b b7 ff ff       	call   32d0 <strtof@plt>
    7b55:	8b 03                	mov    (%rbx),%eax
    7b57:	85 c0                	test   %eax,%eax
    7b59:	74 45                	je     7ba0 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIfLNSB_12chars_formatE3EEEE9_M_invokeERKSt9_Any_dataS8_+0xf0>
    7b5b:	83 f8 22             	cmp    $0x22,%eax
    7b5e:	74 74                	je     7bd4 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIfLNSB_12chars_formatE3EEEE9_M_invokeERKSt9_Any_dataS8_+0x124>
    7b60:	48 8d 05 f9 e6 ff ff 	lea    -0x1907(%rip),%rax        # 6260 <_ZNSt3any17_Manager_internalIfE9_S_manageENS_3_OpEPKS_PNS_4_ArgE>
    7b67:	49 c7 44 24 08 00 00 	movq   $0x0,0x8(%r12)
    7b6e:	00 00 
    7b70:	49 89 04 24          	mov    %rax,(%r12)
    7b74:	c4 c1 7a 11 44 24 08 	vmovss %xmm0,0x8(%r12)
    7b7b:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
    7b80:	64 48 2b 04 25 28 00 	sub    %fs:0x28,%rax
    7b87:	00 00 
    7b89:	75 3f                	jne    7bca <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIfLNSB_12chars_formatE3EEEE9_M_invokeERKSt9_Any_dataS8_+0x11a>
    7b8b:	48 83 c4 18          	add    $0x18,%rsp
    7b8f:	5b                   	pop    %rbx
    7b90:	5d                   	pop    %rbp
    7b91:	4c 89 e0             	mov    %r12,%rax
    7b94:	41 5c                	pop    %r12
    7b96:	41 5d                	pop    %r13
    7b98:	c3                   	ret    
    7b99:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
    7ba0:	4c 01 ed             	add    %r13,%rbp
    7ba3:	48 39 2c 24          	cmp    %rbp,(%rsp)
    7ba7:	74 b7                	je     7b60 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIfLNSB_12chars_formatE3EEEE9_M_invokeERKSt9_Any_dataS8_+0xb0>
    7ba9:	bf 10 00 00 00       	mov    $0x10,%edi
    7bae:	e8 1d b5 ff ff       	call   30d0 <__cxa_allocate_exception@plt>
    7bb3:	48 89 c7             	mov    %rax,%rdi
    7bb6:	48 8d 35 0b 85 00 00 	lea    0x850b(%rip),%rsi        # 100c8 <_fini+0xea7>
    7bbd:	48 89 c5             	mov    %rax,%rbp
    7bc0:	e8 2b b5 ff ff       	call   30f0 <_ZNSt16invalid_argumentC1EPKc@plt>
    7bc5:	e9 3c ff ff ff       	jmp    7b06 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIfLNSB_12chars_formatE3EEEE9_M_invokeERKSt9_Any_dataS8_+0x56>
    7bca:	e8 81 b5 ff ff       	call   3150 <__stack_chk_fail@plt>
    7bcf:	49 89 c4             	mov    %rax,%r12
    7bd2:	eb 34                	jmp    7c08 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIfLNSB_12chars_formatE3EEEE9_M_invokeERKSt9_Any_dataS8_+0x158>
    7bd4:	bf 10 00 00 00       	mov    $0x10,%edi
    7bd9:	e8 f2 b4 ff ff       	call   30d0 <__cxa_allocate_exception@plt>
    7bde:	48 89 c7             	mov    %rax,%rdi
    7be1:	48 8d 35 14 85 00 00 	lea    0x8514(%rip),%rsi        # 100fc <_fini+0xedb>
    7be8:	48 89 c5             	mov    %rax,%rbp
    7beb:	e8 70 b4 ff ff       	call   3060 <_ZNSt11range_errorC1EPKc@plt>
    7bf0:	48 8b 15 e1 c3 00 00 	mov    0xc3e1(%rip),%rdx        # 13fd8 <_ZNSt11range_errorD1Ev@Base>
    7bf7:	48 8d 35 c2 bc 00 00 	lea    0xbcc2(%rip),%rsi        # 138c0 <_ZTISt11range_error@@Base>
    7bfe:	48 89 ef             	mov    %rbp,%rdi
    7c01:	e8 1a b6 ff ff       	call   3220 <__cxa_throw@plt>
    7c06:	eb c7                	jmp    7bcf <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIfLNSB_12chars_formatE3EEEE9_M_invokeERKSt9_Any_dataS8_+0x11f>
    7c08:	48 89 ef             	mov    %rbp,%rdi
    7c0b:	c5 f8 77             	vzeroupper 
    7c0e:	e8 9d b7 ff ff       	call   33b0 <__cxa_free_exception@plt>
    7c13:	4c 89 e7             	mov    %r12,%rdi
    7c16:	e8 45 b7 ff ff       	call   3360 <_Unwind_Resume@plt>
    7c1b:	bf 10 00 00 00       	mov    $0x10,%edi
    7c20:	e8 ab b4 ff ff       	call   30d0 <__cxa_allocate_exception@plt>
    7c25:	48 89 c7             	mov    %rax,%rdi
    7c28:	48 8d 35 bb 84 00 00 	lea    0x84bb(%rip),%rsi        # 100ea <_fini+0xec9>
    7c2f:	48 89 c5             	mov    %rax,%rbp
    7c32:	e8 b9 b4 ff ff       	call   30f0 <_ZNSt16invalid_argumentC1EPKc@plt>
    7c37:	e9 ca fe ff ff       	jmp    7b06 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIfLNSB_12chars_formatE3EEEE9_M_invokeERKSt9_Any_dataS8_+0x56>
    7c3c:	eb 91                	jmp    7bcf <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIfLNSB_12chars_formatE3EEEE9_M_invokeERKSt9_Any_dataS8_+0x11f>
    7c3e:	eb 8f                	jmp    7bcf <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIfLNSB_12chars_formatE3EEEE9_M_invokeERKSt9_Any_dataS8_+0x11f>

0000000000007c40 <_ZN8argparse8Argument14implicit_valueESt3any>:
    7c40:	41 55                	push   %r13
    7c42:	4c 8d 6f 78          	lea    0x78(%rdi),%r13
    7c46:	41 54                	push   %r12
    7c48:	49 89 fc             	mov    %rdi,%r12
    7c4b:	55                   	push   %rbp
    7c4c:	48 83 ec 10          	sub    $0x10,%rsp
    7c50:	64 48 8b 04 25 28 00 	mov    %fs:0x28,%rax
    7c57:	00 00 
    7c59:	48 89 44 24 08       	mov    %rax,0x8(%rsp)
    7c5e:	31 c0                	xor    %eax,%eax
    7c60:	48 8b 06             	mov    (%rsi),%rax
    7c63:	48 85 c0             	test   %rax,%rax
    7c66:	74 78                	je     7ce0 <_ZN8argparse8Argument14implicit_valueESt3any+0xa0>
    7c68:	48 89 f5             	mov    %rsi,%rbp
    7c6b:	49 39 f5             	cmp    %rsi,%r13
    7c6e:	74 33                	je     7ca3 <_ZN8argparse8Argument14implicit_valueESt3any+0x63>
    7c70:	48 8b 4f 78          	mov    0x78(%rdi),%rcx
    7c74:	48 85 c9             	test   %rcx,%rcx
    7c77:	74 19                	je     7c92 <_ZN8argparse8Argument14implicit_valueESt3any+0x52>
    7c79:	31 d2                	xor    %edx,%edx
    7c7b:	4c 89 ee             	mov    %r13,%rsi
    7c7e:	bf 03 00 00 00       	mov    $0x3,%edi
    7c83:	ff d1                	call   *%rcx
    7c85:	49 c7 44 24 78 00 00 	movq   $0x0,0x78(%r12)
    7c8c:	00 00 
    7c8e:	48 8b 45 00          	mov    0x0(%rbp),%rax
    7c92:	4c 89 2c 24          	mov    %r13,(%rsp)
    7c96:	48 89 e2             	mov    %rsp,%rdx
    7c99:	48 89 ee             	mov    %rbp,%rsi
    7c9c:	bf 04 00 00 00       	mov    $0x4,%edi
    7ca1:	ff d0                	call   *%rax
    7ca3:	49 c7 84 24 c8 00 00 	movq   $0x0,0xc8(%r12)
    7caa:	00 00 00 00 00 
    7caf:	49 c7 84 24 d0 00 00 	movq   $0x0,0xd0(%r12)
    7cb6:	00 00 00 00 00 
    7cbb:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
    7cc0:	64 48 2b 04 25 28 00 	sub    %fs:0x28,%rax
    7cc7:	00 00 
    7cc9:	75 35                	jne    7d00 <_ZN8argparse8Argument14implicit_valueESt3any+0xc0>
    7ccb:	48 83 c4 10          	add    $0x10,%rsp
    7ccf:	5d                   	pop    %rbp
    7cd0:	4c 89 e0             	mov    %r12,%rax
    7cd3:	41 5c                	pop    %r12
    7cd5:	41 5d                	pop    %r13
    7cd7:	c3                   	ret    
    7cd8:	0f 1f 84 00 00 00 00 	nopl   0x0(%rax,%rax,1)
    7cdf:	00 
    7ce0:	48 8b 47 78          	mov    0x78(%rdi),%rax
    7ce4:	48 85 c0             	test   %rax,%rax
    7ce7:	74 ba                	je     7ca3 <_ZN8argparse8Argument14implicit_valueESt3any+0x63>
    7ce9:	31 d2                	xor    %edx,%edx
    7ceb:	4c 89 ee             	mov    %r13,%rsi
    7cee:	bf 03 00 00 00       	mov    $0x3,%edi
    7cf3:	ff d0                	call   *%rax
    7cf5:	49 c7 44 24 78 00 00 	movq   $0x0,0x78(%r12)
    7cfc:	00 00 
    7cfe:	eb a3                	jmp    7ca3 <_ZN8argparse8Argument14implicit_valueESt3any+0x63>
    7d00:	e8 4b b4 ff ff       	call   3150 <__stack_chk_fail@plt>
    7d05:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
    7d0c:	00 00 00 
    7d0f:	90                   	nop

0000000000007d10 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE>:
    7d10:	48 85 ff             	test   %rdi,%rdi
    7d13:	0f 84 17 05 00 00    	je     8230 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x520>
    7d19:	48 83 ec 08          	sub    $0x8,%rsp
    7d1d:	0f b6 06             	movzbl (%rsi),%eax
    7d20:	3c 30                	cmp    $0x30,%al
    7d22:	0f 84 48 01 00 00    	je     7e70 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x160>
    7d28:	0f 8f b2 00 00 00    	jg     7de0 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0xd0>
    7d2e:	3c 2e                	cmp    $0x2e,%al
    7d30:	0f 85 9a 00 00 00    	jne    7dd0 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0xc0>
    7d36:	48 ff c6             	inc    %rsi
    7d39:	48 ff cf             	dec    %rdi
    7d3c:	0f 84 8e 00 00 00    	je     7dd0 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0xc0>
    7d42:	0f b6 06             	movzbl (%rsi),%eax
    7d45:	83 e8 30             	sub    $0x30,%eax
    7d48:	83 f8 09             	cmp    $0x9,%eax
    7d4b:	0f 87 7f 00 00 00    	ja     7dd0 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0xc0>
    7d51:	48 89 f8             	mov    %rdi,%rax
    7d54:	48 c1 f8 02          	sar    $0x2,%rax
    7d58:	4c 8d 04 3e          	lea    (%rsi,%rdi,1),%r8
    7d5c:	48 89 fa             	mov    %rdi,%rdx
    7d5f:	48 85 c0             	test   %rax,%rax
    7d62:	0f 8e bc 04 00 00    	jle    8224 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x514>
    7d68:	48 8d 0c 86          	lea    (%rsi,%rax,4),%rcx
    7d6c:	48 89 f0             	mov    %rsi,%rax
    7d6f:	0f be 10             	movsbl (%rax),%edx
    7d72:	83 ea 30             	sub    $0x30,%edx
    7d75:	83 fa 09             	cmp    $0x9,%edx
    7d78:	77 26                	ja     7da0 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x90>
    7d7a:	0f be 50 01          	movsbl 0x1(%rax),%edx
    7d7e:	83 ea 30             	sub    $0x30,%edx
    7d81:	83 fa 09             	cmp    $0x9,%edx
    7d84:	0f 87 96 02 00 00    	ja     8020 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x310>
    7d8a:	0f be 50 02          	movsbl 0x2(%rax),%edx
    7d8e:	83 ea 30             	sub    $0x30,%edx
    7d91:	83 fa 09             	cmp    $0x9,%edx
    7d94:	0f 86 5e 03 00 00    	jbe    80f8 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x3e8>
    7d9a:	48 83 c0 02          	add    $0x2,%rax
    7d9e:	66 90                	xchg   %ax,%ax
    7da0:	48 89 c2             	mov    %rax,%rdx
    7da3:	48 29 f2             	sub    %rsi,%rdx
    7da6:	48 39 fa             	cmp    %rdi,%rdx
    7da9:	0f 87 53 05 00 00    	ja     8302 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x5f2>
    7daf:	48 29 d7             	sub    %rdx,%rdi
    7db2:	0f 84 c0 00 00 00    	je     7e78 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x168>
    7db8:	0f b6 10             	movzbl (%rax),%edx
    7dbb:	83 e2 df             	and    $0xffffffdf,%edx
    7dbe:	80 fa 45             	cmp    $0x45,%dl
    7dc1:	0f 84 61 01 00 00    	je     7f28 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x218>
    7dc7:	66 0f 1f 84 00 00 00 	nopw   0x0(%rax,%rax,1)
    7dce:	00 00 
    7dd0:	31 c0                	xor    %eax,%eax
    7dd2:	48 83 c4 08          	add    $0x8,%rsp
    7dd6:	c3                   	ret    
    7dd7:	66 0f 1f 84 00 00 00 	nopw   0x0(%rax,%rax,1)
    7dde:	00 00 
    7de0:	83 e8 31             	sub    $0x31,%eax
    7de3:	3c 08                	cmp    $0x8,%al
    7de5:	77 e9                	ja     7dd0 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0xc0>
    7de7:	48 89 f8             	mov    %rdi,%rax
    7dea:	48 c1 f8 02          	sar    $0x2,%rax
    7dee:	48 8d 0c 3e          	lea    (%rsi,%rdi,1),%rcx
    7df2:	48 89 fa             	mov    %rdi,%rdx
    7df5:	48 85 c0             	test   %rax,%rax
    7df8:	0f 8e 6d 02 00 00    	jle    806b <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x35b>
    7dfe:	48 8d 14 86          	lea    (%rsi,%rax,4),%rdx
    7e02:	49 89 f0             	mov    %rsi,%r8
    7e05:	41 0f be 00          	movsbl (%r8),%eax
    7e09:	83 e8 30             	sub    $0x30,%eax
    7e0c:	83 f8 09             	cmp    $0x9,%eax
    7e0f:	76 77                	jbe    7e88 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x178>
    7e11:	4c 89 c2             	mov    %r8,%rdx
    7e14:	48 29 f2             	sub    %rsi,%rdx
    7e17:	48 39 d7             	cmp    %rdx,%rdi
    7e1a:	0f 82 fa 04 00 00    	jb     831a <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x60a>
    7e20:	48 29 d7             	sub    %rdx,%rdi
    7e23:	48 83 ff ff          	cmp    $0xffffffffffffffff,%rdi
    7e27:	0f 85 73 01 00 00    	jne    7fa0 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x290>
    7e2d:	41 0f b6 00          	movzbl (%r8),%eax
    7e31:	89 c2                	mov    %eax,%edx
    7e33:	83 e2 df             	and    $0xffffffdf,%edx
    7e36:	80 fa 45             	cmp    $0x45,%dl
    7e39:	0f 84 69 02 00 00    	je     80a8 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x398>
    7e3f:	3c 2e                	cmp    $0x2e,%al
    7e41:	75 8d                	jne    7dd0 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0xc0>
    7e43:	49 8d 70 01          	lea    0x1(%r8),%rsi
    7e47:	48 c7 c7 fe ff ff ff 	mov    $0xfffffffffffffffe,%rdi
    7e4e:	41 0f b6 40 01       	movzbl 0x1(%r8),%eax
    7e53:	83 e8 30             	sub    $0x30,%eax
    7e56:	83 f8 09             	cmp    $0x9,%eax
    7e59:	0f 86 e3 fe ff ff    	jbe    7d42 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x32>
    7e5f:	48 89 f0             	mov    %rsi,%rax
    7e62:	e9 51 ff ff ff       	jmp    7db8 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0xa8>
    7e67:	66 0f 1f 84 00 00 00 	nopw   0x0(%rax,%rax,1)
    7e6e:	00 00 
    7e70:	48 89 f9             	mov    %rdi,%rcx
    7e73:	48 ff c9             	dec    %rcx
    7e76:	75 38                	jne    7eb0 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x1a0>
    7e78:	b8 01 00 00 00       	mov    $0x1,%eax
    7e7d:	48 83 c4 08          	add    $0x8,%rsp
    7e81:	c3                   	ret    
    7e82:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)
    7e88:	41 0f be 40 01       	movsbl 0x1(%r8),%eax
    7e8d:	83 e8 30             	sub    $0x30,%eax
    7e90:	83 f8 09             	cmp    $0x9,%eax
    7e93:	0f 86 17 01 00 00    	jbe    7fb0 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x2a0>
    7e99:	49 ff c0             	inc    %r8
    7e9c:	4c 89 c2             	mov    %r8,%rdx
    7e9f:	48 29 f2             	sub    %rsi,%rdx
    7ea2:	e9 70 ff ff ff       	jmp    7e17 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x107>
    7ea7:	66 0f 1f 84 00 00 00 	nopw   0x0(%rax,%rax,1)
    7eae:	00 00 
    7eb0:	48 89 c8             	mov    %rcx,%rax
    7eb3:	48 c1 f8 02          	sar    $0x2,%rax
    7eb7:	4c 8d 46 01          	lea    0x1(%rsi),%r8
    7ebb:	48 85 c0             	test   %rax,%rax
    7ebe:	0f 8e 9a 03 00 00    	jle    825e <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x54e>
    7ec4:	4c 8d 4c 86 01       	lea    0x1(%rsi,%rax,4),%r9
    7ec9:	4c 89 c2             	mov    %r8,%rdx
    7ecc:	0f be 02             	movsbl (%rdx),%eax
    7ecf:	83 e8 30             	sub    $0x30,%eax
    7ed2:	83 f8 09             	cmp    $0x9,%eax
    7ed5:	0f 86 25 01 00 00    	jbe    8000 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x2f0>
    7edb:	4c 29 c2             	sub    %r8,%rdx
    7ede:	48 39 d1             	cmp    %rdx,%rcx
    7ee1:	0f 82 4b 04 00 00    	jb     8332 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x622>
    7ee7:	49 01 d0             	add    %rdx,%r8
    7eea:	48 29 d1             	sub    %rdx,%rcx
    7eed:	48 89 cf             	mov    %rcx,%rdi
    7ef0:	0f 84 da fe ff ff    	je     7dd0 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0xc0>
    7ef6:	41 0f b6 00          	movzbl (%r8),%eax
    7efa:	89 c2                	mov    %eax,%edx
    7efc:	83 e2 df             	and    $0xffffffdf,%edx
    7eff:	80 fa 45             	cmp    $0x45,%dl
    7f02:	0f 84 57 01 00 00    	je     805f <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x34f>
    7f08:	3c 2e                	cmp    $0x2e,%al
    7f0a:	0f 85 c0 fe ff ff    	jne    7dd0 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0xc0>
    7f10:	49 8d 70 01          	lea    0x1(%r8),%rsi
    7f14:	48 ff cf             	dec    %rdi
    7f17:	0f 85 31 ff ff ff    	jne    7e4e <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x13e>
    7f1d:	e9 56 ff ff ff       	jmp    7e78 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x168>
    7f22:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)
    7f28:	48 ff c0             	inc    %rax
    7f2b:	48 ff cf             	dec    %rdi
    7f2e:	48 85 ff             	test   %rdi,%rdi
    7f31:	0f 84 99 fe ff ff    	je     7dd0 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0xc0>
    7f37:	0f b6 08             	movzbl (%rax),%ecx
    7f3a:	8d 51 d5             	lea    -0x2b(%rcx),%edx
    7f3d:	81 e2 fd 00 00 00    	and    $0xfd,%edx
    7f43:	0f 84 9f 00 00 00    	je     7fe8 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x2d8>
    7f49:	0f b6 10             	movzbl (%rax),%edx
    7f4c:	83 ea 30             	sub    $0x30,%edx
    7f4f:	83 fa 09             	cmp    $0x9,%edx
    7f52:	0f 87 78 fe ff ff    	ja     7dd0 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0xc0>
    7f58:	48 89 fa             	mov    %rdi,%rdx
    7f5b:	48 c1 fa 02          	sar    $0x2,%rdx
    7f5f:	48 8d 34 38          	lea    (%rax,%rdi,1),%rsi
    7f63:	48 89 f9             	mov    %rdi,%rcx
    7f66:	48 85 d2             	test   %rdx,%rdx
    7f69:	0f 8e f7 02 00 00    	jle    8266 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x556>
    7f6f:	4c 8d 04 90          	lea    (%rax,%rdx,4),%r8
    7f73:	48 89 c2             	mov    %rax,%rdx
    7f76:	0f be 0a             	movsbl (%rdx),%ecx
    7f79:	83 e9 30             	sub    $0x30,%ecx
    7f7c:	83 f9 09             	cmp    $0x9,%ecx
    7f7f:	0f 86 ab 00 00 00    	jbe    8030 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x320>
    7f85:	48 29 c2             	sub    %rax,%rdx
    7f88:	48 39 d7             	cmp    %rdx,%rdi
    7f8b:	0f 82 b6 03 00 00    	jb     8347 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x637>
    7f91:	0f 94 c0             	sete   %al
    7f94:	e9 39 fe ff ff       	jmp    7dd2 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0xc2>
    7f99:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
    7fa0:	48 85 ff             	test   %rdi,%rdi
    7fa3:	0f 84 cf fe ff ff    	je     7e78 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x168>
    7fa9:	e9 48 ff ff ff       	jmp    7ef6 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x1e6>
    7fae:	66 90                	xchg   %ax,%ax
    7fb0:	41 0f be 40 02       	movsbl 0x2(%r8),%eax
    7fb5:	83 e8 30             	sub    $0x30,%eax
    7fb8:	83 f8 09             	cmp    $0x9,%eax
    7fbb:	0f 87 8f 00 00 00    	ja     8050 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x340>
    7fc1:	41 0f be 40 03       	movsbl 0x3(%r8),%eax
    7fc6:	83 e8 30             	sub    $0x30,%eax
    7fc9:	83 f8 09             	cmp    $0x9,%eax
    7fcc:	0f 86 56 01 00 00    	jbe    8128 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x418>
    7fd2:	49 83 c0 03          	add    $0x3,%r8
    7fd6:	4c 89 c2             	mov    %r8,%rdx
    7fd9:	48 29 f2             	sub    %rsi,%rdx
    7fdc:	e9 36 fe ff ff       	jmp    7e17 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x107>
    7fe1:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
    7fe8:	48 ff c0             	inc    %rax
    7feb:	48 ff cf             	dec    %rdi
    7fee:	0f 85 55 ff ff ff    	jne    7f49 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x239>
    7ff4:	e9 d7 fd ff ff       	jmp    7dd0 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0xc0>
    7ff9:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
    8000:	0f be 42 01          	movsbl 0x1(%rdx),%eax
    8004:	83 e8 30             	sub    $0x30,%eax
    8007:	83 f8 09             	cmp    $0x9,%eax
    800a:	0f 86 d0 00 00 00    	jbe    80e0 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x3d0>
    8010:	48 ff c2             	inc    %rdx
    8013:	e9 c3 fe ff ff       	jmp    7edb <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x1cb>
    8018:	0f 1f 84 00 00 00 00 	nopl   0x0(%rax,%rax,1)
    801f:	00 
    8020:	48 ff c0             	inc    %rax
    8023:	e9 78 fd ff ff       	jmp    7da0 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x90>
    8028:	0f 1f 84 00 00 00 00 	nopl   0x0(%rax,%rax,1)
    802f:	00 
    8030:	0f be 4a 01          	movsbl 0x1(%rdx),%ecx
    8034:	83 e9 30             	sub    $0x30,%ecx
    8037:	83 f9 09             	cmp    $0x9,%ecx
    803a:	0f 86 d0 00 00 00    	jbe    8110 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x400>
    8040:	48 ff c2             	inc    %rdx
    8043:	e9 3d ff ff ff       	jmp    7f85 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x275>
    8048:	0f 1f 84 00 00 00 00 	nopl   0x0(%rax,%rax,1)
    804f:	00 
    8050:	49 83 c0 02          	add    $0x2,%r8
    8054:	4c 89 c2             	mov    %r8,%rdx
    8057:	48 29 f2             	sub    %rsi,%rdx
    805a:	e9 b8 fd ff ff       	jmp    7e17 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x107>
    805f:	48 ff cf             	dec    %rdi
    8062:	49 8d 40 01          	lea    0x1(%r8),%rax
    8066:	e9 c3 fe ff ff       	jmp    7f2e <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x21e>
    806b:	49 89 f0             	mov    %rsi,%r8
    806e:	48 83 fa 02          	cmp    $0x2,%rdx
    8072:	0f 84 ce 01 00 00    	je     8246 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x536>
    8078:	48 83 fa 03          	cmp    $0x3,%rdx
    807c:	0f 84 b1 01 00 00    	je     8233 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x523>
    8082:	48 83 fa 01          	cmp    $0x1,%rdx
    8086:	0f 85 ec fd ff ff    	jne    7e78 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x168>
    808c:	41 0f be 00          	movsbl (%r8),%eax
    8090:	83 e8 30             	sub    $0x30,%eax
    8093:	83 f8 09             	cmp    $0x9,%eax
    8096:	0f 86 dc fd ff ff    	jbe    7e78 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x168>
    809c:	e9 70 fd ff ff       	jmp    7e11 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x101>
    80a1:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
    80a8:	41 0f b6 48 01       	movzbl 0x1(%r8),%ecx
    80ad:	49 8d 40 01          	lea    0x1(%r8),%rax
    80b1:	8d 51 d5             	lea    -0x2b(%rcx),%edx
    80b4:	81 e2 fd 00 00 00    	and    $0xfd,%edx
    80ba:	48 c7 c7 fe ff ff ff 	mov    $0xfffffffffffffffe,%rdi
    80c1:	0f 85 82 fe ff ff    	jne    7f49 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x239>
    80c7:	49 8d 40 02          	lea    0x2(%r8),%rax
    80cb:	48 c7 c7 fd ff ff ff 	mov    $0xfffffffffffffffd,%rdi
    80d2:	e9 72 fe ff ff       	jmp    7f49 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x239>
    80d7:	66 0f 1f 84 00 00 00 	nopw   0x0(%rax,%rax,1)
    80de:	00 00 
    80e0:	0f be 42 02          	movsbl 0x2(%rdx),%eax
    80e4:	83 e8 30             	sub    $0x30,%eax
    80e7:	83 f8 09             	cmp    $0x9,%eax
    80ea:	76 54                	jbe    8140 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x430>
    80ec:	48 83 c2 02          	add    $0x2,%rdx
    80f0:	e9 e6 fd ff ff       	jmp    7edb <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x1cb>
    80f5:	0f 1f 00             	nopl   (%rax)
    80f8:	0f be 50 03          	movsbl 0x3(%rax),%edx
    80fc:	83 ea 30             	sub    $0x30,%edx
    80ff:	83 fa 09             	cmp    $0x9,%edx
    8102:	76 51                	jbe    8155 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x445>
    8104:	48 83 c0 03          	add    $0x3,%rax
    8108:	e9 93 fc ff ff       	jmp    7da0 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x90>
    810d:	0f 1f 00             	nopl   (%rax)
    8110:	0f be 4a 02          	movsbl 0x2(%rdx),%ecx
    8114:	83 e9 30             	sub    $0x30,%ecx
    8117:	83 f9 09             	cmp    $0x9,%ecx
    811a:	76 72                	jbe    818e <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x47e>
    811c:	48 83 c2 02          	add    $0x2,%rdx
    8120:	e9 60 fe ff ff       	jmp    7f85 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x275>
    8125:	0f 1f 00             	nopl   (%rax)
    8128:	49 83 c0 04          	add    $0x4,%r8
    812c:	4c 39 c2             	cmp    %r8,%rdx
    812f:	0f 85 d0 fc ff ff    	jne    7e05 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0xf5>
    8135:	4c 29 c1             	sub    %r8,%rcx
    8138:	48 89 ca             	mov    %rcx,%rdx
    813b:	e9 2e ff ff ff       	jmp    806e <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x35e>
    8140:	0f be 42 03          	movsbl 0x3(%rdx),%eax
    8144:	83 e8 30             	sub    $0x30,%eax
    8147:	83 f8 09             	cmp    $0x9,%eax
    814a:	76 57                	jbe    81a3 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x493>
    814c:	48 83 c2 03          	add    $0x3,%rdx
    8150:	e9 86 fd ff ff       	jmp    7edb <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x1cb>
    8155:	48 83 c0 04          	add    $0x4,%rax
    8159:	48 39 c1             	cmp    %rax,%rcx
    815c:	0f 85 0d fc ff ff    	jne    7d6f <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x5f>
    8162:	4c 89 c2             	mov    %r8,%rdx
    8165:	48 29 c2             	sub    %rax,%rdx
    8168:	48 83 fa 02          	cmp    $0x2,%rdx
    816c:	0f 84 0b 01 00 00    	je     827d <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x56d>
    8172:	48 83 fa 03          	cmp    $0x3,%rdx
    8176:	0f 84 ef 00 00 00    	je     826b <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x55b>
    817c:	48 83 fa 01          	cmp    $0x1,%rdx
    8180:	0f 84 09 01 00 00    	je     828f <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x57f>
    8186:	4c 89 c0             	mov    %r8,%rax
    8189:	e9 12 fc ff ff       	jmp    7da0 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x90>
    818e:	0f be 4a 03          	movsbl 0x3(%rdx),%ecx
    8192:	83 e9 30             	sub    $0x30,%ecx
    8195:	83 f9 09             	cmp    $0x9,%ecx
    8198:	76 51                	jbe    81eb <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x4db>
    819a:	48 83 c2 03          	add    $0x3,%rdx
    819e:	e9 e2 fd ff ff       	jmp    7f85 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x275>
    81a3:	48 83 c2 04          	add    $0x4,%rdx
    81a7:	49 39 d1             	cmp    %rdx,%r9
    81aa:	0f 85 1c fd ff ff    	jne    7ecc <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x1bc>
    81b0:	48 8d 14 3e          	lea    (%rsi,%rdi,1),%rdx
    81b4:	48 89 d0             	mov    %rdx,%rax
    81b7:	4c 29 c8             	sub    %r9,%rax
    81ba:	48 83 f8 02          	cmp    $0x2,%rax
    81be:	0f 84 ec 00 00 00    	je     82b0 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x5a0>
    81c4:	48 83 f8 03          	cmp    $0x3,%rax
    81c8:	0f 84 d3 00 00 00    	je     82a1 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x591>
    81ce:	48 83 f8 01          	cmp    $0x1,%rax
    81d2:	0f 85 03 fd ff ff    	jne    7edb <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x1cb>
    81d8:	41 0f be 01          	movsbl (%r9),%eax
    81dc:	83 e8 30             	sub    $0x30,%eax
    81df:	83 f8 0a             	cmp    $0xa,%eax
    81e2:	49 0f 43 d1          	cmovae %r9,%rdx
    81e6:	e9 f0 fc ff ff       	jmp    7edb <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x1cb>
    81eb:	48 83 c2 04          	add    $0x4,%rdx
    81ef:	49 39 d0             	cmp    %rdx,%r8
    81f2:	0f 85 7e fd ff ff    	jne    7f76 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x266>
    81f8:	48 89 f1             	mov    %rsi,%rcx
    81fb:	48 29 d1             	sub    %rdx,%rcx
    81fe:	48 83 f9 02          	cmp    $0x2,%rcx
    8202:	0f 84 ce 00 00 00    	je     82d6 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x5c6>
    8208:	48 83 f9 03          	cmp    $0x3,%rcx
    820c:	0f 84 b2 00 00 00    	je     82c4 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x5b4>
    8212:	48 83 f9 01          	cmp    $0x1,%rcx
    8216:	0f 84 cc 00 00 00    	je     82e8 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x5d8>
    821c:	48 89 f2             	mov    %rsi,%rdx
    821f:	e9 61 fd ff ff       	jmp    7f85 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x275>
    8224:	48 89 f0             	mov    %rsi,%rax
    8227:	e9 3c ff ff ff       	jmp    8168 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x458>
    822c:	0f 1f 40 00          	nopl   0x0(%rax)
    8230:	31 c0                	xor    %eax,%eax
    8232:	c3                   	ret    
    8233:	41 0f be 00          	movsbl (%r8),%eax
    8237:	83 e8 30             	sub    $0x30,%eax
    823a:	83 f8 09             	cmp    $0x9,%eax
    823d:	0f 87 ce fb ff ff    	ja     7e11 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x101>
    8243:	49 ff c0             	inc    %r8
    8246:	41 0f be 00          	movsbl (%r8),%eax
    824a:	83 e8 30             	sub    $0x30,%eax
    824d:	83 f8 09             	cmp    $0x9,%eax
    8250:	0f 87 bb fb ff ff    	ja     7e11 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x101>
    8256:	49 ff c0             	inc    %r8
    8259:	e9 2e fe ff ff       	jmp    808c <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x37c>
    825e:	4d 89 c1             	mov    %r8,%r9
    8261:	e9 4a ff ff ff       	jmp    81b0 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x4a0>
    8266:	48 89 c2             	mov    %rax,%rdx
    8269:	eb 93                	jmp    81fe <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x4ee>
    826b:	0f be 10             	movsbl (%rax),%edx
    826e:	83 ea 30             	sub    $0x30,%edx
    8271:	83 fa 09             	cmp    $0x9,%edx
    8274:	0f 87 26 fb ff ff    	ja     7da0 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x90>
    827a:	48 ff c0             	inc    %rax
    827d:	0f be 10             	movsbl (%rax),%edx
    8280:	83 ea 30             	sub    $0x30,%edx
    8283:	83 fa 09             	cmp    $0x9,%edx
    8286:	0f 87 14 fb ff ff    	ja     7da0 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x90>
    828c:	48 ff c0             	inc    %rax
    828f:	0f be 10             	movsbl (%rax),%edx
    8292:	83 ea 30             	sub    $0x30,%edx
    8295:	83 fa 09             	cmp    $0x9,%edx
    8298:	49 0f 46 c0          	cmovbe %r8,%rax
    829c:	e9 ff fa ff ff       	jmp    7da0 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x90>
    82a1:	41 0f be 01          	movsbl (%r9),%eax
    82a5:	83 e8 30             	sub    $0x30,%eax
    82a8:	83 f8 09             	cmp    $0x9,%eax
    82ab:	77 4d                	ja     82fa <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x5ea>
    82ad:	49 ff c1             	inc    %r9
    82b0:	41 0f be 01          	movsbl (%r9),%eax
    82b4:	83 e8 30             	sub    $0x30,%eax
    82b7:	83 f8 09             	cmp    $0x9,%eax
    82ba:	77 3e                	ja     82fa <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x5ea>
    82bc:	49 ff c1             	inc    %r9
    82bf:	e9 14 ff ff ff       	jmp    81d8 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x4c8>
    82c4:	0f be 0a             	movsbl (%rdx),%ecx
    82c7:	83 e9 30             	sub    $0x30,%ecx
    82ca:	83 f9 09             	cmp    $0x9,%ecx
    82cd:	0f 87 b2 fc ff ff    	ja     7f85 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x275>
    82d3:	48 ff c2             	inc    %rdx
    82d6:	0f be 0a             	movsbl (%rdx),%ecx
    82d9:	83 e9 30             	sub    $0x30,%ecx
    82dc:	83 f9 09             	cmp    $0x9,%ecx
    82df:	0f 87 a0 fc ff ff    	ja     7f85 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x275>
    82e5:	48 ff c2             	inc    %rdx
    82e8:	0f be 0a             	movsbl (%rdx),%ecx
    82eb:	83 e9 30             	sub    $0x30,%ecx
    82ee:	83 f9 09             	cmp    $0x9,%ecx
    82f1:	48 0f 46 d6          	cmovbe %rsi,%rdx
    82f5:	e9 8b fc ff ff       	jmp    7f85 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x275>
    82fa:	4c 89 ca             	mov    %r9,%rdx
    82fd:	e9 d9 fb ff ff       	jmp    7edb <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE+0x1cb>
    8302:	48 89 f9             	mov    %rdi,%rcx
    8305:	48 8d 35 34 7d 00 00 	lea    0x7d34(%rip),%rsi        # 10040 <_fini+0xe1f>
    830c:	48 8d 3d 4d 7d 00 00 	lea    0x7d4d(%rip),%rdi        # 10060 <_fini+0xe3f>
    8313:	31 c0                	xor    %eax,%eax
    8315:	e8 a6 af ff ff       	call   32c0 <_ZSt24__throw_out_of_range_fmtPKcz@plt>
    831a:	48 89 f9             	mov    %rdi,%rcx
    831d:	48 8d 35 1c 7d 00 00 	lea    0x7d1c(%rip),%rsi        # 10040 <_fini+0xe1f>
    8324:	48 8d 3d 35 7d 00 00 	lea    0x7d35(%rip),%rdi        # 10060 <_fini+0xe3f>
    832b:	31 c0                	xor    %eax,%eax
    832d:	e8 8e af ff ff       	call   32c0 <_ZSt24__throw_out_of_range_fmtPKcz@plt>
    8332:	48 8d 35 07 7d 00 00 	lea    0x7d07(%rip),%rsi        # 10040 <_fini+0xe1f>
    8339:	48 8d 3d 20 7d 00 00 	lea    0x7d20(%rip),%rdi        # 10060 <_fini+0xe3f>
    8340:	31 c0                	xor    %eax,%eax
    8342:	e8 79 af ff ff       	call   32c0 <_ZSt24__throw_out_of_range_fmtPKcz@plt>
    8347:	48 89 f9             	mov    %rdi,%rcx
    834a:	48 8d 35 ef 7c 00 00 	lea    0x7cef(%rip),%rsi        # 10040 <_fini+0xe1f>
    8351:	48 8d 3d 08 7d 00 00 	lea    0x7d08(%rip),%rdi        # 10060 <_fini+0xe3f>
    8358:	31 c0                	xor    %eax,%eax
    835a:	e8 61 af ff ff       	call   32c0 <_ZSt24__throw_out_of_range_fmtPKcz@plt>
    835f:	90                   	nop

0000000000008360 <_ZNSt7__cxx1115basic_stringbufIcSt11char_traitsIcESaIcEED1Ev>:
    8360:	53                   	push   %rbx
    8361:	48 8d 05 b8 b6 00 00 	lea    0xb6b8(%rip),%rax        # 13a20 <_ZTVNSt7__cxx1115basic_stringbufIcSt11char_traitsIcESaIcEEE+0x10>
    8368:	48 89 07             	mov    %rax,(%rdi)
    836b:	48 89 fb             	mov    %rdi,%rbx
    836e:	48 8b 7f 48          	mov    0x48(%rdi),%rdi
    8372:	48 8d 43 58          	lea    0x58(%rbx),%rax
    8376:	48 39 c7             	cmp    %rax,%rdi
    8379:	74 0d                	je     8388 <_ZNSt7__cxx1115basic_stringbufIcSt11char_traitsIcESaIcEED1Ev+0x28>
    837b:	48 8b 43 58          	mov    0x58(%rbx),%rax
    837f:	48 8d 70 01          	lea    0x1(%rax),%rsi
    8383:	e8 98 af ff ff       	call   3320 <_ZdlPvm@plt>
    8388:	48 8d 05 a1 b5 00 00 	lea    0xb5a1(%rip),%rax        # 13930 <_ZTVSt15basic_streambufIcSt11char_traitsIcEE+0x10>
    838f:	48 89 03             	mov    %rax,(%rbx)
    8392:	48 8d 7b 38          	lea    0x38(%rbx),%rdi
    8396:	5b                   	pop    %rbx
    8397:	e9 94 ae ff ff       	jmp    3230 <_ZNSt6localeD1Ev@plt>
    839c:	0f 1f 40 00          	nopl   0x0(%rax)

00000000000083a0 <_ZNSt7__cxx1115basic_stringbufIcSt11char_traitsIcESaIcEED0Ev>:
    83a0:	55                   	push   %rbp
    83a1:	48 8d 05 78 b6 00 00 	lea    0xb678(%rip),%rax        # 13a20 <_ZTVNSt7__cxx1115basic_stringbufIcSt11char_traitsIcESaIcEEE+0x10>
    83a8:	48 89 07             	mov    %rax,(%rdi)
    83ab:	48 89 fd             	mov    %rdi,%rbp
    83ae:	48 8b 7f 48          	mov    0x48(%rdi),%rdi
    83b2:	48 8d 45 58          	lea    0x58(%rbp),%rax
    83b6:	48 39 c7             	cmp    %rax,%rdi
    83b9:	74 0d                	je     83c8 <_ZNSt7__cxx1115basic_stringbufIcSt11char_traitsIcESaIcEED0Ev+0x28>
    83bb:	48 8b 45 58          	mov    0x58(%rbp),%rax
    83bf:	48 8d 70 01          	lea    0x1(%rax),%rsi
    83c3:	e8 58 af ff ff       	call   3320 <_ZdlPvm@plt>
    83c8:	48 8d 05 61 b5 00 00 	lea    0xb561(%rip),%rax        # 13930 <_ZTVSt15basic_streambufIcSt11char_traitsIcEE+0x10>
    83cf:	48 89 45 00          	mov    %rax,0x0(%rbp)
    83d3:	48 8d 7d 38          	lea    0x38(%rbp),%rdi
    83d7:	e8 54 ae ff ff       	call   3230 <_ZNSt6localeD1Ev@plt>
    83dc:	48 89 ef             	mov    %rbp,%rdi
    83df:	be 68 00 00 00       	mov    $0x68,%esi
    83e4:	5d                   	pop    %rbp
    83e5:	e9 36 af ff ff       	jmp    3320 <_ZdlPvm@plt>
    83ea:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)

00000000000083f0 <_ZNK8argparse8Argument33throw_required_arg_not_used_errorEv>:
    83f0:	41 55                	push   %r13
    83f2:	41 54                	push   %r12
    83f4:	55                   	push   %rbp
    83f5:	53                   	push   %rbx
    83f6:	48 89 fb             	mov    %rdi,%rbx
    83f9:	48 81 ec b8 01 00 00 	sub    $0x1b8,%rsp
    8400:	48 8d 6c 24 20       	lea    0x20(%rsp),%rbp
    8405:	48 89 ef             	mov    %rbp,%rdi
    8408:	64 48 8b 04 25 28 00 	mov    %fs:0x28,%rax
    840f:	00 00 
    8411:	48 89 84 24 a8 01 00 	mov    %rax,0x1a8(%rsp)
    8418:	00 
    8419:	31 c0                	xor    %eax,%eax
    841b:	e8 f0 ae ff ff       	call   3310 <_ZNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEC1Ev@plt>
    8420:	48 8b 03             	mov    (%rbx),%rax
    8423:	48 8d 7c 24 30       	lea    0x30(%rsp),%rdi
    8428:	48 8b 50 08          	mov    0x8(%rax),%rdx
    842c:	48 8b 30             	mov    (%rax),%rsi
    842f:	e8 6c af ff ff       	call   33a0 <_ZSt16__ostream_insertIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_PKS3_l@plt>
    8434:	48 89 c7             	mov    %rax,%rdi
    8437:	48 8d 35 14 7d 00 00 	lea    0x7d14(%rip),%rsi        # 10152 <_fini+0xf31>
    843e:	e8 dd ac ff ff       	call   3120 <_ZStlsISt11char_traitsIcEERSt13basic_ostreamIcT_ES5_PKc@plt>
    8443:	bf 10 00 00 00       	mov    $0x10,%edi
    8448:	49 89 e5             	mov    %rsp,%r13
    844b:	e8 80 ac ff ff       	call   30d0 <__cxa_allocate_exception@plt>
    8450:	48 8d 74 24 38       	lea    0x38(%rsp),%rsi
    8455:	4c 89 ef             	mov    %r13,%rdi
    8458:	49 89 c4             	mov    %rax,%r12
    845b:	e8 20 ad ff ff       	call   3180 <_ZNKSt7__cxx1115basic_stringbufIcSt11char_traitsIcESaIcEE3strEv@plt>
    8460:	4c 89 ee             	mov    %r13,%rsi
    8463:	4c 89 e7             	mov    %r12,%rdi
    8466:	e8 25 ae ff ff       	call   3290 <_ZNSt13runtime_errorC1ERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEE@plt>
    846b:	48 8b 3c 24          	mov    (%rsp),%rdi
    846f:	48 8d 44 24 10       	lea    0x10(%rsp),%rax
    8474:	48 39 c7             	cmp    %rax,%rdi
    8477:	74 0e                	je     8487 <_ZNK8argparse8Argument33throw_required_arg_not_used_errorEv+0x97>
    8479:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
    847e:	48 8d 70 01          	lea    0x1(%rax),%rsi
    8482:	e8 99 ae ff ff       	call   3320 <_ZdlPvm@plt>
    8487:	48 8b 15 3a bb 00 00 	mov    0xbb3a(%rip),%rdx        # 13fc8 <_ZNSt13runtime_errorD1Ev@Base>
    848e:	48 8d 35 0b b5 00 00 	lea    0xb50b(%rip),%rsi        # 139a0 <_ZTISt13runtime_error@@Base>
    8495:	4c 89 e7             	mov    %r12,%rdi
    8498:	e8 83 ad ff ff       	call   3220 <__cxa_throw@plt>
    849d:	48 89 c3             	mov    %rax,%rbx
    84a0:	eb 23                	jmp    84c5 <_ZNK8argparse8Argument33throw_required_arg_not_used_errorEv+0xd5>
    84a2:	48 89 c3             	mov    %rax,%rbx
    84a5:	eb 05                	jmp    84ac <_ZNK8argparse8Argument33throw_required_arg_not_used_errorEv+0xbc>
    84a7:	48 89 c3             	mov    %rax,%rbx
    84aa:	eb 0e                	jmp    84ba <_ZNK8argparse8Argument33throw_required_arg_not_used_errorEv+0xca>
    84ac:	48 8b 3c 24          	mov    (%rsp),%rdi
    84b0:	48 8d 44 24 10       	lea    0x10(%rsp),%rax
    84b5:	48 39 c7             	cmp    %rax,%rdi
    84b8:	75 1e                	jne    84d8 <_ZNK8argparse8Argument33throw_required_arg_not_used_errorEv+0xe8>
    84ba:	4c 89 e7             	mov    %r12,%rdi
    84bd:	c5 f8 77             	vzeroupper 
    84c0:	e8 eb ae ff ff       	call   33b0 <__cxa_free_exception@plt>
    84c5:	48 89 ef             	mov    %rbp,%rdi
    84c8:	c5 f8 77             	vzeroupper 
    84cb:	e8 b0 ae ff ff       	call   3380 <_ZNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEED1Ev@plt>
    84d0:	48 89 df             	mov    %rbx,%rdi
    84d3:	e8 88 ae ff ff       	call   3360 <_Unwind_Resume@plt>
    84d8:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
    84dd:	48 8d 70 01          	lea    0x1(%rax),%rsi
    84e1:	c5 f8 77             	vzeroupper 
    84e4:	e8 37 ae ff ff       	call   3320 <_ZdlPvm@plt>
    84e9:	eb cf                	jmp    84ba <_ZNK8argparse8Argument33throw_required_arg_not_used_errorEv+0xca>
    84eb:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)

00000000000084f0 <_ZNK8argparse8Argument42throw_required_arg_no_value_provided_errorEv>:
    84f0:	41 55                	push   %r13
    84f2:	41 54                	push   %r12
    84f4:	55                   	push   %rbp
    84f5:	53                   	push   %rbx
    84f6:	48 89 fb             	mov    %rdi,%rbx
    84f9:	48 81 ec b8 01 00 00 	sub    $0x1b8,%rsp
    8500:	48 8d 6c 24 20       	lea    0x20(%rsp),%rbp
    8505:	48 89 ef             	mov    %rbp,%rdi
    8508:	64 48 8b 04 25 28 00 	mov    %fs:0x28,%rax
    850f:	00 00 
    8511:	48 89 84 24 a8 01 00 	mov    %rax,0x1a8(%rsp)
    8518:	00 
    8519:	31 c0                	xor    %eax,%eax
    851b:	e8 f0 ad ff ff       	call   3310 <_ZNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEC1Ev@plt>
    8520:	48 8b 53 18          	mov    0x18(%rbx),%rdx
    8524:	48 8b 73 20          	mov    0x20(%rbx),%rsi
    8528:	48 8d 7c 24 30       	lea    0x30(%rsp),%rdi
    852d:	e8 6e ae ff ff       	call   33a0 <_ZSt16__ostream_insertIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_PKS3_l@plt>
    8532:	48 89 c7             	mov    %rax,%rdi
    8535:	48 8d 35 22 7c 00 00 	lea    0x7c22(%rip),%rsi        # 1015e <_fini+0xf3d>
    853c:	e8 df ab ff ff       	call   3120 <_ZStlsISt11char_traitsIcEERSt13basic_ostreamIcT_ES5_PKc@plt>
    8541:	bf 10 00 00 00       	mov    $0x10,%edi
    8546:	49 89 e5             	mov    %rsp,%r13
    8549:	e8 82 ab ff ff       	call   30d0 <__cxa_allocate_exception@plt>
    854e:	48 8d 74 24 38       	lea    0x38(%rsp),%rsi
    8553:	4c 89 ef             	mov    %r13,%rdi
    8556:	49 89 c4             	mov    %rax,%r12
    8559:	e8 22 ac ff ff       	call   3180 <_ZNKSt7__cxx1115basic_stringbufIcSt11char_traitsIcESaIcEE3strEv@plt>
    855e:	4c 89 ee             	mov    %r13,%rsi
    8561:	4c 89 e7             	mov    %r12,%rdi
    8564:	e8 27 ad ff ff       	call   3290 <_ZNSt13runtime_errorC1ERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEE@plt>
    8569:	48 8b 3c 24          	mov    (%rsp),%rdi
    856d:	48 8d 44 24 10       	lea    0x10(%rsp),%rax
    8572:	48 39 c7             	cmp    %rax,%rdi
    8575:	74 0e                	je     8585 <_ZNK8argparse8Argument42throw_required_arg_no_value_provided_errorEv+0x95>
    8577:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
    857c:	48 8d 70 01          	lea    0x1(%rax),%rsi
    8580:	e8 9b ad ff ff       	call   3320 <_ZdlPvm@plt>
    8585:	48 8b 15 3c ba 00 00 	mov    0xba3c(%rip),%rdx        # 13fc8 <_ZNSt13runtime_errorD1Ev@Base>
    858c:	48 8d 35 0d b4 00 00 	lea    0xb40d(%rip),%rsi        # 139a0 <_ZTISt13runtime_error@@Base>
    8593:	4c 89 e7             	mov    %r12,%rdi
    8596:	e8 85 ac ff ff       	call   3220 <__cxa_throw@plt>
    859b:	48 89 c3             	mov    %rax,%rbx
    859e:	eb 23                	jmp    85c3 <_ZNK8argparse8Argument42throw_required_arg_no_value_provided_errorEv+0xd3>
    85a0:	48 89 c3             	mov    %rax,%rbx
    85a3:	eb 05                	jmp    85aa <_ZNK8argparse8Argument42throw_required_arg_no_value_provided_errorEv+0xba>
    85a5:	48 89 c3             	mov    %rax,%rbx
    85a8:	eb 0e                	jmp    85b8 <_ZNK8argparse8Argument42throw_required_arg_no_value_provided_errorEv+0xc8>
    85aa:	48 8b 3c 24          	mov    (%rsp),%rdi
    85ae:	48 8d 44 24 10       	lea    0x10(%rsp),%rax
    85b3:	48 39 c7             	cmp    %rax,%rdi
    85b6:	75 1e                	jne    85d6 <_ZNK8argparse8Argument42throw_required_arg_no_value_provided_errorEv+0xe6>
    85b8:	4c 89 e7             	mov    %r12,%rdi
    85bb:	c5 f8 77             	vzeroupper 
    85be:	e8 ed ad ff ff       	call   33b0 <__cxa_free_exception@plt>
    85c3:	48 89 ef             	mov    %rbp,%rdi
    85c6:	c5 f8 77             	vzeroupper 
    85c9:	e8 b2 ad ff ff       	call   3380 <_ZNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEED1Ev@plt>
    85ce:	48 89 df             	mov    %rbx,%rdi
    85d1:	e8 8a ad ff ff       	call   3360 <_Unwind_Resume@plt>
    85d6:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
    85db:	48 8d 70 01          	lea    0x1(%rax),%rsi
    85df:	c5 f8 77             	vzeroupper 
    85e2:	e8 39 ad ff ff       	call   3320 <_ZdlPvm@plt>
    85e7:	eb cf                	jmp    85b8 <_ZNK8argparse8Argument42throw_required_arg_no_value_provided_errorEv+0xc8>
    85e9:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)

00000000000085f0 <_ZNK8argparse8Argument34throw_nargs_range_validation_errorEv>:
    85f0:	41 55                	push   %r13
    85f2:	41 54                	push   %r12
    85f4:	55                   	push   %rbp
    85f5:	53                   	push   %rbx
    85f6:	48 89 fb             	mov    %rdi,%rbx
    85f9:	48 81 ec b8 01 00 00 	sub    $0x1b8,%rsp
    8600:	4c 8d 64 24 20       	lea    0x20(%rsp),%r12
    8605:	4c 89 e7             	mov    %r12,%rdi
    8608:	64 48 8b 04 25 28 00 	mov    %fs:0x28,%rax
    860f:	00 00 
    8611:	48 89 84 24 a8 01 00 	mov    %rax,0x1a8(%rsp)
    8618:	00 
    8619:	31 c0                	xor    %eax,%eax
    861b:	e8 f0 ac ff ff       	call   3310 <_ZNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEC1Ev@plt>
    8620:	48 8b 53 18          	mov    0x18(%rbx),%rdx
    8624:	48 85 d2             	test   %rdx,%rdx
    8627:	0f 85 ee 00 00 00    	jne    871b <_ZNK8argparse8Argument34throw_nargs_range_validation_errorEv+0x12b>
    862d:	48 8d 6c 24 30       	lea    0x30(%rsp),%rbp
    8632:	4c 8b 83 c8 00 00 00 	mov    0xc8(%rbx),%r8
    8639:	48 8b b3 d0 00 00 00 	mov    0xd0(%rbx),%rsi
    8640:	4c 39 c6             	cmp    %r8,%rsi
    8643:	0f 84 10 01 00 00    	je     8759 <_ZNK8argparse8Argument34throw_nargs_range_validation_errorEv+0x169>
    8649:	48 83 fe ff          	cmp    $0xffffffffffffffff,%rsi
    864d:	48 89 ef             	mov    %rbp,%rdi
    8650:	4c 89 c6             	mov    %r8,%rsi
    8653:	0f 84 e7 00 00 00    	je     8740 <_ZNK8argparse8Argument34throw_nargs_range_validation_errorEv+0x150>
    8659:	e8 c2 a9 ff ff       	call   3020 <_ZNSo9_M_insertImEERSoT_@plt>
    865e:	ba 04 00 00 00       	mov    $0x4,%edx
    8663:	48 8d 35 21 7b 00 00 	lea    0x7b21(%rip),%rsi        # 1018b <_fini+0xf6a>
    866a:	48 89 c7             	mov    %rax,%rdi
    866d:	49 89 c5             	mov    %rax,%r13
    8670:	e8 2b ad ff ff       	call   33a0 <_ZSt16__ostream_insertIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_PKS3_l@plt>
    8675:	48 8b b3 d0 00 00 00 	mov    0xd0(%rbx),%rsi
    867c:	4c 89 ef             	mov    %r13,%rdi
    867f:	e8 9c a9 ff ff       	call   3020 <_ZNSo9_M_insertImEERSoT_@plt>
    8684:	ba 17 00 00 00       	mov    $0x17,%edx
    8689:	48 8d 35 e3 7a 00 00 	lea    0x7ae3(%rip),%rsi        # 10173 <_fini+0xf52>
    8690:	48 89 ef             	mov    %rbp,%rdi
    8693:	e8 08 ad ff ff       	call   33a0 <_ZSt16__ostream_insertIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_PKS3_l@plt>
    8698:	48 8b b3 b8 00 00 00 	mov    0xb8(%rbx),%rsi
    869f:	48 89 ef             	mov    %rbp,%rdi
    86a2:	48 2b b3 b0 00 00 00 	sub    0xb0(%rbx),%rsi
    86a9:	48 c1 fe 04          	sar    $0x4,%rsi
    86ad:	e8 6e a9 ff ff       	call   3020 <_ZNSo9_M_insertImEERSoT_@plt>
    86b2:	48 89 c7             	mov    %rax,%rdi
    86b5:	48 8d 35 ac 7a 00 00 	lea    0x7aac(%rip),%rsi        # 10168 <_fini+0xf47>
    86bc:	e8 5f aa ff ff       	call   3120 <_ZStlsISt11char_traitsIcEERSt13basic_ostreamIcT_ES5_PKc@plt>
    86c1:	bf 10 00 00 00       	mov    $0x10,%edi
    86c6:	48 89 e5             	mov    %rsp,%rbp
    86c9:	e8 02 aa ff ff       	call   30d0 <__cxa_allocate_exception@plt>
    86ce:	48 8d 74 24 38       	lea    0x38(%rsp),%rsi
    86d3:	48 89 ef             	mov    %rbp,%rdi
    86d6:	49 89 c5             	mov    %rax,%r13
    86d9:	e8 a2 aa ff ff       	call   3180 <_ZNKSt7__cxx1115basic_stringbufIcSt11char_traitsIcESaIcEE3strEv@plt>
    86de:	48 89 ee             	mov    %rbp,%rsi
    86e1:	4c 89 ef             	mov    %r13,%rdi
    86e4:	e8 a7 ab ff ff       	call   3290 <_ZNSt13runtime_errorC1ERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEE@plt>
    86e9:	48 8b 3c 24          	mov    (%rsp),%rdi
    86ed:	48 8d 44 24 10       	lea    0x10(%rsp),%rax
    86f2:	48 39 c7             	cmp    %rax,%rdi
    86f5:	74 0e                	je     8705 <_ZNK8argparse8Argument34throw_nargs_range_validation_errorEv+0x115>
    86f7:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
    86fc:	48 8d 70 01          	lea    0x1(%rax),%rsi
    8700:	e8 1b ac ff ff       	call   3320 <_ZdlPvm@plt>
    8705:	48 8b 15 bc b8 00 00 	mov    0xb8bc(%rip),%rdx        # 13fc8 <_ZNSt13runtime_errorD1Ev@Base>
    870c:	48 8d 35 8d b2 00 00 	lea    0xb28d(%rip),%rsi        # 139a0 <_ZTISt13runtime_error@@Base>
    8713:	4c 89 ef             	mov    %r13,%rdi
    8716:	e8 05 ab ff ff       	call   3220 <__cxa_throw@plt>
    871b:	48 8d 6c 24 30       	lea    0x30(%rsp),%rbp
    8720:	48 8b 73 20          	mov    0x20(%rbx),%rsi
    8724:	48 89 ef             	mov    %rbp,%rdi
    8727:	e8 74 ac ff ff       	call   33a0 <_ZSt16__ostream_insertIcSt11char_traitsIcEERSt13basic_ostreamIT_T0_ES6_PKS3_l@plt>
    872c:	48 89 c7             	mov    %rax,%rdi
    872f:	48 8d 35 80 7a 00 00 	lea    0x7a80(%rip),%rsi        # 101b6 <_fini+0xf95>
    8736:	e8 e5 a9 ff ff       	call   3120 <_ZStlsISt11char_traitsIcEERSt13basic_ostreamIcT_ES5_PKc@plt>
    873b:	e9 f2 fe ff ff       	jmp    8632 <_ZNK8argparse8Argument34throw_nargs_range_validation_errorEv+0x42>
    8740:	e8 db a8 ff ff       	call   3020 <_ZNSo9_M_insertImEERSoT_@plt>
    8745:	48 89 c7             	mov    %rax,%rdi
    8748:	48 8d 35 41 7a 00 00 	lea    0x7a41(%rip),%rsi        # 10190 <_fini+0xf6f>
    874f:	e8 cc a9 ff ff       	call   3120 <_ZStlsISt11char_traitsIcEERSt13basic_ostreamIcT_ES5_PKc@plt>
    8754:	e9 2b ff ff ff       	jmp    8684 <_ZNK8argparse8Argument34throw_nargs_range_validation_errorEv+0x94>
    8759:	48 89 ef             	mov    %rbp,%rdi
    875c:	e8 bf a8 ff ff       	call   3020 <_ZNSo9_M_insertImEERSoT_@plt>
    8761:	e9 1e ff ff ff       	jmp    8684 <_ZNK8argparse8Argument34throw_nargs_range_validation_errorEv+0x94>
    8766:	48 89 c5             	mov    %rax,%rbp
    8769:	eb 0a                	jmp    8775 <_ZNK8argparse8Argument34throw_nargs_range_validation_errorEv+0x185>
    876b:	48 89 c5             	mov    %rax,%rbp
    876e:	eb 13                	jmp    8783 <_ZNK8argparse8Argument34throw_nargs_range_validation_errorEv+0x193>
    8770:	48 89 c5             	mov    %rax,%rbp
    8773:	eb 19                	jmp    878e <_ZNK8argparse8Argument34throw_nargs_range_validation_errorEv+0x19e>
    8775:	48 8b 3c 24          	mov    (%rsp),%rdi
    8779:	48 8d 44 24 10       	lea    0x10(%rsp),%rax
    877e:	48 39 c7             	cmp    %rax,%rdi
    8781:	75 1e                	jne    87a1 <_ZNK8argparse8Argument34throw_nargs_range_validation_errorEv+0x1b1>
    8783:	4c 89 ef             	mov    %r13,%rdi
    8786:	c5 f8 77             	vzeroupper 
    8789:	e8 22 ac ff ff       	call   33b0 <__cxa_free_exception@plt>
    878e:	4c 89 e7             	mov    %r12,%rdi
    8791:	c5 f8 77             	vzeroupper 
    8794:	e8 e7 ab ff ff       	call   3380 <_ZNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEED1Ev@plt>
    8799:	48 89 ef             	mov    %rbp,%rdi
    879c:	e8 bf ab ff ff       	call   3360 <_Unwind_Resume@plt>
    87a1:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
    87a6:	48 8d 70 01          	lea    0x1(%rax),%rsi
    87aa:	c5 f8 77             	vzeroupper 
    87ad:	e8 6e ab ff ff       	call   3320 <_ZdlPvm@plt>
    87b2:	eb cf                	jmp    8783 <_ZNK8argparse8Argument34throw_nargs_range_validation_errorEv+0x193>
    87b4:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
    87bb:	00 00 00 
    87be:	66 90                	xchg   %ax,%ax

00000000000087c0 <_ZNSt17_Function_handlerIFvRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEZN8argparse14ArgumentParserC4ES5_S5_NS9_17default_argumentsEEUlRKT_E_E9_M_invokeERKSt9_Any_dataS7_>:
    87c0:	48 83 ec 08          	sub    $0x8,%rsp
    87c4:	48 8b 3f             	mov    (%rdi),%rdi
    87c7:	e8 a4 d8 ff ff       	call   6070 <_ZZN8argparse14ArgumentParserC4ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsEENKUlRKT_E_clIS6_EEDaSA_.isra.0>
    87cc:	0f 1f 40 00          	nopl   0x0(%rax)

00000000000087d0 <_ZN8argparse8Argument13default_valueIbEERS0_OT_>:
    87d0:	41 55                	push   %r13
    87d2:	49 89 f5             	mov    %rsi,%r13
    87d5:	41 54                	push   %r12
    87d7:	49 89 fc             	mov    %rdi,%r12
    87da:	55                   	push   %rbp
    87db:	48 8d 2d 0e 7b 00 00 	lea    0x7b0e(%rip),%rbp        # 102f0 <_fini+0x10cf>
    87e2:	53                   	push   %rbx
    87e3:	48 83 ec 48          	sub    $0x48,%rsp
    87e7:	64 48 8b 04 25 28 00 	mov    %fs:0x28,%rax
    87ee:	00 00 
    87f0:	48 89 44 24 38       	mov    %rax,0x38(%rsp)
    87f5:	31 c0                	xor    %eax,%eax
    87f7:	80 3e 00             	cmpb   $0x0,(%rsi)
    87fa:	48 8d 05 f4 7a 00 00 	lea    0x7af4(%rip),%rax        # 102f5 <_fini+0x10d4>
    8801:	48 0f 44 e8          	cmove  %rax,%rbp
    8805:	48 8d 5c 24 20       	lea    0x20(%rsp),%rbx
    880a:	48 89 ef             	mov    %rbp,%rdi
    880d:	48 89 5c 24 10       	mov    %rbx,0x10(%rsp)
    8812:	e8 19 ab ff ff       	call   3330 <strlen@plt>
    8817:	31 d2                	xor    %edx,%edx
    8819:	a8 04                	test   $0x4,%al
    881b:	74 0c                	je     8829 <_ZN8argparse8Argument13default_valueIbEERS0_OT_+0x59>
    881d:	8b 55 00             	mov    0x0(%rbp),%edx
    8820:	89 54 24 20          	mov    %edx,0x20(%rsp)
    8824:	ba 04 00 00 00       	mov    $0x4,%edx
    8829:	a8 02                	test   $0x2,%al
    882b:	0f 85 3f 01 00 00    	jne    8970 <_ZN8argparse8Argument13default_valueIbEERS0_OT_+0x1a0>
    8831:	a8 01                	test   $0x1,%al
    8833:	0f 85 27 01 00 00    	jne    8960 <_ZN8argparse8Argument13default_valueIbEERS0_OT_+0x190>
    8839:	48 89 44 24 18       	mov    %rax,0x18(%rsp)
    883e:	c6 44 04 20 00       	movb   $0x0,0x20(%rsp,%rax,1)
    8843:	49 8b 7c 24 58       	mov    0x58(%r12),%rdi
    8848:	48 8b 54 24 10       	mov    0x10(%rsp),%rdx
    884d:	48 39 da             	cmp    %rbx,%rdx
    8850:	0f 84 6a 01 00 00    	je     89c0 <_ZN8argparse8Argument13default_valueIbEERS0_OT_+0x1f0>
    8856:	49 8d 74 24 68       	lea    0x68(%r12),%rsi
    885b:	48 8b 4c 24 20       	mov    0x20(%rsp),%rcx
    8860:	48 8b 44 24 18       	mov    0x18(%rsp),%rax
    8865:	48 39 f7             	cmp    %rsi,%rdi
    8868:	0f 84 22 01 00 00    	je     8990 <_ZN8argparse8Argument13default_valueIbEERS0_OT_+0x1c0>
    886e:	c4 e1 f9 6e c8       	vmovq  %rax,%xmm1
    8873:	c4 e3 f1 22 c1 01    	vpinsrq $0x1,%rcx,%xmm1,%xmm0
    8879:	49 8b 74 24 68       	mov    0x68(%r12),%rsi
    887e:	49 89 54 24 58       	mov    %rdx,0x58(%r12)
    8883:	c4 c1 7a 7f 44 24 60 	vmovdqu %xmm0,0x60(%r12)
    888a:	48 85 ff             	test   %rdi,%rdi
    888d:	0f 84 14 01 00 00    	je     89a7 <_ZN8argparse8Argument13default_valueIbEERS0_OT_+0x1d7>
    8893:	48 89 7c 24 10       	mov    %rdi,0x10(%rsp)
    8898:	48 89 74 24 20       	mov    %rsi,0x20(%rsp)
    889d:	48 c7 44 24 18 00 00 	movq   $0x0,0x18(%rsp)
    88a4:	00 00 
    88a6:	c6 07 00             	movb   $0x0,(%rdi)
    88a9:	48 8b 7c 24 10       	mov    0x10(%rsp),%rdi
    88ae:	48 39 df             	cmp    %rbx,%rdi
    88b1:	74 0e                	je     88c1 <_ZN8argparse8Argument13default_valueIbEERS0_OT_+0xf1>
    88b3:	48 8b 44 24 20       	mov    0x20(%rsp),%rax
    88b8:	48 8d 70 01          	lea    0x1(%rax),%rsi
    88bc:	e8 5f aa ff ff       	call   3320 <_ZdlPvm@plt>
    88c1:	41 0f b6 55 00       	movzbl 0x0(%r13),%edx
    88c6:	49 8b 4c 24 48       	mov    0x48(%r12),%rcx
    88cb:	48 8d 05 5e d8 ff ff 	lea    -0x27a2(%rip),%rax        # 6130 <_ZNSt3any17_Manager_internalIbE9_S_manageENS_3_OpEPKS_PNS_4_ArgE>
    88d2:	48 c7 44 24 18 00 00 	movq   $0x0,0x18(%rsp)
    88d9:	00 00 
    88db:	48 89 44 24 10       	mov    %rax,0x10(%rsp)
    88e0:	88 54 24 18          	mov    %dl,0x18(%rsp)
    88e4:	49 8d 5c 24 48       	lea    0x48(%r12),%rbx
    88e9:	48 85 c9             	test   %rcx,%rcx
    88ec:	74 1a                	je     8908 <_ZN8argparse8Argument13default_valueIbEERS0_OT_+0x138>
    88ee:	31 d2                	xor    %edx,%edx
    88f0:	48 89 de             	mov    %rbx,%rsi
    88f3:	bf 03 00 00 00       	mov    $0x3,%edi
    88f8:	ff d1                	call   *%rcx
    88fa:	49 c7 44 24 48 00 00 	movq   $0x0,0x48(%r12)
    8901:	00 00 
    8903:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
    8908:	48 89 5c 24 08       	mov    %rbx,0x8(%rsp)
    890d:	48 8d 6c 24 10       	lea    0x10(%rsp),%rbp
    8912:	48 8d 54 24 08       	lea    0x8(%rsp),%rdx
    8917:	48 89 ee             	mov    %rbp,%rsi
    891a:	bf 04 00 00 00       	mov    $0x4,%edi
    891f:	ff d0                	call   *%rax
    8921:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
    8926:	48 85 c0             	test   %rax,%rax
    8929:	74 0c                	je     8937 <_ZN8argparse8Argument13default_valueIbEERS0_OT_+0x167>
    892b:	31 d2                	xor    %edx,%edx
    892d:	48 89 ee             	mov    %rbp,%rsi
    8930:	bf 03 00 00 00       	mov    $0x3,%edi
    8935:	ff d0                	call   *%rax
    8937:	48 8b 44 24 38       	mov    0x38(%rsp),%rax
    893c:	64 48 2b 04 25 28 00 	sub    %fs:0x28,%rax
    8943:	00 00 
    8945:	0f 85 c0 00 00 00    	jne    8a0b <_ZN8argparse8Argument13default_valueIbEERS0_OT_+0x23b>
    894b:	48 83 c4 48          	add    $0x48,%rsp
    894f:	5b                   	pop    %rbx
    8950:	5d                   	pop    %rbp
    8951:	4c 89 e0             	mov    %r12,%rax
    8954:	41 5c                	pop    %r12
    8956:	41 5d                	pop    %r13
    8958:	c3                   	ret    
    8959:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
    8960:	0f b6 4c 15 00       	movzbl 0x0(%rbp,%rdx,1),%ecx
    8965:	88 0c 13             	mov    %cl,(%rbx,%rdx,1)
    8968:	e9 cc fe ff ff       	jmp    8839 <_ZN8argparse8Argument13default_valueIbEERS0_OT_+0x69>
    896d:	0f 1f 00             	nopl   (%rax)
    8970:	0f b7 4c 15 00       	movzwl 0x0(%rbp,%rdx,1),%ecx
    8975:	66 89 0c 13          	mov    %cx,(%rbx,%rdx,1)
    8979:	48 83 c2 02          	add    $0x2,%rdx
    897d:	a8 01                	test   $0x1,%al
    897f:	0f 84 b4 fe ff ff    	je     8839 <_ZN8argparse8Argument13default_valueIbEERS0_OT_+0x69>
    8985:	eb d9                	jmp    8960 <_ZN8argparse8Argument13default_valueIbEERS0_OT_+0x190>
    8987:	66 0f 1f 84 00 00 00 	nopw   0x0(%rax,%rax,1)
    898e:	00 00 
    8990:	c4 e1 f9 6e d0       	vmovq  %rax,%xmm2
    8995:	49 89 54 24 58       	mov    %rdx,0x58(%r12)
    899a:	c4 e3 e9 22 c1 01    	vpinsrq $0x1,%rcx,%xmm2,%xmm0
    89a0:	c4 c1 7a 7f 44 24 60 	vmovdqu %xmm0,0x60(%r12)
    89a7:	48 89 5c 24 10       	mov    %rbx,0x10(%rsp)
    89ac:	48 8d 5c 24 20       	lea    0x20(%rsp),%rbx
    89b1:	48 89 df             	mov    %rbx,%rdi
    89b4:	e9 e4 fe ff ff       	jmp    889d <_ZN8argparse8Argument13default_valueIbEERS0_OT_+0xcd>
    89b9:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
    89c0:	48 8b 54 24 18       	mov    0x18(%rsp),%rdx
    89c5:	48 85 d2             	test   %rdx,%rdx
    89c8:	74 18                	je     89e2 <_ZN8argparse8Argument13default_valueIbEERS0_OT_+0x212>
    89ca:	48 83 fa 01          	cmp    $0x1,%rdx
    89ce:	74 28                	je     89f8 <_ZN8argparse8Argument13default_valueIbEERS0_OT_+0x228>
    89d0:	48 89 de             	mov    %rbx,%rsi
    89d3:	e8 68 a7 ff ff       	call   3140 <memcpy@plt>
    89d8:	48 8b 54 24 18       	mov    0x18(%rsp),%rdx
    89dd:	49 8b 7c 24 58       	mov    0x58(%r12),%rdi
    89e2:	49 89 54 24 60       	mov    %rdx,0x60(%r12)
    89e7:	c6 04 17 00          	movb   $0x0,(%rdi,%rdx,1)
    89eb:	48 8b 7c 24 10       	mov    0x10(%rsp),%rdi
    89f0:	e9 a8 fe ff ff       	jmp    889d <_ZN8argparse8Argument13default_valueIbEERS0_OT_+0xcd>
    89f5:	0f 1f 00             	nopl   (%rax)
    89f8:	0f b6 44 24 20       	movzbl 0x20(%rsp),%eax
    89fd:	88 07                	mov    %al,(%rdi)
    89ff:	48 8b 54 24 18       	mov    0x18(%rsp),%rdx
    8a04:	49 8b 7c 24 58       	mov    0x58(%r12),%rdi
    8a09:	eb d7                	jmp    89e2 <_ZN8argparse8Argument13default_valueIbEERS0_OT_+0x212>
    8a0b:	e8 40 a7 ff ff       	call   3150 <__stack_chk_fail@plt>

0000000000008a10 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EED1Ev>:
    8a10:	41 54                	push   %r12
    8a12:	49 89 fc             	mov    %rdi,%r12
    8a15:	55                   	push   %rbp
    8a16:	53                   	push   %rbx
    8a17:	48 8b 5f 08          	mov    0x8(%rdi),%rbx
    8a1b:	48 8b 2f             	mov    (%rdi),%rbp
    8a1e:	48 39 eb             	cmp    %rbp,%rbx
    8a21:	74 2c                	je     8a4f <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EED1Ev+0x3f>
    8a23:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)
    8a28:	48 8b 7d 00          	mov    0x0(%rbp),%rdi
    8a2c:	48 8d 45 10          	lea    0x10(%rbp),%rax
    8a30:	48 39 c7             	cmp    %rax,%rdi
    8a33:	74 3b                	je     8a70 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EED1Ev+0x60>
    8a35:	48 8b 45 10          	mov    0x10(%rbp),%rax
    8a39:	48 83 c5 20          	add    $0x20,%rbp
    8a3d:	48 8d 70 01          	lea    0x1(%rax),%rsi
    8a41:	e8 da a8 ff ff       	call   3320 <_ZdlPvm@plt>
    8a46:	48 39 eb             	cmp    %rbp,%rbx
    8a49:	75 dd                	jne    8a28 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EED1Ev+0x18>
    8a4b:	49 8b 2c 24          	mov    (%r12),%rbp
    8a4f:	48 85 ed             	test   %rbp,%rbp
    8a52:	74 2c                	je     8a80 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EED1Ev+0x70>
    8a54:	49 8b 74 24 10       	mov    0x10(%r12),%rsi
    8a59:	5b                   	pop    %rbx
    8a5a:	48 29 ee             	sub    %rbp,%rsi
    8a5d:	48 89 ef             	mov    %rbp,%rdi
    8a60:	5d                   	pop    %rbp
    8a61:	41 5c                	pop    %r12
    8a63:	e9 b8 a8 ff ff       	jmp    3320 <_ZdlPvm@plt>
    8a68:	0f 1f 84 00 00 00 00 	nopl   0x0(%rax,%rax,1)
    8a6f:	00 
    8a70:	48 83 c5 20          	add    $0x20,%rbp
    8a74:	48 39 eb             	cmp    %rbp,%rbx
    8a77:	75 af                	jne    8a28 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EED1Ev+0x18>
    8a79:	eb d0                	jmp    8a4b <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EED1Ev+0x3b>
    8a7b:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)
    8a80:	5b                   	pop    %rbx
    8a81:	5d                   	pop    %rbp
    8a82:	41 5c                	pop    %r12
    8a84:	c3                   	ret    
    8a85:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
    8a8c:	00 00 00 
    8a8f:	90                   	nop

0000000000008a90 <_ZStplIcSt11char_traitsIcESaIcEENSt7__cxx1112basic_stringIT_T0_T1_EEPKS5_RKS8_>:
    8a90:	41 56                	push   %r14
    8a92:	41 55                	push   %r13
    8a94:	41 54                	push   %r12
    8a96:	49 89 fc             	mov    %rdi,%r12
    8a99:	48 89 f7             	mov    %rsi,%rdi
    8a9c:	55                   	push   %rbp
    8a9d:	48 89 f5             	mov    %rsi,%rbp
    8aa0:	4d 8d 74 24 10       	lea    0x10(%r12),%r14
    8aa5:	53                   	push   %rbx
    8aa6:	48 89 d3             	mov    %rdx,%rbx
    8aa9:	e8 82 a8 ff ff       	call   3330 <strlen@plt>
    8aae:	49 c7 44 24 08 00 00 	movq   $0x0,0x8(%r12)
    8ab5:	00 00 
    8ab7:	41 c6 44 24 10 00    	movb   $0x0,0x10(%r12)
    8abd:	4d 89 34 24          	mov    %r14,(%r12)
    8ac1:	4c 89 e7             	mov    %r12,%rdi
    8ac4:	48 8b 73 08          	mov    0x8(%rbx),%rsi
    8ac8:	49 89 c5             	mov    %rax,%r13
    8acb:	48 01 c6             	add    %rax,%rsi
    8ace:	e8 7d a5 ff ff       	call   3050 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE7reserveEm@plt>
    8ad3:	48 b8 ff ff ff ff ff 	movabs $0x3fffffffffffffff,%rax
    8ada:	ff ff 3f 
    8add:	49 2b 44 24 08       	sub    0x8(%r12),%rax
    8ae2:	49 39 c5             	cmp    %rax,%r13
    8ae5:	77 29                	ja     8b10 <_ZStplIcSt11char_traitsIcESaIcEENSt7__cxx1112basic_stringIT_T0_T1_EEPKS5_RKS8_+0x80>
    8ae7:	4c 89 ea             	mov    %r13,%rdx
    8aea:	48 89 ee             	mov    %rbp,%rsi
    8aed:	4c 89 e7             	mov    %r12,%rdi
    8af0:	e8 eb a6 ff ff       	call   31e0 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE9_M_appendEPKcm@plt>
    8af5:	48 8b 53 08          	mov    0x8(%rbx),%rdx
    8af9:	48 8b 33             	mov    (%rbx),%rsi
    8afc:	4c 89 e7             	mov    %r12,%rdi
    8aff:	e8 dc a6 ff ff       	call   31e0 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE9_M_appendEPKcm@plt>
    8b04:	5b                   	pop    %rbx
    8b05:	5d                   	pop    %rbp
    8b06:	4c 89 e0             	mov    %r12,%rax
    8b09:	41 5c                	pop    %r12
    8b0b:	41 5d                	pop    %r13
    8b0d:	41 5e                	pop    %r14
    8b0f:	c3                   	ret    
    8b10:	48 8d 3d e4 77 00 00 	lea    0x77e4(%rip),%rdi        # 102fb <_fini+0x10da>
    8b17:	e8 24 a8 ff ff       	call   3340 <_ZSt20__throw_length_errorPKc@plt>
    8b1c:	48 89 c5             	mov    %rax,%rbp
    8b1f:	49 8b 3c 24          	mov    (%r12),%rdi
    8b23:	49 39 fe             	cmp    %rdi,%r14
    8b26:	74 18                	je     8b40 <_ZStplIcSt11char_traitsIcESaIcEENSt7__cxx1112basic_stringIT_T0_T1_EEPKS5_RKS8_+0xb0>
    8b28:	49 8b 74 24 10       	mov    0x10(%r12),%rsi
    8b2d:	48 ff c6             	inc    %rsi
    8b30:	c5 f8 77             	vzeroupper 
    8b33:	e8 e8 a7 ff ff       	call   3320 <_ZdlPvm@plt>
    8b38:	48 89 ef             	mov    %rbp,%rdi
    8b3b:	e8 20 a8 ff ff       	call   3360 <_Unwind_Resume@plt>
    8b40:	c5 f8 77             	vzeroupper 
    8b43:	eb f3                	jmp    8b38 <_ZStplIcSt11char_traitsIcESaIcEENSt7__cxx1112basic_stringIT_T0_T1_EEPKS5_RKS8_+0xa8>
    8b45:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
    8b4c:	00 00 00 
    8b4f:	90                   	nop

0000000000008b50 <_ZN8argparse8Argument13default_valueINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEERS0_OT_>:
    8b50:	41 57                	push   %r15
    8b52:	41 56                	push   %r14
    8b54:	41 55                	push   %r13
    8b56:	41 54                	push   %r12
    8b58:	49 89 fc             	mov    %rdi,%r12
    8b5b:	55                   	push   %rbp
    8b5c:	53                   	push   %rbx
    8b5d:	48 89 f3             	mov    %rsi,%rbx
    8b60:	48 81 ec 88 00 00 00 	sub    $0x88,%rsp
    8b67:	4c 8b 3e             	mov    (%rsi),%r15
    8b6a:	4c 8b 76 08          	mov    0x8(%rsi),%r14
    8b6e:	64 48 8b 04 25 28 00 	mov    %fs:0x28,%rax
    8b75:	00 00 
    8b77:	48 89 44 24 78       	mov    %rax,0x78(%rsp)
    8b7c:	31 c0                	xor    %eax,%eax
    8b7e:	4c 89 f8             	mov    %r15,%rax
    8b81:	4c 8d 6c 24 60       	lea    0x60(%rsp),%r13
    8b86:	4c 01 f0             	add    %r14,%rax
    8b89:	4c 89 6c 24 50       	mov    %r13,0x50(%rsp)
    8b8e:	74 09                	je     8b99 <_ZN8argparse8Argument13default_valueINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEERS0_OT_+0x49>
    8b90:	4d 85 ff             	test   %r15,%r15
    8b93:	0f 84 aa 03 00 00    	je     8f43 <_ZN8argparse8Argument13default_valueINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEERS0_OT_+0x3f3>
    8b99:	4c 89 74 24 08       	mov    %r14,0x8(%rsp)
    8b9e:	49 83 fe 0f          	cmp    $0xf,%r14
    8ba2:	0f 87 88 02 00 00    	ja     8e30 <_ZN8argparse8Argument13default_valueINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEERS0_OT_+0x2e0>
    8ba8:	49 83 fe 01          	cmp    $0x1,%r14
    8bac:	0f 85 66 02 00 00    	jne    8e18 <_ZN8argparse8Argument13default_valueINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEERS0_OT_+0x2c8>
    8bb2:	41 0f b6 07          	movzbl (%r15),%eax
    8bb6:	48 8d 6c 24 50       	lea    0x50(%rsp),%rbp
    8bbb:	88 44 24 60          	mov    %al,0x60(%rsp)
    8bbf:	4c 89 e8             	mov    %r13,%rax
    8bc2:	4c 89 74 24 58       	mov    %r14,0x58(%rsp)
    8bc7:	42 c6 04 30 00       	movb   $0x0,(%rax,%r14,1)
    8bcc:	41 b8 22 00 00 00    	mov    $0x22,%r8d
    8bd2:	b9 01 00 00 00       	mov    $0x1,%ecx
    8bd7:	31 d2                	xor    %edx,%edx
    8bd9:	31 f6                	xor    %esi,%esi
    8bdb:	48 89 ef             	mov    %rbp,%rdi
    8bde:	e8 9d a4 ff ff       	call   3080 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE14_M_replace_auxEmmmc@plt>
    8be3:	4c 8d 74 24 40       	lea    0x40(%rsp),%r14
    8be8:	4c 89 74 24 30       	mov    %r14,0x30(%rsp)
    8bed:	48 8d 50 10          	lea    0x10(%rax),%rdx
    8bf1:	48 8b 08             	mov    (%rax),%rcx
    8bf4:	48 39 d1             	cmp    %rdx,%rcx
    8bf7:	0f 84 73 02 00 00    	je     8e70 <_ZN8argparse8Argument13default_valueINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEERS0_OT_+0x320>
    8bfd:	48 89 4c 24 30       	mov    %rcx,0x30(%rsp)
    8c02:	48 8b 48 10          	mov    0x10(%rax),%rcx
    8c06:	48 89 4c 24 40       	mov    %rcx,0x40(%rsp)
    8c0b:	48 8b 48 08          	mov    0x8(%rax),%rcx
    8c0f:	c6 40 10 00          	movb   $0x0,0x10(%rax)
    8c13:	48 89 4c 24 38       	mov    %rcx,0x38(%rsp)
    8c18:	48 c7 40 08 00 00 00 	movq   $0x0,0x8(%rax)
    8c1f:	00 
    8c20:	48 89 10             	mov    %rdx,(%rax)
    8c23:	48 8d 7c 24 30       	lea    0x30(%rsp),%rdi
    8c28:	48 8b 74 24 38       	mov    0x38(%rsp),%rsi
    8c2d:	41 b8 22 00 00 00    	mov    $0x22,%r8d
    8c33:	b9 01 00 00 00       	mov    $0x1,%ecx
    8c38:	31 d2                	xor    %edx,%edx
    8c3a:	e8 41 a4 ff ff       	call   3080 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE14_M_replace_auxEmmmc@plt>
    8c3f:	4c 8d 7c 24 20       	lea    0x20(%rsp),%r15
    8c44:	4c 89 7c 24 10       	mov    %r15,0x10(%rsp)
    8c49:	48 8d 50 10          	lea    0x10(%rax),%rdx
    8c4d:	48 8b 08             	mov    (%rax),%rcx
    8c50:	48 39 d1             	cmp    %rdx,%rcx
    8c53:	0f 84 97 02 00 00    	je     8ef0 <_ZN8argparse8Argument13default_valueINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEERS0_OT_+0x3a0>
    8c59:	48 89 4c 24 10       	mov    %rcx,0x10(%rsp)
    8c5e:	48 8b 48 10          	mov    0x10(%rax),%rcx
    8c62:	48 89 4c 24 20       	mov    %rcx,0x20(%rsp)
    8c67:	48 8b 48 08          	mov    0x8(%rax),%rcx
    8c6b:	c6 40 10 00          	movb   $0x0,0x10(%rax)
    8c6f:	48 89 4c 24 18       	mov    %rcx,0x18(%rsp)
    8c74:	48 89 10             	mov    %rdx,(%rax)
    8c77:	48 c7 40 08 00 00 00 	movq   $0x0,0x8(%rax)
    8c7e:	00 
    8c7f:	48 8b 7c 24 30       	mov    0x30(%rsp),%rdi
    8c84:	4c 39 f7             	cmp    %r14,%rdi
    8c87:	74 0e                	je     8c97 <_ZN8argparse8Argument13default_valueINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEERS0_OT_+0x147>
    8c89:	48 8b 44 24 40       	mov    0x40(%rsp),%rax
    8c8e:	48 8d 70 01          	lea    0x1(%rax),%rsi
    8c92:	e8 89 a6 ff ff       	call   3320 <_ZdlPvm@plt>
    8c97:	48 8b 7c 24 50       	mov    0x50(%rsp),%rdi
    8c9c:	4c 39 ef             	cmp    %r13,%rdi
    8c9f:	74 0e                	je     8caf <_ZN8argparse8Argument13default_valueINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEERS0_OT_+0x15f>
    8ca1:	48 8b 44 24 60       	mov    0x60(%rsp),%rax
    8ca6:	48 8d 70 01          	lea    0x1(%rax),%rsi
    8caa:	e8 71 a6 ff ff       	call   3320 <_ZdlPvm@plt>
    8caf:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
    8cb4:	49 8b 7c 24 58       	mov    0x58(%r12),%rdi
    8cb9:	48 8b 54 24 18       	mov    0x18(%rsp),%rdx
    8cbe:	4c 39 f8             	cmp    %r15,%rax
    8cc1:	0f 84 f9 01 00 00    	je     8ec0 <_ZN8argparse8Argument13default_valueINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEERS0_OT_+0x370>
    8cc7:	49 8d 74 24 68       	lea    0x68(%r12),%rsi
    8ccc:	48 8b 4c 24 20       	mov    0x20(%rsp),%rcx
    8cd1:	48 39 f7             	cmp    %rsi,%rdi
    8cd4:	0f 84 26 02 00 00    	je     8f00 <_ZN8argparse8Argument13default_valueINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEERS0_OT_+0x3b0>
    8cda:	c4 e1 f9 6e ca       	vmovq  %rdx,%xmm1
    8cdf:	c4 e3 f1 22 c1 01    	vpinsrq $0x1,%rcx,%xmm1,%xmm0
    8ce5:	49 8b 74 24 68       	mov    0x68(%r12),%rsi
    8cea:	49 89 44 24 58       	mov    %rax,0x58(%r12)
    8cef:	c4 c1 7a 7f 44 24 60 	vmovdqu %xmm0,0x60(%r12)
    8cf6:	48 85 ff             	test   %rdi,%rdi
    8cf9:	0f 84 18 02 00 00    	je     8f17 <_ZN8argparse8Argument13default_valueINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEERS0_OT_+0x3c7>
    8cff:	48 89 7c 24 10       	mov    %rdi,0x10(%rsp)
    8d04:	48 89 74 24 20       	mov    %rsi,0x20(%rsp)
    8d09:	48 c7 44 24 18 00 00 	movq   $0x0,0x18(%rsp)
    8d10:	00 00 
    8d12:	c6 07 00             	movb   $0x0,(%rdi)
    8d15:	48 8b 7c 24 10       	mov    0x10(%rsp),%rdi
    8d1a:	4c 39 ff             	cmp    %r15,%rdi
    8d1d:	74 0e                	je     8d2d <_ZN8argparse8Argument13default_valueINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEERS0_OT_+0x1dd>
    8d1f:	48 8b 44 24 20       	mov    0x20(%rsp),%rax
    8d24:	48 8d 70 01          	lea    0x1(%rax),%rsi
    8d28:	e8 f3 a5 ff ff       	call   3320 <_ZdlPvm@plt>
    8d2d:	48 8d 05 5c d8 ff ff 	lea    -0x27a4(%rip),%rax        # 6590 <_ZNSt3any17_Manager_externalINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEE9_S_manageENS_3_OpEPKS_PNS_4_ArgE>
    8d34:	bf 20 00 00 00       	mov    $0x20,%edi
    8d39:	48 89 44 24 50       	mov    %rax,0x50(%rsp)
    8d3e:	48 c7 44 24 58 00 00 	movq   $0x0,0x58(%rsp)
    8d45:	00 00 
    8d47:	e8 54 a5 ff ff       	call   32a0 <_Znwm@plt>
    8d4c:	48 8d 50 10          	lea    0x10(%rax),%rdx
    8d50:	48 8b 0b             	mov    (%rbx),%rcx
    8d53:	48 89 10             	mov    %rdx,(%rax)
    8d56:	48 8d 53 10          	lea    0x10(%rbx),%rdx
    8d5a:	4d 8d 6c 24 48       	lea    0x48(%r12),%r13
    8d5f:	48 39 d1             	cmp    %rdx,%rcx
    8d62:	0f 84 48 01 00 00    	je     8eb0 <_ZN8argparse8Argument13default_valueINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEERS0_OT_+0x360>
    8d68:	48 89 08             	mov    %rcx,(%rax)
    8d6b:	48 8b 4b 10          	mov    0x10(%rbx),%rcx
    8d6f:	48 89 48 10          	mov    %rcx,0x10(%rax)
    8d73:	48 8b 4b 08          	mov    0x8(%rbx),%rcx
    8d77:	48 89 44 24 58       	mov    %rax,0x58(%rsp)
    8d7c:	48 89 48 08          	mov    %rcx,0x8(%rax)
    8d80:	48 8b 44 24 50       	mov    0x50(%rsp),%rax
    8d85:	48 89 13             	mov    %rdx,(%rbx)
    8d88:	48 c7 43 08 00 00 00 	movq   $0x0,0x8(%rbx)
    8d8f:	00 
    8d90:	c6 43 10 00          	movb   $0x0,0x10(%rbx)
    8d94:	48 85 c0             	test   %rax,%rax
    8d97:	0f 84 e3 00 00 00    	je     8e80 <_ZN8argparse8Argument13default_valueINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEERS0_OT_+0x330>
    8d9d:	49 8b 4c 24 48       	mov    0x48(%r12),%rcx
    8da2:	48 85 c9             	test   %rcx,%rcx
    8da5:	74 1a                	je     8dc1 <_ZN8argparse8Argument13default_valueINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEERS0_OT_+0x271>
    8da7:	31 d2                	xor    %edx,%edx
    8da9:	4c 89 ee             	mov    %r13,%rsi
    8dac:	bf 03 00 00 00       	mov    $0x3,%edi
    8db1:	ff d1                	call   *%rcx
    8db3:	49 c7 44 24 48 00 00 	movq   $0x0,0x48(%r12)
    8dba:	00 00 
    8dbc:	48 8b 44 24 50       	mov    0x50(%rsp),%rax
    8dc1:	4c 89 6c 24 08       	mov    %r13,0x8(%rsp)
    8dc6:	48 8d 54 24 08       	lea    0x8(%rsp),%rdx
    8dcb:	48 89 ee             	mov    %rbp,%rsi
    8dce:	bf 04 00 00 00       	mov    $0x4,%edi
    8dd3:	ff d0                	call   *%rax
    8dd5:	48 8b 44 24 50       	mov    0x50(%rsp),%rax
    8dda:	48 85 c0             	test   %rax,%rax
    8ddd:	74 0c                	je     8deb <_ZN8argparse8Argument13default_valueINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEERS0_OT_+0x29b>
    8ddf:	31 d2                	xor    %edx,%edx
    8de1:	48 89 ee             	mov    %rbp,%rsi
    8de4:	bf 03 00 00 00       	mov    $0x3,%edi
    8de9:	ff d0                	call   *%rax
    8deb:	48 8b 44 24 78       	mov    0x78(%rsp),%rax
    8df0:	64 48 2b 04 25 28 00 	sub    %fs:0x28,%rax
    8df7:	00 00 
    8df9:	0f 85 50 01 00 00    	jne    8f4f <_ZN8argparse8Argument13default_valueINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEERS0_OT_+0x3ff>
    8dff:	48 81 c4 88 00 00 00 	add    $0x88,%rsp
    8e06:	5b                   	pop    %rbx
    8e07:	5d                   	pop    %rbp
    8e08:	4c 89 e0             	mov    %r12,%rax
    8e0b:	41 5c                	pop    %r12
    8e0d:	41 5d                	pop    %r13
    8e0f:	41 5e                	pop    %r14
    8e11:	41 5f                	pop    %r15
    8e13:	c3                   	ret    
    8e14:	0f 1f 40 00          	nopl   0x0(%rax)
    8e18:	4d 85 f6             	test   %r14,%r14
    8e1b:	0f 85 33 01 00 00    	jne    8f54 <_ZN8argparse8Argument13default_valueINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEERS0_OT_+0x404>
    8e21:	4c 89 e8             	mov    %r13,%rax
    8e24:	48 8d 6c 24 50       	lea    0x50(%rsp),%rbp
    8e29:	e9 94 fd ff ff       	jmp    8bc2 <_ZN8argparse8Argument13default_valueINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEERS0_OT_+0x72>
    8e2e:	66 90                	xchg   %ax,%ax
    8e30:	48 8d 6c 24 50       	lea    0x50(%rsp),%rbp
    8e35:	48 89 ef             	mov    %rbp,%rdi
    8e38:	48 8d 74 24 08       	lea    0x8(%rsp),%rsi
    8e3d:	31 d2                	xor    %edx,%edx
    8e3f:	e8 bc a4 ff ff       	call   3300 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE9_M_createERmm@plt>
    8e44:	48 89 44 24 50       	mov    %rax,0x50(%rsp)
    8e49:	48 89 c7             	mov    %rax,%rdi
    8e4c:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
    8e51:	48 89 44 24 60       	mov    %rax,0x60(%rsp)
    8e56:	4c 89 f2             	mov    %r14,%rdx
    8e59:	4c 89 fe             	mov    %r15,%rsi
    8e5c:	e8 df a2 ff ff       	call   3140 <memcpy@plt>
    8e61:	4c 8b 74 24 08       	mov    0x8(%rsp),%r14
    8e66:	48 8b 44 24 50       	mov    0x50(%rsp),%rax
    8e6b:	e9 52 fd ff ff       	jmp    8bc2 <_ZN8argparse8Argument13default_valueINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEERS0_OT_+0x72>
    8e70:	c5 fa 6f 58 10       	vmovdqu 0x10(%rax),%xmm3
    8e75:	c5 f9 7f 5c 24 40    	vmovdqa %xmm3,0x40(%rsp)
    8e7b:	e9 8b fd ff ff       	jmp    8c0b <_ZN8argparse8Argument13default_valueINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEERS0_OT_+0xbb>
    8e80:	49 8b 44 24 48       	mov    0x48(%r12),%rax
    8e85:	48 85 c0             	test   %rax,%rax
    8e88:	0f 84 5d ff ff ff    	je     8deb <_ZN8argparse8Argument13default_valueINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEERS0_OT_+0x29b>
    8e8e:	31 d2                	xor    %edx,%edx
    8e90:	4c 89 ee             	mov    %r13,%rsi
    8e93:	bf 03 00 00 00       	mov    $0x3,%edi
    8e98:	ff d0                	call   *%rax
    8e9a:	49 c7 44 24 48 00 00 	movq   $0x0,0x48(%r12)
    8ea1:	00 00 
    8ea3:	e9 2d ff ff ff       	jmp    8dd5 <_ZN8argparse8Argument13default_valueINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEERS0_OT_+0x285>
    8ea8:	0f 1f 84 00 00 00 00 	nopl   0x0(%rax,%rax,1)
    8eaf:	00 
    8eb0:	c5 fa 6f 6b 10       	vmovdqu 0x10(%rbx),%xmm5
    8eb5:	c5 fa 7f 68 10       	vmovdqu %xmm5,0x10(%rax)
    8eba:	e9 b4 fe ff ff       	jmp    8d73 <_ZN8argparse8Argument13default_valueINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEERS0_OT_+0x223>
    8ebf:	90                   	nop
    8ec0:	48 85 d2             	test   %rdx,%rdx
    8ec3:	74 18                	je     8edd <_ZN8argparse8Argument13default_valueINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEERS0_OT_+0x38d>
    8ec5:	48 83 fa 01          	cmp    $0x1,%rdx
    8ec9:	74 65                	je     8f30 <_ZN8argparse8Argument13default_valueINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEERS0_OT_+0x3e0>
    8ecb:	4c 89 fe             	mov    %r15,%rsi
    8ece:	e8 6d a2 ff ff       	call   3140 <memcpy@plt>
    8ed3:	48 8b 54 24 18       	mov    0x18(%rsp),%rdx
    8ed8:	49 8b 7c 24 58       	mov    0x58(%r12),%rdi
    8edd:	49 89 54 24 60       	mov    %rdx,0x60(%r12)
    8ee2:	c6 04 17 00          	movb   $0x0,(%rdi,%rdx,1)
    8ee6:	48 8b 7c 24 10       	mov    0x10(%rsp),%rdi
    8eeb:	e9 19 fe ff ff       	jmp    8d09 <_ZN8argparse8Argument13default_valueINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEERS0_OT_+0x1b9>
    8ef0:	c5 fa 6f 60 10       	vmovdqu 0x10(%rax),%xmm4
    8ef5:	c5 f9 7f 64 24 20    	vmovdqa %xmm4,0x20(%rsp)
    8efb:	e9 67 fd ff ff       	jmp    8c67 <_ZN8argparse8Argument13default_valueINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEERS0_OT_+0x117>
    8f00:	c4 e1 f9 6e d2       	vmovq  %rdx,%xmm2
    8f05:	49 89 44 24 58       	mov    %rax,0x58(%r12)
    8f0a:	c4 e3 e9 22 c1 01    	vpinsrq $0x1,%rcx,%xmm2,%xmm0
    8f10:	c4 c1 7a 7f 44 24 60 	vmovdqu %xmm0,0x60(%r12)
    8f17:	4c 89 7c 24 10       	mov    %r15,0x10(%rsp)
    8f1c:	4c 8d 7c 24 20       	lea    0x20(%rsp),%r15
    8f21:	4c 89 ff             	mov    %r15,%rdi
    8f24:	e9 e0 fd ff ff       	jmp    8d09 <_ZN8argparse8Argument13default_valueINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEERS0_OT_+0x1b9>
    8f29:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
    8f30:	0f b6 44 24 20       	movzbl 0x20(%rsp),%eax
    8f35:	88 07                	mov    %al,(%rdi)
    8f37:	48 8b 54 24 18       	mov    0x18(%rsp),%rdx
    8f3c:	49 8b 7c 24 58       	mov    0x58(%r12),%rdi
    8f41:	eb 9a                	jmp    8edd <_ZN8argparse8Argument13default_valueINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEERS0_OT_+0x38d>
    8f43:	48 8d 3d b6 70 00 00 	lea    0x70b6(%rip),%rdi        # 10000 <_fini+0xddf>
    8f4a:	e8 71 a4 ff ff       	call   33c0 <_ZSt19__throw_logic_errorPKc@plt>
    8f4f:	e8 fc a1 ff ff       	call   3150 <__stack_chk_fail@plt>
    8f54:	4c 89 ef             	mov    %r13,%rdi
    8f57:	48 8d 6c 24 50       	lea    0x50(%rsp),%rbp
    8f5c:	e9 f5 fe ff ff       	jmp    8e56 <_ZN8argparse8Argument13default_valueINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEERS0_OT_+0x306>
    8f61:	48 89 c5             	mov    %rax,%rbp
    8f64:	eb 1e                	jmp    8f84 <_ZN8argparse8Argument13default_valueINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEERS0_OT_+0x434>
    8f66:	48 89 c5             	mov    %rax,%rbp
    8f69:	48 8b 7c 24 30       	mov    0x30(%rsp),%rdi
    8f6e:	4c 39 f7             	cmp    %r14,%rdi
    8f71:	74 11                	je     8f84 <_ZN8argparse8Argument13default_valueINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEERS0_OT_+0x434>
    8f73:	48 8b 44 24 40       	mov    0x40(%rsp),%rax
    8f78:	48 8d 70 01          	lea    0x1(%rax),%rsi
    8f7c:	c5 f8 77             	vzeroupper 
    8f7f:	e8 9c a3 ff ff       	call   3320 <_ZdlPvm@plt>
    8f84:	48 8b 7c 24 50       	mov    0x50(%rsp),%rdi
    8f89:	4c 39 ef             	cmp    %r13,%rdi
    8f8c:	74 19                	je     8fa7 <_ZN8argparse8Argument13default_valueINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEERS0_OT_+0x457>
    8f8e:	48 8b 44 24 60       	mov    0x60(%rsp),%rax
    8f93:	48 8d 70 01          	lea    0x1(%rax),%rsi
    8f97:	c5 f8 77             	vzeroupper 
    8f9a:	e8 81 a3 ff ff       	call   3320 <_ZdlPvm@plt>
    8f9f:	48 89 ef             	mov    %rbp,%rdi
    8fa2:	e8 b9 a3 ff ff       	call   3360 <_Unwind_Resume@plt>
    8fa7:	c5 f8 77             	vzeroupper 
    8faa:	eb f3                	jmp    8f9f <_ZN8argparse8Argument13default_valueINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEERS0_OT_+0x44f>
    8fac:	0f 1f 40 00          	nopl   0x0(%rax)

0000000000008fb0 <_ZNSt7__cxx1110_List_baseIN8argparse8ArgumentESaIS2_EE8_M_clearEv>:
    8fb0:	41 57                	push   %r15
    8fb2:	41 56                	push   %r14
    8fb4:	41 55                	push   %r13
    8fb6:	41 54                	push   %r12
    8fb8:	55                   	push   %rbp
    8fb9:	53                   	push   %rbx
    8fba:	48 83 ec 28          	sub    $0x28,%rsp
    8fbe:	48 89 7c 24 08       	mov    %rdi,0x8(%rsp)
    8fc3:	48 8b 2f             	mov    (%rdi),%rbp
    8fc6:	64 48 8b 0c 25 28 00 	mov    %fs:0x28,%rcx
    8fcd:	00 00 
    8fcf:	48 89 4c 24 18       	mov    %rcx,0x18(%rsp)
    8fd4:	31 c9                	xor    %ecx,%ecx
    8fd6:	48 39 fd             	cmp    %rdi,%rbp
    8fd9:	0f 84 89 01 00 00    	je     9168 <_ZNSt7__cxx1110_List_baseIN8argparse8ArgumentESaIS2_EE8_M_clearEv+0x1b8>
    8fdf:	4c 8d 7c 24 17       	lea    0x17(%rsp),%r15
    8fe4:	4c 8d 35 a5 a8 00 00 	lea    0xa8a5(%rip),%r14        # 13890 <_ZNSt8__detail9__variant12__gen_vtableIvOZNS0_16_Variant_storageILb0EJSt8functionIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEES3_IFvSC_EEEE8_M_resetEvEUlOT_E_JRSt7variantIJSE_SG_EEEE9_S_vtableE>
    8feb:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)
    8ff0:	48 8b 9d c8 00 00 00 	mov    0xc8(%rbp),%rbx
    8ff7:	4c 8b ad c0 00 00 00 	mov    0xc0(%rbp),%r13
    8ffe:	4c 8b 65 00          	mov    0x0(%rbp),%r12
    9002:	4c 39 eb             	cmp    %r13,%rbx
    9005:	74 32                	je     9039 <_ZNSt7__cxx1110_List_baseIN8argparse8ArgumentESaIS2_EE8_M_clearEv+0x89>
    9007:	66 0f 1f 84 00 00 00 	nopw   0x0(%rax,%rax,1)
    900e:	00 00 
    9010:	49 8b 45 00          	mov    0x0(%r13),%rax
    9014:	48 85 c0             	test   %rax,%rax
    9017:	0f 84 33 01 00 00    	je     9150 <_ZNSt7__cxx1110_List_baseIN8argparse8ArgumentESaIS2_EE8_M_clearEv+0x1a0>
    901d:	4c 89 ee             	mov    %r13,%rsi
    9020:	31 d2                	xor    %edx,%edx
    9022:	bf 03 00 00 00       	mov    $0x3,%edi
    9027:	49 83 c5 10          	add    $0x10,%r13
    902b:	ff d0                	call   *%rax
    902d:	4c 39 eb             	cmp    %r13,%rbx
    9030:	75 de                	jne    9010 <_ZNSt7__cxx1110_List_baseIN8argparse8ArgumentESaIS2_EE8_M_clearEv+0x60>
    9032:	4c 8b ad c0 00 00 00 	mov    0xc0(%rbp),%r13
    9039:	4d 85 ed             	test   %r13,%r13
    903c:	74 12                	je     9050 <_ZNSt7__cxx1110_List_baseIN8argparse8ArgumentESaIS2_EE8_M_clearEv+0xa0>
    903e:	48 8b b5 d0 00 00 00 	mov    0xd0(%rbp),%rsi
    9045:	4c 89 ef             	mov    %r13,%rdi
    9048:	4c 29 ee             	sub    %r13,%rsi
    904b:	e8 d0 a2 ff ff       	call   3320 <_ZdlPvm@plt>
    9050:	0f b6 85 b8 00 00 00 	movzbl 0xb8(%rbp),%eax
    9057:	48 8d b5 98 00 00 00 	lea    0x98(%rbp),%rsi
    905e:	4c 89 ff             	mov    %r15,%rdi
    9061:	41 ff 14 c6          	call   *(%r14,%rax,8)
    9065:	48 8b 85 88 00 00 00 	mov    0x88(%rbp),%rax
    906c:	48 85 c0             	test   %rax,%rax
    906f:	74 10                	je     9081 <_ZNSt7__cxx1110_List_baseIN8argparse8ArgumentESaIS2_EE8_M_clearEv+0xd1>
    9071:	48 8d b5 88 00 00 00 	lea    0x88(%rbp),%rsi
    9078:	31 d2                	xor    %edx,%edx
    907a:	bf 03 00 00 00       	mov    $0x3,%edi
    907f:	ff d0                	call   *%rax
    9081:	48 8b 7d 68          	mov    0x68(%rbp),%rdi
    9085:	48 8d 45 78          	lea    0x78(%rbp),%rax
    9089:	48 39 c7             	cmp    %rax,%rdi
    908c:	74 0d                	je     909b <_ZNSt7__cxx1110_List_baseIN8argparse8ArgumentESaIS2_EE8_M_clearEv+0xeb>
    908e:	48 8b 45 78          	mov    0x78(%rbp),%rax
    9092:	48 8d 70 01          	lea    0x1(%rax),%rsi
    9096:	e8 85 a2 ff ff       	call   3320 <_ZdlPvm@plt>
    909b:	48 8b 45 58          	mov    0x58(%rbp),%rax
    909f:	48 85 c0             	test   %rax,%rax
    90a2:	74 0d                	je     90b1 <_ZNSt7__cxx1110_List_baseIN8argparse8ArgumentESaIS2_EE8_M_clearEv+0x101>
    90a4:	48 8d 75 58          	lea    0x58(%rbp),%rsi
    90a8:	31 d2                	xor    %edx,%edx
    90aa:	bf 03 00 00 00       	mov    $0x3,%edi
    90af:	ff d0                	call   *%rax
    90b1:	48 8b 7d 38          	mov    0x38(%rbp),%rdi
    90b5:	48 8d 45 48          	lea    0x48(%rbp),%rax
    90b9:	48 39 c7             	cmp    %rax,%rdi
    90bc:	74 0d                	je     90cb <_ZNSt7__cxx1110_List_baseIN8argparse8ArgumentESaIS2_EE8_M_clearEv+0x11b>
    90be:	48 8b 45 48          	mov    0x48(%rbp),%rax
    90c2:	48 8d 70 01          	lea    0x1(%rax),%rsi
    90c6:	e8 55 a2 ff ff       	call   3320 <_ZdlPvm@plt>
    90cb:	48 8b 5d 18          	mov    0x18(%rbp),%rbx
    90cf:	4c 8b 6d 10          	mov    0x10(%rbp),%r13
    90d3:	4c 39 eb             	cmp    %r13,%rbx
    90d6:	74 2f                	je     9107 <_ZNSt7__cxx1110_List_baseIN8argparse8ArgumentESaIS2_EE8_M_clearEv+0x157>
    90d8:	0f 1f 84 00 00 00 00 	nopl   0x0(%rax,%rax,1)
    90df:	00 
    90e0:	49 8b 7d 00          	mov    0x0(%r13),%rdi
    90e4:	49 8d 45 10          	lea    0x10(%r13),%rax
    90e8:	48 39 c7             	cmp    %rax,%rdi
    90eb:	74 53                	je     9140 <_ZNSt7__cxx1110_List_baseIN8argparse8ArgumentESaIS2_EE8_M_clearEv+0x190>
    90ed:	49 8b 45 10          	mov    0x10(%r13),%rax
    90f1:	49 83 c5 20          	add    $0x20,%r13
    90f5:	48 8d 70 01          	lea    0x1(%rax),%rsi
    90f9:	e8 22 a2 ff ff       	call   3320 <_ZdlPvm@plt>
    90fe:	4c 39 eb             	cmp    %r13,%rbx
    9101:	75 dd                	jne    90e0 <_ZNSt7__cxx1110_List_baseIN8argparse8ArgumentESaIS2_EE8_M_clearEv+0x130>
    9103:	4c 8b 6d 10          	mov    0x10(%rbp),%r13
    9107:	4d 85 ed             	test   %r13,%r13
    910a:	74 0f                	je     911b <_ZNSt7__cxx1110_List_baseIN8argparse8ArgumentESaIS2_EE8_M_clearEv+0x16b>
    910c:	48 8b 75 20          	mov    0x20(%rbp),%rsi
    9110:	4c 89 ef             	mov    %r13,%rdi
    9113:	4c 29 ee             	sub    %r13,%rsi
    9116:	e8 05 a2 ff ff       	call   3320 <_ZdlPvm@plt>
    911b:	be f0 00 00 00       	mov    $0xf0,%esi
    9120:	48 89 ef             	mov    %rbp,%rdi
    9123:	e8 f8 a1 ff ff       	call   3320 <_ZdlPvm@plt>
    9128:	4c 3b 64 24 08       	cmp    0x8(%rsp),%r12
    912d:	74 39                	je     9168 <_ZNSt7__cxx1110_List_baseIN8argparse8ArgumentESaIS2_EE8_M_clearEv+0x1b8>
    912f:	4c 89 e5             	mov    %r12,%rbp
    9132:	e9 b9 fe ff ff       	jmp    8ff0 <_ZNSt7__cxx1110_List_baseIN8argparse8ArgumentESaIS2_EE8_M_clearEv+0x40>
    9137:	66 0f 1f 84 00 00 00 	nopw   0x0(%rax,%rax,1)
    913e:	00 00 
    9140:	49 83 c5 20          	add    $0x20,%r13
    9144:	4c 39 eb             	cmp    %r13,%rbx
    9147:	75 97                	jne    90e0 <_ZNSt7__cxx1110_List_baseIN8argparse8ArgumentESaIS2_EE8_M_clearEv+0x130>
    9149:	eb b8                	jmp    9103 <_ZNSt7__cxx1110_List_baseIN8argparse8ArgumentESaIS2_EE8_M_clearEv+0x153>
    914b:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)
    9150:	49 83 c5 10          	add    $0x10,%r13
    9154:	4c 39 eb             	cmp    %r13,%rbx
    9157:	0f 85 b3 fe ff ff    	jne    9010 <_ZNSt7__cxx1110_List_baseIN8argparse8ArgumentESaIS2_EE8_M_clearEv+0x60>
    915d:	e9 d0 fe ff ff       	jmp    9032 <_ZNSt7__cxx1110_List_baseIN8argparse8ArgumentESaIS2_EE8_M_clearEv+0x82>
    9162:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)
    9168:	48 8b 44 24 18       	mov    0x18(%rsp),%rax
    916d:	64 48 2b 04 25 28 00 	sub    %fs:0x28,%rax
    9174:	00 00 
    9176:	75 0f                	jne    9187 <_ZNSt7__cxx1110_List_baseIN8argparse8ArgumentESaIS2_EE8_M_clearEv+0x1d7>
    9178:	48 83 c4 28          	add    $0x28,%rsp
    917c:	5b                   	pop    %rbx
    917d:	5d                   	pop    %rbp
    917e:	41 5c                	pop    %r12
    9180:	41 5d                	pop    %r13
    9182:	41 5e                	pop    %r14
    9184:	41 5f                	pop    %r15
    9186:	c3                   	ret    
    9187:	e8 c4 9f ff ff       	call   3150 <__stack_chk_fail@plt>
    918c:	0f 1f 40 00          	nopl   0x0(%rax)

0000000000009190 <_ZN8argparse14ArgumentParserD1Ev>:
    9190:	55                   	push   %rbp
    9191:	48 89 fd             	mov    %rdi,%rbp
    9194:	53                   	push   %rbx
    9195:	48 83 ec 08          	sub    $0x8,%rsp
    9199:	48 8b 9f c8 00 00 00 	mov    0xc8(%rdi),%rbx
    91a0:	48 85 db             	test   %rbx,%rbx
    91a3:	74 1f                	je     91c4 <_ZN8argparse14ArgumentParserD1Ev+0x34>
    91a5:	48 8b 7b 18          	mov    0x18(%rbx),%rdi
    91a9:	e8 72 b5 ff ff       	call   4720 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE8_M_eraseEPSt13_Rb_tree_nodeISA_E.isra.0>
    91ae:	48 89 df             	mov    %rbx,%rdi
    91b1:	48 8b 5b 10          	mov    0x10(%rbx),%rbx
    91b5:	be 38 00 00 00       	mov    $0x38,%esi
    91ba:	e8 61 a1 ff ff       	call   3320 <_ZdlPvm@plt>
    91bf:	48 85 db             	test   %rbx,%rbx
    91c2:	75 e1                	jne    91a5 <_ZN8argparse14ArgumentParserD1Ev+0x15>
    91c4:	48 8d bd a0 00 00 00 	lea    0xa0(%rbp),%rdi
    91cb:	e8 e0 fd ff ff       	call   8fb0 <_ZNSt7__cxx1110_List_baseIN8argparse8ArgumentESaIS2_EE8_M_clearEv>
    91d0:	48 8d bd 88 00 00 00 	lea    0x88(%rbp),%rdi
    91d7:	e8 d4 fd ff ff       	call   8fb0 <_ZNSt7__cxx1110_List_baseIN8argparse8ArgumentESaIS2_EE8_M_clearEv>
    91dc:	48 8b 7d 60          	mov    0x60(%rbp),%rdi
    91e0:	48 8d 45 70          	lea    0x70(%rbp),%rax
    91e4:	48 39 c7             	cmp    %rax,%rdi
    91e7:	74 0d                	je     91f6 <_ZN8argparse14ArgumentParserD1Ev+0x66>
    91e9:	48 8b 45 70          	mov    0x70(%rbp),%rax
    91ed:	48 8d 70 01          	lea    0x1(%rax),%rsi
    91f1:	e8 2a a1 ff ff       	call   3320 <_ZdlPvm@plt>
    91f6:	48 8b 7d 40          	mov    0x40(%rbp),%rdi
    91fa:	48 8d 45 50          	lea    0x50(%rbp),%rax
    91fe:	48 39 c7             	cmp    %rax,%rdi
    9201:	74 0d                	je     9210 <_ZN8argparse14ArgumentParserD1Ev+0x80>
    9203:	48 8b 45 50          	mov    0x50(%rbp),%rax
    9207:	48 8d 70 01          	lea    0x1(%rax),%rsi
    920b:	e8 10 a1 ff ff       	call   3320 <_ZdlPvm@plt>
    9210:	48 8b 7d 20          	mov    0x20(%rbp),%rdi
    9214:	48 8d 45 30          	lea    0x30(%rbp),%rax
    9218:	48 39 c7             	cmp    %rax,%rdi
    921b:	74 0d                	je     922a <_ZN8argparse14ArgumentParserD1Ev+0x9a>
    921d:	48 8b 45 30          	mov    0x30(%rbp),%rax
    9221:	48 8d 70 01          	lea    0x1(%rax),%rsi
    9225:	e8 f6 a0 ff ff       	call   3320 <_ZdlPvm@plt>
    922a:	48 8b 7d 00          	mov    0x0(%rbp),%rdi
    922e:	48 8d 45 10          	lea    0x10(%rbp),%rax
    9232:	48 39 c7             	cmp    %rax,%rdi
    9235:	74 19                	je     9250 <_ZN8argparse14ArgumentParserD1Ev+0xc0>
    9237:	48 8b 75 10          	mov    0x10(%rbp),%rsi
    923b:	48 83 c4 08          	add    $0x8,%rsp
    923f:	5b                   	pop    %rbx
    9240:	48 ff c6             	inc    %rsi
    9243:	5d                   	pop    %rbp
    9244:	e9 d7 a0 ff ff       	jmp    3320 <_ZdlPvm@plt>
    9249:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
    9250:	48 83 c4 08          	add    $0x8,%rsp
    9254:	5b                   	pop    %rbx
    9255:	5d                   	pop    %rbp
    9256:	c3                   	ret    
    9257:	66 0f 1f 84 00 00 00 	nopw   0x0(%rax,%rax,1)
    925e:	00 00 

0000000000009260 <_ZN8argparse7details4reprIiEENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEERKT_>:
    9260:	55                   	push   %rbp
    9261:	48 89 e5             	mov    %rsp,%rbp
    9264:	41 57                	push   %r15
    9266:	41 56                	push   %r14
    9268:	41 55                	push   %r13
    926a:	41 54                	push   %r12
    926c:	49 89 fc             	mov    %rdi,%r12
    926f:	53                   	push   %rbx
    9270:	48 8d 1d 71 a6 00 00 	lea    0xa671(%rip),%rbx        # 138e8 <_ZTVSt9basic_iosIcSt11char_traitsIcEE+0x10>
    9277:	48 83 e4 e0          	and    $0xffffffffffffffe0,%rsp
    927b:	48 81 ec e0 01 00 00 	sub    $0x1e0,%rsp
    9282:	48 89 74 24 38       	mov    %rsi,0x38(%rsp)
    9287:	4c 8d ac 24 c0 00 00 	lea    0xc0(%rsp),%r13
    928e:	00 
    928f:	4c 8d 74 24 40       	lea    0x40(%rsp),%r14
    9294:	4c 89 ef             	mov    %r13,%rdi
    9297:	64 48 8b 04 25 28 00 	mov    %fs:0x28,%rax
    929e:	00 00 
    92a0:	48 89 84 24 d8 01 00 	mov    %rax,0x1d8(%rsp)
    92a7:	00 
    92a8:	31 c0                	xor    %eax,%eax
    92aa:	4c 89 74 24 20       	mov    %r14,0x20(%rsp)
    92af:	e8 4c 9e ff ff       	call   3100 <_ZNSt8ios_baseC2Ev@plt>
    92b4:	31 c0                	xor    %eax,%eax
    92b6:	66 89 84 24 a0 01 00 	mov    %ax,0x1a0(%rsp)
    92bd:	00 
    92be:	4c 8b 3d 0b a7 00 00 	mov    0xa70b(%rip),%r15        # 139d0 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x10>
    92c5:	c5 f9 ef c0          	vpxor  %xmm0,%xmm0,%xmm0
    92c9:	c5 fe 7f 84 24 a8 01 	vmovdqu %ymm0,0x1a8(%rsp)
    92d0:	00 00 
    92d2:	49 8b 47 e8          	mov    -0x18(%r15),%rax
    92d6:	48 8b 15 fb a6 00 00 	mov    0xa6fb(%rip),%rdx        # 139d8 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x18>
    92dd:	48 89 9c 24 c0 00 00 	mov    %rbx,0xc0(%rsp)
    92e4:	00 
    92e5:	48 c7 84 24 98 01 00 	movq   $0x0,0x198(%rsp)
    92ec:	00 00 00 00 00 
    92f1:	4c 89 7c 24 40       	mov    %r15,0x40(%rsp)
    92f6:	48 89 54 04 40       	mov    %rdx,0x40(%rsp,%rax,1)
    92fb:	48 c7 44 24 48 00 00 	movq   $0x0,0x48(%rsp)
    9302:	00 00 
    9304:	31 f6                	xor    %esi,%esi
    9306:	49 8b 7f e8          	mov    -0x18(%r15),%rdi
    930a:	4c 01 f7             	add    %r14,%rdi
    930d:	c5 f8 77             	vzeroupper 
    9310:	e8 1b 9d ff ff       	call   3030 <_ZNSt9basic_iosIcSt11char_traitsIcEE4initEPSt15basic_streambufIcS1_E@plt>
    9315:	48 8b 05 c4 a6 00 00 	mov    0xa6c4(%rip),%rax        # 139e0 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x20>
    931c:	4c 8d 74 24 50       	lea    0x50(%rsp),%r14
    9321:	48 8b 78 e8          	mov    -0x18(%rax),%rdi
    9325:	48 89 44 24 50       	mov    %rax,0x50(%rsp)
    932a:	48 8b 05 b7 a6 00 00 	mov    0xa6b7(%rip),%rax        # 139e8 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x28>
    9331:	4c 01 f7             	add    %r14,%rdi
    9334:	48 89 07             	mov    %rax,(%rdi)
    9337:	31 f6                	xor    %esi,%esi
    9339:	e8 f2 9c ff ff       	call   3030 <_ZNSt9basic_iosIcSt11char_traitsIcEE4initEPSt15basic_streambufIcS1_E@plt>
    933e:	48 8b 05 83 a6 00 00 	mov    0xa683(%rip),%rax        # 139c8 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x8>
    9345:	48 8b 15 a4 a6 00 00 	mov    0xa6a4(%rip),%rdx        # 139f0 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x30>
    934c:	48 8b 40 e8          	mov    -0x18(%rax),%rax
    9350:	48 89 54 04 40       	mov    %rdx,0x40(%rsp,%rax,1)
    9355:	48 8d 05 5c a7 00 00 	lea    0xa75c(%rip),%rax        # 13ab8 <_ZTVNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x18>
    935c:	48 89 44 24 40       	mov    %rax,0x40(%rsp)
    9361:	48 83 c0 50          	add    $0x50,%rax
    9365:	48 89 84 24 c0 00 00 	mov    %rax,0xc0(%rsp)
    936c:	00 
    936d:	48 83 e8 28          	sub    $0x28,%rax
    9371:	48 89 44 24 50       	mov    %rax,0x50(%rsp)
    9376:	48 8d 05 b3 a5 00 00 	lea    0xa5b3(%rip),%rax        # 13930 <_ZTVSt15basic_streambufIcSt11char_traitsIcEE+0x10>
    937d:	48 89 44 24 58       	mov    %rax,0x58(%rsp)
    9382:	48 8d 84 24 90 00 00 	lea    0x90(%rsp),%rax
    9389:	00 
    938a:	48 89 c7             	mov    %rax,%rdi
    938d:	48 89 44 24 30       	mov    %rax,0x30(%rsp)
    9392:	48 c7 44 24 60 00 00 	movq   $0x0,0x60(%rsp)
    9399:	00 00 
    939b:	48 c7 44 24 68 00 00 	movq   $0x0,0x68(%rsp)
    93a2:	00 00 
    93a4:	48 c7 44 24 70 00 00 	movq   $0x0,0x70(%rsp)
    93ab:	00 00 
    93ad:	48 c7 44 24 78 00 00 	movq   $0x0,0x78(%rsp)
    93b4:	00 00 
    93b6:	48 c7 84 24 80 00 00 	movq   $0x0,0x80(%rsp)
    93bd:	00 00 00 00 00 
    93c2:	48 c7 84 24 88 00 00 	movq   $0x0,0x88(%rsp)
    93c9:	00 00 00 00 00 
    93ce:	e8 3d 9d ff ff       	call   3110 <_ZNSt6localeC1Ev@plt>
    93d3:	48 8d 05 46 a6 00 00 	lea    0xa646(%rip),%rax        # 13a20 <_ZTVNSt7__cxx1115basic_stringbufIcSt11char_traitsIcESaIcEEE+0x10>
    93da:	48 89 44 24 58       	mov    %rax,0x58(%rsp)
    93df:	48 8d 84 24 b0 00 00 	lea    0xb0(%rsp),%rax
    93e6:	00 
    93e7:	48 89 44 24 28       	mov    %rax,0x28(%rsp)
    93ec:	48 89 84 24 a0 00 00 	mov    %rax,0xa0(%rsp)
    93f3:	00 
    93f4:	48 8d 44 24 58       	lea    0x58(%rsp),%rax
    93f9:	48 89 c6             	mov    %rax,%rsi
    93fc:	4c 89 ef             	mov    %r13,%rdi
    93ff:	c7 84 24 98 00 00 00 	movl   $0x18,0x98(%rsp)
    9406:	18 00 00 00 
    940a:	48 c7 84 24 a8 00 00 	movq   $0x0,0xa8(%rsp)
    9411:	00 00 00 00 00 
    9416:	c6 84 24 b0 00 00 00 	movb   $0x0,0xb0(%rsp)
    941d:	00 
    941e:	48 89 44 24 18       	mov    %rax,0x18(%rsp)
    9423:	e8 08 9c ff ff       	call   3030 <_ZNSt9basic_iosIcSt11char_traitsIcEE4initEPSt15basic_streambufIcS1_E@plt>
    9428:	48 8b 44 24 38       	mov    0x38(%rsp),%rax
    942d:	4c 89 f7             	mov    %r14,%rdi
    9430:	8b 30                	mov    (%rax),%esi
    9432:	e8 19 9f ff ff       	call   3350 <_ZNSolsEi@plt>
    9437:	48 8b 84 24 80 00 00 	mov    0x80(%rsp),%rax
    943e:	00 
    943f:	4d 8d 74 24 10       	lea    0x10(%r12),%r14
    9444:	4d 89 34 24          	mov    %r14,(%r12)
    9448:	49 c7 44 24 08 00 00 	movq   $0x0,0x8(%r12)
    944f:	00 00 
    9451:	41 c6 44 24 10 00    	movb   $0x0,0x10(%r12)
    9457:	48 85 c0             	test   %rax,%rax
    945a:	0f 84 30 01 00 00    	je     9590 <_ZN8argparse7details4reprIiEENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEERKT_+0x330>
    9460:	4c 8b 44 24 70       	mov    0x70(%rsp),%r8
    9465:	48 8b 4c 24 78       	mov    0x78(%rsp),%rcx
    946a:	4c 39 c0             	cmp    %r8,%rax
    946d:	0f 87 05 01 00 00    	ja     9578 <_ZN8argparse7details4reprIiEENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEERKT_+0x318>
    9473:	49 29 c8             	sub    %rcx,%r8
    9476:	31 d2                	xor    %edx,%edx
    9478:	31 f6                	xor    %esi,%esi
    947a:	4c 89 e7             	mov    %r12,%rdi
    947d:	e8 2e 9d ff ff       	call   31b0 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE10_M_replaceEmmPKcm@plt>
    9482:	48 8d 05 2f a6 00 00 	lea    0xa62f(%rip),%rax        # 13ab8 <_ZTVNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x18>
    9489:	48 89 44 24 40       	mov    %rax,0x40(%rsp)
    948e:	c5 fa 7e 0d 12 a7 00 	vmovq  0xa712(%rip),%xmm1        # 13ba8 <_ZTVNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE@@Base+0x108>
    9495:	00 
    9496:	48 83 c0 50          	add    $0x50,%rax
    949a:	48 89 84 24 c0 00 00 	mov    %rax,0xc0(%rsp)
    94a1:	00 
    94a2:	48 8b bc 24 a0 00 00 	mov    0xa0(%rsp),%rdi
    94a9:	00 
    94aa:	48 8d 05 6f a5 00 00 	lea    0xa56f(%rip),%rax        # 13a20 <_ZTVNSt7__cxx1115basic_stringbufIcSt11char_traitsIcESaIcEEE+0x10>
    94b1:	c4 e3 f1 22 c0 01    	vpinsrq $0x1,%rax,%xmm1,%xmm0
    94b7:	c5 f9 7f 44 24 50    	vmovdqa %xmm0,0x50(%rsp)
    94bd:	48 3b 7c 24 28       	cmp    0x28(%rsp),%rdi
    94c2:	74 11                	je     94d5 <_ZN8argparse7details4reprIiEENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEERKT_+0x275>
    94c4:	48 8b 84 24 b0 00 00 	mov    0xb0(%rsp),%rax
    94cb:	00 
    94cc:	48 8d 70 01          	lea    0x1(%rax),%rsi
    94d0:	e8 4b 9e ff ff       	call   3320 <_ZdlPvm@plt>
    94d5:	48 8b 7c 24 30       	mov    0x30(%rsp),%rdi
    94da:	48 8d 05 4f a4 00 00 	lea    0xa44f(%rip),%rax        # 13930 <_ZTVSt15basic_streambufIcSt11char_traitsIcEE+0x10>
    94e1:	48 89 44 24 58       	mov    %rax,0x58(%rsp)
    94e6:	e8 45 9d ff ff       	call   3230 <_ZNSt6localeD1Ev@plt>
    94eb:	48 8b 05 d6 a4 00 00 	mov    0xa4d6(%rip),%rax        # 139c8 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x8>
    94f2:	48 8b 0d f7 a4 00 00 	mov    0xa4f7(%rip),%rcx        # 139f0 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x30>
    94f9:	48 8b 40 e8          	mov    -0x18(%rax),%rax
    94fd:	48 8b 15 e4 a4 00 00 	mov    0xa4e4(%rip),%rdx        # 139e8 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x28>
    9504:	48 89 4c 04 40       	mov    %rcx,0x40(%rsp,%rax,1)
    9509:	48 8b 05 d0 a4 00 00 	mov    0xa4d0(%rip),%rax        # 139e0 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x20>
    9510:	48 8b 0d c1 a4 00 00 	mov    0xa4c1(%rip),%rcx        # 139d8 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x18>
    9517:	48 89 44 24 50       	mov    %rax,0x50(%rsp)
    951c:	48 8b 40 e8          	mov    -0x18(%rax),%rax
    9520:	4c 89 ef             	mov    %r13,%rdi
    9523:	48 89 54 04 50       	mov    %rdx,0x50(%rsp,%rax,1)
    9528:	49 8b 47 e8          	mov    -0x18(%r15),%rax
    952c:	4c 89 7c 24 40       	mov    %r15,0x40(%rsp)
    9531:	48 89 4c 04 40       	mov    %rcx,0x40(%rsp,%rax,1)
    9536:	48 c7 44 24 48 00 00 	movq   $0x0,0x48(%rsp)
    953d:	00 00 
    953f:	48 89 9c 24 c0 00 00 	mov    %rbx,0xc0(%rsp)
    9546:	00 
    9547:	e8 54 9b ff ff       	call   30a0 <_ZNSt8ios_baseD2Ev@plt>
    954c:	48 8b 84 24 d8 01 00 	mov    0x1d8(%rsp),%rax
    9553:	00 
    9554:	64 48 2b 04 25 28 00 	sub    %fs:0x28,%rax
    955b:	00 00 
    955d:	75 46                	jne    95a5 <_ZN8argparse7details4reprIiEENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEERKT_+0x345>
    955f:	48 8d 65 d8          	lea    -0x28(%rbp),%rsp
    9563:	5b                   	pop    %rbx
    9564:	4c 89 e0             	mov    %r12,%rax
    9567:	41 5c                	pop    %r12
    9569:	41 5d                	pop    %r13
    956b:	41 5e                	pop    %r14
    956d:	41 5f                	pop    %r15
    956f:	5d                   	pop    %rbp
    9570:	c3                   	ret    
    9571:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
    9578:	48 29 c8             	sub    %rcx,%rax
    957b:	49 89 c0             	mov    %rax,%r8
    957e:	31 d2                	xor    %edx,%edx
    9580:	31 f6                	xor    %esi,%esi
    9582:	4c 89 e7             	mov    %r12,%rdi
    9585:	e8 26 9c ff ff       	call   31b0 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE10_M_replaceEmmPKcm@plt>
    958a:	e9 f3 fe ff ff       	jmp    9482 <_ZN8argparse7details4reprIiEENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEERKT_+0x222>
    958f:	90                   	nop
    9590:	48 8d b4 24 a0 00 00 	lea    0xa0(%rsp),%rsi
    9597:	00 
    9598:	4c 89 e7             	mov    %r12,%rdi
    959b:	e8 00 9c ff ff       	call   31a0 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE9_M_assignERKS4_@plt>
    95a0:	e9 dd fe ff ff       	jmp    9482 <_ZN8argparse7details4reprIiEENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEERKT_+0x222>
    95a5:	e8 a6 9b ff ff       	call   3150 <__stack_chk_fail@plt>
    95aa:	49 89 c4             	mov    %rax,%r12
    95ad:	eb 34                	jmp    95e3 <_ZN8argparse7details4reprIiEENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEERKT_+0x383>
    95af:	49 89 c4             	mov    %rax,%r12
    95b2:	eb 44                	jmp    95f8 <_ZN8argparse7details4reprIiEENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEERKT_+0x398>
    95b4:	49 89 c4             	mov    %rax,%r12
    95b7:	e9 9a 00 00 00       	jmp    9656 <_ZN8argparse7details4reprIiEENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEERKT_+0x3f6>
    95bc:	48 89 c2             	mov    %rax,%rdx
    95bf:	e9 ad 00 00 00       	jmp    9671 <_ZN8argparse7details4reprIiEENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEERKT_+0x411>
    95c4:	48 89 c3             	mov    %rax,%rbx
    95c7:	49 8b 3c 24          	mov    (%r12),%rdi
    95cb:	49 39 fe             	cmp    %rdi,%r14
    95ce:	74 10                	je     95e0 <_ZN8argparse7details4reprIiEENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEERKT_+0x380>
    95d0:	49 8b 74 24 10       	mov    0x10(%r12),%rsi
    95d5:	48 ff c6             	inc    %rsi
    95d8:	c5 f8 77             	vzeroupper 
    95db:	e8 40 9d ff ff       	call   3320 <_ZdlPvm@plt>
    95e0:	49 89 dc             	mov    %rbx,%r12
    95e3:	48 8b 7c 24 20       	mov    0x20(%rsp),%rdi
    95e8:	c5 f8 77             	vzeroupper 
    95eb:	e8 90 9d ff ff       	call   3380 <_ZNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEED1Ev@plt>
    95f0:	4c 89 e7             	mov    %r12,%rdi
    95f3:	e8 68 9d ff ff       	call   3360 <_Unwind_Resume@plt>
    95f8:	48 8b 7c 24 18       	mov    0x18(%rsp),%rdi
    95fd:	c5 f8 77             	vzeroupper 
    9600:	e8 5b ed ff ff       	call   8360 <_ZNSt7__cxx1115basic_stringbufIcSt11char_traitsIcESaIcEED1Ev>
    9605:	48 8b 05 bc a3 00 00 	mov    0xa3bc(%rip),%rax        # 139c8 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x8>
    960c:	48 8b 15 dd a3 00 00 	mov    0xa3dd(%rip),%rdx        # 139f0 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x30>
    9613:	48 8b 40 e8          	mov    -0x18(%rax),%rax
    9617:	48 89 54 04 40       	mov    %rdx,0x40(%rsp,%rax,1)
    961c:	48 8b 05 bd a3 00 00 	mov    0xa3bd(%rip),%rax        # 139e0 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x20>
    9623:	48 8b 15 be a3 00 00 	mov    0xa3be(%rip),%rdx        # 139e8 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x28>
    962a:	48 89 44 24 50       	mov    %rax,0x50(%rsp)
    962f:	48 8b 40 e8          	mov    -0x18(%rax),%rax
    9633:	48 89 54 04 50       	mov    %rdx,0x50(%rsp,%rax,1)
    9638:	49 8b 47 e8          	mov    -0x18(%r15),%rax
    963c:	48 8b 15 95 a3 00 00 	mov    0xa395(%rip),%rdx        # 139d8 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x18>
    9643:	4c 89 7c 24 40       	mov    %r15,0x40(%rsp)
    9648:	48 89 54 04 40       	mov    %rdx,0x40(%rsp,%rax,1)
    964d:	48 c7 44 24 48 00 00 	movq   $0x0,0x48(%rsp)
    9654:	00 00 
    9656:	4c 89 ef             	mov    %r13,%rdi
    9659:	48 89 9c 24 c0 00 00 	mov    %rbx,0xc0(%rsp)
    9660:	00 
    9661:	c5 f8 77             	vzeroupper 
    9664:	e8 37 9a ff ff       	call   30a0 <_ZNSt8ios_baseD2Ev@plt>
    9669:	4c 89 e7             	mov    %r12,%rdi
    966c:	e8 ef 9c ff ff       	call   3360 <_Unwind_Resume@plt>
    9671:	49 8b 47 e8          	mov    -0x18(%r15),%rax
    9675:	48 8b 0d 5c a3 00 00 	mov    0xa35c(%rip),%rcx        # 139d8 <_ZTTNSt7__cxx1118basic_stringstreamIcSt11char_traitsIcESaIcEEE+0x18>
    967c:	4c 89 7c 24 40       	mov    %r15,0x40(%rsp)
    9681:	49 89 d4             	mov    %rdx,%r12
    9684:	48 89 4c 04 40       	mov    %rcx,0x40(%rsp,%rax,1)
    9689:	48 c7 44 24 48 00 00 	movq   $0x0,0x48(%rsp)
    9690:	00 00 
    9692:	eb c2                	jmp    9656 <_ZN8argparse7details4reprIiEENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEERKT_+0x3f6>
    9694:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
    969b:	00 00 00 
    969e:	66 90                	xchg   %ax,%ax

00000000000096a0 <_ZN8argparse8Argument13default_valueIiEERS0_OT_>:
    96a0:	41 55                	push   %r13
    96a2:	41 54                	push   %r12
    96a4:	49 89 fc             	mov    %rdi,%r12
    96a7:	55                   	push   %rbp
    96a8:	53                   	push   %rbx
    96a9:	48 89 f3             	mov    %rsi,%rbx
    96ac:	48 83 ec 48          	sub    $0x48,%rsp
    96b0:	48 8d 6c 24 10       	lea    0x10(%rsp),%rbp
    96b5:	48 89 ef             	mov    %rbp,%rdi
    96b8:	64 48 8b 04 25 28 00 	mov    %fs:0x28,%rax
    96bf:	00 00 
    96c1:	48 89 44 24 38       	mov    %rax,0x38(%rsp)
    96c6:	31 c0                	xor    %eax,%eax
    96c8:	e8 93 fb ff ff       	call   9260 <_ZN8argparse7details4reprIiEENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEERKT_>
    96cd:	48 8b 54 24 10       	mov    0x10(%rsp),%rdx
    96d2:	4c 8d 6c 24 20       	lea    0x20(%rsp),%r13
    96d7:	49 8b 7c 24 58       	mov    0x58(%r12),%rdi
    96dc:	4c 39 ea             	cmp    %r13,%rdx
    96df:	0f 84 2b 01 00 00    	je     9810 <_ZN8argparse8Argument13default_valueIiEERS0_OT_+0x170>
    96e5:	49 8d 74 24 68       	lea    0x68(%r12),%rsi
    96ea:	48 8b 4c 24 20       	mov    0x20(%rsp),%rcx
    96ef:	48 8b 44 24 18       	mov    0x18(%rsp),%rax
    96f4:	48 39 f7             	cmp    %rsi,%rdi
    96f7:	0f 84 e3 00 00 00    	je     97e0 <_ZN8argparse8Argument13default_valueIiEERS0_OT_+0x140>
    96fd:	c4 e1 f9 6e c8       	vmovq  %rax,%xmm1
    9702:	c4 e3 f1 22 c1 01    	vpinsrq $0x1,%rcx,%xmm1,%xmm0
    9708:	49 8b 74 24 68       	mov    0x68(%r12),%rsi
    970d:	49 89 54 24 58       	mov    %rdx,0x58(%r12)
    9712:	c4 c1 7a 7f 44 24 60 	vmovdqu %xmm0,0x60(%r12)
    9719:	48 85 ff             	test   %rdi,%rdi
    971c:	0f 84 d5 00 00 00    	je     97f7 <_ZN8argparse8Argument13default_valueIiEERS0_OT_+0x157>
    9722:	48 89 7c 24 10       	mov    %rdi,0x10(%rsp)
    9727:	48 89 74 24 20       	mov    %rsi,0x20(%rsp)
    972c:	48 c7 44 24 18 00 00 	movq   $0x0,0x18(%rsp)
    9733:	00 00 
    9735:	c6 07 00             	movb   $0x0,(%rdi)
    9738:	48 8b 7c 24 10       	mov    0x10(%rsp),%rdi
    973d:	4c 39 ef             	cmp    %r13,%rdi
    9740:	74 0e                	je     9750 <_ZN8argparse8Argument13default_valueIiEERS0_OT_+0xb0>
    9742:	48 8b 44 24 20       	mov    0x20(%rsp),%rax
    9747:	48 8d 70 01          	lea    0x1(%rax),%rsi
    974b:	e8 d0 9b ff ff       	call   3320 <_ZdlPvm@plt>
    9750:	8b 13                	mov    (%rbx),%edx
    9752:	49 8b 4c 24 48       	mov    0x48(%r12),%rcx
    9757:	48 8d 05 a2 ca ff ff 	lea    -0x355e(%rip),%rax        # 6200 <_ZNSt3any17_Manager_internalIiE9_S_manageENS_3_OpEPKS_PNS_4_ArgE>
    975e:	48 c7 44 24 18 00 00 	movq   $0x0,0x18(%rsp)
    9765:	00 00 
    9767:	48 89 44 24 10       	mov    %rax,0x10(%rsp)
    976c:	89 54 24 18          	mov    %edx,0x18(%rsp)
    9770:	4d 8d 6c 24 48       	lea    0x48(%r12),%r13
    9775:	48 85 c9             	test   %rcx,%rcx
    9778:	74 1a                	je     9794 <_ZN8argparse8Argument13default_valueIiEERS0_OT_+0xf4>
    977a:	31 d2                	xor    %edx,%edx
    977c:	4c 89 ee             	mov    %r13,%rsi
    977f:	bf 03 00 00 00       	mov    $0x3,%edi
    9784:	ff d1                	call   *%rcx
    9786:	49 c7 44 24 48 00 00 	movq   $0x0,0x48(%r12)
    978d:	00 00 
    978f:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
    9794:	4c 89 6c 24 08       	mov    %r13,0x8(%rsp)
    9799:	48 8d 54 24 08       	lea    0x8(%rsp),%rdx
    979e:	48 89 ee             	mov    %rbp,%rsi
    97a1:	bf 04 00 00 00       	mov    $0x4,%edi
    97a6:	ff d0                	call   *%rax
    97a8:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
    97ad:	48 85 c0             	test   %rax,%rax
    97b0:	74 0c                	je     97be <_ZN8argparse8Argument13default_valueIiEERS0_OT_+0x11e>
    97b2:	31 d2                	xor    %edx,%edx
    97b4:	48 89 ee             	mov    %rbp,%rsi
    97b7:	bf 03 00 00 00       	mov    $0x3,%edi
    97bc:	ff d0                	call   *%rax
    97be:	48 8b 44 24 38       	mov    0x38(%rsp),%rax
    97c3:	64 48 2b 04 25 28 00 	sub    %fs:0x28,%rax
    97ca:	00 00 
    97cc:	0f 85 89 00 00 00    	jne    985b <_ZN8argparse8Argument13default_valueIiEERS0_OT_+0x1bb>
    97d2:	48 83 c4 48          	add    $0x48,%rsp
    97d6:	5b                   	pop    %rbx
    97d7:	5d                   	pop    %rbp
    97d8:	4c 89 e0             	mov    %r12,%rax
    97db:	41 5c                	pop    %r12
    97dd:	41 5d                	pop    %r13
    97df:	c3                   	ret    
    97e0:	c4 e1 f9 6e d0       	vmovq  %rax,%xmm2
    97e5:	49 89 54 24 58       	mov    %rdx,0x58(%r12)
    97ea:	c4 e3 e9 22 c1 01    	vpinsrq $0x1,%rcx,%xmm2,%xmm0
    97f0:	c4 c1 7a 7f 44 24 60 	vmovdqu %xmm0,0x60(%r12)
    97f7:	4c 89 6c 24 10       	mov    %r13,0x10(%rsp)
    97fc:	4c 8d 6c 24 20       	lea    0x20(%rsp),%r13
    9801:	4c 89 ef             	mov    %r13,%rdi
    9804:	e9 23 ff ff ff       	jmp    972c <_ZN8argparse8Argument13default_valueIiEERS0_OT_+0x8c>
    9809:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
    9810:	48 8b 54 24 18       	mov    0x18(%rsp),%rdx
    9815:	48 85 d2             	test   %rdx,%rdx
    9818:	74 18                	je     9832 <_ZN8argparse8Argument13default_valueIiEERS0_OT_+0x192>
    981a:	48 83 fa 01          	cmp    $0x1,%rdx
    981e:	74 28                	je     9848 <_ZN8argparse8Argument13default_valueIiEERS0_OT_+0x1a8>
    9820:	4c 89 ee             	mov    %r13,%rsi
    9823:	e8 18 99 ff ff       	call   3140 <memcpy@plt>
    9828:	48 8b 54 24 18       	mov    0x18(%rsp),%rdx
    982d:	49 8b 7c 24 58       	mov    0x58(%r12),%rdi
    9832:	49 89 54 24 60       	mov    %rdx,0x60(%r12)
    9837:	c6 04 17 00          	movb   $0x0,(%rdi,%rdx,1)
    983b:	48 8b 7c 24 10       	mov    0x10(%rsp),%rdi
    9840:	e9 e7 fe ff ff       	jmp    972c <_ZN8argparse8Argument13default_valueIiEERS0_OT_+0x8c>
    9845:	0f 1f 00             	nopl   (%rax)
    9848:	0f b6 44 24 20       	movzbl 0x20(%rsp),%eax
    984d:	88 07                	mov    %al,(%rdi)
    984f:	48 8b 54 24 18       	mov    0x18(%rsp),%rdx
    9854:	49 8b 7c 24 58       	mov    0x58(%r12),%rdi
    9859:	eb d7                	jmp    9832 <_ZN8argparse8Argument13default_valueIiEERS0_OT_+0x192>
    985b:	e8 f0 98 ff ff       	call   3150 <__stack_chk_fail@plt>

0000000000009860 <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJRS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_>:
    9860:	41 57                	push   %r15
    9862:	41 56                	push   %r14
    9864:	41 55                	push   %r13
    9866:	41 54                	push   %r12
    9868:	55                   	push   %rbp
    9869:	53                   	push   %rbx
    986a:	48 83 ec 48          	sub    $0x48,%rsp
    986e:	48 89 7c 24 20       	mov    %rdi,0x20(%rsp)
    9873:	4c 8b 67 08          	mov    0x8(%rdi),%r12
    9877:	48 8b 0f             	mov    (%rdi),%rcx
    987a:	64 48 8b 04 25 28 00 	mov    %fs:0x28,%rax
    9881:	00 00 
    9883:	48 89 44 24 38       	mov    %rax,0x38(%rsp)
    9888:	31 c0                	xor    %eax,%eax
    988a:	4c 89 e0             	mov    %r12,%rax
    988d:	48 29 c8             	sub    %rcx,%rax
    9890:	48 c1 f8 04          	sar    $0x4,%rax
    9894:	48 bf ff ff ff ff ff 	movabs $0x7ffffffffffffff,%rdi
    989b:	ff ff 07 
    989e:	48 89 4c 24 10       	mov    %rcx,0x10(%rsp)
    98a3:	48 39 f8             	cmp    %rdi,%rax
    98a6:	0f 84 3d 02 00 00    	je     9ae9 <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJRS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0x289>
    98ac:	48 89 f5             	mov    %rsi,%rbp
    98af:	48 89 d6             	mov    %rdx,%rsi
    98b2:	ba 01 00 00 00       	mov    $0x1,%edx
    98b7:	48 39 d0             	cmp    %rdx,%rax
    98ba:	48 89 c1             	mov    %rax,%rcx
    98bd:	48 89 d0             	mov    %rdx,%rax
    98c0:	48 0f 43 c1          	cmovae %rcx,%rax
    98c4:	31 d2                	xor    %edx,%edx
    98c6:	48 01 c8             	add    %rcx,%rax
    98c9:	0f 92 c2             	setb   %dl
    98cc:	49 89 ef             	mov    %rbp,%r15
    98cf:	48 89 44 24 18       	mov    %rax,0x18(%rsp)
    98d4:	48 89 eb             	mov    %rbp,%rbx
    98d7:	4c 2b 7c 24 10       	sub    0x10(%rsp),%r15
    98dc:	48 85 d2             	test   %rdx,%rdx
    98df:	0f 85 db 01 00 00    	jne    9ac0 <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJRS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0x260>
    98e5:	48 85 c0             	test   %rax,%rax
    98e8:	0f 85 72 01 00 00    	jne    9a60 <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJRS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0x200>
    98ee:	48 c7 44 24 08 00 00 	movq   $0x0,0x8(%rsp)
    98f5:	00 00 
    98f7:	48 8b 06             	mov    (%rsi),%rax
    98fa:	4c 03 7c 24 08       	add    0x8(%rsp),%r15
    98ff:	49 c7 47 08 00 00 00 	movq   $0x0,0x8(%r15)
    9906:	00 
    9907:	48 85 c0             	test   %rax,%rax
    990a:	0f 84 c8 01 00 00    	je     9ad8 <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJRS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0x278>
    9910:	4c 89 7c 24 30       	mov    %r15,0x30(%rsp)
    9915:	48 8d 54 24 30       	lea    0x30(%rsp),%rdx
    991a:	bf 02 00 00 00       	mov    $0x2,%edi
    991f:	ff d0                	call   *%rax
    9921:	4c 8b 6c 24 10       	mov    0x10(%rsp),%r13
    9926:	4c 8b 74 24 08       	mov    0x8(%rsp),%r14
    992b:	4c 39 ed             	cmp    %r13,%rbp
    992e:	74 57                	je     9987 <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJRS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0x127>
    9930:	4c 8d 7c 24 30       	lea    0x30(%rsp),%r15
    9935:	0f 1f 00             	nopl   (%rax)
    9938:	49 c7 46 08 00 00 00 	movq   $0x0,0x8(%r14)
    993f:	00 
    9940:	4d 8b 45 00          	mov    0x0(%r13),%r8
    9944:	4d 85 c0             	test   %r8,%r8
    9947:	0f 84 63 01 00 00    	je     9ab0 <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJRS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0x250>
    994d:	4c 89 74 24 30       	mov    %r14,0x30(%rsp)
    9952:	4c 89 fa             	mov    %r15,%rdx
    9955:	4c 89 ee             	mov    %r13,%rsi
    9958:	bf 04 00 00 00       	mov    $0x4,%edi
    995d:	41 ff d0             	call   *%r8
    9960:	4d 8b 45 00          	mov    0x0(%r13),%r8
    9964:	4d 85 c0             	test   %r8,%r8
    9967:	0f 84 23 01 00 00    	je     9a90 <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJRS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0x230>
    996d:	4c 89 ee             	mov    %r13,%rsi
    9970:	31 d2                	xor    %edx,%edx
    9972:	bf 03 00 00 00       	mov    $0x3,%edi
    9977:	49 83 c5 10          	add    $0x10,%r13
    997b:	41 ff d0             	call   *%r8
    997e:	49 83 c6 10          	add    $0x10,%r14
    9982:	4c 39 ed             	cmp    %r13,%rbp
    9985:	75 b1                	jne    9938 <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJRS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0xd8>
    9987:	4d 8d 7e 10          	lea    0x10(%r14),%r15
    998b:	4c 39 e5             	cmp    %r12,%rbp
    998e:	74 67                	je     99f7 <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJRS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0x197>
    9990:	48 8d 6c 24 30       	lea    0x30(%rsp),%rbp
    9995:	eb 3c                	jmp    99d3 <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJRS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0x173>
    9997:	66 0f 1f 84 00 00 00 	nopw   0x0(%rax,%rax,1)
    999e:	00 00 
    99a0:	4c 89 7c 24 30       	mov    %r15,0x30(%rsp)
    99a5:	48 89 ea             	mov    %rbp,%rdx
    99a8:	48 89 de             	mov    %rbx,%rsi
    99ab:	bf 04 00 00 00       	mov    $0x4,%edi
    99b0:	ff d0                	call   *%rax
    99b2:	48 8b 03             	mov    (%rbx),%rax
    99b5:	48 85 c0             	test   %rax,%rax
    99b8:	74 0c                	je     99c6 <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJRS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0x166>
    99ba:	31 d2                	xor    %edx,%edx
    99bc:	48 89 de             	mov    %rbx,%rsi
    99bf:	bf 03 00 00 00       	mov    $0x3,%edi
    99c4:	ff d0                	call   *%rax
    99c6:	48 83 c3 10          	add    $0x10,%rbx
    99ca:	49 83 c7 10          	add    $0x10,%r15
    99ce:	4c 39 e3             	cmp    %r12,%rbx
    99d1:	74 24                	je     99f7 <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJRS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0x197>
    99d3:	48 8b 03             	mov    (%rbx),%rax
    99d6:	49 c7 47 08 00 00 00 	movq   $0x0,0x8(%r15)
    99dd:	00 
    99de:	48 85 c0             	test   %rax,%rax
    99e1:	75 bd                	jne    99a0 <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJRS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0x140>
    99e3:	48 83 c3 10          	add    $0x10,%rbx
    99e7:	49 c7 07 00 00 00 00 	movq   $0x0,(%r15)
    99ee:	49 83 c7 10          	add    $0x10,%r15
    99f2:	4c 39 e3             	cmp    %r12,%rbx
    99f5:	75 dc                	jne    99d3 <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJRS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0x173>
    99f7:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
    99fc:	48 85 c0             	test   %rax,%rax
    99ff:	74 1c                	je     9a1d <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJRS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0x1bd>
    9a01:	48 8b 4c 24 20       	mov    0x20(%rsp),%rcx
    9a06:	48 89 c7             	mov    %rax,%rdi
    9a09:	48 8b 49 10          	mov    0x10(%rcx),%rcx
    9a0d:	48 89 ce             	mov    %rcx,%rsi
    9a10:	48 29 c6             	sub    %rax,%rsi
    9a13:	48 89 4c 24 10       	mov    %rcx,0x10(%rsp)
    9a18:	e8 03 99 ff ff       	call   3320 <_ZdlPvm@plt>
    9a1d:	48 8b 4c 24 20       	mov    0x20(%rsp),%rcx
    9a22:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
    9a27:	4c 89 79 08          	mov    %r15,0x8(%rcx)
    9a2b:	48 89 01             	mov    %rax,(%rcx)
    9a2e:	48 03 44 24 18       	add    0x18(%rsp),%rax
    9a33:	48 89 41 10          	mov    %rax,0x10(%rcx)
    9a37:	48 8b 44 24 38       	mov    0x38(%rsp),%rax
    9a3c:	64 48 2b 04 25 28 00 	sub    %fs:0x28,%rax
    9a43:	00 00 
    9a45:	0f 85 99 00 00 00    	jne    9ae4 <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJRS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0x284>
    9a4b:	48 83 c4 48          	add    $0x48,%rsp
    9a4f:	5b                   	pop    %rbx
    9a50:	5d                   	pop    %rbp
    9a51:	41 5c                	pop    %r12
    9a53:	41 5d                	pop    %r13
    9a55:	41 5e                	pop    %r14
    9a57:	41 5f                	pop    %r15
    9a59:	c3                   	ret    
    9a5a:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)
    9a60:	48 39 f8             	cmp    %rdi,%rax
    9a63:	48 0f 46 f8          	cmovbe %rax,%rdi
    9a67:	48 c1 e7 04          	shl    $0x4,%rdi
    9a6b:	48 89 7c 24 18       	mov    %rdi,0x18(%rsp)
    9a70:	48 89 74 24 28       	mov    %rsi,0x28(%rsp)
    9a75:	e8 26 98 ff ff       	call   32a0 <_Znwm@plt>
    9a7a:	48 89 44 24 08       	mov    %rax,0x8(%rsp)
    9a7f:	48 8b 74 24 28       	mov    0x28(%rsp),%rsi
    9a84:	e9 6e fe ff ff       	jmp    98f7 <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJRS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0x97>
    9a89:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
    9a90:	49 83 c5 10          	add    $0x10,%r13
    9a94:	49 83 c6 10          	add    $0x10,%r14
    9a98:	49 39 ed             	cmp    %rbp,%r13
    9a9b:	0f 85 97 fe ff ff    	jne    9938 <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJRS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0xd8>
    9aa1:	e9 e1 fe ff ff       	jmp    9987 <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJRS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0x127>
    9aa6:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
    9aad:	00 00 00 
    9ab0:	49 c7 06 00 00 00 00 	movq   $0x0,(%r14)
    9ab7:	e9 a4 fe ff ff       	jmp    9960 <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJRS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0x100>
    9abc:	0f 1f 40 00          	nopl   0x0(%rax)
    9ac0:	48 b8 f0 ff ff ff ff 	movabs $0x7ffffffffffffff0,%rax
    9ac7:	ff ff 7f 
    9aca:	48 89 44 24 18       	mov    %rax,0x18(%rsp)
    9acf:	48 89 c7             	mov    %rax,%rdi
    9ad2:	eb 9c                	jmp    9a70 <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJRS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0x210>
    9ad4:	0f 1f 40 00          	nopl   0x0(%rax)
    9ad8:	49 c7 07 00 00 00 00 	movq   $0x0,(%r15)
    9adf:	e9 3d fe ff ff       	jmp    9921 <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJRS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0xc1>
    9ae4:	e8 67 96 ff ff       	call   3150 <__stack_chk_fail@plt>
    9ae9:	48 8d 3d 20 68 00 00 	lea    0x6820(%rip),%rdi        # 10310 <_fini+0x10ef>
    9af0:	e8 4b 98 ff ff       	call   3340 <_ZSt20__throw_length_errorPKc@plt>
    9af5:	48 89 c7             	mov    %rax,%rdi
    9af8:	c5 f8 77             	vzeroupper 
    9afb:	e8 00 97 ff ff       	call   3200 <__cxa_begin_catch@plt>
    9b00:	48 83 7c 24 08 00    	cmpq   $0x0,0x8(%rsp)
    9b06:	74 14                	je     9b1c <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJRS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0x2bc>
    9b08:	48 8b 74 24 18       	mov    0x18(%rsp),%rsi
    9b0d:	48 8b 7c 24 08       	mov    0x8(%rsp),%rdi
    9b12:	e8 09 98 ff ff       	call   3320 <_ZdlPvm@plt>
    9b17:	e8 f4 96 ff ff       	call   3210 <__cxa_rethrow@plt>
    9b1c:	49 8b 07             	mov    (%r15),%rax
    9b1f:	48 85 c0             	test   %rax,%rax
    9b22:	74 f3                	je     9b17 <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJRS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0x2b7>
    9b24:	31 d2                	xor    %edx,%edx
    9b26:	4c 89 fe             	mov    %r15,%rsi
    9b29:	bf 03 00 00 00       	mov    $0x3,%edi
    9b2e:	ff d0                	call   *%rax
    9b30:	eb e5                	jmp    9b17 <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJRS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0x2b7>
    9b32:	48 89 c5             	mov    %rax,%rbp
    9b35:	c5 f8 77             	vzeroupper 
    9b38:	e8 83 95 ff ff       	call   30c0 <__cxa_end_catch@plt>
    9b3d:	48 89 ef             	mov    %rbp,%rdi
    9b40:	e8 1b 98 ff ff       	call   3360 <_Unwind_Resume@plt>
    9b45:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
    9b4c:	00 00 00 
    9b4f:	90                   	nop

0000000000009b50 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E>:
    9b50:	41 57                	push   %r15
    9b52:	49 89 f7             	mov    %rsi,%r15
    9b55:	41 56                	push   %r14
    9b57:	41 55                	push   %r13
    9b59:	41 54                	push   %r12
    9b5b:	55                   	push   %rbp
    9b5c:	53                   	push   %rbx
    9b5d:	48 89 fb             	mov    %rdi,%rbx
    9b60:	48 81 ec a8 00 00 00 	sub    $0xa8,%rsp
    9b67:	64 48 8b 04 25 28 00 	mov    %fs:0x28,%rax
    9b6e:	00 00 
    9b70:	48 89 84 24 98 00 00 	mov    %rax,0x98(%rsp)
    9b77:	00 
    9b78:	31 c0                	xor    %eax,%eax
    9b7a:	0f b6 87 d9 00 00 00 	movzbl 0xd9(%rdi),%eax
    9b81:	a8 04                	test   $0x4,%al
    9b83:	75 08                	jne    9b8d <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x3d>
    9b85:	a8 08                	test   $0x8,%al
    9b87:	0f 85 f3 03 00 00    	jne    9f80 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x430>
    9b8d:	48 8b bb d0 00 00 00 	mov    0xd0(%rbx),%rdi
    9b94:	80 8b d9 00 00 00 08 	orb    $0x8,0xd9(%rbx)
    9b9b:	48 89 4b 18          	mov    %rcx,0x18(%rbx)
    9b9f:	4c 89 43 20          	mov    %r8,0x20(%rbx)
    9ba3:	48 85 ff             	test   %rdi,%rdi
    9ba6:	0f 84 b4 00 00 00    	je     9c60 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x110>
    9bac:	48 89 d1             	mov    %rdx,%rcx
    9baf:	4c 29 f9             	sub    %r15,%rcx
    9bb2:	4c 8b b3 c8 00 00 00 	mov    0xc8(%rbx),%r14
    9bb9:	48 c1 f9 05          	sar    $0x5,%rcx
    9bbd:	4c 39 f1             	cmp    %r14,%rcx
    9bc0:	73 3e                	jae    9c00 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0xb0>
    9bc2:	48 83 7b 48 00       	cmpq   $0x0,0x48(%rbx)
    9bc7:	0f 84 7b 03 00 00    	je     9f48 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x3f8>
    9bcd:	4c 89 f8             	mov    %r15,%rax
    9bd0:	48 8b 94 24 98 00 00 	mov    0x98(%rsp),%rdx
    9bd7:	00 
    9bd8:	64 48 2b 14 25 28 00 	sub    %fs:0x28,%rdx
    9bdf:	00 00 
    9be1:	0f 85 5c 03 00 00    	jne    9f43 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x3f3>
    9be7:	48 81 c4 a8 00 00 00 	add    $0xa8,%rsp
    9bee:	5b                   	pop    %rbx
    9bef:	5d                   	pop    %rbp
    9bf0:	41 5c                	pop    %r12
    9bf2:	41 5d                	pop    %r13
    9bf4:	41 5e                	pop    %r14
    9bf6:	41 5f                	pop    %r15
    9bf8:	c3                   	ret    
    9bf9:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
    9c00:	49 89 fc             	mov    %rdi,%r12
    9c03:	49 c1 e4 05          	shl    $0x5,%r12
    9c07:	4d 01 fc             	add    %r15,%r12
    9c0a:	48 39 f9             	cmp    %rdi,%rcx
    9c0d:	4c 0f 46 e2          	cmovbe %rdx,%r12
    9c11:	80 bb d8 00 00 00 00 	cmpb   $0x0,0xd8(%rbx)
    9c18:	0f 84 c2 00 00 00    	je     9ce0 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x190>
    9c1e:	c4 c1 f9 6e cc       	vmovq  %r12,%xmm1
    9c23:	4c 89 7c 24 10       	mov    %r15,0x10(%rsp)
    9c28:	c4 e3 f1 22 c3 01    	vpinsrq $0x1,%rbx,%xmm1,%xmm0
    9c2e:	0f b6 93 a8 00 00 00 	movzbl 0xa8(%rbx),%edx
    9c35:	48 8d 05 64 9c 00 00 	lea    0x9c64(%rip),%rax        # 138a0 <_ZNSt8__detail9__variant12__gen_vtableINS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISE_SaISE_EEEEEET_SL_SL_St17basic_string_viewIcSC_EE11ActionApplyJRSt7variantIJSt8functionIFSt3anyRSF_EESR_IFvST_EEEEEE9_S_vtableE>
    9c3c:	c5 fa 7f 44 24 18    	vmovdqu %xmm0,0x18(%rsp)
    9c42:	48 8d b3 88 00 00 00 	lea    0x88(%rbx),%rsi
    9c49:	48 8d 7c 24 10       	lea    0x10(%rsp),%rdi
    9c4e:	ff 14 d0             	call   *(%rax,%rdx,8)
    9c51:	4c 89 e0             	mov    %r12,%rax
    9c54:	e9 77 ff ff ff       	jmp    9bd0 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x80>
    9c59:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
    9c60:	48 8b b3 b8 00 00 00 	mov    0xb8(%rbx),%rsi
    9c67:	4c 8d 43 78          	lea    0x78(%rbx),%r8
    9c6b:	48 3b b3 c0 00 00 00 	cmp    0xc0(%rbx),%rsi
    9c72:	0f 84 e0 01 00 00    	je     9e58 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x308>
    9c78:	48 c7 46 08 00 00 00 	movq   $0x0,0x8(%rsi)
    9c7f:	00 
    9c80:	48 8b 43 78          	mov    0x78(%rbx),%rax
    9c84:	48 85 c0             	test   %rax,%rax
    9c87:	74 47                	je     9cd0 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x180>
    9c89:	48 89 74 24 10       	mov    %rsi,0x10(%rsp)
    9c8e:	48 8d 6c 24 10       	lea    0x10(%rsp),%rbp
    9c93:	48 89 ea             	mov    %rbp,%rdx
    9c96:	4c 89 c6             	mov    %r8,%rsi
    9c99:	bf 02 00 00 00       	mov    $0x2,%edi
    9c9e:	ff d0                	call   *%rax
    9ca0:	48 83 83 b8 00 00 00 	addq   $0x10,0xb8(%rbx)
    9ca7:	10 
    9ca8:	0f b6 93 a8 00 00 00 	movzbl 0xa8(%rbx),%edx
    9caf:	48 8d 05 fa 9b 00 00 	lea    0x9bfa(%rip),%rax        # 138b0 <_ZNSt8__detail9__variant12__gen_vtableINS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISE_SaISE_EEEEEET_SL_SL_St17basic_string_viewIcSC_EEUlRKSL_E_JRSt7variantIJSt8functionIFSt3anyRSF_EEST_IFvSV_EEEEEE9_S_vtableE>
    9cb6:	48 8d b3 88 00 00 00 	lea    0x88(%rbx),%rsi
    9cbd:	48 89 ef             	mov    %rbp,%rdi
    9cc0:	ff 14 d0             	call   *(%rax,%rdx,8)
    9cc3:	4c 89 f8             	mov    %r15,%rax
    9cc6:	e9 05 ff ff ff       	jmp    9bd0 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x80>
    9ccb:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)
    9cd0:	48 c7 06 00 00 00 00 	movq   $0x0,(%rsi)
    9cd7:	48 8d 6c 24 10       	lea    0x10(%rsp),%rbp
    9cdc:	eb c2                	jmp    9ca0 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x150>
    9cde:	66 90                	xchg   %ax,%ax
    9ce0:	4d 89 e5             	mov    %r12,%r13
    9ce3:	4d 29 fd             	sub    %r15,%r13
    9ce6:	4c 89 e8             	mov    %r13,%rax
    9ce9:	48 c1 f8 05          	sar    $0x5,%rax
    9ced:	49 c1 fd 07          	sar    $0x7,%r13
    9cf1:	48 89 44 24 08       	mov    %rax,0x8(%rsp)
    9cf6:	4d 85 ed             	test   %r13,%r13
    9cf9:	0f 8e 21 02 00 00    	jle    9f20 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x3d0>
    9cff:	49 c1 e5 07          	shl    $0x7,%r13
    9d03:	4d 01 fd             	add    %r15,%r13
    9d06:	4c 89 fd             	mov    %r15,%rbp
    9d09:	eb 50                	jmp    9d5b <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x20b>
    9d0b:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)
    9d10:	48 8b 7d 28          	mov    0x28(%rbp),%rdi
    9d14:	48 8b 75 20          	mov    0x20(%rbp),%rsi
    9d18:	48 85 ff             	test   %rdi,%rdi
    9d1b:	74 05                	je     9d22 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x1d2>
    9d1d:	80 3e 2d             	cmpb   $0x2d,(%rsi)
    9d20:	74 76                	je     9d98 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x248>
    9d22:	48 8b 7d 48          	mov    0x48(%rbp),%rdi
    9d26:	48 8b 75 40          	mov    0x40(%rbp),%rsi
    9d2a:	48 85 ff             	test   %rdi,%rdi
    9d2d:	74 09                	je     9d38 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x1e8>
    9d2f:	80 3e 2d             	cmpb   $0x2d,(%rsi)
    9d32:	0f 84 b8 00 00 00    	je     9df0 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x2a0>
    9d38:	48 8b 7d 68          	mov    0x68(%rbp),%rdi
    9d3c:	48 8b 75 60          	mov    0x60(%rbp),%rsi
    9d40:	48 85 ff             	test   %rdi,%rdi
    9d43:	74 09                	je     9d4e <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x1fe>
    9d45:	80 3e 2d             	cmpb   $0x2d,(%rsi)
    9d48:	0f 84 d2 00 00 00    	je     9e20 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x2d0>
    9d4e:	48 83 ed 80          	sub    $0xffffffffffffff80,%rbp
    9d52:	49 39 ed             	cmp    %rbp,%r13
    9d55:	0f 84 1d 01 00 00    	je     9e78 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x328>
    9d5b:	48 8b 7d 08          	mov    0x8(%rbp),%rdi
    9d5f:	48 8b 75 00          	mov    0x0(%rbp),%rsi
    9d63:	48 85 ff             	test   %rdi,%rdi
    9d66:	74 a8                	je     9d10 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x1c0>
    9d68:	80 3e 2d             	cmpb   $0x2d,(%rsi)
    9d6b:	75 a3                	jne    9d10 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x1c0>
    9d6d:	48 ff cf             	dec    %rdi
    9d70:	74 9e                	je     9d10 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x1c0>
    9d72:	48 ff c6             	inc    %rsi
    9d75:	e8 96 df ff ff       	call   7d10 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE>
    9d7a:	84 c0                	test   %al,%al
    9d7c:	75 92                	jne    9d10 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x1c0>
    9d7e:	48 89 e8             	mov    %rbp,%rax
    9d81:	4c 29 f8             	sub    %r15,%rax
    9d84:	48 c1 f8 05          	sar    $0x5,%rax
    9d88:	48 89 44 24 08       	mov    %rax,0x8(%rsp)
    9d8d:	49 89 ec             	mov    %rbp,%r12
    9d90:	eb 2e                	jmp    9dc0 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x270>
    9d92:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)
    9d98:	48 ff cf             	dec    %rdi
    9d9b:	74 85                	je     9d22 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x1d2>
    9d9d:	48 ff c6             	inc    %rsi
    9da0:	e8 6b df ff ff       	call   7d10 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE>
    9da5:	84 c0                	test   %al,%al
    9da7:	0f 85 75 ff ff ff    	jne    9d22 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x1d2>
    9dad:	4c 8d 65 20          	lea    0x20(%rbp),%r12
    9db1:	4c 89 e0             	mov    %r12,%rax
    9db4:	4c 29 f8             	sub    %r15,%rax
    9db7:	48 c1 f8 05          	sar    $0x5,%rax
    9dbb:	48 89 44 24 08       	mov    %rax,0x8(%rsp)
    9dc0:	4c 3b 74 24 08       	cmp    0x8(%rsp),%r14
    9dc5:	0f 86 53 fe ff ff    	jbe    9c1e <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0xce>
    9dcb:	bf 10 00 00 00       	mov    $0x10,%edi
    9dd0:	e8 fb 92 ff ff       	call   30d0 <__cxa_allocate_exception@plt>
    9dd5:	48 89 c7             	mov    %rax,%rdi
    9dd8:	48 8d 35 5e 65 00 00 	lea    0x655e(%rip),%rsi        # 1033d <_fini+0x111c>
    9ddf:	48 89 c5             	mov    %rax,%rbp
    9de2:	e8 09 94 ff ff       	call   31f0 <_ZNSt13runtime_errorC1EPKc@plt>
    9de7:	e9 b0 01 00 00       	jmp    9f9c <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x44c>
    9dec:	0f 1f 40 00          	nopl   0x0(%rax)
    9df0:	48 ff cf             	dec    %rdi
    9df3:	0f 84 3f ff ff ff    	je     9d38 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x1e8>
    9df9:	48 ff c6             	inc    %rsi
    9dfc:	e8 0f df ff ff       	call   7d10 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE>
    9e01:	84 c0                	test   %al,%al
    9e03:	0f 85 2f ff ff ff    	jne    9d38 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x1e8>
    9e09:	4c 8d 65 40          	lea    0x40(%rbp),%r12
    9e0d:	4c 89 e0             	mov    %r12,%rax
    9e10:	4c 29 f8             	sub    %r15,%rax
    9e13:	48 c1 f8 05          	sar    $0x5,%rax
    9e17:	48 89 44 24 08       	mov    %rax,0x8(%rsp)
    9e1c:	eb a2                	jmp    9dc0 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x270>
    9e1e:	66 90                	xchg   %ax,%ax
    9e20:	48 ff cf             	dec    %rdi
    9e23:	0f 84 25 ff ff ff    	je     9d4e <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x1fe>
    9e29:	48 ff c6             	inc    %rsi
    9e2c:	e8 df de ff ff       	call   7d10 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE>
    9e31:	84 c0                	test   %al,%al
    9e33:	0f 85 15 ff ff ff    	jne    9d4e <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x1fe>
    9e39:	4c 8d 65 60          	lea    0x60(%rbp),%r12
    9e3d:	4c 89 e0             	mov    %r12,%rax
    9e40:	4c 29 f8             	sub    %r15,%rax
    9e43:	48 c1 f8 05          	sar    $0x5,%rax
    9e47:	48 89 44 24 08       	mov    %rax,0x8(%rsp)
    9e4c:	e9 6f ff ff ff       	jmp    9dc0 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x270>
    9e51:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
    9e58:	48 8d bb b0 00 00 00 	lea    0xb0(%rbx),%rdi
    9e5f:	4c 89 c2             	mov    %r8,%rdx
    9e62:	e8 f9 f9 ff ff       	call   9860 <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJRS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_>
    9e67:	48 8d 6c 24 10       	lea    0x10(%rsp),%rbp
    9e6c:	e9 37 fe ff ff       	jmp    9ca8 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x158>
    9e71:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
    9e78:	4c 89 e0             	mov    %r12,%rax
    9e7b:	48 29 e8             	sub    %rbp,%rax
    9e7e:	48 c1 f8 05          	sar    $0x5,%rax
    9e82:	48 83 f8 02          	cmp    $0x2,%rax
    9e86:	74 66                	je     9eee <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x39e>
    9e88:	48 83 f8 03          	cmp    $0x3,%rax
    9e8c:	74 4a                	je     9ed8 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x388>
    9e8e:	48 83 f8 01          	cmp    $0x1,%rax
    9e92:	0f 85 28 ff ff ff    	jne    9dc0 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x270>
    9e98:	48 8b 7d 08          	mov    0x8(%rbp),%rdi
    9e9c:	48 8b 45 00          	mov    0x0(%rbp),%rax
    9ea0:	48 85 ff             	test   %rdi,%rdi
    9ea3:	0f 84 17 ff ff ff    	je     9dc0 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x270>
    9ea9:	80 38 2d             	cmpb   $0x2d,(%rax)
    9eac:	0f 85 0e ff ff ff    	jne    9dc0 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x270>
    9eb2:	48 ff cf             	dec    %rdi
    9eb5:	0f 84 05 ff ff ff    	je     9dc0 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x270>
    9ebb:	48 8d 70 01          	lea    0x1(%rax),%rsi
    9ebf:	e8 4c de ff ff       	call   7d10 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE>
    9ec4:	84 c0                	test   %al,%al
    9ec6:	0f 84 b2 fe ff ff    	je     9d7e <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x22e>
    9ecc:	e9 ef fe ff ff       	jmp    9dc0 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x270>
    9ed1:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
    9ed8:	48 8b 45 08          	mov    0x8(%rbp),%rax
    9edc:	48 8b 55 00          	mov    0x0(%rbp),%rdx
    9ee0:	48 85 c0             	test   %rax,%rax
    9ee3:	74 05                	je     9eea <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x39a>
    9ee5:	80 3a 2d             	cmpb   $0x2d,(%rdx)
    9ee8:	74 3e                	je     9f28 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x3d8>
    9eea:	48 83 c5 20          	add    $0x20,%rbp
    9eee:	48 8b 7d 08          	mov    0x8(%rbp),%rdi
    9ef2:	48 8b 45 00          	mov    0x0(%rbp),%rax
    9ef6:	48 85 ff             	test   %rdi,%rdi
    9ef9:	74 05                	je     9f00 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x3b0>
    9efb:	80 38 2d             	cmpb   $0x2d,(%rax)
    9efe:	74 06                	je     9f06 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x3b6>
    9f00:	48 83 c5 20          	add    $0x20,%rbp
    9f04:	eb 92                	jmp    9e98 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x348>
    9f06:	48 ff cf             	dec    %rdi
    9f09:	74 f5                	je     9f00 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x3b0>
    9f0b:	48 8d 70 01          	lea    0x1(%rax),%rsi
    9f0f:	e8 fc dd ff ff       	call   7d10 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE>
    9f14:	84 c0                	test   %al,%al
    9f16:	0f 84 62 fe ff ff    	je     9d7e <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x22e>
    9f1c:	eb e2                	jmp    9f00 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x3b0>
    9f1e:	66 90                	xchg   %ax,%ax
    9f20:	4c 89 fd             	mov    %r15,%rbp
    9f23:	e9 5a ff ff ff       	jmp    9e82 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x332>
    9f28:	48 ff c8             	dec    %rax
    9f2b:	48 89 c7             	mov    %rax,%rdi
    9f2e:	74 ba                	je     9eea <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x39a>
    9f30:	48 8d 72 01          	lea    0x1(%rdx),%rsi
    9f34:	e8 d7 dd ff ff       	call   7d10 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE>
    9f39:	84 c0                	test   %al,%al
    9f3b:	0f 84 3d fe ff ff    	je     9d7e <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x22e>
    9f41:	eb a7                	jmp    9eea <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x39a>
    9f43:	e8 08 92 ff ff       	call   3150 <__stack_chk_fail@plt>
    9f48:	bf 10 00 00 00       	mov    $0x10,%edi
    9f4d:	e8 7e 91 ff ff       	call   30d0 <__cxa_allocate_exception@plt>
    9f52:	4c 8b 7b 20          	mov    0x20(%rbx),%r15
    9f56:	4c 8b 6b 18          	mov    0x18(%rbx),%r13
    9f5a:	48 89 c5             	mov    %rax,%rbp
    9f5d:	4c 89 f8             	mov    %r15,%rax
    9f60:	4c 8d 64 24 40       	lea    0x40(%rsp),%r12
    9f65:	4c 01 e8             	add    %r13,%rax
    9f68:	4c 89 64 24 30       	mov    %r12,0x30(%rsp)
    9f6d:	74 4d                	je     9fbc <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x46c>
    9f6f:	4d 85 ff             	test   %r15,%r15
    9f72:	75 48                	jne    9fbc <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x46c>
    9f74:	48 8d 3d 85 60 00 00 	lea    0x6085(%rip),%rdi        # 10000 <_fini+0xddf>
    9f7b:	e8 40 94 ff ff       	call   33c0 <_ZSt19__throw_logic_errorPKc@plt>
    9f80:	bf 10 00 00 00       	mov    $0x10,%edi
    9f85:	e8 46 91 ff ff       	call   30d0 <__cxa_allocate_exception@plt>
    9f8a:	48 89 c7             	mov    %rax,%rdi
    9f8d:	48 8d 35 96 63 00 00 	lea    0x6396(%rip),%rsi        # 1032a <_fini+0x1109>
    9f94:	48 89 c5             	mov    %rax,%rbp
    9f97:	e8 54 92 ff ff       	call   31f0 <_ZNSt13runtime_errorC1EPKc@plt>
    9f9c:	48 8b 15 25 a0 00 00 	mov    0xa025(%rip),%rdx        # 13fc8 <_ZNSt13runtime_errorD1Ev@Base>
    9fa3:	48 8d 35 f6 99 00 00 	lea    0x99f6(%rip),%rsi        # 139a0 <_ZTISt13runtime_error@@Base>
    9faa:	48 89 ef             	mov    %rbp,%rdi
    9fad:	e8 6e 92 ff ff       	call   3220 <__cxa_throw@plt>
    9fb2:	49 89 c4             	mov    %rax,%r12
    9fb5:	e9 7e 01 00 00       	jmp    a138 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x5e8>
    9fba:	eb f6                	jmp    9fb2 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x462>
    9fbc:	4c 89 6c 24 10       	mov    %r13,0x10(%rsp)
    9fc1:	49 83 fd 0f          	cmp    $0xf,%r13
    9fc5:	0f 86 80 01 00 00    	jbe    a14b <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x5fb>
    9fcb:	4c 8d 74 24 30       	lea    0x30(%rsp),%r14
    9fd0:	48 8d 74 24 10       	lea    0x10(%rsp),%rsi
    9fd5:	31 d2                	xor    %edx,%edx
    9fd7:	4c 89 f7             	mov    %r14,%rdi
    9fda:	e8 21 93 ff ff       	call   3300 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE9_M_createERmm@plt>
    9fdf:	48 89 44 24 30       	mov    %rax,0x30(%rsp)
    9fe4:	48 89 c7             	mov    %rax,%rdi
    9fe7:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
    9fec:	48 89 44 24 40       	mov    %rax,0x40(%rsp)
    9ff1:	4c 89 ea             	mov    %r13,%rdx
    9ff4:	4c 89 fe             	mov    %r15,%rsi
    9ff7:	e8 44 91 ff ff       	call   3140 <memcpy@plt>
    9ffc:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
    a001:	48 8b 54 24 30       	mov    0x30(%rsp),%rdx
    a006:	48 89 44 24 38       	mov    %rax,0x38(%rsp)
    a00b:	c6 04 02 00          	movb   $0x0,(%rdx,%rax,1)
    a00f:	41 b8 17 00 00 00    	mov    $0x17,%r8d
    a015:	48 8d 0d 33 63 00 00 	lea    0x6333(%rip),%rcx        # 1034f <_fini+0x112e>
    a01c:	31 d2                	xor    %edx,%edx
    a01e:	31 f6                	xor    %esi,%esi
    a020:	4c 89 f7             	mov    %r14,%rdi
    a023:	e8 88 91 ff ff       	call   31b0 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE10_M_replaceEmmPKcm@plt>
    a028:	4c 8d 6c 24 60       	lea    0x60(%rsp),%r13
    a02d:	4c 89 6c 24 50       	mov    %r13,0x50(%rsp)
    a032:	48 8d 50 10          	lea    0x10(%rax),%rdx
    a036:	48 8b 08             	mov    (%rax),%rcx
    a039:	48 39 d1             	cmp    %rdx,%rcx
    a03c:	0f 85 91 01 00 00    	jne    a1d3 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x683>
    a042:	c5 fa 6f 50 10       	vmovdqu 0x10(%rax),%xmm2
    a047:	c5 f9 7f 54 24 60    	vmovdqa %xmm2,0x60(%rsp)
    a04d:	48 8b 48 08          	mov    0x8(%rax),%rcx
    a051:	c6 40 10 00          	movb   $0x0,0x10(%rax)
    a055:	48 89 4c 24 58       	mov    %rcx,0x58(%rsp)
    a05a:	48 c7 40 08 00 00 00 	movq   $0x0,0x8(%rax)
    a061:	00 
    a062:	48 89 10             	mov    %rdx,(%rax)
    a065:	48 b8 ff ff ff ff ff 	movabs $0x3fffffffffffffff,%rax
    a06c:	ff ff 3f 
    a06f:	48 2b 44 24 58       	sub    0x58(%rsp),%rax
    a074:	48 83 f8 01          	cmp    $0x1,%rax
    a078:	0f 86 88 01 00 00    	jbe    a206 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x6b6>
    a07e:	48 8d 7c 24 50       	lea    0x50(%rsp),%rdi
    a083:	ba 02 00 00 00       	mov    $0x2,%edx
    a088:	48 8d 35 d8 62 00 00 	lea    0x62d8(%rip),%rsi        # 10367 <_fini+0x1146>
    a08f:	e8 4c 91 ff ff       	call   31e0 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE9_M_appendEPKcm@plt>
    a094:	4c 8d b4 24 80 00 00 	lea    0x80(%rsp),%r14
    a09b:	00 
    a09c:	4c 89 74 24 70       	mov    %r14,0x70(%rsp)
    a0a1:	48 8d 50 10          	lea    0x10(%rax),%rdx
    a0a5:	48 8b 08             	mov    (%rax),%rcx
    a0a8:	48 39 d1             	cmp    %rdx,%rcx
    a0ab:	0f 85 61 01 00 00    	jne    a212 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x6c2>
    a0b1:	c5 fa 6f 58 10       	vmovdqu 0x10(%rax),%xmm3
    a0b6:	c5 f9 7f 9c 24 80 00 	vmovdqa %xmm3,0x80(%rsp)
    a0bd:	00 00 
    a0bf:	48 8b 48 08          	mov    0x8(%rax),%rcx
    a0c3:	c6 40 10 00          	movb   $0x0,0x10(%rax)
    a0c7:	48 89 4c 24 78       	mov    %rcx,0x78(%rsp)
    a0cc:	48 89 10             	mov    %rdx,(%rax)
    a0cf:	48 c7 40 08 00 00 00 	movq   $0x0,0x8(%rax)
    a0d6:	00 
    a0d7:	48 8d 74 24 70       	lea    0x70(%rsp),%rsi
    a0dc:	48 89 ef             	mov    %rbp,%rdi
    a0df:	e8 ac 91 ff ff       	call   3290 <_ZNSt13runtime_errorC1ERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEE@plt>
    a0e4:	48 8b 7c 24 70       	mov    0x70(%rsp),%rdi
    a0e9:	4c 39 f7             	cmp    %r14,%rdi
    a0ec:	74 11                	je     a0ff <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x5af>
    a0ee:	48 8b 84 24 80 00 00 	mov    0x80(%rsp),%rax
    a0f5:	00 
    a0f6:	48 8d 70 01          	lea    0x1(%rax),%rsi
    a0fa:	e8 21 92 ff ff       	call   3320 <_ZdlPvm@plt>
    a0ff:	48 8b 7c 24 50       	mov    0x50(%rsp),%rdi
    a104:	4c 39 ef             	cmp    %r13,%rdi
    a107:	74 0e                	je     a117 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x5c7>
    a109:	48 8b 44 24 60       	mov    0x60(%rsp),%rax
    a10e:	48 8d 70 01          	lea    0x1(%rax),%rsi
    a112:	e8 09 92 ff ff       	call   3320 <_ZdlPvm@plt>
    a117:	48 8b 7c 24 30       	mov    0x30(%rsp),%rdi
    a11c:	4c 39 e7             	cmp    %r12,%rdi
    a11f:	0f 84 77 fe ff ff    	je     9f9c <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x44c>
    a125:	48 8b 44 24 40       	mov    0x40(%rsp),%rax
    a12a:	48 8d 70 01          	lea    0x1(%rax),%rsi
    a12e:	e8 ed 91 ff ff       	call   3320 <_ZdlPvm@plt>
    a133:	e9 64 fe ff ff       	jmp    9f9c <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x44c>
    a138:	48 89 ef             	mov    %rbp,%rdi
    a13b:	c5 f8 77             	vzeroupper 
    a13e:	e8 6d 92 ff ff       	call   33b0 <__cxa_free_exception@plt>
    a143:	4c 89 e7             	mov    %r12,%rdi
    a146:	e8 15 92 ff ff       	call   3360 <_Unwind_Resume@plt>
    a14b:	49 83 fd 01          	cmp    $0x1,%r13
    a14f:	0f 85 91 00 00 00    	jne    a1e6 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x696>
    a155:	41 0f b6 07          	movzbl (%r15),%eax
    a159:	4c 8d 74 24 30       	lea    0x30(%rsp),%r14
    a15e:	88 44 24 40          	mov    %al,0x40(%rsp)
    a162:	e9 95 fe ff ff       	jmp    9ffc <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x4ac>
    a167:	48 89 c3             	mov    %rax,%rbx
    a16a:	eb 54                	jmp    a1c0 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x670>
    a16c:	48 8b 7c 24 70       	mov    0x70(%rsp),%rdi
    a171:	4c 39 f7             	cmp    %r14,%rdi
    a174:	74 14                	je     a18a <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x63a>
    a176:	48 8b 84 24 80 00 00 	mov    0x80(%rsp),%rax
    a17d:	00 
    a17e:	48 8d 70 01          	lea    0x1(%rax),%rsi
    a182:	c5 f8 77             	vzeroupper 
    a185:	e8 96 91 ff ff       	call   3320 <_ZdlPvm@plt>
    a18a:	48 8b 7c 24 50       	mov    0x50(%rsp),%rdi
    a18f:	4c 39 ef             	cmp    %r13,%rdi
    a192:	74 11                	je     a1a5 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x655>
    a194:	48 8b 44 24 60       	mov    0x60(%rsp),%rax
    a199:	48 8d 70 01          	lea    0x1(%rax),%rsi
    a19d:	c5 f8 77             	vzeroupper 
    a1a0:	e8 7b 91 ff ff       	call   3320 <_ZdlPvm@plt>
    a1a5:	48 8b 7c 24 30       	mov    0x30(%rsp),%rdi
    a1aa:	4c 39 e7             	cmp    %r12,%rdi
    a1ad:	74 11                	je     a1c0 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x670>
    a1af:	48 8b 44 24 40       	mov    0x40(%rsp),%rax
    a1b4:	48 8d 70 01          	lea    0x1(%rax),%rsi
    a1b8:	c5 f8 77             	vzeroupper 
    a1bb:	e8 60 91 ff ff       	call   3320 <_ZdlPvm@plt>
    a1c0:	48 89 ef             	mov    %rbp,%rdi
    a1c3:	c5 f8 77             	vzeroupper 
    a1c6:	e8 e5 91 ff ff       	call   33b0 <__cxa_free_exception@plt>
    a1cb:	48 89 df             	mov    %rbx,%rdi
    a1ce:	e8 8d 91 ff ff       	call   3360 <_Unwind_Resume@plt>
    a1d3:	48 89 4c 24 50       	mov    %rcx,0x50(%rsp)
    a1d8:	48 8b 48 10          	mov    0x10(%rax),%rcx
    a1dc:	48 89 4c 24 60       	mov    %rcx,0x60(%rsp)
    a1e1:	e9 67 fe ff ff       	jmp    a04d <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x4fd>
    a1e6:	4c 8d 74 24 30       	lea    0x30(%rsp),%r14
    a1eb:	4d 85 ed             	test   %r13,%r13
    a1ee:	0f 84 08 fe ff ff    	je     9ffc <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x4ac>
    a1f4:	4c 89 e7             	mov    %r12,%rdi
    a1f7:	4c 8d 74 24 30       	lea    0x30(%rsp),%r14
    a1fc:	e9 f0 fd ff ff       	jmp    9ff1 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x4a1>
    a201:	48 89 c3             	mov    %rax,%rbx
    a204:	eb 9f                	jmp    a1a5 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x655>
    a206:	48 8d 3d ee 60 00 00 	lea    0x60ee(%rip),%rdi        # 102fb <_fini+0x10da>
    a20d:	e8 2e 91 ff ff       	call   3340 <_ZSt20__throw_length_errorPKc@plt>
    a212:	48 89 4c 24 70       	mov    %rcx,0x70(%rsp)
    a217:	48 8b 48 10          	mov    0x10(%rax),%rcx
    a21b:	48 89 8c 24 80 00 00 	mov    %rcx,0x80(%rsp)
    a222:	00 
    a223:	e9 97 fe ff ff       	jmp    a0bf <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x56f>
    a228:	48 89 c3             	mov    %rax,%rbx
    a22b:	e9 5a ff ff ff       	jmp    a18a <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x63a>
    a230:	48 89 c3             	mov    %rax,%rbx
    a233:	e9 34 ff ff ff       	jmp    a16c <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E+0x61c>
    a238:	0f 1f 84 00 00 00 00 	nopl   0x0(%rax,%rax,1)
    a23f:	00 

000000000000a240 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE>:
    a240:	41 57                	push   %r15
    a242:	41 56                	push   %r14
    a244:	41 55                	push   %r13
    a246:	41 54                	push   %r12
    a248:	55                   	push   %rbp
    a249:	53                   	push   %rbx
    a24a:	48 89 f3             	mov    %rsi,%rbx
    a24d:	48 81 ec 88 00 00 00 	sub    $0x88,%rsp
    a254:	48 89 7c 24 18       	mov    %rdi,0x18(%rsp)
    a259:	48 8b 36             	mov    (%rsi),%rsi
    a25c:	64 48 8b 04 25 28 00 	mov    %fs:0x28,%rax
    a263:	00 00 
    a265:	48 89 44 24 78       	mov    %rax,0x78(%rsp)
    a26a:	31 c0                	xor    %eax,%eax
    a26c:	48 8b 43 08          	mov    0x8(%rbx),%rax
    a270:	48 83 7f 08 00       	cmpq   $0x0,0x8(%rdi)
    a275:	48 89 44 24 08       	mov    %rax,0x8(%rsp)
    a27a:	75 16                	jne    a292 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x52>
    a27c:	48 39 f0             	cmp    %rsi,%rax
    a27f:	74 11                	je     a292 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x52>
    a281:	e8 1a 8f ff ff       	call   31a0 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE9_M_assignERKS4_@plt>
    a286:	48 8b 43 08          	mov    0x8(%rbx),%rax
    a28a:	48 8b 33             	mov    (%rbx),%rsi
    a28d:	48 89 44 24 08       	mov    %rax,0x8(%rsp)
    a292:	48 8b 44 24 18       	mov    0x18(%rsp),%rax
    a297:	48 8b 80 88 00 00 00 	mov    0x88(%rax),%rax
    a29e:	48 89 44 24 28       	mov    %rax,0x28(%rsp)
    a2a3:	48 8d 46 20          	lea    0x20(%rsi),%rax
    a2a7:	48 89 04 24          	mov    %rax,(%rsp)
    a2ab:	48 3b 44 24 08       	cmp    0x8(%rsp),%rax
    a2b0:	75 5b                	jne    a30d <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0xcd>
    a2b2:	e9 89 04 00 00       	jmp    a740 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x500>
    a2b7:	66 0f 1f 84 00 00 00 	nopw   0x0(%rax,%rax,1)
    a2be:	00 00 
    a2c0:	48 8b 44 24 18       	mov    0x18(%rsp),%rax
    a2c5:	48 05 88 00 00 00    	add    $0x88,%rax
    a2cb:	48 39 44 24 28       	cmp    %rax,0x28(%rsp)
    a2d0:	0f 84 5e 0a 00 00    	je     ad34 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0xaf4>
    a2d6:	48 8b 44 24 28       	mov    0x28(%rsp),%rax
    a2db:	48 8b 54 24 08       	mov    0x8(%rsp),%rdx
    a2e0:	48 8b 34 24          	mov    (%rsp),%rsi
    a2e4:	48 8b 18             	mov    (%rax),%rbx
    a2e7:	48 8d 78 10          	lea    0x10(%rax),%rdi
    a2eb:	31 c9                	xor    %ecx,%ecx
    a2ed:	45 31 c0             	xor    %r8d,%r8d
    a2f0:	e8 5b f8 ff ff       	call   9b50 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E>
    a2f5:	48 89 04 24          	mov    %rax,(%rsp)
    a2f9:	48 89 5c 24 28       	mov    %rbx,0x28(%rsp)
    a2fe:	48 8b 4c 24 08       	mov    0x8(%rsp),%rcx
    a303:	48 39 0c 24          	cmp    %rcx,(%rsp)
    a307:	0f 84 33 04 00 00    	je     a740 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x500>
    a30d:	48 8b 04 24          	mov    (%rsp),%rax
    a311:	4c 8b 68 08          	mov    0x8(%rax),%r13
    a315:	48 8b 28             	mov    (%rax),%rbp
    a318:	4d 85 ed             	test   %r13,%r13
    a31b:	74 a3                	je     a2c0 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x80>
    a31d:	80 7d 00 2d          	cmpb   $0x2d,0x0(%rbp)
    a321:	75 9d                	jne    a2c0 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x80>
    a323:	4c 89 e9             	mov    %r13,%rcx
    a326:	48 ff c9             	dec    %rcx
    a329:	74 95                	je     a2c0 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x80>
    a32b:	0f b6 45 01          	movzbl 0x1(%rbp),%eax
    a32f:	3c 30                	cmp    $0x30,%al
    a331:	0f 84 2c 05 00 00    	je     a863 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x623>
    a337:	0f 8f 97 04 00 00    	jg     a7d4 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x594>
    a33d:	3c 2e                	cmp    $0x2e,%al
    a33f:	0f 85 db 00 00 00    	jne    a420 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x1e0>
    a345:	4c 89 e9             	mov    %r13,%rcx
    a348:	48 8d 7d 02          	lea    0x2(%rbp),%rdi
    a34c:	48 83 e9 02          	sub    $0x2,%rcx
    a350:	0f 84 ca 00 00 00    	je     a420 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x1e0>
    a356:	0f b6 07             	movzbl (%rdi),%eax
    a359:	83 e8 30             	sub    $0x30,%eax
    a35c:	83 f8 09             	cmp    $0x9,%eax
    a35f:	0f 87 bb 00 00 00    	ja     a420 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x1e0>
    a365:	48 89 ce             	mov    %rcx,%rsi
    a368:	48 c1 fe 02          	sar    $0x2,%rsi
    a36c:	48 8d 14 0f          	lea    (%rdi,%rcx,1),%rdx
    a370:	48 89 c8             	mov    %rcx,%rax
    a373:	48 85 f6             	test   %rsi,%rsi
    a376:	0f 8e a6 07 00 00    	jle    ab22 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x8e2>
    a37c:	4c 8d 14 b7          	lea    (%rdi,%rsi,4),%r10
    a380:	48 89 fe             	mov    %rdi,%rsi
    a383:	0f be 06             	movsbl (%rsi),%eax
    a386:	83 e8 30             	sub    $0x30,%eax
    a389:	83 f8 09             	cmp    $0x9,%eax
    a38c:	77 68                	ja     a3f6 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x1b6>
    a38e:	0f be 46 01          	movsbl 0x1(%rsi),%eax
    a392:	83 e8 30             	sub    $0x30,%eax
    a395:	83 f8 09             	cmp    $0x9,%eax
    a398:	0f 87 6e 06 00 00    	ja     aa0c <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x7cc>
    a39e:	0f be 46 02          	movsbl 0x2(%rsi),%eax
    a3a2:	83 e8 30             	sub    $0x30,%eax
    a3a5:	83 f8 09             	cmp    $0x9,%eax
    a3a8:	0f 86 12 07 00 00    	jbe    aac0 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x880>
    a3ae:	48 83 c6 02          	add    $0x2,%rsi
    a3b2:	48 89 f2             	mov    %rsi,%rdx
    a3b5:	48 29 fa             	sub    %rdi,%rdx
    a3b8:	eb 42                	jmp    a3fc <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x1bc>
    a3ba:	48 83 c6 04          	add    $0x4,%rsi
    a3be:	49 39 f2             	cmp    %rsi,%r10
    a3c1:	75 c0                	jne    a383 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x143>
    a3c3:	48 89 d0             	mov    %rdx,%rax
    a3c6:	48 29 f0             	sub    %rsi,%rax
    a3c9:	48 83 f8 02          	cmp    $0x2,%rax
    a3cd:	0f 84 37 08 00 00    	je     ac0a <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x9ca>
    a3d3:	48 83 f8 03          	cmp    $0x3,%rax
    a3d7:	0f 84 1b 08 00 00    	je     abf8 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x9b8>
    a3dd:	48 83 f8 01          	cmp    $0x1,%rax
    a3e1:	0f 85 d9 fe ff ff    	jne    a2c0 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x80>
    a3e7:	0f be 06             	movsbl (%rsi),%eax
    a3ea:	83 e8 30             	sub    $0x30,%eax
    a3ed:	83 f8 09             	cmp    $0x9,%eax
    a3f0:	0f 86 ca fe ff ff    	jbe    a2c0 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x80>
    a3f6:	48 89 f2             	mov    %rsi,%rdx
    a3f9:	48 29 fa             	sub    %rdi,%rdx
    a3fc:	48 39 d1             	cmp    %rdx,%rcx
    a3ff:	0f 82 24 08 00 00    	jb     ac29 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x9e9>
    a405:	48 29 d1             	sub    %rdx,%rcx
    a408:	0f 84 b2 fe ff ff    	je     a2c0 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x80>
    a40e:	0f b6 06             	movzbl (%rsi),%eax
    a411:	83 e0 df             	and    $0xffffffdf,%eax
    a414:	3c 45                	cmp    $0x45,%al
    a416:	0f 84 0c 05 00 00    	je     a928 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x6e8>
    a41c:	0f 1f 40 00          	nopl   0x0(%rax)
    a420:	48 8b 44 24 18       	mov    0x18(%rsp),%rax
    a425:	4c 8b b0 c8 00 00 00 	mov    0xc8(%rax),%r14
    a42c:	48 05 c0 00 00 00    	add    $0xc0,%rax
    a432:	48 89 44 24 10       	mov    %rax,0x10(%rsp)
    a437:	4d 85 f6             	test   %r14,%r14
    a43a:	0f 84 c0 00 00 00    	je     a500 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x2c0>
    a440:	48 89 c3             	mov    %rax,%rbx
    a443:	4d 89 f7             	mov    %r14,%r15
    a446:	eb 11                	jmp    a459 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x219>
    a448:	0f 1f 84 00 00 00 00 	nopl   0x0(%rax,%rax,1)
    a44f:	00 
    a450:	4d 8b 7f 18          	mov    0x18(%r15),%r15
    a454:	4d 85 ff             	test   %r15,%r15
    a457:	74 52                	je     a4ab <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x26b>
    a459:	4d 8b 67 20          	mov    0x20(%r15),%r12
    a45d:	4d 39 e5             	cmp    %r12,%r13
    a460:	4c 89 e2             	mov    %r12,%rdx
    a463:	49 0f 46 d5          	cmovbe %r13,%rdx
    a467:	48 85 d2             	test   %rdx,%rdx
    a46a:	74 10                	je     a47c <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x23c>
    a46c:	49 8b 7f 28          	mov    0x28(%r15),%rdi
    a470:	48 89 ee             	mov    %rbp,%rsi
    a473:	e8 f8 8d ff ff       	call   3270 <memcmp@plt>
    a478:	85 c0                	test   %eax,%eax
    a47a:	75 1f                	jne    a49b <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x25b>
    a47c:	4d 29 ec             	sub    %r13,%r12
    a47f:	b8 00 00 00 80       	mov    $0x80000000,%eax
    a484:	49 39 c4             	cmp    %rax,%r12
    a487:	7d 16                	jge    a49f <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x25f>
    a489:	48 b8 ff ff ff 7f ff 	movabs $0xffffffff7fffffff,%rax
    a490:	ff ff ff 
    a493:	49 39 c4             	cmp    %rax,%r12
    a496:	7e b8                	jle    a450 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x210>
    a498:	44 89 e0             	mov    %r12d,%eax
    a49b:	85 c0                	test   %eax,%eax
    a49d:	78 b1                	js     a450 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x210>
    a49f:	4c 89 fb             	mov    %r15,%rbx
    a4a2:	4d 8b 7f 10          	mov    0x10(%r15),%r15
    a4a6:	4d 85 ff             	test   %r15,%r15
    a4a9:	75 ae                	jne    a459 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x219>
    a4ab:	48 39 5c 24 10       	cmp    %rbx,0x10(%rsp)
    a4b0:	74 4e                	je     a500 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x2c0>
    a4b2:	4c 8b 63 20          	mov    0x20(%rbx),%r12
    a4b6:	4d 39 e5             	cmp    %r12,%r13
    a4b9:	4c 89 e2             	mov    %r12,%rdx
    a4bc:	49 0f 46 d5          	cmovbe %r13,%rdx
    a4c0:	48 85 d2             	test   %rdx,%rdx
    a4c3:	74 10                	je     a4d5 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x295>
    a4c5:	48 8b 73 28          	mov    0x28(%rbx),%rsi
    a4c9:	48 89 ef             	mov    %rbp,%rdi
    a4cc:	e8 9f 8d ff ff       	call   3270 <memcmp@plt>
    a4d1:	85 c0                	test   %eax,%eax
    a4d3:	75 23                	jne    a4f8 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x2b8>
    a4d5:	4c 89 e8             	mov    %r13,%rax
    a4d8:	4c 29 e0             	sub    %r12,%rax
    a4db:	be 00 00 00 80       	mov    $0x80000000,%esi
    a4e0:	48 39 f0             	cmp    %rsi,%rax
    a4e3:	0f 8d 8f 02 00 00    	jge    a778 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x538>
    a4e9:	48 be ff ff ff 7f ff 	movabs $0xffffffff7fffffff,%rsi
    a4f0:	ff ff ff 
    a4f3:	48 39 f0             	cmp    %rsi,%rax
    a4f6:	7e 08                	jle    a500 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x2c0>
    a4f8:	85 c0                	test   %eax,%eax
    a4fa:	0f 89 78 02 00 00    	jns    a778 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x538>
    a500:	80 7d 01 2d          	cmpb   $0x2d,0x1(%rbp)
    a504:	0f 84 c7 07 00 00    	je     acd1 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0xa91>
    a50a:	48 8b 04 24          	mov    (%rsp),%rax
    a50e:	41 bc 01 00 00 00    	mov    $0x1,%r12d
    a514:	4c 8d 78 20          	lea    0x20(%rax),%r15
    a518:	48 8d 5c 24 40       	lea    0x40(%rsp),%rbx
    a51d:	0f 1f 00             	nopl   (%rax)
    a520:	0f b7 4c 24 26       	movzwl 0x26(%rsp),%ecx
    a525:	42 0f b6 44 25 00    	movzbl 0x0(%rbp,%r12,1),%eax
    a52b:	b1 2d                	mov    $0x2d,%cl
    a52d:	88 c5                	mov    %al,%ch
    a52f:	66 89 4c 24 26       	mov    %cx,0x26(%rsp)
    a534:	48 89 5c 24 30       	mov    %rbx,0x30(%rsp)
    a539:	66 89 4c 24 40       	mov    %cx,0x40(%rsp)
    a53e:	48 c7 44 24 38 02 00 	movq   $0x2,0x38(%rsp)
    a545:	00 00 
    a547:	c6 44 24 42 00       	movb   $0x0,0x42(%rsp)
    a54c:	4d 85 f6             	test   %r14,%r14
    a54f:	0f 84 99 00 00 00    	je     a5ee <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x3ae>
    a555:	48 8b 6c 24 10       	mov    0x10(%rsp),%rbp
    a55a:	eb 0d                	jmp    a569 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x329>
    a55c:	0f 1f 40 00          	nopl   0x0(%rax)
    a560:	4d 8b 76 18          	mov    0x18(%r14),%r14
    a564:	4d 85 f6             	test   %r14,%r14
    a567:	74 2e                	je     a597 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x357>
    a569:	4d 8b 6e 20          	mov    0x20(%r14),%r13
    a56d:	49 83 fd 02          	cmp    $0x2,%r13
    a571:	0f 87 e1 00 00 00    	ja     a658 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x418>
    a577:	48 c7 c0 fe ff ff ff 	mov    $0xfffffffffffffffe,%rax
    a57e:	4d 85 ed             	test   %r13,%r13
    a581:	0f 85 19 01 00 00    	jne    a6a0 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x460>
    a587:	85 c0                	test   %eax,%eax
    a589:	78 d5                	js     a560 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x320>
    a58b:	4c 89 f5             	mov    %r14,%rbp
    a58e:	4d 8b 76 10          	mov    0x10(%r14),%r14
    a592:	4d 85 f6             	test   %r14,%r14
    a595:	75 d2                	jne    a569 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x329>
    a597:	48 3b 6c 24 10       	cmp    0x10(%rsp),%rbp
    a59c:	74 50                	je     a5ee <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x3ae>
    a59e:	4c 8b 6d 20          	mov    0x20(%rbp),%r13
    a5a2:	49 83 fd 01          	cmp    $0x1,%r13
    a5a6:	0f 86 fc 01 00 00    	jbe    a7a8 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x568>
    a5ac:	48 8b 75 28          	mov    0x28(%rbp),%rsi
    a5b0:	ba 02 00 00 00       	mov    $0x2,%edx
    a5b5:	48 89 df             	mov    %rbx,%rdi
    a5b8:	e8 b3 8c ff ff       	call   3270 <memcmp@plt>
    a5bd:	85 c0                	test   %eax,%eax
    a5bf:	75 25                	jne    a5e6 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x3a6>
    a5c1:	b8 02 00 00 00       	mov    $0x2,%eax
    a5c6:	4c 29 e8             	sub    %r13,%rax
    a5c9:	be 00 00 00 80       	mov    $0x80000000,%esi
    a5ce:	48 39 f0             	cmp    %rsi,%rax
    a5d1:	0f 8d e9 00 00 00    	jge    a6c0 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x480>
    a5d7:	48 be ff ff ff 7f ff 	movabs $0xffffffff7fffffff,%rsi
    a5de:	ff ff ff 
    a5e1:	48 39 f0             	cmp    %rsi,%rax
    a5e4:	7e 08                	jle    a5ee <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x3ae>
    a5e6:	85 c0                	test   %eax,%eax
    a5e8:	0f 89 d2 00 00 00    	jns    a6c0 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x480>
    a5ee:	bf 10 00 00 00       	mov    $0x10,%edi
    a5f3:	e8 d8 8a ff ff       	call   30d0 <__cxa_allocate_exception@plt>
    a5f8:	4c 8d 6c 24 50       	lea    0x50(%rsp),%r13
    a5fd:	48 8b 14 24          	mov    (%rsp),%rdx
    a601:	48 8d 35 98 5d 00 00 	lea    0x5d98(%rip),%rsi        # 103a0 <_fini+0x117f>
    a608:	4c 89 ef             	mov    %r13,%rdi
    a60b:	49 89 c4             	mov    %rax,%r12
    a60e:	e8 7d e4 ff ff       	call   8a90 <_ZStplIcSt11char_traitsIcESaIcEENSt7__cxx1112basic_stringIT_T0_T1_EEPKS5_RKS8_>
    a613:	4c 89 ee             	mov    %r13,%rsi
    a616:	4c 89 e7             	mov    %r12,%rdi
    a619:	e8 72 8c ff ff       	call   3290 <_ZNSt13runtime_errorC1ERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEE@plt>
    a61e:	48 8b 7c 24 50       	mov    0x50(%rsp),%rdi
    a623:	48 8d 44 24 60       	lea    0x60(%rsp),%rax
    a628:	48 39 c7             	cmp    %rax,%rdi
    a62b:	74 0e                	je     a63b <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x3fb>
    a62d:	48 8b 44 24 60       	mov    0x60(%rsp),%rax
    a632:	48 8d 70 01          	lea    0x1(%rax),%rsi
    a636:	e8 e5 8c ff ff       	call   3320 <_ZdlPvm@plt>
    a63b:	48 8b 15 86 99 00 00 	mov    0x9986(%rip),%rdx        # 13fc8 <_ZNSt13runtime_errorD1Ev@Base>
    a642:	48 8d 35 57 93 00 00 	lea    0x9357(%rip),%rsi        # 139a0 <_ZTISt13runtime_error@@Base>
    a649:	4c 89 e7             	mov    %r12,%rdi
    a64c:	e8 cf 8b ff ff       	call   3220 <__cxa_throw@plt>
    a651:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
    a658:	49 8b 7e 28          	mov    0x28(%r14),%rdi
    a65c:	ba 02 00 00 00       	mov    $0x2,%edx
    a661:	48 89 de             	mov    %rbx,%rsi
    a664:	e8 07 8c ff ff       	call   3270 <memcmp@plt>
    a669:	85 c0                	test   %eax,%eax
    a66b:	0f 85 16 ff ff ff    	jne    a587 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x347>
    a671:	49 8d 45 fe          	lea    -0x2(%r13),%rax
    a675:	be 00 00 00 80       	mov    $0x80000000,%esi
    a67a:	48 39 f0             	cmp    %rsi,%rax
    a67d:	0f 8d 08 ff ff ff    	jge    a58b <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x34b>
    a683:	48 be ff ff ff 7f ff 	movabs $0xffffffff7fffffff,%rsi
    a68a:	ff ff ff 
    a68d:	48 39 f0             	cmp    %rsi,%rax
    a690:	0f 8e ca fe ff ff    	jle    a560 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x320>
    a696:	e9 ec fe ff ff       	jmp    a587 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x347>
    a69b:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)
    a6a0:	49 8b 7e 28          	mov    0x28(%r14),%rdi
    a6a4:	4c 89 ea             	mov    %r13,%rdx
    a6a7:	48 89 de             	mov    %rbx,%rsi
    a6aa:	e8 c1 8b ff ff       	call   3270 <memcmp@plt>
    a6af:	85 c0                	test   %eax,%eax
    a6b1:	0f 85 d0 fe ff ff    	jne    a587 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x347>
    a6b7:	49 8d 45 fe          	lea    -0x2(%r13),%rax
    a6bb:	e9 c7 fe ff ff       	jmp    a587 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x347>
    a6c0:	48 8b 45 30          	mov    0x30(%rbp),%rax
    a6c4:	48 8b 4d 20          	mov    0x20(%rbp),%rcx
    a6c8:	4c 8b 45 28          	mov    0x28(%rbp),%r8
    a6cc:	48 8b 54 24 08       	mov    0x8(%rsp),%rdx
    a6d1:	48 8d 78 10          	lea    0x10(%rax),%rdi
    a6d5:	4c 89 fe             	mov    %r15,%rsi
    a6d8:	e8 73 f4 ff ff       	call   9b50 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E>
    a6dd:	48 8b 7c 24 30       	mov    0x30(%rsp),%rdi
    a6e2:	48 89 c5             	mov    %rax,%rbp
    a6e5:	49 89 c7             	mov    %rax,%r15
    a6e8:	48 39 df             	cmp    %rbx,%rdi
    a6eb:	74 33                	je     a720 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x4e0>
    a6ed:	48 8b 44 24 40       	mov    0x40(%rsp),%rax
    a6f2:	49 ff c4             	inc    %r12
    a6f5:	48 8d 70 01          	lea    0x1(%rax),%rsi
    a6f9:	e8 22 8c ff ff       	call   3320 <_ZdlPvm@plt>
    a6fe:	48 8b 04 24          	mov    (%rsp),%rax
    a702:	4c 39 60 08          	cmp    %r12,0x8(%rax)
    a706:	76 25                	jbe    a72d <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x4ed>
    a708:	48 8b 44 24 18       	mov    0x18(%rsp),%rax
    a70d:	4c 8b b0 c8 00 00 00 	mov    0xc8(%rax),%r14
    a714:	48 8b 04 24          	mov    (%rsp),%rax
    a718:	48 8b 28             	mov    (%rax),%rbp
    a71b:	e9 00 fe ff ff       	jmp    a520 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x2e0>
    a720:	48 8b 04 24          	mov    (%rsp),%rax
    a724:	49 ff c4             	inc    %r12
    a727:	4c 3b 60 08          	cmp    0x8(%rax),%r12
    a72b:	72 db                	jb     a708 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x4c8>
    a72d:	48 89 2c 24          	mov    %rbp,(%rsp)
    a731:	48 8b 4c 24 08       	mov    0x8(%rsp),%rcx
    a736:	48 39 0c 24          	cmp    %rcx,(%rsp)
    a73a:	0f 85 cd fb ff ff    	jne    a30d <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0xcd>
    a740:	48 8b 44 24 18       	mov    0x18(%rsp),%rax
    a745:	c6 80 80 00 00 00 01 	movb   $0x1,0x80(%rax)
    a74c:	48 8b 44 24 78       	mov    0x78(%rsp),%rax
    a751:	64 48 2b 04 25 28 00 	sub    %fs:0x28,%rax
    a758:	00 00 
    a75a:	0f 85 c1 04 00 00    	jne    ac21 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x9e1>
    a760:	48 81 c4 88 00 00 00 	add    $0x88,%rsp
    a767:	5b                   	pop    %rbx
    a768:	5d                   	pop    %rbp
    a769:	41 5c                	pop    %r12
    a76b:	41 5d                	pop    %r13
    a76d:	41 5e                	pop    %r14
    a76f:	41 5f                	pop    %r15
    a771:	c3                   	ret    
    a772:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)
    a778:	48 8b 43 30          	mov    0x30(%rbx),%rax
    a77c:	48 8b 34 24          	mov    (%rsp),%rsi
    a780:	48 8b 4b 20          	mov    0x20(%rbx),%rcx
    a784:	4c 8b 43 28          	mov    0x28(%rbx),%r8
    a788:	48 8b 54 24 08       	mov    0x8(%rsp),%rdx
    a78d:	48 8d 78 10          	lea    0x10(%rax),%rdi
    a791:	48 83 c6 20          	add    $0x20,%rsi
    a795:	e8 b6 f3 ff ff       	call   9b50 <_ZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS9_SaIS9_EEEEEET_SG_SG_St17basic_string_viewIcS7_E>
    a79a:	48 89 04 24          	mov    %rax,(%rsp)
    a79e:	e9 5b fb ff ff       	jmp    a2fe <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0xbe>
    a7a3:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)
    a7a8:	b8 02 00 00 00       	mov    $0x2,%eax
    a7ad:	4d 85 ed             	test   %r13,%r13
    a7b0:	0f 84 30 fe ff ff    	je     a5e6 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x3a6>
    a7b6:	48 8b 55 28          	mov    0x28(%rbp),%rdx
    a7ba:	0f b6 44 24 40       	movzbl 0x40(%rsp),%eax
    a7bf:	0f b6 12             	movzbl (%rdx),%edx
    a7c2:	29 d0                	sub    %edx,%eax
    a7c4:	0f 85 1c fe ff ff    	jne    a5e6 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x3a6>
    a7ca:	b8 01 00 00 00       	mov    $0x1,%eax
    a7cf:	e9 12 fe ff ff       	jmp    a5e6 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x3a6>
    a7d4:	83 e8 31             	sub    $0x31,%eax
    a7d7:	3c 08                	cmp    $0x8,%al
    a7d9:	0f 87 41 fc ff ff    	ja     a420 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x1e0>
    a7df:	48 89 c8             	mov    %rcx,%rax
    a7e2:	48 c1 f8 02          	sar    $0x2,%rax
    a7e6:	48 8d 75 01          	lea    0x1(%rbp),%rsi
    a7ea:	48 85 c0             	test   %rax,%rax
    a7ed:	0f 8e 7a 02 00 00    	jle    aa6d <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x82d>
    a7f3:	48 8d 7c 85 01       	lea    0x1(%rbp,%rax,4),%rdi
    a7f8:	48 89 f0             	mov    %rsi,%rax
    a7fb:	0f be 10             	movsbl (%rax),%edx
    a7fe:	83 ea 30             	sub    $0x30,%edx
    a801:	83 fa 09             	cmp    $0x9,%edx
    a804:	0f 86 06 01 00 00    	jbe    a910 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x6d0>
    a80a:	48 29 f0             	sub    %rsi,%rax
    a80d:	48 39 c1             	cmp    %rax,%rcx
    a810:	0f 82 10 04 00 00    	jb     ac26 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x9e6>
    a816:	48 29 c1             	sub    %rax,%rcx
    a819:	48 01 f0             	add    %rsi,%rax
    a81c:	48 83 f9 ff          	cmp    $0xffffffffffffffff,%rcx
    a820:	0f 85 8a 01 00 00    	jne    a9b0 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x770>
    a826:	0f b6 10             	movzbl (%rax),%edx
    a829:	89 d1                	mov    %edx,%ecx
    a82b:	83 e1 df             	and    $0xffffffdf,%ecx
    a82e:	80 f9 45             	cmp    $0x45,%cl
    a831:	0f 84 00 02 00 00    	je     aa37 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x7f7>
    a837:	48 8d 70 01          	lea    0x1(%rax),%rsi
    a83b:	48 c7 c1 fe ff ff ff 	mov    $0xfffffffffffffffe,%rcx
    a842:	80 fa 2e             	cmp    $0x2e,%dl
    a845:	0f 85 d5 fb ff ff    	jne    a420 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x1e0>
    a84b:	0f b6 40 01          	movzbl 0x1(%rax),%eax
    a84f:	48 89 f7             	mov    %rsi,%rdi
    a852:	83 e8 30             	sub    $0x30,%eax
    a855:	83 f8 09             	cmp    $0x9,%eax
    a858:	0f 86 f8 fa ff ff    	jbe    a356 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x116>
    a85e:	e9 ab fb ff ff       	jmp    a40e <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x1ce>
    a863:	4c 89 e9             	mov    %r13,%rcx
    a866:	48 83 e9 02          	sub    $0x2,%rcx
    a86a:	0f 84 50 fa ff ff    	je     a2c0 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x80>
    a870:	48 89 c8             	mov    %rcx,%rax
    a873:	48 8d 75 02          	lea    0x2(%rbp),%rsi
    a877:	48 c1 f8 02          	sar    $0x2,%rax
    a87b:	48 8d 3c 0e          	lea    (%rsi,%rcx,1),%rdi
    a87f:	48 89 ca             	mov    %rcx,%rdx
    a882:	48 85 c0             	test   %rax,%rax
    a885:	0f 8e 65 03 00 00    	jle    abf0 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x9b0>
    a88b:	4c 8d 54 85 02       	lea    0x2(%rbp,%rax,4),%r10
    a890:	48 89 f0             	mov    %rsi,%rax
    a893:	0f be 10             	movsbl (%rax),%edx
    a896:	83 ea 30             	sub    $0x30,%edx
    a899:	83 fa 09             	cmp    $0x9,%edx
    a89c:	77 24                	ja     a8c2 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x682>
    a89e:	0f be 50 01          	movsbl 0x1(%rax),%edx
    a8a2:	83 ea 30             	sub    $0x30,%edx
    a8a5:	83 fa 09             	cmp    $0x9,%edx
    a8a8:	0f 87 b7 01 00 00    	ja     aa65 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x825>
    a8ae:	0f be 50 02          	movsbl 0x2(%rax),%edx
    a8b2:	83 ea 30             	sub    $0x30,%edx
    a8b5:	83 fa 09             	cmp    $0x9,%edx
    a8b8:	0f 86 3a 02 00 00    	jbe    aaf8 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x8b8>
    a8be:	48 83 c0 02          	add    $0x2,%rax
    a8c2:	48 89 c2             	mov    %rax,%rdx
    a8c5:	48 29 f2             	sub    %rsi,%rdx
    a8c8:	48 39 d1             	cmp    %rdx,%rcx
    a8cb:	0f 82 58 03 00 00    	jb     ac29 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x9e9>
    a8d1:	48 29 d1             	sub    %rdx,%rcx
    a8d4:	0f 84 46 fb ff ff    	je     a420 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x1e0>
    a8da:	0f b6 10             	movzbl (%rax),%edx
    a8dd:	89 d6                	mov    %edx,%esi
    a8df:	83 e6 df             	and    $0xffffffdf,%esi
    a8e2:	40 80 fe 45          	cmp    $0x45,%sil
    a8e6:	0f 84 14 01 00 00    	je     aa00 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x7c0>
    a8ec:	80 fa 2e             	cmp    $0x2e,%dl
    a8ef:	0f 85 2b fb ff ff    	jne    a420 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x1e0>
    a8f5:	48 8d 70 01          	lea    0x1(%rax),%rsi
    a8f9:	48 ff c9             	dec    %rcx
    a8fc:	0f 85 49 ff ff ff    	jne    a84b <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x60b>
    a902:	e9 b9 f9 ff ff       	jmp    a2c0 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x80>
    a907:	66 0f 1f 84 00 00 00 	nopw   0x0(%rax,%rax,1)
    a90e:	00 00 
    a910:	0f be 50 01          	movsbl 0x1(%rax),%edx
    a914:	83 ea 30             	sub    $0x30,%edx
    a917:	83 fa 09             	cmp    $0x9,%edx
    a91a:	0f 86 a0 00 00 00    	jbe    a9c0 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x780>
    a920:	48 ff c0             	inc    %rax
    a923:	e9 e2 fe ff ff       	jmp    a80a <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x5ca>
    a928:	48 ff c6             	inc    %rsi
    a92b:	48 ff c9             	dec    %rcx
    a92e:	48 85 c9             	test   %rcx,%rcx
    a931:	0f 84 e9 fa ff ff    	je     a420 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x1e0>
    a937:	0f b6 06             	movzbl (%rsi),%eax
    a93a:	83 e8 2b             	sub    $0x2b,%eax
    a93d:	a8 fd                	test   $0xfd,%al
    a93f:	0f 84 a0 00 00 00    	je     a9e5 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x7a5>
    a945:	0f b6 06             	movzbl (%rsi),%eax
    a948:	83 e8 30             	sub    $0x30,%eax
    a94b:	83 f8 09             	cmp    $0x9,%eax
    a94e:	0f 87 cc fa ff ff    	ja     a420 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x1e0>
    a954:	48 89 c8             	mov    %rcx,%rax
    a957:	48 c1 f8 02          	sar    $0x2,%rax
    a95b:	48 8d 3c 0e          	lea    (%rsi,%rcx,1),%rdi
    a95f:	48 89 ca             	mov    %rcx,%rdx
    a962:	48 85 c0             	test   %rax,%rax
    a965:	0f 8e fc 02 00 00    	jle    ac67 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0xa27>
    a96b:	4c 8d 14 86          	lea    (%rsi,%rax,4),%r10
    a96f:	48 89 f0             	mov    %rsi,%rax
    a972:	0f be 10             	movsbl (%rax),%edx
    a975:	83 ea 30             	sub    $0x30,%edx
    a978:	83 fa 09             	cmp    $0x9,%edx
    a97b:	0f 86 a2 00 00 00    	jbe    aa23 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x7e3>
    a981:	48 29 f0             	sub    %rsi,%rax
    a984:	48 89 c2             	mov    %rax,%rdx
    a987:	48 39 c8             	cmp    %rcx,%rax
    a98a:	0f 87 99 02 00 00    	ja     ac29 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x9e9>
    a990:	48 29 c1             	sub    %rax,%rcx
    a993:	48 83 f9 ff          	cmp    $0xffffffffffffffff,%rcx
    a997:	0f 84 83 fa ff ff    	je     a420 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x1e0>
    a99d:	48 85 c9             	test   %rcx,%rcx
    a9a0:	0f 85 7a fa ff ff    	jne    a420 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x1e0>
    a9a6:	e9 15 f9 ff ff       	jmp    a2c0 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x80>
    a9ab:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)
    a9b0:	48 85 c9             	test   %rcx,%rcx
    a9b3:	0f 84 07 f9 ff ff    	je     a2c0 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x80>
    a9b9:	e9 1c ff ff ff       	jmp    a8da <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x69a>
    a9be:	66 90                	xchg   %ax,%ax
    a9c0:	0f be 50 02          	movsbl 0x2(%rax),%edx
    a9c4:	83 ea 30             	sub    $0x30,%edx
    a9c7:	83 fa 09             	cmp    $0x9,%edx
    a9ca:	77 4e                	ja     aa1a <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x7da>
    a9cc:	0f be 50 03          	movsbl 0x3(%rax),%edx
    a9d0:	83 ea 30             	sub    $0x30,%edx
    a9d3:	83 fa 09             	cmp    $0x9,%edx
    a9d6:	0f 86 03 01 00 00    	jbe    aadf <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x89f>
    a9dc:	48 83 c0 03          	add    $0x3,%rax
    a9e0:	e9 25 fe ff ff       	jmp    a80a <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x5ca>
    a9e5:	48 ff c6             	inc    %rsi
    a9e8:	48 ff c9             	dec    %rcx
    a9eb:	0f 85 54 ff ff ff    	jne    a945 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x705>
    a9f1:	e9 2a fa ff ff       	jmp    a420 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x1e0>
    a9f6:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
    a9fd:	00 00 00 
    aa00:	48 ff c9             	dec    %rcx
    aa03:	48 8d 70 01          	lea    0x1(%rax),%rsi
    aa07:	e9 22 ff ff ff       	jmp    a92e <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x6ee>
    aa0c:	48 ff c6             	inc    %rsi
    aa0f:	48 89 f2             	mov    %rsi,%rdx
    aa12:	48 29 fa             	sub    %rdi,%rdx
    aa15:	e9 e2 f9 ff ff       	jmp    a3fc <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x1bc>
    aa1a:	48 83 c0 02          	add    $0x2,%rax
    aa1e:	e9 e7 fd ff ff       	jmp    a80a <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x5ca>
    aa23:	0f be 50 01          	movsbl 0x1(%rax),%edx
    aa27:	83 ea 30             	sub    $0x30,%edx
    aa2a:	83 fa 09             	cmp    $0x9,%edx
    aa2d:	76 7c                	jbe    aaab <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x86b>
    aa2f:	48 ff c0             	inc    %rax
    aa32:	e9 4a ff ff ff       	jmp    a981 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x741>
    aa37:	0f b6 58 01          	movzbl 0x1(%rax),%ebx
    aa3b:	48 8d 70 01          	lea    0x1(%rax),%rsi
    aa3f:	8d 53 d5             	lea    -0x2b(%rbx),%edx
    aa42:	81 e2 fd 00 00 00    	and    $0xfd,%edx
    aa48:	48 c7 c1 fe ff ff ff 	mov    $0xfffffffffffffffe,%rcx
    aa4f:	0f 85 f0 fe ff ff    	jne    a945 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x705>
    aa55:	48 8d 70 02          	lea    0x2(%rax),%rsi
    aa59:	48 c7 c1 fd ff ff ff 	mov    $0xfffffffffffffffd,%rcx
    aa60:	e9 e0 fe ff ff       	jmp    a945 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x705>
    aa65:	48 ff c0             	inc    %rax
    aa68:	e9 55 fe ff ff       	jmp    a8c2 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x682>
    aa6d:	48 89 f2             	mov    %rsi,%rdx
    aa70:	4a 8d 44 2d 00       	lea    0x0(%rbp,%r13,1),%rax
    aa75:	48 89 c7             	mov    %rax,%rdi
    aa78:	48 29 d7             	sub    %rdx,%rdi
    aa7b:	48 83 ff 02          	cmp    $0x2,%rdi
    aa7f:	0f 84 c7 01 00 00    	je     ac4c <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0xa0c>
    aa85:	48 83 ff 03          	cmp    $0x3,%rdi
    aa89:	0f 84 af 01 00 00    	je     ac3e <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x9fe>
    aa8f:	48 83 ff 01          	cmp    $0x1,%rdi
    aa93:	0f 85 71 fd ff ff    	jne    a80a <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x5ca>
    aa99:	0f be 3a             	movsbl (%rdx),%edi
    aa9c:	83 ef 30             	sub    $0x30,%edi
    aa9f:	83 ff 0a             	cmp    $0xa,%edi
    aaa2:	48 0f 43 c2          	cmovae %rdx,%rax
    aaa6:	e9 5f fd ff ff       	jmp    a80a <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x5ca>
    aaab:	0f be 50 02          	movsbl 0x2(%rax),%edx
    aaaf:	83 ea 30             	sub    $0x30,%edx
    aab2:	83 fa 09             	cmp    $0x9,%edx
    aab5:	76 56                	jbe    ab0d <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x8cd>
    aab7:	48 83 c0 02          	add    $0x2,%rax
    aabb:	e9 c1 fe ff ff       	jmp    a981 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x741>
    aac0:	0f be 46 03          	movsbl 0x3(%rsi),%eax
    aac4:	83 e8 30             	sub    $0x30,%eax
    aac7:	83 f8 09             	cmp    $0x9,%eax
    aaca:	0f 86 ea f8 ff ff    	jbe    a3ba <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x17a>
    aad0:	48 83 c6 03          	add    $0x3,%rsi
    aad4:	48 89 f2             	mov    %rsi,%rdx
    aad7:	48 29 fa             	sub    %rdi,%rdx
    aada:	e9 1d f9 ff ff       	jmp    a3fc <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x1bc>
    aadf:	48 83 c0 04          	add    $0x4,%rax
    aae3:	48 39 c7             	cmp    %rax,%rdi
    aae6:	0f 85 0f fd ff ff    	jne    a7fb <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x5bb>
    aaec:	48 89 fa             	mov    %rdi,%rdx
    aaef:	e9 7c ff ff ff       	jmp    aa70 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x830>
    aaf4:	0f 1f 40 00          	nopl   0x0(%rax)
    aaf8:	0f be 50 03          	movsbl 0x3(%rax),%edx
    aafc:	83 ea 30             	sub    $0x30,%edx
    aaff:	83 fa 09             	cmp    $0x9,%edx
    ab02:	76 26                	jbe    ab2a <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x8ea>
    ab04:	48 83 c0 03          	add    $0x3,%rax
    ab08:	e9 b5 fd ff ff       	jmp    a8c2 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x682>
    ab0d:	0f be 50 03          	movsbl 0x3(%rax),%edx
    ab11:	83 ea 30             	sub    $0x30,%edx
    ab14:	83 fa 09             	cmp    $0x9,%edx
    ab17:	76 3e                	jbe    ab57 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x917>
    ab19:	48 83 c0 03          	add    $0x3,%rax
    ab1d:	e9 5f fe ff ff       	jmp    a981 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x741>
    ab22:	48 89 fe             	mov    %rdi,%rsi
    ab25:	e9 9f f8 ff ff       	jmp    a3c9 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x189>
    ab2a:	48 83 c0 04          	add    $0x4,%rax
    ab2e:	49 39 c2             	cmp    %rax,%r10
    ab31:	0f 85 5c fd ff ff    	jne    a893 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x653>
    ab37:	48 89 fa             	mov    %rdi,%rdx
    ab3a:	48 29 c2             	sub    %rax,%rdx
    ab3d:	48 83 fa 02          	cmp    $0x2,%rdx
    ab41:	74 53                	je     ab96 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x956>
    ab43:	48 83 fa 03          	cmp    $0x3,%rdx
    ab47:	74 3b                	je     ab84 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x944>
    ab49:	48 83 fa 01          	cmp    $0x1,%rdx
    ab4d:	74 59                	je     aba8 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x968>
    ab4f:	48 89 f8             	mov    %rdi,%rax
    ab52:	e9 6b fd ff ff       	jmp    a8c2 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x682>
    ab57:	48 83 c0 04          	add    $0x4,%rax
    ab5b:	49 39 c2             	cmp    %rax,%r10
    ab5e:	0f 85 0e fe ff ff    	jne    a972 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x732>
    ab64:	48 89 fa             	mov    %rdi,%rdx
    ab67:	48 29 c2             	sub    %rax,%rdx
    ab6a:	48 83 fa 02          	cmp    $0x2,%rdx
    ab6e:	74 5c                	je     abcc <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x98c>
    ab70:	48 83 fa 03          	cmp    $0x3,%rdx
    ab74:	74 44                	je     abba <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x97a>
    ab76:	48 83 fa 01          	cmp    $0x1,%rdx
    ab7a:	74 62                	je     abde <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x99e>
    ab7c:	48 89 f8             	mov    %rdi,%rax
    ab7f:	e9 fd fd ff ff       	jmp    a981 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x741>
    ab84:	0f be 10             	movsbl (%rax),%edx
    ab87:	83 ea 30             	sub    $0x30,%edx
    ab8a:	83 fa 09             	cmp    $0x9,%edx
    ab8d:	0f 87 2f fd ff ff    	ja     a8c2 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x682>
    ab93:	48 ff c0             	inc    %rax
    ab96:	0f be 10             	movsbl (%rax),%edx
    ab99:	83 ea 30             	sub    $0x30,%edx
    ab9c:	83 fa 09             	cmp    $0x9,%edx
    ab9f:	0f 87 1d fd ff ff    	ja     a8c2 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x682>
    aba5:	48 ff c0             	inc    %rax
    aba8:	0f be 10             	movsbl (%rax),%edx
    abab:	83 ea 30             	sub    $0x30,%edx
    abae:	83 fa 09             	cmp    $0x9,%edx
    abb1:	48 0f 46 c7          	cmovbe %rdi,%rax
    abb5:	e9 08 fd ff ff       	jmp    a8c2 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x682>
    abba:	0f be 10             	movsbl (%rax),%edx
    abbd:	83 ea 30             	sub    $0x30,%edx
    abc0:	83 fa 09             	cmp    $0x9,%edx
    abc3:	0f 87 b8 fd ff ff    	ja     a981 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x741>
    abc9:	48 ff c0             	inc    %rax
    abcc:	0f be 10             	movsbl (%rax),%edx
    abcf:	83 ea 30             	sub    $0x30,%edx
    abd2:	83 fa 09             	cmp    $0x9,%edx
    abd5:	0f 87 a6 fd ff ff    	ja     a981 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x741>
    abdb:	48 ff c0             	inc    %rax
    abde:	0f be 10             	movsbl (%rax),%edx
    abe1:	83 ea 30             	sub    $0x30,%edx
    abe4:	83 fa 09             	cmp    $0x9,%edx
    abe7:	48 0f 46 c7          	cmovbe %rdi,%rax
    abeb:	e9 91 fd ff ff       	jmp    a981 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x741>
    abf0:	48 89 f0             	mov    %rsi,%rax
    abf3:	e9 45 ff ff ff       	jmp    ab3d <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x8fd>
    abf8:	0f be 06             	movsbl (%rsi),%eax
    abfb:	83 e8 30             	sub    $0x30,%eax
    abfe:	83 f8 09             	cmp    $0x9,%eax
    ac01:	0f 87 ef f7 ff ff    	ja     a3f6 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x1b6>
    ac07:	48 ff c6             	inc    %rsi
    ac0a:	0f be 06             	movsbl (%rsi),%eax
    ac0d:	83 e8 30             	sub    $0x30,%eax
    ac10:	83 f8 09             	cmp    $0x9,%eax
    ac13:	0f 87 dd f7 ff ff    	ja     a3f6 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x1b6>
    ac19:	48 ff c6             	inc    %rsi
    ac1c:	e9 c6 f7 ff ff       	jmp    a3e7 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x1a7>
    ac21:	e8 2a 85 ff ff       	call   3150 <__stack_chk_fail@plt>
    ac26:	48 89 c2             	mov    %rax,%rdx
    ac29:	48 8d 35 10 54 00 00 	lea    0x5410(%rip),%rsi        # 10040 <_fini+0xe1f>
    ac30:	48 8d 3d 29 54 00 00 	lea    0x5429(%rip),%rdi        # 10060 <_fini+0xe3f>
    ac37:	31 c0                	xor    %eax,%eax
    ac39:	e8 82 86 ff ff       	call   32c0 <_ZSt24__throw_out_of_range_fmtPKcz@plt>
    ac3e:	0f be 3a             	movsbl (%rdx),%edi
    ac41:	83 ef 30             	sub    $0x30,%edi
    ac44:	83 ff 09             	cmp    $0x9,%edi
    ac47:	77 16                	ja     ac5f <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0xa1f>
    ac49:	48 ff c2             	inc    %rdx
    ac4c:	0f be 3a             	movsbl (%rdx),%edi
    ac4f:	83 ef 30             	sub    $0x30,%edi
    ac52:	83 ff 09             	cmp    $0x9,%edi
    ac55:	77 08                	ja     ac5f <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0xa1f>
    ac57:	48 ff c2             	inc    %rdx
    ac5a:	e9 3a fe ff ff       	jmp    aa99 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x859>
    ac5f:	48 89 d0             	mov    %rdx,%rax
    ac62:	e9 a3 fb ff ff       	jmp    a80a <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x5ca>
    ac67:	48 89 f0             	mov    %rsi,%rax
    ac6a:	e9 fb fe ff ff       	jmp    ab6a <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0x92a>
    ac6f:	48 89 c5             	mov    %rax,%rbp
    ac72:	eb 35                	jmp    aca9 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0xa69>
    ac74:	48 89 c5             	mov    %rax,%rbp
    ac77:	eb 05                	jmp    ac7e <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0xa3e>
    ac79:	48 89 c5             	mov    %rax,%rbp
    ac7c:	eb 20                	jmp    ac9e <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0xa5e>
    ac7e:	48 8b 7c 24 50       	mov    0x50(%rsp),%rdi
    ac83:	48 8d 44 24 60       	lea    0x60(%rsp),%rax
    ac88:	48 39 c7             	cmp    %rax,%rdi
    ac8b:	74 11                	je     ac9e <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0xa5e>
    ac8d:	48 8b 44 24 60       	mov    0x60(%rsp),%rax
    ac92:	48 8d 70 01          	lea    0x1(%rax),%rsi
    ac96:	c5 f8 77             	vzeroupper 
    ac99:	e8 82 86 ff ff       	call   3320 <_ZdlPvm@plt>
    ac9e:	4c 89 e7             	mov    %r12,%rdi
    aca1:	c5 f8 77             	vzeroupper 
    aca4:	e8 07 87 ff ff       	call   33b0 <__cxa_free_exception@plt>
    aca9:	48 8b 7c 24 30       	mov    0x30(%rsp),%rdi
    acae:	48 39 df             	cmp    %rbx,%rdi
    acb1:	74 19                	je     accc <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0xa8c>
    acb3:	48 8b 44 24 40       	mov    0x40(%rsp),%rax
    acb8:	48 8d 70 01          	lea    0x1(%rax),%rsi
    acbc:	c5 f8 77             	vzeroupper 
    acbf:	e8 5c 86 ff ff       	call   3320 <_ZdlPvm@plt>
    acc4:	48 89 ef             	mov    %rbp,%rdi
    acc7:	e8 94 86 ff ff       	call   3360 <_Unwind_Resume@plt>
    accc:	c5 f8 77             	vzeroupper 
    accf:	eb f3                	jmp    acc4 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0xa84>
    acd1:	bf 10 00 00 00       	mov    $0x10,%edi
    acd6:	e8 f5 83 ff ff       	call   30d0 <__cxa_allocate_exception@plt>
    acdb:	4c 8d 6c 24 50       	lea    0x50(%rsp),%r13
    ace0:	48 8b 14 24          	mov    (%rsp),%rdx
    ace4:	48 8d 35 b5 56 00 00 	lea    0x56b5(%rip),%rsi        # 103a0 <_fini+0x117f>
    aceb:	4c 89 ef             	mov    %r13,%rdi
    acee:	49 89 c4             	mov    %rax,%r12
    acf1:	e8 9a dd ff ff       	call   8a90 <_ZStplIcSt11char_traitsIcESaIcEENSt7__cxx1112basic_stringIT_T0_T1_EEPKS5_RKS8_>
    acf6:	4c 89 ee             	mov    %r13,%rsi
    acf9:	4c 89 e7             	mov    %r12,%rdi
    acfc:	e8 8f 85 ff ff       	call   3290 <_ZNSt13runtime_errorC1ERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEE@plt>
    ad01:	48 8b 7c 24 50       	mov    0x50(%rsp),%rdi
    ad06:	48 8d 44 24 60       	lea    0x60(%rsp),%rax
    ad0b:	48 39 c7             	cmp    %rax,%rdi
    ad0e:	74 0e                	je     ad1e <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0xade>
    ad10:	48 8b 44 24 60       	mov    0x60(%rsp),%rax
    ad15:	48 8d 70 01          	lea    0x1(%rax),%rsi
    ad19:	e8 02 86 ff ff       	call   3320 <_ZdlPvm@plt>
    ad1e:	48 8b 15 a3 92 00 00 	mov    0x92a3(%rip),%rdx        # 13fc8 <_ZNSt13runtime_errorD1Ev@Base>
    ad25:	48 8d 35 74 8c 00 00 	lea    0x8c74(%rip),%rsi        # 139a0 <_ZTISt13runtime_error@@Base>
    ad2c:	4c 89 e7             	mov    %r12,%rdi
    ad2f:	e8 ec 84 ff ff       	call   3220 <__cxa_throw@plt>
    ad34:	bf 10 00 00 00       	mov    $0x10,%edi
    ad39:	e8 92 83 ff ff       	call   30d0 <__cxa_allocate_exception@plt>
    ad3e:	48 89 c7             	mov    %rax,%rdi
    ad41:	48 8d 35 28 56 00 00 	lea    0x5628(%rip),%rsi        # 10370 <_fini+0x114f>
    ad48:	48 89 c5             	mov    %rax,%rbp
    ad4b:	e8 a0 84 ff ff       	call   31f0 <_ZNSt13runtime_errorC1EPKc@plt>
    ad50:	48 8b 15 71 92 00 00 	mov    0x9271(%rip),%rdx        # 13fc8 <_ZNSt13runtime_errorD1Ev@Base>
    ad57:	48 8d 35 42 8c 00 00 	lea    0x8c42(%rip),%rsi        # 139a0 <_ZTISt13runtime_error@@Base>
    ad5e:	48 89 ef             	mov    %rbp,%rdi
    ad61:	e8 ba 84 ff ff       	call   3220 <__cxa_throw@plt>
    ad66:	49 89 c4             	mov    %rax,%r12
    ad69:	eb 0a                	jmp    ad75 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0xb35>
    ad6b:	48 89 c5             	mov    %rax,%rbp
    ad6e:	eb 18                	jmp    ad88 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0xb48>
    ad70:	48 89 c5             	mov    %rax,%rbp
    ad73:	eb 33                	jmp    ada8 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0xb68>
    ad75:	48 89 ef             	mov    %rbp,%rdi
    ad78:	c5 f8 77             	vzeroupper 
    ad7b:	e8 30 86 ff ff       	call   33b0 <__cxa_free_exception@plt>
    ad80:	4c 89 e7             	mov    %r12,%rdi
    ad83:	e8 d8 85 ff ff       	call   3360 <_Unwind_Resume@plt>
    ad88:	48 8b 7c 24 50       	mov    0x50(%rsp),%rdi
    ad8d:	48 8d 44 24 60       	lea    0x60(%rsp),%rax
    ad92:	48 39 c7             	cmp    %rax,%rdi
    ad95:	74 11                	je     ada8 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0xb68>
    ad97:	48 8b 44 24 60       	mov    0x60(%rsp),%rax
    ad9c:	48 8d 70 01          	lea    0x1(%rax),%rsi
    ada0:	c5 f8 77             	vzeroupper 
    ada3:	e8 78 85 ff ff       	call   3320 <_ZdlPvm@plt>
    ada8:	4c 89 e7             	mov    %r12,%rdi
    adab:	c5 f8 77             	vzeroupper 
    adae:	e8 fd 85 ff ff       	call   33b0 <__cxa_free_exception@plt>
    adb3:	e9 0c ff ff ff       	jmp    acc4 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE+0xa84>
    adb8:	0f 1f 84 00 00 00 00 	nopl   0x0(%rax,%rax,1)
    adbf:	00 

000000000000adc0 <_ZNSt6vectorISt3anySaIS0_EE17_M_default_appendEm>:
    adc0:	41 57                	push   %r15
    adc2:	41 56                	push   %r14
    adc4:	41 55                	push   %r13
    adc6:	41 54                	push   %r12
    adc8:	55                   	push   %rbp
    adc9:	53                   	push   %rbx
    adca:	48 83 ec 38          	sub    $0x38,%rsp
    adce:	64 48 8b 04 25 28 00 	mov    %fs:0x28,%rax
    add5:	00 00 
    add7:	48 89 44 24 28       	mov    %rax,0x28(%rsp)
    addc:	31 c0                	xor    %eax,%eax
    adde:	48 85 f6             	test   %rsi,%rsi
    ade1:	74 68                	je     ae4b <_ZNSt6vectorISt3anySaIS0_EE17_M_default_appendEm+0x8b>
    ade3:	4c 8b 67 08          	mov    0x8(%rdi),%r12
    ade7:	4c 8b 37             	mov    (%rdi),%r14
    adea:	48 8b 57 10          	mov    0x10(%rdi),%rdx
    adee:	4d 89 e7             	mov    %r12,%r15
    adf1:	4d 29 f7             	sub    %r14,%r15
    adf4:	4d 89 fd             	mov    %r15,%r13
    adf7:	48 b9 ff ff ff ff ff 	movabs $0x7ffffffffffffff,%rcx
    adfe:	ff ff 07 
    ae01:	4c 29 e2             	sub    %r12,%rdx
    ae04:	48 89 f3             	mov    %rsi,%rbx
    ae07:	49 c1 fd 04          	sar    $0x4,%r13
    ae0b:	48 89 ce             	mov    %rcx,%rsi
    ae0e:	48 c1 fa 04          	sar    $0x4,%rdx
    ae12:	48 89 fd             	mov    %rdi,%rbp
    ae15:	4c 29 ee             	sub    %r13,%rsi
    ae18:	48 39 d3             	cmp    %rdx,%rbx
    ae1b:	77 53                	ja     ae70 <_ZNSt6vectorISt3anySaIS0_EE17_M_default_appendEm+0xb0>
    ae1d:	4c 89 e0             	mov    %r12,%rax
    ae20:	48 89 da             	mov    %rbx,%rdx
    ae23:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)
    ae28:	48 c7 00 00 00 00 00 	movq   $0x0,(%rax)
    ae2f:	48 c7 40 08 00 00 00 	movq   $0x0,0x8(%rax)
    ae36:	00 
    ae37:	48 83 c0 10          	add    $0x10,%rax
    ae3b:	48 ff ca             	dec    %rdx
    ae3e:	75 e8                	jne    ae28 <_ZNSt6vectorISt3anySaIS0_EE17_M_default_appendEm+0x68>
    ae40:	48 c1 e3 04          	shl    $0x4,%rbx
    ae44:	49 01 dc             	add    %rbx,%r12
    ae47:	4c 89 65 08          	mov    %r12,0x8(%rbp)
    ae4b:	48 8b 44 24 28       	mov    0x28(%rsp),%rax
    ae50:	64 48 2b 04 25 28 00 	sub    %fs:0x28,%rax
    ae57:	00 00 
    ae59:	0f 85 80 01 00 00    	jne    afdf <_ZNSt6vectorISt3anySaIS0_EE17_M_default_appendEm+0x21f>
    ae5f:	48 83 c4 38          	add    $0x38,%rsp
    ae63:	5b                   	pop    %rbx
    ae64:	5d                   	pop    %rbp
    ae65:	41 5c                	pop    %r12
    ae67:	41 5d                	pop    %r13
    ae69:	41 5e                	pop    %r14
    ae6b:	41 5f                	pop    %r15
    ae6d:	c3                   	ret    
    ae6e:	66 90                	xchg   %ax,%ax
    ae70:	48 39 de             	cmp    %rbx,%rsi
    ae73:	0f 82 77 01 00 00    	jb     aff0 <_ZNSt6vectorISt3anySaIS0_EE17_M_default_appendEm+0x230>
    ae79:	4c 39 eb             	cmp    %r13,%rbx
    ae7c:	4c 89 ea             	mov    %r13,%rdx
    ae7f:	48 0f 43 d3          	cmovae %rbx,%rdx
    ae83:	4c 01 ea             	add    %r13,%rdx
    ae86:	0f 82 58 01 00 00    	jb     afe4 <_ZNSt6vectorISt3anySaIS0_EE17_M_default_appendEm+0x224>
    ae8c:	48 c7 44 24 18 00 00 	movq   $0x0,0x18(%rsp)
    ae93:	00 00 
    ae95:	48 c7 44 24 10 00 00 	movq   $0x0,0x10(%rsp)
    ae9c:	00 00 
    ae9e:	48 85 d2             	test   %rdx,%rdx
    aea1:	0f 85 01 01 00 00    	jne    afa8 <_ZNSt6vectorISt3anySaIS0_EE17_M_default_appendEm+0x1e8>
    aea7:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
    aeac:	48 89 d9             	mov    %rbx,%rcx
    aeaf:	4a 8d 14 38          	lea    (%rax,%r15,1),%rdx
    aeb3:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)
    aeb8:	48 c7 02 00 00 00 00 	movq   $0x0,(%rdx)
    aebf:	48 c7 42 08 00 00 00 	movq   $0x0,0x8(%rdx)
    aec6:	00 
    aec7:	48 83 c2 10          	add    $0x10,%rdx
    aecb:	48 ff c9             	dec    %rcx
    aece:	75 e8                	jne    aeb8 <_ZNSt6vectorISt3anySaIS0_EE17_M_default_appendEm+0xf8>
    aed0:	4d 39 e6             	cmp    %r12,%r14
    aed3:	74 62                	je     af37 <_ZNSt6vectorISt3anySaIS0_EE17_M_default_appendEm+0x177>
    aed5:	48 8d 44 24 20       	lea    0x20(%rsp),%rax
    aeda:	48 89 44 24 08       	mov    %rax,0x8(%rsp)
    aedf:	4c 8b 7c 24 10       	mov    0x10(%rsp),%r15
    aee4:	0f 1f 40 00          	nopl   0x0(%rax)
    aee8:	49 c7 47 08 00 00 00 	movq   $0x0,0x8(%r15)
    aeef:	00 
    aef0:	4d 8b 06             	mov    (%r14),%r8
    aef3:	4d 85 c0             	test   %r8,%r8
    aef6:	0f 84 9d 00 00 00    	je     af99 <_ZNSt6vectorISt3anySaIS0_EE17_M_default_appendEm+0x1d9>
    aefc:	4c 89 7c 24 20       	mov    %r15,0x20(%rsp)
    af01:	48 8b 54 24 08       	mov    0x8(%rsp),%rdx
    af06:	4c 89 f6             	mov    %r14,%rsi
    af09:	bf 04 00 00 00       	mov    $0x4,%edi
    af0e:	41 ff d0             	call   *%r8
    af11:	4d 8b 06             	mov    (%r14),%r8
    af14:	4d 85 c0             	test   %r8,%r8
    af17:	74 5f                	je     af78 <_ZNSt6vectorISt3anySaIS0_EE17_M_default_appendEm+0x1b8>
    af19:	4c 89 f6             	mov    %r14,%rsi
    af1c:	31 d2                	xor    %edx,%edx
    af1e:	bf 03 00 00 00       	mov    $0x3,%edi
    af23:	49 83 c6 10          	add    $0x10,%r14
    af27:	41 ff d0             	call   *%r8
    af2a:	49 83 c7 10          	add    $0x10,%r15
    af2e:	4d 39 e6             	cmp    %r12,%r14
    af31:	75 b5                	jne    aee8 <_ZNSt6vectorISt3anySaIS0_EE17_M_default_appendEm+0x128>
    af33:	4c 8b 65 00          	mov    0x0(%rbp),%r12
    af37:	4d 85 e4             	test   %r12,%r12
    af3a:	74 0f                	je     af4b <_ZNSt6vectorISt3anySaIS0_EE17_M_default_appendEm+0x18b>
    af3c:	48 8b 75 10          	mov    0x10(%rbp),%rsi
    af40:	4c 89 e7             	mov    %r12,%rdi
    af43:	4c 29 e6             	sub    %r12,%rsi
    af46:	e8 d5 83 ff ff       	call   3320 <_ZdlPvm@plt>
    af4b:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
    af50:	49 01 dd             	add    %rbx,%r13
    af53:	49 c1 e5 04          	shl    $0x4,%r13
    af57:	4e 8d 34 28          	lea    (%rax,%r13,1),%r14
    af5b:	48 89 45 00          	mov    %rax,0x0(%rbp)
    af5f:	48 8b 44 24 18       	mov    0x18(%rsp),%rax
    af64:	4c 89 75 08          	mov    %r14,0x8(%rbp)
    af68:	48 89 45 10          	mov    %rax,0x10(%rbp)
    af6c:	e9 da fe ff ff       	jmp    ae4b <_ZNSt6vectorISt3anySaIS0_EE17_M_default_appendEm+0x8b>
    af71:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
    af78:	49 83 c6 10          	add    $0x10,%r14
    af7c:	49 83 c7 10          	add    $0x10,%r15
    af80:	4d 39 e6             	cmp    %r12,%r14
    af83:	74 ae                	je     af33 <_ZNSt6vectorISt3anySaIS0_EE17_M_default_appendEm+0x173>
    af85:	49 c7 47 08 00 00 00 	movq   $0x0,0x8(%r15)
    af8c:	00 
    af8d:	4d 8b 06             	mov    (%r14),%r8
    af90:	4d 85 c0             	test   %r8,%r8
    af93:	0f 85 63 ff ff ff    	jne    aefc <_ZNSt6vectorISt3anySaIS0_EE17_M_default_appendEm+0x13c>
    af99:	49 c7 07 00 00 00 00 	movq   $0x0,(%r15)
    afa0:	e9 6c ff ff ff       	jmp    af11 <_ZNSt6vectorISt3anySaIS0_EE17_M_default_appendEm+0x151>
    afa5:	0f 1f 00             	nopl   (%rax)
    afa8:	48 39 ca             	cmp    %rcx,%rdx
    afab:	48 0f 47 d1          	cmova  %rcx,%rdx
    afaf:	48 c1 e2 04          	shl    $0x4,%rdx
    afb3:	48 89 d7             	mov    %rdx,%rdi
    afb6:	48 89 54 24 08       	mov    %rdx,0x8(%rsp)
    afbb:	e8 e0 82 ff ff       	call   32a0 <_Znwm@plt>
    afc0:	48 8b 54 24 08       	mov    0x8(%rsp),%rdx
    afc5:	48 89 44 24 10       	mov    %rax,0x10(%rsp)
    afca:	48 01 c2             	add    %rax,%rdx
    afcd:	48 89 54 24 18       	mov    %rdx,0x18(%rsp)
    afd2:	4c 8b 65 08          	mov    0x8(%rbp),%r12
    afd6:	4c 8b 75 00          	mov    0x0(%rbp),%r14
    afda:	e9 c8 fe ff ff       	jmp    aea7 <_ZNSt6vectorISt3anySaIS0_EE17_M_default_appendEm+0xe7>
    afdf:	e8 6c 81 ff ff       	call   3150 <__stack_chk_fail@plt>
    afe4:	48 ba f0 ff ff ff ff 	movabs $0x7ffffffffffffff0,%rdx
    afeb:	ff ff 7f 
    afee:	eb c3                	jmp    afb3 <_ZNSt6vectorISt3anySaIS0_EE17_M_default_appendEm+0x1f3>
    aff0:	48 8d 3d bc 53 00 00 	lea    0x53bc(%rip),%rdi        # 103b3 <_fini+0x1192>
    aff7:	e8 44 83 ff ff       	call   3340 <_ZSt20__throw_length_errorPKc@plt>
    affc:	0f 1f 40 00          	nopl   0x0(%rax)

000000000000b000 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm1EEEE14__visit_invokeESQ_S10_>:
    b000:	41 55                	push   %r13
    b002:	49 89 fd             	mov    %rdi,%r13
    b005:	41 54                	push   %r12
    b007:	55                   	push   %rbp
    b008:	53                   	push   %rbx
    b009:	48 83 ec 58          	sub    $0x58,%rsp
    b00d:	64 48 8b 04 25 28 00 	mov    %fs:0x28,%rax
    b014:	00 00 
    b016:	48 89 44 24 48       	mov    %rax,0x48(%rsp)
    b01b:	31 c0                	xor    %eax,%eax
    b01d:	48 8b 46 10          	mov    0x10(%rsi),%rax
    b021:	48 c7 44 24 10 00 00 	movq   $0x0,0x10(%rsp)
    b028:	00 00 
    b02a:	48 85 c0             	test   %rax,%rax
    b02d:	0f 84 dd 00 00 00    	je     b110 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm1EEEE14__visit_invokeESQ_S10_+0x110>
    b033:	48 89 e5             	mov    %rsp,%rbp
    b036:	ba 02 00 00 00       	mov    $0x2,%edx
    b03b:	48 89 ef             	mov    %rbp,%rdi
    b03e:	48 89 f3             	mov    %rsi,%rbx
    b041:	ff d0                	call   *%rax
    b043:	c5 fa 6f 4b 10       	vmovdqu 0x10(%rbx),%xmm1
    b048:	48 8b 43 10          	mov    0x10(%rbx),%rax
    b04c:	4d 8b 65 08          	mov    0x8(%r13),%r12
    b050:	49 8b 5d 00          	mov    0x0(%r13),%rbx
    b054:	c5 f9 7f 4c 24 10    	vmovdqa %xmm1,0x10(%rsp)
    b05a:	49 39 dc             	cmp    %rbx,%r12
    b05d:	74 22                	je     b081 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm1EEEE14__visit_invokeESQ_S10_+0x81>
    b05f:	90                   	nop
    b060:	48 85 c0             	test   %rax,%rax
    b063:	0f 84 b0 00 00 00    	je     b119 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm1EEEE14__visit_invokeESQ_S10_+0x119>
    b069:	48 89 de             	mov    %rbx,%rsi
    b06c:	48 89 ef             	mov    %rbp,%rdi
    b06f:	ff 54 24 18          	call   *0x18(%rsp)
    b073:	48 83 c3 20          	add    $0x20,%rbx
    b077:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
    b07c:	49 39 dc             	cmp    %rbx,%r12
    b07f:	75 df                	jne    b060 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm1EEEE14__visit_invokeESQ_S10_+0x60>
    b081:	c5 f9 6f 04 24       	vmovdqa (%rsp),%xmm0
    b086:	c5 f9 6f 54 24 20    	vmovdqa 0x20(%rsp),%xmm2
    b08c:	48 8b 54 24 18       	mov    0x18(%rsp),%rdx
    b091:	48 8b 4c 24 38       	mov    0x38(%rsp),%rcx
    b096:	48 c7 44 24 10 00 00 	movq   $0x0,0x10(%rsp)
    b09d:	00 00 
    b09f:	48 89 44 24 30       	mov    %rax,0x30(%rsp)
    b0a4:	48 89 4c 24 18       	mov    %rcx,0x18(%rsp)
    b0a9:	48 89 54 24 38       	mov    %rdx,0x38(%rsp)
    b0ae:	c5 f9 7f 14 24       	vmovdqa %xmm2,(%rsp)
    b0b3:	c5 f9 7f 44 24 20    	vmovdqa %xmm0,0x20(%rsp)
    b0b9:	48 85 c0             	test   %rax,%rax
    b0bc:	74 26                	je     b0e4 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm1EEEE14__visit_invokeESQ_S10_+0xe4>
    b0be:	48 8d 7c 24 20       	lea    0x20(%rsp),%rdi
    b0c3:	ba 03 00 00 00       	mov    $0x3,%edx
    b0c8:	48 89 fe             	mov    %rdi,%rsi
    b0cb:	ff d0                	call   *%rax
    b0cd:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
    b0d2:	48 85 c0             	test   %rax,%rax
    b0d5:	74 0d                	je     b0e4 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm1EEEE14__visit_invokeESQ_S10_+0xe4>
    b0d7:	ba 03 00 00 00       	mov    $0x3,%edx
    b0dc:	48 89 ee             	mov    %rbp,%rsi
    b0df:	48 89 ef             	mov    %rbp,%rdi
    b0e2:	ff d0                	call   *%rax
    b0e4:	49 8b 5d 10          	mov    0x10(%r13),%rbx
    b0e8:	48 83 7b 48 00       	cmpq   $0x0,0x48(%rbx)
    b0ed:	74 31                	je     b120 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm1EEEE14__visit_invokeESQ_S10_+0x120>
    b0ef:	48 8b 44 24 48       	mov    0x48(%rsp),%rax
    b0f4:	64 48 2b 04 25 28 00 	sub    %fs:0x28,%rax
    b0fb:	00 00 
    b0fd:	0f 85 c6 00 00 00    	jne    b1c9 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm1EEEE14__visit_invokeESQ_S10_+0x1c9>
    b103:	48 83 c4 58          	add    $0x58,%rsp
    b107:	5b                   	pop    %rbx
    b108:	5d                   	pop    %rbp
    b109:	41 5c                	pop    %r12
    b10b:	41 5d                	pop    %r13
    b10d:	c3                   	ret    
    b10e:	66 90                	xchg   %ax,%ax
    b110:	48 8b 07             	mov    (%rdi),%rax
    b113:	48 39 47 08          	cmp    %rax,0x8(%rdi)
    b117:	74 cb                	je     b0e4 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm1EEEE14__visit_invokeESQ_S10_+0xe4>
    b119:	e8 a2 80 ff ff       	call   31c0 <_ZSt25__throw_bad_function_callv@plt>
    b11e:	66 90                	xchg   %ax,%ax
    b120:	80 bb d8 00 00 00 00 	cmpb   $0x0,0xd8(%rbx)
    b127:	75 c6                	jne    b0ef <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm1EEEE14__visit_invokeESQ_S10_+0xef>
    b129:	4c 8b a3 b8 00 00 00 	mov    0xb8(%rbx),%r12
    b130:	49 8b 45 08          	mov    0x8(%r13),%rax
    b134:	48 8b 8b b0 00 00 00 	mov    0xb0(%rbx),%rcx
    b13b:	4c 89 e2             	mov    %r12,%rdx
    b13e:	49 2b 45 00          	sub    0x0(%r13),%rax
    b142:	48 29 ca             	sub    %rcx,%rdx
    b145:	48 c1 f8 05          	sar    $0x5,%rax
    b149:	48 c1 fa 04          	sar    $0x4,%rdx
    b14d:	48 39 d0             	cmp    %rdx,%rax
    b150:	77 4b                	ja     b19d <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm1EEEE14__visit_invokeESQ_S10_+0x19d>
    b152:	73 9b                	jae    b0ef <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm1EEEE14__visit_invokeESQ_S10_+0xef>
    b154:	48 c1 e0 04          	shl    $0x4,%rax
    b158:	4c 8d 2c 01          	lea    (%rcx,%rax,1),%r13
    b15c:	4d 39 ec             	cmp    %r13,%r12
    b15f:	74 8e                	je     b0ef <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm1EEEE14__visit_invokeESQ_S10_+0xef>
    b161:	4c 89 ed             	mov    %r13,%rbp
    b164:	0f 1f 40 00          	nopl   0x0(%rax)
    b168:	48 8b 45 00          	mov    0x0(%rbp),%rax
    b16c:	48 85 c0             	test   %rax,%rax
    b16f:	74 21                	je     b192 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm1EEEE14__visit_invokeESQ_S10_+0x192>
    b171:	48 89 ee             	mov    %rbp,%rsi
    b174:	31 d2                	xor    %edx,%edx
    b176:	bf 03 00 00 00       	mov    $0x3,%edi
    b17b:	48 83 c5 10          	add    $0x10,%rbp
    b17f:	ff d0                	call   *%rax
    b181:	49 39 ec             	cmp    %rbp,%r12
    b184:	75 e2                	jne    b168 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm1EEEE14__visit_invokeESQ_S10_+0x168>
    b186:	4c 89 ab b8 00 00 00 	mov    %r13,0xb8(%rbx)
    b18d:	e9 5d ff ff ff       	jmp    b0ef <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm1EEEE14__visit_invokeESQ_S10_+0xef>
    b192:	48 83 c5 10          	add    $0x10,%rbp
    b196:	49 39 ec             	cmp    %rbp,%r12
    b199:	75 cd                	jne    b168 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm1EEEE14__visit_invokeESQ_S10_+0x168>
    b19b:	eb e9                	jmp    b186 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm1EEEE14__visit_invokeESQ_S10_+0x186>
    b19d:	48 8b 4c 24 48       	mov    0x48(%rsp),%rcx
    b1a2:	64 48 2b 0c 25 28 00 	sub    %fs:0x28,%rcx
    b1a9:	00 00 
    b1ab:	75 1c                	jne    b1c9 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm1EEEE14__visit_invokeESQ_S10_+0x1c9>
    b1ad:	48 83 c4 58          	add    $0x58,%rsp
    b1b1:	48 8d bb b0 00 00 00 	lea    0xb0(%rbx),%rdi
    b1b8:	5b                   	pop    %rbx
    b1b9:	5d                   	pop    %rbp
    b1ba:	48 29 d0             	sub    %rdx,%rax
    b1bd:	41 5c                	pop    %r12
    b1bf:	48 89 c6             	mov    %rax,%rsi
    b1c2:	41 5d                	pop    %r13
    b1c4:	e9 f7 fb ff ff       	jmp    adc0 <_ZNSt6vectorISt3anySaIS0_EE17_M_default_appendEm>
    b1c9:	e8 82 7f ff ff       	call   3150 <__stack_chk_fail@plt>
    b1ce:	48 89 c5             	mov    %rax,%rbp
    b1d1:	eb 05                	jmp    b1d8 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm1EEEE14__visit_invokeESQ_S10_+0x1d8>
    b1d3:	49 89 c4             	mov    %rax,%r12
    b1d6:	eb 22                	jmp    b1fa <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm1EEEE14__visit_invokeESQ_S10_+0x1fa>
    b1d8:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
    b1dd:	48 85 c0             	test   %rax,%rax
    b1e0:	74 3a                	je     b21c <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm1EEEE14__visit_invokeESQ_S10_+0x21c>
    b1e2:	48 89 e7             	mov    %rsp,%rdi
    b1e5:	ba 03 00 00 00       	mov    $0x3,%edx
    b1ea:	48 89 fe             	mov    %rdi,%rsi
    b1ed:	c5 f8 77             	vzeroupper 
    b1f0:	ff d0                	call   *%rax
    b1f2:	48 89 ef             	mov    %rbp,%rdi
    b1f5:	e8 66 81 ff ff       	call   3360 <_Unwind_Resume@plt>
    b1fa:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
    b1ff:	48 85 c0             	test   %rax,%rax
    b202:	74 1d                	je     b221 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm1EEEE14__visit_invokeESQ_S10_+0x221>
    b204:	ba 03 00 00 00       	mov    $0x3,%edx
    b209:	48 89 ee             	mov    %rbp,%rsi
    b20c:	48 89 ef             	mov    %rbp,%rdi
    b20f:	c5 f8 77             	vzeroupper 
    b212:	ff d0                	call   *%rax
    b214:	4c 89 e7             	mov    %r12,%rdi
    b217:	e8 44 81 ff ff       	call   3360 <_Unwind_Resume@plt>
    b21c:	c5 f8 77             	vzeroupper 
    b21f:	eb d1                	jmp    b1f2 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm1EEEE14__visit_invokeESQ_S10_+0x1f2>
    b221:	c5 f8 77             	vzeroupper 
    b224:	eb ee                	jmp    b214 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm1EEEE14__visit_invokeESQ_S10_+0x214>
    b226:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
    b22d:	00 00 00 

000000000000b230 <_ZNSt6vectorISt3anySaIS0_EED1Ev>:
    b230:	41 54                	push   %r12
    b232:	49 89 fc             	mov    %rdi,%r12
    b235:	55                   	push   %rbp
    b236:	53                   	push   %rbx
    b237:	48 8b 5f 08          	mov    0x8(%rdi),%rbx
    b23b:	48 8b 2f             	mov    (%rdi),%rbp
    b23e:	48 39 eb             	cmp    %rbp,%rbx
    b241:	74 27                	je     b26a <_ZNSt6vectorISt3anySaIS0_EED1Ev+0x3a>
    b243:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)
    b248:	48 8b 45 00          	mov    0x0(%rbp),%rax
    b24c:	48 85 c0             	test   %rax,%rax
    b24f:	74 37                	je     b288 <_ZNSt6vectorISt3anySaIS0_EED1Ev+0x58>
    b251:	48 89 ee             	mov    %rbp,%rsi
    b254:	31 d2                	xor    %edx,%edx
    b256:	bf 03 00 00 00       	mov    $0x3,%edi
    b25b:	48 83 c5 10          	add    $0x10,%rbp
    b25f:	ff d0                	call   *%rax
    b261:	48 39 eb             	cmp    %rbp,%rbx
    b264:	75 e2                	jne    b248 <_ZNSt6vectorISt3anySaIS0_EED1Ev+0x18>
    b266:	49 8b 2c 24          	mov    (%r12),%rbp
    b26a:	48 85 ed             	test   %rbp,%rbp
    b26d:	74 29                	je     b298 <_ZNSt6vectorISt3anySaIS0_EED1Ev+0x68>
    b26f:	49 8b 74 24 10       	mov    0x10(%r12),%rsi
    b274:	5b                   	pop    %rbx
    b275:	48 29 ee             	sub    %rbp,%rsi
    b278:	48 89 ef             	mov    %rbp,%rdi
    b27b:	5d                   	pop    %rbp
    b27c:	41 5c                	pop    %r12
    b27e:	e9 9d 80 ff ff       	jmp    3320 <_ZdlPvm@plt>
    b283:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)
    b288:	48 83 c5 10          	add    $0x10,%rbp
    b28c:	48 39 eb             	cmp    %rbp,%rbx
    b28f:	75 b7                	jne    b248 <_ZNSt6vectorISt3anySaIS0_EED1Ev+0x18>
    b291:	eb d3                	jmp    b266 <_ZNSt6vectorISt3anySaIS0_EED1Ev+0x36>
    b293:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)
    b298:	5b                   	pop    %rbx
    b299:	5d                   	pop    %rbp
    b29a:	41 5c                	pop    %r12
    b29c:	c3                   	ret    
    b29d:	0f 1f 00             	nopl   (%rax)

000000000000b2a0 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE24_M_get_insert_unique_posERS5_>:
    b2a0:	41 57                	push   %r15
    b2a2:	41 56                	push   %r14
    b2a4:	41 55                	push   %r13
    b2a6:	41 54                	push   %r12
    b2a8:	55                   	push   %rbp
    b2a9:	53                   	push   %rbx
    b2aa:	48 83 ec 28          	sub    $0x28,%rsp
    b2ae:	4c 8b 7f 10          	mov    0x10(%rdi),%r15
    b2b2:	48 89 7c 24 18       	mov    %rdi,0x18(%rsp)
    b2b7:	48 89 74 24 10       	mov    %rsi,0x10(%rsp)
    b2bc:	4d 85 ff             	test   %r15,%r15
    b2bf:	0f 84 fb 00 00 00    	je     b3c0 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE24_M_get_insert_unique_posERS5_+0x120>
    b2c5:	c5 fa 7e 16          	vmovq  (%rsi),%xmm2
    b2c9:	48 8b 6e 08          	mov    0x8(%rsi),%rbp
    b2cd:	bb 00 00 00 80       	mov    $0x80000000,%ebx
    b2d2:	49 bc ff ff ff 7f ff 	movabs $0xffffffff7fffffff,%r12
    b2d9:	ff ff ff 
    b2dc:	eb 13                	jmp    b2f1 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE24_M_get_insert_unique_posERS5_+0x51>
    b2de:	66 90                	xchg   %ax,%ax
    b2e0:	49 8b 47 10          	mov    0x10(%r15),%rax
    b2e4:	ba 01 00 00 00       	mov    $0x1,%edx
    b2e9:	48 85 c0             	test   %rax,%rax
    b2ec:	74 69                	je     b357 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE24_M_get_insert_unique_posERS5_+0xb7>
    b2ee:	49 89 c7             	mov    %rax,%r15
    b2f1:	c4 c1 7a 7e 4f 20    	vmovq  0x20(%r15),%xmm1
    b2f7:	4d 8b 6f 28          	mov    0x28(%r15),%r13
    b2fb:	62 f2 f5 08 3b c2    	vpminuq %xmm2,%xmm1,%xmm0
    b301:	c4 c1 f9 7e c6       	vmovq  %xmm0,%r14
    b306:	4d 85 f6             	test   %r14,%r14
    b309:	74 2a                	je     b335 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE24_M_get_insert_unique_posERS5_+0x95>
    b30b:	c4 e1 f9 7e c2       	vmovq  %xmm0,%rdx
    b310:	4c 89 ee             	mov    %r13,%rsi
    b313:	48 89 ef             	mov    %rbp,%rdi
    b316:	c5 f9 d6 54 24 08    	vmovq  %xmm2,0x8(%rsp)
    b31c:	c5 f9 d6 0c 24       	vmovq  %xmm1,(%rsp)
    b321:	e8 4a 7f ff ff       	call   3270 <memcmp@plt>
    b326:	85 c0                	test   %eax,%eax
    b328:	c5 fa 7e 0c 24       	vmovq  (%rsp),%xmm1
    b32d:	c5 fa 7e 54 24 08    	vmovq  0x8(%rsp),%xmm2
    b333:	75 13                	jne    b348 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE24_M_get_insert_unique_posERS5_+0xa8>
    b335:	c5 e9 fb c1          	vpsubq %xmm1,%xmm2,%xmm0
    b339:	c4 e1 f9 7e c0       	vmovq  %xmm0,%rax
    b33e:	48 39 d8             	cmp    %rbx,%rax
    b341:	7d 09                	jge    b34c <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE24_M_get_insert_unique_posERS5_+0xac>
    b343:	4c 39 e0             	cmp    %r12,%rax
    b346:	7e 98                	jle    b2e0 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE24_M_get_insert_unique_posERS5_+0x40>
    b348:	85 c0                	test   %eax,%eax
    b34a:	78 94                	js     b2e0 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE24_M_get_insert_unique_posERS5_+0x40>
    b34c:	49 8b 47 18          	mov    0x18(%r15),%rax
    b350:	31 d2                	xor    %edx,%edx
    b352:	48 85 c0             	test   %rax,%rax
    b355:	75 97                	jne    b2ee <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE24_M_get_insert_unique_posERS5_+0x4e>
    b357:	4c 89 fb             	mov    %r15,%rbx
    b35a:	84 d2                	test   %dl,%dl
    b35c:	75 66                	jne    b3c4 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE24_M_get_insert_unique_posERS5_+0x124>
    b35e:	4d 85 f6             	test   %r14,%r14
    b361:	74 28                	je     b38b <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE24_M_get_insert_unique_posERS5_+0xeb>
    b363:	4c 89 f2             	mov    %r14,%rdx
    b366:	48 89 ee             	mov    %rbp,%rsi
    b369:	4c 89 ef             	mov    %r13,%rdi
    b36c:	c5 f9 d6 54 24 08    	vmovq  %xmm2,0x8(%rsp)
    b372:	c5 f9 d6 0c 24       	vmovq  %xmm1,(%rsp)
    b377:	e8 f4 7e ff ff       	call   3270 <memcmp@plt>
    b37c:	85 c0                	test   %eax,%eax
    b37e:	c5 fa 7e 0c 24       	vmovq  (%rsp),%xmm1
    b383:	c5 fa 7e 54 24 08    	vmovq  0x8(%rsp),%xmm2
    b389:	75 19                	jne    b3a4 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE24_M_get_insert_unique_posERS5_+0x104>
    b38b:	c5 f1 fb ca          	vpsubq %xmm2,%xmm1,%xmm1
    b38f:	c4 e1 f9 7e c8       	vmovq  %xmm1,%rax
    b394:	48 3d ff ff ff 7f    	cmp    $0x7fffffff,%rax
    b39a:	7f 0c                	jg     b3a8 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE24_M_get_insert_unique_posERS5_+0x108>
    b39c:	48 3d 00 00 00 80    	cmp    $0xffffffff80000000,%rax
    b3a2:	7c 64                	jl     b408 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE24_M_get_insert_unique_posERS5_+0x168>
    b3a4:	85 c0                	test   %eax,%eax
    b3a6:	78 60                	js     b408 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE24_M_get_insert_unique_posERS5_+0x168>
    b3a8:	48 83 c4 28          	add    $0x28,%rsp
    b3ac:	5b                   	pop    %rbx
    b3ad:	5d                   	pop    %rbp
    b3ae:	41 5c                	pop    %r12
    b3b0:	41 5d                	pop    %r13
    b3b2:	41 5e                	pop    %r14
    b3b4:	4c 89 f8             	mov    %r15,%rax
    b3b7:	31 d2                	xor    %edx,%edx
    b3b9:	41 5f                	pop    %r15
    b3bb:	c3                   	ret    
    b3bc:	0f 1f 40 00          	nopl   0x0(%rax)
    b3c0:	4c 8d 7f 08          	lea    0x8(%rdi),%r15
    b3c4:	48 8b 44 24 18       	mov    0x18(%rsp),%rax
    b3c9:	4c 39 78 18          	cmp    %r15,0x18(%rax)
    b3cd:	74 51                	je     b420 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE24_M_get_insert_unique_posERS5_+0x180>
    b3cf:	4c 89 ff             	mov    %r15,%rdi
    b3d2:	e8 b9 7c ff ff       	call   3090 <_ZSt18_Rb_tree_decrementPSt18_Rb_tree_node_base@plt>
    b3d7:	48 8b 4c 24 10       	mov    0x10(%rsp),%rcx
    b3dc:	c5 fa 7e 48 20       	vmovq  0x20(%rax),%xmm1
    b3e1:	c5 fa 7e 11          	vmovq  (%rcx),%xmm2
    b3e5:	4c 89 fb             	mov    %r15,%rbx
    b3e8:	62 f2 f5 08 3b c2    	vpminuq %xmm2,%xmm1,%xmm0
    b3ee:	48 8b 69 08          	mov    0x8(%rcx),%rbp
    b3f2:	4c 8b 68 28          	mov    0x28(%rax),%r13
    b3f6:	c4 c1 f9 7e c6       	vmovq  %xmm0,%r14
    b3fb:	49 89 c7             	mov    %rax,%r15
    b3fe:	e9 5b ff ff ff       	jmp    b35e <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE24_M_get_insert_unique_posERS5_+0xbe>
    b403:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)
    b408:	48 83 c4 28          	add    $0x28,%rsp
    b40c:	48 89 da             	mov    %rbx,%rdx
    b40f:	5b                   	pop    %rbx
    b410:	5d                   	pop    %rbp
    b411:	41 5c                	pop    %r12
    b413:	41 5d                	pop    %r13
    b415:	41 5e                	pop    %r14
    b417:	31 c0                	xor    %eax,%eax
    b419:	41 5f                	pop    %r15
    b41b:	c3                   	ret    
    b41c:	0f 1f 40 00          	nopl   0x0(%rax)
    b420:	48 83 c4 28          	add    $0x28,%rsp
    b424:	5b                   	pop    %rbx
    b425:	5d                   	pop    %rbp
    b426:	41 5c                	pop    %r12
    b428:	41 5d                	pop    %r13
    b42a:	41 5e                	pop    %r14
    b42c:	4c 89 fa             	mov    %r15,%rdx
    b42f:	31 c0                	xor    %eax,%eax
    b431:	41 5f                	pop    %r15
    b433:	c3                   	ret    
    b434:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
    b43b:	00 00 00 
    b43e:	66 90                	xchg   %ax,%ax

000000000000b440 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE29_M_get_insert_hint_unique_posESt23_Rb_tree_const_iteratorISA_ERS5_>:
    b440:	41 57                	push   %r15
    b442:	48 8d 47 08          	lea    0x8(%rdi),%rax
    b446:	41 56                	push   %r14
    b448:	49 89 d6             	mov    %rdx,%r14
    b44b:	41 55                	push   %r13
    b44d:	41 54                	push   %r12
    b44f:	55                   	push   %rbp
    b450:	48 89 fd             	mov    %rdi,%rbp
    b453:	53                   	push   %rbx
    b454:	48 83 ec 28          	sub    $0x28,%rsp
    b458:	48 39 c6             	cmp    %rax,%rsi
    b45b:	0f 84 8f 01 00 00    	je     b5f0 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE29_M_get_insert_hint_unique_posESt23_Rb_tree_const_iteratorISA_ERS5_+0x1b0>
    b461:	c5 fa 7e 02          	vmovq  (%rdx),%xmm0
    b465:	c5 fa 7e 4e 20       	vmovq  0x20(%rsi),%xmm1
    b46a:	c5 f9 d6 04 24       	vmovq  %xmm0,(%rsp)
    b46f:	62 f2 f5 08 3b c0    	vpminuq %xmm0,%xmm1,%xmm0
    b475:	c4 c1 f9 7e c4       	vmovq  %xmm0,%r12
    b47a:	4c 8b 6a 08          	mov    0x8(%rdx),%r13
    b47e:	4c 8b 7e 28          	mov    0x28(%rsi),%r15
    b482:	48 89 f3             	mov    %rsi,%rbx
    b485:	4d 85 e4             	test   %r12,%r12
    b488:	0f 84 b2 00 00 00    	je     b540 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE29_M_get_insert_hint_unique_posESt23_Rb_tree_const_iteratorISA_ERS5_+0x100>
    b48e:	c4 e1 f9 7e c2       	vmovq  %xmm0,%rdx
    b493:	4c 89 fe             	mov    %r15,%rsi
    b496:	4c 89 ef             	mov    %r13,%rdi
    b499:	c5 f9 d6 4c 24 18    	vmovq  %xmm1,0x18(%rsp)
    b49f:	e8 cc 7d ff ff       	call   3270 <memcmp@plt>
    b4a4:	85 c0                	test   %eax,%eax
    b4a6:	c5 fa 7e 4c 24 18    	vmovq  0x18(%rsp),%xmm1
    b4ac:	0f 85 ae 00 00 00    	jne    b560 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE29_M_get_insert_hint_unique_posESt23_Rb_tree_const_iteratorISA_ERS5_+0x120>
    b4b2:	c5 f9 6f 24 24       	vmovdqa (%rsp),%xmm4
    b4b7:	c5 d9 fb c1          	vpsubq %xmm1,%xmm4,%xmm0
    b4bb:	c4 e1 f9 7e c0       	vmovq  %xmm0,%rax
    b4c0:	48 3d ff ff ff 7f    	cmp    $0x7fffffff,%rax
    b4c6:	7f 19                	jg     b4e1 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE29_M_get_insert_hint_unique_posESt23_Rb_tree_const_iteratorISA_ERS5_+0xa1>
    b4c8:	48 3d 00 00 00 80    	cmp    $0xffffffff80000000,%rax
    b4ce:	0f 8c 92 00 00 00    	jl     b566 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE29_M_get_insert_hint_unique_posESt23_Rb_tree_const_iteratorISA_ERS5_+0x126>
    b4d4:	85 c0                	test   %eax,%eax
    b4d6:	0f 88 8a 00 00 00    	js     b566 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE29_M_get_insert_hint_unique_posESt23_Rb_tree_const_iteratorISA_ERS5_+0x126>
    b4dc:	4d 85 e4             	test   %r12,%r12
    b4df:	74 1e                	je     b4ff <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE29_M_get_insert_hint_unique_posESt23_Rb_tree_const_iteratorISA_ERS5_+0xbf>
    b4e1:	4c 89 e2             	mov    %r12,%rdx
    b4e4:	4c 89 ee             	mov    %r13,%rsi
    b4e7:	4c 89 ff             	mov    %r15,%rdi
    b4ea:	c5 f9 d6 4c 24 18    	vmovq  %xmm1,0x18(%rsp)
    b4f0:	e8 7b 7d ff ff       	call   3270 <memcmp@plt>
    b4f5:	85 c0                	test   %eax,%eax
    b4f7:	c5 fa 7e 4c 24 18    	vmovq  0x18(%rsp),%xmm1
    b4fd:	75 1e                	jne    b51d <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE29_M_get_insert_hint_unique_posESt23_Rb_tree_const_iteratorISA_ERS5_+0xdd>
    b4ff:	c5 f1 fb 0c 24       	vpsubq (%rsp),%xmm1,%xmm1
    b504:	c4 e1 f9 7e c8       	vmovq  %xmm1,%rax
    b509:	48 3d ff ff ff 7f    	cmp    $0x7fffffff,%rax
    b50f:	7f 14                	jg     b525 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE29_M_get_insert_hint_unique_posESt23_Rb_tree_const_iteratorISA_ERS5_+0xe5>
    b511:	48 3d 00 00 00 80    	cmp    $0xffffffff80000000,%rax
    b517:	0f 8c 43 01 00 00    	jl     b660 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE29_M_get_insert_hint_unique_posESt23_Rb_tree_const_iteratorISA_ERS5_+0x220>
    b51d:	85 c0                	test   %eax,%eax
    b51f:	0f 88 3b 01 00 00    	js     b660 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE29_M_get_insert_hint_unique_posESt23_Rb_tree_const_iteratorISA_ERS5_+0x220>
    b525:	48 89 d8             	mov    %rbx,%rax
    b528:	31 d2                	xor    %edx,%edx
    b52a:	48 83 c4 28          	add    $0x28,%rsp
    b52e:	5b                   	pop    %rbx
    b52f:	5d                   	pop    %rbp
    b530:	41 5c                	pop    %r12
    b532:	41 5d                	pop    %r13
    b534:	41 5e                	pop    %r14
    b536:	41 5f                	pop    %r15
    b538:	c3                   	ret    
    b539:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
    b540:	c5 f9 6f 1c 24       	vmovdqa (%rsp),%xmm3
    b545:	c5 e1 fb c1          	vpsubq %xmm1,%xmm3,%xmm0
    b549:	c4 e1 f9 7e c0       	vmovq  %xmm0,%rax
    b54e:	48 3d ff ff ff 7f    	cmp    $0x7fffffff,%rax
    b554:	0f 8e 6e ff ff ff    	jle    b4c8 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE29_M_get_insert_hint_unique_posESt23_Rb_tree_const_iteratorISA_ERS5_+0x88>
    b55a:	eb a3                	jmp    b4ff <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE29_M_get_insert_hint_unique_posESt23_Rb_tree_const_iteratorISA_ERS5_+0xbf>
    b55c:	0f 1f 40 00          	nopl   0x0(%rax)
    b560:	0f 89 7b ff ff ff    	jns    b4e1 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE29_M_get_insert_hint_unique_posESt23_Rb_tree_const_iteratorISA_ERS5_+0xa1>
    b566:	48 89 d8             	mov    %rbx,%rax
    b569:	48 89 da             	mov    %rbx,%rdx
    b56c:	48 39 5d 18          	cmp    %rbx,0x18(%rbp)
    b570:	74 b8                	je     b52a <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE29_M_get_insert_hint_unique_posESt23_Rb_tree_const_iteratorISA_ERS5_+0xea>
    b572:	48 89 df             	mov    %rbx,%rdi
    b575:	e8 16 7b ff ff       	call   3090 <_ZSt18_Rb_tree_decrementPSt18_Rb_tree_node_base@plt>
    b57a:	c5 fa 7e 40 20       	vmovq  0x20(%rax),%xmm0
    b57f:	49 89 c4             	mov    %rax,%r12
    b582:	62 f2 fd 08 3b 0c 24 	vpminuq (%rsp),%xmm0,%xmm1
    b589:	c4 e1 f9 7e ca       	vmovq  %xmm1,%rdx
    b58e:	48 85 d2             	test   %rdx,%rdx
    b591:	74 1c                	je     b5af <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE29_M_get_insert_hint_unique_posESt23_Rb_tree_const_iteratorISA_ERS5_+0x16f>
    b593:	48 8b 78 28          	mov    0x28(%rax),%rdi
    b597:	4c 89 ee             	mov    %r13,%rsi
    b59a:	c5 f9 d6 44 24 18    	vmovq  %xmm0,0x18(%rsp)
    b5a0:	e8 cb 7c ff ff       	call   3270 <memcmp@plt>
    b5a5:	85 c0                	test   %eax,%eax
    b5a7:	c5 fa 7e 44 24 18    	vmovq  0x18(%rsp),%xmm0
    b5ad:	75 1e                	jne    b5cd <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE29_M_get_insert_hint_unique_posESt23_Rb_tree_const_iteratorISA_ERS5_+0x18d>
    b5af:	c5 f9 fb 04 24       	vpsubq (%rsp),%xmm0,%xmm0
    b5b4:	c4 e1 f9 7e c0       	vmovq  %xmm0,%rax
    b5b9:	48 3d ff ff ff 7f    	cmp    $0x7fffffff,%rax
    b5bf:	0f 8f 00 01 00 00    	jg     b6c5 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE29_M_get_insert_hint_unique_posESt23_Rb_tree_const_iteratorISA_ERS5_+0x285>
    b5c5:	48 3d 00 00 00 80    	cmp    $0xffffffff80000000,%rax
    b5cb:	7c 08                	jl     b5d5 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE29_M_get_insert_hint_unique_posESt23_Rb_tree_const_iteratorISA_ERS5_+0x195>
    b5cd:	85 c0                	test   %eax,%eax
    b5cf:	0f 89 f0 00 00 00    	jns    b6c5 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE29_M_get_insert_hint_unique_posESt23_Rb_tree_const_iteratorISA_ERS5_+0x285>
    b5d5:	49 83 7c 24 18 00    	cmpq   $0x0,0x18(%r12)
    b5db:	b8 00 00 00 00       	mov    $0x0,%eax
    b5e0:	48 0f 45 c3          	cmovne %rbx,%rax
    b5e4:	49 0f 44 dc          	cmove  %r12,%rbx
    b5e8:	48 89 da             	mov    %rbx,%rdx
    b5eb:	e9 3a ff ff ff       	jmp    b52a <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE29_M_get_insert_hint_unique_posESt23_Rb_tree_const_iteratorISA_ERS5_+0xea>
    b5f0:	48 83 7f 28 00       	cmpq   $0x0,0x28(%rdi)
    b5f5:	0f 84 ca 00 00 00    	je     b6c5 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE29_M_get_insert_hint_unique_posESt23_Rb_tree_const_iteratorISA_ERS5_+0x285>
    b5fb:	48 8b 5f 20          	mov    0x20(%rdi),%rbx
    b5ff:	4c 8b 2a             	mov    (%rdx),%r13
    b602:	4c 8b 63 20          	mov    0x20(%rbx),%r12
    b606:	4d 39 e5             	cmp    %r12,%r13
    b609:	4c 89 e2             	mov    %r12,%rdx
    b60c:	49 0f 46 d5          	cmovbe %r13,%rdx
    b610:	48 85 d2             	test   %rdx,%rdx
    b613:	74 11                	je     b626 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE29_M_get_insert_hint_unique_posESt23_Rb_tree_const_iteratorISA_ERS5_+0x1e6>
    b615:	49 8b 76 08          	mov    0x8(%r14),%rsi
    b619:	48 8b 7b 28          	mov    0x28(%rbx),%rdi
    b61d:	e8 4e 7c ff ff       	call   3270 <memcmp@plt>
    b622:	85 c0                	test   %eax,%eax
    b624:	75 1c                	jne    b642 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE29_M_get_insert_hint_unique_posESt23_Rb_tree_const_iteratorISA_ERS5_+0x202>
    b626:	4d 29 ec             	sub    %r13,%r12
    b629:	49 81 fc ff ff ff 7f 	cmp    $0x7fffffff,%r12
    b630:	0f 8f 8f 00 00 00    	jg     b6c5 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE29_M_get_insert_hint_unique_posESt23_Rb_tree_const_iteratorISA_ERS5_+0x285>
    b636:	49 81 fc 00 00 00 80 	cmp    $0xffffffff80000000,%r12
    b63d:	7c 07                	jl     b646 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE29_M_get_insert_hint_unique_posESt23_Rb_tree_const_iteratorISA_ERS5_+0x206>
    b63f:	44 89 e0             	mov    %r12d,%eax
    b642:	85 c0                	test   %eax,%eax
    b644:	79 7f                	jns    b6c5 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE29_M_get_insert_hint_unique_posESt23_Rb_tree_const_iteratorISA_ERS5_+0x285>
    b646:	48 83 c4 28          	add    $0x28,%rsp
    b64a:	48 89 da             	mov    %rbx,%rdx
    b64d:	5b                   	pop    %rbx
    b64e:	5d                   	pop    %rbp
    b64f:	41 5c                	pop    %r12
    b651:	41 5d                	pop    %r13
    b653:	41 5e                	pop    %r14
    b655:	31 c0                	xor    %eax,%eax
    b657:	41 5f                	pop    %r15
    b659:	c3                   	ret    
    b65a:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)
    b660:	48 39 5d 20          	cmp    %rbx,0x20(%rbp)
    b664:	74 e0                	je     b646 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE29_M_get_insert_hint_unique_posESt23_Rb_tree_const_iteratorISA_ERS5_+0x206>
    b666:	48 89 df             	mov    %rbx,%rdi
    b669:	e8 42 7a ff ff       	call   30b0 <_ZSt18_Rb_tree_incrementPSt18_Rb_tree_node_base@plt>
    b66e:	c5 fa 7e 40 20       	vmovq  0x20(%rax),%xmm0
    b673:	49 89 c4             	mov    %rax,%r12
    b676:	62 f2 fd 08 3b 0c 24 	vpminuq (%rsp),%xmm0,%xmm1
    b67d:	c4 e1 f9 7e ca       	vmovq  %xmm1,%rdx
    b682:	48 85 d2             	test   %rdx,%rdx
    b685:	74 1c                	je     b6a3 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE29_M_get_insert_hint_unique_posESt23_Rb_tree_const_iteratorISA_ERS5_+0x263>
    b687:	48 8b 70 28          	mov    0x28(%rax),%rsi
    b68b:	4c 89 ef             	mov    %r13,%rdi
    b68e:	c5 f9 d6 44 24 18    	vmovq  %xmm0,0x18(%rsp)
    b694:	e8 d7 7b ff ff       	call   3270 <memcmp@plt>
    b699:	85 c0                	test   %eax,%eax
    b69b:	c5 fa 7e 44 24 18    	vmovq  0x18(%rsp),%xmm0
    b6a1:	75 1e                	jne    b6c1 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE29_M_get_insert_hint_unique_posESt23_Rb_tree_const_iteratorISA_ERS5_+0x281>
    b6a3:	c5 f9 6f 2c 24       	vmovdqa (%rsp),%xmm5
    b6a8:	c5 d1 fb c0          	vpsubq %xmm0,%xmm5,%xmm0
    b6ac:	c4 e1 f9 7e c0       	vmovq  %xmm0,%rax
    b6b1:	48 3d ff ff ff 7f    	cmp    $0x7fffffff,%rax
    b6b7:	7f 0c                	jg     b6c5 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE29_M_get_insert_hint_unique_posESt23_Rb_tree_const_iteratorISA_ERS5_+0x285>
    b6b9:	48 3d 00 00 00 80    	cmp    $0xffffffff80000000,%rax
    b6bf:	7c 1f                	jl     b6e0 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE29_M_get_insert_hint_unique_posESt23_Rb_tree_const_iteratorISA_ERS5_+0x2a0>
    b6c1:	85 c0                	test   %eax,%eax
    b6c3:	78 1b                	js     b6e0 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE29_M_get_insert_hint_unique_posESt23_Rb_tree_const_iteratorISA_ERS5_+0x2a0>
    b6c5:	48 83 c4 28          	add    $0x28,%rsp
    b6c9:	5b                   	pop    %rbx
    b6ca:	48 89 ef             	mov    %rbp,%rdi
    b6cd:	5d                   	pop    %rbp
    b6ce:	41 5c                	pop    %r12
    b6d0:	41 5d                	pop    %r13
    b6d2:	4c 89 f6             	mov    %r14,%rsi
    b6d5:	41 5e                	pop    %r14
    b6d7:	41 5f                	pop    %r15
    b6d9:	e9 c2 fb ff ff       	jmp    b2a0 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE24_M_get_insert_unique_posERS5_>
    b6de:	66 90                	xchg   %ax,%ax
    b6e0:	48 83 7b 18 00       	cmpq   $0x0,0x18(%rbx)
    b6e5:	49 0f 45 dc          	cmovne %r12,%rbx
    b6e9:	b8 00 00 00 00       	mov    $0x0,%eax
    b6ee:	49 0f 45 c4          	cmovne %r12,%rax
    b6f2:	48 89 da             	mov    %rbx,%rdx
    b6f5:	e9 30 fe ff ff       	jmp    b52a <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE29_M_get_insert_hint_unique_posESt23_Rb_tree_const_iteratorISA_ERS5_+0xea>
    b6fa:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)

000000000000b700 <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_>:
    b700:	41 57                	push   %r15
    b702:	49 bf ff ff ff ff ff 	movabs $0x7ffffffffffffff,%r15
    b709:	ff ff 07 
    b70c:	41 56                	push   %r14
    b70e:	41 55                	push   %r13
    b710:	41 54                	push   %r12
    b712:	55                   	push   %rbp
    b713:	53                   	push   %rbx
    b714:	48 83 ec 48          	sub    $0x48,%rsp
    b718:	48 89 7c 24 10       	mov    %rdi,0x10(%rsp)
    b71d:	4c 8b 67 08          	mov    0x8(%rdi),%r12
    b721:	48 8b 0f             	mov    (%rdi),%rcx
    b724:	64 48 8b 04 25 28 00 	mov    %fs:0x28,%rax
    b72b:	00 00 
    b72d:	48 89 44 24 38       	mov    %rax,0x38(%rsp)
    b732:	31 c0                	xor    %eax,%eax
    b734:	4c 89 e0             	mov    %r12,%rax
    b737:	48 29 c8             	sub    %rcx,%rax
    b73a:	48 c1 f8 04          	sar    $0x4,%rax
    b73e:	48 89 0c 24          	mov    %rcx,(%rsp)
    b742:	4c 39 f8             	cmp    %r15,%rax
    b745:	0f 84 46 02 00 00    	je     b991 <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0x291>
    b74b:	48 89 f5             	mov    %rsi,%rbp
    b74e:	48 89 d6             	mov    %rdx,%rsi
    b751:	48 89 c2             	mov    %rax,%rdx
    b754:	b8 01 00 00 00       	mov    $0x1,%eax
    b759:	48 39 c2             	cmp    %rax,%rdx
    b75c:	48 0f 43 c2          	cmovae %rdx,%rax
    b760:	31 c9                	xor    %ecx,%ecx
    b762:	48 01 d0             	add    %rdx,%rax
    b765:	0f 92 c1             	setb   %cl
    b768:	48 89 ea             	mov    %rbp,%rdx
    b76b:	48 89 eb             	mov    %rbp,%rbx
    b76e:	48 2b 14 24          	sub    (%rsp),%rdx
    b772:	48 85 c9             	test   %rcx,%rcx
    b775:	0f 85 f5 01 00 00    	jne    b970 <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0x270>
    b77b:	48 85 c0             	test   %rax,%rax
    b77e:	0f 85 7c 01 00 00    	jne    b900 <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0x200>
    b784:	48 c7 44 24 18 00 00 	movq   $0x0,0x18(%rsp)
    b78b:	00 00 
    b78d:	48 c7 44 24 08 00 00 	movq   $0x0,0x8(%rsp)
    b794:	00 00 
    b796:	41 bf 10 00 00 00    	mov    $0x10,%r15d
    b79c:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
    b7a1:	48 8b 0e             	mov    (%rsi),%rcx
    b7a4:	48 01 d0             	add    %rdx,%rax
    b7a7:	48 c7 40 08 00 00 00 	movq   $0x0,0x8(%rax)
    b7ae:	00 
    b7af:	48 85 c9             	test   %rcx,%rcx
    b7b2:	0f 84 c8 01 00 00    	je     b980 <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0x280>
    b7b8:	48 89 44 24 30       	mov    %rax,0x30(%rsp)
    b7bd:	48 8d 54 24 30       	lea    0x30(%rsp),%rdx
    b7c2:	bf 04 00 00 00       	mov    $0x4,%edi
    b7c7:	ff d1                	call   *%rcx
    b7c9:	4c 8b 34 24          	mov    (%rsp),%r14
    b7cd:	4c 39 f5             	cmp    %r14,%rbp
    b7d0:	74 5f                	je     b831 <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0x131>
    b7d2:	4c 8b 6c 24 08       	mov    0x8(%rsp),%r13
    b7d7:	4c 8d 7c 24 30       	lea    0x30(%rsp),%r15
    b7dc:	0f 1f 40 00          	nopl   0x0(%rax)
    b7e0:	49 c7 45 08 00 00 00 	movq   $0x0,0x8(%r13)
    b7e7:	00 
    b7e8:	4d 8b 06             	mov    (%r14),%r8
    b7eb:	4d 85 c0             	test   %r8,%r8
    b7ee:	0f 84 4c 01 00 00    	je     b940 <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0x240>
    b7f4:	4c 89 6c 24 30       	mov    %r13,0x30(%rsp)
    b7f9:	4c 89 fa             	mov    %r15,%rdx
    b7fc:	4c 89 f6             	mov    %r14,%rsi
    b7ff:	bf 04 00 00 00       	mov    $0x4,%edi
    b804:	41 ff d0             	call   *%r8
    b807:	4d 8b 06             	mov    (%r14),%r8
    b80a:	4d 85 c0             	test   %r8,%r8
    b80d:	0f 84 41 01 00 00    	je     b954 <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0x254>
    b813:	4c 89 f6             	mov    %r14,%rsi
    b816:	31 d2                	xor    %edx,%edx
    b818:	bf 03 00 00 00       	mov    $0x3,%edi
    b81d:	49 83 c6 10          	add    $0x10,%r14
    b821:	41 ff d0             	call   *%r8
    b824:	49 83 c5 10          	add    $0x10,%r13
    b828:	49 39 ee             	cmp    %rbp,%r14
    b82b:	75 b3                	jne    b7e0 <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0xe0>
    b82d:	4d 8d 7d 10          	lea    0x10(%r13),%r15
    b831:	4c 39 e5             	cmp    %r12,%rbp
    b834:	74 61                	je     b897 <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0x197>
    b836:	48 8d 6c 24 30       	lea    0x30(%rsp),%rbp
    b83b:	eb 36                	jmp    b873 <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0x173>
    b83d:	0f 1f 00             	nopl   (%rax)
    b840:	4c 89 7c 24 30       	mov    %r15,0x30(%rsp)
    b845:	48 89 ea             	mov    %rbp,%rdx
    b848:	48 89 de             	mov    %rbx,%rsi
    b84b:	bf 04 00 00 00       	mov    $0x4,%edi
    b850:	ff d0                	call   *%rax
    b852:	48 8b 03             	mov    (%rbx),%rax
    b855:	48 85 c0             	test   %rax,%rax
    b858:	74 0c                	je     b866 <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0x166>
    b85a:	31 d2                	xor    %edx,%edx
    b85c:	48 89 de             	mov    %rbx,%rsi
    b85f:	bf 03 00 00 00       	mov    $0x3,%edi
    b864:	ff d0                	call   *%rax
    b866:	48 83 c3 10          	add    $0x10,%rbx
    b86a:	49 83 c7 10          	add    $0x10,%r15
    b86e:	4c 39 e3             	cmp    %r12,%rbx
    b871:	74 24                	je     b897 <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0x197>
    b873:	48 8b 03             	mov    (%rbx),%rax
    b876:	49 c7 47 08 00 00 00 	movq   $0x0,0x8(%r15)
    b87d:	00 
    b87e:	48 85 c0             	test   %rax,%rax
    b881:	75 bd                	jne    b840 <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0x140>
    b883:	48 83 c3 10          	add    $0x10,%rbx
    b887:	49 c7 07 00 00 00 00 	movq   $0x0,(%r15)
    b88e:	49 83 c7 10          	add    $0x10,%r15
    b892:	4c 39 e3             	cmp    %r12,%rbx
    b895:	75 dc                	jne    b873 <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0x173>
    b897:	48 8b 04 24          	mov    (%rsp),%rax
    b89b:	48 85 c0             	test   %rax,%rax
    b89e:	74 1b                	je     b8bb <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0x1bb>
    b8a0:	48 8b 4c 24 10       	mov    0x10(%rsp),%rcx
    b8a5:	48 89 c7             	mov    %rax,%rdi
    b8a8:	48 8b 49 10          	mov    0x10(%rcx),%rcx
    b8ac:	48 89 ce             	mov    %rcx,%rsi
    b8af:	48 29 c6             	sub    %rax,%rsi
    b8b2:	48 89 0c 24          	mov    %rcx,(%rsp)
    b8b6:	e8 65 7a ff ff       	call   3320 <_ZdlPvm@plt>
    b8bb:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
    b8c0:	48 8b 4c 24 18       	mov    0x18(%rsp),%rcx
    b8c5:	c5 fa 7e 4c 24 08    	vmovq  0x8(%rsp),%xmm1
    b8cb:	48 89 48 10          	mov    %rcx,0x10(%rax)
    b8cf:	c4 c3 f1 22 c7 01    	vpinsrq $0x1,%r15,%xmm1,%xmm0
    b8d5:	c5 fa 7f 00          	vmovdqu %xmm0,(%rax)
    b8d9:	48 8b 44 24 38       	mov    0x38(%rsp),%rax
    b8de:	64 48 2b 04 25 28 00 	sub    %fs:0x28,%rax
    b8e5:	00 00 
    b8e7:	0f 85 9f 00 00 00    	jne    b98c <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0x28c>
    b8ed:	48 83 c4 48          	add    $0x48,%rsp
    b8f1:	5b                   	pop    %rbx
    b8f2:	5d                   	pop    %rbp
    b8f3:	41 5c                	pop    %r12
    b8f5:	41 5d                	pop    %r13
    b8f7:	41 5e                	pop    %r14
    b8f9:	41 5f                	pop    %r15
    b8fb:	c3                   	ret    
    b8fc:	0f 1f 40 00          	nopl   0x0(%rax)
    b900:	4c 39 f8             	cmp    %r15,%rax
    b903:	4c 0f 46 f8          	cmovbe %rax,%r15
    b907:	49 c1 e7 04          	shl    $0x4,%r15
    b90b:	4c 89 ff             	mov    %r15,%rdi
    b90e:	48 89 74 24 28       	mov    %rsi,0x28(%rsp)
    b913:	48 89 54 24 20       	mov    %rdx,0x20(%rsp)
    b918:	e8 83 79 ff ff       	call   32a0 <_Znwm@plt>
    b91d:	49 01 c7             	add    %rax,%r15
    b920:	4c 89 7c 24 18       	mov    %r15,0x18(%rsp)
    b925:	48 89 44 24 08       	mov    %rax,0x8(%rsp)
    b92a:	48 8b 54 24 20       	mov    0x20(%rsp),%rdx
    b92f:	48 8b 74 24 28       	mov    0x28(%rsp),%rsi
    b934:	4c 8d 78 10          	lea    0x10(%rax),%r15
    b938:	e9 5f fe ff ff       	jmp    b79c <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0x9c>
    b93d:	0f 1f 00             	nopl   (%rax)
    b940:	49 c7 45 00 00 00 00 	movq   $0x0,0x0(%r13)
    b947:	00 
    b948:	4d 8b 06             	mov    (%r14),%r8
    b94b:	4d 85 c0             	test   %r8,%r8
    b94e:	0f 85 bf fe ff ff    	jne    b813 <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0x113>
    b954:	49 83 c6 10          	add    $0x10,%r14
    b958:	49 83 c5 10          	add    $0x10,%r13
    b95c:	49 39 ee             	cmp    %rbp,%r14
    b95f:	0f 85 7b fe ff ff    	jne    b7e0 <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0xe0>
    b965:	e9 c3 fe ff ff       	jmp    b82d <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0x12d>
    b96a:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)
    b970:	49 bf f0 ff ff ff ff 	movabs $0x7ffffffffffffff0,%r15
    b977:	ff ff 7f 
    b97a:	eb 8f                	jmp    b90b <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0x20b>
    b97c:	0f 1f 40 00          	nopl   0x0(%rax)
    b980:	48 c7 00 00 00 00 00 	movq   $0x0,(%rax)
    b987:	e9 3d fe ff ff       	jmp    b7c9 <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_+0xc9>
    b98c:	e8 bf 77 ff ff       	call   3150 <__stack_chk_fail@plt>
    b991:	48 8d 3d 78 49 00 00 	lea    0x4978(%rip),%rdi        # 10310 <_fini+0x10ef>
    b998:	e8 a3 79 ff ff       	call   3340 <_ZSt20__throw_length_errorPKc@plt>
    b99d:	0f 1f 00             	nopl   (%rax)

000000000000b9a0 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm0EEEE14__visit_invokeESQ_S10_>:
    b9a0:	41 57                	push   %r15
    b9a2:	41 56                	push   %r14
    b9a4:	41 55                	push   %r13
    b9a6:	41 54                	push   %r12
    b9a8:	49 89 fc             	mov    %rdi,%r12
    b9ab:	55                   	push   %rbp
    b9ac:	53                   	push   %rbx
    b9ad:	48 83 ec 58          	sub    $0x58,%rsp
    b9b1:	64 48 8b 04 25 28 00 	mov    %fs:0x28,%rax
    b9b8:	00 00 
    b9ba:	48 89 44 24 48       	mov    %rax,0x48(%rsp)
    b9bf:	31 c0                	xor    %eax,%eax
    b9c1:	48 8b 46 10          	mov    0x10(%rsi),%rax
    b9c5:	48 c7 44 24 30 00 00 	movq   $0x0,0x30(%rsp)
    b9cc:	00 00 
    b9ce:	48 85 c0             	test   %rax,%rax
    b9d1:	0f 84 39 01 00 00    	je     bb10 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm0EEEE14__visit_invokeESQ_S10_+0x170>
    b9d7:	4c 8d 6c 24 20       	lea    0x20(%rsp),%r13
    b9dc:	ba 02 00 00 00       	mov    $0x2,%edx
    b9e1:	4c 89 ef             	mov    %r13,%rdi
    b9e4:	48 89 f3             	mov    %rsi,%rbx
    b9e7:	ff d0                	call   *%rax
    b9e9:	c5 fa 6f 43 10       	vmovdqu 0x10(%rbx),%xmm0
    b9ee:	48 8b 43 10          	mov    0x10(%rbx),%rax
    b9f2:	49 8b 6c 24 10       	mov    0x10(%r12),%rbp
    b9f7:	49 8b 1c 24          	mov    (%r12),%rbx
    b9fb:	4d 8b 74 24 08       	mov    0x8(%r12),%r14
    ba00:	c5 f9 7f 44 24 30    	vmovdqa %xmm0,0x30(%rsp)
    ba06:	4c 8d bd b0 00 00 00 	lea    0xb0(%rbp),%r15
    ba0d:	49 39 de             	cmp    %rbx,%r14
    ba10:	0f 84 8a 00 00 00    	je     baa0 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm0EEEE14__visit_invokeESQ_S10_+0x100>
    ba16:	4c 8d 64 24 10       	lea    0x10(%rsp),%r12
    ba1b:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)
    ba20:	48 85 c0             	test   %rax,%rax
    ba23:	0f 84 f0 00 00 00    	je     bb19 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm0EEEE14__visit_invokeESQ_S10_+0x179>
    ba29:	48 89 da             	mov    %rbx,%rdx
    ba2c:	4c 89 ee             	mov    %r13,%rsi
    ba2f:	4c 89 e7             	mov    %r12,%rdi
    ba32:	ff 54 24 38          	call   *0x38(%rsp)
    ba36:	48 8b b5 b8 00 00 00 	mov    0xb8(%rbp),%rsi
    ba3d:	48 3b b5 c0 00 00 00 	cmp    0xc0(%rbp),%rsi
    ba44:	0f 84 b6 00 00 00    	je     bb00 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm0EEEE14__visit_invokeESQ_S10_+0x160>
    ba4a:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
    ba4f:	48 c7 46 08 00 00 00 	movq   $0x0,0x8(%rsi)
    ba56:	00 
    ba57:	48 85 c0             	test   %rax,%rax
    ba5a:	0f 84 90 00 00 00    	je     baf0 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm0EEEE14__visit_invokeESQ_S10_+0x150>
    ba60:	48 89 74 24 08       	mov    %rsi,0x8(%rsp)
    ba65:	48 8d 54 24 08       	lea    0x8(%rsp),%rdx
    ba6a:	4c 89 e6             	mov    %r12,%rsi
    ba6d:	bf 04 00 00 00       	mov    $0x4,%edi
    ba72:	ff d0                	call   *%rax
    ba74:	48 83 85 b8 00 00 00 	addq   $0x10,0xb8(%rbp)
    ba7b:	10 
    ba7c:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
    ba81:	48 85 c0             	test   %rax,%rax
    ba84:	74 52                	je     bad8 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm0EEEE14__visit_invokeESQ_S10_+0x138>
    ba86:	31 d2                	xor    %edx,%edx
    ba88:	4c 89 e6             	mov    %r12,%rsi
    ba8b:	bf 03 00 00 00       	mov    $0x3,%edi
    ba90:	48 83 c3 20          	add    $0x20,%rbx
    ba94:	ff d0                	call   *%rax
    ba96:	48 8b 44 24 30       	mov    0x30(%rsp),%rax
    ba9b:	49 39 de             	cmp    %rbx,%r14
    ba9e:	75 80                	jne    ba20 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm0EEEE14__visit_invokeESQ_S10_+0x80>
    baa0:	48 85 c0             	test   %rax,%rax
    baa3:	74 0d                	je     bab2 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm0EEEE14__visit_invokeESQ_S10_+0x112>
    baa5:	ba 03 00 00 00       	mov    $0x3,%edx
    baaa:	4c 89 ee             	mov    %r13,%rsi
    baad:	4c 89 ef             	mov    %r13,%rdi
    bab0:	ff d0                	call   *%rax
    bab2:	48 8b 44 24 48       	mov    0x48(%rsp),%rax
    bab7:	64 48 2b 04 25 28 00 	sub    %fs:0x28,%rax
    babe:	00 00 
    bac0:	75 5c                	jne    bb1e <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm0EEEE14__visit_invokeESQ_S10_+0x17e>
    bac2:	48 83 c4 58          	add    $0x58,%rsp
    bac6:	5b                   	pop    %rbx
    bac7:	5d                   	pop    %rbp
    bac8:	41 5c                	pop    %r12
    baca:	41 5d                	pop    %r13
    bacc:	41 5e                	pop    %r14
    bace:	41 5f                	pop    %r15
    bad0:	c3                   	ret    
    bad1:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
    bad8:	48 83 c3 20          	add    $0x20,%rbx
    badc:	48 8b 44 24 30       	mov    0x30(%rsp),%rax
    bae1:	49 39 de             	cmp    %rbx,%r14
    bae4:	0f 85 36 ff ff ff    	jne    ba20 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm0EEEE14__visit_invokeESQ_S10_+0x80>
    baea:	eb b4                	jmp    baa0 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm0EEEE14__visit_invokeESQ_S10_+0x100>
    baec:	0f 1f 40 00          	nopl   0x0(%rax)
    baf0:	48 c7 06 00 00 00 00 	movq   $0x0,(%rsi)
    baf7:	e9 78 ff ff ff       	jmp    ba74 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm0EEEE14__visit_invokeESQ_S10_+0xd4>
    bafc:	0f 1f 40 00          	nopl   0x0(%rax)
    bb00:	4c 89 e2             	mov    %r12,%rdx
    bb03:	4c 89 ff             	mov    %r15,%rdi
    bb06:	e8 f5 fb ff ff       	call   b700 <_ZNSt6vectorISt3anySaIS0_EE17_M_realloc_insertIJS0_EEEvN9__gnu_cxx17__normal_iteratorIPS0_S2_EEDpOT_>
    bb0b:	e9 6c ff ff ff       	jmp    ba7c <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm0EEEE14__visit_invokeESQ_S10_+0xdc>
    bb10:	48 8b 07             	mov    (%rdi),%rax
    bb13:	48 39 47 08          	cmp    %rax,0x8(%rdi)
    bb17:	74 99                	je     bab2 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm0EEEE14__visit_invokeESQ_S10_+0x112>
    bb19:	e8 a2 76 ff ff       	call   31c0 <_ZSt25__throw_bad_function_callv@plt>
    bb1e:	e8 2d 76 ff ff       	call   3150 <__stack_chk_fail@plt>
    bb23:	48 89 c5             	mov    %rax,%rbp
    bb26:	eb 23                	jmp    bb4b <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm0EEEE14__visit_invokeESQ_S10_+0x1ab>
    bb28:	48 89 c5             	mov    %rax,%rbp
    bb2b:	eb 05                	jmp    bb32 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm0EEEE14__visit_invokeESQ_S10_+0x192>
    bb2d:	48 89 c5             	mov    %rax,%rbp
    bb30:	eb 37                	jmp    bb69 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm0EEEE14__visit_invokeESQ_S10_+0x1c9>
    bb32:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
    bb37:	48 85 c0             	test   %rax,%rax
    bb3a:	74 0f                	je     bb4b <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm0EEEE14__visit_invokeESQ_S10_+0x1ab>
    bb3c:	31 d2                	xor    %edx,%edx
    bb3e:	4c 89 e6             	mov    %r12,%rsi
    bb41:	bf 03 00 00 00       	mov    $0x3,%edi
    bb46:	c5 f8 77             	vzeroupper 
    bb49:	ff d0                	call   *%rax
    bb4b:	48 8b 44 24 30       	mov    0x30(%rsp),%rax
    bb50:	48 85 c0             	test   %rax,%rax
    bb53:	74 36                	je     bb8b <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm0EEEE14__visit_invokeESQ_S10_+0x1eb>
    bb55:	48 8d 7c 24 20       	lea    0x20(%rsp),%rdi
    bb5a:	ba 03 00 00 00       	mov    $0x3,%edx
    bb5f:	48 89 fe             	mov    %rdi,%rsi
    bb62:	c5 f8 77             	vzeroupper 
    bb65:	ff d0                	call   *%rax
    bb67:	eb 1a                	jmp    bb83 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm0EEEE14__visit_invokeESQ_S10_+0x1e3>
    bb69:	48 8b 44 24 30       	mov    0x30(%rsp),%rax
    bb6e:	48 85 c0             	test   %rax,%rax
    bb71:	74 18                	je     bb8b <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm0EEEE14__visit_invokeESQ_S10_+0x1eb>
    bb73:	ba 03 00 00 00       	mov    $0x3,%edx
    bb78:	4c 89 ee             	mov    %r13,%rsi
    bb7b:	4c 89 ef             	mov    %r13,%rdi
    bb7e:	c5 f8 77             	vzeroupper 
    bb81:	ff d0                	call   *%rax
    bb83:	48 89 ef             	mov    %rbp,%rdi
    bb86:	e8 d5 77 ff ff       	call   3360 <_Unwind_Resume@plt>
    bb8b:	c5 f8 77             	vzeroupper 
    bb8e:	eb f3                	jmp    bb83 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFNS0_21__deduce_visit_resultIvEEOZN8argparse8Argument7consumeIN9__gnu_cxx17__normal_iteratorIPKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorISF_SaISF_EEEEEET_SM_SM_St17basic_string_viewIcSD_EE11ActionApplyRSt7variantIJSt8functionIFSt3anyRSG_EESS_IFvSU_EEEEEJEEESt16integer_sequenceImJLm0EEEE14__visit_invokeESQ_S10_+0x1e3>

000000000000bb90 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJRSt17basic_string_viewIcS3_EEEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_>:
    bb90:	41 57                	push   %r15
    bb92:	41 56                	push   %r14
    bb94:	41 55                	push   %r13
    bb96:	41 54                	push   %r12
    bb98:	55                   	push   %rbp
    bb99:	53                   	push   %rbx
    bb9a:	48 89 f3             	mov    %rsi,%rbx
    bb9d:	48 83 ec 48          	sub    $0x48,%rsp
    bba1:	48 89 74 24 10       	mov    %rsi,0x10(%rsp)
    bba6:	48 8b 6f 08          	mov    0x8(%rdi),%rbp
    bbaa:	4c 8b 2f             	mov    (%rdi),%r13
    bbad:	64 48 8b 04 25 28 00 	mov    %fs:0x28,%rax
    bbb4:	00 00 
    bbb6:	48 89 44 24 38       	mov    %rax,0x38(%rsp)
    bbbb:	31 c0                	xor    %eax,%eax
    bbbd:	48 89 e8             	mov    %rbp,%rax
    bbc0:	4c 29 e8             	sub    %r13,%rax
    bbc3:	48 c1 f8 05          	sar    $0x5,%rax
    bbc7:	48 be ff ff ff ff ff 	movabs $0x3ffffffffffffff,%rsi
    bbce:	ff ff 03 
    bbd1:	48 39 f0             	cmp    %rsi,%rax
    bbd4:	0f 84 c1 02 00 00    	je     be9b <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJRSt17basic_string_viewIcS3_EEEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_+0x30b>
    bbda:	48 89 d1             	mov    %rdx,%rcx
    bbdd:	ba 01 00 00 00       	mov    $0x1,%edx
    bbe2:	48 39 d0             	cmp    %rdx,%rax
    bbe5:	48 0f 43 d0          	cmovae %rax,%rdx
    bbe9:	4c 8b 74 24 10       	mov    0x10(%rsp),%r14
    bbee:	48 01 d0             	add    %rdx,%rax
    bbf1:	0f 92 c2             	setb   %dl
    bbf4:	0f b6 d2             	movzbl %dl,%edx
    bbf7:	48 89 44 24 18       	mov    %rax,0x18(%rsp)
    bbfc:	49 89 fc             	mov    %rdi,%r12
    bbff:	4d 29 ee             	sub    %r13,%r14
    bc02:	48 85 d2             	test   %rdx,%rdx
    bc05:	0f 85 05 02 00 00    	jne    be10 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJRSt17basic_string_viewIcS3_EEEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_+0x280>
    bc0b:	48 85 c0             	test   %rax,%rax
    bc0e:	0f 85 6c 01 00 00    	jne    bd80 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJRSt17basic_string_viewIcS3_EEEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_+0x1f0>
    bc14:	48 c7 44 24 08 00 00 	movq   $0x0,0x8(%rsp)
    bc1b:	00 00 
    bc1d:	4c 03 74 24 08       	add    0x8(%rsp),%r14
    bc22:	48 8b 41 08          	mov    0x8(%rcx),%rax
    bc26:	49 8d 7e 10          	lea    0x10(%r14),%rdi
    bc2a:	4c 8b 39             	mov    (%rcx),%r15
    bc2d:	48 89 7c 24 20       	mov    %rdi,0x20(%rsp)
    bc32:	49 89 3e             	mov    %rdi,(%r14)
    bc35:	48 89 c7             	mov    %rax,%rdi
    bc38:	4c 01 ff             	add    %r15,%rdi
    bc3b:	48 89 44 24 28       	mov    %rax,0x28(%rsp)
    bc40:	74 09                	je     bc4b <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJRSt17basic_string_viewIcS3_EEEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_+0xbb>
    bc42:	48 85 c0             	test   %rax,%rax
    bc45:	0f 84 3f 02 00 00    	je     be8a <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJRSt17basic_string_viewIcS3_EEEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_+0x2fa>
    bc4b:	4c 89 7c 24 30       	mov    %r15,0x30(%rsp)
    bc50:	49 83 ff 0f          	cmp    $0xf,%r15
    bc54:	0f 87 e6 01 00 00    	ja     be40 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJRSt17basic_string_viewIcS3_EEEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_+0x2b0>
    bc5a:	49 83 ff 01          	cmp    $0x1,%r15
    bc5e:	0f 85 cc 01 00 00    	jne    be30 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJRSt17basic_string_viewIcS3_EEEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_+0x2a0>
    bc64:	48 8b 44 24 28       	mov    0x28(%rsp),%rax
    bc69:	0f b6 00             	movzbl (%rax),%eax
    bc6c:	41 88 46 10          	mov    %al,0x10(%r14)
    bc70:	48 8b 44 24 20       	mov    0x20(%rsp),%rax
    bc75:	4d 89 7e 08          	mov    %r15,0x8(%r14)
    bc79:	42 c6 04 38 00       	movb   $0x0,(%rax,%r15,1)
    bc7e:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
    bc83:	4c 39 e8             	cmp    %r13,%rax
    bc86:	0f 84 f4 01 00 00    	je     be80 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJRSt17basic_string_viewIcS3_EEEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_+0x2f0>
    bc8c:	4c 29 e8             	sub    %r13,%rax
    bc8f:	48 89 c7             	mov    %rax,%rdi
    bc92:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
    bc97:	4c 89 ea             	mov    %r13,%rdx
    bc9a:	48 01 c7             	add    %rax,%rdi
    bc9d:	0f 1f 00             	nopl   (%rax)
    bca0:	48 8d 48 10          	lea    0x10(%rax),%rcx
    bca4:	48 89 08             	mov    %rcx,(%rax)
    bca7:	48 8d 72 10          	lea    0x10(%rdx),%rsi
    bcab:	48 8b 0a             	mov    (%rdx),%rcx
    bcae:	48 39 f1             	cmp    %rsi,%rcx
    bcb1:	0f 84 29 01 00 00    	je     bde0 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJRSt17basic_string_viewIcS3_EEEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_+0x250>
    bcb7:	48 89 08             	mov    %rcx,(%rax)
    bcba:	48 83 c0 20          	add    $0x20,%rax
    bcbe:	48 83 c2 20          	add    $0x20,%rdx
    bcc2:	48 8b 4a f0          	mov    -0x10(%rdx),%rcx
    bcc6:	48 89 48 f0          	mov    %rcx,-0x10(%rax)
    bcca:	48 8b 4a e8          	mov    -0x18(%rdx),%rcx
    bcce:	48 89 48 e8          	mov    %rcx,-0x18(%rax)
    bcd2:	48 39 f8             	cmp    %rdi,%rax
    bcd5:	75 c9                	jne    bca0 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJRSt17basic_string_viewIcS3_EEEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_+0x110>
    bcd7:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
    bcdc:	4c 8d 77 20          	lea    0x20(%rdi),%r14
    bce0:	48 39 e8             	cmp    %rbp,%rax
    bce3:	74 45                	je     bd2a <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJRSt17basic_string_viewIcS3_EEEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_+0x19a>
    bce5:	48 89 ee             	mov    %rbp,%rsi
    bce8:	48 29 c6             	sub    %rax,%rsi
    bceb:	4c 89 f0             	mov    %r14,%rax
    bcee:	66 90                	xchg   %ax,%ax
    bcf0:	48 8d 50 10          	lea    0x10(%rax),%rdx
    bcf4:	48 89 10             	mov    %rdx,(%rax)
    bcf7:	48 8b 13             	mov    (%rbx),%rdx
    bcfa:	48 8d 4b 10          	lea    0x10(%rbx),%rcx
    bcfe:	48 39 ca             	cmp    %rcx,%rdx
    bd01:	0f 84 a9 00 00 00    	je     bdb0 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJRSt17basic_string_viewIcS3_EEEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_+0x220>
    bd07:	48 89 10             	mov    %rdx,(%rax)
    bd0a:	48 8b 53 10          	mov    0x10(%rbx),%rdx
    bd0e:	48 83 c3 20          	add    $0x20,%rbx
    bd12:	48 89 50 10          	mov    %rdx,0x10(%rax)
    bd16:	48 8b 53 e8          	mov    -0x18(%rbx),%rdx
    bd1a:	48 83 c0 20          	add    $0x20,%rax
    bd1e:	48 89 50 e8          	mov    %rdx,-0x18(%rax)
    bd22:	48 39 eb             	cmp    %rbp,%rbx
    bd25:	75 c9                	jne    bcf0 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJRSt17basic_string_viewIcS3_EEEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_+0x160>
    bd27:	49 01 f6             	add    %rsi,%r14
    bd2a:	4d 85 ed             	test   %r13,%r13
    bd2d:	74 10                	je     bd3f <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJRSt17basic_string_viewIcS3_EEEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_+0x1af>
    bd2f:	49 8b 74 24 10       	mov    0x10(%r12),%rsi
    bd34:	4c 89 ef             	mov    %r13,%rdi
    bd37:	4c 29 ee             	sub    %r13,%rsi
    bd3a:	e8 e1 75 ff ff       	call   3320 <_ZdlPvm@plt>
    bd3f:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
    bd44:	4d 89 74 24 08       	mov    %r14,0x8(%r12)
    bd49:	49 89 04 24          	mov    %rax,(%r12)
    bd4d:	48 03 44 24 18       	add    0x18(%rsp),%rax
    bd52:	49 89 44 24 10       	mov    %rax,0x10(%r12)
    bd57:	48 8b 44 24 38       	mov    0x38(%rsp),%rax
    bd5c:	64 48 2b 04 25 28 00 	sub    %fs:0x28,%rax
    bd63:	00 00 
    bd65:	0f 85 2b 01 00 00    	jne    be96 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJRSt17basic_string_viewIcS3_EEEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_+0x306>
    bd6b:	48 83 c4 48          	add    $0x48,%rsp
    bd6f:	5b                   	pop    %rbx
    bd70:	5d                   	pop    %rbp
    bd71:	41 5c                	pop    %r12
    bd73:	41 5d                	pop    %r13
    bd75:	41 5e                	pop    %r14
    bd77:	41 5f                	pop    %r15
    bd79:	c3                   	ret    
    bd7a:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)
    bd80:	48 39 f0             	cmp    %rsi,%rax
    bd83:	48 0f 46 f0          	cmovbe %rax,%rsi
    bd87:	48 c1 e6 05          	shl    $0x5,%rsi
    bd8b:	48 89 74 24 18       	mov    %rsi,0x18(%rsp)
    bd90:	48 89 f7             	mov    %rsi,%rdi
    bd93:	48 89 4c 24 20       	mov    %rcx,0x20(%rsp)
    bd98:	e8 03 75 ff ff       	call   32a0 <_Znwm@plt>
    bd9d:	48 89 44 24 08       	mov    %rax,0x8(%rsp)
    bda2:	48 8b 4c 24 20       	mov    0x20(%rsp),%rcx
    bda7:	e9 71 fe ff ff       	jmp    bc1d <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJRSt17basic_string_viewIcS3_EEEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_+0x8d>
    bdac:	0f 1f 40 00          	nopl   0x0(%rax)
    bdb0:	c5 fa 6f 4b 10       	vmovdqu 0x10(%rbx),%xmm1
    bdb5:	48 8b 53 08          	mov    0x8(%rbx),%rdx
    bdb9:	48 83 c3 20          	add    $0x20,%rbx
    bdbd:	48 89 50 08          	mov    %rdx,0x8(%rax)
    bdc1:	c5 fa 7f 48 10       	vmovdqu %xmm1,0x10(%rax)
    bdc6:	48 83 c0 20          	add    $0x20,%rax
    bdca:	48 39 dd             	cmp    %rbx,%rbp
    bdcd:	0f 85 1d ff ff ff    	jne    bcf0 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJRSt17basic_string_viewIcS3_EEEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_+0x160>
    bdd3:	e9 4f ff ff ff       	jmp    bd27 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJRSt17basic_string_viewIcS3_EEEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_+0x197>
    bdd8:	0f 1f 84 00 00 00 00 	nopl   0x0(%rax,%rax,1)
    bddf:	00 
    bde0:	c5 fa 6f 42 10       	vmovdqu 0x10(%rdx),%xmm0
    bde5:	48 83 c0 20          	add    $0x20,%rax
    bde9:	c5 fa 7f 40 f0       	vmovdqu %xmm0,-0x10(%rax)
    bdee:	48 8b 4a 08          	mov    0x8(%rdx),%rcx
    bdf2:	48 83 c2 20          	add    $0x20,%rdx
    bdf6:	48 89 48 e8          	mov    %rcx,-0x18(%rax)
    bdfa:	48 39 c7             	cmp    %rax,%rdi
    bdfd:	0f 85 9d fe ff ff    	jne    bca0 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJRSt17basic_string_viewIcS3_EEEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_+0x110>
    be03:	e9 cf fe ff ff       	jmp    bcd7 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJRSt17basic_string_viewIcS3_EEEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_+0x147>
    be08:	0f 1f 84 00 00 00 00 	nopl   0x0(%rax,%rax,1)
    be0f:	00 
    be10:	48 b8 e0 ff ff ff ff 	movabs $0x7fffffffffffffe0,%rax
    be17:	ff ff 7f 
    be1a:	48 89 44 24 18       	mov    %rax,0x18(%rsp)
    be1f:	48 89 c7             	mov    %rax,%rdi
    be22:	e9 6c ff ff ff       	jmp    bd93 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJRSt17basic_string_viewIcS3_EEEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_+0x203>
    be27:	66 0f 1f 84 00 00 00 	nopw   0x0(%rax,%rax,1)
    be2e:	00 00 
    be30:	4d 85 ff             	test   %r15,%r15
    be33:	0f 84 37 fe ff ff    	je     bc70 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJRSt17basic_string_viewIcS3_EEEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_+0xe0>
    be39:	48 8b 7c 24 20       	mov    0x20(%rsp),%rdi
    be3e:	eb 1e                	jmp    be5e <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJRSt17basic_string_viewIcS3_EEEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_+0x2ce>
    be40:	48 8d 74 24 30       	lea    0x30(%rsp),%rsi
    be45:	31 d2                	xor    %edx,%edx
    be47:	4c 89 f7             	mov    %r14,%rdi
    be4a:	e8 b1 74 ff ff       	call   3300 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE9_M_createERmm@plt>
    be4f:	49 89 06             	mov    %rax,(%r14)
    be52:	48 89 c7             	mov    %rax,%rdi
    be55:	48 8b 44 24 30       	mov    0x30(%rsp),%rax
    be5a:	49 89 46 10          	mov    %rax,0x10(%r14)
    be5e:	48 8b 74 24 28       	mov    0x28(%rsp),%rsi
    be63:	4c 89 fa             	mov    %r15,%rdx
    be66:	e8 d5 72 ff ff       	call   3140 <memcpy@plt>
    be6b:	49 8b 06             	mov    (%r14),%rax
    be6e:	4c 8b 7c 24 30       	mov    0x30(%rsp),%r15
    be73:	48 89 44 24 20       	mov    %rax,0x20(%rsp)
    be78:	e9 f3 fd ff ff       	jmp    bc70 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJRSt17basic_string_viewIcS3_EEEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_+0xe0>
    be7d:	0f 1f 00             	nopl   (%rax)
    be80:	48 8b 7c 24 08       	mov    0x8(%rsp),%rdi
    be85:	e9 4d fe ff ff       	jmp    bcd7 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJRSt17basic_string_viewIcS3_EEEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_+0x147>
    be8a:	48 8d 3d 6f 41 00 00 	lea    0x416f(%rip),%rdi        # 10000 <_fini+0xddf>
    be91:	e8 2a 75 ff ff       	call   33c0 <_ZSt19__throw_logic_errorPKc@plt>
    be96:	e8 b5 72 ff ff       	call   3150 <__stack_chk_fail@plt>
    be9b:	48 8d 3d 6e 44 00 00 	lea    0x446e(%rip),%rdi        # 10310 <_fini+0x10ef>
    bea2:	e8 99 74 ff ff       	call   3340 <_ZSt20__throw_length_errorPKc@plt>
    bea7:	48 89 c7             	mov    %rax,%rdi
    beaa:	c5 f8 77             	vzeroupper 
    bead:	e8 4e 73 ff ff       	call   3200 <__cxa_begin_catch@plt>
    beb2:	48 83 7c 24 08 00    	cmpq   $0x0,0x8(%rsp)
    beb8:	74 14                	je     bece <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJRSt17basic_string_viewIcS3_EEEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_+0x33e>
    beba:	48 8b 74 24 18       	mov    0x18(%rsp),%rsi
    bebf:	48 8b 7c 24 08       	mov    0x8(%rsp),%rdi
    bec4:	e8 57 74 ff ff       	call   3320 <_ZdlPvm@plt>
    bec9:	e8 42 73 ff ff       	call   3210 <__cxa_rethrow@plt>
    bece:	49 8b 3e             	mov    (%r14),%rdi
    bed1:	48 39 7c 24 20       	cmp    %rdi,0x20(%rsp)
    bed6:	74 f1                	je     bec9 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJRSt17basic_string_viewIcS3_EEEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_+0x339>
    bed8:	49 8b 76 10          	mov    0x10(%r14),%rsi
    bedc:	48 ff c6             	inc    %rsi
    bedf:	e8 3c 74 ff ff       	call   3320 <_ZdlPvm@plt>
    bee4:	eb e3                	jmp    bec9 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJRSt17basic_string_viewIcS3_EEEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_+0x339>
    bee6:	48 89 c5             	mov    %rax,%rbp
    bee9:	c5 f8 77             	vzeroupper 
    beec:	e8 cf 71 ff ff       	call   30c0 <__cxa_end_catch@plt>
    bef1:	48 89 ef             	mov    %rbp,%rdi
    bef4:	e8 67 74 ff ff       	call   3360 <_Unwind_Resume@plt>
    bef9:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)

000000000000bf00 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJS5_EEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_>:
    bf00:	41 57                	push   %r15
    bf02:	41 56                	push   %r14
    bf04:	41 55                	push   %r13
    bf06:	41 54                	push   %r12
    bf08:	49 bc ff ff ff ff ff 	movabs $0x3ffffffffffffff,%r12
    bf0f:	ff ff 03 
    bf12:	55                   	push   %rbp
    bf13:	53                   	push   %rbx
    bf14:	48 83 ec 18          	sub    $0x18,%rsp
    bf18:	4c 8b 6f 08          	mov    0x8(%rdi),%r13
    bf1c:	4c 8b 3f             	mov    (%rdi),%r15
    bf1f:	4c 89 e8             	mov    %r13,%rax
    bf22:	4c 29 f8             	sub    %r15,%rax
    bf25:	48 c1 f8 05          	sar    $0x5,%rax
    bf29:	4c 39 e0             	cmp    %r12,%rax
    bf2c:	0f 84 1d 02 00 00    	je     c14f <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJS5_EEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_+0x24f>
    bf32:	b9 01 00 00 00       	mov    $0x1,%ecx
    bf37:	48 39 c8             	cmp    %rcx,%rax
    bf3a:	48 0f 43 c8          	cmovae %rax,%rcx
    bf3e:	48 89 f5             	mov    %rsi,%rbp
    bf41:	48 89 f3             	mov    %rsi,%rbx
    bf44:	31 f6                	xor    %esi,%esi
    bf46:	48 01 c8             	add    %rcx,%rax
    bf49:	40 0f 92 c6          	setb   %sil
    bf4d:	48 89 e9             	mov    %rbp,%rcx
    bf50:	49 89 fe             	mov    %rdi,%r14
    bf53:	4c 29 f9             	sub    %r15,%rcx
    bf56:	48 85 f6             	test   %rsi,%rsi
    bf59:	0f 85 d1 01 00 00    	jne    c130 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJS5_EEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_+0x230>
    bf5f:	48 85 c0             	test   %rax,%rax
    bf62:	0f 85 38 01 00 00    	jne    c0a0 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJS5_EEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_+0x1a0>
    bf68:	41 b9 20 00 00 00    	mov    $0x20,%r9d
    bf6e:	45 31 e4             	xor    %r12d,%r12d
    bf71:	45 31 c0             	xor    %r8d,%r8d
    bf74:	49 8d 04 08          	lea    (%r8,%rcx,1),%rax
    bf78:	48 8d 48 10          	lea    0x10(%rax),%rcx
    bf7c:	48 8b 32             	mov    (%rdx),%rsi
    bf7f:	48 89 08             	mov    %rcx,(%rax)
    bf82:	48 8d 4a 10          	lea    0x10(%rdx),%rcx
    bf86:	48 39 ce             	cmp    %rcx,%rsi
    bf89:	0f 84 b1 01 00 00    	je     c140 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJS5_EEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_+0x240>
    bf8f:	48 89 30             	mov    %rsi,(%rax)
    bf92:	48 8b 72 10          	mov    0x10(%rdx),%rsi
    bf96:	48 89 70 10          	mov    %rsi,0x10(%rax)
    bf9a:	48 8b 72 08          	mov    0x8(%rdx),%rsi
    bf9e:	48 89 0a             	mov    %rcx,(%rdx)
    bfa1:	48 89 70 08          	mov    %rsi,0x8(%rax)
    bfa5:	48 c7 42 08 00 00 00 	movq   $0x0,0x8(%rdx)
    bfac:	00 
    bfad:	c6 42 10 00          	movb   $0x0,0x10(%rdx)
    bfb1:	4c 39 fd             	cmp    %r15,%rbp
    bfb4:	74 4e                	je     c004 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJS5_EEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_+0x104>
    bfb6:	48 89 ef             	mov    %rbp,%rdi
    bfb9:	4c 29 ff             	sub    %r15,%rdi
    bfbc:	4c 89 c2             	mov    %r8,%rdx
    bfbf:	4c 89 f8             	mov    %r15,%rax
    bfc2:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)
    bfc8:	48 8d 4a 10          	lea    0x10(%rdx),%rcx
    bfcc:	48 89 0a             	mov    %rcx,(%rdx)
    bfcf:	48 8d 70 10          	lea    0x10(%rax),%rsi
    bfd3:	48 8b 08             	mov    (%rax),%rcx
    bfd6:	48 39 f1             	cmp    %rsi,%rcx
    bfd9:	0f 84 f9 00 00 00    	je     c0d8 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJS5_EEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_+0x1d8>
    bfdf:	48 89 0a             	mov    %rcx,(%rdx)
    bfe2:	48 83 c0 20          	add    $0x20,%rax
    bfe6:	48 83 c2 20          	add    $0x20,%rdx
    bfea:	48 8b 48 f0          	mov    -0x10(%rax),%rcx
    bfee:	48 89 4a f0          	mov    %rcx,-0x10(%rdx)
    bff2:	48 8b 48 e8          	mov    -0x18(%rax),%rcx
    bff6:	48 89 4a e8          	mov    %rcx,-0x18(%rdx)
    bffa:	48 39 e8             	cmp    %rbp,%rax
    bffd:	75 c9                	jne    bfc8 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJS5_EEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_+0xc8>
    bfff:	4d 8d 4c 38 20       	lea    0x20(%r8,%rdi,1),%r9
    c004:	4c 39 ed             	cmp    %r13,%rbp
    c007:	74 49                	je     c052 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJS5_EEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_+0x152>
    c009:	4c 89 ee             	mov    %r13,%rsi
    c00c:	48 29 ee             	sub    %rbp,%rsi
    c00f:	4c 89 c8             	mov    %r9,%rax
    c012:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)
    c018:	48 8d 50 10          	lea    0x10(%rax),%rdx
    c01c:	48 89 10             	mov    %rdx,(%rax)
    c01f:	48 8b 13             	mov    (%rbx),%rdx
    c022:	48 8d 4b 10          	lea    0x10(%rbx),%rcx
    c026:	48 39 ca             	cmp    %rcx,%rdx
    c029:	0f 84 d1 00 00 00    	je     c100 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJS5_EEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_+0x200>
    c02f:	48 89 10             	mov    %rdx,(%rax)
    c032:	48 8b 53 10          	mov    0x10(%rbx),%rdx
    c036:	48 83 c3 20          	add    $0x20,%rbx
    c03a:	48 89 50 10          	mov    %rdx,0x10(%rax)
    c03e:	48 8b 53 e8          	mov    -0x18(%rbx),%rdx
    c042:	48 83 c0 20          	add    $0x20,%rax
    c046:	48 89 50 e8          	mov    %rdx,-0x18(%rax)
    c04a:	4c 39 eb             	cmp    %r13,%rbx
    c04d:	75 c9                	jne    c018 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJS5_EEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_+0x118>
    c04f:	49 01 f1             	add    %rsi,%r9
    c052:	4d 85 ff             	test   %r15,%r15
    c055:	74 21                	je     c078 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJS5_EEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_+0x178>
    c057:	49 8b 76 10          	mov    0x10(%r14),%rsi
    c05b:	4c 89 ff             	mov    %r15,%rdi
    c05e:	4c 29 fe             	sub    %r15,%rsi
    c061:	4c 89 44 24 08       	mov    %r8,0x8(%rsp)
    c066:	4c 89 0c 24          	mov    %r9,(%rsp)
    c06a:	e8 b1 72 ff ff       	call   3320 <_ZdlPvm@plt>
    c06f:	4c 8b 44 24 08       	mov    0x8(%rsp),%r8
    c074:	4c 8b 0c 24          	mov    (%rsp),%r9
    c078:	4d 89 66 10          	mov    %r12,0x10(%r14)
    c07c:	c4 c1 f9 6e c8       	vmovq  %r8,%xmm1
    c081:	c4 c3 f1 22 c1 01    	vpinsrq $0x1,%r9,%xmm1,%xmm0
    c087:	c4 c1 7a 7f 06       	vmovdqu %xmm0,(%r14)
    c08c:	48 83 c4 18          	add    $0x18,%rsp
    c090:	5b                   	pop    %rbx
    c091:	5d                   	pop    %rbp
    c092:	41 5c                	pop    %r12
    c094:	41 5d                	pop    %r13
    c096:	41 5e                	pop    %r14
    c098:	41 5f                	pop    %r15
    c09a:	c3                   	ret    
    c09b:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)
    c0a0:	4c 39 e0             	cmp    %r12,%rax
    c0a3:	4c 0f 46 e0          	cmovbe %rax,%r12
    c0a7:	49 c1 e4 05          	shl    $0x5,%r12
    c0ab:	4c 89 e7             	mov    %r12,%rdi
    c0ae:	48 89 54 24 08       	mov    %rdx,0x8(%rsp)
    c0b3:	48 89 0c 24          	mov    %rcx,(%rsp)
    c0b7:	e8 e4 71 ff ff       	call   32a0 <_Znwm@plt>
    c0bc:	49 89 c0             	mov    %rax,%r8
    c0bf:	48 8b 0c 24          	mov    (%rsp),%rcx
    c0c3:	48 8b 54 24 08       	mov    0x8(%rsp),%rdx
    c0c8:	49 01 c4             	add    %rax,%r12
    c0cb:	4c 8d 48 20          	lea    0x20(%rax),%r9
    c0cf:	e9 a0 fe ff ff       	jmp    bf74 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJS5_EEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_+0x74>
    c0d4:	0f 1f 40 00          	nopl   0x0(%rax)
    c0d8:	c5 fa 6f 50 10       	vmovdqu 0x10(%rax),%xmm2
    c0dd:	48 83 c0 20          	add    $0x20,%rax
    c0e1:	c5 fa 7f 52 10       	vmovdqu %xmm2,0x10(%rdx)
    c0e6:	48 8b 48 e8          	mov    -0x18(%rax),%rcx
    c0ea:	48 83 c2 20          	add    $0x20,%rdx
    c0ee:	48 89 4a e8          	mov    %rcx,-0x18(%rdx)
    c0f2:	48 39 c5             	cmp    %rax,%rbp
    c0f5:	0f 85 cd fe ff ff    	jne    bfc8 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJS5_EEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_+0xc8>
    c0fb:	e9 ff fe ff ff       	jmp    bfff <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJS5_EEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_+0xff>
    c100:	c5 fa 6f 5b 10       	vmovdqu 0x10(%rbx),%xmm3
    c105:	48 8b 53 08          	mov    0x8(%rbx),%rdx
    c109:	48 83 c3 20          	add    $0x20,%rbx
    c10d:	48 89 50 08          	mov    %rdx,0x8(%rax)
    c111:	c5 fa 7f 58 10       	vmovdqu %xmm3,0x10(%rax)
    c116:	48 83 c0 20          	add    $0x20,%rax
    c11a:	49 39 dd             	cmp    %rbx,%r13
    c11d:	0f 85 f5 fe ff ff    	jne    c018 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJS5_EEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_+0x118>
    c123:	e9 27 ff ff ff       	jmp    c04f <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJS5_EEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_+0x14f>
    c128:	0f 1f 84 00 00 00 00 	nopl   0x0(%rax,%rax,1)
    c12f:	00 
    c130:	49 bc e0 ff ff ff ff 	movabs $0x7fffffffffffffe0,%r12
    c137:	ff ff 7f 
    c13a:	e9 6c ff ff ff       	jmp    c0ab <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJS5_EEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_+0x1ab>
    c13f:	90                   	nop
    c140:	c5 fa 6f 62 10       	vmovdqu 0x10(%rdx),%xmm4
    c145:	c5 fa 7f 60 10       	vmovdqu %xmm4,0x10(%rax)
    c14a:	e9 4b fe ff ff       	jmp    bf9a <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJS5_EEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_+0x9a>
    c14f:	48 8d 3d ba 41 00 00 	lea    0x41ba(%rip),%rdi        # 10310 <_fini+0x10ef>
    c156:	e8 e5 71 ff ff       	call   3340 <_ZSt20__throw_length_errorPKc@plt>
    c15b:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)

000000000000c160 <_ZN8argparse14ArgumentParser10parse_argsEiPKPKc>:
    c160:	41 57                	push   %r15
    c162:	41 56                	push   %r14
    c164:	41 55                	push   %r13
    c166:	41 54                	push   %r12
    c168:	49 89 d4             	mov    %rdx,%r12
    c16b:	55                   	push   %rbp
    c16c:	53                   	push   %rbx
    c16d:	48 63 de             	movslq %esi,%rbx
    c170:	48 81 ec 88 00 00 00 	sub    $0x88,%rsp
    c177:	48 89 7c 24 18       	mov    %rdi,0x18(%rsp)
    c17c:	48 8d 4c 24 30       	lea    0x30(%rsp),%rcx
    c181:	4c 8d 7c 24 50       	lea    0x50(%rsp),%r15
    c186:	64 48 8b 04 25 28 00 	mov    %fs:0x28,%rax
    c18d:	00 00 
    c18f:	48 89 44 24 78       	mov    %rax,0x78(%rsp)
    c194:	31 c0                	xor    %eax,%eax
    c196:	48 89 4c 24 10       	mov    %rcx,0x10(%rsp)
    c19b:	48 8d 04 dd 00 00 00 	lea    0x0(,%rbx,8),%rax
    c1a2:	00 
    c1a3:	48 8d 4c 24 28       	lea    0x28(%rsp),%rcx
    c1a8:	48 c7 44 24 30 00 00 	movq   $0x0,0x30(%rsp)
    c1af:	00 00 
    c1b1:	48 c7 44 24 38 00 00 	movq   $0x0,0x38(%rsp)
    c1b8:	00 00 
    c1ba:	48 c7 44 24 40 00 00 	movq   $0x0,0x40(%rsp)
    c1c1:	00 00 
    c1c3:	48 89 4c 24 08       	mov    %rcx,0x8(%rsp)
    c1c8:	4c 8d 6c 24 60       	lea    0x60(%rsp),%r13
    c1cd:	48 85 c0             	test   %rax,%rax
    c1d0:	7f 6f                	jg     c241 <_ZN8argparse14ArgumentParser10parse_argsEiPKPKc+0xe1>
    c1d2:	e9 3f 01 00 00       	jmp    c316 <_ZN8argparse14ArgumentParser10parse_argsEiPKPKc+0x1b6>
    c1d7:	66 0f 1f 84 00 00 00 	nopw   0x0(%rax,%rax,1)
    c1de:	00 00 
    c1e0:	0f b6 55 00          	movzbl 0x0(%rbp),%edx
    c1e4:	88 54 24 60          	mov    %dl,0x60(%rsp)
    c1e8:	4c 89 ea             	mov    %r13,%rdx
    c1eb:	48 89 44 24 58       	mov    %rax,0x58(%rsp)
    c1f0:	c6 04 02 00          	movb   $0x0,(%rdx,%rax,1)
    c1f4:	48 8b 74 24 38       	mov    0x38(%rsp),%rsi
    c1f9:	48 3b 74 24 40       	cmp    0x40(%rsp),%rsi
    c1fe:	0f 84 dc 00 00 00    	je     c2e0 <_ZN8argparse14ArgumentParser10parse_argsEiPKPKc+0x180>
    c204:	48 8d 46 10          	lea    0x10(%rsi),%rax
    c208:	48 89 06             	mov    %rax,(%rsi)
    c20b:	48 8b 44 24 50       	mov    0x50(%rsp),%rax
    c210:	4c 39 e8             	cmp    %r13,%rax
    c213:	0f 84 af 00 00 00    	je     c2c8 <_ZN8argparse14ArgumentParser10parse_argsEiPKPKc+0x168>
    c219:	48 89 06             	mov    %rax,(%rsi)
    c21c:	48 8b 44 24 60       	mov    0x60(%rsp),%rax
    c221:	48 89 46 10          	mov    %rax,0x10(%rsi)
    c225:	48 8b 44 24 58       	mov    0x58(%rsp),%rax
    c22a:	48 89 46 08          	mov    %rax,0x8(%rsi)
    c22e:	48 83 44 24 38 20    	addq   $0x20,0x38(%rsp)
    c234:	49 83 c4 08          	add    $0x8,%r12
    c238:	48 ff cb             	dec    %rbx
    c23b:	0f 84 d5 00 00 00    	je     c316 <_ZN8argparse14ArgumentParser10parse_argsEiPKPKc+0x1b6>
    c241:	49 8b 2c 24          	mov    (%r12),%rbp
    c245:	4c 89 6c 24 50       	mov    %r13,0x50(%rsp)
    c24a:	48 85 ed             	test   %rbp,%rbp
    c24d:	0f 84 1e 02 00 00    	je     c471 <_ZN8argparse14ArgumentParser10parse_argsEiPKPKc+0x311>
    c253:	48 89 ef             	mov    %rbp,%rdi
    c256:	e8 d5 70 ff ff       	call   3330 <strlen@plt>
    c25b:	48 89 44 24 28       	mov    %rax,0x28(%rsp)
    c260:	49 89 c6             	mov    %rax,%r14
    c263:	48 83 f8 0f          	cmp    $0xf,%rax
    c267:	77 1f                	ja     c288 <_ZN8argparse14ArgumentParser10parse_argsEiPKPKc+0x128>
    c269:	48 83 f8 01          	cmp    $0x1,%rax
    c26d:	0f 84 6d ff ff ff    	je     c1e0 <_ZN8argparse14ArgumentParser10parse_argsEiPKPKc+0x80>
    c273:	48 85 c0             	test   %rax,%rax
    c276:	0f 85 06 02 00 00    	jne    c482 <_ZN8argparse14ArgumentParser10parse_argsEiPKPKc+0x322>
    c27c:	4c 89 ea             	mov    %r13,%rdx
    c27f:	e9 67 ff ff ff       	jmp    c1eb <_ZN8argparse14ArgumentParser10parse_argsEiPKPKc+0x8b>
    c284:	0f 1f 40 00          	nopl   0x0(%rax)
    c288:	48 8b 74 24 08       	mov    0x8(%rsp),%rsi
    c28d:	31 d2                	xor    %edx,%edx
    c28f:	4c 89 ff             	mov    %r15,%rdi
    c292:	e8 69 70 ff ff       	call   3300 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE9_M_createERmm@plt>
    c297:	48 89 44 24 50       	mov    %rax,0x50(%rsp)
    c29c:	48 89 c7             	mov    %rax,%rdi
    c29f:	48 8b 44 24 28       	mov    0x28(%rsp),%rax
    c2a4:	48 89 44 24 60       	mov    %rax,0x60(%rsp)
    c2a9:	4c 89 f2             	mov    %r14,%rdx
    c2ac:	48 89 ee             	mov    %rbp,%rsi
    c2af:	e8 8c 6e ff ff       	call   3140 <memcpy@plt>
    c2b4:	48 8b 44 24 28       	mov    0x28(%rsp),%rax
    c2b9:	48 8b 54 24 50       	mov    0x50(%rsp),%rdx
    c2be:	e9 28 ff ff ff       	jmp    c1eb <_ZN8argparse14ArgumentParser10parse_argsEiPKPKc+0x8b>
    c2c3:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)
    c2c8:	c5 f9 6f 44 24 60    	vmovdqa 0x60(%rsp),%xmm0
    c2ce:	c5 fa 7f 46 10       	vmovdqu %xmm0,0x10(%rsi)
    c2d3:	e9 4d ff ff ff       	jmp    c225 <_ZN8argparse14ArgumentParser10parse_argsEiPKPKc+0xc5>
    c2d8:	0f 1f 84 00 00 00 00 	nopl   0x0(%rax,%rax,1)
    c2df:	00 
    c2e0:	48 8b 7c 24 10       	mov    0x10(%rsp),%rdi
    c2e5:	4c 89 fa             	mov    %r15,%rdx
    c2e8:	e8 13 fc ff ff       	call   bf00 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJS5_EEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_>
    c2ed:	48 8b 7c 24 50       	mov    0x50(%rsp),%rdi
    c2f2:	4c 39 ef             	cmp    %r13,%rdi
    c2f5:	0f 84 39 ff ff ff    	je     c234 <_ZN8argparse14ArgumentParser10parse_argsEiPKPKc+0xd4>
    c2fb:	48 8b 44 24 60       	mov    0x60(%rsp),%rax
    c300:	49 83 c4 08          	add    $0x8,%r12
    c304:	48 8d 70 01          	lea    0x1(%rax),%rsi
    c308:	e8 13 70 ff ff       	call   3320 <_ZdlPvm@plt>
    c30d:	48 ff cb             	dec    %rbx
    c310:	0f 85 2b ff ff ff    	jne    c241 <_ZN8argparse14ArgumentParser10parse_argsEiPKPKc+0xe1>
    c316:	48 8b 74 24 10       	mov    0x10(%rsp),%rsi
    c31b:	48 8b 7c 24 18       	mov    0x18(%rsp),%rdi
    c320:	e8 1b df ff ff       	call   a240 <_ZN8argparse14ArgumentParser19parse_args_internalERKSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS7_EE>
    c325:	48 8b 44 24 18       	mov    0x18(%rsp),%rax
    c32a:	48 8b b8 d0 00 00 00 	mov    0xd0(%rax),%rdi
    c331:	48 8d 98 c0 00 00 00 	lea    0xc0(%rax),%rbx
    c338:	48 39 df             	cmp    %rbx,%rdi
    c33b:	75 30                	jne    c36d <_ZN8argparse14ArgumentParser10parse_argsEiPKPKc+0x20d>
    c33d:	e9 96 00 00 00       	jmp    c3d8 <_ZN8argparse14ArgumentParser10parse_argsEiPKPKc+0x278>
    c342:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)
    c348:	f6 c2 08             	test   $0x8,%dl
    c34b:	75 6b                	jne    c3b8 <_ZN8argparse14ArgumentParser10parse_argsEiPKPKc+0x258>
    c34d:	48 83 78 58 00       	cmpq   $0x0,0x58(%rax)
    c352:	0f 84 08 01 00 00    	je     c460 <_ZN8argparse14ArgumentParser10parse_argsEiPKPKc+0x300>
    c358:	0f 1f 84 00 00 00 00 	nopl   0x0(%rax,%rax,1)
    c35f:	00 
    c360:	e8 4b 6d ff ff       	call   30b0 <_ZSt18_Rb_tree_incrementPSt18_Rb_tree_node_base@plt>
    c365:	48 89 c7             	mov    %rax,%rdi
    c368:	48 39 c3             	cmp    %rax,%rbx
    c36b:	74 6b                	je     c3d8 <_ZN8argparse14ArgumentParser10parse_argsEiPKPKc+0x278>
    c36d:	48 8b 47 30          	mov    0x30(%rdi),%rax
    c371:	0f b6 90 e9 00 00 00 	movzbl 0xe9(%rax),%edx
    c378:	4c 8d 40 10          	lea    0x10(%rax),%r8
    c37c:	f6 c2 01             	test   $0x1,%dl
    c37f:	75 c7                	jne    c348 <_ZN8argparse14ArgumentParser10parse_argsEiPKPKc+0x1e8>
    c381:	48 8b 90 c8 00 00 00 	mov    0xc8(%rax),%rdx
    c388:	48 2b 90 c0 00 00 00 	sub    0xc0(%rax),%rdx
    c38f:	48 c1 fa 04          	sar    $0x4,%rdx
    c393:	48 3b 90 d8 00 00 00 	cmp    0xd8(%rax),%rdx
    c39a:	72 09                	jb     c3a5 <_ZN8argparse14ArgumentParser10parse_argsEiPKPKc+0x245>
    c39c:	48 3b 90 e0 00 00 00 	cmp    0xe0(%rax),%rdx
    c3a3:	76 bb                	jbe    c360 <_ZN8argparse14ArgumentParser10parse_argsEiPKPKc+0x200>
    c3a5:	48 83 78 58 00       	cmpq   $0x0,0x58(%rax)
    c3aa:	75 b4                	jne    c360 <_ZN8argparse14ArgumentParser10parse_argsEiPKPKc+0x200>
    c3ac:	4c 89 c7             	mov    %r8,%rdi
    c3af:	e8 3c c2 ff ff       	call   85f0 <_ZNK8argparse8Argument34throw_nargs_range_validation_errorEv>
    c3b4:	0f 1f 40 00          	nopl   0x0(%rax)
    c3b8:	83 e2 02             	and    $0x2,%edx
    c3bb:	74 a3                	je     c360 <_ZN8argparse14ArgumentParser10parse_argsEiPKPKc+0x200>
    c3bd:	48 8b 88 c0 00 00 00 	mov    0xc0(%rax),%rcx
    c3c4:	48 39 88 c8 00 00 00 	cmp    %rcx,0xc8(%rax)
    c3cb:	75 93                	jne    c360 <_ZN8argparse14ArgumentParser10parse_argsEiPKPKc+0x200>
    c3cd:	4c 89 c7             	mov    %r8,%rdi
    c3d0:	e8 1b c1 ff ff       	call   84f0 <_ZNK8argparse8Argument42throw_required_arg_no_value_provided_errorEv>
    c3d5:	0f 1f 00             	nopl   (%rax)
    c3d8:	48 8b 5c 24 38       	mov    0x38(%rsp),%rbx
    c3dd:	48 8b 6c 24 30       	mov    0x30(%rsp),%rbp
    c3e2:	48 39 eb             	cmp    %rbp,%rbx
    c3e5:	74 31                	je     c418 <_ZN8argparse14ArgumentParser10parse_argsEiPKPKc+0x2b8>
    c3e7:	66 0f 1f 84 00 00 00 	nopw   0x0(%rax,%rax,1)
    c3ee:	00 00 
    c3f0:	48 8b 7d 00          	mov    0x0(%rbp),%rdi
    c3f4:	48 8d 45 10          	lea    0x10(%rbp),%rax
    c3f8:	48 39 c7             	cmp    %rax,%rdi
    c3fb:	74 53                	je     c450 <_ZN8argparse14ArgumentParser10parse_argsEiPKPKc+0x2f0>
    c3fd:	48 8b 45 10          	mov    0x10(%rbp),%rax
    c401:	48 83 c5 20          	add    $0x20,%rbp
    c405:	48 8d 70 01          	lea    0x1(%rax),%rsi
    c409:	e8 12 6f ff ff       	call   3320 <_ZdlPvm@plt>
    c40e:	48 39 dd             	cmp    %rbx,%rbp
    c411:	75 dd                	jne    c3f0 <_ZN8argparse14ArgumentParser10parse_argsEiPKPKc+0x290>
    c413:	48 8b 6c 24 30       	mov    0x30(%rsp),%rbp
    c418:	48 85 ed             	test   %rbp,%rbp
    c41b:	74 10                	je     c42d <_ZN8argparse14ArgumentParser10parse_argsEiPKPKc+0x2cd>
    c41d:	48 8b 74 24 40       	mov    0x40(%rsp),%rsi
    c422:	48 89 ef             	mov    %rbp,%rdi
    c425:	48 29 ee             	sub    %rbp,%rsi
    c428:	e8 f3 6e ff ff       	call   3320 <_ZdlPvm@plt>
    c42d:	48 8b 44 24 78       	mov    0x78(%rsp),%rax
    c432:	64 48 2b 04 25 28 00 	sub    %fs:0x28,%rax
    c439:	00 00 
    c43b:	75 40                	jne    c47d <_ZN8argparse14ArgumentParser10parse_argsEiPKPKc+0x31d>
    c43d:	48 81 c4 88 00 00 00 	add    $0x88,%rsp
    c444:	5b                   	pop    %rbx
    c445:	5d                   	pop    %rbp
    c446:	41 5c                	pop    %r12
    c448:	41 5d                	pop    %r13
    c44a:	41 5e                	pop    %r14
    c44c:	41 5f                	pop    %r15
    c44e:	c3                   	ret    
    c44f:	90                   	nop
    c450:	48 83 c5 20          	add    $0x20,%rbp
    c454:	48 39 eb             	cmp    %rbp,%rbx
    c457:	75 97                	jne    c3f0 <_ZN8argparse14ArgumentParser10parse_argsEiPKPKc+0x290>
    c459:	eb b8                	jmp    c413 <_ZN8argparse14ArgumentParser10parse_argsEiPKPKc+0x2b3>
    c45b:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)
    c460:	83 e2 02             	and    $0x2,%edx
    c463:	0f 84 f7 fe ff ff    	je     c360 <_ZN8argparse14ArgumentParser10parse_argsEiPKPKc+0x200>
    c469:	4c 89 c7             	mov    %r8,%rdi
    c46c:	e8 7f bf ff ff       	call   83f0 <_ZNK8argparse8Argument33throw_required_arg_not_used_errorEv>
    c471:	48 8d 3d 88 3b 00 00 	lea    0x3b88(%rip),%rdi        # 10000 <_fini+0xddf>
    c478:	e8 43 6f ff ff       	call   33c0 <_ZSt19__throw_logic_errorPKc@plt>
    c47d:	e8 ce 6c ff ff       	call   3150 <__stack_chk_fail@plt>
    c482:	4c 89 ef             	mov    %r13,%rdi
    c485:	e9 1f fe ff ff       	jmp    c2a9 <_ZN8argparse14ArgumentParser10parse_argsEiPKPKc+0x149>
    c48a:	48 89 c5             	mov    %rax,%rbp
    c48d:	eb 1e                	jmp    c4ad <_ZN8argparse14ArgumentParser10parse_argsEiPKPKc+0x34d>
    c48f:	48 89 c5             	mov    %rax,%rbp
    c492:	48 8b 7c 24 50       	mov    0x50(%rsp),%rdi
    c497:	4c 39 ef             	cmp    %r13,%rdi
    c49a:	74 11                	je     c4ad <_ZN8argparse14ArgumentParser10parse_argsEiPKPKc+0x34d>
    c49c:	48 8b 44 24 60       	mov    0x60(%rsp),%rax
    c4a1:	48 8d 70 01          	lea    0x1(%rax),%rsi
    c4a5:	c5 f8 77             	vzeroupper 
    c4a8:	e8 73 6e ff ff       	call   3320 <_ZdlPvm@plt>
    c4ad:	48 8b 7c 24 10       	mov    0x10(%rsp),%rdi
    c4b2:	c5 f8 77             	vzeroupper 
    c4b5:	e8 56 c5 ff ff       	call   8a10 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EED1Ev>
    c4ba:	48 89 ef             	mov    %rbp,%rdi
    c4bd:	e8 9e 6e ff ff       	call   3360 <_Unwind_Resume@plt>
    c4c2:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
    c4c9:	00 00 00 
    c4cc:	0f 1f 40 00          	nopl   0x0(%rax)

000000000000c4d0 <_ZSt11__make_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_RSS_>:
    c4d0:	41 57                	push   %r15
    c4d2:	48 29 fe             	sub    %rdi,%rsi
    c4d5:	41 56                	push   %r14
    c4d7:	41 55                	push   %r13
    c4d9:	41 54                	push   %r12
    c4db:	55                   	push   %rbp
    c4dc:	53                   	push   %rbx
    c4dd:	48 83 ec 68          	sub    $0x68,%rsp
    c4e1:	64 48 8b 04 25 28 00 	mov    %fs:0x28,%rax
    c4e8:	00 00 
    c4ea:	48 89 44 24 58       	mov    %rax,0x58(%rsp)
    c4ef:	31 c0                	xor    %eax,%eax
    c4f1:	48 83 fe 20          	cmp    $0x20,%rsi
    c4f5:	0f 8e 40 01 00 00    	jle    c63b <_ZSt11__make_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_RSS_+0x16b>
    c4fb:	49 89 f4             	mov    %rsi,%r12
    c4fe:	49 c1 fc 05          	sar    $0x5,%r12
    c502:	49 8d 44 24 fe       	lea    -0x2(%r12),%rax
    c507:	48 89 c3             	mov    %rax,%rbx
    c50a:	48 c1 eb 3f          	shr    $0x3f,%rbx
    c50e:	48 01 c3             	add    %rax,%rbx
    c511:	48 d1 fb             	sar    %rbx
    c514:	48 89 d8             	mov    %rbx,%rax
    c517:	48 c1 e0 05          	shl    $0x5,%rax
    c51b:	4c 8d 7c 07 10       	lea    0x10(%rdi,%rax,1),%r15
    c520:	48 8d 44 24 30       	lea    0x30(%rsp),%rax
    c525:	48 89 44 24 08       	mov    %rax,0x8(%rsp)
    c52a:	49 89 fe             	mov    %rdi,%r14
    c52d:	48 8d 6c 24 20       	lea    0x20(%rsp),%rbp
    c532:	4c 8d 6c 24 40       	lea    0x40(%rsp),%r13
    c537:	e9 a4 00 00 00       	jmp    c5e0 <_ZSt11__make_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_RSS_+0x110>
    c53c:	0f 1f 40 00          	nopl   0x0(%rax)
    c540:	49 8b 0f             	mov    (%r15),%rcx
    c543:	49 8b 77 f8          	mov    -0x8(%r15),%rsi
    c547:	48 89 54 24 10       	mov    %rdx,0x10(%rsp)
    c54c:	48 89 4c 24 20       	mov    %rcx,0x20(%rsp)
    c551:	4d 89 7f f0          	mov    %r15,-0x10(%r15)
    c555:	49 c7 47 f8 00 00 00 	movq   $0x0,-0x8(%r15)
    c55c:	00 
    c55d:	41 c6 07 00          	movb   $0x0,(%r15)
    c561:	4c 89 6c 24 30       	mov    %r13,0x30(%rsp)
    c566:	48 39 ea             	cmp    %rbp,%rdx
    c569:	0f 84 a3 00 00 00    	je     c612 <_ZSt11__make_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_RSS_+0x142>
    c56f:	48 89 54 24 30       	mov    %rdx,0x30(%rsp)
    c574:	48 89 4c 24 40       	mov    %rcx,0x40(%rsp)
    c579:	48 8b 4c 24 08       	mov    0x8(%rsp),%rcx
    c57e:	48 89 74 24 38       	mov    %rsi,0x38(%rsp)
    c583:	4c 89 f7             	mov    %r14,%rdi
    c586:	4c 89 e2             	mov    %r12,%rdx
    c589:	48 89 de             	mov    %rbx,%rsi
    c58c:	48 89 6c 24 10       	mov    %rbp,0x10(%rsp)
    c591:	48 c7 44 24 18 00 00 	movq   $0x0,0x18(%rsp)
    c598:	00 00 
    c59a:	c6 44 24 20 00       	movb   $0x0,0x20(%rsp)
    c59f:	e8 8c 8a ff ff       	call   5030 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0>
    c5a4:	48 8b 7c 24 30       	mov    0x30(%rsp),%rdi
    c5a9:	4c 39 ef             	cmp    %r13,%rdi
    c5ac:	74 0e                	je     c5bc <_ZSt11__make_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_RSS_+0xec>
    c5ae:	48 8b 44 24 40       	mov    0x40(%rsp),%rax
    c5b3:	48 8d 70 01          	lea    0x1(%rax),%rsi
    c5b7:	e8 64 6d ff ff       	call   3320 <_ZdlPvm@plt>
    c5bc:	48 8b 7c 24 10       	mov    0x10(%rsp),%rdi
    c5c1:	48 85 db             	test   %rbx,%rbx
    c5c4:	74 62                	je     c628 <_ZSt11__make_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_RSS_+0x158>
    c5c6:	48 ff cb             	dec    %rbx
    c5c9:	48 39 ef             	cmp    %rbp,%rdi
    c5cc:	74 0e                	je     c5dc <_ZSt11__make_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_RSS_+0x10c>
    c5ce:	48 8b 44 24 20       	mov    0x20(%rsp),%rax
    c5d3:	48 8d 70 01          	lea    0x1(%rax),%rsi
    c5d7:	e8 44 6d ff ff       	call   3320 <_ZdlPvm@plt>
    c5dc:	49 83 ef 20          	sub    $0x20,%r15
    c5e0:	49 8b 57 f0          	mov    -0x10(%r15),%rdx
    c5e4:	48 89 6c 24 10       	mov    %rbp,0x10(%rsp)
    c5e9:	49 39 d7             	cmp    %rdx,%r15
    c5ec:	0f 85 4e ff ff ff    	jne    c540 <_ZSt11__make_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_RSS_+0x70>
    c5f2:	c4 c1 7a 6f 07       	vmovdqu (%r15),%xmm0
    c5f7:	49 8b 77 f8          	mov    -0x8(%r15),%rsi
    c5fb:	41 c6 07 00          	movb   $0x0,(%r15)
    c5ff:	49 c7 47 f8 00 00 00 	movq   $0x0,-0x8(%r15)
    c606:	00 
    c607:	4c 89 6c 24 30       	mov    %r13,0x30(%rsp)
    c60c:	c5 f9 7f 44 24 20    	vmovdqa %xmm0,0x20(%rsp)
    c612:	c5 f9 6f 4c 24 20    	vmovdqa 0x20(%rsp),%xmm1
    c618:	c5 f9 7f 4c 24 40    	vmovdqa %xmm1,0x40(%rsp)
    c61e:	e9 56 ff ff ff       	jmp    c579 <_ZSt11__make_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_RSS_+0xa9>
    c623:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)
    c628:	48 39 ef             	cmp    %rbp,%rdi
    c62b:	74 0e                	je     c63b <_ZSt11__make_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_RSS_+0x16b>
    c62d:	48 8b 44 24 20       	mov    0x20(%rsp),%rax
    c632:	48 8d 70 01          	lea    0x1(%rax),%rsi
    c636:	e8 e5 6c ff ff       	call   3320 <_ZdlPvm@plt>
    c63b:	48 8b 44 24 58       	mov    0x58(%rsp),%rax
    c640:	64 48 2b 04 25 28 00 	sub    %fs:0x28,%rax
    c647:	00 00 
    c649:	75 0f                	jne    c65a <_ZSt11__make_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_RSS_+0x18a>
    c64b:	48 83 c4 68          	add    $0x68,%rsp
    c64f:	5b                   	pop    %rbx
    c650:	5d                   	pop    %rbp
    c651:	41 5c                	pop    %r12
    c653:	41 5d                	pop    %r13
    c655:	41 5e                	pop    %r14
    c657:	41 5f                	pop    %r15
    c659:	c3                   	ret    
    c65a:	e8 f1 6a ff ff       	call   3150 <__stack_chk_fail@plt>
    c65f:	90                   	nop

000000000000c660 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_>:
    c660:	41 57                	push   %r15
    c662:	41 56                	push   %r14
    c664:	41 55                	push   %r13
    c666:	41 54                	push   %r12
    c668:	55                   	push   %rbp
    c669:	53                   	push   %rbx
    c66a:	48 81 ec 88 00 00 00 	sub    $0x88,%rsp
    c671:	48 89 54 24 08       	mov    %rdx,0x8(%rsp)
    c676:	64 48 8b 04 25 28 00 	mov    %fs:0x28,%rax
    c67d:	00 00 
    c67f:	48 89 44 24 78       	mov    %rax,0x78(%rsp)
    c684:	48 89 f0             	mov    %rsi,%rax
    c687:	48 89 34 24          	mov    %rsi,(%rsp)
    c68b:	48 29 f8             	sub    %rdi,%rax
    c68e:	48 3d 00 02 00 00    	cmp    $0x200,%rax
    c694:	0f 8e 37 03 00 00    	jle    c9d1 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x371>
    c69a:	48 89 fd             	mov    %rdi,%rbp
    c69d:	49 89 f4             	mov    %rsi,%r12
    c6a0:	48 85 d2             	test   %rdx,%rdx
    c6a3:	0f 84 d8 01 00 00    	je     c881 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x221>
    c6a9:	48 8d 47 20          	lea    0x20(%rdi),%rax
    c6ad:	48 89 44 24 10       	mov    %rax,0x10(%rsp)
    c6b2:	48 8b 04 24          	mov    (%rsp),%rax
    c6b6:	48 29 e8             	sub    %rbp,%rax
    c6b9:	49 89 c4             	mov    %rax,%r12
    c6bc:	49 c1 fc 05          	sar    $0x5,%r12
    c6c0:	48 c1 e8 3f          	shr    $0x3f,%rax
    c6c4:	49 01 c4             	add    %rax,%r12
    c6c7:	49 d1 fc             	sar    %r12
    c6ca:	49 c1 e4 05          	shl    $0x5,%r12
    c6ce:	49 01 ec             	add    %rbp,%r12
    c6d1:	48 8b 5d 28          	mov    0x28(%rbp),%rbx
    c6d5:	4d 8b 74 24 08       	mov    0x8(%r12),%r14
    c6da:	48 ff 4c 24 08       	decq   0x8(%rsp)
    c6df:	4c 39 f3             	cmp    %r14,%rbx
    c6e2:	0f 84 37 01 00 00    	je     c81f <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x1bf>
    c6e8:	48 8b 04 24          	mov    (%rsp),%rax
    c6ec:	4c 8b 78 e8          	mov    -0x18(%rax),%r15
    c6f0:	4c 8d 68 e0          	lea    -0x20(%rax),%r13
    c6f4:	0f 82 57 01 00 00    	jb     c851 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x1f1>
    c6fa:	4c 39 fb             	cmp    %r15,%rbx
    c6fd:	0f 84 0b 04 00 00    	je     cb0e <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x4ae>
    c703:	4c 39 fb             	cmp    %r15,%rbx
    c706:	0f 82 63 01 00 00    	jb     c86f <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x20f>
    c70c:	4d 39 fe             	cmp    %r15,%r14
    c70f:	74 7f                	je     c790 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x130>
    c711:	0f 83 9a 00 00 00    	jae    c7b1 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x151>
    c717:	4c 89 ee             	mov    %r13,%rsi
    c71a:	48 89 ef             	mov    %rbp,%rdi
    c71d:	e8 5e 6b ff ff       	call   3280 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE4swapERS4_@plt>
    c722:	4c 8b 6d 08          	mov    0x8(%rbp),%r13
    c726:	4c 8b 7c 24 10       	mov    0x10(%rsp),%r15
    c72b:	4c 8b 34 24          	mov    (%rsp),%r14
    c72f:	eb 0d                	jmp    c73e <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0xde>
    c731:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
    c738:	73 25                	jae    c75f <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0xff>
    c73a:	49 83 c7 20          	add    $0x20,%r15
    c73e:	4d 89 fc             	mov    %r15,%r12
    c741:	4d 39 6f 08          	cmp    %r13,0x8(%r15)
    c745:	75 f1                	jne    c738 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0xd8>
    c747:	4d 85 ed             	test   %r13,%r13
    c74a:	74 13                	je     c75f <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0xff>
    c74c:	48 8b 75 00          	mov    0x0(%rbp),%rsi
    c750:	49 8b 3f             	mov    (%r15),%rdi
    c753:	4c 89 ea             	mov    %r13,%rdx
    c756:	e8 15 6b ff ff       	call   3270 <memcmp@plt>
    c75b:	85 c0                	test   %eax,%eax
    c75d:	78 db                	js     c73a <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0xda>
    c75f:	49 8d 5e e0          	lea    -0x20(%r14),%rbx
    c763:	48 8b 73 08          	mov    0x8(%rbx),%rsi
    c767:	49 89 de             	mov    %rbx,%r14
    c76a:	4c 39 ee             	cmp    %r13,%rsi
    c76d:	0f 97 c0             	seta   %al
    c770:	74 56                	je     c7c8 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x168>
    c772:	48 83 eb 20          	sub    $0x20,%rbx
    c776:	84 c0                	test   %al,%al
    c778:	75 e9                	jne    c763 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x103>
    c77a:	4d 39 f7             	cmp    %r14,%r15
    c77d:	73 69                	jae    c7e8 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x188>
    c77f:	4c 89 f6             	mov    %r14,%rsi
    c782:	4c 89 ff             	mov    %r15,%rdi
    c785:	e8 f6 6a ff ff       	call   3280 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE4swapERS4_@plt>
    c78a:	4c 8b 6d 08          	mov    0x8(%rbp),%r13
    c78e:	eb aa                	jmp    c73a <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0xda>
    c790:	4d 85 f6             	test   %r14,%r14
    c793:	74 1c                	je     c7b1 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x151>
    c795:	48 8b 04 24          	mov    (%rsp),%rax
    c799:	49 8b 3c 24          	mov    (%r12),%rdi
    c79d:	48 8b 70 e0          	mov    -0x20(%rax),%rsi
    c7a1:	4c 89 f2             	mov    %r14,%rdx
    c7a4:	e8 c7 6a ff ff       	call   3270 <memcmp@plt>
    c7a9:	85 c0                	test   %eax,%eax
    c7ab:	0f 85 6f 03 00 00    	jne    cb20 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x4c0>
    c7b1:	4c 89 e6             	mov    %r12,%rsi
    c7b4:	48 89 ef             	mov    %rbp,%rdi
    c7b7:	e8 c4 6a ff ff       	call   3280 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE4swapERS4_@plt>
    c7bc:	e9 61 ff ff ff       	jmp    c722 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0xc2>
    c7c1:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
    c7c8:	4d 85 ed             	test   %r13,%r13
    c7cb:	74 ad                	je     c77a <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x11a>
    c7cd:	48 8b 33             	mov    (%rbx),%rsi
    c7d0:	48 8b 7d 00          	mov    0x0(%rbp),%rdi
    c7d4:	4c 89 ea             	mov    %r13,%rdx
    c7d7:	e8 94 6a ff ff       	call   3270 <memcmp@plt>
    c7dc:	85 c0                	test   %eax,%eax
    c7de:	74 9a                	je     c77a <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x11a>
    c7e0:	c1 e8 1f             	shr    $0x1f,%eax
    c7e3:	eb 8d                	jmp    c772 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x112>
    c7e5:	0f 1f 00             	nopl   (%rax)
    c7e8:	48 8b 54 24 08       	mov    0x8(%rsp),%rdx
    c7ed:	48 8b 34 24          	mov    (%rsp),%rsi
    c7f1:	4c 89 ff             	mov    %r15,%rdi
    c7f4:	e8 67 fe ff ff       	call   c660 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_>
    c7f9:	4c 89 f8             	mov    %r15,%rax
    c7fc:	48 29 e8             	sub    %rbp,%rax
    c7ff:	48 3d 00 02 00 00    	cmp    $0x200,%rax
    c805:	0f 8e c6 01 00 00    	jle    c9d1 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x371>
    c80b:	48 83 7c 24 08 00    	cmpq   $0x0,0x8(%rsp)
    c811:	74 6e                	je     c881 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x221>
    c813:	4c 89 3c 24          	mov    %r15,(%rsp)
    c817:	4c 89 f8             	mov    %r15,%rax
    c81a:	e9 97 fe ff ff       	jmp    c6b6 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x56>
    c81f:	48 85 db             	test   %rbx,%rbx
    c822:	0f 84 60 02 00 00    	je     ca88 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x428>
    c828:	48 8b 7d 20          	mov    0x20(%rbp),%rdi
    c82c:	49 8b 34 24          	mov    (%r12),%rsi
    c830:	48 89 da             	mov    %rbx,%rdx
    c833:	48 89 7c 24 18       	mov    %rdi,0x18(%rsp)
    c838:	e8 33 6a ff ff       	call   3270 <memcmp@plt>
    c83d:	85 c0                	test   %eax,%eax
    c83f:	48 8b 04 24          	mov    (%rsp),%rax
    c843:	4c 8b 78 e8          	mov    -0x18(%rax),%r15
    c847:	4c 8d 68 e0          	lea    -0x20(%rax),%r13
    c84b:	0f 89 0a 02 00 00    	jns    ca5b <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x3fb>
    c851:	4d 39 f7             	cmp    %r14,%r15
    c854:	0f 84 9d 01 00 00    	je     c9f7 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x397>
    c85a:	0f 87 51 ff ff ff    	ja     c7b1 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x151>
    c860:	49 39 df             	cmp    %rbx,%r15
    c863:	0f 84 c2 01 00 00    	je     ca2b <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x3cb>
    c869:	0f 87 a8 fe ff ff    	ja     c717 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0xb7>
    c86f:	48 8b 74 24 10       	mov    0x10(%rsp),%rsi
    c874:	48 89 ef             	mov    %rbp,%rdi
    c877:	e8 04 6a ff ff       	call   3280 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE4swapERS4_@plt>
    c87c:	e9 a1 fe ff ff       	jmp    c722 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0xc2>
    c881:	0f b6 84 24 c0 00 00 	movzbl 0xc0(%rsp),%eax
    c888:	00 
    c889:	4c 89 e6             	mov    %r12,%rsi
    c88c:	48 8d 54 24 2f       	lea    0x2f(%rsp),%rdx
    c891:	48 89 ef             	mov    %rbp,%rdi
    c894:	88 44 24 2f          	mov    %al,0x2f(%rsp)
    c898:	e8 33 fc ff ff       	call   c4d0 <_ZSt11__make_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_RSS_>
    c89d:	48 8d 44 24 50       	lea    0x50(%rsp),%rax
    c8a2:	48 89 04 24          	mov    %rax,(%rsp)
    c8a6:	4c 8d 75 10          	lea    0x10(%rbp),%r14
    c8aa:	49 83 ec 10          	sub    $0x10,%r12
    c8ae:	4c 8d 6c 24 40       	lea    0x40(%rsp),%r13
    c8b3:	4c 8d 7c 24 60       	lea    0x60(%rsp),%r15
    c8b8:	0f 1f 84 00 00 00 00 	nopl   0x0(%rax,%rax,1)
    c8bf:	00 
    c8c0:	49 8b 44 24 f0       	mov    -0x10(%r12),%rax
    c8c5:	4c 89 6c 24 30       	mov    %r13,0x30(%rsp)
    c8ca:	49 8d 5c 24 f0       	lea    -0x10(%r12),%rbx
    c8cf:	4c 39 e0             	cmp    %r12,%rax
    c8d2:	0f 84 25 02 00 00    	je     cafd <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x49d>
    c8d8:	48 89 44 24 30       	mov    %rax,0x30(%rsp)
    c8dd:	49 8b 04 24          	mov    (%r12),%rax
    c8e1:	48 89 44 24 40       	mov    %rax,0x40(%rsp)
    c8e6:	4d 89 64 24 f0       	mov    %r12,-0x10(%r12)
    c8eb:	41 c6 04 24 00       	movb   $0x0,(%r12)
    c8f0:	49 8b 44 24 f8       	mov    -0x8(%r12),%rax
    c8f5:	49 c7 44 24 f8 00 00 	movq   $0x0,-0x8(%r12)
    c8fc:	00 00 
    c8fe:	48 89 44 24 38       	mov    %rax,0x38(%rsp)
    c903:	48 8b 45 00          	mov    0x0(%rbp),%rax
    c907:	49 39 c6             	cmp    %rax,%r14
    c90a:	0f 84 bc 01 00 00    	je     cacc <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x46c>
    c910:	49 89 44 24 f0       	mov    %rax,-0x10(%r12)
    c915:	48 8b 45 08          	mov    0x8(%rbp),%rax
    c919:	49 89 44 24 f8       	mov    %rax,-0x8(%r12)
    c91e:	48 8b 45 10          	mov    0x10(%rbp),%rax
    c922:	49 89 04 24          	mov    %rax,(%r12)
    c926:	4c 89 75 00          	mov    %r14,0x0(%rbp)
    c92a:	4c 89 f0             	mov    %r14,%rax
    c92d:	48 c7 45 08 00 00 00 	movq   $0x0,0x8(%rbp)
    c934:	00 
    c935:	c6 00 00             	movb   $0x0,(%rax)
    c938:	4c 89 7c 24 50       	mov    %r15,0x50(%rsp)
    c93d:	48 8b 44 24 30       	mov    0x30(%rsp),%rax
    c942:	4c 39 e8             	cmp    %r13,%rax
    c945:	0f 84 70 01 00 00    	je     cabb <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x45b>
    c94b:	48 89 44 24 50       	mov    %rax,0x50(%rsp)
    c950:	48 8b 44 24 40       	mov    0x40(%rsp),%rax
    c955:	48 89 44 24 60       	mov    %rax,0x60(%rsp)
    c95a:	48 29 eb             	sub    %rbp,%rbx
    c95d:	48 8b 44 24 38       	mov    0x38(%rsp),%rax
    c962:	48 8b 0c 24          	mov    (%rsp),%rcx
    c966:	48 89 da             	mov    %rbx,%rdx
    c969:	48 89 ef             	mov    %rbp,%rdi
    c96c:	48 c1 fa 05          	sar    $0x5,%rdx
    c970:	31 f6                	xor    %esi,%esi
    c972:	48 89 44 24 58       	mov    %rax,0x58(%rsp)
    c977:	4c 89 6c 24 30       	mov    %r13,0x30(%rsp)
    c97c:	48 c7 44 24 38 00 00 	movq   $0x0,0x38(%rsp)
    c983:	00 00 
    c985:	c6 44 24 40 00       	movb   $0x0,0x40(%rsp)
    c98a:	e8 a1 86 ff ff       	call   5030 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0>
    c98f:	48 8b 7c 24 50       	mov    0x50(%rsp),%rdi
    c994:	4c 39 ff             	cmp    %r15,%rdi
    c997:	74 0e                	je     c9a7 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x347>
    c999:	48 8b 44 24 60       	mov    0x60(%rsp),%rax
    c99e:	48 8d 70 01          	lea    0x1(%rax),%rsi
    c9a2:	e8 79 69 ff ff       	call   3320 <_ZdlPvm@plt>
    c9a7:	48 8b 7c 24 30       	mov    0x30(%rsp),%rdi
    c9ac:	4c 39 ef             	cmp    %r13,%rdi
    c9af:	0f 84 f3 00 00 00    	je     caa8 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x448>
    c9b5:	48 8b 44 24 40       	mov    0x40(%rsp),%rax
    c9ba:	49 83 ec 20          	sub    $0x20,%r12
    c9be:	48 8d 70 01          	lea    0x1(%rax),%rsi
    c9c2:	e8 59 69 ff ff       	call   3320 <_ZdlPvm@plt>
    c9c7:	48 83 fb 20          	cmp    $0x20,%rbx
    c9cb:	0f 8f ef fe ff ff    	jg     c8c0 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x260>
    c9d1:	48 8b 44 24 78       	mov    0x78(%rsp),%rax
    c9d6:	64 48 2b 04 25 28 00 	sub    %fs:0x28,%rax
    c9dd:	00 00 
    c9df:	0f 85 54 01 00 00    	jne    cb39 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x4d9>
    c9e5:	48 81 c4 88 00 00 00 	add    $0x88,%rsp
    c9ec:	5b                   	pop    %rbx
    c9ed:	5d                   	pop    %rbp
    c9ee:	41 5c                	pop    %r12
    c9f0:	41 5d                	pop    %r13
    c9f2:	41 5e                	pop    %r14
    c9f4:	41 5f                	pop    %r15
    c9f6:	c3                   	ret    
    c9f7:	4d 85 ff             	test   %r15,%r15
    c9fa:	0f 84 60 fe ff ff    	je     c860 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x200>
    ca00:	48 8b 04 24          	mov    (%rsp),%rax
    ca04:	49 8b 3c 24          	mov    (%r12),%rdi
    ca08:	48 8b 70 e0          	mov    -0x20(%rax),%rsi
    ca0c:	4c 89 fa             	mov    %r15,%rdx
    ca0f:	e8 5c 68 ff ff       	call   3270 <memcmp@plt>
    ca14:	85 c0                	test   %eax,%eax
    ca16:	0f 84 44 fe ff ff    	je     c860 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x200>
    ca1c:	0f 88 8f fd ff ff    	js     c7b1 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x151>
    ca22:	49 39 df             	cmp    %rbx,%r15
    ca25:	0f 85 3e fe ff ff    	jne    c869 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x209>
    ca2b:	4d 85 ff             	test   %r15,%r15
    ca2e:	0f 84 3b fe ff ff    	je     c86f <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x20f>
    ca34:	48 8b 04 24          	mov    (%rsp),%rax
    ca38:	48 8b 7d 20          	mov    0x20(%rbp),%rdi
    ca3c:	48 8b 70 e0          	mov    -0x20(%rax),%rsi
    ca40:	4c 89 fa             	mov    %r15,%rdx
    ca43:	e8 28 68 ff ff       	call   3270 <memcmp@plt>
    ca48:	85 c0                	test   %eax,%eax
    ca4a:	0f 84 1f fe ff ff    	je     c86f <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x20f>
    ca50:	0f 88 c1 fc ff ff    	js     c717 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0xb7>
    ca56:	e9 14 fe ff ff       	jmp    c86f <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x20f>
    ca5b:	49 39 df             	cmp    %rbx,%r15
    ca5e:	48 8b 7c 24 18       	mov    0x18(%rsp),%rdi
    ca63:	0f 85 9a fc ff ff    	jne    c703 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0xa3>
    ca69:	48 8b 70 e0          	mov    -0x20(%rax),%rsi
    ca6d:	48 89 da             	mov    %rbx,%rdx
    ca70:	e8 fb 67 ff ff       	call   3270 <memcmp@plt>
    ca75:	85 c0                	test   %eax,%eax
    ca77:	0f 84 8f fc ff ff    	je     c70c <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0xac>
    ca7d:	0f 89 89 fc ff ff    	jns    c70c <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0xac>
    ca83:	e9 e7 fd ff ff       	jmp    c86f <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x20f>
    ca88:	48 8b 04 24          	mov    (%rsp),%rax
    ca8c:	4c 8b 78 e8          	mov    -0x18(%rax),%r15
    ca90:	4c 8d 68 e0          	lea    -0x20(%rax),%r13
    ca94:	4d 85 ff             	test   %r15,%r15
    ca97:	0f 85 66 fc ff ff    	jne    c703 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0xa3>
    ca9d:	e9 6a fc ff ff       	jmp    c70c <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0xac>
    caa2:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)
    caa8:	49 83 ec 20          	sub    $0x20,%r12
    caac:	48 83 fb 20          	cmp    $0x20,%rbx
    cab0:	0f 8f 0a fe ff ff    	jg     c8c0 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x260>
    cab6:	e9 16 ff ff ff       	jmp    c9d1 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x371>
    cabb:	c5 f9 6f 4c 24 40    	vmovdqa 0x40(%rsp),%xmm1
    cac1:	c5 f9 7f 4c 24 60    	vmovdqa %xmm1,0x60(%rsp)
    cac7:	e9 8e fe ff ff       	jmp    c95a <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x2fa>
    cacc:	48 8b 55 08          	mov    0x8(%rbp),%rdx
    cad0:	48 85 d2             	test   %rdx,%rdx
    cad3:	74 15                	je     caea <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x48a>
    cad5:	48 83 fa 01          	cmp    $0x1,%rdx
    cad9:	74 50                	je     cb2b <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x4cb>
    cadb:	4c 89 f6             	mov    %r14,%rsi
    cade:	4c 89 e7             	mov    %r12,%rdi
    cae1:	e8 5a 66 ff ff       	call   3140 <memcpy@plt>
    cae6:	48 8b 55 08          	mov    0x8(%rbp),%rdx
    caea:	49 89 54 24 f8       	mov    %rdx,-0x8(%r12)
    caef:	41 c6 04 14 00       	movb   $0x0,(%r12,%rdx,1)
    caf4:	48 8b 45 00          	mov    0x0(%rbp),%rax
    caf8:	e9 30 fe ff ff       	jmp    c92d <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x2cd>
    cafd:	c4 c1 7a 6f 04 24    	vmovdqu (%r12),%xmm0
    cb03:	c5 f9 7f 44 24 40    	vmovdqa %xmm0,0x40(%rsp)
    cb09:	e9 d8 fd ff ff       	jmp    c8e6 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x286>
    cb0e:	48 85 db             	test   %rbx,%rbx
    cb11:	0f 84 f5 fb ff ff    	je     c70c <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0xac>
    cb17:	48 8b 7d 20          	mov    0x20(%rbp),%rdi
    cb1b:	e9 49 ff ff ff       	jmp    ca69 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x409>
    cb20:	0f 88 f1 fb ff ff    	js     c717 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0xb7>
    cb26:	e9 86 fc ff ff       	jmp    c7b1 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x151>
    cb2b:	0f b6 45 10          	movzbl 0x10(%rbp),%eax
    cb2f:	41 88 04 24          	mov    %al,(%r12)
    cb33:	48 8b 55 08          	mov    0x8(%rbp),%rdx
    cb37:	eb b1                	jmp    caea <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x48a>
    cb39:	e8 12 66 ff ff       	call   3150 <__stack_chk_fail@plt>
    cb3e:	66 90                	xchg   %ax,%ax

000000000000cb40 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_>:
    cb40:	41 57                	push   %r15
    cb42:	41 56                	push   %r14
    cb44:	41 55                	push   %r13
    cb46:	41 54                	push   %r12
    cb48:	4c 8d a7 a0 00 00 00 	lea    0xa0(%rdi),%r12
    cb4f:	55                   	push   %rbp
    cb50:	48 89 f5             	mov    %rsi,%rbp
    cb53:	53                   	push   %rbx
    cb54:	48 89 d3             	mov    %rdx,%rbx
    cb57:	48 81 ec 98 00 00 00 	sub    $0x98,%rsp
    cb5e:	48 89 7c 24 20       	mov    %rdi,0x20(%rsp)
    cb63:	48 89 f7             	mov    %rsi,%rdi
    cb66:	64 48 8b 04 25 28 00 	mov    %fs:0x28,%rax
    cb6d:	00 00 
    cb6f:	48 89 84 24 88 00 00 	mov    %rax,0x88(%rsp)
    cb76:	00 
    cb77:	31 c0                	xor    %eax,%eax
    cb79:	e8 b2 67 ff ff       	call   3330 <strlen@plt>
    cb7e:	48 89 df             	mov    %rbx,%rdi
    cb81:	48 89 44 24 60       	mov    %rax,0x60(%rsp)
    cb86:	48 89 6c 24 68       	mov    %rbp,0x68(%rsp)
    cb8b:	e8 a0 67 ff ff       	call   3330 <strlen@plt>
    cb90:	bf f0 00 00 00       	mov    $0xf0,%edi
    cb95:	48 89 44 24 70       	mov    %rax,0x70(%rsp)
    cb9a:	48 89 5c 24 78       	mov    %rbx,0x78(%rsp)
    cb9f:	e8 fc 66 ff ff       	call   32a0 <_Znwm@plt>
    cba4:	49 89 c5             	mov    %rax,%r13
    cba7:	48 83 c0 10          	add    $0x10,%rax
    cbab:	48 89 44 24 38       	mov    %rax,0x38(%rsp)
    cbb0:	49 8d 45 48          	lea    0x48(%r13),%rax
    cbb4:	48 89 04 24          	mov    %rax,(%rsp)
    cbb8:	49 89 45 38          	mov    %rax,0x38(%r13)
    cbbc:	49 8d 45 78          	lea    0x78(%r13),%rax
    cbc0:	48 89 44 24 10       	mov    %rax,0x10(%rsp)
    cbc5:	49 89 45 68          	mov    %rax,0x68(%r13)
    cbc9:	48 8b 7c 24 60       	mov    0x60(%rsp),%rdi
    cbce:	48 8d 05 7b 9b ff ff 	lea    -0x6485(%rip),%rax        # 6750 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse8Argument8m_actionMUlS8_E_EE9_M_invokeERKSt9_Any_dataS8_>
    cbd5:	49 89 85 b0 00 00 00 	mov    %rax,0xb0(%r13)
    cbdc:	48 8d 05 2d 97 ff ff 	lea    -0x68d3(%rip),%rax        # 6310 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse8Argument8m_actionMUlS8_E_EE10_M_managerERSt9_Any_dataRKSE_St18_Manager_operation>
    cbe3:	49 89 85 a8 00 00 00 	mov    %rax,0xa8(%r13)
    cbea:	49 c7 45 10 00 00 00 	movq   $0x0,0x10(%r13)
    cbf1:	00 
    cbf2:	49 c7 45 18 00 00 00 	movq   $0x0,0x18(%r13)
    cbf9:	00 
    cbfa:	49 c7 45 20 00 00 00 	movq   $0x0,0x20(%r13)
    cc01:	00 
    cc02:	49 c7 45 28 00 00 00 	movq   $0x0,0x28(%r13)
    cc09:	00 
    cc0a:	49 c7 45 30 00 00 00 	movq   $0x0,0x30(%r13)
    cc11:	00 
    cc12:	49 c7 45 40 00 00 00 	movq   $0x0,0x40(%r13)
    cc19:	00 
    cc1a:	41 c6 45 48 00       	movb   $0x0,0x48(%r13)
    cc1f:	49 c7 45 58 00 00 00 	movq   $0x0,0x58(%r13)
    cc26:	00 
    cc27:	49 c7 45 60 00 00 00 	movq   $0x0,0x60(%r13)
    cc2e:	00 
    cc2f:	49 c7 45 70 00 00 00 	movq   $0x0,0x70(%r13)
    cc36:	00 
    cc37:	41 c6 45 78 00       	movb   $0x0,0x78(%r13)
    cc3c:	49 c7 85 88 00 00 00 	movq   $0x0,0x88(%r13)
    cc43:	00 00 00 00 
    cc47:	49 c7 85 90 00 00 00 	movq   $0x0,0x90(%r13)
    cc4e:	00 00 00 00 
    cc52:	41 c6 85 b8 00 00 00 	movb   $0x0,0xb8(%r13)
    cc59:	00 
    cc5a:	49 c7 85 c0 00 00 00 	movq   $0x0,0xc0(%r13)
    cc61:	00 00 00 00 
    cc65:	49 c7 85 c8 00 00 00 	movq   $0x0,0xc8(%r13)
    cc6c:	00 00 00 00 
    cc70:	49 c7 85 d0 00 00 00 	movq   $0x0,0xd0(%r13)
    cc77:	00 00 00 00 
    cc7b:	49 c7 85 d8 00 00 00 	movq   $0x1,0xd8(%r13)
    cc82:	01 00 00 00 
    cc86:	49 c7 85 e0 00 00 00 	movq   $0x1,0xe0(%r13)
    cc8d:	01 00 00 00 
    cc91:	41 c6 85 e8 00 00 00 	movb   $0x0,0xe8(%r13)
    cc98:	00 
    cc99:	48 8b 44 24 68       	mov    0x68(%rsp),%rax
    cc9e:	48 85 ff             	test   %rdi,%rdi
    cca1:	74 09                	je     ccac <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x16c>
    cca3:	80 38 2d             	cmpb   $0x2d,(%rax)
    cca6:	0f 84 66 04 00 00    	je     d112 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x5d2>
    ccac:	48 8b 7c 24 70       	mov    0x70(%rsp),%rdi
    ccb1:	48 85 ff             	test   %rdi,%rdi
    ccb4:	0f 84 92 05 00 00    	je     d24c <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x70c>
    ccba:	48 8b 44 24 78       	mov    0x78(%rsp),%rax
    ccbf:	31 d2                	xor    %edx,%edx
    ccc1:	80 38 2d             	cmpb   $0x2d,(%rax)
    ccc4:	0f 84 61 05 00 00    	je     d22b <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x6eb>
    ccca:	41 0f b6 85 e9 00 00 	movzbl 0xe9(%r13),%eax
    ccd1:	00 
    ccd2:	83 e2 0f             	and    $0xf,%edx
    ccd5:	83 e0 f0             	and    $0xfffffff0,%eax
    ccd8:	49 8b 6d 18          	mov    0x18(%r13),%rbp
    ccdc:	09 d0                	or     %edx,%eax
    ccde:	41 88 85 e9 00 00 00 	mov    %al,0xe9(%r13)
    cce5:	49 3b 6d 20          	cmp    0x20(%r13),%rbp
    cce9:	0f 84 87 04 00 00    	je     d176 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x636>
    ccef:	48 8b 5c 24 68       	mov    0x68(%rsp),%rbx
    ccf4:	4c 8b 74 24 60       	mov    0x60(%rsp),%r14
    ccf9:	48 89 d8             	mov    %rbx,%rax
    ccfc:	48 8d 7d 10          	lea    0x10(%rbp),%rdi
    cd00:	4c 01 f0             	add    %r14,%rax
    cd03:	48 89 7d 00          	mov    %rdi,0x0(%rbp)
    cd07:	74 09                	je     cd12 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x1d2>
    cd09:	48 85 db             	test   %rbx,%rbx
    cd0c:	0f 84 41 05 00 00    	je     d253 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x713>
    cd12:	4c 89 74 24 58       	mov    %r14,0x58(%rsp)
    cd17:	49 83 fe 0f          	cmp    $0xf,%r14
    cd1b:	0f 87 0b 03 00 00    	ja     d02c <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x4ec>
    cd21:	49 83 fe 01          	cmp    $0x1,%r14
    cd25:	0f 85 a0 04 00 00    	jne    d1cb <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x68b>
    cd2b:	0f b6 03             	movzbl (%rbx),%eax
    cd2e:	88 45 10             	mov    %al,0x10(%rbp)
    cd31:	4c 89 75 08          	mov    %r14,0x8(%rbp)
    cd35:	42 c6 04 37 00       	movb   $0x0,(%rdi,%r14,1)
    cd3a:	49 8b 45 18          	mov    0x18(%r13),%rax
    cd3e:	48 8d 68 20          	lea    0x20(%rax),%rbp
    cd42:	49 89 6d 18          	mov    %rbp,0x18(%r13)
    cd46:	49 39 6d 20          	cmp    %rbp,0x20(%r13)
    cd4a:	0f 84 4b 04 00 00    	je     d19b <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x65b>
    cd50:	48 8b 5c 24 78       	mov    0x78(%rsp),%rbx
    cd55:	4c 8b 74 24 70       	mov    0x70(%rsp),%r14
    cd5a:	48 89 d8             	mov    %rbx,%rax
    cd5d:	48 8d 7d 10          	lea    0x10(%rbp),%rdi
    cd61:	4c 01 f0             	add    %r14,%rax
    cd64:	48 89 7d 00          	mov    %rdi,0x0(%rbp)
    cd68:	74 09                	je     cd73 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x233>
    cd6a:	48 85 db             	test   %rbx,%rbx
    cd6d:	0f 84 f1 04 00 00    	je     d264 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x724>
    cd73:	4c 89 74 24 58       	mov    %r14,0x58(%rsp)
    cd78:	49 83 fe 0f          	cmp    $0xf,%r14
    cd7c:	0f 87 6e 04 00 00    	ja     d1f0 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x6b0>
    cd82:	49 83 fe 01          	cmp    $0x1,%r14
    cd86:	0f 85 54 04 00 00    	jne    d1e0 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x6a0>
    cd8c:	0f b6 03             	movzbl (%rbx),%eax
    cd8f:	88 45 10             	mov    %al,0x10(%rbp)
    cd92:	4c 89 75 08          	mov    %r14,0x8(%rbp)
    cd96:	42 c6 04 37 00       	movb   $0x0,(%rdi,%r14,1)
    cd9b:	49 8b 45 18          	mov    0x18(%r13),%rax
    cd9f:	48 8d 68 20          	lea    0x20(%rax),%rbp
    cda3:	49 89 6d 18          	mov    %rbp,0x18(%r13)
    cda7:	4d 8b 75 10          	mov    0x10(%r13),%r14
    cdab:	49 39 ee             	cmp    %rbp,%r14
    cdae:	74 61                	je     ce11 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x2d1>
    cdb0:	48 89 eb             	mov    %rbp,%rbx
    cdb3:	4c 29 f3             	sub    %r14,%rbx
    cdb6:	48 89 d8             	mov    %rbx,%rax
    cdb9:	48 c1 f8 05          	sar    $0x5,%rax
    cdbd:	ba 3f 00 00 00       	mov    $0x3f,%edx
    cdc2:	f3 48 0f bd c0       	lzcnt  %rax,%rax
    cdc7:	29 c2                	sub    %eax,%edx
    cdc9:	48 63 d2             	movslq %edx,%rdx
    cdcc:	48 01 d2             	add    %rdx,%rdx
    cdcf:	48 89 ee             	mov    %rbp,%rsi
    cdd2:	4c 89 f7             	mov    %r14,%rdi
    cdd5:	e8 86 f8 ff ff       	call   c660 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_>
    cdda:	48 81 fb 00 02 00 00 	cmp    $0x200,%rbx
    cde1:	0f 8e d4 03 00 00    	jle    d1bb <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x67b>
    cde7:	49 8d 9e 00 02 00 00 	lea    0x200(%r14),%rbx
    cdee:	48 89 de             	mov    %rbx,%rsi
    cdf1:	4c 89 f7             	mov    %r14,%rdi
    cdf4:	e8 b7 9d ff ff       	call   6bb0 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0>
    cdf9:	48 39 eb             	cmp    %rbp,%rbx
    cdfc:	74 13                	je     ce11 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x2d1>
    cdfe:	66 90                	xchg   %ax,%ax
    ce00:	48 89 df             	mov    %rbx,%rdi
    ce03:	48 83 c3 20          	add    $0x20,%rbx
    ce07:	e8 d4 9a ff ff       	call   68e0 <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0>
    ce0c:	48 39 dd             	cmp    %rbx,%rbp
    ce0f:	75 ef                	jne    ce00 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x2c0>
    ce11:	4c 89 e6             	mov    %r12,%rsi
    ce14:	4c 89 ef             	mov    %r13,%rdi
    ce17:	e8 d4 64 ff ff       	call   32f0 <_ZNSt8__detail15_List_node_base7_M_hookEPS0_@plt>
    ce1c:	48 8b 44 24 20       	mov    0x20(%rsp),%rax
    ce21:	48 ff 80 b0 00 00 00 	incq   0xb0(%rax)
    ce28:	41 f6 85 e9 00 00 00 	testb  $0x1,0xe9(%r13)
    ce2f:	01 
    ce30:	0f 84 05 03 00 00    	je     d13b <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x5fb>
    ce36:	48 8b 5c 24 20       	mov    0x20(%rsp),%rbx
    ce3b:	49 8b 45 18          	mov    0x18(%r13),%rax
    ce3f:	4d 8b 75 10          	mov    0x10(%r13),%r14
    ce43:	48 8d b3 b8 00 00 00 	lea    0xb8(%rbx),%rsi
    ce4a:	48 81 c3 c0 00 00 00 	add    $0xc0,%rbx
    ce51:	48 89 5c 24 18       	mov    %rbx,0x18(%rsp)
    ce56:	48 89 44 24 28       	mov    %rax,0x28(%rsp)
    ce5b:	48 89 74 24 30       	mov    %rsi,0x30(%rsp)
    ce60:	bb 00 00 00 80       	mov    $0x80000000,%ebx
    ce65:	49 39 c6             	cmp    %rax,%r14
    ce68:	0f 84 90 01 00 00    	je     cffe <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x4be>
    ce6e:	66 90                	xchg   %ax,%ax
    ce70:	48 8b 44 24 20       	mov    0x20(%rsp),%rax
    ce75:	c4 c1 7a 7e 46 08    	vmovq  0x8(%r14),%xmm0
    ce7b:	4c 8b b8 c8 00 00 00 	mov    0xc8(%rax),%r15
    ce82:	4d 8b 26             	mov    (%r14),%r12
    ce85:	48 8b 6c 24 18       	mov    0x18(%rsp),%rbp
    ce8a:	c5 f9 7f 04 24       	vmovdqa %xmm0,(%rsp)
    ce8f:	4d 85 ff             	test   %r15,%r15
    ce92:	75 15                	jne    cea9 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x369>
    ce94:	e9 d8 00 00 00       	jmp    cf71 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x431>
    ce99:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
    cea0:	4d 8b 7f 18          	mov    0x18(%r15),%r15
    cea4:	4d 85 ff             	test   %r15,%r15
    cea7:	74 61                	je     cf0a <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x3ca>
    cea9:	c4 c1 7a 7e 47 20    	vmovq  0x20(%r15),%xmm0
    ceaf:	62 f2 fd 08 3b 0c 24 	vpminuq (%rsp),%xmm0,%xmm1
    ceb6:	c4 e1 f9 7e ca       	vmovq  %xmm1,%rdx
    cebb:	48 85 d2             	test   %rdx,%rdx
    cebe:	74 1c                	je     cedc <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x39c>
    cec0:	49 8b 7f 28          	mov    0x28(%r15),%rdi
    cec4:	4c 89 e6             	mov    %r12,%rsi
    cec7:	c5 f9 d6 44 24 10    	vmovq  %xmm0,0x10(%rsp)
    cecd:	e8 9e 63 ff ff       	call   3270 <memcmp@plt>
    ced2:	85 c0                	test   %eax,%eax
    ced4:	c5 fa 7e 44 24 10    	vmovq  0x10(%rsp),%xmm0
    ceda:	75 1e                	jne    cefa <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x3ba>
    cedc:	c5 f9 fb 04 24       	vpsubq (%rsp),%xmm0,%xmm0
    cee1:	c4 e1 f9 7e c0       	vmovq  %xmm0,%rax
    cee6:	48 39 d8             	cmp    %rbx,%rax
    cee9:	7d 13                	jge    cefe <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x3be>
    ceeb:	48 b9 ff ff ff 7f ff 	movabs $0xffffffff7fffffff,%rcx
    cef2:	ff ff ff 
    cef5:	48 39 c8             	cmp    %rcx,%rax
    cef8:	7e a6                	jle    cea0 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x360>
    cefa:	85 c0                	test   %eax,%eax
    cefc:	78 a2                	js     cea0 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x360>
    cefe:	4c 89 fd             	mov    %r15,%rbp
    cf01:	4d 8b 7f 10          	mov    0x10(%r15),%r15
    cf05:	4d 85 ff             	test   %r15,%r15
    cf08:	75 9f                	jne    cea9 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x369>
    cf0a:	48 3b 6c 24 18       	cmp    0x18(%rsp),%rbp
    cf0f:	74 60                	je     cf71 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x431>
    cf11:	c5 fa 7e 45 20       	vmovq  0x20(%rbp),%xmm0
    cf16:	62 f2 fd 08 3b 0c 24 	vpminuq (%rsp),%xmm0,%xmm1
    cf1d:	c4 e1 f9 7e ca       	vmovq  %xmm1,%rdx
    cf22:	48 85 d2             	test   %rdx,%rdx
    cf25:	74 1c                	je     cf43 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x403>
    cf27:	48 8b 75 28          	mov    0x28(%rbp),%rsi
    cf2b:	4c 89 e7             	mov    %r12,%rdi
    cf2e:	c5 f9 d6 44 24 10    	vmovq  %xmm0,0x10(%rsp)
    cf34:	e8 37 63 ff ff       	call   3270 <memcmp@plt>
    cf39:	85 c0                	test   %eax,%eax
    cf3b:	c5 fa 7e 44 24 10    	vmovq  0x10(%rsp),%xmm0
    cf41:	75 26                	jne    cf69 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x429>
    cf43:	c5 f9 6f 1c 24       	vmovdqa (%rsp),%xmm3
    cf48:	c5 e1 fb c0          	vpsubq %xmm0,%xmm3,%xmm0
    cf4c:	c4 e1 f9 7e c0       	vmovq  %xmm0,%rax
    cf51:	48 39 d8             	cmp    %rbx,%rax
    cf54:	0f 8d 16 01 00 00    	jge    d070 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x530>
    cf5a:	48 b9 ff ff ff 7f ff 	movabs $0xffffffff7fffffff,%rcx
    cf61:	ff ff ff 
    cf64:	48 39 c8             	cmp    %rcx,%rax
    cf67:	7e 08                	jle    cf71 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x431>
    cf69:	85 c0                	test   %eax,%eax
    cf6b:	0f 89 ff 00 00 00    	jns    d070 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x530>
    cf71:	bf 38 00 00 00       	mov    $0x38,%edi
    cf76:	e8 25 63 ff ff       	call   32a0 <_Znwm@plt>
    cf7b:	49 89 c0             	mov    %rax,%r8
    cf7e:	48 8b 04 24          	mov    (%rsp),%rax
    cf82:	c4 c1 f9 6e d4       	vmovq  %r12,%xmm2
    cf87:	49 89 40 20          	mov    %rax,0x20(%r8)
    cf8b:	c4 c3 e9 22 c5 01    	vpinsrq $0x1,%r13,%xmm2,%xmm0
    cf91:	48 8b 7c 24 30       	mov    0x30(%rsp),%rdi
    cf96:	c4 c1 7a 7f 40 28    	vmovdqu %xmm0,0x28(%r8)
    cf9c:	49 8d 50 20          	lea    0x20(%r8),%rdx
    cfa0:	48 89 ee             	mov    %rbp,%rsi
    cfa3:	4c 89 44 24 10       	mov    %r8,0x10(%rsp)
    cfa8:	e8 93 e4 ff ff       	call   b440 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE29_M_get_insert_hint_unique_posESt23_Rb_tree_const_iteratorISA_ERS5_>
    cfad:	48 85 d2             	test   %rdx,%rdx
    cfb0:	4c 8b 44 24 10       	mov    0x10(%rsp),%r8
    cfb5:	49 89 d1             	mov    %rdx,%r9
    cfb8:	0f 84 c2 00 00 00    	je     d080 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x540>
    cfbe:	48 85 c0             	test   %rax,%rax
    cfc1:	75 0b                	jne    cfce <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x48e>
    cfc3:	48 3b 54 24 18       	cmp    0x18(%rsp),%rdx
    cfc8:	0f 85 c4 00 00 00    	jne    d092 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x552>
    cfce:	bf 01 00 00 00       	mov    $0x1,%edi
    cfd3:	48 8b 4c 24 18       	mov    0x18(%rsp),%rcx
    cfd8:	4c 89 ca             	mov    %r9,%rdx
    cfdb:	4c 89 c6             	mov    %r8,%rsi
    cfde:	e8 8d 63 ff ff       	call   3370 <_ZSt29_Rb_tree_insert_and_rebalancebPSt18_Rb_tree_node_baseS0_RS_@plt>
    cfe3:	48 8b 44 24 20       	mov    0x20(%rsp),%rax
    cfe8:	48 ff 80 e0 00 00 00 	incq   0xe0(%rax)
    cfef:	49 83 c6 20          	add    $0x20,%r14
    cff3:	4c 39 74 24 28       	cmp    %r14,0x28(%rsp)
    cff8:	0f 85 72 fe ff ff    	jne    ce70 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x330>
    cffe:	48 8b 84 24 88 00 00 	mov    0x88(%rsp),%rax
    d005:	00 
    d006:	64 48 2b 04 25 28 00 	sub    %fs:0x28,%rax
    d00d:	00 00 
    d00f:	0f 85 60 02 00 00    	jne    d275 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x735>
    d015:	48 8b 44 24 38       	mov    0x38(%rsp),%rax
    d01a:	48 81 c4 98 00 00 00 	add    $0x98,%rsp
    d021:	5b                   	pop    %rbx
    d022:	5d                   	pop    %rbp
    d023:	41 5c                	pop    %r12
    d025:	41 5d                	pop    %r13
    d027:	41 5e                	pop    %r14
    d029:	41 5f                	pop    %r15
    d02b:	c3                   	ret    
    d02c:	4c 8d 7c 24 58       	lea    0x58(%rsp),%r15
    d031:	31 d2                	xor    %edx,%edx
    d033:	4c 89 fe             	mov    %r15,%rsi
    d036:	48 89 ef             	mov    %rbp,%rdi
    d039:	e8 c2 62 ff ff       	call   3300 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE9_M_createERmm@plt>
    d03e:	48 89 45 00          	mov    %rax,0x0(%rbp)
    d042:	48 89 c7             	mov    %rax,%rdi
    d045:	48 8b 44 24 58       	mov    0x58(%rsp),%rax
    d04a:	48 89 45 10          	mov    %rax,0x10(%rbp)
    d04e:	4c 89 f2             	mov    %r14,%rdx
    d051:	48 89 de             	mov    %rbx,%rsi
    d054:	e8 e7 60 ff ff       	call   3140 <memcpy@plt>
    d059:	4c 8b 74 24 58       	mov    0x58(%rsp),%r14
    d05e:	48 8b 7d 00          	mov    0x0(%rbp),%rdi
    d062:	e9 ca fc ff ff       	jmp    cd31 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x1f1>
    d067:	66 0f 1f 84 00 00 00 	nopw   0x0(%rax,%rax,1)
    d06e:	00 00 
    d070:	4c 89 6d 30          	mov    %r13,0x30(%rbp)
    d074:	e9 76 ff ff ff       	jmp    cfef <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x4af>
    d079:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
    d080:	be 38 00 00 00       	mov    $0x38,%esi
    d085:	4c 89 c7             	mov    %r8,%rdi
    d088:	e8 93 62 ff ff       	call   3320 <_ZdlPvm@plt>
    d08d:	e9 5d ff ff ff       	jmp    cfef <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x4af>
    d092:	c5 fa 7e 42 20       	vmovq  0x20(%rdx),%xmm0
    d097:	62 f2 fd 08 3b 0c 24 	vpminuq (%rsp),%xmm0,%xmm1
    d09e:	c4 e1 f9 7e ca       	vmovq  %xmm1,%rdx
    d0a3:	48 85 d2             	test   %rdx,%rdx
    d0a6:	74 32                	je     d0da <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x59a>
    d0a8:	49 8b 71 28          	mov    0x28(%r9),%rsi
    d0ac:	4c 89 e7             	mov    %r12,%rdi
    d0af:	4c 89 44 24 40       	mov    %r8,0x40(%rsp)
    d0b4:	4c 89 4c 24 10       	mov    %r9,0x10(%rsp)
    d0b9:	c5 f9 d6 44 24 48    	vmovq  %xmm0,0x48(%rsp)
    d0bf:	e8 ac 61 ff ff       	call   3270 <memcmp@plt>
    d0c4:	85 c0                	test   %eax,%eax
    d0c6:	4c 8b 4c 24 10       	mov    0x10(%rsp),%r9
    d0cb:	4c 8b 44 24 40       	mov    0x40(%rsp),%r8
    d0d0:	c5 fa 7e 44 24 48    	vmovq  0x48(%rsp),%xmm0
    d0d6:	89 c7                	mov    %eax,%edi
    d0d8:	75 30                	jne    d10a <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x5ca>
    d0da:	c5 f9 6f 24 24       	vmovdqa (%rsp),%xmm4
    d0df:	31 ff                	xor    %edi,%edi
    d0e1:	c5 d9 fb c0          	vpsubq %xmm0,%xmm4,%xmm0
    d0e5:	c4 e1 f9 7e c0       	vmovq  %xmm0,%rax
    d0ea:	48 39 d8             	cmp    %rbx,%rax
    d0ed:	0f 8d e0 fe ff ff    	jge    cfd3 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x493>
    d0f3:	48 be ff ff ff 7f ff 	movabs $0xffffffff7fffffff,%rsi
    d0fa:	ff ff ff 
    d0fd:	48 39 f0             	cmp    %rsi,%rax
    d100:	0f 8e c8 fe ff ff    	jle    cfce <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x48e>
    d106:	c5 f9 7e c7          	vmovd  %xmm0,%edi
    d10a:	c1 ef 1f             	shr    $0x1f,%edi
    d10d:	e9 c1 fe ff ff       	jmp    cfd3 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x493>
    d112:	48 ff cf             	dec    %rdi
    d115:	0f 84 91 fb ff ff    	je     ccac <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x16c>
    d11b:	48 8d 70 01          	lea    0x1(%rax),%rsi
    d11f:	4c 8d 7c 24 58       	lea    0x58(%rsp),%r15
    d124:	e8 e7 ab ff ff       	call   7d10 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE>
    d129:	84 c0                	test   %al,%al
    d12b:	0f 85 7b fb ff ff    	jne    ccac <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x16c>
    d131:	ba 01 00 00 00       	mov    $0x1,%edx
    d136:	e9 8f fb ff ff       	jmp    ccca <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x18a>
    d13b:	48 8d b8 88 00 00 00 	lea    0x88(%rax),%rdi
    d142:	49 8b 55 00          	mov    0x0(%r13),%rdx
    d146:	48 89 c3             	mov    %rax,%rbx
    d149:	4c 39 ef             	cmp    %r13,%rdi
    d14c:	0f 84 e4 fc ff ff    	je     ce36 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x2f6>
    d152:	48 39 d7             	cmp    %rdx,%rdi
    d155:	0f 84 db fc ff ff    	je     ce36 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x2f6>
    d15b:	4c 89 ee             	mov    %r13,%rsi
    d15e:	e8 4d 61 ff ff       	call   32b0 <_ZNSt8__detail15_List_node_base11_M_transferEPS0_S1_@plt>
    d163:	48 ff 83 98 00 00 00 	incq   0x98(%rbx)
    d16a:	48 ff 8b b0 00 00 00 	decq   0xb0(%rbx)
    d171:	e9 c0 fc ff ff       	jmp    ce36 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x2f6>
    d176:	48 8b 7c 24 38       	mov    0x38(%rsp),%rdi
    d17b:	48 8d 54 24 60       	lea    0x60(%rsp),%rdx
    d180:	48 89 ee             	mov    %rbp,%rsi
    d183:	4c 8d 7c 24 58       	lea    0x58(%rsp),%r15
    d188:	e8 03 ea ff ff       	call   bb90 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJRSt17basic_string_viewIcS3_EEEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_>
    d18d:	49 8b 6d 18          	mov    0x18(%r13),%rbp
    d191:	49 39 6d 20          	cmp    %rbp,0x20(%r13)
    d195:	0f 85 b5 fb ff ff    	jne    cd50 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x210>
    d19b:	48 8b 7c 24 38       	mov    0x38(%rsp),%rdi
    d1a0:	48 8d 54 24 70       	lea    0x70(%rsp),%rdx
    d1a5:	48 89 ee             	mov    %rbp,%rsi
    d1a8:	4c 8d 7c 24 58       	lea    0x58(%rsp),%r15
    d1ad:	e8 de e9 ff ff       	call   bb90 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJRSt17basic_string_viewIcS3_EEEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_>
    d1b2:	49 8b 6d 18          	mov    0x18(%r13),%rbp
    d1b6:	e9 ec fb ff ff       	jmp    cda7 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x267>
    d1bb:	48 89 ee             	mov    %rbp,%rsi
    d1be:	4c 89 f7             	mov    %r14,%rdi
    d1c1:	e8 ea 99 ff ff       	call   6bb0 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm2EJLm0ELm1EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0>
    d1c6:	e9 46 fc ff ff       	jmp    ce11 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x2d1>
    d1cb:	4d 85 f6             	test   %r14,%r14
    d1ce:	0f 84 5d fb ff ff    	je     cd31 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x1f1>
    d1d4:	e9 75 fe ff ff       	jmp    d04e <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x50e>
    d1d9:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
    d1e0:	4d 85 f6             	test   %r14,%r14
    d1e3:	0f 84 a9 fb ff ff    	je     cd92 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x252>
    d1e9:	eb 27                	jmp    d212 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x6d2>
    d1eb:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)
    d1f0:	4c 8d 7c 24 58       	lea    0x58(%rsp),%r15
    d1f5:	31 d2                	xor    %edx,%edx
    d1f7:	4c 89 fe             	mov    %r15,%rsi
    d1fa:	48 89 ef             	mov    %rbp,%rdi
    d1fd:	e8 fe 60 ff ff       	call   3300 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE9_M_createERmm@plt>
    d202:	48 89 45 00          	mov    %rax,0x0(%rbp)
    d206:	48 89 c7             	mov    %rax,%rdi
    d209:	48 8b 44 24 58       	mov    0x58(%rsp),%rax
    d20e:	48 89 45 10          	mov    %rax,0x10(%rbp)
    d212:	4c 89 f2             	mov    %r14,%rdx
    d215:	48 89 de             	mov    %rbx,%rsi
    d218:	e8 23 5f ff ff       	call   3140 <memcpy@plt>
    d21d:	4c 8b 74 24 58       	mov    0x58(%rsp),%r14
    d222:	48 8b 7d 00          	mov    0x0(%rbp),%rdi
    d226:	e9 67 fb ff ff       	jmp    cd92 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x252>
    d22b:	48 ff cf             	dec    %rdi
    d22e:	0f 84 96 fa ff ff    	je     ccca <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x18a>
    d234:	48 8d 70 01          	lea    0x1(%rax),%rsi
    d238:	4c 8d 7c 24 58       	lea    0x58(%rsp),%r15
    d23d:	e8 ce aa ff ff       	call   7d10 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE>
    d242:	83 f0 01             	xor    $0x1,%eax
    d245:	89 c2                	mov    %eax,%edx
    d247:	e9 7e fa ff ff       	jmp    ccca <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x18a>
    d24c:	31 d2                	xor    %edx,%edx
    d24e:	e9 77 fa ff ff       	jmp    ccca <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x18a>
    d253:	48 8d 3d a6 2d 00 00 	lea    0x2da6(%rip),%rdi        # 10000 <_fini+0xddf>
    d25a:	4c 8d 7c 24 58       	lea    0x58(%rsp),%r15
    d25f:	e8 5c 61 ff ff       	call   33c0 <_ZSt19__throw_logic_errorPKc@plt>
    d264:	48 8d 3d 95 2d 00 00 	lea    0x2d95(%rip),%rdi        # 10000 <_fini+0xddf>
    d26b:	4c 8d 7c 24 58       	lea    0x58(%rsp),%r15
    d270:	e8 4b 61 ff ff       	call   33c0 <_ZSt19__throw_logic_errorPKc@plt>
    d275:	e8 d6 5e ff ff       	call   3150 <__stack_chk_fail@plt>
    d27a:	48 89 c5             	mov    %rax,%rbp
    d27d:	49 8d bd c0 00 00 00 	lea    0xc0(%r13),%rdi
    d284:	c5 f8 77             	vzeroupper 
    d287:	e8 a4 df ff ff       	call   b230 <_ZNSt6vectorISt3anySaIS0_EED1Ev>
    d28c:	41 0f b6 95 b8 00 00 	movzbl 0xb8(%r13),%edx
    d293:	00 
    d294:	48 8d 05 f5 65 00 00 	lea    0x65f5(%rip),%rax        # 13890 <_ZNSt8__detail9__variant12__gen_vtableIvOZNS0_16_Variant_storageILb0EJSt8functionIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEES3_IFvSC_EEEE8_M_resetEvEUlOT_E_JRSt7variantIJSE_SG_EEEE9_S_vtableE>
    d29b:	49 8d b5 98 00 00 00 	lea    0x98(%r13),%rsi
    d2a2:	4c 89 ff             	mov    %r15,%rdi
    d2a5:	ff 14 d0             	call   *(%rax,%rdx,8)
    d2a8:	49 8b 85 88 00 00 00 	mov    0x88(%r13),%rax
    d2af:	48 85 c0             	test   %rax,%rax
    d2b2:	74 10                	je     d2c4 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x784>
    d2b4:	49 8d b5 88 00 00 00 	lea    0x88(%r13),%rsi
    d2bb:	31 d2                	xor    %edx,%edx
    d2bd:	bf 03 00 00 00       	mov    $0x3,%edi
    d2c2:	ff d0                	call   *%rax
    d2c4:	49 8b 7d 68          	mov    0x68(%r13),%rdi
    d2c8:	48 39 7c 24 10       	cmp    %rdi,0x10(%rsp)
    d2cd:	74 0d                	je     d2dc <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x79c>
    d2cf:	49 8b 45 78          	mov    0x78(%r13),%rax
    d2d3:	48 8d 70 01          	lea    0x1(%rax),%rsi
    d2d7:	e8 44 60 ff ff       	call   3320 <_ZdlPvm@plt>
    d2dc:	49 8b 45 58          	mov    0x58(%r13),%rax
    d2e0:	48 85 c0             	test   %rax,%rax
    d2e3:	74 0d                	je     d2f2 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x7b2>
    d2e5:	49 8d 75 58          	lea    0x58(%r13),%rsi
    d2e9:	31 d2                	xor    %edx,%edx
    d2eb:	bf 03 00 00 00       	mov    $0x3,%edi
    d2f0:	ff d0                	call   *%rax
    d2f2:	49 8b 7d 38          	mov    0x38(%r13),%rdi
    d2f6:	48 39 3c 24          	cmp    %rdi,(%rsp)
    d2fa:	74 0d                	je     d309 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_+0x7c9>
    d2fc:	49 8b 45 48          	mov    0x48(%r13),%rax
    d300:	48 8d 70 01          	lea    0x1(%rax),%rsi
    d304:	e8 17 60 ff ff       	call   3320 <_ZdlPvm@plt>
    d309:	48 8b 7c 24 38       	mov    0x38(%rsp),%rdi
    d30e:	e8 fd b6 ff ff       	call   8a10 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EED1Ev>
    d313:	4c 89 ef             	mov    %r13,%rdi
    d316:	be f0 00 00 00       	mov    $0xf0,%esi
    d31b:	e8 00 60 ff ff       	call   3320 <_ZdlPvm@plt>
    d320:	48 89 ef             	mov    %rbp,%rdi
    d323:	e8 38 60 ff ff       	call   3360 <_Unwind_Resume@plt>
    d328:	0f 1f 84 00 00 00 00 	nopl   0x0(%rax,%rax,1)
    d32f:	00 

000000000000d330 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE>:
    d330:	41 57                	push   %r15
    d332:	41 56                	push   %r14
    d334:	41 55                	push   %r13
    d336:	41 54                	push   %r12
    d338:	41 89 cc             	mov    %ecx,%r12d
    d33b:	55                   	push   %rbp
    d33c:	53                   	push   %rbx
    d33d:	48 89 fb             	mov    %rdi,%rbx
    d340:	48 81 ec a8 00 00 00 	sub    $0xa8,%rsp
    d347:	64 48 8b 04 25 28 00 	mov    %fs:0x28,%rax
    d34e:	00 00 
    d350:	48 89 84 24 98 00 00 	mov    %rax,0x98(%rsp)
    d357:	00 
    d358:	31 c0                	xor    %eax,%eax
    d35a:	48 8d 47 10          	lea    0x10(%rdi),%rax
    d35e:	48 89 07             	mov    %rax,(%rdi)
    d361:	48 89 44 24 30       	mov    %rax,0x30(%rsp)
    d366:	48 8d 46 10          	lea    0x10(%rsi),%rax
    d36a:	48 8b 0e             	mov    (%rsi),%rcx
    d36d:	48 39 c1             	cmp    %rax,%rcx
    d370:	0f 84 4a 08 00 00    	je     dbc0 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0x890>
    d376:	48 89 0f             	mov    %rcx,(%rdi)
    d379:	48 8b 4e 10          	mov    0x10(%rsi),%rcx
    d37d:	48 89 4f 10          	mov    %rcx,0x10(%rdi)
    d381:	48 8b 4e 08          	mov    0x8(%rsi),%rcx
    d385:	48 89 06             	mov    %rax,(%rsi)
    d388:	48 8d 43 30          	lea    0x30(%rbx),%rax
    d38c:	48 89 4b 08          	mov    %rcx,0x8(%rbx)
    d390:	48 c7 46 08 00 00 00 	movq   $0x0,0x8(%rsi)
    d397:	00 
    d398:	c6 46 10 00          	movb   $0x0,0x10(%rsi)
    d39c:	48 89 43 20          	mov    %rax,0x20(%rbx)
    d3a0:	48 89 44 24 08       	mov    %rax,0x8(%rsp)
    d3a5:	48 8d 42 10          	lea    0x10(%rdx),%rax
    d3a9:	48 8b 0a             	mov    (%rdx),%rcx
    d3ac:	48 39 c1             	cmp    %rax,%rcx
    d3af:	0f 84 f3 07 00 00    	je     dba8 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0x878>
    d3b5:	48 89 4b 20          	mov    %rcx,0x20(%rbx)
    d3b9:	48 8b 4a 10          	mov    0x10(%rdx),%rcx
    d3bd:	48 89 4b 30          	mov    %rcx,0x30(%rbx)
    d3c1:	48 8b 4a 08          	mov    0x8(%rdx),%rcx
    d3c5:	48 89 02             	mov    %rax,(%rdx)
    d3c8:	48 8d 43 50          	lea    0x50(%rbx),%rax
    d3cc:	48 c7 42 08 00 00 00 	movq   $0x0,0x8(%rdx)
    d3d3:	00 
    d3d4:	48 89 4b 28          	mov    %rcx,0x28(%rbx)
    d3d8:	48 89 44 24 10       	mov    %rax,0x10(%rsp)
    d3dd:	c6 42 10 00          	movb   $0x0,0x10(%rdx)
    d3e1:	48 89 43 40          	mov    %rax,0x40(%rbx)
    d3e5:	48 8d 43 70          	lea    0x70(%rbx),%rax
    d3e9:	48 89 44 24 18       	mov    %rax,0x18(%rsp)
    d3ee:	48 89 43 60          	mov    %rax,0x60(%rbx)
    d3f2:	48 8d 83 88 00 00 00 	lea    0x88(%rbx),%rax
    d3f9:	c4 e1 f9 6e d8       	vmovq  %rax,%xmm3
    d3fe:	48 89 44 24 20       	mov    %rax,0x20(%rsp)
    d403:	48 8d 83 a0 00 00 00 	lea    0xa0(%rbx),%rax
    d40a:	c4 e2 79 59 c3       	vpbroadcastq %xmm3,%xmm0
    d40f:	c4 e1 f9 6e e0       	vmovq  %rax,%xmm4
    d414:	48 89 44 24 28       	mov    %rax,0x28(%rsp)
    d419:	c5 fa 7f 83 88 00 00 	vmovdqu %xmm0,0x88(%rbx)
    d420:	00 
    d421:	48 8d 83 c0 00 00 00 	lea    0xc0(%rbx),%rax
    d428:	c4 e2 79 59 c4       	vpbroadcastq %xmm4,%xmm0
    d42d:	48 c7 43 48 00 00 00 	movq   $0x0,0x48(%rbx)
    d434:	00 
    d435:	c6 43 50 00          	movb   $0x0,0x50(%rbx)
    d439:	48 c7 43 68 00 00 00 	movq   $0x0,0x68(%rbx)
    d440:	00 
    d441:	c6 43 70 00          	movb   $0x0,0x70(%rbx)
    d445:	c6 83 80 00 00 00 00 	movb   $0x0,0x80(%rbx)
    d44c:	48 c7 83 98 00 00 00 	movq   $0x0,0x98(%rbx)
    d453:	00 00 00 00 
    d457:	48 c7 83 b0 00 00 00 	movq   $0x0,0xb0(%rbx)
    d45e:	00 00 00 00 
    d462:	c7 83 c0 00 00 00 00 	movl   $0x0,0xc0(%rbx)
    d469:	00 00 00 
    d46c:	48 c7 83 c8 00 00 00 	movq   $0x0,0xc8(%rbx)
    d473:	00 00 00 00 
    d477:	48 89 83 d0 00 00 00 	mov    %rax,0xd0(%rbx)
    d47e:	48 89 83 d8 00 00 00 	mov    %rax,0xd8(%rbx)
    d485:	48 c7 83 e0 00 00 00 	movq   $0x0,0xe0(%rbx)
    d48c:	00 00 00 00 
    d490:	c5 fa 7f 83 a0 00 00 	vmovdqu %xmm0,0xa0(%rbx)
    d497:	00 
    d498:	41 f6 c4 01          	test   $0x1,%r12b
    d49c:	75 3a                	jne    d4d8 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0x1a8>
    d49e:	41 83 e4 02          	and    $0x2,%r12d
    d4a2:	0f 85 a8 03 00 00    	jne    d850 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0x520>
    d4a8:	48 8b 84 24 98 00 00 	mov    0x98(%rsp),%rax
    d4af:	00 
    d4b0:	64 48 2b 04 25 28 00 	sub    %fs:0x28,%rax
    d4b7:	00 00 
    d4b9:	0f 85 c1 09 00 00    	jne    de80 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0xb50>
    d4bf:	48 81 c4 a8 00 00 00 	add    $0xa8,%rsp
    d4c6:	5b                   	pop    %rbx
    d4c7:	5d                   	pop    %rbp
    d4c8:	41 5c                	pop    %r12
    d4ca:	41 5d                	pop    %r13
    d4cc:	41 5e                	pop    %r14
    d4ce:	41 5f                	pop    %r15
    d4d0:	c3                   	ret    
    d4d1:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
    d4d8:	48 8d 15 ee 2e 00 00 	lea    0x2eee(%rip),%rdx        # 103cd <_fini+0x11ac>
    d4df:	48 8d 35 ee 2e 00 00 	lea    0x2eee(%rip),%rsi        # 103d4 <_fini+0x11b3>
    d4e6:	48 89 df             	mov    %rbx,%rdi
    d4e9:	e8 52 f6 ff ff       	call   cb40 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_>
    d4ee:	48 8d 0d 3b 8e ff ff 	lea    -0x71c5(%rip),%rcx        # 6330 <_ZNSt17_Function_handlerIFvRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEZN8argparse14ArgumentParserC4ES5_S5_NS9_17default_argumentsEEUlRKT_E_E10_M_managerERSt9_Any_dataRKSH_St18_Manager_operation>
    d4f5:	48 89 5c 24 70       	mov    %rbx,0x70(%rsp)
    d4fa:	c6 84 24 90 00 00 00 	movb   $0x1,0x90(%rsp)
    d501:	01 
    d502:	48 8d 15 b7 b2 ff ff 	lea    -0x4d49(%rip),%rdx        # 87c0 <_ZNSt17_Function_handlerIFvRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEZN8argparse14ArgumentParserC4ES5_S5_NS9_17default_argumentsEEUlRKT_E_E9_M_invokeERKSt9_Any_dataS7_>
    d509:	c4 e1 f9 6e c9       	vmovq  %rcx,%xmm1
    d50e:	c4 e3 f1 22 c2 01    	vpinsrq $0x1,%rdx,%xmm1,%xmm0
    d514:	c5 f9 7f 84 24 80 00 	vmovdqa %xmm0,0x80(%rsp)
    d51b:	00 00 
    d51d:	48 89 c5             	mov    %rax,%rbp
    d520:	0f b6 80 a8 00 00 00 	movzbl 0xa8(%rax),%eax
    d527:	3c 01                	cmp    $0x1,%al
    d529:	0f 84 a1 07 00 00    	je     dcd0 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0x9a0>
    d52f:	4c 8d 7c 24 48       	lea    0x48(%rsp),%r15
    d534:	4c 8d 2d 55 63 00 00 	lea    0x6355(%rip),%r13        # 13890 <_ZNSt8__detail9__variant12__gen_vtableIvOZNS0_16_Variant_storageILb0EJSt8functionIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEES3_IFvSC_EEEE8_M_resetEvEUlOT_E_JRSt7variantIJSE_SG_EEEE9_S_vtableE>
    d53b:	48 8d b5 88 00 00 00 	lea    0x88(%rbp),%rsi
    d542:	4c 89 ff             	mov    %r15,%rdi
    d545:	41 ff 54 c5 00       	call   *0x0(%r13,%rax,8)
    d54a:	c6 85 a8 00 00 00 01 	movb   $0x1,0xa8(%rbp)
    d551:	c5 fa 6f bd 88 00 00 	vmovdqu 0x88(%rbp),%xmm7
    d558:	00 
    d559:	c5 f9 6f 44 24 70    	vmovdqa 0x70(%rsp),%xmm0
    d55f:	c5 f9 7f 7c 24 70    	vmovdqa %xmm7,0x70(%rsp)
    d565:	c5 fa 7f 85 88 00 00 	vmovdqu %xmm0,0x88(%rbp)
    d56c:	00 
    d56d:	48 8b 84 24 80 00 00 	mov    0x80(%rsp),%rax
    d574:	00 
    d575:	48 8b 95 a0 00 00 00 	mov    0xa0(%rbp),%rdx
    d57c:	48 c7 84 24 80 00 00 	movq   $0x0,0x80(%rsp)
    d583:	00 00 00 00 00 
    d588:	48 89 85 98 00 00 00 	mov    %rax,0x98(%rbp)
    d58f:	48 8b 84 24 88 00 00 	mov    0x88(%rsp),%rax
    d596:	00 
    d597:	48 89 94 24 88 00 00 	mov    %rdx,0x88(%rsp)
    d59e:	00 
    d59f:	48 89 85 a0 00 00 00 	mov    %rax,0xa0(%rbp)
    d5a6:	0f b6 84 24 90 00 00 	movzbl 0x90(%rsp),%eax
    d5ad:	00 
    d5ae:	49 8b 44 c5 00       	mov    0x0(%r13,%rax,8),%rax
    d5b3:	4c 8d 74 24 70       	lea    0x70(%rsp),%r14
    d5b8:	4c 89 f6             	mov    %r14,%rsi
    d5bb:	4c 89 ff             	mov    %r15,%rdi
    d5be:	ff d0                	call   *%rax
    d5c0:	80 bd a8 00 00 00 01 	cmpb   $0x1,0xa8(%rbp)
    d5c7:	0f 85 c4 08 00 00    	jne    de91 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0xb61>
    d5cd:	4c 8d ac 24 80 00 00 	lea    0x80(%rsp),%r13
    d5d4:	00 
    d5d5:	4c 89 6c 24 70       	mov    %r13,0x70(%rsp)
    d5da:	c6 84 24 84 00 00 00 	movb   $0x65,0x84(%rsp)
    d5e1:	65 
    d5e2:	c7 84 24 80 00 00 00 	movl   $0x736c6166,0x80(%rsp)
    d5e9:	66 61 6c 73 
    d5ed:	c6 84 24 85 00 00 00 	movb   $0x0,0x85(%rsp)
    d5f4:	00 
    d5f5:	48 c7 44 24 78 05 00 	movq   $0x5,0x78(%rsp)
    d5fc:	00 00 
    d5fe:	48 8b 45 58          	mov    0x58(%rbp),%rax
    d602:	c7 00 66 61 6c 73    	movl   $0x736c6166,(%rax)
    d608:	0f b6 94 24 84 00 00 	movzbl 0x84(%rsp),%edx
    d60f:	00 
    d610:	88 50 04             	mov    %dl,0x4(%rax)
    d613:	48 8b 44 24 78       	mov    0x78(%rsp),%rax
    d618:	48 8b 55 58          	mov    0x58(%rbp),%rdx
    d61c:	48 89 45 60          	mov    %rax,0x60(%rbp)
    d620:	c6 04 02 00          	movb   $0x0,(%rdx,%rax,1)
    d624:	48 c7 44 24 78 00 00 	movq   $0x0,0x78(%rsp)
    d62b:	00 00 
    d62d:	48 8b 44 24 70       	mov    0x70(%rsp),%rax
    d632:	c6 00 00             	movb   $0x0,(%rax)
    d635:	48 8b 7c 24 70       	mov    0x70(%rsp),%rdi
    d63a:	4c 39 ef             	cmp    %r13,%rdi
    d63d:	74 11                	je     d650 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0x320>
    d63f:	48 8b 84 24 80 00 00 	mov    0x80(%rsp),%rax
    d646:	00 
    d647:	48 8d 70 01          	lea    0x1(%rax),%rsi
    d64b:	e8 d0 5c ff ff       	call   3320 <_ZdlPvm@plt>
    d650:	48 8d 0d d9 8a ff ff 	lea    -0x7527(%rip),%rcx        # 6130 <_ZNSt3any17_Manager_internalIbE9_S_manageENS_3_OpEPKS_PNS_4_ArgE>
    d657:	48 8d 45 48          	lea    0x48(%rbp),%rax
    d65b:	48 89 0c 24          	mov    %rcx,(%rsp)
    d65f:	48 89 4c 24 70       	mov    %rcx,0x70(%rsp)
    d664:	48 c7 44 24 78 00 00 	movq   $0x0,0x78(%rsp)
    d66b:	00 00 
    d66d:	4c 39 f0             	cmp    %r14,%rax
    d670:	0f 84 32 07 00 00    	je     dda8 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0xa78>
    d676:	4c 8b 45 48          	mov    0x48(%rbp),%r8
    d67a:	4d 85 c0             	test   %r8,%r8
    d67d:	74 24                	je     d6a3 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0x373>
    d67f:	48 89 c6             	mov    %rax,%rsi
    d682:	31 d2                	xor    %edx,%edx
    d684:	bf 03 00 00 00       	mov    $0x3,%edi
    d689:	48 89 44 24 38       	mov    %rax,0x38(%rsp)
    d68e:	41 ff d0             	call   *%r8
    d691:	48 c7 45 48 00 00 00 	movq   $0x0,0x48(%rbp)
    d698:	00 
    d699:	48 8b 44 24 38       	mov    0x38(%rsp),%rax
    d69e:	48 8b 4c 24 70       	mov    0x70(%rsp),%rcx
    d6a3:	48 89 44 24 48       	mov    %rax,0x48(%rsp)
    d6a8:	4c 89 fa             	mov    %r15,%rdx
    d6ab:	4c 89 f6             	mov    %r14,%rsi
    d6ae:	bf 04 00 00 00       	mov    $0x4,%edi
    d6b3:	ff d1                	call   *%rcx
    d6b5:	48 8b 44 24 70       	mov    0x70(%rsp),%rax
    d6ba:	48 85 c0             	test   %rax,%rax
    d6bd:	74 0c                	je     d6cb <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0x39b>
    d6bf:	31 d2                	xor    %edx,%edx
    d6c1:	4c 89 f6             	mov    %r14,%rsi
    d6c4:	bf 03 00 00 00       	mov    $0x3,%edi
    d6c9:	ff d0                	call   *%rax
    d6cb:	31 d2                	xor    %edx,%edx
    d6cd:	4c 89 fe             	mov    %r15,%rsi
    d6d0:	4c 89 f7             	mov    %r14,%rdi
    d6d3:	4c 89 6c 24 70       	mov    %r13,0x70(%rsp)
    d6d8:	48 c7 44 24 48 1c 00 	movq   $0x1c,0x48(%rsp)
    d6df:	00 00 
    d6e1:	e8 1a 5c ff ff       	call   3300 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE9_M_createERmm@plt>
    d6e6:	48 8b 54 24 48       	mov    0x48(%rsp),%rdx
    d6eb:	48 bf 67 65 20 61 6e 	movabs $0x6520646e61206567,%rdi
    d6f2:	64 20 65 
    d6f5:	48 89 44 24 70       	mov    %rax,0x70(%rsp)
    d6fa:	48 89 94 24 80 00 00 	mov    %rdx,0x80(%rsp)
    d701:	00 
    d702:	c5 f9 6f 05 d6 30 00 	vmovdqa 0x30d6(%rip),%xmm0        # 107e0 <_ZTSN8argparse7details12parse_numberIfLNS0_12chars_formatE3EEE+0x80>
    d709:	00 
    d70a:	48 89 78 10          	mov    %rdi,0x10(%rax)
    d70e:	c7 40 18 78 69 74 73 	movl   $0x73746978,0x18(%rax)
    d715:	c5 fa 7f 00          	vmovdqu %xmm0,(%rax)
    d719:	48 8b 44 24 48       	mov    0x48(%rsp),%rax
    d71e:	48 8b 54 24 70       	mov    0x70(%rsp),%rdx
    d723:	48 89 44 24 78       	mov    %rax,0x78(%rsp)
    d728:	c6 04 02 00          	movb   $0x0,(%rdx,%rax,1)
    d72c:	48 8b 54 24 70       	mov    0x70(%rsp),%rdx
    d731:	48 8b 7d 28          	mov    0x28(%rbp),%rdi
    d735:	4c 39 ea             	cmp    %r13,%rdx
    d738:	0f 84 72 06 00 00    	je     ddb0 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0xa80>
    d73e:	48 8d 4d 38          	lea    0x38(%rbp),%rcx
    d742:	c5 fa 7e 44 24 78    	vmovq  0x78(%rsp),%xmm0
    d748:	48 8b b4 24 80 00 00 	mov    0x80(%rsp),%rsi
    d74f:	00 
    d750:	48 39 cf             	cmp    %rcx,%rdi
    d753:	0f 84 77 04 00 00    	je     dbd0 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0x8a0>
    d759:	c4 e3 f9 22 c6 01    	vpinsrq $0x1,%rsi,%xmm0,%xmm0
    d75f:	48 8b 4d 38          	mov    0x38(%rbp),%rcx
    d763:	48 89 55 28          	mov    %rdx,0x28(%rbp)
    d767:	c5 fa 7f 45 30       	vmovdqu %xmm0,0x30(%rbp)
    d76c:	48 85 ff             	test   %rdi,%rdi
    d76f:	0f 84 6a 04 00 00    	je     dbdf <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0x8af>
    d775:	48 89 7c 24 70       	mov    %rdi,0x70(%rsp)
    d77a:	48 89 8c 24 80 00 00 	mov    %rcx,0x80(%rsp)
    d781:	00 
    d782:	48 8b 04 24          	mov    (%rsp),%rax
    d786:	48 c7 44 24 78 00 00 	movq   $0x0,0x78(%rsp)
    d78d:	00 00 
    d78f:	4c 8d 74 24 50       	lea    0x50(%rsp),%r14
    d794:	c6 07 00             	movb   $0x0,(%rdi)
    d797:	48 89 44 24 50       	mov    %rax,0x50(%rsp)
    d79c:	48 8d 45 78          	lea    0x78(%rbp),%rax
    d7a0:	48 c7 44 24 58 01 00 	movq   $0x1,0x58(%rsp)
    d7a7:	00 00 
    d7a9:	4c 39 f0             	cmp    %r14,%rax
    d7ac:	0f 84 ce 05 00 00    	je     dd80 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0xa50>
    d7b2:	4c 8b 45 78          	mov    0x78(%rbp),%r8
    d7b6:	48 8d 0d 73 89 ff ff 	lea    -0x768d(%rip),%rcx        # 6130 <_ZNSt3any17_Manager_internalIbE9_S_manageENS_3_OpEPKS_PNS_4_ArgE>
    d7bd:	4d 85 c0             	test   %r8,%r8
    d7c0:	74 22                	je     d7e4 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0x4b4>
    d7c2:	48 89 c6             	mov    %rax,%rsi
    d7c5:	31 d2                	xor    %edx,%edx
    d7c7:	bf 03 00 00 00       	mov    $0x3,%edi
    d7cc:	48 89 04 24          	mov    %rax,(%rsp)
    d7d0:	41 ff d0             	call   *%r8
    d7d3:	48 c7 45 78 00 00 00 	movq   $0x0,0x78(%rbp)
    d7da:	00 
    d7db:	48 8b 04 24          	mov    (%rsp),%rax
    d7df:	48 8b 4c 24 50       	mov    0x50(%rsp),%rcx
    d7e4:	48 89 44 24 48       	mov    %rax,0x48(%rsp)
    d7e9:	4c 89 fa             	mov    %r15,%rdx
    d7ec:	4c 89 f6             	mov    %r14,%rsi
    d7ef:	bf 04 00 00 00       	mov    $0x4,%edi
    d7f4:	ff d1                	call   *%rcx
    d7f6:	48 8b 44 24 50       	mov    0x50(%rsp),%rax
    d7fb:	48 c7 85 c8 00 00 00 	movq   $0x0,0xc8(%rbp)
    d802:	00 00 00 00 
    d806:	48 c7 85 d0 00 00 00 	movq   $0x0,0xd0(%rbp)
    d80d:	00 00 00 00 
    d811:	48 85 c0             	test   %rax,%rax
    d814:	74 0c                	je     d822 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0x4f2>
    d816:	31 d2                	xor    %edx,%edx
    d818:	4c 89 f6             	mov    %r14,%rsi
    d81b:	bf 03 00 00 00       	mov    $0x3,%edi
    d820:	ff d0                	call   *%rax
    d822:	48 8b 7c 24 70       	mov    0x70(%rsp),%rdi
    d827:	4c 39 ef             	cmp    %r13,%rdi
    d82a:	0f 84 6e fc ff ff    	je     d49e <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0x16e>
    d830:	48 8b 84 24 80 00 00 	mov    0x80(%rsp),%rax
    d837:	00 
    d838:	48 8d 70 01          	lea    0x1(%rax),%rsi
    d83c:	e8 df 5a ff ff       	call   3320 <_ZdlPvm@plt>
    d841:	41 83 e4 02          	and    $0x2,%r12d
    d845:	0f 84 5d fc ff ff    	je     d4a8 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0x178>
    d84b:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)
    d850:	48 8d 15 80 2b 00 00 	lea    0x2b80(%rip),%rdx        # 103d7 <_fini+0x11b6>
    d857:	48 8d 35 83 2b 00 00 	lea    0x2b83(%rip),%rsi        # 103e1 <_fini+0x11c0>
    d85e:	48 89 df             	mov    %rbx,%rdi
    d861:	e8 da f2 ff ff       	call   cb40 <_ZN8argparse14ArgumentParser12add_argumentIJPKcS3_EEERNS_8ArgumentEDpT_>
    d866:	48 8d 0d 03 8b ff ff 	lea    -0x74fd(%rip),%rcx        # 6370 <_ZNSt17_Function_handlerIFvRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEZN8argparse14ArgumentParserC4ES5_S5_NS9_17default_argumentsEEUlRKT_E0_E10_M_managerERSt9_Any_dataRKSH_St18_Manager_operation>
    d86d:	48 89 5c 24 70       	mov    %rbx,0x70(%rsp)
    d872:	c6 84 24 90 00 00 00 	movb   $0x1,0x90(%rsp)
    d879:	01 
    d87a:	48 8d 15 df 8c ff ff 	lea    -0x7321(%rip),%rdx        # 6560 <_ZNSt17_Function_handlerIFvRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEZN8argparse14ArgumentParserC4ES5_S5_NS9_17default_argumentsEEUlRKT_E0_E9_M_invokeERKSt9_Any_dataS7_>
    d881:	c4 e1 f9 6e d1       	vmovq  %rcx,%xmm2
    d886:	c4 e3 e9 22 c2 01    	vpinsrq $0x1,%rdx,%xmm2,%xmm0
    d88c:	c5 f9 7f 84 24 80 00 	vmovdqa %xmm0,0x80(%rsp)
    d893:	00 00 
    d895:	48 89 c5             	mov    %rax,%rbp
    d898:	0f b6 80 a8 00 00 00 	movzbl 0xa8(%rax),%eax
    d89f:	3c 01                	cmp    $0x1,%al
    d8a1:	0f 84 79 03 00 00    	je     dc20 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0x8f0>
    d8a7:	4c 8d 7c 24 48       	lea    0x48(%rsp),%r15
    d8ac:	4c 8d 25 dd 5f 00 00 	lea    0x5fdd(%rip),%r12        # 13890 <_ZNSt8__detail9__variant12__gen_vtableIvOZNS0_16_Variant_storageILb0EJSt8functionIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEES3_IFvSC_EEEE8_M_resetEvEUlOT_E_JRSt7variantIJSE_SG_EEEE9_S_vtableE>
    d8b3:	48 8d b5 88 00 00 00 	lea    0x88(%rbp),%rsi
    d8ba:	4c 89 ff             	mov    %r15,%rdi
    d8bd:	41 ff 14 c4          	call   *(%r12,%rax,8)
    d8c1:	c6 85 a8 00 00 00 01 	movb   $0x1,0xa8(%rbp)
    d8c8:	c5 fa 6f bd 88 00 00 	vmovdqu 0x88(%rbp),%xmm7
    d8cf:	00 
    d8d0:	c5 f9 6f 44 24 70    	vmovdqa 0x70(%rsp),%xmm0
    d8d6:	c5 f9 7f 7c 24 70    	vmovdqa %xmm7,0x70(%rsp)
    d8dc:	c5 fa 7f 85 88 00 00 	vmovdqu %xmm0,0x88(%rbp)
    d8e3:	00 
    d8e4:	48 8b 84 24 80 00 00 	mov    0x80(%rsp),%rax
    d8eb:	00 
    d8ec:	48 8b 95 a0 00 00 00 	mov    0xa0(%rbp),%rdx
    d8f3:	48 c7 84 24 80 00 00 	movq   $0x0,0x80(%rsp)
    d8fa:	00 00 00 00 00 
    d8ff:	48 89 85 98 00 00 00 	mov    %rax,0x98(%rbp)
    d906:	48 8b 84 24 88 00 00 	mov    0x88(%rsp),%rax
    d90d:	00 
    d90e:	48 89 94 24 88 00 00 	mov    %rdx,0x88(%rsp)
    d915:	00 
    d916:	48 89 85 a0 00 00 00 	mov    %rax,0xa0(%rbp)
    d91d:	0f b6 84 24 90 00 00 	movzbl 0x90(%rsp),%eax
    d924:	00 
    d925:	49 8b 04 c4          	mov    (%r12,%rax,8),%rax
    d929:	4c 8d 74 24 70       	lea    0x70(%rsp),%r14
    d92e:	4c 89 f6             	mov    %r14,%rsi
    d931:	4c 89 ff             	mov    %r15,%rdi
    d934:	ff d0                	call   *%rax
    d936:	80 bd a8 00 00 00 01 	cmpb   $0x1,0xa8(%rbp)
    d93d:	0f 85 47 05 00 00    	jne    de8a <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0xb5a>
    d943:	4c 8d ac 24 80 00 00 	lea    0x80(%rsp),%r13
    d94a:	00 
    d94b:	4c 89 6c 24 70       	mov    %r13,0x70(%rsp)
    d950:	c6 84 24 84 00 00 00 	movb   $0x65,0x84(%rsp)
    d957:	65 
    d958:	c7 84 24 80 00 00 00 	movl   $0x736c6166,0x80(%rsp)
    d95f:	66 61 6c 73 
    d963:	c6 84 24 85 00 00 00 	movb   $0x0,0x85(%rsp)
    d96a:	00 
    d96b:	48 c7 44 24 78 05 00 	movq   $0x5,0x78(%rsp)
    d972:	00 00 
    d974:	48 8b 45 58          	mov    0x58(%rbp),%rax
    d978:	c7 00 66 61 6c 73    	movl   $0x736c6166,(%rax)
    d97e:	0f b6 94 24 84 00 00 	movzbl 0x84(%rsp),%edx
    d985:	00 
    d986:	88 50 04             	mov    %dl,0x4(%rax)
    d989:	48 8b 44 24 78       	mov    0x78(%rsp),%rax
    d98e:	48 8b 55 58          	mov    0x58(%rbp),%rdx
    d992:	48 89 45 60          	mov    %rax,0x60(%rbp)
    d996:	c6 04 02 00          	movb   $0x0,(%rdx,%rax,1)
    d99a:	48 c7 44 24 78 00 00 	movq   $0x0,0x78(%rsp)
    d9a1:	00 00 
    d9a3:	48 8b 44 24 70       	mov    0x70(%rsp),%rax
    d9a8:	c6 00 00             	movb   $0x0,(%rax)
    d9ab:	48 8b 7c 24 70       	mov    0x70(%rsp),%rdi
    d9b0:	4c 39 ef             	cmp    %r13,%rdi
    d9b3:	74 11                	je     d9c6 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0x696>
    d9b5:	48 8b 84 24 80 00 00 	mov    0x80(%rsp),%rax
    d9bc:	00 
    d9bd:	48 8d 70 01          	lea    0x1(%rax),%rsi
    d9c1:	e8 5a 59 ff ff       	call   3320 <_ZdlPvm@plt>
    d9c6:	48 8d 05 63 87 ff ff 	lea    -0x789d(%rip),%rax        # 6130 <_ZNSt3any17_Manager_internalIbE9_S_manageENS_3_OpEPKS_PNS_4_ArgE>
    d9cd:	4c 8d 65 48          	lea    0x48(%rbp),%r12
    d9d1:	48 89 04 24          	mov    %rax,(%rsp)
    d9d5:	48 89 44 24 70       	mov    %rax,0x70(%rsp)
    d9da:	48 c7 44 24 78 00 00 	movq   $0x0,0x78(%rsp)
    d9e1:	00 00 
    d9e3:	4d 39 f4             	cmp    %r14,%r12
    d9e6:	74 3e                	je     da26 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0x6f6>
    d9e8:	48 8b 4d 48          	mov    0x48(%rbp),%rcx
    d9ec:	48 85 c9             	test   %rcx,%rcx
    d9ef:	74 19                	je     da0a <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0x6da>
    d9f1:	31 d2                	xor    %edx,%edx
    d9f3:	4c 89 e6             	mov    %r12,%rsi
    d9f6:	bf 03 00 00 00       	mov    $0x3,%edi
    d9fb:	ff d1                	call   *%rcx
    d9fd:	48 c7 45 48 00 00 00 	movq   $0x0,0x48(%rbp)
    da04:	00 
    da05:	48 8b 44 24 70       	mov    0x70(%rsp),%rax
    da0a:	4c 89 64 24 48       	mov    %r12,0x48(%rsp)
    da0f:	4c 89 fa             	mov    %r15,%rdx
    da12:	4c 89 f6             	mov    %r14,%rsi
    da15:	bf 04 00 00 00       	mov    $0x4,%edi
    da1a:	ff d0                	call   *%rax
    da1c:	48 8b 44 24 70       	mov    0x70(%rsp),%rax
    da21:	48 85 c0             	test   %rax,%rax
    da24:	74 0c                	je     da32 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0x702>
    da26:	31 d2                	xor    %edx,%edx
    da28:	4c 89 f6             	mov    %r14,%rsi
    da2b:	bf 03 00 00 00       	mov    $0x3,%edi
    da30:	ff d0                	call   *%rax
    da32:	31 d2                	xor    %edx,%edx
    da34:	4c 89 fe             	mov    %r15,%rsi
    da37:	4c 89 f7             	mov    %r14,%rdi
    da3a:	4c 89 6c 24 70       	mov    %r13,0x70(%rsp)
    da3f:	48 c7 44 24 48 24 00 	movq   $0x24,0x48(%rsp)
    da46:	00 00 
    da48:	e8 b3 58 ff ff       	call   3300 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE9_M_createERmm@plt>
    da4d:	48 8b 54 24 48       	mov    0x48(%rsp),%rdx
    da52:	c5 f9 6f 05 96 2d 00 	vmovdqa 0x2d96(%rip),%xmm0        # 107f0 <_ZTSN8argparse7details12parse_numberIfLNS0_12chars_formatE3EEE+0x90>
    da59:	00 
    da5a:	48 89 44 24 70       	mov    %rax,0x70(%rsp)
    da5f:	48 89 94 24 80 00 00 	mov    %rdx,0x80(%rsp)
    da66:	00 
    da67:	c5 fa 7f 00          	vmovdqu %xmm0,(%rax)
    da6b:	c7 40 20 78 69 74 73 	movl   $0x73746978,0x20(%rax)
    da72:	c5 f9 6f 05 86 2d 00 	vmovdqa 0x2d86(%rip),%xmm0        # 10800 <_ZTSN8argparse7details12parse_numberIfLNS0_12chars_formatE3EEE+0xa0>
    da79:	00 
    da7a:	c5 fa 7f 40 10       	vmovdqu %xmm0,0x10(%rax)
    da7f:	48 8b 44 24 48       	mov    0x48(%rsp),%rax
    da84:	48 8b 54 24 70       	mov    0x70(%rsp),%rdx
    da89:	48 89 44 24 78       	mov    %rax,0x78(%rsp)
    da8e:	c6 04 02 00          	movb   $0x0,(%rdx,%rax,1)
    da92:	48 8b 54 24 70       	mov    0x70(%rsp),%rdx
    da97:	48 8b 7d 28          	mov    0x28(%rbp),%rdi
    da9b:	4c 39 ea             	cmp    %r13,%rdx
    da9e:	0f 84 74 03 00 00    	je     de18 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0xae8>
    daa4:	48 8d 4d 38          	lea    0x38(%rbp),%rcx
    daa8:	48 8b b4 24 80 00 00 	mov    0x80(%rsp),%rsi
    daaf:	00 
    dab0:	c5 fa 7e 44 24 78    	vmovq  0x78(%rsp),%xmm0
    dab6:	48 39 cf             	cmp    %rcx,%rdi
    dab9:	0f 84 39 01 00 00    	je     dbf8 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0x8c8>
    dabf:	c4 e3 f9 22 c6 01    	vpinsrq $0x1,%rsi,%xmm0,%xmm0
    dac5:	48 8b 4d 38          	mov    0x38(%rbp),%rcx
    dac9:	48 89 55 28          	mov    %rdx,0x28(%rbp)
    dacd:	c5 fa 7f 45 30       	vmovdqu %xmm0,0x30(%rbp)
    dad2:	48 85 ff             	test   %rdi,%rdi
    dad5:	0f 84 2c 01 00 00    	je     dc07 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0x8d7>
    dadb:	48 89 7c 24 70       	mov    %rdi,0x70(%rsp)
    dae0:	48 89 8c 24 80 00 00 	mov    %rcx,0x80(%rsp)
    dae7:	00 
    dae8:	48 8b 04 24          	mov    (%rsp),%rax
    daec:	48 8d 5d 78          	lea    0x78(%rbp),%rbx
    daf0:	4c 8d 74 24 50       	lea    0x50(%rsp),%r14
    daf5:	48 c7 44 24 78 00 00 	movq   $0x0,0x78(%rsp)
    dafc:	00 00 
    dafe:	c6 07 00             	movb   $0x0,(%rdi)
    db01:	48 89 44 24 50       	mov    %rax,0x50(%rsp)
    db06:	48 c7 44 24 58 01 00 	movq   $0x1,0x58(%rsp)
    db0d:	00 00 
    db0f:	4c 39 f3             	cmp    %r14,%rbx
    db12:	0f 84 d8 02 00 00    	je     ddf0 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0xac0>
    db18:	48 8b 4d 78          	mov    0x78(%rbp),%rcx
    db1c:	48 8d 05 0d 86 ff ff 	lea    -0x79f3(%rip),%rax        # 6130 <_ZNSt3any17_Manager_internalIbE9_S_manageENS_3_OpEPKS_PNS_4_ArgE>
    db23:	48 85 c9             	test   %rcx,%rcx
    db26:	74 19                	je     db41 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0x811>
    db28:	31 d2                	xor    %edx,%edx
    db2a:	48 89 de             	mov    %rbx,%rsi
    db2d:	bf 03 00 00 00       	mov    $0x3,%edi
    db32:	ff d1                	call   *%rcx
    db34:	48 c7 45 78 00 00 00 	movq   $0x0,0x78(%rbp)
    db3b:	00 
    db3c:	48 8b 44 24 50       	mov    0x50(%rsp),%rax
    db41:	48 89 5c 24 48       	mov    %rbx,0x48(%rsp)
    db46:	4c 89 fa             	mov    %r15,%rdx
    db49:	4c 89 f6             	mov    %r14,%rsi
    db4c:	bf 04 00 00 00       	mov    $0x4,%edi
    db51:	ff d0                	call   *%rax
    db53:	48 8b 44 24 50       	mov    0x50(%rsp),%rax
    db58:	48 c7 85 c8 00 00 00 	movq   $0x0,0xc8(%rbp)
    db5f:	00 00 00 00 
    db63:	48 c7 85 d0 00 00 00 	movq   $0x0,0xd0(%rbp)
    db6a:	00 00 00 00 
    db6e:	48 85 c0             	test   %rax,%rax
    db71:	74 0c                	je     db7f <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0x84f>
    db73:	31 d2                	xor    %edx,%edx
    db75:	4c 89 f6             	mov    %r14,%rsi
    db78:	bf 03 00 00 00       	mov    $0x3,%edi
    db7d:	ff d0                	call   *%rax
    db7f:	48 8b 7c 24 70       	mov    0x70(%rsp),%rdi
    db84:	4c 39 ef             	cmp    %r13,%rdi
    db87:	0f 84 1b f9 ff ff    	je     d4a8 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0x178>
    db8d:	48 8b 84 24 80 00 00 	mov    0x80(%rsp),%rax
    db94:	00 
    db95:	48 8d 70 01          	lea    0x1(%rax),%rsi
    db99:	e8 82 57 ff ff       	call   3320 <_ZdlPvm@plt>
    db9e:	e9 05 f9 ff ff       	jmp    d4a8 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0x178>
    dba3:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)
    dba8:	c5 fa 6f 72 10       	vmovdqu 0x10(%rdx),%xmm6
    dbad:	c5 fa 7f 73 30       	vmovdqu %xmm6,0x30(%rbx)
    dbb2:	e9 0a f8 ff ff       	jmp    d3c1 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0x91>
    dbb7:	66 0f 1f 84 00 00 00 	nopw   0x0(%rax,%rax,1)
    dbbe:	00 00 
    dbc0:	c5 fa 6f 6e 10       	vmovdqu 0x10(%rsi),%xmm5
    dbc5:	c5 fa 7f 6f 10       	vmovdqu %xmm5,0x10(%rdi)
    dbca:	e9 b2 f7 ff ff       	jmp    d381 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0x51>
    dbcf:	90                   	nop
    dbd0:	48 89 55 28          	mov    %rdx,0x28(%rbp)
    dbd4:	c4 e3 f9 22 c6 01    	vpinsrq $0x1,%rsi,%xmm0,%xmm0
    dbda:	c5 fa 7f 45 30       	vmovdqu %xmm0,0x30(%rbp)
    dbdf:	4c 89 6c 24 70       	mov    %r13,0x70(%rsp)
    dbe4:	4c 8d ac 24 80 00 00 	lea    0x80(%rsp),%r13
    dbeb:	00 
    dbec:	4c 89 ef             	mov    %r13,%rdi
    dbef:	e9 8e fb ff ff       	jmp    d782 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0x452>
    dbf4:	0f 1f 40 00          	nopl   0x0(%rax)
    dbf8:	48 89 55 28          	mov    %rdx,0x28(%rbp)
    dbfc:	c4 e3 f9 22 c6 01    	vpinsrq $0x1,%rsi,%xmm0,%xmm0
    dc02:	c5 fa 7f 45 30       	vmovdqu %xmm0,0x30(%rbp)
    dc07:	4c 89 6c 24 70       	mov    %r13,0x70(%rsp)
    dc0c:	4c 8d ac 24 80 00 00 	lea    0x80(%rsp),%r13
    dc13:	00 
    dc14:	4c 89 ef             	mov    %r13,%rdi
    dc17:	e9 cc fe ff ff       	jmp    dae8 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0x7b8>
    dc1c:	0f 1f 40 00          	nopl   0x0(%rax)
    dc20:	48 8b 44 24 68       	mov    0x68(%rsp),%rax
    dc25:	c5 f9 6f 4c 24 50    	vmovdqa 0x50(%rsp),%xmm1
    dc2b:	48 c7 84 24 80 00 00 	movq   $0x0,0x80(%rsp)
    dc32:	00 00 00 00 00 
    dc37:	48 89 84 24 88 00 00 	mov    %rax,0x88(%rsp)
    dc3e:	00 
    dc3f:	c5 f9 6f 44 24 70    	vmovdqa 0x70(%rsp),%xmm0
    dc45:	c5 f9 7f 4c 24 70    	vmovdqa %xmm1,0x70(%rsp)
    dc4b:	c5 f9 7f 44 24 50    	vmovdqa %xmm0,0x50(%rsp)
    dc51:	c5 fa 6f 95 88 00 00 	vmovdqu 0x88(%rbp),%xmm2
    dc58:	00 
    dc59:	4c 8d 7c 24 48       	lea    0x48(%rsp),%r15
    dc5e:	c5 f9 7f 54 24 50    	vmovdqa %xmm2,0x50(%rsp)
    dc64:	4c 8b 85 98 00 00 00 	mov    0x98(%rbp),%r8
    dc6b:	48 8b 85 a0 00 00 00 	mov    0xa0(%rbp),%rax
    dc72:	c5 fa 7f 85 88 00 00 	vmovdqu %xmm0,0x88(%rbp)
    dc79:	00 
    dc7a:	4c 89 44 24 60       	mov    %r8,0x60(%rsp)
    dc7f:	48 89 8d 98 00 00 00 	mov    %rcx,0x98(%rbp)
    dc86:	48 89 44 24 68       	mov    %rax,0x68(%rsp)
    dc8b:	48 89 95 a0 00 00 00 	mov    %rdx,0xa0(%rbp)
    dc92:	48 8d 05 37 85 ff ff 	lea    -0x7ac9(%rip),%rax        # 61d0 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFvOZNS0_16_Variant_storageILb0EJSt8functionIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEES4_IFvSD_EEEE8_M_resetEvEUlOT_E_RSt7variantIJSF_SH_EEEJEEESt16integer_sequenceImJLm1EEEE14__visit_invokeESM_SP_>
    dc99:	4d 85 c0             	test   %r8,%r8
    dc9c:	0f 84 87 fc ff ff    	je     d929 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0x5f9>
    dca2:	48 8d 7c 24 50       	lea    0x50(%rsp),%rdi
    dca7:	ba 03 00 00 00       	mov    $0x3,%edx
    dcac:	48 89 fe             	mov    %rdi,%rsi
    dcaf:	41 ff d0             	call   *%r8
    dcb2:	0f b6 94 24 90 00 00 	movzbl 0x90(%rsp),%edx
    dcb9:	00 
    dcba:	48 8d 05 cf 5b 00 00 	lea    0x5bcf(%rip),%rax        # 13890 <_ZNSt8__detail9__variant12__gen_vtableIvOZNS0_16_Variant_storageILb0EJSt8functionIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEES3_IFvSC_EEEE8_M_resetEvEUlOT_E_JRSt7variantIJSE_SG_EEEE9_S_vtableE>
    dcc1:	48 8b 04 d0          	mov    (%rax,%rdx,8),%rax
    dcc5:	e9 5f fc ff ff       	jmp    d929 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0x5f9>
    dcca:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)
    dcd0:	48 8b 44 24 68       	mov    0x68(%rsp),%rax
    dcd5:	c5 f9 6f 6c 24 50    	vmovdqa 0x50(%rsp),%xmm5
    dcdb:	48 c7 84 24 80 00 00 	movq   $0x0,0x80(%rsp)
    dce2:	00 00 00 00 00 
    dce7:	48 89 84 24 88 00 00 	mov    %rax,0x88(%rsp)
    dcee:	00 
    dcef:	c5 f9 6f 44 24 70    	vmovdqa 0x70(%rsp),%xmm0
    dcf5:	c5 f9 7f 6c 24 70    	vmovdqa %xmm5,0x70(%rsp)
    dcfb:	c5 f9 7f 44 24 50    	vmovdqa %xmm0,0x50(%rsp)
    dd01:	c5 fa 6f b5 88 00 00 	vmovdqu 0x88(%rbp),%xmm6
    dd08:	00 
    dd09:	4c 8d 7c 24 48       	lea    0x48(%rsp),%r15
    dd0e:	c5 f9 7f 74 24 50    	vmovdqa %xmm6,0x50(%rsp)
    dd14:	4c 8b 85 98 00 00 00 	mov    0x98(%rbp),%r8
    dd1b:	48 8b 85 a0 00 00 00 	mov    0xa0(%rbp),%rax
    dd22:	c5 fa 7f 85 88 00 00 	vmovdqu %xmm0,0x88(%rbp)
    dd29:	00 
    dd2a:	4c 89 44 24 60       	mov    %r8,0x60(%rsp)
    dd2f:	48 89 8d 98 00 00 00 	mov    %rcx,0x98(%rbp)
    dd36:	48 89 44 24 68       	mov    %rax,0x68(%rsp)
    dd3b:	48 89 95 a0 00 00 00 	mov    %rdx,0xa0(%rbp)
    dd42:	48 8d 05 87 84 ff ff 	lea    -0x7b79(%rip),%rax        # 61d0 <_ZNSt8__detail9__variant17__gen_vtable_implINS0_12_Multi_arrayIPFvOZNS0_16_Variant_storageILb0EJSt8functionIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEES4_IFvSD_EEEE8_M_resetEvEUlOT_E_RSt7variantIJSF_SH_EEEJEEESt16integer_sequenceImJLm1EEEE14__visit_invokeESM_SP_>
    dd49:	4d 85 c0             	test   %r8,%r8
    dd4c:	0f 84 61 f8 ff ff    	je     d5b3 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0x283>
    dd52:	48 8d 7c 24 50       	lea    0x50(%rsp),%rdi
    dd57:	ba 03 00 00 00       	mov    $0x3,%edx
    dd5c:	48 89 fe             	mov    %rdi,%rsi
    dd5f:	41 ff d0             	call   *%r8
    dd62:	0f b6 94 24 90 00 00 	movzbl 0x90(%rsp),%edx
    dd69:	00 
    dd6a:	48 8d 05 1f 5b 00 00 	lea    0x5b1f(%rip),%rax        # 13890 <_ZNSt8__detail9__variant12__gen_vtableIvOZNS0_16_Variant_storageILb0EJSt8functionIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEES3_IFvSC_EEEE8_M_resetEvEUlOT_E_JRSt7variantIJSE_SG_EEEE9_S_vtableE>
    dd71:	48 8b 04 d0          	mov    (%rax,%rdx,8),%rax
    dd75:	e9 39 f8 ff ff       	jmp    d5b3 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0x283>
    dd7a:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)
    dd80:	48 c7 85 c8 00 00 00 	movq   $0x0,0xc8(%rbp)
    dd87:	00 00 00 00 
    dd8b:	48 c7 85 d0 00 00 00 	movq   $0x0,0xd0(%rbp)
    dd92:	00 00 00 00 
    dd96:	48 8d 05 93 83 ff ff 	lea    -0x7c6d(%rip),%rax        # 6130 <_ZNSt3any17_Manager_internalIbE9_S_manageENS_3_OpEPKS_PNS_4_ArgE>
    dd9d:	e9 74 fa ff ff       	jmp    d816 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0x4e6>
    dda2:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)
    dda8:	48 89 c8             	mov    %rcx,%rax
    ddab:	e9 0f f9 ff ff       	jmp    d6bf <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0x38f>
    ddb0:	48 8b 54 24 78       	mov    0x78(%rsp),%rdx
    ddb5:	48 85 d2             	test   %rdx,%rdx
    ddb8:	74 1b                	je     ddd5 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0xaa5>
    ddba:	48 83 fa 01          	cmp    $0x1,%rdx
    ddbe:	0f 84 a4 00 00 00    	je     de68 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0xb38>
    ddc4:	4c 89 ee             	mov    %r13,%rsi
    ddc7:	e8 74 53 ff ff       	call   3140 <memcpy@plt>
    ddcc:	48 8b 54 24 78       	mov    0x78(%rsp),%rdx
    ddd1:	48 8b 7d 28          	mov    0x28(%rbp),%rdi
    ddd5:	48 89 55 30          	mov    %rdx,0x30(%rbp)
    ddd9:	c6 04 17 00          	movb   $0x0,(%rdi,%rdx,1)
    dddd:	48 8b 7c 24 70       	mov    0x70(%rsp),%rdi
    dde2:	e9 9b f9 ff ff       	jmp    d782 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0x452>
    dde7:	66 0f 1f 84 00 00 00 	nopw   0x0(%rax,%rax,1)
    ddee:	00 00 
    ddf0:	48 c7 85 c8 00 00 00 	movq   $0x0,0xc8(%rbp)
    ddf7:	00 00 00 00 
    ddfb:	48 c7 85 d0 00 00 00 	movq   $0x0,0xd0(%rbp)
    de02:	00 00 00 00 
    de06:	48 8d 05 23 83 ff ff 	lea    -0x7cdd(%rip),%rax        # 6130 <_ZNSt3any17_Manager_internalIbE9_S_manageENS_3_OpEPKS_PNS_4_ArgE>
    de0d:	e9 61 fd ff ff       	jmp    db73 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0x843>
    de12:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)
    de18:	48 8b 54 24 78       	mov    0x78(%rsp),%rdx
    de1d:	48 85 d2             	test   %rdx,%rdx
    de20:	74 17                	je     de39 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0xb09>
    de22:	48 83 fa 01          	cmp    $0x1,%rdx
    de26:	74 28                	je     de50 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0xb20>
    de28:	4c 89 ee             	mov    %r13,%rsi
    de2b:	e8 10 53 ff ff       	call   3140 <memcpy@plt>
    de30:	48 8b 54 24 78       	mov    0x78(%rsp),%rdx
    de35:	48 8b 7d 28          	mov    0x28(%rbp),%rdi
    de39:	48 89 55 30          	mov    %rdx,0x30(%rbp)
    de3d:	c6 04 17 00          	movb   $0x0,(%rdi,%rdx,1)
    de41:	48 8b 7c 24 70       	mov    0x70(%rsp),%rdi
    de46:	e9 9d fc ff ff       	jmp    dae8 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0x7b8>
    de4b:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)
    de50:	0f b6 84 24 80 00 00 	movzbl 0x80(%rsp),%eax
    de57:	00 
    de58:	88 07                	mov    %al,(%rdi)
    de5a:	48 8b 54 24 78       	mov    0x78(%rsp),%rdx
    de5f:	48 8b 7d 28          	mov    0x28(%rbp),%rdi
    de63:	eb d4                	jmp    de39 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0xb09>
    de65:	0f 1f 00             	nopl   (%rax)
    de68:	0f b6 84 24 80 00 00 	movzbl 0x80(%rsp),%eax
    de6f:	00 
    de70:	88 07                	mov    %al,(%rdi)
    de72:	48 8b 54 24 78       	mov    0x78(%rsp),%rdx
    de77:	48 8b 7d 28          	mov    0x28(%rbp),%rdi
    de7b:	e9 55 ff ff ff       	jmp    ddd5 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0xaa5>
    de80:	e8 cb 52 ff ff       	call   3150 <__stack_chk_fail@plt>
    de85:	48 89 c5             	mov    %rax,%rbp
    de88:	eb 0e                	jmp    de98 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0xb68>
    de8a:	31 ff                	xor    %edi,%edi
    de8c:	e8 91 55 ff ff       	call   3422 <_ZSt26__throw_bad_variant_accessb>
    de91:	31 ff                	xor    %edi,%edi
    de93:	e8 8a 55 ff ff       	call   3422 <_ZSt26__throw_bad_variant_accessb>
    de98:	48 8b bb c8 00 00 00 	mov    0xc8(%rbx),%rdi
    de9f:	c5 f8 77             	vzeroupper 
    dea2:	e8 79 68 ff ff       	call   4720 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE8_M_eraseEPSt13_Rb_tree_nodeISA_E.isra.0>
    dea7:	48 8b 7c 24 28       	mov    0x28(%rsp),%rdi
    deac:	e8 ff b0 ff ff       	call   8fb0 <_ZNSt7__cxx1110_List_baseIN8argparse8ArgumentESaIS2_EE8_M_clearEv>
    deb1:	48 8b 7c 24 20       	mov    0x20(%rsp),%rdi
    deb6:	e8 f5 b0 ff ff       	call   8fb0 <_ZNSt7__cxx1110_List_baseIN8argparse8ArgumentESaIS2_EE8_M_clearEv>
    debb:	48 8b 7b 60          	mov    0x60(%rbx),%rdi
    debf:	48 39 7c 24 18       	cmp    %rdi,0x18(%rsp)
    dec4:	74 0d                	je     ded3 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0xba3>
    dec6:	48 8b 43 70          	mov    0x70(%rbx),%rax
    deca:	48 8d 70 01          	lea    0x1(%rax),%rsi
    dece:	e8 4d 54 ff ff       	call   3320 <_ZdlPvm@plt>
    ded3:	48 8b 7b 40          	mov    0x40(%rbx),%rdi
    ded7:	48 39 7c 24 10       	cmp    %rdi,0x10(%rsp)
    dedc:	74 0d                	je     deeb <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0xbbb>
    dede:	48 8b 43 50          	mov    0x50(%rbx),%rax
    dee2:	48 8d 70 01          	lea    0x1(%rax),%rsi
    dee6:	e8 35 54 ff ff       	call   3320 <_ZdlPvm@plt>
    deeb:	48 8b 7b 20          	mov    0x20(%rbx),%rdi
    deef:	48 39 7c 24 08       	cmp    %rdi,0x8(%rsp)
    def4:	74 0d                	je     df03 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0xbd3>
    def6:	48 8b 43 30          	mov    0x30(%rbx),%rax
    defa:	48 8d 70 01          	lea    0x1(%rax),%rsi
    defe:	e8 1d 54 ff ff       	call   3320 <_ZdlPvm@plt>
    df03:	48 8b 3b             	mov    (%rbx),%rdi
    df06:	48 39 7c 24 30       	cmp    %rdi,0x30(%rsp)
    df0b:	74 0c                	je     df19 <_ZN8argparse14ArgumentParserC1ENSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES6_NS_17default_argumentsE+0xbe9>
    df0d:	48 8b 73 10          	mov    0x10(%rbx),%rsi
    df11:	48 ff c6             	inc    %rsi
    df14:	e8 07 54 ff ff       	call   3320 <_ZdlPvm@plt>
    df19:	48 89 ef             	mov    %rbp,%rdi
    df1c:	e8 3f 54 ff ff       	call   3360 <_Unwind_Resume@plt>
    df21:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
    df28:	00 00 00 
    df2b:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)

000000000000df30 <_ZSt11__make_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_RSS_>:
    df30:	41 57                	push   %r15
    df32:	48 29 fe             	sub    %rdi,%rsi
    df35:	41 56                	push   %r14
    df37:	41 55                	push   %r13
    df39:	41 54                	push   %r12
    df3b:	55                   	push   %rbp
    df3c:	53                   	push   %rbx
    df3d:	48 83 ec 68          	sub    $0x68,%rsp
    df41:	64 48 8b 04 25 28 00 	mov    %fs:0x28,%rax
    df48:	00 00 
    df4a:	48 89 44 24 58       	mov    %rax,0x58(%rsp)
    df4f:	31 c0                	xor    %eax,%eax
    df51:	48 83 fe 20          	cmp    $0x20,%rsi
    df55:	0f 8e 40 01 00 00    	jle    e09b <_ZSt11__make_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_RSS_+0x16b>
    df5b:	49 89 f4             	mov    %rsi,%r12
    df5e:	49 c1 fc 05          	sar    $0x5,%r12
    df62:	49 8d 44 24 fe       	lea    -0x2(%r12),%rax
    df67:	48 89 c3             	mov    %rax,%rbx
    df6a:	48 c1 eb 3f          	shr    $0x3f,%rbx
    df6e:	48 01 c3             	add    %rax,%rbx
    df71:	48 d1 fb             	sar    %rbx
    df74:	48 89 d8             	mov    %rbx,%rax
    df77:	48 c1 e0 05          	shl    $0x5,%rax
    df7b:	4c 8d 7c 07 10       	lea    0x10(%rdi,%rax,1),%r15
    df80:	48 8d 44 24 30       	lea    0x30(%rsp),%rax
    df85:	48 89 44 24 08       	mov    %rax,0x8(%rsp)
    df8a:	49 89 fe             	mov    %rdi,%r14
    df8d:	48 8d 6c 24 20       	lea    0x20(%rsp),%rbp
    df92:	4c 8d 6c 24 40       	lea    0x40(%rsp),%r13
    df97:	e9 a4 00 00 00       	jmp    e040 <_ZSt11__make_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_RSS_+0x110>
    df9c:	0f 1f 40 00          	nopl   0x0(%rax)
    dfa0:	49 8b 0f             	mov    (%r15),%rcx
    dfa3:	49 8b 77 f8          	mov    -0x8(%r15),%rsi
    dfa7:	48 89 54 24 10       	mov    %rdx,0x10(%rsp)
    dfac:	48 89 4c 24 20       	mov    %rcx,0x20(%rsp)
    dfb1:	4d 89 7f f0          	mov    %r15,-0x10(%r15)
    dfb5:	49 c7 47 f8 00 00 00 	movq   $0x0,-0x8(%r15)
    dfbc:	00 
    dfbd:	41 c6 07 00          	movb   $0x0,(%r15)
    dfc1:	4c 89 6c 24 30       	mov    %r13,0x30(%rsp)
    dfc6:	48 39 ea             	cmp    %rbp,%rdx
    dfc9:	0f 84 a3 00 00 00    	je     e072 <_ZSt11__make_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_RSS_+0x142>
    dfcf:	48 89 54 24 30       	mov    %rdx,0x30(%rsp)
    dfd4:	48 89 4c 24 40       	mov    %rcx,0x40(%rsp)
    dfd9:	48 8b 4c 24 08       	mov    0x8(%rsp),%rcx
    dfde:	48 89 74 24 38       	mov    %rsi,0x38(%rsp)
    dfe3:	4c 89 f7             	mov    %r14,%rdi
    dfe6:	4c 89 e2             	mov    %r12,%rdx
    dfe9:	48 89 de             	mov    %rbx,%rsi
    dfec:	48 89 6c 24 10       	mov    %rbp,0x10(%rsp)
    dff1:	48 c7 44 24 18 00 00 	movq   $0x0,0x18(%rsp)
    dff8:	00 00 
    dffa:	c6 44 24 20 00       	movb   $0x0,0x20(%rsp)
    dfff:	e8 ac 69 ff ff       	call   49b0 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0>
    e004:	48 8b 7c 24 30       	mov    0x30(%rsp),%rdi
    e009:	4c 39 ef             	cmp    %r13,%rdi
    e00c:	74 0e                	je     e01c <_ZSt11__make_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_RSS_+0xec>
    e00e:	48 8b 44 24 40       	mov    0x40(%rsp),%rax
    e013:	48 8d 70 01          	lea    0x1(%rax),%rsi
    e017:	e8 04 53 ff ff       	call   3320 <_ZdlPvm@plt>
    e01c:	48 8b 7c 24 10       	mov    0x10(%rsp),%rdi
    e021:	48 85 db             	test   %rbx,%rbx
    e024:	74 62                	je     e088 <_ZSt11__make_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_RSS_+0x158>
    e026:	48 ff cb             	dec    %rbx
    e029:	48 39 ef             	cmp    %rbp,%rdi
    e02c:	74 0e                	je     e03c <_ZSt11__make_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_RSS_+0x10c>
    e02e:	48 8b 44 24 20       	mov    0x20(%rsp),%rax
    e033:	48 8d 70 01          	lea    0x1(%rax),%rsi
    e037:	e8 e4 52 ff ff       	call   3320 <_ZdlPvm@plt>
    e03c:	49 83 ef 20          	sub    $0x20,%r15
    e040:	49 8b 57 f0          	mov    -0x10(%r15),%rdx
    e044:	48 89 6c 24 10       	mov    %rbp,0x10(%rsp)
    e049:	49 39 d7             	cmp    %rdx,%r15
    e04c:	0f 85 4e ff ff ff    	jne    dfa0 <_ZSt11__make_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_RSS_+0x70>
    e052:	c4 c1 7a 6f 07       	vmovdqu (%r15),%xmm0
    e057:	49 8b 77 f8          	mov    -0x8(%r15),%rsi
    e05b:	41 c6 07 00          	movb   $0x0,(%r15)
    e05f:	49 c7 47 f8 00 00 00 	movq   $0x0,-0x8(%r15)
    e066:	00 
    e067:	4c 89 6c 24 30       	mov    %r13,0x30(%rsp)
    e06c:	c5 f9 7f 44 24 20    	vmovdqa %xmm0,0x20(%rsp)
    e072:	c5 f9 6f 4c 24 20    	vmovdqa 0x20(%rsp),%xmm1
    e078:	c5 f9 7f 4c 24 40    	vmovdqa %xmm1,0x40(%rsp)
    e07e:	e9 56 ff ff ff       	jmp    dfd9 <_ZSt11__make_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_RSS_+0xa9>
    e083:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)
    e088:	48 39 ef             	cmp    %rbp,%rdi
    e08b:	74 0e                	je     e09b <_ZSt11__make_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_RSS_+0x16b>
    e08d:	48 8b 44 24 20       	mov    0x20(%rsp),%rax
    e092:	48 8d 70 01          	lea    0x1(%rax),%rsi
    e096:	e8 85 52 ff ff       	call   3320 <_ZdlPvm@plt>
    e09b:	48 8b 44 24 58       	mov    0x58(%rsp),%rax
    e0a0:	64 48 2b 04 25 28 00 	sub    %fs:0x28,%rax
    e0a7:	00 00 
    e0a9:	75 0f                	jne    e0ba <_ZSt11__make_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_RSS_+0x18a>
    e0ab:	48 83 c4 68          	add    $0x68,%rsp
    e0af:	5b                   	pop    %rbx
    e0b0:	5d                   	pop    %rbp
    e0b1:	41 5c                	pop    %r12
    e0b3:	41 5d                	pop    %r13
    e0b5:	41 5e                	pop    %r14
    e0b7:	41 5f                	pop    %r15
    e0b9:	c3                   	ret    
    e0ba:	e8 91 50 ff ff       	call   3150 <__stack_chk_fail@plt>
    e0bf:	90                   	nop

000000000000e0c0 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_>:
    e0c0:	41 57                	push   %r15
    e0c2:	41 56                	push   %r14
    e0c4:	41 55                	push   %r13
    e0c6:	41 54                	push   %r12
    e0c8:	55                   	push   %rbp
    e0c9:	53                   	push   %rbx
    e0ca:	48 81 ec 88 00 00 00 	sub    $0x88,%rsp
    e0d1:	48 89 54 24 08       	mov    %rdx,0x8(%rsp)
    e0d6:	64 48 8b 04 25 28 00 	mov    %fs:0x28,%rax
    e0dd:	00 00 
    e0df:	48 89 44 24 78       	mov    %rax,0x78(%rsp)
    e0e4:	48 89 f0             	mov    %rsi,%rax
    e0e7:	48 89 34 24          	mov    %rsi,(%rsp)
    e0eb:	48 29 f8             	sub    %rdi,%rax
    e0ee:	48 3d 00 02 00 00    	cmp    $0x200,%rax
    e0f4:	0f 8e 37 03 00 00    	jle    e431 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x371>
    e0fa:	48 89 fd             	mov    %rdi,%rbp
    e0fd:	49 89 f4             	mov    %rsi,%r12
    e100:	48 85 d2             	test   %rdx,%rdx
    e103:	0f 84 d8 01 00 00    	je     e2e1 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x221>
    e109:	48 8d 47 20          	lea    0x20(%rdi),%rax
    e10d:	48 89 44 24 10       	mov    %rax,0x10(%rsp)
    e112:	48 8b 04 24          	mov    (%rsp),%rax
    e116:	48 29 e8             	sub    %rbp,%rax
    e119:	49 89 c4             	mov    %rax,%r12
    e11c:	49 c1 fc 05          	sar    $0x5,%r12
    e120:	48 c1 e8 3f          	shr    $0x3f,%rax
    e124:	49 01 c4             	add    %rax,%r12
    e127:	49 d1 fc             	sar    %r12
    e12a:	49 c1 e4 05          	shl    $0x5,%r12
    e12e:	49 01 ec             	add    %rbp,%r12
    e131:	48 8b 5d 28          	mov    0x28(%rbp),%rbx
    e135:	4d 8b 74 24 08       	mov    0x8(%r12),%r14
    e13a:	48 ff 4c 24 08       	decq   0x8(%rsp)
    e13f:	4c 39 f3             	cmp    %r14,%rbx
    e142:	0f 84 37 01 00 00    	je     e27f <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x1bf>
    e148:	48 8b 04 24          	mov    (%rsp),%rax
    e14c:	4c 8b 78 e8          	mov    -0x18(%rax),%r15
    e150:	4c 8d 68 e0          	lea    -0x20(%rax),%r13
    e154:	0f 82 57 01 00 00    	jb     e2b1 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x1f1>
    e15a:	4c 39 fb             	cmp    %r15,%rbx
    e15d:	0f 84 0b 04 00 00    	je     e56e <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x4ae>
    e163:	4c 39 fb             	cmp    %r15,%rbx
    e166:	0f 82 63 01 00 00    	jb     e2cf <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x20f>
    e16c:	4d 39 fe             	cmp    %r15,%r14
    e16f:	74 7f                	je     e1f0 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x130>
    e171:	0f 83 9a 00 00 00    	jae    e211 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x151>
    e177:	4c 89 ee             	mov    %r13,%rsi
    e17a:	48 89 ef             	mov    %rbp,%rdi
    e17d:	e8 fe 50 ff ff       	call   3280 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE4swapERS4_@plt>
    e182:	4c 8b 6d 08          	mov    0x8(%rbp),%r13
    e186:	4c 8b 7c 24 10       	mov    0x10(%rsp),%r15
    e18b:	4c 8b 34 24          	mov    (%rsp),%r14
    e18f:	eb 0d                	jmp    e19e <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0xde>
    e191:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
    e198:	73 25                	jae    e1bf <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0xff>
    e19a:	49 83 c7 20          	add    $0x20,%r15
    e19e:	4d 89 fc             	mov    %r15,%r12
    e1a1:	4d 39 6f 08          	cmp    %r13,0x8(%r15)
    e1a5:	75 f1                	jne    e198 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0xd8>
    e1a7:	4d 85 ed             	test   %r13,%r13
    e1aa:	74 13                	je     e1bf <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0xff>
    e1ac:	48 8b 75 00          	mov    0x0(%rbp),%rsi
    e1b0:	49 8b 3f             	mov    (%r15),%rdi
    e1b3:	4c 89 ea             	mov    %r13,%rdx
    e1b6:	e8 b5 50 ff ff       	call   3270 <memcmp@plt>
    e1bb:	85 c0                	test   %eax,%eax
    e1bd:	78 db                	js     e19a <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0xda>
    e1bf:	49 8d 5e e0          	lea    -0x20(%r14),%rbx
    e1c3:	48 8b 73 08          	mov    0x8(%rbx),%rsi
    e1c7:	49 89 de             	mov    %rbx,%r14
    e1ca:	4c 39 ee             	cmp    %r13,%rsi
    e1cd:	0f 97 c0             	seta   %al
    e1d0:	74 56                	je     e228 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x168>
    e1d2:	48 83 eb 20          	sub    $0x20,%rbx
    e1d6:	84 c0                	test   %al,%al
    e1d8:	75 e9                	jne    e1c3 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x103>
    e1da:	4d 39 f7             	cmp    %r14,%r15
    e1dd:	73 69                	jae    e248 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x188>
    e1df:	4c 89 f6             	mov    %r14,%rsi
    e1e2:	4c 89 ff             	mov    %r15,%rdi
    e1e5:	e8 96 50 ff ff       	call   3280 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE4swapERS4_@plt>
    e1ea:	4c 8b 6d 08          	mov    0x8(%rbp),%r13
    e1ee:	eb aa                	jmp    e19a <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0xda>
    e1f0:	4d 85 f6             	test   %r14,%r14
    e1f3:	74 1c                	je     e211 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x151>
    e1f5:	48 8b 04 24          	mov    (%rsp),%rax
    e1f9:	49 8b 3c 24          	mov    (%r12),%rdi
    e1fd:	48 8b 70 e0          	mov    -0x20(%rax),%rsi
    e201:	4c 89 f2             	mov    %r14,%rdx
    e204:	e8 67 50 ff ff       	call   3270 <memcmp@plt>
    e209:	85 c0                	test   %eax,%eax
    e20b:	0f 85 6f 03 00 00    	jne    e580 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x4c0>
    e211:	4c 89 e6             	mov    %r12,%rsi
    e214:	48 89 ef             	mov    %rbp,%rdi
    e217:	e8 64 50 ff ff       	call   3280 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE4swapERS4_@plt>
    e21c:	e9 61 ff ff ff       	jmp    e182 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0xc2>
    e221:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
    e228:	4d 85 ed             	test   %r13,%r13
    e22b:	74 ad                	je     e1da <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x11a>
    e22d:	48 8b 33             	mov    (%rbx),%rsi
    e230:	48 8b 7d 00          	mov    0x0(%rbp),%rdi
    e234:	4c 89 ea             	mov    %r13,%rdx
    e237:	e8 34 50 ff ff       	call   3270 <memcmp@plt>
    e23c:	85 c0                	test   %eax,%eax
    e23e:	74 9a                	je     e1da <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x11a>
    e240:	c1 e8 1f             	shr    $0x1f,%eax
    e243:	eb 8d                	jmp    e1d2 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x112>
    e245:	0f 1f 00             	nopl   (%rax)
    e248:	48 8b 54 24 08       	mov    0x8(%rsp),%rdx
    e24d:	48 8b 34 24          	mov    (%rsp),%rsi
    e251:	4c 89 ff             	mov    %r15,%rdi
    e254:	e8 67 fe ff ff       	call   e0c0 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_>
    e259:	4c 89 f8             	mov    %r15,%rax
    e25c:	48 29 e8             	sub    %rbp,%rax
    e25f:	48 3d 00 02 00 00    	cmp    $0x200,%rax
    e265:	0f 8e c6 01 00 00    	jle    e431 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x371>
    e26b:	48 83 7c 24 08 00    	cmpq   $0x0,0x8(%rsp)
    e271:	74 6e                	je     e2e1 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x221>
    e273:	4c 89 3c 24          	mov    %r15,(%rsp)
    e277:	4c 89 f8             	mov    %r15,%rax
    e27a:	e9 97 fe ff ff       	jmp    e116 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x56>
    e27f:	48 85 db             	test   %rbx,%rbx
    e282:	0f 84 60 02 00 00    	je     e4e8 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x428>
    e288:	48 8b 7d 20          	mov    0x20(%rbp),%rdi
    e28c:	49 8b 34 24          	mov    (%r12),%rsi
    e290:	48 89 da             	mov    %rbx,%rdx
    e293:	48 89 7c 24 18       	mov    %rdi,0x18(%rsp)
    e298:	e8 d3 4f ff ff       	call   3270 <memcmp@plt>
    e29d:	85 c0                	test   %eax,%eax
    e29f:	48 8b 04 24          	mov    (%rsp),%rax
    e2a3:	4c 8b 78 e8          	mov    -0x18(%rax),%r15
    e2a7:	4c 8d 68 e0          	lea    -0x20(%rax),%r13
    e2ab:	0f 89 0a 02 00 00    	jns    e4bb <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x3fb>
    e2b1:	4d 39 f7             	cmp    %r14,%r15
    e2b4:	0f 84 9d 01 00 00    	je     e457 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x397>
    e2ba:	0f 87 51 ff ff ff    	ja     e211 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x151>
    e2c0:	49 39 df             	cmp    %rbx,%r15
    e2c3:	0f 84 c2 01 00 00    	je     e48b <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x3cb>
    e2c9:	0f 87 a8 fe ff ff    	ja     e177 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0xb7>
    e2cf:	48 8b 74 24 10       	mov    0x10(%rsp),%rsi
    e2d4:	48 89 ef             	mov    %rbp,%rdi
    e2d7:	e8 a4 4f ff ff       	call   3280 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE4swapERS4_@plt>
    e2dc:	e9 a1 fe ff ff       	jmp    e182 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0xc2>
    e2e1:	0f b6 84 24 c0 00 00 	movzbl 0xc0(%rsp),%eax
    e2e8:	00 
    e2e9:	4c 89 e6             	mov    %r12,%rsi
    e2ec:	48 8d 54 24 2f       	lea    0x2f(%rsp),%rdx
    e2f1:	48 89 ef             	mov    %rbp,%rdi
    e2f4:	88 44 24 2f          	mov    %al,0x2f(%rsp)
    e2f8:	e8 33 fc ff ff       	call   df30 <_ZSt11__make_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_RSS_>
    e2fd:	48 8d 44 24 50       	lea    0x50(%rsp),%rax
    e302:	48 89 04 24          	mov    %rax,(%rsp)
    e306:	4c 8d 75 10          	lea    0x10(%rbp),%r14
    e30a:	49 83 ec 10          	sub    $0x10,%r12
    e30e:	4c 8d 6c 24 40       	lea    0x40(%rsp),%r13
    e313:	4c 8d 7c 24 60       	lea    0x60(%rsp),%r15
    e318:	0f 1f 84 00 00 00 00 	nopl   0x0(%rax,%rax,1)
    e31f:	00 
    e320:	49 8b 44 24 f0       	mov    -0x10(%r12),%rax
    e325:	4c 89 6c 24 30       	mov    %r13,0x30(%rsp)
    e32a:	49 8d 5c 24 f0       	lea    -0x10(%r12),%rbx
    e32f:	4c 39 e0             	cmp    %r12,%rax
    e332:	0f 84 25 02 00 00    	je     e55d <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x49d>
    e338:	48 89 44 24 30       	mov    %rax,0x30(%rsp)
    e33d:	49 8b 04 24          	mov    (%r12),%rax
    e341:	48 89 44 24 40       	mov    %rax,0x40(%rsp)
    e346:	4d 89 64 24 f0       	mov    %r12,-0x10(%r12)
    e34b:	41 c6 04 24 00       	movb   $0x0,(%r12)
    e350:	49 8b 44 24 f8       	mov    -0x8(%r12),%rax
    e355:	49 c7 44 24 f8 00 00 	movq   $0x0,-0x8(%r12)
    e35c:	00 00 
    e35e:	48 89 44 24 38       	mov    %rax,0x38(%rsp)
    e363:	48 8b 45 00          	mov    0x0(%rbp),%rax
    e367:	49 39 c6             	cmp    %rax,%r14
    e36a:	0f 84 bc 01 00 00    	je     e52c <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x46c>
    e370:	49 89 44 24 f0       	mov    %rax,-0x10(%r12)
    e375:	48 8b 45 08          	mov    0x8(%rbp),%rax
    e379:	49 89 44 24 f8       	mov    %rax,-0x8(%r12)
    e37e:	48 8b 45 10          	mov    0x10(%rbp),%rax
    e382:	49 89 04 24          	mov    %rax,(%r12)
    e386:	4c 89 75 00          	mov    %r14,0x0(%rbp)
    e38a:	4c 89 f0             	mov    %r14,%rax
    e38d:	48 c7 45 08 00 00 00 	movq   $0x0,0x8(%rbp)
    e394:	00 
    e395:	c6 00 00             	movb   $0x0,(%rax)
    e398:	4c 89 7c 24 50       	mov    %r15,0x50(%rsp)
    e39d:	48 8b 44 24 30       	mov    0x30(%rsp),%rax
    e3a2:	4c 39 e8             	cmp    %r13,%rax
    e3a5:	0f 84 70 01 00 00    	je     e51b <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x45b>
    e3ab:	48 89 44 24 50       	mov    %rax,0x50(%rsp)
    e3b0:	48 8b 44 24 40       	mov    0x40(%rsp),%rax
    e3b5:	48 89 44 24 60       	mov    %rax,0x60(%rsp)
    e3ba:	48 29 eb             	sub    %rbp,%rbx
    e3bd:	48 8b 44 24 38       	mov    0x38(%rsp),%rax
    e3c2:	48 8b 0c 24          	mov    (%rsp),%rcx
    e3c6:	48 89 da             	mov    %rbx,%rdx
    e3c9:	48 89 ef             	mov    %rbp,%rdi
    e3cc:	48 c1 fa 05          	sar    $0x5,%rdx
    e3d0:	31 f6                	xor    %esi,%esi
    e3d2:	48 89 44 24 58       	mov    %rax,0x58(%rsp)
    e3d7:	4c 89 6c 24 30       	mov    %r13,0x30(%rsp)
    e3dc:	48 c7 44 24 38 00 00 	movq   $0x0,0x38(%rsp)
    e3e3:	00 00 
    e3e5:	c6 44 24 40 00       	movb   $0x0,0x40(%rsp)
    e3ea:	e8 c1 65 ff ff       	call   49b0 <_ZSt13__adjust_heapIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElS7_NS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_SS_T1_T2_.isra.0>
    e3ef:	48 8b 7c 24 50       	mov    0x50(%rsp),%rdi
    e3f4:	4c 39 ff             	cmp    %r15,%rdi
    e3f7:	74 0e                	je     e407 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x347>
    e3f9:	48 8b 44 24 60       	mov    0x60(%rsp),%rax
    e3fe:	48 8d 70 01          	lea    0x1(%rax),%rsi
    e402:	e8 19 4f ff ff       	call   3320 <_ZdlPvm@plt>
    e407:	48 8b 7c 24 30       	mov    0x30(%rsp),%rdi
    e40c:	4c 39 ef             	cmp    %r13,%rdi
    e40f:	0f 84 f3 00 00 00    	je     e508 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x448>
    e415:	48 8b 44 24 40       	mov    0x40(%rsp),%rax
    e41a:	49 83 ec 20          	sub    $0x20,%r12
    e41e:	48 8d 70 01          	lea    0x1(%rax),%rsi
    e422:	e8 f9 4e ff ff       	call   3320 <_ZdlPvm@plt>
    e427:	48 83 fb 20          	cmp    $0x20,%rbx
    e42b:	0f 8f ef fe ff ff    	jg     e320 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x260>
    e431:	48 8b 44 24 78       	mov    0x78(%rsp),%rax
    e436:	64 48 2b 04 25 28 00 	sub    %fs:0x28,%rax
    e43d:	00 00 
    e43f:	0f 85 54 01 00 00    	jne    e599 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x4d9>
    e445:	48 81 c4 88 00 00 00 	add    $0x88,%rsp
    e44c:	5b                   	pop    %rbx
    e44d:	5d                   	pop    %rbp
    e44e:	41 5c                	pop    %r12
    e450:	41 5d                	pop    %r13
    e452:	41 5e                	pop    %r14
    e454:	41 5f                	pop    %r15
    e456:	c3                   	ret    
    e457:	4d 85 ff             	test   %r15,%r15
    e45a:	0f 84 60 fe ff ff    	je     e2c0 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x200>
    e460:	48 8b 04 24          	mov    (%rsp),%rax
    e464:	49 8b 3c 24          	mov    (%r12),%rdi
    e468:	48 8b 70 e0          	mov    -0x20(%rax),%rsi
    e46c:	4c 89 fa             	mov    %r15,%rdx
    e46f:	e8 fc 4d ff ff       	call   3270 <memcmp@plt>
    e474:	85 c0                	test   %eax,%eax
    e476:	0f 84 44 fe ff ff    	je     e2c0 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x200>
    e47c:	0f 88 8f fd ff ff    	js     e211 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x151>
    e482:	49 39 df             	cmp    %rbx,%r15
    e485:	0f 85 3e fe ff ff    	jne    e2c9 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x209>
    e48b:	4d 85 ff             	test   %r15,%r15
    e48e:	0f 84 3b fe ff ff    	je     e2cf <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x20f>
    e494:	48 8b 04 24          	mov    (%rsp),%rax
    e498:	48 8b 7d 20          	mov    0x20(%rbp),%rdi
    e49c:	48 8b 70 e0          	mov    -0x20(%rax),%rsi
    e4a0:	4c 89 fa             	mov    %r15,%rdx
    e4a3:	e8 c8 4d ff ff       	call   3270 <memcmp@plt>
    e4a8:	85 c0                	test   %eax,%eax
    e4aa:	0f 84 1f fe ff ff    	je     e2cf <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x20f>
    e4b0:	0f 88 c1 fc ff ff    	js     e177 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0xb7>
    e4b6:	e9 14 fe ff ff       	jmp    e2cf <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x20f>
    e4bb:	49 39 df             	cmp    %rbx,%r15
    e4be:	48 8b 7c 24 18       	mov    0x18(%rsp),%rdi
    e4c3:	0f 85 9a fc ff ff    	jne    e163 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0xa3>
    e4c9:	48 8b 70 e0          	mov    -0x20(%rax),%rsi
    e4cd:	48 89 da             	mov    %rbx,%rdx
    e4d0:	e8 9b 4d ff ff       	call   3270 <memcmp@plt>
    e4d5:	85 c0                	test   %eax,%eax
    e4d7:	0f 84 8f fc ff ff    	je     e16c <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0xac>
    e4dd:	0f 89 89 fc ff ff    	jns    e16c <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0xac>
    e4e3:	e9 e7 fd ff ff       	jmp    e2cf <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x20f>
    e4e8:	48 8b 04 24          	mov    (%rsp),%rax
    e4ec:	4c 8b 78 e8          	mov    -0x18(%rax),%r15
    e4f0:	4c 8d 68 e0          	lea    -0x20(%rax),%r13
    e4f4:	4d 85 ff             	test   %r15,%r15
    e4f7:	0f 85 66 fc ff ff    	jne    e163 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0xa3>
    e4fd:	e9 6a fc ff ff       	jmp    e16c <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0xac>
    e502:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)
    e508:	49 83 ec 20          	sub    $0x20,%r12
    e50c:	48 83 fb 20          	cmp    $0x20,%rbx
    e510:	0f 8f 0a fe ff ff    	jg     e320 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x260>
    e516:	e9 16 ff ff ff       	jmp    e431 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x371>
    e51b:	c5 f9 6f 4c 24 40    	vmovdqa 0x40(%rsp),%xmm1
    e521:	c5 f9 7f 4c 24 60    	vmovdqa %xmm1,0x60(%rsp)
    e527:	e9 8e fe ff ff       	jmp    e3ba <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x2fa>
    e52c:	48 8b 55 08          	mov    0x8(%rbp),%rdx
    e530:	48 85 d2             	test   %rdx,%rdx
    e533:	74 15                	je     e54a <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x48a>
    e535:	48 83 fa 01          	cmp    $0x1,%rdx
    e539:	74 50                	je     e58b <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x4cb>
    e53b:	4c 89 f6             	mov    %r14,%rsi
    e53e:	4c 89 e7             	mov    %r12,%rdi
    e541:	e8 fa 4b ff ff       	call   3140 <memcpy@plt>
    e546:	48 8b 55 08          	mov    0x8(%rbp),%rdx
    e54a:	49 89 54 24 f8       	mov    %rdx,-0x8(%r12)
    e54f:	41 c6 04 14 00       	movb   $0x0,(%r12,%rdx,1)
    e554:	48 8b 45 00          	mov    0x0(%rbp),%rax
    e558:	e9 30 fe ff ff       	jmp    e38d <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x2cd>
    e55d:	c4 c1 7a 6f 04 24    	vmovdqu (%r12),%xmm0
    e563:	c5 f9 7f 44 24 40    	vmovdqa %xmm0,0x40(%rsp)
    e569:	e9 d8 fd ff ff       	jmp    e346 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x286>
    e56e:	48 85 db             	test   %rbx,%rbx
    e571:	0f 84 f5 fb ff ff    	je     e16c <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0xac>
    e577:	48 8b 7d 20          	mov    0x20(%rbp),%rdi
    e57b:	e9 49 ff ff ff       	jmp    e4c9 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x409>
    e580:	0f 88 f1 fb ff ff    	js     e177 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0xb7>
    e586:	e9 86 fc ff ff       	jmp    e211 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x151>
    e58b:	0f b6 45 10          	movzbl 0x10(%rbp),%eax
    e58f:	41 88 04 24          	mov    %al,(%r12)
    e593:	48 8b 55 08          	mov    0x8(%rbp),%rdx
    e597:	eb b1                	jmp    e54a <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_+0x48a>
    e599:	e8 b2 4b ff ff       	call   3150 <__stack_chk_fail@plt>
    e59e:	66 90                	xchg   %ax,%ax

000000000000e5a0 <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_>:
    e5a0:	41 57                	push   %r15
    e5a2:	41 56                	push   %r14
    e5a4:	4c 8d b7 a0 00 00 00 	lea    0xa0(%rdi),%r14
    e5ab:	41 55                	push   %r13
    e5ad:	41 54                	push   %r12
    e5af:	55                   	push   %rbp
    e5b0:	53                   	push   %rbx
    e5b1:	48 89 f3             	mov    %rsi,%rbx
    e5b4:	48 81 ec 88 00 00 00 	sub    $0x88,%rsp
    e5bb:	48 89 7c 24 20       	mov    %rdi,0x20(%rsp)
    e5c0:	48 89 f7             	mov    %rsi,%rdi
    e5c3:	64 48 8b 04 25 28 00 	mov    %fs:0x28,%rax
    e5ca:	00 00 
    e5cc:	48 89 44 24 78       	mov    %rax,0x78(%rsp)
    e5d1:	31 c0                	xor    %eax,%eax
    e5d3:	e8 58 4d ff ff       	call   3330 <strlen@plt>
    e5d8:	bf f0 00 00 00       	mov    $0xf0,%edi
    e5dd:	48 89 5c 24 68       	mov    %rbx,0x68(%rsp)
    e5e2:	48 89 44 24 60       	mov    %rax,0x60(%rsp)
    e5e7:	e8 b4 4c ff ff       	call   32a0 <_Znwm@plt>
    e5ec:	49 89 c5             	mov    %rax,%r13
    e5ef:	48 83 c0 10          	add    $0x10,%rax
    e5f3:	48 89 44 24 38       	mov    %rax,0x38(%rsp)
    e5f8:	49 8d 45 48          	lea    0x48(%r13),%rax
    e5fc:	48 89 04 24          	mov    %rax,(%rsp)
    e600:	49 89 45 38          	mov    %rax,0x38(%r13)
    e604:	48 8b 7c 24 60       	mov    0x60(%rsp),%rdi
    e609:	48 8d 05 40 81 ff ff 	lea    -0x7ec0(%rip),%rax        # 6750 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse8Argument8m_actionMUlS8_E_EE9_M_invokeERKSt9_Any_dataS8_>
    e610:	49 89 85 b0 00 00 00 	mov    %rax,0xb0(%r13)
    e617:	49 8d 5d 78          	lea    0x78(%r13),%rbx
    e61b:	48 8d 05 ee 7c ff ff 	lea    -0x8312(%rip),%rax        # 6310 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse8Argument8m_actionMUlS8_E_EE10_M_managerERSt9_Any_dataRKSE_St18_Manager_operation>
    e622:	49 89 85 a8 00 00 00 	mov    %rax,0xa8(%r13)
    e629:	49 c7 45 10 00 00 00 	movq   $0x0,0x10(%r13)
    e630:	00 
    e631:	49 c7 45 18 00 00 00 	movq   $0x0,0x18(%r13)
    e638:	00 
    e639:	49 c7 45 20 00 00 00 	movq   $0x0,0x20(%r13)
    e640:	00 
    e641:	49 c7 45 28 00 00 00 	movq   $0x0,0x28(%r13)
    e648:	00 
    e649:	49 c7 45 30 00 00 00 	movq   $0x0,0x30(%r13)
    e650:	00 
    e651:	49 c7 45 40 00 00 00 	movq   $0x0,0x40(%r13)
    e658:	00 
    e659:	41 c6 45 48 00       	movb   $0x0,0x48(%r13)
    e65e:	49 c7 45 58 00 00 00 	movq   $0x0,0x58(%r13)
    e665:	00 
    e666:	49 c7 45 60 00 00 00 	movq   $0x0,0x60(%r13)
    e66d:	00 
    e66e:	49 89 5d 68          	mov    %rbx,0x68(%r13)
    e672:	49 c7 45 70 00 00 00 	movq   $0x0,0x70(%r13)
    e679:	00 
    e67a:	41 c6 45 78 00       	movb   $0x0,0x78(%r13)
    e67f:	49 c7 85 88 00 00 00 	movq   $0x0,0x88(%r13)
    e686:	00 00 00 00 
    e68a:	49 c7 85 90 00 00 00 	movq   $0x0,0x90(%r13)
    e691:	00 00 00 00 
    e695:	41 c6 85 b8 00 00 00 	movb   $0x0,0xb8(%r13)
    e69c:	00 
    e69d:	49 c7 85 c0 00 00 00 	movq   $0x0,0xc0(%r13)
    e6a4:	00 00 00 00 
    e6a8:	49 c7 85 c8 00 00 00 	movq   $0x0,0xc8(%r13)
    e6af:	00 00 00 00 
    e6b3:	49 c7 85 d0 00 00 00 	movq   $0x0,0xd0(%r13)
    e6ba:	00 00 00 00 
    e6be:	49 c7 85 d8 00 00 00 	movq   $0x1,0xd8(%r13)
    e6c5:	01 00 00 00 
    e6c9:	49 c7 85 e0 00 00 00 	movq   $0x1,0xe0(%r13)
    e6d0:	01 00 00 00 
    e6d4:	41 c6 85 e8 00 00 00 	movb   $0x0,0xe8(%r13)
    e6db:	00 
    e6dc:	48 8b 44 24 68       	mov    0x68(%rsp),%rax
    e6e1:	48 85 ff             	test   %rdi,%rdi
    e6e4:	74 09                	je     e6ef <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x14f>
    e6e6:	80 38 2d             	cmpb   $0x2d,(%rax)
    e6e9:	0f 84 50 03 00 00    	je     ea3f <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x49f>
    e6ef:	41 80 a5 e9 00 00 00 	andb   $0xf0,0xe9(%r13)
    e6f6:	f0 
    e6f7:	31 ed                	xor    %ebp,%ebp
    e6f9:	48 8b 7c 24 38       	mov    0x38(%rsp),%rdi
    e6fe:	48 8d 44 24 58       	lea    0x58(%rsp),%rax
    e703:	48 8d 54 24 60       	lea    0x60(%rsp),%rdx
    e708:	48 89 ee             	mov    %rbp,%rsi
    e70b:	48 89 44 24 10       	mov    %rax,0x10(%rsp)
    e710:	e8 7b d4 ff ff       	call   bb90 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE17_M_realloc_insertIJRSt17basic_string_viewIcS3_EEEEvN9__gnu_cxx17__normal_iteratorIPS5_S7_EEDpOT_>
    e715:	49 8b 6d 18          	mov    0x18(%r13),%rbp
    e719:	4d 8b 65 10          	mov    0x10(%r13),%r12
    e71d:	49 39 ec             	cmp    %rbp,%r12
    e720:	74 67                	je     e789 <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x1e9>
    e722:	48 89 eb             	mov    %rbp,%rbx
    e725:	4c 29 e3             	sub    %r12,%rbx
    e728:	48 89 d8             	mov    %rbx,%rax
    e72b:	48 c1 f8 05          	sar    $0x5,%rax
    e72f:	ba 3f 00 00 00       	mov    $0x3f,%edx
    e734:	f3 48 0f bd c0       	lzcnt  %rax,%rax
    e739:	29 c2                	sub    %eax,%edx
    e73b:	48 63 d2             	movslq %edx,%rdx
    e73e:	48 01 d2             	add    %rdx,%rdx
    e741:	48 89 ee             	mov    %rbp,%rsi
    e744:	4c 89 e7             	mov    %r12,%rdi
    e747:	e8 74 f9 ff ff       	call   e0c0 <_ZSt16__introsort_loopIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEElNS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_T1_>
    e74c:	48 81 fb 00 02 00 00 	cmp    $0x200,%rbx
    e753:	0f 8e bc 03 00 00    	jle    eb15 <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x575>
    e759:	49 8d 9c 24 00 02 00 	lea    0x200(%r12),%rbx
    e760:	00 
    e761:	48 89 de             	mov    %rbx,%rsi
    e764:	4c 89 e7             	mov    %r12,%rdi
    e767:	e8 54 8a ff ff       	call   71c0 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0>
    e76c:	48 39 eb             	cmp    %rbp,%rbx
    e76f:	74 18                	je     e789 <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x1e9>
    e771:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
    e778:	48 89 df             	mov    %rbx,%rdi
    e77b:	48 83 c3 20          	add    $0x20,%rbx
    e77f:	e8 6c 87 ff ff       	call   6ef0 <_ZSt25__unguarded_linear_insertIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops14_Val_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SS_.isra.0>
    e784:	48 39 dd             	cmp    %rbx,%rbp
    e787:	75 ef                	jne    e778 <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x1d8>
    e789:	4c 89 f6             	mov    %r14,%rsi
    e78c:	4c 89 ef             	mov    %r13,%rdi
    e78f:	e8 5c 4b ff ff       	call   32f0 <_ZNSt8__detail15_List_node_base7_M_hookEPS0_@plt>
    e794:	48 8b 44 24 20       	mov    0x20(%rsp),%rax
    e799:	48 ff 80 b0 00 00 00 	incq   0xb0(%rax)
    e7a0:	41 f6 85 e9 00 00 00 	testb  $0x1,0xe9(%r13)
    e7a7:	01 
    e7a8:	0f 84 2c 03 00 00    	je     eada <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x53a>
    e7ae:	48 8b 5c 24 20       	mov    0x20(%rsp),%rbx
    e7b3:	49 8b 45 18          	mov    0x18(%r13),%rax
    e7b7:	4d 8b 75 10          	mov    0x10(%r13),%r14
    e7bb:	48 8d b3 b8 00 00 00 	lea    0xb8(%rbx),%rsi
    e7c2:	48 81 c3 c0 00 00 00 	add    $0xc0,%rbx
    e7c9:	48 89 5c 24 18       	mov    %rbx,0x18(%rsp)
    e7ce:	48 89 44 24 28       	mov    %rax,0x28(%rsp)
    e7d3:	48 89 74 24 30       	mov    %rsi,0x30(%rsp)
    e7d8:	bb 00 00 00 80       	mov    $0x80000000,%ebx
    e7dd:	49 39 c6             	cmp    %rax,%r14
    e7e0:	0f 84 90 01 00 00    	je     e976 <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x3d6>
    e7e6:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
    e7ed:	00 00 00 
    e7f0:	48 8b 44 24 20       	mov    0x20(%rsp),%rax
    e7f5:	c4 c1 7a 7e 46 08    	vmovq  0x8(%r14),%xmm0
    e7fb:	4c 8b b8 c8 00 00 00 	mov    0xc8(%rax),%r15
    e802:	4d 8b 26             	mov    (%r14),%r12
    e805:	48 8b 6c 24 18       	mov    0x18(%rsp),%rbp
    e80a:	c5 f9 7f 04 24       	vmovdqa %xmm0,(%rsp)
    e80f:	4d 85 ff             	test   %r15,%r15
    e812:	75 15                	jne    e829 <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x289>
    e814:	e9 d8 00 00 00       	jmp    e8f1 <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x351>
    e819:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
    e820:	4d 8b 7f 18          	mov    0x18(%r15),%r15
    e824:	4d 85 ff             	test   %r15,%r15
    e827:	74 61                	je     e88a <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x2ea>
    e829:	c4 c1 7a 7e 47 20    	vmovq  0x20(%r15),%xmm0
    e82f:	62 f2 fd 08 3b 0c 24 	vpminuq (%rsp),%xmm0,%xmm1
    e836:	c4 e1 f9 7e ca       	vmovq  %xmm1,%rdx
    e83b:	48 85 d2             	test   %rdx,%rdx
    e83e:	74 1c                	je     e85c <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x2bc>
    e840:	49 8b 7f 28          	mov    0x28(%r15),%rdi
    e844:	4c 89 e6             	mov    %r12,%rsi
    e847:	c5 f9 d6 44 24 10    	vmovq  %xmm0,0x10(%rsp)
    e84d:	e8 1e 4a ff ff       	call   3270 <memcmp@plt>
    e852:	85 c0                	test   %eax,%eax
    e854:	c5 fa 7e 44 24 10    	vmovq  0x10(%rsp),%xmm0
    e85a:	75 1e                	jne    e87a <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x2da>
    e85c:	c5 f9 fb 04 24       	vpsubq (%rsp),%xmm0,%xmm0
    e861:	c4 e1 f9 7e c0       	vmovq  %xmm0,%rax
    e866:	48 39 d8             	cmp    %rbx,%rax
    e869:	7d 13                	jge    e87e <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x2de>
    e86b:	48 b9 ff ff ff 7f ff 	movabs $0xffffffff7fffffff,%rcx
    e872:	ff ff ff 
    e875:	48 39 c8             	cmp    %rcx,%rax
    e878:	7e a6                	jle    e820 <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x280>
    e87a:	85 c0                	test   %eax,%eax
    e87c:	78 a2                	js     e820 <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x280>
    e87e:	4c 89 fd             	mov    %r15,%rbp
    e881:	4d 8b 7f 10          	mov    0x10(%r15),%r15
    e885:	4d 85 ff             	test   %r15,%r15
    e888:	75 9f                	jne    e829 <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x289>
    e88a:	48 3b 6c 24 18       	cmp    0x18(%rsp),%rbp
    e88f:	74 60                	je     e8f1 <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x351>
    e891:	c5 fa 7e 45 20       	vmovq  0x20(%rbp),%xmm0
    e896:	62 f2 fd 08 3b 0c 24 	vpminuq (%rsp),%xmm0,%xmm1
    e89d:	c4 e1 f9 7e ca       	vmovq  %xmm1,%rdx
    e8a2:	48 85 d2             	test   %rdx,%rdx
    e8a5:	74 1c                	je     e8c3 <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x323>
    e8a7:	48 8b 75 28          	mov    0x28(%rbp),%rsi
    e8ab:	4c 89 e7             	mov    %r12,%rdi
    e8ae:	c5 f9 d6 44 24 10    	vmovq  %xmm0,0x10(%rsp)
    e8b4:	e8 b7 49 ff ff       	call   3270 <memcmp@plt>
    e8b9:	85 c0                	test   %eax,%eax
    e8bb:	c5 fa 7e 44 24 10    	vmovq  0x10(%rsp),%xmm0
    e8c1:	75 26                	jne    e8e9 <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x349>
    e8c3:	c5 f9 6f 1c 24       	vmovdqa (%rsp),%xmm3
    e8c8:	c5 e1 fb c0          	vpsubq %xmm0,%xmm3,%xmm0
    e8cc:	c4 e1 f9 7e c0       	vmovq  %xmm0,%rax
    e8d1:	48 39 d8             	cmp    %rbx,%rax
    e8d4:	0f 8d ce 00 00 00    	jge    e9a8 <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x408>
    e8da:	48 b9 ff ff ff 7f ff 	movabs $0xffffffff7fffffff,%rcx
    e8e1:	ff ff ff 
    e8e4:	48 39 c8             	cmp    %rcx,%rax
    e8e7:	7e 08                	jle    e8f1 <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x351>
    e8e9:	85 c0                	test   %eax,%eax
    e8eb:	0f 89 b7 00 00 00    	jns    e9a8 <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x408>
    e8f1:	bf 38 00 00 00       	mov    $0x38,%edi
    e8f6:	e8 a5 49 ff ff       	call   32a0 <_Znwm@plt>
    e8fb:	49 89 c0             	mov    %rax,%r8
    e8fe:	48 8b 04 24          	mov    (%rsp),%rax
    e902:	c4 c1 f9 6e d4       	vmovq  %r12,%xmm2
    e907:	49 89 40 20          	mov    %rax,0x20(%r8)
    e90b:	c4 c3 e9 22 c5 01    	vpinsrq $0x1,%r13,%xmm2,%xmm0
    e911:	48 8b 7c 24 30       	mov    0x30(%rsp),%rdi
    e916:	c4 c1 7a 7f 40 28    	vmovdqu %xmm0,0x28(%r8)
    e91c:	49 8d 50 20          	lea    0x20(%r8),%rdx
    e920:	48 89 ee             	mov    %rbp,%rsi
    e923:	4c 89 44 24 10       	mov    %r8,0x10(%rsp)
    e928:	e8 13 cb ff ff       	call   b440 <_ZNSt8_Rb_treeISt17basic_string_viewIcSt11char_traitsIcEESt4pairIKS3_St14_List_iteratorIN8argparse8ArgumentEEESt10_Select1stISA_ESt4lessIvESaISA_EE29_M_get_insert_hint_unique_posESt23_Rb_tree_const_iteratorISA_ERS5_>
    e92d:	48 85 d2             	test   %rdx,%rdx
    e930:	4c 8b 44 24 10       	mov    0x10(%rsp),%r8
    e935:	49 89 d1             	mov    %rdx,%r9
    e938:	74 76                	je     e9b0 <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x410>
    e93a:	48 85 c0             	test   %rax,%rax
    e93d:	75 07                	jne    e946 <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x3a6>
    e93f:	48 3b 54 24 18       	cmp    0x18(%rsp),%rdx
    e944:	75 79                	jne    e9bf <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x41f>
    e946:	bf 01 00 00 00       	mov    $0x1,%edi
    e94b:	48 8b 4c 24 18       	mov    0x18(%rsp),%rcx
    e950:	4c 89 ca             	mov    %r9,%rdx
    e953:	4c 89 c6             	mov    %r8,%rsi
    e956:	e8 15 4a ff ff       	call   3370 <_ZSt29_Rb_tree_insert_and_rebalancebPSt18_Rb_tree_node_baseS0_RS_@plt>
    e95b:	48 8b 44 24 20       	mov    0x20(%rsp),%rax
    e960:	48 ff 80 e0 00 00 00 	incq   0xe0(%rax)
    e967:	49 83 c6 20          	add    $0x20,%r14
    e96b:	4c 39 74 24 28       	cmp    %r14,0x28(%rsp)
    e970:	0f 85 7a fe ff ff    	jne    e7f0 <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x250>
    e976:	48 8b 44 24 78       	mov    0x78(%rsp),%rax
    e97b:	64 48 2b 04 25 28 00 	sub    %fs:0x28,%rax
    e982:	00 00 
    e984:	0f 85 fc 01 00 00    	jne    eb86 <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x5e6>
    e98a:	48 8b 44 24 38       	mov    0x38(%rsp),%rax
    e98f:	48 81 c4 88 00 00 00 	add    $0x88,%rsp
    e996:	5b                   	pop    %rbx
    e997:	5d                   	pop    %rbp
    e998:	41 5c                	pop    %r12
    e99a:	41 5d                	pop    %r13
    e99c:	41 5e                	pop    %r14
    e99e:	41 5f                	pop    %r15
    e9a0:	c3                   	ret    
    e9a1:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
    e9a8:	4c 89 6d 30          	mov    %r13,0x30(%rbp)
    e9ac:	eb b9                	jmp    e967 <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x3c7>
    e9ae:	66 90                	xchg   %ax,%ax
    e9b0:	be 38 00 00 00       	mov    $0x38,%esi
    e9b5:	4c 89 c7             	mov    %r8,%rdi
    e9b8:	e8 63 49 ff ff       	call   3320 <_ZdlPvm@plt>
    e9bd:	eb a8                	jmp    e967 <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x3c7>
    e9bf:	c5 fa 7e 42 20       	vmovq  0x20(%rdx),%xmm0
    e9c4:	62 f2 fd 08 3b 0c 24 	vpminuq (%rsp),%xmm0,%xmm1
    e9cb:	c4 e1 f9 7e ca       	vmovq  %xmm1,%rdx
    e9d0:	48 85 d2             	test   %rdx,%rdx
    e9d3:	74 32                	je     ea07 <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x467>
    e9d5:	49 8b 71 28          	mov    0x28(%r9),%rsi
    e9d9:	4c 89 e7             	mov    %r12,%rdi
    e9dc:	4c 89 44 24 40       	mov    %r8,0x40(%rsp)
    e9e1:	4c 89 4c 24 10       	mov    %r9,0x10(%rsp)
    e9e6:	c5 f9 d6 44 24 48    	vmovq  %xmm0,0x48(%rsp)
    e9ec:	e8 7f 48 ff ff       	call   3270 <memcmp@plt>
    e9f1:	85 c0                	test   %eax,%eax
    e9f3:	4c 8b 4c 24 10       	mov    0x10(%rsp),%r9
    e9f8:	4c 8b 44 24 40       	mov    0x40(%rsp),%r8
    e9fd:	c5 fa 7e 44 24 48    	vmovq  0x48(%rsp),%xmm0
    ea03:	89 c7                	mov    %eax,%edi
    ea05:	75 30                	jne    ea37 <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x497>
    ea07:	c5 f9 6f 24 24       	vmovdqa (%rsp),%xmm4
    ea0c:	31 ff                	xor    %edi,%edi
    ea0e:	c5 d9 fb c0          	vpsubq %xmm0,%xmm4,%xmm0
    ea12:	c4 e1 f9 7e c0       	vmovq  %xmm0,%rax
    ea17:	48 39 d8             	cmp    %rbx,%rax
    ea1a:	0f 8d 2b ff ff ff    	jge    e94b <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x3ab>
    ea20:	48 be ff ff ff 7f ff 	movabs $0xffffffff7fffffff,%rsi
    ea27:	ff ff ff 
    ea2a:	48 39 f0             	cmp    %rsi,%rax
    ea2d:	0f 8e 13 ff ff ff    	jle    e946 <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x3a6>
    ea33:	c5 f9 7e c7          	vmovd  %xmm0,%edi
    ea37:	c1 ef 1f             	shr    $0x1f,%edi
    ea3a:	e9 0c ff ff ff       	jmp    e94b <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x3ab>
    ea3f:	48 ff cf             	dec    %rdi
    ea42:	0f 84 a7 fc ff ff    	je     e6ef <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x14f>
    ea48:	48 8d 70 01          	lea    0x1(%rax),%rsi
    ea4c:	48 8d 44 24 58       	lea    0x58(%rsp),%rax
    ea51:	48 89 44 24 10       	mov    %rax,0x10(%rsp)
    ea56:	e8 b5 92 ff ff       	call   7d10 <_ZN8argparse8Argument18is_decimal_literalESt17basic_string_viewIcSt11char_traitsIcEE>
    ea5b:	89 c2                	mov    %eax,%edx
    ea5d:	41 0f b6 85 e9 00 00 	movzbl 0xe9(%r13),%eax
    ea64:	00 
    ea65:	83 f2 01             	xor    $0x1,%edx
    ea68:	83 e0 f0             	and    $0xfffffff0,%eax
    ea6b:	83 e2 0f             	and    $0xf,%edx
    ea6e:	49 8b 6d 18          	mov    0x18(%r13),%rbp
    ea72:	09 d0                	or     %edx,%eax
    ea74:	41 88 85 e9 00 00 00 	mov    %al,0xe9(%r13)
    ea7b:	49 39 6d 20          	cmp    %rbp,0x20(%r13)
    ea7f:	0f 84 74 fc ff ff    	je     e6f9 <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x159>
    ea85:	4c 8b 7c 24 68       	mov    0x68(%rsp),%r15
    ea8a:	4c 8b 64 24 60       	mov    0x60(%rsp),%r12
    ea8f:	4c 89 f8             	mov    %r15,%rax
    ea92:	48 8d 7d 10          	lea    0x10(%rbp),%rdi
    ea96:	4c 01 e0             	add    %r12,%rax
    ea99:	48 89 7d 00          	mov    %rdi,0x0(%rbp)
    ea9d:	74 09                	je     eaa8 <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x508>
    ea9f:	4d 85 ff             	test   %r15,%r15
    eaa2:	0f 84 c8 00 00 00    	je     eb70 <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x5d0>
    eaa8:	4c 89 64 24 58       	mov    %r12,0x58(%rsp)
    eaad:	49 83 fc 0f          	cmp    $0xf,%r12
    eab1:	77 7d                	ja     eb30 <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x590>
    eab3:	49 83 fc 01          	cmp    $0x1,%r12
    eab7:	75 6c                	jne    eb25 <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x585>
    eab9:	41 0f b6 07          	movzbl (%r15),%eax
    eabd:	88 45 10             	mov    %al,0x10(%rbp)
    eac0:	4c 89 65 08          	mov    %r12,0x8(%rbp)
    eac4:	42 c6 04 27 00       	movb   $0x0,(%rdi,%r12,1)
    eac9:	49 8b 45 18          	mov    0x18(%r13),%rax
    eacd:	48 8d 68 20          	lea    0x20(%rax),%rbp
    ead1:	49 89 6d 18          	mov    %rbp,0x18(%r13)
    ead5:	e9 3f fc ff ff       	jmp    e719 <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x179>
    eada:	48 8d b8 88 00 00 00 	lea    0x88(%rax),%rdi
    eae1:	49 8b 55 00          	mov    0x0(%r13),%rdx
    eae5:	48 89 c3             	mov    %rax,%rbx
    eae8:	4c 39 ef             	cmp    %r13,%rdi
    eaeb:	0f 84 bd fc ff ff    	je     e7ae <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x20e>
    eaf1:	48 39 d7             	cmp    %rdx,%rdi
    eaf4:	0f 84 b4 fc ff ff    	je     e7ae <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x20e>
    eafa:	4c 89 ee             	mov    %r13,%rsi
    eafd:	e8 ae 47 ff ff       	call   32b0 <_ZNSt8__detail15_List_node_base11_M_transferEPS0_S1_@plt>
    eb02:	48 ff 83 98 00 00 00 	incq   0x98(%rbx)
    eb09:	48 ff 8b b0 00 00 00 	decq   0xb0(%rbx)
    eb10:	e9 99 fc ff ff       	jmp    e7ae <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x20e>
    eb15:	48 89 ee             	mov    %rbp,%rsi
    eb18:	4c 89 e7             	mov    %r12,%rdi
    eb1b:	e8 a0 86 ff ff       	call   71c0 <_ZSt16__insertion_sortIN9__gnu_cxx17__normal_iteratorIPNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt6vectorIS7_SaIS7_EEEENS0_5__ops15_Iter_comp_iterIZN8argparse8ArgumentC4ILm1EJLm0EEEEOSt5arrayISt17basic_string_viewIcS5_EXT_EESt16integer_sequenceImJXspT0_EEEEUlRKT_RKT0_E_EEEvSP_SP_SS_.isra.0>
    eb20:	e9 64 fc ff ff       	jmp    e789 <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x1e9>
    eb25:	4d 85 e4             	test   %r12,%r12
    eb28:	74 96                	je     eac0 <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x520>
    eb2a:	eb 2b                	jmp    eb57 <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x5b7>
    eb2c:	0f 1f 40 00          	nopl   0x0(%rax)
    eb30:	48 8d 44 24 58       	lea    0x58(%rsp),%rax
    eb35:	31 d2                	xor    %edx,%edx
    eb37:	48 89 c6             	mov    %rax,%rsi
    eb3a:	48 89 ef             	mov    %rbp,%rdi
    eb3d:	48 89 44 24 10       	mov    %rax,0x10(%rsp)
    eb42:	e8 b9 47 ff ff       	call   3300 <_ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE9_M_createERmm@plt>
    eb47:	48 89 45 00          	mov    %rax,0x0(%rbp)
    eb4b:	48 89 c7             	mov    %rax,%rdi
    eb4e:	48 8b 44 24 58       	mov    0x58(%rsp),%rax
    eb53:	48 89 45 10          	mov    %rax,0x10(%rbp)
    eb57:	4c 89 e2             	mov    %r12,%rdx
    eb5a:	4c 89 fe             	mov    %r15,%rsi
    eb5d:	e8 de 45 ff ff       	call   3140 <memcpy@plt>
    eb62:	4c 8b 64 24 58       	mov    0x58(%rsp),%r12
    eb67:	48 8b 7d 00          	mov    0x0(%rbp),%rdi
    eb6b:	e9 50 ff ff ff       	jmp    eac0 <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x520>
    eb70:	48 8d 44 24 58       	lea    0x58(%rsp),%rax
    eb75:	48 8d 3d 84 14 00 00 	lea    0x1484(%rip),%rdi        # 10000 <_fini+0xddf>
    eb7c:	48 89 44 24 10       	mov    %rax,0x10(%rsp)
    eb81:	e8 3a 48 ff ff       	call   33c0 <_ZSt19__throw_logic_errorPKc@plt>
    eb86:	e8 c5 45 ff ff       	call   3150 <__stack_chk_fail@plt>
    eb8b:	48 89 c5             	mov    %rax,%rbp
    eb8e:	49 8d bd c0 00 00 00 	lea    0xc0(%r13),%rdi
    eb95:	c5 f8 77             	vzeroupper 
    eb98:	e8 93 c6 ff ff       	call   b230 <_ZNSt6vectorISt3anySaIS0_EED1Ev>
    eb9d:	41 0f b6 95 b8 00 00 	movzbl 0xb8(%r13),%edx
    eba4:	00 
    eba5:	48 8b 7c 24 10       	mov    0x10(%rsp),%rdi
    ebaa:	48 8d 05 df 4c 00 00 	lea    0x4cdf(%rip),%rax        # 13890 <_ZNSt8__detail9__variant12__gen_vtableIvOZNS0_16_Variant_storageILb0EJSt8functionIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEES3_IFvSC_EEEE8_M_resetEvEUlOT_E_JRSt7variantIJSE_SG_EEEE9_S_vtableE>
    ebb1:	49 8d b5 98 00 00 00 	lea    0x98(%r13),%rsi
    ebb8:	ff 14 d0             	call   *(%rax,%rdx,8)
    ebbb:	49 8b 85 88 00 00 00 	mov    0x88(%r13),%rax
    ebc2:	48 85 c0             	test   %rax,%rax
    ebc5:	74 10                	je     ebd7 <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x637>
    ebc7:	49 8d b5 88 00 00 00 	lea    0x88(%r13),%rsi
    ebce:	31 d2                	xor    %edx,%edx
    ebd0:	bf 03 00 00 00       	mov    $0x3,%edi
    ebd5:	ff d0                	call   *%rax
    ebd7:	49 8b 7d 68          	mov    0x68(%r13),%rdi
    ebdb:	48 39 fb             	cmp    %rdi,%rbx
    ebde:	74 0d                	je     ebed <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x64d>
    ebe0:	49 8b 45 78          	mov    0x78(%r13),%rax
    ebe4:	48 8d 70 01          	lea    0x1(%rax),%rsi
    ebe8:	e8 33 47 ff ff       	call   3320 <_ZdlPvm@plt>
    ebed:	49 8b 45 58          	mov    0x58(%r13),%rax
    ebf1:	48 85 c0             	test   %rax,%rax
    ebf4:	74 0d                	je     ec03 <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x663>
    ebf6:	49 8d 75 58          	lea    0x58(%r13),%rsi
    ebfa:	31 d2                	xor    %edx,%edx
    ebfc:	bf 03 00 00 00       	mov    $0x3,%edi
    ec01:	ff d0                	call   *%rax
    ec03:	49 8b 7d 38          	mov    0x38(%r13),%rdi
    ec07:	48 39 3c 24          	cmp    %rdi,(%rsp)
    ec0b:	74 0d                	je     ec1a <_ZN8argparse14ArgumentParser12add_argumentIJPKcEEERNS_8ArgumentEDpT_+0x67a>
    ec0d:	49 8b 45 48          	mov    0x48(%r13),%rax
    ec11:	48 8d 70 01          	lea    0x1(%rax),%rsi
    ec15:	e8 06 47 ff ff       	call   3320 <_ZdlPvm@plt>
    ec1a:	48 8b 7c 24 38       	mov    0x38(%rsp),%rdi
    ec1f:	e8 ec 9d ff ff       	call   8a10 <_ZNSt6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EED1Ev>
    ec24:	4c 89 ef             	mov    %r13,%rdi
    ec27:	be f0 00 00 00       	mov    $0xf0,%esi
    ec2c:	e8 ef 46 ff ff       	call   3320 <_ZdlPvm@plt>
    ec31:	48 89 ef             	mov    %rbp,%rdi
    ec34:	e8 27 47 ff ff       	call   3360 <_Unwind_Resume@plt>
    ec39:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)

000000000000ec40 <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i>:
    ec40:	41 55                	push   %r13
    ec42:	49 89 fa             	mov    %rdi,%r10
    ec45:	48 89 f7             	mov    %rsi,%rdi
    ec48:	41 54                	push   %r12
    ec4a:	49 89 d3             	mov    %rdx,%r11
    ec4d:	89 ce                	mov    %ecx,%esi
    ec4f:	55                   	push   %rbp
    ec50:	53                   	push   %rbx
    ec51:	4c 39 d7             	cmp    %r10,%rdi
    ec54:	0f 84 16 02 00 00    	je     ee70 <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0x230>
    ec5a:	41 80 3a 2d          	cmpb   $0x2d,(%r10)
    ec5e:	4d 89 d1             	mov    %r10,%r9
    ec61:	bd 01 00 00 00       	mov    $0x1,%ebp
    ec66:	0f 84 2c 01 00 00    	je     ed98 <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0x158>
    ec6c:	83 fe 02             	cmp    $0x2,%esi
    ec6f:	0f 84 37 01 00 00    	je     edac <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0x16c>
    ec75:	83 fe 0a             	cmp    $0xa,%esi
    ec78:	0f 8e b2 00 00 00    	jle    ed30 <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0xf0>
    ec7e:	4c 39 cf             	cmp    %r9,%rdi
    ec81:	0f 84 fb 01 00 00    	je     ee82 <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0x242>
    ec87:	4d 89 c8             	mov    %r9,%r8
    ec8a:	31 d2                	xor    %edx,%edx
    ec8c:	b8 01 00 00 00       	mov    $0x1,%eax
    ec91:	4c 8d 25 88 18 00 00 	lea    0x1888(%rip),%r12        # 10520 <CSWTCH.368>
    ec98:	bb 01 00 00 00       	mov    $0x1,%ebx
    ec9d:	eb 3d                	jmp    ecdc <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0x9c>
    ec9f:	90                   	nop
    eca0:	83 e9 30             	sub    $0x30,%ecx
    eca3:	84 c0                	test   %al,%al
    eca5:	74 2d                	je     ecd4 <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0x94>
    eca7:	45 31 ed             	xor    %r13d,%r13d
    ecaa:	85 f6                	test   %esi,%esi
    ecac:	0f 88 19 02 00 00    	js     eecb <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0x28b>
    ecb2:	89 d0                	mov    %edx,%eax
    ecb4:	f7 e6                	mul    %esi
    ecb6:	0f 80 1d 02 00 00    	jo     eed9 <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0x299>
    ecbc:	89 c2                	mov    %eax,%edx
    ecbe:	31 c0                	xor    %eax,%eax
    ecc0:	45 85 ed             	test   %r13d,%r13d
    ecc3:	75 0f                	jne    ecd4 <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0x94>
    ecc5:	0f b6 c9             	movzbl %cl,%ecx
    ecc8:	01 d1                	add    %edx,%ecx
    ecca:	89 d8                	mov    %ebx,%eax
    eccc:	83 d8 00             	sbb    $0x0,%eax
    eccf:	89 ca                	mov    %ecx,%edx
    ecd1:	83 e0 01             	and    $0x1,%eax
    ecd4:	49 ff c0             	inc    %r8
    ecd7:	4c 39 c7             	cmp    %r8,%rdi
    ecda:	74 36                	je     ed12 <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0xd2>
    ecdc:	45 0f b6 28          	movzbl (%r8),%r13d
    ece0:	44 89 e9             	mov    %r13d,%ecx
    ece3:	41 83 ed 30          	sub    $0x30,%r13d
    ece7:	41 83 fd 09          	cmp    $0x9,%r13d
    eceb:	76 b3                	jbe    eca0 <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0x60>
    eced:	83 e9 41             	sub    $0x41,%ecx
    ecf0:	80 f9 39             	cmp    $0x39,%cl
    ecf3:	0f 87 67 01 00 00    	ja     ee60 <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0x220>
    ecf9:	0f b6 c9             	movzbl %cl,%ecx
    ecfc:	45 0f b6 2c 0c       	movzbl (%r12,%rcx,1),%r13d
    ed01:	44 89 e9             	mov    %r13d,%ecx
    ed04:	44 39 ee             	cmp    %r13d,%esi
    ed07:	7f 9a                	jg     eca3 <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0x63>
    ed09:	4d 39 c1             	cmp    %r8,%r9
    ed0c:	0f 84 70 01 00 00    	je     ee82 <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0x242>
    ed12:	b9 22 00 00 00       	mov    $0x22,%ecx
    ed17:	84 c0                	test   %al,%al
    ed19:	0f 85 0e 01 00 00    	jne    ee2d <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0x1ed>
    ed1f:	5b                   	pop    %rbx
    ed20:	5d                   	pop    %rbp
    ed21:	41 5c                	pop    %r12
    ed23:	89 ca                	mov    %ecx,%edx
    ed25:	4c 89 c0             	mov    %r8,%rax
    ed28:	41 5d                	pop    %r13
    ed2a:	c3                   	ret    
    ed2b:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)
    ed30:	4c 39 cf             	cmp    %r9,%rdi
    ed33:	0f 84 49 01 00 00    	je     ee82 <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0x242>
    ed39:	4d 89 c8             	mov    %r9,%r8
    ed3c:	31 d2                	xor    %edx,%edx
    ed3e:	8d 5e 2f             	lea    0x2f(%rsi),%ebx
    ed41:	41 0f b6 08          	movzbl (%r8),%ecx
    ed45:	80 f9 2f             	cmp    $0x2f,%cl
    ed48:	0f 8e da 00 00 00    	jle    ee28 <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0x1e8>
    ed4e:	0f be c1             	movsbl %cl,%eax
    ed51:	39 d8                	cmp    %ebx,%eax
    ed53:	0f 8f cf 00 00 00    	jg     ee28 <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0x1e8>
    ed59:	45 31 e4             	xor    %r12d,%r12d
    ed5c:	85 f6                	test   %esi,%esi
    ed5e:	0f 88 93 01 00 00    	js     eef7 <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0x2b7>
    ed64:	89 d0                	mov    %edx,%eax
    ed66:	f7 e6                	mul    %esi
    ed68:	0f 80 2d 01 00 00    	jo     ee9b <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0x25b>
    ed6e:	45 85 e4             	test   %r12d,%r12d
    ed71:	0f 85 24 01 00 00    	jne    ee9b <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0x25b>
    ed77:	83 e9 30             	sub    $0x30,%ecx
    ed7a:	0f b6 d1             	movzbl %cl,%edx
    ed7d:	01 c2                	add    %eax,%edx
    ed7f:	0f 82 16 01 00 00    	jb     ee9b <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0x25b>
    ed85:	49 ff c0             	inc    %r8
    ed88:	4c 39 c7             	cmp    %r8,%rdi
    ed8b:	75 b4                	jne    ed41 <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0x101>
    ed8d:	e9 9b 00 00 00       	jmp    ee2d <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0x1ed>
    ed92:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)
    ed98:	4d 8d 4a 01          	lea    0x1(%r10),%r9
    ed9c:	48 c7 c5 ff ff ff ff 	mov    $0xffffffffffffffff,%rbp
    eda3:	83 fe 02             	cmp    $0x2,%esi
    eda6:	0f 85 c9 fe ff ff    	jne    ec75 <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0x35>
    edac:	4c 29 cf             	sub    %r9,%rdi
    edaf:	48 85 ff             	test   %rdi,%rdi
    edb2:	0f 8e ca 00 00 00    	jle    ee82 <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0x242>
    edb8:	31 db                	xor    %ebx,%ebx
    edba:	eb 10                	jmp    edcc <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0x18c>
    edbc:	0f 1f 40 00          	nopl   0x0(%rax)
    edc0:	48 ff c3             	inc    %rbx
    edc3:	48 39 df             	cmp    %rbx,%rdi
    edc6:	0f 84 f4 00 00 00    	je     eec0 <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0x280>
    edcc:	41 80 3c 19 30       	cmpb   $0x30,(%r9,%rbx,1)
    edd1:	4e 8d 04 0b          	lea    (%rbx,%r9,1),%r8
    edd5:	74 e9                	je     edc0 <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0x180>
    edd7:	48 39 fb             	cmp    %rdi,%rbx
    edda:	0f 8d 9d 00 00 00    	jge    ee7d <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0x23d>
    ede0:	49 89 dc             	mov    %rbx,%r12
    ede3:	31 d2                	xor    %edx,%edx
    ede5:	eb 1c                	jmp    ee03 <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0x1c3>
    ede7:	66 0f 1f 84 00 00 00 	nopw   0x0(%rax,%rax,1)
    edee:	00 00 
    edf0:	01 d2                	add    %edx,%edx
    edf2:	0f b6 c9             	movzbl %cl,%ecx
    edf5:	49 ff c4             	inc    %r12
    edf8:	09 ca                	or     %ecx,%edx
    edfa:	4c 39 e7             	cmp    %r12,%rdi
    edfd:	0f 84 e1 00 00 00    	je     eee4 <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0x2a4>
    ee03:	43 0f b6 04 21       	movzbl (%r9,%r12,1),%eax
    ee08:	4f 8d 04 21          	lea    (%r9,%r12,1),%r8
    ee0c:	8d 48 d0             	lea    -0x30(%rax),%ecx
    ee0f:	80 f9 01             	cmp    $0x1,%cl
    ee12:	76 dc                	jbe    edf0 <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0x1b0>
    ee14:	49 29 dc             	sub    %rbx,%r12
    ee17:	49 83 fc 20          	cmp    $0x20,%r12
    ee1b:	0f 9e c0             	setle  %al
    ee1e:	e9 e6 fe ff ff       	jmp    ed09 <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0xc9>
    ee23:	0f 1f 44 00 00       	nopl   0x0(%rax,%rax,1)
    ee28:	4d 39 c8             	cmp    %r9,%r8
    ee2b:	74 55                	je     ee82 <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0x242>
    ee2d:	89 d2                	mov    %edx,%edx
    ee2f:	48 0f af d5          	imul   %rbp,%rdx
    ee33:	b9 22 00 00 00       	mov    $0x22,%ecx
    ee38:	48 63 c2             	movslq %edx,%rax
    ee3b:	89 d6                	mov    %edx,%esi
    ee3d:	48 39 c2             	cmp    %rax,%rdx
    ee40:	0f 85 d9 fe ff ff    	jne    ed1f <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0xdf>
    ee46:	5b                   	pop    %rbx
    ee47:	5d                   	pop    %rbp
    ee48:	31 c9                	xor    %ecx,%ecx
    ee4a:	41 5c                	pop    %r12
    ee4c:	89 ca                	mov    %ecx,%edx
    ee4e:	4c 89 c0             	mov    %r8,%rax
    ee51:	41 89 33             	mov    %esi,(%r11)
    ee54:	41 5d                	pop    %r13
    ee56:	c3                   	ret    
    ee57:	66 0f 1f 84 00 00 00 	nopw   0x0(%rax,%rax,1)
    ee5e:	00 00 
    ee60:	41 bd ff 00 00 00    	mov    $0xff,%r13d
    ee66:	b9 ff ff ff ff       	mov    $0xffffffff,%ecx
    ee6b:	e9 94 fe ff ff       	jmp    ed04 <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0xc4>
    ee70:	49 89 f9             	mov    %rdi,%r9
    ee73:	bd 01 00 00 00       	mov    $0x1,%ebp
    ee78:	e9 ef fd ff ff       	jmp    ec6c <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0x2c>
    ee7d:	4d 39 c8             	cmp    %r9,%r8
    ee80:	75 42                	jne    eec4 <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0x284>
    ee82:	5b                   	pop    %rbx
    ee83:	5d                   	pop    %rbp
    ee84:	4d 89 d0             	mov    %r10,%r8
    ee87:	b9 16 00 00 00       	mov    $0x16,%ecx
    ee8c:	41 5c                	pop    %r12
    ee8e:	89 ca                	mov    %ecx,%edx
    ee90:	4c 89 c0             	mov    %r8,%rax
    ee93:	41 5d                	pop    %r13
    ee95:	c3                   	ret    
    ee96:	80 fa 2f             	cmp    $0x2f,%dl
    ee99:	7e 10                	jle    eeab <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0x26b>
    ee9b:	49 ff c0             	inc    %r8
    ee9e:	4c 39 c7             	cmp    %r8,%rdi
    eea1:	74 0d                	je     eeb0 <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0x270>
    eea3:	41 0f be 10          	movsbl (%r8),%edx
    eea7:	39 d3                	cmp    %edx,%ebx
    eea9:	7d eb                	jge    ee96 <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0x256>
    eeab:	4d 39 c8             	cmp    %r9,%r8
    eeae:	74 d2                	je     ee82 <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0x242>
    eeb0:	b9 22 00 00 00       	mov    $0x22,%ecx
    eeb5:	e9 65 fe ff ff       	jmp    ed1f <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0xdf>
    eeba:	66 0f 1f 44 00 00    	nopw   0x0(%rax,%rax,1)
    eec0:	4d 8d 04 39          	lea    (%r9,%rdi,1),%r8
    eec4:	31 f6                	xor    %esi,%esi
    eec6:	e9 7b ff ff ff       	jmp    ee46 <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0x206>
    eecb:	45 31 ed             	xor    %r13d,%r13d
    eece:	85 d2                	test   %edx,%edx
    eed0:	41 0f 95 c5          	setne  %r13b
    eed4:	e9 d9 fd ff ff       	jmp    ecb2 <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0x72>
    eed9:	41 bd 01 00 00 00    	mov    $0x1,%r13d
    eedf:	e9 d8 fd ff ff       	jmp    ecbc <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0x7c>
    eee4:	4d 8d 04 39          	lea    (%r9,%rdi,1),%r8
    eee8:	48 29 df             	sub    %rbx,%rdi
    eeeb:	48 83 ff 20          	cmp    $0x20,%rdi
    eeef:	0f 9e c0             	setle  %al
    eef2:	e9 12 fe ff ff       	jmp    ed09 <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0xc9>
    eef7:	45 31 e4             	xor    %r12d,%r12d
    eefa:	85 d2                	test   %edx,%edx
    eefc:	41 0f 95 c4          	setne  %r12b
    ef00:	e9 5f fe ff ff       	jmp    ed64 <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i+0x124>
    ef05:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
    ef0c:	00 00 00 
    ef0f:	90                   	nop

000000000000ef10 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIiLi0EEEE9_M_invokeERKSt9_Any_dataS8_>:
    ef10:	41 54                	push   %r12
    ef12:	49 89 fc             	mov    %rdi,%r12
    ef15:	55                   	push   %rbp
    ef16:	53                   	push   %rbx
    ef17:	48 83 ec 10          	sub    $0x10,%rsp
    ef1b:	64 48 8b 04 25 28 00 	mov    %fs:0x28,%rax
    ef22:	00 00 
    ef24:	48 89 44 24 08       	mov    %rax,0x8(%rsp)
    ef29:	48 8b 42 08          	mov    0x8(%rdx),%rax
    ef2d:	48 8b 3a             	mov    (%rdx),%rdi
    ef30:	48 83 f8 01          	cmp    $0x1,%rax
    ef34:	0f 86 86 00 00 00    	jbe    efc0 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIiLi0EEEE9_M_invokeERKSt9_Any_dataS8_+0xb0>
    ef3a:	66 81 3f 30 78       	cmpw   $0x7830,(%rdi)
    ef3f:	0f 85 3b 01 00 00    	jne    f080 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIiLi0EEEE9_M_invokeERKSt9_Any_dataS8_+0x170>
    ef45:	48 8d 1c 07          	lea    (%rdi,%rax,1),%rbx
    ef49:	48 8d 54 24 04       	lea    0x4(%rsp),%rdx
    ef4e:	48 83 c7 02          	add    $0x2,%rdi
    ef52:	b9 10 00 00 00       	mov    $0x10,%ecx
    ef57:	48 89 de             	mov    %rbx,%rsi
    ef5a:	e8 e1 fc ff ff       	call   ec40 <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i>
    ef5f:	85 d2                	test   %edx,%edx
    ef61:	0f 84 59 01 00 00    	je     f0c0 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIiLi0EEEE9_M_invokeERKSt9_Any_dataS8_+0x1b0>
    ef67:	83 fa 16             	cmp    $0x16,%edx
    ef6a:	0f 84 1d 02 00 00    	je     f18d <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIiLi0EEEE9_M_invokeERKSt9_Any_dataS8_+0x27d>
    ef70:	83 fa 22             	cmp    $0x22,%edx
    ef73:	0f 84 b7 01 00 00    	je     f130 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIiLi0EEEE9_M_invokeERKSt9_Any_dataS8_+0x220>
    ef79:	8b 44 24 04          	mov    0x4(%rsp),%eax
    ef7d:	48 8d 0d 7c 72 ff ff 	lea    -0x8d84(%rip),%rcx        # 6200 <_ZNSt3any17_Manager_internalIiE9_S_manageENS_3_OpEPKS_PNS_4_ArgE>
    ef84:	49 c7 44 24 08 00 00 	movq   $0x0,0x8(%r12)
    ef8b:	00 00 
    ef8d:	49 89 0c 24          	mov    %rcx,(%r12)
    ef91:	41 89 44 24 08       	mov    %eax,0x8(%r12)
    ef96:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
    ef9b:	64 48 2b 04 25 28 00 	sub    %fs:0x28,%rax
    efa2:	00 00 
    efa4:	0f 85 81 01 00 00    	jne    f12b <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIiLi0EEEE9_M_invokeERKSt9_Any_dataS8_+0x21b>
    efaa:	48 83 c4 10          	add    $0x10,%rsp
    efae:	5b                   	pop    %rbx
    efaf:	5d                   	pop    %rbp
    efb0:	4c 89 e0             	mov    %r12,%rax
    efb3:	41 5c                	pop    %r12
    efb5:	c3                   	ret    
    efb6:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
    efbd:	00 00 00 
    efc0:	48 8d 1c 07          	lea    (%rdi,%rax,1),%rbx
    efc4:	48 85 c0             	test   %rax,%rax
    efc7:	74 67                	je     f030 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIiLi0EEEE9_M_invokeERKSt9_Any_dataS8_+0x120>
    efc9:	80 3f 30             	cmpb   $0x30,(%rdi)
    efcc:	75 62                	jne    f030 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIiLi0EEEE9_M_invokeERKSt9_Any_dataS8_+0x120>
    efce:	48 8d 54 24 04       	lea    0x4(%rsp),%rdx
    efd3:	b9 08 00 00 00       	mov    $0x8,%ecx
    efd8:	48 89 de             	mov    %rbx,%rsi
    efdb:	e8 60 fc ff ff       	call   ec40 <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i>
    efe0:	85 d2                	test   %edx,%edx
    efe2:	0f 84 08 01 00 00    	je     f0f0 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIiLi0EEEE9_M_invokeERKSt9_Any_dataS8_+0x1e0>
    efe8:	83 fa 16             	cmp    $0x16,%edx
    efeb:	0f 84 60 01 00 00    	je     f151 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIiLi0EEEE9_M_invokeERKSt9_Any_dataS8_+0x241>
    eff1:	83 fa 22             	cmp    $0x22,%edx
    eff4:	75 83                	jne    ef79 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIiLi0EEEE9_M_invokeERKSt9_Any_dataS8_+0x69>
    eff6:	bf 10 00 00 00       	mov    $0x10,%edi
    effb:	e8 d0 40 ff ff       	call   30d0 <__cxa_allocate_exception@plt>
    f000:	48 89 c7             	mov    %rax,%rdi
    f003:	48 8d 35 f2 10 00 00 	lea    0x10f2(%rip),%rsi        # 100fc <_fini+0xedb>
    f00a:	49 89 c4             	mov    %rax,%r12
    f00d:	e8 4e 40 ff ff       	call   3060 <_ZNSt11range_errorC1EPKc@plt>
    f012:	48 8b 15 bf 4f 00 00 	mov    0x4fbf(%rip),%rdx        # 13fd8 <_ZNSt11range_errorD1Ev@Base>
    f019:	48 8d 35 a0 48 00 00 	lea    0x48a0(%rip),%rsi        # 138c0 <_ZTISt11range_error@@Base>
    f020:	4c 89 e7             	mov    %r12,%rdi
    f023:	e8 f8 41 ff ff       	call   3220 <__cxa_throw@plt>
    f028:	0f 1f 84 00 00 00 00 	nopl   0x0(%rax,%rax,1)
    f02f:	00 
    f030:	48 8d 54 24 04       	lea    0x4(%rsp),%rdx
    f035:	b9 0a 00 00 00       	mov    $0xa,%ecx
    f03a:	48 89 de             	mov    %rbx,%rsi
    f03d:	e8 fe fb ff ff       	call   ec40 <_ZSt10from_charsIiENSt9enable_ifIXsrSt5__or_IJS1_IJSt7is_sameINSt9remove_cvIT_E4typeEaES2_IS6_sES2_IS6_iES2_IS6_lES2_IS6_xES2_IS6_nEEES1_IJS2_IS6_hES2_IS6_tES2_IS6_jES2_IS6_mES2_IS6_yES2_IS6_oEEES2_IcS6_EEE5valueESt17from_chars_resultE4typeEPKcSR_RS4_i>
    f042:	85 d2                	test   %edx,%edx
    f044:	74 52                	je     f098 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIiLi0EEEE9_M_invokeERKSt9_Any_dataS8_+0x188>
    f046:	83 fa 16             	cmp    $0x16,%edx
    f049:	0f 84 20 01 00 00    	je     f16f <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIiLi0EEEE9_M_invokeERKSt9_Any_dataS8_+0x25f>
    f04f:	83 fa 22             	cmp    $0x22,%edx
    f052:	0f 85 21 ff ff ff    	jne    ef79 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIiLi0EEEE9_M_invokeERKSt9_Any_dataS8_+0x69>
    f058:	bf 10 00 00 00       	mov    $0x10,%edi
    f05d:	e8 6e 40 ff ff       	call   30d0 <__cxa_allocate_exception@plt>
    f062:	48 89 c7             	mov    %rax,%rdi
    f065:	48 8d 35 90 10 00 00 	lea    0x1090(%rip),%rsi        # 100fc <_fini+0xedb>
    f06c:	49 89 c4             	mov    %rax,%r12
    f06f:	e8 ec 3f ff ff       	call   3060 <_ZNSt11range_errorC1EPKc@plt>
    f074:	eb 9c                	jmp    f012 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIiLi0EEEE9_M_invokeERKSt9_Any_dataS8_+0x102>
    f076:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
    f07d:	00 00 00 
    f080:	66 81 3f 30 58       	cmpw   $0x5830,(%rdi)
    f085:	0f 84 ba fe ff ff    	je     ef45 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIiLi0EEEE9_M_invokeERKSt9_Any_dataS8_+0x35>
    f08b:	48 8d 1c 07          	lea    (%rdi,%rax,1),%rbx
    f08f:	e9 35 ff ff ff       	jmp    efc9 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIiLi0EEEE9_M_invokeERKSt9_Any_dataS8_+0xb9>
    f094:	0f 1f 40 00          	nopl   0x0(%rax)
    f098:	48 39 c3             	cmp    %rax,%rbx
    f09b:	0f 84 d8 fe ff ff    	je     ef79 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIiLi0EEEE9_M_invokeERKSt9_Any_dataS8_+0x69>
    f0a1:	bf 10 00 00 00       	mov    $0x10,%edi
    f0a6:	e8 25 40 ff ff       	call   30d0 <__cxa_allocate_exception@plt>
    f0ab:	48 89 c7             	mov    %rax,%rdi
    f0ae:	48 8d 35 13 10 00 00 	lea    0x1013(%rip),%rsi        # 100c8 <_fini+0xea7>
    f0b5:	49 89 c4             	mov    %rax,%r12
    f0b8:	e8 33 40 ff ff       	call   30f0 <_ZNSt16invalid_argumentC1EPKc@plt>
    f0bd:	eb 56                	jmp    f115 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIiLi0EEEE9_M_invokeERKSt9_Any_dataS8_+0x205>
    f0bf:	90                   	nop
    f0c0:	48 39 c3             	cmp    %rax,%rbx
    f0c3:	0f 84 b0 fe ff ff    	je     ef79 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIiLi0EEEE9_M_invokeERKSt9_Any_dataS8_+0x69>
    f0c9:	bf 10 00 00 00       	mov    $0x10,%edi
    f0ce:	e8 fd 3f ff ff       	call   30d0 <__cxa_allocate_exception@plt>
    f0d3:	48 89 c7             	mov    %rax,%rdi
    f0d6:	48 8d 35 eb 0f 00 00 	lea    0xfeb(%rip),%rsi        # 100c8 <_fini+0xea7>
    f0dd:	49 89 c4             	mov    %rax,%r12
    f0e0:	e8 0b 40 ff ff       	call   30f0 <_ZNSt16invalid_argumentC1EPKc@plt>
    f0e5:	eb 2e                	jmp    f115 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIiLi0EEEE9_M_invokeERKSt9_Any_dataS8_+0x205>
    f0e7:	66 0f 1f 84 00 00 00 	nopw   0x0(%rax,%rax,1)
    f0ee:	00 00 
    f0f0:	48 39 c3             	cmp    %rax,%rbx
    f0f3:	0f 84 80 fe ff ff    	je     ef79 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIiLi0EEEE9_M_invokeERKSt9_Any_dataS8_+0x69>
    f0f9:	bf 10 00 00 00       	mov    $0x10,%edi
    f0fe:	e8 cd 3f ff ff       	call   30d0 <__cxa_allocate_exception@plt>
    f103:	48 89 c7             	mov    %rax,%rdi
    f106:	48 8d 35 bb 0f 00 00 	lea    0xfbb(%rip),%rsi        # 100c8 <_fini+0xea7>
    f10d:	49 89 c4             	mov    %rax,%r12
    f110:	e8 db 3f ff ff       	call   30f0 <_ZNSt16invalid_argumentC1EPKc@plt>
    f115:	48 8b 15 94 4e 00 00 	mov    0x4e94(%rip),%rdx        # 13fb0 <_ZNSt16invalid_argumentD1Ev@Base>
    f11c:	48 8d 35 f5 49 00 00 	lea    0x49f5(%rip),%rsi        # 13b18 <_ZTISt16invalid_argument@@Base>
    f123:	4c 89 e7             	mov    %r12,%rdi
    f126:	e8 f5 40 ff ff       	call   3220 <__cxa_throw@plt>
    f12b:	e8 20 40 ff ff       	call   3150 <__stack_chk_fail@plt>
    f130:	bf 10 00 00 00       	mov    $0x10,%edi
    f135:	e8 96 3f ff ff       	call   30d0 <__cxa_allocate_exception@plt>
    f13a:	48 89 c7             	mov    %rax,%rdi
    f13d:	48 8d 35 b8 0f 00 00 	lea    0xfb8(%rip),%rsi        # 100fc <_fini+0xedb>
    f144:	49 89 c4             	mov    %rax,%r12
    f147:	e8 14 3f ff ff       	call   3060 <_ZNSt11range_errorC1EPKc@plt>
    f14c:	e9 c1 fe ff ff       	jmp    f012 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIiLi0EEEE9_M_invokeERKSt9_Any_dataS8_+0x102>
    f151:	bf 10 00 00 00       	mov    $0x10,%edi
    f156:	e8 75 3f ff ff       	call   30d0 <__cxa_allocate_exception@plt>
    f15b:	48 89 c7             	mov    %rax,%rdi
    f15e:	48 8d 35 85 0f 00 00 	lea    0xf85(%rip),%rsi        # 100ea <_fini+0xec9>
    f165:	49 89 c4             	mov    %rax,%r12
    f168:	e8 83 3f ff ff       	call   30f0 <_ZNSt16invalid_argumentC1EPKc@plt>
    f16d:	eb a6                	jmp    f115 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIiLi0EEEE9_M_invokeERKSt9_Any_dataS8_+0x205>
    f16f:	bf 10 00 00 00       	mov    $0x10,%edi
    f174:	e8 57 3f ff ff       	call   30d0 <__cxa_allocate_exception@plt>
    f179:	48 89 c7             	mov    %rax,%rdi
    f17c:	48 8d 35 67 0f 00 00 	lea    0xf67(%rip),%rsi        # 100ea <_fini+0xec9>
    f183:	49 89 c4             	mov    %rax,%r12
    f186:	e8 65 3f ff ff       	call   30f0 <_ZNSt16invalid_argumentC1EPKc@plt>
    f18b:	eb 88                	jmp    f115 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIiLi0EEEE9_M_invokeERKSt9_Any_dataS8_+0x205>
    f18d:	bf 10 00 00 00       	mov    $0x10,%edi
    f192:	e8 39 3f ff ff       	call   30d0 <__cxa_allocate_exception@plt>
    f197:	48 89 c7             	mov    %rax,%rdi
    f19a:	48 8d 35 49 0f 00 00 	lea    0xf49(%rip),%rsi        # 100ea <_fini+0xec9>
    f1a1:	49 89 c4             	mov    %rax,%r12
    f1a4:	e8 47 3f ff ff       	call   30f0 <_ZNSt16invalid_argumentC1EPKc@plt>
    f1a9:	e9 67 ff ff ff       	jmp    f115 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIiLi0EEEE9_M_invokeERKSt9_Any_dataS8_+0x205>
    f1ae:	48 89 c5             	mov    %rax,%rbp
    f1b1:	eb 10                	jmp    f1c3 <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIiLi0EEEE9_M_invokeERKSt9_Any_dataS8_+0x2b3>
    f1b3:	eb f9                	jmp    f1ae <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIiLi0EEEE9_M_invokeERKSt9_Any_dataS8_+0x29e>
    f1b5:	eb f7                	jmp    f1ae <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIiLi0EEEE9_M_invokeERKSt9_Any_dataS8_+0x29e>
    f1b7:	eb f5                	jmp    f1ae <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIiLi0EEEE9_M_invokeERKSt9_Any_dataS8_+0x29e>
    f1b9:	eb f3                	jmp    f1ae <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIiLi0EEEE9_M_invokeERKSt9_Any_dataS8_+0x29e>
    f1bb:	eb f1                	jmp    f1ae <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIiLi0EEEE9_M_invokeERKSt9_Any_dataS8_+0x29e>
    f1bd:	eb ef                	jmp    f1ae <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIiLi0EEEE9_M_invokeERKSt9_Any_dataS8_+0x29e>
    f1bf:	eb ed                	jmp    f1ae <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIiLi0EEEE9_M_invokeERKSt9_Any_dataS8_+0x29e>
    f1c1:	eb eb                	jmp    f1ae <_ZNSt17_Function_handlerIFSt3anyRKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEEN8argparse7details12parse_numberIiLi0EEEE9_M_invokeERKSt9_Any_dataS8_+0x29e>
    f1c3:	4c 89 e7             	mov    %r12,%rdi
    f1c6:	c5 f8 77             	vzeroupper 
    f1c9:	e8 e2 41 ff ff       	call   33b0 <__cxa_free_exception@plt>
    f1ce:	48 89 ef             	mov    %rbp,%rdi
    f1d1:	e8 8a 41 ff ff       	call   3360 <_Unwind_Resume@plt>
    f1d6:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
    f1dd:	00 00 00 

000000000000f1e0 <__do_global_ctors_aux>:
    f1e0:	48 8b 05 59 46 00 00 	mov    0x4659(%rip),%rax        # 13840 <__CTOR_LIST__>
    f1e7:	48 83 f8 ff          	cmp    $0xffffffffffffffff,%rax
    f1eb:	74 33                	je     f220 <__do_global_ctors_aux+0x40>
    f1ed:	55                   	push   %rbp
    f1ee:	48 89 e5             	mov    %rsp,%rbp
    f1f1:	53                   	push   %rbx
    f1f2:	48 8d 1d 47 46 00 00 	lea    0x4647(%rip),%rbx        # 13840 <__CTOR_LIST__>
    f1f9:	48 83 ec 08          	sub    $0x8,%rsp
    f1fd:	0f 1f 00             	nopl   (%rax)
    f200:	ff d0                	call   *%rax
    f202:	48 8b 43 f8          	mov    -0x8(%rbx),%rax
    f206:	48 83 eb 08          	sub    $0x8,%rbx
    f20a:	48 83 f8 ff          	cmp    $0xffffffffffffffff,%rax
    f20e:	75 f0                	jne    f200 <__do_global_ctors_aux+0x20>
    f210:	48 8b 5d f8          	mov    -0x8(%rbp),%rbx
    f214:	c9                   	leave  
    f215:	c3                   	ret    
    f216:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
    f21d:	00 00 00 
    f220:	c3                   	ret    

Disassembly of section .fini:

000000000000f221 <_fini>:
    f221:	50                   	push   %rax
    f222:	e8 e9 52 ff ff       	call   4510 <__do_global_dtors_aux>
    f227:	58                   	pop    %rax
    f228:	c3                   	ret    
