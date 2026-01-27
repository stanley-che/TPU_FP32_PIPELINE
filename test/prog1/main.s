.data
test1: .word 16,2,4,16,4,10,12,2,14,8,4,14,6,4,2,10,12,6,10,2,14,14,6,8,16,8,16,6,12,10,8,123
test2: .word 470,405,225,197,126,122,56,33,-81,-275,-379,-409,-416,-496,-500
test3: .word 412,-474,443,171,-23,247,221,7,40,221,-90,61,-9,49,-80,-80,221,-379,-161,-397,-173,276,-197,221,-12,-145,101
TEST1_SIZE: .word 32
TEST2_SIZE: .word 15
TEST3_SIZE: .word 27

.text
.globl main
main:
    addi    sp, sp, -4
    sw      s0, 0(sp)
    la      s0, _answer
    ###############################################################
    # copy test data to sorted array                              #
    # t0 : target pointer                                         #
    # t1 : source pointer                                         #
    # t2 : temp                                                   #
    # t3 : inner counter                                          #
    # t4 : TESTX_SIZE address                                     #
    # t5 : outer counter                                          #
    # t6 : target array base pointer                              #
    ###############################################################

    # move array to Stored address
    li      t6, 0x9000          # set target array base pointer
    mv      t0, t6              # set target pointer
    la      t1, test1           # get source pointer
    li      t5, 3               # ounter counter = test array number
    la      t4, TEST1_SIZE      # t4 = &TESTX_SIZE
loop_move_outer:
    lw      t3, 0(t4)           # inner counter = TESTX_SIZE
loop_move_inner:
    lw      t2, 0(t1)           # get stored data
    sw      t2, 0(t0)           # stored to target address
    addi    t0, t0, 4           # target pointer to next
    addi    t1, t1, 4           # source pointer to next
    addi    t3, t3, -1          # inner counter--
    bnez    t3, loop_move_inner # if inner counter != 0, goto next inner loop
end_loop_move_inner:
    lw      t2, 0(t4)           # t2 = TEXTX_SIZE
    slli    t2, t2, 2           # t2 = 4 * TEXTX_SIZE
    add     t6, t6, t2          # t6 += 4 * TRSTX_SIZE
                                # t6 = next target address
    mv      t0, t6              # set next target address
    ## addi    t1, t1, 4           # set next source address
    addi    t4, t4, 4           # next TESTX_SIZE address
    addi    t5, t5, -1          # outter counter--
    bnez    t5, loop_move_outer # if outer counter != 0, goto next outer loop
end_loop_move_outer:

    #################################################################
    # execute merge sort                                            #
    # s0 : target array pointer                                     #
    # s1 : Array size pointer                                       #
    # s2 : counter                                                  #
    # s3 : TESTx_SIZE                                               #
    #################################################################

    # pointer initialize
    li      s0, 0x9000          # set target array address
    la      s1, TEST1_SIZE      # s1 = &TEST1_SIZE
    # loop variable initialize
    li      s2, 3               # counter = 3
    # run the program
loop_main:
    lw      s3, 0(s1)           # s3 =  TESTX_SIZE
Call_mergesort:
    # Caller saved
    addi    sp, sp, -4          # allocate one word from stack
    sw      ra, 0(sp)           # push ra to stack
    # pass parameter
    mv      a0, s0              # arr = s0
    li      a1, 0               # start = 0
    mv      a2, s3              # a2 = TESTx_SIZE
    addi    a2, a2, -1          # end = TESTx_SIZE - 1
    # jump to mergesort
    jal     ra, mergesort
    # Retrieve caller saved
    lw      ra, 0(sp)           # pop ra from stack
    addi    sp, sp, 4           # free one word from stack
end_Call_mergesort:
    # goto next array
    # li      t0, 0x00001000      # t0 = 0x00001000
    mv      t2, s3
    slli    t2, t2, 2
    add     s0, s0, t2          # s0 += 4 * TESTX_size, goto next sorted array
    addi    s1, s1, 4           # next TEST_X address
    # counter operation and check condition
    addi    s2, s2, -1           # counter--
    bnez    s2, loop_main        # if counter > 0, goto next loop_main
end_loop_main:
    # ret
    # end main function
main_exit:
    /* Simulation End */
    lw      s0, 0(sp)
    addi    sp, sp, 4
    ret

mergesort:
    #################################################################
    # separate the array to small pieces                            #
    # param a0 : int *arr                                           #
    # param a1 : int start                                          #
    # param a2 : int end                                            #
    # s0 : mid                                                      #
    # return : void                                                 #
    #################################################################

    # check if condition first
    bge a1, a2, end_mergesort       # if start >= end, return back to caller function
    # Callee saved
    addi    sp, sp, -16
    sw      s0, 12(sp)
    sw      s1, 8(sp)
    sw      s2, 4(sp)
    sw      s3, 0(sp)

    # function statement
    # int mid = (end + start) / 2
    add     s0, a1, a2              # s0 = a1 + a2
    srai    s0, s0, 1               # mid = (a1 + a2) / 2
call_mergesort_0:
    # Call mergesort function
    # Caller saved
    addi    sp, sp, -16
    sw      ra, 12(sp)
    sw      a0, 8(sp)
    sw      a1, 4(sp)
    sw      a2, 0(sp)

    # Pass parameter
    #mv    a0, a0                   # a0 = arr
                                    # param does not change
    #mv    a1, a1                   # a1 = start
                                    # param does not change
    mv      a2, s0                  # a2 = mid

    # jump to mergesort
    jal     ra, mergesort

    # Retrieve caller saved
    lw      a2, 0(sp)
    lw      a1, 4(sp)
    lw      a0, 8(sp)
    lw      ra, 12(sp)
    ### keep data in the stack
    #addi    sp, sp, 16

call_mergesort_1:
    # Call mergesort function
    # Caller saved
    #addi    sp, sp, -16
    #sw      ra, 12(sp)
    #sw      a0, 8(sp)
    #sw      a1, 4(sp)
    #sw      a2, 0(sp)

    # Pass parameter
    #mv    a0, a0                   # a0 = arr
                                    # param does not change
    addi    a1, s0, 1               # a1 = mid + 1
    #mv      a2, a2                 # a2 = end
                                    # param does not change

    # jump to mergesort
    jal     ra, mergesort

    # Retrieve caller saved
    lw      a2, 0(sp)
    lw      a1, 4(sp)
    lw      a0, 8(sp)
    lw      ra, 12(sp)
    ### keep data in the stack
    #addi    sp, sp, 16

call_merge:
    # Call merge function
    # Caller saved
    #addi    sp, sp, -16
    #sw      ra, 12(sp)
    #sw      a0, 8(sp)
    #sw      a1, 4(sp)
    #sw      a2, 0(sp)

    # Pass parameter
    #mv      a0, a0                 # a0 = arr
                                    # param does not change
    #mv      a1, a1                 # a1 = start
                                    # param does not change
    mv      a3, a2                  # a3 = end
                                    # do this line first than load value to a2
    mv      a2, s0                  # a2 = mid

    # jump to merge
    jal     ra, merge

    # Retrieve caller saved
    lw      a2, 0(sp)
    lw      a1, 4(sp)
    lw      a0, 8(sp)
    lw      ra, 12(sp)
    addi    sp, sp, 16

    # Retrieve callee saved
    lw      s3, 0(sp)
    lw      s2, 4(sp)
    lw      s1, 8(sp)
    lw      s0, 12(sp)
    addi    sp, sp, 16

end_mergesort:
    ret

merge:
    #################################################################
    # merge the input array                                         #
    # param a0 : int *arr                                           #
    # param a1 : int start                                          #
    # param a2 : int mid                                            #
    # param a3 : int end                                            #
    # s0 : int temp_size                                            #
    # s1 : int *temp                                                #
    # s2 : int i                                                    #
    # s3 : int left_index                                           #
    # s4 : int right_index                                          #
    # s5 : int left_max                                             #
    # s6 : int right_max                                            #
    # s7 : int arr_index                                            #
    # return : void                                                 #
    #################################################################

    # Callee saved
    addi    sp, sp, -32
    sw      s0, 28(sp)
    sw      s1, 24(sp)
    sw      s2, 20(sp)
    sw      s3, 16(sp)
    sw      s4, 12(sp)
    sw      s5, 8(sp)
    sw      s6, 4(sp)
    sw      s7, 0(sp)

    # int temp_size = end - start + 1;
    sub     s0, a3, a1              # s0 = end - start
    addi    s0, s0, 1               # temp_size = s0 = end - start + 1
    # int temp[temp_size];
    li      s1, 0xa000          # s1 = &temp[0] = 0xa000
    # for loop
    li      s2, 0                   # i = 0
    bge     s2, s0, end_loop_for    # if s2 >= temp_size, skip for loop
loop_for:
    # for statement
    # temp[i] = arr[i + start];
    slli    t0, s2, 2               # t0 = i * 4
    add     t0, t0, s1              # t0 = &temp[i]
    add     t1, s2, a1              # t1 = i + start
    slli    t1, t1, 2               # t1 = 4(i + start)
    add     t1, t1, a0              # t1 = &arr[i + start]
    lw      t1, 0(t1)               # t1 = arr[i + start]
    sw      t1, 0(t0)               # temp[i] = arr[i + start]
    # end for statement
    addi    s2, s2, 1               # i++
    blt     s2, s0, loop_for        # if i < temp_size, goto loop_for
end_loop_for:
    # int left_index = 0;
    li      s3, 0                   # left_index = 0
    # int right_index = mid - start + 1;
    sub     s4, a2, a1              # s4 = mid - start
    addi    s4, s4, 1               # right_index = mid - start + 1
    # int left_max = mid - start;
    sub     s5, a2, a1              # left_max = mid - start
    # int right_max = end - start;
    sub     s6, a3, a1              # right_max = end - start
    # int arr_index = start;
    mv      s7, a1                  # arr_index = start
    # while0 loop
    # while0 condition check
    bgt     s3, s5, end_loop_while0  # if left_index > left_max, goto end_loop_while0
    bgt     s4, s6, end_loop_while0  # if right_index > right_max, goto end_loop_while0
loop_while0:
    # if condition check
    slli    t0, s3, 2               # t0 = left_index * 4
    add     t0, s1, t0              # t0 = &temp[left_index]
    lw      t0, 0(t0)               # t0 = temp[left_index]
    slli    t1, s4, 2               # t1 = right_index * 4
    add     t1, s1, t1              # t1 = &temp[right_index]
    lw      t1, 0(t1)               # t1 = temp[right_index]
    bgt     t0, t1, else_block      # if temp[left_index] > temp[right_index], goto else block
if_block:
    # arr[arr_index] = temp[left_index];
    slli    t1, s7, 2               # t1 = arr_index * 4
    add     t1, a0, t1              # t1 = & arr[arr_index]
    sw      t0, 0(t1)               # arr[arr_index] = temp[left_index]
    # arr_index++;
    addi    s7, s7, 1               # arr_index++
    # left_index++;
    addi    s3, s3, 1               # left_index++
    j       end_if                  # goto end if
else_block:
    # arr[arr_index] = temp[right_index];
    slli    t0, s7, 2               # t0 = arr_index * 4
    add     t0, a0, t0              # t0 = &arr[arr_index]
    sw      t1, 0(t0)               # arr[arr_index] = temp[right_index]
    # arr_index++;
    addi    s7, s7, 1               # arr_index++
    # right_index++;
    addi    s4, s4, 1               # right_index++
end_if:
    # while0 condition check
    bgt     s3, s5, end_loop_while0  # if left_index > left_max, goto end_loop_while0
    bgt     s4, s6, end_loop_while0  # if right_index > right_max, goto end_loop_while0
    j       loop_while0
end_loop_while0:

    # while1 loop
    # while1 condition check
    bgt     s3, s5, end_loop_while1  # if left_index > left_max, goto end_loop_while1
loop_while1:
    # while1 statement
    # arr[arr_index] = temp[left_index];
    slli    t0, s7, 2               # t0 = arr_index * 4
    add     t0, a0, t0              # t0 = &arr[arr_index]
    slli    t1, s3, 2               # t1 = left_index * 4
    add     t1, s1, t1              # t1 = &temp[left_index]
    lw      t1, 0(t1)               # t1 = temp[left_index]
    sw      t1, 0(t0)               # arr[arr_index] = temp[left_index]
    # arr_index++;
    addi    s7, s7, 1               # arr_index++
    # left_index++;
    addi    s3, s3, 1               # left_index++
    # while 1 condition check
    ble     s3, s5, loop_while1     # if left_index <= left_max, goto loop_while1
end_loop_while1:

    # while2 loop
    # while 2 condition check
    bgt     s4, s6, end_loop_while2      # if right_index > right_max, skip while2
loop_while2:
    # while2 statement
    # arr[arr_index] = temp[left_index];
    slli    t0, s7, 2               # t0 = arr_index * 4
    add     t0, a0, t0              # t0 = &arr[arr_index]
    slli    t1, s4, 2               # t1 = right_index * 4
    add     t1, s1, t1              # t1 = &temp[right_index]
    lw      t1, 0(t1)               # t1 = temp[right_index]
    sw      t1, 0(t0)               # arr[arr_index] = temp[right_index]
    # arr_index++;
    addi    s7, s7, 1               # arr_index++
    # right_index++;
    addi    s4, s4, 1               # right_index++
    # while 1 condition check
    ble     s4, s6, loop_while2     # if right_index <= left_max, goto loop_while2
end_loop_while2:

    # Retrieve callee saved
    lw      s7, 0(sp)
    lw      s6, 4(sp)
    lw      s5, 8(sp)
    lw      s4, 12(sp)
    lw      s3, 16(sp)
    lw      s2, 20(sp)
    lw      s1, 24(sp)
    lw      s0, 28(sp)
    addi    sp, sp, 32
end_merge:
    ret