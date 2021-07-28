
coremark.elf:     file format elf32-bigmips


Disassembly of section .text:

80000400 <_start>:
80000400:	3c088000 	lui	t0,0x8000
80000404:	8d080318 	lw	t0,792(t0)
80000408:	3c097fff 	lui	t1,0x7fff
8000040c:	3529fff0 	ori	t1,t1,0xfff0
80000410:	0109e821 	addu	sp,t0,t1
80000414:	3c1c8002 	lui	gp,0x8002
80000418:	279c5468 	addiu	gp,gp,21608
8000041c:	24020008 	li	v0,8
80000420:	3c01bfc0 	lui	at,0xbfc0
80000424:	ac2207fc 	sw	v0,2044(at)
80000428:	3c022410 	lui	v0,0x2410
8000042c:	344200e0 	ori	v0,v0,0xe0
80000430:	40826000 	mtc0	v0,c0_sr
80000434:	40806800 	mtc0	zero,c0_cause
80000438:	3c048000 	lui	a0,0x8000
8000043c:	24840560 	addiu	a0,a0,1376
80000440:	3c058002 	lui	a1,0x8002
80000444:	24a5d548 	addiu	a1,a1,-10936
80000448:	3c018000 	lui	at,0x8000
8000044c:	34210400 	ori	at,at,0x400
80000450:	00813023 	subu	a2,a0,at
80000454:	3c01b000 	lui	at,0xb000
80000458:	34211000 	ori	at,at,0x1000
8000045c:	00c13021 	addu	a2,a2,at

80000460 <data_init>:
80000460:	8cc80000 	lw	t0,0(a2)
80000464:	24c60004 	addiu	a2,a2,4
80000468:	ac880000 	sw	t0,0(a0)
8000046c:	24840004 	addiu	a0,a0,4
80000470:	0085082b 	sltu	at,a0,a1
80000474:	1420fffa 	bnez	at,80000460 <data_init>
80000478:	00000000 	nop
8000047c:	3c048000 	lui	a0,0x8000
80000480:	24840560 	addiu	a0,a0,1376
80000484:	3c058002 	lui	a1,0x8002
80000488:	24a5d548 	addiu	a1,a1,-10936
8000048c:	00a42822 	sub	a1,a1,a0
80000490:	0c00644a 	jal	80019128 <data_cache_hit_writeback_invalidate>
80000494:	00000000 	nop
80000498:	3c048002 	lui	a0,0x8002
8000049c:	2484d548 	addiu	a0,a0,-10936
800004a0:	3c058003 	lui	a1,0x8003
800004a4:	24a5e2f8 	addiu	a1,a1,-7432

800004a8 <bss_init>:
800004a8:	fc800000 	sd	zero,0(a0)
800004ac:	24840008 	addiu	a0,a0,8
800004b0:	0085082b 	sltu	at,a0,a1
800004b4:	1420fffc 	bnez	at,800004a8 <bss_init>
800004b8:	00000000 	nop
800004bc:	3c048002 	lui	a0,0x8002
800004c0:	2484d548 	addiu	a0,a0,-10936
800004c4:	3c058003 	lui	a1,0x8003
800004c8:	24a5e2f8 	addiu	a1,a1,-7432
800004cc:	00a42822 	sub	a1,a1,a0
800004d0:	0c00644a 	jal	80019128 <data_cache_hit_writeback_invalidate>
800004d4:	00000000 	nop
800004d8:	3c088000 	lui	t0,0x8000
800004dc:	25080550 	addiu	t0,t0,1360
800004e0:	3c09a000 	lui	t1,0xa000
800004e4:	240a0004 	li	t2,4

800004e8 <loadintvectorloop>:
800004e8:	8d0b0000 	lw	t3,0(t0)
800004ec:	ad2b0000 	sw	t3,0(t1)
800004f0:	ad2b0080 	sw	t3,128(t1)
800004f4:	ad2b0100 	sw	t3,256(t1)
800004f8:	ad2b0180 	sw	t3,384(t1)
800004fc:	bd300000 	cache	0x10,0(t1)
80000500:	bd300080 	cache	0x10,128(t1)
80000504:	bd300100 	cache	0x10,256(t1)
80000508:	bd300180 	cache	0x10,384(t1)
8000050c:	21080004 	addi	t0,t0,4
80000510:	21290004 	addi	t1,t1,4
80000514:	254affff 	addiu	t2,t2,-1
80000518:	1540fff3 	bnez	t2,800004e8 <loadintvectorloop>
8000051c:	00000000 	nop
80000520:	3c088001 	lui	t0,0x8001
80000524:	25087c08 	addiu	t0,t0,31752
80000528:	3c098002 	lui	t1,0x8002
8000052c:	2529d818 	addiu	t1,t1,-10216
80000530:	ad280000 	sw	t0,0(t1)
80000534:	0c006432 	jal	800190c8 <__do_global_ctors>
80000538:	00000000 	nop
8000053c:	24040000 	li	a0,0
80000540:	0c0003ae 	jal	80000eb8 <main>
80000544:	24050000 	li	a1,0

80000548 <deadloop>:
80000548:	08000152 	j	80000548 <deadloop>
8000054c:	00000000 	nop

80000550 <intvector>:
80000550:	3c1b8000 	lui	k1,0x8000
80000554:	277b25c8 	addiu	k1,k1,9672
80000558:	03600008 	jr	k1
8000055c:	00000000 	nop

80000560 <cmp_idx>:
        Can be used by mergesort.
*/
ee_s32
cmp_idx(list_data *a, list_data *b, core_results *res)
{
    if (res == NULL)
80000560:	50c00005 	beqzl	a2,80000578 <cmp_idx+0x18>
80000564:	84830000 	lh	v1,0(a0)
    {
        a->data16 = (a->data16 & 0xff00) | (0x00ff & (a->data16 >> 8));
        b->data16 = (b->data16 & 0xff00) | (0x00ff & (b->data16 >> 8));
    }
    return a->idx - b->idx;
80000568:	84830002 	lh	v1,2(a0)
8000056c:	84a20002 	lh	v0,2(a1)
}
80000570:	03e00008 	jr	ra
80000574:	00621023 	subu	v0,v1,v0
        a->data16 = (a->data16 & 0xff00) | (0x00ff & (a->data16 >> 8));
80000578:	2402ff00 	li	v0,-256
8000057c:	00623024 	and	a2,v1,v0
80000580:	3063ffff 	andi	v1,v1,0xffff
80000584:	00031a02 	srl	v1,v1,0x8
80000588:	00c31825 	or	v1,a2,v1
8000058c:	a4830000 	sh	v1,0(a0)
        b->data16 = (b->data16 & 0xff00) | (0x00ff & (b->data16 >> 8));
80000590:	84a30000 	lh	v1,0(a1)
80000594:	00621024 	and	v0,v1,v0
80000598:	3063ffff 	andi	v1,v1,0xffff
8000059c:	00031a02 	srl	v1,v1,0x8
800005a0:	00431025 	or	v0,v0,v1
800005a4:	1000fff0 	b	80000568 <cmp_idx+0x8>
800005a8:	a4a20000 	sh	v0,0(a1)

800005ac <calc_func>:
{
800005ac:	27bdffa8 	addiu	sp,sp,-88
800005b0:	ffbf0050 	sd	ra,80(sp)
800005b4:	ffb30048 	sd	s3,72(sp)
800005b8:	ffb20040 	sd	s2,64(sp)
800005bc:	ffb10038 	sd	s1,56(sp)
800005c0:	ffb00030 	sd	s0,48(sp)
    ee_s16 data = *pdata;
800005c4:	84900000 	lh	s0,0(a0)
          & 1;  /* bit 7 indicates if the function result has been cached */
800005c8:	001011c3 	sra	v0,s0,0x7
    if (optype) /* if cached, use cache */
800005cc:	30420001 	andi	v0,v0,0x1
800005d0:	10400003 	beqz	v0,800005e0 <calc_func+0x34>
800005d4:	00809025 	move	s2,a0
        return (data & 0x007f);
800005d8:	10000032 	b	800006a4 <calc_func+0xf8>
800005dc:	3202007f 	andi	v0,s0,0x7f
800005e0:	00a08825 	move	s1,a1
        ee_s16 flag = data & 0x7; /* bits 0-2 is type of function to perform */
800005e4:	32030007 	andi	v1,s0,0x7
               & 0xf);       /* bits 3-6 is specific data for the operation */
800005e8:	001028c3 	sra	a1,s0,0x3
        ee_s16 dtype
800005ec:	30a5000f 	andi	a1,a1,0xf
        dtype |= dtype << 4; /* replicate the lower 4 bits to get an 8b value */
800005f0:	00051100 	sll	v0,a1,0x4
        switch (flag)
800005f4:	1060000d 	beqz	v1,8000062c <calc_func+0x80>
800005f8:	00452825 	or	a1,v0,a1
800005fc:	24020001 	li	v0,1
80000600:	1462001e 	bne	v1,v0,8000067c <calc_func+0xd0>
80000604:	02009825 	move	s3,s0
                retval = core_bench_matrix(&(res->mat), dtype, res->crc);
80000608:	96260038 	lhu	a2,56(s1)
8000060c:	0c000746 	jal	80001d18 <core_bench_matrix>
80000610:	26240028 	addiu	a0,s1,40
80000614:	00029c00 	sll	s3,v0,0x10
                if (res->crcmatrix == 0)
80000618:	9623003c 	lhu	v1,60(s1)
8000061c:	14600017 	bnez	v1,8000067c <calc_func+0xd0>
80000620:	00139c03 	sra	s3,s3,0x10
                    res->crcmatrix = retval;
80000624:	10000015 	b	8000067c <calc_func+0xd0>
80000628:	a622003c 	sh	v0,60(s1)
                retval = core_bench_state(res->size,
8000062c:	86270002 	lh	a3,2(s1)
80000630:	86260000 	lh	a2,0(s1)
80000634:	8e280014 	lw	t0,20(s1)
80000638:	8e240018 	lw	a0,24(s1)
8000063c:	96220038 	lhu	v0,56(s1)
80000640:	afa2002c 	sw	v0,44(sp)
80000644:	28a20022 	slti	v0,a1,34
80000648:	10400002 	beqz	v0,80000654 <calc_func+0xa8>
8000064c:	00a01825 	move	v1,a1
80000650:	24030022 	li	v1,34
80000654:	00031c00 	sll	v1,v1,0x10
80000658:	00031c03 	sra	v1,v1,0x10
8000065c:	afa30024 	sw	v1,36(sp)
80000660:	0c0008b0 	jal	800022c0 <core_bench_state>
80000664:	01002825 	move	a1,t0
80000668:	00029c00 	sll	s3,v0,0x10
                if (res->crcstate == 0)
8000066c:	9623003e 	lhu	v1,62(s1)
80000670:	14600002 	bnez	v1,8000067c <calc_func+0xd0>
80000674:	00139c03 	sra	s3,s3,0x10
                    res->crcstate = retval;
80000678:	a622003e 	sh	v0,62(s1)
        res->crc = crcu16(retval, res->crc);
8000067c:	96250038 	lhu	a1,56(s1)
80000680:	0c00094f 	jal	8000253c <crcu16>
80000684:	3264ffff 	andi	a0,s3,0xffff
80000688:	a6220038 	sh	v0,56(s1)
        retval &= 0x007f;
8000068c:	3262007f 	andi	v0,s3,0x7f
        *pdata = (data & 0xff00) | 0x0080 | retval; /* cache the result */
80000690:	2403ff00 	li	v1,-256
80000694:	02038024 	and	s0,s0,v1
80000698:	00508025 	or	s0,v0,s0
8000069c:	36100080 	ori	s0,s0,0x80
800006a0:	a6500000 	sh	s0,0(s2)
}
800006a4:	dfbf0050 	ld	ra,80(sp)
800006a8:	dfb30048 	ld	s3,72(sp)
800006ac:	dfb20040 	ld	s2,64(sp)
800006b0:	dfb10038 	ld	s1,56(sp)
800006b4:	dfb00030 	ld	s0,48(sp)
800006b8:	03e00008 	jr	ra
800006bc:	27bd0058 	addiu	sp,sp,88

800006c0 <cmp_complex>:
{
800006c0:	27bdffc0 	addiu	sp,sp,-64
800006c4:	ffbf0038 	sd	ra,56(sp)
800006c8:	ffb20030 	sd	s2,48(sp)
800006cc:	ffb10028 	sd	s1,40(sp)
800006d0:	ffb00020 	sd	s0,32(sp)
800006d4:	00a09025 	move	s2,a1
800006d8:	00c08025 	move	s0,a2
    ee_s16 val1 = calc_func(&(a->data16), res);
800006dc:	0c00016b 	jal	800005ac <calc_func>
800006e0:	00c02825 	move	a1,a2
800006e4:	00408825 	move	s1,v0
    ee_s16 val2 = calc_func(&(b->data16), res);
800006e8:	02002825 	move	a1,s0
800006ec:	0c00016b 	jal	800005ac <calc_func>
800006f0:	02402025 	move	a0,s2
}
800006f4:	02221023 	subu	v0,s1,v0
800006f8:	dfbf0038 	ld	ra,56(sp)
800006fc:	dfb20030 	ld	s2,48(sp)
80000700:	dfb10028 	ld	s1,40(sp)
80000704:	dfb00020 	ld	s0,32(sp)
80000708:	03e00008 	jr	ra
8000070c:	27bd0040 	addiu	sp,sp,64

80000710 <copy_info>:

void
copy_info(list_data *to, list_data *from)
{
    to->data16 = from->data16;
80000710:	84a20000 	lh	v0,0(a1)
80000714:	a4820000 	sh	v0,0(a0)
    to->idx    = from->idx;
80000718:	84a20002 	lh	v0,2(a1)
}
8000071c:	03e00008 	jr	ra
80000720:	a4820002 	sh	v0,2(a0)

80000724 <core_list_insert_new>:
                     list_head * memblock_end,
                     list_data * datablock_end)
{
    list_head *newitem;

    if ((*memblock + 1) >= memblock_end)
80000724:	8cc20000 	lw	v0,0(a2)
80000728:	24490008 	addiu	t1,v0,8
8000072c:	8fa30024 	lw	v1,36(sp)
80000730:	0123182b 	sltu	v1,t1,v1
80000734:	10600015 	beqz	v1,8000078c <core_list_insert_new+0x68>
80000738:	00000000 	nop
        return NULL;
    if ((*datablock + 1) >= datablock_end)
8000073c:	8ce30000 	lw	v1,0(a3)
80000740:	24680004 	addiu	t0,v1,4
80000744:	8fa3002c 	lw	v1,44(sp)
80000748:	0103182b 	sltu	v1,t0,v1
8000074c:	10600011 	beqz	v1,80000794 <core_list_insert_new+0x70>
80000750:	00000000 	nop
        return NULL;

    newitem = *memblock;
    (*memblock)++;
80000754:	acc90000 	sw	t1,0(a2)
    newitem->next      = insert_point->next;
80000758:	8c830000 	lw	v1,0(a0)
8000075c:	ac430000 	sw	v1,0(v0)
    insert_point->next = newitem;
80000760:	ac820000 	sw	v0,0(a0)

    newitem->info = *datablock;
80000764:	8ce30000 	lw	v1,0(a3)
80000768:	ac430004 	sw	v1,4(v0)
    (*datablock)++;
8000076c:	24630004 	addiu	v1,v1,4
80000770:	ace30000 	sw	v1,0(a3)
    copy_info(newitem->info, info);
80000774:	8c430004 	lw	v1,4(v0)
    to->data16 = from->data16;
80000778:	84a40000 	lh	a0,0(a1)
8000077c:	a4640000 	sh	a0,0(v1)
    to->idx    = from->idx;
80000780:	84a40002 	lh	a0,2(a1)
}
80000784:	03e00008 	jr	ra
80000788:	a4640002 	sh	a0,2(v1)
        return NULL;
8000078c:	03e00008 	jr	ra
80000790:	00001025 	move	v0,zero

    return newitem;
}
80000794:	03e00008 	jr	ra
80000798:	00001025 	move	v0,zero

8000079c <core_list_remove>:
*/
list_head *
core_list_remove(list_head *item)
{
    list_data *tmp;
    list_head *ret = item->next;
8000079c:	8c820000 	lw	v0,0(a0)
    /* swap data pointers */
    tmp        = item->info;
800007a0:	8c830004 	lw	v1,4(a0)
    item->info = ret->info;
800007a4:	8c450004 	lw	a1,4(v0)
800007a8:	ac850004 	sw	a1,4(a0)
    ret->info  = tmp;
800007ac:	ac430004 	sw	v1,4(v0)
    /* and eliminate item */
    item->next = item->next->next;
800007b0:	8c830000 	lw	v1,0(a0)
800007b4:	8c630000 	lw	v1,0(v1)
800007b8:	ac830000 	sw	v1,0(a0)
    ret->next  = NULL;
    return ret;
}
800007bc:	03e00008 	jr	ra
800007c0:	ac400000 	sw	zero,0(v0)

800007c4 <core_list_undo_remove>:
        The item that was linked back to the list.

*/
list_head *
core_list_undo_remove(list_head *item_removed, list_head *item_modified)
{
800007c4:	00801025 	move	v0,a0
    list_data *tmp;
    /* swap data pointers */
    tmp                 = item_removed->info;
800007c8:	8c830004 	lw	v1,4(a0)
    item_removed->info  = item_modified->info;
800007cc:	8ca40004 	lw	a0,4(a1)
800007d0:	ac440004 	sw	a0,4(v0)
    item_modified->info = tmp;
800007d4:	aca30004 	sw	v1,4(a1)
    /* and insert item */
    item_removed->next  = item_modified->next;
800007d8:	8ca30000 	lw	v1,0(a1)
800007dc:	ac430000 	sw	v1,0(v0)
    item_modified->next = item_removed;
    return item_removed;
}
800007e0:	03e00008 	jr	ra
800007e4:	aca20000 	sw	v0,0(a1)

800007e8 <core_list_find>:
        Found item, or NULL if not found.
*/
list_head *
core_list_find(list_head *list, list_data *info)
{
    if (info->idx >= 0)
800007e8:	84a30002 	lh	v1,2(a1)
800007ec:	0460000c 	bltz	v1,80000820 <core_list_find+0x38>
800007f0:	00000000 	nop
    {
        while (list && (list->info->idx != info->idx))
800007f4:	10800017 	beqz	a0,80000854 <core_list_find+0x6c>
800007f8:	00000000 	nop
800007fc:	8c820004 	lw	v0,4(a0)
80000800:	84420002 	lh	v0,2(v0)
80000804:	10430015 	beq	v0,v1,8000085c <core_list_find+0x74>
80000808:	00000000 	nop
            list = list->next;
8000080c:	8c840000 	lw	a0,0(a0)
        while (list && (list->info->idx != info->idx))
80000810:	5480fffb 	bnezl	a0,80000800 <core_list_find+0x18>
80000814:	8c820004 	lw	v0,4(a0)
    {
        while (list && ((list->info->data16 & 0xff) != info->data16))
            list = list->next;
        return list;
    }
}
80000818:	03e00008 	jr	ra
8000081c:	00801025 	move	v0,a0
        while (list && ((list->info->data16 & 0xff) != info->data16))
80000820:	10800012 	beqz	a0,8000086c <core_list_find+0x84>
80000824:	00000000 	nop
80000828:	84a50000 	lh	a1,0(a1)
8000082c:	00801025 	move	v0,a0
80000830:	8c830004 	lw	v1,4(a0)
80000834:	90630001 	lbu	v1,1(v1)
80000838:	1065000a 	beq	v1,a1,80000864 <core_list_find+0x7c>
8000083c:	00000000 	nop
            list = list->next;
80000840:	8c420000 	lw	v0,0(v0)
        while (list && ((list->info->data16 & 0xff) != info->data16))
80000844:	5440fffb 	bnezl	v0,80000834 <core_list_find+0x4c>
80000848:	8c430004 	lw	v1,4(v0)
8000084c:	10000005 	b	80000864 <core_list_find+0x7c>
80000850:	00000000 	nop
        while (list && (list->info->idx != info->idx))
80000854:	03e00008 	jr	ra
80000858:	00801025 	move	v0,a0
8000085c:	03e00008 	jr	ra
80000860:	00801025 	move	v0,a0
80000864:	03e00008 	jr	ra
80000868:	00000000 	nop
        while (list && ((list->info->data16 & 0xff) != info->data16))
8000086c:	03e00008 	jr	ra
80000870:	00801025 	move	v0,a0

80000874 <core_list_reverse>:

list_head *
core_list_reverse(list_head *list)
{
    list_head *next = NULL, *tmp;
    while (list)
80000874:	10800008 	beqz	a0,80000898 <core_list_reverse+0x24>
80000878:	00801025 	move	v0,a0
    list_head *next = NULL, *tmp;
8000087c:	10000002 	b	80000888 <core_list_reverse+0x14>
80000880:	00002025 	move	a0,zero
    {
        tmp        = list->next;
        list->next = next;
        next       = list;
        list       = tmp;
80000884:	00601025 	move	v0,v1
        tmp        = list->next;
80000888:	8c430000 	lw	v1,0(v0)
        list->next = next;
8000088c:	ac440000 	sw	a0,0(v0)
    while (list)
80000890:	1460fffc 	bnez	v1,80000884 <core_list_reverse+0x10>
80000894:	00402025 	move	a0,v0
    }
    return next;
}
80000898:	03e00008 	jr	ra
8000089c:	00000000 	nop

800008a0 <core_list_mergesort>:
        but the algorithm could theoretically modify where the list starts.

 */
list_head *
core_list_mergesort(list_head *list, list_cmp cmp, core_results *res)
{
800008a0:	27bdff88 	addiu	sp,sp,-120
800008a4:	ffbf0070 	sd	ra,112(sp)
800008a8:	ffbe0068 	sd	s8,104(sp)
800008ac:	ffb70060 	sd	s7,96(sp)
800008b0:	ffb60058 	sd	s6,88(sp)
800008b4:	ffb50050 	sd	s5,80(sp)
800008b8:	ffb40048 	sd	s4,72(sp)
800008bc:	ffb30040 	sd	s3,64(sp)
800008c0:	ffb20038 	sd	s2,56(sp)
800008c4:	ffb10030 	sd	s1,48(sp)
800008c8:	ffb00028 	sd	s0,40(sp)
800008cc:	0080a025 	move	s4,a0
800008d0:	00a0b825 	move	s7,a1
800008d4:	00c0b025 	move	s6,a2
    list_head *p, *q, *e, *tail;
    ee_s32     insize, nmerges, psize, qsize, i;

    insize = 1;
800008d8:	24150001 	li	s5,1
        list = NULL;
        tail = NULL;

        nmerges = 0; /* count number of merges we do in this pass */

        while (p)
800008dc:	12800045 	beqz	s4,800009f4 <core_list_mergesort+0x154>
800008e0:	0000f025 	move	s8,zero
        tail = NULL;
800008e4:	00009025 	move	s2,zero
        list = NULL;
800008e8:	10000025 	b	80000980 <core_list_mergesort+0xe0>
800008ec:	afa00020 	sw	zero,32(sp)
                else if (qsize == 0 || !q)
                {
                    /* q is empty; e must come from p. */
                    e = p;
                    p = p->next;
                    psize--;
800008f0:	02801025 	move	v0,s4
800008f4:	1000000c 	b	80000928 <core_list_mergesort+0x88>
800008f8:	8e940000 	lw	s4,0(s4)
                {
                    /* First element of p is lower (or same); e must come from
                     * p. */
                    e = p;
                    p = p->next;
                    psize--;
800008fc:	02801025 	move	v0,s4
80000900:	10000009 	b	80000928 <core_list_mergesort+0x88>
80000904:	8e940000 	lw	s4,0(s4)
            while (psize > 0 || (qsize > 0 && q))
80000908:	5a60001b 	blezl	s3,80000978 <core_list_mergesort+0xd8>
8000090c:	0200a025 	move	s4,s0
80000910:	52000026 	beqzl	s0,800009ac <core_list_mergesort+0x10c>
80000914:	2bde0002 	slti	s8,s8,2
                if (psize == 0)
80000918:	1620000d 	bnez	s1,80000950 <core_list_mergesort+0xb0>
8000091c:	02001025 	move	v0,s0
                    qsize--;
80000920:	2673ffff 	addiu	s3,s3,-1
                    q = q->next;
80000924:	8e100000 	lw	s0,0(s0)
                    q = q->next;
                    qsize--;
                }

                /* add the next element to the merged list */
                if (tail)
80000928:	52400002 	beqzl	s2,80000934 <core_list_mergesort+0x94>
8000092c:	afa20020 	sw	v0,32(sp)
                {
                    tail->next = e;
80000930:	ae420000 	sw	v0,0(s2)
                }
                else
                {
                    list = e;
80000934:	00409025 	move	s2,v0
            while (psize > 0 || (qsize > 0 && q))
80000938:	1a20fff3 	blez	s1,80000908 <core_list_mergesort+0x68>
8000093c:	00000000 	nop
                else if (qsize == 0 || !q)
80000940:	5260ffeb 	beqzl	s3,800008f0 <core_list_mergesort+0x50>
80000944:	2631ffff 	addiu	s1,s1,-1
80000948:	5200ffe9 	beqzl	s0,800008f0 <core_list_mergesort+0x50>
8000094c:	2631ffff 	addiu	s1,s1,-1
                else if (cmp(p->info, q->info, res) <= 0)
80000950:	02c03025 	move	a2,s6
80000954:	8e050004 	lw	a1,4(s0)
80000958:	02e0f809 	jalr	s7
8000095c:	8e840004 	lw	a0,4(s4)
80000960:	5840ffe6 	blezl	v0,800008fc <core_list_mergesort+0x5c>
80000964:	2631ffff 	addiu	s1,s1,-1
                    qsize--;
80000968:	2673ffff 	addiu	s3,s3,-1
8000096c:	02001025 	move	v0,s0
                    q = q->next;
80000970:	1000ffed 	b	80000928 <core_list_mergesort+0x88>
80000974:	8e100000 	lw	s0,0(s0)
        while (p)
80000978:	5280000c 	beqzl	s4,800009ac <core_list_mergesort+0x10c>
8000097c:	2bde0002 	slti	s8,s8,2
            for (i = 0; i < insize; i++)
80000980:	1aa0fffd 	blez	s5,80000978 <core_list_mergesort+0xd8>
80000984:	27de0001 	addiu	s8,s8,1
80000988:	02808025 	move	s0,s4
            psize = 0;
8000098c:	00008825 	move	s1,zero
                q = q->next;
80000990:	8e100000 	lw	s0,0(s0)
                if (!q)
80000994:	12000003 	beqz	s0,800009a4 <core_list_mergesort+0x104>
80000998:	26310001 	addiu	s1,s1,1
            for (i = 0; i < insize; i++)
8000099c:	56b1fffd 	bnel	s5,s1,80000994 <core_list_mergesort+0xf4>
800009a0:	8e100000 	lw	s0,0(s0)
                    list = e;
800009a4:	1000ffe4 	b	80000938 <core_list_mergesort+0x98>
800009a8:	02a09825 	move	s3,s5
        }

        tail->next = NULL;

        /* If we have done only one merge, we're finished. */
        if (nmerges <= 1) /* allow for nmerges==0, the empty list case */
800009ac:	13c0000e 	beqz	s8,800009e8 <core_list_mergesort+0x148>
800009b0:	ae400000 	sw	zero,0(s2)
        insize *= 2;
    }
#if COMPILER_REQUIRES_SORT_RETURN
    return list;
#endif
}
800009b4:	8fa20020 	lw	v0,32(sp)
800009b8:	dfbf0070 	ld	ra,112(sp)
800009bc:	dfbe0068 	ld	s8,104(sp)
800009c0:	dfb70060 	ld	s7,96(sp)
800009c4:	dfb60058 	ld	s6,88(sp)
800009c8:	dfb50050 	ld	s5,80(sp)
800009cc:	dfb40048 	ld	s4,72(sp)
800009d0:	dfb30040 	ld	s3,64(sp)
800009d4:	dfb20038 	ld	s2,56(sp)
800009d8:	dfb10030 	ld	s1,48(sp)
800009dc:	dfb00028 	ld	s0,40(sp)
800009e0:	03e00008 	jr	ra
800009e4:	27bd0078 	addiu	sp,sp,120
        insize *= 2;
800009e8:	0015a840 	sll	s5,s5,0x1
        p    = list;
800009ec:	1000ffbb 	b	800008dc <core_list_mergesort+0x3c>
800009f0:	8fb40020 	lw	s4,32(sp)
        tail->next = NULL;
800009f4:	ac000000 	sw	zero,0(zero)
        list = NULL;
800009f8:	1000ffee 	b	800009b4 <core_list_mergesort+0x114>
800009fc:	afb40020 	sw	s4,32(sp)

80000a00 <core_bench_list>:
{
80000a00:	27bdff88 	addiu	sp,sp,-120
80000a04:	ffbf0070 	sd	ra,112(sp)
80000a08:	ffbe0068 	sd	s8,104(sp)
80000a0c:	ffb70060 	sd	s7,96(sp)
80000a10:	ffb60058 	sd	s6,88(sp)
80000a14:	ffb50050 	sd	s5,80(sp)
80000a18:	ffb40048 	sd	s4,72(sp)
80000a1c:	ffb30040 	sd	s3,64(sp)
80000a20:	ffb20038 	sd	s2,56(sp)
80000a24:	ffb10030 	sd	s1,48(sp)
80000a28:	ffb00028 	sd	s0,40(sp)
80000a2c:	0080b025 	move	s6,a0
80000a30:	00a0b825 	move	s7,a1
    list_head *list     = res->list;
80000a34:	8c910024 	lw	s1,36(a0)
    ee_s16     find_num = res->seed3;
80000a38:	849e0004 	lh	s8,4(a0)
    for (i = 0; i < find_num; i++)
80000a3c:	1bc00035 	blez	s8,80000b14 <core_bench_list+0x114>
80000a40:	a7a50022 	sh	a1,34(sp)
80000a44:	00009025 	move	s2,zero
    ee_u16     found = 0, missed = 0;
80000a48:	0000a825 	move	s5,zero
80000a4c:	0000a025 	move	s4,zero
    ee_u16     retval = 0;
80000a50:	1000001b 	b	80000ac0 <core_bench_list+0xc0>
80000a54:	00008025 	move	s0,zero
            missed++;
80000a58:	26b50001 	addiu	s5,s5,1
80000a5c:	32b5ffff 	andi	s5,s5,0xffff
            retval += (list->next->info->data16 >> 8) & 1;
80000a60:	8c420000 	lw	v0,0(v0)
80000a64:	8c420004 	lw	v0,4(v0)
80000a68:	80420000 	lb	v0,0(v0)
80000a6c:	30420001 	andi	v0,v0,0x1
80000a70:	02028021 	addu	s0,s0,v0
80000a74:	10000009 	b	80000a9c <core_bench_list+0x9c>
80000a78:	3210ffff 	andi	s0,s0,0xffff
            if (this_find->next != NULL)
80000a7c:	8e620000 	lw	v0,0(s3)
80000a80:	50400007 	beqzl	v0,80000aa0 <core_bench_list+0xa0>
80000a84:	87a20022 	lh	v0,34(sp)
                this_find->next = finder->next;
80000a88:	8c430000 	lw	v1,0(v0)
80000a8c:	ae630000 	sw	v1,0(s3)
                finder->next    = list->next;
80000a90:	8e230000 	lw	v1,0(s1)
80000a94:	ac430000 	sw	v1,0(v0)
                list->next      = finder;
80000a98:	ae220000 	sw	v0,0(s1)
        if (info.idx >= 0)
80000a9c:	87a20022 	lh	v0,34(sp)
80000aa0:	04400002 	bltz	v0,80000aac <core_bench_list+0xac>
80000aa4:	24420001 	addiu	v0,v0,1
            info.idx++;
80000aa8:	a7a20022 	sh	v0,34(sp)
    for (i = 0; i < find_num; i++)
80000aac:	26520001 	addiu	s2,s2,1
80000ab0:	00129400 	sll	s2,s2,0x10
80000ab4:	00129403 	sra	s2,s2,0x10
80000ab8:	53d2001a 	beql	s8,s2,80000b24 <core_bench_list+0x124>
80000abc:	0014a080 	sll	s4,s4,0x2
        info.data16 = (i & 0xff);
80000ac0:	324200ff 	andi	v0,s2,0xff
80000ac4:	a7a20020 	sh	v0,32(sp)
        this_find   = core_list_find(list, &info);
80000ac8:	27a50020 	addiu	a1,sp,32
80000acc:	0c0001fa 	jal	800007e8 <core_list_find>
80000ad0:	02202025 	move	a0,s1
80000ad4:	00409825 	move	s3,v0
        list        = core_list_reverse(list);
80000ad8:	0c00021d 	jal	80000874 <core_list_reverse>
80000adc:	02202025 	move	a0,s1
        if (this_find == NULL)
80000ae0:	1260ffdd 	beqz	s3,80000a58 <core_bench_list+0x58>
80000ae4:	00408825 	move	s1,v0
            found++;
80000ae8:	26940001 	addiu	s4,s4,1
            if (this_find->info->data16 & 0x1) /* use found value */
80000aec:	8e620004 	lw	v0,4(s3)
80000af0:	84420000 	lh	v0,0(v0)
80000af4:	30430001 	andi	v1,v0,0x1
80000af8:	1060ffe0 	beqz	v1,80000a7c <core_bench_list+0x7c>
80000afc:	3294ffff 	andi	s4,s4,0xffff
                retval += (this_find->info->data16 >> 9) & 1;
80000b00:	00021243 	sra	v0,v0,0x9
80000b04:	30420001 	andi	v0,v0,0x1
80000b08:	02028021 	addu	s0,s0,v0
80000b0c:	1000ffdb 	b	80000a7c <core_bench_list+0x7c>
80000b10:	3210ffff 	andi	s0,s0,0xffff
    ee_u16     found = 0, missed = 0;
80000b14:	0000a825 	move	s5,zero
80000b18:	0000a025 	move	s4,zero
    ee_u16     retval = 0;
80000b1c:	00008025 	move	s0,zero
    retval += found * 4 - missed;
80000b20:	0014a080 	sll	s4,s4,0x2
80000b24:	0295a823 	subu	s5,s4,s5
80000b28:	02158021 	addu	s0,s0,s5
    if (finder_idx > 0)
80000b2c:	1ee0002f 	bgtz	s7,80000bec <core_bench_list+0x1ec>
80000b30:	3210ffff 	andi	s0,s0,0xffff
    remover = core_list_remove(list->next);
80000b34:	0c0001e7 	jal	8000079c <core_list_remove>
80000b38:	8e240000 	lw	a0,0(s1)
80000b3c:	00409825 	move	s3,v0
    finder = core_list_find(list, &info);
80000b40:	27a50020 	addiu	a1,sp,32
80000b44:	0c0001fa 	jal	800007e8 <core_list_find>
80000b48:	02202025 	move	a0,s1
    if (!finder)
80000b4c:	1040002e 	beqz	v0,80000c08 <core_bench_list+0x208>
80000b50:	00409025 	move	s2,v0
        retval = crc16(list->info->data16, retval);
80000b54:	8e230004 	lw	v1,4(s1)
80000b58:	02002825 	move	a1,s0
80000b5c:	0c000969 	jal	800025a4 <crc16>
80000b60:	84640000 	lh	a0,0(v1)
        finder = finder->next;
80000b64:	8e520000 	lw	s2,0(s2)
    while (finder)
80000b68:	1640fffa 	bnez	s2,80000b54 <core_bench_list+0x154>
80000b6c:	00408025 	move	s0,v0
    remover = core_list_undo_remove(remover, list->next);
80000b70:	8e250000 	lw	a1,0(s1)
80000b74:	0c0001f1 	jal	800007c4 <core_list_undo_remove>
80000b78:	02602025 	move	a0,s3
    list = core_list_mergesort(list, cmp_idx, NULL);
80000b7c:	00003025 	move	a2,zero
80000b80:	3c058000 	lui	a1,0x8000
80000b84:	24a50560 	addiu	a1,a1,1376
80000b88:	0c000228 	jal	800008a0 <core_list_mergesort>
80000b8c:	02202025 	move	a0,s1
    finder = list->next;
80000b90:	8c510000 	lw	s1,0(v0)
    while (finder)
80000b94:	12200008 	beqz	s1,80000bb8 <core_bench_list+0x1b8>
80000b98:	00409025 	move	s2,v0
        retval = crc16(list->info->data16, retval);
80000b9c:	8e430004 	lw	v1,4(s2)
80000ba0:	02002825 	move	a1,s0
80000ba4:	0c000969 	jal	800025a4 <crc16>
80000ba8:	84640000 	lh	a0,0(v1)
        finder = finder->next;
80000bac:	8e310000 	lw	s1,0(s1)
    while (finder)
80000bb0:	1620fffa 	bnez	s1,80000b9c <core_bench_list+0x19c>
80000bb4:	00408025 	move	s0,v0
}
80000bb8:	02001025 	move	v0,s0
80000bbc:	dfbf0070 	ld	ra,112(sp)
80000bc0:	dfbe0068 	ld	s8,104(sp)
80000bc4:	dfb70060 	ld	s7,96(sp)
80000bc8:	dfb60058 	ld	s6,88(sp)
80000bcc:	dfb50050 	ld	s5,80(sp)
80000bd0:	dfb40048 	ld	s4,72(sp)
80000bd4:	dfb30040 	ld	s3,64(sp)
80000bd8:	dfb20038 	ld	s2,56(sp)
80000bdc:	dfb10030 	ld	s1,48(sp)
80000be0:	dfb00028 	ld	s0,40(sp)
80000be4:	03e00008 	jr	ra
80000be8:	27bd0078 	addiu	sp,sp,120
        list = core_list_mergesort(list, cmp_complex, res);
80000bec:	02c03025 	move	a2,s6
80000bf0:	3c058000 	lui	a1,0x8000
80000bf4:	24a506c0 	addiu	a1,a1,1728
80000bf8:	0c000228 	jal	800008a0 <core_list_mergesort>
80000bfc:	02202025 	move	a0,s1
80000c00:	1000ffcc 	b	80000b34 <core_bench_list+0x134>
80000c04:	00408825 	move	s1,v0
        finder = list->next;
80000c08:	8e320000 	lw	s2,0(s1)
    while (finder)
80000c0c:	5640ffd2 	bnezl	s2,80000b58 <core_bench_list+0x158>
80000c10:	8e230004 	lw	v1,4(s1)
    remover = core_list_undo_remove(remover, list->next);
80000c14:	1000ffd7 	b	80000b74 <core_bench_list+0x174>
80000c18:	8e250000 	lw	a1,0(s1)

80000c1c <core_list_init>:
{
80000c1c:	27bdff88 	addiu	sp,sp,-120
80000c20:	ffbf0070 	sd	ra,112(sp)
80000c24:	ffb50068 	sd	s5,104(sp)
80000c28:	ffb40060 	sd	s4,96(sp)
80000c2c:	ffb30058 	sd	s3,88(sp)
80000c30:	ffb20050 	sd	s2,80(sp)
80000c34:	ffb10048 	sd	s1,72(sp)
80000c38:	ffb00040 	sd	s0,64(sp)
80000c3c:	afa50080 	sw	a1,128(sp)
80000c40:	00c09825 	move	s3,a2
    ee_u32 size     = (blksize / per_item)
80000c44:	0004203c 	dsll32	a0,a0,0x0
80000c48:	0004203e 	dsrl32	a0,a0,0x0
80000c4c:	00041078 	dsll	v0,a0,0x1
80000c50:	0044102d 	daddu	v0,v0,a0
80000c54:	00021938 	dsll	v1,v0,0x4
80000c58:	0043102d 	daddu	v0,v0,v1
80000c5c:	00021a38 	dsll	v1,v0,0x8
80000c60:	0043102d 	daddu	v0,v0,v1
80000c64:	00028438 	dsll	s0,v0,0x10
80000c68:	0050802d 	daddu	s0,v0,s0
80000c6c:	001080b8 	dsll	s0,s0,0x2
80000c70:	0204802d 	daddu	s0,s0,a0
80000c74:	0010813e 	dsrl32	s0,s0,0x4
80000c78:	2610fffe 	addiu	s0,s0,-2
    list_head *memblock_end  = memblock + size;
80000c7c:	00a09025 	move	s2,a1
80000c80:	0010a0c0 	sll	s4,s0,0x3
80000c84:	00b4a021 	addu	s4,a1,s4
    list_data *datablock     = (list_data *)(memblock_end);
80000c88:	afb40030 	sw	s4,48(sp)
    list_data *datablock_end = datablock + size;
80000c8c:	0010a880 	sll	s5,s0,0x2
80000c90:	0295a821 	addu	s5,s4,s5
    list->next         = NULL;
80000c94:	aca00000 	sw	zero,0(a1)
    list->info         = datablock;
80000c98:	8fa20030 	lw	v0,48(sp)
80000c9c:	aca20004 	sw	v0,4(a1)
    list->info->idx    = 0x0000;
80000ca0:	a4400002 	sh	zero,2(v0)
    list->info->data16 = (ee_s16)0x8080;
80000ca4:	8ca20004 	lw	v0,4(a1)
80000ca8:	24038080 	li	v1,-32640
80000cac:	a4430000 	sh	v1,0(v0)
    memblock++;
80000cb0:	8fa20080 	lw	v0,128(sp)
80000cb4:	24420008 	addiu	v0,v0,8
80000cb8:	afa20080 	sw	v0,128(sp)
    datablock++;
80000cbc:	8fa20030 	lw	v0,48(sp)
80000cc0:	24420004 	addiu	v0,v0,4
80000cc4:	afa20030 	sw	v0,48(sp)
    info.idx    = 0x7fff;
80000cc8:	24027fff 	li	v0,32767
80000ccc:	a7a2003a 	sh	v0,58(sp)
    info.data16 = (ee_s16)0xffff;
80000cd0:	2402ffff 	li	v0,-1
80000cd4:	a7a20038 	sh	v0,56(sp)
    core_list_insert_new(
80000cd8:	afb5002c 	sw	s5,44(sp)
80000cdc:	afb40024 	sw	s4,36(sp)
80000ce0:	27a70030 	addiu	a3,sp,48
80000ce4:	27a60080 	addiu	a2,sp,128
80000ce8:	27a50038 	addiu	a1,sp,56
80000cec:	0c0001c9 	jal	80000724 <core_list_insert_new>
80000cf0:	02402025 	move	a0,s2
    for (i = 0; i < size; i++)
80000cf4:	52000016 	beqzl	s0,80000d50 <core_list_init+0x134>
80000cf8:	8e420000 	lw	v0,0(s2)
80000cfc:	00008825 	move	s1,zero
        ee_u16 datpat = ((ee_u16)(seed ^ i) & 0xf);
80000d00:	3223ffff 	andi	v1,s1,0xffff
80000d04:	00731026 	xor	v0,v1,s3
            = (datpat << 3) | (i & 0x7); /* alternate between algorithms */
80000d08:	000210c0 	sll	v0,v0,0x3
80000d0c:	30420078 	andi	v0,v0,0x78
80000d10:	30630007 	andi	v1,v1,0x7
        ee_u16 dat
80000d14:	00431025 	or	v0,v0,v1
        info.data16 = (dat << 8) | dat;  /* fill the data with actual data and
80000d18:	00021a00 	sll	v1,v0,0x8
80000d1c:	00621025 	or	v0,v1,v0
80000d20:	a7a20038 	sh	v0,56(sp)
        core_list_insert_new(
80000d24:	afb5002c 	sw	s5,44(sp)
80000d28:	afb40024 	sw	s4,36(sp)
80000d2c:	27a70030 	addiu	a3,sp,48
80000d30:	27a60080 	addiu	a2,sp,128
80000d34:	27a50038 	addiu	a1,sp,56
80000d38:	0c0001c9 	jal	80000724 <core_list_insert_new>
80000d3c:	02402025 	move	a0,s2
    for (i = 0; i < size; i++)
80000d40:	26310001 	addiu	s1,s1,1
80000d44:	1611ffef 	bne	s0,s1,80000d04 <core_list_init+0xe8>
80000d48:	3223ffff 	andi	v1,s1,0xffff
    finder = list->next;
80000d4c:	8e420000 	lw	v0,0(s2)
    while (finder->next != NULL)
80000d50:	8c430000 	lw	v1,0(v0)
80000d54:	10600021 	beqz	v1,80000ddc <core_list_init+0x1c0>
80000d58:	0010203c 	dsll32	a0,s0,0x0
        if (i < size / 5) /* first 20% of the list in order */
80000d5c:	0004203e 	dsrl32	a0,a0,0x0
80000d60:	00041878 	dsll	v1,a0,0x1
80000d64:	0064182d 	daddu	v1,v1,a0
80000d68:	00032938 	dsll	a1,v1,0x4
80000d6c:	0065182d 	daddu	v1,v1,a1
80000d70:	00032a38 	dsll	a1,v1,0x8
80000d74:	0065182d 	daddu	v1,v1,a1
80000d78:	00038438 	dsll	s0,v1,0x10
80000d7c:	0070182d 	daddu	v1,v1,s0
80000d80:	000380b8 	dsll	s0,v1,0x2
80000d84:	0204802d 	daddu	s0,s0,a0
80000d88:	001080be 	dsrl32	s0,s0,0x2
80000d8c:	24050002 	li	a1,2
80000d90:	1000000c 	b	80000dc4 <core_list_init+0x1a8>
80000d94:	24030001 	li	v1,1
                                & (((i & 0x07) << 8)
80000d98:	00052200 	sll	a0,a1,0x8
80000d9c:	30840700 	andi	a0,a0,0x700
            ee_u16 pat = (ee_u16)(i++ ^ seed); /* get a pseudo random number */
80000da0:	02633826 	xor	a3,s3,v1
                                   | pat); /* make sure the mixed items end up
80000da4:	00872025 	or	a0,a0,a3
                                & (((i & 0x07) << 8)
80000da8:	30843fff 	andi	a0,a0,0x3fff
            finder->info->idx = 0x3fff
80000dac:	a4c40002 	sh	a0,2(a2)
        finder = finder->next;
80000db0:	8c420000 	lw	v0,0(v0)
    while (finder->next != NULL)
80000db4:	24630001 	addiu	v1,v1,1
80000db8:	8c440000 	lw	a0,0(v0)
80000dbc:	10800007 	beqz	a0,80000ddc <core_list_init+0x1c0>
80000dc0:	24a50001 	addiu	a1,a1,1
        if (i < size / 5) /* first 20% of the list in order */
80000dc4:	0070202b 	sltu	a0,v1,s0
80000dc8:	5080fff3 	beqzl	a0,80000d98 <core_list_init+0x17c>
80000dcc:	8c460004 	lw	a2,4(v0)
            finder->info->idx = i++;
80000dd0:	8c440004 	lw	a0,4(v0)
80000dd4:	1000fff6 	b	80000db0 <core_list_init+0x194>
80000dd8:	a4830002 	sh	v1,2(a0)
    list = core_list_mergesort(list, cmp_idx, NULL);
80000ddc:	00003025 	move	a2,zero
80000de0:	3c058000 	lui	a1,0x8000
80000de4:	24a50560 	addiu	a1,a1,1376
80000de8:	0c000228 	jal	800008a0 <core_list_mergesort>
80000dec:	02402025 	move	a0,s2
}
80000df0:	dfbf0070 	ld	ra,112(sp)
80000df4:	dfb50068 	ld	s5,104(sp)
80000df8:	dfb40060 	ld	s4,96(sp)
80000dfc:	dfb30058 	ld	s3,88(sp)
80000e00:	dfb20050 	ld	s2,80(sp)
80000e04:	dfb10048 	ld	s1,72(sp)
80000e08:	dfb00040 	ld	s0,64(sp)
80000e0c:	03e00008 	jr	ra
80000e10:	27bd0078 	addiu	sp,sp,120

80000e14 <iterate>:
                                    (ee_u16)0xe5a4,
                                    (ee_u16)0x8e3a,
                                    (ee_u16)0x8d84 };
void *
iterate(void *pres)
{
80000e14:	27bdffc0 	addiu	sp,sp,-64
80000e18:	ffbf0038 	sd	ra,56(sp)
80000e1c:	ffb20030 	sd	s2,48(sp)
80000e20:	ffb10028 	sd	s1,40(sp)
80000e24:	ffb00020 	sd	s0,32(sp)
    ee_u32        i;
    ee_u16        crc;
    core_results *res        = (core_results *)pres;
    ee_u32        iterations = res->iterations;
80000e28:	8c92001c 	lw	s2,28(a0)
    res->crc                 = 0;
80000e2c:	a4800038 	sh	zero,56(a0)
    res->crclist             = 0;
80000e30:	a480003a 	sh	zero,58(a0)
    res->crcmatrix           = 0;
80000e34:	a480003c 	sh	zero,60(a0)
    res->crcstate            = 0;

    for (i = 0; i < iterations; i++)
80000e38:	12400018 	beqz	s2,80000e9c <iterate+0x88>
80000e3c:	a480003e 	sh	zero,62(a0)
80000e40:	00808025 	move	s0,a0
80000e44:	10000004 	b	80000e58 <iterate+0x44>
80000e48:	00008825 	move	s1,zero
80000e4c:	26310001 	addiu	s1,s1,1
80000e50:	12510013 	beq	s2,s1,80000ea0 <iterate+0x8c>
80000e54:	00001025 	move	v0,zero
    {
        crc      = core_bench_list(res, 1);
80000e58:	24050001 	li	a1,1
80000e5c:	0c000280 	jal	80000a00 <core_bench_list>
80000e60:	02002025 	move	a0,s0
        res->crc = crcu16(crc, res->crc);
80000e64:	96050038 	lhu	a1,56(s0)
80000e68:	0c00094f 	jal	8000253c <crcu16>
80000e6c:	00402025 	move	a0,v0
80000e70:	a6020038 	sh	v0,56(s0)
        crc      = core_bench_list(res, -1);
80000e74:	2405ffff 	li	a1,-1
80000e78:	0c000280 	jal	80000a00 <core_bench_list>
80000e7c:	02002025 	move	a0,s0
        res->crc = crcu16(crc, res->crc);
80000e80:	96050038 	lhu	a1,56(s0)
80000e84:	0c00094f 	jal	8000253c <crcu16>
80000e88:	00402025 	move	a0,v0
        if (i == 0)
80000e8c:	1620ffef 	bnez	s1,80000e4c <iterate+0x38>
80000e90:	a6020038 	sh	v0,56(s0)
            res->crclist = res->crc;
80000e94:	1000ffed 	b	80000e4c <iterate+0x38>
80000e98:	a602003a 	sh	v0,58(s0)
    }
    return NULL;
}
80000e9c:	00001025 	move	v0,zero
80000ea0:	dfbf0038 	ld	ra,56(sp)
80000ea4:	dfb20030 	ld	s2,48(sp)
80000ea8:	dfb10028 	ld	s1,40(sp)
80000eac:	dfb00020 	ld	s0,32(sp)
80000eb0:	03e00008 	jr	ra
80000eb4:	27bd0040 	addiu	sp,sp,64

80000eb8 <main>:
    int   argc = 0;
    char *argv[1];
#else
MAIN_RETURN_TYPE
main(int argc, char *argv[])
{
80000eb8:	27bdff88 	addiu	sp,sp,-120
80000ebc:	ffbf0068 	sd	ra,104(sp)
80000ec0:	ffbe0060 	sd	s8,96(sp)
80000ec4:	ffb70058 	sd	s7,88(sp)
80000ec8:	ffb60050 	sd	s6,80(sp)
80000ecc:	ffb50048 	sd	s5,72(sp)
80000ed0:	ffb40040 	sd	s4,64(sp)
80000ed4:	ffb30038 	sd	s3,56(sp)
80000ed8:	ffb20030 	sd	s2,48(sp)
80000edc:	ffb10028 	sd	s1,40(sp)
80000ee0:	ffb00020 	sd	s0,32(sp)
80000ee4:	f7b40070 	sdc1	$f20,112(sp)
80000ee8:	afa40078 	sw	a0,120(sp)
    core_results results[MULTITHREAD];
#if (MEM_METHOD == MEM_STACK)
    ee_u8 stack_memblock[TOTAL_DATA_SIZE * MULTITHREAD];
#endif
    /* first call any initializations needed */
    portable_init(&(results[0].port), &argc, argv);
80000eec:	00a03025 	move	a2,a1
80000ef0:	27a50078 	addiu	a1,sp,120
80000ef4:	0c00078c 	jal	80001e30 <portable_init>
80000ef8:	27a40062 	addiu	a0,sp,98
    if (sizeof(struct list_head_s) > 128)
    {
        ee_printf("list_head structure too big for comparable data!\n");
        return MAIN_RETURN_VAL;
    }
    results[0].seed1      = get_seed(1);
80000efc:	0c000924 	jal	80002490 <get_seed_32>
80000f00:	24040001 	li	a0,1
80000f04:	a7a20020 	sh	v0,32(sp)
    results[0].seed2      = get_seed(2);
80000f08:	0c000924 	jal	80002490 <get_seed_32>
80000f0c:	24040002 	li	a0,2
80000f10:	a7a20022 	sh	v0,34(sp)
    results[0].seed3      = get_seed(3);
80000f14:	0c000924 	jal	80002490 <get_seed_32>
80000f18:	24040003 	li	a0,3
80000f1c:	a7a20024 	sh	v0,36(sp)
    results[0].iterations = get_seed_32(4);
80000f20:	0c000924 	jal	80002490 <get_seed_32>
80000f24:	24040004 	li	a0,4
80000f28:	afa2003c 	sw	v0,60(sp)
#if CORE_DEBUG
    results[0].iterations = 1;
#endif
    results[0].execs = get_seed_32(5);
80000f2c:	0c000924 	jal	80002490 <get_seed_32>
80000f30:	24040005 	li	a0,5
    if (results[0].execs == 0)
80000f34:	5040000d 	beqzl	v0,80000f6c <main+0xb4>
80000f38:	24020007 	li	v0,7
    results[0].execs = get_seed_32(5);
80000f3c:	afa20040 	sw	v0,64(sp)
    { /* if not supplied, execute all algorithms */
        results[0].execs = ALL_ALGORITHMS_MASK;
    }
    /* put in some default values based on one seed only for easy testing */
    if ((results[0].seed1 == 0) && (results[0].seed2 == 0)
80000f40:	8fa20020 	lw	v0,32(sp)
80000f44:	1440000b 	bnez	v0,80000f74 <main+0xbc>
80000f48:	8fa30020 	lw	v1,32(sp)
        && (results[0].seed3 == 0))
80000f4c:	87a20024 	lh	v0,36(sp)
80000f50:	1440000c 	bnez	v0,80000f84 <main+0xcc>
80000f54:	8fa2003c 	lw	v0,60(sp)
    { /* perfromance run */
        results[0].seed1 = 0;
80000f58:	a7a00020 	sh	zero,32(sp)
        results[0].seed2 = 0;
80000f5c:	a7a00022 	sh	zero,34(sp)
        results[0].seed3 = 0x66;
80000f60:	24020066 	li	v0,102
80000f64:	10000006 	b	80000f80 <main+0xc8>
80000f68:	a7a20024 	sh	v0,36(sp)
        results[0].execs = ALL_ALGORITHMS_MASK;
80000f6c:	1000fff4 	b	80000f40 <main+0x88>
80000f70:	afa20040 	sw	v0,64(sp)
    }
    if ((results[0].seed1 == 1) && (results[0].seed2 == 0)
80000f74:	3c020001 	lui	v0,0x1
80000f78:	10620051 	beq	v1,v0,800010c0 <main+0x208>
80000f7c:	87a20024 	lh	v0,36(sp)
                results[0].size, results[i].seed1, results[i].memblock[3]);
        }
    }

    /* automatically determine number of iterations if not set */
    if (results[0].iterations == 0)
80000f80:	8fa2003c 	lw	v0,60(sp)
80000f84:	14400028 	bnez	v0,80001028 <main+0x170>
80000f88:	24020001 	li	v0,1
    {
        secs_ret secs_passed = 0;
        ee_u32   divisor;
        results[0].iterations = 1;
80000f8c:	afa2003c 	sw	v0,60(sp)
        while (secs_passed < (secs_ret)1)
80000f90:	d7948000 	ldc1	$f20,-32768(gp)
        {
            results[0].iterations *= 10;
80000f94:	8fa3003c 	lw	v1,60(sp)
80000f98:	00031080 	sll	v0,v1,0x2
80000f9c:	00431021 	addu	v0,v0,v1
80000fa0:	00021040 	sll	v0,v0,0x1
            start_time();
80000fa4:	0c000770 	jal	80001dc0 <start_time>
80000fa8:	afa2003c 	sw	v0,60(sp)
            iterate(&results[0]);
80000fac:	0c000385 	jal	80000e14 <iterate>
80000fb0:	27a40020 	addiu	a0,sp,32
            stop_time();
80000fb4:	0c000778 	jal	80001de0 <stop_time>
80000fb8:	00000000 	nop
            secs_passed = time_in_secs(get_time());
80000fbc:	0c000780 	jal	80001e00 <get_time>
80000fc0:	00000000 	nop
80000fc4:	0c000784 	jal	80001e10 <time_in_secs>
80000fc8:	00402025 	move	a0,v0
        while (secs_passed < (secs_ret)1)
80000fcc:	4634003c 	c.lt.d	$f0,$f20
80000fd0:	00000000 	nop
80000fd4:	4501fff0 	bc1t	80000f98 <main+0xe0>
80000fd8:	8fa3003c 	lw	v1,60(sp)
        }
        /* now we know it executes for at least 1 sec, set actual run time at
         * about 10 secs */
        divisor = (ee_u32)secs_passed;
80000fdc:	d7818008 	ldc1	$f1,-32760(gp)
80000fe0:	4620083e 	c.le.d	$f1,$f0
80000fe4:	00000000 	nop
80000fe8:	4503003c 	bc1tl	800010dc <main+0x224>
80000fec:	46210001 	sub.d	$f0,$f0,$f1
80000ff0:	4620000d 	trunc.w.d	$f0,$f0
80000ff4:	44020000 	mfc1	v0,$f0
80000ff8:	00000000 	nop
        if (divisor == 0) /* some machines cast float to int as 0 since this
                             conversion is not defined by ANSI, but we know at
                             least one second passed */
            divisor = 1;
        results[0].iterations *= 1 + 10 / divisor;
80000ffc:	50400001 	beqzl	v0,80001004 <main+0x14c>
80001000:	24020001 	li	v0,1
80001004:	2403000a 	li	v1,10
80001008:	0062001b 	divu	zero,v1,v0
8000100c:	004001f4 	teq	v0,zero,0x7
80001010:	00001012 	mflo	v0
80001014:	24420001 	addiu	v0,v0,1
80001018:	8fa3003c 	lw	v1,60(sp)
8000101c:	00620018 	mult	v1,v0
80001020:	00001012 	mflo	v0
80001024:	afa2003c 	sw	v0,60(sp)
    }
    /* perform actual benchmark */
    start_time();
80001028:	0c000770 	jal	80001dc0 <start_time>
8000102c:	00000000 	nop
    for (i = 0; i < default_num_contexts; i++)
    {
        core_stop_parallel(&results[i]);
    }
#else
    iterate(&results[0]);
80001030:	0c000385 	jal	80000e14 <iterate>
80001034:	27a40020 	addiu	a0,sp,32
#endif
    stop_time();
80001038:	0c000778 	jal	80001de0 <stop_time>
8000103c:	00000000 	nop
    total_time = get_time();
80001040:	0c000780 	jal	80001e00 <get_time>
80001044:	00000000 	nop
80001048:	00409025 	move	s2,v0
    /* get a function of the input to report */
    seedcrc = crc16(results[0].seed1, seedcrc);
8000104c:	00002825 	move	a1,zero
80001050:	0c000969 	jal	800025a4 <crc16>
80001054:	87a40020 	lh	a0,32(sp)
    seedcrc = crc16(results[0].seed2, seedcrc);
80001058:	00402825 	move	a1,v0
8000105c:	0c000969 	jal	800025a4 <crc16>
80001060:	87a40022 	lh	a0,34(sp)
    seedcrc = crc16(results[0].seed3, seedcrc);
80001064:	00402825 	move	a1,v0
80001068:	0c000969 	jal	800025a4 <crc16>
8000106c:	87a40024 	lh	a0,36(sp)
    seedcrc = crc16(results[0].size, seedcrc);
80001070:	00402825 	move	a1,v0
80001074:	0c000969 	jal	800025a4 <crc16>
80001078:	87a4003a 	lh	a0,58(sp)
8000107c:	0040a825 	move	s5,v0

    switch (seedcrc)
80001080:	24027b05 	li	v0,31493
80001084:	12a20034 	beq	s5,v0,80001158 <main+0x2a0>
80001088:	2ea27b06 	sltiu	v0,s5,31494
8000108c:	14400018 	bnez	v0,800010f0 <main+0x238>
80001090:	240218f2 	li	v0,6386
80001094:	34028a02 	li	v0,0x8a02
80001098:	12a2001f 	beq	s5,v0,80001118 <main+0x260>
8000109c:	3402e9f5 	li	v0,0xe9f5
800010a0:	2413ffff 	li	s3,-1
800010a4:	16a2009e 	bne	s5,v0,80001320 <main+0x468>
800010a8:	2414ffff 	li	s4,-1
            known_id = 2;
            ee_printf("Profile generation run parameters for coremark.\n");
            break;
        case 0xe9f5: /* seed1=0, seed2=0, seed3=0x66, size 666 per algorithm */
            known_id = 3;
            ee_printf("2K performance run parameters for coremark.\n");
800010ac:	3c048002 	lui	a0,0x8002
800010b0:	0c001ce2 	jal	80007388 <puts>
800010b4:	2484b100 	addiu	a0,a0,-20224
            known_id = 3;
800010b8:	1000001b 	b	80001128 <main+0x270>
800010bc:	24130003 	li	s3,3
        && (results[0].seed3 == 0))
800010c0:	1440ffaf 	bnez	v0,80000f80 <main+0xc8>
800010c4:	24023415 	li	v0,13333
        results[0].seed1 = 0x3415;
800010c8:	a7a20020 	sh	v0,32(sp)
        results[0].seed2 = 0x3415;
800010cc:	a7a20022 	sh	v0,34(sp)
        results[0].seed3 = 0x66;
800010d0:	24020066 	li	v0,102
800010d4:	1000ffaa 	b	80000f80 <main+0xc8>
800010d8:	a7a20024 	sh	v0,36(sp)
        divisor = (ee_u32)secs_passed;
800010dc:	4620000d 	trunc.w.d	$f0,$f0
800010e0:	44020000 	mfc1	v0,$f0
800010e4:	3c038000 	lui	v1,0x8000
800010e8:	1000ffc4 	b	80000ffc <main+0x144>
800010ec:	00431025 	or	v0,v0,v1
    switch (seedcrc)
800010f0:	12a2001e 	beq	s5,v0,8000116c <main+0x2b4>
800010f4:	24024eaf 	li	v0,20143
800010f8:	2413ffff 	li	s3,-1
800010fc:	16a20088 	bne	s5,v0,80001320 <main+0x468>
80001100:	2414ffff 	li	s4,-1
            ee_printf("Profile generation run parameters for coremark.\n");
80001104:	3c048002 	lui	a0,0x8002
80001108:	0c001ce2 	jal	80007388 <puts>
8000110c:	2484b0d0 	addiu	a0,a0,-20272
            known_id = 2;
80001110:	10000005 	b	80001128 <main+0x270>
80001114:	24130002 	li	s3,2
            ee_printf("6k performance run parameters for coremark.\n");
80001118:	3c048002 	lui	a0,0x8002
8000111c:	0c001ce2 	jal	80007388 <puts>
80001120:	2484b070 	addiu	a0,a0,-20368
            known_id = 0;
80001124:	00009825 	move	s3,zero
            total_errors = -1;
            break;
    }
    if (known_id >= 0)
    {
        for (i = 0; i < default_num_contexts; i++)
80001128:	8f8280e0 	lw	v0,-32544(gp)
8000112c:	00008825 	move	s1,zero
80001130:	1040007b 	beqz	v0,80001320 <main+0x468>
80001134:	0000a025 	move	s4,zero
        {
            results[i].err = 0;
            if ((results[i].execs & ID_LIST)
                && (results[i].crclist != list_known_crc[known_id]))
80001138:	0013f040 	sll	s8,s3,0x1
8000113c:	3c178002 	lui	s7,0x8002
80001140:	26f7a390 	addiu	s7,s7,-23664
80001144:	03d7b821 	addu	s7,s8,s7
                          results[i].crclist,
                          list_known_crc[known_id]);
                results[i].err++;
            }
            if ((results[i].execs & ID_MATRIX)
                && (results[i].crcmatrix != matrix_known_crc[known_id]))
80001148:	3c168002 	lui	s6,0x8002
8000114c:	26d6a380 	addiu	s6,s6,-23680
80001150:	10000031 	b	80001218 <main+0x360>
80001154:	03d6b021 	addu	s6,s8,s6
            ee_printf("6k validation run parameters for coremark.\n");
80001158:	3c048002 	lui	a0,0x8002
8000115c:	0c001ce2 	jal	80007388 <puts>
80001160:	2484b0a0 	addiu	a0,a0,-20320
            known_id = 1;
80001164:	1000fff0 	b	80001128 <main+0x270>
80001168:	24130001 	li	s3,1
            ee_printf("2K validation run parameters for coremark.\n");
8000116c:	3c048002 	lui	a0,0x8002
80001170:	0c001ce2 	jal	80007388 <puts>
80001174:	2484b130 	addiu	a0,a0,-20176
            known_id = 4;
80001178:	1000ffeb 	b	80001128 <main+0x270>
8000117c:	24130004 	li	s3,4
                ee_printf("[%u]ERROR! list crc 0x%04x - should be 0x%04x\n",
80001180:	3c048002 	lui	a0,0x8002
80001184:	0c001bd6 	jal	80006f58 <printf>
80001188:	2484b160 	addiu	a0,a0,-20128
                results[i].err++;
8000118c:	00111100 	sll	v0,s1,0x4
80001190:	00511021 	addu	v0,v0,s1
80001194:	00021080 	sll	v0,v0,0x2
80001198:	27a30020 	addiu	v1,sp,32
8000119c:	00621021 	addu	v0,v1,v0
800011a0:	94430040 	lhu	v1,64(v0)
800011a4:	24630001 	addiu	v1,v1,1
800011a8:	1000002d 	b	80001260 <main+0x3a8>
800011ac:	a4430040 	sh	v1,64(v0)
            {
                ee_printf("[%u]ERROR! matrix crc 0x%04x - should be 0x%04x\n",
800011b0:	3c048002 	lui	a0,0x8002
800011b4:	0c001bd6 	jal	80006f58 <printf>
800011b8:	2484b190 	addiu	a0,a0,-20080
                          i,
                          results[i].crcmatrix,
                          matrix_known_crc[known_id]);
                results[i].err++;
800011bc:	00101100 	sll	v0,s0,0x4
800011c0:	00501021 	addu	v0,v0,s0
800011c4:	00021080 	sll	v0,v0,0x2
800011c8:	27a30020 	addiu	v1,sp,32
800011cc:	00621021 	addu	v0,v1,v0
800011d0:	94430040 	lhu	v1,64(v0)
800011d4:	24630001 	addiu	v1,v1,1
800011d8:	10000031 	b	800012a0 <main+0x3e8>
800011dc:	a4430040 	sh	v1,64(v0)
                          i,
                          results[i].crcstate,
                          state_known_crc[known_id]);
                results[i].err++;
            }
            total_errors += results[i].err;
800011e0:	00101100 	sll	v0,s0,0x4
800011e4:	00508021 	addu	s0,v0,s0
800011e8:	00108080 	sll	s0,s0,0x2
800011ec:	27a20020 	addiu	v0,sp,32
800011f0:	00508021 	addu	s0,v0,s0
800011f4:	96020040 	lhu	v0,64(s0)
800011f8:	0282a021 	addu	s4,s4,v0
800011fc:	0014a400 	sll	s4,s4,0x10
        for (i = 0; i < default_num_contexts; i++)
80001200:	26310001 	addiu	s1,s1,1
80001204:	3231ffff 	andi	s1,s1,0xffff
80001208:	8f8280e0 	lw	v0,-32544(gp)
8000120c:	0222102b 	sltu	v0,s1,v0
80001210:	10400043 	beqz	v0,80001320 <main+0x468>
80001214:	0014a403 	sra	s4,s4,0x10
            results[i].err = 0;
80001218:	00111100 	sll	v0,s1,0x4
8000121c:	00511021 	addu	v0,v0,s1
80001220:	00021080 	sll	v0,v0,0x2
80001224:	27a30020 	addiu	v1,sp,32
80001228:	00621021 	addu	v0,v1,v0
8000122c:	a4400040 	sh	zero,64(v0)
            if ((results[i].execs & ID_LIST)
80001230:	8c420020 	lw	v0,32(v0)
80001234:	30420001 	andi	v0,v0,0x1
80001238:	10400009 	beqz	v0,80001260 <main+0x3a8>
8000123c:	02208025 	move	s0,s1
                && (results[i].crclist != list_known_crc[known_id]))
80001240:	00111100 	sll	v0,s1,0x4
80001244:	00511021 	addu	v0,v0,s1
80001248:	00021080 	sll	v0,v0,0x2
8000124c:	00621021 	addu	v0,v1,v0
80001250:	9446003a 	lhu	a2,58(v0)
80001254:	96e70000 	lhu	a3,0(s7)
80001258:	14c7ffc9 	bne	a2,a3,80001180 <main+0x2c8>
8000125c:	02202825 	move	a1,s1
            if ((results[i].execs & ID_MATRIX)
80001260:	00101100 	sll	v0,s0,0x4
80001264:	00501021 	addu	v0,v0,s0
80001268:	00021080 	sll	v0,v0,0x2
8000126c:	27a30020 	addiu	v1,sp,32
80001270:	00621021 	addu	v0,v1,v0
80001274:	8c420020 	lw	v0,32(v0)
80001278:	30420002 	andi	v0,v0,0x2
8000127c:	10400009 	beqz	v0,800012a4 <main+0x3ec>
80001280:	00101100 	sll	v0,s0,0x4
                && (results[i].crcmatrix != matrix_known_crc[known_id]))
80001284:	00501021 	addu	v0,v0,s0
80001288:	00021080 	sll	v0,v0,0x2
8000128c:	00621021 	addu	v0,v1,v0
80001290:	9446003c 	lhu	a2,60(v0)
80001294:	96c70000 	lhu	a3,0(s6)
80001298:	14c7ffc5 	bne	a2,a3,800011b0 <main+0x2f8>
8000129c:	02002825 	move	a1,s0
            if ((results[i].execs & ID_STATE)
800012a0:	00101100 	sll	v0,s0,0x4
800012a4:	00501021 	addu	v0,v0,s0
800012a8:	00021080 	sll	v0,v0,0x2
800012ac:	27a30020 	addiu	v1,sp,32
800012b0:	00621021 	addu	v0,v1,v0
800012b4:	8c420020 	lw	v0,32(v0)
800012b8:	30420004 	andi	v0,v0,0x4
800012bc:	1040ffc9 	beqz	v0,800011e4 <main+0x32c>
800012c0:	00101100 	sll	v0,s0,0x4
                && (results[i].crcstate != state_known_crc[known_id]))
800012c4:	00501021 	addu	v0,v0,s0
800012c8:	00021080 	sll	v0,v0,0x2
800012cc:	00621021 	addu	v0,v1,v0
800012d0:	9446003e 	lhu	a2,62(v0)
800012d4:	3c028002 	lui	v0,0x8002
800012d8:	2442a370 	addiu	v0,v0,-23696
800012dc:	03c21021 	addu	v0,s8,v0
800012e0:	94470000 	lhu	a3,0(v0)
800012e4:	10c7ffbf 	beq	a2,a3,800011e4 <main+0x32c>
800012e8:	00101100 	sll	v0,s0,0x4
                ee_printf("[%u]ERROR! state crc 0x%04x - should be 0x%04x\n",
800012ec:	02002825 	move	a1,s0
800012f0:	3c048002 	lui	a0,0x8002
800012f4:	0c001bd6 	jal	80006f58 <printf>
800012f8:	2484b1c8 	addiu	a0,a0,-20024
                results[i].err++;
800012fc:	00101100 	sll	v0,s0,0x4
80001300:	00501021 	addu	v0,v0,s0
80001304:	00021080 	sll	v0,v0,0x2
80001308:	27a30020 	addiu	v1,sp,32
8000130c:	00621021 	addu	v0,v1,v0
80001310:	94430040 	lhu	v1,64(v0)
80001314:	24630001 	addiu	v1,v1,1
80001318:	1000ffb1 	b	800011e0 <main+0x328>
8000131c:	a4430040 	sh	v1,64(v0)
        }
    }
    total_errors += check_data_types();
80001320:	0c000970 	jal	800025c0 <check_data_types>
80001324:	00000000 	nop
80001328:	0054a021 	addu	s4,v0,s4
8000132c:	3294ffff 	andi	s4,s4,0xffff
    /* and report results */
    ee_printf("CoreMark Size    : %lu\n", (long unsigned)results[0].size);
80001330:	8fa50038 	lw	a1,56(sp)
80001334:	3c048002 	lui	a0,0x8002
80001338:	0c001bd6 	jal	80006f58 <printf>
8000133c:	2484b1f8 	addiu	a0,a0,-19976
    ee_printf("Total ticks      : %lu\n", (long unsigned)total_time);
80001340:	02402825 	move	a1,s2
80001344:	3c048002 	lui	a0,0x8002
80001348:	0c001bd6 	jal	80006f58 <printf>
8000134c:	2484b210 	addiu	a0,a0,-19952
#if HAS_FLOAT
    ee_printf("Total time (secs): %f\n", time_in_secs(total_time));
80001350:	0c000784 	jal	80001e10 <time_in_secs>
80001354:	02402025 	move	a0,s2
80001358:	44250000 	dmfc1	a1,$f0
8000135c:	3c048002 	lui	a0,0x8002
80001360:	0c001bd6 	jal	80006f58 <printf>
80001364:	2484b228 	addiu	a0,a0,-19928
    if (time_in_secs(total_time) > 0)
80001368:	0c000784 	jal	80001e10 <time_in_secs>
8000136c:	02402025 	move	a0,s2
80001370:	44a00800 	dmtc1	zero,$f1
80001374:	00000000 	nop
80001378:	4620083c 	c.lt.d	$f1,$f0
8000137c:	00000000 	nop
80001380:	45010098 	bc1t	800015e4 <main+0x72c>
80001384:	8fa3003c 	lw	v1,60(sp)
    if (time_in_secs(total_time) > 0)
        ee_printf("Iterations/Sec   : %d\n",
                  default_num_contexts * results[0].iterations
                      / time_in_secs(total_time));
#endif
    if (time_in_secs(total_time) < 10)
80001388:	0c000784 	jal	80001e10 <time_in_secs>
8000138c:	02402025 	move	a0,s2
80001390:	d7818010 	ldc1	$f1,-32752(gp)
80001394:	4621003c 	c.lt.d	$f0,$f1
80001398:	00000000 	nop
8000139c:	450300a2 	bc1tl	80001628 <main+0x770>
800013a0:	3c048002 	lui	a0,0x8002
    total_errors += check_data_types();
800013a4:	0014a400 	sll	s4,s4,0x10
800013a8:	0014a403 	sra	s4,s4,0x10
        ee_printf(
            "ERROR! Must execute for at least 10 secs for a valid result!\n");
        total_errors++;
    }

    ee_printf("Iterations       : %lu\n",
800013ac:	8fa3003c 	lw	v1,60(sp)
800013b0:	8f8280e0 	lw	v0,-32544(gp)
800013b4:	00620018 	mult	v1,v0
800013b8:	00002812 	mflo	a1
800013bc:	3c048002 	lui	a0,0x8002
800013c0:	0c001bd6 	jal	80006f58 <printf>
800013c4:	2484b298 	addiu	a0,a0,-19816
              (long unsigned)default_num_contexts * results[0].iterations);
    ee_printf("Compiler version : %s\n", COMPILER_VERSION);
800013c8:	3c058002 	lui	a1,0x8002
800013cc:	24a5b2b0 	addiu	a1,a1,-19792
800013d0:	3c048002 	lui	a0,0x8002
800013d4:	0c001bd6 	jal	80006f58 <printf>
800013d8:	2484b2c0 	addiu	a0,a0,-19776
    ee_printf("Compiler flags   : %s\n", COMPILER_FLAGS);
800013dc:	3c058002 	lui	a1,0x8002
800013e0:	24a5b2d8 	addiu	a1,a1,-19752
800013e4:	3c048002 	lui	a0,0x8002
800013e8:	0c001bd6 	jal	80006f58 <printf>
800013ec:	2484b2e0 	addiu	a0,a0,-19744
#if (MULTITHREAD > 1)
    ee_printf("Parallel %s : %d\n", PARALLEL_METHOD, default_num_contexts);
#endif
    ee_printf("Memory location  : %s\n", MEM_LOCATION);
800013f0:	3c058002 	lui	a1,0x8002
800013f4:	24a5b2f8 	addiu	a1,a1,-19720
800013f8:	3c048002 	lui	a0,0x8002
800013fc:	0c001bd6 	jal	80006f58 <printf>
80001400:	2484b348 	addiu	a0,a0,-19640
    /* output for verification */
    ee_printf("seedcrc          : 0x%04x\n", seedcrc);
80001404:	02a02825 	move	a1,s5
80001408:	3c048002 	lui	a0,0x8002
8000140c:	0c001bd6 	jal	80006f58 <printf>
80001410:	2484b360 	addiu	a0,a0,-19616
    if (results[0].execs & ID_LIST)
80001414:	8fa20040 	lw	v0,64(sp)
80001418:	30420001 	andi	v0,v0,0x1
8000141c:	10400017 	beqz	v0,8000147c <main+0x5c4>
80001420:	8fa20040 	lw	v0,64(sp)
        for (i = 0; i < default_num_contexts; i++)
80001424:	8f8280e0 	lw	v0,-32544(gp)
80001428:	10400014 	beqz	v0,8000147c <main+0x5c4>
8000142c:	8fa20040 	lw	v0,64(sp)
80001430:	00008025 	move	s0,zero
            ee_printf("[%d]crclist       : 0x%04x\n", i, results[i].crclist);
80001434:	3c118002 	lui	s1,0x8002
80001438:	2631b380 	addiu	s1,s1,-19584
8000143c:	00101100 	sll	v0,s0,0x4
80001440:	00501021 	addu	v0,v0,s0
80001444:	00021080 	sll	v0,v0,0x2
80001448:	27a30020 	addiu	v1,sp,32
8000144c:	00621021 	addu	v0,v1,v0
80001450:	9446003a 	lhu	a2,58(v0)
80001454:	02002825 	move	a1,s0
80001458:	0c001bd6 	jal	80006f58 <printf>
8000145c:	02202025 	move	a0,s1
        for (i = 0; i < default_num_contexts; i++)
80001460:	26100001 	addiu	s0,s0,1
80001464:	3210ffff 	andi	s0,s0,0xffff
80001468:	8f8280e0 	lw	v0,-32544(gp)
8000146c:	0202102b 	sltu	v0,s0,v0
80001470:	1440fff3 	bnez	v0,80001440 <main+0x588>
80001474:	00101100 	sll	v0,s0,0x4
    if (results[0].execs & ID_MATRIX)
80001478:	8fa20040 	lw	v0,64(sp)
8000147c:	30420002 	andi	v0,v0,0x2
80001480:	10400016 	beqz	v0,800014dc <main+0x624>
80001484:	8fa20040 	lw	v0,64(sp)
        for (i = 0; i < default_num_contexts; i++)
80001488:	8f8280e0 	lw	v0,-32544(gp)
8000148c:	10400093 	beqz	v0,800016dc <main+0x824>
80001490:	00008025 	move	s0,zero
            ee_printf("[%d]crcmatrix     : 0x%04x\n", i, results[i].crcmatrix);
80001494:	3c118002 	lui	s1,0x8002
80001498:	2631b3a0 	addiu	s1,s1,-19552
8000149c:	00101100 	sll	v0,s0,0x4
800014a0:	00501021 	addu	v0,v0,s0
800014a4:	00021080 	sll	v0,v0,0x2
800014a8:	27a30020 	addiu	v1,sp,32
800014ac:	00621021 	addu	v0,v1,v0
800014b0:	9446003c 	lhu	a2,60(v0)
800014b4:	02002825 	move	a1,s0
800014b8:	0c001bd6 	jal	80006f58 <printf>
800014bc:	02202025 	move	a0,s1
        for (i = 0; i < default_num_contexts; i++)
800014c0:	26100001 	addiu	s0,s0,1
800014c4:	3210ffff 	andi	s0,s0,0xffff
800014c8:	8f8280e0 	lw	v0,-32544(gp)
800014cc:	0202102b 	sltu	v0,s0,v0
800014d0:	1440fff3 	bnez	v0,800014a0 <main+0x5e8>
800014d4:	00101100 	sll	v0,s0,0x4
    if (results[0].execs & ID_STATE)
800014d8:	8fa20040 	lw	v0,64(sp)
800014dc:	30420004 	andi	v0,v0,0x4
800014e0:	10400016 	beqz	v0,8000153c <main+0x684>
800014e4:	8f8280e0 	lw	v0,-32544(gp)
        for (i = 0; i < default_num_contexts; i++)
800014e8:	10400028 	beqz	v0,8000158c <main+0x6d4>
800014ec:	00000000 	nop
800014f0:	00008025 	move	s0,zero
            ee_printf("[%d]crcstate      : 0x%04x\n", i, results[i].crcstate);
800014f4:	3c118002 	lui	s1,0x8002
800014f8:	2631b3c0 	addiu	s1,s1,-19520
800014fc:	00101100 	sll	v0,s0,0x4
80001500:	00501021 	addu	v0,v0,s0
80001504:	00021080 	sll	v0,v0,0x2
80001508:	27a30020 	addiu	v1,sp,32
8000150c:	00621021 	addu	v0,v1,v0
80001510:	9446003e 	lhu	a2,62(v0)
80001514:	02002825 	move	a1,s0
80001518:	0c001bd6 	jal	80006f58 <printf>
8000151c:	02202025 	move	a0,s1
        for (i = 0; i < default_num_contexts; i++)
80001520:	26100001 	addiu	s0,s0,1
80001524:	3210ffff 	andi	s0,s0,0xffff
80001528:	8f8280e0 	lw	v0,-32544(gp)
8000152c:	0202102b 	sltu	v0,s0,v0
80001530:	1440fff3 	bnez	v0,80001500 <main+0x648>
80001534:	00101100 	sll	v0,s0,0x4
    for (i = 0; i < default_num_contexts; i++)
80001538:	8f8280e0 	lw	v0,-32544(gp)
8000153c:	10400013 	beqz	v0,8000158c <main+0x6d4>
80001540:	00000000 	nop
80001544:	00008025 	move	s0,zero
        ee_printf("[%d]crcfinal      : 0x%04x\n", i, results[i].crc);
80001548:	3c118002 	lui	s1,0x8002
8000154c:	2631b3e0 	addiu	s1,s1,-19488
80001550:	00101900 	sll	v1,s0,0x4
80001554:	00701821 	addu	v1,v1,s0
80001558:	00031880 	sll	v1,v1,0x2
8000155c:	27a20020 	addiu	v0,sp,32
80001560:	00431821 	addu	v1,v0,v1
80001564:	94660038 	lhu	a2,56(v1)
80001568:	02002825 	move	a1,s0
8000156c:	0c001bd6 	jal	80006f58 <printf>
80001570:	02202025 	move	a0,s1
    for (i = 0; i < default_num_contexts; i++)
80001574:	26100001 	addiu	s0,s0,1
80001578:	3210ffff 	andi	s0,s0,0xffff
8000157c:	8f8280e0 	lw	v0,-32544(gp)
80001580:	0202102b 	sltu	v0,s0,v0
80001584:	1440fff3 	bnez	v0,80001554 <main+0x69c>
80001588:	00101900 	sll	v1,s0,0x4
    if (total_errors == 0)
8000158c:	5280002c 	beqzl	s4,80001640 <main+0x788>
80001590:	3c048002 	lui	a0,0x8002
#endif
            ee_printf("\n");
        }
#endif
    }
    if (total_errors > 0)
80001594:	1a80004b 	blez	s4,800016c4 <main+0x80c>
80001598:	3c048002 	lui	a0,0x8002
        ee_printf("Errors detected\n");
8000159c:	0c001ce2 	jal	80007388 <puts>
800015a0:	2484b470 	addiu	a0,a0,-19344
#if (MEM_METHOD == MEM_MALLOC)
    for (i = 0; i < MULTITHREAD; i++)
        portable_free(results[i].memblock[0]);
#endif
    /* And last call any target specific code for finalizing */
    portable_fini(&(results[0].port));
800015a4:	0c0007b0 	jal	80001ec0 <portable_fini>
800015a8:	27a40062 	addiu	a0,sp,98

    return MAIN_RETURN_VAL;
}
800015ac:	00001025 	move	v0,zero
800015b0:	dfbf0068 	ld	ra,104(sp)
800015b4:	dfbe0060 	ld	s8,96(sp)
800015b8:	dfb70058 	ld	s7,88(sp)
800015bc:	dfb60050 	ld	s6,80(sp)
800015c0:	dfb50048 	ld	s5,72(sp)
800015c4:	dfb40040 	ld	s4,64(sp)
800015c8:	dfb30038 	ld	s3,56(sp)
800015cc:	dfb20030 	ld	s2,48(sp)
800015d0:	dfb10028 	ld	s1,40(sp)
800015d4:	dfb00020 	ld	s0,32(sp)
800015d8:	d7b40070 	ldc1	$f20,112(sp)
800015dc:	03e00008 	jr	ra
800015e0:	27bd0078 	addiu	sp,sp,120
                  default_num_contexts * results[0].iterations
800015e4:	8f8280e0 	lw	v0,-32544(gp)
800015e8:	00620018 	mult	v1,v0
800015ec:	00001012 	mflo	v0
        ee_printf("Iterations/Sec   : %f\n",
800015f0:	0002103c 	dsll32	v0,v0,0x0
800015f4:	0002103e 	dsrl32	v0,v0,0x0
800015f8:	44a20000 	dmtc1	v0,$f0
800015fc:	00000000 	nop
80001600:	46a00521 	cvt.d.l	$f20,$f0
                      / time_in_secs(total_time));
80001604:	0c000784 	jal	80001e10 <time_in_secs>
80001608:	02402025 	move	a0,s2
        ee_printf("Iterations/Sec   : %f\n",
8000160c:	4620a003 	div.d	$f0,$f20,$f0
80001610:	44250000 	dmfc1	a1,$f0
80001614:	3c048002 	lui	a0,0x8002
80001618:	0c001bd6 	jal	80006f58 <printf>
8000161c:	2484b240 	addiu	a0,a0,-19904
80001620:	1000ff59 	b	80001388 <main+0x4d0>
80001624:	00000000 	nop
        ee_printf(
80001628:	0c001ce2 	jal	80007388 <puts>
8000162c:	2484b258 	addiu	a0,a0,-19880
        total_errors++;
80001630:	26940001 	addiu	s4,s4,1
80001634:	0014a400 	sll	s4,s4,0x10
80001638:	1000ff5c 	b	800013ac <main+0x4f4>
8000163c:	0014a403 	sra	s4,s4,0x10
        ee_printf(
80001640:	0c001ce2 	jal	80007388 <puts>
80001644:	2484b400 	addiu	a0,a0,-19456
        if (known_id == 3)
80001648:	24020003 	li	v0,3
8000164c:	1662ffd5 	bne	s3,v0,800015a4 <main+0x6ec>
80001650:	8fa3003c 	lw	v1,60(sp)
                      default_num_contexts * results[0].iterations
80001654:	8f8280e0 	lw	v0,-32544(gp)
80001658:	00620018 	mult	v1,v0
8000165c:	00001012 	mflo	v0
            ee_printf("CoreMark 1.0 : %f / %s %s",
80001660:	0002103c 	dsll32	v0,v0,0x0
80001664:	0002103e 	dsrl32	v0,v0,0x0
80001668:	44a20000 	dmtc1	v0,$f0
8000166c:	00000000 	nop
80001670:	46a00521 	cvt.d.l	$f20,$f0
                          / time_in_secs(total_time),
80001674:	0c000784 	jal	80001e10 <time_in_secs>
80001678:	02402025 	move	a0,s2
            ee_printf("CoreMark 1.0 : %f / %s %s",
8000167c:	3c078002 	lui	a3,0x8002
80001680:	24e7b2d8 	addiu	a3,a3,-19752
80001684:	3c068002 	lui	a2,0x8002
80001688:	24c6b2b0 	addiu	a2,a2,-19792
8000168c:	4620a003 	div.d	$f0,$f20,$f0
80001690:	44250000 	dmfc1	a1,$f0
80001694:	3c048002 	lui	a0,0x8002
80001698:	0c001bd6 	jal	80006f58 <printf>
8000169c:	2484b448 	addiu	a0,a0,-19384
            ee_printf(" / %s", mem_name[MEM_METHOD]);
800016a0:	3c028002 	lui	v0,0x8002
800016a4:	8c45ca64 	lw	a1,-13724(v0)
800016a8:	3c048002 	lui	a0,0x8002
800016ac:	0c001bd6 	jal	80006f58 <printf>
800016b0:	2484b468 	addiu	a0,a0,-19352
            ee_printf("\n");
800016b4:	0c001c06 	jal	80007018 <putchar>
800016b8:	2404000a 	li	a0,10
800016bc:	1000ffb9 	b	800015a4 <main+0x6ec>
800016c0:	00000000 	nop
    if (total_errors < 0)
800016c4:	0681ffb7 	bgez	s4,800015a4 <main+0x6ec>
800016c8:	3c048002 	lui	a0,0x8002
        ee_printf(
800016cc:	0c001ce2 	jal	80007388 <puts>
800016d0:	2484b480 	addiu	a0,a0,-19328
800016d4:	1000ffb3 	b	800015a4 <main+0x6ec>
800016d8:	00000000 	nop
    if (results[0].execs & ID_STATE)
800016dc:	8fa20040 	lw	v0,64(sp)
800016e0:	30420004 	andi	v0,v0,0x4
800016e4:	1440ffa9 	bnez	v0,8000158c <main+0x6d4>
800016e8:	8f8280e0 	lw	v0,-32544(gp)
800016ec:	1000ff93 	b	8000153c <main+0x684>
800016f0:	00000000 	nop

800016f4 <core_init_matrix>:
        The seed parameter MUST be supplied from a source that cannot be
   determined at compile time
*/
ee_u32
core_init_matrix(ee_u32 blksize, void *memblk, ee_s32 seed, mat_params *p)
{
800016f4:	27bdffe8 	addiu	sp,sp,-24
800016f8:	ffb20010 	sd	s2,16(sp)
800016fc:	ffb10008 	sd	s1,8(sp)
80001700:	ffb00000 	sd	s0,0(sp)
    MATDAT *A;
    MATDAT *B;
    ee_s32  order = 1;
    MATDAT  val;
    ee_u32  i = 0, j = 0;
    if (seed == 0)
80001704:	14c00002 	bnez	a2,80001710 <core_init_matrix+0x1c>
80001708:	00e08025 	move	s0,a3
        seed = 1;
8000170c:	24060001 	li	a2,1
    while (j < blksize)
80001710:	1080003e 	beqz	a0,8000180c <core_init_matrix+0x118>
80001714:	24b9ffff 	addiu	t9,a1,-1
80001718:	00003825 	move	a3,zero
    {
        i++;
8000171c:	00e01025 	move	v0,a3
80001720:	24e70001 	addiu	a3,a3,1
        j = i * i * 2 * 4;
80001724:	00e70018 	mult	a3,a3
80001728:	00001812 	mflo	v1
8000172c:	000318c0 	sll	v1,v1,0x3
    while (j < blksize)
80001730:	0064182b 	sltu	v1,v1,a0
80001734:	5460fffa 	bnezl	v1,80001720 <core_init_matrix+0x2c>
80001738:	00e01025 	move	v0,a3
    }
    N = i - 1;
    A = (MATDAT *)align_mem(memblk);
8000173c:	24b2ffff 	addiu	s2,a1,-1
80001740:	2403fffc 	li	v1,-4
80001744:	02439024 	and	s2,s2,v1
80001748:	26520004 	addiu	s2,s2,4
    B = A + N * N;
8000174c:	00420018 	mult	v0,v0
80001750:	0000c012 	mflo	t8
80001754:	0018c040 	sll	t8,t8,0x1

    for (i = 0; i < N; i++)
80001758:	10400033 	beqz	v0,80001828 <core_init_matrix+0x134>
8000175c:	0258c821 	addu	t9,s2,t8
80001760:	00027840 	sll	t7,v0,0x1
80001764:	03207025 	move	t6,t9
80001768:	02406825 	move	t5,s2
    B = A + N * N;
8000176c:	00006025 	move	t4,zero
80001770:	240b0001 	li	t3,1
            val          = matrix_clip(val, 0);
            B[i * N + j] = val;
            val          = (val + order);
            val          = matrix_clip(val, 1);
            A[i * N + j] = val;
            order++;
80001774:	00003825 	move	a3,zero
80001778:	10000007 	b	80001798 <core_init_matrix+0xa4>
8000177c:	2451ffff 	addiu	s1,v0,-1
80001780:	006b5821 	addu	t3,v1,t3
    for (i = 0; i < N; i++)
80001784:	258c0001 	addiu	t4,t4,1
80001788:	01cf7021 	addu	t6,t6,t7
8000178c:	0182182b 	sltu	v1,t4,v0
80001790:	10600025 	beqz	v1,80001828 <core_init_matrix+0x134>
80001794:	01af6821 	addu	t5,t5,t7
    while (j < blksize)
80001798:	01a05025 	move	t2,t5
8000179c:	01c04825 	move	t1,t6
800017a0:	01602825 	move	a1,t3
        for (j = 0; j < N; j++)
800017a4:	00004025 	move	t0,zero
            seed         = ((order * seed) % 65536);
800017a8:	00c50018 	mult	a2,a1
800017ac:	00001812 	mflo	v1
800017b0:	000337c3 	sra	a2,v1,0x1f
800017b4:	00062402 	srl	a0,a2,0x10
800017b8:	00643021 	addu	a2,v1,a0
800017bc:	30c6ffff 	andi	a2,a2,0xffff
800017c0:	00c43023 	subu	a2,a2,a0
            val          = (seed + order);
800017c4:	30a4ffff 	andi	a0,a1,0xffff
800017c8:	00861821 	addu	v1,a0,a2
800017cc:	3063ffff 	andi	v1,v1,0xffff
            B[i * N + j] = val;
800017d0:	a5230000 	sh	v1,0(t1)
            val          = (val + order);
800017d4:	00641821 	addu	v1,v1,a0
            val          = matrix_clip(val, 1);
800017d8:	306300ff 	andi	v1,v1,0xff
            A[i * N + j] = val;
800017dc:	a5430000 	sh	v1,0(t2)
            order++;
800017e0:	24a50001 	addiu	a1,a1,1
        for (j = 0; j < N; j++)
800017e4:	25080001 	addiu	t0,t0,1
800017e8:	25290002 	addiu	t1,t1,2
800017ec:	0102182b 	sltu	v1,t0,v0
800017f0:	1460ffed 	bnez	v1,800017a8 <core_init_matrix+0xb4>
800017f4:	254a0002 	addiu	t2,t2,2
800017f8:	256b0001 	addiu	t3,t3,1
            order++;
800017fc:	1040ffe0 	beqz	v0,80001780 <core_init_matrix+0x8c>
80001800:	00e01825 	move	v1,a3
80001804:	1000ffde 	b	80001780 <core_init_matrix+0x8c>
80001808:	02201825 	move	v1,s1
    A = (MATDAT *)align_mem(memblk);
8000180c:	2402fffc 	li	v0,-4
80001810:	0322c824 	and	t9,t9,v0
80001814:	27320004 	addiu	s2,t9,4
    B = A + N * N;
80001818:	27390006 	addiu	t9,t9,6
    N = i - 1;
8000181c:	2402ffff 	li	v0,-1
    B = A + N * N;
80001820:	1000ffcf 	b	80001760 <core_init_matrix+0x6c>
80001824:	24180002 	li	t8,2
        }
    }

    p->A = A;
80001828:	ae120004 	sw	s2,4(s0)
    p->B = B;
8000182c:	ae190008 	sw	t9,8(s0)
    p->C = (MATRES *)align_mem(B + N * N);
80001830:	03381821 	addu	v1,t9,t8
80001834:	2463ffff 	addiu	v1,v1,-1
80001838:	2404fffc 	li	a0,-4
8000183c:	00641824 	and	v1,v1,a0
80001840:	24630004 	addiu	v1,v1,4
80001844:	ae03000c 	sw	v1,12(s0)
    p->N = N;
80001848:	ae020000 	sw	v0,0(s0)
#if CORE_DEBUG
    printmat(A, N, "A");
    printmat(B, N, "B");
#endif
    return N;
}
8000184c:	dfb20010 	ld	s2,16(sp)
80001850:	dfb10008 	ld	s1,8(sp)
80001854:	dfb00000 	ld	s0,0(sp)
80001858:	03e00008 	jr	ra
8000185c:	27bd0018 	addiu	sp,sp,24

80001860 <matrix_sum>:
matrix_sum(ee_u32 N, MATRES *C, MATDAT clipval)
{
    MATRES tmp = 0, prev = 0, cur = 0;
    ee_s16 ret = 0;
    ee_u32 i, j;
    for (i = 0; i < N; i++)
80001860:	10800021 	beqz	a0,800018e8 <matrix_sum+0x88>
80001864:	00045080 	sll	t2,a0,0x2
80001868:	00aa5021 	addu	t2,a1,t2
8000186c:	00047023 	negu	t6,a0
80001870:	000e7080 	sll	t6,t6,0x2
80001874:	00005825 	move	t3,zero
    ee_s16 ret = 0;
80001878:	00001025 	move	v0,zero
    MATRES tmp = 0, prev = 0, cur = 0;
8000187c:	00003825 	move	a3,zero
80001880:	00002825 	move	a1,zero
    for (i = 0; i < N; i++)
80001884:	00046823 	negu	t5,a0
80001888:	10000014 	b	800018dc <matrix_sum+0x7c>
8000188c:	000d68c0 	sll	t5,t5,0x3
                ret += 10;
                tmp = 0;
            }
            else
            {
                ret += (cur > prev) ? 1 : 0;
80001890:	01021021 	addu	v0,t0,v0
80001894:	00021400 	sll	v0,v0,0x10
80001898:	00021403 	sra	v0,v0,0x10
        for (j = 0; j < N; j++)
8000189c:	24630004 	addiu	v1,v1,4
800018a0:	506a000c 	beql	v1,t2,800018d4 <matrix_sum+0x74>
800018a4:	256b0001 	addiu	t3,t3,1
            cur = C[i * N + j];
800018a8:	00e04025 	move	t0,a3
800018ac:	8c670000 	lw	a3,0(v1)
            tmp += cur;
800018b0:	00a72821 	addu	a1,a1,a3
            if (tmp > clipval)
800018b4:	00c5482a 	slt	t1,a2,a1
800018b8:	5120fff5 	beqzl	t1,80001890 <matrix_sum+0x30>
800018bc:	0107402a 	slt	t0,t0,a3
                ret += 10;
800018c0:	2442000a 	addiu	v0,v0,10
800018c4:	00021400 	sll	v0,v0,0x10
800018c8:	00021403 	sra	v0,v0,0x10
                tmp = 0;
800018cc:	1000fff3 	b	8000189c <matrix_sum+0x3c>
800018d0:	00002825 	move	a1,zero
    for (i = 0; i < N; i++)
800018d4:	108b0005 	beq	a0,t3,800018ec <matrix_sum+0x8c>
800018d8:	018d5023 	subu	t2,t4,t5
        for (j = 0; j < N; j++)
800018dc:	014e6021 	addu	t4,t2,t6
{
800018e0:	1000fff1 	b	800018a8 <matrix_sum+0x48>
800018e4:	01801825 	move	v1,t4
    ee_s16 ret = 0;
800018e8:	00001025 	move	v0,zero
            }
            prev = cur;
        }
    }
    return ret;
}
800018ec:	03e00008 	jr	ra
800018f0:	00000000 	nop

800018f4 <matrix_mul_const>:
*/
void
matrix_mul_const(ee_u32 N, MATRES *C, MATDAT *A, MATDAT val)
{
    ee_u32 i, j;
    for (i = 0; i < N; i++)
800018f4:	10800017 	beqz	a0,80001954 <matrix_mul_const+0x60>
800018f8:	00044040 	sll	t0,a0,0x1
800018fc:	00c84021 	addu	t0,a2,t0
80001900:	00046823 	negu	t5,a0
80001904:	000d6840 	sll	t5,t5,0x1
80001908:	00005025 	move	t2,zero
8000190c:	00004825 	move	t1,zero
80001910:	00046023 	negu	t4,a0
80001914:	000c6080 	sll	t4,t4,0x2
    {
        for (j = 0; j < N; j++)
80001918:	010d5821 	addu	t3,t0,t5
8000191c:	000a1880 	sll	v1,t2,0x2
80001920:	00a31821 	addu	v1,a1,v1
{
80001924:	01601025 	move	v0,t3
        {
            C[i * N + j] = (MATRES)A[i * N + j] * (MATRES)val;
80001928:	84460000 	lh	a2,0(v0)
8000192c:	00c70018 	mult	a2,a3
80001930:	00003012 	mflo	a2
80001934:	ac660000 	sw	a2,0(v1)
        for (j = 0; j < N; j++)
80001938:	24420002 	addiu	v0,v0,2
8000193c:	1448fffa 	bne	v0,t0,80001928 <matrix_mul_const+0x34>
80001940:	24630004 	addiu	v1,v1,4
    for (i = 0; i < N; i++)
80001944:	25290001 	addiu	t1,t1,1
80001948:	01445021 	addu	t2,t2,a0
8000194c:	1489fff2 	bne	a0,t1,80001918 <matrix_mul_const+0x24>
80001950:	016c4023 	subu	t0,t3,t4
        }
    }
}
80001954:	03e00008 	jr	ra
80001958:	00000000 	nop

8000195c <matrix_add_const>:
*/
void
matrix_add_const(ee_u32 N, MATDAT *A, MATDAT val)
{
    ee_u32 i, j;
    for (i = 0; i < N; i++)
8000195c:	10800012 	beqz	a0,800019a8 <matrix_add_const+0x4c>
80001960:	00041040 	sll	v0,a0,0x1
80001964:	00a22821 	addu	a1,a1,v0
80001968:	00045023 	negu	t2,a0
8000196c:	000a5040 	sll	t2,t2,0x1
80001970:	00003825 	move	a3,zero
80001974:	00044823 	negu	t1,a0
80001978:	00094880 	sll	t1,t1,0x2
    {
        for (j = 0; j < N; j++)
8000197c:	00aa4021 	addu	t0,a1,t2
{
80001980:	01001025 	move	v0,t0
        {
            A[i * N + j] += val;
80001984:	94430000 	lhu	v1,0(v0)
80001988:	00c31821 	addu	v1,a2,v1
8000198c:	a4430000 	sh	v1,0(v0)
        for (j = 0; j < N; j++)
80001990:	24420002 	addiu	v0,v0,2
80001994:	5445fffc 	bnel	v0,a1,80001988 <matrix_add_const+0x2c>
80001998:	94430000 	lhu	v1,0(v0)
    for (i = 0; i < N; i++)
8000199c:	24e70001 	addiu	a3,a3,1
800019a0:	1487fff6 	bne	a0,a3,8000197c <matrix_add_const+0x20>
800019a4:	01092823 	subu	a1,t0,t1
        }
    }
}
800019a8:	03e00008 	jr	ra
800019ac:	00000000 	nop

800019b0 <matrix_mul_vect>:
*/
void
matrix_mul_vect(ee_u32 N, MATRES *C, MATDAT *A, MATDAT *B)
{
    ee_u32 i, j;
    for (i = 0; i < N; i++)
800019b0:	10800018 	beqz	a0,80001a14 <matrix_mul_vect+0x64>
800019b4:	00a05825 	move	t3,a1
800019b8:	00046880 	sll	t5,a0,0x2
800019bc:	00ad6821 	addu	t5,a1,t5
800019c0:	00045040 	sll	t2,a0,0x1
800019c4:	00ea5021 	addu	t2,a3,t2
800019c8:	00006025 	move	t4,zero
    {
        C[i] = 0;
800019cc:	01604025 	move	t0,t3
800019d0:	ad600000 	sw	zero,0(t3)
        for (j = 0; j < N; j++)
800019d4:	000c2840 	sll	a1,t4,0x1
800019d8:	00c52821 	addu	a1,a2,a1
        C[i] = 0;
800019dc:	00e01025 	move	v0,a3
        {
            C[i] += (MATRES)A[i * N + j] * (MATRES)B[j];
800019e0:	84a90000 	lh	t1,0(a1)
800019e4:	84430000 	lh	v1,0(v0)
800019e8:	01230018 	mult	t1,v1
800019ec:	8d030000 	lw	v1,0(t0)
800019f0:	00004812 	mflo	t1
800019f4:	00691821 	addu	v1,v1,t1
800019f8:	ad030000 	sw	v1,0(t0)
        for (j = 0; j < N; j++)
800019fc:	24420002 	addiu	v0,v0,2
80001a00:	144afff7 	bne	v0,t2,800019e0 <matrix_mul_vect+0x30>
80001a04:	24a50002 	addiu	a1,a1,2
    for (i = 0; i < N; i++)
80001a08:	256b0004 	addiu	t3,t3,4
80001a0c:	156dffef 	bne	t3,t5,800019cc <matrix_mul_vect+0x1c>
80001a10:	01846021 	addu	t4,t4,a0
        }
    }
}
80001a14:	03e00008 	jr	ra
80001a18:	00000000 	nop

80001a1c <matrix_mul_matrix>:
*/
void
matrix_mul_matrix(ee_u32 N, MATRES *C, MATDAT *A, MATDAT *B)
{
    ee_u32 i, j, k;
    for (i = 0; i < N; i++)
80001a1c:	1080002d 	beqz	a0,80001ad4 <matrix_mul_matrix+0xb8>
80001a20:	00806825 	move	t5,a0
{
80001a24:	27bdfff8 	addiu	sp,sp,-8
80001a28:	ffb00000 	sd	s0,0(sp)
80001a2c:	00a0c025 	move	t8,a1
80001a30:	00e0c825 	move	t9,a3
80001a34:	00044040 	sll	t0,a0,0x1
80001a38:	00c06025 	move	t4,a2
80001a3c:	00c83821 	addu	a3,a2,t0
    for (i = 0; i < N; i++)
80001a40:	00007025 	move	t6,zero
80001a44:	00007825 	move	t7,zero
    {
        for (j = 0; j < N; j++)
80001a48:	1000001a 	b	80001ab4 <matrix_mul_matrix+0x98>
80001a4c:	00008025 	move	s0,zero
80001a50:	00405825 	move	t3,v0
        {
            C[i * N + j] = 0;
80001a54:	01202825 	move	a1,t1
80001a58:	ad200000 	sw	zero,0(t1)
80001a5c:	01402025 	move	a0,t2
80001a60:	01801025 	move	v0,t4
            for (k = 0; k < N; k++)
            {
                C[i * N + j] += (MATRES)A[i * N + k] * (MATRES)B[k * N + j];
80001a64:	84460000 	lh	a2,0(v0)
80001a68:	84830000 	lh	v1,0(a0)
80001a6c:	00c30018 	mult	a2,v1
80001a70:	8ca30000 	lw	v1,0(a1)
80001a74:	00003012 	mflo	a2
80001a78:	00661821 	addu	v1,v1,a2
80001a7c:	aca30000 	sw	v1,0(a1)
            for (k = 0; k < N; k++)
80001a80:	24420002 	addiu	v0,v0,2
80001a84:	1447fff7 	bne	v0,a3,80001a64 <matrix_mul_matrix+0x48>
80001a88:	00882021 	addu	a0,a0,t0
        for (j = 0; j < N; j++)
80001a8c:	25620001 	addiu	v0,t3,1
80001a90:	25290004 	addiu	t1,t1,4
80001a94:	15a2ffee 	bne	t5,v0,80001a50 <matrix_mul_matrix+0x34>
80001a98:	254a0002 	addiu	t2,t2,2
    for (i = 0; i < N; i++)
80001a9c:	25e30001 	addiu	v1,t7,1
80001aa0:	01886021 	addu	t4,t4,t0
80001aa4:	01c27021 	addu	t6,t6,v0
80001aa8:	11eb0007 	beq	t7,t3,80001ac8 <matrix_mul_matrix+0xac>
80001aac:	00e83821 	addu	a3,a3,t0
80001ab0:	00607825 	move	t7,v1
        for (j = 0; j < N; j++)
80001ab4:	000e4880 	sll	t1,t6,0x2
80001ab8:	03094821 	addu	t1,t8,t1
80001abc:	03205025 	move	t2,t9
80001ac0:	1000ffe4 	b	80001a54 <matrix_mul_matrix+0x38>
80001ac4:	02005825 	move	t3,s0
            }
        }
    }
}
80001ac8:	dfb00000 	ld	s0,0(sp)
80001acc:	03e00008 	jr	ra
80001ad0:	27bd0008 	addiu	sp,sp,8
80001ad4:	03e00008 	jr	ra
80001ad8:	00000000 	nop

80001adc <matrix_mul_matrix_bitextract>:
*/
void
matrix_mul_matrix_bitextract(ee_u32 N, MATRES *C, MATDAT *A, MATDAT *B)
{
    ee_u32 i, j, k;
    for (i = 0; i < N; i++)
80001adc:	10800033 	beqz	a0,80001bac <matrix_mul_matrix_bitextract+0xd0>
80001ae0:	00806825 	move	t5,a0
{
80001ae4:	27bdfff8 	addiu	sp,sp,-8
80001ae8:	ffb00000 	sd	s0,0(sp)
80001aec:	00a0c025 	move	t8,a1
80001af0:	00e0c825 	move	t9,a3
80001af4:	00044040 	sll	t0,a0,0x1
80001af8:	00c06025 	move	t4,a2
80001afc:	00c83821 	addu	a3,a2,t0
    for (i = 0; i < N; i++)
80001b00:	00007025 	move	t6,zero
80001b04:	00007825 	move	t7,zero
    {
        for (j = 0; j < N; j++)
80001b08:	10000020 	b	80001b8c <matrix_mul_matrix_bitextract+0xb0>
80001b0c:	00008025 	move	s0,zero
80001b10:	00405825 	move	t3,v0
        {
            C[i * N + j] = 0;
80001b14:	01203025 	move	a2,t1
80001b18:	ad200000 	sw	zero,0(t1)
80001b1c:	01402825 	move	a1,t2
80001b20:	01801825 	move	v1,t4
            for (k = 0; k < N; k++)
            {
                MATRES tmp = (MATRES)A[i * N + k] * (MATRES)B[k * N + j];
80001b24:	84640000 	lh	a0,0(v1)
80001b28:	84a20000 	lh	v0,0(a1)
80001b2c:	00820018 	mult	a0,v0
80001b30:	00001012 	mflo	v0
                C[i * N + j] += bit_extract(tmp, 2, 4) * bit_extract(tmp, 5, 7);
80001b34:	00022083 	sra	a0,v0,0x2
80001b38:	3084000f 	andi	a0,a0,0xf
80001b3c:	00021143 	sra	v0,v0,0x5
80001b40:	3042007f 	andi	v0,v0,0x7f
80001b44:	00820018 	mult	a0,v0
80001b48:	8cc20000 	lw	v0,0(a2)
80001b4c:	00002012 	mflo	a0
80001b50:	00441021 	addu	v0,v0,a0
80001b54:	acc20000 	sw	v0,0(a2)
            for (k = 0; k < N; k++)
80001b58:	24630002 	addiu	v1,v1,2
80001b5c:	1467fff1 	bne	v1,a3,80001b24 <matrix_mul_matrix_bitextract+0x48>
80001b60:	00a82821 	addu	a1,a1,t0
        for (j = 0; j < N; j++)
80001b64:	25620001 	addiu	v0,t3,1
80001b68:	25290004 	addiu	t1,t1,4
80001b6c:	15a2ffe8 	bne	t5,v0,80001b10 <matrix_mul_matrix_bitextract+0x34>
80001b70:	254a0002 	addiu	t2,t2,2
    for (i = 0; i < N; i++)
80001b74:	25e30001 	addiu	v1,t7,1
80001b78:	01886021 	addu	t4,t4,t0
80001b7c:	01c27021 	addu	t6,t6,v0
80001b80:	11eb0007 	beq	t7,t3,80001ba0 <matrix_mul_matrix_bitextract+0xc4>
80001b84:	00e83821 	addu	a3,a3,t0
80001b88:	00607825 	move	t7,v1
        for (j = 0; j < N; j++)
80001b8c:	000e4880 	sll	t1,t6,0x2
80001b90:	03094821 	addu	t1,t8,t1
80001b94:	03205025 	move	t2,t9
80001b98:	1000ffde 	b	80001b14 <matrix_mul_matrix_bitextract+0x38>
80001b9c:	02005825 	move	t3,s0
            }
        }
    }
}
80001ba0:	dfb00000 	ld	s0,0(sp)
80001ba4:	03e00008 	jr	ra
80001ba8:	27bd0008 	addiu	sp,sp,8
80001bac:	03e00008 	jr	ra
80001bb0:	00000000 	nop

80001bb4 <matrix_test>:
{
80001bb4:	27bdffa0 	addiu	sp,sp,-96
80001bb8:	ffbf0058 	sd	ra,88(sp)
80001bbc:	ffb60050 	sd	s6,80(sp)
80001bc0:	ffb50048 	sd	s5,72(sp)
80001bc4:	ffb40040 	sd	s4,64(sp)
80001bc8:	ffb30038 	sd	s3,56(sp)
80001bcc:	ffb20030 	sd	s2,48(sp)
80001bd0:	ffb10028 	sd	s1,40(sp)
80001bd4:	ffb00020 	sd	s0,32(sp)
80001bd8:	00808025 	move	s0,a0
80001bdc:	00a08825 	move	s1,a1
80001be0:	00c09025 	move	s2,a2
80001be4:	00e0a825 	move	s5,a3
80001be8:	87b30086 	lh	s3,134(sp)
    MATDAT clipval = matrix_big(val);
80001bec:	2414f000 	li	s4,-4096
80001bf0:	0274a025 	or	s4,s3,s4
    matrix_add_const(N, A, val); /* make sure data changes  */
80001bf4:	02603025 	move	a2,s3
80001bf8:	0c000657 	jal	8000195c <matrix_add_const>
80001bfc:	02402825 	move	a1,s2
    matrix_mul_const(N, C, A, val);
80001c00:	02603825 	move	a3,s3
80001c04:	02403025 	move	a2,s2
80001c08:	02202825 	move	a1,s1
80001c0c:	0c00063d 	jal	800018f4 <matrix_mul_const>
80001c10:	02002025 	move	a0,s0
    crc = crc16(matrix_sum(N, C, clipval), crc);
80001c14:	02803025 	move	a2,s4
80001c18:	02202825 	move	a1,s1
80001c1c:	0c000618 	jal	80001860 <matrix_sum>
80001c20:	02002025 	move	a0,s0
80001c24:	00002825 	move	a1,zero
80001c28:	0c000969 	jal	800025a4 <crc16>
80001c2c:	00402025 	move	a0,v0
80001c30:	0040b025 	move	s6,v0
    matrix_mul_vect(N, C, A, B);
80001c34:	02a03825 	move	a3,s5
80001c38:	02403025 	move	a2,s2
80001c3c:	02202825 	move	a1,s1
80001c40:	0c00066c 	jal	800019b0 <matrix_mul_vect>
80001c44:	02002025 	move	a0,s0
    crc = crc16(matrix_sum(N, C, clipval), crc);
80001c48:	02803025 	move	a2,s4
80001c4c:	02202825 	move	a1,s1
80001c50:	0c000618 	jal	80001860 <matrix_sum>
80001c54:	02002025 	move	a0,s0
80001c58:	02c02825 	move	a1,s6
80001c5c:	0c000969 	jal	800025a4 <crc16>
80001c60:	00402025 	move	a0,v0
80001c64:	0040b025 	move	s6,v0
    matrix_mul_matrix(N, C, A, B);
80001c68:	02a03825 	move	a3,s5
80001c6c:	02403025 	move	a2,s2
80001c70:	02202825 	move	a1,s1
80001c74:	0c000687 	jal	80001a1c <matrix_mul_matrix>
80001c78:	02002025 	move	a0,s0
    crc = crc16(matrix_sum(N, C, clipval), crc);
80001c7c:	02803025 	move	a2,s4
80001c80:	02202825 	move	a1,s1
80001c84:	0c000618 	jal	80001860 <matrix_sum>
80001c88:	02002025 	move	a0,s0
80001c8c:	02c02825 	move	a1,s6
80001c90:	0c000969 	jal	800025a4 <crc16>
80001c94:	00402025 	move	a0,v0
80001c98:	0040b025 	move	s6,v0
    matrix_mul_matrix_bitextract(N, C, A, B);
80001c9c:	02a03825 	move	a3,s5
80001ca0:	02403025 	move	a2,s2
80001ca4:	02202825 	move	a1,s1
80001ca8:	0c0006b7 	jal	80001adc <matrix_mul_matrix_bitextract>
80001cac:	02002025 	move	a0,s0
    crc = crc16(matrix_sum(N, C, clipval), crc);
80001cb0:	02803025 	move	a2,s4
80001cb4:	02202825 	move	a1,s1
80001cb8:	0c000618 	jal	80001860 <matrix_sum>
80001cbc:	02002025 	move	a0,s0
80001cc0:	02c02825 	move	a1,s6
80001cc4:	0c000969 	jal	800025a4 <crc16>
80001cc8:	00402025 	move	a0,v0
80001ccc:	00408825 	move	s1,v0
    matrix_add_const(N, A, -val); /* return matrix to initial value */
80001cd0:	00133023 	negu	a2,s3
80001cd4:	00063400 	sll	a2,a2,0x10
80001cd8:	00063403 	sra	a2,a2,0x10
80001cdc:	02402825 	move	a1,s2
80001ce0:	0c000657 	jal	8000195c <matrix_add_const>
80001ce4:	02002025 	move	a0,s0
}
80001ce8:	00111400 	sll	v0,s1,0x10
80001cec:	00021403 	sra	v0,v0,0x10
80001cf0:	dfbf0058 	ld	ra,88(sp)
80001cf4:	dfb60050 	ld	s6,80(sp)
80001cf8:	dfb50048 	ld	s5,72(sp)
80001cfc:	dfb40040 	ld	s4,64(sp)
80001d00:	dfb30038 	ld	s3,56(sp)
80001d04:	dfb20030 	ld	s2,48(sp)
80001d08:	dfb10028 	ld	s1,40(sp)
80001d0c:	dfb00020 	ld	s0,32(sp)
80001d10:	03e00008 	jr	ra
80001d14:	27bd0060 	addiu	sp,sp,96

80001d18 <core_bench_matrix>:
{
80001d18:	27bdffc8 	addiu	sp,sp,-56
80001d1c:	ffbf0030 	sd	ra,48(sp)
80001d20:	ffb00028 	sd	s0,40(sp)
80001d24:	00c08025 	move	s0,a2
    crc = crc16(matrix_test(N, C, A, B, val), crc);
80001d28:	afa50024 	sw	a1,36(sp)
80001d2c:	8c870008 	lw	a3,8(a0)
80001d30:	8c860004 	lw	a2,4(a0)
80001d34:	8c85000c 	lw	a1,12(a0)
80001d38:	0c0006ed 	jal	80001bb4 <matrix_test>
80001d3c:	8c840000 	lw	a0,0(a0)
80001d40:	02002825 	move	a1,s0
80001d44:	0c000969 	jal	800025a4 <crc16>
80001d48:	00402025 	move	a0,v0
}
80001d4c:	dfbf0030 	ld	ra,48(sp)
80001d50:	dfb00028 	ld	s0,40(sp)
80001d54:	03e00008 	jr	ra
80001d58:	27bd0038 	addiu	sp,sp,56

80001d5c <portable_malloc>:
#if (MEM_METHOD==MEM_MALLOC)
#include <malloc.h>
/* Function: portable_malloc
	Provide malloc() functionality in a platform specific way.
*/
void *portable_malloc(size_t size) {
80001d5c:	27bdffd0 	addiu	sp,sp,-48
80001d60:	ffbf0028 	sd	ra,40(sp)
80001d64:	ffb00020 	sd	s0,32(sp)
	void *mem = malloc(size);
80001d68:	0c001169 	jal	800045a4 <malloc>
80001d6c:	00808025 	move	s0,a0
	if (!mem) {
80001d70:	10400004 	beqz	v0,80001d84 <portable_malloc+0x28>
80001d74:	dfbf0028 	ld	ra,40(sp)
	    printf("Failed to allocated %d bytes!\n", size);
	    console_render();
	    while(1);
	}
	return mem;
}
80001d78:	dfb00020 	ld	s0,32(sp)
80001d7c:	03e00008 	jr	ra
80001d80:	27bd0030 	addiu	sp,sp,48
	    printf("Failed to allocated %d bytes!\n", size);
80001d84:	02002825 	move	a1,s0
80001d88:	3c048002 	lui	a0,0x8002
80001d8c:	0c001bd6 	jal	80006f58 <printf>
80001d90:	2484b500 	addiu	a0,a0,-19200
	    console_render();
80001d94:	0c00619e 	jal	80018678 <console_render>
80001d98:	00000000 	nop
	    while(1);
80001d9c:	1000ffff 	b	80001d9c <portable_malloc+0x40>
80001da0:	00000000 	nop

80001da4 <portable_free>:
/* Function: portable_free
	Provide free() functionality in a platform specific way.
*/
void portable_free(void *p) {
80001da4:	27bdffd8 	addiu	sp,sp,-40
80001da8:	ffbf0020 	sd	ra,32(sp)
	free(p);
80001dac:	0c001179 	jal	800045e4 <free>
80001db0:	00000000 	nop
}
80001db4:	dfbf0020 	ld	ra,32(sp)
80001db8:	03e00008 	jr	ra
80001dbc:	27bd0028 	addiu	sp,sp,40

80001dc0 <start_time>:
	This function will be called right before starting the timed portion of the benchmark.

	Implementation may be capturing a system timer (as implemented in the example code) 
	or zeroing some system parameters - e.g. setting the cpu clocks cycles to 0.
*/
void start_time(void) {
80001dc0:	27bdffd8 	addiu	sp,sp,-40
80001dc4:	ffbf0020 	sd	ra,32(sp)
	GETMYTIME(&start_time_val );      
80001dc8:	0c000a39 	jal	800028e4 <clock>
80001dcc:	00000000 	nop
80001dd0:	af8280f8 	sw	v0,-32520(gp)
	CALLGRIND_START_INSTRUMENTATION
#endif
#if MICA
    asm volatile("int3");/*1 */
#endif
}
80001dd4:	dfbf0020 	ld	ra,32(sp)
80001dd8:	03e00008 	jr	ra
80001ddc:	27bd0028 	addiu	sp,sp,40

80001de0 <stop_time>:
	This function will be called right after ending the timed portion of the benchmark.

	Implementation may be capturing a system timer (as implemented in the example code) 
	or other system parameters - e.g. reading the current value of cpu cycles counter.
*/
void stop_time(void) {
80001de0:	27bdffd8 	addiu	sp,sp,-40
80001de4:	ffbf0020 	sd	ra,32(sp)
	 CALLGRIND_STOP_INSTRUMENTATION 
#endif
#if MICA
    asm volatile("int3");/*1 */
#endif
	GETMYTIME(&stop_time_val );      
80001de8:	0c000a39 	jal	800028e4 <clock>
80001dec:	00000000 	nop
80001df0:	af8280f4 	sw	v0,-32524(gp)
}
80001df4:	dfbf0020 	ld	ra,32(sp)
80001df8:	03e00008 	jr	ra
80001dfc:	27bd0028 	addiu	sp,sp,40

80001e00 <get_time>:
	and the resolution is controlled by <TIMER_RES_DIVIDER>
*/
CORE_TICKS get_time(void) {
	CORE_TICKS elapsed=(CORE_TICKS)(MYTIMEDIFF(stop_time_val, start_time_val));
	return elapsed;
}
80001e00:	8f8380f4 	lw	v1,-32524(gp)
80001e04:	8f8280f8 	lw	v0,-32520(gp)
80001e08:	03e00008 	jr	ra
80001e0c:	00621023 	subu	v0,v1,v0

80001e10 <time_in_secs>:

	The <secs_ret> type is used to accomodate systems with no support for floating point.
	Default implementation implemented by the EE_TICKS_PER_SEC macro above.
*/
secs_ret time_in_secs(CORE_TICKS ticks) {
	secs_ret retval=((secs_ret)ticks) / (secs_ret)EE_TICKS_PER_SEC;
80001e10:	0004203c 	dsll32	a0,a0,0x0
80001e14:	0004203e 	dsrl32	a0,a0,0x0
80001e18:	44a40000 	dmtc1	a0,$f0
80001e1c:	00000000 	nop
80001e20:	46a00021 	cvt.d.l	$f0,$f0
	return retval;
}
80001e24:	d7818020 	ldc1	$f1,-32736(gp)
80001e28:	03e00008 	jr	ra
80001e2c:	46210003 	div.d	$f0,$f0,$f1

80001e30 <portable_init>:
/* Function: portable_init
	Target specific initialization code 
	Test for some common mistakes.
*/
void portable_init(core_portable *p, int *argc, char *argv[])
{
80001e30:	27bdffc0 	addiu	sp,sp,-64
80001e34:	ffbf0038 	sd	ra,56(sp)
80001e38:	ffb10030 	sd	s1,48(sp)
80001e3c:	ffb00028 	sd	s0,40(sp)
    /* enable interrupts (on the CPU) */
    init_interrupts();
80001e40:	0c005e9c 	jal	80017a70 <init_interrupts>
80001e44:	00808025 	move	s0,a0

    /* Initialize peripherals */
    display_init( res, bit, 2, GAMMA_NONE, ANTIALIAS_RESAMPLE );
80001e48:	24110001 	li	s1,1
80001e4c:	afb10024 	sw	s1,36(sp)
80001e50:	00003825 	move	a3,zero
80001e54:	24060002 	li	a2,2
80001e58:	24050001 	li	a1,1
80001e5c:	0c005f1a 	jal	80017c68 <display_init>
80001e60:	00002025 	move	a0,zero
    console_init();
80001e64:	0c006154 	jal	80018550 <console_init>
80001e68:	00000000 	nop
    controller_init();
80001e6c:	0c0061ae 	jal	800186b8 <controller_init>
80001e70:	00000000 	nop
    timer_init();
80001e74:	0c0063dc 	jal	80018f70 <timer_init>
80001e78:	00000000 	nop
    rtc_init();
80001e7c:	0c006216 	jal	80018858 <rtc_init>
80001e80:	00000000 	nop

    console_set_render_mode(RENDER_MANUAL);
80001e84:	0c006152 	jal	80018548 <console_set_render_mode>
80001e88:	00002025 	move	a0,zero
    
    console_clear();
80001e8c:	0c006184 	jal	80018610 <console_clear>
80001e90:	00000000 	nop

	ee_printf("CoreMark N64\n");
80001e94:	3c048002 	lui	a0,0x8002
80001e98:	0c001ce2 	jal	80007388 <puts>
80001e9c:	2484b520 	addiu	a0,a0,-19168
			*argc=nargs;
		}
	}
#endif /* sample of potential platform specific init via command line, reset the number of contexts being used if first argument is M<n>*/

    console_render();
80001ea0:	0c00619e 	jal	80018678 <console_render>
80001ea4:	00000000 	nop
    
	p->portable_id=1;
80001ea8:	a2110000 	sb	s1,0(s0)
}
80001eac:	dfbf0038 	ld	ra,56(sp)
80001eb0:	dfb10030 	ld	s1,48(sp)
80001eb4:	dfb00028 	ld	s0,40(sp)
80001eb8:	03e00008 	jr	ra
80001ebc:	27bd0040 	addiu	sp,sp,64

80001ec0 <portable_fini>:
/* Function: portable_fini
	Target specific final code 
*/
void portable_fini(core_portable *p)
{
80001ec0:	27bdffd8 	addiu	sp,sp,-40
80001ec4:	ffbf0020 	sd	ra,32(sp)
	p->portable_id=0;

	console_render();
80001ec8:	0c00619e 	jal	80018678 <console_render>
80001ecc:	a0800000 	sb	zero,0(a0)

	while(1);
80001ed0:	1000ffff 	b	80001ed0 <portable_fini+0x10>
80001ed4:	00000000 	nop

80001ed8 <core_init_state>:
    ee_u8 *buf = 0;
#if CORE_DEBUG
    ee_u8 *start = p;
    ee_printf("State: %d,%d\n", size, seed);
#endif
    size--;
80001ed8:	2489ffff 	addiu	t1,a0,-1
    next = 0;
    while ((total + next + 1) < size)
80001edc:	2d220002 	sltiu	v0,t1,2
80001ee0:	14400051 	bnez	v0,80002028 <core_init_state+0x150>
80001ee4:	24a50001 	addiu	a1,a1,1
{
80001ee8:	27bdfff0 	addiu	sp,sp,-16
80001eec:	ffb10008 	sd	s1,8(sp)
80001ef0:	ffb00000 	sd	s0,0(sp)
80001ef4:	30a5ffff 	andi	a1,a1,0xffff
    ee_u32 total = 0, next = 0, i;
80001ef8:	00001825 	move	v1,zero
            case 6: /* scientific */
                buf  = scipat[(seed >> 3) & 0x3];
                next = 8;
                break;
            case 7: /* invalid */
                buf  = errpat[(seed >> 3) & 0x3];
80001efc:	3c0d8002 	lui	t5,0x8002
80001f00:	25ada3c0 	addiu	t5,t5,-23616
                next = 8;
80001f04:	24070008 	li	a3,8
                buf  = scipat[(seed >> 3) & 0x3];
80001f08:	3c0c8002 	lui	t4,0x8002
80001f0c:	258ca3d0 	addiu	t4,t4,-23600
                buf  = intpat[(seed >> 3) & 0x3];
80001f10:	3c0b8002 	lui	t3,0x8002
80001f14:	256ba3f0 	addiu	t3,t3,-23568
                next = 4;
80001f18:	240e0004 	li	t6,4
                buf  = floatpat[(seed >> 3) & 0x3];
80001f1c:	3c088002 	lui	t0,0x8002
80001f20:	2508a3e0 	addiu	t0,t0,-23584
            *(p + total + i) = ',';
80001f24:	10000006 	b	80001f40 <core_init_state+0x68>
80001f28:	240a002c 	li	t2,44
80001f2c:	01e31821 	addu	v1,t7,v1
80001f30:	a06a0000 	sb	t2,0(v1)
            total += next + 1;
80001f34:	24a50001 	addiu	a1,a1,1
80001f38:	30a5ffff 	andi	a1,a1,0xffff
            *(p + total + i) = ',';
80001f3c:	03201825 	move	v1,t9
        seed++;
80001f40:	00051400 	sll	v0,a1,0x10
        switch (seed & 0x7)
80001f44:	30af0007 	andi	t7,a1,0x7
80001f48:	2df80007 	sltiu	t8,t7,7
80001f4c:	13000023 	beqz	t8,80001fdc <core_init_state+0x104>
80001f50:	00021403 	sra	v0,v0,0x10
80001f54:	2df80005 	sltiu	t8,t7,5
80001f58:	1300001b 	beqz	t8,80001fc8 <core_init_state+0xf0>
80001f5c:	00021042 	srl	v0,v0,0x1
80001f60:	2def0003 	sltiu	t7,t7,3
80001f64:	15e00013 	bnez	t7,80001fb4 <core_init_state+0xdc>
80001f68:	00000000 	nop
                buf  = floatpat[(seed >> 3) & 0x3];
80001f6c:	3042000c 	andi	v0,v0,0xc
80001f70:	00481021 	addu	v0,v0,t0
80001f74:	8c500000 	lw	s0,0(v0)
                next = 8;
80001f78:	00e07825 	move	t7,a3
    while ((total + next + 1) < size)
80001f7c:	24790001 	addiu	t9,v1,1
80001f80:	032fc821 	addu	t9,t9,t7
80001f84:	0329102b 	sltu	v0,t9,t1
80001f88:	1040001a 	beqz	v0,80001ff4 <core_init_state+0x11c>
80001f8c:	02001025 	move	v0,s0
        if (next > 0)
80001f90:	00c3c021 	addu	t8,a2,v1
80001f94:	020f8021 	addu	s0,s0,t7
                *(p + total + i) = buf[i];
80001f98:	90510000 	lbu	s1,0(v0)
80001f9c:	a3110000 	sb	s1,0(t8)
            for (i = 0; i < next; i++)
80001fa0:	24420001 	addiu	v0,v0,1
80001fa4:	1602fffc 	bne	s0,v0,80001f98 <core_init_state+0xc0>
80001fa8:	27180001 	addiu	t8,t8,1
80001fac:	1000ffdf 	b	80001f2c <core_init_state+0x54>
80001fb0:	00cf7821 	addu	t7,a2,t7
                buf  = intpat[(seed >> 3) & 0x3];
80001fb4:	3042000c 	andi	v0,v0,0xc
80001fb8:	004b1021 	addu	v0,v0,t3
80001fbc:	8c500000 	lw	s0,0(v0)
                break;
80001fc0:	1000ffee 	b	80001f7c <core_init_state+0xa4>
80001fc4:	01c07825 	move	t7,t6
                buf  = scipat[(seed >> 3) & 0x3];
80001fc8:	3042000c 	andi	v0,v0,0xc
80001fcc:	004c1021 	addu	v0,v0,t4
80001fd0:	8c500000 	lw	s0,0(v0)
                break;
80001fd4:	1000ffe9 	b	80001f7c <core_init_state+0xa4>
80001fd8:	00e07825 	move	t7,a3
                buf  = errpat[(seed >> 3) & 0x3];
80001fdc:	00021042 	srl	v0,v0,0x1
80001fe0:	3042000c 	andi	v0,v0,0xc
80001fe4:	004d1021 	addu	v0,v0,t5
80001fe8:	8c500000 	lw	s0,0(v0)
                break;
80001fec:	1000ffe3 	b	80001f7c <core_init_state+0xa4>
80001ff0:	00e07825 	move	t7,a3
            default: /* Never happen, just to make some compilers happy */
                break;
        }
    }
    size++;
    while (total < size)
80001ff4:	0064102b 	sltu	v0,v1,a0
80001ff8:	10400008 	beqz	v0,8000201c <core_init_state+0x144>
80001ffc:	dfb10008 	ld	s1,8(sp)
80002000:	00c31821 	addu	v1,a2,v1
80002004:	00c42021 	addu	a0,a2,a0
    { /* fill the rest with 0 */
        *(p + total) = 0;
80002008:	a0600000 	sb	zero,0(v1)
    while (total < size)
8000200c:	24630001 	addiu	v1,v1,1
80002010:	5483fffe 	bnel	a0,v1,8000200c <core_init_state+0x134>
80002014:	a0600000 	sb	zero,0(v1)
        total++;
    }
#if CORE_DEBUG
    ee_printf("State Input: %s\n", start);
#endif
}
80002018:	dfb10008 	ld	s1,8(sp)
8000201c:	dfb00000 	ld	s0,0(sp)
80002020:	03e00008 	jr	ra
80002024:	27bd0010 	addiu	sp,sp,16
    ee_u32 total = 0, next = 0, i;
80002028:	00001825 	move	v1,zero
    while (total < size)
8000202c:	0064102b 	sltu	v0,v1,a0
80002030:	10400009 	beqz	v0,80002058 <core_init_state+0x180>
80002034:	00000000 	nop
80002038:	00c31821 	addu	v1,a2,v1
8000203c:	00c42021 	addu	a0,a2,a0
        *(p + total) = 0;
80002040:	a0600000 	sb	zero,0(v1)
    while (total < size)
80002044:	24630001 	addiu	v1,v1,1
80002048:	5483fffe 	bnel	a0,v1,80002044 <core_init_state+0x16c>
8000204c:	a0600000 	sb	zero,0(v1)
80002050:	03e00008 	jr	ra
80002054:	00000000 	nop
80002058:	03e00008 	jr	ra
8000205c:	00000000 	nop

80002060 <core_state_transition>:
*/

enum CORE_STATE
core_state_transition(ee_u8 **instr, ee_u32 *transition_count)
{
    ee_u8 *         str = *instr;
80002060:	8c860000 	lw	a2,0(a0)
    ee_u8           NEXT_SYMBOL;
    enum CORE_STATE state = CORE_START;
    for (; *str && state != CORE_INVALID; str++)
80002064:	90c30000 	lbu	v1,0(a2)
80002068:	10600092 	beqz	v1,800022b4 <core_state_transition+0x254>
8000206c:	24c70001 	addiu	a3,a2,1
{
80002070:	27bdfff0 	addiu	sp,sp,-16
80002074:	ffb10008 	sd	s1,8(sp)
80002078:	ffb00000 	sd	s0,0(sp)
    enum CORE_STATE state = CORE_START;
8000207c:	00001025 	move	v0,zero
    {
        NEXT_SYMBOL = *str;
        if (NEXT_SYMBOL == ',') /* end of this input */
80002080:	2409002c 	li	t1,44
    for (; *str && state != CORE_INVALID; str++)
80002084:	240a0001 	li	t2,1
80002088:	3c088002 	lui	t0,0x8002
8000208c:	2508a3a0 	addiu	t0,t0,-23648
                }
                break;
            case CORE_EXPONENT:
                if (ee_isdigit(NEXT_SYMBOL))
                {
                    state = CORE_SCIENTIFIC;
80002090:	24100007 	li	s0,7
                    state = CORE_EXPONENT;
80002094:	24190006 	li	t9,6
                if (NEXT_SYMBOL == 'E' || NEXT_SYMBOL == 'e')
80002098:	240effdf 	li	t6,-33
8000209c:	240d0045 	li	t5,69
                    state = CORE_S2;
800020a0:	24180003 	li	t8,3
                if (NEXT_SYMBOL == '.')
800020a4:	240b002e 	li	t3,46
                    state = CORE_FLOAT;
800020a8:	240f0005 	li	t7,5
                    state = CORE_INT;
800020ac:	240c0004 	li	t4,4
                    state = CORE_S1;
800020b0:	10000019 	b	80002118 <core_state_transition+0xb8>
800020b4:	24110002 	li	s1,2
    retval = ((c >= '0') & (c <= '9')) ? 1 : 0;
800020b8:	2466ffd0 	addiu	a2,v1,-48
                if (ee_isdigit(NEXT_SYMBOL))
800020bc:	30c600ff 	andi	a2,a2,0xff
800020c0:	2cc6000a 	sltiu	a2,a2,10
800020c4:	14c0000b 	bnez	a2,800020f4 <core_state_transition+0x94>
800020c8:	01801025 	move	v0,t4
                else if (NEXT_SYMBOL == '+' || NEXT_SYMBOL == '-')
800020cc:	2462ffd5 	addiu	v0,v1,-43
800020d0:	304200fd 	andi	v0,v0,0xfd
800020d4:	50400007 	beqzl	v0,800020f4 <core_state_transition+0x94>
800020d8:	02201025 	move	v0,s1
                else if (NEXT_SYMBOL == '.')
800020dc:	506b0005 	beql	v1,t3,800020f4 <core_state_transition+0x94>
800020e0:	01e01025 	move	v0,t7
                    transition_count[CORE_INVALID]++;
800020e4:	8ca20004 	lw	v0,4(a1)
800020e8:	24420001 	addiu	v0,v0,1
800020ec:	aca20004 	sw	v0,4(a1)
800020f0:	01401025 	move	v0,t2
                transition_count[CORE_START]++;
800020f4:	8ca30000 	lw	v1,0(a1)
800020f8:	24630001 	addiu	v1,v1,1
800020fc:	aca30000 	sw	v1,0(a1)
    for (; *str && state != CORE_INVALID; str++)
80002100:	90e30000 	lbu	v1,0(a3)
80002104:	1060005b 	beqz	v1,80002274 <core_state_transition+0x214>
80002108:	00e03025 	move	a2,a3
8000210c:	504a0059 	beql	v0,t2,80002274 <core_state_transition+0x214>
80002110:	24020001 	li	v0,1
80002114:	24e70001 	addiu	a3,a3,1
        if (NEXT_SYMBOL == ',') /* end of this input */
80002118:	10690056 	beq	v1,t1,80002274 <core_state_transition+0x214>
8000211c:	00e03025 	move	a2,a3
80002120:	00023080 	sll	a2,v0,0x2
80002124:	01063021 	addu	a2,t0,a2
80002128:	8cc60000 	lw	a2,0(a2)
8000212c:	00c00008 	jr	a2
80002130:	00000000 	nop
    retval = ((c >= '0') & (c <= '9')) ? 1 : 0;
80002134:	2462ffd0 	addiu	v0,v1,-48
                if (ee_isdigit(NEXT_SYMBOL))
80002138:	304200ff 	andi	v0,v0,0xff
8000213c:	2c42000a 	sltiu	v0,v0,10
80002140:	10400006 	beqz	v0,8000215c <core_state_transition+0xfc>
80002144:	00000000 	nop
                    transition_count[CORE_S1]++;
80002148:	8ca20008 	lw	v0,8(a1)
8000214c:	24420001 	addiu	v0,v0,1
80002150:	aca20008 	sw	v0,8(a1)
80002154:	10000028 	b	800021f8 <core_state_transition+0x198>
80002158:	01801025 	move	v0,t4
                else if (NEXT_SYMBOL == '.')
8000215c:	106b0004 	beq	v1,t3,80002170 <core_state_transition+0x110>
80002160:	8ca20008 	lw	v0,8(a1)
                    transition_count[CORE_S1]++;
80002164:	24420001 	addiu	v0,v0,1
80002168:	1000003e 	b	80002264 <core_state_transition+0x204>
8000216c:	aca20008 	sw	v0,8(a1)
                    transition_count[CORE_S1]++;
80002170:	24420001 	addiu	v0,v0,1
80002174:	aca20008 	sw	v0,8(a1)
80002178:	1000001f 	b	800021f8 <core_state_transition+0x198>
8000217c:	01e01025 	move	v0,t7
                if (NEXT_SYMBOL == '.')
80002180:	106b0009 	beq	v1,t3,800021a8 <core_state_transition+0x148>
80002184:	2463ffd0 	addiu	v1,v1,-48
                else if (!ee_isdigit(NEXT_SYMBOL))
80002188:	306300ff 	andi	v1,v1,0xff
8000218c:	2c63000a 	sltiu	v1,v1,10
80002190:	5460001a 	bnezl	v1,800021fc <core_state_transition+0x19c>
80002194:	90e30000 	lbu	v1,0(a3)
                    transition_count[CORE_INT]++;
80002198:	8ca20010 	lw	v0,16(a1)
8000219c:	24420001 	addiu	v0,v0,1
800021a0:	10000030 	b	80002264 <core_state_transition+0x204>
800021a4:	aca20010 	sw	v0,16(a1)
                    transition_count[CORE_INT]++;
800021a8:	8ca20010 	lw	v0,16(a1)
800021ac:	24420001 	addiu	v0,v0,1
800021b0:	aca20010 	sw	v0,16(a1)
800021b4:	10000010 	b	800021f8 <core_state_transition+0x198>
800021b8:	01e01025 	move	v0,t7
                if (NEXT_SYMBOL == 'E' || NEXT_SYMBOL == 'e')
800021bc:	006e3024 	and	a2,v1,t6
800021c0:	10cd0009 	beq	a2,t5,800021e8 <core_state_transition+0x188>
800021c4:	2463ffd0 	addiu	v1,v1,-48
                else if (!ee_isdigit(NEXT_SYMBOL))
800021c8:	306300ff 	andi	v1,v1,0xff
800021cc:	2c63000a 	sltiu	v1,v1,10
800021d0:	5460000a 	bnezl	v1,800021fc <core_state_transition+0x19c>
800021d4:	90e30000 	lbu	v1,0(a3)
                    transition_count[CORE_FLOAT]++;
800021d8:	8ca20014 	lw	v0,20(a1)
800021dc:	24420001 	addiu	v0,v0,1
800021e0:	10000020 	b	80002264 <core_state_transition+0x204>
800021e4:	aca20014 	sw	v0,20(a1)
                    transition_count[CORE_FLOAT]++;
800021e8:	8ca20014 	lw	v0,20(a1)
800021ec:	24420001 	addiu	v0,v0,1
800021f0:	aca20014 	sw	v0,20(a1)
                    state = CORE_S2;
800021f4:	03001025 	move	v0,t8
    for (; *str && state != CORE_INVALID; str++)
800021f8:	90e30000 	lbu	v1,0(a3)
800021fc:	1460ffc5 	bnez	v1,80002114 <core_state_transition+0xb4>
80002200:	00e03025 	move	a2,a3
                break;
            default:
                break;
        }
    }
    *instr = str;
80002204:	1000001c 	b	80002278 <core_state_transition+0x218>
80002208:	ac860000 	sw	a2,0(a0)
                if (NEXT_SYMBOL == '+' || NEXT_SYMBOL == '-')
8000220c:	2463ffd5 	addiu	v1,v1,-43
80002210:	306300fd 	andi	v1,v1,0xfd
80002214:	10600004 	beqz	v1,80002228 <core_state_transition+0x1c8>
80002218:	8ca2000c 	lw	v0,12(a1)
                    transition_count[CORE_S2]++;
8000221c:	24420001 	addiu	v0,v0,1
80002220:	10000010 	b	80002264 <core_state_transition+0x204>
80002224:	aca2000c 	sw	v0,12(a1)
                    transition_count[CORE_S2]++;
80002228:	24420001 	addiu	v0,v0,1
8000222c:	aca2000c 	sw	v0,12(a1)
80002230:	1000fff1 	b	800021f8 <core_state_transition+0x198>
80002234:	03201025 	move	v0,t9
    retval = ((c >= '0') & (c <= '9')) ? 1 : 0;
80002238:	2463ffd0 	addiu	v1,v1,-48
                if (ee_isdigit(NEXT_SYMBOL))
8000223c:	306300ff 	andi	v1,v1,0xff
80002240:	2c63000a 	sltiu	v1,v1,10
80002244:	10600005 	beqz	v1,8000225c <core_state_transition+0x1fc>
80002248:	8ca20018 	lw	v0,24(a1)
                    transition_count[CORE_EXPONENT]++;
8000224c:	24420001 	addiu	v0,v0,1
80002250:	aca20018 	sw	v0,24(a1)
80002254:	1000ffe8 	b	800021f8 <core_state_transition+0x198>
80002258:	02001025 	move	v0,s0
                    transition_count[CORE_EXPONENT]++;
8000225c:	24420001 	addiu	v0,v0,1
80002260:	aca20018 	sw	v0,24(a1)
    for (; *str && state != CORE_INVALID; str++)
80002264:	90e20000 	lbu	v0,0(a3)
80002268:	14400010 	bnez	v0,800022ac <core_state_transition+0x24c>
8000226c:	00e03025 	move	a2,a3
80002270:	24020001 	li	v0,1
    *instr = str;
80002274:	ac860000 	sw	a2,0(a0)
    return state;
}
80002278:	dfb10008 	ld	s1,8(sp)
8000227c:	dfb00000 	ld	s0,0(sp)
80002280:	03e00008 	jr	ra
80002284:	27bd0010 	addiu	sp,sp,16
    retval = ((c >= '0') & (c <= '9')) ? 1 : 0;
80002288:	2463ffd0 	addiu	v1,v1,-48
                if (!ee_isdigit(NEXT_SYMBOL))
8000228c:	306300ff 	andi	v1,v1,0xff
80002290:	2c63000a 	sltiu	v1,v1,10
80002294:	5460ffd9 	bnezl	v1,800021fc <core_state_transition+0x19c>
80002298:	90e30000 	lbu	v1,0(a3)
                    transition_count[CORE_INVALID]++;
8000229c:	8ca20004 	lw	v0,4(a1)
800022a0:	24420001 	addiu	v0,v0,1
800022a4:	1000ffef 	b	80002264 <core_state_transition+0x204>
800022a8:	aca20004 	sw	v0,4(a1)
    enum CORE_STATE state = CORE_START;
800022ac:	1000fff1 	b	80002274 <core_state_transition+0x214>
800022b0:	24020001 	li	v0,1
800022b4:	00001025 	move	v0,zero
}
800022b8:	03e00008 	jr	ra
800022bc:	ac860000 	sw	a2,0(a0)

800022c0 <core_bench_state>:
{
800022c0:	27bdff50 	addiu	sp,sp,-176
800022c4:	ffbf00a8 	sd	ra,168(sp)
800022c8:	ffb700a0 	sd	s7,160(sp)
800022cc:	ffb60098 	sd	s6,152(sp)
800022d0:	ffb50090 	sd	s5,144(sp)
800022d4:	ffb40088 	sd	s4,136(sp)
800022d8:	ffb30080 	sd	s3,128(sp)
800022dc:	ffb20078 	sd	s2,120(sp)
800022e0:	ffb10070 	sd	s1,112(sp)
800022e4:	ffb00068 	sd	s0,104(sp)
800022e8:	0080a025 	move	s4,a0
800022ec:	00a08025 	move	s0,a1
800022f0:	00c0b825 	move	s7,a2
800022f4:	00e0b025 	move	s6,a3
800022f8:	87b500d6 	lh	s5,214(sp)
800022fc:	97b200de 	lhu	s2,222(sp)
    ee_u8 *p = memblock;
80002300:	afa50060 	sw	a1,96(sp)
    for (i = 0; i < NUM_CORE_STATES; i++)
80002304:	27b30040 	addiu	s3,sp,64
80002308:	27b10020 	addiu	s1,sp,32
8000230c:	27a40060 	addiu	a0,sp,96
    ee_u8 *p = memblock;
80002310:	02201825 	move	v1,s1
80002314:	02601025 	move	v0,s3
        final_counts[i] = track_counts[i] = 0;
80002318:	ac400000 	sw	zero,0(v0)
8000231c:	ac600000 	sw	zero,0(v1)
    for (i = 0; i < NUM_CORE_STATES; i++)
80002320:	24420004 	addiu	v0,v0,4
80002324:	1482fffc 	bne	a0,v0,80002318 <core_bench_state+0x58>
80002328:	24630004 	addiu	v1,v1,4
    while (*p != 0)
8000232c:	92020000 	lbu	v0,0(s0)
80002330:	5040000f 	beqzl	v0,80002370 <core_bench_state+0xb0>
80002334:	0214a021 	addu	s4,s0,s4
        enum CORE_STATE fstate = core_state_transition(&p, track_counts);
80002338:	27a50040 	addiu	a1,sp,64
8000233c:	0c000818 	jal	80002060 <core_state_transition>
80002340:	27a40060 	addiu	a0,sp,96
        final_counts[fstate]++;
80002344:	00021080 	sll	v0,v0,0x2
80002348:	27a30020 	addiu	v1,sp,32
8000234c:	00621021 	addu	v0,v1,v0
80002350:	8c430000 	lw	v1,0(v0)
80002354:	24630001 	addiu	v1,v1,1
80002358:	ac430000 	sw	v1,0(v0)
    while (*p != 0)
8000235c:	8fa20060 	lw	v0,96(sp)
80002360:	90420000 	lbu	v0,0(v0)
80002364:	1440fff5 	bnez	v0,8000233c <core_bench_state+0x7c>
80002368:	27a50040 	addiu	a1,sp,64
    while (p < (memblock + blksize))
8000236c:	0214a021 	addu	s4,s0,s4
80002370:	0214102b 	sltu	v0,s0,s4
80002374:	1040002d 	beqz	v0,8000242c <core_bench_state+0x16c>
80002378:	afb00060 	sw	s0,96(sp)
8000237c:	02001025 	move	v0,s0
        if (*p != ',')
80002380:	10000006 	b	8000239c <core_bench_state+0xdc>
80002384:	2404002c 	li	a0,44
        p += step;
80002388:	8fa20060 	lw	v0,96(sp)
8000238c:	00551021 	addu	v0,v0,s5
    while (p < (memblock + blksize))
80002390:	0054182b 	sltu	v1,v0,s4
80002394:	10600006 	beqz	v1,800023b0 <core_bench_state+0xf0>
80002398:	afa20060 	sw	v0,96(sp)
        if (*p != ',')
8000239c:	90430000 	lbu	v1,0(v0)
800023a0:	1064fff9 	beq	v1,a0,80002388 <core_bench_state+0xc8>
800023a4:	00771826 	xor	v1,v1,s7
            *p ^= (ee_u8)seed1;
800023a8:	1000fff7 	b	80002388 <core_bench_state+0xc8>
800023ac:	a0430000 	sb	v1,0(v0)
    p = memblock;
800023b0:	afb00060 	sw	s0,96(sp)
    while (*p != 0)
800023b4:	92020000 	lbu	v0,0(s0)
800023b8:	10400010 	beqz	v0,800023fc <core_bench_state+0x13c>
800023bc:	27a50040 	addiu	a1,sp,64
        enum CORE_STATE fstate = core_state_transition(&p, track_counts);
800023c0:	0c000818 	jal	80002060 <core_state_transition>
800023c4:	27a40060 	addiu	a0,sp,96
        final_counts[fstate]++;
800023c8:	00021880 	sll	v1,v0,0x2
800023cc:	27a20020 	addiu	v0,sp,32
800023d0:	00431821 	addu	v1,v0,v1
800023d4:	8c620000 	lw	v0,0(v1)
800023d8:	24420001 	addiu	v0,v0,1
800023dc:	ac620000 	sw	v0,0(v1)
    while (*p != 0)
800023e0:	8fa20060 	lw	v0,96(sp)
800023e4:	90420000 	lbu	v0,0(v0)
800023e8:	1440fff5 	bnez	v0,800023c0 <core_bench_state+0x100>
800023ec:	27a50040 	addiu	a1,sp,64
    while (p < (memblock + blksize))
800023f0:	0214102b 	sltu	v0,s0,s4
800023f4:	10400010 	beqz	v0,80002438 <core_bench_state+0x178>
800023f8:	afb00060 	sw	s0,96(sp)
        if (*p != ',')
800023fc:	10000006 	b	80002418 <core_bench_state+0x158>
80002400:	2403002c 	li	v1,44
        p += step;
80002404:	8fb00060 	lw	s0,96(sp)
80002408:	02158021 	addu	s0,s0,s5
    while (p < (memblock + blksize))
8000240c:	0214102b 	sltu	v0,s0,s4
80002410:	10400009 	beqz	v0,80002438 <core_bench_state+0x178>
80002414:	afb00060 	sw	s0,96(sp)
        if (*p != ',')
80002418:	92020000 	lbu	v0,0(s0)
8000241c:	1043fff9 	beq	v0,v1,80002404 <core_bench_state+0x144>
80002420:	00561026 	xor	v0,v0,s6
            *p ^= (ee_u8)seed2;
80002424:	1000fff7 	b	80002404 <core_bench_state+0x144>
80002428:	a2020000 	sb	v0,0(s0)
    while (*p != 0)
8000242c:	92020000 	lbu	v0,0(s0)
80002430:	1440ffe3 	bnez	v0,800023c0 <core_bench_state+0x100>
80002434:	27a50040 	addiu	a1,sp,64
    for (i = 0; i < NUM_CORE_STATES; i++)
80002438:	26300020 	addiu	s0,s1,32
        crc = crcu32(final_counts[i], crc);
8000243c:	02402825 	move	a1,s2
80002440:	0c00095c 	jal	80002570 <crcu32>
80002444:	8e240000 	lw	a0,0(s1)
        crc = crcu32(track_counts[i], crc);
80002448:	00402825 	move	a1,v0
8000244c:	0c00095c 	jal	80002570 <crcu32>
80002450:	8e640000 	lw	a0,0(s3)
80002454:	00409025 	move	s2,v0
    for (i = 0; i < NUM_CORE_STATES; i++)
80002458:	26310004 	addiu	s1,s1,4
8000245c:	1630fff7 	bne	s1,s0,8000243c <core_bench_state+0x17c>
80002460:	26730004 	addiu	s3,s3,4
}
80002464:	dfbf00a8 	ld	ra,168(sp)
80002468:	dfb700a0 	ld	s7,160(sp)
8000246c:	dfb60098 	ld	s6,152(sp)
80002470:	dfb50090 	ld	s5,144(sp)
80002474:	dfb40088 	ld	s4,136(sp)
80002478:	dfb30080 	ld	s3,128(sp)
8000247c:	dfb20078 	ld	s2,120(sp)
80002480:	dfb10070 	ld	s1,112(sp)
80002484:	dfb00068 	ld	s0,104(sp)
80002488:	03e00008 	jr	ra
8000248c:	27bd00b0 	addiu	sp,sp,176

80002490 <get_seed_32>:
extern volatile ee_s32 seed5_volatile;
ee_s32
get_seed_32(int i)
{
    ee_s32 retval;
    switch (i)
80002490:	2c820006 	sltiu	v0,a0,6
80002494:	10400016 	beqz	v0,800024f0 <get_seed_32+0x60>
80002498:	00042080 	sll	a0,a0,0x2
8000249c:	3c028002 	lui	v0,0x8002
800024a0:	2442a400 	addiu	v0,v0,-23552
800024a4:	00442021 	addu	a0,v0,a0
800024a8:	8c820000 	lw	v0,0(a0)
800024ac:	00400008 	jr	v0
800024b0:	00000000 	nop
    {
        case 1:
            retval = seed1_volatile;
800024b4:	8f8280f0 	lw	v0,-32528(gp)
            break;
800024b8:	03e00008 	jr	ra
800024bc:	00000000 	nop
        case 2:
            retval = seed2_volatile;
800024c0:	8f8280ec 	lw	v0,-32532(gp)
            break;
800024c4:	03e00008 	jr	ra
800024c8:	00000000 	nop
        case 3:
            retval = seed3_volatile;
800024cc:	8f828018 	lw	v0,-32744(gp)
            break;
800024d0:	03e00008 	jr	ra
800024d4:	00000000 	nop
        case 4:
            retval = seed4_volatile;
800024d8:	8f8280e8 	lw	v0,-32536(gp)
            break;
800024dc:	03e00008 	jr	ra
800024e0:	00000000 	nop
        case 5:
            retval = seed5_volatile;
800024e4:	8f8280e4 	lw	v0,-32540(gp)
            break;
800024e8:	03e00008 	jr	ra
800024ec:	00000000 	nop
        default:
            retval = 0;
            break;
    }
    return retval;
}
800024f0:	03e00008 	jr	ra
800024f4:	00001025 	move	v0,zero

800024f8 <crcu8>:
        Service functions to calculate 16b CRC code.

*/
ee_u16
crcu8(ee_u8 data, ee_u16 crc)
{
800024f8:	00a01025 	move	v0,a1
800024fc:	10000008 	b	80002520 <crcu8+0x28>
80002500:	24030008 	li	v1,8
80002504:	03e00008 	jr	ra
80002508:	00000000 	nop
            crc ^= 0x4002;
            carry = 1;
        }
        else
            carry = 0;
        crc >>= 1;
8000250c:	00021042 	srl	v0,v0,0x1
    for (i = 0; i < 8; i++)
80002510:	2463ffff 	addiu	v1,v1,-1
80002514:	306300ff 	andi	v1,v1,0xff
80002518:	1060fffa 	beqz	v1,80002504 <crcu8+0xc>
8000251c:	00000000 	nop
        x16 = (ee_u8)((data & 1) ^ ((ee_u8)crc & 1));
80002520:	00442826 	xor	a1,v0,a0
80002524:	30a50001 	andi	a1,a1,0x1
        if (x16 == 1)
80002528:	10a0fff8 	beqz	a1,8000250c <crcu8+0x14>
8000252c:	00042042 	srl	a0,a0,0x1
        crc >>= 1;
80002530:	00021042 	srl	v0,v0,0x1
        if (carry)
            crc |= 0x8000;
80002534:	1000fff6 	b	80002510 <crcu8+0x18>
80002538:	3842a001 	xori	v0,v0,0xa001

8000253c <crcu16>:
    }
    return crc;
}
ee_u16
crcu16(ee_u16 newval, ee_u16 crc)
{
8000253c:	27bdffd0 	addiu	sp,sp,-48
80002540:	ffbf0028 	sd	ra,40(sp)
80002544:	ffb00020 	sd	s0,32(sp)
80002548:	00808025 	move	s0,a0
    crc = crcu8((ee_u8)(newval), crc);
8000254c:	0c00093e 	jal	800024f8 <crcu8>
80002550:	308400ff 	andi	a0,a0,0xff
    crc = crcu8((ee_u8)((newval) >> 8), crc);
80002554:	00402825 	move	a1,v0
80002558:	0c00093e 	jal	800024f8 <crcu8>
8000255c:	00102202 	srl	a0,s0,0x8
    return crc;
}
80002560:	dfbf0028 	ld	ra,40(sp)
80002564:	dfb00020 	ld	s0,32(sp)
80002568:	03e00008 	jr	ra
8000256c:	27bd0030 	addiu	sp,sp,48

80002570 <crcu32>:
ee_u16
crcu32(ee_u32 newval, ee_u16 crc)
{
80002570:	27bdffd0 	addiu	sp,sp,-48
80002574:	ffbf0028 	sd	ra,40(sp)
80002578:	ffb00020 	sd	s0,32(sp)
8000257c:	00808025 	move	s0,a0
    return crc;
}
ee_u16
crc16(ee_s16 newval, ee_u16 crc)
{
    return crcu16((ee_u16)newval, crc);
80002580:	0c00094f 	jal	8000253c <crcu16>
80002584:	3084ffff 	andi	a0,a0,0xffff
80002588:	00402825 	move	a1,v0
8000258c:	0c00094f 	jal	8000253c <crcu16>
80002590:	00102402 	srl	a0,s0,0x10
}
80002594:	dfbf0028 	ld	ra,40(sp)
80002598:	dfb00020 	ld	s0,32(sp)
8000259c:	03e00008 	jr	ra
800025a0:	27bd0030 	addiu	sp,sp,48

800025a4 <crc16>:
{
800025a4:	27bdffd8 	addiu	sp,sp,-40
800025a8:	ffbf0020 	sd	ra,32(sp)
    return crcu16((ee_u16)newval, crc);
800025ac:	0c00094f 	jal	8000253c <crcu16>
800025b0:	3084ffff 	andi	a0,a0,0xffff
}
800025b4:	dfbf0020 	ld	ra,32(sp)
800025b8:	03e00008 	jr	ra
800025bc:	27bd0028 	addiu	sp,sp,40

800025c0 <check_data_types>:
    if (retval > 0)
    {
        ee_printf("ERROR: Please modify the datatypes in core_portme.h!\n");
    }
    return retval;
}
800025c0:	03e00008 	jr	ra
800025c4:	00001025 	move	v0,zero

800025c8 <inthandler>:
800025c8:	ff828110 	sd	v0,-32496(gp)
800025cc:	40027000 	mfc0	v0,c0_epc
800025d0:	279b8108 	addiu	k1,gp,-32504
800025d4:	ff610000 	sd	at,0(k1)
800025d8:	401b6000 	mfc0	k1,c0_sr
800025dc:	27818200 	addiu	at,gp,-32256
800025e0:	ac3b0000 	sw	k1,0(at)
800025e4:	2401fffe 	li	at,-2
800025e8:	0361d824 	and	k1,k1,at
800025ec:	409b6000 	mtc0	k1,c0_sr
800025f0:	af828208 	sw	v0,-32248(gp)
800025f4:	ff838118 	sd	v1,-32488(gp)
800025f8:	ff848120 	sd	a0,-32480(gp)
800025fc:	ff858128 	sd	a1,-32472(gp)
80002600:	ff868130 	sd	a2,-32464(gp)
80002604:	ff878138 	sd	a3,-32456(gp)
80002608:	ff888140 	sd	t0,-32448(gp)
8000260c:	ff898148 	sd	t1,-32440(gp)
80002610:	ff8a8150 	sd	t2,-32432(gp)
80002614:	ff8b8158 	sd	t3,-32424(gp)
80002618:	ff8c8160 	sd	t4,-32416(gp)
8000261c:	ff8d8168 	sd	t5,-32408(gp)
80002620:	ff8e8170 	sd	t6,-32400(gp)
80002624:	ff8f8178 	sd	t7,-32392(gp)
80002628:	ff908180 	sd	s0,-32384(gp)
8000262c:	ff918188 	sd	s1,-32376(gp)
80002630:	ff928190 	sd	s2,-32368(gp)
80002634:	ff938198 	sd	s3,-32360(gp)
80002638:	ff9481a0 	sd	s4,-32352(gp)
8000263c:	ff9581a8 	sd	s5,-32344(gp)
80002640:	ff9681b0 	sd	s6,-32336(gp)
80002644:	ff9781b8 	sd	s7,-32328(gp)
80002648:	ff9881c0 	sd	t8,-32320(gp)
8000264c:	ff9981c8 	sd	t9,-32312(gp)
80002650:	ff9c81e0 	sd	gp,-32288(gp)
80002654:	ff9d81e8 	sd	sp,-32280(gp)
80002658:	ff9e81f0 	sd	s8,-32272(gp)
8000265c:	ff9f81f8 	sd	ra,-32264(gp)
80002660:	0000f012 	mflo	s8
80002664:	ff9e8218 	sd	s8,-32232(gp)
80002668:	0000f010 	mfhi	s8
8000266c:	ff9e8210 	sd	s8,-32240(gp)
80002670:	445ef800 	cfc1	s8,$31
80002674:	af9e8220 	sw	s8,-32224(gp)
80002678:	f7808228 	sdc1	$f0,-32216(gp)
8000267c:	f7818230 	sdc1	$f1,-32208(gp)
80002680:	f7828238 	sdc1	$f2,-32200(gp)
80002684:	f7838240 	sdc1	$f3,-32192(gp)
80002688:	f7848248 	sdc1	$f4,-32184(gp)
8000268c:	f7858250 	sdc1	$f5,-32176(gp)
80002690:	f7868258 	sdc1	$f6,-32168(gp)
80002694:	f7878260 	sdc1	$f7,-32160(gp)
80002698:	f7888268 	sdc1	$f8,-32152(gp)
8000269c:	f7898270 	sdc1	$f9,-32144(gp)
800026a0:	f78a8278 	sdc1	$f10,-32136(gp)
800026a4:	f78b8280 	sdc1	$f11,-32128(gp)
800026a8:	f78c8288 	sdc1	$f12,-32120(gp)
800026ac:	f78d8290 	sdc1	$f13,-32112(gp)
800026b0:	f78e8298 	sdc1	$f14,-32104(gp)
800026b4:	f78f82a0 	sdc1	$f15,-32096(gp)
800026b8:	f79082a8 	sdc1	$f16,-32088(gp)
800026bc:	f79182b0 	sdc1	$f17,-32080(gp)
800026c0:	f79282b8 	sdc1	$f18,-32072(gp)
800026c4:	f79382c0 	sdc1	$f19,-32064(gp)
800026c8:	f79482c8 	sdc1	$f20,-32056(gp)
800026cc:	f79582d0 	sdc1	$f21,-32048(gp)
800026d0:	f79682d8 	sdc1	$f22,-32040(gp)
800026d4:	f79782e0 	sdc1	$f23,-32032(gp)
800026d8:	f79882e8 	sdc1	$f24,-32024(gp)
800026dc:	f79982f0 	sdc1	$f25,-32016(gp)
800026e0:	f79a82f8 	sdc1	$f26,-32008(gp)
800026e4:	f79b8300 	sdc1	$f27,-32000(gp)
800026e8:	f79c8308 	sdc1	$f28,-31992(gp)
800026ec:	f79d8310 	sdc1	$f29,-31984(gp)
800026f0:	f79e8318 	sdc1	$f30,-31976(gp)
800026f4:	f79f8320 	sdc1	$f31,-31968(gp)
800026f8:	3c1d8003 	lui	sp,0x8003
800026fc:	27bddcf8 	addiu	sp,sp,-8968
80002700:	401b6800 	mfc0	k1,c0_cause
80002704:	af9b8204 	sw	k1,-32252(gp)
80002708:	337e00ff 	andi	s8,k1,0xff
8000270c:	13c00008 	beqz	s8,80002730 <justaninterrupt>
80002710:	00000000 	nop
80002714:	00000000 	nop
80002718:	0c0066e0 	jal	80019b80 <__onCriticalException>
8000271c:	00000000 	nop
80002720:	00000000 	nop
80002724:	080009e5 	j	80002794 <endint>
80002728:	00000000 	nop
8000272c:	00000000 	nop

80002730 <justaninterrupt>:
80002730:	337e1000 	andi	s8,k1,0x1000
80002734:	13c00008 	beqz	s8,80002758 <notprenmi>
80002738:	00000000 	nop
8000273c:	00000000 	nop
80002740:	0c0066f8 	jal	80019be0 <__onResetException>
80002744:	00000000 	nop
80002748:	00000000 	nop
8000274c:	080009e5 	j	80002794 <endint>
80002750:	00000000 	nop
80002754:	00000000 	nop

80002758 <notprenmi>:
80002758:	337e8000 	andi	s8,k1,0x8000
8000275c:	13c0000a 	beqz	s8,80002788 <notcount>
80002760:	00000000 	nop
80002764:	00000000 	nop
80002768:	401a5800 	mfc0	k0,c0_compare
8000276c:	409a5800 	mtc0	k0,c0_compare
80002770:	0c005e44 	jal	80017910 <__TI_handler>
80002774:	00000000 	nop
80002778:	00000000 	nop
8000277c:	080009e5 	j	80002794 <endint>
80002780:	00000000 	nop
80002784:	00000000 	nop

80002788 <notcount>:
80002788:	0c005dbc 	jal	800176f0 <__MI_handler>
8000278c:	00000000 	nop
80002790:	00000000 	nop

80002794 <endint>:
80002794:	df828110 	ld	v0,-32496(gp)
80002798:	df838118 	ld	v1,-32488(gp)
8000279c:	df848120 	ld	a0,-32480(gp)
800027a0:	df858128 	ld	a1,-32472(gp)
800027a4:	df868130 	ld	a2,-32464(gp)
800027a8:	df878138 	ld	a3,-32456(gp)
800027ac:	df888140 	ld	t0,-32448(gp)
800027b0:	df898148 	ld	t1,-32440(gp)
800027b4:	df8a8150 	ld	t2,-32432(gp)
800027b8:	df8b8158 	ld	t3,-32424(gp)
800027bc:	df8c8160 	ld	t4,-32416(gp)
800027c0:	df8d8168 	ld	t5,-32408(gp)
800027c4:	df8e8170 	ld	t6,-32400(gp)
800027c8:	df8f8178 	ld	t7,-32392(gp)
800027cc:	df908180 	ld	s0,-32384(gp)
800027d0:	df918188 	ld	s1,-32376(gp)
800027d4:	df928190 	ld	s2,-32368(gp)
800027d8:	df938198 	ld	s3,-32360(gp)
800027dc:	df9481a0 	ld	s4,-32352(gp)
800027e0:	df9581a8 	ld	s5,-32344(gp)
800027e4:	df9681b0 	ld	s6,-32336(gp)
800027e8:	df9781b8 	ld	s7,-32328(gp)
800027ec:	df9881c0 	ld	t8,-32320(gp)
800027f0:	df9981c8 	ld	t9,-32312(gp)
800027f4:	df9c81e0 	ld	gp,-32288(gp)
800027f8:	df9d81e8 	ld	sp,-32280(gp)
800027fc:	df9f81f8 	ld	ra,-32264(gp)
80002800:	8f9e8208 	lw	s8,-32248(gp)
80002804:	409e7000 	mtc0	s8,c0_epc
80002808:	8f9e8200 	lw	s8,-32256(gp)
8000280c:	409e6000 	mtc0	s8,c0_sr
80002810:	df9e8218 	ld	s8,-32232(gp)
80002814:	03c00013 	mtlo	s8
80002818:	df9e8210 	ld	s8,-32240(gp)
8000281c:	03c00011 	mthi	s8
80002820:	d7808228 	ldc1	$f0,-32216(gp)
80002824:	d7818230 	ldc1	$f1,-32208(gp)
80002828:	d7828238 	ldc1	$f2,-32200(gp)
8000282c:	d7838240 	ldc1	$f3,-32192(gp)
80002830:	d7848248 	ldc1	$f4,-32184(gp)
80002834:	d7858250 	ldc1	$f5,-32176(gp)
80002838:	d7868258 	ldc1	$f6,-32168(gp)
8000283c:	d7878260 	ldc1	$f7,-32160(gp)
80002840:	d7888268 	ldc1	$f8,-32152(gp)
80002844:	d7898270 	ldc1	$f9,-32144(gp)
80002848:	d78a8278 	ldc1	$f10,-32136(gp)
8000284c:	d78b8280 	ldc1	$f11,-32128(gp)
80002850:	d78c8288 	ldc1	$f12,-32120(gp)
80002854:	d78d8290 	ldc1	$f13,-32112(gp)
80002858:	d78e8298 	ldc1	$f14,-32104(gp)
8000285c:	d78f82a0 	ldc1	$f15,-32096(gp)
80002860:	d79082a8 	ldc1	$f16,-32088(gp)
80002864:	d79182b0 	ldc1	$f17,-32080(gp)
80002868:	d79282b8 	ldc1	$f18,-32072(gp)
8000286c:	d79382c0 	ldc1	$f19,-32064(gp)
80002870:	d79482c8 	ldc1	$f20,-32056(gp)
80002874:	d79582d0 	ldc1	$f21,-32048(gp)
80002878:	d79682d8 	ldc1	$f22,-32040(gp)
8000287c:	d79782e0 	ldc1	$f23,-32032(gp)
80002880:	d79882e8 	ldc1	$f24,-32024(gp)
80002884:	d79982f0 	ldc1	$f25,-32016(gp)
80002888:	d79a82f8 	ldc1	$f26,-32008(gp)
8000288c:	d79b8300 	ldc1	$f27,-32000(gp)
80002890:	d79c8308 	ldc1	$f28,-31992(gp)
80002894:	d79d8310 	ldc1	$f29,-31984(gp)
80002898:	d79e8318 	ldc1	$f30,-31976(gp)
8000289c:	8f9e8220 	lw	s8,-32224(gp)
800028a0:	d79f8320 	ldc1	$f31,-31968(gp)
800028a4:	44def800 	ctc1	s8,$31
800028a8:	df9e81f0 	ld	s8,-32272(gp)
800028ac:	27818108 	addiu	at,gp,-32504
800028b0:	dc210000 	ld	at,0(at)
800028b4:	42000018 	eret
800028b8:	00000000 	nop

800028bc <abort>:
800028bc:	27bdffd0 	addiu	sp,sp,-48
800028c0:	ffbf0028 	sd	ra,40(sp)
800028c4:	ffbe0020 	sd	s8,32(sp)
800028c8:	03a0f025 	move	s8,sp
800028cc:	24040006 	li	a0,6
800028d0:	0c001e09 	jal	80007824 <raise>
800028d4:	00000000 	nop
800028d8:	24040001 	li	a0,1
800028dc:	0c00675a 	jal	80019d68 <_exit>
800028e0:	00000000 	nop

800028e4 <clock>:
800028e4:	27bdffb8 	addiu	sp,sp,-72
800028e8:	ffbf0040 	sd	ra,64(sp)
800028ec:	ffbe0038 	sd	s8,56(sp)
800028f0:	03a0f025 	move	s8,sp
800028f4:	8f828028 	lw	v0,-32728(gp)
800028f8:	27c30028 	addiu	v1,s8,40
800028fc:	00602825 	move	a1,v1
80002900:	00402025 	move	a0,v0
80002904:	0c001f23 	jal	80007c8c <_times_r>
80002908:	00000000 	nop
8000290c:	afc20020 	sw	v0,32(s8)
80002910:	8fc30020 	lw	v1,32(s8)
80002914:	2402ffff 	li	v0,-1
80002918:	10620009 	beq	v1,v0,80002940 <clock+0x5c>
8000291c:	00000000 	nop
80002920:	8fc30028 	lw	v1,40(s8)
80002924:	8fc2002c 	lw	v0,44(s8)
80002928:	00621821 	addu	v1,v1,v0
8000292c:	8fc20030 	lw	v0,48(s8)
80002930:	00621821 	addu	v1,v1,v0
80002934:	8fc20034 	lw	v0,52(s8)
80002938:	00621021 	addu	v0,v1,v0
8000293c:	afc20020 	sw	v0,32(s8)
80002940:	8fc20020 	lw	v0,32(s8)
80002944:	03c0e825 	move	sp,s8
80002948:	dfbf0040 	ld	ra,64(sp)
8000294c:	dfbe0038 	ld	s8,56(sp)
80002950:	27bd0048 	addiu	sp,sp,72
80002954:	03e00008 	jr	ra
80002958:	00000000 	nop

8000295c <__sflush_r>:
8000295c:	27bdffa0 	addiu	sp,sp,-96
80002960:	ffbf0058 	sd	ra,88(sp)
80002964:	ffbe0050 	sd	s8,80(sp)
80002968:	ffb30048 	sd	s3,72(sp)
8000296c:	ffb20040 	sd	s2,64(sp)
80002970:	ffb10038 	sd	s1,56(sp)
80002974:	ffb00030 	sd	s0,48(sp)
80002978:	03a0f025 	move	s8,sp
8000297c:	afc40060 	sw	a0,96(s8)
80002980:	00a08025 	move	s0,a1
80002984:	9602000c 	lhu	v0,12(s0)
80002988:	a7c20028 	sh	v0,40(s8)
8000298c:	97c20028 	lhu	v0,40(s8)
80002990:	30420008 	andi	v0,v0,0x8
80002994:	144000a8 	bnez	v0,80002c38 <__sflush_r+0x2dc>
80002998:	00000000 	nop
8000299c:	8602000c 	lh	v0,12(s0)
800029a0:	34420800 	ori	v0,v0,0x800
800029a4:	00021400 	sll	v0,v0,0x10
800029a8:	00021403 	sra	v0,v0,0x10
800029ac:	a602000c 	sh	v0,12(s0)
800029b0:	8e020004 	lw	v0,4(s0)
800029b4:	1c400004 	bgtz	v0,800029c8 <__sflush_r+0x6c>
800029b8:	00000000 	nop
800029bc:	8e02003c 	lw	v0,60(s0)
800029c0:	1840009a 	blez	v0,80002c2c <__sflush_r+0x2d0>
800029c4:	00000000 	nop
800029c8:	8e020028 	lw	v0,40(s0)
800029cc:	10400097 	beqz	v0,80002c2c <__sflush_r+0x2d0>
800029d0:	00000000 	nop
800029d4:	8fc20060 	lw	v0,96(s8)
800029d8:	8c420000 	lw	v0,0(v0)
800029dc:	afc2002c 	sw	v0,44(s8)
800029e0:	8fc20060 	lw	v0,96(s8)
800029e4:	ac400000 	sw	zero,0(v0)
800029e8:	8602000c 	lh	v0,12(s0)
800029ec:	3042ffff 	andi	v0,v0,0xffff
800029f0:	30421000 	andi	v0,v0,0x1000
800029f4:	10400005 	beqz	v0,80002a0c <__sflush_r+0xb0>
800029f8:	00000000 	nop
800029fc:	8e020050 	lw	v0,80(s0)
80002a00:	afc20020 	sw	v0,32(s8)
80002a04:	1000002c 	b	80002ab8 <__sflush_r+0x15c>
80002a08:	00000000 	nop
80002a0c:	8e020028 	lw	v0,40(s0)
80002a10:	8e03001c 	lw	v1,28(s0)
80002a14:	24070001 	li	a3,1
80002a18:	00003025 	move	a2,zero
80002a1c:	00602825 	move	a1,v1
80002a20:	8fc40060 	lw	a0,96(s8)
80002a24:	0040f809 	jalr	v0
80002a28:	00000000 	nop
80002a2c:	afc20020 	sw	v0,32(s8)
80002a30:	8fc30020 	lw	v1,32(s8)
80002a34:	2402ffff 	li	v0,-1
80002a38:	1462001f 	bne	v1,v0,80002ab8 <__sflush_r+0x15c>
80002a3c:	00000000 	nop
80002a40:	8fc20060 	lw	v0,96(s8)
80002a44:	8c420000 	lw	v0,0(v0)
80002a48:	1040001b 	beqz	v0,80002ab8 <__sflush_r+0x15c>
80002a4c:	00000000 	nop
80002a50:	2402ffff 	li	v0,-1
80002a54:	afc20024 	sw	v0,36(s8)
80002a58:	8fc20060 	lw	v0,96(s8)
80002a5c:	8c430000 	lw	v1,0(v0)
80002a60:	2402001d 	li	v0,29
80002a64:	10620006 	beq	v1,v0,80002a80 <__sflush_r+0x124>
80002a68:	00000000 	nop
80002a6c:	8fc20060 	lw	v0,96(s8)
80002a70:	8c430000 	lw	v1,0(v0)
80002a74:	24020016 	li	v0,22
80002a78:	14620007 	bne	v1,v0,80002a98 <__sflush_r+0x13c>
80002a7c:	00000000 	nop
80002a80:	afc00024 	sw	zero,36(s8)
80002a84:	8fc20060 	lw	v0,96(s8)
80002a88:	8fc3002c 	lw	v1,44(s8)
80002a8c:	ac430000 	sw	v1,0(v0)
80002a90:	10000006 	b	80002aac <__sflush_r+0x150>
80002a94:	00000000 	nop
80002a98:	8602000c 	lh	v0,12(s0)
80002a9c:	34420040 	ori	v0,v0,0x40
80002aa0:	00021400 	sll	v0,v0,0x10
80002aa4:	00021403 	sra	v0,v0,0x10
80002aa8:	a602000c 	sh	v0,12(s0)
80002aac:	8fc20024 	lw	v0,36(s8)
80002ab0:	1000008e 	b	80002cec <__sflush_r+0x390>
80002ab4:	00000000 	nop
80002ab8:	8602000c 	lh	v0,12(s0)
80002abc:	3042ffff 	andi	v0,v0,0xffff
80002ac0:	30420004 	andi	v0,v0,0x4
80002ac4:	1040000c 	beqz	v0,80002af8 <__sflush_r+0x19c>
80002ac8:	00000000 	nop
80002acc:	8e020004 	lw	v0,4(s0)
80002ad0:	8fc30020 	lw	v1,32(s8)
80002ad4:	00621023 	subu	v0,v1,v0
80002ad8:	afc20020 	sw	v0,32(s8)
80002adc:	8e020030 	lw	v0,48(s0)
80002ae0:	10400005 	beqz	v0,80002af8 <__sflush_r+0x19c>
80002ae4:	00000000 	nop
80002ae8:	8e02003c 	lw	v0,60(s0)
80002aec:	8fc30020 	lw	v1,32(s8)
80002af0:	00621023 	subu	v0,v1,v0
80002af4:	afc20020 	sw	v0,32(s8)
80002af8:	8e020028 	lw	v0,40(s0)
80002afc:	8e03001c 	lw	v1,28(s0)
80002b00:	00003825 	move	a3,zero
80002b04:	8fc60020 	lw	a2,32(s8)
80002b08:	00602825 	move	a1,v1
80002b0c:	8fc40060 	lw	a0,96(s8)
80002b10:	0040f809 	jalr	v0
80002b14:	00000000 	nop
80002b18:	afc20020 	sw	v0,32(s8)
80002b1c:	8fc30020 	lw	v1,32(s8)
80002b20:	2402ffff 	li	v0,-1
80002b24:	1462000f 	bne	v1,v0,80002b64 <__sflush_r+0x208>
80002b28:	00000000 	nop
80002b2c:	8fc20060 	lw	v0,96(s8)
80002b30:	8c420000 	lw	v0,0(v0)
80002b34:	1040000b 	beqz	v0,80002b64 <__sflush_r+0x208>
80002b38:	00000000 	nop
80002b3c:	8fc20060 	lw	v0,96(s8)
80002b40:	8c430000 	lw	v1,0(v0)
80002b44:	2402001d 	li	v0,29
80002b48:	10620006 	beq	v1,v0,80002b64 <__sflush_r+0x208>
80002b4c:	00000000 	nop
80002b50:	8fc20060 	lw	v0,96(s8)
80002b54:	8c430000 	lw	v1,0(v0)
80002b58:	24020016 	li	v0,22
80002b5c:	1462002b 	bne	v1,v0,80002c0c <__sflush_r+0x2b0>
80002b60:	00000000 	nop
80002b64:	8603000c 	lh	v1,12(s0)
80002b68:	2402f7ff 	li	v0,-2049
80002b6c:	00621024 	and	v0,v1,v0
80002b70:	00021400 	sll	v0,v0,0x10
80002b74:	00021403 	sra	v0,v0,0x10
80002b78:	a602000c 	sh	v0,12(s0)
80002b7c:	ae000004 	sw	zero,4(s0)
80002b80:	8e020010 	lw	v0,16(s0)
80002b84:	ae020000 	sw	v0,0(s0)
80002b88:	8602000c 	lh	v0,12(s0)
80002b8c:	3042ffff 	andi	v0,v0,0xffff
80002b90:	30421000 	andi	v0,v0,0x1000
80002b94:	1040000b 	beqz	v0,80002bc4 <__sflush_r+0x268>
80002b98:	00000000 	nop
80002b9c:	8fc30020 	lw	v1,32(s8)
80002ba0:	2402ffff 	li	v0,-1
80002ba4:	14620005 	bne	v1,v0,80002bbc <__sflush_r+0x260>
80002ba8:	00000000 	nop
80002bac:	8fc20060 	lw	v0,96(s8)
80002bb0:	8c420000 	lw	v0,0(v0)
80002bb4:	14400003 	bnez	v0,80002bc4 <__sflush_r+0x268>
80002bb8:	00000000 	nop
80002bbc:	8fc20020 	lw	v0,32(s8)
80002bc0:	ae020050 	sw	v0,80(s0)
80002bc4:	8fc20060 	lw	v0,96(s8)
80002bc8:	8fc3002c 	lw	v1,44(s8)
80002bcc:	ac430000 	sw	v1,0(v0)
80002bd0:	8e020030 	lw	v0,48(s0)
80002bd4:	10400015 	beqz	v0,80002c2c <__sflush_r+0x2d0>
80002bd8:	00000000 	nop
80002bdc:	8e030030 	lw	v1,48(s0)
80002be0:	26020040 	addiu	v0,s0,64
80002be4:	10620006 	beq	v1,v0,80002c00 <__sflush_r+0x2a4>
80002be8:	00000000 	nop
80002bec:	8e020030 	lw	v0,48(s0)
80002bf0:	00402825 	move	a1,v0
80002bf4:	8fc40060 	lw	a0,96(s8)
80002bf8:	0c000dcb 	jal	8000372c <_free_r>
80002bfc:	00000000 	nop
80002c00:	ae000030 	sw	zero,48(s0)
80002c04:	10000009 	b	80002c2c <__sflush_r+0x2d0>
80002c08:	00000000 	nop
80002c0c:	8602000c 	lh	v0,12(s0)
80002c10:	34420040 	ori	v0,v0,0x40
80002c14:	00021400 	sll	v0,v0,0x10
80002c18:	00021403 	sra	v0,v0,0x10
80002c1c:	a602000c 	sh	v0,12(s0)
80002c20:	2402ffff 	li	v0,-1
80002c24:	10000031 	b	80002cec <__sflush_r+0x390>
80002c28:	00000000 	nop
80002c2c:	00001025 	move	v0,zero
80002c30:	1000002e 	b	80002cec <__sflush_r+0x390>
80002c34:	00000000 	nop
80002c38:	8e110010 	lw	s1,16(s0)
80002c3c:	16200004 	bnez	s1,80002c50 <__sflush_r+0x2f4>
80002c40:	00000000 	nop
80002c44:	00001025 	move	v0,zero
80002c48:	10000028 	b	80002cec <__sflush_r+0x390>
80002c4c:	00000000 	nop
80002c50:	8e020000 	lw	v0,0(s0)
80002c54:	00519023 	subu	s2,v0,s1
80002c58:	ae110000 	sw	s1,0(s0)
80002c5c:	97c20028 	lhu	v0,40(s8)
80002c60:	30420003 	andi	v0,v0,0x3
80002c64:	14400004 	bnez	v0,80002c78 <__sflush_r+0x31c>
80002c68:	00000000 	nop
80002c6c:	8e020014 	lw	v0,20(s0)
80002c70:	10000002 	b	80002c7c <__sflush_r+0x320>
80002c74:	00000000 	nop
80002c78:	00001025 	move	v0,zero
80002c7c:	ae020008 	sw	v0,8(s0)
80002c80:	10000017 	b	80002ce0 <__sflush_r+0x384>
80002c84:	00000000 	nop
80002c88:	8e020024 	lw	v0,36(s0)
80002c8c:	8e03001c 	lw	v1,28(s0)
80002c90:	02403825 	move	a3,s2
80002c94:	02203025 	move	a2,s1
80002c98:	00602825 	move	a1,v1
80002c9c:	8fc40060 	lw	a0,96(s8)
80002ca0:	0040f809 	jalr	v0
80002ca4:	00000000 	nop
80002ca8:	00409825 	move	s3,v0
80002cac:	1e600009 	bgtz	s3,80002cd4 <__sflush_r+0x378>
80002cb0:	00000000 	nop
80002cb4:	8602000c 	lh	v0,12(s0)
80002cb8:	34420040 	ori	v0,v0,0x40
80002cbc:	00021400 	sll	v0,v0,0x10
80002cc0:	00021403 	sra	v0,v0,0x10
80002cc4:	a602000c 	sh	v0,12(s0)
80002cc8:	2402ffff 	li	v0,-1
80002ccc:	10000007 	b	80002cec <__sflush_r+0x390>
80002cd0:	00000000 	nop
80002cd4:	02601025 	move	v0,s3
80002cd8:	02228821 	addu	s1,s1,v0
80002cdc:	02539023 	subu	s2,s2,s3
80002ce0:	1e40ffe9 	bgtz	s2,80002c88 <__sflush_r+0x32c>
80002ce4:	00000000 	nop
80002ce8:	00001025 	move	v0,zero
80002cec:	03c0e825 	move	sp,s8
80002cf0:	dfbf0058 	ld	ra,88(sp)
80002cf4:	dfbe0050 	ld	s8,80(sp)
80002cf8:	dfb30048 	ld	s3,72(sp)
80002cfc:	dfb20040 	ld	s2,64(sp)
80002d00:	dfb10038 	ld	s1,56(sp)
80002d04:	dfb00030 	ld	s0,48(sp)
80002d08:	27bd0060 	addiu	sp,sp,96
80002d0c:	03e00008 	jr	ra
80002d10:	00000000 	nop

80002d14 <_fflush_r>:
80002d14:	27bdffc0 	addiu	sp,sp,-64
80002d18:	ffbf0038 	sd	ra,56(sp)
80002d1c:	ffbe0030 	sd	s8,48(sp)
80002d20:	ffb00028 	sd	s0,40(sp)
80002d24:	03a0f025 	move	s8,sp
80002d28:	afc40040 	sw	a0,64(s8)
80002d2c:	00a08025 	move	s0,a1
80002d30:	8fc20040 	lw	v0,64(s8)
80002d34:	afc20020 	sw	v0,32(s8)
80002d38:	8fc20020 	lw	v0,32(s8)
80002d3c:	10400008 	beqz	v0,80002d60 <_fflush_r+0x4c>
80002d40:	00000000 	nop
80002d44:	8fc20020 	lw	v0,32(s8)
80002d48:	8c420038 	lw	v0,56(v0)
80002d4c:	14400004 	bnez	v0,80002d60 <_fflush_r+0x4c>
80002d50:	00000000 	nop
80002d54:	8fc40020 	lw	a0,32(s8)
80002d58:	0c000cc3 	jal	8000330c <__sinit>
80002d5c:	00000000 	nop
80002d60:	8602000c 	lh	v0,12(s0)
80002d64:	14400004 	bnez	v0,80002d78 <_fflush_r+0x64>
80002d68:	00000000 	nop
80002d6c:	00001025 	move	v0,zero
80002d70:	10000007 	b	80002d90 <_fflush_r+0x7c>
80002d74:	00000000 	nop
80002d78:	02002825 	move	a1,s0
80002d7c:	8fc40040 	lw	a0,64(s8)
80002d80:	0c000a57 	jal	8000295c <__sflush_r>
80002d84:	00000000 	nop
80002d88:	afc20024 	sw	v0,36(s8)
80002d8c:	8fc20024 	lw	v0,36(s8)
80002d90:	03c0e825 	move	sp,s8
80002d94:	dfbf0038 	ld	ra,56(sp)
80002d98:	dfbe0030 	ld	s8,48(sp)
80002d9c:	dfb00028 	ld	s0,40(sp)
80002da0:	27bd0040 	addiu	sp,sp,64
80002da4:	03e00008 	jr	ra
80002da8:	00000000 	nop

80002dac <fflush>:
80002dac:	27bdffd0 	addiu	sp,sp,-48
80002db0:	ffbf0028 	sd	ra,40(sp)
80002db4:	ffbe0020 	sd	s8,32(sp)
80002db8:	03a0f025 	move	s8,sp
80002dbc:	00801025 	move	v0,a0
80002dc0:	14400009 	bnez	v0,80002de8 <fflush+0x3c>
80002dc4:	00000000 	nop
80002dc8:	8f83802c 	lw	v1,-32724(gp)
80002dcc:	3c028000 	lui	v0,0x8000
80002dd0:	24452d14 	addiu	a1,v0,11540
80002dd4:	00602025 	move	a0,v1
80002dd8:	0c001022 	jal	80004088 <_fwalk_reent>
80002ddc:	00000000 	nop
80002de0:	10000006 	b	80002dfc <fflush+0x50>
80002de4:	00000000 	nop
80002de8:	8f838028 	lw	v1,-32728(gp)
80002dec:	00402825 	move	a1,v0
80002df0:	00602025 	move	a0,v1
80002df4:	0c000b45 	jal	80002d14 <_fflush_r>
80002df8:	00000000 	nop
80002dfc:	03c0e825 	move	sp,s8
80002e00:	dfbf0028 	ld	ra,40(sp)
80002e04:	dfbe0020 	ld	s8,32(sp)
80002e08:	27bd0030 	addiu	sp,sp,48
80002e0c:	03e00008 	jr	ra
80002e10:	00000000 	nop

80002e14 <std>:
80002e14:	27bdffd0 	addiu	sp,sp,-48
80002e18:	ffbf0028 	sd	ra,40(sp)
80002e1c:	ffbe0020 	sd	s8,32(sp)
80002e20:	03a0f025 	move	s8,sp
80002e24:	afc40030 	sw	a0,48(s8)
80002e28:	afc50038 	sw	a1,56(s8)
80002e2c:	afc60040 	sw	a2,64(s8)
80002e30:	8fc20030 	lw	v0,48(s8)
80002e34:	ac400000 	sw	zero,0(v0)
80002e38:	8fc20030 	lw	v0,48(s8)
80002e3c:	ac400004 	sw	zero,4(v0)
80002e40:	8fc20030 	lw	v0,48(s8)
80002e44:	ac400008 	sw	zero,8(v0)
80002e48:	8fc20038 	lw	v0,56(s8)
80002e4c:	00021c00 	sll	v1,v0,0x10
80002e50:	00031c03 	sra	v1,v1,0x10
80002e54:	8fc20030 	lw	v0,48(s8)
80002e58:	a443000c 	sh	v1,12(v0)
80002e5c:	8fc20030 	lw	v0,48(s8)
80002e60:	ac400064 	sw	zero,100(v0)
80002e64:	8fc20040 	lw	v0,64(s8)
80002e68:	00021c00 	sll	v1,v0,0x10
80002e6c:	00031c03 	sra	v1,v1,0x10
80002e70:	8fc20030 	lw	v0,48(s8)
80002e74:	a443000e 	sh	v1,14(v0)
80002e78:	8fc20030 	lw	v0,48(s8)
80002e7c:	ac400010 	sw	zero,16(v0)
80002e80:	8fc20030 	lw	v0,48(s8)
80002e84:	ac400014 	sw	zero,20(v0)
80002e88:	8fc20030 	lw	v0,48(s8)
80002e8c:	ac400018 	sw	zero,24(v0)
80002e90:	8fc20030 	lw	v0,48(s8)
80002e94:	2442005c 	addiu	v0,v0,92
80002e98:	24060008 	li	a2,8
80002e9c:	00002825 	move	a1,zero
80002ea0:	00402025 	move	a0,v0
80002ea4:	0c00173d 	jal	80005cf4 <memset>
80002ea8:	00000000 	nop
80002eac:	8fc20030 	lw	v0,48(s8)
80002eb0:	8fc30030 	lw	v1,48(s8)
80002eb4:	ac43001c 	sw	v1,28(v0)
80002eb8:	8fc20030 	lw	v0,48(s8)
80002ebc:	3c038000 	lui	v1,0x8000
80002ec0:	246379d8 	addiu	v1,v1,31192
80002ec4:	ac430020 	sw	v1,32(v0)
80002ec8:	8fc20030 	lw	v0,48(s8)
80002ecc:	3c038000 	lui	v1,0x8000
80002ed0:	24637ab8 	addiu	v1,v1,31416
80002ed4:	ac430024 	sw	v1,36(v0)
80002ed8:	8fc20030 	lw	v0,48(s8)
80002edc:	3c038000 	lui	v1,0x8000
80002ee0:	24637b70 	addiu	v1,v1,31600
80002ee4:	ac430028 	sw	v1,40(v0)
80002ee8:	8fc20030 	lw	v0,48(s8)
80002eec:	3c038000 	lui	v1,0x8000
80002ef0:	24637c24 	addiu	v1,v1,31780
80002ef4:	ac43002c 	sw	v1,44(v0)
80002ef8:	00000000 	nop
80002efc:	03c0e825 	move	sp,s8
80002f00:	dfbf0028 	ld	ra,40(sp)
80002f04:	dfbe0020 	ld	s8,32(sp)
80002f08:	27bd0030 	addiu	sp,sp,48
80002f0c:	03e00008 	jr	ra
80002f10:	00000000 	nop

80002f14 <stdin_init>:
80002f14:	27bdffd0 	addiu	sp,sp,-48
80002f18:	ffbf0028 	sd	ra,40(sp)
80002f1c:	ffbe0020 	sd	s8,32(sp)
80002f20:	03a0f025 	move	s8,sp
80002f24:	afc40030 	sw	a0,48(s8)
80002f28:	00003025 	move	a2,zero
80002f2c:	24050004 	li	a1,4
80002f30:	8fc40030 	lw	a0,48(s8)
80002f34:	0c000b85 	jal	80002e14 <std>
80002f38:	00000000 	nop
80002f3c:	00000000 	nop
80002f40:	03c0e825 	move	sp,s8
80002f44:	dfbf0028 	ld	ra,40(sp)
80002f48:	dfbe0020 	ld	s8,32(sp)
80002f4c:	27bd0030 	addiu	sp,sp,48
80002f50:	03e00008 	jr	ra
80002f54:	00000000 	nop

80002f58 <stdout_init>:
80002f58:	27bdffd0 	addiu	sp,sp,-48
80002f5c:	ffbf0028 	sd	ra,40(sp)
80002f60:	ffbe0020 	sd	s8,32(sp)
80002f64:	03a0f025 	move	s8,sp
80002f68:	afc40030 	sw	a0,48(s8)
80002f6c:	24060001 	li	a2,1
80002f70:	24050009 	li	a1,9
80002f74:	8fc40030 	lw	a0,48(s8)
80002f78:	0c000b85 	jal	80002e14 <std>
80002f7c:	00000000 	nop
80002f80:	00000000 	nop
80002f84:	03c0e825 	move	sp,s8
80002f88:	dfbf0028 	ld	ra,40(sp)
80002f8c:	dfbe0020 	ld	s8,32(sp)
80002f90:	27bd0030 	addiu	sp,sp,48
80002f94:	03e00008 	jr	ra
80002f98:	00000000 	nop

80002f9c <stderr_init>:
80002f9c:	27bdffd0 	addiu	sp,sp,-48
80002fa0:	ffbf0028 	sd	ra,40(sp)
80002fa4:	ffbe0020 	sd	s8,32(sp)
80002fa8:	03a0f025 	move	s8,sp
80002fac:	afc40030 	sw	a0,48(s8)
80002fb0:	24060002 	li	a2,2
80002fb4:	24050012 	li	a1,18
80002fb8:	8fc40030 	lw	a0,48(s8)
80002fbc:	0c000b85 	jal	80002e14 <std>
80002fc0:	00000000 	nop
80002fc4:	00000000 	nop
80002fc8:	03c0e825 	move	sp,s8
80002fcc:	dfbf0028 	ld	ra,40(sp)
80002fd0:	dfbe0020 	ld	s8,32(sp)
80002fd4:	27bd0030 	addiu	sp,sp,48
80002fd8:	03e00008 	jr	ra
80002fdc:	00000000 	nop

80002fe0 <__sfmoreglue>:
80002fe0:	27bdffc0 	addiu	sp,sp,-64
80002fe4:	ffbf0038 	sd	ra,56(sp)
80002fe8:	ffbe0030 	sd	s8,48(sp)
80002fec:	ffb00028 	sd	s0,40(sp)
80002ff0:	03a0f025 	move	s8,sp
80002ff4:	afc40040 	sw	a0,64(s8)
80002ff8:	00a08025 	move	s0,a1
80002ffc:	2602ffff 	addiu	v0,s0,-1
80003000:	00401825 	move	v1,v0
80003004:	00601025 	move	v0,v1
80003008:	00021040 	sll	v0,v0,0x1
8000300c:	00431021 	addu	v0,v0,v1
80003010:	00021080 	sll	v0,v0,0x2
80003014:	00431021 	addu	v0,v0,v1
80003018:	000210c0 	sll	v0,v0,0x3
8000301c:	24420074 	addiu	v0,v0,116
80003020:	00402825 	move	a1,v0
80003024:	8fc40040 	lw	a0,64(s8)
80003028:	0c001296 	jal	80004a58 <_malloc_r>
8000302c:	00000000 	nop
80003030:	afc20020 	sw	v0,32(s8)
80003034:	8fc20020 	lw	v0,32(s8)
80003038:	14400004 	bnez	v0,8000304c <__sfmoreglue+0x6c>
8000303c:	00000000 	nop
80003040:	00001025 	move	v0,zero
80003044:	10000017 	b	800030a4 <__sfmoreglue+0xc4>
80003048:	00000000 	nop
8000304c:	8fc20020 	lw	v0,32(s8)
80003050:	ac400000 	sw	zero,0(v0)
80003054:	8fc20020 	lw	v0,32(s8)
80003058:	ac500004 	sw	s0,4(v0)
8000305c:	8fc20020 	lw	v0,32(s8)
80003060:	2443000c 	addiu	v1,v0,12
80003064:	8fc20020 	lw	v0,32(s8)
80003068:	ac430008 	sw	v1,8(v0)
8000306c:	8fc20020 	lw	v0,32(s8)
80003070:	2444000c 	addiu	a0,v0,12
80003074:	02001825 	move	v1,s0
80003078:	00601025 	move	v0,v1
8000307c:	00021040 	sll	v0,v0,0x1
80003080:	00431021 	addu	v0,v0,v1
80003084:	00021080 	sll	v0,v0,0x2
80003088:	00431021 	addu	v0,v0,v1
8000308c:	000210c0 	sll	v0,v0,0x3
80003090:	00403025 	move	a2,v0
80003094:	00002825 	move	a1,zero
80003098:	0c00173d 	jal	80005cf4 <memset>
8000309c:	00000000 	nop
800030a0:	8fc20020 	lw	v0,32(s8)
800030a4:	03c0e825 	move	sp,s8
800030a8:	dfbf0038 	ld	ra,56(sp)
800030ac:	dfbe0030 	ld	s8,48(sp)
800030b0:	dfb00028 	ld	s0,40(sp)
800030b4:	27bd0040 	addiu	sp,sp,64
800030b8:	03e00008 	jr	ra
800030bc:	00000000 	nop

800030c0 <__sfp>:
800030c0:	27bdffc0 	addiu	sp,sp,-64
800030c4:	ffbf0038 	sd	ra,56(sp)
800030c8:	ffbe0030 	sd	s8,48(sp)
800030cc:	03a0f025 	move	s8,sp
800030d0:	afc40040 	sw	a0,64(s8)
800030d4:	0c000cf9 	jal	800033e4 <__sfp_lock_acquire>
800030d8:	00000000 	nop
800030dc:	8f82802c 	lw	v0,-32724(gp)
800030e0:	8c420038 	lw	v0,56(v0)
800030e4:	14400005 	bnez	v0,800030fc <__sfp+0x3c>
800030e8:	00000000 	nop
800030ec:	8f82802c 	lw	v0,-32724(gp)
800030f0:	00402025 	move	a0,v0
800030f4:	0c000cc3 	jal	8000330c <__sinit>
800030f8:	00000000 	nop
800030fc:	8f82802c 	lw	v0,-32724(gp)
80003100:	244202e0 	addiu	v0,v0,736
80003104:	afc20028 	sw	v0,40(s8)
80003108:	8fc20028 	lw	v0,40(s8)
8000310c:	8c420008 	lw	v0,8(v0)
80003110:	afc20020 	sw	v0,32(s8)
80003114:	8fc20028 	lw	v0,40(s8)
80003118:	8c420004 	lw	v0,4(v0)
8000311c:	afc20024 	sw	v0,36(s8)
80003120:	10000008 	b	80003144 <__sfp+0x84>
80003124:	00000000 	nop
80003128:	8fc20020 	lw	v0,32(s8)
8000312c:	8442000c 	lh	v0,12(v0)
80003130:	10400027 	beqz	v0,800031d0 <__sfp+0x110>
80003134:	00000000 	nop
80003138:	8fc20020 	lw	v0,32(s8)
8000313c:	24420068 	addiu	v0,v0,104
80003140:	afc20020 	sw	v0,32(s8)
80003144:	8fc20024 	lw	v0,36(s8)
80003148:	2442ffff 	addiu	v0,v0,-1
8000314c:	afc20024 	sw	v0,36(s8)
80003150:	8fc20024 	lw	v0,36(s8)
80003154:	0441fff4 	bgez	v0,80003128 <__sfp+0x68>
80003158:	00000000 	nop
8000315c:	8fc20028 	lw	v0,40(s8)
80003160:	8c420000 	lw	v0,0(v0)
80003164:	1440000c 	bnez	v0,80003198 <__sfp+0xd8>
80003168:	00000000 	nop
8000316c:	24050004 	li	a1,4
80003170:	8fc40040 	lw	a0,64(s8)
80003174:	0c000bf8 	jal	80002fe0 <__sfmoreglue>
80003178:	00000000 	nop
8000317c:	00401825 	move	v1,v0
80003180:	8fc20028 	lw	v0,40(s8)
80003184:	ac430000 	sw	v1,0(v0)
80003188:	8fc20028 	lw	v0,40(s8)
8000318c:	8c420000 	lw	v0,0(v0)
80003190:	10400006 	beqz	v0,800031ac <__sfp+0xec>
80003194:	00000000 	nop
80003198:	8fc20028 	lw	v0,40(s8)
8000319c:	8c420000 	lw	v0,0(v0)
800031a0:	afc20028 	sw	v0,40(s8)
800031a4:	1000ffd8 	b	80003108 <__sfp+0x48>
800031a8:	00000000 	nop
800031ac:	00000000 	nop
800031b0:	0c000d02 	jal	80003408 <__sfp_lock_release>
800031b4:	00000000 	nop
800031b8:	8fc20040 	lw	v0,64(s8)
800031bc:	2403000c 	li	v1,12
800031c0:	ac430000 	sw	v1,0(v0)
800031c4:	00001025 	move	v0,zero
800031c8:	10000028 	b	8000326c <__sfp+0x1ac>
800031cc:	00000000 	nop
800031d0:	00000000 	nop
800031d4:	8fc20020 	lw	v0,32(s8)
800031d8:	2403ffff 	li	v1,-1
800031dc:	a443000e 	sh	v1,14(v0)
800031e0:	8fc20020 	lw	v0,32(s8)
800031e4:	24030001 	li	v1,1
800031e8:	a443000c 	sh	v1,12(v0)
800031ec:	8fc20020 	lw	v0,32(s8)
800031f0:	ac400064 	sw	zero,100(v0)
800031f4:	0c000d02 	jal	80003408 <__sfp_lock_release>
800031f8:	00000000 	nop
800031fc:	8fc20020 	lw	v0,32(s8)
80003200:	ac400000 	sw	zero,0(v0)
80003204:	8fc20020 	lw	v0,32(s8)
80003208:	ac400008 	sw	zero,8(v0)
8000320c:	8fc20020 	lw	v0,32(s8)
80003210:	ac400004 	sw	zero,4(v0)
80003214:	8fc20020 	lw	v0,32(s8)
80003218:	ac400010 	sw	zero,16(v0)
8000321c:	8fc20020 	lw	v0,32(s8)
80003220:	ac400014 	sw	zero,20(v0)
80003224:	8fc20020 	lw	v0,32(s8)
80003228:	ac400018 	sw	zero,24(v0)
8000322c:	8fc20020 	lw	v0,32(s8)
80003230:	2442005c 	addiu	v0,v0,92
80003234:	24060008 	li	a2,8
80003238:	00002825 	move	a1,zero
8000323c:	00402025 	move	a0,v0
80003240:	0c00173d 	jal	80005cf4 <memset>
80003244:	00000000 	nop
80003248:	8fc20020 	lw	v0,32(s8)
8000324c:	ac400030 	sw	zero,48(v0)
80003250:	8fc20020 	lw	v0,32(s8)
80003254:	ac400034 	sw	zero,52(v0)
80003258:	8fc20020 	lw	v0,32(s8)
8000325c:	ac400044 	sw	zero,68(v0)
80003260:	8fc20020 	lw	v0,32(s8)
80003264:	ac400048 	sw	zero,72(v0)
80003268:	8fc20020 	lw	v0,32(s8)
8000326c:	03c0e825 	move	sp,s8
80003270:	dfbf0038 	ld	ra,56(sp)
80003274:	dfbe0030 	ld	s8,48(sp)
80003278:	27bd0040 	addiu	sp,sp,64
8000327c:	03e00008 	jr	ra
80003280:	00000000 	nop

80003284 <_cleanup_r>:
80003284:	27bdffc8 	addiu	sp,sp,-56
80003288:	ffbf0030 	sd	ra,48(sp)
8000328c:	ffbe0028 	sd	s8,40(sp)
80003290:	03a0f025 	move	s8,sp
80003294:	afc40038 	sw	a0,56(s8)
80003298:	3c028001 	lui	v0,0x8001
8000329c:	2442db28 	addiu	v0,v0,-9432
800032a0:	afc20020 	sw	v0,32(s8)
800032a4:	8fc50020 	lw	a1,32(s8)
800032a8:	8fc40038 	lw	a0,56(s8)
800032ac:	0c001022 	jal	80004088 <_fwalk_reent>
800032b0:	00000000 	nop
800032b4:	00000000 	nop
800032b8:	03c0e825 	move	sp,s8
800032bc:	dfbf0030 	ld	ra,48(sp)
800032c0:	dfbe0028 	ld	s8,40(sp)
800032c4:	27bd0038 	addiu	sp,sp,56
800032c8:	03e00008 	jr	ra
800032cc:	00000000 	nop

800032d0 <_cleanup>:
800032d0:	27bdffd0 	addiu	sp,sp,-48
800032d4:	ffbf0028 	sd	ra,40(sp)
800032d8:	ffbe0020 	sd	s8,32(sp)
800032dc:	03a0f025 	move	s8,sp
800032e0:	8f82802c 	lw	v0,-32724(gp)
800032e4:	00402025 	move	a0,v0
800032e8:	0c000ca1 	jal	80003284 <_cleanup_r>
800032ec:	00000000 	nop
800032f0:	00000000 	nop
800032f4:	03c0e825 	move	sp,s8
800032f8:	dfbf0028 	ld	ra,40(sp)
800032fc:	dfbe0020 	ld	s8,32(sp)
80003300:	27bd0030 	addiu	sp,sp,48
80003304:	03e00008 	jr	ra
80003308:	00000000 	nop

8000330c <__sinit>:
8000330c:	27bdffd0 	addiu	sp,sp,-48
80003310:	ffbf0028 	sd	ra,40(sp)
80003314:	ffbe0020 	sd	s8,32(sp)
80003318:	03a0f025 	move	s8,sp
8000331c:	afc40030 	sw	a0,48(s8)
80003320:	0c000d0b 	jal	8000342c <__sinit_lock_acquire>
80003324:	00000000 	nop
80003328:	8fc20030 	lw	v0,48(s8)
8000332c:	8c420038 	lw	v0,56(v0)
80003330:	10400005 	beqz	v0,80003348 <__sinit+0x3c>
80003334:	00000000 	nop
80003338:	0c000d14 	jal	80003450 <__sinit_lock_release>
8000333c:	00000000 	nop
80003340:	10000022 	b	800033cc <__sinit+0xc0>
80003344:	00000000 	nop
80003348:	8fc20030 	lw	v0,48(s8)
8000334c:	3c038000 	lui	v1,0x8000
80003350:	24633284 	addiu	v1,v1,12932
80003354:	ac43003c 	sw	v1,60(v0)
80003358:	8fc20030 	lw	v0,48(s8)
8000335c:	ac4002e0 	sw	zero,736(v0)
80003360:	8fc20030 	lw	v0,48(s8)
80003364:	24030003 	li	v1,3
80003368:	ac4302e4 	sw	v1,740(v0)
8000336c:	8fc20030 	lw	v0,48(s8)
80003370:	244302ec 	addiu	v1,v0,748
80003374:	8fc20030 	lw	v0,48(s8)
80003378:	ac4302e8 	sw	v1,744(v0)
8000337c:	8fc20030 	lw	v0,48(s8)
80003380:	8c420004 	lw	v0,4(v0)
80003384:	00402025 	move	a0,v0
80003388:	0c000bc5 	jal	80002f14 <stdin_init>
8000338c:	00000000 	nop
80003390:	8fc20030 	lw	v0,48(s8)
80003394:	8c420008 	lw	v0,8(v0)
80003398:	00402025 	move	a0,v0
8000339c:	0c000bd6 	jal	80002f58 <stdout_init>
800033a0:	00000000 	nop
800033a4:	8fc20030 	lw	v0,48(s8)
800033a8:	8c42000c 	lw	v0,12(v0)
800033ac:	00402025 	move	a0,v0
800033b0:	0c000be7 	jal	80002f9c <stderr_init>
800033b4:	00000000 	nop
800033b8:	8fc20030 	lw	v0,48(s8)
800033bc:	24030001 	li	v1,1
800033c0:	ac430038 	sw	v1,56(v0)
800033c4:	0c000d14 	jal	80003450 <__sinit_lock_release>
800033c8:	00000000 	nop
800033cc:	03c0e825 	move	sp,s8
800033d0:	dfbf0028 	ld	ra,40(sp)
800033d4:	dfbe0020 	ld	s8,32(sp)
800033d8:	27bd0030 	addiu	sp,sp,48
800033dc:	03e00008 	jr	ra
800033e0:	00000000 	nop

800033e4 <__sfp_lock_acquire>:
800033e4:	27bdfff8 	addiu	sp,sp,-8
800033e8:	ffbe0000 	sd	s8,0(sp)
800033ec:	03a0f025 	move	s8,sp
800033f0:	00000000 	nop
800033f4:	03c0e825 	move	sp,s8
800033f8:	dfbe0000 	ld	s8,0(sp)
800033fc:	27bd0008 	addiu	sp,sp,8
80003400:	03e00008 	jr	ra
80003404:	00000000 	nop

80003408 <__sfp_lock_release>:
80003408:	27bdfff8 	addiu	sp,sp,-8
8000340c:	ffbe0000 	sd	s8,0(sp)
80003410:	03a0f025 	move	s8,sp
80003414:	00000000 	nop
80003418:	03c0e825 	move	sp,s8
8000341c:	dfbe0000 	ld	s8,0(sp)
80003420:	27bd0008 	addiu	sp,sp,8
80003424:	03e00008 	jr	ra
80003428:	00000000 	nop

8000342c <__sinit_lock_acquire>:
8000342c:	27bdfff8 	addiu	sp,sp,-8
80003430:	ffbe0000 	sd	s8,0(sp)
80003434:	03a0f025 	move	s8,sp
80003438:	00000000 	nop
8000343c:	03c0e825 	move	sp,s8
80003440:	dfbe0000 	ld	s8,0(sp)
80003444:	27bd0008 	addiu	sp,sp,8
80003448:	03e00008 	jr	ra
8000344c:	00000000 	nop

80003450 <__sinit_lock_release>:
80003450:	27bdfff8 	addiu	sp,sp,-8
80003454:	ffbe0000 	sd	s8,0(sp)
80003458:	03a0f025 	move	s8,sp
8000345c:	00000000 	nop
80003460:	03c0e825 	move	sp,s8
80003464:	dfbe0000 	ld	s8,0(sp)
80003468:	27bd0008 	addiu	sp,sp,8
8000346c:	03e00008 	jr	ra
80003470:	00000000 	nop

80003474 <__fp_lock>:
80003474:	27bdfff8 	addiu	sp,sp,-8
80003478:	ffbe0000 	sd	s8,0(sp)
8000347c:	03a0f025 	move	s8,sp
80003480:	afc40008 	sw	a0,8(s8)
80003484:	00001025 	move	v0,zero
80003488:	03c0e825 	move	sp,s8
8000348c:	dfbe0000 	ld	s8,0(sp)
80003490:	27bd0008 	addiu	sp,sp,8
80003494:	03e00008 	jr	ra
80003498:	00000000 	nop

8000349c <__fp_unlock>:
8000349c:	27bdfff8 	addiu	sp,sp,-8
800034a0:	ffbe0000 	sd	s8,0(sp)
800034a4:	03a0f025 	move	s8,sp
800034a8:	afc40008 	sw	a0,8(s8)
800034ac:	00001025 	move	v0,zero
800034b0:	03c0e825 	move	sp,s8
800034b4:	dfbe0000 	ld	s8,0(sp)
800034b8:	27bd0008 	addiu	sp,sp,8
800034bc:	03e00008 	jr	ra
800034c0:	00000000 	nop

800034c4 <__fp_lock_all>:
800034c4:	27bdffd0 	addiu	sp,sp,-48
800034c8:	ffbf0028 	sd	ra,40(sp)
800034cc:	ffbe0020 	sd	s8,32(sp)
800034d0:	03a0f025 	move	s8,sp
800034d4:	0c000cf9 	jal	800033e4 <__sfp_lock_acquire>
800034d8:	00000000 	nop
800034dc:	8f838028 	lw	v1,-32728(gp)
800034e0:	3c028000 	lui	v0,0x8000
800034e4:	24453474 	addiu	a1,v0,13428
800034e8:	00602025 	move	a0,v1
800034ec:	0c000fec 	jal	80003fb0 <_fwalk>
800034f0:	00000000 	nop
800034f4:	00000000 	nop
800034f8:	03c0e825 	move	sp,s8
800034fc:	dfbf0028 	ld	ra,40(sp)
80003500:	dfbe0020 	ld	s8,32(sp)
80003504:	27bd0030 	addiu	sp,sp,48
80003508:	03e00008 	jr	ra
8000350c:	00000000 	nop

80003510 <__fp_unlock_all>:
80003510:	27bdffd0 	addiu	sp,sp,-48
80003514:	ffbf0028 	sd	ra,40(sp)
80003518:	ffbe0020 	sd	s8,32(sp)
8000351c:	03a0f025 	move	s8,sp
80003520:	8f838028 	lw	v1,-32728(gp)
80003524:	3c028000 	lui	v0,0x8000
80003528:	2445349c 	addiu	a1,v0,13468
8000352c:	00602025 	move	a0,v1
80003530:	0c000fec 	jal	80003fb0 <_fwalk>
80003534:	00000000 	nop
80003538:	0c000d02 	jal	80003408 <__sfp_lock_release>
8000353c:	00000000 	nop
80003540:	00000000 	nop
80003544:	03c0e825 	move	sp,s8
80003548:	dfbf0028 	ld	ra,40(sp)
8000354c:	dfbe0020 	ld	s8,32(sp)
80003550:	27bd0030 	addiu	sp,sp,48
80003554:	03e00008 	jr	ra
80003558:	00000000 	nop

8000355c <_fprintf_r>:
8000355c:	27bdffc8 	addiu	sp,sp,-56
80003560:	ffbf0030 	sd	ra,48(sp)
80003564:	ffbe0028 	sd	s8,40(sp)
80003568:	03a0f025 	move	s8,sp
8000356c:	afc40038 	sw	a0,56(s8)
80003570:	afc50040 	sw	a1,64(s8)
80003574:	afc60048 	sw	a2,72(s8)
80003578:	ffc70050 	sd	a3,80(s8)
8000357c:	27c20050 	addiu	v0,s8,80
80003580:	afc20024 	sw	v0,36(s8)
80003584:	8fc20024 	lw	v0,36(s8)
80003588:	00403825 	move	a3,v0
8000358c:	8fc60048 	lw	a2,72(s8)
80003590:	8fc50040 	lw	a1,64(s8)
80003594:	8fc40038 	lw	a0,56(s8)
80003598:	0c002372 	jal	80008dc8 <_vfprintf_r>
8000359c:	00000000 	nop
800035a0:	afc20020 	sw	v0,32(s8)
800035a4:	8fc20020 	lw	v0,32(s8)
800035a8:	03c0e825 	move	sp,s8
800035ac:	dfbf0030 	ld	ra,48(sp)
800035b0:	dfbe0028 	ld	s8,40(sp)
800035b4:	27bd0038 	addiu	sp,sp,56
800035b8:	03e00008 	jr	ra
800035bc:	00000000 	nop

800035c0 <fprintf>:
800035c0:	27bdffc8 	addiu	sp,sp,-56
800035c4:	ffbf0030 	sd	ra,48(sp)
800035c8:	ffbe0028 	sd	s8,40(sp)
800035cc:	03a0f025 	move	s8,sp
800035d0:	afc40038 	sw	a0,56(s8)
800035d4:	afc50040 	sw	a1,64(s8)
800035d8:	ffc60048 	sd	a2,72(s8)
800035dc:	ffc70050 	sd	a3,80(s8)
800035e0:	27c20048 	addiu	v0,s8,72
800035e4:	afc20024 	sw	v0,36(s8)
800035e8:	8f828028 	lw	v0,-32728(gp)
800035ec:	8fc30024 	lw	v1,36(s8)
800035f0:	00603825 	move	a3,v1
800035f4:	8fc60040 	lw	a2,64(s8)
800035f8:	8fc50038 	lw	a1,56(s8)
800035fc:	00402025 	move	a0,v0
80003600:	0c002372 	jal	80008dc8 <_vfprintf_r>
80003604:	00000000 	nop
80003608:	afc20020 	sw	v0,32(s8)
8000360c:	8fc20020 	lw	v0,32(s8)
80003610:	03c0e825 	move	sp,s8
80003614:	dfbf0030 	ld	ra,48(sp)
80003618:	dfbe0028 	ld	s8,40(sp)
8000361c:	27bd0038 	addiu	sp,sp,56
80003620:	03e00008 	jr	ra
80003624:	00000000 	nop

80003628 <_fputc_r>:
80003628:	27bdffc8 	addiu	sp,sp,-56
8000362c:	ffbf0030 	sd	ra,48(sp)
80003630:	ffbe0028 	sd	s8,40(sp)
80003634:	03a0f025 	move	s8,sp
80003638:	afc40038 	sw	a0,56(s8)
8000363c:	afc50040 	sw	a1,64(s8)
80003640:	afc60048 	sw	a2,72(s8)
80003644:	8fc20038 	lw	v0,56(s8)
80003648:	afc20020 	sw	v0,32(s8)
8000364c:	8fc20020 	lw	v0,32(s8)
80003650:	10400008 	beqz	v0,80003674 <_fputc_r+0x4c>
80003654:	00000000 	nop
80003658:	8fc20020 	lw	v0,32(s8)
8000365c:	8c420038 	lw	v0,56(v0)
80003660:	14400004 	bnez	v0,80003674 <_fputc_r+0x4c>
80003664:	00000000 	nop
80003668:	8fc40020 	lw	a0,32(s8)
8000366c:	0c000cc3 	jal	8000330c <__sinit>
80003670:	00000000 	nop
80003674:	8fc60048 	lw	a2,72(s8)
80003678:	8fc50040 	lw	a1,64(s8)
8000367c:	8fc40038 	lw	a0,56(s8)
80003680:	0c001c4d 	jal	80007134 <_putc_r>
80003684:	00000000 	nop
80003688:	afc20024 	sw	v0,36(s8)
8000368c:	8fc20024 	lw	v0,36(s8)
80003690:	03c0e825 	move	sp,s8
80003694:	dfbf0030 	ld	ra,48(sp)
80003698:	dfbe0028 	ld	s8,40(sp)
8000369c:	27bd0038 	addiu	sp,sp,56
800036a0:	03e00008 	jr	ra
800036a4:	00000000 	nop

800036a8 <fputc>:
800036a8:	27bdffc0 	addiu	sp,sp,-64
800036ac:	ffbf0038 	sd	ra,56(sp)
800036b0:	ffbe0030 	sd	s8,48(sp)
800036b4:	03a0f025 	move	s8,sp
800036b8:	afc40040 	sw	a0,64(s8)
800036bc:	afc50048 	sw	a1,72(s8)
800036c0:	8f828028 	lw	v0,-32728(gp)
800036c4:	afc20020 	sw	v0,32(s8)
800036c8:	8fc20020 	lw	v0,32(s8)
800036cc:	afc20024 	sw	v0,36(s8)
800036d0:	8fc20024 	lw	v0,36(s8)
800036d4:	10400008 	beqz	v0,800036f8 <fputc+0x50>
800036d8:	00000000 	nop
800036dc:	8fc20024 	lw	v0,36(s8)
800036e0:	8c420038 	lw	v0,56(v0)
800036e4:	14400004 	bnez	v0,800036f8 <fputc+0x50>
800036e8:	00000000 	nop
800036ec:	8fc40024 	lw	a0,36(s8)
800036f0:	0c000cc3 	jal	8000330c <__sinit>
800036f4:	00000000 	nop
800036f8:	8fc60048 	lw	a2,72(s8)
800036fc:	8fc50040 	lw	a1,64(s8)
80003700:	8fc40020 	lw	a0,32(s8)
80003704:	0c001c4d 	jal	80007134 <_putc_r>
80003708:	00000000 	nop
8000370c:	afc20028 	sw	v0,40(s8)
80003710:	8fc20028 	lw	v0,40(s8)
80003714:	03c0e825 	move	sp,s8
80003718:	dfbf0038 	ld	ra,56(sp)
8000371c:	dfbe0030 	ld	s8,48(sp)
80003720:	27bd0040 	addiu	sp,sp,64
80003724:	03e00008 	jr	ra
80003728:	00000000 	nop

8000372c <_free_r>:
8000372c:	27bdffa8 	addiu	sp,sp,-88
80003730:	ffbf0050 	sd	ra,80(sp)
80003734:	ffbe0048 	sd	s8,72(sp)
80003738:	03a0f025 	move	s8,sp
8000373c:	afc40058 	sw	a0,88(s8)
80003740:	afc50060 	sw	a1,96(s8)
80003744:	8fc20060 	lw	v0,96(s8)
80003748:	10400170 	beqz	v0,80003d0c <_free_r+0x5e0>
8000374c:	00000000 	nop
80003750:	8fc40058 	lw	a0,88(s8)
80003754:	0c001ba7 	jal	80006e9c <__malloc_lock>
80003758:	00000000 	nop
8000375c:	8fc20060 	lw	v0,96(s8)
80003760:	2442fff8 	addiu	v0,v0,-8
80003764:	afc20020 	sw	v0,32(s8)
80003768:	8fc20020 	lw	v0,32(s8)
8000376c:	8c420004 	lw	v0,4(v0)
80003770:	afc20034 	sw	v0,52(s8)
80003774:	8fc30034 	lw	v1,52(s8)
80003778:	2402fffe 	li	v0,-2
8000377c:	00621024 	and	v0,v1,v0
80003780:	afc20024 	sw	v0,36(s8)
80003784:	8fc30020 	lw	v1,32(s8)
80003788:	8fc20024 	lw	v0,36(s8)
8000378c:	00621021 	addu	v0,v1,v0
80003790:	afc20038 	sw	v0,56(s8)
80003794:	8fc20038 	lw	v0,56(s8)
80003798:	8c430004 	lw	v1,4(v0)
8000379c:	2402fffc 	li	v0,-4
800037a0:	00621024 	and	v0,v1,v0
800037a4:	afc2003c 	sw	v0,60(s8)
800037a8:	3c028002 	lui	v0,0x8002
800037ac:	2442ce98 	addiu	v0,v0,-12648
800037b0:	8c420008 	lw	v0,8(v0)
800037b4:	8fc30038 	lw	v1,56(s8)
800037b8:	14620038 	bne	v1,v0,8000389c <_free_r+0x170>
800037bc:	00000000 	nop
800037c0:	8fc30024 	lw	v1,36(s8)
800037c4:	8fc2003c 	lw	v0,60(s8)
800037c8:	00621021 	addu	v0,v1,v0
800037cc:	afc20024 	sw	v0,36(s8)
800037d0:	8fc20034 	lw	v0,52(s8)
800037d4:	30420001 	andi	v0,v0,0x1
800037d8:	14400019 	bnez	v0,80003840 <_free_r+0x114>
800037dc:	00000000 	nop
800037e0:	8fc20020 	lw	v0,32(s8)
800037e4:	8c420000 	lw	v0,0(v0)
800037e8:	afc20040 	sw	v0,64(s8)
800037ec:	8fc20040 	lw	v0,64(s8)
800037f0:	00021023 	negu	v0,v0
800037f4:	8fc30020 	lw	v1,32(s8)
800037f8:	00621021 	addu	v0,v1,v0
800037fc:	afc20020 	sw	v0,32(s8)
80003800:	8fc30024 	lw	v1,36(s8)
80003804:	8fc20040 	lw	v0,64(s8)
80003808:	00621021 	addu	v0,v1,v0
8000380c:	afc20024 	sw	v0,36(s8)
80003810:	8fc20020 	lw	v0,32(s8)
80003814:	8c42000c 	lw	v0,12(v0)
80003818:	afc20028 	sw	v0,40(s8)
8000381c:	8fc20020 	lw	v0,32(s8)
80003820:	8c420008 	lw	v0,8(v0)
80003824:	afc2002c 	sw	v0,44(s8)
80003828:	8fc2002c 	lw	v0,44(s8)
8000382c:	8fc30028 	lw	v1,40(s8)
80003830:	ac43000c 	sw	v1,12(v0)
80003834:	8fc20028 	lw	v0,40(s8)
80003838:	8fc3002c 	lw	v1,44(s8)
8000383c:	ac430008 	sw	v1,8(v0)
80003840:	8fc20024 	lw	v0,36(s8)
80003844:	34430001 	ori	v1,v0,0x1
80003848:	8fc20020 	lw	v0,32(s8)
8000384c:	ac430004 	sw	v1,4(v0)
80003850:	3c028002 	lui	v0,0x8002
80003854:	2442ce98 	addiu	v0,v0,-12648
80003858:	8fc30020 	lw	v1,32(s8)
8000385c:	ac430008 	sw	v1,8(v0)
80003860:	8f828030 	lw	v0,-32720(gp)
80003864:	8fc30024 	lw	v1,36(s8)
80003868:	0062102b 	sltu	v0,v1,v0
8000386c:	14400006 	bnez	v0,80003888 <_free_r+0x15c>
80003870:	00000000 	nop
80003874:	8f828328 	lw	v0,-31960(gp)
80003878:	00402825 	move	a1,v0
8000387c:	8fc40058 	lw	a0,88(s8)
80003880:	0c000f4a 	jal	80003d28 <_malloc_trim_r>
80003884:	00000000 	nop
80003888:	8fc40058 	lw	a0,88(s8)
8000388c:	0c001bb1 	jal	80006ec4 <__malloc_unlock>
80003890:	00000000 	nop
80003894:	1000011e 	b	80003d10 <_free_r+0x5e4>
80003898:	00000000 	nop
8000389c:	8fc20038 	lw	v0,56(s8)
800038a0:	8fc3003c 	lw	v1,60(s8)
800038a4:	ac430004 	sw	v1,4(v0)
800038a8:	afc00030 	sw	zero,48(s8)
800038ac:	8fc20034 	lw	v0,52(s8)
800038b0:	30420001 	andi	v0,v0,0x1
800038b4:	14400023 	bnez	v0,80003944 <_free_r+0x218>
800038b8:	00000000 	nop
800038bc:	8fc20020 	lw	v0,32(s8)
800038c0:	8c420000 	lw	v0,0(v0)
800038c4:	afc20040 	sw	v0,64(s8)
800038c8:	8fc20040 	lw	v0,64(s8)
800038cc:	00021023 	negu	v0,v0
800038d0:	8fc30020 	lw	v1,32(s8)
800038d4:	00621021 	addu	v0,v1,v0
800038d8:	afc20020 	sw	v0,32(s8)
800038dc:	8fc30024 	lw	v1,36(s8)
800038e0:	8fc20040 	lw	v0,64(s8)
800038e4:	00621021 	addu	v0,v1,v0
800038e8:	afc20024 	sw	v0,36(s8)
800038ec:	8fc20020 	lw	v0,32(s8)
800038f0:	8c430008 	lw	v1,8(v0)
800038f4:	3c028002 	lui	v0,0x8002
800038f8:	2442cea0 	addiu	v0,v0,-12640
800038fc:	14620005 	bne	v1,v0,80003914 <_free_r+0x1e8>
80003900:	00000000 	nop
80003904:	24020001 	li	v0,1
80003908:	afc20030 	sw	v0,48(s8)
8000390c:	1000000d 	b	80003944 <_free_r+0x218>
80003910:	00000000 	nop
80003914:	8fc20020 	lw	v0,32(s8)
80003918:	8c42000c 	lw	v0,12(v0)
8000391c:	afc20028 	sw	v0,40(s8)
80003920:	8fc20020 	lw	v0,32(s8)
80003924:	8c420008 	lw	v0,8(v0)
80003928:	afc2002c 	sw	v0,44(s8)
8000392c:	8fc2002c 	lw	v0,44(s8)
80003930:	8fc30028 	lw	v1,40(s8)
80003934:	ac43000c 	sw	v1,12(v0)
80003938:	8fc20028 	lw	v0,40(s8)
8000393c:	8fc3002c 	lw	v1,44(s8)
80003940:	ac430008 	sw	v1,8(v0)
80003944:	8fc30038 	lw	v1,56(s8)
80003948:	8fc2003c 	lw	v0,60(s8)
8000394c:	00621021 	addu	v0,v1,v0
80003950:	8c420004 	lw	v0,4(v0)
80003954:	30420001 	andi	v0,v0,0x1
80003958:	1440002e 	bnez	v0,80003a14 <_free_r+0x2e8>
8000395c:	00000000 	nop
80003960:	8fc30024 	lw	v1,36(s8)
80003964:	8fc2003c 	lw	v0,60(s8)
80003968:	00621021 	addu	v0,v1,v0
8000396c:	afc20024 	sw	v0,36(s8)
80003970:	8fc20030 	lw	v0,48(s8)
80003974:	1440001b 	bnez	v0,800039e4 <_free_r+0x2b8>
80003978:	00000000 	nop
8000397c:	8fc20038 	lw	v0,56(s8)
80003980:	8c430008 	lw	v1,8(v0)
80003984:	3c028002 	lui	v0,0x8002
80003988:	2442cea0 	addiu	v0,v0,-12640
8000398c:	14620015 	bne	v1,v0,800039e4 <_free_r+0x2b8>
80003990:	00000000 	nop
80003994:	24020001 	li	v0,1
80003998:	afc20030 	sw	v0,48(s8)
8000399c:	3c028002 	lui	v0,0x8002
800039a0:	2442cea0 	addiu	v0,v0,-12640
800039a4:	8fc30020 	lw	v1,32(s8)
800039a8:	ac43000c 	sw	v1,12(v0)
800039ac:	3c038002 	lui	v1,0x8002
800039b0:	2463cea0 	addiu	v1,v1,-12640
800039b4:	8c42000c 	lw	v0,12(v0)
800039b8:	ac620008 	sw	v0,8(v1)
800039bc:	3c028002 	lui	v0,0x8002
800039c0:	2443cea0 	addiu	v1,v0,-12640
800039c4:	8fc20020 	lw	v0,32(s8)
800039c8:	ac43000c 	sw	v1,12(v0)
800039cc:	8fc20020 	lw	v0,32(s8)
800039d0:	8c43000c 	lw	v1,12(v0)
800039d4:	8fc20020 	lw	v0,32(s8)
800039d8:	ac430008 	sw	v1,8(v0)
800039dc:	1000000d 	b	80003a14 <_free_r+0x2e8>
800039e0:	00000000 	nop
800039e4:	8fc20038 	lw	v0,56(s8)
800039e8:	8c42000c 	lw	v0,12(v0)
800039ec:	afc20028 	sw	v0,40(s8)
800039f0:	8fc20038 	lw	v0,56(s8)
800039f4:	8c420008 	lw	v0,8(v0)
800039f8:	afc2002c 	sw	v0,44(s8)
800039fc:	8fc2002c 	lw	v0,44(s8)
80003a00:	8fc30028 	lw	v1,40(s8)
80003a04:	ac43000c 	sw	v1,12(v0)
80003a08:	8fc20028 	lw	v0,40(s8)
80003a0c:	8fc3002c 	lw	v1,44(s8)
80003a10:	ac430008 	sw	v1,8(v0)
80003a14:	8fc20024 	lw	v0,36(s8)
80003a18:	34430001 	ori	v1,v0,0x1
80003a1c:	8fc20020 	lw	v0,32(s8)
80003a20:	ac430004 	sw	v1,4(v0)
80003a24:	8fc30020 	lw	v1,32(s8)
80003a28:	8fc20024 	lw	v0,36(s8)
80003a2c:	00621021 	addu	v0,v1,v0
80003a30:	8fc30024 	lw	v1,36(s8)
80003a34:	ac430000 	sw	v1,0(v0)
80003a38:	8fc20030 	lw	v0,48(s8)
80003a3c:	144000ae 	bnez	v0,80003cf8 <_free_r+0x5cc>
80003a40:	00000000 	nop
80003a44:	8fc20024 	lw	v0,36(s8)
80003a48:	2c420200 	sltiu	v0,v0,512
80003a4c:	1040002e 	beqz	v0,80003b08 <_free_r+0x3dc>
80003a50:	00000000 	nop
80003a54:	8fc20024 	lw	v0,36(s8)
80003a58:	000210c2 	srl	v0,v0,0x3
80003a5c:	afc20044 	sw	v0,68(s8)
80003a60:	3c028002 	lui	v0,0x8002
80003a64:	2442ce98 	addiu	v0,v0,-12648
80003a68:	8c440004 	lw	a0,4(v0)
80003a6c:	8fc20044 	lw	v0,68(s8)
80003a70:	00021fc3 	sra	v1,v0,0x1f
80003a74:	00031f82 	srl	v1,v1,0x1e
80003a78:	00621021 	addu	v0,v1,v0
80003a7c:	00021083 	sra	v0,v0,0x2
80003a80:	00401825 	move	v1,v0
80003a84:	24020001 	li	v0,1
80003a88:	00621804 	sllv	v1,v0,v1
80003a8c:	3c028002 	lui	v0,0x8002
80003a90:	2442ce98 	addiu	v0,v0,-12648
80003a94:	00831825 	or	v1,a0,v1
80003a98:	ac430004 	sw	v1,4(v0)
80003a9c:	8fc20044 	lw	v0,68(s8)
80003aa0:	24420001 	addiu	v0,v0,1
80003aa4:	00021040 	sll	v0,v0,0x1
80003aa8:	00021880 	sll	v1,v0,0x2
80003aac:	3c028002 	lui	v0,0x8002
80003ab0:	2442ce98 	addiu	v0,v0,-12648
80003ab4:	00621021 	addu	v0,v1,v0
80003ab8:	2442fff8 	addiu	v0,v0,-8
80003abc:	afc20028 	sw	v0,40(s8)
80003ac0:	8fc20028 	lw	v0,40(s8)
80003ac4:	8c420008 	lw	v0,8(v0)
80003ac8:	afc2002c 	sw	v0,44(s8)
80003acc:	8fc20020 	lw	v0,32(s8)
80003ad0:	8fc30028 	lw	v1,40(s8)
80003ad4:	ac43000c 	sw	v1,12(v0)
80003ad8:	8fc20020 	lw	v0,32(s8)
80003adc:	8fc3002c 	lw	v1,44(s8)
80003ae0:	ac430008 	sw	v1,8(v0)
80003ae4:	8fc20028 	lw	v0,40(s8)
80003ae8:	8fc30020 	lw	v1,32(s8)
80003aec:	ac430008 	sw	v1,8(v0)
80003af0:	8fc20028 	lw	v0,40(s8)
80003af4:	8c430008 	lw	v1,8(v0)
80003af8:	8fc2002c 	lw	v0,44(s8)
80003afc:	ac43000c 	sw	v1,12(v0)
80003b00:	1000007d 	b	80003cf8 <_free_r+0x5cc>
80003b04:	00000000 	nop
80003b08:	8fc20024 	lw	v0,36(s8)
80003b0c:	00021242 	srl	v0,v0,0x9
80003b10:	14400005 	bnez	v0,80003b28 <_free_r+0x3fc>
80003b14:	00000000 	nop
80003b18:	8fc20024 	lw	v0,36(s8)
80003b1c:	000210c2 	srl	v0,v0,0x3
80003b20:	10000034 	b	80003bf4 <_free_r+0x4c8>
80003b24:	00000000 	nop
80003b28:	8fc20024 	lw	v0,36(s8)
80003b2c:	00021242 	srl	v0,v0,0x9
80003b30:	2c420005 	sltiu	v0,v0,5
80003b34:	10400006 	beqz	v0,80003b50 <_free_r+0x424>
80003b38:	00000000 	nop
80003b3c:	8fc20024 	lw	v0,36(s8)
80003b40:	00021182 	srl	v0,v0,0x6
80003b44:	24420038 	addiu	v0,v0,56
80003b48:	1000002a 	b	80003bf4 <_free_r+0x4c8>
80003b4c:	00000000 	nop
80003b50:	8fc20024 	lw	v0,36(s8)
80003b54:	00021242 	srl	v0,v0,0x9
80003b58:	2c420015 	sltiu	v0,v0,21
80003b5c:	10400006 	beqz	v0,80003b78 <_free_r+0x44c>
80003b60:	00000000 	nop
80003b64:	8fc20024 	lw	v0,36(s8)
80003b68:	00021242 	srl	v0,v0,0x9
80003b6c:	2442005b 	addiu	v0,v0,91
80003b70:	10000020 	b	80003bf4 <_free_r+0x4c8>
80003b74:	00000000 	nop
80003b78:	8fc20024 	lw	v0,36(s8)
80003b7c:	00021242 	srl	v0,v0,0x9
80003b80:	2c420055 	sltiu	v0,v0,85
80003b84:	10400006 	beqz	v0,80003ba0 <_free_r+0x474>
80003b88:	00000000 	nop
80003b8c:	8fc20024 	lw	v0,36(s8)
80003b90:	00021302 	srl	v0,v0,0xc
80003b94:	2442006e 	addiu	v0,v0,110
80003b98:	10000016 	b	80003bf4 <_free_r+0x4c8>
80003b9c:	00000000 	nop
80003ba0:	8fc20024 	lw	v0,36(s8)
80003ba4:	00021242 	srl	v0,v0,0x9
80003ba8:	2c420155 	sltiu	v0,v0,341
80003bac:	10400006 	beqz	v0,80003bc8 <_free_r+0x49c>
80003bb0:	00000000 	nop
80003bb4:	8fc20024 	lw	v0,36(s8)
80003bb8:	000213c2 	srl	v0,v0,0xf
80003bbc:	24420077 	addiu	v0,v0,119
80003bc0:	1000000c 	b	80003bf4 <_free_r+0x4c8>
80003bc4:	00000000 	nop
80003bc8:	8fc20024 	lw	v0,36(s8)
80003bcc:	00021242 	srl	v0,v0,0x9
80003bd0:	2c420555 	sltiu	v0,v0,1365
80003bd4:	10400006 	beqz	v0,80003bf0 <_free_r+0x4c4>
80003bd8:	00000000 	nop
80003bdc:	8fc20024 	lw	v0,36(s8)
80003be0:	00021482 	srl	v0,v0,0x12
80003be4:	2442007c 	addiu	v0,v0,124
80003be8:	10000002 	b	80003bf4 <_free_r+0x4c8>
80003bec:	00000000 	nop
80003bf0:	2402007e 	li	v0,126
80003bf4:	afc20044 	sw	v0,68(s8)
80003bf8:	8fc20044 	lw	v0,68(s8)
80003bfc:	24420001 	addiu	v0,v0,1
80003c00:	00021040 	sll	v0,v0,0x1
80003c04:	00021880 	sll	v1,v0,0x2
80003c08:	3c028002 	lui	v0,0x8002
80003c0c:	2442ce98 	addiu	v0,v0,-12648
80003c10:	00621021 	addu	v0,v1,v0
80003c14:	2442fff8 	addiu	v0,v0,-8
80003c18:	afc20028 	sw	v0,40(s8)
80003c1c:	8fc20028 	lw	v0,40(s8)
80003c20:	8c420008 	lw	v0,8(v0)
80003c24:	afc2002c 	sw	v0,44(s8)
80003c28:	8fc3002c 	lw	v1,44(s8)
80003c2c:	8fc20028 	lw	v0,40(s8)
80003c30:	14620015 	bne	v1,v0,80003c88 <_free_r+0x55c>
80003c34:	00000000 	nop
80003c38:	3c028002 	lui	v0,0x8002
80003c3c:	2442ce98 	addiu	v0,v0,-12648
80003c40:	8c440004 	lw	a0,4(v0)
80003c44:	8fc20044 	lw	v0,68(s8)
80003c48:	00021fc3 	sra	v1,v0,0x1f
80003c4c:	00031f82 	srl	v1,v1,0x1e
80003c50:	00621021 	addu	v0,v1,v0
80003c54:	00021083 	sra	v0,v0,0x2
80003c58:	00401825 	move	v1,v0
80003c5c:	24020001 	li	v0,1
80003c60:	00621804 	sllv	v1,v0,v1
80003c64:	3c028002 	lui	v0,0x8002
80003c68:	2442ce98 	addiu	v0,v0,-12648
80003c6c:	00831825 	or	v1,a0,v1
80003c70:	ac430004 	sw	v1,4(v0)
80003c74:	10000013 	b	80003cc4 <_free_r+0x598>
80003c78:	00000000 	nop
80003c7c:	8fc2002c 	lw	v0,44(s8)
80003c80:	8c420008 	lw	v0,8(v0)
80003c84:	afc2002c 	sw	v0,44(s8)
80003c88:	8fc3002c 	lw	v1,44(s8)
80003c8c:	8fc20028 	lw	v0,40(s8)
80003c90:	10620009 	beq	v1,v0,80003cb8 <_free_r+0x58c>
80003c94:	00000000 	nop
80003c98:	8fc2002c 	lw	v0,44(s8)
80003c9c:	8c430004 	lw	v1,4(v0)
80003ca0:	2402fffc 	li	v0,-4
80003ca4:	00621024 	and	v0,v1,v0
80003ca8:	8fc30024 	lw	v1,36(s8)
80003cac:	0062102b 	sltu	v0,v1,v0
80003cb0:	1440fff2 	bnez	v0,80003c7c <_free_r+0x550>
80003cb4:	00000000 	nop
80003cb8:	8fc2002c 	lw	v0,44(s8)
80003cbc:	8c42000c 	lw	v0,12(v0)
80003cc0:	afc20028 	sw	v0,40(s8)
80003cc4:	8fc20020 	lw	v0,32(s8)
80003cc8:	8fc30028 	lw	v1,40(s8)
80003ccc:	ac43000c 	sw	v1,12(v0)
80003cd0:	8fc20020 	lw	v0,32(s8)
80003cd4:	8fc3002c 	lw	v1,44(s8)
80003cd8:	ac430008 	sw	v1,8(v0)
80003cdc:	8fc20028 	lw	v0,40(s8)
80003ce0:	8fc30020 	lw	v1,32(s8)
80003ce4:	ac430008 	sw	v1,8(v0)
80003ce8:	8fc20028 	lw	v0,40(s8)
80003cec:	8c430008 	lw	v1,8(v0)
80003cf0:	8fc2002c 	lw	v0,44(s8)
80003cf4:	ac43000c 	sw	v1,12(v0)
80003cf8:	8fc40058 	lw	a0,88(s8)
80003cfc:	0c001bb1 	jal	80006ec4 <__malloc_unlock>
80003d00:	00000000 	nop
80003d04:	10000002 	b	80003d10 <_free_r+0x5e4>
80003d08:	00000000 	nop
80003d0c:	00000000 	nop
80003d10:	03c0e825 	move	sp,s8
80003d14:	dfbf0050 	ld	ra,80(sp)
80003d18:	dfbe0048 	ld	s8,72(sp)
80003d1c:	27bd0058 	addiu	sp,sp,88
80003d20:	03e00008 	jr	ra
80003d24:	00000000 	nop

80003d28 <_malloc_trim_r>:
80003d28:	27bdffb8 	addiu	sp,sp,-72
80003d2c:	ffbf0040 	sd	ra,64(sp)
80003d30:	ffbe0038 	sd	s8,56(sp)
80003d34:	03a0f025 	move	s8,sp
80003d38:	afc40048 	sw	a0,72(s8)
80003d3c:	afc50050 	sw	a1,80(s8)
80003d40:	24021000 	li	v0,4096
80003d44:	afc20020 	sw	v0,32(s8)
80003d48:	8fc40048 	lw	a0,72(s8)
80003d4c:	0c001ba7 	jal	80006e9c <__malloc_lock>
80003d50:	00000000 	nop
80003d54:	3c028002 	lui	v0,0x8002
80003d58:	2442ce98 	addiu	v0,v0,-12648
80003d5c:	8c420008 	lw	v0,8(v0)
80003d60:	8c420004 	lw	v0,4(v0)
80003d64:	00401825 	move	v1,v0
80003d68:	2402fffc 	li	v0,-4
80003d6c:	00621024 	and	v0,v1,v0
80003d70:	afc20024 	sw	v0,36(s8)
80003d74:	8fc30024 	lw	v1,36(s8)
80003d78:	8fc20050 	lw	v0,80(s8)
80003d7c:	00621823 	subu	v1,v1,v0
80003d80:	8fc20020 	lw	v0,32(s8)
80003d84:	00621021 	addu	v0,v1,v0
80003d88:	2443ffef 	addiu	v1,v0,-17
80003d8c:	8fc20020 	lw	v0,32(s8)
80003d90:	0062001b 	divu	zero,v1,v0
80003d94:	004001f4 	teq	v0,zero,0x7
80003d98:	00001010 	mfhi	v0
80003d9c:	00001012 	mflo	v0
80003da0:	2443ffff 	addiu	v1,v0,-1
80003da4:	8fc20020 	lw	v0,32(s8)
80003da8:	00620018 	mult	v1,v0
80003dac:	00001012 	mflo	v0
80003db0:	afc20028 	sw	v0,40(s8)
80003db4:	8fc20020 	lw	v0,32(s8)
80003db8:	8fc30028 	lw	v1,40(s8)
80003dbc:	0062102a 	slt	v0,v1,v0
80003dc0:	10400007 	beqz	v0,80003de0 <_malloc_trim_r+0xb8>
80003dc4:	00000000 	nop
80003dc8:	8fc40048 	lw	a0,72(s8)
80003dcc:	0c001bb1 	jal	80006ec4 <__malloc_unlock>
80003dd0:	00000000 	nop
80003dd4:	00001025 	move	v0,zero
80003dd8:	10000051 	b	80003f20 <_malloc_trim_r+0x1f8>
80003ddc:	00000000 	nop
80003de0:	00002825 	move	a1,zero
80003de4:	8fc40048 	lw	a0,72(s8)
80003de8:	0c001cf2 	jal	800073c8 <_sbrk_r>
80003dec:	00000000 	nop
80003df0:	afc2002c 	sw	v0,44(s8)
80003df4:	3c028002 	lui	v0,0x8002
80003df8:	2442ce98 	addiu	v0,v0,-12648
80003dfc:	8c430008 	lw	v1,8(v0)
80003e00:	8fc20024 	lw	v0,36(s8)
80003e04:	00621021 	addu	v0,v1,v0
80003e08:	8fc3002c 	lw	v1,44(s8)
80003e0c:	10620007 	beq	v1,v0,80003e2c <_malloc_trim_r+0x104>
80003e10:	00000000 	nop
80003e14:	8fc40048 	lw	a0,72(s8)
80003e18:	0c001bb1 	jal	80006ec4 <__malloc_unlock>
80003e1c:	00000000 	nop
80003e20:	00001025 	move	v0,zero
80003e24:	1000003e 	b	80003f20 <_malloc_trim_r+0x1f8>
80003e28:	00000000 	nop
80003e2c:	8fc20028 	lw	v0,40(s8)
80003e30:	00021023 	negu	v0,v0
80003e34:	00402825 	move	a1,v0
80003e38:	8fc40048 	lw	a0,72(s8)
80003e3c:	0c001cf2 	jal	800073c8 <_sbrk_r>
80003e40:	00000000 	nop
80003e44:	afc20030 	sw	v0,48(s8)
80003e48:	8fc30030 	lw	v1,48(s8)
80003e4c:	2402ffff 	li	v0,-1
80003e50:	14620021 	bne	v1,v0,80003ed8 <_malloc_trim_r+0x1b0>
80003e54:	00000000 	nop
80003e58:	00002825 	move	a1,zero
80003e5c:	8fc40048 	lw	a0,72(s8)
80003e60:	0c001cf2 	jal	800073c8 <_sbrk_r>
80003e64:	00000000 	nop
80003e68:	afc2002c 	sw	v0,44(s8)
80003e6c:	3c028002 	lui	v0,0x8002
80003e70:	2442ce98 	addiu	v0,v0,-12648
80003e74:	8c420008 	lw	v0,8(v0)
80003e78:	8fc3002c 	lw	v1,44(s8)
80003e7c:	00621023 	subu	v0,v1,v0
80003e80:	afc20024 	sw	v0,36(s8)
80003e84:	8fc20024 	lw	v0,36(s8)
80003e88:	28420010 	slti	v0,v0,16
80003e8c:	1440000c 	bnez	v0,80003ec0 <_malloc_trim_r+0x198>
80003e90:	00000000 	nop
80003e94:	8f828034 	lw	v0,-32716(gp)
80003e98:	8fc3002c 	lw	v1,44(s8)
80003e9c:	00621823 	subu	v1,v1,v0
80003ea0:	3c028003 	lui	v0,0x8003
80003ea4:	ac43dd00 	sw	v1,-8960(v0)
80003ea8:	8fc20024 	lw	v0,36(s8)
80003eac:	34430001 	ori	v1,v0,0x1
80003eb0:	3c028002 	lui	v0,0x8002
80003eb4:	2442ce98 	addiu	v0,v0,-12648
80003eb8:	8c420008 	lw	v0,8(v0)
80003ebc:	ac430004 	sw	v1,4(v0)
80003ec0:	8fc40048 	lw	a0,72(s8)
80003ec4:	0c001bb1 	jal	80006ec4 <__malloc_unlock>
80003ec8:	00000000 	nop
80003ecc:	00001025 	move	v0,zero
80003ed0:	10000013 	b	80003f20 <_malloc_trim_r+0x1f8>
80003ed4:	00000000 	nop
80003ed8:	8fc30024 	lw	v1,36(s8)
80003edc:	8fc20028 	lw	v0,40(s8)
80003ee0:	00621023 	subu	v0,v1,v0
80003ee4:	34430001 	ori	v1,v0,0x1
80003ee8:	3c028002 	lui	v0,0x8002
80003eec:	2442ce98 	addiu	v0,v0,-12648
80003ef0:	8c420008 	lw	v0,8(v0)
80003ef4:	ac430004 	sw	v1,4(v0)
80003ef8:	3c028003 	lui	v0,0x8003
80003efc:	8c43dd00 	lw	v1,-8960(v0)
80003f00:	8fc20028 	lw	v0,40(s8)
80003f04:	00621823 	subu	v1,v1,v0
80003f08:	3c028003 	lui	v0,0x8003
80003f0c:	ac43dd00 	sw	v1,-8960(v0)
80003f10:	8fc40048 	lw	a0,72(s8)
80003f14:	0c001bb1 	jal	80006ec4 <__malloc_unlock>
80003f18:	00000000 	nop
80003f1c:	24020001 	li	v0,1
80003f20:	03c0e825 	move	sp,s8
80003f24:	dfbf0040 	ld	ra,64(sp)
80003f28:	dfbe0038 	ld	s8,56(sp)
80003f2c:	27bd0048 	addiu	sp,sp,72
80003f30:	03e00008 	jr	ra
80003f34:	00000000 	nop

80003f38 <_fstat_r>:
80003f38:	27bdffc8 	addiu	sp,sp,-56
80003f3c:	ffbf0030 	sd	ra,48(sp)
80003f40:	ffbe0028 	sd	s8,40(sp)
80003f44:	03a0f025 	move	s8,sp
80003f48:	afc40038 	sw	a0,56(s8)
80003f4c:	afc50040 	sw	a1,64(s8)
80003f50:	afc60048 	sw	a2,72(s8)
80003f54:	af808334 	sw	zero,-31948(gp)
80003f58:	8fc50048 	lw	a1,72(s8)
80003f5c:	8fc40040 	lw	a0,64(s8)
80003f60:	0c00675c 	jal	80019d70 <fstat>
80003f64:	00000000 	nop
80003f68:	afc20020 	sw	v0,32(s8)
80003f6c:	8fc30020 	lw	v1,32(s8)
80003f70:	2402ffff 	li	v0,-1
80003f74:	14620007 	bne	v1,v0,80003f94 <_fstat_r+0x5c>
80003f78:	00000000 	nop
80003f7c:	8f828334 	lw	v0,-31948(gp)
80003f80:	10400004 	beqz	v0,80003f94 <_fstat_r+0x5c>
80003f84:	00000000 	nop
80003f88:	8f838334 	lw	v1,-31948(gp)
80003f8c:	8fc20038 	lw	v0,56(s8)
80003f90:	ac430000 	sw	v1,0(v0)
80003f94:	8fc20020 	lw	v0,32(s8)
80003f98:	03c0e825 	move	sp,s8
80003f9c:	dfbf0030 	ld	ra,48(sp)
80003fa0:	dfbe0028 	ld	s8,40(sp)
80003fa4:	27bd0038 	addiu	sp,sp,56
80003fa8:	03e00008 	jr	ra
80003fac:	00000000 	nop

80003fb0 <_fwalk>:
80003fb0:	27bdffa8 	addiu	sp,sp,-88
80003fb4:	ffbf0050 	sd	ra,80(sp)
80003fb8:	ffbe0048 	sd	s8,72(sp)
80003fbc:	ffb40040 	sd	s4,64(sp)
80003fc0:	ffb30038 	sd	s3,56(sp)
80003fc4:	ffb20030 	sd	s2,48(sp)
80003fc8:	ffb10028 	sd	s1,40(sp)
80003fcc:	ffb00020 	sd	s0,32(sp)
80003fd0:	03a0f025 	move	s8,sp
80003fd4:	afc40058 	sw	a0,88(s8)
80003fd8:	00a0a025 	move	s4,a1
80003fdc:	00009825 	move	s3,zero
80003fe0:	8fc20058 	lw	v0,88(s8)
80003fe4:	245102e0 	addiu	s1,v0,736
80003fe8:	10000019 	b	80004050 <_fwalk+0xa0>
80003fec:	00000000 	nop
80003ff0:	8e300008 	lw	s0,8(s1)
80003ff4:	8e320004 	lw	s2,4(s1)
80003ff8:	10000011 	b	80004040 <_fwalk+0x90>
80003ffc:	00000000 	nop
80004000:	8602000c 	lh	v0,12(s0)
80004004:	1040000d 	beqz	v0,8000403c <_fwalk+0x8c>
80004008:	00000000 	nop
8000400c:	8603000c 	lh	v1,12(s0)
80004010:	24020001 	li	v0,1
80004014:	10620009 	beq	v1,v0,8000403c <_fwalk+0x8c>
80004018:	00000000 	nop
8000401c:	8603000e 	lh	v1,14(s0)
80004020:	2402ffff 	li	v0,-1
80004024:	10620005 	beq	v1,v0,8000403c <_fwalk+0x8c>
80004028:	00000000 	nop
8000402c:	02002025 	move	a0,s0
80004030:	0280f809 	jalr	s4
80004034:	00000000 	nop
80004038:	02629825 	or	s3,s3,v0
8000403c:	26100068 	addiu	s0,s0,104
80004040:	2652ffff 	addiu	s2,s2,-1
80004044:	0641ffee 	bgez	s2,80004000 <_fwalk+0x50>
80004048:	00000000 	nop
8000404c:	8e310000 	lw	s1,0(s1)
80004050:	1620ffe7 	bnez	s1,80003ff0 <_fwalk+0x40>
80004054:	00000000 	nop
80004058:	02601025 	move	v0,s3
8000405c:	03c0e825 	move	sp,s8
80004060:	dfbf0050 	ld	ra,80(sp)
80004064:	dfbe0048 	ld	s8,72(sp)
80004068:	dfb40040 	ld	s4,64(sp)
8000406c:	dfb30038 	ld	s3,56(sp)
80004070:	dfb20030 	ld	s2,48(sp)
80004074:	dfb10028 	ld	s1,40(sp)
80004078:	dfb00020 	ld	s0,32(sp)
8000407c:	27bd0058 	addiu	sp,sp,88
80004080:	03e00008 	jr	ra
80004084:	00000000 	nop

80004088 <_fwalk_reent>:
80004088:	27bdffa8 	addiu	sp,sp,-88
8000408c:	ffbf0050 	sd	ra,80(sp)
80004090:	ffbe0048 	sd	s8,72(sp)
80004094:	ffb40040 	sd	s4,64(sp)
80004098:	ffb30038 	sd	s3,56(sp)
8000409c:	ffb20030 	sd	s2,48(sp)
800040a0:	ffb10028 	sd	s1,40(sp)
800040a4:	ffb00020 	sd	s0,32(sp)
800040a8:	03a0f025 	move	s8,sp
800040ac:	afc40058 	sw	a0,88(s8)
800040b0:	00a0a025 	move	s4,a1
800040b4:	00009825 	move	s3,zero
800040b8:	8fc20058 	lw	v0,88(s8)
800040bc:	245102e0 	addiu	s1,v0,736
800040c0:	1000001a 	b	8000412c <_fwalk_reent+0xa4>
800040c4:	00000000 	nop
800040c8:	8e300008 	lw	s0,8(s1)
800040cc:	8e320004 	lw	s2,4(s1)
800040d0:	10000012 	b	8000411c <_fwalk_reent+0x94>
800040d4:	00000000 	nop
800040d8:	8602000c 	lh	v0,12(s0)
800040dc:	1040000e 	beqz	v0,80004118 <_fwalk_reent+0x90>
800040e0:	00000000 	nop
800040e4:	8603000c 	lh	v1,12(s0)
800040e8:	24020001 	li	v0,1
800040ec:	1062000a 	beq	v1,v0,80004118 <_fwalk_reent+0x90>
800040f0:	00000000 	nop
800040f4:	8603000e 	lh	v1,14(s0)
800040f8:	2402ffff 	li	v0,-1
800040fc:	10620006 	beq	v1,v0,80004118 <_fwalk_reent+0x90>
80004100:	00000000 	nop
80004104:	02002825 	move	a1,s0
80004108:	8fc40058 	lw	a0,88(s8)
8000410c:	0280f809 	jalr	s4
80004110:	00000000 	nop
80004114:	02629825 	or	s3,s3,v0
80004118:	26100068 	addiu	s0,s0,104
8000411c:	2652ffff 	addiu	s2,s2,-1
80004120:	0641ffed 	bgez	s2,800040d8 <_fwalk_reent+0x50>
80004124:	00000000 	nop
80004128:	8e310000 	lw	s1,0(s1)
8000412c:	1620ffe6 	bnez	s1,800040c8 <_fwalk_reent+0x40>
80004130:	00000000 	nop
80004134:	02601025 	move	v0,s3
80004138:	03c0e825 	move	sp,s8
8000413c:	dfbf0050 	ld	ra,80(sp)
80004140:	dfbe0048 	ld	s8,72(sp)
80004144:	dfb40040 	ld	s4,64(sp)
80004148:	dfb30038 	ld	s3,56(sp)
8000414c:	dfb20030 	ld	s2,48(sp)
80004150:	dfb10028 	ld	s1,40(sp)
80004154:	dfb00020 	ld	s0,32(sp)
80004158:	27bd0058 	addiu	sp,sp,88
8000415c:	03e00008 	jr	ra
80004160:	00000000 	nop

80004164 <_fwrite_r>:
80004164:	27bdffb0 	addiu	sp,sp,-80
80004168:	ffbf0048 	sd	ra,72(sp)
8000416c:	ffbe0040 	sd	s8,64(sp)
80004170:	03a0f025 	move	s8,sp
80004174:	afc40050 	sw	a0,80(s8)
80004178:	afc50058 	sw	a1,88(s8)
8000417c:	afc60060 	sw	a2,96(s8)
80004180:	afc70068 	sw	a3,104(s8)
80004184:	8fc20058 	lw	v0,88(s8)
80004188:	afc20038 	sw	v0,56(s8)
8000418c:	8fc30068 	lw	v1,104(s8)
80004190:	8fc20060 	lw	v0,96(s8)
80004194:	00620018 	mult	v1,v0
80004198:	00001012 	mflo	v0
8000419c:	afc20020 	sw	v0,32(s8)
800041a0:	8fc20020 	lw	v0,32(s8)
800041a4:	afc2003c 	sw	v0,60(s8)
800041a8:	8fc2003c 	lw	v0,60(s8)
800041ac:	afc20030 	sw	v0,48(s8)
800041b0:	27c20038 	addiu	v0,s8,56
800041b4:	afc20028 	sw	v0,40(s8)
800041b8:	24020001 	li	v0,1
800041bc:	afc2002c 	sw	v0,44(s8)
800041c0:	8fc20050 	lw	v0,80(s8)
800041c4:	afc20024 	sw	v0,36(s8)
800041c8:	8fc20024 	lw	v0,36(s8)
800041cc:	10400008 	beqz	v0,800041f0 <_fwrite_r+0x8c>
800041d0:	00000000 	nop
800041d4:	8fc20024 	lw	v0,36(s8)
800041d8:	8c420038 	lw	v0,56(v0)
800041dc:	14400004 	bnez	v0,800041f0 <_fwrite_r+0x8c>
800041e0:	00000000 	nop
800041e4:	8fc40024 	lw	a0,36(s8)
800041e8:	0c000cc3 	jal	8000330c <__sinit>
800041ec:	00000000 	nop
800041f0:	8fc20074 	lw	v0,116(s8)
800041f4:	8442000c 	lh	v0,12(v0)
800041f8:	3042ffff 	andi	v0,v0,0xffff
800041fc:	30422000 	andi	v0,v0,0x2000
80004200:	1440000e 	bnez	v0,8000423c <_fwrite_r+0xd8>
80004204:	00000000 	nop
80004208:	8fc20074 	lw	v0,116(s8)
8000420c:	8442000c 	lh	v0,12(v0)
80004210:	34422000 	ori	v0,v0,0x2000
80004214:	00021c00 	sll	v1,v0,0x10
80004218:	00031c03 	sra	v1,v1,0x10
8000421c:	8fc20074 	lw	v0,116(s8)
80004220:	a443000c 	sh	v1,12(v0)
80004224:	8fc20074 	lw	v0,116(s8)
80004228:	8c430064 	lw	v1,100(v0)
8000422c:	2402dfff 	li	v0,-8193
80004230:	00621824 	and	v1,v1,v0
80004234:	8fc20074 	lw	v0,116(s8)
80004238:	ac430064 	sw	v1,100(v0)
8000423c:	27c20028 	addiu	v0,s8,40
80004240:	00403025 	move	a2,v0
80004244:	8fc50074 	lw	a1,116(s8)
80004248:	8fc40050 	lw	a0,80(s8)
8000424c:	0c003737 	jal	8000dcdc <__sfvwrite_r>
80004250:	00000000 	nop
80004254:	14400004 	bnez	v0,80004268 <_fwrite_r+0x104>
80004258:	00000000 	nop
8000425c:	8fc20068 	lw	v0,104(s8)
80004260:	10000009 	b	80004288 <_fwrite_r+0x124>
80004264:	00000000 	nop
80004268:	8fc20030 	lw	v0,48(s8)
8000426c:	8fc30020 	lw	v1,32(s8)
80004270:	00621823 	subu	v1,v1,v0
80004274:	8fc20060 	lw	v0,96(s8)
80004278:	0062001b 	divu	zero,v1,v0
8000427c:	004001f4 	teq	v0,zero,0x7
80004280:	00001010 	mfhi	v0
80004284:	00001012 	mflo	v0
80004288:	03c0e825 	move	sp,s8
8000428c:	dfbf0048 	ld	ra,72(sp)
80004290:	dfbe0040 	ld	s8,64(sp)
80004294:	27bd0050 	addiu	sp,sp,80
80004298:	03e00008 	jr	ra
8000429c:	00000000 	nop

800042a0 <fwrite>:
800042a0:	27bdffc8 	addiu	sp,sp,-56
800042a4:	ffbf0030 	sd	ra,48(sp)
800042a8:	ffbe0028 	sd	s8,40(sp)
800042ac:	03a0f025 	move	s8,sp
800042b0:	afc40038 	sw	a0,56(s8)
800042b4:	afc50040 	sw	a1,64(s8)
800042b8:	afc60048 	sw	a2,72(s8)
800042bc:	afc70050 	sw	a3,80(s8)
800042c0:	8f838028 	lw	v1,-32728(gp)
800042c4:	8fc20050 	lw	v0,80(s8)
800042c8:	afa20024 	sw	v0,36(sp)
800042cc:	8fc70048 	lw	a3,72(s8)
800042d0:	8fc60040 	lw	a2,64(s8)
800042d4:	8fc50038 	lw	a1,56(s8)
800042d8:	00602025 	move	a0,v1
800042dc:	0c001059 	jal	80004164 <_fwrite_r>
800042e0:	00000000 	nop
800042e4:	03c0e825 	move	sp,s8
800042e8:	dfbf0030 	ld	ra,48(sp)
800042ec:	dfbe0028 	ld	s8,40(sp)
800042f0:	27bd0038 	addiu	sp,sp,56
800042f4:	03e00008 	jr	ra
800042f8:	00000000 	nop

800042fc <__smakebuf_r>:
800042fc:	27bdffb0 	addiu	sp,sp,-80
80004300:	ffbf0048 	sd	ra,72(sp)
80004304:	ffbe0040 	sd	s8,64(sp)
80004308:	ffb10038 	sd	s1,56(sp)
8000430c:	ffb00030 	sd	s0,48(sp)
80004310:	03a0f025 	move	s8,sp
80004314:	afc40050 	sw	a0,80(s8)
80004318:	00a08025 	move	s0,a1
8000431c:	8602000c 	lh	v0,12(s0)
80004320:	3042ffff 	andi	v0,v0,0xffff
80004324:	30420002 	andi	v0,v0,0x2
80004328:	10400009 	beqz	v0,80004350 <__smakebuf_r+0x54>
8000432c:	00000000 	nop
80004330:	26020043 	addiu	v0,s0,67
80004334:	ae020000 	sw	v0,0(s0)
80004338:	8e020000 	lw	v0,0(s0)
8000433c:	ae020010 	sw	v0,16(s0)
80004340:	24020001 	li	v0,1
80004344:	ae020014 	sw	v0,20(s0)
80004348:	10000051 	b	80004490 <__smakebuf_r+0x194>
8000434c:	00000000 	nop
80004350:	27c30028 	addiu	v1,s8,40
80004354:	27c20024 	addiu	v0,s8,36
80004358:	00603825 	move	a3,v1
8000435c:	00403025 	move	a2,v0
80004360:	02002825 	move	a1,s0
80004364:	8fc40050 	lw	a0,80(s8)
80004368:	0c00112c 	jal	800044b0 <__swhatbuf_r>
8000436c:	00000000 	nop
80004370:	afc20020 	sw	v0,32(s8)
80004374:	8fc20024 	lw	v0,36(s8)
80004378:	00402825 	move	a1,v0
8000437c:	8fc40050 	lw	a0,80(s8)
80004380:	0c001296 	jal	80004a58 <_malloc_r>
80004384:	00000000 	nop
80004388:	00408825 	move	s1,v0
8000438c:	16200017 	bnez	s1,800043ec <__smakebuf_r+0xf0>
80004390:	00000000 	nop
80004394:	8602000c 	lh	v0,12(s0)
80004398:	3042ffff 	andi	v0,v0,0xffff
8000439c:	30420200 	andi	v0,v0,0x200
800043a0:	1440003b 	bnez	v0,80004490 <__smakebuf_r+0x194>
800043a4:	00000000 	nop
800043a8:	8603000c 	lh	v1,12(s0)
800043ac:	2402fffc 	li	v0,-4
800043b0:	00621024 	and	v0,v1,v0
800043b4:	00021400 	sll	v0,v0,0x10
800043b8:	00021403 	sra	v0,v0,0x10
800043bc:	34420002 	ori	v0,v0,0x2
800043c0:	00021400 	sll	v0,v0,0x10
800043c4:	00021403 	sra	v0,v0,0x10
800043c8:	a602000c 	sh	v0,12(s0)
800043cc:	26020043 	addiu	v0,s0,67
800043d0:	ae020000 	sw	v0,0(s0)
800043d4:	8e020000 	lw	v0,0(s0)
800043d8:	ae020010 	sw	v0,16(s0)
800043dc:	24020001 	li	v0,1
800043e0:	ae020014 	sw	v0,20(s0)
800043e4:	1000002a 	b	80004490 <__smakebuf_r+0x194>
800043e8:	00000000 	nop
800043ec:	8fc20050 	lw	v0,80(s8)
800043f0:	3c038000 	lui	v1,0x8000
800043f4:	24633284 	addiu	v1,v1,12932
800043f8:	ac43003c 	sw	v1,60(v0)
800043fc:	8602000c 	lh	v0,12(s0)
80004400:	34420080 	ori	v0,v0,0x80
80004404:	00021400 	sll	v0,v0,0x10
80004408:	00021403 	sra	v0,v0,0x10
8000440c:	a602000c 	sh	v0,12(s0)
80004410:	ae110000 	sw	s1,0(s0)
80004414:	8e020000 	lw	v0,0(s0)
80004418:	ae020010 	sw	v0,16(s0)
8000441c:	8fc20024 	lw	v0,36(s8)
80004420:	ae020014 	sw	v0,20(s0)
80004424:	8fc20028 	lw	v0,40(s8)
80004428:	10400011 	beqz	v0,80004470 <__smakebuf_r+0x174>
8000442c:	00000000 	nop
80004430:	8602000e 	lh	v0,14(s0)
80004434:	00402825 	move	a1,v0
80004438:	8fc40050 	lw	a0,80(s8)
8000443c:	0c00398d 	jal	8000e634 <_isatty_r>
80004440:	00000000 	nop
80004444:	1040000a 	beqz	v0,80004470 <__smakebuf_r+0x174>
80004448:	00000000 	nop
8000444c:	8603000c 	lh	v1,12(s0)
80004450:	2402fffc 	li	v0,-4
80004454:	00621024 	and	v0,v1,v0
80004458:	00021400 	sll	v0,v0,0x10
8000445c:	00021403 	sra	v0,v0,0x10
80004460:	34420001 	ori	v0,v0,0x1
80004464:	00021400 	sll	v0,v0,0x10
80004468:	00021403 	sra	v0,v0,0x10
8000446c:	a602000c 	sh	v0,12(s0)
80004470:	8603000c 	lh	v1,12(s0)
80004474:	8fc20020 	lw	v0,32(s8)
80004478:	00021400 	sll	v0,v0,0x10
8000447c:	00021403 	sra	v0,v0,0x10
80004480:	00621025 	or	v0,v1,v0
80004484:	00021400 	sll	v0,v0,0x10
80004488:	00021403 	sra	v0,v0,0x10
8000448c:	a602000c 	sh	v0,12(s0)
80004490:	03c0e825 	move	sp,s8
80004494:	dfbf0048 	ld	ra,72(sp)
80004498:	dfbe0040 	ld	s8,64(sp)
8000449c:	dfb10038 	ld	s1,56(sp)
800044a0:	dfb00030 	ld	s0,48(sp)
800044a4:	27bd0050 	addiu	sp,sp,80
800044a8:	03e00008 	jr	ra
800044ac:	00000000 	nop

800044b0 <__swhatbuf_r>:
800044b0:	27bdff70 	addiu	sp,sp,-144
800044b4:	ffbf0088 	sd	ra,136(sp)
800044b8:	ffbe0080 	sd	s8,128(sp)
800044bc:	03a0f025 	move	s8,sp
800044c0:	afc40090 	sw	a0,144(s8)
800044c4:	afc50098 	sw	a1,152(s8)
800044c8:	afc600a0 	sw	a2,160(s8)
800044cc:	afc700a8 	sw	a3,168(s8)
800044d0:	24020800 	li	v0,2048
800044d4:	afc20020 	sw	v0,32(s8)
800044d8:	8fc20098 	lw	v0,152(s8)
800044dc:	8442000e 	lh	v0,14(v0)
800044e0:	0440000b 	bltz	v0,80004510 <__swhatbuf_r+0x60>
800044e4:	00000000 	nop
800044e8:	8fc20098 	lw	v0,152(s8)
800044ec:	8442000e 	lh	v0,14(v0)
800044f0:	27c30028 	addiu	v1,s8,40
800044f4:	00603025 	move	a2,v1
800044f8:	00402825 	move	a1,v0
800044fc:	8fc40090 	lw	a0,144(s8)
80004500:	0c000fce 	jal	80003f38 <_fstat_r>
80004504:	00000000 	nop
80004508:	04410014 	bgez	v0,8000455c <__swhatbuf_r+0xac>
8000450c:	00000000 	nop
80004510:	8fc200a8 	lw	v0,168(s8)
80004514:	ac400000 	sw	zero,0(v0)
80004518:	8fc20098 	lw	v0,152(s8)
8000451c:	8442000c 	lh	v0,12(v0)
80004520:	3042ffff 	andi	v0,v0,0xffff
80004524:	30420080 	andi	v0,v0,0x80
80004528:	10400006 	beqz	v0,80004544 <__swhatbuf_r+0x94>
8000452c:	00000000 	nop
80004530:	8fc200a0 	lw	v0,160(s8)
80004534:	24030040 	li	v1,64
80004538:	ac430000 	sw	v1,0(v0)
8000453c:	10000004 	b	80004550 <__swhatbuf_r+0xa0>
80004540:	00000000 	nop
80004544:	8fc200a0 	lw	v0,160(s8)
80004548:	24030400 	li	v1,1024
8000454c:	ac430000 	sw	v1,0(v0)
80004550:	00001025 	move	v0,zero
80004554:	1000000d 	b	8000458c <__swhatbuf_r+0xdc>
80004558:	00000000 	nop
8000455c:	8fc2002c 	lw	v0,44(s8)
80004560:	3042f000 	andi	v0,v0,0xf000
80004564:	38422000 	xori	v0,v0,0x2000
80004568:	2c420001 	sltiu	v0,v0,1
8000456c:	304200ff 	andi	v0,v0,0xff
80004570:	00401825 	move	v1,v0
80004574:	8fc200a8 	lw	v0,168(s8)
80004578:	ac430000 	sw	v1,0(v0)
8000457c:	8fc200a0 	lw	v0,160(s8)
80004580:	24030400 	li	v1,1024
80004584:	ac430000 	sw	v1,0(v0)
80004588:	8fc20020 	lw	v0,32(s8)
8000458c:	03c0e825 	move	sp,s8
80004590:	dfbf0088 	ld	ra,136(sp)
80004594:	dfbe0080 	ld	s8,128(sp)
80004598:	27bd0090 	addiu	sp,sp,144
8000459c:	03e00008 	jr	ra
800045a0:	00000000 	nop

800045a4 <malloc>:
800045a4:	27bdffd0 	addiu	sp,sp,-48
800045a8:	ffbf0028 	sd	ra,40(sp)
800045ac:	ffbe0020 	sd	s8,32(sp)
800045b0:	03a0f025 	move	s8,sp
800045b4:	afc40030 	sw	a0,48(s8)
800045b8:	8f828028 	lw	v0,-32728(gp)
800045bc:	8fc50030 	lw	a1,48(s8)
800045c0:	00402025 	move	a0,v0
800045c4:	0c001296 	jal	80004a58 <_malloc_r>
800045c8:	00000000 	nop
800045cc:	03c0e825 	move	sp,s8
800045d0:	dfbf0028 	ld	ra,40(sp)
800045d4:	dfbe0020 	ld	s8,32(sp)
800045d8:	27bd0030 	addiu	sp,sp,48
800045dc:	03e00008 	jr	ra
800045e0:	00000000 	nop

800045e4 <free>:
800045e4:	27bdffd0 	addiu	sp,sp,-48
800045e8:	ffbf0028 	sd	ra,40(sp)
800045ec:	ffbe0020 	sd	s8,32(sp)
800045f0:	03a0f025 	move	s8,sp
800045f4:	afc40030 	sw	a0,48(s8)
800045f8:	8f828028 	lw	v0,-32728(gp)
800045fc:	8fc50030 	lw	a1,48(s8)
80004600:	00402025 	move	a0,v0
80004604:	0c000dcb 	jal	8000372c <_free_r>
80004608:	00000000 	nop
8000460c:	00000000 	nop
80004610:	03c0e825 	move	sp,s8
80004614:	dfbf0028 	ld	ra,40(sp)
80004618:	dfbe0020 	ld	s8,32(sp)
8000461c:	27bd0030 	addiu	sp,sp,48
80004620:	03e00008 	jr	ra
80004624:	00000000 	nop

80004628 <malloc_extend_top>:
80004628:	27bdffa0 	addiu	sp,sp,-96
8000462c:	ffbf0058 	sd	ra,88(sp)
80004630:	ffbe0050 	sd	s8,80(sp)
80004634:	03a0f025 	move	s8,sp
80004638:	afc40060 	sw	a0,96(s8)
8000463c:	afc50068 	sw	a1,104(s8)
80004640:	afc00030 	sw	zero,48(s8)
80004644:	3c028002 	lui	v0,0x8002
80004648:	2442ce98 	addiu	v0,v0,-12648
8000464c:	8c420008 	lw	v0,8(v0)
80004650:	afc20034 	sw	v0,52(s8)
80004654:	8fc20034 	lw	v0,52(s8)
80004658:	8c430004 	lw	v1,4(v0)
8000465c:	2402fffc 	li	v0,-4
80004660:	00621024 	and	v0,v1,v0
80004664:	afc20038 	sw	v0,56(s8)
80004668:	8fc30034 	lw	v1,52(s8)
8000466c:	8fc20038 	lw	v0,56(s8)
80004670:	00621021 	addu	v0,v1,v0
80004674:	afc2003c 	sw	v0,60(s8)
80004678:	8f838328 	lw	v1,-31960(gp)
8000467c:	8fc20068 	lw	v0,104(s8)
80004680:	00621021 	addu	v0,v1,v0
80004684:	24420010 	addiu	v0,v0,16
80004688:	afc2002c 	sw	v0,44(s8)
8000468c:	24021000 	li	v0,4096
80004690:	afc20040 	sw	v0,64(s8)
80004694:	8f838034 	lw	v1,-32716(gp)
80004698:	2402ffff 	li	v0,-1
8000469c:	10620009 	beq	v1,v0,800046c4 <malloc_extend_top+0x9c>
800046a0:	00000000 	nop
800046a4:	8fc30040 	lw	v1,64(s8)
800046a8:	8fc2002c 	lw	v0,44(s8)
800046ac:	00621021 	addu	v0,v1,v0
800046b0:	2443ffff 	addiu	v1,v0,-1
800046b4:	8fc20040 	lw	v0,64(s8)
800046b8:	00021023 	negu	v0,v0
800046bc:	00621024 	and	v0,v1,v0
800046c0:	afc2002c 	sw	v0,44(s8)
800046c4:	8fc2002c 	lw	v0,44(s8)
800046c8:	00402825 	move	a1,v0
800046cc:	8fc40060 	lw	a0,96(s8)
800046d0:	0c001cf2 	jal	800073c8 <_sbrk_r>
800046d4:	00000000 	nop
800046d8:	afc20020 	sw	v0,32(s8)
800046dc:	8fc30020 	lw	v1,32(s8)
800046e0:	2402ffff 	li	v0,-1
800046e4:	106200d5 	beq	v1,v0,80004a3c <malloc_extend_top+0x414>
800046e8:	00000000 	nop
800046ec:	8fc30020 	lw	v1,32(s8)
800046f0:	8fc2003c 	lw	v0,60(s8)
800046f4:	0062102b 	sltu	v0,v1,v0
800046f8:	10400006 	beqz	v0,80004714 <malloc_extend_top+0xec>
800046fc:	00000000 	nop
80004700:	3c028002 	lui	v0,0x8002
80004704:	2442ce98 	addiu	v0,v0,-12648
80004708:	8fc30034 	lw	v1,52(s8)
8000470c:	146200cb 	bne	v1,v0,80004a3c <malloc_extend_top+0x414>
80004710:	00000000 	nop
80004714:	3c028003 	lui	v0,0x8003
80004718:	8c42dd00 	lw	v0,-8960(v0)
8000471c:	00401825 	move	v1,v0
80004720:	8fc2002c 	lw	v0,44(s8)
80004724:	00621021 	addu	v0,v1,v0
80004728:	00401825 	move	v1,v0
8000472c:	3c028003 	lui	v0,0x8003
80004730:	ac43dd00 	sw	v1,-8960(v0)
80004734:	8fc30020 	lw	v1,32(s8)
80004738:	8fc2003c 	lw	v0,60(s8)
8000473c:	14620013 	bne	v1,v0,8000478c <malloc_extend_top+0x164>
80004740:	00000000 	nop
80004744:	8fc20040 	lw	v0,64(s8)
80004748:	2443ffff 	addiu	v1,v0,-1
8000474c:	8fc2003c 	lw	v0,60(s8)
80004750:	00621024 	and	v0,v1,v0
80004754:	1440000d 	bnez	v0,8000478c <malloc_extend_top+0x164>
80004758:	00000000 	nop
8000475c:	8fc3002c 	lw	v1,44(s8)
80004760:	8fc20038 	lw	v0,56(s8)
80004764:	00621021 	addu	v0,v1,v0
80004768:	afc20044 	sw	v0,68(s8)
8000476c:	3c028002 	lui	v0,0x8002
80004770:	2442ce98 	addiu	v0,v0,-12648
80004774:	8c420008 	lw	v0,8(v0)
80004778:	8fc30044 	lw	v1,68(s8)
8000477c:	34630001 	ori	v1,v1,0x1
80004780:	ac430004 	sw	v1,4(v0)
80004784:	10000097 	b	800049e4 <malloc_extend_top+0x3bc>
80004788:	00000000 	nop
8000478c:	8f838034 	lw	v1,-32716(gp)
80004790:	2402ffff 	li	v0,-1
80004794:	14620005 	bne	v1,v0,800047ac <malloc_extend_top+0x184>
80004798:	00000000 	nop
8000479c:	8fc20020 	lw	v0,32(s8)
800047a0:	af828034 	sw	v0,-32716(gp)
800047a4:	10000009 	b	800047cc <malloc_extend_top+0x1a4>
800047a8:	00000000 	nop
800047ac:	3c028003 	lui	v0,0x8003
800047b0:	8c43dd00 	lw	v1,-8960(v0)
800047b4:	8fc40020 	lw	a0,32(s8)
800047b8:	8fc2003c 	lw	v0,60(s8)
800047bc:	00821023 	subu	v0,a0,v0
800047c0:	00621821 	addu	v1,v1,v0
800047c4:	3c028003 	lui	v0,0x8003
800047c8:	ac43dd00 	sw	v1,-8960(v0)
800047cc:	8fc20020 	lw	v0,32(s8)
800047d0:	24420008 	addiu	v0,v0,8
800047d4:	30420007 	andi	v0,v0,0x7
800047d8:	afc20048 	sw	v0,72(s8)
800047dc:	8fc20048 	lw	v0,72(s8)
800047e0:	1040000b 	beqz	v0,80004810 <malloc_extend_top+0x1e8>
800047e4:	00000000 	nop
800047e8:	24030008 	li	v1,8
800047ec:	8fc20048 	lw	v0,72(s8)
800047f0:	00621023 	subu	v0,v1,v0
800047f4:	afc20024 	sw	v0,36(s8)
800047f8:	8fc30020 	lw	v1,32(s8)
800047fc:	8fc20024 	lw	v0,36(s8)
80004800:	00621021 	addu	v0,v1,v0
80004804:	afc20020 	sw	v0,32(s8)
80004808:	10000002 	b	80004814 <malloc_extend_top+0x1ec>
8000480c:	00000000 	nop
80004810:	afc00024 	sw	zero,36(s8)
80004814:	8fc30020 	lw	v1,32(s8)
80004818:	8fc2002c 	lw	v0,44(s8)
8000481c:	00621021 	addu	v0,v1,v0
80004820:	00401825 	move	v1,v0
80004824:	8fc20040 	lw	v0,64(s8)
80004828:	2442ffff 	addiu	v0,v0,-1
8000482c:	00621024 	and	v0,v1,v0
80004830:	8fc30040 	lw	v1,64(s8)
80004834:	00621023 	subu	v0,v1,v0
80004838:	8fc30024 	lw	v1,36(s8)
8000483c:	00621021 	addu	v0,v1,v0
80004840:	afc20024 	sw	v0,36(s8)
80004844:	8fc20040 	lw	v0,64(s8)
80004848:	2442ffff 	addiu	v0,v0,-1
8000484c:	8fc30024 	lw	v1,36(s8)
80004850:	00621024 	and	v0,v1,v0
80004854:	afc20024 	sw	v0,36(s8)
80004858:	8fc20024 	lw	v0,36(s8)
8000485c:	00402825 	move	a1,v0
80004860:	8fc40060 	lw	a0,96(s8)
80004864:	0c001cf2 	jal	800073c8 <_sbrk_r>
80004868:	00000000 	nop
8000486c:	afc20028 	sw	v0,40(s8)
80004870:	8fc30028 	lw	v1,40(s8)
80004874:	2402ffff 	li	v0,-1
80004878:	14620010 	bne	v1,v0,800048bc <malloc_extend_top+0x294>
8000487c:	00000000 	nop
80004880:	afc00024 	sw	zero,36(s8)
80004884:	24020001 	li	v0,1
80004888:	afc20030 	sw	v0,48(s8)
8000488c:	8fc30020 	lw	v1,32(s8)
80004890:	8fc2002c 	lw	v0,44(s8)
80004894:	00621021 	addu	v0,v1,v0
80004898:	afc20028 	sw	v0,40(s8)
8000489c:	8fc20048 	lw	v0,72(s8)
800048a0:	10400006 	beqz	v0,800048bc <malloc_extend_top+0x294>
800048a4:	00000000 	nop
800048a8:	8fc20048 	lw	v0,72(s8)
800048ac:	2442fff8 	addiu	v0,v0,-8
800048b0:	8fc30028 	lw	v1,40(s8)
800048b4:	00621021 	addu	v0,v1,v0
800048b8:	afc20028 	sw	v0,40(s8)
800048bc:	3c028003 	lui	v0,0x8003
800048c0:	8c42dd00 	lw	v0,-8960(v0)
800048c4:	00401825 	move	v1,v0
800048c8:	8fc20024 	lw	v0,36(s8)
800048cc:	00621021 	addu	v0,v1,v0
800048d0:	00401825 	move	v1,v0
800048d4:	3c028003 	lui	v0,0x8003
800048d8:	ac43dd00 	sw	v1,-8960(v0)
800048dc:	3c028002 	lui	v0,0x8002
800048e0:	2442ce98 	addiu	v0,v0,-12648
800048e4:	8fc30020 	lw	v1,32(s8)
800048e8:	ac430008 	sw	v1,8(v0)
800048ec:	8fc30028 	lw	v1,40(s8)
800048f0:	8fc20020 	lw	v0,32(s8)
800048f4:	00621023 	subu	v0,v1,v0
800048f8:	00401825 	move	v1,v0
800048fc:	8fc20024 	lw	v0,36(s8)
80004900:	00431021 	addu	v0,v0,v1
80004904:	afc20044 	sw	v0,68(s8)
80004908:	3c028002 	lui	v0,0x8002
8000490c:	2442ce98 	addiu	v0,v0,-12648
80004910:	8c420008 	lw	v0,8(v0)
80004914:	8fc30044 	lw	v1,68(s8)
80004918:	34630001 	ori	v1,v1,0x1
8000491c:	ac430004 	sw	v1,4(v0)
80004920:	3c028002 	lui	v0,0x8002
80004924:	2442ce98 	addiu	v0,v0,-12648
80004928:	8fc30034 	lw	v1,52(s8)
8000492c:	1062002d 	beq	v1,v0,800049e4 <malloc_extend_top+0x3bc>
80004930:	00000000 	nop
80004934:	8fc20038 	lw	v0,56(s8)
80004938:	2c420010 	sltiu	v0,v0,16
8000493c:	10400008 	beqz	v0,80004960 <malloc_extend_top+0x338>
80004940:	00000000 	nop
80004944:	3c028002 	lui	v0,0x8002
80004948:	2442ce98 	addiu	v0,v0,-12648
8000494c:	8c420008 	lw	v0,8(v0)
80004950:	24030001 	li	v1,1
80004954:	ac430004 	sw	v1,4(v0)
80004958:	10000039 	b	80004a40 <malloc_extend_top+0x418>
8000495c:	00000000 	nop
80004960:	8fc20038 	lw	v0,56(s8)
80004964:	2443fff4 	addiu	v1,v0,-12
80004968:	2402fff8 	li	v0,-8
8000496c:	00621024 	and	v0,v1,v0
80004970:	afc20038 	sw	v0,56(s8)
80004974:	8fc20034 	lw	v0,52(s8)
80004978:	8c420004 	lw	v0,4(v0)
8000497c:	30430001 	andi	v1,v0,0x1
80004980:	8fc20038 	lw	v0,56(s8)
80004984:	00621825 	or	v1,v1,v0
80004988:	8fc20034 	lw	v0,52(s8)
8000498c:	ac430004 	sw	v1,4(v0)
80004990:	8fc30034 	lw	v1,52(s8)
80004994:	8fc20038 	lw	v0,56(s8)
80004998:	00621021 	addu	v0,v1,v0
8000499c:	24030005 	li	v1,5
800049a0:	ac430004 	sw	v1,4(v0)
800049a4:	8fc20038 	lw	v0,56(s8)
800049a8:	24420004 	addiu	v0,v0,4
800049ac:	8fc30034 	lw	v1,52(s8)
800049b0:	00621021 	addu	v0,v1,v0
800049b4:	24030005 	li	v1,5
800049b8:	ac430004 	sw	v1,4(v0)
800049bc:	8fc20038 	lw	v0,56(s8)
800049c0:	2c420010 	sltiu	v0,v0,16
800049c4:	14400007 	bnez	v0,800049e4 <malloc_extend_top+0x3bc>
800049c8:	00000000 	nop
800049cc:	8fc20034 	lw	v0,52(s8)
800049d0:	24420008 	addiu	v0,v0,8
800049d4:	00402825 	move	a1,v0
800049d8:	8fc40060 	lw	a0,96(s8)
800049dc:	0c000dcb 	jal	8000372c <_free_r>
800049e0:	00000000 	nop
800049e4:	3c028003 	lui	v0,0x8003
800049e8:	8c42dd00 	lw	v0,-8960(v0)
800049ec:	00401825 	move	v1,v0
800049f0:	8f82832c 	lw	v0,-31956(gp)
800049f4:	0043102b 	sltu	v0,v0,v1
800049f8:	10400004 	beqz	v0,80004a0c <malloc_extend_top+0x3e4>
800049fc:	00000000 	nop
80004a00:	3c028003 	lui	v0,0x8003
80004a04:	8c42dd00 	lw	v0,-8960(v0)
80004a08:	af82832c 	sw	v0,-31956(gp)
80004a0c:	3c028003 	lui	v0,0x8003
80004a10:	8c42dd00 	lw	v0,-8960(v0)
80004a14:	00401825 	move	v1,v0
80004a18:	8f828330 	lw	v0,-31952(gp)
80004a1c:	0043102b 	sltu	v0,v0,v1
80004a20:	10400007 	beqz	v0,80004a40 <malloc_extend_top+0x418>
80004a24:	00000000 	nop
80004a28:	3c028003 	lui	v0,0x8003
80004a2c:	8c42dd00 	lw	v0,-8960(v0)
80004a30:	af828330 	sw	v0,-31952(gp)
80004a34:	10000002 	b	80004a40 <malloc_extend_top+0x418>
80004a38:	00000000 	nop
80004a3c:	00000000 	nop
80004a40:	03c0e825 	move	sp,s8
80004a44:	dfbf0058 	ld	ra,88(sp)
80004a48:	dfbe0050 	ld	s8,80(sp)
80004a4c:	27bd0060 	addiu	sp,sp,96
80004a50:	03e00008 	jr	ra
80004a54:	00000000 	nop

80004a58 <_malloc_r>:
80004a58:	27bdff98 	addiu	sp,sp,-104
80004a5c:	ffbf0060 	sd	ra,96(sp)
80004a60:	ffbe0058 	sd	s8,88(sp)
80004a64:	03a0f025 	move	s8,sp
80004a68:	afc40068 	sw	a0,104(s8)
80004a6c:	afc50070 	sw	a1,112(s8)
80004a70:	8fc20070 	lw	v0,112(s8)
80004a74:	2442000b 	addiu	v0,v0,11
80004a78:	2c420017 	sltiu	v0,v0,23
80004a7c:	14400007 	bnez	v0,80004a9c <_malloc_r+0x44>
80004a80:	00000000 	nop
80004a84:	8fc20070 	lw	v0,112(s8)
80004a88:	2443000b 	addiu	v1,v0,11
80004a8c:	2402fff8 	li	v0,-8
80004a90:	00621024 	and	v0,v1,v0
80004a94:	10000002 	b	80004aa0 <_malloc_r+0x48>
80004a98:	00000000 	nop
80004a9c:	24020010 	li	v0,16
80004aa0:	afc20044 	sw	v0,68(s8)
80004aa4:	8fc20044 	lw	v0,68(s8)
80004aa8:	04400006 	bltz	v0,80004ac4 <_malloc_r+0x6c>
80004aac:	00000000 	nop
80004ab0:	8fc30044 	lw	v1,68(s8)
80004ab4:	8fc20070 	lw	v0,112(s8)
80004ab8:	0062102b 	sltu	v0,v1,v0
80004abc:	10400007 	beqz	v0,80004adc <_malloc_r+0x84>
80004ac0:	00000000 	nop
80004ac4:	8fc20068 	lw	v0,104(s8)
80004ac8:	2403000c 	li	v1,12
80004acc:	ac430000 	sw	v1,0(v0)
80004ad0:	00001025 	move	v0,zero
80004ad4:	10000313 	b	80005724 <_malloc_r+0xccc>
80004ad8:	00000000 	nop
80004adc:	8fc40068 	lw	a0,104(s8)
80004ae0:	0c001ba7 	jal	80006e9c <__malloc_lock>
80004ae4:	00000000 	nop
80004ae8:	8fc20044 	lw	v0,68(s8)
80004aec:	2c4201f8 	sltiu	v0,v0,504
80004af0:	10400044 	beqz	v0,80004c04 <_malloc_r+0x1ac>
80004af4:	00000000 	nop
80004af8:	8fc20044 	lw	v0,68(s8)
80004afc:	000210c2 	srl	v0,v0,0x3
80004b00:	afc20024 	sw	v0,36(s8)
80004b04:	8fc20024 	lw	v0,36(s8)
80004b08:	24420001 	addiu	v0,v0,1
80004b0c:	00021040 	sll	v0,v0,0x1
80004b10:	00021880 	sll	v1,v0,0x2
80004b14:	3c028002 	lui	v0,0x8002
80004b18:	2442ce98 	addiu	v0,v0,-12648
80004b1c:	00621021 	addu	v0,v1,v0
80004b20:	2442fff8 	addiu	v0,v0,-8
80004b24:	afc20040 	sw	v0,64(s8)
80004b28:	8fc20040 	lw	v0,64(s8)
80004b2c:	8c42000c 	lw	v0,12(v0)
80004b30:	afc20020 	sw	v0,32(s8)
80004b34:	8fc30020 	lw	v1,32(s8)
80004b38:	8fc20040 	lw	v0,64(s8)
80004b3c:	14620007 	bne	v1,v0,80004b5c <_malloc_r+0x104>
80004b40:	00000000 	nop
80004b44:	8fc20040 	lw	v0,64(s8)
80004b48:	24420008 	addiu	v0,v0,8
80004b4c:	afc20040 	sw	v0,64(s8)
80004b50:	8fc20040 	lw	v0,64(s8)
80004b54:	8c42000c 	lw	v0,12(v0)
80004b58:	afc20020 	sw	v0,32(s8)
80004b5c:	8fc30020 	lw	v1,32(s8)
80004b60:	8fc20040 	lw	v0,64(s8)
80004b64:	10620022 	beq	v1,v0,80004bf0 <_malloc_r+0x198>
80004b68:	00000000 	nop
80004b6c:	8fc20020 	lw	v0,32(s8)
80004b70:	8c430004 	lw	v1,4(v0)
80004b74:	2402fffc 	li	v0,-4
80004b78:	00621024 	and	v0,v1,v0
80004b7c:	afc20048 	sw	v0,72(s8)
80004b80:	8fc20020 	lw	v0,32(s8)
80004b84:	8c42000c 	lw	v0,12(v0)
80004b88:	afc2003c 	sw	v0,60(s8)
80004b8c:	8fc20020 	lw	v0,32(s8)
80004b90:	8c420008 	lw	v0,8(v0)
80004b94:	afc20038 	sw	v0,56(s8)
80004b98:	8fc20038 	lw	v0,56(s8)
80004b9c:	8fc3003c 	lw	v1,60(s8)
80004ba0:	ac43000c 	sw	v1,12(v0)
80004ba4:	8fc2003c 	lw	v0,60(s8)
80004ba8:	8fc30038 	lw	v1,56(s8)
80004bac:	ac430008 	sw	v1,8(v0)
80004bb0:	8fc30020 	lw	v1,32(s8)
80004bb4:	8fc20048 	lw	v0,72(s8)
80004bb8:	00621021 	addu	v0,v1,v0
80004bbc:	8c430004 	lw	v1,4(v0)
80004bc0:	8fc40020 	lw	a0,32(s8)
80004bc4:	8fc20048 	lw	v0,72(s8)
80004bc8:	00821021 	addu	v0,a0,v0
80004bcc:	34630001 	ori	v1,v1,0x1
80004bd0:	ac430004 	sw	v1,4(v0)
80004bd4:	8fc40068 	lw	a0,104(s8)
80004bd8:	0c001bb1 	jal	80006ec4 <__malloc_unlock>
80004bdc:	00000000 	nop
80004be0:	8fc20020 	lw	v0,32(s8)
80004be4:	24420008 	addiu	v0,v0,8
80004be8:	100002ce 	b	80005724 <_malloc_r+0xccc>
80004bec:	00000000 	nop
80004bf0:	8fc20024 	lw	v0,36(s8)
80004bf4:	24420002 	addiu	v0,v0,2
80004bf8:	afc20024 	sw	v0,36(s8)
80004bfc:	10000086 	b	80004e18 <_malloc_r+0x3c0>
80004c00:	00000000 	nop
80004c04:	8fc20044 	lw	v0,68(s8)
80004c08:	00021242 	srl	v0,v0,0x9
80004c0c:	14400005 	bnez	v0,80004c24 <_malloc_r+0x1cc>
80004c10:	00000000 	nop
80004c14:	8fc20044 	lw	v0,68(s8)
80004c18:	000210c2 	srl	v0,v0,0x3
80004c1c:	10000034 	b	80004cf0 <_malloc_r+0x298>
80004c20:	00000000 	nop
80004c24:	8fc20044 	lw	v0,68(s8)
80004c28:	00021242 	srl	v0,v0,0x9
80004c2c:	2c420005 	sltiu	v0,v0,5
80004c30:	10400006 	beqz	v0,80004c4c <_malloc_r+0x1f4>
80004c34:	00000000 	nop
80004c38:	8fc20044 	lw	v0,68(s8)
80004c3c:	00021182 	srl	v0,v0,0x6
80004c40:	24420038 	addiu	v0,v0,56
80004c44:	1000002a 	b	80004cf0 <_malloc_r+0x298>
80004c48:	00000000 	nop
80004c4c:	8fc20044 	lw	v0,68(s8)
80004c50:	00021242 	srl	v0,v0,0x9
80004c54:	2c420015 	sltiu	v0,v0,21
80004c58:	10400006 	beqz	v0,80004c74 <_malloc_r+0x21c>
80004c5c:	00000000 	nop
80004c60:	8fc20044 	lw	v0,68(s8)
80004c64:	00021242 	srl	v0,v0,0x9
80004c68:	2442005b 	addiu	v0,v0,91
80004c6c:	10000020 	b	80004cf0 <_malloc_r+0x298>
80004c70:	00000000 	nop
80004c74:	8fc20044 	lw	v0,68(s8)
80004c78:	00021242 	srl	v0,v0,0x9
80004c7c:	2c420055 	sltiu	v0,v0,85
80004c80:	10400006 	beqz	v0,80004c9c <_malloc_r+0x244>
80004c84:	00000000 	nop
80004c88:	8fc20044 	lw	v0,68(s8)
80004c8c:	00021302 	srl	v0,v0,0xc
80004c90:	2442006e 	addiu	v0,v0,110
80004c94:	10000016 	b	80004cf0 <_malloc_r+0x298>
80004c98:	00000000 	nop
80004c9c:	8fc20044 	lw	v0,68(s8)
80004ca0:	00021242 	srl	v0,v0,0x9
80004ca4:	2c420155 	sltiu	v0,v0,341
80004ca8:	10400006 	beqz	v0,80004cc4 <_malloc_r+0x26c>
80004cac:	00000000 	nop
80004cb0:	8fc20044 	lw	v0,68(s8)
80004cb4:	000213c2 	srl	v0,v0,0xf
80004cb8:	24420077 	addiu	v0,v0,119
80004cbc:	1000000c 	b	80004cf0 <_malloc_r+0x298>
80004cc0:	00000000 	nop
80004cc4:	8fc20044 	lw	v0,68(s8)
80004cc8:	00021242 	srl	v0,v0,0x9
80004ccc:	2c420555 	sltiu	v0,v0,1365
80004cd0:	10400006 	beqz	v0,80004cec <_malloc_r+0x294>
80004cd4:	00000000 	nop
80004cd8:	8fc20044 	lw	v0,68(s8)
80004cdc:	00021482 	srl	v0,v0,0x12
80004ce0:	2442007c 	addiu	v0,v0,124
80004ce4:	10000002 	b	80004cf0 <_malloc_r+0x298>
80004ce8:	00000000 	nop
80004cec:	2402007e 	li	v0,126
80004cf0:	afc20024 	sw	v0,36(s8)
80004cf4:	8fc20024 	lw	v0,36(s8)
80004cf8:	24420001 	addiu	v0,v0,1
80004cfc:	00021040 	sll	v0,v0,0x1
80004d00:	00021880 	sll	v1,v0,0x2
80004d04:	3c028002 	lui	v0,0x8002
80004d08:	2442ce98 	addiu	v0,v0,-12648
80004d0c:	00621021 	addu	v0,v1,v0
80004d10:	2442fff8 	addiu	v0,v0,-8
80004d14:	afc20028 	sw	v0,40(s8)
80004d18:	8fc20028 	lw	v0,40(s8)
80004d1c:	8c42000c 	lw	v0,12(v0)
80004d20:	afc20020 	sw	v0,32(s8)
80004d24:	10000035 	b	80004dfc <_malloc_r+0x3a4>
80004d28:	00000000 	nop
80004d2c:	8fc20020 	lw	v0,32(s8)
80004d30:	8c430004 	lw	v1,4(v0)
80004d34:	2402fffc 	li	v0,-4
80004d38:	00621024 	and	v0,v1,v0
80004d3c:	afc20048 	sw	v0,72(s8)
80004d40:	8fc30048 	lw	v1,72(s8)
80004d44:	8fc20044 	lw	v0,68(s8)
80004d48:	00621023 	subu	v0,v1,v0
80004d4c:	afc2002c 	sw	v0,44(s8)
80004d50:	8fc2002c 	lw	v0,44(s8)
80004d54:	28420010 	slti	v0,v0,16
80004d58:	14400006 	bnez	v0,80004d74 <_malloc_r+0x31c>
80004d5c:	00000000 	nop
80004d60:	8fc20024 	lw	v0,36(s8)
80004d64:	2442ffff 	addiu	v0,v0,-1
80004d68:	afc20024 	sw	v0,36(s8)
80004d6c:	10000027 	b	80004e0c <_malloc_r+0x3b4>
80004d70:	00000000 	nop
80004d74:	8fc2002c 	lw	v0,44(s8)
80004d78:	0440001d 	bltz	v0,80004df0 <_malloc_r+0x398>
80004d7c:	00000000 	nop
80004d80:	8fc20020 	lw	v0,32(s8)
80004d84:	8c42000c 	lw	v0,12(v0)
80004d88:	afc2003c 	sw	v0,60(s8)
80004d8c:	8fc20020 	lw	v0,32(s8)
80004d90:	8c420008 	lw	v0,8(v0)
80004d94:	afc20038 	sw	v0,56(s8)
80004d98:	8fc20038 	lw	v0,56(s8)
80004d9c:	8fc3003c 	lw	v1,60(s8)
80004da0:	ac43000c 	sw	v1,12(v0)
80004da4:	8fc2003c 	lw	v0,60(s8)
80004da8:	8fc30038 	lw	v1,56(s8)
80004dac:	ac430008 	sw	v1,8(v0)
80004db0:	8fc30020 	lw	v1,32(s8)
80004db4:	8fc20048 	lw	v0,72(s8)
80004db8:	00621021 	addu	v0,v1,v0
80004dbc:	8c430004 	lw	v1,4(v0)
80004dc0:	8fc40020 	lw	a0,32(s8)
80004dc4:	8fc20048 	lw	v0,72(s8)
80004dc8:	00821021 	addu	v0,a0,v0
80004dcc:	34630001 	ori	v1,v1,0x1
80004dd0:	ac430004 	sw	v1,4(v0)
80004dd4:	8fc40068 	lw	a0,104(s8)
80004dd8:	0c001bb1 	jal	80006ec4 <__malloc_unlock>
80004ddc:	00000000 	nop
80004de0:	8fc20020 	lw	v0,32(s8)
80004de4:	24420008 	addiu	v0,v0,8
80004de8:	1000024e 	b	80005724 <_malloc_r+0xccc>
80004dec:	00000000 	nop
80004df0:	8fc20020 	lw	v0,32(s8)
80004df4:	8c42000c 	lw	v0,12(v0)
80004df8:	afc20020 	sw	v0,32(s8)
80004dfc:	8fc30020 	lw	v1,32(s8)
80004e00:	8fc20028 	lw	v0,40(s8)
80004e04:	1462ffc9 	bne	v1,v0,80004d2c <_malloc_r+0x2d4>
80004e08:	00000000 	nop
80004e0c:	8fc20024 	lw	v0,36(s8)
80004e10:	24420001 	addiu	v0,v0,1
80004e14:	afc20024 	sw	v0,36(s8)
80004e18:	3c028002 	lui	v0,0x8002
80004e1c:	2442cea0 	addiu	v0,v0,-12640
80004e20:	8c420008 	lw	v0,8(v0)
80004e24:	afc20020 	sw	v0,32(s8)
80004e28:	3c028002 	lui	v0,0x8002
80004e2c:	2442cea0 	addiu	v0,v0,-12640
80004e30:	8fc30020 	lw	v1,32(s8)
80004e34:	10620100 	beq	v1,v0,80005238 <_malloc_r+0x7e0>
80004e38:	00000000 	nop
80004e3c:	8fc20020 	lw	v0,32(s8)
80004e40:	8c430004 	lw	v1,4(v0)
80004e44:	2402fffc 	li	v0,-4
80004e48:	00621024 	and	v0,v1,v0
80004e4c:	afc20048 	sw	v0,72(s8)
80004e50:	8fc30048 	lw	v1,72(s8)
80004e54:	8fc20044 	lw	v0,68(s8)
80004e58:	00621023 	subu	v0,v1,v0
80004e5c:	afc2002c 	sw	v0,44(s8)
80004e60:	8fc2002c 	lw	v0,44(s8)
80004e64:	28420010 	slti	v0,v0,16
80004e68:	1440002a 	bnez	v0,80004f14 <_malloc_r+0x4bc>
80004e6c:	00000000 	nop
80004e70:	8fc30020 	lw	v1,32(s8)
80004e74:	8fc20044 	lw	v0,68(s8)
80004e78:	00621021 	addu	v0,v1,v0
80004e7c:	afc20050 	sw	v0,80(s8)
80004e80:	8fc20044 	lw	v0,68(s8)
80004e84:	34430001 	ori	v1,v0,0x1
80004e88:	8fc20020 	lw	v0,32(s8)
80004e8c:	ac430004 	sw	v1,4(v0)
80004e90:	3c028002 	lui	v0,0x8002
80004e94:	2442cea0 	addiu	v0,v0,-12640
80004e98:	8fc30050 	lw	v1,80(s8)
80004e9c:	ac43000c 	sw	v1,12(v0)
80004ea0:	3c038002 	lui	v1,0x8002
80004ea4:	2463cea0 	addiu	v1,v1,-12640
80004ea8:	8c42000c 	lw	v0,12(v0)
80004eac:	ac620008 	sw	v0,8(v1)
80004eb0:	3c028002 	lui	v0,0x8002
80004eb4:	2443cea0 	addiu	v1,v0,-12640
80004eb8:	8fc20050 	lw	v0,80(s8)
80004ebc:	ac43000c 	sw	v1,12(v0)
80004ec0:	8fc20050 	lw	v0,80(s8)
80004ec4:	8c43000c 	lw	v1,12(v0)
80004ec8:	8fc20050 	lw	v0,80(s8)
80004ecc:	ac430008 	sw	v1,8(v0)
80004ed0:	8fc2002c 	lw	v0,44(s8)
80004ed4:	34420001 	ori	v0,v0,0x1
80004ed8:	00401825 	move	v1,v0
80004edc:	8fc20050 	lw	v0,80(s8)
80004ee0:	ac430004 	sw	v1,4(v0)
80004ee4:	8fc2002c 	lw	v0,44(s8)
80004ee8:	8fc30050 	lw	v1,80(s8)
80004eec:	00621021 	addu	v0,v1,v0
80004ef0:	8fc3002c 	lw	v1,44(s8)
80004ef4:	ac430000 	sw	v1,0(v0)
80004ef8:	8fc40068 	lw	a0,104(s8)
80004efc:	0c001bb1 	jal	80006ec4 <__malloc_unlock>
80004f00:	00000000 	nop
80004f04:	8fc20020 	lw	v0,32(s8)
80004f08:	24420008 	addiu	v0,v0,8
80004f0c:	10000205 	b	80005724 <_malloc_r+0xccc>
80004f10:	00000000 	nop
80004f14:	3c028002 	lui	v0,0x8002
80004f18:	2442cea0 	addiu	v0,v0,-12640
80004f1c:	3c038002 	lui	v1,0x8002
80004f20:	2463cea0 	addiu	v1,v1,-12640
80004f24:	ac43000c 	sw	v1,12(v0)
80004f28:	3c038002 	lui	v1,0x8002
80004f2c:	2463cea0 	addiu	v1,v1,-12640
80004f30:	8c42000c 	lw	v0,12(v0)
80004f34:	ac620008 	sw	v0,8(v1)
80004f38:	8fc2002c 	lw	v0,44(s8)
80004f3c:	04400011 	bltz	v0,80004f84 <_malloc_r+0x52c>
80004f40:	00000000 	nop
80004f44:	8fc30020 	lw	v1,32(s8)
80004f48:	8fc20048 	lw	v0,72(s8)
80004f4c:	00621021 	addu	v0,v1,v0
80004f50:	8c430004 	lw	v1,4(v0)
80004f54:	8fc40020 	lw	a0,32(s8)
80004f58:	8fc20048 	lw	v0,72(s8)
80004f5c:	00821021 	addu	v0,a0,v0
80004f60:	34630001 	ori	v1,v1,0x1
80004f64:	ac430004 	sw	v1,4(v0)
80004f68:	8fc40068 	lw	a0,104(s8)
80004f6c:	0c001bb1 	jal	80006ec4 <__malloc_unlock>
80004f70:	00000000 	nop
80004f74:	8fc20020 	lw	v0,32(s8)
80004f78:	24420008 	addiu	v0,v0,8
80004f7c:	100001e9 	b	80005724 <_malloc_r+0xccc>
80004f80:	00000000 	nop
80004f84:	8fc20048 	lw	v0,72(s8)
80004f88:	2c420200 	sltiu	v0,v0,512
80004f8c:	1040002e 	beqz	v0,80005048 <_malloc_r+0x5f0>
80004f90:	00000000 	nop
80004f94:	8fc20048 	lw	v0,72(s8)
80004f98:	000210c2 	srl	v0,v0,0x3
80004f9c:	afc2004c 	sw	v0,76(s8)
80004fa0:	3c028002 	lui	v0,0x8002
80004fa4:	2442ce98 	addiu	v0,v0,-12648
80004fa8:	8c440004 	lw	a0,4(v0)
80004fac:	8fc2004c 	lw	v0,76(s8)
80004fb0:	00021fc3 	sra	v1,v0,0x1f
80004fb4:	00031f82 	srl	v1,v1,0x1e
80004fb8:	00621021 	addu	v0,v1,v0
80004fbc:	00021083 	sra	v0,v0,0x2
80004fc0:	00401825 	move	v1,v0
80004fc4:	24020001 	li	v0,1
80004fc8:	00621804 	sllv	v1,v0,v1
80004fcc:	3c028002 	lui	v0,0x8002
80004fd0:	2442ce98 	addiu	v0,v0,-12648
80004fd4:	00831825 	or	v1,a0,v1
80004fd8:	ac430004 	sw	v1,4(v0)
80004fdc:	8fc2004c 	lw	v0,76(s8)
80004fe0:	24420001 	addiu	v0,v0,1
80004fe4:	00021040 	sll	v0,v0,0x1
80004fe8:	00021880 	sll	v1,v0,0x2
80004fec:	3c028002 	lui	v0,0x8002
80004ff0:	2442ce98 	addiu	v0,v0,-12648
80004ff4:	00621021 	addu	v0,v1,v0
80004ff8:	2442fff8 	addiu	v0,v0,-8
80004ffc:	afc2003c 	sw	v0,60(s8)
80005000:	8fc2003c 	lw	v0,60(s8)
80005004:	8c420008 	lw	v0,8(v0)
80005008:	afc20038 	sw	v0,56(s8)
8000500c:	8fc20020 	lw	v0,32(s8)
80005010:	8fc3003c 	lw	v1,60(s8)
80005014:	ac43000c 	sw	v1,12(v0)
80005018:	8fc20020 	lw	v0,32(s8)
8000501c:	8fc30038 	lw	v1,56(s8)
80005020:	ac430008 	sw	v1,8(v0)
80005024:	8fc2003c 	lw	v0,60(s8)
80005028:	8fc30020 	lw	v1,32(s8)
8000502c:	ac430008 	sw	v1,8(v0)
80005030:	8fc2003c 	lw	v0,60(s8)
80005034:	8c430008 	lw	v1,8(v0)
80005038:	8fc20038 	lw	v0,56(s8)
8000503c:	ac43000c 	sw	v1,12(v0)
80005040:	1000007d 	b	80005238 <_malloc_r+0x7e0>
80005044:	00000000 	nop
80005048:	8fc20048 	lw	v0,72(s8)
8000504c:	00021242 	srl	v0,v0,0x9
80005050:	14400005 	bnez	v0,80005068 <_malloc_r+0x610>
80005054:	00000000 	nop
80005058:	8fc20048 	lw	v0,72(s8)
8000505c:	000210c2 	srl	v0,v0,0x3
80005060:	10000034 	b	80005134 <_malloc_r+0x6dc>
80005064:	00000000 	nop
80005068:	8fc20048 	lw	v0,72(s8)
8000506c:	00021242 	srl	v0,v0,0x9
80005070:	2c420005 	sltiu	v0,v0,5
80005074:	10400006 	beqz	v0,80005090 <_malloc_r+0x638>
80005078:	00000000 	nop
8000507c:	8fc20048 	lw	v0,72(s8)
80005080:	00021182 	srl	v0,v0,0x6
80005084:	24420038 	addiu	v0,v0,56
80005088:	1000002a 	b	80005134 <_malloc_r+0x6dc>
8000508c:	00000000 	nop
80005090:	8fc20048 	lw	v0,72(s8)
80005094:	00021242 	srl	v0,v0,0x9
80005098:	2c420015 	sltiu	v0,v0,21
8000509c:	10400006 	beqz	v0,800050b8 <_malloc_r+0x660>
800050a0:	00000000 	nop
800050a4:	8fc20048 	lw	v0,72(s8)
800050a8:	00021242 	srl	v0,v0,0x9
800050ac:	2442005b 	addiu	v0,v0,91
800050b0:	10000020 	b	80005134 <_malloc_r+0x6dc>
800050b4:	00000000 	nop
800050b8:	8fc20048 	lw	v0,72(s8)
800050bc:	00021242 	srl	v0,v0,0x9
800050c0:	2c420055 	sltiu	v0,v0,85
800050c4:	10400006 	beqz	v0,800050e0 <_malloc_r+0x688>
800050c8:	00000000 	nop
800050cc:	8fc20048 	lw	v0,72(s8)
800050d0:	00021302 	srl	v0,v0,0xc
800050d4:	2442006e 	addiu	v0,v0,110
800050d8:	10000016 	b	80005134 <_malloc_r+0x6dc>
800050dc:	00000000 	nop
800050e0:	8fc20048 	lw	v0,72(s8)
800050e4:	00021242 	srl	v0,v0,0x9
800050e8:	2c420155 	sltiu	v0,v0,341
800050ec:	10400006 	beqz	v0,80005108 <_malloc_r+0x6b0>
800050f0:	00000000 	nop
800050f4:	8fc20048 	lw	v0,72(s8)
800050f8:	000213c2 	srl	v0,v0,0xf
800050fc:	24420077 	addiu	v0,v0,119
80005100:	1000000c 	b	80005134 <_malloc_r+0x6dc>
80005104:	00000000 	nop
80005108:	8fc20048 	lw	v0,72(s8)
8000510c:	00021242 	srl	v0,v0,0x9
80005110:	2c420555 	sltiu	v0,v0,1365
80005114:	10400006 	beqz	v0,80005130 <_malloc_r+0x6d8>
80005118:	00000000 	nop
8000511c:	8fc20048 	lw	v0,72(s8)
80005120:	00021482 	srl	v0,v0,0x12
80005124:	2442007c 	addiu	v0,v0,124
80005128:	10000002 	b	80005134 <_malloc_r+0x6dc>
8000512c:	00000000 	nop
80005130:	2402007e 	li	v0,126
80005134:	afc2004c 	sw	v0,76(s8)
80005138:	8fc2004c 	lw	v0,76(s8)
8000513c:	24420001 	addiu	v0,v0,1
80005140:	00021040 	sll	v0,v0,0x1
80005144:	00021880 	sll	v1,v0,0x2
80005148:	3c028002 	lui	v0,0x8002
8000514c:	2442ce98 	addiu	v0,v0,-12648
80005150:	00621021 	addu	v0,v1,v0
80005154:	2442fff8 	addiu	v0,v0,-8
80005158:	afc2003c 	sw	v0,60(s8)
8000515c:	8fc2003c 	lw	v0,60(s8)
80005160:	8c420008 	lw	v0,8(v0)
80005164:	afc20038 	sw	v0,56(s8)
80005168:	8fc30038 	lw	v1,56(s8)
8000516c:	8fc2003c 	lw	v0,60(s8)
80005170:	14620015 	bne	v1,v0,800051c8 <_malloc_r+0x770>
80005174:	00000000 	nop
80005178:	3c028002 	lui	v0,0x8002
8000517c:	2442ce98 	addiu	v0,v0,-12648
80005180:	8c440004 	lw	a0,4(v0)
80005184:	8fc2004c 	lw	v0,76(s8)
80005188:	00021fc3 	sra	v1,v0,0x1f
8000518c:	00031f82 	srl	v1,v1,0x1e
80005190:	00621021 	addu	v0,v1,v0
80005194:	00021083 	sra	v0,v0,0x2
80005198:	00401825 	move	v1,v0
8000519c:	24020001 	li	v0,1
800051a0:	00621804 	sllv	v1,v0,v1
800051a4:	3c028002 	lui	v0,0x8002
800051a8:	2442ce98 	addiu	v0,v0,-12648
800051ac:	00831825 	or	v1,a0,v1
800051b0:	ac430004 	sw	v1,4(v0)
800051b4:	10000013 	b	80005204 <_malloc_r+0x7ac>
800051b8:	00000000 	nop
800051bc:	8fc20038 	lw	v0,56(s8)
800051c0:	8c420008 	lw	v0,8(v0)
800051c4:	afc20038 	sw	v0,56(s8)
800051c8:	8fc30038 	lw	v1,56(s8)
800051cc:	8fc2003c 	lw	v0,60(s8)
800051d0:	10620009 	beq	v1,v0,800051f8 <_malloc_r+0x7a0>
800051d4:	00000000 	nop
800051d8:	8fc20038 	lw	v0,56(s8)
800051dc:	8c430004 	lw	v1,4(v0)
800051e0:	2402fffc 	li	v0,-4
800051e4:	00621024 	and	v0,v1,v0
800051e8:	8fc30048 	lw	v1,72(s8)
800051ec:	0062102b 	sltu	v0,v1,v0
800051f0:	1440fff2 	bnez	v0,800051bc <_malloc_r+0x764>
800051f4:	00000000 	nop
800051f8:	8fc20038 	lw	v0,56(s8)
800051fc:	8c42000c 	lw	v0,12(v0)
80005200:	afc2003c 	sw	v0,60(s8)
80005204:	8fc20020 	lw	v0,32(s8)
80005208:	8fc3003c 	lw	v1,60(s8)
8000520c:	ac43000c 	sw	v1,12(v0)
80005210:	8fc20020 	lw	v0,32(s8)
80005214:	8fc30038 	lw	v1,56(s8)
80005218:	ac430008 	sw	v1,8(v0)
8000521c:	8fc2003c 	lw	v0,60(s8)
80005220:	8fc30020 	lw	v1,32(s8)
80005224:	ac430008 	sw	v1,8(v0)
80005228:	8fc2003c 	lw	v0,60(s8)
8000522c:	8c430008 	lw	v1,8(v0)
80005230:	8fc20038 	lw	v0,56(s8)
80005234:	ac43000c 	sw	v1,12(v0)
80005238:	8fc20024 	lw	v0,36(s8)
8000523c:	00021fc3 	sra	v1,v0,0x1f
80005240:	00031f82 	srl	v1,v1,0x1e
80005244:	00621021 	addu	v0,v1,v0
80005248:	00021083 	sra	v0,v0,0x2
8000524c:	00401825 	move	v1,v0
80005250:	24020001 	li	v0,1
80005254:	00621004 	sllv	v0,v0,v1
80005258:	afc20030 	sw	v0,48(s8)
8000525c:	3c028002 	lui	v0,0x8002
80005260:	2442ce98 	addiu	v0,v0,-12648
80005264:	8c420004 	lw	v0,4(v0)
80005268:	8fc30030 	lw	v1,48(s8)
8000526c:	0043102b 	sltu	v0,v0,v1
80005270:	144000db 	bnez	v0,800055e0 <_malloc_r+0xb88>
80005274:	00000000 	nop
80005278:	3c028002 	lui	v0,0x8002
8000527c:	2442ce98 	addiu	v0,v0,-12648
80005280:	8c430004 	lw	v1,4(v0)
80005284:	8fc20030 	lw	v0,48(s8)
80005288:	00621024 	and	v0,v1,v0
8000528c:	14400018 	bnez	v0,800052f0 <_malloc_r+0x898>
80005290:	00000000 	nop
80005294:	8fc30024 	lw	v1,36(s8)
80005298:	2402fffc 	li	v0,-4
8000529c:	00621024 	and	v0,v1,v0
800052a0:	24420004 	addiu	v0,v0,4
800052a4:	afc20024 	sw	v0,36(s8)
800052a8:	8fc20030 	lw	v0,48(s8)
800052ac:	00021040 	sll	v0,v0,0x1
800052b0:	afc20030 	sw	v0,48(s8)
800052b4:	10000007 	b	800052d4 <_malloc_r+0x87c>
800052b8:	00000000 	nop
800052bc:	8fc20024 	lw	v0,36(s8)
800052c0:	24420004 	addiu	v0,v0,4
800052c4:	afc20024 	sw	v0,36(s8)
800052c8:	8fc20030 	lw	v0,48(s8)
800052cc:	00021040 	sll	v0,v0,0x1
800052d0:	afc20030 	sw	v0,48(s8)
800052d4:	3c028002 	lui	v0,0x8002
800052d8:	2442ce98 	addiu	v0,v0,-12648
800052dc:	8c430004 	lw	v1,4(v0)
800052e0:	8fc20030 	lw	v0,48(s8)
800052e4:	00621024 	and	v0,v1,v0
800052e8:	1040fff4 	beqz	v0,800052bc <_malloc_r+0x864>
800052ec:	00000000 	nop
800052f0:	8fc20024 	lw	v0,36(s8)
800052f4:	afc20034 	sw	v0,52(s8)
800052f8:	8fc20024 	lw	v0,36(s8)
800052fc:	24420001 	addiu	v0,v0,1
80005300:	00021040 	sll	v0,v0,0x1
80005304:	00021880 	sll	v1,v0,0x2
80005308:	3c028002 	lui	v0,0x8002
8000530c:	2442ce98 	addiu	v0,v0,-12648
80005310:	00621021 	addu	v0,v1,v0
80005314:	2442fff8 	addiu	v0,v0,-8
80005318:	afc20028 	sw	v0,40(s8)
8000531c:	8fc20028 	lw	v0,40(s8)
80005320:	afc20040 	sw	v0,64(s8)
80005324:	8fc20028 	lw	v0,40(s8)
80005328:	8c42000c 	lw	v0,12(v0)
8000532c:	afc20020 	sw	v0,32(s8)
80005330:	10000065 	b	800054c8 <_malloc_r+0xa70>
80005334:	00000000 	nop
80005338:	8fc20020 	lw	v0,32(s8)
8000533c:	8c430004 	lw	v1,4(v0)
80005340:	2402fffc 	li	v0,-4
80005344:	00621024 	and	v0,v1,v0
80005348:	afc20048 	sw	v0,72(s8)
8000534c:	8fc30048 	lw	v1,72(s8)
80005350:	8fc20044 	lw	v0,68(s8)
80005354:	00621023 	subu	v0,v1,v0
80005358:	afc2002c 	sw	v0,44(s8)
8000535c:	8fc2002c 	lw	v0,44(s8)
80005360:	28420010 	slti	v0,v0,16
80005364:	14400036 	bnez	v0,80005440 <_malloc_r+0x9e8>
80005368:	00000000 	nop
8000536c:	8fc30020 	lw	v1,32(s8)
80005370:	8fc20044 	lw	v0,68(s8)
80005374:	00621021 	addu	v0,v1,v0
80005378:	afc20050 	sw	v0,80(s8)
8000537c:	8fc20044 	lw	v0,68(s8)
80005380:	34430001 	ori	v1,v0,0x1
80005384:	8fc20020 	lw	v0,32(s8)
80005388:	ac430004 	sw	v1,4(v0)
8000538c:	8fc20020 	lw	v0,32(s8)
80005390:	8c42000c 	lw	v0,12(v0)
80005394:	afc2003c 	sw	v0,60(s8)
80005398:	8fc20020 	lw	v0,32(s8)
8000539c:	8c420008 	lw	v0,8(v0)
800053a0:	afc20038 	sw	v0,56(s8)
800053a4:	8fc20038 	lw	v0,56(s8)
800053a8:	8fc3003c 	lw	v1,60(s8)
800053ac:	ac43000c 	sw	v1,12(v0)
800053b0:	8fc2003c 	lw	v0,60(s8)
800053b4:	8fc30038 	lw	v1,56(s8)
800053b8:	ac430008 	sw	v1,8(v0)
800053bc:	3c028002 	lui	v0,0x8002
800053c0:	2442cea0 	addiu	v0,v0,-12640
800053c4:	8fc30050 	lw	v1,80(s8)
800053c8:	ac43000c 	sw	v1,12(v0)
800053cc:	3c038002 	lui	v1,0x8002
800053d0:	2463cea0 	addiu	v1,v1,-12640
800053d4:	8c42000c 	lw	v0,12(v0)
800053d8:	ac620008 	sw	v0,8(v1)
800053dc:	3c028002 	lui	v0,0x8002
800053e0:	2443cea0 	addiu	v1,v0,-12640
800053e4:	8fc20050 	lw	v0,80(s8)
800053e8:	ac43000c 	sw	v1,12(v0)
800053ec:	8fc20050 	lw	v0,80(s8)
800053f0:	8c43000c 	lw	v1,12(v0)
800053f4:	8fc20050 	lw	v0,80(s8)
800053f8:	ac430008 	sw	v1,8(v0)
800053fc:	8fc2002c 	lw	v0,44(s8)
80005400:	34420001 	ori	v0,v0,0x1
80005404:	00401825 	move	v1,v0
80005408:	8fc20050 	lw	v0,80(s8)
8000540c:	ac430004 	sw	v1,4(v0)
80005410:	8fc2002c 	lw	v0,44(s8)
80005414:	8fc30050 	lw	v1,80(s8)
80005418:	00621021 	addu	v0,v1,v0
8000541c:	8fc3002c 	lw	v1,44(s8)
80005420:	ac430000 	sw	v1,0(v0)
80005424:	8fc40068 	lw	a0,104(s8)
80005428:	0c001bb1 	jal	80006ec4 <__malloc_unlock>
8000542c:	00000000 	nop
80005430:	8fc20020 	lw	v0,32(s8)
80005434:	24420008 	addiu	v0,v0,8
80005438:	100000ba 	b	80005724 <_malloc_r+0xccc>
8000543c:	00000000 	nop
80005440:	8fc2002c 	lw	v0,44(s8)
80005444:	0440001d 	bltz	v0,800054bc <_malloc_r+0xa64>
80005448:	00000000 	nop
8000544c:	8fc30020 	lw	v1,32(s8)
80005450:	8fc20048 	lw	v0,72(s8)
80005454:	00621021 	addu	v0,v1,v0
80005458:	8c430004 	lw	v1,4(v0)
8000545c:	8fc40020 	lw	a0,32(s8)
80005460:	8fc20048 	lw	v0,72(s8)
80005464:	00821021 	addu	v0,a0,v0
80005468:	34630001 	ori	v1,v1,0x1
8000546c:	ac430004 	sw	v1,4(v0)
80005470:	8fc20020 	lw	v0,32(s8)
80005474:	8c42000c 	lw	v0,12(v0)
80005478:	afc2003c 	sw	v0,60(s8)
8000547c:	8fc20020 	lw	v0,32(s8)
80005480:	8c420008 	lw	v0,8(v0)
80005484:	afc20038 	sw	v0,56(s8)
80005488:	8fc20038 	lw	v0,56(s8)
8000548c:	8fc3003c 	lw	v1,60(s8)
80005490:	ac43000c 	sw	v1,12(v0)
80005494:	8fc2003c 	lw	v0,60(s8)
80005498:	8fc30038 	lw	v1,56(s8)
8000549c:	ac430008 	sw	v1,8(v0)
800054a0:	8fc40068 	lw	a0,104(s8)
800054a4:	0c001bb1 	jal	80006ec4 <__malloc_unlock>
800054a8:	00000000 	nop
800054ac:	8fc20020 	lw	v0,32(s8)
800054b0:	24420008 	addiu	v0,v0,8
800054b4:	1000009b 	b	80005724 <_malloc_r+0xccc>
800054b8:	00000000 	nop
800054bc:	8fc20020 	lw	v0,32(s8)
800054c0:	8c42000c 	lw	v0,12(v0)
800054c4:	afc20020 	sw	v0,32(s8)
800054c8:	8fc30020 	lw	v1,32(s8)
800054cc:	8fc20028 	lw	v0,40(s8)
800054d0:	1462ff99 	bne	v1,v0,80005338 <_malloc_r+0x8e0>
800054d4:	00000000 	nop
800054d8:	8fc20028 	lw	v0,40(s8)
800054dc:	24420008 	addiu	v0,v0,8
800054e0:	afc20028 	sw	v0,40(s8)
800054e4:	8fc20024 	lw	v0,36(s8)
800054e8:	24420001 	addiu	v0,v0,1
800054ec:	afc20024 	sw	v0,36(s8)
800054f0:	8fc20024 	lw	v0,36(s8)
800054f4:	30420003 	andi	v0,v0,0x3
800054f8:	1440ff8a 	bnez	v0,80005324 <_malloc_r+0x8cc>
800054fc:	00000000 	nop
80005500:	8fc20034 	lw	v0,52(s8)
80005504:	30420003 	andi	v0,v0,0x3
80005508:	1440000c 	bnez	v0,8000553c <_malloc_r+0xae4>
8000550c:	00000000 	nop
80005510:	3c028002 	lui	v0,0x8002
80005514:	2442ce98 	addiu	v0,v0,-12648
80005518:	8c440004 	lw	a0,4(v0)
8000551c:	8fc20030 	lw	v0,48(s8)
80005520:	00021827 	nor	v1,zero,v0
80005524:	3c028002 	lui	v0,0x8002
80005528:	2442ce98 	addiu	v0,v0,-12648
8000552c:	00831824 	and	v1,a0,v1
80005530:	ac430004 	sw	v1,4(v0)
80005534:	1000000c 	b	80005568 <_malloc_r+0xb10>
80005538:	00000000 	nop
8000553c:	8fc20034 	lw	v0,52(s8)
80005540:	2442ffff 	addiu	v0,v0,-1
80005544:	afc20034 	sw	v0,52(s8)
80005548:	8fc20040 	lw	v0,64(s8)
8000554c:	2442fff8 	addiu	v0,v0,-8
80005550:	afc20040 	sw	v0,64(s8)
80005554:	8fc20040 	lw	v0,64(s8)
80005558:	8c420008 	lw	v0,8(v0)
8000555c:	8fc30040 	lw	v1,64(s8)
80005560:	1062ffe7 	beq	v1,v0,80005500 <_malloc_r+0xaa8>
80005564:	00000000 	nop
80005568:	8fc20030 	lw	v0,48(s8)
8000556c:	00021040 	sll	v0,v0,0x1
80005570:	afc20030 	sw	v0,48(s8)
80005574:	3c028002 	lui	v0,0x8002
80005578:	2442ce98 	addiu	v0,v0,-12648
8000557c:	8c420004 	lw	v0,4(v0)
80005580:	8fc30030 	lw	v1,48(s8)
80005584:	0043102b 	sltu	v0,v0,v1
80005588:	14400015 	bnez	v0,800055e0 <_malloc_r+0xb88>
8000558c:	00000000 	nop
80005590:	8fc20030 	lw	v0,48(s8)
80005594:	10400012 	beqz	v0,800055e0 <_malloc_r+0xb88>
80005598:	00000000 	nop
8000559c:	10000007 	b	800055bc <_malloc_r+0xb64>
800055a0:	00000000 	nop
800055a4:	8fc20024 	lw	v0,36(s8)
800055a8:	24420004 	addiu	v0,v0,4
800055ac:	afc20024 	sw	v0,36(s8)
800055b0:	8fc20030 	lw	v0,48(s8)
800055b4:	00021040 	sll	v0,v0,0x1
800055b8:	afc20030 	sw	v0,48(s8)
800055bc:	3c028002 	lui	v0,0x8002
800055c0:	2442ce98 	addiu	v0,v0,-12648
800055c4:	8c430004 	lw	v1,4(v0)
800055c8:	8fc20030 	lw	v0,48(s8)
800055cc:	00621024 	and	v0,v1,v0
800055d0:	1040fff4 	beqz	v0,800055a4 <_malloc_r+0xb4c>
800055d4:	00000000 	nop
800055d8:	1000ff45 	b	800052f0 <_malloc_r+0x898>
800055dc:	00000000 	nop
800055e0:	3c028002 	lui	v0,0x8002
800055e4:	2442ce98 	addiu	v0,v0,-12648
800055e8:	8c420008 	lw	v0,8(v0)
800055ec:	8c430004 	lw	v1,4(v0)
800055f0:	2402fffc 	li	v0,-4
800055f4:	00621824 	and	v1,v1,v0
800055f8:	8fc20044 	lw	v0,68(s8)
800055fc:	00621023 	subu	v0,v1,v0
80005600:	afc2002c 	sw	v0,44(s8)
80005604:	3c028002 	lui	v0,0x8002
80005608:	2442ce98 	addiu	v0,v0,-12648
8000560c:	8c420008 	lw	v0,8(v0)
80005610:	8c430004 	lw	v1,4(v0)
80005614:	2402fffc 	li	v0,-4
80005618:	00621024 	and	v0,v1,v0
8000561c:	8fc30044 	lw	v1,68(s8)
80005620:	0043102b 	sltu	v0,v0,v1
80005624:	14400005 	bnez	v0,8000563c <_malloc_r+0xbe4>
80005628:	00000000 	nop
8000562c:	8fc2002c 	lw	v0,44(s8)
80005630:	28420010 	slti	v0,v0,16
80005634:	10400022 	beqz	v0,800056c0 <_malloc_r+0xc68>
80005638:	00000000 	nop
8000563c:	8fc50044 	lw	a1,68(s8)
80005640:	8fc40068 	lw	a0,104(s8)
80005644:	0c00118a 	jal	80004628 <malloc_extend_top>
80005648:	00000000 	nop
8000564c:	3c028002 	lui	v0,0x8002
80005650:	2442ce98 	addiu	v0,v0,-12648
80005654:	8c420008 	lw	v0,8(v0)
80005658:	8c430004 	lw	v1,4(v0)
8000565c:	2402fffc 	li	v0,-4
80005660:	00621824 	and	v1,v1,v0
80005664:	8fc20044 	lw	v0,68(s8)
80005668:	00621023 	subu	v0,v1,v0
8000566c:	afc2002c 	sw	v0,44(s8)
80005670:	3c028002 	lui	v0,0x8002
80005674:	2442ce98 	addiu	v0,v0,-12648
80005678:	8c420008 	lw	v0,8(v0)
8000567c:	8c430004 	lw	v1,4(v0)
80005680:	2402fffc 	li	v0,-4
80005684:	00621024 	and	v0,v1,v0
80005688:	8fc30044 	lw	v1,68(s8)
8000568c:	0043102b 	sltu	v0,v0,v1
80005690:	14400005 	bnez	v0,800056a8 <_malloc_r+0xc50>
80005694:	00000000 	nop
80005698:	8fc2002c 	lw	v0,44(s8)
8000569c:	28420010 	slti	v0,v0,16
800056a0:	10400007 	beqz	v0,800056c0 <_malloc_r+0xc68>
800056a4:	00000000 	nop
800056a8:	8fc40068 	lw	a0,104(s8)
800056ac:	0c001bb1 	jal	80006ec4 <__malloc_unlock>
800056b0:	00000000 	nop
800056b4:	00001025 	move	v0,zero
800056b8:	1000001a 	b	80005724 <_malloc_r+0xccc>
800056bc:	00000000 	nop
800056c0:	3c028002 	lui	v0,0x8002
800056c4:	2442ce98 	addiu	v0,v0,-12648
800056c8:	8c420008 	lw	v0,8(v0)
800056cc:	afc20020 	sw	v0,32(s8)
800056d0:	8fc20044 	lw	v0,68(s8)
800056d4:	34430001 	ori	v1,v0,0x1
800056d8:	8fc20020 	lw	v0,32(s8)
800056dc:	ac430004 	sw	v1,4(v0)
800056e0:	3c028002 	lui	v0,0x8002
800056e4:	2442ce98 	addiu	v0,v0,-12648
800056e8:	8fc40020 	lw	a0,32(s8)
800056ec:	8fc30044 	lw	v1,68(s8)
800056f0:	00831821 	addu	v1,a0,v1
800056f4:	ac430008 	sw	v1,8(v0)
800056f8:	8fc2002c 	lw	v0,44(s8)
800056fc:	34430001 	ori	v1,v0,0x1
80005700:	3c028002 	lui	v0,0x8002
80005704:	2442ce98 	addiu	v0,v0,-12648
80005708:	8c420008 	lw	v0,8(v0)
8000570c:	ac430004 	sw	v1,4(v0)
80005710:	8fc40068 	lw	a0,104(s8)
80005714:	0c001bb1 	jal	80006ec4 <__malloc_unlock>
80005718:	00000000 	nop
8000571c:	8fc20020 	lw	v0,32(s8)
80005720:	24420008 	addiu	v0,v0,8
80005724:	03c0e825 	move	sp,s8
80005728:	dfbf0060 	ld	ra,96(sp)
8000572c:	dfbe0058 	ld	s8,88(sp)
80005730:	27bd0068 	addiu	sp,sp,104
80005734:	03e00008 	jr	ra
80005738:	00000000 	nop

8000573c <memcpy>:
8000573c:	28ca0008 	slti	t2,a2,8
80005740:	1540005b 	bnez	t2,800058b0 <memcpy+0x174>
80005744:	00801025 	move	v0,a0
80005748:	00a4c026 	xor	t8,a1,a0
8000574c:	33180003 	andi	t8,t8,0x3
80005750:	17000066 	bnez	t8,800058ec <memcpy+0x1b0>
80005754:	00043823 	negu	a3,a0
80005758:	30e70003 	andi	a3,a3,0x3
8000575c:	10e00005 	beqz	a3,80005774 <memcpy+0x38>
80005760:	00c73023 	subu	a2,a2,a3
80005764:	88b80000 	lwl	t8,0(a1)
80005768:	00a72821 	addu	a1,a1,a3
8000576c:	a8980000 	swl	t8,0(a0)
80005770:	00872021 	addu	a0,a0,a3
80005774:	30d8003f 	andi	t8,a2,0x3f
80005778:	10d80026 	beq	a2,t8,80005814 <memcpy+0xd8>
8000577c:	00d83823 	subu	a3,a2,t8
80005780:	00873821 	addu	a3,a0,a3
80005784:	8ca80000 	lw	t0,0(a1)
80005788:	8ca90004 	lw	t1,4(a1)
8000578c:	8caa0008 	lw	t2,8(a1)
80005790:	8cab000c 	lw	t3,12(a1)
80005794:	8cac0010 	lw	t4,16(a1)
80005798:	8cad0014 	lw	t5,20(a1)
8000579c:	8cae0018 	lw	t6,24(a1)
800057a0:	8caf001c 	lw	t7,28(a1)
800057a4:	ac880000 	sw	t0,0(a0)
800057a8:	ac890004 	sw	t1,4(a0)
800057ac:	ac8a0008 	sw	t2,8(a0)
800057b0:	ac8b000c 	sw	t3,12(a0)
800057b4:	ac8c0010 	sw	t4,16(a0)
800057b8:	ac8d0014 	sw	t5,20(a0)
800057bc:	ac8e0018 	sw	t6,24(a0)
800057c0:	ac8f001c 	sw	t7,28(a0)
800057c4:	8ca80020 	lw	t0,32(a1)
800057c8:	8ca90024 	lw	t1,36(a1)
800057cc:	8caa0028 	lw	t2,40(a1)
800057d0:	8cab002c 	lw	t3,44(a1)
800057d4:	8cac0030 	lw	t4,48(a1)
800057d8:	8cad0034 	lw	t5,52(a1)
800057dc:	8cae0038 	lw	t6,56(a1)
800057e0:	8caf003c 	lw	t7,60(a1)
800057e4:	ac880020 	sw	t0,32(a0)
800057e8:	ac890024 	sw	t1,36(a0)
800057ec:	ac8a0028 	sw	t2,40(a0)
800057f0:	ac8b002c 	sw	t3,44(a0)
800057f4:	ac8c0030 	sw	t4,48(a0)
800057f8:	ac8d0034 	sw	t5,52(a0)
800057fc:	ac8e0038 	sw	t6,56(a0)
80005800:	ac8f003c 	sw	t7,60(a0)
80005804:	24840040 	addiu	a0,a0,64
80005808:	1487ffde 	bne	a0,a3,80005784 <memcpy+0x48>
8000580c:	24a50040 	addiu	a1,a1,64
80005810:	03003025 	move	a2,t8
80005814:	30d8001f 	andi	t8,a2,0x1f
80005818:	10d80013 	beq	a2,t8,80005868 <memcpy+0x12c>
8000581c:	00000000 	nop
80005820:	8ca80000 	lw	t0,0(a1)
80005824:	8ca90004 	lw	t1,4(a1)
80005828:	8caa0008 	lw	t2,8(a1)
8000582c:	8cab000c 	lw	t3,12(a1)
80005830:	8cac0010 	lw	t4,16(a1)
80005834:	8cad0014 	lw	t5,20(a1)
80005838:	8cae0018 	lw	t6,24(a1)
8000583c:	8caf001c 	lw	t7,28(a1)
80005840:	24a50020 	addiu	a1,a1,32
80005844:	ac880000 	sw	t0,0(a0)
80005848:	ac890004 	sw	t1,4(a0)
8000584c:	ac8a0008 	sw	t2,8(a0)
80005850:	ac8b000c 	sw	t3,12(a0)
80005854:	ac8c0010 	sw	t4,16(a0)
80005858:	ac8d0014 	sw	t5,20(a0)
8000585c:	ac8e0018 	sw	t6,24(a0)
80005860:	ac8f001c 	sw	t7,28(a0)
80005864:	24840020 	addiu	a0,a0,32
80005868:	33060003 	andi	a2,t8,0x3
8000586c:	10d80007 	beq	a2,t8,8000588c <memcpy+0x150>
80005870:	03063823 	subu	a3,t8,a2
80005874:	00873821 	addu	a3,a0,a3
80005878:	8cab0000 	lw	t3,0(a1)
8000587c:	24840004 	addiu	a0,a0,4
80005880:	24a50004 	addiu	a1,a1,4
80005884:	1487fffc 	bne	a0,a3,80005878 <memcpy+0x13c>
80005888:	ac8bfffc 	sw	t3,-4(a0)
8000588c:	18c00006 	blez	a2,800058a8 <memcpy+0x16c>
80005890:	00863821 	addu	a3,a0,a2
80005894:	80a30000 	lb	v1,0(a1)
80005898:	24840001 	addiu	a0,a0,1
8000589c:	24a50001 	addiu	a1,a1,1
800058a0:	1487fffc 	bne	a0,a3,80005894 <memcpy+0x158>
800058a4:	a083ffff 	sb	v1,-1(a0)
800058a8:	03e00008 	jr	ra
800058ac:	00000000 	nop
800058b0:	30d80003 	andi	t8,a2,0x3
800058b4:	1306fff5 	beq	t8,a2,8000588c <memcpy+0x150>
800058b8:	30990003 	andi	t9,a0,0x3
800058bc:	1720fff3 	bnez	t9,8000588c <memcpy+0x150>
800058c0:	30b90003 	andi	t9,a1,0x3
800058c4:	1720fff1 	bnez	t9,8000588c <memcpy+0x150>
800058c8:	00d83823 	subu	a3,a2,t8
800058cc:	00873821 	addu	a3,a0,a3
800058d0:	8cab0000 	lw	t3,0(a1)
800058d4:	24840004 	addiu	a0,a0,4
800058d8:	24a50004 	addiu	a1,a1,4
800058dc:	1487fffc 	bne	a0,a3,800058d0 <memcpy+0x194>
800058e0:	ac8bfffc 	sw	t3,-4(a0)
800058e4:	1000ffe9 	b	8000588c <memcpy+0x150>
800058e8:	03003025 	move	a2,t8
800058ec:	30e70003 	andi	a3,a3,0x3
800058f0:	10e00006 	beqz	a3,8000590c <memcpy+0x1d0>
800058f4:	00c73023 	subu	a2,a2,a3
800058f8:	88a30000 	lwl	v1,0(a1)
800058fc:	98a30003 	lwr	v1,3(a1)
80005900:	00a72821 	addu	a1,a1,a3
80005904:	a8830000 	swl	v1,0(a0)
80005908:	00872021 	addu	a0,a0,a3
8000590c:	30d8003f 	andi	t8,a2,0x3f
80005910:	10d80036 	beq	a2,t8,800059ec <memcpy+0x2b0>
80005914:	00d83823 	subu	a3,a2,t8
80005918:	00873821 	addu	a3,a0,a3
8000591c:	88a80000 	lwl	t0,0(a1)
80005920:	88a90004 	lwl	t1,4(a1)
80005924:	88aa0008 	lwl	t2,8(a1)
80005928:	88ab000c 	lwl	t3,12(a1)
8000592c:	88ac0010 	lwl	t4,16(a1)
80005930:	88ad0014 	lwl	t5,20(a1)
80005934:	88ae0018 	lwl	t6,24(a1)
80005938:	88af001c 	lwl	t7,28(a1)
8000593c:	98a80003 	lwr	t0,3(a1)
80005940:	98a90007 	lwr	t1,7(a1)
80005944:	98aa000b 	lwr	t2,11(a1)
80005948:	98ab000f 	lwr	t3,15(a1)
8000594c:	98ac0013 	lwr	t4,19(a1)
80005950:	98ad0017 	lwr	t5,23(a1)
80005954:	98ae001b 	lwr	t6,27(a1)
80005958:	98af001f 	lwr	t7,31(a1)
8000595c:	ac880000 	sw	t0,0(a0)
80005960:	ac890004 	sw	t1,4(a0)
80005964:	ac8a0008 	sw	t2,8(a0)
80005968:	ac8b000c 	sw	t3,12(a0)
8000596c:	ac8c0010 	sw	t4,16(a0)
80005970:	ac8d0014 	sw	t5,20(a0)
80005974:	ac8e0018 	sw	t6,24(a0)
80005978:	ac8f001c 	sw	t7,28(a0)
8000597c:	88a80020 	lwl	t0,32(a1)
80005980:	88a90024 	lwl	t1,36(a1)
80005984:	88aa0028 	lwl	t2,40(a1)
80005988:	88ab002c 	lwl	t3,44(a1)
8000598c:	88ac0030 	lwl	t4,48(a1)
80005990:	88ad0034 	lwl	t5,52(a1)
80005994:	88ae0038 	lwl	t6,56(a1)
80005998:	88af003c 	lwl	t7,60(a1)
8000599c:	98a80023 	lwr	t0,35(a1)
800059a0:	98a90027 	lwr	t1,39(a1)
800059a4:	98aa002b 	lwr	t2,43(a1)
800059a8:	98ab002f 	lwr	t3,47(a1)
800059ac:	98ac0033 	lwr	t4,51(a1)
800059b0:	98ad0037 	lwr	t5,55(a1)
800059b4:	98ae003b 	lwr	t6,59(a1)
800059b8:	98af003f 	lwr	t7,63(a1)
800059bc:	ac880020 	sw	t0,32(a0)
800059c0:	ac890024 	sw	t1,36(a0)
800059c4:	ac8a0028 	sw	t2,40(a0)
800059c8:	ac8b002c 	sw	t3,44(a0)
800059cc:	ac8c0030 	sw	t4,48(a0)
800059d0:	ac8d0034 	sw	t5,52(a0)
800059d4:	ac8e0038 	sw	t6,56(a0)
800059d8:	ac8f003c 	sw	t7,60(a0)
800059dc:	24840040 	addiu	a0,a0,64
800059e0:	1487ffce 	bne	a0,a3,8000591c <memcpy+0x1e0>
800059e4:	24a50040 	addiu	a1,a1,64
800059e8:	03003025 	move	a2,t8
800059ec:	30d8001f 	andi	t8,a2,0x1f
800059f0:	10d8001b 	beq	a2,t8,80005a60 <memcpy+0x324>
800059f4:	00000000 	nop
800059f8:	88a80000 	lwl	t0,0(a1)
800059fc:	88a90004 	lwl	t1,4(a1)
80005a00:	88aa0008 	lwl	t2,8(a1)
80005a04:	88ab000c 	lwl	t3,12(a1)
80005a08:	88ac0010 	lwl	t4,16(a1)
80005a0c:	88ad0014 	lwl	t5,20(a1)
80005a10:	88ae0018 	lwl	t6,24(a1)
80005a14:	88af001c 	lwl	t7,28(a1)
80005a18:	98a80003 	lwr	t0,3(a1)
80005a1c:	98a90007 	lwr	t1,7(a1)
80005a20:	98aa000b 	lwr	t2,11(a1)
80005a24:	98ab000f 	lwr	t3,15(a1)
80005a28:	98ac0013 	lwr	t4,19(a1)
80005a2c:	98ad0017 	lwr	t5,23(a1)
80005a30:	98ae001b 	lwr	t6,27(a1)
80005a34:	98af001f 	lwr	t7,31(a1)
80005a38:	24a50020 	addiu	a1,a1,32
80005a3c:	ac880000 	sw	t0,0(a0)
80005a40:	ac890004 	sw	t1,4(a0)
80005a44:	ac8a0008 	sw	t2,8(a0)
80005a48:	ac8b000c 	sw	t3,12(a0)
80005a4c:	ac8c0010 	sw	t4,16(a0)
80005a50:	ac8d0014 	sw	t5,20(a0)
80005a54:	ac8e0018 	sw	t6,24(a0)
80005a58:	ac8f001c 	sw	t7,28(a0)
80005a5c:	24840020 	addiu	a0,a0,32
80005a60:	33060003 	andi	a2,t8,0x3
80005a64:	10d80008 	beq	a2,t8,80005a88 <memcpy+0x34c>
80005a68:	03063823 	subu	a3,t8,a2
80005a6c:	00873821 	addu	a3,a0,a3
80005a70:	88a30000 	lwl	v1,0(a1)
80005a74:	98a30003 	lwr	v1,3(a1)
80005a78:	24840004 	addiu	a0,a0,4
80005a7c:	24a50004 	addiu	a1,a1,4
80005a80:	1487fffb 	bne	a0,a3,80005a70 <memcpy+0x334>
80005a84:	ac83fffc 	sw	v1,-4(a0)
80005a88:	10c0ff87 	beqz	a2,800058a8 <memcpy+0x16c>
80005a8c:	00863821 	addu	a3,a0,a2
80005a90:	80a30000 	lb	v1,0(a1)
80005a94:	24840001 	addiu	a0,a0,1
80005a98:	24a50001 	addiu	a1,a1,1
80005a9c:	1487fffc 	bne	a0,a3,80005a90 <memcpy+0x354>
80005aa0:	a083ffff 	sb	v1,-1(a0)
80005aa4:	03e00008 	jr	ra
80005aa8:	00000000 	nop

80005aac <memmove>:
80005aac:	27bdffe8 	addiu	sp,sp,-24
80005ab0:	ffbe0010 	sd	s8,16(sp)
80005ab4:	03a0f025 	move	s8,sp
80005ab8:	afc40018 	sw	a0,24(s8)
80005abc:	afc50020 	sw	a1,32(s8)
80005ac0:	afc60028 	sw	a2,40(s8)
80005ac4:	8fc20018 	lw	v0,24(s8)
80005ac8:	afc20000 	sw	v0,0(s8)
80005acc:	8fc20020 	lw	v0,32(s8)
80005ad0:	afc20004 	sw	v0,4(s8)
80005ad4:	8fc30004 	lw	v1,4(s8)
80005ad8:	8fc20000 	lw	v0,0(s8)
80005adc:	0062102b 	sltu	v0,v1,v0
80005ae0:	10400023 	beqz	v0,80005b70 <memmove+0xc4>
80005ae4:	00000000 	nop
80005ae8:	8fc30004 	lw	v1,4(s8)
80005aec:	8fc20028 	lw	v0,40(s8)
80005af0:	00621021 	addu	v0,v1,v0
80005af4:	8fc30000 	lw	v1,0(s8)
80005af8:	0062102b 	sltu	v0,v1,v0
80005afc:	1040001c 	beqz	v0,80005b70 <memmove+0xc4>
80005b00:	00000000 	nop
80005b04:	8fc30004 	lw	v1,4(s8)
80005b08:	8fc20028 	lw	v0,40(s8)
80005b0c:	00621021 	addu	v0,v1,v0
80005b10:	afc20004 	sw	v0,4(s8)
80005b14:	8fc30000 	lw	v1,0(s8)
80005b18:	8fc20028 	lw	v0,40(s8)
80005b1c:	00621021 	addu	v0,v1,v0
80005b20:	afc20000 	sw	v0,0(s8)
80005b24:	1000000b 	b	80005b54 <memmove+0xa8>
80005b28:	00000000 	nop
80005b2c:	8fc20004 	lw	v0,4(s8)
80005b30:	2442ffff 	addiu	v0,v0,-1
80005b34:	afc20004 	sw	v0,4(s8)
80005b38:	8fc20000 	lw	v0,0(s8)
80005b3c:	2442ffff 	addiu	v0,v0,-1
80005b40:	afc20000 	sw	v0,0(s8)
80005b44:	8fc20004 	lw	v0,4(s8)
80005b48:	80430000 	lb	v1,0(v0)
80005b4c:	8fc20000 	lw	v0,0(s8)
80005b50:	a0430000 	sb	v1,0(v0)
80005b54:	8fc20028 	lw	v0,40(s8)
80005b58:	2443ffff 	addiu	v1,v0,-1
80005b5c:	afc30028 	sw	v1,40(s8)
80005b60:	1440fff2 	bnez	v0,80005b2c <memmove+0x80>
80005b64:	00000000 	nop
80005b68:	1000005c 	b	80005cdc <memmove+0x230>
80005b6c:	00000000 	nop
80005b70:	8fc20028 	lw	v0,40(s8)
80005b74:	2c420010 	sltiu	v0,v0,16
80005b78:	14400053 	bnez	v0,80005cc8 <memmove+0x21c>
80005b7c:	00000000 	nop
80005b80:	8fc30004 	lw	v1,4(s8)
80005b84:	8fc20000 	lw	v0,0(s8)
80005b88:	00621025 	or	v0,v1,v0
80005b8c:	30420003 	andi	v0,v0,0x3
80005b90:	1440004d 	bnez	v0,80005cc8 <memmove+0x21c>
80005b94:	00000000 	nop
80005b98:	8fc20000 	lw	v0,0(s8)
80005b9c:	afc20008 	sw	v0,8(s8)
80005ba0:	8fc20004 	lw	v0,4(s8)
80005ba4:	afc2000c 	sw	v0,12(s8)
80005ba8:	10000024 	b	80005c3c <memmove+0x190>
80005bac:	00000000 	nop
80005bb0:	8fc3000c 	lw	v1,12(s8)
80005bb4:	24620004 	addiu	v0,v1,4
80005bb8:	afc2000c 	sw	v0,12(s8)
80005bbc:	8fc20008 	lw	v0,8(s8)
80005bc0:	24440004 	addiu	a0,v0,4
80005bc4:	afc40008 	sw	a0,8(s8)
80005bc8:	8c630000 	lw	v1,0(v1)
80005bcc:	ac430000 	sw	v1,0(v0)
80005bd0:	8fc3000c 	lw	v1,12(s8)
80005bd4:	24620004 	addiu	v0,v1,4
80005bd8:	afc2000c 	sw	v0,12(s8)
80005bdc:	8fc20008 	lw	v0,8(s8)
80005be0:	24440004 	addiu	a0,v0,4
80005be4:	afc40008 	sw	a0,8(s8)
80005be8:	8c630000 	lw	v1,0(v1)
80005bec:	ac430000 	sw	v1,0(v0)
80005bf0:	8fc3000c 	lw	v1,12(s8)
80005bf4:	24620004 	addiu	v0,v1,4
80005bf8:	afc2000c 	sw	v0,12(s8)
80005bfc:	8fc20008 	lw	v0,8(s8)
80005c00:	24440004 	addiu	a0,v0,4
80005c04:	afc40008 	sw	a0,8(s8)
80005c08:	8c630000 	lw	v1,0(v1)
80005c0c:	ac430000 	sw	v1,0(v0)
80005c10:	8fc3000c 	lw	v1,12(s8)
80005c14:	24620004 	addiu	v0,v1,4
80005c18:	afc2000c 	sw	v0,12(s8)
80005c1c:	8fc20008 	lw	v0,8(s8)
80005c20:	24440004 	addiu	a0,v0,4
80005c24:	afc40008 	sw	a0,8(s8)
80005c28:	8c630000 	lw	v1,0(v1)
80005c2c:	ac430000 	sw	v1,0(v0)
80005c30:	8fc20028 	lw	v0,40(s8)
80005c34:	2442fff0 	addiu	v0,v0,-16
80005c38:	afc20028 	sw	v0,40(s8)
80005c3c:	8fc20028 	lw	v0,40(s8)
80005c40:	2c420010 	sltiu	v0,v0,16
80005c44:	1040ffda 	beqz	v0,80005bb0 <memmove+0x104>
80005c48:	00000000 	nop
80005c4c:	1000000c 	b	80005c80 <memmove+0x1d4>
80005c50:	00000000 	nop
80005c54:	8fc3000c 	lw	v1,12(s8)
80005c58:	24620004 	addiu	v0,v1,4
80005c5c:	afc2000c 	sw	v0,12(s8)
80005c60:	8fc20008 	lw	v0,8(s8)
80005c64:	24440004 	addiu	a0,v0,4
80005c68:	afc40008 	sw	a0,8(s8)
80005c6c:	8c630000 	lw	v1,0(v1)
80005c70:	ac430000 	sw	v1,0(v0)
80005c74:	8fc20028 	lw	v0,40(s8)
80005c78:	2442fffc 	addiu	v0,v0,-4
80005c7c:	afc20028 	sw	v0,40(s8)
80005c80:	8fc20028 	lw	v0,40(s8)
80005c84:	2c420004 	sltiu	v0,v0,4
80005c88:	1040fff2 	beqz	v0,80005c54 <memmove+0x1a8>
80005c8c:	00000000 	nop
80005c90:	8fc20008 	lw	v0,8(s8)
80005c94:	afc20000 	sw	v0,0(s8)
80005c98:	8fc2000c 	lw	v0,12(s8)
80005c9c:	afc20004 	sw	v0,4(s8)
80005ca0:	10000009 	b	80005cc8 <memmove+0x21c>
80005ca4:	00000000 	nop
80005ca8:	8fc30004 	lw	v1,4(s8)
80005cac:	24620001 	addiu	v0,v1,1
80005cb0:	afc20004 	sw	v0,4(s8)
80005cb4:	8fc20000 	lw	v0,0(s8)
80005cb8:	24440001 	addiu	a0,v0,1
80005cbc:	afc40000 	sw	a0,0(s8)
80005cc0:	80630000 	lb	v1,0(v1)
80005cc4:	a0430000 	sb	v1,0(v0)
80005cc8:	8fc20028 	lw	v0,40(s8)
80005ccc:	2443ffff 	addiu	v1,v0,-1
80005cd0:	afc30028 	sw	v1,40(s8)
80005cd4:	1440fff4 	bnez	v0,80005ca8 <memmove+0x1fc>
80005cd8:	00000000 	nop
80005cdc:	8fc20018 	lw	v0,24(s8)
80005ce0:	03c0e825 	move	sp,s8
80005ce4:	dfbe0010 	ld	s8,16(sp)
80005ce8:	27bd0018 	addiu	sp,sp,24
80005cec:	03e00008 	jr	ra
80005cf0:	00000000 	nop

80005cf4 <memset>:
80005cf4:	28ca0008 	slti	t2,a2,8
80005cf8:	1540003e 	bnez	t2,80005df4 <memset+0x100>
80005cfc:	00801025 	move	v0,a0
80005d00:	10a00007 	beqz	a1,80005d20 <memset+0x2c>
80005d04:	00043823 	negu	a3,a0
80005d08:	00000000 	nop
80005d0c:	30a500ff 	andi	a1,a1,0xff
80005d10:	00055200 	sll	t2,a1,0x8
80005d14:	00aa2825 	or	a1,a1,t2
80005d18:	00055400 	sll	t2,a1,0x10
80005d1c:	00aa2825 	or	a1,a1,t2
80005d20:	30ea0003 	andi	t2,a3,0x3
80005d24:	11400003 	beqz	t2,80005d34 <memset+0x40>
80005d28:	00ca3023 	subu	a2,a2,t2
80005d2c:	a8850000 	swl	a1,0(a0)
80005d30:	008a2021 	addu	a0,a0,t2
80005d34:	30ea0004 	andi	t2,a3,0x4
80005d38:	11400003 	beqz	t2,80005d48 <memset+0x54>
80005d3c:	00ca3023 	subu	a2,a2,t2
80005d40:	ac850000 	sw	a1,0(a0)
80005d44:	008a2021 	addu	a0,a0,t2
80005d48:	30d8003f 	andi	t8,a2,0x3f
80005d4c:	10d80016 	beq	a2,t8,80005da8 <memset+0xb4>
80005d50:	00d83823 	subu	a3,a2,t8
80005d54:	00873821 	addu	a3,a0,a3
80005d58:	ac850000 	sw	a1,0(a0)
80005d5c:	ac850004 	sw	a1,4(a0)
80005d60:	ac850008 	sw	a1,8(a0)
80005d64:	ac85000c 	sw	a1,12(a0)
80005d68:	ac850010 	sw	a1,16(a0)
80005d6c:	ac850014 	sw	a1,20(a0)
80005d70:	ac850018 	sw	a1,24(a0)
80005d74:	ac85001c 	sw	a1,28(a0)
80005d78:	ac850020 	sw	a1,32(a0)
80005d7c:	ac850024 	sw	a1,36(a0)
80005d80:	ac850028 	sw	a1,40(a0)
80005d84:	ac85002c 	sw	a1,44(a0)
80005d88:	ac850030 	sw	a1,48(a0)
80005d8c:	ac850034 	sw	a1,52(a0)
80005d90:	ac850038 	sw	a1,56(a0)
80005d94:	ac85003c 	sw	a1,60(a0)
80005d98:	24840040 	addiu	a0,a0,64
80005d9c:	1487ffee 	bne	a0,a3,80005d58 <memset+0x64>
80005da0:	00000000 	nop
80005da4:	03003025 	move	a2,t8
80005da8:	30d8001f 	andi	t8,a2,0x1f
80005dac:	10d8000a 	beq	a2,t8,80005dd8 <memset+0xe4>
80005db0:	00000000 	nop
80005db4:	ac850000 	sw	a1,0(a0)
80005db8:	ac850004 	sw	a1,4(a0)
80005dbc:	ac850008 	sw	a1,8(a0)
80005dc0:	ac85000c 	sw	a1,12(a0)
80005dc4:	ac850010 	sw	a1,16(a0)
80005dc8:	ac850014 	sw	a1,20(a0)
80005dcc:	ac850018 	sw	a1,24(a0)
80005dd0:	ac85001c 	sw	a1,28(a0)
80005dd4:	24840020 	addiu	a0,a0,32
80005dd8:	33060003 	andi	a2,t8,0x3
80005ddc:	10d80005 	beq	a2,t8,80005df4 <memset+0x100>
80005de0:	03063823 	subu	a3,t8,a2
80005de4:	00873821 	addu	a3,a0,a3
80005de8:	24840004 	addiu	a0,a0,4
80005dec:	1487fffe 	bne	a0,a3,80005de8 <memset+0xf4>
80005df0:	ac85fffc 	sw	a1,-4(a0)
80005df4:	18c00004 	blez	a2,80005e08 <memset+0x114>
80005df8:	00863821 	addu	a3,a0,a2
80005dfc:	24840001 	addiu	a0,a0,1
80005e00:	1487fffe 	bne	a0,a3,80005dfc <memset+0x108>
80005e04:	a085ffff 	sb	a1,-1(a0)
80005e08:	03e00008 	jr	ra
80005e0c:	00000000 	nop

80005e10 <validate_structure>:
80005e10:	27bdffb8 	addiu	sp,sp,-72
80005e14:	ffbf0040 	sd	ra,64(sp)
80005e18:	ffbe0038 	sd	s8,56(sp)
80005e1c:	03a0f025 	move	s8,sp
80005e20:	afc40048 	sw	a0,72(s8)
80005e24:	2402001c 	li	v0,28
80005e28:	afc20020 	sw	v0,32(s8)
80005e2c:	8fc20048 	lw	v0,72(s8)
80005e30:	8c420000 	lw	v0,0(v0)
80005e34:	04400006 	bltz	v0,80005e50 <validate_structure+0x40>
80005e38:	00000000 	nop
80005e3c:	8fc20048 	lw	v0,72(s8)
80005e40:	8c420000 	lw	v0,0(v0)
80005e44:	2842003c 	slti	v0,v0,60
80005e48:	14400020 	bnez	v0,80005ecc <validate_structure+0xbc>
80005e4c:	00000000 	nop
80005e50:	8fc20048 	lw	v0,72(s8)
80005e54:	8c420000 	lw	v0,0(v0)
80005e58:	27c30028 	addiu	v1,s8,40
80005e5c:	2406003c 	li	a2,60
80005e60:	00402825 	move	a1,v0
80005e64:	00602025 	move	a0,v1
80005e68:	0c002ef4 	jal	8000bbd0 <div>
80005e6c:	00000000 	nop
80005e70:	8fc20048 	lw	v0,72(s8)
80005e74:	8c430004 	lw	v1,4(v0)
80005e78:	8fc20028 	lw	v0,40(s8)
80005e7c:	00621821 	addu	v1,v1,v0
80005e80:	8fc20048 	lw	v0,72(s8)
80005e84:	ac430004 	sw	v1,4(v0)
80005e88:	8fc3002c 	lw	v1,44(s8)
80005e8c:	8fc20048 	lw	v0,72(s8)
80005e90:	ac430000 	sw	v1,0(v0)
80005e94:	8fc20048 	lw	v0,72(s8)
80005e98:	8c420000 	lw	v0,0(v0)
80005e9c:	0441000b 	bgez	v0,80005ecc <validate_structure+0xbc>
80005ea0:	00000000 	nop
80005ea4:	8fc20048 	lw	v0,72(s8)
80005ea8:	8c420000 	lw	v0,0(v0)
80005eac:	2443003c 	addiu	v1,v0,60
80005eb0:	8fc20048 	lw	v0,72(s8)
80005eb4:	ac430000 	sw	v1,0(v0)
80005eb8:	8fc20048 	lw	v0,72(s8)
80005ebc:	8c420004 	lw	v0,4(v0)
80005ec0:	2443ffff 	addiu	v1,v0,-1
80005ec4:	8fc20048 	lw	v0,72(s8)
80005ec8:	ac430004 	sw	v1,4(v0)
80005ecc:	8fc20048 	lw	v0,72(s8)
80005ed0:	8c420004 	lw	v0,4(v0)
80005ed4:	04400006 	bltz	v0,80005ef0 <validate_structure+0xe0>
80005ed8:	00000000 	nop
80005edc:	8fc20048 	lw	v0,72(s8)
80005ee0:	8c420004 	lw	v0,4(v0)
80005ee4:	2842003c 	slti	v0,v0,60
80005ee8:	14400023 	bnez	v0,80005f78 <validate_structure+0x168>
80005eec:	00000000 	nop
80005ef0:	8fc20048 	lw	v0,72(s8)
80005ef4:	8c420004 	lw	v0,4(v0)
80005ef8:	27c30030 	addiu	v1,s8,48
80005efc:	2406003c 	li	a2,60
80005f00:	00402825 	move	a1,v0
80005f04:	00602025 	move	a0,v1
80005f08:	0c002ef4 	jal	8000bbd0 <div>
80005f0c:	00000000 	nop
80005f10:	6bc20030 	ldl	v0,48(s8)
80005f14:	6fc20037 	ldr	v0,55(s8)
80005f18:	ffc20028 	sd	v0,40(s8)
80005f1c:	8fc20048 	lw	v0,72(s8)
80005f20:	8c430008 	lw	v1,8(v0)
80005f24:	8fc20028 	lw	v0,40(s8)
80005f28:	00621821 	addu	v1,v1,v0
80005f2c:	8fc20048 	lw	v0,72(s8)
80005f30:	ac430008 	sw	v1,8(v0)
80005f34:	8fc3002c 	lw	v1,44(s8)
80005f38:	8fc20048 	lw	v0,72(s8)
80005f3c:	ac430004 	sw	v1,4(v0)
80005f40:	8fc20048 	lw	v0,72(s8)
80005f44:	8c420004 	lw	v0,4(v0)
80005f48:	0441000b 	bgez	v0,80005f78 <validate_structure+0x168>
80005f4c:	00000000 	nop
80005f50:	8fc20048 	lw	v0,72(s8)
80005f54:	8c420004 	lw	v0,4(v0)
80005f58:	2443003c 	addiu	v1,v0,60
80005f5c:	8fc20048 	lw	v0,72(s8)
80005f60:	ac430004 	sw	v1,4(v0)
80005f64:	8fc20048 	lw	v0,72(s8)
80005f68:	8c420008 	lw	v0,8(v0)
80005f6c:	2443ffff 	addiu	v1,v0,-1
80005f70:	8fc20048 	lw	v0,72(s8)
80005f74:	ac430008 	sw	v1,8(v0)
80005f78:	8fc20048 	lw	v0,72(s8)
80005f7c:	8c420008 	lw	v0,8(v0)
80005f80:	04400006 	bltz	v0,80005f9c <validate_structure+0x18c>
80005f84:	00000000 	nop
80005f88:	8fc20048 	lw	v0,72(s8)
80005f8c:	8c420008 	lw	v0,8(v0)
80005f90:	28420018 	slti	v0,v0,24
80005f94:	14400023 	bnez	v0,80006024 <validate_structure+0x214>
80005f98:	00000000 	nop
80005f9c:	8fc20048 	lw	v0,72(s8)
80005fa0:	8c420008 	lw	v0,8(v0)
80005fa4:	27c30030 	addiu	v1,s8,48
80005fa8:	24060018 	li	a2,24
80005fac:	00402825 	move	a1,v0
80005fb0:	00602025 	move	a0,v1
80005fb4:	0c002ef4 	jal	8000bbd0 <div>
80005fb8:	00000000 	nop
80005fbc:	6bc20030 	ldl	v0,48(s8)
80005fc0:	6fc20037 	ldr	v0,55(s8)
80005fc4:	ffc20028 	sd	v0,40(s8)
80005fc8:	8fc20048 	lw	v0,72(s8)
80005fcc:	8c43000c 	lw	v1,12(v0)
80005fd0:	8fc20028 	lw	v0,40(s8)
80005fd4:	00621821 	addu	v1,v1,v0
80005fd8:	8fc20048 	lw	v0,72(s8)
80005fdc:	ac43000c 	sw	v1,12(v0)
80005fe0:	8fc3002c 	lw	v1,44(s8)
80005fe4:	8fc20048 	lw	v0,72(s8)
80005fe8:	ac430008 	sw	v1,8(v0)
80005fec:	8fc20048 	lw	v0,72(s8)
80005ff0:	8c420008 	lw	v0,8(v0)
80005ff4:	0441000b 	bgez	v0,80006024 <validate_structure+0x214>
80005ff8:	00000000 	nop
80005ffc:	8fc20048 	lw	v0,72(s8)
80006000:	8c420008 	lw	v0,8(v0)
80006004:	24430018 	addiu	v1,v0,24
80006008:	8fc20048 	lw	v0,72(s8)
8000600c:	ac430008 	sw	v1,8(v0)
80006010:	8fc20048 	lw	v0,72(s8)
80006014:	8c42000c 	lw	v0,12(v0)
80006018:	2443ffff 	addiu	v1,v0,-1
8000601c:	8fc20048 	lw	v0,72(s8)
80006020:	ac43000c 	sw	v1,12(v0)
80006024:	8fc20048 	lw	v0,72(s8)
80006028:	8c420010 	lw	v0,16(v0)
8000602c:	04400006 	bltz	v0,80006048 <validate_structure+0x238>
80006030:	00000000 	nop
80006034:	8fc20048 	lw	v0,72(s8)
80006038:	8c420010 	lw	v0,16(v0)
8000603c:	2842000c 	slti	v0,v0,12
80006040:	14400023 	bnez	v0,800060d0 <validate_structure+0x2c0>
80006044:	00000000 	nop
80006048:	8fc20048 	lw	v0,72(s8)
8000604c:	8c420010 	lw	v0,16(v0)
80006050:	27c30030 	addiu	v1,s8,48
80006054:	2406000c 	li	a2,12
80006058:	00402825 	move	a1,v0
8000605c:	00602025 	move	a0,v1
80006060:	0c002ef4 	jal	8000bbd0 <div>
80006064:	00000000 	nop
80006068:	6bc20030 	ldl	v0,48(s8)
8000606c:	6fc20037 	ldr	v0,55(s8)
80006070:	ffc20028 	sd	v0,40(s8)
80006074:	8fc20048 	lw	v0,72(s8)
80006078:	8c430014 	lw	v1,20(v0)
8000607c:	8fc20028 	lw	v0,40(s8)
80006080:	00621821 	addu	v1,v1,v0
80006084:	8fc20048 	lw	v0,72(s8)
80006088:	ac430014 	sw	v1,20(v0)
8000608c:	8fc3002c 	lw	v1,44(s8)
80006090:	8fc20048 	lw	v0,72(s8)
80006094:	ac430010 	sw	v1,16(v0)
80006098:	8fc20048 	lw	v0,72(s8)
8000609c:	8c420010 	lw	v0,16(v0)
800060a0:	0441000b 	bgez	v0,800060d0 <validate_structure+0x2c0>
800060a4:	00000000 	nop
800060a8:	8fc20048 	lw	v0,72(s8)
800060ac:	8c420010 	lw	v0,16(v0)
800060b0:	2443000c 	addiu	v1,v0,12
800060b4:	8fc20048 	lw	v0,72(s8)
800060b8:	ac430010 	sw	v1,16(v0)
800060bc:	8fc20048 	lw	v0,72(s8)
800060c0:	8c420014 	lw	v0,20(v0)
800060c4:	2443ffff 	addiu	v1,v0,-1
800060c8:	8fc20048 	lw	v0,72(s8)
800060cc:	ac430014 	sw	v1,20(v0)
800060d0:	8fc20048 	lw	v0,72(s8)
800060d4:	8c420014 	lw	v0,20(v0)
800060d8:	30420003 	andi	v0,v0,0x3
800060dc:	14400028 	bnez	v0,80006180 <validate_structure+0x370>
800060e0:	00000000 	nop
800060e4:	8fc20048 	lw	v0,72(s8)
800060e8:	8c440014 	lw	a0,20(v0)
800060ec:	3c0251eb 	lui	v0,0x51eb
800060f0:	3442851f 	ori	v0,v0,0x851f
800060f4:	00820018 	mult	a0,v0
800060f8:	00001010 	mfhi	v0
800060fc:	00021943 	sra	v1,v0,0x5
80006100:	000417c3 	sra	v0,a0,0x1f
80006104:	00621823 	subu	v1,v1,v0
80006108:	00601025 	move	v0,v1
8000610c:	00021040 	sll	v0,v0,0x1
80006110:	00431021 	addu	v0,v0,v1
80006114:	000210c0 	sll	v0,v0,0x3
80006118:	00431021 	addu	v0,v0,v1
8000611c:	00021080 	sll	v0,v0,0x2
80006120:	00821823 	subu	v1,a0,v0
80006124:	14600014 	bnez	v1,80006178 <validate_structure+0x368>
80006128:	00000000 	nop
8000612c:	8fc20048 	lw	v0,72(s8)
80006130:	8c420014 	lw	v0,20(v0)
80006134:	2444076c 	addiu	a0,v0,1900
80006138:	3c0251eb 	lui	v0,0x51eb
8000613c:	3442851f 	ori	v0,v0,0x851f
80006140:	00820018 	mult	a0,v0
80006144:	00001010 	mfhi	v0
80006148:	000219c3 	sra	v1,v0,0x7
8000614c:	000417c3 	sra	v0,a0,0x1f
80006150:	00621823 	subu	v1,v1,v0
80006154:	00601025 	move	v0,v1
80006158:	00021040 	sll	v0,v0,0x1
8000615c:	00431021 	addu	v0,v0,v1
80006160:	000210c0 	sll	v0,v0,0x3
80006164:	00431021 	addu	v0,v0,v1
80006168:	00021100 	sll	v0,v0,0x4
8000616c:	00821823 	subu	v1,a0,v0
80006170:	14600003 	bnez	v1,80006180 <validate_structure+0x370>
80006174:	00000000 	nop
80006178:	2402001d 	li	v0,29
8000617c:	afc20020 	sw	v0,32(s8)
80006180:	8fc20048 	lw	v0,72(s8)
80006184:	8c42000c 	lw	v0,12(v0)
80006188:	1c4000b2 	bgtz	v0,80006454 <validate_structure+0x644>
8000618c:	00000000 	nop
80006190:	10000056 	b	800062ec <validate_structure+0x4dc>
80006194:	00000000 	nop
80006198:	8fc20048 	lw	v0,72(s8)
8000619c:	8c420010 	lw	v0,16(v0)
800061a0:	2443ffff 	addiu	v1,v0,-1
800061a4:	8fc20048 	lw	v0,72(s8)
800061a8:	ac430010 	sw	v1,16(v0)
800061ac:	8fc20048 	lw	v0,72(s8)
800061b0:	8c430010 	lw	v1,16(v0)
800061b4:	2402ffff 	li	v0,-1
800061b8:	14620038 	bne	v1,v0,8000629c <validate_structure+0x48c>
800061bc:	00000000 	nop
800061c0:	8fc20048 	lw	v0,72(s8)
800061c4:	8c420014 	lw	v0,20(v0)
800061c8:	2443ffff 	addiu	v1,v0,-1
800061cc:	8fc20048 	lw	v0,72(s8)
800061d0:	ac430014 	sw	v1,20(v0)
800061d4:	8fc20048 	lw	v0,72(s8)
800061d8:	2403000b 	li	v1,11
800061dc:	ac430010 	sw	v1,16(v0)
800061e0:	8fc20048 	lw	v0,72(s8)
800061e4:	8c420014 	lw	v0,20(v0)
800061e8:	30420003 	andi	v0,v0,0x3
800061ec:	14400029 	bnez	v0,80006294 <validate_structure+0x484>
800061f0:	00000000 	nop
800061f4:	8fc20048 	lw	v0,72(s8)
800061f8:	8c440014 	lw	a0,20(v0)
800061fc:	3c0251eb 	lui	v0,0x51eb
80006200:	3442851f 	ori	v0,v0,0x851f
80006204:	00820018 	mult	a0,v0
80006208:	00001010 	mfhi	v0
8000620c:	00021943 	sra	v1,v0,0x5
80006210:	000417c3 	sra	v0,a0,0x1f
80006214:	00621823 	subu	v1,v1,v0
80006218:	00601025 	move	v0,v1
8000621c:	00021040 	sll	v0,v0,0x1
80006220:	00431021 	addu	v0,v0,v1
80006224:	000210c0 	sll	v0,v0,0x3
80006228:	00431021 	addu	v0,v0,v1
8000622c:	00021080 	sll	v0,v0,0x2
80006230:	00821823 	subu	v1,a0,v0
80006234:	14600014 	bnez	v1,80006288 <validate_structure+0x478>
80006238:	00000000 	nop
8000623c:	8fc20048 	lw	v0,72(s8)
80006240:	8c420014 	lw	v0,20(v0)
80006244:	2444076c 	addiu	a0,v0,1900
80006248:	3c0251eb 	lui	v0,0x51eb
8000624c:	3442851f 	ori	v0,v0,0x851f
80006250:	00820018 	mult	a0,v0
80006254:	00001010 	mfhi	v0
80006258:	000219c3 	sra	v1,v0,0x7
8000625c:	000417c3 	sra	v0,a0,0x1f
80006260:	00621823 	subu	v1,v1,v0
80006264:	00601025 	move	v0,v1
80006268:	00021040 	sll	v0,v0,0x1
8000626c:	00431021 	addu	v0,v0,v1
80006270:	000210c0 	sll	v0,v0,0x3
80006274:	00431021 	addu	v0,v0,v1
80006278:	00021100 	sll	v0,v0,0x4
8000627c:	00821823 	subu	v1,a0,v0
80006280:	14600004 	bnez	v1,80006294 <validate_structure+0x484>
80006284:	00000000 	nop
80006288:	2402001d 	li	v0,29
8000628c:	10000002 	b	80006298 <validate_structure+0x488>
80006290:	00000000 	nop
80006294:	2402001c 	li	v0,28
80006298:	afc20020 	sw	v0,32(s8)
8000629c:	8fc20048 	lw	v0,72(s8)
800062a0:	8c43000c 	lw	v1,12(v0)
800062a4:	8fc20048 	lw	v0,72(s8)
800062a8:	8c440010 	lw	a0,16(v0)
800062ac:	24020001 	li	v0,1
800062b0:	1082000a 	beq	a0,v0,800062dc <validate_structure+0x4cc>
800062b4:	00000000 	nop
800062b8:	8fc20048 	lw	v0,72(s8)
800062bc:	8c440010 	lw	a0,16(v0)
800062c0:	3c028002 	lui	v0,0x8002
800062c4:	00042080 	sll	a0,a0,0x2
800062c8:	2442a418 	addiu	v0,v0,-23528
800062cc:	00821021 	addu	v0,a0,v0
800062d0:	8c420000 	lw	v0,0(v0)
800062d4:	10000002 	b	800062e0 <validate_structure+0x4d0>
800062d8:	00000000 	nop
800062dc:	8fc20020 	lw	v0,32(s8)
800062e0:	00431821 	addu	v1,v0,v1
800062e4:	8fc20048 	lw	v0,72(s8)
800062e8:	ac43000c 	sw	v1,12(v0)
800062ec:	8fc20048 	lw	v0,72(s8)
800062f0:	8c42000c 	lw	v0,12(v0)
800062f4:	1840ffa8 	blez	v0,80006198 <validate_structure+0x388>
800062f8:	00000000 	nop
800062fc:	10000069 	b	800064a4 <validate_structure+0x694>
80006300:	00000000 	nop
80006304:	8fc20048 	lw	v0,72(s8)
80006308:	8c43000c 	lw	v1,12(v0)
8000630c:	8fc20048 	lw	v0,72(s8)
80006310:	8c440010 	lw	a0,16(v0)
80006314:	24020001 	li	v0,1
80006318:	1082000a 	beq	a0,v0,80006344 <validate_structure+0x534>
8000631c:	00000000 	nop
80006320:	8fc20048 	lw	v0,72(s8)
80006324:	8c440010 	lw	a0,16(v0)
80006328:	3c028002 	lui	v0,0x8002
8000632c:	00042080 	sll	a0,a0,0x2
80006330:	2442a418 	addiu	v0,v0,-23528
80006334:	00821021 	addu	v0,a0,v0
80006338:	8c420000 	lw	v0,0(v0)
8000633c:	10000002 	b	80006348 <validate_structure+0x538>
80006340:	00000000 	nop
80006344:	8fc20020 	lw	v0,32(s8)
80006348:	00621823 	subu	v1,v1,v0
8000634c:	8fc20048 	lw	v0,72(s8)
80006350:	ac43000c 	sw	v1,12(v0)
80006354:	8fc20048 	lw	v0,72(s8)
80006358:	8c420010 	lw	v0,16(v0)
8000635c:	24430001 	addiu	v1,v0,1
80006360:	8fc20048 	lw	v0,72(s8)
80006364:	ac430010 	sw	v1,16(v0)
80006368:	8fc20048 	lw	v0,72(s8)
8000636c:	8c430010 	lw	v1,16(v0)
80006370:	2402000c 	li	v0,12
80006374:	14620037 	bne	v1,v0,80006454 <validate_structure+0x644>
80006378:	00000000 	nop
8000637c:	8fc20048 	lw	v0,72(s8)
80006380:	8c420014 	lw	v0,20(v0)
80006384:	24430001 	addiu	v1,v0,1
80006388:	8fc20048 	lw	v0,72(s8)
8000638c:	ac430014 	sw	v1,20(v0)
80006390:	8fc20048 	lw	v0,72(s8)
80006394:	ac400010 	sw	zero,16(v0)
80006398:	8fc20048 	lw	v0,72(s8)
8000639c:	8c420014 	lw	v0,20(v0)
800063a0:	30420003 	andi	v0,v0,0x3
800063a4:	14400029 	bnez	v0,8000644c <validate_structure+0x63c>
800063a8:	00000000 	nop
800063ac:	8fc20048 	lw	v0,72(s8)
800063b0:	8c440014 	lw	a0,20(v0)
800063b4:	3c0251eb 	lui	v0,0x51eb
800063b8:	3442851f 	ori	v0,v0,0x851f
800063bc:	00820018 	mult	a0,v0
800063c0:	00001010 	mfhi	v0
800063c4:	00021943 	sra	v1,v0,0x5
800063c8:	000417c3 	sra	v0,a0,0x1f
800063cc:	00621823 	subu	v1,v1,v0
800063d0:	00601025 	move	v0,v1
800063d4:	00021040 	sll	v0,v0,0x1
800063d8:	00431021 	addu	v0,v0,v1
800063dc:	000210c0 	sll	v0,v0,0x3
800063e0:	00431021 	addu	v0,v0,v1
800063e4:	00021080 	sll	v0,v0,0x2
800063e8:	00821823 	subu	v1,a0,v0
800063ec:	14600014 	bnez	v1,80006440 <validate_structure+0x630>
800063f0:	00000000 	nop
800063f4:	8fc20048 	lw	v0,72(s8)
800063f8:	8c420014 	lw	v0,20(v0)
800063fc:	2444076c 	addiu	a0,v0,1900
80006400:	3c0251eb 	lui	v0,0x51eb
80006404:	3442851f 	ori	v0,v0,0x851f
80006408:	00820018 	mult	a0,v0
8000640c:	00001010 	mfhi	v0
80006410:	000219c3 	sra	v1,v0,0x7
80006414:	000417c3 	sra	v0,a0,0x1f
80006418:	00621823 	subu	v1,v1,v0
8000641c:	00601025 	move	v0,v1
80006420:	00021040 	sll	v0,v0,0x1
80006424:	00431021 	addu	v0,v0,v1
80006428:	000210c0 	sll	v0,v0,0x3
8000642c:	00431021 	addu	v0,v0,v1
80006430:	00021100 	sll	v0,v0,0x4
80006434:	00821823 	subu	v1,a0,v0
80006438:	14600004 	bnez	v1,8000644c <validate_structure+0x63c>
8000643c:	00000000 	nop
80006440:	2402001d 	li	v0,29
80006444:	10000002 	b	80006450 <validate_structure+0x640>
80006448:	00000000 	nop
8000644c:	2402001c 	li	v0,28
80006450:	afc20020 	sw	v0,32(s8)
80006454:	8fc20048 	lw	v0,72(s8)
80006458:	8c43000c 	lw	v1,12(v0)
8000645c:	8fc20048 	lw	v0,72(s8)
80006460:	8c440010 	lw	a0,16(v0)
80006464:	24020001 	li	v0,1
80006468:	1082000a 	beq	a0,v0,80006494 <validate_structure+0x684>
8000646c:	00000000 	nop
80006470:	8fc20048 	lw	v0,72(s8)
80006474:	8c440010 	lw	a0,16(v0)
80006478:	3c028002 	lui	v0,0x8002
8000647c:	00042080 	sll	a0,a0,0x2
80006480:	2442a418 	addiu	v0,v0,-23528
80006484:	00821021 	addu	v0,a0,v0
80006488:	8c420000 	lw	v0,0(v0)
8000648c:	10000002 	b	80006498 <validate_structure+0x688>
80006490:	00000000 	nop
80006494:	8fc20020 	lw	v0,32(s8)
80006498:	0043102a 	slt	v0,v0,v1
8000649c:	1440ff99 	bnez	v0,80006304 <validate_structure+0x4f4>
800064a0:	00000000 	nop
800064a4:	00000000 	nop
800064a8:	03c0e825 	move	sp,s8
800064ac:	dfbf0040 	ld	ra,64(sp)
800064b0:	dfbe0038 	ld	s8,56(sp)
800064b4:	27bd0048 	addiu	sp,sp,72
800064b8:	03e00008 	jr	ra
800064bc:	00000000 	nop

800064c0 <mktime>:
800064c0:	27bdff90 	addiu	sp,sp,-112
800064c4:	ffbf0068 	sd	ra,104(sp)
800064c8:	ffbe0060 	sd	s8,96(sp)
800064cc:	03a0f025 	move	s8,sp
800064d0:	afc40070 	sw	a0,112(s8)
800064d4:	ffc00020 	sd	zero,32(s8)
800064d8:	afc00028 	sw	zero,40(s8)
800064dc:	afc00030 	sw	zero,48(s8)
800064e0:	0c003983 	jal	8000e60c <__gettzinfo>
800064e4:	00000000 	nop
800064e8:	afc2003c 	sw	v0,60(s8)
800064ec:	8fc40070 	lw	a0,112(s8)
800064f0:	0c001784 	jal	80005e10 <validate_structure>
800064f4:	00000000 	nop
800064f8:	8fc20070 	lw	v0,112(s8)
800064fc:	8c440000 	lw	a0,0(v0)
80006500:	8fc20070 	lw	v0,112(s8)
80006504:	8c430004 	lw	v1,4(v0)
80006508:	00601025 	move	v0,v1
8000650c:	00021100 	sll	v0,v0,0x4
80006510:	00431023 	subu	v0,v0,v1
80006514:	00021080 	sll	v0,v0,0x2
80006518:	00822021 	addu	a0,a0,v0
8000651c:	8fc20070 	lw	v0,112(s8)
80006520:	8c430008 	lw	v1,8(v0)
80006524:	00601025 	move	v0,v1
80006528:	000210c0 	sll	v0,v0,0x3
8000652c:	00431023 	subu	v0,v0,v1
80006530:	00021140 	sll	v0,v0,0x5
80006534:	00431021 	addu	v0,v0,v1
80006538:	00021100 	sll	v0,v0,0x4
8000653c:	00821021 	addu	v0,a0,v0
80006540:	dfc30020 	ld	v1,32(s8)
80006544:	0062102d 	daddu	v0,v1,v0
80006548:	ffc20020 	sd	v0,32(s8)
8000654c:	8fc20070 	lw	v0,112(s8)
80006550:	8c42000c 	lw	v0,12(v0)
80006554:	2442ffff 	addiu	v0,v0,-1
80006558:	8fc30028 	lw	v1,40(s8)
8000655c:	00621021 	addu	v0,v1,v0
80006560:	afc20028 	sw	v0,40(s8)
80006564:	8fc20070 	lw	v0,112(s8)
80006568:	8c430010 	lw	v1,16(v0)
8000656c:	3c028002 	lui	v0,0x8002
80006570:	00031880 	sll	v1,v1,0x2
80006574:	2442a448 	addiu	v0,v0,-23480
80006578:	00621021 	addu	v0,v1,v0
8000657c:	8c420000 	lw	v0,0(v0)
80006580:	8fc30028 	lw	v1,40(s8)
80006584:	00621021 	addu	v0,v1,v0
80006588:	afc20028 	sw	v0,40(s8)
8000658c:	8fc20070 	lw	v0,112(s8)
80006590:	8c420010 	lw	v0,16(v0)
80006594:	28420002 	slti	v0,v0,2
80006598:	1440002e 	bnez	v0,80006654 <mktime+0x194>
8000659c:	00000000 	nop
800065a0:	8fc20070 	lw	v0,112(s8)
800065a4:	8c420014 	lw	v0,20(v0)
800065a8:	30420003 	andi	v0,v0,0x3
800065ac:	14400029 	bnez	v0,80006654 <mktime+0x194>
800065b0:	00000000 	nop
800065b4:	8fc20070 	lw	v0,112(s8)
800065b8:	8c440014 	lw	a0,20(v0)
800065bc:	3c0251eb 	lui	v0,0x51eb
800065c0:	3442851f 	ori	v0,v0,0x851f
800065c4:	00820018 	mult	a0,v0
800065c8:	00001010 	mfhi	v0
800065cc:	00021943 	sra	v1,v0,0x5
800065d0:	000417c3 	sra	v0,a0,0x1f
800065d4:	00621823 	subu	v1,v1,v0
800065d8:	00601025 	move	v0,v1
800065dc:	00021040 	sll	v0,v0,0x1
800065e0:	00431021 	addu	v0,v0,v1
800065e4:	000210c0 	sll	v0,v0,0x3
800065e8:	00431021 	addu	v0,v0,v1
800065ec:	00021080 	sll	v0,v0,0x2
800065f0:	00821823 	subu	v1,a0,v0
800065f4:	14600014 	bnez	v1,80006648 <mktime+0x188>
800065f8:	00000000 	nop
800065fc:	8fc20070 	lw	v0,112(s8)
80006600:	8c420014 	lw	v0,20(v0)
80006604:	2444076c 	addiu	a0,v0,1900
80006608:	3c0251eb 	lui	v0,0x51eb
8000660c:	3442851f 	ori	v0,v0,0x851f
80006610:	00820018 	mult	a0,v0
80006614:	00001010 	mfhi	v0
80006618:	000219c3 	sra	v1,v0,0x7
8000661c:	000417c3 	sra	v0,a0,0x1f
80006620:	00621823 	subu	v1,v1,v0
80006624:	00601025 	move	v0,v1
80006628:	00021040 	sll	v0,v0,0x1
8000662c:	00431021 	addu	v0,v0,v1
80006630:	000210c0 	sll	v0,v0,0x3
80006634:	00431021 	addu	v0,v0,v1
80006638:	00021100 	sll	v0,v0,0x4
8000663c:	00821823 	subu	v1,a0,v0
80006640:	14600004 	bnez	v1,80006654 <mktime+0x194>
80006644:	00000000 	nop
80006648:	8fc20028 	lw	v0,40(s8)
8000664c:	24420001 	addiu	v0,v0,1
80006650:	afc20028 	sw	v0,40(s8)
80006654:	8fc20070 	lw	v0,112(s8)
80006658:	8fc30028 	lw	v1,40(s8)
8000665c:	ac43001c 	sw	v1,28(v0)
80006660:	8fc20070 	lw	v0,112(s8)
80006664:	8c420014 	lw	v0,20(v0)
80006668:	28422711 	slti	v0,v0,10001
8000666c:	10400006 	beqz	v0,80006688 <mktime+0x1c8>
80006670:	00000000 	nop
80006674:	8fc20070 	lw	v0,112(s8)
80006678:	8c420014 	lw	v0,20(v0)
8000667c:	2842d8f0 	slti	v0,v0,-10000
80006680:	10400004 	beqz	v0,80006694 <mktime+0x1d4>
80006684:	00000000 	nop
80006688:	2402ffff 	li	v0,-1
8000668c:	100001fd 	b	80006e84 <mktime+0x9c4>
80006690:	00000000 	nop
80006694:	8fc20070 	lw	v0,112(s8)
80006698:	8c420014 	lw	v0,20(v0)
8000669c:	afc2002c 	sw	v0,44(s8)
800066a0:	8fc2002c 	lw	v0,44(s8)
800066a4:	28420047 	slti	v0,v0,71
800066a8:	1440003e 	bnez	v0,800067a4 <mktime+0x2e4>
800066ac:	00000000 	nop
800066b0:	24020046 	li	v0,70
800066b4:	afc2002c 	sw	v0,44(s8)
800066b8:	10000032 	b	80006784 <mktime+0x2c4>
800066bc:	00000000 	nop
800066c0:	8fc2002c 	lw	v0,44(s8)
800066c4:	30420003 	andi	v0,v0,0x3
800066c8:	14400027 	bnez	v0,80006768 <mktime+0x2a8>
800066cc:	00000000 	nop
800066d0:	8fc4002c 	lw	a0,44(s8)
800066d4:	3c0251eb 	lui	v0,0x51eb
800066d8:	3442851f 	ori	v0,v0,0x851f
800066dc:	00820018 	mult	a0,v0
800066e0:	00001010 	mfhi	v0
800066e4:	00021943 	sra	v1,v0,0x5
800066e8:	000417c3 	sra	v0,a0,0x1f
800066ec:	00621823 	subu	v1,v1,v0
800066f0:	00601025 	move	v0,v1
800066f4:	00021040 	sll	v0,v0,0x1
800066f8:	00431021 	addu	v0,v0,v1
800066fc:	000210c0 	sll	v0,v0,0x3
80006700:	00431021 	addu	v0,v0,v1
80006704:	00021080 	sll	v0,v0,0x2
80006708:	00821823 	subu	v1,a0,v0
8000670c:	14600013 	bnez	v1,8000675c <mktime+0x29c>
80006710:	00000000 	nop
80006714:	8fc2002c 	lw	v0,44(s8)
80006718:	2444076c 	addiu	a0,v0,1900
8000671c:	3c0251eb 	lui	v0,0x51eb
80006720:	3442851f 	ori	v0,v0,0x851f
80006724:	00820018 	mult	a0,v0
80006728:	00001010 	mfhi	v0
8000672c:	000219c3 	sra	v1,v0,0x7
80006730:	000417c3 	sra	v0,a0,0x1f
80006734:	00621823 	subu	v1,v1,v0
80006738:	00601025 	move	v0,v1
8000673c:	00021040 	sll	v0,v0,0x1
80006740:	00431021 	addu	v0,v0,v1
80006744:	000210c0 	sll	v0,v0,0x3
80006748:	00431021 	addu	v0,v0,v1
8000674c:	00021100 	sll	v0,v0,0x4
80006750:	00821823 	subu	v1,a0,v0
80006754:	14600004 	bnez	v1,80006768 <mktime+0x2a8>
80006758:	00000000 	nop
8000675c:	2402016e 	li	v0,366
80006760:	10000002 	b	8000676c <mktime+0x2ac>
80006764:	00000000 	nop
80006768:	2402016d 	li	v0,365
8000676c:	8fc30028 	lw	v1,40(s8)
80006770:	00621021 	addu	v0,v1,v0
80006774:	afc20028 	sw	v0,40(s8)
80006778:	8fc2002c 	lw	v0,44(s8)
8000677c:	24420001 	addiu	v0,v0,1
80006780:	afc2002c 	sw	v0,44(s8)
80006784:	8fc20070 	lw	v0,112(s8)
80006788:	8c420014 	lw	v0,20(v0)
8000678c:	8fc3002c 	lw	v1,44(s8)
80006790:	0062102a 	slt	v0,v1,v0
80006794:	1440ffca 	bnez	v0,800066c0 <mktime+0x200>
80006798:	00000000 	nop
8000679c:	1000006e 	b	80006958 <mktime+0x498>
800067a0:	00000000 	nop
800067a4:	8fc2002c 	lw	v0,44(s8)
800067a8:	28420046 	slti	v0,v0,70
800067ac:	1040006a 	beqz	v0,80006958 <mktime+0x498>
800067b0:	00000000 	nop
800067b4:	24020045 	li	v0,69
800067b8:	afc2002c 	sw	v0,44(s8)
800067bc:	10000032 	b	80006888 <mktime+0x3c8>
800067c0:	00000000 	nop
800067c4:	8fc2002c 	lw	v0,44(s8)
800067c8:	30420003 	andi	v0,v0,0x3
800067cc:	14400027 	bnez	v0,8000686c <mktime+0x3ac>
800067d0:	00000000 	nop
800067d4:	8fc4002c 	lw	a0,44(s8)
800067d8:	3c0251eb 	lui	v0,0x51eb
800067dc:	3442851f 	ori	v0,v0,0x851f
800067e0:	00820018 	mult	a0,v0
800067e4:	00001010 	mfhi	v0
800067e8:	00021943 	sra	v1,v0,0x5
800067ec:	000417c3 	sra	v0,a0,0x1f
800067f0:	00621823 	subu	v1,v1,v0
800067f4:	00601025 	move	v0,v1
800067f8:	00021040 	sll	v0,v0,0x1
800067fc:	00431021 	addu	v0,v0,v1
80006800:	000210c0 	sll	v0,v0,0x3
80006804:	00431021 	addu	v0,v0,v1
80006808:	00021080 	sll	v0,v0,0x2
8000680c:	00821823 	subu	v1,a0,v0
80006810:	14600013 	bnez	v1,80006860 <mktime+0x3a0>
80006814:	00000000 	nop
80006818:	8fc2002c 	lw	v0,44(s8)
8000681c:	2444076c 	addiu	a0,v0,1900
80006820:	3c0251eb 	lui	v0,0x51eb
80006824:	3442851f 	ori	v0,v0,0x851f
80006828:	00820018 	mult	a0,v0
8000682c:	00001010 	mfhi	v0
80006830:	000219c3 	sra	v1,v0,0x7
80006834:	000417c3 	sra	v0,a0,0x1f
80006838:	00621823 	subu	v1,v1,v0
8000683c:	00601025 	move	v0,v1
80006840:	00021040 	sll	v0,v0,0x1
80006844:	00431021 	addu	v0,v0,v1
80006848:	000210c0 	sll	v0,v0,0x3
8000684c:	00431021 	addu	v0,v0,v1
80006850:	00021100 	sll	v0,v0,0x4
80006854:	00821823 	subu	v1,a0,v0
80006858:	14600004 	bnez	v1,8000686c <mktime+0x3ac>
8000685c:	00000000 	nop
80006860:	2402016e 	li	v0,366
80006864:	10000002 	b	80006870 <mktime+0x3b0>
80006868:	00000000 	nop
8000686c:	2402016d 	li	v0,365
80006870:	8fc30028 	lw	v1,40(s8)
80006874:	00621023 	subu	v0,v1,v0
80006878:	afc20028 	sw	v0,40(s8)
8000687c:	8fc2002c 	lw	v0,44(s8)
80006880:	2442ffff 	addiu	v0,v0,-1
80006884:	afc2002c 	sw	v0,44(s8)
80006888:	8fc20070 	lw	v0,112(s8)
8000688c:	8c420014 	lw	v0,20(v0)
80006890:	8fc3002c 	lw	v1,44(s8)
80006894:	0043102a 	slt	v0,v0,v1
80006898:	1440ffca 	bnez	v0,800067c4 <mktime+0x304>
8000689c:	00000000 	nop
800068a0:	8fc2002c 	lw	v0,44(s8)
800068a4:	30420003 	andi	v0,v0,0x3
800068a8:	14400027 	bnez	v0,80006948 <mktime+0x488>
800068ac:	00000000 	nop
800068b0:	8fc4002c 	lw	a0,44(s8)
800068b4:	3c0251eb 	lui	v0,0x51eb
800068b8:	3442851f 	ori	v0,v0,0x851f
800068bc:	00820018 	mult	a0,v0
800068c0:	00001010 	mfhi	v0
800068c4:	00021943 	sra	v1,v0,0x5
800068c8:	000417c3 	sra	v0,a0,0x1f
800068cc:	00621823 	subu	v1,v1,v0
800068d0:	00601025 	move	v0,v1
800068d4:	00021040 	sll	v0,v0,0x1
800068d8:	00431021 	addu	v0,v0,v1
800068dc:	000210c0 	sll	v0,v0,0x3
800068e0:	00431021 	addu	v0,v0,v1
800068e4:	00021080 	sll	v0,v0,0x2
800068e8:	00821823 	subu	v1,a0,v0
800068ec:	14600013 	bnez	v1,8000693c <mktime+0x47c>
800068f0:	00000000 	nop
800068f4:	8fc2002c 	lw	v0,44(s8)
800068f8:	2444076c 	addiu	a0,v0,1900
800068fc:	3c0251eb 	lui	v0,0x51eb
80006900:	3442851f 	ori	v0,v0,0x851f
80006904:	00820018 	mult	a0,v0
80006908:	00001010 	mfhi	v0
8000690c:	000219c3 	sra	v1,v0,0x7
80006910:	000417c3 	sra	v0,a0,0x1f
80006914:	00621823 	subu	v1,v1,v0
80006918:	00601025 	move	v0,v1
8000691c:	00021040 	sll	v0,v0,0x1
80006920:	00431021 	addu	v0,v0,v1
80006924:	000210c0 	sll	v0,v0,0x3
80006928:	00431021 	addu	v0,v0,v1
8000692c:	00021100 	sll	v0,v0,0x4
80006930:	00821823 	subu	v1,a0,v0
80006934:	14600004 	bnez	v1,80006948 <mktime+0x488>
80006938:	00000000 	nop
8000693c:	2402016e 	li	v0,366
80006940:	10000002 	b	8000694c <mktime+0x48c>
80006944:	00000000 	nop
80006948:	2402016d 	li	v0,365
8000694c:	8fc30028 	lw	v1,40(s8)
80006950:	00621023 	subu	v0,v1,v0
80006954:	afc20028 	sw	v0,40(s8)
80006958:	8fc30028 	lw	v1,40(s8)
8000695c:	00601025 	move	v0,v1
80006960:	00021078 	dsll	v0,v0,0x1
80006964:	0043102d 	daddu	v0,v0,v1
80006968:	00021938 	dsll	v1,v0,0x4
8000696c:	0062182f 	dsubu	v1,v1,v0
80006970:	00031138 	dsll	v0,v1,0x4
80006974:	0043102f 	dsubu	v0,v0,v1
80006978:	000211f8 	dsll	v0,v0,0x7
8000697c:	00401825 	move	v1,v0
80006980:	dfc20020 	ld	v0,32(s8)
80006984:	0043102d 	daddu	v0,v0,v1
80006988:	ffc20020 	sd	v0,32(s8)
8000698c:	0c0020ad 	jal	800082b4 <__tz_lock>
80006990:	00000000 	nop
80006994:	0c0020bf 	jal	800082fc <_tzset_unlocked>
80006998:	00000000 	nop
8000699c:	8f82833c 	lw	v0,-31940(gp)
800069a0:	10400109 	beqz	v0,80006dc8 <mktime+0x908>
800069a4:	00000000 	nop
800069a8:	8fc20070 	lw	v0,112(s8)
800069ac:	8c420014 	lw	v0,20(v0)
800069b0:	2442076c 	addiu	v0,v0,1900
800069b4:	afc20040 	sw	v0,64(s8)
800069b8:	8fc20070 	lw	v0,112(s8)
800069bc:	8c420020 	lw	v0,32(v0)
800069c0:	18400002 	blez	v0,800069cc <mktime+0x50c>
800069c4:	00000000 	nop
800069c8:	24020001 	li	v0,1
800069cc:	afc20044 	sw	v0,68(s8)
800069d0:	8fc20044 	lw	v0,68(s8)
800069d4:	afc20030 	sw	v0,48(s8)
800069d8:	8fc2003c 	lw	v0,60(s8)
800069dc:	8c420004 	lw	v0,4(v0)
800069e0:	8fc30040 	lw	v1,64(s8)
800069e4:	10620006 	beq	v1,v0,80006a00 <mktime+0x540>
800069e8:	00000000 	nop
800069ec:	8fc40040 	lw	a0,64(s8)
800069f0:	0c001f34 	jal	80007cd0 <__tzcalc_limits>
800069f4:	00000000 	nop
800069f8:	104000f3 	beqz	v0,80006dc8 <mktime+0x908>
800069fc:	00000000 	nop
80006a00:	8fc2003c 	lw	v0,60(s8)
80006a04:	dc430020 	ld	v1,32(v0)
80006a08:	8fc2003c 	lw	v0,60(s8)
80006a0c:	8c420050 	lw	v0,80(v0)
80006a10:	0062102f 	dsubu	v0,v1,v0
80006a14:	ffc20048 	sd	v0,72(s8)
80006a18:	8fc2003c 	lw	v0,60(s8)
80006a1c:	dc430048 	ld	v1,72(v0)
80006a20:	8fc2003c 	lw	v0,60(s8)
80006a24:	8c420050 	lw	v0,80(v0)
80006a28:	0062102f 	dsubu	v0,v1,v0
80006a2c:	ffc20050 	sd	v0,80(s8)
80006a30:	8fc2003c 	lw	v0,60(s8)
80006a34:	dc430048 	ld	v1,72(v0)
80006a38:	8fc2003c 	lw	v0,60(s8)
80006a3c:	8c420028 	lw	v0,40(v0)
80006a40:	0062102f 	dsubu	v0,v1,v0
80006a44:	ffc20058 	sd	v0,88(s8)
80006a48:	dfc30020 	ld	v1,32(s8)
80006a4c:	dfc20058 	ld	v0,88(s8)
80006a50:	0062102a 	slt	v0,v1,v0
80006a54:	14400006 	bnez	v0,80006a70 <mktime+0x5b0>
80006a58:	00000000 	nop
80006a5c:	dfc30020 	ld	v1,32(s8)
80006a60:	dfc20050 	ld	v0,80(s8)
80006a64:	0062102a 	slt	v0,v1,v0
80006a68:	144000d7 	bnez	v0,80006dc8 <mktime+0x908>
80006a6c:	00000000 	nop
80006a70:	8fc2003c 	lw	v0,60(s8)
80006a74:	8c420000 	lw	v0,0(v0)
80006a78:	10400011 	beqz	v0,80006ac0 <mktime+0x600>
80006a7c:	00000000 	nop
80006a80:	dfc30020 	ld	v1,32(s8)
80006a84:	dfc20048 	ld	v0,72(s8)
80006a88:	0062102a 	slt	v0,v1,v0
80006a8c:	14400009 	bnez	v0,80006ab4 <mktime+0x5f4>
80006a90:	00000000 	nop
80006a94:	dfc30020 	ld	v1,32(s8)
80006a98:	dfc20058 	ld	v0,88(s8)
80006a9c:	0062102a 	slt	v0,v1,v0
80006aa0:	10400004 	beqz	v0,80006ab4 <mktime+0x5f4>
80006aa4:	00000000 	nop
80006aa8:	24020001 	li	v0,1
80006aac:	10000012 	b	80006af8 <mktime+0x638>
80006ab0:	00000000 	nop
80006ab4:	00001025 	move	v0,zero
80006ab8:	1000000f 	b	80006af8 <mktime+0x638>
80006abc:	00000000 	nop
80006ac0:	dfc30020 	ld	v1,32(s8)
80006ac4:	dfc20048 	ld	v0,72(s8)
80006ac8:	0062102a 	slt	v0,v1,v0
80006acc:	10400006 	beqz	v0,80006ae8 <mktime+0x628>
80006ad0:	00000000 	nop
80006ad4:	dfc30020 	ld	v1,32(s8)
80006ad8:	dfc20058 	ld	v0,88(s8)
80006adc:	0062102a 	slt	v0,v1,v0
80006ae0:	10400004 	beqz	v0,80006af4 <mktime+0x634>
80006ae4:	00000000 	nop
80006ae8:	24020001 	li	v0,1
80006aec:	10000002 	b	80006af8 <mktime+0x638>
80006af0:	00000000 	nop
80006af4:	00001025 	move	v0,zero
80006af8:	afc20030 	sw	v0,48(s8)
80006afc:	8fc20044 	lw	v0,68(s8)
80006b00:	044000b1 	bltz	v0,80006dc8 <mktime+0x908>
80006b04:	00000000 	nop
80006b08:	8fc30030 	lw	v1,48(s8)
80006b0c:	8fc20044 	lw	v0,68(s8)
80006b10:	00621826 	xor	v1,v1,v0
80006b14:	24020001 	li	v0,1
80006b18:	146200ab 	bne	v1,v0,80006dc8 <mktime+0x908>
80006b1c:	00000000 	nop
80006b20:	8fc2003c 	lw	v0,60(s8)
80006b24:	8c430028 	lw	v1,40(v0)
80006b28:	8fc2003c 	lw	v0,60(s8)
80006b2c:	8c420050 	lw	v0,80(v0)
80006b30:	00621023 	subu	v0,v1,v0
80006b34:	afc20034 	sw	v0,52(s8)
80006b38:	8fc20030 	lw	v0,48(s8)
80006b3c:	14400004 	bnez	v0,80006b50 <mktime+0x690>
80006b40:	00000000 	nop
80006b44:	8fc20034 	lw	v0,52(s8)
80006b48:	00021023 	negu	v0,v0
80006b4c:	afc20034 	sw	v0,52(s8)
80006b50:	8fc20070 	lw	v0,112(s8)
80006b54:	8c430000 	lw	v1,0(v0)
80006b58:	8fc20034 	lw	v0,52(s8)
80006b5c:	00621821 	addu	v1,v1,v0
80006b60:	8fc20070 	lw	v0,112(s8)
80006b64:	ac430000 	sw	v1,0(v0)
80006b68:	8fc20034 	lw	v0,52(s8)
80006b6c:	dfc30020 	ld	v1,32(s8)
80006b70:	0062102d 	daddu	v0,v1,v0
80006b74:	ffc20020 	sd	v0,32(s8)
80006b78:	8fc20070 	lw	v0,112(s8)
80006b7c:	8c42000c 	lw	v0,12(v0)
80006b80:	afc20038 	sw	v0,56(s8)
80006b84:	8fc40070 	lw	a0,112(s8)
80006b88:	0c001784 	jal	80005e10 <validate_structure>
80006b8c:	00000000 	nop
80006b90:	8fc20070 	lw	v0,112(s8)
80006b94:	8c43000c 	lw	v1,12(v0)
80006b98:	8fc20038 	lw	v0,56(s8)
80006b9c:	00621023 	subu	v0,v1,v0
80006ba0:	afc20038 	sw	v0,56(s8)
80006ba4:	8fc20038 	lw	v0,56(s8)
80006ba8:	10400087 	beqz	v0,80006dc8 <mktime+0x908>
80006bac:	00000000 	nop
80006bb0:	8fc20038 	lw	v0,56(s8)
80006bb4:	28420002 	slti	v0,v0,2
80006bb8:	14400005 	bnez	v0,80006bd0 <mktime+0x710>
80006bbc:	00000000 	nop
80006bc0:	2402ffff 	li	v0,-1
80006bc4:	afc20038 	sw	v0,56(s8)
80006bc8:	10000007 	b	80006be8 <mktime+0x728>
80006bcc:	00000000 	nop
80006bd0:	8fc20038 	lw	v0,56(s8)
80006bd4:	2842ffff 	slti	v0,v0,-1
80006bd8:	10400003 	beqz	v0,80006be8 <mktime+0x728>
80006bdc:	00000000 	nop
80006be0:	24020001 	li	v0,1
80006be4:	afc20038 	sw	v0,56(s8)
80006be8:	8fc30028 	lw	v1,40(s8)
80006bec:	8fc20038 	lw	v0,56(s8)
80006bf0:	00621021 	addu	v0,v1,v0
80006bf4:	afc20028 	sw	v0,40(s8)
80006bf8:	8fc20070 	lw	v0,112(s8)
80006bfc:	8c43001c 	lw	v1,28(v0)
80006c00:	8fc20038 	lw	v0,56(s8)
80006c04:	00621821 	addu	v1,v1,v0
80006c08:	8fc20070 	lw	v0,112(s8)
80006c0c:	ac43001c 	sw	v1,28(v0)
80006c10:	8fc20070 	lw	v0,112(s8)
80006c14:	8c42001c 	lw	v0,28(v0)
80006c18:	04410033 	bgez	v0,80006ce8 <mktime+0x828>
80006c1c:	00000000 	nop
80006c20:	8fc2002c 	lw	v0,44(s8)
80006c24:	2442ffff 	addiu	v0,v0,-1
80006c28:	afc2002c 	sw	v0,44(s8)
80006c2c:	8fc2002c 	lw	v0,44(s8)
80006c30:	30420003 	andi	v0,v0,0x3
80006c34:	14400027 	bnez	v0,80006cd4 <mktime+0x814>
80006c38:	00000000 	nop
80006c3c:	8fc4002c 	lw	a0,44(s8)
80006c40:	3c0251eb 	lui	v0,0x51eb
80006c44:	3442851f 	ori	v0,v0,0x851f
80006c48:	00820018 	mult	a0,v0
80006c4c:	00001010 	mfhi	v0
80006c50:	00021943 	sra	v1,v0,0x5
80006c54:	000417c3 	sra	v0,a0,0x1f
80006c58:	00621823 	subu	v1,v1,v0
80006c5c:	00601025 	move	v0,v1
80006c60:	00021040 	sll	v0,v0,0x1
80006c64:	00431021 	addu	v0,v0,v1
80006c68:	000210c0 	sll	v0,v0,0x3
80006c6c:	00431021 	addu	v0,v0,v1
80006c70:	00021080 	sll	v0,v0,0x2
80006c74:	00821823 	subu	v1,a0,v0
80006c78:	14600013 	bnez	v1,80006cc8 <mktime+0x808>
80006c7c:	00000000 	nop
80006c80:	8fc2002c 	lw	v0,44(s8)
80006c84:	2444076c 	addiu	a0,v0,1900
80006c88:	3c0251eb 	lui	v0,0x51eb
80006c8c:	3442851f 	ori	v0,v0,0x851f
80006c90:	00820018 	mult	a0,v0
80006c94:	00001010 	mfhi	v0
80006c98:	000219c3 	sra	v1,v0,0x7
80006c9c:	000417c3 	sra	v0,a0,0x1f
80006ca0:	00621823 	subu	v1,v1,v0
80006ca4:	00601025 	move	v0,v1
80006ca8:	00021040 	sll	v0,v0,0x1
80006cac:	00431021 	addu	v0,v0,v1
80006cb0:	000210c0 	sll	v0,v0,0x3
80006cb4:	00431021 	addu	v0,v0,v1
80006cb8:	00021100 	sll	v0,v0,0x4
80006cbc:	00821823 	subu	v1,a0,v0
80006cc0:	14600004 	bnez	v1,80006cd4 <mktime+0x814>
80006cc4:	00000000 	nop
80006cc8:	2402016d 	li	v0,365
80006ccc:	10000002 	b	80006cd8 <mktime+0x818>
80006cd0:	00000000 	nop
80006cd4:	2402016c 	li	v0,364
80006cd8:	8fc30070 	lw	v1,112(s8)
80006cdc:	ac62001c 	sw	v0,28(v1)
80006ce0:	10000039 	b	80006dc8 <mktime+0x908>
80006ce4:	00000000 	nop
80006ce8:	8fc2002c 	lw	v0,44(s8)
80006cec:	30420003 	andi	v0,v0,0x3
80006cf0:	14400027 	bnez	v0,80006d90 <mktime+0x8d0>
80006cf4:	00000000 	nop
80006cf8:	8fc4002c 	lw	a0,44(s8)
80006cfc:	3c0251eb 	lui	v0,0x51eb
80006d00:	3442851f 	ori	v0,v0,0x851f
80006d04:	00820018 	mult	a0,v0
80006d08:	00001010 	mfhi	v0
80006d0c:	00021943 	sra	v1,v0,0x5
80006d10:	000417c3 	sra	v0,a0,0x1f
80006d14:	00621823 	subu	v1,v1,v0
80006d18:	00601025 	move	v0,v1
80006d1c:	00021040 	sll	v0,v0,0x1
80006d20:	00431021 	addu	v0,v0,v1
80006d24:	000210c0 	sll	v0,v0,0x3
80006d28:	00431021 	addu	v0,v0,v1
80006d2c:	00021080 	sll	v0,v0,0x2
80006d30:	00821823 	subu	v1,a0,v0
80006d34:	14600013 	bnez	v1,80006d84 <mktime+0x8c4>
80006d38:	00000000 	nop
80006d3c:	8fc2002c 	lw	v0,44(s8)
80006d40:	2444076c 	addiu	a0,v0,1900
80006d44:	3c0251eb 	lui	v0,0x51eb
80006d48:	3442851f 	ori	v0,v0,0x851f
80006d4c:	00820018 	mult	a0,v0
80006d50:	00001010 	mfhi	v0
80006d54:	000219c3 	sra	v1,v0,0x7
80006d58:	000417c3 	sra	v0,a0,0x1f
80006d5c:	00621823 	subu	v1,v1,v0
80006d60:	00601025 	move	v0,v1
80006d64:	00021040 	sll	v0,v0,0x1
80006d68:	00431021 	addu	v0,v0,v1
80006d6c:	000210c0 	sll	v0,v0,0x3
80006d70:	00431021 	addu	v0,v0,v1
80006d74:	00021100 	sll	v0,v0,0x4
80006d78:	00821823 	subu	v1,a0,v0
80006d7c:	14600004 	bnez	v1,80006d90 <mktime+0x8d0>
80006d80:	00000000 	nop
80006d84:	2402016e 	li	v0,366
80006d88:	10000002 	b	80006d94 <mktime+0x8d4>
80006d8c:	00000000 	nop
80006d90:	2402016d 	li	v0,365
80006d94:	afc20038 	sw	v0,56(s8)
80006d98:	8fc20070 	lw	v0,112(s8)
80006d9c:	8c42001c 	lw	v0,28(v0)
80006da0:	8fc30038 	lw	v1,56(s8)
80006da4:	0043102a 	slt	v0,v0,v1
80006da8:	14400007 	bnez	v0,80006dc8 <mktime+0x908>
80006dac:	00000000 	nop
80006db0:	8fc20070 	lw	v0,112(s8)
80006db4:	8c43001c 	lw	v1,28(v0)
80006db8:	8fc20038 	lw	v0,56(s8)
80006dbc:	00621823 	subu	v1,v1,v0
80006dc0:	8fc20070 	lw	v0,112(s8)
80006dc4:	ac43001c 	sw	v1,28(v0)
80006dc8:	8fc30030 	lw	v1,48(s8)
80006dcc:	24020001 	li	v0,1
80006dd0:	14620008 	bne	v1,v0,80006df4 <mktime+0x934>
80006dd4:	00000000 	nop
80006dd8:	8fc2003c 	lw	v0,60(s8)
80006ddc:	8c420050 	lw	v0,80(v0)
80006de0:	dfc30020 	ld	v1,32(s8)
80006de4:	0062102d 	daddu	v0,v1,v0
80006de8:	ffc20020 	sd	v0,32(s8)
80006dec:	10000006 	b	80006e08 <mktime+0x948>
80006df0:	00000000 	nop
80006df4:	8fc2003c 	lw	v0,60(s8)
80006df8:	8c420028 	lw	v0,40(v0)
80006dfc:	dfc30020 	ld	v1,32(s8)
80006e00:	0062102d 	daddu	v0,v1,v0
80006e04:	ffc20020 	sd	v0,32(s8)
80006e08:	0c0020b6 	jal	800082d8 <__tz_unlock>
80006e0c:	00000000 	nop
80006e10:	8fc20070 	lw	v0,112(s8)
80006e14:	8fc30030 	lw	v1,48(s8)
80006e18:	ac430020 	sw	v1,32(v0)
80006e1c:	8fc20028 	lw	v0,40(s8)
80006e20:	24420004 	addiu	v0,v0,4
80006e24:	3c039249 	lui	v1,0x9249
80006e28:	34632493 	ori	v1,v1,0x2493
80006e2c:	00430018 	mult	v0,v1
80006e30:	00001810 	mfhi	v1
80006e34:	00621821 	addu	v1,v1,v0
80006e38:	00032083 	sra	a0,v1,0x2
80006e3c:	00021fc3 	sra	v1,v0,0x1f
80006e40:	00832023 	subu	a0,a0,v1
80006e44:	00801825 	move	v1,a0
80006e48:	000318c0 	sll	v1,v1,0x3
80006e4c:	00641823 	subu	v1,v1,a0
80006e50:	00432023 	subu	a0,v0,v1
80006e54:	8fc20070 	lw	v0,112(s8)
80006e58:	ac440018 	sw	a0,24(v0)
80006e5c:	8fc20070 	lw	v0,112(s8)
80006e60:	8c420018 	lw	v0,24(v0)
80006e64:	04410006 	bgez	v0,80006e80 <mktime+0x9c0>
80006e68:	00000000 	nop
80006e6c:	8fc20070 	lw	v0,112(s8)
80006e70:	8c420018 	lw	v0,24(v0)
80006e74:	24430007 	addiu	v1,v0,7
80006e78:	8fc20070 	lw	v0,112(s8)
80006e7c:	ac430018 	sw	v1,24(v0)
80006e80:	dfc20020 	ld	v0,32(s8)
80006e84:	03c0e825 	move	sp,s8
80006e88:	dfbf0068 	ld	ra,104(sp)
80006e8c:	dfbe0060 	ld	s8,96(sp)
80006e90:	27bd0070 	addiu	sp,sp,112
80006e94:	03e00008 	jr	ra
80006e98:	00000000 	nop

80006e9c <__malloc_lock>:
80006e9c:	27bdfff8 	addiu	sp,sp,-8
80006ea0:	ffbe0000 	sd	s8,0(sp)
80006ea4:	03a0f025 	move	s8,sp
80006ea8:	afc40008 	sw	a0,8(s8)
80006eac:	00000000 	nop
80006eb0:	03c0e825 	move	sp,s8
80006eb4:	dfbe0000 	ld	s8,0(sp)
80006eb8:	27bd0008 	addiu	sp,sp,8
80006ebc:	03e00008 	jr	ra
80006ec0:	00000000 	nop

80006ec4 <__malloc_unlock>:
80006ec4:	27bdfff8 	addiu	sp,sp,-8
80006ec8:	ffbe0000 	sd	s8,0(sp)
80006ecc:	03a0f025 	move	s8,sp
80006ed0:	afc40008 	sw	a0,8(s8)
80006ed4:	00000000 	nop
80006ed8:	03c0e825 	move	sp,s8
80006edc:	dfbe0000 	ld	s8,0(sp)
80006ee0:	27bd0008 	addiu	sp,sp,8
80006ee4:	03e00008 	jr	ra
80006ee8:	00000000 	nop

80006eec <_printf_r>:
80006eec:	27bdffc8 	addiu	sp,sp,-56
80006ef0:	ffbf0030 	sd	ra,48(sp)
80006ef4:	ffbe0028 	sd	s8,40(sp)
80006ef8:	03a0f025 	move	s8,sp
80006efc:	afc40038 	sw	a0,56(s8)
80006f00:	afc50040 	sw	a1,64(s8)
80006f04:	ffc60048 	sd	a2,72(s8)
80006f08:	ffc70050 	sd	a3,80(s8)
80006f0c:	27c20048 	addiu	v0,s8,72
80006f10:	afc20024 	sw	v0,36(s8)
80006f14:	8fc20038 	lw	v0,56(s8)
80006f18:	8c420008 	lw	v0,8(v0)
80006f1c:	8fc30024 	lw	v1,36(s8)
80006f20:	00603825 	move	a3,v1
80006f24:	8fc60040 	lw	a2,64(s8)
80006f28:	00402825 	move	a1,v0
80006f2c:	8fc40038 	lw	a0,56(s8)
80006f30:	0c002372 	jal	80008dc8 <_vfprintf_r>
80006f34:	00000000 	nop
80006f38:	afc20020 	sw	v0,32(s8)
80006f3c:	8fc20020 	lw	v0,32(s8)
80006f40:	03c0e825 	move	sp,s8
80006f44:	dfbf0030 	ld	ra,48(sp)
80006f48:	dfbe0028 	ld	s8,40(sp)
80006f4c:	27bd0038 	addiu	sp,sp,56
80006f50:	03e00008 	jr	ra
80006f54:	00000000 	nop

80006f58 <printf>:
80006f58:	27bdffc0 	addiu	sp,sp,-64
80006f5c:	ffbf0038 	sd	ra,56(sp)
80006f60:	ffbe0030 	sd	s8,48(sp)
80006f64:	03a0f025 	move	s8,sp
80006f68:	afc40040 	sw	a0,64(s8)
80006f6c:	ffc50048 	sd	a1,72(s8)
80006f70:	ffc60050 	sd	a2,80(s8)
80006f74:	ffc70058 	sd	a3,88(s8)
80006f78:	8f828028 	lw	v0,-32728(gp)
80006f7c:	afc20020 	sw	v0,32(s8)
80006f80:	27c20048 	addiu	v0,s8,72
80006f84:	afc20028 	sw	v0,40(s8)
80006f88:	8fc20020 	lw	v0,32(s8)
80006f8c:	8c420008 	lw	v0,8(v0)
80006f90:	8fc30028 	lw	v1,40(s8)
80006f94:	00603825 	move	a3,v1
80006f98:	8fc60040 	lw	a2,64(s8)
80006f9c:	00402825 	move	a1,v0
80006fa0:	8fc40020 	lw	a0,32(s8)
80006fa4:	0c002372 	jal	80008dc8 <_vfprintf_r>
80006fa8:	00000000 	nop
80006fac:	afc20024 	sw	v0,36(s8)
80006fb0:	8fc20024 	lw	v0,36(s8)
80006fb4:	03c0e825 	move	sp,s8
80006fb8:	dfbf0038 	ld	ra,56(sp)
80006fbc:	dfbe0030 	ld	s8,48(sp)
80006fc0:	27bd0040 	addiu	sp,sp,64
80006fc4:	03e00008 	jr	ra
80006fc8:	00000000 	nop

80006fcc <_putchar_r>:
80006fcc:	27bdffd0 	addiu	sp,sp,-48
80006fd0:	ffbf0028 	sd	ra,40(sp)
80006fd4:	ffbe0020 	sd	s8,32(sp)
80006fd8:	03a0f025 	move	s8,sp
80006fdc:	afc40030 	sw	a0,48(s8)
80006fe0:	afc50038 	sw	a1,56(s8)
80006fe4:	8fc20030 	lw	v0,48(s8)
80006fe8:	8c420008 	lw	v0,8(v0)
80006fec:	00403025 	move	a2,v0
80006ff0:	8fc50038 	lw	a1,56(s8)
80006ff4:	8fc40030 	lw	a0,48(s8)
80006ff8:	0c001c4d 	jal	80007134 <_putc_r>
80006ffc:	00000000 	nop
80007000:	03c0e825 	move	sp,s8
80007004:	dfbf0028 	ld	ra,40(sp)
80007008:	dfbe0020 	ld	s8,32(sp)
8000700c:	27bd0030 	addiu	sp,sp,48
80007010:	03e00008 	jr	ra
80007014:	00000000 	nop

80007018 <putchar>:
80007018:	27bdffc8 	addiu	sp,sp,-56
8000701c:	ffbf0030 	sd	ra,48(sp)
80007020:	ffbe0028 	sd	s8,40(sp)
80007024:	03a0f025 	move	s8,sp
80007028:	afc40038 	sw	a0,56(s8)
8000702c:	8f828028 	lw	v0,-32728(gp)
80007030:	afc20020 	sw	v0,32(s8)
80007034:	8fc20020 	lw	v0,32(s8)
80007038:	8c420008 	lw	v0,8(v0)
8000703c:	00403025 	move	a2,v0
80007040:	8fc50038 	lw	a1,56(s8)
80007044:	8fc40020 	lw	a0,32(s8)
80007048:	0c001c4d 	jal	80007134 <_putc_r>
8000704c:	00000000 	nop
80007050:	03c0e825 	move	sp,s8
80007054:	dfbf0030 	ld	ra,48(sp)
80007058:	dfbe0028 	ld	s8,40(sp)
8000705c:	27bd0038 	addiu	sp,sp,56
80007060:	03e00008 	jr	ra
80007064:	00000000 	nop

80007068 <__sputc_r>:
80007068:	27bdffd0 	addiu	sp,sp,-48
8000706c:	ffbf0028 	sd	ra,40(sp)
80007070:	ffbe0020 	sd	s8,32(sp)
80007074:	03a0f025 	move	s8,sp
80007078:	afc40030 	sw	a0,48(s8)
8000707c:	afc50038 	sw	a1,56(s8)
80007080:	afc60040 	sw	a2,64(s8)
80007084:	8fc20040 	lw	v0,64(s8)
80007088:	8c420008 	lw	v0,8(v0)
8000708c:	2443ffff 	addiu	v1,v0,-1
80007090:	8fc20040 	lw	v0,64(s8)
80007094:	ac430008 	sw	v1,8(v0)
80007098:	8fc20040 	lw	v0,64(s8)
8000709c:	8c420008 	lw	v0,8(v0)
800070a0:	0441000e 	bgez	v0,800070dc <__sputc_r+0x74>
800070a4:	00000000 	nop
800070a8:	8fc20040 	lw	v0,64(s8)
800070ac:	8c430008 	lw	v1,8(v0)
800070b0:	8fc20040 	lw	v0,64(s8)
800070b4:	8c420018 	lw	v0,24(v0)
800070b8:	0062102a 	slt	v0,v1,v0
800070bc:	14400012 	bnez	v0,80007108 <__sputc_r+0xa0>
800070c0:	00000000 	nop
800070c4:	8fc20038 	lw	v0,56(s8)
800070c8:	00021e00 	sll	v1,v0,0x18
800070cc:	00031e03 	sra	v1,v1,0x18
800070d0:	2402000a 	li	v0,10
800070d4:	1062000c 	beq	v1,v0,80007108 <__sputc_r+0xa0>
800070d8:	00000000 	nop
800070dc:	8fc20040 	lw	v0,64(s8)
800070e0:	8c420000 	lw	v0,0(v0)
800070e4:	24440001 	addiu	a0,v0,1
800070e8:	8fc30040 	lw	v1,64(s8)
800070ec:	ac640000 	sw	a0,0(v1)
800070f0:	8fc30038 	lw	v1,56(s8)
800070f4:	306300ff 	andi	v1,v1,0xff
800070f8:	a0430000 	sb	v1,0(v0)
800070fc:	90420000 	lbu	v0,0(v0)
80007100:	10000006 	b	8000711c <__sputc_r+0xb4>
80007104:	00000000 	nop
80007108:	8fc60040 	lw	a2,64(s8)
8000710c:	8fc50038 	lw	a1,56(s8)
80007110:	8fc40030 	lw	a0,48(s8)
80007114:	0c002daf 	jal	8000b6bc <__swbuf_r>
80007118:	00000000 	nop
8000711c:	03c0e825 	move	sp,s8
80007120:	dfbf0028 	ld	ra,40(sp)
80007124:	dfbe0020 	ld	s8,32(sp)
80007128:	27bd0030 	addiu	sp,sp,48
8000712c:	03e00008 	jr	ra
80007130:	00000000 	nop

80007134 <_putc_r>:
80007134:	27bdffc0 	addiu	sp,sp,-64
80007138:	ffbf0038 	sd	ra,56(sp)
8000713c:	ffbe0030 	sd	s8,48(sp)
80007140:	ffb00028 	sd	s0,40(sp)
80007144:	03a0f025 	move	s8,sp
80007148:	afc40040 	sw	a0,64(s8)
8000714c:	afc50048 	sw	a1,72(s8)
80007150:	00c08025 	move	s0,a2
80007154:	8fc20040 	lw	v0,64(s8)
80007158:	afc20020 	sw	v0,32(s8)
8000715c:	8fc20020 	lw	v0,32(s8)
80007160:	10400008 	beqz	v0,80007184 <_putc_r+0x50>
80007164:	00000000 	nop
80007168:	8fc20020 	lw	v0,32(s8)
8000716c:	8c420038 	lw	v0,56(v0)
80007170:	14400004 	bnez	v0,80007184 <_putc_r+0x50>
80007174:	00000000 	nop
80007178:	8fc40020 	lw	a0,32(s8)
8000717c:	0c000cc3 	jal	8000330c <__sinit>
80007180:	00000000 	nop
80007184:	02003025 	move	a2,s0
80007188:	8fc50048 	lw	a1,72(s8)
8000718c:	8fc40040 	lw	a0,64(s8)
80007190:	0c001c1a 	jal	80007068 <__sputc_r>
80007194:	00000000 	nop
80007198:	afc20024 	sw	v0,36(s8)
8000719c:	8fc20024 	lw	v0,36(s8)
800071a0:	03c0e825 	move	sp,s8
800071a4:	dfbf0038 	ld	ra,56(sp)
800071a8:	dfbe0030 	ld	s8,48(sp)
800071ac:	dfb00028 	ld	s0,40(sp)
800071b0:	27bd0040 	addiu	sp,sp,64
800071b4:	03e00008 	jr	ra
800071b8:	00000000 	nop

800071bc <putc>:
800071bc:	27bdffb8 	addiu	sp,sp,-72
800071c0:	ffbf0040 	sd	ra,64(sp)
800071c4:	ffbe0038 	sd	s8,56(sp)
800071c8:	ffb00030 	sd	s0,48(sp)
800071cc:	03a0f025 	move	s8,sp
800071d0:	afc40048 	sw	a0,72(s8)
800071d4:	00a08025 	move	s0,a1
800071d8:	8f828028 	lw	v0,-32728(gp)
800071dc:	afc20020 	sw	v0,32(s8)
800071e0:	8fc20020 	lw	v0,32(s8)
800071e4:	afc20024 	sw	v0,36(s8)
800071e8:	8fc20024 	lw	v0,36(s8)
800071ec:	10400008 	beqz	v0,80007210 <putc+0x54>
800071f0:	00000000 	nop
800071f4:	8fc20024 	lw	v0,36(s8)
800071f8:	8c420038 	lw	v0,56(v0)
800071fc:	14400004 	bnez	v0,80007210 <putc+0x54>
80007200:	00000000 	nop
80007204:	8fc40024 	lw	a0,36(s8)
80007208:	0c000cc3 	jal	8000330c <__sinit>
8000720c:	00000000 	nop
80007210:	02003025 	move	a2,s0
80007214:	8fc50048 	lw	a1,72(s8)
80007218:	8fc40020 	lw	a0,32(s8)
8000721c:	0c001c1a 	jal	80007068 <__sputc_r>
80007220:	00000000 	nop
80007224:	afc20028 	sw	v0,40(s8)
80007228:	8fc20028 	lw	v0,40(s8)
8000722c:	03c0e825 	move	sp,s8
80007230:	dfbf0040 	ld	ra,64(sp)
80007234:	dfbe0038 	ld	s8,56(sp)
80007238:	dfb00030 	ld	s0,48(sp)
8000723c:	27bd0048 	addiu	sp,sp,72
80007240:	03e00008 	jr	ra
80007244:	00000000 	nop

80007248 <_puts_r>:
80007248:	27bdffa0 	addiu	sp,sp,-96
8000724c:	ffbf0058 	sd	ra,88(sp)
80007250:	ffbe0050 	sd	s8,80(sp)
80007254:	03a0f025 	move	s8,sp
80007258:	afc40060 	sw	a0,96(s8)
8000725c:	afc50068 	sw	a1,104(s8)
80007260:	8fc40068 	lw	a0,104(s8)
80007264:	0c001f1d 	jal	80007c74 <strlen>
80007268:	00000000 	nop
8000726c:	afc20020 	sw	v0,32(s8)
80007270:	8fc20068 	lw	v0,104(s8)
80007274:	afc20040 	sw	v0,64(s8)
80007278:	8fc20020 	lw	v0,32(s8)
8000727c:	afc20044 	sw	v0,68(s8)
80007280:	3c028002 	lui	v0,0x8002
80007284:	2442a478 	addiu	v0,v0,-23432
80007288:	afc20048 	sw	v0,72(s8)
8000728c:	24020001 	li	v0,1
80007290:	afc2004c 	sw	v0,76(s8)
80007294:	8fc20020 	lw	v0,32(s8)
80007298:	24420001 	addiu	v0,v0,1
8000729c:	afc20038 	sw	v0,56(s8)
800072a0:	27c20040 	addiu	v0,s8,64
800072a4:	afc20030 	sw	v0,48(s8)
800072a8:	24020002 	li	v0,2
800072ac:	afc20034 	sw	v0,52(s8)
800072b0:	8fc20060 	lw	v0,96(s8)
800072b4:	8c420008 	lw	v0,8(v0)
800072b8:	afc20024 	sw	v0,36(s8)
800072bc:	8fc20060 	lw	v0,96(s8)
800072c0:	afc20028 	sw	v0,40(s8)
800072c4:	8fc20028 	lw	v0,40(s8)
800072c8:	10400008 	beqz	v0,800072ec <_puts_r+0xa4>
800072cc:	00000000 	nop
800072d0:	8fc20028 	lw	v0,40(s8)
800072d4:	8c420038 	lw	v0,56(v0)
800072d8:	14400004 	bnez	v0,800072ec <_puts_r+0xa4>
800072dc:	00000000 	nop
800072e0:	8fc40028 	lw	a0,40(s8)
800072e4:	0c000cc3 	jal	8000330c <__sinit>
800072e8:	00000000 	nop
800072ec:	8fc20024 	lw	v0,36(s8)
800072f0:	8442000c 	lh	v0,12(v0)
800072f4:	3042ffff 	andi	v0,v0,0xffff
800072f8:	30422000 	andi	v0,v0,0x2000
800072fc:	1440000e 	bnez	v0,80007338 <_puts_r+0xf0>
80007300:	00000000 	nop
80007304:	8fc20024 	lw	v0,36(s8)
80007308:	8442000c 	lh	v0,12(v0)
8000730c:	34422000 	ori	v0,v0,0x2000
80007310:	00021c00 	sll	v1,v0,0x10
80007314:	00031c03 	sra	v1,v1,0x10
80007318:	8fc20024 	lw	v0,36(s8)
8000731c:	a443000c 	sh	v1,12(v0)
80007320:	8fc20024 	lw	v0,36(s8)
80007324:	8c430064 	lw	v1,100(v0)
80007328:	2402dfff 	li	v0,-8193
8000732c:	00621824 	and	v1,v1,v0
80007330:	8fc20024 	lw	v0,36(s8)
80007334:	ac430064 	sw	v1,100(v0)
80007338:	27c20030 	addiu	v0,s8,48
8000733c:	00403025 	move	a2,v0
80007340:	8fc50024 	lw	a1,36(s8)
80007344:	8fc40060 	lw	a0,96(s8)
80007348:	0c003737 	jal	8000dcdc <__sfvwrite_r>
8000734c:	00000000 	nop
80007350:	10400004 	beqz	v0,80007364 <_puts_r+0x11c>
80007354:	00000000 	nop
80007358:	2402ffff 	li	v0,-1
8000735c:	10000002 	b	80007368 <_puts_r+0x120>
80007360:	00000000 	nop
80007364:	2402000a 	li	v0,10
80007368:	afc2002c 	sw	v0,44(s8)
8000736c:	8fc2002c 	lw	v0,44(s8)
80007370:	03c0e825 	move	sp,s8
80007374:	dfbf0058 	ld	ra,88(sp)
80007378:	dfbe0050 	ld	s8,80(sp)
8000737c:	27bd0060 	addiu	sp,sp,96
80007380:	03e00008 	jr	ra
80007384:	00000000 	nop

80007388 <puts>:
80007388:	27bdffd0 	addiu	sp,sp,-48
8000738c:	ffbf0028 	sd	ra,40(sp)
80007390:	ffbe0020 	sd	s8,32(sp)
80007394:	03a0f025 	move	s8,sp
80007398:	afc40030 	sw	a0,48(s8)
8000739c:	8f828028 	lw	v0,-32728(gp)
800073a0:	8fc50030 	lw	a1,48(s8)
800073a4:	00402025 	move	a0,v0
800073a8:	0c001c92 	jal	80007248 <_puts_r>
800073ac:	00000000 	nop
800073b0:	03c0e825 	move	sp,s8
800073b4:	dfbf0028 	ld	ra,40(sp)
800073b8:	dfbe0020 	ld	s8,32(sp)
800073bc:	27bd0030 	addiu	sp,sp,48
800073c0:	03e00008 	jr	ra
800073c4:	00000000 	nop

800073c8 <_sbrk_r>:
800073c8:	27bdffc8 	addiu	sp,sp,-56
800073cc:	ffbf0030 	sd	ra,48(sp)
800073d0:	ffbe0028 	sd	s8,40(sp)
800073d4:	03a0f025 	move	s8,sp
800073d8:	afc40038 	sw	a0,56(s8)
800073dc:	afc50040 	sw	a1,64(s8)
800073e0:	af808334 	sw	zero,-31948(gp)
800073e4:	8fc40040 	lw	a0,64(s8)
800073e8:	0c006836 	jal	8001a0d8 <sbrk>
800073ec:	00000000 	nop
800073f0:	afc20020 	sw	v0,32(s8)
800073f4:	8fc30020 	lw	v1,32(s8)
800073f8:	2402ffff 	li	v0,-1
800073fc:	14620007 	bne	v1,v0,8000741c <_sbrk_r+0x54>
80007400:	00000000 	nop
80007404:	8f828334 	lw	v0,-31948(gp)
80007408:	10400004 	beqz	v0,8000741c <_sbrk_r+0x54>
8000740c:	00000000 	nop
80007410:	8f838334 	lw	v1,-31948(gp)
80007414:	8fc20038 	lw	v0,56(s8)
80007418:	ac430000 	sw	v1,0(v0)
8000741c:	8fc20020 	lw	v0,32(s8)
80007420:	03c0e825 	move	sp,s8
80007424:	dfbf0030 	ld	ra,48(sp)
80007428:	dfbe0028 	ld	s8,40(sp)
8000742c:	27bd0038 	addiu	sp,sp,56
80007430:	03e00008 	jr	ra
80007434:	00000000 	nop

80007438 <_init_signal_r>:
80007438:	27bdffc8 	addiu	sp,sp,-56
8000743c:	ffbf0030 	sd	ra,48(sp)
80007440:	ffbe0028 	sd	s8,40(sp)
80007444:	03a0f025 	move	s8,sp
80007448:	afc40038 	sw	a0,56(s8)
8000744c:	8fc20038 	lw	v0,56(s8)
80007450:	8c4202dc 	lw	v0,732(v0)
80007454:	1440001f 	bnez	v0,800074d4 <_init_signal_r+0x9c>
80007458:	00000000 	nop
8000745c:	24050080 	li	a1,128
80007460:	8fc40038 	lw	a0,56(s8)
80007464:	0c001296 	jal	80004a58 <_malloc_r>
80007468:	00000000 	nop
8000746c:	00401825 	move	v1,v0
80007470:	8fc20038 	lw	v0,56(s8)
80007474:	ac4302dc 	sw	v1,732(v0)
80007478:	8fc20038 	lw	v0,56(s8)
8000747c:	8c4202dc 	lw	v0,732(v0)
80007480:	14400004 	bnez	v0,80007494 <_init_signal_r+0x5c>
80007484:	00000000 	nop
80007488:	2402ffff 	li	v0,-1
8000748c:	10000012 	b	800074d8 <_init_signal_r+0xa0>
80007490:	00000000 	nop
80007494:	afc00020 	sw	zero,32(s8)
80007498:	1000000a 	b	800074c4 <_init_signal_r+0x8c>
8000749c:	00000000 	nop
800074a0:	8fc20038 	lw	v0,56(s8)
800074a4:	8c4302dc 	lw	v1,732(v0)
800074a8:	8fc20020 	lw	v0,32(s8)
800074ac:	00021080 	sll	v0,v0,0x2
800074b0:	00621021 	addu	v0,v1,v0
800074b4:	ac400000 	sw	zero,0(v0)
800074b8:	8fc20020 	lw	v0,32(s8)
800074bc:	24420001 	addiu	v0,v0,1
800074c0:	afc20020 	sw	v0,32(s8)
800074c4:	8fc20020 	lw	v0,32(s8)
800074c8:	28420020 	slti	v0,v0,32
800074cc:	1440fff4 	bnez	v0,800074a0 <_init_signal_r+0x68>
800074d0:	00000000 	nop
800074d4:	00001025 	move	v0,zero
800074d8:	03c0e825 	move	sp,s8
800074dc:	dfbf0030 	ld	ra,48(sp)
800074e0:	dfbe0028 	ld	s8,40(sp)
800074e4:	27bd0038 	addiu	sp,sp,56
800074e8:	03e00008 	jr	ra
800074ec:	00000000 	nop

800074f0 <_signal_r>:
800074f0:	27bdffc8 	addiu	sp,sp,-56
800074f4:	ffbf0030 	sd	ra,48(sp)
800074f8:	ffbe0028 	sd	s8,40(sp)
800074fc:	03a0f025 	move	s8,sp
80007500:	afc40038 	sw	a0,56(s8)
80007504:	afc50040 	sw	a1,64(s8)
80007508:	afc60048 	sw	a2,72(s8)
8000750c:	8fc20040 	lw	v0,64(s8)
80007510:	04400005 	bltz	v0,80007528 <_signal_r+0x38>
80007514:	00000000 	nop
80007518:	8fc20040 	lw	v0,64(s8)
8000751c:	28420020 	slti	v0,v0,32
80007520:	14400007 	bnez	v0,80007540 <_signal_r+0x50>
80007524:	00000000 	nop
80007528:	8fc20038 	lw	v0,56(s8)
8000752c:	24030016 	li	v1,22
80007530:	ac430000 	sw	v1,0(v0)
80007534:	2402ffff 	li	v0,-1
80007538:	1000001c 	b	800075ac <_signal_r+0xbc>
8000753c:	00000000 	nop
80007540:	8fc20038 	lw	v0,56(s8)
80007544:	8c4202dc 	lw	v0,732(v0)
80007548:	14400009 	bnez	v0,80007570 <_signal_r+0x80>
8000754c:	00000000 	nop
80007550:	8fc40038 	lw	a0,56(s8)
80007554:	0c001d0e 	jal	80007438 <_init_signal_r>
80007558:	00000000 	nop
8000755c:	10400004 	beqz	v0,80007570 <_signal_r+0x80>
80007560:	00000000 	nop
80007564:	2402ffff 	li	v0,-1
80007568:	10000010 	b	800075ac <_signal_r+0xbc>
8000756c:	00000000 	nop
80007570:	8fc20038 	lw	v0,56(s8)
80007574:	8c4302dc 	lw	v1,732(v0)
80007578:	8fc20040 	lw	v0,64(s8)
8000757c:	00021080 	sll	v0,v0,0x2
80007580:	00621021 	addu	v0,v1,v0
80007584:	8c420000 	lw	v0,0(v0)
80007588:	afc20020 	sw	v0,32(s8)
8000758c:	8fc20038 	lw	v0,56(s8)
80007590:	8c4302dc 	lw	v1,732(v0)
80007594:	8fc20040 	lw	v0,64(s8)
80007598:	00021080 	sll	v0,v0,0x2
8000759c:	00621021 	addu	v0,v1,v0
800075a0:	8fc30048 	lw	v1,72(s8)
800075a4:	ac430000 	sw	v1,0(v0)
800075a8:	8fc20020 	lw	v0,32(s8)
800075ac:	03c0e825 	move	sp,s8
800075b0:	dfbf0030 	ld	ra,48(sp)
800075b4:	dfbe0028 	ld	s8,40(sp)
800075b8:	27bd0038 	addiu	sp,sp,56
800075bc:	03e00008 	jr	ra
800075c0:	00000000 	nop

800075c4 <_raise_r>:
800075c4:	27bdffc8 	addiu	sp,sp,-56
800075c8:	ffbf0030 	sd	ra,48(sp)
800075cc:	ffbe0028 	sd	s8,40(sp)
800075d0:	03a0f025 	move	s8,sp
800075d4:	afc40038 	sw	a0,56(s8)
800075d8:	afc50040 	sw	a1,64(s8)
800075dc:	8fc20040 	lw	v0,64(s8)
800075e0:	04400005 	bltz	v0,800075f8 <_raise_r+0x34>
800075e4:	00000000 	nop
800075e8:	8fc20040 	lw	v0,64(s8)
800075ec:	28420020 	slti	v0,v0,32
800075f0:	14400007 	bnez	v0,80007610 <_raise_r+0x4c>
800075f4:	00000000 	nop
800075f8:	8fc20038 	lw	v0,56(s8)
800075fc:	24030016 	li	v1,22
80007600:	ac430000 	sw	v1,0(v0)
80007604:	2402ffff 	li	v0,-1
80007608:	10000038 	b	800076ec <_raise_r+0x128>
8000760c:	00000000 	nop
80007610:	8fc20038 	lw	v0,56(s8)
80007614:	8c4202dc 	lw	v0,732(v0)
80007618:	14400004 	bnez	v0,8000762c <_raise_r+0x68>
8000761c:	00000000 	nop
80007620:	afc00020 	sw	zero,32(s8)
80007624:	10000008 	b	80007648 <_raise_r+0x84>
80007628:	00000000 	nop
8000762c:	8fc20038 	lw	v0,56(s8)
80007630:	8c4302dc 	lw	v1,732(v0)
80007634:	8fc20040 	lw	v0,64(s8)
80007638:	00021080 	sll	v0,v0,0x2
8000763c:	00621021 	addu	v0,v1,v0
80007640:	8c420000 	lw	v0,0(v0)
80007644:	afc20020 	sw	v0,32(s8)
80007648:	8fc20020 	lw	v0,32(s8)
8000764c:	1440000b 	bnez	v0,8000767c <_raise_r+0xb8>
80007650:	00000000 	nop
80007654:	8fc40038 	lw	a0,56(s8)
80007658:	0c001e67 	jal	8000799c <_getpid_r>
8000765c:	00000000 	nop
80007660:	8fc60040 	lw	a2,64(s8)
80007664:	00402825 	move	a1,v0
80007668:	8fc40038 	lw	a0,56(s8)
8000766c:	0c001e49 	jal	80007924 <_kill_r>
80007670:	00000000 	nop
80007674:	1000001d 	b	800076ec <_raise_r+0x128>
80007678:	00000000 	nop
8000767c:	8fc30020 	lw	v1,32(s8)
80007680:	24020001 	li	v0,1
80007684:	14620004 	bne	v1,v0,80007698 <_raise_r+0xd4>
80007688:	00000000 	nop
8000768c:	00001025 	move	v0,zero
80007690:	10000016 	b	800076ec <_raise_r+0x128>
80007694:	00000000 	nop
80007698:	8fc30020 	lw	v1,32(s8)
8000769c:	2402ffff 	li	v0,-1
800076a0:	14620007 	bne	v1,v0,800076c0 <_raise_r+0xfc>
800076a4:	00000000 	nop
800076a8:	8fc20038 	lw	v0,56(s8)
800076ac:	24030016 	li	v1,22
800076b0:	ac430000 	sw	v1,0(v0)
800076b4:	24020001 	li	v0,1
800076b8:	1000000c 	b	800076ec <_raise_r+0x128>
800076bc:	00000000 	nop
800076c0:	8fc20038 	lw	v0,56(s8)
800076c4:	8c4302dc 	lw	v1,732(v0)
800076c8:	8fc20040 	lw	v0,64(s8)
800076cc:	00021080 	sll	v0,v0,0x2
800076d0:	00621021 	addu	v0,v1,v0
800076d4:	ac400000 	sw	zero,0(v0)
800076d8:	8fc20020 	lw	v0,32(s8)
800076dc:	8fc40040 	lw	a0,64(s8)
800076e0:	0040f809 	jalr	v0
800076e4:	00000000 	nop
800076e8:	00001025 	move	v0,zero
800076ec:	03c0e825 	move	sp,s8
800076f0:	dfbf0030 	ld	ra,48(sp)
800076f4:	dfbe0028 	ld	s8,40(sp)
800076f8:	27bd0038 	addiu	sp,sp,56
800076fc:	03e00008 	jr	ra
80007700:	00000000 	nop

80007704 <__sigtramp_r>:
80007704:	27bdffc8 	addiu	sp,sp,-56
80007708:	ffbf0030 	sd	ra,48(sp)
8000770c:	ffbe0028 	sd	s8,40(sp)
80007710:	03a0f025 	move	s8,sp
80007714:	afc40038 	sw	a0,56(s8)
80007718:	afc50040 	sw	a1,64(s8)
8000771c:	8fc20040 	lw	v0,64(s8)
80007720:	04400005 	bltz	v0,80007738 <__sigtramp_r+0x34>
80007724:	00000000 	nop
80007728:	8fc20040 	lw	v0,64(s8)
8000772c:	28420020 	slti	v0,v0,32
80007730:	14400004 	bnez	v0,80007744 <__sigtramp_r+0x40>
80007734:	00000000 	nop
80007738:	2402ffff 	li	v0,-1
8000773c:	10000033 	b	8000780c <__sigtramp_r+0x108>
80007740:	00000000 	nop
80007744:	8fc20038 	lw	v0,56(s8)
80007748:	8c4202dc 	lw	v0,732(v0)
8000774c:	14400009 	bnez	v0,80007774 <__sigtramp_r+0x70>
80007750:	00000000 	nop
80007754:	8fc40038 	lw	a0,56(s8)
80007758:	0c001d0e 	jal	80007438 <_init_signal_r>
8000775c:	00000000 	nop
80007760:	10400004 	beqz	v0,80007774 <__sigtramp_r+0x70>
80007764:	00000000 	nop
80007768:	2402ffff 	li	v0,-1
8000776c:	10000027 	b	8000780c <__sigtramp_r+0x108>
80007770:	00000000 	nop
80007774:	8fc20038 	lw	v0,56(s8)
80007778:	8c4302dc 	lw	v1,732(v0)
8000777c:	8fc20040 	lw	v0,64(s8)
80007780:	00021080 	sll	v0,v0,0x2
80007784:	00621021 	addu	v0,v1,v0
80007788:	8c420000 	lw	v0,0(v0)
8000778c:	afc20020 	sw	v0,32(s8)
80007790:	8fc20020 	lw	v0,32(s8)
80007794:	14400004 	bnez	v0,800077a8 <__sigtramp_r+0xa4>
80007798:	00000000 	nop
8000779c:	24020001 	li	v0,1
800077a0:	1000001a 	b	8000780c <__sigtramp_r+0x108>
800077a4:	00000000 	nop
800077a8:	8fc30020 	lw	v1,32(s8)
800077ac:	2402ffff 	li	v0,-1
800077b0:	14620004 	bne	v1,v0,800077c4 <__sigtramp_r+0xc0>
800077b4:	00000000 	nop
800077b8:	24020002 	li	v0,2
800077bc:	10000013 	b	8000780c <__sigtramp_r+0x108>
800077c0:	00000000 	nop
800077c4:	8fc30020 	lw	v1,32(s8)
800077c8:	24020001 	li	v0,1
800077cc:	14620004 	bne	v1,v0,800077e0 <__sigtramp_r+0xdc>
800077d0:	00000000 	nop
800077d4:	24020003 	li	v0,3
800077d8:	1000000c 	b	8000780c <__sigtramp_r+0x108>
800077dc:	00000000 	nop
800077e0:	8fc20038 	lw	v0,56(s8)
800077e4:	8c4302dc 	lw	v1,732(v0)
800077e8:	8fc20040 	lw	v0,64(s8)
800077ec:	00021080 	sll	v0,v0,0x2
800077f0:	00621021 	addu	v0,v1,v0
800077f4:	ac400000 	sw	zero,0(v0)
800077f8:	8fc20020 	lw	v0,32(s8)
800077fc:	8fc40040 	lw	a0,64(s8)
80007800:	0040f809 	jalr	v0
80007804:	00000000 	nop
80007808:	00001025 	move	v0,zero
8000780c:	03c0e825 	move	sp,s8
80007810:	dfbf0030 	ld	ra,48(sp)
80007814:	dfbe0028 	ld	s8,40(sp)
80007818:	27bd0038 	addiu	sp,sp,56
8000781c:	03e00008 	jr	ra
80007820:	00000000 	nop

80007824 <raise>:
80007824:	27bdffd0 	addiu	sp,sp,-48
80007828:	ffbf0028 	sd	ra,40(sp)
8000782c:	ffbe0020 	sd	s8,32(sp)
80007830:	03a0f025 	move	s8,sp
80007834:	afc40030 	sw	a0,48(s8)
80007838:	8f828028 	lw	v0,-32728(gp)
8000783c:	8fc50030 	lw	a1,48(s8)
80007840:	00402025 	move	a0,v0
80007844:	0c001d71 	jal	800075c4 <_raise_r>
80007848:	00000000 	nop
8000784c:	03c0e825 	move	sp,s8
80007850:	dfbf0028 	ld	ra,40(sp)
80007854:	dfbe0020 	ld	s8,32(sp)
80007858:	27bd0030 	addiu	sp,sp,48
8000785c:	03e00008 	jr	ra
80007860:	00000000 	nop

80007864 <signal>:
80007864:	27bdffd0 	addiu	sp,sp,-48
80007868:	ffbf0028 	sd	ra,40(sp)
8000786c:	ffbe0020 	sd	s8,32(sp)
80007870:	03a0f025 	move	s8,sp
80007874:	afc40030 	sw	a0,48(s8)
80007878:	afc50038 	sw	a1,56(s8)
8000787c:	8f828028 	lw	v0,-32728(gp)
80007880:	8fc60038 	lw	a2,56(s8)
80007884:	8fc50030 	lw	a1,48(s8)
80007888:	00402025 	move	a0,v0
8000788c:	0c001d3c 	jal	800074f0 <_signal_r>
80007890:	00000000 	nop
80007894:	03c0e825 	move	sp,s8
80007898:	dfbf0028 	ld	ra,40(sp)
8000789c:	dfbe0020 	ld	s8,32(sp)
800078a0:	27bd0030 	addiu	sp,sp,48
800078a4:	03e00008 	jr	ra
800078a8:	00000000 	nop

800078ac <_init_signal>:
800078ac:	27bdffd0 	addiu	sp,sp,-48
800078b0:	ffbf0028 	sd	ra,40(sp)
800078b4:	ffbe0020 	sd	s8,32(sp)
800078b8:	03a0f025 	move	s8,sp
800078bc:	8f828028 	lw	v0,-32728(gp)
800078c0:	00402025 	move	a0,v0
800078c4:	0c001d0e 	jal	80007438 <_init_signal_r>
800078c8:	00000000 	nop
800078cc:	03c0e825 	move	sp,s8
800078d0:	dfbf0028 	ld	ra,40(sp)
800078d4:	dfbe0020 	ld	s8,32(sp)
800078d8:	27bd0030 	addiu	sp,sp,48
800078dc:	03e00008 	jr	ra
800078e0:	00000000 	nop

800078e4 <__sigtramp>:
800078e4:	27bdffd0 	addiu	sp,sp,-48
800078e8:	ffbf0028 	sd	ra,40(sp)
800078ec:	ffbe0020 	sd	s8,32(sp)
800078f0:	03a0f025 	move	s8,sp
800078f4:	afc40030 	sw	a0,48(s8)
800078f8:	8f828028 	lw	v0,-32728(gp)
800078fc:	8fc50030 	lw	a1,48(s8)
80007900:	00402025 	move	a0,v0
80007904:	0c001dc1 	jal	80007704 <__sigtramp_r>
80007908:	00000000 	nop
8000790c:	03c0e825 	move	sp,s8
80007910:	dfbf0028 	ld	ra,40(sp)
80007914:	dfbe0020 	ld	s8,32(sp)
80007918:	27bd0030 	addiu	sp,sp,48
8000791c:	03e00008 	jr	ra
80007920:	00000000 	nop

80007924 <_kill_r>:
80007924:	27bdffc8 	addiu	sp,sp,-56
80007928:	ffbf0030 	sd	ra,48(sp)
8000792c:	ffbe0028 	sd	s8,40(sp)
80007930:	03a0f025 	move	s8,sp
80007934:	afc40038 	sw	a0,56(s8)
80007938:	afc50040 	sw	a1,64(s8)
8000793c:	afc60048 	sw	a2,72(s8)
80007940:	af808334 	sw	zero,-31948(gp)
80007944:	8fc50048 	lw	a1,72(s8)
80007948:	8fc40040 	lw	a0,64(s8)
8000794c:	0c0067a8 	jal	80019ea0 <kill>
80007950:	00000000 	nop
80007954:	afc20020 	sw	v0,32(s8)
80007958:	8fc30020 	lw	v1,32(s8)
8000795c:	2402ffff 	li	v0,-1
80007960:	14620007 	bne	v1,v0,80007980 <_kill_r+0x5c>
80007964:	00000000 	nop
80007968:	8f828334 	lw	v0,-31948(gp)
8000796c:	10400004 	beqz	v0,80007980 <_kill_r+0x5c>
80007970:	00000000 	nop
80007974:	8f838334 	lw	v1,-31948(gp)
80007978:	8fc20038 	lw	v0,56(s8)
8000797c:	ac430000 	sw	v1,0(v0)
80007980:	8fc20020 	lw	v0,32(s8)
80007984:	03c0e825 	move	sp,s8
80007988:	dfbf0030 	ld	ra,48(sp)
8000798c:	dfbe0028 	ld	s8,40(sp)
80007990:	27bd0038 	addiu	sp,sp,56
80007994:	03e00008 	jr	ra
80007998:	00000000 	nop

8000799c <_getpid_r>:
8000799c:	27bdffc8 	addiu	sp,sp,-56
800079a0:	ffbf0030 	sd	ra,48(sp)
800079a4:	ffbe0028 	sd	s8,40(sp)
800079a8:	03a0f025 	move	s8,sp
800079ac:	afc40038 	sw	a0,56(s8)
800079b0:	0c006798 	jal	80019e60 <getpid>
800079b4:	00000000 	nop
800079b8:	afc20020 	sw	v0,32(s8)
800079bc:	8fc20020 	lw	v0,32(s8)
800079c0:	03c0e825 	move	sp,s8
800079c4:	dfbf0030 	ld	ra,48(sp)
800079c8:	dfbe0028 	ld	s8,40(sp)
800079cc:	27bd0038 	addiu	sp,sp,56
800079d0:	03e00008 	jr	ra
800079d4:	00000000 	nop

800079d8 <__sread>:
800079d8:	27bdffc0 	addiu	sp,sp,-64
800079dc:	ffbf0038 	sd	ra,56(sp)
800079e0:	ffbe0030 	sd	s8,48(sp)
800079e4:	ffb10028 	sd	s1,40(sp)
800079e8:	ffb00020 	sd	s0,32(sp)
800079ec:	03a0f025 	move	s8,sp
800079f0:	afc40040 	sw	a0,64(s8)
800079f4:	afc50048 	sw	a1,72(s8)
800079f8:	afc60050 	sw	a2,80(s8)
800079fc:	afc70058 	sw	a3,88(s8)
80007a00:	8fd00048 	lw	s0,72(s8)
80007a04:	8602000e 	lh	v0,14(s0)
80007a08:	00401825 	move	v1,v0
80007a0c:	8fc20058 	lw	v0,88(s8)
80007a10:	00403825 	move	a3,v0
80007a14:	8fc60050 	lw	a2,80(s8)
80007a18:	00602825 	move	a1,v1
80007a1c:	8fc40040 	lw	a0,64(s8)
80007a20:	0c0042eb 	jal	80010bac <_read_r>
80007a24:	00000000 	nop
80007a28:	00408825 	move	s1,v0
80007a2c:	06200006 	bltz	s1,80007a48 <__sread+0x70>
80007a30:	00000000 	nop
80007a34:	8e020050 	lw	v0,80(s0)
80007a38:	02221021 	addu	v0,s1,v0
80007a3c:	ae020050 	sw	v0,80(s0)
80007a40:	10000007 	b	80007a60 <__sread+0x88>
80007a44:	00000000 	nop
80007a48:	8603000c 	lh	v1,12(s0)
80007a4c:	2402efff 	li	v0,-4097
80007a50:	00621024 	and	v0,v1,v0
80007a54:	00021400 	sll	v0,v0,0x10
80007a58:	00021403 	sra	v0,v0,0x10
80007a5c:	a602000c 	sh	v0,12(s0)
80007a60:	02201025 	move	v0,s1
80007a64:	03c0e825 	move	sp,s8
80007a68:	dfbf0038 	ld	ra,56(sp)
80007a6c:	dfbe0030 	ld	s8,48(sp)
80007a70:	dfb10028 	ld	s1,40(sp)
80007a74:	dfb00020 	ld	s0,32(sp)
80007a78:	27bd0040 	addiu	sp,sp,64
80007a7c:	03e00008 	jr	ra
80007a80:	00000000 	nop

80007a84 <__seofread>:
80007a84:	27bdfff8 	addiu	sp,sp,-8
80007a88:	ffbe0000 	sd	s8,0(sp)
80007a8c:	03a0f025 	move	s8,sp
80007a90:	afc40008 	sw	a0,8(s8)
80007a94:	afc50010 	sw	a1,16(s8)
80007a98:	afc60018 	sw	a2,24(s8)
80007a9c:	afc70020 	sw	a3,32(s8)
80007aa0:	00001025 	move	v0,zero
80007aa4:	03c0e825 	move	sp,s8
80007aa8:	dfbe0000 	ld	s8,0(sp)
80007aac:	27bd0008 	addiu	sp,sp,8
80007ab0:	03e00008 	jr	ra
80007ab4:	00000000 	nop

80007ab8 <__swrite>:
80007ab8:	27bdffc0 	addiu	sp,sp,-64
80007abc:	ffbf0038 	sd	ra,56(sp)
80007ac0:	ffbe0030 	sd	s8,48(sp)
80007ac4:	ffb00028 	sd	s0,40(sp)
80007ac8:	03a0f025 	move	s8,sp
80007acc:	afc40040 	sw	a0,64(s8)
80007ad0:	afc50048 	sw	a1,72(s8)
80007ad4:	afc60050 	sw	a2,80(s8)
80007ad8:	afc70058 	sw	a3,88(s8)
80007adc:	8fd00048 	lw	s0,72(s8)
80007ae0:	8602000c 	lh	v0,12(s0)
80007ae4:	3042ffff 	andi	v0,v0,0xffff
80007ae8:	30420100 	andi	v0,v0,0x100
80007aec:	10400008 	beqz	v0,80007b10 <__swrite+0x58>
80007af0:	00000000 	nop
80007af4:	8602000e 	lh	v0,14(s0)
80007af8:	24070002 	li	a3,2
80007afc:	00003025 	move	a2,zero
80007b00:	00402825 	move	a1,v0
80007b04:	8fc40040 	lw	a0,64(s8)
80007b08:	0c003a61 	jal	8000e984 <_lseek_r>
80007b0c:	00000000 	nop
80007b10:	8603000c 	lh	v1,12(s0)
80007b14:	2402efff 	li	v0,-4097
80007b18:	00621024 	and	v0,v1,v0
80007b1c:	00021400 	sll	v0,v0,0x10
80007b20:	00021403 	sra	v0,v0,0x10
80007b24:	a602000c 	sh	v0,12(s0)
80007b28:	8602000e 	lh	v0,14(s0)
80007b2c:	00401825 	move	v1,v0
80007b30:	8fc20058 	lw	v0,88(s8)
80007b34:	00403825 	move	a3,v0
80007b38:	8fc60050 	lw	a2,80(s8)
80007b3c:	00602825 	move	a1,v1
80007b40:	8fc40040 	lw	a0,64(s8)
80007b44:	0c002e32 	jal	8000b8c8 <_write_r>
80007b48:	00000000 	nop
80007b4c:	afc20020 	sw	v0,32(s8)
80007b50:	8fc20020 	lw	v0,32(s8)
80007b54:	03c0e825 	move	sp,s8
80007b58:	dfbf0038 	ld	ra,56(sp)
80007b5c:	dfbe0030 	ld	s8,48(sp)
80007b60:	dfb00028 	ld	s0,40(sp)
80007b64:	27bd0040 	addiu	sp,sp,64
80007b68:	03e00008 	jr	ra
80007b6c:	00000000 	nop

80007b70 <__sseek>:
80007b70:	27bdffc0 	addiu	sp,sp,-64
80007b74:	ffbf0038 	sd	ra,56(sp)
80007b78:	ffbe0030 	sd	s8,48(sp)
80007b7c:	ffb10028 	sd	s1,40(sp)
80007b80:	ffb00020 	sd	s0,32(sp)
80007b84:	03a0f025 	move	s8,sp
80007b88:	afc40040 	sw	a0,64(s8)
80007b8c:	afc50048 	sw	a1,72(s8)
80007b90:	afc60050 	sw	a2,80(s8)
80007b94:	afc70058 	sw	a3,88(s8)
80007b98:	8fd00048 	lw	s0,72(s8)
80007b9c:	8602000e 	lh	v0,14(s0)
80007ba0:	8fc70058 	lw	a3,88(s8)
80007ba4:	8fc60050 	lw	a2,80(s8)
80007ba8:	00402825 	move	a1,v0
80007bac:	8fc40040 	lw	a0,64(s8)
80007bb0:	0c003a61 	jal	8000e984 <_lseek_r>
80007bb4:	00000000 	nop
80007bb8:	00408825 	move	s1,v0
80007bbc:	2402ffff 	li	v0,-1
80007bc0:	16220009 	bne	s1,v0,80007be8 <__sseek+0x78>
80007bc4:	00000000 	nop
80007bc8:	8603000c 	lh	v1,12(s0)
80007bcc:	2402efff 	li	v0,-4097
80007bd0:	00621024 	and	v0,v1,v0
80007bd4:	00021400 	sll	v0,v0,0x10
80007bd8:	00021403 	sra	v0,v0,0x10
80007bdc:	a602000c 	sh	v0,12(s0)
80007be0:	10000007 	b	80007c00 <__sseek+0x90>
80007be4:	00000000 	nop
80007be8:	8602000c 	lh	v0,12(s0)
80007bec:	34421000 	ori	v0,v0,0x1000
80007bf0:	00021400 	sll	v0,v0,0x10
80007bf4:	00021403 	sra	v0,v0,0x10
80007bf8:	a602000c 	sh	v0,12(s0)
80007bfc:	ae110050 	sw	s1,80(s0)
80007c00:	02201025 	move	v0,s1
80007c04:	03c0e825 	move	sp,s8
80007c08:	dfbf0038 	ld	ra,56(sp)
80007c0c:	dfbe0030 	ld	s8,48(sp)
80007c10:	dfb10028 	ld	s1,40(sp)
80007c14:	dfb00020 	ld	s0,32(sp)
80007c18:	27bd0040 	addiu	sp,sp,64
80007c1c:	03e00008 	jr	ra
80007c20:	00000000 	nop

80007c24 <__sclose>:
80007c24:	27bdffc8 	addiu	sp,sp,-56
80007c28:	ffbf0030 	sd	ra,48(sp)
80007c2c:	ffbe0028 	sd	s8,40(sp)
80007c30:	03a0f025 	move	s8,sp
80007c34:	afc40038 	sw	a0,56(s8)
80007c38:	afc50040 	sw	a1,64(s8)
80007c3c:	8fc20040 	lw	v0,64(s8)
80007c40:	afc20020 	sw	v0,32(s8)
80007c44:	8fc20020 	lw	v0,32(s8)
80007c48:	8442000e 	lh	v0,14(v0)
80007c4c:	00402825 	move	a1,v0
80007c50:	8fc40038 	lw	a0,56(s8)
80007c54:	0c002ed8 	jal	8000bb60 <_close_r>
80007c58:	00000000 	nop
80007c5c:	03c0e825 	move	sp,s8
80007c60:	dfbf0030 	ld	ra,48(sp)
80007c64:	dfbe0028 	ld	s8,40(sp)
80007c68:	27bd0038 	addiu	sp,sp,56
80007c6c:	03e00008 	jr	ra
80007c70:	00000000 	nop

80007c74 <strlen>:
80007c74:	64820001 	daddiu	v0,a0,1
80007c78:	90830000 	lbu	v1,0(a0)
80007c7c:	1460fffe 	bnez	v1,80007c78 <strlen+0x4>
80007c80:	64840001 	daddiu	a0,a0,1
80007c84:	03e00008 	jr	ra
80007c88:	0082102f 	dsubu	v0,a0,v0

80007c8c <_times_r>:
80007c8c:	27bdffc8 	addiu	sp,sp,-56
80007c90:	ffbf0030 	sd	ra,48(sp)
80007c94:	ffbe0028 	sd	s8,40(sp)
80007c98:	03a0f025 	move	s8,sp
80007c9c:	afc40038 	sw	a0,56(s8)
80007ca0:	afc50040 	sw	a1,64(s8)
80007ca4:	8fc40040 	lw	a0,64(s8)
80007ca8:	0c006860 	jal	8001a180 <times>
80007cac:	00000000 	nop
80007cb0:	afc20020 	sw	v0,32(s8)
80007cb4:	8fc20020 	lw	v0,32(s8)
80007cb8:	03c0e825 	move	sp,s8
80007cbc:	dfbf0030 	ld	ra,48(sp)
80007cc0:	dfbe0028 	ld	s8,40(sp)
80007cc4:	27bd0038 	addiu	sp,sp,56
80007cc8:	03e00008 	jr	ra
80007ccc:	00000000 	nop

80007cd0 <__tzcalc_limits>:
80007cd0:	27bdffa0 	addiu	sp,sp,-96
80007cd4:	ffbf0058 	sd	ra,88(sp)
80007cd8:	ffbe0050 	sd	s8,80(sp)
80007cdc:	03a0f025 	move	s8,sp
80007ce0:	afc40060 	sw	a0,96(s8)
80007ce4:	0c003983 	jal	8000e60c <__gettzinfo>
80007ce8:	00000000 	nop
80007cec:	afc20034 	sw	v0,52(s8)
80007cf0:	8fc20060 	lw	v0,96(s8)
80007cf4:	284207b2 	slti	v0,v0,1970
80007cf8:	10400004 	beqz	v0,80007d0c <__tzcalc_limits+0x3c>
80007cfc:	00000000 	nop
80007d00:	00001025 	move	v0,zero
80007d04:	10000165 	b	8000829c <__tzcalc_limits+0x5cc>
80007d08:	00000000 	nop
80007d0c:	8fc20034 	lw	v0,52(s8)
80007d10:	8fc30060 	lw	v1,96(s8)
80007d14:	ac430004 	sw	v1,4(v0)
80007d18:	8fc20060 	lw	v0,96(s8)
80007d1c:	2442f84e 	addiu	v0,v0,-1970
80007d20:	afc20038 	sw	v0,56(s8)
80007d24:	8fc30038 	lw	v1,56(s8)
80007d28:	00601025 	move	v0,v1
80007d2c:	000210c0 	sll	v0,v0,0x3
80007d30:	00431021 	addu	v0,v0,v1
80007d34:	000210c0 	sll	v0,v0,0x3
80007d38:	00431021 	addu	v0,v0,v1
80007d3c:	00021880 	sll	v1,v0,0x2
80007d40:	00431821 	addu	v1,v0,v1
80007d44:	8fc20038 	lw	v0,56(s8)
80007d48:	24420001 	addiu	v0,v0,1
80007d4c:	000227c3 	sra	a0,v0,0x1f
80007d50:	00042782 	srl	a0,a0,0x1e
80007d54:	00821021 	addu	v0,a0,v0
80007d58:	00021083 	sra	v0,v0,0x2
80007d5c:	00621821 	addu	v1,v1,v0
80007d60:	8fc20038 	lw	v0,56(s8)
80007d64:	24420045 	addiu	v0,v0,69
80007d68:	3c0451eb 	lui	a0,0x51eb
80007d6c:	3484851f 	ori	a0,a0,0x851f
80007d70:	00440018 	mult	v0,a0
80007d74:	00002010 	mfhi	a0
80007d78:	00042143 	sra	a0,a0,0x5
80007d7c:	000217c3 	sra	v0,v0,0x1f
80007d80:	00441023 	subu	v0,v0,a0
80007d84:	00621821 	addu	v1,v1,v0
80007d88:	8fc20038 	lw	v0,56(s8)
80007d8c:	24420171 	addiu	v0,v0,369
80007d90:	3c0451eb 	lui	a0,0x51eb
80007d94:	3484851f 	ori	a0,a0,0x851f
80007d98:	00440018 	mult	v0,a0
80007d9c:	00002010 	mfhi	a0
80007da0:	000421c3 	sra	a0,a0,0x7
80007da4:	000217c3 	sra	v0,v0,0x1f
80007da8:	00821023 	subu	v0,a0,v0
80007dac:	00621021 	addu	v0,v1,v0
80007db0:	afc2003c 	sw	v0,60(s8)
80007db4:	afc00024 	sw	zero,36(s8)
80007db8:	1000012a 	b	80008264 <__tzcalc_limits+0x594>
80007dbc:	00000000 	nop
80007dc0:	8fc40034 	lw	a0,52(s8)
80007dc4:	8fc30024 	lw	v1,36(s8)
80007dc8:	00601025 	move	v0,v1
80007dcc:	00021080 	sll	v0,v0,0x2
80007dd0:	00431021 	addu	v0,v0,v1
80007dd4:	000210c0 	sll	v0,v0,0x3
80007dd8:	00821021 	addu	v0,a0,v0
80007ddc:	80430008 	lb	v1,8(v0)
80007de0:	2402004a 	li	v0,74
80007de4:	14620047 	bne	v1,v0,80007f04 <__tzcalc_limits+0x234>
80007de8:	00000000 	nop
80007dec:	8fc40034 	lw	a0,52(s8)
80007df0:	8fc30024 	lw	v1,36(s8)
80007df4:	00601025 	move	v0,v1
80007df8:	00021080 	sll	v0,v0,0x2
80007dfc:	00431021 	addu	v0,v0,v1
80007e00:	000210c0 	sll	v0,v0,0x3
80007e04:	00821021 	addu	v0,a0,v0
80007e08:	8c430014 	lw	v1,20(v0)
80007e0c:	8fc2003c 	lw	v0,60(s8)
80007e10:	00622821 	addu	a1,v1,v0
80007e14:	8fc20060 	lw	v0,96(s8)
80007e18:	30420003 	andi	v0,v0,0x3
80007e1c:	14400012 	bnez	v0,80007e68 <__tzcalc_limits+0x198>
80007e20:	00000000 	nop
80007e24:	8fc40060 	lw	a0,96(s8)
80007e28:	3c0251eb 	lui	v0,0x51eb
80007e2c:	3442851f 	ori	v0,v0,0x851f
80007e30:	00820018 	mult	a0,v0
80007e34:	00001010 	mfhi	v0
80007e38:	00021943 	sra	v1,v0,0x5
80007e3c:	000417c3 	sra	v0,a0,0x1f
80007e40:	00621823 	subu	v1,v1,v0
80007e44:	00601025 	move	v0,v1
80007e48:	00021040 	sll	v0,v0,0x1
80007e4c:	00431021 	addu	v0,v0,v1
80007e50:	000210c0 	sll	v0,v0,0x3
80007e54:	00431021 	addu	v0,v0,v1
80007e58:	00021080 	sll	v0,v0,0x2
80007e5c:	00821823 	subu	v1,a0,v0
80007e60:	14600012 	bnez	v1,80007eac <__tzcalc_limits+0x1dc>
80007e64:	00000000 	nop
80007e68:	8fc40060 	lw	a0,96(s8)
80007e6c:	3c0251eb 	lui	v0,0x51eb
80007e70:	3442851f 	ori	v0,v0,0x851f
80007e74:	00820018 	mult	a0,v0
80007e78:	00001010 	mfhi	v0
80007e7c:	000219c3 	sra	v1,v0,0x7
80007e80:	000417c3 	sra	v0,a0,0x1f
80007e84:	00621823 	subu	v1,v1,v0
80007e88:	00601025 	move	v0,v1
80007e8c:	00021040 	sll	v0,v0,0x1
80007e90:	00431021 	addu	v0,v0,v1
80007e94:	000210c0 	sll	v0,v0,0x3
80007e98:	00431021 	addu	v0,v0,v1
80007e9c:	00021100 	sll	v0,v0,0x4
80007ea0:	00821823 	subu	v1,a0,v0
80007ea4:	1460000f 	bnez	v1,80007ee4 <__tzcalc_limits+0x214>
80007ea8:	00000000 	nop
80007eac:	8fc40034 	lw	a0,52(s8)
80007eb0:	8fc30024 	lw	v1,36(s8)
80007eb4:	00601025 	move	v0,v1
80007eb8:	00021080 	sll	v0,v0,0x2
80007ebc:	00431021 	addu	v0,v0,v1
80007ec0:	000210c0 	sll	v0,v0,0x3
80007ec4:	00821021 	addu	v0,a0,v0
80007ec8:	8c420014 	lw	v0,20(v0)
80007ecc:	2842003c 	slti	v0,v0,60
80007ed0:	14400004 	bnez	v0,80007ee4 <__tzcalc_limits+0x214>
80007ed4:	00000000 	nop
80007ed8:	24020001 	li	v0,1
80007edc:	10000002 	b	80007ee8 <__tzcalc_limits+0x218>
80007ee0:	00000000 	nop
80007ee4:	00001025 	move	v0,zero
80007ee8:	00451021 	addu	v0,v0,a1
80007eec:	afc20020 	sw	v0,32(s8)
80007ef0:	8fc20020 	lw	v0,32(s8)
80007ef4:	2442ffff 	addiu	v0,v0,-1
80007ef8:	afc20020 	sw	v0,32(s8)
80007efc:	100000b1 	b	800081c4 <__tzcalc_limits+0x4f4>
80007f00:	00000000 	nop
80007f04:	8fc40034 	lw	a0,52(s8)
80007f08:	8fc30024 	lw	v1,36(s8)
80007f0c:	00601025 	move	v0,v1
80007f10:	00021080 	sll	v0,v0,0x2
80007f14:	00431021 	addu	v0,v0,v1
80007f18:	000210c0 	sll	v0,v0,0x3
80007f1c:	00821021 	addu	v0,a0,v0
80007f20:	80430008 	lb	v1,8(v0)
80007f24:	24020044 	li	v0,68
80007f28:	1462000e 	bne	v1,v0,80007f64 <__tzcalc_limits+0x294>
80007f2c:	00000000 	nop
80007f30:	8fc40034 	lw	a0,52(s8)
80007f34:	8fc30024 	lw	v1,36(s8)
80007f38:	00601025 	move	v0,v1
80007f3c:	00021080 	sll	v0,v0,0x2
80007f40:	00431021 	addu	v0,v0,v1
80007f44:	000210c0 	sll	v0,v0,0x3
80007f48:	00821021 	addu	v0,a0,v0
80007f4c:	8c420014 	lw	v0,20(v0)
80007f50:	8fc3003c 	lw	v1,60(s8)
80007f54:	00621021 	addu	v0,v1,v0
80007f58:	afc20020 	sw	v0,32(s8)
80007f5c:	10000099 	b	800081c4 <__tzcalc_limits+0x4f4>
80007f60:	00000000 	nop
80007f64:	8fc20060 	lw	v0,96(s8)
80007f68:	30420003 	andi	v0,v0,0x3
80007f6c:	14400012 	bnez	v0,80007fb8 <__tzcalc_limits+0x2e8>
80007f70:	00000000 	nop
80007f74:	8fc40060 	lw	a0,96(s8)
80007f78:	3c0251eb 	lui	v0,0x51eb
80007f7c:	3442851f 	ori	v0,v0,0x851f
80007f80:	00820018 	mult	a0,v0
80007f84:	00001010 	mfhi	v0
80007f88:	00021943 	sra	v1,v0,0x5
80007f8c:	000417c3 	sra	v0,a0,0x1f
80007f90:	00621823 	subu	v1,v1,v0
80007f94:	00601025 	move	v0,v1
80007f98:	00021040 	sll	v0,v0,0x1
80007f9c:	00431021 	addu	v0,v0,v1
80007fa0:	000210c0 	sll	v0,v0,0x3
80007fa4:	00431021 	addu	v0,v0,v1
80007fa8:	00021080 	sll	v0,v0,0x2
80007fac:	00821823 	subu	v1,a0,v0
80007fb0:	14600012 	bnez	v1,80007ffc <__tzcalc_limits+0x32c>
80007fb4:	00000000 	nop
80007fb8:	8fc40060 	lw	a0,96(s8)
80007fbc:	3c0251eb 	lui	v0,0x51eb
80007fc0:	3442851f 	ori	v0,v0,0x851f
80007fc4:	00820018 	mult	a0,v0
80007fc8:	00001010 	mfhi	v0
80007fcc:	000219c3 	sra	v1,v0,0x7
80007fd0:	000417c3 	sra	v0,a0,0x1f
80007fd4:	00621823 	subu	v1,v1,v0
80007fd8:	00601025 	move	v0,v1
80007fdc:	00021040 	sll	v0,v0,0x1
80007fe0:	00431021 	addu	v0,v0,v1
80007fe4:	000210c0 	sll	v0,v0,0x3
80007fe8:	00431021 	addu	v0,v0,v1
80007fec:	00021100 	sll	v0,v0,0x4
80007ff0:	00821823 	subu	v1,a0,v0
80007ff4:	14600004 	bnez	v1,80008008 <__tzcalc_limits+0x338>
80007ff8:	00000000 	nop
80007ffc:	24020001 	li	v0,1
80008000:	10000002 	b	8000800c <__tzcalc_limits+0x33c>
80008004:	00000000 	nop
80008008:	00001025 	move	v0,zero
8000800c:	afc20040 	sw	v0,64(s8)
80008010:	8fc30040 	lw	v1,64(s8)
80008014:	00601025 	move	v0,v1
80008018:	00021040 	sll	v0,v0,0x1
8000801c:	00431021 	addu	v0,v0,v1
80008020:	00021100 	sll	v0,v0,0x4
80008024:	3c038002 	lui	v1,0x8002
80008028:	2463a788 	addiu	v1,v1,-22648
8000802c:	00431021 	addu	v0,v0,v1
80008030:	afc20044 	sw	v0,68(s8)
80008034:	8fc2003c 	lw	v0,60(s8)
80008038:	afc20020 	sw	v0,32(s8)
8000803c:	24020001 	li	v0,1
80008040:	afc20028 	sw	v0,40(s8)
80008044:	1000000f 	b	80008084 <__tzcalc_limits+0x3b4>
80008048:	00000000 	nop
8000804c:	8fc30028 	lw	v1,40(s8)
80008050:	3c023fff 	lui	v0,0x3fff
80008054:	3442ffff 	ori	v0,v0,0xffff
80008058:	00621021 	addu	v0,v1,v0
8000805c:	00021080 	sll	v0,v0,0x2
80008060:	8fc30044 	lw	v1,68(s8)
80008064:	00621021 	addu	v0,v1,v0
80008068:	8c420000 	lw	v0,0(v0)
8000806c:	8fc30020 	lw	v1,32(s8)
80008070:	00621021 	addu	v0,v1,v0
80008074:	afc20020 	sw	v0,32(s8)
80008078:	8fc20028 	lw	v0,40(s8)
8000807c:	24420001 	addiu	v0,v0,1
80008080:	afc20028 	sw	v0,40(s8)
80008084:	8fc40034 	lw	a0,52(s8)
80008088:	8fc30024 	lw	v1,36(s8)
8000808c:	00601025 	move	v0,v1
80008090:	00021080 	sll	v0,v0,0x2
80008094:	00431021 	addu	v0,v0,v1
80008098:	000210c0 	sll	v0,v0,0x3
8000809c:	00821021 	addu	v0,a0,v0
800080a0:	8c42000c 	lw	v0,12(v0)
800080a4:	8fc30028 	lw	v1,40(s8)
800080a8:	0062102a 	slt	v0,v1,v0
800080ac:	1440ffe7 	bnez	v0,8000804c <__tzcalc_limits+0x37c>
800080b0:	00000000 	nop
800080b4:	8fc20020 	lw	v0,32(s8)
800080b8:	24420004 	addiu	v0,v0,4
800080bc:	3c039249 	lui	v1,0x9249
800080c0:	34632493 	ori	v1,v1,0x2493
800080c4:	00430018 	mult	v0,v1
800080c8:	00001810 	mfhi	v1
800080cc:	00621821 	addu	v1,v1,v0
800080d0:	00032083 	sra	a0,v1,0x2
800080d4:	00021fc3 	sra	v1,v0,0x1f
800080d8:	00832023 	subu	a0,a0,v1
800080dc:	00801825 	move	v1,a0
800080e0:	000318c0 	sll	v1,v1,0x3
800080e4:	00641823 	subu	v1,v1,a0
800080e8:	00431023 	subu	v0,v0,v1
800080ec:	afc20048 	sw	v0,72(s8)
800080f0:	8fc40034 	lw	a0,52(s8)
800080f4:	8fc30024 	lw	v1,36(s8)
800080f8:	00601025 	move	v0,v1
800080fc:	00021080 	sll	v0,v0,0x2
80008100:	00431021 	addu	v0,v0,v1
80008104:	000210c0 	sll	v0,v0,0x3
80008108:	00821021 	addu	v0,a0,v0
8000810c:	8c430014 	lw	v1,20(v0)
80008110:	8fc20048 	lw	v0,72(s8)
80008114:	00621023 	subu	v0,v1,v0
80008118:	afc20030 	sw	v0,48(s8)
8000811c:	8fc20030 	lw	v0,48(s8)
80008120:	04410004 	bgez	v0,80008134 <__tzcalc_limits+0x464>
80008124:	00000000 	nop
80008128:	8fc20030 	lw	v0,48(s8)
8000812c:	24420007 	addiu	v0,v0,7
80008130:	afc20030 	sw	v0,48(s8)
80008134:	8fc40034 	lw	a0,52(s8)
80008138:	8fc30024 	lw	v1,36(s8)
8000813c:	00601025 	move	v0,v1
80008140:	00021080 	sll	v0,v0,0x2
80008144:	00431021 	addu	v0,v0,v1
80008148:	000210c0 	sll	v0,v0,0x3
8000814c:	00821021 	addu	v0,a0,v0
80008150:	8c420010 	lw	v0,16(v0)
80008154:	2443ffff 	addiu	v1,v0,-1
80008158:	00601025 	move	v0,v1
8000815c:	000210c0 	sll	v0,v0,0x3
80008160:	00431023 	subu	v0,v0,v1
80008164:	8fc30030 	lw	v1,48(s8)
80008168:	00621021 	addu	v0,v1,v0
8000816c:	afc2002c 	sw	v0,44(s8)
80008170:	10000004 	b	80008184 <__tzcalc_limits+0x4b4>
80008174:	00000000 	nop
80008178:	8fc2002c 	lw	v0,44(s8)
8000817c:	2442fff9 	addiu	v0,v0,-7
80008180:	afc2002c 	sw	v0,44(s8)
80008184:	8fc30028 	lw	v1,40(s8)
80008188:	3c023fff 	lui	v0,0x3fff
8000818c:	3442ffff 	ori	v0,v0,0xffff
80008190:	00621021 	addu	v0,v1,v0
80008194:	00021080 	sll	v0,v0,0x2
80008198:	8fc30044 	lw	v1,68(s8)
8000819c:	00621021 	addu	v0,v1,v0
800081a0:	8c420000 	lw	v0,0(v0)
800081a4:	8fc3002c 	lw	v1,44(s8)
800081a8:	0062102a 	slt	v0,v1,v0
800081ac:	1040fff2 	beqz	v0,80008178 <__tzcalc_limits+0x4a8>
800081b0:	00000000 	nop
800081b4:	8fc30020 	lw	v1,32(s8)
800081b8:	8fc2002c 	lw	v0,44(s8)
800081bc:	00621021 	addu	v0,v1,v0
800081c0:	afc20020 	sw	v0,32(s8)
800081c4:	8fc30020 	lw	v1,32(s8)
800081c8:	00601025 	move	v0,v1
800081cc:	00021078 	dsll	v0,v0,0x1
800081d0:	0043102d 	daddu	v0,v0,v1
800081d4:	00021938 	dsll	v1,v0,0x4
800081d8:	0062182f 	dsubu	v1,v1,v0
800081dc:	00031138 	dsll	v0,v1,0x4
800081e0:	0043102f 	dsubu	v0,v0,v1
800081e4:	000211f8 	dsll	v0,v0,0x7
800081e8:	00402825 	move	a1,v0
800081ec:	8fc40034 	lw	a0,52(s8)
800081f0:	8fc30024 	lw	v1,36(s8)
800081f4:	00601025 	move	v0,v1
800081f8:	00021080 	sll	v0,v0,0x2
800081fc:	00431021 	addu	v0,v0,v1
80008200:	000210c0 	sll	v0,v0,0x3
80008204:	00821021 	addu	v0,a0,v0
80008208:	8c420018 	lw	v0,24(v0)
8000820c:	00a2202d 	daddu	a0,a1,v0
80008210:	8fc50034 	lw	a1,52(s8)
80008214:	8fc20024 	lw	v0,36(s8)
80008218:	24430001 	addiu	v1,v0,1
8000821c:	00601025 	move	v0,v1
80008220:	00021080 	sll	v0,v0,0x2
80008224:	00431021 	addu	v0,v0,v1
80008228:	000210c0 	sll	v0,v0,0x3
8000822c:	00a21021 	addu	v0,a1,v0
80008230:	8c420000 	lw	v0,0(v0)
80008234:	0082202d 	daddu	a0,a0,v0
80008238:	8fc50034 	lw	a1,52(s8)
8000823c:	8fc30024 	lw	v1,36(s8)
80008240:	00601025 	move	v0,v1
80008244:	00021080 	sll	v0,v0,0x2
80008248:	00431021 	addu	v0,v0,v1
8000824c:	000210c0 	sll	v0,v0,0x3
80008250:	00a21021 	addu	v0,a1,v0
80008254:	fc440020 	sd	a0,32(v0)
80008258:	8fc20024 	lw	v0,36(s8)
8000825c:	24420001 	addiu	v0,v0,1
80008260:	afc20024 	sw	v0,36(s8)
80008264:	8fc20024 	lw	v0,36(s8)
80008268:	28420002 	slti	v0,v0,2
8000826c:	1440fed4 	bnez	v0,80007dc0 <__tzcalc_limits+0xf0>
80008270:	00000000 	nop
80008274:	8fc20034 	lw	v0,52(s8)
80008278:	dc430020 	ld	v1,32(v0)
8000827c:	8fc20034 	lw	v0,52(s8)
80008280:	dc420048 	ld	v0,72(v0)
80008284:	0062102a 	slt	v0,v1,v0
80008288:	304200ff 	andi	v0,v0,0xff
8000828c:	00401825 	move	v1,v0
80008290:	8fc20034 	lw	v0,52(s8)
80008294:	ac430000 	sw	v1,0(v0)
80008298:	24020001 	li	v0,1
8000829c:	03c0e825 	move	sp,s8
800082a0:	dfbf0058 	ld	ra,88(sp)
800082a4:	dfbe0050 	ld	s8,80(sp)
800082a8:	27bd0060 	addiu	sp,sp,96
800082ac:	03e00008 	jr	ra
800082b0:	00000000 	nop

800082b4 <__tz_lock>:
800082b4:	27bdfff8 	addiu	sp,sp,-8
800082b8:	ffbe0000 	sd	s8,0(sp)
800082bc:	03a0f025 	move	s8,sp
800082c0:	00000000 	nop
800082c4:	03c0e825 	move	sp,s8
800082c8:	dfbe0000 	ld	s8,0(sp)
800082cc:	27bd0008 	addiu	sp,sp,8
800082d0:	03e00008 	jr	ra
800082d4:	00000000 	nop

800082d8 <__tz_unlock>:
800082d8:	27bdfff8 	addiu	sp,sp,-8
800082dc:	ffbe0000 	sd	s8,0(sp)
800082e0:	03a0f025 	move	s8,sp
800082e4:	00000000 	nop
800082e8:	03c0e825 	move	sp,s8
800082ec:	dfbe0000 	ld	s8,0(sp)
800082f0:	27bd0008 	addiu	sp,sp,8
800082f4:	03e00008 	jr	ra
800082f8:	00000000 	nop

800082fc <_tzset_unlocked>:
800082fc:	27bdffd0 	addiu	sp,sp,-48
80008300:	ffbf0028 	sd	ra,40(sp)
80008304:	ffbe0020 	sd	s8,32(sp)
80008308:	03a0f025 	move	s8,sp
8000830c:	8f828028 	lw	v0,-32728(gp)
80008310:	00402025 	move	a0,v0
80008314:	0c0020e1 	jal	80008384 <_tzset_unlocked_r>
80008318:	00000000 	nop
8000831c:	00000000 	nop
80008320:	03c0e825 	move	sp,s8
80008324:	dfbf0028 	ld	ra,40(sp)
80008328:	dfbe0020 	ld	s8,32(sp)
8000832c:	27bd0030 	addiu	sp,sp,48
80008330:	03e00008 	jr	ra
80008334:	00000000 	nop

80008338 <tzset>:
80008338:	27bdffd0 	addiu	sp,sp,-48
8000833c:	ffbf0028 	sd	ra,40(sp)
80008340:	ffbe0020 	sd	s8,32(sp)
80008344:	03a0f025 	move	s8,sp
80008348:	0c0020ad 	jal	800082b4 <__tz_lock>
8000834c:	00000000 	nop
80008350:	8f828028 	lw	v0,-32728(gp)
80008354:	00402025 	move	a0,v0
80008358:	0c0020e1 	jal	80008384 <_tzset_unlocked_r>
8000835c:	00000000 	nop
80008360:	0c0020b6 	jal	800082d8 <__tz_unlock>
80008364:	00000000 	nop
80008368:	00000000 	nop
8000836c:	03c0e825 	move	sp,s8
80008370:	dfbf0028 	ld	ra,40(sp)
80008374:	dfbe0020 	ld	s8,32(sp)
80008378:	27bd0030 	addiu	sp,sp,48
8000837c:	03e00008 	jr	ra
80008380:	00000000 	nop

80008384 <_tzset_unlocked_r>:
80008384:	27bdff88 	addiu	sp,sp,-120
80008388:	ffbf0070 	sd	ra,112(sp)
8000838c:	ffbe0068 	sd	s8,104(sp)
80008390:	03a0f025 	move	s8,sp
80008394:	afc40078 	sw	a0,120(s8)
80008398:	0c003983 	jal	8000e60c <__gettzinfo>
8000839c:	00000000 	nop
800083a0:	afc20050 	sw	v0,80(s8)
800083a4:	3c028002 	lui	v0,0x8002
800083a8:	2445a480 	addiu	a1,v0,-23424
800083ac:	8fc40078 	lw	a0,120(s8)
800083b0:	0c003971 	jal	8000e5c4 <_getenv_r>
800083b4:	00000000 	nop
800083b8:	afc20040 	sw	v0,64(s8)
800083bc:	8fc20040 	lw	v0,64(s8)
800083c0:	14400010 	bnez	v0,80008404 <_tzset_unlocked_r+0x80>
800083c4:	00000000 	nop
800083c8:	af808340 	sw	zero,-31936(gp)
800083cc:	af80833c 	sw	zero,-31940(gp)
800083d0:	3c028002 	lui	v0,0x8002
800083d4:	2442a488 	addiu	v0,v0,-23416
800083d8:	af828038 	sw	v0,-32712(gp)
800083dc:	3c028002 	lui	v0,0x8002
800083e0:	2442a488 	addiu	v0,v0,-23416
800083e4:	af82803c 	sw	v0,-32708(gp)
800083e8:	8f828338 	lw	v0,-31944(gp)
800083ec:	00402025 	move	a0,v0
800083f0:	0c001179 	jal	800045e4 <free>
800083f4:	00000000 	nop
800083f8:	af808338 	sw	zero,-31944(gp)
800083fc:	100001fd 	b	80008bf4 <_tzset_unlocked_r+0x870>
80008400:	00000000 	nop
80008404:	8f828338 	lw	v0,-31944(gp)
80008408:	10400008 	beqz	v0,8000842c <_tzset_unlocked_r+0xa8>
8000840c:	00000000 	nop
80008410:	8f828338 	lw	v0,-31944(gp)
80008414:	00402825 	move	a1,v0
80008418:	8fc40040 	lw	a0,64(s8)
8000841c:	0c0046ae 	jal	80011ab8 <strcmp>
80008420:	00000000 	nop
80008424:	104001e9 	beqz	v0,80008bcc <_tzset_unlocked_r+0x848>
80008428:	00000000 	nop
8000842c:	8f828338 	lw	v0,-31944(gp)
80008430:	00402025 	move	a0,v0
80008434:	0c001179 	jal	800045e4 <free>
80008438:	00000000 	nop
8000843c:	8fc40040 	lw	a0,64(s8)
80008440:	0c001f1d 	jal	80007c74 <strlen>
80008444:	00000000 	nop
80008448:	24420001 	addiu	v0,v0,1
8000844c:	00402825 	move	a1,v0
80008450:	8fc40078 	lw	a0,120(s8)
80008454:	0c001296 	jal	80004a58 <_malloc_r>
80008458:	00000000 	nop
8000845c:	af828338 	sw	v0,-31944(gp)
80008460:	8f828338 	lw	v0,-31944(gp)
80008464:	10400006 	beqz	v0,80008480 <_tzset_unlocked_r+0xfc>
80008468:	00000000 	nop
8000846c:	8f828338 	lw	v0,-31944(gp)
80008470:	8fc50040 	lw	a1,64(s8)
80008474:	00402025 	move	a0,v0
80008478:	0c00473e 	jal	80011cf8 <strcpy>
8000847c:	00000000 	nop
80008480:	8fc20040 	lw	v0,64(s8)
80008484:	80430000 	lb	v1,0(v0)
80008488:	2402003a 	li	v0,58
8000848c:	14620004 	bne	v1,v0,800084a0 <_tzset_unlocked_r+0x11c>
80008490:	00000000 	nop
80008494:	8fc20040 	lw	v0,64(s8)
80008498:	24420001 	addiu	v0,v0,1
8000849c:	afc20040 	sw	v0,64(s8)
800084a0:	27c20060 	addiu	v0,s8,96
800084a4:	00403825 	move	a3,v0
800084a8:	3c028003 	lui	v0,0x8003
800084ac:	2446dd28 	addiu	a2,v0,-8920
800084b0:	3c028002 	lui	v0,0x8002
800084b4:	2445a490 	addiu	a1,v0,-23408
800084b8:	8fc40040 	lw	a0,64(s8)
800084bc:	0c004653 	jal	8001194c <siscanf>
800084c0:	00000000 	nop
800084c4:	184001c4 	blez	v0,80008bd8 <_tzset_unlocked_r+0x854>
800084c8:	00000000 	nop
800084cc:	8fc20060 	lw	v0,96(s8)
800084d0:	00401825 	move	v1,v0
800084d4:	8fc20040 	lw	v0,64(s8)
800084d8:	00431021 	addu	v0,v0,v1
800084dc:	afc20040 	sw	v0,64(s8)
800084e0:	24020001 	li	v0,1
800084e4:	afc20044 	sw	v0,68(s8)
800084e8:	8fc20040 	lw	v0,64(s8)
800084ec:	80430000 	lb	v1,0(v0)
800084f0:	2402002d 	li	v0,45
800084f4:	14620008 	bne	v1,v0,80008518 <_tzset_unlocked_r+0x194>
800084f8:	00000000 	nop
800084fc:	2402ffff 	li	v0,-1
80008500:	afc20044 	sw	v0,68(s8)
80008504:	8fc20040 	lw	v0,64(s8)
80008508:	24420001 	addiu	v0,v0,1
8000850c:	afc20040 	sw	v0,64(s8)
80008510:	10000009 	b	80008538 <_tzset_unlocked_r+0x1b4>
80008514:	00000000 	nop
80008518:	8fc20040 	lw	v0,64(s8)
8000851c:	80430000 	lb	v1,0(v0)
80008520:	2402002b 	li	v0,43
80008524:	14620004 	bne	v1,v0,80008538 <_tzset_unlocked_r+0x1b4>
80008528:	00000000 	nop
8000852c:	8fc20040 	lw	v0,64(s8)
80008530:	24420001 	addiu	v0,v0,1
80008534:	afc20040 	sw	v0,64(s8)
80008538:	a7c00056 	sh	zero,86(s8)
8000853c:	a7c00058 	sh	zero,88(s8)
80008540:	27c40060 	addiu	a0,s8,96
80008544:	27c30054 	addiu	v1,s8,84
80008548:	27c20060 	addiu	v0,s8,96
8000854c:	afa2003c 	sw	v0,60(sp)
80008550:	27c20058 	addiu	v0,s8,88
80008554:	afa20034 	sw	v0,52(sp)
80008558:	27c20060 	addiu	v0,s8,96
8000855c:	afa2002c 	sw	v0,44(sp)
80008560:	27c20056 	addiu	v0,s8,86
80008564:	afa20024 	sw	v0,36(sp)
80008568:	00803825 	move	a3,a0
8000856c:	00603025 	move	a2,v1
80008570:	3c028002 	lui	v0,0x8002
80008574:	2445a4a0 	addiu	a1,v0,-23392
80008578:	8fc40040 	lw	a0,64(s8)
8000857c:	0c004653 	jal	8001194c <siscanf>
80008580:	00000000 	nop
80008584:	18400197 	blez	v0,80008be4 <_tzset_unlocked_r+0x860>
80008588:	00000000 	nop
8000858c:	97c20058 	lhu	v0,88(s8)
80008590:	00402025 	move	a0,v0
80008594:	97c20056 	lhu	v0,86(s8)
80008598:	00401825 	move	v1,v0
8000859c:	00601025 	move	v0,v1
800085a0:	00021100 	sll	v0,v0,0x4
800085a4:	00431023 	subu	v0,v0,v1
800085a8:	00021080 	sll	v0,v0,0x2
800085ac:	00822021 	addu	a0,a0,v0
800085b0:	97c20054 	lhu	v0,84(s8)
800085b4:	00401825 	move	v1,v0
800085b8:	00601025 	move	v0,v1
800085bc:	000210c0 	sll	v0,v0,0x3
800085c0:	00431023 	subu	v0,v0,v1
800085c4:	00021140 	sll	v0,v0,0x5
800085c8:	00431021 	addu	v0,v0,v1
800085cc:	00021100 	sll	v0,v0,0x4
800085d0:	00821821 	addu	v1,a0,v0
800085d4:	8fc20044 	lw	v0,68(s8)
800085d8:	00620018 	mult	v1,v0
800085dc:	8fc20050 	lw	v0,80(s8)
800085e0:	00001812 	mflo	v1
800085e4:	ac430028 	sw	v1,40(v0)
800085e8:	3c028003 	lui	v0,0x8003
800085ec:	2442dd28 	addiu	v0,v0,-8920
800085f0:	af828038 	sw	v0,-32712(gp)
800085f4:	8fc20060 	lw	v0,96(s8)
800085f8:	00401825 	move	v1,v0
800085fc:	8fc20040 	lw	v0,64(s8)
80008600:	00431021 	addu	v0,v0,v1
80008604:	afc20040 	sw	v0,64(s8)
80008608:	27c20060 	addiu	v0,s8,96
8000860c:	00403825 	move	a3,v0
80008610:	3c028003 	lui	v0,0x8003
80008614:	2446dd38 	addiu	a2,v0,-8904
80008618:	3c028002 	lui	v0,0x8002
8000861c:	2445a490 	addiu	a1,v0,-23408
80008620:	8fc40040 	lw	a0,64(s8)
80008624:	0c004653 	jal	8001194c <siscanf>
80008628:	00000000 	nop
8000862c:	1c400009 	bgtz	v0,80008654 <_tzset_unlocked_r+0x2d0>
80008630:	00000000 	nop
80008634:	8f828038 	lw	v0,-32712(gp)
80008638:	af82803c 	sw	v0,-32708(gp)
8000863c:	8fc20050 	lw	v0,80(s8)
80008640:	8c420028 	lw	v0,40(v0)
80008644:	af828340 	sw	v0,-31936(gp)
80008648:	af80833c 	sw	zero,-31940(gp)
8000864c:	10000169 	b	80008bf4 <_tzset_unlocked_r+0x870>
80008650:	00000000 	nop
80008654:	3c028003 	lui	v0,0x8003
80008658:	2442dd38 	addiu	v0,v0,-8904
8000865c:	af82803c 	sw	v0,-32708(gp)
80008660:	8fc20060 	lw	v0,96(s8)
80008664:	00401825 	move	v1,v0
80008668:	8fc20040 	lw	v0,64(s8)
8000866c:	00431021 	addu	v0,v0,v1
80008670:	afc20040 	sw	v0,64(s8)
80008674:	24020001 	li	v0,1
80008678:	afc20044 	sw	v0,68(s8)
8000867c:	8fc20040 	lw	v0,64(s8)
80008680:	80430000 	lb	v1,0(v0)
80008684:	2402002d 	li	v0,45
80008688:	14620008 	bne	v1,v0,800086ac <_tzset_unlocked_r+0x328>
8000868c:	00000000 	nop
80008690:	2402ffff 	li	v0,-1
80008694:	afc20044 	sw	v0,68(s8)
80008698:	8fc20040 	lw	v0,64(s8)
8000869c:	24420001 	addiu	v0,v0,1
800086a0:	afc20040 	sw	v0,64(s8)
800086a4:	10000009 	b	800086cc <_tzset_unlocked_r+0x348>
800086a8:	00000000 	nop
800086ac:	8fc20040 	lw	v0,64(s8)
800086b0:	80430000 	lb	v1,0(v0)
800086b4:	2402002b 	li	v0,43
800086b8:	14620004 	bne	v1,v0,800086cc <_tzset_unlocked_r+0x348>
800086bc:	00000000 	nop
800086c0:	8fc20040 	lw	v0,64(s8)
800086c4:	24420001 	addiu	v0,v0,1
800086c8:	afc20040 	sw	v0,64(s8)
800086cc:	a7c00054 	sh	zero,84(s8)
800086d0:	a7c00056 	sh	zero,86(s8)
800086d4:	a7c00058 	sh	zero,88(s8)
800086d8:	afc00060 	sw	zero,96(s8)
800086dc:	27c40060 	addiu	a0,s8,96
800086e0:	27c30054 	addiu	v1,s8,84
800086e4:	27c20060 	addiu	v0,s8,96
800086e8:	afa2003c 	sw	v0,60(sp)
800086ec:	27c20058 	addiu	v0,s8,88
800086f0:	afa20034 	sw	v0,52(sp)
800086f4:	27c20060 	addiu	v0,s8,96
800086f8:	afa2002c 	sw	v0,44(sp)
800086fc:	27c20056 	addiu	v0,s8,86
80008700:	afa20024 	sw	v0,36(sp)
80008704:	00803825 	move	a3,a0
80008708:	00603025 	move	a2,v1
8000870c:	3c028002 	lui	v0,0x8002
80008710:	2445a4a0 	addiu	a1,v0,-23392
80008714:	8fc40040 	lw	a0,64(s8)
80008718:	0c004653 	jal	8001194c <siscanf>
8000871c:	00000000 	nop
80008720:	1c400008 	bgtz	v0,80008744 <_tzset_unlocked_r+0x3c0>
80008724:	00000000 	nop
80008728:	8fc20050 	lw	v0,80(s8)
8000872c:	8c420028 	lw	v0,40(v0)
80008730:	2443f1f0 	addiu	v1,v0,-3600
80008734:	8fc20050 	lw	v0,80(s8)
80008738:	ac430050 	sw	v1,80(v0)
8000873c:	10000018 	b	800087a0 <_tzset_unlocked_r+0x41c>
80008740:	00000000 	nop
80008744:	97c20058 	lhu	v0,88(s8)
80008748:	00402025 	move	a0,v0
8000874c:	97c20056 	lhu	v0,86(s8)
80008750:	00401825 	move	v1,v0
80008754:	00601025 	move	v0,v1
80008758:	00021100 	sll	v0,v0,0x4
8000875c:	00431023 	subu	v0,v0,v1
80008760:	00021080 	sll	v0,v0,0x2
80008764:	00822021 	addu	a0,a0,v0
80008768:	97c20054 	lhu	v0,84(s8)
8000876c:	00401825 	move	v1,v0
80008770:	00601025 	move	v0,v1
80008774:	000210c0 	sll	v0,v0,0x3
80008778:	00431023 	subu	v0,v0,v1
8000877c:	00021140 	sll	v0,v0,0x5
80008780:	00431021 	addu	v0,v0,v1
80008784:	00021100 	sll	v0,v0,0x4
80008788:	00821821 	addu	v1,a0,v0
8000878c:	8fc20044 	lw	v0,68(s8)
80008790:	00620018 	mult	v1,v0
80008794:	8fc20050 	lw	v0,80(s8)
80008798:	00001812 	mflo	v1
8000879c:	ac430050 	sw	v1,80(v0)
800087a0:	8fc20060 	lw	v0,96(s8)
800087a4:	00401825 	move	v1,v0
800087a8:	8fc20040 	lw	v0,64(s8)
800087ac:	00431021 	addu	v0,v0,v1
800087b0:	afc20040 	sw	v0,64(s8)
800087b4:	afc00048 	sw	zero,72(s8)
800087b8:	100000ee 	b	80008b74 <_tzset_unlocked_r+0x7f0>
800087bc:	00000000 	nop
800087c0:	8fc20040 	lw	v0,64(s8)
800087c4:	80430000 	lb	v1,0(v0)
800087c8:	2402002c 	li	v0,44
800087cc:	14620004 	bne	v1,v0,800087e0 <_tzset_unlocked_r+0x45c>
800087d0:	00000000 	nop
800087d4:	8fc20040 	lw	v0,64(s8)
800087d8:	24420001 	addiu	v0,v0,1
800087dc:	afc20040 	sw	v0,64(s8)
800087e0:	8fc20040 	lw	v0,64(s8)
800087e4:	80430000 	lb	v1,0(v0)
800087e8:	2402004d 	li	v0,77
800087ec:	14620056 	bne	v1,v0,80008948 <_tzset_unlocked_r+0x5c4>
800087f0:	00000000 	nop
800087f4:	27c40060 	addiu	a0,s8,96
800087f8:	27c3005a 	addiu	v1,s8,90
800087fc:	27c20060 	addiu	v0,s8,96
80008800:	afa2003c 	sw	v0,60(sp)
80008804:	27c2005e 	addiu	v0,s8,94
80008808:	afa20034 	sw	v0,52(sp)
8000880c:	27c20060 	addiu	v0,s8,96
80008810:	afa2002c 	sw	v0,44(sp)
80008814:	27c2005c 	addiu	v0,s8,92
80008818:	afa20024 	sw	v0,36(sp)
8000881c:	00803825 	move	a3,a0
80008820:	00603025 	move	a2,v1
80008824:	3c028002 	lui	v0,0x8002
80008828:	2445a4b8 	addiu	a1,v0,-23368
8000882c:	8fc40040 	lw	a0,64(s8)
80008830:	0c004653 	jal	8001194c <siscanf>
80008834:	00000000 	nop
80008838:	00401825 	move	v1,v0
8000883c:	24020003 	li	v0,3
80008840:	146200eb 	bne	v1,v0,80008bf0 <_tzset_unlocked_r+0x86c>
80008844:	00000000 	nop
80008848:	97c2005a 	lhu	v0,90(s8)
8000884c:	104000e8 	beqz	v0,80008bf0 <_tzset_unlocked_r+0x86c>
80008850:	00000000 	nop
80008854:	97c2005a 	lhu	v0,90(s8)
80008858:	2c42000d 	sltiu	v0,v0,13
8000885c:	104000e4 	beqz	v0,80008bf0 <_tzset_unlocked_r+0x86c>
80008860:	00000000 	nop
80008864:	97c2005c 	lhu	v0,92(s8)
80008868:	104000e1 	beqz	v0,80008bf0 <_tzset_unlocked_r+0x86c>
8000886c:	00000000 	nop
80008870:	97c2005c 	lhu	v0,92(s8)
80008874:	2c420006 	sltiu	v0,v0,6
80008878:	104000dd 	beqz	v0,80008bf0 <_tzset_unlocked_r+0x86c>
8000887c:	00000000 	nop
80008880:	97c2005e 	lhu	v0,94(s8)
80008884:	2c420007 	sltiu	v0,v0,7
80008888:	104000d9 	beqz	v0,80008bf0 <_tzset_unlocked_r+0x86c>
8000888c:	00000000 	nop
80008890:	8fc40050 	lw	a0,80(s8)
80008894:	8fc30048 	lw	v1,72(s8)
80008898:	00601025 	move	v0,v1
8000889c:	00021080 	sll	v0,v0,0x2
800088a0:	00431021 	addu	v0,v0,v1
800088a4:	000210c0 	sll	v0,v0,0x3
800088a8:	00821021 	addu	v0,a0,v0
800088ac:	2403004d 	li	v1,77
800088b0:	a0430008 	sb	v1,8(v0)
800088b4:	97c2005a 	lhu	v0,90(s8)
800088b8:	00402825 	move	a1,v0
800088bc:	8fc40050 	lw	a0,80(s8)
800088c0:	8fc30048 	lw	v1,72(s8)
800088c4:	00601025 	move	v0,v1
800088c8:	00021080 	sll	v0,v0,0x2
800088cc:	00431021 	addu	v0,v0,v1
800088d0:	000210c0 	sll	v0,v0,0x3
800088d4:	00821021 	addu	v0,a0,v0
800088d8:	ac45000c 	sw	a1,12(v0)
800088dc:	97c2005c 	lhu	v0,92(s8)
800088e0:	00402825 	move	a1,v0
800088e4:	8fc40050 	lw	a0,80(s8)
800088e8:	8fc30048 	lw	v1,72(s8)
800088ec:	00601025 	move	v0,v1
800088f0:	00021080 	sll	v0,v0,0x2
800088f4:	00431021 	addu	v0,v0,v1
800088f8:	000210c0 	sll	v0,v0,0x3
800088fc:	00821021 	addu	v0,a0,v0
80008900:	ac450010 	sw	a1,16(v0)
80008904:	97c2005e 	lhu	v0,94(s8)
80008908:	00402825 	move	a1,v0
8000890c:	8fc40050 	lw	a0,80(s8)
80008910:	8fc30048 	lw	v1,72(s8)
80008914:	00601025 	move	v0,v1
80008918:	00021080 	sll	v0,v0,0x2
8000891c:	00431021 	addu	v0,v0,v1
80008920:	000210c0 	sll	v0,v0,0x3
80008924:	00821021 	addu	v0,a0,v0
80008928:	ac450014 	sw	a1,20(v0)
8000892c:	8fc20060 	lw	v0,96(s8)
80008930:	00401825 	move	v1,v0
80008934:	8fc20040 	lw	v0,64(s8)
80008938:	00431021 	addu	v0,v0,v1
8000893c:	afc20040 	sw	v0,64(s8)
80008940:	1000004f 	b	80008a80 <_tzset_unlocked_r+0x6fc>
80008944:	00000000 	nop
80008948:	8fc20040 	lw	v0,64(s8)
8000894c:	80430000 	lb	v1,0(v0)
80008950:	2402004a 	li	v0,74
80008954:	14620008 	bne	v1,v0,80008978 <_tzset_unlocked_r+0x5f4>
80008958:	00000000 	nop
8000895c:	2402004a 	li	v0,74
80008960:	afc2004c 	sw	v0,76(s8)
80008964:	8fc20040 	lw	v0,64(s8)
80008968:	24420001 	addiu	v0,v0,1
8000896c:	afc20040 	sw	v0,64(s8)
80008970:	10000003 	b	80008980 <_tzset_unlocked_r+0x5fc>
80008974:	00000000 	nop
80008978:	24020044 	li	v0,68
8000897c:	afc2004c 	sw	v0,76(s8)
80008980:	27c20064 	addiu	v0,s8,100
80008984:	2406000a 	li	a2,10
80008988:	00402825 	move	a1,v0
8000898c:	8fc40040 	lw	a0,64(s8)
80008990:	0c0048f8 	jal	800123e0 <strtoul>
80008994:	00000000 	nop
80008998:	3042ffff 	andi	v0,v0,0xffff
8000899c:	a7c2005e 	sh	v0,94(s8)
800089a0:	8fc20064 	lw	v0,100(s8)
800089a4:	8fc30040 	lw	v1,64(s8)
800089a8:	1462001e 	bne	v1,v0,80008a24 <_tzset_unlocked_r+0x6a0>
800089ac:	00000000 	nop
800089b0:	8fc20048 	lw	v0,72(s8)
800089b4:	1440000e 	bnez	v0,800089f0 <_tzset_unlocked_r+0x66c>
800089b8:	00000000 	nop
800089bc:	8fc20050 	lw	v0,80(s8)
800089c0:	2403004d 	li	v1,77
800089c4:	a0430008 	sb	v1,8(v0)
800089c8:	8fc20050 	lw	v0,80(s8)
800089cc:	24030003 	li	v1,3
800089d0:	ac43000c 	sw	v1,12(v0)
800089d4:	8fc20050 	lw	v0,80(s8)
800089d8:	24030002 	li	v1,2
800089dc:	ac430010 	sw	v1,16(v0)
800089e0:	8fc20050 	lw	v0,80(s8)
800089e4:	ac400014 	sw	zero,20(v0)
800089e8:	10000023 	b	80008a78 <_tzset_unlocked_r+0x6f4>
800089ec:	00000000 	nop
800089f0:	8fc20050 	lw	v0,80(s8)
800089f4:	2403004d 	li	v1,77
800089f8:	a0430030 	sb	v1,48(v0)
800089fc:	8fc20050 	lw	v0,80(s8)
80008a00:	2403000b 	li	v1,11
80008a04:	ac430034 	sw	v1,52(v0)
80008a08:	8fc20050 	lw	v0,80(s8)
80008a0c:	24030001 	li	v1,1
80008a10:	ac430038 	sw	v1,56(v0)
80008a14:	8fc20050 	lw	v0,80(s8)
80008a18:	ac40003c 	sw	zero,60(v0)
80008a1c:	10000016 	b	80008a78 <_tzset_unlocked_r+0x6f4>
80008a20:	00000000 	nop
80008a24:	8fc2004c 	lw	v0,76(s8)
80008a28:	00022600 	sll	a0,v0,0x18
80008a2c:	00042603 	sra	a0,a0,0x18
80008a30:	8fc50050 	lw	a1,80(s8)
80008a34:	8fc30048 	lw	v1,72(s8)
80008a38:	00601025 	move	v0,v1
80008a3c:	00021080 	sll	v0,v0,0x2
80008a40:	00431021 	addu	v0,v0,v1
80008a44:	000210c0 	sll	v0,v0,0x3
80008a48:	00a21021 	addu	v0,a1,v0
80008a4c:	a0440008 	sb	a0,8(v0)
80008a50:	97c2005e 	lhu	v0,94(s8)
80008a54:	00402825 	move	a1,v0
80008a58:	8fc40050 	lw	a0,80(s8)
80008a5c:	8fc30048 	lw	v1,72(s8)
80008a60:	00601025 	move	v0,v1
80008a64:	00021080 	sll	v0,v0,0x2
80008a68:	00431021 	addu	v0,v0,v1
80008a6c:	000210c0 	sll	v0,v0,0x3
80008a70:	00821021 	addu	v0,a0,v0
80008a74:	ac450014 	sw	a1,20(v0)
80008a78:	8fc20064 	lw	v0,100(s8)
80008a7c:	afc20040 	sw	v0,64(s8)
80008a80:	24020002 	li	v0,2
80008a84:	a7c20054 	sh	v0,84(s8)
80008a88:	a7c00056 	sh	zero,86(s8)
80008a8c:	a7c00058 	sh	zero,88(s8)
80008a90:	afc00060 	sw	zero,96(s8)
80008a94:	8fc20040 	lw	v0,64(s8)
80008a98:	80430000 	lb	v1,0(v0)
80008a9c:	2402002f 	li	v0,47
80008aa0:	14620012 	bne	v1,v0,80008aec <_tzset_unlocked_r+0x768>
80008aa4:	00000000 	nop
80008aa8:	27c40060 	addiu	a0,s8,96
80008aac:	27c30054 	addiu	v1,s8,84
80008ab0:	27c20060 	addiu	v0,s8,96
80008ab4:	afa2003c 	sw	v0,60(sp)
80008ab8:	27c20058 	addiu	v0,s8,88
80008abc:	afa20034 	sw	v0,52(sp)
80008ac0:	27c20060 	addiu	v0,s8,96
80008ac4:	afa2002c 	sw	v0,44(sp)
80008ac8:	27c20056 	addiu	v0,s8,86
80008acc:	afa20024 	sw	v0,36(sp)
80008ad0:	00803825 	move	a3,a0
80008ad4:	00603025 	move	a2,v1
80008ad8:	3c028002 	lui	v0,0x8002
80008adc:	2445a4d0 	addiu	a1,v0,-23344
80008ae0:	8fc40040 	lw	a0,64(s8)
80008ae4:	0c004653 	jal	8001194c <siscanf>
80008ae8:	00000000 	nop
80008aec:	97c20058 	lhu	v0,88(s8)
80008af0:	00402025 	move	a0,v0
80008af4:	97c20056 	lhu	v0,86(s8)
80008af8:	00401825 	move	v1,v0
80008afc:	00601025 	move	v0,v1
80008b00:	00021100 	sll	v0,v0,0x4
80008b04:	00431023 	subu	v0,v0,v1
80008b08:	00021080 	sll	v0,v0,0x2
80008b0c:	00822021 	addu	a0,a0,v0
80008b10:	97c20054 	lhu	v0,84(s8)
80008b14:	00401825 	move	v1,v0
80008b18:	00601025 	move	v0,v1
80008b1c:	000210c0 	sll	v0,v0,0x3
80008b20:	00431023 	subu	v0,v0,v1
80008b24:	00021140 	sll	v0,v0,0x5
80008b28:	00431021 	addu	v0,v0,v1
80008b2c:	00021100 	sll	v0,v0,0x4
80008b30:	00822021 	addu	a0,a0,v0
80008b34:	8fc50050 	lw	a1,80(s8)
80008b38:	8fc30048 	lw	v1,72(s8)
80008b3c:	00601025 	move	v0,v1
80008b40:	00021080 	sll	v0,v0,0x2
80008b44:	00431021 	addu	v0,v0,v1
80008b48:	000210c0 	sll	v0,v0,0x3
80008b4c:	00a21021 	addu	v0,a1,v0
80008b50:	ac440018 	sw	a0,24(v0)
80008b54:	8fc20060 	lw	v0,96(s8)
80008b58:	00401825 	move	v1,v0
80008b5c:	8fc20040 	lw	v0,64(s8)
80008b60:	00431021 	addu	v0,v0,v1
80008b64:	afc20040 	sw	v0,64(s8)
80008b68:	8fc20048 	lw	v0,72(s8)
80008b6c:	24420001 	addiu	v0,v0,1
80008b70:	afc20048 	sw	v0,72(s8)
80008b74:	8fc20048 	lw	v0,72(s8)
80008b78:	28420002 	slti	v0,v0,2
80008b7c:	1440ff10 	bnez	v0,800087c0 <_tzset_unlocked_r+0x43c>
80008b80:	00000000 	nop
80008b84:	8fc20050 	lw	v0,80(s8)
80008b88:	8c420004 	lw	v0,4(v0)
80008b8c:	00402025 	move	a0,v0
80008b90:	0c001f34 	jal	80007cd0 <__tzcalc_limits>
80008b94:	00000000 	nop
80008b98:	8fc20050 	lw	v0,80(s8)
80008b9c:	8c420028 	lw	v0,40(v0)
80008ba0:	af828340 	sw	v0,-31936(gp)
80008ba4:	8fc20050 	lw	v0,80(s8)
80008ba8:	8c430028 	lw	v1,40(v0)
80008bac:	8fc20050 	lw	v0,80(s8)
80008bb0:	8c420050 	lw	v0,80(v0)
80008bb4:	00621026 	xor	v0,v1,v0
80008bb8:	0002102b 	sltu	v0,zero,v0
80008bbc:	304200ff 	andi	v0,v0,0xff
80008bc0:	af82833c 	sw	v0,-31940(gp)
80008bc4:	1000000b 	b	80008bf4 <_tzset_unlocked_r+0x870>
80008bc8:	00000000 	nop
80008bcc:	00000000 	nop
80008bd0:	10000008 	b	80008bf4 <_tzset_unlocked_r+0x870>
80008bd4:	00000000 	nop
80008bd8:	00000000 	nop
80008bdc:	10000005 	b	80008bf4 <_tzset_unlocked_r+0x870>
80008be0:	00000000 	nop
80008be4:	00000000 	nop
80008be8:	10000002 	b	80008bf4 <_tzset_unlocked_r+0x870>
80008bec:	00000000 	nop
80008bf0:	00000000 	nop
80008bf4:	03c0e825 	move	sp,s8
80008bf8:	dfbf0070 	ld	ra,112(sp)
80008bfc:	dfbe0068 	ld	s8,104(sp)
80008c00:	27bd0078 	addiu	sp,sp,120
80008c04:	03e00008 	jr	ra
80008c08:	00000000 	nop

80008c0c <_tzset_r>:
80008c0c:	27bdffd0 	addiu	sp,sp,-48
80008c10:	ffbf0028 	sd	ra,40(sp)
80008c14:	ffbe0020 	sd	s8,32(sp)
80008c18:	03a0f025 	move	s8,sp
80008c1c:	afc40030 	sw	a0,48(s8)
80008c20:	0c0020ad 	jal	800082b4 <__tz_lock>
80008c24:	00000000 	nop
80008c28:	8fc40030 	lw	a0,48(s8)
80008c2c:	0c0020e1 	jal	80008384 <_tzset_unlocked_r>
80008c30:	00000000 	nop
80008c34:	0c0020b6 	jal	800082d8 <__tz_unlock>
80008c38:	00000000 	nop
80008c3c:	00000000 	nop
80008c40:	03c0e825 	move	sp,s8
80008c44:	dfbf0028 	ld	ra,40(sp)
80008c48:	dfbe0020 	ld	s8,32(sp)
80008c4c:	27bd0030 	addiu	sp,sp,48
80008c50:	03e00008 	jr	ra
80008c54:	00000000 	nop

80008c58 <__sbprintf>:
80008c58:	27bdfb58 	addiu	sp,sp,-1192
80008c5c:	ffbf04a0 	sd	ra,1184(sp)
80008c60:	ffbe0498 	sd	s8,1176(sp)
80008c64:	ffb00490 	sd	s0,1168(sp)
80008c68:	03a0f025 	move	s8,sp
80008c6c:	afc404a8 	sw	a0,1192(s8)
80008c70:	00a08025 	move	s0,a1
80008c74:	afc604b8 	sw	a2,1208(s8)
80008c78:	afc704c0 	sw	a3,1216(s8)
80008c7c:	8603000c 	lh	v1,12(s0)
80008c80:	2402fffd 	li	v0,-3
80008c84:	00621024 	and	v0,v1,v0
80008c88:	00021400 	sll	v0,v0,0x10
80008c8c:	00021403 	sra	v0,v0,0x10
80008c90:	a7c20034 	sh	v0,52(s8)
80008c94:	8e020064 	lw	v0,100(s0)
80008c98:	afc2008c 	sw	v0,140(s8)
80008c9c:	8602000e 	lh	v0,14(s0)
80008ca0:	a7c20036 	sh	v0,54(s8)
80008ca4:	8e02001c 	lw	v0,28(s0)
80008ca8:	afc20044 	sw	v0,68(s8)
80008cac:	8e020024 	lw	v0,36(s0)
80008cb0:	afc2004c 	sw	v0,76(s8)
80008cb4:	27c20090 	addiu	v0,s8,144
80008cb8:	afc20028 	sw	v0,40(s8)
80008cbc:	8fc20028 	lw	v0,40(s8)
80008cc0:	afc20038 	sw	v0,56(s8)
80008cc4:	24020400 	li	v0,1024
80008cc8:	afc20030 	sw	v0,48(s8)
80008ccc:	8fc20030 	lw	v0,48(s8)
80008cd0:	afc2003c 	sw	v0,60(s8)
80008cd4:	afc00040 	sw	zero,64(s8)
80008cd8:	27c20028 	addiu	v0,s8,40
80008cdc:	8fc704c0 	lw	a3,1216(s8)
80008ce0:	8fc604b8 	lw	a2,1208(s8)
80008ce4:	00402825 	move	a1,v0
80008ce8:	8fc404a8 	lw	a0,1192(s8)
80008cec:	0c002372 	jal	80008dc8 <_vfprintf_r>
80008cf0:	00000000 	nop
80008cf4:	afc20020 	sw	v0,32(s8)
80008cf8:	8fc20020 	lw	v0,32(s8)
80008cfc:	0440000a 	bltz	v0,80008d28 <__sbprintf+0xd0>
80008d00:	00000000 	nop
80008d04:	27c20028 	addiu	v0,s8,40
80008d08:	00402825 	move	a1,v0
80008d0c:	8fc404a8 	lw	a0,1192(s8)
80008d10:	0c000b45 	jal	80002d14 <_fflush_r>
80008d14:	00000000 	nop
80008d18:	10400003 	beqz	v0,80008d28 <__sbprintf+0xd0>
80008d1c:	00000000 	nop
80008d20:	2402ffff 	li	v0,-1
80008d24:	afc20020 	sw	v0,32(s8)
80008d28:	87c20034 	lh	v0,52(s8)
80008d2c:	3042ffff 	andi	v0,v0,0xffff
80008d30:	30420040 	andi	v0,v0,0x40
80008d34:	10400006 	beqz	v0,80008d50 <__sbprintf+0xf8>
80008d38:	00000000 	nop
80008d3c:	8602000c 	lh	v0,12(s0)
80008d40:	34420040 	ori	v0,v0,0x40
80008d44:	00021400 	sll	v0,v0,0x10
80008d48:	00021403 	sra	v0,v0,0x10
80008d4c:	a602000c 	sh	v0,12(s0)
80008d50:	8fc20020 	lw	v0,32(s8)
80008d54:	03c0e825 	move	sp,s8
80008d58:	dfbf04a0 	ld	ra,1184(sp)
80008d5c:	dfbe0498 	ld	s8,1176(sp)
80008d60:	dfb00490 	ld	s0,1168(sp)
80008d64:	27bd04a8 	addiu	sp,sp,1192
80008d68:	03e00008 	jr	ra
80008d6c:	00000000 	nop

80008d70 <vfprintf>:
80008d70:	27bdffc8 	addiu	sp,sp,-56
80008d74:	ffbf0030 	sd	ra,48(sp)
80008d78:	ffbe0028 	sd	s8,40(sp)
80008d7c:	03a0f025 	move	s8,sp
80008d80:	afc40038 	sw	a0,56(s8)
80008d84:	afc50040 	sw	a1,64(s8)
80008d88:	afc60048 	sw	a2,72(s8)
80008d8c:	8f828028 	lw	v0,-32728(gp)
80008d90:	8fc70048 	lw	a3,72(s8)
80008d94:	8fc60040 	lw	a2,64(s8)
80008d98:	8fc50038 	lw	a1,56(s8)
80008d9c:	00402025 	move	a0,v0
80008da0:	0c002372 	jal	80008dc8 <_vfprintf_r>
80008da4:	00000000 	nop
80008da8:	afc20020 	sw	v0,32(s8)
80008dac:	8fc20020 	lw	v0,32(s8)
80008db0:	03c0e825 	move	sp,s8
80008db4:	dfbf0030 	ld	ra,48(sp)
80008db8:	dfbe0028 	ld	s8,40(sp)
80008dbc:	27bd0038 	addiu	sp,sp,56
80008dc0:	03e00008 	jr	ra
80008dc4:	00000000 	nop

80008dc8 <_vfprintf_r>:
80008dc8:	27bdfe78 	addiu	sp,sp,-392
80008dcc:	ffbf0180 	sd	ra,384(sp)
80008dd0:	ffbe0178 	sd	s8,376(sp)
80008dd4:	ffb50170 	sd	s5,368(sp)
80008dd8:	ffb40168 	sd	s4,360(sp)
80008ddc:	ffb30160 	sd	s3,352(sp)
80008de0:	ffb20158 	sd	s2,344(sp)
80008de4:	ffb10150 	sd	s1,336(sp)
80008de8:	ffb00148 	sd	s0,328(sp)
80008dec:	03a0f025 	move	s8,sp
80008df0:	afc40188 	sw	a0,392(s8)
80008df4:	afc50190 	sw	a1,400(s8)
80008df8:	afc60198 	sw	a2,408(s8)
80008dfc:	afc701a0 	sw	a3,416(s8)
80008e00:	8fc40188 	lw	a0,392(s8)
80008e04:	0c0039e3 	jal	8000e78c <_localeconv_r>
80008e08:	00000000 	nop
80008e0c:	8c420000 	lw	v0,0(v0)
80008e10:	afc20088 	sw	v0,136(s8)
80008e14:	8fc40088 	lw	a0,136(s8)
80008e18:	0c001f1d 	jal	80007c74 <strlen>
80008e1c:	00000000 	nop
80008e20:	afc2008c 	sw	v0,140(s8)
80008e24:	ffc000a8 	sd	zero,168(s8)
80008e28:	afc00054 	sw	zero,84(s8)
80008e2c:	afc000c0 	sw	zero,192(s8)
80008e30:	afc00078 	sw	zero,120(s8)
80008e34:	afc0007c 	sw	zero,124(s8)
80008e38:	8fc20188 	lw	v0,392(s8)
80008e3c:	afc20090 	sw	v0,144(s8)
80008e40:	8fc20090 	lw	v0,144(s8)
80008e44:	10400008 	beqz	v0,80008e68 <_vfprintf_r+0xa0>
80008e48:	00000000 	nop
80008e4c:	8fc20090 	lw	v0,144(s8)
80008e50:	8c420038 	lw	v0,56(v0)
80008e54:	14400004 	bnez	v0,80008e68 <_vfprintf_r+0xa0>
80008e58:	00000000 	nop
80008e5c:	8fc40090 	lw	a0,144(s8)
80008e60:	0c000cc3 	jal	8000330c <__sinit>
80008e64:	00000000 	nop
80008e68:	8fc20190 	lw	v0,400(s8)
80008e6c:	8442000c 	lh	v0,12(v0)
80008e70:	3042ffff 	andi	v0,v0,0xffff
80008e74:	30422000 	andi	v0,v0,0x2000
80008e78:	1440000e 	bnez	v0,80008eb4 <_vfprintf_r+0xec>
80008e7c:	00000000 	nop
80008e80:	8fc20190 	lw	v0,400(s8)
80008e84:	8442000c 	lh	v0,12(v0)
80008e88:	34422000 	ori	v0,v0,0x2000
80008e8c:	00021c00 	sll	v1,v0,0x10
80008e90:	00031c03 	sra	v1,v1,0x10
80008e94:	8fc20190 	lw	v0,400(s8)
80008e98:	a443000c 	sh	v1,12(v0)
80008e9c:	8fc20190 	lw	v0,400(s8)
80008ea0:	8c430064 	lw	v1,100(v0)
80008ea4:	2402dfff 	li	v0,-8193
80008ea8:	00621824 	and	v1,v1,v0
80008eac:	8fc20190 	lw	v0,400(s8)
80008eb0:	ac430064 	sw	v1,100(v0)
80008eb4:	8fc20190 	lw	v0,400(s8)
80008eb8:	8442000c 	lh	v0,12(v0)
80008ebc:	3042ffff 	andi	v0,v0,0xffff
80008ec0:	30420008 	andi	v0,v0,0x8
80008ec4:	10400005 	beqz	v0,80008edc <_vfprintf_r+0x114>
80008ec8:	00000000 	nop
80008ecc:	8fc20190 	lw	v0,400(s8)
80008ed0:	8c420010 	lw	v0,16(v0)
80008ed4:	1440000a 	bnez	v0,80008f00 <_vfprintf_r+0x138>
80008ed8:	00000000 	nop
80008edc:	8fc50190 	lw	a1,400(s8)
80008ee0:	8fc40188 	lw	a0,392(s8)
80008ee4:	0c002e52 	jal	8000b948 <__swsetup_r>
80008ee8:	00000000 	nop
80008eec:	10400004 	beqz	v0,80008f00 <_vfprintf_r+0x138>
80008ef0:	00000000 	nop
80008ef4:	2402ffff 	li	v0,-1
80008ef8:	100008d0 	b	8000b23c <_vfprintf_r+0x2474>
80008efc:	00000000 	nop
80008f00:	8fc20190 	lw	v0,400(s8)
80008f04:	8442000c 	lh	v0,12(v0)
80008f08:	3042ffff 	andi	v0,v0,0xffff
80008f0c:	3043001a 	andi	v1,v0,0x1a
80008f10:	2402000a 	li	v0,10
80008f14:	1462000e 	bne	v1,v0,80008f50 <_vfprintf_r+0x188>
80008f18:	00000000 	nop
80008f1c:	8fc20190 	lw	v0,400(s8)
80008f20:	8442000e 	lh	v0,14(v0)
80008f24:	0440000a 	bltz	v0,80008f50 <_vfprintf_r+0x188>
80008f28:	00000000 	nop
80008f2c:	8fc201a0 	lw	v0,416(s8)
80008f30:	00403825 	move	a3,v0
80008f34:	8fc60198 	lw	a2,408(s8)
80008f38:	8fc50190 	lw	a1,400(s8)
80008f3c:	8fc40188 	lw	a0,392(s8)
80008f40:	0c002316 	jal	80008c58 <__sbprintf>
80008f44:	00000000 	nop
80008f48:	100008bc 	b	8000b23c <_vfprintf_r+0x2474>
80008f4c:	00000000 	nop
80008f50:	8fd50198 	lw	s5,408(s8)
80008f54:	27d000d8 	addiu	s0,s8,216
80008f58:	afd000c8 	sw	s0,200(s8)
80008f5c:	afc000d0 	sw	zero,208(s8)
80008f60:	afc000cc 	sw	zero,204(s8)
80008f64:	afc00048 	sw	zero,72(s8)
80008f68:	02a09025 	move	s2,s5
80008f6c:	10000002 	b	80008f78 <_vfprintf_r+0x1b0>
80008f70:	00000000 	nop
80008f74:	26b50001 	addiu	s5,s5,1
80008f78:	82a20000 	lb	v0,0(s5)
80008f7c:	10400005 	beqz	v0,80008f94 <_vfprintf_r+0x1cc>
80008f80:	00000000 	nop
80008f84:	82a30000 	lb	v1,0(s5)
80008f88:	24020025 	li	v0,37
80008f8c:	1462fff9 	bne	v1,v0,80008f74 <_vfprintf_r+0x1ac>
80008f90:	00000000 	nop
80008f94:	02b28823 	subu	s1,s5,s2
80008f98:	1220001c 	beqz	s1,8000900c <_vfprintf_r+0x244>
80008f9c:	00000000 	nop
80008fa0:	ae120000 	sw	s2,0(s0)
80008fa4:	02201025 	move	v0,s1
80008fa8:	ae020004 	sw	v0,4(s0)
80008fac:	8fc200d0 	lw	v0,208(s8)
80008fb0:	02201825 	move	v1,s1
80008fb4:	00431021 	addu	v0,v0,v1
80008fb8:	afc200d0 	sw	v0,208(s8)
80008fbc:	26100008 	addiu	s0,s0,8
80008fc0:	8fc200cc 	lw	v0,204(s8)
80008fc4:	24420001 	addiu	v0,v0,1
80008fc8:	afc200cc 	sw	v0,204(s8)
80008fcc:	8fc200cc 	lw	v0,204(s8)
80008fd0:	28420008 	slti	v0,v0,8
80008fd4:	1440000a 	bnez	v0,80009000 <_vfprintf_r+0x238>
80008fd8:	00000000 	nop
80008fdc:	27c200c8 	addiu	v0,s8,200
80008fe0:	00403025 	move	a2,v0
80008fe4:	8fc50190 	lw	a1,400(s8)
80008fe8:	8fc40188 	lw	a0,392(s8)
80008fec:	0c00512c 	jal	800144b0 <__sprint_r>
80008ff0:	00000000 	nop
80008ff4:	14400813 	bnez	v0,8000b044 <_vfprintf_r+0x227c>
80008ff8:	00000000 	nop
80008ffc:	27d000d8 	addiu	s0,s8,216
80009000:	8fc20048 	lw	v0,72(s8)
80009004:	00511021 	addu	v0,v0,s1
80009008:	afc20048 	sw	v0,72(s8)
8000900c:	82a20000 	lb	v0,0(s5)
80009010:	104007fa 	beqz	v0,8000affc <_vfprintf_r+0x2234>
80009014:	00000000 	nop
80009018:	afd50094 	sw	s5,148(s8)
8000901c:	26b50001 	addiu	s5,s5,1
80009020:	00008825 	move	s1,zero
80009024:	afc0006c 	sw	zero,108(s8)
80009028:	afc0004c 	sw	zero,76(s8)
8000902c:	2402ffff 	li	v0,-1
80009030:	afc20050 	sw	v0,80(s8)
80009034:	a3c000a0 	sb	zero,160(s8)
80009038:	afc00058 	sw	zero,88(s8)
8000903c:	02a01025 	move	v0,s5
80009040:	24550001 	addiu	s5,v0,1
80009044:	80420000 	lb	v0,0(v0)
80009048:	0040a025 	move	s4,v0
8000904c:	2682ffe0 	addiu	v0,s4,-32
80009050:	2c430059 	sltiu	v1,v0,89
80009054:	1060036a 	beqz	v1,80009e00 <_vfprintf_r+0x1038>
80009058:	00000000 	nop
8000905c:	00021880 	sll	v1,v0,0x2
80009060:	3c028002 	lui	v0,0x8002
80009064:	2442a56c 	addiu	v0,v0,-23188
80009068:	00621021 	addu	v0,v1,v0
8000906c:	8c420000 	lw	v0,0(v0)
80009070:	00400008 	jr	v0
80009074:	00000000 	nop
80009078:	83c200a0 	lb	v0,160(s8)
8000907c:	1440ffef 	bnez	v0,8000903c <_vfprintf_r+0x274>
80009080:	00000000 	nop
80009084:	24020020 	li	v0,32
80009088:	a3c200a0 	sb	v0,160(s8)
8000908c:	1000ffeb 	b	8000903c <_vfprintf_r+0x274>
80009090:	00000000 	nop
80009094:	36310001 	ori	s1,s1,0x1
80009098:	1000ffe8 	b	8000903c <_vfprintf_r+0x274>
8000909c:	00000000 	nop
800090a0:	8fc201a0 	lw	v0,416(s8)
800090a4:	24430008 	addiu	v1,v0,8
800090a8:	afc301a0 	sw	v1,416(s8)
800090ac:	24420004 	addiu	v0,v0,4
800090b0:	8c420000 	lw	v0,0(v0)
800090b4:	afc2004c 	sw	v0,76(s8)
800090b8:	8fc2004c 	lw	v0,76(s8)
800090bc:	04400003 	bltz	v0,800090cc <_vfprintf_r+0x304>
800090c0:	00000000 	nop
800090c4:	1000ffdd 	b	8000903c <_vfprintf_r+0x274>
800090c8:	00000000 	nop
800090cc:	8fc2004c 	lw	v0,76(s8)
800090d0:	00021023 	negu	v0,v0
800090d4:	afc2004c 	sw	v0,76(s8)
800090d8:	36310004 	ori	s1,s1,0x4
800090dc:	1000ffd7 	b	8000903c <_vfprintf_r+0x274>
800090e0:	00000000 	nop
800090e4:	2402002b 	li	v0,43
800090e8:	a3c200a0 	sb	v0,160(s8)
800090ec:	1000ffd3 	b	8000903c <_vfprintf_r+0x274>
800090f0:	00000000 	nop
800090f4:	02a01025 	move	v0,s5
800090f8:	24550001 	addiu	s5,v0,1
800090fc:	80420000 	lb	v0,0(v0)
80009100:	0040a025 	move	s4,v0
80009104:	2402002a 	li	v0,42
80009108:	1682000e 	bne	s4,v0,80009144 <_vfprintf_r+0x37c>
8000910c:	00000000 	nop
80009110:	8fc201a0 	lw	v0,416(s8)
80009114:	24430008 	addiu	v1,v0,8
80009118:	afc301a0 	sw	v1,416(s8)
8000911c:	24420004 	addiu	v0,v0,4
80009120:	8c420000 	lw	v0,0(v0)
80009124:	afc20050 	sw	v0,80(s8)
80009128:	8fc20050 	lw	v0,80(s8)
8000912c:	0441ffc3 	bgez	v0,8000903c <_vfprintf_r+0x274>
80009130:	00000000 	nop
80009134:	2402ffff 	li	v0,-1
80009138:	afc20050 	sw	v0,80(s8)
8000913c:	1000ffbf 	b	8000903c <_vfprintf_r+0x274>
80009140:	00000000 	nop
80009144:	00009825 	move	s3,zero
80009148:	1000000c 	b	8000917c <_vfprintf_r+0x3b4>
8000914c:	00000000 	nop
80009150:	02601025 	move	v0,s3
80009154:	00021080 	sll	v0,v0,0x2
80009158:	00531021 	addu	v0,v0,s3
8000915c:	00021040 	sll	v0,v0,0x1
80009160:	00401825 	move	v1,v0
80009164:	2682ffd0 	addiu	v0,s4,-48
80009168:	00629821 	addu	s3,v1,v0
8000916c:	02a01025 	move	v0,s5
80009170:	24550001 	addiu	s5,v0,1
80009174:	80420000 	lb	v0,0(v0)
80009178:	0040a025 	move	s4,v0
8000917c:	2682ffd0 	addiu	v0,s4,-48
80009180:	2c42000a 	sltiu	v0,v0,10
80009184:	1440fff2 	bnez	v0,80009150 <_vfprintf_r+0x388>
80009188:	00000000 	nop
8000918c:	02601025 	move	v0,s3
80009190:	04410002 	bgez	v0,8000919c <_vfprintf_r+0x3d4>
80009194:	00000000 	nop
80009198:	2402ffff 	li	v0,-1
8000919c:	afc20050 	sw	v0,80(s8)
800091a0:	1000ffaa 	b	8000904c <_vfprintf_r+0x284>
800091a4:	00000000 	nop
800091a8:	36310080 	ori	s1,s1,0x80
800091ac:	1000ffa3 	b	8000903c <_vfprintf_r+0x274>
800091b0:	00000000 	nop
800091b4:	00009825 	move	s3,zero
800091b8:	02601025 	move	v0,s3
800091bc:	00021080 	sll	v0,v0,0x2
800091c0:	00531021 	addu	v0,v0,s3
800091c4:	00021040 	sll	v0,v0,0x1
800091c8:	00401825 	move	v1,v0
800091cc:	2682ffd0 	addiu	v0,s4,-48
800091d0:	00629821 	addu	s3,v1,v0
800091d4:	02a01025 	move	v0,s5
800091d8:	24550001 	addiu	s5,v0,1
800091dc:	80420000 	lb	v0,0(v0)
800091e0:	0040a025 	move	s4,v0
800091e4:	2682ffd0 	addiu	v0,s4,-48
800091e8:	2c42000a 	sltiu	v0,v0,10
800091ec:	1440fff2 	bnez	v0,800091b8 <_vfprintf_r+0x3f0>
800091f0:	00000000 	nop
800091f4:	afd3004c 	sw	s3,76(s8)
800091f8:	1000ff94 	b	8000904c <_vfprintf_r+0x284>
800091fc:	00000000 	nop
80009200:	36310008 	ori	s1,s1,0x8
80009204:	1000ff8d 	b	8000903c <_vfprintf_r+0x274>
80009208:	00000000 	nop
8000920c:	36310040 	ori	s1,s1,0x40
80009210:	1000ff8a 	b	8000903c <_vfprintf_r+0x274>
80009214:	00000000 	nop
80009218:	82a30000 	lb	v1,0(s5)
8000921c:	2402006c 	li	v0,108
80009220:	14620005 	bne	v1,v0,80009238 <_vfprintf_r+0x470>
80009224:	00000000 	nop
80009228:	26b50001 	addiu	s5,s5,1
8000922c:	36310020 	ori	s1,s1,0x20
80009230:	1000ff82 	b	8000903c <_vfprintf_r+0x274>
80009234:	00000000 	nop
80009238:	36310010 	ori	s1,s1,0x10
8000923c:	1000ff7f 	b	8000903c <_vfprintf_r+0x274>
80009240:	00000000 	nop
80009244:	36310020 	ori	s1,s1,0x20
80009248:	1000ff7c 	b	8000903c <_vfprintf_r+0x274>
8000924c:	00000000 	nop
80009250:	27d20118 	addiu	s2,s8,280
80009254:	8fc201a0 	lw	v0,416(s8)
80009258:	24430008 	addiu	v1,v0,8
8000925c:	afc301a0 	sw	v1,416(s8)
80009260:	24420004 	addiu	v0,v0,4
80009264:	8c420000 	lw	v0,0(v0)
80009268:	00021600 	sll	v0,v0,0x18
8000926c:	00021603 	sra	v0,v0,0x18
80009270:	a2420000 	sb	v0,0(s2)
80009274:	24020001 	li	v0,1
80009278:	afc20074 	sw	v0,116(s8)
8000927c:	a3c000a0 	sb	zero,160(s8)
80009280:	100002eb 	b	80009e30 <_vfprintf_r+0x1068>
80009284:	00000000 	nop
80009288:	36310010 	ori	s1,s1,0x10
8000928c:	32220020 	andi	v0,s1,0x20
80009290:	10400007 	beqz	v0,800092b0 <_vfprintf_r+0x4e8>
80009294:	00000000 	nop
80009298:	8fc201a0 	lw	v0,416(s8)
8000929c:	24430008 	addiu	v1,v0,8
800092a0:	afc301a0 	sw	v1,416(s8)
800092a4:	dc420000 	ld	v0,0(v0)
800092a8:	1000001e 	b	80009324 <_vfprintf_r+0x55c>
800092ac:	00000000 	nop
800092b0:	32220010 	andi	v0,s1,0x10
800092b4:	10400008 	beqz	v0,800092d8 <_vfprintf_r+0x510>
800092b8:	00000000 	nop
800092bc:	8fc201a0 	lw	v0,416(s8)
800092c0:	24430008 	addiu	v1,v0,8
800092c4:	afc301a0 	sw	v1,416(s8)
800092c8:	24420004 	addiu	v0,v0,4
800092cc:	8c420000 	lw	v0,0(v0)
800092d0:	10000014 	b	80009324 <_vfprintf_r+0x55c>
800092d4:	00000000 	nop
800092d8:	32220040 	andi	v0,s1,0x40
800092dc:	1040000c 	beqz	v0,80009310 <_vfprintf_r+0x548>
800092e0:	00000000 	nop
800092e4:	8fc201a0 	lw	v0,416(s8)
800092e8:	24430008 	addiu	v1,v0,8
800092ec:	afc301a0 	sw	v1,416(s8)
800092f0:	24420004 	addiu	v0,v0,4
800092f4:	8c420000 	lw	v0,0(v0)
800092f8:	00021400 	sll	v0,v0,0x10
800092fc:	00021403 	sra	v0,v0,0x10
80009300:	0002143c 	dsll32	v0,v0,0x10
80009304:	0002143f 	dsra32	v0,v0,0x10
80009308:	10000006 	b	80009324 <_vfprintf_r+0x55c>
8000930c:	00000000 	nop
80009310:	8fc201a0 	lw	v0,416(s8)
80009314:	24430008 	addiu	v1,v0,8
80009318:	afc301a0 	sw	v1,416(s8)
8000931c:	24420004 	addiu	v0,v0,4
80009320:	8c420000 	lw	v0,0(v0)
80009324:	ffc20060 	sd	v0,96(s8)
80009328:	dfc20060 	ld	v0,96(s8)
8000932c:	04410006 	bgez	v0,80009348 <_vfprintf_r+0x580>
80009330:	00000000 	nop
80009334:	dfc20060 	ld	v0,96(s8)
80009338:	0002102f 	dnegu	v0,v0
8000933c:	ffc20060 	sd	v0,96(s8)
80009340:	2402002d 	li	v0,45
80009344:	a3c200a0 	sb	v0,160(s8)
80009348:	24020001 	li	v0,1
8000934c:	afc20068 	sw	v0,104(s8)
80009350:	1000020f 	b	80009b90 <_vfprintf_r+0xdc8>
80009354:	00000000 	nop
80009358:	32220008 	andi	v0,s1,0x8
8000935c:	10400008 	beqz	v0,80009380 <_vfprintf_r+0x5b8>
80009360:	00000000 	nop
80009364:	8fc201a0 	lw	v0,416(s8)
80009368:	24430008 	addiu	v1,v0,8
8000936c:	afc301a0 	sw	v1,416(s8)
80009370:	d4400000 	ldc1	$f0,0(v0)
80009374:	f7c000a8 	sdc1	$f0,168(s8)
80009378:	10000006 	b	80009394 <_vfprintf_r+0x5cc>
8000937c:	00000000 	nop
80009380:	8fc201a0 	lw	v0,416(s8)
80009384:	24430008 	addiu	v1,v0,8
80009388:	afc301a0 	sw	v1,416(s8)
8000938c:	d4400000 	ldc1	$f0,0(v0)
80009390:	f7c000a8 	sdc1	$f0,168(s8)
80009394:	dfc300a8 	ld	v1,168(s8)
80009398:	2402ffff 	li	v0,-1
8000939c:	000217fc 	dsll32	v0,v0,0x1f
800093a0:	6442ffff 	daddiu	v0,v0,-1
800093a4:	00621824 	and	v1,v1,v0
800093a8:	24020001 	li	v0,1
800093ac:	d7808040 	ldc1	$f0,-32704(gp)
800093b0:	44a30800 	dmtc1	v1,$f1
800093b4:	00000000 	nop
800093b8:	46200837 	c.ule.d	$f1,$f0
800093bc:	00000000 	nop
800093c0:	45010002 	bc1t	800093cc <_vfprintf_r+0x604>
800093c4:	00000000 	nop
800093c8:	00001025 	move	v0,zero
800093cc:	304200ff 	andi	v0,v0,0xff
800093d0:	38420001 	xori	v0,v0,0x1
800093d4:	304200ff 	andi	v0,v0,0xff
800093d8:	10400004 	beqz	v0,800093ec <_vfprintf_r+0x624>
800093dc:	00000000 	nop
800093e0:	24020001 	li	v0,1
800093e4:	10000002 	b	800093f0 <_vfprintf_r+0x628>
800093e8:	00000000 	nop
800093ec:	00001025 	move	v0,zero
800093f0:	10400019 	beqz	v0,80009458 <_vfprintf_r+0x690>
800093f4:	00000000 	nop
800093f8:	d7c000a8 	ldc1	$f0,168(s8)
800093fc:	44a00800 	dmtc1	zero,$f1
80009400:	00000000 	nop
80009404:	4621003c 	c.lt.d	$f0,$f1
80009408:	00000000 	nop
8000940c:	45000003 	bc1f	8000941c <_vfprintf_r+0x654>
80009410:	00000000 	nop
80009414:	2402002d 	li	v0,45
80009418:	a3c200a0 	sb	v0,160(s8)
8000941c:	2a820048 	slti	v0,s4,72
80009420:	10400005 	beqz	v0,80009438 <_vfprintf_r+0x670>
80009424:	00000000 	nop
80009428:	3c028002 	lui	v0,0x8002
8000942c:	2452a4f0 	addiu	s2,v0,-23312
80009430:	10000003 	b	80009440 <_vfprintf_r+0x678>
80009434:	00000000 	nop
80009438:	3c028002 	lui	v0,0x8002
8000943c:	2452a4f8 	addiu	s2,v0,-23304
80009440:	24020003 	li	v0,3
80009444:	afc20074 	sw	v0,116(s8)
80009448:	2402ff7f 	li	v0,-129
8000944c:	02228824 	and	s1,s1,v0
80009450:	10000277 	b	80009e30 <_vfprintf_r+0x1068>
80009454:	00000000 	nop
80009458:	d7c000a8 	ldc1	$f0,168(s8)
8000945c:	46200031 	c.un.d	$f0,$f0
80009460:	00000000 	nop
80009464:	45000017 	bc1f	800094c4 <_vfprintf_r+0x6fc>
80009468:	00000000 	nop
8000946c:	dfc200a8 	ld	v0,168(s8)
80009470:	000217fe 	dsrl32	v0,v0,0x1f
80009474:	30420001 	andi	v0,v0,0x1
80009478:	10400003 	beqz	v0,80009488 <_vfprintf_r+0x6c0>
8000947c:	00000000 	nop
80009480:	2402002d 	li	v0,45
80009484:	a3c200a0 	sb	v0,160(s8)
80009488:	2a820048 	slti	v0,s4,72
8000948c:	10400005 	beqz	v0,800094a4 <_vfprintf_r+0x6dc>
80009490:	00000000 	nop
80009494:	3c028002 	lui	v0,0x8002
80009498:	2452a500 	addiu	s2,v0,-23296
8000949c:	10000003 	b	800094ac <_vfprintf_r+0x6e4>
800094a0:	00000000 	nop
800094a4:	3c028002 	lui	v0,0x8002
800094a8:	2452a508 	addiu	s2,v0,-23288
800094ac:	24020003 	li	v0,3
800094b0:	afc20074 	sw	v0,116(s8)
800094b4:	2402ff7f 	li	v0,-129
800094b8:	02228824 	and	s1,s1,v0
800094bc:	1000025c 	b	80009e30 <_vfprintf_r+0x1068>
800094c0:	00000000 	nop
800094c4:	8fc30050 	lw	v1,80(s8)
800094c8:	2402ffff 	li	v0,-1
800094cc:	14620005 	bne	v1,v0,800094e4 <_vfprintf_r+0x71c>
800094d0:	00000000 	nop
800094d4:	24020006 	li	v0,6
800094d8:	afc20050 	sw	v0,80(s8)
800094dc:	1000000c 	b	80009510 <_vfprintf_r+0x748>
800094e0:	00000000 	nop
800094e4:	24020067 	li	v0,103
800094e8:	12820004 	beq	s4,v0,800094fc <_vfprintf_r+0x734>
800094ec:	00000000 	nop
800094f0:	24020047 	li	v0,71
800094f4:	16820006 	bne	s4,v0,80009510 <_vfprintf_r+0x748>
800094f8:	00000000 	nop
800094fc:	8fc20050 	lw	v0,80(s8)
80009500:	14400003 	bnez	v0,80009510 <_vfprintf_r+0x748>
80009504:	00000000 	nop
80009508:	24020001 	li	v0,1
8000950c:	afc20050 	sw	v0,80(s8)
80009510:	36310100 	ori	s1,s1,0x100
80009514:	d7c000a8 	ldc1	$f0,168(s8)
80009518:	afb20044 	sw	s2,68(sp)
8000951c:	27c200c0 	addiu	v0,s8,192
80009520:	afa2003c 	sw	v0,60(sp)
80009524:	afb40034 	sw	s4,52(sp)
80009528:	27c200b0 	addiu	v0,s8,176
8000952c:	afa2002c 	sw	v0,44(sp)
80009530:	27c200a1 	addiu	v0,s8,161
80009534:	afa20024 	sw	v0,36(sp)
80009538:	02203825 	move	a3,s1
8000953c:	8fc60050 	lw	a2,80(s8)
80009540:	44250000 	dmfc1	a1,$f0
80009544:	8fc40188 	lw	a0,392(s8)
80009548:	0c002c9b 	jal	8000b26c <cvt>
8000954c:	00000000 	nop
80009550:	00409025 	move	s2,v0
80009554:	24020067 	li	v0,103
80009558:	12820004 	beq	s4,v0,8000956c <_vfprintf_r+0x7a4>
8000955c:	00000000 	nop
80009560:	24020047 	li	v0,71
80009564:	1682000e 	bne	s4,v0,800095a0 <_vfprintf_r+0x7d8>
80009568:	00000000 	nop
8000956c:	8fc200b0 	lw	v0,176(s8)
80009570:	2842fffd 	slti	v0,v0,-3
80009574:	14400006 	bnez	v0,80009590 <_vfprintf_r+0x7c8>
80009578:	00000000 	nop
8000957c:	8fc200b0 	lw	v0,176(s8)
80009580:	8fc30050 	lw	v1,80(s8)
80009584:	0062102a 	slt	v0,v1,v0
80009588:	10400004 	beqz	v0,8000959c <_vfprintf_r+0x7d4>
8000958c:	00000000 	nop
80009590:	2694fffe 	addiu	s4,s4,-2
80009594:	10000002 	b	800095a0 <_vfprintf_r+0x7d8>
80009598:	00000000 	nop
8000959c:	24140067 	li	s4,103
800095a0:	2a820066 	slti	v0,s4,102
800095a4:	1040001d 	beqz	v0,8000961c <_vfprintf_r+0x854>
800095a8:	00000000 	nop
800095ac:	8fc200b0 	lw	v0,176(s8)
800095b0:	2442ffff 	addiu	v0,v0,-1
800095b4:	afc200b0 	sw	v0,176(s8)
800095b8:	8fc200b0 	lw	v0,176(s8)
800095bc:	27c300b8 	addiu	v1,s8,184
800095c0:	02803025 	move	a2,s4
800095c4:	00402825 	move	a1,v0
800095c8:	00602025 	move	a0,v1
800095cc:	0c002d2a 	jal	8000b4a8 <exponent>
800095d0:	00000000 	nop
800095d4:	afc20054 	sw	v0,84(s8)
800095d8:	8fc200c0 	lw	v0,192(s8)
800095dc:	8fc30054 	lw	v1,84(s8)
800095e0:	00621021 	addu	v0,v1,v0
800095e4:	afc20074 	sw	v0,116(s8)
800095e8:	8fc200c0 	lw	v0,192(s8)
800095ec:	28420002 	slti	v0,v0,2
800095f0:	10400004 	beqz	v0,80009604 <_vfprintf_r+0x83c>
800095f4:	00000000 	nop
800095f8:	32220001 	andi	v0,s1,0x1
800095fc:	1040004d 	beqz	v0,80009734 <_vfprintf_r+0x96c>
80009600:	00000000 	nop
80009604:	8fc30074 	lw	v1,116(s8)
80009608:	8fc2008c 	lw	v0,140(s8)
8000960c:	00621021 	addu	v0,v1,v0
80009610:	afc20074 	sw	v0,116(s8)
80009614:	10000047 	b	80009734 <_vfprintf_r+0x96c>
80009618:	00000000 	nop
8000961c:	24020066 	li	v0,102
80009620:	16820024 	bne	s4,v0,800096b4 <_vfprintf_r+0x8ec>
80009624:	00000000 	nop
80009628:	8fc200b0 	lw	v0,176(s8)
8000962c:	18400011 	blez	v0,80009674 <_vfprintf_r+0x8ac>
80009630:	00000000 	nop
80009634:	8fc200b0 	lw	v0,176(s8)
80009638:	afc20074 	sw	v0,116(s8)
8000963c:	8fc20050 	lw	v0,80(s8)
80009640:	14400004 	bnez	v0,80009654 <_vfprintf_r+0x88c>
80009644:	00000000 	nop
80009648:	32220001 	andi	v0,s1,0x1
8000964c:	10400037 	beqz	v0,8000972c <_vfprintf_r+0x964>
80009650:	00000000 	nop
80009654:	8fc30050 	lw	v1,80(s8)
80009658:	8fc2008c 	lw	v0,140(s8)
8000965c:	00621821 	addu	v1,v1,v0
80009660:	8fc20074 	lw	v0,116(s8)
80009664:	00621021 	addu	v0,v1,v0
80009668:	afc20074 	sw	v0,116(s8)
8000966c:	1000002f 	b	8000972c <_vfprintf_r+0x964>
80009670:	00000000 	nop
80009674:	8fc20050 	lw	v0,80(s8)
80009678:	14400004 	bnez	v0,8000968c <_vfprintf_r+0x8c4>
8000967c:	00000000 	nop
80009680:	32220001 	andi	v0,s1,0x1
80009684:	10400007 	beqz	v0,800096a4 <_vfprintf_r+0x8dc>
80009688:	00000000 	nop
8000968c:	8fc30050 	lw	v1,80(s8)
80009690:	8fc2008c 	lw	v0,140(s8)
80009694:	00621021 	addu	v0,v1,v0
80009698:	24420001 	addiu	v0,v0,1
8000969c:	10000002 	b	800096a8 <_vfprintf_r+0x8e0>
800096a0:	00000000 	nop
800096a4:	24020001 	li	v0,1
800096a8:	afc20074 	sw	v0,116(s8)
800096ac:	1000001f 	b	8000972c <_vfprintf_r+0x964>
800096b0:	00000000 	nop
800096b4:	8fc300b0 	lw	v1,176(s8)
800096b8:	8fc200c0 	lw	v0,192(s8)
800096bc:	0062102a 	slt	v0,v1,v0
800096c0:	1440000c 	bnez	v0,800096f4 <_vfprintf_r+0x92c>
800096c4:	00000000 	nop
800096c8:	8fc200b0 	lw	v0,176(s8)
800096cc:	afc20074 	sw	v0,116(s8)
800096d0:	32220001 	andi	v0,s1,0x1
800096d4:	10400015 	beqz	v0,8000972c <_vfprintf_r+0x964>
800096d8:	00000000 	nop
800096dc:	8fc30074 	lw	v1,116(s8)
800096e0:	8fc2008c 	lw	v0,140(s8)
800096e4:	00621021 	addu	v0,v1,v0
800096e8:	afc20074 	sw	v0,116(s8)
800096ec:	1000000f 	b	8000972c <_vfprintf_r+0x964>
800096f0:	00000000 	nop
800096f4:	8fc200c0 	lw	v0,192(s8)
800096f8:	00401825 	move	v1,v0
800096fc:	8fc2008c 	lw	v0,140(s8)
80009700:	00621021 	addu	v0,v1,v0
80009704:	afc20074 	sw	v0,116(s8)
80009708:	8fc200b0 	lw	v0,176(s8)
8000970c:	1c400007 	bgtz	v0,8000972c <_vfprintf_r+0x964>
80009710:	00000000 	nop
80009714:	8fc200b0 	lw	v0,176(s8)
80009718:	24030001 	li	v1,1
8000971c:	00621023 	subu	v0,v1,v0
80009720:	8fc30074 	lw	v1,116(s8)
80009724:	00621021 	addu	v0,v1,v0
80009728:	afc20074 	sw	v0,116(s8)
8000972c:	8fc200b0 	lw	v0,176(s8)
80009730:	afc20058 	sw	v0,88(s8)
80009734:	83c200a1 	lb	v0,161(s8)
80009738:	104001bc 	beqz	v0,80009e2c <_vfprintf_r+0x1064>
8000973c:	00000000 	nop
80009740:	2402002d 	li	v0,45
80009744:	a3c200a0 	sb	v0,160(s8)
80009748:	100001b8 	b	80009e2c <_vfprintf_r+0x1064>
8000974c:	00000000 	nop
80009750:	32220020 	andi	v0,s1,0x20
80009754:	1040000a 	beqz	v0,80009780 <_vfprintf_r+0x9b8>
80009758:	00000000 	nop
8000975c:	8fc201a0 	lw	v0,416(s8)
80009760:	24430008 	addiu	v1,v0,8
80009764:	afc301a0 	sw	v1,416(s8)
80009768:	24420004 	addiu	v0,v0,4
8000976c:	8c420000 	lw	v0,0(v0)
80009770:	8fc30048 	lw	v1,72(s8)
80009774:	fc430000 	sd	v1,0(v0)
80009778:	1000061e 	b	8000aff4 <_vfprintf_r+0x222c>
8000977c:	00000000 	nop
80009780:	32220010 	andi	v0,s1,0x10
80009784:	1040000a 	beqz	v0,800097b0 <_vfprintf_r+0x9e8>
80009788:	00000000 	nop
8000978c:	8fc201a0 	lw	v0,416(s8)
80009790:	24430008 	addiu	v1,v0,8
80009794:	afc301a0 	sw	v1,416(s8)
80009798:	24420004 	addiu	v0,v0,4
8000979c:	8c420000 	lw	v0,0(v0)
800097a0:	8fc30048 	lw	v1,72(s8)
800097a4:	ac430000 	sw	v1,0(v0)
800097a8:	10000612 	b	8000aff4 <_vfprintf_r+0x222c>
800097ac:	00000000 	nop
800097b0:	32220040 	andi	v0,s1,0x40
800097b4:	1040000c 	beqz	v0,800097e8 <_vfprintf_r+0xa20>
800097b8:	00000000 	nop
800097bc:	8fc201a0 	lw	v0,416(s8)
800097c0:	24430008 	addiu	v1,v0,8
800097c4:	afc301a0 	sw	v1,416(s8)
800097c8:	24420004 	addiu	v0,v0,4
800097cc:	8c420000 	lw	v0,0(v0)
800097d0:	8fc30048 	lw	v1,72(s8)
800097d4:	00031c00 	sll	v1,v1,0x10
800097d8:	00031c03 	sra	v1,v1,0x10
800097dc:	a4430000 	sh	v1,0(v0)
800097e0:	10000604 	b	8000aff4 <_vfprintf_r+0x222c>
800097e4:	00000000 	nop
800097e8:	8fc201a0 	lw	v0,416(s8)
800097ec:	24430008 	addiu	v1,v0,8
800097f0:	afc301a0 	sw	v1,416(s8)
800097f4:	24420004 	addiu	v0,v0,4
800097f8:	8c420000 	lw	v0,0(v0)
800097fc:	8fc30048 	lw	v1,72(s8)
80009800:	ac430000 	sw	v1,0(v0)
80009804:	100005fb 	b	8000aff4 <_vfprintf_r+0x222c>
80009808:	00000000 	nop
8000980c:	36310010 	ori	s1,s1,0x10
80009810:	32220020 	andi	v0,s1,0x20
80009814:	10400007 	beqz	v0,80009834 <_vfprintf_r+0xa6c>
80009818:	00000000 	nop
8000981c:	8fc201a0 	lw	v0,416(s8)
80009820:	24430008 	addiu	v1,v0,8
80009824:	afc301a0 	sw	v1,416(s8)
80009828:	dc420000 	ld	v0,0(v0)
8000982c:	10000020 	b	800098b0 <_vfprintf_r+0xae8>
80009830:	00000000 	nop
80009834:	32220010 	andi	v0,s1,0x10
80009838:	1040000a 	beqz	v0,80009864 <_vfprintf_r+0xa9c>
8000983c:	00000000 	nop
80009840:	8fc201a0 	lw	v0,416(s8)
80009844:	24430008 	addiu	v1,v0,8
80009848:	afc301a0 	sw	v1,416(s8)
8000984c:	24420004 	addiu	v0,v0,4
80009850:	8c420000 	lw	v0,0(v0)
80009854:	0002103c 	dsll32	v0,v0,0x0
80009858:	0002103e 	dsrl32	v0,v0,0x0
8000985c:	10000014 	b	800098b0 <_vfprintf_r+0xae8>
80009860:	00000000 	nop
80009864:	32220040 	andi	v0,s1,0x40
80009868:	1040000a 	beqz	v0,80009894 <_vfprintf_r+0xacc>
8000986c:	00000000 	nop
80009870:	8fc201a0 	lw	v0,416(s8)
80009874:	24430008 	addiu	v1,v0,8
80009878:	afc301a0 	sw	v1,416(s8)
8000987c:	24420004 	addiu	v0,v0,4
80009880:	8c420000 	lw	v0,0(v0)
80009884:	3042ffff 	andi	v0,v0,0xffff
80009888:	3042ffff 	andi	v0,v0,0xffff
8000988c:	10000008 	b	800098b0 <_vfprintf_r+0xae8>
80009890:	00000000 	nop
80009894:	8fc201a0 	lw	v0,416(s8)
80009898:	24430008 	addiu	v1,v0,8
8000989c:	afc301a0 	sw	v1,416(s8)
800098a0:	24420004 	addiu	v0,v0,4
800098a4:	8c420000 	lw	v0,0(v0)
800098a8:	0002103c 	dsll32	v0,v0,0x0
800098ac:	0002103e 	dsrl32	v0,v0,0x0
800098b0:	ffc20060 	sd	v0,96(s8)
800098b4:	afc00068 	sw	zero,104(s8)
800098b8:	100000b4 	b	80009b8c <_vfprintf_r+0xdc4>
800098bc:	00000000 	nop
800098c0:	8fc201a0 	lw	v0,416(s8)
800098c4:	24430008 	addiu	v1,v0,8
800098c8:	afc301a0 	sw	v1,416(s8)
800098cc:	24420004 	addiu	v0,v0,4
800098d0:	8c420000 	lw	v0,0(v0)
800098d4:	0002103c 	dsll32	v0,v0,0x0
800098d8:	0002103e 	dsrl32	v0,v0,0x0
800098dc:	ffc20060 	sd	v0,96(s8)
800098e0:	24020002 	li	v0,2
800098e4:	afc20068 	sw	v0,104(s8)
800098e8:	3c028002 	lui	v0,0x8002
800098ec:	2442a510 	addiu	v0,v0,-23280
800098f0:	afc20078 	sw	v0,120(s8)
800098f4:	36310002 	ori	s1,s1,0x2
800098f8:	24020030 	li	v0,48
800098fc:	a3c20140 	sb	v0,320(s8)
80009900:	24140078 	li	s4,120
80009904:	24020078 	li	v0,120
80009908:	a3c20141 	sb	v0,321(s8)
8000990c:	1000009f 	b	80009b8c <_vfprintf_r+0xdc4>
80009910:	00000000 	nop
80009914:	8fc201a0 	lw	v0,416(s8)
80009918:	24430008 	addiu	v1,v0,8
8000991c:	afc301a0 	sw	v1,416(s8)
80009920:	24420004 	addiu	v0,v0,4
80009924:	8c520000 	lw	s2,0(v0)
80009928:	a3c000a0 	sb	zero,160(s8)
8000992c:	1640000b 	bnez	s2,8000995c <_vfprintf_r+0xb94>
80009930:	00000000 	nop
80009934:	3c028002 	lui	v0,0x8002
80009938:	2452a528 	addiu	s2,v0,-23256
8000993c:	8fc20050 	lw	v0,80(s8)
80009940:	2c430007 	sltiu	v1,v0,7
80009944:	14600002 	bnez	v1,80009950 <_vfprintf_r+0xb88>
80009948:	00000000 	nop
8000994c:	24020006 	li	v0,6
80009950:	afc20074 	sw	v0,116(s8)
80009954:	10000136 	b	80009e30 <_vfprintf_r+0x1068>
80009958:	00000000 	nop
8000995c:	8fc20050 	lw	v0,80(s8)
80009960:	04400014 	bltz	v0,800099b4 <_vfprintf_r+0xbec>
80009964:	00000000 	nop
80009968:	8fc20050 	lw	v0,80(s8)
8000996c:	00403025 	move	a2,v0
80009970:	00002825 	move	a1,zero
80009974:	02402025 	move	a0,s2
80009978:	0c003adc 	jal	8000eb70 <memchr>
8000997c:	00000000 	nop
80009980:	afc20098 	sw	v0,152(s8)
80009984:	8fc20098 	lw	v0,152(s8)
80009988:	10400006 	beqz	v0,800099a4 <_vfprintf_r+0xbdc>
8000998c:	00000000 	nop
80009990:	8fc20098 	lw	v0,152(s8)
80009994:	00521023 	subu	v0,v0,s2
80009998:	afc20074 	sw	v0,116(s8)
8000999c:	10000124 	b	80009e30 <_vfprintf_r+0x1068>
800099a0:	00000000 	nop
800099a4:	8fc20050 	lw	v0,80(s8)
800099a8:	afc20074 	sw	v0,116(s8)
800099ac:	10000120 	b	80009e30 <_vfprintf_r+0x1068>
800099b0:	00000000 	nop
800099b4:	02402025 	move	a0,s2
800099b8:	0c001f1d 	jal	80007c74 <strlen>
800099bc:	00000000 	nop
800099c0:	afc20074 	sw	v0,116(s8)
800099c4:	1000011a 	b	80009e30 <_vfprintf_r+0x1068>
800099c8:	00000000 	nop
800099cc:	36310010 	ori	s1,s1,0x10
800099d0:	32220020 	andi	v0,s1,0x20
800099d4:	10400007 	beqz	v0,800099f4 <_vfprintf_r+0xc2c>
800099d8:	00000000 	nop
800099dc:	8fc201a0 	lw	v0,416(s8)
800099e0:	24430008 	addiu	v1,v0,8
800099e4:	afc301a0 	sw	v1,416(s8)
800099e8:	dc420000 	ld	v0,0(v0)
800099ec:	10000020 	b	80009a70 <_vfprintf_r+0xca8>
800099f0:	00000000 	nop
800099f4:	32220010 	andi	v0,s1,0x10
800099f8:	1040000a 	beqz	v0,80009a24 <_vfprintf_r+0xc5c>
800099fc:	00000000 	nop
80009a00:	8fc201a0 	lw	v0,416(s8)
80009a04:	24430008 	addiu	v1,v0,8
80009a08:	afc301a0 	sw	v1,416(s8)
80009a0c:	24420004 	addiu	v0,v0,4
80009a10:	8c420000 	lw	v0,0(v0)
80009a14:	0002103c 	dsll32	v0,v0,0x0
80009a18:	0002103e 	dsrl32	v0,v0,0x0
80009a1c:	10000014 	b	80009a70 <_vfprintf_r+0xca8>
80009a20:	00000000 	nop
80009a24:	32220040 	andi	v0,s1,0x40
80009a28:	1040000a 	beqz	v0,80009a54 <_vfprintf_r+0xc8c>
80009a2c:	00000000 	nop
80009a30:	8fc201a0 	lw	v0,416(s8)
80009a34:	24430008 	addiu	v1,v0,8
80009a38:	afc301a0 	sw	v1,416(s8)
80009a3c:	24420004 	addiu	v0,v0,4
80009a40:	8c420000 	lw	v0,0(v0)
80009a44:	3042ffff 	andi	v0,v0,0xffff
80009a48:	3042ffff 	andi	v0,v0,0xffff
80009a4c:	10000008 	b	80009a70 <_vfprintf_r+0xca8>
80009a50:	00000000 	nop
80009a54:	8fc201a0 	lw	v0,416(s8)
80009a58:	24430008 	addiu	v1,v0,8
80009a5c:	afc301a0 	sw	v1,416(s8)
80009a60:	24420004 	addiu	v0,v0,4
80009a64:	8c420000 	lw	v0,0(v0)
80009a68:	0002103c 	dsll32	v0,v0,0x0
80009a6c:	0002103e 	dsrl32	v0,v0,0x0
80009a70:	ffc20060 	sd	v0,96(s8)
80009a74:	24020001 	li	v0,1
80009a78:	afc20068 	sw	v0,104(s8)
80009a7c:	10000043 	b	80009b8c <_vfprintf_r+0xdc4>
80009a80:	00000000 	nop
80009a84:	3c028002 	lui	v0,0x8002
80009a88:	2442a530 	addiu	v0,v0,-23248
80009a8c:	afc20078 	sw	v0,120(s8)
80009a90:	10000004 	b	80009aa4 <_vfprintf_r+0xcdc>
80009a94:	00000000 	nop
80009a98:	3c028002 	lui	v0,0x8002
80009a9c:	2442a510 	addiu	v0,v0,-23280
80009aa0:	afc20078 	sw	v0,120(s8)
80009aa4:	32220020 	andi	v0,s1,0x20
80009aa8:	10400007 	beqz	v0,80009ac8 <_vfprintf_r+0xd00>
80009aac:	00000000 	nop
80009ab0:	8fc201a0 	lw	v0,416(s8)
80009ab4:	24430008 	addiu	v1,v0,8
80009ab8:	afc301a0 	sw	v1,416(s8)
80009abc:	dc420000 	ld	v0,0(v0)
80009ac0:	10000020 	b	80009b44 <_vfprintf_r+0xd7c>
80009ac4:	00000000 	nop
80009ac8:	32220010 	andi	v0,s1,0x10
80009acc:	1040000a 	beqz	v0,80009af8 <_vfprintf_r+0xd30>
80009ad0:	00000000 	nop
80009ad4:	8fc201a0 	lw	v0,416(s8)
80009ad8:	24430008 	addiu	v1,v0,8
80009adc:	afc301a0 	sw	v1,416(s8)
80009ae0:	24420004 	addiu	v0,v0,4
80009ae4:	8c420000 	lw	v0,0(v0)
80009ae8:	0002103c 	dsll32	v0,v0,0x0
80009aec:	0002103e 	dsrl32	v0,v0,0x0
80009af0:	10000014 	b	80009b44 <_vfprintf_r+0xd7c>
80009af4:	00000000 	nop
80009af8:	32220040 	andi	v0,s1,0x40
80009afc:	1040000a 	beqz	v0,80009b28 <_vfprintf_r+0xd60>
80009b00:	00000000 	nop
80009b04:	8fc201a0 	lw	v0,416(s8)
80009b08:	24430008 	addiu	v1,v0,8
80009b0c:	afc301a0 	sw	v1,416(s8)
80009b10:	24420004 	addiu	v0,v0,4
80009b14:	8c420000 	lw	v0,0(v0)
80009b18:	3042ffff 	andi	v0,v0,0xffff
80009b1c:	3042ffff 	andi	v0,v0,0xffff
80009b20:	10000008 	b	80009b44 <_vfprintf_r+0xd7c>
80009b24:	00000000 	nop
80009b28:	8fc201a0 	lw	v0,416(s8)
80009b2c:	24430008 	addiu	v1,v0,8
80009b30:	afc301a0 	sw	v1,416(s8)
80009b34:	24420004 	addiu	v0,v0,4
80009b38:	8c420000 	lw	v0,0(v0)
80009b3c:	0002103c 	dsll32	v0,v0,0x0
80009b40:	0002103e 	dsrl32	v0,v0,0x0
80009b44:	ffc20060 	sd	v0,96(s8)
80009b48:	24020002 	li	v0,2
80009b4c:	afc20068 	sw	v0,104(s8)
80009b50:	32220001 	andi	v0,s1,0x1
80009b54:	1040000c 	beqz	v0,80009b88 <_vfprintf_r+0xdc0>
80009b58:	00000000 	nop
80009b5c:	dfc20060 	ld	v0,96(s8)
80009b60:	10400009 	beqz	v0,80009b88 <_vfprintf_r+0xdc0>
80009b64:	00000000 	nop
80009b68:	24020030 	li	v0,48
80009b6c:	a3c20140 	sb	v0,320(s8)
80009b70:	00141600 	sll	v0,s4,0x18
80009b74:	00021603 	sra	v0,v0,0x18
80009b78:	a3c20141 	sb	v0,321(s8)
80009b7c:	36310002 	ori	s1,s1,0x2
80009b80:	10000002 	b	80009b8c <_vfprintf_r+0xdc4>
80009b84:	00000000 	nop
80009b88:	00000000 	nop
80009b8c:	a3c000a0 	sb	zero,160(s8)
80009b90:	8fc20050 	lw	v0,80(s8)
80009b94:	afc2006c 	sw	v0,108(s8)
80009b98:	8fc2006c 	lw	v0,108(s8)
80009b9c:	04400003 	bltz	v0,80009bac <_vfprintf_r+0xde4>
80009ba0:	00000000 	nop
80009ba4:	2402ff7f 	li	v0,-129
80009ba8:	02228824 	and	s1,s1,v0
80009bac:	27d20118 	addiu	s2,s8,280
80009bb0:	26520028 	addiu	s2,s2,40
80009bb4:	dfc20060 	ld	v0,96(s8)
80009bb8:	14400004 	bnez	v0,80009bcc <_vfprintf_r+0xe04>
80009bbc:	00000000 	nop
80009bc0:	8fc20050 	lw	v0,80(s8)
80009bc4:	1040007f 	beqz	v0,80009dc4 <_vfprintf_r+0xffc>
80009bc8:	00000000 	nop
80009bcc:	8fc30068 	lw	v1,104(s8)
80009bd0:	24020002 	li	v0,2
80009bd4:	1062005f 	beq	v1,v0,80009d54 <_vfprintf_r+0xf8c>
80009bd8:	00000000 	nop
80009bdc:	8fc20068 	lw	v0,104(s8)
80009be0:	2c420003 	sltiu	v0,v0,3
80009be4:	1040006b 	beqz	v0,80009d94 <_vfprintf_r+0xfcc>
80009be8:	00000000 	nop
80009bec:	8fc20068 	lw	v0,104(s8)
80009bf0:	10400008 	beqz	v0,80009c14 <_vfprintf_r+0xe4c>
80009bf4:	00000000 	nop
80009bf8:	8fc30068 	lw	v1,104(s8)
80009bfc:	24020001 	li	v0,1
80009c00:	1062001f 	beq	v1,v0,80009c80 <_vfprintf_r+0xeb8>
80009c04:	00000000 	nop
80009c08:	10000062 	b	80009d94 <_vfprintf_r+0xfcc>
80009c0c:	00000000 	nop
80009c10:	00000000 	nop
80009c14:	93c20067 	lbu	v0,103(s8)
80009c18:	30420007 	andi	v0,v0,0x7
80009c1c:	304200ff 	andi	v0,v0,0xff
80009c20:	24420030 	addiu	v0,v0,48
80009c24:	304200ff 	andi	v0,v0,0xff
80009c28:	2652ffff 	addiu	s2,s2,-1
80009c2c:	00021600 	sll	v0,v0,0x18
80009c30:	00021603 	sra	v0,v0,0x18
80009c34:	a2420000 	sb	v0,0(s2)
80009c38:	dfc20060 	ld	v0,96(s8)
80009c3c:	000210fa 	dsrl	v0,v0,0x3
80009c40:	ffc20060 	sd	v0,96(s8)
80009c44:	dfc20060 	ld	v0,96(s8)
80009c48:	1440fff1 	bnez	v0,80009c10 <_vfprintf_r+0xe48>
80009c4c:	00000000 	nop
80009c50:	32220001 	andi	v0,s1,0x1
80009c54:	10400058 	beqz	v0,80009db8 <_vfprintf_r+0xff0>
80009c58:	00000000 	nop
80009c5c:	82430000 	lb	v1,0(s2)
80009c60:	24020030 	li	v0,48
80009c64:	10620054 	beq	v1,v0,80009db8 <_vfprintf_r+0xff0>
80009c68:	00000000 	nop
80009c6c:	2652ffff 	addiu	s2,s2,-1
80009c70:	24020030 	li	v0,48
80009c74:	a2420000 	sb	v0,0(s2)
80009c78:	1000004f 	b	80009db8 <_vfprintf_r+0xff0>
80009c7c:	00000000 	nop
80009c80:	dfc20060 	ld	v0,96(s8)
80009c84:	2c42000a 	sltiu	v0,v0,10
80009c88:	1040000a 	beqz	v0,80009cb4 <_vfprintf_r+0xeec>
80009c8c:	00000000 	nop
80009c90:	93c20067 	lbu	v0,103(s8)
80009c94:	24420030 	addiu	v0,v0,48
80009c98:	304200ff 	andi	v0,v0,0xff
80009c9c:	2652ffff 	addiu	s2,s2,-1
80009ca0:	00021600 	sll	v0,v0,0x18
80009ca4:	00021603 	sra	v0,v0,0x18
80009ca8:	a2420000 	sb	v0,0(s2)
80009cac:	10000043 	b	80009dbc <_vfprintf_r+0xff4>
80009cb0:	00000000 	nop
80009cb4:	dfc40060 	ld	a0,96(s8)
80009cb8:	3c02f333 	lui	v0,0xf333
80009cbc:	34423333 	ori	v0,v0,0x3333
80009cc0:	00021438 	dsll	v0,v0,0x10
80009cc4:	34423333 	ori	v0,v0,0x3333
80009cc8:	000214b8 	dsll	v0,v0,0x12
80009ccc:	3442cccd 	ori	v0,v0,0xcccd
80009cd0:	0082001d 	dmultu	a0,v0
80009cd4:	00001010 	mfhi	v0
80009cd8:	000218fa 	dsrl	v1,v0,0x3
80009cdc:	00601025 	move	v0,v1
80009ce0:	000210b8 	dsll	v0,v0,0x2
80009ce4:	0043102d 	daddu	v0,v0,v1
80009ce8:	00021078 	dsll	v0,v0,0x1
80009cec:	0082182f 	dsubu	v1,a0,v0
80009cf0:	00031000 	sll	v0,v1,0x0
80009cf4:	304200ff 	andi	v0,v0,0xff
80009cf8:	24420030 	addiu	v0,v0,48
80009cfc:	304200ff 	andi	v0,v0,0xff
80009d00:	2652ffff 	addiu	s2,s2,-1
80009d04:	00021600 	sll	v0,v0,0x18
80009d08:	00021603 	sra	v0,v0,0x18
80009d0c:	a2420000 	sb	v0,0(s2)
80009d10:	dfc30060 	ld	v1,96(s8)
80009d14:	3c02f333 	lui	v0,0xf333
80009d18:	34423333 	ori	v0,v0,0x3333
80009d1c:	00021438 	dsll	v0,v0,0x10
80009d20:	34423333 	ori	v0,v0,0x3333
80009d24:	000214b8 	dsll	v0,v0,0x12
80009d28:	3442cccd 	ori	v0,v0,0xcccd
80009d2c:	0062001d 	dmultu	v1,v0
80009d30:	00001010 	mfhi	v0
80009d34:	000210fa 	dsrl	v0,v0,0x3
80009d38:	ffc20060 	sd	v0,96(s8)
80009d3c:	dfc20060 	ld	v0,96(s8)
80009d40:	1440ffdc 	bnez	v0,80009cb4 <_vfprintf_r+0xeec>
80009d44:	00000000 	nop
80009d48:	1000001c 	b	80009dbc <_vfprintf_r+0xff4>
80009d4c:	00000000 	nop
80009d50:	00000000 	nop
80009d54:	dfc20060 	ld	v0,96(s8)
80009d58:	00021000 	sll	v0,v0,0x0
80009d5c:	3042000f 	andi	v0,v0,0xf
80009d60:	8fc30078 	lw	v1,120(s8)
80009d64:	00621021 	addu	v0,v1,v0
80009d68:	2652ffff 	addiu	s2,s2,-1
80009d6c:	80420000 	lb	v0,0(v0)
80009d70:	a2420000 	sb	v0,0(s2)
80009d74:	dfc20060 	ld	v0,96(s8)
80009d78:	0002113a 	dsrl	v0,v0,0x4
80009d7c:	ffc20060 	sd	v0,96(s8)
80009d80:	dfc20060 	ld	v0,96(s8)
80009d84:	1440fff2 	bnez	v0,80009d50 <_vfprintf_r+0xf88>
80009d88:	00000000 	nop
80009d8c:	1000000b 	b	80009dbc <_vfprintf_r+0xff4>
80009d90:	00000000 	nop
80009d94:	3c028002 	lui	v0,0x8002
80009d98:	2452a548 	addiu	s2,v0,-23224
80009d9c:	02402025 	move	a0,s2
80009da0:	0c001f1d 	jal	80007c74 <strlen>
80009da4:	00000000 	nop
80009da8:	afc20074 	sw	v0,116(s8)
80009dac:	00000000 	nop
80009db0:	1000001f 	b	80009e30 <_vfprintf_r+0x1068>
80009db4:	00000000 	nop
80009db8:	00000000 	nop
80009dbc:	1000000a 	b	80009de8 <_vfprintf_r+0x1020>
80009dc0:	00000000 	nop
80009dc4:	8fc20068 	lw	v0,104(s8)
80009dc8:	14400007 	bnez	v0,80009de8 <_vfprintf_r+0x1020>
80009dcc:	00000000 	nop
80009dd0:	32220001 	andi	v0,s1,0x1
80009dd4:	10400004 	beqz	v0,80009de8 <_vfprintf_r+0x1020>
80009dd8:	00000000 	nop
80009ddc:	2652ffff 	addiu	s2,s2,-1
80009de0:	24020030 	li	v0,48
80009de4:	a2420000 	sb	v0,0(s2)
80009de8:	27c20118 	addiu	v0,s8,280
80009dec:	24420028 	addiu	v0,v0,40
80009df0:	00521023 	subu	v0,v0,s2
80009df4:	afc20074 	sw	v0,116(s8)
80009df8:	1000000d 	b	80009e30 <_vfprintf_r+0x1068>
80009dfc:	00000000 	nop
80009e00:	12800481 	beqz	s4,8000b008 <_vfprintf_r+0x2240>
80009e04:	00000000 	nop
80009e08:	27d20118 	addiu	s2,s8,280
80009e0c:	00141600 	sll	v0,s4,0x18
80009e10:	00021603 	sra	v0,v0,0x18
80009e14:	a2420000 	sb	v0,0(s2)
80009e18:	24020001 	li	v0,1
80009e1c:	afc20074 	sw	v0,116(s8)
80009e20:	a3c000a0 	sb	zero,160(s8)
80009e24:	10000002 	b	80009e30 <_vfprintf_r+0x1068>
80009e28:	00000000 	nop
80009e2c:	00000000 	nop
80009e30:	8fc3006c 	lw	v1,108(s8)
80009e34:	8fc20074 	lw	v0,116(s8)
80009e38:	0043202a 	slt	a0,v0,v1
80009e3c:	10800002 	beqz	a0,80009e48 <_vfprintf_r+0x1080>
80009e40:	00000000 	nop
80009e44:	00601025 	move	v0,v1
80009e48:	afc20070 	sw	v0,112(s8)
80009e4c:	83c200a0 	lb	v0,160(s8)
80009e50:	10400004 	beqz	v0,80009e64 <_vfprintf_r+0x109c>
80009e54:	00000000 	nop
80009e58:	8fc20070 	lw	v0,112(s8)
80009e5c:	24420001 	addiu	v0,v0,1
80009e60:	afc20070 	sw	v0,112(s8)
80009e64:	32220002 	andi	v0,s1,0x2
80009e68:	10400004 	beqz	v0,80009e7c <_vfprintf_r+0x10b4>
80009e6c:	00000000 	nop
80009e70:	8fc20070 	lw	v0,112(s8)
80009e74:	24420002 	addiu	v0,v0,2
80009e78:	afc20070 	sw	v0,112(s8)
80009e7c:	32220084 	andi	v0,s1,0x84
80009e80:	1440003f 	bnez	v0,80009f80 <_vfprintf_r+0x11b8>
80009e84:	00000000 	nop
80009e88:	8fc3004c 	lw	v1,76(s8)
80009e8c:	8fc20070 	lw	v0,112(s8)
80009e90:	00629823 	subu	s3,v1,v0
80009e94:	1a60003a 	blez	s3,80009f80 <_vfprintf_r+0x11b8>
80009e98:	00000000 	nop
80009e9c:	1000001b 	b	80009f0c <_vfprintf_r+0x1144>
80009ea0:	00000000 	nop
80009ea4:	3c028002 	lui	v0,0x8002
80009ea8:	2442a6d0 	addiu	v0,v0,-22832
80009eac:	ae020000 	sw	v0,0(s0)
80009eb0:	24020010 	li	v0,16
80009eb4:	ae020004 	sw	v0,4(s0)
80009eb8:	8fc200d0 	lw	v0,208(s8)
80009ebc:	24420010 	addiu	v0,v0,16
80009ec0:	afc200d0 	sw	v0,208(s8)
80009ec4:	26100008 	addiu	s0,s0,8
80009ec8:	8fc200cc 	lw	v0,204(s8)
80009ecc:	24420001 	addiu	v0,v0,1
80009ed0:	afc200cc 	sw	v0,204(s8)
80009ed4:	8fc200cc 	lw	v0,204(s8)
80009ed8:	28420008 	slti	v0,v0,8
80009edc:	1440000a 	bnez	v0,80009f08 <_vfprintf_r+0x1140>
80009ee0:	00000000 	nop
80009ee4:	27c200c8 	addiu	v0,s8,200
80009ee8:	00403025 	move	a2,v0
80009eec:	8fc50190 	lw	a1,400(s8)
80009ef0:	8fc40188 	lw	a0,392(s8)
80009ef4:	0c00512c 	jal	800144b0 <__sprint_r>
80009ef8:	00000000 	nop
80009efc:	14400454 	bnez	v0,8000b050 <_vfprintf_r+0x2288>
80009f00:	00000000 	nop
80009f04:	27d000d8 	addiu	s0,s8,216
80009f08:	2673fff0 	addiu	s3,s3,-16
80009f0c:	2a620011 	slti	v0,s3,17
80009f10:	1040ffe4 	beqz	v0,80009ea4 <_vfprintf_r+0x10dc>
80009f14:	00000000 	nop
80009f18:	3c028002 	lui	v0,0x8002
80009f1c:	2442a6d0 	addiu	v0,v0,-22832
80009f20:	ae020000 	sw	v0,0(s0)
80009f24:	02601025 	move	v0,s3
80009f28:	ae020004 	sw	v0,4(s0)
80009f2c:	8fc200d0 	lw	v0,208(s8)
80009f30:	02601825 	move	v1,s3
80009f34:	00431021 	addu	v0,v0,v1
80009f38:	afc200d0 	sw	v0,208(s8)
80009f3c:	26100008 	addiu	s0,s0,8
80009f40:	8fc200cc 	lw	v0,204(s8)
80009f44:	24420001 	addiu	v0,v0,1
80009f48:	afc200cc 	sw	v0,204(s8)
80009f4c:	8fc200cc 	lw	v0,204(s8)
80009f50:	28420008 	slti	v0,v0,8
80009f54:	1440000a 	bnez	v0,80009f80 <_vfprintf_r+0x11b8>
80009f58:	00000000 	nop
80009f5c:	27c200c8 	addiu	v0,s8,200
80009f60:	00403025 	move	a2,v0
80009f64:	8fc50190 	lw	a1,400(s8)
80009f68:	8fc40188 	lw	a0,392(s8)
80009f6c:	0c00512c 	jal	800144b0 <__sprint_r>
80009f70:	00000000 	nop
80009f74:	14400439 	bnez	v0,8000b05c <_vfprintf_r+0x2294>
80009f78:	00000000 	nop
80009f7c:	27d000d8 	addiu	s0,s8,216
80009f80:	83c200a0 	lb	v0,160(s8)
80009f84:	10400019 	beqz	v0,80009fec <_vfprintf_r+0x1224>
80009f88:	00000000 	nop
80009f8c:	27c200a0 	addiu	v0,s8,160
80009f90:	ae020000 	sw	v0,0(s0)
80009f94:	24020001 	li	v0,1
80009f98:	ae020004 	sw	v0,4(s0)
80009f9c:	8fc200d0 	lw	v0,208(s8)
80009fa0:	24420001 	addiu	v0,v0,1
80009fa4:	afc200d0 	sw	v0,208(s8)
80009fa8:	26100008 	addiu	s0,s0,8
80009fac:	8fc200cc 	lw	v0,204(s8)
80009fb0:	24420001 	addiu	v0,v0,1
80009fb4:	afc200cc 	sw	v0,204(s8)
80009fb8:	8fc200cc 	lw	v0,204(s8)
80009fbc:	28420008 	slti	v0,v0,8
80009fc0:	1440000a 	bnez	v0,80009fec <_vfprintf_r+0x1224>
80009fc4:	00000000 	nop
80009fc8:	27c200c8 	addiu	v0,s8,200
80009fcc:	00403025 	move	a2,v0
80009fd0:	8fc50190 	lw	a1,400(s8)
80009fd4:	8fc40188 	lw	a0,392(s8)
80009fd8:	0c00512c 	jal	800144b0 <__sprint_r>
80009fdc:	00000000 	nop
80009fe0:	14400421 	bnez	v0,8000b068 <_vfprintf_r+0x22a0>
80009fe4:	00000000 	nop
80009fe8:	27d000d8 	addiu	s0,s8,216
80009fec:	32220002 	andi	v0,s1,0x2
80009ff0:	10400019 	beqz	v0,8000a058 <_vfprintf_r+0x1290>
80009ff4:	00000000 	nop
80009ff8:	27c20140 	addiu	v0,s8,320
80009ffc:	ae020000 	sw	v0,0(s0)
8000a000:	24020002 	li	v0,2
8000a004:	ae020004 	sw	v0,4(s0)
8000a008:	8fc200d0 	lw	v0,208(s8)
8000a00c:	24420002 	addiu	v0,v0,2
8000a010:	afc200d0 	sw	v0,208(s8)
8000a014:	26100008 	addiu	s0,s0,8
8000a018:	8fc200cc 	lw	v0,204(s8)
8000a01c:	24420001 	addiu	v0,v0,1
8000a020:	afc200cc 	sw	v0,204(s8)
8000a024:	8fc200cc 	lw	v0,204(s8)
8000a028:	28420008 	slti	v0,v0,8
8000a02c:	1440000a 	bnez	v0,8000a058 <_vfprintf_r+0x1290>
8000a030:	00000000 	nop
8000a034:	27c200c8 	addiu	v0,s8,200
8000a038:	00403025 	move	a2,v0
8000a03c:	8fc50190 	lw	a1,400(s8)
8000a040:	8fc40188 	lw	a0,392(s8)
8000a044:	0c00512c 	jal	800144b0 <__sprint_r>
8000a048:	00000000 	nop
8000a04c:	14400409 	bnez	v0,8000b074 <_vfprintf_r+0x22ac>
8000a050:	00000000 	nop
8000a054:	27d000d8 	addiu	s0,s8,216
8000a058:	32230084 	andi	v1,s1,0x84
8000a05c:	24020080 	li	v0,128
8000a060:	1462003f 	bne	v1,v0,8000a160 <_vfprintf_r+0x1398>
8000a064:	00000000 	nop
8000a068:	8fc3004c 	lw	v1,76(s8)
8000a06c:	8fc20070 	lw	v0,112(s8)
8000a070:	00629823 	subu	s3,v1,v0
8000a074:	1a60003a 	blez	s3,8000a160 <_vfprintf_r+0x1398>
8000a078:	00000000 	nop
8000a07c:	1000001b 	b	8000a0ec <_vfprintf_r+0x1324>
8000a080:	00000000 	nop
8000a084:	3c028002 	lui	v0,0x8002
8000a088:	2442a6e0 	addiu	v0,v0,-22816
8000a08c:	ae020000 	sw	v0,0(s0)
8000a090:	24020010 	li	v0,16
8000a094:	ae020004 	sw	v0,4(s0)
8000a098:	8fc200d0 	lw	v0,208(s8)
8000a09c:	24420010 	addiu	v0,v0,16
8000a0a0:	afc200d0 	sw	v0,208(s8)
8000a0a4:	26100008 	addiu	s0,s0,8
8000a0a8:	8fc200cc 	lw	v0,204(s8)
8000a0ac:	24420001 	addiu	v0,v0,1
8000a0b0:	afc200cc 	sw	v0,204(s8)
8000a0b4:	8fc200cc 	lw	v0,204(s8)
8000a0b8:	28420008 	slti	v0,v0,8
8000a0bc:	1440000a 	bnez	v0,8000a0e8 <_vfprintf_r+0x1320>
8000a0c0:	00000000 	nop
8000a0c4:	27c200c8 	addiu	v0,s8,200
8000a0c8:	00403025 	move	a2,v0
8000a0cc:	8fc50190 	lw	a1,400(s8)
8000a0d0:	8fc40188 	lw	a0,392(s8)
8000a0d4:	0c00512c 	jal	800144b0 <__sprint_r>
8000a0d8:	00000000 	nop
8000a0dc:	144003e8 	bnez	v0,8000b080 <_vfprintf_r+0x22b8>
8000a0e0:	00000000 	nop
8000a0e4:	27d000d8 	addiu	s0,s8,216
8000a0e8:	2673fff0 	addiu	s3,s3,-16
8000a0ec:	2a620011 	slti	v0,s3,17
8000a0f0:	1040ffe4 	beqz	v0,8000a084 <_vfprintf_r+0x12bc>
8000a0f4:	00000000 	nop
8000a0f8:	3c028002 	lui	v0,0x8002
8000a0fc:	2442a6e0 	addiu	v0,v0,-22816
8000a100:	ae020000 	sw	v0,0(s0)
8000a104:	02601025 	move	v0,s3
8000a108:	ae020004 	sw	v0,4(s0)
8000a10c:	8fc200d0 	lw	v0,208(s8)
8000a110:	02601825 	move	v1,s3
8000a114:	00431021 	addu	v0,v0,v1
8000a118:	afc200d0 	sw	v0,208(s8)
8000a11c:	26100008 	addiu	s0,s0,8
8000a120:	8fc200cc 	lw	v0,204(s8)
8000a124:	24420001 	addiu	v0,v0,1
8000a128:	afc200cc 	sw	v0,204(s8)
8000a12c:	8fc200cc 	lw	v0,204(s8)
8000a130:	28420008 	slti	v0,v0,8
8000a134:	1440000a 	bnez	v0,8000a160 <_vfprintf_r+0x1398>
8000a138:	00000000 	nop
8000a13c:	27c200c8 	addiu	v0,s8,200
8000a140:	00403025 	move	a2,v0
8000a144:	8fc50190 	lw	a1,400(s8)
8000a148:	8fc40188 	lw	a0,392(s8)
8000a14c:	0c00512c 	jal	800144b0 <__sprint_r>
8000a150:	00000000 	nop
8000a154:	144003cd 	bnez	v0,8000b08c <_vfprintf_r+0x22c4>
8000a158:	00000000 	nop
8000a15c:	27d000d8 	addiu	s0,s8,216
8000a160:	8fc3006c 	lw	v1,108(s8)
8000a164:	8fc20074 	lw	v0,116(s8)
8000a168:	00629823 	subu	s3,v1,v0
8000a16c:	1a60003a 	blez	s3,8000a258 <_vfprintf_r+0x1490>
8000a170:	00000000 	nop
8000a174:	1000001b 	b	8000a1e4 <_vfprintf_r+0x141c>
8000a178:	00000000 	nop
8000a17c:	3c028002 	lui	v0,0x8002
8000a180:	2442a6e0 	addiu	v0,v0,-22816
8000a184:	ae020000 	sw	v0,0(s0)
8000a188:	24020010 	li	v0,16
8000a18c:	ae020004 	sw	v0,4(s0)
8000a190:	8fc200d0 	lw	v0,208(s8)
8000a194:	24420010 	addiu	v0,v0,16
8000a198:	afc200d0 	sw	v0,208(s8)
8000a19c:	26100008 	addiu	s0,s0,8
8000a1a0:	8fc200cc 	lw	v0,204(s8)
8000a1a4:	24420001 	addiu	v0,v0,1
8000a1a8:	afc200cc 	sw	v0,204(s8)
8000a1ac:	8fc200cc 	lw	v0,204(s8)
8000a1b0:	28420008 	slti	v0,v0,8
8000a1b4:	1440000a 	bnez	v0,8000a1e0 <_vfprintf_r+0x1418>
8000a1b8:	00000000 	nop
8000a1bc:	27c200c8 	addiu	v0,s8,200
8000a1c0:	00403025 	move	a2,v0
8000a1c4:	8fc50190 	lw	a1,400(s8)
8000a1c8:	8fc40188 	lw	a0,392(s8)
8000a1cc:	0c00512c 	jal	800144b0 <__sprint_r>
8000a1d0:	00000000 	nop
8000a1d4:	144003b0 	bnez	v0,8000b098 <_vfprintf_r+0x22d0>
8000a1d8:	00000000 	nop
8000a1dc:	27d000d8 	addiu	s0,s8,216
8000a1e0:	2673fff0 	addiu	s3,s3,-16
8000a1e4:	2a620011 	slti	v0,s3,17
8000a1e8:	1040ffe4 	beqz	v0,8000a17c <_vfprintf_r+0x13b4>
8000a1ec:	00000000 	nop
8000a1f0:	3c028002 	lui	v0,0x8002
8000a1f4:	2442a6e0 	addiu	v0,v0,-22816
8000a1f8:	ae020000 	sw	v0,0(s0)
8000a1fc:	02601025 	move	v0,s3
8000a200:	ae020004 	sw	v0,4(s0)
8000a204:	8fc200d0 	lw	v0,208(s8)
8000a208:	02601825 	move	v1,s3
8000a20c:	00431021 	addu	v0,v0,v1
8000a210:	afc200d0 	sw	v0,208(s8)
8000a214:	26100008 	addiu	s0,s0,8
8000a218:	8fc200cc 	lw	v0,204(s8)
8000a21c:	24420001 	addiu	v0,v0,1
8000a220:	afc200cc 	sw	v0,204(s8)
8000a224:	8fc200cc 	lw	v0,204(s8)
8000a228:	28420008 	slti	v0,v0,8
8000a22c:	1440000a 	bnez	v0,8000a258 <_vfprintf_r+0x1490>
8000a230:	00000000 	nop
8000a234:	27c200c8 	addiu	v0,s8,200
8000a238:	00403025 	move	a2,v0
8000a23c:	8fc50190 	lw	a1,400(s8)
8000a240:	8fc40188 	lw	a0,392(s8)
8000a244:	0c00512c 	jal	800144b0 <__sprint_r>
8000a248:	00000000 	nop
8000a24c:	14400395 	bnez	v0,8000b0a4 <_vfprintf_r+0x22dc>
8000a250:	00000000 	nop
8000a254:	27d000d8 	addiu	s0,s8,216
8000a258:	32220100 	andi	v0,s1,0x100
8000a25c:	1440001b 	bnez	v0,8000a2cc <_vfprintf_r+0x1504>
8000a260:	00000000 	nop
8000a264:	ae120000 	sw	s2,0(s0)
8000a268:	8fc20074 	lw	v0,116(s8)
8000a26c:	ae020004 	sw	v0,4(s0)
8000a270:	8fc300d0 	lw	v1,208(s8)
8000a274:	8fc20074 	lw	v0,116(s8)
8000a278:	00621021 	addu	v0,v1,v0
8000a27c:	afc200d0 	sw	v0,208(s8)
8000a280:	26100008 	addiu	s0,s0,8
8000a284:	8fc200cc 	lw	v0,204(s8)
8000a288:	24420001 	addiu	v0,v0,1
8000a28c:	afc200cc 	sw	v0,204(s8)
8000a290:	8fc200cc 	lw	v0,204(s8)
8000a294:	28420008 	slti	v0,v0,8
8000a298:	144002f9 	bnez	v0,8000ae80 <_vfprintf_r+0x20b8>
8000a29c:	00000000 	nop
8000a2a0:	27c200c8 	addiu	v0,s8,200
8000a2a4:	00403025 	move	a2,v0
8000a2a8:	8fc50190 	lw	a1,400(s8)
8000a2ac:	8fc40188 	lw	a0,392(s8)
8000a2b0:	0c00512c 	jal	800144b0 <__sprint_r>
8000a2b4:	00000000 	nop
8000a2b8:	1440037d 	bnez	v0,8000b0b0 <_vfprintf_r+0x22e8>
8000a2bc:	00000000 	nop
8000a2c0:	27d000d8 	addiu	s0,s8,216
8000a2c4:	100002ee 	b	8000ae80 <_vfprintf_r+0x20b8>
8000a2c8:	00000000 	nop
8000a2cc:	2a820066 	slti	v0,s4,102
8000a2d0:	1440021e 	bnez	v0,8000ab4c <_vfprintf_r+0x1d84>
8000a2d4:	00000000 	nop
8000a2d8:	d7c000a8 	ldc1	$f0,168(s8)
8000a2dc:	44a00800 	dmtc1	zero,$f1
8000a2e0:	00000000 	nop
8000a2e4:	46210032 	c.eq.d	$f0,$f1
8000a2e8:	00000000 	nop
8000a2ec:	4500007a 	bc1f	8000a4d8 <_vfprintf_r+0x1710>
8000a2f0:	00000000 	nop
8000a2f4:	3c028002 	lui	v0,0x8002
8000a2f8:	2442a568 	addiu	v0,v0,-23192
8000a2fc:	ae020000 	sw	v0,0(s0)
8000a300:	24020001 	li	v0,1
8000a304:	ae020004 	sw	v0,4(s0)
8000a308:	8fc200d0 	lw	v0,208(s8)
8000a30c:	24420001 	addiu	v0,v0,1
8000a310:	afc200d0 	sw	v0,208(s8)
8000a314:	26100008 	addiu	s0,s0,8
8000a318:	8fc200cc 	lw	v0,204(s8)
8000a31c:	24420001 	addiu	v0,v0,1
8000a320:	afc200cc 	sw	v0,204(s8)
8000a324:	8fc200cc 	lw	v0,204(s8)
8000a328:	28420008 	slti	v0,v0,8
8000a32c:	1440000a 	bnez	v0,8000a358 <_vfprintf_r+0x1590>
8000a330:	00000000 	nop
8000a334:	27c200c8 	addiu	v0,s8,200
8000a338:	00403025 	move	a2,v0
8000a33c:	8fc50190 	lw	a1,400(s8)
8000a340:	8fc40188 	lw	a0,392(s8)
8000a344:	0c00512c 	jal	800144b0 <__sprint_r>
8000a348:	00000000 	nop
8000a34c:	1440035b 	bnez	v0,8000b0bc <_vfprintf_r+0x22f4>
8000a350:	00000000 	nop
8000a354:	27d000d8 	addiu	s0,s8,216
8000a358:	8fc300b0 	lw	v1,176(s8)
8000a35c:	8fc200c0 	lw	v0,192(s8)
8000a360:	0062102a 	slt	v0,v1,v0
8000a364:	14400004 	bnez	v0,8000a378 <_vfprintf_r+0x15b0>
8000a368:	00000000 	nop
8000a36c:	32220001 	andi	v0,s1,0x1
8000a370:	104002c3 	beqz	v0,8000ae80 <_vfprintf_r+0x20b8>
8000a374:	00000000 	nop
8000a378:	8fc20088 	lw	v0,136(s8)
8000a37c:	ae020000 	sw	v0,0(s0)
8000a380:	8fc2008c 	lw	v0,140(s8)
8000a384:	ae020004 	sw	v0,4(s0)
8000a388:	8fc300d0 	lw	v1,208(s8)
8000a38c:	8fc2008c 	lw	v0,140(s8)
8000a390:	00621021 	addu	v0,v1,v0
8000a394:	afc200d0 	sw	v0,208(s8)
8000a398:	26100008 	addiu	s0,s0,8
8000a39c:	8fc200cc 	lw	v0,204(s8)
8000a3a0:	24420001 	addiu	v0,v0,1
8000a3a4:	afc200cc 	sw	v0,204(s8)
8000a3a8:	8fc200cc 	lw	v0,204(s8)
8000a3ac:	28420008 	slti	v0,v0,8
8000a3b0:	1440000a 	bnez	v0,8000a3dc <_vfprintf_r+0x1614>
8000a3b4:	00000000 	nop
8000a3b8:	27c200c8 	addiu	v0,s8,200
8000a3bc:	00403025 	move	a2,v0
8000a3c0:	8fc50190 	lw	a1,400(s8)
8000a3c4:	8fc40188 	lw	a0,392(s8)
8000a3c8:	0c00512c 	jal	800144b0 <__sprint_r>
8000a3cc:	00000000 	nop
8000a3d0:	1440033d 	bnez	v0,8000b0c8 <_vfprintf_r+0x2300>
8000a3d4:	00000000 	nop
8000a3d8:	27d000d8 	addiu	s0,s8,216
8000a3dc:	8fc200c0 	lw	v0,192(s8)
8000a3e0:	2453ffff 	addiu	s3,v0,-1
8000a3e4:	1a6002a6 	blez	s3,8000ae80 <_vfprintf_r+0x20b8>
8000a3e8:	00000000 	nop
8000a3ec:	1000001b 	b	8000a45c <_vfprintf_r+0x1694>
8000a3f0:	00000000 	nop
8000a3f4:	3c028002 	lui	v0,0x8002
8000a3f8:	2442a6e0 	addiu	v0,v0,-22816
8000a3fc:	ae020000 	sw	v0,0(s0)
8000a400:	24020010 	li	v0,16
8000a404:	ae020004 	sw	v0,4(s0)
8000a408:	8fc200d0 	lw	v0,208(s8)
8000a40c:	24420010 	addiu	v0,v0,16
8000a410:	afc200d0 	sw	v0,208(s8)
8000a414:	26100008 	addiu	s0,s0,8
8000a418:	8fc200cc 	lw	v0,204(s8)
8000a41c:	24420001 	addiu	v0,v0,1
8000a420:	afc200cc 	sw	v0,204(s8)
8000a424:	8fc200cc 	lw	v0,204(s8)
8000a428:	28420008 	slti	v0,v0,8
8000a42c:	1440000a 	bnez	v0,8000a458 <_vfprintf_r+0x1690>
8000a430:	00000000 	nop
8000a434:	27c200c8 	addiu	v0,s8,200
8000a438:	00403025 	move	a2,v0
8000a43c:	8fc50190 	lw	a1,400(s8)
8000a440:	8fc40188 	lw	a0,392(s8)
8000a444:	0c00512c 	jal	800144b0 <__sprint_r>
8000a448:	00000000 	nop
8000a44c:	14400321 	bnez	v0,8000b0d4 <_vfprintf_r+0x230c>
8000a450:	00000000 	nop
8000a454:	27d000d8 	addiu	s0,s8,216
8000a458:	2673fff0 	addiu	s3,s3,-16
8000a45c:	2a620011 	slti	v0,s3,17
8000a460:	1040ffe4 	beqz	v0,8000a3f4 <_vfprintf_r+0x162c>
8000a464:	00000000 	nop
8000a468:	3c028002 	lui	v0,0x8002
8000a46c:	2442a6e0 	addiu	v0,v0,-22816
8000a470:	ae020000 	sw	v0,0(s0)
8000a474:	02601025 	move	v0,s3
8000a478:	ae020004 	sw	v0,4(s0)
8000a47c:	8fc200d0 	lw	v0,208(s8)
8000a480:	02601825 	move	v1,s3
8000a484:	00431021 	addu	v0,v0,v1
8000a488:	afc200d0 	sw	v0,208(s8)
8000a48c:	26100008 	addiu	s0,s0,8
8000a490:	8fc200cc 	lw	v0,204(s8)
8000a494:	24420001 	addiu	v0,v0,1
8000a498:	afc200cc 	sw	v0,204(s8)
8000a49c:	8fc200cc 	lw	v0,204(s8)
8000a4a0:	28420008 	slti	v0,v0,8
8000a4a4:	14400276 	bnez	v0,8000ae80 <_vfprintf_r+0x20b8>
8000a4a8:	00000000 	nop
8000a4ac:	27c200c8 	addiu	v0,s8,200
8000a4b0:	00403025 	move	a2,v0
8000a4b4:	8fc50190 	lw	a1,400(s8)
8000a4b8:	8fc40188 	lw	a0,392(s8)
8000a4bc:	0c00512c 	jal	800144b0 <__sprint_r>
8000a4c0:	00000000 	nop
8000a4c4:	14400306 	bnez	v0,8000b0e0 <_vfprintf_r+0x2318>
8000a4c8:	00000000 	nop
8000a4cc:	27d000d8 	addiu	s0,s8,216
8000a4d0:	1000026b 	b	8000ae80 <_vfprintf_r+0x20b8>
8000a4d4:	00000000 	nop
8000a4d8:	8fc200b0 	lw	v0,176(s8)
8000a4dc:	1c400093 	bgtz	v0,8000a72c <_vfprintf_r+0x1964>
8000a4e0:	00000000 	nop
8000a4e4:	3c028002 	lui	v0,0x8002
8000a4e8:	2442a568 	addiu	v0,v0,-23192
8000a4ec:	ae020000 	sw	v0,0(s0)
8000a4f0:	24020001 	li	v0,1
8000a4f4:	ae020004 	sw	v0,4(s0)
8000a4f8:	8fc200d0 	lw	v0,208(s8)
8000a4fc:	24420001 	addiu	v0,v0,1
8000a500:	afc200d0 	sw	v0,208(s8)
8000a504:	26100008 	addiu	s0,s0,8
8000a508:	8fc200cc 	lw	v0,204(s8)
8000a50c:	24420001 	addiu	v0,v0,1
8000a510:	afc200cc 	sw	v0,204(s8)
8000a514:	8fc200cc 	lw	v0,204(s8)
8000a518:	28420008 	slti	v0,v0,8
8000a51c:	1440000a 	bnez	v0,8000a548 <_vfprintf_r+0x1780>
8000a520:	00000000 	nop
8000a524:	27c200c8 	addiu	v0,s8,200
8000a528:	00403025 	move	a2,v0
8000a52c:	8fc50190 	lw	a1,400(s8)
8000a530:	8fc40188 	lw	a0,392(s8)
8000a534:	0c00512c 	jal	800144b0 <__sprint_r>
8000a538:	00000000 	nop
8000a53c:	144002eb 	bnez	v0,8000b0ec <_vfprintf_r+0x2324>
8000a540:	00000000 	nop
8000a544:	27d000d8 	addiu	s0,s8,216
8000a548:	8fc200b0 	lw	v0,176(s8)
8000a54c:	14400007 	bnez	v0,8000a56c <_vfprintf_r+0x17a4>
8000a550:	00000000 	nop
8000a554:	8fc200c0 	lw	v0,192(s8)
8000a558:	14400004 	bnez	v0,8000a56c <_vfprintf_r+0x17a4>
8000a55c:	00000000 	nop
8000a560:	32220001 	andi	v0,s1,0x1
8000a564:	10400246 	beqz	v0,8000ae80 <_vfprintf_r+0x20b8>
8000a568:	00000000 	nop
8000a56c:	8fc20088 	lw	v0,136(s8)
8000a570:	ae020000 	sw	v0,0(s0)
8000a574:	8fc2008c 	lw	v0,140(s8)
8000a578:	ae020004 	sw	v0,4(s0)
8000a57c:	8fc300d0 	lw	v1,208(s8)
8000a580:	8fc2008c 	lw	v0,140(s8)
8000a584:	00621021 	addu	v0,v1,v0
8000a588:	afc200d0 	sw	v0,208(s8)
8000a58c:	26100008 	addiu	s0,s0,8
8000a590:	8fc200cc 	lw	v0,204(s8)
8000a594:	24420001 	addiu	v0,v0,1
8000a598:	afc200cc 	sw	v0,204(s8)
8000a59c:	8fc200cc 	lw	v0,204(s8)
8000a5a0:	28420008 	slti	v0,v0,8
8000a5a4:	1440000a 	bnez	v0,8000a5d0 <_vfprintf_r+0x1808>
8000a5a8:	00000000 	nop
8000a5ac:	27c200c8 	addiu	v0,s8,200
8000a5b0:	00403025 	move	a2,v0
8000a5b4:	8fc50190 	lw	a1,400(s8)
8000a5b8:	8fc40188 	lw	a0,392(s8)
8000a5bc:	0c00512c 	jal	800144b0 <__sprint_r>
8000a5c0:	00000000 	nop
8000a5c4:	144002cc 	bnez	v0,8000b0f8 <_vfprintf_r+0x2330>
8000a5c8:	00000000 	nop
8000a5cc:	27d000d8 	addiu	s0,s8,216
8000a5d0:	8fc200b0 	lw	v0,176(s8)
8000a5d4:	00029823 	negu	s3,v0
8000a5d8:	1a60003a 	blez	s3,8000a6c4 <_vfprintf_r+0x18fc>
8000a5dc:	00000000 	nop
8000a5e0:	1000001b 	b	8000a650 <_vfprintf_r+0x1888>
8000a5e4:	00000000 	nop
8000a5e8:	3c028002 	lui	v0,0x8002
8000a5ec:	2442a6e0 	addiu	v0,v0,-22816
8000a5f0:	ae020000 	sw	v0,0(s0)
8000a5f4:	24020010 	li	v0,16
8000a5f8:	ae020004 	sw	v0,4(s0)
8000a5fc:	8fc200d0 	lw	v0,208(s8)
8000a600:	24420010 	addiu	v0,v0,16
8000a604:	afc200d0 	sw	v0,208(s8)
8000a608:	26100008 	addiu	s0,s0,8
8000a60c:	8fc200cc 	lw	v0,204(s8)
8000a610:	24420001 	addiu	v0,v0,1
8000a614:	afc200cc 	sw	v0,204(s8)
8000a618:	8fc200cc 	lw	v0,204(s8)
8000a61c:	28420008 	slti	v0,v0,8
8000a620:	1440000a 	bnez	v0,8000a64c <_vfprintf_r+0x1884>
8000a624:	00000000 	nop
8000a628:	27c200c8 	addiu	v0,s8,200
8000a62c:	00403025 	move	a2,v0
8000a630:	8fc50190 	lw	a1,400(s8)
8000a634:	8fc40188 	lw	a0,392(s8)
8000a638:	0c00512c 	jal	800144b0 <__sprint_r>
8000a63c:	00000000 	nop
8000a640:	144002b0 	bnez	v0,8000b104 <_vfprintf_r+0x233c>
8000a644:	00000000 	nop
8000a648:	27d000d8 	addiu	s0,s8,216
8000a64c:	2673fff0 	addiu	s3,s3,-16
8000a650:	2a620011 	slti	v0,s3,17
8000a654:	1040ffe4 	beqz	v0,8000a5e8 <_vfprintf_r+0x1820>
8000a658:	00000000 	nop
8000a65c:	3c028002 	lui	v0,0x8002
8000a660:	2442a6e0 	addiu	v0,v0,-22816
8000a664:	ae020000 	sw	v0,0(s0)
8000a668:	02601025 	move	v0,s3
8000a66c:	ae020004 	sw	v0,4(s0)
8000a670:	8fc200d0 	lw	v0,208(s8)
8000a674:	02601825 	move	v1,s3
8000a678:	00431021 	addu	v0,v0,v1
8000a67c:	afc200d0 	sw	v0,208(s8)
8000a680:	26100008 	addiu	s0,s0,8
8000a684:	8fc200cc 	lw	v0,204(s8)
8000a688:	24420001 	addiu	v0,v0,1
8000a68c:	afc200cc 	sw	v0,204(s8)
8000a690:	8fc200cc 	lw	v0,204(s8)
8000a694:	28420008 	slti	v0,v0,8
8000a698:	1440000a 	bnez	v0,8000a6c4 <_vfprintf_r+0x18fc>
8000a69c:	00000000 	nop
8000a6a0:	27c200c8 	addiu	v0,s8,200
8000a6a4:	00403025 	move	a2,v0
8000a6a8:	8fc50190 	lw	a1,400(s8)
8000a6ac:	8fc40188 	lw	a0,392(s8)
8000a6b0:	0c00512c 	jal	800144b0 <__sprint_r>
8000a6b4:	00000000 	nop
8000a6b8:	14400295 	bnez	v0,8000b110 <_vfprintf_r+0x2348>
8000a6bc:	00000000 	nop
8000a6c0:	27d000d8 	addiu	s0,s8,216
8000a6c4:	ae120000 	sw	s2,0(s0)
8000a6c8:	8fc200c0 	lw	v0,192(s8)
8000a6cc:	ae020004 	sw	v0,4(s0)
8000a6d0:	8fc200d0 	lw	v0,208(s8)
8000a6d4:	8fc300c0 	lw	v1,192(s8)
8000a6d8:	00431021 	addu	v0,v0,v1
8000a6dc:	afc200d0 	sw	v0,208(s8)
8000a6e0:	26100008 	addiu	s0,s0,8
8000a6e4:	8fc200cc 	lw	v0,204(s8)
8000a6e8:	24420001 	addiu	v0,v0,1
8000a6ec:	afc200cc 	sw	v0,204(s8)
8000a6f0:	8fc200cc 	lw	v0,204(s8)
8000a6f4:	28420008 	slti	v0,v0,8
8000a6f8:	144001e1 	bnez	v0,8000ae80 <_vfprintf_r+0x20b8>
8000a6fc:	00000000 	nop
8000a700:	27c200c8 	addiu	v0,s8,200
8000a704:	00403025 	move	a2,v0
8000a708:	8fc50190 	lw	a1,400(s8)
8000a70c:	8fc40188 	lw	a0,392(s8)
8000a710:	0c00512c 	jal	800144b0 <__sprint_r>
8000a714:	00000000 	nop
8000a718:	14400280 	bnez	v0,8000b11c <_vfprintf_r+0x2354>
8000a71c:	00000000 	nop
8000a720:	27d000d8 	addiu	s0,s8,216
8000a724:	100001d6 	b	8000ae80 <_vfprintf_r+0x20b8>
8000a728:	00000000 	nop
8000a72c:	afd2009c 	sw	s2,156(s8)
8000a730:	8fc200c0 	lw	v0,192(s8)
8000a734:	00401825 	move	v1,v0
8000a738:	8fc2009c 	lw	v0,156(s8)
8000a73c:	00431021 	addu	v0,v0,v1
8000a740:	00521023 	subu	v0,v0,s2
8000a744:	afc20080 	sw	v0,128(s8)
8000a748:	8fc30080 	lw	v1,128(s8)
8000a74c:	8fc20058 	lw	v0,88(s8)
8000a750:	0043102a 	slt	v0,v0,v1
8000a754:	10400003 	beqz	v0,8000a764 <_vfprintf_r+0x199c>
8000a758:	00000000 	nop
8000a75c:	8fc20058 	lw	v0,88(s8)
8000a760:	afc20080 	sw	v0,128(s8)
8000a764:	8fc20080 	lw	v0,128(s8)
8000a768:	18400019 	blez	v0,8000a7d0 <_vfprintf_r+0x1a08>
8000a76c:	00000000 	nop
8000a770:	ae120000 	sw	s2,0(s0)
8000a774:	8fc20080 	lw	v0,128(s8)
8000a778:	ae020004 	sw	v0,4(s0)
8000a77c:	8fc300d0 	lw	v1,208(s8)
8000a780:	8fc20080 	lw	v0,128(s8)
8000a784:	00621021 	addu	v0,v1,v0
8000a788:	afc200d0 	sw	v0,208(s8)
8000a78c:	26100008 	addiu	s0,s0,8
8000a790:	8fc200cc 	lw	v0,204(s8)
8000a794:	24420001 	addiu	v0,v0,1
8000a798:	afc200cc 	sw	v0,204(s8)
8000a79c:	8fc200cc 	lw	v0,204(s8)
8000a7a0:	28420008 	slti	v0,v0,8
8000a7a4:	1440000a 	bnez	v0,8000a7d0 <_vfprintf_r+0x1a08>
8000a7a8:	00000000 	nop
8000a7ac:	27c200c8 	addiu	v0,s8,200
8000a7b0:	00403025 	move	a2,v0
8000a7b4:	8fc50190 	lw	a1,400(s8)
8000a7b8:	8fc40188 	lw	a0,392(s8)
8000a7bc:	0c00512c 	jal	800144b0 <__sprint_r>
8000a7c0:	00000000 	nop
8000a7c4:	14400258 	bnez	v0,8000b128 <_vfprintf_r+0x2360>
8000a7c8:	00000000 	nop
8000a7cc:	27d000d8 	addiu	s0,s8,216
8000a7d0:	8fc20080 	lw	v0,128(s8)
8000a7d4:	04410002 	bgez	v0,8000a7e0 <_vfprintf_r+0x1a18>
8000a7d8:	00000000 	nop
8000a7dc:	00001025 	move	v0,zero
8000a7e0:	8fc30058 	lw	v1,88(s8)
8000a7e4:	00621023 	subu	v0,v1,v0
8000a7e8:	afc20080 	sw	v0,128(s8)
8000a7ec:	8fc20080 	lw	v0,128(s8)
8000a7f0:	1840003d 	blez	v0,8000a8e8 <_vfprintf_r+0x1b20>
8000a7f4:	00000000 	nop
8000a7f8:	1000001d 	b	8000a870 <_vfprintf_r+0x1aa8>
8000a7fc:	00000000 	nop
8000a800:	3c028002 	lui	v0,0x8002
8000a804:	2442a6e0 	addiu	v0,v0,-22816
8000a808:	ae020000 	sw	v0,0(s0)
8000a80c:	24020010 	li	v0,16
8000a810:	ae020004 	sw	v0,4(s0)
8000a814:	8fc200d0 	lw	v0,208(s8)
8000a818:	24420010 	addiu	v0,v0,16
8000a81c:	afc200d0 	sw	v0,208(s8)
8000a820:	26100008 	addiu	s0,s0,8
8000a824:	8fc200cc 	lw	v0,204(s8)
8000a828:	24420001 	addiu	v0,v0,1
8000a82c:	afc200cc 	sw	v0,204(s8)
8000a830:	8fc200cc 	lw	v0,204(s8)
8000a834:	28420008 	slti	v0,v0,8
8000a838:	1440000a 	bnez	v0,8000a864 <_vfprintf_r+0x1a9c>
8000a83c:	00000000 	nop
8000a840:	27c200c8 	addiu	v0,s8,200
8000a844:	00403025 	move	a2,v0
8000a848:	8fc50190 	lw	a1,400(s8)
8000a84c:	8fc40188 	lw	a0,392(s8)
8000a850:	0c00512c 	jal	800144b0 <__sprint_r>
8000a854:	00000000 	nop
8000a858:	14400236 	bnez	v0,8000b134 <_vfprintf_r+0x236c>
8000a85c:	00000000 	nop
8000a860:	27d000d8 	addiu	s0,s8,216
8000a864:	8fc20080 	lw	v0,128(s8)
8000a868:	2442fff0 	addiu	v0,v0,-16
8000a86c:	afc20080 	sw	v0,128(s8)
8000a870:	8fc20080 	lw	v0,128(s8)
8000a874:	28420011 	slti	v0,v0,17
8000a878:	1040ffe1 	beqz	v0,8000a800 <_vfprintf_r+0x1a38>
8000a87c:	00000000 	nop
8000a880:	3c028002 	lui	v0,0x8002
8000a884:	2442a6e0 	addiu	v0,v0,-22816
8000a888:	ae020000 	sw	v0,0(s0)
8000a88c:	8fc20080 	lw	v0,128(s8)
8000a890:	ae020004 	sw	v0,4(s0)
8000a894:	8fc300d0 	lw	v1,208(s8)
8000a898:	8fc20080 	lw	v0,128(s8)
8000a89c:	00621021 	addu	v0,v1,v0
8000a8a0:	afc200d0 	sw	v0,208(s8)
8000a8a4:	26100008 	addiu	s0,s0,8
8000a8a8:	8fc200cc 	lw	v0,204(s8)
8000a8ac:	24420001 	addiu	v0,v0,1
8000a8b0:	afc200cc 	sw	v0,204(s8)
8000a8b4:	8fc200cc 	lw	v0,204(s8)
8000a8b8:	28420008 	slti	v0,v0,8
8000a8bc:	1440000a 	bnez	v0,8000a8e8 <_vfprintf_r+0x1b20>
8000a8c0:	00000000 	nop
8000a8c4:	27c200c8 	addiu	v0,s8,200
8000a8c8:	00403025 	move	a2,v0
8000a8cc:	8fc50190 	lw	a1,400(s8)
8000a8d0:	8fc40188 	lw	a0,392(s8)
8000a8d4:	0c00512c 	jal	800144b0 <__sprint_r>
8000a8d8:	00000000 	nop
8000a8dc:	14400218 	bnez	v0,8000b140 <_vfprintf_r+0x2378>
8000a8e0:	00000000 	nop
8000a8e4:	27d000d8 	addiu	s0,s8,216
8000a8e8:	8fc20058 	lw	v0,88(s8)
8000a8ec:	02429021 	addu	s2,s2,v0
8000a8f0:	8fc300b0 	lw	v1,176(s8)
8000a8f4:	8fc200c0 	lw	v0,192(s8)
8000a8f8:	0062102a 	slt	v0,v1,v0
8000a8fc:	14400004 	bnez	v0,8000a910 <_vfprintf_r+0x1b48>
8000a900:	00000000 	nop
8000a904:	32220001 	andi	v0,s1,0x1
8000a908:	1040001a 	beqz	v0,8000a974 <_vfprintf_r+0x1bac>
8000a90c:	00000000 	nop
8000a910:	8fc20088 	lw	v0,136(s8)
8000a914:	ae020000 	sw	v0,0(s0)
8000a918:	8fc2008c 	lw	v0,140(s8)
8000a91c:	ae020004 	sw	v0,4(s0)
8000a920:	8fc300d0 	lw	v1,208(s8)
8000a924:	8fc2008c 	lw	v0,140(s8)
8000a928:	00621021 	addu	v0,v1,v0
8000a92c:	afc200d0 	sw	v0,208(s8)
8000a930:	26100008 	addiu	s0,s0,8
8000a934:	8fc200cc 	lw	v0,204(s8)
8000a938:	24420001 	addiu	v0,v0,1
8000a93c:	afc200cc 	sw	v0,204(s8)
8000a940:	8fc200cc 	lw	v0,204(s8)
8000a944:	28420008 	slti	v0,v0,8
8000a948:	1440000a 	bnez	v0,8000a974 <_vfprintf_r+0x1bac>
8000a94c:	00000000 	nop
8000a950:	27c200c8 	addiu	v0,s8,200
8000a954:	00403025 	move	a2,v0
8000a958:	8fc50190 	lw	a1,400(s8)
8000a95c:	8fc40188 	lw	a0,392(s8)
8000a960:	0c00512c 	jal	800144b0 <__sprint_r>
8000a964:	00000000 	nop
8000a968:	144001f8 	bnez	v0,8000b14c <_vfprintf_r+0x2384>
8000a96c:	00000000 	nop
8000a970:	27d000d8 	addiu	s0,s8,216
8000a974:	8fc200c0 	lw	v0,192(s8)
8000a978:	00401825 	move	v1,v0
8000a97c:	8fc2009c 	lw	v0,156(s8)
8000a980:	00431021 	addu	v0,v0,v1
8000a984:	00521023 	subu	v0,v0,s2
8000a988:	afc20084 	sw	v0,132(s8)
8000a98c:	8fc300c0 	lw	v1,192(s8)
8000a990:	8fc200b0 	lw	v0,176(s8)
8000a994:	00621023 	subu	v0,v1,v0
8000a998:	8fc30084 	lw	v1,132(s8)
8000a99c:	0043102a 	slt	v0,v0,v1
8000a9a0:	10400005 	beqz	v0,8000a9b8 <_vfprintf_r+0x1bf0>
8000a9a4:	00000000 	nop
8000a9a8:	8fc300c0 	lw	v1,192(s8)
8000a9ac:	8fc200b0 	lw	v0,176(s8)
8000a9b0:	00621023 	subu	v0,v1,v0
8000a9b4:	afc20084 	sw	v0,132(s8)
8000a9b8:	8fc20084 	lw	v0,132(s8)
8000a9bc:	18400019 	blez	v0,8000aa24 <_vfprintf_r+0x1c5c>
8000a9c0:	00000000 	nop
8000a9c4:	ae120000 	sw	s2,0(s0)
8000a9c8:	8fc20084 	lw	v0,132(s8)
8000a9cc:	ae020004 	sw	v0,4(s0)
8000a9d0:	8fc300d0 	lw	v1,208(s8)
8000a9d4:	8fc20084 	lw	v0,132(s8)
8000a9d8:	00621021 	addu	v0,v1,v0
8000a9dc:	afc200d0 	sw	v0,208(s8)
8000a9e0:	26100008 	addiu	s0,s0,8
8000a9e4:	8fc200cc 	lw	v0,204(s8)
8000a9e8:	24420001 	addiu	v0,v0,1
8000a9ec:	afc200cc 	sw	v0,204(s8)
8000a9f0:	8fc200cc 	lw	v0,204(s8)
8000a9f4:	28420008 	slti	v0,v0,8
8000a9f8:	1440000a 	bnez	v0,8000aa24 <_vfprintf_r+0x1c5c>
8000a9fc:	00000000 	nop
8000aa00:	27c200c8 	addiu	v0,s8,200
8000aa04:	00403025 	move	a2,v0
8000aa08:	8fc50190 	lw	a1,400(s8)
8000aa0c:	8fc40188 	lw	a0,392(s8)
8000aa10:	0c00512c 	jal	800144b0 <__sprint_r>
8000aa14:	00000000 	nop
8000aa18:	144001cf 	bnez	v0,8000b158 <_vfprintf_r+0x2390>
8000aa1c:	00000000 	nop
8000aa20:	27d000d8 	addiu	s0,s8,216
8000aa24:	8fc300c0 	lw	v1,192(s8)
8000aa28:	8fc200b0 	lw	v0,176(s8)
8000aa2c:	00621823 	subu	v1,v1,v0
8000aa30:	8fc20084 	lw	v0,132(s8)
8000aa34:	04410002 	bgez	v0,8000aa40 <_vfprintf_r+0x1c78>
8000aa38:	00000000 	nop
8000aa3c:	00001025 	move	v0,zero
8000aa40:	00621023 	subu	v0,v1,v0
8000aa44:	afc20084 	sw	v0,132(s8)
8000aa48:	8fc20084 	lw	v0,132(s8)
8000aa4c:	1840010c 	blez	v0,8000ae80 <_vfprintf_r+0x20b8>
8000aa50:	00000000 	nop
8000aa54:	1000001d 	b	8000aacc <_vfprintf_r+0x1d04>
8000aa58:	00000000 	nop
8000aa5c:	3c028002 	lui	v0,0x8002
8000aa60:	2442a6e0 	addiu	v0,v0,-22816
8000aa64:	ae020000 	sw	v0,0(s0)
8000aa68:	24020010 	li	v0,16
8000aa6c:	ae020004 	sw	v0,4(s0)
8000aa70:	8fc200d0 	lw	v0,208(s8)
8000aa74:	24420010 	addiu	v0,v0,16
8000aa78:	afc200d0 	sw	v0,208(s8)
8000aa7c:	26100008 	addiu	s0,s0,8
8000aa80:	8fc200cc 	lw	v0,204(s8)
8000aa84:	24420001 	addiu	v0,v0,1
8000aa88:	afc200cc 	sw	v0,204(s8)
8000aa8c:	8fc200cc 	lw	v0,204(s8)
8000aa90:	28420008 	slti	v0,v0,8
8000aa94:	1440000a 	bnez	v0,8000aac0 <_vfprintf_r+0x1cf8>
8000aa98:	00000000 	nop
8000aa9c:	27c200c8 	addiu	v0,s8,200
8000aaa0:	00403025 	move	a2,v0
8000aaa4:	8fc50190 	lw	a1,400(s8)
8000aaa8:	8fc40188 	lw	a0,392(s8)
8000aaac:	0c00512c 	jal	800144b0 <__sprint_r>
8000aab0:	00000000 	nop
8000aab4:	144001ab 	bnez	v0,8000b164 <_vfprintf_r+0x239c>
8000aab8:	00000000 	nop
8000aabc:	27d000d8 	addiu	s0,s8,216
8000aac0:	8fc20084 	lw	v0,132(s8)
8000aac4:	2442fff0 	addiu	v0,v0,-16
8000aac8:	afc20084 	sw	v0,132(s8)
8000aacc:	8fc20084 	lw	v0,132(s8)
8000aad0:	28420011 	slti	v0,v0,17
8000aad4:	1040ffe1 	beqz	v0,8000aa5c <_vfprintf_r+0x1c94>
8000aad8:	00000000 	nop
8000aadc:	3c028002 	lui	v0,0x8002
8000aae0:	2442a6e0 	addiu	v0,v0,-22816
8000aae4:	ae020000 	sw	v0,0(s0)
8000aae8:	8fc20084 	lw	v0,132(s8)
8000aaec:	ae020004 	sw	v0,4(s0)
8000aaf0:	8fc300d0 	lw	v1,208(s8)
8000aaf4:	8fc20084 	lw	v0,132(s8)
8000aaf8:	00621021 	addu	v0,v1,v0
8000aafc:	afc200d0 	sw	v0,208(s8)
8000ab00:	26100008 	addiu	s0,s0,8
8000ab04:	8fc200cc 	lw	v0,204(s8)
8000ab08:	24420001 	addiu	v0,v0,1
8000ab0c:	afc200cc 	sw	v0,204(s8)
8000ab10:	8fc200cc 	lw	v0,204(s8)
8000ab14:	28420008 	slti	v0,v0,8
8000ab18:	144000d9 	bnez	v0,8000ae80 <_vfprintf_r+0x20b8>
8000ab1c:	00000000 	nop
8000ab20:	27c200c8 	addiu	v0,s8,200
8000ab24:	00403025 	move	a2,v0
8000ab28:	8fc50190 	lw	a1,400(s8)
8000ab2c:	8fc40188 	lw	a0,392(s8)
8000ab30:	0c00512c 	jal	800144b0 <__sprint_r>
8000ab34:	00000000 	nop
8000ab38:	1440018d 	bnez	v0,8000b170 <_vfprintf_r+0x23a8>
8000ab3c:	00000000 	nop
8000ab40:	27d000d8 	addiu	s0,s8,216
8000ab44:	100000ce 	b	8000ae80 <_vfprintf_r+0x20b8>
8000ab48:	00000000 	nop
8000ab4c:	8fc200c0 	lw	v0,192(s8)
8000ab50:	28420002 	slti	v0,v0,2
8000ab54:	10400004 	beqz	v0,8000ab68 <_vfprintf_r+0x1da0>
8000ab58:	00000000 	nop
8000ab5c:	32220001 	andi	v0,s1,0x1
8000ab60:	10400094 	beqz	v0,8000adb4 <_vfprintf_r+0x1fec>
8000ab64:	00000000 	nop
8000ab68:	ae120000 	sw	s2,0(s0)
8000ab6c:	24020001 	li	v0,1
8000ab70:	ae020004 	sw	v0,4(s0)
8000ab74:	8fc200d0 	lw	v0,208(s8)
8000ab78:	24420001 	addiu	v0,v0,1
8000ab7c:	afc200d0 	sw	v0,208(s8)
8000ab80:	26100008 	addiu	s0,s0,8
8000ab84:	8fc200cc 	lw	v0,204(s8)
8000ab88:	24420001 	addiu	v0,v0,1
8000ab8c:	afc200cc 	sw	v0,204(s8)
8000ab90:	8fc200cc 	lw	v0,204(s8)
8000ab94:	28420008 	slti	v0,v0,8
8000ab98:	1440000a 	bnez	v0,8000abc4 <_vfprintf_r+0x1dfc>
8000ab9c:	00000000 	nop
8000aba0:	27c200c8 	addiu	v0,s8,200
8000aba4:	00403025 	move	a2,v0
8000aba8:	8fc50190 	lw	a1,400(s8)
8000abac:	8fc40188 	lw	a0,392(s8)
8000abb0:	0c00512c 	jal	800144b0 <__sprint_r>
8000abb4:	00000000 	nop
8000abb8:	14400170 	bnez	v0,8000b17c <_vfprintf_r+0x23b4>
8000abbc:	00000000 	nop
8000abc0:	27d000d8 	addiu	s0,s8,216
8000abc4:	26520001 	addiu	s2,s2,1
8000abc8:	8fc20088 	lw	v0,136(s8)
8000abcc:	ae020000 	sw	v0,0(s0)
8000abd0:	8fc2008c 	lw	v0,140(s8)
8000abd4:	ae020004 	sw	v0,4(s0)
8000abd8:	8fc300d0 	lw	v1,208(s8)
8000abdc:	8fc2008c 	lw	v0,140(s8)
8000abe0:	00621021 	addu	v0,v1,v0
8000abe4:	afc200d0 	sw	v0,208(s8)
8000abe8:	26100008 	addiu	s0,s0,8
8000abec:	8fc200cc 	lw	v0,204(s8)
8000abf0:	24420001 	addiu	v0,v0,1
8000abf4:	afc200cc 	sw	v0,204(s8)
8000abf8:	8fc200cc 	lw	v0,204(s8)
8000abfc:	28420008 	slti	v0,v0,8
8000ac00:	1440000a 	bnez	v0,8000ac2c <_vfprintf_r+0x1e64>
8000ac04:	00000000 	nop
8000ac08:	27c200c8 	addiu	v0,s8,200
8000ac0c:	00403025 	move	a2,v0
8000ac10:	8fc50190 	lw	a1,400(s8)
8000ac14:	8fc40188 	lw	a0,392(s8)
8000ac18:	0c00512c 	jal	800144b0 <__sprint_r>
8000ac1c:	00000000 	nop
8000ac20:	14400159 	bnez	v0,8000b188 <_vfprintf_r+0x23c0>
8000ac24:	00000000 	nop
8000ac28:	27d000d8 	addiu	s0,s8,216
8000ac2c:	d7c000a8 	ldc1	$f0,168(s8)
8000ac30:	44a00800 	dmtc1	zero,$f1
8000ac34:	00000000 	nop
8000ac38:	46210032 	c.eq.d	$f0,$f1
8000ac3c:	00000000 	nop
8000ac40:	4501001d 	bc1t	8000acb8 <_vfprintf_r+0x1ef0>
8000ac44:	00000000 	nop
8000ac48:	ae120000 	sw	s2,0(s0)
8000ac4c:	8fc200c0 	lw	v0,192(s8)
8000ac50:	2442ffff 	addiu	v0,v0,-1
8000ac54:	ae020004 	sw	v0,4(s0)
8000ac58:	8fc200d0 	lw	v0,208(s8)
8000ac5c:	8fc300c0 	lw	v1,192(s8)
8000ac60:	00431021 	addu	v0,v0,v1
8000ac64:	2442ffff 	addiu	v0,v0,-1
8000ac68:	afc200d0 	sw	v0,208(s8)
8000ac6c:	26100008 	addiu	s0,s0,8
8000ac70:	8fc200cc 	lw	v0,204(s8)
8000ac74:	24420001 	addiu	v0,v0,1
8000ac78:	afc200cc 	sw	v0,204(s8)
8000ac7c:	8fc200cc 	lw	v0,204(s8)
8000ac80:	28420008 	slti	v0,v0,8
8000ac84:	14400064 	bnez	v0,8000ae18 <_vfprintf_r+0x2050>
8000ac88:	00000000 	nop
8000ac8c:	27c200c8 	addiu	v0,s8,200
8000ac90:	00403025 	move	a2,v0
8000ac94:	8fc50190 	lw	a1,400(s8)
8000ac98:	8fc40188 	lw	a0,392(s8)
8000ac9c:	0c00512c 	jal	800144b0 <__sprint_r>
8000aca0:	00000000 	nop
8000aca4:	1440013b 	bnez	v0,8000b194 <_vfprintf_r+0x23cc>
8000aca8:	00000000 	nop
8000acac:	27d000d8 	addiu	s0,s8,216
8000acb0:	10000059 	b	8000ae18 <_vfprintf_r+0x2050>
8000acb4:	00000000 	nop
8000acb8:	8fc200c0 	lw	v0,192(s8)
8000acbc:	2453ffff 	addiu	s3,v0,-1
8000acc0:	1a600055 	blez	s3,8000ae18 <_vfprintf_r+0x2050>
8000acc4:	00000000 	nop
8000acc8:	1000001b 	b	8000ad38 <_vfprintf_r+0x1f70>
8000accc:	00000000 	nop
8000acd0:	3c028002 	lui	v0,0x8002
8000acd4:	2442a6e0 	addiu	v0,v0,-22816
8000acd8:	ae020000 	sw	v0,0(s0)
8000acdc:	24020010 	li	v0,16
8000ace0:	ae020004 	sw	v0,4(s0)
8000ace4:	8fc200d0 	lw	v0,208(s8)
8000ace8:	24420010 	addiu	v0,v0,16
8000acec:	afc200d0 	sw	v0,208(s8)
8000acf0:	26100008 	addiu	s0,s0,8
8000acf4:	8fc200cc 	lw	v0,204(s8)
8000acf8:	24420001 	addiu	v0,v0,1
8000acfc:	afc200cc 	sw	v0,204(s8)
8000ad00:	8fc200cc 	lw	v0,204(s8)
8000ad04:	28420008 	slti	v0,v0,8
8000ad08:	1440000a 	bnez	v0,8000ad34 <_vfprintf_r+0x1f6c>
8000ad0c:	00000000 	nop
8000ad10:	27c200c8 	addiu	v0,s8,200
8000ad14:	00403025 	move	a2,v0
8000ad18:	8fc50190 	lw	a1,400(s8)
8000ad1c:	8fc40188 	lw	a0,392(s8)
8000ad20:	0c00512c 	jal	800144b0 <__sprint_r>
8000ad24:	00000000 	nop
8000ad28:	1440011d 	bnez	v0,8000b1a0 <_vfprintf_r+0x23d8>
8000ad2c:	00000000 	nop
8000ad30:	27d000d8 	addiu	s0,s8,216
8000ad34:	2673fff0 	addiu	s3,s3,-16
8000ad38:	2a620011 	slti	v0,s3,17
8000ad3c:	1040ffe4 	beqz	v0,8000acd0 <_vfprintf_r+0x1f08>
8000ad40:	00000000 	nop
8000ad44:	3c028002 	lui	v0,0x8002
8000ad48:	2442a6e0 	addiu	v0,v0,-22816
8000ad4c:	ae020000 	sw	v0,0(s0)
8000ad50:	02601025 	move	v0,s3
8000ad54:	ae020004 	sw	v0,4(s0)
8000ad58:	8fc200d0 	lw	v0,208(s8)
8000ad5c:	02601825 	move	v1,s3
8000ad60:	00431021 	addu	v0,v0,v1
8000ad64:	afc200d0 	sw	v0,208(s8)
8000ad68:	26100008 	addiu	s0,s0,8
8000ad6c:	8fc200cc 	lw	v0,204(s8)
8000ad70:	24420001 	addiu	v0,v0,1
8000ad74:	afc200cc 	sw	v0,204(s8)
8000ad78:	8fc200cc 	lw	v0,204(s8)
8000ad7c:	28420008 	slti	v0,v0,8
8000ad80:	14400025 	bnez	v0,8000ae18 <_vfprintf_r+0x2050>
8000ad84:	00000000 	nop
8000ad88:	27c200c8 	addiu	v0,s8,200
8000ad8c:	00403025 	move	a2,v0
8000ad90:	8fc50190 	lw	a1,400(s8)
8000ad94:	8fc40188 	lw	a0,392(s8)
8000ad98:	0c00512c 	jal	800144b0 <__sprint_r>
8000ad9c:	00000000 	nop
8000ada0:	14400102 	bnez	v0,8000b1ac <_vfprintf_r+0x23e4>
8000ada4:	00000000 	nop
8000ada8:	27d000d8 	addiu	s0,s8,216
8000adac:	1000001a 	b	8000ae18 <_vfprintf_r+0x2050>
8000adb0:	00000000 	nop
8000adb4:	ae120000 	sw	s2,0(s0)
8000adb8:	24020001 	li	v0,1
8000adbc:	ae020004 	sw	v0,4(s0)
8000adc0:	8fc200d0 	lw	v0,208(s8)
8000adc4:	24420001 	addiu	v0,v0,1
8000adc8:	afc200d0 	sw	v0,208(s8)
8000adcc:	26100008 	addiu	s0,s0,8
8000add0:	8fc200cc 	lw	v0,204(s8)
8000add4:	24420001 	addiu	v0,v0,1
8000add8:	afc200cc 	sw	v0,204(s8)
8000addc:	8fc200cc 	lw	v0,204(s8)
8000ade0:	28420008 	slti	v0,v0,8
8000ade4:	1440000d 	bnez	v0,8000ae1c <_vfprintf_r+0x2054>
8000ade8:	00000000 	nop
8000adec:	27c200c8 	addiu	v0,s8,200
8000adf0:	00403025 	move	a2,v0
8000adf4:	8fc50190 	lw	a1,400(s8)
8000adf8:	8fc40188 	lw	a0,392(s8)
8000adfc:	0c00512c 	jal	800144b0 <__sprint_r>
8000ae00:	00000000 	nop
8000ae04:	144000ec 	bnez	v0,8000b1b8 <_vfprintf_r+0x23f0>
8000ae08:	00000000 	nop
8000ae0c:	27d000d8 	addiu	s0,s8,216
8000ae10:	10000002 	b	8000ae1c <_vfprintf_r+0x2054>
8000ae14:	00000000 	nop
8000ae18:	00000000 	nop
8000ae1c:	27c200b8 	addiu	v0,s8,184
8000ae20:	ae020000 	sw	v0,0(s0)
8000ae24:	8fc20054 	lw	v0,84(s8)
8000ae28:	ae020004 	sw	v0,4(s0)
8000ae2c:	8fc300d0 	lw	v1,208(s8)
8000ae30:	8fc20054 	lw	v0,84(s8)
8000ae34:	00621021 	addu	v0,v1,v0
8000ae38:	afc200d0 	sw	v0,208(s8)
8000ae3c:	26100008 	addiu	s0,s0,8
8000ae40:	8fc200cc 	lw	v0,204(s8)
8000ae44:	24420001 	addiu	v0,v0,1
8000ae48:	afc200cc 	sw	v0,204(s8)
8000ae4c:	8fc200cc 	lw	v0,204(s8)
8000ae50:	28420008 	slti	v0,v0,8
8000ae54:	1440000a 	bnez	v0,8000ae80 <_vfprintf_r+0x20b8>
8000ae58:	00000000 	nop
8000ae5c:	27c200c8 	addiu	v0,s8,200
8000ae60:	00403025 	move	a2,v0
8000ae64:	8fc50190 	lw	a1,400(s8)
8000ae68:	8fc40188 	lw	a0,392(s8)
8000ae6c:	0c00512c 	jal	800144b0 <__sprint_r>
8000ae70:	00000000 	nop
8000ae74:	144000d3 	bnez	v0,8000b1c4 <_vfprintf_r+0x23fc>
8000ae78:	00000000 	nop
8000ae7c:	27d000d8 	addiu	s0,s8,216
8000ae80:	32220004 	andi	v0,s1,0x4
8000ae84:	1040003d 	beqz	v0,8000af7c <_vfprintf_r+0x21b4>
8000ae88:	00000000 	nop
8000ae8c:	8fc3004c 	lw	v1,76(s8)
8000ae90:	8fc20070 	lw	v0,112(s8)
8000ae94:	00629823 	subu	s3,v1,v0
8000ae98:	1a600038 	blez	s3,8000af7c <_vfprintf_r+0x21b4>
8000ae9c:	00000000 	nop
8000aea0:	1000001b 	b	8000af10 <_vfprintf_r+0x2148>
8000aea4:	00000000 	nop
8000aea8:	3c028002 	lui	v0,0x8002
8000aeac:	2442a6d0 	addiu	v0,v0,-22832
8000aeb0:	ae020000 	sw	v0,0(s0)
8000aeb4:	24020010 	li	v0,16
8000aeb8:	ae020004 	sw	v0,4(s0)
8000aebc:	8fc200d0 	lw	v0,208(s8)
8000aec0:	24420010 	addiu	v0,v0,16
8000aec4:	afc200d0 	sw	v0,208(s8)
8000aec8:	26100008 	addiu	s0,s0,8
8000aecc:	8fc200cc 	lw	v0,204(s8)
8000aed0:	24420001 	addiu	v0,v0,1
8000aed4:	afc200cc 	sw	v0,204(s8)
8000aed8:	8fc200cc 	lw	v0,204(s8)
8000aedc:	28420008 	slti	v0,v0,8
8000aee0:	1440000a 	bnez	v0,8000af0c <_vfprintf_r+0x2144>
8000aee4:	00000000 	nop
8000aee8:	27c200c8 	addiu	v0,s8,200
8000aeec:	00403025 	move	a2,v0
8000aef0:	8fc50190 	lw	a1,400(s8)
8000aef4:	8fc40188 	lw	a0,392(s8)
8000aef8:	0c00512c 	jal	800144b0 <__sprint_r>
8000aefc:	00000000 	nop
8000af00:	144000b3 	bnez	v0,8000b1d0 <_vfprintf_r+0x2408>
8000af04:	00000000 	nop
8000af08:	27d000d8 	addiu	s0,s8,216
8000af0c:	2673fff0 	addiu	s3,s3,-16
8000af10:	2a620011 	slti	v0,s3,17
8000af14:	1040ffe4 	beqz	v0,8000aea8 <_vfprintf_r+0x20e0>
8000af18:	00000000 	nop
8000af1c:	3c028002 	lui	v0,0x8002
8000af20:	2442a6d0 	addiu	v0,v0,-22832
8000af24:	ae020000 	sw	v0,0(s0)
8000af28:	02601025 	move	v0,s3
8000af2c:	ae020004 	sw	v0,4(s0)
8000af30:	8fc200d0 	lw	v0,208(s8)
8000af34:	02601825 	move	v1,s3
8000af38:	00431021 	addu	v0,v0,v1
8000af3c:	afc200d0 	sw	v0,208(s8)
8000af40:	8fc200cc 	lw	v0,204(s8)
8000af44:	24420001 	addiu	v0,v0,1
8000af48:	afc200cc 	sw	v0,204(s8)
8000af4c:	8fc200cc 	lw	v0,204(s8)
8000af50:	28420008 	slti	v0,v0,8
8000af54:	14400009 	bnez	v0,8000af7c <_vfprintf_r+0x21b4>
8000af58:	00000000 	nop
8000af5c:	27c200c8 	addiu	v0,s8,200
8000af60:	00403025 	move	a2,v0
8000af64:	8fc50190 	lw	a1,400(s8)
8000af68:	8fc40188 	lw	a0,392(s8)
8000af6c:	0c00512c 	jal	800144b0 <__sprint_r>
8000af70:	00000000 	nop
8000af74:	14400099 	bnez	v0,8000b1dc <_vfprintf_r+0x2414>
8000af78:	00000000 	nop
8000af7c:	8fc3004c 	lw	v1,76(s8)
8000af80:	8fc20070 	lw	v0,112(s8)
8000af84:	0043202a 	slt	a0,v0,v1
8000af88:	10800002 	beqz	a0,8000af94 <_vfprintf_r+0x21cc>
8000af8c:	00000000 	nop
8000af90:	00601025 	move	v0,v1
8000af94:	8fc30048 	lw	v1,72(s8)
8000af98:	00621021 	addu	v0,v1,v0
8000af9c:	afc20048 	sw	v0,72(s8)
8000afa0:	8fc200d0 	lw	v0,208(s8)
8000afa4:	10400009 	beqz	v0,8000afcc <_vfprintf_r+0x2204>
8000afa8:	00000000 	nop
8000afac:	27c200c8 	addiu	v0,s8,200
8000afb0:	00403025 	move	a2,v0
8000afb4:	8fc50190 	lw	a1,400(s8)
8000afb8:	8fc40188 	lw	a0,392(s8)
8000afbc:	0c00512c 	jal	800144b0 <__sprint_r>
8000afc0:	00000000 	nop
8000afc4:	14400088 	bnez	v0,8000b1e8 <_vfprintf_r+0x2420>
8000afc8:	00000000 	nop
8000afcc:	afc000cc 	sw	zero,204(s8)
8000afd0:	27d000d8 	addiu	s0,s8,216
8000afd4:	8fc2007c 	lw	v0,124(s8)
8000afd8:	1040f7e3 	beqz	v0,80008f68 <_vfprintf_r+0x1a0>
8000afdc:	00000000 	nop
8000afe0:	8fc5007c 	lw	a1,124(s8)
8000afe4:	8fc40188 	lw	a0,392(s8)
8000afe8:	0c000dcb 	jal	8000372c <_free_r>
8000afec:	00000000 	nop
8000aff0:	afc0007c 	sw	zero,124(s8)
8000aff4:	1000f7dc 	b	80008f68 <_vfprintf_r+0x1a0>
8000aff8:	00000000 	nop
8000affc:	00000000 	nop
8000b000:	10000002 	b	8000b00c <_vfprintf_r+0x2244>
8000b004:	00000000 	nop
8000b008:	00000000 	nop
8000b00c:	8fc200d0 	lw	v0,208(s8)
8000b010:	10400009 	beqz	v0,8000b038 <_vfprintf_r+0x2270>
8000b014:	00000000 	nop
8000b018:	27c200c8 	addiu	v0,s8,200
8000b01c:	00403025 	move	a2,v0
8000b020:	8fc50190 	lw	a1,400(s8)
8000b024:	8fc40188 	lw	a0,392(s8)
8000b028:	0c00512c 	jal	800144b0 <__sprint_r>
8000b02c:	00000000 	nop
8000b030:	14400070 	bnez	v0,8000b1f4 <_vfprintf_r+0x242c>
8000b034:	00000000 	nop
8000b038:	afc000cc 	sw	zero,204(s8)
8000b03c:	1000006e 	b	8000b1f8 <_vfprintf_r+0x2430>
8000b040:	00000000 	nop
8000b044:	00000000 	nop
8000b048:	1000006b 	b	8000b1f8 <_vfprintf_r+0x2430>
8000b04c:	00000000 	nop
8000b050:	00000000 	nop
8000b054:	10000068 	b	8000b1f8 <_vfprintf_r+0x2430>
8000b058:	00000000 	nop
8000b05c:	00000000 	nop
8000b060:	10000065 	b	8000b1f8 <_vfprintf_r+0x2430>
8000b064:	00000000 	nop
8000b068:	00000000 	nop
8000b06c:	10000062 	b	8000b1f8 <_vfprintf_r+0x2430>
8000b070:	00000000 	nop
8000b074:	00000000 	nop
8000b078:	1000005f 	b	8000b1f8 <_vfprintf_r+0x2430>
8000b07c:	00000000 	nop
8000b080:	00000000 	nop
8000b084:	1000005c 	b	8000b1f8 <_vfprintf_r+0x2430>
8000b088:	00000000 	nop
8000b08c:	00000000 	nop
8000b090:	10000059 	b	8000b1f8 <_vfprintf_r+0x2430>
8000b094:	00000000 	nop
8000b098:	00000000 	nop
8000b09c:	10000056 	b	8000b1f8 <_vfprintf_r+0x2430>
8000b0a0:	00000000 	nop
8000b0a4:	00000000 	nop
8000b0a8:	10000053 	b	8000b1f8 <_vfprintf_r+0x2430>
8000b0ac:	00000000 	nop
8000b0b0:	00000000 	nop
8000b0b4:	10000050 	b	8000b1f8 <_vfprintf_r+0x2430>
8000b0b8:	00000000 	nop
8000b0bc:	00000000 	nop
8000b0c0:	1000004d 	b	8000b1f8 <_vfprintf_r+0x2430>
8000b0c4:	00000000 	nop
8000b0c8:	00000000 	nop
8000b0cc:	1000004a 	b	8000b1f8 <_vfprintf_r+0x2430>
8000b0d0:	00000000 	nop
8000b0d4:	00000000 	nop
8000b0d8:	10000047 	b	8000b1f8 <_vfprintf_r+0x2430>
8000b0dc:	00000000 	nop
8000b0e0:	00000000 	nop
8000b0e4:	10000044 	b	8000b1f8 <_vfprintf_r+0x2430>
8000b0e8:	00000000 	nop
8000b0ec:	00000000 	nop
8000b0f0:	10000041 	b	8000b1f8 <_vfprintf_r+0x2430>
8000b0f4:	00000000 	nop
8000b0f8:	00000000 	nop
8000b0fc:	1000003e 	b	8000b1f8 <_vfprintf_r+0x2430>
8000b100:	00000000 	nop
8000b104:	00000000 	nop
8000b108:	1000003b 	b	8000b1f8 <_vfprintf_r+0x2430>
8000b10c:	00000000 	nop
8000b110:	00000000 	nop
8000b114:	10000038 	b	8000b1f8 <_vfprintf_r+0x2430>
8000b118:	00000000 	nop
8000b11c:	00000000 	nop
8000b120:	10000035 	b	8000b1f8 <_vfprintf_r+0x2430>
8000b124:	00000000 	nop
8000b128:	00000000 	nop
8000b12c:	10000032 	b	8000b1f8 <_vfprintf_r+0x2430>
8000b130:	00000000 	nop
8000b134:	00000000 	nop
8000b138:	1000002f 	b	8000b1f8 <_vfprintf_r+0x2430>
8000b13c:	00000000 	nop
8000b140:	00000000 	nop
8000b144:	1000002c 	b	8000b1f8 <_vfprintf_r+0x2430>
8000b148:	00000000 	nop
8000b14c:	00000000 	nop
8000b150:	10000029 	b	8000b1f8 <_vfprintf_r+0x2430>
8000b154:	00000000 	nop
8000b158:	00000000 	nop
8000b15c:	10000026 	b	8000b1f8 <_vfprintf_r+0x2430>
8000b160:	00000000 	nop
8000b164:	00000000 	nop
8000b168:	10000023 	b	8000b1f8 <_vfprintf_r+0x2430>
8000b16c:	00000000 	nop
8000b170:	00000000 	nop
8000b174:	10000020 	b	8000b1f8 <_vfprintf_r+0x2430>
8000b178:	00000000 	nop
8000b17c:	00000000 	nop
8000b180:	1000001d 	b	8000b1f8 <_vfprintf_r+0x2430>
8000b184:	00000000 	nop
8000b188:	00000000 	nop
8000b18c:	1000001a 	b	8000b1f8 <_vfprintf_r+0x2430>
8000b190:	00000000 	nop
8000b194:	00000000 	nop
8000b198:	10000017 	b	8000b1f8 <_vfprintf_r+0x2430>
8000b19c:	00000000 	nop
8000b1a0:	00000000 	nop
8000b1a4:	10000014 	b	8000b1f8 <_vfprintf_r+0x2430>
8000b1a8:	00000000 	nop
8000b1ac:	00000000 	nop
8000b1b0:	10000011 	b	8000b1f8 <_vfprintf_r+0x2430>
8000b1b4:	00000000 	nop
8000b1b8:	00000000 	nop
8000b1bc:	1000000e 	b	8000b1f8 <_vfprintf_r+0x2430>
8000b1c0:	00000000 	nop
8000b1c4:	00000000 	nop
8000b1c8:	1000000b 	b	8000b1f8 <_vfprintf_r+0x2430>
8000b1cc:	00000000 	nop
8000b1d0:	00000000 	nop
8000b1d4:	10000008 	b	8000b1f8 <_vfprintf_r+0x2430>
8000b1d8:	00000000 	nop
8000b1dc:	00000000 	nop
8000b1e0:	10000005 	b	8000b1f8 <_vfprintf_r+0x2430>
8000b1e4:	00000000 	nop
8000b1e8:	00000000 	nop
8000b1ec:	10000002 	b	8000b1f8 <_vfprintf_r+0x2430>
8000b1f0:	00000000 	nop
8000b1f4:	00000000 	nop
8000b1f8:	8fc2007c 	lw	v0,124(s8)
8000b1fc:	10400005 	beqz	v0,8000b214 <_vfprintf_r+0x244c>
8000b200:	00000000 	nop
8000b204:	8fc5007c 	lw	a1,124(s8)
8000b208:	8fc40188 	lw	a0,392(s8)
8000b20c:	0c000dcb 	jal	8000372c <_free_r>
8000b210:	00000000 	nop
8000b214:	8fc20190 	lw	v0,400(s8)
8000b218:	8442000c 	lh	v0,12(v0)
8000b21c:	3042ffff 	andi	v0,v0,0xffff
8000b220:	30420040 	andi	v0,v0,0x40
8000b224:	14400004 	bnez	v0,8000b238 <_vfprintf_r+0x2470>
8000b228:	00000000 	nop
8000b22c:	8fc20048 	lw	v0,72(s8)
8000b230:	10000002 	b	8000b23c <_vfprintf_r+0x2474>
8000b234:	00000000 	nop
8000b238:	2402ffff 	li	v0,-1
8000b23c:	03c0e825 	move	sp,s8
8000b240:	dfbf0180 	ld	ra,384(sp)
8000b244:	dfbe0178 	ld	s8,376(sp)
8000b248:	dfb50170 	ld	s5,368(sp)
8000b24c:	dfb40168 	ld	s4,360(sp)
8000b250:	dfb30160 	ld	s3,352(sp)
8000b254:	dfb20158 	ld	s2,344(sp)
8000b258:	dfb10150 	ld	s1,336(sp)
8000b25c:	dfb00148 	ld	s0,328(sp)
8000b260:	27bd0188 	addiu	sp,sp,392
8000b264:	03e00008 	jr	ra
8000b268:	00000000 	nop

8000b26c <cvt>:
8000b26c:	27bdff98 	addiu	sp,sp,-104
8000b270:	ffbf0060 	sd	ra,96(sp)
8000b274:	ffbe0058 	sd	s8,88(sp)
8000b278:	03a0f025 	move	s8,sp
8000b27c:	afc40068 	sw	a0,104(s8)
8000b280:	ffc50070 	sd	a1,112(s8)
8000b284:	afc60078 	sw	a2,120(s8)
8000b288:	afc70080 	sw	a3,128(s8)
8000b28c:	d7c00070 	ldc1	$f0,112(s8)
8000b290:	f7c00050 	sdc1	$f0,80(s8)
8000b294:	8fc20050 	lw	v0,80(s8)
8000b298:	0441000b 	bgez	v0,8000b2c8 <cvt+0x5c>
8000b29c:	00000000 	nop
8000b2a0:	dfc30070 	ld	v1,112(s8)
8000b2a4:	2402ffff 	li	v0,-1
8000b2a8:	000217fc 	dsll32	v0,v0,0x1f
8000b2ac:	00621026 	xor	v0,v1,v0
8000b2b0:	ffc20070 	sd	v0,112(s8)
8000b2b4:	8fc2008c 	lw	v0,140(s8)
8000b2b8:	2403002d 	li	v1,45
8000b2bc:	a0430000 	sb	v1,0(v0)
8000b2c0:	10000003 	b	8000b2d0 <cvt+0x64>
8000b2c4:	00000000 	nop
8000b2c8:	8fc2008c 	lw	v0,140(s8)
8000b2cc:	a0400000 	sb	zero,0(v0)
8000b2d0:	8fc3009c 	lw	v1,156(s8)
8000b2d4:	24020066 	li	v0,102
8000b2d8:	10620005 	beq	v1,v0,8000b2f0 <cvt+0x84>
8000b2dc:	00000000 	nop
8000b2e0:	8fc3009c 	lw	v1,156(s8)
8000b2e4:	24020046 	li	v0,70
8000b2e8:	14620005 	bne	v1,v0,8000b300 <cvt+0x94>
8000b2ec:	00000000 	nop
8000b2f0:	24020003 	li	v0,3
8000b2f4:	afc20038 	sw	v0,56(s8)
8000b2f8:	1000000e 	b	8000b334 <cvt+0xc8>
8000b2fc:	00000000 	nop
8000b300:	8fc3009c 	lw	v1,156(s8)
8000b304:	24020065 	li	v0,101
8000b308:	10620005 	beq	v1,v0,8000b320 <cvt+0xb4>
8000b30c:	00000000 	nop
8000b310:	8fc3009c 	lw	v1,156(s8)
8000b314:	24020045 	li	v0,69
8000b318:	14620004 	bne	v1,v0,8000b32c <cvt+0xc0>
8000b31c:	00000000 	nop
8000b320:	8fc20078 	lw	v0,120(s8)
8000b324:	24420001 	addiu	v0,v0,1
8000b328:	afc20078 	sw	v0,120(s8)
8000b32c:	24020002 	li	v0,2
8000b330:	afc20038 	sw	v0,56(s8)
8000b334:	27c20048 	addiu	v0,s8,72
8000b338:	afa20034 	sw	v0,52(sp)
8000b33c:	27c20044 	addiu	v0,s8,68
8000b340:	afa2002c 	sw	v0,44(sp)
8000b344:	8fc20094 	lw	v0,148(s8)
8000b348:	afa20024 	sw	v0,36(sp)
8000b34c:	8fc70078 	lw	a3,120(s8)
8000b350:	8fc60038 	lw	a2,56(s8)
8000b354:	dfc50070 	ld	a1,112(s8)
8000b358:	8fc40068 	lw	a0,104(s8)
8000b35c:	0c00302e 	jal	8000c0b8 <_dtoa_r>
8000b360:	00000000 	nop
8000b364:	afc20040 	sw	v0,64(s8)
8000b368:	8fc3009c 	lw	v1,156(s8)
8000b36c:	24020067 	li	v0,103
8000b370:	10620005 	beq	v1,v0,8000b388 <cvt+0x11c>
8000b374:	00000000 	nop
8000b378:	8fc3009c 	lw	v1,156(s8)
8000b37c:	24020047 	li	v0,71
8000b380:	14620005 	bne	v1,v0,8000b398 <cvt+0x12c>
8000b384:	00000000 	nop
8000b388:	8fc20080 	lw	v0,128(s8)
8000b38c:	30420001 	andi	v0,v0,0x1
8000b390:	10400039 	beqz	v0,8000b478 <cvt+0x20c>
8000b394:	00000000 	nop
8000b398:	8fc20078 	lw	v0,120(s8)
8000b39c:	8fc30040 	lw	v1,64(s8)
8000b3a0:	00621021 	addu	v0,v1,v0
8000b3a4:	afc2003c 	sw	v0,60(s8)
8000b3a8:	8fc3009c 	lw	v1,156(s8)
8000b3ac:	24020066 	li	v0,102
8000b3b0:	10620005 	beq	v1,v0,8000b3c8 <cvt+0x15c>
8000b3b4:	00000000 	nop
8000b3b8:	8fc3009c 	lw	v1,156(s8)
8000b3bc:	24020046 	li	v0,70
8000b3c0:	14620018 	bne	v1,v0,8000b424 <cvt+0x1b8>
8000b3c4:	00000000 	nop
8000b3c8:	8fc20040 	lw	v0,64(s8)
8000b3cc:	80430000 	lb	v1,0(v0)
8000b3d0:	24020030 	li	v0,48
8000b3d4:	1462000d 	bne	v1,v0,8000b40c <cvt+0x1a0>
8000b3d8:	00000000 	nop
8000b3dc:	d7c00070 	ldc1	$f0,112(s8)
8000b3e0:	44a00800 	dmtc1	zero,$f1
8000b3e4:	00000000 	nop
8000b3e8:	46210032 	c.eq.d	$f0,$f1
8000b3ec:	00000000 	nop
8000b3f0:	45010006 	bc1t	8000b40c <cvt+0x1a0>
8000b3f4:	00000000 	nop
8000b3f8:	24030001 	li	v1,1
8000b3fc:	8fc20078 	lw	v0,120(s8)
8000b400:	00621823 	subu	v1,v1,v0
8000b404:	8fc20094 	lw	v0,148(s8)
8000b408:	ac430000 	sw	v1,0(v0)
8000b40c:	8fc20094 	lw	v0,148(s8)
8000b410:	8c420000 	lw	v0,0(v0)
8000b414:	00401825 	move	v1,v0
8000b418:	8fc2003c 	lw	v0,60(s8)
8000b41c:	00431021 	addu	v0,v0,v1
8000b420:	afc2003c 	sw	v0,60(s8)
8000b424:	d7c00070 	ldc1	$f0,112(s8)
8000b428:	44a00800 	dmtc1	zero,$f1
8000b42c:	00000000 	nop
8000b430:	46210032 	c.eq.d	$f0,$f1
8000b434:	00000000 	nop
8000b438:	4500000a 	bc1f	8000b464 <cvt+0x1f8>
8000b43c:	00000000 	nop
8000b440:	8fc2003c 	lw	v0,60(s8)
8000b444:	afc20048 	sw	v0,72(s8)
8000b448:	10000006 	b	8000b464 <cvt+0x1f8>
8000b44c:	00000000 	nop
8000b450:	8fc20048 	lw	v0,72(s8)
8000b454:	24430001 	addiu	v1,v0,1
8000b458:	afc30048 	sw	v1,72(s8)
8000b45c:	24030030 	li	v1,48
8000b460:	a0430000 	sb	v1,0(v0)
8000b464:	8fc20048 	lw	v0,72(s8)
8000b468:	8fc3003c 	lw	v1,60(s8)
8000b46c:	0043102b 	sltu	v0,v0,v1
8000b470:	1440fff7 	bnez	v0,8000b450 <cvt+0x1e4>
8000b474:	00000000 	nop
8000b478:	8fc30048 	lw	v1,72(s8)
8000b47c:	8fc20040 	lw	v0,64(s8)
8000b480:	00621823 	subu	v1,v1,v0
8000b484:	8fc200a4 	lw	v0,164(s8)
8000b488:	ac430000 	sw	v1,0(v0)
8000b48c:	8fc20040 	lw	v0,64(s8)
8000b490:	03c0e825 	move	sp,s8
8000b494:	dfbf0060 	ld	ra,96(sp)
8000b498:	dfbe0058 	ld	s8,88(sp)
8000b49c:	27bd0068 	addiu	sp,sp,104
8000b4a0:	03e00008 	jr	ra
8000b4a4:	00000000 	nop

8000b4a8 <exponent>:
8000b4a8:	27bdffe0 	addiu	sp,sp,-32
8000b4ac:	ffbe0018 	sd	s8,24(sp)
8000b4b0:	ffb10010 	sd	s1,16(sp)
8000b4b4:	ffb00008 	sd	s0,8(sp)
8000b4b8:	03a0f025 	move	s8,sp
8000b4bc:	afc40020 	sw	a0,32(s8)
8000b4c0:	afc50028 	sw	a1,40(s8)
8000b4c4:	afc60030 	sw	a2,48(s8)
8000b4c8:	8fd00020 	lw	s0,32(s8)
8000b4cc:	02001025 	move	v0,s0
8000b4d0:	24500001 	addiu	s0,v0,1
8000b4d4:	8fc30030 	lw	v1,48(s8)
8000b4d8:	00031e00 	sll	v1,v1,0x18
8000b4dc:	00031e03 	sra	v1,v1,0x18
8000b4e0:	a0430000 	sb	v1,0(v0)
8000b4e4:	8fc20028 	lw	v0,40(s8)
8000b4e8:	0441000a 	bgez	v0,8000b514 <exponent+0x6c>
8000b4ec:	00000000 	nop
8000b4f0:	8fc20028 	lw	v0,40(s8)
8000b4f4:	00021023 	negu	v0,v0
8000b4f8:	afc20028 	sw	v0,40(s8)
8000b4fc:	02001025 	move	v0,s0
8000b500:	24500001 	addiu	s0,v0,1
8000b504:	2403002d 	li	v1,45
8000b508:	a0430000 	sb	v1,0(v0)
8000b50c:	10000005 	b	8000b524 <exponent+0x7c>
8000b510:	00000000 	nop
8000b514:	02001025 	move	v0,s0
8000b518:	24500001 	addiu	s0,v0,1
8000b51c:	2403002b 	li	v1,43
8000b520:	a0430000 	sb	v1,0(v0)
8000b524:	27d10007 	addiu	s1,s8,7
8000b528:	8fc20028 	lw	v0,40(s8)
8000b52c:	2842000a 	slti	v0,v0,10
8000b530:	1440004c 	bnez	v0,8000b664 <exponent+0x1bc>
8000b534:	00000000 	nop
8000b538:	8fc40028 	lw	a0,40(s8)
8000b53c:	00801825 	move	v1,a0
8000b540:	00601025 	move	v0,v1
8000b544:	00021078 	dsll	v0,v0,0x1
8000b548:	0043102d 	daddu	v0,v0,v1
8000b54c:	00022938 	dsll	a1,v0,0x4
8000b550:	0045102d 	daddu	v0,v0,a1
8000b554:	00022a38 	dsll	a1,v0,0x8
8000b558:	0045102d 	daddu	v0,v0,a1
8000b55c:	00022c38 	dsll	a1,v0,0x10
8000b560:	0045102d 	daddu	v0,v0,a1
8000b564:	00021078 	dsll	v0,v0,0x1
8000b568:	0043102d 	daddu	v0,v0,v1
8000b56c:	0002103e 	dsrl32	v0,v0,0x0
8000b570:	00021000 	sll	v0,v0,0x0
8000b574:	00021883 	sra	v1,v0,0x2
8000b578:	000417c3 	sra	v0,a0,0x1f
8000b57c:	00621823 	subu	v1,v1,v0
8000b580:	00601025 	move	v0,v1
8000b584:	00021080 	sll	v0,v0,0x2
8000b588:	00431021 	addu	v0,v0,v1
8000b58c:	00021040 	sll	v0,v0,0x1
8000b590:	00821823 	subu	v1,a0,v0
8000b594:	306200ff 	andi	v0,v1,0xff
8000b598:	24420030 	addiu	v0,v0,48
8000b59c:	304200ff 	andi	v0,v0,0xff
8000b5a0:	2631ffff 	addiu	s1,s1,-1
8000b5a4:	00021600 	sll	v0,v0,0x18
8000b5a8:	00021603 	sra	v0,v0,0x18
8000b5ac:	a2220000 	sb	v0,0(s1)
8000b5b0:	8fc50028 	lw	a1,40(s8)
8000b5b4:	00a01825 	move	v1,a1
8000b5b8:	00601025 	move	v0,v1
8000b5bc:	00021078 	dsll	v0,v0,0x1
8000b5c0:	0043102d 	daddu	v0,v0,v1
8000b5c4:	00022138 	dsll	a0,v0,0x4
8000b5c8:	0044102d 	daddu	v0,v0,a0
8000b5cc:	00022238 	dsll	a0,v0,0x8
8000b5d0:	0044102d 	daddu	v0,v0,a0
8000b5d4:	00022438 	dsll	a0,v0,0x10
8000b5d8:	0044102d 	daddu	v0,v0,a0
8000b5dc:	00021078 	dsll	v0,v0,0x1
8000b5e0:	0043102d 	daddu	v0,v0,v1
8000b5e4:	0002103e 	dsrl32	v0,v0,0x0
8000b5e8:	00021000 	sll	v0,v0,0x0
8000b5ec:	00021883 	sra	v1,v0,0x2
8000b5f0:	000517c3 	sra	v0,a1,0x1f
8000b5f4:	00621023 	subu	v0,v1,v0
8000b5f8:	afc20028 	sw	v0,40(s8)
8000b5fc:	8fc20028 	lw	v0,40(s8)
8000b600:	2842000a 	slti	v0,v0,10
8000b604:	1040ffcc 	beqz	v0,8000b538 <exponent+0x90>
8000b608:	00000000 	nop
8000b60c:	8fc20028 	lw	v0,40(s8)
8000b610:	304200ff 	andi	v0,v0,0xff
8000b614:	24420030 	addiu	v0,v0,48
8000b618:	304200ff 	andi	v0,v0,0xff
8000b61c:	2631ffff 	addiu	s1,s1,-1
8000b620:	00021600 	sll	v0,v0,0x18
8000b624:	00021603 	sra	v0,v0,0x18
8000b628:	a2220000 	sb	v0,0(s1)
8000b62c:	10000007 	b	8000b64c <exponent+0x1a4>
8000b630:	00000000 	nop
8000b634:	02201825 	move	v1,s1
8000b638:	24710001 	addiu	s1,v1,1
8000b63c:	02001025 	move	v0,s0
8000b640:	24500001 	addiu	s0,v0,1
8000b644:	80630000 	lb	v1,0(v1)
8000b648:	a0430000 	sb	v1,0(v0)
8000b64c:	27c20007 	addiu	v0,s8,7
8000b650:	0222102b 	sltu	v0,s1,v0
8000b654:	1440fff7 	bnez	v0,8000b634 <exponent+0x18c>
8000b658:	00000000 	nop
8000b65c:	1000000e 	b	8000b698 <exponent+0x1f0>
8000b660:	00000000 	nop
8000b664:	02001025 	move	v0,s0
8000b668:	24500001 	addiu	s0,v0,1
8000b66c:	24030030 	li	v1,48
8000b670:	a0430000 	sb	v1,0(v0)
8000b674:	8fc20028 	lw	v0,40(s8)
8000b678:	304200ff 	andi	v0,v0,0xff
8000b67c:	24420030 	addiu	v0,v0,48
8000b680:	304300ff 	andi	v1,v0,0xff
8000b684:	02001025 	move	v0,s0
8000b688:	24500001 	addiu	s0,v0,1
8000b68c:	00031e00 	sll	v1,v1,0x18
8000b690:	00031e03 	sra	v1,v1,0x18
8000b694:	a0430000 	sb	v1,0(v0)
8000b698:	8fc20020 	lw	v0,32(s8)
8000b69c:	02021023 	subu	v0,s0,v0
8000b6a0:	03c0e825 	move	sp,s8
8000b6a4:	dfbe0018 	ld	s8,24(sp)
8000b6a8:	dfb10010 	ld	s1,16(sp)
8000b6ac:	dfb00008 	ld	s0,8(sp)
8000b6b0:	27bd0020 	addiu	sp,sp,32
8000b6b4:	03e00008 	jr	ra
8000b6b8:	00000000 	nop

8000b6bc <__swbuf_r>:
8000b6bc:	27bdffb0 	addiu	sp,sp,-80
8000b6c0:	ffbf0048 	sd	ra,72(sp)
8000b6c4:	ffbe0040 	sd	s8,64(sp)
8000b6c8:	ffb20038 	sd	s2,56(sp)
8000b6cc:	ffb10030 	sd	s1,48(sp)
8000b6d0:	ffb00028 	sd	s0,40(sp)
8000b6d4:	03a0f025 	move	s8,sp
8000b6d8:	afc40050 	sw	a0,80(s8)
8000b6dc:	00a09025 	move	s2,a1
8000b6e0:	00c08025 	move	s0,a2
8000b6e4:	8fc20050 	lw	v0,80(s8)
8000b6e8:	afc20020 	sw	v0,32(s8)
8000b6ec:	8fc20020 	lw	v0,32(s8)
8000b6f0:	10400008 	beqz	v0,8000b714 <__swbuf_r+0x58>
8000b6f4:	00000000 	nop
8000b6f8:	8fc20020 	lw	v0,32(s8)
8000b6fc:	8c420038 	lw	v0,56(v0)
8000b700:	14400004 	bnez	v0,8000b714 <__swbuf_r+0x58>
8000b704:	00000000 	nop
8000b708:	8fc40020 	lw	a0,32(s8)
8000b70c:	0c000cc3 	jal	8000330c <__sinit>
8000b710:	00000000 	nop
8000b714:	8e020018 	lw	v0,24(s0)
8000b718:	ae020008 	sw	v0,8(s0)
8000b71c:	8602000c 	lh	v0,12(s0)
8000b720:	3042ffff 	andi	v0,v0,0xffff
8000b724:	30420008 	andi	v0,v0,0x8
8000b728:	10400004 	beqz	v0,8000b73c <__swbuf_r+0x80>
8000b72c:	00000000 	nop
8000b730:	8e020010 	lw	v0,16(s0)
8000b734:	1440000a 	bnez	v0,8000b760 <__swbuf_r+0xa4>
8000b738:	00000000 	nop
8000b73c:	02002825 	move	a1,s0
8000b740:	8fc40050 	lw	a0,80(s8)
8000b744:	0c002e52 	jal	8000b948 <__swsetup_r>
8000b748:	00000000 	nop
8000b74c:	10400004 	beqz	v0,8000b760 <__swbuf_r+0xa4>
8000b750:	00000000 	nop
8000b754:	2402ffff 	li	v0,-1
8000b758:	10000040 	b	8000b85c <__swbuf_r+0x1a0>
8000b75c:	00000000 	nop
8000b760:	324200ff 	andi	v0,s2,0xff
8000b764:	00409025 	move	s2,v0
8000b768:	8602000c 	lh	v0,12(s0)
8000b76c:	3042ffff 	andi	v0,v0,0xffff
8000b770:	30422000 	andi	v0,v0,0x2000
8000b774:	1440000a 	bnez	v0,8000b7a0 <__swbuf_r+0xe4>
8000b778:	00000000 	nop
8000b77c:	8602000c 	lh	v0,12(s0)
8000b780:	34422000 	ori	v0,v0,0x2000
8000b784:	00021400 	sll	v0,v0,0x10
8000b788:	00021403 	sra	v0,v0,0x10
8000b78c:	a602000c 	sh	v0,12(s0)
8000b790:	8e030064 	lw	v1,100(s0)
8000b794:	2402dfff 	li	v0,-8193
8000b798:	00621024 	and	v0,v1,v0
8000b79c:	ae020064 	sw	v0,100(s0)
8000b7a0:	8e030000 	lw	v1,0(s0)
8000b7a4:	8e020010 	lw	v0,16(s0)
8000b7a8:	00628823 	subu	s1,v1,v0
8000b7ac:	8e020014 	lw	v0,20(s0)
8000b7b0:	0222102a 	slt	v0,s1,v0
8000b7b4:	1440000b 	bnez	v0,8000b7e4 <__swbuf_r+0x128>
8000b7b8:	00000000 	nop
8000b7bc:	02002825 	move	a1,s0
8000b7c0:	8fc40050 	lw	a0,80(s8)
8000b7c4:	0c000b45 	jal	80002d14 <_fflush_r>
8000b7c8:	00000000 	nop
8000b7cc:	10400004 	beqz	v0,8000b7e0 <__swbuf_r+0x124>
8000b7d0:	00000000 	nop
8000b7d4:	2402ffff 	li	v0,-1
8000b7d8:	10000020 	b	8000b85c <__swbuf_r+0x1a0>
8000b7dc:	00000000 	nop
8000b7e0:	00008825 	move	s1,zero
8000b7e4:	8e020008 	lw	v0,8(s0)
8000b7e8:	2442ffff 	addiu	v0,v0,-1
8000b7ec:	ae020008 	sw	v0,8(s0)
8000b7f0:	8e020000 	lw	v0,0(s0)
8000b7f4:	24430001 	addiu	v1,v0,1
8000b7f8:	ae030000 	sw	v1,0(s0)
8000b7fc:	324300ff 	andi	v1,s2,0xff
8000b800:	a0430000 	sb	v1,0(v0)
8000b804:	26310001 	addiu	s1,s1,1
8000b808:	8e020014 	lw	v0,20(s0)
8000b80c:	12220009 	beq	s1,v0,8000b834 <__swbuf_r+0x178>
8000b810:	00000000 	nop
8000b814:	8602000c 	lh	v0,12(s0)
8000b818:	3042ffff 	andi	v0,v0,0xffff
8000b81c:	30420001 	andi	v0,v0,0x1
8000b820:	1040000d 	beqz	v0,8000b858 <__swbuf_r+0x19c>
8000b824:	00000000 	nop
8000b828:	2402000a 	li	v0,10
8000b82c:	1642000a 	bne	s2,v0,8000b858 <__swbuf_r+0x19c>
8000b830:	00000000 	nop
8000b834:	02002825 	move	a1,s0
8000b838:	8fc40050 	lw	a0,80(s8)
8000b83c:	0c000b45 	jal	80002d14 <_fflush_r>
8000b840:	00000000 	nop
8000b844:	10400004 	beqz	v0,8000b858 <__swbuf_r+0x19c>
8000b848:	00000000 	nop
8000b84c:	2402ffff 	li	v0,-1
8000b850:	10000002 	b	8000b85c <__swbuf_r+0x1a0>
8000b854:	00000000 	nop
8000b858:	02401025 	move	v0,s2
8000b85c:	03c0e825 	move	sp,s8
8000b860:	dfbf0048 	ld	ra,72(sp)
8000b864:	dfbe0040 	ld	s8,64(sp)
8000b868:	dfb20038 	ld	s2,56(sp)
8000b86c:	dfb10030 	ld	s1,48(sp)
8000b870:	dfb00028 	ld	s0,40(sp)
8000b874:	27bd0050 	addiu	sp,sp,80
8000b878:	03e00008 	jr	ra
8000b87c:	00000000 	nop

8000b880 <__swbuf>:
8000b880:	27bdffd0 	addiu	sp,sp,-48
8000b884:	ffbf0028 	sd	ra,40(sp)
8000b888:	ffbe0020 	sd	s8,32(sp)
8000b88c:	03a0f025 	move	s8,sp
8000b890:	00801825 	move	v1,a0
8000b894:	00a02025 	move	a0,a1
8000b898:	8f828028 	lw	v0,-32728(gp)
8000b89c:	00803025 	move	a2,a0
8000b8a0:	00602825 	move	a1,v1
8000b8a4:	00402025 	move	a0,v0
8000b8a8:	0c002daf 	jal	8000b6bc <__swbuf_r>
8000b8ac:	00000000 	nop
8000b8b0:	03c0e825 	move	sp,s8
8000b8b4:	dfbf0028 	ld	ra,40(sp)
8000b8b8:	dfbe0020 	ld	s8,32(sp)
8000b8bc:	27bd0030 	addiu	sp,sp,48
8000b8c0:	03e00008 	jr	ra
8000b8c4:	00000000 	nop

8000b8c8 <_write_r>:
8000b8c8:	27bdffc8 	addiu	sp,sp,-56
8000b8cc:	ffbf0030 	sd	ra,48(sp)
8000b8d0:	ffbe0028 	sd	s8,40(sp)
8000b8d4:	03a0f025 	move	s8,sp
8000b8d8:	afc40038 	sw	a0,56(s8)
8000b8dc:	afc50040 	sw	a1,64(s8)
8000b8e0:	afc60048 	sw	a2,72(s8)
8000b8e4:	afc70050 	sw	a3,80(s8)
8000b8e8:	af808334 	sw	zero,-31948(gp)
8000b8ec:	8fc60050 	lw	a2,80(s8)
8000b8f0:	8fc50048 	lw	a1,72(s8)
8000b8f4:	8fc40040 	lw	a0,64(s8)
8000b8f8:	0c006866 	jal	8001a198 <write>
8000b8fc:	00000000 	nop
8000b900:	afc20020 	sw	v0,32(s8)
8000b904:	8fc30020 	lw	v1,32(s8)
8000b908:	2402ffff 	li	v0,-1
8000b90c:	14620007 	bne	v1,v0,8000b92c <_write_r+0x64>
8000b910:	00000000 	nop
8000b914:	8f828334 	lw	v0,-31948(gp)
8000b918:	10400004 	beqz	v0,8000b92c <_write_r+0x64>
8000b91c:	00000000 	nop
8000b920:	8f838334 	lw	v1,-31948(gp)
8000b924:	8fc20038 	lw	v0,56(s8)
8000b928:	ac430000 	sw	v1,0(v0)
8000b92c:	8fc20020 	lw	v0,32(s8)
8000b930:	03c0e825 	move	sp,s8
8000b934:	dfbf0030 	ld	ra,48(sp)
8000b938:	dfbe0028 	ld	s8,40(sp)
8000b93c:	27bd0038 	addiu	sp,sp,56
8000b940:	03e00008 	jr	ra
8000b944:	00000000 	nop

8000b948 <__swsetup_r>:
8000b948:	27bdffc0 	addiu	sp,sp,-64
8000b94c:	ffbf0038 	sd	ra,56(sp)
8000b950:	ffbe0030 	sd	s8,48(sp)
8000b954:	ffb00028 	sd	s0,40(sp)
8000b958:	03a0f025 	move	s8,sp
8000b95c:	afc40040 	sw	a0,64(s8)
8000b960:	00a08025 	move	s0,a1
8000b964:	8f828028 	lw	v0,-32728(gp)
8000b968:	afc20020 	sw	v0,32(s8)
8000b96c:	8fc20020 	lw	v0,32(s8)
8000b970:	10400008 	beqz	v0,8000b994 <__swsetup_r+0x4c>
8000b974:	00000000 	nop
8000b978:	8fc20020 	lw	v0,32(s8)
8000b97c:	8c420038 	lw	v0,56(v0)
8000b980:	14400004 	bnez	v0,8000b994 <__swsetup_r+0x4c>
8000b984:	00000000 	nop
8000b988:	8fc40020 	lw	a0,32(s8)
8000b98c:	0c000cc3 	jal	8000330c <__sinit>
8000b990:	00000000 	nop
8000b994:	8602000c 	lh	v0,12(s0)
8000b998:	3042ffff 	andi	v0,v0,0xffff
8000b99c:	30420008 	andi	v0,v0,0x8
8000b9a0:	14400031 	bnez	v0,8000ba68 <__swsetup_r+0x120>
8000b9a4:	00000000 	nop
8000b9a8:	8602000c 	lh	v0,12(s0)
8000b9ac:	3042ffff 	andi	v0,v0,0xffff
8000b9b0:	30420010 	andi	v0,v0,0x10
8000b9b4:	1440000c 	bnez	v0,8000b9e8 <__swsetup_r+0xa0>
8000b9b8:	00000000 	nop
8000b9bc:	8fc20040 	lw	v0,64(s8)
8000b9c0:	24030009 	li	v1,9
8000b9c4:	ac430000 	sw	v1,0(v0)
8000b9c8:	8602000c 	lh	v0,12(s0)
8000b9cc:	34420040 	ori	v0,v0,0x40
8000b9d0:	00021400 	sll	v0,v0,0x10
8000b9d4:	00021403 	sra	v0,v0,0x10
8000b9d8:	a602000c 	sh	v0,12(s0)
8000b9dc:	2402ffff 	li	v0,-1
8000b9e0:	10000058 	b	8000bb44 <__swsetup_r+0x1fc>
8000b9e4:	00000000 	nop
8000b9e8:	8602000c 	lh	v0,12(s0)
8000b9ec:	3042ffff 	andi	v0,v0,0xffff
8000b9f0:	30420004 	andi	v0,v0,0x4
8000b9f4:	10400017 	beqz	v0,8000ba54 <__swsetup_r+0x10c>
8000b9f8:	00000000 	nop
8000b9fc:	8e020030 	lw	v0,48(s0)
8000ba00:	1040000b 	beqz	v0,8000ba30 <__swsetup_r+0xe8>
8000ba04:	00000000 	nop
8000ba08:	8e030030 	lw	v1,48(s0)
8000ba0c:	26020040 	addiu	v0,s0,64
8000ba10:	10620006 	beq	v1,v0,8000ba2c <__swsetup_r+0xe4>
8000ba14:	00000000 	nop
8000ba18:	8e020030 	lw	v0,48(s0)
8000ba1c:	00402825 	move	a1,v0
8000ba20:	8fc40040 	lw	a0,64(s8)
8000ba24:	0c000dcb 	jal	8000372c <_free_r>
8000ba28:	00000000 	nop
8000ba2c:	ae000030 	sw	zero,48(s0)
8000ba30:	8603000c 	lh	v1,12(s0)
8000ba34:	2402ffdb 	li	v0,-37
8000ba38:	00621024 	and	v0,v1,v0
8000ba3c:	00021400 	sll	v0,v0,0x10
8000ba40:	00021403 	sra	v0,v0,0x10
8000ba44:	a602000c 	sh	v0,12(s0)
8000ba48:	ae000004 	sw	zero,4(s0)
8000ba4c:	8e020010 	lw	v0,16(s0)
8000ba50:	ae020000 	sw	v0,0(s0)
8000ba54:	8602000c 	lh	v0,12(s0)
8000ba58:	34420008 	ori	v0,v0,0x8
8000ba5c:	00021400 	sll	v0,v0,0x10
8000ba60:	00021403 	sra	v0,v0,0x10
8000ba64:	a602000c 	sh	v0,12(s0)
8000ba68:	8e020010 	lw	v0,16(s0)
8000ba6c:	1440000f 	bnez	v0,8000baac <__swsetup_r+0x164>
8000ba70:	00000000 	nop
8000ba74:	8602000c 	lh	v0,12(s0)
8000ba78:	3042ffff 	andi	v0,v0,0xffff
8000ba7c:	30420200 	andi	v0,v0,0x200
8000ba80:	10400006 	beqz	v0,8000ba9c <__swsetup_r+0x154>
8000ba84:	00000000 	nop
8000ba88:	8602000c 	lh	v0,12(s0)
8000ba8c:	3042ffff 	andi	v0,v0,0xffff
8000ba90:	30420080 	andi	v0,v0,0x80
8000ba94:	10400005 	beqz	v0,8000baac <__swsetup_r+0x164>
8000ba98:	00000000 	nop
8000ba9c:	02002825 	move	a1,s0
8000baa0:	8fc40040 	lw	a0,64(s8)
8000baa4:	0c0010bf 	jal	800042fc <__smakebuf_r>
8000baa8:	00000000 	nop
8000baac:	8602000c 	lh	v0,12(s0)
8000bab0:	3042ffff 	andi	v0,v0,0xffff
8000bab4:	30420001 	andi	v0,v0,0x1
8000bab8:	10400007 	beqz	v0,8000bad8 <__swsetup_r+0x190>
8000babc:	00000000 	nop
8000bac0:	ae000008 	sw	zero,8(s0)
8000bac4:	8e020014 	lw	v0,20(s0)
8000bac8:	00021023 	negu	v0,v0
8000bacc:	ae020018 	sw	v0,24(s0)
8000bad0:	1000000b 	b	8000bb00 <__swsetup_r+0x1b8>
8000bad4:	00000000 	nop
8000bad8:	8602000c 	lh	v0,12(s0)
8000badc:	3042ffff 	andi	v0,v0,0xffff
8000bae0:	30420002 	andi	v0,v0,0x2
8000bae4:	14400004 	bnez	v0,8000baf8 <__swsetup_r+0x1b0>
8000bae8:	00000000 	nop
8000baec:	8e020014 	lw	v0,20(s0)
8000baf0:	10000002 	b	8000bafc <__swsetup_r+0x1b4>
8000baf4:	00000000 	nop
8000baf8:	00001025 	move	v0,zero
8000bafc:	ae020008 	sw	v0,8(s0)
8000bb00:	8e020010 	lw	v0,16(s0)
8000bb04:	1440000e 	bnez	v0,8000bb40 <__swsetup_r+0x1f8>
8000bb08:	00000000 	nop
8000bb0c:	8602000c 	lh	v0,12(s0)
8000bb10:	3042ffff 	andi	v0,v0,0xffff
8000bb14:	30420080 	andi	v0,v0,0x80
8000bb18:	10400009 	beqz	v0,8000bb40 <__swsetup_r+0x1f8>
8000bb1c:	00000000 	nop
8000bb20:	8602000c 	lh	v0,12(s0)
8000bb24:	34420040 	ori	v0,v0,0x40
8000bb28:	00021400 	sll	v0,v0,0x10
8000bb2c:	00021403 	sra	v0,v0,0x10
8000bb30:	a602000c 	sh	v0,12(s0)
8000bb34:	2402ffff 	li	v0,-1
8000bb38:	10000002 	b	8000bb44 <__swsetup_r+0x1fc>
8000bb3c:	00000000 	nop
8000bb40:	00001025 	move	v0,zero
8000bb44:	03c0e825 	move	sp,s8
8000bb48:	dfbf0038 	ld	ra,56(sp)
8000bb4c:	dfbe0030 	ld	s8,48(sp)
8000bb50:	dfb00028 	ld	s0,40(sp)
8000bb54:	27bd0040 	addiu	sp,sp,64
8000bb58:	03e00008 	jr	ra
8000bb5c:	00000000 	nop

8000bb60 <_close_r>:
8000bb60:	27bdffc8 	addiu	sp,sp,-56
8000bb64:	ffbf0030 	sd	ra,48(sp)
8000bb68:	ffbe0028 	sd	s8,40(sp)
8000bb6c:	03a0f025 	move	s8,sp
8000bb70:	afc40038 	sw	a0,56(s8)
8000bb74:	afc50040 	sw	a1,64(s8)
8000bb78:	af808334 	sw	zero,-31948(gp)
8000bb7c:	8fc40040 	lw	a0,64(s8)
8000bb80:	0c006710 	jal	80019c40 <close>
8000bb84:	00000000 	nop
8000bb88:	afc20020 	sw	v0,32(s8)
8000bb8c:	8fc30020 	lw	v1,32(s8)
8000bb90:	2402ffff 	li	v0,-1
8000bb94:	14620007 	bne	v1,v0,8000bbb4 <_close_r+0x54>
8000bb98:	00000000 	nop
8000bb9c:	8f828334 	lw	v0,-31948(gp)
8000bba0:	10400004 	beqz	v0,8000bbb4 <_close_r+0x54>
8000bba4:	00000000 	nop
8000bba8:	8f838334 	lw	v1,-31948(gp)
8000bbac:	8fc20038 	lw	v0,56(s8)
8000bbb0:	ac430000 	sw	v1,0(v0)
8000bbb4:	8fc20020 	lw	v0,32(s8)
8000bbb8:	03c0e825 	move	sp,s8
8000bbbc:	dfbf0030 	ld	ra,48(sp)
8000bbc0:	dfbe0028 	ld	s8,40(sp)
8000bbc4:	27bd0038 	addiu	sp,sp,56
8000bbc8:	03e00008 	jr	ra
8000bbcc:	00000000 	nop

8000bbd0 <div>:
8000bbd0:	27bdfff0 	addiu	sp,sp,-16
8000bbd4:	ffbe0008 	sd	s8,8(sp)
8000bbd8:	03a0f025 	move	s8,sp
8000bbdc:	afc40010 	sw	a0,16(s8)
8000bbe0:	afc50018 	sw	a1,24(s8)
8000bbe4:	afc60020 	sw	a2,32(s8)
8000bbe8:	8fc30018 	lw	v1,24(s8)
8000bbec:	8fc20020 	lw	v0,32(s8)
8000bbf0:	0062001a 	div	zero,v1,v0
8000bbf4:	004001f4 	teq	v0,zero,0x7
8000bbf8:	00001010 	mfhi	v0
8000bbfc:	00001012 	mflo	v0
8000bc00:	afc20000 	sw	v0,0(s8)
8000bc04:	8fc30018 	lw	v1,24(s8)
8000bc08:	8fc20020 	lw	v0,32(s8)
8000bc0c:	0062001a 	div	zero,v1,v0
8000bc10:	004001f4 	teq	v0,zero,0x7
8000bc14:	00001010 	mfhi	v0
8000bc18:	afc20004 	sw	v0,4(s8)
8000bc1c:	8fc20018 	lw	v0,24(s8)
8000bc20:	0440000d 	bltz	v0,8000bc58 <div+0x88>
8000bc24:	00000000 	nop
8000bc28:	8fc20004 	lw	v0,4(s8)
8000bc2c:	0441000a 	bgez	v0,8000bc58 <div+0x88>
8000bc30:	00000000 	nop
8000bc34:	8fc20000 	lw	v0,0(s8)
8000bc38:	24420001 	addiu	v0,v0,1
8000bc3c:	afc20000 	sw	v0,0(s8)
8000bc40:	8fc30004 	lw	v1,4(s8)
8000bc44:	8fc20020 	lw	v0,32(s8)
8000bc48:	00621023 	subu	v0,v1,v0
8000bc4c:	afc20004 	sw	v0,4(s8)
8000bc50:	1000000e 	b	8000bc8c <div+0xbc>
8000bc54:	00000000 	nop
8000bc58:	8fc20018 	lw	v0,24(s8)
8000bc5c:	0441000b 	bgez	v0,8000bc8c <div+0xbc>
8000bc60:	00000000 	nop
8000bc64:	8fc20004 	lw	v0,4(s8)
8000bc68:	18400008 	blez	v0,8000bc8c <div+0xbc>
8000bc6c:	00000000 	nop
8000bc70:	8fc20000 	lw	v0,0(s8)
8000bc74:	2442ffff 	addiu	v0,v0,-1
8000bc78:	afc20000 	sw	v0,0(s8)
8000bc7c:	8fc30004 	lw	v1,4(s8)
8000bc80:	8fc20020 	lw	v0,32(s8)
8000bc84:	00621021 	addu	v0,v1,v0
8000bc88:	afc20004 	sw	v0,4(s8)
8000bc8c:	8fc20010 	lw	v0,16(s8)
8000bc90:	dfc30000 	ld	v1,0(s8)
8000bc94:	b0430000 	sdl	v1,0(v0)
8000bc98:	b4430007 	sdr	v1,7(v0)
8000bc9c:	8fc20010 	lw	v0,16(s8)
8000bca0:	03c0e825 	move	sp,s8
8000bca4:	dfbe0008 	ld	s8,8(sp)
8000bca8:	27bd0010 	addiu	sp,sp,16
8000bcac:	03e00008 	jr	ra
8000bcb0:	00000000 	nop

8000bcb4 <quorem>:
8000bcb4:	27bdff98 	addiu	sp,sp,-104
8000bcb8:	ffbf0060 	sd	ra,96(sp)
8000bcbc:	ffbe0058 	sd	s8,88(sp)
8000bcc0:	03a0f025 	move	s8,sp
8000bcc4:	afc40068 	sw	a0,104(s8)
8000bcc8:	afc50070 	sw	a1,112(s8)
8000bccc:	8fc20070 	lw	v0,112(s8)
8000bcd0:	8c420010 	lw	v0,16(v0)
8000bcd4:	afc20020 	sw	v0,32(s8)
8000bcd8:	8fc20068 	lw	v0,104(s8)
8000bcdc:	8c420010 	lw	v0,16(v0)
8000bce0:	8fc30020 	lw	v1,32(s8)
8000bce4:	0043102a 	slt	v0,v0,v1
8000bce8:	10400004 	beqz	v0,8000bcfc <quorem+0x48>
8000bcec:	00000000 	nop
8000bcf0:	00001025 	move	v0,zero
8000bcf4:	100000ea 	b	8000c0a0 <quorem+0x3ec>
8000bcf8:	00000000 	nop
8000bcfc:	8fc20070 	lw	v0,112(s8)
8000bd00:	24420014 	addiu	v0,v0,20
8000bd04:	afc20038 	sw	v0,56(s8)
8000bd08:	8fc20020 	lw	v0,32(s8)
8000bd0c:	2442ffff 	addiu	v0,v0,-1
8000bd10:	afc20020 	sw	v0,32(s8)
8000bd14:	8fc20020 	lw	v0,32(s8)
8000bd18:	00021080 	sll	v0,v0,0x2
8000bd1c:	8fc30038 	lw	v1,56(s8)
8000bd20:	00621021 	addu	v0,v1,v0
8000bd24:	afc2003c 	sw	v0,60(s8)
8000bd28:	8fc20068 	lw	v0,104(s8)
8000bd2c:	24420014 	addiu	v0,v0,20
8000bd30:	afc20030 	sw	v0,48(s8)
8000bd34:	8fc20020 	lw	v0,32(s8)
8000bd38:	00021080 	sll	v0,v0,0x2
8000bd3c:	8fc30030 	lw	v1,48(s8)
8000bd40:	00621021 	addu	v0,v1,v0
8000bd44:	afc20034 	sw	v0,52(s8)
8000bd48:	8fc20034 	lw	v0,52(s8)
8000bd4c:	8c430000 	lw	v1,0(v0)
8000bd50:	8fc2003c 	lw	v0,60(s8)
8000bd54:	8c420000 	lw	v0,0(v0)
8000bd58:	24420001 	addiu	v0,v0,1
8000bd5c:	0062001b 	divu	zero,v1,v0
8000bd60:	004001f4 	teq	v0,zero,0x7
8000bd64:	00001010 	mfhi	v0
8000bd68:	00001012 	mflo	v0
8000bd6c:	afc2002c 	sw	v0,44(s8)
8000bd70:	8fc2002c 	lw	v0,44(s8)
8000bd74:	1040005e 	beqz	v0,8000bef0 <quorem+0x23c>
8000bd78:	00000000 	nop
8000bd7c:	afc00024 	sw	zero,36(s8)
8000bd80:	afc00028 	sw	zero,40(s8)
8000bd84:	8fc20038 	lw	v0,56(s8)
8000bd88:	24430004 	addiu	v1,v0,4
8000bd8c:	afc30038 	sw	v1,56(s8)
8000bd90:	8c420000 	lw	v0,0(v0)
8000bd94:	afc20040 	sw	v0,64(s8)
8000bd98:	8fc20040 	lw	v0,64(s8)
8000bd9c:	3043ffff 	andi	v1,v0,0xffff
8000bda0:	8fc2002c 	lw	v0,44(s8)
8000bda4:	00620018 	mult	v1,v0
8000bda8:	8fc20028 	lw	v0,40(s8)
8000bdac:	00001812 	mflo	v1
8000bdb0:	00431021 	addu	v0,v0,v1
8000bdb4:	afc20044 	sw	v0,68(s8)
8000bdb8:	8fc20040 	lw	v0,64(s8)
8000bdbc:	00021c02 	srl	v1,v0,0x10
8000bdc0:	8fc2002c 	lw	v0,44(s8)
8000bdc4:	00620018 	mult	v1,v0
8000bdc8:	8fc20044 	lw	v0,68(s8)
8000bdcc:	00021402 	srl	v0,v0,0x10
8000bdd0:	00001812 	mflo	v1
8000bdd4:	00621021 	addu	v0,v1,v0
8000bdd8:	afc20048 	sw	v0,72(s8)
8000bddc:	8fc20048 	lw	v0,72(s8)
8000bde0:	00021402 	srl	v0,v0,0x10
8000bde4:	afc20028 	sw	v0,40(s8)
8000bde8:	8fc20030 	lw	v0,48(s8)
8000bdec:	8c420000 	lw	v0,0(v0)
8000bdf0:	3043ffff 	andi	v1,v0,0xffff
8000bdf4:	8fc20044 	lw	v0,68(s8)
8000bdf8:	3042ffff 	andi	v0,v0,0xffff
8000bdfc:	00621823 	subu	v1,v1,v0
8000be00:	8fc20024 	lw	v0,36(s8)
8000be04:	00621021 	addu	v0,v1,v0
8000be08:	afc2004c 	sw	v0,76(s8)
8000be0c:	8fc2004c 	lw	v0,76(s8)
8000be10:	00021403 	sra	v0,v0,0x10
8000be14:	afc20024 	sw	v0,36(s8)
8000be18:	8fc20030 	lw	v0,48(s8)
8000be1c:	8c420000 	lw	v0,0(v0)
8000be20:	00021c02 	srl	v1,v0,0x10
8000be24:	8fc20048 	lw	v0,72(s8)
8000be28:	3042ffff 	andi	v0,v0,0xffff
8000be2c:	00621823 	subu	v1,v1,v0
8000be30:	8fc20024 	lw	v0,36(s8)
8000be34:	00621021 	addu	v0,v1,v0
8000be38:	afc20050 	sw	v0,80(s8)
8000be3c:	8fc20050 	lw	v0,80(s8)
8000be40:	00021403 	sra	v0,v0,0x10
8000be44:	afc20024 	sw	v0,36(s8)
8000be48:	8fc20050 	lw	v0,80(s8)
8000be4c:	00021c00 	sll	v1,v0,0x10
8000be50:	8fc2004c 	lw	v0,76(s8)
8000be54:	3042ffff 	andi	v0,v0,0xffff
8000be58:	00622025 	or	a0,v1,v0
8000be5c:	8fc20030 	lw	v0,48(s8)
8000be60:	24430004 	addiu	v1,v0,4
8000be64:	afc30030 	sw	v1,48(s8)
8000be68:	00801825 	move	v1,a0
8000be6c:	ac430000 	sw	v1,0(v0)
8000be70:	8fc30038 	lw	v1,56(s8)
8000be74:	8fc2003c 	lw	v0,60(s8)
8000be78:	0043102b 	sltu	v0,v0,v1
8000be7c:	1040ffc1 	beqz	v0,8000bd84 <quorem+0xd0>
8000be80:	00000000 	nop
8000be84:	8fc20034 	lw	v0,52(s8)
8000be88:	8c420000 	lw	v0,0(v0)
8000be8c:	14400018 	bnez	v0,8000bef0 <quorem+0x23c>
8000be90:	00000000 	nop
8000be94:	8fc20068 	lw	v0,104(s8)
8000be98:	24420014 	addiu	v0,v0,20
8000be9c:	afc20030 	sw	v0,48(s8)
8000bea0:	10000004 	b	8000beb4 <quorem+0x200>
8000bea4:	00000000 	nop
8000bea8:	8fc20020 	lw	v0,32(s8)
8000beac:	2442ffff 	addiu	v0,v0,-1
8000beb0:	afc20020 	sw	v0,32(s8)
8000beb4:	8fc20034 	lw	v0,52(s8)
8000beb8:	2442fffc 	addiu	v0,v0,-4
8000bebc:	afc20034 	sw	v0,52(s8)
8000bec0:	8fc30034 	lw	v1,52(s8)
8000bec4:	8fc20030 	lw	v0,48(s8)
8000bec8:	0043102b 	sltu	v0,v0,v1
8000becc:	10400005 	beqz	v0,8000bee4 <quorem+0x230>
8000bed0:	00000000 	nop
8000bed4:	8fc20034 	lw	v0,52(s8)
8000bed8:	8c420000 	lw	v0,0(v0)
8000bedc:	1040fff2 	beqz	v0,8000bea8 <quorem+0x1f4>
8000bee0:	00000000 	nop
8000bee4:	8fc20068 	lw	v0,104(s8)
8000bee8:	8fc30020 	lw	v1,32(s8)
8000beec:	ac430010 	sw	v1,16(v0)
8000bef0:	8fc50070 	lw	a1,112(s8)
8000bef4:	8fc40068 	lw	a0,104(s8)
8000bef8:	0c003f83 	jal	8000fe0c <__mcmp>
8000befc:	00000000 	nop
8000bf00:	04400066 	bltz	v0,8000c09c <quorem+0x3e8>
8000bf04:	00000000 	nop
8000bf08:	8fc2002c 	lw	v0,44(s8)
8000bf0c:	24420001 	addiu	v0,v0,1
8000bf10:	afc2002c 	sw	v0,44(s8)
8000bf14:	afc00024 	sw	zero,36(s8)
8000bf18:	afc00028 	sw	zero,40(s8)
8000bf1c:	8fc20068 	lw	v0,104(s8)
8000bf20:	24420014 	addiu	v0,v0,20
8000bf24:	afc20030 	sw	v0,48(s8)
8000bf28:	8fc20070 	lw	v0,112(s8)
8000bf2c:	24420014 	addiu	v0,v0,20
8000bf30:	afc20038 	sw	v0,56(s8)
8000bf34:	8fc20038 	lw	v0,56(s8)
8000bf38:	24430004 	addiu	v1,v0,4
8000bf3c:	afc30038 	sw	v1,56(s8)
8000bf40:	8c420000 	lw	v0,0(v0)
8000bf44:	afc20040 	sw	v0,64(s8)
8000bf48:	8fc20040 	lw	v0,64(s8)
8000bf4c:	3042ffff 	andi	v0,v0,0xffff
8000bf50:	8fc30028 	lw	v1,40(s8)
8000bf54:	00621021 	addu	v0,v1,v0
8000bf58:	afc20044 	sw	v0,68(s8)
8000bf5c:	8fc20040 	lw	v0,64(s8)
8000bf60:	00021c02 	srl	v1,v0,0x10
8000bf64:	8fc20044 	lw	v0,68(s8)
8000bf68:	00021402 	srl	v0,v0,0x10
8000bf6c:	00621021 	addu	v0,v1,v0
8000bf70:	afc20048 	sw	v0,72(s8)
8000bf74:	8fc20048 	lw	v0,72(s8)
8000bf78:	00021402 	srl	v0,v0,0x10
8000bf7c:	afc20028 	sw	v0,40(s8)
8000bf80:	8fc20030 	lw	v0,48(s8)
8000bf84:	8c420000 	lw	v0,0(v0)
8000bf88:	3043ffff 	andi	v1,v0,0xffff
8000bf8c:	8fc20044 	lw	v0,68(s8)
8000bf90:	3042ffff 	andi	v0,v0,0xffff
8000bf94:	00621823 	subu	v1,v1,v0
8000bf98:	8fc20024 	lw	v0,36(s8)
8000bf9c:	00621021 	addu	v0,v1,v0
8000bfa0:	afc2004c 	sw	v0,76(s8)
8000bfa4:	8fc2004c 	lw	v0,76(s8)
8000bfa8:	00021403 	sra	v0,v0,0x10
8000bfac:	afc20024 	sw	v0,36(s8)
8000bfb0:	8fc20030 	lw	v0,48(s8)
8000bfb4:	8c420000 	lw	v0,0(v0)
8000bfb8:	00021c02 	srl	v1,v0,0x10
8000bfbc:	8fc20048 	lw	v0,72(s8)
8000bfc0:	3042ffff 	andi	v0,v0,0xffff
8000bfc4:	00621823 	subu	v1,v1,v0
8000bfc8:	8fc20024 	lw	v0,36(s8)
8000bfcc:	00621021 	addu	v0,v1,v0
8000bfd0:	afc20050 	sw	v0,80(s8)
8000bfd4:	8fc20050 	lw	v0,80(s8)
8000bfd8:	00021403 	sra	v0,v0,0x10
8000bfdc:	afc20024 	sw	v0,36(s8)
8000bfe0:	8fc20050 	lw	v0,80(s8)
8000bfe4:	00021c00 	sll	v1,v0,0x10
8000bfe8:	8fc2004c 	lw	v0,76(s8)
8000bfec:	3042ffff 	andi	v0,v0,0xffff
8000bff0:	00622025 	or	a0,v1,v0
8000bff4:	8fc20030 	lw	v0,48(s8)
8000bff8:	24430004 	addiu	v1,v0,4
8000bffc:	afc30030 	sw	v1,48(s8)
8000c000:	00801825 	move	v1,a0
8000c004:	ac430000 	sw	v1,0(v0)
8000c008:	8fc30038 	lw	v1,56(s8)
8000c00c:	8fc2003c 	lw	v0,60(s8)
8000c010:	0043102b 	sltu	v0,v0,v1
8000c014:	1040ffc7 	beqz	v0,8000bf34 <quorem+0x280>
8000c018:	00000000 	nop
8000c01c:	8fc20068 	lw	v0,104(s8)
8000c020:	24420014 	addiu	v0,v0,20
8000c024:	afc20030 	sw	v0,48(s8)
8000c028:	8fc20020 	lw	v0,32(s8)
8000c02c:	00021080 	sll	v0,v0,0x2
8000c030:	8fc30030 	lw	v1,48(s8)
8000c034:	00621021 	addu	v0,v1,v0
8000c038:	afc20034 	sw	v0,52(s8)
8000c03c:	8fc20034 	lw	v0,52(s8)
8000c040:	8c420000 	lw	v0,0(v0)
8000c044:	14400015 	bnez	v0,8000c09c <quorem+0x3e8>
8000c048:	00000000 	nop
8000c04c:	10000004 	b	8000c060 <quorem+0x3ac>
8000c050:	00000000 	nop
8000c054:	8fc20020 	lw	v0,32(s8)
8000c058:	2442ffff 	addiu	v0,v0,-1
8000c05c:	afc20020 	sw	v0,32(s8)
8000c060:	8fc20034 	lw	v0,52(s8)
8000c064:	2442fffc 	addiu	v0,v0,-4
8000c068:	afc20034 	sw	v0,52(s8)
8000c06c:	8fc30034 	lw	v1,52(s8)
8000c070:	8fc20030 	lw	v0,48(s8)
8000c074:	0043102b 	sltu	v0,v0,v1
8000c078:	10400005 	beqz	v0,8000c090 <quorem+0x3dc>
8000c07c:	00000000 	nop
8000c080:	8fc20034 	lw	v0,52(s8)
8000c084:	8c420000 	lw	v0,0(v0)
8000c088:	1040fff2 	beqz	v0,8000c054 <quorem+0x3a0>
8000c08c:	00000000 	nop
8000c090:	8fc20068 	lw	v0,104(s8)
8000c094:	8fc30020 	lw	v1,32(s8)
8000c098:	ac430010 	sw	v1,16(v0)
8000c09c:	8fc2002c 	lw	v0,44(s8)
8000c0a0:	03c0e825 	move	sp,s8
8000c0a4:	dfbf0060 	ld	ra,96(sp)
8000c0a8:	dfbe0058 	ld	s8,88(sp)
8000c0ac:	27bd0068 	addiu	sp,sp,104
8000c0b0:	03e00008 	jr	ra
8000c0b4:	00000000 	nop

8000c0b8 <_dtoa_r>:
8000c0b8:	27bdff20 	addiu	sp,sp,-224
8000c0bc:	ffbf00d8 	sd	ra,216(sp)
8000c0c0:	ffbe00d0 	sd	s8,208(sp)
8000c0c4:	03a0f025 	move	s8,sp
8000c0c8:	afc400e0 	sw	a0,224(s8)
8000c0cc:	ffc500e8 	sd	a1,232(s8)
8000c0d0:	afc600f0 	sw	a2,240(s8)
8000c0d4:	afc700f8 	sw	a3,248(s8)
8000c0d8:	afc0006c 	sw	zero,108(s8)
8000c0dc:	d7c000e8 	ldc1	$f0,232(s8)
8000c0e0:	f7c000b8 	sdc1	$f0,184(s8)
8000c0e4:	8fc200e0 	lw	v0,224(s8)
8000c0e8:	8c420040 	lw	v0,64(v0)
8000c0ec:	10400015 	beqz	v0,8000c144 <_dtoa_r+0x8c>
8000c0f0:	00000000 	nop
8000c0f4:	8fc200e0 	lw	v0,224(s8)
8000c0f8:	8c420040 	lw	v0,64(v0)
8000c0fc:	8fc300e0 	lw	v1,224(s8)
8000c100:	8c630044 	lw	v1,68(v1)
8000c104:	ac430004 	sw	v1,4(v0)
8000c108:	8fc200e0 	lw	v0,224(s8)
8000c10c:	8c430044 	lw	v1,68(v0)
8000c110:	8fc200e0 	lw	v0,224(s8)
8000c114:	8c420040 	lw	v0,64(v0)
8000c118:	24040001 	li	a0,1
8000c11c:	00641804 	sllv	v1,a0,v1
8000c120:	ac430008 	sw	v1,8(v0)
8000c124:	8fc200e0 	lw	v0,224(s8)
8000c128:	8c420040 	lw	v0,64(v0)
8000c12c:	00402825 	move	a1,v0
8000c130:	8fc400e0 	lw	a0,224(s8)
8000c134:	0c003bae 	jal	8000eeb8 <_Bfree>
8000c138:	00000000 	nop
8000c13c:	8fc200e0 	lw	v0,224(s8)
8000c140:	ac400040 	sw	zero,64(v0)
8000c144:	8fc200b8 	lw	v0,184(s8)
8000c148:	0441000b 	bgez	v0,8000c178 <_dtoa_r+0xc0>
8000c14c:	00000000 	nop
8000c150:	8fc2010c 	lw	v0,268(s8)
8000c154:	24030001 	li	v1,1
8000c158:	ac430000 	sw	v1,0(v0)
8000c15c:	8fc300b8 	lw	v1,184(s8)
8000c160:	3c027fff 	lui	v0,0x7fff
8000c164:	3442ffff 	ori	v0,v0,0xffff
8000c168:	00621024 	and	v0,v1,v0
8000c16c:	afc200b8 	sw	v0,184(s8)
8000c170:	10000003 	b	8000c180 <_dtoa_r+0xc8>
8000c174:	00000000 	nop
8000c178:	8fc2010c 	lw	v0,268(s8)
8000c17c:	ac400000 	sw	zero,0(v0)
8000c180:	8fc300b8 	lw	v1,184(s8)
8000c184:	3c027ff0 	lui	v0,0x7ff0
8000c188:	00621824 	and	v1,v1,v0
8000c18c:	3c027ff0 	lui	v0,0x7ff0
8000c190:	14620027 	bne	v1,v0,8000c230 <_dtoa_r+0x178>
8000c194:	00000000 	nop
8000c198:	8fc20104 	lw	v0,260(s8)
8000c19c:	2403270f 	li	v1,9999
8000c1a0:	ac430000 	sw	v1,0(v0)
8000c1a4:	8fc200bc 	lw	v0,188(s8)
8000c1a8:	1440000b 	bnez	v0,8000c1d8 <_dtoa_r+0x120>
8000c1ac:	00000000 	nop
8000c1b0:	8fc300b8 	lw	v1,184(s8)
8000c1b4:	3c02000f 	lui	v0,0xf
8000c1b8:	3442ffff 	ori	v0,v0,0xffff
8000c1bc:	00621024 	and	v0,v1,v0
8000c1c0:	14400005 	bnez	v0,8000c1d8 <_dtoa_r+0x120>
8000c1c4:	00000000 	nop
8000c1c8:	3c028002 	lui	v0,0x8002
8000c1cc:	2442a6f0 	addiu	v0,v0,-22800
8000c1d0:	10000003 	b	8000c1e0 <_dtoa_r+0x128>
8000c1d4:	00000000 	nop
8000c1d8:	3c028002 	lui	v0,0x8002
8000c1dc:	2442a700 	addiu	v0,v0,-22784
8000c1e0:	afc20080 	sw	v0,128(s8)
8000c1e4:	8fc20114 	lw	v0,276(s8)
8000c1e8:	1040000e 	beqz	v0,8000c224 <_dtoa_r+0x16c>
8000c1ec:	00000000 	nop
8000c1f0:	8fc20080 	lw	v0,128(s8)
8000c1f4:	24420003 	addiu	v0,v0,3
8000c1f8:	80420000 	lb	v0,0(v0)
8000c1fc:	10400005 	beqz	v0,8000c214 <_dtoa_r+0x15c>
8000c200:	00000000 	nop
8000c204:	8fc20080 	lw	v0,128(s8)
8000c208:	24420008 	addiu	v0,v0,8
8000c20c:	10000003 	b	8000c21c <_dtoa_r+0x164>
8000c210:	00000000 	nop
8000c214:	8fc20080 	lw	v0,128(s8)
8000c218:	24420003 	addiu	v0,v0,3
8000c21c:	8fc30114 	lw	v1,276(s8)
8000c220:	ac620000 	sw	v0,0(v1)
8000c224:	8fc20080 	lw	v0,128(s8)
8000c228:	10000639 	b	8000db10 <_dtoa_r+0x1a58>
8000c22c:	00000000 	nop
8000c230:	d7c000b8 	ldc1	$f0,184(s8)
8000c234:	44a00800 	dmtc1	zero,$f1
8000c238:	00000000 	nop
8000c23c:	46210032 	c.eq.d	$f0,$f1
8000c240:	00000000 	nop
8000c244:	45000011 	bc1f	8000c28c <_dtoa_r+0x1d4>
8000c248:	00000000 	nop
8000c24c:	8fc20104 	lw	v0,260(s8)
8000c250:	24030001 	li	v1,1
8000c254:	ac430000 	sw	v1,0(v0)
8000c258:	3c028002 	lui	v0,0x8002
8000c25c:	2442a708 	addiu	v0,v0,-22776
8000c260:	afc20080 	sw	v0,128(s8)
8000c264:	8fc20114 	lw	v0,276(s8)
8000c268:	10400005 	beqz	v0,8000c280 <_dtoa_r+0x1c8>
8000c26c:	00000000 	nop
8000c270:	8fc20080 	lw	v0,128(s8)
8000c274:	24430001 	addiu	v1,v0,1
8000c278:	8fc20114 	lw	v0,276(s8)
8000c27c:	ac430000 	sw	v1,0(v0)
8000c280:	8fc20080 	lw	v0,128(s8)
8000c284:	10000622 	b	8000db10 <_dtoa_r+0x1a58>
8000c288:	00000000 	nop
8000c28c:	d7c000b8 	ldc1	$f0,184(s8)
8000c290:	27c300ac 	addiu	v1,s8,172
8000c294:	27c200b0 	addiu	v0,s8,176
8000c298:	00603825 	move	a3,v1
8000c29c:	00403025 	move	a2,v0
8000c2a0:	44250000 	dmfc1	a1,$f0
8000c2a4:	8fc400e0 	lw	a0,224(s8)
8000c2a8:	0c00415c 	jal	80010570 <__d2b>
8000c2ac:	00000000 	nop
8000c2b0:	afc20068 	sw	v0,104(s8)
8000c2b4:	8fc200b8 	lw	v0,184(s8)
8000c2b8:	00021502 	srl	v0,v0,0x14
8000c2bc:	304207ff 	andi	v0,v0,0x7ff
8000c2c0:	afc2002c 	sw	v0,44(s8)
8000c2c4:	8fc2002c 	lw	v0,44(s8)
8000c2c8:	10400012 	beqz	v0,8000c314 <_dtoa_r+0x25c>
8000c2cc:	00000000 	nop
8000c2d0:	d7c000b8 	ldc1	$f0,184(s8)
8000c2d4:	f7c000c0 	sdc1	$f0,192(s8)
8000c2d8:	8fc300c0 	lw	v1,192(s8)
8000c2dc:	3c02000f 	lui	v0,0xf
8000c2e0:	3442ffff 	ori	v0,v0,0xffff
8000c2e4:	00621024 	and	v0,v1,v0
8000c2e8:	afc200c0 	sw	v0,192(s8)
8000c2ec:	8fc300c0 	lw	v1,192(s8)
8000c2f0:	3c023ff0 	lui	v0,0x3ff0
8000c2f4:	00621025 	or	v0,v1,v0
8000c2f8:	afc200c0 	sw	v0,192(s8)
8000c2fc:	8fc2002c 	lw	v0,44(s8)
8000c300:	2442fc01 	addiu	v0,v0,-1023
8000c304:	afc2002c 	sw	v0,44(s8)
8000c308:	afc00064 	sw	zero,100(s8)
8000c30c:	1000002a 	b	8000c3b8 <_dtoa_r+0x300>
8000c310:	00000000 	nop
8000c314:	8fc300ac 	lw	v1,172(s8)
8000c318:	8fc200b0 	lw	v0,176(s8)
8000c31c:	00621021 	addu	v0,v1,v0
8000c320:	24420432 	addiu	v0,v0,1074
8000c324:	afc2002c 	sw	v0,44(s8)
8000c328:	8fc2002c 	lw	v0,44(s8)
8000c32c:	28420021 	slti	v0,v0,33
8000c330:	1440000d 	bnez	v0,8000c368 <_dtoa_r+0x2b0>
8000c334:	00000000 	nop
8000c338:	8fc300b8 	lw	v1,184(s8)
8000c33c:	24040040 	li	a0,64
8000c340:	8fc2002c 	lw	v0,44(s8)
8000c344:	00821023 	subu	v0,a0,v0
8000c348:	00431804 	sllv	v1,v1,v0
8000c34c:	8fc400bc 	lw	a0,188(s8)
8000c350:	8fc2002c 	lw	v0,44(s8)
8000c354:	2442ffe0 	addiu	v0,v0,-32
8000c358:	00441006 	srlv	v0,a0,v0
8000c35c:	00621025 	or	v0,v1,v0
8000c360:	10000006 	b	8000c37c <_dtoa_r+0x2c4>
8000c364:	00000000 	nop
8000c368:	8fc300bc 	lw	v1,188(s8)
8000c36c:	24040020 	li	a0,32
8000c370:	8fc2002c 	lw	v0,44(s8)
8000c374:	00821023 	subu	v0,a0,v0
8000c378:	00431004 	sllv	v0,v1,v0
8000c37c:	afc20084 	sw	v0,132(s8)
8000c380:	9fc20084 	lwu	v0,132(s8)
8000c384:	44a20000 	dmtc1	v0,$f0
8000c388:	00000000 	nop
8000c38c:	46a00021 	cvt.d.l	$f0,$f0
8000c390:	f7c000c0 	sdc1	$f0,192(s8)
8000c394:	8fc300c0 	lw	v1,192(s8)
8000c398:	3c02fe10 	lui	v0,0xfe10
8000c39c:	00621021 	addu	v0,v1,v0
8000c3a0:	afc200c0 	sw	v0,192(s8)
8000c3a4:	8fc2002c 	lw	v0,44(s8)
8000c3a8:	2442fbcd 	addiu	v0,v0,-1075
8000c3ac:	afc2002c 	sw	v0,44(s8)
8000c3b0:	24020001 	li	v0,1
8000c3b4:	afc20064 	sw	v0,100(s8)
8000c3b8:	d7c100c0 	ldc1	$f1,192(s8)
8000c3bc:	d7808048 	ldc1	$f0,-32696(gp)
8000c3c0:	46200841 	sub.d	$f1,$f1,$f0
8000c3c4:	d7808050 	ldc1	$f0,-32688(gp)
8000c3c8:	46200842 	mul.d	$f1,$f1,$f0
8000c3cc:	d7808058 	ldc1	$f0,-32680(gp)
8000c3d0:	46200840 	add.d	$f1,$f1,$f0
8000c3d4:	8fc2002c 	lw	v0,44(s8)
8000c3d8:	44820000 	mtc1	v0,$f0
8000c3dc:	00000000 	nop
8000c3e0:	468000a1 	cvt.d.w	$f2,$f0
8000c3e4:	d7808060 	ldc1	$f0,-32672(gp)
8000c3e8:	46201002 	mul.d	$f0,$f2,$f0
8000c3ec:	46200800 	add.d	$f0,$f1,$f0
8000c3f0:	f7c00078 	sdc1	$f0,120(s8)
8000c3f4:	d7c00078 	ldc1	$f0,120(s8)
8000c3f8:	4620000d 	trunc.w.d	$f0,$f0
8000c3fc:	44020000 	mfc1	v0,$f0
8000c400:	00000000 	nop
8000c404:	afc20040 	sw	v0,64(s8)
8000c408:	d7c00078 	ldc1	$f0,120(s8)
8000c40c:	44a00800 	dmtc1	zero,$f1
8000c410:	00000000 	nop
8000c414:	4621003c 	c.lt.d	$f0,$f1
8000c418:	00000000 	nop
8000c41c:	4500000d 	bc1f	8000c454 <_dtoa_r+0x39c>
8000c420:	00000000 	nop
8000c424:	8fc20040 	lw	v0,64(s8)
8000c428:	44820000 	mtc1	v0,$f0
8000c42c:	00000000 	nop
8000c430:	46800021 	cvt.d.w	$f0,$f0
8000c434:	d7c10078 	ldc1	$f1,120(s8)
8000c438:	46200832 	c.eq.d	$f1,$f0
8000c43c:	00000000 	nop
8000c440:	45010004 	bc1t	8000c454 <_dtoa_r+0x39c>
8000c444:	00000000 	nop
8000c448:	8fc20040 	lw	v0,64(s8)
8000c44c:	2442ffff 	addiu	v0,v0,-1
8000c450:	afc20040 	sw	v0,64(s8)
8000c454:	24020001 	li	v0,1
8000c458:	afc20044 	sw	v0,68(s8)
8000c45c:	8fc20040 	lw	v0,64(s8)
8000c460:	04400014 	bltz	v0,8000c4b4 <_dtoa_r+0x3fc>
8000c464:	00000000 	nop
8000c468:	8fc20040 	lw	v0,64(s8)
8000c46c:	28420017 	slti	v0,v0,23
8000c470:	10400010 	beqz	v0,8000c4b4 <_dtoa_r+0x3fc>
8000c474:	00000000 	nop
8000c478:	d7c100b8 	ldc1	$f1,184(s8)
8000c47c:	3c028002 	lui	v0,0x8002
8000c480:	8fc30040 	lw	v1,64(s8)
8000c484:	000318c0 	sll	v1,v1,0x3
8000c488:	2442a830 	addiu	v0,v0,-22480
8000c48c:	00621021 	addu	v0,v1,v0
8000c490:	d4400000 	ldc1	$f0,0(v0)
8000c494:	4620083c 	c.lt.d	$f1,$f0
8000c498:	00000000 	nop
8000c49c:	45000004 	bc1f	8000c4b0 <_dtoa_r+0x3f8>
8000c4a0:	00000000 	nop
8000c4a4:	8fc20040 	lw	v0,64(s8)
8000c4a8:	2442ffff 	addiu	v0,v0,-1
8000c4ac:	afc20040 	sw	v0,64(s8)
8000c4b0:	afc00044 	sw	zero,68(s8)
8000c4b4:	8fc300ac 	lw	v1,172(s8)
8000c4b8:	8fc2002c 	lw	v0,44(s8)
8000c4bc:	00621023 	subu	v0,v1,v0
8000c4c0:	2442ffff 	addiu	v0,v0,-1
8000c4c4:	afc2003c 	sw	v0,60(s8)
8000c4c8:	8fc2003c 	lw	v0,60(s8)
8000c4cc:	04400006 	bltz	v0,8000c4e8 <_dtoa_r+0x430>
8000c4d0:	00000000 	nop
8000c4d4:	afc00020 	sw	zero,32(s8)
8000c4d8:	8fc2003c 	lw	v0,60(s8)
8000c4dc:	afc20054 	sw	v0,84(s8)
8000c4e0:	10000005 	b	8000c4f8 <_dtoa_r+0x440>
8000c4e4:	00000000 	nop
8000c4e8:	8fc2003c 	lw	v0,60(s8)
8000c4ec:	00021023 	negu	v0,v0
8000c4f0:	afc20020 	sw	v0,32(s8)
8000c4f4:	afc00054 	sw	zero,84(s8)
8000c4f8:	8fc20040 	lw	v0,64(s8)
8000c4fc:	0440000a 	bltz	v0,8000c528 <_dtoa_r+0x470>
8000c500:	00000000 	nop
8000c504:	afc00024 	sw	zero,36(s8)
8000c508:	8fc20040 	lw	v0,64(s8)
8000c50c:	afc20058 	sw	v0,88(s8)
8000c510:	8fc30054 	lw	v1,84(s8)
8000c514:	8fc20040 	lw	v0,64(s8)
8000c518:	00621021 	addu	v0,v1,v0
8000c51c:	afc20054 	sw	v0,84(s8)
8000c520:	10000009 	b	8000c548 <_dtoa_r+0x490>
8000c524:	00000000 	nop
8000c528:	8fc30020 	lw	v1,32(s8)
8000c52c:	8fc20040 	lw	v0,64(s8)
8000c530:	00621023 	subu	v0,v1,v0
8000c534:	afc20020 	sw	v0,32(s8)
8000c538:	8fc20040 	lw	v0,64(s8)
8000c53c:	00021023 	negu	v0,v0
8000c540:	afc20024 	sw	v0,36(s8)
8000c544:	afc00058 	sw	zero,88(s8)
8000c548:	8fc200f0 	lw	v0,240(s8)
8000c54c:	04400005 	bltz	v0,8000c564 <_dtoa_r+0x4ac>
8000c550:	00000000 	nop
8000c554:	8fc200f0 	lw	v0,240(s8)
8000c558:	2842000a 	slti	v0,v0,10
8000c55c:	14400002 	bnez	v0,8000c568 <_dtoa_r+0x4b0>
8000c560:	00000000 	nop
8000c564:	afc000f0 	sw	zero,240(s8)
8000c568:	24020001 	li	v0,1
8000c56c:	afc20060 	sw	v0,96(s8)
8000c570:	8fc200f0 	lw	v0,240(s8)
8000c574:	28420006 	slti	v0,v0,6
8000c578:	14400005 	bnez	v0,8000c590 <_dtoa_r+0x4d8>
8000c57c:	00000000 	nop
8000c580:	8fc200f0 	lw	v0,240(s8)
8000c584:	2442fffc 	addiu	v0,v0,-4
8000c588:	afc200f0 	sw	v0,240(s8)
8000c58c:	afc00060 	sw	zero,96(s8)
8000c590:	24020001 	li	v0,1
8000c594:	afc20048 	sw	v0,72(s8)
8000c598:	2402ffff 	li	v0,-1
8000c59c:	afc20038 	sw	v0,56(s8)
8000c5a0:	8fc20038 	lw	v0,56(s8)
8000c5a4:	afc20034 	sw	v0,52(s8)
8000c5a8:	8fc200f0 	lw	v0,240(s8)
8000c5ac:	2c420006 	sltiu	v0,v0,6
8000c5b0:	1040002c 	beqz	v0,8000c664 <_dtoa_r+0x5ac>
8000c5b4:	00000000 	nop
8000c5b8:	8fc200f0 	lw	v0,240(s8)
8000c5bc:	00021880 	sll	v1,v0,0x2
8000c5c0:	3c028002 	lui	v0,0x8002
8000c5c4:	2442a750 	addiu	v0,v0,-22704
8000c5c8:	00621021 	addu	v0,v1,v0
8000c5cc:	8c420000 	lw	v0,0(v0)
8000c5d0:	00400008 	jr	v0
8000c5d4:	00000000 	nop
8000c5d8:	24020012 	li	v0,18
8000c5dc:	afc2002c 	sw	v0,44(s8)
8000c5e0:	afc000f8 	sw	zero,248(s8)
8000c5e4:	1000001f 	b	8000c664 <_dtoa_r+0x5ac>
8000c5e8:	00000000 	nop
8000c5ec:	afc00048 	sw	zero,72(s8)
8000c5f0:	8fc200f8 	lw	v0,248(s8)
8000c5f4:	1c400003 	bgtz	v0,8000c604 <_dtoa_r+0x54c>
8000c5f8:	00000000 	nop
8000c5fc:	24020001 	li	v0,1
8000c600:	afc200f8 	sw	v0,248(s8)
8000c604:	8fc200f8 	lw	v0,248(s8)
8000c608:	afc2002c 	sw	v0,44(s8)
8000c60c:	8fc2002c 	lw	v0,44(s8)
8000c610:	afc20038 	sw	v0,56(s8)
8000c614:	8fc20038 	lw	v0,56(s8)
8000c618:	afc20034 	sw	v0,52(s8)
8000c61c:	10000011 	b	8000c664 <_dtoa_r+0x5ac>
8000c620:	00000000 	nop
8000c624:	afc00048 	sw	zero,72(s8)
8000c628:	8fc300f8 	lw	v1,248(s8)
8000c62c:	8fc20040 	lw	v0,64(s8)
8000c630:	00621021 	addu	v0,v1,v0
8000c634:	24420001 	addiu	v0,v0,1
8000c638:	afc2002c 	sw	v0,44(s8)
8000c63c:	8fc2002c 	lw	v0,44(s8)
8000c640:	afc20034 	sw	v0,52(s8)
8000c644:	8fc2002c 	lw	v0,44(s8)
8000c648:	2442ffff 	addiu	v0,v0,-1
8000c64c:	afc20038 	sw	v0,56(s8)
8000c650:	8fc2002c 	lw	v0,44(s8)
8000c654:	1c400003 	bgtz	v0,8000c664 <_dtoa_r+0x5ac>
8000c658:	00000000 	nop
8000c65c:	24020001 	li	v0,1
8000c660:	afc2002c 	sw	v0,44(s8)
8000c664:	24020004 	li	v0,4
8000c668:	afc2003c 	sw	v0,60(s8)
8000c66c:	8fc200e0 	lw	v0,224(s8)
8000c670:	ac400044 	sw	zero,68(v0)
8000c674:	10000009 	b	8000c69c <_dtoa_r+0x5e4>
8000c678:	00000000 	nop
8000c67c:	8fc200e0 	lw	v0,224(s8)
8000c680:	8c420044 	lw	v0,68(v0)
8000c684:	24430001 	addiu	v1,v0,1
8000c688:	8fc200e0 	lw	v0,224(s8)
8000c68c:	ac430044 	sw	v1,68(v0)
8000c690:	8fc2003c 	lw	v0,60(s8)
8000c694:	00021040 	sll	v0,v0,0x1
8000c698:	afc2003c 	sw	v0,60(s8)
8000c69c:	8fc2003c 	lw	v0,60(s8)
8000c6a0:	24430014 	addiu	v1,v0,20
8000c6a4:	8fc2002c 	lw	v0,44(s8)
8000c6a8:	0043102b 	sltu	v0,v0,v1
8000c6ac:	1040fff3 	beqz	v0,8000c67c <_dtoa_r+0x5c4>
8000c6b0:	00000000 	nop
8000c6b4:	8fc200e0 	lw	v0,224(s8)
8000c6b8:	8c420044 	lw	v0,68(v0)
8000c6bc:	00402825 	move	a1,v0
8000c6c0:	8fc400e0 	lw	a0,224(s8)
8000c6c4:	0c003b5b 	jal	8000ed6c <_Balloc>
8000c6c8:	00000000 	nop
8000c6cc:	afc20088 	sw	v0,136(s8)
8000c6d0:	8fc20088 	lw	v0,136(s8)
8000c6d4:	14400009 	bnez	v0,8000c6fc <_dtoa_r+0x644>
8000c6d8:	00000000 	nop
8000c6dc:	3c028002 	lui	v0,0x8002
8000c6e0:	2447a710 	addiu	a3,v0,-22768
8000c6e4:	00003025 	move	a2,zero
8000c6e8:	240501aa 	li	a1,426
8000c6ec:	3c028002 	lui	v0,0x8002
8000c6f0:	2444a728 	addiu	a0,v0,-22744
8000c6f4:	0c0068d2 	jal	8001a348 <__assert_func>
8000c6f8:	00000000 	nop
8000c6fc:	8fc30088 	lw	v1,136(s8)
8000c700:	8fc200e0 	lw	v0,224(s8)
8000c704:	ac430040 	sw	v1,64(v0)
8000c708:	8fc200e0 	lw	v0,224(s8)
8000c70c:	8c420040 	lw	v0,64(v0)
8000c710:	afc2008c 	sw	v0,140(s8)
8000c714:	8fc2008c 	lw	v0,140(s8)
8000c718:	afc20080 	sw	v0,128(s8)
8000c71c:	8fc20034 	lw	v0,52(s8)
8000c720:	04400163 	bltz	v0,8000ccb0 <_dtoa_r+0xbf8>
8000c724:	00000000 	nop
8000c728:	8fc20034 	lw	v0,52(s8)
8000c72c:	2842000f 	slti	v0,v0,15
8000c730:	1040015f 	beqz	v0,8000ccb0 <_dtoa_r+0xbf8>
8000c734:	00000000 	nop
8000c738:	8fc20060 	lw	v0,96(s8)
8000c73c:	1040015c 	beqz	v0,8000ccb0 <_dtoa_r+0xbf8>
8000c740:	00000000 	nop
8000c744:	afc0002c 	sw	zero,44(s8)
8000c748:	d7c000b8 	ldc1	$f0,184(s8)
8000c74c:	f7c000c0 	sdc1	$f0,192(s8)
8000c750:	8fc20040 	lw	v0,64(s8)
8000c754:	afc20090 	sw	v0,144(s8)
8000c758:	8fc20034 	lw	v0,52(s8)
8000c75c:	afc20094 	sw	v0,148(s8)
8000c760:	24020002 	li	v0,2
8000c764:	afc20030 	sw	v0,48(s8)
8000c768:	8fc20040 	lw	v0,64(s8)
8000c76c:	1840003d 	blez	v0,8000c864 <_dtoa_r+0x7ac>
8000c770:	00000000 	nop
8000c774:	8fc20040 	lw	v0,64(s8)
8000c778:	3043000f 	andi	v1,v0,0xf
8000c77c:	3c028002 	lui	v0,0x8002
8000c780:	000318c0 	sll	v1,v1,0x3
8000c784:	2442a830 	addiu	v0,v0,-22480
8000c788:	00621021 	addu	v0,v1,v0
8000c78c:	d4400000 	ldc1	$f0,0(v0)
8000c790:	f7c00078 	sdc1	$f0,120(s8)
8000c794:	8fc20040 	lw	v0,64(s8)
8000c798:	00021103 	sra	v0,v0,0x4
8000c79c:	afc2003c 	sw	v0,60(s8)
8000c7a0:	8fc2003c 	lw	v0,60(s8)
8000c7a4:	30420010 	andi	v0,v0,0x10
8000c7a8:	10400025 	beqz	v0,8000c840 <_dtoa_r+0x788>
8000c7ac:	00000000 	nop
8000c7b0:	8fc2003c 	lw	v0,60(s8)
8000c7b4:	3042000f 	andi	v0,v0,0xf
8000c7b8:	afc2003c 	sw	v0,60(s8)
8000c7bc:	d7c100b8 	ldc1	$f1,184(s8)
8000c7c0:	3c028002 	lui	v0,0x8002
8000c7c4:	2442a8f8 	addiu	v0,v0,-22280
8000c7c8:	d4400020 	ldc1	$f0,32(v0)
8000c7cc:	46200803 	div.d	$f0,$f1,$f0
8000c7d0:	f7c000b8 	sdc1	$f0,184(s8)
8000c7d4:	8fc20030 	lw	v0,48(s8)
8000c7d8:	24420001 	addiu	v0,v0,1
8000c7dc:	afc20030 	sw	v0,48(s8)
8000c7e0:	10000017 	b	8000c840 <_dtoa_r+0x788>
8000c7e4:	00000000 	nop
8000c7e8:	8fc2003c 	lw	v0,60(s8)
8000c7ec:	30420001 	andi	v0,v0,0x1
8000c7f0:	1040000d 	beqz	v0,8000c828 <_dtoa_r+0x770>
8000c7f4:	00000000 	nop
8000c7f8:	8fc20030 	lw	v0,48(s8)
8000c7fc:	24420001 	addiu	v0,v0,1
8000c800:	afc20030 	sw	v0,48(s8)
8000c804:	3c028002 	lui	v0,0x8002
8000c808:	8fc3002c 	lw	v1,44(s8)
8000c80c:	000318c0 	sll	v1,v1,0x3
8000c810:	2442a8f8 	addiu	v0,v0,-22280
8000c814:	00621021 	addu	v0,v1,v0
8000c818:	d4400000 	ldc1	$f0,0(v0)
8000c81c:	d7c10078 	ldc1	$f1,120(s8)
8000c820:	46200802 	mul.d	$f0,$f1,$f0
8000c824:	f7c00078 	sdc1	$f0,120(s8)
8000c828:	8fc2003c 	lw	v0,60(s8)
8000c82c:	00021043 	sra	v0,v0,0x1
8000c830:	afc2003c 	sw	v0,60(s8)
8000c834:	8fc2002c 	lw	v0,44(s8)
8000c838:	24420001 	addiu	v0,v0,1
8000c83c:	afc2002c 	sw	v0,44(s8)
8000c840:	8fc2003c 	lw	v0,60(s8)
8000c844:	1440ffe8 	bnez	v0,8000c7e8 <_dtoa_r+0x730>
8000c848:	00000000 	nop
8000c84c:	d7c100b8 	ldc1	$f1,184(s8)
8000c850:	d7c00078 	ldc1	$f0,120(s8)
8000c854:	46200803 	div.d	$f0,$f1,$f0
8000c858:	f7c000b8 	sdc1	$f0,184(s8)
8000c85c:	1000002f 	b	8000c91c <_dtoa_r+0x864>
8000c860:	00000000 	nop
8000c864:	8fc20040 	lw	v0,64(s8)
8000c868:	00021023 	negu	v0,v0
8000c86c:	afc20098 	sw	v0,152(s8)
8000c870:	8fc20098 	lw	v0,152(s8)
8000c874:	10400029 	beqz	v0,8000c91c <_dtoa_r+0x864>
8000c878:	00000000 	nop
8000c87c:	d7c100b8 	ldc1	$f1,184(s8)
8000c880:	8fc20098 	lw	v0,152(s8)
8000c884:	3043000f 	andi	v1,v0,0xf
8000c888:	3c028002 	lui	v0,0x8002
8000c88c:	000318c0 	sll	v1,v1,0x3
8000c890:	2442a830 	addiu	v0,v0,-22480
8000c894:	00621021 	addu	v0,v1,v0
8000c898:	d4400000 	ldc1	$f0,0(v0)
8000c89c:	46200802 	mul.d	$f0,$f1,$f0
8000c8a0:	f7c000b8 	sdc1	$f0,184(s8)
8000c8a4:	8fc20098 	lw	v0,152(s8)
8000c8a8:	00021103 	sra	v0,v0,0x4
8000c8ac:	afc2003c 	sw	v0,60(s8)
8000c8b0:	10000017 	b	8000c910 <_dtoa_r+0x858>
8000c8b4:	00000000 	nop
8000c8b8:	8fc2003c 	lw	v0,60(s8)
8000c8bc:	30420001 	andi	v0,v0,0x1
8000c8c0:	1040000d 	beqz	v0,8000c8f8 <_dtoa_r+0x840>
8000c8c4:	00000000 	nop
8000c8c8:	8fc20030 	lw	v0,48(s8)
8000c8cc:	24420001 	addiu	v0,v0,1
8000c8d0:	afc20030 	sw	v0,48(s8)
8000c8d4:	d7c100b8 	ldc1	$f1,184(s8)
8000c8d8:	3c028002 	lui	v0,0x8002
8000c8dc:	8fc3002c 	lw	v1,44(s8)
8000c8e0:	000318c0 	sll	v1,v1,0x3
8000c8e4:	2442a8f8 	addiu	v0,v0,-22280
8000c8e8:	00621021 	addu	v0,v1,v0
8000c8ec:	d4400000 	ldc1	$f0,0(v0)
8000c8f0:	46200802 	mul.d	$f0,$f1,$f0
8000c8f4:	f7c000b8 	sdc1	$f0,184(s8)
8000c8f8:	8fc2003c 	lw	v0,60(s8)
8000c8fc:	00021043 	sra	v0,v0,0x1
8000c900:	afc2003c 	sw	v0,60(s8)
8000c904:	8fc2002c 	lw	v0,44(s8)
8000c908:	24420001 	addiu	v0,v0,1
8000c90c:	afc2002c 	sw	v0,44(s8)
8000c910:	8fc2003c 	lw	v0,60(s8)
8000c914:	1440ffe8 	bnez	v0,8000c8b8 <_dtoa_r+0x800>
8000c918:	00000000 	nop
8000c91c:	8fc20044 	lw	v0,68(s8)
8000c920:	10400019 	beqz	v0,8000c988 <_dtoa_r+0x8d0>
8000c924:	00000000 	nop
8000c928:	d7c100b8 	ldc1	$f1,184(s8)
8000c92c:	d7808068 	ldc1	$f0,-32664(gp)
8000c930:	4620083c 	c.lt.d	$f1,$f0
8000c934:	00000000 	nop
8000c938:	45000013 	bc1f	8000c988 <_dtoa_r+0x8d0>
8000c93c:	00000000 	nop
8000c940:	8fc20034 	lw	v0,52(s8)
8000c944:	18400010 	blez	v0,8000c988 <_dtoa_r+0x8d0>
8000c948:	00000000 	nop
8000c94c:	8fc20038 	lw	v0,56(s8)
8000c950:	184000cb 	blez	v0,8000cc80 <_dtoa_r+0xbc8>
8000c954:	00000000 	nop
8000c958:	8fc20038 	lw	v0,56(s8)
8000c95c:	afc20034 	sw	v0,52(s8)
8000c960:	8fc20040 	lw	v0,64(s8)
8000c964:	2442ffff 	addiu	v0,v0,-1
8000c968:	afc20040 	sw	v0,64(s8)
8000c96c:	d7c100b8 	ldc1	$f1,184(s8)
8000c970:	d7808070 	ldc1	$f0,-32656(gp)
8000c974:	46200802 	mul.d	$f0,$f1,$f0
8000c978:	f7c000b8 	sdc1	$f0,184(s8)
8000c97c:	8fc20030 	lw	v0,48(s8)
8000c980:	24420001 	addiu	v0,v0,1
8000c984:	afc20030 	sw	v0,48(s8)
8000c988:	8fc20030 	lw	v0,48(s8)
8000c98c:	44820000 	mtc1	v0,$f0
8000c990:	00000000 	nop
8000c994:	46800061 	cvt.d.w	$f1,$f0
8000c998:	d7c000b8 	ldc1	$f0,184(s8)
8000c99c:	46200842 	mul.d	$f1,$f1,$f0
8000c9a0:	d7808078 	ldc1	$f0,-32648(gp)
8000c9a4:	46200800 	add.d	$f0,$f1,$f0
8000c9a8:	f7c000c8 	sdc1	$f0,200(s8)
8000c9ac:	8fc300c8 	lw	v1,200(s8)
8000c9b0:	3c02fcc0 	lui	v0,0xfcc0
8000c9b4:	00621021 	addu	v0,v1,v0
8000c9b8:	afc200c8 	sw	v0,200(s8)
8000c9bc:	8fc20034 	lw	v0,52(s8)
8000c9c0:	1440001b 	bnez	v0,8000ca30 <_dtoa_r+0x978>
8000c9c4:	00000000 	nop
8000c9c8:	afc00070 	sw	zero,112(s8)
8000c9cc:	8fc20070 	lw	v0,112(s8)
8000c9d0:	afc20074 	sw	v0,116(s8)
8000c9d4:	d7c100b8 	ldc1	$f1,184(s8)
8000c9d8:	d7808080 	ldc1	$f0,-32640(gp)
8000c9dc:	46200801 	sub.d	$f0,$f1,$f0
8000c9e0:	f7c000b8 	sdc1	$f0,184(s8)
8000c9e4:	d7c100b8 	ldc1	$f1,184(s8)
8000c9e8:	d7c000c8 	ldc1	$f0,200(s8)
8000c9ec:	4621003c 	c.lt.d	$f0,$f1
8000c9f0:	00000000 	nop
8000c9f4:	450102a2 	bc1t	8000d480 <_dtoa_r+0x13c8>
8000c9f8:	00000000 	nop
8000c9fc:	d7c000b8 	ldc1	$f0,184(s8)
8000ca00:	dfc300c8 	ld	v1,200(s8)
8000ca04:	2402ffff 	li	v0,-1
8000ca08:	000217fc 	dsll32	v0,v0,0x1f
8000ca0c:	00621026 	xor	v0,v1,v0
8000ca10:	44a20800 	dmtc1	v0,$f1
8000ca14:	00000000 	nop
8000ca18:	4621003c 	c.lt.d	$f0,$f1
8000ca1c:	00000000 	nop
8000ca20:	4501028b 	bc1t	8000d450 <_dtoa_r+0x1398>
8000ca24:	00000000 	nop
8000ca28:	10000099 	b	8000cc90 <_dtoa_r+0xbd8>
8000ca2c:	00000000 	nop
8000ca30:	8fc20048 	lw	v0,72(s8)
8000ca34:	10400044 	beqz	v0,8000cb48 <_dtoa_r+0xa90>
8000ca38:	00000000 	nop
8000ca3c:	8fc20034 	lw	v0,52(s8)
8000ca40:	2443ffff 	addiu	v1,v0,-1
8000ca44:	3c028002 	lui	v0,0x8002
8000ca48:	000318c0 	sll	v1,v1,0x3
8000ca4c:	2442a830 	addiu	v0,v0,-22480
8000ca50:	00621021 	addu	v0,v1,v0
8000ca54:	d4400000 	ldc1	$f0,0(v0)
8000ca58:	d7818088 	ldc1	$f1,-32632(gp)
8000ca5c:	46200843 	div.d	$f1,$f1,$f0
8000ca60:	d7c000c8 	ldc1	$f0,200(s8)
8000ca64:	46200801 	sub.d	$f0,$f1,$f0
8000ca68:	f7c000c8 	sdc1	$f0,200(s8)
8000ca6c:	afc0002c 	sw	zero,44(s8)
8000ca70:	d7c000b8 	ldc1	$f0,184(s8)
8000ca74:	4620000d 	trunc.w.d	$f0,$f0
8000ca78:	44020000 	mfc1	v0,$f0
8000ca7c:	00000000 	nop
8000ca80:	afc2009c 	sw	v0,156(s8)
8000ca84:	d7c100b8 	ldc1	$f1,184(s8)
8000ca88:	8fc2009c 	lw	v0,156(s8)
8000ca8c:	44820000 	mtc1	v0,$f0
8000ca90:	00000000 	nop
8000ca94:	46800021 	cvt.d.w	$f0,$f0
8000ca98:	46200801 	sub.d	$f0,$f1,$f0
8000ca9c:	f7c000b8 	sdc1	$f0,184(s8)
8000caa0:	8fc2009c 	lw	v0,156(s8)
8000caa4:	304200ff 	andi	v0,v0,0xff
8000caa8:	24420030 	addiu	v0,v0,48
8000caac:	304400ff 	andi	a0,v0,0xff
8000cab0:	8fc20080 	lw	v0,128(s8)
8000cab4:	24430001 	addiu	v1,v0,1
8000cab8:	afc30080 	sw	v1,128(s8)
8000cabc:	00041e00 	sll	v1,a0,0x18
8000cac0:	00031e03 	sra	v1,v1,0x18
8000cac4:	a0430000 	sb	v1,0(v0)
8000cac8:	d7c100b8 	ldc1	$f1,184(s8)
8000cacc:	d7c000c8 	ldc1	$f0,200(s8)
8000cad0:	4620083c 	c.lt.d	$f1,$f0
8000cad4:	00000000 	nop
8000cad8:	450103f8 	bc1t	8000dabc <_dtoa_r+0x1a04>
8000cadc:	00000000 	nop
8000cae0:	d7c000b8 	ldc1	$f0,184(s8)
8000cae4:	d7818068 	ldc1	$f1,-32664(gp)
8000cae8:	46200841 	sub.d	$f1,$f1,$f0
8000caec:	d7c000c8 	ldc1	$f0,200(s8)
8000caf0:	4620083c 	c.lt.d	$f1,$f0
8000caf4:	00000000 	nop
8000caf8:	450100c6 	bc1t	8000ce14 <_dtoa_r+0xd5c>
8000cafc:	00000000 	nop
8000cb00:	8fc2002c 	lw	v0,44(s8)
8000cb04:	24420001 	addiu	v0,v0,1
8000cb08:	afc2002c 	sw	v0,44(s8)
8000cb0c:	8fc3002c 	lw	v1,44(s8)
8000cb10:	8fc20034 	lw	v0,52(s8)
8000cb14:	0062102a 	slt	v0,v1,v0
8000cb18:	1040005c 	beqz	v0,8000cc8c <_dtoa_r+0xbd4>
8000cb1c:	00000000 	nop
8000cb20:	d7c100c8 	ldc1	$f1,200(s8)
8000cb24:	d7808070 	ldc1	$f0,-32656(gp)
8000cb28:	46200802 	mul.d	$f0,$f1,$f0
8000cb2c:	f7c000c8 	sdc1	$f0,200(s8)
8000cb30:	d7c100b8 	ldc1	$f1,184(s8)
8000cb34:	d7808070 	ldc1	$f0,-32656(gp)
8000cb38:	46200802 	mul.d	$f0,$f1,$f0
8000cb3c:	f7c000b8 	sdc1	$f0,184(s8)
8000cb40:	1000ffcb 	b	8000ca70 <_dtoa_r+0x9b8>
8000cb44:	00000000 	nop
8000cb48:	d7c100c8 	ldc1	$f1,200(s8)
8000cb4c:	8fc20034 	lw	v0,52(s8)
8000cb50:	2443ffff 	addiu	v1,v0,-1
8000cb54:	3c028002 	lui	v0,0x8002
8000cb58:	000318c0 	sll	v1,v1,0x3
8000cb5c:	2442a830 	addiu	v0,v0,-22480
8000cb60:	00621021 	addu	v0,v1,v0
8000cb64:	d4400000 	ldc1	$f0,0(v0)
8000cb68:	46200802 	mul.d	$f0,$f1,$f0
8000cb6c:	f7c000c8 	sdc1	$f0,200(s8)
8000cb70:	24020001 	li	v0,1
8000cb74:	afc2002c 	sw	v0,44(s8)
8000cb78:	d7c000b8 	ldc1	$f0,184(s8)
8000cb7c:	4620000d 	trunc.w.d	$f0,$f0
8000cb80:	44020000 	mfc1	v0,$f0
8000cb84:	00000000 	nop
8000cb88:	afc2009c 	sw	v0,156(s8)
8000cb8c:	d7c100b8 	ldc1	$f1,184(s8)
8000cb90:	8fc2009c 	lw	v0,156(s8)
8000cb94:	44820000 	mtc1	v0,$f0
8000cb98:	00000000 	nop
8000cb9c:	46800021 	cvt.d.w	$f0,$f0
8000cba0:	46200801 	sub.d	$f0,$f1,$f0
8000cba4:	f7c000b8 	sdc1	$f0,184(s8)
8000cba8:	8fc2009c 	lw	v0,156(s8)
8000cbac:	304200ff 	andi	v0,v0,0xff
8000cbb0:	24420030 	addiu	v0,v0,48
8000cbb4:	304400ff 	andi	a0,v0,0xff
8000cbb8:	8fc20080 	lw	v0,128(s8)
8000cbbc:	24430001 	addiu	v1,v0,1
8000cbc0:	afc30080 	sw	v1,128(s8)
8000cbc4:	00041e00 	sll	v1,a0,0x18
8000cbc8:	00031e03 	sra	v1,v1,0x18
8000cbcc:	a0430000 	sb	v1,0(v0)
8000cbd0:	8fc3002c 	lw	v1,44(s8)
8000cbd4:	8fc20034 	lw	v0,52(s8)
8000cbd8:	14620020 	bne	v1,v0,8000cc5c <_dtoa_r+0xba4>
8000cbdc:	00000000 	nop
8000cbe0:	d7c100b8 	ldc1	$f1,184(s8)
8000cbe4:	d7c200c8 	ldc1	$f2,200(s8)
8000cbe8:	d7808088 	ldc1	$f0,-32632(gp)
8000cbec:	46201000 	add.d	$f0,$f2,$f0
8000cbf0:	4621003c 	c.lt.d	$f0,$f1
8000cbf4:	00000000 	nop
8000cbf8:	45010089 	bc1t	8000ce20 <_dtoa_r+0xd68>
8000cbfc:	00000000 	nop
8000cc00:	d7c100b8 	ldc1	$f1,184(s8)
8000cc04:	d7c000c8 	ldc1	$f0,200(s8)
8000cc08:	d7828088 	ldc1	$f2,-32632(gp)
8000cc0c:	46201001 	sub.d	$f0,$f2,$f0
8000cc10:	4620083c 	c.lt.d	$f1,$f0
8000cc14:	00000000 	nop
8000cc18:	45010003 	bc1t	8000cc28 <_dtoa_r+0xb70>
8000cc1c:	00000000 	nop
8000cc20:	1000001b 	b	8000cc90 <_dtoa_r+0xbd8>
8000cc24:	00000000 	nop
8000cc28:	8fc20080 	lw	v0,128(s8)
8000cc2c:	2442ffff 	addiu	v0,v0,-1
8000cc30:	afc20080 	sw	v0,128(s8)
8000cc34:	8fc20080 	lw	v0,128(s8)
8000cc38:	80430000 	lb	v1,0(v0)
8000cc3c:	24020030 	li	v0,48
8000cc40:	1062fff9 	beq	v1,v0,8000cc28 <_dtoa_r+0xb70>
8000cc44:	00000000 	nop
8000cc48:	8fc20080 	lw	v0,128(s8)
8000cc4c:	24420001 	addiu	v0,v0,1
8000cc50:	afc20080 	sw	v0,128(s8)
8000cc54:	1000039d 	b	8000dacc <_dtoa_r+0x1a14>
8000cc58:	00000000 	nop
8000cc5c:	8fc2002c 	lw	v0,44(s8)
8000cc60:	24420001 	addiu	v0,v0,1
8000cc64:	afc2002c 	sw	v0,44(s8)
8000cc68:	d7c100b8 	ldc1	$f1,184(s8)
8000cc6c:	d7808070 	ldc1	$f0,-32656(gp)
8000cc70:	46200802 	mul.d	$f0,$f1,$f0
8000cc74:	f7c000b8 	sdc1	$f0,184(s8)
8000cc78:	1000ffbf 	b	8000cb78 <_dtoa_r+0xac0>
8000cc7c:	00000000 	nop
8000cc80:	00000000 	nop
8000cc84:	10000002 	b	8000cc90 <_dtoa_r+0xbd8>
8000cc88:	00000000 	nop
8000cc8c:	00000000 	nop
8000cc90:	8fc2008c 	lw	v0,140(s8)
8000cc94:	afc20080 	sw	v0,128(s8)
8000cc98:	d7c000c0 	ldc1	$f0,192(s8)
8000cc9c:	f7c000b8 	sdc1	$f0,184(s8)
8000cca0:	8fc20090 	lw	v0,144(s8)
8000cca4:	afc20040 	sw	v0,64(s8)
8000cca8:	8fc20094 	lw	v0,148(s8)
8000ccac:	afc20034 	sw	v0,52(s8)
8000ccb0:	8fc200b0 	lw	v0,176(s8)
8000ccb4:	04400096 	bltz	v0,8000cf10 <_dtoa_r+0xe58>
8000ccb8:	00000000 	nop
8000ccbc:	8fc20040 	lw	v0,64(s8)
8000ccc0:	2842000f 	slti	v0,v0,15
8000ccc4:	10400092 	beqz	v0,8000cf10 <_dtoa_r+0xe58>
8000ccc8:	00000000 	nop
8000cccc:	3c028002 	lui	v0,0x8002
8000ccd0:	8fc30040 	lw	v1,64(s8)
8000ccd4:	000318c0 	sll	v1,v1,0x3
8000ccd8:	2442a830 	addiu	v0,v0,-22480
8000ccdc:	00621021 	addu	v0,v1,v0
8000cce0:	d4400000 	ldc1	$f0,0(v0)
8000cce4:	f7c00078 	sdc1	$f0,120(s8)
8000cce8:	8fc200f8 	lw	v0,248(s8)
8000ccec:	04410014 	bgez	v0,8000cd40 <_dtoa_r+0xc88>
8000ccf0:	00000000 	nop
8000ccf4:	8fc20034 	lw	v0,52(s8)
8000ccf8:	1c400011 	bgtz	v0,8000cd40 <_dtoa_r+0xc88>
8000ccfc:	00000000 	nop
8000cd00:	afc00070 	sw	zero,112(s8)
8000cd04:	8fc20070 	lw	v0,112(s8)
8000cd08:	afc20074 	sw	v0,116(s8)
8000cd0c:	8fc20034 	lw	v0,52(s8)
8000cd10:	044001d2 	bltz	v0,8000d45c <_dtoa_r+0x13a4>
8000cd14:	00000000 	nop
8000cd18:	d7c100b8 	ldc1	$f1,184(s8)
8000cd1c:	d7c20078 	ldc1	$f2,120(s8)
8000cd20:	d7808080 	ldc1	$f0,-32640(gp)
8000cd24:	46201002 	mul.d	$f0,$f2,$f0
8000cd28:	4620083e 	c.le.d	$f1,$f0
8000cd2c:	00000000 	nop
8000cd30:	450101ca 	bc1t	8000d45c <_dtoa_r+0x13a4>
8000cd34:	00000000 	nop
8000cd38:	100001d5 	b	8000d490 <_dtoa_r+0x13d8>
8000cd3c:	00000000 	nop
8000cd40:	24020001 	li	v0,1
8000cd44:	afc2002c 	sw	v0,44(s8)
8000cd48:	d7c100b8 	ldc1	$f1,184(s8)
8000cd4c:	d7c00078 	ldc1	$f0,120(s8)
8000cd50:	46200803 	div.d	$f0,$f1,$f0
8000cd54:	4620000d 	trunc.w.d	$f0,$f0
8000cd58:	44020000 	mfc1	v0,$f0
8000cd5c:	00000000 	nop
8000cd60:	afc2009c 	sw	v0,156(s8)
8000cd64:	d7c100b8 	ldc1	$f1,184(s8)
8000cd68:	8fc2009c 	lw	v0,156(s8)
8000cd6c:	44820000 	mtc1	v0,$f0
8000cd70:	00000000 	nop
8000cd74:	468000a1 	cvt.d.w	$f2,$f0
8000cd78:	d7c00078 	ldc1	$f0,120(s8)
8000cd7c:	46201002 	mul.d	$f0,$f2,$f0
8000cd80:	46200801 	sub.d	$f0,$f1,$f0
8000cd84:	f7c000b8 	sdc1	$f0,184(s8)
8000cd88:	8fc2009c 	lw	v0,156(s8)
8000cd8c:	304200ff 	andi	v0,v0,0xff
8000cd90:	24420030 	addiu	v0,v0,48
8000cd94:	304400ff 	andi	a0,v0,0xff
8000cd98:	8fc20080 	lw	v0,128(s8)
8000cd9c:	24430001 	addiu	v1,v0,1
8000cda0:	afc30080 	sw	v1,128(s8)
8000cda4:	00041e00 	sll	v1,a0,0x18
8000cda8:	00031e03 	sra	v1,v1,0x18
8000cdac:	a0430000 	sb	v1,0(v0)
8000cdb0:	8fc3002c 	lw	v1,44(s8)
8000cdb4:	8fc20034 	lw	v0,52(s8)
8000cdb8:	1462003f 	bne	v1,v0,8000ceb8 <_dtoa_r+0xe00>
8000cdbc:	00000000 	nop
8000cdc0:	d7c000b8 	ldc1	$f0,184(s8)
8000cdc4:	46200000 	add.d	$f0,$f0,$f0
8000cdc8:	f7c000b8 	sdc1	$f0,184(s8)
8000cdcc:	d7c000b8 	ldc1	$f0,184(s8)
8000cdd0:	d7c10078 	ldc1	$f1,120(s8)
8000cdd4:	4620083c 	c.lt.d	$f1,$f0
8000cdd8:	00000000 	nop
8000cddc:	45010013 	bc1t	8000ce2c <_dtoa_r+0xd74>
8000cde0:	00000000 	nop
8000cde4:	d7c000b8 	ldc1	$f0,184(s8)
8000cde8:	d7c10078 	ldc1	$f1,120(s8)
8000cdec:	46200832 	c.eq.d	$f1,$f0
8000cdf0:	00000000 	nop
8000cdf4:	45000040 	bc1f	8000cef8 <_dtoa_r+0xe40>
8000cdf8:	00000000 	nop
8000cdfc:	8fc2009c 	lw	v0,156(s8)
8000ce00:	30420001 	andi	v0,v0,0x1
8000ce04:	1040003c 	beqz	v0,8000cef8 <_dtoa_r+0xe40>
8000ce08:	00000000 	nop
8000ce0c:	10000007 	b	8000ce2c <_dtoa_r+0xd74>
8000ce10:	00000000 	nop
8000ce14:	00000000 	nop
8000ce18:	10000013 	b	8000ce68 <_dtoa_r+0xdb0>
8000ce1c:	00000000 	nop
8000ce20:	00000000 	nop
8000ce24:	10000010 	b	8000ce68 <_dtoa_r+0xdb0>
8000ce28:	00000000 	nop
8000ce2c:	00000000 	nop
8000ce30:	1000000d 	b	8000ce68 <_dtoa_r+0xdb0>
8000ce34:	00000000 	nop
8000ce38:	8fc30080 	lw	v1,128(s8)
8000ce3c:	8fc2008c 	lw	v0,140(s8)
8000ce40:	14620009 	bne	v1,v0,8000ce68 <_dtoa_r+0xdb0>
8000ce44:	00000000 	nop
8000ce48:	8fc20040 	lw	v0,64(s8)
8000ce4c:	24420001 	addiu	v0,v0,1
8000ce50:	afc20040 	sw	v0,64(s8)
8000ce54:	8fc20080 	lw	v0,128(s8)
8000ce58:	24030030 	li	v1,48
8000ce5c:	a0430000 	sb	v1,0(v0)
8000ce60:	10000009 	b	8000ce88 <_dtoa_r+0xdd0>
8000ce64:	00000000 	nop
8000ce68:	8fc20080 	lw	v0,128(s8)
8000ce6c:	2442ffff 	addiu	v0,v0,-1
8000ce70:	afc20080 	sw	v0,128(s8)
8000ce74:	8fc20080 	lw	v0,128(s8)
8000ce78:	80430000 	lb	v1,0(v0)
8000ce7c:	24020039 	li	v0,57
8000ce80:	1062ffed 	beq	v1,v0,8000ce38 <_dtoa_r+0xd80>
8000ce84:	00000000 	nop
8000ce88:	8fc20080 	lw	v0,128(s8)
8000ce8c:	24430001 	addiu	v1,v0,1
8000ce90:	afc30080 	sw	v1,128(s8)
8000ce94:	80430000 	lb	v1,0(v0)
8000ce98:	306300ff 	andi	v1,v1,0xff
8000ce9c:	24630001 	addiu	v1,v1,1
8000cea0:	306300ff 	andi	v1,v1,0xff
8000cea4:	00031e00 	sll	v1,v1,0x18
8000cea8:	00031e03 	sra	v1,v1,0x18
8000ceac:	a0430000 	sb	v1,0(v0)
8000ceb0:	10000011 	b	8000cef8 <_dtoa_r+0xe40>
8000ceb4:	00000000 	nop
8000ceb8:	d7c100b8 	ldc1	$f1,184(s8)
8000cebc:	d7808070 	ldc1	$f0,-32656(gp)
8000cec0:	46200802 	mul.d	$f0,$f1,$f0
8000cec4:	f7c000b8 	sdc1	$f0,184(s8)
8000cec8:	d7c000b8 	ldc1	$f0,184(s8)
8000cecc:	44a00800 	dmtc1	zero,$f1
8000ced0:	00000000 	nop
8000ced4:	46210032 	c.eq.d	$f0,$f1
8000ced8:	00000000 	nop
8000cedc:	45010009 	bc1t	8000cf04 <_dtoa_r+0xe4c>
8000cee0:	00000000 	nop
8000cee4:	8fc2002c 	lw	v0,44(s8)
8000cee8:	24420001 	addiu	v0,v0,1
8000ceec:	afc2002c 	sw	v0,44(s8)
8000cef0:	1000ff95 	b	8000cd48 <_dtoa_r+0xc90>
8000cef4:	00000000 	nop
8000cef8:	00000000 	nop
8000cefc:	100002f3 	b	8000dacc <_dtoa_r+0x1a14>
8000cf00:	00000000 	nop
8000cf04:	00000000 	nop
8000cf08:	100002f0 	b	8000dacc <_dtoa_r+0x1a14>
8000cf0c:	00000000 	nop
8000cf10:	8fc20020 	lw	v0,32(s8)
8000cf14:	afc2004c 	sw	v0,76(s8)
8000cf18:	8fc20024 	lw	v0,36(s8)
8000cf1c:	afc20050 	sw	v0,80(s8)
8000cf20:	afc0006c 	sw	zero,108(s8)
8000cf24:	8fc2006c 	lw	v0,108(s8)
8000cf28:	afc20070 	sw	v0,112(s8)
8000cf2c:	8fc20048 	lw	v0,72(s8)
8000cf30:	10400044 	beqz	v0,8000d044 <_dtoa_r+0xf8c>
8000cf34:	00000000 	nop
8000cf38:	8fc200f0 	lw	v0,240(s8)
8000cf3c:	28420002 	slti	v0,v0,2
8000cf40:	1040000e 	beqz	v0,8000cf7c <_dtoa_r+0xec4>
8000cf44:	00000000 	nop
8000cf48:	8fc20064 	lw	v0,100(s8)
8000cf4c:	10400005 	beqz	v0,8000cf64 <_dtoa_r+0xeac>
8000cf50:	00000000 	nop
8000cf54:	8fc200b0 	lw	v0,176(s8)
8000cf58:	24420433 	addiu	v0,v0,1075
8000cf5c:	10000004 	b	8000cf70 <_dtoa_r+0xeb8>
8000cf60:	00000000 	nop
8000cf64:	8fc200ac 	lw	v0,172(s8)
8000cf68:	24030036 	li	v1,54
8000cf6c:	00621023 	subu	v0,v1,v0
8000cf70:	afc2002c 	sw	v0,44(s8)
8000cf74:	10000026 	b	8000d010 <_dtoa_r+0xf58>
8000cf78:	00000000 	nop
8000cf7c:	8fc20034 	lw	v0,52(s8)
8000cf80:	2442ffff 	addiu	v0,v0,-1
8000cf84:	afc2003c 	sw	v0,60(s8)
8000cf88:	8fc30050 	lw	v1,80(s8)
8000cf8c:	8fc2003c 	lw	v0,60(s8)
8000cf90:	0062102a 	slt	v0,v1,v0
8000cf94:	14400007 	bnez	v0,8000cfb4 <_dtoa_r+0xefc>
8000cf98:	00000000 	nop
8000cf9c:	8fc30050 	lw	v1,80(s8)
8000cfa0:	8fc2003c 	lw	v0,60(s8)
8000cfa4:	00621023 	subu	v0,v1,v0
8000cfa8:	afc20050 	sw	v0,80(s8)
8000cfac:	1000000e 	b	8000cfe8 <_dtoa_r+0xf30>
8000cfb0:	00000000 	nop
8000cfb4:	8fc3003c 	lw	v1,60(s8)
8000cfb8:	8fc20050 	lw	v0,80(s8)
8000cfbc:	00621023 	subu	v0,v1,v0
8000cfc0:	afc2003c 	sw	v0,60(s8)
8000cfc4:	8fc2003c 	lw	v0,60(s8)
8000cfc8:	8fc30058 	lw	v1,88(s8)
8000cfcc:	00621021 	addu	v0,v1,v0
8000cfd0:	afc20058 	sw	v0,88(s8)
8000cfd4:	8fc30024 	lw	v1,36(s8)
8000cfd8:	8fc2003c 	lw	v0,60(s8)
8000cfdc:	00621021 	addu	v0,v1,v0
8000cfe0:	afc20024 	sw	v0,36(s8)
8000cfe4:	afc00050 	sw	zero,80(s8)
8000cfe8:	8fc20034 	lw	v0,52(s8)
8000cfec:	afc2002c 	sw	v0,44(s8)
8000cff0:	8fc2002c 	lw	v0,44(s8)
8000cff4:	04410006 	bgez	v0,8000d010 <_dtoa_r+0xf58>
8000cff8:	00000000 	nop
8000cffc:	8fc3004c 	lw	v1,76(s8)
8000d000:	8fc2002c 	lw	v0,44(s8)
8000d004:	00621023 	subu	v0,v1,v0
8000d008:	afc2004c 	sw	v0,76(s8)
8000d00c:	afc0002c 	sw	zero,44(s8)
8000d010:	8fc30020 	lw	v1,32(s8)
8000d014:	8fc2002c 	lw	v0,44(s8)
8000d018:	00621021 	addu	v0,v1,v0
8000d01c:	afc20020 	sw	v0,32(s8)
8000d020:	8fc30054 	lw	v1,84(s8)
8000d024:	8fc2002c 	lw	v0,44(s8)
8000d028:	00621021 	addu	v0,v1,v0
8000d02c:	afc20054 	sw	v0,84(s8)
8000d030:	24050001 	li	a1,1
8000d034:	8fc400e0 	lw	a0,224(s8)
8000d038:	0c003d48 	jal	8000f520 <__i2b>
8000d03c:	00000000 	nop
8000d040:	afc20070 	sw	v0,112(s8)
8000d044:	8fc2004c 	lw	v0,76(s8)
8000d048:	18400017 	blez	v0,8000d0a8 <_dtoa_r+0xff0>
8000d04c:	00000000 	nop
8000d050:	8fc20054 	lw	v0,84(s8)
8000d054:	18400014 	blez	v0,8000d0a8 <_dtoa_r+0xff0>
8000d058:	00000000 	nop
8000d05c:	8fc3004c 	lw	v1,76(s8)
8000d060:	8fc20054 	lw	v0,84(s8)
8000d064:	0062202a 	slt	a0,v1,v0
8000d068:	10800002 	beqz	a0,8000d074 <_dtoa_r+0xfbc>
8000d06c:	00000000 	nop
8000d070:	00601025 	move	v0,v1
8000d074:	afc2002c 	sw	v0,44(s8)
8000d078:	8fc30020 	lw	v1,32(s8)
8000d07c:	8fc2002c 	lw	v0,44(s8)
8000d080:	00621023 	subu	v0,v1,v0
8000d084:	afc20020 	sw	v0,32(s8)
8000d088:	8fc3004c 	lw	v1,76(s8)
8000d08c:	8fc2002c 	lw	v0,44(s8)
8000d090:	00621023 	subu	v0,v1,v0
8000d094:	afc2004c 	sw	v0,76(s8)
8000d098:	8fc30054 	lw	v1,84(s8)
8000d09c:	8fc2002c 	lw	v0,44(s8)
8000d0a0:	00621023 	subu	v0,v1,v0
8000d0a4:	afc20054 	sw	v0,84(s8)
8000d0a8:	8fc20024 	lw	v0,36(s8)
8000d0ac:	1840002e 	blez	v0,8000d168 <_dtoa_r+0x10b0>
8000d0b0:	00000000 	nop
8000d0b4:	8fc20048 	lw	v0,72(s8)
8000d0b8:	10400025 	beqz	v0,8000d150 <_dtoa_r+0x1098>
8000d0bc:	00000000 	nop
8000d0c0:	8fc20050 	lw	v0,80(s8)
8000d0c4:	18400013 	blez	v0,8000d114 <_dtoa_r+0x105c>
8000d0c8:	00000000 	nop
8000d0cc:	8fc60050 	lw	a2,80(s8)
8000d0d0:	8fc50070 	lw	a1,112(s8)
8000d0d4:	8fc400e0 	lw	a0,224(s8)
8000d0d8:	0c003e7f 	jal	8000f9fc <__pow5mult>
8000d0dc:	00000000 	nop
8000d0e0:	afc20070 	sw	v0,112(s8)
8000d0e4:	8fc60068 	lw	a2,104(s8)
8000d0e8:	8fc50070 	lw	a1,112(s8)
8000d0ec:	8fc400e0 	lw	a0,224(s8)
8000d0f0:	0c003d6d 	jal	8000f5b4 <__multiply>
8000d0f4:	00000000 	nop
8000d0f8:	afc200a0 	sw	v0,160(s8)
8000d0fc:	8fc50068 	lw	a1,104(s8)
8000d100:	8fc400e0 	lw	a0,224(s8)
8000d104:	0c003bae 	jal	8000eeb8 <_Bfree>
8000d108:	00000000 	nop
8000d10c:	8fc200a0 	lw	v0,160(s8)
8000d110:	afc20068 	sw	v0,104(s8)
8000d114:	8fc30024 	lw	v1,36(s8)
8000d118:	8fc20050 	lw	v0,80(s8)
8000d11c:	00621023 	subu	v0,v1,v0
8000d120:	afc2003c 	sw	v0,60(s8)
8000d124:	8fc2003c 	lw	v0,60(s8)
8000d128:	1040000f 	beqz	v0,8000d168 <_dtoa_r+0x10b0>
8000d12c:	00000000 	nop
8000d130:	8fc6003c 	lw	a2,60(s8)
8000d134:	8fc50068 	lw	a1,104(s8)
8000d138:	8fc400e0 	lw	a0,224(s8)
8000d13c:	0c003e7f 	jal	8000f9fc <__pow5mult>
8000d140:	00000000 	nop
8000d144:	afc20068 	sw	v0,104(s8)
8000d148:	10000007 	b	8000d168 <_dtoa_r+0x10b0>
8000d14c:	00000000 	nop
8000d150:	8fc60024 	lw	a2,36(s8)
8000d154:	8fc50068 	lw	a1,104(s8)
8000d158:	8fc400e0 	lw	a0,224(s8)
8000d15c:	0c003e7f 	jal	8000f9fc <__pow5mult>
8000d160:	00000000 	nop
8000d164:	afc20068 	sw	v0,104(s8)
8000d168:	24050001 	li	a1,1
8000d16c:	8fc400e0 	lw	a0,224(s8)
8000d170:	0c003d48 	jal	8000f520 <__i2b>
8000d174:	00000000 	nop
8000d178:	afc20074 	sw	v0,116(s8)
8000d17c:	8fc20058 	lw	v0,88(s8)
8000d180:	18400007 	blez	v0,8000d1a0 <_dtoa_r+0x10e8>
8000d184:	00000000 	nop
8000d188:	8fc60058 	lw	a2,88(s8)
8000d18c:	8fc50074 	lw	a1,116(s8)
8000d190:	8fc400e0 	lw	a0,224(s8)
8000d194:	0c003e7f 	jal	8000f9fc <__pow5mult>
8000d198:	00000000 	nop
8000d19c:	afc20074 	sw	v0,116(s8)
8000d1a0:	afc0005c 	sw	zero,92(s8)
8000d1a4:	8fc200f0 	lw	v0,240(s8)
8000d1a8:	28420002 	slti	v0,v0,2
8000d1ac:	10400017 	beqz	v0,8000d20c <_dtoa_r+0x1154>
8000d1b0:	00000000 	nop
8000d1b4:	8fc200bc 	lw	v0,188(s8)
8000d1b8:	14400014 	bnez	v0,8000d20c <_dtoa_r+0x1154>
8000d1bc:	00000000 	nop
8000d1c0:	8fc300b8 	lw	v1,184(s8)
8000d1c4:	3c02000f 	lui	v0,0xf
8000d1c8:	3442ffff 	ori	v0,v0,0xffff
8000d1cc:	00621024 	and	v0,v1,v0
8000d1d0:	1440000e 	bnez	v0,8000d20c <_dtoa_r+0x1154>
8000d1d4:	00000000 	nop
8000d1d8:	8fc300b8 	lw	v1,184(s8)
8000d1dc:	3c027ff0 	lui	v0,0x7ff0
8000d1e0:	00621024 	and	v0,v1,v0
8000d1e4:	10400009 	beqz	v0,8000d20c <_dtoa_r+0x1154>
8000d1e8:	00000000 	nop
8000d1ec:	8fc20020 	lw	v0,32(s8)
8000d1f0:	24420001 	addiu	v0,v0,1
8000d1f4:	afc20020 	sw	v0,32(s8)
8000d1f8:	8fc20054 	lw	v0,84(s8)
8000d1fc:	24420001 	addiu	v0,v0,1
8000d200:	afc20054 	sw	v0,84(s8)
8000d204:	24020001 	li	v0,1
8000d208:	afc2005c 	sw	v0,92(s8)
8000d20c:	8fc20058 	lw	v0,88(s8)
8000d210:	10400011 	beqz	v0,8000d258 <_dtoa_r+0x11a0>
8000d214:	00000000 	nop
8000d218:	8fc20074 	lw	v0,116(s8)
8000d21c:	8c420010 	lw	v0,16(v0)
8000d220:	2442ffff 	addiu	v0,v0,-1
8000d224:	8fc30074 	lw	v1,116(s8)
8000d228:	24420004 	addiu	v0,v0,4
8000d22c:	00021080 	sll	v0,v0,0x2
8000d230:	00621021 	addu	v0,v1,v0
8000d234:	8c420004 	lw	v0,4(v0)
8000d238:	00402025 	move	a0,v0
8000d23c:	0c003ccf 	jal	8000f33c <__hi0bits>
8000d240:	00000000 	nop
8000d244:	00401825 	move	v1,v0
8000d248:	24020020 	li	v0,32
8000d24c:	00431023 	subu	v0,v0,v1
8000d250:	10000002 	b	8000d25c <_dtoa_r+0x11a4>
8000d254:	00000000 	nop
8000d258:	24020001 	li	v0,1
8000d25c:	8fc30054 	lw	v1,84(s8)
8000d260:	00431021 	addu	v0,v0,v1
8000d264:	3042001f 	andi	v0,v0,0x1f
8000d268:	afc2002c 	sw	v0,44(s8)
8000d26c:	8fc2002c 	lw	v0,44(s8)
8000d270:	10400005 	beqz	v0,8000d288 <_dtoa_r+0x11d0>
8000d274:	00000000 	nop
8000d278:	24030020 	li	v1,32
8000d27c:	8fc2002c 	lw	v0,44(s8)
8000d280:	00621023 	subu	v0,v1,v0
8000d284:	afc2002c 	sw	v0,44(s8)
8000d288:	8fc2002c 	lw	v0,44(s8)
8000d28c:	28420005 	slti	v0,v0,5
8000d290:	14400012 	bnez	v0,8000d2dc <_dtoa_r+0x1224>
8000d294:	00000000 	nop
8000d298:	8fc2002c 	lw	v0,44(s8)
8000d29c:	2442fffc 	addiu	v0,v0,-4
8000d2a0:	afc2002c 	sw	v0,44(s8)
8000d2a4:	8fc30020 	lw	v1,32(s8)
8000d2a8:	8fc2002c 	lw	v0,44(s8)
8000d2ac:	00621021 	addu	v0,v1,v0
8000d2b0:	afc20020 	sw	v0,32(s8)
8000d2b4:	8fc3004c 	lw	v1,76(s8)
8000d2b8:	8fc2002c 	lw	v0,44(s8)
8000d2bc:	00621021 	addu	v0,v1,v0
8000d2c0:	afc2004c 	sw	v0,76(s8)
8000d2c4:	8fc30054 	lw	v1,84(s8)
8000d2c8:	8fc2002c 	lw	v0,44(s8)
8000d2cc:	00621021 	addu	v0,v1,v0
8000d2d0:	afc20054 	sw	v0,84(s8)
8000d2d4:	10000014 	b	8000d328 <_dtoa_r+0x1270>
8000d2d8:	00000000 	nop
8000d2dc:	8fc2002c 	lw	v0,44(s8)
8000d2e0:	28420004 	slti	v0,v0,4
8000d2e4:	10400010 	beqz	v0,8000d328 <_dtoa_r+0x1270>
8000d2e8:	00000000 	nop
8000d2ec:	8fc2002c 	lw	v0,44(s8)
8000d2f0:	2442001c 	addiu	v0,v0,28
8000d2f4:	afc2002c 	sw	v0,44(s8)
8000d2f8:	8fc30020 	lw	v1,32(s8)
8000d2fc:	8fc2002c 	lw	v0,44(s8)
8000d300:	00621021 	addu	v0,v1,v0
8000d304:	afc20020 	sw	v0,32(s8)
8000d308:	8fc3004c 	lw	v1,76(s8)
8000d30c:	8fc2002c 	lw	v0,44(s8)
8000d310:	00621021 	addu	v0,v1,v0
8000d314:	afc2004c 	sw	v0,76(s8)
8000d318:	8fc30054 	lw	v1,84(s8)
8000d31c:	8fc2002c 	lw	v0,44(s8)
8000d320:	00621021 	addu	v0,v1,v0
8000d324:	afc20054 	sw	v0,84(s8)
8000d328:	8fc20020 	lw	v0,32(s8)
8000d32c:	18400007 	blez	v0,8000d34c <_dtoa_r+0x1294>
8000d330:	00000000 	nop
8000d334:	8fc60020 	lw	a2,32(s8)
8000d338:	8fc50068 	lw	a1,104(s8)
8000d33c:	8fc400e0 	lw	a0,224(s8)
8000d340:	0c003eea 	jal	8000fba8 <__lshift>
8000d344:	00000000 	nop
8000d348:	afc20068 	sw	v0,104(s8)
8000d34c:	8fc20054 	lw	v0,84(s8)
8000d350:	18400007 	blez	v0,8000d370 <_dtoa_r+0x12b8>
8000d354:	00000000 	nop
8000d358:	8fc60054 	lw	a2,84(s8)
8000d35c:	8fc50074 	lw	a1,116(s8)
8000d360:	8fc400e0 	lw	a0,224(s8)
8000d364:	0c003eea 	jal	8000fba8 <__lshift>
8000d368:	00000000 	nop
8000d36c:	afc20074 	sw	v0,116(s8)
8000d370:	8fc20044 	lw	v0,68(s8)
8000d374:	1040001d 	beqz	v0,8000d3ec <_dtoa_r+0x1334>
8000d378:	00000000 	nop
8000d37c:	8fc50074 	lw	a1,116(s8)
8000d380:	8fc40068 	lw	a0,104(s8)
8000d384:	0c003f83 	jal	8000fe0c <__mcmp>
8000d388:	00000000 	nop
8000d38c:	04410017 	bgez	v0,8000d3ec <_dtoa_r+0x1334>
8000d390:	00000000 	nop
8000d394:	8fc20040 	lw	v0,64(s8)
8000d398:	2442ffff 	addiu	v0,v0,-1
8000d39c:	afc20040 	sw	v0,64(s8)
8000d3a0:	00003825 	move	a3,zero
8000d3a4:	2406000a 	li	a2,10
8000d3a8:	8fc50068 	lw	a1,104(s8)
8000d3ac:	8fc400e0 	lw	a0,224(s8)
8000d3b0:	0c003bcd 	jal	8000ef34 <__multadd>
8000d3b4:	00000000 	nop
8000d3b8:	afc20068 	sw	v0,104(s8)
8000d3bc:	8fc20048 	lw	v0,72(s8)
8000d3c0:	10400008 	beqz	v0,8000d3e4 <_dtoa_r+0x132c>
8000d3c4:	00000000 	nop
8000d3c8:	00003825 	move	a3,zero
8000d3cc:	2406000a 	li	a2,10
8000d3d0:	8fc50070 	lw	a1,112(s8)
8000d3d4:	8fc400e0 	lw	a0,224(s8)
8000d3d8:	0c003bcd 	jal	8000ef34 <__multadd>
8000d3dc:	00000000 	nop
8000d3e0:	afc20070 	sw	v0,112(s8)
8000d3e4:	8fc20038 	lw	v0,56(s8)
8000d3e8:	afc20034 	sw	v0,52(s8)
8000d3ec:	8fc20034 	lw	v0,52(s8)
8000d3f0:	1c400031 	bgtz	v0,8000d4b8 <_dtoa_r+0x1400>
8000d3f4:	00000000 	nop
8000d3f8:	8fc200f0 	lw	v0,240(s8)
8000d3fc:	28420003 	slti	v0,v0,3
8000d400:	1440002d 	bnez	v0,8000d4b8 <_dtoa_r+0x1400>
8000d404:	00000000 	nop
8000d408:	8fc20034 	lw	v0,52(s8)
8000d40c:	04400016 	bltz	v0,8000d468 <_dtoa_r+0x13b0>
8000d410:	00000000 	nop
8000d414:	00003825 	move	a3,zero
8000d418:	24060005 	li	a2,5
8000d41c:	8fc50074 	lw	a1,116(s8)
8000d420:	8fc400e0 	lw	a0,224(s8)
8000d424:	0c003bcd 	jal	8000ef34 <__multadd>
8000d428:	00000000 	nop
8000d42c:	afc20074 	sw	v0,116(s8)
8000d430:	8fc50074 	lw	a1,116(s8)
8000d434:	8fc40068 	lw	a0,104(s8)
8000d438:	0c003f83 	jal	8000fe0c <__mcmp>
8000d43c:	00000000 	nop
8000d440:	1c400012 	bgtz	v0,8000d48c <_dtoa_r+0x13d4>
8000d444:	00000000 	nop
8000d448:	10000007 	b	8000d468 <_dtoa_r+0x13b0>
8000d44c:	00000000 	nop
8000d450:	00000000 	nop
8000d454:	10000005 	b	8000d46c <_dtoa_r+0x13b4>
8000d458:	00000000 	nop
8000d45c:	00000000 	nop
8000d460:	10000002 	b	8000d46c <_dtoa_r+0x13b4>
8000d464:	00000000 	nop
8000d468:	00000000 	nop
8000d46c:	8fc200f8 	lw	v0,248(s8)
8000d470:	00021027 	nor	v0,zero,v0
8000d474:	afc20040 	sw	v0,64(s8)
8000d478:	10000178 	b	8000da5c <_dtoa_r+0x19a4>
8000d47c:	00000000 	nop
8000d480:	00000000 	nop
8000d484:	10000002 	b	8000d490 <_dtoa_r+0x13d8>
8000d488:	00000000 	nop
8000d48c:	00000000 	nop
8000d490:	8fc20080 	lw	v0,128(s8)
8000d494:	24430001 	addiu	v1,v0,1
8000d498:	afc30080 	sw	v1,128(s8)
8000d49c:	24030031 	li	v1,49
8000d4a0:	a0430000 	sb	v1,0(v0)
8000d4a4:	8fc20040 	lw	v0,64(s8)
8000d4a8:	24420001 	addiu	v0,v0,1
8000d4ac:	afc20040 	sw	v0,64(s8)
8000d4b0:	1000016a 	b	8000da5c <_dtoa_r+0x19a4>
8000d4b4:	00000000 	nop
8000d4b8:	8fc20048 	lw	v0,72(s8)
8000d4bc:	104000fe 	beqz	v0,8000d8b8 <_dtoa_r+0x1800>
8000d4c0:	00000000 	nop
8000d4c4:	8fc2004c 	lw	v0,76(s8)
8000d4c8:	18400007 	blez	v0,8000d4e8 <_dtoa_r+0x1430>
8000d4cc:	00000000 	nop
8000d4d0:	8fc6004c 	lw	a2,76(s8)
8000d4d4:	8fc50070 	lw	a1,112(s8)
8000d4d8:	8fc400e0 	lw	a0,224(s8)
8000d4dc:	0c003eea 	jal	8000fba8 <__lshift>
8000d4e0:	00000000 	nop
8000d4e4:	afc20070 	sw	v0,112(s8)
8000d4e8:	8fc20070 	lw	v0,112(s8)
8000d4ec:	afc2006c 	sw	v0,108(s8)
8000d4f0:	8fc2005c 	lw	v0,92(s8)
8000d4f4:	10400028 	beqz	v0,8000d598 <_dtoa_r+0x14e0>
8000d4f8:	00000000 	nop
8000d4fc:	8fc20070 	lw	v0,112(s8)
8000d500:	8c420004 	lw	v0,4(v0)
8000d504:	00402825 	move	a1,v0
8000d508:	8fc400e0 	lw	a0,224(s8)
8000d50c:	0c003b5b 	jal	8000ed6c <_Balloc>
8000d510:	00000000 	nop
8000d514:	afc200a4 	sw	v0,164(s8)
8000d518:	8fc200a4 	lw	v0,164(s8)
8000d51c:	14400009 	bnez	v0,8000d544 <_dtoa_r+0x148c>
8000d520:	00000000 	nop
8000d524:	3c028002 	lui	v0,0x8002
8000d528:	2447a710 	addiu	a3,v0,-22768
8000d52c:	00003025 	move	a2,zero
8000d530:	240502ea 	li	a1,746
8000d534:	3c028002 	lui	v0,0x8002
8000d538:	2444a728 	addiu	a0,v0,-22744
8000d53c:	0c0068d2 	jal	8001a348 <__assert_func>
8000d540:	00000000 	nop
8000d544:	8fc200a4 	lw	v0,164(s8)
8000d548:	afc20070 	sw	v0,112(s8)
8000d54c:	8fc20070 	lw	v0,112(s8)
8000d550:	2443000c 	addiu	v1,v0,12
8000d554:	8fc2006c 	lw	v0,108(s8)
8000d558:	2444000c 	addiu	a0,v0,12
8000d55c:	8fc2006c 	lw	v0,108(s8)
8000d560:	8c420010 	lw	v0,16(v0)
8000d564:	24420002 	addiu	v0,v0,2
8000d568:	00021080 	sll	v0,v0,0x2
8000d56c:	00403025 	move	a2,v0
8000d570:	00802825 	move	a1,a0
8000d574:	00602025 	move	a0,v1
8000d578:	0c0015cf 	jal	8000573c <memcpy>
8000d57c:	00000000 	nop
8000d580:	24060001 	li	a2,1
8000d584:	8fc50070 	lw	a1,112(s8)
8000d588:	8fc400e0 	lw	a0,224(s8)
8000d58c:	0c003eea 	jal	8000fba8 <__lshift>
8000d590:	00000000 	nop
8000d594:	afc20070 	sw	v0,112(s8)
8000d598:	24020001 	li	v0,1
8000d59c:	afc2002c 	sw	v0,44(s8)
8000d5a0:	8fc50074 	lw	a1,116(s8)
8000d5a4:	8fc40068 	lw	a0,104(s8)
8000d5a8:	0c002f2d 	jal	8000bcb4 <quorem>
8000d5ac:	00000000 	nop
8000d5b0:	24420030 	addiu	v0,v0,48
8000d5b4:	afc20028 	sw	v0,40(s8)
8000d5b8:	8fc5006c 	lw	a1,108(s8)
8000d5bc:	8fc40068 	lw	a0,104(s8)
8000d5c0:	0c003f83 	jal	8000fe0c <__mcmp>
8000d5c4:	00000000 	nop
8000d5c8:	afc2003c 	sw	v0,60(s8)
8000d5cc:	8fc60070 	lw	a2,112(s8)
8000d5d0:	8fc50074 	lw	a1,116(s8)
8000d5d4:	8fc400e0 	lw	a0,224(s8)
8000d5d8:	0c003fcf 	jal	8000ff3c <__mdiff>
8000d5dc:	00000000 	nop
8000d5e0:	afc200a8 	sw	v0,168(s8)
8000d5e4:	8fc200a8 	lw	v0,168(s8)
8000d5e8:	8c42000c 	lw	v0,12(v0)
8000d5ec:	14400007 	bnez	v0,8000d60c <_dtoa_r+0x1554>
8000d5f0:	00000000 	nop
8000d5f4:	8fc500a8 	lw	a1,168(s8)
8000d5f8:	8fc40068 	lw	a0,104(s8)
8000d5fc:	0c003f83 	jal	8000fe0c <__mcmp>
8000d600:	00000000 	nop
8000d604:	10000002 	b	8000d610 <_dtoa_r+0x1558>
8000d608:	00000000 	nop
8000d60c:	24020001 	li	v0,1
8000d610:	afc20098 	sw	v0,152(s8)
8000d614:	8fc500a8 	lw	a1,168(s8)
8000d618:	8fc400e0 	lw	a0,224(s8)
8000d61c:	0c003bae 	jal	8000eeb8 <_Bfree>
8000d620:	00000000 	nop
8000d624:	8fc20098 	lw	v0,152(s8)
8000d628:	1440001b 	bnez	v0,8000d698 <_dtoa_r+0x15e0>
8000d62c:	00000000 	nop
8000d630:	8fc200f0 	lw	v0,240(s8)
8000d634:	14400018 	bnez	v0,8000d698 <_dtoa_r+0x15e0>
8000d638:	00000000 	nop
8000d63c:	8fc200bc 	lw	v0,188(s8)
8000d640:	30420001 	andi	v0,v0,0x1
8000d644:	14400014 	bnez	v0,8000d698 <_dtoa_r+0x15e0>
8000d648:	00000000 	nop
8000d64c:	8fc30028 	lw	v1,40(s8)
8000d650:	24020039 	li	v0,57
8000d654:	1062004d 	beq	v1,v0,8000d78c <_dtoa_r+0x16d4>
8000d658:	00000000 	nop
8000d65c:	8fc2003c 	lw	v0,60(s8)
8000d660:	18400004 	blez	v0,8000d674 <_dtoa_r+0x15bc>
8000d664:	00000000 	nop
8000d668:	8fc20028 	lw	v0,40(s8)
8000d66c:	24420001 	addiu	v0,v0,1
8000d670:	afc20028 	sw	v0,40(s8)
8000d674:	8fc20080 	lw	v0,128(s8)
8000d678:	24430001 	addiu	v1,v0,1
8000d67c:	afc30080 	sw	v1,128(s8)
8000d680:	8fc30028 	lw	v1,40(s8)
8000d684:	00031e00 	sll	v1,v1,0x18
8000d688:	00031e03 	sra	v1,v1,0x18
8000d68c:	a0430000 	sb	v1,0(v0)
8000d690:	100000f2 	b	8000da5c <_dtoa_r+0x19a4>
8000d694:	00000000 	nop
8000d698:	8fc2003c 	lw	v0,60(s8)
8000d69c:	0440000b 	bltz	v0,8000d6cc <_dtoa_r+0x1614>
8000d6a0:	00000000 	nop
8000d6a4:	8fc2003c 	lw	v0,60(s8)
8000d6a8:	1440002f 	bnez	v0,8000d768 <_dtoa_r+0x16b0>
8000d6ac:	00000000 	nop
8000d6b0:	8fc200f0 	lw	v0,240(s8)
8000d6b4:	1440002c 	bnez	v0,8000d768 <_dtoa_r+0x16b0>
8000d6b8:	00000000 	nop
8000d6bc:	8fc200bc 	lw	v0,188(s8)
8000d6c0:	30420001 	andi	v0,v0,0x1
8000d6c4:	14400028 	bnez	v0,8000d768 <_dtoa_r+0x16b0>
8000d6c8:	00000000 	nop
8000d6cc:	8fc20098 	lw	v0,152(s8)
8000d6d0:	1840001c 	blez	v0,8000d744 <_dtoa_r+0x168c>
8000d6d4:	00000000 	nop
8000d6d8:	24060001 	li	a2,1
8000d6dc:	8fc50068 	lw	a1,104(s8)
8000d6e0:	8fc400e0 	lw	a0,224(s8)
8000d6e4:	0c003eea 	jal	8000fba8 <__lshift>
8000d6e8:	00000000 	nop
8000d6ec:	afc20068 	sw	v0,104(s8)
8000d6f0:	8fc50074 	lw	a1,116(s8)
8000d6f4:	8fc40068 	lw	a0,104(s8)
8000d6f8:	0c003f83 	jal	8000fe0c <__mcmp>
8000d6fc:	00000000 	nop
8000d700:	afc20098 	sw	v0,152(s8)
8000d704:	8fc20098 	lw	v0,152(s8)
8000d708:	1c400008 	bgtz	v0,8000d72c <_dtoa_r+0x1674>
8000d70c:	00000000 	nop
8000d710:	8fc20098 	lw	v0,152(s8)
8000d714:	1440000b 	bnez	v0,8000d744 <_dtoa_r+0x168c>
8000d718:	00000000 	nop
8000d71c:	8fc20028 	lw	v0,40(s8)
8000d720:	30420001 	andi	v0,v0,0x1
8000d724:	10400007 	beqz	v0,8000d744 <_dtoa_r+0x168c>
8000d728:	00000000 	nop
8000d72c:	8fc20028 	lw	v0,40(s8)
8000d730:	24430001 	addiu	v1,v0,1
8000d734:	afc30028 	sw	v1,40(s8)
8000d738:	24030039 	li	v1,57
8000d73c:	10430016 	beq	v0,v1,8000d798 <_dtoa_r+0x16e0>
8000d740:	00000000 	nop
8000d744:	8fc20080 	lw	v0,128(s8)
8000d748:	24430001 	addiu	v1,v0,1
8000d74c:	afc30080 	sw	v1,128(s8)
8000d750:	8fc30028 	lw	v1,40(s8)
8000d754:	00031e00 	sll	v1,v1,0x18
8000d758:	00031e03 	sra	v1,v1,0x18
8000d75c:	a0430000 	sb	v1,0(v0)
8000d760:	100000be 	b	8000da5c <_dtoa_r+0x19a4>
8000d764:	00000000 	nop
8000d768:	8fc20098 	lw	v0,152(s8)
8000d76c:	1840001e 	blez	v0,8000d7e8 <_dtoa_r+0x1730>
8000d770:	00000000 	nop
8000d774:	8fc30028 	lw	v1,40(s8)
8000d778:	24020039 	li	v0,57
8000d77c:	1462000e 	bne	v1,v0,8000d7b8 <_dtoa_r+0x1700>
8000d780:	00000000 	nop
8000d784:	10000005 	b	8000d79c <_dtoa_r+0x16e4>
8000d788:	00000000 	nop
8000d78c:	00000000 	nop
8000d790:	10000002 	b	8000d79c <_dtoa_r+0x16e4>
8000d794:	00000000 	nop
8000d798:	00000000 	nop
8000d79c:	8fc20080 	lw	v0,128(s8)
8000d7a0:	24430001 	addiu	v1,v0,1
8000d7a4:	afc30080 	sw	v1,128(s8)
8000d7a8:	24030039 	li	v1,57
8000d7ac:	a0430000 	sb	v1,0(v0)
8000d7b0:	1000007b 	b	8000d9a0 <_dtoa_r+0x18e8>
8000d7b4:	00000000 	nop
8000d7b8:	8fc20028 	lw	v0,40(s8)
8000d7bc:	304200ff 	andi	v0,v0,0xff
8000d7c0:	24420001 	addiu	v0,v0,1
8000d7c4:	304400ff 	andi	a0,v0,0xff
8000d7c8:	8fc20080 	lw	v0,128(s8)
8000d7cc:	24430001 	addiu	v1,v0,1
8000d7d0:	afc30080 	sw	v1,128(s8)
8000d7d4:	00041e00 	sll	v1,a0,0x18
8000d7d8:	00031e03 	sra	v1,v1,0x18
8000d7dc:	a0430000 	sb	v1,0(v0)
8000d7e0:	1000009e 	b	8000da5c <_dtoa_r+0x19a4>
8000d7e4:	00000000 	nop
8000d7e8:	8fc20080 	lw	v0,128(s8)
8000d7ec:	24430001 	addiu	v1,v0,1
8000d7f0:	afc30080 	sw	v1,128(s8)
8000d7f4:	8fc30028 	lw	v1,40(s8)
8000d7f8:	00031e00 	sll	v1,v1,0x18
8000d7fc:	00031e03 	sra	v1,v1,0x18
8000d800:	a0430000 	sb	v1,0(v0)
8000d804:	8fc3002c 	lw	v1,44(s8)
8000d808:	8fc20034 	lw	v0,52(s8)
8000d80c:	1062004a 	beq	v1,v0,8000d938 <_dtoa_r+0x1880>
8000d810:	00000000 	nop
8000d814:	00003825 	move	a3,zero
8000d818:	2406000a 	li	a2,10
8000d81c:	8fc50068 	lw	a1,104(s8)
8000d820:	8fc400e0 	lw	a0,224(s8)
8000d824:	0c003bcd 	jal	8000ef34 <__multadd>
8000d828:	00000000 	nop
8000d82c:	afc20068 	sw	v0,104(s8)
8000d830:	8fc3006c 	lw	v1,108(s8)
8000d834:	8fc20070 	lw	v0,112(s8)
8000d838:	1462000c 	bne	v1,v0,8000d86c <_dtoa_r+0x17b4>
8000d83c:	00000000 	nop
8000d840:	00003825 	move	a3,zero
8000d844:	2406000a 	li	a2,10
8000d848:	8fc50070 	lw	a1,112(s8)
8000d84c:	8fc400e0 	lw	a0,224(s8)
8000d850:	0c003bcd 	jal	8000ef34 <__multadd>
8000d854:	00000000 	nop
8000d858:	afc20070 	sw	v0,112(s8)
8000d85c:	8fc20070 	lw	v0,112(s8)
8000d860:	afc2006c 	sw	v0,108(s8)
8000d864:	1000000f 	b	8000d8a4 <_dtoa_r+0x17ec>
8000d868:	00000000 	nop
8000d86c:	00003825 	move	a3,zero
8000d870:	2406000a 	li	a2,10
8000d874:	8fc5006c 	lw	a1,108(s8)
8000d878:	8fc400e0 	lw	a0,224(s8)
8000d87c:	0c003bcd 	jal	8000ef34 <__multadd>
8000d880:	00000000 	nop
8000d884:	afc2006c 	sw	v0,108(s8)
8000d888:	00003825 	move	a3,zero
8000d88c:	2406000a 	li	a2,10
8000d890:	8fc50070 	lw	a1,112(s8)
8000d894:	8fc400e0 	lw	a0,224(s8)
8000d898:	0c003bcd 	jal	8000ef34 <__multadd>
8000d89c:	00000000 	nop
8000d8a0:	afc20070 	sw	v0,112(s8)
8000d8a4:	8fc2002c 	lw	v0,44(s8)
8000d8a8:	24420001 	addiu	v0,v0,1
8000d8ac:	afc2002c 	sw	v0,44(s8)
8000d8b0:	1000ff3b 	b	8000d5a0 <_dtoa_r+0x14e8>
8000d8b4:	00000000 	nop
8000d8b8:	24020001 	li	v0,1
8000d8bc:	afc2002c 	sw	v0,44(s8)
8000d8c0:	8fc50074 	lw	a1,116(s8)
8000d8c4:	8fc40068 	lw	a0,104(s8)
8000d8c8:	0c002f2d 	jal	8000bcb4 <quorem>
8000d8cc:	00000000 	nop
8000d8d0:	24420030 	addiu	v0,v0,48
8000d8d4:	afc20028 	sw	v0,40(s8)
8000d8d8:	8fc20080 	lw	v0,128(s8)
8000d8dc:	24430001 	addiu	v1,v0,1
8000d8e0:	afc30080 	sw	v1,128(s8)
8000d8e4:	8fc30028 	lw	v1,40(s8)
8000d8e8:	00031e00 	sll	v1,v1,0x18
8000d8ec:	00031e03 	sra	v1,v1,0x18
8000d8f0:	a0430000 	sb	v1,0(v0)
8000d8f4:	8fc3002c 	lw	v1,44(s8)
8000d8f8:	8fc20034 	lw	v0,52(s8)
8000d8fc:	0062102a 	slt	v0,v1,v0
8000d900:	10400010 	beqz	v0,8000d944 <_dtoa_r+0x188c>
8000d904:	00000000 	nop
8000d908:	00003825 	move	a3,zero
8000d90c:	2406000a 	li	a2,10
8000d910:	8fc50068 	lw	a1,104(s8)
8000d914:	8fc400e0 	lw	a0,224(s8)
8000d918:	0c003bcd 	jal	8000ef34 <__multadd>
8000d91c:	00000000 	nop
8000d920:	afc20068 	sw	v0,104(s8)
8000d924:	8fc2002c 	lw	v0,44(s8)
8000d928:	24420001 	addiu	v0,v0,1
8000d92c:	afc2002c 	sw	v0,44(s8)
8000d930:	1000ffe3 	b	8000d8c0 <_dtoa_r+0x1808>
8000d934:	00000000 	nop
8000d938:	00000000 	nop
8000d93c:	10000002 	b	8000d948 <_dtoa_r+0x1890>
8000d940:	00000000 	nop
8000d944:	00000000 	nop
8000d948:	24060001 	li	a2,1
8000d94c:	8fc50068 	lw	a1,104(s8)
8000d950:	8fc400e0 	lw	a0,224(s8)
8000d954:	0c003eea 	jal	8000fba8 <__lshift>
8000d958:	00000000 	nop
8000d95c:	afc20068 	sw	v0,104(s8)
8000d960:	8fc50074 	lw	a1,116(s8)
8000d964:	8fc40068 	lw	a0,104(s8)
8000d968:	0c003f83 	jal	8000fe0c <__mcmp>
8000d96c:	00000000 	nop
8000d970:	afc2003c 	sw	v0,60(s8)
8000d974:	8fc2003c 	lw	v0,60(s8)
8000d978:	1c400008 	bgtz	v0,8000d99c <_dtoa_r+0x18e4>
8000d97c:	00000000 	nop
8000d980:	8fc2003c 	lw	v0,60(s8)
8000d984:	1440002a 	bnez	v0,8000da30 <_dtoa_r+0x1978>
8000d988:	00000000 	nop
8000d98c:	8fc20028 	lw	v0,40(s8)
8000d990:	30420001 	andi	v0,v0,0x1
8000d994:	10400026 	beqz	v0,8000da30 <_dtoa_r+0x1978>
8000d998:	00000000 	nop
8000d99c:	00000000 	nop
8000d9a0:	1000000f 	b	8000d9e0 <_dtoa_r+0x1928>
8000d9a4:	00000000 	nop
8000d9a8:	8fc30080 	lw	v1,128(s8)
8000d9ac:	8fc2008c 	lw	v0,140(s8)
8000d9b0:	1462000b 	bne	v1,v0,8000d9e0 <_dtoa_r+0x1928>
8000d9b4:	00000000 	nop
8000d9b8:	8fc20040 	lw	v0,64(s8)
8000d9bc:	24420001 	addiu	v0,v0,1
8000d9c0:	afc20040 	sw	v0,64(s8)
8000d9c4:	8fc20080 	lw	v0,128(s8)
8000d9c8:	24430001 	addiu	v1,v0,1
8000d9cc:	afc30080 	sw	v1,128(s8)
8000d9d0:	24030031 	li	v1,49
8000d9d4:	a0430000 	sb	v1,0(v0)
8000d9d8:	10000020 	b	8000da5c <_dtoa_r+0x19a4>
8000d9dc:	00000000 	nop
8000d9e0:	8fc20080 	lw	v0,128(s8)
8000d9e4:	2442ffff 	addiu	v0,v0,-1
8000d9e8:	afc20080 	sw	v0,128(s8)
8000d9ec:	8fc20080 	lw	v0,128(s8)
8000d9f0:	80430000 	lb	v1,0(v0)
8000d9f4:	24020039 	li	v0,57
8000d9f8:	1062ffeb 	beq	v1,v0,8000d9a8 <_dtoa_r+0x18f0>
8000d9fc:	00000000 	nop
8000da00:	8fc20080 	lw	v0,128(s8)
8000da04:	24430001 	addiu	v1,v0,1
8000da08:	afc30080 	sw	v1,128(s8)
8000da0c:	80430000 	lb	v1,0(v0)
8000da10:	306300ff 	andi	v1,v1,0xff
8000da14:	24630001 	addiu	v1,v1,1
8000da18:	306300ff 	andi	v1,v1,0xff
8000da1c:	00031e00 	sll	v1,v1,0x18
8000da20:	00031e03 	sra	v1,v1,0x18
8000da24:	a0430000 	sb	v1,0(v0)
8000da28:	1000000c 	b	8000da5c <_dtoa_r+0x19a4>
8000da2c:	00000000 	nop
8000da30:	8fc20080 	lw	v0,128(s8)
8000da34:	2442ffff 	addiu	v0,v0,-1
8000da38:	afc20080 	sw	v0,128(s8)
8000da3c:	8fc20080 	lw	v0,128(s8)
8000da40:	80430000 	lb	v1,0(v0)
8000da44:	24020030 	li	v0,48
8000da48:	1062fff9 	beq	v1,v0,8000da30 <_dtoa_r+0x1978>
8000da4c:	00000000 	nop
8000da50:	8fc20080 	lw	v0,128(s8)
8000da54:	24420001 	addiu	v0,v0,1
8000da58:	afc20080 	sw	v0,128(s8)
8000da5c:	8fc50074 	lw	a1,116(s8)
8000da60:	8fc400e0 	lw	a0,224(s8)
8000da64:	0c003bae 	jal	8000eeb8 <_Bfree>
8000da68:	00000000 	nop
8000da6c:	8fc20070 	lw	v0,112(s8)
8000da70:	10400015 	beqz	v0,8000dac8 <_dtoa_r+0x1a10>
8000da74:	00000000 	nop
8000da78:	8fc2006c 	lw	v0,108(s8)
8000da7c:	10400009 	beqz	v0,8000daa4 <_dtoa_r+0x19ec>
8000da80:	00000000 	nop
8000da84:	8fc3006c 	lw	v1,108(s8)
8000da88:	8fc20070 	lw	v0,112(s8)
8000da8c:	10620005 	beq	v1,v0,8000daa4 <_dtoa_r+0x19ec>
8000da90:	00000000 	nop
8000da94:	8fc5006c 	lw	a1,108(s8)
8000da98:	8fc400e0 	lw	a0,224(s8)
8000da9c:	0c003bae 	jal	8000eeb8 <_Bfree>
8000daa0:	00000000 	nop
8000daa4:	8fc50070 	lw	a1,112(s8)
8000daa8:	8fc400e0 	lw	a0,224(s8)
8000daac:	0c003bae 	jal	8000eeb8 <_Bfree>
8000dab0:	00000000 	nop
8000dab4:	10000005 	b	8000dacc <_dtoa_r+0x1a14>
8000dab8:	00000000 	nop
8000dabc:	00000000 	nop
8000dac0:	10000002 	b	8000dacc <_dtoa_r+0x1a14>
8000dac4:	00000000 	nop
8000dac8:	00000000 	nop
8000dacc:	8fc50068 	lw	a1,104(s8)
8000dad0:	8fc400e0 	lw	a0,224(s8)
8000dad4:	0c003bae 	jal	8000eeb8 <_Bfree>
8000dad8:	00000000 	nop
8000dadc:	8fc20080 	lw	v0,128(s8)
8000dae0:	a0400000 	sb	zero,0(v0)
8000dae4:	8fc20040 	lw	v0,64(s8)
8000dae8:	24430001 	addiu	v1,v0,1
8000daec:	8fc20104 	lw	v0,260(s8)
8000daf0:	ac430000 	sw	v1,0(v0)
8000daf4:	8fc20114 	lw	v0,276(s8)
8000daf8:	10400004 	beqz	v0,8000db0c <_dtoa_r+0x1a54>
8000dafc:	00000000 	nop
8000db00:	8fc20114 	lw	v0,276(s8)
8000db04:	8fc30080 	lw	v1,128(s8)
8000db08:	ac430000 	sw	v1,0(v0)
8000db0c:	8fc2008c 	lw	v0,140(s8)
8000db10:	03c0e825 	move	sp,s8
8000db14:	dfbf00d8 	ld	ra,216(sp)
8000db18:	dfbe00d0 	ld	s8,208(sp)
8000db1c:	27bd00e0 	addiu	sp,sp,224
8000db20:	03e00008 	jr	ra
8000db24:	00000000 	nop

8000db28 <_fclose_r>:
8000db28:	27bdffc0 	addiu	sp,sp,-64
8000db2c:	ffbf0038 	sd	ra,56(sp)
8000db30:	ffbe0030 	sd	s8,48(sp)
8000db34:	ffb00028 	sd	s0,40(sp)
8000db38:	03a0f025 	move	s8,sp
8000db3c:	afc40040 	sw	a0,64(s8)
8000db40:	00a08025 	move	s0,a1
8000db44:	16000004 	bnez	s0,8000db58 <_fclose_r+0x30>
8000db48:	00000000 	nop
8000db4c:	00001025 	move	v0,zero
8000db50:	1000004b 	b	8000dc80 <_fclose_r+0x158>
8000db54:	00000000 	nop
8000db58:	8fc20040 	lw	v0,64(s8)
8000db5c:	afc20024 	sw	v0,36(s8)
8000db60:	8fc20024 	lw	v0,36(s8)
8000db64:	10400008 	beqz	v0,8000db88 <_fclose_r+0x60>
8000db68:	00000000 	nop
8000db6c:	8fc20024 	lw	v0,36(s8)
8000db70:	8c420038 	lw	v0,56(v0)
8000db74:	14400004 	bnez	v0,8000db88 <_fclose_r+0x60>
8000db78:	00000000 	nop
8000db7c:	8fc40024 	lw	a0,36(s8)
8000db80:	0c000cc3 	jal	8000330c <__sinit>
8000db84:	00000000 	nop
8000db88:	8602000c 	lh	v0,12(s0)
8000db8c:	14400004 	bnez	v0,8000dba0 <_fclose_r+0x78>
8000db90:	00000000 	nop
8000db94:	00001025 	move	v0,zero
8000db98:	10000039 	b	8000dc80 <_fclose_r+0x158>
8000db9c:	00000000 	nop
8000dba0:	02002825 	move	a1,s0
8000dba4:	8fc40040 	lw	a0,64(s8)
8000dba8:	0c000a57 	jal	8000295c <__sflush_r>
8000dbac:	00000000 	nop
8000dbb0:	afc20020 	sw	v0,32(s8)
8000dbb4:	8e02002c 	lw	v0,44(s0)
8000dbb8:	1040000b 	beqz	v0,8000dbe8 <_fclose_r+0xc0>
8000dbbc:	00000000 	nop
8000dbc0:	8e02002c 	lw	v0,44(s0)
8000dbc4:	8e03001c 	lw	v1,28(s0)
8000dbc8:	00602825 	move	a1,v1
8000dbcc:	8fc40040 	lw	a0,64(s8)
8000dbd0:	0040f809 	jalr	v0
8000dbd4:	00000000 	nop
8000dbd8:	04410003 	bgez	v0,8000dbe8 <_fclose_r+0xc0>
8000dbdc:	00000000 	nop
8000dbe0:	2402ffff 	li	v0,-1
8000dbe4:	afc20020 	sw	v0,32(s8)
8000dbe8:	8602000c 	lh	v0,12(s0)
8000dbec:	3042ffff 	andi	v0,v0,0xffff
8000dbf0:	30420080 	andi	v0,v0,0x80
8000dbf4:	10400006 	beqz	v0,8000dc10 <_fclose_r+0xe8>
8000dbf8:	00000000 	nop
8000dbfc:	8e020010 	lw	v0,16(s0)
8000dc00:	00402825 	move	a1,v0
8000dc04:	8fc40040 	lw	a0,64(s8)
8000dc08:	0c000dcb 	jal	8000372c <_free_r>
8000dc0c:	00000000 	nop
8000dc10:	8e020030 	lw	v0,48(s0)
8000dc14:	1040000b 	beqz	v0,8000dc44 <_fclose_r+0x11c>
8000dc18:	00000000 	nop
8000dc1c:	8e030030 	lw	v1,48(s0)
8000dc20:	26020040 	addiu	v0,s0,64
8000dc24:	10620006 	beq	v1,v0,8000dc40 <_fclose_r+0x118>
8000dc28:	00000000 	nop
8000dc2c:	8e020030 	lw	v0,48(s0)
8000dc30:	00402825 	move	a1,v0
8000dc34:	8fc40040 	lw	a0,64(s8)
8000dc38:	0c000dcb 	jal	8000372c <_free_r>
8000dc3c:	00000000 	nop
8000dc40:	ae000030 	sw	zero,48(s0)
8000dc44:	8e020044 	lw	v0,68(s0)
8000dc48:	10400007 	beqz	v0,8000dc68 <_fclose_r+0x140>
8000dc4c:	00000000 	nop
8000dc50:	8e020044 	lw	v0,68(s0)
8000dc54:	00402825 	move	a1,v0
8000dc58:	8fc40040 	lw	a0,64(s8)
8000dc5c:	0c000dcb 	jal	8000372c <_free_r>
8000dc60:	00000000 	nop
8000dc64:	ae000044 	sw	zero,68(s0)
8000dc68:	0c000cf9 	jal	800033e4 <__sfp_lock_acquire>
8000dc6c:	00000000 	nop
8000dc70:	a600000c 	sh	zero,12(s0)
8000dc74:	0c000d02 	jal	80003408 <__sfp_lock_release>
8000dc78:	00000000 	nop
8000dc7c:	8fc20020 	lw	v0,32(s8)
8000dc80:	03c0e825 	move	sp,s8
8000dc84:	dfbf0038 	ld	ra,56(sp)
8000dc88:	dfbe0030 	ld	s8,48(sp)
8000dc8c:	dfb00028 	ld	s0,40(sp)
8000dc90:	27bd0040 	addiu	sp,sp,64
8000dc94:	03e00008 	jr	ra
8000dc98:	00000000 	nop

8000dc9c <fclose>:
8000dc9c:	27bdffd0 	addiu	sp,sp,-48
8000dca0:	ffbf0028 	sd	ra,40(sp)
8000dca4:	ffbe0020 	sd	s8,32(sp)
8000dca8:	03a0f025 	move	s8,sp
8000dcac:	00801825 	move	v1,a0
8000dcb0:	8f828028 	lw	v0,-32728(gp)
8000dcb4:	00602825 	move	a1,v1
8000dcb8:	00402025 	move	a0,v0
8000dcbc:	0c0036ca 	jal	8000db28 <_fclose_r>
8000dcc0:	00000000 	nop
8000dcc4:	03c0e825 	move	sp,s8
8000dcc8:	dfbf0028 	ld	ra,40(sp)
8000dccc:	dfbe0020 	ld	s8,32(sp)
8000dcd0:	27bd0030 	addiu	sp,sp,48
8000dcd4:	03e00008 	jr	ra
8000dcd8:	00000000 	nop

8000dcdc <__sfvwrite_r>:
8000dcdc:	27bdff88 	addiu	sp,sp,-120
8000dce0:	ffbf0070 	sd	ra,112(sp)
8000dce4:	ffbe0068 	sd	s8,104(sp)
8000dce8:	ffb50060 	sd	s5,96(sp)
8000dcec:	ffb40058 	sd	s4,88(sp)
8000dcf0:	ffb30050 	sd	s3,80(sp)
8000dcf4:	ffb20048 	sd	s2,72(sp)
8000dcf8:	ffb10040 	sd	s1,64(sp)
8000dcfc:	ffb00038 	sd	s0,56(sp)
8000dd00:	03a0f025 	move	s8,sp
8000dd04:	afc40078 	sw	a0,120(s8)
8000dd08:	00a08025 	move	s0,a1
8000dd0c:	00c0a825 	move	s5,a2
8000dd10:	00009825 	move	s3,zero
8000dd14:	8eb20008 	lw	s2,8(s5)
8000dd18:	16400004 	bnez	s2,8000dd2c <__sfvwrite_r+0x50>
8000dd1c:	00000000 	nop
8000dd20:	00001025 	move	v0,zero
8000dd24:	100001b7 	b	8000e404 <__sfvwrite_r+0x728>
8000dd28:	00000000 	nop
8000dd2c:	8602000c 	lh	v0,12(s0)
8000dd30:	3042ffff 	andi	v0,v0,0xffff
8000dd34:	30420008 	andi	v0,v0,0x8
8000dd38:	10400004 	beqz	v0,8000dd4c <__sfvwrite_r+0x70>
8000dd3c:	00000000 	nop
8000dd40:	8e020010 	lw	v0,16(s0)
8000dd44:	1440000a 	bnez	v0,8000dd70 <__sfvwrite_r+0x94>
8000dd48:	00000000 	nop
8000dd4c:	02002825 	move	a1,s0
8000dd50:	8fc40078 	lw	a0,120(s8)
8000dd54:	0c002e52 	jal	8000b948 <__swsetup_r>
8000dd58:	00000000 	nop
8000dd5c:	10400004 	beqz	v0,8000dd70 <__sfvwrite_r+0x94>
8000dd60:	00000000 	nop
8000dd64:	2402ffff 	li	v0,-1
8000dd68:	100001a6 	b	8000e404 <__sfvwrite_r+0x728>
8000dd6c:	00000000 	nop
8000dd70:	8eb40000 	lw	s4,0(s5)
8000dd74:	00009025 	move	s2,zero
8000dd78:	8602000c 	lh	v0,12(s0)
8000dd7c:	3042ffff 	andi	v0,v0,0xffff
8000dd80:	30420002 	andi	v0,v0,0x2
8000dd84:	10400027 	beqz	v0,8000de24 <__sfvwrite_r+0x148>
8000dd88:	00000000 	nop
8000dd8c:	10000004 	b	8000dda0 <__sfvwrite_r+0xc4>
8000dd90:	00000000 	nop
8000dd94:	8e930000 	lw	s3,0(s4)
8000dd98:	8e920004 	lw	s2,4(s4)
8000dd9c:	26940008 	addiu	s4,s4,8
8000dda0:	1240fffc 	beqz	s2,8000dd94 <__sfvwrite_r+0xb8>
8000dda4:	00000000 	nop
8000dda8:	8e030024 	lw	v1,36(s0)
8000ddac:	8e05001c 	lw	a1,28(s0)
8000ddb0:	02401025 	move	v0,s2
8000ddb4:	3c047fff 	lui	a0,0x7fff
8000ddb8:	3484fc01 	ori	a0,a0,0xfc01
8000ddbc:	0044202b 	sltu	a0,v0,a0
8000ddc0:	14800003 	bnez	a0,8000ddd0 <__sfvwrite_r+0xf4>
8000ddc4:	00000000 	nop
8000ddc8:	3c027fff 	lui	v0,0x7fff
8000ddcc:	3442fc00 	ori	v0,v0,0xfc00
8000ddd0:	00403825 	move	a3,v0
8000ddd4:	02603025 	move	a2,s3
8000ddd8:	8fc40078 	lw	a0,120(s8)
8000dddc:	0060f809 	jalr	v1
8000dde0:	00000000 	nop
8000dde4:	00408825 	move	s1,v0
8000dde8:	1a200179 	blez	s1,8000e3d0 <__sfvwrite_r+0x6f4>
8000ddec:	00000000 	nop
8000ddf0:	02201025 	move	v0,s1
8000ddf4:	02629821 	addu	s3,s3,v0
8000ddf8:	02201025 	move	v0,s1
8000ddfc:	02429023 	subu	s2,s2,v0
8000de00:	8ea20008 	lw	v0,8(s5)
8000de04:	02201825 	move	v1,s1
8000de08:	00431023 	subu	v0,v0,v1
8000de0c:	aea20008 	sw	v0,8(s5)
8000de10:	8ea20008 	lw	v0,8(s5)
8000de14:	1440ffe2 	bnez	v0,8000dda0 <__sfvwrite_r+0xc4>
8000de18:	00000000 	nop
8000de1c:	10000169 	b	8000e3c4 <__sfvwrite_r+0x6e8>
8000de20:	00000000 	nop
8000de24:	8602000c 	lh	v0,12(s0)
8000de28:	3042ffff 	andi	v0,v0,0xffff
8000de2c:	30420001 	andi	v0,v0,0x1
8000de30:	144000e8 	bnez	v0,8000e1d4 <__sfvwrite_r+0x4f8>
8000de34:	00000000 	nop
8000de38:	10000004 	b	8000de4c <__sfvwrite_r+0x170>
8000de3c:	00000000 	nop
8000de40:	8e930000 	lw	s3,0(s4)
8000de44:	8e920004 	lw	s2,4(s4)
8000de48:	26940008 	addiu	s4,s4,8
8000de4c:	1240fffc 	beqz	s2,8000de40 <__sfvwrite_r+0x164>
8000de50:	00000000 	nop
8000de54:	8e110008 	lw	s1,8(s0)
8000de58:	8602000c 	lh	v0,12(s0)
8000de5c:	3042ffff 	andi	v0,v0,0xffff
8000de60:	30420200 	andi	v0,v0,0x200
8000de64:	10400084 	beqz	v0,8000e078 <__sfvwrite_r+0x39c>
8000de68:	00000000 	nop
8000de6c:	02201025 	move	v0,s1
8000de70:	0242102b 	sltu	v0,s2,v0
8000de74:	1440006a 	bnez	v0,8000e020 <__sfvwrite_r+0x344>
8000de78:	00000000 	nop
8000de7c:	8602000c 	lh	v0,12(s0)
8000de80:	3042ffff 	andi	v0,v0,0xffff
8000de84:	30420480 	andi	v0,v0,0x480
8000de88:	10400065 	beqz	v0,8000e020 <__sfvwrite_r+0x344>
8000de8c:	00000000 	nop
8000de90:	8e030000 	lw	v1,0(s0)
8000de94:	8e020010 	lw	v0,16(s0)
8000de98:	00621023 	subu	v0,v1,v0
8000de9c:	afc20034 	sw	v0,52(s8)
8000dea0:	8e030014 	lw	v1,20(s0)
8000dea4:	00601025 	move	v0,v1
8000dea8:	00021040 	sll	v0,v0,0x1
8000deac:	00431021 	addu	v0,v0,v1
8000deb0:	00021fc2 	srl	v1,v0,0x1f
8000deb4:	00621021 	addu	v0,v1,v0
8000deb8:	00021043 	sra	v0,v0,0x1
8000debc:	afc2002c 	sw	v0,44(s8)
8000dec0:	8fc20034 	lw	v0,52(s8)
8000dec4:	02421021 	addu	v0,s2,v0
8000dec8:	24430001 	addiu	v1,v0,1
8000decc:	8fc2002c 	lw	v0,44(s8)
8000ded0:	0043102b 	sltu	v0,v0,v1
8000ded4:	10400005 	beqz	v0,8000deec <__sfvwrite_r+0x210>
8000ded8:	00000000 	nop
8000dedc:	8fc20034 	lw	v0,52(s8)
8000dee0:	02421021 	addu	v0,s2,v0
8000dee4:	24420001 	addiu	v0,v0,1
8000dee8:	afc2002c 	sw	v0,44(s8)
8000deec:	8602000c 	lh	v0,12(s0)
8000def0:	3042ffff 	andi	v0,v0,0xffff
8000def4:	30420400 	andi	v0,v0,0x400
8000def8:	10400021 	beqz	v0,8000df80 <__sfvwrite_r+0x2a4>
8000defc:	00000000 	nop
8000df00:	8fc2002c 	lw	v0,44(s8)
8000df04:	00402825 	move	a1,v0
8000df08:	8fc40078 	lw	a0,120(s8)
8000df0c:	0c001296 	jal	80004a58 <_malloc_r>
8000df10:	00000000 	nop
8000df14:	afc20028 	sw	v0,40(s8)
8000df18:	8fc20028 	lw	v0,40(s8)
8000df1c:	14400006 	bnez	v0,8000df38 <__sfvwrite_r+0x25c>
8000df20:	00000000 	nop
8000df24:	8fc20078 	lw	v0,120(s8)
8000df28:	2403000c 	li	v1,12
8000df2c:	ac430000 	sw	v1,0(v0)
8000df30:	1000012e 	b	8000e3ec <__sfvwrite_r+0x710>
8000df34:	00000000 	nop
8000df38:	8e020010 	lw	v0,16(s0)
8000df3c:	8fc30034 	lw	v1,52(s8)
8000df40:	00603025 	move	a2,v1
8000df44:	00402825 	move	a1,v0
8000df48:	8fc40028 	lw	a0,40(s8)
8000df4c:	0c0015cf 	jal	8000573c <memcpy>
8000df50:	00000000 	nop
8000df54:	8603000c 	lh	v1,12(s0)
8000df58:	2402fb7f 	li	v0,-1153
8000df5c:	00621024 	and	v0,v1,v0
8000df60:	00021400 	sll	v0,v0,0x10
8000df64:	00021403 	sra	v0,v0,0x10
8000df68:	34420080 	ori	v0,v0,0x80
8000df6c:	00021400 	sll	v0,v0,0x10
8000df70:	00021403 	sra	v0,v0,0x10
8000df74:	a602000c 	sh	v0,12(s0)
8000df78:	1000001c 	b	8000dfec <__sfvwrite_r+0x310>
8000df7c:	00000000 	nop
8000df80:	8e020010 	lw	v0,16(s0)
8000df84:	8fc3002c 	lw	v1,44(s8)
8000df88:	00603025 	move	a2,v1
8000df8c:	00402825 	move	a1,v0
8000df90:	8fc40078 	lw	a0,120(s8)
8000df94:	0c00430b 	jal	80010c2c <_realloc_r>
8000df98:	00000000 	nop
8000df9c:	afc20028 	sw	v0,40(s8)
8000dfa0:	8fc20028 	lw	v0,40(s8)
8000dfa4:	14400011 	bnez	v0,8000dfec <__sfvwrite_r+0x310>
8000dfa8:	00000000 	nop
8000dfac:	8e020010 	lw	v0,16(s0)
8000dfb0:	00402825 	move	a1,v0
8000dfb4:	8fc40078 	lw	a0,120(s8)
8000dfb8:	0c000dcb 	jal	8000372c <_free_r>
8000dfbc:	00000000 	nop
8000dfc0:	8603000c 	lh	v1,12(s0)
8000dfc4:	2402ff7f 	li	v0,-129
8000dfc8:	00621024 	and	v0,v1,v0
8000dfcc:	00021400 	sll	v0,v0,0x10
8000dfd0:	00021403 	sra	v0,v0,0x10
8000dfd4:	a602000c 	sh	v0,12(s0)
8000dfd8:	8fc20078 	lw	v0,120(s8)
8000dfdc:	2403000c 	li	v1,12
8000dfe0:	ac430000 	sw	v1,0(v0)
8000dfe4:	10000101 	b	8000e3ec <__sfvwrite_r+0x710>
8000dfe8:	00000000 	nop
8000dfec:	8fc20028 	lw	v0,40(s8)
8000dff0:	ae020010 	sw	v0,16(s0)
8000dff4:	8fc20034 	lw	v0,52(s8)
8000dff8:	8fc30028 	lw	v1,40(s8)
8000dffc:	00621021 	addu	v0,v1,v0
8000e000:	ae020000 	sw	v0,0(s0)
8000e004:	8fc2002c 	lw	v0,44(s8)
8000e008:	ae020014 	sw	v0,20(s0)
8000e00c:	02408825 	move	s1,s2
8000e010:	8fc3002c 	lw	v1,44(s8)
8000e014:	8fc20034 	lw	v0,52(s8)
8000e018:	00621023 	subu	v0,v1,v0
8000e01c:	ae020008 	sw	v0,8(s0)
8000e020:	02201025 	move	v0,s1
8000e024:	0242102b 	sltu	v0,s2,v0
8000e028:	10400002 	beqz	v0,8000e034 <__sfvwrite_r+0x358>
8000e02c:	00000000 	nop
8000e030:	02408825 	move	s1,s2
8000e034:	8e020000 	lw	v0,0(s0)
8000e038:	02201825 	move	v1,s1
8000e03c:	00603025 	move	a2,v1
8000e040:	02602825 	move	a1,s3
8000e044:	00402025 	move	a0,v0
8000e048:	0c0016ab 	jal	80005aac <memmove>
8000e04c:	00000000 	nop
8000e050:	8e020008 	lw	v0,8(s0)
8000e054:	00511023 	subu	v0,v0,s1
8000e058:	ae020008 	sw	v0,8(s0)
8000e05c:	8e020000 	lw	v0,0(s0)
8000e060:	02201825 	move	v1,s1
8000e064:	00431021 	addu	v0,v0,v1
8000e068:	ae020000 	sw	v0,0(s0)
8000e06c:	02408825 	move	s1,s2
8000e070:	1000004b 	b	8000e1a0 <__sfvwrite_r+0x4c4>
8000e074:	00000000 	nop
8000e078:	8e030000 	lw	v1,0(s0)
8000e07c:	8e020010 	lw	v0,16(s0)
8000e080:	0043102b 	sltu	v0,v0,v1
8000e084:	14400005 	bnez	v0,8000e09c <__sfvwrite_r+0x3c0>
8000e088:	00000000 	nop
8000e08c:	8e020014 	lw	v0,20(s0)
8000e090:	0242102b 	sltu	v0,s2,v0
8000e094:	10400021 	beqz	v0,8000e11c <__sfvwrite_r+0x440>
8000e098:	00000000 	nop
8000e09c:	02202025 	move	a0,s1
8000e0a0:	02401025 	move	v0,s2
8000e0a4:	0082182b 	sltu	v1,a0,v0
8000e0a8:	10600002 	beqz	v1,8000e0b4 <__sfvwrite_r+0x3d8>
8000e0ac:	00000000 	nop
8000e0b0:	00801025 	move	v0,a0
8000e0b4:	00408825 	move	s1,v0
8000e0b8:	8e020000 	lw	v0,0(s0)
8000e0bc:	02201825 	move	v1,s1
8000e0c0:	00603025 	move	a2,v1
8000e0c4:	02602825 	move	a1,s3
8000e0c8:	00402025 	move	a0,v0
8000e0cc:	0c0016ab 	jal	80005aac <memmove>
8000e0d0:	00000000 	nop
8000e0d4:	8e020008 	lw	v0,8(s0)
8000e0d8:	00511023 	subu	v0,v0,s1
8000e0dc:	ae020008 	sw	v0,8(s0)
8000e0e0:	8e020000 	lw	v0,0(s0)
8000e0e4:	02201825 	move	v1,s1
8000e0e8:	00431021 	addu	v0,v0,v1
8000e0ec:	ae020000 	sw	v0,0(s0)
8000e0f0:	8e020008 	lw	v0,8(s0)
8000e0f4:	14400029 	bnez	v0,8000e19c <__sfvwrite_r+0x4c0>
8000e0f8:	00000000 	nop
8000e0fc:	02002825 	move	a1,s0
8000e100:	8fc40078 	lw	a0,120(s8)
8000e104:	0c000b45 	jal	80002d14 <_fflush_r>
8000e108:	00000000 	nop
8000e10c:	10400023 	beqz	v0,8000e19c <__sfvwrite_r+0x4c0>
8000e110:	00000000 	nop
8000e114:	100000b5 	b	8000e3ec <__sfvwrite_r+0x710>
8000e118:	00000000 	nop
8000e11c:	3c027fff 	lui	v0,0x7fff
8000e120:	3442ffff 	ori	v0,v0,0xffff
8000e124:	0242102b 	sltu	v0,s2,v0
8000e128:	10400004 	beqz	v0,8000e13c <__sfvwrite_r+0x460>
8000e12c:	00000000 	nop
8000e130:	02401025 	move	v0,s2
8000e134:	10000003 	b	8000e144 <__sfvwrite_r+0x468>
8000e138:	00000000 	nop
8000e13c:	3c027fff 	lui	v0,0x7fff
8000e140:	3442ffff 	ori	v0,v0,0xffff
8000e144:	8e030014 	lw	v1,20(s0)
8000e148:	0043001a 	div	zero,v0,v1
8000e14c:	006001f4 	teq	v1,zero,0x7
8000e150:	00001010 	mfhi	v0
8000e154:	00001812 	mflo	v1
8000e158:	8e020014 	lw	v0,20(s0)
8000e15c:	00000000 	nop
8000e160:	00620018 	mult	v1,v0
8000e164:	00008812 	mflo	s1
8000e168:	8e020024 	lw	v0,36(s0)
8000e16c:	8e03001c 	lw	v1,28(s0)
8000e170:	02203825 	move	a3,s1
8000e174:	02603025 	move	a2,s3
8000e178:	00602825 	move	a1,v1
8000e17c:	8fc40078 	lw	a0,120(s8)
8000e180:	0040f809 	jalr	v0
8000e184:	00000000 	nop
8000e188:	00408825 	move	s1,v0
8000e18c:	1a200093 	blez	s1,8000e3dc <__sfvwrite_r+0x700>
8000e190:	00000000 	nop
8000e194:	10000002 	b	8000e1a0 <__sfvwrite_r+0x4c4>
8000e198:	00000000 	nop
8000e19c:	00000000 	nop
8000e1a0:	02201025 	move	v0,s1
8000e1a4:	02629821 	addu	s3,s3,v0
8000e1a8:	02201025 	move	v0,s1
8000e1ac:	02429023 	subu	s2,s2,v0
8000e1b0:	8ea20008 	lw	v0,8(s5)
8000e1b4:	02201825 	move	v1,s1
8000e1b8:	00431023 	subu	v0,v0,v1
8000e1bc:	aea20008 	sw	v0,8(s5)
8000e1c0:	8ea20008 	lw	v0,8(s5)
8000e1c4:	1440ff21 	bnez	v0,8000de4c <__sfvwrite_r+0x170>
8000e1c8:	00000000 	nop
8000e1cc:	1000007d 	b	8000e3c4 <__sfvwrite_r+0x6e8>
8000e1d0:	00000000 	nop
8000e1d4:	afc00020 	sw	zero,32(s8)
8000e1d8:	afc00024 	sw	zero,36(s8)
8000e1dc:	10000005 	b	8000e1f4 <__sfvwrite_r+0x518>
8000e1e0:	00000000 	nop
8000e1e4:	afc00020 	sw	zero,32(s8)
8000e1e8:	8e930000 	lw	s3,0(s4)
8000e1ec:	8e920004 	lw	s2,4(s4)
8000e1f0:	26940008 	addiu	s4,s4,8
8000e1f4:	1240fffb 	beqz	s2,8000e1e4 <__sfvwrite_r+0x508>
8000e1f8:	00000000 	nop
8000e1fc:	8fc20020 	lw	v0,32(s8)
8000e200:	14400013 	bnez	v0,8000e250 <__sfvwrite_r+0x574>
8000e204:	00000000 	nop
8000e208:	02403025 	move	a2,s2
8000e20c:	2405000a 	li	a1,10
8000e210:	02602025 	move	a0,s3
8000e214:	0c003adc 	jal	8000eb70 <memchr>
8000e218:	00000000 	nop
8000e21c:	afc20030 	sw	v0,48(s8)
8000e220:	8fc20030 	lw	v0,48(s8)
8000e224:	10400006 	beqz	v0,8000e240 <__sfvwrite_r+0x564>
8000e228:	00000000 	nop
8000e22c:	8fc20030 	lw	v0,48(s8)
8000e230:	24420001 	addiu	v0,v0,1
8000e234:	00531023 	subu	v0,v0,s3
8000e238:	10000002 	b	8000e244 <__sfvwrite_r+0x568>
8000e23c:	00000000 	nop
8000e240:	26420001 	addiu	v0,s2,1
8000e244:	afc20024 	sw	v0,36(s8)
8000e248:	24020001 	li	v0,1
8000e24c:	afc20020 	sw	v0,32(s8)
8000e250:	8fc30024 	lw	v1,36(s8)
8000e254:	02401025 	move	v0,s2
8000e258:	0062202b 	sltu	a0,v1,v0
8000e25c:	10800002 	beqz	a0,8000e268 <__sfvwrite_r+0x58c>
8000e260:	00000000 	nop
8000e264:	00601025 	move	v0,v1
8000e268:	0040f825 	move	ra,v0
8000e26c:	8e030008 	lw	v1,8(s0)
8000e270:	8e020014 	lw	v0,20(s0)
8000e274:	00628821 	addu	s1,v1,v0
8000e278:	8e030000 	lw	v1,0(s0)
8000e27c:	8e020010 	lw	v0,16(s0)
8000e280:	0043102b 	sltu	v0,v0,v1
8000e284:	10400017 	beqz	v0,8000e2e4 <__sfvwrite_r+0x608>
8000e288:	00000000 	nop
8000e28c:	023f102a 	slt	v0,s1,ra
8000e290:	10400014 	beqz	v0,8000e2e4 <__sfvwrite_r+0x608>
8000e294:	00000000 	nop
8000e298:	8e020000 	lw	v0,0(s0)
8000e29c:	02201825 	move	v1,s1
8000e2a0:	00603025 	move	a2,v1
8000e2a4:	02602825 	move	a1,s3
8000e2a8:	00402025 	move	a0,v0
8000e2ac:	0c0016ab 	jal	80005aac <memmove>
8000e2b0:	00000000 	nop
8000e2b4:	8e020000 	lw	v0,0(s0)
8000e2b8:	02201825 	move	v1,s1
8000e2bc:	00431021 	addu	v0,v0,v1
8000e2c0:	ae020000 	sw	v0,0(s0)
8000e2c4:	02002825 	move	a1,s0
8000e2c8:	8fc40078 	lw	a0,120(s8)
8000e2cc:	0c000b45 	jal	80002d14 <_fflush_r>
8000e2d0:	00000000 	nop
8000e2d4:	10400023 	beqz	v0,8000e364 <__sfvwrite_r+0x688>
8000e2d8:	00000000 	nop
8000e2dc:	10000043 	b	8000e3ec <__sfvwrite_r+0x710>
8000e2e0:	00000000 	nop
8000e2e4:	8e110014 	lw	s1,20(s0)
8000e2e8:	03f1102a 	slt	v0,ra,s1
8000e2ec:	1440000e 	bnez	v0,8000e328 <__sfvwrite_r+0x64c>
8000e2f0:	00000000 	nop
8000e2f4:	8e020024 	lw	v0,36(s0)
8000e2f8:	8e03001c 	lw	v1,28(s0)
8000e2fc:	02203825 	move	a3,s1
8000e300:	02603025 	move	a2,s3
8000e304:	00602825 	move	a1,v1
8000e308:	8fc40078 	lw	a0,120(s8)
8000e30c:	0040f809 	jalr	v0
8000e310:	00000000 	nop
8000e314:	00408825 	move	s1,v0
8000e318:	1e200012 	bgtz	s1,8000e364 <__sfvwrite_r+0x688>
8000e31c:	00000000 	nop
8000e320:	10000032 	b	8000e3ec <__sfvwrite_r+0x710>
8000e324:	00000000 	nop
8000e328:	03e08825 	move	s1,ra
8000e32c:	8e020000 	lw	v0,0(s0)
8000e330:	02201825 	move	v1,s1
8000e334:	00603025 	move	a2,v1
8000e338:	02602825 	move	a1,s3
8000e33c:	00402025 	move	a0,v0
8000e340:	0c0016ab 	jal	80005aac <memmove>
8000e344:	00000000 	nop
8000e348:	8e020008 	lw	v0,8(s0)
8000e34c:	00511023 	subu	v0,v0,s1
8000e350:	ae020008 	sw	v0,8(s0)
8000e354:	8e020000 	lw	v0,0(s0)
8000e358:	02201825 	move	v1,s1
8000e35c:	00431021 	addu	v0,v0,v1
8000e360:	ae020000 	sw	v0,0(s0)
8000e364:	8fc20024 	lw	v0,36(s8)
8000e368:	00511023 	subu	v0,v0,s1
8000e36c:	afc20024 	sw	v0,36(s8)
8000e370:	8fc20024 	lw	v0,36(s8)
8000e374:	14400008 	bnez	v0,8000e398 <__sfvwrite_r+0x6bc>
8000e378:	00000000 	nop
8000e37c:	02002825 	move	a1,s0
8000e380:	8fc40078 	lw	a0,120(s8)
8000e384:	0c000b45 	jal	80002d14 <_fflush_r>
8000e388:	00000000 	nop
8000e38c:	14400016 	bnez	v0,8000e3e8 <__sfvwrite_r+0x70c>
8000e390:	00000000 	nop
8000e394:	afc00020 	sw	zero,32(s8)
8000e398:	02201025 	move	v0,s1
8000e39c:	02629821 	addu	s3,s3,v0
8000e3a0:	02201025 	move	v0,s1
8000e3a4:	02429023 	subu	s2,s2,v0
8000e3a8:	8ea20008 	lw	v0,8(s5)
8000e3ac:	02201825 	move	v1,s1
8000e3b0:	00431023 	subu	v0,v0,v1
8000e3b4:	aea20008 	sw	v0,8(s5)
8000e3b8:	8ea20008 	lw	v0,8(s5)
8000e3bc:	1440ff8d 	bnez	v0,8000e1f4 <__sfvwrite_r+0x518>
8000e3c0:	00000000 	nop
8000e3c4:	00001025 	move	v0,zero
8000e3c8:	1000000e 	b	8000e404 <__sfvwrite_r+0x728>
8000e3cc:	00000000 	nop
8000e3d0:	00000000 	nop
8000e3d4:	10000005 	b	8000e3ec <__sfvwrite_r+0x710>
8000e3d8:	00000000 	nop
8000e3dc:	00000000 	nop
8000e3e0:	10000002 	b	8000e3ec <__sfvwrite_r+0x710>
8000e3e4:	00000000 	nop
8000e3e8:	00000000 	nop
8000e3ec:	8602000c 	lh	v0,12(s0)
8000e3f0:	34420040 	ori	v0,v0,0x40
8000e3f4:	00021400 	sll	v0,v0,0x10
8000e3f8:	00021403 	sra	v0,v0,0x10
8000e3fc:	a602000c 	sh	v0,12(s0)
8000e400:	2402ffff 	li	v0,-1
8000e404:	03c0e825 	move	sp,s8
8000e408:	dfbf0070 	ld	ra,112(sp)
8000e40c:	dfbe0068 	ld	s8,104(sp)
8000e410:	dfb50060 	ld	s5,96(sp)
8000e414:	dfb40058 	ld	s4,88(sp)
8000e418:	dfb30050 	ld	s3,80(sp)
8000e41c:	dfb20048 	ld	s2,72(sp)
8000e420:	dfb10040 	ld	s1,64(sp)
8000e424:	dfb00038 	ld	s0,56(sp)
8000e428:	27bd0078 	addiu	sp,sp,120
8000e42c:	03e00008 	jr	ra
8000e430:	00000000 	nop

8000e434 <_findenv_r>:
8000e434:	27bdffb0 	addiu	sp,sp,-80
8000e438:	ffbf0048 	sd	ra,72(sp)
8000e43c:	ffbe0040 	sd	s8,64(sp)
8000e440:	ffb20038 	sd	s2,56(sp)
8000e444:	ffb10030 	sd	s1,48(sp)
8000e448:	ffb00028 	sd	s0,40(sp)
8000e44c:	03a0f025 	move	s8,sp
8000e450:	afc40050 	sw	a0,80(s8)
8000e454:	00a08825 	move	s1,a1
8000e458:	afc60060 	sw	a2,96(s8)
8000e45c:	8fc40050 	lw	a0,80(s8)
8000e460:	0c00578f 	jal	80015e3c <__env_lock>
8000e464:	00000000 	nop
8000e468:	8f828090 	lw	v0,-32624(gp)
8000e46c:	8c420000 	lw	v0,0(v0)
8000e470:	14400007 	bnez	v0,8000e490 <_findenv_r+0x5c>
8000e474:	00000000 	nop
8000e478:	8fc40050 	lw	a0,80(s8)
8000e47c:	0c005799 	jal	80015e64 <__env_unlock>
8000e480:	00000000 	nop
8000e484:	00001025 	move	v0,zero
8000e488:	10000045 	b	8000e5a0 <_findenv_r+0x16c>
8000e48c:	00000000 	nop
8000e490:	afd10020 	sw	s1,32(s8)
8000e494:	10000004 	b	8000e4a8 <_findenv_r+0x74>
8000e498:	00000000 	nop
8000e49c:	8fc20020 	lw	v0,32(s8)
8000e4a0:	24420001 	addiu	v0,v0,1
8000e4a4:	afc20020 	sw	v0,32(s8)
8000e4a8:	8fc20020 	lw	v0,32(s8)
8000e4ac:	80420000 	lb	v0,0(v0)
8000e4b0:	10400006 	beqz	v0,8000e4cc <_findenv_r+0x98>
8000e4b4:	00000000 	nop
8000e4b8:	8fc20020 	lw	v0,32(s8)
8000e4bc:	80430000 	lb	v1,0(v0)
8000e4c0:	2402003d 	li	v0,61
8000e4c4:	1462fff5 	bne	v1,v0,8000e49c <_findenv_r+0x68>
8000e4c8:	00000000 	nop
8000e4cc:	8fc20020 	lw	v0,32(s8)
8000e4d0:	80430000 	lb	v1,0(v0)
8000e4d4:	2402003d 	li	v0,61
8000e4d8:	1062002d 	beq	v1,v0,8000e590 <_findenv_r+0x15c>
8000e4dc:	00000000 	nop
8000e4e0:	8fc20020 	lw	v0,32(s8)
8000e4e4:	00519023 	subu	s2,v0,s1
8000e4e8:	8f828090 	lw	v0,-32624(gp)
8000e4ec:	8c500000 	lw	s0,0(v0)
8000e4f0:	10000024 	b	8000e584 <_findenv_r+0x150>
8000e4f4:	00000000 	nop
8000e4f8:	8e020000 	lw	v0,0(s0)
8000e4fc:	02401825 	move	v1,s2
8000e500:	00603025 	move	a2,v1
8000e504:	02202825 	move	a1,s1
8000e508:	00402025 	move	a0,v0
8000e50c:	0c004780 	jal	80011e00 <strncmp>
8000e510:	00000000 	nop
8000e514:	1440001a 	bnez	v0,8000e580 <_findenv_r+0x14c>
8000e518:	00000000 	nop
8000e51c:	8e020000 	lw	v0,0(s0)
8000e520:	02401825 	move	v1,s2
8000e524:	00431021 	addu	v0,v0,v1
8000e528:	afc20020 	sw	v0,32(s8)
8000e52c:	8fc20020 	lw	v0,32(s8)
8000e530:	80430000 	lb	v1,0(v0)
8000e534:	2402003d 	li	v0,61
8000e538:	14620011 	bne	v1,v0,8000e580 <_findenv_r+0x14c>
8000e53c:	00000000 	nop
8000e540:	8f828090 	lw	v0,-32624(gp)
8000e544:	8c420000 	lw	v0,0(v0)
8000e548:	02021023 	subu	v0,s0,v0
8000e54c:	00021083 	sra	v0,v0,0x2
8000e550:	00401825 	move	v1,v0
8000e554:	8fc20060 	lw	v0,96(s8)
8000e558:	ac430000 	sw	v1,0(v0)
8000e55c:	8fc40050 	lw	a0,80(s8)
8000e560:	0c005799 	jal	80015e64 <__env_unlock>
8000e564:	00000000 	nop
8000e568:	8fc20020 	lw	v0,32(s8)
8000e56c:	24420001 	addiu	v0,v0,1
8000e570:	afc20020 	sw	v0,32(s8)
8000e574:	8fc20020 	lw	v0,32(s8)
8000e578:	10000009 	b	8000e5a0 <_findenv_r+0x16c>
8000e57c:	00000000 	nop
8000e580:	26100004 	addiu	s0,s0,4
8000e584:	8e020000 	lw	v0,0(s0)
8000e588:	1440ffdb 	bnez	v0,8000e4f8 <_findenv_r+0xc4>
8000e58c:	00000000 	nop
8000e590:	8fc40050 	lw	a0,80(s8)
8000e594:	0c005799 	jal	80015e64 <__env_unlock>
8000e598:	00000000 	nop
8000e59c:	00001025 	move	v0,zero
8000e5a0:	03c0e825 	move	sp,s8
8000e5a4:	dfbf0048 	ld	ra,72(sp)
8000e5a8:	dfbe0040 	ld	s8,64(sp)
8000e5ac:	dfb20038 	ld	s2,56(sp)
8000e5b0:	dfb10030 	ld	s1,48(sp)
8000e5b4:	dfb00028 	ld	s0,40(sp)
8000e5b8:	27bd0050 	addiu	sp,sp,80
8000e5bc:	03e00008 	jr	ra
8000e5c0:	00000000 	nop

8000e5c4 <_getenv_r>:
8000e5c4:	27bdffc8 	addiu	sp,sp,-56
8000e5c8:	ffbf0030 	sd	ra,48(sp)
8000e5cc:	ffbe0028 	sd	s8,40(sp)
8000e5d0:	03a0f025 	move	s8,sp
8000e5d4:	afc40038 	sw	a0,56(s8)
8000e5d8:	afc50040 	sw	a1,64(s8)
8000e5dc:	27c20020 	addiu	v0,s8,32
8000e5e0:	00403025 	move	a2,v0
8000e5e4:	8fc50040 	lw	a1,64(s8)
8000e5e8:	8fc40038 	lw	a0,56(s8)
8000e5ec:	0c00390d 	jal	8000e434 <_findenv_r>
8000e5f0:	00000000 	nop
8000e5f4:	03c0e825 	move	sp,s8
8000e5f8:	dfbf0030 	ld	ra,48(sp)
8000e5fc:	dfbe0028 	ld	s8,40(sp)
8000e600:	27bd0038 	addiu	sp,sp,56
8000e604:	03e00008 	jr	ra
8000e608:	00000000 	nop

8000e60c <__gettzinfo>:
8000e60c:	27bdfff8 	addiu	sp,sp,-8
8000e610:	ffbe0000 	sd	s8,0(sp)
8000e614:	03a0f025 	move	s8,sp
8000e618:	3c028002 	lui	v0,0x8002
8000e61c:	2442d2a0 	addiu	v0,v0,-11616
8000e620:	03c0e825 	move	sp,s8
8000e624:	dfbe0000 	ld	s8,0(sp)
8000e628:	27bd0008 	addiu	sp,sp,8
8000e62c:	03e00008 	jr	ra
8000e630:	00000000 	nop

8000e634 <_isatty_r>:
8000e634:	27bdffc8 	addiu	sp,sp,-56
8000e638:	ffbf0030 	sd	ra,48(sp)
8000e63c:	ffbe0028 	sd	s8,40(sp)
8000e640:	03a0f025 	move	s8,sp
8000e644:	afc40038 	sw	a0,56(s8)
8000e648:	afc50040 	sw	a1,64(s8)
8000e64c:	af808334 	sw	zero,-31948(gp)
8000e650:	8fc40040 	lw	a0,64(s8)
8000e654:	0c00679e 	jal	80019e78 <isatty>
8000e658:	00000000 	nop
8000e65c:	afc20020 	sw	v0,32(s8)
8000e660:	8fc30020 	lw	v1,32(s8)
8000e664:	2402ffff 	li	v0,-1
8000e668:	14620007 	bne	v1,v0,8000e688 <_isatty_r+0x54>
8000e66c:	00000000 	nop
8000e670:	8f828334 	lw	v0,-31948(gp)
8000e674:	10400004 	beqz	v0,8000e688 <_isatty_r+0x54>
8000e678:	00000000 	nop
8000e67c:	8f838334 	lw	v1,-31948(gp)
8000e680:	8fc20038 	lw	v0,56(s8)
8000e684:	ac430000 	sw	v1,0(v0)
8000e688:	8fc20020 	lw	v0,32(s8)
8000e68c:	03c0e825 	move	sp,s8
8000e690:	dfbf0030 	ld	ra,48(sp)
8000e694:	dfbe0028 	ld	s8,40(sp)
8000e698:	27bd0038 	addiu	sp,sp,56
8000e69c:	03e00008 	jr	ra
8000e6a0:	00000000 	nop

8000e6a4 <__get_global_locale>:
8000e6a4:	27bdfff8 	addiu	sp,sp,-8
8000e6a8:	ffbe0000 	sd	s8,0(sp)
8000e6ac:	03a0f025 	move	s8,sp
8000e6b0:	3c028002 	lui	v0,0x8002
8000e6b4:	2442d2f8 	addiu	v0,v0,-11528
8000e6b8:	03c0e825 	move	sp,s8
8000e6bc:	dfbe0000 	ld	s8,0(sp)
8000e6c0:	27bd0008 	addiu	sp,sp,8
8000e6c4:	03e00008 	jr	ra
8000e6c8:	00000000 	nop

8000e6cc <__get_current_locale>:
8000e6cc:	27bdffd0 	addiu	sp,sp,-48
8000e6d0:	ffbf0028 	sd	ra,40(sp)
8000e6d4:	ffbe0020 	sd	s8,32(sp)
8000e6d8:	03a0f025 	move	s8,sp
8000e6dc:	0c0039a9 	jal	8000e6a4 <__get_global_locale>
8000e6e0:	00000000 	nop
8000e6e4:	03c0e825 	move	sp,s8
8000e6e8:	dfbf0028 	ld	ra,40(sp)
8000e6ec:	dfbe0020 	ld	s8,32(sp)
8000e6f0:	27bd0030 	addiu	sp,sp,48
8000e6f4:	03e00008 	jr	ra
8000e6f8:	00000000 	nop

8000e6fc <__get_C_locale>:
8000e6fc:	27bdffd0 	addiu	sp,sp,-48
8000e700:	ffbf0028 	sd	ra,40(sp)
8000e704:	ffbe0020 	sd	s8,32(sp)
8000e708:	03a0f025 	move	s8,sp
8000e70c:	0c0039a9 	jal	8000e6a4 <__get_global_locale>
8000e710:	00000000 	nop
8000e714:	03c0e825 	move	sp,s8
8000e718:	dfbf0028 	ld	ra,40(sp)
8000e71c:	dfbe0020 	ld	s8,32(sp)
8000e720:	27bd0030 	addiu	sp,sp,48
8000e724:	03e00008 	jr	ra
8000e728:	00000000 	nop

8000e72c <__localeconv_l>:
8000e72c:	27bdffc8 	addiu	sp,sp,-56
8000e730:	ffbf0030 	sd	ra,48(sp)
8000e734:	ffbe0028 	sd	s8,40(sp)
8000e738:	03a0f025 	move	s8,sp
8000e73c:	afc40038 	sw	a0,56(s8)
8000e740:	8fc20038 	lw	v0,56(s8)
8000e744:	244200f0 	addiu	v0,v0,240
8000e748:	afc20020 	sw	v0,32(s8)
8000e74c:	0c0039bf 	jal	8000e6fc <__get_C_locale>
8000e750:	00000000 	nop
8000e754:	00401825 	move	v1,v0
8000e758:	8fc20038 	lw	v0,56(s8)
8000e75c:	14430004 	bne	v0,v1,8000e770 <__localeconv_l+0x44>
8000e760:	00000000 	nop
8000e764:	8fc20020 	lw	v0,32(s8)
8000e768:	10000002 	b	8000e774 <__localeconv_l+0x48>
8000e76c:	00000000 	nop
8000e770:	8fc20020 	lw	v0,32(s8)
8000e774:	03c0e825 	move	sp,s8
8000e778:	dfbf0030 	ld	ra,48(sp)
8000e77c:	dfbe0028 	ld	s8,40(sp)
8000e780:	27bd0038 	addiu	sp,sp,56
8000e784:	03e00008 	jr	ra
8000e788:	00000000 	nop

8000e78c <_localeconv_r>:
8000e78c:	27bdffd0 	addiu	sp,sp,-48
8000e790:	ffbf0028 	sd	ra,40(sp)
8000e794:	ffbe0020 	sd	s8,32(sp)
8000e798:	03a0f025 	move	s8,sp
8000e79c:	afc40030 	sw	a0,48(s8)
8000e7a0:	0c0039b3 	jal	8000e6cc <__get_current_locale>
8000e7a4:	00000000 	nop
8000e7a8:	00402025 	move	a0,v0
8000e7ac:	0c0039cb 	jal	8000e72c <__localeconv_l>
8000e7b0:	00000000 	nop
8000e7b4:	03c0e825 	move	sp,s8
8000e7b8:	dfbf0028 	ld	ra,40(sp)
8000e7bc:	dfbe0020 	ld	s8,32(sp)
8000e7c0:	27bd0030 	addiu	sp,sp,48
8000e7c4:	03e00008 	jr	ra
8000e7c8:	00000000 	nop

8000e7cc <localeconv>:
8000e7cc:	27bdffd0 	addiu	sp,sp,-48
8000e7d0:	ffbf0028 	sd	ra,40(sp)
8000e7d4:	ffbe0020 	sd	s8,32(sp)
8000e7d8:	03a0f025 	move	s8,sp
8000e7dc:	0c0039b3 	jal	8000e6cc <__get_current_locale>
8000e7e0:	00000000 	nop
8000e7e4:	00402025 	move	a0,v0
8000e7e8:	0c0039cb 	jal	8000e72c <__localeconv_l>
8000e7ec:	00000000 	nop
8000e7f0:	03c0e825 	move	sp,s8
8000e7f4:	dfbf0028 	ld	ra,40(sp)
8000e7f8:	dfbe0020 	ld	s8,32(sp)
8000e7fc:	27bd0030 	addiu	sp,sp,48
8000e800:	03e00008 	jr	ra
8000e804:	00000000 	nop

8000e808 <__get_global_locale>:
8000e808:	27bdfff8 	addiu	sp,sp,-8
8000e80c:	ffbe0000 	sd	s8,0(sp)
8000e810:	03a0f025 	move	s8,sp
8000e814:	3c028002 	lui	v0,0x8002
8000e818:	2442d2f8 	addiu	v0,v0,-11528
8000e81c:	03c0e825 	move	sp,s8
8000e820:	dfbe0000 	ld	s8,0(sp)
8000e824:	27bd0008 	addiu	sp,sp,8
8000e828:	03e00008 	jr	ra
8000e82c:	00000000 	nop

8000e830 <__get_current_locale>:
8000e830:	27bdffd0 	addiu	sp,sp,-48
8000e834:	ffbf0028 	sd	ra,40(sp)
8000e838:	ffbe0020 	sd	s8,32(sp)
8000e83c:	03a0f025 	move	s8,sp
8000e840:	0c003a02 	jal	8000e808 <__get_global_locale>
8000e844:	00000000 	nop
8000e848:	03c0e825 	move	sp,s8
8000e84c:	dfbf0028 	ld	ra,40(sp)
8000e850:	dfbe0020 	ld	s8,32(sp)
8000e854:	27bd0030 	addiu	sp,sp,48
8000e858:	03e00008 	jr	ra
8000e85c:	00000000 	nop

8000e860 <_setlocale_r>:
8000e860:	27bdffd0 	addiu	sp,sp,-48
8000e864:	ffbf0028 	sd	ra,40(sp)
8000e868:	ffbe0020 	sd	s8,32(sp)
8000e86c:	03a0f025 	move	s8,sp
8000e870:	afc40030 	sw	a0,48(s8)
8000e874:	afc50038 	sw	a1,56(s8)
8000e878:	afc60040 	sw	a2,64(s8)
8000e87c:	8fc20040 	lw	v0,64(s8)
8000e880:	10400019 	beqz	v0,8000e8e8 <_setlocale_r+0x88>
8000e884:	00000000 	nop
8000e888:	3c028002 	lui	v0,0x8002
8000e88c:	2445a778 	addiu	a1,v0,-22664
8000e890:	8fc40040 	lw	a0,64(s8)
8000e894:	0c0046ae 	jal	80011ab8 <strcmp>
8000e898:	00000000 	nop
8000e89c:	10400012 	beqz	v0,8000e8e8 <_setlocale_r+0x88>
8000e8a0:	00000000 	nop
8000e8a4:	3c028002 	lui	v0,0x8002
8000e8a8:	2445a780 	addiu	a1,v0,-22656
8000e8ac:	8fc40040 	lw	a0,64(s8)
8000e8b0:	0c0046ae 	jal	80011ab8 <strcmp>
8000e8b4:	00000000 	nop
8000e8b8:	1040000b 	beqz	v0,8000e8e8 <_setlocale_r+0x88>
8000e8bc:	00000000 	nop
8000e8c0:	3c028002 	lui	v0,0x8002
8000e8c4:	2445a770 	addiu	a1,v0,-22672
8000e8c8:	8fc40040 	lw	a0,64(s8)
8000e8cc:	0c0046ae 	jal	80011ab8 <strcmp>
8000e8d0:	00000000 	nop
8000e8d4:	10400004 	beqz	v0,8000e8e8 <_setlocale_r+0x88>
8000e8d8:	00000000 	nop
8000e8dc:	00001025 	move	v0,zero
8000e8e0:	10000003 	b	8000e8f0 <_setlocale_r+0x90>
8000e8e4:	00000000 	nop
8000e8e8:	3c028002 	lui	v0,0x8002
8000e8ec:	2442a780 	addiu	v0,v0,-22656
8000e8f0:	03c0e825 	move	sp,s8
8000e8f4:	dfbf0028 	ld	ra,40(sp)
8000e8f8:	dfbe0020 	ld	s8,32(sp)
8000e8fc:	27bd0030 	addiu	sp,sp,48
8000e900:	03e00008 	jr	ra
8000e904:	00000000 	nop

8000e908 <__locale_mb_cur_max>:
8000e908:	27bdffd0 	addiu	sp,sp,-48
8000e90c:	ffbf0028 	sd	ra,40(sp)
8000e910:	ffbe0020 	sd	s8,32(sp)
8000e914:	03a0f025 	move	s8,sp
8000e918:	0c003a0c 	jal	8000e830 <__get_current_locale>
8000e91c:	00000000 	nop
8000e920:	80420128 	lb	v0,296(v0)
8000e924:	03c0e825 	move	sp,s8
8000e928:	dfbf0028 	ld	ra,40(sp)
8000e92c:	dfbe0020 	ld	s8,32(sp)
8000e930:	27bd0030 	addiu	sp,sp,48
8000e934:	03e00008 	jr	ra
8000e938:	00000000 	nop

8000e93c <setlocale>:
8000e93c:	27bdffd0 	addiu	sp,sp,-48
8000e940:	ffbf0028 	sd	ra,40(sp)
8000e944:	ffbe0020 	sd	s8,32(sp)
8000e948:	03a0f025 	move	s8,sp
8000e94c:	afc40030 	sw	a0,48(s8)
8000e950:	afc50038 	sw	a1,56(s8)
8000e954:	8f828028 	lw	v0,-32728(gp)
8000e958:	8fc60038 	lw	a2,56(s8)
8000e95c:	8fc50030 	lw	a1,48(s8)
8000e960:	00402025 	move	a0,v0
8000e964:	0c003a18 	jal	8000e860 <_setlocale_r>
8000e968:	00000000 	nop
8000e96c:	03c0e825 	move	sp,s8
8000e970:	dfbf0028 	ld	ra,40(sp)
8000e974:	dfbe0020 	ld	s8,32(sp)
8000e978:	27bd0030 	addiu	sp,sp,48
8000e97c:	03e00008 	jr	ra
8000e980:	00000000 	nop

8000e984 <_lseek_r>:
8000e984:	27bdffc8 	addiu	sp,sp,-56
8000e988:	ffbf0030 	sd	ra,48(sp)
8000e98c:	ffbe0028 	sd	s8,40(sp)
8000e990:	03a0f025 	move	s8,sp
8000e994:	afc40038 	sw	a0,56(s8)
8000e998:	afc50040 	sw	a1,64(s8)
8000e99c:	afc60048 	sw	a2,72(s8)
8000e9a0:	afc70050 	sw	a3,80(s8)
8000e9a4:	af808334 	sw	zero,-31948(gp)
8000e9a8:	8fc60050 	lw	a2,80(s8)
8000e9ac:	8fc50048 	lw	a1,72(s8)
8000e9b0:	8fc40040 	lw	a0,64(s8)
8000e9b4:	0c0067ae 	jal	80019eb8 <lseek>
8000e9b8:	00000000 	nop
8000e9bc:	afc20020 	sw	v0,32(s8)
8000e9c0:	8fc30020 	lw	v1,32(s8)
8000e9c4:	2402ffff 	li	v0,-1
8000e9c8:	14620007 	bne	v1,v0,8000e9e8 <_lseek_r+0x64>
8000e9cc:	00000000 	nop
8000e9d0:	8f828334 	lw	v0,-31948(gp)
8000e9d4:	10400004 	beqz	v0,8000e9e8 <_lseek_r+0x64>
8000e9d8:	00000000 	nop
8000e9dc:	8f838334 	lw	v1,-31948(gp)
8000e9e0:	8fc20038 	lw	v0,56(s8)
8000e9e4:	ac430000 	sw	v1,0(v0)
8000e9e8:	8fc20020 	lw	v0,32(s8)
8000e9ec:	03c0e825 	move	sp,s8
8000e9f0:	dfbf0030 	ld	ra,48(sp)
8000e9f4:	dfbe0028 	ld	s8,40(sp)
8000e9f8:	27bd0038 	addiu	sp,sp,56
8000e9fc:	03e00008 	jr	ra
8000ea00:	00000000 	nop

8000ea04 <__get_global_locale>:
8000ea04:	27bdfff8 	addiu	sp,sp,-8
8000ea08:	ffbe0000 	sd	s8,0(sp)
8000ea0c:	03a0f025 	move	s8,sp
8000ea10:	3c028002 	lui	v0,0x8002
8000ea14:	2442d2f8 	addiu	v0,v0,-11528
8000ea18:	03c0e825 	move	sp,s8
8000ea1c:	dfbe0000 	ld	s8,0(sp)
8000ea20:	27bd0008 	addiu	sp,sp,8
8000ea24:	03e00008 	jr	ra
8000ea28:	00000000 	nop

8000ea2c <__get_current_locale>:
8000ea2c:	27bdffd0 	addiu	sp,sp,-48
8000ea30:	ffbf0028 	sd	ra,40(sp)
8000ea34:	ffbe0020 	sd	s8,32(sp)
8000ea38:	03a0f025 	move	s8,sp
8000ea3c:	0c003a81 	jal	8000ea04 <__get_global_locale>
8000ea40:	00000000 	nop
8000ea44:	03c0e825 	move	sp,s8
8000ea48:	dfbf0028 	ld	ra,40(sp)
8000ea4c:	dfbe0020 	ld	s8,32(sp)
8000ea50:	27bd0030 	addiu	sp,sp,48
8000ea54:	03e00008 	jr	ra
8000ea58:	00000000 	nop

8000ea5c <_mbtowc_r>:
8000ea5c:	27bdffc8 	addiu	sp,sp,-56
8000ea60:	ffbf0030 	sd	ra,48(sp)
8000ea64:	ffbe0028 	sd	s8,40(sp)
8000ea68:	03a0f025 	move	s8,sp
8000ea6c:	afc40038 	sw	a0,56(s8)
8000ea70:	afc50040 	sw	a1,64(s8)
8000ea74:	afc60048 	sw	a2,72(s8)
8000ea78:	afc70050 	sw	a3,80(s8)
8000ea7c:	0c003a8b 	jal	8000ea2c <__get_current_locale>
8000ea80:	00000000 	nop
8000ea84:	8c4200e4 	lw	v0,228(v0)
8000ea88:	8fc3005c 	lw	v1,92(s8)
8000ea8c:	afa30024 	sw	v1,36(sp)
8000ea90:	8fc70050 	lw	a3,80(s8)
8000ea94:	8fc60048 	lw	a2,72(s8)
8000ea98:	8fc50040 	lw	a1,64(s8)
8000ea9c:	8fc40038 	lw	a0,56(s8)
8000eaa0:	0040f809 	jalr	v0
8000eaa4:	00000000 	nop
8000eaa8:	03c0e825 	move	sp,s8
8000eaac:	dfbf0030 	ld	ra,48(sp)
8000eab0:	dfbe0028 	ld	s8,40(sp)
8000eab4:	27bd0038 	addiu	sp,sp,56
8000eab8:	03e00008 	jr	ra
8000eabc:	00000000 	nop

8000eac0 <__ascii_mbtowc>:
8000eac0:	27bdfff0 	addiu	sp,sp,-16
8000eac4:	ffbe0008 	sd	s8,8(sp)
8000eac8:	03a0f025 	move	s8,sp
8000eacc:	afc40010 	sw	a0,16(s8)
8000ead0:	afc50018 	sw	a1,24(s8)
8000ead4:	afc60020 	sw	a2,32(s8)
8000ead8:	afc70028 	sw	a3,40(s8)
8000eadc:	8fc20020 	lw	v0,32(s8)
8000eae0:	afc20000 	sw	v0,0(s8)
8000eae4:	8fc20018 	lw	v0,24(s8)
8000eae8:	14400003 	bnez	v0,8000eaf8 <__ascii_mbtowc+0x38>
8000eaec:	00000000 	nop
8000eaf0:	27c20004 	addiu	v0,s8,4
8000eaf4:	afc20018 	sw	v0,24(s8)
8000eaf8:	8fc20020 	lw	v0,32(s8)
8000eafc:	14400004 	bnez	v0,8000eb10 <__ascii_mbtowc+0x50>
8000eb00:	00000000 	nop
8000eb04:	00001025 	move	v0,zero
8000eb08:	10000014 	b	8000eb5c <__ascii_mbtowc+0x9c>
8000eb0c:	00000000 	nop
8000eb10:	8fc20028 	lw	v0,40(s8)
8000eb14:	14400004 	bnez	v0,8000eb28 <__ascii_mbtowc+0x68>
8000eb18:	00000000 	nop
8000eb1c:	2402fffe 	li	v0,-2
8000eb20:	1000000e 	b	8000eb5c <__ascii_mbtowc+0x9c>
8000eb24:	00000000 	nop
8000eb28:	8fc20000 	lw	v0,0(s8)
8000eb2c:	90420000 	lbu	v0,0(v0)
8000eb30:	00401825 	move	v1,v0
8000eb34:	8fc20018 	lw	v0,24(s8)
8000eb38:	ac430000 	sw	v1,0(v0)
8000eb3c:	8fc20000 	lw	v0,0(s8)
8000eb40:	90420000 	lbu	v0,0(v0)
8000eb44:	14400004 	bnez	v0,8000eb58 <__ascii_mbtowc+0x98>
8000eb48:	00000000 	nop
8000eb4c:	00001025 	move	v0,zero
8000eb50:	10000002 	b	8000eb5c <__ascii_mbtowc+0x9c>
8000eb54:	00000000 	nop
8000eb58:	24020001 	li	v0,1
8000eb5c:	03c0e825 	move	sp,s8
8000eb60:	dfbe0008 	ld	s8,8(sp)
8000eb64:	27bd0010 	addiu	sp,sp,16
8000eb68:	03e00008 	jr	ra
8000eb6c:	00000000 	nop

8000eb70 <memchr>:
8000eb70:	27bdffe0 	addiu	sp,sp,-32
8000eb74:	ffbe0018 	sd	s8,24(sp)
8000eb78:	03a0f025 	move	s8,sp
8000eb7c:	afc40020 	sw	a0,32(s8)
8000eb80:	afc50028 	sw	a1,40(s8)
8000eb84:	afc60030 	sw	a2,48(s8)
8000eb88:	8fc20020 	lw	v0,32(s8)
8000eb8c:	afc20000 	sw	v0,0(s8)
8000eb90:	8fc20028 	lw	v0,40(s8)
8000eb94:	a3c20010 	sb	v0,16(s8)
8000eb98:	10000014 	b	8000ebec <memchr+0x7c>
8000eb9c:	00000000 	nop
8000eba0:	8fc20030 	lw	v0,48(s8)
8000eba4:	2443ffff 	addiu	v1,v0,-1
8000eba8:	afc30030 	sw	v1,48(s8)
8000ebac:	14400004 	bnez	v0,8000ebc0 <memchr+0x50>
8000ebb0:	00000000 	nop
8000ebb4:	00001025 	move	v0,zero
8000ebb8:	10000067 	b	8000ed58 <memchr+0x1e8>
8000ebbc:	00000000 	nop
8000ebc0:	8fc20000 	lw	v0,0(s8)
8000ebc4:	90420000 	lbu	v0,0(v0)
8000ebc8:	93c30010 	lbu	v1,16(s8)
8000ebcc:	14620004 	bne	v1,v0,8000ebe0 <memchr+0x70>
8000ebd0:	00000000 	nop
8000ebd4:	8fc20000 	lw	v0,0(s8)
8000ebd8:	1000005f 	b	8000ed58 <memchr+0x1e8>
8000ebdc:	00000000 	nop
8000ebe0:	8fc20000 	lw	v0,0(s8)
8000ebe4:	24420001 	addiu	v0,v0,1
8000ebe8:	afc20000 	sw	v0,0(s8)
8000ebec:	8fc20000 	lw	v0,0(s8)
8000ebf0:	30420003 	andi	v0,v0,0x3
8000ebf4:	1440ffea 	bnez	v0,8000eba0 <memchr+0x30>
8000ebf8:	00000000 	nop
8000ebfc:	8fc20030 	lw	v0,48(s8)
8000ec00:	2c420004 	sltiu	v0,v0,4
8000ec04:	1440004e 	bnez	v0,8000ed40 <memchr+0x1d0>
8000ec08:	00000000 	nop
8000ec0c:	8fc20000 	lw	v0,0(s8)
8000ec10:	afc20004 	sw	v0,4(s8)
8000ec14:	93c20010 	lbu	v0,16(s8)
8000ec18:	00021a00 	sll	v1,v0,0x8
8000ec1c:	93c20010 	lbu	v0,16(s8)
8000ec20:	00621025 	or	v0,v1,v0
8000ec24:	afc20008 	sw	v0,8(s8)
8000ec28:	8fc20008 	lw	v0,8(s8)
8000ec2c:	00021400 	sll	v0,v0,0x10
8000ec30:	8fc30008 	lw	v1,8(s8)
8000ec34:	00621025 	or	v0,v1,v0
8000ec38:	afc20008 	sw	v0,8(s8)
8000ec3c:	24020020 	li	v0,32
8000ec40:	afc2000c 	sw	v0,12(s8)
8000ec44:	1000000a 	b	8000ec70 <memchr+0x100>
8000ec48:	00000000 	nop
8000ec4c:	8fc30008 	lw	v1,8(s8)
8000ec50:	8fc2000c 	lw	v0,12(s8)
8000ec54:	00431004 	sllv	v0,v1,v0
8000ec58:	8fc30008 	lw	v1,8(s8)
8000ec5c:	00621025 	or	v0,v1,v0
8000ec60:	afc20008 	sw	v0,8(s8)
8000ec64:	8fc2000c 	lw	v0,12(s8)
8000ec68:	00021040 	sll	v0,v0,0x1
8000ec6c:	afc2000c 	sw	v0,12(s8)
8000ec70:	8fc2000c 	lw	v0,12(s8)
8000ec74:	2c420020 	sltiu	v0,v0,32
8000ec78:	1440fff4 	bnez	v0,8000ec4c <memchr+0xdc>
8000ec7c:	00000000 	nop
8000ec80:	10000019 	b	8000ece8 <memchr+0x178>
8000ec84:	00000000 	nop
8000ec88:	8fc20004 	lw	v0,4(s8)
8000ec8c:	8c430000 	lw	v1,0(v0)
8000ec90:	8fc20008 	lw	v0,8(s8)
8000ec94:	00621826 	xor	v1,v1,v0
8000ec98:	3c02fefe 	lui	v0,0xfefe
8000ec9c:	3442feff 	ori	v0,v0,0xfeff
8000eca0:	00621821 	addu	v1,v1,v0
8000eca4:	8fc20004 	lw	v0,4(s8)
8000eca8:	8c440000 	lw	a0,0(v0)
8000ecac:	8fc20008 	lw	v0,8(s8)
8000ecb0:	00821026 	xor	v0,a0,v0
8000ecb4:	00021027 	nor	v0,zero,v0
8000ecb8:	00621824 	and	v1,v1,v0
8000ecbc:	3c028080 	lui	v0,0x8080
8000ecc0:	34428080 	ori	v0,v0,0x8080
8000ecc4:	00621024 	and	v0,v1,v0
8000ecc8:	1440000d 	bnez	v0,8000ed00 <memchr+0x190>
8000eccc:	00000000 	nop
8000ecd0:	8fc20030 	lw	v0,48(s8)
8000ecd4:	2442fffc 	addiu	v0,v0,-4
8000ecd8:	afc20030 	sw	v0,48(s8)
8000ecdc:	8fc20004 	lw	v0,4(s8)
8000ece0:	24420004 	addiu	v0,v0,4
8000ece4:	afc20004 	sw	v0,4(s8)
8000ece8:	8fc20030 	lw	v0,48(s8)
8000ecec:	2c420004 	sltiu	v0,v0,4
8000ecf0:	1040ffe5 	beqz	v0,8000ec88 <memchr+0x118>
8000ecf4:	00000000 	nop
8000ecf8:	10000002 	b	8000ed04 <memchr+0x194>
8000ecfc:	00000000 	nop
8000ed00:	00000000 	nop
8000ed04:	8fc20004 	lw	v0,4(s8)
8000ed08:	afc20000 	sw	v0,0(s8)
8000ed0c:	1000000c 	b	8000ed40 <memchr+0x1d0>
8000ed10:	00000000 	nop
8000ed14:	8fc20000 	lw	v0,0(s8)
8000ed18:	90420000 	lbu	v0,0(v0)
8000ed1c:	93c30010 	lbu	v1,16(s8)
8000ed20:	14620004 	bne	v1,v0,8000ed34 <memchr+0x1c4>
8000ed24:	00000000 	nop
8000ed28:	8fc20000 	lw	v0,0(s8)
8000ed2c:	1000000a 	b	8000ed58 <memchr+0x1e8>
8000ed30:	00000000 	nop
8000ed34:	8fc20000 	lw	v0,0(s8)
8000ed38:	24420001 	addiu	v0,v0,1
8000ed3c:	afc20000 	sw	v0,0(s8)
8000ed40:	8fc20030 	lw	v0,48(s8)
8000ed44:	2443ffff 	addiu	v1,v0,-1
8000ed48:	afc30030 	sw	v1,48(s8)
8000ed4c:	1440fff1 	bnez	v0,8000ed14 <memchr+0x1a4>
8000ed50:	00000000 	nop
8000ed54:	00001025 	move	v0,zero
8000ed58:	03c0e825 	move	sp,s8
8000ed5c:	dfbe0018 	ld	s8,24(sp)
8000ed60:	27bd0020 	addiu	sp,sp,32
8000ed64:	03e00008 	jr	ra
8000ed68:	00000000 	nop

8000ed6c <_Balloc>:
8000ed6c:	27bdffc8 	addiu	sp,sp,-56
8000ed70:	ffbf0030 	sd	ra,48(sp)
8000ed74:	ffbe0028 	sd	s8,40(sp)
8000ed78:	03a0f025 	move	s8,sp
8000ed7c:	afc40038 	sw	a0,56(s8)
8000ed80:	afc50040 	sw	a1,64(s8)
8000ed84:	8fc20038 	lw	v0,56(s8)
8000ed88:	8c42004c 	lw	v0,76(v0)
8000ed8c:	14400010 	bnez	v0,8000edd0 <_Balloc+0x64>
8000ed90:	00000000 	nop
8000ed94:	24060021 	li	a2,33
8000ed98:	24050004 	li	a1,4
8000ed9c:	8fc40038 	lw	a0,56(s8)
8000eda0:	0c005715 	jal	80015c54 <_calloc_r>
8000eda4:	00000000 	nop
8000eda8:	00401825 	move	v1,v0
8000edac:	8fc20038 	lw	v0,56(s8)
8000edb0:	ac43004c 	sw	v1,76(v0)
8000edb4:	8fc20038 	lw	v0,56(s8)
8000edb8:	8c42004c 	lw	v0,76(v0)
8000edbc:	14400004 	bnez	v0,8000edd0 <_Balloc+0x64>
8000edc0:	00000000 	nop
8000edc4:	00001025 	move	v0,zero
8000edc8:	10000035 	b	8000eea0 <_Balloc+0x134>
8000edcc:	00000000 	nop
8000edd0:	8fc20038 	lw	v0,56(s8)
8000edd4:	8c43004c 	lw	v1,76(v0)
8000edd8:	8fc20040 	lw	v0,64(s8)
8000eddc:	00021080 	sll	v0,v0,0x2
8000ede0:	00621021 	addu	v0,v1,v0
8000ede4:	8c420000 	lw	v0,0(v0)
8000ede8:	afc20020 	sw	v0,32(s8)
8000edec:	8fc20020 	lw	v0,32(s8)
8000edf0:	1040000b 	beqz	v0,8000ee20 <_Balloc+0xb4>
8000edf4:	00000000 	nop
8000edf8:	8fc20038 	lw	v0,56(s8)
8000edfc:	8c43004c 	lw	v1,76(v0)
8000ee00:	8fc20040 	lw	v0,64(s8)
8000ee04:	00021080 	sll	v0,v0,0x2
8000ee08:	00621021 	addu	v0,v1,v0
8000ee0c:	8fc30020 	lw	v1,32(s8)
8000ee10:	8c630000 	lw	v1,0(v1)
8000ee14:	ac430000 	sw	v1,0(v0)
8000ee18:	1000001a 	b	8000ee84 <_Balloc+0x118>
8000ee1c:	00000000 	nop
8000ee20:	24030001 	li	v1,1
8000ee24:	8fc20040 	lw	v0,64(s8)
8000ee28:	00431004 	sllv	v0,v1,v0
8000ee2c:	afc20024 	sw	v0,36(s8)
8000ee30:	8fc20024 	lw	v0,36(s8)
8000ee34:	24420005 	addiu	v0,v0,5
8000ee38:	00021080 	sll	v0,v0,0x2
8000ee3c:	00403025 	move	a2,v0
8000ee40:	24050001 	li	a1,1
8000ee44:	8fc40038 	lw	a0,56(s8)
8000ee48:	0c005715 	jal	80015c54 <_calloc_r>
8000ee4c:	00000000 	nop
8000ee50:	afc20020 	sw	v0,32(s8)
8000ee54:	8fc20020 	lw	v0,32(s8)
8000ee58:	14400004 	bnez	v0,8000ee6c <_Balloc+0x100>
8000ee5c:	00000000 	nop
8000ee60:	00001025 	move	v0,zero
8000ee64:	1000000e 	b	8000eea0 <_Balloc+0x134>
8000ee68:	00000000 	nop
8000ee6c:	8fc20020 	lw	v0,32(s8)
8000ee70:	8fc30040 	lw	v1,64(s8)
8000ee74:	ac430004 	sw	v1,4(v0)
8000ee78:	8fc20020 	lw	v0,32(s8)
8000ee7c:	8fc30024 	lw	v1,36(s8)
8000ee80:	ac430008 	sw	v1,8(v0)
8000ee84:	8fc20020 	lw	v0,32(s8)
8000ee88:	ac400010 	sw	zero,16(v0)
8000ee8c:	8fc20020 	lw	v0,32(s8)
8000ee90:	8c430010 	lw	v1,16(v0)
8000ee94:	8fc20020 	lw	v0,32(s8)
8000ee98:	ac43000c 	sw	v1,12(v0)
8000ee9c:	8fc20020 	lw	v0,32(s8)
8000eea0:	03c0e825 	move	sp,s8
8000eea4:	dfbf0030 	ld	ra,48(sp)
8000eea8:	dfbe0028 	ld	s8,40(sp)
8000eeac:	27bd0038 	addiu	sp,sp,56
8000eeb0:	03e00008 	jr	ra
8000eeb4:	00000000 	nop

8000eeb8 <_Bfree>:
8000eeb8:	27bdfff8 	addiu	sp,sp,-8
8000eebc:	ffbe0000 	sd	s8,0(sp)
8000eec0:	03a0f025 	move	s8,sp
8000eec4:	afc40008 	sw	a0,8(s8)
8000eec8:	afc50010 	sw	a1,16(s8)
8000eecc:	8fc20010 	lw	v0,16(s8)
8000eed0:	10400012 	beqz	v0,8000ef1c <_Bfree+0x64>
8000eed4:	00000000 	nop
8000eed8:	8fc20008 	lw	v0,8(s8)
8000eedc:	8c43004c 	lw	v1,76(v0)
8000eee0:	8fc20010 	lw	v0,16(s8)
8000eee4:	8c420004 	lw	v0,4(v0)
8000eee8:	00021080 	sll	v0,v0,0x2
8000eeec:	00621021 	addu	v0,v1,v0
8000eef0:	8c430000 	lw	v1,0(v0)
8000eef4:	8fc20010 	lw	v0,16(s8)
8000eef8:	ac430000 	sw	v1,0(v0)
8000eefc:	8fc20008 	lw	v0,8(s8)
8000ef00:	8c43004c 	lw	v1,76(v0)
8000ef04:	8fc20010 	lw	v0,16(s8)
8000ef08:	8c420004 	lw	v0,4(v0)
8000ef0c:	00021080 	sll	v0,v0,0x2
8000ef10:	00621021 	addu	v0,v1,v0
8000ef14:	8fc30010 	lw	v1,16(s8)
8000ef18:	ac430000 	sw	v1,0(v0)
8000ef1c:	00000000 	nop
8000ef20:	03c0e825 	move	sp,s8
8000ef24:	dfbe0000 	ld	s8,0(sp)
8000ef28:	27bd0008 	addiu	sp,sp,8
8000ef2c:	03e00008 	jr	ra
8000ef30:	00000000 	nop

8000ef34 <__multadd>:
8000ef34:	27bdffb0 	addiu	sp,sp,-80
8000ef38:	ffbf0048 	sd	ra,72(sp)
8000ef3c:	ffbe0040 	sd	s8,64(sp)
8000ef40:	03a0f025 	move	s8,sp
8000ef44:	afc40050 	sw	a0,80(s8)
8000ef48:	afc50058 	sw	a1,88(s8)
8000ef4c:	afc60060 	sw	a2,96(s8)
8000ef50:	afc70068 	sw	a3,104(s8)
8000ef54:	8fc20058 	lw	v0,88(s8)
8000ef58:	8c420010 	lw	v0,16(v0)
8000ef5c:	afc20028 	sw	v0,40(s8)
8000ef60:	8fc20058 	lw	v0,88(s8)
8000ef64:	24420014 	addiu	v0,v0,20
8000ef68:	afc20024 	sw	v0,36(s8)
8000ef6c:	afc00020 	sw	zero,32(s8)
8000ef70:	8fc20024 	lw	v0,36(s8)
8000ef74:	8c420000 	lw	v0,0(v0)
8000ef78:	afc2002c 	sw	v0,44(s8)
8000ef7c:	8fc2002c 	lw	v0,44(s8)
8000ef80:	3043ffff 	andi	v1,v0,0xffff
8000ef84:	8fc20060 	lw	v0,96(s8)
8000ef88:	00620018 	mult	v1,v0
8000ef8c:	8fc20068 	lw	v0,104(s8)
8000ef90:	00001812 	mflo	v1
8000ef94:	00621021 	addu	v0,v1,v0
8000ef98:	afc20030 	sw	v0,48(s8)
8000ef9c:	8fc2002c 	lw	v0,44(s8)
8000efa0:	00021c02 	srl	v1,v0,0x10
8000efa4:	8fc20060 	lw	v0,96(s8)
8000efa8:	00620018 	mult	v1,v0
8000efac:	8fc20030 	lw	v0,48(s8)
8000efb0:	00021402 	srl	v0,v0,0x10
8000efb4:	00001812 	mflo	v1
8000efb8:	00621021 	addu	v0,v1,v0
8000efbc:	afc20034 	sw	v0,52(s8)
8000efc0:	8fc20034 	lw	v0,52(s8)
8000efc4:	00021402 	srl	v0,v0,0x10
8000efc8:	afc20068 	sw	v0,104(s8)
8000efcc:	8fc20034 	lw	v0,52(s8)
8000efd0:	00022400 	sll	a0,v0,0x10
8000efd4:	8fc20030 	lw	v0,48(s8)
8000efd8:	3043ffff 	andi	v1,v0,0xffff
8000efdc:	8fc20024 	lw	v0,36(s8)
8000efe0:	24450004 	addiu	a1,v0,4
8000efe4:	afc50024 	sw	a1,36(s8)
8000efe8:	00831821 	addu	v1,a0,v1
8000efec:	ac430000 	sw	v1,0(v0)
8000eff0:	8fc20020 	lw	v0,32(s8)
8000eff4:	24420001 	addiu	v0,v0,1
8000eff8:	afc20020 	sw	v0,32(s8)
8000effc:	8fc30020 	lw	v1,32(s8)
8000f000:	8fc20028 	lw	v0,40(s8)
8000f004:	0062102a 	slt	v0,v1,v0
8000f008:	1440ffd9 	bnez	v0,8000ef70 <__multadd+0x3c>
8000f00c:	00000000 	nop
8000f010:	8fc20068 	lw	v0,104(s8)
8000f014:	1040003b 	beqz	v0,8000f104 <__multadd+0x1d0>
8000f018:	00000000 	nop
8000f01c:	8fc20058 	lw	v0,88(s8)
8000f020:	8c420008 	lw	v0,8(v0)
8000f024:	8fc30028 	lw	v1,40(s8)
8000f028:	0062102a 	slt	v0,v1,v0
8000f02c:	14400029 	bnez	v0,8000f0d4 <__multadd+0x1a0>
8000f030:	00000000 	nop
8000f034:	8fc20058 	lw	v0,88(s8)
8000f038:	8c420004 	lw	v0,4(v0)
8000f03c:	24420001 	addiu	v0,v0,1
8000f040:	00402825 	move	a1,v0
8000f044:	8fc40050 	lw	a0,80(s8)
8000f048:	0c003b5b 	jal	8000ed6c <_Balloc>
8000f04c:	00000000 	nop
8000f050:	afc20038 	sw	v0,56(s8)
8000f054:	8fc20038 	lw	v0,56(s8)
8000f058:	14400009 	bnez	v0,8000f080 <__multadd+0x14c>
8000f05c:	00000000 	nop
8000f060:	3c028002 	lui	v0,0x8002
8000f064:	2447a7e8 	addiu	a3,v0,-22552
8000f068:	00003025 	move	a2,zero
8000f06c:	240500b5 	li	a1,181
8000f070:	3c028002 	lui	v0,0x8002
8000f074:	2444a800 	addiu	a0,v0,-22528
8000f078:	0c0068d2 	jal	8001a348 <__assert_func>
8000f07c:	00000000 	nop
8000f080:	8fc20038 	lw	v0,56(s8)
8000f084:	afc2003c 	sw	v0,60(s8)
8000f088:	8fc2003c 	lw	v0,60(s8)
8000f08c:	2443000c 	addiu	v1,v0,12
8000f090:	8fc20058 	lw	v0,88(s8)
8000f094:	2444000c 	addiu	a0,v0,12
8000f098:	8fc20058 	lw	v0,88(s8)
8000f09c:	8c420010 	lw	v0,16(v0)
8000f0a0:	24420002 	addiu	v0,v0,2
8000f0a4:	00021080 	sll	v0,v0,0x2
8000f0a8:	00403025 	move	a2,v0
8000f0ac:	00802825 	move	a1,a0
8000f0b0:	00602025 	move	a0,v1
8000f0b4:	0c0015cf 	jal	8000573c <memcpy>
8000f0b8:	00000000 	nop
8000f0bc:	8fc50058 	lw	a1,88(s8)
8000f0c0:	8fc40050 	lw	a0,80(s8)
8000f0c4:	0c003bae 	jal	8000eeb8 <_Bfree>
8000f0c8:	00000000 	nop
8000f0cc:	8fc2003c 	lw	v0,60(s8)
8000f0d0:	afc20058 	sw	v0,88(s8)
8000f0d4:	8fc20028 	lw	v0,40(s8)
8000f0d8:	24430001 	addiu	v1,v0,1
8000f0dc:	afc30028 	sw	v1,40(s8)
8000f0e0:	8fc30068 	lw	v1,104(s8)
8000f0e4:	8fc40058 	lw	a0,88(s8)
8000f0e8:	24420004 	addiu	v0,v0,4
8000f0ec:	00021080 	sll	v0,v0,0x2
8000f0f0:	00821021 	addu	v0,a0,v0
8000f0f4:	ac430004 	sw	v1,4(v0)
8000f0f8:	8fc20058 	lw	v0,88(s8)
8000f0fc:	8fc30028 	lw	v1,40(s8)
8000f100:	ac430010 	sw	v1,16(v0)
8000f104:	8fc20058 	lw	v0,88(s8)
8000f108:	03c0e825 	move	sp,s8
8000f10c:	dfbf0048 	ld	ra,72(sp)
8000f110:	dfbe0040 	ld	s8,64(sp)
8000f114:	27bd0050 	addiu	sp,sp,80
8000f118:	03e00008 	jr	ra
8000f11c:	00000000 	nop

8000f120 <__s2b>:
8000f120:	27bdffb8 	addiu	sp,sp,-72
8000f124:	ffbf0040 	sd	ra,64(sp)
8000f128:	ffbe0038 	sd	s8,56(sp)
8000f12c:	03a0f025 	move	s8,sp
8000f130:	afc40048 	sw	a0,72(s8)
8000f134:	afc50050 	sw	a1,80(s8)
8000f138:	afc60058 	sw	a2,88(s8)
8000f13c:	afc70060 	sw	a3,96(s8)
8000f140:	8fc20060 	lw	v0,96(s8)
8000f144:	24450008 	addiu	a1,v0,8
8000f148:	00a02025 	move	a0,a1
8000f14c:	00801025 	move	v0,a0
8000f150:	000210f8 	dsll	v0,v0,0x3
8000f154:	0044102f 	dsubu	v0,v0,a0
8000f158:	000219b8 	dsll	v1,v0,0x6
8000f15c:	0043102d 	daddu	v0,v0,v1
8000f160:	000210f8 	dsll	v0,v0,0x3
8000f164:	0044102d 	daddu	v0,v0,a0
8000f168:	00021bf8 	dsll	v1,v0,0xf
8000f16c:	0062182f 	dsubu	v1,v1,v0
8000f170:	000310f8 	dsll	v0,v1,0x3
8000f174:	00401825 	move	v1,v0
8000f178:	0064182d 	daddu	v1,v1,a0
8000f17c:	0003103e 	dsrl32	v0,v1,0x0
8000f180:	00021000 	sll	v0,v0,0x0
8000f184:	00021843 	sra	v1,v0,0x1
8000f188:	000517c3 	sra	v0,a1,0x1f
8000f18c:	00621023 	subu	v0,v1,v0
8000f190:	afc20030 	sw	v0,48(s8)
8000f194:	afc00028 	sw	zero,40(s8)
8000f198:	24020001 	li	v0,1
8000f19c:	afc2002c 	sw	v0,44(s8)
8000f1a0:	10000007 	b	8000f1c0 <__s2b+0xa0>
8000f1a4:	00000000 	nop
8000f1a8:	8fc2002c 	lw	v0,44(s8)
8000f1ac:	00021040 	sll	v0,v0,0x1
8000f1b0:	afc2002c 	sw	v0,44(s8)
8000f1b4:	8fc20028 	lw	v0,40(s8)
8000f1b8:	24420001 	addiu	v0,v0,1
8000f1bc:	afc20028 	sw	v0,40(s8)
8000f1c0:	8fc30030 	lw	v1,48(s8)
8000f1c4:	8fc2002c 	lw	v0,44(s8)
8000f1c8:	0043102a 	slt	v0,v0,v1
8000f1cc:	1440fff6 	bnez	v0,8000f1a8 <__s2b+0x88>
8000f1d0:	00000000 	nop
8000f1d4:	8fc50028 	lw	a1,40(s8)
8000f1d8:	8fc40048 	lw	a0,72(s8)
8000f1dc:	0c003b5b 	jal	8000ed6c <_Balloc>
8000f1e0:	00000000 	nop
8000f1e4:	afc20034 	sw	v0,52(s8)
8000f1e8:	8fc20034 	lw	v0,52(s8)
8000f1ec:	14400009 	bnez	v0,8000f214 <__s2b+0xf4>
8000f1f0:	00000000 	nop
8000f1f4:	3c028002 	lui	v0,0x8002
8000f1f8:	2447a7e8 	addiu	a3,v0,-22552
8000f1fc:	00003025 	move	a2,zero
8000f200:	240500ce 	li	a1,206
8000f204:	3c028002 	lui	v0,0x8002
8000f208:	2444a800 	addiu	a0,v0,-22528
8000f20c:	0c0068d2 	jal	8001a348 <__assert_func>
8000f210:	00000000 	nop
8000f214:	8fc20034 	lw	v0,52(s8)
8000f218:	afc20020 	sw	v0,32(s8)
8000f21c:	8fc20020 	lw	v0,32(s8)
8000f220:	8fc3006c 	lw	v1,108(s8)
8000f224:	ac430014 	sw	v1,20(v0)
8000f228:	8fc20020 	lw	v0,32(s8)
8000f22c:	24030001 	li	v1,1
8000f230:	ac430010 	sw	v1,16(v0)
8000f234:	24020009 	li	v0,9
8000f238:	afc20024 	sw	v0,36(s8)
8000f23c:	8fc20058 	lw	v0,88(s8)
8000f240:	2842000a 	slti	v0,v0,10
8000f244:	1440001d 	bnez	v0,8000f2bc <__s2b+0x19c>
8000f248:	00000000 	nop
8000f24c:	8fc20050 	lw	v0,80(s8)
8000f250:	24420009 	addiu	v0,v0,9
8000f254:	afc20050 	sw	v0,80(s8)
8000f258:	8fc20050 	lw	v0,80(s8)
8000f25c:	24430001 	addiu	v1,v0,1
8000f260:	afc30050 	sw	v1,80(s8)
8000f264:	80420000 	lb	v0,0(v0)
8000f268:	2442ffd0 	addiu	v0,v0,-48
8000f26c:	00403825 	move	a3,v0
8000f270:	2406000a 	li	a2,10
8000f274:	8fc50020 	lw	a1,32(s8)
8000f278:	8fc40048 	lw	a0,72(s8)
8000f27c:	0c003bcd 	jal	8000ef34 <__multadd>
8000f280:	00000000 	nop
8000f284:	afc20020 	sw	v0,32(s8)
8000f288:	8fc20024 	lw	v0,36(s8)
8000f28c:	24420001 	addiu	v0,v0,1
8000f290:	afc20024 	sw	v0,36(s8)
8000f294:	8fc30024 	lw	v1,36(s8)
8000f298:	8fc20058 	lw	v0,88(s8)
8000f29c:	0062102a 	slt	v0,v1,v0
8000f2a0:	1440ffed 	bnez	v0,8000f258 <__s2b+0x138>
8000f2a4:	00000000 	nop
8000f2a8:	8fc20050 	lw	v0,80(s8)
8000f2ac:	24420001 	addiu	v0,v0,1
8000f2b0:	afc20050 	sw	v0,80(s8)
8000f2b4:	10000015 	b	8000f30c <__s2b+0x1ec>
8000f2b8:	00000000 	nop
8000f2bc:	8fc20050 	lw	v0,80(s8)
8000f2c0:	2442000a 	addiu	v0,v0,10
8000f2c4:	afc20050 	sw	v0,80(s8)
8000f2c8:	10000010 	b	8000f30c <__s2b+0x1ec>
8000f2cc:	00000000 	nop
8000f2d0:	8fc20050 	lw	v0,80(s8)
8000f2d4:	24430001 	addiu	v1,v0,1
8000f2d8:	afc30050 	sw	v1,80(s8)
8000f2dc:	80420000 	lb	v0,0(v0)
8000f2e0:	2442ffd0 	addiu	v0,v0,-48
8000f2e4:	00403825 	move	a3,v0
8000f2e8:	2406000a 	li	a2,10
8000f2ec:	8fc50020 	lw	a1,32(s8)
8000f2f0:	8fc40048 	lw	a0,72(s8)
8000f2f4:	0c003bcd 	jal	8000ef34 <__multadd>
8000f2f8:	00000000 	nop
8000f2fc:	afc20020 	sw	v0,32(s8)
8000f300:	8fc20024 	lw	v0,36(s8)
8000f304:	24420001 	addiu	v0,v0,1
8000f308:	afc20024 	sw	v0,36(s8)
8000f30c:	8fc30024 	lw	v1,36(s8)
8000f310:	8fc20060 	lw	v0,96(s8)
8000f314:	0062102a 	slt	v0,v1,v0
8000f318:	1440ffed 	bnez	v0,8000f2d0 <__s2b+0x1b0>
8000f31c:	00000000 	nop
8000f320:	8fc20020 	lw	v0,32(s8)
8000f324:	03c0e825 	move	sp,s8
8000f328:	dfbf0040 	ld	ra,64(sp)
8000f32c:	dfbe0038 	ld	s8,56(sp)
8000f330:	27bd0048 	addiu	sp,sp,72
8000f334:	03e00008 	jr	ra
8000f338:	00000000 	nop

8000f33c <__hi0bits>:
8000f33c:	27bdfff0 	addiu	sp,sp,-16
8000f340:	ffbe0008 	sd	s8,8(sp)
8000f344:	ffb00000 	sd	s0,0(sp)
8000f348:	03a0f025 	move	s8,sp
8000f34c:	00801025 	move	v0,a0
8000f350:	00008025 	move	s0,zero
8000f354:	3c03ffff 	lui	v1,0xffff
8000f358:	00431824 	and	v1,v0,v1
8000f35c:	14600003 	bnez	v1,8000f36c <__hi0bits+0x30>
8000f360:	00000000 	nop
8000f364:	24100010 	li	s0,16
8000f368:	00021400 	sll	v0,v0,0x10
8000f36c:	3c03ff00 	lui	v1,0xff00
8000f370:	00431824 	and	v1,v0,v1
8000f374:	14600003 	bnez	v1,8000f384 <__hi0bits+0x48>
8000f378:	00000000 	nop
8000f37c:	26100008 	addiu	s0,s0,8
8000f380:	00021200 	sll	v0,v0,0x8
8000f384:	3c03f000 	lui	v1,0xf000
8000f388:	00431824 	and	v1,v0,v1
8000f38c:	14600003 	bnez	v1,8000f39c <__hi0bits+0x60>
8000f390:	00000000 	nop
8000f394:	26100004 	addiu	s0,s0,4
8000f398:	00021100 	sll	v0,v0,0x4
8000f39c:	3c03c000 	lui	v1,0xc000
8000f3a0:	00431824 	and	v1,v0,v1
8000f3a4:	14600003 	bnez	v1,8000f3b4 <__hi0bits+0x78>
8000f3a8:	00000000 	nop
8000f3ac:	26100002 	addiu	s0,s0,2
8000f3b0:	00021080 	sll	v0,v0,0x2
8000f3b4:	00401825 	move	v1,v0
8000f3b8:	04600009 	bltz	v1,8000f3e0 <__hi0bits+0xa4>
8000f3bc:	00000000 	nop
8000f3c0:	26100001 	addiu	s0,s0,1
8000f3c4:	3c034000 	lui	v1,0x4000
8000f3c8:	00431024 	and	v0,v0,v1
8000f3cc:	14400004 	bnez	v0,8000f3e0 <__hi0bits+0xa4>
8000f3d0:	00000000 	nop
8000f3d4:	24020020 	li	v0,32
8000f3d8:	10000002 	b	8000f3e4 <__hi0bits+0xa8>
8000f3dc:	00000000 	nop
8000f3e0:	02001025 	move	v0,s0
8000f3e4:	03c0e825 	move	sp,s8
8000f3e8:	dfbe0008 	ld	s8,8(sp)
8000f3ec:	dfb00000 	ld	s0,0(sp)
8000f3f0:	27bd0010 	addiu	sp,sp,16
8000f3f4:	03e00008 	jr	ra
8000f3f8:	00000000 	nop

8000f3fc <__lo0bits>:
8000f3fc:	27bdffe8 	addiu	sp,sp,-24
8000f400:	ffbe0010 	sd	s8,16(sp)
8000f404:	ffb10008 	sd	s1,8(sp)
8000f408:	ffb00000 	sd	s0,0(sp)
8000f40c:	03a0f025 	move	s8,sp
8000f410:	afc40018 	sw	a0,24(s8)
8000f414:	8fc20018 	lw	v0,24(s8)
8000f418:	8c500000 	lw	s0,0(v0)
8000f41c:	32020007 	andi	v0,s0,0x7
8000f420:	10400016 	beqz	v0,8000f47c <__lo0bits+0x80>
8000f424:	00000000 	nop
8000f428:	32020001 	andi	v0,s0,0x1
8000f42c:	10400004 	beqz	v0,8000f440 <__lo0bits+0x44>
8000f430:	00000000 	nop
8000f434:	00001025 	move	v0,zero
8000f438:	10000032 	b	8000f504 <__lo0bits+0x108>
8000f43c:	00000000 	nop
8000f440:	32020002 	andi	v0,s0,0x2
8000f444:	10400007 	beqz	v0,8000f464 <__lo0bits+0x68>
8000f448:	00000000 	nop
8000f44c:	00101842 	srl	v1,s0,0x1
8000f450:	8fc20018 	lw	v0,24(s8)
8000f454:	ac430000 	sw	v1,0(v0)
8000f458:	24020001 	li	v0,1
8000f45c:	10000029 	b	8000f504 <__lo0bits+0x108>
8000f460:	00000000 	nop
8000f464:	00101882 	srl	v1,s0,0x2
8000f468:	8fc20018 	lw	v0,24(s8)
8000f46c:	ac430000 	sw	v1,0(v0)
8000f470:	24020002 	li	v0,2
8000f474:	10000023 	b	8000f504 <__lo0bits+0x108>
8000f478:	00000000 	nop
8000f47c:	00008825 	move	s1,zero
8000f480:	3202ffff 	andi	v0,s0,0xffff
8000f484:	14400003 	bnez	v0,8000f494 <__lo0bits+0x98>
8000f488:	00000000 	nop
8000f48c:	24110010 	li	s1,16
8000f490:	00108402 	srl	s0,s0,0x10
8000f494:	320200ff 	andi	v0,s0,0xff
8000f498:	14400003 	bnez	v0,8000f4a8 <__lo0bits+0xac>
8000f49c:	00000000 	nop
8000f4a0:	26310008 	addiu	s1,s1,8
8000f4a4:	00108202 	srl	s0,s0,0x8
8000f4a8:	3202000f 	andi	v0,s0,0xf
8000f4ac:	14400003 	bnez	v0,8000f4bc <__lo0bits+0xc0>
8000f4b0:	00000000 	nop
8000f4b4:	26310004 	addiu	s1,s1,4
8000f4b8:	00108102 	srl	s0,s0,0x4
8000f4bc:	32020003 	andi	v0,s0,0x3
8000f4c0:	14400003 	bnez	v0,8000f4d0 <__lo0bits+0xd4>
8000f4c4:	00000000 	nop
8000f4c8:	26310002 	addiu	s1,s1,2
8000f4cc:	00108082 	srl	s0,s0,0x2
8000f4d0:	32020001 	andi	v0,s0,0x1
8000f4d4:	14400008 	bnez	v0,8000f4f8 <__lo0bits+0xfc>
8000f4d8:	00000000 	nop
8000f4dc:	26310001 	addiu	s1,s1,1
8000f4e0:	00108042 	srl	s0,s0,0x1
8000f4e4:	16000004 	bnez	s0,8000f4f8 <__lo0bits+0xfc>
8000f4e8:	00000000 	nop
8000f4ec:	24020020 	li	v0,32
8000f4f0:	10000004 	b	8000f504 <__lo0bits+0x108>
8000f4f4:	00000000 	nop
8000f4f8:	8fc20018 	lw	v0,24(s8)
8000f4fc:	ac500000 	sw	s0,0(v0)
8000f500:	02201025 	move	v0,s1
8000f504:	03c0e825 	move	sp,s8
8000f508:	dfbe0010 	ld	s8,16(sp)
8000f50c:	dfb10008 	ld	s1,8(sp)
8000f510:	dfb00000 	ld	s0,0(sp)
8000f514:	27bd0018 	addiu	sp,sp,24
8000f518:	03e00008 	jr	ra
8000f51c:	00000000 	nop

8000f520 <__i2b>:
8000f520:	27bdffc8 	addiu	sp,sp,-56
8000f524:	ffbf0030 	sd	ra,48(sp)
8000f528:	ffbe0028 	sd	s8,40(sp)
8000f52c:	03a0f025 	move	s8,sp
8000f530:	afc40038 	sw	a0,56(s8)
8000f534:	afc50040 	sw	a1,64(s8)
8000f538:	24050001 	li	a1,1
8000f53c:	8fc40038 	lw	a0,56(s8)
8000f540:	0c003b5b 	jal	8000ed6c <_Balloc>
8000f544:	00000000 	nop
8000f548:	afc20020 	sw	v0,32(s8)
8000f54c:	8fc20020 	lw	v0,32(s8)
8000f550:	14400009 	bnez	v0,8000f578 <__i2b+0x58>
8000f554:	00000000 	nop
8000f558:	3c028002 	lui	v0,0x8002
8000f55c:	2447a7e8 	addiu	a3,v0,-22552
8000f560:	00003025 	move	a2,zero
8000f564:	24050140 	li	a1,320
8000f568:	3c028002 	lui	v0,0x8002
8000f56c:	2444a800 	addiu	a0,v0,-22528
8000f570:	0c0068d2 	jal	8001a348 <__assert_func>
8000f574:	00000000 	nop
8000f578:	8fc20020 	lw	v0,32(s8)
8000f57c:	afc20024 	sw	v0,36(s8)
8000f580:	8fc30040 	lw	v1,64(s8)
8000f584:	8fc20024 	lw	v0,36(s8)
8000f588:	ac430014 	sw	v1,20(v0)
8000f58c:	8fc20024 	lw	v0,36(s8)
8000f590:	24030001 	li	v1,1
8000f594:	ac430010 	sw	v1,16(v0)
8000f598:	8fc20024 	lw	v0,36(s8)
8000f59c:	03c0e825 	move	sp,s8
8000f5a0:	dfbf0030 	ld	ra,48(sp)
8000f5a4:	dfbe0028 	ld	s8,40(sp)
8000f5a8:	27bd0038 	addiu	sp,sp,56
8000f5ac:	03e00008 	jr	ra
8000f5b0:	00000000 	nop

8000f5b4 <__multiply>:
8000f5b4:	27bdff88 	addiu	sp,sp,-120
8000f5b8:	ffbf0070 	sd	ra,112(sp)
8000f5bc:	ffbe0068 	sd	s8,104(sp)
8000f5c0:	03a0f025 	move	s8,sp
8000f5c4:	afc40078 	sw	a0,120(s8)
8000f5c8:	afc50080 	sw	a1,128(s8)
8000f5cc:	afc60088 	sw	a2,136(s8)
8000f5d0:	8fc20080 	lw	v0,128(s8)
8000f5d4:	8c430010 	lw	v1,16(v0)
8000f5d8:	8fc20088 	lw	v0,136(s8)
8000f5dc:	8c420010 	lw	v0,16(v0)
8000f5e0:	0062102a 	slt	v0,v1,v0
8000f5e4:	10400007 	beqz	v0,8000f604 <__multiply+0x50>
8000f5e8:	00000000 	nop
8000f5ec:	8fc20080 	lw	v0,128(s8)
8000f5f0:	afc20040 	sw	v0,64(s8)
8000f5f4:	8fc20088 	lw	v0,136(s8)
8000f5f8:	afc20080 	sw	v0,128(s8)
8000f5fc:	8fc20040 	lw	v0,64(s8)
8000f600:	afc20088 	sw	v0,136(s8)
8000f604:	8fc20080 	lw	v0,128(s8)
8000f608:	8c420004 	lw	v0,4(v0)
8000f60c:	afc20020 	sw	v0,32(s8)
8000f610:	8fc20080 	lw	v0,128(s8)
8000f614:	8c420010 	lw	v0,16(v0)
8000f618:	afc20044 	sw	v0,68(s8)
8000f61c:	8fc20088 	lw	v0,136(s8)
8000f620:	8c420010 	lw	v0,16(v0)
8000f624:	afc20048 	sw	v0,72(s8)
8000f628:	8fc30044 	lw	v1,68(s8)
8000f62c:	8fc20048 	lw	v0,72(s8)
8000f630:	00621021 	addu	v0,v1,v0
8000f634:	afc20024 	sw	v0,36(s8)
8000f638:	8fc20080 	lw	v0,128(s8)
8000f63c:	8c420008 	lw	v0,8(v0)
8000f640:	8fc30024 	lw	v1,36(s8)
8000f644:	0043102a 	slt	v0,v0,v1
8000f648:	10400004 	beqz	v0,8000f65c <__multiply+0xa8>
8000f64c:	00000000 	nop
8000f650:	8fc20020 	lw	v0,32(s8)
8000f654:	24420001 	addiu	v0,v0,1
8000f658:	afc20020 	sw	v0,32(s8)
8000f65c:	8fc50020 	lw	a1,32(s8)
8000f660:	8fc40078 	lw	a0,120(s8)
8000f664:	0c003b5b 	jal	8000ed6c <_Balloc>
8000f668:	00000000 	nop
8000f66c:	afc2004c 	sw	v0,76(s8)
8000f670:	8fc2004c 	lw	v0,76(s8)
8000f674:	14400009 	bnez	v0,8000f69c <__multiply+0xe8>
8000f678:	00000000 	nop
8000f67c:	3c028002 	lui	v0,0x8002
8000f680:	2447a7e8 	addiu	a3,v0,-22552
8000f684:	00003025 	move	a2,zero
8000f688:	2405015d 	li	a1,349
8000f68c:	3c028002 	lui	v0,0x8002
8000f690:	2444a800 	addiu	a0,v0,-22528
8000f694:	0c0068d2 	jal	8001a348 <__assert_func>
8000f698:	00000000 	nop
8000f69c:	8fc2004c 	lw	v0,76(s8)
8000f6a0:	afc20040 	sw	v0,64(s8)
8000f6a4:	8fc20040 	lw	v0,64(s8)
8000f6a8:	24420014 	addiu	v0,v0,20
8000f6ac:	afc2002c 	sw	v0,44(s8)
8000f6b0:	8fc20024 	lw	v0,36(s8)
8000f6b4:	00021080 	sll	v0,v0,0x2
8000f6b8:	8fc3002c 	lw	v1,44(s8)
8000f6bc:	00621021 	addu	v0,v1,v0
8000f6c0:	afc20050 	sw	v0,80(s8)
8000f6c4:	10000006 	b	8000f6e0 <__multiply+0x12c>
8000f6c8:	00000000 	nop
8000f6cc:	8fc2002c 	lw	v0,44(s8)
8000f6d0:	ac400000 	sw	zero,0(v0)
8000f6d4:	8fc2002c 	lw	v0,44(s8)
8000f6d8:	24420004 	addiu	v0,v0,4
8000f6dc:	afc2002c 	sw	v0,44(s8)
8000f6e0:	8fc3002c 	lw	v1,44(s8)
8000f6e4:	8fc20050 	lw	v0,80(s8)
8000f6e8:	0062102b 	sltu	v0,v1,v0
8000f6ec:	1440fff7 	bnez	v0,8000f6cc <__multiply+0x118>
8000f6f0:	00000000 	nop
8000f6f4:	8fc20080 	lw	v0,128(s8)
8000f6f8:	24420014 	addiu	v0,v0,20
8000f6fc:	afc20050 	sw	v0,80(s8)
8000f700:	8fc20044 	lw	v0,68(s8)
8000f704:	00021080 	sll	v0,v0,0x2
8000f708:	8fc30050 	lw	v1,80(s8)
8000f70c:	00621021 	addu	v0,v1,v0
8000f710:	afc20054 	sw	v0,84(s8)
8000f714:	8fc20088 	lw	v0,136(s8)
8000f718:	24420014 	addiu	v0,v0,20
8000f71c:	afc20030 	sw	v0,48(s8)
8000f720:	8fc20048 	lw	v0,72(s8)
8000f724:	00021080 	sll	v0,v0,0x2
8000f728:	8fc30030 	lw	v1,48(s8)
8000f72c:	00621021 	addu	v0,v1,v0
8000f730:	afc20058 	sw	v0,88(s8)
8000f734:	8fc20040 	lw	v0,64(s8)
8000f738:	24420014 	addiu	v0,v0,20
8000f73c:	afc20038 	sw	v0,56(s8)
8000f740:	10000088 	b	8000f964 <__multiply+0x3b0>
8000f744:	00000000 	nop
8000f748:	8fc20030 	lw	v0,48(s8)
8000f74c:	8c420000 	lw	v0,0(v0)
8000f750:	3042ffff 	andi	v0,v0,0xffff
8000f754:	afc2005c 	sw	v0,92(s8)
8000f758:	8fc2005c 	lw	v0,92(s8)
8000f75c:	10400039 	beqz	v0,8000f844 <__multiply+0x290>
8000f760:	00000000 	nop
8000f764:	8fc20050 	lw	v0,80(s8)
8000f768:	afc2002c 	sw	v0,44(s8)
8000f76c:	8fc20038 	lw	v0,56(s8)
8000f770:	afc20034 	sw	v0,52(s8)
8000f774:	afc00028 	sw	zero,40(s8)
8000f778:	8fc2002c 	lw	v0,44(s8)
8000f77c:	8c420000 	lw	v0,0(v0)
8000f780:	3043ffff 	andi	v1,v0,0xffff
8000f784:	8fc2005c 	lw	v0,92(s8)
8000f788:	00620018 	mult	v1,v0
8000f78c:	8fc20034 	lw	v0,52(s8)
8000f790:	8c420000 	lw	v0,0(v0)
8000f794:	3042ffff 	andi	v0,v0,0xffff
8000f798:	00001812 	mflo	v1
8000f79c:	00621021 	addu	v0,v1,v0
8000f7a0:	8fc30028 	lw	v1,40(s8)
8000f7a4:	00621021 	addu	v0,v1,v0
8000f7a8:	afc20060 	sw	v0,96(s8)
8000f7ac:	8fc20060 	lw	v0,96(s8)
8000f7b0:	00021402 	srl	v0,v0,0x10
8000f7b4:	afc20028 	sw	v0,40(s8)
8000f7b8:	8fc2002c 	lw	v0,44(s8)
8000f7bc:	24430004 	addiu	v1,v0,4
8000f7c0:	afc3002c 	sw	v1,44(s8)
8000f7c4:	8c420000 	lw	v0,0(v0)
8000f7c8:	00021c02 	srl	v1,v0,0x10
8000f7cc:	8fc2005c 	lw	v0,92(s8)
8000f7d0:	00620018 	mult	v1,v0
8000f7d4:	8fc20034 	lw	v0,52(s8)
8000f7d8:	8c420000 	lw	v0,0(v0)
8000f7dc:	00021402 	srl	v0,v0,0x10
8000f7e0:	00001812 	mflo	v1
8000f7e4:	00621021 	addu	v0,v1,v0
8000f7e8:	8fc30028 	lw	v1,40(s8)
8000f7ec:	00621021 	addu	v0,v1,v0
8000f7f0:	afc2003c 	sw	v0,60(s8)
8000f7f4:	8fc2003c 	lw	v0,60(s8)
8000f7f8:	00021402 	srl	v0,v0,0x10
8000f7fc:	afc20028 	sw	v0,40(s8)
8000f800:	8fc2003c 	lw	v0,60(s8)
8000f804:	00022400 	sll	a0,v0,0x10
8000f808:	8fc20060 	lw	v0,96(s8)
8000f80c:	3043ffff 	andi	v1,v0,0xffff
8000f810:	8fc20034 	lw	v0,52(s8)
8000f814:	24450004 	addiu	a1,v0,4
8000f818:	afc50034 	sw	a1,52(s8)
8000f81c:	00831825 	or	v1,a0,v1
8000f820:	ac430000 	sw	v1,0(v0)
8000f824:	8fc3002c 	lw	v1,44(s8)
8000f828:	8fc20054 	lw	v0,84(s8)
8000f82c:	0062102b 	sltu	v0,v1,v0
8000f830:	1440ffd1 	bnez	v0,8000f778 <__multiply+0x1c4>
8000f834:	00000000 	nop
8000f838:	8fc20034 	lw	v0,52(s8)
8000f83c:	8fc30028 	lw	v1,40(s8)
8000f840:	ac430000 	sw	v1,0(v0)
8000f844:	8fc20030 	lw	v0,48(s8)
8000f848:	8c420000 	lw	v0,0(v0)
8000f84c:	00021402 	srl	v0,v0,0x10
8000f850:	afc2005c 	sw	v0,92(s8)
8000f854:	8fc2005c 	lw	v0,92(s8)
8000f858:	1040003c 	beqz	v0,8000f94c <__multiply+0x398>
8000f85c:	00000000 	nop
8000f860:	8fc20050 	lw	v0,80(s8)
8000f864:	afc2002c 	sw	v0,44(s8)
8000f868:	8fc20038 	lw	v0,56(s8)
8000f86c:	afc20034 	sw	v0,52(s8)
8000f870:	afc00028 	sw	zero,40(s8)
8000f874:	8fc20034 	lw	v0,52(s8)
8000f878:	8c420000 	lw	v0,0(v0)
8000f87c:	afc2003c 	sw	v0,60(s8)
8000f880:	8fc2002c 	lw	v0,44(s8)
8000f884:	8c420000 	lw	v0,0(v0)
8000f888:	3043ffff 	andi	v1,v0,0xffff
8000f88c:	8fc2005c 	lw	v0,92(s8)
8000f890:	00620018 	mult	v1,v0
8000f894:	8fc20034 	lw	v0,52(s8)
8000f898:	8c420000 	lw	v0,0(v0)
8000f89c:	00021402 	srl	v0,v0,0x10
8000f8a0:	00001812 	mflo	v1
8000f8a4:	00621021 	addu	v0,v1,v0
8000f8a8:	8fc30028 	lw	v1,40(s8)
8000f8ac:	00621021 	addu	v0,v1,v0
8000f8b0:	afc20060 	sw	v0,96(s8)
8000f8b4:	8fc20060 	lw	v0,96(s8)
8000f8b8:	00021402 	srl	v0,v0,0x10
8000f8bc:	afc20028 	sw	v0,40(s8)
8000f8c0:	8fc20060 	lw	v0,96(s8)
8000f8c4:	00022400 	sll	a0,v0,0x10
8000f8c8:	8fc2003c 	lw	v0,60(s8)
8000f8cc:	3043ffff 	andi	v1,v0,0xffff
8000f8d0:	8fc20034 	lw	v0,52(s8)
8000f8d4:	24450004 	addiu	a1,v0,4
8000f8d8:	afc50034 	sw	a1,52(s8)
8000f8dc:	00831825 	or	v1,a0,v1
8000f8e0:	ac430000 	sw	v1,0(v0)
8000f8e4:	8fc2002c 	lw	v0,44(s8)
8000f8e8:	24430004 	addiu	v1,v0,4
8000f8ec:	afc3002c 	sw	v1,44(s8)
8000f8f0:	8c420000 	lw	v0,0(v0)
8000f8f4:	00021c02 	srl	v1,v0,0x10
8000f8f8:	8fc2005c 	lw	v0,92(s8)
8000f8fc:	00620018 	mult	v1,v0
8000f900:	8fc20034 	lw	v0,52(s8)
8000f904:	8c420000 	lw	v0,0(v0)
8000f908:	3042ffff 	andi	v0,v0,0xffff
8000f90c:	00001812 	mflo	v1
8000f910:	00621021 	addu	v0,v1,v0
8000f914:	8fc30028 	lw	v1,40(s8)
8000f918:	00621021 	addu	v0,v1,v0
8000f91c:	afc2003c 	sw	v0,60(s8)
8000f920:	8fc2003c 	lw	v0,60(s8)
8000f924:	00021402 	srl	v0,v0,0x10
8000f928:	afc20028 	sw	v0,40(s8)
8000f92c:	8fc3002c 	lw	v1,44(s8)
8000f930:	8fc20054 	lw	v0,84(s8)
8000f934:	0062102b 	sltu	v0,v1,v0
8000f938:	1440ffd1 	bnez	v0,8000f880 <__multiply+0x2cc>
8000f93c:	00000000 	nop
8000f940:	8fc20034 	lw	v0,52(s8)
8000f944:	8fc3003c 	lw	v1,60(s8)
8000f948:	ac430000 	sw	v1,0(v0)
8000f94c:	8fc20030 	lw	v0,48(s8)
8000f950:	24420004 	addiu	v0,v0,4
8000f954:	afc20030 	sw	v0,48(s8)
8000f958:	8fc20038 	lw	v0,56(s8)
8000f95c:	24420004 	addiu	v0,v0,4
8000f960:	afc20038 	sw	v0,56(s8)
8000f964:	8fc30030 	lw	v1,48(s8)
8000f968:	8fc20058 	lw	v0,88(s8)
8000f96c:	0062102b 	sltu	v0,v1,v0
8000f970:	1440ff75 	bnez	v0,8000f748 <__multiply+0x194>
8000f974:	00000000 	nop
8000f978:	8fc20040 	lw	v0,64(s8)
8000f97c:	24420014 	addiu	v0,v0,20
8000f980:	afc20038 	sw	v0,56(s8)
8000f984:	8fc20024 	lw	v0,36(s8)
8000f988:	00021080 	sll	v0,v0,0x2
8000f98c:	8fc30038 	lw	v1,56(s8)
8000f990:	00621021 	addu	v0,v1,v0
8000f994:	afc20034 	sw	v0,52(s8)
8000f998:	10000004 	b	8000f9ac <__multiply+0x3f8>
8000f99c:	00000000 	nop
8000f9a0:	8fc20024 	lw	v0,36(s8)
8000f9a4:	2442ffff 	addiu	v0,v0,-1
8000f9a8:	afc20024 	sw	v0,36(s8)
8000f9ac:	8fc20024 	lw	v0,36(s8)
8000f9b0:	18400008 	blez	v0,8000f9d4 <__multiply+0x420>
8000f9b4:	00000000 	nop
8000f9b8:	8fc20034 	lw	v0,52(s8)
8000f9bc:	2442fffc 	addiu	v0,v0,-4
8000f9c0:	afc20034 	sw	v0,52(s8)
8000f9c4:	8fc20034 	lw	v0,52(s8)
8000f9c8:	8c420000 	lw	v0,0(v0)
8000f9cc:	1040fff4 	beqz	v0,8000f9a0 <__multiply+0x3ec>
8000f9d0:	00000000 	nop
8000f9d4:	8fc20040 	lw	v0,64(s8)
8000f9d8:	8fc30024 	lw	v1,36(s8)
8000f9dc:	ac430010 	sw	v1,16(v0)
8000f9e0:	8fc20040 	lw	v0,64(s8)
8000f9e4:	03c0e825 	move	sp,s8
8000f9e8:	dfbf0070 	ld	ra,112(sp)
8000f9ec:	dfbe0068 	ld	s8,104(sp)
8000f9f0:	27bd0078 	addiu	sp,sp,120
8000f9f4:	03e00008 	jr	ra
8000f9f8:	00000000 	nop

8000f9fc <__pow5mult>:
8000f9fc:	27bdffc0 	addiu	sp,sp,-64
8000fa00:	ffbf0038 	sd	ra,56(sp)
8000fa04:	ffbe0030 	sd	s8,48(sp)
8000fa08:	03a0f025 	move	s8,sp
8000fa0c:	afc40040 	sw	a0,64(s8)
8000fa10:	afc50048 	sw	a1,72(s8)
8000fa14:	afc60050 	sw	a2,80(s8)
8000fa18:	8fc20050 	lw	v0,80(s8)
8000fa1c:	30420003 	andi	v0,v0,0x3
8000fa20:	afc20028 	sw	v0,40(s8)
8000fa24:	8fc20028 	lw	v0,40(s8)
8000fa28:	1040000f 	beqz	v0,8000fa68 <__pow5mult+0x6c>
8000fa2c:	00000000 	nop
8000fa30:	8fc20028 	lw	v0,40(s8)
8000fa34:	2443ffff 	addiu	v1,v0,-1
8000fa38:	3c028002 	lui	v0,0x8002
8000fa3c:	00031880 	sll	v1,v1,0x2
8000fa40:	2442a948 	addiu	v0,v0,-22200
8000fa44:	00621021 	addu	v0,v1,v0
8000fa48:	8c420000 	lw	v0,0(v0)
8000fa4c:	00003825 	move	a3,zero
8000fa50:	00403025 	move	a2,v0
8000fa54:	8fc50048 	lw	a1,72(s8)
8000fa58:	8fc40040 	lw	a0,64(s8)
8000fa5c:	0c003bcd 	jal	8000ef34 <__multadd>
8000fa60:	00000000 	nop
8000fa64:	afc20048 	sw	v0,72(s8)
8000fa68:	8fc20050 	lw	v0,80(s8)
8000fa6c:	00021083 	sra	v0,v0,0x2
8000fa70:	afc20050 	sw	v0,80(s8)
8000fa74:	8fc20050 	lw	v0,80(s8)
8000fa78:	14400004 	bnez	v0,8000fa8c <__pow5mult+0x90>
8000fa7c:	00000000 	nop
8000fa80:	8fc20048 	lw	v0,72(s8)
8000fa84:	10000042 	b	8000fb90 <__pow5mult+0x194>
8000fa88:	00000000 	nop
8000fa8c:	8fc20040 	lw	v0,64(s8)
8000fa90:	8c420048 	lw	v0,72(v0)
8000fa94:	afc20020 	sw	v0,32(s8)
8000fa98:	8fc20020 	lw	v0,32(s8)
8000fa9c:	1440000d 	bnez	v0,8000fad4 <__pow5mult+0xd8>
8000faa0:	00000000 	nop
8000faa4:	24050271 	li	a1,625
8000faa8:	8fc40040 	lw	a0,64(s8)
8000faac:	0c003d48 	jal	8000f520 <__i2b>
8000fab0:	00000000 	nop
8000fab4:	00401825 	move	v1,v0
8000fab8:	8fc20040 	lw	v0,64(s8)
8000fabc:	ac430048 	sw	v1,72(v0)
8000fac0:	8fc20040 	lw	v0,64(s8)
8000fac4:	8c420048 	lw	v0,72(v0)
8000fac8:	afc20020 	sw	v0,32(s8)
8000facc:	8fc20020 	lw	v0,32(s8)
8000fad0:	ac400000 	sw	zero,0(v0)
8000fad4:	8fc20050 	lw	v0,80(s8)
8000fad8:	30420001 	andi	v0,v0,0x1
8000fadc:	1040000d 	beqz	v0,8000fb14 <__pow5mult+0x118>
8000fae0:	00000000 	nop
8000fae4:	8fc60020 	lw	a2,32(s8)
8000fae8:	8fc50048 	lw	a1,72(s8)
8000faec:	8fc40040 	lw	a0,64(s8)
8000faf0:	0c003d6d 	jal	8000f5b4 <__multiply>
8000faf4:	00000000 	nop
8000faf8:	afc2002c 	sw	v0,44(s8)
8000fafc:	8fc50048 	lw	a1,72(s8)
8000fb00:	8fc40040 	lw	a0,64(s8)
8000fb04:	0c003bae 	jal	8000eeb8 <_Bfree>
8000fb08:	00000000 	nop
8000fb0c:	8fc2002c 	lw	v0,44(s8)
8000fb10:	afc20048 	sw	v0,72(s8)
8000fb14:	8fc20050 	lw	v0,80(s8)
8000fb18:	00021043 	sra	v0,v0,0x1
8000fb1c:	afc20050 	sw	v0,80(s8)
8000fb20:	8fc20050 	lw	v0,80(s8)
8000fb24:	10400018 	beqz	v0,8000fb88 <__pow5mult+0x18c>
8000fb28:	00000000 	nop
8000fb2c:	8fc20020 	lw	v0,32(s8)
8000fb30:	8c420000 	lw	v0,0(v0)
8000fb34:	afc20024 	sw	v0,36(s8)
8000fb38:	8fc20024 	lw	v0,36(s8)
8000fb3c:	1440000e 	bnez	v0,8000fb78 <__pow5mult+0x17c>
8000fb40:	00000000 	nop
8000fb44:	8fc60020 	lw	a2,32(s8)
8000fb48:	8fc50020 	lw	a1,32(s8)
8000fb4c:	8fc40040 	lw	a0,64(s8)
8000fb50:	0c003d6d 	jal	8000f5b4 <__multiply>
8000fb54:	00000000 	nop
8000fb58:	00401825 	move	v1,v0
8000fb5c:	8fc20020 	lw	v0,32(s8)
8000fb60:	ac430000 	sw	v1,0(v0)
8000fb64:	8fc20020 	lw	v0,32(s8)
8000fb68:	8c420000 	lw	v0,0(v0)
8000fb6c:	afc20024 	sw	v0,36(s8)
8000fb70:	8fc20024 	lw	v0,36(s8)
8000fb74:	ac400000 	sw	zero,0(v0)
8000fb78:	8fc20024 	lw	v0,36(s8)
8000fb7c:	afc20020 	sw	v0,32(s8)
8000fb80:	1000ffd4 	b	8000fad4 <__pow5mult+0xd8>
8000fb84:	00000000 	nop
8000fb88:	00000000 	nop
8000fb8c:	8fc20048 	lw	v0,72(s8)
8000fb90:	03c0e825 	move	sp,s8
8000fb94:	dfbf0038 	ld	ra,56(sp)
8000fb98:	dfbe0030 	ld	s8,48(sp)
8000fb9c:	27bd0040 	addiu	sp,sp,64
8000fba0:	03e00008 	jr	ra
8000fba4:	00000000 	nop

8000fba8 <__lshift>:
8000fba8:	27bdffa8 	addiu	sp,sp,-88
8000fbac:	ffbf0050 	sd	ra,80(sp)
8000fbb0:	ffbe0048 	sd	s8,72(sp)
8000fbb4:	03a0f025 	move	s8,sp
8000fbb8:	afc40058 	sw	a0,88(s8)
8000fbbc:	afc50060 	sw	a1,96(s8)
8000fbc0:	afc60068 	sw	a2,104(s8)
8000fbc4:	8fc20068 	lw	v0,104(s8)
8000fbc8:	00021143 	sra	v0,v0,0x5
8000fbcc:	afc20038 	sw	v0,56(s8)
8000fbd0:	8fc20060 	lw	v0,96(s8)
8000fbd4:	8c420004 	lw	v0,4(v0)
8000fbd8:	afc20024 	sw	v0,36(s8)
8000fbdc:	8fc20060 	lw	v0,96(s8)
8000fbe0:	8c430010 	lw	v1,16(v0)
8000fbe4:	8fc20038 	lw	v0,56(s8)
8000fbe8:	00621021 	addu	v0,v1,v0
8000fbec:	24420001 	addiu	v0,v0,1
8000fbf0:	afc20028 	sw	v0,40(s8)
8000fbf4:	8fc20060 	lw	v0,96(s8)
8000fbf8:	8c420008 	lw	v0,8(v0)
8000fbfc:	afc20020 	sw	v0,32(s8)
8000fc00:	10000007 	b	8000fc20 <__lshift+0x78>
8000fc04:	00000000 	nop
8000fc08:	8fc20024 	lw	v0,36(s8)
8000fc0c:	24420001 	addiu	v0,v0,1
8000fc10:	afc20024 	sw	v0,36(s8)
8000fc14:	8fc20020 	lw	v0,32(s8)
8000fc18:	00021040 	sll	v0,v0,0x1
8000fc1c:	afc20020 	sw	v0,32(s8)
8000fc20:	8fc30028 	lw	v1,40(s8)
8000fc24:	8fc20020 	lw	v0,32(s8)
8000fc28:	0043102a 	slt	v0,v0,v1
8000fc2c:	1440fff6 	bnez	v0,8000fc08 <__lshift+0x60>
8000fc30:	00000000 	nop
8000fc34:	8fc50024 	lw	a1,36(s8)
8000fc38:	8fc40058 	lw	a0,88(s8)
8000fc3c:	0c003b5b 	jal	8000ed6c <_Balloc>
8000fc40:	00000000 	nop
8000fc44:	afc2003c 	sw	v0,60(s8)
8000fc48:	8fc2003c 	lw	v0,60(s8)
8000fc4c:	14400009 	bnez	v0,8000fc74 <__lshift+0xcc>
8000fc50:	00000000 	nop
8000fc54:	3c028002 	lui	v0,0x8002
8000fc58:	2447a7e8 	addiu	a3,v0,-22552
8000fc5c:	00003025 	move	a2,zero
8000fc60:	240501d9 	li	a1,473
8000fc64:	3c028002 	lui	v0,0x8002
8000fc68:	2444a800 	addiu	a0,v0,-22528
8000fc6c:	0c0068d2 	jal	8001a348 <__assert_func>
8000fc70:	00000000 	nop
8000fc74:	8fc2003c 	lw	v0,60(s8)
8000fc78:	afc20040 	sw	v0,64(s8)
8000fc7c:	8fc20040 	lw	v0,64(s8)
8000fc80:	24420014 	addiu	v0,v0,20
8000fc84:	afc20030 	sw	v0,48(s8)
8000fc88:	afc00020 	sw	zero,32(s8)
8000fc8c:	10000008 	b	8000fcb0 <__lshift+0x108>
8000fc90:	00000000 	nop
8000fc94:	8fc20030 	lw	v0,48(s8)
8000fc98:	24430004 	addiu	v1,v0,4
8000fc9c:	afc30030 	sw	v1,48(s8)
8000fca0:	ac400000 	sw	zero,0(v0)
8000fca4:	8fc20020 	lw	v0,32(s8)
8000fca8:	24420001 	addiu	v0,v0,1
8000fcac:	afc20020 	sw	v0,32(s8)
8000fcb0:	8fc30020 	lw	v1,32(s8)
8000fcb4:	8fc20038 	lw	v0,56(s8)
8000fcb8:	0062102a 	slt	v0,v1,v0
8000fcbc:	1440fff5 	bnez	v0,8000fc94 <__lshift+0xec>
8000fcc0:	00000000 	nop
8000fcc4:	8fc20060 	lw	v0,96(s8)
8000fcc8:	24420014 	addiu	v0,v0,20
8000fccc:	afc2002c 	sw	v0,44(s8)
8000fcd0:	8fc20060 	lw	v0,96(s8)
8000fcd4:	8c420010 	lw	v0,16(v0)
8000fcd8:	00021080 	sll	v0,v0,0x2
8000fcdc:	8fc3002c 	lw	v1,44(s8)
8000fce0:	00621021 	addu	v0,v1,v0
8000fce4:	afc20044 	sw	v0,68(s8)
8000fce8:	8fc20068 	lw	v0,104(s8)
8000fcec:	3042001f 	andi	v0,v0,0x1f
8000fcf0:	afc20068 	sw	v0,104(s8)
8000fcf4:	8fc20068 	lw	v0,104(s8)
8000fcf8:	10400028 	beqz	v0,8000fd9c <__lshift+0x1f4>
8000fcfc:	00000000 	nop
8000fd00:	24030020 	li	v1,32
8000fd04:	8fc20068 	lw	v0,104(s8)
8000fd08:	00621023 	subu	v0,v1,v0
8000fd0c:	afc20024 	sw	v0,36(s8)
8000fd10:	afc00034 	sw	zero,52(s8)
8000fd14:	8fc2002c 	lw	v0,44(s8)
8000fd18:	8c430000 	lw	v1,0(v0)
8000fd1c:	8fc20068 	lw	v0,104(s8)
8000fd20:	00432004 	sllv	a0,v1,v0
8000fd24:	8fc20030 	lw	v0,48(s8)
8000fd28:	24430004 	addiu	v1,v0,4
8000fd2c:	afc30030 	sw	v1,48(s8)
8000fd30:	8fc30034 	lw	v1,52(s8)
8000fd34:	00831825 	or	v1,a0,v1
8000fd38:	ac430000 	sw	v1,0(v0)
8000fd3c:	8fc2002c 	lw	v0,44(s8)
8000fd40:	24430004 	addiu	v1,v0,4
8000fd44:	afc3002c 	sw	v1,44(s8)
8000fd48:	8c430000 	lw	v1,0(v0)
8000fd4c:	8fc20024 	lw	v0,36(s8)
8000fd50:	00431006 	srlv	v0,v1,v0
8000fd54:	afc20034 	sw	v0,52(s8)
8000fd58:	8fc3002c 	lw	v1,44(s8)
8000fd5c:	8fc20044 	lw	v0,68(s8)
8000fd60:	0062102b 	sltu	v0,v1,v0
8000fd64:	1440ffeb 	bnez	v0,8000fd14 <__lshift+0x16c>
8000fd68:	00000000 	nop
8000fd6c:	8fc20030 	lw	v0,48(s8)
8000fd70:	8fc30034 	lw	v1,52(s8)
8000fd74:	ac430000 	sw	v1,0(v0)
8000fd78:	8fc20030 	lw	v0,48(s8)
8000fd7c:	8c420000 	lw	v0,0(v0)
8000fd80:	10400013 	beqz	v0,8000fdd0 <__lshift+0x228>
8000fd84:	00000000 	nop
8000fd88:	8fc20028 	lw	v0,40(s8)
8000fd8c:	24420001 	addiu	v0,v0,1
8000fd90:	afc20028 	sw	v0,40(s8)
8000fd94:	1000000e 	b	8000fdd0 <__lshift+0x228>
8000fd98:	00000000 	nop
8000fd9c:	8fc3002c 	lw	v1,44(s8)
8000fda0:	24620004 	addiu	v0,v1,4
8000fda4:	afc2002c 	sw	v0,44(s8)
8000fda8:	8fc20030 	lw	v0,48(s8)
8000fdac:	24440004 	addiu	a0,v0,4
8000fdb0:	afc40030 	sw	a0,48(s8)
8000fdb4:	8c630000 	lw	v1,0(v1)
8000fdb8:	ac430000 	sw	v1,0(v0)
8000fdbc:	8fc3002c 	lw	v1,44(s8)
8000fdc0:	8fc20044 	lw	v0,68(s8)
8000fdc4:	0062102b 	sltu	v0,v1,v0
8000fdc8:	1440fff4 	bnez	v0,8000fd9c <__lshift+0x1f4>
8000fdcc:	00000000 	nop
8000fdd0:	8fc20028 	lw	v0,40(s8)
8000fdd4:	2443ffff 	addiu	v1,v0,-1
8000fdd8:	8fc20040 	lw	v0,64(s8)
8000fddc:	ac430010 	sw	v1,16(v0)
8000fde0:	8fc50060 	lw	a1,96(s8)
8000fde4:	8fc40058 	lw	a0,88(s8)
8000fde8:	0c003bae 	jal	8000eeb8 <_Bfree>
8000fdec:	00000000 	nop
8000fdf0:	8fc20040 	lw	v0,64(s8)
8000fdf4:	03c0e825 	move	sp,s8
8000fdf8:	dfbf0050 	ld	ra,80(sp)
8000fdfc:	dfbe0048 	ld	s8,72(sp)
8000fe00:	27bd0058 	addiu	sp,sp,88
8000fe04:	03e00008 	jr	ra
8000fe08:	00000000 	nop

8000fe0c <__mcmp>:
8000fe0c:	27bdffe0 	addiu	sp,sp,-32
8000fe10:	ffbe0018 	sd	s8,24(sp)
8000fe14:	03a0f025 	move	s8,sp
8000fe18:	afc40020 	sw	a0,32(s8)
8000fe1c:	afc50028 	sw	a1,40(s8)
8000fe20:	8fc20020 	lw	v0,32(s8)
8000fe24:	8c420010 	lw	v0,16(v0)
8000fe28:	afc20008 	sw	v0,8(s8)
8000fe2c:	8fc20028 	lw	v0,40(s8)
8000fe30:	8c420010 	lw	v0,16(v0)
8000fe34:	afc2000c 	sw	v0,12(s8)
8000fe38:	8fc30008 	lw	v1,8(s8)
8000fe3c:	8fc2000c 	lw	v0,12(s8)
8000fe40:	00621023 	subu	v0,v1,v0
8000fe44:	afc20008 	sw	v0,8(s8)
8000fe48:	8fc20008 	lw	v0,8(s8)
8000fe4c:	10400004 	beqz	v0,8000fe60 <__mcmp+0x54>
8000fe50:	00000000 	nop
8000fe54:	8fc20008 	lw	v0,8(s8)
8000fe58:	10000033 	b	8000ff28 <__mcmp+0x11c>
8000fe5c:	00000000 	nop
8000fe60:	8fc20020 	lw	v0,32(s8)
8000fe64:	24420014 	addiu	v0,v0,20
8000fe68:	afc20010 	sw	v0,16(s8)
8000fe6c:	8fc2000c 	lw	v0,12(s8)
8000fe70:	00021080 	sll	v0,v0,0x2
8000fe74:	8fc30010 	lw	v1,16(s8)
8000fe78:	00621021 	addu	v0,v1,v0
8000fe7c:	afc20000 	sw	v0,0(s8)
8000fe80:	8fc20028 	lw	v0,40(s8)
8000fe84:	24420014 	addiu	v0,v0,20
8000fe88:	afc20014 	sw	v0,20(s8)
8000fe8c:	8fc2000c 	lw	v0,12(s8)
8000fe90:	00021080 	sll	v0,v0,0x2
8000fe94:	8fc30014 	lw	v1,20(s8)
8000fe98:	00621021 	addu	v0,v1,v0
8000fe9c:	afc20004 	sw	v0,4(s8)
8000fea0:	8fc20000 	lw	v0,0(s8)
8000fea4:	2442fffc 	addiu	v0,v0,-4
8000fea8:	afc20000 	sw	v0,0(s8)
8000feac:	8fc20000 	lw	v0,0(s8)
8000feb0:	8c430000 	lw	v1,0(v0)
8000feb4:	8fc20004 	lw	v0,4(s8)
8000feb8:	2442fffc 	addiu	v0,v0,-4
8000febc:	afc20004 	sw	v0,4(s8)
8000fec0:	8fc20004 	lw	v0,4(s8)
8000fec4:	8c420000 	lw	v0,0(v0)
8000fec8:	1062000e 	beq	v1,v0,8000ff04 <__mcmp+0xf8>
8000fecc:	00000000 	nop
8000fed0:	8fc20000 	lw	v0,0(s8)
8000fed4:	8c430000 	lw	v1,0(v0)
8000fed8:	8fc20004 	lw	v0,4(s8)
8000fedc:	8c420000 	lw	v0,0(v0)
8000fee0:	0062102b 	sltu	v0,v1,v0
8000fee4:	10400004 	beqz	v0,8000fef8 <__mcmp+0xec>
8000fee8:	00000000 	nop
8000feec:	2402ffff 	li	v0,-1
8000fef0:	1000000d 	b	8000ff28 <__mcmp+0x11c>
8000fef4:	00000000 	nop
8000fef8:	24020001 	li	v0,1
8000fefc:	1000000a 	b	8000ff28 <__mcmp+0x11c>
8000ff00:	00000000 	nop
8000ff04:	8fc30000 	lw	v1,0(s8)
8000ff08:	8fc20010 	lw	v0,16(s8)
8000ff0c:	0043102b 	sltu	v0,v0,v1
8000ff10:	10400003 	beqz	v0,8000ff20 <__mcmp+0x114>
8000ff14:	00000000 	nop
8000ff18:	1000ffe1 	b	8000fea0 <__mcmp+0x94>
8000ff1c:	00000000 	nop
8000ff20:	00000000 	nop
8000ff24:	00001025 	move	v0,zero
8000ff28:	03c0e825 	move	sp,s8
8000ff2c:	dfbe0018 	ld	s8,24(sp)
8000ff30:	27bd0020 	addiu	sp,sp,32
8000ff34:	03e00008 	jr	ra
8000ff38:	00000000 	nop

8000ff3c <__mdiff>:
8000ff3c:	27bdff98 	addiu	sp,sp,-104
8000ff40:	ffbf0060 	sd	ra,96(sp)
8000ff44:	ffbe0058 	sd	s8,88(sp)
8000ff48:	03a0f025 	move	s8,sp
8000ff4c:	afc40068 	sw	a0,104(s8)
8000ff50:	afc50070 	sw	a1,112(s8)
8000ff54:	afc60078 	sw	a2,120(s8)
8000ff58:	8fc50078 	lw	a1,120(s8)
8000ff5c:	8fc40070 	lw	a0,112(s8)
8000ff60:	0c003f83 	jal	8000fe0c <__mcmp>
8000ff64:	00000000 	nop
8000ff68:	afc20020 	sw	v0,32(s8)
8000ff6c:	8fc20020 	lw	v0,32(s8)
8000ff70:	1440001b 	bnez	v0,8000ffe0 <__mdiff+0xa4>
8000ff74:	00000000 	nop
8000ff78:	00002825 	move	a1,zero
8000ff7c:	8fc40068 	lw	a0,104(s8)
8000ff80:	0c003b5b 	jal	8000ed6c <_Balloc>
8000ff84:	00000000 	nop
8000ff88:	afc20054 	sw	v0,84(s8)
8000ff8c:	8fc20054 	lw	v0,84(s8)
8000ff90:	14400009 	bnez	v0,8000ffb8 <__mdiff+0x7c>
8000ff94:	00000000 	nop
8000ff98:	3c028002 	lui	v0,0x8002
8000ff9c:	2447a7e8 	addiu	a3,v0,-22552
8000ffa0:	00003025 	move	a2,zero
8000ffa4:	24050232 	li	a1,562
8000ffa8:	3c028002 	lui	v0,0x8002
8000ffac:	2444a800 	addiu	a0,v0,-22528
8000ffb0:	0c0068d2 	jal	8001a348 <__assert_func>
8000ffb4:	00000000 	nop
8000ffb8:	8fc20054 	lw	v0,84(s8)
8000ffbc:	afc20038 	sw	v0,56(s8)
8000ffc0:	8fc20038 	lw	v0,56(s8)
8000ffc4:	24030001 	li	v1,1
8000ffc8:	ac430010 	sw	v1,16(v0)
8000ffcc:	8fc20038 	lw	v0,56(s8)
8000ffd0:	ac400014 	sw	zero,20(v0)
8000ffd4:	8fc20038 	lw	v0,56(s8)
8000ffd8:	100000a2 	b	80010264 <__mdiff+0x328>
8000ffdc:	00000000 	nop
8000ffe0:	8fc20020 	lw	v0,32(s8)
8000ffe4:	0441000b 	bgez	v0,80010014 <__mdiff+0xd8>
8000ffe8:	00000000 	nop
8000ffec:	8fc20070 	lw	v0,112(s8)
8000fff0:	afc20038 	sw	v0,56(s8)
8000fff4:	8fc20078 	lw	v0,120(s8)
8000fff8:	afc20070 	sw	v0,112(s8)
8000fffc:	8fc20038 	lw	v0,56(s8)
80010000:	afc20078 	sw	v0,120(s8)
80010004:	24020001 	li	v0,1
80010008:	afc20020 	sw	v0,32(s8)
8001000c:	10000002 	b	80010018 <__mdiff+0xdc>
80010010:	00000000 	nop
80010014:	afc00020 	sw	zero,32(s8)
80010018:	8fc20070 	lw	v0,112(s8)
8001001c:	8c420004 	lw	v0,4(v0)
80010020:	00402825 	move	a1,v0
80010024:	8fc40068 	lw	a0,104(s8)
80010028:	0c003b5b 	jal	8000ed6c <_Balloc>
8001002c:	00000000 	nop
80010030:	afc2003c 	sw	v0,60(s8)
80010034:	8fc2003c 	lw	v0,60(s8)
80010038:	14400009 	bnez	v0,80010060 <__mdiff+0x124>
8001003c:	00000000 	nop
80010040:	3c028002 	lui	v0,0x8002
80010044:	2447a7e8 	addiu	a3,v0,-22552
80010048:	00003025 	move	a2,zero
8001004c:	24050240 	li	a1,576
80010050:	3c028002 	lui	v0,0x8002
80010054:	2444a800 	addiu	a0,v0,-22528
80010058:	0c0068d2 	jal	8001a348 <__assert_func>
8001005c:	00000000 	nop
80010060:	8fc2003c 	lw	v0,60(s8)
80010064:	afc20038 	sw	v0,56(s8)
80010068:	8fc20038 	lw	v0,56(s8)
8001006c:	8fc30020 	lw	v1,32(s8)
80010070:	ac43000c 	sw	v1,12(v0)
80010074:	8fc20070 	lw	v0,112(s8)
80010078:	8c420010 	lw	v0,16(v0)
8001007c:	afc20024 	sw	v0,36(s8)
80010080:	8fc20070 	lw	v0,112(s8)
80010084:	24420014 	addiu	v0,v0,20
80010088:	afc2002c 	sw	v0,44(s8)
8001008c:	8fc20024 	lw	v0,36(s8)
80010090:	00021080 	sll	v0,v0,0x2
80010094:	8fc3002c 	lw	v1,44(s8)
80010098:	00621021 	addu	v0,v1,v0
8001009c:	afc20040 	sw	v0,64(s8)
800100a0:	8fc20078 	lw	v0,120(s8)
800100a4:	8c420010 	lw	v0,16(v0)
800100a8:	afc20044 	sw	v0,68(s8)
800100ac:	8fc20078 	lw	v0,120(s8)
800100b0:	24420014 	addiu	v0,v0,20
800100b4:	afc20030 	sw	v0,48(s8)
800100b8:	8fc20044 	lw	v0,68(s8)
800100bc:	00021080 	sll	v0,v0,0x2
800100c0:	8fc30030 	lw	v1,48(s8)
800100c4:	00621021 	addu	v0,v1,v0
800100c8:	afc20048 	sw	v0,72(s8)
800100cc:	8fc20038 	lw	v0,56(s8)
800100d0:	24420014 	addiu	v0,v0,20
800100d4:	afc20034 	sw	v0,52(s8)
800100d8:	afc00028 	sw	zero,40(s8)
800100dc:	8fc2002c 	lw	v0,44(s8)
800100e0:	8c420000 	lw	v0,0(v0)
800100e4:	3043ffff 	andi	v1,v0,0xffff
800100e8:	8fc20030 	lw	v0,48(s8)
800100ec:	8c420000 	lw	v0,0(v0)
800100f0:	3042ffff 	andi	v0,v0,0xffff
800100f4:	00621823 	subu	v1,v1,v0
800100f8:	8fc20028 	lw	v0,40(s8)
800100fc:	00621021 	addu	v0,v1,v0
80010100:	afc2004c 	sw	v0,76(s8)
80010104:	8fc2004c 	lw	v0,76(s8)
80010108:	00021403 	sra	v0,v0,0x10
8001010c:	afc20028 	sw	v0,40(s8)
80010110:	8fc2002c 	lw	v0,44(s8)
80010114:	24430004 	addiu	v1,v0,4
80010118:	afc3002c 	sw	v1,44(s8)
8001011c:	8c420000 	lw	v0,0(v0)
80010120:	00021c02 	srl	v1,v0,0x10
80010124:	8fc20030 	lw	v0,48(s8)
80010128:	24440004 	addiu	a0,v0,4
8001012c:	afc40030 	sw	a0,48(s8)
80010130:	8c420000 	lw	v0,0(v0)
80010134:	00021402 	srl	v0,v0,0x10
80010138:	00621823 	subu	v1,v1,v0
8001013c:	8fc20028 	lw	v0,40(s8)
80010140:	00621021 	addu	v0,v1,v0
80010144:	afc20050 	sw	v0,80(s8)
80010148:	8fc20050 	lw	v0,80(s8)
8001014c:	00021403 	sra	v0,v0,0x10
80010150:	afc20028 	sw	v0,40(s8)
80010154:	8fc20050 	lw	v0,80(s8)
80010158:	00021c00 	sll	v1,v0,0x10
8001015c:	8fc2004c 	lw	v0,76(s8)
80010160:	3042ffff 	andi	v0,v0,0xffff
80010164:	00622025 	or	a0,v1,v0
80010168:	8fc20034 	lw	v0,52(s8)
8001016c:	24430004 	addiu	v1,v0,4
80010170:	afc30034 	sw	v1,52(s8)
80010174:	00801825 	move	v1,a0
80010178:	ac430000 	sw	v1,0(v0)
8001017c:	8fc30030 	lw	v1,48(s8)
80010180:	8fc20048 	lw	v0,72(s8)
80010184:	0062102b 	sltu	v0,v1,v0
80010188:	1440ffd4 	bnez	v0,800100dc <__mdiff+0x1a0>
8001018c:	00000000 	nop
80010190:	1000001f 	b	80010210 <__mdiff+0x2d4>
80010194:	00000000 	nop
80010198:	8fc2002c 	lw	v0,44(s8)
8001019c:	8c420000 	lw	v0,0(v0)
800101a0:	3043ffff 	andi	v1,v0,0xffff
800101a4:	8fc20028 	lw	v0,40(s8)
800101a8:	00621021 	addu	v0,v1,v0
800101ac:	afc2004c 	sw	v0,76(s8)
800101b0:	8fc2004c 	lw	v0,76(s8)
800101b4:	00021403 	sra	v0,v0,0x10
800101b8:	afc20028 	sw	v0,40(s8)
800101bc:	8fc2002c 	lw	v0,44(s8)
800101c0:	24430004 	addiu	v1,v0,4
800101c4:	afc3002c 	sw	v1,44(s8)
800101c8:	8c420000 	lw	v0,0(v0)
800101cc:	00021c02 	srl	v1,v0,0x10
800101d0:	8fc20028 	lw	v0,40(s8)
800101d4:	00621021 	addu	v0,v1,v0
800101d8:	afc20050 	sw	v0,80(s8)
800101dc:	8fc20050 	lw	v0,80(s8)
800101e0:	00021403 	sra	v0,v0,0x10
800101e4:	afc20028 	sw	v0,40(s8)
800101e8:	8fc20050 	lw	v0,80(s8)
800101ec:	00021c00 	sll	v1,v0,0x10
800101f0:	8fc2004c 	lw	v0,76(s8)
800101f4:	3042ffff 	andi	v0,v0,0xffff
800101f8:	00622025 	or	a0,v1,v0
800101fc:	8fc20034 	lw	v0,52(s8)
80010200:	24430004 	addiu	v1,v0,4
80010204:	afc30034 	sw	v1,52(s8)
80010208:	00801825 	move	v1,a0
8001020c:	ac430000 	sw	v1,0(v0)
80010210:	8fc3002c 	lw	v1,44(s8)
80010214:	8fc20040 	lw	v0,64(s8)
80010218:	0062102b 	sltu	v0,v1,v0
8001021c:	1440ffde 	bnez	v0,80010198 <__mdiff+0x25c>
80010220:	00000000 	nop
80010224:	10000004 	b	80010238 <__mdiff+0x2fc>
80010228:	00000000 	nop
8001022c:	8fc20024 	lw	v0,36(s8)
80010230:	2442ffff 	addiu	v0,v0,-1
80010234:	afc20024 	sw	v0,36(s8)
80010238:	8fc20034 	lw	v0,52(s8)
8001023c:	2442fffc 	addiu	v0,v0,-4
80010240:	afc20034 	sw	v0,52(s8)
80010244:	8fc20034 	lw	v0,52(s8)
80010248:	8c420000 	lw	v0,0(v0)
8001024c:	1040fff7 	beqz	v0,8001022c <__mdiff+0x2f0>
80010250:	00000000 	nop
80010254:	8fc20038 	lw	v0,56(s8)
80010258:	8fc30024 	lw	v1,36(s8)
8001025c:	ac430010 	sw	v1,16(v0)
80010260:	8fc20038 	lw	v0,56(s8)
80010264:	03c0e825 	move	sp,s8
80010268:	dfbf0060 	ld	ra,96(sp)
8001026c:	dfbe0058 	ld	s8,88(sp)
80010270:	27bd0068 	addiu	sp,sp,104
80010274:	03e00008 	jr	ra
80010278:	00000000 	nop

8001027c <__ulp>:
8001027c:	27bdffe0 	addiu	sp,sp,-32
80010280:	ffbe0018 	sd	s8,24(sp)
80010284:	ffb00010 	sd	s0,16(sp)
80010288:	03a0f025 	move	s8,sp
8001028c:	f7cc0020 	sdc1	$f12,32(s8)
80010290:	d7c00020 	ldc1	$f0,32(s8)
80010294:	f7c00000 	sdc1	$f0,0(s8)
80010298:	8fc30000 	lw	v1,0(s8)
8001029c:	3c027ff0 	lui	v0,0x7ff0
800102a0:	00621824 	and	v1,v1,v0
800102a4:	3c02fcc0 	lui	v0,0xfcc0
800102a8:	00621021 	addu	v0,v1,v0
800102ac:	00408025 	move	s0,v0
800102b0:	1a000006 	blez	s0,800102cc <__ulp+0x50>
800102b4:	00000000 	nop
800102b8:	02001025 	move	v0,s0
800102bc:	afc20008 	sw	v0,8(s8)
800102c0:	afc0000c 	sw	zero,12(s8)
800102c4:	10000017 	b	80010324 <__ulp+0xa8>
800102c8:	00000000 	nop
800102cc:	00101023 	negu	v0,s0
800102d0:	00028503 	sra	s0,v0,0x14
800102d4:	2a020014 	slti	v0,s0,20
800102d8:	10400007 	beqz	v0,800102f8 <__ulp+0x7c>
800102dc:	00000000 	nop
800102e0:	3c020008 	lui	v0,0x8
800102e4:	02021007 	srav	v0,v0,s0
800102e8:	afc20008 	sw	v0,8(s8)
800102ec:	afc0000c 	sw	zero,12(s8)
800102f0:	1000000c 	b	80010324 <__ulp+0xa8>
800102f4:	00000000 	nop
800102f8:	afc00008 	sw	zero,8(s8)
800102fc:	2610ffec 	addiu	s0,s0,-20
80010300:	2a02001f 	slti	v0,s0,31
80010304:	10400005 	beqz	v0,8001031c <__ulp+0xa0>
80010308:	00000000 	nop
8001030c:	3c028000 	lui	v0,0x8000
80010310:	02021006 	srlv	v0,v0,s0
80010314:	10000002 	b	80010320 <__ulp+0xa4>
80010318:	00000000 	nop
8001031c:	24020001 	li	v0,1
80010320:	afc2000c 	sw	v0,12(s8)
80010324:	d7c00008 	ldc1	$f0,8(s8)
80010328:	03c0e825 	move	sp,s8
8001032c:	dfbe0018 	ld	s8,24(sp)
80010330:	dfb00010 	ld	s0,16(sp)
80010334:	27bd0020 	addiu	sp,sp,32
80010338:	03e00008 	jr	ra
8001033c:	00000000 	nop

80010340 <__b2d>:
80010340:	27bdffb0 	addiu	sp,sp,-80
80010344:	ffbf0048 	sd	ra,72(sp)
80010348:	ffbe0040 	sd	s8,64(sp)
8001034c:	03a0f025 	move	s8,sp
80010350:	afc40050 	sw	a0,80(s8)
80010354:	afc50058 	sw	a1,88(s8)
80010358:	8fc20050 	lw	v0,80(s8)
8001035c:	24420014 	addiu	v0,v0,20
80010360:	afc20024 	sw	v0,36(s8)
80010364:	8fc20050 	lw	v0,80(s8)
80010368:	8c420010 	lw	v0,16(v0)
8001036c:	00021080 	sll	v0,v0,0x2
80010370:	8fc30024 	lw	v1,36(s8)
80010374:	00621021 	addu	v0,v1,v0
80010378:	afc20020 	sw	v0,32(s8)
8001037c:	8fc20020 	lw	v0,32(s8)
80010380:	2442fffc 	addiu	v0,v0,-4
80010384:	afc20020 	sw	v0,32(s8)
80010388:	8fc20020 	lw	v0,32(s8)
8001038c:	8c420000 	lw	v0,0(v0)
80010390:	afc20028 	sw	v0,40(s8)
80010394:	8fc40028 	lw	a0,40(s8)
80010398:	0c003ccf 	jal	8000f33c <__hi0bits>
8001039c:	00000000 	nop
800103a0:	afc2002c 	sw	v0,44(s8)
800103a4:	24030020 	li	v1,32
800103a8:	8fc2002c 	lw	v0,44(s8)
800103ac:	00621823 	subu	v1,v1,v0
800103b0:	8fc20058 	lw	v0,88(s8)
800103b4:	ac430000 	sw	v1,0(v0)
800103b8:	8fc2002c 	lw	v0,44(s8)
800103bc:	2842000b 	slti	v0,v0,11
800103c0:	10400024 	beqz	v0,80010454 <__b2d+0x114>
800103c4:	00000000 	nop
800103c8:	2403000b 	li	v1,11
800103cc:	8fc2002c 	lw	v0,44(s8)
800103d0:	00621023 	subu	v0,v1,v0
800103d4:	8fc30028 	lw	v1,40(s8)
800103d8:	00431806 	srlv	v1,v1,v0
800103dc:	3c023ff0 	lui	v0,0x3ff0
800103e0:	00621025 	or	v0,v1,v0
800103e4:	afc20038 	sw	v0,56(s8)
800103e8:	8fc30020 	lw	v1,32(s8)
800103ec:	8fc20024 	lw	v0,36(s8)
800103f0:	0043102b 	sltu	v0,v0,v1
800103f4:	10400008 	beqz	v0,80010418 <__b2d+0xd8>
800103f8:	00000000 	nop
800103fc:	8fc20020 	lw	v0,32(s8)
80010400:	2442fffc 	addiu	v0,v0,-4
80010404:	afc20020 	sw	v0,32(s8)
80010408:	8fc20020 	lw	v0,32(s8)
8001040c:	8c420000 	lw	v0,0(v0)
80010410:	10000002 	b	8001041c <__b2d+0xdc>
80010414:	00000000 	nop
80010418:	00001025 	move	v0,zero
8001041c:	afc20034 	sw	v0,52(s8)
80010420:	8fc2002c 	lw	v0,44(s8)
80010424:	24420015 	addiu	v0,v0,21
80010428:	8fc30028 	lw	v1,40(s8)
8001042c:	00431804 	sllv	v1,v1,v0
80010430:	2404000b 	li	a0,11
80010434:	8fc2002c 	lw	v0,44(s8)
80010438:	00821023 	subu	v0,a0,v0
8001043c:	8fc40034 	lw	a0,52(s8)
80010440:	00441006 	srlv	v0,a0,v0
80010444:	00621025 	or	v0,v1,v0
80010448:	afc2003c 	sw	v0,60(s8)
8001044c:	10000041 	b	80010554 <__b2d+0x214>
80010450:	00000000 	nop
80010454:	8fc30020 	lw	v1,32(s8)
80010458:	8fc20024 	lw	v0,36(s8)
8001045c:	0043102b 	sltu	v0,v0,v1
80010460:	10400008 	beqz	v0,80010484 <__b2d+0x144>
80010464:	00000000 	nop
80010468:	8fc20020 	lw	v0,32(s8)
8001046c:	2442fffc 	addiu	v0,v0,-4
80010470:	afc20020 	sw	v0,32(s8)
80010474:	8fc20020 	lw	v0,32(s8)
80010478:	8c420000 	lw	v0,0(v0)
8001047c:	10000002 	b	80010488 <__b2d+0x148>
80010480:	00000000 	nop
80010484:	00001025 	move	v0,zero
80010488:	afc20030 	sw	v0,48(s8)
8001048c:	8fc2002c 	lw	v0,44(s8)
80010490:	2442fff5 	addiu	v0,v0,-11
80010494:	afc2002c 	sw	v0,44(s8)
80010498:	8fc2002c 	lw	v0,44(s8)
8001049c:	10400027 	beqz	v0,8001053c <__b2d+0x1fc>
800104a0:	00000000 	nop
800104a4:	8fc30028 	lw	v1,40(s8)
800104a8:	8fc2002c 	lw	v0,44(s8)
800104ac:	00431804 	sllv	v1,v1,v0
800104b0:	24040020 	li	a0,32
800104b4:	8fc2002c 	lw	v0,44(s8)
800104b8:	00821023 	subu	v0,a0,v0
800104bc:	8fc40030 	lw	a0,48(s8)
800104c0:	00441006 	srlv	v0,a0,v0
800104c4:	00621825 	or	v1,v1,v0
800104c8:	3c023ff0 	lui	v0,0x3ff0
800104cc:	00621025 	or	v0,v1,v0
800104d0:	afc20038 	sw	v0,56(s8)
800104d4:	8fc30020 	lw	v1,32(s8)
800104d8:	8fc20024 	lw	v0,36(s8)
800104dc:	0043102b 	sltu	v0,v0,v1
800104e0:	10400008 	beqz	v0,80010504 <__b2d+0x1c4>
800104e4:	00000000 	nop
800104e8:	8fc20020 	lw	v0,32(s8)
800104ec:	2442fffc 	addiu	v0,v0,-4
800104f0:	afc20020 	sw	v0,32(s8)
800104f4:	8fc20020 	lw	v0,32(s8)
800104f8:	8c420000 	lw	v0,0(v0)
800104fc:	10000002 	b	80010508 <__b2d+0x1c8>
80010500:	00000000 	nop
80010504:	00001025 	move	v0,zero
80010508:	afc20028 	sw	v0,40(s8)
8001050c:	8fc30030 	lw	v1,48(s8)
80010510:	8fc2002c 	lw	v0,44(s8)
80010514:	00431804 	sllv	v1,v1,v0
80010518:	24040020 	li	a0,32
8001051c:	8fc2002c 	lw	v0,44(s8)
80010520:	00821023 	subu	v0,a0,v0
80010524:	8fc40028 	lw	a0,40(s8)
80010528:	00441006 	srlv	v0,a0,v0
8001052c:	00621025 	or	v0,v1,v0
80010530:	afc2003c 	sw	v0,60(s8)
80010534:	10000007 	b	80010554 <__b2d+0x214>
80010538:	00000000 	nop
8001053c:	8fc30028 	lw	v1,40(s8)
80010540:	3c023ff0 	lui	v0,0x3ff0
80010544:	00621025 	or	v0,v1,v0
80010548:	afc20038 	sw	v0,56(s8)
8001054c:	8fc20030 	lw	v0,48(s8)
80010550:	afc2003c 	sw	v0,60(s8)
80010554:	d7c00038 	ldc1	$f0,56(s8)
80010558:	03c0e825 	move	sp,s8
8001055c:	dfbf0048 	ld	ra,72(sp)
80010560:	dfbe0040 	ld	s8,64(sp)
80010564:	27bd0050 	addiu	sp,sp,80
80010568:	03e00008 	jr	ra
8001056c:	00000000 	nop

80010570 <__d2b>:
80010570:	27bdffa0 	addiu	sp,sp,-96
80010574:	ffbf0058 	sd	ra,88(sp)
80010578:	ffbe0050 	sd	s8,80(sp)
8001057c:	ffb00048 	sd	s0,72(sp)
80010580:	03a0f025 	move	s8,sp
80010584:	afc40060 	sw	a0,96(s8)
80010588:	ffc50068 	sd	a1,104(s8)
8001058c:	afc60070 	sw	a2,112(s8)
80010590:	afc70078 	sw	a3,120(s8)
80010594:	d7c00068 	ldc1	$f0,104(s8)
80010598:	f7c00038 	sdc1	$f0,56(s8)
8001059c:	d7c00068 	ldc1	$f0,104(s8)
800105a0:	f7c00038 	sdc1	$f0,56(s8)
800105a4:	24050001 	li	a1,1
800105a8:	8fc40060 	lw	a0,96(s8)
800105ac:	0c003b5b 	jal	8000ed6c <_Balloc>
800105b0:	00000000 	nop
800105b4:	afc20028 	sw	v0,40(s8)
800105b8:	8fc20028 	lw	v0,40(s8)
800105bc:	14400009 	bnez	v0,800105e4 <__d2b+0x74>
800105c0:	00000000 	nop
800105c4:	3c028002 	lui	v0,0x8002
800105c8:	2447a7e8 	addiu	a3,v0,-22552
800105cc:	00003025 	move	a2,zero
800105d0:	2405030a 	li	a1,778
800105d4:	3c028002 	lui	v0,0x8002
800105d8:	2444a800 	addiu	a0,v0,-22528
800105dc:	0c0068d2 	jal	8001a348 <__assert_func>
800105e0:	00000000 	nop
800105e4:	8fc20028 	lw	v0,40(s8)
800105e8:	afc2002c 	sw	v0,44(s8)
800105ec:	8fc2002c 	lw	v0,44(s8)
800105f0:	24420014 	addiu	v0,v0,20
800105f4:	afc20030 	sw	v0,48(s8)
800105f8:	8fc30038 	lw	v1,56(s8)
800105fc:	3c02000f 	lui	v0,0xf
80010600:	3442ffff 	ori	v0,v0,0xffff
80010604:	00621024 	and	v0,v1,v0
80010608:	afc20044 	sw	v0,68(s8)
8001060c:	8fc30038 	lw	v1,56(s8)
80010610:	3c027fff 	lui	v0,0x7fff
80010614:	3442ffff 	ori	v0,v0,0xffff
80010618:	00621024 	and	v0,v1,v0
8001061c:	afc20038 	sw	v0,56(s8)
80010620:	8fc20038 	lw	v0,56(s8)
80010624:	00021502 	srl	v0,v0,0x14
80010628:	afc20034 	sw	v0,52(s8)
8001062c:	8fc20034 	lw	v0,52(s8)
80010630:	10400005 	beqz	v0,80010648 <__d2b+0xd8>
80010634:	00000000 	nop
80010638:	8fc30044 	lw	v1,68(s8)
8001063c:	3c020010 	lui	v0,0x10
80010640:	00621025 	or	v0,v1,v0
80010644:	afc20044 	sw	v0,68(s8)
80010648:	8fc2003c 	lw	v0,60(s8)
8001064c:	1040002f 	beqz	v0,8001070c <__d2b+0x19c>
80010650:	00000000 	nop
80010654:	8fc2003c 	lw	v0,60(s8)
80010658:	afc20040 	sw	v0,64(s8)
8001065c:	27c20040 	addiu	v0,s8,64
80010660:	00402025 	move	a0,v0
80010664:	0c003cff 	jal	8000f3fc <__lo0bits>
80010668:	00000000 	nop
8001066c:	afc20024 	sw	v0,36(s8)
80010670:	8fc20024 	lw	v0,36(s8)
80010674:	10400010 	beqz	v0,800106b8 <__d2b+0x148>
80010678:	00000000 	nop
8001067c:	8fc30044 	lw	v1,68(s8)
80010680:	24040020 	li	a0,32
80010684:	8fc20024 	lw	v0,36(s8)
80010688:	00821023 	subu	v0,a0,v0
8001068c:	00431804 	sllv	v1,v1,v0
80010690:	8fc20040 	lw	v0,64(s8)
80010694:	00621825 	or	v1,v1,v0
80010698:	8fc20030 	lw	v0,48(s8)
8001069c:	ac430000 	sw	v1,0(v0)
800106a0:	8fc30044 	lw	v1,68(s8)
800106a4:	8fc20024 	lw	v0,36(s8)
800106a8:	00431006 	srlv	v0,v1,v0
800106ac:	afc20044 	sw	v0,68(s8)
800106b0:	10000004 	b	800106c4 <__d2b+0x154>
800106b4:	00000000 	nop
800106b8:	8fc30040 	lw	v1,64(s8)
800106bc:	8fc20030 	lw	v0,48(s8)
800106c0:	ac430000 	sw	v1,0(v0)
800106c4:	8fc20030 	lw	v0,48(s8)
800106c8:	24420004 	addiu	v0,v0,4
800106cc:	8fc30044 	lw	v1,68(s8)
800106d0:	ac430000 	sw	v1,0(v0)
800106d4:	8c420000 	lw	v0,0(v0)
800106d8:	10400004 	beqz	v0,800106ec <__d2b+0x17c>
800106dc:	00000000 	nop
800106e0:	24020002 	li	v0,2
800106e4:	10000002 	b	800106f0 <__d2b+0x180>
800106e8:	00000000 	nop
800106ec:	24020001 	li	v0,1
800106f0:	8fc3002c 	lw	v1,44(s8)
800106f4:	ac620010 	sw	v0,16(v1)
800106f8:	8fc2002c 	lw	v0,44(s8)
800106fc:	8c420010 	lw	v0,16(v0)
80010700:	afc20020 	sw	v0,32(s8)
80010704:	10000012 	b	80010750 <__d2b+0x1e0>
80010708:	00000000 	nop
8001070c:	27c20044 	addiu	v0,s8,68
80010710:	00402025 	move	a0,v0
80010714:	0c003cff 	jal	8000f3fc <__lo0bits>
80010718:	00000000 	nop
8001071c:	afc20024 	sw	v0,36(s8)
80010720:	8fc30044 	lw	v1,68(s8)
80010724:	8fc20030 	lw	v0,48(s8)
80010728:	ac430000 	sw	v1,0(v0)
8001072c:	8fc2002c 	lw	v0,44(s8)
80010730:	24030001 	li	v1,1
80010734:	ac430010 	sw	v1,16(v0)
80010738:	8fc2002c 	lw	v0,44(s8)
8001073c:	8c420010 	lw	v0,16(v0)
80010740:	afc20020 	sw	v0,32(s8)
80010744:	8fc20024 	lw	v0,36(s8)
80010748:	24420020 	addiu	v0,v0,32
8001074c:	afc20024 	sw	v0,36(s8)
80010750:	8fc20034 	lw	v0,52(s8)
80010754:	1040000e 	beqz	v0,80010790 <__d2b+0x220>
80010758:	00000000 	nop
8001075c:	8fc20034 	lw	v0,52(s8)
80010760:	2443fbcd 	addiu	v1,v0,-1075
80010764:	8fc20024 	lw	v0,36(s8)
80010768:	00621821 	addu	v1,v1,v0
8001076c:	8fc20070 	lw	v0,112(s8)
80010770:	ac430000 	sw	v1,0(v0)
80010774:	24030035 	li	v1,53
80010778:	8fc20024 	lw	v0,36(s8)
8001077c:	00621823 	subu	v1,v1,v0
80010780:	8fc20078 	lw	v0,120(s8)
80010784:	ac430000 	sw	v1,0(v0)
80010788:	10000017 	b	800107e8 <__d2b+0x278>
8001078c:	00000000 	nop
80010790:	8fc20034 	lw	v0,52(s8)
80010794:	2443fbce 	addiu	v1,v0,-1074
80010798:	8fc20024 	lw	v0,36(s8)
8001079c:	00621821 	addu	v1,v1,v0
800107a0:	8fc20070 	lw	v0,112(s8)
800107a4:	ac430000 	sw	v1,0(v0)
800107a8:	8fc20020 	lw	v0,32(s8)
800107ac:	00028140 	sll	s0,v0,0x5
800107b0:	8fc30020 	lw	v1,32(s8)
800107b4:	3c023fff 	lui	v0,0x3fff
800107b8:	3442ffff 	ori	v0,v0,0xffff
800107bc:	00621021 	addu	v0,v1,v0
800107c0:	00021080 	sll	v0,v0,0x2
800107c4:	8fc30030 	lw	v1,48(s8)
800107c8:	00621021 	addu	v0,v1,v0
800107cc:	8c420000 	lw	v0,0(v0)
800107d0:	00402025 	move	a0,v0
800107d4:	0c003ccf 	jal	8000f33c <__hi0bits>
800107d8:	00000000 	nop
800107dc:	02021823 	subu	v1,s0,v0
800107e0:	8fc20078 	lw	v0,120(s8)
800107e4:	ac430000 	sw	v1,0(v0)
800107e8:	8fc2002c 	lw	v0,44(s8)
800107ec:	03c0e825 	move	sp,s8
800107f0:	dfbf0058 	ld	ra,88(sp)
800107f4:	dfbe0050 	ld	s8,80(sp)
800107f8:	dfb00048 	ld	s0,72(sp)
800107fc:	27bd0060 	addiu	sp,sp,96
80010800:	03e00008 	jr	ra
80010804:	00000000 	nop

80010808 <__ratio>:
80010808:	27bdffb0 	addiu	sp,sp,-80
8001080c:	ffbf0048 	sd	ra,72(sp)
80010810:	ffbe0040 	sd	s8,64(sp)
80010814:	03a0f025 	move	s8,sp
80010818:	afc40050 	sw	a0,80(s8)
8001081c:	afc50058 	sw	a1,88(s8)
80010820:	27c20038 	addiu	v0,s8,56
80010824:	00402825 	move	a1,v0
80010828:	8fc40050 	lw	a0,80(s8)
8001082c:	0c0040d0 	jal	80010340 <__b2d>
80010830:	00000000 	nop
80010834:	f7c00028 	sdc1	$f0,40(s8)
80010838:	27c2003c 	addiu	v0,s8,60
8001083c:	00402825 	move	a1,v0
80010840:	8fc40058 	lw	a0,88(s8)
80010844:	0c0040d0 	jal	80010340 <__b2d>
80010848:	00000000 	nop
8001084c:	f7c00030 	sdc1	$f0,48(s8)
80010850:	8fc30038 	lw	v1,56(s8)
80010854:	8fc2003c 	lw	v0,60(s8)
80010858:	00621823 	subu	v1,v1,v0
8001085c:	8fc20050 	lw	v0,80(s8)
80010860:	8c440010 	lw	a0,16(v0)
80010864:	8fc20058 	lw	v0,88(s8)
80010868:	8c420010 	lw	v0,16(v0)
8001086c:	00821023 	subu	v0,a0,v0
80010870:	00021140 	sll	v0,v0,0x5
80010874:	00621021 	addu	v0,v1,v0
80010878:	afc20020 	sw	v0,32(s8)
8001087c:	8fc20020 	lw	v0,32(s8)
80010880:	18400008 	blez	v0,800108a4 <__ratio+0x9c>
80010884:	00000000 	nop
80010888:	8fc30028 	lw	v1,40(s8)
8001088c:	8fc20020 	lw	v0,32(s8)
80010890:	00021500 	sll	v0,v0,0x14
80010894:	00621021 	addu	v0,v1,v0
80010898:	afc20028 	sw	v0,40(s8)
8001089c:	10000009 	b	800108c4 <__ratio+0xbc>
800108a0:	00000000 	nop
800108a4:	8fc20020 	lw	v0,32(s8)
800108a8:	00021023 	negu	v0,v0
800108ac:	afc20020 	sw	v0,32(s8)
800108b0:	8fc30030 	lw	v1,48(s8)
800108b4:	8fc20020 	lw	v0,32(s8)
800108b8:	00021500 	sll	v0,v0,0x14
800108bc:	00621021 	addu	v0,v1,v0
800108c0:	afc20030 	sw	v0,48(s8)
800108c4:	d7c10028 	ldc1	$f1,40(s8)
800108c8:	d7c00030 	ldc1	$f0,48(s8)
800108cc:	46200803 	div.d	$f0,$f1,$f0
800108d0:	03c0e825 	move	sp,s8
800108d4:	dfbf0048 	ld	ra,72(sp)
800108d8:	dfbe0040 	ld	s8,64(sp)
800108dc:	27bd0050 	addiu	sp,sp,80
800108e0:	03e00008 	jr	ra
800108e4:	00000000 	nop

800108e8 <_mprec_log10>:
800108e8:	27bdfff0 	addiu	sp,sp,-16
800108ec:	ffbe0008 	sd	s8,8(sp)
800108f0:	03a0f025 	move	s8,sp
800108f4:	afc40010 	sw	a0,16(s8)
800108f8:	d7808098 	ldc1	$f0,-32616(gp)
800108fc:	f7c00000 	sdc1	$f0,0(s8)
80010900:	8fc20010 	lw	v0,16(s8)
80010904:	28420018 	slti	v0,v0,24
80010908:	10400010 	beqz	v0,8001094c <_mprec_log10+0x64>
8001090c:	00000000 	nop
80010910:	3c028002 	lui	v0,0x8002
80010914:	8fc30010 	lw	v1,16(s8)
80010918:	000318c0 	sll	v1,v1,0x3
8001091c:	2442a830 	addiu	v0,v0,-22480
80010920:	00621021 	addu	v0,v1,v0
80010924:	d4400000 	ldc1	$f0,0(v0)
80010928:	1000000c 	b	8001095c <_mprec_log10+0x74>
8001092c:	00000000 	nop
80010930:	d7c10000 	ldc1	$f1,0(s8)
80010934:	d78080a0 	ldc1	$f0,-32608(gp)
80010938:	46200802 	mul.d	$f0,$f1,$f0
8001093c:	f7c00000 	sdc1	$f0,0(s8)
80010940:	8fc20010 	lw	v0,16(s8)
80010944:	2442ffff 	addiu	v0,v0,-1
80010948:	afc20010 	sw	v0,16(s8)
8001094c:	8fc20010 	lw	v0,16(s8)
80010950:	1c40fff7 	bgtz	v0,80010930 <_mprec_log10+0x48>
80010954:	00000000 	nop
80010958:	d7c00000 	ldc1	$f0,0(s8)
8001095c:	03c0e825 	move	sp,s8
80010960:	dfbe0008 	ld	s8,8(sp)
80010964:	27bd0010 	addiu	sp,sp,16
80010968:	03e00008 	jr	ra
8001096c:	00000000 	nop

80010970 <__copybits>:
80010970:	27bdffe8 	addiu	sp,sp,-24
80010974:	ffbe0010 	sd	s8,16(sp)
80010978:	03a0f025 	move	s8,sp
8001097c:	afc40018 	sw	a0,24(s8)
80010980:	afc50020 	sw	a1,32(s8)
80010984:	afc60028 	sw	a2,40(s8)
80010988:	8fc20020 	lw	v0,32(s8)
8001098c:	2442ffff 	addiu	v0,v0,-1
80010990:	00021143 	sra	v0,v0,0x5
80010994:	24420001 	addiu	v0,v0,1
80010998:	00021080 	sll	v0,v0,0x2
8001099c:	8fc30018 	lw	v1,24(s8)
800109a0:	00621021 	addu	v0,v1,v0
800109a4:	afc20004 	sw	v0,4(s8)
800109a8:	8fc20028 	lw	v0,40(s8)
800109ac:	24420014 	addiu	v0,v0,20
800109b0:	afc20000 	sw	v0,0(s8)
800109b4:	8fc20028 	lw	v0,40(s8)
800109b8:	8c420010 	lw	v0,16(v0)
800109bc:	00021080 	sll	v0,v0,0x2
800109c0:	8fc30000 	lw	v1,0(s8)
800109c4:	00621021 	addu	v0,v1,v0
800109c8:	afc20008 	sw	v0,8(s8)
800109cc:	10000009 	b	800109f4 <__copybits+0x84>
800109d0:	00000000 	nop
800109d4:	8fc30000 	lw	v1,0(s8)
800109d8:	24620004 	addiu	v0,v1,4
800109dc:	afc20000 	sw	v0,0(s8)
800109e0:	8fc20018 	lw	v0,24(s8)
800109e4:	24440004 	addiu	a0,v0,4
800109e8:	afc40018 	sw	a0,24(s8)
800109ec:	8c630000 	lw	v1,0(v1)
800109f0:	ac430000 	sw	v1,0(v0)
800109f4:	8fc30000 	lw	v1,0(s8)
800109f8:	8fc20008 	lw	v0,8(s8)
800109fc:	0062102b 	sltu	v0,v1,v0
80010a00:	1440fff4 	bnez	v0,800109d4 <__copybits+0x64>
80010a04:	00000000 	nop
80010a08:	10000005 	b	80010a20 <__copybits+0xb0>
80010a0c:	00000000 	nop
80010a10:	8fc20018 	lw	v0,24(s8)
80010a14:	24430004 	addiu	v1,v0,4
80010a18:	afc30018 	sw	v1,24(s8)
80010a1c:	ac400000 	sw	zero,0(v0)
80010a20:	8fc30018 	lw	v1,24(s8)
80010a24:	8fc20004 	lw	v0,4(s8)
80010a28:	0062102b 	sltu	v0,v1,v0
80010a2c:	1440fff8 	bnez	v0,80010a10 <__copybits+0xa0>
80010a30:	00000000 	nop
	...
80010a3c:	03c0e825 	move	sp,s8
80010a40:	dfbe0010 	ld	s8,16(sp)
80010a44:	27bd0018 	addiu	sp,sp,24
80010a48:	03e00008 	jr	ra
80010a4c:	00000000 	nop

80010a50 <__any_on>:
80010a50:	27bdffe0 	addiu	sp,sp,-32
80010a54:	ffbe0018 	sd	s8,24(sp)
80010a58:	03a0f025 	move	s8,sp
80010a5c:	afc40020 	sw	a0,32(s8)
80010a60:	afc50028 	sw	a1,40(s8)
80010a64:	8fc20020 	lw	v0,32(s8)
80010a68:	24420014 	addiu	v0,v0,20
80010a6c:	afc20004 	sw	v0,4(s8)
80010a70:	8fc20020 	lw	v0,32(s8)
80010a74:	8c420010 	lw	v0,16(v0)
80010a78:	afc20008 	sw	v0,8(s8)
80010a7c:	8fc20028 	lw	v0,40(s8)
80010a80:	00021143 	sra	v0,v0,0x5
80010a84:	afc20000 	sw	v0,0(s8)
80010a88:	8fc30000 	lw	v1,0(s8)
80010a8c:	8fc20008 	lw	v0,8(s8)
80010a90:	0043102a 	slt	v0,v0,v1
80010a94:	10400005 	beqz	v0,80010aac <__any_on+0x5c>
80010a98:	00000000 	nop
80010a9c:	8fc20008 	lw	v0,8(s8)
80010aa0:	afc20000 	sw	v0,0(s8)
80010aa4:	10000023 	b	80010b34 <__any_on+0xe4>
80010aa8:	00000000 	nop
80010aac:	8fc30000 	lw	v1,0(s8)
80010ab0:	8fc20008 	lw	v0,8(s8)
80010ab4:	0062102a 	slt	v0,v1,v0
80010ab8:	1040001e 	beqz	v0,80010b34 <__any_on+0xe4>
80010abc:	00000000 	nop
80010ac0:	8fc20028 	lw	v0,40(s8)
80010ac4:	3042001f 	andi	v0,v0,0x1f
80010ac8:	afc20028 	sw	v0,40(s8)
80010acc:	8fc20028 	lw	v0,40(s8)
80010ad0:	10400018 	beqz	v0,80010b34 <__any_on+0xe4>
80010ad4:	00000000 	nop
80010ad8:	8fc20000 	lw	v0,0(s8)
80010adc:	00021080 	sll	v0,v0,0x2
80010ae0:	8fc30004 	lw	v1,4(s8)
80010ae4:	00621021 	addu	v0,v1,v0
80010ae8:	8c420000 	lw	v0,0(v0)
80010aec:	afc2000c 	sw	v0,12(s8)
80010af0:	8fc2000c 	lw	v0,12(s8)
80010af4:	afc20010 	sw	v0,16(s8)
80010af8:	8fc30010 	lw	v1,16(s8)
80010afc:	8fc20028 	lw	v0,40(s8)
80010b00:	00431006 	srlv	v0,v1,v0
80010b04:	afc20010 	sw	v0,16(s8)
80010b08:	8fc30010 	lw	v1,16(s8)
80010b0c:	8fc20028 	lw	v0,40(s8)
80010b10:	00431004 	sllv	v0,v1,v0
80010b14:	afc20010 	sw	v0,16(s8)
80010b18:	8fc30010 	lw	v1,16(s8)
80010b1c:	8fc2000c 	lw	v0,12(s8)
80010b20:	10620004 	beq	v1,v0,80010b34 <__any_on+0xe4>
80010b24:	00000000 	nop
80010b28:	24020001 	li	v0,1
80010b2c:	1000001a 	b	80010b98 <__any_on+0x148>
80010b30:	00000000 	nop
80010b34:	8fc20004 	lw	v0,4(s8)
80010b38:	afc20014 	sw	v0,20(s8)
80010b3c:	8fc20000 	lw	v0,0(s8)
80010b40:	00021080 	sll	v0,v0,0x2
80010b44:	8fc30004 	lw	v1,4(s8)
80010b48:	00621021 	addu	v0,v1,v0
80010b4c:	afc20004 	sw	v0,4(s8)
80010b50:	1000000b 	b	80010b80 <__any_on+0x130>
80010b54:	00000000 	nop
80010b58:	8fc20004 	lw	v0,4(s8)
80010b5c:	2442fffc 	addiu	v0,v0,-4
80010b60:	afc20004 	sw	v0,4(s8)
80010b64:	8fc20004 	lw	v0,4(s8)
80010b68:	8c420000 	lw	v0,0(v0)
80010b6c:	10400004 	beqz	v0,80010b80 <__any_on+0x130>
80010b70:	00000000 	nop
80010b74:	24020001 	li	v0,1
80010b78:	10000007 	b	80010b98 <__any_on+0x148>
80010b7c:	00000000 	nop
80010b80:	8fc30004 	lw	v1,4(s8)
80010b84:	8fc20014 	lw	v0,20(s8)
80010b88:	0043102b 	sltu	v0,v0,v1
80010b8c:	1440fff2 	bnez	v0,80010b58 <__any_on+0x108>
80010b90:	00000000 	nop
80010b94:	00001025 	move	v0,zero
80010b98:	03c0e825 	move	sp,s8
80010b9c:	dfbe0018 	ld	s8,24(sp)
80010ba0:	27bd0020 	addiu	sp,sp,32
80010ba4:	03e00008 	jr	ra
80010ba8:	00000000 	nop

80010bac <_read_r>:
80010bac:	27bdffc8 	addiu	sp,sp,-56
80010bb0:	ffbf0030 	sd	ra,48(sp)
80010bb4:	ffbe0028 	sd	s8,40(sp)
80010bb8:	03a0f025 	move	s8,sp
80010bbc:	afc40038 	sw	a0,56(s8)
80010bc0:	afc50040 	sw	a1,64(s8)
80010bc4:	afc60048 	sw	a2,72(s8)
80010bc8:	afc70050 	sw	a3,80(s8)
80010bcc:	af808334 	sw	zero,-31948(gp)
80010bd0:	8fc60050 	lw	a2,80(s8)
80010bd4:	8fc50048 	lw	a1,72(s8)
80010bd8:	8fc40040 	lw	a0,64(s8)
80010bdc:	0c0067ea 	jal	80019fa8 <read>
80010be0:	00000000 	nop
80010be4:	afc20020 	sw	v0,32(s8)
80010be8:	8fc30020 	lw	v1,32(s8)
80010bec:	2402ffff 	li	v0,-1
80010bf0:	14620007 	bne	v1,v0,80010c10 <_read_r+0x64>
80010bf4:	00000000 	nop
80010bf8:	8f828334 	lw	v0,-31948(gp)
80010bfc:	10400004 	beqz	v0,80010c10 <_read_r+0x64>
80010c00:	00000000 	nop
80010c04:	8f838334 	lw	v1,-31948(gp)
80010c08:	8fc20038 	lw	v0,56(s8)
80010c0c:	ac430000 	sw	v1,0(v0)
80010c10:	8fc20020 	lw	v0,32(s8)
80010c14:	03c0e825 	move	sp,s8
80010c18:	dfbf0030 	ld	ra,48(sp)
80010c1c:	dfbe0028 	ld	s8,40(sp)
80010c20:	27bd0038 	addiu	sp,sp,56
80010c24:	03e00008 	jr	ra
80010c28:	00000000 	nop

80010c2c <_realloc_r>:
80010c2c:	27bdff68 	addiu	sp,sp,-152
80010c30:	ffbf0090 	sd	ra,144(sp)
80010c34:	ffbe0088 	sd	s8,136(sp)
80010c38:	03a0f025 	move	s8,sp
80010c3c:	afc40098 	sw	a0,152(s8)
80010c40:	afc500a0 	sw	a1,160(s8)
80010c44:	afc600a8 	sw	a2,168(s8)
80010c48:	8fc200a0 	lw	v0,160(s8)
80010c4c:	14400007 	bnez	v0,80010c6c <_realloc_r+0x40>
80010c50:	00000000 	nop
80010c54:	8fc500a8 	lw	a1,168(s8)
80010c58:	8fc40098 	lw	a0,152(s8)
80010c5c:	0c001296 	jal	80004a58 <_malloc_r>
80010c60:	00000000 	nop
80010c64:	10000333 	b	80011934 <_realloc_r+0xd08>
80010c68:	00000000 	nop
80010c6c:	8fc40098 	lw	a0,152(s8)
80010c70:	0c001ba7 	jal	80006e9c <__malloc_lock>
80010c74:	00000000 	nop
80010c78:	8fc200a0 	lw	v0,160(s8)
80010c7c:	2442fff8 	addiu	v0,v0,-8
80010c80:	afc20050 	sw	v0,80(s8)
80010c84:	8fc20050 	lw	v0,80(s8)
80010c88:	afc20020 	sw	v0,32(s8)
80010c8c:	8fc20050 	lw	v0,80(s8)
80010c90:	8c430004 	lw	v1,4(v0)
80010c94:	2402fffc 	li	v0,-4
80010c98:	00621024 	and	v0,v1,v0
80010c9c:	afc20054 	sw	v0,84(s8)
80010ca0:	8fc20054 	lw	v0,84(s8)
80010ca4:	afc20024 	sw	v0,36(s8)
80010ca8:	8fc200a8 	lw	v0,168(s8)
80010cac:	2442000b 	addiu	v0,v0,11
80010cb0:	2c420017 	sltiu	v0,v0,23
80010cb4:	14400007 	bnez	v0,80010cd4 <_realloc_r+0xa8>
80010cb8:	00000000 	nop
80010cbc:	8fc200a8 	lw	v0,168(s8)
80010cc0:	2443000b 	addiu	v1,v0,11
80010cc4:	2402fff8 	li	v0,-8
80010cc8:	00621024 	and	v0,v1,v0
80010ccc:	10000002 	b	80010cd8 <_realloc_r+0xac>
80010cd0:	00000000 	nop
80010cd4:	24020010 	li	v0,16
80010cd8:	afc20058 	sw	v0,88(s8)
80010cdc:	8fc20058 	lw	v0,88(s8)
80010ce0:	04400006 	bltz	v0,80010cfc <_realloc_r+0xd0>
80010ce4:	00000000 	nop
80010ce8:	8fc30058 	lw	v1,88(s8)
80010cec:	8fc200a8 	lw	v0,168(s8)
80010cf0:	0062102b 	sltu	v0,v1,v0
80010cf4:	10400007 	beqz	v0,80010d14 <_realloc_r+0xe8>
80010cf8:	00000000 	nop
80010cfc:	8fc20098 	lw	v0,152(s8)
80010d00:	2403000c 	li	v1,12
80010d04:	ac430000 	sw	v1,0(v0)
80010d08:	00001025 	move	v0,zero
80010d0c:	10000309 	b	80011934 <_realloc_r+0xd08>
80010d10:	00000000 	nop
80010d14:	8fc30054 	lw	v1,84(s8)
80010d18:	8fc20058 	lw	v0,88(s8)
80010d1c:	0062102a 	slt	v0,v1,v0
80010d20:	104002c6 	beqz	v0,8001183c <_realloc_r+0xc10>
80010d24:	00000000 	nop
80010d28:	8fc30050 	lw	v1,80(s8)
80010d2c:	8fc20054 	lw	v0,84(s8)
80010d30:	00621021 	addu	v0,v1,v0
80010d34:	afc20028 	sw	v0,40(s8)
80010d38:	3c028002 	lui	v0,0x8002
80010d3c:	2442ce98 	addiu	v0,v0,-12648
80010d40:	8c420008 	lw	v0,8(v0)
80010d44:	8fc30028 	lw	v1,40(s8)
80010d48:	1062000b 	beq	v1,v0,80010d78 <_realloc_r+0x14c>
80010d4c:	00000000 	nop
80010d50:	8fc20028 	lw	v0,40(s8)
80010d54:	8c430004 	lw	v1,4(v0)
80010d58:	2402fffe 	li	v0,-2
80010d5c:	00621024 	and	v0,v1,v0
80010d60:	8fc30028 	lw	v1,40(s8)
80010d64:	00621021 	addu	v0,v1,v0
80010d68:	8c420004 	lw	v0,4(v0)
80010d6c:	30420001 	andi	v0,v0,0x1
80010d70:	1440004f 	bnez	v0,80010eb0 <_realloc_r+0x284>
80010d74:	00000000 	nop
80010d78:	8fc20028 	lw	v0,40(s8)
80010d7c:	8c430004 	lw	v1,4(v0)
80010d80:	2402fffc 	li	v0,-4
80010d84:	00621024 	and	v0,v1,v0
80010d88:	afc2002c 	sw	v0,44(s8)
80010d8c:	3c028002 	lui	v0,0x8002
80010d90:	2442ce98 	addiu	v0,v0,-12648
80010d94:	8c420008 	lw	v0,8(v0)
80010d98:	8fc30028 	lw	v1,40(s8)
80010d9c:	1462002a 	bne	v1,v0,80010e48 <_realloc_r+0x21c>
80010da0:	00000000 	nop
80010da4:	8fc3002c 	lw	v1,44(s8)
80010da8:	8fc20024 	lw	v0,36(s8)
80010dac:	00621021 	addu	v0,v1,v0
80010db0:	00401825 	move	v1,v0
80010db4:	8fc20058 	lw	v0,88(s8)
80010db8:	24420010 	addiu	v0,v0,16
80010dbc:	0062102a 	slt	v0,v1,v0
80010dc0:	1440003f 	bnez	v0,80010ec0 <_realloc_r+0x294>
80010dc4:	00000000 	nop
80010dc8:	8fc30024 	lw	v1,36(s8)
80010dcc:	8fc2002c 	lw	v0,44(s8)
80010dd0:	00621021 	addu	v0,v1,v0
80010dd4:	afc20024 	sw	v0,36(s8)
80010dd8:	3c028002 	lui	v0,0x8002
80010ddc:	2442ce98 	addiu	v0,v0,-12648
80010de0:	8fc40050 	lw	a0,80(s8)
80010de4:	8fc30058 	lw	v1,88(s8)
80010de8:	00831821 	addu	v1,a0,v1
80010dec:	ac430008 	sw	v1,8(v0)
80010df0:	8fc30024 	lw	v1,36(s8)
80010df4:	8fc20058 	lw	v0,88(s8)
80010df8:	00621823 	subu	v1,v1,v0
80010dfc:	3c028002 	lui	v0,0x8002
80010e00:	2442ce98 	addiu	v0,v0,-12648
80010e04:	8c420008 	lw	v0,8(v0)
80010e08:	34630001 	ori	v1,v1,0x1
80010e0c:	ac430004 	sw	v1,4(v0)
80010e10:	8fc20050 	lw	v0,80(s8)
80010e14:	8c420004 	lw	v0,4(v0)
80010e18:	30430001 	andi	v1,v0,0x1
80010e1c:	8fc20058 	lw	v0,88(s8)
80010e20:	00621825 	or	v1,v1,v0
80010e24:	8fc20050 	lw	v0,80(s8)
80010e28:	ac430004 	sw	v1,4(v0)
80010e2c:	8fc40098 	lw	a0,152(s8)
80010e30:	0c001bb1 	jal	80006ec4 <__malloc_unlock>
80010e34:	00000000 	nop
80010e38:	8fc20050 	lw	v0,80(s8)
80010e3c:	24420008 	addiu	v0,v0,8
80010e40:	100002bc 	b	80011934 <_realloc_r+0xd08>
80010e44:	00000000 	nop
80010e48:	8fc3002c 	lw	v1,44(s8)
80010e4c:	8fc20024 	lw	v0,36(s8)
80010e50:	00621021 	addu	v0,v1,v0
80010e54:	00401825 	move	v1,v0
80010e58:	8fc20058 	lw	v0,88(s8)
80010e5c:	0062102a 	slt	v0,v1,v0
80010e60:	14400017 	bnez	v0,80010ec0 <_realloc_r+0x294>
80010e64:	00000000 	nop
80010e68:	8fc20028 	lw	v0,40(s8)
80010e6c:	8c42000c 	lw	v0,12(v0)
80010e70:	afc2005c 	sw	v0,92(s8)
80010e74:	8fc20028 	lw	v0,40(s8)
80010e78:	8c420008 	lw	v0,8(v0)
80010e7c:	afc20060 	sw	v0,96(s8)
80010e80:	8fc20060 	lw	v0,96(s8)
80010e84:	8fc3005c 	lw	v1,92(s8)
80010e88:	ac43000c 	sw	v1,12(v0)
80010e8c:	8fc2005c 	lw	v0,92(s8)
80010e90:	8fc30060 	lw	v1,96(s8)
80010e94:	ac430008 	sw	v1,8(v0)
80010e98:	8fc30024 	lw	v1,36(s8)
80010e9c:	8fc2002c 	lw	v0,44(s8)
80010ea0:	00621021 	addu	v0,v1,v0
80010ea4:	afc20024 	sw	v0,36(s8)
80010ea8:	10000265 	b	80011840 <_realloc_r+0xc14>
80010eac:	00000000 	nop
80010eb0:	afc00028 	sw	zero,40(s8)
80010eb4:	afc0002c 	sw	zero,44(s8)
80010eb8:	10000002 	b	80010ec4 <_realloc_r+0x298>
80010ebc:	00000000 	nop
80010ec0:	00000000 	nop
80010ec4:	8fc20050 	lw	v0,80(s8)
80010ec8:	8c420004 	lw	v0,4(v0)
80010ecc:	30420001 	andi	v0,v0,0x1
80010ed0:	144001c9 	bnez	v0,800115f8 <_realloc_r+0x9cc>
80010ed4:	00000000 	nop
80010ed8:	8fc20050 	lw	v0,80(s8)
80010edc:	8c420000 	lw	v0,0(v0)
80010ee0:	00021023 	negu	v0,v0
80010ee4:	8fc30050 	lw	v1,80(s8)
80010ee8:	00621021 	addu	v0,v1,v0
80010eec:	afc20064 	sw	v0,100(s8)
80010ef0:	8fc20064 	lw	v0,100(s8)
80010ef4:	8c430004 	lw	v1,4(v0)
80010ef8:	2402fffc 	li	v0,-4
80010efc:	00621024 	and	v0,v1,v0
80010f00:	afc20068 	sw	v0,104(s8)
80010f04:	8fc20028 	lw	v0,40(s8)
80010f08:	10400137 	beqz	v0,800113e8 <_realloc_r+0x7bc>
80010f0c:	00000000 	nop
80010f10:	3c028002 	lui	v0,0x8002
80010f14:	2442ce98 	addiu	v0,v0,-12648
80010f18:	8c420008 	lw	v0,8(v0)
80010f1c:	8fc30028 	lw	v1,40(s8)
80010f20:	146200a0 	bne	v1,v0,800111a4 <_realloc_r+0x578>
80010f24:	00000000 	nop
80010f28:	8fc3002c 	lw	v1,44(s8)
80010f2c:	8fc20068 	lw	v0,104(s8)
80010f30:	00621821 	addu	v1,v1,v0
80010f34:	8fc20024 	lw	v0,36(s8)
80010f38:	00621021 	addu	v0,v1,v0
80010f3c:	00401825 	move	v1,v0
80010f40:	8fc20058 	lw	v0,88(s8)
80010f44:	24420010 	addiu	v0,v0,16
80010f48:	0062102a 	slt	v0,v1,v0
80010f4c:	14400126 	bnez	v0,800113e8 <_realloc_r+0x7bc>
80010f50:	00000000 	nop
80010f54:	8fc20064 	lw	v0,100(s8)
80010f58:	8c42000c 	lw	v0,12(v0)
80010f5c:	afc2005c 	sw	v0,92(s8)
80010f60:	8fc20064 	lw	v0,100(s8)
80010f64:	8c420008 	lw	v0,8(v0)
80010f68:	afc20060 	sw	v0,96(s8)
80010f6c:	8fc20060 	lw	v0,96(s8)
80010f70:	8fc3005c 	lw	v1,92(s8)
80010f74:	ac43000c 	sw	v1,12(v0)
80010f78:	8fc2005c 	lw	v0,92(s8)
80010f7c:	8fc30060 	lw	v1,96(s8)
80010f80:	ac430008 	sw	v1,8(v0)
80010f84:	8fc20064 	lw	v0,100(s8)
80010f88:	afc20020 	sw	v0,32(s8)
80010f8c:	8fc30068 	lw	v1,104(s8)
80010f90:	8fc2002c 	lw	v0,44(s8)
80010f94:	00621021 	addu	v0,v1,v0
80010f98:	8fc30024 	lw	v1,36(s8)
80010f9c:	00621021 	addu	v0,v1,v0
80010fa0:	afc20024 	sw	v0,36(s8)
80010fa4:	8fc20020 	lw	v0,32(s8)
80010fa8:	24420008 	addiu	v0,v0,8
80010fac:	afc2006c 	sw	v0,108(s8)
80010fb0:	8fc20054 	lw	v0,84(s8)
80010fb4:	2442fffc 	addiu	v0,v0,-4
80010fb8:	afc20074 	sw	v0,116(s8)
80010fbc:	8fc20074 	lw	v0,116(s8)
80010fc0:	2c420025 	sltiu	v0,v0,37
80010fc4:	10400057 	beqz	v0,80011124 <_realloc_r+0x4f8>
80010fc8:	00000000 	nop
80010fcc:	8fc200a0 	lw	v0,160(s8)
80010fd0:	afc20030 	sw	v0,48(s8)
80010fd4:	8fc2006c 	lw	v0,108(s8)
80010fd8:	afc20034 	sw	v0,52(s8)
80010fdc:	8fc20074 	lw	v0,116(s8)
80010fe0:	2c420014 	sltiu	v0,v0,20
80010fe4:	14400039 	bnez	v0,800110cc <_realloc_r+0x4a0>
80010fe8:	00000000 	nop
80010fec:	8fc30030 	lw	v1,48(s8)
80010ff0:	24620004 	addiu	v0,v1,4
80010ff4:	afc20030 	sw	v0,48(s8)
80010ff8:	8fc20034 	lw	v0,52(s8)
80010ffc:	24440004 	addiu	a0,v0,4
80011000:	afc40034 	sw	a0,52(s8)
80011004:	8c630000 	lw	v1,0(v1)
80011008:	ac430000 	sw	v1,0(v0)
8001100c:	8fc30030 	lw	v1,48(s8)
80011010:	24620004 	addiu	v0,v1,4
80011014:	afc20030 	sw	v0,48(s8)
80011018:	8fc20034 	lw	v0,52(s8)
8001101c:	24440004 	addiu	a0,v0,4
80011020:	afc40034 	sw	a0,52(s8)
80011024:	8c630000 	lw	v1,0(v1)
80011028:	ac430000 	sw	v1,0(v0)
8001102c:	8fc20074 	lw	v0,116(s8)
80011030:	2c42001c 	sltiu	v0,v0,28
80011034:	14400025 	bnez	v0,800110cc <_realloc_r+0x4a0>
80011038:	00000000 	nop
8001103c:	8fc30030 	lw	v1,48(s8)
80011040:	24620004 	addiu	v0,v1,4
80011044:	afc20030 	sw	v0,48(s8)
80011048:	8fc20034 	lw	v0,52(s8)
8001104c:	24440004 	addiu	a0,v0,4
80011050:	afc40034 	sw	a0,52(s8)
80011054:	8c630000 	lw	v1,0(v1)
80011058:	ac430000 	sw	v1,0(v0)
8001105c:	8fc30030 	lw	v1,48(s8)
80011060:	24620004 	addiu	v0,v1,4
80011064:	afc20030 	sw	v0,48(s8)
80011068:	8fc20034 	lw	v0,52(s8)
8001106c:	24440004 	addiu	a0,v0,4
80011070:	afc40034 	sw	a0,52(s8)
80011074:	8c630000 	lw	v1,0(v1)
80011078:	ac430000 	sw	v1,0(v0)
8001107c:	8fc20074 	lw	v0,116(s8)
80011080:	2c420024 	sltiu	v0,v0,36
80011084:	14400011 	bnez	v0,800110cc <_realloc_r+0x4a0>
80011088:	00000000 	nop
8001108c:	8fc30030 	lw	v1,48(s8)
80011090:	24620004 	addiu	v0,v1,4
80011094:	afc20030 	sw	v0,48(s8)
80011098:	8fc20034 	lw	v0,52(s8)
8001109c:	24440004 	addiu	a0,v0,4
800110a0:	afc40034 	sw	a0,52(s8)
800110a4:	8c630000 	lw	v1,0(v1)
800110a8:	ac430000 	sw	v1,0(v0)
800110ac:	8fc30030 	lw	v1,48(s8)
800110b0:	24620004 	addiu	v0,v1,4
800110b4:	afc20030 	sw	v0,48(s8)
800110b8:	8fc20034 	lw	v0,52(s8)
800110bc:	24440004 	addiu	a0,v0,4
800110c0:	afc40034 	sw	a0,52(s8)
800110c4:	8c630000 	lw	v1,0(v1)
800110c8:	ac430000 	sw	v1,0(v0)
800110cc:	8fc30030 	lw	v1,48(s8)
800110d0:	24620004 	addiu	v0,v1,4
800110d4:	afc20030 	sw	v0,48(s8)
800110d8:	8fc20034 	lw	v0,52(s8)
800110dc:	24440004 	addiu	a0,v0,4
800110e0:	afc40034 	sw	a0,52(s8)
800110e4:	8c630000 	lw	v1,0(v1)
800110e8:	ac430000 	sw	v1,0(v0)
800110ec:	8fc30030 	lw	v1,48(s8)
800110f0:	24620004 	addiu	v0,v1,4
800110f4:	afc20030 	sw	v0,48(s8)
800110f8:	8fc20034 	lw	v0,52(s8)
800110fc:	24440004 	addiu	a0,v0,4
80011100:	afc40034 	sw	a0,52(s8)
80011104:	8c630000 	lw	v1,0(v1)
80011108:	ac430000 	sw	v1,0(v0)
8001110c:	8fc20030 	lw	v0,48(s8)
80011110:	8c430000 	lw	v1,0(v0)
80011114:	8fc20034 	lw	v0,52(s8)
80011118:	ac430000 	sw	v1,0(v0)
8001111c:	10000006 	b	80011138 <_realloc_r+0x50c>
80011120:	00000000 	nop
80011124:	8fc60074 	lw	a2,116(s8)
80011128:	8fc500a0 	lw	a1,160(s8)
8001112c:	8fc4006c 	lw	a0,108(s8)
80011130:	0c0016ab 	jal	80005aac <memmove>
80011134:	00000000 	nop
80011138:	3c028002 	lui	v0,0x8002
8001113c:	2442ce98 	addiu	v0,v0,-12648
80011140:	8fc40020 	lw	a0,32(s8)
80011144:	8fc30058 	lw	v1,88(s8)
80011148:	00831821 	addu	v1,a0,v1
8001114c:	ac430008 	sw	v1,8(v0)
80011150:	8fc30024 	lw	v1,36(s8)
80011154:	8fc20058 	lw	v0,88(s8)
80011158:	00621823 	subu	v1,v1,v0
8001115c:	3c028002 	lui	v0,0x8002
80011160:	2442ce98 	addiu	v0,v0,-12648
80011164:	8c420008 	lw	v0,8(v0)
80011168:	34630001 	ori	v1,v1,0x1
8001116c:	ac430004 	sw	v1,4(v0)
80011170:	8fc20020 	lw	v0,32(s8)
80011174:	8c420004 	lw	v0,4(v0)
80011178:	30430001 	andi	v1,v0,0x1
8001117c:	8fc20058 	lw	v0,88(s8)
80011180:	00621825 	or	v1,v1,v0
80011184:	8fc20020 	lw	v0,32(s8)
80011188:	ac430004 	sw	v1,4(v0)
8001118c:	8fc40098 	lw	a0,152(s8)
80011190:	0c001bb1 	jal	80006ec4 <__malloc_unlock>
80011194:	00000000 	nop
80011198:	8fc2006c 	lw	v0,108(s8)
8001119c:	100001e5 	b	80011934 <_realloc_r+0xd08>
800111a0:	00000000 	nop
800111a4:	8fc3002c 	lw	v1,44(s8)
800111a8:	8fc20068 	lw	v0,104(s8)
800111ac:	00621821 	addu	v1,v1,v0
800111b0:	8fc20024 	lw	v0,36(s8)
800111b4:	00621021 	addu	v0,v1,v0
800111b8:	00401825 	move	v1,v0
800111bc:	8fc20058 	lw	v0,88(s8)
800111c0:	0062102a 	slt	v0,v1,v0
800111c4:	14400088 	bnez	v0,800113e8 <_realloc_r+0x7bc>
800111c8:	00000000 	nop
800111cc:	8fc20028 	lw	v0,40(s8)
800111d0:	8c42000c 	lw	v0,12(v0)
800111d4:	afc2005c 	sw	v0,92(s8)
800111d8:	8fc20028 	lw	v0,40(s8)
800111dc:	8c420008 	lw	v0,8(v0)
800111e0:	afc20060 	sw	v0,96(s8)
800111e4:	8fc20060 	lw	v0,96(s8)
800111e8:	8fc3005c 	lw	v1,92(s8)
800111ec:	ac43000c 	sw	v1,12(v0)
800111f0:	8fc2005c 	lw	v0,92(s8)
800111f4:	8fc30060 	lw	v1,96(s8)
800111f8:	ac430008 	sw	v1,8(v0)
800111fc:	8fc20064 	lw	v0,100(s8)
80011200:	8c42000c 	lw	v0,12(v0)
80011204:	afc2005c 	sw	v0,92(s8)
80011208:	8fc20064 	lw	v0,100(s8)
8001120c:	8c420008 	lw	v0,8(v0)
80011210:	afc20060 	sw	v0,96(s8)
80011214:	8fc20060 	lw	v0,96(s8)
80011218:	8fc3005c 	lw	v1,92(s8)
8001121c:	ac43000c 	sw	v1,12(v0)
80011220:	8fc2005c 	lw	v0,92(s8)
80011224:	8fc30060 	lw	v1,96(s8)
80011228:	ac430008 	sw	v1,8(v0)
8001122c:	8fc20064 	lw	v0,100(s8)
80011230:	afc20020 	sw	v0,32(s8)
80011234:	8fc3002c 	lw	v1,44(s8)
80011238:	8fc20068 	lw	v0,104(s8)
8001123c:	00621021 	addu	v0,v1,v0
80011240:	8fc30024 	lw	v1,36(s8)
80011244:	00621021 	addu	v0,v1,v0
80011248:	afc20024 	sw	v0,36(s8)
8001124c:	8fc20020 	lw	v0,32(s8)
80011250:	24420008 	addiu	v0,v0,8
80011254:	afc2006c 	sw	v0,108(s8)
80011258:	8fc20054 	lw	v0,84(s8)
8001125c:	2442fffc 	addiu	v0,v0,-4
80011260:	afc20070 	sw	v0,112(s8)
80011264:	8fc20070 	lw	v0,112(s8)
80011268:	2c420025 	sltiu	v0,v0,37
8001126c:	10400057 	beqz	v0,800113cc <_realloc_r+0x7a0>
80011270:	00000000 	nop
80011274:	8fc200a0 	lw	v0,160(s8)
80011278:	afc20038 	sw	v0,56(s8)
8001127c:	8fc2006c 	lw	v0,108(s8)
80011280:	afc2003c 	sw	v0,60(s8)
80011284:	8fc20070 	lw	v0,112(s8)
80011288:	2c420014 	sltiu	v0,v0,20
8001128c:	14400039 	bnez	v0,80011374 <_realloc_r+0x748>
80011290:	00000000 	nop
80011294:	8fc30038 	lw	v1,56(s8)
80011298:	24620004 	addiu	v0,v1,4
8001129c:	afc20038 	sw	v0,56(s8)
800112a0:	8fc2003c 	lw	v0,60(s8)
800112a4:	24440004 	addiu	a0,v0,4
800112a8:	afc4003c 	sw	a0,60(s8)
800112ac:	8c630000 	lw	v1,0(v1)
800112b0:	ac430000 	sw	v1,0(v0)
800112b4:	8fc30038 	lw	v1,56(s8)
800112b8:	24620004 	addiu	v0,v1,4
800112bc:	afc20038 	sw	v0,56(s8)
800112c0:	8fc2003c 	lw	v0,60(s8)
800112c4:	24440004 	addiu	a0,v0,4
800112c8:	afc4003c 	sw	a0,60(s8)
800112cc:	8c630000 	lw	v1,0(v1)
800112d0:	ac430000 	sw	v1,0(v0)
800112d4:	8fc20070 	lw	v0,112(s8)
800112d8:	2c42001c 	sltiu	v0,v0,28
800112dc:	14400025 	bnez	v0,80011374 <_realloc_r+0x748>
800112e0:	00000000 	nop
800112e4:	8fc30038 	lw	v1,56(s8)
800112e8:	24620004 	addiu	v0,v1,4
800112ec:	afc20038 	sw	v0,56(s8)
800112f0:	8fc2003c 	lw	v0,60(s8)
800112f4:	24440004 	addiu	a0,v0,4
800112f8:	afc4003c 	sw	a0,60(s8)
800112fc:	8c630000 	lw	v1,0(v1)
80011300:	ac430000 	sw	v1,0(v0)
80011304:	8fc30038 	lw	v1,56(s8)
80011308:	24620004 	addiu	v0,v1,4
8001130c:	afc20038 	sw	v0,56(s8)
80011310:	8fc2003c 	lw	v0,60(s8)
80011314:	24440004 	addiu	a0,v0,4
80011318:	afc4003c 	sw	a0,60(s8)
8001131c:	8c630000 	lw	v1,0(v1)
80011320:	ac430000 	sw	v1,0(v0)
80011324:	8fc20070 	lw	v0,112(s8)
80011328:	2c420024 	sltiu	v0,v0,36
8001132c:	14400011 	bnez	v0,80011374 <_realloc_r+0x748>
80011330:	00000000 	nop
80011334:	8fc30038 	lw	v1,56(s8)
80011338:	24620004 	addiu	v0,v1,4
8001133c:	afc20038 	sw	v0,56(s8)
80011340:	8fc2003c 	lw	v0,60(s8)
80011344:	24440004 	addiu	a0,v0,4
80011348:	afc4003c 	sw	a0,60(s8)
8001134c:	8c630000 	lw	v1,0(v1)
80011350:	ac430000 	sw	v1,0(v0)
80011354:	8fc30038 	lw	v1,56(s8)
80011358:	24620004 	addiu	v0,v1,4
8001135c:	afc20038 	sw	v0,56(s8)
80011360:	8fc2003c 	lw	v0,60(s8)
80011364:	24440004 	addiu	a0,v0,4
80011368:	afc4003c 	sw	a0,60(s8)
8001136c:	8c630000 	lw	v1,0(v1)
80011370:	ac430000 	sw	v1,0(v0)
80011374:	8fc30038 	lw	v1,56(s8)
80011378:	24620004 	addiu	v0,v1,4
8001137c:	afc20038 	sw	v0,56(s8)
80011380:	8fc2003c 	lw	v0,60(s8)
80011384:	24440004 	addiu	a0,v0,4
80011388:	afc4003c 	sw	a0,60(s8)
8001138c:	8c630000 	lw	v1,0(v1)
80011390:	ac430000 	sw	v1,0(v0)
80011394:	8fc30038 	lw	v1,56(s8)
80011398:	24620004 	addiu	v0,v1,4
8001139c:	afc20038 	sw	v0,56(s8)
800113a0:	8fc2003c 	lw	v0,60(s8)
800113a4:	24440004 	addiu	a0,v0,4
800113a8:	afc4003c 	sw	a0,60(s8)
800113ac:	8c630000 	lw	v1,0(v1)
800113b0:	ac430000 	sw	v1,0(v0)
800113b4:	8fc20038 	lw	v0,56(s8)
800113b8:	8c430000 	lw	v1,0(v0)
800113bc:	8fc2003c 	lw	v0,60(s8)
800113c0:	ac430000 	sw	v1,0(v0)
800113c4:	1000011e 	b	80011840 <_realloc_r+0xc14>
800113c8:	00000000 	nop
800113cc:	8fc60070 	lw	a2,112(s8)
800113d0:	8fc500a0 	lw	a1,160(s8)
800113d4:	8fc4006c 	lw	a0,108(s8)
800113d8:	0c0016ab 	jal	80005aac <memmove>
800113dc:	00000000 	nop
800113e0:	10000117 	b	80011840 <_realloc_r+0xc14>
800113e4:	00000000 	nop
800113e8:	8fc20064 	lw	v0,100(s8)
800113ec:	10400082 	beqz	v0,800115f8 <_realloc_r+0x9cc>
800113f0:	00000000 	nop
800113f4:	8fc30068 	lw	v1,104(s8)
800113f8:	8fc20024 	lw	v0,36(s8)
800113fc:	00621021 	addu	v0,v1,v0
80011400:	00401825 	move	v1,v0
80011404:	8fc20058 	lw	v0,88(s8)
80011408:	0062102a 	slt	v0,v1,v0
8001140c:	1440007a 	bnez	v0,800115f8 <_realloc_r+0x9cc>
80011410:	00000000 	nop
80011414:	8fc20064 	lw	v0,100(s8)
80011418:	8c42000c 	lw	v0,12(v0)
8001141c:	afc2005c 	sw	v0,92(s8)
80011420:	8fc20064 	lw	v0,100(s8)
80011424:	8c420008 	lw	v0,8(v0)
80011428:	afc20060 	sw	v0,96(s8)
8001142c:	8fc20060 	lw	v0,96(s8)
80011430:	8fc3005c 	lw	v1,92(s8)
80011434:	ac43000c 	sw	v1,12(v0)
80011438:	8fc2005c 	lw	v0,92(s8)
8001143c:	8fc30060 	lw	v1,96(s8)
80011440:	ac430008 	sw	v1,8(v0)
80011444:	8fc20064 	lw	v0,100(s8)
80011448:	afc20020 	sw	v0,32(s8)
8001144c:	8fc30024 	lw	v1,36(s8)
80011450:	8fc20068 	lw	v0,104(s8)
80011454:	00621021 	addu	v0,v1,v0
80011458:	afc20024 	sw	v0,36(s8)
8001145c:	8fc20020 	lw	v0,32(s8)
80011460:	24420008 	addiu	v0,v0,8
80011464:	afc2006c 	sw	v0,108(s8)
80011468:	8fc20054 	lw	v0,84(s8)
8001146c:	2442fffc 	addiu	v0,v0,-4
80011470:	afc20078 	sw	v0,120(s8)
80011474:	8fc20078 	lw	v0,120(s8)
80011478:	2c420025 	sltiu	v0,v0,37
8001147c:	10400057 	beqz	v0,800115dc <_realloc_r+0x9b0>
80011480:	00000000 	nop
80011484:	8fc200a0 	lw	v0,160(s8)
80011488:	afc20040 	sw	v0,64(s8)
8001148c:	8fc2006c 	lw	v0,108(s8)
80011490:	afc20044 	sw	v0,68(s8)
80011494:	8fc20078 	lw	v0,120(s8)
80011498:	2c420014 	sltiu	v0,v0,20
8001149c:	14400039 	bnez	v0,80011584 <_realloc_r+0x958>
800114a0:	00000000 	nop
800114a4:	8fc30040 	lw	v1,64(s8)
800114a8:	24620004 	addiu	v0,v1,4
800114ac:	afc20040 	sw	v0,64(s8)
800114b0:	8fc20044 	lw	v0,68(s8)
800114b4:	24440004 	addiu	a0,v0,4
800114b8:	afc40044 	sw	a0,68(s8)
800114bc:	8c630000 	lw	v1,0(v1)
800114c0:	ac430000 	sw	v1,0(v0)
800114c4:	8fc30040 	lw	v1,64(s8)
800114c8:	24620004 	addiu	v0,v1,4
800114cc:	afc20040 	sw	v0,64(s8)
800114d0:	8fc20044 	lw	v0,68(s8)
800114d4:	24440004 	addiu	a0,v0,4
800114d8:	afc40044 	sw	a0,68(s8)
800114dc:	8c630000 	lw	v1,0(v1)
800114e0:	ac430000 	sw	v1,0(v0)
800114e4:	8fc20078 	lw	v0,120(s8)
800114e8:	2c42001c 	sltiu	v0,v0,28
800114ec:	14400025 	bnez	v0,80011584 <_realloc_r+0x958>
800114f0:	00000000 	nop
800114f4:	8fc30040 	lw	v1,64(s8)
800114f8:	24620004 	addiu	v0,v1,4
800114fc:	afc20040 	sw	v0,64(s8)
80011500:	8fc20044 	lw	v0,68(s8)
80011504:	24440004 	addiu	a0,v0,4
80011508:	afc40044 	sw	a0,68(s8)
8001150c:	8c630000 	lw	v1,0(v1)
80011510:	ac430000 	sw	v1,0(v0)
80011514:	8fc30040 	lw	v1,64(s8)
80011518:	24620004 	addiu	v0,v1,4
8001151c:	afc20040 	sw	v0,64(s8)
80011520:	8fc20044 	lw	v0,68(s8)
80011524:	24440004 	addiu	a0,v0,4
80011528:	afc40044 	sw	a0,68(s8)
8001152c:	8c630000 	lw	v1,0(v1)
80011530:	ac430000 	sw	v1,0(v0)
80011534:	8fc20078 	lw	v0,120(s8)
80011538:	2c420024 	sltiu	v0,v0,36
8001153c:	14400011 	bnez	v0,80011584 <_realloc_r+0x958>
80011540:	00000000 	nop
80011544:	8fc30040 	lw	v1,64(s8)
80011548:	24620004 	addiu	v0,v1,4
8001154c:	afc20040 	sw	v0,64(s8)
80011550:	8fc20044 	lw	v0,68(s8)
80011554:	24440004 	addiu	a0,v0,4
80011558:	afc40044 	sw	a0,68(s8)
8001155c:	8c630000 	lw	v1,0(v1)
80011560:	ac430000 	sw	v1,0(v0)
80011564:	8fc30040 	lw	v1,64(s8)
80011568:	24620004 	addiu	v0,v1,4
8001156c:	afc20040 	sw	v0,64(s8)
80011570:	8fc20044 	lw	v0,68(s8)
80011574:	24440004 	addiu	a0,v0,4
80011578:	afc40044 	sw	a0,68(s8)
8001157c:	8c630000 	lw	v1,0(v1)
80011580:	ac430000 	sw	v1,0(v0)
80011584:	8fc30040 	lw	v1,64(s8)
80011588:	24620004 	addiu	v0,v1,4
8001158c:	afc20040 	sw	v0,64(s8)
80011590:	8fc20044 	lw	v0,68(s8)
80011594:	24440004 	addiu	a0,v0,4
80011598:	afc40044 	sw	a0,68(s8)
8001159c:	8c630000 	lw	v1,0(v1)
800115a0:	ac430000 	sw	v1,0(v0)
800115a4:	8fc30040 	lw	v1,64(s8)
800115a8:	24620004 	addiu	v0,v1,4
800115ac:	afc20040 	sw	v0,64(s8)
800115b0:	8fc20044 	lw	v0,68(s8)
800115b4:	24440004 	addiu	a0,v0,4
800115b8:	afc40044 	sw	a0,68(s8)
800115bc:	8c630000 	lw	v1,0(v1)
800115c0:	ac430000 	sw	v1,0(v0)
800115c4:	8fc20040 	lw	v0,64(s8)
800115c8:	8c430000 	lw	v1,0(v0)
800115cc:	8fc20044 	lw	v0,68(s8)
800115d0:	ac430000 	sw	v1,0(v0)
800115d4:	1000009a 	b	80011840 <_realloc_r+0xc14>
800115d8:	00000000 	nop
800115dc:	8fc60078 	lw	a2,120(s8)
800115e0:	8fc500a0 	lw	a1,160(s8)
800115e4:	8fc4006c 	lw	a0,108(s8)
800115e8:	0c0016ab 	jal	80005aac <memmove>
800115ec:	00000000 	nop
800115f0:	10000093 	b	80011840 <_realloc_r+0xc14>
800115f4:	00000000 	nop
800115f8:	8fc500a8 	lw	a1,168(s8)
800115fc:	8fc40098 	lw	a0,152(s8)
80011600:	0c001296 	jal	80004a58 <_malloc_r>
80011604:	00000000 	nop
80011608:	afc2006c 	sw	v0,108(s8)
8001160c:	8fc2006c 	lw	v0,108(s8)
80011610:	14400007 	bnez	v0,80011630 <_realloc_r+0xa04>
80011614:	00000000 	nop
80011618:	8fc40098 	lw	a0,152(s8)
8001161c:	0c001bb1 	jal	80006ec4 <__malloc_unlock>
80011620:	00000000 	nop
80011624:	00001025 	move	v0,zero
80011628:	100000c2 	b	80011934 <_realloc_r+0xd08>
8001162c:	00000000 	nop
80011630:	8fc2006c 	lw	v0,108(s8)
80011634:	2442fff8 	addiu	v0,v0,-8
80011638:	afc20020 	sw	v0,32(s8)
8001163c:	8fc20050 	lw	v0,80(s8)
80011640:	8c430004 	lw	v1,4(v0)
80011644:	2402fffe 	li	v0,-2
80011648:	00621024 	and	v0,v1,v0
8001164c:	8fc30050 	lw	v1,80(s8)
80011650:	00621021 	addu	v0,v1,v0
80011654:	8fc30020 	lw	v1,32(s8)
80011658:	1462000c 	bne	v1,v0,8001168c <_realloc_r+0xa60>
8001165c:	00000000 	nop
80011660:	8fc20020 	lw	v0,32(s8)
80011664:	8c430004 	lw	v1,4(v0)
80011668:	2402fffc 	li	v0,-4
8001166c:	00621024 	and	v0,v1,v0
80011670:	8fc30024 	lw	v1,36(s8)
80011674:	00621021 	addu	v0,v1,v0
80011678:	afc20024 	sw	v0,36(s8)
8001167c:	8fc20050 	lw	v0,80(s8)
80011680:	afc20020 	sw	v0,32(s8)
80011684:	1000006e 	b	80011840 <_realloc_r+0xc14>
80011688:	00000000 	nop
8001168c:	8fc20054 	lw	v0,84(s8)
80011690:	2442fffc 	addiu	v0,v0,-4
80011694:	afc2007c 	sw	v0,124(s8)
80011698:	8fc2007c 	lw	v0,124(s8)
8001169c:	2c420025 	sltiu	v0,v0,37
800116a0:	10400057 	beqz	v0,80011800 <_realloc_r+0xbd4>
800116a4:	00000000 	nop
800116a8:	8fc200a0 	lw	v0,160(s8)
800116ac:	afc20048 	sw	v0,72(s8)
800116b0:	8fc2006c 	lw	v0,108(s8)
800116b4:	afc2004c 	sw	v0,76(s8)
800116b8:	8fc2007c 	lw	v0,124(s8)
800116bc:	2c420014 	sltiu	v0,v0,20
800116c0:	14400039 	bnez	v0,800117a8 <_realloc_r+0xb7c>
800116c4:	00000000 	nop
800116c8:	8fc30048 	lw	v1,72(s8)
800116cc:	24620004 	addiu	v0,v1,4
800116d0:	afc20048 	sw	v0,72(s8)
800116d4:	8fc2004c 	lw	v0,76(s8)
800116d8:	24440004 	addiu	a0,v0,4
800116dc:	afc4004c 	sw	a0,76(s8)
800116e0:	8c630000 	lw	v1,0(v1)
800116e4:	ac430000 	sw	v1,0(v0)
800116e8:	8fc30048 	lw	v1,72(s8)
800116ec:	24620004 	addiu	v0,v1,4
800116f0:	afc20048 	sw	v0,72(s8)
800116f4:	8fc2004c 	lw	v0,76(s8)
800116f8:	24440004 	addiu	a0,v0,4
800116fc:	afc4004c 	sw	a0,76(s8)
80011700:	8c630000 	lw	v1,0(v1)
80011704:	ac430000 	sw	v1,0(v0)
80011708:	8fc2007c 	lw	v0,124(s8)
8001170c:	2c42001c 	sltiu	v0,v0,28
80011710:	14400025 	bnez	v0,800117a8 <_realloc_r+0xb7c>
80011714:	00000000 	nop
80011718:	8fc30048 	lw	v1,72(s8)
8001171c:	24620004 	addiu	v0,v1,4
80011720:	afc20048 	sw	v0,72(s8)
80011724:	8fc2004c 	lw	v0,76(s8)
80011728:	24440004 	addiu	a0,v0,4
8001172c:	afc4004c 	sw	a0,76(s8)
80011730:	8c630000 	lw	v1,0(v1)
80011734:	ac430000 	sw	v1,0(v0)
80011738:	8fc30048 	lw	v1,72(s8)
8001173c:	24620004 	addiu	v0,v1,4
80011740:	afc20048 	sw	v0,72(s8)
80011744:	8fc2004c 	lw	v0,76(s8)
80011748:	24440004 	addiu	a0,v0,4
8001174c:	afc4004c 	sw	a0,76(s8)
80011750:	8c630000 	lw	v1,0(v1)
80011754:	ac430000 	sw	v1,0(v0)
80011758:	8fc2007c 	lw	v0,124(s8)
8001175c:	2c420024 	sltiu	v0,v0,36
80011760:	14400011 	bnez	v0,800117a8 <_realloc_r+0xb7c>
80011764:	00000000 	nop
80011768:	8fc30048 	lw	v1,72(s8)
8001176c:	24620004 	addiu	v0,v1,4
80011770:	afc20048 	sw	v0,72(s8)
80011774:	8fc2004c 	lw	v0,76(s8)
80011778:	24440004 	addiu	a0,v0,4
8001177c:	afc4004c 	sw	a0,76(s8)
80011780:	8c630000 	lw	v1,0(v1)
80011784:	ac430000 	sw	v1,0(v0)
80011788:	8fc30048 	lw	v1,72(s8)
8001178c:	24620004 	addiu	v0,v1,4
80011790:	afc20048 	sw	v0,72(s8)
80011794:	8fc2004c 	lw	v0,76(s8)
80011798:	24440004 	addiu	a0,v0,4
8001179c:	afc4004c 	sw	a0,76(s8)
800117a0:	8c630000 	lw	v1,0(v1)
800117a4:	ac430000 	sw	v1,0(v0)
800117a8:	8fc30048 	lw	v1,72(s8)
800117ac:	24620004 	addiu	v0,v1,4
800117b0:	afc20048 	sw	v0,72(s8)
800117b4:	8fc2004c 	lw	v0,76(s8)
800117b8:	24440004 	addiu	a0,v0,4
800117bc:	afc4004c 	sw	a0,76(s8)
800117c0:	8c630000 	lw	v1,0(v1)
800117c4:	ac430000 	sw	v1,0(v0)
800117c8:	8fc30048 	lw	v1,72(s8)
800117cc:	24620004 	addiu	v0,v1,4
800117d0:	afc20048 	sw	v0,72(s8)
800117d4:	8fc2004c 	lw	v0,76(s8)
800117d8:	24440004 	addiu	a0,v0,4
800117dc:	afc4004c 	sw	a0,76(s8)
800117e0:	8c630000 	lw	v1,0(v1)
800117e4:	ac430000 	sw	v1,0(v0)
800117e8:	8fc20048 	lw	v0,72(s8)
800117ec:	8c430000 	lw	v1,0(v0)
800117f0:	8fc2004c 	lw	v0,76(s8)
800117f4:	ac430000 	sw	v1,0(v0)
800117f8:	10000006 	b	80011814 <_realloc_r+0xbe8>
800117fc:	00000000 	nop
80011800:	8fc6007c 	lw	a2,124(s8)
80011804:	8fc500a0 	lw	a1,160(s8)
80011808:	8fc4006c 	lw	a0,108(s8)
8001180c:	0c0016ab 	jal	80005aac <memmove>
80011810:	00000000 	nop
80011814:	8fc500a0 	lw	a1,160(s8)
80011818:	8fc40098 	lw	a0,152(s8)
8001181c:	0c000dcb 	jal	8000372c <_free_r>
80011820:	00000000 	nop
80011824:	8fc40098 	lw	a0,152(s8)
80011828:	0c001bb1 	jal	80006ec4 <__malloc_unlock>
8001182c:	00000000 	nop
80011830:	8fc2006c 	lw	v0,108(s8)
80011834:	1000003f 	b	80011934 <_realloc_r+0xd08>
80011838:	00000000 	nop
8001183c:	00000000 	nop
80011840:	8fc30024 	lw	v1,36(s8)
80011844:	8fc20058 	lw	v0,88(s8)
80011848:	00621023 	subu	v0,v1,v0
8001184c:	afc20080 	sw	v0,128(s8)
80011850:	8fc20080 	lw	v0,128(s8)
80011854:	2c420010 	sltiu	v0,v0,16
80011858:	14400021 	bnez	v0,800118e0 <_realloc_r+0xcb4>
8001185c:	00000000 	nop
80011860:	8fc30020 	lw	v1,32(s8)
80011864:	8fc20058 	lw	v0,88(s8)
80011868:	00621021 	addu	v0,v1,v0
8001186c:	afc20084 	sw	v0,132(s8)
80011870:	8fc20020 	lw	v0,32(s8)
80011874:	8c420004 	lw	v0,4(v0)
80011878:	30430001 	andi	v1,v0,0x1
8001187c:	8fc20058 	lw	v0,88(s8)
80011880:	00621825 	or	v1,v1,v0
80011884:	8fc20020 	lw	v0,32(s8)
80011888:	ac430004 	sw	v1,4(v0)
8001188c:	8fc20080 	lw	v0,128(s8)
80011890:	34430001 	ori	v1,v0,0x1
80011894:	8fc20084 	lw	v0,132(s8)
80011898:	ac430004 	sw	v1,4(v0)
8001189c:	8fc30084 	lw	v1,132(s8)
800118a0:	8fc20080 	lw	v0,128(s8)
800118a4:	00621021 	addu	v0,v1,v0
800118a8:	8c430004 	lw	v1,4(v0)
800118ac:	8fc40084 	lw	a0,132(s8)
800118b0:	8fc20080 	lw	v0,128(s8)
800118b4:	00821021 	addu	v0,a0,v0
800118b8:	34630001 	ori	v1,v1,0x1
800118bc:	ac430004 	sw	v1,4(v0)
800118c0:	8fc20084 	lw	v0,132(s8)
800118c4:	24420008 	addiu	v0,v0,8
800118c8:	00402825 	move	a1,v0
800118cc:	8fc40098 	lw	a0,152(s8)
800118d0:	0c000dcb 	jal	8000372c <_free_r>
800118d4:	00000000 	nop
800118d8:	10000011 	b	80011920 <_realloc_r+0xcf4>
800118dc:	00000000 	nop
800118e0:	8fc20020 	lw	v0,32(s8)
800118e4:	8c420004 	lw	v0,4(v0)
800118e8:	30430001 	andi	v1,v0,0x1
800118ec:	8fc20024 	lw	v0,36(s8)
800118f0:	00621825 	or	v1,v1,v0
800118f4:	8fc20020 	lw	v0,32(s8)
800118f8:	ac430004 	sw	v1,4(v0)
800118fc:	8fc30020 	lw	v1,32(s8)
80011900:	8fc20024 	lw	v0,36(s8)
80011904:	00621021 	addu	v0,v1,v0
80011908:	8c430004 	lw	v1,4(v0)
8001190c:	8fc40020 	lw	a0,32(s8)
80011910:	8fc20024 	lw	v0,36(s8)
80011914:	00821021 	addu	v0,a0,v0
80011918:	34630001 	ori	v1,v1,0x1
8001191c:	ac430004 	sw	v1,4(v0)
80011920:	8fc40098 	lw	a0,152(s8)
80011924:	0c001bb1 	jal	80006ec4 <__malloc_unlock>
80011928:	00000000 	nop
8001192c:	8fc20020 	lw	v0,32(s8)
80011930:	24420008 	addiu	v0,v0,8
80011934:	03c0e825 	move	sp,s8
80011938:	dfbf0090 	ld	ra,144(sp)
8001193c:	dfbe0088 	ld	s8,136(sp)
80011940:	27bd0098 	addiu	sp,sp,152
80011944:	03e00008 	jr	ra
80011948:	00000000 	nop

8001194c <siscanf>:
8001194c:	27bdff60 	addiu	sp,sp,-160
80011950:	ffbf0098 	sd	ra,152(sp)
80011954:	ffbe0090 	sd	s8,144(sp)
80011958:	03a0f025 	move	s8,sp
8001195c:	afc400a0 	sw	a0,160(s8)
80011960:	afc500a8 	sw	a1,168(s8)
80011964:	ffc600b0 	sd	a2,176(s8)
80011968:	ffc700b8 	sd	a3,184(s8)
8001196c:	24020204 	li	v0,516
80011970:	a7c20034 	sh	v0,52(s8)
80011974:	8fc200a0 	lw	v0,160(s8)
80011978:	afc20028 	sw	v0,40(s8)
8001197c:	8fc20028 	lw	v0,40(s8)
80011980:	afc20038 	sw	v0,56(s8)
80011984:	8fc400a0 	lw	a0,160(s8)
80011988:	0c001f1d 	jal	80007c74 <strlen>
8001198c:	00000000 	nop
80011990:	afc2002c 	sw	v0,44(s8)
80011994:	8fc2002c 	lw	v0,44(s8)
80011998:	afc2003c 	sw	v0,60(s8)
8001199c:	3c028000 	lui	v0,0x8000
800119a0:	24427a84 	addiu	v0,v0,31364
800119a4:	afc20048 	sw	v0,72(s8)
800119a8:	afc00058 	sw	zero,88(s8)
800119ac:	afc0006c 	sw	zero,108(s8)
800119b0:	2402ffff 	li	v0,-1
800119b4:	a7c20036 	sh	v0,54(s8)
800119b8:	27c200b0 	addiu	v0,s8,176
800119bc:	afc20024 	sw	v0,36(s8)
800119c0:	8f828028 	lw	v0,-32728(gp)
800119c4:	8fc30024 	lw	v1,36(s8)
800119c8:	27c40028 	addiu	a0,s8,40
800119cc:	00603825 	move	a3,v1
800119d0:	8fc600a8 	lw	a2,168(s8)
800119d4:	00802825 	move	a1,a0
800119d8:	00402025 	move	a0,v0
800119dc:	0c004a44 	jal	80012910 <__ssvfiscanf_r>
800119e0:	00000000 	nop
800119e4:	afc20020 	sw	v0,32(s8)
800119e8:	8fc20020 	lw	v0,32(s8)
800119ec:	03c0e825 	move	sp,s8
800119f0:	dfbf0098 	ld	ra,152(sp)
800119f4:	dfbe0090 	ld	s8,144(sp)
800119f8:	27bd00a0 	addiu	sp,sp,160
800119fc:	03e00008 	jr	ra
80011a00:	00000000 	nop

80011a04 <_siscanf_r>:
80011a04:	27bdff60 	addiu	sp,sp,-160
80011a08:	ffbf0098 	sd	ra,152(sp)
80011a0c:	ffbe0090 	sd	s8,144(sp)
80011a10:	03a0f025 	move	s8,sp
80011a14:	afc400a0 	sw	a0,160(s8)
80011a18:	afc500a8 	sw	a1,168(s8)
80011a1c:	afc600b0 	sw	a2,176(s8)
80011a20:	ffc700b8 	sd	a3,184(s8)
80011a24:	24020204 	li	v0,516
80011a28:	a7c20034 	sh	v0,52(s8)
80011a2c:	8fc200a8 	lw	v0,168(s8)
80011a30:	afc20028 	sw	v0,40(s8)
80011a34:	8fc20028 	lw	v0,40(s8)
80011a38:	afc20038 	sw	v0,56(s8)
80011a3c:	8fc400a8 	lw	a0,168(s8)
80011a40:	0c001f1d 	jal	80007c74 <strlen>
80011a44:	00000000 	nop
80011a48:	afc2002c 	sw	v0,44(s8)
80011a4c:	8fc2002c 	lw	v0,44(s8)
80011a50:	afc2003c 	sw	v0,60(s8)
80011a54:	3c028000 	lui	v0,0x8000
80011a58:	24427a84 	addiu	v0,v0,31364
80011a5c:	afc20048 	sw	v0,72(s8)
80011a60:	afc00058 	sw	zero,88(s8)
80011a64:	afc0006c 	sw	zero,108(s8)
80011a68:	2402ffff 	li	v0,-1
80011a6c:	a7c20036 	sh	v0,54(s8)
80011a70:	27c200b8 	addiu	v0,s8,184
80011a74:	afc20024 	sw	v0,36(s8)
80011a78:	8fc20024 	lw	v0,36(s8)
80011a7c:	27c30028 	addiu	v1,s8,40
80011a80:	00403825 	move	a3,v0
80011a84:	8fc600b0 	lw	a2,176(s8)
80011a88:	00602825 	move	a1,v1
80011a8c:	8fc400a0 	lw	a0,160(s8)
80011a90:	0c004a44 	jal	80012910 <__ssvfiscanf_r>
80011a94:	00000000 	nop
80011a98:	afc20020 	sw	v0,32(s8)
80011a9c:	8fc20020 	lw	v0,32(s8)
80011aa0:	03c0e825 	move	sp,s8
80011aa4:	dfbf0098 	ld	ra,152(sp)
80011aa8:	dfbe0090 	ld	s8,144(sp)
80011aac:	27bd00a0 	addiu	sp,sp,160
80011ab0:	03e00008 	jr	ra
80011ab4:	00000000 	nop

80011ab8 <strcmp>:
80011ab8:	00854025 	or	t0,a0,a1
80011abc:	31080003 	andi	t0,t0,0x3
80011ac0:	1500005e 	bnez	t0,80011c3c <strcmp+0x184>
80011ac4:	3c180101 	lui	t8,0x101
80011ac8:	37180101 	ori	t8,t8,0x101
80011acc:	3c197f7f 	lui	t9,0x7f7f
80011ad0:	37397f7f 	ori	t9,t9,0x7f7f
80011ad4:	8c820000 	lw	v0,0(a0)
80011ad8:	8ca30000 	lw	v1,0(a1)
80011adc:	00584023 	subu	t0,v0,t8
80011ae0:	14430040 	bne	v0,v1,80011be4 <strcmp+0x12c>
80011ae4:	00594827 	nor	t1,v0,t9
80011ae8:	01094024 	and	t0,t0,t1
80011aec:	1500003b 	bnez	t0,80011bdc <strcmp+0x124>
80011af0:	00000000 	nop
80011af4:	8c820004 	lw	v0,4(a0)
80011af8:	8ca30004 	lw	v1,4(a1)
80011afc:	00584023 	subu	t0,v0,t8
80011b00:	14430038 	bne	v0,v1,80011be4 <strcmp+0x12c>
80011b04:	00594827 	nor	t1,v0,t9
80011b08:	01094024 	and	t0,t0,t1
80011b0c:	15000033 	bnez	t0,80011bdc <strcmp+0x124>
80011b10:	00000000 	nop
80011b14:	8c820008 	lw	v0,8(a0)
80011b18:	8ca30008 	lw	v1,8(a1)
80011b1c:	00584023 	subu	t0,v0,t8
80011b20:	14430030 	bne	v0,v1,80011be4 <strcmp+0x12c>
80011b24:	00594827 	nor	t1,v0,t9
80011b28:	01094024 	and	t0,t0,t1
80011b2c:	1500002b 	bnez	t0,80011bdc <strcmp+0x124>
80011b30:	00000000 	nop
80011b34:	8c82000c 	lw	v0,12(a0)
80011b38:	8ca3000c 	lw	v1,12(a1)
80011b3c:	00584023 	subu	t0,v0,t8
80011b40:	14430028 	bne	v0,v1,80011be4 <strcmp+0x12c>
80011b44:	00594827 	nor	t1,v0,t9
80011b48:	01094024 	and	t0,t0,t1
80011b4c:	15000023 	bnez	t0,80011bdc <strcmp+0x124>
80011b50:	00000000 	nop
80011b54:	8c820010 	lw	v0,16(a0)
80011b58:	8ca30010 	lw	v1,16(a1)
80011b5c:	00584023 	subu	t0,v0,t8
80011b60:	14430020 	bne	v0,v1,80011be4 <strcmp+0x12c>
80011b64:	00594827 	nor	t1,v0,t9
80011b68:	01094024 	and	t0,t0,t1
80011b6c:	1500001b 	bnez	t0,80011bdc <strcmp+0x124>
80011b70:	00000000 	nop
80011b74:	8c820014 	lw	v0,20(a0)
80011b78:	8ca30014 	lw	v1,20(a1)
80011b7c:	00584023 	subu	t0,v0,t8
80011b80:	14430018 	bne	v0,v1,80011be4 <strcmp+0x12c>
80011b84:	00594827 	nor	t1,v0,t9
80011b88:	01094024 	and	t0,t0,t1
80011b8c:	15000013 	bnez	t0,80011bdc <strcmp+0x124>
80011b90:	00000000 	nop
80011b94:	8c820018 	lw	v0,24(a0)
80011b98:	8ca30018 	lw	v1,24(a1)
80011b9c:	00584023 	subu	t0,v0,t8
80011ba0:	14430010 	bne	v0,v1,80011be4 <strcmp+0x12c>
80011ba4:	00594827 	nor	t1,v0,t9
80011ba8:	01094024 	and	t0,t0,t1
80011bac:	1500000b 	bnez	t0,80011bdc <strcmp+0x124>
80011bb0:	00000000 	nop
80011bb4:	8c82001c 	lw	v0,28(a0)
80011bb8:	8ca3001c 	lw	v1,28(a1)
80011bbc:	00584023 	subu	t0,v0,t8
80011bc0:	14430008 	bne	v0,v1,80011be4 <strcmp+0x12c>
80011bc4:	00594827 	nor	t1,v0,t9
80011bc8:	01094024 	and	t0,t0,t1
80011bcc:	15000003 	bnez	t0,80011bdc <strcmp+0x124>
80011bd0:	24840020 	addiu	a0,a0,32
80011bd4:	1000ffbf 	b	80011ad4 <strcmp+0x1c>
80011bd8:	24a50020 	addiu	a1,a1,32
80011bdc:	03e00008 	jr	ra
80011be0:	00001025 	move	v0,zero
80011be4:	00024602 	srl	t0,v0,0x18
80011be8:	11000012 	beqz	t0,80011c34 <strcmp+0x17c>
80011bec:	00034e02 	srl	t1,v1,0x18
80011bf0:	15090010 	bne	t0,t1,80011c34 <strcmp+0x17c>
80011bf4:	0002c402 	srl	t8,v0,0x10
80011bf8:	0003cc02 	srl	t9,v1,0x10
80011bfc:	331800ff 	andi	t8,t8,0xff
80011c00:	1300000a 	beqz	t8,80011c2c <strcmp+0x174>
80011c04:	333900ff 	andi	t9,t9,0xff
80011c08:	17190008 	bne	t8,t9,80011c2c <strcmp+0x174>
80011c0c:	00024202 	srl	t0,v0,0x8
80011c10:	00034a02 	srl	t1,v1,0x8
80011c14:	310800ff 	andi	t0,t0,0xff
80011c18:	11000006 	beqz	t0,80011c34 <strcmp+0x17c>
80011c1c:	312900ff 	andi	t1,t1,0xff
80011c20:	15090004 	bne	t0,t1,80011c34 <strcmp+0x17c>
80011c24:	305800ff 	andi	t8,v0,0xff
80011c28:	307900ff 	andi	t9,v1,0xff
80011c2c:	03e00008 	jr	ra
80011c30:	03191023 	subu	v0,t8,t9
80011c34:	03e00008 	jr	ra
80011c38:	01091023 	subu	v0,t0,t1
80011c3c:	90820000 	lbu	v0,0(a0)
80011c40:	90a30000 	lbu	v1,0(a1)
80011c44:	10400028 	beqz	v0,80011ce8 <strcmp+0x230>
80011c48:	00000000 	nop
80011c4c:	14430026 	bne	v0,v1,80011ce8 <strcmp+0x230>
80011c50:	90980001 	lbu	t8,1(a0)
80011c54:	90b90001 	lbu	t9,1(a1)
80011c58:	13000025 	beqz	t8,80011cf0 <strcmp+0x238>
80011c5c:	00000000 	nop
80011c60:	17190023 	bne	t8,t9,80011cf0 <strcmp+0x238>
80011c64:	90820002 	lbu	v0,2(a0)
80011c68:	90a30002 	lbu	v1,2(a1)
80011c6c:	1040001e 	beqz	v0,80011ce8 <strcmp+0x230>
80011c70:	00000000 	nop
80011c74:	1443001c 	bne	v0,v1,80011ce8 <strcmp+0x230>
80011c78:	90980003 	lbu	t8,3(a0)
80011c7c:	90b90003 	lbu	t9,3(a1)
80011c80:	1300001b 	beqz	t8,80011cf0 <strcmp+0x238>
80011c84:	00000000 	nop
80011c88:	17190019 	bne	t8,t9,80011cf0 <strcmp+0x238>
80011c8c:	90820004 	lbu	v0,4(a0)
80011c90:	90a30004 	lbu	v1,4(a1)
80011c94:	10400014 	beqz	v0,80011ce8 <strcmp+0x230>
80011c98:	00000000 	nop
80011c9c:	14430012 	bne	v0,v1,80011ce8 <strcmp+0x230>
80011ca0:	90980005 	lbu	t8,5(a0)
80011ca4:	90b90005 	lbu	t9,5(a1)
80011ca8:	13000011 	beqz	t8,80011cf0 <strcmp+0x238>
80011cac:	00000000 	nop
80011cb0:	1719000f 	bne	t8,t9,80011cf0 <strcmp+0x238>
80011cb4:	90820006 	lbu	v0,6(a0)
80011cb8:	90a30006 	lbu	v1,6(a1)
80011cbc:	1040000a 	beqz	v0,80011ce8 <strcmp+0x230>
80011cc0:	00000000 	nop
80011cc4:	14430008 	bne	v0,v1,80011ce8 <strcmp+0x230>
80011cc8:	90980007 	lbu	t8,7(a0)
80011ccc:	90b90007 	lbu	t9,7(a1)
80011cd0:	13000007 	beqz	t8,80011cf0 <strcmp+0x238>
80011cd4:	00000000 	nop
80011cd8:	17190005 	bne	t8,t9,80011cf0 <strcmp+0x238>
80011cdc:	24840008 	addiu	a0,a0,8
80011ce0:	1000ffd6 	b	80011c3c <strcmp+0x184>
80011ce4:	24a50008 	addiu	a1,a1,8
80011ce8:	03e00008 	jr	ra
80011cec:	00431023 	subu	v0,v0,v1
80011cf0:	03e00008 	jr	ra
80011cf4:	03191023 	subu	v0,t8,t9

80011cf8 <strcpy>:
80011cf8:	27bdffe8 	addiu	sp,sp,-24
80011cfc:	ffbe0010 	sd	s8,16(sp)
80011d00:	03a0f025 	move	s8,sp
80011d04:	afc40018 	sw	a0,24(s8)
80011d08:	afc50020 	sw	a1,32(s8)
80011d0c:	8fc20018 	lw	v0,24(s8)
80011d10:	afc20000 	sw	v0,0(s8)
80011d14:	8fc20020 	lw	v0,32(s8)
80011d18:	afc20004 	sw	v0,4(s8)
80011d1c:	8fc30004 	lw	v1,4(s8)
80011d20:	8fc20000 	lw	v0,0(s8)
80011d24:	00621025 	or	v0,v1,v0
80011d28:	30420003 	andi	v0,v0,0x3
80011d2c:	14400022 	bnez	v0,80011db8 <strcpy+0xc0>
80011d30:	00000000 	nop
80011d34:	8fc20000 	lw	v0,0(s8)
80011d38:	afc20008 	sw	v0,8(s8)
80011d3c:	8fc20004 	lw	v0,4(s8)
80011d40:	afc2000c 	sw	v0,12(s8)
80011d44:	10000009 	b	80011d6c <strcpy+0x74>
80011d48:	00000000 	nop
80011d4c:	8fc3000c 	lw	v1,12(s8)
80011d50:	24620004 	addiu	v0,v1,4
80011d54:	afc2000c 	sw	v0,12(s8)
80011d58:	8fc20008 	lw	v0,8(s8)
80011d5c:	24440004 	addiu	a0,v0,4
80011d60:	afc40008 	sw	a0,8(s8)
80011d64:	8c630000 	lw	v1,0(v1)
80011d68:	ac430000 	sw	v1,0(v0)
80011d6c:	8fc2000c 	lw	v0,12(s8)
80011d70:	8c430000 	lw	v1,0(v0)
80011d74:	3c02fefe 	lui	v0,0xfefe
80011d78:	3442feff 	ori	v0,v0,0xfeff
80011d7c:	00621821 	addu	v1,v1,v0
80011d80:	8fc2000c 	lw	v0,12(s8)
80011d84:	8c420000 	lw	v0,0(v0)
80011d88:	00021027 	nor	v0,zero,v0
80011d8c:	00621024 	and	v0,v1,v0
80011d90:	00401825 	move	v1,v0
80011d94:	3c028080 	lui	v0,0x8080
80011d98:	34428080 	ori	v0,v0,0x8080
80011d9c:	00621024 	and	v0,v1,v0
80011da0:	1040ffea 	beqz	v0,80011d4c <strcpy+0x54>
80011da4:	00000000 	nop
80011da8:	8fc20008 	lw	v0,8(s8)
80011dac:	afc20000 	sw	v0,0(s8)
80011db0:	8fc2000c 	lw	v0,12(s8)
80011db4:	afc20004 	sw	v0,4(s8)
80011db8:	00000000 	nop
80011dbc:	8fc30004 	lw	v1,4(s8)
80011dc0:	24620001 	addiu	v0,v1,1
80011dc4:	afc20004 	sw	v0,4(s8)
80011dc8:	8fc20000 	lw	v0,0(s8)
80011dcc:	24440001 	addiu	a0,v0,1
80011dd0:	afc40000 	sw	a0,0(s8)
80011dd4:	80630000 	lb	v1,0(v1)
80011dd8:	a0430000 	sb	v1,0(v0)
80011ddc:	80420000 	lb	v0,0(v0)
80011de0:	1440fff6 	bnez	v0,80011dbc <strcpy+0xc4>
80011de4:	00000000 	nop
80011de8:	8fc20018 	lw	v0,24(s8)
80011dec:	03c0e825 	move	sp,s8
80011df0:	dfbe0010 	ld	s8,16(sp)
80011df4:	27bd0018 	addiu	sp,sp,24
80011df8:	03e00008 	jr	ra
80011dfc:	00000000 	nop

80011e00 <strncmp>:
80011e00:	27bdfff0 	addiu	sp,sp,-16
80011e04:	ffbe0008 	sd	s8,8(sp)
80011e08:	03a0f025 	move	s8,sp
80011e0c:	afc40010 	sw	a0,16(s8)
80011e10:	afc50018 	sw	a1,24(s8)
80011e14:	afc60020 	sw	a2,32(s8)
80011e18:	8fc20020 	lw	v0,32(s8)
80011e1c:	14400004 	bnez	v0,80011e30 <strncmp+0x30>
80011e20:	00000000 	nop
80011e24:	00001025 	move	v0,zero
80011e28:	1000005b 	b	80011f98 <strncmp+0x198>
80011e2c:	00000000 	nop
80011e30:	8fc30010 	lw	v1,16(s8)
80011e34:	8fc20018 	lw	v0,24(s8)
80011e38:	00621025 	or	v0,v1,v0
80011e3c:	30420003 	andi	v0,v0,0x3
80011e40:	14400044 	bnez	v0,80011f54 <strncmp+0x154>
80011e44:	00000000 	nop
80011e48:	8fc20010 	lw	v0,16(s8)
80011e4c:	afc20000 	sw	v0,0(s8)
80011e50:	8fc20018 	lw	v0,24(s8)
80011e54:	afc20004 	sw	v0,4(s8)
80011e58:	1000001e 	b	80011ed4 <strncmp+0xd4>
80011e5c:	00000000 	nop
80011e60:	8fc20020 	lw	v0,32(s8)
80011e64:	2442fffc 	addiu	v0,v0,-4
80011e68:	afc20020 	sw	v0,32(s8)
80011e6c:	8fc20020 	lw	v0,32(s8)
80011e70:	1040000f 	beqz	v0,80011eb0 <strncmp+0xb0>
80011e74:	00000000 	nop
80011e78:	8fc20000 	lw	v0,0(s8)
80011e7c:	8c430000 	lw	v1,0(v0)
80011e80:	3c02fefe 	lui	v0,0xfefe
80011e84:	3442feff 	ori	v0,v0,0xfeff
80011e88:	00621821 	addu	v1,v1,v0
80011e8c:	8fc20000 	lw	v0,0(s8)
80011e90:	8c420000 	lw	v0,0(v0)
80011e94:	00021027 	nor	v0,zero,v0
80011e98:	00621824 	and	v1,v1,v0
80011e9c:	3c028080 	lui	v0,0x8080
80011ea0:	34428080 	ori	v0,v0,0x8080
80011ea4:	00621024 	and	v0,v1,v0
80011ea8:	10400004 	beqz	v0,80011ebc <strncmp+0xbc>
80011eac:	00000000 	nop
80011eb0:	00001025 	move	v0,zero
80011eb4:	10000038 	b	80011f98 <strncmp+0x198>
80011eb8:	00000000 	nop
80011ebc:	8fc20000 	lw	v0,0(s8)
80011ec0:	24420004 	addiu	v0,v0,4
80011ec4:	afc20000 	sw	v0,0(s8)
80011ec8:	8fc20004 	lw	v0,4(s8)
80011ecc:	24420004 	addiu	v0,v0,4
80011ed0:	afc20004 	sw	v0,4(s8)
80011ed4:	8fc20020 	lw	v0,32(s8)
80011ed8:	2c420004 	sltiu	v0,v0,4
80011edc:	14400007 	bnez	v0,80011efc <strncmp+0xfc>
80011ee0:	00000000 	nop
80011ee4:	8fc20000 	lw	v0,0(s8)
80011ee8:	8c430000 	lw	v1,0(v0)
80011eec:	8fc20004 	lw	v0,4(s8)
80011ef0:	8c420000 	lw	v0,0(v0)
80011ef4:	1062ffda 	beq	v1,v0,80011e60 <strncmp+0x60>
80011ef8:	00000000 	nop
80011efc:	8fc20000 	lw	v0,0(s8)
80011f00:	afc20010 	sw	v0,16(s8)
80011f04:	8fc20004 	lw	v0,4(s8)
80011f08:	afc20018 	sw	v0,24(s8)
80011f0c:	10000011 	b	80011f54 <strncmp+0x154>
80011f10:	00000000 	nop
80011f14:	8fc20020 	lw	v0,32(s8)
80011f18:	10400005 	beqz	v0,80011f30 <strncmp+0x130>
80011f1c:	00000000 	nop
80011f20:	8fc20010 	lw	v0,16(s8)
80011f24:	80420000 	lb	v0,0(v0)
80011f28:	14400004 	bnez	v0,80011f3c <strncmp+0x13c>
80011f2c:	00000000 	nop
80011f30:	00001025 	move	v0,zero
80011f34:	10000018 	b	80011f98 <strncmp+0x198>
80011f38:	00000000 	nop
80011f3c:	8fc20010 	lw	v0,16(s8)
80011f40:	24420001 	addiu	v0,v0,1
80011f44:	afc20010 	sw	v0,16(s8)
80011f48:	8fc20018 	lw	v0,24(s8)
80011f4c:	24420001 	addiu	v0,v0,1
80011f50:	afc20018 	sw	v0,24(s8)
80011f54:	8fc20020 	lw	v0,32(s8)
80011f58:	2443ffff 	addiu	v1,v0,-1
80011f5c:	afc30020 	sw	v1,32(s8)
80011f60:	10400007 	beqz	v0,80011f80 <strncmp+0x180>
80011f64:	00000000 	nop
80011f68:	8fc20010 	lw	v0,16(s8)
80011f6c:	80430000 	lb	v1,0(v0)
80011f70:	8fc20018 	lw	v0,24(s8)
80011f74:	80420000 	lb	v0,0(v0)
80011f78:	1062ffe6 	beq	v1,v0,80011f14 <strncmp+0x114>
80011f7c:	00000000 	nop
80011f80:	8fc20010 	lw	v0,16(s8)
80011f84:	90420000 	lbu	v0,0(v0)
80011f88:	00401825 	move	v1,v0
80011f8c:	8fc20018 	lw	v0,24(s8)
80011f90:	90420000 	lbu	v0,0(v0)
80011f94:	00621023 	subu	v0,v1,v0
80011f98:	03c0e825 	move	sp,s8
80011f9c:	dfbe0008 	ld	s8,8(sp)
80011fa0:	27bd0010 	addiu	sp,sp,16
80011fa4:	03e00008 	jr	ra
80011fa8:	00000000 	nop

80011fac <__locale_ctype_ptr_l>:
80011fac:	27bdfff8 	addiu	sp,sp,-8
80011fb0:	ffbe0000 	sd	s8,0(sp)
80011fb4:	03a0f025 	move	s8,sp
80011fb8:	afc40008 	sw	a0,8(s8)
80011fbc:	3c028002 	lui	v0,0x8002
80011fc0:	2442af60 	addiu	v0,v0,-20640
80011fc4:	03c0e825 	move	sp,s8
80011fc8:	dfbe0000 	ld	s8,0(sp)
80011fcc:	27bd0008 	addiu	sp,sp,8
80011fd0:	03e00008 	jr	ra
80011fd4:	00000000 	nop

80011fd8 <__get_global_locale>:
80011fd8:	27bdfff8 	addiu	sp,sp,-8
80011fdc:	ffbe0000 	sd	s8,0(sp)
80011fe0:	03a0f025 	move	s8,sp
80011fe4:	3c028002 	lui	v0,0x8002
80011fe8:	2442d2f8 	addiu	v0,v0,-11528
80011fec:	03c0e825 	move	sp,s8
80011ff0:	dfbe0000 	ld	s8,0(sp)
80011ff4:	27bd0008 	addiu	sp,sp,8
80011ff8:	03e00008 	jr	ra
80011ffc:	00000000 	nop

80012000 <__get_current_locale>:
80012000:	27bdffd0 	addiu	sp,sp,-48
80012004:	ffbf0028 	sd	ra,40(sp)
80012008:	ffbe0020 	sd	s8,32(sp)
8001200c:	03a0f025 	move	s8,sp
80012010:	0c0047f6 	jal	80011fd8 <__get_global_locale>
80012014:	00000000 	nop
80012018:	03c0e825 	move	sp,s8
8001201c:	dfbf0028 	ld	ra,40(sp)
80012020:	dfbe0020 	ld	s8,32(sp)
80012024:	27bd0030 	addiu	sp,sp,48
80012028:	03e00008 	jr	ra
8001202c:	00000000 	nop

80012030 <_strtoul_l>:
80012030:	27bdff98 	addiu	sp,sp,-104
80012034:	ffbf0060 	sd	ra,96(sp)
80012038:	ffbe0058 	sd	s8,88(sp)
8001203c:	ffb60050 	sd	s6,80(sp)
80012040:	ffb50048 	sd	s5,72(sp)
80012044:	ffb40040 	sd	s4,64(sp)
80012048:	ffb30038 	sd	s3,56(sp)
8001204c:	ffb20030 	sd	s2,48(sp)
80012050:	ffb10028 	sd	s1,40(sp)
80012054:	ffb00020 	sd	s0,32(sp)
80012058:	03a0f025 	move	s8,sp
8001205c:	afc40068 	sw	a0,104(s8)
80012060:	afc50070 	sw	a1,112(s8)
80012064:	afc60078 	sw	a2,120(s8)
80012068:	afc70080 	sw	a3,128(s8)
8001206c:	8fd10070 	lw	s1,112(s8)
80012070:	0000a025 	move	s4,zero
80012074:	02201025 	move	v0,s1
80012078:	24510001 	addiu	s1,v0,1
8001207c:	90420000 	lbu	v0,0(v0)
80012080:	00408025 	move	s0,v0
80012084:	8fc4008c 	lw	a0,140(s8)
80012088:	0c0047eb 	jal	80011fac <__locale_ctype_ptr_l>
8001208c:	00000000 	nop
80012090:	00401825 	move	v1,v0
80012094:	02001025 	move	v0,s0
80012098:	24420001 	addiu	v0,v0,1
8001209c:	00621021 	addu	v0,v1,v0
800120a0:	80420000 	lb	v0,0(v0)
800120a4:	304200ff 	andi	v0,v0,0xff
800120a8:	30420008 	andi	v0,v0,0x8
800120ac:	1440fff1 	bnez	v0,80012074 <_strtoul_l+0x44>
800120b0:	00000000 	nop
800120b4:	2402002d 	li	v0,45
800120b8:	16020008 	bne	s0,v0,800120dc <_strtoul_l+0xac>
800120bc:	00000000 	nop
800120c0:	24140001 	li	s4,1
800120c4:	02201025 	move	v0,s1
800120c8:	24510001 	addiu	s1,v0,1
800120cc:	90420000 	lbu	v0,0(v0)
800120d0:	00408025 	move	s0,v0
800120d4:	10000008 	b	800120f8 <_strtoul_l+0xc8>
800120d8:	00000000 	nop
800120dc:	2402002b 	li	v0,43
800120e0:	16020005 	bne	s0,v0,800120f8 <_strtoul_l+0xc8>
800120e4:	00000000 	nop
800120e8:	02201025 	move	v0,s1
800120ec:	24510001 	addiu	s1,v0,1
800120f0:	90420000 	lbu	v0,0(v0)
800120f4:	00408025 	move	s0,v0
800120f8:	8fc20080 	lw	v0,128(s8)
800120fc:	10400005 	beqz	v0,80012114 <_strtoul_l+0xe4>
80012100:	00000000 	nop
80012104:	8fc30080 	lw	v1,128(s8)
80012108:	24020010 	li	v0,16
8001210c:	14620012 	bne	v1,v0,80012158 <_strtoul_l+0x128>
80012110:	00000000 	nop
80012114:	24020030 	li	v0,48
80012118:	1602000f 	bne	s0,v0,80012158 <_strtoul_l+0x128>
8001211c:	00000000 	nop
80012120:	92230000 	lbu	v1,0(s1)
80012124:	24020078 	li	v0,120
80012128:	10620005 	beq	v1,v0,80012140 <_strtoul_l+0x110>
8001212c:	00000000 	nop
80012130:	92230000 	lbu	v1,0(s1)
80012134:	24020058 	li	v0,88
80012138:	14620007 	bne	v1,v0,80012158 <_strtoul_l+0x128>
8001213c:	00000000 	nop
80012140:	26220001 	addiu	v0,s1,1
80012144:	90420000 	lbu	v0,0(v0)
80012148:	00408025 	move	s0,v0
8001214c:	26310002 	addiu	s1,s1,2
80012150:	24020010 	li	v0,16
80012154:	afc20080 	sw	v0,128(s8)
80012158:	8fc20080 	lw	v0,128(s8)
8001215c:	14400009 	bnez	v0,80012184 <_strtoul_l+0x154>
80012160:	00000000 	nop
80012164:	24020030 	li	v0,48
80012168:	16020004 	bne	s0,v0,8001217c <_strtoul_l+0x14c>
8001216c:	00000000 	nop
80012170:	24020008 	li	v0,8
80012174:	10000002 	b	80012180 <_strtoul_l+0x150>
80012178:	00000000 	nop
8001217c:	2402000a 	li	v0,10
80012180:	afc20080 	sw	v0,128(s8)
80012184:	8fc20080 	lw	v0,128(s8)
80012188:	2403ffff 	li	v1,-1
8001218c:	0062001b 	divu	zero,v1,v0
80012190:	004001f4 	teq	v0,zero,0x7
80012194:	00001010 	mfhi	v0
80012198:	0000a812 	mflo	s5
8001219c:	8fc20080 	lw	v0,128(s8)
800121a0:	2403ffff 	li	v1,-1
800121a4:	0062001b 	divu	zero,v1,v0
800121a8:	004001f4 	teq	v0,zero,0x7
800121ac:	00001010 	mfhi	v0
800121b0:	0040b025 	move	s6,v0
800121b4:	00009025 	move	s2,zero
800121b8:	00009825 	move	s3,zero
800121bc:	2a020030 	slti	v0,s0,48
800121c0:	14400007 	bnez	v0,800121e0 <_strtoul_l+0x1b0>
800121c4:	00000000 	nop
800121c8:	2a02003a 	slti	v0,s0,58
800121cc:	10400004 	beqz	v0,800121e0 <_strtoul_l+0x1b0>
800121d0:	00000000 	nop
800121d4:	2610ffd0 	addiu	s0,s0,-48
800121d8:	10000011 	b	80012220 <_strtoul_l+0x1f0>
800121dc:	00000000 	nop
800121e0:	2a020041 	slti	v0,s0,65
800121e4:	14400007 	bnez	v0,80012204 <_strtoul_l+0x1d4>
800121e8:	00000000 	nop
800121ec:	2a02005b 	slti	v0,s0,91
800121f0:	10400004 	beqz	v0,80012204 <_strtoul_l+0x1d4>
800121f4:	00000000 	nop
800121f8:	2610ffc9 	addiu	s0,s0,-55
800121fc:	10000008 	b	80012220 <_strtoul_l+0x1f0>
80012200:	00000000 	nop
80012204:	2a020061 	slti	v0,s0,97
80012208:	14400023 	bnez	v0,80012298 <_strtoul_l+0x268>
8001220c:	00000000 	nop
80012210:	2a02007b 	slti	v0,s0,123
80012214:	10400020 	beqz	v0,80012298 <_strtoul_l+0x268>
80012218:	00000000 	nop
8001221c:	2610ffa9 	addiu	s0,s0,-87
80012220:	8fc20080 	lw	v0,128(s8)
80012224:	0202102a 	slt	v0,s0,v0
80012228:	1040001a 	beqz	v0,80012294 <_strtoul_l+0x264>
8001222c:	00000000 	nop
80012230:	06600009 	bltz	s3,80012258 <_strtoul_l+0x228>
80012234:	00000000 	nop
80012238:	02b2102b 	sltu	v0,s5,s2
8001223c:	14400006 	bnez	v0,80012258 <_strtoul_l+0x228>
80012240:	00000000 	nop
80012244:	16550007 	bne	s2,s5,80012264 <_strtoul_l+0x234>
80012248:	00000000 	nop
8001224c:	02d0102a 	slt	v0,s6,s0
80012250:	10400004 	beqz	v0,80012264 <_strtoul_l+0x234>
80012254:	00000000 	nop
80012258:	2413ffff 	li	s3,-1
8001225c:	10000007 	b	8001227c <_strtoul_l+0x24c>
80012260:	00000000 	nop
80012264:	24130001 	li	s3,1
80012268:	8fc20080 	lw	v0,128(s8)
8001226c:	02420018 	mult	s2,v0
80012270:	00009012 	mflo	s2
80012274:	02001025 	move	v0,s0
80012278:	02429021 	addu	s2,s2,v0
8001227c:	02201025 	move	v0,s1
80012280:	24510001 	addiu	s1,v0,1
80012284:	90420000 	lbu	v0,0(v0)
80012288:	00408025 	move	s0,v0
8001228c:	1000ffcb 	b	800121bc <_strtoul_l+0x18c>
80012290:	00000000 	nop
80012294:	00000000 	nop
80012298:	06610007 	bgez	s3,800122b8 <_strtoul_l+0x288>
8001229c:	00000000 	nop
800122a0:	2412ffff 	li	s2,-1
800122a4:	8fc20068 	lw	v0,104(s8)
800122a8:	24030022 	li	v1,34
800122ac:	ac430000 	sw	v1,0(v0)
800122b0:	10000004 	b	800122c4 <_strtoul_l+0x294>
800122b4:	00000000 	nop
800122b8:	12800002 	beqz	s4,800122c4 <_strtoul_l+0x294>
800122bc:	00000000 	nop
800122c0:	00129023 	negu	s2,s2
800122c4:	8fc20078 	lw	v0,120(s8)
800122c8:	10400009 	beqz	v0,800122f0 <_strtoul_l+0x2c0>
800122cc:	00000000 	nop
800122d0:	12600004 	beqz	s3,800122e4 <_strtoul_l+0x2b4>
800122d4:	00000000 	nop
800122d8:	2622ffff 	addiu	v0,s1,-1
800122dc:	10000002 	b	800122e8 <_strtoul_l+0x2b8>
800122e0:	00000000 	nop
800122e4:	8fc20070 	lw	v0,112(s8)
800122e8:	8fc30078 	lw	v1,120(s8)
800122ec:	ac620000 	sw	v0,0(v1)
800122f0:	02401025 	move	v0,s2
800122f4:	03c0e825 	move	sp,s8
800122f8:	dfbf0060 	ld	ra,96(sp)
800122fc:	dfbe0058 	ld	s8,88(sp)
80012300:	dfb60050 	ld	s6,80(sp)
80012304:	dfb50048 	ld	s5,72(sp)
80012308:	dfb40040 	ld	s4,64(sp)
8001230c:	dfb30038 	ld	s3,56(sp)
80012310:	dfb20030 	ld	s2,48(sp)
80012314:	dfb10028 	ld	s1,40(sp)
80012318:	dfb00020 	ld	s0,32(sp)
8001231c:	27bd0068 	addiu	sp,sp,104
80012320:	03e00008 	jr	ra
80012324:	00000000 	nop

80012328 <_strtoul_r>:
80012328:	27bdffc8 	addiu	sp,sp,-56
8001232c:	ffbf0030 	sd	ra,48(sp)
80012330:	ffbe0028 	sd	s8,40(sp)
80012334:	03a0f025 	move	s8,sp
80012338:	afc40038 	sw	a0,56(s8)
8001233c:	afc50040 	sw	a1,64(s8)
80012340:	afc60048 	sw	a2,72(s8)
80012344:	afc70050 	sw	a3,80(s8)
80012348:	0c004800 	jal	80012000 <__get_current_locale>
8001234c:	00000000 	nop
80012350:	afa20024 	sw	v0,36(sp)
80012354:	8fc70050 	lw	a3,80(s8)
80012358:	8fc60048 	lw	a2,72(s8)
8001235c:	8fc50040 	lw	a1,64(s8)
80012360:	8fc40038 	lw	a0,56(s8)
80012364:	0c00480c 	jal	80012030 <_strtoul_l>
80012368:	00000000 	nop
8001236c:	03c0e825 	move	sp,s8
80012370:	dfbf0030 	ld	ra,48(sp)
80012374:	dfbe0028 	ld	s8,40(sp)
80012378:	27bd0038 	addiu	sp,sp,56
8001237c:	03e00008 	jr	ra
80012380:	00000000 	nop

80012384 <strtoul_l>:
80012384:	27bdffc8 	addiu	sp,sp,-56
80012388:	ffbf0030 	sd	ra,48(sp)
8001238c:	ffbe0028 	sd	s8,40(sp)
80012390:	03a0f025 	move	s8,sp
80012394:	afc40038 	sw	a0,56(s8)
80012398:	afc50040 	sw	a1,64(s8)
8001239c:	afc60048 	sw	a2,72(s8)
800123a0:	afc70050 	sw	a3,80(s8)
800123a4:	8f838028 	lw	v1,-32728(gp)
800123a8:	8fc20050 	lw	v0,80(s8)
800123ac:	afa20024 	sw	v0,36(sp)
800123b0:	8fc70048 	lw	a3,72(s8)
800123b4:	8fc60040 	lw	a2,64(s8)
800123b8:	8fc50038 	lw	a1,56(s8)
800123bc:	00602025 	move	a0,v1
800123c0:	0c00480c 	jal	80012030 <_strtoul_l>
800123c4:	00000000 	nop
800123c8:	03c0e825 	move	sp,s8
800123cc:	dfbf0030 	ld	ra,48(sp)
800123d0:	dfbe0028 	ld	s8,40(sp)
800123d4:	27bd0038 	addiu	sp,sp,56
800123d8:	03e00008 	jr	ra
800123dc:	00000000 	nop

800123e0 <strtoul>:
800123e0:	27bdffc0 	addiu	sp,sp,-64
800123e4:	ffbf0038 	sd	ra,56(sp)
800123e8:	ffbe0030 	sd	s8,48(sp)
800123ec:	ffb00028 	sd	s0,40(sp)
800123f0:	03a0f025 	move	s8,sp
800123f4:	afc40040 	sw	a0,64(s8)
800123f8:	afc50048 	sw	a1,72(s8)
800123fc:	afc60050 	sw	a2,80(s8)
80012400:	8f908028 	lw	s0,-32728(gp)
80012404:	0c004800 	jal	80012000 <__get_current_locale>
80012408:	00000000 	nop
8001240c:	afa20024 	sw	v0,36(sp)
80012410:	8fc70050 	lw	a3,80(s8)
80012414:	8fc60048 	lw	a2,72(s8)
80012418:	8fc50040 	lw	a1,64(s8)
8001241c:	02002025 	move	a0,s0
80012420:	0c00480c 	jal	80012030 <_strtoul_l>
80012424:	00000000 	nop
80012428:	03c0e825 	move	sp,s8
8001242c:	dfbf0038 	ld	ra,56(sp)
80012430:	dfbe0030 	ld	s8,48(sp)
80012434:	dfb00028 	ld	s0,40(sp)
80012438:	27bd0040 	addiu	sp,sp,64
8001243c:	03e00008 	jr	ra
80012440:	00000000 	nop

80012444 <__get_global_locale>:
80012444:	27bdfff8 	addiu	sp,sp,-8
80012448:	ffbe0000 	sd	s8,0(sp)
8001244c:	03a0f025 	move	s8,sp
80012450:	3c028002 	lui	v0,0x8002
80012454:	2442d2f8 	addiu	v0,v0,-11528
80012458:	03c0e825 	move	sp,s8
8001245c:	dfbe0000 	ld	s8,0(sp)
80012460:	27bd0008 	addiu	sp,sp,8
80012464:	03e00008 	jr	ra
80012468:	00000000 	nop

8001246c <__get_current_locale>:
8001246c:	27bdffd0 	addiu	sp,sp,-48
80012470:	ffbf0028 	sd	ra,40(sp)
80012474:	ffbe0020 	sd	s8,32(sp)
80012478:	03a0f025 	move	s8,sp
8001247c:	0c004911 	jal	80012444 <__get_global_locale>
80012480:	00000000 	nop
80012484:	03c0e825 	move	sp,s8
80012488:	dfbf0028 	ld	ra,40(sp)
8001248c:	dfbe0020 	ld	s8,32(sp)
80012490:	27bd0030 	addiu	sp,sp,48
80012494:	03e00008 	jr	ra
80012498:	00000000 	nop

8001249c <_sungetc_r>:
8001249c:	27bdffc8 	addiu	sp,sp,-56
800124a0:	ffbf0030 	sd	ra,48(sp)
800124a4:	ffbe0028 	sd	s8,40(sp)
800124a8:	ffb00020 	sd	s0,32(sp)
800124ac:	03a0f025 	move	s8,sp
800124b0:	afc40038 	sw	a0,56(s8)
800124b4:	afc50040 	sw	a1,64(s8)
800124b8:	00c08025 	move	s0,a2
800124bc:	8fc30040 	lw	v1,64(s8)
800124c0:	2402ffff 	li	v0,-1
800124c4:	14620004 	bne	v1,v0,800124d8 <_sungetc_r+0x3c>
800124c8:	00000000 	nop
800124cc:	2402ffff 	li	v0,-1
800124d0:	10000050 	b	80012614 <_sungetc_r+0x178>
800124d4:	00000000 	nop
800124d8:	8603000c 	lh	v1,12(s0)
800124dc:	2402ffdf 	li	v0,-33
800124e0:	00621024 	and	v0,v1,v0
800124e4:	00021400 	sll	v0,v0,0x10
800124e8:	00021403 	sra	v0,v0,0x10
800124ec:	a602000c 	sh	v0,12(s0)
800124f0:	8fc20040 	lw	v0,64(s8)
800124f4:	304200ff 	andi	v0,v0,0xff
800124f8:	afc20040 	sw	v0,64(s8)
800124fc:	8e020030 	lw	v0,48(s0)
80012500:	1040001c 	beqz	v0,80012574 <_sungetc_r+0xd8>
80012504:	00000000 	nop
80012508:	8e030004 	lw	v1,4(s0)
8001250c:	8e020034 	lw	v0,52(s0)
80012510:	0062102a 	slt	v0,v1,v0
80012514:	1440000a 	bnez	v0,80012540 <_sungetc_r+0xa4>
80012518:	00000000 	nop
8001251c:	02002825 	move	a1,s0
80012520:	8fc40038 	lw	a0,56(s8)
80012524:	0c005023 	jal	8001408c <__submore>
80012528:	00000000 	nop
8001252c:	10400004 	beqz	v0,80012540 <_sungetc_r+0xa4>
80012530:	00000000 	nop
80012534:	2402ffff 	li	v0,-1
80012538:	10000036 	b	80012614 <_sungetc_r+0x178>
8001253c:	00000000 	nop
80012540:	8e020000 	lw	v0,0(s0)
80012544:	2442ffff 	addiu	v0,v0,-1
80012548:	ae020000 	sw	v0,0(s0)
8001254c:	8e020000 	lw	v0,0(s0)
80012550:	8fc30040 	lw	v1,64(s8)
80012554:	306300ff 	andi	v1,v1,0xff
80012558:	a0430000 	sb	v1,0(v0)
8001255c:	8e020004 	lw	v0,4(s0)
80012560:	24420001 	addiu	v0,v0,1
80012564:	ae020004 	sw	v0,4(s0)
80012568:	8fc20040 	lw	v0,64(s8)
8001256c:	10000029 	b	80012614 <_sungetc_r+0x178>
80012570:	00000000 	nop
80012574:	8e020010 	lw	v0,16(s0)
80012578:	10400016 	beqz	v0,800125d4 <_sungetc_r+0x138>
8001257c:	00000000 	nop
80012580:	8e030000 	lw	v1,0(s0)
80012584:	8e020010 	lw	v0,16(s0)
80012588:	0043102b 	sltu	v0,v0,v1
8001258c:	10400011 	beqz	v0,800125d4 <_sungetc_r+0x138>
80012590:	00000000 	nop
80012594:	8e020000 	lw	v0,0(s0)
80012598:	2442ffff 	addiu	v0,v0,-1
8001259c:	90420000 	lbu	v0,0(v0)
800125a0:	00401825 	move	v1,v0
800125a4:	8fc20040 	lw	v0,64(s8)
800125a8:	1443000a 	bne	v0,v1,800125d4 <_sungetc_r+0x138>
800125ac:	00000000 	nop
800125b0:	8e020000 	lw	v0,0(s0)
800125b4:	2442ffff 	addiu	v0,v0,-1
800125b8:	ae020000 	sw	v0,0(s0)
800125bc:	8e020004 	lw	v0,4(s0)
800125c0:	24420001 	addiu	v0,v0,1
800125c4:	ae020004 	sw	v0,4(s0)
800125c8:	8fc20040 	lw	v0,64(s8)
800125cc:	10000011 	b	80012614 <_sungetc_r+0x178>
800125d0:	00000000 	nop
800125d4:	8e020004 	lw	v0,4(s0)
800125d8:	ae02003c 	sw	v0,60(s0)
800125dc:	8e020000 	lw	v0,0(s0)
800125e0:	ae020038 	sw	v0,56(s0)
800125e4:	26020040 	addiu	v0,s0,64
800125e8:	ae020030 	sw	v0,48(s0)
800125ec:	24020003 	li	v0,3
800125f0:	ae020034 	sw	v0,52(s0)
800125f4:	8fc20040 	lw	v0,64(s8)
800125f8:	304200ff 	andi	v0,v0,0xff
800125fc:	a2020042 	sb	v0,66(s0)
80012600:	26020042 	addiu	v0,s0,66
80012604:	ae020000 	sw	v0,0(s0)
80012608:	24020001 	li	v0,1
8001260c:	ae020004 	sw	v0,4(s0)
80012610:	8fc20040 	lw	v0,64(s8)
80012614:	03c0e825 	move	sp,s8
80012618:	dfbf0030 	ld	ra,48(sp)
8001261c:	dfbe0028 	ld	s8,40(sp)
80012620:	dfb00020 	ld	s0,32(sp)
80012624:	27bd0038 	addiu	sp,sp,56
80012628:	03e00008 	jr	ra
8001262c:	00000000 	nop

80012630 <__ssrefill_r>:
80012630:	27bdffc8 	addiu	sp,sp,-56
80012634:	ffbf0030 	sd	ra,48(sp)
80012638:	ffbe0028 	sd	s8,40(sp)
8001263c:	ffb00020 	sd	s0,32(sp)
80012640:	03a0f025 	move	s8,sp
80012644:	afc40038 	sw	a0,56(s8)
80012648:	00a08025 	move	s0,a1
8001264c:	8e020030 	lw	v0,48(s0)
80012650:	10400015 	beqz	v0,800126a8 <__ssrefill_r+0x78>
80012654:	00000000 	nop
80012658:	8e030030 	lw	v1,48(s0)
8001265c:	26020040 	addiu	v0,s0,64
80012660:	10620006 	beq	v1,v0,8001267c <__ssrefill_r+0x4c>
80012664:	00000000 	nop
80012668:	8e020030 	lw	v0,48(s0)
8001266c:	00402825 	move	a1,v0
80012670:	8fc40038 	lw	a0,56(s8)
80012674:	0c000dcb 	jal	8000372c <_free_r>
80012678:	00000000 	nop
8001267c:	ae000030 	sw	zero,48(s0)
80012680:	8e02003c 	lw	v0,60(s0)
80012684:	ae020004 	sw	v0,4(s0)
80012688:	8e020004 	lw	v0,4(s0)
8001268c:	10400006 	beqz	v0,800126a8 <__ssrefill_r+0x78>
80012690:	00000000 	nop
80012694:	8e020038 	lw	v0,56(s0)
80012698:	ae020000 	sw	v0,0(s0)
8001269c:	00001025 	move	v0,zero
800126a0:	1000000a 	b	800126cc <__ssrefill_r+0x9c>
800126a4:	00000000 	nop
800126a8:	8e020010 	lw	v0,16(s0)
800126ac:	ae020000 	sw	v0,0(s0)
800126b0:	ae000004 	sw	zero,4(s0)
800126b4:	8602000c 	lh	v0,12(s0)
800126b8:	34420020 	ori	v0,v0,0x20
800126bc:	00021400 	sll	v0,v0,0x10
800126c0:	00021403 	sra	v0,v0,0x10
800126c4:	a602000c 	sh	v0,12(s0)
800126c8:	2402ffff 	li	v0,-1
800126cc:	03c0e825 	move	sp,s8
800126d0:	dfbf0030 	ld	ra,48(sp)
800126d4:	dfbe0028 	ld	s8,40(sp)
800126d8:	dfb00020 	ld	s0,32(sp)
800126dc:	27bd0038 	addiu	sp,sp,56
800126e0:	03e00008 	jr	ra
800126e4:	00000000 	nop

800126e8 <_sfread_r>:
800126e8:	27bdffb0 	addiu	sp,sp,-80
800126ec:	ffbf0048 	sd	ra,72(sp)
800126f0:	ffbe0040 	sd	s8,64(sp)
800126f4:	ffb20038 	sd	s2,56(sp)
800126f8:	ffb10030 	sd	s1,48(sp)
800126fc:	ffb00028 	sd	s0,40(sp)
80012700:	03a0f025 	move	s8,sp
80012704:	afc40050 	sw	a0,80(s8)
80012708:	afc50058 	sw	a1,88(s8)
8001270c:	afc60060 	sw	a2,96(s8)
80012710:	afc70068 	sw	a3,104(s8)
80012714:	8fc30068 	lw	v1,104(s8)
80012718:	8fc20060 	lw	v0,96(s8)
8001271c:	00620018 	mult	v1,v0
80012720:	00008012 	mflo	s0
80012724:	16000004 	bnez	s0,80012738 <_sfread_r+0x50>
80012728:	00000000 	nop
8001272c:	00001025 	move	v0,zero
80012730:	10000041 	b	80012838 <_sfread_r+0x150>
80012734:	00000000 	nop
80012738:	afd00020 	sw	s0,32(s8)
8001273c:	8fd10058 	lw	s1,88(s8)
80012740:	10000024 	b	800127d4 <_sfread_r+0xec>
80012744:	00000000 	nop
80012748:	8fc20074 	lw	v0,116(s8)
8001274c:	8c420000 	lw	v0,0(v0)
80012750:	02401825 	move	v1,s2
80012754:	00603025 	move	a2,v1
80012758:	00402825 	move	a1,v0
8001275c:	02202025 	move	a0,s1
80012760:	0c0015cf 	jal	8000573c <memcpy>
80012764:	00000000 	nop
80012768:	8fc20074 	lw	v0,116(s8)
8001276c:	8c420000 	lw	v0,0(v0)
80012770:	02401825 	move	v1,s2
80012774:	00431821 	addu	v1,v0,v1
80012778:	8fc20074 	lw	v0,116(s8)
8001277c:	ac430000 	sw	v1,0(v0)
80012780:	8fc20074 	lw	v0,116(s8)
80012784:	ac400004 	sw	zero,4(v0)
80012788:	02401025 	move	v0,s2
8001278c:	02228821 	addu	s1,s1,v0
80012790:	02401025 	move	v0,s2
80012794:	02028023 	subu	s0,s0,v0
80012798:	8fc50074 	lw	a1,116(s8)
8001279c:	8fc40050 	lw	a0,80(s8)
800127a0:	0c00498c 	jal	80012630 <__ssrefill_r>
800127a4:	00000000 	nop
800127a8:	1040000a 	beqz	v0,800127d4 <_sfread_r+0xec>
800127ac:	00000000 	nop
800127b0:	8fc20020 	lw	v0,32(s8)
800127b4:	00501823 	subu	v1,v0,s0
800127b8:	8fc20060 	lw	v0,96(s8)
800127bc:	0062001b 	divu	zero,v1,v0
800127c0:	004001f4 	teq	v0,zero,0x7
800127c4:	00001010 	mfhi	v0
800127c8:	00001012 	mflo	v0
800127cc:	1000001a 	b	80012838 <_sfread_r+0x150>
800127d0:	00000000 	nop
800127d4:	8fc20074 	lw	v0,116(s8)
800127d8:	8c520004 	lw	s2,4(v0)
800127dc:	02401025 	move	v0,s2
800127e0:	0050102b 	sltu	v0,v0,s0
800127e4:	1440ffd8 	bnez	v0,80012748 <_sfread_r+0x60>
800127e8:	00000000 	nop
800127ec:	8fc20074 	lw	v0,116(s8)
800127f0:	8c420000 	lw	v0,0(v0)
800127f4:	02003025 	move	a2,s0
800127f8:	00402825 	move	a1,v0
800127fc:	02202025 	move	a0,s1
80012800:	0c0015cf 	jal	8000573c <memcpy>
80012804:	00000000 	nop
80012808:	8fc20074 	lw	v0,116(s8)
8001280c:	8c420004 	lw	v0,4(v0)
80012810:	00501023 	subu	v0,v0,s0
80012814:	00401825 	move	v1,v0
80012818:	8fc20074 	lw	v0,116(s8)
8001281c:	ac430004 	sw	v1,4(v0)
80012820:	8fc20074 	lw	v0,116(s8)
80012824:	8c420000 	lw	v0,0(v0)
80012828:	00501821 	addu	v1,v0,s0
8001282c:	8fc20074 	lw	v0,116(s8)
80012830:	ac430000 	sw	v1,0(v0)
80012834:	8fc20068 	lw	v0,104(s8)
80012838:	03c0e825 	move	sp,s8
8001283c:	dfbf0048 	ld	ra,72(sp)
80012840:	dfbe0040 	ld	s8,64(sp)
80012844:	dfb20038 	ld	s2,56(sp)
80012848:	dfb10030 	ld	s1,48(sp)
8001284c:	dfb00028 	ld	s0,40(sp)
80012850:	27bd0050 	addiu	sp,sp,80
80012854:	03e00008 	jr	ra
80012858:	00000000 	nop

8001285c <__wctob>:
8001285c:	27bdffc0 	addiu	sp,sp,-64
80012860:	ffbf0038 	sd	ra,56(sp)
80012864:	ffbe0030 	sd	s8,48(sp)
80012868:	03a0f025 	move	s8,sp
8001286c:	afc40040 	sw	a0,64(s8)
80012870:	afc50048 	sw	a1,72(s8)
80012874:	8fc30048 	lw	v1,72(s8)
80012878:	2402ffff 	li	v0,-1
8001287c:	14620004 	bne	v1,v0,80012890 <__wctob+0x34>
80012880:	00000000 	nop
80012884:	2402ffff 	li	v0,-1
80012888:	1000001b 	b	800128f8 <__wctob+0x9c>
8001288c:	00000000 	nop
80012890:	24060008 	li	a2,8
80012894:	00002825 	move	a1,zero
80012898:	27c20020 	addiu	v0,s8,32
8001289c:	00402025 	move	a0,v0
800128a0:	0c00173d 	jal	80005cf4 <memset>
800128a4:	00000000 	nop
800128a8:	0c00491b 	jal	8001246c <__get_current_locale>
800128ac:	00000000 	nop
800128b0:	8c4200e0 	lw	v0,224(v0)
800128b4:	8fc30048 	lw	v1,72(s8)
800128b8:	27c40028 	addiu	a0,s8,40
800128bc:	27c50020 	addiu	a1,s8,32
800128c0:	00a03825 	move	a3,a1
800128c4:	00603025 	move	a2,v1
800128c8:	00802825 	move	a1,a0
800128cc:	8fc40040 	lw	a0,64(s8)
800128d0:	0040f809 	jalr	v0
800128d4:	00000000 	nop
800128d8:	00401825 	move	v1,v0
800128dc:	24020001 	li	v0,1
800128e0:	14620004 	bne	v1,v0,800128f4 <__wctob+0x98>
800128e4:	00000000 	nop
800128e8:	93c20028 	lbu	v0,40(s8)
800128ec:	10000002 	b	800128f8 <__wctob+0x9c>
800128f0:	00000000 	nop
800128f4:	00001025 	move	v0,zero
800128f8:	03c0e825 	move	sp,s8
800128fc:	dfbf0038 	ld	ra,56(sp)
80012900:	dfbe0030 	ld	s8,48(sp)
80012904:	27bd0040 	addiu	sp,sp,64
80012908:	03e00008 	jr	ra
8001290c:	00000000 	nop

80012910 <__ssvfiscanf_r>:
80012910:	27bdfe00 	addiu	sp,sp,-512
80012914:	ffbf01f8 	sd	ra,504(sp)
80012918:	ffbe01f0 	sd	s8,496(sp)
8001291c:	ffb501e8 	sd	s5,488(sp)
80012920:	ffb401e0 	sd	s4,480(sp)
80012924:	ffb301d8 	sd	s3,472(sp)
80012928:	ffb201d0 	sd	s2,464(sp)
8001292c:	ffb101c8 	sd	s1,456(sp)
80012930:	ffb001c0 	sd	s0,448(sp)
80012934:	03a0f025 	move	s8,sp
80012938:	afc40200 	sw	a0,512(s8)
8001293c:	00a08025 	move	s0,a1
80012940:	afc60210 	sw	a2,528(s8)
80012944:	afc70218 	sw	a3,536(s8)
80012948:	8fd50210 	lw	s5,528(s8)
8001294c:	afc00030 	sw	zero,48(s8)
80012950:	24020001 	li	v0,1
80012954:	afc20058 	sw	v0,88(s8)
80012958:	afc00038 	sw	zero,56(s8)
8001295c:	afc0003c 	sw	zero,60(s8)
80012960:	8602000c 	lh	v0,12(s0)
80012964:	3042ffff 	andi	v0,v0,0xffff
80012968:	30422000 	andi	v0,v0,0x2000
8001296c:	1440000a 	bnez	v0,80012998 <__ssvfiscanf_r+0x88>
80012970:	00000000 	nop
80012974:	8602000c 	lh	v0,12(s0)
80012978:	34422000 	ori	v0,v0,0x2000
8001297c:	00021400 	sll	v0,v0,0x10
80012980:	00021403 	sra	v0,v0,0x10
80012984:	a602000c 	sh	v0,12(s0)
80012988:	8e030064 	lw	v1,100(s0)
8001298c:	2402dfff 	li	v0,-8193
80012990:	00621024 	and	v0,v1,v0
80012994:	ae020064 	sw	v0,100(s0)
80012998:	afc00028 	sw	zero,40(s8)
8001299c:	afc0002c 	sw	zero,44(s8)
800129a0:	92a20000 	lbu	v0,0(s5)
800129a4:	afc20078 	sw	v0,120(s8)
800129a8:	8fc20058 	lw	v0,88(s8)
800129ac:	02a2a821 	addu	s5,s5,v0
800129b0:	8fc20078 	lw	v0,120(s8)
800129b4:	10400589 	beqz	v0,80013fdc <__ssvfiscanf_r+0x16cc>
800129b8:	00000000 	nop
800129bc:	8fc30058 	lw	v1,88(s8)
800129c0:	24020001 	li	v0,1
800129c4:	1462002a 	bne	v1,v0,80012a70 <__ssvfiscanf_r+0x160>
800129c8:	00000000 	nop
800129cc:	8fc20078 	lw	v0,120(s8)
800129d0:	24430001 	addiu	v1,v0,1
800129d4:	3c028002 	lui	v0,0x8002
800129d8:	2442af60 	addiu	v0,v0,-20640
800129dc:	00621021 	addu	v0,v1,v0
800129e0:	80420000 	lb	v0,0(v0)
800129e4:	304200ff 	andi	v0,v0,0xff
800129e8:	30420008 	andi	v0,v0,0x8
800129ec:	10400020 	beqz	v0,80012a70 <__ssvfiscanf_r+0x160>
800129f0:	00000000 	nop
800129f4:	8e020004 	lw	v0,4(s0)
800129f8:	1c400007 	bgtz	v0,80012a18 <__ssvfiscanf_r+0x108>
800129fc:	00000000 	nop
80012a00:	02002825 	move	a1,s0
80012a04:	8fc40200 	lw	a0,512(s8)
80012a08:	0c00498c 	jal	80012630 <__ssrefill_r>
80012a0c:	00000000 	nop
80012a10:	14400538 	bnez	v0,80013ef4 <__ssvfiscanf_r+0x15e4>
80012a14:	00000000 	nop
80012a18:	8e020000 	lw	v0,0(s0)
80012a1c:	90420000 	lbu	v0,0(v0)
80012a20:	24430001 	addiu	v1,v0,1
80012a24:	3c028002 	lui	v0,0x8002
80012a28:	2442af60 	addiu	v0,v0,-20640
80012a2c:	00621021 	addu	v0,v1,v0
80012a30:	80420000 	lb	v0,0(v0)
80012a34:	304200ff 	andi	v0,v0,0xff
80012a38:	30420008 	andi	v0,v0,0x8
80012a3c:	1040052d 	beqz	v0,80013ef4 <__ssvfiscanf_r+0x15e4>
80012a40:	00000000 	nop
80012a44:	8fc2002c 	lw	v0,44(s8)
80012a48:	24420001 	addiu	v0,v0,1
80012a4c:	afc2002c 	sw	v0,44(s8)
80012a50:	8e020004 	lw	v0,4(s0)
80012a54:	2442ffff 	addiu	v0,v0,-1
80012a58:	ae020004 	sw	v0,4(s0)
80012a5c:	8e020000 	lw	v0,0(s0)
80012a60:	24420001 	addiu	v0,v0,1
80012a64:	ae020000 	sw	v0,0(s0)
80012a68:	1000ffe2 	b	800129f4 <__ssvfiscanf_r+0xe4>
80012a6c:	00000000 	nop
80012a70:	8fc30078 	lw	v1,120(s8)
80012a74:	24020025 	li	v0,37
80012a78:	14620012 	bne	v1,v0,80012ac4 <__ssvfiscanf_r+0x1b4>
80012a7c:	00000000 	nop
80012a80:	00009025 	move	s2,zero
80012a84:	00008825 	move	s1,zero
80012a88:	02a01025 	move	v0,s5
80012a8c:	24550001 	addiu	s5,v0,1
80012a90:	90420000 	lbu	v0,0(v0)
80012a94:	00409825 	move	s3,v0
80012a98:	2662ffdb 	addiu	v0,s3,-37
80012a9c:	2c430054 	sltiu	v1,v0,84
80012aa0:	10600551 	beqz	v1,80013fe8 <__ssvfiscanf_r+0x16d8>
80012aa4:	00000000 	nop
80012aa8:	00021880 	sll	v1,v0,0x2
80012aac:	3c028002 	lui	v0,0x8002
80012ab0:	2442a958 	addiu	v0,v0,-22184
80012ab4:	00621021 	addu	v0,v1,v0
80012ab8:	8c420000 	lw	v0,0(v0)
80012abc:	00400008 	jr	v0
80012ac0:	00000000 	nop
80012ac4:	00000000 	nop
80012ac8:	8fc20058 	lw	v0,88(s8)
80012acc:	00021023 	negu	v0,v0
80012ad0:	02a21021 	addu	v0,s5,v0
80012ad4:	afc20040 	sw	v0,64(s8)
80012ad8:	00009825 	move	s3,zero
80012adc:	1000001d 	b	80012b54 <__ssvfiscanf_r+0x244>
80012ae0:	00000000 	nop
80012ae4:	8e020004 	lw	v0,4(s0)
80012ae8:	1c400007 	bgtz	v0,80012b08 <__ssvfiscanf_r+0x1f8>
80012aec:	00000000 	nop
80012af0:	02002825 	move	a1,s0
80012af4:	8fc40200 	lw	a0,512(s8)
80012af8:	0c00498c 	jal	80012630 <__ssrefill_r>
80012afc:	00000000 	nop
80012b00:	1440050b 	bnez	v0,80013f30 <__ssvfiscanf_r+0x1620>
80012b04:	00000000 	nop
80012b08:	8e020000 	lw	v0,0(s0)
80012b0c:	90430000 	lbu	v1,0(v0)
80012b10:	8fc20040 	lw	v0,64(s8)
80012b14:	90420000 	lbu	v0,0(v0)
80012b18:	14620536 	bne	v1,v0,80013ff4 <__ssvfiscanf_r+0x16e4>
80012b1c:	00000000 	nop
80012b20:	8e020004 	lw	v0,4(s0)
80012b24:	2442ffff 	addiu	v0,v0,-1
80012b28:	ae020004 	sw	v0,4(s0)
80012b2c:	8e020000 	lw	v0,0(s0)
80012b30:	24420001 	addiu	v0,v0,1
80012b34:	ae020000 	sw	v0,0(s0)
80012b38:	8fc2002c 	lw	v0,44(s8)
80012b3c:	24420001 	addiu	v0,v0,1
80012b40:	afc2002c 	sw	v0,44(s8)
80012b44:	8fc20040 	lw	v0,64(s8)
80012b48:	24420001 	addiu	v0,v0,1
80012b4c:	afc20040 	sw	v0,64(s8)
80012b50:	26730001 	addiu	s3,s3,1
80012b54:	8fc20058 	lw	v0,88(s8)
80012b58:	0262102a 	slt	v0,s3,v0
80012b5c:	1440ffe1 	bnez	v0,80012ae4 <__ssvfiscanf_r+0x1d4>
80012b60:	00000000 	nop
80012b64:	100004f0 	b	80013f28 <__ssvfiscanf_r+0x1618>
80012b68:	00000000 	nop
80012b6c:	3222009f 	andi	v0,s1,0x9f
80012b70:	14400523 	bnez	v0,80014000 <__ssvfiscanf_r+0x16f0>
80012b74:	00000000 	nop
80012b78:	16400521 	bnez	s2,80014000 <__ssvfiscanf_r+0x16f0>
80012b7c:	00000000 	nop
80012b80:	36310010 	ori	s1,s1,0x10
80012b84:	1000ffc0 	b	80012a88 <__ssvfiscanf_r+0x178>
80012b88:	00000000 	nop
80012b8c:	3222000f 	andi	v0,s1,0xf
80012b90:	1440051e 	bnez	v0,8001400c <__ssvfiscanf_r+0x16fc>
80012b94:	00000000 	nop
80012b98:	92a30000 	lbu	v1,0(s5)
80012b9c:	2402006c 	li	v0,108
80012ba0:	14620005 	bne	v1,v0,80012bb8 <__ssvfiscanf_r+0x2a8>
80012ba4:	00000000 	nop
80012ba8:	26b50001 	addiu	s5,s5,1
80012bac:	36310002 	ori	s1,s1,0x2
80012bb0:	1000ffb5 	b	80012a88 <__ssvfiscanf_r+0x178>
80012bb4:	00000000 	nop
80012bb8:	36310001 	ori	s1,s1,0x1
80012bbc:	1000ffb2 	b	80012a88 <__ssvfiscanf_r+0x178>
80012bc0:	00000000 	nop
80012bc4:	3222000f 	andi	v0,s1,0xf
80012bc8:	14400513 	bnez	v0,80014018 <__ssvfiscanf_r+0x1708>
80012bcc:	00000000 	nop
80012bd0:	36310002 	ori	s1,s1,0x2
80012bd4:	1000ffac 	b	80012a88 <__ssvfiscanf_r+0x178>
80012bd8:	00000000 	nop
80012bdc:	3222000f 	andi	v0,s1,0xf
80012be0:	14400510 	bnez	v0,80014024 <__ssvfiscanf_r+0x1714>
80012be4:	00000000 	nop
80012be8:	36310004 	ori	s1,s1,0x4
80012bec:	1000ffa6 	b	80012a88 <__ssvfiscanf_r+0x178>
80012bf0:	00000000 	nop
80012bf4:	3222008f 	andi	v0,s1,0x8f
80012bf8:	1440050d 	bnez	v0,80014030 <__ssvfiscanf_r+0x1720>
80012bfc:	00000000 	nop
80012c00:	02401025 	move	v0,s2
80012c04:	00021080 	sll	v0,v0,0x2
80012c08:	00521021 	addu	v0,v0,s2
80012c0c:	00021040 	sll	v0,v0,0x1
80012c10:	00401825 	move	v1,v0
80012c14:	02601025 	move	v0,s3
80012c18:	00621021 	addu	v0,v1,v0
80012c1c:	2452ffd0 	addiu	s2,v0,-48
80012c20:	1000ff99 	b	80012a88 <__ssvfiscanf_r+0x178>
80012c24:	00000000 	nop
80012c28:	36310001 	ori	s1,s1,0x1
80012c2c:	24130003 	li	s3,3
80012c30:	3c028001 	lui	v0,0x8001
80012c34:	24426ec8 	addiu	v0,v0,28360
80012c38:	afc2003c 	sw	v0,60(s8)
80012c3c:	2402000a 	li	v0,10
80012c40:	afc20030 	sw	v0,48(s8)
80012c44:	10000076 	b	80012e20 <__ssvfiscanf_r+0x510>
80012c48:	00000000 	nop
80012c4c:	24130003 	li	s3,3
80012c50:	3c028001 	lui	v0,0x8001
80012c54:	24426ec8 	addiu	v0,v0,28360
80012c58:	afc2003c 	sw	v0,60(s8)
80012c5c:	afc00030 	sw	zero,48(s8)
80012c60:	1000006f 	b	80012e20 <__ssvfiscanf_r+0x510>
80012c64:	00000000 	nop
80012c68:	36310001 	ori	s1,s1,0x1
80012c6c:	24130003 	li	s3,3
80012c70:	3c028001 	lui	v0,0x8001
80012c74:	24422328 	addiu	v0,v0,9000
80012c78:	afc2003c 	sw	v0,60(s8)
80012c7c:	24020008 	li	v0,8
80012c80:	afc20030 	sw	v0,48(s8)
80012c84:	10000066 	b	80012e20 <__ssvfiscanf_r+0x510>
80012c88:	00000000 	nop
80012c8c:	24130003 	li	s3,3
80012c90:	3c028001 	lui	v0,0x8001
80012c94:	24422328 	addiu	v0,v0,9000
80012c98:	afc2003c 	sw	v0,60(s8)
80012c9c:	2402000a 	li	v0,10
80012ca0:	afc20030 	sw	v0,48(s8)
80012ca4:	1000005e 	b	80012e20 <__ssvfiscanf_r+0x510>
80012ca8:	00000000 	nop
80012cac:	36310200 	ori	s1,s1,0x200
80012cb0:	24130003 	li	s3,3
80012cb4:	3c028001 	lui	v0,0x8001
80012cb8:	24422328 	addiu	v0,v0,9000
80012cbc:	afc2003c 	sw	v0,60(s8)
80012cc0:	24020010 	li	v0,16
80012cc4:	afc20030 	sw	v0,48(s8)
80012cc8:	10000055 	b	80012e20 <__ssvfiscanf_r+0x510>
80012ccc:	00000000 	nop
80012cd0:	24130002 	li	s3,2
80012cd4:	10000052 	b	80012e20 <__ssvfiscanf_r+0x510>
80012cd8:	00000000 	nop
80012cdc:	27c20080 	addiu	v0,s8,128
80012ce0:	02a02825 	move	a1,s5
80012ce4:	00402025 	move	a0,v0
80012ce8:	0c005916 	jal	80016458 <__sccl>
80012cec:	00000000 	nop
80012cf0:	0040a825 	move	s5,v0
80012cf4:	36310040 	ori	s1,s1,0x40
80012cf8:	24130001 	li	s3,1
80012cfc:	10000048 	b	80012e20 <__ssvfiscanf_r+0x510>
80012d00:	00000000 	nop
80012d04:	36310040 	ori	s1,s1,0x40
80012d08:	00009825 	move	s3,zero
80012d0c:	10000044 	b	80012e20 <__ssvfiscanf_r+0x510>
80012d10:	00000000 	nop
80012d14:	36310220 	ori	s1,s1,0x220
80012d18:	24130003 	li	s3,3
80012d1c:	3c028001 	lui	v0,0x8001
80012d20:	24422328 	addiu	v0,v0,9000
80012d24:	afc2003c 	sw	v0,60(s8)
80012d28:	24020010 	li	v0,16
80012d2c:	afc20030 	sw	v0,48(s8)
80012d30:	1000003b 	b	80012e20 <__ssvfiscanf_r+0x510>
80012d34:	00000000 	nop
80012d38:	32220010 	andi	v0,s1,0x10
80012d3c:	14400470 	bnez	v0,80013f00 <__ssvfiscanf_r+0x15f0>
80012d40:	00000000 	nop
80012d44:	32220004 	andi	v0,s1,0x4
80012d48:	1040000e 	beqz	v0,80012d84 <__ssvfiscanf_r+0x474>
80012d4c:	00000000 	nop
80012d50:	8fc20218 	lw	v0,536(s8)
80012d54:	24430008 	addiu	v1,v0,8
80012d58:	afc30218 	sw	v1,536(s8)
80012d5c:	24420004 	addiu	v0,v0,4
80012d60:	8c420000 	lw	v0,0(v0)
80012d64:	afc20068 	sw	v0,104(s8)
80012d68:	8fc2002c 	lw	v0,44(s8)
80012d6c:	00021c00 	sll	v1,v0,0x10
80012d70:	00031c03 	sra	v1,v1,0x10
80012d74:	8fc20068 	lw	v0,104(s8)
80012d78:	a4430000 	sh	v1,0(v0)
80012d7c:	1000046a 	b	80013f28 <__ssvfiscanf_r+0x1618>
80012d80:	00000000 	nop
80012d84:	32220001 	andi	v0,s1,0x1
80012d88:	1040000c 	beqz	v0,80012dbc <__ssvfiscanf_r+0x4ac>
80012d8c:	00000000 	nop
80012d90:	8fc20218 	lw	v0,536(s8)
80012d94:	24430008 	addiu	v1,v0,8
80012d98:	afc30218 	sw	v1,536(s8)
80012d9c:	24420004 	addiu	v0,v0,4
80012da0:	8c420000 	lw	v0,0(v0)
80012da4:	afc20064 	sw	v0,100(s8)
80012da8:	8fc20064 	lw	v0,100(s8)
80012dac:	8fc3002c 	lw	v1,44(s8)
80012db0:	ac430000 	sw	v1,0(v0)
80012db4:	1000045c 	b	80013f28 <__ssvfiscanf_r+0x1618>
80012db8:	00000000 	nop
80012dbc:	32220002 	andi	v0,s1,0x2
80012dc0:	1040000c 	beqz	v0,80012df4 <__ssvfiscanf_r+0x4e4>
80012dc4:	00000000 	nop
80012dc8:	8fc20218 	lw	v0,536(s8)
80012dcc:	24430008 	addiu	v1,v0,8
80012dd0:	afc30218 	sw	v1,536(s8)
80012dd4:	24420004 	addiu	v0,v0,4
80012dd8:	8c420000 	lw	v0,0(v0)
80012ddc:	afc20060 	sw	v0,96(s8)
80012de0:	8fc3002c 	lw	v1,44(s8)
80012de4:	8fc20060 	lw	v0,96(s8)
80012de8:	fc430000 	sd	v1,0(v0)
80012dec:	1000044e 	b	80013f28 <__ssvfiscanf_r+0x1618>
80012df0:	00000000 	nop
80012df4:	8fc20218 	lw	v0,536(s8)
80012df8:	24430008 	addiu	v1,v0,8
80012dfc:	afc30218 	sw	v1,536(s8)
80012e00:	24420004 	addiu	v0,v0,4
80012e04:	8c420000 	lw	v0,0(v0)
80012e08:	afc2005c 	sw	v0,92(s8)
80012e0c:	8fc2005c 	lw	v0,92(s8)
80012e10:	8fc3002c 	lw	v1,44(s8)
80012e14:	ac430000 	sw	v1,0(v0)
80012e18:	10000443 	b	80013f28 <__ssvfiscanf_r+0x1618>
80012e1c:	00000000 	nop
80012e20:	8e020004 	lw	v0,4(s0)
80012e24:	1c400007 	bgtz	v0,80012e44 <__ssvfiscanf_r+0x534>
80012e28:	00000000 	nop
80012e2c:	02002825 	move	a1,s0
80012e30:	8fc40200 	lw	a0,512(s8)
80012e34:	0c00498c 	jal	80012630 <__ssrefill_r>
80012e38:	00000000 	nop
80012e3c:	1440043f 	bnez	v0,80013f3c <__ssvfiscanf_r+0x162c>
80012e40:	00000000 	nop
80012e44:	32220040 	andi	v0,s1,0x40
80012e48:	14400022 	bnez	v0,80012ed4 <__ssvfiscanf_r+0x5c4>
80012e4c:	00000000 	nop
80012e50:	10000015 	b	80012ea8 <__ssvfiscanf_r+0x598>
80012e54:	00000000 	nop
80012e58:	8fc2002c 	lw	v0,44(s8)
80012e5c:	24420001 	addiu	v0,v0,1
80012e60:	afc2002c 	sw	v0,44(s8)
80012e64:	8e020004 	lw	v0,4(s0)
80012e68:	2442ffff 	addiu	v0,v0,-1
80012e6c:	ae020004 	sw	v0,4(s0)
80012e70:	8e020004 	lw	v0,4(s0)
80012e74:	18400006 	blez	v0,80012e90 <__ssvfiscanf_r+0x580>
80012e78:	00000000 	nop
80012e7c:	8e020000 	lw	v0,0(s0)
80012e80:	24420001 	addiu	v0,v0,1
80012e84:	ae020000 	sw	v0,0(s0)
80012e88:	10000007 	b	80012ea8 <__ssvfiscanf_r+0x598>
80012e8c:	00000000 	nop
80012e90:	02002825 	move	a1,s0
80012e94:	8fc40200 	lw	a0,512(s8)
80012e98:	0c00498c 	jal	80012630 <__ssrefill_r>
80012e9c:	00000000 	nop
80012ea0:	14400429 	bnez	v0,80013f48 <__ssvfiscanf_r+0x1638>
80012ea4:	00000000 	nop
80012ea8:	8e020000 	lw	v0,0(s0)
80012eac:	90420000 	lbu	v0,0(v0)
80012eb0:	24430001 	addiu	v1,v0,1
80012eb4:	3c028002 	lui	v0,0x8002
80012eb8:	2442af60 	addiu	v0,v0,-20640
80012ebc:	00621021 	addu	v0,v1,v0
80012ec0:	80420000 	lb	v0,0(v0)
80012ec4:	304200ff 	andi	v0,v0,0xff
80012ec8:	30420008 	andi	v0,v0,0x8
80012ecc:	1440ffe2 	bnez	v0,80012e58 <__ssvfiscanf_r+0x548>
80012ed0:	00000000 	nop
80012ed4:	24020003 	li	v0,3
80012ed8:	126202e9 	beq	s3,v0,80013a80 <__ssvfiscanf_r+0x1170>
80012edc:	00000000 	nop
80012ee0:	2a620004 	slti	v0,s3,4
80012ee4:	1040feae 	beqz	v0,800129a0 <__ssvfiscanf_r+0x90>
80012ee8:	00000000 	nop
80012eec:	24020002 	li	v0,2
80012ef0:	126201d9 	beq	s3,v0,80013658 <__ssvfiscanf_r+0xd48>
80012ef4:	00000000 	nop
80012ef8:	2a620003 	slti	v0,s3,3
80012efc:	1040fea8 	beqz	v0,800129a0 <__ssvfiscanf_r+0x90>
80012f00:	00000000 	nop
80012f04:	12600006 	beqz	s3,80012f20 <__ssvfiscanf_r+0x610>
80012f08:	00000000 	nop
80012f0c:	24020001 	li	v0,1
80012f10:	126200ca 	beq	s3,v0,8001323c <__ssvfiscanf_r+0x92c>
80012f14:	00000000 	nop
80012f18:	10000403 	b	80013f28 <__ssvfiscanf_r+0x1618>
80012f1c:	00000000 	nop
80012f20:	16400002 	bnez	s2,80012f2c <__ssvfiscanf_r+0x61c>
80012f24:	00000000 	nop
80012f28:	24120001 	li	s2,1
80012f2c:	32220001 	andi	v0,s1,0x1
80012f30:	10400079 	beqz	v0,80013118 <__ssvfiscanf_r+0x808>
80012f34:	00000000 	nop
80012f38:	32220010 	andi	v0,s1,0x10
80012f3c:	10400004 	beqz	v0,80012f50 <__ssvfiscanf_r+0x640>
80012f40:	00000000 	nop
80012f44:	afc00034 	sw	zero,52(s8)
80012f48:	10000007 	b	80012f68 <__ssvfiscanf_r+0x658>
80012f4c:	00000000 	nop
80012f50:	8fc20218 	lw	v0,536(s8)
80012f54:	24430008 	addiu	v1,v0,8
80012f58:	afc30218 	sw	v1,536(s8)
80012f5c:	24420004 	addiu	v0,v0,4
80012f60:	8c420000 	lw	v0,0(v0)
80012f64:	afc20034 	sw	v0,52(s8)
80012f68:	00009825 	move	s3,zero
80012f6c:	1000005d 	b	800130e4 <__ssvfiscanf_r+0x7d4>
80012f70:	00000000 	nop
80012f74:	0c003a42 	jal	8000e908 <__locale_mb_cur_max>
80012f78:	00000000 	nop
80012f7c:	126203f5 	beq	s3,v0,80013f54 <__ssvfiscanf_r+0x1644>
80012f80:	00000000 	nop
80012f84:	8e020000 	lw	v0,0(s0)
80012f88:	90430000 	lbu	v1,0(v0)
80012f8c:	02601025 	move	v0,s3
80012f90:	24530001 	addiu	s3,v0,1
80012f94:	00031e00 	sll	v1,v1,0x18
80012f98:	00031e03 	sra	v1,v1,0x18
80012f9c:	27c40028 	addiu	a0,s8,40
80012fa0:	00821021 	addu	v0,a0,v0
80012fa4:	a0430158 	sb	v1,344(v0)
80012fa8:	8e020004 	lw	v0,4(s0)
80012fac:	2442ffff 	addiu	v0,v0,-1
80012fb0:	ae020004 	sw	v0,4(s0)
80012fb4:	8e020000 	lw	v0,0(s0)
80012fb8:	24420001 	addiu	v0,v0,1
80012fbc:	ae020000 	sw	v0,0(s0)
80012fc0:	8fc30038 	lw	v1,56(s8)
80012fc4:	24020003 	li	v0,3
80012fc8:	14620005 	bne	v1,v0,80012fe0 <__ssvfiscanf_r+0x6d0>
80012fcc:	00000000 	nop
80012fd0:	8fc301a8 	lw	v1,424(s8)
80012fd4:	24020004 	li	v0,4
80012fd8:	10620007 	beq	v1,v0,80012ff8 <__ssvfiscanf_r+0x6e8>
80012fdc:	00000000 	nop
80012fe0:	27c201a8 	addiu	v0,s8,424
80012fe4:	24060008 	li	a2,8
80012fe8:	00002825 	move	a1,zero
80012fec:	00402025 	move	a0,v0
80012ff0:	0c00173d 	jal	80005cf4 <memset>
80012ff4:	00000000 	nop
80012ff8:	02601825 	move	v1,s3
80012ffc:	27c40180 	addiu	a0,s8,384
80013000:	27c201a8 	addiu	v0,s8,424
80013004:	afa20024 	sw	v0,36(sp)
80013008:	00603825 	move	a3,v1
8001300c:	00803025 	move	a2,a0
80013010:	8fc50034 	lw	a1,52(s8)
80013014:	8fc40200 	lw	a0,512(s8)
80013018:	0c0058a0 	jal	80016280 <_mbrtowc_r>
8001301c:	00000000 	nop
80013020:	afc20038 	sw	v0,56(s8)
80013024:	8fc30038 	lw	v1,56(s8)
80013028:	2402ffff 	li	v0,-1
8001302c:	106203cc 	beq	v1,v0,80013f60 <__ssvfiscanf_r+0x1650>
80013030:	00000000 	nop
80013034:	8fc20038 	lw	v0,56(s8)
80013038:	14400006 	bnez	v0,80013054 <__ssvfiscanf_r+0x744>
8001303c:	00000000 	nop
80013040:	32220010 	andi	v0,s1,0x10
80013044:	14400003 	bnez	v0,80013054 <__ssvfiscanf_r+0x744>
80013048:	00000000 	nop
8001304c:	8fc20034 	lw	v0,52(s8)
80013050:	ac400000 	sw	zero,0(v0)
80013054:	8fc30038 	lw	v1,56(s8)
80013058:	2402fffe 	li	v0,-2
8001305c:	10620014 	beq	v1,v0,800130b0 <__ssvfiscanf_r+0x7a0>
80013060:	00000000 	nop
80013064:	8fc2002c 	lw	v0,44(s8)
80013068:	00531021 	addu	v0,v0,s3
8001306c:	afc2002c 	sw	v0,44(s8)
80013070:	8fc30038 	lw	v1,56(s8)
80013074:	24020003 	li	v0,3
80013078:	14620005 	bne	v1,v0,80013090 <__ssvfiscanf_r+0x780>
8001307c:	00000000 	nop
80013080:	8fc301a8 	lw	v1,424(s8)
80013084:	24020004 	li	v0,4
80013088:	10620002 	beq	v1,v0,80013094 <__ssvfiscanf_r+0x784>
8001308c:	00000000 	nop
80013090:	2652ffff 	addiu	s2,s2,-1
80013094:	32220010 	andi	v0,s1,0x10
80013098:	14400004 	bnez	v0,800130ac <__ssvfiscanf_r+0x79c>
8001309c:	00000000 	nop
800130a0:	8fc20034 	lw	v0,52(s8)
800130a4:	24420004 	addiu	v0,v0,4
800130a8:	afc20034 	sw	v0,52(s8)
800130ac:	00009825 	move	s3,zero
800130b0:	8e020004 	lw	v0,4(s0)
800130b4:	1c40000b 	bgtz	v0,800130e4 <__ssvfiscanf_r+0x7d4>
800130b8:	00000000 	nop
800130bc:	02002825 	move	a1,s0
800130c0:	8fc40200 	lw	a0,512(s8)
800130c4:	0c00498c 	jal	80012630 <__ssrefill_r>
800130c8:	00000000 	nop
800130cc:	10400005 	beqz	v0,800130e4 <__ssvfiscanf_r+0x7d4>
800130d0:	00000000 	nop
800130d4:	12600007 	beqz	s3,800130f4 <__ssvfiscanf_r+0x7e4>
800130d8:	00000000 	nop
800130dc:	100003b0 	b	80013fa0 <__ssvfiscanf_r+0x1690>
800130e0:	00000000 	nop
800130e4:	1640ffa3 	bnez	s2,80012f74 <__ssvfiscanf_r+0x664>
800130e8:	00000000 	nop
800130ec:	10000002 	b	800130f8 <__ssvfiscanf_r+0x7e8>
800130f0:	00000000 	nop
800130f4:	00000000 	nop
800130f8:	32220010 	andi	v0,s1,0x10
800130fc:	14400383 	bnez	v0,80013f0c <__ssvfiscanf_r+0x15fc>
80013100:	00000000 	nop
80013104:	8fc20028 	lw	v0,40(s8)
80013108:	24420001 	addiu	v0,v0,1
8001310c:	afc20028 	sw	v0,40(s8)
80013110:	1000037e 	b	80013f0c <__ssvfiscanf_r+0x15fc>
80013114:	00000000 	nop
80013118:	32220010 	andi	v0,s1,0x10
8001311c:	1040002e 	beqz	v0,800131d8 <__ssvfiscanf_r+0x8c8>
80013120:	00000000 	nop
80013124:	afc00044 	sw	zero,68(s8)
80013128:	8e130004 	lw	s3,4(s0)
8001312c:	02401025 	move	v0,s2
80013130:	0262102a 	slt	v0,s3,v0
80013134:	10400016 	beqz	v0,80013190 <__ssvfiscanf_r+0x880>
80013138:	00000000 	nop
8001313c:	02601825 	move	v1,s3
80013140:	8fc20044 	lw	v0,68(s8)
80013144:	00431021 	addu	v0,v0,v1
80013148:	afc20044 	sw	v0,68(s8)
8001314c:	02601025 	move	v0,s3
80013150:	02429023 	subu	s2,s2,v0
80013154:	8e020000 	lw	v0,0(s0)
80013158:	02601825 	move	v1,s3
8001315c:	00431021 	addu	v0,v0,v1
80013160:	ae020000 	sw	v0,0(s0)
80013164:	02002825 	move	a1,s0
80013168:	8fc40200 	lw	a0,512(s8)
8001316c:	0c00498c 	jal	80012630 <__ssrefill_r>
80013170:	00000000 	nop
80013174:	1040ffec 	beqz	v0,80013128 <__ssvfiscanf_r+0x818>
80013178:	00000000 	nop
8001317c:	8fc20044 	lw	v0,68(s8)
80013180:	1440000e 	bnez	v0,800131bc <__ssvfiscanf_r+0x8ac>
80013184:	00000000 	nop
80013188:	10000385 	b	80013fa0 <__ssvfiscanf_r+0x1690>
8001318c:	00000000 	nop
80013190:	8fc20044 	lw	v0,68(s8)
80013194:	00521021 	addu	v0,v0,s2
80013198:	afc20044 	sw	v0,68(s8)
8001319c:	8e020004 	lw	v0,4(s0)
800131a0:	00521023 	subu	v0,v0,s2
800131a4:	ae020004 	sw	v0,4(s0)
800131a8:	8e020000 	lw	v0,0(s0)
800131ac:	00521021 	addu	v0,v0,s2
800131b0:	ae020000 	sw	v0,0(s0)
800131b4:	10000002 	b	800131c0 <__ssvfiscanf_r+0x8b0>
800131b8:	00000000 	nop
800131bc:	00000000 	nop
800131c0:	8fc3002c 	lw	v1,44(s8)
800131c4:	8fc20044 	lw	v0,68(s8)
800131c8:	00621021 	addu	v0,v1,v0
800131cc:	afc2002c 	sw	v0,44(s8)
800131d0:	1000034e 	b	80013f0c <__ssvfiscanf_r+0x15fc>
800131d4:	00000000 	nop
800131d8:	8fc20218 	lw	v0,536(s8)
800131dc:	24430008 	addiu	v1,v0,8
800131e0:	afc30218 	sw	v1,536(s8)
800131e4:	24420004 	addiu	v0,v0,4
800131e8:	8c540000 	lw	s4,0(v0)
800131ec:	afb00024 	sw	s0,36(sp)
800131f0:	02403825 	move	a3,s2
800131f4:	24060001 	li	a2,1
800131f8:	02802825 	move	a1,s4
800131fc:	8fc40200 	lw	a0,512(s8)
80013200:	0c0049ba 	jal	800126e8 <_sfread_r>
80013204:	00000000 	nop
80013208:	afc20074 	sw	v0,116(s8)
8001320c:	8fc20074 	lw	v0,116(s8)
80013210:	10400356 	beqz	v0,80013f6c <__ssvfiscanf_r+0x165c>
80013214:	00000000 	nop
80013218:	8fc3002c 	lw	v1,44(s8)
8001321c:	8fc20074 	lw	v0,116(s8)
80013220:	00621021 	addu	v0,v1,v0
80013224:	afc2002c 	sw	v0,44(s8)
80013228:	8fc20028 	lw	v0,40(s8)
8001322c:	24420001 	addiu	v0,v0,1
80013230:	afc20028 	sw	v0,40(s8)
80013234:	10000335 	b	80013f0c <__ssvfiscanf_r+0x15fc>
80013238:	00000000 	nop
8001323c:	16400002 	bnez	s2,80013248 <__ssvfiscanf_r+0x938>
80013240:	00000000 	nop
80013244:	2412ffff 	li	s2,-1
80013248:	32220001 	andi	v0,s1,0x1
8001324c:	10400094 	beqz	v0,800134a0 <__ssvfiscanf_r+0xb90>
80013250:	00000000 	nop
80013254:	32220010 	andi	v0,s1,0x10
80013258:	10400005 	beqz	v0,80013270 <__ssvfiscanf_r+0x960>
8001325c:	00000000 	nop
80013260:	27c20078 	addiu	v0,s8,120
80013264:	afc20034 	sw	v0,52(s8)
80013268:	10000007 	b	80013288 <__ssvfiscanf_r+0x978>
8001326c:	00000000 	nop
80013270:	8fc20218 	lw	v0,536(s8)
80013274:	24430008 	addiu	v1,v0,8
80013278:	afc30218 	sw	v1,536(s8)
8001327c:	24420004 	addiu	v0,v0,4
80013280:	8c420000 	lw	v0,0(v0)
80013284:	afc20034 	sw	v0,52(s8)
80013288:	00009825 	move	s3,zero
8001328c:	10000075 	b	80013464 <__ssvfiscanf_r+0xb54>
80013290:	00000000 	nop
80013294:	0c003a42 	jal	8000e908 <__locale_mb_cur_max>
80013298:	00000000 	nop
8001329c:	12620336 	beq	s3,v0,80013f78 <__ssvfiscanf_r+0x1668>
800132a0:	00000000 	nop
800132a4:	8e020000 	lw	v0,0(s0)
800132a8:	90430000 	lbu	v1,0(v0)
800132ac:	02601025 	move	v0,s3
800132b0:	24530001 	addiu	s3,v0,1
800132b4:	00031e00 	sll	v1,v1,0x18
800132b8:	00031e03 	sra	v1,v1,0x18
800132bc:	27c40028 	addiu	a0,s8,40
800132c0:	00821021 	addu	v0,a0,v0
800132c4:	a0430158 	sb	v1,344(v0)
800132c8:	8e020004 	lw	v0,4(s0)
800132cc:	2442ffff 	addiu	v0,v0,-1
800132d0:	ae020004 	sw	v0,4(s0)
800132d4:	8e020000 	lw	v0,0(s0)
800132d8:	24420001 	addiu	v0,v0,1
800132dc:	ae020000 	sw	v0,0(s0)
800132e0:	8fc30038 	lw	v1,56(s8)
800132e4:	24020003 	li	v0,3
800132e8:	14620005 	bne	v1,v0,80013300 <__ssvfiscanf_r+0x9f0>
800132ec:	00000000 	nop
800132f0:	8fc301b0 	lw	v1,432(s8)
800132f4:	24020004 	li	v0,4
800132f8:	10620007 	beq	v1,v0,80013318 <__ssvfiscanf_r+0xa08>
800132fc:	00000000 	nop
80013300:	27c201b0 	addiu	v0,s8,432
80013304:	24060008 	li	a2,8
80013308:	00002825 	move	a1,zero
8001330c:	00402025 	move	a0,v0
80013310:	0c00173d 	jal	80005cf4 <memset>
80013314:	00000000 	nop
80013318:	02601825 	move	v1,s3
8001331c:	27c40180 	addiu	a0,s8,384
80013320:	27c201b0 	addiu	v0,s8,432
80013324:	afa20024 	sw	v0,36(sp)
80013328:	00603825 	move	a3,v1
8001332c:	00803025 	move	a2,a0
80013330:	8fc50034 	lw	a1,52(s8)
80013334:	8fc40200 	lw	a0,512(s8)
80013338:	0c0058a0 	jal	80016280 <_mbrtowc_r>
8001333c:	00000000 	nop
80013340:	afc20038 	sw	v0,56(s8)
80013344:	8fc30038 	lw	v1,56(s8)
80013348:	2402ffff 	li	v0,-1
8001334c:	1062030d 	beq	v1,v0,80013f84 <__ssvfiscanf_r+0x1674>
80013350:	00000000 	nop
80013354:	8fc20038 	lw	v0,56(s8)
80013358:	14400003 	bnez	v0,80013368 <__ssvfiscanf_r+0xa58>
8001335c:	00000000 	nop
80013360:	8fc20034 	lw	v0,52(s8)
80013364:	ac400000 	sw	zero,0(v0)
80013368:	8fc30038 	lw	v1,56(s8)
8001336c:	2402fffe 	li	v0,-2
80013370:	1062002f 	beq	v1,v0,80013430 <__ssvfiscanf_r+0xb20>
80013374:	00000000 	nop
80013378:	8fc20034 	lw	v0,52(s8)
8001337c:	8c420000 	lw	v0,0(v0)
80013380:	00402825 	move	a1,v0
80013384:	8fc40200 	lw	a0,512(s8)
80013388:	0c004a17 	jal	8001285c <__wctob>
8001338c:	00000000 	nop
80013390:	27c30028 	addiu	v1,s8,40
80013394:	00621021 	addu	v0,v1,v0
80013398:	80420058 	lb	v0,88(v0)
8001339c:	14400011 	bnez	v0,800133e4 <__ssvfiscanf_r+0xad4>
800133a0:	00000000 	nop
800133a4:	1000000b 	b	800133d4 <__ssvfiscanf_r+0xac4>
800133a8:	00000000 	nop
800133ac:	2673ffff 	addiu	s3,s3,-1
800133b0:	27c20028 	addiu	v0,s8,40
800133b4:	00531021 	addu	v0,v0,s3
800133b8:	80420158 	lb	v0,344(v0)
800133bc:	304200ff 	andi	v0,v0,0xff
800133c0:	02003025 	move	a2,s0
800133c4:	00402825 	move	a1,v0
800133c8:	8fc40200 	lw	a0,512(s8)
800133cc:	0c004927 	jal	8001249c <_sungetc_r>
800133d0:	00000000 	nop
800133d4:	1660fff5 	bnez	s3,800133ac <__ssvfiscanf_r+0xa9c>
800133d8:	00000000 	nop
800133dc:	10000026 	b	80013478 <__ssvfiscanf_r+0xb68>
800133e0:	00000000 	nop
800133e4:	8fc2002c 	lw	v0,44(s8)
800133e8:	00531021 	addu	v0,v0,s3
800133ec:	afc2002c 	sw	v0,44(s8)
800133f0:	8fc30038 	lw	v1,56(s8)
800133f4:	24020003 	li	v0,3
800133f8:	14620005 	bne	v1,v0,80013410 <__ssvfiscanf_r+0xb00>
800133fc:	00000000 	nop
80013400:	8fc301b0 	lw	v1,432(s8)
80013404:	24020004 	li	v0,4
80013408:	10620002 	beq	v1,v0,80013414 <__ssvfiscanf_r+0xb04>
8001340c:	00000000 	nop
80013410:	2652ffff 	addiu	s2,s2,-1
80013414:	32220010 	andi	v0,s1,0x10
80013418:	14400004 	bnez	v0,8001342c <__ssvfiscanf_r+0xb1c>
8001341c:	00000000 	nop
80013420:	8fc20034 	lw	v0,52(s8)
80013424:	24420004 	addiu	v0,v0,4
80013428:	afc20034 	sw	v0,52(s8)
8001342c:	00009825 	move	s3,zero
80013430:	8e020004 	lw	v0,4(s0)
80013434:	1c40000b 	bgtz	v0,80013464 <__ssvfiscanf_r+0xb54>
80013438:	00000000 	nop
8001343c:	02002825 	move	a1,s0
80013440:	8fc40200 	lw	a0,512(s8)
80013444:	0c00498c 	jal	80012630 <__ssrefill_r>
80013448:	00000000 	nop
8001344c:	10400005 	beqz	v0,80013464 <__ssvfiscanf_r+0xb54>
80013450:	00000000 	nop
80013454:	12600007 	beqz	s3,80013474 <__ssvfiscanf_r+0xb64>
80013458:	00000000 	nop
8001345c:	100002d0 	b	80013fa0 <__ssvfiscanf_r+0x1690>
80013460:	00000000 	nop
80013464:	1640ff8b 	bnez	s2,80013294 <__ssvfiscanf_r+0x984>
80013468:	00000000 	nop
8001346c:	10000002 	b	80013478 <__ssvfiscanf_r+0xb68>
80013470:	00000000 	nop
80013474:	00000000 	nop
80013478:	32220010 	andi	v0,s1,0x10
8001347c:	144002a6 	bnez	v0,80013f18 <__ssvfiscanf_r+0x1608>
80013480:	00000000 	nop
80013484:	8fc20034 	lw	v0,52(s8)
80013488:	ac400000 	sw	zero,0(v0)
8001348c:	8fc20028 	lw	v0,40(s8)
80013490:	24420001 	addiu	v0,v0,1
80013494:	afc20028 	sw	v0,40(s8)
80013498:	1000029f 	b	80013f18 <__ssvfiscanf_r+0x1608>
8001349c:	00000000 	nop
800134a0:	32220010 	andi	v0,s1,0x10
800134a4:	1040002f 	beqz	v0,80013564 <__ssvfiscanf_r+0xc54>
800134a8:	00000000 	nop
800134ac:	00009825 	move	s3,zero
800134b0:	10000018 	b	80013514 <__ssvfiscanf_r+0xc04>
800134b4:	00000000 	nop
800134b8:	26730001 	addiu	s3,s3,1
800134bc:	8e020004 	lw	v0,4(s0)
800134c0:	2442ffff 	addiu	v0,v0,-1
800134c4:	ae020004 	sw	v0,4(s0)
800134c8:	8e020000 	lw	v0,0(s0)
800134cc:	24420001 	addiu	v0,v0,1
800134d0:	ae020000 	sw	v0,0(s0)
800134d4:	2652ffff 	addiu	s2,s2,-1
800134d8:	12400017 	beqz	s2,80013538 <__ssvfiscanf_r+0xc28>
800134dc:	00000000 	nop
800134e0:	8e020004 	lw	v0,4(s0)
800134e4:	1c40000b 	bgtz	v0,80013514 <__ssvfiscanf_r+0xc04>
800134e8:	00000000 	nop
800134ec:	02002825 	move	a1,s0
800134f0:	8fc40200 	lw	a0,512(s8)
800134f4:	0c00498c 	jal	80012630 <__ssrefill_r>
800134f8:	00000000 	nop
800134fc:	10400005 	beqz	v0,80013514 <__ssvfiscanf_r+0xc04>
80013500:	00000000 	nop
80013504:	1660000f 	bnez	s3,80013544 <__ssvfiscanf_r+0xc34>
80013508:	00000000 	nop
8001350c:	100002a4 	b	80013fa0 <__ssvfiscanf_r+0x1690>
80013510:	00000000 	nop
80013514:	8e020000 	lw	v0,0(s0)
80013518:	90420000 	lbu	v0,0(v0)
8001351c:	27c30028 	addiu	v1,s8,40
80013520:	00621021 	addu	v0,v1,v0
80013524:	80420058 	lb	v0,88(v0)
80013528:	1440ffe3 	bnez	v0,800134b8 <__ssvfiscanf_r+0xba8>
8001352c:	00000000 	nop
80013530:	10000005 	b	80013548 <__ssvfiscanf_r+0xc38>
80013534:	00000000 	nop
80013538:	00000000 	nop
8001353c:	10000002 	b	80013548 <__ssvfiscanf_r+0xc38>
80013540:	00000000 	nop
80013544:	00000000 	nop
80013548:	126002bc 	beqz	s3,8001403c <__ssvfiscanf_r+0x172c>
8001354c:	00000000 	nop
80013550:	8fc2002c 	lw	v0,44(s8)
80013554:	00531021 	addu	v0,v0,s3
80013558:	afc2002c 	sw	v0,44(s8)
8001355c:	1000026e 	b	80013f18 <__ssvfiscanf_r+0x1608>
80013560:	00000000 	nop
80013564:	8fc20218 	lw	v0,536(s8)
80013568:	24430008 	addiu	v1,v0,8
8001356c:	afc30218 	sw	v1,536(s8)
80013570:	24420004 	addiu	v0,v0,4
80013574:	8c540000 	lw	s4,0(v0)
80013578:	02808825 	move	s1,s4
8001357c:	1000001d 	b	800135f4 <__ssvfiscanf_r+0xce4>
80013580:	00000000 	nop
80013584:	8e020004 	lw	v0,4(s0)
80013588:	2442ffff 	addiu	v0,v0,-1
8001358c:	ae020004 	sw	v0,4(s0)
80013590:	8e020000 	lw	v0,0(s0)
80013594:	24430001 	addiu	v1,v0,1
80013598:	ae030000 	sw	v1,0(s0)
8001359c:	90430000 	lbu	v1,0(v0)
800135a0:	02801025 	move	v0,s4
800135a4:	24540001 	addiu	s4,v0,1
800135a8:	00031e00 	sll	v1,v1,0x18
800135ac:	00031e03 	sra	v1,v1,0x18
800135b0:	a0430000 	sb	v1,0(v0)
800135b4:	2652ffff 	addiu	s2,s2,-1
800135b8:	12400017 	beqz	s2,80013618 <__ssvfiscanf_r+0xd08>
800135bc:	00000000 	nop
800135c0:	8e020004 	lw	v0,4(s0)
800135c4:	1c40000b 	bgtz	v0,800135f4 <__ssvfiscanf_r+0xce4>
800135c8:	00000000 	nop
800135cc:	02002825 	move	a1,s0
800135d0:	8fc40200 	lw	a0,512(s8)
800135d4:	0c00498c 	jal	80012630 <__ssrefill_r>
800135d8:	00000000 	nop
800135dc:	10400005 	beqz	v0,800135f4 <__ssvfiscanf_r+0xce4>
800135e0:	00000000 	nop
800135e4:	1691000f 	bne	s4,s1,80013624 <__ssvfiscanf_r+0xd14>
800135e8:	00000000 	nop
800135ec:	1000026c 	b	80013fa0 <__ssvfiscanf_r+0x1690>
800135f0:	00000000 	nop
800135f4:	8e020000 	lw	v0,0(s0)
800135f8:	90420000 	lbu	v0,0(v0)
800135fc:	27c30028 	addiu	v1,s8,40
80013600:	00621021 	addu	v0,v1,v0
80013604:	80420058 	lb	v0,88(v0)
80013608:	1440ffde 	bnez	v0,80013584 <__ssvfiscanf_r+0xc74>
8001360c:	00000000 	nop
80013610:	10000005 	b	80013628 <__ssvfiscanf_r+0xd18>
80013614:	00000000 	nop
80013618:	00000000 	nop
8001361c:	10000002 	b	80013628 <__ssvfiscanf_r+0xd18>
80013620:	00000000 	nop
80013624:	00000000 	nop
80013628:	02919823 	subu	s3,s4,s1
8001362c:	12600286 	beqz	s3,80014048 <__ssvfiscanf_r+0x1738>
80013630:	00000000 	nop
80013634:	a2800000 	sb	zero,0(s4)
80013638:	8fc20028 	lw	v0,40(s8)
8001363c:	24420001 	addiu	v0,v0,1
80013640:	afc20028 	sw	v0,40(s8)
80013644:	8fc2002c 	lw	v0,44(s8)
80013648:	00531021 	addu	v0,v0,s3
8001364c:	afc2002c 	sw	v0,44(s8)
80013650:	10000231 	b	80013f18 <__ssvfiscanf_r+0x1608>
80013654:	00000000 	nop
80013658:	16400002 	bnez	s2,80013664 <__ssvfiscanf_r+0xd54>
8001365c:	00000000 	nop
80013660:	2412ffff 	li	s2,-1
80013664:	32220001 	andi	v0,s1,0x1
80013668:	1040009b 	beqz	v0,800138d8 <__ssvfiscanf_r+0xfc8>
8001366c:	00000000 	nop
80013670:	32220010 	andi	v0,s1,0x10
80013674:	10400005 	beqz	v0,8001368c <__ssvfiscanf_r+0xd7c>
80013678:	00000000 	nop
8001367c:	27c20078 	addiu	v0,s8,120
80013680:	afc20034 	sw	v0,52(s8)
80013684:	10000007 	b	800136a4 <__ssvfiscanf_r+0xd94>
80013688:	00000000 	nop
8001368c:	8fc20218 	lw	v0,536(s8)
80013690:	24430008 	addiu	v1,v0,8
80013694:	afc30218 	sw	v1,536(s8)
80013698:	24420004 	addiu	v0,v0,4
8001369c:	8c420000 	lw	v0,0(v0)
800136a0:	afc20034 	sw	v0,52(s8)
800136a4:	00009825 	move	s3,zero
800136a8:	10000071 	b	80013870 <__ssvfiscanf_r+0xf60>
800136ac:	00000000 	nop
800136b0:	0c003a42 	jal	8000e908 <__locale_mb_cur_max>
800136b4:	00000000 	nop
800136b8:	12620235 	beq	s3,v0,80013f90 <__ssvfiscanf_r+0x1680>
800136bc:	00000000 	nop
800136c0:	8e020000 	lw	v0,0(s0)
800136c4:	90430000 	lbu	v1,0(v0)
800136c8:	02601025 	move	v0,s3
800136cc:	24530001 	addiu	s3,v0,1
800136d0:	00031e00 	sll	v1,v1,0x18
800136d4:	00031e03 	sra	v1,v1,0x18
800136d8:	27c40028 	addiu	a0,s8,40
800136dc:	00821021 	addu	v0,a0,v0
800136e0:	a0430158 	sb	v1,344(v0)
800136e4:	8e020004 	lw	v0,4(s0)
800136e8:	2442ffff 	addiu	v0,v0,-1
800136ec:	ae020004 	sw	v0,4(s0)
800136f0:	8e020000 	lw	v0,0(s0)
800136f4:	24420001 	addiu	v0,v0,1
800136f8:	ae020000 	sw	v0,0(s0)
800136fc:	8fc30038 	lw	v1,56(s8)
80013700:	24020003 	li	v0,3
80013704:	14620005 	bne	v1,v0,8001371c <__ssvfiscanf_r+0xe0c>
80013708:	00000000 	nop
8001370c:	8fc301b8 	lw	v1,440(s8)
80013710:	24020004 	li	v0,4
80013714:	10620007 	beq	v1,v0,80013734 <__ssvfiscanf_r+0xe24>
80013718:	00000000 	nop
8001371c:	27c201b8 	addiu	v0,s8,440
80013720:	24060008 	li	a2,8
80013724:	00002825 	move	a1,zero
80013728:	00402025 	move	a0,v0
8001372c:	0c00173d 	jal	80005cf4 <memset>
80013730:	00000000 	nop
80013734:	02601825 	move	v1,s3
80013738:	27c40180 	addiu	a0,s8,384
8001373c:	27c201b8 	addiu	v0,s8,440
80013740:	afa20024 	sw	v0,36(sp)
80013744:	00603825 	move	a3,v1
80013748:	00803025 	move	a2,a0
8001374c:	8fc50034 	lw	a1,52(s8)
80013750:	8fc40200 	lw	a0,512(s8)
80013754:	0c0058a0 	jal	80016280 <_mbrtowc_r>
80013758:	00000000 	nop
8001375c:	afc20038 	sw	v0,56(s8)
80013760:	8fc30038 	lw	v1,56(s8)
80013764:	2402ffff 	li	v0,-1
80013768:	1062020c 	beq	v1,v0,80013f9c <__ssvfiscanf_r+0x168c>
8001376c:	00000000 	nop
80013770:	8fc20038 	lw	v0,56(s8)
80013774:	14400003 	bnez	v0,80013784 <__ssvfiscanf_r+0xe74>
80013778:	00000000 	nop
8001377c:	8fc20034 	lw	v0,52(s8)
80013780:	ac400000 	sw	zero,0(v0)
80013784:	8fc30038 	lw	v1,56(s8)
80013788:	2402fffe 	li	v0,-2
8001378c:	1062002b 	beq	v1,v0,8001383c <__ssvfiscanf_r+0xf2c>
80013790:	00000000 	nop
80013794:	8fc20034 	lw	v0,52(s8)
80013798:	8c420000 	lw	v0,0(v0)
8001379c:	00402025 	move	a0,v0
800137a0:	0c00587b 	jal	800161ec <iswspace>
800137a4:	00000000 	nop
800137a8:	10400011 	beqz	v0,800137f0 <__ssvfiscanf_r+0xee0>
800137ac:	00000000 	nop
800137b0:	1000000b 	b	800137e0 <__ssvfiscanf_r+0xed0>
800137b4:	00000000 	nop
800137b8:	2673ffff 	addiu	s3,s3,-1
800137bc:	27c20028 	addiu	v0,s8,40
800137c0:	00531021 	addu	v0,v0,s3
800137c4:	80420158 	lb	v0,344(v0)
800137c8:	304200ff 	andi	v0,v0,0xff
800137cc:	02003025 	move	a2,s0
800137d0:	00402825 	move	a1,v0
800137d4:	8fc40200 	lw	a0,512(s8)
800137d8:	0c004927 	jal	8001249c <_sungetc_r>
800137dc:	00000000 	nop
800137e0:	1660fff5 	bnez	s3,800137b8 <__ssvfiscanf_r+0xea8>
800137e4:	00000000 	nop
800137e8:	10000031 	b	800138b0 <__ssvfiscanf_r+0xfa0>
800137ec:	00000000 	nop
800137f0:	8fc2002c 	lw	v0,44(s8)
800137f4:	00531021 	addu	v0,v0,s3
800137f8:	afc2002c 	sw	v0,44(s8)
800137fc:	8fc30038 	lw	v1,56(s8)
80013800:	24020003 	li	v0,3
80013804:	14620005 	bne	v1,v0,8001381c <__ssvfiscanf_r+0xf0c>
80013808:	00000000 	nop
8001380c:	8fc301b8 	lw	v1,440(s8)
80013810:	24020004 	li	v0,4
80013814:	10620002 	beq	v1,v0,80013820 <__ssvfiscanf_r+0xf10>
80013818:	00000000 	nop
8001381c:	2652ffff 	addiu	s2,s2,-1
80013820:	32220010 	andi	v0,s1,0x10
80013824:	14400004 	bnez	v0,80013838 <__ssvfiscanf_r+0xf28>
80013828:	00000000 	nop
8001382c:	8fc20034 	lw	v0,52(s8)
80013830:	24420004 	addiu	v0,v0,4
80013834:	afc20034 	sw	v0,52(s8)
80013838:	00009825 	move	s3,zero
8001383c:	8e020004 	lw	v0,4(s0)
80013840:	1c40000b 	bgtz	v0,80013870 <__ssvfiscanf_r+0xf60>
80013844:	00000000 	nop
80013848:	02002825 	move	a1,s0
8001384c:	8fc40200 	lw	a0,512(s8)
80013850:	0c00498c 	jal	80012630 <__ssrefill_r>
80013854:	00000000 	nop
80013858:	10400005 	beqz	v0,80013870 <__ssvfiscanf_r+0xf60>
8001385c:	00000000 	nop
80013860:	12600012 	beqz	s3,800138ac <__ssvfiscanf_r+0xf9c>
80013864:	00000000 	nop
80013868:	100001cd 	b	80013fa0 <__ssvfiscanf_r+0x1690>
8001386c:	00000000 	nop
80013870:	8e020000 	lw	v0,0(s0)
80013874:	90420000 	lbu	v0,0(v0)
80013878:	24430001 	addiu	v1,v0,1
8001387c:	3c028002 	lui	v0,0x8002
80013880:	2442af60 	addiu	v0,v0,-20640
80013884:	00621021 	addu	v0,v1,v0
80013888:	80420000 	lb	v0,0(v0)
8001388c:	304200ff 	andi	v0,v0,0xff
80013890:	30420008 	andi	v0,v0,0x8
80013894:	14400006 	bnez	v0,800138b0 <__ssvfiscanf_r+0xfa0>
80013898:	00000000 	nop
8001389c:	1640ff84 	bnez	s2,800136b0 <__ssvfiscanf_r+0xda0>
800138a0:	00000000 	nop
800138a4:	10000002 	b	800138b0 <__ssvfiscanf_r+0xfa0>
800138a8:	00000000 	nop
800138ac:	00000000 	nop
800138b0:	32220010 	andi	v0,s1,0x10
800138b4:	1440019b 	bnez	v0,80013f24 <__ssvfiscanf_r+0x1614>
800138b8:	00000000 	nop
800138bc:	8fc20034 	lw	v0,52(s8)
800138c0:	ac400000 	sw	zero,0(v0)
800138c4:	8fc20028 	lw	v0,40(s8)
800138c8:	24420001 	addiu	v0,v0,1
800138cc:	afc20028 	sw	v0,40(s8)
800138d0:	10000194 	b	80013f24 <__ssvfiscanf_r+0x1614>
800138d4:	00000000 	nop
800138d8:	32220010 	andi	v0,s1,0x10
800138dc:	1040002d 	beqz	v0,80013994 <__ssvfiscanf_r+0x1084>
800138e0:	00000000 	nop
800138e4:	00009825 	move	s3,zero
800138e8:	10000014 	b	8001393c <__ssvfiscanf_r+0x102c>
800138ec:	00000000 	nop
800138f0:	26730001 	addiu	s3,s3,1
800138f4:	8e020004 	lw	v0,4(s0)
800138f8:	2442ffff 	addiu	v0,v0,-1
800138fc:	ae020004 	sw	v0,4(s0)
80013900:	8e020000 	lw	v0,0(s0)
80013904:	24420001 	addiu	v0,v0,1
80013908:	ae020000 	sw	v0,0(s0)
8001390c:	2652ffff 	addiu	s2,s2,-1
80013910:	12400017 	beqz	s2,80013970 <__ssvfiscanf_r+0x1060>
80013914:	00000000 	nop
80013918:	8e020004 	lw	v0,4(s0)
8001391c:	1c400007 	bgtz	v0,8001393c <__ssvfiscanf_r+0x102c>
80013920:	00000000 	nop
80013924:	02002825 	move	a1,s0
80013928:	8fc40200 	lw	a0,512(s8)
8001392c:	0c00498c 	jal	80012630 <__ssrefill_r>
80013930:	00000000 	nop
80013934:	14400011 	bnez	v0,8001397c <__ssvfiscanf_r+0x106c>
80013938:	00000000 	nop
8001393c:	8e020000 	lw	v0,0(s0)
80013940:	90420000 	lbu	v0,0(v0)
80013944:	24430001 	addiu	v1,v0,1
80013948:	3c028002 	lui	v0,0x8002
8001394c:	2442af60 	addiu	v0,v0,-20640
80013950:	00621021 	addu	v0,v1,v0
80013954:	80420000 	lb	v0,0(v0)
80013958:	304200ff 	andi	v0,v0,0xff
8001395c:	30420008 	andi	v0,v0,0x8
80013960:	1040ffe3 	beqz	v0,800138f0 <__ssvfiscanf_r+0xfe0>
80013964:	00000000 	nop
80013968:	10000005 	b	80013980 <__ssvfiscanf_r+0x1070>
8001396c:	00000000 	nop
80013970:	00000000 	nop
80013974:	10000002 	b	80013980 <__ssvfiscanf_r+0x1070>
80013978:	00000000 	nop
8001397c:	00000000 	nop
80013980:	8fc2002c 	lw	v0,44(s8)
80013984:	00531021 	addu	v0,v0,s3
80013988:	afc2002c 	sw	v0,44(s8)
8001398c:	10000165 	b	80013f24 <__ssvfiscanf_r+0x1614>
80013990:	00000000 	nop
80013994:	8fc20218 	lw	v0,536(s8)
80013998:	24430008 	addiu	v1,v0,8
8001399c:	afc30218 	sw	v1,536(s8)
800139a0:	24420004 	addiu	v0,v0,4
800139a4:	8c510000 	lw	s1,0(v0)
800139a8:	0220a025 	move	s4,s1
800139ac:	10000019 	b	80013a14 <__ssvfiscanf_r+0x1104>
800139b0:	00000000 	nop
800139b4:	8e020004 	lw	v0,4(s0)
800139b8:	2442ffff 	addiu	v0,v0,-1
800139bc:	ae020004 	sw	v0,4(s0)
800139c0:	8e020000 	lw	v0,0(s0)
800139c4:	24430001 	addiu	v1,v0,1
800139c8:	ae030000 	sw	v1,0(s0)
800139cc:	90430000 	lbu	v1,0(v0)
800139d0:	02801025 	move	v0,s4
800139d4:	24540001 	addiu	s4,v0,1
800139d8:	00031e00 	sll	v1,v1,0x18
800139dc:	00031e03 	sra	v1,v1,0x18
800139e0:	a0430000 	sb	v1,0(v0)
800139e4:	2652ffff 	addiu	s2,s2,-1
800139e8:	12400017 	beqz	s2,80013a48 <__ssvfiscanf_r+0x1138>
800139ec:	00000000 	nop
800139f0:	8e020004 	lw	v0,4(s0)
800139f4:	1c400007 	bgtz	v0,80013a14 <__ssvfiscanf_r+0x1104>
800139f8:	00000000 	nop
800139fc:	02002825 	move	a1,s0
80013a00:	8fc40200 	lw	a0,512(s8)
80013a04:	0c00498c 	jal	80012630 <__ssrefill_r>
80013a08:	00000000 	nop
80013a0c:	14400011 	bnez	v0,80013a54 <__ssvfiscanf_r+0x1144>
80013a10:	00000000 	nop
80013a14:	8e020000 	lw	v0,0(s0)
80013a18:	90420000 	lbu	v0,0(v0)
80013a1c:	24430001 	addiu	v1,v0,1
80013a20:	3c028002 	lui	v0,0x8002
80013a24:	2442af60 	addiu	v0,v0,-20640
80013a28:	00621021 	addu	v0,v1,v0
80013a2c:	80420000 	lb	v0,0(v0)
80013a30:	304200ff 	andi	v0,v0,0xff
80013a34:	30420008 	andi	v0,v0,0x8
80013a38:	1040ffde 	beqz	v0,800139b4 <__ssvfiscanf_r+0x10a4>
80013a3c:	00000000 	nop
80013a40:	10000005 	b	80013a58 <__ssvfiscanf_r+0x1148>
80013a44:	00000000 	nop
80013a48:	00000000 	nop
80013a4c:	10000002 	b	80013a58 <__ssvfiscanf_r+0x1148>
80013a50:	00000000 	nop
80013a54:	00000000 	nop
80013a58:	a2800000 	sb	zero,0(s4)
80013a5c:	02911023 	subu	v0,s4,s1
80013a60:	8fc3002c 	lw	v1,44(s8)
80013a64:	00621021 	addu	v0,v1,v0
80013a68:	afc2002c 	sw	v0,44(s8)
80013a6c:	8fc20028 	lw	v0,40(s8)
80013a70:	24420001 	addiu	v0,v0,1
80013a74:	afc20028 	sw	v0,40(s8)
80013a78:	1000012a 	b	80013f24 <__ssvfiscanf_r+0x1614>
80013a7c:	00000000 	nop
80013a80:	afc00048 	sw	zero,72(s8)
80013a84:	afc0004c 	sw	zero,76(s8)
80013a88:	2642ffff 	addiu	v0,s2,-1
80013a8c:	2c420027 	sltiu	v0,v0,39
80013a90:	14400004 	bnez	v0,80013aa4 <__ssvfiscanf_r+0x1194>
80013a94:	00000000 	nop
80013a98:	2642ffd9 	addiu	v0,s2,-39
80013a9c:	afc20048 	sw	v0,72(s8)
80013aa0:	24120027 	li	s2,39
80013aa4:	36310d80 	ori	s1,s1,0xd80
80013aa8:	27d40180 	addiu	s4,s8,384
80013aac:	10000085 	b	80013cc4 <__ssvfiscanf_r+0x13b4>
80013ab0:	00000000 	nop
80013ab4:	8e020000 	lw	v0,0(s0)
80013ab8:	90420000 	lbu	v0,0(v0)
80013abc:	00409825 	move	s3,v0
80013ac0:	2662ffd5 	addiu	v0,s3,-43
80013ac4:	2c43004e 	sltiu	v1,v0,78
80013ac8:	10600082 	beqz	v1,80013cd4 <__ssvfiscanf_r+0x13c4>
80013acc:	00000000 	nop
80013ad0:	00021880 	sll	v1,v0,0x2
80013ad4:	3c028002 	lui	v0,0x8002
80013ad8:	2442aaa8 	addiu	v0,v0,-21848
80013adc:	00621021 	addu	v0,v1,v0
80013ae0:	8c420000 	lw	v0,0(v0)
80013ae4:	00400008 	jr	v0
80013ae8:	00000000 	nop
80013aec:	32220800 	andi	v0,s1,0x800
80013af0:	1040005c 	beqz	v0,80013c64 <__ssvfiscanf_r+0x1354>
80013af4:	00000000 	nop
80013af8:	8fc20030 	lw	v0,48(s8)
80013afc:	14400004 	bnez	v0,80013b10 <__ssvfiscanf_r+0x1200>
80013b00:	00000000 	nop
80013b04:	24020008 	li	v0,8
80013b08:	afc20030 	sw	v0,48(s8)
80013b0c:	36310200 	ori	s1,s1,0x200
80013b10:	32220400 	andi	v0,s1,0x400
80013b14:	10400005 	beqz	v0,80013b2c <__ssvfiscanf_r+0x121c>
80013b18:	00000000 	nop
80013b1c:	2402fa7f 	li	v0,-1409
80013b20:	02228824 	and	s1,s1,v0
80013b24:	10000050 	b	80013c68 <__ssvfiscanf_r+0x1358>
80013b28:	00000000 	nop
80013b2c:	2402fc7f 	li	v0,-897
80013b30:	02228824 	and	s1,s1,v0
80013b34:	8fc20048 	lw	v0,72(s8)
80013b38:	10400005 	beqz	v0,80013b50 <__ssvfiscanf_r+0x1240>
80013b3c:	00000000 	nop
80013b40:	8fc20048 	lw	v0,72(s8)
80013b44:	2442ffff 	addiu	v0,v0,-1
80013b48:	afc20048 	sw	v0,72(s8)
80013b4c:	26520001 	addiu	s2,s2,1
80013b50:	8fc2004c 	lw	v0,76(s8)
80013b54:	24420001 	addiu	v0,v0,1
80013b58:	afc2004c 	sw	v0,76(s8)
80013b5c:	10000047 	b	80013c7c <__ssvfiscanf_r+0x136c>
80013b60:	00000000 	nop
80013b64:	3c028002 	lui	v0,0x8002
80013b68:	8fc30030 	lw	v1,48(s8)
80013b6c:	00031840 	sll	v1,v1,0x1
80013b70:	2442abe0 	addiu	v0,v0,-21536
80013b74:	00621021 	addu	v0,v1,v0
80013b78:	84420000 	lh	v0,0(v0)
80013b7c:	afc20030 	sw	v0,48(s8)
80013b80:	2402f47f 	li	v0,-2945
80013b84:	02228824 	and	s1,s1,v0
80013b88:	10000037 	b	80013c68 <__ssvfiscanf_r+0x1358>
80013b8c:	00000000 	nop
80013b90:	3c028002 	lui	v0,0x8002
80013b94:	8fc30030 	lw	v1,48(s8)
80013b98:	00031840 	sll	v1,v1,0x1
80013b9c:	2442abe0 	addiu	v0,v0,-21536
80013ba0:	00621021 	addu	v0,v1,v0
80013ba4:	84420000 	lh	v0,0(v0)
80013ba8:	afc20030 	sw	v0,48(s8)
80013bac:	8fc20030 	lw	v0,48(s8)
80013bb0:	28420009 	slti	v0,v0,9
80013bb4:	1440001f 	bnez	v0,80013c34 <__ssvfiscanf_r+0x1324>
80013bb8:	00000000 	nop
80013bbc:	2402f47f 	li	v0,-2945
80013bc0:	02228824 	and	s1,s1,v0
80013bc4:	10000028 	b	80013c68 <__ssvfiscanf_r+0x1358>
80013bc8:	00000000 	nop
80013bcc:	8fc20030 	lw	v0,48(s8)
80013bd0:	2842000b 	slti	v0,v0,11
80013bd4:	1440001a 	bnez	v0,80013c40 <__ssvfiscanf_r+0x1330>
80013bd8:	00000000 	nop
80013bdc:	2402f47f 	li	v0,-2945
80013be0:	02228824 	and	s1,s1,v0
80013be4:	10000020 	b	80013c68 <__ssvfiscanf_r+0x1358>
80013be8:	00000000 	nop
80013bec:	32220080 	andi	v0,s1,0x80
80013bf0:	10400016 	beqz	v0,80013c4c <__ssvfiscanf_r+0x133c>
80013bf4:	00000000 	nop
80013bf8:	2402ff7f 	li	v0,-129
80013bfc:	02228824 	and	s1,s1,v0
80013c00:	10000019 	b	80013c68 <__ssvfiscanf_r+0x1358>
80013c04:	00000000 	nop
80013c08:	32230600 	andi	v1,s1,0x600
80013c0c:	24020200 	li	v0,512
80013c10:	14620011 	bne	v1,v0,80013c58 <__ssvfiscanf_r+0x1348>
80013c14:	00000000 	nop
80013c18:	24020010 	li	v0,16
80013c1c:	afc20030 	sw	v0,48(s8)
80013c20:	2402fdff 	li	v0,-513
80013c24:	02228824 	and	s1,s1,v0
80013c28:	36310500 	ori	s1,s1,0x500
80013c2c:	1000000e 	b	80013c68 <__ssvfiscanf_r+0x1358>
80013c30:	00000000 	nop
80013c34:	00000000 	nop
80013c38:	10000026 	b	80013cd4 <__ssvfiscanf_r+0x13c4>
80013c3c:	00000000 	nop
80013c40:	00000000 	nop
80013c44:	10000023 	b	80013cd4 <__ssvfiscanf_r+0x13c4>
80013c48:	00000000 	nop
80013c4c:	00000000 	nop
80013c50:	10000020 	b	80013cd4 <__ssvfiscanf_r+0x13c4>
80013c54:	00000000 	nop
80013c58:	00000000 	nop
80013c5c:	1000001d 	b	80013cd4 <__ssvfiscanf_r+0x13c4>
80013c60:	00000000 	nop
80013c64:	00000000 	nop
80013c68:	02801025 	move	v0,s4
80013c6c:	24540001 	addiu	s4,v0,1
80013c70:	00131e00 	sll	v1,s3,0x18
80013c74:	00031e03 	sra	v1,v1,0x18
80013c78:	a0430000 	sb	v1,0(v0)
80013c7c:	8e020004 	lw	v0,4(s0)
80013c80:	2442ffff 	addiu	v0,v0,-1
80013c84:	ae020004 	sw	v0,4(s0)
80013c88:	8e020004 	lw	v0,4(s0)
80013c8c:	18400006 	blez	v0,80013ca8 <__ssvfiscanf_r+0x1398>
80013c90:	00000000 	nop
80013c94:	8e020000 	lw	v0,0(s0)
80013c98:	24420001 	addiu	v0,v0,1
80013c9c:	ae020000 	sw	v0,0(s0)
80013ca0:	10000007 	b	80013cc0 <__ssvfiscanf_r+0x13b0>
80013ca4:	00000000 	nop
80013ca8:	02002825 	move	a1,s0
80013cac:	8fc40200 	lw	a0,512(s8)
80013cb0:	0c00498c 	jal	80012630 <__ssrefill_r>
80013cb4:	00000000 	nop
80013cb8:	14400009 	bnez	v0,80013ce0 <__ssvfiscanf_r+0x13d0>
80013cbc:	00000000 	nop
80013cc0:	2652ffff 	addiu	s2,s2,-1
80013cc4:	1640ff7b 	bnez	s2,80013ab4 <__ssvfiscanf_r+0x11a4>
80013cc8:	00000000 	nop
80013ccc:	10000005 	b	80013ce4 <__ssvfiscanf_r+0x13d4>
80013cd0:	00000000 	nop
80013cd4:	00000000 	nop
80013cd8:	10000002 	b	80013ce4 <__ssvfiscanf_r+0x13d4>
80013cdc:	00000000 	nop
80013ce0:	00000000 	nop
80013ce4:	32220100 	andi	v0,s1,0x100
80013ce8:	1040000f 	beqz	v0,80013d28 <__ssvfiscanf_r+0x1418>
80013cec:	00000000 	nop
80013cf0:	27c20180 	addiu	v0,s8,384
80013cf4:	0054102b 	sltu	v0,v0,s4
80013cf8:	10400008 	beqz	v0,80013d1c <__ssvfiscanf_r+0x140c>
80013cfc:	00000000 	nop
80013d00:	2694ffff 	addiu	s4,s4,-1
80013d04:	82820000 	lb	v0,0(s4)
80013d08:	02003025 	move	a2,s0
80013d0c:	00402825 	move	a1,v0
80013d10:	8fc40200 	lw	a0,512(s8)
80013d14:	0c004927 	jal	8001249c <_sungetc_r>
80013d18:	00000000 	nop
80013d1c:	27c20180 	addiu	v0,s8,384
80013d20:	128200cc 	beq	s4,v0,80014054 <__ssvfiscanf_r+0x1744>
80013d24:	00000000 	nop
80013d28:	32220010 	andi	v0,s1,0x10
80013d2c:	14400068 	bnez	v0,80013ed0 <__ssvfiscanf_r+0x15c0>
80013d30:	00000000 	nop
80013d34:	a2800000 	sb	zero,0(s4)
80013d38:	27c30180 	addiu	v1,s8,384
80013d3c:	8fc2003c 	lw	v0,60(s8)
80013d40:	8fc70030 	lw	a3,48(s8)
80013d44:	00003025 	move	a2,zero
80013d48:	00602825 	move	a1,v1
80013d4c:	8fc40200 	lw	a0,512(s8)
80013d50:	0040f809 	jalr	v0
80013d54:	00000000 	nop
80013d58:	afc2006c 	sw	v0,108(s8)
80013d5c:	32220020 	andi	v0,s1,0x20
80013d60:	1040000c 	beqz	v0,80013d94 <__ssvfiscanf_r+0x1484>
80013d64:	00000000 	nop
80013d68:	8fc20218 	lw	v0,536(s8)
80013d6c:	24430008 	addiu	v1,v0,8
80013d70:	afc30218 	sw	v1,536(s8)
80013d74:	24420004 	addiu	v0,v0,4
80013d78:	8c420000 	lw	v0,0(v0)
80013d7c:	afc20070 	sw	v0,112(s8)
80013d80:	8fc3006c 	lw	v1,108(s8)
80013d84:	8fc20070 	lw	v0,112(s8)
80013d88:	ac430000 	sw	v1,0(v0)
80013d8c:	1000004d 	b	80013ec4 <__ssvfiscanf_r+0x15b4>
80013d90:	00000000 	nop
80013d94:	32220004 	andi	v0,s1,0x4
80013d98:	1040000e 	beqz	v0,80013dd4 <__ssvfiscanf_r+0x14c4>
80013d9c:	00000000 	nop
80013da0:	8fc20218 	lw	v0,536(s8)
80013da4:	24430008 	addiu	v1,v0,8
80013da8:	afc30218 	sw	v1,536(s8)
80013dac:	24420004 	addiu	v0,v0,4
80013db0:	8c420000 	lw	v0,0(v0)
80013db4:	afc20068 	sw	v0,104(s8)
80013db8:	8fc2006c 	lw	v0,108(s8)
80013dbc:	00021c00 	sll	v1,v0,0x10
80013dc0:	00031c03 	sra	v1,v1,0x10
80013dc4:	8fc20068 	lw	v0,104(s8)
80013dc8:	a4430000 	sh	v1,0(v0)
80013dcc:	1000003d 	b	80013ec4 <__ssvfiscanf_r+0x15b4>
80013dd0:	00000000 	nop
80013dd4:	32220001 	andi	v0,s1,0x1
80013dd8:	1040000c 	beqz	v0,80013e0c <__ssvfiscanf_r+0x14fc>
80013ddc:	00000000 	nop
80013de0:	8fc20218 	lw	v0,536(s8)
80013de4:	24430008 	addiu	v1,v0,8
80013de8:	afc30218 	sw	v1,536(s8)
80013dec:	24420004 	addiu	v0,v0,4
80013df0:	8c420000 	lw	v0,0(v0)
80013df4:	afc20064 	sw	v0,100(s8)
80013df8:	8fc3006c 	lw	v1,108(s8)
80013dfc:	8fc20064 	lw	v0,100(s8)
80013e00:	ac430000 	sw	v1,0(v0)
80013e04:	1000002f 	b	80013ec4 <__ssvfiscanf_r+0x15b4>
80013e08:	00000000 	nop
80013e0c:	32220002 	andi	v0,s1,0x2
80013e10:	10400023 	beqz	v0,80013ea0 <__ssvfiscanf_r+0x1590>
80013e14:	00000000 	nop
80013e18:	8fc3003c 	lw	v1,60(s8)
80013e1c:	3c028001 	lui	v0,0x8001
80013e20:	24422328 	addiu	v0,v0,9000
80013e24:	1462000b 	bne	v1,v0,80013e54 <__ssvfiscanf_r+0x1544>
80013e28:	00000000 	nop
80013e2c:	27c20180 	addiu	v0,s8,384
80013e30:	8fc70030 	lw	a3,48(s8)
80013e34:	00003025 	move	a2,zero
80013e38:	00402825 	move	a1,v0
80013e3c:	8fc40200 	lw	a0,512(s8)
80013e40:	0c005cd8 	jal	80017360 <_strtoull_r>
80013e44:	00000000 	nop
80013e48:	ffc20050 	sd	v0,80(s8)
80013e4c:	10000009 	b	80013e74 <__ssvfiscanf_r+0x1564>
80013e50:	00000000 	nop
80013e54:	27c20180 	addiu	v0,s8,384
80013e58:	8fc70030 	lw	a3,48(s8)
80013e5c:	00003025 	move	a2,zero
80013e60:	00402825 	move	a1,v0
80013e64:	8fc40200 	lw	a0,512(s8)
80013e68:	0c005a6c 	jal	800169b0 <_strtoll_r>
80013e6c:	00000000 	nop
80013e70:	ffc20050 	sd	v0,80(s8)
80013e74:	8fc20218 	lw	v0,536(s8)
80013e78:	24430008 	addiu	v1,v0,8
80013e7c:	afc30218 	sw	v1,536(s8)
80013e80:	24420004 	addiu	v0,v0,4
80013e84:	8c420000 	lw	v0,0(v0)
80013e88:	afc20060 	sw	v0,96(s8)
80013e8c:	dfc30050 	ld	v1,80(s8)
80013e90:	8fc20060 	lw	v0,96(s8)
80013e94:	fc430000 	sd	v1,0(v0)
80013e98:	1000000a 	b	80013ec4 <__ssvfiscanf_r+0x15b4>
80013e9c:	00000000 	nop
80013ea0:	8fc20218 	lw	v0,536(s8)
80013ea4:	24430008 	addiu	v1,v0,8
80013ea8:	afc30218 	sw	v1,536(s8)
80013eac:	24420004 	addiu	v0,v0,4
80013eb0:	8c420000 	lw	v0,0(v0)
80013eb4:	afc2005c 	sw	v0,92(s8)
80013eb8:	8fc3006c 	lw	v1,108(s8)
80013ebc:	8fc2005c 	lw	v0,92(s8)
80013ec0:	ac430000 	sw	v1,0(v0)
80013ec4:	8fc20028 	lw	v0,40(s8)
80013ec8:	24420001 	addiu	v0,v0,1
80013ecc:	afc20028 	sw	v0,40(s8)
80013ed0:	27c20180 	addiu	v0,s8,384
80013ed4:	02821823 	subu	v1,s4,v0
80013ed8:	8fc2004c 	lw	v0,76(s8)
80013edc:	00621021 	addu	v0,v1,v0
80013ee0:	8fc3002c 	lw	v1,44(s8)
80013ee4:	00621021 	addu	v0,v1,v0
80013ee8:	afc2002c 	sw	v0,44(s8)
80013eec:	1000000e 	b	80013f28 <__ssvfiscanf_r+0x1618>
80013ef0:	00000000 	nop
80013ef4:	00000000 	nop
80013ef8:	1000faa9 	b	800129a0 <__ssvfiscanf_r+0x90>
80013efc:	00000000 	nop
80013f00:	00000000 	nop
80013f04:	1000faa6 	b	800129a0 <__ssvfiscanf_r+0x90>
80013f08:	00000000 	nop
80013f0c:	00000000 	nop
80013f10:	1000faa3 	b	800129a0 <__ssvfiscanf_r+0x90>
80013f14:	00000000 	nop
80013f18:	00000000 	nop
80013f1c:	1000faa0 	b	800129a0 <__ssvfiscanf_r+0x90>
80013f20:	00000000 	nop
80013f24:	00000000 	nop
80013f28:	1000fa9d 	b	800129a0 <__ssvfiscanf_r+0x90>
80013f2c:	00000000 	nop
80013f30:	00000000 	nop
80013f34:	1000001a 	b	80013fa0 <__ssvfiscanf_r+0x1690>
80013f38:	00000000 	nop
80013f3c:	00000000 	nop
80013f40:	10000017 	b	80013fa0 <__ssvfiscanf_r+0x1690>
80013f44:	00000000 	nop
80013f48:	00000000 	nop
80013f4c:	10000014 	b	80013fa0 <__ssvfiscanf_r+0x1690>
80013f50:	00000000 	nop
80013f54:	00000000 	nop
80013f58:	10000011 	b	80013fa0 <__ssvfiscanf_r+0x1690>
80013f5c:	00000000 	nop
80013f60:	00000000 	nop
80013f64:	1000000e 	b	80013fa0 <__ssvfiscanf_r+0x1690>
80013f68:	00000000 	nop
80013f6c:	00000000 	nop
80013f70:	1000000b 	b	80013fa0 <__ssvfiscanf_r+0x1690>
80013f74:	00000000 	nop
80013f78:	00000000 	nop
80013f7c:	10000008 	b	80013fa0 <__ssvfiscanf_r+0x1690>
80013f80:	00000000 	nop
80013f84:	00000000 	nop
80013f88:	10000005 	b	80013fa0 <__ssvfiscanf_r+0x1690>
80013f8c:	00000000 	nop
80013f90:	00000000 	nop
80013f94:	10000002 	b	80013fa0 <__ssvfiscanf_r+0x1690>
80013f98:	00000000 	nop
80013f9c:	00000000 	nop
80013fa0:	8fc20028 	lw	v0,40(s8)
80013fa4:	10400009 	beqz	v0,80013fcc <__ssvfiscanf_r+0x16bc>
80013fa8:	00000000 	nop
80013fac:	8602000c 	lh	v0,12(s0)
80013fb0:	3042ffff 	andi	v0,v0,0xffff
80013fb4:	30420040 	andi	v0,v0,0x40
80013fb8:	14400004 	bnez	v0,80013fcc <__ssvfiscanf_r+0x16bc>
80013fbc:	00000000 	nop
80013fc0:	8fc20028 	lw	v0,40(s8)
80013fc4:	10000002 	b	80013fd0 <__ssvfiscanf_r+0x16c0>
80013fc8:	00000000 	nop
80013fcc:	2402ffff 	li	v0,-1
80013fd0:	afc20028 	sw	v0,40(s8)
80013fd4:	10000020 	b	80014058 <__ssvfiscanf_r+0x1748>
80013fd8:	00000000 	nop
80013fdc:	00000000 	nop
80013fe0:	1000001d 	b	80014058 <__ssvfiscanf_r+0x1748>
80013fe4:	00000000 	nop
80013fe8:	00000000 	nop
80013fec:	1000001a 	b	80014058 <__ssvfiscanf_r+0x1748>
80013ff0:	00000000 	nop
80013ff4:	00000000 	nop
80013ff8:	10000017 	b	80014058 <__ssvfiscanf_r+0x1748>
80013ffc:	00000000 	nop
80014000:	00000000 	nop
80014004:	10000014 	b	80014058 <__ssvfiscanf_r+0x1748>
80014008:	00000000 	nop
8001400c:	00000000 	nop
80014010:	10000011 	b	80014058 <__ssvfiscanf_r+0x1748>
80014014:	00000000 	nop
80014018:	00000000 	nop
8001401c:	1000000e 	b	80014058 <__ssvfiscanf_r+0x1748>
80014020:	00000000 	nop
80014024:	00000000 	nop
80014028:	1000000b 	b	80014058 <__ssvfiscanf_r+0x1748>
8001402c:	00000000 	nop
80014030:	00000000 	nop
80014034:	10000008 	b	80014058 <__ssvfiscanf_r+0x1748>
80014038:	00000000 	nop
8001403c:	00000000 	nop
80014040:	10000005 	b	80014058 <__ssvfiscanf_r+0x1748>
80014044:	00000000 	nop
80014048:	00000000 	nop
8001404c:	10000002 	b	80014058 <__ssvfiscanf_r+0x1748>
80014050:	00000000 	nop
80014054:	00000000 	nop
80014058:	8fc20028 	lw	v0,40(s8)
8001405c:	03c0e825 	move	sp,s8
80014060:	dfbf01f8 	ld	ra,504(sp)
80014064:	dfbe01f0 	ld	s8,496(sp)
80014068:	dfb501e8 	ld	s5,488(sp)
8001406c:	dfb401e0 	ld	s4,480(sp)
80014070:	dfb301d8 	ld	s3,472(sp)
80014074:	dfb201d0 	ld	s2,464(sp)
80014078:	dfb101c8 	ld	s1,456(sp)
8001407c:	dfb001c0 	ld	s0,448(sp)
80014080:	27bd0200 	addiu	sp,sp,512
80014084:	03e00008 	jr	ra
80014088:	00000000 	nop

8001408c <__submore>:
8001408c:	27bdffb8 	addiu	sp,sp,-72
80014090:	ffbf0040 	sd	ra,64(sp)
80014094:	ffbe0038 	sd	s8,56(sp)
80014098:	ffb20030 	sd	s2,48(sp)
8001409c:	ffb10028 	sd	s1,40(sp)
800140a0:	ffb00020 	sd	s0,32(sp)
800140a4:	03a0f025 	move	s8,sp
800140a8:	afc40048 	sw	a0,72(s8)
800140ac:	00a08025 	move	s0,a1
800140b0:	8e030030 	lw	v1,48(s0)
800140b4:	26020040 	addiu	v0,s0,64
800140b8:	1462001e 	bne	v1,v0,80014134 <__submore+0xa8>
800140bc:	00000000 	nop
800140c0:	24050400 	li	a1,1024
800140c4:	8fc40048 	lw	a0,72(s8)
800140c8:	0c001296 	jal	80004a58 <_malloc_r>
800140cc:	00000000 	nop
800140d0:	00408825 	move	s1,v0
800140d4:	16200004 	bnez	s1,800140e8 <__submore+0x5c>
800140d8:	00000000 	nop
800140dc:	2402ffff 	li	v0,-1
800140e0:	10000031 	b	800141a8 <__submore+0x11c>
800140e4:	00000000 	nop
800140e8:	ae110030 	sw	s1,48(s0)
800140ec:	24020400 	li	v0,1024
800140f0:	ae020034 	sw	v0,52(s0)
800140f4:	263103fd 	addiu	s1,s1,1021
800140f8:	24120003 	li	s2,3
800140fc:	10000006 	b	80014118 <__submore+0x8c>
80014100:	00000000 	nop
80014104:	02401025 	move	v0,s2
80014108:	02221021 	addu	v0,s1,v0
8001410c:	02121821 	addu	v1,s0,s2
80014110:	90630040 	lbu	v1,64(v1)
80014114:	a0430000 	sb	v1,0(v0)
80014118:	2652ffff 	addiu	s2,s2,-1
8001411c:	0641fff9 	bgez	s2,80014104 <__submore+0x78>
80014120:	00000000 	nop
80014124:	ae110000 	sw	s1,0(s0)
80014128:	00001025 	move	v0,zero
8001412c:	1000001e 	b	800141a8 <__submore+0x11c>
80014130:	00000000 	nop
80014134:	8e120034 	lw	s2,52(s0)
80014138:	8e020030 	lw	v0,48(s0)
8001413c:	00121840 	sll	v1,s2,0x1
80014140:	00603025 	move	a2,v1
80014144:	00402825 	move	a1,v0
80014148:	8fc40048 	lw	a0,72(s8)
8001414c:	0c00430b 	jal	80010c2c <_realloc_r>
80014150:	00000000 	nop
80014154:	00408825 	move	s1,v0
80014158:	16200004 	bnez	s1,8001416c <__submore+0xe0>
8001415c:	00000000 	nop
80014160:	2402ffff 	li	v0,-1
80014164:	10000010 	b	800141a8 <__submore+0x11c>
80014168:	00000000 	nop
8001416c:	02401025 	move	v0,s2
80014170:	02221021 	addu	v0,s1,v0
80014174:	02401825 	move	v1,s2
80014178:	00603025 	move	a2,v1
8001417c:	02202825 	move	a1,s1
80014180:	00402025 	move	a0,v0
80014184:	0c0015cf 	jal	8000573c <memcpy>
80014188:	00000000 	nop
8001418c:	02401025 	move	v0,s2
80014190:	02221021 	addu	v0,s1,v0
80014194:	ae020000 	sw	v0,0(s0)
80014198:	ae110030 	sw	s1,48(s0)
8001419c:	00121040 	sll	v0,s2,0x1
800141a0:	ae020034 	sw	v0,52(s0)
800141a4:	00001025 	move	v0,zero
800141a8:	03c0e825 	move	sp,s8
800141ac:	dfbf0040 	ld	ra,64(sp)
800141b0:	dfbe0038 	ld	s8,56(sp)
800141b4:	dfb20030 	ld	s2,48(sp)
800141b8:	dfb10028 	ld	s1,40(sp)
800141bc:	dfb00020 	ld	s0,32(sp)
800141c0:	27bd0048 	addiu	sp,sp,72
800141c4:	03e00008 	jr	ra
800141c8:	00000000 	nop

800141cc <_ungetc_r>:
800141cc:	27bdffc0 	addiu	sp,sp,-64
800141d0:	ffbf0038 	sd	ra,56(sp)
800141d4:	ffbe0030 	sd	s8,48(sp)
800141d8:	ffb00028 	sd	s0,40(sp)
800141dc:	03a0f025 	move	s8,sp
800141e0:	afc40040 	sw	a0,64(s8)
800141e4:	afc50048 	sw	a1,72(s8)
800141e8:	00c08025 	move	s0,a2
800141ec:	8fc30048 	lw	v1,72(s8)
800141f0:	2402ffff 	li	v0,-1
800141f4:	14620004 	bne	v1,v0,80014208 <_ungetc_r+0x3c>
800141f8:	00000000 	nop
800141fc:	2402ffff 	li	v0,-1
80014200:	10000092 	b	8001444c <_ungetc_r+0x280>
80014204:	00000000 	nop
80014208:	8fc20040 	lw	v0,64(s8)
8001420c:	afc20020 	sw	v0,32(s8)
80014210:	8fc20020 	lw	v0,32(s8)
80014214:	10400008 	beqz	v0,80014238 <_ungetc_r+0x6c>
80014218:	00000000 	nop
8001421c:	8fc20020 	lw	v0,32(s8)
80014220:	8c420038 	lw	v0,56(v0)
80014224:	14400004 	bnez	v0,80014238 <_ungetc_r+0x6c>
80014228:	00000000 	nop
8001422c:	8fc40020 	lw	a0,32(s8)
80014230:	0c000cc3 	jal	8000330c <__sinit>
80014234:	00000000 	nop
80014238:	8602000c 	lh	v0,12(s0)
8001423c:	3042ffff 	andi	v0,v0,0xffff
80014240:	30422000 	andi	v0,v0,0x2000
80014244:	1440000a 	bnez	v0,80014270 <_ungetc_r+0xa4>
80014248:	00000000 	nop
8001424c:	8602000c 	lh	v0,12(s0)
80014250:	34422000 	ori	v0,v0,0x2000
80014254:	00021400 	sll	v0,v0,0x10
80014258:	00021403 	sra	v0,v0,0x10
8001425c:	a602000c 	sh	v0,12(s0)
80014260:	8e030064 	lw	v1,100(s0)
80014264:	2402dfff 	li	v0,-8193
80014268:	00621024 	and	v0,v1,v0
8001426c:	ae020064 	sw	v0,100(s0)
80014270:	8603000c 	lh	v1,12(s0)
80014274:	2402ffdf 	li	v0,-33
80014278:	00621024 	and	v0,v1,v0
8001427c:	00021400 	sll	v0,v0,0x10
80014280:	00021403 	sra	v0,v0,0x10
80014284:	a602000c 	sh	v0,12(s0)
80014288:	8602000c 	lh	v0,12(s0)
8001428c:	3042ffff 	andi	v0,v0,0xffff
80014290:	30420004 	andi	v0,v0,0x4
80014294:	14400024 	bnez	v0,80014328 <_ungetc_r+0x15c>
80014298:	00000000 	nop
8001429c:	8602000c 	lh	v0,12(s0)
800142a0:	3042ffff 	andi	v0,v0,0xffff
800142a4:	30420010 	andi	v0,v0,0x10
800142a8:	14400004 	bnez	v0,800142bc <_ungetc_r+0xf0>
800142ac:	00000000 	nop
800142b0:	2402ffff 	li	v0,-1
800142b4:	10000065 	b	8001444c <_ungetc_r+0x280>
800142b8:	00000000 	nop
800142bc:	8602000c 	lh	v0,12(s0)
800142c0:	3042ffff 	andi	v0,v0,0xffff
800142c4:	30420008 	andi	v0,v0,0x8
800142c8:	10400012 	beqz	v0,80014314 <_ungetc_r+0x148>
800142cc:	00000000 	nop
800142d0:	02002825 	move	a1,s0
800142d4:	8fc40040 	lw	a0,64(s8)
800142d8:	0c000b45 	jal	80002d14 <_fflush_r>
800142dc:	00000000 	nop
800142e0:	10400004 	beqz	v0,800142f4 <_ungetc_r+0x128>
800142e4:	00000000 	nop
800142e8:	2402ffff 	li	v0,-1
800142ec:	10000057 	b	8001444c <_ungetc_r+0x280>
800142f0:	00000000 	nop
800142f4:	8603000c 	lh	v1,12(s0)
800142f8:	2402fff7 	li	v0,-9
800142fc:	00621024 	and	v0,v1,v0
80014300:	00021400 	sll	v0,v0,0x10
80014304:	00021403 	sra	v0,v0,0x10
80014308:	a602000c 	sh	v0,12(s0)
8001430c:	ae000008 	sw	zero,8(s0)
80014310:	ae000018 	sw	zero,24(s0)
80014314:	8602000c 	lh	v0,12(s0)
80014318:	34420004 	ori	v0,v0,0x4
8001431c:	00021400 	sll	v0,v0,0x10
80014320:	00021403 	sra	v0,v0,0x10
80014324:	a602000c 	sh	v0,12(s0)
80014328:	8fc20048 	lw	v0,72(s8)
8001432c:	304200ff 	andi	v0,v0,0xff
80014330:	afc20048 	sw	v0,72(s8)
80014334:	8e020030 	lw	v0,48(s0)
80014338:	1040001c 	beqz	v0,800143ac <_ungetc_r+0x1e0>
8001433c:	00000000 	nop
80014340:	8e030004 	lw	v1,4(s0)
80014344:	8e020034 	lw	v0,52(s0)
80014348:	0062102a 	slt	v0,v1,v0
8001434c:	1440000a 	bnez	v0,80014378 <_ungetc_r+0x1ac>
80014350:	00000000 	nop
80014354:	02002825 	move	a1,s0
80014358:	8fc40040 	lw	a0,64(s8)
8001435c:	0c005023 	jal	8001408c <__submore>
80014360:	00000000 	nop
80014364:	10400004 	beqz	v0,80014378 <_ungetc_r+0x1ac>
80014368:	00000000 	nop
8001436c:	2402ffff 	li	v0,-1
80014370:	10000036 	b	8001444c <_ungetc_r+0x280>
80014374:	00000000 	nop
80014378:	8e020000 	lw	v0,0(s0)
8001437c:	2442ffff 	addiu	v0,v0,-1
80014380:	ae020000 	sw	v0,0(s0)
80014384:	8e020000 	lw	v0,0(s0)
80014388:	8fc30048 	lw	v1,72(s8)
8001438c:	306300ff 	andi	v1,v1,0xff
80014390:	a0430000 	sb	v1,0(v0)
80014394:	8e020004 	lw	v0,4(s0)
80014398:	24420001 	addiu	v0,v0,1
8001439c:	ae020004 	sw	v0,4(s0)
800143a0:	8fc20048 	lw	v0,72(s8)
800143a4:	10000029 	b	8001444c <_ungetc_r+0x280>
800143a8:	00000000 	nop
800143ac:	8e020010 	lw	v0,16(s0)
800143b0:	10400016 	beqz	v0,8001440c <_ungetc_r+0x240>
800143b4:	00000000 	nop
800143b8:	8e030000 	lw	v1,0(s0)
800143bc:	8e020010 	lw	v0,16(s0)
800143c0:	0043102b 	sltu	v0,v0,v1
800143c4:	10400011 	beqz	v0,8001440c <_ungetc_r+0x240>
800143c8:	00000000 	nop
800143cc:	8e020000 	lw	v0,0(s0)
800143d0:	2442ffff 	addiu	v0,v0,-1
800143d4:	90420000 	lbu	v0,0(v0)
800143d8:	00401825 	move	v1,v0
800143dc:	8fc20048 	lw	v0,72(s8)
800143e0:	1443000a 	bne	v0,v1,8001440c <_ungetc_r+0x240>
800143e4:	00000000 	nop
800143e8:	8e020000 	lw	v0,0(s0)
800143ec:	2442ffff 	addiu	v0,v0,-1
800143f0:	ae020000 	sw	v0,0(s0)
800143f4:	8e020004 	lw	v0,4(s0)
800143f8:	24420001 	addiu	v0,v0,1
800143fc:	ae020004 	sw	v0,4(s0)
80014400:	8fc20048 	lw	v0,72(s8)
80014404:	10000011 	b	8001444c <_ungetc_r+0x280>
80014408:	00000000 	nop
8001440c:	8e020004 	lw	v0,4(s0)
80014410:	ae02003c 	sw	v0,60(s0)
80014414:	8e020000 	lw	v0,0(s0)
80014418:	ae020038 	sw	v0,56(s0)
8001441c:	26020040 	addiu	v0,s0,64
80014420:	ae020030 	sw	v0,48(s0)
80014424:	24020003 	li	v0,3
80014428:	ae020034 	sw	v0,52(s0)
8001442c:	8fc20048 	lw	v0,72(s8)
80014430:	304200ff 	andi	v0,v0,0xff
80014434:	a2020042 	sb	v0,66(s0)
80014438:	26020042 	addiu	v0,s0,66
8001443c:	ae020000 	sw	v0,0(s0)
80014440:	24020001 	li	v0,1
80014444:	ae020004 	sw	v0,4(s0)
80014448:	8fc20048 	lw	v0,72(s8)
8001444c:	03c0e825 	move	sp,s8
80014450:	dfbf0038 	ld	ra,56(sp)
80014454:	dfbe0030 	ld	s8,48(sp)
80014458:	dfb00028 	ld	s0,40(sp)
8001445c:	27bd0040 	addiu	sp,sp,64
80014460:	03e00008 	jr	ra
80014464:	00000000 	nop

80014468 <ungetc>:
80014468:	27bdffd0 	addiu	sp,sp,-48
8001446c:	ffbf0028 	sd	ra,40(sp)
80014470:	ffbe0020 	sd	s8,32(sp)
80014474:	03a0f025 	move	s8,sp
80014478:	afc40030 	sw	a0,48(s8)
8001447c:	00a01825 	move	v1,a1
80014480:	8f828028 	lw	v0,-32728(gp)
80014484:	00603025 	move	a2,v1
80014488:	8fc50030 	lw	a1,48(s8)
8001448c:	00402025 	move	a0,v0
80014490:	0c005073 	jal	800141cc <_ungetc_r>
80014494:	00000000 	nop
80014498:	03c0e825 	move	sp,s8
8001449c:	dfbf0028 	ld	ra,40(sp)
800144a0:	dfbe0020 	ld	s8,32(sp)
800144a4:	27bd0030 	addiu	sp,sp,48
800144a8:	03e00008 	jr	ra
800144ac:	00000000 	nop

800144b0 <__sprint_r>:
800144b0:	27bdffb0 	addiu	sp,sp,-80
800144b4:	ffbf0048 	sd	ra,72(sp)
800144b8:	ffbe0040 	sd	s8,64(sp)
800144bc:	ffb10038 	sd	s1,56(sp)
800144c0:	ffb00030 	sd	s0,48(sp)
800144c4:	03a0f025 	move	s8,sp
800144c8:	afc40050 	sw	a0,80(s8)
800144cc:	afc50058 	sw	a1,88(s8)
800144d0:	00c08025 	move	s0,a2
800144d4:	00008825 	move	s1,zero
800144d8:	8e020008 	lw	v0,8(s0)
800144dc:	14400005 	bnez	v0,800144f4 <__sprint_r+0x44>
800144e0:	00000000 	nop
800144e4:	ae000004 	sw	zero,4(s0)
800144e8:	00001025 	move	v0,zero
800144ec:	10000043 	b	800145fc <__sprint_r+0x14c>
800144f0:	00000000 	nop
800144f4:	8fc20058 	lw	v0,88(s8)
800144f8:	8c420064 	lw	v0,100(v0)
800144fc:	30422000 	andi	v0,v0,0x2000
80014500:	10400035 	beqz	v0,800145d8 <__sprint_r+0x128>
80014504:	00000000 	nop
80014508:	8e020000 	lw	v0,0(s0)
8001450c:	afc20020 	sw	v0,32(s8)
80014510:	1000002c 	b	800145c4 <__sprint_r+0x114>
80014514:	00000000 	nop
80014518:	8fc20020 	lw	v0,32(s8)
8001451c:	8c420000 	lw	v0,0(v0)
80014520:	afc20028 	sw	v0,40(s8)
80014524:	8fc20020 	lw	v0,32(s8)
80014528:	8c420004 	lw	v0,4(v0)
8001452c:	00021082 	srl	v0,v0,0x2
80014530:	afc2002c 	sw	v0,44(s8)
80014534:	afc00024 	sw	zero,36(s8)
80014538:	10000015 	b	80014590 <__sprint_r+0xe0>
8001453c:	00000000 	nop
80014540:	8fc20024 	lw	v0,36(s8)
80014544:	00021080 	sll	v0,v0,0x2
80014548:	8fc30028 	lw	v1,40(s8)
8001454c:	00621021 	addu	v0,v1,v0
80014550:	8c420000 	lw	v0,0(v0)
80014554:	8fc60058 	lw	a2,88(s8)
80014558:	00402825 	move	a1,v0
8001455c:	8fc40050 	lw	a0,80(s8)
80014560:	0c005836 	jal	800160d8 <_fputwc_r>
80014564:	00000000 	nop
80014568:	00401825 	move	v1,v0
8001456c:	2402ffff 	li	v0,-1
80014570:	14620004 	bne	v1,v0,80014584 <__sprint_r+0xd4>
80014574:	00000000 	nop
80014578:	2411ffff 	li	s1,-1
8001457c:	1000001c 	b	800145f0 <__sprint_r+0x140>
80014580:	00000000 	nop
80014584:	8fc20024 	lw	v0,36(s8)
80014588:	24420001 	addiu	v0,v0,1
8001458c:	afc20024 	sw	v0,36(s8)
80014590:	8fc30024 	lw	v1,36(s8)
80014594:	8fc2002c 	lw	v0,44(s8)
80014598:	0062102a 	slt	v0,v1,v0
8001459c:	1440ffe8 	bnez	v0,80014540 <__sprint_r+0x90>
800145a0:	00000000 	nop
800145a4:	8e030008 	lw	v1,8(s0)
800145a8:	8fc2002c 	lw	v0,44(s8)
800145ac:	00021080 	sll	v0,v0,0x2
800145b0:	00621023 	subu	v0,v1,v0
800145b4:	ae020008 	sw	v0,8(s0)
800145b8:	8fc20020 	lw	v0,32(s8)
800145bc:	24420008 	addiu	v0,v0,8
800145c0:	afc20020 	sw	v0,32(s8)
800145c4:	8e020008 	lw	v0,8(s0)
800145c8:	1440ffd3 	bnez	v0,80014518 <__sprint_r+0x68>
800145cc:	00000000 	nop
800145d0:	10000007 	b	800145f0 <__sprint_r+0x140>
800145d4:	00000000 	nop
800145d8:	02003025 	move	a2,s0
800145dc:	8fc50058 	lw	a1,88(s8)
800145e0:	8fc40050 	lw	a0,80(s8)
800145e4:	0c003737 	jal	8000dcdc <__sfvwrite_r>
800145e8:	00000000 	nop
800145ec:	00408825 	move	s1,v0
800145f0:	ae000008 	sw	zero,8(s0)
800145f4:	ae000004 	sw	zero,4(s0)
800145f8:	02201025 	move	v0,s1
800145fc:	03c0e825 	move	sp,s8
80014600:	dfbf0048 	ld	ra,72(sp)
80014604:	dfbe0040 	ld	s8,64(sp)
80014608:	dfb10038 	ld	s1,56(sp)
8001460c:	dfb00030 	ld	s0,48(sp)
80014610:	27bd0050 	addiu	sp,sp,80
80014614:	03e00008 	jr	ra
80014618:	00000000 	nop

8001461c <__sbprintf>:
8001461c:	27bdfb58 	addiu	sp,sp,-1192
80014620:	ffbf04a0 	sd	ra,1184(sp)
80014624:	ffbe0498 	sd	s8,1176(sp)
80014628:	ffb00490 	sd	s0,1168(sp)
8001462c:	03a0f025 	move	s8,sp
80014630:	afc404a8 	sw	a0,1192(s8)
80014634:	00a08025 	move	s0,a1
80014638:	afc604b8 	sw	a2,1208(s8)
8001463c:	afc704c0 	sw	a3,1216(s8)
80014640:	8603000c 	lh	v1,12(s0)
80014644:	2402fffd 	li	v0,-3
80014648:	00621024 	and	v0,v1,v0
8001464c:	00021400 	sll	v0,v0,0x10
80014650:	00021403 	sra	v0,v0,0x10
80014654:	a7c20034 	sh	v0,52(s8)
80014658:	8e020064 	lw	v0,100(s0)
8001465c:	afc2008c 	sw	v0,140(s8)
80014660:	8602000e 	lh	v0,14(s0)
80014664:	a7c20036 	sh	v0,54(s8)
80014668:	8e02001c 	lw	v0,28(s0)
8001466c:	afc20044 	sw	v0,68(s8)
80014670:	8e020024 	lw	v0,36(s0)
80014674:	afc2004c 	sw	v0,76(s8)
80014678:	27c20090 	addiu	v0,s8,144
8001467c:	afc20028 	sw	v0,40(s8)
80014680:	8fc20028 	lw	v0,40(s8)
80014684:	afc20038 	sw	v0,56(s8)
80014688:	24020400 	li	v0,1024
8001468c:	afc20030 	sw	v0,48(s8)
80014690:	8fc20030 	lw	v0,48(s8)
80014694:	afc2003c 	sw	v0,60(s8)
80014698:	afc00040 	sw	zero,64(s8)
8001469c:	27c20028 	addiu	v0,s8,40
800146a0:	8fc704c0 	lw	a3,1216(s8)
800146a4:	8fc604b8 	lw	a2,1208(s8)
800146a8:	00402825 	move	a1,v0
800146ac:	8fc404a8 	lw	a0,1192(s8)
800146b0:	0c0051e3 	jal	8001478c <_vfiprintf_r>
800146b4:	00000000 	nop
800146b8:	afc20020 	sw	v0,32(s8)
800146bc:	8fc20020 	lw	v0,32(s8)
800146c0:	0440000a 	bltz	v0,800146ec <__sbprintf+0xd0>
800146c4:	00000000 	nop
800146c8:	27c20028 	addiu	v0,s8,40
800146cc:	00402825 	move	a1,v0
800146d0:	8fc404a8 	lw	a0,1192(s8)
800146d4:	0c000b45 	jal	80002d14 <_fflush_r>
800146d8:	00000000 	nop
800146dc:	10400003 	beqz	v0,800146ec <__sbprintf+0xd0>
800146e0:	00000000 	nop
800146e4:	2402ffff 	li	v0,-1
800146e8:	afc20020 	sw	v0,32(s8)
800146ec:	87c20034 	lh	v0,52(s8)
800146f0:	3042ffff 	andi	v0,v0,0xffff
800146f4:	30420040 	andi	v0,v0,0x40
800146f8:	10400006 	beqz	v0,80014714 <__sbprintf+0xf8>
800146fc:	00000000 	nop
80014700:	8602000c 	lh	v0,12(s0)
80014704:	34420040 	ori	v0,v0,0x40
80014708:	00021400 	sll	v0,v0,0x10
8001470c:	00021403 	sra	v0,v0,0x10
80014710:	a602000c 	sh	v0,12(s0)
80014714:	8fc20020 	lw	v0,32(s8)
80014718:	03c0e825 	move	sp,s8
8001471c:	dfbf04a0 	ld	ra,1184(sp)
80014720:	dfbe0498 	ld	s8,1176(sp)
80014724:	dfb00490 	ld	s0,1168(sp)
80014728:	27bd04a8 	addiu	sp,sp,1192
8001472c:	03e00008 	jr	ra
80014730:	00000000 	nop

80014734 <vfiprintf>:
80014734:	27bdffc8 	addiu	sp,sp,-56
80014738:	ffbf0030 	sd	ra,48(sp)
8001473c:	ffbe0028 	sd	s8,40(sp)
80014740:	03a0f025 	move	s8,sp
80014744:	afc40038 	sw	a0,56(s8)
80014748:	afc50040 	sw	a1,64(s8)
8001474c:	afc60048 	sw	a2,72(s8)
80014750:	8f828028 	lw	v0,-32728(gp)
80014754:	8fc70048 	lw	a3,72(s8)
80014758:	8fc60040 	lw	a2,64(s8)
8001475c:	8fc50038 	lw	a1,56(s8)
80014760:	00402025 	move	a0,v0
80014764:	0c0051e3 	jal	8001478c <_vfiprintf_r>
80014768:	00000000 	nop
8001476c:	afc20020 	sw	v0,32(s8)
80014770:	8fc20020 	lw	v0,32(s8)
80014774:	03c0e825 	move	sp,s8
80014778:	dfbf0030 	ld	ra,48(sp)
8001477c:	dfbe0028 	ld	s8,40(sp)
80014780:	27bd0038 	addiu	sp,sp,56
80014784:	03e00008 	jr	ra
80014788:	00000000 	nop

8001478c <_vfiprintf_r>:
8001478c:	27bdfee0 	addiu	sp,sp,-288
80014790:	ffbf0118 	sd	ra,280(sp)
80014794:	ffbe0110 	sd	s8,272(sp)
80014798:	ffb50108 	sd	s5,264(sp)
8001479c:	ffb40100 	sd	s4,256(sp)
800147a0:	ffb300f8 	sd	s3,248(sp)
800147a4:	ffb200f0 	sd	s2,240(sp)
800147a8:	ffb100e8 	sd	s1,232(sp)
800147ac:	ffb000e0 	sd	s0,224(sp)
800147b0:	03a0f025 	move	s8,sp
800147b4:	afc40120 	sw	a0,288(s8)
800147b8:	afc50128 	sw	a1,296(s8)
800147bc:	afc60130 	sw	a2,304(s8)
800147c0:	afc70138 	sw	a3,312(s8)
800147c4:	afc00048 	sw	zero,72(s8)
800147c8:	afc0004c 	sw	zero,76(s8)
800147cc:	8fc20120 	lw	v0,288(s8)
800147d0:	afc20050 	sw	v0,80(s8)
800147d4:	8fc20050 	lw	v0,80(s8)
800147d8:	10400008 	beqz	v0,800147fc <_vfiprintf_r+0x70>
800147dc:	00000000 	nop
800147e0:	8fc20050 	lw	v0,80(s8)
800147e4:	8c420038 	lw	v0,56(v0)
800147e8:	14400004 	bnez	v0,800147fc <_vfiprintf_r+0x70>
800147ec:	00000000 	nop
800147f0:	8fc40050 	lw	a0,80(s8)
800147f4:	0c000cc3 	jal	8000330c <__sinit>
800147f8:	00000000 	nop
800147fc:	8fc20128 	lw	v0,296(s8)
80014800:	8442000c 	lh	v0,12(v0)
80014804:	3042ffff 	andi	v0,v0,0xffff
80014808:	30422000 	andi	v0,v0,0x2000
8001480c:	1440000e 	bnez	v0,80014848 <_vfiprintf_r+0xbc>
80014810:	00000000 	nop
80014814:	8fc20128 	lw	v0,296(s8)
80014818:	8442000c 	lh	v0,12(v0)
8001481c:	34422000 	ori	v0,v0,0x2000
80014820:	00021c00 	sll	v1,v0,0x10
80014824:	00031c03 	sra	v1,v1,0x10
80014828:	8fc20128 	lw	v0,296(s8)
8001482c:	a443000c 	sh	v1,12(v0)
80014830:	8fc20128 	lw	v0,296(s8)
80014834:	8c430064 	lw	v1,100(v0)
80014838:	2402dfff 	li	v0,-8193
8001483c:	00621824 	and	v1,v1,v0
80014840:	8fc20128 	lw	v0,296(s8)
80014844:	ac430064 	sw	v1,100(v0)
80014848:	8fc20128 	lw	v0,296(s8)
8001484c:	8442000c 	lh	v0,12(v0)
80014850:	3042ffff 	andi	v0,v0,0xffff
80014854:	30420008 	andi	v0,v0,0x8
80014858:	10400005 	beqz	v0,80014870 <_vfiprintf_r+0xe4>
8001485c:	00000000 	nop
80014860:	8fc20128 	lw	v0,296(s8)
80014864:	8c420010 	lw	v0,16(v0)
80014868:	1440000a 	bnez	v0,80014894 <_vfiprintf_r+0x108>
8001486c:	00000000 	nop
80014870:	8fc50128 	lw	a1,296(s8)
80014874:	8fc40120 	lw	a0,288(s8)
80014878:	0c002e52 	jal	8000b948 <__swsetup_r>
8001487c:	00000000 	nop
80014880:	10400004 	beqz	v0,80014894 <_vfiprintf_r+0x108>
80014884:	00000000 	nop
80014888:	2402ffff 	li	v0,-1
8001488c:	10000494 	b	80015ae0 <_vfiprintf_r+0x1354>
80014890:	00000000 	nop
80014894:	8fc20128 	lw	v0,296(s8)
80014898:	8442000c 	lh	v0,12(v0)
8001489c:	3042ffff 	andi	v0,v0,0xffff
800148a0:	3043001a 	andi	v1,v0,0x1a
800148a4:	2402000a 	li	v0,10
800148a8:	1462000e 	bne	v1,v0,800148e4 <_vfiprintf_r+0x158>
800148ac:	00000000 	nop
800148b0:	8fc20128 	lw	v0,296(s8)
800148b4:	8442000e 	lh	v0,14(v0)
800148b8:	0440000a 	bltz	v0,800148e4 <_vfiprintf_r+0x158>
800148bc:	00000000 	nop
800148c0:	8fc20138 	lw	v0,312(s8)
800148c4:	00403825 	move	a3,v0
800148c8:	8fc60130 	lw	a2,304(s8)
800148cc:	8fc50128 	lw	a1,296(s8)
800148d0:	8fc40120 	lw	a0,288(s8)
800148d4:	0c005187 	jal	8001461c <__sbprintf>
800148d8:	00000000 	nop
800148dc:	10000480 	b	80015ae0 <_vfiprintf_r+0x1354>
800148e0:	00000000 	nop
800148e4:	8fd40130 	lw	s4,304(s8)
800148e8:	27d00070 	addiu	s0,s8,112
800148ec:	afd00060 	sw	s0,96(s8)
800148f0:	afc00068 	sw	zero,104(s8)
800148f4:	afc00064 	sw	zero,100(s8)
800148f8:	afc00020 	sw	zero,32(s8)
800148fc:	02809025 	move	s2,s4
80014900:	10000002 	b	8001490c <_vfiprintf_r+0x180>
80014904:	00000000 	nop
80014908:	26940001 	addiu	s4,s4,1
8001490c:	82820000 	lb	v0,0(s4)
80014910:	10400005 	beqz	v0,80014928 <_vfiprintf_r+0x19c>
80014914:	00000000 	nop
80014918:	82830000 	lb	v1,0(s4)
8001491c:	24020025 	li	v0,37
80014920:	1462fff9 	bne	v1,v0,80014908 <_vfiprintf_r+0x17c>
80014924:	00000000 	nop
80014928:	02928823 	subu	s1,s4,s2
8001492c:	1220001c 	beqz	s1,800149a0 <_vfiprintf_r+0x214>
80014930:	00000000 	nop
80014934:	ae120000 	sw	s2,0(s0)
80014938:	02201025 	move	v0,s1
8001493c:	ae020004 	sw	v0,4(s0)
80014940:	8fc20068 	lw	v0,104(s8)
80014944:	02201825 	move	v1,s1
80014948:	00431021 	addu	v0,v0,v1
8001494c:	afc20068 	sw	v0,104(s8)
80014950:	26100008 	addiu	s0,s0,8
80014954:	8fc20064 	lw	v0,100(s8)
80014958:	24420001 	addiu	v0,v0,1
8001495c:	afc20064 	sw	v0,100(s8)
80014960:	8fc20064 	lw	v0,100(s8)
80014964:	28420008 	slti	v0,v0,8
80014968:	1440000a 	bnez	v0,80014994 <_vfiprintf_r+0x208>
8001496c:	00000000 	nop
80014970:	27c20060 	addiu	v0,s8,96
80014974:	00403025 	move	a2,v0
80014978:	8fc50128 	lw	a1,296(s8)
8001497c:	8fc40120 	lw	a0,288(s8)
80014980:	0c00512c 	jal	800144b0 <__sprint_r>
80014984:	00000000 	nop
80014988:	1440041c 	bnez	v0,800159fc <_vfiprintf_r+0x1270>
8001498c:	00000000 	nop
80014990:	27d00070 	addiu	s0,s8,112
80014994:	8fc20020 	lw	v0,32(s8)
80014998:	00511021 	addu	v0,v0,s1
8001499c:	afc20020 	sw	v0,32(s8)
800149a0:	82820000 	lb	v0,0(s4)
800149a4:	10400403 	beqz	v0,800159b4 <_vfiprintf_r+0x1228>
800149a8:	00000000 	nop
800149ac:	afd40054 	sw	s4,84(s8)
800149b0:	26940001 	addiu	s4,s4,1
800149b4:	00008825 	move	s1,zero
800149b8:	afc0003c 	sw	zero,60(s8)
800149bc:	afc00024 	sw	zero,36(s8)
800149c0:	2402ffff 	li	v0,-1
800149c4:	afc20028 	sw	v0,40(s8)
800149c8:	a3c0005c 	sb	zero,92(s8)
800149cc:	02801025 	move	v0,s4
800149d0:	24540001 	addiu	s4,v0,1
800149d4:	80420000 	lb	v0,0(v0)
800149d8:	0040a825 	move	s5,v0
800149dc:	26a2ffe0 	addiu	v0,s5,-32
800149e0:	2c430059 	sltiu	v1,v0,89
800149e4:	10600268 	beqz	v1,80015388 <_vfiprintf_r+0xbfc>
800149e8:	00000000 	nop
800149ec:	00021880 	sll	v1,v0,0x2
800149f0:	3c028002 	lui	v0,0x8002
800149f4:	2442ac5c 	addiu	v0,v0,-21412
800149f8:	00621021 	addu	v0,v1,v0
800149fc:	8c420000 	lw	v0,0(v0)
80014a00:	00400008 	jr	v0
80014a04:	00000000 	nop
80014a08:	83c2005c 	lb	v0,92(s8)
80014a0c:	1440ffef 	bnez	v0,800149cc <_vfiprintf_r+0x240>
80014a10:	00000000 	nop
80014a14:	24020020 	li	v0,32
80014a18:	a3c2005c 	sb	v0,92(s8)
80014a1c:	1000ffeb 	b	800149cc <_vfiprintf_r+0x240>
80014a20:	00000000 	nop
80014a24:	36310001 	ori	s1,s1,0x1
80014a28:	1000ffe8 	b	800149cc <_vfiprintf_r+0x240>
80014a2c:	00000000 	nop
80014a30:	8fc20138 	lw	v0,312(s8)
80014a34:	24430008 	addiu	v1,v0,8
80014a38:	afc30138 	sw	v1,312(s8)
80014a3c:	24420004 	addiu	v0,v0,4
80014a40:	8c420000 	lw	v0,0(v0)
80014a44:	afc20024 	sw	v0,36(s8)
80014a48:	8fc20024 	lw	v0,36(s8)
80014a4c:	04400003 	bltz	v0,80014a5c <_vfiprintf_r+0x2d0>
80014a50:	00000000 	nop
80014a54:	1000ffdd 	b	800149cc <_vfiprintf_r+0x240>
80014a58:	00000000 	nop
80014a5c:	8fc20024 	lw	v0,36(s8)
80014a60:	00021023 	negu	v0,v0
80014a64:	afc20024 	sw	v0,36(s8)
80014a68:	36310004 	ori	s1,s1,0x4
80014a6c:	1000ffd7 	b	800149cc <_vfiprintf_r+0x240>
80014a70:	00000000 	nop
80014a74:	2402002b 	li	v0,43
80014a78:	a3c2005c 	sb	v0,92(s8)
80014a7c:	1000ffd3 	b	800149cc <_vfiprintf_r+0x240>
80014a80:	00000000 	nop
80014a84:	02801025 	move	v0,s4
80014a88:	24540001 	addiu	s4,v0,1
80014a8c:	80420000 	lb	v0,0(v0)
80014a90:	0040a825 	move	s5,v0
80014a94:	2402002a 	li	v0,42
80014a98:	16a2000e 	bne	s5,v0,80014ad4 <_vfiprintf_r+0x348>
80014a9c:	00000000 	nop
80014aa0:	8fc20138 	lw	v0,312(s8)
80014aa4:	24430008 	addiu	v1,v0,8
80014aa8:	afc30138 	sw	v1,312(s8)
80014aac:	24420004 	addiu	v0,v0,4
80014ab0:	8c420000 	lw	v0,0(v0)
80014ab4:	afc20028 	sw	v0,40(s8)
80014ab8:	8fc20028 	lw	v0,40(s8)
80014abc:	0441ffc3 	bgez	v0,800149cc <_vfiprintf_r+0x240>
80014ac0:	00000000 	nop
80014ac4:	2402ffff 	li	v0,-1
80014ac8:	afc20028 	sw	v0,40(s8)
80014acc:	1000ffbf 	b	800149cc <_vfiprintf_r+0x240>
80014ad0:	00000000 	nop
80014ad4:	00009825 	move	s3,zero
80014ad8:	1000000c 	b	80014b0c <_vfiprintf_r+0x380>
80014adc:	00000000 	nop
80014ae0:	02601025 	move	v0,s3
80014ae4:	00021080 	sll	v0,v0,0x2
80014ae8:	00531021 	addu	v0,v0,s3
80014aec:	00021040 	sll	v0,v0,0x1
80014af0:	00401825 	move	v1,v0
80014af4:	26a2ffd0 	addiu	v0,s5,-48
80014af8:	00629821 	addu	s3,v1,v0
80014afc:	02801025 	move	v0,s4
80014b00:	24540001 	addiu	s4,v0,1
80014b04:	80420000 	lb	v0,0(v0)
80014b08:	0040a825 	move	s5,v0
80014b0c:	26a2ffd0 	addiu	v0,s5,-48
80014b10:	2c42000a 	sltiu	v0,v0,10
80014b14:	1440fff2 	bnez	v0,80014ae0 <_vfiprintf_r+0x354>
80014b18:	00000000 	nop
80014b1c:	02601025 	move	v0,s3
80014b20:	04410002 	bgez	v0,80014b2c <_vfiprintf_r+0x3a0>
80014b24:	00000000 	nop
80014b28:	2402ffff 	li	v0,-1
80014b2c:	afc20028 	sw	v0,40(s8)
80014b30:	1000ffaa 	b	800149dc <_vfiprintf_r+0x250>
80014b34:	00000000 	nop
80014b38:	36310080 	ori	s1,s1,0x80
80014b3c:	1000ffa3 	b	800149cc <_vfiprintf_r+0x240>
80014b40:	00000000 	nop
80014b44:	00009825 	move	s3,zero
80014b48:	02601025 	move	v0,s3
80014b4c:	00021080 	sll	v0,v0,0x2
80014b50:	00531021 	addu	v0,v0,s3
80014b54:	00021040 	sll	v0,v0,0x1
80014b58:	00401825 	move	v1,v0
80014b5c:	26a2ffd0 	addiu	v0,s5,-48
80014b60:	00629821 	addu	s3,v1,v0
80014b64:	02801025 	move	v0,s4
80014b68:	24540001 	addiu	s4,v0,1
80014b6c:	80420000 	lb	v0,0(v0)
80014b70:	0040a825 	move	s5,v0
80014b74:	26a2ffd0 	addiu	v0,s5,-48
80014b78:	2c42000a 	sltiu	v0,v0,10
80014b7c:	1440fff2 	bnez	v0,80014b48 <_vfiprintf_r+0x3bc>
80014b80:	00000000 	nop
80014b84:	afd30024 	sw	s3,36(s8)
80014b88:	1000ff94 	b	800149dc <_vfiprintf_r+0x250>
80014b8c:	00000000 	nop
80014b90:	36310040 	ori	s1,s1,0x40
80014b94:	1000ff8d 	b	800149cc <_vfiprintf_r+0x240>
80014b98:	00000000 	nop
80014b9c:	82830000 	lb	v1,0(s4)
80014ba0:	2402006c 	li	v0,108
80014ba4:	14620005 	bne	v1,v0,80014bbc <_vfiprintf_r+0x430>
80014ba8:	00000000 	nop
80014bac:	26940001 	addiu	s4,s4,1
80014bb0:	36310020 	ori	s1,s1,0x20
80014bb4:	1000ff85 	b	800149cc <_vfiprintf_r+0x240>
80014bb8:	00000000 	nop
80014bbc:	36310010 	ori	s1,s1,0x10
80014bc0:	1000ff82 	b	800149cc <_vfiprintf_r+0x240>
80014bc4:	00000000 	nop
80014bc8:	36310020 	ori	s1,s1,0x20
80014bcc:	1000ff7f 	b	800149cc <_vfiprintf_r+0x240>
80014bd0:	00000000 	nop
80014bd4:	27d200b0 	addiu	s2,s8,176
80014bd8:	8fc20138 	lw	v0,312(s8)
80014bdc:	24430008 	addiu	v1,v0,8
80014be0:	afc30138 	sw	v1,312(s8)
80014be4:	24420004 	addiu	v0,v0,4
80014be8:	8c420000 	lw	v0,0(v0)
80014bec:	00021600 	sll	v0,v0,0x18
80014bf0:	00021603 	sra	v0,v0,0x18
80014bf4:	a2420000 	sb	v0,0(s2)
80014bf8:	24020001 	li	v0,1
80014bfc:	afc20044 	sw	v0,68(s8)
80014c00:	a3c0005c 	sb	zero,92(s8)
80014c04:	100001ea 	b	800153b0 <_vfiprintf_r+0xc24>
80014c08:	00000000 	nop
80014c0c:	36310010 	ori	s1,s1,0x10
80014c10:	32220020 	andi	v0,s1,0x20
80014c14:	10400007 	beqz	v0,80014c34 <_vfiprintf_r+0x4a8>
80014c18:	00000000 	nop
80014c1c:	8fc20138 	lw	v0,312(s8)
80014c20:	24430008 	addiu	v1,v0,8
80014c24:	afc30138 	sw	v1,312(s8)
80014c28:	dc420000 	ld	v0,0(v0)
80014c2c:	1000001e 	b	80014ca8 <_vfiprintf_r+0x51c>
80014c30:	00000000 	nop
80014c34:	32220010 	andi	v0,s1,0x10
80014c38:	10400008 	beqz	v0,80014c5c <_vfiprintf_r+0x4d0>
80014c3c:	00000000 	nop
80014c40:	8fc20138 	lw	v0,312(s8)
80014c44:	24430008 	addiu	v1,v0,8
80014c48:	afc30138 	sw	v1,312(s8)
80014c4c:	24420004 	addiu	v0,v0,4
80014c50:	8c420000 	lw	v0,0(v0)
80014c54:	10000014 	b	80014ca8 <_vfiprintf_r+0x51c>
80014c58:	00000000 	nop
80014c5c:	32220040 	andi	v0,s1,0x40
80014c60:	1040000c 	beqz	v0,80014c94 <_vfiprintf_r+0x508>
80014c64:	00000000 	nop
80014c68:	8fc20138 	lw	v0,312(s8)
80014c6c:	24430008 	addiu	v1,v0,8
80014c70:	afc30138 	sw	v1,312(s8)
80014c74:	24420004 	addiu	v0,v0,4
80014c78:	8c420000 	lw	v0,0(v0)
80014c7c:	00021400 	sll	v0,v0,0x10
80014c80:	00021403 	sra	v0,v0,0x10
80014c84:	0002143c 	dsll32	v0,v0,0x10
80014c88:	0002143f 	dsra32	v0,v0,0x10
80014c8c:	10000006 	b	80014ca8 <_vfiprintf_r+0x51c>
80014c90:	00000000 	nop
80014c94:	8fc20138 	lw	v0,312(s8)
80014c98:	24430008 	addiu	v1,v0,8
80014c9c:	afc30138 	sw	v1,312(s8)
80014ca0:	24420004 	addiu	v0,v0,4
80014ca4:	8c420000 	lw	v0,0(v0)
80014ca8:	ffc20030 	sd	v0,48(s8)
80014cac:	dfc20030 	ld	v0,48(s8)
80014cb0:	04410006 	bgez	v0,80014ccc <_vfiprintf_r+0x540>
80014cb4:	00000000 	nop
80014cb8:	dfc20030 	ld	v0,48(s8)
80014cbc:	0002102f 	dnegu	v0,v0
80014cc0:	ffc20030 	sd	v0,48(s8)
80014cc4:	2402002d 	li	v0,45
80014cc8:	a3c2005c 	sb	v0,92(s8)
80014ccc:	24020001 	li	v0,1
80014cd0:	afc20038 	sw	v0,56(s8)
80014cd4:	10000110 	b	80015118 <_vfiprintf_r+0x98c>
80014cd8:	00000000 	nop
80014cdc:	32220020 	andi	v0,s1,0x20
80014ce0:	1040000a 	beqz	v0,80014d0c <_vfiprintf_r+0x580>
80014ce4:	00000000 	nop
80014ce8:	8fc20138 	lw	v0,312(s8)
80014cec:	24430008 	addiu	v1,v0,8
80014cf0:	afc30138 	sw	v1,312(s8)
80014cf4:	24420004 	addiu	v0,v0,4
80014cf8:	8c420000 	lw	v0,0(v0)
80014cfc:	8fc30020 	lw	v1,32(s8)
80014d00:	fc430000 	sd	v1,0(v0)
80014d04:	10000329 	b	800159ac <_vfiprintf_r+0x1220>
80014d08:	00000000 	nop
80014d0c:	32220010 	andi	v0,s1,0x10
80014d10:	1040000a 	beqz	v0,80014d3c <_vfiprintf_r+0x5b0>
80014d14:	00000000 	nop
80014d18:	8fc20138 	lw	v0,312(s8)
80014d1c:	24430008 	addiu	v1,v0,8
80014d20:	afc30138 	sw	v1,312(s8)
80014d24:	24420004 	addiu	v0,v0,4
80014d28:	8c420000 	lw	v0,0(v0)
80014d2c:	8fc30020 	lw	v1,32(s8)
80014d30:	ac430000 	sw	v1,0(v0)
80014d34:	1000031d 	b	800159ac <_vfiprintf_r+0x1220>
80014d38:	00000000 	nop
80014d3c:	32220040 	andi	v0,s1,0x40
80014d40:	1040000c 	beqz	v0,80014d74 <_vfiprintf_r+0x5e8>
80014d44:	00000000 	nop
80014d48:	8fc20138 	lw	v0,312(s8)
80014d4c:	24430008 	addiu	v1,v0,8
80014d50:	afc30138 	sw	v1,312(s8)
80014d54:	24420004 	addiu	v0,v0,4
80014d58:	8c420000 	lw	v0,0(v0)
80014d5c:	8fc30020 	lw	v1,32(s8)
80014d60:	00031c00 	sll	v1,v1,0x10
80014d64:	00031c03 	sra	v1,v1,0x10
80014d68:	a4430000 	sh	v1,0(v0)
80014d6c:	1000030f 	b	800159ac <_vfiprintf_r+0x1220>
80014d70:	00000000 	nop
80014d74:	8fc20138 	lw	v0,312(s8)
80014d78:	24430008 	addiu	v1,v0,8
80014d7c:	afc30138 	sw	v1,312(s8)
80014d80:	24420004 	addiu	v0,v0,4
80014d84:	8c420000 	lw	v0,0(v0)
80014d88:	8fc30020 	lw	v1,32(s8)
80014d8c:	ac430000 	sw	v1,0(v0)
80014d90:	10000306 	b	800159ac <_vfiprintf_r+0x1220>
80014d94:	00000000 	nop
80014d98:	36310010 	ori	s1,s1,0x10
80014d9c:	32220020 	andi	v0,s1,0x20
80014da0:	10400007 	beqz	v0,80014dc0 <_vfiprintf_r+0x634>
80014da4:	00000000 	nop
80014da8:	8fc20138 	lw	v0,312(s8)
80014dac:	24430008 	addiu	v1,v0,8
80014db0:	afc30138 	sw	v1,312(s8)
80014db4:	dc420000 	ld	v0,0(v0)
80014db8:	10000020 	b	80014e3c <_vfiprintf_r+0x6b0>
80014dbc:	00000000 	nop
80014dc0:	32220010 	andi	v0,s1,0x10
80014dc4:	1040000a 	beqz	v0,80014df0 <_vfiprintf_r+0x664>
80014dc8:	00000000 	nop
80014dcc:	8fc20138 	lw	v0,312(s8)
80014dd0:	24430008 	addiu	v1,v0,8
80014dd4:	afc30138 	sw	v1,312(s8)
80014dd8:	24420004 	addiu	v0,v0,4
80014ddc:	8c420000 	lw	v0,0(v0)
80014de0:	0002103c 	dsll32	v0,v0,0x0
80014de4:	0002103e 	dsrl32	v0,v0,0x0
80014de8:	10000014 	b	80014e3c <_vfiprintf_r+0x6b0>
80014dec:	00000000 	nop
80014df0:	32220040 	andi	v0,s1,0x40
80014df4:	1040000a 	beqz	v0,80014e20 <_vfiprintf_r+0x694>
80014df8:	00000000 	nop
80014dfc:	8fc20138 	lw	v0,312(s8)
80014e00:	24430008 	addiu	v1,v0,8
80014e04:	afc30138 	sw	v1,312(s8)
80014e08:	24420004 	addiu	v0,v0,4
80014e0c:	8c420000 	lw	v0,0(v0)
80014e10:	3042ffff 	andi	v0,v0,0xffff
80014e14:	3042ffff 	andi	v0,v0,0xffff
80014e18:	10000008 	b	80014e3c <_vfiprintf_r+0x6b0>
80014e1c:	00000000 	nop
80014e20:	8fc20138 	lw	v0,312(s8)
80014e24:	24430008 	addiu	v1,v0,8
80014e28:	afc30138 	sw	v1,312(s8)
80014e2c:	24420004 	addiu	v0,v0,4
80014e30:	8c420000 	lw	v0,0(v0)
80014e34:	0002103c 	dsll32	v0,v0,0x0
80014e38:	0002103e 	dsrl32	v0,v0,0x0
80014e3c:	ffc20030 	sd	v0,48(s8)
80014e40:	afc00038 	sw	zero,56(s8)
80014e44:	100000b3 	b	80015114 <_vfiprintf_r+0x988>
80014e48:	00000000 	nop
80014e4c:	8fc20138 	lw	v0,312(s8)
80014e50:	24430008 	addiu	v1,v0,8
80014e54:	afc30138 	sw	v1,312(s8)
80014e58:	24420004 	addiu	v0,v0,4
80014e5c:	8c420000 	lw	v0,0(v0)
80014e60:	0002103c 	dsll32	v0,v0,0x0
80014e64:	0002103e 	dsrl32	v0,v0,0x0
80014e68:	ffc20030 	sd	v0,48(s8)
80014e6c:	24020002 	li	v0,2
80014e70:	afc20038 	sw	v0,56(s8)
80014e74:	3c028002 	lui	v0,0x8002
80014e78:	2442ac08 	addiu	v0,v0,-21496
80014e7c:	afc20048 	sw	v0,72(s8)
80014e80:	36310002 	ori	s1,s1,0x2
80014e84:	24020030 	li	v0,48
80014e88:	a3c200d8 	sb	v0,216(s8)
80014e8c:	24020078 	li	v0,120
80014e90:	a3c200d9 	sb	v0,217(s8)
80014e94:	1000009f 	b	80015114 <_vfiprintf_r+0x988>
80014e98:	00000000 	nop
80014e9c:	8fc20138 	lw	v0,312(s8)
80014ea0:	24430008 	addiu	v1,v0,8
80014ea4:	afc30138 	sw	v1,312(s8)
80014ea8:	24420004 	addiu	v0,v0,4
80014eac:	8c520000 	lw	s2,0(v0)
80014eb0:	a3c0005c 	sb	zero,92(s8)
80014eb4:	1640000b 	bnez	s2,80014ee4 <_vfiprintf_r+0x758>
80014eb8:	00000000 	nop
80014ebc:	3c028002 	lui	v0,0x8002
80014ec0:	2452ac20 	addiu	s2,v0,-21472
80014ec4:	8fc20028 	lw	v0,40(s8)
80014ec8:	2c430007 	sltiu	v1,v0,7
80014ecc:	14600002 	bnez	v1,80014ed8 <_vfiprintf_r+0x74c>
80014ed0:	00000000 	nop
80014ed4:	24020006 	li	v0,6
80014ed8:	afc20044 	sw	v0,68(s8)
80014edc:	10000134 	b	800153b0 <_vfiprintf_r+0xc24>
80014ee0:	00000000 	nop
80014ee4:	8fc20028 	lw	v0,40(s8)
80014ee8:	04400014 	bltz	v0,80014f3c <_vfiprintf_r+0x7b0>
80014eec:	00000000 	nop
80014ef0:	8fc20028 	lw	v0,40(s8)
80014ef4:	00403025 	move	a2,v0
80014ef8:	00002825 	move	a1,zero
80014efc:	02402025 	move	a0,s2
80014f00:	0c003adc 	jal	8000eb70 <memchr>
80014f04:	00000000 	nop
80014f08:	afc20058 	sw	v0,88(s8)
80014f0c:	8fc20058 	lw	v0,88(s8)
80014f10:	10400006 	beqz	v0,80014f2c <_vfiprintf_r+0x7a0>
80014f14:	00000000 	nop
80014f18:	8fc20058 	lw	v0,88(s8)
80014f1c:	00521023 	subu	v0,v0,s2
80014f20:	afc20044 	sw	v0,68(s8)
80014f24:	10000122 	b	800153b0 <_vfiprintf_r+0xc24>
80014f28:	00000000 	nop
80014f2c:	8fc20028 	lw	v0,40(s8)
80014f30:	afc20044 	sw	v0,68(s8)
80014f34:	1000011e 	b	800153b0 <_vfiprintf_r+0xc24>
80014f38:	00000000 	nop
80014f3c:	02402025 	move	a0,s2
80014f40:	0c001f1d 	jal	80007c74 <strlen>
80014f44:	00000000 	nop
80014f48:	afc20044 	sw	v0,68(s8)
80014f4c:	10000118 	b	800153b0 <_vfiprintf_r+0xc24>
80014f50:	00000000 	nop
80014f54:	36310010 	ori	s1,s1,0x10
80014f58:	32220020 	andi	v0,s1,0x20
80014f5c:	10400007 	beqz	v0,80014f7c <_vfiprintf_r+0x7f0>
80014f60:	00000000 	nop
80014f64:	8fc20138 	lw	v0,312(s8)
80014f68:	24430008 	addiu	v1,v0,8
80014f6c:	afc30138 	sw	v1,312(s8)
80014f70:	dc420000 	ld	v0,0(v0)
80014f74:	10000020 	b	80014ff8 <_vfiprintf_r+0x86c>
80014f78:	00000000 	nop
80014f7c:	32220010 	andi	v0,s1,0x10
80014f80:	1040000a 	beqz	v0,80014fac <_vfiprintf_r+0x820>
80014f84:	00000000 	nop
80014f88:	8fc20138 	lw	v0,312(s8)
80014f8c:	24430008 	addiu	v1,v0,8
80014f90:	afc30138 	sw	v1,312(s8)
80014f94:	24420004 	addiu	v0,v0,4
80014f98:	8c420000 	lw	v0,0(v0)
80014f9c:	0002103c 	dsll32	v0,v0,0x0
80014fa0:	0002103e 	dsrl32	v0,v0,0x0
80014fa4:	10000014 	b	80014ff8 <_vfiprintf_r+0x86c>
80014fa8:	00000000 	nop
80014fac:	32220040 	andi	v0,s1,0x40
80014fb0:	1040000a 	beqz	v0,80014fdc <_vfiprintf_r+0x850>
80014fb4:	00000000 	nop
80014fb8:	8fc20138 	lw	v0,312(s8)
80014fbc:	24430008 	addiu	v1,v0,8
80014fc0:	afc30138 	sw	v1,312(s8)
80014fc4:	24420004 	addiu	v0,v0,4
80014fc8:	8c420000 	lw	v0,0(v0)
80014fcc:	3042ffff 	andi	v0,v0,0xffff
80014fd0:	3042ffff 	andi	v0,v0,0xffff
80014fd4:	10000008 	b	80014ff8 <_vfiprintf_r+0x86c>
80014fd8:	00000000 	nop
80014fdc:	8fc20138 	lw	v0,312(s8)
80014fe0:	24430008 	addiu	v1,v0,8
80014fe4:	afc30138 	sw	v1,312(s8)
80014fe8:	24420004 	addiu	v0,v0,4
80014fec:	8c420000 	lw	v0,0(v0)
80014ff0:	0002103c 	dsll32	v0,v0,0x0
80014ff4:	0002103e 	dsrl32	v0,v0,0x0
80014ff8:	ffc20030 	sd	v0,48(s8)
80014ffc:	24020001 	li	v0,1
80015000:	afc20038 	sw	v0,56(s8)
80015004:	10000043 	b	80015114 <_vfiprintf_r+0x988>
80015008:	00000000 	nop
8001500c:	3c028002 	lui	v0,0x8002
80015010:	2442ac28 	addiu	v0,v0,-21464
80015014:	afc20048 	sw	v0,72(s8)
80015018:	10000004 	b	8001502c <_vfiprintf_r+0x8a0>
8001501c:	00000000 	nop
80015020:	3c028002 	lui	v0,0x8002
80015024:	2442ac08 	addiu	v0,v0,-21496
80015028:	afc20048 	sw	v0,72(s8)
8001502c:	32220020 	andi	v0,s1,0x20
80015030:	10400007 	beqz	v0,80015050 <_vfiprintf_r+0x8c4>
80015034:	00000000 	nop
80015038:	8fc20138 	lw	v0,312(s8)
8001503c:	24430008 	addiu	v1,v0,8
80015040:	afc30138 	sw	v1,312(s8)
80015044:	dc420000 	ld	v0,0(v0)
80015048:	10000020 	b	800150cc <_vfiprintf_r+0x940>
8001504c:	00000000 	nop
80015050:	32220010 	andi	v0,s1,0x10
80015054:	1040000a 	beqz	v0,80015080 <_vfiprintf_r+0x8f4>
80015058:	00000000 	nop
8001505c:	8fc20138 	lw	v0,312(s8)
80015060:	24430008 	addiu	v1,v0,8
80015064:	afc30138 	sw	v1,312(s8)
80015068:	24420004 	addiu	v0,v0,4
8001506c:	8c420000 	lw	v0,0(v0)
80015070:	0002103c 	dsll32	v0,v0,0x0
80015074:	0002103e 	dsrl32	v0,v0,0x0
80015078:	10000014 	b	800150cc <_vfiprintf_r+0x940>
8001507c:	00000000 	nop
80015080:	32220040 	andi	v0,s1,0x40
80015084:	1040000a 	beqz	v0,800150b0 <_vfiprintf_r+0x924>
80015088:	00000000 	nop
8001508c:	8fc20138 	lw	v0,312(s8)
80015090:	24430008 	addiu	v1,v0,8
80015094:	afc30138 	sw	v1,312(s8)
80015098:	24420004 	addiu	v0,v0,4
8001509c:	8c420000 	lw	v0,0(v0)
800150a0:	3042ffff 	andi	v0,v0,0xffff
800150a4:	3042ffff 	andi	v0,v0,0xffff
800150a8:	10000008 	b	800150cc <_vfiprintf_r+0x940>
800150ac:	00000000 	nop
800150b0:	8fc20138 	lw	v0,312(s8)
800150b4:	24430008 	addiu	v1,v0,8
800150b8:	afc30138 	sw	v1,312(s8)
800150bc:	24420004 	addiu	v0,v0,4
800150c0:	8c420000 	lw	v0,0(v0)
800150c4:	0002103c 	dsll32	v0,v0,0x0
800150c8:	0002103e 	dsrl32	v0,v0,0x0
800150cc:	ffc20030 	sd	v0,48(s8)
800150d0:	24020002 	li	v0,2
800150d4:	afc20038 	sw	v0,56(s8)
800150d8:	32220001 	andi	v0,s1,0x1
800150dc:	1040000c 	beqz	v0,80015110 <_vfiprintf_r+0x984>
800150e0:	00000000 	nop
800150e4:	dfc20030 	ld	v0,48(s8)
800150e8:	10400009 	beqz	v0,80015110 <_vfiprintf_r+0x984>
800150ec:	00000000 	nop
800150f0:	24020030 	li	v0,48
800150f4:	a3c200d8 	sb	v0,216(s8)
800150f8:	00151600 	sll	v0,s5,0x18
800150fc:	00021603 	sra	v0,v0,0x18
80015100:	a3c200d9 	sb	v0,217(s8)
80015104:	36310002 	ori	s1,s1,0x2
80015108:	10000002 	b	80015114 <_vfiprintf_r+0x988>
8001510c:	00000000 	nop
80015110:	00000000 	nop
80015114:	a3c0005c 	sb	zero,92(s8)
80015118:	8fc20028 	lw	v0,40(s8)
8001511c:	afc2003c 	sw	v0,60(s8)
80015120:	8fc2003c 	lw	v0,60(s8)
80015124:	04400003 	bltz	v0,80015134 <_vfiprintf_r+0x9a8>
80015128:	00000000 	nop
8001512c:	2402ff7f 	li	v0,-129
80015130:	02228824 	and	s1,s1,v0
80015134:	27d200b0 	addiu	s2,s8,176
80015138:	26520028 	addiu	s2,s2,40
8001513c:	dfc20030 	ld	v0,48(s8)
80015140:	14400004 	bnez	v0,80015154 <_vfiprintf_r+0x9c8>
80015144:	00000000 	nop
80015148:	8fc20028 	lw	v0,40(s8)
8001514c:	1040007f 	beqz	v0,8001534c <_vfiprintf_r+0xbc0>
80015150:	00000000 	nop
80015154:	8fc30038 	lw	v1,56(s8)
80015158:	24020002 	li	v0,2
8001515c:	1062005f 	beq	v1,v0,800152dc <_vfiprintf_r+0xb50>
80015160:	00000000 	nop
80015164:	8fc20038 	lw	v0,56(s8)
80015168:	2c420003 	sltiu	v0,v0,3
8001516c:	1040006b 	beqz	v0,8001531c <_vfiprintf_r+0xb90>
80015170:	00000000 	nop
80015174:	8fc20038 	lw	v0,56(s8)
80015178:	10400008 	beqz	v0,8001519c <_vfiprintf_r+0xa10>
8001517c:	00000000 	nop
80015180:	8fc30038 	lw	v1,56(s8)
80015184:	24020001 	li	v0,1
80015188:	1062001f 	beq	v1,v0,80015208 <_vfiprintf_r+0xa7c>
8001518c:	00000000 	nop
80015190:	10000062 	b	8001531c <_vfiprintf_r+0xb90>
80015194:	00000000 	nop
80015198:	00000000 	nop
8001519c:	93c20037 	lbu	v0,55(s8)
800151a0:	30420007 	andi	v0,v0,0x7
800151a4:	304200ff 	andi	v0,v0,0xff
800151a8:	24420030 	addiu	v0,v0,48
800151ac:	304200ff 	andi	v0,v0,0xff
800151b0:	2652ffff 	addiu	s2,s2,-1
800151b4:	00021600 	sll	v0,v0,0x18
800151b8:	00021603 	sra	v0,v0,0x18
800151bc:	a2420000 	sb	v0,0(s2)
800151c0:	dfc20030 	ld	v0,48(s8)
800151c4:	000210fa 	dsrl	v0,v0,0x3
800151c8:	ffc20030 	sd	v0,48(s8)
800151cc:	dfc20030 	ld	v0,48(s8)
800151d0:	1440fff1 	bnez	v0,80015198 <_vfiprintf_r+0xa0c>
800151d4:	00000000 	nop
800151d8:	32220001 	andi	v0,s1,0x1
800151dc:	10400058 	beqz	v0,80015340 <_vfiprintf_r+0xbb4>
800151e0:	00000000 	nop
800151e4:	82430000 	lb	v1,0(s2)
800151e8:	24020030 	li	v0,48
800151ec:	10620054 	beq	v1,v0,80015340 <_vfiprintf_r+0xbb4>
800151f0:	00000000 	nop
800151f4:	2652ffff 	addiu	s2,s2,-1
800151f8:	24020030 	li	v0,48
800151fc:	a2420000 	sb	v0,0(s2)
80015200:	1000004f 	b	80015340 <_vfiprintf_r+0xbb4>
80015204:	00000000 	nop
80015208:	dfc20030 	ld	v0,48(s8)
8001520c:	2c42000a 	sltiu	v0,v0,10
80015210:	1040000a 	beqz	v0,8001523c <_vfiprintf_r+0xab0>
80015214:	00000000 	nop
80015218:	93c20037 	lbu	v0,55(s8)
8001521c:	24420030 	addiu	v0,v0,48
80015220:	304200ff 	andi	v0,v0,0xff
80015224:	2652ffff 	addiu	s2,s2,-1
80015228:	00021600 	sll	v0,v0,0x18
8001522c:	00021603 	sra	v0,v0,0x18
80015230:	a2420000 	sb	v0,0(s2)
80015234:	10000043 	b	80015344 <_vfiprintf_r+0xbb8>
80015238:	00000000 	nop
8001523c:	dfc40030 	ld	a0,48(s8)
80015240:	3c02f333 	lui	v0,0xf333
80015244:	34423333 	ori	v0,v0,0x3333
80015248:	00021438 	dsll	v0,v0,0x10
8001524c:	34423333 	ori	v0,v0,0x3333
80015250:	000214b8 	dsll	v0,v0,0x12
80015254:	3442cccd 	ori	v0,v0,0xcccd
80015258:	0082001d 	dmultu	a0,v0
8001525c:	00001010 	mfhi	v0
80015260:	000218fa 	dsrl	v1,v0,0x3
80015264:	00601025 	move	v0,v1
80015268:	000210b8 	dsll	v0,v0,0x2
8001526c:	0043102d 	daddu	v0,v0,v1
80015270:	00021078 	dsll	v0,v0,0x1
80015274:	0082182f 	dsubu	v1,a0,v0
80015278:	00031000 	sll	v0,v1,0x0
8001527c:	304200ff 	andi	v0,v0,0xff
80015280:	24420030 	addiu	v0,v0,48
80015284:	304200ff 	andi	v0,v0,0xff
80015288:	2652ffff 	addiu	s2,s2,-1
8001528c:	00021600 	sll	v0,v0,0x18
80015290:	00021603 	sra	v0,v0,0x18
80015294:	a2420000 	sb	v0,0(s2)
80015298:	dfc30030 	ld	v1,48(s8)
8001529c:	3c02f333 	lui	v0,0xf333
800152a0:	34423333 	ori	v0,v0,0x3333
800152a4:	00021438 	dsll	v0,v0,0x10
800152a8:	34423333 	ori	v0,v0,0x3333
800152ac:	000214b8 	dsll	v0,v0,0x12
800152b0:	3442cccd 	ori	v0,v0,0xcccd
800152b4:	0062001d 	dmultu	v1,v0
800152b8:	00001010 	mfhi	v0
800152bc:	000210fa 	dsrl	v0,v0,0x3
800152c0:	ffc20030 	sd	v0,48(s8)
800152c4:	dfc20030 	ld	v0,48(s8)
800152c8:	1440ffdc 	bnez	v0,8001523c <_vfiprintf_r+0xab0>
800152cc:	00000000 	nop
800152d0:	1000001c 	b	80015344 <_vfiprintf_r+0xbb8>
800152d4:	00000000 	nop
800152d8:	00000000 	nop
800152dc:	dfc20030 	ld	v0,48(s8)
800152e0:	00021000 	sll	v0,v0,0x0
800152e4:	3042000f 	andi	v0,v0,0xf
800152e8:	8fc30048 	lw	v1,72(s8)
800152ec:	00621021 	addu	v0,v1,v0
800152f0:	2652ffff 	addiu	s2,s2,-1
800152f4:	80420000 	lb	v0,0(v0)
800152f8:	a2420000 	sb	v0,0(s2)
800152fc:	dfc20030 	ld	v0,48(s8)
80015300:	0002113a 	dsrl	v0,v0,0x4
80015304:	ffc20030 	sd	v0,48(s8)
80015308:	dfc20030 	ld	v0,48(s8)
8001530c:	1440fff2 	bnez	v0,800152d8 <_vfiprintf_r+0xb4c>
80015310:	00000000 	nop
80015314:	1000000b 	b	80015344 <_vfiprintf_r+0xbb8>
80015318:	00000000 	nop
8001531c:	3c028002 	lui	v0,0x8002
80015320:	2452ac40 	addiu	s2,v0,-21440
80015324:	02402025 	move	a0,s2
80015328:	0c001f1d 	jal	80007c74 <strlen>
8001532c:	00000000 	nop
80015330:	afc20044 	sw	v0,68(s8)
80015334:	00000000 	nop
80015338:	1000001d 	b	800153b0 <_vfiprintf_r+0xc24>
8001533c:	00000000 	nop
80015340:	00000000 	nop
80015344:	1000000a 	b	80015370 <_vfiprintf_r+0xbe4>
80015348:	00000000 	nop
8001534c:	8fc20038 	lw	v0,56(s8)
80015350:	14400007 	bnez	v0,80015370 <_vfiprintf_r+0xbe4>
80015354:	00000000 	nop
80015358:	32220001 	andi	v0,s1,0x1
8001535c:	10400004 	beqz	v0,80015370 <_vfiprintf_r+0xbe4>
80015360:	00000000 	nop
80015364:	2652ffff 	addiu	s2,s2,-1
80015368:	24020030 	li	v0,48
8001536c:	a2420000 	sb	v0,0(s2)
80015370:	27c200b0 	addiu	v0,s8,176
80015374:	24420028 	addiu	v0,v0,40
80015378:	00521023 	subu	v0,v0,s2
8001537c:	afc20044 	sw	v0,68(s8)
80015380:	1000000b 	b	800153b0 <_vfiprintf_r+0xc24>
80015384:	00000000 	nop
80015388:	12a0018d 	beqz	s5,800159c0 <_vfiprintf_r+0x1234>
8001538c:	00000000 	nop
80015390:	27d200b0 	addiu	s2,s8,176
80015394:	00151600 	sll	v0,s5,0x18
80015398:	00021603 	sra	v0,v0,0x18
8001539c:	a2420000 	sb	v0,0(s2)
800153a0:	24020001 	li	v0,1
800153a4:	afc20044 	sw	v0,68(s8)
800153a8:	a3c0005c 	sb	zero,92(s8)
800153ac:	00000000 	nop
800153b0:	8fc3003c 	lw	v1,60(s8)
800153b4:	8fc20044 	lw	v0,68(s8)
800153b8:	0043202a 	slt	a0,v0,v1
800153bc:	10800002 	beqz	a0,800153c8 <_vfiprintf_r+0xc3c>
800153c0:	00000000 	nop
800153c4:	00601025 	move	v0,v1
800153c8:	afc20040 	sw	v0,64(s8)
800153cc:	83c2005c 	lb	v0,92(s8)
800153d0:	10400004 	beqz	v0,800153e4 <_vfiprintf_r+0xc58>
800153d4:	00000000 	nop
800153d8:	8fc20040 	lw	v0,64(s8)
800153dc:	24420001 	addiu	v0,v0,1
800153e0:	afc20040 	sw	v0,64(s8)
800153e4:	32220002 	andi	v0,s1,0x2
800153e8:	10400004 	beqz	v0,800153fc <_vfiprintf_r+0xc70>
800153ec:	00000000 	nop
800153f0:	8fc20040 	lw	v0,64(s8)
800153f4:	24420002 	addiu	v0,v0,2
800153f8:	afc20040 	sw	v0,64(s8)
800153fc:	32220084 	andi	v0,s1,0x84
80015400:	1440003f 	bnez	v0,80015500 <_vfiprintf_r+0xd74>
80015404:	00000000 	nop
80015408:	8fc30024 	lw	v1,36(s8)
8001540c:	8fc20040 	lw	v0,64(s8)
80015410:	00629823 	subu	s3,v1,v0
80015414:	1a60003a 	blez	s3,80015500 <_vfiprintf_r+0xd74>
80015418:	00000000 	nop
8001541c:	1000001b 	b	8001548c <_vfiprintf_r+0xd00>
80015420:	00000000 	nop
80015424:	3c028002 	lui	v0,0x8002
80015428:	2442adc0 	addiu	v0,v0,-21056
8001542c:	ae020000 	sw	v0,0(s0)
80015430:	24020010 	li	v0,16
80015434:	ae020004 	sw	v0,4(s0)
80015438:	8fc20068 	lw	v0,104(s8)
8001543c:	24420010 	addiu	v0,v0,16
80015440:	afc20068 	sw	v0,104(s8)
80015444:	26100008 	addiu	s0,s0,8
80015448:	8fc20064 	lw	v0,100(s8)
8001544c:	24420001 	addiu	v0,v0,1
80015450:	afc20064 	sw	v0,100(s8)
80015454:	8fc20064 	lw	v0,100(s8)
80015458:	28420008 	slti	v0,v0,8
8001545c:	1440000a 	bnez	v0,80015488 <_vfiprintf_r+0xcfc>
80015460:	00000000 	nop
80015464:	27c20060 	addiu	v0,s8,96
80015468:	00403025 	move	a2,v0
8001546c:	8fc50128 	lw	a1,296(s8)
80015470:	8fc40120 	lw	a0,288(s8)
80015474:	0c00512c 	jal	800144b0 <__sprint_r>
80015478:	00000000 	nop
8001547c:	14400162 	bnez	v0,80015a08 <_vfiprintf_r+0x127c>
80015480:	00000000 	nop
80015484:	27d00070 	addiu	s0,s8,112
80015488:	2673fff0 	addiu	s3,s3,-16
8001548c:	2a620011 	slti	v0,s3,17
80015490:	1040ffe4 	beqz	v0,80015424 <_vfiprintf_r+0xc98>
80015494:	00000000 	nop
80015498:	3c028002 	lui	v0,0x8002
8001549c:	2442adc0 	addiu	v0,v0,-21056
800154a0:	ae020000 	sw	v0,0(s0)
800154a4:	02601025 	move	v0,s3
800154a8:	ae020004 	sw	v0,4(s0)
800154ac:	8fc20068 	lw	v0,104(s8)
800154b0:	02601825 	move	v1,s3
800154b4:	00431021 	addu	v0,v0,v1
800154b8:	afc20068 	sw	v0,104(s8)
800154bc:	26100008 	addiu	s0,s0,8
800154c0:	8fc20064 	lw	v0,100(s8)
800154c4:	24420001 	addiu	v0,v0,1
800154c8:	afc20064 	sw	v0,100(s8)
800154cc:	8fc20064 	lw	v0,100(s8)
800154d0:	28420008 	slti	v0,v0,8
800154d4:	1440000a 	bnez	v0,80015500 <_vfiprintf_r+0xd74>
800154d8:	00000000 	nop
800154dc:	27c20060 	addiu	v0,s8,96
800154e0:	00403025 	move	a2,v0
800154e4:	8fc50128 	lw	a1,296(s8)
800154e8:	8fc40120 	lw	a0,288(s8)
800154ec:	0c00512c 	jal	800144b0 <__sprint_r>
800154f0:	00000000 	nop
800154f4:	14400147 	bnez	v0,80015a14 <_vfiprintf_r+0x1288>
800154f8:	00000000 	nop
800154fc:	27d00070 	addiu	s0,s8,112
80015500:	83c2005c 	lb	v0,92(s8)
80015504:	10400019 	beqz	v0,8001556c <_vfiprintf_r+0xde0>
80015508:	00000000 	nop
8001550c:	27c2005c 	addiu	v0,s8,92
80015510:	ae020000 	sw	v0,0(s0)
80015514:	24020001 	li	v0,1
80015518:	ae020004 	sw	v0,4(s0)
8001551c:	8fc20068 	lw	v0,104(s8)
80015520:	24420001 	addiu	v0,v0,1
80015524:	afc20068 	sw	v0,104(s8)
80015528:	26100008 	addiu	s0,s0,8
8001552c:	8fc20064 	lw	v0,100(s8)
80015530:	24420001 	addiu	v0,v0,1
80015534:	afc20064 	sw	v0,100(s8)
80015538:	8fc20064 	lw	v0,100(s8)
8001553c:	28420008 	slti	v0,v0,8
80015540:	1440000a 	bnez	v0,8001556c <_vfiprintf_r+0xde0>
80015544:	00000000 	nop
80015548:	27c20060 	addiu	v0,s8,96
8001554c:	00403025 	move	a2,v0
80015550:	8fc50128 	lw	a1,296(s8)
80015554:	8fc40120 	lw	a0,288(s8)
80015558:	0c00512c 	jal	800144b0 <__sprint_r>
8001555c:	00000000 	nop
80015560:	1440012f 	bnez	v0,80015a20 <_vfiprintf_r+0x1294>
80015564:	00000000 	nop
80015568:	27d00070 	addiu	s0,s8,112
8001556c:	32220002 	andi	v0,s1,0x2
80015570:	10400019 	beqz	v0,800155d8 <_vfiprintf_r+0xe4c>
80015574:	00000000 	nop
80015578:	27c200d8 	addiu	v0,s8,216
8001557c:	ae020000 	sw	v0,0(s0)
80015580:	24020002 	li	v0,2
80015584:	ae020004 	sw	v0,4(s0)
80015588:	8fc20068 	lw	v0,104(s8)
8001558c:	24420002 	addiu	v0,v0,2
80015590:	afc20068 	sw	v0,104(s8)
80015594:	26100008 	addiu	s0,s0,8
80015598:	8fc20064 	lw	v0,100(s8)
8001559c:	24420001 	addiu	v0,v0,1
800155a0:	afc20064 	sw	v0,100(s8)
800155a4:	8fc20064 	lw	v0,100(s8)
800155a8:	28420008 	slti	v0,v0,8
800155ac:	1440000a 	bnez	v0,800155d8 <_vfiprintf_r+0xe4c>
800155b0:	00000000 	nop
800155b4:	27c20060 	addiu	v0,s8,96
800155b8:	00403025 	move	a2,v0
800155bc:	8fc50128 	lw	a1,296(s8)
800155c0:	8fc40120 	lw	a0,288(s8)
800155c4:	0c00512c 	jal	800144b0 <__sprint_r>
800155c8:	00000000 	nop
800155cc:	14400117 	bnez	v0,80015a2c <_vfiprintf_r+0x12a0>
800155d0:	00000000 	nop
800155d4:	27d00070 	addiu	s0,s8,112
800155d8:	32230084 	andi	v1,s1,0x84
800155dc:	24020080 	li	v0,128
800155e0:	1462003f 	bne	v1,v0,800156e0 <_vfiprintf_r+0xf54>
800155e4:	00000000 	nop
800155e8:	8fc30024 	lw	v1,36(s8)
800155ec:	8fc20040 	lw	v0,64(s8)
800155f0:	00629823 	subu	s3,v1,v0
800155f4:	1a60003a 	blez	s3,800156e0 <_vfiprintf_r+0xf54>
800155f8:	00000000 	nop
800155fc:	1000001b 	b	8001566c <_vfiprintf_r+0xee0>
80015600:	00000000 	nop
80015604:	3c028002 	lui	v0,0x8002
80015608:	2442add0 	addiu	v0,v0,-21040
8001560c:	ae020000 	sw	v0,0(s0)
80015610:	24020010 	li	v0,16
80015614:	ae020004 	sw	v0,4(s0)
80015618:	8fc20068 	lw	v0,104(s8)
8001561c:	24420010 	addiu	v0,v0,16
80015620:	afc20068 	sw	v0,104(s8)
80015624:	26100008 	addiu	s0,s0,8
80015628:	8fc20064 	lw	v0,100(s8)
8001562c:	24420001 	addiu	v0,v0,1
80015630:	afc20064 	sw	v0,100(s8)
80015634:	8fc20064 	lw	v0,100(s8)
80015638:	28420008 	slti	v0,v0,8
8001563c:	1440000a 	bnez	v0,80015668 <_vfiprintf_r+0xedc>
80015640:	00000000 	nop
80015644:	27c20060 	addiu	v0,s8,96
80015648:	00403025 	move	a2,v0
8001564c:	8fc50128 	lw	a1,296(s8)
80015650:	8fc40120 	lw	a0,288(s8)
80015654:	0c00512c 	jal	800144b0 <__sprint_r>
80015658:	00000000 	nop
8001565c:	144000f6 	bnez	v0,80015a38 <_vfiprintf_r+0x12ac>
80015660:	00000000 	nop
80015664:	27d00070 	addiu	s0,s8,112
80015668:	2673fff0 	addiu	s3,s3,-16
8001566c:	2a620011 	slti	v0,s3,17
80015670:	1040ffe4 	beqz	v0,80015604 <_vfiprintf_r+0xe78>
80015674:	00000000 	nop
80015678:	3c028002 	lui	v0,0x8002
8001567c:	2442add0 	addiu	v0,v0,-21040
80015680:	ae020000 	sw	v0,0(s0)
80015684:	02601025 	move	v0,s3
80015688:	ae020004 	sw	v0,4(s0)
8001568c:	8fc20068 	lw	v0,104(s8)
80015690:	02601825 	move	v1,s3
80015694:	00431021 	addu	v0,v0,v1
80015698:	afc20068 	sw	v0,104(s8)
8001569c:	26100008 	addiu	s0,s0,8
800156a0:	8fc20064 	lw	v0,100(s8)
800156a4:	24420001 	addiu	v0,v0,1
800156a8:	afc20064 	sw	v0,100(s8)
800156ac:	8fc20064 	lw	v0,100(s8)
800156b0:	28420008 	slti	v0,v0,8
800156b4:	1440000a 	bnez	v0,800156e0 <_vfiprintf_r+0xf54>
800156b8:	00000000 	nop
800156bc:	27c20060 	addiu	v0,s8,96
800156c0:	00403025 	move	a2,v0
800156c4:	8fc50128 	lw	a1,296(s8)
800156c8:	8fc40120 	lw	a0,288(s8)
800156cc:	0c00512c 	jal	800144b0 <__sprint_r>
800156d0:	00000000 	nop
800156d4:	144000db 	bnez	v0,80015a44 <_vfiprintf_r+0x12b8>
800156d8:	00000000 	nop
800156dc:	27d00070 	addiu	s0,s8,112
800156e0:	8fc3003c 	lw	v1,60(s8)
800156e4:	8fc20044 	lw	v0,68(s8)
800156e8:	00629823 	subu	s3,v1,v0
800156ec:	1a60003a 	blez	s3,800157d8 <_vfiprintf_r+0x104c>
800156f0:	00000000 	nop
800156f4:	1000001b 	b	80015764 <_vfiprintf_r+0xfd8>
800156f8:	00000000 	nop
800156fc:	3c028002 	lui	v0,0x8002
80015700:	2442add0 	addiu	v0,v0,-21040
80015704:	ae020000 	sw	v0,0(s0)
80015708:	24020010 	li	v0,16
8001570c:	ae020004 	sw	v0,4(s0)
80015710:	8fc20068 	lw	v0,104(s8)
80015714:	24420010 	addiu	v0,v0,16
80015718:	afc20068 	sw	v0,104(s8)
8001571c:	26100008 	addiu	s0,s0,8
80015720:	8fc20064 	lw	v0,100(s8)
80015724:	24420001 	addiu	v0,v0,1
80015728:	afc20064 	sw	v0,100(s8)
8001572c:	8fc20064 	lw	v0,100(s8)
80015730:	28420008 	slti	v0,v0,8
80015734:	1440000a 	bnez	v0,80015760 <_vfiprintf_r+0xfd4>
80015738:	00000000 	nop
8001573c:	27c20060 	addiu	v0,s8,96
80015740:	00403025 	move	a2,v0
80015744:	8fc50128 	lw	a1,296(s8)
80015748:	8fc40120 	lw	a0,288(s8)
8001574c:	0c00512c 	jal	800144b0 <__sprint_r>
80015750:	00000000 	nop
80015754:	144000be 	bnez	v0,80015a50 <_vfiprintf_r+0x12c4>
80015758:	00000000 	nop
8001575c:	27d00070 	addiu	s0,s8,112
80015760:	2673fff0 	addiu	s3,s3,-16
80015764:	2a620011 	slti	v0,s3,17
80015768:	1040ffe4 	beqz	v0,800156fc <_vfiprintf_r+0xf70>
8001576c:	00000000 	nop
80015770:	3c028002 	lui	v0,0x8002
80015774:	2442add0 	addiu	v0,v0,-21040
80015778:	ae020000 	sw	v0,0(s0)
8001577c:	02601025 	move	v0,s3
80015780:	ae020004 	sw	v0,4(s0)
80015784:	8fc20068 	lw	v0,104(s8)
80015788:	02601825 	move	v1,s3
8001578c:	00431021 	addu	v0,v0,v1
80015790:	afc20068 	sw	v0,104(s8)
80015794:	26100008 	addiu	s0,s0,8
80015798:	8fc20064 	lw	v0,100(s8)
8001579c:	24420001 	addiu	v0,v0,1
800157a0:	afc20064 	sw	v0,100(s8)
800157a4:	8fc20064 	lw	v0,100(s8)
800157a8:	28420008 	slti	v0,v0,8
800157ac:	1440000a 	bnez	v0,800157d8 <_vfiprintf_r+0x104c>
800157b0:	00000000 	nop
800157b4:	27c20060 	addiu	v0,s8,96
800157b8:	00403025 	move	a2,v0
800157bc:	8fc50128 	lw	a1,296(s8)
800157c0:	8fc40120 	lw	a0,288(s8)
800157c4:	0c00512c 	jal	800144b0 <__sprint_r>
800157c8:	00000000 	nop
800157cc:	144000a3 	bnez	v0,80015a5c <_vfiprintf_r+0x12d0>
800157d0:	00000000 	nop
800157d4:	27d00070 	addiu	s0,s8,112
800157d8:	ae120000 	sw	s2,0(s0)
800157dc:	8fc20044 	lw	v0,68(s8)
800157e0:	ae020004 	sw	v0,4(s0)
800157e4:	8fc30068 	lw	v1,104(s8)
800157e8:	8fc20044 	lw	v0,68(s8)
800157ec:	00621021 	addu	v0,v1,v0
800157f0:	afc20068 	sw	v0,104(s8)
800157f4:	26100008 	addiu	s0,s0,8
800157f8:	8fc20064 	lw	v0,100(s8)
800157fc:	24420001 	addiu	v0,v0,1
80015800:	afc20064 	sw	v0,100(s8)
80015804:	8fc20064 	lw	v0,100(s8)
80015808:	28420008 	slti	v0,v0,8
8001580c:	1440000a 	bnez	v0,80015838 <_vfiprintf_r+0x10ac>
80015810:	00000000 	nop
80015814:	27c20060 	addiu	v0,s8,96
80015818:	00403025 	move	a2,v0
8001581c:	8fc50128 	lw	a1,296(s8)
80015820:	8fc40120 	lw	a0,288(s8)
80015824:	0c00512c 	jal	800144b0 <__sprint_r>
80015828:	00000000 	nop
8001582c:	1440008e 	bnez	v0,80015a68 <_vfiprintf_r+0x12dc>
80015830:	00000000 	nop
80015834:	27d00070 	addiu	s0,s8,112
80015838:	32220004 	andi	v0,s1,0x4
8001583c:	1040003d 	beqz	v0,80015934 <_vfiprintf_r+0x11a8>
80015840:	00000000 	nop
80015844:	8fc30024 	lw	v1,36(s8)
80015848:	8fc20040 	lw	v0,64(s8)
8001584c:	00629823 	subu	s3,v1,v0
80015850:	1a600038 	blez	s3,80015934 <_vfiprintf_r+0x11a8>
80015854:	00000000 	nop
80015858:	1000001b 	b	800158c8 <_vfiprintf_r+0x113c>
8001585c:	00000000 	nop
80015860:	3c028002 	lui	v0,0x8002
80015864:	2442adc0 	addiu	v0,v0,-21056
80015868:	ae020000 	sw	v0,0(s0)
8001586c:	24020010 	li	v0,16
80015870:	ae020004 	sw	v0,4(s0)
80015874:	8fc20068 	lw	v0,104(s8)
80015878:	24420010 	addiu	v0,v0,16
8001587c:	afc20068 	sw	v0,104(s8)
80015880:	26100008 	addiu	s0,s0,8
80015884:	8fc20064 	lw	v0,100(s8)
80015888:	24420001 	addiu	v0,v0,1
8001588c:	afc20064 	sw	v0,100(s8)
80015890:	8fc20064 	lw	v0,100(s8)
80015894:	28420008 	slti	v0,v0,8
80015898:	1440000a 	bnez	v0,800158c4 <_vfiprintf_r+0x1138>
8001589c:	00000000 	nop
800158a0:	27c20060 	addiu	v0,s8,96
800158a4:	00403025 	move	a2,v0
800158a8:	8fc50128 	lw	a1,296(s8)
800158ac:	8fc40120 	lw	a0,288(s8)
800158b0:	0c00512c 	jal	800144b0 <__sprint_r>
800158b4:	00000000 	nop
800158b8:	1440006e 	bnez	v0,80015a74 <_vfiprintf_r+0x12e8>
800158bc:	00000000 	nop
800158c0:	27d00070 	addiu	s0,s8,112
800158c4:	2673fff0 	addiu	s3,s3,-16
800158c8:	2a620011 	slti	v0,s3,17
800158cc:	1040ffe4 	beqz	v0,80015860 <_vfiprintf_r+0x10d4>
800158d0:	00000000 	nop
800158d4:	3c028002 	lui	v0,0x8002
800158d8:	2442adc0 	addiu	v0,v0,-21056
800158dc:	ae020000 	sw	v0,0(s0)
800158e0:	02601025 	move	v0,s3
800158e4:	ae020004 	sw	v0,4(s0)
800158e8:	8fc20068 	lw	v0,104(s8)
800158ec:	02601825 	move	v1,s3
800158f0:	00431021 	addu	v0,v0,v1
800158f4:	afc20068 	sw	v0,104(s8)
800158f8:	8fc20064 	lw	v0,100(s8)
800158fc:	24420001 	addiu	v0,v0,1
80015900:	afc20064 	sw	v0,100(s8)
80015904:	8fc20064 	lw	v0,100(s8)
80015908:	28420008 	slti	v0,v0,8
8001590c:	14400009 	bnez	v0,80015934 <_vfiprintf_r+0x11a8>
80015910:	00000000 	nop
80015914:	27c20060 	addiu	v0,s8,96
80015918:	00403025 	move	a2,v0
8001591c:	8fc50128 	lw	a1,296(s8)
80015920:	8fc40120 	lw	a0,288(s8)
80015924:	0c00512c 	jal	800144b0 <__sprint_r>
80015928:	00000000 	nop
8001592c:	14400054 	bnez	v0,80015a80 <_vfiprintf_r+0x12f4>
80015930:	00000000 	nop
80015934:	8fc30024 	lw	v1,36(s8)
80015938:	8fc20040 	lw	v0,64(s8)
8001593c:	0043202a 	slt	a0,v0,v1
80015940:	10800002 	beqz	a0,8001594c <_vfiprintf_r+0x11c0>
80015944:	00000000 	nop
80015948:	00601025 	move	v0,v1
8001594c:	8fc30020 	lw	v1,32(s8)
80015950:	00621021 	addu	v0,v1,v0
80015954:	afc20020 	sw	v0,32(s8)
80015958:	8fc20068 	lw	v0,104(s8)
8001595c:	10400009 	beqz	v0,80015984 <_vfiprintf_r+0x11f8>
80015960:	00000000 	nop
80015964:	27c20060 	addiu	v0,s8,96
80015968:	00403025 	move	a2,v0
8001596c:	8fc50128 	lw	a1,296(s8)
80015970:	8fc40120 	lw	a0,288(s8)
80015974:	0c00512c 	jal	800144b0 <__sprint_r>
80015978:	00000000 	nop
8001597c:	14400043 	bnez	v0,80015a8c <_vfiprintf_r+0x1300>
80015980:	00000000 	nop
80015984:	afc00064 	sw	zero,100(s8)
80015988:	27d00070 	addiu	s0,s8,112
8001598c:	8fc2004c 	lw	v0,76(s8)
80015990:	1040fbda 	beqz	v0,800148fc <_vfiprintf_r+0x170>
80015994:	00000000 	nop
80015998:	8fc5004c 	lw	a1,76(s8)
8001599c:	8fc40120 	lw	a0,288(s8)
800159a0:	0c000dcb 	jal	8000372c <_free_r>
800159a4:	00000000 	nop
800159a8:	afc0004c 	sw	zero,76(s8)
800159ac:	1000fbd3 	b	800148fc <_vfiprintf_r+0x170>
800159b0:	00000000 	nop
800159b4:	00000000 	nop
800159b8:	10000002 	b	800159c4 <_vfiprintf_r+0x1238>
800159bc:	00000000 	nop
800159c0:	00000000 	nop
800159c4:	8fc20068 	lw	v0,104(s8)
800159c8:	10400009 	beqz	v0,800159f0 <_vfiprintf_r+0x1264>
800159cc:	00000000 	nop
800159d0:	27c20060 	addiu	v0,s8,96
800159d4:	00403025 	move	a2,v0
800159d8:	8fc50128 	lw	a1,296(s8)
800159dc:	8fc40120 	lw	a0,288(s8)
800159e0:	0c00512c 	jal	800144b0 <__sprint_r>
800159e4:	00000000 	nop
800159e8:	1440002b 	bnez	v0,80015a98 <_vfiprintf_r+0x130c>
800159ec:	00000000 	nop
800159f0:	afc00064 	sw	zero,100(s8)
800159f4:	10000029 	b	80015a9c <_vfiprintf_r+0x1310>
800159f8:	00000000 	nop
800159fc:	00000000 	nop
80015a00:	10000026 	b	80015a9c <_vfiprintf_r+0x1310>
80015a04:	00000000 	nop
80015a08:	00000000 	nop
80015a0c:	10000023 	b	80015a9c <_vfiprintf_r+0x1310>
80015a10:	00000000 	nop
80015a14:	00000000 	nop
80015a18:	10000020 	b	80015a9c <_vfiprintf_r+0x1310>
80015a1c:	00000000 	nop
80015a20:	00000000 	nop
80015a24:	1000001d 	b	80015a9c <_vfiprintf_r+0x1310>
80015a28:	00000000 	nop
80015a2c:	00000000 	nop
80015a30:	1000001a 	b	80015a9c <_vfiprintf_r+0x1310>
80015a34:	00000000 	nop
80015a38:	00000000 	nop
80015a3c:	10000017 	b	80015a9c <_vfiprintf_r+0x1310>
80015a40:	00000000 	nop
80015a44:	00000000 	nop
80015a48:	10000014 	b	80015a9c <_vfiprintf_r+0x1310>
80015a4c:	00000000 	nop
80015a50:	00000000 	nop
80015a54:	10000011 	b	80015a9c <_vfiprintf_r+0x1310>
80015a58:	00000000 	nop
80015a5c:	00000000 	nop
80015a60:	1000000e 	b	80015a9c <_vfiprintf_r+0x1310>
80015a64:	00000000 	nop
80015a68:	00000000 	nop
80015a6c:	1000000b 	b	80015a9c <_vfiprintf_r+0x1310>
80015a70:	00000000 	nop
80015a74:	00000000 	nop
80015a78:	10000008 	b	80015a9c <_vfiprintf_r+0x1310>
80015a7c:	00000000 	nop
80015a80:	00000000 	nop
80015a84:	10000005 	b	80015a9c <_vfiprintf_r+0x1310>
80015a88:	00000000 	nop
80015a8c:	00000000 	nop
80015a90:	10000002 	b	80015a9c <_vfiprintf_r+0x1310>
80015a94:	00000000 	nop
80015a98:	00000000 	nop
80015a9c:	8fc2004c 	lw	v0,76(s8)
80015aa0:	10400005 	beqz	v0,80015ab8 <_vfiprintf_r+0x132c>
80015aa4:	00000000 	nop
80015aa8:	8fc5004c 	lw	a1,76(s8)
80015aac:	8fc40120 	lw	a0,288(s8)
80015ab0:	0c000dcb 	jal	8000372c <_free_r>
80015ab4:	00000000 	nop
80015ab8:	8fc20128 	lw	v0,296(s8)
80015abc:	8442000c 	lh	v0,12(v0)
80015ac0:	3042ffff 	andi	v0,v0,0xffff
80015ac4:	30420040 	andi	v0,v0,0x40
80015ac8:	14400004 	bnez	v0,80015adc <_vfiprintf_r+0x1350>
80015acc:	00000000 	nop
80015ad0:	8fc20020 	lw	v0,32(s8)
80015ad4:	10000002 	b	80015ae0 <_vfiprintf_r+0x1354>
80015ad8:	00000000 	nop
80015adc:	2402ffff 	li	v0,-1
80015ae0:	03c0e825 	move	sp,s8
80015ae4:	dfbf0118 	ld	ra,280(sp)
80015ae8:	dfbe0110 	ld	s8,272(sp)
80015aec:	dfb50108 	ld	s5,264(sp)
80015af0:	dfb40100 	ld	s4,256(sp)
80015af4:	dfb300f8 	ld	s3,248(sp)
80015af8:	dfb200f0 	ld	s2,240(sp)
80015afc:	dfb100e8 	ld	s1,232(sp)
80015b00:	dfb000e0 	ld	s0,224(sp)
80015b04:	27bd0120 	addiu	sp,sp,288
80015b08:	03e00008 	jr	ra
80015b0c:	00000000 	nop

80015b10 <__get_global_locale>:
80015b10:	27bdfff8 	addiu	sp,sp,-8
80015b14:	ffbe0000 	sd	s8,0(sp)
80015b18:	03a0f025 	move	s8,sp
80015b1c:	3c028002 	lui	v0,0x8002
80015b20:	2442d2f8 	addiu	v0,v0,-11528
80015b24:	03c0e825 	move	sp,s8
80015b28:	dfbe0000 	ld	s8,0(sp)
80015b2c:	27bd0008 	addiu	sp,sp,8
80015b30:	03e00008 	jr	ra
80015b34:	00000000 	nop

80015b38 <__get_current_locale>:
80015b38:	27bdffd0 	addiu	sp,sp,-48
80015b3c:	ffbf0028 	sd	ra,40(sp)
80015b40:	ffbe0020 	sd	s8,32(sp)
80015b44:	03a0f025 	move	s8,sp
80015b48:	0c0056c4 	jal	80015b10 <__get_global_locale>
80015b4c:	00000000 	nop
80015b50:	03c0e825 	move	sp,s8
80015b54:	dfbf0028 	ld	ra,40(sp)
80015b58:	dfbe0020 	ld	s8,32(sp)
80015b5c:	27bd0030 	addiu	sp,sp,48
80015b60:	03e00008 	jr	ra
80015b64:	00000000 	nop

80015b68 <_wctomb_r>:
80015b68:	27bdffd0 	addiu	sp,sp,-48
80015b6c:	ffbf0028 	sd	ra,40(sp)
80015b70:	ffbe0020 	sd	s8,32(sp)
80015b74:	03a0f025 	move	s8,sp
80015b78:	afc40030 	sw	a0,48(s8)
80015b7c:	afc50038 	sw	a1,56(s8)
80015b80:	afc60040 	sw	a2,64(s8)
80015b84:	afc70048 	sw	a3,72(s8)
80015b88:	0c0056ce 	jal	80015b38 <__get_current_locale>
80015b8c:	00000000 	nop
80015b90:	8c4200e0 	lw	v0,224(v0)
80015b94:	8fc70048 	lw	a3,72(s8)
80015b98:	8fc60040 	lw	a2,64(s8)
80015b9c:	8fc50038 	lw	a1,56(s8)
80015ba0:	8fc40030 	lw	a0,48(s8)
80015ba4:	0040f809 	jalr	v0
80015ba8:	00000000 	nop
80015bac:	03c0e825 	move	sp,s8
80015bb0:	dfbf0028 	ld	ra,40(sp)
80015bb4:	dfbe0020 	ld	s8,32(sp)
80015bb8:	27bd0030 	addiu	sp,sp,48
80015bbc:	03e00008 	jr	ra
80015bc0:	00000000 	nop

80015bc4 <__ascii_wctomb>:
80015bc4:	27bdfff0 	addiu	sp,sp,-16
80015bc8:	ffbe0008 	sd	s8,8(sp)
80015bcc:	03a0f025 	move	s8,sp
80015bd0:	afc40010 	sw	a0,16(s8)
80015bd4:	afc50018 	sw	a1,24(s8)
80015bd8:	afc60020 	sw	a2,32(s8)
80015bdc:	afc70028 	sw	a3,40(s8)
80015be0:	8fc20020 	lw	v0,32(s8)
80015be4:	afc20000 	sw	v0,0(s8)
80015be8:	8fc20018 	lw	v0,24(s8)
80015bec:	14400004 	bnez	v0,80015c00 <__ascii_wctomb+0x3c>
80015bf0:	00000000 	nop
80015bf4:	00001025 	move	v0,zero
80015bf8:	10000011 	b	80015c40 <__ascii_wctomb+0x7c>
80015bfc:	00000000 	nop
80015c00:	8fc20000 	lw	v0,0(s8)
80015c04:	2c420100 	sltiu	v0,v0,256
80015c08:	14400007 	bnez	v0,80015c28 <__ascii_wctomb+0x64>
80015c0c:	00000000 	nop
80015c10:	8fc20010 	lw	v0,16(s8)
80015c14:	2403008a 	li	v1,138
80015c18:	ac430000 	sw	v1,0(v0)
80015c1c:	2402ffff 	li	v0,-1
80015c20:	10000007 	b	80015c40 <__ascii_wctomb+0x7c>
80015c24:	00000000 	nop
80015c28:	8fc20000 	lw	v0,0(s8)
80015c2c:	00021e00 	sll	v1,v0,0x18
80015c30:	00031e03 	sra	v1,v1,0x18
80015c34:	8fc20018 	lw	v0,24(s8)
80015c38:	a0430000 	sb	v1,0(v0)
80015c3c:	24020001 	li	v0,1
80015c40:	03c0e825 	move	sp,s8
80015c44:	dfbe0008 	ld	s8,8(sp)
80015c48:	27bd0010 	addiu	sp,sp,16
80015c4c:	03e00008 	jr	ra
80015c50:	00000000 	nop

80015c54 <_calloc_r>:
80015c54:	27bdffb8 	addiu	sp,sp,-72
80015c58:	ffbf0040 	sd	ra,64(sp)
80015c5c:	ffbe0038 	sd	s8,56(sp)
80015c60:	03a0f025 	move	s8,sp
80015c64:	afc40048 	sw	a0,72(s8)
80015c68:	afc50050 	sw	a1,80(s8)
80015c6c:	afc60058 	sw	a2,88(s8)
80015c70:	00002025 	move	a0,zero
80015c74:	9fc30050 	lwu	v1,80(s8)
80015c78:	9fc20058 	lwu	v0,88(s8)
80015c7c:	0062001c 	dmult	v1,v0
80015c80:	00001012 	mflo	v0
80015c84:	00022800 	sll	a1,v0,0x0
80015c88:	0005183c 	dsll32	v1,a1,0x0
80015c8c:	0003183e 	dsrl32	v1,v1,0x0
80015c90:	10430002 	beq	v0,v1,80015c9c <_calloc_r+0x48>
80015c94:	00000000 	nop
80015c98:	24040001 	li	a0,1
80015c9c:	00a01025 	move	v0,a1
80015ca0:	afc20034 	sw	v0,52(s8)
80015ca4:	00801025 	move	v0,a0
80015ca8:	30420001 	andi	v0,v0,0x1
80015cac:	304200ff 	andi	v0,v0,0xff
80015cb0:	10400009 	beqz	v0,80015cd8 <_calloc_r+0x84>
80015cb4:	00000000 	nop
80015cb8:	0c0057a3 	jal	80015e8c <__errno>
80015cbc:	00000000 	nop
80015cc0:	00401825 	move	v1,v0
80015cc4:	2402000c 	li	v0,12
80015cc8:	ac620000 	sw	v0,0(v1)
80015ccc:	00001025 	move	v0,zero
80015cd0:	10000054 	b	80015e24 <_calloc_r+0x1d0>
80015cd4:	00000000 	nop
80015cd8:	8fc20034 	lw	v0,52(s8)
80015cdc:	00402825 	move	a1,v0
80015ce0:	8fc40048 	lw	a0,72(s8)
80015ce4:	0c001296 	jal	80004a58 <_malloc_r>
80015ce8:	00000000 	nop
80015cec:	afc20024 	sw	v0,36(s8)
80015cf0:	8fc20024 	lw	v0,36(s8)
80015cf4:	14400004 	bnez	v0,80015d08 <_calloc_r+0xb4>
80015cf8:	00000000 	nop
80015cfc:	00001025 	move	v0,zero
80015d00:	10000048 	b	80015e24 <_calloc_r+0x1d0>
80015d04:	00000000 	nop
80015d08:	8fc20024 	lw	v0,36(s8)
80015d0c:	2442fff8 	addiu	v0,v0,-8
80015d10:	afc20028 	sw	v0,40(s8)
80015d14:	8fc20028 	lw	v0,40(s8)
80015d18:	8c430004 	lw	v1,4(v0)
80015d1c:	2402fffc 	li	v0,-4
80015d20:	00621024 	and	v0,v1,v0
80015d24:	afc2002c 	sw	v0,44(s8)
80015d28:	8fc2002c 	lw	v0,44(s8)
80015d2c:	2442fffc 	addiu	v0,v0,-4
80015d30:	afc20030 	sw	v0,48(s8)
80015d34:	8fc20030 	lw	v0,48(s8)
80015d38:	2c420025 	sltiu	v0,v0,37
80015d3c:	10400033 	beqz	v0,80015e0c <_calloc_r+0x1b8>
80015d40:	00000000 	nop
80015d44:	8fc20024 	lw	v0,36(s8)
80015d48:	afc20020 	sw	v0,32(s8)
80015d4c:	8fc20030 	lw	v0,48(s8)
80015d50:	2c420014 	sltiu	v0,v0,20
80015d54:	14400021 	bnez	v0,80015ddc <_calloc_r+0x188>
80015d58:	00000000 	nop
80015d5c:	8fc20020 	lw	v0,32(s8)
80015d60:	24430004 	addiu	v1,v0,4
80015d64:	afc30020 	sw	v1,32(s8)
80015d68:	ac400000 	sw	zero,0(v0)
80015d6c:	8fc20020 	lw	v0,32(s8)
80015d70:	24430004 	addiu	v1,v0,4
80015d74:	afc30020 	sw	v1,32(s8)
80015d78:	ac400000 	sw	zero,0(v0)
80015d7c:	8fc20030 	lw	v0,48(s8)
80015d80:	2c42001c 	sltiu	v0,v0,28
80015d84:	14400015 	bnez	v0,80015ddc <_calloc_r+0x188>
80015d88:	00000000 	nop
80015d8c:	8fc20020 	lw	v0,32(s8)
80015d90:	24430004 	addiu	v1,v0,4
80015d94:	afc30020 	sw	v1,32(s8)
80015d98:	ac400000 	sw	zero,0(v0)
80015d9c:	8fc20020 	lw	v0,32(s8)
80015da0:	24430004 	addiu	v1,v0,4
80015da4:	afc30020 	sw	v1,32(s8)
80015da8:	ac400000 	sw	zero,0(v0)
80015dac:	8fc20030 	lw	v0,48(s8)
80015db0:	2c420024 	sltiu	v0,v0,36
80015db4:	14400009 	bnez	v0,80015ddc <_calloc_r+0x188>
80015db8:	00000000 	nop
80015dbc:	8fc20020 	lw	v0,32(s8)
80015dc0:	24430004 	addiu	v1,v0,4
80015dc4:	afc30020 	sw	v1,32(s8)
80015dc8:	ac400000 	sw	zero,0(v0)
80015dcc:	8fc20020 	lw	v0,32(s8)
80015dd0:	24430004 	addiu	v1,v0,4
80015dd4:	afc30020 	sw	v1,32(s8)
80015dd8:	ac400000 	sw	zero,0(v0)
80015ddc:	8fc20020 	lw	v0,32(s8)
80015de0:	24430004 	addiu	v1,v0,4
80015de4:	afc30020 	sw	v1,32(s8)
80015de8:	ac400000 	sw	zero,0(v0)
80015dec:	8fc20020 	lw	v0,32(s8)
80015df0:	24430004 	addiu	v1,v0,4
80015df4:	afc30020 	sw	v1,32(s8)
80015df8:	ac400000 	sw	zero,0(v0)
80015dfc:	8fc20020 	lw	v0,32(s8)
80015e00:	ac400000 	sw	zero,0(v0)
80015e04:	10000006 	b	80015e20 <_calloc_r+0x1cc>
80015e08:	00000000 	nop
80015e0c:	8fc60030 	lw	a2,48(s8)
80015e10:	00002825 	move	a1,zero
80015e14:	8fc40024 	lw	a0,36(s8)
80015e18:	0c00173d 	jal	80005cf4 <memset>
80015e1c:	00000000 	nop
80015e20:	8fc20024 	lw	v0,36(s8)
80015e24:	03c0e825 	move	sp,s8
80015e28:	dfbf0040 	ld	ra,64(sp)
80015e2c:	dfbe0038 	ld	s8,56(sp)
80015e30:	27bd0048 	addiu	sp,sp,72
80015e34:	03e00008 	jr	ra
80015e38:	00000000 	nop

80015e3c <__env_lock>:
80015e3c:	27bdfff8 	addiu	sp,sp,-8
80015e40:	ffbe0000 	sd	s8,0(sp)
80015e44:	03a0f025 	move	s8,sp
80015e48:	afc40008 	sw	a0,8(s8)
80015e4c:	00000000 	nop
80015e50:	03c0e825 	move	sp,s8
80015e54:	dfbe0000 	ld	s8,0(sp)
80015e58:	27bd0008 	addiu	sp,sp,8
80015e5c:	03e00008 	jr	ra
80015e60:	00000000 	nop

80015e64 <__env_unlock>:
80015e64:	27bdfff8 	addiu	sp,sp,-8
80015e68:	ffbe0000 	sd	s8,0(sp)
80015e6c:	03a0f025 	move	s8,sp
80015e70:	afc40008 	sw	a0,8(s8)
80015e74:	00000000 	nop
80015e78:	03c0e825 	move	sp,s8
80015e7c:	dfbe0000 	ld	s8,0(sp)
80015e80:	27bd0008 	addiu	sp,sp,8
80015e84:	03e00008 	jr	ra
80015e88:	00000000 	nop

80015e8c <__errno>:
80015e8c:	27bdfff8 	addiu	sp,sp,-8
80015e90:	ffbe0000 	sd	s8,0(sp)
80015e94:	03a0f025 	move	s8,sp
80015e98:	8f828028 	lw	v0,-32728(gp)
80015e9c:	03c0e825 	move	sp,s8
80015ea0:	dfbe0000 	ld	s8,0(sp)
80015ea4:	27bd0008 	addiu	sp,sp,8
80015ea8:	03e00008 	jr	ra
80015eac:	00000000 	nop

80015eb0 <__sputc_r>:
80015eb0:	27bdffd0 	addiu	sp,sp,-48
80015eb4:	ffbf0028 	sd	ra,40(sp)
80015eb8:	ffbe0020 	sd	s8,32(sp)
80015ebc:	03a0f025 	move	s8,sp
80015ec0:	afc40030 	sw	a0,48(s8)
80015ec4:	afc50038 	sw	a1,56(s8)
80015ec8:	afc60040 	sw	a2,64(s8)
80015ecc:	8fc20040 	lw	v0,64(s8)
80015ed0:	8c420008 	lw	v0,8(v0)
80015ed4:	2443ffff 	addiu	v1,v0,-1
80015ed8:	8fc20040 	lw	v0,64(s8)
80015edc:	ac430008 	sw	v1,8(v0)
80015ee0:	8fc20040 	lw	v0,64(s8)
80015ee4:	8c420008 	lw	v0,8(v0)
80015ee8:	0441000e 	bgez	v0,80015f24 <__sputc_r+0x74>
80015eec:	00000000 	nop
80015ef0:	8fc20040 	lw	v0,64(s8)
80015ef4:	8c430008 	lw	v1,8(v0)
80015ef8:	8fc20040 	lw	v0,64(s8)
80015efc:	8c420018 	lw	v0,24(v0)
80015f00:	0062102a 	slt	v0,v1,v0
80015f04:	14400012 	bnez	v0,80015f50 <__sputc_r+0xa0>
80015f08:	00000000 	nop
80015f0c:	8fc20038 	lw	v0,56(s8)
80015f10:	00021e00 	sll	v1,v0,0x18
80015f14:	00031e03 	sra	v1,v1,0x18
80015f18:	2402000a 	li	v0,10
80015f1c:	1062000c 	beq	v1,v0,80015f50 <__sputc_r+0xa0>
80015f20:	00000000 	nop
80015f24:	8fc20040 	lw	v0,64(s8)
80015f28:	8c420000 	lw	v0,0(v0)
80015f2c:	24440001 	addiu	a0,v0,1
80015f30:	8fc30040 	lw	v1,64(s8)
80015f34:	ac640000 	sw	a0,0(v1)
80015f38:	8fc30038 	lw	v1,56(s8)
80015f3c:	306300ff 	andi	v1,v1,0xff
80015f40:	a0430000 	sb	v1,0(v0)
80015f44:	90420000 	lbu	v0,0(v0)
80015f48:	10000006 	b	80015f64 <__sputc_r+0xb4>
80015f4c:	00000000 	nop
80015f50:	8fc60040 	lw	a2,64(s8)
80015f54:	8fc50038 	lw	a1,56(s8)
80015f58:	8fc40030 	lw	a0,48(s8)
80015f5c:	0c002daf 	jal	8000b6bc <__swbuf_r>
80015f60:	00000000 	nop
80015f64:	03c0e825 	move	sp,s8
80015f68:	dfbf0028 	ld	ra,40(sp)
80015f6c:	dfbe0020 	ld	s8,32(sp)
80015f70:	27bd0030 	addiu	sp,sp,48
80015f74:	03e00008 	jr	ra
80015f78:	00000000 	nop

80015f7c <__fputwc>:
80015f7c:	27bdffc0 	addiu	sp,sp,-64
80015f80:	ffbf0038 	sd	ra,56(sp)
80015f84:	ffbe0030 	sd	s8,48(sp)
80015f88:	03a0f025 	move	s8,sp
80015f8c:	afc40040 	sw	a0,64(s8)
80015f90:	afc50048 	sw	a1,72(s8)
80015f94:	afc60050 	sw	a2,80(s8)
80015f98:	0c003a42 	jal	8000e908 <__locale_mb_cur_max>
80015f9c:	00000000 	nop
80015fa0:	00401825 	move	v1,v0
80015fa4:	24020001 	li	v0,1
80015fa8:	14620010 	bne	v1,v0,80015fec <__fputwc+0x70>
80015fac:	00000000 	nop
80015fb0:	8fc20048 	lw	v0,72(s8)
80015fb4:	1840000d 	blez	v0,80015fec <__fputwc+0x70>
80015fb8:	00000000 	nop
80015fbc:	8fc20048 	lw	v0,72(s8)
80015fc0:	28420100 	slti	v0,v0,256
80015fc4:	10400009 	beqz	v0,80015fec <__fputwc+0x70>
80015fc8:	00000000 	nop
80015fcc:	8fc20048 	lw	v0,72(s8)
80015fd0:	00021600 	sll	v0,v0,0x18
80015fd4:	00021603 	sra	v0,v0,0x18
80015fd8:	a3c20028 	sb	v0,40(s8)
80015fdc:	24020001 	li	v0,1
80015fe0:	afc20024 	sw	v0,36(s8)
80015fe4:	10000019 	b	8001604c <__fputwc+0xd0>
80015fe8:	00000000 	nop
80015fec:	8fc20050 	lw	v0,80(s8)
80015ff0:	2442005c 	addiu	v0,v0,92
80015ff4:	27c30028 	addiu	v1,s8,40
80015ff8:	00403825 	move	a3,v0
80015ffc:	8fc60048 	lw	a2,72(s8)
80016000:	00602825 	move	a1,v1
80016004:	8fc40040 	lw	a0,64(s8)
80016008:	0c005d35 	jal	800174d4 <_wcrtomb_r>
8001600c:	00000000 	nop
80016010:	afc20024 	sw	v0,36(s8)
80016014:	8fc30024 	lw	v1,36(s8)
80016018:	2402ffff 	li	v0,-1
8001601c:	1462000b 	bne	v1,v0,8001604c <__fputwc+0xd0>
80016020:	00000000 	nop
80016024:	8fc20050 	lw	v0,80(s8)
80016028:	8442000c 	lh	v0,12(v0)
8001602c:	34420040 	ori	v0,v0,0x40
80016030:	00021c00 	sll	v1,v0,0x10
80016034:	00031c03 	sra	v1,v1,0x10
80016038:	8fc20050 	lw	v0,80(s8)
8001603c:	a443000c 	sh	v1,12(v0)
80016040:	2402ffff 	li	v0,-1
80016044:	1000001e 	b	800160c0 <__fputwc+0x144>
80016048:	00000000 	nop
8001604c:	afc00020 	sw	zero,32(s8)
80016050:	10000015 	b	800160a8 <__fputwc+0x12c>
80016054:	00000000 	nop
80016058:	8fc20020 	lw	v0,32(s8)
8001605c:	27c30020 	addiu	v1,s8,32
80016060:	00621021 	addu	v0,v1,v0
80016064:	80420008 	lb	v0,8(v0)
80016068:	304200ff 	andi	v0,v0,0xff
8001606c:	8fc60050 	lw	a2,80(s8)
80016070:	00402825 	move	a1,v0
80016074:	8fc40040 	lw	a0,64(s8)
80016078:	0c0057ac 	jal	80015eb0 <__sputc_r>
8001607c:	00000000 	nop
80016080:	00401825 	move	v1,v0
80016084:	2402ffff 	li	v0,-1
80016088:	14620004 	bne	v1,v0,8001609c <__fputwc+0x120>
8001608c:	00000000 	nop
80016090:	2402ffff 	li	v0,-1
80016094:	1000000a 	b	800160c0 <__fputwc+0x144>
80016098:	00000000 	nop
8001609c:	8fc20020 	lw	v0,32(s8)
800160a0:	24420001 	addiu	v0,v0,1
800160a4:	afc20020 	sw	v0,32(s8)
800160a8:	8fc30020 	lw	v1,32(s8)
800160ac:	8fc20024 	lw	v0,36(s8)
800160b0:	0062102b 	sltu	v0,v1,v0
800160b4:	1440ffe8 	bnez	v0,80016058 <__fputwc+0xdc>
800160b8:	00000000 	nop
800160bc:	8fc20048 	lw	v0,72(s8)
800160c0:	03c0e825 	move	sp,s8
800160c4:	dfbf0038 	ld	ra,56(sp)
800160c8:	dfbe0030 	ld	s8,48(sp)
800160cc:	27bd0040 	addiu	sp,sp,64
800160d0:	03e00008 	jr	ra
800160d4:	00000000 	nop

800160d8 <_fputwc_r>:
800160d8:	27bdffc8 	addiu	sp,sp,-56
800160dc:	ffbf0030 	sd	ra,48(sp)
800160e0:	ffbe0028 	sd	s8,40(sp)
800160e4:	03a0f025 	move	s8,sp
800160e8:	afc40038 	sw	a0,56(s8)
800160ec:	afc50040 	sw	a1,64(s8)
800160f0:	afc60048 	sw	a2,72(s8)
800160f4:	8fc20048 	lw	v0,72(s8)
800160f8:	8442000c 	lh	v0,12(v0)
800160fc:	3042ffff 	andi	v0,v0,0xffff
80016100:	30422000 	andi	v0,v0,0x2000
80016104:	1440000d 	bnez	v0,8001613c <_fputwc_r+0x64>
80016108:	00000000 	nop
8001610c:	8fc20048 	lw	v0,72(s8)
80016110:	8442000c 	lh	v0,12(v0)
80016114:	34422000 	ori	v0,v0,0x2000
80016118:	00021c00 	sll	v1,v0,0x10
8001611c:	00031c03 	sra	v1,v1,0x10
80016120:	8fc20048 	lw	v0,72(s8)
80016124:	a443000c 	sh	v1,12(v0)
80016128:	8fc20048 	lw	v0,72(s8)
8001612c:	8c420064 	lw	v0,100(v0)
80016130:	34432000 	ori	v1,v0,0x2000
80016134:	8fc20048 	lw	v0,72(s8)
80016138:	ac430064 	sw	v1,100(v0)
8001613c:	8fc60048 	lw	a2,72(s8)
80016140:	8fc50040 	lw	a1,64(s8)
80016144:	8fc40038 	lw	a0,56(s8)
80016148:	0c0057df 	jal	80015f7c <__fputwc>
8001614c:	00000000 	nop
80016150:	afc20020 	sw	v0,32(s8)
80016154:	8fc20020 	lw	v0,32(s8)
80016158:	03c0e825 	move	sp,s8
8001615c:	dfbf0030 	ld	ra,48(sp)
80016160:	dfbe0028 	ld	s8,40(sp)
80016164:	27bd0038 	addiu	sp,sp,56
80016168:	03e00008 	jr	ra
8001616c:	00000000 	nop

80016170 <fputwc>:
80016170:	27bdffc8 	addiu	sp,sp,-56
80016174:	ffbf0030 	sd	ra,48(sp)
80016178:	ffbe0028 	sd	s8,40(sp)
8001617c:	03a0f025 	move	s8,sp
80016180:	afc40038 	sw	a0,56(s8)
80016184:	afc50040 	sw	a1,64(s8)
80016188:	8f828028 	lw	v0,-32728(gp)
8001618c:	afc20020 	sw	v0,32(s8)
80016190:	8fc20020 	lw	v0,32(s8)
80016194:	afc20024 	sw	v0,36(s8)
80016198:	8fc20024 	lw	v0,36(s8)
8001619c:	10400008 	beqz	v0,800161c0 <fputwc+0x50>
800161a0:	00000000 	nop
800161a4:	8fc20024 	lw	v0,36(s8)
800161a8:	8c420038 	lw	v0,56(v0)
800161ac:	14400004 	bnez	v0,800161c0 <fputwc+0x50>
800161b0:	00000000 	nop
800161b4:	8fc40024 	lw	a0,36(s8)
800161b8:	0c000cc3 	jal	8000330c <__sinit>
800161bc:	00000000 	nop
800161c0:	8fc60040 	lw	a2,64(s8)
800161c4:	8fc50038 	lw	a1,56(s8)
800161c8:	8fc40020 	lw	a0,32(s8)
800161cc:	0c005836 	jal	800160d8 <_fputwc_r>
800161d0:	00000000 	nop
800161d4:	03c0e825 	move	sp,s8
800161d8:	dfbf0030 	ld	ra,48(sp)
800161dc:	dfbe0028 	ld	s8,40(sp)
800161e0:	27bd0038 	addiu	sp,sp,56
800161e4:	03e00008 	jr	ra
800161e8:	00000000 	nop

800161ec <iswspace>:
800161ec:	27bdffd0 	addiu	sp,sp,-48
800161f0:	ffbf0028 	sd	ra,40(sp)
800161f4:	ffbe0020 	sd	s8,32(sp)
800161f8:	03a0f025 	move	s8,sp
800161fc:	afc40030 	sw	a0,48(s8)
80016200:	00002825 	move	a1,zero
80016204:	8fc40030 	lw	a0,48(s8)
80016208:	0c005da2 	jal	80017688 <iswspace_l>
8001620c:	00000000 	nop
80016210:	03c0e825 	move	sp,s8
80016214:	dfbf0028 	ld	ra,40(sp)
80016218:	dfbe0020 	ld	s8,32(sp)
8001621c:	27bd0030 	addiu	sp,sp,48
80016220:	03e00008 	jr	ra
80016224:	00000000 	nop

80016228 <__get_global_locale>:
80016228:	27bdfff8 	addiu	sp,sp,-8
8001622c:	ffbe0000 	sd	s8,0(sp)
80016230:	03a0f025 	move	s8,sp
80016234:	3c028002 	lui	v0,0x8002
80016238:	2442d2f8 	addiu	v0,v0,-11528
8001623c:	03c0e825 	move	sp,s8
80016240:	dfbe0000 	ld	s8,0(sp)
80016244:	27bd0008 	addiu	sp,sp,8
80016248:	03e00008 	jr	ra
8001624c:	00000000 	nop

80016250 <__get_current_locale>:
80016250:	27bdffd0 	addiu	sp,sp,-48
80016254:	ffbf0028 	sd	ra,40(sp)
80016258:	ffbe0020 	sd	s8,32(sp)
8001625c:	03a0f025 	move	s8,sp
80016260:	0c00588a 	jal	80016228 <__get_global_locale>
80016264:	00000000 	nop
80016268:	03c0e825 	move	sp,s8
8001626c:	dfbf0028 	ld	ra,40(sp)
80016270:	dfbe0020 	ld	s8,32(sp)
80016274:	27bd0030 	addiu	sp,sp,48
80016278:	03e00008 	jr	ra
8001627c:	00000000 	nop

80016280 <_mbrtowc_r>:
80016280:	27bdffc0 	addiu	sp,sp,-64
80016284:	ffbf0038 	sd	ra,56(sp)
80016288:	ffbe0030 	sd	s8,48(sp)
8001628c:	03a0f025 	move	s8,sp
80016290:	afc40040 	sw	a0,64(s8)
80016294:	afc50048 	sw	a1,72(s8)
80016298:	afc60050 	sw	a2,80(s8)
8001629c:	afc70058 	sw	a3,88(s8)
800162a0:	afc00028 	sw	zero,40(s8)
800162a4:	8fc20050 	lw	v0,80(s8)
800162a8:	14400010 	bnez	v0,800162ec <_mbrtowc_r+0x6c>
800162ac:	00000000 	nop
800162b0:	0c005894 	jal	80016250 <__get_current_locale>
800162b4:	00000000 	nop
800162b8:	8c4200e4 	lw	v0,228(v0)
800162bc:	8fc30064 	lw	v1,100(s8)
800162c0:	afa30024 	sw	v1,36(sp)
800162c4:	24070001 	li	a3,1
800162c8:	3c038002 	lui	v1,0x8002
800162cc:	2466b068 	addiu	a2,v1,-20376
800162d0:	00002825 	move	a1,zero
800162d4:	8fc40040 	lw	a0,64(s8)
800162d8:	0040f809 	jalr	v0
800162dc:	00000000 	nop
800162e0:	afc20028 	sw	v0,40(s8)
800162e4:	1000000d 	b	8001631c <_mbrtowc_r+0x9c>
800162e8:	00000000 	nop
800162ec:	0c005894 	jal	80016250 <__get_current_locale>
800162f0:	00000000 	nop
800162f4:	8c4200e4 	lw	v0,228(v0)
800162f8:	8fc30064 	lw	v1,100(s8)
800162fc:	afa30024 	sw	v1,36(sp)
80016300:	8fc70058 	lw	a3,88(s8)
80016304:	8fc60050 	lw	a2,80(s8)
80016308:	8fc50048 	lw	a1,72(s8)
8001630c:	8fc40040 	lw	a0,64(s8)
80016310:	0040f809 	jalr	v0
80016314:	00000000 	nop
80016318:	afc20028 	sw	v0,40(s8)
8001631c:	8fc30028 	lw	v1,40(s8)
80016320:	2402ffff 	li	v0,-1
80016324:	14620009 	bne	v1,v0,8001634c <_mbrtowc_r+0xcc>
80016328:	00000000 	nop
8001632c:	8fc20064 	lw	v0,100(s8)
80016330:	ac400000 	sw	zero,0(v0)
80016334:	8fc20040 	lw	v0,64(s8)
80016338:	2403008a 	li	v1,138
8001633c:	ac430000 	sw	v1,0(v0)
80016340:	2402ffff 	li	v0,-1
80016344:	10000002 	b	80016350 <_mbrtowc_r+0xd0>
80016348:	00000000 	nop
8001634c:	8fc20028 	lw	v0,40(s8)
80016350:	03c0e825 	move	sp,s8
80016354:	dfbf0038 	ld	ra,56(sp)
80016358:	dfbe0030 	ld	s8,48(sp)
8001635c:	27bd0040 	addiu	sp,sp,64
80016360:	03e00008 	jr	ra
80016364:	00000000 	nop

80016368 <mbrtowc>:
80016368:	27bdffc0 	addiu	sp,sp,-64
8001636c:	ffbf0038 	sd	ra,56(sp)
80016370:	ffbe0030 	sd	s8,48(sp)
80016374:	03a0f025 	move	s8,sp
80016378:	afc40040 	sw	a0,64(s8)
8001637c:	afc50048 	sw	a1,72(s8)
80016380:	afc60050 	sw	a2,80(s8)
80016384:	afc70058 	sw	a3,88(s8)
80016388:	afc00028 	sw	zero,40(s8)
8001638c:	8f828028 	lw	v0,-32728(gp)
80016390:	afc2002c 	sw	v0,44(s8)
80016394:	8fc20048 	lw	v0,72(s8)
80016398:	14400010 	bnez	v0,800163dc <mbrtowc+0x74>
8001639c:	00000000 	nop
800163a0:	0c005894 	jal	80016250 <__get_current_locale>
800163a4:	00000000 	nop
800163a8:	8c4200e4 	lw	v0,228(v0)
800163ac:	8fc30058 	lw	v1,88(s8)
800163b0:	afa30024 	sw	v1,36(sp)
800163b4:	24070001 	li	a3,1
800163b8:	3c038002 	lui	v1,0x8002
800163bc:	2466b068 	addiu	a2,v1,-20376
800163c0:	00002825 	move	a1,zero
800163c4:	8fc4002c 	lw	a0,44(s8)
800163c8:	0040f809 	jalr	v0
800163cc:	00000000 	nop
800163d0:	afc20028 	sw	v0,40(s8)
800163d4:	1000000d 	b	8001640c <mbrtowc+0xa4>
800163d8:	00000000 	nop
800163dc:	0c005894 	jal	80016250 <__get_current_locale>
800163e0:	00000000 	nop
800163e4:	8c4200e4 	lw	v0,228(v0)
800163e8:	8fc30058 	lw	v1,88(s8)
800163ec:	afa30024 	sw	v1,36(sp)
800163f0:	8fc70050 	lw	a3,80(s8)
800163f4:	8fc60048 	lw	a2,72(s8)
800163f8:	8fc50040 	lw	a1,64(s8)
800163fc:	8fc4002c 	lw	a0,44(s8)
80016400:	0040f809 	jalr	v0
80016404:	00000000 	nop
80016408:	afc20028 	sw	v0,40(s8)
8001640c:	8fc30028 	lw	v1,40(s8)
80016410:	2402ffff 	li	v0,-1
80016414:	14620009 	bne	v1,v0,8001643c <mbrtowc+0xd4>
80016418:	00000000 	nop
8001641c:	8fc20058 	lw	v0,88(s8)
80016420:	ac400000 	sw	zero,0(v0)
80016424:	8fc2002c 	lw	v0,44(s8)
80016428:	2403008a 	li	v1,138
8001642c:	ac430000 	sw	v1,0(v0)
80016430:	2402ffff 	li	v0,-1
80016434:	10000002 	b	80016440 <mbrtowc+0xd8>
80016438:	00000000 	nop
8001643c:	8fc20028 	lw	v0,40(s8)
80016440:	03c0e825 	move	sp,s8
80016444:	dfbf0038 	ld	ra,56(sp)
80016448:	dfbe0030 	ld	s8,48(sp)
8001644c:	27bd0040 	addiu	sp,sp,64
80016450:	03e00008 	jr	ra
80016454:	00000000 	nop

80016458 <__sccl>:
80016458:	27bdffe0 	addiu	sp,sp,-32
8001645c:	ffbe0018 	sd	s8,24(sp)
80016460:	ffb20010 	sd	s2,16(sp)
80016464:	ffb10008 	sd	s1,8(sp)
80016468:	ffb00000 	sd	s0,0(sp)
8001646c:	03a0f025 	move	s8,sp
80016470:	00801825 	move	v1,a0
80016474:	00a01025 	move	v0,a1
80016478:	00402025 	move	a0,v0
8001647c:	24820001 	addiu	v0,a0,1
80016480:	90840000 	lbu	a0,0(a0)
80016484:	00808825 	move	s1,a0
80016488:	2404005e 	li	a0,94
8001648c:	16240008 	bne	s1,a0,800164b0 <__sccl+0x58>
80016490:	00000000 	nop
80016494:	24120001 	li	s2,1
80016498:	00402025 	move	a0,v0
8001649c:	24820001 	addiu	v0,a0,1
800164a0:	90840000 	lbu	a0,0(a0)
800164a4:	00808825 	move	s1,a0
800164a8:	10000002 	b	800164b4 <__sccl+0x5c>
800164ac:	00000000 	nop
800164b0:	00009025 	move	s2,zero
800164b4:	00008025 	move	s0,zero
800164b8:	10000007 	b	800164d8 <__sccl+0x80>
800164bc:	00000000 	nop
800164c0:	02002025 	move	a0,s0
800164c4:	00642021 	addu	a0,v1,a0
800164c8:	00122e00 	sll	a1,s2,0x18
800164cc:	00052e03 	sra	a1,a1,0x18
800164d0:	a0850000 	sb	a1,0(a0)
800164d4:	26100001 	addiu	s0,s0,1
800164d8:	2a040100 	slti	a0,s0,256
800164dc:	1480fff8 	bnez	a0,800164c0 <__sccl+0x68>
800164e0:	00000000 	nop
800164e4:	16200004 	bnez	s1,800164f8 <__sccl+0xa0>
800164e8:	00000000 	nop
800164ec:	2442ffff 	addiu	v0,v0,-1
800164f0:	10000037 	b	800165d0 <__sccl+0x178>
800164f4:	00000000 	nop
800164f8:	24040001 	li	a0,1
800164fc:	00929023 	subu	s2,a0,s2
80016500:	02202025 	move	a0,s1
80016504:	00642021 	addu	a0,v1,a0
80016508:	00122e00 	sll	a1,s2,0x18
8001650c:	00052e03 	sra	a1,a1,0x18
80016510:	a0850000 	sb	a1,0(a0)
80016514:	00402025 	move	a0,v0
80016518:	24820001 	addiu	v0,a0,1
8001651c:	90840000 	lbu	a0,0(a0)
80016520:	00808025 	move	s0,a0
80016524:	2404005d 	li	a0,93
80016528:	12040029 	beq	s0,a0,800165d0 <__sccl+0x178>
8001652c:	00000000 	nop
80016530:	2a04005e 	slti	a0,s0,94
80016534:	10800022 	beqz	a0,800165c0 <__sccl+0x168>
80016538:	00000000 	nop
8001653c:	12000006 	beqz	s0,80016558 <__sccl+0x100>
80016540:	00000000 	nop
80016544:	2404002d 	li	a0,45
80016548:	12040006 	beq	s0,a0,80016564 <__sccl+0x10c>
8001654c:	00000000 	nop
80016550:	1000001b 	b	800165c0 <__sccl+0x168>
80016554:	00000000 	nop
80016558:	2442ffff 	addiu	v0,v0,-1
8001655c:	1000001c 	b	800165d0 <__sccl+0x178>
80016560:	00000000 	nop
80016564:	90440000 	lbu	a0,0(v0)
80016568:	00808025 	move	s0,a0
8001656c:	2404005d 	li	a0,93
80016570:	12040004 	beq	s0,a0,80016584 <__sccl+0x12c>
80016574:	00000000 	nop
80016578:	0211202a 	slt	a0,s0,s1
8001657c:	10800004 	beqz	a0,80016590 <__sccl+0x138>
80016580:	00000000 	nop
80016584:	2411002d 	li	s1,45
80016588:	1000000f 	b	800165c8 <__sccl+0x170>
8001658c:	00000000 	nop
80016590:	24420001 	addiu	v0,v0,1
80016594:	26310001 	addiu	s1,s1,1
80016598:	02202025 	move	a0,s1
8001659c:	00642021 	addu	a0,v1,a0
800165a0:	00122e00 	sll	a1,s2,0x18
800165a4:	00052e03 	sra	a1,a1,0x18
800165a8:	a0850000 	sb	a1,0(a0)
800165ac:	0230202a 	slt	a0,s1,s0
800165b0:	1480fff8 	bnez	a0,80016594 <__sccl+0x13c>
800165b4:	00000000 	nop
800165b8:	1000ffd6 	b	80016514 <__sccl+0xbc>
800165bc:	00000000 	nop
800165c0:	02008825 	move	s1,s0
800165c4:	00000000 	nop
800165c8:	1000ffcd 	b	80016500 <__sccl+0xa8>
800165cc:	00000000 	nop
800165d0:	03c0e825 	move	sp,s8
800165d4:	dfbe0018 	ld	s8,24(sp)
800165d8:	dfb20010 	ld	s2,16(sp)
800165dc:	dfb10008 	ld	s1,8(sp)
800165e0:	dfb00000 	ld	s0,0(sp)
800165e4:	27bd0020 	addiu	sp,sp,32
800165e8:	03e00008 	jr	ra
800165ec:	00000000 	nop

800165f0 <__locale_ctype_ptr_l>:
800165f0:	27bdfff8 	addiu	sp,sp,-8
800165f4:	ffbe0000 	sd	s8,0(sp)
800165f8:	03a0f025 	move	s8,sp
800165fc:	afc40008 	sw	a0,8(s8)
80016600:	3c028002 	lui	v0,0x8002
80016604:	2442af60 	addiu	v0,v0,-20640
80016608:	03c0e825 	move	sp,s8
8001660c:	dfbe0000 	ld	s8,0(sp)
80016610:	27bd0008 	addiu	sp,sp,8
80016614:	03e00008 	jr	ra
80016618:	00000000 	nop

8001661c <__get_global_locale>:
8001661c:	27bdfff8 	addiu	sp,sp,-8
80016620:	ffbe0000 	sd	s8,0(sp)
80016624:	03a0f025 	move	s8,sp
80016628:	3c028002 	lui	v0,0x8002
8001662c:	2442d2f8 	addiu	v0,v0,-11528
80016630:	03c0e825 	move	sp,s8
80016634:	dfbe0000 	ld	s8,0(sp)
80016638:	27bd0008 	addiu	sp,sp,8
8001663c:	03e00008 	jr	ra
80016640:	00000000 	nop

80016644 <__get_current_locale>:
80016644:	27bdffd0 	addiu	sp,sp,-48
80016648:	ffbf0028 	sd	ra,40(sp)
8001664c:	ffbe0020 	sd	s8,32(sp)
80016650:	03a0f025 	move	s8,sp
80016654:	0c005987 	jal	8001661c <__get_global_locale>
80016658:	00000000 	nop
8001665c:	03c0e825 	move	sp,s8
80016660:	dfbf0028 	ld	ra,40(sp)
80016664:	dfbe0020 	ld	s8,32(sp)
80016668:	27bd0030 	addiu	sp,sp,48
8001666c:	03e00008 	jr	ra
80016670:	00000000 	nop

80016674 <_strtoll_l>:
80016674:	27bdff98 	addiu	sp,sp,-104
80016678:	ffbf0060 	sd	ra,96(sp)
8001667c:	ffbe0058 	sd	s8,88(sp)
80016680:	ffb60050 	sd	s6,80(sp)
80016684:	ffb50048 	sd	s5,72(sp)
80016688:	ffb40040 	sd	s4,64(sp)
8001668c:	ffb30038 	sd	s3,56(sp)
80016690:	ffb20030 	sd	s2,48(sp)
80016694:	ffb10028 	sd	s1,40(sp)
80016698:	ffb00020 	sd	s0,32(sp)
8001669c:	03a0f025 	move	s8,sp
800166a0:	afc40068 	sw	a0,104(s8)
800166a4:	afc50070 	sw	a1,112(s8)
800166a8:	afc60078 	sw	a2,120(s8)
800166ac:	afc70080 	sw	a3,128(s8)
800166b0:	8fd10070 	lw	s1,112(s8)
800166b4:	0000a825 	move	s5,zero
800166b8:	02201025 	move	v0,s1
800166bc:	24510001 	addiu	s1,v0,1
800166c0:	90420000 	lbu	v0,0(v0)
800166c4:	00408025 	move	s0,v0
800166c8:	8fc4008c 	lw	a0,140(s8)
800166cc:	0c00597c 	jal	800165f0 <__locale_ctype_ptr_l>
800166d0:	00000000 	nop
800166d4:	00401825 	move	v1,v0
800166d8:	02001025 	move	v0,s0
800166dc:	24420001 	addiu	v0,v0,1
800166e0:	00621021 	addu	v0,v1,v0
800166e4:	80420000 	lb	v0,0(v0)
800166e8:	304200ff 	andi	v0,v0,0xff
800166ec:	30420008 	andi	v0,v0,0x8
800166f0:	1440fff1 	bnez	v0,800166b8 <_strtoll_l+0x44>
800166f4:	00000000 	nop
800166f8:	2402002d 	li	v0,45
800166fc:	16020008 	bne	s0,v0,80016720 <_strtoll_l+0xac>
80016700:	00000000 	nop
80016704:	24150001 	li	s5,1
80016708:	02201025 	move	v0,s1
8001670c:	24510001 	addiu	s1,v0,1
80016710:	90420000 	lbu	v0,0(v0)
80016714:	00408025 	move	s0,v0
80016718:	10000008 	b	8001673c <_strtoll_l+0xc8>
8001671c:	00000000 	nop
80016720:	2402002b 	li	v0,43
80016724:	16020005 	bne	s0,v0,8001673c <_strtoll_l+0xc8>
80016728:	00000000 	nop
8001672c:	02201025 	move	v0,s1
80016730:	24510001 	addiu	s1,v0,1
80016734:	90420000 	lbu	v0,0(v0)
80016738:	00408025 	move	s0,v0
8001673c:	8fc20080 	lw	v0,128(s8)
80016740:	10400005 	beqz	v0,80016758 <_strtoll_l+0xe4>
80016744:	00000000 	nop
80016748:	8fc30080 	lw	v1,128(s8)
8001674c:	24020010 	li	v0,16
80016750:	14620012 	bne	v1,v0,8001679c <_strtoll_l+0x128>
80016754:	00000000 	nop
80016758:	24020030 	li	v0,48
8001675c:	1602000f 	bne	s0,v0,8001679c <_strtoll_l+0x128>
80016760:	00000000 	nop
80016764:	92230000 	lbu	v1,0(s1)
80016768:	24020078 	li	v0,120
8001676c:	10620005 	beq	v1,v0,80016784 <_strtoll_l+0x110>
80016770:	00000000 	nop
80016774:	92230000 	lbu	v1,0(s1)
80016778:	24020058 	li	v0,88
8001677c:	14620007 	bne	v1,v0,8001679c <_strtoll_l+0x128>
80016780:	00000000 	nop
80016784:	26220001 	addiu	v0,s1,1
80016788:	90420000 	lbu	v0,0(v0)
8001678c:	00408025 	move	s0,v0
80016790:	26310002 	addiu	s1,s1,2
80016794:	24020010 	li	v0,16
80016798:	afc20080 	sw	v0,128(s8)
8001679c:	8fc20080 	lw	v0,128(s8)
800167a0:	14400009 	bnez	v0,800167c8 <_strtoll_l+0x154>
800167a4:	00000000 	nop
800167a8:	24020030 	li	v0,48
800167ac:	16020004 	bne	s0,v0,800167c0 <_strtoll_l+0x14c>
800167b0:	00000000 	nop
800167b4:	24020008 	li	v0,8
800167b8:	10000002 	b	800167c4 <_strtoll_l+0x150>
800167bc:	00000000 	nop
800167c0:	2402000a 	li	v0,10
800167c4:	afc20080 	sw	v0,128(s8)
800167c8:	12a00005 	beqz	s5,800167e0 <_strtoll_l+0x16c>
800167cc:	00000000 	nop
800167d0:	2402ffff 	li	v0,-1
800167d4:	000217fc 	dsll32	v0,v0,0x1f
800167d8:	10000004 	b	800167ec <_strtoll_l+0x178>
800167dc:	00000000 	nop
800167e0:	2402ffff 	li	v0,-1
800167e4:	000217fc 	dsll32	v0,v0,0x1f
800167e8:	6442ffff 	daddiu	v0,v0,-1
800167ec:	00409825 	move	s3,v0
800167f0:	8fc20080 	lw	v0,128(s8)
800167f4:	0262001f 	ddivu	zero,s3,v0
800167f8:	004001f4 	teq	v0,zero,0x7
800167fc:	00001010 	mfhi	v0
80016800:	0002b000 	sll	s6,v0,0x0
80016804:	8fc20080 	lw	v0,128(s8)
80016808:	0262001f 	ddivu	zero,s3,v0
8001680c:	004001f4 	teq	v0,zero,0x7
80016810:	00001010 	mfhi	v0
80016814:	00009812 	mflo	s3
80016818:	00009025 	move	s2,zero
8001681c:	0000a025 	move	s4,zero
80016820:	2a020030 	slti	v0,s0,48
80016824:	14400007 	bnez	v0,80016844 <_strtoll_l+0x1d0>
80016828:	00000000 	nop
8001682c:	2a02003a 	slti	v0,s0,58
80016830:	10400004 	beqz	v0,80016844 <_strtoll_l+0x1d0>
80016834:	00000000 	nop
80016838:	2610ffd0 	addiu	s0,s0,-48
8001683c:	10000011 	b	80016884 <_strtoll_l+0x210>
80016840:	00000000 	nop
80016844:	2a020041 	slti	v0,s0,65
80016848:	14400007 	bnez	v0,80016868 <_strtoll_l+0x1f4>
8001684c:	00000000 	nop
80016850:	2a02005b 	slti	v0,s0,91
80016854:	10400004 	beqz	v0,80016868 <_strtoll_l+0x1f4>
80016858:	00000000 	nop
8001685c:	2610ffc9 	addiu	s0,s0,-55
80016860:	10000008 	b	80016884 <_strtoll_l+0x210>
80016864:	00000000 	nop
80016868:	2a020061 	slti	v0,s0,97
8001686c:	14400023 	bnez	v0,800168fc <_strtoll_l+0x288>
80016870:	00000000 	nop
80016874:	2a02007b 	slti	v0,s0,123
80016878:	10400020 	beqz	v0,800168fc <_strtoll_l+0x288>
8001687c:	00000000 	nop
80016880:	2610ffa9 	addiu	s0,s0,-87
80016884:	8fc20080 	lw	v0,128(s8)
80016888:	0202102a 	slt	v0,s0,v0
8001688c:	1040001a 	beqz	v0,800168f8 <_strtoll_l+0x284>
80016890:	00000000 	nop
80016894:	06800009 	bltz	s4,800168bc <_strtoll_l+0x248>
80016898:	00000000 	nop
8001689c:	0272102b 	sltu	v0,s3,s2
800168a0:	14400006 	bnez	v0,800168bc <_strtoll_l+0x248>
800168a4:	00000000 	nop
800168a8:	16530007 	bne	s2,s3,800168c8 <_strtoll_l+0x254>
800168ac:	00000000 	nop
800168b0:	02d0102a 	slt	v0,s6,s0
800168b4:	10400004 	beqz	v0,800168c8 <_strtoll_l+0x254>
800168b8:	00000000 	nop
800168bc:	2414ffff 	li	s4,-1
800168c0:	10000007 	b	800168e0 <_strtoll_l+0x26c>
800168c4:	00000000 	nop
800168c8:	24140001 	li	s4,1
800168cc:	8fc20080 	lw	v0,128(s8)
800168d0:	0242001c 	dmult	s2,v0
800168d4:	00009012 	mflo	s2
800168d8:	02001025 	move	v0,s0
800168dc:	0242902d 	daddu	s2,s2,v0
800168e0:	02201025 	move	v0,s1
800168e4:	24510001 	addiu	s1,v0,1
800168e8:	90420000 	lbu	v0,0(v0)
800168ec:	00408025 	move	s0,v0
800168f0:	1000ffcb 	b	80016820 <_strtoll_l+0x1ac>
800168f4:	00000000 	nop
800168f8:	00000000 	nop
800168fc:	06810010 	bgez	s4,80016940 <_strtoll_l+0x2cc>
80016900:	00000000 	nop
80016904:	12a00005 	beqz	s5,8001691c <_strtoll_l+0x2a8>
80016908:	00000000 	nop
8001690c:	2402ffff 	li	v0,-1
80016910:	000217fc 	dsll32	v0,v0,0x1f
80016914:	10000004 	b	80016928 <_strtoll_l+0x2b4>
80016918:	00000000 	nop
8001691c:	2402ffff 	li	v0,-1
80016920:	000217fc 	dsll32	v0,v0,0x1f
80016924:	6442ffff 	daddiu	v0,v0,-1
80016928:	00409025 	move	s2,v0
8001692c:	8fc20068 	lw	v0,104(s8)
80016930:	24030022 	li	v1,34
80016934:	ac430000 	sw	v1,0(v0)
80016938:	10000004 	b	8001694c <_strtoll_l+0x2d8>
8001693c:	00000000 	nop
80016940:	12a00002 	beqz	s5,8001694c <_strtoll_l+0x2d8>
80016944:	00000000 	nop
80016948:	0012902f 	dnegu	s2,s2
8001694c:	8fc20078 	lw	v0,120(s8)
80016950:	10400009 	beqz	v0,80016978 <_strtoll_l+0x304>
80016954:	00000000 	nop
80016958:	12800004 	beqz	s4,8001696c <_strtoll_l+0x2f8>
8001695c:	00000000 	nop
80016960:	2622ffff 	addiu	v0,s1,-1
80016964:	10000002 	b	80016970 <_strtoll_l+0x2fc>
80016968:	00000000 	nop
8001696c:	8fc20070 	lw	v0,112(s8)
80016970:	8fc30078 	lw	v1,120(s8)
80016974:	ac620000 	sw	v0,0(v1)
80016978:	02401025 	move	v0,s2
8001697c:	03c0e825 	move	sp,s8
80016980:	dfbf0060 	ld	ra,96(sp)
80016984:	dfbe0058 	ld	s8,88(sp)
80016988:	dfb60050 	ld	s6,80(sp)
8001698c:	dfb50048 	ld	s5,72(sp)
80016990:	dfb40040 	ld	s4,64(sp)
80016994:	dfb30038 	ld	s3,56(sp)
80016998:	dfb20030 	ld	s2,48(sp)
8001699c:	dfb10028 	ld	s1,40(sp)
800169a0:	dfb00020 	ld	s0,32(sp)
800169a4:	27bd0068 	addiu	sp,sp,104
800169a8:	03e00008 	jr	ra
800169ac:	00000000 	nop

800169b0 <_strtoll_r>:
800169b0:	27bdffc8 	addiu	sp,sp,-56
800169b4:	ffbf0030 	sd	ra,48(sp)
800169b8:	ffbe0028 	sd	s8,40(sp)
800169bc:	03a0f025 	move	s8,sp
800169c0:	afc40038 	sw	a0,56(s8)
800169c4:	afc50040 	sw	a1,64(s8)
800169c8:	afc60048 	sw	a2,72(s8)
800169cc:	afc70050 	sw	a3,80(s8)
800169d0:	0c005991 	jal	80016644 <__get_current_locale>
800169d4:	00000000 	nop
800169d8:	afa20024 	sw	v0,36(sp)
800169dc:	8fc70050 	lw	a3,80(s8)
800169e0:	8fc60048 	lw	a2,72(s8)
800169e4:	8fc50040 	lw	a1,64(s8)
800169e8:	8fc40038 	lw	a0,56(s8)
800169ec:	0c00599d 	jal	80016674 <_strtoll_l>
800169f0:	00000000 	nop
800169f4:	03c0e825 	move	sp,s8
800169f8:	dfbf0030 	ld	ra,48(sp)
800169fc:	dfbe0028 	ld	s8,40(sp)
80016a00:	27bd0038 	addiu	sp,sp,56
80016a04:	03e00008 	jr	ra
80016a08:	00000000 	nop

80016a0c <strtoll_l>:
80016a0c:	27bdffc8 	addiu	sp,sp,-56
80016a10:	ffbf0030 	sd	ra,48(sp)
80016a14:	ffbe0028 	sd	s8,40(sp)
80016a18:	03a0f025 	move	s8,sp
80016a1c:	afc40038 	sw	a0,56(s8)
80016a20:	afc50040 	sw	a1,64(s8)
80016a24:	afc60048 	sw	a2,72(s8)
80016a28:	afc70050 	sw	a3,80(s8)
80016a2c:	8f838028 	lw	v1,-32728(gp)
80016a30:	8fc20050 	lw	v0,80(s8)
80016a34:	afa20024 	sw	v0,36(sp)
80016a38:	8fc70048 	lw	a3,72(s8)
80016a3c:	8fc60040 	lw	a2,64(s8)
80016a40:	8fc50038 	lw	a1,56(s8)
80016a44:	00602025 	move	a0,v1
80016a48:	0c00599d 	jal	80016674 <_strtoll_l>
80016a4c:	00000000 	nop
80016a50:	03c0e825 	move	sp,s8
80016a54:	dfbf0030 	ld	ra,48(sp)
80016a58:	dfbe0028 	ld	s8,40(sp)
80016a5c:	27bd0038 	addiu	sp,sp,56
80016a60:	03e00008 	jr	ra
80016a64:	00000000 	nop

80016a68 <strtoll>:
80016a68:	27bdffc0 	addiu	sp,sp,-64
80016a6c:	ffbf0038 	sd	ra,56(sp)
80016a70:	ffbe0030 	sd	s8,48(sp)
80016a74:	ffb00028 	sd	s0,40(sp)
80016a78:	03a0f025 	move	s8,sp
80016a7c:	afc40040 	sw	a0,64(s8)
80016a80:	afc50048 	sw	a1,72(s8)
80016a84:	afc60050 	sw	a2,80(s8)
80016a88:	8f908028 	lw	s0,-32728(gp)
80016a8c:	0c005991 	jal	80016644 <__get_current_locale>
80016a90:	00000000 	nop
80016a94:	afa20024 	sw	v0,36(sp)
80016a98:	8fc70050 	lw	a3,80(s8)
80016a9c:	8fc60048 	lw	a2,72(s8)
80016aa0:	8fc50040 	lw	a1,64(s8)
80016aa4:	02002025 	move	a0,s0
80016aa8:	0c00599d 	jal	80016674 <_strtoll_l>
80016aac:	00000000 	nop
80016ab0:	03c0e825 	move	sp,s8
80016ab4:	dfbf0038 	ld	ra,56(sp)
80016ab8:	dfbe0030 	ld	s8,48(sp)
80016abc:	dfb00028 	ld	s0,40(sp)
80016ac0:	27bd0040 	addiu	sp,sp,64
80016ac4:	03e00008 	jr	ra
80016ac8:	00000000 	nop

80016acc <__locale_ctype_ptr_l>:
80016acc:	27bdfff8 	addiu	sp,sp,-8
80016ad0:	ffbe0000 	sd	s8,0(sp)
80016ad4:	03a0f025 	move	s8,sp
80016ad8:	afc40008 	sw	a0,8(s8)
80016adc:	3c028002 	lui	v0,0x8002
80016ae0:	2442af60 	addiu	v0,v0,-20640
80016ae4:	03c0e825 	move	sp,s8
80016ae8:	dfbe0000 	ld	s8,0(sp)
80016aec:	27bd0008 	addiu	sp,sp,8
80016af0:	03e00008 	jr	ra
80016af4:	00000000 	nop

80016af8 <__get_global_locale>:
80016af8:	27bdfff8 	addiu	sp,sp,-8
80016afc:	ffbe0000 	sd	s8,0(sp)
80016b00:	03a0f025 	move	s8,sp
80016b04:	3c028002 	lui	v0,0x8002
80016b08:	2442d2f8 	addiu	v0,v0,-11528
80016b0c:	03c0e825 	move	sp,s8
80016b10:	dfbe0000 	ld	s8,0(sp)
80016b14:	27bd0008 	addiu	sp,sp,8
80016b18:	03e00008 	jr	ra
80016b1c:	00000000 	nop

80016b20 <__get_current_locale>:
80016b20:	27bdffd0 	addiu	sp,sp,-48
80016b24:	ffbf0028 	sd	ra,40(sp)
80016b28:	ffbe0020 	sd	s8,32(sp)
80016b2c:	03a0f025 	move	s8,sp
80016b30:	0c005abe 	jal	80016af8 <__get_global_locale>
80016b34:	00000000 	nop
80016b38:	03c0e825 	move	sp,s8
80016b3c:	dfbf0028 	ld	ra,40(sp)
80016b40:	dfbe0020 	ld	s8,32(sp)
80016b44:	27bd0030 	addiu	sp,sp,48
80016b48:	03e00008 	jr	ra
80016b4c:	00000000 	nop

80016b50 <_strtol_l>:
80016b50:	27bdff98 	addiu	sp,sp,-104
80016b54:	ffbf0060 	sd	ra,96(sp)
80016b58:	ffbe0058 	sd	s8,88(sp)
80016b5c:	ffb60050 	sd	s6,80(sp)
80016b60:	ffb50048 	sd	s5,72(sp)
80016b64:	ffb40040 	sd	s4,64(sp)
80016b68:	ffb30038 	sd	s3,56(sp)
80016b6c:	ffb20030 	sd	s2,48(sp)
80016b70:	ffb10028 	sd	s1,40(sp)
80016b74:	ffb00020 	sd	s0,32(sp)
80016b78:	03a0f025 	move	s8,sp
80016b7c:	afc40068 	sw	a0,104(s8)
80016b80:	afc50070 	sw	a1,112(s8)
80016b84:	afc60078 	sw	a2,120(s8)
80016b88:	afc70080 	sw	a3,128(s8)
80016b8c:	8fd10070 	lw	s1,112(s8)
80016b90:	0000a825 	move	s5,zero
80016b94:	8fc20080 	lw	v0,128(s8)
80016b98:	04400009 	bltz	v0,80016bc0 <_strtol_l+0x70>
80016b9c:	00000000 	nop
80016ba0:	8fc30080 	lw	v1,128(s8)
80016ba4:	24020001 	li	v0,1
80016ba8:	10620005 	beq	v1,v0,80016bc0 <_strtol_l+0x70>
80016bac:	00000000 	nop
80016bb0:	8fc20080 	lw	v0,128(s8)
80016bb4:	28420025 	slti	v0,v0,37
80016bb8:	14400009 	bnez	v0,80016be0 <_strtol_l+0x90>
80016bbc:	00000000 	nop
80016bc0:	0c0057a3 	jal	80015e8c <__errno>
80016bc4:	00000000 	nop
80016bc8:	00401825 	move	v1,v0
80016bcc:	24020016 	li	v0,22
80016bd0:	ac620000 	sw	v0,0(v1)
80016bd4:	00001025 	move	v0,zero
80016bd8:	100000ae 	b	80016e94 <_strtol_l+0x344>
80016bdc:	00000000 	nop
80016be0:	02201025 	move	v0,s1
80016be4:	24510001 	addiu	s1,v0,1
80016be8:	90420000 	lbu	v0,0(v0)
80016bec:	00408025 	move	s0,v0
80016bf0:	8fc4008c 	lw	a0,140(s8)
80016bf4:	0c005ab3 	jal	80016acc <__locale_ctype_ptr_l>
80016bf8:	00000000 	nop
80016bfc:	00401825 	move	v1,v0
80016c00:	02001025 	move	v0,s0
80016c04:	24420001 	addiu	v0,v0,1
80016c08:	00621021 	addu	v0,v1,v0
80016c0c:	80420000 	lb	v0,0(v0)
80016c10:	304200ff 	andi	v0,v0,0xff
80016c14:	30420008 	andi	v0,v0,0x8
80016c18:	1440fff1 	bnez	v0,80016be0 <_strtol_l+0x90>
80016c1c:	00000000 	nop
80016c20:	2402002d 	li	v0,45
80016c24:	16020008 	bne	s0,v0,80016c48 <_strtol_l+0xf8>
80016c28:	00000000 	nop
80016c2c:	24150001 	li	s5,1
80016c30:	02201025 	move	v0,s1
80016c34:	24510001 	addiu	s1,v0,1
80016c38:	90420000 	lbu	v0,0(v0)
80016c3c:	00408025 	move	s0,v0
80016c40:	10000008 	b	80016c64 <_strtol_l+0x114>
80016c44:	00000000 	nop
80016c48:	2402002b 	li	v0,43
80016c4c:	16020005 	bne	s0,v0,80016c64 <_strtol_l+0x114>
80016c50:	00000000 	nop
80016c54:	02201025 	move	v0,s1
80016c58:	24510001 	addiu	s1,v0,1
80016c5c:	90420000 	lbu	v0,0(v0)
80016c60:	00408025 	move	s0,v0
80016c64:	8fc20080 	lw	v0,128(s8)
80016c68:	10400005 	beqz	v0,80016c80 <_strtol_l+0x130>
80016c6c:	00000000 	nop
80016c70:	8fc30080 	lw	v1,128(s8)
80016c74:	24020010 	li	v0,16
80016c78:	14620012 	bne	v1,v0,80016cc4 <_strtol_l+0x174>
80016c7c:	00000000 	nop
80016c80:	24020030 	li	v0,48
80016c84:	1602000f 	bne	s0,v0,80016cc4 <_strtol_l+0x174>
80016c88:	00000000 	nop
80016c8c:	92230000 	lbu	v1,0(s1)
80016c90:	24020078 	li	v0,120
80016c94:	10620005 	beq	v1,v0,80016cac <_strtol_l+0x15c>
80016c98:	00000000 	nop
80016c9c:	92230000 	lbu	v1,0(s1)
80016ca0:	24020058 	li	v0,88
80016ca4:	14620007 	bne	v1,v0,80016cc4 <_strtol_l+0x174>
80016ca8:	00000000 	nop
80016cac:	26220001 	addiu	v0,s1,1
80016cb0:	90420000 	lbu	v0,0(v0)
80016cb4:	00408025 	move	s0,v0
80016cb8:	26310002 	addiu	s1,s1,2
80016cbc:	24020010 	li	v0,16
80016cc0:	afc20080 	sw	v0,128(s8)
80016cc4:	8fc20080 	lw	v0,128(s8)
80016cc8:	14400009 	bnez	v0,80016cf0 <_strtol_l+0x1a0>
80016ccc:	00000000 	nop
80016cd0:	24020030 	li	v0,48
80016cd4:	16020004 	bne	s0,v0,80016ce8 <_strtol_l+0x198>
80016cd8:	00000000 	nop
80016cdc:	24020008 	li	v0,8
80016ce0:	10000002 	b	80016cec <_strtol_l+0x19c>
80016ce4:	00000000 	nop
80016ce8:	2402000a 	li	v0,10
80016cec:	afc20080 	sw	v0,128(s8)
80016cf0:	12a00004 	beqz	s5,80016d04 <_strtol_l+0x1b4>
80016cf4:	00000000 	nop
80016cf8:	3c028000 	lui	v0,0x8000
80016cfc:	10000003 	b	80016d0c <_strtol_l+0x1bc>
80016d00:	00000000 	nop
80016d04:	3c027fff 	lui	v0,0x7fff
80016d08:	3442ffff 	ori	v0,v0,0xffff
80016d0c:	00409825 	move	s3,v0
80016d10:	8fc20080 	lw	v0,128(s8)
80016d14:	0262001b 	divu	zero,s3,v0
80016d18:	004001f4 	teq	v0,zero,0x7
80016d1c:	00001010 	mfhi	v0
80016d20:	0040b025 	move	s6,v0
80016d24:	8fc20080 	lw	v0,128(s8)
80016d28:	0262001b 	divu	zero,s3,v0
80016d2c:	004001f4 	teq	v0,zero,0x7
80016d30:	00001010 	mfhi	v0
80016d34:	00009812 	mflo	s3
80016d38:	00009025 	move	s2,zero
80016d3c:	0000a025 	move	s4,zero
80016d40:	2a020030 	slti	v0,s0,48
80016d44:	14400007 	bnez	v0,80016d64 <_strtol_l+0x214>
80016d48:	00000000 	nop
80016d4c:	2a02003a 	slti	v0,s0,58
80016d50:	10400004 	beqz	v0,80016d64 <_strtol_l+0x214>
80016d54:	00000000 	nop
80016d58:	2610ffd0 	addiu	s0,s0,-48
80016d5c:	10000011 	b	80016da4 <_strtol_l+0x254>
80016d60:	00000000 	nop
80016d64:	2a020041 	slti	v0,s0,65
80016d68:	14400007 	bnez	v0,80016d88 <_strtol_l+0x238>
80016d6c:	00000000 	nop
80016d70:	2a02005b 	slti	v0,s0,91
80016d74:	10400004 	beqz	v0,80016d88 <_strtol_l+0x238>
80016d78:	00000000 	nop
80016d7c:	2610ffc9 	addiu	s0,s0,-55
80016d80:	10000008 	b	80016da4 <_strtol_l+0x254>
80016d84:	00000000 	nop
80016d88:	2a020061 	slti	v0,s0,97
80016d8c:	14400023 	bnez	v0,80016e1c <_strtol_l+0x2cc>
80016d90:	00000000 	nop
80016d94:	2a02007b 	slti	v0,s0,123
80016d98:	10400020 	beqz	v0,80016e1c <_strtol_l+0x2cc>
80016d9c:	00000000 	nop
80016da0:	2610ffa9 	addiu	s0,s0,-87
80016da4:	8fc20080 	lw	v0,128(s8)
80016da8:	0202102a 	slt	v0,s0,v0
80016dac:	1040001a 	beqz	v0,80016e18 <_strtol_l+0x2c8>
80016db0:	00000000 	nop
80016db4:	06800009 	bltz	s4,80016ddc <_strtol_l+0x28c>
80016db8:	00000000 	nop
80016dbc:	0272102b 	sltu	v0,s3,s2
80016dc0:	14400006 	bnez	v0,80016ddc <_strtol_l+0x28c>
80016dc4:	00000000 	nop
80016dc8:	16530007 	bne	s2,s3,80016de8 <_strtol_l+0x298>
80016dcc:	00000000 	nop
80016dd0:	02d0102a 	slt	v0,s6,s0
80016dd4:	10400004 	beqz	v0,80016de8 <_strtol_l+0x298>
80016dd8:	00000000 	nop
80016ddc:	2414ffff 	li	s4,-1
80016de0:	10000007 	b	80016e00 <_strtol_l+0x2b0>
80016de4:	00000000 	nop
80016de8:	24140001 	li	s4,1
80016dec:	8fc20080 	lw	v0,128(s8)
80016df0:	02420018 	mult	s2,v0
80016df4:	00009012 	mflo	s2
80016df8:	02001025 	move	v0,s0
80016dfc:	02429021 	addu	s2,s2,v0
80016e00:	02201025 	move	v0,s1
80016e04:	24510001 	addiu	s1,v0,1
80016e08:	90420000 	lbu	v0,0(v0)
80016e0c:	00408025 	move	s0,v0
80016e10:	1000ffcb 	b	80016d40 <_strtol_l+0x1f0>
80016e14:	00000000 	nop
80016e18:	00000000 	nop
80016e1c:	0681000e 	bgez	s4,80016e58 <_strtol_l+0x308>
80016e20:	00000000 	nop
80016e24:	12a00004 	beqz	s5,80016e38 <_strtol_l+0x2e8>
80016e28:	00000000 	nop
80016e2c:	3c028000 	lui	v0,0x8000
80016e30:	10000003 	b	80016e40 <_strtol_l+0x2f0>
80016e34:	00000000 	nop
80016e38:	3c027fff 	lui	v0,0x7fff
80016e3c:	3442ffff 	ori	v0,v0,0xffff
80016e40:	00409025 	move	s2,v0
80016e44:	8fc20068 	lw	v0,104(s8)
80016e48:	24030022 	li	v1,34
80016e4c:	ac430000 	sw	v1,0(v0)
80016e50:	10000004 	b	80016e64 <_strtol_l+0x314>
80016e54:	00000000 	nop
80016e58:	12a00002 	beqz	s5,80016e64 <_strtol_l+0x314>
80016e5c:	00000000 	nop
80016e60:	00129023 	negu	s2,s2
80016e64:	8fc20078 	lw	v0,120(s8)
80016e68:	10400009 	beqz	v0,80016e90 <_strtol_l+0x340>
80016e6c:	00000000 	nop
80016e70:	12800004 	beqz	s4,80016e84 <_strtol_l+0x334>
80016e74:	00000000 	nop
80016e78:	2622ffff 	addiu	v0,s1,-1
80016e7c:	10000002 	b	80016e88 <_strtol_l+0x338>
80016e80:	00000000 	nop
80016e84:	8fc20070 	lw	v0,112(s8)
80016e88:	8fc30078 	lw	v1,120(s8)
80016e8c:	ac620000 	sw	v0,0(v1)
80016e90:	02401025 	move	v0,s2
80016e94:	03c0e825 	move	sp,s8
80016e98:	dfbf0060 	ld	ra,96(sp)
80016e9c:	dfbe0058 	ld	s8,88(sp)
80016ea0:	dfb60050 	ld	s6,80(sp)
80016ea4:	dfb50048 	ld	s5,72(sp)
80016ea8:	dfb40040 	ld	s4,64(sp)
80016eac:	dfb30038 	ld	s3,56(sp)
80016eb0:	dfb20030 	ld	s2,48(sp)
80016eb4:	dfb10028 	ld	s1,40(sp)
80016eb8:	dfb00020 	ld	s0,32(sp)
80016ebc:	27bd0068 	addiu	sp,sp,104
80016ec0:	03e00008 	jr	ra
80016ec4:	00000000 	nop

80016ec8 <_strtol_r>:
80016ec8:	27bdffc8 	addiu	sp,sp,-56
80016ecc:	ffbf0030 	sd	ra,48(sp)
80016ed0:	ffbe0028 	sd	s8,40(sp)
80016ed4:	03a0f025 	move	s8,sp
80016ed8:	afc40038 	sw	a0,56(s8)
80016edc:	afc50040 	sw	a1,64(s8)
80016ee0:	afc60048 	sw	a2,72(s8)
80016ee4:	afc70050 	sw	a3,80(s8)
80016ee8:	0c005ac8 	jal	80016b20 <__get_current_locale>
80016eec:	00000000 	nop
80016ef0:	afa20024 	sw	v0,36(sp)
80016ef4:	8fc70050 	lw	a3,80(s8)
80016ef8:	8fc60048 	lw	a2,72(s8)
80016efc:	8fc50040 	lw	a1,64(s8)
80016f00:	8fc40038 	lw	a0,56(s8)
80016f04:	0c005ad4 	jal	80016b50 <_strtol_l>
80016f08:	00000000 	nop
80016f0c:	03c0e825 	move	sp,s8
80016f10:	dfbf0030 	ld	ra,48(sp)
80016f14:	dfbe0028 	ld	s8,40(sp)
80016f18:	27bd0038 	addiu	sp,sp,56
80016f1c:	03e00008 	jr	ra
80016f20:	00000000 	nop

80016f24 <strtol_l>:
80016f24:	27bdffc8 	addiu	sp,sp,-56
80016f28:	ffbf0030 	sd	ra,48(sp)
80016f2c:	ffbe0028 	sd	s8,40(sp)
80016f30:	03a0f025 	move	s8,sp
80016f34:	afc40038 	sw	a0,56(s8)
80016f38:	afc50040 	sw	a1,64(s8)
80016f3c:	afc60048 	sw	a2,72(s8)
80016f40:	afc70050 	sw	a3,80(s8)
80016f44:	8f838028 	lw	v1,-32728(gp)
80016f48:	8fc20050 	lw	v0,80(s8)
80016f4c:	afa20024 	sw	v0,36(sp)
80016f50:	8fc70048 	lw	a3,72(s8)
80016f54:	8fc60040 	lw	a2,64(s8)
80016f58:	8fc50038 	lw	a1,56(s8)
80016f5c:	00602025 	move	a0,v1
80016f60:	0c005ad4 	jal	80016b50 <_strtol_l>
80016f64:	00000000 	nop
80016f68:	03c0e825 	move	sp,s8
80016f6c:	dfbf0030 	ld	ra,48(sp)
80016f70:	dfbe0028 	ld	s8,40(sp)
80016f74:	27bd0038 	addiu	sp,sp,56
80016f78:	03e00008 	jr	ra
80016f7c:	00000000 	nop

80016f80 <strtol>:
80016f80:	27bdffc0 	addiu	sp,sp,-64
80016f84:	ffbf0038 	sd	ra,56(sp)
80016f88:	ffbe0030 	sd	s8,48(sp)
80016f8c:	ffb00028 	sd	s0,40(sp)
80016f90:	03a0f025 	move	s8,sp
80016f94:	afc40040 	sw	a0,64(s8)
80016f98:	afc50048 	sw	a1,72(s8)
80016f9c:	afc60050 	sw	a2,80(s8)
80016fa0:	8f908028 	lw	s0,-32728(gp)
80016fa4:	0c005ac8 	jal	80016b20 <__get_current_locale>
80016fa8:	00000000 	nop
80016fac:	afa20024 	sw	v0,36(sp)
80016fb0:	8fc70050 	lw	a3,80(s8)
80016fb4:	8fc60048 	lw	a2,72(s8)
80016fb8:	8fc50040 	lw	a1,64(s8)
80016fbc:	02002025 	move	a0,s0
80016fc0:	0c005ad4 	jal	80016b50 <_strtol_l>
80016fc4:	00000000 	nop
80016fc8:	03c0e825 	move	sp,s8
80016fcc:	dfbf0038 	ld	ra,56(sp)
80016fd0:	dfbe0030 	ld	s8,48(sp)
80016fd4:	dfb00028 	ld	s0,40(sp)
80016fd8:	27bd0040 	addiu	sp,sp,64
80016fdc:	03e00008 	jr	ra
80016fe0:	00000000 	nop

80016fe4 <__locale_ctype_ptr_l>:
80016fe4:	27bdfff8 	addiu	sp,sp,-8
80016fe8:	ffbe0000 	sd	s8,0(sp)
80016fec:	03a0f025 	move	s8,sp
80016ff0:	afc40008 	sw	a0,8(s8)
80016ff4:	3c028002 	lui	v0,0x8002
80016ff8:	2442af60 	addiu	v0,v0,-20640
80016ffc:	03c0e825 	move	sp,s8
80017000:	dfbe0000 	ld	s8,0(sp)
80017004:	27bd0008 	addiu	sp,sp,8
80017008:	03e00008 	jr	ra
8001700c:	00000000 	nop

80017010 <__get_global_locale>:
80017010:	27bdfff8 	addiu	sp,sp,-8
80017014:	ffbe0000 	sd	s8,0(sp)
80017018:	03a0f025 	move	s8,sp
8001701c:	3c028002 	lui	v0,0x8002
80017020:	2442d2f8 	addiu	v0,v0,-11528
80017024:	03c0e825 	move	sp,s8
80017028:	dfbe0000 	ld	s8,0(sp)
8001702c:	27bd0008 	addiu	sp,sp,8
80017030:	03e00008 	jr	ra
80017034:	00000000 	nop

80017038 <__get_current_locale>:
80017038:	27bdffd0 	addiu	sp,sp,-48
8001703c:	ffbf0028 	sd	ra,40(sp)
80017040:	ffbe0020 	sd	s8,32(sp)
80017044:	03a0f025 	move	s8,sp
80017048:	0c005c04 	jal	80017010 <__get_global_locale>
8001704c:	00000000 	nop
80017050:	03c0e825 	move	sp,s8
80017054:	dfbf0028 	ld	ra,40(sp)
80017058:	dfbe0020 	ld	s8,32(sp)
8001705c:	27bd0030 	addiu	sp,sp,48
80017060:	03e00008 	jr	ra
80017064:	00000000 	nop

80017068 <_strtoull_l>:
80017068:	27bdff98 	addiu	sp,sp,-104
8001706c:	ffbf0060 	sd	ra,96(sp)
80017070:	ffbe0058 	sd	s8,88(sp)
80017074:	ffb60050 	sd	s6,80(sp)
80017078:	ffb50048 	sd	s5,72(sp)
8001707c:	ffb40040 	sd	s4,64(sp)
80017080:	ffb30038 	sd	s3,56(sp)
80017084:	ffb20030 	sd	s2,48(sp)
80017088:	ffb10028 	sd	s1,40(sp)
8001708c:	ffb00020 	sd	s0,32(sp)
80017090:	03a0f025 	move	s8,sp
80017094:	afc40068 	sw	a0,104(s8)
80017098:	afc50070 	sw	a1,112(s8)
8001709c:	afc60078 	sw	a2,120(s8)
800170a0:	afc70080 	sw	a3,128(s8)
800170a4:	8fd10070 	lw	s1,112(s8)
800170a8:	0000a025 	move	s4,zero
800170ac:	02201025 	move	v0,s1
800170b0:	24510001 	addiu	s1,v0,1
800170b4:	90420000 	lbu	v0,0(v0)
800170b8:	00408025 	move	s0,v0
800170bc:	8fc4008c 	lw	a0,140(s8)
800170c0:	0c005bf9 	jal	80016fe4 <__locale_ctype_ptr_l>
800170c4:	00000000 	nop
800170c8:	00401825 	move	v1,v0
800170cc:	02001025 	move	v0,s0
800170d0:	24420001 	addiu	v0,v0,1
800170d4:	00621021 	addu	v0,v1,v0
800170d8:	80420000 	lb	v0,0(v0)
800170dc:	304200ff 	andi	v0,v0,0xff
800170e0:	30420008 	andi	v0,v0,0x8
800170e4:	1440fff1 	bnez	v0,800170ac <_strtoull_l+0x44>
800170e8:	00000000 	nop
800170ec:	2402002d 	li	v0,45
800170f0:	16020008 	bne	s0,v0,80017114 <_strtoull_l+0xac>
800170f4:	00000000 	nop
800170f8:	24140001 	li	s4,1
800170fc:	02201025 	move	v0,s1
80017100:	24510001 	addiu	s1,v0,1
80017104:	90420000 	lbu	v0,0(v0)
80017108:	00408025 	move	s0,v0
8001710c:	10000008 	b	80017130 <_strtoull_l+0xc8>
80017110:	00000000 	nop
80017114:	2402002b 	li	v0,43
80017118:	16020005 	bne	s0,v0,80017130 <_strtoull_l+0xc8>
8001711c:	00000000 	nop
80017120:	02201025 	move	v0,s1
80017124:	24510001 	addiu	s1,v0,1
80017128:	90420000 	lbu	v0,0(v0)
8001712c:	00408025 	move	s0,v0
80017130:	8fc20080 	lw	v0,128(s8)
80017134:	10400005 	beqz	v0,8001714c <_strtoull_l+0xe4>
80017138:	00000000 	nop
8001713c:	8fc30080 	lw	v1,128(s8)
80017140:	24020010 	li	v0,16
80017144:	14620012 	bne	v1,v0,80017190 <_strtoull_l+0x128>
80017148:	00000000 	nop
8001714c:	24020030 	li	v0,48
80017150:	1602000f 	bne	s0,v0,80017190 <_strtoull_l+0x128>
80017154:	00000000 	nop
80017158:	92230000 	lbu	v1,0(s1)
8001715c:	24020078 	li	v0,120
80017160:	10620005 	beq	v1,v0,80017178 <_strtoull_l+0x110>
80017164:	00000000 	nop
80017168:	92230000 	lbu	v1,0(s1)
8001716c:	24020058 	li	v0,88
80017170:	14620007 	bne	v1,v0,80017190 <_strtoull_l+0x128>
80017174:	00000000 	nop
80017178:	26220001 	addiu	v0,s1,1
8001717c:	90420000 	lbu	v0,0(v0)
80017180:	00408025 	move	s0,v0
80017184:	26310002 	addiu	s1,s1,2
80017188:	24020010 	li	v0,16
8001718c:	afc20080 	sw	v0,128(s8)
80017190:	8fc20080 	lw	v0,128(s8)
80017194:	14400009 	bnez	v0,800171bc <_strtoull_l+0x154>
80017198:	00000000 	nop
8001719c:	24020030 	li	v0,48
800171a0:	16020004 	bne	s0,v0,800171b4 <_strtoull_l+0x14c>
800171a4:	00000000 	nop
800171a8:	24020008 	li	v0,8
800171ac:	10000002 	b	800171b8 <_strtoull_l+0x150>
800171b0:	00000000 	nop
800171b4:	2402000a 	li	v0,10
800171b8:	afc20080 	sw	v0,128(s8)
800171bc:	8fc20080 	lw	v0,128(s8)
800171c0:	2403ffff 	li	v1,-1
800171c4:	0062001f 	ddivu	zero,v1,v0
800171c8:	004001f4 	teq	v0,zero,0x7
800171cc:	00001010 	mfhi	v0
800171d0:	0000a812 	mflo	s5
800171d4:	8fc20080 	lw	v0,128(s8)
800171d8:	2403ffff 	li	v1,-1
800171dc:	0062001f 	ddivu	zero,v1,v0
800171e0:	004001f4 	teq	v0,zero,0x7
800171e4:	00001010 	mfhi	v0
800171e8:	0002b000 	sll	s6,v0,0x0
800171ec:	00009025 	move	s2,zero
800171f0:	00009825 	move	s3,zero
800171f4:	2a020030 	slti	v0,s0,48
800171f8:	14400007 	bnez	v0,80017218 <_strtoull_l+0x1b0>
800171fc:	00000000 	nop
80017200:	2a02003a 	slti	v0,s0,58
80017204:	10400004 	beqz	v0,80017218 <_strtoull_l+0x1b0>
80017208:	00000000 	nop
8001720c:	2610ffd0 	addiu	s0,s0,-48
80017210:	10000011 	b	80017258 <_strtoull_l+0x1f0>
80017214:	00000000 	nop
80017218:	2a020041 	slti	v0,s0,65
8001721c:	14400007 	bnez	v0,8001723c <_strtoull_l+0x1d4>
80017220:	00000000 	nop
80017224:	2a02005b 	slti	v0,s0,91
80017228:	10400004 	beqz	v0,8001723c <_strtoull_l+0x1d4>
8001722c:	00000000 	nop
80017230:	2610ffc9 	addiu	s0,s0,-55
80017234:	10000008 	b	80017258 <_strtoull_l+0x1f0>
80017238:	00000000 	nop
8001723c:	2a020061 	slti	v0,s0,97
80017240:	14400023 	bnez	v0,800172d0 <_strtoull_l+0x268>
80017244:	00000000 	nop
80017248:	2a02007b 	slti	v0,s0,123
8001724c:	10400020 	beqz	v0,800172d0 <_strtoull_l+0x268>
80017250:	00000000 	nop
80017254:	2610ffa9 	addiu	s0,s0,-87
80017258:	8fc20080 	lw	v0,128(s8)
8001725c:	0202102a 	slt	v0,s0,v0
80017260:	1040001a 	beqz	v0,800172cc <_strtoull_l+0x264>
80017264:	00000000 	nop
80017268:	06600009 	bltz	s3,80017290 <_strtoull_l+0x228>
8001726c:	00000000 	nop
80017270:	02b2102b 	sltu	v0,s5,s2
80017274:	14400006 	bnez	v0,80017290 <_strtoull_l+0x228>
80017278:	00000000 	nop
8001727c:	16550007 	bne	s2,s5,8001729c <_strtoull_l+0x234>
80017280:	00000000 	nop
80017284:	02d0102a 	slt	v0,s6,s0
80017288:	10400004 	beqz	v0,8001729c <_strtoull_l+0x234>
8001728c:	00000000 	nop
80017290:	2413ffff 	li	s3,-1
80017294:	10000007 	b	800172b4 <_strtoull_l+0x24c>
80017298:	00000000 	nop
8001729c:	24130001 	li	s3,1
800172a0:	8fc20080 	lw	v0,128(s8)
800172a4:	0242001c 	dmult	s2,v0
800172a8:	00009012 	mflo	s2
800172ac:	02001025 	move	v0,s0
800172b0:	0242902d 	daddu	s2,s2,v0
800172b4:	02201025 	move	v0,s1
800172b8:	24510001 	addiu	s1,v0,1
800172bc:	90420000 	lbu	v0,0(v0)
800172c0:	00408025 	move	s0,v0
800172c4:	1000ffcb 	b	800171f4 <_strtoull_l+0x18c>
800172c8:	00000000 	nop
800172cc:	00000000 	nop
800172d0:	06610007 	bgez	s3,800172f0 <_strtoull_l+0x288>
800172d4:	00000000 	nop
800172d8:	2412ffff 	li	s2,-1
800172dc:	8fc20068 	lw	v0,104(s8)
800172e0:	24030022 	li	v1,34
800172e4:	ac430000 	sw	v1,0(v0)
800172e8:	10000004 	b	800172fc <_strtoull_l+0x294>
800172ec:	00000000 	nop
800172f0:	12800002 	beqz	s4,800172fc <_strtoull_l+0x294>
800172f4:	00000000 	nop
800172f8:	0012902f 	dnegu	s2,s2
800172fc:	8fc20078 	lw	v0,120(s8)
80017300:	10400009 	beqz	v0,80017328 <_strtoull_l+0x2c0>
80017304:	00000000 	nop
80017308:	12600004 	beqz	s3,8001731c <_strtoull_l+0x2b4>
8001730c:	00000000 	nop
80017310:	2622ffff 	addiu	v0,s1,-1
80017314:	10000002 	b	80017320 <_strtoull_l+0x2b8>
80017318:	00000000 	nop
8001731c:	8fc20070 	lw	v0,112(s8)
80017320:	8fc30078 	lw	v1,120(s8)
80017324:	ac620000 	sw	v0,0(v1)
80017328:	02401025 	move	v0,s2
8001732c:	03c0e825 	move	sp,s8
80017330:	dfbf0060 	ld	ra,96(sp)
80017334:	dfbe0058 	ld	s8,88(sp)
80017338:	dfb60050 	ld	s6,80(sp)
8001733c:	dfb50048 	ld	s5,72(sp)
80017340:	dfb40040 	ld	s4,64(sp)
80017344:	dfb30038 	ld	s3,56(sp)
80017348:	dfb20030 	ld	s2,48(sp)
8001734c:	dfb10028 	ld	s1,40(sp)
80017350:	dfb00020 	ld	s0,32(sp)
80017354:	27bd0068 	addiu	sp,sp,104
80017358:	03e00008 	jr	ra
8001735c:	00000000 	nop

80017360 <_strtoull_r>:
80017360:	27bdffc8 	addiu	sp,sp,-56
80017364:	ffbf0030 	sd	ra,48(sp)
80017368:	ffbe0028 	sd	s8,40(sp)
8001736c:	03a0f025 	move	s8,sp
80017370:	afc40038 	sw	a0,56(s8)
80017374:	afc50040 	sw	a1,64(s8)
80017378:	afc60048 	sw	a2,72(s8)
8001737c:	afc70050 	sw	a3,80(s8)
80017380:	0c005c0e 	jal	80017038 <__get_current_locale>
80017384:	00000000 	nop
80017388:	afa20024 	sw	v0,36(sp)
8001738c:	8fc70050 	lw	a3,80(s8)
80017390:	8fc60048 	lw	a2,72(s8)
80017394:	8fc50040 	lw	a1,64(s8)
80017398:	8fc40038 	lw	a0,56(s8)
8001739c:	0c005c1a 	jal	80017068 <_strtoull_l>
800173a0:	00000000 	nop
800173a4:	03c0e825 	move	sp,s8
800173a8:	dfbf0030 	ld	ra,48(sp)
800173ac:	dfbe0028 	ld	s8,40(sp)
800173b0:	27bd0038 	addiu	sp,sp,56
800173b4:	03e00008 	jr	ra
800173b8:	00000000 	nop

800173bc <strtoull_l>:
800173bc:	27bdffc8 	addiu	sp,sp,-56
800173c0:	ffbf0030 	sd	ra,48(sp)
800173c4:	ffbe0028 	sd	s8,40(sp)
800173c8:	03a0f025 	move	s8,sp
800173cc:	afc40038 	sw	a0,56(s8)
800173d0:	afc50040 	sw	a1,64(s8)
800173d4:	afc60048 	sw	a2,72(s8)
800173d8:	afc70050 	sw	a3,80(s8)
800173dc:	8f838028 	lw	v1,-32728(gp)
800173e0:	8fc20050 	lw	v0,80(s8)
800173e4:	afa20024 	sw	v0,36(sp)
800173e8:	8fc70048 	lw	a3,72(s8)
800173ec:	8fc60040 	lw	a2,64(s8)
800173f0:	8fc50038 	lw	a1,56(s8)
800173f4:	00602025 	move	a0,v1
800173f8:	0c005c1a 	jal	80017068 <_strtoull_l>
800173fc:	00000000 	nop
80017400:	03c0e825 	move	sp,s8
80017404:	dfbf0030 	ld	ra,48(sp)
80017408:	dfbe0028 	ld	s8,40(sp)
8001740c:	27bd0038 	addiu	sp,sp,56
80017410:	03e00008 	jr	ra
80017414:	00000000 	nop

80017418 <strtoull>:
80017418:	27bdffc0 	addiu	sp,sp,-64
8001741c:	ffbf0038 	sd	ra,56(sp)
80017420:	ffbe0030 	sd	s8,48(sp)
80017424:	ffb00028 	sd	s0,40(sp)
80017428:	03a0f025 	move	s8,sp
8001742c:	afc40040 	sw	a0,64(s8)
80017430:	afc50048 	sw	a1,72(s8)
80017434:	afc60050 	sw	a2,80(s8)
80017438:	8f908028 	lw	s0,-32728(gp)
8001743c:	0c005c0e 	jal	80017038 <__get_current_locale>
80017440:	00000000 	nop
80017444:	afa20024 	sw	v0,36(sp)
80017448:	8fc70050 	lw	a3,80(s8)
8001744c:	8fc60048 	lw	a2,72(s8)
80017450:	8fc50040 	lw	a1,64(s8)
80017454:	02002025 	move	a0,s0
80017458:	0c005c1a 	jal	80017068 <_strtoull_l>
8001745c:	00000000 	nop
80017460:	03c0e825 	move	sp,s8
80017464:	dfbf0038 	ld	ra,56(sp)
80017468:	dfbe0030 	ld	s8,48(sp)
8001746c:	dfb00028 	ld	s0,40(sp)
80017470:	27bd0040 	addiu	sp,sp,64
80017474:	03e00008 	jr	ra
80017478:	00000000 	nop

8001747c <__get_global_locale>:
8001747c:	27bdfff8 	addiu	sp,sp,-8
80017480:	ffbe0000 	sd	s8,0(sp)
80017484:	03a0f025 	move	s8,sp
80017488:	3c028002 	lui	v0,0x8002
8001748c:	2442d2f8 	addiu	v0,v0,-11528
80017490:	03c0e825 	move	sp,s8
80017494:	dfbe0000 	ld	s8,0(sp)
80017498:	27bd0008 	addiu	sp,sp,8
8001749c:	03e00008 	jr	ra
800174a0:	00000000 	nop

800174a4 <__get_current_locale>:
800174a4:	27bdffd0 	addiu	sp,sp,-48
800174a8:	ffbf0028 	sd	ra,40(sp)
800174ac:	ffbe0020 	sd	s8,32(sp)
800174b0:	03a0f025 	move	s8,sp
800174b4:	0c005d1f 	jal	8001747c <__get_global_locale>
800174b8:	00000000 	nop
800174bc:	03c0e825 	move	sp,s8
800174c0:	dfbf0028 	ld	ra,40(sp)
800174c4:	dfbe0020 	ld	s8,32(sp)
800174c8:	27bd0030 	addiu	sp,sp,48
800174cc:	03e00008 	jr	ra
800174d0:	00000000 	nop

800174d4 <_wcrtomb_r>:
800174d4:	27bdffb8 	addiu	sp,sp,-72
800174d8:	ffbf0040 	sd	ra,64(sp)
800174dc:	ffbe0038 	sd	s8,56(sp)
800174e0:	03a0f025 	move	s8,sp
800174e4:	afc40048 	sw	a0,72(s8)
800174e8:	afc50050 	sw	a1,80(s8)
800174ec:	afc60058 	sw	a2,88(s8)
800174f0:	afc70060 	sw	a3,96(s8)
800174f4:	afc00020 	sw	zero,32(s8)
800174f8:	8fc20050 	lw	v0,80(s8)
800174fc:	1440000e 	bnez	v0,80017538 <_wcrtomb_r+0x64>
80017500:	00000000 	nop
80017504:	0c005d29 	jal	800174a4 <__get_current_locale>
80017508:	00000000 	nop
8001750c:	8c4200e0 	lw	v0,224(v0)
80017510:	27c30028 	addiu	v1,s8,40
80017514:	8fc70060 	lw	a3,96(s8)
80017518:	00003025 	move	a2,zero
8001751c:	00602825 	move	a1,v1
80017520:	8fc40048 	lw	a0,72(s8)
80017524:	0040f809 	jalr	v0
80017528:	00000000 	nop
8001752c:	afc20020 	sw	v0,32(s8)
80017530:	1000000b 	b	80017560 <_wcrtomb_r+0x8c>
80017534:	00000000 	nop
80017538:	0c005d29 	jal	800174a4 <__get_current_locale>
8001753c:	00000000 	nop
80017540:	8c4200e0 	lw	v0,224(v0)
80017544:	8fc70060 	lw	a3,96(s8)
80017548:	8fc60058 	lw	a2,88(s8)
8001754c:	8fc50050 	lw	a1,80(s8)
80017550:	8fc40048 	lw	a0,72(s8)
80017554:	0040f809 	jalr	v0
80017558:	00000000 	nop
8001755c:	afc20020 	sw	v0,32(s8)
80017560:	8fc30020 	lw	v1,32(s8)
80017564:	2402ffff 	li	v0,-1
80017568:	14620009 	bne	v1,v0,80017590 <_wcrtomb_r+0xbc>
8001756c:	00000000 	nop
80017570:	8fc20060 	lw	v0,96(s8)
80017574:	ac400000 	sw	zero,0(v0)
80017578:	8fc20048 	lw	v0,72(s8)
8001757c:	2403008a 	li	v1,138
80017580:	ac430000 	sw	v1,0(v0)
80017584:	2402ffff 	li	v0,-1
80017588:	10000002 	b	80017594 <_wcrtomb_r+0xc0>
8001758c:	00000000 	nop
80017590:	8fc20020 	lw	v0,32(s8)
80017594:	03c0e825 	move	sp,s8
80017598:	dfbf0040 	ld	ra,64(sp)
8001759c:	dfbe0038 	ld	s8,56(sp)
800175a0:	27bd0048 	addiu	sp,sp,72
800175a4:	03e00008 	jr	ra
800175a8:	00000000 	nop

800175ac <wcrtomb>:
800175ac:	27bdffb8 	addiu	sp,sp,-72
800175b0:	ffbf0040 	sd	ra,64(sp)
800175b4:	ffbe0038 	sd	s8,56(sp)
800175b8:	03a0f025 	move	s8,sp
800175bc:	afc40048 	sw	a0,72(s8)
800175c0:	afc50050 	sw	a1,80(s8)
800175c4:	afc60058 	sw	a2,88(s8)
800175c8:	afc00020 	sw	zero,32(s8)
800175cc:	8f828028 	lw	v0,-32728(gp)
800175d0:	afc20024 	sw	v0,36(s8)
800175d4:	8fc20048 	lw	v0,72(s8)
800175d8:	1440000e 	bnez	v0,80017614 <wcrtomb+0x68>
800175dc:	00000000 	nop
800175e0:	0c005d29 	jal	800174a4 <__get_current_locale>
800175e4:	00000000 	nop
800175e8:	8c4200e0 	lw	v0,224(v0)
800175ec:	27c30028 	addiu	v1,s8,40
800175f0:	8fc70058 	lw	a3,88(s8)
800175f4:	00003025 	move	a2,zero
800175f8:	00602825 	move	a1,v1
800175fc:	8fc40024 	lw	a0,36(s8)
80017600:	0040f809 	jalr	v0
80017604:	00000000 	nop
80017608:	afc20020 	sw	v0,32(s8)
8001760c:	1000000b 	b	8001763c <wcrtomb+0x90>
80017610:	00000000 	nop
80017614:	0c005d29 	jal	800174a4 <__get_current_locale>
80017618:	00000000 	nop
8001761c:	8c4200e0 	lw	v0,224(v0)
80017620:	8fc70058 	lw	a3,88(s8)
80017624:	8fc60050 	lw	a2,80(s8)
80017628:	8fc50048 	lw	a1,72(s8)
8001762c:	8fc40024 	lw	a0,36(s8)
80017630:	0040f809 	jalr	v0
80017634:	00000000 	nop
80017638:	afc20020 	sw	v0,32(s8)
8001763c:	8fc30020 	lw	v1,32(s8)
80017640:	2402ffff 	li	v0,-1
80017644:	14620009 	bne	v1,v0,8001766c <wcrtomb+0xc0>
80017648:	00000000 	nop
8001764c:	8fc20058 	lw	v0,88(s8)
80017650:	ac400000 	sw	zero,0(v0)
80017654:	8fc20024 	lw	v0,36(s8)
80017658:	2403008a 	li	v1,138
8001765c:	ac430000 	sw	v1,0(v0)
80017660:	2402ffff 	li	v0,-1
80017664:	10000002 	b	80017670 <wcrtomb+0xc4>
80017668:	00000000 	nop
8001766c:	8fc20020 	lw	v0,32(s8)
80017670:	03c0e825 	move	sp,s8
80017674:	dfbf0040 	ld	ra,64(sp)
80017678:	dfbe0038 	ld	s8,56(sp)
8001767c:	27bd0048 	addiu	sp,sp,72
80017680:	03e00008 	jr	ra
80017684:	00000000 	nop

80017688 <iswspace_l>:
80017688:	27bdfff8 	addiu	sp,sp,-8
8001768c:	ffbe0000 	sd	s8,0(sp)
80017690:	03a0f025 	move	s8,sp
80017694:	afc40008 	sw	a0,8(s8)
80017698:	afc50010 	sw	a1,16(s8)
8001769c:	8fc20008 	lw	v0,8(s8)
800176a0:	2c420100 	sltiu	v0,v0,256
800176a4:	1040000b 	beqz	v0,800176d4 <iswspace_l+0x4c>
800176a8:	00000000 	nop
800176ac:	8fc20008 	lw	v0,8(s8)
800176b0:	24430001 	addiu	v1,v0,1
800176b4:	3c028002 	lui	v0,0x8002
800176b8:	2442af60 	addiu	v0,v0,-20640
800176bc:	00621021 	addu	v0,v1,v0
800176c0:	80420000 	lb	v0,0(v0)
800176c4:	304200ff 	andi	v0,v0,0xff
800176c8:	30420008 	andi	v0,v0,0x8
800176cc:	10000002 	b	800176d8 <iswspace_l+0x50>
800176d0:	00000000 	nop
800176d4:	00001025 	move	v0,zero
800176d8:	03c0e825 	move	sp,s8
800176dc:	dfbe0000 	ld	s8,0(sp)
800176e0:	27bd0008 	addiu	sp,sp,8
800176e4:	03e00008 	jr	ra
800176e8:	00000000 	nop
800176ec:	00000000 	nop

800176f0 <__MI_handler>:
800176f0:	27bdffc8 	addiu	sp,sp,-56
800176f4:	3c02a430 	lui	v0,0xa430
800176f8:	ffb10028 	sd	s1,40(sp)
800176fc:	8c510008 	lw	s1,8(v0)
80017700:	8c42000c 	lw	v0,12(v0)
80017704:	ffbf0030 	sd	ra,48(sp)
80017708:	02228824 	and	s1,s1,v0
8001770c:	32220001 	andi	v0,s1,0x1
80017710:	14400015 	bnez	v0,80017768 <__MI_handler+0x78>
80017714:	ffb00020 	sd	s0,32(sp)
80017718:	32220002 	andi	v0,s1,0x2
8001771c:	54400025 	bnezl	v0,800177b4 <__MI_handler+0xc4>
80017720:	8f908350 	lw	s0,-31920(gp)
80017724:	32220004 	andi	v0,s1,0x4
80017728:	54400032 	bnezl	v0,800177f4 <__MI_handler+0x104>
8001772c:	8f908364 	lw	s0,-31900(gp)
80017730:	32220008 	andi	v0,s1,0x8
80017734:	1440003f 	bnez	v0,80017834 <__MI_handler+0x144>
80017738:	3c02a440 	lui	v0,0xa440
8001773c:	32220010 	andi	v0,s1,0x10
80017740:	1440004e 	bnez	v0,8001787c <__MI_handler+0x18c>
80017744:	8f90835c 	lw	s0,-31908(gp)
80017748:	32310020 	andi	s1,s1,0x20
8001774c:	1620005d 	bnez	s1,800178c4 <__MI_handler+0x1d4>
80017750:	8f908358 	lw	s0,-31912(gp)
80017754:	dfbf0030 	ld	ra,48(sp)
80017758:	dfb10028 	ld	s1,40(sp)
8001775c:	dfb00020 	ld	s0,32(sp)
80017760:	03e00008 	jr	ra
80017764:	27bd0038 	addiu	sp,sp,56
80017768:	8f90834c 	lw	s0,-31924(gp)
8001776c:	3c02a404 	lui	v0,0xa404
80017770:	24030008 	li	v1,8
80017774:	ac430010 	sw	v1,16(v0)
80017778:	1200ffe8 	beqz	s0,8001771c <__MI_handler+0x2c>
8001777c:	32220002 	andi	v0,s1,0x2
80017780:	8e020000 	lw	v0,0(s0)
80017784:	00000000 	nop
80017788:	50400004 	beqzl	v0,8001779c <__MI_handler+0xac>
8001778c:	8e100004 	lw	s0,4(s0)
80017790:	0040f809 	jalr	v0
80017794:	00000000 	nop
80017798:	8e100004 	lw	s0,4(s0)
8001779c:	5600fffa 	bnezl	s0,80017788 <__MI_handler+0x98>
800177a0:	8e020000 	lw	v0,0(s0)
800177a4:	32220002 	andi	v0,s1,0x2
800177a8:	1040ffdf 	beqz	v0,80017728 <__MI_handler+0x38>
800177ac:	32220004 	andi	v0,s1,0x4
800177b0:	8f908350 	lw	s0,-31920(gp)
800177b4:	3c02a480 	lui	v0,0xa480
800177b8:	ac400018 	sw	zero,24(v0)
800177bc:	1200ffda 	beqz	s0,80017728 <__MI_handler+0x38>
800177c0:	32220004 	andi	v0,s1,0x4
800177c4:	8e020000 	lw	v0,0(s0)
800177c8:	50400004 	beqzl	v0,800177dc <__MI_handler+0xec>
800177cc:	8e100004 	lw	s0,4(s0)
800177d0:	0040f809 	jalr	v0
800177d4:	00000000 	nop
800177d8:	8e100004 	lw	s0,4(s0)
800177dc:	5600fffa 	bnezl	s0,800177c8 <__MI_handler+0xd8>
800177e0:	8e020000 	lw	v0,0(s0)
800177e4:	32220004 	andi	v0,s1,0x4
800177e8:	1040ffd2 	beqz	v0,80017734 <__MI_handler+0x44>
800177ec:	32220008 	andi	v0,s1,0x8
800177f0:	8f908364 	lw	s0,-31900(gp)
800177f4:	3c02a450 	lui	v0,0xa450
800177f8:	ac40000c 	sw	zero,12(v0)
800177fc:	1200ffcd 	beqz	s0,80017734 <__MI_handler+0x44>
80017800:	32220008 	andi	v0,s1,0x8
80017804:	8e020000 	lw	v0,0(s0)
80017808:	50400004 	beqzl	v0,8001781c <__MI_handler+0x12c>
8001780c:	8e100004 	lw	s0,4(s0)
80017810:	0040f809 	jalr	v0
80017814:	00000000 	nop
80017818:	8e100004 	lw	s0,4(s0)
8001781c:	5600fffa 	bnezl	s0,80017808 <__MI_handler+0x118>
80017820:	8e020000 	lw	v0,0(s0)
80017824:	32220008 	andi	v0,s1,0x8
80017828:	1040ffc5 	beqz	v0,80017740 <__MI_handler+0x50>
8001782c:	32220010 	andi	v0,s1,0x10
80017830:	3c02a440 	lui	v0,0xa440
80017834:	8c430010 	lw	v1,16(v0)
80017838:	8f908360 	lw	s0,-31904(gp)
8001783c:	ac430010 	sw	v1,16(v0)
80017840:	1200ffbf 	beqz	s0,80017740 <__MI_handler+0x50>
80017844:	32220010 	andi	v0,s1,0x10
80017848:	8e020000 	lw	v0,0(s0)
8001784c:	00000000 	nop
80017850:	50400004 	beqzl	v0,80017864 <__MI_handler+0x174>
80017854:	8e100004 	lw	s0,4(s0)
80017858:	0040f809 	jalr	v0
8001785c:	00000000 	nop
80017860:	8e100004 	lw	s0,4(s0)
80017864:	5600fffa 	bnezl	s0,80017850 <__MI_handler+0x160>
80017868:	8e020000 	lw	v0,0(s0)
8001786c:	32220010 	andi	v0,s1,0x10
80017870:	5040ffb6 	beqzl	v0,8001774c <__MI_handler+0x5c>
80017874:	32310020 	andi	s1,s1,0x20
80017878:	8f90835c 	lw	s0,-31908(gp)
8001787c:	3c02a460 	lui	v0,0xa460
80017880:	24030002 	li	v1,2
80017884:	ac430010 	sw	v1,16(v0)
80017888:	5200ffb0 	beqzl	s0,8001774c <__MI_handler+0x5c>
8001788c:	32310020 	andi	s1,s1,0x20
80017890:	8e020000 	lw	v0,0(s0)
80017894:	00000000 	nop
80017898:	50400004 	beqzl	v0,800178ac <__MI_handler+0x1bc>
8001789c:	8e100004 	lw	s0,4(s0)
800178a0:	0040f809 	jalr	v0
800178a4:	00000000 	nop
800178a8:	8e100004 	lw	s0,4(s0)
800178ac:	5600fffa 	bnezl	s0,80017898 <__MI_handler+0x1a8>
800178b0:	8e020000 	lw	v0,0(s0)
800178b4:	32310020 	andi	s1,s1,0x20
800178b8:	1220ffa7 	beqz	s1,80017758 <__MI_handler+0x68>
800178bc:	dfbf0030 	ld	ra,48(sp)
800178c0:	8f908358 	lw	s0,-31912(gp)
800178c4:	3c02a430 	lui	v0,0xa430
800178c8:	24030800 	li	v1,2048
800178cc:	ac430000 	sw	v1,0(v0)
800178d0:	1200ffa1 	beqz	s0,80017758 <__MI_handler+0x68>
800178d4:	dfbf0030 	ld	ra,48(sp)
800178d8:	8e020000 	lw	v0,0(s0)
800178dc:	00000000 	nop
800178e0:	50400004 	beqzl	v0,800178f4 <__MI_handler+0x204>
800178e4:	8e100004 	lw	s0,4(s0)
800178e8:	0040f809 	jalr	v0
800178ec:	00000000 	nop
800178f0:	8e100004 	lw	s0,4(s0)
800178f4:	5600fffa 	bnezl	s0,800178e0 <__MI_handler+0x1f0>
800178f8:	8e020000 	lw	v0,0(s0)
800178fc:	dfbf0030 	ld	ra,48(sp)
80017900:	dfb10028 	ld	s1,40(sp)
80017904:	dfb00020 	ld	s0,32(sp)
80017908:	03e00008 	jr	ra
8001790c:	27bd0038 	addiu	sp,sp,56

80017910 <__TI_handler>:
80017910:	27bdffd0 	addiu	sp,sp,-48
80017914:	ffb00020 	sd	s0,32(sp)
80017918:	8f908354 	lw	s0,-31916(gp)
8001791c:	12000009 	beqz	s0,80017944 <__TI_handler+0x34>
80017920:	ffbf0028 	sd	ra,40(sp)
80017924:	8e020000 	lw	v0,0(s0)
80017928:	50400004 	beqzl	v0,8001793c <__TI_handler+0x2c>
8001792c:	8e100004 	lw	s0,4(s0)
80017930:	0040f809 	jalr	v0
80017934:	00000000 	nop
80017938:	8e100004 	lw	s0,4(s0)
8001793c:	5600fffa 	bnezl	s0,80017928 <__TI_handler+0x18>
80017940:	8e020000 	lw	v0,0(s0)
80017944:	dfbf0028 	ld	ra,40(sp)
80017948:	dfb00020 	ld	s0,32(sp)
8001794c:	03e00008 	jr	ra
80017950:	27bd0030 	addiu	sp,sp,48
80017954:	00000000 	nop

80017958 <register_VI_handler>:
80017958:	27bdffc8 	addiu	sp,sp,-56
8001795c:	ffb00020 	sd	s0,32(sp)
80017960:	00808025 	move	s0,a0
80017964:	24040008 	li	a0,8
80017968:	ffb10028 	sd	s1,40(sp)
8001796c:	ffbf0030 	sd	ra,48(sp)
80017970:	0c001169 	jal	800045a4 <malloc>
80017974:	8f918360 	lw	s1,-31904(gp)
80017978:	10400003 	beqz	v0,80017988 <register_VI_handler+0x30>
8001797c:	af828360 	sw	v0,-31904(gp)
80017980:	ac510004 	sw	s1,4(v0)
80017984:	ac500000 	sw	s0,0(v0)
80017988:	dfbf0030 	ld	ra,48(sp)
8001798c:	dfb10028 	ld	s1,40(sp)
80017990:	dfb00020 	ld	s0,32(sp)
80017994:	03e00008 	jr	ra
80017998:	27bd0038 	addiu	sp,sp,56
8001799c:	00000000 	nop

800179a0 <unregister_VI_handler>:
800179a0:	8f828360 	lw	v0,-31904(gp)
800179a4:	1040000f 	beqz	v0,800179e4 <unregister_VI_handler+0x44>
800179a8:	00003025 	move	a2,zero
800179ac:	10000006 	b	800179c8 <unregister_VI_handler+0x28>
800179b0:	8c450000 	lw	a1,0(v0)
800179b4:	00000000 	nop
800179b8:	1060000a 	beqz	v1,800179e4 <unregister_VI_handler+0x44>
800179bc:	00403025 	move	a2,v0
800179c0:	00601025 	move	v0,v1
800179c4:	8c450000 	lw	a1,0(v0)
800179c8:	1485fffb 	bne	a0,a1,800179b8 <unregister_VI_handler+0x18>
800179cc:	8c430004 	lw	v1,4(v0)
800179d0:	10c00007 	beqz	a2,800179f0 <unregister_VI_handler+0x50>
800179d4:	00402025 	move	a0,v0
800179d8:	acc30004 	sw	v1,4(a2)
800179dc:	08001179 	j	800045e4 <free>
800179e0:	00402025 	move	a0,v0
800179e4:	03e00008 	jr	ra
800179e8:	00000000 	nop
800179ec:	00000000 	nop
800179f0:	08001179 	j	800045e4 <free>
800179f4:	af838360 	sw	v1,-31904(gp)

800179f8 <register_TI_handler>:
800179f8:	27bdffc8 	addiu	sp,sp,-56
800179fc:	ffb00020 	sd	s0,32(sp)
80017a00:	00808025 	move	s0,a0
80017a04:	24040008 	li	a0,8
80017a08:	ffb10028 	sd	s1,40(sp)
80017a0c:	ffbf0030 	sd	ra,48(sp)
80017a10:	0c001169 	jal	800045a4 <malloc>
80017a14:	8f918354 	lw	s1,-31916(gp)
80017a18:	10400003 	beqz	v0,80017a28 <register_TI_handler+0x30>
80017a1c:	af828354 	sw	v0,-31916(gp)
80017a20:	ac510004 	sw	s1,4(v0)
80017a24:	ac500000 	sw	s0,0(v0)
80017a28:	dfbf0030 	ld	ra,48(sp)
80017a2c:	dfb10028 	ld	s1,40(sp)
80017a30:	dfb00020 	ld	s0,32(sp)
80017a34:	03e00008 	jr	ra
80017a38:	27bd0038 	addiu	sp,sp,56
80017a3c:	00000000 	nop

80017a40 <set_VI_interrupt>:
80017a40:	10800007 	beqz	a0,80017a60 <set_VI_interrupt+0x20>
80017a44:	3c02a430 	lui	v0,0xa430
80017a48:	24030080 	li	v1,128
80017a4c:	ac43000c 	sw	v1,12(v0)
80017a50:	3c02a440 	lui	v0,0xa440
80017a54:	ac45000c 	sw	a1,12(v0)
80017a58:	03e00008 	jr	ra
80017a5c:	00000000 	nop
80017a60:	24030040 	li	v1,64
80017a64:	ac43000c 	sw	v1,12(v0)
80017a68:	03e00008 	jr	ra
80017a6c:	00000000 	nop

80017a70 <init_interrupts>:
80017a70:	8f8280ac 	lw	v0,-32596(gp)
80017a74:	04410007 	bgez	v0,80017a94 <init_interrupts+0x24>
80017a78:	3c02a430 	lui	v0,0xa430
80017a7c:	24030555 	li	v1,1365
80017a80:	ac43000c 	sw	v1,12(v0)
80017a84:	af8080ac 	sw	zero,-32596(gp)
80017a88:	40026000 	mfc0	v0,c0_sr
80017a8c:	34420401 	ori	v0,v0,0x401
80017a90:	40826000 	mtc0	v0,c0_sr
80017a94:	03e00008 	jr	ra
80017a98:	00000000 	nop
80017a9c:	00000000 	nop

80017aa0 <disable_interrupts>:
80017aa0:	8f8280ac 	lw	v0,-32596(gp)
80017aa4:	0440000b 	bltz	v0,80017ad4 <disable_interrupts+0x34>
80017aa8:	00000000 	nop
80017aac:	54400008 	bnezl	v0,80017ad0 <disable_interrupts+0x30>
80017ab0:	24420001 	addiu	v0,v0,1
80017ab4:	40036000 	mfc0	v1,c0_sr
80017ab8:	2404fffe 	li	a0,-2
80017abc:	00641824 	and	v1,v1,a0
80017ac0:	40836000 	mtc0	v1,c0_sr
80017ac4:	40034800 	mfc0	v1,c0_count
80017ac8:	af838368 	sw	v1,-31896(gp)
80017acc:	24420001 	addiu	v0,v0,1
80017ad0:	af8280ac 	sw	v0,-32596(gp)
80017ad4:	03e00008 	jr	ra
80017ad8:	00000000 	nop
80017adc:	00000000 	nop

80017ae0 <enable_interrupts>:
80017ae0:	8f8280ac 	lw	v0,-32596(gp)
80017ae4:	27bdffd0 	addiu	sp,sp,-48
80017ae8:	04400008 	bltz	v0,80017b0c <enable_interrupts+0x2c>
80017aec:	ffbf0028 	sd	ra,40(sp)
80017af0:	10400009 	beqz	v0,80017b18 <enable_interrupts+0x38>
80017af4:	2442ffff 	addiu	v0,v0,-1
80017af8:	14400004 	bnez	v0,80017b0c <enable_interrupts+0x2c>
80017afc:	af8280ac 	sw	v0,-32596(gp)
80017b00:	40026000 	mfc0	v0,c0_sr
80017b04:	34420001 	ori	v0,v0,0x1
80017b08:	40826000 	mtc0	v0,c0_sr
80017b0c:	dfbf0028 	ld	ra,40(sp)
80017b10:	03e00008 	jr	ra
80017b14:	27bd0030 	addiu	sp,sp,48
80017b18:	3c028002 	lui	v0,0x8002
80017b1c:	2442b660 	addiu	v0,v0,-18848
80017b20:	3c078002 	lui	a3,0x8002
80017b24:	3c068002 	lui	a2,0x8002
80017b28:	3c048002 	lui	a0,0x8002
80017b2c:	afa20024 	sw	v0,36(sp)
80017b30:	24e7b610 	addiu	a3,a3,-18928
80017b34:	24c6b688 	addiu	a2,a2,-18808
80017b38:	2405027e 	li	a1,638
80017b3c:	0c005ed2 	jal	80017b48 <debug_assert_func_f>
80017b40:	2484b628 	addiu	a0,a0,-18904
80017b44:	00000000 	nop

80017b48 <debug_assert_func_f>:
80017b48:	27bdffa0 	addiu	sp,sp,-96
80017b4c:	afa70078 	sw	a3,120(sp)
80017b50:	ffbf0058 	sd	ra,88(sp)
80017b54:	ffb20050 	sd	s2,80(sp)
80017b58:	ffb10048 	sd	s1,72(sp)
80017b5c:	00a09025 	move	s2,a1
80017b60:	ffb00040 	sd	s0,64(sp)
80017b64:	afa40060 	sw	a0,96(sp)
80017b68:	00c08025 	move	s0,a2
80017b6c:	0c006154 	jal	80018550 <console_init>
80017b70:	8fb10084 	lw	s1,132(sp)
80017b74:	0c0061ac 	jal	800186b0 <console_set_debug>
80017b78:	24040001 	li	a0,1
80017b7c:	0c006152 	jal	80018548 <console_set_render_mode>
80017b80:	00002025 	move	a0,zero
80017b84:	8f828028 	lw	v0,-32728(gp)
80017b88:	8fa60078 	lw	a2,120(sp)
80017b8c:	8c440008 	lw	a0,8(v0)
80017b90:	12000019 	beqz	s0,80017bf8 <debug_assert_func_f+0xb0>
80017b94:	8fa70060 	lw	a3,96(sp)
80017b98:	3c028002 	lui	v0,0x8002
80017b9c:	2442b6a0 	addiu	v0,v0,-18784
80017ba0:	3c058002 	lui	a1,0x8002
80017ba4:	afb00034 	sw	s0,52(sp)
80017ba8:	afa2002c 	sw	v0,44(sp)
80017bac:	afb20024 	sw	s2,36(sp)
80017bb0:	0c000d70 	jal	800035c0 <fprintf>
80017bb4:	24a5b6b0 	addiu	a1,a1,-18768
80017bb8:	1220000b 	beqz	s1,80017be8 <debug_assert_func_f+0xa0>
80017bbc:	00000000 	nop
80017bc0:	8f828028 	lw	v0,-32728(gp)
80017bc4:	27a60088 	addiu	a2,sp,136
80017bc8:	8c440008 	lw	a0,8(v0)
80017bcc:	02202825 	move	a1,s1
80017bd0:	0c00235c 	jal	80008d70 <vfprintf>
80017bd4:	afa60038 	sw	a2,56(sp)
80017bd8:	8f828028 	lw	v0,-32728(gp)
80017bdc:	2404000a 	li	a0,10
80017be0:	0c000daa 	jal	800036a8 <fputc>
80017be4:	8c450008 	lw	a1,8(v0)
80017be8:	0c00619e 	jal	80018678 <console_render>
80017bec:	00000000 	nop
80017bf0:	0c000a2f 	jal	800028bc <abort>
80017bf4:	00000000 	nop
80017bf8:	3c028002 	lui	v0,0x8002
80017bfc:	2442b1c0 	addiu	v0,v0,-20032
80017c00:	1000ffe7 	b	80017ba0 <debug_assert_func_f+0x58>
80017c04:	00408025 	move	s0,v0

80017c08 <debug_assert_func>:
80017c08:	27bdffd0 	addiu	sp,sp,-48
80017c0c:	afa00024 	sw	zero,36(sp)
80017c10:	ffbf0028 	sd	ra,40(sp)
80017c14:	0c005ed2 	jal	80017b48 <debug_assert_func_f>
80017c18:	00000000 	nop
80017c1c:	00000000 	nop

80017c20 <__display_callback>:
80017c20:	8f8280b4 	lw	v0,-32588(gp)
80017c24:	0440000d 	bltz	v0,80017c5c <__display_callback+0x3c>
80017c28:	8f8380b0 	lw	v1,-32592(gp)
80017c2c:	1043000b 	beq	v0,v1,80017c5c <__display_callback+0x3c>
80017c30:	3c038003 	lui	v1,0x8003
80017c34:	2463dd48 	addiu	v1,v1,-8888
80017c38:	00021080 	sll	v0,v0,0x2
80017c3c:	00431021 	addu	v0,v0,v1
80017c40:	8c430000 	lw	v1,0(v0)
80017c44:	3c02a440 	lui	v0,0xa440
80017c48:	ac430004 	sw	v1,4(v0)
80017c4c:	8f8280b4 	lw	v0,-32588(gp)
80017c50:	af8280b8 	sw	v0,-32584(gp)
80017c54:	2402ffff 	li	v0,-1
80017c58:	af8280b4 	sw	v0,-32588(gp)
80017c5c:	03e00008 	jr	ra
80017c60:	00000000 	nop
80017c64:	00000000 	nop

80017c68 <display_init>:
80017c68:	27bdff50 	addiu	sp,sp,-176
80017c6c:	ffbf00a8 	sd	ra,168(sp)
80017c70:	ffb50088 	sd	s5,136(sp)
80017c74:	ffb40080 	sd	s4,128(sp)
80017c78:	ffb30078 	sd	s3,120(sp)
80017c7c:	ffb20070 	sd	s2,112(sp)
80017c80:	ffb10068 	sd	s1,104(sp)
80017c84:	ffb00060 	sd	s0,96(sp)
80017c88:	00c08825 	move	s1,a2
80017c8c:	00809025 	move	s2,a0
80017c90:	00a09825 	move	s3,a1
80017c94:	00e0a825 	move	s5,a3
80017c98:	8fb400d4 	lw	s4,212(sp)
80017c9c:	ffbe00a0 	sd	s8,160(sp)
80017ca0:	ffb70098 	sd	s7,152(sp)
80017ca4:	0c006464 	jal	80019190 <get_tv_type>
80017ca8:	ffb60090 	sd	s6,144(sp)
80017cac:	0c005ea8 	jal	80017aa0 <disable_interrupts>
80017cb0:	00408025 	move	s0,v0
80017cb4:	2622fffe 	addiu	v0,s1,-2
80017cb8:	2c420002 	sltiu	v0,v0,2
80017cbc:	50400001 	beqzl	v0,80017cc4 <display_init+0x5c>
80017cc0:	24110003 	li	s1,3
80017cc4:	2e440006 	sltiu	a0,s2,6
80017cc8:	108000e5 	beqz	a0,80018060 <display_init+0x3f8>
80017ccc:	af9180bc 	sw	s1,-32580(gp)
80017cd0:	3c038002 	lui	v1,0x8002
80017cd4:	00121080 	sll	v0,s2,0x2
80017cd8:	2463b6e0 	addiu	v1,v1,-18720
80017cdc:	00621021 	addu	v0,v1,v0
80017ce0:	8c420000 	lw	v0,0(v0)
80017ce4:	00400008 	jr	v0
80017ce8:	00000000 	nop
80017cec:	00000000 	nop
80017cf0:	2610000f 	addiu	s0,s0,15
80017cf4:	24023000 	li	v0,12288
80017cf8:	3c038002 	lui	v1,0x8002
80017cfc:	2463b710 	addiu	v1,v1,-18672
80017d00:	00108080 	sll	s0,s0,0x2
80017d04:	02038021 	addu	s0,s0,v1
80017d08:	8e110000 	lw	s1,0(s0)
80017d0c:	6a2a0000 	ldl	t2,0(s1)
80017d10:	6a290008 	ldl	t1,8(s1)
80017d14:	6a280010 	ldl	t0,16(s1)
80017d18:	6a270018 	ldl	a3,24(s1)
80017d1c:	6a260020 	ldl	a2,32(s1)
80017d20:	6a250028 	ldl	a1,40(s1)
80017d24:	6a230030 	ldl	v1,48(s1)
80017d28:	6e2a0007 	ldr	t2,7(s1)
80017d2c:	6e29000f 	ldr	t1,15(s1)
80017d30:	6e280017 	ldr	t0,23(s1)
80017d34:	6e260027 	ldr	a2,39(s1)
80017d38:	6e25002f 	ldr	a1,47(s1)
80017d3c:	6e230037 	ldr	v1,55(s1)
80017d40:	6e27001f 	ldr	a3,31(s1)
80017d44:	ffaa0020 	sd	t2,32(sp)
80017d48:	ffa90028 	sd	t1,40(sp)
80017d4c:	ffa80030 	sd	t0,48(sp)
80017d50:	ffa70038 	sd	a3,56(sp)
80017d54:	ffa60040 	sd	a2,64(sp)
80017d58:	ffa50048 	sd	a1,72(sp)
80017d5c:	126000bc 	beqz	s3,80018050 <display_init+0x3e8>
80017d60:	ffa30050 	sd	v1,80(sp)
80017d64:	24030001 	li	v1,1
80017d68:	52630002 	beql	s3,v1,80017d74 <display_init+0x10c>
80017d6c:	34420003 	ori	v0,v0,0x3
80017d70:	24030001 	li	v1,1
80017d74:	12a300b2 	beq	s5,v1,80018040 <display_init+0x3d8>
80017d78:	24030002 	li	v1,2
80017d7c:	52a30002 	beql	s5,v1,80017d88 <display_init+0x120>
80017d80:	3442000c 	ori	v0,v0,0xc
80017d84:	24030002 	li	v1,2
80017d88:	52830007 	beql	s4,v1,80017da8 <display_init+0x140>
80017d8c:	34420110 	ori	v0,v0,0x110
80017d90:	2e830003 	sltiu	v1,s4,3
80017d94:	106000a6 	beqz	v1,80018030 <display_init+0x3c8>
80017d98:	24030003 	li	v1,3
80017d9c:	52800002 	beqzl	s4,80017da8 <display_init+0x140>
80017da0:	34420300 	ori	v0,v0,0x300
80017da4:	34420210 	ori	v0,v0,0x210
80017da8:	27b00020 	addiu	s0,sp,32
80017dac:	00000000 	nop
80017db0:	3c17a440 	lui	s7,0xa440
80017db4:	afa20020 	sw	v0,32(sp)
80017db8:	afa00044 	sw	zero,68(sp)
80017dbc:	02003025 	move	a2,s0
80017dc0:	00001025 	move	v0,zero
80017dc4:	02f0b823 	subu	s7,s7,s0
80017dc8:	2408000e 	li	t0,14
80017dcc:	00000000 	nop
80017dd0:	2443fffd 	addiu	v1,v0,-3
80017dd4:	2c630002 	sltiu	v1,v1,2
80017dd8:	14600005 	bnez	v1,80017df0 <display_init+0x188>
80017ddc:	24420001 	addiu	v0,v0,1
80017de0:	8cc70000 	lw	a3,0(a2)
80017de4:	00d71821 	addu	v1,a2,s7
80017de8:	10480003 	beq	v0,t0,80017df8 <display_init+0x190>
80017dec:	ac670000 	sw	a3,0(v1)
80017df0:	1000fff7 	b	80017dd0 <display_init+0x168>
80017df4:	24c60004 	addiu	a2,a2,4
80017df8:	1080001b 	beqz	a0,80017e68 <display_init+0x200>
80017dfc:	00129080 	sll	s2,s2,0x2
80017e00:	3c028002 	lui	v0,0x8002
80017e04:	2442b6f8 	addiu	v0,v0,-18696
80017e08:	00529021 	addu	s2,v0,s2
80017e0c:	8e420000 	lw	v0,0(s2)
80017e10:	00400008 	jr	v0
80017e14:	00000000 	nop
80017e18:	26100003 	addiu	s0,s0,3
80017e1c:	1000ffb6 	b	80017cf8 <display_init+0x90>
80017e20:	24023040 	li	v0,12352
80017e24:	00000000 	nop
80017e28:	26100009 	addiu	s0,s0,9
80017e2c:	1000ffb2 	b	80017cf8 <display_init+0x90>
80017e30:	24023040 	li	v0,12352
80017e34:	00000000 	nop
80017e38:	2610000c 	addiu	s0,s0,12
80017e3c:	1000ffae 	b	80017cf8 <display_init+0x90>
80017e40:	24023000 	li	v0,12288
80017e44:	00000000 	nop
80017e48:	26100006 	addiu	s0,s0,6
80017e4c:	1000ffaa 	b	80017cf8 <display_init+0x90>
80017e50:	24023000 	li	v0,12288
80017e54:	00000000 	nop
80017e58:	24020200 	li	v0,512
80017e5c:	af828370 	sw	v0,-31888(gp)
80017e60:	240200f0 	li	v0,240
80017e64:	af82836c 	sw	v0,-31892(gp)
80017e68:	12600002 	beqz	s3,80017e74 <display_init+0x20c>
80017e6c:	24020002 	li	v0,2
80017e70:	24020004 	li	v0,4
80017e74:	8f8380bc 	lw	v1,-32580(gp)
80017e78:	af828374 	sw	v0,-31884(gp)
80017e7c:	10600042 	beqz	v1,80017f88 <display_init+0x320>
80017e80:	3c168003 	lui	s6,0x8003
80017e84:	3c128003 	lui	s2,0x8003
80017e88:	2652dd58 	addiu	s2,s2,-8872
80017e8c:	26d3dd48 	addiu	s3,s6,-8888
80017e90:	0000f025 	move	s8,zero
80017e94:	3c15a000 	lui	s5,0xa000
80017e98:	2414ffc0 	li	s4,-64
80017e9c:	8f848370 	lw	a0,-31888(gp)
80017ea0:	8f83836c 	lw	v1,-31892(gp)
80017ea4:	26520004 	addiu	s2,s2,4
80017ea8:	00830018 	mult	a0,v1
80017eac:	26730004 	addiu	s3,s3,4
80017eb0:	27de0001 	addiu	s8,s8,1
80017eb4:	00001812 	mflo	v1
	...
80017ec0:	00620018 	mult	v1,v0
80017ec4:	00003012 	mflo	a2
80017ec8:	24c4003f 	addiu	a0,a2,63
80017ecc:	0c001169 	jal	800045a4 <malloc>
80017ed0:	afa60058 	sw	a2,88(sp)
80017ed4:	00552025 	or	a0,v0,s5
80017ed8:	2484003f 	addiu	a0,a0,63
80017edc:	00942024 	and	a0,a0,s4
80017ee0:	8fa60058 	lw	a2,88(sp)
80017ee4:	00002825 	move	a1,zero
80017ee8:	ae42fffc 	sw	v0,-4(s2)
80017eec:	0c00173d 	jal	80005cf4 <memset>
80017ef0:	ae64fffc 	sw	a0,-4(s3)
80017ef4:	8f8280bc 	lw	v0,-32580(gp)
80017ef8:	03c2102b 	sltu	v0,s8,v0
80017efc:	10400024 	beqz	v0,80017f90 <display_init+0x328>
80017f00:	8ec4dd48 	lw	a0,-8888(s6)
80017f04:	1000ffe5 	b	80017e9c <display_init+0x234>
80017f08:	8f828374 	lw	v0,-31884(gp)
80017f0c:	00000000 	nop
80017f10:	24020200 	li	v0,512
80017f14:	af828370 	sw	v0,-31888(gp)
80017f18:	240201e0 	li	v0,480
80017f1c:	1000ffd2 	b	80017e68 <display_init+0x200>
80017f20:	af82836c 	sw	v0,-31892(gp)
80017f24:	00000000 	nop
80017f28:	24020280 	li	v0,640
80017f2c:	af828370 	sw	v0,-31888(gp)
80017f30:	240201e0 	li	v0,480
80017f34:	1000ffcc 	b	80017e68 <display_init+0x200>
80017f38:	af82836c 	sw	v0,-31892(gp)
80017f3c:	00000000 	nop
80017f40:	24020280 	li	v0,640
80017f44:	af828370 	sw	v0,-31888(gp)
80017f48:	240200f0 	li	v0,240
80017f4c:	1000ffc6 	b	80017e68 <display_init+0x200>
80017f50:	af82836c 	sw	v0,-31892(gp)
80017f54:	00000000 	nop
80017f58:	24020140 	li	v0,320
80017f5c:	af828370 	sw	v0,-31888(gp)
80017f60:	240200f0 	li	v0,240
80017f64:	1000ffc0 	b	80017e68 <display_init+0x200>
80017f68:	af82836c 	sw	v0,-31892(gp)
80017f6c:	00000000 	nop
80017f70:	24020100 	li	v0,256
80017f74:	af828370 	sw	v0,-31888(gp)
80017f78:	240200f0 	li	v0,240
80017f7c:	1000ffba 	b	80017e68 <display_init+0x200>
80017f80:	af82836c 	sw	v0,-31892(gp)
80017f84:	00000000 	nop
80017f88:	8ec4dd48 	lw	a0,-8888(s6)
80017f8c:	00000000 	nop
80017f90:	8e230024 	lw	v1,36(s1)
80017f94:	2402ffff 	li	v0,-1
80017f98:	af8280b0 	sw	v0,-32592(gp)
80017f9c:	af8280b4 	sw	v0,-32588(gp)
80017fa0:	af8080b8 	sw	zero,-32584(gp)
80017fa4:	afa40024 	sw	a0,36(sp)
80017fa8:	afa30044 	sw	v1,68(sp)
80017fac:	00001025 	move	v0,zero
80017fb0:	2405000e 	li	a1,14
80017fb4:	00000000 	nop
80017fb8:	2443fffd 	addiu	v1,v0,-3
80017fbc:	2c630002 	sltiu	v1,v1,2
80017fc0:	14600005 	bnez	v1,80017fd8 <display_init+0x370>
80017fc4:	24420001 	addiu	v0,v0,1
80017fc8:	8e040000 	lw	a0,0(s0)
80017fcc:	02f01821 	addu	v1,s7,s0
80017fd0:	10450003 	beq	v0,a1,80017fe0 <display_init+0x378>
80017fd4:	ac640000 	sw	a0,0(v1)
80017fd8:	1000fff7 	b	80017fb8 <display_init+0x350>
80017fdc:	26100004 	addiu	s0,s0,4
80017fe0:	0c005eb8 	jal	80017ae0 <enable_interrupts>
80017fe4:	00000000 	nop
80017fe8:	3c048001 	lui	a0,0x8001
80017fec:	0c005e56 	jal	80017958 <register_VI_handler>
80017ff0:	24847c20 	addiu	a0,a0,31776
80017ff4:	dfbf00a8 	ld	ra,168(sp)
80017ff8:	dfbe00a0 	ld	s8,160(sp)
80017ffc:	dfb70098 	ld	s7,152(sp)
80018000:	dfb60090 	ld	s6,144(sp)
80018004:	dfb50088 	ld	s5,136(sp)
80018008:	dfb40080 	ld	s4,128(sp)
8001800c:	dfb30078 	ld	s3,120(sp)
80018010:	dfb20070 	ld	s2,112(sp)
80018014:	dfb10068 	ld	s1,104(sp)
80018018:	dfb00060 	ld	s0,96(sp)
8001801c:	24050200 	li	a1,512
80018020:	24040001 	li	a0,1
80018024:	08005e90 	j	80017a40 <set_VI_interrupt>
80018028:	27bd00b0 	addiu	sp,sp,176
8001802c:	00000000 	nop
80018030:	5283ff5d 	beql	s4,v1,80017da8 <display_init+0x140>
80018034:	34420010 	ori	v0,v0,0x10
80018038:	1000ff5d 	b	80017db0 <display_init+0x148>
8001803c:	27b00020 	addiu	s0,sp,32
80018040:	1683ff53 	bne	s4,v1,80017d90 <display_init+0x128>
80018044:	34420008 	ori	v0,v0,0x8
80018048:	1000ff57 	b	80017da8 <display_init+0x140>
8001804c:	34420110 	ori	v0,v0,0x110
80018050:	3c030001 	lui	v1,0x1
80018054:	24630002 	addiu	v1,v1,2
80018058:	1000ff45 	b	80017d70 <display_init+0x108>
8001805c:	00431025 	or	v0,v0,v1
80018060:	1000ff25 	b	80017cf8 <display_init+0x90>
80018064:	24023000 	li	v0,12288

80018068 <display_close>:
80018068:	27bdffc0 	addiu	sp,sp,-64
8001806c:	ffbf0038 	sd	ra,56(sp)
80018070:	ffb20030 	sd	s2,48(sp)
80018074:	ffb10028 	sd	s1,40(sp)
80018078:	0c005ea8 	jal	80017aa0 <disable_interrupts>
8001807c:	ffb00020 	sd	s0,32(sp)
80018080:	00002825 	move	a1,zero
80018084:	0c005e90 	jal	80017a40 <set_VI_interrupt>
80018088:	00002025 	move	a0,zero
8001808c:	3c048001 	lui	a0,0x8001
80018090:	0c005e68 	jal	800179a0 <unregister_VI_handler>
80018094:	24847c20 	addiu	a0,a0,31776
80018098:	2402ffff 	li	v0,-1
8001809c:	af8280b8 	sw	v0,-32584(gp)
800180a0:	af8280b0 	sw	v0,-32592(gp)
800180a4:	af8280b4 	sw	v0,-32588(gp)
800180a8:	3c02a440 	lui	v0,0xa440
800180ac:	af808370 	sw	zero,-31888(gp)
800180b0:	af80836c 	sw	zero,-31892(gp)
800180b4:	ac400004 	sw	zero,4(v0)
800180b8:	8f8380bc 	lw	v1,-32580(gp)
800180bc:	10600011 	beqz	v1,80018104 <display_close+0x9c>
800180c0:	3c108003 	lui	s0,0x8003
800180c4:	3c118003 	lui	s1,0x8003
800180c8:	2610dd58 	addiu	s0,s0,-8872
800180cc:	2631dd48 	addiu	s1,s1,-8888
800180d0:	00009025 	move	s2,zero
800180d4:	8e040000 	lw	a0,0(s0)
800180d8:	10800004 	beqz	a0,800180ec <display_close+0x84>
800180dc:	26520001 	addiu	s2,s2,1
800180e0:	0c001179 	jal	800045e4 <free>
800180e4:	00000000 	nop
800180e8:	8f8380bc 	lw	v1,-32580(gp)
800180ec:	0243102b 	sltu	v0,s2,v1
800180f0:	ae000000 	sw	zero,0(s0)
800180f4:	ae200000 	sw	zero,0(s1)
800180f8:	26100004 	addiu	s0,s0,4
800180fc:	1440fff5 	bnez	v0,800180d4 <display_close+0x6c>
80018100:	26310004 	addiu	s1,s1,4
80018104:	dfbf0038 	ld	ra,56(sp)
80018108:	dfb20030 	ld	s2,48(sp)
8001810c:	dfb10028 	ld	s1,40(sp)
80018110:	dfb00020 	ld	s0,32(sp)
80018114:	08005eb8 	j	80017ae0 <enable_interrupts>
80018118:	27bd0040 	addiu	sp,sp,64
8001811c:	00000000 	nop

80018120 <display_lock>:
80018120:	27bdffd0 	addiu	sp,sp,-48
80018124:	ffbf0028 	sd	ra,40(sp)
80018128:	0c005ea8 	jal	80017aa0 <disable_interrupts>
8001812c:	00000000 	nop
80018130:	8f8380bc 	lw	v1,-32580(gp)
80018134:	1060000d 	beqz	v1,8001816c <display_lock+0x4c>
80018138:	8f8480b8 	lw	a0,-32584(gp)
8001813c:	8f8580b0 	lw	a1,-32592(gp)
80018140:	8f8680b4 	lw	a2,-32588(gp)
80018144:	00001025 	move	v0,zero
80018148:	50820006 	beql	a0,v0,80018164 <display_lock+0x44>
8001814c:	24420001 	addiu	v0,v0,1
80018150:	50a20004 	beql	a1,v0,80018164 <display_lock+0x44>
80018154:	24420001 	addiu	v0,v0,1
80018158:	54c2000b 	bnel	a2,v0,80018188 <display_lock+0x68>
8001815c:	af8280b0 	sw	v0,-32592(gp)
80018160:	24420001 	addiu	v0,v0,1
80018164:	1462fff8 	bne	v1,v0,80018148 <display_lock+0x28>
80018168:	00000000 	nop
8001816c:	00001025 	move	v0,zero
80018170:	0c005eb8 	jal	80017ae0 <enable_interrupts>
80018174:	afa20020 	sw	v0,32(sp)
80018178:	dfbf0028 	ld	ra,40(sp)
8001817c:	8fa20020 	lw	v0,32(sp)
80018180:	03e00008 	jr	ra
80018184:	27bd0030 	addiu	sp,sp,48
80018188:	24420001 	addiu	v0,v0,1
8001818c:	0c005eb8 	jal	80017ae0 <enable_interrupts>
80018190:	afa20020 	sw	v0,32(sp)
80018194:	dfbf0028 	ld	ra,40(sp)
80018198:	8fa20020 	lw	v0,32(sp)
8001819c:	03e00008 	jr	ra
800181a0:	27bd0030 	addiu	sp,sp,48
800181a4:	00000000 	nop

800181a8 <display_show>:
800181a8:	14800003 	bnez	a0,800181b8 <display_show+0x10>
800181ac:	00000000 	nop
800181b0:	03e00008 	jr	ra
800181b4:	00000000 	nop
800181b8:	27bdffd0 	addiu	sp,sp,-48
800181bc:	ffb00020 	sd	s0,32(sp)
800181c0:	ffbf0028 	sd	ra,40(sp)
800181c4:	0c005ea8 	jal	80017aa0 <disable_interrupts>
800181c8:	00808025 	move	s0,a0
800181cc:	8f8280b0 	lw	v0,-32592(gp)
800181d0:	2610ffff 	addiu	s0,s0,-1
800181d4:	14500005 	bne	v0,s0,800181ec <display_show+0x44>
800181d8:	dfbf0028 	ld	ra,40(sp)
800181dc:	2403ffff 	li	v1,-1
800181e0:	af8380b0 	sw	v1,-32592(gp)
800181e4:	af8280b4 	sw	v0,-32588(gp)
800181e8:	dfbf0028 	ld	ra,40(sp)
800181ec:	dfb00020 	ld	s0,32(sp)
800181f0:	08005eb8 	j	80017ae0 <enable_interrupts>
800181f4:	27bd0030 	addiu	sp,sp,48

800181f8 <__console_render.part.0>:
800181f8:	27bdffa8 	addiu	sp,sp,-88
800181fc:	ffbf0050 	sd	ra,80(sp)
80018200:	ffb50048 	sd	s5,72(sp)
80018204:	ffb40040 	sd	s4,64(sp)
80018208:	ffb30038 	sd	s3,56(sp)
8001820c:	ffb20030 	sd	s2,48(sp)
80018210:	ffb10028 	sd	s1,40(sp)
80018214:	ffb00020 	sd	s0,32(sp)
80018218:	0c006048 	jal	80018120 <display_lock>
8001821c:	00000000 	nop
80018220:	1040fffd 	beqz	v0,80018218 <__console_render.part.0+0x20>
80018224:	af828378 	sw	v0,-31880(gp)
80018228:	00002825 	move	a1,zero
8001822c:	0c00635a 	jal	80018d68 <graphics_fill_screen>
80018230:	00402025 	move	a0,v0
80018234:	0000a025 	move	s4,zero
80018238:	24120008 	li	s2,8
8001823c:	24130240 	li	s3,576
80018240:	24150700 	li	s5,1792
80018244:	02808825 	move	s1,s4
80018248:	10000005 	b	80018260 <__console_render.part.0+0x68>
8001824c:	24100040 	li	s0,64
80018250:	0c006382 	jal	80018e08 <graphics_draw_character>
80018254:	26310001 	addiu	s1,s1,1
80018258:	52130013 	beql	s0,s3,800182a8 <__console_render.part.0+0xb0>
8001825c:	26940040 	addiu	s4,s4,64
80018260:	8f828380 	lw	v0,-31872(gp)
80018264:	02002825 	move	a1,s0
80018268:	00511021 	addu	v0,v0,s1
8001826c:	80470000 	lb	a3,0(v0)
80018270:	8f848378 	lw	a0,-31880(gp)
80018274:	02403025 	move	a2,s2
80018278:	14e0fff5 	bnez	a3,80018250 <__console_render.part.0+0x58>
8001827c:	26100008 	addiu	s0,s0,8
80018280:	dfbf0050 	ld	ra,80(sp)
80018284:	dfb50048 	ld	s5,72(sp)
80018288:	dfb40040 	ld	s4,64(sp)
8001828c:	dfb30038 	ld	s3,56(sp)
80018290:	dfb20030 	ld	s2,48(sp)
80018294:	dfb10028 	ld	s1,40(sp)
80018298:	dfb00020 	ld	s0,32(sp)
8001829c:	0800606a 	j	800181a8 <display_show>
800182a0:	27bd0058 	addiu	sp,sp,88
800182a4:	00000000 	nop
800182a8:	1695ffe6 	bne	s4,s5,80018244 <__console_render.part.0+0x4c>
800182ac:	26520008 	addiu	s2,s2,8
800182b0:	1000fff3 	b	80018280 <__console_render.part.0+0x88>
800182b4:	8f848378 	lw	a0,-31880(gp)

800182b8 <__console_write>:
800182b8:	27bdff88 	addiu	sp,sp,-120
800182bc:	ffb20038 	sd	s2,56(sp)
800182c0:	8f928380 	lw	s2,-31872(gp)
800182c4:	ffb70060 	sd	s7,96(sp)
800182c8:	0080b825 	move	s7,a0
800182cc:	02402025 	move	a0,s2
800182d0:	ffb60058 	sd	s6,88(sp)
800182d4:	ffb10030 	sd	s1,48(sp)
800182d8:	ffb00028 	sd	s0,40(sp)
800182dc:	ffbf0070 	sd	ra,112(sp)
800182e0:	ffbe0068 	sd	s8,104(sp)
800182e4:	ffb50050 	sd	s5,80(sp)
800182e8:	ffb40048 	sd	s4,72(sp)
800182ec:	ffb30040 	sd	s3,64(sp)
800182f0:	0c001f1d 	jal	80007c74 <strlen>
800182f4:	00a0b025 	move	s6,a1
800182f8:	938380c0 	lbu	v1,-32576(gp)
800182fc:	00408025 	move	s0,v0
80018300:	14600073 	bnez	v1,800184d0 <__console_write+0x218>
80018304:	00408825 	move	s1,v0
80018308:	12c00059 	beqz	s6,80018470 <__console_write+0x1b8>
8001830c:	00001825 	move	v1,zero
80018310:	0000a025 	move	s4,zero
80018314:	24100700 	li	s0,1792
80018318:	26450040 	addiu	a1,s2,64
8001831c:	10000008 	b	80018340 <__console_write+0x88>
80018320:	24130020 	li	s3,32
80018324:	00000000 	nop
80018328:	a2260000 	sb	a2,0(s1)
8001832c:	02801825 	move	v1,s4
80018330:	00408825 	move	s1,v0
80018334:	0296202b 	sltu	a0,s4,s6
80018338:	5080004d 	beqzl	a0,80018470 <__console_write+0x1b8>
8001833c:	00408025 	move	s0,v0
80018340:	12300043 	beq	s1,s0,80018450 <__console_write+0x198>
80018344:	240606c1 	li	a2,1729
80018348:	02801025 	move	v0,s4
8001834c:	02e21021 	addu	v0,s7,v0
80018350:	80460000 	lb	a2,0(v0)
80018354:	241e000a 	li	s8,10
80018358:	24740001 	addiu	s4,v1,1
8001835c:	2415000d 	li	s5,13
80018360:	02201825 	move	v1,s1
80018364:	10de0026 	beq	a2,s8,80018400 <__console_write+0x148>
80018368:	26220001 	addiu	v0,s1,1
8001836c:	10d50024 	beq	a2,s5,80018400 <__console_write+0x148>
80018370:	24040009 	li	a0,9
80018374:	54c4ffec 	bnel	a2,a0,80018328 <__console_write+0x70>
80018378:	02518821 	addu	s1,s2,s1
8001837c:	32240003 	andi	a0,s1,0x3
80018380:	14800009 	bnez	a0,800183a8 <__console_write+0xf0>
80018384:	02433821 	addu	a3,s2,v1
80018388:	02511821 	addu	v1,s2,s1
8001838c:	30440003 	andi	a0,v0,0x3
80018390:	10800049 	beqz	a0,800184b8 <__console_write+0x200>
80018394:	a0730000 	sb	s3,0(v1)
80018398:	00401825 	move	v1,v0
8001839c:	00000000 	nop
800183a0:	24420001 	addiu	v0,v0,1
800183a4:	02433821 	addu	a3,s2,v1
800183a8:	30460003 	andi	a2,v0,0x3
800183ac:	00408825 	move	s1,v0
800183b0:	00401825 	move	v1,v0
800183b4:	14c0fffa 	bnez	a2,800183a0 <__console_write+0xe8>
800183b8:	a0f30000 	sb	s3,0(a3)
800183bc:	1450ffdd 	bne	v0,s0,80018334 <__console_write+0x7c>
800183c0:	02801825 	move	v1,s4
800183c4:	240606c1 	li	a2,1729
800183c8:	02402025 	move	a0,s2
800183cc:	0c0016ab 	jal	80005aac <memmove>
800183d0:	afa50020 	sw	a1,32(sp)
800183d4:	0296102b 	sltu	v0,s4,s6
800183d8:	10400054 	beqz	v0,8001852c <__console_write+0x274>
800183dc:	8fa50020 	lw	a1,32(sp)
800183e0:	02801025 	move	v0,s4
800183e4:	02e21021 	addu	v0,s7,v0
800183e8:	80460000 	lb	a2,0(v0)
800183ec:	241106c0 	li	s1,1728
800183f0:	26940001 	addiu	s4,s4,1
800183f4:	02201825 	move	v1,s1
800183f8:	14deffdc 	bne	a2,s8,8001836c <__console_write+0xb4>
800183fc:	26220001 	addiu	v0,s1,1
80018400:	3224003f 	andi	a0,s1,0x3f
80018404:	14800008 	bnez	a0,80018428 <__console_write+0x170>
80018408:	02433821 	addu	a3,s2,v1
8001840c:	02518821 	addu	s1,s2,s1
80018410:	3043003f 	andi	v1,v0,0x3f
80018414:	10600028 	beqz	v1,800184b8 <__console_write+0x200>
80018418:	a2330000 	sb	s3,0(s1)
8001841c:	00401825 	move	v1,v0
80018420:	24420001 	addiu	v0,v0,1
80018424:	02433821 	addu	a3,s2,v1
80018428:	3046003f 	andi	a2,v0,0x3f
8001842c:	00408825 	move	s1,v0
80018430:	00401825 	move	v1,v0
80018434:	14c0fffa 	bnez	a2,80018420 <__console_write+0x168>
80018438:	a0f30000 	sb	s3,0(a3)
8001843c:	1450ffbd 	bne	v0,s0,80018334 <__console_write+0x7c>
80018440:	02801825 	move	v1,s4
80018444:	1000ffe0 	b	800183c8 <__console_write+0x110>
80018448:	240606c1 	li	a2,1729
8001844c:	00000000 	nop
80018450:	02402025 	move	a0,s2
80018454:	afa30024 	sw	v1,36(sp)
80018458:	afa50020 	sw	a1,32(sp)
8001845c:	0c0016ab 	jal	80005aac <memmove>
80018460:	241106c0 	li	s1,1728
80018464:	8fa30024 	lw	v1,36(sp)
80018468:	1000ffb7 	b	80018348 <__console_write+0x90>
8001846c:	8fa50020 	lw	a1,32(sp)
80018470:	8f84837c 	lw	a0,-31876(gp)
80018474:	02501021 	addu	v0,s2,s0
80018478:	24030001 	li	v1,1
8001847c:	1083001a 	beq	a0,v1,800184e8 <__console_write+0x230>
80018480:	a0400000 	sb	zero,0(v0)
80018484:	dfbf0070 	ld	ra,112(sp)
80018488:	dfbe0068 	ld	s8,104(sp)
8001848c:	dfb70060 	ld	s7,96(sp)
80018490:	dfb50050 	ld	s5,80(sp)
80018494:	dfb40048 	ld	s4,72(sp)
80018498:	dfb30040 	ld	s3,64(sp)
8001849c:	dfb20038 	ld	s2,56(sp)
800184a0:	dfb10030 	ld	s1,48(sp)
800184a4:	dfb00028 	ld	s0,40(sp)
800184a8:	02c01025 	move	v0,s6
800184ac:	dfb60058 	ld	s6,88(sp)
800184b0:	03e00008 	jr	ra
800184b4:	27bd0078 	addiu	sp,sp,120
800184b8:	00408825 	move	s1,v0
800184bc:	1450ff9d 	bne	v0,s0,80018334 <__console_write+0x7c>
800184c0:	02801825 	move	v1,s4
800184c4:	1000ffc0 	b	800183c8 <__console_write+0x110>
800184c8:	240606c1 	li	a2,1729
800184cc:	00000000 	nop
800184d0:	02c03025 	move	a2,s6
800184d4:	02e02825 	move	a1,s7
800184d8:	0c006866 	jal	8001a198 <write>
800184dc:	24040002 	li	a0,2
800184e0:	1000ff89 	b	80018308 <__console_write+0x50>
800184e4:	8f928380 	lw	s2,-31872(gp)
800184e8:	1240ffe7 	beqz	s2,80018488 <__console_write+0x1d0>
800184ec:	dfbf0070 	ld	ra,112(sp)
800184f0:	0c00607e 	jal	800181f8 <__console_render.part.0>
800184f4:	00000000 	nop
800184f8:	dfbf0070 	ld	ra,112(sp)
800184fc:	dfbe0068 	ld	s8,104(sp)
80018500:	dfb70060 	ld	s7,96(sp)
80018504:	dfb50050 	ld	s5,80(sp)
80018508:	dfb40048 	ld	s4,72(sp)
8001850c:	dfb30040 	ld	s3,64(sp)
80018510:	dfb20038 	ld	s2,56(sp)
80018514:	dfb10030 	ld	s1,48(sp)
80018518:	dfb00028 	ld	s0,40(sp)
8001851c:	02c01025 	move	v0,s6
80018520:	dfb60058 	ld	s6,88(sp)
80018524:	03e00008 	jr	ra
80018528:	27bd0078 	addiu	sp,sp,120
8001852c:	8f83837c 	lw	v1,-31876(gp)
80018530:	24020001 	li	v0,1
80018534:	1462ffd3 	bne	v1,v0,80018484 <__console_write+0x1cc>
80018538:	a24006c0 	sb	zero,1728(s2)
8001853c:	1000ffec 	b	800184f0 <__console_write+0x238>
80018540:	00000000 	nop
80018544:	00000000 	nop

80018548 <console_set_render_mode>:
80018548:	03e00008 	jr	ra
8001854c:	af84837c 	sw	a0,-31876(gp)

80018550 <console_init>:
80018550:	27bdffb0 	addiu	sp,sp,-80
80018554:	ffb00038 	sd	s0,56(sp)
80018558:	24100001 	li	s0,1
8001855c:	ffbf0048 	sd	ra,72(sp)
80018560:	0c00601a 	jal	80018068 <display_close>
80018564:	ffb10040 	sd	s1,64(sp)
80018568:	00003825 	move	a3,zero
8001856c:	24060002 	li	a2,2
80018570:	00002825 	move	a1,zero
80018574:	24040005 	li	a0,5
80018578:	0c005f1a 	jal	80017c68 <display_init>
8001857c:	afb00024 	sw	s0,36(sp)
80018580:	0c001169 	jal	800045a4 <malloc>
80018584:	24040701 	li	a0,1793
80018588:	1040000d 	beqz	v0,800185c0 <console_init+0x70>
8001858c:	af828380 	sw	v0,-31872(gp)
80018590:	8f828028 	lw	v0,-32728(gp)
80018594:	8f91837c 	lw	s1,-31876(gp)
80018598:	8c440008 	lw	a0,8(v0)
8001859c:	0c000b6b 	jal	80002dac <fflush>
800185a0:	af80837c 	sw	zero,-31876(gp)
800185a4:	8f848380 	lw	a0,-31872(gp)
800185a8:	24060701 	li	a2,1793
800185ac:	00002825 	move	a1,zero
800185b0:	0c00173d 	jal	80005cf4 <memset>
800185b4:	af91837c 	sw	s1,-31876(gp)
800185b8:	12300011 	beq	s1,s0,80018600 <console_init+0xb0>
800185bc:	00000000 	nop
800185c0:	24020001 	li	v0,1
800185c4:	af82837c 	sw	v0,-31876(gp)
800185c8:	a38280c0 	sb	v0,-32576(gp)
800185cc:	3c028002 	lui	v0,0x8002
800185d0:	27a40028 	addiu	a0,sp,40
800185d4:	244282b8 	addiu	v0,v0,-32072
800185d8:	afa00028 	sw	zero,40(sp)
800185dc:	afa2002c 	sw	v0,44(sp)
800185e0:	0c0068b6 	jal	8001a2d8 <hook_stdio_calls>
800185e4:	afa00030 	sw	zero,48(sp)
800185e8:	dfbf0048 	ld	ra,72(sp)
800185ec:	dfb10040 	ld	s1,64(sp)
800185f0:	dfb00038 	ld	s0,56(sp)
800185f4:	03e00008 	jr	ra
800185f8:	27bd0050 	addiu	sp,sp,80
800185fc:	00000000 	nop
80018600:	0c00607e 	jal	800181f8 <__console_render.part.0>
80018604:	00000000 	nop
80018608:	1000ffee 	b	800185c4 <console_init+0x74>
8001860c:	24020001 	li	v0,1

80018610 <console_clear>:
80018610:	8f828380 	lw	v0,-31872(gp)
80018614:	27bdffd0 	addiu	sp,sp,-48
80018618:	ffbf0028 	sd	ra,40(sp)
8001861c:	1040000e 	beqz	v0,80018658 <console_clear+0x48>
80018620:	ffb00020 	sd	s0,32(sp)
80018624:	8f828028 	lw	v0,-32728(gp)
80018628:	8f90837c 	lw	s0,-31876(gp)
8001862c:	8c440008 	lw	a0,8(v0)
80018630:	0c000b6b 	jal	80002dac <fflush>
80018634:	af80837c 	sw	zero,-31876(gp)
80018638:	8f848380 	lw	a0,-31872(gp)
8001863c:	24060701 	li	a2,1793
80018640:	00002825 	move	a1,zero
80018644:	0c00173d 	jal	80005cf4 <memset>
80018648:	af90837c 	sw	s0,-31876(gp)
8001864c:	24020001 	li	v0,1
80018650:	12020005 	beq	s0,v0,80018668 <console_clear+0x58>
80018654:	dfbf0028 	ld	ra,40(sp)
80018658:	dfbf0028 	ld	ra,40(sp)
8001865c:	dfb00020 	ld	s0,32(sp)
80018660:	03e00008 	jr	ra
80018664:	27bd0030 	addiu	sp,sp,48
80018668:	dfb00020 	ld	s0,32(sp)
8001866c:	0800607e 	j	800181f8 <__console_render.part.0>
80018670:	27bd0030 	addiu	sp,sp,48
80018674:	00000000 	nop

80018678 <console_render>:
80018678:	8f828028 	lw	v0,-32728(gp)
8001867c:	27bdffd8 	addiu	sp,sp,-40
80018680:	8c440008 	lw	a0,8(v0)
80018684:	ffbf0020 	sd	ra,32(sp)
80018688:	0c000b6b 	jal	80002dac <fflush>
8001868c:	00000000 	nop
80018690:	8f828380 	lw	v0,-31872(gp)
80018694:	10400004 	beqz	v0,800186a8 <console_render+0x30>
80018698:	dfbf0020 	ld	ra,32(sp)
8001869c:	0800607e 	j	800181f8 <__console_render.part.0>
800186a0:	27bd0028 	addiu	sp,sp,40
800186a4:	00000000 	nop
800186a8:	03e00008 	jr	ra
800186ac:	27bd0028 	addiu	sp,sp,40

800186b0 <console_set_debug>:
800186b0:	03e00008 	jr	ra
800186b4:	a38480c0 	sb	a0,-32576(gp)

800186b8 <controller_init>:
800186b8:	3c058003 	lui	a1,0x8003
800186bc:	3c048003 	lui	a0,0x8003
800186c0:	24a3dda8 	addiu	v1,a1,-8792
800186c4:	2482dd68 	addiu	v0,a0,-8856
800186c8:	fca0dda8 	sd	zero,-8792(a1)
800186cc:	fc80dd68 	sd	zero,-8856(a0)
800186d0:	fc600008 	sd	zero,8(v1)
800186d4:	fc600010 	sd	zero,16(v1)
800186d8:	fc600018 	sd	zero,24(v1)
800186dc:	fc600020 	sd	zero,32(v1)
800186e0:	fc600028 	sd	zero,40(v1)
800186e4:	fc600030 	sd	zero,48(v1)
800186e8:	fc600038 	sd	zero,56(v1)
800186ec:	fc400008 	sd	zero,8(v0)
800186f0:	fc400010 	sd	zero,16(v0)
800186f4:	fc400018 	sd	zero,24(v0)
800186f8:	fc400020 	sd	zero,32(v0)
800186fc:	fc400028 	sd	zero,40(v0)
80018700:	fc400030 	sd	zero,48(v0)
80018704:	03e00008 	jr	ra
80018708:	fc400038 	sd	zero,56(v0)
8001870c:	00000000 	nop

80018710 <joybus_rtc_status>:
80018710:	3c027f80 	lui	v0,0x7f80
80018714:	34428183 	ori	v0,v0,0x8183
80018718:	27bdff58 	addiu	sp,sp,-168
8001871c:	00021078 	dsll	v0,v0,0x1
80018720:	ffa20060 	sd	v0,96(sp)
80018724:	2402ffff 	li	v0,-1
80018728:	0002107c 	dsll32	v0,v0,0x1
8001872c:	27a40060 	addiu	a0,sp,96
80018730:	ffa20068 	sd	v0,104(sp)
80018734:	27a50020 	addiu	a1,sp,32
80018738:	24020001 	li	v0,1
8001873c:	ffbf00a0 	sd	ra,160(sp)
80018740:	ffa20098 	sd	v0,152(sp)
80018744:	ffa00070 	sd	zero,112(sp)
80018748:	ffa00078 	sd	zero,120(sp)
8001874c:	ffa00080 	sd	zero,128(sp)
80018750:	ffa00088 	sd	zero,136(sp)
80018754:	0c006476 	jal	800191d8 <joybus_exec>
80018758:	ffa00090 	sd	zero,144(sp)
8001875c:	93a30029 	lbu	v1,41(sp)
80018760:	93a20028 	lbu	v0,40(sp)
80018764:	93a4002a 	lbu	a0,42(sp)
80018768:	dfbf00a0 	ld	ra,160(sp)
8001876c:	00031c00 	sll	v1,v1,0x10
80018770:	00021200 	sll	v0,v0,0x8
80018774:	00621025 	or	v0,v1,v0
80018778:	00441025 	or	v0,v0,a0
8001877c:	03e00008 	jr	ra
80018780:	27bd00a8 	addiu	sp,sp,168
80018784:	00000000 	nop

80018788 <joybus_rtc_write.isra.0>:
80018788:	3c020a01 	lui	v0,0xa01
8001878c:	24420800 	addiu	v0,v0,2048
80018790:	27bdff58 	addiu	sp,sp,-168
80018794:	00821025 	or	v0,a0,v0
80018798:	ffa20060 	sd	v0,96(sp)
8001879c:	2402ffff 	li	v0,-1
800187a0:	dca30000 	ld	v1,0(a1)
800187a4:	0002147c 	dsll32	v0,v0,0x11
800187a8:	ffa20070 	sd	v0,112(sp)
800187ac:	27a50020 	addiu	a1,sp,32
800187b0:	27a40060 	addiu	a0,sp,96
800187b4:	24020001 	li	v0,1
800187b8:	ffbf00a0 	sd	ra,160(sp)
800187bc:	ffa30068 	sd	v1,104(sp)
800187c0:	ffa00078 	sd	zero,120(sp)
800187c4:	ffa00080 	sd	zero,128(sp)
800187c8:	ffa00088 	sd	zero,136(sp)
800187cc:	ffa00090 	sd	zero,144(sp)
800187d0:	0c006476 	jal	800191d8 <joybus_exec>
800187d4:	ffa20098 	sd	v0,152(sp)
800187d8:	dfbf00a0 	ld	ra,160(sp)
800187dc:	03e00008 	jr	ra
800187e0:	27bd00a8 	addiu	sp,sp,168
800187e4:	00000000 	nop

800187e8 <joybus_rtc_read.isra.0>:
800187e8:	3c020209 	lui	v0,0x209
800187ec:	24420700 	addiu	v0,v0,1792
800187f0:	27bdff50 	addiu	sp,sp,-176
800187f4:	2403ffff 	li	v1,-1
800187f8:	00821025 	or	v0,a0,v0
800187fc:	0003347c 	dsll32	a2,v1,0x11
80018800:	ffb000a0 	sd	s0,160(sp)
80018804:	ffa20060 	sd	v0,96(sp)
80018808:	00a08025 	move	s0,a1
8001880c:	24020001 	li	v0,1
80018810:	27a50020 	addiu	a1,sp,32
80018814:	27a40060 	addiu	a0,sp,96
80018818:	ffbf00a8 	sd	ra,168(sp)
8001881c:	ffa20098 	sd	v0,152(sp)
80018820:	ffa30068 	sd	v1,104(sp)
80018824:	ffa60070 	sd	a2,112(sp)
80018828:	ffa00078 	sd	zero,120(sp)
8001882c:	ffa00080 	sd	zero,128(sp)
80018830:	ffa00088 	sd	zero,136(sp)
80018834:	0c006476 	jal	800191d8 <joybus_exec>
80018838:	ffa00090 	sd	zero,144(sp)
8001883c:	dfa20028 	ld	v0,40(sp)
80018840:	dfbf00a8 	ld	ra,168(sp)
80018844:	fe020000 	sd	v0,0(s0)
80018848:	dfb000a0 	ld	s0,160(sp)
8001884c:	03e00008 	jr	ra
80018850:	27bd00b0 	addiu	sp,sp,176
80018854:	00000000 	nop

80018858 <rtc_init>:
80018858:	8f8380d0 	lw	v1,-32560(gp)
8001885c:	27bdffd0 	addiu	sp,sp,-48
80018860:	2402ffff 	li	v0,-1
80018864:	1062001c 	beq	v1,v0,800188d8 <rtc_init+0x80>
80018868:	ffbf0028 	sd	ra,40(sp)
8001886c:	24040001 	li	a0,1
80018870:	10640007 	beq	v1,a0,80018890 <rtc_init+0x38>
80018874:	00001025 	move	v0,zero
80018878:	dfbf0028 	ld	ra,40(sp)
8001887c:	03e00008 	jr	ra
80018880:	27bd0030 	addiu	sp,sp,48
80018884:	00000000 	nop
80018888:	24020001 	li	v0,1
8001888c:	af8280d0 	sw	v0,-32560(gp)
80018890:	27a50020 	addiu	a1,sp,32
80018894:	00002025 	move	a0,zero
80018898:	0c0061fa 	jal	800187e8 <joybus_rtc_read.isra.0>
8001889c:	ff808390 	sd	zero,-31856(gp)
800188a0:	27a50020 	addiu	a1,sp,32
800188a4:	3c020300 	lui	v0,0x300
800188a8:	00002025 	move	a0,zero
800188ac:	0c0061e2 	jal	80018788 <joybus_rtc_write.isra.0>
800188b0:	afa20020 	sw	v0,32(sp)
800188b4:	0c006468 	jal	800191a0 <wait_ms>
800188b8:	24040014 	li	a0,20
800188bc:	3c048002 	lui	a0,0x8002
800188c0:	0c0068ca 	jal	8001a328 <hook_time_call>
800188c4:	24848b60 	addiu	a0,a0,-29856
800188c8:	dfbf0028 	ld	ra,40(sp)
800188cc:	24020001 	li	v0,1
800188d0:	03e00008 	jr	ra
800188d4:	27bd0030 	addiu	sp,sp,48
800188d8:	0c0061c4 	jal	80018710 <joybus_rtc_status>
800188dc:	00000000 	nop
800188e0:	00021202 	srl	v0,v0,0x8
800188e4:	24031000 	li	v1,4096
800188e8:	1043ffe7 	beq	v0,v1,80018888 <rtc_init+0x30>
800188ec:	dfbf0028 	ld	ra,40(sp)
800188f0:	00001025 	move	v0,zero
800188f4:	af8080d0 	sw	zero,-32560(gp)
800188f8:	03e00008 	jr	ra
800188fc:	27bd0030 	addiu	sp,sp,48

80018900 <rtc_get>:
80018900:	8f8380d0 	lw	v1,-32560(gp)
80018904:	27bdff90 	addiu	sp,sp,-112
80018908:	2402ffff 	li	v0,-1
8001890c:	ffb00028 	sd	s0,40(sp)
80018910:	ffbf0068 	sd	ra,104(sp)
80018914:	ffb70060 	sd	s7,96(sp)
80018918:	ffb60058 	sd	s6,88(sp)
8001891c:	ffb50050 	sd	s5,80(sp)
80018920:	ffb40048 	sd	s4,72(sp)
80018924:	ffb30040 	sd	s3,64(sp)
80018928:	ffb20038 	sd	s2,56(sp)
8001892c:	ffb10030 	sd	s1,48(sp)
80018930:	10620079 	beq	v1,v0,80018b18 <rtc_get+0x218>
80018934:	00808025 	move	s0,a0
80018938:	24040001 	li	a0,1
8001893c:	1064000e 	beq	v1,a0,80018978 <rtc_get+0x78>
80018940:	00001025 	move	v0,zero
80018944:	dfbf0068 	ld	ra,104(sp)
80018948:	dfb70060 	ld	s7,96(sp)
8001894c:	dfb60058 	ld	s6,88(sp)
80018950:	dfb50050 	ld	s5,80(sp)
80018954:	dfb40048 	ld	s4,72(sp)
80018958:	dfb30040 	ld	s3,64(sp)
8001895c:	dfb20038 	ld	s2,56(sp)
80018960:	dfb10030 	ld	s1,48(sp)
80018964:	dfb00028 	ld	s0,40(sp)
80018968:	03e00008 	jr	ra
8001896c:	27bd0070 	addiu	sp,sp,112
80018970:	24020001 	li	v0,1
80018974:	af8280d0 	sw	v0,-32560(gp)
80018978:	0c006406 	jal	80019018 <timer_ticks>
8001897c:	00000000 	nop
80018980:	df838390 	ld	v1,-31856(gp)
80018984:	10600007 	beqz	v1,800189a4 <rtc_get+0xa4>
80018988:	00408825 	move	s1,v0
8001898c:	0043182f 	dsubu	v1,v0,v1
80018990:	3c0202cb 	lui	v0,0x2cb
80018994:	64424179 	daddiu	v0,v0,16761
80018998:	0062182a 	slt	v1,v1,v0
8001899c:	14600050 	bnez	v1,80018ae0 <rtc_get+0x1e0>
800189a0:	df8380c8 	ld	v1,-32568(gp)
800189a4:	27a50020 	addiu	a1,sp,32
800189a8:	0c0061fa 	jal	800187e8 <joybus_rtc_read.isra.0>
800189ac:	24040002 	li	a0,2
800189b0:	93a20027 	lbu	v0,39(sp)
800189b4:	93a80026 	lbu	t0,38(sp)
800189b8:	00022102 	srl	a0,v0,0x4
800189bc:	00041880 	sll	v1,a0,0x2
800189c0:	00831821 	addu	v1,a0,v1
800189c4:	3042000f 	andi	v0,v0,0xf
800189c8:	00031840 	sll	v1,v1,0x1
800189cc:	00084902 	srl	t1,t0,0x4
800189d0:	00621821 	addu	v1,v1,v0
800189d4:	93af0022 	lbu	t7,34(sp)
800189d8:	306300ff 	andi	v1,v1,0xff
800189dc:	93ad0020 	lbu	t5,32(sp)
800189e0:	93b80021 	lbu	t8,33(sp)
800189e4:	93a70023 	lbu	a3,35(sp)
800189e8:	93a60024 	lbu	a2,36(sp)
800189ec:	93a40025 	lbu	a0,37(sp)
800189f0:	00091080 	sll	v0,t1,0x2
800189f4:	39ef0080 	xori	t7,t7,0x80
800189f8:	01224821 	addu	t1,t1,v0
800189fc:	00031040 	sll	v0,v1,0x1
80018a00:	000d6102 	srl	t4,t5,0x4
80018a04:	00185902 	srl	t3,t8,0x4
80018a08:	000f5102 	srl	t2,t7,0x4
80018a0c:	0007c902 	srl	t9,a3,0x4
80018a10:	00062902 	srl	a1,a2,0x4
80018a14:	00047102 	srl	t6,a0,0x4
80018a18:	00094840 	sll	t1,t1,0x1
80018a1c:	00431021 	addu	v0,v0,v1
80018a20:	3108000f 	andi	t0,t0,0xf
80018a24:	01284021 	addu	t0,t1,t0
80018a28:	000cb880 	sll	s7,t4,0x2
80018a2c:	000bb080 	sll	s6,t3,0x2
80018a30:	000aa880 	sll	s5,t2,0x2
80018a34:	0019a080 	sll	s4,t9,0x2
80018a38:	00059880 	sll	s3,a1,0x2
80018a3c:	000e9080 	sll	s2,t6,0x2
80018a40:	000210c0 	sll	v0,v0,0x3
80018a44:	00431021 	addu	v0,v0,v1
80018a48:	01976021 	addu	t4,t4,s7
80018a4c:	01765821 	addu	t3,t3,s6
80018a50:	01555021 	addu	t2,t2,s5
80018a54:	03344821 	addu	t1,t9,s4
80018a58:	00b32821 	addu	a1,a1,s3
80018a5c:	3084000f 	andi	a0,a0,0xf
80018a60:	01d27021 	addu	t6,t6,s2
80018a64:	310800ff 	andi	t0,t0,0xff
80018a68:	000c6040 	sll	t4,t4,0x1
80018a6c:	31ad000f 	andi	t5,t5,0xf
80018a70:	000b5840 	sll	t3,t3,0x1
80018a74:	3318000f 	andi	t8,t8,0xf
80018a78:	000a5040 	sll	t2,t2,0x1
80018a7c:	31ef000f 	andi	t7,t7,0xf
80018a80:	00094840 	sll	t1,t1,0x1
80018a84:	30e7000f 	andi	a3,a3,0xf
80018a88:	00052840 	sll	a1,a1,0x1
80018a8c:	30c6000f 	andi	a2,a2,0xf
80018a90:	2484ffff 	addiu	a0,a0,-1
80018a94:	000e7040 	sll	t6,t6,0x1
80018a98:	2503076c 	addiu	v1,t0,1900
80018a9c:	00021080 	sll	v0,v0,0x2
80018aa0:	018d6821 	addu	t5,t4,t5
80018aa4:	01785821 	addu	t3,t3,t8
80018aa8:	014f5021 	addu	t2,t2,t7
80018aac:	01273821 	addu	a3,t1,a3
80018ab0:	00a62821 	addu	a1,a1,a2
80018ab4:	008e2021 	addu	a0,a0,t6
80018ab8:	00621821 	addu	v1,v1,v0
80018abc:	a38d80ce 	sb	t5,-32562(gp)
80018ac0:	a38b80cd 	sb	t3,-32563(gp)
80018ac4:	a38a80cc 	sb	t2,-32564(gp)
80018ac8:	a38780cb 	sb	a3,-32565(gp)
80018acc:	a38580cf 	sb	a1,-32561(gp)
80018ad0:	a38480ca 	sb	a0,-32566(gp)
80018ad4:	a78380c8 	sh	v1,-32568(gp)
80018ad8:	ff918390 	sd	s1,-31856(gp)
80018adc:	df8380c8 	ld	v1,-32568(gp)
80018ae0:	24020001 	li	v0,1
80018ae4:	b2030000 	sdl	v1,0(s0)
80018ae8:	b6030007 	sdr	v1,7(s0)
80018aec:	dfbf0068 	ld	ra,104(sp)
80018af0:	dfb70060 	ld	s7,96(sp)
80018af4:	dfb60058 	ld	s6,88(sp)
80018af8:	dfb50050 	ld	s5,80(sp)
80018afc:	dfb40048 	ld	s4,72(sp)
80018b00:	dfb30040 	ld	s3,64(sp)
80018b04:	dfb20038 	ld	s2,56(sp)
80018b08:	dfb10030 	ld	s1,48(sp)
80018b0c:	dfb00028 	ld	s0,40(sp)
80018b10:	03e00008 	jr	ra
80018b14:	27bd0070 	addiu	sp,sp,112
80018b18:	0c0061c4 	jal	80018710 <joybus_rtc_status>
80018b1c:	00000000 	nop
80018b20:	00021202 	srl	v0,v0,0x8
80018b24:	24031000 	li	v1,4096
80018b28:	1043ff91 	beq	v0,v1,80018970 <rtc_get+0x70>
80018b2c:	dfbf0068 	ld	ra,104(sp)
80018b30:	dfb70060 	ld	s7,96(sp)
80018b34:	dfb60058 	ld	s6,88(sp)
80018b38:	dfb50050 	ld	s5,80(sp)
80018b3c:	dfb40048 	ld	s4,72(sp)
80018b40:	dfb30040 	ld	s3,64(sp)
80018b44:	dfb20038 	ld	s2,56(sp)
80018b48:	dfb10030 	ld	s1,48(sp)
80018b4c:	dfb00028 	ld	s0,40(sp)
80018b50:	00001025 	move	v0,zero
80018b54:	af8080d0 	sw	zero,-32560(gp)
80018b58:	03e00008 	jr	ra
80018b5c:	27bd0070 	addiu	sp,sp,112

80018b60 <newlib_time_hook>:
80018b60:	27bdffb0 	addiu	sp,sp,-80
80018b64:	ffbf0048 	sd	ra,72(sp)
80018b68:	0c006240 	jal	80018900 <rtc_get>
80018b6c:	27848388 	addiu	a0,gp,-31864
80018b70:	10400015 	beqz	v0,80018bc8 <newlib_time_hook+0x68>
80018b74:	9388838e 	lbu	t0,-31858(gp)
80018b78:	97828388 	lhu	v0,-31864(gp)
80018b7c:	2442f894 	addiu	v0,v0,-1900
80018b80:	9387838d 	lbu	a3,-31859(gp)
80018b84:	9386838c 	lbu	a2,-31860(gp)
80018b88:	9385838b 	lbu	a1,-31861(gp)
80018b8c:	9383838a 	lbu	v1,-31862(gp)
80018b90:	afa20034 	sw	v0,52(sp)
80018b94:	27a40020 	addiu	a0,sp,32
80018b98:	2402ffff 	li	v0,-1
80018b9c:	afa80020 	sw	t0,32(sp)
80018ba0:	afa70024 	sw	a3,36(sp)
80018ba4:	afa60028 	sw	a2,40(sp)
80018ba8:	afa5002c 	sw	a1,44(sp)
80018bac:	afa30030 	sw	v1,48(sp)
80018bb0:	0c001930 	jal	800064c0 <mktime>
80018bb4:	afa20040 	sw	v0,64(sp)
80018bb8:	dfbf0048 	ld	ra,72(sp)
80018bbc:	03e00008 	jr	ra
80018bc0:	27bd0050 	addiu	sp,sp,80
80018bc4:	00000000 	nop
80018bc8:	1000fffb 	b	80018bb8 <newlib_time_hook+0x58>
80018bcc:	2402ffff 	li	v0,-1

80018bd0 <graphics_draw_character.part.0>:
80018bd0:	2482ffff 	addiu	v0,a0,-1
80018bd4:	3c038003 	lui	v1,0x8003
80018bd8:	2463dd48 	addiu	v1,v1,-8888
80018bdc:	00021080 	sll	v0,v0,0x2
80018be0:	8f848374 	lw	a0,-31884(gp)
80018be4:	00431021 	addu	v0,v0,v1
80018be8:	24030002 	li	v1,2
80018bec:	8f8d8398 	lw	t5,-31848(gp)
80018bf0:	8c4c0000 	lw	t4,0(v0)
80018bf4:	8f8980d4 	lw	t1,-32556(gp)
80018bf8:	1083002f 	beq	a0,v1,80018cb8 <graphics_draw_character.part.0+0xe8>
80018bfc:	000738c0 	sll	a3,a3,0x3
80018c00:	3c188002 	lui	t8,0x8002
80018c04:	00c04025 	move	t0,a2
80018c08:	31ab00ff 	andi	t3,t5,0xff
80018c0c:	24ce0008 	addiu	t6,a2,8
80018c10:	2718bb48 	addiu	t8,t8,-17592
80018c14:	00e67823 	subu	t7,a3,a2
80018c18:	24aa0008 	addiu	t2,a1,8
80018c1c:	01e81021 	addu	v0,t7,t0
80018c20:	03021021 	addu	v0,t8,v0
80018c24:	90420000 	lbu	v0,0(v0)
80018c28:	1000000d 	b	80018c60 <graphics_draw_character.part.0+0x90>
80018c2c:	00a02025 	move	a0,a1
80018c30:	04c10008 	bgez	a2,80018c54 <graphics_draw_character.part.0+0x84>
80018c34:	00021040 	sll	v0,v0,0x1
80018c38:	8f838370 	lw	v1,-31888(gp)
80018c3c:	01030018 	mult	t0,v1
80018c40:	00001812 	mflo	v1
80018c44:	00641821 	addu	v1,v1,a0
80018c48:	00031880 	sll	v1,v1,0x2
80018c4c:	01831821 	addu	v1,t4,v1
80018c50:	ac690000 	sw	t1,0(v1)
80018c54:	24840001 	addiu	a0,a0,1
80018c58:	11440012 	beq	t2,a0,80018ca4 <graphics_draw_character.part.0+0xd4>
80018c5c:	304200ff 	andi	v0,v0,0xff
80018c60:	00023600 	sll	a2,v0,0x18
80018c64:	1160fff2 	beqz	t3,80018c30 <graphics_draw_character.part.0+0x60>
80018c68:	00063603 	sra	a2,a2,0x18
80018c6c:	8f838370 	lw	v1,-31888(gp)
80018c70:	01030018 	mult	t0,v1
80018c74:	00001812 	mflo	v1
80018c78:	04c00002 	bltz	a2,80018c84 <graphics_draw_character.part.0+0xb4>
80018c7c:	01203825 	move	a3,t1
80018c80:	01a03825 	move	a3,t5
80018c84:	00641821 	addu	v1,v1,a0
80018c88:	00031880 	sll	v1,v1,0x2
80018c8c:	01831821 	addu	v1,t4,v1
80018c90:	00021040 	sll	v0,v0,0x1
80018c94:	24840001 	addiu	a0,a0,1
80018c98:	ac670000 	sw	a3,0(v1)
80018c9c:	1544fff0 	bne	t2,a0,80018c60 <graphics_draw_character.part.0+0x90>
80018ca0:	304200ff 	andi	v0,v0,0xff
80018ca4:	25080001 	addiu	t0,t0,1
80018ca8:	150effdd 	bne	t0,t6,80018c20 <graphics_draw_character.part.0+0x50>
80018cac:	01e81021 	addu	v0,t7,t0
80018cb0:	03e00008 	jr	ra
80018cb4:	00000000 	nop
80018cb8:	8f8b8370 	lw	t3,-31888(gp)
80018cbc:	3c0a8002 	lui	t2,0x8002
80018cc0:	00cb0018 	mult	a2,t3
80018cc4:	254abb48 	addiu	t2,t2,-17592
80018cc8:	000d4027 	nor	t0,zero,t5
80018ccc:	25420008 	addiu	v0,t2,8
80018cd0:	31080001 	andi	t0,t0,0x1
80018cd4:	00001812 	mflo	v1
80018cd8:	00652821 	addu	a1,v1,a1
80018cdc:	00052840 	sll	a1,a1,0x1
80018ce0:	24a50010 	addiu	a1,a1,16
80018ce4:	01475021 	addu	t2,t2,a3
80018ce8:	3129ffff 	andi	t1,t1,0xffff
80018cec:	000b5840 	sll	t3,t3,0x1
80018cf0:	01852821 	addu	a1,t4,a1
80018cf4:	00473821 	addu	a3,v0,a3
80018cf8:	31adffff 	andi	t5,t5,0xffff
80018cfc:	00000000 	nop
80018d00:	91420000 	lbu	v0,0(t2)
80018d04:	10000008 	b	80018d28 <graphics_draw_character.part.0+0x158>
80018d08:	24a3fff0 	addiu	v1,a1,-16
80018d0c:	00000000 	nop
80018d10:	04820001 	bltzl	a0,80018d18 <graphics_draw_character.part.0+0x148>
80018d14:	a4690000 	sh	t1,0(v1)
80018d18:	00021040 	sll	v0,v0,0x1
80018d1c:	24630002 	addiu	v1,v1,2
80018d20:	10a3000c 	beq	a1,v1,80018d54 <graphics_draw_character.part.0+0x184>
80018d24:	304200ff 	andi	v0,v0,0xff
80018d28:	00022600 	sll	a0,v0,0x18
80018d2c:	1500fff8 	bnez	t0,80018d10 <graphics_draw_character.part.0+0x140>
80018d30:	00042603 	sra	a0,a0,0x18
80018d34:	04800002 	bltz	a0,80018d40 <graphics_draw_character.part.0+0x170>
80018d38:	01203025 	move	a2,t1
80018d3c:	01a03025 	move	a2,t5
80018d40:	a4660000 	sh	a2,0(v1)
80018d44:	00021040 	sll	v0,v0,0x1
80018d48:	24630002 	addiu	v1,v1,2
80018d4c:	14a3fff6 	bne	a1,v1,80018d28 <graphics_draw_character.part.0+0x158>
80018d50:	304200ff 	andi	v0,v0,0xff
80018d54:	254a0001 	addiu	t2,t2,1
80018d58:	1547ffe9 	bne	t2,a3,80018d00 <graphics_draw_character.part.0+0x130>
80018d5c:	00ab2821 	addu	a1,a1,t3
80018d60:	03e00008 	jr	ra
80018d64:	00000000 	nop

80018d68 <graphics_fill_screen>:
80018d68:	10800018 	beqz	a0,80018dcc <graphics_fill_screen+0x64>
80018d6c:	2484ffff 	addiu	a0,a0,-1
80018d70:	3c028003 	lui	v0,0x8003
80018d74:	2442dd48 	addiu	v0,v0,-8888
80018d78:	00042080 	sll	a0,a0,0x2
80018d7c:	8f83836c 	lw	v1,-31892(gp)
80018d80:	00822021 	addu	a0,a0,v0
80018d84:	8f828370 	lw	v0,-31888(gp)
80018d88:	8f878374 	lw	a3,-31884(gp)
80018d8c:	00620018 	mult	v1,v0
80018d90:	24060002 	li	a2,2
80018d94:	00001812 	mflo	v1
80018d98:	10e6000f 	beq	a3,a2,80018dd8 <graphics_fill_screen+0x70>
80018d9c:	8c820000 	lw	v0,0(a0)
80018da0:	1060000a 	beqz	v1,80018dcc <graphics_fill_screen+0x64>
80018da4:	00002025 	move	a0,zero
80018da8:	ac450000 	sw	a1,0(v0)
80018dac:	8f83836c 	lw	v1,-31892(gp)
80018db0:	8f868370 	lw	a2,-31888(gp)
80018db4:	24840001 	addiu	a0,a0,1
80018db8:	00c30018 	mult	a2,v1
80018dbc:	00001812 	mflo	v1
80018dc0:	0083182b 	sltu	v1,a0,v1
80018dc4:	1460fff8 	bnez	v1,80018da8 <graphics_fill_screen+0x40>
80018dc8:	24420004 	addiu	v0,v0,4
80018dcc:	03e00008 	jr	ra
80018dd0:	00000000 	nop
80018dd4:	00000000 	nop
80018dd8:	1060fffc 	beqz	v1,80018dcc <graphics_fill_screen+0x64>
80018ddc:	00031840 	sll	v1,v1,0x1
80018de0:	30a5ffff 	andi	a1,a1,0xffff
80018de4:	00431821 	addu	v1,v0,v1
80018de8:	a4450000 	sh	a1,0(v0)
80018dec:	00000000 	nop
80018df0:	24420002 	addiu	v0,v0,2
80018df4:	5443fffe 	bnel	v0,v1,80018df0 <graphics_fill_screen+0x88>
80018df8:	a4450000 	sh	a1,0(v0)
80018dfc:	03e00008 	jr	ra
80018e00:	00000000 	nop
80018e04:	00000000 	nop

80018e08 <graphics_draw_character>:
80018e08:	10800003 	beqz	a0,80018e18 <graphics_draw_character+0x10>
80018e0c:	00000000 	nop
80018e10:	080062f4 	j	80018bd0 <graphics_draw_character.part.0>
80018e14:	00000000 	nop
80018e18:	03e00008 	jr	ra
80018e1c:	00000000 	nop

80018e20 <timer_callback_overflow>:
80018e20:	8f82839c 	lw	v0,-31844(gp)
80018e24:	24420001 	addiu	v0,v0,1
80018e28:	af82839c 	sw	v0,-31844(gp)
80018e2c:	03e00008 	jr	ra
80018e30:	00000000 	nop
80018e34:	00000000 	nop

80018e38 <timer_callback>:
80018e38:	27bdffc8 	addiu	sp,sp,-56
80018e3c:	8f8883a0 	lw	t0,-31840(gp)
80018e40:	ffbf0030 	sd	ra,48(sp)
80018e44:	ffb10028 	sd	s1,40(sp)
80018e48:	ffb00020 	sd	s0,32(sp)
80018e4c:	40065800 	mfc0	a2,c0_compare
80018e50:	1100001c 	beqz	t0,80018ec4 <timer_callback+0x8c>
80018e54:	40074800 	mfc0	a3,c0_count
80018e58:	01008025 	move	s0,t0
80018e5c:	10000003 	b	80018e6c <timer_callback+0x34>
80018e60:	00008825 	move	s1,zero
80018e64:	00000000 	nop
80018e68:	00408025 	move	s0,v0
80018e6c:	8e03000c 	lw	v1,12(s0)
80018e70:	30620082 	andi	v0,v1,0x82
80018e74:	54400009 	bnezl	v0,80018e9c <timer_callback+0x64>
80018e78:	8e020014 	lw	v0,20(s0)
80018e7c:	8e020000 	lw	v0,0(s0)
80018e80:	00e22023 	subu	a0,a3,v0
80018e84:	00461023 	subu	v0,v0,a2
80018e88:	04400003 	bltz	v0,80018e98 <timer_callback+0x60>
80018e8c:	248500ea 	addiu	a1,a0,234
80018e90:	04a3001f 	bgezl	a1,80018f10 <timer_callback+0xd8>
80018e94:	8e020010 	lw	v0,16(s0)
80018e98:	8e020014 	lw	v0,20(s0)
80018e9c:	1440fff2 	bnez	v0,80018e68 <timer_callback+0x30>
80018ea0:	02008825 	move	s1,s0
80018ea4:	01001025 	move	v0,t0
80018ea8:	2404ff7f 	li	a0,-129
80018eac:	8c43000c 	lw	v1,12(v0)
80018eb0:	00641824 	and	v1,v1,a0
80018eb4:	ac43000c 	sw	v1,12(v0)
80018eb8:	8c420014 	lw	v0,20(v0)
80018ebc:	5440fffc 	bnezl	v0,80018eb0 <timer_callback+0x78>
80018ec0:	8c43000c 	lw	v1,12(v0)
80018ec4:	40044800 	mfc0	a0,c0_count
80018ec8:	11000009 	beqz	t0,80018ef0 <timer_callback+0xb8>
80018ecc:	2403ffff 	li	v1,-1
80018ed0:	8d020000 	lw	v0,0(t0)
80018ed4:	00441023 	subu	v0,v0,a0
80018ed8:	0043282b 	sltu	a1,v0,v1
80018edc:	54a00001 	bnezl	a1,80018ee4 <timer_callback+0xac>
80018ee0:	00401825 	move	v1,v0
80018ee4:	8d080014 	lw	t0,20(t0)
80018ee8:	5500fffa 	bnezl	t0,80018ed4 <timer_callback+0x9c>
80018eec:	8d020000 	lw	v0,0(t0)
80018ef0:	00831821 	addu	v1,a0,v1
80018ef4:	40835800 	mtc0	v1,c0_compare
80018ef8:	dfbf0030 	ld	ra,48(sp)
80018efc:	dfb10028 	ld	s1,40(sp)
80018f00:	dfb00020 	ld	s0,32(sp)
80018f04:	03e00008 	jr	ra
80018f08:	27bd0038 	addiu	sp,sp,56
80018f0c:	00000000 	nop
80018f10:	10400004 	beqz	v0,80018f24 <timer_callback+0xec>
80018f14:	ae040008 	sw	a0,8(s0)
80018f18:	0040f809 	jalr	v0
80018f1c:	00000000 	nop
80018f20:	8e03000c 	lw	v1,12(s0)
80018f24:	30620001 	andi	v0,v1,0x1
80018f28:	1040000a 	beqz	v0,80018f54 <timer_callback+0x11c>
80018f2c:	30620040 	andi	v0,v1,0x40
80018f30:	8e040000 	lw	a0,0(s0)
80018f34:	8e050004 	lw	a1,4(s0)
80018f38:	00852021 	addu	a0,a0,a1
80018f3c:	8f8883a0 	lw	t0,-31840(gp)
80018f40:	1040ffc2 	beqz	v0,80018e4c <timer_callback+0x14>
80018f44:	ae040000 	sw	a0,0(s0)
80018f48:	34630080 	ori	v1,v1,0x80
80018f4c:	1000ffbf 	b	80018e4c <timer_callback+0x14>
80018f50:	ae03000c 	sw	v1,12(s0)
80018f54:	12200004 	beqz	s1,80018f68 <timer_callback+0x130>
80018f58:	8e080014 	lw	t0,20(s0)
80018f5c:	ae280014 	sw	t0,20(s1)
80018f60:	1000ffba 	b	80018e4c <timer_callback+0x14>
80018f64:	8f8883a0 	lw	t0,-31840(gp)
80018f68:	1000ffb8 	b	80018e4c <timer_callback+0x14>
80018f6c:	af8883a0 	sw	t0,-31840(gp)

80018f70 <timer_init>:
80018f70:	8f8283a0 	lw	v0,-31840(gp)
80018f74:	27bdffd0 	addiu	sp,sp,-48
80018f78:	1440001c 	bnez	v0,80018fec <timer_init+0x7c>
80018f7c:	ffbf0028 	sd	ra,40(sp)
80018f80:	0c001169 	jal	800045a4 <malloc>
80018f84:	24040018 	li	a0,24
80018f88:	10400008 	beqz	v0,80018fac <timer_init+0x3c>
80018f8c:	24030041 	li	v1,65
80018f90:	ac43000c 	sw	v1,12(v0)
80018f94:	3c038002 	lui	v1,0x8002
80018f98:	24638e20 	addiu	v1,v1,-29152
80018f9c:	fc400000 	sd	zero,0(v0)
80018fa0:	ac430010 	sw	v1,16(v0)
80018fa4:	ac400014 	sw	zero,20(v0)
80018fa8:	af8283a0 	sw	v0,-31840(gp)
80018fac:	0c005ea8 	jal	80017aa0 <disable_interrupts>
80018fb0:	00000000 	nop
80018fb4:	af80839c 	sw	zero,-31844(gp)
80018fb8:	24020001 	li	v0,1
80018fbc:	40824800 	mtc0	v0,c0_count
80018fc0:	00001025 	move	v0,zero
80018fc4:	40825800 	mtc0	v0,c0_compare
80018fc8:	40026000 	mfc0	v0,c0_sr
80018fcc:	34428000 	ori	v0,v0,0x8000
80018fd0:	40826000 	mtc0	v0,c0_sr
80018fd4:	3c048002 	lui	a0,0x8002
80018fd8:	0c005e7e 	jal	800179f8 <register_TI_handler>
80018fdc:	24848e38 	addiu	a0,a0,-29128
80018fe0:	dfbf0028 	ld	ra,40(sp)
80018fe4:	08005eb8 	j	80017ae0 <enable_interrupts>
80018fe8:	27bd0030 	addiu	sp,sp,48
80018fec:	3c028002 	lui	v0,0x8002
80018ff0:	2442c388 	addiu	v0,v0,-15480
80018ff4:	3c078002 	lui	a3,0x8002
80018ff8:	3c068002 	lui	a2,0x8002
80018ffc:	3c048002 	lui	a0,0x8002
80019000:	afa20024 	sw	v0,36(sp)
80019004:	24e7c348 	addiu	a3,a3,-15544
80019008:	24c6c3f0 	addiu	a2,a2,-15376
8001900c:	240500cc 	li	a1,204
80019010:	0c005ed2 	jal	80017b48 <debug_assert_func_f>
80019014:	2484c358 	addiu	a0,a0,-15528

80019018 <timer_ticks>:
80019018:	8f8283a0 	lw	v0,-31840(gp)
8001901c:	27bdffd0 	addiu	sp,sp,-48
80019020:	1040001e 	beqz	v0,8001909c <timer_ticks+0x84>
80019024:	ffbf0028 	sd	ra,40(sp)
80019028:	40026000 	mfc0	v0,c0_sr
8001902c:	30420001 	andi	v0,v0,0x1
80019030:	1440000f 	bnez	v0,80019070 <timer_ticks+0x58>
80019034:	00000000 	nop
80019038:	40034800 	mfc0	v1,c0_count
8001903c:	8f84839c 	lw	a0,-31844(gp)
80019040:	8f828368 	lw	v0,-31896(gp)
80019044:	0062102b 	sltu	v0,v1,v0
80019048:	00822021 	addu	a0,a0,v0
8001904c:	dfbf0028 	ld	ra,40(sp)
80019050:	0003183c 	dsll32	v1,v1,0x0
80019054:	0004103c 	dsll32	v0,a0,0x0
80019058:	0003183e 	dsrl32	v1,v1,0x0
8001905c:	0043102d 	daddu	v0,v0,v1
80019060:	03e00008 	jr	ra
80019064:	27bd0030 	addiu	sp,sp,48
80019068:	0460fffa 	bltz	v1,80019054 <timer_ticks+0x3c>
8001906c:	0003183c 	dsll32	v1,v1,0x0
80019070:	40024800 	mfc0	v0,c0_count
80019074:	8f84839c 	lw	a0,-31844(gp)
80019078:	40034800 	mfc0	v1,c0_count
8001907c:	0440fffa 	bltz	v0,80019068 <timer_ticks+0x50>
80019080:	dfbf0028 	ld	ra,40(sp)
80019084:	0003183c 	dsll32	v1,v1,0x0
80019088:	0004103c 	dsll32	v0,a0,0x0
8001908c:	0003183e 	dsrl32	v1,v1,0x0
80019090:	0043102d 	daddu	v0,v0,v1
80019094:	03e00008 	jr	ra
80019098:	27bd0030 	addiu	sp,sp,48
8001909c:	3c028002 	lui	v0,0x8002
800190a0:	2442c3c0 	addiu	v0,v0,-15424
800190a4:	3c078002 	lui	a3,0x8002
800190a8:	3c068002 	lui	a2,0x8002
800190ac:	3c048002 	lui	a0,0x8002
800190b0:	afa20024 	sw	v0,36(sp)
800190b4:	24e7c3b0 	addiu	a3,a3,-15440
800190b8:	24c6c3e0 	addiu	a2,a2,-15392
800190bc:	240501ae 	li	a1,430
800190c0:	0c005ed2 	jal	80017b48 <debug_assert_func_f>
800190c4:	2484c358 	addiu	a0,a0,-15528

800190c8 <__do_global_ctors>:
800190c8:	27bdffc8 	addiu	sp,sp,-56
800190cc:	ffb10028 	sd	s1,40(sp)
800190d0:	ffb00020 	sd	s0,32(sp)
800190d4:	3c118002 	lui	s1,0x8002
800190d8:	3c108002 	lui	s0,0x8002
800190dc:	ffbf0030 	sd	ra,48(sp)
800190e0:	2610ca5c 	addiu	s0,s0,-13732
800190e4:	2631ca60 	addiu	s1,s1,-13728
800190e8:	0230102b 	sltu	v0,s1,s0
800190ec:	1040000a 	beqz	v0,80019118 <__do_global_ctors+0x50>
800190f0:	dfbf0030 	ld	ra,48(sp)
800190f4:	8e020000 	lw	v0,0(s0)
800190f8:	1040fffb 	beqz	v0,800190e8 <__do_global_ctors+0x20>
800190fc:	2610fffc 	addiu	s0,s0,-4
80019100:	0040f809 	jalr	v0
80019104:	00000000 	nop
80019108:	0230102b 	sltu	v0,s1,s0
8001910c:	5440fffa 	bnezl	v0,800190f8 <__do_global_ctors+0x30>
80019110:	8e020000 	lw	v0,0(s0)
80019114:	dfbf0030 	ld	ra,48(sp)
80019118:	dfb10028 	ld	s1,40(sp)
8001911c:	dfb00020 	ld	s0,32(sp)
80019120:	03e00008 	jr	ra
80019124:	27bd0038 	addiu	sp,sp,56

80019128 <data_cache_hit_writeback_invalidate>:
80019128:	10a0000f 	beqz	a1,80019168 <data_cache_hit_writeback_invalidate+0x40>
8001912c:	2402fff0 	li	v0,-16
80019130:	00823024 	and	a2,a0,v0
80019134:	00862023 	subu	a0,a0,a2
80019138:	00852021 	addu	a0,a0,a1
8001913c:	1880000a 	blez	a0,80019168 <data_cache_hit_writeback_invalidate+0x40>
80019140:	2483ffff 	addiu	v1,a0,-1
80019144:	00621824 	and	v1,v1,v0
80019148:	24c20010 	addiu	v0,a2,16
8001914c:	10000003 	b	8001915c <data_cache_hit_writeback_invalidate+0x34>
80019150:	00621821 	addu	v1,v1,v0
80019154:	00000000 	nop
80019158:	24420010 	addiu	v0,v0,16
8001915c:	bcd50000 	cache	0x15,0(a2)
80019160:	1443fffd 	bne	v0,v1,80019158 <data_cache_hit_writeback_invalidate+0x30>
80019164:	00403025 	move	a2,v0
80019168:	03e00008 	jr	ra
8001916c:	00000000 	nop

80019170 <get_memory_size>:
80019170:	8f8380d8 	lw	v1,-32552(gp)
80019174:	240217d9 	li	v0,6105
80019178:	10620003 	beq	v1,v0,80019188 <get_memory_size+0x18>
8001917c:	3c02a000 	lui	v0,0xa000
80019180:	03e00008 	jr	ra
80019184:	8c420318 	lw	v0,792(v0)
80019188:	03e00008 	jr	ra
8001918c:	8c4203f0 	lw	v0,1008(v0)

80019190 <get_tv_type>:
80019190:	3c028000 	lui	v0,0x8000
80019194:	03e00008 	jr	ra
80019198:	8c420300 	lw	v0,768(v0)
8001919c:	00000000 	nop

800191a0 <wait_ms>:
800191a0:	3402b71b 	li	v0,0xb71b
800191a4:	00820018 	mult	a0,v0
800191a8:	00002012 	mflo	a0
800191ac:	40034800 	mfc0	v1,c0_count
	...
800191b8:	40024800 	mfc0	v0,c0_count
800191bc:	00431023 	subu	v0,v0,v1
800191c0:	0044102b 	sltu	v0,v0,a0
800191c4:	1440fffc 	bnez	v0,800191b8 <wait_ms+0x18>
800191c8:	00000000 	nop
800191cc:	03e00008 	jr	ra
800191d0:	00000000 	nop
800191d4:	00000000 	nop

800191d8 <joybus_exec>:
800191d8:	27bdff30 	addiu	sp,sp,-208
800191dc:	ffb200b8 	sd	s2,184(sp)
800191e0:	27b2002f 	addiu	s2,sp,47
800191e4:	00129102 	srl	s2,s2,0x4
800191e8:	00129100 	sll	s2,s2,0x4
800191ec:	ffb300c0 	sd	s3,192(sp)
800191f0:	26530040 	addiu	s3,s2,64
800191f4:	ffb100b0 	sd	s1,176(sp)
800191f8:	ffb000a8 	sd	s0,168(sp)
800191fc:	00808825 	move	s1,a0
80019200:	00a08025 	move	s0,a1
80019204:	02602025 	move	a0,s3
80019208:	ffbf00c8 	sd	ra,200(sp)
8001920c:	0c00644a 	jal	80019128 <data_cache_hit_writeback_invalidate>
80019210:	24050040 	li	a1,64
80019214:	6a2a0000 	ldl	t2,0(s1)
80019218:	6a290008 	ldl	t1,8(s1)
8001921c:	6a280010 	ldl	t0,16(s1)
80019220:	6a270018 	ldl	a3,24(s1)
80019224:	6a260020 	ldl	a2,32(s1)
80019228:	6a250028 	ldl	a1,40(s1)
8001922c:	6a240030 	ldl	a0,48(s1)
80019230:	6a230038 	ldl	v1,56(s1)
80019234:	6e29000f 	ldr	t1,15(s1)
80019238:	6e23003f 	ldr	v1,63(s1)
8001923c:	6e280017 	ldr	t0,23(s1)
80019240:	6e27001f 	ldr	a3,31(s1)
80019244:	6e260027 	ldr	a2,39(s1)
80019248:	6e25002f 	ldr	a1,47(s1)
8001924c:	6e240037 	ldr	a0,55(s1)
80019250:	6e2a0007 	ldr	t2,7(s1)
80019254:	3c022000 	lui	v0,0x2000
80019258:	02621025 	or	v0,s3,v0
8001925c:	fc4a0000 	sd	t2,0(v0)
80019260:	fc430038 	sd	v1,56(v0)
80019264:	fc490008 	sd	t1,8(v0)
80019268:	fc480010 	sd	t0,16(v0)
8001926c:	fc470018 	sd	a3,24(v0)
80019270:	fc460020 	sd	a2,32(v0)
80019274:	fc450028 	sd	a1,40(v0)
80019278:	0c005ea8 	jal	80017aa0 <disable_interrupts>
8001927c:	fc440030 	sd	a0,48(v0)
80019280:	3c03a480 	lui	v1,0xa480
80019284:	00000000 	nop
80019288:	8c620018 	lw	v0,24(v1)
8001928c:	30420003 	andi	v0,v0,0x3
80019290:	1440fffd 	bnez	v0,80019288 <joybus_exec+0xb0>
80019294:	00000000 	nop
80019298:	ac730000 	sw	s3,0(v1)
8001929c:	3c021fc0 	lui	v0,0x1fc0
800192a0:	244207c0 	addiu	v0,v0,1984
800192a4:	ac620010 	sw	v0,16(v1)
800192a8:	3c11a480 	lui	s1,0xa480
800192ac:	00000000 	nop
800192b0:	8e220018 	lw	v0,24(s1)
800192b4:	30420003 	andi	v0,v0,0x3
800192b8:	1440fffd 	bnez	v0,800192b0 <joybus_exec+0xd8>
800192bc:	24050040 	li	a1,64
800192c0:	0c00644a 	jal	80019128 <data_cache_hit_writeback_invalidate>
800192c4:	02402025 	move	a0,s2
800192c8:	ae320000 	sw	s2,0(s1)
800192cc:	3c021fc0 	lui	v0,0x1fc0
800192d0:	244207c0 	addiu	v0,v0,1984
800192d4:	ae220004 	sw	v0,4(s1)
800192d8:	3c03a480 	lui	v1,0xa480
800192dc:	00000000 	nop
800192e0:	8c620018 	lw	v0,24(v1)
800192e4:	30420003 	andi	v0,v0,0x3
800192e8:	1440fffd 	bnez	v0,800192e0 <joybus_exec+0x108>
800192ec:	00000000 	nop
800192f0:	0c005eb8 	jal	80017ae0 <enable_interrupts>
800192f4:	00000000 	nop
800192f8:	3c022000 	lui	v0,0x2000
800192fc:	02429025 	or	s2,s2,v0
80019300:	de490000 	ld	t1,0(s2)
80019304:	de440028 	ld	a0,40(s2)
80019308:	de430030 	ld	v1,48(s2)
8001930c:	de420038 	ld	v0,56(s2)
80019310:	de480008 	ld	t0,8(s2)
80019314:	de470010 	ld	a3,16(s2)
80019318:	de460018 	ld	a2,24(s2)
8001931c:	de450020 	ld	a1,32(s2)
80019320:	b2090000 	sdl	t1,0(s0)
80019324:	b6090007 	sdr	t1,7(s0)
80019328:	b2080008 	sdl	t0,8(s0)
8001932c:	b608000f 	sdr	t0,15(s0)
80019330:	b2070010 	sdl	a3,16(s0)
80019334:	b6070017 	sdr	a3,23(s0)
80019338:	b2060018 	sdl	a2,24(s0)
8001933c:	b606001f 	sdr	a2,31(s0)
80019340:	b2050020 	sdl	a1,32(s0)
80019344:	b6050027 	sdr	a1,39(s0)
80019348:	b2040028 	sdl	a0,40(s0)
8001934c:	b604002f 	sdr	a0,47(s0)
80019350:	b2030030 	sdl	v1,48(s0)
80019354:	b6030037 	sdr	v1,55(s0)
80019358:	b2020038 	sdl	v0,56(s0)
8001935c:	b602003f 	sdr	v0,63(s0)
80019360:	dfbf00c8 	ld	ra,200(sp)
80019364:	dfb300c0 	ld	s3,192(sp)
80019368:	dfb200b8 	ld	s2,184(sp)
8001936c:	dfb100b0 	ld	s1,176(sp)
80019370:	dfb000a8 	ld	s0,168(sp)
80019374:	03e00008 	jr	ra
80019378:	27bd00d0 	addiu	sp,sp,208
8001937c:	00000000 	nop

80019380 <exception_default_handler>:
80019380:	27bdff48 	addiu	sp,sp,-184
80019384:	8c82000c 	lw	v0,12(a0)
80019388:	ffb00080 	sd	s0,128(sp)
8001938c:	00808025 	move	s0,a0
80019390:	8c840004 	lw	a0,4(a0)
80019394:	2403000f 	li	v1,15
80019398:	ffb30098 	sd	s3,152(sp)
8001939c:	ffb20090 	sd	s2,144(sp)
800193a0:	ffb10088 	sd	s1,136(sp)
800193a4:	8c520104 	lw	s2,260(v0)
800193a8:	ffbf00b0 	sd	ra,176(sp)
800193ac:	8c530100 	lw	s3,256(v0)
800193b0:	ffb500a8 	sd	s5,168(sp)
800193b4:	8c510120 	lw	s1,288(v0)
800193b8:	14830006 	bne	a0,v1,800193d4 <exception_default_handler+0x54>
800193bc:	ffb400a0 	sd	s4,160(sp)
800193c0:	8c430120 	lw	v1,288(v0)
800193c4:	3c04fffc 	lui	a0,0xfffc
800193c8:	24840fff 	addiu	a0,a0,4095
800193cc:	00641824 	and	v1,v1,a0
800193d0:	ac430120 	sw	v1,288(v0)
800193d4:	0c006154 	jal	80018550 <console_init>
800193d8:	00000000 	nop
800193dc:	0c0061ac 	jal	800186b0 <console_set_debug>
800193e0:	24040001 	li	a0,1
800193e4:	0c006152 	jal	80018548 <console_set_render_mode>
800193e8:	00002025 	move	a0,zero
800193ec:	8e02000c 	lw	v0,12(s0)
800193f0:	8f838028 	lw	v1,-32728(gp)
800193f4:	8c470108 	lw	a3,264(v0)
800193f8:	00121743 	sra	v0,s2,0x1d
800193fc:	8c640008 	lw	a0,8(v1)
80019400:	8e060008 	lw	a2,8(s0)
80019404:	30420004 	andi	v0,v0,0x4
80019408:	3c058002 	lui	a1,0x8002
8001940c:	00473821 	addu	a3,v0,a3
80019410:	0c000d70 	jal	800035c0 <fprintf>
80019414:	24a5c400 	addiu	a1,a1,-15360
80019418:	8f828028 	lw	v0,-32728(gp)
8001941c:	00123f02 	srl	a3,s2,0x1c
80019420:	8c440008 	lw	a0,8(v0)
80019424:	3c058002 	lui	a1,0x8002
80019428:	001217c2 	srl	v0,s2,0x1f
8001942c:	afa20024 	sw	v0,36(sp)
80019430:	30e70003 	andi	a3,a3,0x3
80019434:	02403025 	move	a2,s2
80019438:	0c000d70 	jal	800035c0 <fprintf>
8001943c:	24a5c420 	addiu	a1,a1,-15328
80019440:	40024000 	mfc0	v0,c0_badvaddr
80019444:	8f838028 	lw	v1,-32728(gp)
80019448:	3c058002 	lui	a1,0x8002
8001944c:	8c640008 	lw	a0,8(v1)
80019450:	02203825 	move	a3,s1
80019454:	02603025 	move	a2,s3
80019458:	24a5c440 	addiu	a1,a1,-15296
8001945c:	0c000d70 	jal	800035c0 <fprintf>
80019460:	afa20024 	sw	v0,36(sp)
80019464:	8f828028 	lw	v0,-32728(gp)
80019468:	3c048002 	lui	a0,0x8002
8001946c:	8c470008 	lw	a3,8(v0)
80019470:	24060040 	li	a2,64
80019474:	24050001 	li	a1,1
80019478:	0c0010a8 	jal	800042a0 <fwrite>
8001947c:	2484c468 	addiu	a0,a0,-15256
80019480:	8f828028 	lw	v0,-32728(gp)
80019484:	3c048002 	lui	a0,0x8002
80019488:	8c470008 	lw	a3,8(v0)
8001948c:	24060040 	li	a2,64
80019490:	24050001 	li	a1,1
80019494:	0c0010a8 	jal	800042a0 <fwrite>
80019498:	2484c4b0 	addiu	a0,a0,-15184
8001949c:	8f828028 	lw	v0,-32728(gp)
800194a0:	00112c03 	sra	a1,s1,0x10
800194a4:	8c440008 	lw	a0,8(v0)
800194a8:	00127bc3 	sra	t7,s2,0xf
800194ac:	00127383 	sra	t6,s2,0xe
800194b0:	00126b43 	sra	t5,s2,0xd
800194b4:	00126303 	sra	t4,s2,0xc
800194b8:	00125ac3 	sra	t3,s2,0xb
800194bc:	00125283 	sra	t2,s2,0xa
800194c0:	00124a43 	sra	t1,s2,0x9
800194c4:	00114443 	sra	t0,s1,0x11
800194c8:	00111bc3 	sra	v1,s1,0xf
800194cc:	00111383 	sra	v0,s1,0xe
800194d0:	30a50001 	andi	a1,a1,0x1
800194d4:	00129203 	sra	s2,s2,0x8
800194d8:	31ef0001 	andi	t7,t7,0x1
800194dc:	31ce0001 	andi	t6,t6,0x1
800194e0:	31ad0001 	andi	t5,t5,0x1
800194e4:	318c0001 	andi	t4,t4,0x1
800194e8:	316b0001 	andi	t3,t3,0x1
800194ec:	314a0001 	andi	t2,t2,0x1
800194f0:	31290001 	andi	t1,t1,0x1
800194f4:	31080001 	andi	t0,t0,0x1
800194f8:	30630001 	andi	v1,v1,0x1
800194fc:	32520001 	andi	s2,s2,0x1
80019500:	30420001 	andi	v0,v0,0x1
80019504:	00113b43 	sra	a3,s1,0xd
80019508:	00113303 	sra	a2,s1,0xc
8001950c:	afa50034 	sw	a1,52(sp)
80019510:	3c058002 	lui	a1,0x8002
80019514:	afaf007c 	sw	t7,124(sp)
80019518:	afae0074 	sw	t6,116(sp)
8001951c:	afad006c 	sw	t5,108(sp)
80019520:	afac0064 	sw	t4,100(sp)
80019524:	afab005c 	sw	t3,92(sp)
80019528:	afaa0054 	sw	t2,84(sp)
8001952c:	afa9004c 	sw	t1,76(sp)
80019530:	afb20044 	sw	s2,68(sp)
80019534:	afa8003c 	sw	t0,60(sp)
80019538:	afa3002c 	sw	v1,44(sp)
8001953c:	30e70001 	andi	a3,a3,0x1
80019540:	30c60001 	andi	a2,a2,0x1
80019544:	24a5c4f8 	addiu	a1,a1,-15112
80019548:	0c000d70 	jal	800035c0 <fprintf>
8001954c:	afa20024 	sw	v0,36(sp)
80019550:	8f828028 	lw	v0,-32728(gp)
80019554:	00112ac3 	sra	a1,s1,0xb
80019558:	8c440008 	lw	a0,8(v0)
8001955c:	001373c3 	sra	t6,s3,0xf
80019560:	00136b83 	sra	t5,s3,0xe
80019564:	00136343 	sra	t4,s3,0xd
80019568:	00135b03 	sra	t3,s3,0xc
8001956c:	001352c3 	sra	t2,s3,0xb
80019570:	00134a83 	sra	t1,s3,0xa
80019574:	00134243 	sra	t0,s3,0x9
80019578:	00111a83 	sra	v1,s1,0xa
8001957c:	00111243 	sra	v0,s1,0x9
80019580:	30a50001 	andi	a1,a1,0x1
80019584:	00139a03 	sra	s3,s3,0x8
80019588:	31ce0001 	andi	t6,t6,0x1
8001958c:	31ad0001 	andi	t5,t5,0x1
80019590:	318c0001 	andi	t4,t4,0x1
80019594:	316b0001 	andi	t3,t3,0x1
80019598:	314a0001 	andi	t2,t2,0x1
8001959c:	31290001 	andi	t1,t1,0x1
800195a0:	31080001 	andi	t0,t0,0x1
800195a4:	30630001 	andi	v1,v1,0x1
800195a8:	32730001 	andi	s3,s3,0x1
800195ac:	30420001 	andi	v0,v0,0x1
800195b0:	00113a03 	sra	a3,s1,0x8
800195b4:	001131c3 	sra	a2,s1,0x7
800195b8:	afa50034 	sw	a1,52(sp)
800195bc:	3c058002 	lui	a1,0x8002
800195c0:	afae0074 	sw	t6,116(sp)
800195c4:	afad006c 	sw	t5,108(sp)
800195c8:	afac0064 	sw	t4,100(sp)
800195cc:	afab005c 	sw	t3,92(sp)
800195d0:	afaa0054 	sw	t2,84(sp)
800195d4:	afa9004c 	sw	t1,76(sp)
800195d8:	afa80044 	sw	t0,68(sp)
800195dc:	afb3003c 	sw	s3,60(sp)
800195e0:	afa3002c 	sw	v1,44(sp)
800195e4:	30e70001 	andi	a3,a3,0x1
800195e8:	30c60001 	andi	a2,a2,0x1
800195ec:	24a5c540 	addiu	a1,a1,-15040
800195f0:	0c000d70 	jal	800035c0 <fprintf>
800195f4:	afa20024 	sw	v0,36(sp)
800195f8:	8f828028 	lw	v0,-32728(gp)
800195fc:	00112983 	sra	a1,s1,0x6
80019600:	8c440008 	lw	a0,8(v0)
80019604:	00111943 	sra	v1,s1,0x5
80019608:	00111103 	sra	v0,s1,0x4
8001960c:	30a50001 	andi	a1,a1,0x1
80019610:	30630001 	andi	v1,v1,0x1
80019614:	30420001 	andi	v0,v0,0x1
80019618:	001138c3 	sra	a3,s1,0x3
8001961c:	00113083 	sra	a2,s1,0x2
80019620:	afa50034 	sw	a1,52(sp)
80019624:	3c058002 	lui	a1,0x8002
80019628:	afa3002c 	sw	v1,44(sp)
8001962c:	30e70001 	andi	a3,a3,0x1
80019630:	30c60001 	andi	a2,a2,0x1
80019634:	24a5c588 	addiu	a1,a1,-14968
80019638:	0c000d70 	jal	800035c0 <fprintf>
8001963c:	afa20024 	sw	v0,36(sp)
80019640:	8f828028 	lw	v0,-32728(gp)
80019644:	3c048002 	lui	a0,0x8002
80019648:	8c470008 	lw	a3,8(v0)
8001964c:	24060040 	li	a2,64
80019650:	24050001 	li	a1,1
80019654:	0c0010a8 	jal	800042a0 <fwrite>
80019658:	2484c5a8 	addiu	a0,a0,-14936
8001965c:	8e02000c 	lw	v0,12(s0)
80019660:	3c058002 	lui	a1,0x8002
80019664:	dc460000 	ld	a2,0(v0)
80019668:	8f828028 	lw	v0,-32728(gp)
8001966c:	00063000 	sll	a2,a2,0x0
80019670:	8c440008 	lw	a0,8(v0)
80019674:	0c000d70 	jal	800035c0 <fprintf>
80019678:	24a5c5f0 	addiu	a1,a1,-14864
8001967c:	8e02000c 	lw	v0,12(s0)
80019680:	3c058002 	lui	a1,0x8002
80019684:	dc460008 	ld	a2,8(v0)
80019688:	8f828028 	lw	v0,-32728(gp)
8001968c:	00063000 	sll	a2,a2,0x0
80019690:	8c440008 	lw	a0,8(v0)
80019694:	0c000d70 	jal	800035c0 <fprintf>
80019698:	24a5c600 	addiu	a1,a1,-14848
8001969c:	8e02000c 	lw	v0,12(s0)
800196a0:	3c058002 	lui	a1,0x8002
800196a4:	dc460010 	ld	a2,16(v0)
800196a8:	8f828028 	lw	v0,-32728(gp)
800196ac:	00063000 	sll	a2,a2,0x0
800196b0:	8c440008 	lw	a0,8(v0)
800196b4:	0c000d70 	jal	800035c0 <fprintf>
800196b8:	24a5c610 	addiu	a1,a1,-14832
800196bc:	8e02000c 	lw	v0,12(s0)
800196c0:	3c058002 	lui	a1,0x8002
800196c4:	dc460018 	ld	a2,24(v0)
800196c8:	8f828028 	lw	v0,-32728(gp)
800196cc:	00063000 	sll	a2,a2,0x0
800196d0:	8c440008 	lw	a0,8(v0)
800196d4:	0c000d70 	jal	800035c0 <fprintf>
800196d8:	24a5c620 	addiu	a1,a1,-14816
800196dc:	8e02000c 	lw	v0,12(s0)
800196e0:	3c058002 	lui	a1,0x8002
800196e4:	dc460020 	ld	a2,32(v0)
800196e8:	8f828028 	lw	v0,-32728(gp)
800196ec:	00063000 	sll	a2,a2,0x0
800196f0:	8c440008 	lw	a0,8(v0)
800196f4:	0c000d70 	jal	800035c0 <fprintf>
800196f8:	24a5c630 	addiu	a1,a1,-14800
800196fc:	8e02000c 	lw	v0,12(s0)
80019700:	3c058002 	lui	a1,0x8002
80019704:	dc460028 	ld	a2,40(v0)
80019708:	8f828028 	lw	v0,-32728(gp)
8001970c:	00063000 	sll	a2,a2,0x0
80019710:	8c440008 	lw	a0,8(v0)
80019714:	0c000d70 	jal	800035c0 <fprintf>
80019718:	24a5c640 	addiu	a1,a1,-14784
8001971c:	8e02000c 	lw	v0,12(s0)
80019720:	3c058002 	lui	a1,0x8002
80019724:	dc460030 	ld	a2,48(v0)
80019728:	8f828028 	lw	v0,-32728(gp)
8001972c:	00063000 	sll	a2,a2,0x0
80019730:	8c440008 	lw	a0,8(v0)
80019734:	0c000d70 	jal	800035c0 <fprintf>
80019738:	24a5c650 	addiu	a1,a1,-14768
8001973c:	8e02000c 	lw	v0,12(s0)
80019740:	3c058002 	lui	a1,0x8002
80019744:	dc460038 	ld	a2,56(v0)
80019748:	8f828028 	lw	v0,-32728(gp)
8001974c:	00063000 	sll	a2,a2,0x0
80019750:	8c440008 	lw	a0,8(v0)
80019754:	0c000d70 	jal	800035c0 <fprintf>
80019758:	24a5c660 	addiu	a1,a1,-14752
8001975c:	8e02000c 	lw	v0,12(s0)
80019760:	3c058002 	lui	a1,0x8002
80019764:	dc460040 	ld	a2,64(v0)
80019768:	8f828028 	lw	v0,-32728(gp)
8001976c:	00063000 	sll	a2,a2,0x0
80019770:	8c440008 	lw	a0,8(v0)
80019774:	0c000d70 	jal	800035c0 <fprintf>
80019778:	24a5c670 	addiu	a1,a1,-14736
8001977c:	8e02000c 	lw	v0,12(s0)
80019780:	3c058002 	lui	a1,0x8002
80019784:	dc460048 	ld	a2,72(v0)
80019788:	8f828028 	lw	v0,-32728(gp)
8001978c:	00063000 	sll	a2,a2,0x0
80019790:	8c440008 	lw	a0,8(v0)
80019794:	0c000d70 	jal	800035c0 <fprintf>
80019798:	24a5c680 	addiu	a1,a1,-14720
8001979c:	8e02000c 	lw	v0,12(s0)
800197a0:	3c058002 	lui	a1,0x8002
800197a4:	dc460050 	ld	a2,80(v0)
800197a8:	8f828028 	lw	v0,-32728(gp)
800197ac:	00063000 	sll	a2,a2,0x0
800197b0:	8c440008 	lw	a0,8(v0)
800197b4:	0c000d70 	jal	800035c0 <fprintf>
800197b8:	24a5c690 	addiu	a1,a1,-14704
800197bc:	8e02000c 	lw	v0,12(s0)
800197c0:	3c058002 	lui	a1,0x8002
800197c4:	dc460058 	ld	a2,88(v0)
800197c8:	8f828028 	lw	v0,-32728(gp)
800197cc:	00063000 	sll	a2,a2,0x0
800197d0:	8c440008 	lw	a0,8(v0)
800197d4:	0c000d70 	jal	800035c0 <fprintf>
800197d8:	24a5c6a0 	addiu	a1,a1,-14688
800197dc:	8e02000c 	lw	v0,12(s0)
800197e0:	3c058002 	lui	a1,0x8002
800197e4:	dc460060 	ld	a2,96(v0)
800197e8:	8f828028 	lw	v0,-32728(gp)
800197ec:	00063000 	sll	a2,a2,0x0
800197f0:	8c440008 	lw	a0,8(v0)
800197f4:	0c000d70 	jal	800035c0 <fprintf>
800197f8:	24a5c6b0 	addiu	a1,a1,-14672
800197fc:	8e02000c 	lw	v0,12(s0)
80019800:	3c058002 	lui	a1,0x8002
80019804:	dc460068 	ld	a2,104(v0)
80019808:	8f828028 	lw	v0,-32728(gp)
8001980c:	00063000 	sll	a2,a2,0x0
80019810:	8c440008 	lw	a0,8(v0)
80019814:	0c000d70 	jal	800035c0 <fprintf>
80019818:	24a5c6c0 	addiu	a1,a1,-14656
8001981c:	8e02000c 	lw	v0,12(s0)
80019820:	3c058002 	lui	a1,0x8002
80019824:	dc460070 	ld	a2,112(v0)
80019828:	8f828028 	lw	v0,-32728(gp)
8001982c:	00063000 	sll	a2,a2,0x0
80019830:	8c440008 	lw	a0,8(v0)
80019834:	0c000d70 	jal	800035c0 <fprintf>
80019838:	24a5c6d0 	addiu	a1,a1,-14640
8001983c:	8e02000c 	lw	v0,12(s0)
80019840:	3c058002 	lui	a1,0x8002
80019844:	dc460078 	ld	a2,120(v0)
80019848:	8f828028 	lw	v0,-32728(gp)
8001984c:	00063000 	sll	a2,a2,0x0
80019850:	8c440008 	lw	a0,8(v0)
80019854:	0c000d70 	jal	800035c0 <fprintf>
80019858:	24a5c6e0 	addiu	a1,a1,-14624
8001985c:	8e02000c 	lw	v0,12(s0)
80019860:	3c058002 	lui	a1,0x8002
80019864:	dc4600c0 	ld	a2,192(v0)
80019868:	8f828028 	lw	v0,-32728(gp)
8001986c:	00063000 	sll	a2,a2,0x0
80019870:	8c440008 	lw	a0,8(v0)
80019874:	0c000d70 	jal	800035c0 <fprintf>
80019878:	24a5c6f0 	addiu	a1,a1,-14608
8001987c:	8e02000c 	lw	v0,12(s0)
80019880:	3c058002 	lui	a1,0x8002
80019884:	dc4600c8 	ld	a2,200(v0)
80019888:	8f828028 	lw	v0,-32728(gp)
8001988c:	00063000 	sll	a2,a2,0x0
80019890:	8c440008 	lw	a0,8(v0)
80019894:	0c000d70 	jal	800035c0 <fprintf>
80019898:	24a5c700 	addiu	a1,a1,-14592
8001989c:	8e02000c 	lw	v0,12(s0)
800198a0:	3c058002 	lui	a1,0x8002
800198a4:	dc460080 	ld	a2,128(v0)
800198a8:	8f828028 	lw	v0,-32728(gp)
800198ac:	00063000 	sll	a2,a2,0x0
800198b0:	8c440008 	lw	a0,8(v0)
800198b4:	0c000d70 	jal	800035c0 <fprintf>
800198b8:	24a5c710 	addiu	a1,a1,-14576
800198bc:	8e02000c 	lw	v0,12(s0)
800198c0:	3c058002 	lui	a1,0x8002
800198c4:	dc460088 	ld	a2,136(v0)
800198c8:	8f828028 	lw	v0,-32728(gp)
800198cc:	00063000 	sll	a2,a2,0x0
800198d0:	8c440008 	lw	a0,8(v0)
800198d4:	0c000d70 	jal	800035c0 <fprintf>
800198d8:	24a5c720 	addiu	a1,a1,-14560
800198dc:	8e02000c 	lw	v0,12(s0)
800198e0:	3c058002 	lui	a1,0x8002
800198e4:	dc460090 	ld	a2,144(v0)
800198e8:	8f828028 	lw	v0,-32728(gp)
800198ec:	00063000 	sll	a2,a2,0x0
800198f0:	8c440008 	lw	a0,8(v0)
800198f4:	0c000d70 	jal	800035c0 <fprintf>
800198f8:	24a5c730 	addiu	a1,a1,-14544
800198fc:	8e02000c 	lw	v0,12(s0)
80019900:	3c058002 	lui	a1,0x8002
80019904:	dc460098 	ld	a2,152(v0)
80019908:	8f828028 	lw	v0,-32728(gp)
8001990c:	00063000 	sll	a2,a2,0x0
80019910:	8c440008 	lw	a0,8(v0)
80019914:	0c000d70 	jal	800035c0 <fprintf>
80019918:	24a5c740 	addiu	a1,a1,-14528
8001991c:	8e02000c 	lw	v0,12(s0)
80019920:	3c058002 	lui	a1,0x8002
80019924:	dc4600a0 	ld	a2,160(v0)
80019928:	8f828028 	lw	v0,-32728(gp)
8001992c:	00063000 	sll	a2,a2,0x0
80019930:	8c440008 	lw	a0,8(v0)
80019934:	0c000d70 	jal	800035c0 <fprintf>
80019938:	24a5c750 	addiu	a1,a1,-14512
8001993c:	8e02000c 	lw	v0,12(s0)
80019940:	3c058002 	lui	a1,0x8002
80019944:	dc4600a8 	ld	a2,168(v0)
80019948:	8f828028 	lw	v0,-32728(gp)
8001994c:	00063000 	sll	a2,a2,0x0
80019950:	8c440008 	lw	a0,8(v0)
80019954:	0c000d70 	jal	800035c0 <fprintf>
80019958:	24a5c760 	addiu	a1,a1,-14496
8001995c:	8e02000c 	lw	v0,12(s0)
80019960:	3c058002 	lui	a1,0x8002
80019964:	dc4600b0 	ld	a2,176(v0)
80019968:	8f828028 	lw	v0,-32728(gp)
8001996c:	00063000 	sll	a2,a2,0x0
80019970:	8c440008 	lw	a0,8(v0)
80019974:	0c000d70 	jal	800035c0 <fprintf>
80019978:	24a5c770 	addiu	a1,a1,-14480
8001997c:	8e02000c 	lw	v0,12(s0)
80019980:	3c058002 	lui	a1,0x8002
80019984:	dc4600b8 	ld	a2,184(v0)
80019988:	8f828028 	lw	v0,-32728(gp)
8001998c:	00063000 	sll	a2,a2,0x0
80019990:	8c440008 	lw	a0,8(v0)
80019994:	0c000d70 	jal	800035c0 <fprintf>
80019998:	24a5c780 	addiu	a1,a1,-14464
8001999c:	8e02000c 	lw	v0,12(s0)
800199a0:	3c058002 	lui	a1,0x8002
800199a4:	dc4600e0 	ld	a2,224(v0)
800199a8:	8f828028 	lw	v0,-32728(gp)
800199ac:	00063000 	sll	a2,a2,0x0
800199b0:	8c440008 	lw	a0,8(v0)
800199b4:	0c000d70 	jal	800035c0 <fprintf>
800199b8:	24a5c790 	addiu	a1,a1,-14448
800199bc:	8e02000c 	lw	v0,12(s0)
800199c0:	3c058002 	lui	a1,0x8002
800199c4:	dc4600e8 	ld	a2,232(v0)
800199c8:	8f828028 	lw	v0,-32728(gp)
800199cc:	00063000 	sll	a2,a2,0x0
800199d0:	8c440008 	lw	a0,8(v0)
800199d4:	0c000d70 	jal	800035c0 <fprintf>
800199d8:	24a5c7a0 	addiu	a1,a1,-14432
800199dc:	8e02000c 	lw	v0,12(s0)
800199e0:	3c058002 	lui	a1,0x8002
800199e4:	dc4600f0 	ld	a2,240(v0)
800199e8:	8f828028 	lw	v0,-32728(gp)
800199ec:	00063000 	sll	a2,a2,0x0
800199f0:	8c440008 	lw	a0,8(v0)
800199f4:	0c000d70 	jal	800035c0 <fprintf>
800199f8:	24a5c7b0 	addiu	a1,a1,-14416
800199fc:	8e02000c 	lw	v0,12(s0)
80019a00:	3c058002 	lui	a1,0x8002
80019a04:	dc4600f8 	ld	a2,248(v0)
80019a08:	8f828028 	lw	v0,-32728(gp)
80019a0c:	00063000 	sll	a2,a2,0x0
80019a10:	8c440008 	lw	a0,8(v0)
80019a14:	0c000d70 	jal	800035c0 <fprintf>
80019a18:	24a5c7c0 	addiu	a1,a1,-14400
80019a1c:	8e02000c 	lw	v0,12(s0)
80019a20:	3c058002 	lui	a1,0x8002
80019a24:	dc460118 	ld	a2,280(v0)
80019a28:	8f828028 	lw	v0,-32728(gp)
80019a2c:	24a5c7d0 	addiu	a1,a1,-14384
80019a30:	8c440008 	lw	a0,8(v0)
80019a34:	0c000d70 	jal	800035c0 <fprintf>
80019a38:	3c138002 	lui	s3,0x8002
80019a3c:	8e02000c 	lw	v0,12(s0)
80019a40:	3c058002 	lui	a1,0x8002
80019a44:	dc460110 	ld	a2,272(v0)
80019a48:	8f828028 	lw	v0,-32728(gp)
80019a4c:	24a5c7e0 	addiu	a1,a1,-14368
80019a50:	8c440008 	lw	a0,8(v0)
80019a54:	0c000d70 	jal	800035c0 <fprintf>
80019a58:	3c12aaaa 	lui	s2,0xaaaa
80019a5c:	8f828028 	lw	v0,-32728(gp)
80019a60:	3c048002 	lui	a0,0x8002
80019a64:	8c470008 	lw	a3,8(v0)
80019a68:	24060040 	li	a2,64
80019a6c:	24050001 	li	a1,1
80019a70:	0c0010a8 	jal	800042a0 <fwrite>
80019a74:	2484c7f0 	addiu	a0,a0,-14352
80019a78:	8e02000c 	lw	v0,12(s0)
80019a7c:	00003025 	move	a2,zero
80019a80:	dc470128 	ld	a3,296(v0)
80019a84:	8f828028 	lw	v0,-32728(gp)
80019a88:	2665c838 	addiu	a1,s3,-14280
80019a8c:	8c440008 	lw	a0,8(v0)
80019a90:	0c000d70 	jal	800035c0 <fprintf>
80019a94:	3c115555 	lui	s1,0x5555
80019a98:	0000a825 	move	s5,zero
80019a9c:	24140020 	li	s4,32
80019aa0:	3652aaaa 	ori	s2,s2,0xaaaa
80019aa4:	26315555 	addiu	s1,s1,21845
80019aa8:	26b50001 	addiu	s5,s5,1
80019aac:	00000000 	nop
80019ab0:	26a20025 	addiu	v0,s5,37
80019ab4:	8e03000c 	lw	v1,12(s0)
80019ab8:	2665c838 	addiu	a1,s3,-14280
80019abc:	000210c0 	sll	v0,v0,0x3
80019ac0:	12b4001c 	beq	s5,s4,80019b34 <exception_default_handler+0x7b4>
80019ac4:	02a03025 	move	a2,s5
80019ac8:	00621021 	addu	v0,v1,v0
80019acc:	dc470000 	ld	a3,0(v0)
80019ad0:	8f828028 	lw	v0,-32728(gp)
80019ad4:	0c000d70 	jal	800035c0 <fprintf>
80019ad8:	8c440008 	lw	a0,8(v0)
80019adc:	00151080 	sll	v0,s5,0x2
80019ae0:	00551021 	addu	v0,v0,s5
80019ae4:	00021900 	sll	v1,v0,0x4
80019ae8:	00431021 	addu	v0,v0,v1
80019aec:	00021a00 	sll	v1,v0,0x8
80019af0:	00431021 	addu	v0,v0,v1
80019af4:	00021c00 	sll	v1,v0,0x10
80019af8:	00431021 	addu	v0,v0,v1
80019afc:	02421023 	subu	v0,s2,v0
80019b00:	0051102b 	sltu	v0,v0,s1
80019b04:	1040ffea 	beqz	v0,80019ab0 <exception_default_handler+0x730>
80019b08:	26b50001 	addiu	s5,s5,1
80019b0c:	8f828028 	lw	v0,-32728(gp)
80019b10:	8c450008 	lw	a1,8(v0)
80019b14:	0c000daa 	jal	800036a8 <fputc>
80019b18:	2404000a 	li	a0,10
80019b1c:	26a20025 	addiu	v0,s5,37
80019b20:	8e03000c 	lw	v1,12(s0)
80019b24:	2665c838 	addiu	a1,s3,-14280
80019b28:	000210c0 	sll	v0,v0,0x3
80019b2c:	16b4ffe6 	bne	s5,s4,80019ac8 <exception_default_handler+0x748>
80019b30:	02a03025 	move	a2,s5
80019b34:	dfbf00b0 	ld	ra,176(sp)
80019b38:	3c028002 	lui	v0,0x8002
80019b3c:	24429b68 	addiu	v0,v0,-25752
80019b40:	dfb500a8 	ld	s5,168(sp)
80019b44:	dfb400a0 	ld	s4,160(sp)
80019b48:	dfb30098 	ld	s3,152(sp)
80019b4c:	dfb20090 	ld	s2,144(sp)
80019b50:	dfb10088 	ld	s1,136(sp)
80019b54:	dfb00080 	ld	s0,128(sp)
80019b58:	ac620108 	sw	v0,264(v1)
80019b5c:	03e00008 	jr	ra
80019b60:	27bd00b8 	addiu	sp,sp,184
80019b64:	00000000 	nop

80019b68 <exception_halt>:
80019b68:	27bdffd8 	addiu	sp,sp,-40
80019b6c:	ffbf0020 	sd	ra,32(sp)
80019b70:	0c00619e 	jal	80018678 <console_render>
80019b74:	00000000 	nop
80019b78:	0c000a2f 	jal	800028bc <abort>
80019b7c:	00000000 	nop

80019b80 <__onCriticalException>:
80019b80:	8f8580dc 	lw	a1,-32548(gp)
80019b84:	27bdffc8 	addiu	sp,sp,-56
80019b88:	10a00011 	beqz	a1,80019bd0 <__onCriticalException+0x50>
80019b8c:	ffbf0030 	sd	ra,48(sp)
80019b90:	27868100 	addiu	a2,gp,-32512
80019b94:	8cc20104 	lw	v0,260(a2)
80019b98:	3c048002 	lui	a0,0x8002
80019b9c:	00021082 	srl	v0,v0,0x2
80019ba0:	3042001f 	andi	v0,v0,0x1f
80019ba4:	2484c9c8 	addiu	a0,a0,-13880
80019ba8:	00021880 	sll	v1,v0,0x2
80019bac:	00641821 	addu	v1,v1,a0
80019bb0:	8c630000 	lw	v1,0(v1)
80019bb4:	24040002 	li	a0,2
80019bb8:	afa40020 	sw	a0,32(sp)
80019bbc:	afa6002c 	sw	a2,44(sp)
80019bc0:	afa20024 	sw	v0,36(sp)
80019bc4:	afa30028 	sw	v1,40(sp)
80019bc8:	00a0f809 	jalr	a1
80019bcc:	27a40020 	addiu	a0,sp,32
80019bd0:	dfbf0030 	ld	ra,48(sp)
80019bd4:	03e00008 	jr	ra
80019bd8:	27bd0038 	addiu	sp,sp,56
80019bdc:	00000000 	nop

80019be0 <__onResetException>:
80019be0:	8f8580dc 	lw	a1,-32548(gp)
80019be4:	27bdffc8 	addiu	sp,sp,-56
80019be8:	10a00011 	beqz	a1,80019c30 <__onResetException+0x50>
80019bec:	ffbf0030 	sd	ra,48(sp)
80019bf0:	27868100 	addiu	a2,gp,-32512
80019bf4:	8cc20104 	lw	v0,260(a2)
80019bf8:	3c048002 	lui	a0,0x8002
80019bfc:	00021082 	srl	v0,v0,0x2
80019c00:	3042001f 	andi	v0,v0,0x1f
80019c04:	2484c9c8 	addiu	a0,a0,-13880
80019c08:	00021880 	sll	v1,v0,0x2
80019c0c:	00641821 	addu	v1,v1,a0
80019c10:	8c630000 	lw	v1,0(v1)
80019c14:	24040001 	li	a0,1
80019c18:	afa40020 	sw	a0,32(sp)
80019c1c:	afa6002c 	sw	a2,44(sp)
80019c20:	afa20024 	sw	v0,36(sp)
80019c24:	afa30028 	sw	v1,40(sp)
80019c28:	00a0f809 	jalr	a1
80019c2c:	27a40020 	addiu	a0,sp,32
80019c30:	dfbf0030 	ld	ra,48(sp)
80019c34:	03e00008 	jr	ra
80019c38:	27bd0038 	addiu	sp,sp,56
80019c3c:	00000000 	nop

80019c40 <close>:
80019c40:	18800044 	blez	a0,80019d54 <close+0x114>
80019c44:	00802825 	move	a1,a0
80019c48:	3c028003 	lui	v0,0x8003
80019c4c:	2442de00 	addiu	v0,v0,-8704
80019c50:	00401825 	move	v1,v0
80019c54:	00002025 	move	a0,zero
80019c58:	10000004 	b	80019c6c <close+0x2c>
80019c5c:	24070064 	li	a3,100
80019c60:	24840001 	addiu	a0,a0,1
80019c64:	50870010 	beql	a0,a3,80019ca8 <close+0x68>
80019c68:	00003825 	move	a3,zero
80019c6c:	8c660000 	lw	a2,0(v1)
80019c70:	14a6fffb 	bne	a1,a2,80019c60 <close+0x20>
80019c74:	2463000c 	addiu	v1,v1,12
80019c78:	00041840 	sll	v1,a0,0x1
80019c7c:	00641821 	addu	v1,v1,a0
80019c80:	3c048003 	lui	a0,0x8003
80019c84:	2484ddf8 	addiu	a0,a0,-8712
80019c88:	00031880 	sll	v1,v1,0x2
80019c8c:	00641821 	addu	v1,v1,a0
80019c90:	8c640000 	lw	a0,0(v1)
80019c94:	3c038003 	lui	v1,0x8003
80019c98:	000420c0 	sll	a0,a0,0x3
80019c9c:	2463e2a8 	addiu	v1,v1,-7512
80019ca0:	00641821 	addu	v1,v1,a0
80019ca4:	8c670004 	lw	a3,4(v1)
80019ca8:	00001825 	move	v1,zero
80019cac:	10000005 	b	80019cc4 <close+0x84>
80019cb0:	24060064 	li	a2,100
80019cb4:	00000000 	nop
80019cb8:	24630001 	addiu	v1,v1,1
80019cbc:	1066000b 	beq	v1,a2,80019cec <close+0xac>
80019cc0:	00002025 	move	a0,zero
80019cc4:	8c440000 	lw	a0,0(v0)
80019cc8:	14a4fffb 	bne	a1,a0,80019cb8 <close+0x78>
80019ccc:	2442000c 	addiu	v0,v0,12
80019cd0:	00031040 	sll	v0,v1,0x1
80019cd4:	00431821 	addu	v1,v0,v1
80019cd8:	3c028003 	lui	v0,0x8003
80019cdc:	00031880 	sll	v1,v1,0x2
80019ce0:	2442ddf8 	addiu	v0,v0,-8712
80019ce4:	00431021 	addu	v0,v0,v1
80019ce8:	8c440004 	lw	a0,4(v0)
80019cec:	10e0001a 	beqz	a3,80019d58 <close+0x118>
80019cf0:	3c028002 	lui	v0,0x8002
80019cf4:	3c028003 	lui	v0,0x8003
80019cf8:	2442ddf8 	addiu	v0,v0,-8712
80019cfc:	244604b0 	addiu	a2,v0,1200
80019d00:	8c430008 	lw	v1,8(v0)
80019d04:	00000000 	nop
80019d08:	54650005 	bnel	v1,a1,80019d20 <close+0xe0>
80019d0c:	2442000c 	addiu	v0,v0,12
80019d10:	ac400000 	sw	zero,0(v0)
80019d14:	ac400004 	sw	zero,4(v0)
80019d18:	ac400008 	sw	zero,8(v0)
80019d1c:	2442000c 	addiu	v0,v0,12
80019d20:	5446fff9 	bnel	v0,a2,80019d08 <close+0xc8>
80019d24:	8c430008 	lw	v1,8(v0)
80019d28:	8cf90014 	lw	t9,20(a3)
80019d2c:	13200005 	beqz	t9,80019d44 <close+0x104>
80019d30:	3c028002 	lui	v0,0x8002
80019d34:	03200008 	jr	t9
80019d38:	00000000 	nop
80019d3c:	1000ffeb 	b	80019cec <close+0xac>
80019d40:	00002025 	move	a0,zero
80019d44:	24030058 	li	v1,88
80019d48:	ac43d79c 	sw	v1,-10340(v0)
80019d4c:	03e00008 	jr	ra
80019d50:	2402ffff 	li	v0,-1
80019d54:	3c028002 	lui	v0,0x8002
80019d58:	24030016 	li	v1,22
80019d5c:	1000fffb 	b	80019d4c <close+0x10c>
80019d60:	ac43d79c 	sw	v1,-10340(v0)
80019d64:	00000000 	nop

80019d68 <_exit>:
80019d68:	1000ffff 	b	80019d68 <_exit>
80019d6c:	00000000 	nop

80019d70 <fstat>:
80019d70:	10a00035 	beqz	a1,80019e48 <fstat+0xd8>
80019d74:	3c028002 	lui	v0,0x8002
80019d78:	18800033 	blez	a0,80019e48 <fstat+0xd8>
80019d7c:	00003025 	move	a2,zero
80019d80:	3c028003 	lui	v0,0x8003
80019d84:	2442de00 	addiu	v0,v0,-8704
80019d88:	00401825 	move	v1,v0
80019d8c:	10000005 	b	80019da4 <fstat+0x34>
80019d90:	24080064 	li	t0,100
80019d94:	00000000 	nop
80019d98:	24c60001 	addiu	a2,a2,1
80019d9c:	50c80010 	beql	a2,t0,80019de0 <fstat+0x70>
80019da0:	00004025 	move	t0,zero
80019da4:	8c670000 	lw	a3,0(v1)
80019da8:	1487fffb 	bne	a0,a3,80019d98 <fstat+0x28>
80019dac:	2463000c 	addiu	v1,v1,12
80019db0:	00061840 	sll	v1,a2,0x1
80019db4:	00661821 	addu	v1,v1,a2
80019db8:	3c068003 	lui	a2,0x8003
80019dbc:	24c6ddf8 	addiu	a2,a2,-8712
80019dc0:	00031880 	sll	v1,v1,0x2
80019dc4:	00661821 	addu	v1,v1,a2
80019dc8:	8c660000 	lw	a2,0(v1)
80019dcc:	3c038003 	lui	v1,0x8003
80019dd0:	000630c0 	sll	a2,a2,0x3
80019dd4:	2463e2a8 	addiu	v1,v1,-7512
80019dd8:	00661821 	addu	v1,v1,a2
80019ddc:	8c680004 	lw	t0,4(v1)
80019de0:	00001825 	move	v1,zero
80019de4:	10000005 	b	80019dfc <fstat+0x8c>
80019de8:	24070064 	li	a3,100
80019dec:	00000000 	nop
80019df0:	24630001 	addiu	v1,v1,1
80019df4:	5067000b 	beql	v1,a3,80019e24 <fstat+0xb4>
80019df8:	00002025 	move	a0,zero
80019dfc:	8c460000 	lw	a2,0(v0)
80019e00:	1486fffb 	bne	a0,a2,80019df0 <fstat+0x80>
80019e04:	2442000c 	addiu	v0,v0,12
80019e08:	00031040 	sll	v0,v1,0x1
80019e0c:	00431821 	addu	v1,v0,v1
80019e10:	3c028003 	lui	v0,0x8003
80019e14:	00031880 	sll	v1,v1,0x2
80019e18:	2442ddf8 	addiu	v0,v0,-8712
80019e1c:	00431021 	addu	v0,v0,v1
80019e20:	8c440004 	lw	a0,4(v0)
80019e24:	11000008 	beqz	t0,80019e48 <fstat+0xd8>
80019e28:	3c028002 	lui	v0,0x8002
80019e2c:	8d190004 	lw	t9,4(t0)
80019e30:	13200009 	beqz	t9,80019e58 <fstat+0xe8>
80019e34:	24030058 	li	v1,88
80019e38:	03200008 	jr	t9
80019e3c:	00000000 	nop
80019e40:	1000fff8 	b	80019e24 <fstat+0xb4>
80019e44:	00002025 	move	a0,zero
80019e48:	24030016 	li	v1,22
80019e4c:	ac43d79c 	sw	v1,-10340(v0)
80019e50:	03e00008 	jr	ra
80019e54:	2402ffff 	li	v0,-1
80019e58:	1000fffd 	b	80019e50 <fstat+0xe0>
80019e5c:	ac43d79c 	sw	v1,-10340(v0)

80019e60 <getpid>:
80019e60:	3c028002 	lui	v0,0x8002
80019e64:	24030058 	li	v1,88
80019e68:	ac43d79c 	sw	v1,-10340(v0)
80019e6c:	03e00008 	jr	ra
80019e70:	2402ffff 	li	v0,-1
80019e74:	00000000 	nop

80019e78 <isatty>:
80019e78:	2c840003 	sltiu	a0,a0,3
80019e7c:	14800005 	bnez	a0,80019e94 <isatty+0x1c>
80019e80:	24020001 	li	v0,1
80019e84:	3c028002 	lui	v0,0x8002
80019e88:	24030009 	li	v1,9
80019e8c:	ac43d79c 	sw	v1,-10340(v0)
80019e90:	00001025 	move	v0,zero
80019e94:	03e00008 	jr	ra
80019e98:	00000000 	nop
80019e9c:	00000000 	nop

80019ea0 <kill>:
80019ea0:	3c028002 	lui	v0,0x8002
80019ea4:	24030058 	li	v1,88
80019ea8:	ac43d79c 	sw	v1,-10340(v0)
80019eac:	03e00008 	jr	ra
80019eb0:	2402ffff 	li	v0,-1
80019eb4:	00000000 	nop

80019eb8 <lseek>:
80019eb8:	18800036 	blez	a0,80019f94 <lseek+0xdc>
80019ebc:	00003825 	move	a3,zero
80019ec0:	3c028003 	lui	v0,0x8003
80019ec4:	2442de00 	addiu	v0,v0,-8704
80019ec8:	00401825 	move	v1,v0
80019ecc:	10000005 	b	80019ee4 <lseek+0x2c>
80019ed0:	24090064 	li	t1,100
80019ed4:	00000000 	nop
80019ed8:	24e70001 	addiu	a3,a3,1
80019edc:	50e90010 	beql	a3,t1,80019f20 <lseek+0x68>
80019ee0:	00004825 	move	t1,zero
80019ee4:	8c680000 	lw	t0,0(v1)
80019ee8:	1488fffb 	bne	a0,t0,80019ed8 <lseek+0x20>
80019eec:	2463000c 	addiu	v1,v1,12
80019ef0:	00071840 	sll	v1,a3,0x1
80019ef4:	00671821 	addu	v1,v1,a3
80019ef8:	3c078003 	lui	a3,0x8003
80019efc:	24e7ddf8 	addiu	a3,a3,-8712
80019f00:	00031880 	sll	v1,v1,0x2
80019f04:	00671821 	addu	v1,v1,a3
80019f08:	8c670000 	lw	a3,0(v1)
80019f0c:	3c038003 	lui	v1,0x8003
80019f10:	000738c0 	sll	a3,a3,0x3
80019f14:	2463e2a8 	addiu	v1,v1,-7512
80019f18:	00671821 	addu	v1,v1,a3
80019f1c:	8c690004 	lw	t1,4(v1)
80019f20:	00001825 	move	v1,zero
80019f24:	10000005 	b	80019f3c <lseek+0x84>
80019f28:	24080064 	li	t0,100
80019f2c:	00000000 	nop
80019f30:	24630001 	addiu	v1,v1,1
80019f34:	5068000b 	beql	v1,t0,80019f64 <lseek+0xac>
80019f38:	00002025 	move	a0,zero
80019f3c:	8c470000 	lw	a3,0(v0)
80019f40:	1487fffb 	bne	a0,a3,80019f30 <lseek+0x78>
80019f44:	2442000c 	addiu	v0,v0,12
80019f48:	00031040 	sll	v0,v1,0x1
80019f4c:	00431821 	addu	v1,v0,v1
80019f50:	3c028003 	lui	v0,0x8003
80019f54:	00031880 	sll	v1,v1,0x2
80019f58:	2442ddf8 	addiu	v0,v0,-8712
80019f5c:	00431021 	addu	v0,v0,v1
80019f60:	8c440004 	lw	a0,4(v0)
80019f64:	1120000c 	beqz	t1,80019f98 <lseek+0xe0>
80019f68:	3c028002 	lui	v0,0x8002
80019f6c:	8d390008 	lw	t9,8(t1)
80019f70:	13200005 	beqz	t9,80019f88 <lseek+0xd0>
80019f74:	24030058 	li	v1,88
80019f78:	03200008 	jr	t9
80019f7c:	00000000 	nop
80019f80:	1000fff8 	b	80019f64 <lseek+0xac>
80019f84:	00002025 	move	a0,zero
80019f88:	ac43d79c 	sw	v1,-10340(v0)
80019f8c:	03e00008 	jr	ra
80019f90:	2402ffff 	li	v0,-1
80019f94:	3c028002 	lui	v0,0x8002
80019f98:	24030016 	li	v1,22
80019f9c:	1000fffb 	b	80019f8c <lseek+0xd4>
80019fa0:	ac43d79c 	sw	v1,-10340(v0)
80019fa4:	00000000 	nop

80019fa8 <read>:
80019fa8:	00804025 	move	t0,a0
80019fac:	14800008 	bnez	a0,80019fd0 <read+0x28>
80019fb0:	00a05825 	move	t3,a1
80019fb4:	3c028003 	lui	v0,0x8003
80019fb8:	8c59dde8 	lw	t9,-8728(v0)
80019fbc:	1320003a 	beqz	t9,8001a0a8 <read+0x100>
80019fc0:	00c02825 	move	a1,a2
80019fc4:	03200008 	jr	t9
80019fc8:	01602025 	move	a0,t3
80019fcc:	00000000 	nop
80019fd0:	2482ffff 	addiu	v0,a0,-1
80019fd4:	2c420002 	sltiu	v0,v0,2
80019fd8:	54400034 	bnezl	v0,8001a0ac <read+0x104>
80019fdc:	3c028002 	lui	v0,0x8002
80019fe0:	18800038 	blez	a0,8001a0c4 <read+0x11c>
80019fe4:	00003825 	move	a3,zero
80019fe8:	3c028003 	lui	v0,0x8003
80019fec:	2442de00 	addiu	v0,v0,-8704
80019ff0:	00401825 	move	v1,v0
80019ff4:	10000005 	b	8001a00c <read+0x64>
80019ff8:	240a0064 	li	t2,100
80019ffc:	00000000 	nop
8001a000:	24e70001 	addiu	a3,a3,1
8001a004:	10ea0010 	beq	a3,t2,8001a048 <read+0xa0>
8001a008:	00002825 	move	a1,zero
8001a00c:	8c690000 	lw	t1,0(v1)
8001a010:	1509fffb 	bne	t0,t1,8001a000 <read+0x58>
8001a014:	2463000c 	addiu	v1,v1,12
8001a018:	00071840 	sll	v1,a3,0x1
8001a01c:	00671821 	addu	v1,v1,a3
8001a020:	3c048003 	lui	a0,0x8003
8001a024:	2484ddf8 	addiu	a0,a0,-8712
8001a028:	00031880 	sll	v1,v1,0x2
8001a02c:	00641821 	addu	v1,v1,a0
8001a030:	8c640000 	lw	a0,0(v1)
8001a034:	3c038003 	lui	v1,0x8003
8001a038:	000420c0 	sll	a0,a0,0x3
8001a03c:	2463e2a8 	addiu	v1,v1,-7512
8001a040:	00641821 	addu	v1,v1,a0
8001a044:	8c650004 	lw	a1,4(v1)
8001a048:	00001825 	move	v1,zero
8001a04c:	10000005 	b	8001a064 <read+0xbc>
8001a050:	24090064 	li	t1,100
8001a054:	00000000 	nop
8001a058:	24630001 	addiu	v1,v1,1
8001a05c:	1069000b 	beq	v1,t1,8001a08c <read+0xe4>
8001a060:	00002025 	move	a0,zero
8001a064:	8c470000 	lw	a3,0(v0)
8001a068:	1507fffb 	bne	t0,a3,8001a058 <read+0xb0>
8001a06c:	2442000c 	addiu	v0,v0,12
8001a070:	00031040 	sll	v0,v1,0x1
8001a074:	00431821 	addu	v1,v0,v1
8001a078:	3c028003 	lui	v0,0x8003
8001a07c:	00031880 	sll	v1,v1,0x2
8001a080:	2442ddf8 	addiu	v0,v0,-8712
8001a084:	00431021 	addu	v0,v0,v1
8001a088:	8c440004 	lw	a0,4(v0)
8001a08c:	10a0000e 	beqz	a1,8001a0c8 <read+0x120>
8001a090:	3c028002 	lui	v0,0x8002
8001a094:	8cb9000c 	lw	t9,12(a1)
8001a098:	13200008 	beqz	t9,8001a0bc <read+0x114>
8001a09c:	24030058 	li	v1,88
8001a0a0:	03200008 	jr	t9
8001a0a4:	01602825 	move	a1,t3
8001a0a8:	3c028002 	lui	v0,0x8002
8001a0ac:	24030009 	li	v1,9
8001a0b0:	ac43d79c 	sw	v1,-10340(v0)
8001a0b4:	03e00008 	jr	ra
8001a0b8:	2402ffff 	li	v0,-1
8001a0bc:	1000fffd 	b	8001a0b4 <read+0x10c>
8001a0c0:	ac43d79c 	sw	v1,-10340(v0)
8001a0c4:	3c028002 	lui	v0,0x8002
8001a0c8:	24030016 	li	v1,22
8001a0cc:	1000fff9 	b	8001a0b4 <read+0x10c>
8001a0d0:	ac43d79c 	sw	v1,-10340(v0)
8001a0d4:	00000000 	nop

8001a0d8 <sbrk>:
8001a0d8:	27bdffc8 	addiu	sp,sp,-56
8001a0dc:	ffb10028 	sd	s1,40(sp)
8001a0e0:	ffb00020 	sd	s0,32(sp)
8001a0e4:	ffbf0030 	sd	ra,48(sp)
8001a0e8:	0c005ea8 	jal	80017aa0 <disable_interrupts>
8001a0ec:	00808025 	move	s0,a0
8001a0f0:	8f9183a8 	lw	s1,-31832(gp)
8001a0f4:	1220001a 	beqz	s1,8001a160 <sbrk+0x88>
8001a0f8:	3c028003 	lui	v0,0x8003
8001a0fc:	8f8283a4 	lw	v0,-31836(gp)
8001a100:	02302021 	addu	a0,s1,s0
8001a104:	0044102b 	sltu	v0,v0,a0
8001a108:	14400009 	bnez	v0,8001a130 <sbrk+0x58>
8001a10c:	3c028002 	lui	v0,0x8002
8001a110:	0c005eb8 	jal	80017ae0 <enable_interrupts>
8001a114:	af8483a8 	sw	a0,-31832(gp)
8001a118:	dfbf0030 	ld	ra,48(sp)
8001a11c:	dfb00020 	ld	s0,32(sp)
8001a120:	02201025 	move	v0,s1
8001a124:	dfb10028 	ld	s1,40(sp)
8001a128:	03e00008 	jr	ra
8001a12c:	27bd0038 	addiu	sp,sp,56
8001a130:	2403000c 	li	v1,12
8001a134:	af9183a8 	sw	s1,-31832(gp)
8001a138:	0c005eb8 	jal	80017ae0 <enable_interrupts>
8001a13c:	ac43d79c 	sw	v1,-10340(v0)
8001a140:	dfbf0030 	ld	ra,48(sp)
8001a144:	2411ffff 	li	s1,-1
8001a148:	dfb00020 	ld	s0,32(sp)
8001a14c:	02201025 	move	v0,s1
8001a150:	dfb10028 	ld	s1,40(sp)
8001a154:	03e00008 	jr	ra
8001a158:	27bd0038 	addiu	sp,sp,56
8001a15c:	00000000 	nop
8001a160:	2442e2f8 	addiu	v0,v0,-7432
8001a164:	0c00645c 	jal	80019170 <get_memory_size>
8001a168:	af8283a8 	sw	v0,-31832(gp)
8001a16c:	3c037fff 	lui	v1,0x7fff
8001a170:	00431021 	addu	v0,v0,v1
8001a174:	8f9183a8 	lw	s1,-31832(gp)
8001a178:	1000ffe1 	b	8001a100 <sbrk+0x28>
8001a17c:	af8283a4 	sw	v0,-31836(gp)

8001a180 <times>:
8001a180:	3c028002 	lui	v0,0x8002
8001a184:	24030058 	li	v1,88
8001a188:	ac43d79c 	sw	v1,-10340(v0)
8001a18c:	03e00008 	jr	ra
8001a190:	2402ffff 	li	v0,-1
8001a194:	00000000 	nop

8001a198 <write>:
8001a198:	00803825 	move	a3,a0
8001a19c:	10800042 	beqz	a0,8001a2a8 <write+0x110>
8001a1a0:	00a05825 	move	t3,a1
8001a1a4:	24020001 	li	v0,1
8001a1a8:	1082003b 	beq	a0,v0,8001a298 <write+0x100>
8001a1ac:	24020002 	li	v0,2
8001a1b0:	50820033 	beql	a0,v0,8001a280 <write+0xe8>
8001a1b4:	3c028003 	lui	v0,0x8003
8001a1b8:	18800042 	blez	a0,8001a2c4 <write+0x12c>
8001a1bc:	00004025 	move	t0,zero
8001a1c0:	3c028003 	lui	v0,0x8003
8001a1c4:	2442de00 	addiu	v0,v0,-8704
8001a1c8:	00401825 	move	v1,v0
8001a1cc:	10000005 	b	8001a1e4 <write+0x4c>
8001a1d0:	240a0064 	li	t2,100
8001a1d4:	00000000 	nop
8001a1d8:	25080001 	addiu	t0,t0,1
8001a1dc:	110a0010 	beq	t0,t2,8001a220 <write+0x88>
8001a1e0:	00002825 	move	a1,zero
8001a1e4:	8c690000 	lw	t1,0(v1)
8001a1e8:	14e9fffb 	bne	a3,t1,8001a1d8 <write+0x40>
8001a1ec:	2463000c 	addiu	v1,v1,12
8001a1f0:	00081840 	sll	v1,t0,0x1
8001a1f4:	00681821 	addu	v1,v1,t0
8001a1f8:	3c048003 	lui	a0,0x8003
8001a1fc:	2484ddf8 	addiu	a0,a0,-8712
8001a200:	00031880 	sll	v1,v1,0x2
8001a204:	00641821 	addu	v1,v1,a0
8001a208:	8c640000 	lw	a0,0(v1)
8001a20c:	3c038003 	lui	v1,0x8003
8001a210:	000420c0 	sll	a0,a0,0x3
8001a214:	2463e2a8 	addiu	v1,v1,-7512
8001a218:	00641821 	addu	v1,v1,a0
8001a21c:	8c650004 	lw	a1,4(v1)
8001a220:	00001825 	move	v1,zero
8001a224:	10000005 	b	8001a23c <write+0xa4>
8001a228:	24090064 	li	t1,100
8001a22c:	00000000 	nop
8001a230:	24630001 	addiu	v1,v1,1
8001a234:	1069000b 	beq	v1,t1,8001a264 <write+0xcc>
8001a238:	00002025 	move	a0,zero
8001a23c:	8c480000 	lw	t0,0(v0)
8001a240:	14e8fffb 	bne	a3,t0,8001a230 <write+0x98>
8001a244:	2442000c 	addiu	v0,v0,12
8001a248:	00031040 	sll	v0,v1,0x1
8001a24c:	00431821 	addu	v1,v0,v1
8001a250:	3c028003 	lui	v0,0x8003
8001a254:	00031880 	sll	v1,v1,0x2
8001a258:	2442ddf8 	addiu	v0,v0,-8712
8001a25c:	00431021 	addu	v0,v0,v1
8001a260:	8c440004 	lw	a0,4(v0)
8001a264:	10a00018 	beqz	a1,8001a2c8 <write+0x130>
8001a268:	3c028002 	lui	v0,0x8002
8001a26c:	8cb90010 	lw	t9,16(a1)
8001a270:	13200012 	beqz	t9,8001a2bc <write+0x124>
8001a274:	24030058 	li	v1,88
8001a278:	03200008 	jr	t9
8001a27c:	01602825 	move	a1,t3
8001a280:	8c59ddf0 	lw	t9,-8720(v0)
8001a284:	13200009 	beqz	t9,8001a2ac <write+0x114>
8001a288:	3c028002 	lui	v0,0x8002
8001a28c:	00c02825 	move	a1,a2
8001a290:	03200008 	jr	t9
8001a294:	01602025 	move	a0,t3
8001a298:	3c028003 	lui	v0,0x8003
8001a29c:	8c59ddec 	lw	t9,-8724(v0)
8001a2a0:	1720fffb 	bnez	t9,8001a290 <write+0xf8>
8001a2a4:	00c02825 	move	a1,a2
8001a2a8:	3c028002 	lui	v0,0x8002
8001a2ac:	24030009 	li	v1,9
8001a2b0:	ac43d79c 	sw	v1,-10340(v0)
8001a2b4:	03e00008 	jr	ra
8001a2b8:	2402ffff 	li	v0,-1
8001a2bc:	1000fffd 	b	8001a2b4 <write+0x11c>
8001a2c0:	ac43d79c 	sw	v1,-10340(v0)
8001a2c4:	3c028002 	lui	v0,0x8002
8001a2c8:	24030016 	li	v1,22
8001a2cc:	1000fff9 	b	8001a2b4 <write+0x11c>
8001a2d0:	ac43d79c 	sw	v1,-10340(v0)
8001a2d4:	00000000 	nop

8001a2d8 <hook_stdio_calls>:
8001a2d8:	10800011 	beqz	a0,8001a320 <hook_stdio_calls+0x48>
8001a2dc:	00000000 	nop
8001a2e0:	8c820000 	lw	v0,0(a0)
8001a2e4:	10400002 	beqz	v0,8001a2f0 <hook_stdio_calls+0x18>
8001a2e8:	3c038003 	lui	v1,0x8003
8001a2ec:	ac62dde8 	sw	v0,-8728(v1)
8001a2f0:	8c820004 	lw	v0,4(a0)
8001a2f4:	10400002 	beqz	v0,8001a300 <hook_stdio_calls+0x28>
8001a2f8:	3c038003 	lui	v1,0x8003
8001a2fc:	ac62ddec 	sw	v0,-8724(v1)
8001a300:	8c820008 	lw	v0,8(a0)
8001a304:	10400004 	beqz	v0,8001a318 <hook_stdio_calls+0x40>
8001a308:	3c038003 	lui	v1,0x8003
8001a30c:	ac62ddf0 	sw	v0,-8720(v1)
8001a310:	03e00008 	jr	ra
8001a314:	00001025 	move	v0,zero
8001a318:	03e00008 	jr	ra
8001a31c:	00001025 	move	v0,zero
8001a320:	03e00008 	jr	ra
8001a324:	2402ffff 	li	v0,-1

8001a328 <hook_time_call>:
8001a328:	10800004 	beqz	a0,8001a33c <hook_time_call+0x14>
8001a32c:	00000000 	nop
8001a330:	af8483ac 	sw	a0,-31828(gp)
8001a334:	03e00008 	jr	ra
8001a338:	00001025 	move	v0,zero
8001a33c:	03e00008 	jr	ra
8001a340:	2402ffff 	li	v0,-1
8001a344:	00000000 	nop

8001a348 <__assert_func>:
8001a348:	8f8283b0 	lw	v0,-31824(gp)
8001a34c:	27bdffd8 	addiu	sp,sp,-40
8001a350:	10400003 	beqz	v0,8001a360 <__assert_func+0x18>
8001a354:	ffbf0020 	sd	ra,32(sp)
8001a358:	0040f809 	jalr	v0
8001a35c:	00000000 	nop
8001a360:	0c000a2f 	jal	800028bc <abort>
8001a364:	00000000 	nop
	...
