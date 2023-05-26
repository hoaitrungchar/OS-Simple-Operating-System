//#ifdef MM_PAGING
/*
 * PAGING based Memory Management
 * Virtual memory module mm/mm-vm.c
 */

#include "string.h"
#include "mm.h"
#include <stdlib.h>
#include <stdio.h>



/*------------------Bat dau phan lam----------------*/

/*------------------Tam thoi la FIFO----------------*/



uint32_t* FIFO_find_vt_page_for_swap(){
    #ifdef TDBG
        printf("FIFO_find_vt_page_for_swap\n");
    #endif
    pthread_mutex_lock(&FIFO_lock);
    if(FIFO_head==NULL) return;
    struct FIFO_struct * temp=FIFO_tail;
    uint32_t* pte_ret;
    pte_ret=temp->pte;
    if(FIFO_head==FIFO_tail){
        FIFO_head=FIFO_tail=NULL;
    }
    else{
        FIFO_tail=FIFO_tail->FIFO_pre;
        FIFO_tail->FIFO_next=NULL;
    }
    free(temp);
    pthread_mutex_unlock(&FIFO_lock);
    return pte_ret;
}

void FIFO_add_page(uint32_t *pte_add){
  #ifdef TDBG
      printf("FIFO_add_page\n");
  #endif
  pthread_mutex_lock(&FIFO_lock);
  struct FIFO_struct * temp= malloc(sizeof(struct FIFO_struct)); 
  temp->pte=pte_add;
  if(FIFO_head==NULL){
      FIFO_head=FIFO_tail=temp;
      temp->FIFO_next=NULL;
      temp->FIFO_pre=NULL;
  }
  else{
      temp->FIFO_pre=NULL;
      temp->FIFO_next=FIFO_head;
      FIFO_head->FIFO_pre=temp;
      FIFO_head=temp;
  }
  pthread_mutex_unlock(&FIFO_lock);
}

/*------------------Ket thuc phan lam --------------*/

/*enlist_vm_freerg_list - add new rg to freerg_list
 *@mm: memory region
 *@rg_elmt: new region
 *
 */
int enlist_vm_freerg_list(struct mm_struct *mm, struct vm_rg_struct rg_elmt)
{
  #ifdef TDBG
        printf("enlist_vm_freerg_list\n");
  #endif
  /* ------------------Bat dau phan lam----------------------- */
  /*struct vm_rg_struct *rg_node = mm->mmap->vm_freerg_list;
  if (rg_elmt.rg_start >= rg_elmt.rg_end)
    return -1;
  if (rg_node != NULL)
    rg_elmt.rg_next = rg_node;
  mm->mmap->vm_freerg_list = &rg_elmt;*/
  struct vm_rg_struct *rg_node=malloc(sizeof(struct vm_rg_struct));
  if (rg_elmt.rg_start >= rg_elmt.rg_end)
    return -1;
  //add node to freerg_list
  rg_node->rg_start=rg_elmt.rg_start;
  rg_node->rg_end=rg_elmt.rg_end;
  rg_node->rg_next=mm->mmap->vm_freerg_list;
  mm->mmap->vm_freerg_list=rg_node;
  /* ------------------Ket thuc phan lam----------------------- */
  return 0;
}

/*get_vma_by_num - get vm area by numID
 *@mm: memory region
 *@vmaid: ID vm area to alloc memory region
 *
 */
struct vm_area_struct *get_vma_by_num(struct mm_struct *mm, int vmaid)
{
  #ifdef TDBG
        printf("get_vma_by_num\n");
  #endif
  struct vm_area_struct *pvma= mm->mmap;

  if(mm->mmap == NULL)
    return NULL;

  int vmait = 0;
  
  while (vmait < vmaid)
  {
    if(pvma == NULL)
	  return NULL;
    vmait++;
    pvma = pvma->vm_next;
  }

  return pvma;
}

/*get_symrg_byid - get mem region by region ID
 *@mm: memory region
 *@rgid: region ID act as symbol index of variable
 *
 */
struct vm_rg_struct* get_symrg_byid(struct mm_struct *mm, int rgid)
{
  #ifdef TDBG
        printf("get_symrg_byid\n");
  #endif
  if(rgid < 0 || rgid > PAGING_MAX_SYMTBL_SZ)
    return NULL;

  return &mm->symrgtbl[rgid];
}

/*__alloc - allocate a region memory
 *@caller: caller
 *@vmaid: ID vm area to alloc memory region
 *@rgid: memory region ID (used to identify variable in symbole table)
 *@size: allocated size 
 *@alloc_addr: address of allocated memory region
 *
 */
int __alloc(struct pcb_t *caller, int vmaid, int rgid, int size, int *alloc_addr)
{
  #ifdef TDBG
        printf("__alloc\n");
  #endif
  /*Allocate at the toproof */
  struct vm_rg_struct rgnode;
  if(rgid < 0 || rgid > PAGING_MAX_SYMTBL_SZ)
  {
    printf("Process %d alloc error: Invalid region\n",caller->pid);
    return -1;
  }  
  if (get_free_vmrg_area(caller, vmaid, size, &rgnode) == 0)
  {
    caller->mm->symrgtbl[rgid].rg_start = rgnode.rg_start;
    caller->mm->symrgtbl[rgid].rg_end = rgnode.rg_end;

    *alloc_addr = rgnode.rg_start;
    
    #ifdef RAM_STATUS_DUMP
  	printf("-------------------------\n");
  	printf("Process %d ALLOC CALL | SIZE = %d\n",caller->pid ,size);
  	printf("-------------------------\n");
  	for (int it = 0; it < PAGING_MAX_SYMTBL_SZ; it++)
  	{
  		if (caller->mm->symrgtbl[it].rg_start == 0 && caller->mm->symrgtbl[it].rg_end == 0)
  			continue;
  		printf("Region id %d : start = %lu, end = %lu\n", it, caller->mm->symrgtbl[it].rg_start, caller->mm->symrgtbl[it].rg_end); 
  	}
   #endif

    return 0;
  }

  /* TODO get_free_vmrg_area FAILED handle the region management (Fig.6)*/

  /*Attempt to increate limit to get space */
  struct vm_area_struct *cur_vma = get_vma_by_num(caller->mm, vmaid);
  int inc_sz = PAGING_PAGE_ALIGNSZ(size);
  //int inc_limit_ret
  int old_sbrk ;

  old_sbrk = cur_vma->sbrk;

  /* TODO INCREASE THE LIMIT
   * inc_vma_limit(caller, vmaid, inc_sz)
   */
  inc_vma_limit(caller, vmaid, inc_sz);
  /* ------------------Bat dau phan lam----------------------- */
  cur_vma->sbrk = old_sbrk + size;
  /* ------------------Ket thuc phan lam---------------------- */
  /*Successful increase limit */
  caller->mm->symrgtbl[rgid].rg_start = old_sbrk;
  caller->mm->symrgtbl[rgid].rg_end = old_sbrk + size;

  *alloc_addr = old_sbrk;
  
  #ifdef RAM_STATUS_DUMP
  	printf("-------------------------\n");
  	printf("Process %d ALLOC CALL | SIZE = %d\n",caller->pid ,size);
  	printf("-------------------------\n");
  	for (int it = 0; it < PAGING_MAX_SYMTBL_SZ; it++)
  	{
  		if (caller->mm->symrgtbl[it].rg_start == 0 && caller->mm->symrgtbl[it].rg_end == 0)
  			continue;
  		printf("Region id %d : start = %lu, end = %lu\n", it, caller->mm->symrgtbl[it].rg_start, caller->mm->symrgtbl[it].rg_end); 
  	}
    RAM_dump(caller->mram);
  #endif

  return 0;
}

/*__free - remove a region memory
 *@caller: caller
 *@vmaid: ID vm area to alloc memory region
 *@rgid: memory region ID (used to identify variable in symbole table)
 *@size: allocated size 
 *
 */
int __free(struct pcb_t *caller, int vmaid, int rgid)
{
  #ifdef TDBG
        printf("__free\n");
  #endif
  struct vm_rg_struct *rgnode;

  if(rgid < 0 || rgid > PAGING_MAX_SYMTBL_SZ){
    return -1;
    printf("Process %d free error: Invalid region\n",caller->pid);
  }
    

  /* TODO: Manage the collect freed region to freerg_list */
  /* ------------------Bat dau phan lam----------------------- */

  rgnode= get_symrg_byid(caller->mm, rgid);
  struct vm_rg_struct* rgnode_temp=malloc(sizeof(struct vm_rg_struct));
  
  #ifdef RAM_STATUS_DUMP
  	printf("-------------------------\n");
  	printf("Process %d FREE CALL | Region id %d : [%lu,%lu]\n",caller->pid, rgid, rgnode->rg_start, rgnode->rg_end);
  	for (int it = 0; it < PAGING_MAX_SYMTBL_SZ; it++)
  	{
  		if (caller->mm->symrgtbl[it].rg_start == 0 && caller->mm->symrgtbl[it].rg_end == 0)
  			continue;
  		else
  			printf("Region id %d : start = %lu, end = %lu\n", it, caller->mm->symrgtbl[it].rg_start, caller->mm->symrgtbl[it].rg_end); 
  	}
  
  #endif

  //Create new node for region
  rgnode_temp->rg_start=rgnode->rg_start;
  rgnode_temp->rg_end=rgnode->rg_end;
  rgnode->rg_start=rgnode->rg_end=0;
  

  /* ------------------Ket thuc phan lam----------------------- */
  
  /*enlist the obsoleted memory region */
  enlist_vm_freerg_list(caller->mm, *rgnode_temp);
 
  return 0;
}

/*pgalloc - PAGING-based allocate a region memory
 *@proc:  Process executing the instruction
 *@size: allocated size 
 *@reg_index: memory region ID (used to identify variable in symbole table)
 */
int pgalloc(struct pcb_t *proc, uint32_t size, uint32_t reg_index)
{
  #ifdef TDBG
        printf("pgalloc\n");
  #endif
  int addr;

  /* By default using vmaid = 0 */
  return __alloc(proc, 0, reg_index, size, &addr);
}

/*pgfree - PAGING-based free a region memory
 *@proc: Process executing the instruction
 *@size: allocated size 
 *@reg_index: memory region ID (used to identify variable in symbole table)
 */

int pgfree_data(struct pcb_t *proc, uint32_t reg_index)
{
  #ifdef TDBG
        printf("pgfree_data\n");
  #endif
  return __free(proc, 0, reg_index);
}

/*pg_getpage - get the page in ram
 *@mm: memory region
 *@pagenum: PGN
 *@framenum: return FPN
 *@caller: caller
 *
 */
int pg_getpage(struct mm_struct *mm, int pgn, int *fpn, struct pcb_t *caller)
{
  #ifdef TDBG
        printf("pg_getpage\n");
  #endif
  pthread_mutex_lock(&MEM_in_use);
  uint32_t pte = mm->pgd[pgn];
 
  if (!PAGING_PAGE_PRESENT(pte))
  { /* Page is not online, make it actively living */
    //int vicpgn, swpfpn; 
    //uint32_t vicpte;
    //uint32_t vicpte
    //int vicfpn;
    

    //int tgtfpn = PAGING_SWP(pte);//the target frame storing our variable

    /* TODO: Play with your paging theory here */
    /* ------------------Bat dau phan lam----------------------- */
    int fpn_temp=-1;
    if(MEMPHY_get_freefp(caller->mram, &fpn_temp)==0){
      //Tao node moi
      struct framephy_struct *newnode=malloc(sizeof(struct framephy_struct));
      newnode->fpn=fpn_temp;
      newnode->owner=caller->mm;

      //lay gia tri tgtfpn
      int tgtfpn =GETVAL(pte,GENMASK(10,0),5);

      //Copy frame from SWAP to RAM
      __swap_cp_page(caller->active_mswp, tgtfpn,caller->mram,fpn_temp);
      //Cap nhat gia tri pte
      pte_set_fpn(&pte,fpn_temp);
      
      //Them page moi vao FIFO
      FIFO_add_page(&pte);
    }
    else{
      int tgtfpn =GETVAL(pte,GENMASK(10,0),5);

      int vicfpn, swpfpn; uint32_t* vicpte;
      /* Find pointer to pte of victim frame*/
      vicpte=FIFO_find_vt_page_for_swap(caller->mm, &vicfpn);
      
      /* Variable for value of pte*/
      uint32_t vicpte_temp=*vicpte;
      /*Get victim frame*/
      vicfpn=GETVAL(vicpte_temp,PAGING_PTE_FPN_MASK, PAGING_PTE_FPN_LOBIT); //8191 in decimal is 0->12 bit =1 in binary (total 13bit)
      /* Get free frame in MEMSWP */
      if(MEMPHY_get_freefp(caller->active_mswp, &swpfpn)<0)
      {
        printf("Out of SWAP");
        return -3000;
      }

      /* Copy victim frame to swap */
      __swap_cp_page(caller->mram, vicfpn,caller->active_mswp, swpfpn);

      /* Copy target frame from swap to mem */
      __swap_cp_page(caller->active_mswp, tgtfpn,caller->mram,vicfpn);

      //Cap nhat cho pte tro den page vua bi thay rang du lieu do da chuyen vao SWAP
      pte_set_swap(&vicpte_temp,0,swpfpn);

      //Cap nhat lai gia tri cua pte vua bi SWAP qua con tro
      *vicpte=vicpte_temp;

      //Cap nhat gia tri frame number moi (trong Ram) cho page entry (bao rang pte da co frame number moi)
      pte_set_fpn(&pte,vicfpn);

      //Them page moi vao FIFO
      FIFO_add_page(&pte);

      //Put frame trong trong swap vao free frame list
      MEMPHY_put_freefp(caller->active_mswp,tgtfpn);
    }

    /*--------------------Ket thuc phan lam----------------------*/

    /*------------Code cua thay ---------------*/
    /* Do swap frame from MEMRAM to MEMSWP and vice versa*/
    /* Copy victim frame to swap */
    //__swap_cp_page();
    /* Copy target frame from swap to mem */
    //__swap_cp_page();

    /* Update page table */
    //pte_set_swap() &mm->pgd;

    /* Update its online status of the target page */
    //pte_set_fpn() & mm->pgd[pgn];
    //pte_set_fpn(&pte, tgtfpn);
    //enlist_pgn_node(&caller->mm->fifo_pgn,pgn);
    /*------------Het code thay ---------------*/


  }
  /*------------Code cua thay ---------------*/
  //fpn = PAGING_FPN(pte);
  /*------------Het code thay ---------------*/

  /* ------------------Bat dau phan lam----------------------- */
  *fpn=GETVAL(pte,PAGING_PTE_FPN_MASK, PAGING_PTE_FPN_LOBIT);
  pthread_mutex_unlock(&MEM_in_use);
  /* ------------------Ket thuc phan lam---------------------- */
  return 0;
}

/*pg_getval - read value at given offset
 *@mm: memory region
 *@addr: virtual address to acess 
 *@value: value
 *
 */
int pg_getval(struct mm_struct *mm, int addr, BYTE *data, struct pcb_t *caller)
{
  #ifdef TDBG
        printf("pg_getval\n");
  #endif
  int pgn = PAGING_PGN(addr);
  int off = PAGING_OFFST(addr);
  int fpn;

  /* Get the page to MEMRAM, swap from MEMSWAP if needed */
  if(pg_getpage(mm, pgn, &fpn, caller) != 0) 
    return -1; /* invalid page access */

  int phyaddr = (fpn << PAGING_ADDR_FPN_LOBIT) + off;

  MEMPHY_read(caller->mram,phyaddr, data);

  return 0;
}

/*pg_setval - write value to given offset
 *@mm: memory region
 *@addr: virtual address to acess 
 *@value: value
 *
 */
int pg_setval(struct mm_struct *mm, int addr, BYTE value, struct pcb_t *caller)
{
  #ifdef TDBG
        printf("pg_setval\n");
  #endif
  int pgn = PAGING_PGN(addr);
  int off = PAGING_OFFST(addr);
  int fpn;

  /* Get the page to MEMRAM, swap from MEMSWAP if needed */
  if(pg_getpage(mm, pgn, &fpn, caller) != 0) 
    return -1; /* invalid page access */

  int phyaddr = (fpn << PAGING_ADDR_FPN_LOBIT) + off;

  MEMPHY_write(caller->mram,phyaddr, value);

   return 0;
}

/*__read - read value in region memory
 *@caller: caller
 *@vmaid: ID vm area to alloc memory region
 *@offset: offset to acess in memory region 
 *@rgid: memory region ID (used to identify variable in symbole table)
 *@size: allocated size 
 *
 */
int __read(struct pcb_t *caller, int vmaid, int rgid, int offset, BYTE *data)
{
  #ifdef TDBG
        printf("__read\n");
  #endif
  struct vm_rg_struct *currg = get_symrg_byid(caller->mm, rgid);

  struct vm_area_struct *cur_vma = get_vma_by_num(caller->mm, vmaid);

  if(currg == NULL || cur_vma == NULL) /* Invalid memory identify */
  {
    printf("Process %d read error: Invalid region\n",caller->pid);
    return -1;
  }
	

  /*------------------Bat dau phan lam----------------*/
  if(currg->rg_start>=currg->rg_end){
    printf("Process %d read error: Region not found (freed or unintialized)\n",caller->pid);
    return -1;
  }
  else if(currg->rg_start+offset>=currg->rg_end||offset<0){
    printf("Process %d read error: Invalid offset when read!\n",caller->pid);
    return -1;
  }
  else{
    pg_getval(caller->mm, currg->rg_start + offset, data, caller);
  }
  /*------------------Ket thuc phan lam---------------*/


  return 0;
}


/*pgread - PAGING-based read a region memory */
int pgread(
		struct pcb_t * proc, // Process executing the instruction
		uint32_t source, // Index of source register
		uint32_t offset, // Source address = [source] + [offset]
		uint32_t destination) 
{
  #ifdef TDBG
        printf("pgread\n");
  #endif
  BYTE data;
  int val = __read(proc, 0, source, offset, &data);

  destination = (uint32_t) data;
#ifdef IODUMP
if(val==0)
  printf("Process %d read region=%d offset=%d value=%d\n", proc->pid,source, offset, data);
else
  printf("Process %d error when read region=%d offset=%d \n", proc->pid,source, offset);
#ifdef PAGETBL_DUMP
  print_pgtbl(proc, 0, -1); //print max TBL
#endif
  MEMPHY_dump(proc->mram);
#endif

  return val;
}

/*__write - write a region memory
 *@caller: caller
 *@vmaid: ID vm area to alloc memory region
 *@offset: offset to acess in memory region 
 *@rgid: memory region ID (used to identify variable in symbole table)
 *@size: allocated size 
 *
 */
int __write(struct pcb_t *caller, int vmaid, int rgid, int offset, BYTE value)
{
  #ifdef TDBG
        printf("__write\n");
  #endif
  struct vm_rg_struct *currg = get_symrg_byid(caller->mm, rgid);

  struct vm_area_struct *cur_vma = get_vma_by_num(caller->mm, vmaid);
  
  if(currg == NULL || cur_vma == NULL) /* Invalid memory identify */
  {
    printf("Process %d write error: Invalid region\n",caller->pid);
    return -1;
  }
	  
    /*------------------Bat dau phan lam----------------*/
  if(currg->rg_start>=currg->rg_end){
    printf("Process %d write error: Region not found (freed or unintialized)\n", caller->pid);
  }
  else if(currg->rg_start+offset>=currg->rg_end||offset<0){
    printf("Process %d write error: Invalid offset when write!\n", caller->pid);
    return -1;
  }
  else{
    pg_setval(caller->mm, currg->rg_start + offset, value, caller);
  }
  /*------------------Ket thuc phan lam---------------*/

  return 0;
}

/*pgwrite - PAGING-based write a region memory */
int pgwrite(
		struct pcb_t * proc, // Process executing the instruction
		BYTE data, // Data to be wrttien into memory
		uint32_t destination, // Index of destination register
		uint32_t offset)
{
  #ifdef TDBG
        printf("pgwrite\n");
  #endif
#ifdef IODUMP
  printf("Process %d write region=%d offset=%d value=%d\n",proc->pid ,destination, offset, data);
#endif
  int x=__write(proc, 0, destination, offset, data);
#ifdef IODUMP
#ifdef PAGETBL_DUMP
  print_pgtbl(proc, 0, -1); //print max TBL
#endif
  MEMPHY_dump(proc->mram);
#endif

  return x;
}


/*free_pcb_memphy - collect all memphy of pcb
 *@caller: caller
 *@vmaid: ID vm area to alloc memory region
 *@incpgnum: number of page
 */
int free_pcb_memph(struct pcb_t *caller)
{
  #ifdef TDBG
        printf("free_pcb_memph\n");
  #endif
  int pagenum, fpn;
  uint32_t pte;
  for(pagenum = 0; pagenum < PAGING_MAX_PGN; pagenum++)
  {
    pte= caller->mm->pgd[pagenum];

    if (!PAGING_PAGE_PRESENT(pte))
    {
      fpn = PAGING_FPN(pte);
      MEMPHY_put_freefp(caller->mram, fpn);
    } else {
      fpn = PAGING_SWP(pte);
      MEMPHY_put_freefp(caller->active_mswp, fpn);    
    }
  }

  return 0;
}

/*get_vm_area_node - get vm area for a number of pages
 *@caller: caller
 *@vmaid: ID vm area to alloc memory region
 *@incpgnum: number of page
 *@vmastart: vma end
 *@vmaend: vma end
 *
 */
struct vm_rg_struct* get_vm_area_node_at_brk(struct pcb_t *caller, int vmaid, int size, int alignedsz)
{
  #ifdef TDBG
        printf("get_vm_area_node_at_brk\n");
  #endif
  struct vm_rg_struct * newrg;
  struct vm_area_struct *cur_vma = get_vma_by_num(caller->mm, vmaid);

  newrg = malloc(sizeof(struct vm_rg_struct));

  newrg->rg_start = cur_vma->sbrk;
  newrg->rg_end = newrg->rg_start + size;

  return newrg;
}

/*validate_overlap_vm_area
 *@caller: caller
 *@vmaid: ID vm area to alloc memory region
 *@vmastart: vma end
 *@vmaend: vma end
 *
 */
int validate_overlap_vm_area(struct pcb_t *caller, int vmaid, int vmastart, int vmaend)
{
  //struct vm_area_struct *vma = caller->mm->mmap;
  #ifdef TDBG
          printf("validate_overlap_vm_area\n");
  #endif
  /* TODO validate the planned memory area is not overlapped */
  struct vm_area_struct *vmit = caller->mm->mmap;
  while(vmit!=NULL) {
    if ((vmastart < vmit->vm_start && vmaend > vmit->vm_start)){
      printf("vm area overlap\n");
      return -1;
    }
    vmit=vmit->vm_next;
  }
  return 0;
}

/*inc_vma_limit - increase vm area limits to reserve space for new variable
 *@caller: caller
 *@vmaid: ID vm area to alloc memory region
 *@inc_sz: increment size 
 *
 */
int inc_vma_limit(struct pcb_t *caller, int vmaid, int inc_sz)
{
  #ifdef TDBG
          printf("inc_vma_limit\n");
  #endif
  struct vm_rg_struct * newrg = malloc(sizeof(struct vm_rg_struct));
  int inc_amt = PAGING_PAGE_ALIGNSZ(inc_sz);
  int incnumpage =  inc_amt / PAGING_PAGESZ;
  struct vm_rg_struct *area = get_vm_area_node_at_brk(caller, vmaid, inc_sz, inc_amt);
  struct vm_area_struct *cur_vma = get_vma_by_num(caller->mm, vmaid);

  int old_end = cur_vma->vm_end;

  /*Validate overlap of obtained region */
  if (validate_overlap_vm_area(caller, vmaid, area->rg_start, area->rg_end) < 0)
    return -1; /*Overlap and failed allocation */

  /* The obtained vm area (only) 
   * now will be alloc real ram region */
  cur_vma->vm_end += inc_sz;
  if (vm_map_ram(caller, area->rg_start, area->rg_end, 
                    old_end, incnumpage , newrg) < 0)
    return -1; /* Map the memory to MEMRAM */

  return 0;

}

/*find_victim_page - find victim page
 *@caller: caller
 *@pgn: return page number
 *
 */
int find_victim_page(struct mm_struct *mm, int *retpgn) 
{
  #ifdef TDBG
          printf("find_victim_page\n");
  #endif
  struct pgn_t *pg = mm->fifo_pgn;

  /* TODO: Implement the theorical mechanism to find the victim page */

  free(pg);

  return 0;
}


/*get_free_vmrg_area - get a free vm region
 *@caller: caller
 *@vmaid: ID vm area to alloc memory region
 *@size: allocated size 
 *
 */
int get_free_vmrg_area(struct pcb_t *caller, int vmaid, int size, struct vm_rg_struct *newrg)
{
  #ifdef TDBG
          printf("get_free_vmrg_area\n");
  #endif
  struct vm_area_struct *cur_vma = get_vma_by_num(caller->mm, vmaid);

  struct vm_rg_struct *rgit = cur_vma->vm_freerg_list;

  if (rgit == NULL)
    return -1;

  /* Probe unintialized newrg */
  newrg->rg_start = newrg->rg_end = -1;

  /* Traverse on list of free vm region to find a fit space */
  while (rgit != NULL)
  {
    if (rgit->rg_start + size <= rgit->rg_end)
    { /* Current region has enough space */
      newrg->rg_start = rgit->rg_start;
      newrg->rg_end = rgit->rg_start + size;

      /* Update left space in chosen region */
      if (rgit->rg_start + size < rgit->rg_end)
      {
        rgit->rg_start = rgit->rg_start + size;
      }
      else
      { /*Use up all space, remove current node */
        /*Clone next rg node */
        struct vm_rg_struct *nextrg = rgit->rg_next;

        /*Cloning */
        if (nextrg != NULL)
        {
          rgit->rg_start = nextrg->rg_start;
          rgit->rg_end = nextrg->rg_end;

          rgit->rg_next = nextrg->rg_next;

          free(nextrg);
        }
        else
        { /*End of free list */
          rgit->rg_start = rgit->rg_end;	//dummy, size 0 region
          rgit->rg_next = NULL;
        }
      }
      //break;
    }
    else
    {
      rgit = rgit->rg_next;	// Traverse next rg
    }
  }

 if(newrg->rg_start == -1) // new region not found
   return -1;

 return 0;
}

//#endif
